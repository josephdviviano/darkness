/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2024-2026 darkness contributors
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *****************************************************************************/

#ifndef __PROBE_FILE_H
#define __PROBE_FILE_H

/// @file ProbeFile.h
/// Probe file format with integrity checking for baked acoustic probe data.
///
/// === Version 5 (current) — v4 + coverage-based bake-range derivation ===
///
/// File layout (all fields little-endian):
///   [0..3]   magic       "DKPr" (0x72504B44)
///   [4..7]   version     format version (currently 5; v4 still ACCEPTED —
///            see the v4 note below)
///   [8..11]  batchCount  number of probe batches that follow
///   [12..15] totalProbes total probe count across all batches (sanity)
///   [16..19] bakedPathingVisRangeFt   float — the IPLPathBakeParams::visRange
///            (single-edge distance cap, engine feet) the pathing section was
///            baked with. 0 when the file carries no pathing batch. The
///            runtime clamps per-voice `inputs.visRange` to this value —
///            longer single edges do not exist in the baked graph, so
///            requesting more silently buys nothing (multi-hop pathRange
///            covers long routes instead).
///   [20..23] bakedPathingNumSamples   uint32 — IPLPathBakeParams::numSamples
///            the pathing section was baked with. 0 when no pathing batch.
///            Runtime `IPLSimulationSettings::numVisSamples` MUST equal this
///            or bake-accepted edges are re-rejected at runtime (pathing
///            collapses to the 0.1f sentinel). Loaders compare against the
///            active profile constant (SteamAudioPathing.h
///            kPathingVisSamplesShip/Dev) and trigger a loud pathing-only
///            re-bake on mismatch instead of running with a torn edge set.
///   [24..27] bakedReflectionRays      uint32 — the reflection IR bake's ray
///            count (IPLReflectionsBakeParams::numRays). Together with the
///            dedup radius below this identifies the REFLECTION half of the
///            bake-quality profile (--bake-quality dev = rays 2048 + 18 ft
///            dedup; ship = the yaml values). 0 = unknown (never written by
///            current code — every file carries a reflection section).
///   [28..31] bakedProbeDedupRadiusFt  float — the reflection batch's global
///            dedup radius (engine feet) at bake time; the probe-DENSITY
///            half of the profile.
///   [32..35] bakedPathingDensity      uint32 — PathingProbeDensity tier the
///            pathing section's probe LAYOUT was emitted at
///            (audio.pathing_probes.density: 1=baseline, 2=bends; see
///            ProbeManager.h). 0 when no pathing batch. Mismatch against the
///            active config = same policy as bakedPathingNumSamples: loud
///            message + automatic pathing-only re-bake — a baseline cache
///            consumed at bends density silently lacks the per-portal bend
///            pairs the runtime layout expects (and vice versa).
///   [36..39] bakedPathingCoverageFt   float (v5) — the coverage governing
///            value the visRange cap was derived from: max over rooms of
///            (max portal → nearest same-room probe distance, POST hub
///            fill). Diagnostic record of the derivation input; 0 when no
///            pathing batch (or a v4 file).
///   [40..43] bakedPathingRCovFt       float (v5) — the coverage radius
///            (kPathingCoverageRadiusFt, SteamAudioPathing.h) the hub-fill
///            pass targeted at bake time. 0 when no pathing batch (or a v4
///            file). Mismatch against the active constant = same policy as
///            bakedPathingNumSamples: loud message + automatic pathing-only
///            re-bake — the fill probes and the coverage-derived visRange
///            both depend on it.
///
/// === Version 4 (legacy, still ACCEPTED) — v3 + layout-density record ===
///
/// 36-byte header ending at bakedPathingDensity; no coverage fields. v4
/// files load normally with bakedPathingCoverageFt/bakedPathingRCovFt = 0.
/// Unlike v1-v3 this is NOT rejected: the batch payloads and every other
/// header field are bit-identical to v5, so the reflection IR blob (the
/// expensive half) is fully reusable. The zeroed bakedPathingRCovFt then
/// fails AudioService::pathingBakeCoverageMismatch, which triggers the
/// loud automatic PATHING-ONLY re-bake (reflections carried forward) and
/// rewrites the file as v5 — the pre-coverage probe layout (no HubFill
/// probes, max-span-derived visRange) is exactly what that mismatch check
/// exists to keep out of a coverage-era run.
///
/// Reflection-profile mismatch policy (differs from the pathing fields):
/// a mismatch against the active profile does NOT auto-re-bake — the
/// reflection bake is the expensive half — it emits an UNCONDITIONAL loud
/// [FALLBACK] on EVERY load (AudioService::loadProbes) so dev-tier reverb
/// is never consumed silently in a ship run. A pathing-only re-bake
/// carries the reflection blob forward unchanged and PRESERVES these two
/// fields: the header describes the blob actually in the file, never the
/// profile that happened to be active during the re-bake.
///
/// Followed by `batchCount` batch records, each:
///   [0..3]   purpose     ProbePurpose enum (0=Reflections, 1=Pathing)
///   [4..7]   probeCount  probe count in this batch
///   [8..11]  payloadSize size of the Steam Audio serialized blob in bytes
///   [12..15] crc32       CRC-32 of the payload bytes
///   [16..15+payloadSize] payload (opaque Steam Audio probe batch data)
///
/// === Version 3 (legacy) — bake profile without density, not accepted ===
///
/// Same batch records but a 32-byte header ending at
/// bakedProbeDedupRadiusFt — no record of the pathing LAYOUT density.
/// Rejected (UnsupportedVersion) rather than migrated: v3 predates the
/// density tiers, so its pathing section was baked from the pre-tier
/// layout (upper centroids for every non-flat room, single center probes
/// on all non-door portals) — a layout no current density setting
/// reproduces, and one the density-mismatch check exists to keep out.
/// The version bump costs each mission ONE loud full re-bake (reflection
/// IRs cannot carry forward across the version reject — loadProbes fails,
/// so the startup flow runs the normal no-cache full bake; minutes at the
/// dev profile under the PR-A range cap).
///
/// === Version 2 (legacy) — multi-batch, no bake profile, not accepted ===
///
/// Same batch records but a 20-byte header with a zero `reserved` word where
/// v3 stores the pathing bake profile. Rejected (UnsupportedVersion) rather
/// than migrated: without the recorded visRange/numSamples we cannot verify
/// the cache matches the active runtime profile — the exact silent mismatch
/// v3 exists to prevent. The caller's version-check flow surfaces a loud
/// "re-bake" message; the re-bake is fast under the v3 bake-range cap.
///
/// === Version 1 (legacy) — single batch, no longer accepted ===
///
/// Pre-multi-batch files used a flat header { magic, version=1, probeCount,
/// payloadSize, crc32 } followed by a single payload. loadProbeFile returns
/// UnsupportedVersion for these; the user must delete the .probes file and
/// re-bake. The format is unrecoverable because we cannot determine whether
/// the single batch held reflections-only, pathing-only, or both.
///
/// Corner case for all legacy formats: the loader reads a v4-sized header
/// prefix (36 bytes) before the version check, so a legacy file SHORTER
/// than that (possible only for synthetic zero-batch/empty-payload files —
/// real bakes always carry multi-KB payloads) reports FileTooSmall instead
/// of UnsupportedVersion. Both statuses are loud delete-and-re-bake
/// rejections.

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace Darkness {

// ── CRC-32 (ISO 3309 / zlib polynomial 0xEDB88320) ───────────────────────

/// Compute CRC-32 of a byte buffer using the standard zlib polynomial.
/// This is the same algorithm used by PNG, ZIP, gzip, etc.
inline uint32_t crc32(const uint8_t *data, size_t length) {
    // Pre-computed lookup table for the standard CRC-32 polynomial.
    // Generated once on first call (static local, thread-safe in C++11+).
    static const auto table = []() {
        std::array<uint32_t, 256> t{};
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int j = 0; j < 8; ++j)
                c = (c >> 1) ^ ((c & 1) ? 0xEDB88320u : 0u);
            t[i] = c;
        }
        return t;
    }();

    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < length; ++i)
        crc = table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    return crc ^ 0xFFFFFFFFu;
}

// ── Probe file header ─────────────────────────────────────────────────────

static constexpr uint32_t kProbeFileMagic   = 0x72504B44u; // "DKPr" little-endian
static constexpr uint32_t kProbeFileVersion = 6u;
/// v4 (36-byte header) and v5 (44-byte) files are still accepted — the
/// loader zero-fills the missing tail fields and AudioService's mismatch
/// checks schedule the loud pathing-only re-bake: zero coverage (v4) trips
/// the coverage check, zero layout version (v4 AND v5) trips the
/// layout-version check. See the format doc.
static constexpr uint32_t kProbeFileVersionV5 = 5u;
static constexpr uint32_t kProbeFileVersionV4 = 4u;
static constexpr size_t   kProbeFileHeaderSize = 48u;        // 12 x 4-byte fields
static constexpr size_t   kProbeFileHeaderSizeV5 = 44u;      // v5 prefix
static constexpr size_t   kProbeFileHeaderSizeV4 = 36u;      // v4 prefix
static constexpr size_t   kProbeBatchRecordHeaderSize = 16u; // 4 x uint32_t

/// On-disk multi-batch file header (v5; the first 36 bytes are the v4
/// layout verbatim). All fields little-endian, 4-byte, so the struct is
/// packing-free on every supported ABI (static_assert below). Pathing
/// fields are 0 when the file has no pathing batch; the reflection fields
/// describe the reflection blob actually in the file (preserved across
/// pathing-only re-bakes). Full semantics in the format doc at the top of
/// this file.
struct ProbeFileHeader {
    uint32_t magic       = kProbeFileMagic;
    uint32_t version     = kProbeFileVersion;
    uint32_t batchCount  = 0;
    uint32_t totalProbes = 0;
    float    bakedPathingVisRangeFt = 0.0f; // v3: replaces the v2 reserved word
    uint32_t bakedPathingNumSamples = 0;    // v3: appended
    uint32_t bakedReflectionRays    = 0;    // v3: reflection-profile half
    float    bakedProbeDedupRadiusFt = 0.0f; // v3: probe-density half
    uint32_t bakedPathingDensity    = 0;    // v4: PathingProbeDensity tier
    float    bakedPathingCoverageFt = 0.0f; // v5: coverage governing (ft)
    float    bakedPathingRCovFt     = 0.0f; // v5: hub-fill coverage radius (ft)
    /// v6: pathing probe LAYOUT generation (kPathingLayoutVersion,
    /// SteamAudioPathing.h). 0 = pre-portal-first layout (v4/v5 files,
    /// room centroids + bend pairs); 1 = portal-first (WR-aperture nodes
    /// + region coverage fill). None of the OTHER header fields change
    /// across a layout rewrite, so without this a pre-rewrite cache loads
    /// silently and the run keeps the old layout with zero signal — the
    /// silent-stale-cache class the mismatch machinery exists to prevent.
    uint32_t bakedPathingLayoutVersion = 0;
};
static_assert(sizeof(ProbeFileHeader) == kProbeFileHeaderSize,
              "ProbeFileHeader must be exactly 48 bytes");
static_assert(offsetof(ProbeFileHeader, bakedPathingCoverageFt)
                  == kProbeFileHeaderSizeV4,
              "v5 tail must start exactly where the v4 header ended");
static_assert(offsetof(ProbeFileHeader, bakedPathingLayoutVersion)
                  == kProbeFileHeaderSizeV5,
              "v6 tail must start exactly where the v5 header ended");

/// On-disk per-batch record header (unchanged since v2). One precedes each
/// batch payload.
struct ProbeBatchRecordHeader {
    uint32_t purpose     = 0;   // ProbePurpose (0=Reflections, 1=Pathing)
    uint32_t probeCount  = 0;
    uint32_t payloadSize = 0;
    uint32_t crc32       = 0;
};
static_assert(sizeof(ProbeBatchRecordHeader) == kProbeBatchRecordHeaderSize,
              "ProbeBatchRecordHeader must be exactly 16 bytes");

/// Single in-memory probe batch read from disk. Matches one
/// ProbeBatchRecordHeader + payload pair.
struct ProbeBatchRecord {
    uint32_t              purpose = 0;
    uint32_t              probeCount = 0;
    uint32_t              crc32 = 0;
    std::vector<uint8_t>  payload;
};

/// Validation result for loadProbeFile / validateProbeFile.
enum class ProbeFileStatus {
    Ok,
    FileNotFound,
    FileTooSmall,
    BadMagic,
    UnsupportedVersion,
    SizeMismatch,    // payloadSize doesn't match actual file content
    CrcMismatch,     // CRC-32 of payload doesn't match header
    ReadError,       // I/O error during read
};

/// Human-readable description of a ProbeFileStatus.
inline const char *probeFileStatusString(ProbeFileStatus s) {
    switch (s) {
    case ProbeFileStatus::Ok:                 return "ok";
    case ProbeFileStatus::FileNotFound:       return "file not found";
    case ProbeFileStatus::FileTooSmall:       return "file too small for header";
    case ProbeFileStatus::BadMagic:           return "bad magic (not a .probes file)";
    case ProbeFileStatus::UnsupportedVersion: return "unsupported probe file version";
    case ProbeFileStatus::SizeMismatch:       return "payload size mismatch";
    case ProbeFileStatus::CrcMismatch:        return "CRC-32 checksum mismatch (corrupt data)";
    case ProbeFileStatus::ReadError:          return "I/O read error";
    }
    return "unknown error";
}

// ── Write ─────────────────────────────────────────────────────────────────

/// Write a multi-batch probe file with header + per-batch integrity envelopes.
/// Uses atomic write (tmp file + rename) so a crash never leaves a corrupt file.
///
/// @param outputPath   Final destination path (e.g. "mission.probes")
/// @param batches      Vector of per-batch records; CRC is computed per batch.
/// @param bakedPathingVisRangeFt v3 bake-profile record: the pathing bake's
///                     single-edge visRange cap in engine feet. Pass 0 when
///                     no pathing batch is written.
/// @param bakedPathingNumSamples v3 bake-profile record: the pathing bake's
///                     IPLPathBakeParams::numSamples. Pass 0 when no pathing
///                     batch is written.
/// @param bakedReflectionRays v3 bake-profile record: the reflection bake's
///                     ray count. On a pathing-only re-bake pass the LOADED
///                     file's value, not the active profile's — the header
///                     must describe the carried-forward blob.
/// @param bakedProbeDedupRadiusFt v3 bake-profile record: the reflection
///                     batch's global dedup radius (ft). Same carry-forward
///                     rule as bakedReflectionRays.
/// @param bakedPathingDensity v4 bake-profile record: PathingProbeDensity
///                     tier of the pathing section's probe layout
///                     (1=baseline, 2=bends). Pass 0 when no pathing batch
///                     is written.
/// @param bakedPathingCoverageFt v5 bake-profile record: the coverage
///                     governing value (max room-wise portal → nearest
///                     same-room probe, post hub fill) the visRange cap
///                     was derived from. Pass 0 when no pathing batch.
/// @param bakedPathingRCovFt v5 bake-profile record: the hub-fill coverage
///                     radius (kPathingCoverageRadiusFt) active at bake
///                     time. Pass 0 when no pathing batch.
/// @return true on success
inline bool writeProbeFile(const std::string &outputPath,
                           const std::vector<ProbeBatchRecord> &batches,
                           float bakedPathingVisRangeFt = 0.0f,
                           uint32_t bakedPathingNumSamples = 0,
                           uint32_t bakedReflectionRays = 0,
                           float bakedProbeDedupRadiusFt = 0.0f,
                           uint32_t bakedPathingDensity = 0,
                           float bakedPathingCoverageFt = 0.0f,
                           float bakedPathingRCovFt = 0.0f,
                           uint32_t bakedPathingLayoutVersion = 0)
{
    // Build outer header
    ProbeFileHeader hdr;
    hdr.batchCount = static_cast<uint32_t>(batches.size());
    hdr.totalProbes = 0;
    for (const auto &b : batches) hdr.totalProbes += b.probeCount;
    hdr.bakedPathingVisRangeFt = bakedPathingVisRangeFt;
    hdr.bakedPathingNumSamples = bakedPathingNumSamples;
    hdr.bakedReflectionRays    = bakedReflectionRays;
    hdr.bakedProbeDedupRadiusFt = bakedProbeDedupRadiusFt;
    hdr.bakedPathingDensity    = bakedPathingDensity;
    hdr.bakedPathingCoverageFt = bakedPathingCoverageFt;
    hdr.bakedPathingRCovFt     = bakedPathingRCovFt;
    hdr.bakedPathingLayoutVersion = bakedPathingLayoutVersion;

    std::string tmpPath = outputPath + ".tmp";
    FILE *f = std::fopen(tmpPath.c_str(), "wb");
    if (!f) return false;

    bool ok = true;

    if (std::fwrite(&hdr, sizeof(hdr), 1, f) != 1)
        ok = false;

    for (const auto &batch : batches) {
        if (!ok) break;
        ProbeBatchRecordHeader rh;
        rh.purpose     = batch.purpose;
        rh.probeCount  = batch.probeCount;
        rh.payloadSize = static_cast<uint32_t>(batch.payload.size());
        rh.crc32       = crc32(batch.payload.data(), batch.payload.size());

        if (std::fwrite(&rh, sizeof(rh), 1, f) != 1) { ok = false; break; }
        if (rh.payloadSize > 0 &&
            std::fwrite(batch.payload.data(), 1, rh.payloadSize, f) != rh.payloadSize) {
            ok = false; break;
        }
    }

    if (ok && std::fflush(f) != 0)
        ok = false;

    std::fclose(f);

    if (!ok) {
        std::remove(tmpPath.c_str());
        return false;
    }

    // Atomic rename — if this fails the old file (if any) is untouched
    if (std::rename(tmpPath.c_str(), outputPath.c_str()) != 0) {
        std::remove(tmpPath.c_str());
        return false;
    }

    return true;
}

// ── Read / Validate ───────────────────────────────────────────────────────

/// Load a multi-batch probe file from disk, validating CRC for each batch.
/// On success, `outHeader` carries the outer header and `outBatches` carries
/// one entry per probe batch. Legacy v1 files return UnsupportedVersion —
/// the caller must surface a clear "delete and re-bake" message.
inline ProbeFileStatus loadProbeFile(const std::string &path,
                                     ProbeFileHeader &outHeader,
                                     std::vector<ProbeBatchRecord> &outBatches) {
    outBatches.clear();
    FILE *f = std::fopen(path.c_str(), "rb");
    if (!f) return ProbeFileStatus::FileNotFound;

    // Read the v4-sized prefix first — magic/version live there, and a v4
    // file's header ENDS there. The v5 tail is read conditionally below so
    // batch records are parsed from the correct offset for both versions.
    outHeader = ProbeFileHeader{};
    if (std::fread(&outHeader, kProbeFileHeaderSizeV4, 1, f) != 1) {
        std::fclose(f);
        return ProbeFileStatus::FileTooSmall;
    }

    if (outHeader.magic != kProbeFileMagic) {
        std::fclose(f);
        return ProbeFileStatus::BadMagic;
    }

    if (outHeader.version == kProbeFileVersion) {
        // v6: read the full tail (coverage floats + layout version).
        if (std::fread(&outHeader.bakedPathingCoverageFt,
                       kProbeFileHeaderSize - kProbeFileHeaderSizeV4,
                       1, f) != 1) {
            std::fclose(f);
            return ProbeFileStatus::FileTooSmall;
        }
    } else if (outHeader.version == kProbeFileVersionV5) {
        // v5: coverage tail only; layout version stays zero-initialized,
        // which fails the layout-mismatch check in AudioService and
        // triggers the loud automatic pathing-only re-bake (the v5 layout
        // predates portal-first placement). Reflection IRs carry forward.
        if (std::fread(&outHeader.bakedPathingCoverageFt,
                       kProbeFileHeaderSizeV5 - kProbeFileHeaderSizeV4,
                       1, f) != 1) {
            std::fclose(f);
            return ProbeFileStatus::FileTooSmall;
        }
    } else if (outHeader.version == kProbeFileVersionV4) {
        // v4: accepted with zeroed coverage AND layout fields (the struct
        // init above already zeroed them); both mismatch checks fire, one
        // loud pathing-only re-bake rewrites the file as v6. The
        // reflection IR blob (the expensive half) carries forward
        // unchanged. See the format doc's v4 section.
    } else {
        std::fclose(f);
        return ProbeFileStatus::UnsupportedVersion;
    }

    outBatches.reserve(outHeader.batchCount);
    for (uint32_t i = 0; i < outHeader.batchCount; ++i) {
        ProbeBatchRecordHeader rh;
        if (std::fread(&rh, sizeof(rh), 1, f) != 1) {
            std::fclose(f);
            return ProbeFileStatus::FileTooSmall;
        }
        ProbeBatchRecord rec;
        rec.purpose    = rh.purpose;
        rec.probeCount = rh.probeCount;
        rec.crc32      = rh.crc32;
        rec.payload.resize(rh.payloadSize);
        if (rh.payloadSize > 0 &&
            std::fread(rec.payload.data(), 1, rh.payloadSize, f) != rh.payloadSize) {
            std::fclose(f);
            return ProbeFileStatus::ReadError;
        }
        if (crc32(rec.payload.data(), rec.payload.size()) != rh.crc32) {
            std::fclose(f);
            return ProbeFileStatus::CrcMismatch;
        }
        outBatches.push_back(std::move(rec));
    }
    std::fclose(f);

    return ProbeFileStatus::Ok;
}

} // namespace Darkness

#endif // __PROBE_FILE_H
