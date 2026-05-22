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
/// === Version 2 (current) — multi-batch container ===
///
/// File layout (all fields little-endian):
///   [0..3]   magic       "DKPr" (0x72504B44)
///   [4..7]   version     format version (currently 2)
///   [8..11]  batchCount  number of probe batches that follow
///   [12..15] totalProbes total probe count across all batches (sanity)
///   [16..19] reserved    zero
///
/// Followed by `batchCount` batch records, each:
///   [0..3]   purpose     ProbePurpose enum (0=Reflections, 1=Pathing)
///   [4..7]   probeCount  probe count in this batch
///   [8..11]  payloadSize size of the Steam Audio serialized blob in bytes
///   [12..15] crc32       CRC-32 of the payload bytes
///   [16..15+payloadSize] payload (opaque Steam Audio probe batch data)
///
/// === Version 1 (legacy) — single batch, no longer accepted ===
///
/// Pre-multi-batch files used a flat header { magic, version=1, probeCount,
/// payloadSize, crc32 } followed by a single payload. loadProbeFile returns
/// UnsupportedVersion for these; the user must delete the .probes file and
/// re-bake. The format is unrecoverable because we cannot determine whether
/// the single batch held reflections-only, pathing-only, or both.

#include <array>
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
static constexpr uint32_t kProbeFileVersion = 2u;
static constexpr size_t   kProbeFileHeaderSize = 20u;       // 5 x uint32_t
static constexpr size_t   kProbeBatchRecordHeaderSize = 16u; // 4 x uint32_t

/// On-disk multi-batch file header (v2). All fields are little-endian uint32_t.
struct ProbeFileHeader {
    uint32_t magic       = kProbeFileMagic;
    uint32_t version     = kProbeFileVersion;
    uint32_t batchCount  = 0;
    uint32_t totalProbes = 0;
    uint32_t reserved    = 0;
};
static_assert(sizeof(ProbeFileHeader) == kProbeFileHeaderSize,
              "ProbeFileHeader must be exactly 20 bytes");

/// On-disk per-batch record header (v2). One precedes each batch payload.
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
/// @return true on success
inline bool writeProbeFile(const std::string &outputPath,
                           const std::vector<ProbeBatchRecord> &batches)
{
    // Build outer header
    ProbeFileHeader hdr;
    hdr.batchCount = static_cast<uint32_t>(batches.size());
    hdr.totalProbes = 0;
    for (const auto &b : batches) hdr.totalProbes += b.probeCount;

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

    if (std::fread(&outHeader, sizeof(outHeader), 1, f) != 1) {
        std::fclose(f);
        return ProbeFileStatus::FileTooSmall;
    }

    if (outHeader.magic != kProbeFileMagic) {
        std::fclose(f);
        return ProbeFileStatus::BadMagic;
    }

    if (outHeader.version != kProbeFileVersion) {
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
