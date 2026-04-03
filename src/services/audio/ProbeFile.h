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
/// File layout (all fields little-endian):
///   [0..3]   magic     "DKPr" (0x72504B44)
///   [4..7]   version   format version (currently 1)
///   [8..11]  probeCount  expected number of probes in the payload
///   [12..15] payloadSize size of the Steam Audio serialized blob in bytes
///   [16..19] crc32     CRC-32 of the payload bytes (ISO 3309 / zlib polynomial)
///   [20..19+payloadSize]  payload (opaque Steam Audio probe batch data)

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
static constexpr uint32_t kProbeFileVersion = 1u;
static constexpr size_t   kProbeFileHeaderSize = 20u;       // 5 x uint32_t

/// On-disk probe file header. All fields are little-endian uint32_t.
struct ProbeFileHeader {
    uint32_t magic       = kProbeFileMagic;
    uint32_t version     = kProbeFileVersion;
    uint32_t probeCount  = 0;
    uint32_t payloadSize = 0;
    uint32_t crc32       = 0;
};
static_assert(sizeof(ProbeFileHeader) == kProbeFileHeaderSize,
              "ProbeFileHeader must be exactly 20 bytes");

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

/// Write a probe file with header + integrity envelope.
/// Uses atomic write (tmp file + rename) so a crash never leaves a corrupt file.
///
/// @param outputPath   Final destination path (e.g. "mission.probes")
/// @param payload      Serialized Steam Audio probe batch data
/// @param payloadSize  Size of the payload in bytes
/// @param probeCount   Number of probes (stored in header for sanity checking on load)
/// @return true on success
inline bool writeProbeFile(const std::string &outputPath,
                           const uint8_t *payload, size_t payloadSize,
                           uint32_t probeCount)
{
    // Build header
    ProbeFileHeader hdr;
    hdr.probeCount  = probeCount;
    hdr.payloadSize = static_cast<uint32_t>(payloadSize);
    hdr.crc32       = crc32(payload, payloadSize);

    // Write to a temporary file first (atomic write pattern)
    std::string tmpPath = outputPath + ".tmp";
    FILE *f = std::fopen(tmpPath.c_str(), "wb");
    if (!f) return false;

    bool ok = true;

    // Write header
    if (std::fwrite(&hdr, sizeof(hdr), 1, f) != 1)
        ok = false;

    // Write payload
    if (ok && std::fwrite(payload, 1, payloadSize, f) != payloadSize)
        ok = false;

    // Flush to disk before rename
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

/// Validate a probe file on disk without loading the full payload.
/// Reads header + computes CRC of payload in streaming fashion.
inline ProbeFileStatus validateProbeFile(const std::string &path) {
    FILE *f = std::fopen(path.c_str(), "rb");
    if (!f) return ProbeFileStatus::FileNotFound;

    ProbeFileHeader hdr;
    if (std::fread(&hdr, sizeof(hdr), 1, f) != 1) {
        std::fclose(f);
        return ProbeFileStatus::FileTooSmall;
    }

    if (hdr.magic != kProbeFileMagic) {
        std::fclose(f);
        return ProbeFileStatus::BadMagic;
    }

    if (hdr.version != kProbeFileVersion) {
        std::fclose(f);
        return ProbeFileStatus::UnsupportedVersion;
    }

    // Verify file has enough bytes for the declared payload
    std::fseek(f, 0, SEEK_END);
    long fileSize = std::ftell(f);
    if (fileSize < 0 ||
        static_cast<size_t>(fileSize) != kProbeFileHeaderSize + hdr.payloadSize) {
        std::fclose(f);
        return ProbeFileStatus::SizeMismatch;
    }

    // Read payload and verify CRC
    std::fseek(f, kProbeFileHeaderSize, SEEK_SET);
    std::vector<uint8_t> payload(hdr.payloadSize);
    if (std::fread(payload.data(), 1, hdr.payloadSize, f) != hdr.payloadSize) {
        std::fclose(f);
        return ProbeFileStatus::ReadError;
    }
    std::fclose(f);

    if (crc32(payload.data(), payload.size()) != hdr.crc32)
        return ProbeFileStatus::CrcMismatch;

    return ProbeFileStatus::Ok;
}

/// Load a probe file from disk, validating integrity.
/// On success, outPayload contains the raw Steam Audio blob and
/// outHeader contains the validated header (including probe count).
inline ProbeFileStatus loadProbeFile(const std::string &path,
                                     ProbeFileHeader &outHeader,
                                     std::vector<uint8_t> &outPayload) {
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

    // Verify file size matches header
    std::fseek(f, 0, SEEK_END);
    long fileSize = std::ftell(f);
    if (fileSize < 0 ||
        static_cast<size_t>(fileSize) != kProbeFileHeaderSize + outHeader.payloadSize) {
        std::fclose(f);
        return ProbeFileStatus::SizeMismatch;
    }

    // Read payload
    std::fseek(f, kProbeFileHeaderSize, SEEK_SET);
    outPayload.resize(outHeader.payloadSize);
    if (std::fread(outPayload.data(), 1, outHeader.payloadSize, f) != outHeader.payloadSize) {
        std::fclose(f);
        return ProbeFileStatus::ReadError;
    }
    std::fclose(f);

    // Verify CRC
    if (crc32(outPayload.data(), outPayload.size()) != outHeader.crc32)
        return ProbeFileStatus::CrcMismatch;

    return ProbeFileStatus::Ok;
}

} // namespace Darkness

#endif // __PROBE_FILE_H
