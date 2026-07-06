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
 *****************************************************************************/

#include <catch2/catch_test_macros.hpp>
#include "audio/ProbeFile.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

using namespace Darkness;

/// Helper: return a unique temp file path in the system temp directory.
static std::string tempPath(const char *suffix) {
    // Use FIXTURES_DIR if available (build-defined), else /tmp
#ifdef FIXTURES_DIR
    std::string dir = FIXTURES_DIR;
#else
    std::string dir = "/tmp";
#endif
    return dir + "/test_probe_" + suffix + ".probes";
}

/// Helper: write raw bytes to a file (bypassing our format, for corruption tests).
static bool writeRaw(const std::string &path, const void *data, size_t size) {
    FILE *f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    bool ok = std::fwrite(data, 1, size, f) == size;
    std::fclose(f);
    return ok;
}

/// Helper: build a single-batch record list for round-trip tests. Uses raw
/// purpose values to keep tests independent of the ProbePurpose enum (lives
/// in ProbeManager.h, but the wire format owns the integer mapping).
static constexpr uint32_t kPurposeReflections = 0;
static constexpr uint32_t kPurposePathing     = 1;

static std::vector<ProbeBatchRecord>
makeRecords(std::initializer_list<std::pair<uint32_t, std::vector<uint8_t>>> items,
            std::initializer_list<uint32_t> probeCounts)
{
    std::vector<ProbeBatchRecord> out;
    auto cit = probeCounts.begin();
    for (auto &it : items) {
        ProbeBatchRecord rec;
        rec.purpose    = it.first;
        rec.probeCount = (cit != probeCounts.end()) ? *cit++ : 0u;
        rec.payload    = it.second;
        out.push_back(std::move(rec));
    }
    return out;
}

// ════════════════════════════════════════════════════════════════════════════
// CRC-32 correctness
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("CRC-32 of empty buffer is 0x00000000", "[probe][crc]") {
    CHECK(crc32(nullptr, 0) == 0x00000000u);
}

TEST_CASE("CRC-32 of known test vectors", "[probe][crc]") {
    // "123456789" -> 0xCBF43926 (canonical CRC-32 check value)
    const uint8_t data[] = "123456789";
    CHECK(crc32(data, 9) == 0xCBF43926u);
}

TEST_CASE("CRC-32 single byte", "[probe][crc]") {
    const uint8_t a = 'A';
    uint32_t c1 = crc32(&a, 1);
    // Different input must produce different CRC
    const uint8_t b = 'B';
    uint32_t c2 = crc32(&b, 1);
    CHECK(c1 != c2);
}

// ════════════════════════════════════════════════════════════════════════════
// ProbeFileHeader layout
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("ProbeFileHeader is 24 bytes with correct defaults", "[probe][header]") {
    ProbeFileHeader hdr;
    CHECK(sizeof(hdr) == 24);
    CHECK(hdr.magic == kProbeFileMagic);
    CHECK(hdr.version == kProbeFileVersion);
    CHECK(hdr.batchCount == 0);
    CHECK(hdr.totalProbes == 0);
    CHECK(hdr.bakedPathingVisRangeFt == 0.0f);
    CHECK(hdr.bakedPathingNumSamples == 0);
}

TEST_CASE("ProbeBatchRecordHeader is 16 bytes", "[probe][header]") {
    ProbeBatchRecordHeader rh;
    CHECK(sizeof(rh) == 16);
    CHECK(rh.purpose == 0);
    CHECK(rh.probeCount == 0);
    CHECK(rh.payloadSize == 0);
    CHECK(rh.crc32 == 0);
}

// ════════════════════════════════════════════════════════════════════════════
// Round-trip: write then read (multi-batch v2)
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Write + load round-trips a single batch", "[probe][file]") {
    std::string path = tempPath("roundtrip_single");

    auto batches = makeRecords(
        { { kPurposeReflections, {0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03} } },
        { 42u });

    REQUIRE(writeProbeFile(path, batches));

    ProbeFileHeader hdr;
    std::vector<ProbeBatchRecord> loaded;
    CHECK(loadProbeFile(path, hdr, loaded) == ProbeFileStatus::Ok);
    CHECK(hdr.batchCount == 1);
    CHECK(hdr.totalProbes == 42);
    REQUIRE(loaded.size() == 1);
    CHECK(loaded[0].purpose == kPurposeReflections);
    CHECK(loaded[0].probeCount == 42);
    CHECK(loaded[0].payload == batches[0].payload);

    std::remove(path.c_str());
}

TEST_CASE("Write + load round-trips two batches", "[probe][file]") {
    std::string path = tempPath("roundtrip_two");

    auto batches = makeRecords(
        {
            { kPurposeReflections, {0x10, 0x20, 0x30, 0x40} },
            { kPurposePathing,     {0xAA, 0xBB, 0xCC, 0xDD, 0xEE} },
        },
        { 100u, 25u });

    REQUIRE(writeProbeFile(path, batches));

    ProbeFileHeader hdr;
    std::vector<ProbeBatchRecord> loaded;
    CHECK(loadProbeFile(path, hdr, loaded) == ProbeFileStatus::Ok);
    CHECK(hdr.batchCount == 2);
    CHECK(hdr.totalProbes == 125);
    REQUIRE(loaded.size() == 2);
    CHECK(loaded[0].purpose == kPurposeReflections);
    CHECK(loaded[0].probeCount == 100);
    CHECK(loaded[1].purpose == kPurposePathing);
    CHECK(loaded[1].probeCount == 25);

    std::remove(path.c_str());
}

TEST_CASE("Write + load round-trips v3 pathing bake profile", "[probe][file]") {
    std::string path = tempPath("roundtrip_profile");

    auto batches = makeRecords(
        { { kPurposePathing, {0x01, 0x02, 0x03} } },
        { 7u });

    REQUIRE(writeProbeFile(path, batches, /*visRangeFt*/ 300.0f,
                           /*numSamples*/ 16u));

    ProbeFileHeader hdr;
    std::vector<ProbeBatchRecord> loaded;
    CHECK(loadProbeFile(path, hdr, loaded) == ProbeFileStatus::Ok);
    CHECK(hdr.bakedPathingVisRangeFt == 300.0f);
    CHECK(hdr.bakedPathingNumSamples == 16u);

    // Default (no pathing batch) writes zeros.
    REQUIRE(writeProbeFile(path, batches));
    CHECK(loadProbeFile(path, hdr, loaded) == ProbeFileStatus::Ok);
    CHECK(hdr.bakedPathingVisRangeFt == 0.0f);
    CHECK(hdr.bakedPathingNumSamples == 0u);

    std::remove(path.c_str());
}

TEST_CASE("Write + load with empty payload batch", "[probe][file]") {
    std::string path = tempPath("empty_payload");

    auto batches = makeRecords(
        { { kPurposeReflections, {} } },
        { 0u });

    REQUIRE(writeProbeFile(path, batches));

    ProbeFileHeader hdr;
    std::vector<ProbeBatchRecord> loaded;
    CHECK(loadProbeFile(path, hdr, loaded) == ProbeFileStatus::Ok);
    REQUIRE(loaded.size() == 1);
    CHECK(loaded[0].probeCount == 0);
    CHECK(loaded[0].payload.empty());

    std::remove(path.c_str());
}

TEST_CASE("Write + load with large payload", "[probe][file]") {
    std::string path = tempPath("large");

    // 1 MB of pseudo-random data
    std::vector<uint8_t> payload(1024 * 1024);
    for (size_t i = 0; i < payload.size(); ++i)
        payload[i] = static_cast<uint8_t>((i * 137 + 43) & 0xFF);

    auto batches = makeRecords(
        { { kPurposeReflections, payload } },
        { 9999u });

    REQUIRE(writeProbeFile(path, batches));

    ProbeFileHeader hdr;
    std::vector<ProbeBatchRecord> loaded;
    CHECK(loadProbeFile(path, hdr, loaded) == ProbeFileStatus::Ok);
    REQUIRE(loaded.size() == 1);
    CHECK(loaded[0].probeCount == 9999);
    CHECK(loaded[0].payload == payload);

    std::remove(path.c_str());
}

// ════════════════════════════════════════════════════════════════════════════
// Atomic write: tmp file must not persist on success
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Atomic write does not leave .tmp file", "[probe][file]") {
    std::string path = tempPath("atomic");
    std::string tmpPath = path + ".tmp";

    auto batches = makeRecords(
        { { kPurposeReflections, {1, 2, 3} } },
        { 1u });
    REQUIRE(writeProbeFile(path, batches));

    // The .tmp file should have been renamed away
    FILE *f = std::fopen(tmpPath.c_str(), "rb");
    CHECK(f == nullptr);  // tmp file should not exist
    if (f) std::fclose(f);

    std::remove(path.c_str());
}

// ════════════════════════════════════════════════════════════════════════════
// Corruption detection
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("FileNotFound for nonexistent path", "[probe][corrupt]") {
    ProbeFileHeader hdr;
    std::vector<ProbeBatchRecord> loaded;
    CHECK(loadProbeFile("/tmp/nonexistent_probe_file_xyz.probes", hdr, loaded)
          == ProbeFileStatus::FileNotFound);
}

TEST_CASE("FileTooSmall for truncated header", "[probe][corrupt]") {
    std::string path = tempPath("too_small");

    // Write just 10 bytes (header is 20)
    uint8_t partial[10] = {};
    REQUIRE(writeRaw(path, partial, sizeof(partial)));

    ProbeFileHeader hdr;
    std::vector<ProbeBatchRecord> loaded;
    CHECK(loadProbeFile(path, hdr, loaded) == ProbeFileStatus::FileTooSmall);
    std::remove(path.c_str());
}

TEST_CASE("BadMagic for wrong magic bytes", "[probe][corrupt]") {
    std::string path = tempPath("bad_magic");

    ProbeFileHeader hdr;
    hdr.magic = 0xDEADBEEF;  // wrong magic
    REQUIRE(writeRaw(path, &hdr, sizeof(hdr)));

    ProbeFileHeader gotHdr;
    std::vector<ProbeBatchRecord> loaded;
    CHECK(loadProbeFile(path, gotHdr, loaded) == ProbeFileStatus::BadMagic);
    std::remove(path.c_str());
}

TEST_CASE("UnsupportedVersion for future version", "[probe][corrupt]") {
    std::string path = tempPath("bad_version");

    ProbeFileHeader hdr;
    hdr.version = 999;
    REQUIRE(writeRaw(path, &hdr, sizeof(hdr)));

    ProbeFileHeader gotHdr;
    std::vector<ProbeBatchRecord> loaded;
    CHECK(loadProbeFile(path, gotHdr, loaded) == ProbeFileStatus::UnsupportedVersion);
    std::remove(path.c_str());
}

TEST_CASE("UnsupportedVersion for legacy v2 file", "[probe][corrupt]") {
    // Legacy v2 had a 20-byte header { magic, version=2, batchCount,
    // totalProbes, reserved } — no pathing bake profile. The v3 loader
    // must reject it (the caller then surfaces a loud re-bake message);
    // migration is impossible because the bake's visRange/numSamples
    // are unrecorded and cannot be verified against the runtime profile.
    std::string path = tempPath("legacy_v2");

    // 20-byte v2 header followed by the first bytes of a batch record
    // header (real v2 files always have batch records after the header;
    // the v3 loader reads 24 bytes before checking the version, so the
    // file must be at least that long to exercise the version check).
    struct LegacyV2FilePrefix {
        uint32_t magic;
        uint32_t version;
        uint32_t batchCount;
        uint32_t totalProbes;
        uint32_t reserved;
        uint32_t firstRecordPurpose;
    } legacy{};
    legacy.magic              = kProbeFileMagic;
    legacy.version            = 2;
    legacy.batchCount         = 1;
    legacy.firstRecordPurpose = 0;
    REQUIRE(writeRaw(path, &legacy, sizeof(legacy)));

    ProbeFileHeader hdr;
    std::vector<ProbeBatchRecord> loaded;
    CHECK(loadProbeFile(path, hdr, loaded) == ProbeFileStatus::UnsupportedVersion);
    std::remove(path.c_str());
}

TEST_CASE("UnsupportedVersion for legacy v1 file", "[probe][corrupt]") {
    // Legacy v1 had { magic, version=1, probeCount, payloadSize, crc32 }
    // followed by the serialized payload (never empty in real files —
    // the v3 loader reads a 24-byte header before the version check, so
    // the payload bytes are what get it past the size check). The new
    // loader must reject without trying to migrate.
    std::string path = tempPath("legacy_v1");

    struct LegacyV1FilePrefix {
        uint32_t magic;
        uint32_t version;
        uint32_t probeCount;
        uint32_t payloadSize;
        uint32_t crc32;
        uint32_t payloadStart;
    } legacy{};
    legacy.magic        = kProbeFileMagic;
    legacy.version      = 1;
    legacy.probeCount   = 100;
    legacy.payloadSize  = 4;
    legacy.crc32        = 0;
    legacy.payloadStart = 0xDEADBEEF;
    REQUIRE(writeRaw(path, &legacy, sizeof(legacy)));

    ProbeFileHeader hdr;
    std::vector<ProbeBatchRecord> loaded;
    CHECK(loadProbeFile(path, hdr, loaded) == ProbeFileStatus::UnsupportedVersion);
    std::remove(path.c_str());
}

TEST_CASE("CrcMismatch when payload byte is flipped", "[probe][corrupt]") {
    std::string path = tempPath("crc_flip");

    std::vector<uint8_t> payload = {0xAA, 0xBB, 0xCC, 0xDD};
    auto batches = makeRecords({ { kPurposeReflections, payload } }, { 1u });
    REQUIRE(writeProbeFile(path, batches));

    // Round-trip OK before corruption
    {
        ProbeFileHeader hdr;
        std::vector<ProbeBatchRecord> loaded;
        REQUIRE(loadProbeFile(path, hdr, loaded) == ProbeFileStatus::Ok);
    }

    // Corrupt the first payload byte. Layout: file header (20) +
    // record header (16) + payload (4). Payload starts at byte 36.
    FILE *f = std::fopen(path.c_str(), "r+b");
    REQUIRE(f != nullptr);
    std::fseek(f, kProbeFileHeaderSize + kProbeBatchRecordHeaderSize, SEEK_SET);
    uint8_t corrupted = 0xFF;
    std::fwrite(&corrupted, 1, 1, f);
    std::fclose(f);

    ProbeFileHeader hdr;
    std::vector<ProbeBatchRecord> loaded;
    CHECK(loadProbeFile(path, hdr, loaded) == ProbeFileStatus::CrcMismatch);
    std::remove(path.c_str());
}

// ════════════════════════════════════════════════════════════════════════════
// probeFileStatusString coverage
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("All ProbeFileStatus values have non-empty descriptions", "[probe][status]") {
    CHECK(std::strlen(probeFileStatusString(ProbeFileStatus::Ok)) > 0);
    CHECK(std::strlen(probeFileStatusString(ProbeFileStatus::FileNotFound)) > 0);
    CHECK(std::strlen(probeFileStatusString(ProbeFileStatus::FileTooSmall)) > 0);
    CHECK(std::strlen(probeFileStatusString(ProbeFileStatus::BadMagic)) > 0);
    CHECK(std::strlen(probeFileStatusString(ProbeFileStatus::UnsupportedVersion)) > 0);
    CHECK(std::strlen(probeFileStatusString(ProbeFileStatus::SizeMismatch)) > 0);
    CHECK(std::strlen(probeFileStatusString(ProbeFileStatus::CrcMismatch)) > 0);
    CHECK(std::strlen(probeFileStatusString(ProbeFileStatus::ReadError)) > 0);
}
