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

TEST_CASE("ProbeFileHeader is 20 bytes with correct defaults", "[probe][header]") {
    ProbeFileHeader hdr;
    CHECK(sizeof(hdr) == 20);
    CHECK(hdr.magic == kProbeFileMagic);
    CHECK(hdr.version == kProbeFileVersion);
    CHECK(hdr.probeCount == 0);
    CHECK(hdr.payloadSize == 0);
    CHECK(hdr.crc32 == 0);
}

// ════════════════════════════════════════════════════════════════════════════
// Round-trip: write then read
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Write + load round-trips correctly", "[probe][file]") {
    std::string path = tempPath("roundtrip");

    // Fake payload (not real Steam Audio data, but exercises the envelope)
    std::vector<uint8_t> payload = {0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03};
    uint32_t probeCount = 42;

    REQUIRE(writeProbeFile(path, payload.data(), payload.size(), probeCount));

    // Validate without loading payload
    CHECK(validateProbeFile(path) == ProbeFileStatus::Ok);

    // Full load
    ProbeFileHeader hdr;
    std::vector<uint8_t> loaded;
    CHECK(loadProbeFile(path, hdr, loaded) == ProbeFileStatus::Ok);
    CHECK(hdr.probeCount == probeCount);
    CHECK(hdr.payloadSize == payload.size());
    CHECK(loaded == payload);
    CHECK(hdr.crc32 == crc32(payload.data(), payload.size()));

    std::remove(path.c_str());
}

TEST_CASE("Write + load with empty payload", "[probe][file]") {
    std::string path = tempPath("empty_payload");

    REQUIRE(writeProbeFile(path, nullptr, 0, 0));
    CHECK(validateProbeFile(path) == ProbeFileStatus::Ok);

    ProbeFileHeader hdr;
    std::vector<uint8_t> loaded;
    CHECK(loadProbeFile(path, hdr, loaded) == ProbeFileStatus::Ok);
    CHECK(hdr.probeCount == 0);
    CHECK(loaded.empty());

    std::remove(path.c_str());
}

TEST_CASE("Write + load with large payload", "[probe][file]") {
    std::string path = tempPath("large");

    // 1 MB of pseudo-random data
    std::vector<uint8_t> payload(1024 * 1024);
    for (size_t i = 0; i < payload.size(); ++i)
        payload[i] = static_cast<uint8_t>((i * 137 + 43) & 0xFF);

    REQUIRE(writeProbeFile(path, payload.data(), payload.size(), 9999));
    CHECK(validateProbeFile(path) == ProbeFileStatus::Ok);

    ProbeFileHeader hdr;
    std::vector<uint8_t> loaded;
    CHECK(loadProbeFile(path, hdr, loaded) == ProbeFileStatus::Ok);
    CHECK(hdr.probeCount == 9999);
    CHECK(loaded == payload);

    std::remove(path.c_str());
}

// ════════════════════════════════════════════════════════════════════════════
// Atomic write: tmp file must not persist on success
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Atomic write does not leave .tmp file", "[probe][file]") {
    std::string path = tempPath("atomic");
    std::string tmpPath = path + ".tmp";

    std::vector<uint8_t> payload = {1, 2, 3};
    REQUIRE(writeProbeFile(path, payload.data(), payload.size(), 1));

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
    CHECK(validateProbeFile("/tmp/nonexistent_probe_file_xyz.probes")
          == ProbeFileStatus::FileNotFound);
}

TEST_CASE("FileTooSmall for truncated header", "[probe][corrupt]") {
    std::string path = tempPath("too_small");

    // Write just 10 bytes (header is 20)
    uint8_t partial[10] = {};
    REQUIRE(writeRaw(path, partial, sizeof(partial)));

    CHECK(validateProbeFile(path) == ProbeFileStatus::FileTooSmall);
    std::remove(path.c_str());
}

TEST_CASE("BadMagic for wrong magic bytes", "[probe][corrupt]") {
    std::string path = tempPath("bad_magic");

    ProbeFileHeader hdr;
    hdr.magic = 0xDEADBEEF;  // wrong magic
    REQUIRE(writeRaw(path, &hdr, sizeof(hdr)));

    CHECK(validateProbeFile(path) == ProbeFileStatus::BadMagic);
    std::remove(path.c_str());
}

TEST_CASE("UnsupportedVersion for future version", "[probe][corrupt]") {
    std::string path = tempPath("bad_version");

    ProbeFileHeader hdr;
    hdr.version = 999;
    REQUIRE(writeRaw(path, &hdr, sizeof(hdr)));

    CHECK(validateProbeFile(path) == ProbeFileStatus::UnsupportedVersion);
    std::remove(path.c_str());
}

TEST_CASE("SizeMismatch when file is truncated after header", "[probe][corrupt]") {
    std::string path = tempPath("size_mismatch");

    // Write a valid header claiming 100 bytes of payload, but provide 0
    ProbeFileHeader hdr;
    hdr.payloadSize = 100;
    hdr.probeCount = 5;
    hdr.crc32 = 0;
    REQUIRE(writeRaw(path, &hdr, sizeof(hdr)));

    CHECK(validateProbeFile(path) == ProbeFileStatus::SizeMismatch);
    std::remove(path.c_str());
}

TEST_CASE("CrcMismatch when payload is flipped", "[probe][corrupt]") {
    std::string path = tempPath("crc_flip");

    // Write a valid file first
    std::vector<uint8_t> payload = {0xAA, 0xBB, 0xCC, 0xDD};
    REQUIRE(writeProbeFile(path, payload.data(), payload.size(), 1));

    // Verify it's valid
    REQUIRE(validateProbeFile(path) == ProbeFileStatus::Ok);

    // Now corrupt one byte of the payload (byte 21 = first payload byte)
    FILE *f = std::fopen(path.c_str(), "r+b");
    REQUIRE(f != nullptr);
    std::fseek(f, kProbeFileHeaderSize, SEEK_SET);
    uint8_t corrupted = 0xFF;
    std::fwrite(&corrupted, 1, 1, f);
    std::fclose(f);

    CHECK(validateProbeFile(path) == ProbeFileStatus::CrcMismatch);
    std::remove(path.c_str());
}

TEST_CASE("CrcMismatch when header CRC field is zeroed", "[probe][corrupt]") {
    std::string path = tempPath("crc_zero");

    std::vector<uint8_t> payload = {0x01, 0x02, 0x03};
    REQUIRE(writeProbeFile(path, payload.data(), payload.size(), 1));

    // Zero out the CRC field (bytes 16-19)
    FILE *f = std::fopen(path.c_str(), "r+b");
    REQUIRE(f != nullptr);
    uint32_t zero = 0;
    std::fseek(f, 16, SEEK_SET);
    std::fwrite(&zero, sizeof(zero), 1, f);
    std::fclose(f);

    CHECK(validateProbeFile(path) == ProbeFileStatus::CrcMismatch);
    std::remove(path.c_str());
}

TEST_CASE("SizeMismatch when extra bytes appended", "[probe][corrupt]") {
    std::string path = tempPath("extra_bytes");

    std::vector<uint8_t> payload = {0x10, 0x20};
    REQUIRE(writeProbeFile(path, payload.data(), payload.size(), 1));

    // Append garbage bytes
    FILE *f = std::fopen(path.c_str(), "ab");
    REQUIRE(f != nullptr);
    uint8_t garbage[8] = {0xFF};
    std::fwrite(garbage, 1, sizeof(garbage), f);
    std::fclose(f);

    CHECK(validateProbeFile(path) == ProbeFileStatus::SizeMismatch);
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
