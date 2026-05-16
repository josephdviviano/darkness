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

#include <cstdint>
#include <cstring>
#include <sstream>
#include <vector>

#include "audio/EnvSoundDatabase.h"

using namespace Darkness;

namespace {

// Tiny little-endian writer to build a synthetic ENV_SOUND buffer for the
// decoder tests. Mirrors the on-disk layout documented in
// EnvSoundDatabase.h:
//
//   int32 nLocalTagRequired
//   uint8[nLocalTagRequired]
//   tagdb root:
//     int32 nData, {int32 schemaObjID, float weight}[nData]
//     int32 nBranch, {uint32 keyType, uint8[8] keyUnion, tagdb child}[nBranch]
struct Bw {
    std::vector<uint8_t> b;
    void i32(int32_t v) {
        for (int i = 0; i < 4; ++i)
            b.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
    }
    void u32(uint32_t v) {
        for (int i = 0; i < 4; ++i)
            b.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
    }
    void f32(float v) {
        uint32_t u;
        std::memcpy(&u, &v, 4);
        u32(u);
    }
    void u8(uint8_t v) { b.push_back(v); }
    void raw(const void *p, size_t n) {
        const auto *src = static_cast<const uint8_t *>(p);
        b.insert(b.end(), src, src + n);
    }
};

} // namespace

// ════════════════════════════════════════════════════════════════════════════
// ENV_SOUND chunk — raw byte storage (kept from the first-pass parser tests)
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("EnvSoundDatabase default state is empty", "[env_sound][parser]") {
    EnvSoundDatabase db;
    CHECK_FALSE(db.isLoaded());
    CHECK(db.rawSize() == 0);
    CHECK(db.headerBytes(16).empty());
    CHECK(db.rawBytes().empty());
    CHECK(db.entries().empty());
    CHECK(db.tailBytes().empty());
}

TEST_CASE("EnvSoundDatabase rejects nullptr data", "[env_sound][parser]") {
    EnvSoundDatabase db;
    CHECK_FALSE(db.loadFromChunk(nullptr, 16));
    CHECK_FALSE(db.isLoaded());
}

TEST_CASE("EnvSoundDatabase rejects size==0", "[env_sound][parser]") {
    EnvSoundDatabase db;
    const uint8_t dummy = 0xAB;
    CHECK_FALSE(db.loadFromChunk(&dummy, 0));
    CHECK_FALSE(db.isLoaded());
}

TEST_CASE("EnvSoundDatabase headerBytes clamps to available size",
          "[env_sound][parser]") {
    // A 4-byte buffer that's just nLocalTagRequired=0 followed by EOF.
    // Decode itself will fail (no root db), but rawBytes / headerBytes
    // should still echo what we passed in for inspection purposes.
    Bw w; w.i32(0);
    EnvSoundDatabase db;
    (void)db.loadFromChunk(w.b.data(), w.b.size());

    auto header = db.headerBytes(16);
    REQUIRE(header.size() == 4);
    CHECK(header[0] == 0x00);
}

// ════════════════════════════════════════════════════════════════════════════
// Decoder tests — exercise the synthetic-buffer code paths
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("EnvSoundDatabase decodes an empty tag tree",
          "[env_sound][parser][decode]") {
    // Minimal valid chunk: no required-tag bytes, root has no data and
    // no branches.
    Bw w;
    w.i32(0);         // nLocalTagRequired
    w.i32(0);         // root nData
    w.i32(0);         // root nBranch

    EnvSoundDatabase db;
    REQUIRE(db.loadFromChunk(w.b.data(), w.b.size()));
    CHECK(db.isLoaded());
    CHECK(db.entries().empty());
    CHECK(db.localTagRequired().empty());
    CHECK(db.tailBytes().empty());
}

TEST_CASE("EnvSoundDatabase decodes a single-leaf tree",
          "[env_sound][parser][decode]") {
    // Required-tag bitarray: 1 byte = 0x01.
    // Root has 0 data, 1 branch:
    //   key: type=20, min=-128, max=-1
    //   child: 2 data records, 0 branches
    Bw w;
    w.i32(1); w.u8(0x01);    // local-tag-required
    w.i32(0);                // root nData=0
    w.i32(1);                // root nBranch=1
    w.u32(20);               // key type
    w.i32(-128); w.i32(-1);  // 8-byte union (read as min/max here)
    w.i32(2);                // child nData=2
    w.i32(-6587); w.f32(1.0f);
    w.i32(-729);  w.f32(0.5f);
    w.i32(0);                // child nBranch=0

    EnvSoundDatabase db;
    REQUIRE(db.loadFromChunk(w.b.data(), w.b.size()));
    REQUIRE(db.isLoaded());
    REQUIRE(db.entries().size() == 1);
    const auto &e = db.entries()[0];
    REQUIRE(e.keyPath.size() == 1);
    CHECK(e.keyPath[0].keyType == 20);
    CHECK(e.keyPath[0].keyMin  == -128);
    CHECK(e.keyPath[0].keyMax  == -1);
    REQUIRE(e.data.size() == 2);
    CHECK(e.data[0].schemaObjID == -6587);
    CHECK(e.data[0].weight == 1.0f);
    CHECK(e.data[1].schemaObjID == -729);
    CHECK(e.data[1].weight == 0.5f);

    // local-tag-required round-trip
    REQUIRE(db.localTagRequired().size() == 1);
    CHECK(db.localTagRequired()[0] == 0x01);
    CHECK(db.tailBytes().empty());
}

TEST_CASE("EnvSoundDatabase decodes nested branches with intermediate data",
          "[env_sound][parser][decode]") {
    // Root has one data record + one branch leading to another node
    // with its own data. Verifies that interior nodes carrying data
    // are surfaced (mirrors how the engine stores fallback object sets
    // at non-leaf levels of the tree).
    Bw w;
    w.i32(0);                          // no required-tag bytes
    w.i32(1);                          // root nData=1
    w.i32(100); w.f32(2.0f);           //   one record
    w.i32(1);                          // root nBranch=1
    w.u32(7);                          // key type = 7
    w.i32(0); w.i32(255);              //   min=0, max=255
    w.i32(1);                          // child nData=1
    w.i32(200); w.f32(3.0f);
    w.i32(0);                          // child nBranch=0

    EnvSoundDatabase db;
    REQUIRE(db.loadFromChunk(w.b.data(), w.b.size()));
    REQUIRE(db.entries().size() == 2);
    // Root entry has no keys
    CHECK(db.entries()[0].keyPath.empty());
    CHECK(db.entries()[0].data.size() == 1);
    CHECK(db.entries()[0].data[0].schemaObjID == 100);
    // Child entry has one key segment
    REQUIRE(db.entries()[1].keyPath.size() == 1);
    CHECK(db.entries()[1].keyPath[0].keyType == 7);
    CHECK(db.entries()[1].keyPath[0].keyMax == 255);
    CHECK(db.entries()[1].data[0].schemaObjID == 200);
}

TEST_CASE("EnvSoundDatabase stashes unparseable tail in tailBytes",
          "[env_sound][parser][decode]") {
    // Build a header that promises a root node, then truncate after the
    // count word. Decode should fail gracefully and surface the leftover
    // bytes via tailBytes() rather than crashing or aborting the parse.
    Bw w;
    w.i32(0);     // no required-tag bytes
    w.i32(5);     // root nData = 5, but we don't write the 5 records
    // Add a couple garbage trailing bytes.
    w.u8(0xAA); w.u8(0xBB); w.u8(0xCC);

    EnvSoundDatabase db;
    // Parser returns false (no entries decoded) and stores the unparsed
    // tail. Calling code can still inspect rawBytes() for fingerprinting.
    db.loadFromChunk(w.b.data(), w.b.size());
    CHECK(db.entries().empty());
    CHECK(db.tailBytes().size() > 0);
}

TEST_CASE("EnvSoundDatabase dump produces non-empty output",
          "[env_sound][parser]") {
    Bw w;
    w.i32(0); w.i32(0); w.i32(0);

    EnvSoundDatabase db;
    REQUIRE(db.loadFromChunk(w.b.data(), w.b.size()));
    std::ostringstream oss;
    db.dump(oss);
    CHECK_FALSE(oss.str().empty());
    CHECK(oss.str().find("ENV_SOUND") != std::string::npos);
}

TEST_CASE("EnvSoundDatabase dump on empty database is well-defined",
          "[env_sound][parser]") {
    EnvSoundDatabase db;
    std::ostringstream oss;
    db.dump(oss);
    CHECK_FALSE(oss.str().empty());
}
