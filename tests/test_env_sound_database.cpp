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
#include <sstream>
#include <vector>

#include "audio/EnvSoundDatabase.h"

using namespace Darkness;

// ════════════════════════════════════════════════════════════════════════════
// Unit E: ENV_SOUND chunk — chunk-presence detection + raw byte storage
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("EnvSoundDatabase default state is empty", "[env_sound][parser]") {
    EnvSoundDatabase db;
    CHECK_FALSE(db.isLoaded());
    CHECK(db.rawSize() == 0);
    CHECK(db.headerBytes(16).empty());
    CHECK(db.rawBytes().empty());
}

TEST_CASE("EnvSoundDatabase loads a 64-byte synthetic buffer",
          "[env_sound][parser]") {
    std::vector<uint8_t> buf(64);
    for (size_t i = 0; i < buf.size(); ++i) {
        buf[i] = static_cast<uint8_t>(i & 0xFF);
    }

    EnvSoundDatabase db;
    REQUIRE(db.loadFromChunk(buf.data(), buf.size()));
    CHECK(db.isLoaded());
    CHECK(db.rawSize() == 64);

    // headerBytes(16) must echo the first 16 input bytes verbatim.
    auto header = db.headerBytes(16);
    REQUIRE(header.size() == 16);
    for (size_t i = 0; i < 16; ++i) {
        CHECK(header[i] == buf[i]);
    }

    // Full raw-byte copy must also match exactly.
    REQUIRE(db.rawBytes().size() == buf.size());
    for (size_t i = 0; i < buf.size(); ++i) {
        CHECK(db.rawBytes()[i] == buf[i]);
    }
}

TEST_CASE("EnvSoundDatabase rejects nullptr data", "[env_sound][parser]") {
    EnvSoundDatabase db;
    CHECK_FALSE(db.loadFromChunk(nullptr, 16));
    CHECK_FALSE(db.isLoaded());
    CHECK(db.rawSize() == 0);
}

TEST_CASE("EnvSoundDatabase rejects size==0", "[env_sound][parser]") {
    EnvSoundDatabase db;
    const uint8_t dummy = 0xAB;
    CHECK_FALSE(db.loadFromChunk(&dummy, 0));
    CHECK_FALSE(db.isLoaded());
    CHECK(db.rawSize() == 0);
}

TEST_CASE("EnvSoundDatabase headerBytes clamps to available size",
          "[env_sound][parser]") {
    // When the chunk is smaller than the requested header length,
    // headerBytes must return the entire chunk without overrunning.
    std::vector<uint8_t> buf = {0x01, 0x02, 0x03, 0x04};
    EnvSoundDatabase db;
    REQUIRE(db.loadFromChunk(buf.data(), buf.size()));

    auto header = db.headerBytes(16);
    REQUIRE(header.size() == 4);
    CHECK(header[0] == 0x01);
    CHECK(header[3] == 0x04);
}

TEST_CASE("EnvSoundDatabase reload overwrites previous bytes",
          "[env_sound][parser]") {
    EnvSoundDatabase db;

    std::vector<uint8_t> first(32, 0xAA);
    REQUIRE(db.loadFromChunk(first.data(), first.size()));
    CHECK(db.rawSize() == 32);

    std::vector<uint8_t> second(8, 0x55);
    REQUIRE(db.loadFromChunk(second.data(), second.size()));
    CHECK(db.rawSize() == 8);
    CHECK(db.rawBytes()[0] == 0x55);
}

TEST_CASE("EnvSoundDatabase dump produces non-empty output when loaded",
          "[env_sound][parser]") {
    std::vector<uint8_t> buf(20);
    for (size_t i = 0; i < buf.size(); ++i) {
        buf[i] = static_cast<uint8_t>(0xF0 | (i & 0x0F));
    }

    EnvSoundDatabase db;
    REQUIRE(db.loadFromChunk(buf.data(), buf.size()));

    std::ostringstream oss;
    db.dump(oss);
    const std::string out = oss.str();
    CHECK_FALSE(out.empty());
    // Output should mention the size in some form.
    CHECK(out.find("20") != std::string::npos);
}

TEST_CASE("EnvSoundDatabase dump on empty database is well-defined",
          "[env_sound][parser]") {
    EnvSoundDatabase db;
    std::ostringstream oss;
    db.dump(oss);
    // Empty-state dump must still produce some descriptive output so
    // callers can distinguish "missing chunk" from "absent dump call".
    CHECK_FALSE(oss.str().empty());
}
