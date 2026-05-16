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

#include "audio/SpeechDatabase.h"

using namespace Darkness;

// ════════════════════════════════════════════════════════════════════════════
// Unit D: Speech_DB chunk first-pass parser
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("SpeechDatabase default-constructed is empty", "[speech][parser]") {
    SpeechDatabase db;
    CHECK_FALSE(db.isLoaded());
    CHECK(db.rawSize() == 0);
    CHECK(db.headerBytes().empty());
    CHECK(db.rawBytes().empty());
}

TEST_CASE("SpeechDatabase loads a synthetic buffer", "[speech][parser]") {
    // Build a 32-byte test buffer with a recognisable pattern so we can
    // verify header peek and raw-size accounting.
    std::vector<uint8_t> buf;
    for (uint8_t i = 0; i < 32; ++i)
        buf.push_back(i);

    SpeechDatabase db;
    REQUIRE(db.loadFromChunk(buf.data(), buf.size()));
    CHECK(db.isLoaded());
    CHECK(db.rawSize() == 32);

    auto header = db.headerBytes(16);
    REQUIRE(header.size() == 16);
    for (size_t i = 0; i < header.size(); ++i)
        CHECK(header[i] == static_cast<uint8_t>(i));

    // Underlying buffer matches the input exactly.
    REQUIRE(db.rawBytes().size() == buf.size());
    for (size_t i = 0; i < buf.size(); ++i)
        CHECK(db.rawBytes()[i] == buf[i]);
}

TEST_CASE("SpeechDatabase headerBytes clamps when chunk is shorter than N",
          "[speech][parser]") {
    std::vector<uint8_t> buf = {0xDE, 0xAD, 0xBE, 0xEF};

    SpeechDatabase db;
    REQUIRE(db.loadFromChunk(buf.data(), buf.size()));

    // Default of 16 bytes requested but only 4 are present.
    auto header = db.headerBytes();
    REQUIRE(header.size() == 4);
    CHECK(header[0] == 0xDE);
    CHECK(header[1] == 0xAD);
    CHECK(header[2] == 0xBE);
    CHECK(header[3] == 0xEF);

    // Explicit n smaller than buffer also works.
    auto two = db.headerBytes(2);
    REQUIRE(two.size() == 2);
    CHECK(two[0] == 0xDE);
    CHECK(two[1] == 0xAD);
}

TEST_CASE("SpeechDatabase rejects empty input", "[speech][parser]") {
    SpeechDatabase db;

    // Null data pointer -> false, remains unloaded.
    CHECK_FALSE(db.loadFromChunk(nullptr, 0));
    CHECK_FALSE(db.isLoaded());
    CHECK(db.rawSize() == 0);

    // Zero-size with non-null pointer also rejected.
    uint8_t dummy = 0;
    CHECK_FALSE(db.loadFromChunk(&dummy, 0));
    CHECK_FALSE(db.isLoaded());
    CHECK(db.rawSize() == 0);
}

TEST_CASE("SpeechDatabase second load replaces first", "[speech][parser]") {
    std::vector<uint8_t> a = {1, 2, 3, 4};
    std::vector<uint8_t> b = {9, 8, 7, 6, 5};

    SpeechDatabase db;
    REQUIRE(db.loadFromChunk(a.data(), a.size()));
    REQUIRE(db.rawSize() == 4);

    REQUIRE(db.loadFromChunk(b.data(), b.size()));
    CHECK(db.rawSize() == 5);
    auto header = db.headerBytes(5);
    REQUIRE(header.size() == 5);
    CHECK(header[0] == 9);
    CHECK(header[4] == 5);
}

TEST_CASE("SpeechDatabase clear empties the buffer", "[speech][parser]") {
    std::vector<uint8_t> buf = {1, 2, 3};

    SpeechDatabase db;
    REQUIRE(db.loadFromChunk(buf.data(), buf.size()));
    REQUIRE(db.isLoaded());

    db.clear();
    CHECK_FALSE(db.isLoaded());
    CHECK(db.rawSize() == 0);
    CHECK(db.headerBytes().empty());
}

TEST_CASE("SpeechDatabase dump produces non-empty output when loaded",
          "[speech][parser]") {
    std::vector<uint8_t> buf;
    for (uint8_t i = 0; i < 20; ++i)
        buf.push_back(static_cast<uint8_t>(0xA0 + i));

    SpeechDatabase db;
    REQUIRE(db.loadFromChunk(buf.data(), buf.size()));

    std::ostringstream oss;
    db.dump(oss);
    const std::string out = oss.str();
    CHECK_FALSE(out.empty());
    // Should mention the byte count and at least one hex digit pair.
    CHECK(out.find("20") != std::string::npos);
    CHECK(out.find("a0") != std::string::npos);
}

TEST_CASE("SpeechDatabase dump prints a marker when not loaded",
          "[speech][parser]") {
    SpeechDatabase db;
    std::ostringstream oss;
    db.dump(oss);
    const std::string out = oss.str();
    CHECK_FALSE(out.empty());
}
