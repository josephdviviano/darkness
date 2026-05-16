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
#include <string>
#include <vector>

#include "audio/SpeechDatabase.h"

using namespace Darkness;

namespace {

// Synthetic-buffer builder for the Speech_DB layout (see SpeechDatabase.h).
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

    // Writes a name-map entry: '+' tag followed by a fixed 16-byte
    // NUL-padded field. Names longer than 15 chars are truncated to
    // match the engine's Label::text[16] convention.
    void nameEntry(const std::string &s) {
        u8('+');
        char buf[16] = {0};
        std::memcpy(buf, s.data(), std::min<size_t>(s.size(), 15));
        raw(buf, 16);
    }
    void emptyNameEntry() { u8('-'); }

    // Emits a NameMap with the given names. Bound fields are unused at
    // runtime but we set them so the parser sees plausible values.
    void nameMap(const std::vector<std::string> &names) {
        i32(static_cast<int32_t>(names.size() - 1)); // upperBound
        i32(0);                                       // lowerBound
        i32(static_cast<int32_t>(names.size()));      // size
        for (const auto &n : names) nameEntry(n);
    }
};

} // namespace

// ════════════════════════════════════════════════════════════════════════════
// Speech_DB chunk — raw byte / API smoke tests
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("SpeechDatabase default-constructed is empty", "[speech][parser]") {
    SpeechDatabase db;
    CHECK_FALSE(db.isLoaded());
    CHECK(db.rawSize() == 0);
    CHECK(db.headerBytes().empty());
    CHECK(db.rawBytes().empty());
    CHECK(db.conceptNames().empty());
    CHECK(db.tagNames().empty());
    CHECK(db.valueNames().empty());
    CHECK(db.entries().empty());
    CHECK(db.voiceCount() == 0);
}

TEST_CASE("SpeechDatabase rejects empty input", "[speech][parser]") {
    SpeechDatabase db;
    CHECK_FALSE(db.loadFromChunk(nullptr, 0));
    CHECK_FALSE(db.isLoaded());

    uint8_t dummy = 0;
    CHECK_FALSE(db.loadFromChunk(&dummy, 0));
    CHECK_FALSE(db.isLoaded());
}

TEST_CASE("SpeechDatabase clear empties the buffer", "[speech][parser]") {
    // Build a minimum decodable buffer.
    Bw w;
    w.nameMap({"c0"});
    w.nameMap({"t0"});
    w.nameMap({"v0"});
    w.i32(0);  // nConceptPrio
    w.i32(0);  // nTagFlags
    w.i32(0);  // nVoices

    SpeechDatabase db;
    REQUIRE(db.loadFromChunk(w.b.data(), w.b.size()));
    REQUIRE(db.isLoaded());

    db.clear();
    CHECK_FALSE(db.isLoaded());
    CHECK(db.rawSize() == 0);
    CHECK(db.headerBytes().empty());
}

TEST_CASE("SpeechDatabase second load replaces first", "[speech][parser]") {
    Bw a;
    a.nameMap({"alpha"});
    a.nameMap({"alpha_tag"});
    a.nameMap({"alpha_val"});
    a.i32(0); a.i32(0); a.i32(0);

    Bw b;
    b.nameMap({"beta_concept"});
    b.nameMap({"beta_tag"});
    b.nameMap({"beta_val"});
    b.i32(0); b.i32(0); b.i32(0);

    SpeechDatabase db;
    REQUIRE(db.loadFromChunk(a.b.data(), a.b.size()));
    REQUIRE(db.conceptNames().front() == "alpha");

    REQUIRE(db.loadFromChunk(b.b.data(), b.b.size()));
    CHECK(db.conceptNames().front() == "beta_concept");
}

// ════════════════════════════════════════════════════════════════════════════
// Decoder tests — synthetic buffers that exercise the full pipeline
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("SpeechDatabase decodes domain name maps",
          "[speech][parser][decode]") {
    Bw w;
    w.nameMap({"sleeping", "atlevelone", "atleveltwo"});
    w.nameMap({"Event", "Damage"});
    w.nameMap({"Acquire", "Footstep", "Damage", "Collision"});
    w.i32(3);  // nConceptPrio
    w.i32(1); w.i32(2); w.i32(3);
    w.i32(2);  // nTagFlags
    w.i32(2); w.i32(1);
    w.i32(0);  // nVoices

    SpeechDatabase db;
    REQUIRE(db.loadFromChunk(w.b.data(), w.b.size()));
    REQUIRE(db.conceptNames().size() == 3);
    CHECK(db.conceptNames()[0] == "sleeping");
    CHECK(db.conceptNames()[1] == "atlevelone");
    REQUIRE(db.tagNames().size() == 2);
    CHECK(db.tagNames()[0] == "Event");
    REQUIRE(db.valueNames().size() == 4);
    CHECK(db.valueNames()[3] == "Collision");
    REQUIRE(db.conceptPriorities().size() == 3);
    CHECK(db.conceptPriorities()[2] == 3);
    REQUIRE(db.tagFlags().size() == 2);
    CHECK(db.tagFlags()[0] == 2);
    CHECK(db.voiceCount() == 0);
    CHECK(db.entries().empty());
}

TEST_CASE("SpeechDatabase handles empty name-map slots",
          "[speech][parser][decode]") {
    // Sparse name map: index 0 is "x", index 1 is a '-' hole, index 2 is "z".
    Bw w;
    w.i32(2); w.i32(0); w.i32(3);    // upper=2, lower=0, size=3
    w.nameEntry("x");
    w.emptyNameEntry();
    w.nameEntry("z");
    // remaining maps + zero arrays + zero voices
    w.nameMap({"t0"});
    w.nameMap({"v0"});
    w.i32(0); w.i32(0); w.i32(0);

    SpeechDatabase db;
    REQUIRE(db.loadFromChunk(w.b.data(), w.b.size()));
    REQUIRE(db.conceptNames().size() == 3);
    CHECK(db.conceptNames()[0] == "x");
    CHECK(db.conceptNames()[1].empty());
    CHECK(db.conceptNames()[2] == "z");
}

TEST_CASE("SpeechDatabase decodes a voice with one tagged concept entry",
          "[speech][parser][decode]") {
    // Domain: 2 concepts, 1 tag, 2 values; 1 voice -> 2 concept dbs.
    // Voice 0 / concept 0: empty.
    // Voice 0 / concept 1: 1 branch keyed by (type=4, min=10, max=20),
    //   leaf with 1 record (schemaObjID=-42, weight=1.0).
    Bw w;
    w.nameMap({"sleeping", "atlevelzero"});
    w.nameMap({"Event"});
    w.nameMap({"Damage", "Footstep"});
    w.i32(2); w.i32(1); w.i32(2);   // nConceptPrio + values
    w.i32(1); w.i32(2);             // nTagFlags + value
    w.i32(1);                       // nVoices

    // Voice 0 / Concept 0 — completely empty db
    w.i32(0);    // nData
    w.i32(0);    // nBranch

    // Voice 0 / Concept 1 — one branch, one leaf
    w.i32(0);    // root nData
    w.i32(1);    // root nBranch=1
    w.u32(4);    // keyType
    w.i32(10); w.i32(20);  // min/max
    w.i32(1);    // child nData=1
    w.i32(-42); w.f32(1.0f);
    w.i32(0);    // child nBranch=0

    SpeechDatabase db;
    REQUIRE(db.loadFromChunk(w.b.data(), w.b.size()));
    CHECK(db.voiceCount() == 1);
    REQUIRE(db.entries().size() == 1);
    const auto &e = db.entries()[0];
    CHECK(e.voiceIndex == 0);
    CHECK(e.conceptIndex == 1);
    REQUIRE(e.keyPath.size() == 1);
    CHECK(e.keyPath[0].keyType == 4);
    CHECK(e.keyPath[0].keyMin  == 10);
    CHECK(e.keyPath[0].keyMax  == 20);
    REQUIRE(e.data.size() == 1);
    CHECK(e.data[0].schemaObjID == -42);
    CHECK(e.data[0].weight == 1.0f);
    CHECK(db.tailBytes().empty());
}

TEST_CASE("SpeechDatabase clamps long names to 15 chars",
          "[speech][parser][decode]") {
    // Engine's Label::text is 16 bytes incl. NUL terminator, so anything
    // longer than 15 chars on disk gets truncated by `strnlen(buf, 16)`.
    Bw w;
    w.nameMap({std::string(20, 'A')});  // builder itself clips to 15
    w.nameMap({"t"});
    w.nameMap({"v"});
    w.i32(0); w.i32(0); w.i32(0);

    SpeechDatabase db;
    REQUIRE(db.loadFromChunk(w.b.data(), w.b.size()));
    REQUIRE(db.conceptNames().size() == 1);
    CHECK(db.conceptNames()[0].size() == 15);
}

TEST_CASE("SpeechDatabase headerBytes clamps when chunk is shorter than N",
          "[speech][parser]") {
    std::vector<uint8_t> buf = {0xDE, 0xAD, 0xBE, 0xEF};
    SpeechDatabase db;
    // Will fail to fully decode (no domain), but rawBytes are stored.
    db.loadFromChunk(buf.data(), buf.size());
    auto header = db.headerBytes();
    REQUIRE(header.size() == 4);
    CHECK(header[0] == 0xDE);
}

TEST_CASE("SpeechDatabase dump produces non-empty output when loaded",
          "[speech][parser]") {
    Bw w;
    w.nameMap({"c"});
    w.nameMap({"t"});
    w.nameMap({"v"});
    w.i32(0); w.i32(0); w.i32(0);

    SpeechDatabase db;
    REQUIRE(db.loadFromChunk(w.b.data(), w.b.size()));
    std::ostringstream oss;
    db.dump(oss);
    const std::string out = oss.str();
    CHECK_FALSE(out.empty());
    CHECK(out.find("concepts=1") != std::string::npos);
}

TEST_CASE("SpeechDatabase dump prints a marker when not loaded",
          "[speech][parser]") {
    SpeechDatabase db;
    std::ostringstream oss;
    db.dump(oss);
    CHECK_FALSE(oss.str().empty());
}

// ════════════════════════════════════════════════════════════════════════════
// Resolver helpers — turn KeySegment indices into human-readable text.
// These are the bridge that lets ENV_SOUND (no name maps of its own) print
// readable output and that any future selector code will use to match
// queries to tag-database leaves.
// ════════════════════════════════════════════════════════════════════════════

namespace {
// Builds a SpeechDatabase with concrete tag/value name maps and a flag
// vector that marks tag 0 as int-typed (bit 0) and tag 1 as enum-typed
// (bit 1). No voices — the resolver doesn't depend on the voice tree.
SpeechDatabase makeResolverFixture() {
    Bw w;
    w.nameMap({"concept0"});
    w.nameMap({"CreatureType", "Event"});
    w.nameMap({"Idle", "Greet", "Damage"});
    w.i32(1); w.i32(0);                     // 1 concept-priority value
    w.i32(2);                                // nTagFlags
    w.i32(SpeechDatabase::kTagFlagInt);      // tag 0 = int-typed
    w.i32(SpeechDatabase::kTagFlagEnum);     // tag 1 = enum-typed
    w.i32(0);                                // 0 voices
    SpeechDatabase db;
    REQUIRE(db.loadFromChunk(w.b.data(), w.b.size()));
    return db;
}
} // namespace

TEST_CASE("SpeechDatabase resolver: tag flag predicates honour bit layout",
          "[speech][resolver]") {
    SpeechDatabase db = makeResolverFixture();
    CHECK(db.isIntTag(0));
    CHECK_FALSE(db.isEnumTag(0));
    CHECK(db.isEnumTag(1));
    CHECK_FALSE(db.isIntTag(1));
    // Out-of-range tag-type yields false for both — no UB, no exception.
    CHECK_FALSE(db.isIntTag(99));
    CHECK_FALSE(db.isEnumTag(99));
}

TEST_CASE("SpeechDatabase resolver: name lookups fall back to sentinel",
          "[speech][resolver]") {
    SpeechDatabase db = makeResolverFixture();
    CHECK(db.tagTypeName(0) == "CreatureType");
    CHECK(db.tagTypeName(1) == "Event");
    CHECK(db.tagTypeName(7) == "<tag 7>");
    CHECK(db.valueName(0) == "Idle");
    CHECK(db.valueName(2) == "Damage");
    CHECK(db.valueName(99) == "<value 99>");
    CHECK(db.conceptName(0) == "concept0");
    CHECK(db.conceptName(5) == "<concept 5>");
}

TEST_CASE("SpeechDatabase resolver: int-typed key formats as range",
          "[speech][resolver]") {
    SpeechDatabase db = makeResolverFixture();
    SpeechDatabase::KeySegment seg{};
    seg.keyType = 0;   // CreatureType, int-typed
    seg.keyMin = -256;
    seg.keyMax = -1;
    CHECK(db.formatKey(seg) == "CreatureType=[-256..-1]");
}

TEST_CASE("SpeechDatabase resolver: enum-typed key formats as csv list",
          "[speech][resolver]") {
    SpeechDatabase db = makeResolverFixture();
    SpeechDatabase::KeySegment seg{};
    seg.keyType = 1;                            // Event, enum-typed
    seg.keyEnums[0] = 0;                        // Idle
    seg.keyEnums[1] = 1;                        // Greet
    seg.keyEnums[2] = SpeechDatabase::kEnumTerminator;
    CHECK(db.formatKey(seg) == "Event=Idle,Greet");
}

TEST_CASE("SpeechDatabase resolver: enum key with only terminator labels empty",
          "[speech][resolver]") {
    SpeechDatabase db = makeResolverFixture();
    SpeechDatabase::KeySegment seg{};
    seg.keyType = 1;
    seg.keyEnums[0] = SpeechDatabase::kEnumTerminator;
    CHECK(db.formatKey(seg) == "Event=<empty>");
}

TEST_CASE("SpeechDatabase resolver: unknown tag flag falls back to int range",
          "[speech][resolver]") {
    // Tag 99 is outside the flag table — neither int nor enum. The
    // resolver should still print *something* useful; we treat unflagged
    // tags as integer ranges because that matches the stock Thief 2
    // pattern for "we never set a flag, treat as numeric".
    SpeechDatabase db = makeResolverFixture();
    SpeechDatabase::KeySegment seg{};
    seg.keyType = 99;
    seg.keyMin = 5;
    seg.keyMax = 10;
    CHECK(db.formatKey(seg) == "<tag 99>=[5..10]");
}

TEST_CASE("SpeechDatabase resolver: formatKeyPath joins with comma-space",
          "[speech][resolver]") {
    SpeechDatabase db = makeResolverFixture();
    std::vector<SpeechDatabase::KeySegment> path(2);
    path[0].keyType = 0;
    path[0].keyMin = 1; path[0].keyMax = 2;
    path[1].keyType = 1;
    path[1].keyEnums[0] = 2; // Damage
    path[1].keyEnums[1] = SpeechDatabase::kEnumTerminator;
    CHECK(db.formatKeyPath(path) == "CreatureType=[1..2], Event=Damage");
    CHECK(db.formatKeyPath({}) == "<root>");
}
