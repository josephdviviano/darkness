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
#include <random>
#include <set>
#include <string>
#include <vector>

#include "audio/SpeechDatabase.h"
#include "audio/SpeechSelector.h"

using namespace Darkness;

namespace {

// Mirror of the synthetic-buffer builder in test_speech_database.cpp.
// Kept local to this TU to avoid coupling test files; the layout is
// stable (Speech_DB v1.3 / cSpeechDomain).
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
    void nameEntry(const std::string &s) {
        u8('+');
        char buf[16] = {0};
        std::memcpy(buf, s.data(), std::min<size_t>(s.size(), 15));
        raw(buf, 16);
    }
    void nameMap(const std::vector<std::string> &names) {
        i32(static_cast<int32_t>(names.size() - 1));
        i32(0);
        i32(static_cast<int32_t>(names.size()));
        for (const auto &n : names) nameEntry(n);
    }
};

// ── Fixture: 2 voices × 2 concepts each ────────────────────────────────
//
// Domain:
//   concepts = ["Greet", "Alert"]
//   tags     = ["Event"]       (enum-typed via tagFlags bit 1)
//   values   = ["Idle", "Combat"]
//
// Voice 0 / Greet  : default leaf → schemaObjID -100
// Voice 0 / Greet  : Event=Idle   → schemaObjID -101
// Voice 0 / Alert  : default leaf → schemaObjID -110
// Voice 1 / Greet  : default leaf → schemaObjID -200
// Voice 1 / Alert  : default leaf → schemaObjID -210
// Voice 1 / Alert  : Event=Combat → schemaObjID -211
//
// This exercises both voice-disambiguation and tag-filtered selection.
SpeechDatabase buildTwoVoiceFixture() {
    Bw w;
    w.nameMap({"Greet", "Alert"});            // concepts
    w.nameMap({"Event"});                       // tags
    w.nameMap({"Idle", "Combat"});             // values

    w.i32(2); w.i32(50); w.i32(75);            // 2 concept priorities
    w.i32(1); w.i32(SpeechDatabase::kTagFlagEnum);  // tag 0 = enum
    w.i32(2);                                    // nVoices

    // ─── Voice 0 ─── concept 0 (Greet): default leaf + Event=Idle leaf
    w.i32(1);                                   // root nData=1
    w.i32(-100); w.f32(1.0f);
    w.i32(1);                                   // root nBranch=1
    w.u32(0);                                   // keyType=Event
    // enum keys: 8-byte payload as enum bytes terminated by 0xFF
    w.u8(0); w.u8(0xFF);
    for (int i = 0; i < 6; ++i) w.u8(0xFF);    // pad to 8 bytes
    w.i32(1);                                   // child nData=1
    w.i32(-101); w.f32(1.0f);
    w.i32(0);                                   // child nBranch=0

    // ─── Voice 0 ─── concept 1 (Alert): default leaf only
    w.i32(1);                                   // nData=1
    w.i32(-110); w.f32(1.0f);
    w.i32(0);                                   // nBranch=0

    // ─── Voice 1 ─── concept 0 (Greet): default leaf only
    w.i32(1);
    w.i32(-200); w.f32(1.0f);
    w.i32(0);

    // ─── Voice 1 ─── concept 1 (Alert): default leaf + Event=Combat leaf
    w.i32(1);
    w.i32(-210); w.f32(1.0f);
    w.i32(1);                                   // nBranch=1
    w.u32(0);                                   // keyType=Event
    w.u8(1); w.u8(0xFF);                        // Combat (value index 1)
    for (int i = 0; i < 6; ++i) w.u8(0xFF);
    w.i32(1);                                   // child nData=1
    w.i32(-211); w.f32(1.0f);
    w.i32(0);                                   // child nBranch=0

    SpeechDatabase db;
    REQUIRE(db.loadFromChunk(w.b.data(), w.b.size()));
    return db;
}

} // namespace

// ════════════════════════════════════════════════════════════════════════
// Selector — voice disambiguation
// ════════════════════════════════════════════════════════════════════════

TEST_CASE("SpeechSelector: voice 0 and voice 1 yield different default schemas",
          "[speech][selector]")
{
    SpeechDatabase db = buildTwoVoiceFixture();
    SpeechSelector sel(db);
    REQUIRE(sel.isReady());

    // No tag query → only the default (empty-keyPath) leaves match.
    auto m0 = sel.selectMatches(0, "Greet", {});
    auto m1 = sel.selectMatches(1, "Greet", {});
    REQUIRE(m0.size() == 1);
    REQUIRE(m1.size() == 1);
    CHECK(m0[0].schemaObjID == -100);
    CHECK(m1[0].schemaObjID == -200);
}

TEST_CASE("SpeechSelector: selecting concept across voices does not cross-talk",
          "[speech][selector]")
{
    SpeechDatabase db = buildTwoVoiceFixture();
    SpeechSelector sel(db);
    auto greet = sel.selectMatches(0, "Greet", {});
    auto alert = sel.selectMatches(0, "Alert", {});
    REQUIRE(greet.size() == 1);
    REQUIRE(alert.size() == 1);
    CHECK(greet[0].schemaObjID == -100);
    CHECK(alert[0].schemaObjID == -110);
}

// ════════════════════════════════════════════════════════════════════════
// Selector — tag-filtered matching
// ════════════════════════════════════════════════════════════════════════

TEST_CASE("SpeechSelector: Event=Idle query matches both default and Idle leaves",
          "[speech][selector]")
{
    SpeechDatabase db = buildTwoVoiceFixture();
    SpeechSelector sel(db);

    // Voice 0 / Greet with Event=Idle:
    //  - default leaf (empty keyPath)         → matches → -100
    //  - Event=Idle leaf (keyPath constrains) → matches → -101
    SpeechSelector::TagQuery q;
    q.tagName = "Event";
    q.enumValues = {"Idle"};
    auto matches = sel.selectMatches(0, "Greet", {q});
    std::set<int32_t> ids;
    for (const auto &c : matches) ids.insert(c.schemaObjID);
    CHECK(ids.count(-100) == 1);
    CHECK(ids.count(-101) == 1);
    CHECK(matches.size() == 2);
}

TEST_CASE("SpeechSelector: Event=Combat selects -211 not -101 for voice 1 alert",
          "[speech][selector]")
{
    SpeechDatabase db = buildTwoVoiceFixture();
    SpeechSelector sel(db);

    SpeechSelector::TagQuery q;
    q.tagName = "Event";
    q.enumValues = {"Combat"};
    auto matches = sel.selectMatches(1, "Alert", {q});
    std::set<int32_t> ids;
    for (const auto &c : matches) ids.insert(c.schemaObjID);
    CHECK(ids.count(-210) == 1);  // default leaf still matches
    CHECK(ids.count(-211) == 1);  // tag-constrained leaf matches
}

TEST_CASE("SpeechSelector: tag-mismatched query drops the constrained leaf",
          "[speech][selector]")
{
    SpeechDatabase db = buildTwoVoiceFixture();
    SpeechSelector sel(db);

    // Voice 0 / Greet with Event=Combat — the Idle leaf must drop.
    SpeechSelector::TagQuery q;
    q.tagName = "Event";
    q.enumValues = {"Combat"};
    auto matches = sel.selectMatches(0, "Greet", {q});
    std::set<int32_t> ids;
    for (const auto &c : matches) ids.insert(c.schemaObjID);
    CHECK(ids.count(-100) == 1);
    CHECK(ids.count(-101) == 0);
    CHECK(matches.size() == 1);
}

TEST_CASE("SpeechSelector: unknown tag name leaves constrained leaves filtered out",
          "[speech][selector]")
{
    SpeechDatabase db = buildTwoVoiceFixture();
    SpeechSelector sel(db);

    SpeechSelector::TagQuery q;
    q.tagName = "NotARealTag";
    q.enumValues = {"Idle"};
    auto matches = sel.selectMatches(0, "Greet", {q});
    REQUIRE(matches.size() == 1);
    CHECK(matches[0].schemaObjID == -100);  // only the default survives
}

// ════════════════════════════════════════════════════════════════════════
// Selector — miss paths
// ════════════════════════════════════════════════════════════════════════

TEST_CASE("SpeechSelector: unknown concept returns no candidates",
          "[speech][selector]")
{
    SpeechDatabase db = buildTwoVoiceFixture();
    SpeechSelector sel(db);
    auto m = sel.selectMatches(0, "NoSuchConcept", {});
    CHECK(m.empty());
}

TEST_CASE("SpeechSelector: out-of-range voice returns no candidates",
          "[speech][selector]")
{
    SpeechDatabase db = buildTwoVoiceFixture();
    SpeechSelector sel(db);
    auto m = sel.selectMatches(99, "Greet", {});
    CHECK(m.empty());
    m = sel.selectMatches(-1, "Greet", {});
    CHECK(m.empty());
}

TEST_CASE("SpeechSelector: empty/unloaded DB returns no candidates",
          "[speech][selector]")
{
    SpeechDatabase empty;
    SpeechSelector sel(empty);
    CHECK_FALSE(sel.isReady());
    auto m = sel.selectMatches(0, "Greet", {});
    CHECK(m.empty());
}

// ════════════════════════════════════════════════════════════════════════
// Selector — pickOne weighted random
// ════════════════════════════════════════════════════════════════════════

TEST_CASE("SpeechSelector: pickOne on empty list returns sentinel candidate",
          "[speech][selector]")
{
    SpeechDatabase db = buildTwoVoiceFixture();
    SpeechSelector sel(db);
    std::mt19937 rng(0xC0DE);
    auto pick = sel.pickOne({}, rng);
    CHECK(pick.schemaObjID == 0);
}

TEST_CASE("SpeechSelector: pickOne is deterministic when seeded",
          "[speech][selector]")
{
    // Two candidates, equal weight — same seed must yield same pick.
    std::vector<SpeechSelector::Candidate> cands = {
        {1001, 1.0f}, {1002, 1.0f}
    };
    SpeechDatabase db = buildTwoVoiceFixture();
    SpeechSelector sel(db);
    std::mt19937 a(42), b(42);
    auto pa = sel.pickOne(cands, a);
    auto pb = sel.pickOne(cands, b);
    CHECK(pa.schemaObjID == pb.schemaObjID);
}

TEST_CASE("SpeechSelector: pickOne weights bias selection towards heavier candidate",
          "[speech][selector]")
{
    // 1:99 weight split — over many trials the heavy candidate dominates.
    std::vector<SpeechSelector::Candidate> cands = {
        {1, 1.0f}, {2, 99.0f}
    };
    SpeechDatabase db = buildTwoVoiceFixture();
    SpeechSelector sel(db);
    std::mt19937 rng(0xBEEF);

    int hits1 = 0, hits2 = 0;
    for (int i = 0; i < 1000; ++i) {
        auto p = sel.pickOne(cands, rng);
        if (p.schemaObjID == 1) ++hits1;
        else if (p.schemaObjID == 2) ++hits2;
    }
    // Heavy bucket must win clearly. Bounds are loose; tightening would
    // make the test flaky.
    CHECK(hits2 > 800);
    CHECK(hits1 < 200);
}

TEST_CASE("SpeechSelector: selectOne uses seed for reproducibility",
          "[speech][selector]")
{
    SpeechDatabase db = buildTwoVoiceFixture();
    SpeechSelector a(db);
    SpeechSelector b(db);
    a.seed(7);
    b.seed(7);

    SpeechSelector::TagQuery q;
    q.tagName = "Event";
    q.enumValues = {"Idle"};

    auto pa = a.selectOne(0, "Greet", {q});
    auto pb = b.selectOne(0, "Greet", {q});
    CHECK(pa.schemaObjID == pb.schemaObjID);
    CHECK(pa.schemaObjID != 0);
}
