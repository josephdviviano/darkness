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

#include <array>
#include <cstdint>
#include <cstring>

#include "audio/AIHearingData.h"

using namespace Darkness;

// ════════════════════════════════════════════════════════════════════════════
// Struct layout — sizes must match on-disk chunk sizes exactly.
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("AIHearingStats is 48 bytes", "[ai_hearing][parser]") {
    CHECK(sizeof(AIHearingStats) == 48);
    // 6 floats dist_muls + 6 int32 db_adds = 12 * 4 bytes.
    CHECK(sizeof(AIHearingStats) ==
          sizeof(float)   * AI_HEARING_COUNT +
          sizeof(int32_t) * AI_HEARING_COUNT);
}

TEST_CASE("AISoundTweaks is 24 bytes", "[ai_hearing][parser]") {
    CHECK(sizeof(AISoundTweaks) == 24);
    // 6 int32 defaultRanges = 6 * 4 bytes.
    CHECK(sizeof(AISoundTweaks) ==
          sizeof(int32_t) * AI_SOUND_TYPE_COUNT);
}

// ════════════════════════════════════════════════════════════════════════════
// AIHearStat chunk decoder — 48-byte buffer: 6 floats then 6 int32s.
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("readAIHearStat round-trips synthetic 48-byte buffer",
          "[ai_hearing][parser]") {
    // Build a synthetic chunk: 6 float dist_muls then 6 int32 db_adds.
    const float   wantDist[6] = { 0.0f, 0.1f, 0.5f, 1.0f, 2.0f, 4.5f };
    const int32_t wantDb  [6] = { 99999, 500, 100, 0, -100, -500 };

    uint8_t buf[48];
    std::memcpy(buf + 0,  wantDist, sizeof(wantDist));
    std::memcpy(buf + 24, wantDb,   sizeof(wantDb));

    AIHearingStats out{};
    REQUIRE(readAIHearStat(buf, sizeof(buf), out));

    for (int i = 0; i < AI_HEARING_COUNT; ++i) {
        CHECK(out.dist_muls[i] == wantDist[i]);
        CHECK(out.db_adds[i]   == wantDb[i]);
    }
}

// Regression: the original engine stores db_adds as int32, not float.
// Verify the exact byte sequence the gam file ships with round-trips to
// the expected integer centibel values. Decoding bytes 24-47 as floats
// (the previous buggy behavior) produced denormals and NaNs.
TEST_CASE("readAIHearStat decodes original-engine default bytes",
          "[ai_hearing][parser]") {
    const float   wantDist[6] = { 0.0f, 0.25f, 0.65f, 1.0f, 1.5f, 3.0f };
    const int32_t wantDb  [6] = { 1000000, 1000, 200, 0, -200, -1000 };

    uint8_t buf[48];
    std::memcpy(buf + 0,  wantDist, sizeof(wantDist));
    std::memcpy(buf + 24, wantDb,   sizeof(wantDb));

    AIHearingStats out{};
    REQUIRE(readAIHearStat(buf, sizeof(buf), out));

    for (int i = 0; i < AI_HEARING_COUNT; ++i) {
        CHECK(out.dist_muls[i] == wantDist[i]);
        CHECK(out.db_adds[i]   == wantDb[i]);
    }
    CHECK(out.db_adds[AI_HEARING_DEAF]      == 1000000);
    CHECK(out.db_adds[AI_HEARING_NORMAL]    == 0);
    CHECK(out.db_adds[AI_HEARING_VERY_HIGH] == -1000);
}

TEST_CASE("readAIHearStat rejects size mismatch", "[ai_hearing][parser]") {
    uint8_t buf[64] = {};
    AIHearingStats out{};

    // Too small.
    CHECK_FALSE(readAIHearStat(buf, 47, out));
    CHECK_FALSE(readAIHearStat(buf, 0,  out));
    CHECK_FALSE(readAIHearStat(buf, 24, out));

    // Too large.
    CHECK_FALSE(readAIHearStat(buf, 49, out));
    CHECK_FALSE(readAIHearStat(buf, 64, out));

    // Null pointer.
    CHECK_FALSE(readAIHearStat(nullptr, 48, out));

    // Correct size succeeds.
    CHECK(readAIHearStat(buf, 48, out));
}

// ════════════════════════════════════════════════════════════════════════════
// AISNDTWK chunk decoder — 24-byte little-endian int32 round-trip.
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("readAISndTwk round-trips synthetic 24-byte buffer",
          "[ai_hearing][parser]") {
    const int32_t want[6] = { 0, 50, 200, 400, 800, 2400 };

    uint8_t buf[24];
    std::memcpy(buf, want, sizeof(want));

    AISoundTweaks out{};
    REQUIRE(readAISndTwk(buf, sizeof(buf), out));

    for (int i = 0; i < AI_SOUND_TYPE_COUNT; ++i)
        CHECK(out.defaultRanges[i] == want[i]);
}

TEST_CASE("readAISndTwk rejects size mismatch", "[ai_hearing][parser]") {
    uint8_t buf[32] = {};
    AISoundTweaks out{};

    // Too small.
    CHECK_FALSE(readAISndTwk(buf, 23, out));
    CHECK_FALSE(readAISndTwk(buf, 0,  out));
    CHECK_FALSE(readAISndTwk(buf, 12, out));

    // Too large.
    CHECK_FALSE(readAISndTwk(buf, 25, out));
    CHECK_FALSE(readAISndTwk(buf, 32, out));

    // Null pointer.
    CHECK_FALSE(readAISndTwk(nullptr, 24, out));

    // Correct size succeeds.
    CHECK(readAISndTwk(buf, 24, out));
}

// ════════════════════════════════════════════════════════════════════════════
// kDefaultAIHearingStats — must match the original engine's default values.
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("kDefaultAIHearingStats matches original engine defaults",
          "[ai_hearing][parser]") {
    const float   expectedDist[6] = { 0.0f, 0.25f, 0.65f, 1.0f, 1.5f, 3.0f };
    const int32_t expectedDb  [6] = { 1000000, 1000, 200, 0, -200, -1000 };

    for (int i = 0; i < AI_HEARING_COUNT; ++i) {
        CHECK(kDefaultAIHearingStats.dist_muls[i] == expectedDist[i]);
        CHECK(kDefaultAIHearingStats.db_adds[i]   == expectedDb[i]);
    }

    // Deaf slot specifically — distance multiplier zero and a giant dB add,
    // so the AI cannot hear anything.
    CHECK(kDefaultAIHearingStats.dist_muls[AI_HEARING_DEAF] == 0.0f);
    CHECK(kDefaultAIHearingStats.db_adds[AI_HEARING_DEAF]   == 1000000);

    // Normal slot is the identity — distance unmodified, no dB add.
    CHECK(kDefaultAIHearingStats.dist_muls[AI_HEARING_NORMAL] == 1.0f);
    CHECK(kDefaultAIHearingStats.db_adds[AI_HEARING_NORMAL]   == 0);
}

// ════════════════════════════════════════════════════════════════════════════
// Name helpers — display strings for headless dumps / debug UI.
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("aiHearingRatingName returns expected strings", "[ai_hearing][parser]") {
    CHECK(std::strcmp(aiHearingRatingName(AI_HEARING_DEAF),      "Deaf")     == 0);
    CHECK(std::strcmp(aiHearingRatingName(AI_HEARING_VERY_LOW),  "VeryLow")  == 0);
    CHECK(std::strcmp(aiHearingRatingName(AI_HEARING_LOW),       "Low")      == 0);
    CHECK(std::strcmp(aiHearingRatingName(AI_HEARING_NORMAL),    "Normal")   == 0);
    CHECK(std::strcmp(aiHearingRatingName(AI_HEARING_HIGH),      "High")     == 0);
    CHECK(std::strcmp(aiHearingRatingName(AI_HEARING_VERY_HIGH), "VeryHigh") == 0);

    // Out-of-range.
    CHECK(std::strcmp(aiHearingRatingName(-1), "?") == 0);
    CHECK(std::strcmp(aiHearingRatingName(6),  "?") == 0);
    CHECK(std::strcmp(aiHearingRatingName(99), "?") == 0);
}

TEST_CASE("aiSoundTypeName returns expected strings", "[ai_hearing][parser]") {
    CHECK(std::strcmp(aiSoundTypeName(AI_SOUND_UNTYPED),         "Untyped")       == 0);
    CHECK(std::strcmp(aiSoundTypeName(AI_SOUND_INFORM),          "Inform")        == 0);
    CHECK(std::strcmp(aiSoundTypeName(AI_SOUND_MINOR_ANOMALY),   "MinorAnomaly")  == 0);
    CHECK(std::strcmp(aiSoundTypeName(AI_SOUND_MAJOR_ANOMALY),   "MajorAnomaly")  == 0);
    CHECK(std::strcmp(aiSoundTypeName(AI_SOUND_NON_COMBAT_HIGH), "NonCombatHigh") == 0);
    CHECK(std::strcmp(aiSoundTypeName(AI_SOUND_COMBAT),          "Combat")        == 0);

    // Out-of-range.
    CHECK(std::strcmp(aiSoundTypeName(-1), "?") == 0);
    CHECK(std::strcmp(aiSoundTypeName(6),  "?") == 0);
    CHECK(std::strcmp(aiSoundTypeName(99), "?") == 0);
}
