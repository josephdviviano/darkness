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

// Tests for AIHearingService::isAudible — pure free-function range math.
//
// The full service requires AudioService + the gamesys to be loaded, which
// is heavy for a unit test. Instead, the service's audibility decision is
// extracted into a static helper that takes the global tables, the AI's
// rating, and the emission's (soundType, baseRange, distance) directly.
// These tests exercise that helper end-to-end and pin the per-rating
// dist-multiplier semantics that the original engine baked into the
// AIHearStat chunk.

#include <catch2/catch_test_macros.hpp>

#include <cstdint>

#include "ai/AIHearingService.h"
#include "audio/AIHearingData.h"

using namespace Darkness;

namespace {

// A custom AIHearingStats table designed for round-number assertions.
// dist_muls: Deaf=0x, VeryLow=0.5x, Low=0.75x, Normal=1x, High=2x, VeryHigh=3x.
// db_adds: unused by the first-pass distance check, but populated so the
// struct is fully initialized.
constexpr AIHearingStats kStatsForTest = {
    /* dist_muls */ { 0.0f, 0.5f, 0.75f, 1.0f, 2.0f, 3.0f },
    /* db_adds   */ {  1000000,  500, 200,    0,  -200, -500 },
};

// Zero-tweak table — no sound-type bonus. Many ranges in this test are
// large enough that the bonus would dominate; zeroing it isolates the
// per-rating multiplier behavior.
constexpr AISoundTweaks kZeroTweaks = { { 0, 0, 0, 0, 0, 0 } };

} // anonymous namespace

// ════════════════════════════════════════════════════════════════════════════
// Per-rating distance multiplier — the table's contract.
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Normal rating preserves base range exactly",
          "[ai_hearing][service]") {
    const float baseRange = 100.0f;

    // At rating Normal, dist_muls = 1.0, so effective_range == baseRange.
    // A distance just inside is audible, just outside is not.
    CHECK(AIHearingService::isAudible(kStatsForTest, kZeroTweaks,
                                      AI_HEARING_NORMAL, AI_SOUND_UNTYPED,
                                      baseRange, 99.0f));
    CHECK(AIHearingService::isAudible(kStatsForTest, kZeroTweaks,
                                      AI_HEARING_NORMAL, AI_SOUND_UNTYPED,
                                      baseRange, 100.0f));
    CHECK_FALSE(AIHearingService::isAudible(kStatsForTest, kZeroTweaks,
                                            AI_HEARING_NORMAL, AI_SOUND_UNTYPED,
                                            baseRange, 100.1f));
}

TEST_CASE("VeryHigh rating extends range to 3x base",
          "[ai_hearing][service]") {
    const float baseRange = 100.0f;
    // dist_muls[VERY_HIGH] = 3.0, so the AI hears out to 300 units.
    CHECK(AIHearingService::isAudible(kStatsForTest, kZeroTweaks,
                                      AI_HEARING_VERY_HIGH, AI_SOUND_UNTYPED,
                                      baseRange, 299.9f));
    CHECK(AIHearingService::isAudible(kStatsForTest, kZeroTweaks,
                                      AI_HEARING_VERY_HIGH, AI_SOUND_UNTYPED,
                                      baseRange, 300.0f));
    CHECK_FALSE(AIHearingService::isAudible(kStatsForTest, kZeroTweaks,
                                            AI_HEARING_VERY_HIGH, AI_SOUND_UNTYPED,
                                            baseRange, 300.1f));
    // And — out past the Normal rating's range but well within VeryHigh.
    CHECK(AIHearingService::isAudible(kStatsForTest, kZeroTweaks,
                                      AI_HEARING_VERY_HIGH, AI_SOUND_UNTYPED,
                                      baseRange, 250.0f));
}

TEST_CASE("Deaf rating cannot hear anything (range = 0)",
          "[ai_hearing][service]") {
    const float baseRange = 100.0f;
    // dist_muls[DEAF] = 0, so even d=0 should be a miss (we treat Deaf as
    // a special case in the helper — never audible regardless of range).
    CHECK_FALSE(AIHearingService::isAudible(kStatsForTest, kZeroTweaks,
                                            AI_HEARING_DEAF, AI_SOUND_UNTYPED,
                                            baseRange, 0.0f));
    CHECK_FALSE(AIHearingService::isAudible(kStatsForTest, kZeroTweaks,
                                            AI_HEARING_DEAF, AI_SOUND_UNTYPED,
                                            baseRange, 50.0f));
    CHECK_FALSE(AIHearingService::isAudible(kStatsForTest, kZeroTweaks,
                                            AI_HEARING_DEAF, AI_SOUND_COMBAT,
                                            baseRange, 0.0f));
}

TEST_CASE("Low rating attenuates range to 0.75x base",
          "[ai_hearing][service]") {
    const float baseRange = 200.0f;
    // dist_muls[LOW] = 0.75 → 150 ft.
    CHECK(AIHearingService::isAudible(kStatsForTest, kZeroTweaks,
                                      AI_HEARING_LOW, AI_SOUND_UNTYPED,
                                      baseRange, 149.0f));
    CHECK_FALSE(AIHearingService::isAudible(kStatsForTest, kZeroTweaks,
                                            AI_HEARING_LOW, AI_SOUND_UNTYPED,
                                            baseRange, 151.0f));
}

// ════════════════════════════════════════════════════════════════════════════
// Sound-type bonus — additive on top of the rating-scaled range.
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Sound-type default range adds on top of rating-scaled range",
          "[ai_hearing][service]") {
    // Combat carries 50 extra units beyond the rating-scaled range.
    AISoundTweaks tweaks = { { 0, 0, 0, 0, 0, 50 } };

    const float baseRange = 100.0f;
    // Normal rating: 100 base + 50 combat bonus = 150 effective.
    CHECK(AIHearingService::isAudible(kStatsForTest, tweaks,
                                      AI_HEARING_NORMAL, AI_SOUND_COMBAT,
                                      baseRange, 149.0f));
    CHECK_FALSE(AIHearingService::isAudible(kStatsForTest, tweaks,
                                            AI_HEARING_NORMAL, AI_SOUND_COMBAT,
                                            baseRange, 150.5f));
    // Untyped slot has 0 bonus — back to the unmodified 100.
    CHECK_FALSE(AIHearingService::isAudible(kStatsForTest, tweaks,
                                            AI_HEARING_NORMAL, AI_SOUND_UNTYPED,
                                            baseRange, 101.0f));
}

TEST_CASE("Negative sound-type tweaks are clamped to zero",
          "[ai_hearing][service]") {
    // Hypothetical negative tweak — never seen in the shipping AISNDTWK
    // chunk, but the helper should still be robust.
    AISoundTweaks tweaks = { { -1000, 0, 0, 0, 0, 0 } };

    const float baseRange = 100.0f;
    // Even with a -1000 type bonus, the effective range stays at 100.
    CHECK(AIHearingService::isAudible(kStatsForTest, tweaks,
                                      AI_HEARING_NORMAL, AI_SOUND_UNTYPED,
                                      baseRange, 99.0f));
    CHECK_FALSE(AIHearingService::isAudible(kStatsForTest, tweaks,
                                            AI_HEARING_NORMAL, AI_SOUND_UNTYPED,
                                            baseRange, 200.0f));
}

// ════════════════════════════════════════════════════════════════════════════
// Out-of-range inputs — defensive clamping.
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Out-of-range rating slot returns false",
          "[ai_hearing][service]") {
    const float baseRange = 100.0f;
    CHECK_FALSE(AIHearingService::isAudible(kStatsForTest, kZeroTweaks,
                                            -1, AI_SOUND_UNTYPED,
                                            baseRange, 1.0f));
    CHECK_FALSE(AIHearingService::isAudible(kStatsForTest, kZeroTweaks,
                                            AI_HEARING_COUNT, AI_SOUND_UNTYPED,
                                            baseRange, 1.0f));
    CHECK_FALSE(AIHearingService::isAudible(kStatsForTest, kZeroTweaks,
                                            999, AI_SOUND_UNTYPED,
                                            baseRange, 1.0f));
}

TEST_CASE("Out-of-range sound type falls back to slot 0",
          "[ai_hearing][service]") {
    AISoundTweaks tweaks = { { 0, 999, 999, 999, 999, 999 } };

    const float baseRange = 100.0f;
    // soundType=42 is out of range; helper falls back to slot 0 (bonus=0),
    // so the effective range is just baseRange * Normal dist_mul = 100.
    CHECK(AIHearingService::isAudible(kStatsForTest, tweaks,
                                      AI_HEARING_NORMAL, 42,
                                      baseRange, 99.0f));
    CHECK_FALSE(AIHearingService::isAudible(kStatsForTest, tweaks,
                                            AI_HEARING_NORMAL, 42,
                                            baseRange, 101.0f));
}

// ════════════════════════════════════════════════════════════════════════════
// Use the engine's default table — sanity check against the values the
// gamesys ships with.
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("kDefaultAIHearingStats has expected per-rating reach",
          "[ai_hearing][service]") {
    AISoundTweaks zero = { { 0, 0, 0, 0, 0, 0 } };
    const float baseRange = 100.0f;

    // Defaults: Deaf=0, VeryLow=0.25, Low=0.65, Normal=1.0, High=1.5, VeryHigh=3.0.
    CHECK_FALSE(AIHearingService::isAudible(kDefaultAIHearingStats, zero,
                                            AI_HEARING_DEAF, AI_SOUND_UNTYPED,
                                            baseRange, 0.5f));
    CHECK(AIHearingService::isAudible(kDefaultAIHearingStats, zero,
                                      AI_HEARING_VERY_LOW, AI_SOUND_UNTYPED,
                                      baseRange, 24.0f));
    CHECK_FALSE(AIHearingService::isAudible(kDefaultAIHearingStats, zero,
                                            AI_HEARING_VERY_LOW, AI_SOUND_UNTYPED,
                                            baseRange, 26.0f));
    CHECK(AIHearingService::isAudible(kDefaultAIHearingStats, zero,
                                      AI_HEARING_HIGH, AI_SOUND_UNTYPED,
                                      baseRange, 149.0f));
    CHECK_FALSE(AIHearingService::isAudible(kDefaultAIHearingStats, zero,
                                            AI_HEARING_HIGH, AI_SOUND_UNTYPED,
                                            baseRange, 151.0f));
    CHECK(AIHearingService::isAudible(kDefaultAIHearingStats, zero,
                                      AI_HEARING_VERY_HIGH, AI_SOUND_UNTYPED,
                                      baseRange, 299.0f));
    CHECK_FALSE(AIHearingService::isAudible(kDefaultAIHearingStats, zero,
                                            AI_HEARING_VERY_HIGH, AI_SOUND_UNTYPED,
                                            baseRange, 301.0f));
}
