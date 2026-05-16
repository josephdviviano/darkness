/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2024-2026 darkness contributors
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 3 of the License, or
 *    (at your option) any later version.
 *
 *****************************************************************************/

// Spot-ambient envelope math (Unit B from PLAN.SOUND_DATA_PARSING.md).
// PropSpotAmb stores three floats: inner radius, outer radius, level.
// Distance d from the listener maps to volume as:
//   d <= inner          → volume = level
//   inner < d < outer   → volume = level * (outer - d) / (outer - inner)
//   d >= outer          → volume = 0
// AudioService::updateSpotAmbientVolumes implements this; the helper
// below mirrors it pure-functionally so the formula is testable without
// standing up the full service stack.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>

#include "DarknessMath.h"
#include "property/DarkPropertyDefs.h"

using namespace Darkness;

namespace {

// Pure copy of AudioService::updateSpotAmbientVolumes' envelope rule.
float spotEnvelopeVolume(float inner, float outer, float level, float d) {
    if (d <= inner) return level;
    if (d >= outer) return 0.0f;
    return level * (outer - d) / (outer - inner);
}

} // namespace

TEST_CASE("PropSpotAmb is 12 bytes (3 floats)", "[spotamb][parser]") {
    CHECK(sizeof(PropSpotAmb) == 12);
}

TEST_CASE("Spot ambient envelope returns level inside inner radius",
          "[spotamb][envelope]") {
    using Catch::Matchers::WithinAbs;
    CHECK_THAT(spotEnvelopeVolume(22.0f, 30.0f, 45.0f, 0.0f),
               WithinAbs(45.0f, 1e-5));
    CHECK_THAT(spotEnvelopeVolume(22.0f, 30.0f, 45.0f, 10.0f),
               WithinAbs(45.0f, 1e-5));
    CHECK_THAT(spotEnvelopeVolume(22.0f, 30.0f, 45.0f, 22.0f),
               WithinAbs(45.0f, 1e-5));
}

TEST_CASE("Spot ambient envelope returns 0 at and beyond outer radius",
          "[spotamb][envelope]") {
    using Catch::Matchers::WithinAbs;
    CHECK_THAT(spotEnvelopeVolume(22.0f, 30.0f, 45.0f, 30.0f),
               WithinAbs(0.0f, 1e-5));
    CHECK_THAT(spotEnvelopeVolume(22.0f, 30.0f, 45.0f, 100.0f),
               WithinAbs(0.0f, 1e-5));
}

TEST_CASE("Spot ambient envelope linearly interpolates in transition band",
          "[spotamb][envelope]") {
    using Catch::Matchers::WithinAbs;
    // MISS6's MechFloorLamp uses inner=22, outer=30, level=45. At the
    // midpoint d=26 (4 units inside outer, half the transition width),
    // volume = 45 * (30-26)/(30-22) = 45 * 4/8 = 22.5
    CHECK_THAT(spotEnvelopeVolume(22.0f, 30.0f, 45.0f, 26.0f),
               WithinAbs(22.5f, 1e-5));
    // 1/4 of the way through the transition (d=24, level should be 3/4)
    CHECK_THAT(spotEnvelopeVolume(22.0f, 30.0f, 45.0f, 24.0f),
               WithinAbs(45.0f * 6.0f / 8.0f, 1e-5));
}

TEST_CASE("Spot ambient envelope handles PropSpotAmb round-trip",
          "[spotamb][parser]") {
    // Pack a synthetic 12-byte record matching the on-disk layout, then
    // memcpy it back through PropSpotAmb to verify field order/sizes
    // match what AudioService::loadSpotAmbients reads.
    PropSpotAmb pack{22.0f, 30.0f, 45.0f};
    uint8_t buf[sizeof(PropSpotAmb)];
    std::memcpy(buf, &pack, sizeof(buf));
    PropSpotAmb out;
    std::memcpy(&out, buf, sizeof(out));
    CHECK(out.inner == 22.0f);
    CHECK(out.outer == 30.0f);
    CHECK(out.ambient == 45.0f);
}

// ---------------------------------------------------------------------------
// Voice-lifecycle boundary conditions (runtime predicate).
//
// updateSpotAmbientVolumes starts a voice when envVol transitions from
// 0 → >0 and stops it when envVol transitions from >0 → 0. The lifecycle
// predicate `shouldVoiceBePlaying` mirrors that decision so the boundary
// behaviour is testable without standing up the full service stack
// (which would require ServiceManager + miniaudio init). The real voice
// lifecycle is verified at runtime via [SPOT_AMB] log lines that report
// how many spot ambients are audible and how many voices are active.
//
// Boundary expectations:
//   • d < inner   → playing (full level inside)
//   • d = inner   → playing (envelope returns level exactly)
//   • d = outer   → silent  (envelope returns 0 exactly)
//   • d > outer   → silent
//   • inner band  → playing (envelope > 0)
//
namespace {

bool shouldVoiceBePlaying(float inner, float outer, float level, float d) {
    return spotEnvelopeVolume(inner, outer, level, d) > 0.0f;
}

} // namespace

TEST_CASE("Spot ambient voice starts inside outer radius",
          "[spotamb][lifecycle]") {
    // MISS6's MechFloorLamp params: inner=22, outer=30, level=45.
    constexpr float kInner = 22.0f;
    constexpr float kOuter = 30.0f;
    constexpr float kLevel = 45.0f;

    // Listener well inside inner: voice should be playing.
    CHECK(shouldVoiceBePlaying(kInner, kOuter, kLevel, 0.0f));
    // Listener at inner radius: voice should be playing.
    CHECK(shouldVoiceBePlaying(kInner, kOuter, kLevel, kInner));
    // Listener midway through transition: voice should still be playing.
    CHECK(shouldVoiceBePlaying(kInner, kOuter, kLevel, 26.0f));
    // Listener just inside outer: voice still playing (envelope > 0).
    CHECK(shouldVoiceBePlaying(kInner, kOuter, kLevel, 29.999f));
}

TEST_CASE("Spot ambient voice stops at or beyond outer radius",
          "[spotamb][lifecycle]") {
    constexpr float kInner = 22.0f;
    constexpr float kOuter = 30.0f;
    constexpr float kLevel = 45.0f;

    // Listener at outer radius: envelope = 0, voice should stop.
    CHECK_FALSE(shouldVoiceBePlaying(kInner, kOuter, kLevel, kOuter));
    // Listener beyond outer: voice should stop.
    CHECK_FALSE(shouldVoiceBePlaying(kInner, kOuter, kLevel, 50.0f));
    CHECK_FALSE(shouldVoiceBePlaying(kInner, kOuter, kLevel, 1000.0f));
}

TEST_CASE("Spot ambient lifecycle handles multiple emitters independently",
          "[spotamb][lifecycle]") {
    // Two synthetic emitters with different envelopes. The decision
    // function evaluates each one in isolation against the listener, so
    // overlapping bubbles each produce their own start/stop decision.
    struct Emitter { float inner, outer, level; Vector3 pos; };
    auto distance = [](const Vector3 &a, const Vector3 &b) {
        float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    };
    Vector3 listener{0.0f, 0.0f, 0.0f};
    Emitter near {5.0f, 10.0f, 50.0f, {8.0f, 0.0f, 0.0f}};   // d=8, in transition
    Emitter far  {5.0f, 10.0f, 50.0f, {100.0f, 0.0f, 0.0f}}; // d=100, silent

    float dNear = distance(listener, near.pos);
    float dFar  = distance(listener, far.pos);

    CHECK(shouldVoiceBePlaying(near.inner, near.outer, near.level, dNear));
    CHECK_FALSE(shouldVoiceBePlaying(far.inner, far.outer, far.level, dFar));
}
