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

#include <cstdint>
#include <cstring>

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
