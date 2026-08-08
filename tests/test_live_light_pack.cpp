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
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *****************************************************************************/

// The S4c differential slot transport (LiveLightPack.h) — the one float
// that carries a promoted light's frozen + current shadow-pool slots to
// live_lights.sh.
//
// This exists because the encoding silently outgrew its radix once: it was
// `100 + frozen*16 + current`, written when the pool held 16 slots, and the
// pool later grew to 66. Every promoted light whose CURRENT slot landed at
// 16 or above then decoded to a different pair, so the shader differenced
// two unrelated lights' shadow faces. It was invisible at rest — a settled
// door carries no differential term at all — and destroyed the lightmap for
// the whole duration of a door swing.
//
// So the test sweeps the WHOLE legal slot range rather than a few examples:
// an off-by-one in the radix must fail here, not in a recording.

#include <catch2/catch_test_macros.hpp>

#include "../src/main/LiveLightPack.h"

using namespace Darkness;

// Kept in sync with PostProcess.h's kShadowMaxPoolSlots by the
// static_assert in ShadowMapCache.h; duplicated here as a plain number so
// this test needs no bgfx.
static constexpr int kPoolSlotsUnderTest = 66;

TEST_CASE("live light pack: every legal slot pair round-trips",
          "[livelight]") {
    for (int frozen = 0; frozen < kPoolSlotsUnderTest; ++frozen) {
        for (int current = 0; current < kPoolSlotsUnderTest; ++current) {
            const float packed = packLiveDiffSlots(frozen, current);
            int df = -1, dc = -1;
            unpackLiveDiffSlots(packed, df, dc);
            INFO("frozen=" << frozen << " current=" << current
                           << " packed=" << packed);
            REQUIRE(df == frozen);
            REQUIRE(dc == current);
        }
    }
}

TEST_CASE("live light pack: the radix covers the pool", "[livelight]") {
    // The failure this guards is exactly the shipped bug: a current slot
    // at or above the radix borrows into the frozen field.
    REQUIRE(kLiveSlotPackBase >= kPoolSlotsUnderTest);
}

TEST_CASE("live light pack: differential band never collides with a "
          "plain slot", "[livelight]") {
    // A non-differential light sends its raw slot index (or a negative for
    // unshadowed). The shader tells the two apart by magnitude alone, so
    // no plain slot may reach the band.
    for (int slot = 0; slot < kPoolSlotsUnderTest; ++slot) {
        INFO("plain slot " << slot);
        REQUIRE_FALSE(isLiveDiffPacked(static_cast<float>(slot)));
    }
    REQUIRE_FALSE(isLiveDiffPacked(-1.0f));
    REQUIRE(isLiveDiffPacked(packLiveDiffSlots(0, 0)));
    REQUIRE(isLiveDiffPacked(
        packLiveDiffSlots(kPoolSlotsUnderTest - 1,
                          kPoolSlotsUnderTest - 1)));
}

TEST_CASE("live light pack: packed values stay exact in fp32",
          "[livelight]") {
    // The decode does floor(t / base) in float. Integers are exact in fp32
    // to 2^24; assert the largest packed value has a wide margin, so the
    // floor can never land a slot off by one.
    const float maxPacked = packLiveDiffSlots(kPoolSlotsUnderTest - 1,
                                              kPoolSlotsUnderTest - 1);
    REQUIRE(maxPacked < 16777216.0f / 1000.0f);
    // And that adding 1 is still distinguishable at that magnitude.
    REQUIRE(maxPacked + 1.0f != maxPacked);
}
