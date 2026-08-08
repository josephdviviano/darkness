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

// Half-float round-trip. The lightmap atlas stores its whole dynamic range
// through these two functions, so an error at one exponent would be a band of
// wrong lumels with no other symptom.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "HalfFloat.h"

#include <cmath>
#include <cstdint>
#include <limits>

using Darkness::floatToHalf;
using Darkness::halfToFloat;

TEST_CASE("half round-trip is exact for every representable half", "[half]") {
    // Every finite half must survive half -> float -> half unchanged. This is
    // the property the atlas depends on: a blend that reads, adds zero and
    // writes back must not drift.
    int checked = 0;
    for (uint32_t bits = 0; bits <= 0xFFFFu; ++bits) {
        const uint16_t h = static_cast<uint16_t>(bits);
        const uint32_t exp = (h >> 10) & 0x1Fu;
        if (exp == 31) continue;              // inf/NaN handled separately
        const float f = halfToFloat(h);
        REQUIRE(floatToHalf(f) == h);
        ++checked;
    }
    REQUIRE(checked == 63488);                 // 65536 - 2048 inf/NaN encodings
}

TEST_CASE("half boundary values decode to the exact IEEE values", "[half]") {
    // Smallest normal, largest finite, smallest subnormal — the three places a
    // hand-written converter goes wrong.
    CHECK(halfToFloat(0x0400) == std::ldexp(1.0f, -14));       // 6.103515625e-5
    CHECK(halfToFloat(0x0001) == std::ldexp(1.0f, -24));       // 5.9604645e-8
    CHECK(halfToFloat(0x03FF) == std::ldexp(1023.0f, -24));    // largest subnormal
    CHECK(halfToFloat(0x7BFF) == 65504.0f);                    // largest finite
    CHECK(halfToFloat(0x0000) == 0.0f);
    CHECK(halfToFloat(0x3C00) == 1.0f);

    CHECK(floatToHalf(std::ldexp(1.0f, -14)) == 0x0400);
    CHECK(floatToHalf(std::ldexp(1.0f, -24)) == 0x0001);
    CHECK(floatToHalf(65504.0f) == 0x7BFF);
    CHECK(floatToHalf(1.0f) == 0x3C00);
    CHECK(floatToHalf(0.0f) == 0x0000);
}

TEST_CASE("half saturates rather than wrapping", "[half]") {
    // Dropping the atlas's 0..1 clamp means bright lumels can exceed 1; they
    // must never wrap to a small or negative value on the way to the GPU.
    CHECK(halfToFloat(floatToHalf(70000.0f)) == std::numeric_limits<float>::infinity());
    CHECK(halfToFloat(floatToHalf(-70000.0f)) == -std::numeric_limits<float>::infinity());
    CHECK(std::isnan(halfToFloat(floatToHalf(std::nanf("")))));
    // Underflow goes to zero, not to a wrapped exponent.
    CHECK(halfToFloat(floatToHalf(std::ldexp(1.0f, -30))) == 0.0f);
}

TEST_CASE("half resolves the band where the lightmap actually lives", "[half]") {
    // Measured on MISS6: 82% of lit texels sit between 13/255 and 20/255. In
    // RGBA8 that band is 8 codes. This is the number the format change exists
    // to move, so it is pinned rather than described.
    const float lo = 13.0f / 255.0f, hi = 20.0f / 255.0f;
    int distinct = 0;
    uint16_t prev = floatToHalf(lo);
    for (uint16_t h = prev; halfToFloat(h) < hi; ++h) {
        if (h != prev) ++distinct;
        prev = h;
    }
    CHECK(distinct > 500);

    // And relative precision there is far below one 8-bit count.
    const float v = 14.0f / 255.0f;
    const float step = halfToFloat(static_cast<uint16_t>(floatToHalf(v) + 1))
                     - halfToFloat(floatToHalf(v));
    CHECK(step / v < 0.001f);                  // 8-bit is 7.1% at this level
}

TEST_CASE("half preserves lightmap values through a store/load cycle", "[half]") {
    // Realistic lumel magnitudes, including the sub-quantisation tail the
    // physical falloff produces and the >1 values the clamp used to eat.
    const float values[] = {0.0f,    1e-4f,  0.0051f, 0.0157f, 0.055f,
                            0.0784f, 0.25f,  0.5f,    0.941f,  1.0f,
                            1.7f,    4.25f,  17.0f};
    for (float v : values) {
        const float back = halfToFloat(floatToHalf(v));
        if (v == 0.0f) {
            CHECK(back == 0.0f);
        } else {
            CHECK_THAT(back, Catch::Matchers::WithinRel(v, 0.001f));
        }
    }
}
