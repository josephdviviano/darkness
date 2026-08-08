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

// IEEE 754 binary16 ("half") conversion.
//
// bgfx/bx already ship these, and the renderer could use bx::halfFromFloat —
// but the lightmap bake also runs inside darknessHeadless, which deliberately
// links no rendering dependency at all. A twenty-line self-contained pair keeps
// that true; it is the same reason LightmapBake.h does its own atlas writing
// rather than calling into the renderer.
//
// Round-to-nearest-even in both directions, subnormals handled. The Catch2
// round-trip in tests/test_half_float.cpp pins the edge cases (subnormal
// boundary, the smallest normal, overflow to inf, NaN) — the lightmap atlas
// stores its entire dynamic range through here, so a quiet error at one
// exponent would be a band of wrong lumels nobody could trace back.

#pragma once

#include <cstdint>
#include <cstring>

namespace Darkness {

inline uint16_t floatToHalf(float f) {
    uint32_t x;
    std::memcpy(&x, &f, sizeof(x));
    const uint32_t sign = (x >> 16) & 0x8000u;
    const uint32_t rawExp = (x >> 23) & 0xFFu;
    uint32_t mant = x & 0x007FFFFFu;
    const int32_t exp = static_cast<int32_t>(rawExp) - 127 + 15;

    if (rawExp == 0xFFu)                       // inf / NaN
        return static_cast<uint16_t>(sign | 0x7C00u | (mant ? 0x0200u : 0u));
    if (exp >= 31)                             // overflows the half range
        return static_cast<uint16_t>(sign | 0x7C00u);
    if (exp <= 0) {                            // subnormal, or underflow to zero
        if (exp < -10) return static_cast<uint16_t>(sign);
        mant |= 0x00800000u;                   // restore the implicit leading 1
        const uint32_t shift = static_cast<uint32_t>(14 - exp);   // 11..24
        uint32_t h = mant >> shift;
        const uint32_t rem = mant & ((1u << shift) - 1u);
        const uint32_t halfway = 1u << (shift - 1);
        if (rem > halfway || (rem == halfway && (h & 1u))) ++h;
        return static_cast<uint16_t>(sign | h);
    }
    // Normal. A round that carries out of the mantissa lands in the exponent,
    // which is exactly right — 0x3FF+1 becomes the next exponent's 0.
    uint32_t h = (static_cast<uint32_t>(exp) << 10) | (mant >> 13);
    const uint32_t rem = mant & 0x1FFFu;
    if (rem > 0x1000u || (rem == 0x1000u && (h & 1u))) ++h;
    return static_cast<uint16_t>(sign | h);
}

inline float halfToFloatRef(uint16_t h) {
    const uint32_t sign = static_cast<uint32_t>(h & 0x8000u) << 16;
    const uint32_t exp = (h >> 10) & 0x1Fu;
    const uint32_t mant = h & 0x3FFu;
    uint32_t x;
    if (exp == 0) {
        if (mant == 0) {
            x = sign;                          // +-0
        } else {                               // subnormal: renormalise
            uint32_t e = 0, m = mant;
            while (!(m & 0x400u)) { m <<= 1; ++e; }
            m &= 0x3FFu;
            x = sign | ((127u - 15u - e + 1u) << 23) | (m << 13);
        }
    } else if (exp == 31) {
        x = sign | 0x7F800000u | (mant << 13); // inf / NaN
    } else {
        x = sign | ((exp + 127u - 15u) << 23) | (mant << 13);
    }
    float f;
    std::memcpy(&f, &x, sizeof(f));
    return f;
}



// Table-driven half->float. The reference above is branchy (and loops on
// subnormals), which is fine per-call and NOT fine in the lightmap blend:
// that inner loop runs once per channel per overlay per texel and measured
// 39-41 ms per door event on MISS6 — the dominant cost of the whole event,
// and a regression the half-float conversion introduced over the 8-bit
// `byte / 255.0f` it replaced.
//
// 65536 entries x 4 B = 256 KB, built once from halfToFloatRef so the two
// are identical BY CONSTRUCTION rather than by inspection; the exhaustive
// Catch2 round-trip covers the table because it calls this entry point.
namespace detail {
struct HalfToFloatTable {
    float v[65536];
    HalfToFloatTable() {
        for (uint32_t i = 0; i < 65536u; ++i)
            v[i] = halfToFloatRef(static_cast<uint16_t>(i));
    }
};
// A C++17 inline VARIABLE at namespace scope, deliberately not a
// function-local static: the latter is initialised on first use, which
// costs a thread-safe-init guard check on EVERY call, and in the lightmap
// blend's inner loop that guard cost more than the branchy conversion it
// was meant to replace (measured 132 ms/event vs 40 ms). Namespace-scope
// dynamic initialisation happens before main and the access is a plain
// indexed load.
inline const HalfToFloatTable kHalfToFloat;
} // namespace detail

inline float halfToFloat(uint16_t h) {
    return detail::kHalfToFloat.v[h];
}

} // namespace Darkness
