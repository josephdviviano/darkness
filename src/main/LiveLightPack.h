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

// The S4c differential slot transport: how a promoted light's TWO S1
// shadow-pool slots (frozen reference + current pose) travel to the shader
// in the single float `u_liveLightColor[i].w`.
//
// THIS IS A C++<->GLSL MIRROR. The decode in `shaders/live_lights.sh`
// (liveLightSum) must match `unpackLiveDiffSlots` below exactly, and both
// are pinned by tests/test_live_light_pack.cpp. Edit the two ends together.
//
// WHY THE BASE IS A NAMED CONSTANT WITH A static_assert:
// the original encoding was `100 + frozen*16 + current`, correct while the
// shadow pool held 16 slots. The pool later grew to 66
// (PostProcess.h kShadowMaxPoolSlots) and the encoding was not revisited,
// so any light whose CURRENT slot landed at 16 or above decoded to a
// different pair entirely --
//   pack(F, C) = 100 + 16F + C  ->  decode gives (F + C/16, C mod 16)
// -- i.e. the shader differenced two unrelated lights' shadow faces. That
// is invisible at rest (a settled door has no differential term at all)
// and destroys the lightmap mid-swing, which is exactly the shape of the
// bug this constant now prevents: the static_assert fails the BUILD if the
// pool ever outgrows the encoding again.
//
// Magnitude check: the largest packed value is
// 100 + 128*(kShadowMaxPoolSlots-1) + (kShadowMaxPoolSlots-1) = 8485 at 66
// slots. Integers are exact in fp32 to 2^24, and every backend the project
// targets (D3D11/12, Vulkan, Metal, GL core) carries `float` uniforms at
// fp32 -- so the decode's floor() is exact with a wide margin. Keep the
// base a power of two so the division stays exact too.

// This header is deliberately dependency-free (no bgfx, no PostProcess.h)
// so the round-trip test can pin it directly. The cross-check that the
// shadow pool still FITS the packing lives in ShadowMapCache.h, where
// kShadowMaxPoolSlots is already in scope.

#pragma once

#include <cmath>

namespace Darkness {

// Mirrors LIVE_LIGHT_CAP in shaders/live_lights.sh.
// 32 = the upper industry-standard shadowed-light budget (rationale at
// PostProcess.h kShadowMaxPoolSlots, which is sized to match: 2 slots
// per differential + live emitters). The shader loop is count-gated, so
// inactive entries cost one compare each.
constexpr int kLiveLightCap = 32;

// Values at or above this mark the entry as a DIFFERENTIAL pair rather
// than a plain single slot. Plain slots are the raw slot index (>= 0) or a
// negative for "unshadowed", so the differential band starts above any
// slot index the pool can produce.
constexpr float kLiveDiffPackBias = 100.0f;

// Radix of the two-slot packing. Must exceed the largest slot index.
constexpr int kLiveSlotPackBase = 128;

// Pack a (frozen, current) slot pair for u_liveLightColor[i].w.
inline float packLiveDiffSlots(int slotFrozen, int slotCurrent) {
    return kLiveDiffPackBias +
           static_cast<float>(kLiveSlotPackBase) *
               static_cast<float>(slotFrozen) +
           static_cast<float>(slotCurrent);
}

// The GLSL decode, in the same arithmetic the shader uses (float divide +
// floor), so a round-trip test proves what the GPU will actually compute
// rather than what integer math would.
inline void unpackLiveDiffSlots(float packed, int &slotFrozen,
                                int &slotCurrent) {
    const float t = packed - kLiveDiffPackBias;
    const float base = static_cast<float>(kLiveSlotPackBase);
    const float f = std::floor(t / base);
    slotFrozen = static_cast<int>(f);
    slotCurrent = static_cast<int>(t - f * base);
}

// True when a packed .w carries a differential pair. Mirrors the shader's
// `sw >= 99.5` test (a half-unit guard band against float drift).
inline bool isLiveDiffPacked(float packed) {
    return packed >= kLiveDiffPackBias - 0.5f;
}

}   // namespace Darkness
