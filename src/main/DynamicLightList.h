/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2025 darkness contributors
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

// Per-frame dynamic light registry. Gameplay systems (player flashlight,
// fire arrows, mage spells, creature glow) call `add()` each frame to
// register transient lights; the per-object illumination pass iterates
// these against every visible object with a fresh portal raycast (no
// cache, since the lights move every frame).
//
// Dynamic lights are independent from the static light table and the
// per-cell light_indices — they are not bake-time data.
//
// Dark Engine convention: ~32 simultaneous dynamic lights total. We keep
// the same upper bound; gameplay systems that exceed it would have logged
// "Out of Dynamic Lights" in the original engine.

#pragma once

#include "DarknessMath.h"

#include <cstdint>
#include <vector>

namespace Darkness {

struct DynamicLight {
    Vector3 loc;
    Vector3 bright;        // RGB; same scaling convention as static lights
    float   radius;        // 0 = no cutoff (purely 1/r falloff)
};

class DynamicLightList {
public:
    // Hard cap, matches the engine's `MAX_DYNAMIC = 32` — sized to the
    // shadow-bit storage in the original. Excess registrations are dropped
    // with a one-time warning.
    static constexpr int kMaxDynamicLights = 32;

    // Called once per frame at the start of simulate(). Drops everything
    // registered the previous frame.
    void reset() { mLights.clear(); }

    // Register a dynamic light for this frame. `bright` is RGB and follows
    // the same convention as static lights — typical values 0..tens, with a
    // 1/r distance attenuation applied during the per-object compute.
    // Calls beyond kMaxDynamicLights are silently dropped (avoiding spam).
    void add(const Vector3 &pos, const Vector3 &bright, float radius = 0.0f) {
        if (static_cast<int>(mLights.size()) >= kMaxDynamicLights) return;
        mLights.push_back({ pos, bright, radius });
    }

    int  count() const                     { return static_cast<int>(mLights.size()); }
    const DynamicLight &operator[](int i) const { return mLights[i]; }

private:
    std::vector<DynamicLight> mLights;
};

} // namespace Darkness
