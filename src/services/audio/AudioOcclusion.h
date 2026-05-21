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

#ifndef __AUDIO_OCCLUSION_H
#define __AUDIO_OCCLUSION_H

#include "AudioUnits.h"

#include <algorithm>

/// @file AudioOcclusion.h
/// Tunable parameters for Steam Audio's volumetric occlusion (sphere
/// radius + ray sample count). Engine units are feet; getRadiusMeters()
/// converts when feeding IPLSimulationInputs. Setters/getters run on the
/// main thread only.

struct _IPLScene_t;
typedef _IPLScene_t* IPLScene;

namespace Darkness {

class AudioOcclusion {
public:
    explicit AudioOcclusion(IPLScene scene = nullptr);
    ~AudioOcclusion();

    void setScene(IPLScene scene) { mScene = scene; }
    IPLScene getScene() const { return mScene; }

    /// Sphere radius (engine feet). Clamp [0.1, 200] — wide enough for
    /// large environmental sources without the setter swallowing the value.
    void setRadius(float r) {
        mRadius = std::max(0.1f, std::min(r, 200.0f));
    }
    float getRadius() const { return mRadius; }
    float getRadiusMeters() const { return mRadius * kFeetToMeters; }

    /// Number of ray samples. Clamp [4, 64] — below 4 the model
    /// degrades to a single raycast and pops; above 64 the per-source
    /// cost dominates the direct simulator budget.
    void setSamples(int n) {
        mSamples = std::max(4, std::min(n, 64));
    }
    int getSamples() const { return mSamples; }

private:
    IPLScene mScene = nullptr;
    float mRadius = 10.0f;  // engine feet
    int mSamples = 16;
};

} // namespace Darkness

#endif // __AUDIO_OCCLUSION_H
