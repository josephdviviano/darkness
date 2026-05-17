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

/// @file AudioOcclusion.h
/// Volumetric occlusion configuration for per-voice direct-path simulation.
///
/// Extracted from AudioService — owns the tunable parameters that drive
/// Steam Audio's volumetric occlusion model (sphere radius + ray sample
/// count). The actual ray-cast sampling runs inside Steam Audio's direct
/// simulator using these inputs; this module owns the values and provides
/// the IPL-ready conversions (engine feet → meters) and the radius
/// override used for door / local-sound voices that need a wider sphere
/// to keep narrow doorframes from over-occluding.
///
/// Public AudioService API (setOcclusionRadius / setOcclusionSamples and
/// their getters) is preserved as a facade on top of this class so
/// existing callers in RenderConfig and DebugConsole need no changes.
///
/// Threading: setters / getters are called only from the main thread
/// (config loader + per-frame loopStep). Values are plain floats / ints,
/// not atomics — the audio thread reads occlusion inputs through
/// IPL simulator state populated on the main thread, not directly here.

#include <algorithm>

// Forward declarations for Steam Audio opaque handles. Kept here so
// future callers wanting to push inputs directly through this class
// don't need to repeat the typedef dance.
struct _IPLScene_t;
typedef _IPLScene_t* IPLScene;

namespace Darkness {

/// Tunable parameters for Steam Audio's volumetric occlusion model.
///
/// Volumetric occlusion samples N points from a sphere centered on the
/// source and casts rays from each to the listener. Fraction of unblocked
/// rays = unoccluded fraction; larger radius = smoother transitions
/// around corners, smaller = tighter response.
class AudioOcclusion {
public:
    /// Construct with the acoustic scene the IPL simulator is using.
    /// We don't own the scene — it's released by AudioService via
    /// destroyAcousticScene(). The pointer is kept here so a future
    /// extension (e.g. an out-of-band ray-cast for systems that don't
    /// have an IPL simulator source per voice) has the handle ready.
    explicit AudioOcclusion(IPLScene scene = nullptr);
    ~AudioOcclusion();

    /// The acoustic scene may be rebuilt across missions — AudioService
    /// calls this whenever it commits a new IPLScene.
    void setScene(IPLScene scene) { mScene = scene; }
    IPLScene getScene() const { return mScene; }

    // ── Sphere radius (engine feet) ──
    //
    // Stored in engine feet to match the rest of the engine; convert to
    // meters via getRadiusMeters() when pushing to Steam Audio. The clamp
    // bounds [0.1, 200] were widened from the original [0.3, 30] so
    // designers can dial in very wide spheres for big environmental
    // sources (large machinery, fountains) without the setter swallowing
    // the value.
    void setRadius(float r) {
        mRadius = std::max(0.1f, std::min(r, 200.0f));
    }
    float getRadius() const { return mRadius; }

    /// Engine feet → meters conversion (kFeetToMeters = 0.3048).
    /// Use this when writing IPLSimulationInputs::occlusionRadius.
    float getRadiusMeters() const {
        constexpr float kFeetToMeters = 0.3048f;
        return mRadius * kFeetToMeters;
    }

    // ── Sample count ──
    //
    // More samples = smoother gradient, higher CPU cost per source.
    // Clamp [4, 64] matches the original Dark Engine port: below 4
    // samples the volumetric model degrades to a single raycast and
    // produces popping; above 64 the per-source cost dominates the
    // direct simulator budget without audible improvement.
    void setSamples(int n) {
        mSamples = std::max(4, std::min(n, 64));
    }
    int getSamples() const { return mSamples; }

private:
    IPLScene mScene = nullptr;

    /// Volumetric occlusion source sphere radius (engine feet).
    /// Default 10 ft = small machine.
    float mRadius = 10.0f;

    /// Number of ray samples for volumetric occlusion (4-64).
    int mSamples = 16;
};

} // namespace Darkness

#endif // __AUDIO_OCCLUSION_H
