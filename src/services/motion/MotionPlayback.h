/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2005-2009 openDarkEngine team
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

// MotionPlayback.h — Per-entity motion playback state
//
// Separated from MotionService so each consumer (player, future NPCs) owns
// its own playback instance with independent timing and flag callbacks.
//
// Architecture:
//   MotionService   — global clip database (read-only after load)
//   MotionPlayback  — per-entity mutable state (this file)
//
// The playback tracks:
//   - Current clip reference (const, from MotionService)
//   - Elapsed time within clip, with timeWarp speed scaling
//   - Flag emission (footfalls, triggers) via callback
//
// TimeWarp maps the player's actual movement speed to the mocap clip's
// natural speed. Walking faster → faster animation playback, preserving
// the illusion that footsteps match ground contact.
//
// Consumers call update() each physics step and receive the sampled root
// translation (lateral sway + vertical bob) to feed into their pose system.

#pragma once

#include <algorithm>
#include <cmath>
#include <functional>

#include "MotionService.h"

namespace Darkness {

// ── Flag callback ──
// Consumers (sound, physics, scripts) register a listener to receive
// motion flag events as they fire during playback. The callback receives
// the combined flag bits and the frame number that triggered them.
//
// TODO(sound): This callback mechanism is ready but currently never fires
// because MI flag arrays are not loading from motions.crf (truncated on
// disk). Fix the flag parsing in MotionService::parseMI() before wiring
// up footstep sounds. See TODO(sound) in MotionService.h.
using MotionFlagCallback = std::function<void(uint32_t flags, int frame)>;

// ════════════════════════════════════════════════════════════════════
// MotionPlayback — per-entity motion clip player
// ════════════════════════════════════════════════════════════════════
//
// Usage:
//   MotionPlayback playback;
//   playback.start(clip, horizontalSpeed, stretch, timewarpMin, timewarpMax);
//   ...
//   Vector3 offset = playback.update(dt, baselineZ);
//   if (playback.isFinished()) { /* chain next clip */ }

class MotionPlayback {
public:
    /// Start playing a clip. Computes timeWarp from the player's current
    /// horizontal speed vs the clip's natural locomotion speed.
    ///
    /// Parameters:
    ///   clip       — motion clip to play (must outlive playback)
    ///   speed      — player's current horizontal speed (units/sec)
    ///   stretch    — gait stretch factor (WALK_STRETCH / RUN_STRETCH / etc.)     // TUNING
    ///   twMin      — minimum allowed timeWarp                                     // TUNING
    ///   twMax      — maximum allowed timeWarp                                     // TUNING
    inline void start(const MotionClip &clip, float speed,
                      float stretch, float twMin, float twMax) {
        mClip = &clip;
        mClipTime = 0.0f;
        mActive = true;
        mFinished = false;
        mLastFlagFrame = -1;

        // Compute timeWarp: ratio of actual speed to the clip's natural speed.
        // naturalSpeed = total forward displacement / duration
        // If the player walks faster than the motion's natural pace, we speed
        // up the animation proportionally (and vice versa).
        float naturalSpeed = 0.0f;
        if (clip.duration > 0.0f && std::abs(clip.totalForwardDisp) > 0.001f) {
            naturalSpeed = std::abs(clip.totalForwardDisp) / clip.duration;
        }

        if (naturalSpeed > 0.001f && stretch > 0.001f) {
            mTimeWarp = speed / (naturalSpeed * stretch);
            mTimeWarp = std::clamp(mTimeWarp, twMin, twMax);
        } else {
            // Clip has no forward displacement (e.g. idle breathing) or
            // stretch is zero — play at natural rate.
            mTimeWarp = 1.0f;
        }
    }

    /// Advance playback by dt (seconds, pre-multiplied by physics timestep).
    /// Returns the sampled root translation at the current time, with:
    ///   - X component zeroed (forward motion handled by physics movement)
    ///   - Y component = lateral sway from mocap
    ///   - Z component = vertical bob from mocap + baselineZ offset
    ///
    /// baselineZ positions the camera at correct standing/crouching height.
    /// The mocap Z oscillation (typically ±0.05–0.12 units per stride) rides
    /// on top of this constant.
    ///
    /// Emits flag callbacks for any motion flags crossed since the last update.
    inline Vector3 update(float dt, float baselineZ) {
        if (!mActive || !mClip) return Vector3(0.0f, 0.0f, baselineZ);

        float prevTime = mClipTime;
        mClipTime += dt * mTimeWarp;

        // Check if clip has finished
        if (mClipTime >= mClip->duration) {
            mClipTime = mClip->duration;
            mFinished = true;
        }

        // Emit flag callbacks for all flags crossed in [prevTime, mClipTime]
        emitFlags(prevTime, mClipTime);

        // Sample root translation at current time
        Vector3 root = MotionService::sampleRootTranslation(*mClip, mClipTime);

        // Transform to camera-space output:
        //   - Zero X (forward displacement handled by physics velocity)
        //   - Keep Y (lateral sway — feeds head spring as lateral target)
        //   - Keep Z + baseline (vertical bob on top of standing height)
        return Vector3(0.0f, root.y, root.z + baselineZ);
    }

    /// Is a clip currently playing?
    inline bool isActive() const { return mActive; }

    /// Has the current clip finished playing?
    inline bool isFinished() const { return mFinished; }

    /// Stop playback immediately.
    inline void stop() {
        mActive = false;
        mFinished = false;
        mClip = nullptr;
        mClipTime = 0.0f;
        mTimeWarp = 1.0f;
        mLastFlagFrame = -1;
    }

    /// Register a callback for motion flag events.
    /// Called each time playback crosses a frame with flags set.
    /// Used for sound (footfalls), physics (foot contact), scripts (triggers).
    inline void setFlagCallback(MotionFlagCallback cb) {
        mFlagCb = std::move(cb);
    }

    // ── Diagnostics (read-only) ──

    /// Name of the currently playing clip (or "(none)").
    inline const char* clipName() const {
        return (mClip && mActive) ? mClip->name.c_str() : "(none)";
    }

    /// Current time within the clip (seconds).
    inline float clipTime() const { return mClipTime; }

    /// Current timeWarp factor.
    inline float timeWarp() const { return mTimeWarp; }

    /// Duration of the current clip (seconds).
    inline float clipDuration() const {
        return mClip ? mClip->duration : 0.0f;
    }

    /// Progress through current clip [0, 1].
    inline float clipProgress() const {
        if (!mClip || mClip->duration <= 0.0f) return 0.0f;
        return std::min(mClipTime / mClip->duration, 1.0f);
    }

private:
    /// Emit flag callbacks for all motion flags between prevTime and curTime.
    /// Iterates the clip's flag list in order, calling back for each flag
    /// whose frame falls in the (prevFrame, curFrame] range.
    inline void emitFlags(float prevTime, float curTime) {
        if (!mFlagCb || !mClip || mClip->flags.empty()) return;

        int prevFrame = static_cast<int>(prevTime * static_cast<float>(mClip->freq));
        int curFrame  = static_cast<int>(curTime * static_cast<float>(mClip->freq));

        // Don't re-emit flags for the same frame range
        if (curFrame <= mLastFlagFrame) return;
        int startFrame = std::max(mLastFlagFrame, prevFrame);

        for (const auto &[frame, bits] : mClip->flags) {
            if (frame > startFrame && frame <= curFrame) {
                mFlagCb(bits, frame);
            }
        }

        mLastFlagFrame = curFrame;
    }

    const MotionClip *mClip = nullptr;
    float mClipTime   = 0.0f;    // elapsed time in clip (seconds)
    float mTimeWarp   = 1.0f;    // speed scaling factor
    bool  mActive     = false;   // currently playing?
    bool  mFinished   = false;   // reached end of clip?
    int   mLastFlagFrame = -1;   // last frame whose flags were emitted
    MotionFlagCallback mFlagCb;  // callback for motion flag events
};

} // namespace Darkness
