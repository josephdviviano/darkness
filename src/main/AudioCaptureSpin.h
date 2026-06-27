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

// ── AudioCaptureSpin.h ────────────────────────────────────────────────────
//
// Stationary spin-in-place acoustic capture for quick Steam Audio positional
// analysis. Companion to the auto-fly probe tour (AutoFlyTour.h), but instead
// of travelling between waypoints it pins the listener at one world point and
// rotates the camera in place so a single invocation captures the full
// azimuth sweep of the spatializer's response at that point.
//
// Driven by --audio-capture x,y,z (see RenderConfig.h). The sequence:
//   1. player spawns normally (physics on),
//   2. on the first movement tick, physics mode is forced off,
//   3. the camera/listener is teleported to the requested point,
//   4. audio_log is enabled (done at the seed site in DarknessRender.cpp),
//   5. the camera spins `rotations` full turns over `durationSec` seconds
//      while audio_log records, then
//   6. the capture requests a clean program exit.
//
// The yaw rate is constant (rotations·2π / durationSec rad/s) so the angular
// position is a linear function of capture-relative time — every azimuth is
// dwelt on for the same duration, which keeps the captured log uniformly
// sampled across the sweep. Pitch and roll are held level for the whole run.
//
// Time is accumulated from the per-frame dt (which LoopService caps at 0.1 s),
// so a render stall lengthens wall-clock but never skips part of the sweep;
// the capture always covers exactly `rotations` turns of motion.
//
// Coordinate convention: Dark Engine Z-up (X=right, Y=forward, Z=up). Camera
// yaw rotates in the XY plane; at yaw=0 the camera faces +X.
//
// Header-only — matches the convention of the other src/main/ debug overlays
// (AutoFlyTour, RoomDebugViz, SpawnFinder, etc.) per CLAUDE.md.
// ──────────────────────────────────────────────────────────────────────────

#pragma once

#include <cmath>
#include <cstdio>

#include "DarknessMath.h"

namespace Darkness {

struct AudioCaptureSpin {
    // ── User-facing knobs (set from CLI) ──

    // Requested-on master flag. Set true (by --audio-capture) to ask for the
    // capture; the first tick() flips `active` and teleports the listener.
    bool    enabled = false;

    // World point (Dark Engine feet, Z-up) the listener is pinned to for the
    // capture.
    Vector3 target{0.0f, 0.0f, 0.0f};

    // Length of the capture window in seconds of accumulated frame time.
    float   durationSec = 15.0f;

    // Number of full yaw turns completed over `durationSec`. 3 turns / 15 s
    // gives a leisurely 72°/s sweep — slow enough that transient SA artifacts
    // (reflection deferral, occlusion-sample flap) stay legible in the log.
    float   rotations = 3.0f;

    // ── Runtime state (filled by begin() / tick()) ──

    bool    active = false;      // true once the listener has been teleported
    float   elapsedSec = 0.0f;   // accumulated capture time
    float   startYaw = 0.0f;     // yaw at capture start; sweep is relative to it

    // ── Lifecycle ──

    // Pin the camera/listener to `target`, level pitch + roll, and arm the
    // sweep. Records the starting yaw so the rotation is measured relative to
    // wherever the camera happened to be facing at spawn. Emits an
    // [AUDIO_CAPTURE] activation line.
    void begin(float camPos[3], float &camYaw, float &camPitch, float &camRoll) {
        camPos[0] = target.x;
        camPos[1] = target.y;
        camPos[2] = target.z;
        camPitch  = 0.0f;
        camRoll   = 0.0f;
        startYaw  = camYaw;
        elapsedSec = 0.0f;
        active = true;
        std::fprintf(stderr,
            "[AUDIO_CAPTURE] begin at (%.2f, %.2f, %.2f): %.1f s, "
            "%.1f rotation(s) — physics off, audio_log on\n",
            target.x, target.y, target.z, durationSec, rotations);
    }

    // Advance one frame: hold position, rotate yaw at the constant sweep rate,
    // and accumulate elapsed time. Returns true exactly once, on the frame the
    // capture window closes — the caller should then request program exit.
    //
    // Position is re-pinned every frame (defensive): with physics off and the
    // movement code short-circuited nothing else should touch camPos, but
    // re-asserting it costs nothing and guarantees the listener never drifts.
    bool tick(float dt, float camPos[3], float &camYaw,
              float &camPitch, float &camRoll) {
        if (!active) return false;

        camPos[0] = target.x;
        camPos[1] = target.y;
        camPos[2] = target.z;
        camPitch  = 0.0f;
        camRoll   = 0.0f;

        if (dt > 0.0f) elapsedSec += dt;

        const float rate = (durationSec > 1e-4f)
            ? rotations * 2.0f * static_cast<float>(M_PI) / durationSec
            : 0.0f;  // rad/s
        // Unbounded yaw is fine — it only ever feeds cos/sin in the view
        // matrix and the listener transform, so it wraps naturally and stays
        // smooth (no ±π discontinuity from manual wrapping).
        camYaw = startYaw + elapsedSec * rate;

        if (elapsedSec >= durationSec) {
            std::fprintf(stderr,
                "[AUDIO_CAPTURE] complete: %.1f s elapsed, %.1f rotation(s) "
                "— requesting clean exit\n",
                elapsedSec, rotations);
            active = false;
            return true;
        }
        return false;
    }
};

} // namespace Darkness
