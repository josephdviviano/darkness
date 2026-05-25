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

// ── AutoFlyTour.h ─────────────────────────────────────────────────────────
//
// Deterministic auto-fly waypoint tour for repeatable audio perf-profiling.
//
// Drives the fly-mode camera through a fixed random sequence of the N nearest
// pathing probes so every iteration of `tools/perf_sweep.sh` (which runs
// darknessRender for ~60 s with `--exit-after-seconds`) captures the SAME
// listener trajectory. Without this the listener sits at the player spawn for
// the whole run and the captured audio_perf.jsonl reflects only one geometric
// configuration — useless for comparing two tuning variants.
//
// Why "N-nearest then shuffle" instead of (a) all probes or (b) ordered
// nearest-first:
//   - (a) would have tours scale with mission size — a 4000-probe MISS1 would
//     take many minutes per loop, defeating the 60-s sweep window.
//   - (b) makes the tour curl outward in concentric rings — perceptually a
//     boring orbit AND degenerate for perf (all probes at similar visibility
//     to the spawn cluster). Random shuffle of the local set forces the camera
//     to crisscross rooms, hitting more distinct portal/pathing-edge regions.
//
// Coordinate convention: Dark Engine Z-up (X=right, Y=forward, Z=up). Camera
// yaw rotates in the XY plane, pitch tilts about the camera-right axis.
//
// Header-only — matches the convention of the other src/main/ debug overlays
// (RoomDebugViz, FrobSystem, GrabSystem, SpawnFinder, etc.) per CLAUDE.md.
// ──────────────────────────────────────────────────────────────────────────

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

#include "DarknessMath.h"

namespace Darkness {

struct AutoFlyTour {
    // ── User-facing knobs (set from CLI / YAML / console) ──

    // Requested-on master flag. Set true to ask for activation; the next
    // tick() with valid probes will flip `active` true and snapshot
    // waypoints. Set false to stop driving the camera (manual fly resumes).
    bool   enabled = false;

    // How many of the closest pathing probes form the tour. Probe count
    // varies by mission (a few hundred to a few thousand); 50 is a tour
    // length that completes in ~60 s at 10 ft/s with typical probe
    // spacing of ~10 ft.
    int    waypointCount = 50;

    // Movement speed in world units (feet) per second. 10 ft/s ≈ brisk
    // player walk — comparable to the manual fly-mode default and slow
    // enough that voices stay in earshot long enough to influence DSP
    // load measurements.
    float  speed = 10.0f;

    // Optional dwell time at each waypoint before continuing. Default 0
    // (continuous motion). Set non-zero when the sweep wants per-waypoint
    // perf-window samples to be cleanly separable in post-analysis.
    float  pauseAtWaypointSec = 0.0f;

    // PRNG seed for the visit-order shuffle. Constant default (0xC0FFEE
    // — the same constant used in other deterministic-bake paths
    // elsewhere in this codebase) so two sweep iterations of the same
    // mission with the same waypointCount visit waypoints in the same
    // order, making perf-window-to-perf-window comparison comparable.
    uint32_t seed = 0xC0FFEEu;

    // ── Runtime state (filled by activate() / tick()) ──

    std::vector<Vector3> waypoints;     // built once at activation
    int    currentWaypointIdx = 0;      // index into waypoints[]; loops
    float  pauseAccumSec = 0.0f;        // dwell timer for pauseAtWaypointSec
    bool   active = false;              // true once waypoints[] is built

    // ── Smoothing knobs (not user-exposed) ──

    // Look-direction slerp half-life in seconds — the time for the
    // current heading to close half the gap to the target heading each
    // frame. ~0.25 s gives a noticeable swing without a snap; with the
    // ~10 ft/s default speed and ~10 ft waypoint spacing it settles long
    // before the next waypoint transition.
    float  headingHalfLifeSec = 0.25f;

    // Snap threshold (feet) — when within this distance of the current
    // waypoint, advance to the next. Half a foot is tight enough that
    // the camera visibly arrives but loose enough that timestep jitter
    // doesn't make us orbit.
    float  arrivalRadiusFt = 0.5f;

    // ── Lifecycle ──

    // Snapshot the N-nearest probes from `allProbes` to `fromPos`,
    // shuffle deterministically with `seed`, and arm the tour. Emits a
    // [AUTO_FLY] activation line on success, [FALLBACK] when no probes
    // are loaded (per feedback_no_silent_fallbacks — gracefully
    // degrades to standard fly mode).
    void activate(const Vector3 &fromPos,
                  const std::vector<Vector3> &allProbes)
    {
        waypoints.clear();
        currentWaypointIdx = 0;
        pauseAccumSec = 0.0f;
        active = false;

        if (allProbes.empty()) {
            std::fprintf(stderr,
                "[FALLBACK] auto-fly requested but no pathing probes "
                "loaded — disabling\n");
            enabled = false;
            return;
        }

        // N-nearest selection: build (sqdist, index) pairs, partial-sort
        // by distance, take the first min(N, P). We snapshot once at
        // activation; the probe set does not change at runtime, so a
        // per-frame query would be wasted work.
        const size_t P = allProbes.size();
        const size_t N = static_cast<size_t>(std::max(1, waypointCount));
        std::vector<std::pair<float, size_t>> ranked;
        ranked.reserve(P);
        for (size_t i = 0; i < P; ++i) {
            const Vector3 d = allProbes[i] - fromPos;
            // glm::length2 in DarknessMath
            ranked.emplace_back(glm::dot(d, d), i);
        }
        const size_t take = std::min(N, P);
        std::partial_sort(ranked.begin(), ranked.begin() + take, ranked.end(),
            [](const auto &a, const auto &b) { return a.first < b.first; });

        waypoints.reserve(take);
        for (size_t k = 0; k < take; ++k) {
            waypoints.push_back(allProbes[ranked[k].second]);
        }

        // Deterministic shuffle — same seed gives the same visit order
        // across sweep iterations. mt19937 is overkill for ≤ a few
        // thousand swaps but it's the standard portable PRNG and the
        // sequence is identical across compilers / OSes.
        std::mt19937 rng(seed);
        std::shuffle(waypoints.begin(), waypoints.end(), rng);

        active = true;
        std::fprintf(stderr,
            "[AUTO_FLY] activated: %zu waypoints (%zu probes total, "
            "seed=0x%08x, speed=%.1f ft/s, pause=%.2f s)\n",
            waypoints.size(), P, seed, speed, pauseAtWaypointSec);
    }

    // Stop driving the camera. Safe to call when inactive.
    void deactivate() {
        if (active) {
            std::fprintf(stderr,
                "[AUTO_FLY] deactivated at waypoint %d/%zu\n",
                currentWaypointIdx, waypoints.size());
        }
        active = false;
        waypoints.clear();
        currentWaypointIdx = 0;
        pauseAccumSec = 0.0f;
    }

    // Advance the camera one frame's worth toward the current waypoint.
    // Modifies camPos / camYaw / camPitch in place. Caller is responsible
    // for not invoking this when physics mode is on (the integrator owns
    // the position there).
    //
    // Position: straight-line lerp at constant `speed` (no easing — we
    // want a constant per-frame DSP load profile; ease-in/ease-out would
    // make the early/late part of each segment have different listener
    // velocity, smearing comparisons).
    //
    // Orientation: exponential-decay slerp toward the bearing to the
    // current waypoint. Smooth (per feedback_smooth_transitions) — no
    // snap on waypoint transition.
    void tick(float dt, float camPos[3], float &camYaw, float &camPitch) {
        if (!active || waypoints.empty()) return;
        if (dt <= 0.0f) return;

        const Vector3 cur(camPos[0], camPos[1], camPos[2]);
        const Vector3 tgt = waypoints[currentWaypointIdx];
        Vector3 delta = tgt - cur;
        float dist = glm::length(delta);

        // ── Heading: slerp toward the bearing to the target ──
        // We compute target yaw/pitch from the bearing, then ease the
        // current camera angles toward those targets with a per-frame
        // closure fraction derived from headingHalfLifeSec.
        // alpha = 1 - 0.5^(dt / halfLife)  ⇒  dt = halfLife → close 50%.
        if (dist > 1e-4f) {
            // Z-up bearing: yaw is angle of XY projection from +X;
            // pitch is the elevation above the XY plane.
            float horizLen = std::sqrt(delta.x * delta.x + delta.y * delta.y);
            float targetYaw   = std::atan2(delta.y, delta.x);
            float targetPitch = std::atan2(delta.z, std::max(horizLen, 1e-4f));

            // Choose the equivalent target yaw closest to current yaw so
            // the slerp goes the short way around (don't wrap +π/-π).
            float dy = targetYaw - camYaw;
            while (dy >  static_cast<float>(M_PI)) dy -= 2.0f * static_cast<float>(M_PI);
            while (dy < -static_cast<float>(M_PI)) dy += 2.0f * static_cast<float>(M_PI);

            float alpha = 1.0f
                - std::pow(0.5f, dt / std::max(headingHalfLifeSec, 1e-3f));
            camYaw   += alpha * dy;
            camPitch += alpha * (targetPitch - camPitch);
        }

        // ── Dwell at waypoint (if configured) ──
        if (dist <= arrivalRadiusFt && pauseAtWaypointSec > 0.0f) {
            pauseAccumSec += dt;
            if (pauseAccumSec < pauseAtWaypointSec) {
                return;  // hold position; heading still slerps toward
                         // the (already-reached) waypoint, which is a
                         // no-op once the angles have converged
            }
            pauseAccumSec = 0.0f;  // dwell complete; fall through to advance
        }

        // ── Advance / loop ──
        if (dist <= arrivalRadiusFt) {
            const int prev = currentWaypointIdx;
            currentWaypointIdx =
                (currentWaypointIdx + 1) % static_cast<int>(waypoints.size());
            const Vector3 &p = waypoints[currentWaypointIdx];
            std::fprintf(stderr,
                "[AUTO_FLY] reached waypoint %d/%zu -> next %d at "
                "(%.1f,%.1f,%.1f)\n",
                prev, waypoints.size(), currentWaypointIdx,
                p.x, p.y, p.z);
            // Don't move this frame — let next tick aim at the new
            // waypoint so the orientation slerp has a frame to react
            // before we start traveling toward it.
            return;
        }

        // ── Move at constant speed toward target ──
        // Step length capped to the remaining distance so we never
        // overshoot — at low frame rates a long dt could otherwise
        // place us past the waypoint and the [AUTO_FLY] log would
        // skip the arrival event.
        float step = std::min(speed * dt, dist);
        Vector3 dir = delta / dist;
        camPos[0] = cur.x + dir.x * step;
        camPos[1] = cur.y + dir.y * step;
        camPos[2] = cur.z + dir.z * step;
    }
};

} // namespace Darkness
