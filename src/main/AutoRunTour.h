/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2026 darkness contributors
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

// ── AutoRunTour.h ─────────────────────────────────────────────────────────
//
// Deterministic ON-FOOT waypoint tour for repeatable audio perf-profiling.
//
// Physics-mode sibling of AutoFlyTour: instead of driving the fly camera
// directly, it emits per-frame MOVEMENT INTENTS (forward + yaw + speed mode)
// into the same IPhysicsWorld injection API the keyboard uses
// (setPlayerMovement / setPlayerYaw / setPlayerRunning). The real player
// integrator then does everything a human-held W+Ctrl would: gravity, BSP
// collision, stair-stepping, stride accumulation — and therefore FOOTSTEPS.
// Footsteps matter because each stride spawns a full voice (schema lookup,
// initVoiceDSP, binaural + baked-reflection convolution), giving the stress
// run a steady voice-churn load (~2.5 spawns/s at run speed) that a flying
// camera can never produce. Per feedback_simulation_over_hacks: nothing
// downstream of the input layer is faked while walking.
//
// Locomotion is HYBRID (user design, 2026-07-04): walk by default; when a
// waypoint is unreachable on foot — wedged against object collision the
// route trace can't see, z-offset beyond stair-stepping, behind a closed
// door the walker cannot frob, or in a disconnected room — the tour lifts
// physics OFF, glides straight to the waypoint (noclip, AutoFlyTour-style),
// then drops physics back ON and resumes running. Guaranteed full-tour
// coverage at the price of a few footstep-less seconds per assist; every
// assist is loud ([AUTO_RUN] fly-assist lines) so a tour that degenerates
// into mostly-flying is visible at a glance in the log. Five MISS6 smoke
// iterations proved walking-only cannot cover Thief levels: audio probes
// are not nav nodes, and the missions are full of furniture, thresholds,
// and level changes that defeat straight-line pursuit.
//
// Walking legs are routed through the room graph: pathfinder(from, to)
// returns the BFS portal-CENTER chain (RoomService::propagateSoundPath
// pathOut hops — aperture midpoints, deliberately NOT paths[].chain's
// corner-hugging acoustic bend anchors, which steered the walker into door
// frames), with live door blocking so shut doors mark a waypoint
// fly-assist rather than a grind target.
//
// Determinism: same mission + same .probes + same seed + same waypointCount
// → same waypoint order. Player physics is a fixed-substep integrator, so
// the trajectory (and thus footstep count) is reproducible run-to-run to
// within render-dt sampling jitter of the yaw — good enough for the
// aggregate-percentile A/B comparison tools/perf_diff.py performs.
//
// Coordinate convention: Dark Engine Z-up (X=right, Y=forward, Z=up). Yaw
// rotates in the XY plane; at yaw=0 the player faces +X.
//
// Header-only — matches the convention of the other src/main/ helpers
// (AutoFlyTour, AudioCaptureSpin, RoomDebugViz, etc.) per CLAUDE.md.
// ──────────────────────────────────────────────────────────────────────────

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <functional>
#include <random>
#include <vector>

#include "DarknessMath.h"

namespace Darkness {

struct AutoRunTour {
    // Speed mode maps onto the player integrator's existing modes via the
    // caller (Run → setPlayerRunning(true), Creep → setPlayerSneaking(true),
    // Walk → neither). Run is the default: fastest room coverage AND the
    // loudest footstep schema volume scale, i.e. the maximal stress case.
    enum class SpeedMode { Run, Walk, Creep };

    // ── User-facing knobs (set from CLI / console) ──

    // Requested-on master flag. Set true to ask for activation; the next
    // updateMovement tick with a live physics world + loaded probes flips
    // `active` and snapshots waypoints. Set false to return control to the
    // keyboard.
    bool      enabled = false;

    // How many probes form the tour. Same default as AutoFlyTour; the
    // room-spanning guarantee below may extend the candidate take past N
    // until a second room appears, but the final tour is still capped at N.
    int       waypointCount = 50;

    SpeedMode speedMode = SpeedMode::Run;

    // PRNG seed for the visit-order shuffle (same constant family as
    // AutoFlyTour so the two harnesses' docs read alike).
    uint32_t  seed = 0xC0FFEEu;

    // Candidate pre-filter: prefer probes near the activation Z so the
    // tour stays predominantly ON FOOT (fly-assist makes any probe
    // reachable, but flying doesn't generate footsteps — the harness's
    // whole point). 12 ft admits same-floor probes plus gentle ramps.
    float     maxWaypointZDeltaFt = 12.0f;

    // ── Waypoints ──

    struct Waypoint {
        Vector3 pos{0.0f, 0.0f, 0.0f};
        int32_t roomID = -1;
    };

    std::vector<Waypoint> waypoints;   // built once at activation
    int   currentWaypointIdx = 0;      // loops
    bool  active = false;

    // ── Walking-leg routing ──
    // A straight line between two audio probes routinely crosses walls
    // (probes are acoustic sample points, not nav nodes — the first MISS6
    // smoke run stuck-skipped 8 of 9 waypoints). Walking legs therefore
    // follow the room graph's portal-CENTER chain. Returns false when the
    // waypoint is UNROUTABLE on foot (disconnected, or every route
    // crosses a closed door) — the tour then fly-assists instead of
    // grinding against a door. true + empty chainOut = same-room direct
    // leg. Unset ⇒ direct legs everywhere (the stuck/timeout triggers
    // still arm fly-assist when a direct leg fails).
    using Pathfinder =
        std::function<bool(const Vector3 &from, const Vector3 &to,
                           std::vector<Vector3> &chainOut)>;
    Pathfinder pathfinder;

    // Remaining leg targets for the CURRENT waypoint; back() is the
    // waypoint itself, earlier entries are portal centers.
    std::deque<Vector3> legs;

    // ── Fly-assist state ──

    bool  flyAssist      = false;  // true while gliding (physics is OFF —
                                   // the caller owns the toggle + body sync)
    float flyAssistSpeed = 25.0f;  // ft/s — brisk; minimizes footstep-less time
    int   flyAssists     = 0;      // running count, reported on deactivate

    // ── Smoothing / arrival (not user-exposed) ──

    float headingHalfLifeSec = 0.25f;  // same feel as AutoFlyTour
    float arrivalRadiusFt    = 2.5f;   // XY-only; player capsule ≈ 1.2 ft
                                       // radius + stride granularity make
                                       // the fly tour's 0.5 ft unreachable
    // Looser radius for INTERMEDIATE portal centers: the door frame's
    // collision hull crowds the 2.5 ft circle; passing "near enough"
    // through the aperture is all a leg needs.
    float anchorArrivalRadiusFt = 4.0f;
    // 3D arrival radius while flying (Z matters in the air).
    float flyArrivalRadiusFt    = 2.0f;
    // Forward-gate half-angle: only press "forward" when the bearing error
    // is inside this cone. 60° is forgiving enough to keep moving through
    // the slerp on shallow turns but stops the face-into-wall case on
    // reversals.
    float forwardGateRad     = 60.0f * static_cast<float>(M_PI) / 180.0f;

    // ── Stuck / timeout triggers (arm fly-assist, never hang) ──

    float stuckWindowSec     = 3.0f;   // evaluate progress on this cadence
    float stuckMinProgressFt = 1.0f;   // XY displacement below this = stuck
    float waypointTimeoutSec = 30.0f;  // absolute per-waypoint cap

    float   stuckTimerSec      = 0.0f;
    float   waypointElapsedSec = 0.0f;
    Vector3 progressAnchor{0.0f, 0.0f, 0.0f};

    // ── Lifecycle ──

    // Build the tour from `allProbes` (positions + room IDs, from
    // AudioService::getPathingProbeViz()) around `fromPos`:
    //   1. drop probes in BSP void (roomID -1) or beyond the Z filter,
    //   2. rank by XY distance, take the N nearest,
    //   3. if those N sit in ONE room, extend the take until a second room
    //      appears (swapping it in for the farthest kept probe) so every
    //      loop is guaranteed at least one portal crossing,
    //   4. shuffle deterministically.
    // Emits [AUTO_RUN] on success, [FALLBACK] + self-disable when no usable
    // probes exist (per feedback_no_silent_fallbacks).
    void activate(const Vector3 &fromPos,
                  const std::vector<Waypoint> &allProbes)
    {
        waypoints.clear();
        currentWaypointIdx = 0;
        flyAssists = 0;
        flyAssist = false;
        resetProgress(fromPos);
        active = false;

        // Filter: walk-friendly candidates only.
        std::vector<Waypoint> usable;
        usable.reserve(allProbes.size());
        for (const Waypoint &w : allProbes) {
            if (w.roomID < 0) continue;  // BSP void — unwalkable
            if (std::fabs(w.pos.z - fromPos.z) > maxWaypointZDeltaFt) continue;
            usable.push_back(w);
        }
        if (usable.empty()) {
            std::fprintf(stderr,
                "[FALLBACK] auto-run requested but no walkable pathing "
                "probes (%zu total, all void or beyond dz=%.1f ft) — "
                "disabling\n", allProbes.size(), maxWaypointZDeltaFt);
            enabled = false;
            return;
        }

        // Rank by XY distance (walking metric — see header comment).
        std::vector<std::pair<float, size_t>> ranked;
        ranked.reserve(usable.size());
        for (size_t i = 0; i < usable.size(); ++i) {
            const float dx = usable[i].pos.x - fromPos.x;
            const float dy = usable[i].pos.y - fromPos.y;
            ranked.emplace_back(dx * dx + dy * dy, i);
        }
        std::sort(ranked.begin(), ranked.end(),
            [](const auto &a, const auto &b) { return a.first < b.first; });

        const size_t take =
            std::min(static_cast<size_t>(std::max(1, waypointCount)),
                     usable.size());
        waypoints.reserve(take);
        for (size_t k = 0; k < take; ++k)
            waypoints.push_back(usable[ranked[k].second]);

        // Room-spanning guarantee: the whole point of the run tour is
        // crossing portals, so a single-room waypoint set is useless. Scan
        // outward past N for the first probe in a different room and swap
        // it in for the farthest kept probe.
        int32_t firstRoom = waypoints.front().roomID;
        bool spansRooms = std::any_of(waypoints.begin(), waypoints.end(),
            [&](const Waypoint &w) { return w.roomID != firstRoom; });
        if (!spansRooms) {
            for (size_t k = take; k < ranked.size(); ++k) {
                const Waypoint &cand = usable[ranked[k].second];
                if (cand.roomID != firstRoom) {
                    waypoints.back() = cand;
                    spansRooms = true;
                    break;
                }
            }
        }
        if (!spansRooms) {
            // Only one room within reach — still runnable (footstep churn
            // is intact) but the portal-crossing load is absent. Loud, not
            // fatal.
            std::fprintf(stderr,
                "[AUTO_RUN] WARNING: all %zu reachable probes sit in room "
                "%d — tour will not cross portals\n",
                waypoints.size(), firstRoom);
        }

        std::mt19937 rng(seed);
        std::shuffle(waypoints.begin(), waypoints.end(), rng);

        rebuildLegs(fromPos);

        // Count distinct rooms for the activation line — the acceptance
        // criterion for PR 0.1 is "crosses ≥ 6 rooms on MISS6", so make
        // the number greppable.
        std::vector<int32_t> rooms;
        for (const Waypoint &w : waypoints) rooms.push_back(w.roomID);
        std::sort(rooms.begin(), rooms.end());
        rooms.erase(std::unique(rooms.begin(), rooms.end()), rooms.end());

        active = true;
        std::fprintf(stderr,
            "[AUTO_RUN] activated: %zu waypoints spanning %zu rooms "
            "(%zu usable / %zu total probes, seed=0x%08x, mode=%s, "
            "dzMax=%.1f ft)\n",
            waypoints.size(), rooms.size(), usable.size(),
            allProbes.size(), seed,
            speedMode == SpeedMode::Run  ? "run"
          : speedMode == SpeedMode::Walk ? "walk" : "creep",
            maxWaypointZDeltaFt);
    }

    void deactivate() {
        if (active) {
            std::fprintf(stderr,
                "[AUTO_RUN] deactivated at waypoint %d/%zu "
                "(%d fly-assists)\n",
                currentWaypointIdx, waypoints.size(), flyAssists);
        }
        active = false;
        waypoints.clear();
        legs.clear();
        currentWaypointIdx = 0;
        flyAssist = false;
    }

    // Advance one WALKING frame. `camPos` is the player's eye position
    // (read-only — the physics integrator owns it); `camYaw` is steered
    // toward the bearing, `camPitch` decays level. Returns the forward
    // intent in [0, 1] for IPhysicsWorld::setPlayerMovement — 0 while
    // turning in place or dwelling on an arrival frame, 1 when clear to
    // move. Must not be called while `flyAssist` is set (the caller
    // routes to tickAssist instead).
    //
    // Steering target is the front of the leg chain (portal centers, then
    // the waypoint itself). The stuck / timeout triggers arm fly-assist
    // for the WHOLE waypoint — a blocked leg means walking has failed,
    // not that the next leg would fare better.
    float tick(float dt, const float camPos[3],
               float &camYaw, float &camPitch)
    {
        if (!active || waypoints.empty() || dt <= 0.0f || flyAssist)
            return 0.0f;

        const Vector3 cur(camPos[0], camPos[1], camPos[2]);
        const Waypoint &wp = waypoints[currentWaypointIdx];
        const Vector3 target = legs.empty() ? wp.pos : legs.front();
        const float dx = target.x - cur.x;
        const float dy = target.y - cur.y;
        const float distXY = std::sqrt(dx * dx + dy * dy);

        waypointElapsedSec += dt;
        stuckTimerSec += dt;

        // ── Heading: slerp yaw toward the XY bearing; level the pitch ──
        // (pitch affects the HRTF listener orientation, so a run that
        // spawned mid-mouse-look shouldn't tour the level staring at the
        // floor). Same closure-fraction formula as AutoFlyTour.
        float yawErr = 0.0f;
        if (distXY > 1e-4f) {
            const float targetYaw = std::atan2(dy, dx);
            yawErr = targetYaw - camYaw;
            while (yawErr >  static_cast<float>(M_PI)) yawErr -= 2.0f * static_cast<float>(M_PI);
            while (yawErr < -static_cast<float>(M_PI)) yawErr += 2.0f * static_cast<float>(M_PI);
            const float alpha = 1.0f
                - std::pow(0.5f, dt / std::max(headingHalfLifeSec, 1e-3f));
            camYaw   += alpha * yawErr;
            camPitch += alpha * (0.0f - camPitch);
            // Recompute the residual error AFTER the slerp so the forward
            // gate sees this frame's actual heading.
            yawErr -= alpha * yawErr;
        }

        // ── Arrival (XY-only; anchors get the looser doorway radius) ──
        const float arriveR =
            (legs.size() > 1) ? anchorArrivalRadiusFt : arrivalRadiusFt;
        if (distXY <= arriveR) {
            if (legs.size() > 1) {
                // Intermediate portal center — keep walking the chain.
                // Reaching one is progress by definition: restart the
                // stuck window so a long multi-leg chain can't trip it
                // between anchors.
                legs.pop_front();
                stuckTimerSec  = 0.0f;
                progressAnchor = cur;
            } else {
                advanceWaypoint(cur);
            }
            return 0.0f;  // let next tick aim at the new target first
        }

        // ── Stuck trigger: no XY progress over the window ──
        if (stuckTimerSec >= stuckWindowSec) {
            const float px = cur.x - progressAnchor.x;
            const float py = cur.y - progressAnchor.y;
            if (std::sqrt(px * px + py * py) < stuckMinProgressFt) {
                std::fprintf(stderr,
                    "[AUTO_RUN] stuck at (%.1f,%.1f,%.1f) — %.1f ft "
                    "progress in %.1f s toward waypoint %d "
                    "(%.1f,%.1f,%.1f)\n",
                    cur.x, cur.y, cur.z,
                    std::sqrt(px * px + py * py), stuckTimerSec,
                    currentWaypointIdx, wp.pos.x, wp.pos.y, wp.pos.z);
                beginFlyAssist("stuck");
                return 0.0f;
            }
            // Progress made — slide the window forward.
            stuckTimerSec = 0.0f;
            progressAnchor = cur;
        }

        // ── Timeout trigger: waypoint is taking absurdly long ──
        // (catches slow-orbit pathologies the progress test can't — e.g.
        // circling a pillar making > 1 ft/3 s but never arriving).
        if (waypointElapsedSec >= waypointTimeoutSec) {
            std::fprintf(stderr,
                "[AUTO_RUN] timeout: waypoint %d not reached in %.0f s "
                "(%.1f ft away)\n",
                currentWaypointIdx, waypointTimeoutSec, distXY);
            beginFlyAssist("timeout");
            return 0.0f;
        }

        // ── Forward gate ──
        return (std::fabs(yawErr) <= forwardGateRad) ? 1.0f : 0.0f;
    }

    // Advance one FLYING frame (only while `flyAssist` is set; the caller
    // has physics OFF and this glide owns the camera). Straight-line
    // constant-speed 3D lerp toward the current waypoint, yaw slerped to
    // the bearing, pitch leveled. Returns true when the glide has landed
    // AND walking should resume — the caller then re-enables physics and
    // syncs the player body to the camera (setPlayerPosition +
    // setPlayerYaw, same as the physics_mode console toggle). May return
    // false on the landing frame if the NEXT waypoint immediately
    // re-arms an assist (unroutable chain), in which case the glide
    // simply continues to the new target.
    bool tickAssist(float dt, float camPos[3],
                    float &camYaw, float &camPitch)
    {
        if (!active || waypoints.empty() || !flyAssist || dt <= 0.0f)
            return false;

        const Vector3 cur(camPos[0], camPos[1], camPos[2]);
        const Waypoint &wp = waypoints[currentWaypointIdx];
        const Vector3 delta = wp.pos - cur;
        const float dist = glm::length(delta);

        if (dist <= flyArrivalRadiusFt) {
            std::fprintf(stderr,
                "[AUTO_RUN] fly-assist landed at waypoint %d (room %d) — "
                "resuming on foot\n", currentWaypointIdx, wp.roomID);
            flyAssist = false;
            advanceWaypoint(cur);   // arrival counts as reached; may
                                    // re-arm assist for the next waypoint
            return !flyAssist;
        }

        // Yaw toward the XY bearing (same slerp as walking); pitch level.
        const float horiz = std::sqrt(delta.x * delta.x + delta.y * delta.y);
        if (horiz > 1e-4f) {
            const float targetYaw = std::atan2(delta.y, delta.x);
            float dyaw = targetYaw - camYaw;
            while (dyaw >  static_cast<float>(M_PI)) dyaw -= 2.0f * static_cast<float>(M_PI);
            while (dyaw < -static_cast<float>(M_PI)) dyaw += 2.0f * static_cast<float>(M_PI);
            const float alpha = 1.0f
                - std::pow(0.5f, dt / std::max(headingHalfLifeSec, 1e-3f));
            camYaw   += alpha * dyaw;
            camPitch += alpha * (0.0f - camPitch);
        }

        // Constant-speed glide, step capped so we never overshoot.
        const float step = std::min(flyAssistSpeed * dt, dist);
        const Vector3 dir = delta / dist;
        camPos[0] = cur.x + dir.x * step;
        camPos[1] = cur.y + dir.y * step;
        camPos[2] = cur.z + dir.z * step;
        return false;
    }

private:
    void beginFlyAssist(const char *reason) {
        const Waypoint &wp = waypoints[currentWaypointIdx];
        flyAssist = true;
        ++flyAssists;
        legs.clear();
        std::fprintf(stderr,
            "[AUTO_RUN] fly-assist #%d to waypoint %d (%.1f,%.1f,%.1f) "
            "room %d — reason: %s (physics off for the glide)\n",
            flyAssists, currentWaypointIdx,
            wp.pos.x, wp.pos.y, wp.pos.z, wp.roomID, reason);
    }

    void advanceWaypoint(const Vector3 &cur) {
        const int prev = currentWaypointIdx;
        currentWaypointIdx =
            (currentWaypointIdx + 1) % static_cast<int>(waypoints.size());
        const Waypoint &next = waypoints[currentWaypointIdx];
        std::fprintf(stderr,
            "[AUTO_RUN] reached waypoint %d/%zu (room %d) -> next %d "
            "at (%.1f,%.1f,%.1f) room %d\n",
            prev, waypoints.size(), waypoints[prev].roomID,
            currentWaypointIdx,
            next.pos.x, next.pos.y, next.pos.z, next.roomID);
        resetProgress(cur);
        rebuildLegs(cur);
    }

    // Route the current waypoint through the room graph: portal centers
    // first, waypoint last. Unroutable (closed door / disconnected) ⇒
    // fly-assist immediately — walking into a shut door for a full
    // stuck-window per waypoint wasted most of the tour in early smoke
    // runs, and the walker cannot frob doors.
    void rebuildLegs(const Vector3 &cur) {
        legs.clear();
        const Waypoint &wp = waypoints[currentWaypointIdx];
        std::vector<Vector3> chain;
        bool routable = true;
        if (pathfinder) routable = pathfinder(cur, wp.pos, chain);
        if (!routable) {
            beginFlyAssist("unroutable (closed door / disconnected)");
            return;
        }
        for (const Vector3 &a : chain) legs.push_back(a);
        if (!chain.empty()) {
            std::fprintf(stderr,
                "[AUTO_RUN] waypoint %d routed via %zu portal center(s)\n",
                currentWaypointIdx, chain.size());
        }
        legs.push_back(wp.pos);
    }

    void resetProgress(const Vector3 &at) {
        stuckTimerSec = 0.0f;
        waypointElapsedSec = 0.0f;
        progressAnchor = at;
    }
};

} // namespace Darkness
