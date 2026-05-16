/******************************************************************************
 *
 *    This file is part of the Darkness engine
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

// GrabSystem.h — Player-driven physics manipulation for held objects.
//
// What this is and isn't:
//   FrobSystem owns "what does pressing F on this object mean" — target
//   acquisition and dispatch (toggle door, run script, fire interact message,
//   etc.). GrabSystem owns the case where the answer is "drive the object's
//   physics from player input" — carrying a crate, throwing a body, and
//   eventually Amnesia-style hinge/slider/valve manipulation.
//
// The unifying primitive is "drive a body's grip point toward a target point
// every frame." Different grip modes describe how that drive works:
//
//   - kCarry       Kinematic teleport. The body is suspended in ODE
//                  (no integration, no collision) and warped each frame to
//                  camera_pos + forward*holdDistance. This is the original
//                  Thief 2 carry behavior and the only mode implemented now.
//
//   - kForceDrag   Future. Soft 6-DOF spring-damper force toward the target.
//                  Body stays simulated in ODE, gravity still pulls on heavy
//                  objects, and a force tries to lift them. Lets heavy
//                  objects sag below the carry point.
//
//   - kHandlePush  Future. Apply force at a body-local grip point without
//                  positional override. Lets ODE's existing constraints
//                  (hinge for doors, slider for drawers, revolute for
//                  valves) decide the resulting motion. This is the
//                  Amnesia / Hand-of-Lux mechanic.
//
// All three modes share the same lifecycle (grab → per-frame update →
// release with impulse) and the same data structure (ActiveGrab). Adding a
// future mode is a new branch in update() and the corresponding force/joint
// setup in grab(); the public API does not need to change.
//
// Throw / drop semantics — anchored to the original Dark Engine throw model:
//
//   The original engine routes both drop and throw through a single throw
//   call parameterized by `power`. Drop uses power=0.05; player throw uses
//   power=30.0. `power` is a linear velocity in world units/sec applied
//   directly to the body (hard-set, not an impulse). Mass attenuation
//   follows momentum conservation:
//
//       coeff    = launcherMass / (objectMass + launcherMass)
//       velocity = camera_forward * power * coeff
//
//   With launcherMass ≈ 100 (player), a 1 kg crate launches at ~30 u/s and
//   a 50 kg crate at ~20 u/s. Heavy objects still move, but slower.
//
//   We expose this as releaseWithPower(objID, power) so callers don't need
//   to know the formula. A raw releaseWithVelocity(objID, vel) escape hatch
//   stays available for future systems that compute their own velocity
//   (explosions, projectile launchers).

#pragma once

#include <cstdint>
#include <cstdio>
#include <vector>
#include <algorithm>

#include "DarknessMath.h"
#include "DarknessRendererCore.h"      // Camera
#include "physics/DarkPhysics.h"
#include "worldquery/ObjectState.h"    // ObjectStateMap

namespace Darkness {

// ── Grip modes ────────────────────────────────────────────────────────────────

enum class GrabMode : uint8_t {
    kCarry      = 0,  // Kinematic teleport to camera offset (implemented)
    kForceDrag  = 1,  // Future: soft 6-DOF force toward target
    kHandlePush = 2,  // Future: force at body-local grip, constraints decide
};

// ── Tunable carry parameters ──────────────────────────────────────────────────

struct CarryParams {
    // Distance forward from the camera at which the object is held.
    // 2.0 units is the ballpark for Thief 2 — close enough to read object
    // labels but far enough that the object is not clipping into the camera.
    float holdDistance = 2.0f;

    // Vertical offset relative to the camera (added along world Z, since
    // Dark Engine is Z-up). Held objects sit well below the crosshair so
    // the player can see the world over the top of whatever they're
    // carrying — held objects render opaque, so this offset is what
    // preserves forward visibility.
    float heightOffset = -0.7f;

    // If true, the held object's orientation rotates with the camera so its
    // initial relative pose is preserved — turning your head turns the
    // object the same way. If false, the object keeps its world-space
    // orientation at grab time (looks unstable when you turn).
    bool lockOrientationToCamera = true;
};

// ── Grab request ──────────────────────────────────────────────────────────────

struct GrabRequest {
    int32_t objID = 0;
    GrabMode mode = GrabMode::kCarry;

    // Grip point in world space at the moment of grabbing. Used by future
    // force-drag / handle-push modes to know where on the body the player
    // grabbed. kCarry mode does not use this (the whole body is teleported).
    Vector3 gripPointWorld = Vector3(0.0f);
};

// ── GrabSystem ────────────────────────────────────────────────────────────────

class GrabSystem {
public:
    GrabSystem() = default;

    /// Initialize with required dependencies. Call once at mission load,
    /// after DarkPhysics and ObjectStateMap exist.
    void init(DarkPhysics *physics, ObjectStateMap *states) {
        mPhysics = physics;
        mStates = states;
    }

    /// Replace the carry parameters (e.g. from YAML config or debug console).
    void setCarryParams(const CarryParams &p) { mCarry = p; }
    const CarryParams &getCarryParams() const { return mCarry; }

    /// Begin grabbing an object. Returns true on success.
    /// Fails if: the object has no dynamic body, the requested mode isn't
    /// implemented, or the object is already grabbed.
    bool grab(const GrabRequest &req, const Camera &cam) {
        if (!mPhysics || req.objID == 0) return false;
        if (isGrabbing(req.objID)) return false;
        if (!mPhysics->hasDynamicBody(req.objID)) {
            std::fprintf(stderr,
                "[GrabSystem] grab(%d) skipped: no dynamic body\n", req.objID);
            return false;
        }

        // Only kCarry is implemented today. Future modes log and refuse so
        // callers can detect "not yet supported" without crashing.
        if (req.mode != GrabMode::kCarry) {
            std::fprintf(stderr,
                "[GrabSystem] grab(%d) mode=%d not yet implemented\n",
                req.objID, static_cast<int>(req.mode));
            return false;
        }

        if (!mPhysics->enterHold(req.objID)) return false;

        ActiveGrab g;
        g.objID = req.objID;
        g.mode = req.mode;
        g.gripPointLocal = Vector3(0.0f);  // unused for carry
        g.snapshotWorldRot = lookupBodyRotation(req.objID);
        g.relRotToCamera = computeRelativeRotation(cam, g.snapshotWorldRot);
        mGrabs.push_back(g);

        std::fprintf(stderr,
            "[GrabSystem] grab(%d) ok (mode=carry, %zu active)\n",
            req.objID, mGrabs.size());
        return true;
    }

    /// Release with a raw world-space linear velocity. Escape hatch for
    /// callers that compute their own velocity (e.g. explosions, springs).
    /// Most call sites should use releaseWithPower() instead.
    ///
    /// Returns false (and keeps the grab active) if the physics layer
    /// refuses the release for clearance reasons. Callers should surface
    /// this as "can't drop here — back away from the wall" feedback.
    bool releaseWithVelocity(int32_t objID, const Vector3 &linVel) {
        auto it = std::find_if(mGrabs.begin(), mGrabs.end(),
            [&](const ActiveGrab &g) { return g.objID == objID; });
        if (it == mGrabs.end()) return false;
        if (!mPhysics->releaseFromHold(objID, linVel)) return false;
        mGrabs.erase(it);
        return true;
    }

    /// Release using the original Dark Engine's throw formula:
    ///   velocity = direction * power * launcherMass / (objMass + launcherMass)
    ///
    /// `power` is in world units/sec (matches the engine's `power` parameter
    /// for throws — drop = 0.05, player throw = 30.0). `direction` is a
    /// unit vector (typically camera forward). The mass attenuation models
    /// the player absorbing recoil from launching the object.
    ///
    /// Returns false (and keeps the grab active) if the physics layer
    /// refuses the release for clearance reasons — e.g. the player is
    /// pressed against a wall and there's no room ahead to safely drop
    /// the object. The carrier has to step away before the throw lands.
    bool releaseWithPower(int32_t objID, const Vector3 &direction, float power) {
        auto it = std::find_if(mGrabs.begin(), mGrabs.end(),
            [&](const ActiveGrab &g) { return g.objID == objID; });
        if (it == mGrabs.end()) return false;

        const float objMass = mPhysics->getDynamicBodyMass(objID);
        const float coeff = mLauncherMass / (objMass + mLauncherMass);
        const Vector3 vel = direction * (power * coeff);

        if (!mPhysics->releaseFromHold(objID, vel)) {
            std::fprintf(stderr,
                "[GrabSystem] release(%d) BLOCKED — keeping grab active "
                "(power=%.2f mass=%.1f)\n", objID, power, objMass);
            return false;
        }
        mGrabs.erase(it);

        std::fprintf(stderr,
            "[GrabSystem] release(%d) power=%.2f mass=%.1f coeff=%.3f "
            "vel=(%.2f,%.2f,%.2f) %zu remaining\n",
            objID, power, objMass, coeff,
            vel.x, vel.y, vel.z, mGrabs.size());
        return true;
    }

    /// Release every active grab with the same power + direction. The
    /// common "throw whatever I'm holding" case. Per-object failures
    /// (release blocked by geometry) are silent here — the per-object
    /// log line in releaseWithPower already calls them out.
    /// Returns true if at least one release succeeded.
    bool releaseAllWithPower(const Vector3 &direction, float power) {
        // Iterate over a snapshot since release mutates mGrabs.
        std::vector<int32_t> ids;
        ids.reserve(mGrabs.size());
        for (const auto &g : mGrabs) ids.push_back(g.objID);
        bool any = false;
        for (int32_t id : ids)
            any = releaseWithPower(id, direction, power) || any;
        return any;
    }

    bool isGrabbing(int32_t objID) const {
        for (const auto &g : mGrabs)
            if (g.objID == objID) return true;
        return false;
    }
    bool isGrabbingAny() const { return !mGrabs.empty(); }
    size_t activeCount() const { return mGrabs.size(); }

    /// Returns the first held objID, or 0 if nothing is held. Convenience
    /// for the common "single-object hold" case.
    int32_t firstHeldObjID() const {
        return mGrabs.empty() ? 0 : mGrabs.front().objID;
    }

    /// Per-frame update. Drives every active grab toward its target.
    /// Call once per render frame, after camera position is finalized,
    /// before the physics step (so any contact queries within the step
    /// see the held position).
    void update(const Camera &cam, float /*dt*/) {
        if (mGrabs.empty() || !mPhysics) return;

        const Vector3 camPos(cam.pos[0], cam.pos[1], cam.pos[2]);
        const Vector3 camFwd = cameraForward(cam);
        const Vector3 worldUp(0.0f, 0.0f, 1.0f);

        for (auto &g : mGrabs) {
            switch (g.mode) {
                case GrabMode::kCarry:
                    updateCarry(g, cam, camPos, camFwd, worldUp);
                    break;
                case GrabMode::kForceDrag:
                case GrabMode::kHandlePush:
                    // Not implemented — grab() refused these modes, so we
                    // shouldn't reach here. Guard anyway.
                    break;
            }
        }
    }

private:
    struct ActiveGrab {
        int32_t objID = 0;
        GrabMode mode = GrabMode::kCarry;

        // Grip in body-local space (future: kForceDrag / kHandlePush).
        Vector3 gripPointLocal = Vector3(0.0f);

        // World rotation at grab time (kCarry: used if not locking to camera).
        glm::mat3 snapshotWorldRot = glm::mat3(1.0f);

        // Body rotation expressed in camera-local frame at grab time.
        // Each frame, world rotation = camera_basis_now * relRotToCamera.
        // Preserves the "object held at the same relative angle to my view"
        // feel as the camera turns.
        glm::mat3 relRotToCamera = glm::mat3(1.0f);
    };

    // ── kCarry implementation ────────────────────────────────────────────────

    void updateCarry(const ActiveGrab &g, const Camera &cam,
                     const Vector3 &camPos, const Vector3 &camFwd,
                     const Vector3 &worldUp) {
        const Vector3 target = camPos
            + camFwd * mCarry.holdDistance
            + worldUp * mCarry.heightOffset;

        const glm::mat3 rot = mCarry.lockOrientationToCamera
            ? cameraBasis(cam) * g.relRotToCamera
            : g.snapshotWorldRot;

        // DarkPhysics::setHoldTransform writes both ODE state and the
        // renderer's ObjectStateMap (using applyModelMatrix with the
        // object's stored P$Scale, matching syncDynamicBodies). One source
        // of truth — no need to write the StateMap from here.
        mPhysics->setHoldTransform(g.objID, target, rot);
    }

    // ── Camera math ──────────────────────────────────────────────────────────

    static Vector3 cameraForward(const Camera &cam) {
        // Z-up Dark Engine: forward at yaw=0 is +X.
        const float cp = std::cos(cam.pitch);
        return Vector3(
            std::cos(cam.yaw) * cp,
            std::sin(cam.yaw) * cp,
            std::sin(cam.pitch));
    }

    /// Camera basis as a rotation matrix. Columns are right, forward, up
    /// (Dark Engine Z-up convention). Used to map between camera-local and
    /// world-space rotations.
    static glm::mat3 cameraBasis(const Camera &cam) {
        const float cy = std::cos(cam.yaw);
        const float sy = std::sin(cam.yaw);
        const float cp = std::cos(cam.pitch);
        const float sp = std::sin(cam.pitch);

        // Right vector at yaw=0 is +Y in this convention (forward x worldUp).
        // Right = forward × worldUp ; worldUp = (0,0,1).
        // forward = (cy*cp, sy*cp, sp), worldUp = (0,0,1)
        // forward × worldUp = (sy*cp*1 - sp*0, sp*0 - cy*cp*1, 0) = (sy*cp, -cy*cp, 0)
        // Normalize: divide by cp (assume not near vertical).
        glm::vec3 forward(cy*cp, sy*cp, sp);
        glm::vec3 right(sy, -cy, 0.0f);
        glm::vec3 up = glm::cross(right, forward);

        glm::mat3 b;
        b[0] = right;
        b[1] = forward;
        b[2] = up;
        return b;
    }

    static glm::mat3 computeRelativeRotation(const Camera &cam,
                                             const glm::mat3 &worldRot) {
        // worldRot = camBasis * relRot  ⇒  relRot = camBasis⁻¹ * worldRot.
        // The basis we built is orthonormal, so transpose is the inverse.
        glm::mat3 cb = cameraBasis(cam);
        return glm::transpose(cb) * worldRot;
    }

    // ── Body rotation lookup ─────────────────────────────────────────────────
    //
    // Read the body's current world rotation from ObjectStateMap. For
    // pushable crates this is identity until the body wakes (no override),
    // which is fine — the grab snapshot is only used to compute the
    // camera-relative initial pose, and identity-relative-to-camera
    // produces a sensible carry orientation.

    glm::mat3 lookupBodyRotation(int32_t objID) const {
        if (!mStates) return glm::mat3(1.0f);
        const ObjectState *s = mStates->tryGet(objID);
        if (!s || !s->hasMatrix) return glm::mat3(1.0f);
        return glm::mat3(
            glm::vec3(s->modelMatrix[0],  s->modelMatrix[1],  s->modelMatrix[2]),
            glm::vec3(s->modelMatrix[4],  s->modelMatrix[5],  s->modelMatrix[6]),
            glm::vec3(s->modelMatrix[8],  s->modelMatrix[9],  s->modelMatrix[10]));
    }

    // ── Data ─────────────────────────────────────────────────────────────────

    DarkPhysics *mPhysics = nullptr;

    // Read-only access to the body's pre-grab rotation. The grab system
    // does not write here — DarkPhysics::setHoldTransform owns the per-
    // frame StateMap update, ensuring scale and matrix flag stay
    // consistent with syncDynamicBodies.
    ObjectStateMap *mStates = nullptr;

    CarryParams mCarry;

    // Effective "launcher mass" used in the original-engine momentum
    // attenuation formula coeff = launcherMass / (objMass + launcherMass).
    // Original Thief models the player as roughly 100 kg. Tunable via
    // setLauncherMass() so YAML config can drive it later.
    float mLauncherMass = 100.0f;

    std::vector<ActiveGrab> mGrabs;

public:
    /// Replace the launcher mass used in throw momentum scaling.
    void setLauncherMass(float m) { mLauncherMass = std::max(0.01f, m); }
    float getLauncherMass() const { return mLauncherMass; }
};

} // namespace Darkness
