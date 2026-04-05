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

// FrobSystem.h — Player interaction with world objects
//
// Casts a short ray from the camera each frame to find the nearest frobbable
// object. When the player right-clicks, dispatches the appropriate action
// (door toggle, script message, pickup, etc.).
//
// The frob ray tests both world geometry (via RayCaster portal BFS) and
// object OBBs (via ObjectCollisionGeometry ray-vs-OBB). The nearest hit
// that has a FrobInfo property or is a door becomes the frob target.
//
// Visual feedback: the frob target's object name is shown as debug text.
// Full HUD integration (crosshair highlight, use prompt) deferred to later.

#pragma once

#include <cstdint>
#include <cmath>
#include <string>
#include <functional>

#include "DarknessMath.h"
#include "DarknessRendererCore.h"
#include "physics/ObjectCollisionGeometry.h"
#include "property/DarkPropertyDefs.h"
#include "property/TypedProperty.h"
#include "property/PropertyService.h"
#include "object/ObjectService.h"
#include "sim/DoorSystem.h"
#include "sim/MessageDispatch.h"

namespace Darkness {

// ── FrobInfo property (from Dark Engine) ──
// P$FrobInfo: 16 bytes, 3 action fields + padding.
// Each field is a bitfield of frob actions for that context.
struct PropFrobInfo {
    uint32_t worldAction;      // actions when frobbed in world
    uint32_t invAction;        // actions when frobbed in inventory
    uint32_t toolAction;       // actions when used as tool
    uint32_t pad;              // zero padding
};

// Frob action flags (from Dark Engine)
static constexpr uint32_t kFrobMove        = 0x001;  // pick up object
static constexpr uint32_t kFrobScript      = 0x002;  // run script
static constexpr uint32_t kFrobDelete      = 0x004;  // delete object
static constexpr uint32_t kFrobIgnore      = 0x008;  // no-op
static constexpr uint32_t kFrobFocusScript = 0x010;  // script on focus
static constexpr uint32_t kFrobDefault     = 0x100;  // default action (use act/react)

/// Default frob distance — matches the original Dark Engine
/// (head_focus_dist2_tol = 64.0, so sqrt(64) = 8.0 world units).
static constexpr float kDefaultFrobDistance = 8.0f;

// ── Frob result ──
struct FrobTarget {
    int32_t objID = 0;            // 0 = no target
    float distance = 0.0f;        // distance to hit point
    Vector3 hitPoint = {0, 0, 0}; // world-space hit point on object
    bool isDoor = false;           // true if target is a door
    uint32_t frobActions = 0;      // worldAction flags from FrobInfo (0 if door)
    std::string name;              // object/archetype name for display
};

// ── FrobSystem ──

class FrobSystem {
public:
    FrobSystem() = default;

    void init(PropertyService *propSvc, ObjectService *objSvc,
              DoorSystem *doorSys, ObjectCollisionWorld *collisionWorld,
              MessageDispatch *msgDispatch = nullptr) {
        mPropSvc = propSvc;
        mObjSvc = objSvc;
        mDoorSys = doorSys;
        mCollisionWorld = collisionWorld;
        mMsgDispatch = msgDispatch;
    }

    /// Set world query for position lookups of objects without collision bodies.
    /// Enables frob targeting of levers, switches, books, and other small objects
    /// that have FrobInfo but no P$PhysType collision body.
    void setWorldQuery(const IWorldQuery *wq) { mWorldQuery = wq; }

    /// Build the frobbable-object cache: scans all positioned objects and stores
    /// those with FrobInfo (via inheritance) that lack collision bodies.
    /// Must be called after buildObjectCollision so collision bodies exist.
    void buildFrobCache() {
        if (!mPropSvc || !mWorldQuery) return;
        mFrobCache.clear();

        // Scan all positioned objects (same set as script instantiation)
        auto allPositioned = getAllObjectsWithProperty(mPropSvc, "Position");
        for (int objID : allPositioned) {
            if (objID <= 0) continue;

            // Skip objects that have collision bodies (handled by ray-vs-OBB)
            if (mCollisionWorld && mCollisionWorld->findBodyByObjID(objID))
                continue;

            // Check FrobInfo via inheritance
            PropFrobInfo frobInfo;
            if (!getTypedProperty<PropFrobInfo>(mPropSvc, "FrobInfo", objID, frobInfo))
                continue;
            if (frobInfo.worldAction == 0 || (frobInfo.worldAction & kFrobIgnore))
                continue;

            FrobCacheEntry entry;
            entry.objID = objID;
            entry.worldAction = frobInfo.worldAction;
            mFrobCache.push_back(entry);
        }
        std::fprintf(stderr, "[FrobSystem] %zu frobbable objects without collision bodies cached\n",
                     mFrobCache.size());
    }

    /// Update frob target each frame. Cast ray from camera and find nearest
    /// frobbable object. Call before render so highlight is current.
    void update(const Camera &cam) {
        mTarget = {};  // clear

        if (!mCollisionWorld) return;

        // Compute ray from camera position along look direction
        float cosPitch = std::cos(cam.pitch);
        Vector3 forward(
            std::cos(cam.yaw) * cosPitch,
            std::sin(cam.yaw) * cosPitch,
            std::sin(cam.pitch));

        Vector3 rayStart(cam.pos[0], cam.pos[1], cam.pos[2]);
        Vector3 rayEnd = rayStart + forward * mFrobDistance;

        // Test all collision bodies against the ray.
        // We iterate all bodies rather than doing spatial lookup because
        // the frob ray is very short (~5 units) and there are typically
        // only a few hundred objects. If this becomes a bottleneck,
        // we can add spatial pre-filtering.
        float bestT = 2.0f;  // > 1.0 means no hit yet

        const auto &bodies = mCollisionWorld->getBodies();
        for (const auto &body : bodies) {
            // Quick AABB pre-test: skip bodies whose AABB is far from ray
            // (the ray is very short so most bodies will be skipped)
            float midX = (body.aabbMin.x + body.aabbMax.x) * 0.5f;
            float midY = (body.aabbMin.y + body.aabbMax.y) * 0.5f;
            float midZ = (body.aabbMin.z + body.aabbMax.z) * 0.5f;
            float dx = midX - rayStart.x;
            float dy = midY - rayStart.y;
            float dz = midZ - rayStart.z;
            float distSq = dx*dx + dy*dy + dz*dz;
            // Skip objects far beyond frob range (generous margin for large objects)
            if (distSq > (mFrobDistance + 10.0f) * (mFrobDistance + 10.0f))
                continue;

            RayOBBResult result = rayVsOBB(rayStart, rayEnd, body);
            if (!result.hit || result.t >= bestT)
                continue;

            // Check if this object is frobbable
            bool isDoor = mDoorSys && mDoorSys->isDoor(body.objID);
            bool hasFrob = false;
            uint32_t frobActions = 0;

            if (!isDoor && mPropSvc) {
                PropFrobInfo frobInfo;
                if (getTypedProperty<PropFrobInfo>(mPropSvc, "FrobInfo",
                                                    body.objID, frobInfo)) {
                    frobActions = frobInfo.worldAction;
                    hasFrob = (frobActions != 0 &&
                               !(frobActions & kFrobIgnore));
                }
            }

            if (!isDoor && !hasFrob)
                continue;  // not frobbable

            // This is the new best frob target
            bestT = result.t;
            mTarget.objID = body.objID;
            mTarget.distance = result.t * mFrobDistance;
            mTarget.hitPoint = result.point;
            mTarget.isDoor = isDoor;
            mTarget.frobActions = frobActions;

            // Get display name
            if (mObjSvc) {
                mTarget.name = mObjSvc->getName(body.objID);
                if (mTarget.name.empty()) {
                    // Fall back to archetype name
                    mTarget.name = "obj " + std::to_string(body.objID);
                }
            }
        }

        // ── Fallback: proximity-based frob for objects without collision bodies ──
        // Levers, switches, books, and other small frobbable objects often lack
        // P$PhysType collision bodies. Uses the pre-built frob cache (built once
        // at init via buildFrobCache) to avoid per-frame property lookups.
        if (mTarget.objID == 0 && mWorldQuery) {
            for (const auto &entry : mFrobCache) {
                // Get object position (from ObjectState or P$Position via IWorldQuery)
                Vector3 objPos = mWorldQuery->getPosition(entry.objID);
                Vector3 toObj = objPos - rayStart;
                float dist = glm::length(toObj);
                if (dist > mFrobDistance || dist < 0.1f)
                    continue;

                // Cone test: object must be roughly in front of the camera
                // (within ~30 degrees of look direction)
                float dotFwd = glm::dot(glm::normalize(toObj), forward);
                if (dotFwd < 0.85f)
                    continue;

                // Parametric distance along ray
                float t = dist / mFrobDistance;
                if (t >= bestT)
                    continue;

                bestT = t;
                mTarget.objID = entry.objID;
                mTarget.distance = dist;
                mTarget.hitPoint = objPos;
                mTarget.isDoor = false;
                mTarget.frobActions = entry.worldAction;
                if (mObjSvc) {
                    mTarget.name = mObjSvc->getName(entry.objID);
                    if (mTarget.name.empty())
                        mTarget.name = "obj " + std::to_string(entry.objID);
                }
            }
        }
    }

    /// Execute frob on the current target. Returns true if action was taken.
    /// Routes through MessageDispatch which handles ControlDevice link traversal —
    /// frobbing a lever sends FrobWorldEnd to the lever, which follows
    /// ControlDevice relations to send TurnOn to linked doors/objects.
    bool executeFrob() {
        if (mTarget.objID == 0) return false;

        // Route through MessageDispatch if available — this handles
        // ControlDevice link traversal (lever→door) and built-in handlers
        if (mMsgDispatch) {
            mMsgDispatch->frobWorldEnd(mTarget.objID, 0 /* player */);
            std::fprintf(stderr, "Frob: %s obj %d (%s) dist=%.1f\n",
                         mTarget.isDoor ? "door" : "object",
                         mTarget.objID, mTarget.name.c_str(),
                         mTarget.distance);
            return true;
        }

        // Fallback: direct door toggle if no message dispatch
        if (mTarget.isDoor && mDoorSys) {
            mDoorSys->activate(mTarget.objID, kDoorToggle);
            return true;
        }

        return false;
    }

    /// Get the current frob target (updated each frame by update()).
    const FrobTarget &getTarget() const { return mTarget; }

    /// Check if there's a valid frob target under the crosshair.
    bool hasTarget() const { return mTarget.objID != 0; }

    /// Set the maximum frob distance (configurable via debug console / YAML).
    void setFrobDistance(float dist) { mFrobDistance = dist; }
    float getFrobDistance() const { return mFrobDistance; }

    /// Set the message dispatch system (for ControlDevice link traversal).
    void setMessageDispatch(MessageDispatch *md) { mMsgDispatch = md; }

private:
    PropertyService *mPropSvc = nullptr;
    ObjectService *mObjSvc = nullptr;
    DoorSystem *mDoorSys = nullptr;
    ObjectCollisionWorld *mCollisionWorld = nullptr;
    MessageDispatch *mMsgDispatch = nullptr;
    const IWorldQuery *mWorldQuery = nullptr;
    FrobTarget mTarget;
    float mFrobDistance = kDefaultFrobDistance;

    // Pre-built cache of frobbable objects without collision bodies
    struct FrobCacheEntry {
        int32_t objID;
        uint32_t worldAction;
    };
    std::vector<FrobCacheEntry> mFrobCache;
};

} // namespace Darkness
