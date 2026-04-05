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

class ScriptManager;  // forward declaration — defined in sim/ScriptManager.h

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

    /// Build the frobbable-object cache from the placement map.
    /// Scans all positioned objects and stores those with FrobInfo (via
    /// inheritance) that lack collision bodies — includes their positions
    /// from the allPlacements map (reliable, unlike PropertyService Variant
    /// access which may fail for P$Position's binary format).
    template <typename PlacementMap>
    void buildFrobCache(const PlacementMap &allPlacements) {
        if (!mPropSvc) return;
        mFrobCache.clear();

        int frobCount = 0, doorCount = 0;
        for (const auto &[objID, placement] : allPlacements) {
            if (objID <= 0) continue;

            bool isDoor = mDoorSys && mDoorSys->isDoor(objID);

            // Non-doors: check FrobInfo via inheritance
            if (!isDoor) {
                PropFrobInfo frobInfo;
                if (!getTypedProperty<PropFrobInfo>(mPropSvc, "FrobInfo", objID, frobInfo))
                    continue;
                if (frobInfo.worldAction == 0 || (frobInfo.worldAction & kFrobIgnore))
                    continue;

                FrobCacheEntry entry;
                entry.objID = objID;
                entry.worldAction = frobInfo.worldAction;
                entry.position = Vector3(placement.x, placement.y, placement.z);
                mFrobCache.push_back(entry);
                frobCount++;
            } else {
                // Doors: always frobbable (worldAction unused, isDoor flag handles it)
                FrobCacheEntry entry;
                entry.objID = objID;
                entry.worldAction = kFrobScript;
                entry.position = Vector3(placement.x, placement.y, placement.z);
                mFrobCache.push_back(entry);
                doorCount++;
            }
        }
        std::fprintf(stderr, "[FrobSystem] cached %d frobbable + %d doors = %zu total\n",
                     frobCount, doorCount, mFrobCache.size());
    }

    /// Update frob target each frame. Cast ray from camera and find nearest
    /// frobbable object. Call before render so highlight is current.
    void update(const Camera &cam) {
        mTarget = {};  // clear

        // Compute ray from camera position along look direction
        float cosPitch = std::cos(cam.pitch);
        Vector3 forward(
            std::cos(cam.yaw) * cosPitch,
            std::sin(cam.yaw) * cosPitch,
            std::sin(cam.pitch));

        Vector3 rayStart(cam.pos[0], cam.pos[1], cam.pos[2]);
        Vector3 rayEnd = rayStart + forward * mFrobDistance;

        // Test all collision bodies against the ray (if collision world available).
        float bestT = 2.0f;  // > 1.0 means no hit yet

        if (mCollisionWorld) {
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
        } // end mCollisionWorld

        // ── Proximity-based frob for all frobbable objects ──
        // Most frobbable objects (levers, switches, books, doors) are small and
        // may lack collision bodies. Use the pre-built frob cache with a cone
        // test from the crosshair direction. This also covers doors (which are
        // brush geometry without OBB collision bodies).
        if (!mFrobCache.empty()) {
            // Debug: log closest object stats every N frames
            mDbgFrame++;
            float dbgClosestDist = 999.0f;
            int dbgInRange = 0;
            int dbgInCone = 0;

            for (const auto &entry : mFrobCache) {
                Vector3 objPos = entry.position;
                Vector3 toObj = objPos - rayStart;
                float dist = glm::length(toObj);

                if (dist < dbgClosestDist)
                    dbgClosestDist = dist;

                if (dist > mFrobDistance || dist < 0.1f)
                    continue;
                dbgInRange++;

                // Cone test: object must be roughly in front of the camera
                // (within ~30 degrees of look direction for crosshair targeting)
                float dotFwd = glm::dot(glm::normalize(toObj), forward);
                if (dotFwd < 0.85f)  // ~30 degrees
                    continue;
                dbgInCone++;

                // Parametric distance along ray
                float t = dist / mFrobDistance;
                if (t >= bestT)
                    continue;

                bestT = t;
                mTarget.objID = entry.objID;
                mTarget.distance = dist;
                mTarget.hitPoint = objPos;
                mTarget.isDoor = (mDoorSys && mDoorSys->isDoor(entry.objID));
                mTarget.frobActions = entry.worldAction;
                if (mObjSvc) {
                    mTarget.name = mObjSvc->getName(entry.objID);
                    if (mTarget.name.empty())
                        mTarget.name = "obj " + std::to_string(entry.objID);
                }
            }

            if (mDbgFrame % 120 == 0) {
                std::fprintf(stderr, "[FrobDbg] pos=(%.1f,%.1f,%.1f) fwd=(%.2f,%.2f,%.2f) "
                             "frobDist=%.1f closestObj=%.1f inRange=%d inCone=%d target=%d\n",
                             rayStart.x, rayStart.y, rayStart.z,
                             forward.x, forward.y, forward.z,
                             mFrobDistance, dbgClosestDist,
                             dbgInRange, dbgInCone, mTarget.objID);
            }
        }
    }

    /// Execute frob on the current target. Returns true if action was taken.
    /// Routes through MessageDispatch which handles ControlDevice link traversal —
    /// frobbing a lever sends FrobWorldEnd to the lever, which follows
    /// ControlDevice relations to send TurnOn to linked doors/objects.
    bool executeFrob() {
        if (mTarget.objID == 0) return false;

        std::fprintf(stderr, "Frob: %s obj %d (%s) dist=%.1f actions=0x%x hasScripts=%d\n",
                     mTarget.isDoor ? "door" : "object",
                     mTarget.objID, mTarget.name.c_str(),
                     mTarget.distance, mTarget.frobActions,
                     mScriptManager ? (int)mScriptManager->hasScripts(mTarget.objID) : -1);

        // Route through ScriptManager if available — scripts get first crack
        // at FrobWorldEnd, then fall through to global MessageDispatch handlers.
        // ScriptManager also handles ControlDevice link propagation.
        if (mScriptManager) {
            ScriptMessage msg;
            msg.to = mTarget.objID;
            msg.name = "FrobWorldEnd";
            msg.from = 0;  // player
            mScriptManager->sendMessageWithLinks(msg);
            return true;
        }

        // Fallback: route through MessageDispatch directly (no script support)
        if (mMsgDispatch) {
            mMsgDispatch->frobWorldEnd(mTarget.objID, 0);
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

    /// Set the script manager for routing frobs through scripts.
    /// When set, FrobWorldEnd goes through ScriptManager (scripts get first
    /// crack) instead of directly to MessageDispatch.
    void setScriptManager(ScriptManager *sm) { mScriptManager = sm; }

private:
    PropertyService *mPropSvc = nullptr;
    ObjectService *mObjSvc = nullptr;
    DoorSystem *mDoorSys = nullptr;
    ObjectCollisionWorld *mCollisionWorld = nullptr;
    MessageDispatch *mMsgDispatch = nullptr;
    ScriptManager *mScriptManager = nullptr;
    const IWorldQuery *mWorldQuery = nullptr;
    FrobTarget mTarget;
    float mFrobDistance = kDefaultFrobDistance;
    int mDbgFrame = 0;  // debug frame counter for periodic logging

    // Pre-built cache of frobbable objects (includes doors)
    struct FrobCacheEntry {
        int32_t objID;
        uint32_t worldAction;
        Vector3 position;  // from allPlacements at load time
    };
    std::vector<FrobCacheEntry> mFrobCache;
};

} // namespace Darkness
