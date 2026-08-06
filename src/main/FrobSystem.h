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
// The frob ray tests object OBBs (via ObjectCollisionGeometry ray-vs-OBB).
// The nearest hit that has a FrobInfo property or is a door becomes the frob
// target. World geometry is NOT tested — nothing here consults RayCaster — so
// there is no line-of-sight check and a frobbable close behind a thin wall is
// still reachable. Flagged as a placeholder in DESIGN_DOC.md.
//
// Doors get their own exact ray-vs-leaf test (rayVsDoorLeaf) rather than the
// proximity cone the other cached frobbables use. A door is the one frobbable
// that TRAVELS: a swung leaf ends up a door's width away from where it was
// placed, and its ObjectState position stays at the hinge-anchored base by
// design (DoorSystem::applyDoorTransform), so neither the placement cache nor
// the live query position points at what the player is looking at.
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
#include "worldquery/ObjectState.h"

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

    /// Live position source for the proximity-cone frob path. The cache
    /// stores the load-time placement, but objects can move at runtime
    /// (dropped/thrown crates, doors, etc.). When set, the cone loop uses
    /// the StateMap's current position whenever it has one — falls back
    /// to the cached placement only for objects that have never moved.
    /// Phase 2 will replace the proximity cone with a mesh-padded raycast,
    /// at which point this setter and the cache both go away.
    void setObjectStates(const ObjectStateMap *states) { mStates = states; }

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
                entry.isDoor = true;
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
            setTarget(body.objID, result.t * mFrobDistance, result.point,
                      isDoor, frobActions);
        }
        } // end mCollisionWorld

        // ── Cached frobbables: doors exactly, everything else by cone ──
        // Most frobbable objects (levers, switches, books) are small and may
        // lack collision bodies, so they fall back to a cone test from the
        // crosshair direction. Doors take the exact leaf test instead — they
        // move too far from their placement point for a cone around it to
        // mean anything (see rayVsDoorLeaf).
        if (!mFrobCache.empty()) {
            for (const auto &entry : mFrobCache) {
                if (entry.isDoor) {
                    RayOBBResult leafHit;
                    if (rayVsDoorLeaf(entry.objID, rayStart, rayEnd, leafHit)) {
                        if (leafHit.hit && leafHit.t < bestT) {
                            bestT = leafHit.t;
                            setTarget(entry.objID, leafHit.t * mFrobDistance,
                                      leafHit.point, true, entry.worldAction);
                        }
                        continue;  // exact test done — no cone for this door
                    }
                    // No usable leaf bounds. The cone below is all this door
                    // has, and it aims at the placement point — which is the
                    // very bug the exact test exists to fix, so say so.
                    static int boundsWarn = 0;
                    if (boundsWarn++ < 10)
                        std::fprintf(stderr,
                            "[FALLBACK] FrobSystem: door %d has no usable leaf bounds "
                            "(zero edgeLengths) — frob targeting falls back to a cone "
                            "around its CLOSED placement point\n", entry.objID);
                }

                // Prefer the live runtime position if the StateMap has one;
                // fall back to the load-time placement for objects that
                // have never been touched. Without this, dropped or thrown
                // objects remain frobbable from their original spawn point.
                Vector3 objPos = entry.position;
                if (mStates) {
                    if (const ObjectState *s = mStates->tryGet(entry.objID)) {
                        // The composed matrix's origin, not the query
                        // position: a sim system may deliberately answer
                        // position queries with something other than where
                        // the visual sits (a rotating door reports its
                        // hinge-anchored base). What the player aims at is
                        // the visual.
                        objPos = s->hasMatrix
                            ? Vector3(s->modelMatrix[12], s->modelMatrix[13],
                                      s->modelMatrix[14])
                            : s->position;
                    }
                }
                Vector3 toObj = objPos - rayStart;
                float dist = glm::length(toObj);

                if (dist > mFrobDistance || dist < 0.1f)
                    continue;

                // Cone test: object must be roughly in front of the camera
                // (within ~30 degrees of look direction for crosshair targeting)
                float dotFwd = glm::dot(glm::normalize(toObj), forward);
                if (dotFwd < 0.85f)  // ~30 degrees
                    continue;

                // Parametric distance along ray
                float t = dist / mFrobDistance;
                if (t >= bestT)
                    continue;

                bestT = t;
                setTarget(entry.objID, dist, objPos,
                          mDoorSys && mDoorSys->isDoor(entry.objID),
                          entry.worldAction);
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
            std::fprintf(stderr, "[FALLBACK] FrobSystem: no ScriptManager, routing obj %d through MessageDispatch directly\n", mTarget.objID);
            mMsgDispatch->frobWorldEnd(mTarget.objID, 0);
            return true;
        }

        // Fallback: direct door toggle if no message dispatch
        if (mTarget.isDoor && mDoorSys) {
            std::fprintf(stderr, "[FALLBACK] FrobSystem: no ScriptManager or MessageDispatch, direct door toggle obj %d\n", mTarget.objID);
            mDoorSys->activate(mTarget.objID, kDoorToggle);
            return true;
        }

        std::fprintf(stderr, "[FALLBACK] FrobSystem: no frob handler available for obj %d (no ScriptMgr, no MsgDispatch, %s)\n",
                     mTarget.objID, mTarget.isDoor ? "isDoor but no DoorSys" : "not a door");
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
    /// Ray-vs-door-leaf at the door's CURRENT pose.
    ///
    /// DoorSystem already publishes the leaf box its audio occluder uses —
    /// edgeLengths in final world size, paired with a rigid (unscaled)
    /// transform. Testing frob against that same box means the slab that
    /// blocks sound and the slab the player can reach are one object in one
    /// place, at every point in the swing.
    ///
    /// Returns false when the door has no usable bounds, leaving the caller
    /// to fall back to the proximity cone.
    bool rayVsDoorLeaf(int32_t objID, const Vector3 &rayStart,
                       const Vector3 &rayEnd, RayOBBResult &out) const {
        if (!mDoorSys) return false;
        const DoorState *door = mDoorSys->getDoor(objID);
        if (!door || glm::length(door->edgeLengths) < 0.01f)
            return false;

        // The leaf box is centred on the model origin — the same assumption
        // DoorSystem's audio occluder makes (buildBoxMesh centres on the
        // origin), and what a pivotOffset of -width/2 encodes.
        out = rayVsBoxPose(mDoorSys->getCurrentWorldMatrix(objID),
                           door->edgeLengths,  // already post-scale world size
                           rayStart, rayEnd);
        return true;
    }

    /// Record the frob target, resolving its display name.
    void setTarget(int32_t objID, float distance, const Vector3 &hitPoint,
                   bool isDoor, uint32_t frobActions) {
        mTarget.objID = objID;
        mTarget.distance = distance;
        mTarget.hitPoint = hitPoint;
        mTarget.isDoor = isDoor;
        mTarget.frobActions = frobActions;

        if (!mObjSvc) return;
        mTarget.name = mObjSvc->getName(objID);
        if (mTarget.name.empty()) {
            mTarget.name = "obj " + std::to_string(objID);
            static int w = 0; if (w++ < 10)
                std::fprintf(stderr, "[DEFAULT] FrobSystem: obj %d has no name, using fallback '%s'\n",
                             objID, mTarget.name.c_str());
        }
    }

    PropertyService *mPropSvc = nullptr;
    ObjectService *mObjSvc = nullptr;
    DoorSystem *mDoorSys = nullptr;
    ObjectCollisionWorld *mCollisionWorld = nullptr;
    MessageDispatch *mMsgDispatch = nullptr;
    ScriptManager *mScriptManager = nullptr;
    const IWorldQuery *mWorldQuery = nullptr;
    const ObjectStateMap *mStates = nullptr;
    FrobTarget mTarget;
    float mFrobDistance = kDefaultFrobDistance;

    // Pre-built cache of frobbable objects (includes doors)
    struct FrobCacheEntry {
        int32_t objID;
        uint32_t worldAction;
        Vector3 position;   // from allPlacements at load time
        bool isDoor = false;  // doors take the exact leaf test, not the cone
    };
    std::vector<FrobCacheEntry> mFrobCache;
};

} // namespace Darkness
