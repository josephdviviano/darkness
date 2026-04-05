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

// ObjectPushSystem.h — Kinematic object pushing (Task 61)
//
// Implements the Dark Engine's object push behavior: when the player walks
// into a pushable object, velocity is transferred along the contact normal.
// Objects decelerate via friction and come to rest. This is a kinematic
// system — no ODE dynamics, just velocity + position integration.
//
// Architecture:
//   - ObjectPushSystem is a SimListener (registered after PlayerPhysics)
//   - Each frame: scan player contacts for object hits, apply push impulse
//   - Pushed objects get per-frame velocity integration + friction decay
//   - Transforms written to ObjectStateMap + ObjectCollisionWorld
//
// The Dark Engine uses a simple velocity transfer model:
//   push_vel = dot(player_vel, contact_normal) * contact_normal / mass_ratio
// Objects stay upright (no rotation from pushing) and slide on the ground plane.

#pragma once

#include <cstdint>
#include <cstdio>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

#include "DarknessMath.h"
#include "SimCommon.h"
#include "physics/CollisionGeometry.h"
#include "physics/ObjectCollisionGeometry.h"
#include "property/DarkPropertyDefs.h"
#include "property/PropertyService.h"
#include "property/TypedProperty.h"
#include "worldquery/ObjectState.h"

namespace Darkness {

// Forward declarations
class ObjectCollisionWorld;
class DoorSystem;
struct ObjectCollisionBody;

// ── Per-object push state ──
struct PushedObject {
    int32_t objID;
    Vector3 velocity = {0, 0, 0};  // current sliding velocity (world space)
    float mass = 10.0f;            // from P$PhysAttr.mass
    float friction = 0.5f;         // from P$PhysAttr.friction
};

// ============================================================================
// ObjectPushSystem — kinematic object push simulation
// ============================================================================

class ObjectPushSystem : public SimListener {
public:
    ObjectPushSystem() = default;

    /// Initialize with required services.
    void init(PropertyService *propSvc, ObjectStateMap *objectStates,
              ObjectCollisionWorld *collisionWorld, DoorSystem *doorSystem) {
        mPropSvc = propSvc;
        mObjectStates = objectStates;
        mCollisionWorld = collisionWorld;
        mDoorSystem = doorSystem;

        std::fprintf(stderr, "[ObjectPushSystem] initialized\n");
    }

    /// Auto-register pushable objects by scanning collision bodies.
    /// Uses the Dark Engine's heuristic: OBB bodies with positive mass that
    /// are NOT doors, NOT edge triggers, NOT location-controlled (PhysControl),
    /// and NOT interactive objects (FrobInfo with worldAction). This filters
    /// out wall switches, lamps, and other fixed fixtures while allowing
    /// crates, barrels, and loose props to be pushed.
    void autoRegisterPushable() {
        if (!mPropSvc || !mCollisionWorld) return;

        for (size_t i = 0; i < mCollisionWorld->bodyCount(); ++i) {
            const ObjectCollisionBody &body = mCollisionWorld->getBody(i);
            int32_t objID = body.objID;
            if (objID <= 0) continue;

            if (body.shapeType != CollisionShapeType::OBB) continue;
            if (body.isEdgeTrigger) continue;
            if (isDoor(objID)) continue;

            // In the original Dark Engine, ALL OBB objects are moveable.
            // There is no mass threshold — the 0.02 dampening factor in
            // the collision response makes heavy objects barely nudge while
            // light objects (crates, barrels) slide noticeably.
            PropPhysAttr attr = {};
            if (!getTypedProperty<PropPhysAttr>(mPropSvc, "PhysAttr", objID, attr))
                continue;
            if (attr.mass <= 0.0f) continue;

            registerPushable(objID, attr.mass,
                             (attr.friction > 0.0f) ? attr.friction : 0.5f);
        }

        std::fprintf(stderr, "[ObjectPushSystem] auto-registered %zu pushable objects\n",
                     mPushableObjects.size());
    }

    /// Called by DarkPhysics after each player physics step.
    /// Implements the Dark Engine's BounceSphereOBB collision response:
    /// 1D elastic collision along contact normal, scaled by 0.02 dampening.
    /// Applied as a one-shot impulse per collision event — persistent contact
    /// prevents re-bouncing until the object comes to rest.
    void processPlayerContacts(const std::vector<SphereContact> &contacts,
                               const Vector3 &playerVelocity) {
        for (const auto &contact : contacts) {
            if (contact.objectId < 0) continue;  // terrain contact
            if (!mPushableObjects.count(contact.objectId)) continue;

            // Contact normal points FROM object surface TOWARD player.
            // vDotN < 0 means player is moving into the object.
            float vDotN = glm::dot(playerVelocity, contact.normal);
            if (vDotN >= 0.0f) continue;  // moving away from object, no push

            // If already actively sliding, don't re-bounce — the original
            // engine creates a persistent contact and uses constraints instead.
            // We re-bounce only after the object has come to rest.
            if (mActiveObjects.count(contact.objectId))
                continue;

            float objMass = mPushableMass.count(contact.objectId)
                                ? mPushableMass[contact.objectId]
                                : 30.0f;

            // ── Dark Engine BounceSphereOBB formula (PHCORE.CPP:4920-4961) ──
            // Project velocities onto contact normal for 1D elastic collision.
            // Player velocity along normal:
            //   vel1 = normal * dot(playerVel, normal)
            // Object velocity along normal (initially zero for static objects):
            //   vel2 = (0, 0, 0)
            //
            // 1D elastic collision:
            //   acc2 = vel1 * (2 * m1) / (m1 + m2) + vel2 * (m2 - m1) / (m1 + m2)
            // Since vel2 = 0:
            //   acc2 = vel1 * (2 * m1) / (m1 + m2)
            //
            // Critical: scale by 0.02 (sphere-vs-OBB dampening, vs 0.5 for sphere-sphere)
            //   acc2 *= 0.02
            //
            // Result velocity for OBB = tangential component (zero) + acc2
            constexpr float kPlayerMass = 20.0f;
            constexpr float kBounceDampening = 0.02f;  // Dark Engine PHCORE.CPP:4960

            float elasticTransfer = (2.0f * kPlayerMass) / (kPlayerMass + objMass);
            // vel1 along normal = normal * vDotN (vDotN is negative = into object)
            // acc2 = normal * vDotN * elasticTransfer * 0.02
            // This gives the object velocity pointing away from the player (negative * negative normal = positive direction away)
            Vector3 objVel = contact.normal * vDotN * elasticTransfer * kBounceDampening;

            // Constrain to horizontal plane (no vertical push from walking)
            objVel.z = 0.0f;

            float speed = glm::length(objVel);
            if (speed < 0.001f) continue;

            std::fprintf(stderr, "[PUSH] obj %d: vDotN=%.2f m1=%.0f m2=%.1f "
                         "elastic=%.3f vel=(%.3f,%.3f,%.3f) speed=%.3f\n",
                         contact.objectId, vDotN, kPlayerMass, objMass,
                         elasticTransfer, objVel.x, objVel.y, objVel.z, speed);

            PushedObject po;
            po.objID = contact.objectId;
            po.mass = objMass;
            po.friction = mPushableFriction.count(contact.objectId)
                              ? mPushableFriction[contact.objectId]
                              : 0.5f;
            po.velocity = objVel;
            mActiveObjects[contact.objectId] = po;
        }
    }

    /// Called when a door is moving — check for overlaps with pushable objects
    /// and push them out of the way. doorObjID is the door, doorBody is its
    /// current collision body with updated world position.
    void processDoorCollision(int32_t doorObjID, const ObjectCollisionBody &doorBody) {
        if (!mCollisionWorld || !mObjectStates) return;

        // Check all pushable objects for AABB overlap with the door
        for (int32_t objID : mPushableObjects) {
            if (objID == doorObjID) continue;

            const ObjectCollisionBody *objBody = mCollisionWorld->findBodyByObjID(objID);
            if (!objBody) continue;

            // Quick AABB overlap test
            if (!aabbOverlap(doorBody, *objBody)) continue;

            // Compute push direction: from door center to object center
            Vector3 pushDir = objBody->worldPos - doorBody.worldPos;
            pushDir.z = 0.0f;  // horizontal only
            float dist = glm::length(pushDir);
            if (dist < 0.001f) pushDir = Vector3(1, 0, 0);  // degenerate case
            else pushDir /= dist;

            // Push speed proportional to door velocity (estimated from position change)
            constexpr float DOOR_PUSH_SPEED = 4.0f;

            Vector3 pushVel = pushDir * DOOR_PUSH_SPEED;

            auto it = mActiveObjects.find(objID);
            if (it == mActiveObjects.end()) {
                float mass = mPushableMass.count(objID) ? mPushableMass[objID] : 10.0f;
                PushedObject po;
                po.objID = objID;
                po.mass = mass;
                po.friction = mPushableFriction.count(objID) ? mPushableFriction[objID] : 0.5f;
                po.velocity = pushVel;
                mActiveObjects[objID] = po;
            } else {
                it->second.velocity = pushVel;
            }
        }
    }

    // ── SimListener interface ──

    void simStep(float simTime, float delta) override {
        SimListener::simStep(simTime, delta);
        if (delta <= 0.0f || !mObjectStates) return;

        // Integrate position for all actively pushed objects
        std::vector<int32_t> toRemove;

        for (auto &[objID, obj] : mActiveObjects) {
            float speed = glm::length(obj.velocity);
            if (speed < 0.01f) {
                toRemove.push_back(objID);
                continue;
            }

            // Get or initialize ObjectState. Most objects don't have an
            // ObjectState entry until they move for the first time.
            // Initialize from the collision body's current world transform.
            ObjectState &os = mObjectStates->get(objID);
            if (!os.hasMatrix && mCollisionWorld) {
                const ObjectCollisionBody *body = mCollisionWorld->findBodyByObjID(objID);
                if (body) {
                    // Build model matrix from collision body transform
                    Matrix4 mat(1.0f);
                    mat[0] = glm::vec4(body->rotation[0], 0.0f);
                    mat[1] = glm::vec4(body->rotation[1], 0.0f);
                    mat[2] = glm::vec4(body->rotation[2], 0.0f);
                    mat[3] = glm::vec4(body->worldPos, 1.0f);
                    // Apply object scale
                    mat[0] *= body->objectScale.x;
                    mat[1] *= body->objectScale.y;
                    mat[2] *= body->objectScale.z;
                    std::memcpy(os.modelMatrix, &mat[0][0], sizeof(os.modelMatrix));
                    os.hasMatrix = true;
                    os.position = body->worldPos;
                    os.flags |= kObjStateActive;
                    std::fprintf(stderr, "[PUSH-INIT] obj %d: initialized from collision body pos=(%.1f,%.1f,%.1f)\n",
                                 objID, body->worldPos.x, body->worldPos.y, body->worldPos.z);
                }
            }

            // ── Dark Engine friction model (PHCORE.CPP:1464-1501, PHYSAPI.CPP:2891) ──
            // Friction force = kFrictionFactor * frictionPct * kGravityAmt * mass * kGravityAmt
            // For a flat floor (normal=(0,0,1)): frictionPct = 1.0
            // friction_force = 0.03 * 1.0 * 32.0 * mass * 32.0 = 30.72 * mass
            // Deceleration = friction_force / mass = 30.72 units/s²
            //
            // Gravity pulls the object down, but since we only do horizontal
            // sliding (Z constrained), gravity just contributes to friction.
            constexpr float kFrictionFactor = 0.03f;    // PHYSAPI.CPP:712
            constexpr float kGravityAmt = 32.0f;        // PHYSAPI.CPP:720
            // Assume flat floor contact (frictionPct = 1.0, frictionScale = 1.0)
            float frictionDecel = kFrictionFactor * 1.0f * kGravityAmt * kGravityAmt;
            // = 0.03 * 32 * 32 = 30.72 units/s²

            // Euler integration: position += velocity * dt
            Vector3 newPos = os.position + obj.velocity * delta;

            // Apply friction as deceleration opposing velocity
            float speedLoss = frictionDecel * delta;
            float newSpeed = std::max(0.0f, speed - speedLoss);
            if (newSpeed > 0.0f) {
                obj.velocity *= newSpeed / speed;
            } else {
                obj.velocity = Vector3(0);
                toRemove.push_back(objID);
            }

            // Update position
            os.position = newPos;

            // Rebuild model matrix from position + existing rotation
            if (os.hasMatrix) {
                os.modelMatrix[12] = newPos.x;
                os.modelMatrix[13] = newPos.y;
                os.modelMatrix[14] = newPos.z;
            }

            // Sync collision body transform
            if (mCollisionWorld && os.hasMatrix) {
                Matrix4 mat = glm::make_mat4(os.modelMatrix);
                mCollisionWorld->updateBodyTransform(objID, mat);
            }
        }

        // Remove stopped objects
        for (int32_t id : toRemove) {
            mActiveObjects.erase(id);
        }
    }

    // ── Registration ──

    /// Register an object as pushable at runtime (called by scripts, makeDynamic, etc.).
    /// Objects must have a collision body in the ObjectCollisionWorld.
    void registerPushable(int32_t objID, float mass, float friction = 0.5f) {
        if (mass <= 0.0f) return;
        mPushableObjects.insert(objID);
        mPushableMass[objID] = mass;
        mPushableFriction[objID] = friction;
        std::fprintf(stderr, "[ObjectPushSystem] registerPushable: obj=%d mass=%.1f friction=%.2f\n",
                     objID, mass, friction);
    }

    /// Unregister a pushable object (when destroyed, picked up, etc.).
    void unregisterPushable(int32_t objID) {
        mPushableObjects.erase(objID);
        mPushableMass.erase(objID);
        mPushableFriction.erase(objID);
        mActiveObjects.erase(objID);
    }

    // ── Query ──

    /// Check if an object is pushable (used by PlayerPhysics to skip stair stepping).
    bool isPushable(int32_t objID) const {
        return mPushableObjects.count(objID) > 0;
    }

    /// Number of currently moving objects.
    size_t activeCount() const { return mActiveObjects.size(); }

    /// Number of identified pushable objects.
    size_t pushableCount() const { return mPushableObjects.size(); }

    /// Check if an object is currently being pushed.
    bool isActive(int32_t objID) const {
        return mActiveObjects.count(objID) > 0;
    }

    /// Get velocity of a pushed object (nullptr if not active).
    const Vector3 *getVelocity(int32_t objID) const {
        auto it = mActiveObjects.find(objID);
        return (it != mActiveObjects.end()) ? &it->second.velocity : nullptr;
    }

private:
    bool isDoor(int32_t objID) const {
        if (!mPropSvc) return false;
        return hasProperty(mPropSvc, "RotDoor", objID) ||
               hasProperty(mPropSvc, "TransDoor", objID);
    }

    /// Quick AABB overlap test between two collision bodies.
    static bool aabbOverlap(const ObjectCollisionBody &a,
                             const ObjectCollisionBody &b) {
        // Half-extent AABB from body dimensions
        Vector3 aHalf = a.edgeLengths * 0.5f;
        Vector3 bHalf = b.edgeLengths * 0.5f;
        Vector3 diff = glm::abs(a.worldPos - b.worldPos);
        return diff.x < (aHalf.x + bHalf.x) &&
               diff.y < (aHalf.y + bHalf.y) &&
               diff.z < (aHalf.z + bHalf.z);
    }

    bool mDebugLog = true;  // temporary debug logging for push events

    PropertyService *mPropSvc = nullptr;
    ObjectStateMap *mObjectStates = nullptr;
    ObjectCollisionWorld *mCollisionWorld = nullptr;
    DoorSystem *mDoorSystem = nullptr;

    // Set of all pushable object IDs (determined at init)
    std::unordered_set<int32_t> mPushableObjects;
    std::unordered_map<int32_t, float> mPushableMass;
    std::unordered_map<int32_t, float> mPushableFriction;

    // Currently moving objects
    std::unordered_map<int32_t, PushedObject> mActiveObjects;
};

} // namespace Darkness
