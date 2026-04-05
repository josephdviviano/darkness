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

        // Scan for pushable objects: positive mass, collision body, not door/trigger
        if (!propSvc) return;

        auto allPhysAttr = getAllObjectsWithProperty(propSvc, "PhysAttr");
        for (int objID : allPhysAttr) {
            if (objID <= 0) continue;  // skip archetypes

            PropPhysAttr attr = {};
            if (!getTypedProperty<PropPhysAttr>(propSvc, "PhysAttr", objID, attr))
                continue;

            if (attr.mass <= 0.0f) continue;
            if (attr.edgeTrigger) continue;

            // Skip doors — they have their own movement system
            if (isDoor(objID)) continue;

            mPushableObjects.insert(objID);
            mPushableMass[objID] = attr.mass;
            mPushableFriction[objID] = (attr.friction > 0.0f) ? attr.friction : 0.5f;
        }

        std::fprintf(stderr, "[ObjectPushSystem] %zu pushable objects identified\n",
                     mPushableObjects.size());
    }

    /// Called by DarkPhysics after each player physics step.
    /// Scans player contacts for object hits and applies push impulse.
    void processPlayerContacts(const std::vector<SphereContact> &contacts,
                               const Vector3 &playerVelocity) {
        for (const auto &contact : contacts) {
            if (contact.objectId < 0) continue;  // terrain contact
            if (!mPushableObjects.count(contact.objectId)) continue;

            // Calculate push velocity: project player velocity onto contact normal
            float vDotN = glm::dot(playerVelocity, contact.normal);
            if (vDotN >= 0.0f) continue;  // moving away from object, no push

            // Push strength scales inversely with mass ratio
            // Player mass is assumed ~1.0 (normalized)
            float mass = mPushableMass.count(contact.objectId)
                             ? mPushableMass[contact.objectId]
                             : 10.0f;
            float massRatio = 1.0f / std::max(mass * 0.1f, 0.1f);

            // Transfer velocity along contact normal (negated: normal points
            // away from wall toward player, we want push direction into wall)
            Vector3 pushVel = -contact.normal * vDotN * massRatio;

            // Constrain to horizontal plane (objects don't fly upward from pushes)
            pushVel.z = 0.0f;

            float pushSpeed = glm::length(pushVel);
            if (pushSpeed < 0.1f) continue;  // too weak to bother

            // Clamp push speed to prevent extreme velocities
            constexpr float MAX_PUSH_SPEED = 8.0f;
            if (pushSpeed > MAX_PUSH_SPEED)
                pushVel *= MAX_PUSH_SPEED / pushSpeed;

            // Apply to object state
            auto it = mActiveObjects.find(contact.objectId);
            if (it == mActiveObjects.end()) {
                PushedObject po;
                po.objID = contact.objectId;
                po.mass = mass;
                po.friction = mPushableFriction.count(contact.objectId)
                                  ? mPushableFriction[contact.objectId]
                                  : 0.5f;
                po.velocity = pushVel;
                mActiveObjects[contact.objectId] = po;
            } else {
                // Blend with existing velocity (don't completely override)
                it->second.velocity = it->second.velocity * 0.3f + pushVel * 0.7f;
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

            // Get current position
            ObjectState &os = mObjectStates->get(objID);
            Vector3 newPos = os.position + obj.velocity * delta;

            // Apply friction deceleration
            // Dark Engine uses aggressive friction: objects stop quickly
            float frictionForce = obj.friction * 9.81f;  // gravity-scaled friction
            float speedLoss = frictionForce * delta;
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

    // ── Query ──

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
