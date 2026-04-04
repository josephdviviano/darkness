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

// PressurePlateSystem.h — Pressure plate simulation
//
// Implements the Dark Engine pressure plate system: objects that depress
// when sufficient weight is placed on them and send script messages to
// linked targets via ControlDevice relations.
//
// Architecture:
//   - PressurePlateSystem is a SimListener: receives simStep() from SimService
//   - Each plate has a PlateState with position, velocity, and state machine
//   - Weight detection: checks if the player is standing on the plate via
//     a callback that queries PlayerPhysics ground contact object IDs
//   - Translation along local -Z axis (down when pressed, up when released)
//   - Script messages: PressurePlateActivating/Active/Deactivating/Inactive
//
// Inspired by the openDarkEngine (OPDE) service architecture (GPLv2).
// Pressure plate algorithm reimplemented from Dark Engine behavior analysis.

#pragma once

#include <cstdint>
#include <cmath>
#include <cstring>
#include <unordered_map>
#include <functional>

#include "DarknessMath.h"
#include "SimCommon.h"
#include "MessageDispatch.h"
#include "object/ObjectService.h"
#include "property/DarkPropertyDefs.h"
#include "property/PropertyService.h"
#include "property/TypedProperty.h"
#include "worldquery/ObjectState.h"

namespace Darkness {

// Forward declarations
class PropertyService;
class ObjectStateMap;
class MessageDispatch;

// ── Pressure plate states (matches Dark Engine ePPState) ──
enum PlateStatus : int32_t {
    kPlateInactive     = 0,  // Plate is up, no weight
    kPlateActive       = 1,  // Plate is down, weight detected
    kPlateDeactivating = 2,  // Plate moving back up
    kPlateActivating   = 3,  // Plate moving down
};

// ── Per-plate runtime state ──
struct PlateState {
    int32_t objID = 0;

    // Config from P$PhysPPlat
    float activationWeight = 0.0f;  // Weight threshold (currently: any player contact)
    float travel = 0.0f;            // Distance plate depresses (world units)
    float speed = 0.0f;             // Movement speed (units/sec)
    float pauseMode = -1.0f;        // -1=immediate, -2=latched, >0=timed hold (seconds)
    bool blockVision = false;

    // State machine
    PlateStatus status = kPlateInactive;
    float currentOffset = 0.0f;     // Current depression offset (0=up, travel=fully down)
    float pauseTimer = 0.0f;        // Countdown for timed hold mode

    // Base transform
    SimTransform base;
    Vector3 localDown{0.0f, 0.0f, -1.0f};  // Local "down" direction (plate depresses along -Z)
    bool hasObjectState = false;
};

// ── Callback types ──
using PlateCollisionCallback = std::function<void(int32_t objID,
                                                   const Matrix4 &worldMatrix)>;

// Callback to check if the player is standing on a given object ID.
// Returns true if the player's ground contact includes this object.
using PlayerOnObjectCallback = std::function<bool(int32_t objID)>;

// ============================================================================
// PressurePlateSystem — manages all pressure plates in the level
// ============================================================================

class PressurePlateSystem : public SimListener {
public:
    PressurePlateSystem() = default;

    // ── Initialization ──

    /// Scan for all objects with P$PhysPPlat and create PlateState entries.
    void init(PropertyService *propSvc, ObjectService *objSvc,
              ObjectStateMap *objectStates,
              const std::unordered_map<int32_t, ObjPlacementInfo> *placements = nullptr) {
        mObjectStates = objectStates;
        mObjSvc = objSvc;
        mPlacements = placements;

        if (!mPlacements) {
            std::fprintf(stderr, "PressurePlateSystem: no placement data — skipping\n");
            return;
        }

        // Scan all positioned concrete objects for P$PhysPPlat (with inheritance)
        for (const auto &[id, placement] : *mPlacements) {
            if (id <= 0) continue;

            PropPhysPPlate prop;
            if (!getTypedProperty<PropPhysPPlate>(propSvc, "PhysPPlat", id, prop))
                continue;

            PlateState plate;
            plate.objID = id;
            plate.activationWeight = prop.weight;
            plate.travel = prop.travel;
            plate.speed = prop.speed;
            plate.pauseMode = prop.pause;
            plate.blockVision = (prop.blockVision != 0);
            plate.status = kPlateInactive;
            plate.currentOffset = 0.0f;

            // Get base transform
            initBaseTransform(plate, propSvc, id);

            // Compute local "down" direction from the plate's orientation.
            // The plate depresses along its local -Z axis.
            Matrix3 rot3 = Matrix3(plate.base.rotation);
            plate.localDown = -rot3[2];  // -Z column of rotation matrix

            std::fprintf(stderr, "  PressurePlate %d: weight=%.1f travel=%.2f "
                         "speed=%.1f pause=%.1f pos=(%.1f,%.1f,%.1f)\n",
                         id, plate.activationWeight, plate.travel,
                         plate.speed, plate.pauseMode,
                         plate.base.position.x, plate.base.position.y,
                         plate.base.position.z);

            mPlates[id] = std::move(plate);
        }

        std::fprintf(stderr, "PressurePlateSystem: %zu pressure plates\n", mPlates.size());
    }

    // ── Queries ──

    bool isPlate(int32_t objID) const {
        return mPlates.find(objID) != mPlates.end();
    }

    PlateStatus getStatus(int32_t objID) const {
        auto it = mPlates.find(objID);
        return (it != mPlates.end()) ? it->second.status : kPlateInactive;
    }

    // ── Callbacks ──

    void setCollisionUpdateCallback(PlateCollisionCallback cb) {
        mCollisionUpdateCb = std::move(cb);
    }

    void setPlayerOnObjectCallback(PlayerOnObjectCallback cb) {
        mPlayerOnObjectCb = std::move(cb);
    }

    void setMessageDispatch(MessageDispatch *msgDispatch) {
        mMsgDispatch = msgDispatch;
    }

    // ── SimListener interface ──

    void simStep(float simTime, float delta) override {
        if (delta <= 0.0f) return;
        for (auto &[id, plate] : mPlates) {
            updatePlate(plate, delta);
        }
    }

private:
    // ── Per-frame plate update (state machine) ──

    void updatePlate(PlateState &plate, float dt) {
        bool weightDetected = checkWeight(plate);

        switch (plate.status) {
        case kPlateInactive:
            if (weightDetected) {
                // Start depressing
                plate.status = kPlateActivating;
                sendMessage(plate, kPlateActivating);
                ensureObjectState(plate);
                std::fprintf(stderr, "  PressurePlate %d: activating\n", plate.objID);
            }
            break;

        case kPlateActivating:
            // Move plate down
            plate.currentOffset += plate.speed * dt;
            if (plate.currentOffset >= plate.travel) {
                // Fully depressed
                plate.currentOffset = plate.travel;
                plate.status = kPlateActive;
                // Initialize pause timer for timed mode
                if (plate.pauseMode > 0.0f)
                    plate.pauseTimer = plate.pauseMode;
                sendMessage(plate, kPlateActive);
                std::fprintf(stderr, "  PressurePlate %d: active (fully depressed)\n",
                             plate.objID);
            }
            applyTransform(plate);
            break;

        case kPlateActive:
            // Check if plate should deactivate based on pause mode
            if (plate.pauseMode < -1.5f) {
                // Latched mode (pause == -2): stay active indefinitely
                // Only deactivates via script TurnOff (not implemented yet)
            } else if (plate.pauseMode < -0.5f) {
                // Immediate mode (pause == -1): deactivate when weight removed
                if (!weightDetected) {
                    startDeactivating(plate);
                }
            } else {
                // Timed mode (pause > 0): count down, then deactivate
                if (!weightDetected) {
                    plate.pauseTimer -= dt;
                    if (plate.pauseTimer <= 0.0f) {
                        startDeactivating(plate);
                    }
                } else {
                    // Reset timer while weight is still on
                    plate.pauseTimer = plate.pauseMode;
                }
            }
            break;

        case kPlateDeactivating:
            // Move plate back up
            plate.currentOffset -= plate.speed * dt;
            if (plate.currentOffset <= 0.0f) {
                // Fully retracted
                plate.currentOffset = 0.0f;
                plate.status = kPlateInactive;
                sendMessage(plate, kPlateInactive);
                std::fprintf(stderr, "  PressurePlate %d: inactive (retracted)\n",
                             plate.objID);
            }
            applyTransform(plate);
            break;
        }
    }

    void startDeactivating(PlateState &plate) {
        plate.status = kPlateDeactivating;
        sendMessage(plate, kPlateDeactivating);
        std::fprintf(stderr, "  PressurePlate %d: deactivating\n", plate.objID);
    }

    // ── Weight detection ──

    /// Check if enough weight is on the plate. Currently simplified to
    /// checking if the player is standing on the plate object. The original
    /// engine recursively accumulates mass of all objects resting on the
    /// plate's top face — this will be expanded with ODE integration (Task 60).
    bool checkWeight(const PlateState &plate) const {
        if (!mPlayerOnObjectCb) return false;
        return mPlayerOnObjectCb(plate.objID);
    }

    // ── Transform helpers ──

    void initBaseTransform(PlateState &plate, PropertyService *propSvc,
                           int32_t objID) {
        static constexpr float kAngScale = 2.0f * 3.14159265f / 65536.0f;

        if (mPlacements) {
            auto it = mPlacements->find(objID);
            if (it != mPlacements->end()) {
                const auto &pl = it->second;
                plate.base.position = Vector3(pl.x, pl.y, pl.z);
                plate.base.scale = Vector3(pl.sx, pl.sy, pl.sz);

                float h   = static_cast<float>(pl.heading) * kAngScale;
                float pit = static_cast<float>(pl.pitch)   * kAngScale;
                float b   = static_cast<float>(pl.bank)    * kAngScale;
                plate.base.rotation = Matrix4(glm::mat3(
                    glm::eulerAngleZYX(h, pit, b)));
                return;
            }
        }
        if (mObjSvc) {
            plate.base.position = mObjSvc->position(objID);
        }
    }

    void ensureObjectState(PlateState &plate) {
        if (plate.hasObjectState || !mObjectStates) return;
        plate.hasObjectState = true;
        applyTransform(plate);
    }

    /// Write the plate's current position to ObjectState.
    /// Plate translates along its local -Z axis by currentOffset.
    void applyTransform(PlateState &plate) {
        if (!mObjectStates) return;

        Vector3 offset = plate.localDown * plate.currentOffset;
        Vector3 worldPos = plate.base.position + offset;

        Matrix4 worldTranslate = glm::translate(Matrix4(1.0f), worldPos);
        Matrix4 scaleMat = glm::scale(Matrix4(1.0f), plate.base.scale);
        Matrix4 fullGlm = worldTranslate * plate.base.rotation * scaleMat;

        applyModelMatrix(*mObjectStates, plate.objID, fullGlm,
                         worldPos, plate.base.scale);

        if (mCollisionUpdateCb) {
            mCollisionUpdateCb(plate.objID, fullGlm);
        }
    }

    // ── Messaging ──

    void sendMessage(const PlateState &plate, PlateStatus status) {
        if (!mMsgDispatch) return;

        // Send PressurePlate* message names matching Dark Engine convention.
        // Scripts (e.g. StdPressurePlate) listen for these and route TurnOn/TurnOff
        // via ControlDevice links. Without scripts, we send TurnOn/TurnOff
        // directly via MessageDispatch for connected objects.
        const char *msgName = nullptr;
        switch (status) {
        case kPlateActivating:   msgName = "PressurePlateActivating"; break;
        case kPlateActive:       msgName = "PressurePlateActive"; break;
        case kPlateDeactivating: msgName = "PressurePlateDeactivating"; break;
        case kPlateInactive:     msgName = "PressurePlateInactive"; break;
        }

        if (msgName) {
            // Send the specific pressure plate message
            sendScriptMessage(plate.objID, msgName);

            // Also send TurnOn when fully active, TurnOff when fully inactive,
            // following SwitchLinks. This provides basic functionality without
            // the script system (Phase 6).
            if (status == kPlateActive) {
                sendTurnOn(plate.objID);
            } else if (status == kPlateInactive) {
                sendTurnOff(plate.objID);
            }
        }
    }

    void sendScriptMessage(int32_t objID, const char *msgName) {
        if (mMsgDispatch)
            mMsgDispatch->sendMessage({objID, msgName, objID, {}});
    }

    void sendTurnOn(int32_t objID) {
        if (mMsgDispatch)
            mMsgDispatch->turnOn(objID, objID);
    }

    void sendTurnOff(int32_t objID) {
        if (mMsgDispatch)
            mMsgDispatch->turnOff(objID, objID);
    }

    // ── Data ──
    std::unordered_map<int32_t, PlateState> mPlates;
    ObjectStateMap *mObjectStates = nullptr;
    ObjectService *mObjSvc = nullptr;
    const std::unordered_map<int32_t, ObjPlacementInfo> *mPlacements = nullptr;
    PlateCollisionCallback mCollisionUpdateCb;
    PlayerOnObjectCallback mPlayerOnObjectCb;
    MessageDispatch *mMsgDispatch = nullptr;
};

} // namespace Darkness
