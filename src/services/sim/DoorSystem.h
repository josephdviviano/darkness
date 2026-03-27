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

// DoorSystem.h — Rotating and translating door simulation
//
// Implements the Dark Engine door system: velocity-controlled rotation/translation
// with open/close limits, sound blocking, and vision blocking. Doors are driven
// by script messages (TurnOn/TurnOff) or frob interaction.
//
// Architecture:
//   - DoorSystem is a SimListener: receives simStep(simTime, dt) from SimService
//   - Each door has a DoorState with current angle/position, velocity, and status
//   - DoorState writes to ObjectStateMap so the renderer sees updated transforms
//   - Door open/close events update portal blocking for audio and visibility
//
// Inspired by the openDarkEngine (OPDE) service architecture (GPLv2).
// Door physics algorithm reimplemented from Dark Engine behavior analysis.

#pragma once

#include <cstdint>
#include <cmath>
#include <unordered_map>
#include <functional>

#include "DarknessMath.h"
#include "SimCommon.h"
#include "object/ObjectService.h"
#include "property/DarkPropertyDefs.h"
#include "property/PropertyService.h"
#include "property/TypedProperty.h"
#include "worldquery/ObjectState.h"

namespace Darkness {

// Forward declarations
class PropertyService;
class ObjectStateMap;

// ── Door status (matches Dark Engine eDoorStatus) ──
enum DoorStatus : int32_t {
    kDoorClosed  = 0,
    kDoorOpen    = 1,
    kDoorClosing = 2,
    kDoorOpening = 3,
    kDoorHalt    = 4,
};

// ── Door activation commands ──
enum DoorAction : int32_t {
    kDoorToggle = 0,
    kDoorDoOpen = 1,
    kDoorDoClose = 2,
};

// ── Door type ──
enum DoorType : int32_t {
    kDoorRotating    = 0,
    kDoorTranslating = 1,
};

// ── Per-door runtime state ──
struct DoorState {
    int32_t objID = 0;
    DoorType type = kDoorRotating;
    DoorStatus status = kDoorClosed;

    // Axis of rotation/translation (0=X, 1=Y, 2=Z)
    int axis = 2;  // Z is most common for rotating doors

    // Movement parameters (in radians for rotation, world units for translation)
    float closedValue = 0.0f;    // angle or distance when closed
    float openValue   = 0.0f;    // angle or distance when open
    float speed       = 0.0f;    // radians/sec or units/sec
    float currentValue = 0.0f;   // current angle or distance from base
    float velocity    = 0.0f;    // current movement rate (signed)

    bool clockwise = false;      // rotation direction (rotating doors only)
    bool hardLimits = true;

    // Blocking
    float soundBlocking = 0.0f;  // 0.0-1.0 blocking factor when closed
    bool visionBlocking = false;
    int32_t room1 = -1, room2 = -1;
    float pushMass = 0.0f;

    // Base transform (object's static position from P$Position)
    Vector3 basePosition = {0.0f, 0.0f, 0.0f};
    float baseHeading = 0.0f;    // radians
    float basePitch   = 0.0f;    // radians
    float baseBank    = 0.0f;    // radians
    Vector3 baseScale = {1.0f, 1.0f, 1.0f};

    // Renderer index — index into mission.objData.objects[] for quick lookup.
    // -1 if not found (archetype-only door, shouldn't happen for concrete objects).
    int renderIndex = -1;
};

// ── Callback for door events (sound blocking, script messages, etc.) ──
using DoorEventCallback = std::function<void(int32_t objID, DoorStatus newStatus,
                                              const DoorState &door)>;

// ============================================================================
// DoorSystem — manages all doors in the level
// ============================================================================

class DoorSystem : public SimListener {
public:
    DoorSystem() = default;

    // ── Initialization ──

    /// Scan for all objects with P$RotDoor or P$TransDoor properties and
    /// create DoorState entries. Call after level load, before sim starts.
    void init(PropertyService *propSvc, ObjectService *objSvc,
              ObjectStateMap *objectStates) {
        mObjectStates = objectStates;
        mObjSvc = objSvc;

        // Scan for rotating doors
        initDoorsOfType<PropRotDoor>(propSvc, "RotDoor", kDoorRotating);

        // Scan for translating doors
        initDoorsOfType<PropTransDoor>(propSvc, "TransDoor", kDoorTranslating);

        std::fprintf(stderr, "DoorSystem: %zu doors initialized (%zu rotating, %zu translating)\n",
                     mDoors.size(),
                     std::count_if(mDoors.begin(), mDoors.end(),
                         [](const auto &p) { return p.second.type == kDoorRotating; }),
                     std::count_if(mDoors.begin(), mDoors.end(),
                         [](const auto &p) { return p.second.type == kDoorTranslating; }));
    }

    // ── Door control ──

    /// Activate a door: open, close, or toggle.
    bool activate(int32_t objID, DoorAction action) {
        auto it = mDoors.find(objID);
        if (it == mDoors.end()) return false;

        DoorState &door = it->second;
        switch (action) {
        case kDoorDoOpen:
            startOpening(door);
            break;
        case kDoorDoClose:
            startClosing(door);
            break;
        case kDoorToggle:
            if (door.status == kDoorOpen || door.status == kDoorOpening)
                startClosing(door);
            else
                startOpening(door);
            break;
        }
        return true;
    }

    /// Get the current status of a door.
    DoorStatus getStatus(int32_t objID) const {
        auto it = mDoors.find(objID);
        return (it != mDoors.end()) ? it->second.status : kDoorClosed;
    }

    /// Get the open fraction (0.0 = closed, 1.0 = open) for portal blocking.
    float getOpenFraction(int32_t objID) const {
        auto it = mDoors.find(objID);
        if (it == mDoors.end()) return 1.0f;  // unknown = fully open
        const DoorState &door = it->second;
        float range = door.openValue - door.closedValue;
        if (std::abs(range) < 1e-6f) return 1.0f;
        return std::clamp((door.currentValue - door.closedValue) / range, 0.0f, 1.0f);
    }

    /// Get the open fraction for a portal between two rooms.
    /// Returns 1.0 (fully open) if no door exists between those rooms.
    float getOpenFractionForRooms(int32_t room1, int32_t room2) const {
        uint64_t key = roomPairKey(room1, room2);
        auto it = mRoomPairToDoor.find(key);
        if (it == mRoomPairToDoor.end()) return 1.0f;
        return getOpenFraction(it->second);
    }

    /// Get the sound blocking factor (0.0-1.0) for a portal between two rooms.
    /// Returns 0.0 if no door or door is fully open.
    float getSoundBlockingForRooms(int32_t room1, int32_t room2) const {
        uint64_t key = roomPairKey(room1, room2);
        auto it = mRoomPairToDoor.find(key);
        if (it == mRoomPairToDoor.end()) return 0.0f;
        auto doorIt = mDoors.find(it->second);
        if (doorIt == mDoors.end()) return 0.0f;
        const DoorState &door = doorIt->second;
        // Blocking scales inversely with open fraction: closed=full blocking, open=none
        float openFrac = getOpenFraction(door.objID);
        return door.soundBlocking * (1.0f - openFrac);
    }

    /// Check if an object is a door.
    bool isDoor(int32_t objID) const { return mDoors.find(objID) != mDoors.end(); }

    /// Get a door's state (for collision, etc.). Returns nullptr if not a door.
    const DoorState *getDoor(int32_t objID) const {
        auto it = mDoors.find(objID);
        return (it != mDoors.end()) ? &it->second : nullptr;
    }

    /// Set callback for door open/close events (audio blocking, script messages).
    void setEventCallback(DoorEventCallback cb) { mEventCallback = std::move(cb); }

    /// Set callback for continuous audio blocking updates during door animation.
    /// Called per-frame with (room1, room2, blockingFactor) while a door is moving.
    using AudioBlockingCallback = std::function<void(int32_t room1, int32_t room2, float factor)>;
    void setAudioBlockingCallback(AudioBlockingCallback cb) { mAudioBlockingCallback = std::move(cb); }

    /// Get all door IDs (for debug enumeration).
    std::vector<int32_t> getAllDoorIDs() const {
        std::vector<int32_t> ids;
        ids.reserve(mDoors.size());
        for (const auto &[id, _] : mDoors) ids.push_back(id);
        return ids;
    }

    // ── SimListener interface ──

    void simStep(float simTime, float delta) override {
        if (delta <= 0.0f) return;
        for (auto &[id, door] : mDoors) {
            if (door.status == kDoorOpening || door.status == kDoorClosing) {
                updateDoor(door, delta);

                // Continuously update audio blocking during animation.
                // This is called via the mAudioBlockingCallback which is
                // set by the renderer to call AudioService::setBlockingFactor.
                if (mAudioBlockingCallback &&
                    door.room1 >= 0 && door.room2 >= 0 &&
                    door.room1 != door.room2 && door.soundBlocking > 0.0f) {
                    float openFrac = getOpenFraction(id);
                    float blocking = door.soundBlocking * (1.0f - openFrac);
                    mAudioBlockingCallback(door.room1, door.room2, blocking);
                }
            }
        }
    }

private:
    // ── Door initialization helpers ──

    template <typename PropT>
    void initDoorsOfType(PropertyService *propSvc, const char *propName,
                          DoorType doorType) {
        // Find all objects with this door property
        auto ids = Darkness::getAllObjectsWithProperty(propSvc, propName);
        for (int id : ids) {
            if (id <= 0) continue;  // skip archetypes

            PropT prop;
            if (!getTypedProperty<PropT>(propSvc, propName, id, prop))
                continue;

            DoorState door;
            door.objID = id;
            door.type = doorType;
            initDoorFromProperty(door, prop, doorType);

            // Get object's base position from ObjectService (reads P$Position
            // via PositionPropertyStorage, which handles the disk→quaternion conversion)
            initDoorBaseTransform(door, propSvc, id);

            // Initialize ObjectState so the renderer can read the door's position
            if (mObjectStates) {
                ObjectState &os = mObjectStates->get(id);
                os.position = door.basePosition;
                os.heading = door.baseHeading;
                os.pitch = door.basePitch;
                os.bank = door.baseBank;
                os.scale = door.baseScale;
                os.setAngles(os.heading, os.pitch, os.bank);
                os.flags = kObjStateActive;
            }

            mDoors[id] = door;

            // Build room pair → door mapping for portal queries
            if (door.room1 >= 0 && door.room2 >= 0 && door.room1 != door.room2) {
                mRoomPairToDoor[roomPairKey(door.room1, door.room2)] = id;
            }
        }
    }

    void initDoorFromProperty(DoorState &door, const PropRotDoor &prop, DoorType) {
        static constexpr float kDegToRad = 3.14159265f / 180.0f;
        door.closedValue = prop.closedAngle * kDegToRad;
        door.openValue = prop.openAngle * kDegToRad;
        door.speed = prop.speed * kDegToRad;  // convert deg/sec to rad/sec
        door.axis = prop.axis;
        door.status = static_cast<DoorStatus>(prop.status);
        door.hardLimits = (prop.hardLimits != 0);
        door.soundBlocking = prop.blockSound / 100.0f;
        door.visionBlocking = (prop.blockVision != 0);
        door.pushMass = prop.pushMass;
        door.room1 = prop.room1;
        door.room2 = prop.room2;
        door.clockwise = (prop.clockwise != 0);

        // Initial position: closed angle (most doors start closed)
        if (door.status == kDoorClosed)
            door.currentValue = door.closedValue;
        else if (door.status == kDoorOpen)
            door.currentValue = door.openValue;
        else
            door.currentValue = door.closedValue;  // default
    }

    void initDoorFromProperty(DoorState &door, const PropTransDoor &prop, DoorType) {
        door.closedValue = prop.closed;
        door.openValue = prop.open;
        door.speed = prop.speed;
        door.axis = prop.axis;
        door.status = static_cast<DoorStatus>(prop.status);
        door.hardLimits = (prop.hardLimits != 0);
        door.soundBlocking = prop.blockSound / 100.0f;
        door.visionBlocking = (prop.blockVision != 0);
        door.pushMass = prop.pushMass;
        door.room1 = prop.room1;
        door.room2 = prop.room2;

        // Initial position
        if (door.status == kDoorClosed)
            door.currentValue = door.closedValue;
        else if (door.status == kDoorOpen)
            door.currentValue = door.openValue;
        else
            door.currentValue = door.closedValue;
    }

    void initDoorBaseTransform(DoorState &door, PropertyService *propSvc,
                                int32_t objID) {
        if (!mObjSvc) return;

        // Get position from ObjectService (reads P$Position via PositionPropertyStorage)
        door.basePosition = mObjSvc->position(objID);

        // Get orientation as quaternion, convert to Euler angles.
        // Dark Engine rotation order: Rz(heading) * Ry(pitch) * Rx(bank)
        Quaternion q = mObjSvc->orientation(objID);
        Matrix3 rotMat = glm::mat3_cast(q);
        glm::extractEulerAngleZYX(Matrix4(rotMat), door.baseHeading,
                                   door.basePitch, door.baseBank);

        // P$Scale: direct ownership only (kPropertyNoInherit)
        Vector3 scale(1.0f);
        if (getTypedProperty<Vector3>(propSvc, "Scale", objID, scale)) {
            door.baseScale = scale;
        }
    }

    // ── Door movement ──

    void startOpening(DoorState &door) {
        if (door.status == kDoorOpen) return;

        // Remove blocking when door starts opening
        if (door.status == kDoorClosed) {
            emitEvent(door, kDoorOpening);
        }

        door.status = kDoorOpening;

        // Set velocity direction based on door type
        if (door.type == kDoorRotating) {
            // Rotating door: direction depends on clockwise flag
            float dir = door.clockwise ? -1.0f : 1.0f;
            if (door.openValue < door.closedValue) dir = -dir;
            door.velocity = door.speed * dir;
        } else {
            // Translating door: direction from closed→open
            float dir = (door.openValue > door.closedValue) ? 1.0f : -1.0f;
            door.velocity = door.speed * dir;
        }
    }

    void startClosing(DoorState &door) {
        if (door.status == kDoorClosed) return;

        door.status = kDoorClosing;

        // Reverse velocity direction
        if (door.type == kDoorRotating) {
            float dir = door.clockwise ? 1.0f : -1.0f;
            if (door.openValue < door.closedValue) dir = -dir;
            door.velocity = door.speed * dir;
        } else {
            float dir = (door.openValue > door.closedValue) ? -1.0f : 1.0f;
            door.velocity = door.speed * dir;
        }
    }

    void updateDoor(DoorState &door, float dt) {
        // Guard against zero velocity (e.g. speed=0 in property data) —
        // snap to target immediately instead of getting stuck forever.
        if (std::abs(door.velocity) < 1e-6f) {
            if (door.status == kDoorOpening) {
                door.currentValue = door.openValue;
                door.velocity = 0.0f;
                door.status = kDoorOpen;
                applyDoorTransform(door);
                emitEvent(door, kDoorOpen);
            } else if (door.status == kDoorClosing) {
                door.currentValue = door.closedValue;
                door.velocity = 0.0f;
                door.status = kDoorClosed;
                applyDoorTransform(door);
                emitEvent(door, kDoorClosed);
            }
            return;
        }

        door.currentValue += door.velocity * dt;

        // Check limits
        float minVal = std::min(door.closedValue, door.openValue);
        float maxVal = std::max(door.closedValue, door.openValue);

        bool reachedOpen = false;
        bool reachedClosed = false;

        if (door.status == kDoorOpening) {
            if ((door.velocity > 0 && door.currentValue >= door.openValue) ||
                (door.velocity < 0 && door.currentValue <= door.openValue)) {
                door.currentValue = door.openValue;
                door.velocity = 0.0f;
                door.status = kDoorOpen;
                reachedOpen = true;
            }
        } else if (door.status == kDoorClosing) {
            if ((door.velocity > 0 && door.currentValue >= door.closedValue) ||
                (door.velocity < 0 && door.currentValue <= door.closedValue)) {
                door.currentValue = door.closedValue;
                door.velocity = 0.0f;
                door.status = kDoorClosed;
                reachedClosed = true;
            }
        }

        // Clamp to range (safety)
        door.currentValue = std::clamp(door.currentValue, minVal, maxVal);

        // Update ObjectState transform
        applyDoorTransform(door);

        // Emit events on state change
        if (reachedOpen) emitEvent(door, kDoorOpen);
        if (reachedClosed) emitEvent(door, kDoorClosed);
    }

    void applyDoorTransform(DoorState &door) {
        if (!mObjectStates) return;

        ObjectState &os = mObjectStates->get(door.objID);

        if (door.type == kDoorRotating) {
            // Apply rotation offset around the door's axis.
            // The rotation is relative to the door's base orientation.
            float angleOffset = door.currentValue;
            float h = door.baseHeading;
            float p = door.basePitch;
            float b = door.baseBank;

            // Add angle offset on the appropriate axis
            switch (door.axis) {
            case 0: b += angleOffset; break;  // X axis = bank
            case 1: p += angleOffset; break;  // Y axis = pitch
            case 2: h += angleOffset; break;  // Z axis = heading
            }

            // Compute position offset from rotation around pivot point.
            // For rotating doors, the object rotates about its base position,
            // which is typically the hinge point set in DromEd.
            os.setTransform(door.basePosition, h, p, b);
            os.scale = door.baseScale;

        } else {
            // Translating door: offset position along axis in object-local frame
            float offset = door.currentValue;

            // Transform the axis-aligned offset into world space using the
            // door's base orientation
            Vector3 localOffset(0.0f);
            switch (door.axis) {
            case 0: localOffset.x = offset; break;
            case 1: localOffset.y = offset; break;
            case 2: localOffset.z = offset; break;
            }

            // Rotate offset by base orientation
            Matrix3 rotMat = glm::mat3(glm::eulerAngleZYX(door.baseHeading,
                                                            door.basePitch,
                                                            door.baseBank));
            Vector3 worldOffset = rotMat * localOffset;

            os.setTransform(door.basePosition + worldOffset,
                           door.baseHeading, door.basePitch, door.baseBank);
            os.scale = door.baseScale;
        }
    }

    void emitEvent(const DoorState &door, DoorStatus newStatus) {
        if (mEventCallback)
            mEventCallback(door.objID, newStatus, door);
    }

    // ── Helpers ──

    /// Pack two room IDs into a single key (order-independent).
    static uint64_t roomPairKey(int32_t r1, int32_t r2) {
        if (r1 > r2) std::swap(r1, r2);
        return (static_cast<uint64_t>(static_cast<uint32_t>(r1)) << 32) |
                static_cast<uint64_t>(static_cast<uint32_t>(r2));
    }

    // ── Data ──
    std::unordered_map<int32_t, DoorState> mDoors;
    std::unordered_map<uint64_t, int32_t> mRoomPairToDoor;  // room pair → door objID
    ObjectStateMap *mObjectStates = nullptr;
    ObjectService *mObjSvc = nullptr;
    DoorEventCallback mEventCallback;
    AudioBlockingCallback mAudioBlockingCallback;
};

} // namespace Darkness
