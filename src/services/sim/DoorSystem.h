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
#include "BinMeshParser.h"
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

    // Base transform from P$Position. We store the angles as float radians
    // converted directly from the binary radians in the ObjectPlacement data
    // (NOT via quaternion round-trip, which can produce mirrored Euler angles).
    Vector3 basePosition = {0.0f, 0.0f, 0.0f};
    float baseHeading = 0.0f;    // radians (from int16 binary radians)
    float basePitch   = 0.0f;    // radians
    float baseBank    = 0.0f;    // radians
    Vector3 baseScale = {1.0f, 1.0f, 1.0f};
    bool hasObjectState = false;  // true once ObjectState entry is created

    // Pivot offset: vector from hinge to model center in the door's local frame.
    // For rotating doors, the center traces an arc around the hinge. The pivot
    // is computed from the .bin model bounding box — half the door width along
    // the axis perpendicular to the rotation axis.
    Vector3 pivotOffset = {0.0f, 0.0f, 0.0f};

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
    /// parsedModels is used to compute hinge pivot offsets from bounding boxes.
    /// objectPlacements: objID → (index, heading, pitch, bank, x, y, z, sx, sy, sz)
    /// from the mission's ObjectPlacement array — used for raw binary radian angles
    /// that match the static renderer exactly (no quaternion round-trip).
    struct ObjPlacementInfo {
        float x, y, z;
        int16_t heading, pitch, bank;
        float sx, sy, sz;
        char modelName[16];
    };

    void init(PropertyService *propSvc, ObjectService *objSvc,
              ObjectStateMap *objectStates,
              const std::unordered_map<std::string, ParsedBinMesh> *parsedModels = nullptr,
              const std::unordered_map<int32_t, ObjPlacementInfo> *placements = nullptr) {
        mObjectStates = objectStates;
        mObjSvc = objSvc;
        mParsedModels = parsedModels;
        mPlacements = placements;

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
        static constexpr float kTwoPi = 2.0f * 3.14159265f;
        auto it = mDoors.find(objID);
        if (it == mDoors.end()) return 1.0f;
        const DoorState &door = it->second;
        float signedOpen = door.openValue;
        float signedClosed = door.closedValue;
        if (door.type == kDoorRotating && door.clockwise) {
            signedOpen  = (door.openValue > 0.01f)   ? -(kTwoPi - door.openValue)   : 0.0f;
            signedClosed = (door.closedValue > 0.01f) ? -(kTwoPi - door.closedValue) : 0.0f;
        }
        float range = signedOpen - signedClosed;
        if (std::abs(range) < 1e-6f) return 1.0f;
        return std::clamp((door.currentValue - signedClosed) / range, 0.0f, 1.0f);
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
        ++mFrameCount;
        for (auto &[id, door] : mDoors) {
            if (door.status == kDoorOpening || door.status == kDoorClosing) {
                updateDoor(door, delta);

                // Log every animation frame
                {
                    const ObjectState *os = mObjectStates ? mObjectStates->tryGet(id) : nullptr;
                    std::fprintf(stderr, "  Door %d: val=%.4f vel=%.4f frac=%.3f pos=(%.2f,%.2f,%.2f)",
                                 id, door.currentValue, door.velocity, getOpenFraction(id),
                                 os ? os->position.x : 0, os ? os->position.y : 0, os ? os->position.z : 0);
                    if (os && os->hasMatrix) {
                        const float *m = os->modelMatrix;
                        std::fprintf(stderr, " mtx12-14=(%.2f,%.2f,%.2f)", m[12], m[13], m[14]);
                    }
                    std::fprintf(stderr, "\n");
                }

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

            // Get the door's base transform. Use raw binary radians from the
            // ObjectPlacement data (which the static renderer also uses) to avoid
            // quaternion round-trip that can produce mirrored Euler angles.
            initDoorBaseTransform(door, propSvc, id);

            // Compute hinge pivot offset from model bounding box.
            if (doorType == kDoorRotating) {
                computePivotOffset(door, propSvc, id);
            }

            // Do NOT create ObjectState entries here. Doors render through the
            // static P$Position path until they start animating.

            // Log door init details for debugging
            std::fprintf(stderr, "  Door %d: type=%s axis=%d closed=%.3f open=%.3f speed=%.3f "
                         "clockwise=%d status=%d\n",
                         id, (doorType == kDoorRotating ? "rot" : "trans"),
                         door.axis, door.closedValue, door.openValue,
                         door.speed, door.clockwise ? 1 : 0, door.status);
            std::fprintf(stderr, "    basePos=(%.2f,%.2f,%.2f) angles=(h=%.4f,p=%.4f,b=%.4f) "
                         "scale=(%.2f,%.2f,%.2f)\n",
                         door.basePosition.x, door.basePosition.y, door.basePosition.z,
                         door.baseHeading, door.basePitch, door.baseBank,
                         door.baseScale.x, door.baseScale.y, door.baseScale.z);
            std::fprintf(stderr, "    pivot=(%.2f,%.2f,%.2f) rooms=(%d,%d) blocking=%.0f%%\n",
                         door.pivotOffset.x, door.pivotOffset.y, door.pivotOffset.z,
                         door.room1, door.room2, door.soundBlocking * 100.0f);

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
        // Speed is already in radians/sec (physics velocity unit) — do NOT convert.
        // The Dark Engine stores open/closed angles in degrees for DromEd editing
        // convenience, but base_speed is a physics parameter in native units.
        door.speed = prop.speed;
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
        static constexpr float kAngScale = 2.0f * 3.14159265f / 65536.0f;

        // Prefer raw ObjectPlacement data (binary radians → float radians directly).
        // This matches the static renderer's conversion exactly, avoiding the
        // quaternion round-trip which can produce mirrored Euler angles.
        if (mPlacements) {
            auto it = mPlacements->find(objID);
            if (it != mPlacements->end()) {
                const auto &p = it->second;
                door.basePosition = Vector3(p.x, p.y, p.z);
                door.baseHeading = static_cast<float>(p.heading) * kAngScale;
                door.basePitch   = static_cast<float>(p.pitch)   * kAngScale;
                door.baseBank    = static_cast<float>(p.bank)    * kAngScale;
                door.baseScale   = Vector3(p.sx, p.sy, p.sz);
                return;
            }
        }

        // Fallback: use ObjectService (quaternion round-trip)
        if (mObjSvc) {
            door.basePosition = mObjSvc->position(objID);
            Quaternion q = mObjSvc->orientation(objID);
            Matrix3 rotMat = glm::mat3_cast(q);
            glm::extractEulerAngleZYX(Matrix4(rotMat), door.baseHeading,
                                       door.basePitch, door.baseBank);
        }

        // P$Scale: direct ownership only (kPropertyNoInherit)
        Vector3 scale(1.0f);
        if (getTypedProperty<Vector3>(propSvc, "Scale", objID, scale)) {
            door.baseScale = scale;
        }
    }

    /// Compute the COG (pivot) offset for a rotating door.
    /// Matches the Dark Engine algorithm in doorphys.cpp UpdateDoorPhysics:
    ///   If COG is zero, compute from OBB edge lengths:
    ///     axis 0 or 1 (X/Y rotation): cog.z = edgeLengths.z / 2
    ///     axis 2      (Z rotation):   cog.x = edgeLengths.x / 2
    /// This places the COG at the center of the door slab, so that rotation
    /// around the model origin (hinge) correctly arcs the visual center.
    void computePivotOffset(DoorState &door, PropertyService *propSvc,
                             int32_t objID) {
        // Try P$PhysDims for explicit OBB dimensions
        PropPhysDims dims;
        Vector3 edgeLengths(0.0f);
        if (getTypedProperty<PropPhysDims>(propSvc, "PhysDims", objID, dims)) {
            edgeLengths = Vector3(dims.sizeX, dims.sizeY, dims.sizeZ);
        }

        // Fall back to model bounding box if PhysDims has no size
        if (glm::length(edgeLengths) < 0.01f && mParsedModels) {
            // Look up model name
            char modelName[16] = {};
            if (getTypedProperty<char[16]>(propSvc, "ModelName", objID, modelName)) {
                std::string mname(modelName);
                auto it = mParsedModels->find(mname);
                if (it != mParsedModels->end()) {
                    const auto &mesh = it->second;
                    edgeLengths = Vector3(
                        mesh.bboxMax[0] - mesh.bboxMin[0],
                        mesh.bboxMax[1] - mesh.bboxMin[1],
                        mesh.bboxMax[2] - mesh.bboxMin[2]);
                }
            }
        }

        // Compute COG offset matching the Dark Engine convention
        door.pivotOffset = Vector3(0.0f);
        switch (door.axis) {
        case 0:  // X rotation
        case 1:  // Y rotation
            door.pivotOffset.z = edgeLengths.z / 2.0f;
            break;
        case 2:  // Z rotation (most common for doors)
            door.pivotOffset.x = edgeLengths.x / 2.0f;
            break;
        }

        // Log bbox using model name from ObjectPlacement data
        if (mParsedModels && mPlacements) {
            auto pit = mPlacements->find(objID);
            if (pit != mPlacements->end()) {
                std::string mname(pit->second.modelName,
                                  strnlen(pit->second.modelName, 16));
                auto mit = mParsedModels->find(mname);
                if (mit != mParsedModels->end()) {
                    const auto &mesh = mit->second;
                    std::fprintf(stderr, "  Door %d: model '%s' "
                                 "bbox=(%.2f,%.2f,%.2f)-(%.2f,%.2f,%.2f) "
                                 "center=(%.2f,%.2f,%.2f)\n",
                                 objID, mname.c_str(),
                                 mesh.bboxMin[0], mesh.bboxMin[1], mesh.bboxMin[2],
                                 mesh.bboxMax[0], mesh.bboxMax[1], mesh.bboxMax[2],
                                 (mesh.bboxMin[0]+mesh.bboxMax[0])*0.5f,
                                 (mesh.bboxMin[1]+mesh.bboxMax[1])*0.5f,
                                 (mesh.bboxMin[2]+mesh.bboxMax[2])*0.5f);
                } else {
                    std::fprintf(stderr, "  Door %d: model '%s' NOT FOUND in parsedModels\n",
                                 objID, mname.c_str());
                }
            }
        }

        std::fprintf(stderr, "  Door %d: pivot offset (%.2f, %.2f, %.2f) "
                     "from edge lengths (%.1f, %.1f, %.1f)\n",
                     door.objID, door.pivotOffset.x, door.pivotOffset.y,
                     door.pivotOffset.z, edgeLengths.x, edgeLengths.y,
                     edgeLengths.z);
    }

    // ── Door movement ──

    /// Ensure the door has an ObjectState entry (created on-demand when
    /// animation starts, so static doors render through the normal path).
    void ensureObjectState(DoorState &door) {
        if (door.hasObjectState || !mObjectStates) return;
        door.hasObjectState = true;
        applyDoorTransform(door);

        // Log the ObjectState creation for debugging
        const ObjectState *os = mObjectStates->tryGet(door.objID);
        if (os) {
            std::fprintf(stderr, "  Door %d: ObjectState CREATED at currentValue=%.4f\n"
                         "    pos=(%.2f,%.2f,%.2f) angles=(h=%.4f,p=%.4f,b=%.4f) hasMatrix=%d\n",
                         door.objID, door.currentValue,
                         os->position.x, os->position.y, os->position.z,
                         os->heading, os->pitch, os->bank, os->hasMatrix ? 1 : 0);
            if (os->hasMatrix) {
                const float *m = os->modelMatrix;
                std::fprintf(stderr, "    matrix: [%.3f %.3f %.3f %.3f]\n"
                             "            [%.3f %.3f %.3f %.3f]\n"
                             "            [%.3f %.3f %.3f %.3f]\n"
                             "            [%.3f %.3f %.3f %.3f]\n",
                             m[0],m[1],m[2],m[3], m[4],m[5],m[6],m[7],
                             m[8],m[9],m[10],m[11], m[12],m[13],m[14],m[15]);
            }
        }
    }

    void startOpening(DoorState &door) {
        if (door.status == kDoorOpen) return;

        ensureObjectState(door);

        // Remove blocking when door starts opening
        if (door.status == kDoorClosed) {
            emitEvent(door, kDoorOpening);
        }

        door.status = kDoorOpening;

        // Set velocity direction based on door type.
        // For rotating doors, clockwise=true means negative rotation direction.
        // The Dark Engine convention: clockwise doors rotate from closed toward
        // -openValue (e.g., a 270° clockwise door goes from 0 to -4.712 rad).
        if (door.type == kDoorRotating) {
            float dir = door.clockwise ? -1.0f : 1.0f;
            door.velocity = door.speed * dir;
        } else {
            float dir = (door.openValue > door.closedValue) ? 1.0f : -1.0f;
            door.velocity = door.speed * dir;
        }
    }

    void startClosing(DoorState &door) {
        if (door.status == kDoorClosed) return;

        ensureObjectState(door);

        door.status = kDoorClosing;

        // Reverse direction
        if (door.type == kDoorRotating) {
            float dir = door.clockwise ? 1.0f : -1.0f;
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

        // Check limits. For clockwise rotating doors, currentValue goes negative.
        // A clockwise door with open=270° (4.712 rad) reaches its open position
        // by rotating 90° clockwise (the short way around), not 270°.
        // The signed target is -(2π - openValue) when openValue > π.
        static constexpr float kTwoPi = 2.0f * 3.14159265f;
        float signedOpen = door.openValue;
        float signedClosed = door.closedValue;
        if (door.type == kDoorRotating && door.clockwise) {
            signedOpen  = (door.openValue > 0.01f)   ? -(kTwoPi - door.openValue)   : 0.0f;
            signedClosed = (door.closedValue > 0.01f) ? -(kTwoPi - door.closedValue) : 0.0f;
        }

        bool reachedOpen = false;
        bool reachedClosed = false;

        if (door.status == kDoorOpening) {
            if ((door.velocity > 0 && door.currentValue >= signedOpen) ||
                (door.velocity < 0 && door.currentValue <= signedOpen)) {
                door.currentValue = signedOpen;
                door.velocity = 0.0f;
                door.status = kDoorOpen;
                reachedOpen = true;
            }
        } else if (door.status == kDoorClosing) {
            if ((door.velocity > 0 && door.currentValue >= signedClosed) ||
                (door.velocity < 0 && door.currentValue <= signedClosed)) {
                door.currentValue = signedClosed;
                door.velocity = 0.0f;
                door.status = kDoorClosed;
                reachedClosed = true;
            }
        }

        // Clamp to range (safety)
        float minVal = std::min(signedClosed, signedOpen);
        float maxVal = std::max(signedClosed, signedOpen);
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
            // Build the combined rotation: R_base * R_local_offset (in glm).
            // P$Position is the model center (not the hinge). The COG pivot offset
            // gives the vector from hinge to center in local space. As the door
            // rotates, the center traces an arc around the fixed hinge:
            //   base_cog = R_base * pivotOffset
            //   cur_cog  = R_combined * pivotOffset
            //   worldPos = basePosition + (cur_cog - base_cog)
            Matrix4 baseMat = glm::eulerAngleZYX(door.baseHeading,
                                                   door.basePitch,
                                                   door.baseBank);
            Vector3 axisVec(0.0f);
            switch (door.axis) {
            case 0: axisVec.x = 1.0f; break;
            case 1: axisVec.y = 1.0f; break;
            case 2: axisVec.z = 1.0f; break;
            }
            Matrix4 offsetMat = glm::rotate(Matrix4(1.0f), door.currentValue, axisVec);
            Matrix4 combined = baseMat * offsetMat;

            // Position arc: center traces arc around hinge
            Matrix3 baseMat3 = Matrix3(baseMat);
            Matrix3 combinedMat3 = Matrix3(combined);
            Vector3 baseCog = baseMat3 * door.pivotOffset;
            Vector3 curCog = combinedMat3 * door.pivotOffset;
            Vector3 worldPos = door.basePosition + (curCog - baseCog);

            // Build bx-format model matrix directly from the combined rotation.
            // Transpose the glm column-major 3x3 into bx row-major — equivalent
            // to bx::mtxRotateXYZ(-b,-p,-h) without lossy Euler extraction.
            Matrix3 rot3 = Matrix3(combined);
            float *m = os.modelMatrix;
            m[ 0] = rot3[0][0]; m[ 1] = rot3[1][0]; m[ 2] = rot3[2][0]; m[ 3] = 0.0f;
            m[ 4] = rot3[0][1]; m[ 5] = rot3[1][1]; m[ 6] = rot3[2][1]; m[ 7] = 0.0f;
            m[ 8] = rot3[0][2]; m[ 9] = rot3[1][2]; m[10] = rot3[2][2]; m[11] = 0.0f;
            // Scale (row-scaling)
            m[ 0] *= door.baseScale.x; m[ 1] *= door.baseScale.x; m[ 2] *= door.baseScale.x;
            m[ 4] *= door.baseScale.y; m[ 5] *= door.baseScale.y; m[ 6] *= door.baseScale.y;
            m[ 8] *= door.baseScale.z; m[ 9] *= door.baseScale.z; m[10] *= door.baseScale.z;
            // Translation — center position on the arc
            m[12] = worldPos.x; m[13] = worldPos.y; m[14] = worldPos.z; m[15] = 1.0f;

            os.hasMatrix = true;
            os.position = worldPos;
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
    const std::unordered_map<std::string, ParsedBinMesh> *mParsedModels = nullptr;
    const std::unordered_map<int32_t, ObjPlacementInfo> *mPlacements = nullptr;
    DoorEventCallback mEventCallback;
    AudioBlockingCallback mAudioBlockingCallback;
    uint32_t mFrameCount = 0;
};

} // namespace Darkness
