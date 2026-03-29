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
#include <cstring>
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

    // Base transform from P$Position. We store the rotation as a matrix
    // (not Euler angles) to avoid the lossy extractEulerAngleZYX round-trip
    // which can produce mirrored representations. The original engine works
    // with rotation matrices directly (mx_ang2mat) — we do the same.
    Vector3 basePosition = {0.0f, 0.0f, 0.0f};
    Matrix4 baseRotation = Matrix4(1.0f);  // glm column-major rotation matrix
    Vector3 baseScale = {1.0f, 1.0f, 1.0f};
    bool hasObjectState = false;  // true once ObjectState entry is created
    bool isBrushDoor = false;     // true if terrain brush (no .bin model to render)

    // Pivot offset: vector from hinge to model center in the door's local frame.
    // For rotating doors, the center traces an arc around the hinge. The pivot
    // is computed from the .bin model bounding box — half the door width along
    // the axis perpendicular to the rotation axis.
    Vector3 pivotOffset = {0.0f, 0.0f, 0.0f};

    // Renderer index — index into mission.objData.objects[] for quick lookup.
    // -1 if not found (archetype-only door, shouldn't happen for concrete objects).
    int renderIndex = -1;

    // Debug: frame counter for per-activation logging (reset on each activate)
    int logFrames = 0;
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
        door.logFrames = 0;  // reset per-activation debug counter
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

                // Log first 3 frames of each activation + final frame
                if (door.logFrames < 3 ||
                    door.status == kDoorOpen || door.status == kDoorClosed) {
                    const ObjectState *os = mObjectStates ? mObjectStates->tryGet(id) : nullptr;
                    if (os && os->hasMatrix) {
                        const float *m = os->modelMatrix;
                        // Compute where hinge and far-edge ended up in world space
                        // by transforming local-space points through the bx row-major matrix.
                        // bx row-major: result = v * M (row-vector convention)
                        auto xformPt = [&](float lx, float ly, float lz) -> Vector3 {
                            return Vector3(
                                lx*m[0] + ly*m[4] + lz*m[8]  + m[12],
                                lx*m[1] + ly*m[5] + lz*m[9]  + m[13],
                                lx*m[2] + ly*m[6] + lz*m[10] + m[14]);
                        };
                        // Log all 8 bbox corners transformed to world space.
                        // Get model bbox from parsedModels via placement modelName.
                        float bmin[3] = {-2, -0.1f, -4};  // fallback
                        float bmax[3] = { 2,  0.1f,  4};
                        if (mPlacements) {
                            auto pit = mPlacements->find(id);
                            if (pit != mPlacements->end()) {
                                std::string mname(pit->second.modelName,
                                    strnlen(pit->second.modelName, 16));
                                if (mParsedModels) {
                                    auto mit = mParsedModels->find(mname);
                                    if (mit != mParsedModels->end()) {
                                        for (int i = 0; i < 3; ++i) {
                                            bmin[i] = mit->second.bboxMin[i];
                                            bmax[i] = mit->second.bboxMax[i];
                                        }
                                    }
                                }
                            }
                        }
                        std::fprintf(stderr, "  Door %d frame %d: val=%.3f bbox=(%.1f,%.1f,%.1f)-(%.1f,%.1f,%.1f)\n",
                                     id, door.logFrames, door.currentValue,
                                     bmin[0], bmin[1], bmin[2], bmax[0], bmax[1], bmax[2]);
                        // Transform and log all 8 corners
                        for (int ci = 0; ci < 8; ++ci) {
                            float cx = (ci & 1) ? bmax[0] : bmin[0];
                            float cy = (ci & 2) ? bmax[1] : bmin[1];
                            float cz = (ci & 4) ? bmax[2] : bmin[2];
                            Vector3 w = xformPt(cx, cy, cz);
                            std::fprintf(stderr, "    v%d local=(%.1f,%.1f,%.1f) -> world=(%.2f,%.2f,%.2f)\n",
                                         ci, cx, cy, cz, w.x, w.y, w.z);
                        }
                    }
                    ++door.logFrames;
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

            // Check if this door is in the render array (= has a .bin model).
            // Dark Engine doors are typically TERRAIN BRUSHES — part of the
            // world geometry, not .bin model objects. The .bin model (if any)
            // is inherited from the archetype for physics collision shape only.
            // We must NOT render a .bin model for these — the visual is the brush.
            bool isRenderedObject = mPlacements &&
                mPlacements->find(id) != mPlacements->end();
            door.isBrushDoor = !isRenderedObject;

            // Detailed init logging for door geometry debugging
            {
                Quaternion q = mObjSvc ? mObjSvc->orientation(id) : Quaternion(1,0,0,0);
                std::fprintf(stderr, "  Door %d: type=%s axis=%d closed=%.3f open=%.3f speed=%.3f "
                             "clockwise=%d isBrush=%d rooms=(%d,%d)\n",
                             id, (doorType == kDoorRotating ? "rot" : "trans"),
                             door.axis, door.closedValue, door.openValue,
                             door.speed, door.clockwise ? 1 : 0, door.isBrushDoor ? 1 : 0,
                             door.room1, door.room2);
                std::fprintf(stderr, "    basePos=(%.2f,%.2f,%.2f) pivot=(%.2f,%.2f,%.2f)\n",
                             door.basePosition.x, door.basePosition.y, door.basePosition.z,
                             door.pivotOffset.x, door.pivotOffset.y, door.pivotOffset.z);
                std::fprintf(stderr, "    quat=(w=%.4f,x=%.4f,y=%.4f,z=%.4f)\n",
                             q.w, q.x, q.y, q.z);
                std::fprintf(stderr, "    baseMat row0=(%.4f,%.4f,%.4f) row1=(%.4f,%.4f,%.4f) row2=(%.4f,%.4f,%.4f)\n",
                             door.baseRotation[0][0], door.baseRotation[1][0], door.baseRotation[2][0],
                             door.baseRotation[0][1], door.baseRotation[1][1], door.baseRotation[2][1],
                             door.baseRotation[0][2], door.baseRotation[1][2], door.baseRotation[2][2]);
                // Log hinge and far-edge in world space (local→world via baseMat+basePos)
                Vector3 hingeLocal = door.pivotOffset;  // hinge in local space
                Vector3 farLocal = -door.pivotOffset;    // far edge in local space
                Matrix3 rot3 = Matrix3(door.baseRotation);
                Vector3 hingeWorld = door.basePosition + rot3 * hingeLocal;
                Vector3 farWorld = door.basePosition + rot3 * farLocal;
                std::fprintf(stderr, "    hingeWorld=(%.2f,%.2f,%.2f) farWorld=(%.2f,%.2f,%.2f)\n",
                             hingeWorld.x, hingeWorld.y, hingeWorld.z,
                             farWorld.x, farWorld.y, farWorld.z);
                // Log placement angles if available
                if (mPlacements) {
                    auto pit = mPlacements->find(id);
                    if (pit != mPlacements->end()) {
                        const auto &pl = pit->second;
                        float angScale = 2.0f * 3.14159265f / 65536.0f;
                        std::fprintf(stderr, "    placement h=%d(%.1f°) p=%d(%.1f°) b=%d(%.1f°)\n",
                                     pl.heading, pl.heading * angScale * 180.0f / 3.14159265f,
                                     pl.pitch, pl.pitch * angScale * 180.0f / 3.14159265f,
                                     pl.bank, pl.bank * angScale * 180.0f / 3.14159265f);
                    }
                }
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

        // Get position and rotation from placement data (allPlacements map,
        // which includes ALL concrete objects regardless of RenderType).
        if (mPlacements) {
            auto it = mPlacements->find(objID);
            if (it != mPlacements->end()) {
                const auto &pl = it->second;
                door.basePosition = Vector3(pl.x, pl.y, pl.z);
                door.baseScale = Vector3(pl.sx, pl.sy, pl.sz);

                // Build rotation matching Dark Engine convention:
                // Rz(heading) * Ry(pitch) * Rx(bank).
                // No angle negation — we use direct GLM memcpy (not 3x3 transpose)
                // so the GPU sees the GLM matrix as-is.
                float h   = static_cast<float>(pl.heading) * kAngScale;
                float pit = static_cast<float>(pl.pitch)   * kAngScale;
                float b   = static_cast<float>(pl.bank)    * kAngScale;
                door.baseRotation = Matrix4(glm::mat3(
                    glm::eulerAngleZYX(h, pit, b)));
            } else {
                std::fprintf(stderr, "  WARNING: Door %d: not found in allPlacements — "
                             "baseRotation will be identity. Door will snap on first activation.\n", objID);
                if (mObjSvc) {
                    door.basePosition = mObjSvc->position(objID);
                }
            }
        } else {
            std::fprintf(stderr, "  WARNING: Door %d: no placement data provided — "
                         "baseRotation will be identity.\n", objID);
            if (mObjSvc) {
                door.basePosition = mObjSvc->position(objID);
            }
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
            door.pivotOffset.z = -edgeLengths.z / 2.0f;
            break;
        case 2:  // Z rotation (most common for doors)
            door.pivotOffset.x = -edgeLengths.x / 2.0f;
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
    /// All doors (including brush doors) get ObjectState — the door's .bin
    /// model (inherited from archetype) is the visual that animates. The WR
    /// cell geometry is just the door frame, not the door slab.
    void ensureObjectState(DoorState &door) {
        if (door.hasObjectState || !mObjectStates) return;
        door.hasObjectState = true;
        applyDoorTransform(door);
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
            // Rotate around the hinge edge, not the model center.
            //
            // The .bin model origin is at the center of the door slab.
            // pivotOffset = half the door width, pointing from center to
            // the hinge edge. To rotate around that edge:
            //
            //   M = T(basePos) * R_base * T(+pivot) * R_offset * T(-pivot) * S
            const Matrix4 &baseMat = door.baseRotation;
            Vector3 axisVec(0.0f);
            switch (door.axis) {
            case 0: axisVec.x = 1.0f; break;
            case 1: axisVec.y = 1.0f; break;
            case 2: axisVec.z = 1.0f; break;
            }
            Matrix4 fullGlm;
            if (std::abs(door.currentValue) < 1e-7f) {
                // At rest: T(basePos) * R_base * S — matches static renderer.
                Matrix4 scaleMat = glm::scale(Matrix4(1.0f), door.baseScale);
                Matrix4 worldTranslate = glm::translate(Matrix4(1.0f), door.basePosition);
                fullGlm = worldTranslate * baseMat * scaleMat;
            } else {
                // Animating: rotate around hinge edge (at +pivotOffset from center).
                Matrix4 offsetRot = glm::rotate(Matrix4(1.0f), door.currentValue, axisVec);
                Matrix4 toPivot = glm::translate(Matrix4(1.0f), -door.pivotOffset);
                Matrix4 fromPivot = glm::translate(Matrix4(1.0f), door.pivotOffset);
                Matrix4 scaleMat = glm::scale(Matrix4(1.0f), door.baseScale);
                Matrix4 worldTranslate = glm::translate(Matrix4(1.0f), door.basePosition);
                fullGlm = worldTranslate * baseMat * fromPivot * offsetRot * toPivot * scaleMat;
            }

            // Copy GLM column-major matrix directly to the model matrix.
            // bgfx passes the float[16] to the GPU without transposing, and
            // the GPU reads it as column-major — which is GLM's native format.
            // No 3x3 transpose needed (and doing one breaks pivot transforms
            // by inverting the rotation while leaving translation unchanged).
            std::memcpy(os.modelMatrix, glm::value_ptr(fullGlm), 16 * sizeof(float));

            os.hasMatrix = true;
            os.position = door.basePosition;  // approximate for queries
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
            Matrix3 rotMat = Matrix3(door.baseRotation);
            Vector3 worldOffset = rotMat * localOffset;

            // For translating doors, build the matrix directly too
            Matrix4 transMat = glm::translate(Matrix4(1.0f), worldOffset);
            Matrix4 worldTranslate = glm::translate(Matrix4(1.0f), door.basePosition);
            Matrix4 scaleMat = glm::scale(Matrix4(1.0f), door.baseScale);
            Matrix4 fullGlm = worldTranslate * transMat * door.baseRotation * scaleMat;

            float *m = os.modelMatrix;
            for (int col = 0; col < 3; ++col)
                for (int row = 0; row < 3; ++row)
                    m[row * 4 + col] = fullGlm[col][row];
            m[ 3] = 0.0f; m[ 7] = 0.0f; m[11] = 0.0f;
            m[12] = fullGlm[3][0]; m[13] = fullGlm[3][1];
            m[14] = fullGlm[3][2]; m[15] = 1.0f;

            os.hasMatrix = true;
            os.position = door.basePosition + worldOffset;
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
