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
#include "audio/AudioLog.h"
#include "object/ObjectService.h"
#include "property/DarkPropertyDefs.h"
#include "property/PropertyService.h"
#include "property/TypedProperty.h"
#include "BinMeshParser.h"
#include "sim/DoorAudioGeometry.h"
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

    // OBB dimensions in world units. Sourced from P$PhysDims, falling back to
    // model bounding box multiplied by baseScale. Already in final (post-scale)
    // world size, so audio-geometry consumers should NOT multiply by baseScale
    // again. Zero means the door has no usable bounds (logged at init time).
    Vector3 edgeLengths = {0.0f, 0.0f, 0.0f};

    // Dominant surface material NAME of the door's .bin model (the texture
    // name of its largest submesh, e.g. "PORTC.GIF", "DOOR.GIF"). Resolved
    // once at init from mParsedModels. The audio side maps this through the
    // same acoustic-material keyword lookup the world geometry uses, so a
    // metal gate transmits/reflects differently from a wooden door instead
    // of every door being hardcoded to wood. Empty = unknown (audio falls
    // back to its default material).
    std::string acousticMaterial;

    // Renderer index — index into mission.objData.objects[] for quick lookup.
    // -1 if not found (archetype-only door, shouldn't happen for concrete objects).
    int renderIndex = -1;

    // Debug: frame counter for per-activation logging (reset on each activate)
    int blockingLogCount = 0;  // one-shot counter for blocking factor logs
};

// ── Callback for door events (sound blocking, script messages, etc.) ──
using DoorEventCallback = std::function<void(int32_t objID, DoorStatus newStatus,
                                              DoorStatus oldStatus,
                                              const DoorState &door)>;

// DoorAudioGeometry lives in sim/DoorAudioGeometry.h so AudioService.cpp can
// consume it without pulling in this header's transitive BinMeshParser.h
// dependency (which lives in src/main and isn't visible from DarknessServices
// translation units).

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
    // ObjPlacementInfo is defined in SimCommon.h (shared with TweqSystem)

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
        DoorStatus prevStatus = door.status;
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
        AUDIO_LOG("[DOOR_ACT] obj=%d action=%d status:%d->%d rooms(%d,%d) sndBlock=%.2f\n",
                  objID, (int)action, (int)prevStatus, (int)door.status,
                  door.room1, door.room2, door.soundBlocking);
        return true;
    }

    /// Get the current status of a door.
    DoorStatus getStatus(int32_t objID) const {
        auto it = mDoors.find(objID);
        if (it == mDoors.end()) {
            static int warnCount = 0;
            if (warnCount++ < 5)
                std::fprintf(stderr, "[DEFAULT] DoorSystem::getStatus: obj %d not a door, returning kDoorClosed\n", objID);
            return kDoorClosed;
        }
        return it->second.status;
    }

    /// Get the open fraction (0.0 = closed, 1.0 = open) for portal blocking.
    float getOpenFraction(int32_t objID) const {
        static constexpr float kTwoPi = 2.0f * 3.14159265f;
        auto it = mDoors.find(objID);
        if (it == mDoors.end()) {
            static int warnCount = 0;
            if (warnCount++ < 5)
                std::fprintf(stderr, "[DEFAULT] DoorSystem::getOpenFraction: obj %d not a door, returning 1.0 (fully open)\n", objID);
            return 1.0f;
        }
        const DoorState &door = it->second;
        float signedOpen = door.openValue;
        float signedClosed = door.closedValue;
        if (door.type == kDoorRotating && door.clockwise) {
            signedOpen  = (door.openValue > 0.01f)   ? -(kTwoPi - door.openValue)   : 0.0f;
            signedClosed = (door.closedValue > 0.01f) ? -(kTwoPi - door.closedValue) : 0.0f;
        }
        float range = signedOpen - signedClosed;
        if (std::abs(range) < 1e-6f) {
            static int warnCount = 0;
            if (warnCount++ < 5)
                std::fprintf(stderr, "[FALLBACK] DoorSystem::getOpenFraction: obj %d range~0 (open=%.3f closed=%.3f), returning 1.0\n",
                             objID, signedOpen, signedClosed);
            return 1.0f;
        }
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
        if (doorIt == mDoors.end()) {
            std::fprintf(stderr, "[FALLBACK] DoorSystem::getSoundBlockingForRooms: room pair %d<->%d mapped to door %d but door not in mDoors!\n",
                         room1, room2, it->second);
            return 0.0f;
        }
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

    /// Set callback for runtime collision body updates during door animation.
    /// Called per-frame with (objID, worldMatrix) while a door is moving.
    /// The callback should update the door's collision OBB to match its visual
    /// position, allowing the player to walk through open doors.
    using CollisionUpdateCallback = std::function<void(int32_t objID, const Matrix4 &worldMatrix)>;
    void setCollisionUpdateCallback(CollisionUpdateCallback cb) { mCollisionUpdateCb = std::move(cb); }

    /// Set callback for runtime audio-geometry transform updates. Called
    /// per-frame with (objID, audioWorldMatrix) while a door is moving — the
    /// matrix is the door's pose WITHOUT scale baked in (since door audio
    /// meshes already encode their final dimensions in their vertices). The
    /// callback should push the new transform into Steam Audio's acoustic
    /// scene via iplInstancedMeshUpdateTransform so geometry-aware path
    /// validation sees doors at their current position.
    using AudioMeshUpdateCallback = std::function<void(
        int32_t objID, const Matrix4 &worldMatrix, float openFraction)>;
    void setAudioMeshUpdateCallback(AudioMeshUpdateCallback cb) { mAudioMeshUpdateCb = std::move(cb); }

    /// Snapshot every door's audio geometry — local-space OBB mesh + initial
    /// world transform — for one-shot registration with the audio scene
    /// (AudioService::registerDoorGeometry). Doors with zero edgeLengths are
    /// skipped: they have no usable bounds and would degenerate the BVH.
    /// Call AFTER init() and after the renderer has populated parsedModels.
    std::vector<DoorAudioGeometry> getAudioGeometryInventory() const {
        std::vector<DoorAudioGeometry> out;
        out.reserve(mDoors.size());
        for (const auto &[id, door] : mDoors) {
            if (glm::length(door.edgeLengths) < 0.01f) {
                std::fprintf(stderr,
                    "[FALLBACK] DoorSystem::getAudioGeometryInventory: door %d "
                    "has zero edgeLengths — skipping (no audio occlusion mesh)\n",
                    id);
                continue;
            }
            DoorAudioGeometry geom;
            geom.objID = id;
            buildBoxMesh(door.edgeLengths, geom.localVertices, geom.indices);
            geom.worldTransform = computeAudioWorldMatrix(door);
            geom.materialName = door.acousticMaterial;
            out.push_back(std::move(geom));
        }
        return out;
    }

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

                // Update audio blocking continuously during animation.
                // Blocking scales with closed fraction: fully closed = full blocking,
                // fully open = zero blocking. Applies symmetrically for both opening
                // and closing so the LPF transitions smoothly in both directions.
                // Door's own sounds use skipPortalRouting and are unaffected.
                if (mAudioBlockingCallback &&
                    door.room1 >= 0 && door.room2 >= 0 &&
                    door.room1 != door.room2 && door.soundBlocking > 0.0f) {
                    float openFrac = getOpenFraction(id);
                    float blocking = door.soundBlocking * (1.0f - openFrac);
                    mAudioBlockingCallback(door.room1, door.room2, blocking);
                    // One-shot diagnostic: log first blocking change per door activation
                    if (door.blockingLogCount < 2) {
                        std::fprintf(stderr, "  Door %d: blocking rooms(%d,%d) "
                                     "factor=%.3f (sndBlock=%.2f, %s)\n",
                                     id, door.room1, door.room2,
                                     blocking, door.soundBlocking,
                                     door.status == kDoorOpening ? "opening" : "closing");
                        ++door.blockingLogCount;
                    }
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

            // Populate door.edgeLengths (always needed for the audio
            // occlusion OBB — registerDoorGeometry skips doors with
            // zero edgeLengths) and, for rotating doors only, compute
            // the hinge pivot offset. Previously this call was gated
            // on `doorType == kDoorRotating`, which silently denied
            // every TransDoor (portcullises, sliding gates, secret
            // slabs) an audio OBB — see MISS6 doors 34, 35, 705, 895.
            populateOBBAndPivot(door, propSvc, id);

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
                             "clockwise=%d isBrush=%d rooms=(%d,%d) sndBlock=%.2f\n",
                             id, (doorType == kDoorRotating ? "rot" : "trans"),
                             door.axis, door.closedValue, door.openValue,
                             door.speed, door.clockwise ? 1 : 0, door.isBrushDoor ? 1 : 0,
                             door.room1, door.room2, door.soundBlocking);
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
        // blockSound is a 0-100 percentage matching the Dark Engine
        // convention (default: 60 = 0.6 transmission loss). An earlier
        // version of this code halved the value (divided by 200) "to
        // compensate for Steam Audio's geometry-based occlusion through
        // door frame walls" — but doors are not in the Steam Audio
        // acoustic scene (only WR cell geometry is), so there was no
        // compensating attenuation, and doors blocked ~30% instead of
        // ~60%. Long portal-graph paths with several closed doors then
        // transmitted too much sound (e.g., church ambient audible from
        // spawn rooms away).
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
        // See RotDoor initializer above for the divisor rationale.
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
                std::fprintf(stderr, "[FALLBACK] DoorSystem: door %d not found in allPlacements — "
                             "baseRotation will be identity. Door will snap on first activation.\n", objID);
                if (mObjSvc) {
                    door.basePosition = mObjSvc->position(objID);
                }
            }
        } else {
            std::fprintf(stderr, "[FALLBACK] DoorSystem: door %d no placement data provided — "
                         "baseRotation will be identity.\n", objID);
            if (mObjSvc) {
                door.basePosition = mObjSvc->position(objID);
            }
        }

        // P$Scale: direct ownership only (kPropertyNoInherit)
        Vector3 scale(1.0f);
        if (getTypedProperty<Vector3>(propSvc, "ModelScale", objID, scale)) {
            door.baseScale = scale;
        }
    }

    /// Populate `door.edgeLengths` from P$PhysDims (or fall back to the
    /// model bbox) and, for rotating doors only, derive the hinge
    /// pivot offset from those edges. Called for EVERY door regardless
    /// of type — translating doors need the edges for the audio
    /// occlusion OBB (registerDoorGeometry skips zero-edge doors), even
    /// though they ignore the pivot.
    ///
    /// Pivot computation matches the Dark Engine algorithm in
    /// doorphys.cpp UpdateDoorPhysics: if COG is zero, derive from OBB
    /// edge lengths so the rotating slab arcs around the hinge edge
    /// rather than the model center:
    ///   axis 0 or 1 (X/Y rotation): cog.z = edgeLengths.z / 2
    ///   axis 2      (Z rotation):   cog.x = edgeLengths.x / 2
    void populateOBBAndPivot(DoorState &door, PropertyService *propSvc,
                              int32_t objID) {
        // Try P$PhysDims for explicit OBB dimensions
        PropPhysDims dims;
        Vector3 edgeLengths(0.0f);
        if (getTypedProperty<PropPhysDims>(propSvc, "PhysDims", objID, dims)) {
            edgeLengths = Vector3(dims.sizeX, dims.sizeY, dims.sizeZ);
        }

        // Fall back to model bounding box if PhysDims has no size.
        // Apply door.baseScale since PhysDims.size already includes scale
        // but raw model bbox dimensions do not.
        if (glm::length(edgeLengths) < 0.01f) {
            std::fprintf(stderr, "[FALLBACK] DoorSystem::populateOBBAndPivot: door %d has no P$PhysDims size, trying model bbox\n", objID);
        }
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
                        mesh.bboxMax[2] - mesh.bboxMin[2]) * door.baseScale;
                }
            }
        }

        if (glm::length(edgeLengths) < 0.01f) {
            std::fprintf(stderr, "[DEFAULT] DoorSystem::populateOBBAndPivot: door %d has no PhysDims AND no model bbox — edgeLengths will be zero (audio OBB skipped, pivot at origin)!\n", objID);
        }

        // Cache the OBB dimensions for downstream consumers (e.g. audio
        // geometry registration). Already in final post-scale world size.
        door.edgeLengths = edgeLengths;

        // Resolve the door's dominant surface material NAME from its .bin
        // model (the texture name of the largest-area submesh). Handed to
        // the audio side so a door's acoustic transmission/reflection tracks
        // its material (metal gate vs oak) instead of every door being
        // hardcoded to wood — same texture→material keyword lookup the world
        // geometry uses. Empty when there is no model or no materials.
        if (mParsedModels) {
            char modelName[16] = {};
            if (getTypedProperty<char[16]>(propSvc, "ModelName", objID,
                                           modelName)) {
                auto it = mParsedModels->find(std::string(modelName));
                if (it != mParsedModels->end()) {
                    const ParsedBinMesh &mesh = it->second;
                    // Largest-AREA submesh's material = the door's main
                    // acoustic surface. NOT triangle count: a door slab is a
                    // few big quads while the handle is many tiny triangles,
                    // so a count-based pick wrongly returns the handle's
                    // material for most doors (measured: DHANDLE on 101/200
                    // MISS7 doors). Area sums the actual triangle areas.
                    float bestArea = 0.0f;
                    int bestMat = -1;
                    for (const auto &sm : mesh.subMeshes) {
                        if (sm.matIndex < 0) continue;
                        float area = 0.0f;
                        const uint32_t end = sm.firstIndex + sm.indexCount;
                        for (uint32_t k = sm.firstIndex; k + 2 < end; k += 3) {
                            if (k + 2 >= mesh.indices.size()) break;
                            const BinVert &a = mesh.vertices[mesh.indices[k]];
                            const BinVert &b = mesh.vertices[mesh.indices[k+1]];
                            const BinVert &c = mesh.vertices[mesh.indices[k+2]];
                            const float e1x=b.x-a.x, e1y=b.y-a.y, e1z=b.z-a.z;
                            const float e2x=c.x-a.x, e2y=c.y-a.y, e2z=c.z-a.z;
                            const float cx=e1y*e2z-e1z*e2y;
                            const float cy=e1z*e2x-e1x*e2z;
                            const float cz=e1x*e2y-e1y*e2x;
                            area += 0.5f*std::sqrt(cx*cx+cy*cy+cz*cz);
                        }
                        if (area > bestArea) { bestArea = area; bestMat = sm.matIndex; }
                    }
                    if (bestMat < 0 && !mesh.materials.empty())
                        bestMat = 0;   // no submesh info: first material
                    if (bestMat >= 0
                        && bestMat < static_cast<int>(mesh.materials.size())) {
                        const char *mn = mesh.materials[bestMat].name;
                        door.acousticMaterial.assign(
                            mn, strnlen(mn, sizeof(mesh.materials[bestMat].name)));
                    }
                }
            }
        }

        // Pivot offset is only meaningful for rotating doors — translating
        // doors slide along an axis from the model origin and never
        // consult pivotOffset (see applyDoorTransform's translating
        // branch, which uses door.currentValue directly).
        door.pivotOffset = Vector3(0.0f);
        if (door.type == kDoorRotating) {
            switch (door.axis) {
            case 0:  // X rotation
            case 1:  // Y rotation
                door.pivotOffset.z = -edgeLengths.z / 2.0f;
                break;
            case 2:  // Z rotation (most common for doors)
                door.pivotOffset.x = -edgeLengths.x / 2.0f;
                break;
            }
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
            emitEvent(door, kDoorOpening, kDoorClosed);
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
                emitEvent(door, kDoorOpen, kDoorOpening);
            } else if (door.status == kDoorClosing) {
                door.currentValue = door.closedValue;
                door.velocity = 0.0f;
                door.status = kDoorClosed;
                applyDoorTransform(door);
                emitEvent(door, kDoorClosed, kDoorClosing);
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
        if (reachedOpen) emitEvent(door, kDoorOpen, kDoorOpening);
        if (reachedClosed) emitEvent(door, kDoorClosed, kDoorClosing);
    }

    // Compute the door's world transform WITHOUT baseScale applied. Used by
    // audio geometry (door OBB mesh already encodes its world dimensions in
    // vertex coords, so a second scale multiply would double-scale).
    Matrix4 computeAudioWorldMatrix(const DoorState &door) const {
        Matrix4 worldTranslate = glm::translate(Matrix4(1.0f), door.basePosition);

        if (door.type == kDoorRotating) {
            const Matrix4 &baseMat = door.baseRotation;
            Vector3 axisVec(0.0f);
            switch (door.axis) {
            case 0: axisVec.x = 1.0f; break;
            case 1: axisVec.y = 1.0f; break;
            case 2: axisVec.z = 1.0f; break;
            }
            if (std::abs(door.currentValue) < 1e-7f) {
                return worldTranslate * baseMat;
            }
            Matrix4 offsetRot = glm::rotate(Matrix4(1.0f), door.currentValue, axisVec);
            Matrix4 toPivot   = glm::translate(Matrix4(1.0f), -door.pivotOffset);
            Matrix4 fromPivot = glm::translate(Matrix4(1.0f),  door.pivotOffset);
            return worldTranslate * baseMat * fromPivot * offsetRot * toPivot;
        }

        // Translating door
        float offset = door.currentValue;
        Vector3 localOffset(0.0f);
        switch (door.axis) {
        case 0: localOffset.x = offset; break;
        case 1: localOffset.y = offset; break;
        case 2: localOffset.z = offset; break;
        }
        Matrix3 rotMat = Matrix3(door.baseRotation);
        Vector3 worldOffset = rotMat * localOffset;
        Matrix4 transMat = glm::translate(Matrix4(1.0f), worldOffset);
        return worldTranslate * transMat * door.baseRotation;
    }

    // Build a closed-box triangle mesh centred at the model origin with the
    // given edge lengths. 8 vertices, 12 triangles, outward-facing winding.
    // Used for door audio OBB occluders.
    static void buildBoxMesh(const Vector3 &edges,
                             std::vector<float> &outVerts,
                             std::vector<int32_t> &outIndices) {
        const float hx = edges.x * 0.5f;
        const float hy = edges.y * 0.5f;
        const float hz = edges.z * 0.5f;
        const float v[8][3] = {
            {-hx, -hy, -hz}, { hx, -hy, -hz}, { hx,  hy, -hz}, {-hx,  hy, -hz},
            {-hx, -hy,  hz}, { hx, -hy,  hz}, { hx,  hy,  hz}, {-hx,  hy,  hz},
        };
        outVerts.clear();
        outVerts.reserve(24);
        for (int i = 0; i < 8; ++i) {
            outVerts.push_back(v[i][0]);
            outVerts.push_back(v[i][1]);
            outVerts.push_back(v[i][2]);
        }
        // 12 triangles, outward-facing (CCW from outside). For Steam Audio's
        // ray-cast, winding only matters when materials use one-sided masks;
        // CCW is the safe default.
        static const int32_t tri[12][3] = {
            // -Z face
            {0, 2, 1}, {0, 3, 2},
            // +Z face
            {4, 5, 6}, {4, 6, 7},
            // -Y face
            {0, 1, 5}, {0, 5, 4},
            // +Y face
            {3, 6, 2}, {3, 7, 6},
            // -X face
            {0, 4, 7}, {0, 7, 3},
            // +X face
            {1, 2, 6}, {1, 6, 5},
        };
        outIndices.clear();
        outIndices.reserve(36);
        for (int i = 0; i < 12; ++i) {
            outIndices.push_back(tri[i][0]);
            outIndices.push_back(tri[i][1]);
            outIndices.push_back(tri[i][2]);
        }
    }

    void applyDoorTransform(DoorState &door) {
        if (!mObjectStates) return;

        ObjectState &os = mObjectStates->get(door.objID);
        Matrix4 fullGlm(1.0f);
        // Audio mesh uses the unscaled transform — its vertices already encode
        // world dimensions (see DoorAudioGeometry / buildBoxMesh).
        Matrix4 audioGlm = computeAudioWorldMatrix(door);

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
            fullGlm = worldTranslate * transMat * door.baseRotation * scaleMat;

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

        // Update collision body to match the new visual transform.
        // fullGlm includes position, rotation, pivot, and scale — everything
        // needed to recompute the OBB center and orientation.
        // Brush doors and doors without P$PhysType have no collision body;
        // updateBodyTransform silently returns false for those.
        if (mCollisionUpdateCb) {
            mCollisionUpdateCb(door.objID, fullGlm);
        }

        // Push the unscaled transform into the audio acoustic scene so Steam
        // Audio's geometry-aware path validation sees the door at its current
        // pose. Skipped silently when no door audio mesh is registered for
        // this object (zero-edgeLengths doors are filtered at registration).
        if (mAudioMeshUpdateCb) {
            // Open fraction rides along so the audio side can phase-align
            // its pathing transition to the physical door state.
            mAudioMeshUpdateCb(door.objID, audioGlm,
                               getOpenFraction(door.objID));
        }
    }

    void emitEvent(const DoorState &door, DoorStatus newStatus, DoorStatus oldStatus) {
        if (mEventCallback)
            mEventCallback(door.objID, newStatus, oldStatus, door);
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
    CollisionUpdateCallback mCollisionUpdateCb;
    AudioMeshUpdateCallback mAudioMeshUpdateCb;
    uint32_t mFrameCount = 0;
};

} // namespace Darkness
