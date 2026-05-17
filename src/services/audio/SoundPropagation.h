/******************************************************************************
 *
 *    This file is part of the darkness project
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

#ifndef __SOUND_PROPAGATION_H
#define __SOUND_PROPAGATION_H

/// @file SoundPropagation.h
/// Portal-graph sound propagation + runtime door/loud-room cost tables.
///
/// Extracted from AudioService — owns the per-portal door blocking factor
/// map and the per-room LoudRoom transmission map, and forwards
/// propagateSound() calls to RoomService::propagateSoundPath with those
/// runtime tables wired in as callbacks. AudioService keeps thin facades
/// (propagateSound / setBlockingFactor / getBlockingFactor) that route
/// here so existing callers in DarknessRender and AI hearing need no
/// changes.
///
/// Threading: all calls are expected to come from the main thread (same
/// constraint as AudioService's per-frame propagation). The internal maps
/// are not locked.

#include "DarknessMath.h"
#include "room/RoomService.h"  // SoundPropInfo / SoundPropParams

#include <cstdint>
#include <unordered_map>

namespace Darkness {

class Room;
class RoomService;

/// Default propagation cutoff. Kept identical to the long-standing
/// AudioService::SOUND_MAX_DIST value so the public propagateSound
/// signature on AudioService is unchanged.
constexpr float kDefaultSoundMaxDist = 200.0f;

/// Owns the runtime cost tables (door blocking + LoudRoom transmission)
/// and forwards propagateSound() into RoomService's portal-graph BFS.
class SoundPropagation {
public:
    /// Construct with a back-reference to RoomService. The reference must
    /// remain valid for the lifetime of this object — RoomService is
    /// looked up in AudioService::bootstrapFinished() and released in
    /// shutdown(), and SoundPropagation lives only between those two
    /// points.
    explicit SoundPropagation(RoomService *roomService);
    ~SoundPropagation();

    /// Set the portal blocking factor between two rooms.
    /// Used by door open/close logic to control AI sound propagation.
    /// @param room1   First room ID
    /// @param room2   Second room ID
    /// @param factor  0.0 = fully open, 1.0 = fully blocked
    void setBlockingFactor(int room1, int room2, float factor);

    /// @return Blocking factor [0,1], or 0.0 (fully open) if not set.
    float getBlockingFactor(int room1, int room2) const;

    /// Set the LoudRoom transmission multiplier for a room (default 1.0 =
    /// no effect). Values < 1.0 dampen sound passing through, > 1.0 amplify.
    /// Loaded from P$LoudRoom on room objects.
    void setRoomTransmission(int32_t roomID, float transmission);

    /// @return Transmission factor for `roomID`, or 1.0 if unset.
    float getRoomTransmission(int32_t roomID) const;

    /// Wipe both maps (called on mission unload).
    void clear();

    /// Propagate a sound from `sourcePos` to `listenerPos` through the
    /// room portal graph. Resolves source/listener rooms via
    /// RoomService::roomFromPoint internally.
    SoundPropInfo propagateSound(const Vector3 &sourcePos,
                                 const Vector3 &listenerPos,
                                 float maxDist,
                                 uint32_t maxPaths,
                                 float maxPathDiff) const;

    /// Room-explicit overload. `sourceRoomID` / `listenerRoomID` of -1
    /// indicate "outside all rooms" — the underlying BFS falls back to
    /// euclidean distance. RoomService is responsible for resolving the
    /// integer IDs back to Room pointers, so callers can stash room IDs
    /// across frames without risking dangling pointers if the room
    /// database is rebuilt.
    SoundPropInfo propagateSound(const Vector3 &sourcePos,
                                 const Vector3 &listenerPos,
                                 int32_t sourceRoomID,
                                 int32_t listenerRoomID,
                                 float maxDist,
                                 uint32_t maxPaths,
                                 float maxPathDiff) const;

    /// Full-control overload used by loopStep, which wants to plug its
    /// own `chainOut` accumulator into the params (for the show_vpos
    /// debug overlay). The caller fills `paramsInOut.maxDist /
    /// maxPaths / maxPathDiff / chainOut`; this method overwrites the
    /// doorBlocking + loudRoom callbacks with the ones backed by our
    /// internal maps, then forwards into RoomService.
    SoundPropInfo propagateSoundWithParams(const Vector3 &sourcePos,
                                           const Vector3 &listenerPos,
                                           int32_t sourceRoomID,
                                           int32_t listenerRoomID,
                                           SoundPropParams &paramsInOut) const;

    /// Read-only access to the LoudRoom map for callers that need to
    /// iterate it (e.g. the acoustic verification dump in
    /// loadAuxiliarySoundData).
    const std::unordered_map<int32_t, float> &roomTransmissionMap() const {
        return mRoomTransmission;
    }

private:
    RoomService *mRoomService = nullptr;

    /// Portal blocking factors for AI hearing propagation.
    /// Key: (room1 << 16) | room2 (bidirectional — stored both ways).
    std::unordered_map<uint32_t, float> mBlockingFactors;

    /// Per-room LoudRoom transmission factor (keyed by room ID, default 1.0).
    /// Values < 1.0 dampen sound passing through the room, > 1.0 amplify.
    /// Parsed from P$LoudRoom property on room objects.
    std::unordered_map<int32_t, float> mRoomTransmission;
};

} // namespace Darkness

#endif // __SOUND_PROPAGATION_H
