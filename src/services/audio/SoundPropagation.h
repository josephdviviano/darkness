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
/// Portal-graph sound propagation + runtime door / loud-room cost tables.
/// Owns the per-portal blocking factor map and per-room LoudRoom
/// transmission map; forwards propagateSound() into
/// RoomService::propagateSoundPath with those tables wired as callbacks.
/// Main-thread only; internal maps are not locked.

#include "DarknessMath.h"
#include "room/RoomService.h"  // SoundPropInfo / SoundPropParams

#include <cstdint>
#include <functional>
#include <unordered_map>
#include <utility>

namespace Darkness {

class Room;
class RoomService;

/// Matches AudioService::SOUND_MAX_DIST so the public propagateSound
/// signature on AudioService is unchanged.
constexpr float kDefaultSoundMaxDist = 200.0f;

class SoundPropagation {
public:
    /// `roomService` must outlive this object.
    explicit SoundPropagation(RoomService *roomService);
    ~SoundPropagation();

    /// Portal blocking factor: 0=open, 1=fully blocked. Drives AI hearing
    /// + door LPF.
    void setBlockingFactor(int room1, int room2, float factor);
    /// 0.0 if unset.
    float getBlockingFactor(int room1, int room2) const;

    /// LoudRoom transmission multiplier (<1 dampens, >1 amplifies sound
    /// passing through). Loaded from P$LoudRoom.
    void setRoomTransmission(int32_t roomID, float transmission);
    /// 1.0 if unset.
    float getRoomTransmission(int32_t roomID) const;

    /// Mission-unload reset.
    void clear();

    /// Propagate sourcePos → listenerPos through the portal graph.
    /// Resolves rooms via RoomService::roomFromPoint.
    SoundPropInfo propagateSound(const Vector3 &sourcePos,
                                 const Vector3 &listenerPos,
                                 float maxDist,
                                 uint32_t maxPaths,
                                 float maxPathDiff) const;

    /// Room-explicit. -1 = outside all rooms → BFS falls back to
    /// Euclidean. Callers can stash room IDs across frames since
    /// RoomService resolves them internally.
    SoundPropInfo propagateSound(const Vector3 &sourcePos,
                                 const Vector3 &listenerPos,
                                 int32_t sourceRoomID,
                                 int32_t listenerRoomID,
                                 float maxDist,
                                 uint32_t maxPaths,
                                 float maxPathDiff) const;

    /// Full-control overload used by loopStep — caller fills maxDist/
    /// maxPaths/maxPathDiff/chainOut; this method overwrites the
    /// doorBlocking + loudRoom callbacks with ones backed by our maps.
    SoundPropInfo propagateSoundWithParams(const Vector3 &sourcePos,
                                           const Vector3 &listenerPos,
                                           int32_t sourceRoomID,
                                           int32_t listenerRoomID,
                                           SoundPropParams &paramsInOut) const;

    /// Read-only access for the acoustic-verification dump.
    const std::unordered_map<int32_t, float> &roomTransmissionMap() const {
        return mRoomTransmission;
    }

    /// BSP-aware line-of-sight callback for chain reconstruction. When set,
    /// per-bend segments get raycast against the BSP and bends that land
    /// on wall-overlapping portal regions get refined or dropped.
    using LineOfSightFn = std::function<bool(const Vector3 &a, const Vector3 &b)>;
    void setLineOfSightFn(LineOfSightFn fn) { mLineOfSightFn = std::move(fn); }

private:
    RoomService *mRoomService = nullptr;
    LineOfSightFn mLineOfSightFn;

    /// Contribute one door's blocking to a room-pair boundary and recompute
    /// that boundary's EFFECTIVE factor. Contributions are keyed by the
    /// contributing door's own room pair (`originKey`) so several doors can
    /// touch the same boundary without clobbering one another: a wide door's
    /// spread onto (room1,R) and the door actually on (room1,R) coexist, and
    /// each can be raised / lowered / cleared independently. Effective factor
    /// = MAX over the boundary's live contributions (the tightest-sealing door
    /// governs the boundary). NO adjacent-room propagation here — the spread
    /// is added by `setBlockingFactor` via `blockAdjacentRooms`, which calls
    /// only this, so it cannot recurse.
    void contributeBlocking(int room1, int room2, uint32_t originKey,
                            float factor);

    /// Multi-room doorway spread: a door on the (room1,room2) boundary also
    /// blocks any room R that is portal-adjacent to BOTH room1 and room2
    /// (the pairs (room1,R) and (room2,R)) so a wide doorway whose opening
    /// spans more than two room subdivisions is not under-blocked. Every
    /// spread contribution carries the door's own `originKey`, so opening the
    /// door clears exactly its own contributions and never another door's.
    /// factor <= 0 erases this door's contribution.
    void blockAdjacentRooms(int room1, int room2, uint32_t originKey,
                            float factor);

    /// Undirected room-pair key: (min<<16)|max, so (a,b) and (b,a) collide.
    static uint32_t pairKey(int room1, int room2) {
        const int lo = room1 < room2 ? room1 : room2;
        const int hi = room1 < room2 ? room2 : room1;
        return (static_cast<uint32_t>(lo) << 16) |
               static_cast<uint32_t>(hi & 0xFFFF);
    }

    /// Effective blocking per boundary: undirected pairKey → MAX contribution.
    std::unordered_map<uint32_t, float> mBlockingFactors;
    /// Live per-boundary contributions: pairKey(boundary) → ( pairKey(door) →
    /// factor ). The effective map above is the per-boundary max over these,
    /// so one door lowering/clearing its contribution never wipes another's.
    std::unordered_map<uint32_t, std::unordered_map<uint32_t, float>>
        mBlockingContrib;
    /// Per-room transmission factor (default 1.0).
    std::unordered_map<int32_t, float> mRoomTransmission;
};

} // namespace Darkness

#endif // __SOUND_PROPAGATION_H
