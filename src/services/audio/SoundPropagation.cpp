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

#include "SoundPropagation.h"

#include "AudioLog.h"
#include "room/Room.h"
#include "room/RoomService.h"

namespace Darkness {

//------------------------------------------------------
SoundPropagation::SoundPropagation(RoomService *roomService)
    : mRoomService(roomService)
{
}

//------------------------------------------------------
SoundPropagation::~SoundPropagation() = default;

//------------------------------------------------------
void SoundPropagation::setBlockingFactor(int room1, int room2, float factor)
{
    // Store bidirectionally so lookup works in either direction
    uint32_t key1 = (static_cast<uint32_t>(room1) << 16) |
                     static_cast<uint32_t>(room2 & 0xFFFF);
    uint32_t key2 = (static_cast<uint32_t>(room2) << 16) |
                     static_cast<uint32_t>(room1 & 0xFFFF);

    if (factor <= 0.0f) {
        // Remove blocking (fully open)
        mBlockingFactors.erase(key1);
        mBlockingFactors.erase(key2);
    } else {
        mBlockingFactors[key1] = factor;
        mBlockingFactors[key2] = factor;
    }

    AUDIO_LOG("[DOOR_BLOCK] setBlockingFactor room(%d,%d) factor=%.3f mapSize=%zu\n",
              room1, room2, factor, mBlockingFactors.size());
}

//------------------------------------------------------
float SoundPropagation::getBlockingFactor(int room1, int room2) const
{
    uint32_t key = (static_cast<uint32_t>(room1) << 16) |
                    static_cast<uint32_t>(room2 & 0xFFFF);
    auto it = mBlockingFactors.find(key);
    if (it != mBlockingFactors.end())
        return it->second;
    // Open portals have zero blocking. Room portals are real architectural
    // doorways (not BSP cell boundaries), so traversing an open doorway has
    // no artificial penalty. Door blocking is set explicitly via
    // setBlockingFactor() when doors close.
    return 0.0f;
}

//------------------------------------------------------
void SoundPropagation::setRoomTransmission(int32_t roomID, float transmission)
{
    // Only store non-default values to keep the lookup table small.
    if (transmission == 1.0f) {
        mRoomTransmission.erase(roomID);
    } else {
        mRoomTransmission[roomID] = transmission;
    }
}

//------------------------------------------------------
float SoundPropagation::getRoomTransmission(int32_t roomID) const
{
    auto it = mRoomTransmission.find(roomID);
    return (it != mRoomTransmission.end()) ? it->second : 1.0f;
}

//------------------------------------------------------
void SoundPropagation::clear()
{
    mBlockingFactors.clear();
    mRoomTransmission.clear();
}

//------------------------------------------------------
SoundPropInfo SoundPropagation::propagateSound(const Vector3 &sourcePos,
                                                const Vector3 &listenerPos,
                                                float maxDist,
                                                uint32_t maxPaths,
                                                float maxPathDiff) const
{
    if (!mRoomService || !mRoomService->isLoaded())
        return {};

    Room *sourceRoom = mRoomService->roomFromPoint(sourcePos);
    Room *listenerRoom = mRoomService->roomFromPoint(listenerPos);

    int32_t srcID = sourceRoom ? sourceRoom->getRoomID() : -1;
    int32_t lstID = listenerRoom ? listenerRoom->getRoomID() : -1;

    // If source or listener is outside all room OBBs, the room-explicit
    // overload handles it with a euclidean distance fallback. This matches
    // the original Dark Engine behavior: objects outside the room database
    // cannot propagate sound through the room portal graph. The ambient
    // system uses euclidean distance directly, so this fallback only affects
    // non-ambient voices (footsteps, one-shots) that happen to be outside rooms.
    return propagateSound(sourcePos, listenerPos, srcID, lstID,
                          maxDist, maxPaths, maxPathDiff);
}

//------------------------------------------------------
SoundPropInfo SoundPropagation::propagateSound(const Vector3 &sourcePos,
                                                const Vector3 &listenerPos,
                                                int32_t sourceRoomID,
                                                int32_t listenerRoomID,
                                                float maxDist,
                                                uint32_t maxPaths,
                                                float maxPathDiff) const
{
    SoundPropParams params;
    params.maxDist     = maxDist;
    params.maxPaths    = maxPaths;
    params.maxPathDiff = maxPathDiff;
    return propagateSoundWithParams(sourcePos, listenerPos,
                                    sourceRoomID, listenerRoomID,
                                    params);
}

//------------------------------------------------------
SoundPropInfo SoundPropagation::propagateSoundWithParams(
    const Vector3 &sourcePos,
    const Vector3 &listenerPos,
    int32_t sourceRoomID,
    int32_t listenerRoomID,
    SoundPropParams &params) const
{
    // BFS body lives in RoomService::propagateSoundPath — that lets the
    // headless trace-path tool exercise the same graph traversal without
    // an audio backend. We thread our runtime cost data through as
    // callbacks: door blocking (closed doors set via setBlockingFactor)
    // and LoudRoom multipliers (from P$LoudRoom). Anything else about the
    // result — the per-portal raycast / edge projection, the
    // virtualPosition anchor selection, the BFS-failure log — happens
    // inside RoomService and is unchanged behaviorally.
    if (!mRoomService)
        return {};

    // Resolve the integer room IDs back to Room pointers at call time.
    // Looking up here (rather than caching a Room* on the caller side)
    // avoids the dangling-pointer hazard if the room database is rebuilt
    // between the caller storing the ID and the BFS running. -1 means
    // "outside all rooms"; RoomService treats a null Room* as the same
    // fallback case (euclidean distance, no portal traversal).
    Room *sourceRoom   = (sourceRoomID   >= 0) ? mRoomService->getRoomByID(sourceRoomID)   : nullptr;
    Room *listenerRoom = (listenerRoomID >= 0) ? mRoomService->getRoomByID(listenerRoomID) : nullptr;

    params.doorBlocking = [this](int32_t a, int32_t b) {
        return getBlockingFactor(a, b);
    };
    params.loudRoom = [this](int32_t roomID) -> float {
        return getRoomTransmission(roomID);
    };

    return mRoomService->propagateSoundPath(
        sourcePos, listenerPos, sourceRoom, listenerRoom, params);
}

} // namespace Darkness
