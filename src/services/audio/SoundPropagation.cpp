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
#include "room/RoomPortal.h"
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
void SoundPropagation::setBlockingCore(int room1, int room2, float factor)
{
    // Bidirectional — store both keys. No adjacent-room spread (see header).
    const uint32_t key1 = (static_cast<uint32_t>(room1) << 16) |
                          static_cast<uint32_t>(room2 & 0xFFFF);
    const uint32_t key2 = (static_cast<uint32_t>(room2) << 16) |
                          static_cast<uint32_t>(room1 & 0xFFFF);

    if (factor <= 0.0f) {
        mBlockingFactors.erase(key1);
        mBlockingFactors.erase(key2);
    } else {
        mBlockingFactors[key1] = factor;
        mBlockingFactors[key2] = factor;
    }
}

//------------------------------------------------------
void SoundPropagation::blockAdjacentRooms(int room1, int room2, float factor)
{
    if (!mRoomService) return;
    Room *r1 = mRoomService->getRoomByID(room1);
    Room *r2 = mRoomService->getRoomByID(room2);
    if (!r1 || !r2) return;

    // Any room R portal-adjacent to BOTH room1 and room2 sits in the shared
    // doorway footprint; the door blocks (room1,R) and (room2,R) too, so a
    // wide opening spanning >2 room subdivisions is not under-blocked. O(P1
    // x P2) over the two rooms' portal lists — a handful each. The spread
    // calls only the core setter, so it never re-enters this function.
    for (uint32_t i = 0; i < r1->getPortalCount(); ++i) {
        RoomPortal *p1 = r1->getPortal(i);
        Room *far1 = p1 ? p1->getFarRoom() : nullptr;
        if (!far1) continue;
        const int32_t rr = far1->getRoomID();
        if (rr == room2) continue;   // the door's own pair, set below
        for (uint32_t j = 0; j < r2->getPortalCount(); ++j) {
            RoomPortal *p2 = r2->getPortal(j);
            Room *far2 = p2 ? p2->getFarRoom() : nullptr;
            if (!far2) continue;
            if (far2->getRoomID() == rr) {
                setBlockingCore(room1, rr, factor);
                setBlockingCore(room2, rr, factor);
                break;   // R found for this p1; move to r1's next portal
            }
        }
    }
}

//------------------------------------------------------
void SoundPropagation::setBlockingFactor(int room1, int room2, float factor)
{
    setBlockingCore(room1, room2, factor);
    // Multi-room doorway spread (original-engine parity): also block the
    // pairs to any room adjacent to both of the door's rooms. factor <= 0
    // erases those too.
    blockAdjacentRooms(room1, room2, factor);
    // BFS reads the map fresh on each query — no cache invalidation needed.
    AUDIO_LOG("[DOOR_BLOCK] setBlockingFactor room(%d,%d) factor=%.3f mapSize=%zu\n",
              room1, room2, factor, mBlockingFactors.size());
}

//------------------------------------------------------
float SoundPropagation::getBlockingFactor(int room1, int room2) const
{
    const uint32_t key = (static_cast<uint32_t>(room1) << 16) |
                         static_cast<uint32_t>(room2 & 0xFFFF);
    auto it = mBlockingFactors.find(key);
    return (it != mBlockingFactors.end()) ? it->second : 0.0f;
}

//------------------------------------------------------
void SoundPropagation::setRoomTransmission(int32_t roomID, float transmission)
{
    // Sparse storage — drop default values. BFS reads the map fresh per query.
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

    const int32_t srcID = sourceRoom ? sourceRoom->getRoomID() : -1;
    const int32_t lstID = listenerRoom ? listenerRoom->getRoomID() : -1;

    // -1 IDs trigger the Euclidean fallback inside the room-explicit
    // overload — matches the original engine: objects outside the room
    // database can't propagate through portals.
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
    // BFS lives in RoomService::propagateSoundPath; we thread our cost
    // tables in as callbacks (doorBlocking + loudRoom). Per-portal
    // raycasts, virtualPosition anchor selection, and BFS-failure logs
    // are unchanged inside RoomService.
    if (!mRoomService)
        return {};

    // Resolve IDs at call time so a rebuilt room DB can't cause dangling
    // Room pointers. -1 → nullptr → RoomService falls back to Euclidean.
    Room *sourceRoom   = (sourceRoomID   >= 0) ? mRoomService->getRoomByID(sourceRoomID)   : nullptr;
    Room *listenerRoom = (listenerRoomID >= 0) ? mRoomService->getRoomByID(listenerRoomID) : nullptr;

    params.doorBlocking = [this](int32_t a, int32_t b) {
        return getBlockingFactor(a, b);
    };
    params.loudRoom = [this](int32_t roomID) -> float {
        return getRoomTransmission(roomID);
    };
    // BSP-aware LoS refines chain bends — without it, BFS may return a
    // wall-piercing best-effort chain.
    if (mLineOfSightFn && !params.losClear) {
        params.losClear = mLineOfSightFn;
    }

    return mRoomService->propagateSoundPath(
        sourcePos, listenerPos, sourceRoom, listenerRoom, params);
}

} // namespace Darkness
