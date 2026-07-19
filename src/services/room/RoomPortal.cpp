/******************************************************************************
 *
 *    This file is part of openDarkEngine project
 *    Copyright (C) 2005-2009 openDarkEngine team
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
 *	  $Id$
 *
 *****************************************************************************/

#include "RoomPortal.h"
#include "FileCompat.h"
#include "Room.h"
#include "RoomService.h"

#include <cmath>
#include <cstdio>
#include <limits>

namespace Darkness {
/*----------------------------------------------------*/
/*--------------------- RoomPortal -------------------*/
/*----------------------------------------------------*/
RoomPortal::RoomPortal(RoomService *owner) : mOwner(owner) {}

//------------------------------------------------------
RoomPortal::~RoomPortal() { clear(); }

//------------------------------------------------------
void RoomPortal::read(const FilePtr &sf) {
    *sf >> mID >> mIndex >> mPlane >> mEdgeCount;
    // Portal planes and edge half-planes are stored on disk in the SAME
    // equation form as our Plane struct (`dot(normal, point) + d = 0`).
    // No transformation is applied; data is loaded verbatim.
    //
    // Portal plane normal direction (per Dark Engine convention): the
    // normal points INTO the FAR room — the room on the OTHER side of
    // the portal from the owner (= the room whose portal list this
    // entry lives in). So `getDistance(pos) > 0` means the point is on
    // the far-room side. RoomService::roomFromPoint uses this: when
    // disambiguating between two overlapping OBBs whose portal points
    // owner→neighbor, positive distance eliminates the owner.
    //
    // Edge plane normals (the half-planes that bound the portal
    // polygon) point OUTWARD from the polygon interior. So the
    // polygon membership test is "every edge plane gives getDistance
    // ≤ 0 = inside the polygon". Matches the original engine's
    // portal convention. RoomPortal::isInside / raycast /
    // getRaycastProjection follow this directly.
    //
    // History: an earlier version of this code negated `d`, which moved
    // every portal plane to its antipodal position across the origin —
    // produced symmetric audio results in the disambiguator only by
    // happy accident, and made the portal polygon overlay render in
    // the sky for any level with play space at negative Z. A later
    // attempt to also flip the normal kept the plane location correct
    // but inverted the disambiguator's polarity, causing source
    // positions to resolve to the wrong room. The correct fix is to
    // leave both fields alone.
    //
    // Note: room *bounding* planes use a different disk convention
    // (Option 1 with OUTWARD normals) and DO need a flip — see
    // `Room::read` for the conversion there.

    mEdges.resize(mEdgeCount);

    for (size_t i = 0; i < mEdgeCount; ++i) {
        *sf >> mEdges[i];
    }

    // Capture room IDs but defer the Room* lookup to linkRooms().
    // Rooms are loaded sequentially; resolving here would silently null
    // out every forward reference (portal points to a room that hasn't
    // been added to mRoomsByID yet), which broke the entire room-portal
    // graph and made all cross-room sound propagation BFS return
    // reached=false. The two-phase load avoids this.
    *sf >> mSrcRoomID >> mDestRoomID;

    *sf >> mCenter;
    *sf >> mDestPortal;
}

//------------------------------------------------------
void RoomPortal::linkRooms() {
    // The on-disk "src_room" and "dest_room" fields are INVERTED relative
    // to how the rest of the codebase uses near/far:
    //   - Disk `dest_room` = the room that OWNS this portal record (the
    //     room whose portal list this entry lives in). This is what
    //     callers expect from getNearRoom().
    //   - Disk `src_room`  = the room on the OTHER SIDE of the portal
    //     (the neighbor this portal connects to). This is what callers
    //     expect from getFarRoom().
    //
    // Verified empirically: every portal stored in room 0's list has
    // `dest_room == 0` and `src_room` ∈ {4, 6, 21, 82} (the four neighbors
    // listed in [PORTAL_LINK] from a Thief 2 mission). Without this swap,
    // getFarRoom() returns the owning room (self), which makes BFS
    // through the portal graph degenerate to a self-loop for every step
    // — all cross-room voices then fail propagation and get silenced.
    //
    // OPDE has the same field naming and silently inherits the
    // inversion, but its codebase never actually traverses the portal
    // graph at runtime, so the bug never manifested there.
    mSrcRoom  = mOwner->getRoomByID(mDestRoomID);  // owner → near
    mDestRoom = mOwner->getRoomByID(mSrcRoomID);   // neighbor → far

    if (!mSrcRoom) {
        std::fprintf(stderr,
            "[FALLBACK] RoomPortal::linkRooms: portal id=%d nearRoomID=%d "
            "not found in RoomService — portal unusable\n",
            mID, mDestRoomID);
    }
    if (!mDestRoom) {
        std::fprintf(stderr,
            "[FALLBACK] RoomPortal::linkRooms: portal id=%d farRoomID=%d "
            "not found in RoomService — portal unusable\n",
            mID, mSrcRoomID);
    }
}

//------------------------------------------------------
void RoomPortal::write(const FilePtr &sf) {
    // Disk plane convention matches our Plane struct directly; no
    // transformation needed (see read() for the rationale).
    *sf << mID << mIndex << mPlane << mEdgeCount;

    for (size_t i = 0; i < mEdgeCount; ++i) {
        *sf << mEdges[i];
    }

    // Disk's `src_room` field = the other (far) room; disk's
    // `dest_room` field = the owning (near) room. linkRooms() swaps
    // these into mSrcRoom (near) and mDestRoom (far). Reverse the
    // swap here so the on-disk representation round-trips.
    *sf << mDestRoom->getRoomID() << mSrcRoom->getRoomID();
    *sf << mCenter;
    *sf << mDestPortal;
}

//------------------------------------------------------
// Edge-plane convention (per the original Dark Engine):
//   • Each edge plane's normal points OUTWARD from the polygon.
//   • Inside the polygon → getDistance ≤ 0 for every edge plane.
//   • Outside (past one edge) → that edge gives getDistance > 0.
//
// Verified against the original engine's portal raycast, which rejects
// an intersection point when any edge-plane distance exceeds a small
// on-plane epsilon (the point lies outside the polygon).
//
// Earlier versions of this file had the opposite convention here
// (treating negative side as outside). With the data we read verbatim
// from disk, that made every point-in-polygon test fail, every BFS
// path always took the anchor-projection branch, and virtualPosition
// always landed on a polygon corner (e.g. (79.45,-55.5366,-60.5) for
// the portal between MISS6 rooms 217 and 219 — the polygon's bottom-
// far corner). The downstream effect: sound through every portal got
// an inflated path length, often pushing it past the radius cutoff
// and silencing the source even between adjacent open-air rooms.

//------------------------------------------------------
bool RoomPortal::isInside(const Vector3 &point) {
    // Inside iff no edge plane reports a POSITIVE distance to the point.
    for (size_t i = 0; i < mEdgeCount; ++i) {
        if (mEdges[i].getSide(point) == Plane::POSITIVE_SIDE)
            return false;
    }

    return true;
}

//------------------------------------------------------
bool RoomPortal::raycast(const Vector3 &origin, const Vector3 &dir) const
{
    // Ray-portal plane intersection. The portal plane normal points
    // from near-room into the far-room (per Dark Engine convention).
    float denom = glm::dot(mPlane.normal, dir);
    if (std::fabs(denom) < 1e-7f)
        return false;  // ray parallel to portal plane

    // Parametric intersection: t where origin + t*dir hits the plane
    float t = -mPlane.getDistance(origin) / denom;

    // Compute intersection point (allow the full ray segment, not just [0,1])
    Vector3 hitPt = origin + dir * t;

    // Point-in-polygon test: edge normals point OUTWARD from the
    // polygon, so inside = getDistance ≤ epsilon for every edge.
    constexpr float kEpsilon = 0.0001f;
    for (uint32_t i = 0; i < mEdgeCount; ++i) {
        if (mEdges[i].getDistance(hitPt) > kEpsilon)
            return false;  // outside this edge
    }

    return true;
}

//------------------------------------------------------
Vector3 RoomPortal::closestPointOnPolygon(const Vector3 &ref) const
{
    // Step 1: project ref onto the portal's plane. mPlane.getDistance is
    // the signed offset; subtracting normal*offset moves the point onto
    // the plane.
    Vector3 p = ref - mPlane.normal * mPlane.getDistance(ref);

    if (mEdgeCount == 0) return p;

    // Step 2: if the projection lies inside the polygon (every edge in
    // its negative half-space, with a small ε for floating-point slack),
    // we are done — that point is the genuine nearest.
    //
    // Step 3: otherwise clamp ITERATIVELY against the MOST-VIOLATED edge
    // plane each pass. Picking the worst edge first guarantees we land on
    // an actual edge (a line, not a corner) for the common case where two
    // or three edges are violated simultaneously. The legacy ordering
    // (sequential edge index) would bounce the point to a corner — the
    // root cause of the audible "anchor pulse" investigated under
    // PLAN.SOUND_PROPAGATION_NEXT.md SP-2.
    //
    // Convex polygons converge in at most mEdgeCount passes in practice
    // (each pass either accepts or strictly reduces the worst violation);
    // we bound the loop generously as a safety net.
    constexpr float kEps = 1e-4f;
    const uint32_t kMaxIter = mEdgeCount * 2 + 4;
    for (uint32_t iter = 0; iter < kMaxIter; ++iter) {
        float worst = -std::numeric_limits<float>::infinity();
        uint32_t worstIdx = 0;
        for (uint32_t i = 0; i < mEdgeCount; ++i) {
            float d = mEdges[i].getDistance(p);
            if (d > worst) {
                worst = d;
                worstIdx = i;
            }
        }
        if (worst <= kEps) break;
        p -= mEdges[worstIdx].normal * worst;
    }
    return p;
}

//------------------------------------------------------
void RoomPortal::clear() {
    mID = 0;
    mIndex = 0;
    mPlane = Plane();
    mEdgeCount = 0;
    mEdges.clear();
    mSrcRoom = NULL;
    mDestRoom = NULL;
    mCenter = Vector3(0.0f);
    mDestPortal = 0;
}
} // namespace Darkness
