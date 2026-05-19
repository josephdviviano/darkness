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

#ifndef __ROOMPORTAL_H
#define __ROOMPORTAL_H

#include <vector>

#include "File.h"
#include "RoomCommon.h"
#include "SharedPtr.h"
#include "config.h"
#include "integers.h"

#include "DarknessMath.h"

namespace Darkness {
/** @brief A room portal. Room portals connect two Room instances (doorways)
 */
class RoomPortal {
public:
    RoomPortal(RoomService *owner);
    ~RoomPortal();

    /// reads the room portal from the specified file. Stores the
    /// source/destination room IDs but does NOT resolve them to Room
    /// pointers — those rooms may not be loaded yet at this point. Call
    /// linkRooms() once every room has been read.
    void read(const FilePtr &sf);

    /// Resolve mSrcRoom and mDestRoom from the IDs captured during read().
    /// Must be called after every Room has been registered in
    /// RoomService::mRoomsByID. Logs a [FALLBACK] and leaves the pointer
    /// null if either ID does not resolve — that portal is then unusable.
    void linkRooms();

    /// writes the room portal into the specified file
    void write(const FilePtr &sf);

    bool isInside(const Vector3 &point);

    /// getter for the room that is the target of this portal
    inline Room *getFarRoom() const { return mDestRoom; };

    /// getter for the room that is the source for this portal
    inline Room *getNearRoom() const { return mSrcRoom; };

    // --- Read-only accessors for IWorldQuery ---

    /// Portal ID
    int32_t getPortalID() const { return mID; }

    /// Center point of the portal (should not be in solid space)
    const Vector3 &getCenter() const { return mCenter; }

    /// Plane this portal lies on
    const Plane &getPlane() const { return mPlane; }

    /// Index of this portal in the room's portal list
    uint32_t getIndex() const { return mIndex; }

    /// Portal ID on the other side of this portal
    int32_t getDestPortalID() const { return mDestPortal; }

    /// Number of edge planes defining the portal boundary
    uint32_t getEdgeCount() const { return mEdgeCount; }

    /// Get edge plane by index (bounds checked, returns identity plane if OOB)
    const Plane &getEdgePlane(uint32_t i) const {
        static const Plane identity{};
        return (i < mEdgeCount) ? mEdges[i] : identity;
    }

    /// Test whether a ray from origin in direction dir passes through the portal.
    /// Uses ray-plane intersection followed by point-in-edge-planes test.
    /// @param origin  Ray origin
    /// @param dir     Ray direction (NOT necessarily unit length — treated as a segment endpoint offset)
    /// @return true if the ray passes through the portal polygon
    bool raycast(const Vector3 &origin, const Vector3 &dir) const;

    /// Closest point on the (convex) portal polygon to an external
    /// reference point. Drives the sound-propagation anchor projection
    /// algorithm:
    ///
    ///   1. Project `ref` onto the portal's plane.
    ///   2. If the projection lies inside the polygon (all edge planes
    ///      satisfied), return it.
    ///   3. Otherwise clamp iteratively against the MOST-violated edge
    ///      plane until inside. Always picking the worst-violated edge
    ///      first guarantees the result lands on an actual edge (not
    ///      sequentially-clamped to a polygon corner), which is the
    ///      geometric source of the audible "anchor pulse" the
    ///      legacy `getRaycastProjection` used to produce when
    ///      multiple edges were violated at once.
    ///
    /// This is the only "where on the polygon does the sound path bend"
    /// primitive in the engine. Both the BFS hop-cost heuristic and the
    /// per-portal anchor projection share it — keeping chain selection
    /// and final-distance computation geometrically consistent.
    ///
    /// @param ref  External reference point (source position, listener
    ///             position, or a previously-resolved bend point on an
    ///             adjacent polygon)
    /// @return The closest point on the polygon to `ref`.
    Vector3 closestPointOnPolygon(const Vector3 &ref) const;

private:
    void clear();

    /// Owner service
    RoomService *mOwner;
    /// Portal ID
    int32_t mID;
    /// The index of this portal in the room's portal list
    uint32_t mIndex;
    /// Plane this portal lies on
    Plane mPlane;
    /// Number of portal edges
    uint32_t mEdgeCount;
    /// Plane list - planes that make up the portal
    std::vector<Plane> mEdges;
    // Source and destination rooms. Pointers stay null until linkRooms()
    // runs in a second pass — the on-disk IDs are captured by read() into
    // the mSrc/mDestRoomID fields below, and only resolved once every Room
    // has been registered in RoomService::mRoomsByID. Reading rooms
    // sequentially and resolving inline produced null pointers for any
    // forward reference, breaking the room-portal graph.
    /// room number this portal goes to
    Room *mDestRoom;
    /// the source room number
    Room *mSrcRoom;
    /// Captured on-disk room IDs, resolved to mSrc/mDestRoom by linkRooms()
    int32_t mSrcRoomID = -1;
    int32_t mDestRoomID = -1;
    /// center point of the portal. (should not be in solid space)
    Vector3 mCenter;
    /// portal ID on the other side of this portal
    int32_t mDestPortal;
};

/// Shared pointer to a room portal instance
typedef shared_ptr<RoomPortal> RoomPortalPtr;
} // namespace Darkness

#endif
