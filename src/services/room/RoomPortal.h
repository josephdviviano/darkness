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

    /// reads the room portal from the specified file
    void read(const FilePtr &sf);

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
    // Source and destination rooms
    /// room number this portal goes to
    Room *mDestRoom;
    /// the source room number
    Room *mSrcRoom;
    /// center point of the portal. (should not be in solid space)
    Vector3 mCenter;
    /// portal ID on the other side of this portal
    int32_t mDestPortal;
};

/// Shared pointer to a room portal instance
typedef shared_ptr<RoomPortal> RoomPortalPtr;
} // namespace Darkness

#endif
