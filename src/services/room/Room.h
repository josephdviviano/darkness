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
/** @file Room.h
 * @brief A single room in room database.
 */

#ifndef __ROOM_H
#define __ROOM_H

#include <set>
#include <vector>

#include "Array.h"
#include "File.h"
#include "Plane.h"
#include "Quaternion.h"
#include "RoomCommon.h"
#include "SharedPtr.h"
#include "Vector3.h"
#include "config.h"
#include "integers.h"

namespace Darkness {
/** @brief A single Room. Rooms are space bounded elements that are used
 * for sound propagation, path finding and as a script message sources.
 */
class Room {
public:
    Room(RoomService *owner);
    ~Room();

    void read(const FilePtr &sf);
    void write(const FilePtr &sf);

    int32_t getObjectID() const { return mObjectID; };
    int16_t getRoomID() const { return mRoomID; };

    bool isInside(const Vector3 &point);

    /** Gets portal for a given position
     * @param pos The position to find portal for
     * @return RoomPortal for the given point, or NULL if none found */
    RoomPortal *getPortalForPoint(const Vector3 &pos);

    /** Attaches the given object to the room (into specified id set)
     *  @param idset the id set to use (0/1 typically)
     *  @param id the object id to attach
     */
    void attachObj(size_t idset, int id);

    /** Detaches the given object from the room (into specified id set)
     *  @param idset the id set to use (0/1 typically)
     *  @param id the object id to detach
     */
    void detachObj(size_t idset, int id);

    // --- Read-only accessors for IWorldQuery ---

    /// Center point of the room (should not be in solid space)
    const Vector3 &getCenter() const { return mCenter; }

    /// Number of portals connecting this room to neighbors
    uint32_t getPortalCount() const { return mPortalCount; }

    /// Get portal by index, or nullptr if out of range
    RoomPortal *getPortal(uint32_t index) const {
        if (index >= mPortalCount)
            return nullptr;
        return mPortals[index];
    }

    /// Get the 6 bounding planes that define this room's volume
    const Plane *getBoundingPlanes() const { return mPlanes; }

    /// Get a copy of the object IDs in the specified id set
    /// @param idset 0=objects, 1=creatures typically
    std::vector<int> getObjectIDs(size_t idset) const {
        std::vector<int> result;
        if (idset < mIDLists.size()) {
            const auto &s = mIDLists[idset];
            result.assign(s.begin(), s.end());
        }
        return result;
    }

private:
    /// clears the room into an empty state, drops all allocations
    void clear();

    /// Owner service
    RoomService *mOwner;

    /// Object ID
    int32_t mObjectID;
    /// Room number
    int16_t mRoomID;
    /// Center point of the room. Should not be in solid space or overlapping
    /// another room
    Vector3 mCenter;
    /// Bounding box as described by 6 enclosing planes
    Plane mPlanes[6];
    /// Portal count
    uint32_t mPortalCount;
    /// Portal list
    SimpleArray<RoomPortal *> mPortals;
    /// Portal to portal distances (a 2d array, single index here for simplicity
    /// of allocations)
    float *mPortalDistances;

    typedef std::set<int> IDSet;
    typedef std::vector<IDSet> IDLists;

    /// lists of ID's
    IDLists mIDLists;
};

/// Shared pointer to a room instance
typedef shared_ptr<Room> RoomPtr;
} // namespace Darkness

#endif
