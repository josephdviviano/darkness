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

#include "Room.h"
#include "FileCompat.h"
#include "RoomPortal.h"
#include "RoomService.h"

namespace Darkness {
/*----------------------------------------------------*/
/*------------------------ Room ----------------------*/
/*----------------------------------------------------*/
Room::Room(RoomService *owner)
    : mOwner(owner), mObjectID(0), mRoomID(0), mCenter(), mPortalCount(0),
      mPortals(), mPortalDistances() {
    // nothing
}

//------------------------------------------------------
Room::~Room() { clear(); }

//------------------------------------------------------
void Room::read(const FilePtr &sf) {
    clear();

    *sf >> mObjectID >> mRoomID >> mCenter;

    for (size_t i = 0; i < 6; ++i) {
        *sf >> mPlanes[i];
    }

    *sf >> mPortalCount;

    // read the room portals
    mPortals.resize(mPortalCount);
    for (size_t i = 0; i < mPortalCount; ++i) {
        mPortals[i] = std::make_unique<RoomPortal>(mOwner);
        mPortals[i]->read(sf);
    }

    // the portal-to-portal distances
    mPortalDistances.resize(mPortalCount * mPortalCount);
    sf->readElem(mPortalDistances.data(), sizeof(float), mPortalCount * mPortalCount);

    // the object ID lists
    // there are usually two lists of ID's in the room database
    // This could have worked based on some subscription system of id->location
    // providers
    uint32_t num_lists;
    *sf >> num_lists;

    for (size_t li = 0; li < num_lists; ++li) {
        // read the list
        IDSet ids = IDSet();
        mIDLists.push_back(ids);

        uint32_t count;
        *sf >> count;

        for (size_t i = 0; i < count; ++i) {
            int32_t id;
            *sf >> id;

            // indirect. RoomService will call us back...
            mOwner->_attachObjRoom(li, id, this);
        }
    }
}

//------------------------------------------------------
void Room::write(const FilePtr &sf) {
    *sf << mObjectID << mRoomID << mCenter;

    for (size_t i = 0; i < 6; ++i) {
        *sf << mPlanes[i];
    }

    // write the portals
    *sf << mPortalCount;

    for (size_t i = 0; i < mPortalCount; ++i) {
        mPortals[i]->write(sf);
    }

    // the portal-to-portal distances
    sf->writeElem(mPortalDistances.data(), sizeof(float), mPortalCount * mPortalCount);

    // the object ID lists
    uint32_t num_lists = mIDLists.size();
    *sf << num_lists;

    for (size_t li = 0; li < num_lists; ++li) {
        // read the list
        IDSet ids = mIDLists[li];

        uint32_t count = ids.size();
        *sf << count;

        IDSet::iterator i = ids.begin();

        for (; i != ids.end(); ++i) {
            int32_t id = *i;
            *sf << id;
        }
    }
}

//------------------------------------------------------
bool Room::isInside(const Vector3 &point) {
    // iterate over all the planes. Have to have positive side
    for (size_t i = 0; i < 6; ++i) {
        if (mPlanes[i].getSide(point) == Plane::NEGATIVE_SIDE)
            return false;
    }

    return true;
}

//------------------------------------------------------
RoomPortal *Room::getPortalForPoint(const Vector3 &pos) {
    for (size_t i = 0; i < mPortalCount; ++i) {
        RoomPortal *rp = mPortals[i].get();
        if (rp->isInside(pos))
            return rp;
    }

    return NULL;
}

//------------------------------------------------------
void Room::attachObj(size_t idset, int id) {
    // just verify if we have the idset handy
    if (mIDLists.size() <= idset) {
        mIDLists.resize(idset + 1);
    }

    IDSet &ir = mIDLists[idset];
    ir.insert(id);
}

//------------------------------------------------------
void Room::detachObj(size_t idset, int id) {
    if (mIDLists.size() <= idset) {
        return;
    } else {
        mIDLists[idset].erase(id);
    }
}

//------------------------------------------------------
void Room::clear() {
    mObjectID = 0;
    mRoomID = 0;
    mCenter = Vector3(0.0f);

    // unique_ptr handles portal deletion automatically
    mPortals.clear();
    mPortalCount = 0;
    mPortalDistances.clear();

    mIDLists.clear();
}
} // namespace Darkness
