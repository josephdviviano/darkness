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
    // Dark Engine plane convention: dot(n, p) >= d means inside.
    // Our Plane::getDistance uses dot(n, p) + d, so negate d.
    mPlane.d = -mPlane.d;

    mEdges.resize(mEdgeCount);

    for (size_t i = 0; i < mEdgeCount; ++i) {
        *sf >> mEdges[i];
        mEdges[i].d = -mEdges[i].d;
    }

    // room ID's to be linked
    int32_t src_room, dest_room;

    *sf >> src_room >> dest_room;

    // link through room service
    mSrcRoom = mOwner->getRoomByID(src_room);
    mDestRoom = mOwner->getRoomByID(dest_room);

    *sf >> mCenter;
    *sf >> mDestPortal;
}

//------------------------------------------------------
void RoomPortal::write(const FilePtr &sf) {
    // NOTE: mPlane.d and mEdges[i].d were negated on load (see read()) to convert
    // from Dark Engine convention to our Plane convention. If we ever re-save
    // ROOM_DB, we must negate d again here to restore the on-disk convention.
    *sf << mID << mIndex << mPlane << mEdgeCount;

    for (size_t i = 0; i < mEdgeCount; ++i) {
        *sf << mEdges[i];
    }

    *sf << mSrcRoom->getRoomID() << mDestRoom->getRoomID();
    *sf << mCenter;
    *sf << mDestPortal;
}

//------------------------------------------------------
bool RoomPortal::isInside(const Vector3 &point) {
    // iterate over all the planes. Have to have positive side
    for (size_t i = 0; i < mEdgeCount; ++i) {
        if (mEdges[i].getSide(point) == Plane::NEGATIVE_SIDE)
            return false;
    }

    return true;
}

//------------------------------------------------------
bool RoomPortal::raycast(const Vector3 &origin, const Vector3 &dir) const
{
    // Ray-portal plane intersection.
    // Our planes have d negated on load, so getDistance() = -original_dist.
    // The portal plane normal points from near-room to far-room.
    float denom = glm::dot(mPlane.normal, dir);
    if (std::fabs(denom) < 1e-7f)
        return false;  // ray parallel to portal plane

    // Parametric intersection: t where origin + t*dir hits the plane
    float t = -mPlane.getDistance(origin) / denom;

    // Compute intersection point (allow the full ray segment, not just [0,1])
    Vector3 hitPt = origin + dir * t;

    // Point-in-polygon test: check all edge half-planes.
    // After our d-negation, "inside the portal" means getDistance >= -epsilon
    // for all edge planes (positive side in our convention = inside).
    constexpr float kEpsilon = 0.0001f;
    for (uint32_t i = 0; i < mEdgeCount; ++i) {
        if (mEdges[i].getDistance(hitPt) < -kEpsilon)
            return false;  // outside this edge
    }

    return true;
}

//------------------------------------------------------
bool RoomPortal::getRaycastProjection(const Vector3 &origin, const Vector3 &dir,
                                       Vector3 &projPt) const
{
    // Intersect ray with portal plane to get the base point.
    float denom = glm::dot(mPlane.normal, dir);
    if (std::fabs(denom) < 1e-7f)
        return false;  // parallel

    float t = -mPlane.getDistance(origin) / denom;
    projPt = origin + dir * t;

    // Iterative edge clamping: for each violated edge, project the point
    // onto the edge plane. This is the Dark Engine's approximate nearest-point
    // algorithm (inspired by OPDE's portal projection approach).
    // After our d-negation: "outside" = getDistance < -epsilon.
    // Projection: move the point onto the edge plane by subtracting the
    // violation distance along the edge normal.
    constexpr float kEpsilon = 0.0001f;
    for (uint32_t i = 0; i < mEdgeCount; ++i) {
        float dist = mEdges[i].getDistance(projPt);
        if (dist < -kEpsilon) {
            // Point is outside this edge. Project onto the edge plane.
            // dist is negative (outside), so subtracting normal*dist moves inward.
            projPt -= mEdges[i].normal * dist;
        }
    }

    return true;
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
