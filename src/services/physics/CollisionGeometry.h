/******************************************************************************
 *
 *    This file is part of the Darkness engine
 *    Copyright (C) 2024-2026 Darkness contributors
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

/******************************************************************************
 *
 *    CollisionGeometry — sphere-vs-cell-polygon collision queries over WR
 *    world geometry data.
 *
 *    This generalizes the renderer's applyCameraCollision() into a reusable
 *    physics utility. The key differences from the camera-only version:
 *    - Accepts arbitrary sphere position and radius (not just camera)
 *    - Returns contact normals and penetration depths (for constraint response)
 *    - Handles portal transitions (sphere near portal checks adjacent cells)
 *    - Multi-iteration constraint projection with contact accumulation
 *
 *    The Darkness collision model:
 *    1. For each solid polygon in the cell, test sphere-vs-plane distance
 *    2. If dist < radius, project sphere center onto plane and test point-in-polygon
 *    3. Accumulate contact normals, then remove velocity components along contacts
 *    4. Iterate 3 times to handle corner/edge convergence
 *
 *    WR cell conventions:
 *    - Cell planes face inward (positive = inside cell)
 *    - Solid polygons = polygons[0 .. numPolygons-numPortals-1]
 *    - Portal polygons = polygons[numSolid .. numPolygons-1]
 *
 *    Depends on WRChunkParser.h and CellGeometry.h (both bgfx-free).
 *
 *****************************************************************************/

#ifndef __COLLISIONGEOMETRY_H
#define __COLLISIONGEOMETRY_H

#include <cmath>
#include <cstdint>
#include <vector>

#include "DarknessMath.h"
#include "WRChunkParser.h"
#include "CellGeometry.h"

namespace Darkness {

/// Result of a single sphere-polygon contact test
struct SphereContact {
    Vector3 normal;        // contact normal (pointing away from wall, into cell)
    float penetration;     // depth of sphere penetration (positive = overlapping)
    int32_t cellIdx;       // which cell the contact polygon belongs to
    int32_t polyIdx;       // polygon index within that cell
    int32_t textureIdx;    // texture index (-1 if untextured)
};

/// Collision geometry wrapper over WR parsed data.
/// Provides sphere-vs-cell-polygon collision queries for the physics system.
/// The WRParsedData must outlive this object.
class CollisionGeometry {
public:
    /// Construct with a reference to parsed WR cell data.
    /// The WR data must be valid for the lifetime of this object.
    explicit CollisionGeometry(const WRParsedData &wr) : mWR(wr) {}

    // ── Cell queries (delegated to CellGeometry.h) ──

    /// Find the cell containing a point. Returns -1 if outside all cells.
    /// When near a water boundary, prefers the smallest containing cell.
    int32_t findCell(const Vector3 &pos) const {
        return findCameraCell(mWR, pos.x, pos.y, pos.z);
    }

    /// Find the cell containing a point, given raw coordinates.
    int32_t findCell(float x, float y, float z) const {
        return findCameraCell(mWR, x, y, z);
    }

    /// Get the media type (1=air, 2=water) at a position.
    uint8_t getMediaType(const Vector3 &pos) const {
        return getCameraMediaType(mWR, pos.x, pos.y, pos.z);
    }

    // ── Sphere collision queries ──

    /// Test a sphere against all solid polygons in a specific cell.
    /// Returns contacts where the sphere penetrates a wall polygon.
    /// The sphere is tested against each solid polygon's plane and polygon area.
    ///
    /// @param center   Sphere center position
    /// @param radius   Sphere radius
    /// @param cellIdx  Cell to test against
    /// @return Vector of contacts (may be empty if no penetration)
    std::vector<SphereContact> sphereVsCellPolygons(
        const Vector3 &center, float radius, int32_t cellIdx) const;

    /// Test a sphere against solid polygons in the containing cell AND
    /// adjacent cells reachable via portals. This handles the case where
    /// the sphere is near a portal boundary and should collide with walls
    /// in the adjacent cell too.
    ///
    /// @param center   Sphere center position
    /// @param radius   Sphere radius
    /// @return Vector of all contacts from the containing cell and neighbors
    std::vector<SphereContact> sphereVsWorld(
        const Vector3 &center, float radius) const;

    /// Apply constraint-based collision resolution to a sphere.
    /// Iteratively projects the sphere position out of all penetrating
    /// polygons (3-iteration constraint projection).
    ///
    /// @param center    IN/OUT: sphere center, modified to resolved position
    /// @param radius    Sphere radius
    /// @param cellIdx   IN/OUT: cell index, updated if sphere crosses a portal
    /// @param contacts  OUT: filled with contacts from all iterations
    /// @param maxIters  Number of constraint projection iterations (default 3)
    void resolveCollisions(
        Vector3 &center, float radius,
        int32_t &cellIdx,
        std::vector<SphereContact> &contacts,
        int maxIters = 3) const;

    /// Quick ground test — cast a sphere downward by a small amount to
    /// check if there's a walkable surface below.
    ///
    /// @param center    Sphere center position
    /// @param radius    Sphere radius
    /// @param cellIdx   Cell to start from
    /// @param maxDrop   Maximum distance to check below (default 0.5 units)
    /// @param outNormal OUT: ground surface normal if hit
    /// @return true if ground was found within maxDrop distance
    bool groundTest(const Vector3 &center, float radius, int32_t cellIdx,
                    float maxDrop, Vector3 &outNormal) const;

    /// Access the underlying WR data (for raycast delegation, etc.)
    const WRParsedData &getWR() const { return mWR; }

private:
    const WRParsedData &mWR;
};

} // namespace Darkness

#endif // __COLLISIONGEOMETRY_H
