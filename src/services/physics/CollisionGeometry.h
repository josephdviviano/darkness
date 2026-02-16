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
 *    world geometry data. Header-only (inline), matching the pattern of
 *    CellGeometry.h and RayCaster.h.
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
 *    2. If dist < radius, project sphere center onto plane and test
 *       point-in-polygon
 *    3. Accumulate contact normals, then remove velocity components
 *       along contacts
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

#include <algorithm>
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
    uint8_t age = 0;       // frames since last detection (0 = just detected this frame)
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
    ///
    /// Algorithm (per polygon):
    ///   1. Compute signed distance from sphere center to polygon plane
    ///   2. If dist >= radius, sphere doesn't reach this plane — skip
    ///   3. Project sphere center onto the plane
    ///   4. Test if projected point is inside the polygon (winding test)
    ///   5. If inside, record contact with penetration = radius - dist
    ///
    /// Cell planes face inward, so the plane normal points INTO the cell
    /// (toward the interior). A positive distance means the sphere center
    /// is inside the cell; negative means past the wall. We push whenever
    /// dist < radius.
    /// Append contacts to outContacts (caller manages clear).
    inline void sphereVsCellPolygons(
        const Vector3 &center, float radius, int32_t cellIdx,
        std::vector<SphereContact> &outContacts) const
    {
        if (cellIdx < 0 || cellIdx >= static_cast<int32_t>(mWR.numCells))
            return;

        const auto &cell = mWR.cells[cellIdx];
        int numSolid = cell.numPolygons - cell.numPortals;

        // Clear edge dedup buffer for this cell test pass
        mVisitedEdges.clear();

        for (int pi = 0; pi < numSolid; ++pi) {
            const auto &poly = cell.polygons[pi];
            if (poly.count < 3)
                continue;

            const auto &plane = cell.planes[poly.plane];
            float dist = plane.getDistance(center);

            // Sphere doesn't reach this plane — no contact.
            // For point detectors (radius=0), contact requires dist < 0
            // (the point has crossed the plane surface).
            if (dist >= radius)
                continue;

            // Spurious contact rejection — the center is too far past the wall.
            // For real spheres (radius > 0), the sphere can only legitimately
            // contact a wall when its center is within one radius of the plane.
            // For point detectors (radius ~= 0), use a fixed max penetration
            // threshold to reject contacts from adjacent cell walls that are
            // far behind the point.
            constexpr float POINT_MAX_PENETRATION = 0.5f;
            float maxBehind = (radius > 0.001f) ? radius : POINT_MAX_PENETRATION;
            if (dist < -maxBehind)
                continue;

            // Project sphere center onto the polygon's plane to check if
            // the penetration is through this polygon's surface area.
            // Use negated normal for the winding test: WR polygon vertices
            // are CCW when viewed from outside the cell (outward-facing for
            // rendering), but cell plane normals face inward. Negate so
            // the winding test matches the vertex order.
            Vector3 projected = center - plane.normal * dist;
            Vector3 windNormal(-plane.normal.x, -plane.normal.y, -plane.normal.z);

            if (pointInConvexPolygon(projected, cell.vertices,
                                      cell.polyIndices[pi], windNormal)) {
                // Face contact: sphere center projects inside the polygon face.
                // Normal is the inward-facing plane normal (push direction).
                SphereContact contact;
                contact.normal = plane.normal;
                contact.penetration = radius - dist;
                contact.cellIdx = cellIdx;
                contact.polyIdx = pi;

                if (pi < static_cast<int>(cell.numTextured)) {
                    contact.textureIdx = static_cast<int32_t>(cell.texturing[pi].txt);
                } else {
                    contact.textureIdx = -1;
                }

                outContacts.push_back(contact);
            } else if (radius > 0.001f) {
                // Edge fallback: sphere center projects outside the polygon face,
                // but may still intersect a polygon edge. Only test for real spheres
                // (HEAD/BODY), not point detectors (SHIN/KNEE/FOOT).
                sphereVsPolygonEdges(center, radius, cell, pi, cellIdx, outContacts);
            }
        }
    }

    /// Test a sphere against solid polygons in the containing cell AND
    /// adjacent cells reachable via portals. This handles the case where
    /// the sphere is near a portal boundary and should collide with walls
    /// in the adjacent cell too.
    ///
    /// Portal adjacency check: for each portal polygon in the containing
    /// cell, if the sphere center is within radius of the portal plane,
    /// also test the target cell's solid polygons.
    inline std::vector<SphereContact> sphereVsWorld(
        const Vector3 &center, float radius) const
    {
        int32_t cellIdx = findCell(center);
        if (cellIdx < 0)
            return {};

        // Test the containing cell first
        std::vector<SphereContact> contacts;
        sphereVsCellPolygons(center, radius, cellIdx, contacts);

        // Check portal polygons to find adjacent cells where the sphere
        // might also be colliding. The sphere reaches into an adjacent
        // cell when its distance to the portal plane is less than its radius.
        const auto &cell = mWR.cells[cellIdx];
        int numSolid = cell.numPolygons - cell.numPortals;

        for (int pi = numSolid; pi < cell.numPolygons; ++pi) {
            const auto &poly = cell.polygons[pi];
            if (poly.count < 3)
                continue;

            const auto &plane = cell.planes[poly.plane];
            float dist = plane.getDistance(center);

            // Sphere extends past this portal plane — check adjacent cell
            if (dist < radius) {
                int32_t tgtCell = static_cast<int32_t>(poly.tgtCell);
                if (tgtCell >= 0 && tgtCell < static_cast<int32_t>(mWR.numCells)) {
                    sphereVsCellPolygons(center, radius, tgtCell, contacts);
                }
            }
        }

        return contacts;
    }

    /// Apply constraint-based collision resolution to a sphere.
    /// Iteratively projects the sphere position out of all penetrating
    /// polygons (3-iteration constraint projection).
    ///
    /// Each iteration:
    ///   1. Find all sphere-polygon contacts at current position
    ///   2. For each contact, push sphere along contact normal by penetration
    ///   3. Re-find cell (may have crossed portal during push)
    ///   4. Repeat until no contacts or max iterations reached
    ///
    /// This handles corners (2 intersecting walls) and edges (3+ walls)
    /// by converging through multiple push iterations — the same approach
    /// as the renderer's camera collision.
    inline void resolveCollisions(
        Vector3 &center, float radius,
        int32_t &cellIdx,
        std::vector<SphereContact> &contacts,
        int maxIters = 3) const
    {
        // Save original position for revert on failure
        Vector3 origCenter = center;
        int32_t origCell = cellIdx;

        for (int iter = 0; iter < maxIters; ++iter) {
            if (cellIdx < 0)
                break;

            std::vector<SphereContact> iterContacts;
            sphereVsCellPolygons(center, radius, cellIdx, iterContacts);

            // Also check adjacent cells via portals
            const auto &cell = mWR.cells[cellIdx];
            int numSolid = cell.numPolygons - cell.numPortals;
            for (int pi = numSolid; pi < cell.numPolygons; ++pi) {
                const auto &poly = cell.polygons[pi];
                if (poly.count < 3)
                    continue;
                const auto &plane = cell.planes[poly.plane];
                float dist = plane.getDistance(center);
                if (dist < radius) {
                    int32_t tgtCell = static_cast<int32_t>(poly.tgtCell);
                    if (tgtCell >= 0 && tgtCell < static_cast<int32_t>(mWR.numCells)) {
                        sphereVsCellPolygons(center, radius, tgtCell, iterContacts);
                    }
                }
            }

            if (iterContacts.empty())
                break; // No contacts — done

            // Push sphere out of each penetrating polygon
            for (const auto &c : iterContacts) {
                center += c.normal * c.penetration;
            }

            // Record contacts for caller (ground detection, contact events)
            contacts.insert(contacts.end(),
                            iterContacts.begin(), iterContacts.end());

            // Push may have moved sphere through a portal — update cell
            int32_t newCell = findCell(center);
            if (newCell >= 0) {
                cellIdx = newCell;
            } else {
                // Pushed outside all cells — revert to original position
                center = origCenter;
                cellIdx = origCell;
                return;
            }
        }

        // Final validation — sphere must still be in a valid cell
        if (findCell(center) < 0) {
            center = origCenter;
            cellIdx = origCell;
        }
    }

    /// Quick ground test — probe along a given direction to check for
    /// a walkable surface nearby.
    ///
    /// The probe direction should be a unit vector pointing "into" the
    /// expected ground surface. For flat ground this is (0,0,-1); for
    /// slopes it follows the last known ground normal's inverse.
    /// Ground probes follow the contact normal direction rather than
    /// always going straight down, improving reliability on steep slopes.
    ///
    /// Returns the most upward-facing contact normal and its texture
    /// index. scratchContacts is a caller-provided buffer (cleared
    /// internally) to avoid per-call heap allocation on the hot path.
    inline bool groundTest(const Vector3 &center, float radius, int32_t cellIdx,
                           float maxDrop, const Vector3 &probeDir,
                           Vector3 &outNormal, int32_t &outTextureIdx,
                           std::vector<SphereContact> &scratchContacts) const
    {
        if (cellIdx < 0 || cellIdx >= static_cast<int32_t>(mWR.numCells))
            return false;

        // Probe position: move center along probeDir by maxDrop
        Vector3 testPos = center + probeDir * maxDrop;

        // Find the cell at the test position (may differ from current cell
        // if we're near a ledge or stairway)
        int32_t testCell = findCell(testPos);
        if (testCell < 0)
            testCell = cellIdx; // fallback to current cell

        scratchContacts.clear();
        sphereVsCellPolygons(testPos, radius, testCell, scratchContacts);

        // Also check adjacent cells at the probed position
        if (testCell >= 0 && testCell < static_cast<int32_t>(mWR.numCells)) {
            const auto &cell = mWR.cells[testCell];
            int numSolid = cell.numPolygons - cell.numPortals;
            for (int pi = numSolid; pi < cell.numPolygons; ++pi) {
                const auto &poly = cell.polygons[pi];
                if (poly.count < 3)
                    continue;
                const auto &plane = cell.planes[poly.plane];
                float dist = plane.getDistance(testPos);
                if (dist < radius) {
                    int32_t tgtCell = static_cast<int32_t>(poly.tgtCell);
                    if (tgtCell >= 0 && tgtCell < static_cast<int32_t>(mWR.numCells)) {
                        sphereVsCellPolygons(testPos, radius, tgtCell, scratchContacts);
                    }
                }
            }
        }

        // Find the most upward-facing contact normal (ground = normal.z > 0)
        bool found = false;
        float bestZ = -1.0f;
        outTextureIdx = -1;
        for (const auto &c : scratchContacts) {
            // Ground normals point upward (positive Z in Z-up coordinate system)
            if (c.normal.z > 0.0f && c.normal.z > bestZ) {
                bestZ = c.normal.z;
                outNormal = c.normal;
                outTextureIdx = c.textureIdx;
                found = true;
            }
        }

        return found;
    }

    /// Access the underlying WR data (for raycast delegation, etc.)
    const WRParsedData &getWR() const { return mWR; }

private:
    const WRParsedData &mWR;

    // Scratch buffer for edge contact de-duplication within a single cell test.
    // Keyed on ordered vertex index pairs packed into uint16_t.
    // Mutable because sphereVsCellPolygons() is const but needs scratch space.
    mutable std::vector<uint16_t> mVisitedEdges;

    /// Fallback edge contact test — called when sphere is within radius of a polygon's
    /// plane but its center projects outside the polygon face. Tests each edge of the
    /// polygon for sphere-edge intersection using closest-point-on-segment.
    ///
    /// Contact normal points from the closest edge point toward the sphere center
    /// (radial push direction). This produces correct sliding behavior along wall
    /// edges and around convex corners.
    ///
    /// Shared edges between adjacent polygons are de-duplicated via mVisitedEdges
    /// (keyed on ordered vertex index pair) to prevent double-push.
    inline void sphereVsPolygonEdges(
        const Vector3 &center, float radius,
        const WRParsedCell &cell, int pi, int32_t cellIdx,
        std::vector<SphereContact> &outContacts) const
    {
        const auto &indices = cell.polyIndices[pi];
        int n = static_cast<int>(indices.size());
        if (n < 3) return;

        int32_t textureIdx = -1;
        if (pi < static_cast<int>(cell.numTextured))
            textureIdx = static_cast<int32_t>(cell.texturing[pi].txt);

        for (int i = 0; i < n; ++i) {
            uint8_t idxA = indices[i];
            uint8_t idxB = indices[(i + 1) % n];

            // De-duplicate shared edges — adjacent polygons in the same cell
            // share edges. Ordered pair key: min in high byte, max in low byte.
            uint16_t edgeKey = (static_cast<uint16_t>(std::min(idxA, idxB)) << 8)
                             | static_cast<uint16_t>(std::max(idxA, idxB));
            bool seen = false;
            for (auto k : mVisitedEdges) {
                if (k == edgeKey) { seen = true; break; }
            }
            if (seen) continue;
            mVisitedEdges.push_back(edgeKey);

            const Vector3 &a = cell.vertices[idxA];
            const Vector3 &b = cell.vertices[idxB];

            // Closest point on segment a→b to sphere center
            // Standard parametric clamping: t = dot(P-A, B-A) / |B-A|², clamped [0,1]
            // (same algorithm as Godot's geometry_3d.h::get_closest_point_to_segment)
            Vector3 edge = b - a;
            float edgeLenSq = glm::dot(edge, edge);
            if (edgeLenSq < 1e-12f) continue; // degenerate edge

            float t = glm::dot(center - a, edge) / edgeLenSq;
            t = std::clamp(t, 0.0f, 1.0f);
            Vector3 closest = a + edge * t;

            Vector3 diff = center - closest;
            float distSq = glm::dot(diff, diff);

            if (distSq < radius * radius && distSq > 1e-12f) {
                float dist = std::sqrt(distSq);
                SphereContact contact;
                contact.normal = diff / dist;
                contact.penetration = radius - dist;
                contact.cellIdx = cellIdx;
                contact.polyIdx = pi;
                contact.textureIdx = textureIdx;
                outContacts.push_back(contact);
            }
        }
    }
};

} // namespace Darkness

#endif // __COLLISIONGEOMETRY_H
