/******************************************************************************
 *
 *    This file is part of the Darkness engine
 *
 *    RayCaster — ray-vs-world-geometry intersection via portal graph traversal.
 *    Traces a ray through WR cells by testing solid polygons for intersection
 *    and following portal polygons to adjacent cells. Finds the closest hit
 *    along the ray segment [from, to].
 *
 *    Typical Thief 2 missions have ~200-1500 cells; a single ray touches
 *    only a handful via portal graph traversal, making this very efficient
 *    for AI line-of-sight, sound occlusion, and physics ray checks.
 *
 *    Depends on CellGeometry.h (bgfx-free). No bgfx dependency.
 *
 *****************************************************************************/

#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include "CellGeometry.h"
#include "worldquery/WorldQueryTypes.h"

namespace Darkness {

// Max cells the ray will traverse before giving up (safety bound)
static constexpr int   MAX_RAY_CELLS = 64;
// Threshold for treating a ray as parallel to a plane
static constexpr float RAY_EPSILON   = 1e-6f;

// ── Ray-polygon intersection ──
// Tests whether a ray (origin + t*dir) intersects a specific polygon in a cell.
// Returns true if the ray hits the polygon, filling outT and outPoint.
// outNormal is set to the surface normal facing the ray origin (= cell plane inward
// normal, which points toward the interior where the ray originates).

inline bool rayIntersectPolygon(const Vector3 &origin,
                                 const Vector3 &dir,
                                 const WRParsedCell &cell,
                                 int polyIdx,
                                 float &outT,
                                 Vector3 &outPoint,
                                 Vector3 &outNormal) {
    const auto &poly = cell.polygons[polyIdx];
    const auto &plane = cell.planes[poly.plane];

    // Ray-plane intersection: t = -(plane.normal · origin + plane.d) / (plane.normal · dir)
    float denom = plane.normal.dotProduct(dir);

    // If ray is nearly parallel to the plane, no intersection
    if (std::fabs(denom) < RAY_EPSILON)
        return false;

    float t = -plane.getDistance(origin) / denom;

    // Hit must be in the forward direction along the ray
    if (t < 0.0f)
        return false;

    // Compute hit point
    Vector3 hitPoint(
        origin.x + dir.x * t,
        origin.y + dir.y * t,
        origin.z + dir.z * t
    );

    // Polygon vertex winding normal = -plane.normal (outward from cell, since cell
    // planes face inward). Used only for the winding test, not the returned normal.
    Vector3 windingNormal(
        -plane.normal.x,
        -plane.normal.y,
        -plane.normal.z
    );

    // Test if the hit point lies inside the convex polygon
    if (!pointInConvexPolygon(hitPoint, cell.vertices, cell.polyIndices[polyIdx], windingNormal))
        return false;

    outT = t;
    outPoint = hitPoint;
    // Surface normal facing the ray: cell planes face inward (toward the ray origin),
    // so plane.normal is the correct surface normal for hits from inside the cell.
    outNormal = plane.normal;
    return true;
}

// ── Ray-vs-world traversal ──
// Traces a ray from 'from' to 'to' through the WR cell graph.
// Returns true if the ray hits solid geometry, filling 'hit' with the closest
// intersection. Uses BFS portal traversal: tests solid polygons for hits,
// follows portal polygons to adjacent cells.
//
// hit.hitEntity = 0 for world geometry hits.
// hit.textureIndex = WR texture list index of the hit polygon (-1 if untextured).

inline bool raycastWorld(const WRParsedData &wr,
                          const Vector3 &from,
                          const Vector3 &to,
                          RayHit &hit) {
    // Compute ray direction and length
    Vector3 delta(to.x - from.x, to.y - from.y, to.z - from.z);
    float maxDist = std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);

    // Degenerate ray (zero length)
    if (maxDist < RAY_EPSILON)
        return false;

    // Normalized direction
    float invDist = 1.0f / maxDist;
    Vector3 dir(delta.x * invDist, delta.y * invDist, delta.z * invDist);

    // Find the starting cell
    int32_t startCell = findCameraCell(wr, from.x, from.y, from.z);
    if (startCell < 0)
        return false;

    // BFS traversal state
    // Using a simple array-based queue and visited bitset for efficiency
    std::vector<int32_t> queue;
    queue.reserve(32);
    queue.push_back(startCell);

    std::vector<bool> visited(wr.numCells, false);
    visited[startCell] = true;

    bool foundHit = false;
    float bestT = maxDist; // Only accept hits within the ray segment [from, to]

    size_t queueHead = 0;
    int cellsProcessed = 0;

    while (queueHead < queue.size() && cellsProcessed < MAX_RAY_CELLS) {
        int32_t cellIdx = queue[queueHead++];
        ++cellsProcessed;

        const auto &cell = wr.cells[cellIdx];
        int numSolid = cell.numPolygons - cell.numPortals;

        // Test solid polygons for intersection
        for (int pi = 0; pi < numSolid; ++pi) {
            float t;
            Vector3 hitPoint, hitNormal;

            if (rayIntersectPolygon(from, dir, cell, pi, t, hitPoint, hitNormal)) {
                if (t <= bestT) {
                    bestT = t;
                    hit.point = hitPoint;
                    hit.normal = hitNormal;
                    hit.distance = t;
                    hit.hitEntity = 0; // world geometry

                    // Populate texture index if this polygon has texturing data
                    if (pi < static_cast<int>(cell.numTextured)) {
                        hit.textureIndex = static_cast<int32_t>(cell.texturing[pi].txt);
                    } else {
                        hit.textureIndex = -1;
                    }

                    foundHit = true;
                }
            }
        }

        // Test portal polygons — if the ray passes through a portal,
        // enqueue the target cell for further testing
        for (int pi = numSolid; pi < cell.numPolygons; ++pi) {
            float t;
            Vector3 hitPoint, hitNormal;

            if (rayIntersectPolygon(from, dir, cell, pi, t, hitPoint, hitNormal)) {
                // Only follow portals that are closer than the best solid hit
                if (t <= bestT) {
                    int32_t tgtCell = static_cast<int32_t>(cell.polygons[pi].tgtCell);
                    if (tgtCell >= 0 && tgtCell < static_cast<int32_t>(wr.numCells)
                        && !visited[tgtCell]) {
                        visited[tgtCell] = true;
                        queue.push_back(tgtCell);
                    }
                }
            }
        }
    }

    return foundHit;
}

} // namespace Darkness
