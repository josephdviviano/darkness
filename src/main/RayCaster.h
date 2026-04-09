/******************************************************************************
 *
 *    This file is part of the Darkness engine
 *
 *    RayCaster — ray-vs-world-geometry intersection via portal graph traversal.
 *    Traces a ray through WR cells using parametric plane-exit traversal,
 *    matching the original Dark Engine's approach. Finds the closest exit
 *    plane in each cell, determines if it's a portal or solid surface, and
 *    follows portals to adjacent cells sequentially.
 *
 *    Typical Thief 2 missions have ~200-1500 cells; a single ray touches
 *    only a handful via portal traversal, making this very efficient
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

// Epsilon constants matching the original Dark Engine raycaster.
// RAYCAST_EPSILON is the base spatial tolerance; the extended version
// (divided by 3) accounts for the 3× ray extension used for backface
// culling. Time epsilon scales inversely with ray length so that
// short rays (e.g. stair step probes in thin riser cells) use a
// proportionally larger tolerance, preventing misses.
static constexpr double RAYCAST_EPSILON            = 0.001;
static constexpr double EXTENDED_RAYCAST_EPSILON   = RAYCAST_EPSILON / 3.0;

// ── Parametric ray-vs-world traversal ──
// Traces a ray from 'from' to 'to' through the WR cell graph using
// parametric plane-exit traversal.  Matches the original Dark Engine's
// PortalRaycast: extends the ray backward for first-cell backface
// culling, finds the closest exit plane parametrically, then tests
// whether the exit is through a portal (advance to next cell) or a
// solid surface (hit).
//
// hit.hitEntity = 0 for world geometry hits.
// hit.textureIndex = WR texture list index of the hit polygon (-1 if untextured).

/// Overload that also outputs the terminal cell — the cell where the hit
/// occurred, or the last cell traversed if no hit. Enables cell hint
/// propagation between chained raycasts (e.g. the 3-phase stair step check).
/// Pass outTerminalCell = nullptr to ignore.
inline bool raycastWorld(const WRParsedData &wr,
                          const Vector3 &from,
                          const Vector3 &to,
                          RayHit &hit,
                          int32_t *outTerminalCell,
                          int32_t startCellHint = -1);

/// Standard overload — no cell output, no cell hint.
inline bool raycastWorld(const WRParsedData &wr,
                          const Vector3 &from,
                          const Vector3 &to,
                          RayHit &hit) {
    return raycastWorld(wr, from, to, hit, nullptr, -1);
}

inline bool raycastWorld(const WRParsedData &wr,
                          const Vector3 &from,
                          const Vector3 &to,
                          RayHit &hit,
                          int32_t *outTerminalCell,
                          int32_t startCellHint) {
    // ── Ray setup ──
    Vector3 delta = to - from;
    float rayLength = glm::length(delta);
    if (rayLength < 1e-7f) {
        if (outTerminalCell) *outTerminalCell = -1;
        return false;
    }

    // Find starting cell
    int32_t curCell = (startCellHint >= 0 && startCellHint < static_cast<int32_t>(wr.numCells))
                    ? startCellHint
                    : findCameraCell(wr, from.x, from.y, from.z);
    if (curCell < 0) {
        if (outTerminalCell) *outTerminalCell = -1;
        return false;
    }

    // ── Parametric epsilon setup ──
    // Space epsilon is a fixed spatial tolerance. Time epsilon scales
    // inversely with ray length so short rays get proportionally more
    // tolerance (critical for thin riser cells in stair geometry).
    double spaceEpsilon = EXTENDED_RAYCAST_EPSILON;
    double timeEpsilon  = spaceEpsilon / static_cast<double>(rayLength);
    double endTime      = 1.0 - timeEpsilon;

    // ── Extended ray for backface culling ──
    // Instead of finding times from the start point, extend backward by 2×
    // the ray length.  This lets us backface-cull planes in the starting cell
    // that are behind the ray origin.
    //
    // Parametric space:  T=0 → earlyPoint,  T=2/3 → from,  T=1.0 → to.
    // ray_vector = delta * 3.0 (scaled so T=0..1 spans the extended range).
    Vector3 earlyPoint = from - delta * 2.0f;
    Vector3 rayVec     = delta * 3.0f;

    // ── Sequential cell traversal ──
    int cellsProcessed = 0;

    while (cellsProcessed < MAX_RAY_CELLS) {
        ++cellsProcessed;

        if (curCell < 0 || curCell >= static_cast<int32_t>(wr.numCells))
            break;

        const auto &cell = wr.cells[curCell];

        // Find the closest exit plane: the plane that is in front of early_point
        // (positive distance = inside cell) but crossed by end_point (negative distance).
        double bestTime = 1e30;
        int bestPlaneIdx = -1;

        for (int pi = 0; pi < static_cast<int>(cell.numPlanes); ++pi) {
            const auto &plane = cell.planes[pi];

            // Signed distance from early_point to this plane (double precision
            // to match original engine's numerical behavior in thin cells).
            double startDist = static_cast<double>(plane.normal.x) * earlyPoint.x
                             + static_cast<double>(plane.normal.y) * earlyPoint.y
                             + static_cast<double>(plane.normal.z) * earlyPoint.z
                             + static_cast<double>(plane.d);

            // Backface cull: early_point must be on the positive (inside) side.
            // Planes facing away from the ray can't be exit planes.
            if (startDist <= spaceEpsilon)
                continue;

            // Signed distance from end_point to this plane.
            double endDist = static_cast<double>(plane.normal.x) * to.x
                           + static_cast<double>(plane.normal.y) * to.y
                           + static_cast<double>(plane.normal.z) * to.z
                           + static_cast<double>(plane.d);

            // End must cross to the negative side (through the plane).
            if (endDist >= 0.0)
                continue;

            // Parametric crossing time in the extended ray's T=[0,1] space.
            double t = startDist / (startDist - endDist);
            if (t < bestTime) {
                bestTime = t;
                bestPlaneIdx = pi;
            }
        }

        // If all exit planes are beyond our endpoint, the ray ends inside
        // this cell without hitting anything.
        if (bestTime > endTime) {
            if (outTerminalCell) *outTerminalCell = curCell;
            return false;
        }

        // No valid exit plane found (degenerate geometry).
        if (bestPlaneIdx < 0) {
            if (outTerminalCell) *outTerminalCell = curCell;
            return false;
        }

        // ── Compute hit point on the exit plane ──
        Vector3 hitPoint = earlyPoint + rayVec * static_cast<float>(bestTime);

        // ── Categorize polygons on this plane ──
        // Original engine first checks if the exit plane has any portals.
        // If no portals → solid hit. If all portals → pass through.
        // Mixed → 2D hull test to determine which polygon was hit.
        int numSolid = cell.numPolygons - cell.numPortals;

        bool planeHasSolid = false;

        // Portal exits on this plane (bounded — typical cells have <10 portals)
        struct PortalExit { int polyIdx; int32_t tgtCell; };
        PortalExit portalExits[64];
        int numPortalExits = 0;

        for (int i = 0; i < numSolid; ++i) {
            if (cell.polygons[i].plane == bestPlaneIdx)
                planeHasSolid = true;
        }

        for (int i = numSolid; i < cell.numPolygons; ++i) {
            if (cell.polygons[i].plane == bestPlaneIdx && numPortalExits < 64) {
                portalExits[numPortalExits].polyIdx = i;
                portalExits[numPortalExits].tgtCell =
                    static_cast<int32_t>(cell.polygons[i].tgtCell);
                numPortalExits++;
            }
        }

        // ── Decision: portal pass-through or solid hit? ──

        // No portals on this plane → solid hit
        if (numPortalExits == 0) {
            // Find which solid polygon was hit (for texture info).
            Vector3 windingNormal = -cell.planes[bestPlaneIdx].normal;
            int hitPolyIdx = -1;
            for (int i = 0; i < numSolid; ++i) {
                if (cell.polygons[i].plane == bestPlaneIdx) {
                    if (pointInConvexPolygon(hitPoint, cell.vertices,
                                              cell.polyIndices[i], windingNormal)) {
                        hitPolyIdx = i;
                        break;
                    }
                }
            }

            hit.point    = hitPoint;
            hit.normal   = cell.planes[bestPlaneIdx].normal;
            hit.distance = glm::length(hitPoint - from);
            hit.hitEntity = 0;
            hit.cellIdx  = curCell;
            hit.polyIdx  = hitPolyIdx;

            if (hitPolyIdx >= 0 && hitPolyIdx < static_cast<int>(cell.numTextured))
                hit.textureIndex = static_cast<int32_t>(cell.texturing[hitPolyIdx].txt);
            else
                hit.textureIndex = -1;

            if (outTerminalCell) *outTerminalCell = curCell;
            return true;
        }

        // Single portal covering the entire plane (no solid polys) → pass through
        if (numPortalExits == 1 && !planeHasSolid) {
            int32_t nextCell = portalExits[0].tgtCell;
            if (nextCell >= 0 && nextCell < static_cast<int32_t>(wr.numCells)) {
                curCell = nextCell;
                continue;
            }
            // Invalid target — treat as solid
            break;
        }

        // ── Mixed plane: 2D hull test to find which polygon was hit ──
        Vector3 windingNormal = -cell.planes[bestPlaneIdx].normal;

        // Test portal polygons first. If we're in a portal, advance.
        // Original skips the last portal if the plane is all-portals
        // (optimization: if not in any tested portal, it must be the last one).
        int portalsToTest = numPortalExits - (planeHasSolid ? 0 : 1);
        int hitPortal = -1;

        for (int i = 0; i < portalsToTest; ++i) {
            int pi = portalExits[i].polyIdx;
            if (pointInConvexPolygon(hitPoint, cell.vertices,
                                      cell.polyIndices[pi], windingNormal)) {
                hitPortal = i;
                break;
            }
        }

        // If not in any tested portal and plane is all-portals, it's the last portal
        if (hitPortal < 0 && !planeHasSolid) {
            hitPortal = numPortalExits - 1;
        }

        if (hitPortal >= 0) {
            // Pass through portal to next cell
            int32_t nextCell = portalExits[hitPortal].tgtCell;
            if (nextCell >= 0 && nextCell < static_cast<int32_t>(wr.numCells)) {
                curCell = nextCell;
                continue;
            }
            break;
        }

        // Not in any portal → hit a solid surface on this plane.
        // Find which solid polygon for texture info.
        int hitPolyIdx = -1;
        for (int i = 0; i < numSolid; ++i) {
            if (cell.polygons[i].plane == bestPlaneIdx) {
                if (pointInConvexPolygon(hitPoint, cell.vertices,
                                          cell.polyIndices[i], windingNormal)) {
                    hitPolyIdx = i;
                    break;
                }
            }
        }

        hit.point    = hitPoint;
        hit.normal   = cell.planes[bestPlaneIdx].normal;
        hit.distance = glm::length(hitPoint - from);
        hit.hitEntity = 0;
        hit.cellIdx  = curCell;
        hit.polyIdx  = hitPolyIdx;

        if (hitPolyIdx >= 0 && hitPolyIdx < static_cast<int>(cell.numTextured))
            hit.textureIndex = static_cast<int32_t>(cell.texturing[hitPolyIdx].txt);
        else
            hit.textureIndex = -1;

        if (outTerminalCell) *outTerminalCell = curCell;
        return true;
    }

    // Max cells exceeded or invalid cell — no hit
    if (outTerminalCell) *outTerminalCell = curCell;
    return false;
}

} // namespace Darkness
