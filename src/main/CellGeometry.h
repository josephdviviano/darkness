/******************************************************************************
 *
 *    This file is part of the Darkness engine
 *
 *    CellGeometry — bgfx-free geometry utilities for WR cell queries.
 *    Extracted from DarknessRendererCore.h so that non-rendering code
 *    (raycaster, tests) can use cell queries without linking bgfx.
 *
 *    Depends only on WRChunkParser.h (DarknessBase primitives).
 *
 *****************************************************************************/

#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include "WRChunkParser.h"

namespace Darkness {

// ── Camera cell detection ──
// Find which cell the camera is in. Returns cell index, or -1 if outside
// all cells. When the camera is near a water boundary, it may be inside
// both an air and water cell — we prefer the smallest containing cell.

inline int32_t findCameraCell(const WRParsedData &wr,
                               float cx, float cy, float cz) {
    Vector3 pt(cx, cy, cz);
    float bestRadius = 1e30f;
    int32_t bestCell = -1;

    for (uint32_t i = 0; i < wr.numCells; ++i) {
        const auto &cell = wr.cells[i];

        // Quick reject: skip cells whose bounding sphere doesn't contain the point
        float dx = cx - cell.center.x;
        float dy = cy - cell.center.y;
        float dz = cz - cell.center.z;
        float dist2 = dx*dx + dy*dy + dz*dz;
        if (dist2 > cell.radius * cell.radius)
            continue;

        // Precise test: point must be on the positive side of all cell planes.
        // Dark Engine convention: cell planes face inward, so inside = positive
        // distance (normal · point + d > 0). Small negative epsilon for boundary.
        bool inside = true;
        for (const auto &plane : cell.planes) {
            if (plane.getDistance(pt) < -0.1f) {
                inside = false;
                break;
            }
        }

        if (inside) {
            // Prefer the smallest containing cell — water cells are typically
            // smaller than the adjacent air cells they border.
            if (cell.radius < bestRadius) {
                bestRadius = cell.radius;
                bestCell = static_cast<int32_t>(i);
            }
        }
    }

    return bestCell;
}

// Convenience wrapper: returns mediaType (1=air, 2=water) for underwater detection
inline uint8_t getCameraMediaType(const WRParsedData &wr,
                                   float cx, float cy, float cz) {
    int32_t cell = findCameraCell(wr, cx, cy, cz);
    if (cell >= 0 && cell < static_cast<int32_t>(wr.numCells))
        return wr.cells[cell].mediaType;
    return 1; // default: air
}

// Test if a point (assumed to lie on the polygon's plane) is inside a convex polygon.
// Uses the winding/cross-product method: the point must be on the interior side of
// every edge when traversed in order, relative to the polygon's plane normal.
inline bool pointInConvexPolygon(const Vector3 &p,
                                 const std::vector<Vector3> &verts,
                                 const std::vector<uint8_t> &indices,
                                 const Vector3 &normal) {
    int n = static_cast<int>(indices.size());
    if (n < 3) return false;

    for (int i = 0; i < n; ++i) {
        const auto &a = verts[indices[i]];
        const auto &b = verts[indices[(i + 1) % n]];
        Vector3 edge = b - a;
        Vector3 toP  = p - a;
        Vector3 cross = edge.crossProduct(toP);
        if (cross.dotProduct(normal) < 0.0f)
            return false;
    }
    return true;
}

} // namespace Darkness
