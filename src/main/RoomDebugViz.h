// ── RoomDebugViz.h ────────────────────────────────────────────────────────
//
// CPU-side geometry helpers for the room/portal wireframe debug overlay.
//
// Computes the polytope vertices of each Room (intersection of its 6 bounding
// half-planes) and the polygon vertices of each Portal (intersection of the
// portal plane with the per-portal edge half-planes). Output is consumed by
// the renderer to build a single bgfx LINE-list vertex buffer.
//
// No bgfx dependency: this header is pure data.
//
// (c) Darkness contributors — GPLv3.
// ──────────────────────────────────────────────────────────────────────────

#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

#include "DarknessMath.h"
#include "room/Room.h"
#include "room/RoomPortal.h"

namespace Darkness {

// Solves the linear system { n_i · p + d_i = 0 } for three planes.
// Returns true and writes the intersection point into `out` if the
// planes are non-degenerate (their normal triple has non-zero determinant).
// Uses Cramer's rule with a tolerance suitable for engine-unit coordinates.
inline bool intersectThreePlanes(const Plane &a, const Plane &b,
                                  const Plane &c, Vector3 &out)
{
    // The plane equation in our convention is `dot(n, p) + d = 0`, so
    // `dot(n, p) = -d` is the right-hand side of the linear system.
    Vector3 bc = glm::cross(b.normal, c.normal);
    float det = glm::dot(a.normal, bc);
    if (std::fabs(det) < 1e-6f)
        return false;

    Vector3 ca = glm::cross(c.normal, a.normal);
    Vector3 ab = glm::cross(a.normal, b.normal);

    out = (-a.d * bc + -b.d * ca + -c.d * ab) / det;
    return true;
}

// One corner of a room's bounding polytope. The `planes` array records which
// 3 of the 6 room planes meet at this corner; we use it later to walk edges
// (an edge of the polytope is shared by exactly two corners that lie on the
// same two planes).
struct RoomCorner {
    Vector3 pos;
    std::array<uint8_t, 3> planes;  // indices into Room::mPlanes (0..5)
};

// Compute all corners of a Room's bounding polytope.
//
// Enumerates every triple of the 6 planes (C(6,3) = 20), solves for the
// intersection point, and keeps the point only if it lies on the inside
// half-space of every other plane (i.e. it really is a vertex of the
// convex polytope, not just a triple-plane intersection in space).
//
// For a typical axis-aligned Room this returns exactly 8 corners. Skewed
// rooms may produce fewer if the polytope is degenerate.
inline std::vector<RoomCorner> computeRoomCorners(const Plane planes[6])
{
    std::vector<RoomCorner> corners;
    corners.reserve(8);

    // Slightly negative tolerance so points lying ON a plane (distance ≈ 0)
    // are accepted as "inside" the half-space of that plane. Engine-unit
    // numerical noise is well within this band.
    constexpr float kInsideTol = 1e-3f;

    for (int i = 0; i < 6; ++i) {
        for (int j = i + 1; j < 6; ++j) {
            for (int k = j + 1; k < 6; ++k) {
                Vector3 p;
                if (!intersectThreePlanes(planes[i], planes[j], planes[k], p))
                    continue;

                bool inside = true;
                for (int m = 0; m < 6; ++m) {
                    if (m == i || m == j || m == k) continue;
                    // Plane::getDistance(p) = dot(normal, p) + d. Room
                    // bounding planes are stored INWARD-facing (Room::read
                    // negates both normal and d on load), so "inside the
                    // half-space" means getDistance ≥ 0 and "outside"
                    // means getDistance < 0. Reject points that fall
                    // strictly outside any non-defining plane.
                    if (planes[m].getDistance(p) < -kInsideTol) {
                        inside = false;
                        break;
                    }
                }
                if (!inside) continue;

                RoomCorner rc;
                rc.pos = p;
                rc.planes[0] = static_cast<uint8_t>(i);
                rc.planes[1] = static_cast<uint8_t>(j);
                rc.planes[2] = static_cast<uint8_t>(k);
                corners.push_back(rc);
            }
        }
    }
    return corners;
}

// Two RoomCorner indices that define one edge of the polytope.
struct CornerEdge {
    uint16_t a, b;
};

// Build the edge list for a polytope from its corner set.
// Two corners share an edge iff they lie on the same 2 of the 6 planes
// (the edge IS the intersection line of those 2 planes, restricted to the
// polytope). For non-degenerate convex polytopes each plane pair yields at
// most 2 corners that share it — which gives the unique edge.
inline std::vector<CornerEdge> computeRoomEdges(const std::vector<RoomCorner> &corners)
{
    std::vector<CornerEdge> edges;
    edges.reserve(12);

    auto sharedPlanes = [](const RoomCorner &x, const RoomCorner &y) -> int {
        int hits = 0;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                if (x.planes[i] == y.planes[j]) ++hits;
        return hits;
    };

    for (uint16_t i = 0; i < corners.size(); ++i) {
        for (uint16_t j = i + 1; j < corners.size(); ++j) {
            if (sharedPlanes(corners[i], corners[j]) >= 2)
                edges.push_back({i, j});
        }
    }
    return edges;
}

// Compute polygon vertices of a Portal in world space.
//
// A portal has one defining plane and `mEdgeCount` edge half-planes
// (each constraining the polygon to lie inside the edge). The polygon
// vertices are points where the portal plane meets two adjacent edge
// planes, filtered to the inside half of every other edge.
//
// Returns vertices in walked order around the portal so the caller can
// emit them as a closed line strip. Falls back to angular sort around
// the portal center if intrinsic walking ever produces a degenerate
// ordering — robust against minor numerical noise.
inline std::vector<Vector3> computePortalPolygon(const RoomPortal &portal)
{
    const Plane &portalPlane = portal.getPlane();
    uint32_t ec = portal.getEdgeCount();
    if (ec < 3) return {};  // degenerate; not a polygon

    std::vector<Vector3> verts;
    verts.reserve(ec);

    constexpr float kInsideTol = 1e-3f;

    // For each pair of edges, find the point on the portal plane that lies
    // on both edges. Filter by inside-ness of all other edges.
    for (uint32_t i = 0; i < ec; ++i) {
        for (uint32_t j = i + 1; j < ec; ++j) {
            Vector3 p;
            if (!intersectThreePlanes(portalPlane,
                                       portal.getEdgePlane(i),
                                       portal.getEdgePlane(j), p))
                continue;

            bool inside = true;
            for (uint32_t k = 0; k < ec; ++k) {
                if (k == i || k == j) continue;
                // Portal edge planes are stored verbatim from disk with
                // OUTWARD-facing normals (RoomPortal::read documents the
                // convention). Inside the polygon means getDistance ≤ 0
                // for every edge; reject points that fall strictly
                // outside (positive distance beyond tolerance).
                //
                // History: this used to test `< -kInsideTol` like
                // computeRoomCorners above, but room bounding planes are
                // INWARD-facing (negated on read) while edge planes are
                // not — so the room test and the portal test must use
                // opposite polarity. The old test rejected every real
                // polygon vertex for any rectangular portal (opposite
                // edge gives dist ≈ -width), leaving the portal overlay
                // empty.
                if (portal.getEdgePlane(k).getDistance(p) > kInsideTol) {
                    inside = false;
                    break;
                }
            }
            if (inside) verts.push_back(p);
        }
    }

    if (verts.size() < 3) return verts;

    // Sort vertices in angular order around the portal center, projected
    // onto the portal plane. This produces a stable winding for edge
    // rendering even when the corner-discovery loop returned them in an
    // arbitrary order.
    Vector3 center = portal.getCenter();

    // Build a 2D basis on the portal plane.
    Vector3 n = portalPlane.normal;
    Vector3 helper = std::fabs(n.x) > 0.9f ? Vector3(0, 1, 0) : Vector3(1, 0, 0);
    Vector3 u = glm::normalize(glm::cross(helper, n));
    Vector3 v = glm::cross(n, u);

    std::sort(verts.begin(), verts.end(),
        [&](const Vector3 &a, const Vector3 &b) {
            Vector3 da = a - center;
            Vector3 db = b - center;
            float aa = std::atan2(glm::dot(da, v), glm::dot(da, u));
            float ba = std::atan2(glm::dot(db, v), glm::dot(db, u));
            return aa < ba;
        });

    return verts;
}

// Deterministic, visually-distinct ABGR color from a room ID. Uses a
// fixed-prime hash to scramble adjacent IDs into different hues, then
// constructs an HSV→RGB result with high saturation and brightness so
// every room stands out from world geometry.
inline uint32_t colorFromRoomID(int32_t roomID)
{
    // Hash to [0, 1) for hue.
    uint32_t h = static_cast<uint32_t>(roomID) * 2654435761u;
    float hue = static_cast<float>(h & 0xFFFFFF) / static_cast<float>(0xFFFFFF);

    // HSV → RGB with S=1, V=1.
    float k = hue * 6.0f;
    int sector = static_cast<int>(k) % 6;
    float f = k - std::floor(k);
    float r = 0, g = 0, b = 0;
    switch (sector) {
        case 0: r = 1;     g = f;     b = 0;     break;
        case 1: r = 1 - f; g = 1;     b = 0;     break;
        case 2: r = 0;     g = 1;     b = f;     break;
        case 3: r = 0;     g = 1 - f; b = 1;     break;
        case 4: r = f;     g = 0;     b = 1;     break;
        case 5: r = 1;     g = 0;     b = 1 - f; break;
    }

    uint32_t R = static_cast<uint32_t>(r * 255.0f);
    uint32_t G = static_cast<uint32_t>(g * 255.0f);
    uint32_t B = static_cast<uint32_t>(b * 255.0f);
    // PosColorVertex packs ABGR (BGFX_VERTEX_LAYOUT_COLOR_NORMALIZED).
    return 0xFF000000u | (B << 16) | (G << 8) | R;
}

} // namespace Darkness
