/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2025 darkness contributors
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

// DoorPortalBinding.h — bind door leaves to the WR portal polygons they fill.
//
// Kept free of bgfx (like CellGeometry.h) so the tests can exercise the
// binding rule directly; DarknessRendererCore.h includes it and the renderer
// consumes the result as its portal-cull door gate.

#pragma once

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <vector>
#include <algorithm>
#include <unordered_map>

#include "DarknessMath.h"
#include "WRChunkParser.h"

namespace Darkness {

// ── Door → WR portal aperture binding ──
//
// A shut door must close exactly the opening it physically fills. Binding it
// to anything coarser deletes real geometry: keying the gate on the ROOM_DB
// room pair instead blanks the whole shared boundary of those two rooms, and
// because a Thief doorway carries its own thin threshold room brush, the cells
// dropped are the doorway's — so the jamb, head and threshold reveal stop
// being drawn while the door is shut. It reads exactly like the frame surfaces
// have flipped to face away from the viewer.
//
// So we bind by geometry instead. A WR portal polygon belongs to a door when
// all three hold against the leaf's closed-pose OBB:
//
//   1. the portal plane is parallel to the leaf face,
//   2. the centroid is within the leaf footprint laterally and vertically,
//   3. the leaf OBSTRUCTS it — the portal passes through the slab.
//
// All three are load-bearing. Parallelism alone claims every coplanar portal
// further along the same wall; the footprint alone claims the floor/ceiling
// BSP splits crowding a doorway; and without (3) a door claims the whole
// stack of coplanar boundaries behind its opening — see kApertureSlabEps,
// which is where the first version of this went wrong.

// Closed-pose world OBB of one vision-blocking door leaf.
struct DoorAperture {
    int32_t           objID  = 0;
    Darkness::Vector3 center{0.0f};   // leaf centre, closed pose
    Darkness::Vector3 axis[3];        // orthonormal world axes of the leaf
    Darkness::Vector3 half{0.0f};     // half extents along those axes
    int               thinAxis = 1;   // index of the smallest half extent
};

// cellPair → the door leaves filling the opening between those two cells.
using DoorPortalMap = std::unordered_map<uint64_t, std::vector<int32_t>>;

inline uint64_t cellPairKey(uint32_t a, uint32_t b) {
    uint32_t lo = std::min(a, b), hi = std::max(a, b);
    return (static_cast<uint64_t>(lo) << 32) | hi;
}

// In-plane slack: the aperture is cut slightly larger than the leaf filling
// it, and the portal centroid sits at the opening's centre, so a foot of
// clearance never reaches a neighbouring opening. Measured lateral/vertical
// overshoot past the leaf footprint: median 0.00 ft, max 1.00 ft.
constexpr float kApertureInPlaneMargin = 1.0f;

// Through-plane slack. This one has to be TIGHT, and the first cut of this
// code got it wrong: a 2 ft margin let a shut door claim every coplanar cell
// boundary within 2 ft of its face, which in a BSP-split doorway is a stack
// of them. Door 419 in MISS6 claimed boundaries at 0.04, 0.62, 0.87, 1.37 and
// 1.88 ft — only the first is its opening; blocking the rest cut real cells
// out of the rooms on either side, so shutting the door punched fresh holes
// near the doorway. 122 of 344 claims were more than a foot off the leaf.
//
// The physical rule is simply that the leaf must OBSTRUCT the portal, i.e.
// the portal passes through the slab: through <= halfThickness + eps. The
// data is sharply bimodal — 29 of 30 MISS6 doors have their nearest portal at
// 0.04 ft (inside the leaf), and the aperture count is flat across
// eps = 0.20…0.40 before climbing again as genuine BSP boundaries drift in.
// 0.30 ft sits in the middle of that gap.
constexpr float kApertureSlabEps = 0.30f;

// Fallback for a leaf whose origin is offset from the opening it fills (a
// tall gate whose P$PhysDims is not centred on the leaf, e.g. MISS6 door 705,
// whose only candidate sits 1.50 ft out). Rather than widen the slab test for
// everyone, such a door claims ONLY its nearest cluster of candidates, out to
// this absolute ceiling. Still one opening, never a stack of boundaries.
constexpr float kApertureNearestTol  = 0.25f;
constexpr float kApertureHardCeiling = 3.0f;

// cos 45° — the leaf face and the opening it fills are nominally coplanar;
// the tolerance only has to absorb frame bevels and non-orthogonal brushwork.
constexpr float kApertureParallelDot   = 0.70f;

// Bind every door to the WR portal polygons it fills. Doors that claim no
// portal are reported: they simply never block, which over-renders (costs
// frames) rather than punching holes, but it is never intentional.
inline DoorPortalMap
buildDoorPortalMap(const Darkness::WRParsedData &wr,
                   const std::vector<DoorAperture> &doors) {
    DoorPortalMap out;
    if (doors.empty()) return out;

    // Gather candidates per door first: which portal a leaf really fills is a
    // question about that leaf's whole candidate set (is this the boundary at
    // the leaf, or one of the BSP splits stacked behind it?), so it cannot be
    // decided one portal at a time.
    struct Candidate { float through; uint64_t key; };
    std::vector<std::vector<Candidate>> cands(doors.size());

    for (uint32_t ci = 0; ci < wr.numCells; ++ci) {
        const auto &cell = wr.cells[ci];
        int numSolid = cell.numPolygons - cell.numPortals;

        for (int pi = numSolid; pi < cell.numPolygons; ++pi) {
            const auto &poly = cell.polygons[pi];
            if (poly.count < 3) continue;
            if (poly.tgtCell >= wr.numCells || poly.tgtCell == ci) continue;
            if (poly.plane >= cell.planes.size()) continue;

            const auto &indices = cell.polyIndices[pi];
            Darkness::Vector3 centroid(0.0f);
            int n = 0;
            for (int vi = 0; vi < poly.count; ++vi) {
                uint8_t idx = indices[vi];
                if (idx >= cell.vertices.size()) continue;
                centroid += cell.vertices[idx];
                ++n;
            }
            if (n < 3) continue;
            centroid /= static_cast<float>(n);

            const Darkness::Vector3 &pn = cell.planes[poly.plane].normal;

            for (size_t di = 0; di < doors.size(); ++di) {
                const DoorAperture &d = doors[di];

                if (std::fabs(glm::dot(pn, d.axis[d.thinAxis])) <
                        kApertureParallelDot)
                    continue;

                Darkness::Vector3 rel = centroid - d.center;
                const float through = std::fabs(glm::dot(rel, d.axis[d.thinAxis]));
                if (through > d.half[d.thinAxis] + kApertureHardCeiling)
                    continue;

                bool insidePlane = true;
                for (int k = 0; k < 3 && insidePlane; ++k) {
                    if (k == d.thinAxis) continue;
                    insidePlane = std::fabs(glm::dot(rel, d.axis[k]))
                                  <= d.half[k] + kApertureInPlaneMargin;
                }
                if (!insidePlane) continue;

                cands[di].push_back({through, cellPairKey(ci, poly.tgtCell)});
            }
        }
    }

    int bound = 0, viaNearest = 0;
    for (size_t di = 0; di < doors.size(); ++di) {
        const DoorAperture &d = doors[di];
        const auto &cl = cands[di];
        if (cl.empty()) {
            std::fprintf(stderr,
                "[FALLBACK] buildDoorPortalMap: door %d claims no WR portal "
                "(centre %.1f,%.1f,%.1f half %.2f,%.2f,%.2f) — it will never "
                "block portal traversal\n",
                d.objID, d.center.x, d.center.y, d.center.z,
                d.half.x, d.half.y, d.half.z);
            continue;
        }

        // Primary: the leaf physically obstructs the portal.
        float cut = d.half[d.thinAxis] + kApertureSlabEps;
        bool any = false;
        for (const auto &c : cl) if (c.through <= cut) { any = true; break; }

        if (!any) {
            // Leaf origin is offset from the opening. Take the nearest
            // cluster only — never the whole stack behind it.
            float nearest = cl[0].through;
            for (const auto &c : cl) nearest = std::min(nearest, c.through);
            cut = nearest + kApertureNearestTol;
            ++viaNearest;
            std::fprintf(stderr,
                "[FALLBACK] buildDoorPortalMap: door %d fills no portal within "
                "%.2f ft of its leaf (nearest %.2f ft, half-thickness %.2f) — "
                "binding to that nearest opening only. Its P$PhysDims is "
                "probably not centred on the leaf.\n",
                d.objID, kApertureSlabEps, nearest, d.half[d.thinAxis]);
        }

        for (const auto &c : cl) {
            if (c.through > cut) continue;
            auto &leaves = out[c.key];
            if (std::find(leaves.begin(), leaves.end(), d.objID) == leaves.end())
                leaves.push_back(d.objID);
        }
        ++bound;
    }

    std::fprintf(stderr,
        "[DoorPortal] %d/%zu vision-blocking doors bound to %zu apertures "
        "(%d via the nearest-opening fallback)\n",
        bound, doors.size(), out.size(), viaNearest);

    return out;
}

} // namespace Darkness
