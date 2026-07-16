/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2024-2026 darkness contributors
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

#ifndef __WR_ACOUSTIC_TOPOLOGY_H
#define __WR_ACOUSTIC_TOPOLOGY_H

/// @file WRAcousticTopology.h
/// Builds WorldApertureData (see that header's WHY) from the compiled WR
/// cell geometry + ROOM_DB: which ROOM_DB portals correspond to a REAL
/// world-geometry opening, where that opening actually is, and how the
/// level's air decomposes into regions bounded by real apertures.
///
/// Renderer-side because the WR data lives here, mirroring the
/// setRaycaster / setPointInAirFn precedent. The oracle itself:
///
///   a ROOM_DB portal is REAL  <=>  a WR portal polygon CENTROID exists
///   within kApertureMatchRadiusFt of its center.
///
/// The radius is measured, not tuned: across MISS2's ground truth every
/// door-classified portal has its nearest WR portal CENTROID within
/// 4.03 ft, every fictional boundary's nearest centroid is at least
/// 8.32 ft away — no overlap, ~60% margin on both sides of 5.0
/// (PLAN.PATHING_DESIGN.md §34). CAVEAT the metric honestly: this is
/// centroid distance, NOT distance to portal AREA — the calibration was
/// measured in centroid space and holds there. A very LARGE real opening
/// whose ROOM_DB center is offset far from the polygon centroid could in
/// principle miss; nearMissCount (nearest centroid in (5, 12] ft) is
/// reported per level so that class is visible instead of silent. Any
/// metric change must be re-calibrated offline across all 15 levels
/// first (the wr_portals / wr_cells verbs are the instruments).
///
/// The geometry walk (portal polygons are the trailing numPortals entries of
/// a cell's polygon list; centroid; inradius = min centroid-to-edge-segment
/// distance) intentionally matches the `wr_cells` / `wr_portals` headless
/// verbs in DarknessHeadless.cpp — those verbs are the offline instruments
/// this builder's numbers are validated against (504 apertures / 0 in-solid /
/// 364 regions on MISS2). Keep them in sync.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "CellGeometry.h"
#include "WRChunkParser.h"
#include "audio/WorldApertureData.h"
#include "room/Room.h"
#include "room/RoomPortal.h"
#include "room/RoomService.h"

namespace Darkness {

/// Everything the renderer needs to wire AudioService for the portal-first
/// pathing bake: the aperture data itself plus the cell->region table the
/// region-of-point callback closes over.
struct WRAcousticTopologyResult {
    WorldApertureData data;
    std::vector<int32_t> cellRegion;   ///< cell index -> region id
    int rdbPortalsTotal   = 0;         ///< deduped ROOM_DB portals examined
    int fictionalPortals  = 0;         ///< no WR aperture within the radius
    int unplaceableCount  = 0;         ///< matched, but no in-air nudge found
    /// ROOM_DB portals whose NEAREST WR portal centroid falls in the
    /// (kApertureMatchRadiusFt, 12] ft band — the oracle's sensitivity
    /// zone. High counts on a level mean the centroid-distance calibration
    /// (measured on MISS2/MISS6) deserves an offline re-check there.
    int nearMissCount     = 0;
};

/// Point -> containing WR cell, gridded. findCameraCell is a full
/// O(numCells) scan; the region-of-point callback and the bake's coverage
/// fill call it thousands of times per bake (once per floor candidate,
/// per surviving candidate, per probe in parity), which measured out at
/// hundreds of ms of pure rescanning. Cells are bucketed by their bounding
/// spheres' AABBs at build time; a lookup tests only the handful of cells
/// whose sphere covers the query point (same sphere-reject + all-planes
/// test as findCameraCell, so results agree exactly).
class WRCellLocator {
public:
    explicit WRCellLocator(const WRParsedData &wr) : mWr(&wr) {
        for (uint32_t ci = 0; ci < wr.numCells; ++ci) {
            const auto &c = wr.cells[ci];
            const int lo[3] = {gridOf(c.center.x - c.radius),
                               gridOf(c.center.y - c.radius),
                               gridOf(c.center.z - c.radius)};
            const int hi[3] = {gridOf(c.center.x + c.radius),
                               gridOf(c.center.y + c.radius),
                               gridOf(c.center.z + c.radius)};
            for (int i = lo[0]; i <= hi[0]; ++i)
                for (int j = lo[1]; j <= hi[1]; ++j)
                    for (int k = lo[2]; k <= hi[2]; ++k)
                        mGrid[key(i, j, k)].push_back(ci);
        }
    }

    /// Same contract as findCameraCell: containing cell index, or -1.
    int32_t locate(float x, float y, float z) const {
        auto it = mGrid.find(key(gridOf(x), gridOf(y), gridOf(z)));
        if (it == mGrid.end()) return -1;
        const Vector3 pt(x, y, z);
        for (uint32_t ci : it->second) {
            const auto &cell = mWr->cells[ci];
            const float dx = x - cell.center.x;
            const float dy = y - cell.center.y;
            const float dz = z - cell.center.z;
            if (dx * dx + dy * dy + dz * dz > cell.radius * cell.radius)
                continue;
            bool inside = true;
            for (const auto &plane : cell.planes) {
                if (plane.getDistance(pt) < -0.1f) { inside = false; break; }
            }
            if (inside) return static_cast<int32_t>(ci);
        }
        return -1;
    }

private:
    // 16 ft buckets: cell bounding spheres are tens of ft, so each sphere
    // spans a handful of buckets and each bucket holds a handful of cells.
    static int gridOf(float v) {
        return static_cast<int>(std::floor(v / 16.0f));
    }
    static uint64_t key(int i, int j, int k) {
        const auto u = [](int v) {
            return static_cast<uint64_t>(static_cast<int64_t>(v) & 0x1FFFFF);
        };
        return (u(i) << 42) | (u(j) << 21) | u(k);
    }
    const WRParsedData *mWr;
    std::unordered_map<uint64_t, std::vector<uint32_t>> mGrid;
};

namespace detail {

/// One deduped WR portal polygon (each physical face appears in both cells'
/// polygon lists; deduped by cell pair + quantized centroid).
struct WRPortalPoly {
    uint32_t cellA = 0, cellB = 0;     ///< cellA < cellB
    Vector3  centroid{0.0f, 0.0f, 0.0f};
    float    inradiusFt = -1.0f;
};

struct UnionFind {
    std::vector<int32_t> parent;
    explicit UnionFind(size_t n) : parent(n) {
        for (size_t i = 0; i < n; ++i) parent[i] = static_cast<int32_t>(i);
    }
    int32_t find(int32_t x) {
        while (parent[static_cast<size_t>(x)] != x) {
            parent[static_cast<size_t>(x)] =
                parent[static_cast<size_t>(parent[static_cast<size_t>(x)])];
            x = parent[static_cast<size_t>(x)];
        }
        return x;
    }
    void unite(int32_t a, int32_t b) {
        a = find(a); b = find(b);
        if (a != b) parent[static_cast<size_t>(a)] = b;
    }
};

} // namespace detail

/// The measured oracle radius — see the file comment. Shared with the bake's
/// consumers through WorldApertureData semantics, not re-tuned per level.
constexpr float kApertureMatchRadiusFt = 5.0f;

inline WRAcousticTopologyResult
buildWRAcousticTopology(const WRParsedData &wr, RoomService &roomSvc)
{
    using detail::WRPortalPoly;
    using detail::UnionFind;

    WRAcousticTopologyResult out;
    if (wr.numCells == 0) return out;

    // ── 1. Deduped WR portal polygons + per-cell interior reference ──────
    // Interior reference = vertex average (cells are convex, so it is
    // strictly inside) — used for the in-air nudge direction. The cell's
    // stored `center` is a bounding-sphere center and may lie OUTSIDE a
    // long thin cell, so it cannot serve here.
    std::vector<Vector3> cellInterior(wr.numCells, Vector3(0, 0, 0));
    std::vector<WRPortalPoly> portals;
    {
        std::map<std::tuple<uint32_t, uint32_t, int, int, int>, int> dedup;
        for (uint32_t ci = 0; ci < wr.numCells; ++ci) {
            const auto &cell = wr.cells[ci];
            if (!cell.vertices.empty()) {
                Vector3 acc(0, 0, 0);
                for (const auto &v : cell.vertices) acc += v;
                cellInterior[ci] = acc / static_cast<float>(cell.vertices.size());
            }
            const int numSolid = static_cast<int>(cell.numPolygons)
                               - static_cast<int>(cell.numPortals);
            for (int pi = numSolid; pi < static_cast<int>(cell.numPolygons); ++pi) {
                if (pi < 0 || pi >= static_cast<int>(cell.polyIndices.size()))
                    continue;
                const auto &idx = cell.polyIndices[pi];
                if (idx.size() < 3) continue;
                int32_t tgt = static_cast<int32_t>(cell.polygons[pi].tgtCell);
                if (tgt < 0 || tgt >= static_cast<int32_t>(wr.numCells))
                    continue;

                Vector3 cen(0, 0, 0);
                bool badIdx = false;
                for (uint8_t vi : idx) {
                    if (vi >= cell.vertices.size()) { badIdx = true; break; }
                    cen += cell.vertices[vi];
                }
                if (badIdx) continue;
                cen /= static_cast<float>(idx.size());

                // inradius = min centroid-to-edge-SEGMENT distance = half the
                // opening's narrow dimension (same metric as wr_portals).
                float inradius = -1.0f;
                for (size_t v = 0; v < idx.size(); ++v) {
                    const Vector3 &a = cell.vertices[idx[v]];
                    const Vector3 &b = cell.vertices[idx[(v + 1) % idx.size()]];
                    const Vector3 ab = b - a;
                    const float ab2 = glm::dot(ab, ab);
                    float d;
                    if (ab2 < 1e-8f) {
                        d = glm::length(cen - a);
                    } else {
                        float t = glm::dot(cen - a, ab) / ab2;
                        t = glm::clamp(t, 0.0f, 1.0f);
                        d = glm::length(cen - (a + t * ab));
                    }
                    if (inradius < 0.0f || d < inradius) inradius = d;
                }

                const uint32_t lo = std::min(ci, static_cast<uint32_t>(tgt));
                const uint32_t hi = std::max(ci, static_cast<uint32_t>(tgt));
                const auto key = std::make_tuple(
                    lo, hi,
                    static_cast<int>(std::lround(cen.x * 10.0f)),
                    static_cast<int>(std::lround(cen.y * 10.0f)),
                    static_cast<int>(std::lround(cen.z * 10.0f)));
                if (dedup.count(key)) continue;
                dedup[key] = static_cast<int>(portals.size());
                WRPortalPoly p;
                p.cellA = lo;
                p.cellB = hi;
                p.centroid = cen;
                p.inradiusFt = inradius;
                portals.push_back(p);
            }
        }
    }

    // ── 2. Spatial hash over portal centroids (ball queries at R=5) ──────
    constexpr float kGridFt = 8.0f;
    auto gridKey = [](const Vector3 &p) {
        return std::make_tuple(
            static_cast<int>(std::floor(p.x / kGridFt)),
            static_cast<int>(std::floor(p.y / kGridFt)),
            static_cast<int>(std::floor(p.z / kGridFt)));
    };
    std::map<std::tuple<int, int, int>, std::vector<int>> grid;
    for (size_t i = 0; i < portals.size(); ++i)
        grid[gridKey(portals[i].centroid)].push_back(static_cast<int>(i));
    auto ballQuery = [&](const Vector3 &p, float r, std::vector<int> &hits) {
        hits.clear();
        const float r2 = r * r;
        const int lo[3] = {static_cast<int>(std::floor((p.x - r) / kGridFt)),
                           static_cast<int>(std::floor((p.y - r) / kGridFt)),
                           static_cast<int>(std::floor((p.z - r) / kGridFt))};
        const int hi[3] = {static_cast<int>(std::floor((p.x + r) / kGridFt)),
                           static_cast<int>(std::floor((p.y + r) / kGridFt)),
                           static_cast<int>(std::floor((p.z + r) / kGridFt))};
        for (int i = lo[0]; i <= hi[0]; ++i)
            for (int j = lo[1]; j <= hi[1]; ++j)
                for (int k = lo[2]; k <= hi[2]; ++k) {
                    auto it = grid.find(std::make_tuple(i, j, k));
                    if (it == grid.end()) continue;
                    for (int pi : it->second) {
                        const Vector3 d = portals[static_cast<size_t>(pi)]
                                              .centroid - p;
                        if (glm::dot(d, d) <= r2) hits.push_back(pi);
                    }
                }
    };

    // ── 3. Oracle match: deduped ROOM_DB portals vs WR portal area ───────
    // Aperture IDENTITY via union-find on WR portals: all WR portals matched
    // by one ROOM_DB portal are the same opening, and two ROOM_DB portals
    // matching overlapping WR sets (a threshold slab's two faces) are the
    // same opening too — connected components of that relation are the
    // physical apertures. This replaces the old geometric same-opening
    // heuristics with an identity.
    struct RdbMatch {
        Vector3 center;
        int32_t portalID = -1;
        int roomA = -1, roomB = -1;
        std::vector<int> matched;
    };
    std::vector<RdbMatch> matches;
    std::vector<char> isCut(portals.size(), 0);
    {
        std::vector<int> hits;
        std::vector<int> nearHits;
        const auto &rooms = roomSvc.getAllRooms();
        for (const auto &roomPtr : rooms) {
            if (!roomPtr) continue;
            const uint32_t pc = roomPtr->getPortalCount();
            for (uint32_t i = 0; i < pc; ++i) {
                RoomPortal *portal = roomPtr->getPortal(i);
                if (!portal) continue;
                // Same back-link dedup as the bake's classification pass.
                if (portal->getPortalID() > portal->getDestPortalID())
                    continue;
                ++out.rdbPortalsTotal;
                const Vector3 center = portal->getCenter();
                ballQuery(center, kApertureMatchRadiusFt, hits);
                if (hits.empty()) {
                    ++out.fictionalPortals;
                    // Sensitivity diagnostic: fiction whose nearest centroid
                    // sits just past the radius. See the file comment on the
                    // centroid-distance metric's calibration scope.
                    ballQuery(center, 12.0f, nearHits);
                    if (!nearHits.empty()) ++out.nearMissCount;
                    continue;
                }
                RdbMatch m;
                m.center = center;
                m.portalID = portal->getPortalID();
                m.roomA = static_cast<int>(roomPtr->getRoomID());
                Room *far = portal->getFarRoom();
                m.roomB = far ? static_cast<int>(far->getRoomID()) : -1;
                m.matched = hits;
                for (int h : hits) isCut[static_cast<size_t>(h)] = 1;
                matches.push_back(std::move(m));
            }
        }
    }

    // ── 4. Regions: flood the cell graph, cutting matched apertures ──────
    // A region = one air volume bounded by real openings — what a ROOM_DB
    // room pretends to be (MISS2: ~47 multi-cell regions vs 404 rooms).
    UnionFind cellUF(wr.numCells);
    for (size_t i = 0; i < portals.size(); ++i) {
        if (isCut[i]) continue;
        cellUF.unite(static_cast<int32_t>(portals[i].cellA),
                     static_cast<int32_t>(portals[i].cellB));
    }
    out.cellRegion.assign(wr.numCells, -1);
    {
        std::unordered_map<int32_t, int32_t> compact;
        for (uint32_t c = 0; c < wr.numCells; ++c) {
            const int32_t root = cellUF.find(static_cast<int32_t>(c));
            auto it = compact.find(root);
            if (it == compact.end()) {
                it = compact.emplace(root,
                        static_cast<int32_t>(compact.size())).first;
            }
            out.cellRegion[c] = it->second;
        }
        out.data.numRegions = static_cast<int>(compact.size());
    }

    // ── 5. Emit records: one per (matched ROOM_DB portal, TRUE region pair) ─
    // Two corrections over the naive "rep portal's two cells" emission:
    //
    // (a) SIDE RESOLUTION. A doorway is a frame CHAIN of thin cut cells
    //     (A | f1 | f2 | B): a member portal's immediate cells are often
    //     frame SINGLETON regions, not the two air volumes the opening
    //     joins. Region IDs taken from immediate cells send coverage
    //     demand into 1-cell singletons (the true far side then looks
    //     sealed and unseeded) and make parity skip the aperture as
    //     empty-side. So each side is resolved by walking cut portals of
    //     this match through the frame until it exits into a region that
    //     owns cells beyond the frame itself — the TRUE side.
    //
    // (b) MULTI-OPENING BALLS. One match ball can span two distinct
    //     physical openings (a doorway plus a nearby window joining a
    //     different region pair). All hits were cut — both openings
    //     became region boundaries — so each TRUE pair gets its own
    //     record, or the second opening is silently unprobed AND
    //     invisible to [REGION_PARITY] (which iterates records).
    //
    // probePos: aperture centroid nudged off the face into a side cell and
    // VERIFIED against the cell network — in-air by construction, or the
    // record is dropped LOUDLY below.
    const WRCellLocator locator(wr);
    constexpr float kNudgeFt = 0.25f;
    // GLOBAL cut-portal adjacency + per-cell incident-cut count, for side
    // resolution. Built from ALL matches' cuts, not per-match: a door's
    // 5-ft ball routinely misses members of its own frame chain (they were
    // cut by NEIGHBORING matches), and a per-match walk then stopped inside
    // the frame and reported a slab region as a "side" — which is how the
    // same doorway re-emitted as several spurious openings (measured on
    // MISS7 door 36: three probes stacked in one doorway).
    std::map<uint32_t, std::vector<uint32_t>> cutAdj;
    std::map<uint32_t, int> cutCount;   // incident cut portals per cell
    for (size_t pi = 0; pi < portals.size(); ++pi) {
        if (!isCut[pi]) continue;
        const WRPortalPoly &p = portals[pi];
        cutAdj[p.cellA].push_back(p.cellB);
        cutAdj[p.cellB].push_back(p.cellA);
        ++cutCount[p.cellA];
        ++cutCount[p.cellB];
    }
    // Resolve a starting cell to its TRUE side region: keep crossing cut
    // portals while the current cell is a PASS-THROUGH (>= 2 incident cut
    // portals — a frame slab has a cut face front AND back); stop at the
    // first cell that is not (a 1-cut-portal cell is a genuine terminus:
    // a room whose only cut is its own door). Threshold-free and local.
    // KNOWN APPROXIMATION: a 1-cell room whose single cell touches two
    // doorways reads as pass-through and resolves through — acoustically
    // it IS mostly a pass-through, and both doors' flank influence covers
    // it. Walk budget 64 cells (frame chains are a handful).
    auto resolveSide = [&](uint32_t start) -> int32_t {
        std::set<uint32_t> seen{start};
        std::vector<uint32_t> q{start};
        int budget = 64;
        while (!q.empty() && budget-- > 0) {
            const uint32_t c = q.back();
            q.pop_back();
            auto cc = cutCount.find(c);
            if (cc == cutCount.end() || cc->second < 2)
                return out.cellRegion[c];       // terminus: a true side
            auto it = cutAdj.find(c);
            if (it == cutAdj.end()) continue;
            for (uint32_t n : it->second)
                if (seen.insert(n).second) q.push_back(n);
        }
        return out.cellRegion[start];   // fully-framed pocket: keep raw
    };
    for (const RdbMatch &m : matches) {

        std::map<std::pair<int32_t, int32_t>, int> repByPair;
        for (int h : m.matched) {
            const WRPortalPoly &p = portals[static_cast<size_t>(h)];
            const int32_t ra = resolveSide(p.cellA);
            const int32_t rb = resolveSide(p.cellB);
            const auto pairKey = std::make_pair(std::min(ra, rb),
                                                std::max(ra, rb));
            auto it = repByPair.find(pairKey);
            if (it == repByPair.end()
                || p.inradiusFt >
                       portals[static_cast<size_t>(it->second)].inradiusFt) {
                repByPair[pairKey] = h;
            }
        }
        for (const auto &kv : repByPair) {
            const WRPortalPoly &rep = portals[static_cast<size_t>(kv.second)];
            WorldApertureRecord rec;
            rec.wrCentroid = rep.centroid;
            rec.apertureInradiusFt = rep.inradiusFt;
            rec.portalID = m.portalID;
            rec.rdbCenter = m.center;
            rec.roomAID = std::min(m.roomA, m.roomB);
            rec.roomBID = std::max(m.roomA, m.roomB);
            rec.regionA = out.cellRegion[rep.cellA];
            rec.regionB = out.cellRegion[rep.cellB];
            // Identity = resolved region pair + COARSE (8 ft) centroid
            // cell. Cross-record safe: the same doorway matched through
            // two different ROOM_DB portals now hashes identically (the
            // old per-match union root gave it two keys and the claim
            // dedup missed it). Coarse-grid boundary imprecision is
            // acceptable here because this key only backs DIAGNOSTIC
            // dedup ([REGION_PARITY]'s per-opening count) — EMISSION
            // dedups with an exact distance test on (pair, centroid).
            const auto q8 = [](float v) -> uint64_t {
                return static_cast<uint64_t>(
                    static_cast<int64_t>(std::floor(v / 8.0f)) & 0xFFFF);
            };
            rec.apertureKey =
                (static_cast<uint64_t>(
                     static_cast<uint32_t>(kv.first.first)) << 48)
                ^ (static_cast<uint64_t>(
                       static_cast<uint32_t>(kv.first.second)) << 33)
                ^ (q8(rep.centroid.x) << 22)
                ^ (q8(rep.centroid.y) << 11)
                ^ q8(rep.centroid.z);

            bool placed = false;
            for (uint32_t side : {rep.cellA, rep.cellB}) {
                Vector3 dir = cellInterior[side] - rep.centroid;
                const float len = glm::length(dir);
                if (len < 1e-4f) continue;
                const Vector3 cand = rep.centroid + dir * (kNudgeFt / len);
                if (locator.locate(cand.x, cand.y, cand.z) >= 0) {
                    rec.probePos = cand;
                    placed = true;
                    break;
                }
            }
            if (!placed) {
                // Degenerate side-cell geometry (paper-thin frame cell whose
                // vertex average sits on the face). Fall back to the raw
                // centroid IF the network claims it; otherwise drop — LOUDLY:
                // a real aperture vanishing from the data means placement and
                // [REGION_PARITY] never see that opening.
                if (locator.locate(rep.centroid.x, rep.centroid.y,
                                   rep.centroid.z) >= 0) {
                    rec.probePos = rep.centroid;
                    placed = true;
                }
            }
            if (!placed) {
                ++out.unplaceableCount;
                std::fprintf(stderr,
                    "[FALLBACK] aperture topology: REAL aperture at "
                    "(%.1f,%.1f,%.1f) (rooms %d<->%d, inradius %.2f ft) has "
                    "no in-air anchor — record DROPPED; that opening is "
                    "invisible to placement and region parity\n",
                    rep.centroid.x, rep.centroid.y, rep.centroid.z,
                    rec.roomAID, rec.roomBID, rep.inradiusFt);
                continue;
            }
            out.data.apertures.push_back(rec);
        }
    }

    out.data.valid = true;
    return out;
}

} // namespace Darkness

#endif // __WR_ACOUSTIC_TOPOLOGY_H
