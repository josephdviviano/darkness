/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2024-2026 darkness contributors
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 3 of the License, or
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

#ifndef __HYBRID_ROUTE_GRAPH_H
#define __HYBRID_ROUTE_GRAPH_H

/// @file HybridRouteGraph.h
/// Our own routing over the baked pathing probe graph — the first concrete
/// piece of the self-routed hybrid (.claude/PLAN.SELF_ROUTED_HYBRID.md).
///
/// Steam Audio's pathing hides WHICH doors the sound's route passes through
/// (the public C API returns only eqCoeffs + SH, never the path). We need
/// that to drive the door-fraction volume gate correctly: gate = PRODUCT of
/// the open-fractions of the doors ON THE ACTIVE PATH, so a closing door on
/// the route fades the routed sound with it, while an off-path door closing
/// nearby does not touch it (the bug the min-over-route-SCOPE gate had).
///
/// This reconstructs the route ourselves: a Dijkstra over the probe graph
/// (the same probe positions + visibility edges Steam Audio routes on),
/// treating an edge crossing a FULLY-CLOSED door as removed, and returns the
/// doors the shortest surviving path crosses. Microseconds — pure graph
/// traversal, no per-edge ray casts (the reason SA's own search is slow).
///
/// Header-only so it is unit-testable without a live Steam Audio scene.

#include "DarknessMath.h"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

namespace Darkness {

class HybridRouteGraph {
public:
    /// One door as its CLOSED-pose oriented box (the doorway footprint). An
    /// edge is "on" this door if its segment intersects the box. This
    /// generalises across door sizes and orientations where a doorway sphere
    /// did not: the thin slab catches the flank-to-flank edge that threads
    /// the opening (perpendicular, through the slab) and rejects edges that
    /// merely pass NEAR the doorway in an adjacent room. Pose-independent —
    /// the CLOSED pose is the doorway, regardless of the door's swing.
    /// `worldToLocal` is the inverse of the closed-pose world transform;
    /// `halfExtents` (margin already folded in) is the local box half-size,
    /// centred at the local origin (the box mesh is authored so).
    struct DoorBox {
        int32_t id = -1;
        Matrix4 worldToLocal{1.0f};
        Vector3 halfExtents{0.0f, 0.0f, 0.0f};
    };

    /// Build the routing graph. `edges` are undirected (a,b) index pairs into
    /// `probePos` (self-loops and out-of-range indices are skipped). Each
    /// edge is annotated with the doors whose doorway box it crosses.
    /// Rebuild whenever the probe graph is (re)baked.
    void build(const std::vector<Vector3> &probePos,
               const std::vector<std::pair<int, int>> &edges,
               const std::vector<DoorBox> &doors)
    {
        mProbePos = probePos;
        mAdj.assign(mProbePos.size(), {});
        mEdgeDoors.clear();
        mEdgeDoors.reserve(edges.size());
        buildGrid();

        for (const auto &e : edges) {
            const int a = e.first, b = e.second;
            if (a == b) continue;
            if (a < 0 || b < 0 ||
                a >= static_cast<int>(mProbePos.size()) ||
                b >= static_cast<int>(mProbePos.size()))
                continue;
            const int edgeIdx = static_cast<int>(mEdgeDoors.size());
            const float len = glm::length(mProbePos[b] - mProbePos[a]);
            mAdj[a].push_back({b, edgeIdx, len});
            mAdj[b].push_back({a, edgeIdx, len});
            // Doors whose doorway box this edge's segment intersects. The
            // box is the door's closed-pose slab, so only an edge that
            // actually threads the opening (crossing the slab) is annotated.
            std::vector<int32_t> onEdge;
            for (const DoorBox &d : doors) {
                if (segmentObbHit(mProbePos[a], mProbePos[b],
                                  d.worldToLocal, d.halfExtents))
                    onEdge.push_back(d.id);
            }
            mEdgeDoors.push_back(std::move(onEdge));
        }
        mBuilt = true;
    }

    bool   built() const { return mBuilt; }
    size_t numProbes() const { return mProbePos.size(); }
    /// Total door↔edge associations (diagnostic).
    size_t doorEdgeCount() const {
        size_t n = 0;
        for (const auto &v : mEdgeDoors) n += v.size();
        return n;
    }
    /// Count of DISTINCT door IDs that annotate at least one edge. Paired with
    /// the door count passed to build(), the difference is the number of doors
    /// mapped to NO edge — a door that would gate nothing (a route leak past a
    /// closed door). Surfaced loudly so an over-tight door box does not fail
    /// silently.
    size_t doorsWithEdges() const {
        std::unordered_map<int32_t, char> seen;
        for (const auto &v : mEdgeDoors)
            for (int32_t id : v) seen[id] = 1;
        return seen.size();
    }

    /// Nearest probe index to `p` (grid-accelerated), or -1 if empty.
    int nearestProbe(const Vector3 &p) const
    {
        if (mProbePos.empty()) return -1;
        int best = -1;
        float bestSq = std::numeric_limits<float>::max();
        // Expand ring by ring over grid cells until a hit is found and the
        // next ring cannot beat it.
        const GridKey k = keyOf(p);
        for (int r = 0; r <= mGridMaxRadius; ++r) {
            for (int dx = -r; dx <= r; ++dx)
              for (int dy = -r; dy <= r; ++dy)
                for (int dz = -r; dz <= r; ++dz) {
                    // Only the shell at Chebyshev distance r.
                    if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != r)
                        continue;
                    auto it = mGrid.find({k.x+dx, k.y+dy, k.z+dz});
                    if (it == mGrid.end()) continue;
                    for (int idx : it->second) {
                        const float dsq = glm::dot(mProbePos[idx]-p,
                                                   mProbePos[idx]-p);
                        if (dsq < bestSq) { bestSq = dsq; best = idx; }
                    }
                }
            // Once we have a hit, one extra ring guarantees correctness
            // (a closer probe could sit just across a cell boundary).
            if (best >= 0 && r >= 1 &&
                std::sqrt(bestSq) <= static_cast<float>(r) * mGridCell)
                break;
        }
        if (best < 0) {   // grid miss (degenerate) — linear fallback
            for (size_t i = 0; i < mProbePos.size(); ++i) {
                const float dsq = glm::dot(mProbePos[i]-p, mProbePos[i]-p);
                if (dsq < bestSq) { bestSq = dsq; best = static_cast<int>(i); }
            }
        }
        return best;
    }

    /// Shortest door-aware route from the probe nearest `src` to the probe
    /// nearest `lst`. An edge whose box-crossed door has fraction(id) <
    /// `closedThreshold` is removed (fully-closed door blocks the route,
    /// forcing the search around it). Fills `outDoors` with the sorted unique
    /// door IDs on the surviving shortest path and RETURNS whether a route
    /// exists at all. The distinction matters for the gate:
    ///   returns true,  outDoors non-empty -> gate = product of fractions
    ///   returns true,  outDoors empty     -> door-free route, gate = 1
    ///   returns false                     -> UNREACHABLE, gate = 0 (so a
    ///                                        door closing the last route
    ///                                        fades to silence, not to 1)
    /// Unbuilt or same-probe endpoints count as reachable with no doors.
    bool route(const Vector3 &src, const Vector3 &lst,
               const std::function<float(int32_t)> &fraction,
               float closedThreshold,
               std::vector<int32_t> &outDoors) const
    {
        outDoors.clear();
        if (!mBuilt || mProbePos.empty()) return true;   // no graph: no gate
        const int s = nearestProbe(src);
        const int t = nearestProbe(lst);
        if (s < 0 || t < 0) return true;
        if (s == t) return true;   // same probe region: reachable, no door

        auto edgeBlocked = [&](int edgeIdx) -> bool {
            for (int32_t id : mEdgeDoors[edgeIdx])
                if (fraction(id) < closedThreshold) return true;
            return false;
        };

        const float kInf = std::numeric_limits<float>::max();
        std::vector<float> dist(mProbePos.size(), kInf);
        std::vector<int>   prevEdge(mProbePos.size(), -1);
        std::vector<int>   prevNode(mProbePos.size(), -1);
        using QE = std::pair<float, int>;   // (dist, node)
        std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
        dist[s] = 0.0f;
        pq.push({0.0f, s});
        while (!pq.empty()) {
            auto [d, u] = pq.top(); pq.pop();
            if (d > dist[u]) continue;
            if (u == t) break;
            for (const Edge &e : mAdj[u]) {
                if (edgeBlocked(e.edgeIdx)) continue;
                const float nd = d + e.len;
                if (nd < dist[e.to]) {
                    dist[e.to] = nd;
                    prevEdge[e.to] = e.edgeIdx;
                    prevNode[e.to] = u;
                    pq.push({nd, e.to});
                }
            }
        }
        if (dist[t] == kInf) return false;   // unreachable with doors closed

        // Walk back, collecting the doors on the path's edges.
        for (int cur = t; prevNode[cur] >= 0; cur = prevNode[cur]) {
            for (int32_t id : mEdgeDoors[prevEdge[cur]])
                outDoors.push_back(id);
        }
        std::sort(outDoors.begin(), outDoors.end());
        outDoors.erase(std::unique(outDoors.begin(), outDoors.end()),
                       outDoors.end());
        return true;
    }

private:
    struct Edge { int to; int edgeIdx; float len; };

    bool mBuilt = false;
    std::vector<Vector3> mProbePos;
    std::vector<std::vector<Edge>> mAdj;
    std::vector<std::vector<int32_t>> mEdgeDoors;

    // ── uniform grid over probe positions for nearestProbe ──
    struct GridKey { int x, y, z; bool operator==(const GridKey&o) const {
        return x==o.x && y==o.y && z==o.z; } };
    struct GridKeyHash { size_t operator()(const GridKey &k) const {
        return (static_cast<size_t>(static_cast<uint32_t>(k.x)) * 73856093u) ^
               (static_cast<size_t>(static_cast<uint32_t>(k.y)) * 19349663u) ^
               (static_cast<size_t>(static_cast<uint32_t>(k.z)) * 83492791u); } };
    std::unordered_map<GridKey, std::vector<int>, GridKeyHash> mGrid;
    float mGridCell = 16.0f;   // feet
    int   mGridMaxRadius = 1;

    GridKey keyOf(const Vector3 &p) const {
        return { static_cast<int>(std::floor(p.x / mGridCell)),
                 static_cast<int>(std::floor(p.y / mGridCell)),
                 static_cast<int>(std::floor(p.z / mGridCell)) };
    }
    void buildGrid() {
        mGrid.clear();
        int maxAbs = 1;
        for (size_t i = 0; i < mProbePos.size(); ++i) {
            const GridKey k = keyOf(mProbePos[i]);
            mGrid[k].push_back(static_cast<int>(i));
            maxAbs = std::max({maxAbs, std::abs(k.x), std::abs(k.y),
                               std::abs(k.z)});
        }
        // Worst-case ring expansion bound (a far-outside query still finds
        // the nearest probe): span of the populated grid.
        mGridMaxRadius = std::max(1, 2 * maxAbs + 2);
    }

    /// True if segment [a,b] (world) intersects the door's oriented box.
    /// Transforms both endpoints into the door's local frame — where the box
    /// is axis-aligned, centred at the origin, spanning [-halfExtents,
    /// +halfExtents] — then runs a segment-vs-AABB slab test there.
    static bool segmentObbHit(const Vector3 &a, const Vector3 &b,
                              const Matrix4 &worldToLocal,
                              const Vector3 &halfExtents) {
        const Vector3 la = Vector3(worldToLocal * glm::vec4(a, 1.0f));
        const Vector3 lb = Vector3(worldToLocal * glm::vec4(b, 1.0f));
        return segmentAabbHit(la, lb, -halfExtents, halfExtents);
    }

    /// Segment [p0,p1] vs axis-aligned box [mn,mx] via the slab method,
    /// parameterising the segment as p0 + t*(p1-p0), t in [0,1].
    static bool segmentAabbHit(const Vector3 &p0, const Vector3 &p1,
                               const Vector3 &mn, const Vector3 &mx) {
        const Vector3 d = p1 - p0;
        float tmin = 0.0f, tmax = 1.0f;
        for (int i = 0; i < 3; ++i) {
            if (std::abs(d[i]) < 1e-9f) {
                // Segment parallel to this slab: outside it => no hit.
                if (p0[i] < mn[i] || p0[i] > mx[i]) return false;
            } else {
                float t1 = (mn[i] - p0[i]) / d[i];
                float t2 = (mx[i] - p0[i]) / d[i];
                if (t1 > t2) std::swap(t1, t2);
                tmin = std::max(tmin, t1);
                tmax = std::min(tmax, t2);
                if (tmin > tmax) return false;
            }
        }
        return true;
    }
};

} // namespace Darkness

#endif // __HYBRID_ROUTE_GRAPH_H
