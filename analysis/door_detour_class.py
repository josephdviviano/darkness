#!/usr/bin/env python3
###############################################################################
#
#    This file is part of the darkness project
#    Copyright (C) 2024-2026 darkness contributors
#
#    This program is free software; you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation; either version 2 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program; if not, write to the Free Software
#    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#
###############################################################################
"""Per-portal alternate-route detour classification for one mission.

For every portal in the ROOM_DB room graph (dumped by the existing
`darknessHeadless <mis> room_graph` verb; parsing machinery shared with
analysis/room_graph_stats.py), remove that single portal and compute the
shortest alternate route between the two rooms it connects (Dijkstra over
room centers + portal centers). Each portal is classified:

  NONE   removing the portal disconnects its two rooms — a closed door on
         this portal leaves the pathing solver NO alternate route, so a
         failing findAlternatePaths search must exhaust the entire
         reachable graph component before giving up;
  LONG   an alternate route exists but the detour (alt-route length minus
         the removed portal's own edge length) exceeds --long-ft;
  SHORT  an alternate route exists within --long-ft of detour.

This is the graph-side half of the door-event pathing-latency hypothesis:
iteration cost should track the explored-graph size, i.e. NONE-class door
portals should produce the monster iterations, LONG-class the medium
spikes, SHORT-class the quick solves. The room graph is a proxy for the
baked Steam Audio probe graph (probes are emitted per portal/door, so
room-graph connectivity mirrors probe-graph connectivity).

Door annotation: pass one or more engine run logs via --map-log to pick up
the authoritative [DOOR_PORTAL_MAP] registration lines (door world AABB
matched to the nearest room portal within the engine's 3 ft point-to-AABB
classification distance — same rule as the probe-bake DoorPair classifier;
the printed midpoint is a display/join key only). Table rows for mapped
portals gain the door ID, and a per-door summary is printed.

Usage:
  python3 analysis/door_detour_class.py <levels_dir> <mission> \
      [--headless BIN] [--long-ft FT] [--map-log LOG ...] [--full-table]

  <levels_dir>   REQUIRED path to a directory of .mis files
  <mission>      mission file name inside levels_dir (case-insensitive,
                 e.g. MISS2.MIS or miss2.mis)
  --headless     path to the darknessHeadless binary
                 (default: build/default/src/main/darknessHeadless under
                 the repo root)
  --long-ft      detour threshold separating SHORT from LONG (default 100)
  --map-log      engine run log(s) containing [DOOR_PORTAL_MAP] lines;
                 repeatable
  --full-table   print every portal row (default: door-mapped + NONE +
                 LONG rows only, to keep the table readable)
"""

import argparse
import heapq
import math
import os
import re
import sys

# Shared room_graph parsing machinery (LevelData.parse_line understands the
# ROOM / PORTAL / PDIST / ROOMGRAPH_SUMMARY records emitted by the headless
# room_graph verb).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from room_graph_stats import LevelData, run_level  # noqa: E402


class UndirectedPortal:
    """One physical portal (the two directed halves paired up)."""

    __slots__ = ("idx", "a", "b", "center", "weight")

    def __init__(self, idx, a, b, center):
        self.idx = idx
        self.a = a          # lower room ID
        self.b = b          # higher room ID
        self.center = center
        self.weight = 0.0   # dist(centerA, portal) + dist(portal, centerB)


def pair_directed_portals(lvl):
    """Collapse the directed PORTAL records into physical portals.

    Every physical portal appears once per side. Multiple portals can join
    the same room pair (double doors, wide arches split by the compiler),
    so we group directed records by unordered room pair and greedily match
    each A->B record with the geometrically closest unmatched B->A record.
    Unmatched leftovers (odd counts) still become portals — counted as
    anomalies, not dropped, so no door can silently vanish from the table.
    """
    groups = {}
    for near, far, x, y, z in lvl.portals_directed:
        if far < 0 or near == far:
            continue  # anomalies already recorded by LevelData consumers
        key = (min(near, far), max(near, far))
        groups.setdefault(key, {"fwd": [], "rev": []})
        side = "fwd" if near == key[0] else "rev"
        groups[key][side].append((x, y, z))

    portals = []
    unpaired = 0
    for (a, b), g in sorted(groups.items()):
        fwd, rev = list(g["fwd"]), list(g["rev"])
        for f in fwd:
            if rev:
                # nearest remaining back-link
                jbest = min(range(len(rev)), key=lambda j: math.dist(f, rev[j]))
                r = rev.pop(jbest)
                center = tuple((f[k] + r[k]) * 0.5 for k in range(3))
            else:
                unpaired += 1
                center = f
            portals.append(UndirectedPortal(len(portals), a, b, center))
        for r in rev:  # back-links with no forward record
            unpaired += 1
            portals.append(UndirectedPortal(len(portals), a, b, r))
    return portals, unpaired


def build_graph(lvl, portals):
    """adjacency: room -> list of (neighbor, weight, portal_idx)."""
    adj = {rid: [] for rid in lvl.rooms}
    for p in portals:
        ra, rb = lvl.rooms.get(p.a), lvl.rooms.get(p.b)
        if ra is None or rb is None:
            continue
        p.weight = (math.dist(ra[:3], p.center) +
                    math.dist(p.center, rb[:3]))
        adj.setdefault(p.a, []).append((p.b, p.weight, p.idx))
        adj.setdefault(p.b, []).append((p.a, p.weight, p.idx))
    return adj


def dijkstra_without(adj, src, dst, banned_portal):
    """Shortest src->dst distance with one portal edge removed."""
    dist = {src: 0.0}
    pq = [(0.0, src)]
    while pq:
        d, u = heapq.heappop(pq)
        if u == dst:
            return d
        if d > dist.get(u, math.inf):
            continue
        for v, w, pidx in adj[u]:
            if pidx == banned_portal:
                continue
            nd = d + w
            if nd < dist.get(v, math.inf):
                dist[v] = nd
                heapq.heappush(pq, (nd, v))
    return math.inf


def component_sizes(adj, src, dst, banned_portal):
    """Room counts of the two components containing src / dst after the
    portal is removed (equal if still connected). For NONE-class portals
    this is the 'explored graph size before a failing search gives up'."""
    def flood(start):
        seen = {start}
        stack = [start]
        while stack:
            u = stack.pop()
            for v, _w, pidx in adj[u]:
                if pidx == banned_portal or v in seen:
                    continue
                seen.add(v)
                stack.append(v)
        return seen
    ca = flood(src)
    return len(ca), (len(ca) if dst in ca else len(flood(dst)))


# [DOOR_PORTAL_MAP] door=1141 mid=(-85.0,-52.0,-4.5) portal rooms 12<->34
#     center=(-85.5,-52.0,-4.0) d=0.71ft
# [DOOR_PORTAL_MAP] door=1776 mid=(-54.0,-43.0,-4.0) NO portal within 3.0ft
#     (nearest rooms 25<->204 d=5.00ft)
MAP_RE = re.compile(
    r"\[DOOR_PORTAL_MAP\] door=(\d+) mid=\(([-\d.]+),([-\d.]+),([-\d.]+)\) "
    r"(?:portal rooms (\d+)<->(\d+) center=\(([-\d.]+),([-\d.]+),([-\d.]+)\) "
    r"d=([\d.]+)ft"
    r"|NO portal within ([\d.]+)ft "
    r"\(nearest rooms (-?\d+)<->(-?\d+) d=([\d.]+)ft\))")


def parse_map_logs(paths):
    """door ID -> dict(mid, rooms, center, dist, proxy).

    rooms=None when the engine reported no portal within the match
    distance AND the nearest one is too far to trust; doors whose nearest
    portal is just outside the engine's 3 ft rule are proxy-mapped
    (proxy=True) so the forensic correlation still gets a detour class —
    clearly marked, never silently merged. (Since the engine rule became
    point-to-door-AABB the old midpoint-offset near-misses match exactly
    and this path is mostly idle; it stays for genuine borderline
    doors.)"""
    doors = {}
    for path in paths:
        with open(path, "r", errors="replace") as fh:
            for line in fh:
                m = MAP_RE.search(line)
                if not m:
                    continue
                did = int(m.group(1))
                mid = tuple(float(m.group(k)) for k in (2, 3, 4))
                if m.group(5) is not None:
                    doors[did] = {
                        "mid": mid,
                        "rooms": (int(m.group(5)), int(m.group(6))),
                        "center": tuple(float(m.group(k)) for k in (7, 8, 9)),
                        "dist": float(m.group(10)),
                        "proxy": False,
                    }
                else:
                    doors.setdefault(did, {
                        "mid": mid,
                        "rooms": (int(m.group(12)), int(m.group(13))),
                        "center": None,   # engine line has no center here
                        "dist": float(m.group(14)),
                        "proxy": True,
                    })
    return doors


def main():
    ap = argparse.ArgumentParser(
        description="Per-portal alternate-route detour classification "
                    "(NONE / LONG / SHORT) for one mission's ROOM_DB graph.")
    ap.add_argument("levels_dir", help="directory containing .mis files")
    ap.add_argument("mission", help="mission file name (case-insensitive)")
    ap.add_argument("--headless", default=None,
                    help="path to darknessHeadless binary")
    ap.add_argument("--long-ft", type=float, default=100.0,
                    help="detour threshold (ft) separating SHORT from LONG "
                         "(default 100)")
    ap.add_argument("--map-log", action="append", default=[],
                    help="engine log with [DOOR_PORTAL_MAP] lines (repeatable)")
    ap.add_argument("--full-table", action="store_true",
                    help="print all portal rows, not just door/NONE/LONG")
    args = ap.parse_args()

    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    headless = args.headless or os.path.join(
        repo_root, "build", "default", "src", "main", "darknessHeadless")
    if not os.path.isfile(headless):
        sys.exit(f"headless binary not found: {headless} (use --headless)")
    if not os.path.isdir(args.levels_dir):
        sys.exit(f"not a directory: {args.levels_dir}")

    mis_path = None
    for f in sorted(os.listdir(args.levels_dir)):
        if f.lower() == args.mission.lower():
            mis_path = os.path.join(args.levels_dir, f)
            break
    if mis_path is None:
        sys.exit(f"mission '{args.mission}' not found in {args.levels_dir}")

    lvl = run_level(headless, mis_path, repo_root)
    if lvl is None:
        sys.exit("room_graph dump failed")

    portals, unpaired = pair_directed_portals(lvl)
    adj = build_graph(lvl, portals)

    # Door -> portal annotation from engine registration logs.
    doors = parse_map_logs(args.map_log)
    portal_doors = {}   # portal idx -> [door IDs]
    door_portal = {}    # door ID -> portal idx (or None)
    # Proxy-mapping acceptance distance for doors just past the engine's
    # 3 ft point-to-AABB rule; beyond this the door probably isn't on a
    # room boundary at all (closet fully inside one room). Mostly idle
    # since the engine switched from midpoint to AABB distance.
    kProxyAcceptFt = 6.0
    for did, rec in sorted(doors.items()):
        if rec["rooms"] is None or (rec["proxy"]
                                    and rec["dist"] > kProxyAcceptFt):
            door_portal[did] = None
            continue
        a, b = min(rec["rooms"]), max(rec["rooms"])
        # nearest same-room-pair portal to the engine-reported portal
        # center (exact matches) or to the door midpoint (proxy matches —
        # the NO-portal engine line carries no portal center).
        ref = rec["center"] if rec["center"] is not None else rec["mid"]
        best, bestd = None, math.inf
        for p in portals:
            if (p.a, p.b) != (a, b):
                continue
            d = math.dist(p.center, ref)
            if d < bestd:
                best, bestd = p, d
        door_portal[did] = best.idx if best is not None else None
        if best is not None:
            portal_doors.setdefault(best.idx, []).append(did)

    # Classify every portal.
    rows = []
    counts = {"NONE": 0, "LONG": 0, "SHORT": 0}
    for p in portals:
        alt = dijkstra_without(adj, p.a, p.b, p.idx)
        if math.isinf(alt):
            cls = "NONE"
            detour = math.inf
            ca, cb = component_sizes(adj, p.a, p.b, p.idx)
        else:
            detour = alt - p.weight
            cls = "LONG" if detour > args.long_ft else "SHORT"
            ca = cb = None
        counts[cls] += 1
        rows.append((p, cls, detour, alt, ca, cb))

    name = os.path.basename(mis_path)
    print(f"\n{'=' * 100}")
    print(f"PORTAL DETOUR CLASSIFICATION — {name}  "
          f"(rooms={len(lvl.rooms)}, physical portals={len(portals)}, "
          f"unpaired directed records={unpaired}, long-ft={args.long_ft:g})")
    print(f"{'=' * 100}")
    print(f"class counts: NONE={counts['NONE']}  LONG={counts['LONG']}  "
          f"SHORT={counts['SHORT']}")

    hdr = (f"{'portal':>6} {'roomA':>5} {'roomB':>5} "
           f"{'center':>26} {'edge ft':>8} {'alt ft':>8} {'detour ft':>9} "
           f"{'class':<5} {'compA/compB':>11}  doors")
    print()
    print(hdr)
    print("-" * len(hdr))
    shown = 0
    for p, cls, detour, alt, ca, cb in rows:
        interesting = (args.full_table or cls != "SHORT"
                       or p.idx in portal_doors)
        if not interesting:
            continue
        shown += 1
        c = f"({p.center[0]:.1f},{p.center[1]:.1f},{p.center[2]:.1f})"
        altS = "-" if math.isinf(alt) else f"{alt:8.1f}"
        detS = "inf" if math.isinf(detour) else f"{detour:9.1f}"
        compS = f"{ca}/{cb}" if ca is not None else "-"
        doorS = ",".join(str(d) for d in portal_doors.get(p.idx, [])) or "-"
        print(f"{p.idx:>6} {p.a:>5} {p.b:>5} {c:>26} {p.weight:8.1f} "
              f"{altS:>8} {detS:>9} {cls:<5} {compS:>11}  {doorS}")
    print(f"({shown} of {len(rows)} portal rows shown"
          f"{'' if args.full_table else '; SHORT non-door rows elided'})")

    if doors:
        print()
        print("PER-DOOR SUMMARY (from [DOOR_PORTAL_MAP] engine lines; "
              "class suffixed '~' = proxy match, nearest portal just past "
              "the engine's 3 ft rule)")
        dh = (f"{'door':>6} {'portal':>6} {'roomA':>5} {'roomB':>5} "
              f"{'match ft':>8} {'detour ft':>9} {'class':<6} {'compA/compB':>11}")
        print(dh)
        print("-" * len(dh))
        for did in sorted(doors):
            pidx = door_portal.get(did)
            if pidx is None:
                print(f"{did:>6} {'-':>6} {'-':>5} {'-':>5} "
                      f"{doors[did]['dist'] if doors[did]['dist'] is not None else -1.0:8.2f} "
                      f"{'-':>9} {'UNMAPPED':<6}")
                continue
            p, cls, detour, alt, ca, cb = rows[pidx]
            if doors[did]["proxy"]:
                cls = cls + "~"
            detS = "inf" if math.isinf(detour) else f"{detour:9.1f}"
            compS = f"{ca}/{cb}" if ca is not None else "-"
            print(f"{did:>6} {pidx:>6} {p.a:>5} {p.b:>5} "
                  f"{doors[did]['dist']:8.2f} {detS:>9} {cls:<6} {compS:>11}")


if __name__ == "__main__":
    main()
