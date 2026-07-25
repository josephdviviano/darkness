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
"""Attribute pathing-probe density (and cost) to the placement RULE that
created each probe.

Consumes the two per-mission sidecars written by the §55 Phase 0
instrumentation (PLAN.PATHING_DESIGN.md):

  <mission>.probes.pathing.positions.csv
      index,x,y,z,radiusFt,purpose,origin,provenanceId,absorbed
      One row per BAKED probe. `origin` is the creating rule
      (PathingProbeOrigin); `provenanceId` is the source feature's identity
      (portalID / region id / emitter idx / -1); `absorbed` counts how many
      candidates the proximity dedup merged into this probe.

  <mission>.probes.pathing.edges.csv          (optional, from level-load)
      i,j,lengthFt
      The adjacency graph under the baked visRange — pathing solve cost
      scales with reachable edges, so per-probe degree is the cost currency.

The positions sidecar records only SURVIVORS, so it cannot show how many
candidates a rule emitted-then-lost to dedup. The engine's [DENSITY_CENSUS]
stderr block (emitted -> survived per origin) carries that; this script
attributes the density that DID survive, plus the runtime cost each rule
imposes via edge degree.

    python3 analysis/probe_density_census.py [--baked DIR] [--mission miss9]

Density drivers this surfaces (see §55b):
  * which RULE contributes the most probes (count / %),
  * which rule concentrates dedup burden (absorbed),
  * near-coincident stacking per rule (<1 ft neighbor fraction),
  * runaway features — one provenanceId spawning a probe stack,
  * edge degree per rule — the rules whose probes cost the most to solve.

stdlib only (matches the other analysis/ scripts): no numpy/pandas.
"""
import argparse
import csv
import glob
import math
import os
from collections import defaultdict

DEFAULT_BAKED = os.path.expanduser("~/darkness/thief2/baked_probes")

# Origins the engine only emits AFTER the dedup snapshot (they bypass dedup);
# every other origin is a pre-dedup aperture/door/portal/emitter rule. Kept
# here only for the section header note — the tables read the CSV verbatim.
HUBFILL_ORIGINS = {"region_seed", "fill_site", "stitch_stone", "joiner_stone"}

# Canonical origin print order (Unknown/legacy first so a stray row is loud).
ORIGIN_ORDER = [
    "unknown", "(pre-census)",
    "aperture_single", "door_flank", "door_fallback_flank",
    "throat_pair", "hatch_pair", "emitter_mirror",
    "fill_site", "region_seed", "stitch_stone", "joiner_stone",
]


class Probe:
    __slots__ = ("x", "y", "z", "radius", "purpose", "origin",
                 "prov", "absorbed", "degree")

    def __init__(self, x, y, z, radius, purpose, origin, prov, absorbed):
        self.x, self.y, self.z = x, y, z
        self.radius = radius
        self.purpose = purpose
        self.origin = origin
        self.prov = prov
        self.absorbed = absorbed
        self.degree = 0   # filled from the edges sidecar when present


def _load_positions(path):
    """Parse a positions sidecar. Handles both the 9-column enriched form and
    the legacy 5-column (or 4-column) form — legacy probes get origin
    '(pre-census)' so they are visibly un-attributed rather than silently
    lumped into a real rule."""
    probes = []
    with open(path) as f:
        for row in csv.reader(l for l in f if not l.startswith("#")):
            if not row or row[0] == "index":
                continue
            x, y, z = float(row[1]), float(row[2]), float(row[3])
            radius = float(row[4]) if len(row) > 4 else float("nan")
            if len(row) >= 9:
                purpose, origin = row[5], row[6]
                prov, absorbed = int(row[7]), int(row[8])
            else:
                purpose, origin, prov, absorbed = "(pre-census)", \
                    "(pre-census)", -1, 0
            probes.append(Probe(x, y, z, radius, purpose, origin,
                                prov, absorbed))
    return probes


def _load_edges(path, probes):
    """Fill probe.degree from an edges sidecar. Returns (edge_count, lengths)
    or (None, None) when the sidecar is absent."""
    if not os.path.exists(path):
        return None, None
    lengths = []
    count = 0
    n = len(probes)
    with open(path) as f:
        for row in csv.reader(l for l in f if not l.startswith("#")):
            if not row or row[0] == "i":
                continue
            i, j = int(row[0]), int(row[1])
            if 0 <= i < n and 0 <= j < n:
                probes[i].degree += 1
                probes[j].degree += 1
                count += 1
                if len(row) > 2:
                    lengths.append(float(row[2]))
    return count, lengths


def _dist(a, b):
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def _pct(values, p):
    if not values:
        return float("nan")
    s = sorted(values)
    i = min(len(s) - 1, int(p / 100.0 * len(s)))
    return s[i]


def _nearest_neighbors(probes):
    """Brute-force nearest-neighbor distance per probe. O(N^2) — fine for the
    few-hundred-probe missions; matches density_survey.py."""
    nnd = [1e9] * len(probes)
    for i, a in enumerate(probes):
        best = 1e9
        for j, b in enumerate(probes):
            if i == j:
                continue
            d = _dist(a, b)
            if d < best:
                best = d
        nnd[i] = best
    return nnd


def _fmt(v, spec="{:.1f}"):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "-"
    return spec.format(v)


def census_mission(name, probes, edge_count, edge_lengths):
    n = len(probes)
    nnd = _nearest_neighbors(probes)
    by_origin = defaultdict(list)
    for idx, p in enumerate(probes):
        by_origin[p.origin].append(idx)

    have_edges = edge_count is not None

    print(f"\n=== {name} ===  N={n} probes"
          + (f", edges={edge_count}" if have_edges else ", edges=(no sidecar)"))

    hdr = (f"  {'origin':<20} {'count':>5} {'%':>5} {'absorb':>6} "
           f"{'rad50':>6} {'rad90':>6} {'nnd10':>6} {'nnd50':>6} "
           f"{'<1ft':>5}")
    if have_edges:
        hdr += f" {'deg50':>6} {'deg90':>6} {'degMx':>6}"
    print(hdr)

    seen = set()
    ordered = [o for o in ORIGIN_ORDER if o in by_origin] + \
              [o for o in by_origin if o not in ORIGIN_ORDER]
    for origin in ordered:
        if origin in seen:
            continue
        seen.add(origin)
        idxs = by_origin[origin]
        radii = [probes[i].radius for i in idxs]
        onnd = [nnd[i] for i in idxs]
        absorbed = sum(probes[i].absorbed for i in idxs)
        sub1 = sum(1 for i in idxs if nnd[i] < 1.0)
        line = (f"  {origin:<20} {len(idxs):>5} "
                f"{100.0*len(idxs)/n:>4.0f}% {absorbed:>6} "
                f"{_fmt(_pct(radii,50)):>6} {_fmt(_pct(radii,90)):>6} "
                f"{_fmt(_pct(onnd,10),'{:.2f}'):>6} "
                f"{_fmt(_pct(onnd,50),'{:.2f}'):>6} {sub1:>5}")
        if have_edges:
            degs = [probes[i].degree for i in idxs]
            line += (f" {_fmt(_pct(degs,50),'{:.0f}'):>6} "
                     f"{_fmt(_pct(degs,90),'{:.0f}'):>6} "
                     f"{_fmt(max(degs) if degs else None,'{:.0f}'):>6}")
        print(line)

    # ── Runaway features: one provenanceId spawning a probe stack ──
    # A single door/portal/region should map to a small, bounded probe
    # count (a door -> 2 flanks, an aperture -> 1-2). A high count under
    # one provenanceId is a compound-feature stacking bug (§55b driver 2).
    stacks = defaultdict(list)   # (origin, prov) -> [idx]
    for idx, p in enumerate(probes):
        if p.prov >= 0:          # -1 = no identity (joiner stones, legacy)
            stacks[(p.origin, p.prov)].append(idx)
    worst = sorted(stacks.items(), key=lambda kv: -len(kv[1]))[:6]
    worst = [(k, v) for k, v in worst if len(v) > 2]
    if worst:
        print("  runaway features (>2 probes sharing one provenanceId):")
        for (origin, prov), idxs in worst:
            # Centroid of the stack, for eyeballing against a level map.
            cx = sum(probes[i].x for i in idxs) / len(idxs)
            cy = sum(probes[i].y for i in idxs) / len(idxs)
            cz = sum(probes[i].z for i in idxs) / len(idxs)
            print(f"    {origin:<20} id={prov:<6} {len(idxs)} probes "
                  f"near ({cx:.0f},{cy:.0f},{cz:.0f})")

    if have_edges and edge_lengths:
        print(f"  edge length ft: p50={_pct(edge_lengths,50):.1f} "
              f"p90={_pct(edge_lengths,90):.1f} "
              f"max={max(edge_lengths):.1f}")

    return nnd


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--baked", default=DEFAULT_BAKED,
                    help=f"baked_probes dir (default: {DEFAULT_BAKED})")
    ap.add_argument("--mission", default=None,
                    help="restrict to one mission name (e.g. miss9)")
    args = ap.parse_args()

    pattern = os.path.join(args.baked, "*.probes.pathing.positions.csv")
    paths = sorted(glob.glob(pattern))
    if args.mission:
        paths = [p for p in paths
                 if os.path.basename(p).split(".")[0] == args.mission]
    if not paths:
        print(f"No positions sidecars found under {args.baked} "
              f"(pattern {os.path.basename(pattern)}).")
        print("Bake with --force-pathing-bake to emit the enriched sidecars.")
        return

    agg_count = defaultdict(int)
    agg_absorbed = defaultdict(int)
    agg_sub1 = defaultdict(int)
    agg_deg = defaultdict(int)     # total edge endpoints owned per origin
    grand_total = 0
    grand_deg = 0                  # 2 * total edges (both endpoints counted)
    n_with_edges = 0

    for path in paths:
        name = os.path.basename(path).split(".")[0]
        probes = _load_positions(path)
        if len(probes) < 2:
            continue
        edge_path = path.replace(".positions.csv", ".edges.csv")
        edge_count, edge_lengths = _load_edges(edge_path, probes)
        nnd = census_mission(name, probes, edge_count, edge_lengths)
        grand_total += len(probes)
        if edge_count is not None:
            n_with_edges += 1
        for idx, p in enumerate(probes):
            agg_count[p.origin] += 1
            agg_absorbed[p.origin] += p.absorbed
            if nnd[idx] < 1.0:
                agg_sub1[p.origin] += 1
            if edge_count is not None:
                agg_deg[p.origin] += p.degree
                grand_deg += p.degree

    if grand_total and len(paths) > 1:
        # Game-wide attribution. `cnt%` = density share (probe COUNT); `edge%`
        # = pathing-COST share (fraction of all graph edge endpoints owned by
        # this rule — solve cost scales with reachable edges, so this is the
        # runtime-cost lever ranking). `absorb` = dedup burden concentrated;
        # `<1ft` = near-coincident stacking.
        print(f"\n=== ALL MISSIONS ===  {grand_total} probes across "
              f"{len(paths)} missions ({n_with_edges} with edge sidecars)")
        print(f"  {'origin':<20} {'count':>6} {'cnt%':>5} {'edge%':>6} "
              f"{'absorb':>7} {'<1ft':>6}")
        ordered = [o for o in ORIGIN_ORDER if o in agg_count] + \
                  [o for o in agg_count if o not in ORIGIN_ORDER]
        seen = set()
        for origin in ordered:
            if origin in seen:
                continue
            seen.add(origin)
            edge_share = (f"{100.0*agg_deg[origin]/grand_deg:>5.0f}%"
                          if grand_deg else "    -")
            print(f"  {origin:<20} {agg_count[origin]:>6} "
                  f"{100.0*agg_count[origin]/grand_total:>4.0f}% {edge_share:>6} "
                  f"{agg_absorbed[origin]:>7} {agg_sub1[origin]:>6}")


if __name__ == "__main__":
    main()
