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
"""Room/portal graph (ROOM_DB) distribution analysis across missions.

Runs the `darknessHeadless <mis> room_graph` verb on every .mis file in a
directory and prints per-level + aggregate distributions of the ORIGINAL
game's room graph: node degrees, intra-room portal-to-portal hop distances
(the level compiler's precomputed hop-cost tables), adjacent-room
center-to-center distances, and per-room span proxies.

Purpose: ground the probe-graph single-edge visRange cap (see
AudioService::prepareProbeBakeParams) in the actual distribution of
room-scale hops instead of a worst-case clamp. The per-cap coverage table
at the end answers "what fraction of the original graph's hops does a
given edge cap cover?".

Usage:
  python3 analysis/room_graph_stats.py <levels_dir> [--headless BIN]

  <levels_dir>   path to a directory of .mis files (searched non-recursively,
                 case-insensitive extension match)
  --headless     path to the darknessHeadless binary
                 (default: build/default/src/main/darknessHeadless, resolved
                 relative to the repository root, i.e. this script's parent
                 directory)
"""

import argparse
import math
import os
import subprocess
import sys

# Candidate single-edge caps (feet) to evaluate coverage for. 400 is the
# current bake ceiling; the hypothesis is that room-scale hops are far
# shorter and a much lower cap covers nearly everything.
CAPS_FT = [60, 80, 100, 120, 150, 200, 400]

# Distance histogram bin edges (feet). Final bin is open-ended.
DIST_BINS = [0, 10, 20, 30, 40, 50, 60, 80, 100, 120, 150, 200, 300, 400]

HIST_BAR_WIDTH = 50


def percentile(sorted_vals, p):
    """Nearest-rank percentile on an already-sorted list."""
    if not sorted_vals:
        return float("nan")
    k = max(1, math.ceil(p / 100.0 * len(sorted_vals)))
    return sorted_vals[min(k, len(sorted_vals)) - 1]


def pctl_row(vals, pcts):
    s = sorted(vals)
    return [percentile(s, p) for p in pcts] + [s[-1] if s else float("nan")]


def fmt(v, w=7):
    if isinstance(v, float) and math.isnan(v):
        return "-".rjust(w)
    return f"{v:.1f}".rjust(w) if isinstance(v, float) else str(v).rjust(w)


def text_histogram(vals, bins, label_unit="ft"):
    """Print a text histogram of vals over the given bin edges."""
    counts = [0] * (len(bins))  # bins[i]..bins[i+1], last is >= bins[-1]
    for v in vals:
        placed = False
        for i in range(len(bins) - 1):
            if bins[i] <= v < bins[i + 1]:
                counts[i] += 1
                placed = True
                break
        if not placed:
            counts[-1] += 1
    peak = max(counts) if counts else 1
    total = len(vals) or 1
    cum = 0
    lines = []
    for i, c in enumerate(counts):
        if i < len(bins) - 1:
            rng = f"{bins[i]:>4}-{bins[i+1]:<4}"
        else:
            rng = f">={bins[-1]:<7}"
        cum += c
        bar = "#" * int(round(HIST_BAR_WIDTH * c / peak)) if peak else ""
        lines.append(
            f"  {rng} {label_unit} |{bar:<{HIST_BAR_WIDTH}}| "
            f"{c:>7}  {100.0 * c / total:5.1f}%  cum {100.0 * cum / total:5.1f}%"
        )
    return "\n".join(lines)


def degree_histogram(degrees):
    """Integer-bucket histogram for node degrees."""
    if not degrees:
        return "  (no data)"
    top = max(degrees)
    counts = {}
    for d in degrees:
        counts[d] = counts.get(d, 0) + 1
    peak = max(counts.values())
    total = len(degrees)
    lines = []
    cum = 0
    for d in range(0, top + 1):
        c = counts.get(d, 0)
        if c == 0 and d > 12:
            continue  # sparse tail: only print non-empty buckets past 12
        cum += c
        bar = "#" * int(round(HIST_BAR_WIDTH * c / peak))
        lines.append(
            f"  deg {d:>3} |{bar:<{HIST_BAR_WIDTH}}| "
            f"{c:>6}  {100.0 * c / total:5.1f}%  cum {100.0 * cum / total:5.1f}%"
        )
    return "\n".join(lines)


class LevelData:
    def __init__(self, name):
        self.name = name
        self.rooms = {}          # roomID -> (x, y, z, degree)
        self.portals_directed = []  # (near, far, x, y, z)
        self.pdists = []         # (roomID, i, j, dist)
        self.summary = {}
        self.anomalies = []

    def parse_line(self, line):
        parts = line.split()
        if not parts:
            return
        tag = parts[0]
        if tag == "ROOM" and len(parts) == 6:
            rid = int(parts[1])
            self.rooms[rid] = (
                float(parts[2]), float(parts[3]), float(parts[4]),
                int(parts[5]),
            )
        elif tag == "PORTAL" and len(parts) == 6:
            self.portals_directed.append(
                (int(parts[1]), int(parts[2]),
                 float(parts[3]), float(parts[4]), float(parts[5]))
            )
        elif tag == "PDIST" and len(parts) == 5:
            self.pdists.append(
                (int(parts[1]), int(parts[2]), int(parts[3]), float(parts[4]))
            )
        elif tag == "ROOMGRAPH_SUMMARY":
            for kv in parts[1:]:
                k, _, v = kv.partition("=")
                self.summary[k] = int(v)

    # ---- derived metrics ----

    def degrees(self):
        return [r[3] for r in self.rooms.values()]

    def adjacent_pairs(self):
        """Unique unordered adjacent room-ID pairs (back-links removed)."""
        pairs = set()
        for near, far, _x, _y, _z in self.portals_directed:
            if far < 0:
                self.anomalies.append(f"portal from room {near} has no far room")
                continue
            if near == far:
                self.anomalies.append(f"self-portal on room {near}")
                continue
            pairs.add((min(near, far), max(near, far)))
        return pairs

    def adjacent_center_dists(self):
        out = []
        for a, b in self.adjacent_pairs():
            ra, rb = self.rooms.get(a), self.rooms.get(b)
            if ra is None or rb is None:
                self.anomalies.append(f"adjacent pair ({a},{b}) references unknown room")
                continue
            out.append(math.dist(ra[:3], rb[:3]))
        return out

    def intra_room_hops(self):
        """All precomputed intra-room portal-to-portal distances (> 0)."""
        return [d for (_r, _i, _j, d) in self.pdists if d > 0.0]

    def zero_hops(self):
        return sum(1 for (_r, _i, _j, d) in self.pdists if d <= 0.0)

    def room_spans(self):
        """Per-room max intra-room portal distance (rooms with >= 2 portals)."""
        spans = {}
        for rid, _i, _j, d in self.pdists:
            if d > spans.get(rid, 0.0):
                spans[rid] = d
        return list(spans.values())

    def edge_count(self):
        n = len(self.portals_directed)
        if n % 2 != 0:
            self.anomalies.append(f"odd directed-portal count {n} (unpaired back-link)")
        return n // 2


def run_level(headless, mis_path, repo_root):
    lvl = LevelData(os.path.splitext(os.path.basename(mis_path))[0].upper())
    proc = subprocess.run(
        [headless, mis_path, "room_graph"],
        capture_output=True, text=True, cwd=repo_root,
    )
    for line in proc.stdout.splitlines():
        lvl.parse_line(line)
    if "rooms" not in lvl.summary:
        err_tail = "\n".join(proc.stderr.splitlines()[-5:])
        print(f"  !! {lvl.name}: no ROOMGRAPH_SUMMARY (exit={proc.returncode})")
        if err_tail:
            print("     stderr tail:\n" + err_tail)
        return None
    return lvl


def main():
    ap = argparse.ArgumentParser(
        description="ROOM_DB room/portal graph distribution analysis "
                    "across a directory of .mis files."
    )
    ap.add_argument("levels_dir", help="path to a directory of .mis files")
    ap.add_argument("--headless", default=None,
                    help="path to darknessHeadless binary")
    args = ap.parse_args()

    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    headless = args.headless or os.path.join(
        repo_root, "build", "default", "src", "main", "darknessHeadless")
    if not os.path.isfile(headless):
        sys.exit(f"headless binary not found: {headless} (use --headless)")
    if not os.path.isdir(args.levels_dir):
        sys.exit(f"not a directory: {args.levels_dir}")

    mis_files = sorted(
        os.path.join(args.levels_dir, f)
        for f in os.listdir(args.levels_dir)
        if f.lower().endswith(".mis")
    )
    if not mis_files:
        sys.exit(f"no .mis files found in {args.levels_dir}")

    levels = []
    for mp in mis_files:
        lvl = run_level(headless, mp, repo_root)
        if lvl is not None:
            levels.append(lvl)

    if not levels:
        sys.exit("no level produced usable room_graph output")

    # ---------------- per-level table ----------------
    print()
    print("=" * 118)
    print("PER-LEVEL ROOM GRAPH SUMMARY")
    print("=" * 118)
    hdr = (f"{'level':<10} {'rooms':>6} {'edges':>6} "
           f"{'deg p50':>8} {'deg max':>8} "
           f"{'hop p50':>8} {'hop p90':>8} {'hop p99':>8} {'hop max':>8} "
           f"{'adj p50':>8} {'adj max':>8} {'span p90':>9} {'span max':>9}")
    print(hdr)
    print("-" * len(hdr))
    for lvl in levels:
        degs = sorted(lvl.degrees())
        hops = sorted(lvl.intra_room_hops())
        adjs = sorted(lvl.adjacent_center_dists())
        spans = sorted(lvl.room_spans())
        print(f"{lvl.name:<10} {len(lvl.rooms):>6} {lvl.edge_count():>6} "
              f"{fmt(float(percentile(degs, 50)), 8)} {fmt(float(degs[-1]) if degs else float('nan'), 8)} "
              f"{fmt(percentile(hops, 50), 8)} {fmt(percentile(hops, 90), 8)} "
              f"{fmt(percentile(hops, 99), 8)} {fmt(hops[-1] if hops else float('nan'), 8)} "
              f"{fmt(percentile(adjs, 50), 8)} {fmt(adjs[-1] if adjs else float('nan'), 8)} "
              f"{fmt(percentile(spans, 90), 9)} {fmt(spans[-1] if spans else float('nan'), 9)}")

    # ---------------- anomalies ----------------
    print()
    print("ANOMALIES / DATA-ACCESS NOTES")
    any_anom = False
    for lvl in levels:
        s = lvl.summary
        notes = []
        if s.get("zero_portal_rooms"):
            notes.append(f"{s['zero_portal_rooms']} zero-portal rooms")
        if s.get("null_portals"):
            notes.append(f"{s['null_portals']} null portals")
        if s.get("null_far_rooms"):
            notes.append(f"{s['null_far_rooms']} null far rooms")
        if s.get("zero_pdist_pairs"):
            notes.append(f"{s['zero_pdist_pairs']} zero portal-dist pairs "
                         f"(missing hop-table entries, excluded from stats)")
        notes.extend(sorted(set(lvl.anomalies)))
        if notes:
            any_anom = True
            print(f"  {lvl.name}: " + "; ".join(notes))
    if not any_anom:
        print("  (none)")

    # ---------------- aggregates ----------------
    all_degs = sorted(d for lvl in levels for d in lvl.degrees())
    all_hops = sorted(h for lvl in levels for h in lvl.intra_room_hops())
    all_adjs = sorted(a for lvl in levels for a in lvl.adjacent_center_dists())
    all_spans = sorted(s for lvl in levels for s in lvl.room_spans())
    total_rooms = sum(len(lvl.rooms) for lvl in levels)
    total_edges = sum(lvl.edge_count() for lvl in levels)

    print()
    print("=" * 118)
    print(f"AGGREGATE ({len(levels)} levels): "
          f"{total_rooms} rooms, {total_edges} portal links (undirected), "
          f"{len(all_hops)} intra-room hop samples, "
          f"{len(all_adjs)} adjacent-room pairs")
    print("=" * 118)

    pcts = [50, 90, 95, 99]
    print()
    print(f"{'metric':<44} {'n':>7} " +
          " ".join(f"{'p' + str(p):>8}" for p in pcts) + f" {'max':>8}")
    rows = [
        ("node degree (portals per room)", [float(d) for d in all_degs]),
        ("intra-room portal-to-portal hop (ft)", all_hops),
        ("adjacent-room center-to-center (ft)", all_adjs),
        ("room span proxy: max intra-room hop (ft)", all_spans),
    ]
    for label, vals in rows:
        r = pctl_row(vals, pcts)
        print(f"{label:<44} {len(vals):>7} " + " ".join(fmt(v, 8) for v in r))

    print()
    print("NODE DEGREE HISTOGRAM (aggregate)")
    print(degree_histogram(all_degs))

    print()
    print("INTRA-ROOM PORTAL-TO-PORTAL HOP DISTANCE HISTOGRAM (aggregate)")
    print(text_histogram(all_hops, DIST_BINS))

    print()
    print("ADJACENT-ROOM CENTER-TO-CENTER DISTANCE HISTOGRAM (aggregate)")
    print(text_histogram(all_adjs, DIST_BINS))

    # ---------------- per-cap coverage ----------------
    print()
    print("EDGE-CAP COVERAGE (aggregate; % of samples <= cap)")
    print(f"{'cap (ft)':>9} {'intra-room hops':>17} {'adjacent centers':>18} "
          f"{'room spans':>12}")
    for cap in CAPS_FT:
        h = 100.0 * sum(1 for v in all_hops if v <= cap) / (len(all_hops) or 1)
        a = 100.0 * sum(1 for v in all_adjs if v <= cap) / (len(all_adjs) or 1)
        s = 100.0 * sum(1 for v in all_spans if v <= cap) / (len(all_spans) or 1)
        print(f"{cap:>9} {h:>16.2f}% {a:>17.2f}% {s:>11.2f}%")

    # ---------------- outliers ----------------
    print()
    print("TOP 15 LONGEST INTRA-ROOM HOPS (level, room, ft)")
    hop_recs = [(lvl.name, r, d)
                for lvl in levels for (r, _i, _j, d) in lvl.pdists if d > 0.0]
    for name, room, d in sorted(hop_recs, key=lambda t: -t[2])[:15]:
        print(f"  {name:<10} room {room:<5} {d:8.1f}")

    print()
    print("TOP 10 LONGEST ADJACENT-ROOM CENTER DISTANCES (level, roomA-roomB, ft)")
    adj_recs = []
    for lvl in levels:
        for a, b in lvl.adjacent_pairs():
            ra, rb = lvl.rooms.get(a), lvl.rooms.get(b)
            if ra is None or rb is None:
                continue
            adj_recs.append((lvl.name, a, b, math.dist(ra[:3], rb[:3])))
    for name, a, b, d in sorted(adj_recs, key=lambda t: -t[3])[:10]:
        print(f"  {name:<10} rooms {a:>4}-{b:<4} {d:8.1f}")


if __name__ == "__main__":
    main()
