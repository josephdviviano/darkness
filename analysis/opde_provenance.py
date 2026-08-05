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
"""Measure how much of Darkness's source is still openDarkEngine (OPDE) code.

Regenerates the figures in .claude/NOTES.OPDE_FUNCTIONALITY.md ("Provenance"
section). Run this instead of trusting the numbers in the doc — they were wrong
once already, which is why this script exists.

    python3 analysis/opde_provenance.py [--opde ../openDarkEngine] [--markdown]

Darkness is NOT a git fork of OPDE: history begins with a fresh `init commit`
and shares no ancestry with upstream. So `git log`/`blame`/`merge-base` reveal
nothing about provenance and file comparison is the only way to measure it.

Two normalizations matter, and skipping either badly undercounts inheritance:

  1. Content: strip comments/blanks, then apply the `opde`->`darkness` rename
     that commit 0e475b0 made across the whole codebase.
  2. Filenames: OPDE prefixed files `Opde*` (OpdeService.cpp); Darkness renamed
     them `Darkness*` (DarknessService.cpp). Naive basename matching scores
     these as brand-new files.

"OPDE-derived" means >=70% similar to some OPDE file after normalization.
Attribution: openDarkEngine by Filip Volejnik et al., GPLv2-or-later. See
AUTHORS.
"""

import argparse
import difflib
import os
import re
import sys
from collections import defaultdict

EXT = (".cpp", ".h", ".inl")

# >=95% verbatim, >=70% still recognizably the same file, <35% means the name
# was reused for what is effectively new code (e.g. main/LightmapAtlas.h).
VERBATIM = 0.95
INHERITED = 0.70
REWRITTEN = 0.35


def filename_key(name):
    """Basename match key, immune to the Opde*/Darkness* file rename."""
    return name.lower().replace("opde", "").replace("darkness", "")


def collect(root):
    out = defaultdict(list)
    for dirpath, _, filenames in os.walk(root):
        for f in filenames:
            if f.endswith(EXT):
                out[filename_key(f)].append(os.path.join(dirpath, f))
    return out


def normalize(path):
    """Comment-stripped, rename-normalized code lines. Blank lines dropped."""
    try:
        text = open(path, encoding="utf-8", errors="replace").read()
    except OSError:
        return []
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    text = re.sub(r"//[^\n]*", "", text)
    lines = []
    for line in text.split("\n"):
        s = line.strip().lower()
        if not s:
            continue
        s = s.replace("opde", "darkness").replace("open dark engine", "darkness")
        lines.append(re.sub(r"\s+", " ", s))
    return lines


def raw_loc(path):
    try:
        return sum(1 for _ in open(path, encoding="utf-8", errors="replace"))
    except OSError:
        return 0


def area(path, dark_src):
    """Group a file into a reporting area (services/audio, base/file, main/...)."""
    rel = os.path.relpath(path, dark_src)
    parts = rel.split(os.sep)
    if parts[0] in ("services", "base") and len(parts) > 2:
        return parts[0] + "/" + parts[1]
    if len(parts) > 1:
        return parts[0] + "/"
    return parts[0]


def match(dark_src, opde_src):
    opde = collect(opde_src)
    rows = []
    for key, dark_paths in sorted(collect(dark_src).items()):
        for dpath in dark_paths:
            dlines = normalize(dpath)
            best = (0.0, None, 0)
            for opath in opde.get(key, []):
                olines = normalize(opath)
                if not olines and not dlines:
                    ratio = 1.0
                else:
                    ratio = difflib.SequenceMatcher(None, olines, dlines).ratio()
                if ratio > best[0]:
                    best = (ratio, opath, len(olines))
            rows.append(
                dict(
                    dark=dpath,
                    opde=best[1],
                    ratio=round(best[0], 3),
                    dnorm=len(dlines),
                    draw=raw_loc(dpath),
                )
            )
    return rows, opde


def report(rows, opde, dark_src, markdown):
    inherited = [r for r in rows if r["opde"] and r["ratio"] >= INHERITED]
    t_raw = sum(r["draw"] for r in rows)
    t_norm = sum(r["dnorm"] for r in rows)
    i_raw = sum(r["draw"] for r in inherited)
    i_norm = sum(r["dnorm"] for r in inherited)
    verbatim = sum(1 for r in inherited if r["ratio"] >= VERBATIM)

    print("=== HEADLINE ===")
    print(f"OPDE-derived raw LOC       : {i_raw:,} / {t_raw:,} = {100*i_raw/t_raw:.1f}%")
    print(f"OPDE-derived (code only)   : {i_norm:,} / {t_norm:,} = {100*i_norm/t_norm:.1f}%")
    print(f"Files with OPDE ancestor   : {len(inherited)} / {len(rows)} "
          f"= {100*len(inherited)/len(rows):.0f}%  ({verbatim} are >={VERBATIM:.0%} verbatim)")

    # "Dropped entirely" = no Darkness successor at ANY similarity, i.e. the file
    # is simply gone (nearly all of these are Ogre-facing: render, draw, worldrep,
    # scenemanager, gui, input, python bindings). Files that DO have a successor but
    # were rewritten past recognition (RoomService.cpp at 0.31) are not "dropped" —
    # they show up under NAME-REUSE TRAPS instead.
    survived = {r["opde"] for r in rows if r["opde"]}
    dropped = [p for paths in opde.values() for p in paths if p not in survived]
    d_norm = sum(len(normalize(p)) for p in dropped)
    total_opde = sum(len(paths) for paths in opde.values())
    print(f"OPDE files dropped entirely: {len(dropped)} / {total_opde} "
          f"(~{d_norm:,} code LOC)")

    agg = defaultdict(lambda: defaultdict(int))
    for r in rows:
        a = area(r["dark"], dark_src)
        agg[a]["files"] += 1
        agg[a]["raw"] += r["draw"]
        if r["opde"] and r["ratio"] >= INHERITED:
            agg[a]["inh"] += r["draw"]

    print("\n=== PER-AREA ===")
    if markdown:
        print("| Area | Files | Raw LOC | OPDE-derived | % OPDE |")
        print("|---|---|---|---|---|")
    for a in sorted(agg, key=lambda x: -agg[x]["inh"]):
        v = agg[a]
        pct = 100 * v["inh"] / v["raw"] if v["raw"] else 0
        if markdown:
            print(f"| `{a}` | {v['files']} | {v['raw']:,} | {v['inh']:,} | {pct:.0f}% |")
        else:
            print(f"{a:<28}{v['files']:>5}{v['raw']:>10,}{v['inh']:>10,}{pct:>7.0f}%")

    # Name-reuse traps: OPDE filename retained, code is effectively new.
    traps = [r for r in rows if r["opde"] and r["ratio"] < REWRITTEN]
    if traps:
        print("\n=== NAME-REUSE TRAPS (OPDE name, but NOT OPDE code) ===")
        for r in sorted(traps, key=lambda x: -x["dnorm"]):
            print(f"  {r['ratio']:.2f}  {os.path.relpath(r['dark'], dark_src)}")

    # Structural dependency: LOC share understates how load-bearing OPDE is.
    inh_headers = {os.path.basename(r["dark"]) for r in inherited}
    new_files = [r["dark"] for r in rows if not r["opde"]]
    hits = defaultdict(int)
    touching = 0
    for f in new_files:
        try:
            text = open(f, encoding="utf-8", errors="replace").read()
        except OSError:
            continue
        incs = {os.path.basename(i)
                for i in re.findall(r'#include\s+[<"]([^">]+)[">]', text)}
        common = incs & inh_headers
        if common:
            touching += 1
            for c in common:
                hits[c] += 1
    if new_files:
        print(f"\n=== STRUCTURAL DEPENDENCY ===")
        print(f"New Darkness files including >=1 OPDE-derived header: "
              f"{touching} / {len(new_files)} = {100*touching/len(new_files):.0f}%")
        for h, n in sorted(hits.items(), key=lambda x: -x[1])[:10]:
            print(f"  {n:>3}  {h}")


def main():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--opde", default=os.path.join(here, "..", "openDarkEngine"),
                    help="path to the pre-fork openDarkEngine checkout")
    ap.add_argument("--dark", default=here, help="path to the darkness checkout")
    ap.add_argument("--markdown", action="store_true",
                    help="emit the per-area table as markdown for pasting into notes")
    args = ap.parse_args()

    dark_src = os.path.join(args.dark, "src")
    opde_src = os.path.join(args.opde, "src")
    for p, label in ((dark_src, "darkness"), (opde_src, "openDarkEngine")):
        if not os.path.isdir(p):
            sys.exit(f"error: {label} src not found at {p}")

    rows, opde = match(dark_src, opde_src)
    report(rows, opde, dark_src, args.markdown)


if __name__ == "__main__":
    main()
