#!/usr/bin/env python3
# ******************************************************************************
#
#    This file is part of the darkness project
#    Copyright (C) 2026 darkness project contributors
#
#    This program is free software; you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation; either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program; if not, see <http://www.gnu.org/licenses/>.
#
# ******************************************************************************
#
# tools/perf_diff.py — compare two `audio_perf.jsonl` run artifacts.
#
# Companion to PLAN.AUDIO_PROFILING.md §1.3. The C++ side (Agent A) emits one
# `event:"run.meta"` header followed by N `event:"perf.window"` records into
# `<run_dir>/audio_perf.jsonl`. This script:
#   1. Reads the meta block from both runs and prints knob deltas.
#   2. Aggregates each percentile field across all windows using an n-weighted
#      mean (so a 5-second window with n=120 callbacks contributes more weight
#      than a window with n=10).  This is the right summary for percentile
#      values that come from histograms — averaging un-weighted p50s would let
#      a sparse window distort the run summary.
#   3. With `--plot`, writes `compare.png` overlaying p95 trajectories.
#
# Standard-library-only by default. matplotlib only loaded under `--plot`.

import argparse
import json
import sys
from pathlib import Path

# ----------------------------------------------------------------------------
# JSONL schema assumptions (mirrors PLAN.AUDIO_PROFILING §1.1-1.2).
#
# Line 1: {"event":"run.meta", "audio": {...full audio.* knob snapshot...},
#          "git_commit": "...", "mission": "...", "started_at_utc": "...", ...}
#
# Each window: {"event":"perf.window", "wall_clock_s": <float seconds since
#               run start>, "stages": {
#                 "<stage_name>": {"n": <int>, "p50": <ms>, "p95": <ms>,
#                                  "p99": <ms>, ...},
#                 ...}}
#
# The aggregator is defensive: missing percentile fields are silently skipped
# for that window (but the per-run loader warns once per missing key class).
# ----------------------------------------------------------------------------

PERCENTILE_KEYS = ("p50", "p95", "p99")


def iter_jsonl(path: Path):
    """Stream a JSONL file line-by-line. Bad lines emit a single stderr warning
    and are skipped — no silent fallback."""
    with path.open("r", encoding="utf-8") as fh:
        for lineno, raw in enumerate(fh, start=1):
            raw = raw.strip()
            if not raw:
                continue
            try:
                yield lineno, json.loads(raw)
            except json.JSONDecodeError:
                sys.stderr.write(
                    f"[WARN] skipping malformed JSONL line {lineno} in {path}\n"
                )


def load_run(run_dir: Path):
    """Load `<run_dir>/audio_perf.jsonl`. Returns (meta_dict, windows_list).
    Raises SystemExit (non-zero) on missing or unreadable files."""
    jsonl = run_dir / "audio_perf.jsonl"
    if not jsonl.is_file():
        sys.stderr.write(f"[ERROR] no audio_perf.jsonl in {run_dir}\n")
        sys.exit(2)

    meta = None
    windows = []
    for lineno, record in iter_jsonl(jsonl):
        ev = record.get("event")
        if ev == "run.meta":
            if meta is not None:
                sys.stderr.write(
                    f"[WARN] duplicate run.meta at line {lineno} in {jsonl}; "
                    "keeping the first\n"
                )
                continue
            meta = record
        elif ev == "perf.window":
            windows.append(record)
        # Other events (refl_evict, beat, refl_skip) are tracked elsewhere and
        # intentionally ignored here.

    if meta is None:
        sys.stderr.write(
            f"[WARN] no run.meta record found in {jsonl}; knob diff will be empty\n"
        )
        meta = {}

    return meta, windows


# ----------------------------------------------------------------------------
# Knob diff
# ----------------------------------------------------------------------------

def flatten(obj, prefix=""):
    """Yield (dotted_path, leaf_value) for every leaf in a nested dict.

    Lists are treated as leaves (compared by repr); the audio knob tree is
    expected to be scalar-leafed in practice, but list comparison still works."""
    if isinstance(obj, dict):
        for key, val in obj.items():
            sub = f"{prefix}.{key}" if prefix else key
            yield from flatten(val, sub)
    else:
        yield prefix, obj


def diff_knobs(meta_a: dict, meta_b: dict):
    audio_a = meta_a.get("audio", {}) or {}
    audio_b = meta_b.get("audio", {}) or {}
    flat_a = dict(flatten(audio_a, "audio"))
    flat_b = dict(flatten(audio_b, "audio"))
    all_keys = sorted(set(flat_a) | set(flat_b))
    deltas = []
    for k in all_keys:
        va = flat_a.get(k, "<absent>")
        vb = flat_b.get(k, "<absent>")
        if va != vb:
            deltas.append((k, va, vb))
    return deltas


# ----------------------------------------------------------------------------
# Per-stage n-weighted percentile aggregation.
#
# For each stage name (e.g. "callback"), for each percentile key (p50/p95/p99),
# compute weighted_mean = sum(n_i * pX_i) / sum(n_i) across all windows that
# carry the stage. This matches how the C++ side already drains histograms:
# each 5s window's percentile is computed over its own n samples, and our
# aggregate must remain proportional to total sample count.
# ----------------------------------------------------------------------------

def aggregate(windows):
    """Return {stage_name: {pkey: (weighted_mean_ms, total_n)}}."""
    accum = {}  # stage -> pkey -> [sum_weighted, sum_n]
    n_only = {}  # stage -> total_n (covers stages with no percentile fields)
    for w in windows:
        stages = w.get("stages") or {}
        for stage, fields in stages.items():
            if not isinstance(fields, dict):
                continue
            n = fields.get("n")
            if not isinstance(n, (int, float)) or n <= 0:
                # Skip windows with no samples for this stage — common when a
                # subsystem (e.g. reverb worker) was idle during the window.
                continue
            n_only[stage] = n_only.get(stage, 0) + n
            for pkey in PERCENTILE_KEYS:
                val = fields.get(pkey)
                if val is None:
                    continue
                if not isinstance(val, (int, float)):
                    continue
                bucket = accum.setdefault(stage, {}).setdefault(pkey, [0.0, 0])
                bucket[0] += float(val) * n
                bucket[1] += n
    out = {}
    for stage, total_n in n_only.items():
        out[stage] = {"_total_n": total_n}
        for pkey, (wsum, nsum) in accum.get(stage, {}).items():
            if nsum > 0:
                out[stage][pkey] = (wsum / nsum, nsum)
    return out


def format_table(agg_a, agg_b):
    """Return a list of printable lines for the side-by-side percentile table."""
    all_stages = sorted(set(agg_a) | set(agg_b))
    # One row per (stage, pkey).
    rows = []
    for stage in all_stages:
        for pkey in PERCENTILE_KEYS:
            a = agg_a.get(stage, {}).get(pkey)
            b = agg_b.get(stage, {}).get(pkey)
            if a is None and b is None:
                continue
            rows.append((stage, pkey, a, b))

    if not rows:
        return ["(no percentile data found in either run)"]

    header = ("Stage", "runA (n)", "runB (n)", "Δ%")
    widths = [len(h) for h in header]
    formatted = []
    for stage, pkey, a, b in rows:
        label = f"{stage} {pkey}"
        if a is None:
            a_cell = "  ---"
        else:
            mean_a, na = a
            a_cell = f"{mean_a:6.3f} ms ({_short_count(na)})"
        if b is None:
            b_cell = "  ---"
        else:
            mean_b, nb = b
            b_cell = f"{mean_b:6.3f} ms ({_short_count(nb)})"
        if a and b and a[0] != 0:
            delta = (b[0] - a[0]) / a[0] * 100.0
            delta_cell = f"{delta:+6.1f}%"
        else:
            delta_cell = "  ---"
        formatted.append((label, a_cell, b_cell, delta_cell))
        for i, cell in enumerate((label, a_cell, b_cell, delta_cell)):
            widths[i] = max(widths[i], len(cell))

    def fmt_row(cells):
        return " | ".join(c.ljust(widths[i]) for i, c in enumerate(cells))

    out = [
        fmt_row(header),
        "-+-".join("-" * w for w in widths),
    ]
    out.extend(fmt_row(r) for r in formatted)
    return out


def _short_count(n):
    """Compact integer formatter (12345 -> '12k')."""
    if n >= 10000:
        return f"{n // 1000}k"
    return str(n)


# ----------------------------------------------------------------------------
# Optional plot
# ----------------------------------------------------------------------------

def maybe_plot(runs, out_path):
    """Overlay p95 trajectories per stage for both runs. Degrades gracefully
    when matplotlib is missing — prints a [WARN] and returns without crashing."""
    try:
        import matplotlib

        matplotlib.use("Agg")  # headless backend; never pop a window
        import matplotlib.pyplot as plt
    except ImportError:
        sys.stderr.write(
            "[WARN] matplotlib not available; skipping --plot output\n"
        )
        return

    fig, ax = plt.subplots(figsize=(12, 6))
    any_data = False
    for (label, _meta, windows) in runs:
        # Group by stage.
        by_stage = {}
        for w in windows:
            wc = w.get("wall_clock_s")
            if not isinstance(wc, (int, float)):
                continue
            stages = w.get("stages") or {}
            for stage, fields in stages.items():
                p95 = (fields or {}).get("p95")
                if not isinstance(p95, (int, float)):
                    continue
                by_stage.setdefault(stage, []).append((wc, p95))
        for stage, pts in by_stage.items():
            pts.sort()
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            ax.plot(xs, ys, label=f"{label} {stage} p95", alpha=0.7)
            any_data = True

    if not any_data:
        sys.stderr.write(
            "[WARN] no p95 trajectory data to plot; skipping write\n"
        )
        plt.close(fig)
        return

    ax.set_xlabel("Wall clock (s since run start)")
    ax.set_ylabel("p95 latency (ms)")
    ax.set_title("Per-stage p95 trajectory: runA vs runB")
    ax.legend(loc="best", fontsize="x-small")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    sys.stderr.write(f"[INFO] wrote {out_path}\n")


# ----------------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------------

def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Compare two audio_perf.jsonl run artifacts (see "
        ".claude/PLAN.AUDIO_PROFILING.md §1.3).",
    )
    parser.add_argument("run_a", type=Path, help="run-A directory (contains audio_perf.jsonl)")
    parser.add_argument("run_b", type=Path, help="run-B directory (contains audio_perf.jsonl)")
    parser.add_argument(
        "--plot",
        action="store_true",
        help="write compare.png with p95-per-stage trajectory overlay",
    )
    parser.add_argument(
        "--plot-out",
        type=Path,
        default=Path("compare.png"),
        help="path for the --plot output (default: ./compare.png)",
    )
    args = parser.parse_args(argv)

    meta_a, windows_a = load_run(args.run_a)
    meta_b, windows_b = load_run(args.run_b)

    # 1. Knob diff
    print("Knob diff (runA → runB):")
    deltas = diff_knobs(meta_a, meta_b)
    if not deltas:
        print("  (no knob differences)")
    else:
        for key, va, vb in deltas:
            print(f"  {key}: {va} → {vb}")
    print()

    # 2. Aggregated percentile table
    agg_a = aggregate(windows_a)
    agg_b = aggregate(windows_b)
    print(
        f"Aggregated percentile table (n-weighted mean across "
        f"{len(windows_a)} runA / {len(windows_b)} runB windows):"
    )
    for line in format_table(agg_a, agg_b):
        print(line)

    # 3. Optional plot
    if args.plot:
        maybe_plot(
            [("runA", meta_a, windows_a), ("runB", meta_b, windows_b)],
            args.plot_out,
        )

    return 0


if __name__ == "__main__":
    sys.exit(main())
