#!/usr/bin/env python3
# ******************************************************************************
#
#    This file is part of the darkness project
#    Copyright (C) 2026 darkness project contributors
#
#    GPLv3 — see perf_diff.py for the full notice.
#
# ******************************************************************************
#
# tools/perf_diff_test.py — self-test for perf_diff.py without external fixtures.
#
# Writes two hand-crafted JSONL artifacts into a temp dir, runs the loader +
# aggregator + knob-diff, and asserts the n-weighted mean arithmetic matches a
# direct manual computation. Designed so we can validate the diff math before
# Agent A's sink lands and produces real artifacts.

import io
import json
import sys
import tempfile
import unittest
from pathlib import Path

# Make the sibling module importable regardless of cwd.
sys.path.insert(0, str(Path(__file__).resolve().parent))

import perf_diff  # noqa: E402


def write_jsonl(path: Path, records):
    with path.open("w", encoding="utf-8") as fh:
        for rec in records:
            fh.write(json.dumps(rec) + "\n")


# Two windows:
#   - window 1: callback p95=0.5ms over n=100 samples
#   - window 2: callback p95=1.0ms over n=300 samples
# Manual n-weighted mean: (0.5*100 + 1.0*300) / 400 = 350/400 = 0.875 ms.
SAMPLE_A = [
    {
        "event": "run.meta",
        "audio": {
            "reflections": {"hybrid_transition_time": 1.0},
            "performance": {"reverb_voices": 16},
        },
        "git_commit": "abc123",
    },
    {
        "event": "perf.window",
        "wall_clock_s": 5.0,
        "stages": {
            "callback": {"n": 100, "p50": 0.3, "p95": 0.5, "p99": 0.7},
        },
    },
    {
        "event": "perf.window",
        "wall_clock_s": 10.0,
        "stages": {
            "callback": {"n": 300, "p50": 0.4, "p95": 1.0, "p99": 1.5},
        },
    },
]

# Run B changes hybrid_transition_time and reverb_voices, and has a single
# window with p95=0.7ms over n=200.
SAMPLE_B = [
    {
        "event": "run.meta",
        "audio": {
            "reflections": {"hybrid_transition_time": 0.5},
            "performance": {"reverb_voices": 24},
        },
        "git_commit": "def456",
    },
    {
        "event": "perf.window",
        "wall_clock_s": 5.0,
        "stages": {
            "callback": {"n": 200, "p50": 0.25, "p95": 0.7, "p99": 0.9},
        },
    },
]


class PerfDiffTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.run_a = Path(self.tmp.name) / "runA"
        self.run_b = Path(self.tmp.name) / "runB"
        self.run_a.mkdir()
        self.run_b.mkdir()
        write_jsonl(self.run_a / "audio_perf.jsonl", SAMPLE_A)
        write_jsonl(self.run_b / "audio_perf.jsonl", SAMPLE_B)

    def tearDown(self):
        self.tmp.cleanup()

    def test_load_run(self):
        meta, windows = perf_diff.load_run(self.run_a)
        self.assertEqual(meta.get("git_commit"), "abc123")
        self.assertEqual(len(windows), 2)

    def test_knob_diff(self):
        meta_a, _ = perf_diff.load_run(self.run_a)
        meta_b, _ = perf_diff.load_run(self.run_b)
        deltas = perf_diff.diff_knobs(meta_a, meta_b)
        keys = {d[0] for d in deltas}
        self.assertIn("audio.reflections.hybrid_transition_time", keys)
        self.assertIn("audio.performance.reverb_voices", keys)
        # Verify direction of the change is preserved.
        delta_map = {d[0]: (d[1], d[2]) for d in deltas}
        self.assertEqual(
            delta_map["audio.reflections.hybrid_transition_time"], (1.0, 0.5)
        )
        self.assertEqual(
            delta_map["audio.performance.reverb_voices"], (16, 24)
        )

    def test_aggregate_n_weighted_mean(self):
        _, windows = perf_diff.load_run(self.run_a)
        agg = perf_diff.aggregate(windows)
        # Manual: p95 = (0.5*100 + 1.0*300)/400 = 0.875; total_n = 400
        mean, n = agg["callback"]["p95"]
        self.assertAlmostEqual(mean, 0.875, places=6)
        self.assertEqual(n, 400)
        # p50 = (0.3*100 + 0.4*300)/400 = (30+120)/400 = 0.375
        mean_p50, _ = agg["callback"]["p50"]
        self.assertAlmostEqual(mean_p50, 0.375, places=6)

    def test_aggregate_single_window(self):
        _, windows = perf_diff.load_run(self.run_b)
        agg = perf_diff.aggregate(windows)
        mean, n = agg["callback"]["p95"]
        self.assertAlmostEqual(mean, 0.7, places=6)
        self.assertEqual(n, 200)

    def test_malformed_line_skipped(self):
        # Append a busted line; aggregator should warn + continue.
        with (self.run_a / "audio_perf.jsonl").open("a") as fh:
            fh.write("this is not json\n")
        captured = io.StringIO()
        real_stderr = sys.stderr
        sys.stderr = captured
        try:
            meta, windows = perf_diff.load_run(self.run_a)
        finally:
            sys.stderr = real_stderr
        self.assertEqual(len(windows), 2)
        self.assertIn("malformed JSONL", captured.getvalue())

    def test_zero_n_window_ignored(self):
        records = [
            {"event": "run.meta", "audio": {}},
            {"event": "perf.window", "stages": {"x": {"n": 0, "p50": 99.0}}},
            {"event": "perf.window", "stages": {"x": {"n": 10, "p50": 1.0}}},
        ]
        with tempfile.TemporaryDirectory() as td:
            run = Path(td) / "z"
            run.mkdir()
            write_jsonl(run / "audio_perf.jsonl", records)
            _, windows = perf_diff.load_run(run)
            agg = perf_diff.aggregate(windows)
        mean, n = agg["x"]["p50"]
        # The n=0 window contributed nothing.
        self.assertAlmostEqual(mean, 1.0, places=6)
        self.assertEqual(n, 10)

    def test_missing_file_exits(self):
        with tempfile.TemporaryDirectory() as td:
            run = Path(td) / "missing"
            run.mkdir()
            with self.assertRaises(SystemExit) as cm:
                perf_diff.load_run(run)
            self.assertNotEqual(cm.exception.code, 0)

    def test_format_table_nonempty(self):
        _, wa = perf_diff.load_run(self.run_a)
        _, wb = perf_diff.load_run(self.run_b)
        lines = perf_diff.format_table(perf_diff.aggregate(wa), perf_diff.aggregate(wb))
        joined = "\n".join(lines)
        self.assertIn("callback p95", joined)
        # Δ% should be present and non-empty.
        self.assertIn("%", joined)


if __name__ == "__main__":
    unittest.main()
