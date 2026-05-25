# Audio profiling tools

Companion tooling for the audio-pipeline tuning workflow described in
`.claude/PLAN.AUDIO_PROFILING.md` (sections 1.1-1.4 and Part 4). The
sweep-ordering rationale below mirrors PLAN §4.0 (strategic ordering)
and PLAN §4.P1.* / §4.P2.* (per-sweep detail) — the git-tracked
companion here is the operator-facing summary, while the gitignored
PLAN holds the deeper design notes.

These scripts operate against the per-run JSONL artifact emitted by
`AudioService::dumpAudioStatusPeriodic` into
`./perf/<mission>/<utc_iso>__<perf_label>/audio_perf.jsonl`. The artifact
schema is defined in PLAN.AUDIO_PROFILING.md §1.1-1.2.

## File-format overview

Each run produces one JSONL file with:

- **Line 1**: `{"event":"run.meta", ...}` — git commit, mission, full
  `audio.*` config snapshot after CLI/YAML merge, device info (sample
  rate, frame size, deadline ms), `started_at_utc`.
- **Lines 2+**: `{"event":"perf.window", "wall_clock_s": <float>,
  "stages": {<stage>: {"n": <int>, "p50": ..., "p95": ..., "p99": ...},
  ...}}` — one record every 5 seconds, sub-objects for `callback`,
  `worker`, `refl_cache`, etc.
- **Event records** (`refl_evict`, `beat`, `refl_skip`) interleave;
  `perf_diff.py` ignores them.

See PLAN §1.7 for the sentinel-vs-real-value rule: missing or skipped
values are represented as JSON `null`, not as in-band magic numbers.

## How to capture a single run

```bash
./build/default/src/main/darknessRender <mission.mis>            \
    --res /Volumes/THIEF2_INSTALL_C/THIEF2/RES                   \
    --schemas /Volumes/THIEF2_CD2/EDITOR/SCHEMA                  \
    --config darknessRender.yaml                                 \
    --perf-label baseline                                        \
    --exit-after-seconds 60
```

This writes:

```
./perf/<mission_basename>/<UTC-timestamp>__baseline/audio_perf.jsonl
```

### Recommended `--exit-after-seconds`

- **60s** — default for most A/B comparisons; gives ~12 perf-windows of
  data at the 5-second emit cadence and is fast enough to iterate.
- **120s** — when chasing p99 tails or rare spawn-storm spikes;
  doubles sample count per stage so p99 narrows materially.

## How to run a sweep

`tools/perf_sweep.sh` loops over knob values, launches one
`darknessRender` per value, and writes the resulting JSONL into the
standard `./perf/...` layout. It uses `--skip-reflection-bake`
(commit `fd61ebc`) so iterations are fast; the script prints a stale-
probes warning on startup — heed it.

```bash
tools/perf_sweep.sh <mission.mis> <knob> <val1> <val2> ...
```

Example:

```bash
tools/perf_sweep.sh \
    ../thief_2_service_release/rdrive/prj/thief2/levels/shipping/MISS6.MIS \
    audio.reflections.hybrid_transition_time \
    0.5 0.75 1.0 1.5
```

### Environment-variable overrides

| Variable             | Default                                          |
|----------------------|--------------------------------------------------|
| `DARKNESS_BIN`        | `./build/default/src/main/darknessRender`                                       |
| `DARKNESS_RES`        | `/Volumes/THIEF2_INSTALL_C/THIEF2/RES`                                          |
| `DARKNESS_SCHEMAS`    | `/Volumes/THIEF2_CD2/EDITOR/SCHEMA`                                             |
| `DARKNESS_CONFIG`     | `./darknessRender.yaml`                                                         |
| `DARKNESS_DURATION`   | `60` (seconds; passed to `--exit-after-seconds`)                                |
| `AUTO_FLY`            | `1` (auto-fly probe tour ON by default; set to `0` to disable)                  |
| `AUTO_FLY_SPEED`      | unset → binary default 10 ft/s; override e.g. `AUTO_FLY_SPEED=15`               |
| `AUTO_FLY_WAYPOINTS`  | unset → binary default 50 (N-nearest probes)                                    |
| `AUTO_FLY_SEED`       | unset → binary default `0xC0FFEE` (decimal or `0xHEX` accepted)                 |
| `AUTO_FLY_PAUSE_SEC`  | unset → binary default 0 (continuous motion; nonzero pauses N s at each WP)     |
| `FORCE_PATHING_BAKE`  | `0` — set to `1` to append `--force-pathing-bake` (Sweep 2 Phase B; see below)  |

Auto-fly defaults to ON so every sweep iteration captures a comparable
moving-listener load profile rather than a stationary scene. Set
`AUTO_FLY=0` only when debugging startup-only knobs that don't need
movement (e.g. memory-budget allocation paths).

### `--continue-on-error`

By default the sweep stops on the first non-zero exit. Pass
`--continue-on-error` to power through (handy when one knob value is
known to crash and you want the rest of the data anyway).

## How to run the pathing-probe count matrix (Sweep 2, Phase A)

Sweep 2 in the table below ("`pathing_probes.dedup_radius_ft`") needs to
be evaluated **across all levels** because Thief 2 mission geometries
vary wildly — a dedup radius that hits the target 150-180 probe band on
MISS6 (dense townhouses) may over-cluster MISS1 (cathedral) or
under-cluster MISS15 (caverns). Phase A produces the
`mission × radius → probe_count` matrix needed to pick a per-mission
override scheme or a defensible single default.

Phase A runs entirely against the headless `probe_plan` verb (Cap A from
PLAN.PROBE_DEBUG_TOOLING.md). **No bake. No playtest.** It just runs the
placement logic dry on every mission for every radius value and emits
the counts.

```bash
tools/perf_probe_plan_sweep.sh
```

Default: all `*.mis` / `*.MIS` files under
`../thief_2_service_release/rdrive/prj/thief2/levels/shipping/` ×
radii `{5, 10, 15, 20}`. ~seconds per (mission, radius) pair.

### Output

```
./perf/probe_plan/<utc_iso>/probe_count_matrix.csv
```

with columns `mission,dedup_radius_ft,pathing_probes,reflection_probes,dedup_dropped`.
Plus a live stdout table.

Quick query — which (mission, radius) combos hit the target band?

```bash
awk -F, 'NR>1 && $3+0 >= 150 && $3+0 <= 180' ./perf/probe_plan/<TS>/probe_count_matrix.csv
```

### Environment-variable overrides

| Variable             | Default                                                                |
|----------------------|------------------------------------------------------------------------|
| `DARKNESS_HEADLESS`  | `./build/default/src/main/darknessHeadless`                            |
| `DARKNESS_RES`       | `/Volumes/THIEF2_INSTALL_C/THIEF2/RES`                                 |
| `DARKNESS_SCHEMAS`   | `/Volumes/THIEF2_CD2/EDITOR/SCHEMA`                                    |
| `LEVELS_DIR`         | `../thief_2_service_release/rdrive/prj/thief2/levels/shipping`         |
| `RADII`              | `"5 10 15 20"`                                                         |
| `MISSIONS`           | (empty — all missions in `LEVELS_DIR`)                                 |

Override examples:

```bash
# Different radii granularity
RADII="8 10 12 14 16" tools/perf_probe_plan_sweep.sh

# Just one mission
MISSIONS="MISS6.mis" tools/perf_probe_plan_sweep.sh
```

### Phase A caveat: no door OBBs

`darknessHeadless` cannot register door OBBs (renderer-only code path),
so `DoorPair` probes are absent from the Phase A count. `darknessRender`
(live mode) emits `DoorPair` probes for every door. The Phase A matrix
is **a lower bound** on real pathing-probe count by however many door-
pair probes the mission has (~roughly equal to door count). Cap A's
`[PROBE_PLAN] WARN: no door OBBs registered ... DoorPair classification
absent` line surfaces this on every run.

### Phase B — live p99 acceptance check (use `FORCE_PATHING_BAKE=1`)

Once Phase A picks a candidate `dedup_radius_ft` per most-divergent
mission, Phase B runs the live `darknessRender` per (mission, radius)
pair and checks `[PERF pathing] p99` etc. Each iteration must re-bake
the pathing section against that iteration's radius value while keeping
the reflection bake bytes from a prior good bake.

Tooling for this composition is the `--force-pathing-bake` CLI flag
(symmetric to `--skip-reflection-bake`) plus the `FORCE_PATHING_BAKE=1`
env var on `perf_sweep.sh`. The composition in the sweep harness is:

- Always: `--skip-reflection-bake` (carry reflection bytes forward).
- When `FORCE_PATHING_BAKE=1`: also `--force-pathing-bake` (drop the
  cached pathing section, re-bake pathing fresh).

Net effect per iteration: load `.probes` → carry reflection forward →
re-bake pathing (seconds) → run for `DURATION` s → next iteration.

```bash
FORCE_PATHING_BAKE=1 tools/perf_sweep.sh \
    ../path/MISS6.MIS audio.pathing_probes.dedup_radius_ft \
    <radii from Phase A>
```

`perf_sweep.sh` emits a `[SWEEP_PHASE_B]` startup banner whenever
`FORCE_PATHING_BAKE=1` is paired with an `audio.pathing_probes.*`
knob (the correct mode), and a `[SWEEP_PHASE_B] WARN` when paired
with any other knob (likely a typo — per-iter bake cost with no
benefit).

## How to compare two runs

```bash
python3 tools/perf_diff.py <runA_dir> <runB_dir>
```

Outputs:

1. **Knob diff** — only the `audio.*` leaves that differ between runs.
2. **Aggregated percentile table** — per stage / percentile, n-weighted
   mean across all `perf.window` records in each run, with `Δ%`.

Add `--plot` to write `compare.png` overlaying p95 trajectories per
stage for both runs. Requires `matplotlib`; if missing, the script
warns and skips the plot (no crash).

The diff math is unit-tested by `tools/perf_diff_test.py` — run with:

```bash
python3 tools/perf_diff_test.py
```

## Where artifacts go

```
./perf/<mission_basename>/<utc_iso>__<perf_label>/audio_perf.jsonl
```

`/perf/` is gitignored — the artifacts are large and bound to the
specific build (`git_commit` in `run.meta` is the link back to source).

## Sweeps to run — Phase 1 first, Phase 2 deferred

The sweep set is reorganized into two phases. Phase 1 (pathing) is
tractable today and unblocks Phase 2; Phase 2 (reflection) is held
until Phase 1 finishes. See `.claude/PLAN.AUDIO_PROFILING.md` §4.0
for the full strategic-ordering rationale (reflection bakes are
multi-minute per mission, so reflection sweeps must amortize ONE good
bake per level via `--skip-reflection-bake`; that bake's input probe
set comes from Phase 1, so Phase 1 must settle first).

### Phase 1 — Pathing tuning (do now)

| # | Knob(s) swept                                                              | Sweep values                                            | Phase | Per-iter bake cost                                | Acceptance criterion                                                                                                |
|---|----------------------------------------------------------------------------|---------------------------------------------------------|-------|----------------------------------------------------|----------------------------------------------------------------------------------------------------------------------|
| 1 | `pathing_probes.dedup_radius_ft` (matrix step — `darknessHeadless probe_plan`) | {5, 10, 15, 20}                                | A     | none (dry-run only)                               | probe count in `[150, 180]` for most missions; surfaces per-mission divergence                                       |
| 2 | `pathing_probes.dedup_radius_ft` (live p99 check on most-divergent missions) | radii from Phase A                              | B     | seconds (pathing-only re-bake; reflection carried forward) | per-mission `[PERF pathing] p99 < 2 ms`                                                                              |
| 3 | `pathing_update_interval`                                                  | {0.0, 0.05, 0.1, 0.2, 0.3}                              | runtime | none (knob doesn't affect probes — reflection carried forward, no bake) | `[PERF loopStep] p99` drop ≥ 2 ms at 0.3 vs 0.0; no `[PERF pathing] BUDGET_WARN`; subjective portal-traversal smoothness |

Phase 1 invocations:

```bash
# Phase 1.A — probe-count matrix (no bake, no playtest)
tools/perf_probe_plan_sweep.sh

# Phase 1.B — live p99 check at chosen radii (per-iter pathing re-bake)
FORCE_PATHING_BAKE=1 tools/perf_sweep.sh \
    ../path/MISS6.MIS audio.pathing_probes.dedup_radius_ft \
    <radii from Phase A>

# Phase 1 runtime — pathing-update-interval ablation (no bake at all)
tools/perf_sweep.sh \
    ../path/MISS6.MIS audio.propagation.pathing_update_interval \
    0.0 0.05 0.1 0.2 0.3
```

### Phase 2 — Reflection tuning (deferred — blocked on reflection-bake budget)

Both sweeps below are held until: (a) Phase 1 settles the pathing-probe
placement, and (b) one fresh reflection bake per shipping mission has
been executed at that placement. After (b), every Phase 2 iteration
carries those reflection bytes forward via `--skip-reflection-bake`
(no per-iter bake cost). See PLAN §4.0 for why pathing must settle
first.

| # | Knob(s) swept                                                              | Sweep values                                            | Per-iter bake cost                              | Acceptance criterion                                                                                                |
|---|----------------------------------------------------------------------------|---------------------------------------------------------|--------------------------------------------------|----------------------------------------------------------------------------------------------------------------------|
| 4 | `reverb_voices` (hold `reverb_threads=8`, `conv_share=0.5`)                | {16, 24, 32}                                            | none (reflection carried forward; pathing stable) | `[PERF refl_cache] evictions/sec` flat or lower; `[PERF worker_iter] max_apply p99` not disproportionate             |
| 5 | `reflection_throttle` × `reverb_voices_realtime` (joint sweep, ambisonics order 1) | throttle ∈ {4, 8, 16} × realtime ∈ {0, 4, 8} | none (reflection carried forward; pathing stable) | `[PERF audio] refl_sim p99 < 8 ms`; no `BUDGET_WARN`; `[BEAT] ac_freq` does NOT correlate with `simRate/throttle`    |

When Phase 2 is unblocked, every iteration uses `tools/perf_sweep.sh`
with default settings — `--skip-reflection-bake` is always passed by
the script, and `FORCE_PATHING_BAKE` stays at its default `0` (no
pathing re-bake — the cached pathing is the Phase 1 winner).

## Repeatable flythroughs — auto-fly probe tour

Without movement, the listener position is stationary and every sweep
iteration captures the same fixed scene. Auto-fly drives the camera
through a deterministic random tour of the N nearest pathing probes so
load-profile-over-time is comparable across sweeps.

Activates **only in fly mode** (physics off) so it doesn't fight the
physics integrator — when `--auto-fly` is passed, `physics_mode` is
forced false.

### CLI (composes with `perf_sweep.sh`)

```bash
./build/default/src/main/darknessRender <mission.mis> \
    --res ... --schemas ... \
    --skip-reflection-bake \
    --auto-fly \
    --auto-fly-speed 10        # ft/s (default 10)         \
    --auto-fly-waypoints 50    # N-nearest probes (default 50)  \
    --auto-fly-seed 0xC0FFEE   # PRNG seed (default 0xC0FFEE)   \
    --auto-fly-pause-sec 0     # seconds at each waypoint       \
    --perf-label baseline      \
    --exit-after-seconds 60
```

### Debug-console toggles

`auto_fly` (bool), `auto_fly_speed` (0.5-50), `auto_fly_pause_sec` (0-30).
Live-tunable during interactive testing; CLI flags are the right path
for sweep harnesses.

### Determinism

The same seed + same probe set + same mission produces the same
sequence of waypoints. So as long as you keep `--auto-fly-seed`,
`--auto-fly-waypoints`, and the `.probes` file fixed across sweep
iterations, perf-window N of run A and perf-window N of run B are
sampling the same listener position to within a few feet.

### Activation log

On first activation:
```
[AUTO_FLY] activated: 50 waypoints seed=0xc0ffee speed=10.0 pause=0.0s
```
Per waypoint reached:
```
[AUTO_FLY] reached waypoint 12/50 -> next 31 at (123.4, -88.1, 5.2)
```

### Fallback

If `--auto-fly` is set but the mission has no pathing probes (no
`.probes` file or pathing batch empty):
```
[FALLBACK] auto-fly requested but no pathing probes loaded — disabling
```
Camera reverts to standard fly mode; the binary stays alive so the
sweep iteration still captures *something* (perf data sans movement).

## Pitfall: stale `.probes`

`perf_sweep.sh` passes `--skip-reflection-bake`. If you changed any
pathing-probe knob since the last bake, the `.probes` file is stale and
every sweep iteration will use it. The script warns at startup; either
delete the stale `.probes` and run once without `--skip-reflection-bake`
to seed it, or accept that the sweep dimension is orthogonal to probe
geometry.

See PLAN §1.7 (pitfall guards) for the full list of measurement
correctness rules.
