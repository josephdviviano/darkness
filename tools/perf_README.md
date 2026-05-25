# Audio profiling tools

Companion tooling for the audio-pipeline tuning workflow described in
`.claude/PLAN.AUDIO_PROFILING.md` (sections 1.1-1.4 and 4.2-4.5).

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
| `DARKNESS_BIN`       | `./build/default/src/main/darknessRender`        |
| `DARKNESS_RES`       | `/Volumes/THIEF2_INSTALL_C/THIEF2/RES`           |
| `DARKNESS_SCHEMAS`   | `/Volumes/THIEF2_CD2/EDITOR/SCHEMA`              |
| `DARKNESS_CONFIG`    | `./darknessRender.yaml`                          |
| `DARKNESS_DURATION`  | `60` (seconds; passed to `--exit-after-seconds`) |

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

### Phase B (deferred)

If Phase A picks a `dedup_radius_ft` that needs the `[PERF pathing] p99`
runtime acceptance check, that's Phase B — a live playtest per (mission,
radius) pair with the corresponding rebake. Currently `perf_sweep.sh`
caches the existing `.probes` file via `--skip-reflection-bake`, so it
won't force a pathing rebake between iterations. Phase B will need
either:
- A `--skip-pathing-bake=false` (force-rebake) CLI flag, OR
- An explicit `rm <mission>.probes` between sweep iterations

Not implemented yet — defer until Phase A picks a candidate radius.

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

## The five tuning runs to do

From `.claude/PLAN.AUDIO_PROFILING.md` §4.2 - §4.5 (§4.1 was removed
out-of-band — `hybrid_transition_time = 1.0s` is now a fixed design
point per the Steam Audio Unity/Unreal reference integrations).

| # | Knob(s) swept                                                              | Sweep values                                            | Acceptance criterion                                                                                                |
|---|----------------------------------------------------------------------------|---------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------|
| 1 | `reverb_voices` (hold `reverb_threads=8`, `conv_share=0.5`)                | {16, 24, 32}                                            | `[PERF refl_cache] evictions/sec` flat or lower; `[PERF worker_iter] max_apply p99` not disproportionate             |
| 2 | `pathing_probes.dedup_radius_ft` (per mission, via `darknessHeadless probe_plan`) | {5, 10, 15, 20}                                  | probe count in `[150, 180]` for most missions; per-mission acceptance: `[PERF pathing] p99 < 2 ms`                  |
| 3 | `reflection_throttle` × `reverb_voices_realtime` (joint sweep, ambisonics order 1) | throttle ∈ {4, 8, 16} × realtime ∈ {0, 4, 8} | `[PERF audio] refl_sim p99 < 8 ms`; no `BUDGET_WARN`; `[BEAT] ac_freq` does NOT correlate with `simRate/throttle`    |
| 4 | `pathing_update_interval`                                                  | {0.0, 0.05, 0.1, 0.2, 0.3}                              | `[PERF loopStep] p99` drop ≥ 2 ms at 0.3 vs 0.0; no `[PERF pathing] BUDGET_WARN`; subjective portal-traversal smoothness |

(Sweep 2 currently calls into `darknessHeadless probe_plan` for the
matrix step before live-runs on the divergent missions. See PLAN §4.3
for the full method.)

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
