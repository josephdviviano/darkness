#!/usr/bin/env bash
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
# tools/perf_sweep.sh — unattended audio-knob sweep harness.
#
# Loops over a list of knob values, launches `darknessRender` for each, and
# captures the per-run JSONL artifact under ./perf/<mission>/. See
# .claude/PLAN.AUDIO_PROFILING.md §1.4.
#
# Usage:
#   tools/perf_sweep.sh <mission.mis> <knob> <val1> <val2> ...
#
# Example:
#   tools/perf_sweep.sh .../MISS6.MIS audio.reflections.hybrid_transition_time \
#       0.5 0.75 1.0 1.5
#
# Environment overrides:
#   DARKNESS_BIN         (default: ./build/default/src/main/darknessRender)
#   DARKNESS_RES         (default: /Volumes/THIEF2_INSTALL_C/THIEF2/RES)
#   DARKNESS_SCHEMAS     (default: /Volumes/THIEF2_CD2/EDITOR/SCHEMA)
#   DARKNESS_CONFIG      (default: ./darknessRender.yaml)
#   DARKNESS_DURATION    (default: 60 — passed to --exit-after-seconds)
#
#   AUTO_FLY             (default: 1 — auto-fly probe tour ON; set to 0 to
#                         disable; needed because without movement every
#                         iteration captures a stationary listener)
#   AUTO_FLY_SPEED       (override --auto-fly-speed; unset → binary default 10 ft/s)
#   AUTO_FLY_WAYPOINTS   (override --auto-fly-waypoints; unset → binary default 50)
#   AUTO_FLY_SEED        (override --auto-fly-seed; unset → binary default 0xC0FFEE)
#   AUTO_FLY_PAUSE_SEC   (override --auto-fly-pause-sec; unset → binary default 0)
#
#   AUTO_RUN             (default: 0 — ON-FOOT probe tour via --auto-run;
#                         the player RUNS between rooms generating footstep
#                         voices. Supersedes AUTO_FLY when 1 — the two are
#                         mutually exclusive in the binary and auto-run wins)
#   AUTO_RUN_WAYPOINTS   (override --auto-run-waypoints; unset → binary default 50)
#   AUTO_RUN_SEED        (override --auto-run-seed; unset → binary default 0xC0FFEE)
#   AUTO_RUN_SPEED_MODE  (override --auto-run-speed-mode run|walk|creep; unset → run)
#   AUDIO_RNG_SEED       (override --audio-rng-seed; unset → unseeded/random_device.
#                         Set for strict A/B so both runs pick identical wavs)
#
#   FORCE_PATHING_BAKE   (default: 0 — when 1, appends --force-pathing-bake so
#                         the pathing section is re-baked every iteration. This
#                         is the canonical "Sweep 2 Phase B mode" — slower per
#                         iter (a fresh pathing bake costs seconds) but correct
#                         for sweeps over pathing_probes.* knobs whose effect
#                         only shows up in a fresh bake. Composes with
#                         --skip-reflection-bake (always on here), so the
#                         iteration is: load .probes → carry reflection bytes
#                         forward → re-bake pathing → run for DURATION s.
#                         See PLAN.AUDIO_PROFILING.md §4.3 (Sweep 2 Phase B).)
#
# Flags:
#   --continue-on-error   keep going after a failing iteration (default: stop on
#                         first non-zero exit)

set -u  # error on unset vars; intentionally NOT `-e` so we can react per-run

# bash, not zsh — works on macOS + Linux.

print_usage() {
    cat <<'EOF'
Usage: tools/perf_sweep.sh [--continue-on-error] <mission.mis> <knob> <val1> [<val2> ...]

Loops over knob values, launches darknessRender for each, captures
audio_perf.jsonl under ./perf/<mission_basename>/<timestamp>__<label>/.

Each iteration runs:
  darknessRender <mission> --res <RES> --schemas <SCHEMAS>            \
      --config <CONFIG> --skip-reflection-bake                         \
      --set <knob>=<val> --perf-label <knob_short>_<val>               \
      --exit-after-seconds <DURATION>

Environment overrides:
  DARKNESS_BIN       (default: ./build/default/src/main/darknessRender)
  DARKNESS_RES       (default: /Volumes/THIEF2_INSTALL_C/THIEF2/RES)
  DARKNESS_SCHEMAS   (default: /Volumes/THIEF2_CD2/EDITOR/SCHEMA)
  DARKNESS_CONFIG    (default: ./darknessRender.yaml)
  DARKNESS_DURATION  (default: 60)
EOF
}

# --- arg parsing -------------------------------------------------------------

CONTINUE_ON_ERROR=0
POSITIONAL=()
for arg in "$@"; do
    case "$arg" in
        --continue-on-error)
            CONTINUE_ON_ERROR=1
            ;;
        --help|-h)
            print_usage
            exit 0
            ;;
        *)
            POSITIONAL+=("$arg")
            ;;
    esac
done

if [ "${#POSITIONAL[@]}" -lt 3 ]; then
    print_usage
    exit 1
fi

MISSION="${POSITIONAL[0]}"
KNOB="${POSITIONAL[1]}"
# Remaining positionals are the values to sweep.
VALUES=("${POSITIONAL[@]:2}")

# --- env overrides + defaults -------------------------------------------------

BIN="${DARKNESS_BIN:-./build/default/src/main/darknessRender}"
RES="${DARKNESS_RES:-/Volumes/THIEF2_INSTALL_C/THIEF2/RES}"
SCHEMAS="${DARKNESS_SCHEMAS:-/Volumes/THIEF2_CD2/EDITOR/SCHEMA}"
CONFIG="${DARKNESS_CONFIG:-./darknessRender.yaml}"
DURATION="${DARKNESS_DURATION:-60}"

# Auto-fly probe tour is ON by default for sweep runs — without it, every
# iteration captures a stationary listener and the perf-window data isn't
# representative of real gameplay load. The auto-fly module itself uses
# fixed defaults (speed=10ft/s, waypoints=50, seed=0xC0FFEE, pause=0) so
# the visit sequence is deterministic across iterations of the same
# mission. Override individual knobs via the env vars below; set
# AUTO_FLY=0 to disable entirely (use only for debugging startup-only
# knobs that don't need movement).
AUTO_FLY="${AUTO_FLY:-1}"
AUTO_FLY_SPEED="${AUTO_FLY_SPEED:-}"
AUTO_FLY_WAYPOINTS="${AUTO_FLY_WAYPOINTS:-}"
AUTO_FLY_SEED="${AUTO_FLY_SEED:-}"
AUTO_FLY_PAUSE_SEC="${AUTO_FLY_PAUSE_SEC:-}"

# On-foot auto-run tour (AutoRunTour.h). Off by default to preserve the
# established fly-tour sweep baseline; set AUTO_RUN=1 for footstep-churn
# stress runs (PLAN.AUDIO_PERF.md Phase 0). Supersedes AUTO_FLY when on.
AUTO_RUN="${AUTO_RUN:-0}"
AUTO_RUN_WAYPOINTS="${AUTO_RUN_WAYPOINTS:-}"
AUTO_RUN_SEED="${AUTO_RUN_SEED:-}"
AUTO_RUN_SPEED_MODE="${AUTO_RUN_SPEED_MODE:-}"
AUDIO_RNG_SEED="${AUDIO_RNG_SEED:-}"

# Audio-log verbosity gates ALL [PERF *] histogram recording — a sweep
# without it yields zeroed histograms discovered only after the fact.
# Default ON for sweeps; AUDIO_LOG=0 only if the YAML already sets
# developer.audio_log and you want the sweep to inherit it.
AUDIO_LOG="${AUDIO_LOG:-1}"

# Force-pathing-bake (Sweep 2 Phase B). When 1, every iteration drops the
# cached pathing section and re-bakes it fresh. Reflection bytes carry
# forward via --skip-reflection-bake (always passed below). Only switch
# this on for sweeps over pathing_probes.* knobs — for other sweeps
# (e.g. reflection_throttle), the cached pathing is stable across
# iterations and forcing a rebake just wastes time. See
# PLAN.AUDIO_PROFILING.md §4.3.
FORCE_PATHING_BAKE="${FORCE_PATHING_BAKE:-0}"

# Phase B startup warning — alert when force-rebake is on but the knob
# being swept isn't a pathing-probe knob. Force-rebake + non-pathing knob
# is wasteful (every iteration pays the pathing-bake cost for zero
# benefit) and almost always a typo / misconfiguration. Loud warning per
# feedback_no_silent_fallbacks.
if [ "$FORCE_PATHING_BAKE" = "1" ]; then
    case "$KNOB" in
        audio.pathing_probes.*)
            echo "[SWEEP_PHASE_B] FORCE_PATHING_BAKE=1 + ${KNOB} sweep" >&2
            echo "[SWEEP_PHASE_B] Each iteration will re-bake pathing fresh" \
                 "(seconds) and carry reflection bytes forward." >&2
            echo "[SWEEP_PHASE_B] This is the correct mode for pathing-probe" \
                 "sweeps. See PLAN.AUDIO_PROFILING.md §4.3 (Sweep 2 Phase B)." >&2
            ;;
        *)
            echo "[SWEEP_PHASE_B] WARN: FORCE_PATHING_BAKE=1 but sweeping" \
                 "'${KNOB}' (not a pathing-probe knob)." >&2
            echo "[SWEEP_PHASE_B] WARN: every iteration will pay a pathing-bake" \
                 "cost for no benefit." >&2
            echo "[SWEEP_PHASE_B] WARN: set FORCE_PATHING_BAKE=0 unless the knob" \
                 "you're sweeping actually changes pathing-probe placement." >&2
            ;;
    esac
fi

# --- validation (fail loud, no silent fallback) -------------------------------

if [ ! -x "$BIN" ]; then
    echo "[ERROR] darknessRender binary not found or not executable: $BIN" >&2
    echo "        set DARKNESS_BIN=/abs/path or build the default preset first." >&2
    exit 2
fi
if [ ! -f "$MISSION" ]; then
    echo "[ERROR] mission file not found: $MISSION" >&2
    exit 2
fi
if [ ! -d "$RES" ]; then
    echo "[ERROR] resource dir not found: $RES" >&2
    echo "        mount the install ISO or set DARKNESS_RES." >&2
    exit 2
fi
if [ ! -d "$SCHEMAS" ]; then
    echo "[WARN] schema dir not found: $SCHEMAS — continuing (schemas optional for some runs)" >&2
fi
if [ ! -f "$CONFIG" ]; then
    echo "[WARN] config yaml not found: $CONFIG — darknessRender will fall back to defaults" >&2
fi

# --- stale-probes warning -----------------------------------------------------
#
# We're passing --skip-reflection-bake to avoid the ~minutes-long probe-bake
# step every iteration. That re-uses whatever `.probes` file already exists on
# disk. If the user has changed any pathing-probe knob since the last bake, the
# results will be stale. Mirrors the [REFL_SKIP] semantics in commit fd61ebc.
#
# We don't have a robust way to detect "is the .probes file consistent with the
# YAML we're about to load" from a shell script, so we warn unconditionally on
# first iteration. This is intentional — the warning must be impossible to miss.

MISSION_BASENAME="$(basename "$MISSION")"
MISSION_DIR="$(dirname "$MISSION")"
PROBES_FILE="${MISSION_DIR}/${MISSION_BASENAME%.*}.probes"
if [ -f "$PROBES_FILE" ]; then
    echo "[REFL_SKIP-warn] --skip-reflection-bake will reuse: $PROBES_FILE" >&2
    echo "[REFL_SKIP-warn] if you changed pathing-probe knobs since the last bake," >&2
    echo "[REFL_SKIP-warn] delete this file to force a re-bake on the first iteration." >&2
else
    echo "[REFL_SKIP-warn] no .probes file at $PROBES_FILE" >&2
    echo "[REFL_SKIP-warn] first iteration will likely fail or take a long time;" >&2
    echo "[REFL_SKIP-warn] run once WITHOUT --skip-reflection-bake to seed it." >&2
fi

# --- per-iteration helpers ----------------------------------------------------

# Compose a short knob label for the per-run dir name.
# Strips a leading "audio." prefix and replaces "." with "_" so paths stay sane.
shorten_knob() {
    local k="$1"
    k="${k#audio.}"
    k="${k//./_}"
    echo "$k"
}

KNOB_SHORT="$(shorten_knob "$KNOB")"
OUT_BASE="./perf/${MISSION_BASENAME%.*}"
mkdir -p "$OUT_BASE"

# --- main sweep loop ----------------------------------------------------------

FAIL=0
ITER=0
for VAL in "${VALUES[@]}"; do
    ITER=$((ITER + 1))
    LABEL="${KNOB_SHORT}_${VAL}"

    # The binary creates its own per-run dir at openPerfJsonl time using
    # utcIsoCompact() (format YYYYMMDDTHHMMSSZ). We DON'T pre-create one
    # here — the script's date format and the binary's would diverge by
    # both shape (dashes vs none) and instant (script captures before
    # exec; binary captures after init), leaving an orphaned empty dir.
    # Instead, capture the binary's stderr, parse the [PERF_SINK] opened
    # line that AudioService emits, and report the actual path.

    echo "[SWEEP] iter ${ITER}/${#VALUES[@]}: ${KNOB}=${VAL}" >&2
    echo "[SWEEP] launching: $BIN $MISSION ... --set ${KNOB}=${VAL}" >&2

    # Tee stderr to a temp file so we can both stream it live AND grep
    # the [PERF_SINK] line out of it after the binary exits.
    # Build auto-fly arg list dynamically so unset overrides fall back to
    # the binary's compiled-in defaults.
    AUTO_FLY_ARGS=()
    if [ "$AUTO_RUN" = "1" ] || [ "$AUTO_RUN" = "true" ]; then
        # On-foot tour supersedes the fly tour (mutually exclusive in the
        # binary — physics ON vs OFF; auto-run wins there too).
        AUTO_FLY_ARGS+=(--auto-run)
        [ -n "$AUTO_RUN_WAYPOINTS" ]  && AUTO_FLY_ARGS+=(--auto-run-waypoints  "$AUTO_RUN_WAYPOINTS")
        [ -n "$AUTO_RUN_SEED" ]       && AUTO_FLY_ARGS+=(--auto-run-seed       "$AUTO_RUN_SEED")
        [ -n "$AUTO_RUN_SPEED_MODE" ] && AUTO_FLY_ARGS+=(--auto-run-speed-mode "$AUTO_RUN_SPEED_MODE")
    elif [ "$AUTO_FLY" = "1" ] || [ "$AUTO_FLY" = "true" ]; then
        AUTO_FLY_ARGS+=(--auto-fly)
        [ -n "$AUTO_FLY_SPEED" ]      && AUTO_FLY_ARGS+=(--auto-fly-speed     "$AUTO_FLY_SPEED")
        [ -n "$AUTO_FLY_WAYPOINTS" ]  && AUTO_FLY_ARGS+=(--auto-fly-waypoints "$AUTO_FLY_WAYPOINTS")
        [ -n "$AUTO_FLY_SEED" ]       && AUTO_FLY_ARGS+=(--auto-fly-seed      "$AUTO_FLY_SEED")
        [ -n "$AUTO_FLY_PAUSE_SEC" ]  && AUTO_FLY_ARGS+=(--auto-fly-pause-sec "$AUTO_FLY_PAUSE_SEC")
    fi
    [ -n "$AUDIO_RNG_SEED" ] && AUTO_FLY_ARGS+=(--audio-rng-seed "$AUDIO_RNG_SEED")
    if [ "$AUDIO_LOG" = "1" ] || [ "$AUDIO_LOG" = "true" ]; then
        AUTO_FLY_ARGS+=(--audio-log)
    fi

    # Sweep 2 Phase B: append --force-pathing-bake when FORCE_PATHING_BAKE=1.
    # The flag composes with --skip-reflection-bake (always passed) — the
    # iteration loads the existing .probes, carries reflection bytes
    # forward, and re-bakes only pathing. See PLAN.AUDIO_PROFILING.md §4.3.
    FORCE_PATHING_BAKE_ARGS=()
    if [ "$FORCE_PATHING_BAKE" = "1" ] || [ "$FORCE_PATHING_BAKE" = "true" ]; then
        FORCE_PATHING_BAKE_ARGS+=(--force-pathing-bake)
    fi

    STDERR_TMP="$(mktemp -t perf_sweep_stderr.XXXXXX)"
    "$BIN" \
        "$MISSION" \
        --res "$RES" \
        --schemas "$SCHEMAS" \
        --config "$CONFIG" \
        --skip-reflection-bake \
        "${FORCE_PATHING_BAKE_ARGS[@]}" \
        --set "${KNOB}=${VAL}" \
        --perf-label "$LABEL" \
        --exit-after-seconds "$DURATION" \
        "${AUTO_FLY_ARGS[@]}" \
        2> >(tee "$STDERR_TMP" >&2)
    RC=$?

    JSONL_PATH="$(grep -E '^\[PERF_SINK\] audio_perf\.jsonl opened: ' "$STDERR_TMP" \
                  | head -1 | sed -E 's/^\[PERF_SINK\] audio_perf\.jsonl opened: //')"
    rm -f "$STDERR_TMP"

    if [ -z "$JSONL_PATH" ]; then
        JSONL_PATH="(no [PERF_SINK] line found — JSONL may not have opened; check stderr above)"
    fi
    echo "[SWEEP] ${KNOB}=${VAL} → ${JSONL_PATH} (exit ${RC})"

    if [ "$RC" -ne 0 ]; then
        FAIL=1
        if [ "$CONTINUE_ON_ERROR" -eq 0 ]; then
            echo "[ERROR] iteration ${ITER} failed (exit ${RC}); aborting." >&2
            echo "        pass --continue-on-error to keep going past failures." >&2
            exit "$RC"
        else
            echo "[WARN] iteration ${ITER} failed (exit ${RC}); continuing." >&2
        fi
    fi
done

if [ "$FAIL" -ne 0 ]; then
    echo "[SWEEP] completed with at least one failed iteration." >&2
    exit 1
fi

echo "[SWEEP] all ${#VALUES[@]} iterations completed successfully."
echo "[SWEEP] results under: ${OUT_BASE}/"
