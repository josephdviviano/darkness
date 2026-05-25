#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
#
# perf_probe_plan_sweep.sh — Phase A of PLAN.AUDIO_PROFILING.md §4.3.
#
# For each shipping Thief 2 mission × each pathing dedup-radius value,
# runs the headless `probe_plan` verb (Capability A from
# PLAN.PROBE_DEBUG_TOOLING.md) and emits a combined CSV that answers
# "what dedup_radius_ft hits the target 150-180 pathing-probe band
# across all levels?". No bake, no playtest — pure placement dry-run.
#
# Output:
#   ./perf/probe_plan/<utc_iso>/probe_count_matrix.csv
#     mission,dedup_radius_ft,pathing_probes,reflection_probes,dedup_dropped
#   plus a stdout summary table.
#
# Configuration via environment variables (with defaults):
#   DARKNESS_HEADLESS=./build/default/src/main/darknessHeadless
#   DARKNESS_RES=/Volumes/THIEF2_INSTALL_C/THIEF2/RES
#   DARKNESS_SCHEMAS=/Volumes/THIEF2_CD2/EDITOR/SCHEMA
#   LEVELS_DIR=../thief_2_service_release/rdrive/prj/thief2/levels/shipping
#   RADII="5 10 15 20"   (space-separated)
#   MISSIONS=""          (optional: explicit list; defaults to all *.mis/*.MIS)
#
# Usage:
#   tools/perf_probe_plan_sweep.sh
#   RADII="8 12 16" tools/perf_probe_plan_sweep.sh
#   MISSIONS="MISS6.mis MISS7.MIS" tools/perf_probe_plan_sweep.sh

set -uo pipefail

# --- config ------------------------------------------------------------------

DARKNESS_HEADLESS="${DARKNESS_HEADLESS:-./build/default/src/main/darknessHeadless}"
DARKNESS_RES="${DARKNESS_RES:-/Volumes/THIEF2_INSTALL_C/THIEF2/RES}"
DARKNESS_SCHEMAS="${DARKNESS_SCHEMAS:-/Volumes/THIEF2_CD2/EDITOR/SCHEMA}"
LEVELS_DIR="${LEVELS_DIR:-../thief_2_service_release/rdrive/prj/thief2/levels/shipping}"
RADII="${RADII:-5 10 15 20}"

# --- preflight --------------------------------------------------------------

if [ ! -x "$DARKNESS_HEADLESS" ]; then
    echo "[ERROR] darknessHeadless not found at: $DARKNESS_HEADLESS" >&2
    echo "        Build first: cmake --build build/default --target darknessHeadless" >&2
    exit 1
fi
if [ ! -d "$DARKNESS_RES" ]; then
    echo "[ERROR] --res directory not found: $DARKNESS_RES" >&2
    echo "        Mount disk 1: hdiutil mount ../disk_images/thief_2_disk_1.iso" >&2
    exit 1
fi
if [ ! -d "$DARKNESS_SCHEMAS" ]; then
    echo "[ERROR] --schemas directory not found: $DARKNESS_SCHEMAS" >&2
    echo "        Mount disk 2: hdiutil mount ../disk_images/thief_2_disk_2.iso" >&2
    exit 1
fi
if [ ! -d "$LEVELS_DIR" ]; then
    echo "[ERROR] LEVELS_DIR not found: $LEVELS_DIR" >&2
    exit 1
fi

# --- mission discovery -------------------------------------------------------

if [ -n "${MISSIONS:-}" ]; then
    # Explicit list, resolved against LEVELS_DIR
    MISSION_FILES=()
    for m in $MISSIONS; do
        if [ -f "${LEVELS_DIR}/${m}" ]; then
            MISSION_FILES+=("${LEVELS_DIR}/${m}")
        else
            echo "[WARN] mission not found, skipping: ${LEVELS_DIR}/${m}" >&2
        fi
    done
else
    # All shipping *.mis/*.MIS, alphabetized
    # shellcheck disable=SC2207
    MISSION_FILES=($(ls "$LEVELS_DIR"/*.{mis,MIS} 2>/dev/null | sort -u))
fi

if [ ${#MISSION_FILES[@]} -eq 0 ]; then
    echo "[ERROR] no mission files found in $LEVELS_DIR" >&2
    exit 1
fi

# --- output setup ------------------------------------------------------------

TS="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_DIR="./perf/probe_plan/${TS}"
mkdir -p "$OUT_DIR"
CSV="${OUT_DIR}/probe_count_matrix.csv"

echo "mission,dedup_radius_ft,pathing_probes,reflection_probes,dedup_dropped" > "$CSV"

echo "[SWEEP] missions: ${#MISSION_FILES[@]}, radii: $RADII"
echo "[SWEEP] output:   $CSV"
echo

# --- sweep loop -------------------------------------------------------------

# Parse a probe_plan stdout block for the three numbers we care about.
# Captures stderr too so [PROBE_PLAN] WARN about missing door OBBs (the
# headless-mode caveat from Cap A) doesn't get lost.
#
# Lines look like:
#   [PROBE_PLAN] reflections: floor=N elevation=N emitter=N globalDeduped=N total=N
#   [PROBE_PLAN] pathing: Portal=N DoorPair=N Centroid=N Emitter=N  postDedup=N
#   [PROBE_PLAN] pathing dedup_dropped: N (centroids=N doorPairs=N other=N)
parse_probe_plan() {
    local input="$1"
    local refl pathing dropped
    refl="$(echo "$input" | grep -E '^\[PROBE_PLAN\] reflections:' | sed -E 's/.*total=([0-9]+).*/\1/')"
    pathing="$(echo "$input" | grep -E '^\[PROBE_PLAN\] pathing:' | sed -E 's/.*postDedup=([0-9]+).*/\1/')"
    dropped="$(echo "$input" | grep -E '^\[PROBE_PLAN\] pathing dedup_dropped:' | sed -E 's/.*dedup_dropped: ([0-9]+).*/\1/')"
    echo "${refl:-NA},${pathing:-NA},${dropped:-NA}"
}

TOTAL_RUNS=$((${#MISSION_FILES[@]} * $(echo "$RADII" | wc -w)))
RUN=0

# Stdout summary header
printf "%-20s %-8s  %10s  %10s  %10s\n" "mission" "radius" "pathing" "reflection" "dedup_drop"
printf "%-20s %-8s  %10s  %10s  %10s\n" "-------" "------" "-------" "----------" "----------"

for MISSION_PATH in "${MISSION_FILES[@]}"; do
    MISSION_BASENAME="$(basename "$MISSION_PATH")"
    for R in $RADII; do
        RUN=$((RUN + 1))
        # darknessHeadless probe_plan honors --set; the headless main
        # passes argv through to RenderConfig::applyCliOverrides.
        OUTPUT="$("$DARKNESS_HEADLESS" \
            probe_plan "$MISSION_PATH" \
            --res "$DARKNESS_RES" \
            --schemas "$DARKNESS_SCHEMAS" \
            --set "audio.pathing_probes.dedup_radius_ft=${R}" \
            2>&1)"
        RC=$?
        if [ "$RC" -ne 0 ]; then
            echo "[WARN] iter ${RUN}/${TOTAL_RUNS} ${MISSION_BASENAME} r=${R}: exit ${RC}" >&2
            echo "${MISSION_BASENAME},${R},ERROR,ERROR,ERROR" >> "$CSV"
            printf "%-20s %-8s  %10s  %10s  %10s\n" "$MISSION_BASENAME" "$R" "ERR" "ERR" "ERR"
            continue
        fi
        PARSED="$(parse_probe_plan "$OUTPUT")"
        IFS=',' read -r REFL PATHING DROPPED <<< "$PARSED"
        echo "${MISSION_BASENAME},${R},${PATHING},${REFL},${DROPPED}" >> "$CSV"
        printf "%-20s %-8s  %10s  %10s  %10s\n" "$MISSION_BASENAME" "$R" "$PATHING" "$REFL" "$DROPPED"
    done
done

echo
echo "[SWEEP] complete. CSV: $CSV"
echo "[SWEEP] quick read: awk -F, 'NR>1 && \$3+0 >= 150 && \$3+0 <= 180' $CSV"
