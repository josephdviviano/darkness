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
# tools/wav_artifacts.py — offline artifact analyzer for --capture-wav output
# (PLAN.AUDIO_PERF.md PR 0.2).
#
# The engine records its final stereo f32 output to output.wav next to each
# run's audio_perf.jsonl ([WAV_CAPTURE] opened/closed stderr lines). This
# script turns that capture into a pass/fail artifact report:
#
#   Single-file mode:
#       python3 tools/wav_artifacts.py perf/miss6/<run>/output.wav
#     Reports: duration, clipping runs, dropouts (sustained digital silence
#     mid-run), clicks (robust first-difference outliers), DC offset,
#     low-frequency amplitude modulation (the wet-bus "beating" detector —
#     cross-check against the in-engine [BEAT] tap), per-band RMS.
#
#   Compare mode (A/B):
#       python3 tools/wav_artifacts.py baseline.wav candidate.wav
#     Runs are seeded/deterministic but NOT sample-aligned (voice-init timing
#     jitters run-to-run), so we align by first-audio onset and compare
#     aggregate statistics — band-RMS deltas (dB), envelope correlation, and
#     a per-metric artifact-count table — never raw samples.
#
# Exit codes: 0 = no artifacts above thresholds, 1 = artifacts found
# (details printed), 2 = usage / IO error.
#
# Dependencies: python3 + numpy ONLY (no scipy). The stdlib `wave` module
# rejects WAVE_FORMAT_IEEE_FLOAT (format 3), which is exactly what the
# engine's f32 capture uses, so the RIFF container is parsed manually.

import struct
import sys

import numpy as np

# ── artifact thresholds (shared by single + compare modes) ──────────────────
CLIP_LEVEL          = 0.985   # |s| at/above this counts as clipped
DROPOUT_LEVEL       = 1e-5    # |s| below this is "digital silence"
DROPOUT_MIN_MS      = 30.0    # silence must persist this long to be a dropout
CLICK_MAD_FACTOR    = 12.0    # |diff| > k * rolling MAD flags a click
CLICK_MIN_DIFF      = 1e-3    # ...AND above this absolute inter-sample jump
                              # (-60 dBFS) — statistical outliers below it are
                              # inaudible and would flap the exit code
CLICK_MERGE_MS      = 5.0     # flagged samples merged within this window
CLICK_REPORT_WORST  = 10      # timestamps printed for the worst N clicks
DC_OFFSET_LIMIT     = 0.01    # |mean| above this flags a DC problem
MOD_BAND_HZ         = (0.5, 8.0)   # LF amplitude-modulation search band
MOD_PROMINENCE_DB   = 12.0    # envelope-spectrum peak must exceed the band
                              # median by this much to count as beating
MOD_DEPTH_FLOOR     = 0.05    # ...AND the modulation must move the envelope
                              # by at least this fraction of its mean level —
                              # spectral prominence alone flags inaudible
                              # bookkeeping (PR #4 review S4)
FIRST_AUDIO_LEVEL   = 1e-4    # onset threshold for alignment / dropout gating
# Octave-ish analysis bands (Hz): lows / low-mids / highs / air. FFT-integrated
# (no scipy filters). Upper edge is clamped to Nyquist at runtime.
RMS_BANDS = [(20.0, 250.0), (250.0, 1000.0), (1000.0, 4000.0), (4000.0, 16000.0)]


# ── WAV reading ──────────────────────────────────────────────────────────────

def read_wav(path):
    """Parse a RIFF/WAVE file; return (samples float32 [frames, channels], rate).

    Handles format 1 (PCM 16/24/32), format 3 (IEEE float 32/64) and
    WAVE_FORMAT_EXTENSIBLE (0xFFFE) wrapping either. Everything is normalized
    to float32 in [-1, 1]. Raises ValueError on malformed input.
    """
    with open(path, "rb") as f:
        riff = f.read(12)
        if len(riff) < 12 or riff[0:4] != b"RIFF" or riff[8:12] != b"WAVE":
            raise ValueError(f"{path}: not a RIFF/WAVE file")

        fmt = None
        data = None
        while True:
            hdr = f.read(8)
            if len(hdr) < 8:
                break
            cid, size = hdr[0:4], struct.unpack("<I", hdr[4:8])[0]
            if cid == b"fmt ":
                fmt = f.read(size)
            elif cid == b"data":
                data = f.read(size)
            else:
                f.seek(size, 1)
            if size % 2:                      # RIFF chunks are word-aligned
                f.seek(1, 1)

        if fmt is None or data is None:
            raise ValueError(f"{path}: missing fmt/data chunk")

        (tag, channels, rate, _brate, _balign, bits) = struct.unpack(
            "<HHIIHH", fmt[:16])
        if tag == 0xFFFE and len(fmt) >= 40:  # EXTENSIBLE: real tag in SubFormat
            tag = struct.unpack("<H", fmt[24:26])[0]

        if tag == 3 and bits == 32:
            x = np.frombuffer(data, dtype="<f4").astype(np.float32)
        elif tag == 3 and bits == 64:
            x = np.frombuffer(data, dtype="<f8").astype(np.float32)
        elif tag == 1 and bits == 16:
            x = np.frombuffer(data, dtype="<i2").astype(np.float32) / 32768.0
        elif tag == 1 and bits == 32:
            x = np.frombuffer(data, dtype="<i4").astype(np.float32) / 2147483648.0
        elif tag == 1 and bits == 24:
            raw = np.frombuffer(data, dtype=np.uint8)
            raw = raw[: (len(raw) // 3) * 3].reshape(-1, 3)
            i32 = (raw[:, 0].astype(np.int32)
                   | (raw[:, 1].astype(np.int32) << 8)
                   | (raw[:, 2].astype(np.int32) << 16))
            i32 = np.where(i32 & 0x800000, i32 - 0x1000000, i32)
            x = i32.astype(np.float32) / 8388608.0
        else:
            raise ValueError(f"{path}: unsupported WAV (format={tag} bits={bits})")

        if channels < 1:
            raise ValueError(f"{path}: bad channel count {channels}")
        x = x[: (len(x) // channels) * channels].reshape(-1, channels)
        return x, rate


# ── helpers ──────────────────────────────────────────────────────────────────

def runs_of(mask):
    """Return (start, length) pairs for each run of True in a boolean array."""
    if not mask.any():
        return []
    d = np.diff(mask.astype(np.int8))
    starts = np.flatnonzero(d == 1) + 1
    ends = np.flatnonzero(d == -1) + 1
    if mask[0]:
        starts = np.concatenate(([0], starts))
    if mask[-1]:
        ends = np.concatenate((ends, [len(mask)]))
    return list(zip(starts, ends - starts))


def first_last_audio(peak, rate):
    """Index of first/last sample above FIRST_AUDIO_LEVEL (None if silent)."""
    nz = np.flatnonzero(peak > FIRST_AUDIO_LEVEL)
    if len(nz) == 0:
        return None, None
    return int(nz[0]), int(nz[-1])


def envelope(mono_abs, rate, hop_ms=10.0):
    """|x| lowpassed via boxcar mean per hop → (env array, env_rate Hz)."""
    hop = max(1, int(rate * hop_ms / 1000.0))
    n = (len(mono_abs) // hop) * hop
    if n == 0:
        return np.zeros(0, dtype=np.float32), 1000.0 / hop_ms
    env = mono_abs[:n].reshape(-1, hop).mean(axis=1)
    return env, rate / hop


def band_rms_db(mono, rate):
    """FFT-integrated RMS (dBFS) per RMS_BANDS entry. Parseval: total signal
    power is distributed over rfft bins, so summing |X|^2 in a bin range is
    the power of that band."""
    n = len(mono)
    if n == 0:
        return [float("-inf")] * len(RMS_BANDS)
    spec = np.abs(np.fft.rfft(mono)) ** 2 / (n * n)
    # one-sided spectrum: double everything except DC and (if present) Nyquist
    spec[1:] *= 2.0
    if n % 2 == 0 and len(spec) > 1:
        spec[-1] /= 2.0
    freqs = np.fft.rfftfreq(n, 1.0 / rate)
    out = []
    nyq = rate / 2.0
    for lo, hi in RMS_BANDS:
        hi = min(hi, nyq)
        sel = (freqs >= lo) & (freqs < hi)
        power = float(spec[sel].sum()) if sel.any() else 0.0
        out.append(10.0 * np.log10(power) if power > 0 else float("-inf"))
    return out


def db_str(v):
    return f"{v:7.2f} dB" if np.isfinite(v) else "   -inf dB"


# ── single-file analysis ─────────────────────────────────────────────────────

def analyze(path):
    """Analyze one capture; returns a dict of metrics (prints nothing)."""
    x, rate = read_wav(path)
    frames, channels = x.shape
    duration = frames / rate if rate else 0.0
    peak = np.abs(x).max(axis=1) if channels > 1 else np.abs(x[:, 0])
    mono = x.mean(axis=1)

    r = {
        "path": path, "rate": rate, "channels": channels,
        "frames": frames, "duration": duration,
    }

    # Degenerate capture (e.g. an unfinalized WAV whose header still says 0
    # data bytes): report it as the artifact it is instead of crashing the
    # metric code on empty arrays.
    if frames < 2:
        r.update({
            "first_audio_s": None, "last_audio_s": None,
            "clip_count": 0, "clip_worst_run": 0, "clip_worst_run_ms": 0.0,
            "dropouts": [], "dropout_count": 0,
            "click_count": 0, "click_worst": [],
            "dc_offset": [0.0] * channels, "dc_flag": False,
            "mod_freq_hz": None, "mod_prominence_db": None, "mod_flag": False,
            "_env": np.zeros(0, dtype=np.float32), "_env_rate": 100.0,
            "band_rms_db": [float("-inf")] * len(RMS_BANDS),
            "rms_db": float("-inf"),
            "artifacts": [f"empty capture: {frames} frame(s) of data "
                          "(unfinalized WAV header? check for the "
                          "[WAV_CAPTURE] closed: line in the run log)"],
        })
        return r

    # -- onset / tail (gates the dropout scan; also the A/B alignment key) --
    fa, la = first_last_audio(peak, rate)
    r["first_audio_s"] = fa / rate if fa is not None else None
    r["last_audio_s"] = la / rate if la is not None else None

    # -- clipping: runs of |s| >= CLIP_LEVEL on any channel --
    clip_runs = runs_of(peak >= CLIP_LEVEL)
    r["clip_count"] = len(clip_runs)
    r["clip_worst_run"] = max((ln for _, ln in clip_runs), default=0)
    r["clip_worst_run_ms"] = r["clip_worst_run"] * 1000.0 / rate

    # -- dropouts: sustained digital silence strictly inside the audio span --
    r["dropouts"] = []
    if fa is not None and la is not None and la > fa:
        min_len = int(rate * DROPOUT_MIN_MS / 1000.0)
        span = peak[fa:la]
        for start, length in runs_of(span < DROPOUT_LEVEL):
            if length >= min_len:
                r["dropouts"].append(((fa + start) / rate, length * 1000.0 / rate))
    r["dropout_count"] = len(r["dropouts"])

    # -- clicks: robust outliers in the first-difference signal --
    # Rolling MAD approximated per ~100 ms block (exact rolling MAD over
    # millions of samples is O(n·w); block MAD tracks the local noise floor
    # closely enough for a 12x threshold). Blocks are computed on |diff|,
    # then broadcast back to sample resolution.
    diff = np.abs(np.diff(mono))
    block = max(1, min(int(rate * 0.1), len(diff)))  # short file → one block
    nblocks = max(1, len(diff) // block)
    trimmed = diff[: nblocks * block].reshape(nblocks, block)
    med = np.median(trimmed, axis=1)
    mad = np.median(np.abs(trimmed - med[:, None]), axis=1)
    # Floor: quantization-scale noise; prevents 0-MAD silent blocks from
    # flagging every sample of the next fade-in.
    mad = np.maximum(mad, 1e-6)
    rolling_mad = np.repeat(mad, block)
    if len(rolling_mad) < len(diff):  # tail partial block reuses last block
        rolling_mad = np.concatenate(
            (rolling_mad, np.full(len(diff) - len(rolling_mad), rolling_mad[-1])))
    flagged = np.flatnonzero(
        (diff > rolling_mad * CLICK_MAD_FACTOR) & (diff > CLICK_MIN_DIFF))
    merge = int(rate * CLICK_MERGE_MS / 1000.0)
    clicks = []  # (sample_idx, severity = diff / rolling MAD)
    for idx in flagged:
        sev = float(diff[idx] / rolling_mad[idx])
        if clicks and idx - clicks[-1][0] <= merge:
            if sev > clicks[-1][1]:
                clicks[-1] = (int(idx), sev)
        else:
            clicks.append((int(idx), sev))
    r["click_count"] = len(clicks)
    r["click_worst"] = sorted(clicks, key=lambda c: -c[1])[:CLICK_REPORT_WORST]

    # -- DC offset --
    r["dc_offset"] = [float(x[:, c].mean()) for c in range(channels)]
    r["dc_flag"] = any(abs(d) > DC_OFFSET_LIMIT for d in r["dc_offset"])

    # -- LF amplitude modulation (wet-bus beating detector) --
    # Envelope via |x| boxcar-lowpassed to 100 Hz, then FFT of the
    # mean-removed envelope; peak in MOD_BAND_HZ with prominence measured
    # against the band's median bin. Cross-check hits against the
    # in-engine [BEAT] tap before blaming the capture.
    #
    # PR #4 review S4: restrict to the ACTIVE region [first_audio,
    # last_audio] — every real capture starts with a silent lead-in
    # (probe bake / level load), and including it turns the envelope into
    # a step function whose spectrum flags clean content. Additionally
    # gate on modulation DEPTH: prominence alone (peak vs band median)
    # fires on inaudible wiggle when the band is otherwise flat.
    if fa is not None and la is not None and la > fa:
        mod_src = mono[fa:la]
    else:
        mod_src = mono
    env, env_rate = envelope(np.abs(mod_src), rate)
    r["mod_freq_hz"] = None
    r["mod_prominence_db"] = None
    r["mod_depth"] = None
    r["mod_flag"] = False
    if len(env) >= int(env_rate * 4):  # need >= 4 s for 0.5 Hz resolution
        w = env - env.mean()
        w = w * np.hanning(len(w))
        espec = np.abs(np.fft.rfft(w)) ** 2
        efreqs = np.fft.rfftfreq(len(w), 1.0 / env_rate)
        sel = (efreqs >= MOD_BAND_HZ[0]) & (efreqs <= MOD_BAND_HZ[1])
        if sel.any():
            band = espec[sel]
            bfreqs = efreqs[sel]
            pk = int(np.argmax(band))
            med_p = float(np.median(band))
            if band[pk] > 0 and med_p > 0:
                prom = 10.0 * np.log10(band[pk] / med_p)
                # Sine-amplitude estimate from the Hann-windowed |rfft|²
                # bin: A ≈ 4·sqrt(P)/N (Hann coherent gain 0.5). Depth =
                # that amplitude relative to the mean envelope level.
                amp = 4.0 * float(np.sqrt(band[pk])) / max(len(w), 1)
                depth = amp / max(float(env.mean()), 1e-9)
                r["mod_freq_hz"] = float(bfreqs[pk])
                r["mod_prominence_db"] = float(prom)
                r["mod_depth"] = float(depth)
                r["mod_flag"] = (prom > MOD_PROMINENCE_DB
                                 and depth > MOD_DEPTH_FLOOR)
    r["_env"] = env          # kept for compare mode (not printed)
    r["_env_rate"] = env_rate

    # -- per-band RMS --
    r["band_rms_db"] = band_rms_db(mono, rate)
    r["rms_db"] = (10.0 * np.log10(float(np.mean(mono ** 2)))
                   if np.any(mono) else float("-inf"))

    # -- verdicts --
    r["artifacts"] = []
    if r["clip_count"] > 0:
        r["artifacts"].append(
            f"clipping: {r['clip_count']} run(s), worst "
            f"{r['clip_worst_run']} samples ({r['clip_worst_run_ms']:.2f} ms)")
    if r["dropout_count"] > 0:
        r["artifacts"].append(f"dropouts: {r['dropout_count']} mid-run silence gap(s)")
    if r["click_count"] > 0:
        r["artifacts"].append(f"clicks: {r['click_count']} first-difference outlier(s)")
    if r["dc_flag"]:
        r["artifacts"].append(
            "dc offset: " + ", ".join(f"ch{c}={d:+.4f}"
                                      for c, d in enumerate(r["dc_offset"])))
    if r["mod_flag"]:
        r["artifacts"].append(
            f"LF amplitude modulation: {r['mod_freq_hz']:.2f} Hz peak, "
            f"prominence {r['mod_prominence_db']:.1f} dB "
            f"(cross-check the in-engine [BEAT] tap)")
    if r["first_audio_s"] is None:
        r["artifacts"].append("file is entirely silent (no sample above "
                              f"{FIRST_AUDIO_LEVEL})")
    return r


def print_report(r):
    print(f"== {r['path']}")
    print(f"   {r['duration']:.2f} s, {r['channels']} ch, {r['rate']} Hz, "
          f"{r['frames']} frames, overall RMS {db_str(r['rms_db'])}")
    if r["first_audio_s"] is not None:
        print(f"   first audio {r['first_audio_s']:.3f} s, "
              f"last audio {r['last_audio_s']:.3f} s")
    else:
        print("   NO AUDIO detected (entirely silent)")

    print(f"   clipping   : {r['clip_count']} run(s)"
          + (f", worst {r['clip_worst_run']} samples "
             f"({r['clip_worst_run_ms']:.2f} ms)" if r["clip_count"] else ""))
    print(f"   dropouts   : {r['dropout_count']}"
          + (" (>%g ms silence mid-run)" % DROPOUT_MIN_MS
             if r["dropout_count"] else ""))
    for t, ms in r["dropouts"][:10]:
        print(f"                at {t:8.3f} s, {ms:7.1f} ms")
    print(f"   clicks     : {r['click_count']}")
    for idx, sev in r["click_worst"]:
        print(f"                at {idx / r['rate']:8.3f} s, "
              f"{sev:5.1f}x rolling MAD")
    print("   dc offset  : "
          + ", ".join(f"ch{c}={d:+.5f}" for c, d in enumerate(r["dc_offset"])))
    if r["mod_freq_hz"] is not None:
        state = "FLAG" if r["mod_flag"] else "ok"
        print(f"   LF ampl mod: peak {r['mod_freq_hz']:.2f} Hz, prominence "
              f"{r['mod_prominence_db']:.1f} dB [{state}] "
              f"(band {MOD_BAND_HZ[0]}-{MOD_BAND_HZ[1]} Hz)")
    else:
        print("   LF ampl mod: n/a (capture too short for 0.5 Hz resolution)")
    print("   band RMS   : "
          + "  ".join(f"{int(lo)}-{int(hi)}Hz {db_str(v)}"
                      for (lo, hi), v in zip(RMS_BANDS, r["band_rms_db"])))
    if r["artifacts"]:
        print("   ARTIFACTS:")
        for a in r["artifacts"]:
            print(f"     - {a}")
    else:
        print("   no artifacts above thresholds")


# ── compare mode ─────────────────────────────────────────────────────────────

def compare(ra, rb):
    """A/B comparison of two analyzed captures. Aligned by first-audio onset;
    aggregate stats only (runs are deterministic but not sample-aligned)."""
    print("\n== A/B comparison (aligned by first-audio onset)")
    if ra["rate"] != rb["rate"]:
        print(f"   WARNING: sample rates differ ({ra['rate']} vs {rb['rate']}) — "
              "band comparisons remain valid (frequency-domain), envelope "
              "correlation uses the common envelope rate")

    print(f"   {'metric':<22} {'A':>12} {'B':>12} {'delta':>12}")
    print(f"   {'duration (s)':<22} {ra['duration']:>12.2f} {rb['duration']:>12.2f} "
          f"{rb['duration'] - ra['duration']:>+12.2f}")

    # band-RMS deltas over the onset-aligned common span
    def aligned_mono(r):
        x, _rate = read_wav(r["path"])
        mono = x.mean(axis=1)
        start = int((r["first_audio_s"] or 0.0) * r["rate"])
        return mono[start:]

    ma, mb = aligned_mono(ra), aligned_mono(rb)
    common_a = min(len(ma), int(len(mb) * ra["rate"] / rb["rate"]))
    common_b = min(len(mb), int(len(ma) * rb["rate"] / ra["rate"]))
    ba = band_rms_db(ma[:common_a], ra["rate"])
    bb = band_rms_db(mb[:common_b], rb["rate"])
    for (lo, hi), va, vb in zip(RMS_BANDS, ba, bb):
        d = vb - va if np.isfinite(va) and np.isfinite(vb) else float("nan")
        print(f"   {'rms %d-%dHz (dB)' % (lo, hi):<22} {va:>12.2f} {vb:>12.2f} "
              f"{d:>+12.2f}")

    # envelope correlation over the aligned common span
    ea = ra["_env"][int((ra["first_audio_s"] or 0) * ra["_env_rate"]):]
    eb = rb["_env"][int((rb["first_audio_s"] or 0) * rb["_env_rate"]):]
    n = min(len(ea), len(eb))
    if n > 16 and np.std(ea[:n]) > 0 and np.std(eb[:n]) > 0:
        corr = float(np.corrcoef(ea[:n], eb[:n])[0, 1])
        print(f"   {'envelope correlation':<22} {corr:>38.3f}")
        if corr < 0.5:
            print("     NOTE: low correlation — runs likely diverged "
                  "(different tour/seed?); treat per-band deltas with care")
    else:
        print(f"   {'envelope correlation':<22} {'n/a':>38}")

    # per-metric artifact-count table
    print(f"\n   {'artifact counts':<22} {'A':>12} {'B':>12} {'delta':>12}")
    for label, key in [("clip runs", "clip_count"),
                       ("dropouts", "dropout_count"),
                       ("clicks", "click_count")]:
        print(f"   {label:<22} {ra[key]:>12d} {rb[key]:>12d} "
              f"{rb[key] - ra[key]:>+12d}")
    fa = "yes" if ra["mod_flag"] else "no"
    fb = "yes" if rb["mod_flag"] else "no"
    print(f"   {'LF-mod flagged':<22} {fa:>12} {fb:>12}")
    da = "yes" if ra["dc_flag"] else "no"
    db = "yes" if rb["dc_flag"] else "no"
    print(f"   {'DC-offset flagged':<22} {da:>12} {db:>12}")


# ── entry point ──────────────────────────────────────────────────────────────

def main(argv):
    flags = [a for a in argv[1:] if a.startswith("-")]
    args = [a for a in argv[1:] if not a.startswith("-")]
    if flags or len(args) not in (1, 2):
        print("usage: wav_artifacts.py <run.wav>          # single-file report\n"
              "       wav_artifacts.py <A.wav> <B.wav>    # A/B comparison\n"
              "exit codes: 0 = clean, 1 = artifacts found, 2 = usage/IO error",
              file=sys.stderr)
        return 2

    try:
        reports = [analyze(p) for p in args]
    except (OSError, ValueError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2

    for r in reports:
        print_report(r)
        print()

    if len(reports) == 2:
        compare(reports[0], reports[1])

    return 1 if any(r["artifacts"] for r in reports) else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
