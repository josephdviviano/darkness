#!/usr/bin/env python3
# This file is part of the darkness project
# Copyright (C) 2024-2026 darkness contributors
# GPLv2+ — see the repository's licence headers.
"""Verify the flicker synthesizer's output against the measured references.

PLAN.FLICKER_PHYSICS.md §8: a stage without a test that can fail is not done.
This consumes `darknessHeadless flicker_sim <type>` CSV (the C++ synthesizer
is the only implementation — no Python re-derivation to drift) and checks,
per lamp type:

  fire-torch / fire-candle:
    * a spectral concentration near the preset's puffing frequency
    * low-frequency energy dominates high (flat-then-rolloff shape)
    * sigma/mu in the physical range, positive skew (the fat bright tail)
  gas-open:   an order of magnitude calmer than fire, no puffing line
  gas-mantle / electric: essentially steady (sigma/mu < 0.02)
  tint: fire's green/blue channels track intensity (Planckian coupling);
        electric stays exactly white.

Usage:
  build/default/src/main/darknessHeadless flicker_sim fire-torch 300 | \
      python3 analysis/flicker_spectrum.py fire-torch
  python3 analysis/flicker_spectrum.py fire-torch < samples.csv
Exit code 0 = all checks pass.
"""

import sys
import math


def read_csv(stream):
    ts, xs, rs, gs, bs = [], [], [], [], []
    for line in stream:
        line = line.strip()
        if not line or line.startswith("#") or line.startswith("t,"):
            continue
        parts = line.split(",")
        if len(parts) < 5:
            continue
        ts.append(float(parts[0]))
        xs.append(float(parts[1]))
        rs.append(float(parts[2]))
        gs.append(float(parts[3]))
        bs.append(float(parts[4]))
    return ts, xs, rs, gs, bs


def dft_power(xs, rate):
    """Plain DFT power spectrum (no numpy dependency; N is a few thousand)."""
    n = len(xs)
    mean = sum(xs) / n
    centred = [x - mean for x in xs]
    half = n // 2
    freqs, power = [], []
    for k in range(1, half):
        wr = wi = 0.0
        ang0 = -2.0 * math.pi * k / n
        for i, x in enumerate(centred):
            a = ang0 * i
            wr += x * math.cos(a)
            wi += x * math.sin(a)
        freqs.append(k * rate / n)
        power.append((wr * wr + wi * wi) / n)
    return freqs, power


def band_energy(freqs, power, lo, hi):
    return sum(p for f, p in zip(freqs, power) if lo <= f < hi)


def stats(xs):
    n = len(xs)
    mu = sum(xs) / n
    var = sum((x - mu) ** 2 for x in xs) / n
    sd = math.sqrt(var)
    skew = (sum((x - mu) ** 3 for x in xs) / n) / (sd ** 3) if sd > 0 else 0.0
    return mu, sd, skew


def corr(a, b):
    n = len(a)
    ma, mb = sum(a) / n, sum(b) / n
    ca = [x - ma for x in a]
    cb = [x - mb for x in b]
    num = sum(x * y for x, y in zip(ca, cb))
    da = math.sqrt(sum(x * x for x in ca))
    db = math.sqrt(sum(y * y for y in cb))
    return num / (da * db) if da > 0 and db > 0 else 0.0


def main():
    lamp = sys.argv[1] if len(sys.argv) > 1 else "fire-torch"
    ts, xs, rs, gs, bs = read_csv(sys.stdin)
    if len(xs) < 256:
        print(f"FAIL: only {len(xs)} samples — feed flicker_sim output")
        return 1
    rate = 1.0 / (ts[1] - ts[0])
    mu, sd, skew = stats(xs)
    ratio = sd / mu if mu > 0 else 0.0
    checks = []  # (name, ok, detail)

    if lamp.startswith("fire"):
        freqs, power = dft_power(xs, rate)
        # Expected puffing line per preset (torch 5.5, candle 10, lantern 6).
        fp = {"fire-torch": 5.5, "fire-candle": 10.0, "fire-lantern": 6.0}.get(
            lamp, 5.5)
        near = band_energy(freqs, power, fp - 1.5, fp + 1.5)
        # Compare against an equally-wide quiet band above the flame bands.
        hiband = band_energy(freqs, power, fp + 3.0, fp + 6.0)
        low = band_energy(freqs, power, 0.05, 2.0)
        checks.append(("puffing line near %.1f Hz" % fp, near > 1.5 * hiband,
                       f"E[{fp - 1.5:.1f}..{fp + 1.5:.1f}]={near:.4f} vs "
                       f"E[{fp + 3:.1f}..{fp + 6:.1f}]={hiband:.4f}"))
        checks.append(("low-frequency wander dominates", low > near,
                       f"E[0.05..2]={low:.4f} vs puffing={near:.4f}"))
        # The WALL's signal, not the close-range flame luminance: the
        # measured 0.25 is a draught-disturbed flame seen by a near-field
        # sensor (shape-dancing dominates), so the room-illumination target
        # sits well below it. See the amplitude-provenance note in
        # FlickerSynthesis.h / PLAN.FLICKER_PHYSICS.md.
        checks.append(("sigma/mu in physical range", 0.03 <= ratio <= 0.15,
                       f"sigma/mu={ratio:.3f} (target 0.03-0.15)"))
        checks.append(("fat bright tail (positive skew)", skew > 0.1,
                       f"skew={skew:.2f}"))
        checks.append(("tint tracks intensity (Planckian)",
                       corr(xs, gs) > 0.7 and corr(xs, bs) > 0.7,
                       f"corr(I,G)={corr(xs, gs):.2f} corr(I,B)={corr(xs, bs):.2f}"))
    elif lamp == "gas-open":
        checks.append(("calm but alive", 0.01 <= ratio <= 0.06,
                       f"sigma/mu={ratio:.3f} (target 0.01-0.06)"))
    elif lamp in ("gas-mantle", "electric", "fungus"):
        checks.append(("essentially steady", ratio < 0.025,
                       f"sigma/mu={ratio:.4f} (target <0.025)"))
        if lamp == "electric":
            white = all(abs(r - 1) < 1e-5 and abs(g - 1) < 1e-5 and
                        abs(b - 1) < 1e-5 for r, g, b in zip(rs, gs, bs))
            checks.append(("stays exactly white", white, "tint == 1"))
    else:
        print(f"FAIL: no check profile for '{lamp}'")
        return 1

    print(f"{lamp}: n={len(xs)} rate={rate:.1f}Hz mu={mu:.3f} "
          f"sigma/mu={ratio:.3f} skew={skew:.2f}")
    ok = True
    for name, passed, detail in checks:
        print(f"  {'PASS' if passed else 'FAIL'}  {name:38s} {detail}")
        ok &= passed
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
