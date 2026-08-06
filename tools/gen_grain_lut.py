#!/usr/bin/env python3
"""Derive the film-grain variance constants used by shaders/fs_grain.sc.

The shader needs sqrt(V(u)), the emulsion-granularity amplitude from the
filtered Boolean model of silver-halide coverage (Newson, Delon & Galerne
2017; see .claude/NOTES.FILM_GRAIN.md §4.4-4.5). V(u) is an integral with no
closed form, so the shader carries a fitted approximation instead of a table.

This script exists because those three constants are the only numbers in the
grain pipeline that carry physics, and they were briefly in the source with no
reproducible provenance — the comment pointed at a scratchpad path that did
not survive the session. Run this to re-derive them.

    python3 tools/gen_grain_lut.py

Refit whenever the model changes. The fitted form is

    sqrt(V(u)) ~= A * sqrt(u) * (1-u)^Q * (1 + B*u)

chosen because sqrt(u) is cheaper than a second pow() in the shader and the
exponent lands within 0.4% of 1/2 when left free.
"""
import numpy as np
from scipy.integrate import quad
from scipy.optimize import curve_fit


def w(x):
    """Mean covariogram exponent. 2 beyond two grain radii, where discs can
    no longer overlap and the covariance is exactly zero."""
    if x >= 2.0:
        return 2.0
    return 1.0 + (x * np.sqrt(4.0 - x * x) + 4.0 * np.arcsin(x / 2.0)) / (2.0 * np.pi)


def V(u):
    if u <= 0.0 or u >= 1.0:
        return 0.0
    return 0.5 * quad(lambda x: ((1.0 - u) ** w(x) - (1.0 - u) ** 2) * x,
                      0.0, 2.0, limit=200)[0]


def main():
    us = np.linspace(1e-4, 1.0 - 1e-4, 4000)
    y = np.array([np.sqrt(V(u)) for u in us])

    fit = lambda u, A, Q, B: A * np.sqrt(u) * np.power(1.0 - u, Q) * (1.0 + B * u)
    (A, Q, B), _ = curve_fit(fit, us, y, p0=[0.5, 0.68, 0.05], maxfev=60000)
    err = np.abs(fit(us, A, Q, B) - y)
    peak = us[y.argmax()]

    print("Boundary conditions (must both hold, else the model is wrong):")
    for u in (0.2, 0.5, 0.8):
        cb = (1 - u) ** w(0.0) - (1 - u) ** 2
        print("   C_B(%.1f, 0) = %.6f   u(1-u) = %.6f   %s"
              % (u, cb, u * (1 - u), "OK" if abs(cb - u * (1 - u)) < 1e-12 else "FAIL"))
    print("   C_B(u, >=2) == 0 :", all(abs((1 - u) ** w(2.0) - (1 - u) ** 2) < 1e-15
                                       for u in (0.2, 0.5, 0.8)))
    print()
    print("Physical peak at u = %.5f, V = %.6f, sqrt(V) = %.6f"
          % (peak, V(peak), np.sqrt(V(peak))))
    print()
    print("Fit  sqrt(V(u)) ~= A * sqrt(u) * (1-u)^Q * (1 + B*u)")
    print("   max |error| %.6f  (%.4f%% of peak, %.1fx below an 8-bit step)"
          % (err.max(), 100 * err.max() / y.max(), (1.0 / 255.0) / err.max()))
    print("   endpoints: fit(0) = %.8f   fit(1) = %.8f" % (fit(1e-9, A, Q, B),
                                                           fit(1 - 1e-9, A, Q, B)))
    print()
    print("Paste into shaders/fs_grain.sc:")
    print("#define GRAIN_V_A %.6f" % A)
    print("#define GRAIN_V_Q %.6f" % Q)
    print("#define GRAIN_V_B %.6f" % B)


if __name__ == "__main__":
    main()
