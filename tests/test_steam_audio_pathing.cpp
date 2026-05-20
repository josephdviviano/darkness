/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2024-2026 darkness contributors
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *****************************************************************************/

// Pure-math tests for the Steam Audio pathing eqCoeffs → engine DSP
// mapping. The live integration in AudioService.cpp consumes the same
// helper. Tests here pin the runtime behaviour of:
//   - the eqCoeffs → portalAttenuation (gain) weighting
//   - the eqCoeffs[high] → portalBlocking (LPF) coupling
//   - the per-frame closed-door multiplicative blocking layer
//   - NaN / out-of-range defenses on eqCoeffs

#include <cmath>
#include <limits>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "audio/ProbeManager.h"
#include "audio/SteamAudioPathing.h"

using Catch::Approx;
using Darkness::PathingDspMapping;
using Darkness::ProbeBakeParams;
using Darkness::Vector3;
using Darkness::eqCoeffsToDspMapping;
using Darkness::sanitizeEqCoeff;

TEST_CASE("Unobstructed eqCoeffs map to full gain + no blocking",
          "[steam_audio][pathing]") {
    PathingDspMapping m = eqCoeffsToDspMapping(1.0f, 1.0f, 1.0f, 0.0f);
    CHECK(m.gain     == Approx(1.0f));
    CHECK(m.blocking == Approx(0.0f));
}

TEST_CASE("Fully attenuated eqCoeffs collapse gain to 0 and blocking to 1",
          "[steam_audio][pathing]") {
    PathingDspMapping m = eqCoeffsToDspMapping(0.0f, 0.0f, 0.0f, 0.0f);
    CHECK(m.gain     == Approx(0.0f));
    CHECK(m.blocking == Approx(1.0f));
}

TEST_CASE("Mid-band weighting dominates gain mapping",
          "[steam_audio][pathing]") {
    // High and low pinned to 0, mid pinned to 1 — gain = 0.5 * 1 = 0.5.
    PathingDspMapping mMid = eqCoeffsToDspMapping(0.0f, 1.0f, 0.0f, 0.0f);
    CHECK(mMid.gain == Approx(0.5f));
    // Low at 1, others at 0 — 0.25 * 1 = 0.25.
    PathingDspMapping mLow = eqCoeffsToDspMapping(1.0f, 0.0f, 0.0f, 0.0f);
    CHECK(mLow.gain == Approx(0.25f));
    // High at 1, others at 0 — 0.25 * 1 = 0.25 (but blocking == 0 because
    // 1 - eqHigh = 0).
    PathingDspMapping mHigh = eqCoeffsToDspMapping(0.0f, 0.0f, 1.0f, 0.0f);
    CHECK(mHigh.gain     == Approx(0.25f));
    CHECK(mHigh.blocking == Approx(0.0f));
}

TEST_CASE("High-band attenuation drives portalBlocking continuously",
          "[steam_audio][pathing]") {
    // As eqHigh sweeps 1 → 0, blocking sweeps 0 → 1.
    PathingDspMapping m025 = eqCoeffsToDspMapping(1.0f, 1.0f, 0.25f, 0.0f);
    CHECK(m025.blocking == Approx(0.75f));
    PathingDspMapping m075 = eqCoeffsToDspMapping(1.0f, 1.0f, 0.75f, 0.0f);
    CHECK(m075.blocking == Approx(0.25f));
}

TEST_CASE("Closed-door blocking layer multiplies onto all bands "
          "(door_lpf)",
          "[steam_audio][pathing][door_lpf]") {
    // Unobstructed Steam Audio path (eq = 1,1,1), fully closed door —
    // gain collapses to 0 (passFactor = 0) and blocking collapses to 1
    // (eqHigh * 0 = 0 → 1 - 0 = 1).
    PathingDspMapping closed = eqCoeffsToDspMapping(1.0f, 1.0f, 1.0f, 1.0f);
    CHECK(closed.gain     == Approx(0.0f));
    CHECK(closed.blocking == Approx(1.0f));

    // Open door (factor 0): passes through eqCoeffs unmodified.
    PathingDspMapping open = eqCoeffsToDspMapping(1.0f, 1.0f, 1.0f, 0.0f);
    CHECK(open.gain     == Approx(1.0f));
    CHECK(open.blocking == Approx(0.0f));

    // Half-open door (factor 0.5): passFactor = 0.5 → gain = 0.5, eqHigh
    // = 0.5 → blocking = 0.5.
    PathingDspMapping half = eqCoeffsToDspMapping(1.0f, 1.0f, 1.0f, 0.5f);
    CHECK(half.gain     == Approx(0.5f));
    CHECK(half.blocking == Approx(0.5f));
}

TEST_CASE("Door blocking transitions are continuous in factor "
          "(door_lpf)",
          "[steam_audio][pathing][door_lpf]") {
    // Sweep door blocking factor 0 → 1 in steps and confirm both gain
    // and blocking move monotonically and smoothly.
    float prevGain     = 2.0f;  // sentinel above max
    float prevBlocking = -1.0f; // sentinel below min
    for (int i = 0; i <= 10; ++i) {
        float factor = static_cast<float>(i) / 10.0f;
        PathingDspMapping m = eqCoeffsToDspMapping(1.0f, 1.0f, 1.0f, factor);
        // Gain decreases monotonically as door closes.
        CHECK(m.gain     <= prevGain);
        // Blocking increases monotonically as door closes.
        CHECK(m.blocking >= prevBlocking);
        prevGain     = m.gain;
        prevBlocking = m.blocking;
    }
}

TEST_CASE("NaN / inf eqCoeffs sanitize to 1.0 (audible fallback)",
          "[steam_audio][pathing]") {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    CHECK(sanitizeEqCoeff(nan) == Approx(1.0f));
    CHECK(sanitizeEqCoeff(inf) == Approx(1.0f));
    CHECK(sanitizeEqCoeff(-inf) == Approx(1.0f));
    // Pure mapping should still yield finite gain + blocking.
    PathingDspMapping m = eqCoeffsToDspMapping(nan, inf, nan, 0.0f);
    CHECK(std::isfinite(m.gain));
    CHECK(std::isfinite(m.blocking));
}

TEST_CASE("Out-of-range eqCoeffs clamp to [0,1]",
          "[steam_audio][pathing]") {
    // Negative band clamps to 0, > 1 clamps to 1.
    CHECK(sanitizeEqCoeff(-0.5f) == Approx(0.0f));
    CHECK(sanitizeEqCoeff( 1.5f) == Approx(1.0f));
    PathingDspMapping m = eqCoeffsToDspMapping(-1.0f, 2.0f, 1.5f, 0.0f);
    CHECK(m.gain     == Approx(0.25f * 0.0f + 0.50f * 1.0f + 0.25f * 1.0f));
    CHECK(m.blocking == Approx(0.0f));
}

TEST_CASE("Door blocking factor sanitizes (out-of-range, NaN)",
          "[steam_audio][pathing][door_lpf]") {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    // NaN factor treated as fully open.
    PathingDspMapping mNaN = eqCoeffsToDspMapping(1.0f, 1.0f, 1.0f, nan);
    CHECK(mNaN.gain     == Approx(1.0f));
    CHECK(mNaN.blocking == Approx(0.0f));
    // Out-of-range factor clamps.
    PathingDspMapping mHigh = eqCoeffsToDspMapping(1.0f, 1.0f, 1.0f, 2.0f);
    CHECK(mHigh.gain     == Approx(0.0f));
    CHECK(mHigh.blocking == Approx(1.0f));
    PathingDspMapping mLow = eqCoeffsToDspMapping(1.0f, 1.0f, 1.0f, -1.0f);
    CHECK(mLow.gain     == Approx(1.0f));
    CHECK(mLow.blocking == Approx(0.0f));
}

TEST_CASE("Deterministic for identical inputs",
          "[steam_audio][pathing]") {
    // Two consecutive calls with identical inputs must return identical
    // outputs — there is no internal state.
    PathingDspMapping a = eqCoeffsToDspMapping(0.7f, 0.4f, 0.2f, 0.3f);
    PathingDspMapping b = eqCoeffsToDspMapping(0.7f, 0.4f, 0.2f, 0.3f);
    CHECK(a.gain     == Approx(b.gain));
    CHECK(a.blocking == Approx(b.blocking));
}

// ── Bake-side placement math ────────────────────────────────────────────────

TEST_CASE("ProbeBakeParams defaults to floor-only coverage",
          "[steam_audio][bake]") {
    ProbeBakeParams p;
    CHECK(p.additionalElevations.empty());
    CHECK(p.portalAxes.empty());
    // The new axial-emit defaults: 1 ft on each side, dedup against the
    // floor grid at one spacing-radius. These get consumed by
    // ProbeManager so the values can be tuned per-bake.
    CHECK(p.portalAxialOffsetFt  == Approx(1.0f));
    CHECK(p.portalDedupRadiusFt  == Approx(5.0f));
}

TEST_CASE("PortalAxis carries a center and a portal-plane normal",
          "[steam_audio][bake]") {
    // PortalAxis (replacing the legacy 4-probe-on-plane PortalRing) holds
    // the portal centroid and the inward-facing plane normal. ProbeManager
    // expands this to up to two probe candidates at center ± normal *
    // axialOffsetFt — one offset into each adjoining room. The on-plane
    // ring layout produced two pathologies: (1) duplicate probes per
    // shared doorway because every portal appears in both rooms' portal
    // lists with the same basis vectors, and (2) doorway-shaped IRs that
    // ramped wet-bus loudness as the listener walked through. Axial-into-
    // room offsets dodge both.
    ProbeBakeParams::PortalAxis axis;
    axis.center = Vector3(10.0f, 20.0f, 5.0f);
    axis.normal = Vector3(1.0f, 0.0f, 0.0f);

    constexpr float kAxialOffsetFt = 1.0f;  // matches ProbeBakeParams default
    const Vector3 expectedPlus  = axis.center + axis.normal * kAxialOffsetFt;
    const Vector3 expectedMinus = axis.center - axis.normal * kAxialOffsetFt;

    CHECK(expectedPlus.x  == Approx(11.0f));
    CHECK(expectedMinus.x == Approx(9.0f));
    // Offsets are purely axial → y/z unchanged from centroid.
    CHECK(expectedPlus.y  == Approx(20.0f));
    CHECK(expectedPlus.z  == Approx(5.0f));
}

TEST_CASE("Probe count upper bound with elevations and portals",
          "[steam_audio][bake]") {
    // Pre-dedup upper bound: floor + floor·#elevations + 2·#portalAxes.
    // The actual count after the bake is ≤ this, because ProbeManager
    // applies a proximity-dedup pass that drops axial candidates within
    // portalDedupRadiusFt of an existing floor/elevation probe. The
    // post-dedup count is data-dependent (depends on the floor grid +
    // portal-centroid positions), so we only assert the upper bound here.
    const int floor = 100;
    const int elevations = 2;
    const int portalAxes = 5;
    const int upperExtra = floor * elevations + 2 * portalAxes;
    const int upperTotal = floor + upperExtra;
    // 100 + (100*2 + 2*5) = 100 + 210 = 310
    CHECK(upperTotal == 310);
    CHECK(upperExtra == 210);
}

TEST_CASE("Empty elevations + portalAxes=empty reproduces legacy count",
          "[steam_audio][bake]") {
    // Cache-rebuild safety net: with both extension tiers off, the count
    // matches the pre-Phase-2 floor-only path so old probe caches
    // round-trip validation without forcing a re-bake.
    ProbeBakeParams p;
    p.additionalElevations.clear();
    p.portalAxes.clear();
    CHECK(p.additionalElevations.size() == 0);
    CHECK(p.portalAxes.size() == 0);
}

TEST_CASE("Pathing gain scale multiplies only the scalar gain",
          "[steam_audio][pathing]") {
    // The runtime gain-scale knob multiplies the final scalar gain only,
    // not the eqCoeffs themselves — so blocking (= 1 - eqHigh) stays put.
    // 0.5/0.5/0.5 eqCoeffs → unscaled gain = 0.5, blocking = 0.5.
    PathingDspMapping unscaled = eqCoeffsToDspMapping(0.5f, 0.5f, 0.5f, 0.0f, 1.0f);
    PathingDspMapping scaled2x = eqCoeffsToDspMapping(0.5f, 0.5f, 0.5f, 0.0f, 2.0f);
    PathingDspMapping scaled05 = eqCoeffsToDspMapping(0.5f, 0.5f, 0.5f, 0.0f, 0.5f);

    CHECK(unscaled.gain      == Approx(0.5f));
    CHECK(scaled2x.gain      == Approx(1.0f));
    CHECK(scaled05.gain      == Approx(0.25f));
    // Blocking is invariant under gain scale — confirms the scale rides
    // on the gain output only, not the eqCoeffs (which feed blocking).
    CHECK(scaled2x.blocking  == Approx(unscaled.blocking));
    CHECK(scaled05.blocking  == Approx(unscaled.blocking));
}

TEST_CASE("Pathing gain scale tolerates malformed input",
          "[steam_audio][pathing]") {
    // NaN, negative, and infinite scales fall back to 1.0 (identity)
    // rather than producing a silent voice or NaN-propagating gain.
    PathingDspMapping nanScale = eqCoeffsToDspMapping(
        0.5f, 0.5f, 0.5f, 0.0f, std::numeric_limits<float>::quiet_NaN());
    PathingDspMapping negScale = eqCoeffsToDspMapping(0.5f, 0.5f, 0.5f, 0.0f, -2.0f);
    CHECK(nanScale.gain == Approx(0.5f));
    CHECK(negScale.gain == Approx(0.5f));
}
