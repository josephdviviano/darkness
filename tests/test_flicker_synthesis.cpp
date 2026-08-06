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

// FlickerSynthesis invariants (PLAN.FLICKER_PHYSICS.md §8). The spectral
// checks live in analysis/flicker_spectrum.py against flicker_sim output;
// what belongs HERE is what must never regress silently: determinism,
// bounds, classification, and the passthrough contract. The S6 gather's
// cosine sampler rides along at the end — it is the same class of pure,
// statistically-checkable function.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "FlickerSynthesis.h"
#include "LightmapBake.h"

#include <cmath>
#include <vector>

using namespace Darkness;

TEST_CASE("flicker synthesis is deterministic per seed", "[flicker]") {
    const FlickerPreset &p = flickerPreset(LampType::FireTorch);
    FlickerState a, b;
    seedFlicker(a, 42);
    seedFlicker(b, 42);
    for (int i = 0; i < 2000; ++i) {
        const FlickerOutput oa = updateFlicker(a, p, 1.0f / 60.0f);
        const FlickerOutput ob = updateFlicker(b, p, 1.0f / 60.0f);
        REQUIRE(oa.intensity == ob.intensity);
        REQUIRE(oa.tint.x == ob.tint.x);
        REQUIRE(oa.tint.y == ob.tint.y);
        REQUIRE(oa.tint.z == ob.tint.z);
    }

    // Different seeds must decorrelate — a torch wall must not pulse in
    // unison. Identical sequences would mean the seed is ignored.
    FlickerState c;
    seedFlicker(c, 43);
    int differing = 0;
    seedFlicker(a, 42);
    for (int i = 0; i < 500; ++i) {
        const FlickerOutput oa = updateFlicker(a, p, 1.0f / 60.0f);
        const FlickerOutput oc = updateFlicker(c, p, 1.0f / 60.0f);
        if (std::fabs(oa.intensity - oc.intensity) > 1e-6f) ++differing;
    }
    REQUIRE(differing > 250);
}

TEST_CASE("flicker output stays within the documented bounds", "[flicker]") {
    for (int t = 0; t < static_cast<int>(LampType::kCount); ++t) {
        const LampType type = static_cast<LampType>(t);
        if (!lampTypeFlickers(type)) continue;
        const FlickerPreset &p = flickerPreset(type);
        FlickerState s;
        seedFlicker(s, 7u + static_cast<uint32_t>(t));
        for (int i = 0; i < 20000; ++i) {
            const FlickerOutput o = updateFlicker(s, p, 1.0f / 60.0f);
            REQUIRE(o.intensity >= kFlickerMinMult);
            REQUIRE(o.intensity <= kFlickerMaxMult);
            REQUIRE(o.tint.x > 0.0f);
            REQUIRE(o.tint.y > 0.0f);
            REQUIRE(o.tint.z > 0.0f);
        }
    }
}

TEST_CASE("fire flickers, electric holds steady", "[flicker]") {
    auto sigmaOverMu = [](LampType type) {
        const FlickerPreset &p = flickerPreset(type);
        FlickerState s;
        seedFlicker(s, 99);
        std::vector<float> xs;
        for (int i = 0; i < 6000; ++i)
            xs.push_back(updateFlicker(s, p, 1.0f / 60.0f).intensity);
        double mu = 0, var = 0;
        for (float x : xs) mu += x;
        mu /= xs.size();
        for (float x : xs) var += (x - mu) * (x - mu);
        var /= xs.size();
        return std::sqrt(var) / mu;
    };
    const double torch = sigmaOverMu(LampType::FireTorch);
    const double gas = sigmaOverMu(LampType::GasOpen);
    const double elec = sigmaOverMu(LampType::ElectricSteady);
    // The physical ordering, with real gaps: fire >> gas >> electric.
    // Bands target the WALL's illumination signal, not close-range flame
    // luminance — see the amplitude-provenance note in FlickerSynthesis.h.
    REQUIRE(torch > 0.03);
    REQUIRE(torch < 0.15);
    REQUIRE(gas < torch * 0.7);
    REQUIRE(elec < 0.02);
}

TEST_CASE("classification matches the T2 archetype families", "[flicker]") {
    // Names as they appear in DARK.GAM (lowercased by the caller).
    REQUIRE(classifyLampName("torches") == LampType::FireTorch);
    REQUIRE(classifyLampName("mc3torch") == LampType::FireTorch);
    REQUIRE(classifyLampName("torch_flame") == LampType::FireTorch);
    REQUIRE(classifyLampName("fire_flame") == LampType::FireTorch);
    REQUIRE(classifyLampName("extinguishable") == LampType::FireTorch);
    REQUIRE(classifyLampName("candle+stick1") == LampType::FireCandle);
    REQUIRE(classifyLampName("chandelier") == LampType::FireCandle);
    REQUIRE(classifyLampName("lantern") == LampType::FireLantern);
    REQUIRE(classifyLampName("gasflame") == LampType::GasOpen);
    REQUIRE(classifyLampName("gaslightbasic") == LampType::GasMantle);
    REQUIRE(classifyLampName("gaslight_lp") == LampType::GasMantle);
    REQUIRE(classifyLampName("oldelecwalllight") == LampType::ElectricSteady);
    REQUIRE(classifyLampName("oldstreetlamp2") == LampType::ElectricSteady);
    REQUIRE(classifyLampName("minelight") == LampType::ElectricSteady);
    REQUIRE(classifyLampName("hangeleclight") == LampType::ElectricSteady);
    REQUIRE(classifyLampName("lclite") == LampType::ElectricSteady);
    REQUIRE(classifyLampName("oldhangingshort") == LampType::ElectricSteady);
    REQUIRE(classifyLampName("mechredalarm") == LampType::Alarm);
    REQUIRE(classifyLampName("mechbluealarm") == LampType::Alarm);
    REQUIRE(classifyLampName("litemush_fragil") == LampType::Fungus);
    REQUIRE(classifyLampName("windowshade") == LampType::Daylight);
    REQUIRE(classifyLampName("skylight_op") == LampType::Daylight);
    REQUIRE(classifyLampName("gasmine_explode") == LampType::Effect);
    REQUIRE(classifyLampName("gasarrow_hit") == LampType::Effect);
    REQUIRE(classifyLampName("flamesparx") == LampType::Effect);
    // Generic parents must NOT classify — reaching them means unclassified.
    REQUIRE(classifyLampName("lights") == LampType::Unclassified);
    REQUIRE(classifyLampName("object") == LampType::Unclassified);
    REQUIRE(classifyLampName("") == LampType::Unclassified);
    // The passthrough contract.
    REQUIRE_FALSE(lampTypeFlickers(LampType::Alarm));
    REQUIRE_FALSE(lampTypeFlickers(LampType::Daylight));
    REQUIRE_FALSE(lampTypeFlickers(LampType::Effect));
    REQUIRE_FALSE(lampTypeFlickers(LampType::Unclassified));
    REQUIRE(lampTypeFlickers(LampType::FireTorch));
}

TEST_CASE("interpolated output is continuous across sample boundaries",
          "[flicker]") {
    // Smooth transitions doctrine: the largest frame-to-frame step at 60 fps
    // must stay well under a visible pop even for the liveliest preset.
    const FlickerPreset &p = flickerPreset(LampType::FireTorch);
    FlickerState s;
    seedFlicker(s, 1234);
    float prev = updateFlicker(s, p, 1.0f / 60.0f).intensity;
    float maxStep = 0.0f;
    for (int i = 0; i < 20000; ++i) {
        const float cur = updateFlicker(s, p, 1.0f / 60.0f).intensity;
        maxStep = std::max(maxStep, std::fabs(cur - prev));
        prev = cur;
    }
    // A whole gust arriving in one synthesis tick spread over ~2.4 frames:
    // even that stays under ~0.35; a pop of 0.5+ would mean interpolation
    // is broken.
    REQUIRE(maxStep < 0.45f);
}

TEST_CASE("cosine hemisphere sampler has the right statistics", "[flicker][bounce]") {
    // Cosine-weighted sampling about +Z: E[z] = 2/3, all directions in the
    // upper hemisphere, azimuthally unbiased. These are the properties the
    // bounce estimator's math depends on (the pdf cancellation in
    // bakeAtlasWithOverlays' gather comment is only valid if the sampler
    // actually IS cosine-weighted).
    uint32_t rng = 12345u;
    const Vector3 n(0.0f, 0.0f, 1.0f);
    const int N = 200000;
    double sumZ = 0.0, sumX = 0.0, sumY = 0.0;
    for (int i = 0; i < N; ++i) {
        const Vector3 d = cosineHemisphereDir(rng, n);
        REQUIRE(d.z >= 0.0f);
        REQUIRE(std::fabs(glm::length(d) - 1.0f) < 1e-3f);
        sumZ += d.z;
        sumX += d.x;
        sumY += d.y;
    }
    REQUIRE(std::fabs(sumZ / N - 2.0 / 3.0) < 0.01);
    REQUIRE(std::fabs(sumX / N) < 0.01);
    REQUIRE(std::fabs(sumY / N) < 0.01);

    // And about an arbitrary axis: mean direction must align with it.
    const Vector3 axis = glm::normalize(Vector3(0.3f, -0.7f, 0.2f));
    Vector3 mean(0.0f);
    for (int i = 0; i < N; ++i) mean += cosineHemisphereDir(rng, axis);
    mean /= static_cast<float>(N);
    REQUIRE(glm::dot(glm::normalize(mean), axis) > 0.999f);
}

TEST_CASE("bakedAmbientValue matches a seeded accumulator", "[bounce]") {
    // The gather composes ambient AFTER the direct bake; this identity is
    // what keeps a gather-off bake and a compose-path bake equivalent in the
    // linear storage modes.
    BakeFormula f;
    f.fivebit = BakeFormula::FiveBit::Continuous;
    const Vector3 amb(0.08f, 0.07f, 0.06f);
    LumelAccumulator acc;
    acc.seedAmbient(amb, f);
    const Vector3 a = acc.finalize(f);
    const Vector3 b = bakedAmbientValue(amb, f);
    REQUIRE(a.x == Catch::Approx(b.x));
    REQUIRE(a.y == Catch::Approx(b.y));
    REQUIRE(a.z == Catch::Approx(b.z));
}

TEST_CASE("octahedral encoding round-trips at 8-bit precision", "[bounce][spec]") {
    // The direction atlas stores octahedral-encoded unit vectors in two u8
    // channels; the GLSL decoder in lm_specular.sh mirrors octahedralDecode.
    // Worst-case angular error at 8 bits should stay well under 2 degrees —
    // far below anything a specular highlight can show.
    uint32_t rng = 555u;
    float worstDot = 1.0f;
    for (int i = 0; i < 50000; ++i) {
        Vector3 d(bakeRand01(rng) * 2.0f - 1.0f,
                  bakeRand01(rng) * 2.0f - 1.0f,
                  bakeRand01(rng) * 2.0f - 1.0f);
        const float len = glm::length(d);
        if (len < 1e-3f) continue;
        d /= len;
        float u = 0, v = 0;
        octahedralEncode(d, u, v);
        // Quantise exactly as the atlas does.
        u = static_cast<float>(static_cast<uint8_t>(u * 255.0f)) / 255.0f;
        v = static_cast<float>(static_cast<uint8_t>(v * 255.0f)) / 255.0f;
        const Vector3 back = octahedralDecode(u, v);
        worstDot = std::min(worstDot, glm::dot(d, back));
    }
    // cos(2°) ≈ 0.99939
    REQUIRE(worstDot > 0.9993f);
}
