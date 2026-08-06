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

// Physically-grounded lamp flicker — PLAN.FLICKER_PHYSICS.md.
//
// One synthesizer, typed parameter presets. The signal model comes from
// measured flame behaviour (citations in the plan doc):
//
//   m(t) = 1 + puff + wander + gust        (clamped, per-type amplitudes)
//
//  * puff   — the buoyant "puffing" oscillation of a diffusion flame: a
//             narrowband stochastic oscillator. ~10–12 Hz for a candle,
//             falling with flame size (f ≈ 1.5/√D), so torches sit ~5–7 Hz.
//  * wander — an Ornstein–Uhlenbeck (leaky random walk) term reproducing the
//             measured flat-then-rolloff luminance spectrum (flat to ~4 Hz,
//             steep rolloff above; σ/μ ≈ 0.25 for an open flame).
//  * gust   — Poisson draught events with exponential-tailed amplitude and
//             exponential decay: the measured fat BRIGHT tail. Flames answer
//             a draught by flaring, not dimming.
//
// Colour is NOT a second knob: flame light is near-blackbody (~1000–2000 K),
// so the tint rides the same signal along a Planckian ramp — dim moments sag
// toward orange-red, bright moments shift warm-white. Electric and mantle
// lamps dim toward grey instead (their radiator does not cool visibly at
// these timescales), so their colourStrength is zero.
//
// The synthesizer runs at kFlickerRate (25 Hz) and the per-frame call
// smooth-interpolates between samples — the same virtualize-then-interpolate
// doctrine player physics uses, and the practice Quake/Valve's 10 Hz light
// styles validated for decades.
//
// IT NEVER REPLACES AUTHORED BEHAVIOUR: the output is a multiplier applied on
// top of the vintage animation envelope, so scripted switches, tweqs and the
// original modes keep working. Alarm/daylight/effect lights bypass entirely.
// Determinism: state is seeded from the light number, so runs reproduce.

#pragma once

#include "DarknessMath.h"

#include <cmath>
#include <cstdint>
#include <string>

namespace Darkness {

// ── Lamp taxonomy ───────────────────────────────────────────────────────────
//
// Families the T2 gamesys actually carries (verified against DARK.GAM's
// archetype tree: torches/Candle*/GasLightBasic/GasFlame/ElecWallLight/
// Streetlamp*/litemush_fragil/...). Classification walks the object's
// inheritance ancestry and matches names — see classifyLampName().
enum class LampType : uint8_t {
    Unclassified = 0,   // vintage passthrough; counted loudly at load
    FireTorch,          // wall torches, braziers, bonfires, generic flames
    FireCandle,         // candles, chandeliers
    FireLantern,        // enclosed flame — a damped torch
    GasOpen,            // open gas flame (fishtail-burner character)
    GasMantle,          // mantle fixture — near-steady white
    ElectricSteady,     // incandescent-era: essentially steady
    Alarm,              // authored periodic device — passthrough
    Fungus,             // bioluminescence — slow breathing only
    Daylight,           // window/skylight proxies — passthrough
    Effect,             // explosion flashes etc. — passthrough
    kCount
};

inline const char *lampTypeName(LampType t) {
    switch (t) {
    case LampType::Unclassified:   return "unclassified";
    case LampType::FireTorch:      return "fire-torch";
    case LampType::FireCandle:     return "fire-candle";
    case LampType::FireLantern:    return "fire-lantern";
    case LampType::GasOpen:        return "gas-open";
    case LampType::GasMantle:      return "gas-mantle";
    case LampType::ElectricSteady: return "electric";
    case LampType::Alarm:          return "alarm";
    case LampType::Fungus:         return "fungus";
    case LampType::Daylight:       return "daylight";
    case LampType::Effect:         return "effect";
    default:                       return "?";
    }
}

// Does this type get the synthesizer at all? Alarms keep their authored
// square wave, daylight and one-shot effects stay untouched, and an
// unclassified light behaves exactly as vintage (and is counted).
inline bool lampTypeFlickers(LampType t) {
    switch (t) {
    case LampType::Unclassified:
    case LampType::Alarm:
    case LampType::Daylight:
    case LampType::Effect:
        return false;
    default:
        return true;
    }
}

// Name-keyword classification, first match wins. Callers walk the ancestry
// nearest-first (BFS), so a specific ancestor ("GasFlame") is consulted
// before a generic one ("Lights" — deliberately NOT matched; reaching it
// means unclassified). Input must be lowercased.
inline LampType classifyLampName(const std::string &n) {
    auto has = [&](const char *kw) { return n.find(kw) != std::string::npos; };
    if (n.empty()) return LampType::Unclassified;
    // Effects and devices before broader keywords ("gasarrow" carries "gas",
    // "gasmine_explode" too — neither is a lamp).
    if (has("alarm"))                                  return LampType::Alarm;
    if (has("explo") || has("arrow") || has("_hit") ||
        has("sparx") || has("flash"))                  return LampType::Effect;
    if (has("mush"))                                   return LampType::Fungus;
    if (has("window") || has("skylight") || has("shade"))
                                                       return LampType::Daylight;
    // Gas: the open flame before the fixtures.
    if (has("gasflame"))                               return LampType::GasOpen;
    if (has("gas"))                                    return LampType::GasMantle;
    // Electric family names in the T2 gamesys. "lclite" is the building
    // light family; "hanging" catches OldHangingShort/HangingLamp.
    if (has("elec") || has("streetlamp") || has("strlight") ||
        has("paglight") || has("minelight") || has("lightbright") ||
        has("bulb") || has("hanging") || has("lclite") ||
        has("light_on") || has("light_off"))           return LampType::ElectricSteady;
    // Fire family.
    if (has("candle") || has("chandel"))               return LampType::FireCandle;
    if (has("lantern"))                                return LampType::FireLantern;
    if (has("torch") || has("brazier") || has("flame") ||
        has("fire") || has("extinguish"))              return LampType::FireTorch;
    return LampType::Unclassified;
}

// ── Presets ─────────────────────────────────────────────────────────────────
//
// Starting values from PLAN.FLICKER_PHYSICS.md §4; expected to be tuned on
// screen. Amplitudes are FRACTIONS of the authored brightness; the global
// console `flicker_scale` scales every deviation uniformly on top.
struct FlickerPreset {
    float puffAmp   = 0.0f;   // narrowband oscillation amplitude
    float puffFreq  = 0.0f;   // Hz; 0 = no puff term
    float wanderAmp = 0.0f;   // OU stationary standard deviation
    float wanderTau = 1.0f;   // OU relaxation time, seconds
    float gustRate  = 0.0f;   // Poisson arrivals per second
    float gustAmp   = 0.0f;   // mean gust amplitude (exponential tail on top)
    float gustTau   = 0.6f;   // gust decay time, seconds
    float colourStrength = 0.0f; // 0..1 scale on the Planckian ramp
};

inline const FlickerPreset &flickerPreset(LampType t) {
    // AMPLITUDE PROVENANCE (PLAN.FLICKER_PHYSICS.md §"amplitude"): the
    // measured candle σ/μ ≈ 0.25 is biased HIGH for our purposes three ways —
    // the reference flame was deliberately draught-disturbed; a close-range
    // photodiode sees the bright lobe DANCING (shape redistribution, which
    // barely modulates the flux a wall receives — especially the puffing
    // term); and scaling a whole overlay uniformly renders any σ as
    // large-field luminance modulation, exactly where the eye's temporal
    // sensitivity peaks (~5–10 Hz). So the presets target the WALL's signal:
    // fire σ/μ ≈ 0.05–0.10 total, puff cut hardest, gusts kept as the
    // character-carrying events. `flicker_scale` remains the taste dial.
    static const FlickerPreset kNone{};
    // Torch gusts stay the strongest term: an open torch is the most
    // draught-exposed lamp in the game, and the gust tail is what carries
    // the measured bright-skew (the spectrum test asserts it).
    static const FlickerPreset kTorch   {0.030f, 5.5f,  0.055f, 1.0f,
                                         0.15f,  0.22f, 0.7f,   1.0f};
    static const FlickerPreset kCandle  {0.020f, 10.0f, 0.045f, 0.7f,
                                         0.12f,  0.15f, 0.5f,   1.0f};
    static const FlickerPreset kLantern {0.012f, 6.0f,  0.030f, 1.3f,
                                         0.05f,  0.08f, 0.8f,   0.7f};
    static const FlickerPreset kGasOpen {0.010f, 0.0f,  0.030f, 2.0f,
                                         0.02f,  0.06f, 1.0f,   0.3f};
    static const FlickerPreset kMantle  {0.0f,  0.0f,  0.010f, 3.0f,
                                         0.0f,  0.0f,  1.0f,  0.0f};
    static const FlickerPreset kElectric{0.0f,  0.0f,  0.006f, 2.0f,
                                         0.0f,  0.0f,  1.0f,  0.0f};
    static const FlickerPreset kFungus  {0.0f,  0.0f,  0.02f, 5.0f,
                                         0.0f,  0.0f,  1.0f,  0.0f};
    switch (t) {
    case LampType::FireTorch:      return kTorch;
    case LampType::FireCandle:     return kCandle;
    case LampType::FireLantern:    return kLantern;
    case LampType::GasOpen:        return kGasOpen;
    case LampType::GasMantle:      return kMantle;
    case LampType::ElectricSteady: return kElectric;
    case LampType::Fungus:         return kFungus;
    default:                       return kNone;
    }
}

// ── The synthesizer ─────────────────────────────────────────────────────────

// Synthesis rate. 25 Hz covers the torch puffing band (4–7 Hz) properly and
// renders a candle's ~10 Hz as the stochastic shimmer it perceptually is;
// the per-frame call interpolates smoothly between samples.
constexpr float kFlickerRate   = 25.0f;
constexpr float kFlickerDt     = 1.0f / kFlickerRate;
// Deviation clamp: a gust may briefly push past the authored maximum (real
// flames flare), but bounded so object lighting cannot run away.
constexpr float kFlickerMinMult = 0.0f;
constexpr float kFlickerMaxMult = 1.35f;

// Planckian ramp endpoints, relative to the light's AUTHORED colour (which
// is preserved exactly at nominal intensity). Ratios derived from Planck's
// law in the Wien limit at R/G/B ≈ 600/550/450 nm for ~1600 K vs ~1900 K vs
// ~2100 K, peak-normalised. Full strength is deliberately strong; presets
// scale it via colourStrength and the ramp is entered gradually.
constexpr float kPlanckDim[3]    = {1.00f, 0.81f, 0.45f};  // toward 1600 K
constexpr float kPlanckBright[3] = {1.00f, 1.11f, 1.49f};  // toward 2100 K
// Deviation magnitude at which the ramp saturates.
constexpr float kColourRampRange = 0.30f;

struct FlickerOutput {
    float   intensity = 1.0f;    // multiplier on the vintage envelope
    Vector3 tint{1.0f};          // per-channel multiplier (Planckian ramp)
};

struct FlickerState {
    uint32_t rng = 0x9E3779B9u;
    float phase = 0.0f;          // puff oscillator phase, radians
    float puffAmpMod = 1.0f;     // OU-modulated puff amplitude (around 1)
    float wander = 0.0f;         // OU state
    float gustEnv = 0.0f;        // decaying gust envelope
    float accum = 0.0f;          // time into the current synthesis interval
    float prevSample = 1.0f, nextSample = 1.0f;
    Vector3 prevTint{1.0f}, nextTint{1.0f};
};

namespace flickerdetail {

inline float rand01(uint32_t &st) {
    st ^= st << 13; st ^= st >> 17; st ^= st << 5;
    return static_cast<float>(st) * (1.0f / 4294967296.0f);
}
// Irwin–Hall approximation of a standard normal: the sum of four uniforms
// has variance 4/12; scale to unit. Cheap, bounded (|g| ≤ √12 ≈ 3.46), and
// deterministic — all three properties wanted here.
inline float randGauss(uint32_t &st) {
    const float s = rand01(st) + rand01(st) + rand01(st) + rand01(st);
    return (s - 2.0f) * 1.7320508f;
}
// Exponential tail for gust amplitudes (the measured fat bright tail).
inline float randExp(uint32_t &st) {
    const float u = rand01(st);
    return -std::log(u > 1e-6f ? u : 1e-6f);
}

// One synthesis step at kFlickerDt. Returns the new sample (intensity
// multiplier before the global scale) and writes the matching tint.
inline float synthTick(FlickerState &s, const FlickerPreset &p,
                       Vector3 &tintOut) {
    const float dt = kFlickerDt;

    // Ornstein–Uhlenbeck wander: x' = −x/τ + σ·√(2/τ)·white. Discretised
    // per tick; stationary standard deviation = wanderAmp.
    if (p.wanderAmp > 0.0f) {
        const float a = dt / p.wanderTau;
        s.wander += -a * s.wander
                  + p.wanderAmp * std::sqrt(2.0f * a) * randGauss(s.rng);
    }

    // Puff: phase-noisy oscillator with OU-modulated amplitude. The
    // frequency jitter (±10%) and amplitude modulation are what make it a
    // narrowband LINE rather than a pure tone — a real flame's puffing
    // wanders in both.
    float puff = 0.0f;
    if (p.puffAmp > 0.0f && p.puffFreq > 0.0f) {
        const float am = dt / 0.35f;   // amplitude-mod relaxation ~0.35 s
        s.puffAmpMod += -am * (s.puffAmpMod - 1.0f)
                      + 0.45f * std::sqrt(2.0f * am) * randGauss(s.rng);
        if (s.puffAmpMod < 0.0f) s.puffAmpMod = 0.0f;
        if (s.puffAmpMod > 2.2f) s.puffAmpMod = 2.2f;
        const float f = p.puffFreq * (1.0f + 0.10f * randGauss(s.rng));
        s.phase += 6.2831853f * f * dt;
        if (s.phase > 6.2831853f) s.phase -= 6.2831853f;
        puff = p.puffAmp * s.puffAmpMod * std::sin(s.phase);
    }

    // Gusts: Poisson arrivals, exponential-tailed amplitude, exponential
    // decay. A draught FANS a flame — positive excursions — and briefly
    // stirs the puffing too.
    if (p.gustRate > 0.0f) {
        if (rand01(s.rng) < p.gustRate * dt)
            s.gustEnv += p.gustAmp * (0.5f + randExp(s.rng));
        s.gustEnv *= std::exp(-dt / p.gustTau);
        if (s.gustEnv < 1e-4f) s.gustEnv = 0.0f;
    }

    float m = 1.0f + puff + s.wander + s.gustEnv;
    if (m < kFlickerMinMult) m = kFlickerMinMult;
    if (m > kFlickerMaxMult) m = kFlickerMaxMult;

    // Colour rides the same signal: normalised deviation into the Planckian
    // ramp, scaled by the preset's strength. Electric/mantle presets have
    // strength 0 and stay exactly white.
    tintOut = Vector3(1.0f);
    if (p.colourStrength > 0.0f) {
        float d = (m - 1.0f) / kColourRampRange;
        if (d < -1.0f) d = -1.0f;
        if (d >  1.0f) d =  1.0f;
        const float *end = (d < 0.0f) ? kPlanckDim : kPlanckBright;
        const float w = p.colourStrength * std::fabs(d);
        tintOut = Vector3(1.0f + w * (end[0] - 1.0f),
                          1.0f + w * (end[1] - 1.0f),
                          1.0f + w * (end[2] - 1.0f));
    }
    return m;
}

} // namespace flickerdetail

inline void seedFlicker(FlickerState &s, uint32_t seed) {
    // xorshift32 must never see 0; fold the golden ratio in so consecutive
    // light numbers decorrelate immediately.
    s.rng = (seed * 2654435761u) ^ 0x9E3779B9u;
    if (s.rng == 0) s.rng = 0x9E3779B9u;
    // Stagger the synthesis phase so a room of torches doesn't tick its
    // samples on the same frames — spreads both the visual rhythm and the
    // per-frame blend load.
    s.accum = flickerdetail::rand01(s.rng) * kFlickerDt;
}

// Advance by a frame's dt and return the smoothly-interpolated output.
// Synthesis happens at kFlickerRate; between samples the output is
// smoothstep-interpolated, so 25 Hz never shows as stepping.
inline FlickerOutput updateFlicker(FlickerState &s, const FlickerPreset &p,
                                   float dt) {
    s.accum += dt;
    // Catch up over hitches, but never spin: past a handful of ticks the
    // intermediate samples were invisible anyway.
    int guard = 8;
    while (s.accum >= kFlickerDt && guard-- > 0) {
        s.accum -= kFlickerDt;
        s.prevSample = s.nextSample;
        s.prevTint   = s.nextTint;
        s.nextSample = flickerdetail::synthTick(s, p, s.nextTint);
    }
    if (guard <= 0) s.accum = 0.0f;

    float t = s.accum / kFlickerDt;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    const float w = t * t * (3.0f - 2.0f * t);

    FlickerOutput out;
    out.intensity = s.prevSample + (s.nextSample - s.prevSample) * w;
    out.tint      = s.prevTint + (s.nextTint - s.prevTint) * w;
    return out;
}

} // namespace Darkness
