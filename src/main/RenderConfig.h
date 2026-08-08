// RenderConfig.h — YAML + CLI configuration for darknessRender
// Config precedence: CLI flags > YAML config file > hardcoded defaults
#pragma once

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace Darkness {

/// Water tunable bounds — SINGLE SOURCE OF TRUTH.
///
/// Both consumers must read these: the YAML clamp in loadConfig() below, and
/// the debug-console slider registration in DarknessRender.cpp
/// (registerConsoleSettings). They used to be written out twice, by hand, and
/// they disagreed — in both directions:
///
///     setting          YAML max   console max
///     wave_amplitude      10.0        5.0
///     uv_distortion        0.1        0.5
///     water_rotation       1.0        0.5
///
/// Only the YAML-wider rows were a real bug: `water: { wave_amplitude: 8.0 }`
/// boots at 8.0 and renders fine, but DebugConsole::addFloat's setter rejects
/// out-of-range input, so the first console edit strands the value — there is
/// no way back to what the user's own config file asked for. The console-wider
/// row (uv_distortion) is not that bug: every YAML-valid value was already
/// settable. It just offered values that could never be persisted back.
///
/// So: **the YAML clamp is authoritative and the console follows it.** The
/// config file is the persistent contract; the console must not offer a value
/// you cannot save. Values below are therefore the pre-existing YAML bounds,
/// unchanged — this fix moves the console, not the config, and no config that
/// worked before behaves differently now.
///
/// These are not physical limits. To retune, change it HERE; both consumers
/// follow and cannot drift apart again.
namespace WaterRange {
constexpr float kWaveAmplitudeMin = 0.0f, kWaveAmplitudeMax = 10.0f; // console was 5.0
constexpr float kUvDistortionMin  = 0.0f, kUvDistortionMax  = 0.1f;  // console was 0.5
constexpr float kWaterRotationMin = 0.0f, kWaterRotationMax = 1.0f;  // console was 0.5
constexpr float kWaterScrollMin   = 0.0f, kWaterScrollMax   = 1.0f;  // already agreed
} // namespace WaterRange

/// Post-process tunable bounds — SINGLE SOURCE OF TRUTH, same contract as
/// WaterRange above: the YAML clamp is authoritative and the debug-console
/// registration in DarknessRender.cpp reads these same constants, so the two
/// cannot drift apart and the console cannot offer an unsaveable value.
///
/// Ranges are deliberately generous rather than tasteful — these are the
/// limits of what the shader handles sanely, not a recommendation. The
/// identity defaults (exposure 1, brightness 0, contrast 1, saturation 1,
/// gamma 1) sit inside every range.
namespace PostRange {
constexpr float kExposureMin   =  0.0f, kExposureMax   = 8.0f;
constexpr float kBrightnessMin = -1.0f, kBrightnessMax = 1.0f;
constexpr float kContrastMin   =  0.0f, kContrastMax   = 4.0f;
constexpr float kSaturationMin =  0.0f, kSaturationMax = 4.0f;
constexpr float kFilterMin     =  0.0f, kFilterMax     = 1.0f;
constexpr float kGammaMin      =  0.1f, kGammaMax      = 4.0f;
// Bloom. Iteration ceiling must match kMaxBloomIterations in PostProcess.h,
// which sizes the reserved bgfx view-id range.
constexpr int   kBloomIterMin  = 1,     kBloomIterMax  = 4;
constexpr float kBloomBlurMin  =  0.0f, kBloomBlurMax  = 8.0f;
constexpr float kBloomIntenMin =  0.0f, kBloomIntenMax = 8.0f;
// NewDark bloom style. Threshold, saturation and range use NewDark's own
// documented ranges verbatim (new_config_vars.txt:375-384); prescale and
// scale are documented as "0.0 - " (unbounded above), so the ceilings here
// are ours.
constexpr float kNdThresholdMin  = 0.0f, kNdThresholdMax  =   1.0f;
constexpr float kNdPrescaleMin   = 0.0f, kNdPrescaleMax   =   8.0f;
constexpr float kNdScaleMin      = 0.0f, kNdScaleMax      =  20.0f;
constexpr float kNdSaturationMin = 0.0f, kNdSaturationMax =   1.0f;
// Bloom radius as a percentage of the screen diagonal — NewDark's
// bloom_range, whose documented range is 0.0-10.0 and whose default is 2.
// 0 means "no pyramid", which is our default rather than NewDark's; see
// PostProcessSettings::bloomRange. Applies to both styles.
constexpr float kBloomRangeMin   = 0.0f, kBloomRangeMax   =  10.0f;
// Tone-curve shaping. AgX contrast is Godot's parameter (default 1.25);
// Lottes' contrast/shoulder/white are his (shoulder is conventionally just
// under 1 — at exactly 1 the curve loses most of its shoulder character).
// The white floor is above middle grey because the curve solve divides by
// (white^ad - mid^ad) and collapses if they meet.
constexpr float kAgxContrastMin  = 0.60f, kAgxContrastMax  =   2.0f;
constexpr float kLottesConMin    = 0.60f, kLottesConMax    =   3.0f;
constexpr float kLottesShoulderMin = 0.70f, kLottesShoulderMax = 1.0f;
constexpr float kLottesWhiteMin  = 1.00f, kLottesWhiteMax  =   8.0f;
// Film response. Strength is the master; 0 is an exact no-op.
constexpr float kFilmStrengthMin = 0.0f, kFilmStrengthMax =   1.0f;
constexpr float kFilmSatMin      = 0.0f, kFilmSatMax      =   2.0f;
constexpr float kFilmFalloffMin  = 0.25f, kFilmFalloffMax =   6.0f;
// Tints are multiplicative and must not exceed 1 — a tint that brightens
// would undo the gloom the stage exists to create.
constexpr float kFilmTintMin     = 0.0f, kFilmTintMax     =   1.0f;
// Halation strength is generous above 1 because it multiplies the bloom
// texture, whose own level depends on bloom_intensity / bloomscale.
constexpr float kHalationMin     = 0.0f, kHalationMax     =   4.0f;
// Halation threshold is compared against the EXPOSED value. Measured basis:
// the static lightmaps of five retail missions have a median of 0.03-0.09 and
// only 0.9-3.8% of texels exceed 0.50, so useful values sit around 0.4-0.9 and
// 1.0 thresholds out the entire frame. Radius is in output pixels and the
// ceiling is deliberately tight: halation is the LOCAL glow, and the blur
// chain tops out near 13 px anyway (see renderHalation).
constexpr float kHaloThreshMin   = 0.0f, kHaloThreshMax   =   4.0f;
constexpr float kHaloRadiusMin   = 1.0f, kHaloRadiusMax   =  16.0f;
// Physical amplitude. `strength` is dimensionally mu_r/sigma, so the model's
// validity condition sigma >= 3*mu_r reads strength <= 1/3 — and report §4.4's
// sigma_n <= 0.076 ceiling lands in the same place. Stylisation beyond this
// belongs in grain.gain, which is deliberately outside the physical model.
constexpr float kGrainMin        = 0.0f, kGrainMax        = 0.3333f;
constexpr float kGrainGainMin    = 0.0f, kGrainGainMax    =   4.0f;
constexpr float kGrainSizeMin    = 0.25f, kGrainSizeMax   =   8.0f;
// Negative-film shadow weighting, pow(1 - luma, bias). 1 is a straight ramp;
// higher confines the grain to the shadows. Always zero at white.
// 0 IS reachable and means a normally-processed stock (the physical curve
// alone). It was clamped to 0.25 while the shader documented 0 as valid,
// which made the physical baseline impossible to A/B.
constexpr float kGrainBiasMin    = 0.0f, kGrainBiasMax    =   6.0f;
// Lightmap "2X modulate" multiplier. 1.0 is correct for everything we can
// currently load (WR and WRRGB) and is the only value that agrees with the
// object and sky passes. 2.0 is what 32-bit 2X lightmaps want, and becomes a
// per-mission value once WREXT lands (WR-1). Above 2 is beyond anything the
// format describes and is purely a look choice.
constexpr float kLightmapScaleMin = 0.25f, kLightmapScaleMax = 4.0f;
// Sky glow intensity multiplier. Ceiling is generous because the useful
// range depends entirely on what glow_scale a given mission authored, and
// that varies.
constexpr float kSkyGlowMin      = 0.0f, kSkyGlowMax      = 32.0f;
// Sky overbright. Ceiling is modest on purpose: this multiplies what the
// player directly sees, not just the bloom source, so large values wash the
// sky out rather than merely making the moon glow.
constexpr float kSkyOverbrightMin = 1.0f, kSkyOverbrightMax = 4.0f;
// SMAA edge-detection threshold. The floor is not 0: at zero every pixel is
// an edge, the weight pass runs its full search everywhere, and the result is
// both slower and worse. The reference's own presets span 0.05 to 0.15 and it
// documents 0.05 as the useful lower limit.
constexpr float kSmaaThresholdMin = 0.01f, kSmaaThresholdMax = 0.5f;
} // namespace PostRange

/// Light-corona tunable bounds — same single-source-of-truth contract as
/// PostRange and WaterRange: YAML clamps against these and the debug console
/// registers against these, so the two cannot drift.
namespace CoronaRange {
// Intensity multiplies the corona's authored alpha. Above ~1 it pushes the
// glow past unity in the HDR target, which is what makes it bloom — so the
// ceiling is generous, the same reasoning as kSkyGlowMax.
constexpr float kIntensityMin  = 0.0f, kIntensityMax  = 8.0f;
constexpr float kSizeMin       = 0.0f, kSizeMax       = 4.0f;
constexpr float kMaxDistMin    = 0.1f, kMaxDistMax    = 4.0f;
// Physical model's reference distance, WORLD UNITS (not a multiplier). The
// floor is deliberately above zero: at 0 the curve returns radiusNear, which
// is its smallest value, and every corona would collapse. The ceiling is the
// synthesized size curve's own 48-unit reference, past which the curve clamps
// and the knob stops doing anything.
constexpr float kRefDistMin    = 1.0f, kRefDistMax    = 48.0f;
// Occlusion crossfade. 0 means "snap", which is the original engine's own
// behaviour and worth being able to reproduce; the ceiling is well past
// anything that reads as responsive.
constexpr float kFadeMin       = 0.0f, kFadeMax       = 2.0f;
// Rays per visibility test. 1 = centre only (binary, pops on edges); the
// ceiling matches the clamp in updateCoronas().
constexpr int   kSamplesMin    = 1,    kSamplesMax    = 9;
// Coronas re-traced per frame. 0 = no budget, trace every one every frame.
constexpr int   kBudgetMin     = 0,    kBudgetMax     = 512;
// Depth-fade distance, world units. 0 would be a hard silhouette edge; the
// ceiling is well past where the glow stops reading as attached to its lamp.
constexpr float kDepthFadeMin  = 0.0f, kDepthFadeMax  = 32.0f;
constexpr float kDepthOffsetMin = 0.0f, kDepthOffsetMax = 8.0f;
} // namespace CoronaRange

/// Clamp a YAML-supplied value, announcing when the config file asked for
/// something we would not honour. Silently rewriting a user's stated intent is
/// the same class of bug as a silent fallback: the value in the file and the
/// value in the engine disagree, and nothing says so.
inline float clampConfigValue(float v, float lo, float hi, const char *key) {
    if (v < lo || v > hi) {
        const float clamped = (v < lo) ? lo : hi;
        std::fprintf(stderr,
            "[FALLBACK] config: %s = %g is outside [%g, %g] — clamping to %g. "
            "The engine will not use the value your config file specifies.\n",
            key, static_cast<double>(v), static_cast<double>(lo),
            static_cast<double>(hi), static_cast<double>(clamped));
        return clamped;
    }
    return v;
}

/// Read an `[r, g, b]` multiplicative tint into `out`, clamped to [0, 1].
///
/// Shared by every tint the grading stages take. The 1.0 ceiling is the point
/// rather than an arbitrary bound: these tints multiply, so a value above 1
/// would BRIGHTEN the region it applies to, and each of them exists to darken
/// or shift one — a "shadow tint" that lifts the shadows is not a shadow
/// tint. Clamping is announced by clampConfigValue, so an out-of-range value
/// is not silently reinterpreted.
///
/// A malformed sequence leaves `out` at its default and says so, rather than
/// half-applying a partial triple.
inline void readTint(const YAML::Node &parent, const char *key, float out[3],
                     const char *fullKeyForMessages) {
    YAML::Node n = parent[key];
    if (!n)
        return;
    if (!n.IsSequence() || n.size() != 3) {
        std::fprintf(stderr,
            "[FALLBACK] config: %s must be a 3-element sequence [r, g, b] — "
            "ignoring it and leaving the tint at its default.\n",
            fullKeyForMessages);
        return;
    }
    char buf[192];
    for (int i = 0; i < 3; ++i) {
        std::snprintf(buf, sizeof(buf), "%s[%d]", fullKeyForMessages, i);
        out[i] = clampConfigValue(n[i].as<float>(),
                                  PostRange::kFilmTintMin,
                                  PostRange::kFilmTintMax, buf);
    }
}

// All configurable settings for the renderer.
// Defaults match the original hardcoded values.
struct RenderConfig {
    // -- graphics --
    // Filter terminology: both keys end in "_filter" and use string enums.
    //   texture_filter:  point | bilinear | trilinear | anisotropic
    //   lightmap_filter: bilinear | bicubic
    // Internally still stored as small ints to keep the runtime hot path branchless.
    int  filterMode        = 0;     // texture filter: 0=point, 1=bilinear, 2=trilinear, 3=anisotropic
    int  lightmapFiltering = 0;     // lightmap filter: 0=bilinear (default), 1=bicubic
    // Honour the tweq update-rate gates: 184 of the 456 shipped tweq configs
    // run only while their object is on screen, and 9 run only while it is
    // off. Faithful when on. Turn it OFF if the gating ever reads worse than
    // it simulates — a part caught mid-motion as it comes into view. This is a
    // look-versus-fidelity dial, NOT a performance one: the entire tweq step
    // measures 0.054 ms/frame on the heaviest shipped level, so there is
    // nothing here to win back.
    bool tweqVisibilityGating = true;

    bool linearMips        = false; // gamma-correct mipmap generation
    bool sharpMips         = false; // unsharp mask on mip levels

    // -- graphics.render_scale -- internal render resolution
    //
    // The scene renders at `renderHeight` and the composite resolves it to the
    // window. Below native this is the low-resolution "vintage" look (Return
    // of the Obra Dinn, No One Lives Under the Lighthouse); the point filter
    // is what makes it read as chunky pixels rather than a blurry upscale.
    //
    // Deliberately NOT offered as a supersampling knob above native. It
    // multiplies every per-pixel cost, and PLAN.DYNAMIC_LIGHTS.md's deferred
    // G-buffer plus PLAN.VISUALS.md's SSAO and volumetrics are all
    // resolution-proportional — see PLAN.VISUALS.md "Anti-aliasing".
    //
    // Startup-only: the scene target and every view rect size off it.
    int  renderHeight      = 0;      // 0 = native (match the window)
    bool renderPointFilter = true;   // point upscale — the vintage look
    bool renderIntegerScale = true;  // snap to a whole-number upscale ratio

    // -- graphics.antialiasing --
    //
    // 0 = none, 1 = SMAA. Matches AntiAliasMode in PostProcess.h; stored as an
    // int here so RenderConfig stays free of render-module types.
    //
    // Off by default. SMAA is a fixed-cost screen-space pass (~0.2-0.4 ms at
    // 1080p, three fullscreen passes) rather than a multiplier on every
    // per-pixel cost, which is why it is the antialiasing this engine can
    // afford once deferred lighting, SSAO and volumetrics land — see
    // PLAN.VISUALS.md "Anti-aliasing". It needs post_process.enabled, and it
    // is mutually exclusive with a below-native render_scale.
    int   antiAliasMode  = 0;
    float smaaThreshold  = 0.1f;

    // -- graphics.post_process -- (ranges: see PostRange above)
    //
    // Off by default, and the defaults below are the identity transform, so
    // both "disabled" and "enabled at defaults" reproduce the original image.
    // Enabling this routes the scene through a floating-point target before
    // resolving to the display, which is what preserves the >1.0 overbright
    // the lightmap path already produces. Nothing here changes simulation.
    //
    // Names and defaults follow NewDark's software colour-correction vars
    // (d3d_disp_sw_cc_bright 0, _contr 1, _sat 1, _rgbfilter 1 1 1) so the
    // settings mean the same thing they do there.
    bool  postProcess     = false;  // master toggle
    float ppExposure      = 1.0f;   // linear multiplier applied pre-tonemap
    // 0=none (clamp), 1=reinhard, 2=aces, 3=agx, 4=pbrneutral.
    // Mirrors Darkness::ToneMapOperator — keep the two in step.
    int   ppToneMap       = 0;
    float ppBrightness    = 0.0f;   // additive offset, post-tonemap
    float ppContrast      = 1.0f;   // about the 0.5 pivot
    float ppSaturation    = 1.0f;   // 0 = greyscale
    float ppFilterR       = 1.0f;   // RGB colour filter (sepia etc.)
    float ppFilterG       = 1.0f;
    float ppFilterB       = 1.0f;
    float ppGamma         = 1.0f;   // pow() applied last
    // Luminance weighting for the saturation pivot and bloom intensity:
    // 0 = crt (Rec.601-era, original colour intent), 1 = lcd (Rec.709).
    int   ppLumaMode      = 0;

    // Lightmap multiplier — see RuntimeState::lightmapScale. 1.0 is correct:
    // it is what retail 16-bit lightmaps are authored for AND the only value
    // that puts world geometry on the same fullbright reference as objects
    // and sky. Put brightness in `exposure`, which lifts all three together.
    float lightmapScale = 1.0f;

    // -- graphics.post_process.bloom -- (defaults are HPL2's shipped values)
    bool  ppBloom           = false;
    int   ppBloomStyle      = 0;    // 0 = amnesia (HPL2), 1 = newdark
    int   ppBloomIterations = 2;
    float ppBloomBlurSize   = 1.0f;
    float ppBloomIntensity  = 1.0f;
    // NewDark-style-only knobs; defaults are NewDark 1.28's documented ones.
    float ppNdThreshold     = 0.6f;
    float ppNdPrescale      = 1.0f;
    float ppNdScale         = 5.0f;
    float ppNdSaturation    = 0.7f;
    // Glow radius, % of screen diagonal (NewDark's bloom_range). Applies to
    // both styles. 0 = no pyramid, i.e. the pre-VIS-3b chain exactly.
    float ppBloomRange      = 0.0f;

    // -- tone-curve shaping (operator-scoped) --
    float ppAgxContrast     = 1.25f;
    float ppLottesContrast  = 1.30f;
    float ppLottesShoulder  = 0.977f;
    float ppLottesWhite     = 3.0f;

    // -- graphics.post_process.film -- all default to an exact no-op
    float ppFilmStrength    = 0.0f;
    float ppFilmShadowSat   = 0.55f;
    float ppFilmHighSat     = 1.0f;
    float ppFilmShadowTint[3] = { 1.0f, 1.0f, 1.0f };
    float ppFilmHighTint[3]   = { 1.0f, 0.98f, 0.94f };
    float ppFilmFalloff     = 1.5f;

    // -- graphics.post_process.halation / .grain --
    float ppHalationStrength = 0.0f;
    float ppHalationTint[3]  = { 1.0f, 0.32f, 0.12f };
    float ppHalationThreshold = 0.6f;
    float ppHalationRadius    = 10.0f;
    float ppGrainStrength    = 0.0f;
    float ppGrainSize        = 1.5f;
    float ppGrainShadowBias  = 1.0f;
    float ppGrainGain        = 1.0f;

    // -- graphics.sky_glow -- SKYOBJVAR sun/moon glow disc
    bool  skyGlow          = true;
    float skyGlowIntensity = 1.0f;  // multiplies the mission's glow_scale
    float skyOverbright    = 1.0f;  // sky brightness multiplier; >1 lets the
                                    // moon reach the overbright bloom range

    // -- graphics.coronas -- light-corona billboards (ranges: CoronaRange)
    //
    // `enabled` covers BOTH sources; `synthesized` covers only the
    // enhancement. That split matters: P$Corona is an original Thief 2
    // property and drawing it is replication, whereas putting a glow on every
    // visible lamp is something the original engine never did. Turning
    // `synthesized` off leaves exactly the coronas the mission authored —
    // which across retail Thief 2 is one, in MISS5.
    //
    // `synthesized` is read once at load (the corona list is built there), so
    // unlike the rest of this block it is not live-editable.
    bool  coronas             = true;
    bool  coronasSynthesized  = true;
    float coronaIntensity     = 1.0f;
    float coronaSizeScale     = 1.0f;
    float coronaMaxDistScale  = 1.0f;
    // Mirrors Darkness::CoronaDistanceModel — keep the two in step.
    // 0 = engine (original growth to constant apparent size), 1 = physical
    // (constant world radius; the glare shrinks like the lamp does).
    int   coronaDistanceModel = 1;
    float coronaRefDistance   = 20.0f;  // physical model only, world units
    float coronaFadeSeconds   = 0.15f;
    int   coronaTraceSamples  = 5;
    int   coronaTraceBudget   = 32;
    bool  coronaTraceObjects  = true;
    bool  coronaDepthFade     = true;
    float coronaDepthFadeRange = 1.5f;
    float coronaDepthFadeOffset = 0.5f;

    // -- paths -- (CLI flags --res / --schemas override these)
    std::string resPath;     // Thief 2 RES directory containing fam.crf / obj.crf / snd.crf
    std::string schemasPath; // schema directory (.sch / .spc / .arc files)

    // -- water -- (ranges: see WaterRange below)
    float waveAmplitude   = 0.3f;    // vertex Z displacement in world units (0 = flat)
    float uvDistortion    = 0.015f;  // UV wobble strength (0 = no ripple)
    float waterRotation   = 0.015f;  // UV rotation speed in rad/s (0 = no rotation)
    float waterScrollSpeed = 0.05f;  // UV scroll speed in world units/s (0 = no drift)

    // -- audio.performance: engine + reverb throughput knobs --
    // Scope tags:
    //   [GLOBAL]      audio engine / hardware
    //   [DIRECT]      direct path only (no reflections)
    //   [REALTIME]    realtime ray-traced reflections only
    //   [BAKE]        offline probe bake only
    //   [REFLECTIONS] both realtime + baked-probe reflection convolution
    int  audioSampleRate         = 48000; // [GLOBAL] device output sample rate (22050|32000|44100|48000|96000)
    int  audioFrameSize          = 1024;  // [GLOBAL] audio engine frame size in samples (256–4096)
    int  audioSoundCacheMB       = 64;    // [GLOBAL] decoded-audio LRU cache budget (MB)

    // -- audio.engine: device-callback / mixer-thread topology (PR D) --
    // ring_mixer: true = mix graph renders on a dedicated mixer thread into
    // a lock-free ring; the device callback only drains the ring (crackle
    // fix — the render no longer competes with the HAL deadline). false =
    // legacy in-callback rendering (pre-PR-D behavior, kept for A/B).
    bool  audioRingMixer         = true;  // [GLOBAL]
    // ring_margin_ms: ring fill target the mixer thread maintains. <= 0 =
    // auto (two engine blocks, min 21.4 ms). Larger = more scheduling
    // slack, more output latency. (0=auto–500)
    float audioRingMarginMs      = -1.0f; // [GLOBAL]
    int  reflectionRateDivisor   = 2;     // [REFLECTIONS] reflection pipeline rate: 1=full 48kHz, 2=half 24kHz, 4=quarter 12kHz
    int  maxActiveVoices         = 64;    // [GLOBAL] hard cap on simultaneous voices (Dark Engine baseline)
    // [REFLECTIONS] Cap on total reverb voices (realtime + baked combined).
    // CPU governor — every reverb voice runs a per-source convolution
    // regardless of mode. Setting to 0 disables all reverb convolution
    // entirely (fully dry).
    int  reverbVoices            = 16;
    // [REALTIME] Of the reverb voices above, how many may run with
    // realtime ray-traced IRs. 0 = baked-only (recommended; all eligible
    // voices route through baked-probe reverb). Range [0, reverbVoices].
    int  reverbVoicesRealtime    = 0;
    int  reflectionThrottle      = 4;     // [REFLECTIONS] signal the reflection-sim worker every Nth audio loop step (1–32); paces the shared baked-reverb IR refresh in baked-only mode too
    int  simMaxOcclusionSamples  = 32;    // [DIRECT+REFLECTIONS] per-source occlusion sample cap (Steam Audio sim) (4–256)
    // Explicit thread counts for reverb work. `convThreads` are the
    // per-voice convolution workers; `simThreads` are the Steam Audio
    // ray-trace simulator threads. Both default to 0 = auto: total =
    // max(2, hwconc - 2), split into ~35% conv / ~65% sim in baked-only
    // mode (~45% / ~55% with realtime voices) — the sim is the bottleneck
    // in both modes. Set BOTH > 0 to use literal values. If only one is
    // > 0, AudioService emits a warning and falls back to auto for both.
    int   convThreads            = 0;
    int   simThreads             = 0;
    std::string sceneType        = "default"; // [REFLECTIONS] IPL scene backend ("default" or "embree" — embree falls back to default if Steam Audio wasn't built with embree)

    // -- audio.reflections: convolution reverb feel --
    //
    // Bake and realtime parameters are split: the bake runs once per
    // mission and is cached as a .probes file, so it can afford much
    // higher quality settings than the realtime sim, which runs every
    // reflection_throttle audio frames.
    //
    // `ambisonicsOrder` here is the REALTIME order; the bake can record
    // a higher order in `bakeAmbisonicsOrder` since the runtime decoder
    // downmixes higher-order baked IRs to the requested realtime order.

    /// Reflection pipeline is HYBRID-only (Steam Audio's
    /// `IPL_REFLECTIONEFFECTTYPE_HYBRID`): early convolution head
    /// (length = `hybridTransitionTime`) plus a parametric tail driven by
    /// RT60 baked into the probe data. CONVOLUTION and PARAMETRIC modes
    /// were removed — the `reflections.type` YAML key is now deprecated
    /// and emits a [FALLBACK] warning if present.

    bool  realtimeReflections = true;  // master enable for HYBRID reflection pipeline
    float hybridTransitionTime = 1.0f;  // seconds — convolution head length (Steam Audio Unity/Unreal default)
    float hybridOverlapPercent = 0.25f; // fraction of transition_time for crossfade

    int   ambisonicsOrder     = 0;      // realtime ambisonic order (0–3)

    // Realtime simulation params (running every reflection_throttle frames)
    int   realtimeNumRays         = 1024;  // rays per realtime sim step (128–8192)
    int   realtimeNumBounces      = 4;     // bounces per ray (1–8)
    float realtimeDuration        = 1.1f;  // IR duration in seconds (>hybridTransitionTime + 0.1 s margin)
    int   realtimeDiffuseSamples  = 32;    // diffuse scattering samples per bounce (16–256)

    // Offline bake params (run once per mission; cached as .probes files).
    int   bakeNumRays             = 4096;  // rays per bake step (1024–65536)
    int   bakeNumBounces          = 8;     // bake bounces (1–64)
    float bakeDuration            = 1.1f;  // bake IR duration in seconds (>= realtime.duration)
    int   bakeDiffuseSamples      = 256;   // bake diffuse samples (32–4096)
    int   bakeAmbisonicsOrder     = 1;     // bake ambisonic order (0–3)

    // -- audio.probes: baked-probe grid generation --
    // Spacing/height feed bakeProbes(); a denser grid produces smoother reverb
    // interpolation at the cost of ~(spacing_old/spacing_new)^2 disk space and
    // proportionally longer bake time. The defaults match the prior hardcoded
    // values; halve spacing (e.g. 2.5) to test whether residual footstep
    // reverb A/B variance is driven by probe sparsity.
    float audioProbeSpacingFt = 5.0f;  // grid spacing in feet (1.0–20.0; requires re-bake to take effect)
    float audioProbeHeightFt  = 5.0f;  // probe height above floor in feet (0.5–20.0)
    // Extra elevation tier (in feet, above each floor probe) to densify
    // pathing coverage for wall-mounted torches and ceiling lamps. Default
    // {10.0} covers wall-height emitters. Empty = floor-only (legacy).
    std::vector<float> audioProbeElevations = { 10.0f };
    // Bake-time validity filter. Drops any probe candidate that either
    //   (a) doesn't sit inside any room (BSP void / inside solid), or
    //   (b) is within this many engine feet of the nearest room wall.
    // 0 disables the clearance check (the inside-solid check still
    // runs). Higher values prefer well-conditioned IRs over coverage
    // near walls; cranking past ~half the typical corridor width will
    // start rejecting probes in narrow passages. Requires a re-bake.
    float audioProbeMinWallClearanceFt = 5.0f;
    // Elevation-tier sparsity multiplier. Floor probes are binned on a
    // coarser (x, y) grid (binSize = spacing × this) and one elevation
    // probe is placed at each bin's centroid per tier. Default 2.0 =
    // 2×2 binning = 1:4 ratio (~75% fewer elevation probes vs legacy
    // 1:1). Higher values further reduce density; 1.0 restores legacy
    // 1:1 behaviour. Requires a re-bake.
    float audioProbeElevationSparsityMul = 2.0f;
    // Global dedup pass radius (engine feet) applied after all probe
    // placement (floor, elevation, portal, emitter). Probes within this
    // distance of an earlier-kept probe get dropped. Default 2.0 ft is
    // conservative — catches obvious overlaps without eating into the
    // 5 ft grid spacing. 0 = dedup disabled. Requires a re-bake.
    float audioProbeGlobalDedupRadiusFt = 2.0f;

    // -- audio.pathing_probes: sparse ROOM_PORTAL pathing batch --
    //
    // Splits pathing probes onto a separate Steam Audio probe batch
    // (sparse ROOM_PORTAL graph: one probe per room centroid, two per
    // portal). Reflection probes remain dense (UNIFORMFLOOR + elevation
    // + portal axes + emitter) for high-quality IR sampling.
    //
    // The cost of Steam Audio's `findAlternatePaths` is roughly quadratic
    // in probe count when dynamic geometry (e.g. door OBBs) invalidates
    // baked paths at runtime; a sparse pathing batch keeps that cost
    // microsecond-scale. Disable to revert to single-batch baking (the
    // .probes file will then contain only the reflection batch and
    // runtime pathing falls back to the synthetic-bypass branch).
    bool audioPathingProbesEnabled = true;

    // Proximity dedup radius for the PATHING batch only (engine feet).
    // Applied after all pathing-candidate emission (portal, centroid,
    // emitter). 10 ft handles the typical compound-doorway / sub-room
    // clusters in Thief 2 levels without dropping legitimately distinct
    // probes. 0 disables the pass. Separate from `global_dedup_radius_ft`,
    // which only affects the dense reflection batch.
    float audioPathingDedupRadiusFt = 10.0f;

    // EXPERIMENTAL single-edge visRange override for the pathing bake
    // (`audio.pathing_probes.vis_range_override_ft`). 0 (default) = use
    // the coverage-derived cap (governing x margin, clamped — see
    // AudioService::prepareProbeBakeParams). Nonzero = force the bake's
    // IPLPathBakeParams::visRange to EXACTLY this value, bypassing the
    // derivation and its clamps. A/B lever for the range sweep the
    // offline §37 analysis motivates (~80 ft keeps the aperture graph
    // healthy at a fraction of the edges); recorded in the .probes
    // header like the derived value, so runtime follows automatically.
    // Requires --force-pathing-bake to take effect on an existing cache
    // (deliberately no auto-rebake: experimental knob).
    float audioPathingVisRangeOverrideFt = 0.0f;

    // Pathing probe layout density tier (`audio.pathing_probes.density`).
    // Valid values:
    //   "baseline" — Tier 0: the original Dark Engine room/portal
    //                graph's nodes (1 per room centroid + 1 per non-door
    //                portal center + door flanking pairs + emitter
    //                mirrors). Selectable — fast dev bakes / low-end.
    //   "bends"    — Tier 1 (default): baseline, with every non-door
    //                portal's center probe replaced by a flanking pair —
    //                explicit solver bend points at each opening.
    //   ("high" is RESERVED for a future Tier 2 — room-span subdivision
    //    for long halls — and is rejected at parse until it exists.)
    // WHY bends is the default (user decision 2026-07-12, supersedes the
    // 2026-07-11 baseline flip): fidelity first — flanking pairs at every
    // aperture give the solver explicit bend points, and they measured
    // BETTER worst-case door spikes. The baseline flip rested on
    // misattributed numbers: re-review at ns8 found bends' worst
    // door-spike window = 316.8 ms vs baseline's 535-696 ms (baseline's
    // spikes were hidden by log rate-limiting; the "387-700 ms" figure
    // pinned on bends was old scatter). Baseline still wins median
    // pathing (p50 66 vs 85 ms) and bake time (17.7 vs 28.4 min at ns8),
    // which is why it remains selectable rather than removed.
    // Kept as the validated string; mapped to the PathingProbeDensity
    // enum at the AudioService boundary (DarknessRender.cpp). Recorded
    // in the .probes v4 header — changing it triggers a loud automatic
    // pathing-only re-bake on next run.
    std::string audioPathingDensity = "bends";

    // Force a fresh pathing bake even when the existing .probes file
    // already contains a valid pathing section. The loaded reflection IR
    // section is carried forward unchanged (pathing-only re-bake), so
    // this is fast — useful for iterating on pathing-bake parameters
    // (e.g. dedup radius) without paying the reflection bake.
    //
    // Default false. No YAML key by design — this is a per-invocation
    // override. See PLAN.AUDIO_PROFILING.md §4.3.
    bool forcePathingBake = false;

    // Bake-quality profile: true = `--bake-quality dev`. Besides the
    // reflection-bake overrides applied directly in the CLI parser, this
    // flag selects the pathing visibility sampling constant
    // (SteamAudioPathing.h kPathingVisSamplesDev vs Ship — both 4 since
    // 2026-07-11, split retained structurally) for BOTH
    // the bake and the runtime pathing simulator — one flag, both sides,
    // so they cannot diverge within a run. Cross-run cache mismatches
    // are caught against the .probes v3 header (automatic pathing-only
    // re-bake). No YAML key by design — per-invocation profile.
    bool devBakeProfile = false;

    // -- audio.occlusion: occlusion + material scaling --
    // (diffuseSamples / bakeDiffuseSamples moved to realtimeDiffuseSamples /
    //  bakeDiffuseSamples under reflections — legacy occlusion.diffuse_samples
    //  / occlusion.bake_diffuse_samples keys still parsed with deprecation warning.)
    float occlusionRadius     = 5.0f;  // volumetric occlusion sphere radius (world units = feet, 0.3–30)
    int   occlusionSamples    = 16;    // ray samples per source for occlusion gradient (4–64)
    float transmissionScale   = 1.0f;  // multiplier for material transmission (1=physical, 10=through-walls game-friendly)
    float absorptionScale     = 1.0f;  // multiplier for material absorption (1=physical, <1=more reflective)

    // -- audio.propagation: cell-graph sound routing + door blocking --
    bool  portalRouting       = true;   // portal-graph sound routing through doorways
    bool  probePathing        = true;   // baked probe diffraction (when available)
    float propagationMaxDist  = 200.0f; // max sound propagation distance through portal graph (world units)
    float doorLpfOpenHz       = 20000.0f; // LPF cutoff for fully open door (Hz)
    float doorLpfBlockedHz    = 800.0f;   // LPF cutoff for fully blocked door (Hz)
    // Floor on propagation/portal scale. Default 0.0 disables the floor
    // (was 0.001; the FP-noise reasoning was speculative — left in place
    // as a config knob in case it needs to be re-enabled). With smooth
    // occlusion ramps, attenuation naturally reaches 0 without dropouts.
    float propMinAttenuation  = 0.0f;
    // N-path BFS: how many simultaneous portal-graph paths to keep per
    // listener room. 1 = single shortest path; 2 = original Dark Engine
    // (the per-room propagation record's second predecessor slot); 3+ = modernized. Clamped to [1, 4].
    uint32_t propMaxPaths     = 2;
    // Alternates kept only if their effective distance is within this
    // many world units of the primary. Matches the original engine's
    // distance-difference cap default = 10. Clamped to [0, 50].
    float    propMaxPathDiff  = 10.0f;
    // Multiplier on the scalar gain produced by Steam Audio's baked
    // pathing eqCoeffs. 1.0 = identity. Use > 1 to make through-portal
    // sound louder than the bake implies, without re-baking. Does NOT
    // affect the LPF blocking factor. Clamped to [0.1, 10.0].
    float    pathingGainScale = 1.0f;
    // Companion knob: multiplier on the LPF blocking factor emitted
    // by eqCoeffsToDspMapping. Legacy mapping (blocking = 1 − eqHigh)
    // produces blocking ≈ 0.98 for typical cross-room ambients
    // (eqHigh ≈ 0.02), pegging the door-LPF cutoff at ~400 Hz and
    // making distant voices unrecognisably muffled. Lower this
    // (e.g. 0.3-0.5) to keep the LPF more open. Clamped to [0.0, 1.0].
    float    pathingBlockingScale = 1.0f;
    // Minimum interval (seconds) between successive Steam Audio
    // pathing-simulation updates. iplSimulatorRunPathing is CPU-heavy
    // and runs on the PathingSimulator worker thread; this interval
    // sets how often the worker is signalled. Since staging went
    // event-driven, an idle due tick stages nothing — the interval is
    // door-event quantization delay, not a pacing knob; 0.05 s halves
    // the 0.1 s Unity/Unreal-default door-event latency at measured
    // near-zero cost (2026-07-19 survey). 0.0 = run every frame
    // (legacy / A-B diagnostic). Clamped to [0.0, 1.0] seconds.
    // (Default aligned with AudioService::mPathingUpdateInterval — this
    // value overwrites the service default at init wiring, so the two
    // must agree or a config without the key silently reverts.)
    float    pathingUpdateInterval = 0.05f;

    // Router-gated search (PLAN.PATHING_DESIGN.md §49 lever 1): suppress
    // a Steam Audio pathing solve whenever the hybrid gate's route says
    // the voice is unreachable — SA's findAlternatePaths would drain the
    // entire reachable probe component (~16-36 BVH rays per edge, the
    // measured 170-745 ms [PATHING_SLOW] class) only to return the same
    // no-route verdict the volume gate already silences the voice on.
    // Default ON; the switch exists for A/B measurement only.
    bool     pathingRouterGate = true;

    // No-route movement damping (PLAN.PATHING_DESIGN.md §53 lever A):
    // multiplier on the 5 ft solve-memo movement threshold for voices
    // whose retained Steam Audio verdict is no-route. Their
    // movement-triggered re-discovery is a full-component drain repeated
    // every 5 ft of listener travel (measured: 24-45 per 45 s MISS7
    // tour); attachment/visibility changes on the scale of probe
    // spacing, so 4x (20 ft) re-checks are enough — room changes and the
    // quiet-gated door trigger still fire discovery immediately.
    // 1 = damping off (A/B). Clamped [1, 16].
    float    pathingNoRouteMoveMul = 4.0f;

    // Time constant (ms) for the audio-thread smoother on pathing EQ/SH
    // parameters. Steam Audio's built-in PathEffect ramps are frame-
    // count based and collapse to ~20-60 ms at our small buffers, so a
    // fresh solve with a large level change (door between loud/quiet
    // spaces) steps audibly; this smoother is time-based and frame-size
    // independent. 0 disables (verbatim application, Valve-plugin
    // behavior). Clamped to [0, 1000].
    float    pathingSmoothingMs = 100.0f;

    // Per-band weights for collapsing Steam Audio's 3-band eqCoeffs into
    // the scalar portalAttenuation gain. Applied as
    //   gain = wL·eqLow + wM·eqMid + wH·eqHigh
    // then scaled by pathingGainScale. Default {0.25, 0.50, 0.25}
    // matches the legacy mid-heavy perceptual weighting (roughly
    // A-weighted). Weights are NOT auto-normalised — sums other than
    // 1.0 produce a flat boost/cut. Each component clamped to [0, 1].
    // Typical tunings:
    //   {0.25, 0.50, 0.25} default mid-heavy
    //   {0.50, 0.40, 0.10} bass-heavy — fuller cross-room sounds
    //   {0.33, 0.34, 0.33} flat — highs contribute equally to loudness
    //   {0.10, 0.40, 0.50} treble-heavy — muffled = quiet
    float    pathingGainWeightLow  = 0.25f;
    float    pathingGainWeightMid  = 0.50f;
    float    pathingGainWeightHigh = 0.25f;

    // -- audio.spatialization: HRTF + distance model --
    float hrtfVolume          = 1.0f;   // HRTF output gain (1.0 = raw HRTF, lower = quieter)
    std::string hrtfInterpolation = "bilinear"; // HRTF interpolation: "nearest" or "bilinear"
    float spatialBlend        = 1.0f;   // binaural blend (0=mono, 1=full HRTF)

    // -- audio.ambient: P$AmbientHack tuning --
    //
    // Voice lifecycle (Group D — 2026-05): spawn fires on `dist < amb.radius`
    // (no hysteresis multiplier; the original-engine spawn boundary), halt
    // fires on a dB-based audibility threshold rather than the Euclidean
    // stop boundary. Halt is preceded by a `halt_fade_out_ms` ramp so the
    // discrete stopVoice event is perceptually hidden; spawn is followed by
    // a `spawn_fade_in_ms` ramp so the first audio callback's silent
    // initVoiceDSP defaults don't pop. See AmbientSoundManager.cpp for the
    // audibility derivation (mapping.gain × distanceAttenuation × schema_cb).
    int   ambDefaultPriority    = 64;   // priority for ambients without explicit value
    // [AMBIENT] volume ramp 0→full on voice spawn (0–2000 ms). 150 ms hides
    // the spawn pop without making the appearance feel laggy.
    int   ambientSpawnFadeInMs  = 150;
    // [AMBIENT] volume ramp full→0 before halt-stop (0–2000 ms). 250 ms
    // hides the halt pop; the voice keeps running through the fade so a
    // late audibility recovery can cancel and ramp back up.
    int   ambientHaltFadeOutMs  = 250;
    // [AMBIENT] dB level below which voice is considered "inaudible" and
    // halt is triggered (-80 to -20). −50 dB ≈ 0.00316 linear; well below
    // the perceptual floor of a stationary listener in a typical Thief
    // mission ambient mix.
    float ambientHaltAudibilityThresholdDb = -50.0f;
    // [AMBIENT] consecutive frames below threshold required to trigger
    // halt (5–600). At 60 Hz, 30 frames ≈ 0.5 s — long enough to ignore
    // single-frame eq-flicker dips, short enough that a player walking
    // away from a source halts the voice within a beat.
    int   ambientHaltBelowThresholdFrames = 30;
    // Per-voice spatialBlend override for AMB_ENVIRONMENTAL ambients (room
    // tone, wind, church reverberance). 1.0 = full HRTF point-source pan;
    // 0.0 = mono passthrough (no directional cue). Object-attached ambients
    // (no AMB_ENVIRONMENTAL flag) keep full HRTF at 1.0 regardless.
    // Default 0.3 = mostly diffuse with a subtle directional hint, so
    // "wind from outside" still leans in the right direction but doesn't
    // feel like a laser pointer.
    float ambEnvironmentalSpatialBlend = 0.3f;  // (0.0–1.0)
    // Global linear multiplier applied to every ambient voice's per-frame
    // volume. Compensates for the loudness re-baseline
    // introduced when Steam Audio became the sole player-audio propagation
    // authority (centibel falloff curve over schema radius retired). One
    // knob, default 1.0 = no change. Tune by ear.
    float ambGlobalVolumeScale = 1.0f;  // (0.0–4.0)

    // -- audio.mixer: global gains --
    float mixerMasterGain     = 1.0f;   // global output gain multiplier
    float mixerDirectGain     = 1.0f;   // dry-bus multiplier (direct path only)
    float mixerReflectionGain = 1.0f;   // reverb wet bus gain multiplier
    float reflectionRampMs    = 10.0f;  // reflection-bus fade-in time (ms) on first activation

    // -- audio.dsp: master bus DSP chain --
    bool  dspLimiter         = true;    // soft tanh limiter (prevents digital clipping)
    float dspLimiterKnee     = 0.8f;    // knee threshold (0.5–0.95, higher = later onset)
    bool  dspCompressor      = true;    // master bus compressor (tames transients)
    float dspCompThreshold   = -15.0f;  // compressor threshold in dBFS (-30 to 0)
    float dspCompRatio       = 3.0f;    // compression ratio (1.5 to 10)
    float dspCompAttackMs    = 10.0f;   // compressor attack time (1–100 ms)
    float dspCompReleaseMs   = 250.0f;  // compressor release time (50–2000 ms)
    bool  dspEQ              = true;    // low-shelf EQ (adds bass weight)
    float dspEQFreq          = 120.0f;  // EQ center frequency in Hz (60–500)
    float dspEQGain          = 3.0f;    // EQ gain in dB (-6 to +6)
    float dspEQQ             = 0.707f;  // EQ filter Q (0.3–2.0; 0.707 = Butterworth)
    bool  dspDucking         = false;   // ambient ducking when SFX plays (disabled by default)
    float dspDuckAmount      = 0.5f;    // ambient volume during ducking (0.1–1.0)
    float dspDuckAttackMs    = 50.0f;   // ducking attack time (10–500 ms)
    float dspDuckReleaseMs   = 500.0f;  // ducking release time (50–5000 ms)
    bool  dspWetSaturation       = false;  // wet-bus tape/phonograph saturator (off = transparent)
    float dspWetSaturationDrive  = 1.0f;   // saturation drive (1.0 = brick-wall only, 2-4 = tape, 5-10 = phonograph)


    // -- physics --
    int physicsRate = 60;  // physics timestep Hz: 12 = vintage (12.5Hz), 60 = modern, 120 = ultra

    // -- developer --
    bool showObjects      = true;   // render object meshes
    bool showFallbackCubes = false; // show colored cubes for objects with missing models
    bool portalCulling    = true;   // portal/frustum culling
    bool cameraCollision  = false;  // sphere collision against world geometry
    bool debugObjects     = false;  // dump per-object filtering diagnostics to stderr
    // --rebake-lightmaps: recompute the whole lightmap atlas from the mission's
    // own light table at load (PLAN.HIGH_RES_SHADOWS S0) so it can be compared
    // against the shipped one in game via the `rebaked_lightmaps` console
    // toggle. CLI-only and deliberately not a YAML key: it is a debug
    // instrument with a multi-second load cost, not a setting.
    bool rebakeLightmaps  = false;
    int  rebakeDensity    = 2;      // lumels per shipped lumel, per axis
    int  rebakeSamples    = 16;     // emitter samples; 1 = original hard shadows
    float rebakeEmitter   = 0.75f;  // emitter radius, world units
    int  rebakeSupersample = 2;     // receiver-side box supersampling factor
    // S6: hemisphere gather over the direct bake — one pass computes both.
    // Bounce = cosine-weighted single-bounce gather (albedo × direct at the
    // hit); AO scales the AMBIENT term by hemisphere openness (occlusion
    // within a short physical range). 0 disables either.
    int   rebakeBounce    = 32;     // gather rays per lumel; 0 = direct-only
    float rebakeAO        = 0.6f;   // ambient occlusion strength, 0..1
    // Falloff naturalisation: widen each cell's light list by N portal hops
    // (the baked lists otherwise CUT lights at cell seams), and fade
    // radius-limited lights over the last fraction instead of a hard ring.
    int   rebakeReach     = 1;      // portal hops; 0 = original lists exactly
    float rebakeSoftRadius = 0.15f; // fade fraction; 0 = original hard edge
    // Physical falloff: finite-emitter inverse-square 1/(r²+a²) anchored to
    // the original's illuminance at rebakeFalloffAnchor, with light reach
    // determined by GEOMETRY (sub-quantisation spheres + occlusion probes)
    // instead of the authored cell lists. The physically-rooted answer to
    // hard light borders — the falloff terminates on its own.
    bool  rebakeFalloffPhysical = true;
    float rebakeFalloffAnchor   = 8.0f;
    // Throw-derived intensity: anchor each light at THIS fraction of its
    // authored throw (explicit radius, else per-cell-list reach), clamped
    // to [anchor, kThrowAnchorMax]. Recovers per-light intensity the 1/r
    // authoring encoded as reach; 0 = single global anchor (the pre-throw
    // physical look). Default set from the shipped-atlas luminance-ratio
    // sweep (PLAN.HIGH_RES_SHADOWS falloff section).
    float rebakeThrowAlpha      = 0.5f;
    // S3 door shadows: door-adjacent lights become overlays and door
    // leaves occlude their bake rays; door events re-bake those overlays
    // live. Physical-falloff mode only. Kill switch for A/B and for
    // isolating bake-cost regressions.
    bool  rebakeDoorShadows     = true;
    // DEV-ONLY --stress-doors-cycles N: stop the door-stress harness after
    // N toggle cycles (0 = forever). Exists so "system at rest afterwards"
    // is a reproducible measurement window.
    int   stressDoorsCycles     = 0;
    // S1 shadow-map oracle (PLAN.HIGH_RES_SHADOWS §S1). CLI-only like the
    // rebake family — promotion to YAML waits for the knobs to stabilise
    // (handoff §4.7). Face size is the per-face edge in texels; the
    // cross-check renders a pool of lights at startup and compares N random
    // (point, light) face lookups against raycastWorld (0 = don't run).
    int shadowFaceSize        = 256;
    int shadowCrossCheckPairs = 0;
    // --door-diff-diag <objID>: S4c artifact harness — for one door,
    // compare the mid-swing differential result against the settled CPU
    // truth over its cone polys; histograms + panel images under
    // doorshadow_diag/. 0 = off.
    int  doorDiffDiag         = 0;
    // --door-swing-diag <objID> [--door-swing-steps N]: the TRAJECTORY
    // harness. Same comparison as --door-diff-diag but swept across the
    // swing instead of sampled at one pose, and with the promoted lights
    // pushed through the REAL uniform transport (pack -> decode) rather
    // than handed straight to the lookup. Catches pose-dependent faults
    // (visible as a hump in the per-step curve) and slot-encoding faults,
    // neither of which a single-pose, transport-free sample can see.
    // 0 = off.
    int  doorSwingDiag        = 0;
    int  doorSwingSteps       = 8;
    // --lumel-bake-test: the S2 GPU engine's acceptance — GPU-bake one
    // door overlay and diff it against the CPU bake of the same rect.
    bool lumelBakeTest        = false;
    // Door-event re-bake path. GPU is the default since 2026-08-08.
    //
    // It was parked in favour of the CPU ray path because the GPU bake
    // missed thin/grazing WORLD occluders and grew grazing acne
    // (gpuMissedOccluder=148 / gpuPhantomShadow=171 on door 403). Both
    // causes were found and fixed rather than worked around:
    //   * every PCF tap was compared against the CENTRE fragment's
    //     distance, so a grazing receiver self-shadowed at the kernel
    //     edges. Taps are now depth-referenced to where their own ray
    //     meets the receiver plane — exact, because WR polygons are
    //     planar. Measured on door 407: phantom 341 -> 15.
    //   * the depth bias was a flat half world unit, which is thicker
    //     than a door leaf and was swallowing occluders whole. It is now
    //     distance-proportional, because a face texel subtends a fixed
    //     ANGLE. Measured: gross error 193 -> 77, and balanced (39
    //     phantom / 38 missed) instead of lopsided.
    // The bake target is RGBA16F like the rest of the pipeline, so the
    // path no longer quantises what it writes.
    //
    // The CPU ray path remains as --rebake-cpu-events: it is still the
    // arbiter the diagnostics measure against, and the fallback if a
    // backend's readback misbehaves.
    bool rebakeCpuEvents      = false;
    // DEV-ONLY --stress-frob-obj "a,b": send FrobWorldEnd to exactly these
    // object IDs every ~3 s (5 s warmup) through ScriptManager — the same
    // message FrobSystem::executeFrob sends, so lever→ControlDevice→script
    // chains run headlessly. Built to reproduce "switch 434 doesn't toggle
    // the cathedral lights" without a mouse.
    std::string stressFrobObjs;
    bool stepLog          = false;  // stair step diagnostics to stderr ([STEP] prefix)
    // Muted stderr log channels (comma-separated tag list, e.g.
    // "ODE,TWEQ"). Gated tags: ODE (awake-body physics dump), TWEQ /
    // TWEQ_MODEL (tweq activation + model-swap traces), PATH_RAW (audio
    // pathing solver raw output). Console: log_ode / log_tweq /
    // log_tweq_model / log_path_raw toggle them live. [FALLBACK] lines
    // are never muteable.
    std::string logMute   = "ODE";
    bool togglePlatforms  = false;  // auto-activate all moving terrain at startup
    bool noProbes         = false;  // skip probe baking (no spatial audio)
    bool audioLog         = false;  // enable audio/sound/schema log output

    // -- audio perf-capture (PLAN.AUDIO_PROFILING.md §1.2, §1.4) --
    // Label used to tag the per-run JSONL artifact directory.
    //   ./perf/<mission>/<utc_iso>__<perf_label>/audio_perf.jsonl
    // Defaults to "default" so an un-labelled invocation still produces an
    // artifact. Must be filesystem-safe (alphanumeric + '_-.') — the binary
    // validates and rejects on launch.
    std::string perfLabel = "default";
    // Walltime budget (seconds) from main() start. 0 = disabled (run forever
    // until SDL quit). Set via --exit-after-seconds N to let scripted sweeps
    // (tools/perf_sweep.sh) run unattended.
    float       exitAfterSeconds = 0.0f;
    // Capture the engine's final stereo f32 output to output.wav in the
    // per-run perf directory (next to audio_perf.jsonl). Listenable per-run
    // evidence for A/B comparisons; analyzed offline by
    // tools/wav_artifacts.py (PLAN.AUDIO_PERF.md PR 0.2). Set via
    // --capture-wav or YAML developer.capture_wav.
    bool        captureWav = false;

    // -- auto-fly probe-tour (companion to --exit-after-seconds) --
    // Drives the fly-mode camera through a deterministic random tour of the
    // N nearest pathing probes. Without movement, the listener position is
    // stationary and the JSONL perf artifact reflects only one geometric
    // configuration; with auto-fly enabled every sweep iteration captures
    // the same flythrough trajectory. See src/main/AutoFlyTour.h.
    //
    // Activation forces fly mode (physics off) on first frame after init
    // because the physics integrator owns camera position when on.
    bool        autoFly             = false;     // --auto-fly
    float       autoFlySpeed        = 10.0f;     // --auto-fly-speed (ft/s)
    int         autoFlyWaypoints    = 50;        // --auto-fly-waypoints
    uint32_t    autoFlySeed         = 0xC0FFEEu; // --auto-fly-seed
    float       autoFlyPauseSec     = 0.0f;      // --auto-fly-pause-sec

    // -- audio capture point (spin-in-place acoustic probe) --
    // --audio-capture x,y,z pins the listener at a fixed world point
    // (Dark Engine feet, Z-up), forces physics off, enables audio_log, and
    // spins the camera in place for `audioCaptureSeconds` completing
    // `audioCaptureRotations` full turns before exiting cleanly. Lets us do a
    // full-azimuth Steam Audio capture at a chosen point with no manual
    // navigation. See src/main/AudioCaptureSpin.h. Mutually exclusive with
    // --auto-fly (capture wins if both are given).
    bool        audioCapture          = false;   // --audio-capture x,y,z
    float       audioCaptureX         = 0.0f;     // target X
    float       audioCaptureY         = 0.0f;     // target Y
    float       audioCaptureZ         = 0.0f;     // target Z
    float       audioCaptureSeconds   = 15.0f;    // --audio-capture-seconds
    float       audioCaptureRotations = 3.0f;     // --audio-capture-rotations

    // -- auto-run probe-tour (on-foot stress harness) --
    // Physics-mode sibling of --auto-fly: the player RUNS a deterministic
    // waypoint tour of nearby pathing probes via the real movement-intent
    // API, generating footstep voices (~2.5 spawns/s at run speed), BSP
    // collision, and portal crossings — the audio stress profile a flying
    // camera cannot produce. See src/main/AutoRunTour.h. Mutually
    // exclusive with --auto-fly (auto-run wins with a warning) and
    // --audio-capture (capture wins).
    bool        autoRun             = false;      // --auto-run
    int         autoRunWaypoints    = 50;         // --auto-run-waypoints
    uint32_t    autoRunSeed         = 0xC0FFEEu;  // --auto-run-seed
    std::string autoRunSpeedMode    = "run";      // --auto-run-speed-mode run|walk|creep

    // -- audio sample-selection RNG seed --
    // Seeds AudioService's schema-sample-selection PRNG so two A/B stress
    // runs pick the SAME wav per schema event (sample choice varies clip
    // length → voice lifetime → voice-count profile). -1 = unseeded
    // (std::random_device, the default shipping behavior).
    int64_t     audioRngSeed        = -1;         // --audio-rng-seed

    // -- door-swing stress harness (DEV-ONLY) --
    // --stress-doors N toggles the N doors nearest the camera (of the
    // doors with a usable audio OBB) open/closed every ~2 s during a run.
    // Exists solely to exercise the O2a door-dirty-gated pathing
    // re-solve path and its [DOOR_ROUTE_LATENCY] staleness metric
    // (PLAN.PATHING_DESIGN.md §6 decision 1) under a scripted swing load
    // that a hands-off tour can't produce. Bypasses the frob/script
    // layer (DoorSystem::activate directly), so no door foley schemas
    // fire — intentional: the metric under test is route updates of
    // OTHER voices, not door sounds. 0 = disabled (the default; never
    // ship-enabled).
    int         stressDoors         = 0;          // --stress-doors N

    // --stress-door-ids "a,b,..." (DEV-ONLY, diagnostic companion to
    // --stress-doors): toggle EXACTLY these door object IDs every cycle
    // instead of the N nearest the camera. Exists for the door-event
    // pathing-latency hypothesis runs: single-door runs pinned to a door
    // of a known alternate-route detour class (NONE/LONG/SHORT — see
    // analysis/door_detour_class.py) need the same door swinging all run,
    // which the distance-ranked selection can't guarantee on a moving
    // tour. Empty = disabled; when set it overrides the nearest-N pick
    // (a nonzero --stress-doors is still required to arm the harness).
    std::vector<int32_t> stressDoorIDs;           // --stress-door-ids "a,b"

    // --spawn-override "x,y,z[,yaw]" (DEV-ONLY, diagnostic companion to
    // --stress-doors / --auto-run): force the camera/player start to an
    // explicit engine-feet position instead of the mission's spawn
    // marker. Positioned runtime measurements (e.g. the door-stress-
    // while-standing-in-a-hub-room cell from PLAN.PATHING_DESIGN.md §10)
    // need the listener parked in a SPECIFIC room; nothing ships a
    // teleport. Applied loudly ([SPAWN_OVERRIDE] banner) right after
    // initRuntimeState in DarknessRender.cpp. Never ship-enabled.
    // --spawn-at-door <objID> (DEV-ONLY): park the camera a few feet from
    // that door, facing it. Same purpose as --spawn-override but without
    // needing coordinates — verifying a door swing otherwise meant walking
    // across the map on every run. 0 = off.
    int         spawnAtDoor         = 0;          // --spawn-at-door <objID>
    bool        spawnOverride       = false;      // --spawn-override "x,y,z[,yaw]"
    float       spawnOverrideX      = 0.0f;
    float       spawnOverrideY      = 0.0f;
    float       spawnOverrideZ      = 0.0f;
    float       spawnOverrideYaw    = 0.0f;
};

// Result of CLI parsing — values that are CLI-only (not in YAML).
struct CliResult {
    const char* misPath    = nullptr;  // positional arg: mission file
    std::string resPath;               // --res <path>
    std::string schemasPath;           // --schemas <path>
    std::string configPath;            // --config <path>
    bool        helpRequested = false; // --help / -h
};

// Load settings from a YAML config file into cfg.
// Returns true if the file was loaded successfully.
// Returns false (silently) if the file doesn't exist — this is the normal case.
// Prints to stderr and returns false on parse errors.
inline bool loadConfigFromYAML(const std::string& path, RenderConfig& cfg) {
    // Check if file exists before trying to parse
    FILE* f = std::fopen(path.c_str(), "r");
    if (!f) return false;
    std::fclose(f);

    try {
        YAML::Node root = YAML::LoadFile(path);

        // paths section — fallback values used when --res / --schemas not given
        if (YAML::Node paths = root["paths"]) {
            if (paths["res"])     cfg.resPath     = paths["res"].as<std::string>();
            if (paths["schemas"]) cfg.schemasPath = paths["schemas"].as<std::string>();
        }

        // graphics section — both filter knobs use string enums for symmetry.
        if (YAML::Node gfx = root["graphics"]) {
            if (gfx["texture_filter"]) {
                std::string val = gfx["texture_filter"].as<std::string>();
                if      (val == "point")        cfg.filterMode = 0;
                else if (val == "bilinear")     cfg.filterMode = 1;
                else if (val == "trilinear")    cfg.filterMode = 2;
                else if (val == "anisotropic")  cfg.filterMode = 3;
                else                            cfg.filterMode = 0; // unknown → default
            }
            if (gfx["lightmap_filter"]) {
                std::string val = gfx["lightmap_filter"].as<std::string>();
                if      (val == "bicubic")  cfg.lightmapFiltering = 1;
                else                        cfg.lightmapFiltering = 0; // "bilinear" or unknown → default
            }
            if (gfx["lightmap_scale"]) {
                cfg.lightmapScale = clampConfigValue(
                    gfx["lightmap_scale"].as<float>(),
                    PostRange::kLightmapScaleMin, PostRange::kLightmapScaleMax,
                    "graphics.lightmap_scale");
            }
            if (gfx["tweq_visibility_gating"])
                cfg.tweqVisibilityGating = gfx["tweq_visibility_gating"].as<bool>();
            if (gfx["linear_mips"])  cfg.linearMips = gfx["linear_mips"].as<bool>();
            if (gfx["sharp_mips"])   cfg.sharpMips  = gfx["sharp_mips"].as<bool>();

            // graphics.msaa was removed 2026-08-05, not deprecated: it is
            // incompatible with the deferred G-buffer in
            // PLAN.DYNAMIC_LIGHTS.md and it costs the readable depth buffer
            // that the corona fade, SSAO and volumetrics all need (bgfx will
            // not resolve a multisampled depth attachment on any backend).
            // Announce a stale key rather than ignoring it, so a config that
            // still sets it does not look like it is being honoured.
            if (gfx["msaa"]) {
                std::fprintf(stderr,
                    "[FALLBACK] config: graphics.msaa is no longer supported "
                    "and is being ignored — MSAA was removed because it "
                    "cannot coexist with a readable depth buffer or a "
                    "deferred G-buffer. Use graphics.render_scale for the "
                    "vintage look; SMAA/TAA are the planned antialiasing "
                    "(see PLAN.VISUALS.md). Delete the key to silence this.\n");
            }

            // sky_glow — SKYOBJVAR sun/moon glow disc
            if (YAML::Node sg = gfx["sky_glow"]) {
                if (sg["enabled"]) cfg.skyGlow = sg["enabled"].as<bool>();
                if (sg["intensity"]) {
                    cfg.skyGlowIntensity = clampConfigValue(
                        sg["intensity"].as<float>(),
                        PostRange::kSkyGlowMin, PostRange::kSkyGlowMax,
                        "graphics.sky_glow.intensity");
                }
                if (sg["sky_overbright"]) {
                    cfg.skyOverbright = clampConfigValue(
                        sg["sky_overbright"].as<float>(),
                        PostRange::kSkyOverbrightMin, PostRange::kSkyOverbrightMax,
                        "graphics.sky_glow.sky_overbright");
                }
            }

            // render_scale — internal resolution + how it reaches the window
            if (YAML::Node rs = gfx["render_scale"]) {
                if (rs["height"]) {
                    const int v = rs["height"].as<int>();
                    if (v == 0 || (v >= 60 && v <= 4320)) {
                        cfg.renderHeight = v;
                    } else {
                        std::fprintf(stderr,
                            "[FALLBACK] config: graphics.render_scale.height = "
                            "%d is not 0 (native) or within 60-4320 — using "
                            "native resolution.\n", v);
                        cfg.renderHeight = 0;
                    }
                }
                if (rs["filter"]) {
                    const std::string v = rs["filter"].as<std::string>();
                    if      (v == "point")  cfg.renderPointFilter = true;
                    else if (v == "linear") cfg.renderPointFilter = false;
                    else {
                        std::fprintf(stderr,
                            "[FALLBACK] config: graphics.render_scale.filter = "
                            "'%s' is not point|linear — using point.\n",
                            v.c_str());
                        cfg.renderPointFilter = true;
                    }
                }
                if (rs["integer"])
                    cfg.renderIntegerScale = rs["integer"].as<bool>();
            }

            // antialiasing — the resolve that runs after the composite
            if (YAML::Node aa = gfx["antialiasing"]) {
                if (aa["mode"]) {
                    const std::string v = aa["mode"].as<std::string>();
                    if      (v == "none") cfg.antiAliasMode = 0;
                    else if (v == "smaa") cfg.antiAliasMode = 1;
                    else {
                        std::fprintf(stderr,
                            "[FALLBACK] config: graphics.antialiasing.mode = "
                            "'%s' is not none|smaa — using none. (TAA is on "
                            "the roadmap, not in this build.)\n", v.c_str());
                        cfg.antiAliasMode = 0;
                    }
                }
                if (aa["smaa_threshold"]) {
                    cfg.smaaThreshold = clampConfigValue(
                        aa["smaa_threshold"].as<float>(),
                        PostRange::kSmaaThresholdMin,
                        PostRange::kSmaaThresholdMax,
                        "graphics.antialiasing.smaa_threshold");
                }
            }

            // coronas — light-corona billboards (authored P$Corona +
            // synthesized glows on visible light-emitting objects)
            if (YAML::Node co = gfx["coronas"]) {
                if (co["enabled"]) cfg.coronas = co["enabled"].as<bool>();
                if (co["synthesized"])
                    cfg.coronasSynthesized = co["synthesized"].as<bool>();
                if (co["intensity"]) {
                    cfg.coronaIntensity = clampConfigValue(
                        co["intensity"].as<float>(),
                        CoronaRange::kIntensityMin, CoronaRange::kIntensityMax,
                        "graphics.coronas.intensity");
                }
                if (co["size_scale"]) {
                    cfg.coronaSizeScale = clampConfigValue(
                        co["size_scale"].as<float>(),
                        CoronaRange::kSizeMin, CoronaRange::kSizeMax,
                        "graphics.coronas.size_scale");
                }
                if (co["max_distance_scale"]) {
                    cfg.coronaMaxDistScale = clampConfigValue(
                        co["max_distance_scale"].as<float>(),
                        CoronaRange::kMaxDistMin, CoronaRange::kMaxDistMax,
                        "graphics.coronas.max_distance_scale");
                }
                if (co["distance_model"]) {
                    std::string val = co["distance_model"].as<std::string>();
                    if      (val == "engine")   cfg.coronaDistanceModel = 0;
                    else if (val == "physical") cfg.coronaDistanceModel = 1;
                    else {
                        std::fprintf(stderr,
                            "[FALLBACK] config: graphics.coronas.distance_model "
                            "= '%s' is not one of engine|physical — using "
                            "'physical'. Coronas will hold a constant world "
                            "size rather than the engine's growth.\n",
                            val.c_str());
                        cfg.coronaDistanceModel = 1;
                    }
                }
                if (co["physical_reference_distance"]) {
                    cfg.coronaRefDistance = clampConfigValue(
                        co["physical_reference_distance"].as<float>(),
                        CoronaRange::kRefDistMin, CoronaRange::kRefDistMax,
                        "graphics.coronas.physical_reference_distance");
                }
                if (co["fade_seconds"]) {
                    cfg.coronaFadeSeconds = clampConfigValue(
                        co["fade_seconds"].as<float>(),
                        CoronaRange::kFadeMin, CoronaRange::kFadeMax,
                        "graphics.coronas.fade_seconds");
                }
                if (co["trace_samples"]) {
                    cfg.coronaTraceSamples = static_cast<int>(clampConfigValue(
                        static_cast<float>(co["trace_samples"].as<int>()),
                        static_cast<float>(CoronaRange::kSamplesMin),
                        static_cast<float>(CoronaRange::kSamplesMax),
                        "graphics.coronas.trace_samples"));
                }
                if (co["depth_fade"])
                    cfg.coronaDepthFade = co["depth_fade"].as<bool>();
                if (co["depth_fade_range"]) {
                    cfg.coronaDepthFadeRange = clampConfigValue(
                        co["depth_fade_range"].as<float>(),
                        CoronaRange::kDepthFadeMin, CoronaRange::kDepthFadeMax,
                        "graphics.coronas.depth_fade_range");
                }
                if (co["depth_fade_offset"]) {
                    cfg.coronaDepthFadeOffset = clampConfigValue(
                        co["depth_fade_offset"].as<float>(),
                        CoronaRange::kDepthOffsetMin, CoronaRange::kDepthOffsetMax,
                        "graphics.coronas.depth_fade_offset");
                }
                if (co["trace_objects"])
                    cfg.coronaTraceObjects = co["trace_objects"].as<bool>();
                if (co["trace_budget"]) {
                    cfg.coronaTraceBudget = static_cast<int>(clampConfigValue(
                        static_cast<float>(co["trace_budget"].as<int>()),
                        static_cast<float>(CoronaRange::kBudgetMin),
                        static_cast<float>(CoronaRange::kBudgetMax),
                        "graphics.coronas.trace_budget"));
                }
            }

            // post_process subsection — HDR scene target + composite resolve.
            if (YAML::Node pp = gfx["post_process"]) {
                if (pp["enabled"]) cfg.postProcess = pp["enabled"].as<bool>();

                if (pp["tonemap"]) {
                    std::string val = pp["tonemap"].as<std::string>();
                    if      (val == "none")       cfg.ppToneMap = 0;
                    else if (val == "reinhard")   cfg.ppToneMap = 1;
                    else if (val == "aces")       cfg.ppToneMap = 2;
                    else if (val == "agx")        cfg.ppToneMap = 3;
                    else if (val == "pbrneutral") cfg.ppToneMap = 4;
                    else if (val == "lottes")     cfg.ppToneMap = 5;
                    else {
                        // Unlike the older enum keys above, announce this
                        // rather than defaulting in silence — a typo here
                        // would otherwise look like "tone mapping does
                        // nothing on my machine".
                        std::fprintf(stderr,
                            "[FALLBACK] config: graphics.post_process.tonemap = "
                            "'%s' is not one of none|reinhard|aces|agx|"
                            "pbrneutral|lottes — using 'none'. Tone mapping "
                            "will not be applied.\n",
                            val.c_str());
                        cfg.ppToneMap = 0;
                    }
                }

                if (pp["exposure"]) {
                    cfg.ppExposure = clampConfigValue(
                        pp["exposure"].as<float>(),
                        PostRange::kExposureMin, PostRange::kExposureMax,
                        "graphics.post_process.exposure");
                }
                if (pp["brightness"]) {
                    cfg.ppBrightness = clampConfigValue(
                        pp["brightness"].as<float>(),
                        PostRange::kBrightnessMin, PostRange::kBrightnessMax,
                        "graphics.post_process.brightness");
                }
                if (pp["contrast"]) {
                    cfg.ppContrast = clampConfigValue(
                        pp["contrast"].as<float>(),
                        PostRange::kContrastMin, PostRange::kContrastMax,
                        "graphics.post_process.contrast");
                }
                if (pp["saturation"]) {
                    cfg.ppSaturation = clampConfigValue(
                        pp["saturation"].as<float>(),
                        PostRange::kSaturationMin, PostRange::kSaturationMax,
                        "graphics.post_process.saturation");
                }
                if (pp["gamma"]) {
                    cfg.ppGamma = clampConfigValue(
                        pp["gamma"].as<float>(),
                        PostRange::kGammaMin, PostRange::kGammaMax,
                        "graphics.post_process.gamma");
                }
                if (pp["luma_mode"]) {
                    std::string val = pp["luma_mode"].as<std::string>();
                    if      (val == "crt") cfg.ppLumaMode = 0;
                    else if (val == "lcd") cfg.ppLumaMode = 1;
                    else {
                        std::fprintf(stderr,
                            "[FALLBACK] config: graphics.post_process.luma_mode "
                            "= '%s' is not one of crt|lcd — using 'crt'.\n",
                            val.c_str());
                        cfg.ppLumaMode = 0;
                    }
                }

                // bloom subsection
                if (YAML::Node bl = pp["bloom"]) {
                    if (bl["enabled"]) cfg.ppBloom = bl["enabled"].as<bool>();
                    if (bl["style"]) {
                        std::string val = bl["style"].as<std::string>();
                        if      (val == "amnesia") cfg.ppBloomStyle = 0;
                        else if (val == "newdark") cfg.ppBloomStyle = 1;
                        else {
                            std::fprintf(stderr,
                                "[FALLBACK] config: graphics.post_process."
                                "bloom.style = '%s' is not one of "
                                "amnesia|newdark — using 'amnesia'.\n",
                                val.c_str());
                            cfg.ppBloomStyle = 0;
                        }
                    }
                    if (bl["threshold"]) {
                        cfg.ppNdThreshold = clampConfigValue(
                            bl["threshold"].as<float>(),
                            PostRange::kNdThresholdMin, PostRange::kNdThresholdMax,
                            "graphics.post_process.bloom.threshold");
                    }
                    if (bl["prescale"]) {
                        cfg.ppNdPrescale = clampConfigValue(
                            bl["prescale"].as<float>(),
                            PostRange::kNdPrescaleMin, PostRange::kNdPrescaleMax,
                            "graphics.post_process.bloom.prescale");
                    }
                    if (bl["scale"]) {
                        cfg.ppNdScale = clampConfigValue(
                            bl["scale"].as<float>(),
                            PostRange::kNdScaleMin, PostRange::kNdScaleMax,
                            "graphics.post_process.bloom.scale");
                    }
                    if (bl["saturation"]) {
                        cfg.ppNdSaturation = clampConfigValue(
                            bl["saturation"].as<float>(),
                            PostRange::kNdSaturationMin, PostRange::kNdSaturationMax,
                            "graphics.post_process.bloom.saturation");
                    }
                    if (bl["iterations"]) {
                        cfg.ppBloomIterations = static_cast<int>(clampConfigValue(
                            static_cast<float>(bl["iterations"].as<int>()),
                            static_cast<float>(PostRange::kBloomIterMin),
                            static_cast<float>(PostRange::kBloomIterMax),
                            "graphics.post_process.bloom.iterations"));
                    }
                    if (bl["blur_size"]) {
                        cfg.ppBloomBlurSize = clampConfigValue(
                            bl["blur_size"].as<float>(),
                            PostRange::kBloomBlurMin, PostRange::kBloomBlurMax,
                            "graphics.post_process.bloom.blur_size");
                    }
                    if (bl["intensity"]) {
                        cfg.ppBloomIntensity = clampConfigValue(
                            bl["intensity"].as<float>(),
                            PostRange::kBloomIntenMin, PostRange::kBloomIntenMax,
                            "graphics.post_process.bloom.intensity");
                    }
                    if (bl["range"]) {
                        cfg.ppBloomRange = clampConfigValue(
                            bl["range"].as<float>(),
                            PostRange::kBloomRangeMin, PostRange::kBloomRangeMax,
                            "graphics.post_process.bloom.range");
                    }
                }

                // Tone-curve shaping. Each is read only by its own operator;
                // they sit at the post_process level rather than under a
                // per-operator map because the console has to expose them
                // flat anyway.
                if (pp["agx_contrast"]) {
                    cfg.ppAgxContrast = clampConfigValue(
                        pp["agx_contrast"].as<float>(),
                        PostRange::kAgxContrastMin, PostRange::kAgxContrastMax,
                        "graphics.post_process.agx_contrast");
                }
                if (pp["lottes_contrast"]) {
                    cfg.ppLottesContrast = clampConfigValue(
                        pp["lottes_contrast"].as<float>(),
                        PostRange::kLottesConMin, PostRange::kLottesConMax,
                        "graphics.post_process.lottes_contrast");
                }
                if (pp["lottes_shoulder"]) {
                    cfg.ppLottesShoulder = clampConfigValue(
                        pp["lottes_shoulder"].as<float>(),
                        PostRange::kLottesShoulderMin,
                        PostRange::kLottesShoulderMax,
                        "graphics.post_process.lottes_shoulder");
                }
                if (pp["lottes_white"]) {
                    cfg.ppLottesWhite = clampConfigValue(
                        pp["lottes_white"].as<float>(),
                        PostRange::kLottesWhiteMin, PostRange::kLottesWhiteMax,
                        "graphics.post_process.lottes_white");
                }

                // film subsection — the response stage after the curve.
                if (YAML::Node fl = pp["film"]) {
                    if (fl["strength"]) {
                        cfg.ppFilmStrength = clampConfigValue(
                            fl["strength"].as<float>(),
                            PostRange::kFilmStrengthMin,
                            PostRange::kFilmStrengthMax,
                            "graphics.post_process.film.strength");
                    }
                    if (fl["shadow_saturation"]) {
                        cfg.ppFilmShadowSat = clampConfigValue(
                            fl["shadow_saturation"].as<float>(),
                            PostRange::kFilmSatMin, PostRange::kFilmSatMax,
                            "graphics.post_process.film.shadow_saturation");
                    }
                    if (fl["highlight_saturation"]) {
                        cfg.ppFilmHighSat = clampConfigValue(
                            fl["highlight_saturation"].as<float>(),
                            PostRange::kFilmSatMin, PostRange::kFilmSatMax,
                            "graphics.post_process.film.highlight_saturation");
                    }
                    if (fl["tone_falloff"]) {
                        cfg.ppFilmFalloff = clampConfigValue(
                            fl["tone_falloff"].as<float>(),
                            PostRange::kFilmFalloffMin,
                            PostRange::kFilmFalloffMax,
                            "graphics.post_process.film.tone_falloff");
                    }
                    readTint(fl, "shadow_tint", cfg.ppFilmShadowTint,
                             "graphics.post_process.film.shadow_tint");
                    readTint(fl, "highlight_tint", cfg.ppFilmHighTint,
                             "graphics.post_process.film.highlight_tint");
                }

                // halation — warm bleed borrowed from the bloom pyramid.
                if (YAML::Node ha = pp["halation"]) {
                    if (ha["strength"]) {
                        cfg.ppHalationStrength = clampConfigValue(
                            ha["strength"].as<float>(),
                            PostRange::kHalationMin, PostRange::kHalationMax,
                            "graphics.post_process.halation.strength");
                    }
                    if (ha["threshold"]) {
                        cfg.ppHalationThreshold = clampConfigValue(
                            ha["threshold"].as<float>(),
                            PostRange::kHaloThreshMin, PostRange::kHaloThreshMax,
                            "graphics.post_process.halation.threshold");
                    }
                    if (ha["radius"]) {
                        cfg.ppHalationRadius = clampConfigValue(
                            ha["radius"].as<float>(),
                            PostRange::kHaloRadiusMin, PostRange::kHaloRadiusMax,
                            "graphics.post_process.halation.radius");
                    }
                    readTint(ha, "tint", cfg.ppHalationTint,
                             "graphics.post_process.halation.tint");
                }

                // grain
                if (YAML::Node gr = pp["grain"]) {
                    if (gr["strength"]) {
                        cfg.ppGrainStrength = clampConfigValue(
                            gr["strength"].as<float>(),
                            PostRange::kGrainMin, PostRange::kGrainMax,
                            "graphics.post_process.grain.strength");
                    }
                    // Renamed from `shadow_bias`: it is one factor of the
                    // amplitude now, not the whole weighting, and 0 is a
                    // meaningful value (normally-processed stock) where the
                    // old key's floor was 0.25.
                    if (gr["shadow_bias"]) {
                        std::fprintf(stderr,
                            "[FALLBACK] config: graphics.post_process.grain."
                            "shadow_bias was renamed to `push` — it is now one "
                            "factor of the amplitude rather than the whole "
                            "weighting, and 0 is valid (a normally-processed "
                            "stock). The value here is IGNORED; move it to "
                            "`push`.\n");
                    }
                    if (gr["push"]) {
                        cfg.ppGrainShadowBias = clampConfigValue(
                            gr["push"].as<float>(),
                            PostRange::kGrainBiasMin, PostRange::kGrainBiasMax,
                            "graphics.post_process.grain.push");
                    }
                    if (gr["gain"]) {
                        cfg.ppGrainGain = clampConfigValue(
                            gr["gain"].as<float>(),
                            PostRange::kGrainGainMin, PostRange::kGrainGainMax,
                            "graphics.post_process.grain.gain");
                    }
                    if (gr["size"]) {
                        cfg.ppGrainSize = clampConfigValue(
                            gr["size"].as<float>(),
                            PostRange::kGrainSizeMin, PostRange::kGrainSizeMax,
                            "graphics.post_process.grain.size");
                    }
                }

                // rgb_filter: [r, g, b] — matches NewDark's
                // d3d_disp_sw_cc_rgbfilter taking three values.
                if (YAML::Node rgb = pp["rgb_filter"]) {
                    if (rgb.IsSequence() && rgb.size() == 3) {
                        cfg.ppFilterR = clampConfigValue(rgb[0].as<float>(),
                            PostRange::kFilterMin, PostRange::kFilterMax,
                            "graphics.post_process.rgb_filter[0]");
                        cfg.ppFilterG = clampConfigValue(rgb[1].as<float>(),
                            PostRange::kFilterMin, PostRange::kFilterMax,
                            "graphics.post_process.rgb_filter[1]");
                        cfg.ppFilterB = clampConfigValue(rgb[2].as<float>(),
                            PostRange::kFilterMin, PostRange::kFilterMax,
                            "graphics.post_process.rgb_filter[2]");
                    } else {
                        std::fprintf(stderr,
                            "[FALLBACK] config: graphics.post_process.rgb_filter "
                            "must be a 3-element sequence [r, g, b] — ignoring "
                            "it and leaving the filter at white (1, 1, 1).\n");
                    }
                }
            }
        }

        // water section
        if (YAML::Node water = root["water"]) {
            if (water["wave_amplitude"]) {
                cfg.waveAmplitude = clampConfigValue(
                    water["wave_amplitude"].as<float>(),
                    WaterRange::kWaveAmplitudeMin,
                    WaterRange::kWaveAmplitudeMax, "water.wave_amplitude");
            }
            if (water["uv_distortion"]) {
                cfg.uvDistortion = clampConfigValue(
                    water["uv_distortion"].as<float>(),
                    WaterRange::kUvDistortionMin,
                    WaterRange::kUvDistortionMax, "water.uv_distortion");
            }
            if (water["rotation_speed"]) {
                cfg.waterRotation = clampConfigValue(
                    water["rotation_speed"].as<float>(),
                    WaterRange::kWaterRotationMin,
                    WaterRange::kWaterRotationMax, "water.rotation_speed");
            }
            if (water["scroll_speed"]) {
                cfg.waterScrollSpeed = clampConfigValue(
                    water["scroll_speed"].as<float>(),
                    WaterRange::kWaterScrollMin,
                    WaterRange::kWaterScrollMax, "water.scroll_speed");
            }
        }

        // audio section — organized into named subsections.
        // Layout: audio.{performance,reflections,occlusion,propagation,
        //               spatialization,ambient,mixer,dsp}
        if (YAML::Node audio = root["audio"]) {
            // -- audio.performance --
            if (YAML::Node perf = audio["performance"]) {
                if (perf["sample_rate"]) {
                    int v = perf["sample_rate"].as<int>();
                    // Snap to one of the supported rates
                    if      (v <= 22050) cfg.audioSampleRate = 22050;
                    else if (v <= 32000) cfg.audioSampleRate = 32000;
                    else if (v <= 44100) cfg.audioSampleRate = 44100;
                    else if (v <= 48000) cfg.audioSampleRate = 48000;
                    else                 cfg.audioSampleRate = 96000;
                }
                if (perf["frame_size"]) {
                    cfg.audioFrameSize = perf["frame_size"].as<int>();
                    if (cfg.audioFrameSize < 256)  cfg.audioFrameSize = 256;
                    if (cfg.audioFrameSize > 4096) cfg.audioFrameSize = 4096;
                }
                if (perf["sound_cache_mb"]) {
                    cfg.audioSoundCacheMB = perf["sound_cache_mb"].as<int>();
                    if (cfg.audioSoundCacheMB < 4)    cfg.audioSoundCacheMB = 4;
                    if (cfg.audioSoundCacheMB > 1024) cfg.audioSoundCacheMB = 1024;
                }
                if (perf["rate_divisor"]) {
                    int div = perf["rate_divisor"].as<int>();
                    cfg.reflectionRateDivisor = (div >= 4) ? 4 : (div >= 2) ? 2 : 1;
                }
                if (perf["max_active_voices"]) {
                    cfg.maxActiveVoices = perf["max_active_voices"].as<int>();
                    if (cfg.maxActiveVoices < 8)   cfg.maxActiveVoices = 8;
                    if (cfg.maxActiveVoices > 256) cfg.maxActiveVoices = 256;
                }
                // Reverb voice caps (renamed from max_reflection_voices /
                // max_realtime_voices in 2026-05 config cleanup).
                if (perf["reverb_voices"]) {
                    cfg.reverbVoices = perf["reverb_voices"].as<int>();
                    if (cfg.reverbVoices < 0)  cfg.reverbVoices = 0;
                    if (cfg.reverbVoices > 64) cfg.reverbVoices = 64;
                }
                if (perf["reverb_voices_realtime"]) {
                    cfg.reverbVoicesRealtime = perf["reverb_voices_realtime"].as<int>();
                    if (cfg.reverbVoicesRealtime < 0)  cfg.reverbVoicesRealtime = 0;
                    if (cfg.reverbVoicesRealtime > 64) cfg.reverbVoicesRealtime = 64;
                }
                if (perf["reflection_throttle"]) {
                    cfg.reflectionThrottle = perf["reflection_throttle"].as<int>();
                    if (cfg.reflectionThrottle < 1)  cfg.reflectionThrottle = 1;
                    if (cfg.reflectionThrottle > 32) cfg.reflectionThrottle = 32;
                }
                if (perf["sim_max_occlusion_samples"]) {
                    cfg.simMaxOcclusionSamples = perf["sim_max_occlusion_samples"].as<int>();
                    if (cfg.simMaxOcclusionSamples < 4)   cfg.simMaxOcclusionSamples = 4;
                    if (cfg.simMaxOcclusionSamples > 256) cfg.simMaxOcclusionSamples = 256;
                }
                // Explicit thread counts for reverb work. Both 0 = auto.
                // Both > 0 = use literal values. Mixed = auto + warn (see
                // AudioService init).
                if (perf["conv_threads"]) {
                    cfg.convThreads = perf["conv_threads"].as<int>();
                    if (cfg.convThreads < 0)  cfg.convThreads = 0;
                    if (cfg.convThreads > 64) cfg.convThreads = 64;
                }
                if (perf["sim_threads"]) {
                    cfg.simThreads = perf["sim_threads"].as<int>();
                    if (cfg.simThreads < 0)  cfg.simThreads = 0;
                    if (cfg.simThreads > 64) cfg.simThreads = 64;
                }
                if (perf["scene_type"]) {
                    cfg.sceneType = perf["scene_type"].as<std::string>();
                    if (cfg.sceneType != "default" && cfg.sceneType != "embree")
                        cfg.sceneType = "default";
                }
                // Deprecated keys — emit one-shot warnings so existing yamls
                // get a friendly notice. AudioService auto-derives the
                // equivalent values now (see NOTES.AUDIO_CONFIG_AUDIT.md).
                static const char* kDeprecated[] = {
                    "convolution_workers", "simulator_threads",
                    "max_reflection_voices", "max_realtime_voices",
                    "sim_max_rays", "direct_max_sources",
                    "reflection_max_sources", "sim_max_sources",
                    "reflection_demote_hysteresis_frames",
                    "reverb_threads", "reverb_threads_conv_share",
                };
                for (const char* key : kDeprecated) {
                    if (perf[key]) {
                        std::fprintf(stderr,
                            "WARN: audio.performance.%s is no longer used "
                            "(replaced by conv_threads/sim_threads/"
                            "reverb_voices/reverb_voices_realtime or "
                            "auto-derived). Safe to remove from "
                            "darknessRender.yaml.\n", key);
                    }
                }
            }

            // -- audio.engine (device-callback / mixer-thread topology, PR D) --
            if (YAML::Node eng = audio["engine"]) {
                if (eng["ring_mixer"]) {
                    cfg.audioRingMixer = eng["ring_mixer"].as<bool>();
                }
                if (eng["ring_margin_ms"]) {
                    cfg.audioRingMarginMs = eng["ring_margin_ms"].as<float>();
                    // <= 0 = auto; clamp the ceiling so a typo can't
                    // request a multi-second ring.
                    if (cfg.audioRingMarginMs > 500.0f)
                        cfg.audioRingMarginMs = 500.0f;
                }
            }

            // -- audio.reflections --
            //
            // Layout (see PLAN.AUDIO_REALTIME_ARCHITECTURE.md):
            //   reflections.enabled                      — master toggle
            //   reflections.ambisonics_order             — REALTIME ambisonic
            //                                              order (top-level
            //                                              key, for symmetry
            //                                              with the existing
            //                                              ambisonicsOrder
            //                                              consumers in
            //                                              AudioService).
            //   reflections.type / hybrid_*              — algorithm select
            //   reflections.realtime.{rays,bounces,duration,diffuse_samples}
            //   reflections.bake.{rays,bounces,duration,diffuse_samples,ambisonics_order}
            //
            // bake.ambisonics_order may be >= the realtime order — the
            // runtime IPLReflectionEffect processes only the first
            // (realtime_order+1)^2 channels of each baked IR (Steam Audio
            // truncates the higher-order channels at apply time). The
            // AudioService validator rejects bake_order < realtime_order
            // only (the runtime cannot synthesise channels that the bake
            // did not generate).
            if (YAML::Node refl = audio["reflections"]) {
                if (refl["enabled"]) cfg.realtimeReflections = refl["enabled"].as<bool>();

                if (refl["ambisonics_order"]) {
                    cfg.ambisonicsOrder = refl["ambisonics_order"].as<int>();
                    if (cfg.ambisonicsOrder < 0) cfg.ambisonicsOrder = 0;
                    if (cfg.ambisonicsOrder > 3) cfg.ambisonicsOrder = 3;
                }

                if (refl["type"]) std::fprintf(stderr, "[FALLBACK] darknessRender.yaml: 'reflections.type' is deprecated and ignored — HYBRID mode is now the only supported reflection algorithm\n");
                if (refl["bake_skip"]) {
                    std::fprintf(stderr,
                        "[FALLBACK] darknessRender.yaml: "
                        "'audio.reflections.bake_skip' is deprecated and "
                        "ignored — reflection bake is no longer skippable; "
                        "every bake runs both pathing and reflection sections. "
                        "Remove the key from your YAML.\n");
                }
                if (refl["hybrid_transition_time"]) {
                    cfg.hybridTransitionTime = refl["hybrid_transition_time"].as<float>();
                    if (cfg.hybridTransitionTime < 0.1f) cfg.hybridTransitionTime = 0.1f;
                    if (cfg.hybridTransitionTime > 8.0f) cfg.hybridTransitionTime = 8.0f;
                }
                if (refl["hybrid_overlap_percent"]) {
                    cfg.hybridOverlapPercent = refl["hybrid_overlap_percent"].as<float>();
                    if (cfg.hybridOverlapPercent < 0.0f) cfg.hybridOverlapPercent = 0.0f;
                    if (cfg.hybridOverlapPercent > 1.0f) cfg.hybridOverlapPercent = 1.0f;
                }

                if (YAML::Node rt = refl["realtime"]) {
                    if (rt["rays"]) {
                        cfg.realtimeNumRays = rt["rays"].as<int>();
                        if (cfg.realtimeNumRays < 128)  cfg.realtimeNumRays = 128;
                        if (cfg.realtimeNumRays > 8192) cfg.realtimeNumRays = 8192;
                    }
                    if (rt["bounces"]) {
                        cfg.realtimeNumBounces = rt["bounces"].as<int>();
                        if (cfg.realtimeNumBounces < 1) cfg.realtimeNumBounces = 1;
                        if (cfg.realtimeNumBounces > 8) cfg.realtimeNumBounces = 8;
                    }
                    if (rt["duration"]) {
                        cfg.realtimeDuration = rt["duration"].as<float>();
                        if (cfg.realtimeDuration < 0.5f) cfg.realtimeDuration = 0.5f;
                        if (cfg.realtimeDuration > 4.0f) cfg.realtimeDuration = 4.0f;
                    }
                    if (rt["diffuse_samples"]) {
                        cfg.realtimeDiffuseSamples = rt["diffuse_samples"].as<int>();
                        if (cfg.realtimeDiffuseSamples < 16)  cfg.realtimeDiffuseSamples = 16;
                        if (cfg.realtimeDiffuseSamples > 256) cfg.realtimeDiffuseSamples = 256;
                    }
                }

                // New bake sub-block.
                if (YAML::Node bk = refl["bake"]) {
                    if (bk["rays"]) {
                        cfg.bakeNumRays = bk["rays"].as<int>();
                        if (cfg.bakeNumRays < 1024)  cfg.bakeNumRays = 1024;
                        if (cfg.bakeNumRays > 65536) cfg.bakeNumRays = 65536;
                    }
                    if (bk["bounces"]) {
                        cfg.bakeNumBounces = bk["bounces"].as<int>();
                        if (cfg.bakeNumBounces < 1)  cfg.bakeNumBounces = 1;
                        if (cfg.bakeNumBounces > 64) cfg.bakeNumBounces = 64;
                    }
                    if (bk["duration"]) {
                        cfg.bakeDuration = bk["duration"].as<float>();
                        if (cfg.bakeDuration < 0.5f) cfg.bakeDuration = 0.5f;
                        if (cfg.bakeDuration > 8.0f) cfg.bakeDuration = 8.0f;
                    }
                    if (bk["diffuse_samples"]) {
                        cfg.bakeDiffuseSamples = bk["diffuse_samples"].as<int>();
                        if (cfg.bakeDiffuseSamples < 32)   cfg.bakeDiffuseSamples = 32;
                        if (cfg.bakeDiffuseSamples > 4096) cfg.bakeDiffuseSamples = 4096;
                    }
                    if (bk["ambisonics_order"]) {
                        cfg.bakeAmbisonicsOrder = bk["ambisonics_order"].as<int>();
                        if (cfg.bakeAmbisonicsOrder < 0) cfg.bakeAmbisonicsOrder = 0;
                        if (cfg.bakeAmbisonicsOrder > 3) cfg.bakeAmbisonicsOrder = 3;
                    }
                }

                if (refl["runtime_ir_clamp_ms"]) std::fprintf(stderr, "[FALLBACK] darknessRender.yaml: 'reflections.runtime_ir_clamp_ms' is deprecated and ignored — the HYBRID convolution head is already bounded by hybrid_transition_time\n");
            }

            // -- audio.probes --
            // Bake-time grid parameters. A halved spacing quadruples probe
            // count on a 2D floor grid and adds proportional bake time, but
            // reduces footstep-reverb amplitude variance between probes.
            if (YAML::Node prb = audio["probes"]) {
                if (prb["spacing"]) {
                    cfg.audioProbeSpacingFt = prb["spacing"].as<float>();
                    if (cfg.audioProbeSpacingFt < 1.0f)  cfg.audioProbeSpacingFt = 1.0f;
                    if (cfg.audioProbeSpacingFt > 20.0f) cfg.audioProbeSpacingFt = 20.0f;
                }
                if (prb["height"]) {
                    cfg.audioProbeHeightFt = prb["height"].as<float>();
                    if (cfg.audioProbeHeightFt < 0.5f)  cfg.audioProbeHeightFt = 0.5f;
                    if (cfg.audioProbeHeightFt > 20.0f) cfg.audioProbeHeightFt = 20.0f;
                }
                if (prb["elevations"]) {
                    cfg.audioProbeElevations.clear();
                    for (const auto &el : prb["elevations"]) {
                        float v = el.as<float>();
                        if (v > 0.0f && v < 200.0f) {
                            cfg.audioProbeElevations.push_back(v);
                        }
                    }
                }
                if (prb["min_wall_clearance_ft"]) {
                    cfg.audioProbeMinWallClearanceFt =
                        prb["min_wall_clearance_ft"].as<float>();
                    // Hard floor of 0 (disables check); upper guard
                    // matches AudioService::setProbeMinWallClearanceFt.
                    if (cfg.audioProbeMinWallClearanceFt < 0.0f)
                        cfg.audioProbeMinWallClearanceFt = 0.0f;
                    if (cfg.audioProbeMinWallClearanceFt > 50.0f)
                        cfg.audioProbeMinWallClearanceFt = 50.0f;
                }
                if (prb["elevation_sparsity_mul"]) {
                    cfg.audioProbeElevationSparsityMul =
                        prb["elevation_sparsity_mul"].as<float>();
                    if (cfg.audioProbeElevationSparsityMul < 1.0f)
                        cfg.audioProbeElevationSparsityMul = 1.0f;
                    if (cfg.audioProbeElevationSparsityMul > 8.0f)
                        cfg.audioProbeElevationSparsityMul = 8.0f;
                }
                if (prb["global_dedup_radius_ft"]) {
                    cfg.audioProbeGlobalDedupRadiusFt =
                        prb["global_dedup_radius_ft"].as<float>();
                    if (cfg.audioProbeGlobalDedupRadiusFt < 0.0f)
                        cfg.audioProbeGlobalDedupRadiusFt = 0.0f;
                    if (cfg.audioProbeGlobalDedupRadiusFt > 10.0f)
                        cfg.audioProbeGlobalDedupRadiusFt = 10.0f;
                }
            }

            // -- audio.pathing_probes --
            if (YAML::Node pp = audio["pathing_probes"]) {
                if (pp["enabled"])
                    cfg.audioPathingProbesEnabled = pp["enabled"].as<bool>();
                if (pp["dedup_radius_ft"]) {
                    cfg.audioPathingDedupRadiusFt =
                        pp["dedup_radius_ft"].as<float>();
                    if (cfg.audioPathingDedupRadiusFt < 0.0f)
                        cfg.audioPathingDedupRadiusFt = 0.0f;
                    if (cfg.audioPathingDedupRadiusFt > 30.0f)
                        cfg.audioPathingDedupRadiusFt = 30.0f;
                }
                if (pp["vis_range_override_ft"]) {
                    cfg.audioPathingVisRangeOverrideFt =
                        pp["vis_range_override_ft"].as<float>();
                    if (cfg.audioPathingVisRangeOverrideFt < 0.0f)
                        cfg.audioPathingVisRangeOverrideFt = 0.0f;
                    if (cfg.audioPathingVisRangeOverrideFt > 400.0f)
                        cfg.audioPathingVisRangeOverrideFt = 400.0f;
                }
                if (pp["density"]) {
                    // Name list mirrors pathingProbeDensityFromName
                    // (ProbeManager.h, the canonical string→enum map;
                    // not included here to keep RenderConfig free of
                    // audio-stack headers). A name accepted here but
                    // unknown there is caught loudly at the setter
                    // boundary in DarknessRender.cpp.
                    const std::string d = pp["density"].as<std::string>();
                    if (d == "baseline" || d == "bends") {
                        cfg.audioPathingDensity = d;
                    } else {
                        // Reject-at-parse, loudly. "high" is reserved for
                        // a future Tier 2 (room-span subdivision) and is
                        // deliberately NOT accepted until that tier
                        // exists — accepting it now would be a silently-
                        // ignored knob (per the single-source-of-truth
                        // rule from the PR-A config audit). Keep the
                        // default rather than aborting the whole config
                        // load, but say exactly what happened.
                        std::fprintf(stderr,
                            "[FALLBACK] audio.pathing_probes.density: "
                            "invalid value '%s' — valid values are "
                            "'baseline' (Tier 0: original room/portal "
                            "graph nodes) and 'bends' (Tier 1, "
                            "default: + flanking pairs at every "
                            "portal). 'high' is reserved for a future "
                            "Tier 2 and not yet implemented. Using "
                            "default '%s'.\n",
                            d.c_str(), cfg.audioPathingDensity.c_str());
                    }
                }
            }

            // -- audio.occlusion --
            if (YAML::Node occ = audio["occlusion"]) {
                if (occ["radius"]) {
                    cfg.occlusionRadius = occ["radius"].as<float>();
                    // Range in engine units (feet). Converted to meters at the
                    // IPL boundary, so 30 ft ≈ 9 m caps a "very large industrial
                    // source" which is about as wide as makes physical sense.
                    if (cfg.occlusionRadius < 0.3f) cfg.occlusionRadius = 0.3f;
                    if (cfg.occlusionRadius > 30.0f) cfg.occlusionRadius = 30.0f;
                }
                if (occ["samples"]) {
                    cfg.occlusionSamples = occ["samples"].as<int>();
                    if (cfg.occlusionSamples < 4)  cfg.occlusionSamples = 4;
                    if (cfg.occlusionSamples > 64) cfg.occlusionSamples = 64;
                }
                if (occ["transmission_scale"]) {
                    cfg.transmissionScale = occ["transmission_scale"].as<float>();
                    if (cfg.transmissionScale < 0.1f)   cfg.transmissionScale = 0.1f;
                    if (cfg.transmissionScale > 100.0f) cfg.transmissionScale = 100.0f;
                }
                if (occ["absorption_scale"]) {
                    cfg.absorptionScale = occ["absorption_scale"].as<float>();
                    if (cfg.absorptionScale < 0.01f) cfg.absorptionScale = 0.01f;
                    if (cfg.absorptionScale > 10.0f) cfg.absorptionScale = 10.0f;
                }
            }

            // -- audio.propagation --
            if (YAML::Node prop = audio["propagation"]) {
                if (prop["portal_routing"]) cfg.portalRouting = prop["portal_routing"].as<bool>();
                if (prop["probe_pathing"])  cfg.probePathing  = prop["probe_pathing"].as<bool>();
                if (prop["max_distance"]) {
                    cfg.propagationMaxDist = prop["max_distance"].as<float>();
                    if (cfg.propagationMaxDist < 10.0f)   cfg.propagationMaxDist = 10.0f;
                    if (cfg.propagationMaxDist > 5000.0f) cfg.propagationMaxDist = 5000.0f;
                }
                if (prop["door_lpf_open_hz"]) {
                    cfg.doorLpfOpenHz = prop["door_lpf_open_hz"].as<float>();
                    if (cfg.doorLpfOpenHz < 1000.0f)  cfg.doorLpfOpenHz = 1000.0f;
                    if (cfg.doorLpfOpenHz > 24000.0f) cfg.doorLpfOpenHz = 24000.0f;
                }
                if (prop["door_lpf_blocked_hz"]) {
                    cfg.doorLpfBlockedHz = prop["door_lpf_blocked_hz"].as<float>();
                    if (cfg.doorLpfBlockedHz < 100.0f)  cfg.doorLpfBlockedHz = 100.0f;
                    if (cfg.doorLpfBlockedHz > 10000.0f) cfg.doorLpfBlockedHz = 10000.0f;
                }
                if (prop["min_attenuation"]) {
                    cfg.propMinAttenuation = prop["min_attenuation"].as<float>();
                    if (cfg.propMinAttenuation < 0.0f)   cfg.propMinAttenuation = 0.0f;
                    if (cfg.propMinAttenuation > 0.1f)   cfg.propMinAttenuation = 0.1f;
                }
                if (prop["max_paths"]) {
                    int n = prop["max_paths"].as<int>();
                    if (n < 1) n = 1;
                    if (n > 4) n = 4;
                    cfg.propMaxPaths = static_cast<uint32_t>(n);
                }
                if (prop["max_path_diff"]) {
                    cfg.propMaxPathDiff = prop["max_path_diff"].as<float>();
                    if (cfg.propMaxPathDiff < 0.0f)  cfg.propMaxPathDiff = 0.0f;
                    if (cfg.propMaxPathDiff > 50.0f) cfg.propMaxPathDiff = 50.0f;
                }
                if (prop["pathing_gain_scale"]) {
                    cfg.pathingGainScale = prop["pathing_gain_scale"].as<float>();
                    if (cfg.pathingGainScale < 0.1f)  cfg.pathingGainScale = 0.1f;
                    if (cfg.pathingGainScale > 10.0f) cfg.pathingGainScale = 10.0f;
                }
                if (prop["pathing_blocking_scale"]) {
                    cfg.pathingBlockingScale = prop["pathing_blocking_scale"].as<float>();
                    if (cfg.pathingBlockingScale < 0.0f) cfg.pathingBlockingScale = 0.0f;
                    if (cfg.pathingBlockingScale > 1.0f) cfg.pathingBlockingScale = 1.0f;
                }
                if (prop["pathing_update_interval"]) {
                    cfg.pathingUpdateInterval = prop["pathing_update_interval"].as<float>();
                    if (cfg.pathingUpdateInterval < 0.0f) cfg.pathingUpdateInterval = 0.0f;
                    if (cfg.pathingUpdateInterval > 1.0f) cfg.pathingUpdateInterval = 1.0f;
                }
                if (prop["pathing_router_gate"]) {
                    cfg.pathingRouterGate = prop["pathing_router_gate"].as<bool>();
                }
                if (prop["pathing_noroute_move_mul"]) {
                    cfg.pathingNoRouteMoveMul = prop["pathing_noroute_move_mul"].as<float>();
                    if (cfg.pathingNoRouteMoveMul < 1.0f)  cfg.pathingNoRouteMoveMul = 1.0f;
                    if (cfg.pathingNoRouteMoveMul > 16.0f) cfg.pathingNoRouteMoveMul = 16.0f;
                }
                if (prop["pathing_smoothing_ms"]) {
                    cfg.pathingSmoothingMs = prop["pathing_smoothing_ms"].as<float>();
                    if (cfg.pathingSmoothingMs < 0.0f) cfg.pathingSmoothingMs = 0.0f;
                    if (cfg.pathingSmoothingMs > 1000.0f) cfg.pathingSmoothingMs = 1000.0f;
                }
                if (prop["pathing_gain_band_weights"]) {
                    YAML::Node w = prop["pathing_gain_band_weights"];
                    if (w.IsSequence() && w.size() == 3) {
                        auto clamp01 = [](float v) {
                            if (v < 0.0f) return 0.0f;
                            return v > 1.0f ? 1.0f : v;
                        };
                        cfg.pathingGainWeightLow  = clamp01(w[0].as<float>());
                        cfg.pathingGainWeightMid  = clamp01(w[1].as<float>());
                        cfg.pathingGainWeightHigh = clamp01(w[2].as<float>());
                    } else {
                        std::fprintf(stderr, "[CONFIG] pathing_gain_band_weights must be a "
                                     "3-element sequence [low, mid, high]; ignoring.\n");
                    }
                }
            }

            // -- audio.spatialization --
            if (YAML::Node spat = audio["spatialization"]) {
                if (spat["hrtf_volume"]) {
                    cfg.hrtfVolume = spat["hrtf_volume"].as<float>();
                    if (cfg.hrtfVolume < 0.0f) cfg.hrtfVolume = 0.0f;
                    if (cfg.hrtfVolume > 4.0f) cfg.hrtfVolume = 4.0f;
                }
                if (spat["hrtf_interpolation"]) {
                    cfg.hrtfInterpolation = spat["hrtf_interpolation"].as<std::string>();
                    if (cfg.hrtfInterpolation != "nearest" && cfg.hrtfInterpolation != "bilinear")
                        cfg.hrtfInterpolation = "bilinear";
                }
                if (spat["spatial_blend"]) {
                    cfg.spatialBlend = spat["spatial_blend"].as<float>();
                    if (cfg.spatialBlend < 0.0f) cfg.spatialBlend = 0.0f;
                    if (cfg.spatialBlend > 1.0f) cfg.spatialBlend = 1.0f;
                }
                if (spat["distance_model"]) {
                    std::fprintf(stderr,
                        "[CONFIG_DEPRECATED] audio.spatialization.distance_model "
                        "is no longer supported. The distance model is now always "
                        "INVERSEDISTANCE with per-voice minDistance derived from "
                        "the schema's P$SchAttFac. Remove the key from your YAML.\n");
                }
            }

            // -- audio.ambient --
            if (YAML::Node amb = audio["ambient"]) {
                // Group D (2026-05): the radius-multiplier gate was replaced
                // by a dB-based audibility halt. Old keys are still parsed
                // but ignored; emit a one-shot WARN so existing yamls get a
                // pointer to the replacement knobs.
                static const char* kDeprecatedAmbient[] = {
                    "hysteresis_start_mul",
                    "hysteresis_stop_mul",
                };
                for (const char* key : kDeprecatedAmbient) {
                    if (amb[key]) {
                        std::fprintf(stderr,
                            "WARN: audio.ambient.%s is no longer used "
                            "(replaced by halt_audibility_threshold_db / "
                            "halt_below_threshold_frames + spawn_fade_in_ms"
                            " / halt_fade_out_ms — see darknessRender."
                            "example.yaml). Safe to remove from "
                            "darknessRender.yaml.\n", key);
                    }
                }
                if (amb["falloff_curve"]) {
                    std::fprintf(stderr,
                        "[CONFIG_DEPRECATED] audio.ambient.falloff_curve is no "
                        "longer supported. The Dark Engine centibel falloff curve "
                        "was removed when Steam Audio became the sole player-audio "
                        "propagation authority — Steam Audio's distance model "
                        "handles all attenuation now. Tune audio.ambient."
                        "global_volume_scale to compensate for the loudness "
                        "re-baseline. Remove the key from your YAML.\n");
                }
                if (amb["spawn_fade_in_ms"]) {
                    cfg.ambientSpawnFadeInMs = amb["spawn_fade_in_ms"].as<int>();
                    if (cfg.ambientSpawnFadeInMs < 0)    cfg.ambientSpawnFadeInMs = 0;
                    if (cfg.ambientSpawnFadeInMs > 2000) cfg.ambientSpawnFadeInMs = 2000;
                }
                if (amb["halt_fade_out_ms"]) {
                    cfg.ambientHaltFadeOutMs = amb["halt_fade_out_ms"].as<int>();
                    if (cfg.ambientHaltFadeOutMs < 0)    cfg.ambientHaltFadeOutMs = 0;
                    if (cfg.ambientHaltFadeOutMs > 2000) cfg.ambientHaltFadeOutMs = 2000;
                }
                if (amb["halt_audibility_threshold_db"]) {
                    cfg.ambientHaltAudibilityThresholdDb =
                        amb["halt_audibility_threshold_db"].as<float>();
                    if (cfg.ambientHaltAudibilityThresholdDb < -80.0f)
                        cfg.ambientHaltAudibilityThresholdDb = -80.0f;
                    if (cfg.ambientHaltAudibilityThresholdDb > -20.0f)
                        cfg.ambientHaltAudibilityThresholdDb = -20.0f;
                }
                if (amb["halt_below_threshold_frames"]) {
                    cfg.ambientHaltBelowThresholdFrames =
                        amb["halt_below_threshold_frames"].as<int>();
                    if (cfg.ambientHaltBelowThresholdFrames < 5)
                        cfg.ambientHaltBelowThresholdFrames = 5;
                    if (cfg.ambientHaltBelowThresholdFrames > 600)
                        cfg.ambientHaltBelowThresholdFrames = 600;
                }
                if (amb["default_priority"]) {
                    cfg.ambDefaultPriority = amb["default_priority"].as<int>();
                    if (cfg.ambDefaultPriority < 0)   cfg.ambDefaultPriority = 0;
                    if (cfg.ambDefaultPriority > 255) cfg.ambDefaultPriority = 255;
                }
                if (amb["environmental_spatial_blend"]) {
                    cfg.ambEnvironmentalSpatialBlend = amb["environmental_spatial_blend"].as<float>();
                    if (cfg.ambEnvironmentalSpatialBlend < 0.0f) cfg.ambEnvironmentalSpatialBlend = 0.0f;
                    if (cfg.ambEnvironmentalSpatialBlend > 1.0f) cfg.ambEnvironmentalSpatialBlend = 1.0f;
                }
                if (amb["global_volume_scale"]) {
                    cfg.ambGlobalVolumeScale = amb["global_volume_scale"].as<float>();
                    if (cfg.ambGlobalVolumeScale < 0.0f) cfg.ambGlobalVolumeScale = 0.0f;
                    if (cfg.ambGlobalVolumeScale > 4.0f) cfg.ambGlobalVolumeScale = 4.0f;
                }
            }

            // -- audio.mixer --
            if (YAML::Node mix = audio["mixer"]) {
                if (mix["master_gain"]) {
                    cfg.mixerMasterGain = mix["master_gain"].as<float>();
                    if (cfg.mixerMasterGain < 0.0f) cfg.mixerMasterGain = 0.0f;
                    if (cfg.mixerMasterGain > 4.0f) cfg.mixerMasterGain = 4.0f;
                }
                if (mix["direct_gain"]) {
                    cfg.mixerDirectGain = mix["direct_gain"].as<float>();
                    if (cfg.mixerDirectGain < 0.0f) cfg.mixerDirectGain = 0.0f;
                    if (cfg.mixerDirectGain > 4.0f) cfg.mixerDirectGain = 4.0f;
                }
                if (mix["reflection_gain"]) {
                    cfg.mixerReflectionGain = mix["reflection_gain"].as<float>();
                    if (cfg.mixerReflectionGain < 0.0f) cfg.mixerReflectionGain = 0.0f;
                    if (cfg.mixerReflectionGain > 4.0f) cfg.mixerReflectionGain = 4.0f;
                }
                if (mix["reflection_ramp_ms"]) {
                    cfg.reflectionRampMs = mix["reflection_ramp_ms"].as<float>();
                    if (cfg.reflectionRampMs < 1.0f)   cfg.reflectionRampMs = 1.0f;
                    if (cfg.reflectionRampMs > 1000.0f) cfg.reflectionRampMs = 1000.0f;
                }
            }

            // -- audio.dsp --
            if (YAML::Node dsp = audio["dsp"]) {
                if (dsp["limiter_enabled"]) cfg.dspLimiter = dsp["limiter_enabled"].as<bool>();
                if (dsp["limiter_knee"]) {
                    cfg.dspLimiterKnee = dsp["limiter_knee"].as<float>();
                    if (cfg.dspLimiterKnee < 0.5f)  cfg.dspLimiterKnee = 0.5f;
                    if (cfg.dspLimiterKnee > 0.95f) cfg.dspLimiterKnee = 0.95f;
                }
                if (dsp["compressor_enabled"]) cfg.dspCompressor = dsp["compressor_enabled"].as<bool>();
                if (dsp["compressor_threshold_db"]) {
                    cfg.dspCompThreshold = dsp["compressor_threshold_db"].as<float>();
                    if (cfg.dspCompThreshold < -30.0f) cfg.dspCompThreshold = -30.0f;
                    if (cfg.dspCompThreshold > 0.0f)   cfg.dspCompThreshold = 0.0f;
                }
                if (dsp["compressor_ratio"]) {
                    cfg.dspCompRatio = dsp["compressor_ratio"].as<float>();
                    if (cfg.dspCompRatio < 1.5f) cfg.dspCompRatio = 1.5f;
                    if (cfg.dspCompRatio > 10.0f) cfg.dspCompRatio = 10.0f;
                }
                if (dsp["compressor_attack_ms"]) {
                    cfg.dspCompAttackMs = dsp["compressor_attack_ms"].as<float>();
                    if (cfg.dspCompAttackMs < 1.0f)   cfg.dspCompAttackMs = 1.0f;
                    if (cfg.dspCompAttackMs > 100.0f) cfg.dspCompAttackMs = 100.0f;
                }
                if (dsp["compressor_release_ms"]) {
                    cfg.dspCompReleaseMs = dsp["compressor_release_ms"].as<float>();
                    if (cfg.dspCompReleaseMs < 50.0f)   cfg.dspCompReleaseMs = 50.0f;
                    if (cfg.dspCompReleaseMs > 2000.0f) cfg.dspCompReleaseMs = 2000.0f;
                }
                if (dsp["eq_enabled"]) cfg.dspEQ = dsp["eq_enabled"].as<bool>();
                if (dsp["eq_freq_hz"]) {
                    cfg.dspEQFreq = dsp["eq_freq_hz"].as<float>();
                    if (cfg.dspEQFreq < 60.0f)  cfg.dspEQFreq = 60.0f;
                    if (cfg.dspEQFreq > 500.0f) cfg.dspEQFreq = 500.0f;
                }
                if (dsp["eq_gain_db"]) {
                    cfg.dspEQGain = dsp["eq_gain_db"].as<float>();
                    if (cfg.dspEQGain < -6.0f) cfg.dspEQGain = -6.0f;
                    if (cfg.dspEQGain > 6.0f)  cfg.dspEQGain = 6.0f;
                }
                if (dsp["eq_q"]) {
                    cfg.dspEQQ = dsp["eq_q"].as<float>();
                    if (cfg.dspEQQ < 0.3f) cfg.dspEQQ = 0.3f;
                    if (cfg.dspEQQ > 2.0f) cfg.dspEQQ = 2.0f;
                }
                if (dsp["ducking_enabled"]) cfg.dspDucking = dsp["ducking_enabled"].as<bool>();
                if (dsp["ducking_amount"]) {
                    cfg.dspDuckAmount = dsp["ducking_amount"].as<float>();
                    if (cfg.dspDuckAmount < 0.1f) cfg.dspDuckAmount = 0.1f;
                    if (cfg.dspDuckAmount > 1.0f) cfg.dspDuckAmount = 1.0f;
                }
                if (dsp["ducking_attack_ms"]) {
                    cfg.dspDuckAttackMs = dsp["ducking_attack_ms"].as<float>();
                    if (cfg.dspDuckAttackMs < 10.0f)  cfg.dspDuckAttackMs = 10.0f;
                    if (cfg.dspDuckAttackMs > 500.0f) cfg.dspDuckAttackMs = 500.0f;
                }
                if (dsp["ducking_release_ms"]) {
                    cfg.dspDuckReleaseMs = dsp["ducking_release_ms"].as<float>();
                    if (cfg.dspDuckReleaseMs < 50.0f)   cfg.dspDuckReleaseMs = 50.0f;
                    if (cfg.dspDuckReleaseMs > 5000.0f) cfg.dspDuckReleaseMs = 5000.0f;
                }
                if (dsp["wet_saturation_enabled"]) cfg.dspWetSaturation = dsp["wet_saturation_enabled"].as<bool>();
                if (dsp["wet_saturation_drive"]) {
                    cfg.dspWetSaturationDrive = dsp["wet_saturation_drive"].as<float>();
                    if (cfg.dspWetSaturationDrive < 1.0f)  cfg.dspWetSaturationDrive = 1.0f;
                    if (cfg.dspWetSaturationDrive > 10.0f) cfg.dspWetSaturationDrive = 10.0f;
                }
            }
        }

        // physics section
        if (YAML::Node phys = root["physics"]) {
            if (phys["rate"]) {
                // Accept string names or integer Hz values
                try {
                    std::string val = phys["rate"].as<std::string>();
                    if (val == "vintage" || val == "12.5" || val == "12")
                        cfg.physicsRate = 12;
                    else if (val == "ultra" || val == "120")
                        cfg.physicsRate = 120;
                    else
                        cfg.physicsRate = 60;  // "modern" or unknown → default
                } catch (...) {
                    int val = phys["rate"].as<int>(60);
                    if (val <= 12) cfg.physicsRate = 12;
                    else if (val >= 120) cfg.physicsRate = 120;
                    else cfg.physicsRate = 60;
                }
            }
        }

        // developer section
        if (YAML::Node dev = root["developer"]) {
            if (dev["show_objects"])        cfg.showObjects       = dev["show_objects"].as<bool>();
            if (dev["show_fallback_cubes"]) cfg.showFallbackCubes = dev["show_fallback_cubes"].as<bool>();
            if (dev["portal_culling"])      cfg.portalCulling     = dev["portal_culling"].as<bool>();
            if (dev["camera_collision"])    cfg.cameraCollision   = dev["camera_collision"].as<bool>();
            if (dev["step_log"])            cfg.stepLog           = dev["step_log"].as<bool>();
            if (dev["log_mute"])            cfg.logMute           = dev["log_mute"].as<std::string>();
            if (dev["debug_objects"])       cfg.debugObjects      = dev["debug_objects"].as<bool>();
            if (dev["toggle_platforms"])    cfg.togglePlatforms   = dev["toggle_platforms"].as<bool>();
            if (dev["no_probes"])           cfg.noProbes          = dev["no_probes"].as<bool>();
            if (dev["audio_log"])           cfg.audioLog          = dev["audio_log"].as<bool>();
            if (dev["capture_wav"])         cfg.captureWav        = dev["capture_wav"].as<bool>();
        }

        std::fprintf(stderr, "Loaded config from %s\n", path.c_str());
        return true;
    } catch (const YAML::Exception& e) {
        // The file existed and we tried to parse it — a syntax error here
        // means everything past the bad line was silently skipped, leaving
        // the program running with a half-applied config. The library only
        // returns false (so unit tests can exercise this path); the BINARY
        // is expected to detect "file existed but parse failed" and abort.
        // See DarknessRender.cpp's call site for the abort.
        std::fprintf(stderr,
            "\nERROR: failed to parse %s\n"
            "  %s\n"
            "Fix the syntax error or remove the file to run with defaults.\n",
            path.c_str(), e.what());
        return false;
    }
}

// ── --set dispatch table (PLAN.AUDIO_PROFILING.md §1.4) ──
//
// Generic CLI override: `--set audio.dotted.path=value`. Applied AFTER YAML
// load (so it wins over file values) and BEFORE the AudioService init pass
// reads `cfg`. The supported leaves cover every audio-tunable knob enumerated
// in PLAN.AUDIO_PROFILING.md §3 (the per-tunable metric map). Each entry
// here maps yaml-key -> RenderConfig field; clamping is performed inline,
// matching the YAML loader's clamps so a --set override and a YAML value
// behave identically.
//
// Leaf types supported:
//   - float  (numeric leaves; std::stof)
//   - int    (integer leaves; std::stoi)
//   - bool   (true/false/1/0/yes/no)
//   - string (verbatim, with light validation for enum-like keys)
//
// Per feedback_no_silent_fallbacks: unknown paths emit a [FALLBACK] stderr
// line listing the path that was rejected so a typo is impossible to miss.
//
// Implementation note: a hardcoded if/else dispatch is fine — the set is
// small, additive over the project lifetime, and avoids pulling in a
// YAML/JSON-path mini-parser. When a new YAML knob lands, also add it
// here (and to the help text further down).
inline bool applySetOverride(const std::string& path, const std::string& valueStr,
                             RenderConfig& cfg) {
    auto toBool = [&](bool& out) -> bool {
        if (valueStr == "true"  || valueStr == "1" || valueStr == "yes") { out = true;  return true; }
        if (valueStr == "false" || valueStr == "0" || valueStr == "no")  { out = false; return true; }
        return false;
    };
    auto toFloat = [&](float& out) -> bool {
        try { out = std::stof(valueStr); return true; } catch (...) { return false; }
    };
    auto toInt = [&](int& out) -> bool {
        try { out = std::stoi(valueStr); return true; } catch (...) { return false; }
    };

    // Helpers wrap field assignment + clamp into one line each. Each lambda
    // returns the new clamped value for traceability; the discard is fine.
    auto clampF = [](float v, float lo, float hi) {
        return (v < lo) ? lo : (v > hi ? hi : v);
    };
    auto clampI = [](int v, int lo, int hi) {
        return (v < lo) ? lo : (v > hi ? hi : v);
    };

    // -- audio.performance --
    if (path == "audio.performance.sample_rate") {
        int v; if (!toInt(v)) return false;
        if      (v <= 22050) cfg.audioSampleRate = 22050;
        else if (v <= 32000) cfg.audioSampleRate = 32000;
        else if (v <= 44100) cfg.audioSampleRate = 44100;
        else if (v <= 48000) cfg.audioSampleRate = 48000;
        else                 cfg.audioSampleRate = 96000;
        return true;
    }
    if (path == "audio.performance.frame_size") {
        int v; if (!toInt(v)) return false;
        cfg.audioFrameSize = clampI(v, 256, 4096); return true;
    }
    if (path == "audio.performance.sound_cache_mb") {
        int v; if (!toInt(v)) return false;
        cfg.audioSoundCacheMB = clampI(v, 4, 1024); return true;
    }
    if (path == "audio.performance.rate_divisor") {
        int v; if (!toInt(v)) return false;
        cfg.reflectionRateDivisor = (v >= 4) ? 4 : (v >= 2) ? 2 : 1; return true;
    }
    if (path == "audio.performance.max_active_voices") {
        int v; if (!toInt(v)) return false;
        cfg.maxActiveVoices = clampI(v, 8, 256); return true;
    }
    if (path == "audio.performance.reverb_voices") {
        int v; if (!toInt(v)) return false;
        cfg.reverbVoices = clampI(v, 0, 64); return true;
    }
    if (path == "audio.performance.reverb_voices_realtime") {
        int v; if (!toInt(v)) return false;
        cfg.reverbVoicesRealtime = clampI(v, 0, 64); return true;
    }
    if (path == "audio.performance.reflection_throttle") {
        int v; if (!toInt(v)) return false;
        cfg.reflectionThrottle = clampI(v, 1, 32); return true;
    }
    if (path == "audio.performance.sim_max_occlusion_samples") {
        int v; if (!toInt(v)) return false;
        cfg.simMaxOcclusionSamples = clampI(v, 4, 256); return true;
    }
    if (path == "audio.performance.conv_threads") {
        int v; if (!toInt(v)) return false;
        cfg.convThreads = clampI(v, 0, 64); return true;
    }
    if (path == "audio.performance.sim_threads") {
        int v; if (!toInt(v)) return false;
        cfg.simThreads = clampI(v, 0, 64); return true;
    }
    if (path == "audio.performance.scene_type") {
        cfg.sceneType = (valueStr == "embree" ? "embree" : "default");
        return true;
    }

    // -- audio.engine (PR D ring mixer) --
    if (path == "audio.engine.ring_mixer") {
        return toBool(cfg.audioRingMixer);
    }
    if (path == "audio.engine.ring_margin_ms") {
        float v; if (!toFloat(v)) return false;
        // <= 0 = auto (two engine blocks, min 21.4 ms); cap mirrors the
        // YAML loader's anti-typo ceiling.
        cfg.audioRingMarginMs = (v > 500.0f) ? 500.0f : v;
        return true;
    }

    // -- audio.reflections --
    if (path == "audio.reflections.enabled") {
        return toBool(cfg.realtimeReflections);
    }
    if (path == "audio.reflections.ambisonics_order") {
        int v; if (!toInt(v)) return false;
        cfg.ambisonicsOrder = clampI(v, 0, 3); return true;
    }
    if (path == "audio.reflections.bake_skip") {
        std::fprintf(stderr,
            "[FALLBACK] applySetOverride: 'audio.reflections.bake_skip' "
            "is deprecated and ignored — reflection bake is no longer "
            "skippable.\n");
        return true;
    }
    if (path == "audio.reflections.hybrid_transition_time") {
        float v; if (!toFloat(v)) return false;
        cfg.hybridTransitionTime = clampF(v, 0.1f, 8.0f); return true;
    }
    if (path == "audio.reflections.hybrid_overlap_percent") {
        float v; if (!toFloat(v)) return false;
        cfg.hybridOverlapPercent = clampF(v, 0.0f, 1.0f); return true;
    }
    if (path == "audio.reflections.realtime.rays") {
        int v; if (!toInt(v)) return false;
        cfg.realtimeNumRays = clampI(v, 128, 8192); return true;
    }
    if (path == "audio.reflections.realtime.bounces") {
        int v; if (!toInt(v)) return false;
        cfg.realtimeNumBounces = clampI(v, 1, 8); return true;
    }
    if (path == "audio.reflections.realtime.duration") {
        float v; if (!toFloat(v)) return false;
        cfg.realtimeDuration = clampF(v, 0.5f, 4.0f); return true;
    }
    if (path == "audio.reflections.realtime.diffuse_samples") {
        int v; if (!toInt(v)) return false;
        cfg.realtimeDiffuseSamples = clampI(v, 16, 256); return true;
    }
    if (path == "audio.reflections.bake.rays") {
        int v; if (!toInt(v)) return false;
        cfg.bakeNumRays = clampI(v, 1024, 65536); return true;
    }
    if (path == "audio.reflections.bake.bounces") {
        int v; if (!toInt(v)) return false;
        cfg.bakeNumBounces = clampI(v, 1, 64); return true;
    }
    if (path == "audio.reflections.bake.duration") {
        float v; if (!toFloat(v)) return false;
        cfg.bakeDuration = clampF(v, 0.5f, 8.0f); return true;
    }
    if (path == "audio.reflections.bake.diffuse_samples") {
        int v; if (!toInt(v)) return false;
        cfg.bakeDiffuseSamples = clampI(v, 32, 4096); return true;
    }
    if (path == "audio.reflections.bake.ambisonics_order") {
        int v; if (!toInt(v)) return false;
        cfg.bakeAmbisonicsOrder = clampI(v, 0, 3); return true;
    }

    // -- audio.probes --
    if (path == "audio.probes.spacing") {
        float v; if (!toFloat(v)) return false;
        cfg.audioProbeSpacingFt = clampF(v, 1.0f, 20.0f); return true;
    }
    if (path == "audio.probes.height") {
        float v; if (!toFloat(v)) return false;
        cfg.audioProbeHeightFt = clampF(v, 0.5f, 20.0f); return true;
    }
    if (path == "audio.probes.min_wall_clearance_ft") {
        float v; if (!toFloat(v)) return false;
        cfg.audioProbeMinWallClearanceFt = clampF(v, 0.0f, 50.0f); return true;
    }
    if (path == "audio.probes.elevation_sparsity_mul") {
        float v; if (!toFloat(v)) return false;
        cfg.audioProbeElevationSparsityMul = clampF(v, 1.0f, 8.0f); return true;
    }
    if (path == "audio.probes.global_dedup_radius_ft") {
        float v; if (!toFloat(v)) return false;
        cfg.audioProbeGlobalDedupRadiusFt = clampF(v, 0.0f, 10.0f); return true;
    }

    // -- audio.pathing_probes --
    if (path == "audio.pathing_probes.enabled") {
        return toBool(cfg.audioPathingProbesEnabled);
    }
    if (path == "audio.pathing_probes.dedup_radius_ft") {
        float v; if (!toFloat(v)) return false;
        cfg.audioPathingDedupRadiusFt = clampF(v, 0.0f, 30.0f); return true;
    }
    if (path == "audio.pathing_probes.vis_range_override_ft") {
        float v; if (!toFloat(v)) return false;
        cfg.audioPathingVisRangeOverrideFt = clampF(v, 0.0f, 400.0f);
        return true;
    }
    if (path == "audio.pathing_probes.density") {
        // Same validation as the YAML parser: baseline | bends only
        // ("high" reserved for a future Tier 2); name list mirrors
        // pathingProbeDensityFromName (ProbeManager.h) — see the YAML
        // parser comment. Returning false routes through the caller's
        // loud invalid-value report.
        if (valueStr == "baseline" || valueStr == "bends") {
            cfg.audioPathingDensity = valueStr;
            return true;
        }
        std::fprintf(stderr,
            "[FALLBACK] audio.pathing_probes.density: invalid value "
            "'%s' — valid: 'baseline' | 'bends' ('high' reserved, not "
            "yet implemented)\n", valueStr.c_str());
        return false;
    }

    // -- audio.occlusion --
    if (path == "audio.occlusion.radius") {
        float v; if (!toFloat(v)) return false;
        cfg.occlusionRadius = clampF(v, 0.3f, 30.0f); return true;
    }
    if (path == "audio.occlusion.samples") {
        int v; if (!toInt(v)) return false;
        cfg.occlusionSamples = clampI(v, 4, 64); return true;
    }
    if (path == "audio.occlusion.transmission_scale") {
        float v; if (!toFloat(v)) return false;
        cfg.transmissionScale = clampF(v, 0.1f, 100.0f); return true;
    }
    if (path == "audio.occlusion.absorption_scale") {
        float v; if (!toFloat(v)) return false;
        cfg.absorptionScale = clampF(v, 0.01f, 10.0f); return true;
    }

    // -- audio.propagation --
    if (path == "audio.propagation.portal_routing") {
        return toBool(cfg.portalRouting);
    }
    if (path == "audio.propagation.probe_pathing") {
        return toBool(cfg.probePathing);
    }
    if (path == "audio.propagation.max_distance") {
        float v; if (!toFloat(v)) return false;
        cfg.propagationMaxDist = clampF(v, 10.0f, 5000.0f); return true;
    }
    if (path == "audio.propagation.door_lpf_open_hz") {
        float v; if (!toFloat(v)) return false;
        cfg.doorLpfOpenHz = clampF(v, 1000.0f, 24000.0f); return true;
    }
    if (path == "audio.propagation.door_lpf_blocked_hz") {
        float v; if (!toFloat(v)) return false;
        cfg.doorLpfBlockedHz = clampF(v, 100.0f, 10000.0f); return true;
    }
    if (path == "audio.propagation.min_attenuation") {
        float v; if (!toFloat(v)) return false;
        cfg.propMinAttenuation = clampF(v, 0.0f, 0.1f); return true;
    }
    if (path == "audio.propagation.max_paths") {
        int v; if (!toInt(v)) return false;
        cfg.propMaxPaths = static_cast<uint32_t>(clampI(v, 1, 4)); return true;
    }
    if (path == "audio.propagation.max_path_diff") {
        float v; if (!toFloat(v)) return false;
        cfg.propMaxPathDiff = clampF(v, 0.0f, 50.0f); return true;
    }
    if (path == "audio.propagation.pathing_gain_scale") {
        float v; if (!toFloat(v)) return false;
        cfg.pathingGainScale = clampF(v, 0.1f, 10.0f); return true;
    }
    if (path == "audio.propagation.pathing_blocking_scale") {
        float v; if (!toFloat(v)) return false;
        cfg.pathingBlockingScale = clampF(v, 0.0f, 1.0f); return true;
    }
    if (path == "audio.propagation.pathing_update_interval") {
        float v; if (!toFloat(v)) return false;
        cfg.pathingUpdateInterval = clampF(v, 0.0f, 1.0f); return true;
    }
    if (path == "audio.propagation.pathing_router_gate") {
        bool v; if (!toBool(v)) return false;
        cfg.pathingRouterGate = v; return true;
    }
    if (path == "audio.propagation.pathing_noroute_move_mul") {
        float v; if (!toFloat(v)) return false;
        cfg.pathingNoRouteMoveMul = clampF(v, 1.0f, 16.0f); return true;
    }
    if (path == "audio.propagation.pathing_smoothing_ms") {
        float v; if (!toFloat(v)) return false;
        cfg.pathingSmoothingMs = clampF(v, 0.0f, 1000.0f); return true;
    }

    // -- audio.spatialization --
    if (path == "audio.spatialization.hrtf_volume") {
        float v; if (!toFloat(v)) return false;
        cfg.hrtfVolume = clampF(v, 0.0f, 4.0f); return true;
    }
    if (path == "audio.spatialization.hrtf_interpolation") {
        cfg.hrtfInterpolation = (valueStr == "nearest") ? "nearest" : "bilinear";
        return true;
    }
    if (path == "audio.spatialization.spatial_blend") {
        float v; if (!toFloat(v)) return false;
        cfg.spatialBlend = clampF(v, 0.0f, 1.0f); return true;
    }

    // -- audio.ambient --
    if (path == "audio.ambient.spawn_fade_in_ms") {
        int v; if (!toInt(v)) return false;
        cfg.ambientSpawnFadeInMs = clampI(v, 0, 2000); return true;
    }
    if (path == "audio.ambient.halt_fade_out_ms") {
        int v; if (!toInt(v)) return false;
        cfg.ambientHaltFadeOutMs = clampI(v, 0, 2000); return true;
    }
    if (path == "audio.ambient.halt_audibility_threshold_db") {
        float v; if (!toFloat(v)) return false;
        cfg.ambientHaltAudibilityThresholdDb = clampF(v, -80.0f, -20.0f); return true;
    }
    if (path == "audio.ambient.halt_below_threshold_frames") {
        int v; if (!toInt(v)) return false;
        cfg.ambientHaltBelowThresholdFrames = clampI(v, 5, 600); return true;
    }
    if (path == "audio.ambient.default_priority") {
        int v; if (!toInt(v)) return false;
        cfg.ambDefaultPriority = clampI(v, 0, 255); return true;
    }
    if (path == "audio.ambient.environmental_spatial_blend") {
        float v; if (!toFloat(v)) return false;
        cfg.ambEnvironmentalSpatialBlend = clampF(v, 0.0f, 1.0f); return true;
    }
    if (path == "audio.ambient.global_volume_scale") {
        float v; if (!toFloat(v)) return false;
        cfg.ambGlobalVolumeScale = clampF(v, 0.0f, 4.0f); return true;
    }

    // -- audio.mixer --
    if (path == "audio.mixer.master_gain") {
        float v; if (!toFloat(v)) return false;
        cfg.mixerMasterGain = clampF(v, 0.0f, 4.0f); return true;
    }
    if (path == "audio.mixer.direct_gain") {
        float v; if (!toFloat(v)) return false;
        cfg.mixerDirectGain = clampF(v, 0.0f, 4.0f); return true;
    }
    if (path == "audio.mixer.reflection_gain") {
        float v; if (!toFloat(v)) return false;
        cfg.mixerReflectionGain = clampF(v, 0.0f, 4.0f); return true;
    }
    if (path == "audio.mixer.reflection_ramp_ms") {
        float v; if (!toFloat(v)) return false;
        cfg.reflectionRampMs = clampF(v, 1.0f, 1000.0f); return true;
    }

    // -- audio.dsp --
    if (path == "audio.dsp.limiter_enabled")   { return toBool(cfg.dspLimiter); }
    if (path == "audio.dsp.limiter_knee") {
        float v; if (!toFloat(v)) return false;
        cfg.dspLimiterKnee = clampF(v, 0.5f, 0.95f); return true;
    }
    if (path == "audio.dsp.compressor_enabled") { return toBool(cfg.dspCompressor); }
    if (path == "audio.dsp.compressor_threshold_db") {
        float v; if (!toFloat(v)) return false;
        cfg.dspCompThreshold = clampF(v, -30.0f, 0.0f); return true;
    }
    if (path == "audio.dsp.compressor_ratio") {
        float v; if (!toFloat(v)) return false;
        cfg.dspCompRatio = clampF(v, 1.5f, 10.0f); return true;
    }
    if (path == "audio.dsp.compressor_attack_ms") {
        float v; if (!toFloat(v)) return false;
        cfg.dspCompAttackMs = clampF(v, 1.0f, 100.0f); return true;
    }
    if (path == "audio.dsp.compressor_release_ms") {
        float v; if (!toFloat(v)) return false;
        cfg.dspCompReleaseMs = clampF(v, 50.0f, 2000.0f); return true;
    }
    if (path == "audio.dsp.eq_enabled") { return toBool(cfg.dspEQ); }
    if (path == "audio.dsp.eq_freq_hz") {
        float v; if (!toFloat(v)) return false;
        cfg.dspEQFreq = clampF(v, 60.0f, 500.0f); return true;
    }
    if (path == "audio.dsp.eq_gain_db") {
        float v; if (!toFloat(v)) return false;
        cfg.dspEQGain = clampF(v, -6.0f, 6.0f); return true;
    }
    if (path == "audio.dsp.eq_q") {
        float v; if (!toFloat(v)) return false;
        cfg.dspEQQ = clampF(v, 0.3f, 2.0f); return true;
    }
    if (path == "audio.dsp.ducking_enabled") { return toBool(cfg.dspDucking); }
    if (path == "audio.dsp.ducking_amount") {
        float v; if (!toFloat(v)) return false;
        cfg.dspDuckAmount = clampF(v, 0.1f, 1.0f); return true;
    }
    if (path == "audio.dsp.ducking_attack_ms") {
        float v; if (!toFloat(v)) return false;
        cfg.dspDuckAttackMs = clampF(v, 10.0f, 500.0f); return true;
    }
    if (path == "audio.dsp.ducking_release_ms") {
        float v; if (!toFloat(v)) return false;
        cfg.dspDuckReleaseMs = clampF(v, 50.0f, 5000.0f); return true;
    }
    if (path == "audio.dsp.wet_saturation_enabled") { return toBool(cfg.dspWetSaturation); }
    if (path == "audio.dsp.wet_saturation_drive") {
        float v; if (!toFloat(v)) return false;
        cfg.dspWetSaturationDrive = clampF(v, 1.0f, 10.0f); return true;
    }

    return false; // unknown path
}

// Validate --perf-label string: only [A-Za-z0-9_.-] allowed so the resulting
// directory name is safe across macOS / Linux / Windows + shell-quote-free.
inline bool isPerfLabelValid(const std::string& s) {
    if (s.empty() || s.size() > 64) return false;
    for (char c : s) {
        bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                  (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.';
        if (!ok) return false;
    }
    return true;
}

// Parse CLI arguments. The CLI surface is intentionally minimal — every
// other tunable lives in the YAML config. The flags below are the things
// that genuinely need per-invocation override:
//   <mission.mis>      first non-flag arg, the mission to load
//   --res <path>       runtime asset directory (overrides paths.res)
//   --schemas <path>   schema directory (overrides paths.schemas)
//   --config <path>    YAML path (defaults to ./darknessRender.yaml)
//   --force-pathing-bake    drop existing pathing section + re-bake it
//   --set <p>=<v>      generic YAML-path override; repeatable
//   --perf-label <s>   tag the per-run audio_perf.jsonl directory
//   --exit-after-seconds N  exit cleanly after N seconds of wall-clock
//   --auto-fly         enable deterministic probe-tour flythrough
//   --auto-fly-speed N        ft/s (default 10)
//   --auto-fly-waypoints N    N-nearest probes to visit (default 50)
//   --auto-fly-seed N         shuffle seed (default 0xC0FFEE)
//   --auto-fly-pause-sec N    dwell per waypoint (default 0)
//   --audio-capture x,y,z     pin listener at a point, spin in place, exit
//   --audio-capture-seconds N capture window length (default 15)
//   --audio-capture-rotations N full yaw turns over the window (default 3)
//   --capture-wav      record final engine output to output.wav in the
//                      per-run perf directory (= developer.capture_wav)
//   --stress-doors N   DEV-ONLY: toggle the N nearest doors every ~2 s
//                      (O2a door-route-latency stress harness)
//   --stress-door-ids "a,b"  DEV-ONLY: with --stress-doors armed, toggle
//                      exactly these door object IDs instead of nearest-N
//   --spawn-override "x,y,z[,yaw]"  DEV-ONLY: force the camera/player start
//                      position (positioned diagnostic runs)
//   --help / -h        print usage
//
// Unknown flags are reported but otherwise ignored — when a removed flag
// shows up in old shell history, the user gets a clear message instead
// of a silent parse miss.
inline CliResult applyCliOverrides(int argc, char* argv[], RenderConfig& cfg) {
    CliResult cli;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            cli.helpRequested = true;
        } else if (std::strcmp(argv[i], "--res") == 0 && i + 1 < argc) {
            cli.resPath = argv[++i];
        } else if (std::strcmp(argv[i], "--schemas") == 0 && i + 1 < argc) {
            cli.schemasPath = argv[++i];
        } else if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            cli.configPath = argv[++i];
        } else if (std::strcmp(argv[i], "--rebake-lightmaps") == 0) {
            cfg.rebakeLightmaps = true;
            // Optional density argument: --rebake-lightmaps 4
            if (i + 1 < argc && argv[i + 1][0] != '-')
                cfg.rebakeDensity = std::max(1, std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--rebake-samples") == 0 && i + 1 < argc) {
            cfg.rebakeSamples = std::max(1, std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--rebake-emitter") == 0 && i + 1 < argc) {
            cfg.rebakeEmitter = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(argv[i], "--rebake-supersample") == 0 && i + 1 < argc) {
            cfg.rebakeSupersample = std::max(1, std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--rebake-bounce") == 0 && i + 1 < argc) {
            cfg.rebakeBounce = std::max(0, std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--rebake-ao") == 0 && i + 1 < argc) {
            cfg.rebakeAO = std::min(1.0f, std::max(0.0f,
                static_cast<float>(std::atof(argv[++i]))));
        } else if (std::strcmp(argv[i], "--rebake-reach") == 0 && i + 1 < argc) {
            cfg.rebakeReach = std::max(0, std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--rebake-soft-radius") == 0 && i + 1 < argc) {
            cfg.rebakeSoftRadius = std::min(0.9f, std::max(0.0f,
                static_cast<float>(std::atof(argv[++i]))));
        } else if (std::strcmp(argv[i], "--rebake-falloff-original") == 0) {
            cfg.rebakeFalloffPhysical = false;
        } else if (std::strcmp(argv[i], "--rebake-falloff-anchor") == 0 && i + 1 < argc) {
            cfg.rebakeFalloffAnchor = std::min(32.0f, std::max(1.0f,
                static_cast<float>(std::atof(argv[++i]))));
        } else if (std::strcmp(argv[i], "--rebake-throw-alpha") == 0 && i + 1 < argc) {
            cfg.rebakeThrowAlpha = std::min(1.0f, std::max(0.0f,
                static_cast<float>(std::atof(argv[++i]))));
        } else if (std::strcmp(argv[i], "--rebake-no-door-shadows") == 0) {
            cfg.rebakeDoorShadows = false;
        } else if (std::strcmp(argv[i], "--stress-doors-cycles") == 0 && i + 1 < argc) {
            cfg.stressDoorsCycles = std::max(0, std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--shadow-face-size") == 0 && i + 1 < argc) {
            cfg.shadowFaceSize = std::min(2048, std::max(64,
                std::atoi(argv[++i])));
        } else if (std::strcmp(argv[i], "--door-diff-diag") == 0 &&
                   i + 1 < argc) {
            cfg.doorDiffDiag = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--spawn-at-door") == 0 &&
                   i + 1 < argc) {
            cfg.spawnAtDoor = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--door-swing-diag") == 0 &&
                   i + 1 < argc) {
            cfg.doorSwingDiag = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--door-swing-steps") == 0 &&
                   i + 1 < argc) {
            cfg.doorSwingSteps = std::min(32, std::max(2,
                std::atoi(argv[++i])));
        } else if (std::strcmp(argv[i], "--lumel-bake-test") == 0) {
            cfg.lumelBakeTest = true;
        } else if (std::strcmp(argv[i], "--rebake-cpu-events") == 0) {
            cfg.rebakeCpuEvents = true;
        } else if (std::strcmp(argv[i], "--rebake-gpu-events") == 0) {
            cfg.rebakeCpuEvents = false;
        } else if (std::strcmp(argv[i], "--shadow-crosscheck") == 0) {
            // Optional pair-count argument: --shadow-crosscheck 50000
            cfg.shadowCrossCheckPairs = 20000;
            if (i + 1 < argc && argv[i + 1][0] != '-')
                cfg.shadowCrossCheckPairs = std::max(100, std::atoi(argv[++i]));
        } else if (std::strcmp(argv[i], "--stress-frob-obj") == 0 && i + 1 < argc) {
            cfg.stressFrobObjs = argv[++i];
        } else if (std::strcmp(argv[i], "--skip-reflection-bake") == 0) {
            // Deprecated CLI flag — reflection bake is no longer
            // skippable; every probe-bake-needed path runs the full bake
            // (pathing + reflections).
            std::fprintf(stderr,
                "[FALLBACK] --skip-reflection-bake is deprecated and "
                "ignored — reflection bake is no longer skippable. "
                "Remove the flag from your invocation.\n");
        } else if (std::strcmp(argv[i], "--force-pathing-bake") == 0) {
            // Force a fresh pathing bake even when the existing .probes
            // file already has a valid pathing section. See
            // PLAN.AUDIO_PROFILING.md §4.3.
            cfg.forcePathingBake = true;
        } else if (std::strcmp(argv[i], "--set") == 0 && i + 1 < argc) {
            // --set audio.foo.bar=value  (yaml-dotted-path=leaf)
            std::string arg = argv[++i];
            auto eq = arg.find('=');
            if (eq == std::string::npos) {
                std::fprintf(stderr,
                    "[FALLBACK] --set: missing '=' in '%s' — expected "
                    "audio.dotted.path=value (ignored)\n", arg.c_str());
                continue;
            }
            std::string path  = arg.substr(0, eq);
            std::string value = arg.substr(eq + 1);
            if (!applySetOverride(path, value, cfg)) {
                std::fprintf(stderr,
                    "[FALLBACK] --set: unknown YAML path '%s' — ignored "
                    "(value was '%s')\n", path.c_str(), value.c_str());
            } else {
                std::fprintf(stderr,
                    "--set: '%s' = '%s' applied (clamped to legal range)\n",
                    path.c_str(), value.c_str());
            }
        } else if (std::strcmp(argv[i], "--perf-label") == 0 && i + 1 < argc) {
            std::string label = argv[++i];
            if (!isPerfLabelValid(label)) {
                std::fprintf(stderr,
                    "[FALLBACK] --perf-label: '%s' rejected — must be "
                    "1-64 chars of [A-Za-z0-9_.-]. Keeping default '%s'.\n",
                    label.c_str(), cfg.perfLabel.c_str());
            } else {
                cfg.perfLabel = label;
            }
        } else if (std::strcmp(argv[i], "--exit-after-seconds") == 0 && i + 1 < argc) {
            try {
                cfg.exitAfterSeconds = std::stof(argv[++i]);
                if (cfg.exitAfterSeconds < 0.0f) cfg.exitAfterSeconds = 0.0f;
            } catch (...) {
                std::fprintf(stderr,
                    "[FALLBACK] --exit-after-seconds: non-numeric value "
                    "'%s' — ignored (run remains open-ended)\n", argv[i]);
            }
        } else if (std::strcmp(argv[i], "--auto-fly") == 0) {
            cfg.autoFly = true;
        } else if (std::strcmp(argv[i], "--auto-fly-speed") == 0 && i + 1 < argc) {
            try {
                cfg.autoFlySpeed = std::stof(argv[++i]);
                if (cfg.autoFlySpeed < 0.0f) cfg.autoFlySpeed = 0.0f;
            } catch (...) {
                std::fprintf(stderr,
                    "[FALLBACK] --auto-fly-speed: non-numeric value '%s'"
                    " — keeping default %.1f\n", argv[i], cfg.autoFlySpeed);
            }
        } else if (std::strcmp(argv[i], "--auto-fly-waypoints") == 0 && i + 1 < argc) {
            try {
                cfg.autoFlyWaypoints = std::stoi(argv[++i]);
                if (cfg.autoFlyWaypoints < 1) cfg.autoFlyWaypoints = 1;
            } catch (...) {
                std::fprintf(stderr,
                    "[FALLBACK] --auto-fly-waypoints: non-integer value "
                    "'%s' — keeping default %d\n",
                    argv[i], cfg.autoFlyWaypoints);
            }
        } else if (std::strcmp(argv[i], "--auto-fly-seed") == 0 && i + 1 < argc) {
            try {
                // Accept decimal or 0xHEX; std::stoul base=0 auto-detects.
                cfg.autoFlySeed = static_cast<uint32_t>(
                    std::stoul(argv[++i], nullptr, 0));
            } catch (...) {
                std::fprintf(stderr,
                    "[FALLBACK] --auto-fly-seed: invalid '%s' — keeping "
                    "default 0x%08x\n", argv[i], cfg.autoFlySeed);
            }
        } else if (std::strcmp(argv[i], "--auto-fly-pause-sec") == 0 && i + 1 < argc) {
            try {
                cfg.autoFlyPauseSec = std::stof(argv[++i]);
                if (cfg.autoFlyPauseSec < 0.0f) cfg.autoFlyPauseSec = 0.0f;
            } catch (...) {
                std::fprintf(stderr,
                    "[FALLBACK] --auto-fly-pause-sec: non-numeric value "
                    "'%s' — keeping default %.2f\n",
                    argv[i], cfg.autoFlyPauseSec);
            }
        } else if (std::strcmp(argv[i], "--audio-capture") == 0 && i + 1 < argc) {
            // --audio-capture x,y,z — pin the listener at a world point and
            // spin in place for a hands-free acoustic capture. Parsed as a
            // single comma-separated arg so the position is one shell token.
            float x = 0.0f, y = 0.0f, z = 0.0f;
            if (std::sscanf(argv[++i], "%f,%f,%f", &x, &y, &z) == 3) {
                cfg.audioCapture  = true;
                cfg.audioCaptureX = x;
                cfg.audioCaptureY = y;
                cfg.audioCaptureZ = z;
            } else {
                std::fprintf(stderr,
                    "[FALLBACK] --audio-capture: could not parse '%s' as "
                    "x,y,z (e.g. --audio-capture 12.5,-45,-47) — capture "
                    "disabled\n", argv[i]);
            }
        } else if (std::strcmp(argv[i], "--audio-capture-seconds") == 0 && i + 1 < argc) {
            try {
                cfg.audioCaptureSeconds = std::stof(argv[++i]);
                if (cfg.audioCaptureSeconds < 0.1f) cfg.audioCaptureSeconds = 0.1f;
            } catch (...) {
                std::fprintf(stderr,
                    "[FALLBACK] --audio-capture-seconds: non-numeric value "
                    "'%s' — keeping default %.1f\n",
                    argv[i], cfg.audioCaptureSeconds);
            }
        } else if (std::strcmp(argv[i], "--audio-capture-rotations") == 0 && i + 1 < argc) {
            try {
                cfg.audioCaptureRotations = std::stof(argv[++i]);
                if (cfg.audioCaptureRotations < 0.0f) cfg.audioCaptureRotations = 0.0f;
            } catch (...) {
                std::fprintf(stderr,
                    "[FALLBACK] --audio-capture-rotations: non-numeric value "
                    "'%s' — keeping default %.1f\n",
                    argv[i], cfg.audioCaptureRotations);
            }
        } else if (std::strcmp(argv[i], "--auto-run") == 0) {
            cfg.autoRun = true;
        } else if (std::strcmp(argv[i], "--auto-run-waypoints") == 0 && i + 1 < argc) {
            try {
                cfg.autoRunWaypoints = std::stoi(argv[++i]);
                if (cfg.autoRunWaypoints < 1) cfg.autoRunWaypoints = 1;
            } catch (...) {
                std::fprintf(stderr,
                    "[FALLBACK] --auto-run-waypoints: non-integer value "
                    "'%s' — keeping default %d\n",
                    argv[i], cfg.autoRunWaypoints);
            }
        } else if (std::strcmp(argv[i], "--auto-run-seed") == 0 && i + 1 < argc) {
            try {
                // Accept decimal or 0xHEX; std::stoul base=0 auto-detects.
                cfg.autoRunSeed = static_cast<uint32_t>(
                    std::stoul(argv[++i], nullptr, 0));
            } catch (...) {
                std::fprintf(stderr,
                    "[FALLBACK] --auto-run-seed: invalid '%s' — keeping "
                    "default 0x%08x\n", argv[i], cfg.autoRunSeed);
            }
        } else if (std::strcmp(argv[i], "--auto-run-speed-mode") == 0 && i + 1 < argc) {
            const char *mode = argv[++i];
            if (std::strcmp(mode, "run") == 0 || std::strcmp(mode, "walk") == 0
                || std::strcmp(mode, "creep") == 0) {
                cfg.autoRunSpeedMode = mode;
            } else {
                std::fprintf(stderr,
                    "[FALLBACK] --auto-run-speed-mode: unknown mode '%s' "
                    "(expected run|walk|creep) — keeping default '%s'\n",
                    mode, cfg.autoRunSpeedMode.c_str());
            }
        } else if (std::strcmp(argv[i], "--audio-rng-seed") == 0 && i + 1 < argc) {
            try {
                // Accept decimal or 0xHEX. Negative = leave unseeded.
                cfg.audioRngSeed = static_cast<int64_t>(
                    std::stoll(argv[++i], nullptr, 0));
            } catch (...) {
                std::fprintf(stderr,
                    "[FALLBACK] --audio-rng-seed: invalid '%s' — keeping "
                    "unseeded (random_device)\n", argv[i]);
            }
        } else if (std::strcmp(argv[i], "--stress-doors") == 0 && i + 1 < argc) {
            // DEV-ONLY door-swing stress harness (see AppConfig comment).
            try {
                cfg.stressDoors = std::stoi(argv[++i]);
                if (cfg.stressDoors < 0) cfg.stressDoors = 0;
            } catch (...) {
                std::fprintf(stderr,
                    "[FALLBACK] --stress-doors: non-integer value '%s' — "
                    "harness stays disabled\n", argv[i]);
                cfg.stressDoors = 0;
            }
        } else if (std::strcmp(argv[i], "--stress-door-ids") == 0 && i + 1 < argc) {
            // DEV-ONLY explicit-door companion to --stress-doors (see
            // AppConfig comment). Comma-separated object IDs; a malformed
            // token discards the whole list LOUDLY rather than silently
            // stressing a partial set.
            const char* arg = argv[++i];
            std::vector<int32_t> ids;
            bool ok = true;
            const char* p = arg;
            while (*p != '\0') {
                char* end = nullptr;
                long v = std::strtol(p, &end, 10);
                if (end == p) { ok = false; break; }
                ids.push_back(static_cast<int32_t>(v));
                p = end;
                if (*p == ',') ++p;
                else if (*p != '\0') { ok = false; break; }
            }
            if (ok && !ids.empty()) {
                cfg.stressDoorIDs = ids;
            } else {
                std::fprintf(stderr,
                    "[FALLBACK] --stress-door-ids: malformed list '%s' — "
                    "explicit-door override stays disabled\n", arg);
            }
        } else if (std::strcmp(argv[i], "--spawn-override") == 0 && i + 1 < argc) {
            // DEV-ONLY positioned-start override (see AppConfig comment).
            // "x,y,z" or "x,y,z,yaw"; a malformed token discards the
            // whole override LOUDLY rather than silently starting
            // somewhere unintended.
            const char* arg = argv[++i];
            float vals[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            int   count = 0;
            bool  ok = true;
            const char* p = arg;
            while (*p != '\0' && count < 4) {
                char* end = nullptr;
                float v = std::strtof(p, &end);
                if (end == p) { ok = false; break; }
                vals[count++] = v;
                p = end;
                if (*p == ',') ++p;
                else if (*p != '\0') { ok = false; break; }
            }
            if (ok && *p == '\0' && (count == 3 || count == 4)) {
                cfg.spawnOverride    = true;
                cfg.spawnOverrideX   = vals[0];
                cfg.spawnOverrideY   = vals[1];
                cfg.spawnOverrideZ   = vals[2];
                cfg.spawnOverrideYaw = (count == 4) ? vals[3] : 0.0f;
            } else {
                std::fprintf(stderr,
                    "[FALLBACK] --spawn-override: malformed \"x,y,z[,yaw]\" "
                    "value '%s' — using the mission spawn\n", arg);
            }
        } else if (std::strcmp(argv[i], "--bake-quality") == 0 && i + 1 < argc) {
            // Bake-quality profile. "dev" forces the fast iteration bake
            // (rays=2048 bounces=8 diffuse=256, ~64× cheaper per probe
            // than the ship-quality yaml settings — MISS2's 2590-probe
            // ship bake projected ~25 HOURS). "ship" is a documented
            // no-op: respect the yaml. Applied here because CLI parsing
            // runs after the yaml load, so this deliberately overrides
            // reflections.bake.* values.
            const char *q = argv[++i];
            if (std::strcmp(q, "dev") == 0) {
                // Reflection rays halved 4096 → 2048 (2026-07: dev bakes
                // must total < 10 min; reflection phase measured ~10 min
                // at 4096 on MISS2's 626 dev probes, cost is linear in
                // rays → ~5 min). Dev-tier IR fidelity is acceptable for
                // iteration; ship bakes are untouched.
                cfg.bakeNumRays        = 2048;
                cfg.bakeNumBounces     = 8;
                cfg.bakeDiffuseSamples = 256;
                // Pathing visibility sampling selects
                // kPathingVisSamplesDev for BOTH bake and runtime — see
                // devBakeProfile. (Ship and Dev are both 4 since
                // 2026-07-11, so this no longer changes ray count; the
                // profile plumbing is kept for the .probes header record.)
                cfg.devBakeProfile     = true;
                // Density reduction (user directive 2026-07-05: dev bakes
                // must be < 10 min). FLOOR_POLY emits one candidate per
                // BSP floor polygon regardless of spacing, so the GLOBAL
                // DEDUP radius is the density control: 12 ft collapses
                // MISS2's 3,304 candidates ~4-6x harder than the yaml's
                // 5 ft. Spacing raised alongside because committed probes
                // get influence radius = spacing — the sparser set must
                // still cover the level or [PERF refl_cache] hitRate
                // drops and reverb regions go dark (watch that metric on
                // every dev-bake validation).
                cfg.audioProbeGlobalDedupRadiusFt = 18.0f;
                cfg.audioProbeSpacingFt           = 20.0f;
                std::fprintf(stderr,
                    "--bake-quality dev: bake rays=2048 bounces=8 "
                    "diffuse=256 (~64x cheaper/probe than ship yaml) + "
                    "probe density reduced (global_dedup 18 ft, spacing "
                    "20 ft); pathing numSamples 4 (same as ship; "
                    "recorded in the .probes header). Cached "
                    ".probes from this bake are DEV QUALITY/DENSITY — "
                    "re-bake without this flag for milestone/ship "
                    "fidelity (the header mismatch check will do it "
                    "automatically, pathing-only).\n");
            } else if (std::strcmp(q, "ship") != 0) {
                std::fprintf(stderr,
                    "[FALLBACK] --bake-quality: unknown profile '%s' "
                    "(expected dev|ship) — keeping yaml bake settings\n", q);
            }
        } else if (std::strcmp(argv[i], "--audio-log") == 0) {
            // CLI mirror of the YAML `developer.audio_log` key. Perf runs
            // need it: nearly all [PERF *] histogram recording is gated on
            // the audio_log verbosity flag, and the generic --set resolver
            // only covers audio.* leaves — developer.* is out of its reach.
            cfg.audioLog = true;
        } else if (std::strcmp(argv[i], "--capture-wav") == 0) {
            // CLI mirror of the YAML `developer.capture_wav` key. Records
            // the engine's final stereo output to output.wav next to the
            // per-run audio_perf.jsonl — listenable evidence for A/B runs,
            // analyzed offline by tools/wav_artifacts.py
            // (PLAN.AUDIO_PERF.md PR 0.2).
            cfg.captureWav = true;
        } else if (argv[i][0] != '-' && !cli.misPath) {
            // First non-flag argument is the mission file
            cli.misPath = argv[i];
        } else if (argv[i][0] == '-') {
            std::fprintf(stderr,
                "Warning: unknown CLI flag '%s' (ignored). All tunables live in the YAML config; run --help for the full list.\n",
                argv[i]);
        }
    }

    return cli;
}

} // namespace Darkness
