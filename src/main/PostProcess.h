/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2025 darkness contributors
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

// PostProcess — offscreen scene target and the composite pass that resolves
// it to the display backbuffer.
//
// WHY THIS EXISTS AT ALL
//
// The lightmapped world shader computes `diffuse.rgb * light.rgb * 2.0`
// (shaders/fs_lightmapped.sc). That 2x is the Dark Engine's overbright
// lightmap convention — NewDark's own documentation describes the same
// mode as "a cheap man's HDR-like effect on lightmaps", where "the texture
// can be up to twice as bright as the fullbright texture"
// (new_dark/doc/new_config_vars.txt, `set_lighting_depth` / 32-bit 2X mode).
//
// Rendering straight to an 8-bit backbuffer throws the top half of that
// range away every frame: everything above 1.0 clamps flat. Routing the
// scene through a floating-point target first preserves it, which is what
// gives tone mapping something to roll off and (later) bloom something real
// to threshold. This is recovering signal the renderer already computes, not
// synthesising a new effect.
//
// VIEW LAYOUT
//
//   View 0  Sky      ─┐
//   View 1  World    ─┼─ render into mFrameBuffer when enabled,
//   View 2  Debug    ─┘  or straight to the backbuffer when disabled
//   View 3  Composite → always the backbuffer
//
// bgfx submits views in id order, so the composite lands last. bgfx's own
// debug-text overlay (the backtick console) draws to the backbuffer
// independently of all four views, so console text is never tone mapped.
//
// ENABLE/DISABLE IS PER-FRAME
//
// The target binding is re-applied every frame from the current toggle
// rather than latched at init, so the debug console can switch
// post-processing on and off live without recreating GPU resources. When
// disabled the scene views bind BGFX_INVALID_HANDLE — the exact
// direct-to-backbuffer path that existed before this module — so the
// "off" state carries no residual cost and no behavioural drift.

#pragma once

#include <cstdint>
#include <cstdio>
#include <cmath>
#include <limits>
#include <vector>

#include <bgfx/bgfx.h>

#include "SMAALookupTextures.h"

namespace Darkness {

// bgfx view ids. Views 0-2 predate this module.
//
// bgfx submits views in id order, so the bloom chain must occupy ids
// between the scene and the composite. Each blur pass needs its own id
// because each targets a different framebuffer; two passes (horizontal +
// vertical) per iteration.
static constexpr bgfx::ViewId kViewSky          = 0;
static constexpr bgfx::ViewId kViewWorld        = 1;
// Light coronas. Their own view, not the tail of kViewWorld, because the
// corona shader SAMPLES the scene depth buffer — and a texture cannot be
// bound as a framebuffer attachment and as a sampler in the same draw. This
// view therefore targets `colorOnlyFrameBuffer`: the same colour texture,
// no depth attachment, so depth is free to be read. Coronas neither test nor
// write depth (see renderCoronas), so they lose nothing by its absence.
static constexpr bgfx::ViewId kViewCorona       = 2;
static constexpr bgfx::ViewId kViewDebug        = 3;
// Bright-pass extraction. Only the NewDark style submits it; the Amnesia
// style blurs the scene directly and leaves this view unused.
static constexpr bgfx::ViewId kViewBloomExtract = 4;
static constexpr bgfx::ViewId kViewBloomBase    = 5;

// HPL2 ships 2 iterations (cPostEffectParams_Bloom::mlBlurIterations).
// Reserve headroom for 4 so the setting is explorable without renumbering
// views; unused ids simply never get submitted.
static constexpr int kMaxBloomIterations = 4;

// Bloom pyramid (VIS-3b). Level 0 IS the existing quarter-res blur output;
// the pyramid adds levels below it, each half the previous in both axes.
//
// The pyramid is what makes a WIDE radius possible at all: the 5-tap kernel
// is only a Gaussian at its designed tap spacing, so radius cannot come from
// a bigger step (see fs_bloom_blur.sc and VIS-2f). Halving resolution widens
// the reach while every filter stays at one texel of its own level.
//
// WHY NINE. A level-0 texel is 4 screen pixels at ANY resolution, because
// level 0 is always quarter res — so a given pyramid depth reaches a fixed
// number of PIXELS. `bloom_range` is a percentage of the screen diagonal,
// which grows with resolution, so higher resolutions need more depth for the
// same setting. Measured: NewDark's default 2% needs 6 levels at 720p and 9
// at 4K. Nine is therefore the count that keeps the documented default
// reachable on a 4K display; the allocation loop stops early by itself when a
// level would go degenerate (720p runs out of halvings at 7).
static constexpr int kMaxBloomLevels = 9;

/// Glow radius each pyramid depth actually produces, in LEVEL-0 TEXELS
/// (multiply by 4 for screen pixels). Index is `levels - 1`.
///
/// MEASURED, not derived. The obvious model — depth k doubles the reach, so
/// radius is 2^(levels-1) — is wrong here, because the scatter lerp weights
/// the widest level at only `scatter^(levels-1)` of the result. Trusting the
/// model made `bloom_range` under-deliver by ~35% at its own default and more
/// at higher settings, which is exactly the "key that lies" failure the
/// setting was deleted for in VIS-2f. These numbers come from re-executing
/// the real down/up chain on an impulse and taking the energy-weighted RMS
/// radius (`<scratchpad>/pyramid.py`).
///
/// Resolution-independent to within ~5% (checked at 720p / 1080p / 4K); the
/// residual is odd-dimension rounding in the smallest levels. Values are the
/// 4K measurement, whose halvings are the cleanest.
///
/// Re-measure if kBloomScatter, the tent weights, or the downsample change —
/// all three move these numbers.
static constexpr float kBloomPyramidRadius[kMaxBloomLevels] = {
    0.0000f, 1.7321f, 3.0000f,  4.5835f,  6.7980f,
    10.2702f, 14.3948f, 20.1728f, 25.6736f,
};

/// How strongly each pyramid level is mixed into the one above it, on the way
/// back up. The blend is a lerp rather than an add (see fs_bloom_upsample.sc),
/// so the weights across the whole pyramid sum to 1 at every step and widening
/// the glow does not brighten it.
///
/// **This is the one number in the pyramid that is a judgement call rather
/// than derived, and it cannot be verified from here — `screencapture` is
/// blocked.** It trades core sharpness against halo spread: the base level
/// keeps `(1-scatter)^(levels-1)` of the final weight and the widest level
/// `scatter^(levels-1)`, so higher values scatter more light outward and leave
/// the bright core softer. 0.5 weights the scales evenly and is the neutral
/// starting point; if the glow reads as too hazy around lamps, this is the
/// constant to lower, not bloom_range.
static constexpr float kBloomScatter = 0.5f;

// One down pass and one up pass per level TRANSITION, hence kMaxBloomLevels-1
// of each. Both ranges sit after the ping-pong blur views so that chain keeps
// its ids.
static constexpr bgfx::ViewId kViewBloomPyrDown =
    kViewBloomBase + kMaxBloomIterations * 2;
static constexpr bgfx::ViewId kViewBloomPyrUp =
    kViewBloomPyrDown + (kMaxBloomLevels - 1);

// Halation (VIS-13) has its OWN extract + blur chain rather than borrowing
// the bloom texture, because it is the opposite kind of glow. Bloom here is
// wide (bloom_range is a percentage of the screen diagonal) and, in the
// amnesia style, thresholdless. Halation is tight and hard-thresholded: on
// film it only appears where light was intense enough to punch through the
// emulsion and reflect off the base, so a dim lamp does not halate at all.
// Tinting the bloom texture gave neither property.
//
// 4 iterations at quarter res spans roughly 6.5 to 13 screen pixels of
// radius, which is the local range this effect wants. For a wide warm glow,
// that is what bloom itself is for.
static constexpr int kMaxHalationIterations = 4;

static constexpr bgfx::ViewId kViewHalationExtract =
    kViewBloomPyrUp + (kMaxBloomLevels - 1);
static constexpr bgfx::ViewId kViewHalationBase = kViewHalationExtract + 1;

static constexpr bgfx::ViewId kViewComposite =
    kViewHalationBase + kMaxHalationIterations * 2;

// SMAA, three passes, after the composite because it antialiases the FINAL
// display-referred image — the one tone mapping and colour correction have
// already produced. Running it on the HDR scene instead would have it decide
// what counts as an edge using contrast the tone curve is about to change.
//
// When SMAA is off these ids are never submitted and the composite targets
// the backbuffer directly, exactly as before.
static constexpr bgfx::ViewId kViewSmaaEdges   = kViewComposite + 1;
static constexpr bgfx::ViewId kViewSmaaWeights = kViewComposite + 2;
static constexpr bgfx::ViewId kViewSmaaBlend   = kViewComposite + 3;

// Film grain, LAST — after antialiasing, immediately before UI.
//
// Report §9.1 (NOTES.FILM_GRAIN.md): grain must follow AA and any temporal
// upscale, because AA smooths it away and temporal upscalers reject it as
// invalid history. It previously lived inside the composite, i.e. BEFORE
// SMAA, which meant SMAA antialiased the grain and grain strength depended
// on whether antialiasing happened to be on.
//
// When grain is off this view is never submitted and the chain ends exactly
// where it did before — composite or SMAA blend straight to the backbuffer.
static constexpr bgfx::ViewId kViewGrain = kViewComposite + 4;

// ── Shadow-map faces (S1, PLAN.HIGH_RES_SHADOWS.md) — the TAIL of the view
// range, after every screen pass. Face renders target the shadow atlas
// framebuffer, so their position in the order only matters relative to
// consumers of the atlas: sitting at the tail means an update submitted
// during a frame lands one frame late for same-frame readers, which is
// acceptable for S3's door events (revisit with bgfx::setViewOrder if S4
// needs same-frame). One view per (pool slot, face): each carries its own
// viewport rect into the tiled atlas and its own face view-projection.
//
// kShadowMaxPoolSlots bounds the VIEW ID RESERVATION (and the runtime
// pool); 6 is kShadowFaceCount (static_asserted in ShadowMapCache.h).
// 66 slots = 32 concurrent S4c differentials (2 slots each: frozen +
// current) + live emitters (player lantern etc.) + spare. 32 shadowed
// live lights is the
// upper industry-standard tier (Unity HDRP defaults 24 shadowed
// punctuals; Godot's whole default atlas is 88 low-res tiles; HPL2
// ships 11 maps). The old per-slot view scheme capped the pool at ~38
// slots (bgfx 256-view budget); every face now renders into the ONE
// kViewShadowFaces via per-draw tile remap + scissor, so the pool is
// bounded by atlas MEMORY alone (66 slots @256² ≈ 100 MB R32F + the
// same again for depth — shadow_face_size scales it down).
static constexpr int kShadowMaxPoolSlots = 66;
static constexpr bgfx::ViewId kViewShadowFaces = kViewGrain + 1;
// One id past the face view: the shadow debug HUD (draws the atlas tiles
// to the backbuffer) and the readback blit both live here — both must run
// AFTER every face render, and neither targets the atlas.
static constexpr bgfx::ViewId kViewShadowDebug = kViewShadowFaces + 1;
// S2 lumel-bake pass: GPU overlay re-bakes draw packed rects into a
// scratch RT here — after the face views (it samples the atlas they
// write) and before nothing (readback consumes it).
static constexpr bgfx::ViewId kViewLumelBake = kViewShadowDebug + 1;
// Blit/readback companion: bgfx executes a view's blits BEFORE its draw
// items, so copying a lumel-bake result must happen on a LATER view.
static constexpr bgfx::ViewId kViewLumelRead = kViewLumelBake + 1;
static_assert(kViewLumelRead < 256,
              "bgfx view budget exceeded — shrink kShadowMaxPoolSlots");

/// Which engine's bloom construction to run.
///
/// Not two tunings of one effect — two different places to decide what
/// glows. Amnesia has no bright pass at all and suppresses dim areas via a
/// luminance weight in the combine, giving soft glow that fades in
/// continuously. NewDark thresholds up front, so glow appears only above a
/// cutoff and is then desaturated and scaled hard (default scale 5).
enum class BloomStyle : int {
    Amnesia = 0,
    NewDark = 1,
};

/// Luminance weighting, i.e. "what counts as bright".
///
/// Used for both the saturation pivot and the bloom intensity term, because
/// they are the same question asked twice.
enum class LumaMode : int {
    // Rec.601-era weights. This is also exactly HPL2's shipped bloom vector
    // (cPostEffectParams_Bloom::mvRgbToIntensity = 0.3, 0.58, 0.12), so CRT
    // mode is simultaneously the original-colour-intent choice and the
    // Amnesia-faithful one.
    //
    // The right default for this content: Thief 2's textures were authored
    // in 1999 against CRTs. Rec.709 weights green harder and pulls
    // desaturated output greener than the source art intends.
    CRT = 0,
    // Rec.709 / sRGB weights — correct for how a modern LCD actually
    // displays, and what contemporary engines assume.
    //
    // This is also NewDark's choice: cc.fx declares
    // LUMINANCE_VECTOR = (0.2125, 0.7154, 0.0721). We use the exact Rec.709
    // values instead of NewDark's rounding; the two differ by under 1/4000
    // per channel, far below an 8-bit quantisation step.
    //
    // So: LCD mode is the NewDark-matched setting, CRT mode is the
    // HPL2/Amnesia-matched one. They genuinely disagree, and the split is
    // the point — this enum is the one place that disagreement is decided.
    LCD = 1,
};

inline void lumaWeightsFor(LumaMode mode, float outRGB[3]) {
    if (mode == LumaMode::LCD) {
        outRGB[0] = 0.2126f; outRGB[1] = 0.7152f; outRGB[2] = 0.0722f;
    } else {
        outRGB[0] = 0.30f;   outRGB[1] = 0.58f;   outRGB[2] = 0.12f;
    }
}

// Tone-mapping operators. Values are the wire format for the shader's
// u_ccParams0.y and for the config/console enum — keep them stable.
//
// The two modern curves (VIS-3) exist because ACES, while still the safe
// default, is a 2015 fit that lifts middle grey (0.18 -> 0.267) and
// desaturates warm highlights hard — a torch at (2.0, 1.0, 0.4) leaves ACES
// at (0.91, 0.80, 0.54), close to yellow-white. On a game whose whole look
// is small warm sources in the dark that is the wrong failure mode.
//
// Numbers here and in fs_composite.sc were produced by re-implementing every
// curve in Python against this renderer's actual signal range and asserting
// monotonicity, black preservation, output range and NaN-freedom over 0..4
// plus the whole 0..2 RGB cube.
enum class ToneMapOperator : int {
    None     = 0,  // clamp only: reproduces the pre-post-process image exactly
    Reinhard = 1,
    ACES     = 2,

    /// AgX, via Godot 4's closed-form approximation of Blender's AgX.
    ///
    /// The one that fits this renderer best, for a reason that is worth
    /// stating rather than rediscovering: VIS-3 warned that our signal is
    /// ~0..2 rather than physically-scaled radiance and that a curve tuned
    /// for real HDR input would need its exposure remapped. AgX does not.
    /// Godot's SDR white point is 2.0 and our ceiling is 2.0 because of the
    /// lightmap overbright convention, so the curve's anchors already land
    /// on ours: 0.18 maps to 0.18 exactly and 2.0 maps to 1.0 exactly.
    AgX = 3,

    /// Khronos PBR Neutral (2024). Preserves albedo rather than being
    /// filmic — hue is left alone until the very top of the range. Its
    /// black-point offset is unconditional, so mid greys come out 0.04
    /// darker; on Thief content that reads as deeper blacks.
    PBRNeutral = 4,

    /// Timothy Lottes (GDC 2016), with midIn/midOut pinned to 0.18.
    ///
    /// AgX's midtone behaviour with a far softer rolloff at both ends, which
    /// is the specific thing AgX does not give. Measured against it:
    ///
    /// | | AgX @1.25 | Lottes @1.30 |
    /// |---|---|---|
    /// | middle grey 0.18 | 0.1800 | 0.1800 |
    /// | shoulder length  | 0.76   | **1.36** |
    /// | toe gradient     | 1.013 (linear) | **0.986** (compressing) |
    /// | ceiling 2.0      | 1.000 (clips) | 0.915 (headroom) |
    ///
    /// The shoulder number is the point: 1.8x more scene range spent on the
    /// top of the rolloff is the difference between a highlight that rolls
    /// and one that arrives. Leaving 2.0 at 0.915 rather than pure white is
    /// deliberate too — film rarely reaches paper white, and the reserve is
    /// what lets a torch stay coloured instead of clipping.
    Lottes = 5,
};

/// Colour-correction + tone-mapping settings.
///
/// Defaults are deliberately the identity transform, and match the defaults
/// of NewDark's software colour-correction vars (`d3d_disp_sw_cc_bright` 0,
/// `_contr` 1, `_sat` 1, `_rgbfilter` 1 1 1) so settings carry over with the
/// same meaning. `tonemap` is the one knob whose default is ours to pick:
/// None keeps first-boot output identical to the direct path.
struct PostProcessSettings {
    bool  enabled    = false;   // master toggle; false = legacy direct path
    float exposure   = 1.0f;
    ToneMapOperator tonemap = ToneMapOperator::None;
    float brightness = 0.0f;
    float contrast   = 1.0f;
    float saturation = 1.0f;
    float filterR    = 1.0f;
    float filterG    = 1.0f;
    float filterB    = 1.0f;
    float gamma      = 1.0f;
    LumaMode lumaMode = LumaMode::CRT;

    // ── Bloom ──
    // Defaults are HPL2's shipped values (Amnesia: The Dark Descent):
    // mlBlurIterations 2, mfBlurSize 1.0, and an intensity scale of 1.0
    // multiplying the luma vector — which is literally what
    // LuxMapHandler.cpp:171 does (`mvRgbToIntensity * 1.0f`), i.e. the
    // intended tuning knob is a scale on that vector, not a threshold.
    //
    // Disabled by default so the identity property holds.
    bool  bloomEnabled    = false;
    BloomStyle bloomStyle = BloomStyle::Amnesia;
    int   bloomIterations = 2;
    float bloomBlurSize   = 1.0f;
    float bloomIntensity  = 1.0f;

    // ── NewDark bloom style ──
    // Defaults are NewDark 1.28's documented ones, verbatim from
    // new_dark/doc/new_config_vars.txt lines 369-384:
    //   bloomprescale 1, bloomscale 5, bloom_saturation 0.7,
    //   bloom_threshold 0.6.
    // Ignored entirely when bloomStyle is Amnesia.
    //
    float ndThreshold  = 0.6f;
    float ndPrescale   = 1.0f;
    float ndScale      = 5.0f;
    float ndSaturation = 0.7f;

    // ── Bloom radius (VIS-3b) ──
    //
    // NewDark's fifth bloom parameter, `bloom_range`: glow radius as a
    // PERCENTAGE OF THE SCREEN DIAGONAL, NewDark's default 2
    // (new_config_vars.txt:369-384). At 1280x720 that is a ~29px radius.
    //
    // History worth knowing, because this key was deleted once already. It
    // could not be honoured by the ping-pong blur: the 5-tap kernel is a
    // 9-tap Gaussian collapsed onto bilinear pairs and is only a Gaussian at
    // its designed tap spacing, so a wide radius asked for a step that turned
    // the kernel into a sparse comb — which aliases, and read on screen as
    // grain. Rather than leave a key that lied, VIS-2f removed it (CFG-1: a
    // key is deleted or made real, never both). This is it being made real
    // instead, via the progressive-downsample pyramid: each level halves
    // resolution, so radius comes from pyramid DEPTH while the step stays at
    // one texel where the kernel is valid.
    //
    // 0 DISABLES THE PYRAMID, and that is the default. The chain then runs
    // exactly as it did before this existed — the ping-pong blur alone, which
    // is HPL2's actual construction and therefore what the amnesia style
    // should be doing. Set it to 2 to get NewDark's documented radius.
    //
    // Applies to both styles: it controls how far the glow spreads, which is
    // orthogonal to the styles' disagreement about what glows in the first
    // place.
    float bloomRange = 0.0f;

    // ── Tone-curve shaping ──
    // Both are operator-scoped: agxContrast is read only when tonemap is AgX,
    // the lottes* trio only when it is Lottes. Defaults reproduce each
    // curve's reference configuration exactly.
    float agxContrast    = 1.25f;   // Godot's default

    // Lottes' contrast and shoulder. Shoulder is the parameter no other
    // operator here exposes, and the reason this curve was added.
    float lottesContrast = 1.30f;
    float lottesShoulder = 0.977f;
    // The scene value that maps to display white. NOT a free knob: the b/c
    // constants are solved so f(white) == 1.0, and everything above it clips.
    // 3.0 leaves our own 2.0 ceiling at 0.915, i.e. a little headroom above
    // the brightest thing the scene can produce, which is what keeps flames
    // coloured rather than clipped.
    float lottesWhite    = 3.0f;

    // ── Film response (VIS-12) ──
    //
    // Master strength first, because it is the knob that matters: 0 makes the
    // whole stage an exact no-op and preserves the identity property, and
    // everything below describes the look that strength scales toward.
    float filmStrength = 0.0f;

    // Saturation at each end of the range. The shadow figure is the one doing
    // the work — it desaturates the gloom without touching the torches, which
    // a single global saturation scalar cannot do.
    float filmShadowSat    = 0.55f;
    float filmHighlightSat = 1.0f;

    // Split tone, multiplicative, so neither end can add light.
    //
    // SHADOW TINT IS NEUTRAL BY DEFAULT. Cool shadows are the obvious move
    // and are wrong past a very small amount — visibly blue shadows are the
    // Thief 3 look, which reads as a filter over the world rather than as
    // darkness in it. The original engine never tints its ambient in any of
    // the 32 retail missions of either game, and Thief 1 has no field with
    // which to. Shadows here get desaturated, not coloured; the knob exists
    // for a whisper of it, and (0.97, 0.98, 1.00) is about as far as it
    // should go.
    float filmShadowTint[3]    = { 1.0f, 1.0f, 1.0f };

    // Highlights warm, because the light sources in this game are fire.
    // Restrained on purpose — the reference look is a warm SOURCE in a
    // neutral world, not a sepia frame.
    float filmHighlightTint[3] = { 1.0f, 0.98f, 0.94f };

    // How tightly each tint is confined to its own end. Higher leaves more
    // of the midtones untouched.
    float filmToneFalloff = 1.5f;

    // ── Halation (VIS-13) ──
    //
    // The CineStill effect. That stock is Kodak Vision3 with the remjet
    // anti-halation backing stripped off: light bright enough to punch
    // through the emulsion reflects off the film base and re-exposes it from
    // behind, and because long wavelengths penetrate deepest the halo it
    // leaves is red-orange.
    //
    // Two properties define it, and BOTH are things the first implementation
    // got wrong by tinting the bloom texture:
    //
    //  * IT IS THRESHOLDED HARD. Only genuinely intense sources halate — a
    //    dim lamp does not. Borrowing the bloom texture inherited the amnesia
    //    style's lack of a bright pass, so everything glowed a bit red.
    //  * IT IS LOCAL. A halo hugging the source, not a screen-scale wash.
    //    Borrowing the bloom texture inherited bloom_range, which at the
    //    shipped 2% is a ~27px pyramid — twice the radius this wants and
    //    spread over the whole frame.
    //
    // So halation now runs its own extract + blur, independent of bloom.
    // It follows the same recipe the digital-photography version does:
    // isolate the highlights, tint them, blur by a SMALL radius, add back.
    float halationStrength = 0.0f;
    float halationTint[3]  = { 1.0f, 0.32f, 0.12f };   // red-orange

    /// Value a pixel must exceed, AFTER EXPOSURE, to halate at all.
    ///
    /// **Measured, and the first value shipped here was wrong.** VIS-13 set
    /// this to 1.0 reasoning that lit geometry "reaches 2.0" under the
    /// lightmap overbright convention. That is the CEILING, not the
    /// distribution: `fs_lightmapped.sc` computes `albedo * light * 2.0`, the
    /// atlas clamps light to 1.0, and the actual static lightmaps across five
    /// retail missions have a MEDIAN of 0.03-0.09 and a p99 of 0.47-0.85.
    /// Only 0.9-3.8% of texels even exceed 0.50 — which is what a pure WHITE
    /// texture would need to reach 1.0 at all. A threshold of 1.0 therefore
    /// sat above essentially the whole frame and the effect did nothing.
    /// `reportLightmapRange()` prints this per mission at load.
    ///
    /// 0.6 catches roughly the brightest one percent: surfaces at the top of
    /// the lighting range, plus coronas and anything emissive. Lower and
    /// ordinary lit stone starts to bleed, which is the wash this effect was
    /// rebuilt to stop being.
    ///
    /// Compared against the EXPOSED value, not the raw scene, so the number
    /// keeps meaning the same thing when exposure moves — exposure is part of
    /// how brightly the negative was exposed, and halation happens at the
    /// negative. Implemented by dividing the threshold through by exposure
    /// before it reaches the extract pass.
    float halationThreshold = 0.6f;

    /// Halo radius in OUTPUT PIXELS. Converted to blur iterations at quarter
    /// resolution, where one pass of the 5-tap kernel is about 6.5 px of
    /// sigma; the reachable span is therefore ~6.5 to 13 px, and asking for
    /// more says so rather than silently under-delivering.
    ///
    /// Deliberately a tight range. Halation is the LOCAL glow; if a wide
    /// warm spread is what is wanted, that is bloom's `range` plus a warm
    /// tint, and it is a different effect.
    float halationRadius = 10.0f;

    // ── Grain ──
    // Additive, with a negative-film response: pronounced in the shadows,
    // fading through the midtones, gone in the whites.
    float grainStrength = 0.0f;

    /// Output pixels per noise cell — the grain's physical size. The noise
    /// is value noise, so this is a real correlation length rather than a
    /// sampling stride: below ~1 it approaches per-pixel speckle, above ~2
    /// it clumps like a fast stock.
    float grainSize = 1.5f;

    /// Exponent on the process-emphasis factor `pow(1 - u, push)`, applied
    /// PER CHANNEL on top of the physical `sqrt(V(u))` curve.
    ///
    /// **0 is a normally-processed stock** — the physical curve alone,
    /// peaking at u = 0.436. Higher pushes grain toward the shadows, which is
    /// what pushed film does. 1.0 peaks at u = 0.234.
    ///
    /// Defaulted to 1.0 to match PLAN.FILM_GRAIN.md; it shipped at 2.0 once
    /// by accident, which peaks at 0.16 and fails the report's §11.2
    /// signature test (peak should be near 0.44 with the physical factor
    /// alone).
    float grainShadowBias = 1.0f;

    /// Non-physical multiplier, kept OUTSIDE the physical amplitude.
    ///
    /// Report §4.4 caps the physical model at sigma_n <= 0.076 and says a
    /// stronger look "should be a separate non-physical gain" rather than
    /// pushing the physical parameters out of validity. `grainStrength` is
    /// dimensionally mu_r/sigma, so the model's `sigma >= 3 mu_r` condition
    /// reads `strength <= 1/3` — that is where its range now stops, and
    /// anything beyond it lives here instead.
    float grainGain = 1.0f;
};

/// Derive the shader's curve constants for whichever operator is selected.
///
/// These involve several `pow()` of what are now uniforms, so they are
/// computed once per frame on the CPU rather than per pixel. Both branches
/// are their reference derivations: AgX's is Godot's
/// (`environment_storage.cpp`, `environment_get_tonemap_parameters`), and
/// Lottes' is the two-point solve from his GDC 2016 talk.
inline void computeCurveParams(const PostProcessSettings &s, float out[4]) {
    out[0] = out[1] = out[2] = out[3] = 0.0f;

    if (s.tonemap == ToneMapOperator::AgX) {
        constexpr float kMid = 0.18f;
        constexpr float kShoulderMax = 1.0f - kMid;
        // AgX's own white point, which is where our 2.0 ceiling landing on
        // exactly 1.0 comes from — see VIS-3a. Not the same quantity as
        // lottesWhite and deliberately not user-facing.
        constexpr float kWhite = 2.0f;

        const float c = s.agxContrast > 0.05f ? s.agxContrast : 0.05f;
        const float midPowC = std::pow(kMid, c);
        const float toeA = ((1.0f / kMid) - 1.0f) * midPowC;
        const float den = midPowC + toeA;
        const float slope = (c * std::pow(kMid, c - 1.0f) * toeA) / (den * den);
        const float w = ((kWhite - kMid) * (kWhite - kMid) / kShoulderMax) * slope;

        out[0] = c; out[1] = toeA; out[2] = slope; out[3] = w;

    } else if (s.tonemap == ToneMapOperator::Lottes) {
        // midIn == midOut == 0.18 is the pinning that makes middle grey pass
        // through untouched. Lottes' own midOut is 0.267 (an ACES-like lift);
        // using that here would undo the reason this renderer picked AgX over
        // ACES in the first place. See the enum comment.
        constexpr float kMidIn = 0.18f, kMidOut = 0.18f;

        const float a = s.lottesContrast > 0.05f ? s.lottesContrast : 0.05f;
        const float d = s.lottesShoulder > 0.05f ? s.lottesShoulder : 0.05f;
        const float hdr = s.lottesWhite > kMidIn * 2.0f ? s.lottesWhite
                                                        : kMidIn * 2.0f;

        const float midA   = std::pow(kMidIn, a);
        const float midAD  = std::pow(kMidIn, a * d);
        const float hdrA   = std::pow(hdr, a);
        const float hdrAD  = std::pow(hdr, a * d);
        const float denom  = (hdrAD - midAD) * kMidOut;

        if (std::fabs(denom) < 1e-9f) {
            // Degenerate only if white collapses onto middle grey, which the
            // clamp above already prevents. Fall through to a linear-ish
            // identity rather than dividing by zero.
            out[0] = 1.0f; out[1] = 1.0f; out[2] = 0.0f; out[3] = 1.0f;
            return;
        }

        out[0] = a;
        out[1] = a * d;
        out[2] = (-midA + hdrA * kMidOut) / denom;
        out[3] = (hdrAD * midA - hdrA * midAD * kMidOut) / denom;
    }
}

// Tunable bounds live in RenderConfig.h as `PostRange`, matching the
// WaterRange contract: the YAML clamp is authoritative and the debug console
// follows it, so the console can never offer a value the config cannot persist.

/// Which antialiasing resolve to run, if any.
///
/// An enum rather than a bool because the roadmap has a second entry: TAA,
/// once a velocity buffer exists for SSAO and volumetric scattering to share
/// (PLAN.VISUALS.md, "Anti-aliasing"). MSAA is deliberately absent — it was
/// removed from the engine, not merely defaulted off.
enum class AntiAliasMode : int {
    None = 0,
    SMAA = 1,
};

/// SMAA intermediate view, for checking what the passes actually see.
///
/// Worth having rather than inferring from the final image: "SMAA looks like
/// it is doing nothing" has two very different causes — the threshold is too
/// high and no edges are being found, or edges are found and the weights are
/// wrong — and the edge view separates them in one glance.
enum class SmaaDebugView : int {
    Off     = 0,
    Edges   = 1,
    Weights = 2,
};

/// Antialiasing settings.
struct AntiAliasSettings {
    AntiAliasMode mode = AntiAliasMode::None;

    /// Edge-detection threshold. The reference presets span 0.05 (Ultra)
    /// to 0.15 (Low); lower finds more edges, which means more antialiasing
    /// and more work in the weight pass. 0.1 is the reference's own default
    /// and what it calls a good starting point for most scenes.
    float smaaThreshold = 0.1f;

    SmaaDebugView debug = SmaaDebugView::Off;
};

/// Vertex for the fullscreen composite triangle: clip-space position + UV.
struct PostProcessVertex {
    float x, y, z;
    float u, v;

    inline static bgfx::VertexLayout layout;

    static void init() {
        layout.begin()
            .add(bgfx::Attrib::Position,  3, bgfx::AttribType::Float)
            .add(bgfx::Attrib::TexCoord0, 2, bgfx::AttribType::Float)
            .end();
    }
};

/// GPU resources owned by the SMAA passes.
///
/// Lives inside PostProcessResources rather than beside it: SMAA consumes the
/// composite's output, shares its fullscreen triangle, and is meaningless
/// without it, so one owner means one lifetime to get right.
struct SmaaResources {
    // The composite's destination when SMAA is running. LDR, because that is
    // what SMAA is specified against and what the composite emits.
    bgfx::FrameBufferHandle colorFB    = BGFX_INVALID_HANDLE;
    bgfx::TextureHandle     colorTex   = BGFX_INVALID_HANDLE;  // owned by colorFB
    bgfx::FrameBufferHandle edgesFB    = BGFX_INVALID_HANDLE;
    bgfx::TextureHandle     edgesTex   = BGFX_INVALID_HANDLE;  // owned by edgesFB
    bgfx::FrameBufferHandle weightsFB  = BGFX_INVALID_HANDLE;
    bgfx::TextureHandle     weightsTex = BGFX_INVALID_HANDLE;  // owned by weightsFB

    // The two precomputed lookup tables. Data, not tuning: see
    // SMAALookupTextures.h.
    bgfx::TextureHandle areaTex   = BGFX_INVALID_HANDLE;
    bgfx::TextureHandle searchTex = BGFX_INVALID_HANDLE;

    bgfx::ProgramHandle edgeProgram   = BGFX_INVALID_HANDLE;
    bgfx::ProgramHandle weightProgram = BGFX_INVALID_HANDLE;
    bgfx::ProgramHandle blendProgram  = BGFX_INVALID_HANDLE;

    bgfx::UniformHandle s_smaaColor   = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle s_smaaEdges   = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle s_smaaWeights = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle s_smaaArea    = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle s_smaaSearch  = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_smaaMetrics = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_smaaParams  = BGFX_INVALID_HANDLE;

    uint16_t width  = 0;   // output resolution — SMAA runs at display size
    uint16_t height = 0;

    bool valid() const {
        return bgfx::isValid(colorFB) && bgfx::isValid(edgesFB)
            && bgfx::isValid(weightsFB) && bgfx::isValid(areaTex)
            && bgfx::isValid(searchTex) && bgfx::isValid(edgeProgram)
            && bgfx::isValid(weightProgram) && bgfx::isValid(blendProgram);
    }
};

/// GPU resources owned by the post-process pipeline.
struct PostProcessResources {
    bgfx::FrameBufferHandle  frameBuffer = BGFX_INVALID_HANDLE;
    bgfx::TextureHandle      sceneColor  = BGFX_INVALID_HANDLE;  // owned by frameBuffer
    bgfx::ProgramHandle      compositeProgram = BGFX_INVALID_HANDLE;
    bgfx::VertexBufferHandle triangleVBH = BGFX_INVALID_HANDLE;

    bgfx::UniformHandle s_texScene    = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle s_texBloom    = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_ccParams0   = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_ccParams1   = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_ccFilter    = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_lumaWeights = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_bloomParams = BGFX_INVALID_HANDLE;
    // Tone-curve constants + the film-response / halation / grain stages.
    bgfx::UniformHandle u_curveParams = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_filmParams  = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_filmTintLo  = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_filmTintHi  = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_halation    = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_grain       = BGFX_INVALID_HANDLE;

    // Bloom: two ping-pong blur targets at quarter resolution, matching
    // HPL2's `GetTempFrameBuffer(vSize/4, ...)`.
    bgfx::FrameBufferHandle bloomFB[2]  = { BGFX_INVALID_HANDLE, BGFX_INVALID_HANDLE };
    bgfx::TextureHandle     bloomTex[2] = { BGFX_INVALID_HANDLE, BGFX_INVALID_HANDLE };
    bgfx::ProgramHandle     blurProgram    = BGFX_INVALID_HANDLE;
    bgfx::ProgramHandle     extractProgram = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle     s_texBlurSrc    = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle     u_blurStep      = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle     u_extractParams = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle     u_bloomStyle    = BGFX_INVALID_HANDLE;

    // ── Bloom pyramid (VIS-3b) ──
    // Levels BELOW the quarter-res blur output, each half the previous. Index
    // 0 here is pyramid level 1: level 0 is bloomFB[1], which the ping-pong
    // blur already owns, and the up leg accumulates back into it.
    //
    // Built lazily-ish — only when the programs compiled. A missing pyramid
    // is not fatal: the chain degrades to the ping-pong blur alone, which is
    // exactly the pre-VIS-3b behaviour.
    // NOT `= {}`. A bgfx handle is a bare uint16 index and the invalid value
    // is UINT16_MAX, so zero-initialising leaves every entry reading as
    // handle 0 — which bgfx::isValid reports as VALID, and destroyPostProcess
    // would then destroy framebuffer 0 out from under whoever owns it. The
    // static_assert is here because this list has to be written out by hand.
    static_assert(kMaxBloomLevels - 1 == 8,
                  "pyramid handle initialisers below must match "
                  "kMaxBloomLevels - 1");
    bgfx::FrameBufferHandle bloomPyrFB[kMaxBloomLevels - 1] = {
        BGFX_INVALID_HANDLE, BGFX_INVALID_HANDLE, BGFX_INVALID_HANDLE,
        BGFX_INVALID_HANDLE, BGFX_INVALID_HANDLE, BGFX_INVALID_HANDLE,
        BGFX_INVALID_HANDLE, BGFX_INVALID_HANDLE,
    };
    bgfx::TextureHandle bloomPyrTex[kMaxBloomLevels - 1] = {
        BGFX_INVALID_HANDLE, BGFX_INVALID_HANDLE, BGFX_INVALID_HANDLE,
        BGFX_INVALID_HANDLE, BGFX_INVALID_HANDLE, BGFX_INVALID_HANDLE,
        BGFX_INVALID_HANDLE, BGFX_INVALID_HANDLE,
    };
    uint16_t                bloomPyrW[kMaxBloomLevels - 1]   = {};
    uint16_t                bloomPyrH[kMaxBloomLevels - 1]   = {};
    /// How many levels actually got built, counting level 0. 1 means "no
    /// pyramid available", which is a valid state, not a failure.
    int                     bloomPyrLevels = 1;
    bgfx::ProgramHandle     downsampleProgram = BGFX_INVALID_HANDLE;
    bgfx::ProgramHandle     upsampleProgram   = BGFX_INVALID_HANDLE;

    // ── Halation (VIS-13) ──
    // Its own ping-pong pair at quarter res, reusing the bloom extract and
    // blur programs with different parameters. Separate targets rather than
    // borrowing bloom's because both chains are live in the same frame.
    bgfx::FrameBufferHandle haloFB[2]  = { BGFX_INVALID_HANDLE, BGFX_INVALID_HANDLE };
    bgfx::TextureHandle     haloTex[2] = { BGFX_INVALID_HANDLE, BGFX_INVALID_HANDLE };
    bgfx::UniformHandle     s_texHalation = BGFX_INVALID_HANDLE;

    // ── Film grain (final pass) ──
    // grainFB is what the composite (or SMAA) writes into when grain is on,
    // so the grain pass has something to read while it resolves to the
    // backbuffer. The noise field itself is procedural — see fs_grain.sc.
    bgfx::FrameBufferHandle grainFB     = BGFX_INVALID_HANDLE;
    bgfx::TextureHandle     grainTex    = BGFX_INVALID_HANDLE;  // owned by grainFB
    bgfx::ProgramHandle     grainProgram = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle     u_grainRes  = BGFX_INVALID_HANDLE;

    // SMAA's targets, LUTs and programs. Built only when the config asks for
    // it — three full-resolution targets is real memory to spend on a pass
    // that may never be switched on.
    SmaaResources smaa;

    // Scene depth, and a framebuffer over the same colour texture WITHOUT it.
    // Both exist only when the backend gave us a sampleable depth format —
    // see createPostProcess. The corona depth fade is the only consumer.
    bgfx::TextureHandle     sceneDepth = BGFX_INVALID_HANDLE;
    bgfx::FrameBufferHandle colorOnlyFrameBuffer = BGFX_INVALID_HANDLE;
    bool                    depthSampleable = false;

    uint16_t width       = 0;   // scene target size (may be below the window)
    uint16_t height      = 0;
    /// Point-sample the scene texture in the composite. This is the whole of
    /// the "vintage" look: a low-resolution scene upscaled with POINT keeps
    /// hard pixel edges, where LINEAR would just look like a blurry frame.
    bool     pointUpscale = false;
    /// Window size — where the composite lands. Differs from width/height
    /// when render_scale asks for a lower internal resolution.
    uint16_t outWidth    = 0;
    uint16_t outHeight   = 0;
    uint16_t bloomWidth  = 0;
    uint16_t bloomHeight = 0;

    // False when the driver refused a float render target and we fell back to
    // RGBA8. Overbright above 1.0 clips at the target in that case, so tone
    // mapping has nothing to roll off. Recorded so callers can say so rather
    // than silently producing a worse image.
    bool hdrCapable = false;

    bool valid() const { return bgfx::isValid(frameBuffer) &&
                                bgfx::isValid(compositeProgram); }

    /// Bloom needs its own targets and programs on top of the base pass.
    /// The extract program is required too: without it the NewDark style
    /// would silently fall through to an un-thresholded blur, i.e. quietly
    /// render the other style.
    bool bloomValid() const {
        return valid() && bgfx::isValid(blurProgram)
            && bgfx::isValid(extractProgram)
            && bgfx::isValid(bloomFB[0]) && bgfx::isValid(bloomFB[1]);
    }
};

/// Internal render resolution, resolved from the config against the window.
///
/// `requestedHeight` 0 (or >= the window) means native. Otherwise the scene
/// renders smaller and the composite scales it back up — the vintage look.
///
/// `integerScale` divides BOTH axes by the same whole number, which is what
/// makes the pixel grid uniform. Without it a 1280-wide window showing a
/// 380-wide image gives some pixels 3 screen-pixels across and some 4, and
/// the grid visibly breaks up. With it, 1280x720 / 4 = 320x180 — every
/// source pixel is exactly a 4x4 block.
struct RenderResolution {
    uint16_t width  = 0;
    uint16_t height = 0;
    int      divisor = 1;   // 1 = native
    bool     isNative() const { return divisor == 1; }
};

inline RenderResolution resolveRenderResolution(uint16_t windowW,
                                                uint16_t windowH,
                                                int requestedHeight,
                                                bool integerScale) {
    RenderResolution r;
    r.width = windowW; r.height = windowH; r.divisor = 1;
    if (requestedHeight <= 0 || requestedHeight >= int(windowH))
        return r;

    if (integerScale) {
        // Round to the nearest whole divisor rather than truncating, so
        // asking for 240 out of 720 gives exactly 3x and not 2x.
        int div = (int(windowH) + requestedHeight / 2) / requestedHeight;
        if (div < 1) div = 1;
        r.divisor = div;
        r.width  = static_cast<uint16_t>(windowW / div);
        r.height = static_cast<uint16_t>(windowH / div);
    } else {
        // Preserve the window's aspect so the composite's stretch does not
        // distort. The projection uses the WINDOW aspect regardless, so a
        // non-square internal pixel resolves correctly.
        r.height = static_cast<uint16_t>(requestedHeight);
        r.width  = static_cast<uint16_t>(
            (int(windowW) * requestedHeight + int(windowH) / 2) / int(windowH));
        r.divisor = 0; // non-integer
    }
    if (r.width  == 0) r.width  = 1;
    if (r.height == 0) r.height = 1;
    return r;
}

/// Pick the scene colour format, announcing a downgrade rather than taking it
/// silently — an RGBA8 scene target defeats the entire point of the pass.
inline bgfx::TextureFormat::Enum pickSceneColorFormat(bool &outHdrCapable) {
    if (bgfx::isTextureValid(0, false, 1, bgfx::TextureFormat::RGBA16F,
                             BGFX_TEXTURE_RT)) {
        outHdrCapable = true;
        return bgfx::TextureFormat::RGBA16F;
    }
    std::fprintf(stderr,
        "[FALLBACK] postprocess: RGBA16F render target is unsupported on this "
        "backend — falling back to RGBA8. Lighting above 1.0 will clip at the "
        "scene target instead of at tone mapping, so highlight rolloff will be "
        "wrong and bloom would have nothing above threshold to gather.\n");
    outHdrCapable = false;
    return bgfx::TextureFormat::RGBA8;
}

/// Pick a supported depth format. Formats differ in availability across
/// Metal/D3D/GL/Vulkan, so probe rather than assume.
///
/// `sampleFlags` is what the depth attachment will actually be created with.
/// A write-only depth buffer is cheaper and universally supported; a
/// *sampleable* one is what the corona depth fade needs, and not every
/// backend offers every format for reading. Probing with the real flags is
/// the only honest test — `isTextureValid` with different flags answers a
/// different question.
inline bool pickDepthFormat(uint64_t sampleFlags,
                            bgfx::TextureFormat::Enum &outFormat,
                            const char *&outName) {
    struct Candidate { bgfx::TextureFormat::Enum fmt; const char *name; };
    // D32F first when sampling: a plain 32-bit float depth is the most widely
    // readable. D24S8 leads for the write-only case because it is the
    // cheapest universally-supported render-target depth.
    const Candidate sampled[] = {
        { bgfx::TextureFormat::D32F,  "D32F"  },
        { bgfx::TextureFormat::D24S8, "D24S8" },
        { bgfx::TextureFormat::D24,   "D24"   },
        { bgfx::TextureFormat::D16,   "D16"   },
    };
    const Candidate writeOnly[] = {
        { bgfx::TextureFormat::D24S8, "D24S8" },
        { bgfx::TextureFormat::D32F,  "D32F"  },
        { bgfx::TextureFormat::D24,   "D24"   },
        { bgfx::TextureFormat::D16,   "D16"   },
    };
    const bool wantSampling = (sampleFlags & BGFX_TEXTURE_RT_WRITE_ONLY) == 0;
    const Candidate *list = wantSampling ? sampled : writeOnly;
    for (int i = 0; i < 4; ++i) {
        const Candidate &c = list[i];
        if (bgfx::isTextureValid(0, false, 1, c.fmt, sampleFlags)) {
            outFormat = c.fmt;
            outName   = c.name;
            return true;
        }
    }
    return false;
}

/// Build the fullscreen triangle used by the composite pass.
///
/// A single oversized triangle rather than a two-triangle quad: no diagonal
/// seam, one fewer vertex, and no risk of double-shading along the shared
/// edge. Clip-space coordinates are written directly, so the vertex shader
/// applies no transform.
///
/// V orientation depends on the backend. With `originBottomLeft` (OpenGL)
/// UV (0,0) is the bottom-left of the sampled target; on Metal/D3D/Vulkan it
/// is the top-left. Getting this wrong renders the scene vertically mirrored,
/// which is exactly the kind of bug that looks like a broken camera.
inline bgfx::VertexBufferHandle buildFullscreenTriangle() {
    const bool originBottomLeft = bgfx::getCaps()->originBottomLeft;

    // Clip-space verts: (-1,-1), (3,-1), (-1,3) cover the whole [-1,1] square.
    //   u = (clipX + 1) / 2
    //   v = (clipY + 1) / 2   with originBottomLeft
    //   v = (1 - clipY) / 2   otherwise
    PostProcessVertex verts[3];
    verts[0] = { -1.0f, -1.0f, 0.0f, 0.0f, originBottomLeft ?  0.0f :  1.0f };
    verts[1] = {  3.0f, -1.0f, 0.0f, 2.0f, originBottomLeft ?  0.0f :  1.0f };
    verts[2] = { -1.0f,  3.0f, 0.0f, 0.0f, originBottomLeft ?  2.0f : -1.0f };

    return bgfx::createVertexBuffer(
        bgfx::copy(verts, sizeof(verts)), PostProcessVertex::layout);
}

/// Create the offscreen target, composite program and fullscreen triangle.
/// Returns false (leaving `out` invalid) if the target could not be built;
/// callers should fall back to the direct path and say so.
/// `width`/`height` size the SCENE target; `outWidth`/`outHeight` size the
/// composite's viewport, i.e. the window. They differ when
/// graphics.render_scale asks for a lower internal resolution — the composite
/// is where the upscale happens.
inline bool createPostProcess(PostProcessResources &out,
                              uint16_t width, uint16_t height,
                              uint16_t outWidth, uint16_t outHeight,
                              bool pointUpscale,
                              bgfx::ProgramHandle compositeProgram,
                              bgfx::ProgramHandle blurProgram,
                              bgfx::ProgramHandle extractProgram,
                              bgfx::ProgramHandle downsampleProgram,
                              bgfx::ProgramHandle upsampleProgram,
                              bgfx::ProgramHandle grainProgram) {
    out.width  = width;
    out.height = height;
    out.outWidth  = outWidth;
    out.outHeight = outHeight;
    out.pointUpscale = pointUpscale;
    out.compositeProgram = compositeProgram;
    out.blurProgram      = blurProgram;
    out.extractProgram   = extractProgram;
    out.downsampleProgram = downsampleProgram;
    out.upsampleProgram   = upsampleProgram;

    if (!bgfx::isValid(compositeProgram)) {
        std::fprintf(stderr,
            "[FALLBACK] postprocess: composite program failed to build — "
            "post-processing disabled, rendering direct to backbuffer.\n");
        return false;
    }

    const bgfx::TextureFormat::Enum colorFmt = pickSceneColorFormat(out.hdrCapable);

    // LINEAR filtering, not point.
    //
    // The composite itself is a 1:1 resolve at exact texel centres, where
    // bilinear and point return identical results — so this costs the
    // pass-through nothing. But bloom's first blur pass reads this same
    // texture while rendering at quarter resolution, and there point
    // sampling would drop three of every four texels: bright thin geometry
    // (a lamp filament, a candle) would flicker in and out of the bloom as
    // the camera moved. Linear gives that downsample its 2x2 average.
    //
    // Clamp so the oversized triangle's out-of-range UVs (which land
    // outside the visible region) cannot wrap.
    //
    const uint64_t rtFlag = BGFX_TEXTURE_RT;

    const uint64_t colorFlags = rtFlag
                              | BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP;

    // ── Depth: sampleable if the backend allows it ──
    //
    // The corona pass reads this to fade its billboards behind geometry, so a
    // readable depth buffer is worth asking for. It is not worth *insisting*
    // on: a write-only depth buffer is cheaper and universally supported, and
    // everything except the corona depth fade works fine with one. So probe
    // for sampleable first and fall back rather than failing the whole pass.
    //
    // Depth is created SAMPLEABLE, because the corona depth fade reads it and
    // SSAO / volumetric scattering on the roadmap will too. That is only
    // possible because this engine has no MSAA: bgfx refuses to resolve a
    // multisampled depth attachment on any backend —
    //
    //   "Frame buffer depth MSAA texture cannot be resolved. It must be
    //    created with `BGFX_TEXTURE_RT_WRITE_ONLY` flag."   (bgfx.cpp:4561)
    //
    // — and that incompatibility is one of the two reasons MSAA was removed
    // (the other being the deferred G-buffer in PLAN.DYNAMIC_LIGHTS.md). See
    // PLAN.VISUALS.md "Anti-aliasing"; SMAA/TAA replace it.
    //
    // Still probe rather than assume: a write-only depth buffer is cheaper and
    // universally supported, and everything except the depth-reading effects
    // works fine with one, so degrade instead of failing the whole pass.
    bgfx::TextureFormat::Enum depthFmt = bgfx::TextureFormat::D24S8;
    const char *depthName = "unknown";
    const uint64_t sampleableDepthFlags =
        rtFlag | BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP;
    uint64_t depthFlags = sampleableDepthFlags;

    if (pickDepthFormat(sampleableDepthFlags, depthFmt, depthName)) {
        out.depthSampleable = true;
    } else if (pickDepthFormat(rtFlag | BGFX_TEXTURE_RT_WRITE_ONLY,
                               depthFmt, depthName)) {
        out.depthSampleable = false;
        depthFlags = rtFlag | BGFX_TEXTURE_RT_WRITE_ONLY;
        std::fprintf(stderr,
            "[FALLBACK] postprocess: no sampleable depth format on this "
            "backend — using a write-only depth buffer. The corona depth fade "
            "has nothing to read, so it is off and coronas fall back to "
            "ray-traced occlusion alone.\n");
    } else {
        std::fprintf(stderr,
            "[FALLBACK] postprocess: no supported depth render-target format — "
            "post-processing disabled, rendering direct to backbuffer.\n");
        return false;
    }

    bgfx::TextureHandle attachments[2];
    attachments[0] = bgfx::createTexture2D(width, height, false, 1, colorFmt, colorFlags);
    attachments[1] = bgfx::createTexture2D(width, height, false, 1, depthFmt,
                                           depthFlags);

    if (!bgfx::isValid(attachments[0]) || !bgfx::isValid(attachments[1])) {
        std::fprintf(stderr,
            "[FALLBACK] postprocess: render-target texture creation failed — "
            "post-processing disabled, rendering direct to backbuffer.\n");
        if (bgfx::isValid(attachments[0])) bgfx::destroy(attachments[0]);
        if (bgfx::isValid(attachments[1])) bgfx::destroy(attachments[1]);
        return false;
    }

    // destroyTextures=true: the framebuffer owns both attachments from here.
    // That transfer only happens on success, so a failure leaves the two
    // textures ours to release.
    out.frameBuffer = bgfx::createFrameBuffer(2, attachments, true);
    if (!bgfx::isValid(out.frameBuffer)) {
        std::fprintf(stderr,
            "[FALLBACK] postprocess: framebuffer creation failed — "
            "post-processing disabled, rendering direct to backbuffer.\n");
        bgfx::destroy(attachments[0]);
        bgfx::destroy(attachments[1]);
        return false;
    }
    out.sceneColor = bgfx::getTexture(out.frameBuffer, 0);
    if (out.depthSampleable)
        out.sceneDepth = bgfx::getTexture(out.frameBuffer, 1);

    // ── Colour-only view of the same scene target ──
    //
    // Shares attachment 0 with the main framebuffer and deliberately has NO
    // depth attachment. The corona pass renders through this so it can sample
    // the depth texture: binding a texture as an attachment and as a sampler
    // in the same draw is a read/write hazard, and undefined behaviour on
    // every backend that bothers to say. `destroyTextures=false` — the main
    // framebuffer owns that colour texture.
    if (out.depthSampleable) {
        bgfx::Attachment colorOnly[1];
        colorOnly[0].init(out.sceneColor);
        out.colorOnlyFrameBuffer = bgfx::createFrameBuffer(1, colorOnly, false);
        if (!bgfx::isValid(out.colorOnlyFrameBuffer)) {
            // Not fatal: coronas fall back to ray-traced occlusion, which is
            // what they used before the depth fade existed.
            std::fprintf(stderr,
                "[FALLBACK] postprocess: colour-only framebuffer creation "
                "failed — the corona depth fade cannot run and coronas will "
                "fall back to ray-traced occlusion alone.\n");
            out.depthSampleable = false;
            out.sceneDepth = BGFX_INVALID_HANDLE;
        }
    }

    PostProcessVertex::init();
    out.triangleVBH = buildFullscreenTriangle();

    out.s_texScene    = bgfx::createUniform("s_texScene",    bgfx::UniformType::Sampler);
    out.s_texBloom    = bgfx::createUniform("s_texBloom",    bgfx::UniformType::Sampler);
    out.u_ccParams0   = bgfx::createUniform("u_ccParams0",   bgfx::UniformType::Vec4);
    out.u_ccParams1   = bgfx::createUniform("u_ccParams1",   bgfx::UniformType::Vec4);
    out.u_ccFilter    = bgfx::createUniform("u_ccFilter",    bgfx::UniformType::Vec4);
    out.u_lumaWeights = bgfx::createUniform("u_lumaWeights", bgfx::UniformType::Vec4);
    out.u_bloomParams = bgfx::createUniform("u_bloomParams", bgfx::UniformType::Vec4);
    out.u_bloomStyle  = bgfx::createUniform("u_bloomStyle",  bgfx::UniformType::Vec4);
    out.s_texBlurSrc  = bgfx::createUniform("s_texBlurSrc",  bgfx::UniformType::Sampler);
    out.u_blurStep    = bgfx::createUniform("u_blurStep",    bgfx::UniformType::Vec4);
    out.u_extractParams =
        bgfx::createUniform("u_extractParams", bgfx::UniformType::Vec4);
    out.u_curveParams = bgfx::createUniform("u_curveParams", bgfx::UniformType::Vec4);
    out.u_filmParams  = bgfx::createUniform("u_filmParams",  bgfx::UniformType::Vec4);
    out.u_filmTintLo  = bgfx::createUniform("u_filmTintLo",  bgfx::UniformType::Vec4);
    out.u_filmTintHi  = bgfx::createUniform("u_filmTintHi",  bgfx::UniformType::Vec4);
    out.u_halation    = bgfx::createUniform("u_halation",    bgfx::UniformType::Vec4);
    out.u_grain       = bgfx::createUniform("u_grain",       bgfx::UniformType::Vec4);
    out.s_texHalation =
        bgfx::createUniform("s_texHalation", bgfx::UniformType::Sampler);
    out.u_grainRes  = bgfx::createUniform("u_grainRes",  bgfx::UniformType::Vec4);

    // ── Bloom targets ──
    // Quarter resolution, matching HPL2's `GetTempFrameBuffer(vSize/4, ...)`.
    // Same colour format as the scene target so overbright survives the blur;
    // HPL2 used 8-bit here because its whole chain was 8-bit.
    //
    // Failure to build bloom is NOT fatal to the base pass: tone mapping and
    // colour correction still work, so degrade to those rather than dropping
    // the user all the way back to the direct path.
    out.bloomWidth  = static_cast<uint16_t>(width  / 4);
    out.bloomHeight = static_cast<uint16_t>(height / 4);
    if (out.bloomWidth < 1)  out.bloomWidth  = 1;
    if (out.bloomHeight < 1) out.bloomHeight = 1;

    if (!bgfx::isValid(blurProgram) || !bgfx::isValid(extractProgram)) {
        std::fprintf(stderr,
            "[FALLBACK] postprocess: bloom %s program failed to build — "
            "bloom unavailable; tone mapping and colour correction still "
            "active.\n",
            bgfx::isValid(blurProgram) ? "extract" : "blur");
    } else {
        // Linear + clamp: the 5-tap kernel places samples between texels and
        // relies on hardware bilinear to weight each pair (see
        // fs_bloom_blur.sc), so point sampling here would silently collapse
        // the Gaussian into a box.
        const uint64_t bloomFlags = BGFX_TEXTURE_RT
                                  | BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP;
        bool ok = true;
        for (int i = 0; i < 2; ++i) {
            out.bloomFB[i] = bgfx::createFrameBuffer(
                out.bloomWidth, out.bloomHeight, colorFmt, bloomFlags);
            if (!bgfx::isValid(out.bloomFB[i])) {
                ok = false;
                break;
            }
            out.bloomTex[i] = bgfx::getTexture(out.bloomFB[i], 0);
        }
        if (!ok) {
            std::fprintf(stderr,
                "[FALLBACK] postprocess: bloom framebuffer creation failed — "
                "bloom unavailable; tone mapping and colour correction still "
                "active.\n");
            for (int i = 0; i < 2; ++i) {
                if (bgfx::isValid(out.bloomFB[i]))
                    bgfx::destroy(out.bloomFB[i]);
                out.bloomFB[i]  = BGFX_INVALID_HANDLE;
                out.bloomTex[i] = BGFX_INVALID_HANDLE;
            }
        }

        // ── Bloom pyramid levels (VIS-3b) ──
        //
        // Level 0 is bloomFB[1] above; these are the halvings below it. They
        // are built unconditionally rather than on demand because
        // bloom.range is live-editable from the console, and allocating
        // render targets mid-frame in response to a slider is how you get a
        // stutter. The whole chain below quarter res is 1/3 of one
        // quarter-res target in total (1/4 + 1/16 + ... ), so the memory is
        // not worth deferring.
        if (ok && bgfx::isValid(downsampleProgram)
               && bgfx::isValid(upsampleProgram)) {
            uint16_t w = out.bloomWidth;
            uint16_t h = out.bloomHeight;
            int levels = 1;   // counting level 0

            for (int i = 0; i < kMaxBloomLevels - 1; ++i) {
                w = static_cast<uint16_t>(w / 2);
                h = static_cast<uint16_t>(h / 2);
                // Stop before a degenerate level. A 1-texel axis makes the
                // tent's neighbour taps land on the clamped edge, which is a
                // uniform colour smear rather than a blur.
                if (w < 2 || h < 2)
                    break;

                out.bloomPyrFB[i] =
                    bgfx::createFrameBuffer(w, h, colorFmt, bloomFlags);
                if (!bgfx::isValid(out.bloomPyrFB[i]))
                    break;
                out.bloomPyrTex[i] = bgfx::getTexture(out.bloomPyrFB[i], 0);
                out.bloomPyrW[i]   = w;
                out.bloomPyrH[i]   = h;
                ++levels;
            }
            out.bloomPyrLevels = levels;
        }

        // ── Halation targets ──
        // Same quarter resolution as bloom's, and the same linear+clamp
        // sampling requirement: the blur kernel places taps between texels.
        if (ok) {
            for (int i = 0; i < 2; ++i) {
                out.haloFB[i] = bgfx::createFrameBuffer(
                    out.bloomWidth, out.bloomHeight, colorFmt, bloomFlags);
                if (!bgfx::isValid(out.haloFB[i])) {
                    std::fprintf(stderr,
                        "[FALLBACK] postprocess: halation framebuffer creation "
                        "failed — halation unavailable; bloom and the rest of "
                        "the chain still active.\n");
                    for (int k = 0; k < 2; ++k) {
                        if (bgfx::isValid(out.haloFB[k]))
                            bgfx::destroy(out.haloFB[k]);
                        out.haloFB[k]  = BGFX_INVALID_HANDLE;
                        out.haloTex[k] = BGFX_INVALID_HANDLE;
                    }
                    break;
                }
                out.haloTex[i] = bgfx::getTexture(out.haloFB[i], 0);
            }
        }

        // ── Grain: final-pass target ──
        // No noise texture: the field is generated per pixel (see
        // fs_grain.sc). A stored template is periodic, and at any tile size
        // smaller than the screen that period is visible — measured
        // autocorrelation 1.0000 at one tile period.
        if (bgfx::isValid(grainProgram)) {
            out.grainProgram = grainProgram;
            const uint64_t grainFlags = BGFX_TEXTURE_RT
                                      | BGFX_SAMPLER_U_CLAMP
                                      | BGFX_SAMPLER_V_CLAMP;
            out.grainFB = bgfx::createFrameBuffer(outWidth, outHeight,
                                                  bgfx::TextureFormat::RGBA8,
                                                  grainFlags);
            if (bgfx::isValid(out.grainFB)) {
                out.grainTex = bgfx::getTexture(out.grainFB, 0);
                std::fprintf(stderr,
                    "Grain: procedural correlated field, 3 independent "
                    "channels, final pass after antialiasing\n");
            } else {
                std::fprintf(stderr,
                    "[FALLBACK] postprocess: grain target creation failed — "
                    "film grain unavailable; the rest of the chain still "
                    "runs.\n");
            }
        } else {
            std::fprintf(stderr,
                "[FALLBACK] postprocess: grain program failed to build — "
                "film grain unavailable.\n");
        }

        // Say what actually got built, once. "The pyramid is not widening my
        // bloom" and "the pyramid was never created" look identical from
        // outside, which is the §4 lesson applied ahead of time.
        std::fprintf(stderr,
            "Bloom: %ux%u base, pyramid %d level(s)%s\n",
            out.bloomWidth, out.bloomHeight, out.bloomPyrLevels,
            out.bloomPyrLevels > 1
                ? " — bloom_range can widen the glow"
                : " — no pyramid, bloom_range has nothing to widen with");
    }

    // Composite targets the backbuffer and never clears — it overwrites every
    // pixel it covers, and the triangle covers all of them.
    // Name the pyramid views so a capture shows which leg a pass belongs to.
    for (int i = 0; i < kMaxBloomLevels - 1; ++i) {
        char name[32];
        std::snprintf(name, sizeof(name), "bloom-down-%d", i + 1);
        bgfx::setViewName(static_cast<bgfx::ViewId>(kViewBloomPyrDown + i), name);
        std::snprintf(name, sizeof(name), "bloom-up-%d", i + 1);
        bgfx::setViewName(static_cast<bgfx::ViewId>(kViewBloomPyrUp + i), name);
    }

    bgfx::setViewName(kViewHalationExtract, "halation-extract");
    for (int i = 0; i < kMaxHalationIterations; ++i) {
        char hname[32];
        std::snprintf(hname, sizeof(hname), "halation-blur-%d", i);
        bgfx::setViewName(
            static_cast<bgfx::ViewId>(kViewHalationBase + i * 2), hname);
        std::snprintf(hname, sizeof(hname), "halation-blur-%dv", i);
        bgfx::setViewName(
            static_cast<bgfx::ViewId>(kViewHalationBase + i * 2 + 1), hname);
    }

    bgfx::setViewName(kViewComposite, "composite");
    bgfx::setViewName(kViewGrain, "film-grain");
    bgfx::setViewClear(kViewComposite, BGFX_CLEAR_NONE, 0, 1.0f, 0);
    bgfx::setViewRect(kViewComposite, 0, 0, outWidth, outHeight);
    bgfx::setViewFrameBuffer(kViewComposite, BGFX_INVALID_HANDLE);

    std::fprintf(stderr,
        "Post-process: %ux%u scene target%s, colour=%s depth=%s, "
        "backend=%s\n",
        width, height,
        (width == outWidth && height == outHeight)
            ? "" : (pointUpscale ? " -> point upscale (vintage)"
                                 : " -> linear upscale"),
        out.hdrCapable ? "RGBA16F" : "RGBA8 (no HDR)",
        depthName,
        bgfx::getRendererName(bgfx::getRendererType()));

    return true;
}

/// Upload one of the SMAA lookup tables, widening it if the backend will not
/// take the narrow format.
///
/// R8 and RG8 are universally supported in practice, but "in practice" is not
/// a thing to find out from a black screen: probe, and if the format is
/// missing, expand to RGBA8 rather than failing. The shaders select .r / .rg,
/// so a widened table reads identically — this changes memory, not results.
inline bgfx::TextureHandle createSmaaLookupTexture(
        const char *name, const uint8_t *data, uint16_t width, uint16_t height,
        uint8_t channels, uint64_t samplerFlags) {
    const bgfx::TextureFormat::Enum narrow =
        (channels == 1) ? bgfx::TextureFormat::R8 : bgfx::TextureFormat::RG8;

    if (bgfx::isTextureValid(0, false, 1, narrow, samplerFlags)) {
        return bgfx::createTexture2D(
            width, height, false, 1, narrow, samplerFlags,
            bgfx::copy(data, uint32_t(width) * height * channels));
    }

    std::fprintf(stderr,
        "[FALLBACK] smaa: this backend has no %s texture format for the %s "
        "lookup table — widening it to RGBA8. Same values, 2-4x the memory.\n",
        (channels == 1) ? "R8" : "RG8", name);

    std::vector<uint8_t> widened(size_t(width) * height * 4, 0);
    for (size_t i = 0, n = size_t(width) * height; i < n; ++i) {
        widened[i * 4 + 0] = data[i * channels];
        if (channels > 1)
            widened[i * 4 + 1] = data[i * channels + 1];
        widened[i * 4 + 3] = 255;
    }
    return bgfx::createTexture2D(
        width, height, false, 1, bgfx::TextureFormat::RGBA8, samplerFlags,
        bgfx::copy(widened.data(), uint32_t(widened.size())));
}

/// Build SMAA's targets, lookup tables and uniforms.
///
/// `width`/`height` are the OUTPUT (window) resolution: SMAA is a fixed-cost
/// screen-space pass and runs where the pixels actually are, which is also why
/// it is the antialiasing this engine can afford — see PLAN.VISUALS.md.
///
/// Returns false having said why; the caller keeps the composite path.
inline bool createSmaa(SmaaResources &out, uint16_t width, uint16_t height,
                       bgfx::ProgramHandle edgeProgram,
                       bgfx::ProgramHandle weightProgram,
                       bgfx::ProgramHandle blendProgram) {
    out.width  = width;
    out.height = height;
    out.edgeProgram   = edgeProgram;
    out.weightProgram = weightProgram;
    out.blendProgram  = blendProgram;

    if (!bgfx::isValid(edgeProgram) || !bgfx::isValid(weightProgram)
            || !bgfx::isValid(blendProgram)) {
        std::fprintf(stderr,
            "[FALLBACK] smaa: one of the three pass programs failed to "
            "build — antialiasing unavailable, compositing straight to the "
            "backbuffer.\n");
        return false;
    }

    // LINEAR everywhere except SearchTex, and that is not a preference.
    //
    // SMAA's searches read a sample placed BETWEEN two texels and rely on the
    // filter to average the pair — that is how a 32-step search covers 64
    // pixels, and how the blend pass lands its two taps at fractional offsets.
    // Point sampling would not soften the result, it would return distances
    // that mean something else entirely.
    const uint64_t rtFlags = BGFX_TEXTURE_RT
                           | BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP;

    out.colorFB = bgfx::createFrameBuffer(width, height,
                                          bgfx::TextureFormat::RGBA8, rtFlags);

    // Edges are two channels; ask for RG8 and widen if the backend refuses,
    // for the same reason as the lookup tables.
    bgfx::TextureFormat::Enum edgeFmt = bgfx::TextureFormat::RG8;
    if (!bgfx::isTextureValid(0, false, 1, edgeFmt, rtFlags)) {
        std::fprintf(stderr,
            "[FALLBACK] smaa: no RG8 render target on this backend — using "
            "RGBA8 for the edge buffer. Same result, more bandwidth.\n");
        edgeFmt = bgfx::TextureFormat::RGBA8;
    }
    out.edgesFB   = bgfx::createFrameBuffer(width, height, edgeFmt, rtFlags);
    out.weightsFB = bgfx::createFrameBuffer(width, height,
                                            bgfx::TextureFormat::RGBA8, rtFlags);

    if (!bgfx::isValid(out.colorFB) || !bgfx::isValid(out.edgesFB)
            || !bgfx::isValid(out.weightsFB)) {
        std::fprintf(stderr,
            "[FALLBACK] smaa: render-target creation failed — antialiasing "
            "unavailable, compositing straight to the backbuffer.\n");
        return false;
    }
    out.colorTex   = bgfx::getTexture(out.colorFB,   0);
    out.edgesTex   = bgfx::getTexture(out.edgesFB,   0);
    out.weightsTex = bgfx::getTexture(out.weightsFB, 0);

    out.areaTex = createSmaaLookupTexture(
        "AreaTex", kSmaaAreaTex, kSmaaAreaTexWidth, kSmaaAreaTexHeight, 2,
        BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP);

    // POINT for SearchTex, and this one is equally load-bearing the other
    // way: it packs discrete run-length codes, so an interpolated value
    // between two of them is a distance that was never in the table.
    out.searchTex = createSmaaLookupTexture(
        "SearchTex", kSmaaSearchTex, kSmaaSearchTexWidth, kSmaaSearchTexHeight,
        1,
        BGFX_SAMPLER_MIN_POINT | BGFX_SAMPLER_MAG_POINT | BGFX_SAMPLER_MIP_POINT
            | BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP);

    if (!bgfx::isValid(out.areaTex) || !bgfx::isValid(out.searchTex)) {
        std::fprintf(stderr,
            "[FALLBACK] smaa: lookup-table upload failed — antialiasing "
            "unavailable, compositing straight to the backbuffer.\n");
        return false;
    }

    out.s_smaaColor   = bgfx::createUniform("s_smaaColor",   bgfx::UniformType::Sampler);
    out.s_smaaEdges   = bgfx::createUniform("s_smaaEdges",   bgfx::UniformType::Sampler);
    out.s_smaaWeights = bgfx::createUniform("s_smaaWeights", bgfx::UniformType::Sampler);
    out.s_smaaArea    = bgfx::createUniform("s_smaaArea",    bgfx::UniformType::Sampler);
    out.s_smaaSearch  = bgfx::createUniform("s_smaaSearch",  bgfx::UniformType::Sampler);
    out.u_smaaMetrics = bgfx::createUniform("u_smaaMetrics", bgfx::UniformType::Vec4);
    out.u_smaaParams  = bgfx::createUniform("u_smaaParams",  bgfx::UniformType::Vec4);

    bgfx::setViewName(kViewSmaaEdges,   "smaa-edges");
    bgfx::setViewName(kViewSmaaWeights, "smaa-weights");
    bgfx::setViewName(kViewSmaaBlend,   "smaa-blend");

    std::fprintf(stderr, "SMAA: %ux%u, edges=%s, threshold is live in the "
                         "console as smaa_threshold\n",
                 width, height,
                 edgeFmt == bgfx::TextureFormat::RG8 ? "RG8" : "RGBA8");
    return true;
}

inline void destroySmaa(SmaaResources &smaa) {
    // Textures fetched with getTexture() are owned by their framebuffers
    // (created with destroyTextures defaulting to true), so they must not be
    // destroyed separately — only the LUTs are ours to free.
    if (bgfx::isValid(smaa.colorFB))       bgfx::destroy(smaa.colorFB);
    if (bgfx::isValid(smaa.edgesFB))       bgfx::destroy(smaa.edgesFB);
    if (bgfx::isValid(smaa.weightsFB))     bgfx::destroy(smaa.weightsFB);
    if (bgfx::isValid(smaa.areaTex))       bgfx::destroy(smaa.areaTex);
    if (bgfx::isValid(smaa.searchTex))     bgfx::destroy(smaa.searchTex);
    if (bgfx::isValid(smaa.edgeProgram))   bgfx::destroy(smaa.edgeProgram);
    if (bgfx::isValid(smaa.weightProgram)) bgfx::destroy(smaa.weightProgram);
    if (bgfx::isValid(smaa.blendProgram))  bgfx::destroy(smaa.blendProgram);
    if (bgfx::isValid(smaa.s_smaaColor))   bgfx::destroy(smaa.s_smaaColor);
    if (bgfx::isValid(smaa.s_smaaEdges))   bgfx::destroy(smaa.s_smaaEdges);
    if (bgfx::isValid(smaa.s_smaaWeights)) bgfx::destroy(smaa.s_smaaWeights);
    if (bgfx::isValid(smaa.s_smaaArea))    bgfx::destroy(smaa.s_smaaArea);
    if (bgfx::isValid(smaa.s_smaaSearch))  bgfx::destroy(smaa.s_smaaSearch);
    if (bgfx::isValid(smaa.u_smaaMetrics)) bgfx::destroy(smaa.u_smaaMetrics);
    if (bgfx::isValid(smaa.u_smaaParams))  bgfx::destroy(smaa.u_smaaParams);
    smaa = SmaaResources{};
}

inline void destroyPostProcess(PostProcessResources &pp) {
    // sceneColor / bloomTex are owned by their framebuffers (created with
    // destroyTextures=true), so they must not be destroyed separately.
    // Destroyed BEFORE frameBuffer: it borrows that framebuffer's colour
    // texture (created with destroyTextures=false), so it must go while the
    // owner is still alive.
    if (bgfx::isValid(pp.colorOnlyFrameBuffer))
        bgfx::destroy(pp.colorOnlyFrameBuffer);
    if (bgfx::isValid(pp.frameBuffer))      bgfx::destroy(pp.frameBuffer);
    if (bgfx::isValid(pp.bloomFB[0]))       bgfx::destroy(pp.bloomFB[0]);
    if (bgfx::isValid(pp.bloomFB[1]))       bgfx::destroy(pp.bloomFB[1]);
    for (int i = 0; i < kMaxBloomLevels - 1; ++i) {
        // Textures came from getTexture() and are owned by their framebuffer.
        if (bgfx::isValid(pp.bloomPyrFB[i])) bgfx::destroy(pp.bloomPyrFB[i]);
    }
    if (bgfx::isValid(pp.triangleVBH))      bgfx::destroy(pp.triangleVBH);
    if (bgfx::isValid(pp.compositeProgram)) bgfx::destroy(pp.compositeProgram);
    if (bgfx::isValid(pp.blurProgram))      bgfx::destroy(pp.blurProgram);
    if (bgfx::isValid(pp.extractProgram))   bgfx::destroy(pp.extractProgram);
    if (bgfx::isValid(pp.downsampleProgram)) bgfx::destroy(pp.downsampleProgram);
    if (bgfx::isValid(pp.upsampleProgram))   bgfx::destroy(pp.upsampleProgram);
    if (bgfx::isValid(pp.u_bloomStyle))     bgfx::destroy(pp.u_bloomStyle);
    if (bgfx::isValid(pp.u_extractParams))  bgfx::destroy(pp.u_extractParams);
    if (bgfx::isValid(pp.s_texScene))       bgfx::destroy(pp.s_texScene);
    if (bgfx::isValid(pp.s_texBloom))       bgfx::destroy(pp.s_texBloom);
    if (bgfx::isValid(pp.u_ccParams0))      bgfx::destroy(pp.u_ccParams0);
    if (bgfx::isValid(pp.u_ccParams1))      bgfx::destroy(pp.u_ccParams1);
    if (bgfx::isValid(pp.u_ccFilter))       bgfx::destroy(pp.u_ccFilter);
    if (bgfx::isValid(pp.u_lumaWeights))    bgfx::destroy(pp.u_lumaWeights);
    if (bgfx::isValid(pp.u_bloomParams))    bgfx::destroy(pp.u_bloomParams);
    if (bgfx::isValid(pp.s_texBlurSrc))     bgfx::destroy(pp.s_texBlurSrc);
    if (bgfx::isValid(pp.u_blurStep))       bgfx::destroy(pp.u_blurStep);
    if (bgfx::isValid(pp.u_curveParams))    bgfx::destroy(pp.u_curveParams);
    if (bgfx::isValid(pp.u_filmParams))     bgfx::destroy(pp.u_filmParams);
    if (bgfx::isValid(pp.u_filmTintLo))     bgfx::destroy(pp.u_filmTintLo);
    if (bgfx::isValid(pp.u_filmTintHi))     bgfx::destroy(pp.u_filmTintHi);
    if (bgfx::isValid(pp.u_halation))       bgfx::destroy(pp.u_halation);
    if (bgfx::isValid(pp.u_grain))          bgfx::destroy(pp.u_grain);
    if (bgfx::isValid(pp.haloFB[0]))        bgfx::destroy(pp.haloFB[0]);
    if (bgfx::isValid(pp.haloFB[1]))        bgfx::destroy(pp.haloFB[1]);
    if (bgfx::isValid(pp.s_texHalation))    bgfx::destroy(pp.s_texHalation);
    if (bgfx::isValid(pp.grainFB))          bgfx::destroy(pp.grainFB);
    if (bgfx::isValid(pp.grainProgram))     bgfx::destroy(pp.grainProgram);
    if (bgfx::isValid(pp.u_grainRes))       bgfx::destroy(pp.u_grainRes);
    destroySmaa(pp.smaa);
    pp = PostProcessResources{};
}

/// Whether the final grain pass runs this frame.
///
/// Grain needs its own target to resolve from, so unlike the other post
/// stages it cannot simply no-op inside an existing pass — the chain has to
/// be routed differently. Callers use this to decide where the composite and
/// SMAA write.
inline bool grainShouldRun(const PostProcessResources &pp,
                           const PostProcessSettings &settings, bool ppActive) {
    if (settings.grainStrength <= 0.0f)
        return false;

    // Say why, once. Asking for grain with post-processing off otherwise
    // produces nothing and no explanation — the same silent refusal
    // smaaShouldRun already warns about.
    if (!ppActive) {
        static bool warned = false;
        if (!warned) {
            warned = true;
            std::fprintf(stderr,
                "[FALLBACK] postprocess: film grain needs "
                "graphics.post_process.enabled — grain runs as the final "
                "resolve of the post chain, and with no post chain there is "
                "nothing for it to resolve. Grain is off.\n");
        }
        return false;
    }

    return bgfx::isValid(pp.grainProgram) && bgfx::isValid(pp.grainFB);
}

/// Resolve `src` to the backbuffer, adding film grain.
///
/// Always the last pass before UI — see kViewGrain.
inline void submitGrain(const PostProcessResources &pp,
                        const PostProcessSettings &settings,
                        bgfx::TextureHandle src, uint32_t frameIndex) {
    bgfx::setViewFrameBuffer(kViewGrain, BGFX_INVALID_HANDLE);
    bgfx::setViewRect(kViewGrain, 0, 0, pp.outWidth, pp.outHeight);
    bgfx::setViewClear(kViewGrain, BGFX_CLEAR_NONE, 0, 1.0f, 0);

    // Resolution scale. Grain has a fixed physical size on the negative, so
    // it should occupy a fixed FRACTION of the frame however finely that
    // frame is drawn (report §9.4); `size` is quoted against 1080p and the
    // shader multiplies by this.
    const float resScale = static_cast<float>(pp.outHeight) / 1080.0f;

    // Per-frame seed. It goes into the HASH, never into a coordinate — a
    // seed added to coordinates translates the field instead of replacing
    // it, which is what made two earlier attempts read as a pattern sliding
    // across the screen.
    //
    // THE WRAP IS NOT OPTIONAL. frameIndex grows without bound and
    // `float(frameIndex) * phi` loses mantissa as it does, so fract() starts
    // quantising: measured, 2000 consecutive frames yielded 2000 distinct
    // seeds after a minute of play, 64 after an hour and 16 after four —
    // i.e. the grain decays into a static pattern over a session. An earlier
    // version wrapped for exactly this reason and the wrap was dropped in a
    // rewrite. 8191 frames is ~2.3 minutes at 60 Hz, longer than anyone can
    // hold a noise pattern in mind, and the golden-ratio low-discrepancy
    // property survives any window.
    const float wrapped = static_cast<float>(frameIndex & 8191u);
    const float goldenSeq = wrapped * 0.6180339887f;

    // Scaled to the hash's alias period so distinct frames can never collide.
    // The seed enters grainHash as p3.z, which is multiplied by **0.0973** —
    // NOT the 0.1031 of the x lane. The real period is 1/0.0973 = 10.2775;
    // measured, a seed delta of 10.2775 reproduces the field at |r| = 0.983
    // while 9.6993 gives 0.017. Spanning exactly one period means two seeds
    // in [0, P) can never differ by P, so the alias is unreachable rather
    // than merely avoided. Do NOT "correct" this to 1/0.1031 — that lands
    // 5.6% short and is safe only by luck.
    const float seed = (goldenSeq - std::floor(goldenSeq)) * 10.2775f;

    // Prove the index advances PER FRAME, once, for the first three frames.
    // An earlier wiring passed wall-clock seconds into this parameter and it
    // converted silently to an integer, so the seed changed once a SECOND.
    {
        static int reports = 0;
        if (reports < 3) {
            ++reports;
            std::fprintf(stderr, "Grain frame %u -> seed %.4f\n",
                         frameIndex, static_cast<double>(seed));
        }
    }

    const float res[4] = { resScale, settings.grainGain, 0.0f, 0.0f };

    const float grain[4] = {
        settings.grainStrength,
        settings.grainSize,
        settings.grainShadowBias,
        seed,
    };

    bgfx::setUniform(pp.u_grain, grain);
    bgfx::setUniform(pp.u_grainRes, res);
    bgfx::setTexture(0, pp.s_texScene, src,
                     BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP);
    bgfx::setVertexBuffer(0, pp.triangleVBH);
    bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A);
    bgfx::submit(kViewGrain, pp.grainProgram);
}

/// Whether SMAA can run this frame, saying once why not when it cannot.
///
/// Two ways to be asked for something that cannot be delivered:
///
///  * Post-processing is off. SMAA antialiases the composite's output, and
///    with no composite there is nothing to read.
///  * render_scale is below native. SMAA would run at output resolution and
///    round off the upscaled blocks — erasing the pixel grid the setting was
///    turned on to produce. These two are answers to opposite questions, so
///    refusing is right; silently smoothing the vintage look would not be.
inline bool smaaShouldRun(const PostProcessResources &pp,
                          const AntiAliasSettings &aa, bool ppActive) {
    if (aa.mode != AntiAliasMode::SMAA)
        return false;

    if (!pp.smaa.valid())
        return false;   // creation already said why

    if (!ppActive) {
        static bool warnedPost = false;
        if (!warnedPost) {
            warnedPost = true;
            std::fprintf(stderr,
                "[FALLBACK] smaa: antialiasing is on but post-processing is "
                "off — SMAA resolves the composite's output, so there is "
                "nothing for it to read. Set graphics.post_process.enabled.\n");
        }
        return false;
    }

    if (pp.width != pp.outWidth || pp.height != pp.outHeight) {
        static bool warnedScale = false;
        if (!warnedScale) {
            warnedScale = true;
            std::fprintf(stderr,
                "[FALLBACK] smaa: antialiasing is on together with "
                "graphics.render_scale (%ux%u internal -> %ux%u window), and "
                "the two cancel out — SMAA runs at window resolution and would "
                "smooth the very pixel blocks render_scale exists to make. "
                "SMAA is off; set render_scale.height to 0 to use it.\n",
                pp.width, pp.height, pp.outWidth, pp.outHeight);
        }
        return false;
    }

    return true;
}

/// Run SMAA's three passes, resolving `pp.smaa.colorTex` to the backbuffer.
///
/// Call only when smaaShouldRun() returned true, and only after
/// submitComposite() has been told to render into the SMAA colour target.
inline void submitSmaa(const PostProcessResources &pp,
                       const AntiAliasSettings &aa, bool toGrain) {
    const SmaaResources &smaa = pp.smaa;

    const float metrics[4] = {
        1.0f / static_cast<float>(smaa.width),
        1.0f / static_cast<float>(smaa.height),
        static_cast<float>(smaa.width),
        static_cast<float>(smaa.height),
    };
    const float params[4] = {
        aa.smaaThreshold,
        static_cast<float>(static_cast<int>(aa.debug)),
        0.0f, 0.0f,
    };

    // ── Pass 1: edge detection ──
    //
    // Cleared, and that clear is load-bearing: the edge shader `discard`s on
    // non-edge pixels rather than writing zero, so without it last frame's
    // edges would persist wherever this frame has none.
    bgfx::setViewFrameBuffer(kViewSmaaEdges, smaa.edgesFB);
    bgfx::setViewRect(kViewSmaaEdges, 0, 0, smaa.width, smaa.height);
    bgfx::setViewClear(kViewSmaaEdges, BGFX_CLEAR_COLOR, 0x00000000, 1.0f, 0);
    bgfx::setUniform(smaa.u_smaaMetrics, metrics);
    bgfx::setUniform(smaa.u_smaaParams,  params);
    bgfx::setTexture(0, smaa.s_smaaColor, smaa.colorTex);
    bgfx::setVertexBuffer(0, pp.triangleVBH);
    bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A);
    bgfx::submit(kViewSmaaEdges, smaa.edgeProgram);

    // ── Pass 2: blending weights ──
    bgfx::setViewFrameBuffer(kViewSmaaWeights, smaa.weightsFB);
    bgfx::setViewRect(kViewSmaaWeights, 0, 0, smaa.width, smaa.height);
    bgfx::setViewClear(kViewSmaaWeights, BGFX_CLEAR_COLOR, 0x00000000, 1.0f, 0);
    bgfx::setUniform(smaa.u_smaaMetrics, metrics);
    bgfx::setUniform(smaa.u_smaaParams,  params);
    // Stages must match the SAMPLER2D registers the shaders declare: bgfx
    // binds by stage, and on Metal/D3D the register is baked at compile time.
    bgfx::setTexture(0, smaa.s_smaaEdges,  smaa.edgesTex);
    bgfx::setTexture(1, smaa.s_smaaArea,   smaa.areaTex);
    bgfx::setTexture(2, smaa.s_smaaSearch, smaa.searchTex);
    bgfx::setVertexBuffer(0, pp.triangleVBH);
    bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A);
    bgfx::submit(kViewSmaaWeights, smaa.weightProgram);

    // ── Pass 3: neighborhood blending — to the grain target if grain still
    // has to run, otherwise straight to the backbuffer ──
    // Assign-then-overwrite, NOT a ternary: BGFX_INVALID_HANDLE is a braced
    // initialiser and cannot be a ternary operand (HANDOFF.VISUAL_PIPELINE.md
    // §7 says so, and this walked into it anyway).
    bgfx::FrameBufferHandle blendTarget = BGFX_INVALID_HANDLE;
    if (toGrain) blendTarget = pp.grainFB;
    bgfx::setViewFrameBuffer(kViewSmaaBlend, blendTarget);
    bgfx::setViewRect(kViewSmaaBlend, 0, 0, smaa.width, smaa.height);
    bgfx::setViewClear(kViewSmaaBlend, BGFX_CLEAR_NONE, 0, 1.0f, 0);
    bgfx::setUniform(smaa.u_smaaMetrics, metrics);
    bgfx::setUniform(smaa.u_smaaParams,  params);
    bgfx::setTexture(0, smaa.s_smaaColor,   smaa.colorTex);
    bgfx::setTexture(1, smaa.s_smaaWeights, smaa.weightsTex);
    bgfx::setTexture(2, smaa.s_smaaEdges,   smaa.edgesTex);  // debug view only
    bgfx::setVertexBuffer(0, pp.triangleVBH);
    bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A);
    bgfx::submit(kViewSmaaBlend, smaa.blendProgram);
}

/// Point the scene views at the offscreen target, or back at the backbuffer.
/// Called every frame so the console toggle takes effect live.
inline void bindSceneTarget(const PostProcessResources &pp, bool active) {
    // BGFX_INVALID_HANDLE is a braced initializer, so it cannot appear as a
    // ternary operand — assign it, then overwrite.
    bgfx::FrameBufferHandle target = BGFX_INVALID_HANDLE;
    if (active)
        target = pp.frameBuffer;
    bgfx::setViewFrameBuffer(kViewSky,   target);
    bgfx::setViewFrameBuffer(kViewWorld, target);
    bgfx::setViewFrameBuffer(kViewDebug, target);

    // Scene view rects follow the target, every frame. Under render_scale the
    // offscreen target is smaller than the window, so the two cases need
    // different viewports — and post-processing is toggleable live from the
    // console, so latching this at init would leave the scene drawn into a
    // small corner of the backbuffer the moment it was switched off.
    if (pp.outWidth > 0 && pp.outHeight > 0 && pp.width != pp.outWidth) {
        const uint16_t vw = active ? pp.width  : pp.outWidth;
        const uint16_t vh = active ? pp.height : pp.outHeight;
        bgfx::setViewRect(kViewSky,    0, 0, vw, vh);
        bgfx::setViewRect(kViewWorld,  0, 0, vw, vh);
        bgfx::setViewRect(kViewCorona, 0, 0, vw, vh);
        bgfx::setViewRect(kViewDebug,  0, 0, vw, vh);
    }

    // Coronas go to the depth-free view of the same colour texture when one
    // exists, so the pass can sample depth. Without it they share the normal
    // target and simply do not read depth — the fade turns itself off, and
    // the ray trace stays their only occluder.
    bgfx::FrameBufferHandle coronaTarget = target;
    if (active && bgfx::isValid(pp.colorOnlyFrameBuffer))
        coronaTarget = pp.colorOnlyFrameBuffer;
    bgfx::setViewFrameBuffer(kViewCorona, coronaTarget);
}

/// One separable blur pass: `src` → `dstFB`, stepping along `dirX/dirY`.
inline void submitBlurPass(const PostProcessResources &pp, bgfx::ViewId view,
                           bgfx::FrameBufferHandle dstFB,
                           bgfx::TextureHandle src,
                           float dirX, float dirY, float blurSize) {
    bgfx::setViewFrameBuffer(view, dstFB);
    bgfx::setViewRect(view, 0, 0, pp.bloomWidth, pp.bloomHeight);
    bgfx::setViewClear(view, BGFX_CLEAR_NONE, 0, 1.0f, 0);

    // Step is in UV space, so it is relative to the blur target's own size —
    // a fixed blur size therefore covers the same fraction of the screen at
    // any resolution, which is what makes the look resolution-independent.
    const float step[4] = {
        dirX / static_cast<float>(pp.bloomWidth)  * blurSize,
        dirY / static_cast<float>(pp.bloomHeight) * blurSize,
        0.0f, 0.0f,
    };
    bgfx::setUniform(pp.u_blurStep, step);
    bgfx::setTexture(0, pp.s_texBlurSrc, src);
    bgfx::setVertexBuffer(0, pp.triangleVBH);
    bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A);
    bgfx::submit(view, pp.blurProgram);
}

/// How many pyramid levels a requested radius needs, counting level 0.
///
/// `range` is NewDark's `bloom_range`: glow radius as a percentage of the
/// screen DIAGONAL (new_config_vars.txt:369-384, default 2).
///
/// The depth is chosen from kBloomPyramidRadius, the MEASURED radius of each
/// depth, rather than from the reach-doubles-per-level model — see that
/// table's comment for why the model is wrong here.
///
/// Depth is a whole number of levels, so the radius is quantised: consecutive
/// depths sit roughly 1.45x apart. Picking the shallowest depth that REACHES
/// the request would therefore overshoot by up to 45% (measured worst case
/// 1.63x), which is no more honest than undershooting. Picking the NEAREST
/// depth halves that to about 1.25x. The comparison is in log space because
/// the levels are geometrically spaced — nearest in linear distance would
/// systematically favour the deeper, wider level.
///
/// Returns 1 for "no pyramid": that is what range 0 means, and also what a
/// request below the shallowest pyramid's reach means, since the base blur
/// already covers that.
inline int bloomPyramidLevelsFor(float range, uint16_t outWidth,
                                 uint16_t outHeight, int available) {
    if (range <= 0.0f || available <= 1)
        return 1;

    const float diagonal = std::sqrt(
        static_cast<float>(outWidth)  * static_cast<float>(outWidth) +
        static_cast<float>(outHeight) * static_cast<float>(outHeight));

    // Requested radius in level-0 texels. Level 0 is quarter res, so one of
    // its texels is 4 screen pixels at any resolution — hence the 0.25.
    const float radiusTexels = (range * 0.01f) * diagonal * 0.25f;

    if (available > kMaxBloomLevels)
        available = kMaxBloomLevels;

    // Below the shallowest pyramid's own reach there is nothing to add: the
    // base blur is already at least this wide.
    if (radiusTexels < kBloomPyramidRadius[1])
        return 1;

    // Nearest depth in log space. Index 0 is skipped — its radius is 0, which
    // has no logarithm and means "no pyramid" anyway.
    int    best     = 1;
    float  bestDist = std::numeric_limits<float>::max();
    for (int levels = 2; levels <= available; ++levels) {
        const float dist = std::fabs(
            std::log(kBloomPyramidRadius[levels - 1] / radiusTexels));
        if (dist < bestDist) {
            bestDist = dist;
            best     = levels;
        }
    }

    // A depth short of the deepest built one is the nearest match, not a
    // shortfall — only complain when the request ran past the end.
    if (best < available || kBloomPyramidRadius[available - 1] >= radiusTexels)
        return best;

    // Deeper than anything built. Say so once — quietly running a shallower
    // pyramid is precisely the "key that lies" failure bloom_range was
    // deleted for in VIS-2f, and it must not come back with the key.
    static bool warned = false;
    if (!warned) {
        warned = true;
        std::fprintf(stderr,
            "[FALLBACK] postprocess: bloom_range %.2f%% asks for a glow "
            "radius of %.1f level-0 texels (%.0f px) but the deepest pyramid "
            "available here reaches %.1f (%.0f px) — the glow will be tighter "
            "than requested. %d of %d levels were built; a higher display "
            "resolution needs more depth for the same percentage.\n",
            static_cast<double>(range),
            static_cast<double>(radiusTexels),
            static_cast<double>(radiusTexels * 4.0f),
            static_cast<double>(kBloomPyramidRadius[available - 1]),
            static_cast<double>(kBloomPyramidRadius[available - 1] * 4.0f),
            available, kMaxBloomLevels);
    }
    return available;
}

/// Widen the glow by walking a progressive-downsample pyramid (VIS-3b).
///
/// Runs after the ping-pong blur has filled level 0 (bloomFB[1]) and leaves
/// its result there, so the composite reads the same texture either way.
///
/// Down the chain each level is an exact 2x box reduction; back up, each
/// level is added into the one above through a 3x3 tent. The sum is a stack
/// of progressively wider, progressively dimmer kernels — a much better wide
/// Gaussian than any single pass at this cost, and crucially every filter
/// stays at ~1 texel of its own level, which is where the 5-tap kernel and
/// the tent are both valid. Radius comes from DEPTH, never from step size.
/// That is the whole reason this exists; see fs_bloom_blur.sc and VIS-2f.
///
/// No-ops when bloom_range is 0, which is the default — the chain is then
/// exactly the pre-VIS-3b one, i.e. HPL2's own construction.
inline void renderBloomPyramid(const PostProcessResources &pp,
                               const PostProcessSettings &settings) {
    const int levels = bloomPyramidLevelsFor(
        settings.bloomRange, pp.outWidth, pp.outHeight, pp.bloomPyrLevels);
    if (levels <= 1)
        return;
    if (!bgfx::isValid(pp.downsampleProgram)
        || !bgfx::isValid(pp.upsampleProgram))
        return;

    // Say what the chain is actually doing, once per depth change. "Built 7
    // levels" only proves allocation; this proves the passes SUBMIT, and it
    // is the difference between a wider glow and a setting that quietly does
    // nothing. Same reason the corona and SelfLit producers announce
    // themselves — see HANDOFF.VISUAL_PIPELINE.md §4.
    static int reported = -1;
    if (reported != levels) {
        reported = levels;
        std::fprintf(stderr,
            "Bloom pyramid: %d level(s) active for bloom_range %.2f%% "
            "(~%.0f px glow radius)\n",
            levels, static_cast<double>(settings.bloomRange),
            static_cast<double>(kBloomPyramidRadius[levels - 1] * 4.0f));
    }

    // ── Down leg ──
    // Level 0 is bloomTex[1]; pyramid index i holds level i+1.
    bgfx::ViewId view = kViewBloomPyrDown;
    bgfx::TextureHandle src = pp.bloomTex[1];
    uint16_t srcW = pp.bloomWidth;
    uint16_t srcH = pp.bloomHeight;

    for (int i = 0; i < levels - 1; ++i) {
        bgfx::setViewFrameBuffer(view, pp.bloomPyrFB[i]);
        bgfx::setViewRect(view, 0, 0, pp.bloomPyrW[i], pp.bloomPyrH[i]);
        bgfx::setViewClear(view, BGFX_CLEAR_NONE, 0, 1.0f, 0);

        // SOURCE texel size — the downsample offsets are half a source texel,
        // which is where the source texel centres are relative to a
        // destination centre sitting on their shared boundary.
        const float step[4] = {
            1.0f / static_cast<float>(srcW),
            1.0f / static_cast<float>(srcH),
            0.0f, 0.0f,
        };
        bgfx::setUniform(pp.u_blurStep, step);
        bgfx::setTexture(0, pp.s_texBlurSrc, src);
        bgfx::setVertexBuffer(0, pp.triangleVBH);
        bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A);
        bgfx::submit(view, pp.downsampleProgram);

        src  = pp.bloomPyrTex[i];
        srcW = pp.bloomPyrW[i];
        srcH = pp.bloomPyrH[i];
        ++view;
    }

    // ── Up leg ──
    // Walk back, adding each level into the one above. The accumulation is
    // the framebuffer's (BLEND_ADD), not the shader's, because the
    // destination cannot be both render target and sampler in one draw.
    view = kViewBloomPyrUp;
    for (int i = levels - 2; i >= 0; --i) {
        const bool toBase = (i == 0);
        const bgfx::FrameBufferHandle dst =
            toBase ? pp.bloomFB[1] : pp.bloomPyrFB[i - 1];
        const uint16_t dstW = toBase ? pp.bloomWidth  : pp.bloomPyrW[i - 1];
        const uint16_t dstH = toBase ? pp.bloomHeight : pp.bloomPyrH[i - 1];

        bgfx::setViewFrameBuffer(view, dst);
        bgfx::setViewRect(view, 0, 0, dstW, dstH);
        bgfx::setViewClear(view, BGFX_CLEAR_NONE, 0, 1.0f, 0);

        // Tent offsets are one SOURCE texel — the small level being read —
        // and .z is the scatter weight the blend unit lerps by.
        const float step[4] = {
            1.0f / static_cast<float>(pp.bloomPyrW[i]),
            1.0f / static_cast<float>(pp.bloomPyrH[i]),
            kBloomScatter,
            0.0f,
        };
        bgfx::setUniform(pp.u_blurStep, step);
        bgfx::setTexture(0, pp.s_texBlurSrc, pp.bloomPyrTex[i]);
        bgfx::setVertexBuffer(0, pp.triangleVBH);
        // BLEND_ALPHA with the shader's alpha = scatter IS the lerp; see
        // fs_bloom_upsample.sc. No WRITE_A: alpha is an input to the blend
        // unit here, not something to leave behind in the target.
        bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_BLEND_ALPHA);
        bgfx::submit(view, pp.upsampleProgram);
        ++view;
    }
}

/// Sigma, in OUTPUT PIXELS, of one pass of the 5-tap blur at quarter res.
///
/// Derived from the kernel itself rather than guessed: with weights
/// (0.2270, 0.3162, 0.0703) at offsets (0, ±1.3846, ±3.2308) the variance is
/// 2*(0.3162*1.3846² + 0.0703*3.2308²) = 2.679 texels², so sigma is 1.637
/// quarter-res texels, and a quarter-res texel is 4 output pixels.
static constexpr float kBlurSigmaPixels = 1.637f * 4.0f;   // ≈ 6.55

/// Render the halation halo into haloTex[1]: threshold hard, blur small.
///
/// This is the CineStill recipe, and each step is one of its defining
/// properties rather than a tuning choice — see PostProcessSettings.
/// Deliberately independent of bloom: it reads the SCENE texture, because
/// halation is caused by the bright source itself, not by bloom's glow.
///
/// Returns false when halation did not run, so the composite can zero its
/// contribution rather than sampling a stale target.
inline bool renderHalation(const PostProcessResources &pp,
                           const PostProcessSettings &settings) {
    if (settings.halationStrength <= 0.0f)
        return false;
    if (!pp.valid() || !bgfx::isValid(pp.extractProgram)
        || !bgfx::isValid(pp.blurProgram)
        || !bgfx::isValid(pp.haloFB[0]) || !bgfx::isValid(pp.haloFB[1]))
        return false;

    // ── Threshold + downsample ──
    // Same shader as bloom's bright pass, but with halation's own (much
    // higher) threshold. prescale stays 1: the strength knob at composite
    // time is where intensity belongs, so that changing radius or threshold
    // does not also change brightness.
    bgfx::setViewFrameBuffer(kViewHalationExtract, pp.haloFB[1]);
    bgfx::setViewRect(kViewHalationExtract, 0, 0, pp.bloomWidth, pp.bloomHeight);
    bgfx::setViewClear(kViewHalationExtract, BGFX_CLEAR_NONE, 0, 1.0f, 0);

    // Threshold the EXPOSED value: sceneColor*exposure > threshold is the
    // same test as sceneColor > threshold/exposure, and the extract pass
    // reads the scene target before the composite applies exposure. Doing it
    // here costs nothing and keeps the knob meaning one thing.
    const float exposure = settings.exposure > 1e-3f ? settings.exposure : 1e-3f;
    const float extract[4] = {
        settings.halationThreshold / exposure,
        1.0f,
        1.0f / static_cast<float>(pp.width),
        1.0f / static_cast<float>(pp.height),
    };
    bgfx::setUniform(pp.u_extractParams, extract);
    bgfx::setTexture(0, pp.s_texBlurSrc, pp.sceneColor);
    bgfx::setVertexBuffer(0, pp.triangleVBH);
    bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A);
    bgfx::submit(kViewHalationExtract, pp.extractProgram);

    // ── Blur, radius expressed in output pixels ──
    // N passes of a Gaussian compose as sigma*sqrt(N), so the iteration count
    // is (radius / sigma_per_pass)². Step stays at 1.0 throughout — widening
    // it would turn the 5-tap kernel into a sparse comb (see fs_bloom_blur.sc
    // and VIS-2f), which is the one thing this must not do.
    // Scale with output height against a 1080p reference, exactly as grain
    // does. Report §9.4 says halation radius and grain size should share a
    // "film format" scaling; once grain scaled and halation did not, the two
    // were scaled DIFFERENTLY, which is worse than neither scaling.
    const float haloResScale = static_cast<float>(pp.outHeight) / 1080.0f;
    const float wanted =
        (settings.halationRadius * haloResScale) / kBlurSigmaPixels;
    int iterations = static_cast<int>(std::ceil(wanted * wanted));
    if (iterations < 1)
        iterations = 1;
    if (iterations > kMaxHalationIterations) {
        static bool warned = false;
        if (!warned) {
            warned = true;
            std::fprintf(stderr,
                "[FALLBACK] postprocess: halation radius %.1f px needs %d blur "
                "iterations but the cap is %d — the halo will be tighter than "
                "requested (max ~%.1f px). Halation is deliberately the LOCAL "
                "glow; for a wide warm spread use bloom's range with a warm "
                "tint instead.\n",
                static_cast<double>(settings.halationRadius), iterations,
                kMaxHalationIterations,
                static_cast<double>(kBlurSigmaPixels
                    * std::sqrt(static_cast<float>(kMaxHalationIterations))));
        }
        iterations = kMaxHalationIterations;
    }

    // Reachability, once per settings change. "No fallback appeared" does
    // not mean the pass ran — see HANDOFF.VISUAL_PIPELINE.md §4 — and this
    // also states the radius actually achieved, which is quantised to whole
    // blur iterations and so is rarely exactly what was asked for.
    {
        static int reportedIters = -1;
        static float reportedThresh = -1.0f;
        if (reportedIters != iterations
            || reportedThresh != settings.halationThreshold) {
            reportedIters = iterations;
            reportedThresh = settings.halationThreshold;
            std::fprintf(stderr,
                "Halation: threshold %.2f post-exposure (scene %.2f), %d blur "
                "iteration(s) -> ~%.1f px halo (asked %.1f)\n",
                static_cast<double>(settings.halationThreshold),
                static_cast<double>(settings.halationThreshold / exposure),
                iterations,
                static_cast<double>(kBlurSigmaPixels
                    * std::sqrt(static_cast<float>(iterations))),
                static_cast<double>(settings.halationRadius));
        }
    }

    bgfx::ViewId view = kViewHalationBase;
    bgfx::TextureHandle src = pp.haloTex[1];
    for (int i = 0; i < iterations; ++i) {
        submitBlurPass(pp, view++, pp.haloFB[0], src, 1.0f, 0.0f, 1.0f);
        submitBlurPass(pp, view++, pp.haloFB[1], pp.haloTex[0], 0.0f, 1.0f, 1.0f);
        src = pp.haloTex[1];
    }
    return true;
}

/// Run the bloom blur chain, leaving the result in bloomTex[1].
///
/// Structure follows HPL2's cPostEffect_Bloom::RenderEffect: blur the scene
/// once, then re-blur the previous result `mlBlurIterations - 1` more times,
/// each iteration being a horizontal pass followed by a vertical one. Two
/// ping-pong quarter-res buffers, no mip pyramid.
///
/// Returns false if bloom did not run, so the caller can zero the bloom
/// contribution rather than compositing a stale or uninitialised target.
inline bool renderBloom(const PostProcessResources &pp,
                        const PostProcessSettings &settings) {
    if (!settings.bloomEnabled || !pp.bloomValid())
        return false;

    int iterations = settings.bloomIterations;
    if (iterations < 1)                    iterations = 1;
    if (iterations > kMaxBloomIterations)  iterations = kMaxBloomIterations;

    const bool newDark = (settings.bloomStyle == BloomStyle::NewDark);

    float blurSize = settings.bloomBlurSize;

    // ── Downsample (+ bright pass for the NewDark style) ──
    //
    // Runs for BOTH styles. It is not merely NewDark's bright pass: it is
    // also the only place a correct 4x downsample happens. Letting the
    // first blur pass do the downsampling instead undersamples — it is
    // 1-tap along the axis it is not blurring — which shows up as grain or
    // shimmer on thin bright features. See fs_bloom_extract.sc.
    //
    // The amnesia style passes threshold 0 / prescale 1, so the bright-pass
    // stage is an exact identity and this is a pure downsample for it.
    bgfx::setViewFrameBuffer(kViewBloomExtract, pp.bloomFB[1]);
    bgfx::setViewRect(kViewBloomExtract, 0, 0, pp.bloomWidth, pp.bloomHeight);
    bgfx::setViewClear(kViewBloomExtract, BGFX_CLEAR_NONE, 0, 1.0f, 0);

    const float extract[4] = {
        newDark ? settings.ndThreshold : 0.0f,
        newDark ? settings.ndPrescale  : 1.0f,
        // Source texel size — the scene target, not the blur target.
        1.0f / static_cast<float>(pp.width),
        1.0f / static_cast<float>(pp.height),
    };
    bgfx::setUniform(pp.u_extractParams, extract);
    bgfx::setTexture(0, pp.s_texBlurSrc, pp.sceneColor);
    bgfx::setVertexBuffer(0, pp.triangleVBH);
    bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A);
    bgfx::submit(kViewBloomExtract, pp.extractProgram);

    bgfx::TextureHandle src = pp.bloomTex[1];

    // ── Kernel-validity cap on the per-pass step ──
    //
    // The 5-tap kernel in fs_bloom_blur.sc is a 9-tap Gaussian collapsed
    // onto bilinear pairs, which is only a Gaussian at its designed tap
    // spacing (0, +-1.3846, +-3.2308 texels). Scaling those offsets does
    // not widen the blur — it turns it into a sparse comb. Each bilinear
    // tap covers roughly +-1 texel, so beyond a step of about 1.3 the
    // outer taps stop overlapping and leave unsampled gaps. Sampling a
    // gappy comb instead of a continuous kernel is aliasing, and it reads
    // on screen as grain.
    //
    // Radius must therefore come from iterations, not from a bigger step:
    // N passes of a Gaussian compose to sigma * sqrt(N).
    constexpr float kMaxBlurStep = 1.3f;
    if (blurSize > kMaxBlurStep) {
        const float excess = blurSize / kMaxBlurStep;
        const int wanted = static_cast<int>(
            std::ceil(static_cast<float>(iterations) * excess * excess));
        blurSize = kMaxBlurStep;
        if (wanted > kMaxBloomIterations) {
            // Warn once, not per frame — the requested radius is simply
            // not reachable at quarter res with this kernel. Reaching it
            // properly needs a progressive-downsample pyramid, where each
            // level halves resolution so the step stays near 1 texel.
            static bool warned = false;
            if (!warned) {
                warned = true;
                std::fprintf(stderr,
                    "[FALLBACK] postprocess: bloom radius needs %d blur "
                    "iterations but the cap is %d — the glow will be "
                    "tighter than requested. Widening the per-pass step "
                    "instead would alias (visible as grain), so it is "
                    "capped at %.2f texels.\n",
                    wanted, kMaxBloomIterations,
                    static_cast<double>(kMaxBlurStep));
            }
            iterations = kMaxBloomIterations;
        } else {
            iterations = wanted;
        }
    }

    bgfx::ViewId view = kViewBloomBase;
    for (int i = 0; i < iterations; ++i) {
        // Both passes now run entirely at quarter resolution — the extract
        // above already downsampled — so the blur step is in the target's
        // own texel units throughout and the kernel keeps its intended
        // shape on the first iteration as well as later ones.
        submitBlurPass(pp, view++, pp.bloomFB[0], src, 1.0f, 0.0f, blurSize);
        // Vertical: bloomTex[0] → bloomFB[1].
        submitBlurPass(pp, view++, pp.bloomFB[1], pp.bloomTex[0],
                       0.0f, 1.0f, blurSize);
        src = pp.bloomTex[1];
    }

    renderBloomPyramid(pp, settings);
    return true;
}

/// Resolve the scene target to the backbuffer through the composite shader.
/// `bloomActive` must be the return value of renderBloom() for this frame.
/// `toSmaa` diverts the result into the SMAA colour target instead, for the
/// antialiasing passes to resolve — pass smaaShouldRun()'s answer.
inline void submitComposite(const PostProcessResources &pp,
                            const PostProcessSettings &settings,
                            bool bloomActive, bool halationActive, bool toSmaa,
                            bool toGrain) {
    if (!pp.valid())
        return;

    // Re-bound every frame rather than latched at init, for the same reason
    // the scene views are: antialiasing is toggleable live from the console,
    // and a composite still pointing at last frame's target would either show
    // a stale image or drop the frame entirely.
    // Where the composite lands depends on what still has to run after it.
    // SMAA takes priority (it resolves into the grain target itself); with
    // neither, straight to the backbuffer as before.
    bgfx::FrameBufferHandle compositeTarget = BGFX_INVALID_HANDLE;
    if (toSmaa)        compositeTarget = pp.smaa.colorFB;
    else if (toGrain)  compositeTarget = pp.grainFB;
    bgfx::setViewFrameBuffer(kViewComposite, compositeTarget);
    bgfx::setViewRect(kViewComposite, 0, 0, pp.outWidth, pp.outHeight);

    float luma[3];
    lumaWeightsFor(settings.lumaMode, luma);

    const float params0[4] = {
        settings.exposure,
        static_cast<float>(static_cast<int>(settings.tonemap)),
        settings.gamma,
        0.0f,
    };
    const float params1[4] = {
        settings.brightness,
        settings.contrast,
        settings.saturation,
        0.0f,
    };
    const float filter[4] = {
        settings.filterR, settings.filterG, settings.filterB, 0.0f,
    };
    const float lumaWeights[4] = { luma[0], luma[1], luma[2], 0.0f };

    // rgb = luma weights scaled by intensity. HPL2 tunes bloom strength
    // exactly this way — LuxMapHandler.cpp:171 scales mvRgbToIntensity
    // rather than introducing a separate multiplier.
    // w = 0 zeroes the contribution entirely when bloom did not run this
    // frame, so a stale or never-written blur target cannot leak in.
    const float bloomParams[4] = {
        luma[0] * settings.bloomIntensity,
        luma[1] * settings.bloomIntensity,
        luma[2] * settings.bloomIntensity,
        bloomActive ? 1.0f : 0.0f,
    };

    const float bloomStyle[4] = {
        (settings.bloomStyle == BloomStyle::NewDark) ? 1.0f : 0.0f,
        settings.ndScale,
        settings.ndSaturation,
        0.0f,
    };

    float curve[4];
    computeCurveParams(settings, curve);

    // Report the live curve and its solved constants once per operator
    // change. Two reasons this earns its line: the constants are solved on
    // the CPU from knobs the console can move, so "the curve is not doing
    // what I set" and "the solve produced nonsense" look identical from
    // outside; and it is the only way to confirm from a log which operator a
    // session actually ran. Rate-limited to operator changes so riding a
    // slider does not spam.
    {
        static int reportedOp = -1;
        const int op = static_cast<int>(settings.tonemap);
        if (reportedOp != op) {
            reportedOp = op;
            static const char *kNames[] = {
                "none", "reinhard", "aces", "agx", "pbrneutral", "lottes" };
            const char *name = (op >= 0 && op <= 5) ? kNames[op] : "?";
            if (settings.tonemap == ToneMapOperator::Lottes) {
                std::fprintf(stderr,
                    "Tone curve: %s — contrast %.3f shoulder %.3f white %.2f "
                    "-> a %.6f ad %.6f b %.8f c %.8f (mid 0.18 pinned)\n",
                    name, static_cast<double>(settings.lottesContrast),
                    static_cast<double>(settings.lottesShoulder),
                    static_cast<double>(settings.lottesWhite),
                    static_cast<double>(curve[0]), static_cast<double>(curve[1]),
                    static_cast<double>(curve[2]), static_cast<double>(curve[3]));
            } else if (settings.tonemap == ToneMapOperator::AgX) {
                std::fprintf(stderr,
                    "Tone curve: %s — contrast %.3f -> toe_a %.6f slope %.6f "
                    "w %.6f\n",
                    name, static_cast<double>(settings.agxContrast),
                    static_cast<double>(curve[1]), static_cast<double>(curve[2]),
                    static_cast<double>(curve[3]));
            } else {
                std::fprintf(stderr, "Tone curve: %s\n", name);
            }
        }
    }

    // Film response. Strength 0 makes the shader's mix() an exact identity,
    // which is what keeps the composite bit-identical to the direct path at
    // defaults — the whole stage ships off.
    const float filmParams[4] = {
        settings.filmStrength,
        settings.filmShadowSat,
        settings.filmHighlightSat,
        settings.filmToneFalloff,
    };
    const float tintLo[4] = { settings.filmShadowTint[0],
                              settings.filmShadowTint[1],
                              settings.filmShadowTint[2], 0.0f };
    const float tintHi[4] = { settings.filmHighlightTint[0],
                              settings.filmHighlightTint[1],
                              settings.filmHighlightTint[2], 0.0f };

    // Halation has its own chain now, so it no longer depends on bloom
    // having run — only on its own pass having run. w = 0 zeroes the
    // contribution when it did not, so a stale target cannot leak in.
    const float halation[4] = { settings.halationTint[0],
                                settings.halationTint[1],
                                settings.halationTint[2],
                                halationActive ? settings.halationStrength
                                               : 0.0f };

    // Grain advances with wall time rather than frame index so it does not
    // freeze when the frame rate does, and does not run faster on a faster
    // machine. Wrapped to keep float precision usable in a long session:
    // past a few thousand seconds the fractional part of a raw timer starts
    // quantising and the noise visibly stops moving.
    bgfx::setUniform(pp.u_ccParams0,   params0);
    bgfx::setUniform(pp.u_ccParams1,   params1);
    bgfx::setUniform(pp.u_ccFilter,    filter);
    bgfx::setUniform(pp.u_lumaWeights, lumaWeights);
    bgfx::setUniform(pp.u_bloomParams, bloomParams);
    bgfx::setUniform(pp.u_bloomStyle,  bloomStyle);
    bgfx::setUniform(pp.u_curveParams, curve);
    bgfx::setUniform(pp.u_filmParams,  filmParams);
    bgfx::setUniform(pp.u_filmTintLo,  tintLo);
    bgfx::setUniform(pp.u_filmTintHi,  tintHi);
    bgfx::setUniform(pp.u_halation,    halation);
    // Explicit sampler flags rather than the texture's baked ones: bloom's
    // first blur reads this SAME texture and needs LINEAR for its 2x2
    // downsample, so the point filter has to be chosen here, per-draw, not
    // baked at creation.
    bgfx::setTexture(0, pp.s_texScene, pp.sceneColor,
                     (pp.pointUpscale ? BGFX_SAMPLER_POINT : 0)
                     | BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP);

    // The bloom sampler must always have a valid texture bound even when
    // bloom is off, or backends that validate bindings will complain; the
    // w=0 above is what actually neutralises it. Fall back to the scene
    // texture when no bloom target exists.
    bgfx::TextureHandle bloomTex = pp.sceneColor;
    if (bgfx::isValid(pp.bloomTex[1]))
        bloomTex = pp.bloomTex[1];
    bgfx::setTexture(1, pp.s_texBloom, bloomTex);

    // Same rule for halation: always bind something valid, and let the
    // strength above be what actually neutralises it.
    bgfx::TextureHandle haloTex = pp.sceneColor;
    if (bgfx::isValid(pp.haloTex[1]))
        haloTex = pp.haloTex[1];
    bgfx::setTexture(2, pp.s_texHalation, haloTex);

    bgfx::setVertexBuffer(0, pp.triangleVBH);
    // No depth test, no depth write, no culling: the triangle is already in
    // clip space and covers the frame.
    bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A);
    bgfx::submit(kViewComposite, pp.compositeProgram);
}

} // namespace Darkness
