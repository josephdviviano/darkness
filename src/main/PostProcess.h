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

#include <bgfx/bgfx.h>

namespace Darkness {

// bgfx view ids. Views 0-2 predate this module.
//
// bgfx submits views in id order, so the bloom chain must occupy ids
// between the scene and the composite. Each blur pass needs its own id
// because each targets a different framebuffer; two passes (horizontal +
// vertical) per iteration.
static constexpr bgfx::ViewId kViewSky          = 0;
static constexpr bgfx::ViewId kViewWorld        = 1;
static constexpr bgfx::ViewId kViewDebug        = 2;
// Bright-pass extraction. Only the NewDark style submits it; the Amnesia
// style blurs the scene directly and leaves this view unused.
static constexpr bgfx::ViewId kViewBloomExtract = 3;
static constexpr bgfx::ViewId kViewBloomBase    = 4;

// HPL2 ships 2 iterations (cPostEffectParams_Bloom::mlBlurIterations).
// Reserve headroom for 4 so the setting is explorable without renumbering
// views; unused ids simply never get submitted.
static constexpr int kMaxBloomIterations = 4;

static constexpr bgfx::ViewId kViewComposite =
    kViewBloomBase + kMaxBloomIterations * 2;

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

/// Translate a sample count into bgfx's render-target MSAA bits.
///
/// bgfx packs the sample count into the texture flags rather than taking it
/// as a number, and the same value is needed in two places: the offscreen
/// scene target, and (for the direct path when post-processing is off) the
/// backbuffer reset flags. Keeping the mapping here stops the two drifting.
inline uint64_t msaaTextureFlag(int samples) {
    switch (samples) {
        case 2:  return BGFX_TEXTURE_RT_MSAA_X2;
        case 4:  return BGFX_TEXTURE_RT_MSAA_X4;
        case 8:  return BGFX_TEXTURE_RT_MSAA_X8;
        case 16: return BGFX_TEXTURE_RT_MSAA_X16;
        default: return BGFX_TEXTURE_RT;
    }
}

inline uint32_t msaaResetFlag(int samples) {
    switch (samples) {
        case 2:  return BGFX_RESET_MSAA_X2;
        case 4:  return BGFX_RESET_MSAA_X4;
        case 8:  return BGFX_RESET_MSAA_X8;
        case 16: return BGFX_RESET_MSAA_X16;
        default: return 0;
    }
}

inline void lumaWeightsFor(LumaMode mode, float outRGB[3]) {
    if (mode == LumaMode::LCD) {
        outRGB[0] = 0.2126f; outRGB[1] = 0.7152f; outRGB[2] = 0.0722f;
    } else {
        outRGB[0] = 0.30f;   outRGB[1] = 0.58f;   outRGB[2] = 0.12f;
    }
}

// Tone-mapping operators. Values are the wire format for the shader's
// u_ccParams0.y and for the config/console enum — keep them stable.
enum class ToneMapOperator : int {
    None     = 0,  // clamp only: reproduces the pre-post-process image exactly
    Reinhard = 1,
    ACES     = 2,
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
    // NewDark's fifth parameter, bloom_range (radius as a percentage of the
    // screen diagonal, default 2), is deliberately NOT implemented. At
    // 1280x720 it asks for a ~29px radius, which at quarter resolution is
    // ~7.3 texels — far beyond what the 5-tap kernel can express as a step,
    // and reaching it by iteration alone needs roughly 10 passes. Both
    // styles therefore share bloomBlurSize / bloomIterations, whose HPL2
    // defaults (1.0 / 2) sit exactly at the kernel's designed tap spacing
    // and produce a genuine Gaussian rather than a sparse comb.
    //
    // Consequence, stated plainly: the newdark style's glow is TIGHTER than
    // NewDark's nominal radius. It reproduces NewDark's decision about
    // *what* glows (threshold, desaturation, scale), not how far the glow
    // spreads. Restoring the true radius means a progressive-downsample
    // pyramid, where halving resolution per level keeps the step near one
    // texel — see VIS-3.
    float ndThreshold  = 0.6f;
    float ndPrescale   = 1.0f;
    float ndScale      = 5.0f;
    float ndSaturation = 0.7f;
};

// Tunable bounds live in RenderConfig.h as `PostRange`, matching the
// WaterRange contract: the YAML clamp is authoritative and the debug console
// follows it, so the console can never offer a value the config cannot persist.

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

    uint16_t width       = 0;
    uint16_t height      = 0;
    uint16_t bloomWidth  = 0;
    uint16_t bloomHeight = 0;
    int      msaaSamples = 0;   // 0 = off; what was actually achieved

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
inline bool pickDepthFormat(bgfx::TextureFormat::Enum &outFormat,
                            const char *&outName) {
    struct Candidate { bgfx::TextureFormat::Enum fmt; const char *name; };
    const Candidate candidates[] = {
        { bgfx::TextureFormat::D24S8, "D24S8" },
        { bgfx::TextureFormat::D32F,  "D32F"  },
        { bgfx::TextureFormat::D24,   "D24"   },
        { bgfx::TextureFormat::D16,   "D16"   },
    };
    for (const Candidate &c : candidates) {
        if (bgfx::isTextureValid(0, false, 1, c.fmt, BGFX_TEXTURE_RT_WRITE_ONLY)) {
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
inline bool createPostProcess(PostProcessResources &out,
                              uint16_t width, uint16_t height,
                              bgfx::ProgramHandle compositeProgram,
                              bgfx::ProgramHandle blurProgram,
                              bgfx::ProgramHandle extractProgram,
                              int msaaSamples) {
    out.width  = width;
    out.height = height;
    out.compositeProgram = compositeProgram;
    out.blurProgram      = blurProgram;
    out.extractProgram   = extractProgram;

    if (!bgfx::isValid(compositeProgram)) {
        std::fprintf(stderr,
            "[FALLBACK] postprocess: composite program failed to build — "
            "post-processing disabled, rendering direct to backbuffer.\n");
        return false;
    }

    const bgfx::TextureFormat::Enum colorFmt = pickSceneColorFormat(out.hdrCapable);

    bgfx::TextureFormat::Enum depthFmt = bgfx::TextureFormat::D24S8;
    const char *depthName = "unknown";
    if (!pickDepthFormat(depthFmt, depthName)) {
        std::fprintf(stderr,
            "[FALLBACK] postprocess: no supported depth render-target format — "
            "post-processing disabled, rendering direct to backbuffer.\n");
        return false;
    }

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
    // MSAA lives on this target, not the backbuffer: once the scene renders
    // offscreen, backbuffer multisampling would only ever anti-alias the
    // composite triangle's own screen-edge, which has no visible geometry
    // on it. Both attachments must carry the same sample count or the
    // framebuffer is invalid. bgfx resolves automatically when the colour
    // texture is later sampled by the composite.
    uint64_t rtFlag = msaaTextureFlag(msaaSamples);
    if (msaaSamples > 0 &&
        !bgfx::isTextureValid(0, false, 1, colorFmt, rtFlag)) {
        std::fprintf(stderr,
            "[FALLBACK] postprocess: %dx MSAA is unsupported for the scene "
            "target on this backend — continuing without it. Geometry edges "
            "will alias.\n", msaaSamples);
        rtFlag = BGFX_TEXTURE_RT;
        msaaSamples = 0;
    }
    out.msaaSamples = msaaSamples;

    const uint64_t colorFlags = rtFlag
                              | BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP;

    bgfx::TextureHandle attachments[2];
    attachments[0] = bgfx::createTexture2D(width, height, false, 1, colorFmt, colorFlags);
    attachments[1] = bgfx::createTexture2D(width, height, false, 1, depthFmt,
                                           rtFlag | BGFX_TEXTURE_RT_WRITE_ONLY);

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
    }

    // Composite targets the backbuffer and never clears — it overwrites every
    // pixel it covers, and the triangle covers all of them.
    bgfx::setViewName(kViewComposite, "composite");
    bgfx::setViewClear(kViewComposite, BGFX_CLEAR_NONE, 0, 1.0f, 0);
    bgfx::setViewRect(kViewComposite, 0, 0, width, height);
    bgfx::setViewFrameBuffer(kViewComposite, BGFX_INVALID_HANDLE);

    std::fprintf(stderr,
        "Post-process: %ux%u scene target, colour=%s depth=%s msaa=%dx, "
        "backend=%s\n",
        width, height,
        out.hdrCapable ? "RGBA16F" : "RGBA8 (no HDR)",
        depthName,
        out.msaaSamples > 0 ? out.msaaSamples : 1,
        bgfx::getRendererName(bgfx::getRendererType()));

    return true;
}

inline void destroyPostProcess(PostProcessResources &pp) {
    // sceneColor / bloomTex are owned by their framebuffers (created with
    // destroyTextures=true), so they must not be destroyed separately.
    if (bgfx::isValid(pp.frameBuffer))      bgfx::destroy(pp.frameBuffer);
    if (bgfx::isValid(pp.bloomFB[0]))       bgfx::destroy(pp.bloomFB[0]);
    if (bgfx::isValid(pp.bloomFB[1]))       bgfx::destroy(pp.bloomFB[1]);
    if (bgfx::isValid(pp.triangleVBH))      bgfx::destroy(pp.triangleVBH);
    if (bgfx::isValid(pp.compositeProgram)) bgfx::destroy(pp.compositeProgram);
    if (bgfx::isValid(pp.blurProgram))      bgfx::destroy(pp.blurProgram);
    if (bgfx::isValid(pp.extractProgram))   bgfx::destroy(pp.extractProgram);
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
    pp = PostProcessResources{};
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
    return true;
}

/// Resolve the scene target to the backbuffer through the composite shader.
/// `bloomActive` must be the return value of renderBloom() for this frame.
inline void submitComposite(const PostProcessResources &pp,
                            const PostProcessSettings &settings,
                            bool bloomActive) {
    if (!pp.valid())
        return;

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

    bgfx::setUniform(pp.u_ccParams0,   params0);
    bgfx::setUniform(pp.u_ccParams1,   params1);
    bgfx::setUniform(pp.u_ccFilter,    filter);
    bgfx::setUniform(pp.u_lumaWeights, lumaWeights);
    bgfx::setUniform(pp.u_bloomParams, bloomParams);
    bgfx::setUniform(pp.u_bloomStyle,  bloomStyle);
    bgfx::setTexture(0, pp.s_texScene, pp.sceneColor);

    // The bloom sampler must always have a valid texture bound even when
    // bloom is off, or backends that validate bindings will complain; the
    // w=0 above is what actually neutralises it. Fall back to the scene
    // texture when no bloom target exists.
    bgfx::TextureHandle bloomTex = pp.sceneColor;
    if (bgfx::isValid(pp.bloomTex[1]))
        bloomTex = pp.bloomTex[1];
    bgfx::setTexture(1, pp.s_texBloom, bloomTex);

    bgfx::setVertexBuffer(0, pp.triangleVBH);
    // No depth test, no depth write, no culling: the triangle is already in
    // clip space and covers the frame.
    bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A);
    bgfx::submit(kViewComposite, pp.compositeProgram);
}

} // namespace Darkness
