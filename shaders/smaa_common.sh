// Shared SMAA constants and portability helpers.
//
// SMAA (Enhanced Subpixel Morphological Antialiasing), Jimenez et al. 2012.
// The three passes are ports of the reference implementation, MIT licensed:
//
//   Copyright (C) 2013 Jorge Jimenez, Jose I. Echevarria, Belen Masia,
//                      Fernando Navarro, Diego Gutierrez.
//
// Structure and naming follow the GLSL port in Godot 4
// (servers/rendering/renderer_rd/shaders/effects/smaa_*.glsl, MIT), which is
// the same reference expressed in a language close enough to bgfx's dialect
// that divergences below are worth stating explicitly:
//
//   * No sRGB round trip in the blending pass. Godot converts to linear,
//     blends, and converts back. The reference blends in whatever space the
//     input is in, and SMAA is specified to run on the final display-referred
//     image, which is what our composite pass hands it. Adding the round trip
//     would change the weighting of a coverage blend the area table already
//     expresses perceptually.
//   * No stencil optimisation. Godot's edge pass writes stencil so the weight
//     pass can skip non-edge pixels. bgfx exposes stencil per draw rather than
//     as a pipeline state we can attach to a fullscreen pass this cheaply, and
//     the weight pass already early-outs on a zero edge sample.
//
// PORTABILITY: this compiles for GLSL 1.20 (the desktop-GL profile bgfx's
// shader pipeline targets), which has neither round() nor fma() nor the
// offset-sampling entry points. The helpers below are the reference's own
// GLSL 1.10 fallbacks — an explicit multiply-add, and an offset expressed as a
// UV delta — not approximations of them.

#ifndef SMAA_COMMON_SH_HEADER_GUARD
#define SMAA_COMMON_SH_HEADER_GUARD

// Quality: the reference "Ultra" preset, which is also what Godot ships. The
// knobs below are the preset; they are compile-time because two of them bound
// loops. Only the edge-detection threshold is exposed at runtime, because it
// is the only one that is a uniform in the reference too.
#define SMAA_MAX_SEARCH_STEPS      32
#define SMAA_MAX_SEARCH_STEPS_DIAG 16
#define SMAA_CORNER_ROUNDING       25

#define SMAA_LOCAL_CONTRAST_ADAPTATION_FACTOR 2.0
#define SMAA_CORNER_ROUNDING_NORM (float(SMAA_CORNER_ROUNDING) / 100.0)

// AreaTex geometry. SUBTEX_SIZE is 1/7 because the table stacks seven
// 160x80 blocks, one per subsample pattern; SMAA 1x always reads block 0.
#define SMAA_AREATEX_MAX_DISTANCE      16.0
#define SMAA_AREATEX_MAX_DISTANCE_DIAG 20.0
#define SMAA_AREATEX_PIXEL_SIZE  vec2(1.0 / 160.0, 1.0 / 560.0)
#define SMAA_AREATEX_SUBTEX_SIZE (1.0 / 7.0)
#define SMAA_AREATEX_SELECT(s)   (s).rg

// SearchTex is a 66x33 logical table packed into 64x16.
#define SMAA_SEARCHTEX_SIZE        vec2(66.0, 33.0)
#define SMAA_SEARCHTEX_PACKED_SIZE vec2(64.0, 16.0)
#define SMAA_SEARCHTEX_SELECT(s)   (s).r

// u_smaaMetrics = (1/width, 1/height, width, height) of the pass's own target.
// The reference calls this SMAA_RT_METRICS and every offset is expressed in
// it, which is what makes the passes resolution-independent.
uniform vec4 u_smaaMetrics;

// round(): GLSL 1.20 predates it. The reference's own fallback is floor(x+0.5).
// Not merely "close enough": GLSL leaves round()'s behaviour at exactly .5
// implementation-defined, so floor(x+0.5) is one of the permitted answers.
float smaaRound1(float x) { return floor(x + 0.5); }
vec2  smaaRound2(vec2  x) { return floor(x + 0.5); }
vec4  smaaRound4(vec4  x) { return floor(x + 0.5); }

// mad(): written out rather than fma(), which GLSL 1.20 also lacks. The
// reference does not depend on fma's single-rounding — it uses it for the
// instruction, not the precision.
vec4 smaaMad4(vec4 a, vec4 b, vec4 c) { return a * b + c; }
vec2 smaaMad2(vec2 a, vec2 b, vec2 c) { return a * b + c; }

// Conditional move. The reference selects with a bvec; a float 0/1 select via
// mix() is componentwise-identical and avoids bvec, whose translation differs
// across the backends bgfx compiles for.
vec2 smaaMovc2(vec2 cond, vec2 variable, vec2 value) {
	return mix(variable, value, cond);
}
vec4 smaaMovc4(vec4 cond, vec4 variable, vec4 value) {
	return mix(variable, value, cond);
}

// Offset sampling. GLSL 1.20 has no textureLodOffset, so the integer texel
// offset becomes a UV delta — the reference's GLSL 1.10 path exactly. Reads
// u_smaaMetrics from the including shader's scope.
#define SMAA_SAMPLE_OFFSET(_tex, _uv, _ox, _oy) \
	texture2DLod(_tex, (_uv) + vec2(_ox, _oy) * u_smaaMetrics.xy, 0.0)

#endif // SMAA_COMMON_SH_HEADER_GUARD
