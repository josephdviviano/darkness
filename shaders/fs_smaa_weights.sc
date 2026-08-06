$input v_texcoord0, v_offset0, v_offset1, v_offset2

// SMAA pass 2 of 3 — blending weight calculation.
//
// Reference: SMAABlendingWeightCalculationPS. This is where SMAA earns its
// name: for each edge pixel it works out how far the edge runs in both
// directions, classifies the shape at each end (the "crossing edges"), and
// looks the pair up in AreaTex to get the exact coverage area the revectorised
// silhouette would have had. That is a table lookup of a geometric answer, not
// a blur — which is why SMAA keeps 1-pixel detail that a filter would smear.
//
// Output packs the four weights the blending pass reads:
//   rg = left / top contribution, ba = right / bottom contribution.
//
// SMAA 1x, so the subsample offsets are all zero and AreaTex block 0 is the
// only one read. The other six blocks exist for S2x/T2x/4x; leaving the
// offsets in as named constants keeps this a translation of the reference
// rather than a fork of it, which matters if TAA later brings T2x within
// reach.

#include <bgfx_shader.sh>
#include "smaa_common.sh"

SAMPLER2D(s_smaaEdges,  0);
SAMPLER2D(s_smaaArea,   1);
SAMPLER2D(s_smaaSearch, 2);

// SMAA 1x reads a single, unjittered sample, so every subsample index is 0.
#define SMAA_SUBSAMPLE_INDICES vec4(0.0, 0.0, 0.0, 0.0)

// ── Diagonal search ──
//
// Diagonal patterns are handled before the orthogonal ones because a diagonal
// edge would otherwise be classified as a staircase of tiny horizontal and
// vertical runs, and revectorised as such.

vec2 smaaDecodeDiagBilinearAccess2(vec2 e) {
	// The bilinear fetch below reads two edge texels at once; this undoes the
	// interpolation to recover which of the two carried the edge.
	e.r = e.r * abs(5.0 * e.r - 5.0 * 0.75);
	return smaaRound2(e);
}

vec4 smaaDecodeDiagBilinearAccess4(vec4 e) {
	e.rb = e.rb * abs(5.0 * e.rb - 5.0 * 0.75);
	return smaaRound4(e);
}

// Walk along a diagonal until the edge stops. Returns (steps taken, last
// edge sample) and reports the sample it stopped on through `e`.
//
// The loop is a bounded `for` with the reference's `while` condition tested
// inside it. Same iteration count — the counter in coord.z cannot exceed the
// bound — but a compile-time trip limit, which is what the HLSL and Metal
// compilers need to generate a loop at all.
vec2 smaaSearchDiag1(vec2 texcoord, vec2 dir, out vec2 e) {
	e = vec2(0.0, 0.0);
	vec4 coord = vec4(texcoord, -1.0, 1.0);
	vec3 t = vec3(u_smaaMetrics.xy, 1.0);
	for (int i = 0; i < SMAA_MAX_SEARCH_STEPS_DIAG; ++i) {
		if (coord.z >= float(SMAA_MAX_SEARCH_STEPS_DIAG - 1) || coord.w <= 0.9)
			break;
		coord.xyz = t * vec3(dir, 1.0) + coord.xyz;
		e = texture2DLod(s_smaaEdges, coord.xy, 0.0).rg;
		coord.w = dot(e, vec2(0.5, 0.5));
	}
	return coord.zw;
}

vec2 smaaSearchDiag2(vec2 texcoord, vec2 dir, out vec2 e) {
	e = vec2(0.0, 0.0);
	vec4 coord = vec4(texcoord, -1.0, 1.0);
	coord.x += 0.25 * u_smaaMetrics.x;   // bias into the bilinear pair
	vec3 t = vec3(u_smaaMetrics.xy, 1.0);
	for (int i = 0; i < SMAA_MAX_SEARCH_STEPS_DIAG; ++i) {
		if (coord.z >= float(SMAA_MAX_SEARCH_STEPS_DIAG - 1) || coord.w <= 0.9)
			break;
		coord.xyz = t * vec3(dir, 1.0) + coord.xyz;
		e = texture2DLod(s_smaaEdges, coord.xy, 0.0).rg;
		e = smaaDecodeDiagBilinearAccess2(e);
		coord.w = dot(e, vec2(0.5, 0.5));
	}
	return coord.zw;
}

vec2 smaaAreaDiag(vec2 dist, vec2 e, float offset) {
	vec2 coord = smaaMad2(vec2(SMAA_AREATEX_MAX_DISTANCE_DIAG,
	                           SMAA_AREATEX_MAX_DISTANCE_DIAG), e, dist);
	coord = smaaMad2(SMAA_AREATEX_PIXEL_SIZE, coord,
	                 0.5 * SMAA_AREATEX_PIXEL_SIZE);
	coord.x += 0.5;                                   // diagonal half of the table
	coord.y += SMAA_AREATEX_SUBTEX_SIZE * offset;     // subsample block
	return SMAA_AREATEX_SELECT(texture2DLod(s_smaaArea, coord, 0.0));
}

vec2 smaaCalculateDiagWeights(vec2 texcoord, vec2 e, vec4 subsampleIndices) {
	vec2 weights = vec2(0.0, 0.0);

	vec4 d;
	vec2 end;
	if (e.r > 0.0) {
		d.xz = smaaSearchDiag1(texcoord, vec2(-1.0, 1.0), end);
		d.x += float(end.y > 0.9);
	} else {
		d.xz = vec2(0.0, 0.0);
	}
	d.yw = smaaSearchDiag1(texcoord, vec2(1.0, -1.0), end);

	if (d.x + d.y > 2.0) {   // d.x + d.y + 1 > 3, i.e. a run worth shaping
		vec4 coords = smaaMad4(vec4(-d.x + 0.25, d.x, d.y, -d.y - 0.25),
		                       u_smaaMetrics.xyxy, texcoord.xyxy);
		vec4 c;
		c.xy = SMAA_SAMPLE_OFFSET(s_smaaEdges, coords.xy, -1.0, 0.0).rg;
		c.zw = SMAA_SAMPLE_OFFSET(s_smaaEdges, coords.zw,  1.0, 0.0).rg;
		c.yxwz = smaaDecodeDiagBilinearAccess4(c.xyzw);

		vec2 cc = smaaMad2(vec2(2.0, 2.0), c.xz, c.yw);
		cc = smaaMovc2(step(0.9, d.zw), cc, vec2(0.0, 0.0));

		weights += smaaAreaDiag(d.xy, cc, subsampleIndices.z);
	}

	d.xz = smaaSearchDiag2(texcoord, vec2(-1.0, -1.0), end);
	if (SMAA_SAMPLE_OFFSET(s_smaaEdges, texcoord, 1.0, 0.0).r > 0.0) {
		d.yw = smaaSearchDiag2(texcoord, vec2(1.0, 1.0), end);
		d.y += float(end.y > 0.9);
	} else {
		d.yw = vec2(0.0, 0.0);
	}

	if (d.x + d.y > 2.0) {
		vec4 coords = smaaMad4(vec4(-d.x, -d.x, d.y, d.y),
		                       u_smaaMetrics.xyxy, texcoord.xyxy);
		vec4 c;
		c.x = SMAA_SAMPLE_OFFSET(s_smaaEdges, coords.xy, -1.0,  0.0).g;
		c.y = SMAA_SAMPLE_OFFSET(s_smaaEdges, coords.xy,  0.0, -1.0).r;
		c.zw = SMAA_SAMPLE_OFFSET(s_smaaEdges, coords.zw, 1.0, 0.0).gr;
		vec2 cc = smaaMad2(vec2(2.0, 2.0), c.xz, c.yw);
		cc = smaaMovc2(step(0.9, d.zw), cc, vec2(0.0, 0.0));

		weights += smaaAreaDiag(d.xy, cc, subsampleIndices.w).gr;
	}

	return weights;
}

// ── Orthogonal search ──

// Decode a run length from SearchTex. The table is a packed 66x33 logical
// grid; the scale/bias arithmetic is what maps an edge pair onto it.
float smaaSearchLength(vec2 e, float offset) {
	vec2 scale = SMAA_SEARCHTEX_SIZE * vec2(0.5, -1.0);
	vec2 bias  = SMAA_SEARCHTEX_SIZE * vec2(offset, 1.0);

	scale += vec2(-1.0,  1.0);
	bias  += vec2( 0.5, -0.5);

	scale *= 1.0 / SMAA_SEARCHTEX_PACKED_SIZE;
	bias  *= 1.0 / SMAA_SEARCHTEX_PACKED_SIZE;

	return SMAA_SEARCHTEX_SELECT(
	    texture2DLod(s_smaaSearch, smaaMad2(scale, e, bias), 0.0));
}

// Each step covers TWO pixels: the sample sits between texels so the bilinear
// unit averages a pair, and 0.8281 is the threshold that says "both were
// edges". That is what makes a 32-step search reach 64 pixels.
float smaaSearchXLeft(vec2 texcoord, float end) {
	vec2 e = vec2(0.0, 1.0);
	for (int i = 0; i < SMAA_MAX_SEARCH_STEPS + 1; ++i) {
		if (!(texcoord.x > end && e.g > 0.8281 && e.r == 0.0))
			break;
		e = texture2DLod(s_smaaEdges, texcoord, 0.0).rg;
		texcoord = -vec2(2.0, 0.0) * u_smaaMetrics.xy + texcoord;
	}
	float offset = -(255.0 / 127.0) * smaaSearchLength(e, 0.0) + 3.25;
	return u_smaaMetrics.x * offset + texcoord.x;
}

float smaaSearchXRight(vec2 texcoord, float end) {
	vec2 e = vec2(0.0, 1.0);
	for (int i = 0; i < SMAA_MAX_SEARCH_STEPS + 1; ++i) {
		if (!(texcoord.x < end && e.g > 0.8281 && e.r == 0.0))
			break;
		e = texture2DLod(s_smaaEdges, texcoord, 0.0).rg;
		texcoord = vec2(2.0, 0.0) * u_smaaMetrics.xy + texcoord;
	}
	float offset = -(255.0 / 127.0) * smaaSearchLength(e, 0.5) + 3.25;
	return -u_smaaMetrics.x * offset + texcoord.x;
}

float smaaSearchYUp(vec2 texcoord, float end) {
	vec2 e = vec2(1.0, 0.0);
	for (int i = 0; i < SMAA_MAX_SEARCH_STEPS + 1; ++i) {
		if (!(texcoord.y > end && e.r > 0.8281 && e.g == 0.0))
			break;
		e = texture2DLod(s_smaaEdges, texcoord, 0.0).rg;
		texcoord = -vec2(0.0, 2.0) * u_smaaMetrics.xy + texcoord;
	}
	float offset = -(255.0 / 127.0) * smaaSearchLength(e.gr, 0.0) + 3.25;
	return u_smaaMetrics.y * offset + texcoord.y;
}

float smaaSearchYDown(vec2 texcoord, float end) {
	vec2 e = vec2(1.0, 0.0);
	for (int i = 0; i < SMAA_MAX_SEARCH_STEPS + 1; ++i) {
		if (!(texcoord.y < end && e.r > 0.8281 && e.g == 0.0))
			break;
		e = texture2DLod(s_smaaEdges, texcoord, 0.0).rg;
		texcoord = vec2(0.0, 2.0) * u_smaaMetrics.xy + texcoord;
	}
	float offset = -(255.0 / 127.0) * smaaSearchLength(e.gr, 0.5) + 3.25;
	return -u_smaaMetrics.y * offset + texcoord.y;
}

// Look the (distance, crossing-edge) pair up in AreaTex. `dist` arrives as
// sqrt(distance) because the table is stored with that spacing — near the
// edge's ends, where the coverage changes fastest, it gets the most entries.
vec2 smaaArea(vec2 dist, float e1, float e2, float offset) {
	vec2 texcoord = smaaMad2(
	    vec2(SMAA_AREATEX_MAX_DISTANCE, SMAA_AREATEX_MAX_DISTANCE),
	    smaaRound2(4.0 * vec2(e1, e2)), dist);

	texcoord = smaaMad2(SMAA_AREATEX_PIXEL_SIZE, texcoord,
	                    0.5 * SMAA_AREATEX_PIXEL_SIZE);
	texcoord.y = SMAA_AREATEX_SUBTEX_SIZE * offset + texcoord.y;

	return SMAA_AREATEX_SELECT(texture2DLod(s_smaaArea, texcoord, 0.0));
}

// Corner rounding. Without it, a 90-degree corner gets antialiased as two
// independent edges and comes out visibly clipped — the reference's
// SMAA_CORNER_ROUNDING is how much of that clipping to give back.
void smaaDetectHorizontalCornerPattern(inout vec2 weights, vec4 coord, vec2 d) {
	vec2 leftRight = step(d.xy, d.yx);
	vec2 rounding = (1.0 - SMAA_CORNER_ROUNDING_NORM) * leftRight;
	rounding /= leftRight.x + leftRight.y;

	vec2 factor = vec2(1.0, 1.0);
	factor.x -= rounding.x * SMAA_SAMPLE_OFFSET(s_smaaEdges, coord.xy, 0.0,  1.0).r;
	factor.x -= rounding.y * SMAA_SAMPLE_OFFSET(s_smaaEdges, coord.zw, 1.0,  1.0).r;
	factor.y -= rounding.x * SMAA_SAMPLE_OFFSET(s_smaaEdges, coord.xy, 0.0, -2.0).r;
	factor.y -= rounding.y * SMAA_SAMPLE_OFFSET(s_smaaEdges, coord.zw, 1.0, -2.0).r;

	weights *= clamp(factor, 0.0, 1.0);
}

void smaaDetectVerticalCornerPattern(inout vec2 weights, vec4 coord, vec2 d) {
	vec2 leftRight = step(d.xy, d.yx);
	vec2 rounding = (1.0 - SMAA_CORNER_ROUNDING_NORM) * leftRight;
	rounding /= leftRight.x + leftRight.y;

	vec2 factor = vec2(1.0, 1.0);
	factor.x -= rounding.x * SMAA_SAMPLE_OFFSET(s_smaaEdges, coord.xy,  1.0, 0.0).g;
	factor.x -= rounding.y * SMAA_SAMPLE_OFFSET(s_smaaEdges, coord.zw,  1.0, 1.0).g;
	factor.y -= rounding.x * SMAA_SAMPLE_OFFSET(s_smaaEdges, coord.xy, -2.0, 0.0).g;
	factor.y -= rounding.y * SMAA_SAMPLE_OFFSET(s_smaaEdges, coord.zw, -2.0, 1.0).g;

	weights *= clamp(factor, 0.0, 1.0);
}

void main()
{
	vec4 weights = vec4(0.0, 0.0, 0.0, 0.0);
	vec2 uv       = v_texcoord0.xy;
	vec2 pixcoord = v_texcoord0.zw;
	vec4 subsampleIndices = SMAA_SUBSAMPLE_INDICES;

	vec2 e = texture2DLod(s_smaaEdges, uv, 0.0).rg;

	if (e.g > 0.0) {   // Edge at north.
		weights.rg = smaaCalculateDiagWeights(uv, e, subsampleIndices);

		// A diagonal pattern was found, so the orthogonal search would only
		// fight it — the reference signals "nothing found" as r == -g, which
		// can only hold when both are zero.
		if (weights.r == -weights.g) {
			vec2 d;
			vec3 coords;

			coords.x = smaaSearchXLeft(v_offset0.xy, v_offset2.x);
			coords.y = v_offset1.y;
			d.x = coords.x;

			// The crossing edge at the left end of the run.
			float e1 = texture2DLod(s_smaaEdges, coords.xy, 0.0).r;

			coords.z = smaaSearchXRight(v_offset0.zw, v_offset2.y);
			d.y = coords.z;

			// Back from UV to pixels, which is what the area table indexes.
			d = abs(smaaRound2(smaaMad2(u_smaaMetrics.zz, d, -pixcoord.xx)));

			vec2 sqrtD = sqrt(d);

			float e2 = SMAA_SAMPLE_OFFSET(s_smaaEdges, coords.zy, 1.0, 0.0).r;

			weights.rg = smaaArea(sqrtD, e1, e2, subsampleIndices.y);

			// Through a temporary rather than passing weights.rg directly:
			// a swizzle is a legal inout argument in GLSL, but not every
			// backend bgfx cross-compiles to agrees, and a silently dropped
			// write-back here would just look like corner rounding is off.
			coords.y = uv.y;
			vec2 hWeights = weights.rg;
			smaaDetectHorizontalCornerPattern(hWeights, coords.xyzy, d);
			weights.rg = hWeights;
		} else {
			e.r = 0.0;   // Diagonal handled it; skip the vertical pass.
		}
	}

	if (e.r > 0.0) {   // Edge at west.
		vec2 d;
		vec3 coords;

		coords.y = smaaSearchYUp(v_offset1.xy, v_offset2.z);
		coords.x = v_offset0.x;
		d.x = coords.y;

		float e1 = texture2DLod(s_smaaEdges, coords.xy, 0.0).g;

		coords.z = smaaSearchYDown(v_offset1.zw, v_offset2.w);
		d.y = coords.z;

		d = abs(smaaRound2(smaaMad2(u_smaaMetrics.ww, d, -pixcoord.yy)));

		vec2 sqrtD = sqrt(d);

		float e2 = SMAA_SAMPLE_OFFSET(s_smaaEdges, coords.xz, 0.0, 1.0).g;

		weights.ba = smaaArea(sqrtD, e1, e2, subsampleIndices.x);

		coords.x = uv.x;
		vec2 vWeights = weights.ba;
		smaaDetectVerticalCornerPattern(vWeights, coords.xyxz, d);
		weights.ba = vWeights;
	}

	gl_FragColor = weights;
}
