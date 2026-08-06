$input v_texcoord0, v_offset0

// SMAA pass 3 of 3 — neighborhood blending.
//
// Reference: SMAANeighborhoodBlendingPS. Gathers the four weights that apply
// to this pixel — two of them stored on its right and bottom neighbours — and
// resolves the final colour as a two-tap blend along whichever axis won.
//
// The four weights are read as a.x (right neighbour), a.y (bottom neighbour),
// a.z (top) and a.w (left), which is the packing the weight pass wrote.
//
// No colour-space conversion, deliberately. See smaa_common.sh.
//
// This pass is also where the debug views live: they need the same fullscreen
// geometry and the same two source textures, so a separate program would be
// three more shader variants to keep in step for no benefit. The branch is on
// a uniform, so it costs a scalar compare per pixel and nothing else.

#include <bgfx_shader.sh>
#include "smaa_common.sh"

SAMPLER2D(s_smaaColor,   0);
SAMPLER2D(s_smaaWeights, 1);
SAMPLER2D(s_smaaEdges,   2);

// u_smaaParams.y = debug view: 0 = off, 1 = edges, 2 = weights
uniform vec4 u_smaaParams;

void main()
{
	vec2 uv = v_texcoord0.xy;

	// ── Debug views ──
	// Edges: red = a left edge, green = a top edge. Weights: the raw
	// blending weights, which read as thin coloured ribbons along every
	// antialiased silhouette. Both are on the same "is SMAA seeing anything"
	// question — an all-black edge view means the threshold is too high.
	if (u_smaaParams.y > 0.5) {
		if (u_smaaParams.y < 1.5) {
			gl_FragColor = vec4(texture2DLod(s_smaaEdges, uv, 0.0).rg,
			                    0.0, 1.0);
		} else {
			gl_FragColor = vec4(texture2DLod(s_smaaWeights, uv, 0.0).rgb, 1.0);
		}
		return;
	}

	vec4 a;
	a.x = texture2DLod(s_smaaWeights, v_offset0.xy, 0.0).a;  // right
	a.y = texture2DLod(s_smaaWeights, v_offset0.zw, 0.0).g;  // bottom
	a.wz = texture2DLod(s_smaaWeights, uv, 0.0).xz;          // left, top

	// No weight reaches this pixel — pass the colour through untouched. This
	// is the majority of the screen, and keeping it a pure copy is what makes
	// SMAA cost a flat fraction of a millisecond rather than a full blend.
	if (dot(a, vec4(1.0, 1.0, 1.0, 1.0)) < 1e-5) {
		gl_FragColor = texture2DLod(s_smaaColor, uv, 0.0);
		return;
	}

	// Pick the axis with the stronger pair of weights, then blend along it.
	float h = float(max(a.x, a.z) > max(a.y, a.w));

	vec4 blendingOffset = vec4(0.0, a.y, 0.0, a.w);
	vec2 blendingWeight = a.yw;

	blendingOffset = smaaMovc4(vec4(h, h, h, h), blendingOffset,
	                           vec4(a.x, 0.0, a.z, 0.0));
	blendingWeight = smaaMovc2(vec2(h, h), blendingWeight, a.xz);
	blendingWeight /= dot(blendingWeight, vec2(1.0, 1.0));

	vec4 blendingCoord = smaaMad4(blendingOffset,
	                              vec4(u_smaaMetrics.xy, -u_smaaMetrics.xy),
	                              vec4(uv, uv));

	vec4 color = blendingWeight.x * texture2DLod(s_smaaColor,
	                                             blendingCoord.xy, 0.0);
	color += blendingWeight.y * texture2DLod(s_smaaColor,
	                                         blendingCoord.zw, 0.0);

	gl_FragColor = color;
}
