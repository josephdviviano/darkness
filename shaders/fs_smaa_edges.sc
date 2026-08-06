$input v_texcoord0, v_offset0, v_offset1, v_offset2

// SMAA pass 1 of 3 — colour edge detection.
//
// Reference: SMAAColorEdgeDetectionPS. Colour rather than luma detection,
// matching Godot's choice: luma detection misses edges between two colours of
// equal brightness, which in this engine means the boundary between a lit
// stone wall and a lit wooden beam of the same value — common in the source
// art, and exactly the silhouette the player is trying to read in the dark.
//
// Output is a two-channel edge mask: r = an edge on this pixel's left side,
// g = an edge on its top side. Every later pass reads that convention.
//
// The local contrast adaptation at the end is what stops SMAA rounding off
// texture detail: an edge is discarded when a *neighbouring* edge is much
// stronger, so only the dominant discontinuity in a neighbourhood survives.

#include <bgfx_shader.sh>
#include "smaa_common.sh"

SAMPLER2D(s_smaaColor, 0);

// u_smaaParams.x = edge-detection threshold
uniform vec4 u_smaaParams;

void main()
{
	vec2 threshold = vec2(u_smaaParams.x, u_smaaParams.x);
	vec2 uv = v_texcoord0.xy;

	// Delta to the left and top neighbours, as the largest per-channel
	// difference — the "colour" in colour edge detection.
	vec4 delta;
	vec3 C = texture2DLod(s_smaaColor, uv, 0.0).rgb;

	vec3 Cleft = texture2DLod(s_smaaColor, v_offset0.xy, 0.0).rgb;
	vec3 t = abs(C - Cleft);
	delta.x = max(max(t.r, t.g), t.b);

	vec3 Ctop = texture2DLod(s_smaaColor, v_offset0.zw, 0.0).rgb;
	t = abs(C - Ctop);
	delta.y = max(max(t.r, t.g), t.b);

	vec2 edges = step(threshold, delta.xy);

	// Nothing crossed the threshold — leave the target's cleared zero rather
	// than writing it, which is the reference's early-out.
	if (dot(edges, vec2(1.0, 1.0)) == 0.0)
		discard;

	// Right and bottom neighbours, for the contrast adaptation below.
	vec3 Cright = texture2DLod(s_smaaColor, v_offset1.xy, 0.0).rgb;
	t = abs(C - Cright);
	delta.z = max(max(t.r, t.g), t.b);

	vec3 Cbottom = texture2DLod(s_smaaColor, v_offset1.zw, 0.0).rgb;
	t = abs(C - Cbottom);
	delta.w = max(max(t.r, t.g), t.b);

	vec2 maxDelta = max(delta.xy, delta.zw);

	// ...and the neighbours one step further out, so the comparison sees a
	// two-pixel neighbourhood rather than an immediate one.
	vec3 Cleftleft = texture2DLod(s_smaaColor, v_offset2.xy, 0.0).rgb;
	t = abs(Cleft - Cleftleft);
	delta.z = max(max(t.r, t.g), t.b);

	vec3 Ctoptop = texture2DLod(s_smaaColor, v_offset2.zw, 0.0).rgb;
	t = abs(Ctop - Ctoptop);
	delta.w = max(max(t.r, t.g), t.b);

	maxDelta = max(maxDelta.xy, delta.zw);
	float finalDelta = max(maxDelta.x, maxDelta.y);

	edges.xy *= step(finalDelta,
	                 SMAA_LOCAL_CONTRAST_ADAPTATION_FACTOR * delta.xy);

	gl_FragColor = vec4(edges, 0.0, 1.0);
}
