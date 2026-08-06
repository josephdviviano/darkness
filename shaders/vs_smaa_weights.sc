$input a_position, a_texcoord0
$output v_texcoord0, v_offset0, v_offset1, v_offset2

// SMAA pass 2 of 3 — blending weight calculation, vertex stage.
//
// Reference: SMAABlendingWeightCalculationVS.
//
// v_texcoord0.xy is the UV; .zw is the same point in PIXELS, which the
// fragment stage needs to turn a search result back into a distance.
//
// v_offset0 / v_offset1 are the quarter-texel-biased start points for the
// horizontal and vertical searches — the bias is what lets the search read two
// edge samples per fetch through the bilinear unit. v_offset2 carries the far
// ends of both searches, so the fragment stage has a bound to stop at rather
// than a step count to track.

#include <bgfx_shader.sh>
#include "smaa_common.sh"

void main()
{
	gl_Position = vec4(a_position.xy, 0.0, 1.0);

	v_texcoord0 = vec4(a_texcoord0, a_texcoord0 * u_smaaMetrics.zw);

	v_offset0 = smaaMad4(u_smaaMetrics.xyxy, vec4(-0.25, -0.125, 1.25, -0.125),
	                     a_texcoord0.xyxy);
	v_offset1 = smaaMad4(u_smaaMetrics.xyxy, vec4(-0.125, -0.25, -0.125, 1.25),
	                     a_texcoord0.xyxy);
	v_offset2 = smaaMad4(u_smaaMetrics.xxyy,
	                     vec4(-2.0, 2.0, -2.0, 2.0)
	                         * float(SMAA_MAX_SEARCH_STEPS),
	                     vec4(v_offset0.xz, v_offset1.yw));
}
