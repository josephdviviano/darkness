$input a_position, a_texcoord0
$output v_texcoord0, v_offset0, v_offset1, v_offset2

// SMAA pass 1 of 3 — edge detection, vertex stage.
//
// Reference: SMAAEdgeDetectionVS. The three offsets are the left/top,
// right/bottom and left-left/top-top neighbours; precomputing them here rather
// than in the fragment shader is what lets the texture units prefetch, and is
// the reason the reference splits the pass this way.
//
// Positions arrive already in clip space from the shared fullscreen triangle
// (buildFullscreenTriangle() in PostProcess.h), so there is no transform.

#include <bgfx_shader.sh>
#include "smaa_common.sh"

void main()
{
	gl_Position = vec4(a_position.xy, 0.0, 1.0);

	v_texcoord0 = vec4(a_texcoord0, 0.0, 0.0);

	v_offset0 = smaaMad4(u_smaaMetrics.xyxy, vec4(-1.0, 0.0, 0.0, -1.0),
	                     a_texcoord0.xyxy);
	v_offset1 = smaaMad4(u_smaaMetrics.xyxy, vec4( 1.0, 0.0, 0.0,  1.0),
	                     a_texcoord0.xyxy);
	v_offset2 = smaaMad4(u_smaaMetrics.xyxy, vec4(-2.0, 0.0, 0.0, -2.0),
	                     a_texcoord0.xyxy);
}
