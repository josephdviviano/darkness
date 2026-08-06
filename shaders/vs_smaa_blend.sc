$input a_position, a_texcoord0
$output v_texcoord0, v_offset0

// SMAA pass 3 of 3 — neighborhood blending, vertex stage.
//
// Reference: SMAANeighborhoodBlendingVS. One offset: the right and bottom
// neighbours, whose weights this pixel has to consult because a weight is
// stored on only one side of each edge.

#include <bgfx_shader.sh>
#include "smaa_common.sh"

void main()
{
	gl_Position = vec4(a_position.xy, 0.0, 1.0);

	v_texcoord0 = vec4(a_texcoord0, 0.0, 0.0);

	v_offset0 = smaaMad4(u_smaaMetrics.xyxy, vec4(1.0, 0.0, 0.0, 1.0),
	                     a_texcoord0.xyxy);
}
