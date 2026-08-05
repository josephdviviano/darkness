$input a_position, a_texcoord0
$output v_texcoord0

// Fullscreen-triangle vertex shader for the composite pass.
//
// Positions arrive already in clip space (see buildFullscreenTriangle() in
// PostProcess.h), so there is no transform to apply — this is a passthrough.
// Depth is pinned to 0.0 because the composite pass runs with depth test and
// depth write both disabled.

#include <bgfx_shader.sh>

void main()
{
	gl_Position = vec4(a_position.xy, 0.0, 1.0);
	v_texcoord0 = a_texcoord0;
}
