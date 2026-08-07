$input a_position, a_texcoord0
$output v_lumelUV

#include <bgfx_shader.sh>

// S2 lumel-bake pass (PLAN.HIGH_RES_SHADOWS "S2 promoted"): quads cover
// packed rects of a scratch RT; each fragment is one lumel. The lumel-to-
// world mapping is affine and rides fs uniforms — no bake mesh needed for
// the per-poly event path.
void main()
{
    gl_Position = mul(u_modelViewProj, vec4(a_position, 1.0));
    v_lumelUV = a_texcoord0;
}
