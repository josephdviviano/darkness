$input a_position
$output v_worldPos

#include <bgfx_shader.sh>

// Shadow-face depth pass (S1, PLAN.HIGH_RES_SHADOWS.md). ALL faces of
// ALL pool slots render into ONE bgfx view (the 256-view budget capped
// the pool at 15 differentials otherwise): the viewport is the whole
// atlas, and each draw carries its face's view-projection COMPOSED with
// a clip-space tile-placement remap in u_shadowFaceMtx, plus a matching
// per-draw scissor. World geometry is authored in world space, so
// a_position doubles as the world position the fragment stage measures
// distance from.
uniform mat4 u_shadowFaceMtx;

void main()
{
    gl_Position = mul(u_shadowFaceMtx, vec4(a_position, 1.0));
    v_worldPos = a_position;
}
