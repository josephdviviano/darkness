$input a_position
$output v_worldPos

#include <bgfx_shader.sh>

// Shadow-face depth pass (S1, PLAN.HIGH_RES_SHADOWS.md): the per-view
// transform is one omni face's view-projection (ShadowFaceMath.h); world
// geometry is authored in world space, so a_position doubles as the world
// position the fragment stage measures distance from.
void main()
{
    gl_Position = mul(u_modelViewProj, vec4(a_position, 1.0));
    v_worldPos = a_position;
}
