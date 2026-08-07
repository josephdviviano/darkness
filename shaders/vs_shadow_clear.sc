$input a_position

#include <bgfx_shader.sh>

// Tile clear for the single-view shadow atlas: a unit quad pushed
// through the tile-placement remap alone (u_shadowFaceMtx carries no
// projection here), at far depth with DEPTH_TEST_ALWAYS — resets one
// tile's depth and stored distance without touching the rest of the
// atlas. Per-view clears died with the single-view refactor; this is
// their replacement.
uniform mat4 u_shadowFaceMtx;

void main()
{
    vec4 p = mul(u_shadowFaceMtx, vec4(a_position.xy, 0.0, 1.0));
    // z = w: the far plane in both clip conventions (GL -1..1 and ZO
    // 0..1 both place far at ndc z = +1).
    gl_Position = vec4(p.x, p.y, p.w, p.w);
}
