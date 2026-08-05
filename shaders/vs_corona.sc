$input a_position, a_color0, a_texcoord0
$output v_color0, v_texcoord0, v_fogDist, v_viewZ

#include <bgfx_shader.sh>

// Light corona billboard. The quad arrives already oriented — CoronaSystem
// builds it in world space from the camera basis — so this is a plain
// transform. a_color0 is a FLOAT4 attribute rather than the usual normalized
// uint8: the per-corona tint routinely exceeds 1.0, which is what carries the
// glow into the overbright range that bloom responds to, and a packed 8-bit
// colour could not express it.
void main()
{
    gl_Position = mul(u_modelViewProj, vec4(a_position, 1.0));
    v_color0 = a_color0;
    v_texcoord0 = a_texcoord0;

    vec3 viewPos = mul(u_modelView, vec4(a_position, 1.0)).xyz;
    // Radial distance for fog, matching what every other pass feeds its fog.
    v_fogDist = length(viewPos);
    // Distance ALONG the view axis for the depth comparison. That is the
    // measure a depth buffer stores, so the two must agree — using the radial
    // distance here would over-report depth toward the screen edges and fade
    // coronas that nothing is occluding. Right-handed view space looks down
    // -Z, hence the negation.
    v_viewZ = -viewPos.z;
}
