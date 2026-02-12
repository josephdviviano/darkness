$input a_position, a_texcoord0, a_texcoord1
$output v_texcoord0, v_texcoord1, v_fogDist

#include <bgfx_shader.sh>

void main()
{
    gl_Position = mul(u_modelViewProj, vec4(a_position, 1.0));
    v_texcoord0 = a_texcoord0;
    v_texcoord1 = a_texcoord1;
    // View-space distance for fog — length of the view-space position vector
    vec3 viewPos = mul(u_modelView, vec4(a_position, 1.0)).xyz;
    v_fogDist = length(viewPos);
}
