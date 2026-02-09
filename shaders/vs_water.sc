$input a_position, a_color0, a_texcoord0
$output v_color0, v_texcoord0, v_fogDist

// Water vertex shader: sine-wave Z displacement + world-space UV computation.
// u_waterParams: x=time, y=scroll_speed, z=wave_amplitude, w=uv_distortion
// u_waterFlow:   x=rotation_speed, y=use_world_uv (0 or 1), z=tex_unit_len
//
// When u_waterFlow.y > 0.5, UVs are computed from world-space vertex position
// with rotation and scrolling (matching the Dark Engine's flow animation).
// Otherwise, pre-computed UVs from the mesh are passed through.

#include <bgfx_shader.sh>

uniform vec4 u_waterParams;
uniform vec4 u_waterFlow;

void main()
{
    float t = u_waterParams.x;
    float amp = u_waterParams.z;

    // Two overlapping sine waves along X and Y, displacing Z (up in Z-up coords)
    vec3 pos = a_position;
    float wave1 = sin(pos.x * 0.08 + pos.y * 0.06 + t * 1.2) * amp;
    float wave2 = sin(pos.x * 0.15 - pos.y * 0.12 + t * 1.8) * amp * 0.4;
    pos.z += wave1 + wave2;

    gl_Position = mul(u_modelViewProj, vec4(pos, 1.0));
    v_color0 = a_color0;

    // UV computation: world-space rotation for flow water, passthrough otherwise
    if (u_waterFlow.y > 0.5) {
        // Flow-textured water: compute UVs from world position with rotation.
        // The Dark Engine builds rotated UV basis vectors in world space:
        //   u_hat = (cos(angle), sin(angle))
        //   v_hat = (sin(angle), -cos(angle))
        // and projects vertex positions onto these axes.
        float angle = t * u_waterFlow.x;
        float texScale = u_waterFlow.z;
        float scrollSpeed = u_waterParams.y;

        // Center scrolls over time (simulates center_change * dt accumulation)
        float scroll = t * scrollSpeed;
        vec2 center = vec2(scroll * 0.3, scroll * 0.7);

        // Rotated basis vectors for texture projection
        vec2 u_hat = vec2(cos(angle), sin(angle));
        vec2 v_hat = vec2(sin(angle), -cos(angle));

        // Project world position onto rotated basis, scaled by tex_unit_len
        vec2 offset = a_position.xy - center;
        v_texcoord0 = vec2(dot(offset, u_hat), dot(offset, v_hat)) / texScale;
    } else {
        // Non-flow water: use pre-computed UVs from mesh
        v_texcoord0 = a_texcoord0;
    }

    // View-space distance for fog
    vec3 viewPos = mul(u_modelView, vec4(pos, 1.0)).xyz;
    v_fogDist = length(viewPos);
}
