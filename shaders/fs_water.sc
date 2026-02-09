$input v_color0, v_texcoord0, v_fogDist

// Water fragment shader: texture * vertex color with UV distortion.
// UV rotation and scrolling are handled in the vertex shader (world-space).
// u_waterParams: x=time, y=scroll speed, z=wave amplitude, w=UV distortion strength
// No alpha discard — vertex alpha (35% for water) is preserved for blending.

#include <bgfx_shader.sh>

SAMPLER2D(s_texColor, 0);

uniform vec4 u_waterParams;

// u_fogColor.rgb = fog color, u_fogParams.x = enabled (0/1), u_fogParams.y = fog distance
uniform vec4 u_fogColor;
uniform vec4 u_fogParams;

void main()
{
    float t = u_waterParams.x;
    float distStr = u_waterParams.w;

    // Start with UVs from vertex shader (already rotated + scrolled for flow water)
    vec2 uv = v_texcoord0;

    // UV distortion — two sine ripples for wobble/shimmer effect
    float ripple1 = sin(uv.x * 25.0 + uv.y * 15.0 + t * 2.0);
    float ripple2 = sin(uv.x * 18.0 - uv.y * 22.0 + t * 2.7);
    uv.x += ripple1 * distStr;
    uv.y += ripple2 * distStr;

    vec4 texColor = texture2D(s_texColor, uv);
    vec4 finalColor = texColor * v_color0;
    // Linear distance fog (applied before alpha blending with background)
    float fogFactor = clamp(v_fogDist / u_fogParams.y, 0.0, 1.0) * u_fogParams.x;
    finalColor.rgb = mix(finalColor.rgb, u_fogColor.rgb, fogFactor);
    gl_FragColor = finalColor;
}
