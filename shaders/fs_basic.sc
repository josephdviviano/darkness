$input v_color0, v_fogDist

#include <bgfx_shader.sh>

// u_fogColor.rgb = fog color, u_fogParams.x = enabled (0/1), u_fogParams.y = fog distance
uniform vec4 u_fogColor;
uniform vec4 u_fogParams;

// u_objectParams.x = per-object alpha (1.0 = opaque, < 1.0 = translucent)
// Used by RenderAlpha property for semi-transparent objects (e.g. windows)
uniform vec4 u_objectParams;

void main()
{
    vec4 color = v_color0;
    // Apply per-object opacity (1.0 for opaque objects, < 1.0 for translucent)
    color.a *= u_objectParams.x;
    // Linear distance fog: blend toward fog color from 0 at camera to full at fogDistance
    float fogFactor = clamp(v_fogDist / u_fogParams.y, 0.0, 1.0) * u_fogParams.x;
    color.rgb = mix(color.rgb, u_fogColor.rgb, fogFactor);
    gl_FragColor = color;
}
