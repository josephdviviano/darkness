$input v_color0, v_texcoord0, v_fogDist

#include <bgfx_shader.sh>

SAMPLER2D(s_texColor, 0);

// u_fogColor.rgb = fog color, u_fogParams.x = enabled (0/1), u_fogParams.y = fog distance
uniform vec4 u_fogColor;
uniform vec4 u_fogParams;

// u_objectParams.x = per-object alpha (1.0 = opaque, < 1.0 = translucent
//                                      via RenderAlpha property)
// u_objectParams.y = frob highlight intensity (additive 0..~0.47)
uniform vec4 u_objectParams;

// Note: u_objectLight is intentionally NOT used here. v_color0 from the
// per-vertex vertex shader already carries the full lighting term
// (ambient + per-light Lambertian + ExtraLight), so the scalar tint
// would be a duplicate.

void main()
{
    vec4 texColor = texture2D(s_texColor, v_texcoord0);
    // Alpha test: discard fully transparent pixels (palette index 0 = alpha 0).
    // Test texture alpha before applying per-object opacity.
    if (texColor.a < 0.5) discard;

    // Texture × per-vertex Lambertian sum (carried in v_color0).
    vec4 finalColor = texColor * v_color0;

    // Per-object opacity (1.0 for opaque, < 1.0 for translucent).
    finalColor.a *= u_objectParams.x;

    // Frob highlight: additive brightness boost (Dark Engine convention).
    finalColor.rgb += vec3_splat(u_objectParams.y);

    // Linear distance fog: 0 at camera → full at fogDistance.
    float fogFactor = clamp(v_fogDist / u_fogParams.y, 0.0, 1.0) * u_fogParams.x;
    finalColor.rgb = mix(finalColor.rgb, u_fogColor.rgb, fogFactor);

    gl_FragColor = finalColor;
}
