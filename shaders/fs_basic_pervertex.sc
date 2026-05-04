$input v_color0, v_texcoord0, v_fogDist

#include <bgfx_shader.sh>

// u_fogColor.rgb = fog color, u_fogParams.x = enabled (0/1), u_fogParams.y = fog distance
uniform vec4 u_fogColor;
uniform vec4 u_fogParams;

// u_objectParams.x = per-object alpha
// u_objectParams.y = frob highlight intensity
uniform vec4 u_objectParams;

// Same convention as fs_textured_pervertex: v_color0 already carries the
// full lighting term, so u_objectLight is intentionally not consumed.
//
// v_texcoord0 is part of the shared per-vertex varying file but unused
// in the basic (untextured) fragment path. Reading varyings the vertex
// shader writes is harmless; bgfx attaches by name.

void main()
{
    vec4 finalColor = v_color0;

    // Per-object opacity (1.0 for opaque, < 1.0 for translucent).
    finalColor.a *= u_objectParams.x;

    // Frob highlight: additive brightness boost.
    finalColor.rgb += vec3_splat(u_objectParams.y);

    // Linear distance fog.
    float fogFactor = clamp(v_fogDist / u_fogParams.y, 0.0, 1.0) * u_fogParams.x;
    finalColor.rgb = mix(finalColor.rgb, u_fogColor.rgb, fogFactor);

    gl_FragColor = finalColor;
}
