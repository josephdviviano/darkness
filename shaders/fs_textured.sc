$input v_color0, v_texcoord0, v_fogDist

#include <bgfx_shader.sh>

SAMPLER2D(s_texColor, 0);

// u_fogColor.rgb = fog color, u_fogParams.x = enabled (0/1), u_fogParams.y = fog distance
uniform vec4 u_fogColor;
uniform vec4 u_fogParams;

// u_objectParams.x = per-object alpha (1.0 = opaque, < 1.0 = translucent)
// Used by RenderAlpha property for semi-transparent objects (e.g. windows)
uniform vec4 u_objectParams;

// u_objectLight.rgb = per-object lighting tint computed by ObjectIlluminator
// (ambient + visible cell lights + sun + ExtraLight). When per-object
// lighting is disabled, the renderer submits white so this is a no-op.
uniform vec4 u_objectLight;

void main()
{
    vec4 texColor = texture2D(s_texColor, v_texcoord0);
    // Alpha test: discard fully transparent pixels (palette index 0 = alpha 0)
    // Test texture alpha before applying per-object opacity to avoid
    // discarding visible texels on translucent objects
    if (texColor.a < 0.5) discard;
    vec4 finalColor = texColor * v_color0;
    // Apply per-object opacity (1.0 for opaque objects, < 1.0 for translucent)
    finalColor.a *= u_objectParams.x;
    // Modulate by per-object lighting tint (Dark Engine convention: a single
    // RGB color from cell lights + ambient applied uniformly across the model).
    finalColor.rgb *= u_objectLight.rgb;
    // Frob highlight: additive brightness boost (Dark Engine convention).
    // u_objectParams.y = highlight intensity (0.0 = none, ~0.47 = full highlight).
    finalColor.rgb += vec3_splat(u_objectParams.y);
    // Linear distance fog
    float fogFactor = clamp(v_fogDist / u_fogParams.y, 0.0, 1.0) * u_fogParams.x;
    finalColor.rgb = mix(finalColor.rgb, u_fogColor.rgb, fogFactor);
    gl_FragColor = finalColor;
}
