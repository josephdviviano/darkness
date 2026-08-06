$input v_texcoord0, v_texcoord1, v_worldPos, v_fogDist

#include <bgfx_shader.sh>
#include "lm_specular.sh"

SAMPLER2D(s_texColor, 0);
SAMPLER2D(s_texLightmap, 1);
SAMPLER2D(s_texLmDir, 2);

uniform vec4 u_fogColor;
uniform vec4 u_fogParams;
// x = lightmap scale. See graphics.lightmap_scale — this is the "2X modulate"
// multiplier, and whether it belongs here at all is content-dependent:
// NewDark's docs say 16-bit lightmaps (which is what retail ships) top out at
// "texture seen fullbright", i.e. x1, and that overbrightening is the 32-bit
// 2X mode only. We default to 2.0 because it is the look this renderer was
// tuned against, NOT because retail authored for it. NOTES.PROJECT.md
// "Lightmap scale" has the evidence.
uniform vec4 u_lightmapScale;
uniform vec4 u_lmAtlasSize;  // x = width, y = height
// Dominant-direction specular (see lm_specular.sh for the layout).
uniform vec4 u_specParams;
uniform vec4 u_camPosWorld;

// Cubic B-spline basis functions for bicubic filtering.
// Uses Godot's 4-tap approach: combine pairs of cubic weights and use
// GPU bilinear hardware taps at offset positions for efficient sampling.
float w0(float a) { return (1.0/6.0) * (a * (a * (-a + 3.0) - 3.0) + 1.0); }
float w1(float a) { return (1.0/6.0) * (a * a * (3.0*a - 6.0) + 4.0); }
float w2(float a) { return (1.0/6.0) * (a * (a * (-3.0*a + 3.0) + 3.0) + 1.0); }
float w3(float a) { return (1.0/6.0) * (a * a * a); }

// Combined weights for pairs of cubic samples
float g0(float a) { return w0(a) + w1(a); }
float g1(float a) { return w2(a) + w3(a); }

// Offset within each pair — positions bilinear tap to blend two cubic samples
float h0(float a) { return -1.0 + w1(a) / (w0(a) + w1(a)); }
float h1(float a) { return  1.0 + w3(a) / (w2(a) + w3(a)); }

// Sample lightmap atlas with cubic B-spline filtering using 4 bilinear taps
vec4 sampleBicubicLm(sampler2D tex, vec2 uv, vec2 texSize) {
    vec2 texelSize = vec2(1.0) / texSize;
    vec2 coord = uv * texSize + vec2(0.5);
    vec2 icoord = floor(coord);
    vec2 fcoord = fract(coord);

    float g0x = g0(fcoord.x); float g1x = g1(fcoord.x);
    float h0x = h0(fcoord.x); float h1x = h1(fcoord.x);
    float h0y = h0(fcoord.y); float h1y = h1(fcoord.y);

    vec2 p0 = (vec2(icoord.x + h0x, icoord.y + h0y) - vec2(0.5)) * texelSize;
    vec2 p1 = (vec2(icoord.x + h1x, icoord.y + h0y) - vec2(0.5)) * texelSize;
    vec2 p2 = (vec2(icoord.x + h0x, icoord.y + h1y) - vec2(0.5)) * texelSize;
    vec2 p3 = (vec2(icoord.x + h1x, icoord.y + h1y) - vec2(0.5)) * texelSize;

    return g0(fcoord.y) * (g0x * texture2D(tex, p0) + g1x * texture2D(tex, p1))
         + g1(fcoord.y) * (g0x * texture2D(tex, p2) + g1x * texture2D(tex, p3));
}

void main()
{
    vec4 diffuse = texture2D(s_texColor, v_texcoord0);
    if (diffuse.a < 0.5) discard;
    // u_lightmapScale.y = lighting-only debug view. Must stay in step with
    // fs_lightmapped.sc — the console toggle drives both programs.
    diffuse.rgb = mix(diffuse.rgb, vec3(1.0, 1.0, 1.0), u_lightmapScale.y);

    // Bicubic-filtered lightmap sample
    vec4 light = sampleBicubicLm(s_texLightmap, v_texcoord1, u_lmAtlasSize.xy);

    // Modulate diffuse by lightmap (2x intensity to match Dark Engine convention)
    vec4 finalColor = vec4(diffuse.rgb * light.rgb * u_lightmapScale.x, diffuse.a);

    // Rough Fresnel specular from the baked dominant direction — must stay
    // in step with fs_lightmapped.sc. Direction is low-frequency; plain
    // bilinear is fine even in the bicubic program.
    vec4 dirTex = texture2D(s_texLmDir, v_texcoord1);
    finalColor.rgb += lmSpecular(dirTex, v_worldPos, u_camPosWorld.xyz,
                                 light.rgb, diffuse.rgb, u_specParams);

    // Linear distance fog
    float fogFactor = clamp(v_fogDist / u_fogParams.y, 0.0, 1.0) * u_fogParams.x;
    finalColor.rgb = mix(finalColor.rgb, u_fogColor.rgb, fogFactor);

    gl_FragColor = finalColor;
}
