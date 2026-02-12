$input v_texcoord0, v_texcoord1, v_fogDist

#include <bgfx_shader.sh>

SAMPLER2D(s_texColor, 0);
SAMPLER2D(s_texLightmap, 1);

uniform vec4 u_fogColor;
uniform vec4 u_fogParams;
uniform vec4 u_lmAtlasSize;  // x = width, y = height

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

    // Bicubic-filtered lightmap sample
    vec4 light = sampleBicubicLm(s_texLightmap, v_texcoord1, u_lmAtlasSize.xy);

    // Modulate diffuse by lightmap (2x intensity to match Dark Engine convention)
    vec4 finalColor = vec4(diffuse.rgb * light.rgb * 2.0, diffuse.a);

    // Linear distance fog
    float fogFactor = clamp(v_fogDist / u_fogParams.y, 0.0, 1.0) * u_fogParams.x;
    finalColor.rgb = mix(finalColor.rgb, u_fogColor.rgb, fogFactor);

    gl_FragColor = finalColor;
}
