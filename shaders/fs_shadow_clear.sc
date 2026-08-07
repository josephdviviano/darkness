#include <bgfx_shader.sh>

// 1.0 = "no occluder within reach" — the value cleared tiles must read
// (see kShadowClearPalette's old comment; the palette died with the
// per-view clears).
void main()
{
    gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0);
}
