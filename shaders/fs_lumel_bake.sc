$input v_lumelUV

#include <bgfx_shader.sh>
#include "live_lights.sh"

// S2 lumel bake: ONE light's overlay contribution for one polygon rect —
// the bake formula (half-Lambert x physical falloff x per-light K) with
// visibility from the S1 face atlas (door-inclusive), zero bounces. The
// GPU twin of the CPU overlay bake; [LUMEL_BAKE] diffs the two.
//
// worldPos = origin + u*spanU + v*spanV  (mirror of LumelGrid::at with
// the half-lumel offset folded into origin CPU-side).
uniform vec4 u_bakeOrigin;   // xyz origin, w = surface offset
uniform vec4 u_bakeSpanU;    // xyz = axisU * stepU * lx
uniform vec4 u_bakeSpanV;    // xyz = axisV * stepV * ly
uniform vec4 u_bakeNormal;   // xyz plane normal (faces the cell's air)
uniform vec4 u_bakeLight;    // xyz light pos, w = reach^2
uniform vec4 u_bakeColor;    // rgb = bright x brightScale x K_i, w = slot
uniform vec4 u_bakeSpot;     // xyz = cone axis, w = inner cosine (-1 = omni)
                             // (the OUTER cosine rides u_bakeNormal.w)

void main()
{
    vec3 worldPos = u_bakeOrigin.xyz
                  + v_lumelUV.x * u_bakeSpanU.xyz
                  + v_lumelUV.y * u_bakeSpanV.xyz;
    vec3 v = u_bakeLight.xyz - worldPos;
    float d2 = dot(v, v);
    if (u_bakeLight.w > 0.0 && d2 > u_bakeLight.w) {
        gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }
    float dist = sqrt(max(d2, 1e-6));
    float cosT = dot(u_bakeNormal.xyz, v) / dist;
    if (cosT <= 0.0) {
        gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }
    float halfLam = cosT * 0.5 + 0.5;

    // Spotlight cone, mirroring bakeOneLight's construction exactly: a
    // linear ramp between the two cone COSINES, hard cutoff at the outer.
    // Spot door lights used to fall back to CPU rays for want of this —
    // 240 of door 407's 1681 rects, the last CPU path in the event.
    float spot = 1.0;
    if (u_bakeSpot.w != -1.0) {
        float dotVal = dot(-v / dist, u_bakeSpot.xyz);
        if (dotVal <= u_bakeNormal.w) {
            gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
            return;
        }
        if (dotVal < u_bakeSpot.w) {
            float denom = u_bakeSpot.w - u_bakeNormal.w;
            spot = (denom > 1e-6) ? (dotVal - u_bakeNormal.w) / denom : 1.0;
        }
    }
    float fall = 1.0 / (d2 + u_liveFalloff.x);
    // Visibility from the offset probe point, like the bake's rays.
    vec3 probe = worldPos + u_bakeNormal.xyz * u_bakeOrigin.w;
    // PCSS: event re-bakes keep the load bake's SOFT penumbra (a 1-tap
    // bake permanently hardened every shadow edge a door swing touched
    // — the rest-state artifact class the --door-diff-diag harness
    // exposed). PCSS emitter size rides u_liveFalloff.y; 0 = exact
    // hard tap (the self-test path).
    // The polygon plane goes in so every PCF tap is depth-referenced to
    // where its OWN ray meets that plane, not to this fragment's distance.
    // Without it, grazing receivers self-shadow at the kernel edges — the
    // gpuPhantomShadow class the door-diff harness counted.
    float shadow = liveShadowFactorPCSS(
        probe, u_bakeLight.xyz, u_bakeColor.w,
        inversesqrt(max(u_bakeLight.w, 1e-6)),
        u_liveFalloff.y, u_bakeNormal.xyz);

    // The Continuous 5-bit storage transform — the CPU accumulator's
    // exact math (LumelAccumulator::add, Continuous case, incl. the C1
    // toe). Without it the GPU rect is ~3.5% hot plus offset (measured
    // mean 11.75/255 by the [LUMEL_BAKE] self-test). Omni lights only —
    // spotFactor folding is a recorded TODO for spot door lights.
    vec3 c = u_bakeColor.rgb * (halfLam * fall);
    float peak = max(c.r, max(c.g, c.b));
    if (peak <= 1e-6) {
        gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }
    float lux = peak * 255.0 * shadow - 0.5;
    if (lux <= 0.0) {
        gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }
    lux *= spot;          // AFTER the -0.5 and its test: LumelAccumulator's
                          // Continuous case does exactly this, and the order
                          // is visible in the result inside the penumbra.
    if (lux <= 0.0) {
        gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }
    lux = min(lux, 255.0);
    vec3 m = c * (255.0 / peak);
    vec3 gv = m * (lux * 31.0 / 65536.0);
    vec3 toe = mix(gv - 0.5, 0.5 * gv * gv, vec3(lessThan(gv, vec3(1.0, 1.0, 1.0))));
    gl_FragColor = vec4(max(toe, vec3(0.0, 0.0, 0.0)) * (8.0 / 255.0), 1.0);
}
