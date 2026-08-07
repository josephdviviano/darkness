// S4a (PLAN.HIGH_RES_SHADOWS §S4): per-fragment term for LIVE lights —
// lights promoted out of the baked atlas (a door is swinging through
// their cone) plus transient sources like the flashlight test vehicle.
//
// The term must be BRIGHTNESS-CONTINUOUS with the overlay it replaces:
// same half-Lambert response the bake uses (backface culled first, like
// bakeOneLight), same physical falloff 1/(d²+a²) with the light's
// per-light throw intensity K_i PREMULTIPLIED into the colour CPU-side
// (mirrors the object per-vertex path), and the result adds to the
// sampled lightmap BEFORE the lightmap scale so display treatment
// matches the baked path exactly.
//
// The face normal comes from screen-space derivatives — exact on flat
// polygons, the same construction lm_specular.sh ships. Shadows arrive
// in S4b (S1 atlas face lookup); until then a promoted light is
// unshadowed for the frames it is live.
//
// GLSL 120-safe: no integer textures, no dynamic array params — the
// uniforms are declared here and both lightmapped fragment shaders
// share them (edit alongside ObjectIllumination/DoorShadow C++ sides).

#ifndef LIVE_LIGHT_CAP
#define LIVE_LIGHT_CAP 4
#endif

// .x = active light count (uniform flow — safe around derivatives)
uniform vec4 u_liveLightCount;
// .x = emitter a² (shared falloff denominator term)
uniform vec4 u_liveFalloff;
// xyz = world position, .w = reach² (cutoff; contribution there is
// sub-quantisation by construction, so the edge cannot pop)
uniform vec4 u_liveLightPos[LIVE_LIGHT_CAP];
// rgb = bright × brightScale × K_i (throw intensity folded), .w unused
uniform vec4 u_liveLightColor[LIVE_LIGHT_CAP];

vec3 liveLightSum(vec3 worldPos, vec3 camPos)
{
    vec3 sum = vec3(0.0, 0.0, 0.0);
    int n = int(u_liveLightCount.x);
    if (n <= 0) return sum;
    // Viewer-oriented face normal: a visible fragment faces the camera.
    vec3 nrm = normalize(cross(dFdy(worldPos), dFdx(worldPos)));
    vec3 toCam = camPos - worldPos;
    nrm = nrm * sign(dot(nrm, toCam));
    for (int i = 0; i < LIVE_LIGHT_CAP; ++i) {
        if (i >= n) break;
        vec3 v = u_liveLightPos[i].xyz - worldPos;
        float d2 = dot(v, v);
        if (u_liveLightPos[i].w > 0.0 && d2 > u_liveLightPos[i].w)
            continue;
        float dist = sqrt(max(d2, 1e-6));
        float cosT = dot(nrm, v) / dist;
        // The bake culls backfaces BEFORE half-Lambert — mirror it.
        if (cosT <= 0.0) continue;
        float halfLam = cosT * 0.5 + 0.5;
        float fall = 1.0 / (d2 + u_liveFalloff.x);
        sum += u_liveLightColor[i].rgb * (halfLam * fall);
    }
    return sum;
}
