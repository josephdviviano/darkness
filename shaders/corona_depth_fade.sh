#ifndef CORONA_DEPTH_FADE_SH
#define CORONA_DEPTH_FADE_SH

// Shared corona shading, so the plain and MSAA fragment shaders cannot drift.
// Each includes this after defining coronaSceneDepth().
//
// THE FADE
//
// A corona is a camera-facing billboard drawn additively with depth testing
// OFF, because a depth-tested one would be sliced along the plane of whatever
// wall its lamp is mounted on. The cost of that is bleed: without a depth
// test the sprite paints over geometry standing in front of it.
//
// So the occlusion is done here instead, per pixel, by comparing against the
// scene depth the world pass already wrote. This is the "proximity fade" /
// soft-particle construction — Godot exposes the same thing on
// StandardMaterial3D as `proximity_fade_distance`.
//
// ONE-SIDED, and that is the whole trick. Only geometry NEARER than the
// billboard attenuates it. A two-sided fade would also react to the wall
// BEHIND the flame — which for a wall-mounted torch is a few inches away, and
// would gut exactly the coronas that matter most.

// u_coronaDepth.x = fade distance in world units (how far in front of the
//                   billboard an occluder must be to hide it completely)
// u_coronaDepth.y = near plane, .z = far plane
// u_coronaDepth.w = 1 when a depth buffer is bound, 0 when there is none
//                   (post-processing off, or no sampleable depth format)
uniform vec4 u_coronaDepth;

// Depth-buffer value (0..1) to view-space distance along the view axis.
//
// One formula covers every backend. The projection matrix differs with
// bgfx's homogeneousDepth — clip z in [-1,1] versus [0,1] — but the value
// STORED in the depth buffer is [0,1] after the viewport transform either
// way, and both conventions reduce to this same expression.
float coronaLinearDepth(float d, float zNear, float zFar)
{
    return (zNear * zFar) / (zFar - d * (zFar - zNear));
}

vec4 coronaShade(vec4 texColor, vec4 vColor, float viewZ, float fogDist,
                 vec4 fogParams, vec2 screenUv)
{
    // NO alpha-test discard. Every other textured pass in this renderer
    // discards below 0.5 because palette index 0 is a hard transparency key;
    // a corona is the opposite case — its shape IS a continuous alpha ramp
    // from an opaque centre to a transparent rim, and cutting that at 0.5
    // would replace the glow with a hard-edged disc.
    //
    // The sprite stores its shape in alpha over flat white, so this multiply
    // is what gives the billboard its falloff.
    vec3 color = texColor.rgb * texColor.a * vColor.rgb;

    if (u_coronaDepth.w > 0.5)
    {
        float sceneD = coronaSceneDepth(screenUv);
        float sceneZ = coronaLinearDepth(sceneD, u_coronaDepth.y, u_coronaDepth.z);
        float range  = max(u_coronaDepth.x, 0.0001);
        // sceneZ >= viewZ  → nothing in front → 1, untouched.
        // sceneZ == viewZ - range → 0, fully hidden.
        // Sky writes the far plane, so open sky never attenuates anything.
        color *= clamp((sceneZ - viewZ) / range + 1.0, 0.0, 1.0);
    }

    // Additive fog, NOT the mix() the opaque passes use. This draw adds to
    // whatever is already in the target, so mixing toward the fog colour
    // would BRIGHTEN the quad's whole footprint as fog thickens instead of
    // hiding the glow behind it. Attenuating the contribution is the additive
    // equivalent, and it reaches zero exactly where an opaque surface would
    // be fully fogged.
    float fogFactor = clamp(fogDist / fogParams.y, 0.0, 1.0) * fogParams.x;
    color *= (1.0 - fogFactor);

    // Alpha 0: with additive blending the destination alpha would otherwise
    // accumulate one contribution per corona drawn over the same pixel. The
    // draw writes RGB only, so this is belt-and-braces.
    //
    // Returned rather than written to gl_FragColor directly: shaderc's HLSL
    // path makes gl_FragColor part of main()'s output struct, so assigning it
    // from a helper fails to parse.
    return vec4(color, 0.0);
}

#endif // CORONA_DEPTH_FADE_SH
