/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2025 darkness contributors
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *****************************************************************************/

// CoronaSystem — light coronas: the camera-facing glare billboard drawn at a
// light source.
//
// TWO SOURCES, ONE RENDERER
//
// 1. AUTHORED. P$Corona is an original Thief 2 property (retail THIEF2.EXE
//    v1.07 carries `cCoronaProperty`; Thief 1 Gold has no such chunk in any
//    mission or in its gamesys). A designer places one deliberately, with a
//    texture, radii, a distance limit and an alpha.
//
//    Retail Thief 2 authors exactly ONE functional corona across all fifteen
//    shipped missions — MISS5 object 35, texture "corona", 2.0 / 65.0 /
//    300.0 / 0.15, flags 0x81. The other 27 records (MISS6 x5, MISS12 x12,
//    MISS13 x10) have every authored field zeroed and carry only the cached
//    anchor position, so they draw nothing. That one record is therefore the
//    engine's own statement of what a corona should look like, and it is
//    where kRetailTemplate below comes from rather than taste.
//
// 2. SYNTHESIZED. Because the retail campaign barely uses the property, the
//    authentic path alone would put a single glow in the whole game. The
//    synthesizer gives every *visible light-emitting object* a corona built
//    from that same template, scaled by the light's own authored brightness.
//    This is an ENHANCEMENT — the original engine drew no such glow — and it
//    is why `graphics.coronas.synthesized` exists as its own switch: turning
//    it off leaves strictly the coronas the mission asked for.
//
// WHY A TRACE AND NOT A DEPTH TEST
//
// The billboard draws with depth testing OFF and visibility decided by a ray
// to the light. That is the engine's construction, and it is also the only
// one that looks right: a corona quad straddles the surface its light is
// mounted on, so depth-testing it would slice the disc along the wall plane
// instead of hiding it. NewDark's `enhanced_corona_trace` documents the same
// model — its fix for "coronas are drawn on top of objects/characters" is a
// *better trace*, not a depth test.
//
// The cost of that model is popping, since one ray gives a binary answer.
// Two things soften it: the trace samples a small disc of points around the
// light rather than just its centre, so an edge occluder yields a partial
// fraction; and that fraction is approached exponentially over
// `fadeSeconds` rather than assigned, so even a genuinely binary transition
// crossfades.

#pragma once

#include "DarknessMath.h"
#include "LightingSystem.h"     // LightSource (live animated brightness)
#include "ObjectPropParser.h"   // ObjectPropData — the visible-model set
#include "RayCaster.h"
#include "RenderParamsParser.h" // hsbToRgb
#include "physics/ObjectCollisionGeometry.h"
#include "WRChunkParser.h"

#include "object/ObjectService.h"
#include "property/DarkPropertyDefs.h"
#include "property/PropertyService.h"
#include "property/TypedProperty.h"
#include "worldquery/ObjectState.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace Darkness {

/// The one authored corona in retail Thief 2 (MISS5 object 35). Used verbatim
/// for synthesized coronas, so their shape is the shipped game's answer to
/// "what does a corona look like" rather than an invented one.
struct CoronaTemplate {
    float radiusNear = 2.0f;
    float radiusFar  = 65.0f;
    float maxDist    = 300.0f;
    float alpha      = 0.15f;
    const char *texture = "corona";
};
inline constexpr CoronaTemplate kRetailTemplate{};

/// Brightness that maps to template scale 1.0. The median concrete P$Light
/// brightness across retail missions is ~120-125 (MISS6 125, MISS1 125,
/// MISS12 120), so a typical lamp gets the shipped corona and only genuinely
/// brighter or dimmer sources deviate.
inline constexpr float kReferenceBrightness = 125.0f;

/// Bounds on that deviation. A 350-brightness lamp (MISS6's brightest) would
/// otherwise reach 1.67x and a near-dead light would collapse to nothing;
/// clamping keeps both ends of the campaign's range presentable.
inline constexpr float kMinBrightScale = 0.5f;
inline constexpr float kMaxBrightScale = 1.75f;

/// P$RenderType value NewDark adds for "draw the corona but not the model".
/// Absent from Thief 2's own `objectrendertype` enum (Normal 0, Not Rendered
/// 1, No Lightmap 2, Editor Only 3), so no retail object can carry it; it is
/// handled purely so fan missions that do behave correctly.
inline constexpr uint32_t kRenderTypeCoronaOnly = 4;

/// One corona, resolved at load. `objID` is the object it hangs off; the
/// world position is recomputed every frame from that object plus `offset`,
/// so a corona on a moving lamp moves with it.
struct CoronaDef {
    int32_t  objID       = 0;
    Vector3  offset{0.0f, 0.0f, 0.0f};
    float    radiusNear  = 0.0f;
    float    radiusFar   = 0.0f;
    float    maxDist     = 0.0f;
    float    alpha       = 0.0f;
    Vector3  color{1.0f, 1.0f, 1.0f};
    uint32_t flags       = 0;
    std::string texture;
    bool     synthesized = false;
    /// >= 0 when the source is an animated light: the corona then tracks its
    /// live brightness, so a flickering torch flickers its glow too.
    int16_t  animLightNum = -1;
    float    animMaxBright = 0.0f;
};

/// Per-frame state, parallel to `defs`.
struct CoronaRuntime {
    std::vector<CoronaDef> defs;
    /// Smoothed visibility in [0,1]. Separate from the instantaneous trace
    /// result so the crossfade survives across frames.
    std::vector<float>     visibility;
    /// Most recent trace result, before smoothing. Kept so a corona skipped
    /// by the per-frame trace budget holds its last answer instead of
    /// decaying toward zero.
    std::vector<float>     traced;

    int authoredCount  = 0;
    int synthCount     = 0;
    int missingTexture = 0;

    /// Round-robin cursor into `defs` for the trace budget.
    size_t traceCursor = 0;

    /// Candidate occluder indices, gathered once per corona and reused by all
    /// of that corona's sample rays. Held here so it keeps its capacity.
    std::vector<uint32_t> occluderScratch;

    bool empty() const { return defs.empty(); }
};

/// Quads produced for one frame, grouped by texture so each group is one
/// draw call.
struct CoronaBatch {
    std::string texture;
    /// 4 vertices per corona, in the order TL, TR, BR, BL.
    struct Vertex {
        float x, y, z;
        float u, v;
        float r, g, b, a;
    };
    std::vector<Vertex> vertices;
    std::vector<uint16_t> indices;
};

// ── Load-time construction ──────────────────────────────────────────────────

namespace detail {

/// Resolve an object's current world position: runtime state first (doors,
/// platforms, tweq-animated lamps), static P$Position otherwise. Same
/// resolution order SelfLitProducer uses, for the same reason — a light that
/// moves must be found where it now is.
inline Vector3 coronaObjectPos(ObjectService *objSvc,
                               const ObjectStateMap *objectStates,
                               int32_t objID) {
    if (objectStates) {
        if (const ObjectState *st = objectStates->tryGet(objID))
            return st->position;
    }
    return objSvc->position(objID);
}

inline Vector3 coronaLightColor(PropertyService *propSvc, int32_t objID) {
    PropLightColor lc{};
    if (getTypedProperty<PropLightColor>(propSvc, "LightColor", objID, lc))
        return hsbToRgb(lc.hue, lc.saturation);
    return Vector3(1.0f, 1.0f, 1.0f);
}

/// sqrt, because a glow's apparent radius scales with the square root of
/// source intensity: the falloff is roughly inverse-square, so the distance
/// at which it crosses a fixed visible threshold goes as sqrt(I).
inline float coronaBrightScale(float brightness) {
    if (!(brightness > 0.0f)) return 0.0f;
    const float s = std::sqrt(brightness / kReferenceBrightness);
    return std::clamp(s, kMinBrightScale, kMaxBrightScale);
}

} // namespace detail

/// Read every authored P$Corona record.
///
/// Only objects that DIRECTLY own a record contribute, matching how
/// SelfLitProducer walks P$SelfLit: `getAllObjectsWithProperty` iterates the
/// storage's own ids, not the inheritance chain. That is correct here because
/// a corona's cached anchor is per-instance — an inherited one would place
/// every instance's glow at the archetype's stale position.
inline void parseAuthoredCoronas(PropertyService *propSvc,
                                 ObjectService *objSvc,
                                 const ObjectStateMap *objectStates,
                                 CoronaRuntime &out) {
    if (!propSvc || !objSvc) return;

    for (int objID : getAllObjectsWithProperty(propSvc, "Corona")) {
        if (objID <= 0) continue; // archetypes have no world position

        PropCorona pc{};
        if (!getTypedProperty<PropCorona>(propSvc, "Corona", objID, pc))
            continue;

        // A record with no alpha, no size or no texture is DromEd having
        // written the property out at its defaults — 27 of retail's 28
        // records look exactly like this. Drawing them would put invisible
        // (or worse, zero-sized NaN-UV) quads in the batch.
        const bool hasSize = (pc.radiusNear > 0.0f) || (pc.radiusFar > 0.0f);
        if (!(pc.alpha > 0.0f) || !hasSize || pc.texture[0] == '\0')
            continue;

        CoronaDef def;
        def.objID      = objID;
        def.radiusNear = pc.radiusNear;
        def.radiusFar  = pc.radiusFar;
        def.maxDist    = (pc.maxDist > 0.0f) ? pc.maxDist
                                             : kRetailTemplate.maxDist;
        def.alpha      = pc.alpha;
        def.flags      = pc.flags;
        def.color      = detail::coronaLightColor(propSvc, objID);

        char tex[17] = {};
        std::memcpy(tex, pc.texture, 16);
        def.texture = tex;

        // The cached anchor minus the object's origin is the vhot offset the
        // engine resolved. Storing the difference rather than the absolute
        // position is what lets the corona follow a moving object while still
        // landing exactly where the original engine put it on a static one.
        const Vector3 objPos =
            detail::coronaObjectPos(objSvc, objectStates, objID);
        const Vector3 cached(pc.posX, pc.posY, pc.posZ);
        Vector3 delta = cached - objPos;
        // Guard against a stale cache from a since-moved object: a huge delta
        // is not a vhot, and honouring it would strand the glow across the
        // level. Vhots live inside the model's bounding box.
        if (glm::length(delta) > 16.0f)
            delta = Vector3(0.0f, 0.0f, 0.0f);
        def.offset = delta;

        out.defs.push_back(std::move(def));
        ++out.authoredCount;
    }
}

/// Give every visible light-emitting object a corona built from the retail
/// template. ENHANCEMENT — see the file header.
///
/// Eligibility is deliberately narrow: the object must have a light AND be
/// something the renderer actually draws. Thief levels are full of invisible
/// fill lights placed to shape the lightmap, and a glow hanging in mid-air
/// where a designer put a bounce light is worse than no glow at all. The
/// visible-model set comes from ObjectPropData::objects, which resolves
/// ModelName through archetype inheritance — a direct property read finds
/// nothing, because retail concretes carry zero direct ModelName records.
inline void synthesizeCoronas(PropertyService *propSvc,
                              ObjectService *objSvc,
                              const ObjectStateMap *objectStates,
                              const ObjectPropData &objData,
                              const std::unordered_map<int16_t, LightSource> &lightSources,
                              CoronaRuntime &out) {
    if (!propSvc || !objSvc) return;

    // Objects the renderer draws.
    std::unordered_set<int32_t> visible;
    visible.reserve(objData.objects.size() * 2);
    for (const auto &o : objData.objects)
        visible.insert(o.objID);

    // Objects that already carry an authored corona — the designer's own
    // record wins, and a second synthesized glow on top would double it.
    std::unordered_set<int32_t> authored;
    for (const auto &d : out.defs)
        authored.insert(d.objID);

    // objectId -> lightNum, so a synthesized corona can follow the live
    // animated brightness rather than a static authored maximum.
    std::unordered_map<int32_t, int16_t> objToAnimLight;
    objToAnimLight.reserve(lightSources.size() * 2);
    for (const auto &kv : lightSources)
        objToAnimLight[kv.second.objectId] = kv.second.lightNum;

    auto eligible = [&](int32_t objID) {
        if (objID <= 0) return false;
        if (authored.count(objID)) return false;
        if (visible.count(objID)) return true;
        // NewDark's "Corona Only" render type: no model, corona still drawn.
        PropRenderType rt{};
        return getTypedProperty<PropRenderType>(propSvc, "RenderType", objID, rt)
               && rt.mode == kRenderTypeCoronaOnly;
    };

    auto emit = [&](int32_t objID, float brightness, const Vector3 &offset,
                    int16_t animLightNum, float animMaxBright) {
        const float scale = detail::coronaBrightScale(brightness);
        if (scale <= 0.0f) return; // brightness 0 = the light is off

        CoronaDef def;
        def.objID        = objID;
        def.offset       = offset;
        def.radiusNear   = kRetailTemplate.radiusNear * scale;
        def.radiusFar    = kRetailTemplate.radiusFar  * scale;
        def.maxDist      = kRetailTemplate.maxDist    * scale;
        def.alpha        = kRetailTemplate.alpha;
        def.color        = detail::coronaLightColor(propSvc, objID);
        def.texture      = kRetailTemplate.texture;
        def.synthesized  = true;
        def.animLightNum = animLightNum;
        def.animMaxBright = animMaxBright;
        out.defs.push_back(std::move(def));
        ++out.synthCount;
    };

    // Animated lights first: when an object has both, the animated record is
    // the one carrying live brightness, and emitting from both would stack
    // two coronas on one lamp.
    std::unordered_set<int32_t> done;

    for (int objID : getAllObjectsWithProperty(propSvc, "AnimLight")) {
        if (!eligible(objID)) continue;
        PropAnimLight al{};
        if (!getTypedProperty<PropAnimLight>(propSvc, "AnimLight", objID, al))
            continue;
        auto it = objToAnimLight.find(objID);
        const int16_t lightNum = (it != objToAnimLight.end()) ? it->second
                                                              : int16_t(-1);
        emit(objID, al.maxBrightness,
             Vector3(al.offsetX, al.offsetY, al.offsetZ),
             lightNum, al.maxBrightness);
        done.insert(objID);
    }

    for (int objID : getAllObjectsWithProperty(propSvc, "Light")) {
        if (done.count(objID) || !eligible(objID)) continue;
        PropLight pl{};
        if (!getTypedProperty<PropLight>(propSvc, "Light", objID, pl))
            continue;
        emit(objID, pl.brightness,
             Vector3(pl.offsetX, pl.offsetY, pl.offsetZ), int16_t(-1), 0.0f);
    }
}

/// Size the parallel per-corona state. Call once after both producers have
/// run. Visibility starts at zero so coronas fade in on the first frames
/// rather than all popping on at once.
inline void finalizeCoronas(CoronaRuntime &rt) {
    rt.visibility.assign(rt.defs.size(), 0.0f);
    rt.traced.assign(rt.defs.size(), 0.0f);
}

// ── Per-frame update ────────────────────────────────────────────────────────

/// Tunables the update needs from config. Kept as a struct so the render loop
/// can pass its mutable console-editable copy without CoronaSystem depending
/// on RenderConfig.
struct CoronaSettings {
    bool  enabled      = true;
    bool  synthesized  = true;   // load-time only; here for the console's sake
    float intensity    = 1.0f;   // multiplies alpha; >1 pushes into bloom
    float sizeScale    = 1.0f;   // multiplies both radii
    float maxDistScale = 1.0f;   // multiplies the per-corona distance limit
    float fadeSeconds  = 0.15f;  // occlusion crossfade time constant
    /// Rays per corona per visibility test. 1 = centre only (binary, pops).
    /// Higher samples the disc so a corona clipping an edge partially fades.
    int   traceSamples = 5;
    /// Coronas re-traced per frame. The rest hold their previous answer, so
    /// this bounds raycast cost without freezing the fade.
    int   traceBudget  = 32;
    /// Per-pixel depth fade: attenuate the billboard where scene geometry
    /// stands in FRONT of it. This is what stops a corona painting over a
    /// wall that is clearly nearer than its lamp — the ray trace below
    /// answers "is the source visible", which is a different question and
    /// cannot express partial coverage of the drawn disc.
    bool  depthFade      = true;
    /// How far in front of the billboard an occluder must be to hide it
    /// completely, in world units. Small values give a crisp silhouette;
    /// larger ones let the glow bleed softly around an edge, which is what
    /// real glare does.
    float depthFadeRange = 1.5f;
    /// Pull the billboard this far toward the camera along the view axis
    /// before drawing. Without it a corona fades against the very lamp it
    /// belongs to: the emitter's own model writes depth at almost exactly the
    /// corona's distance, so the depth fade reads it as an occluder.
    /// HPL2's cBillboard carries the same knob as `mfForwardOffset`
    /// (BillBoard.cpp:292), applied along the camera forward; ours is signed
    /// the intuitive way round, positive = toward the viewer.
    float depthFadeOffset = 0.5f;
    /// Trace against objects as well as terrain. The original engine traced
    /// terrain ONLY, which is why its coronas shine through closed doors and
    /// draw over characters; NewDark added the same switch as
    /// `enhanced_corona_trace`. Default on — the artifact is very visible.
    bool  traceObjects = true;
};

/// Advance visibility and produce the draw batches for this frame.
///
/// `viewProj`-free by design: the quad is built in world space from the
/// camera basis, so the same batch works for any view that shares the camera.
inline void updateCoronas(CoronaRuntime &rt,
                          const CoronaSettings &cfg,
                          const WRParsedData &wr,
                          ObjectService *objSvc,
                          const ObjectStateMap *objectStates,
                          const std::unordered_map<int16_t, LightSource> &lightSources,
                          const ObjectCollisionWorld *objCollision,
                          const Vector3 &camPos,
                          const Vector3 &camRight,
                          const Vector3 &camUp,
                          const Vector3 &camForward,
                          int32_t camCellHint,
                          float dt,
                          std::vector<CoronaBatch> &outBatches) {
    outBatches.clear();
    if (!cfg.enabled || rt.empty() || !objSvc) return;

    const size_t n = rt.defs.size();
    if (rt.visibility.size() != n) finalizeCoronas(rt);

    // Exponential approach toward the traced value. Frame-rate independent:
    // the same wall-clock fade whatever the frame time.
    const float fade = (cfg.fadeSeconds > 1e-4f)
        ? (1.0f - std::exp(-dt / cfg.fadeSeconds))
        : 1.0f;

    const int budget = (cfg.traceBudget > 0)
        ? std::min<int>(cfg.traceBudget, static_cast<int>(n))
        : static_cast<int>(n);
    const int samples = std::clamp(cfg.traceSamples, 1, 9);

    // Which coronas get a fresh trace this frame.
    std::vector<uint8_t> retrace(n, 0);
    for (int i = 0; i < budget; ++i) {
        retrace[rt.traceCursor] = 1;
        rt.traceCursor = (rt.traceCursor + 1) % n;
    }

    std::unordered_map<std::string, size_t> batchIndex;

    for (size_t i = 0; i < n; ++i) {
        const CoronaDef &def = rt.defs[i];

        const Vector3 pos =
            detail::coronaObjectPos(objSvc, objectStates, def.objID) + def.offset;

        const Vector3 toCam = camPos - pos;
        const float dist = glm::length(toCam);
        const float maxDist = def.maxDist * cfg.maxDistScale;

        // Behind the camera, or past the light's own limit.
        const bool inRange = (dist <= maxDist) &&
                             (glm::dot(pos - camPos, camForward) > 0.0f);

        if (!inRange) {
            // Decay rather than snap: walking backwards past the limit should
            // not blink the glow out.
            rt.traced[i] = 0.0f;
            rt.visibility[i] += (0.0f - rt.visibility[i]) * fade;
            continue;
        }

        if (retrace[i]) {
            // The ray runs all the way to the emitter, with no epsilon pulled
            // back off it. That was tried: the worry was that a torch mounted
            // on a wall would have its object origin inside its own bracket
            // and occlude itself. Measured on MISS6, backing the endpoint off
            // by 1 and then 4 world units left the clear count identical at
            // 4 of 102, so the occlusion is real wall geometry rather than
            // self-shadowing, and the epsilon was doing nothing but risking
            // false positives around corners. Do not reintroduce it without
            // a measurement that shows it changing something.
            const float sampleRadius = def.radiusNear * cfg.sizeScale;

            // Objects: doors, crates, bookcases, anything with a collision
            // body. Terrain alone is what the original engine traced, and it
            // is why its coronas shine through closed doors and paint over
            // characters — the exact artifact NewDark's `enhanced_corona_trace`
            // was added to fix.
            //
            // Candidates are gathered ONCE for the whole sample bundle, padded
            // by the disc radius so the off-centre rays are covered too. Every
            // sample ray then tests only those few bodies instead of walking
            // the level's entire object list.
            const bool useObjects = cfg.traceObjects && (objCollision != nullptr);
            if (useObjects) {
                objCollision->gatherSegmentOccluders(
                    camPos, pos, sampleRadius, def.objID, rt.occluderScratch);
            }

            int clear = 0;
            for (int s = 0; s < samples; ++s) {
                Vector3 target = pos;
                if (s > 0) {
                    // Offset the extra samples around the disc in the camera
                    // plane, at the corona's near radius. These are the points
                    // whose occlusion decides whether an edge cuts the glow.
                    const float ang =
                        6.2831853f * static_cast<float>(s - 1) /
                        static_cast<float>(samples - 1);
                    target += camRight * (std::cos(ang) * sampleRadius) +
                              camUp    * (std::sin(ang) * sampleRadius);
                }
                RayHit hit;
                int32_t terminalCell = -1;
                if (raycastWorld(wr, camPos, target, hit, &terminalCell,
                                 camCellHint))
                    continue; // blocked by terrain

                bool blocked = false;
                if (useObjects) {
                    for (uint32_t bodyIdx : rt.occluderScratch) {
                        if (objCollision->segmentHitsBody(camPos, target,
                                                          bodyIdx)) {
                            blocked = true;
                            break;
                        }
                    }
                }
                if (!blocked) ++clear;
            }
            rt.traced[i] = static_cast<float>(clear) /
                           static_cast<float>(samples);
        }

        rt.visibility[i] += (rt.traced[i] - rt.visibility[i]) * fade;

        const float vis = rt.visibility[i];
        if (vis <= 0.002f) continue; // below this it contributes nothing

        // Interpolate the world radius across the light's visible range, so
        // the billboard holds roughly constant apparent size. Both endpoints
        // come from the record; `radiusFar > radiusNear` is expected.
        const float t = (maxDist > 1e-4f) ? std::clamp(dist / maxDist, 0.0f, 1.0f)
                                          : 0.0f;
        const float radius =
            (def.radiusNear + (def.radiusFar - def.radiusNear) * t) *
            cfg.sizeScale;
        if (radius <= 1e-4f) continue;

        // Live animated brightness, so a flickering torch flickers its glow.
        float animScale = 1.0f;
        if (def.animLightNum >= 0 && def.animMaxBright > 0.0f) {
            auto it = lightSources.find(def.animLightNum);
            if (it != lightSources.end()) {
                animScale = std::clamp(
                    it->second.brightness / def.animMaxBright, 0.0f, 1.0f);
            }
        }
        if (animScale <= 0.002f) continue;

        // The gain can and should exceed 1.0 — that is the whole mechanism by
        // which a corona reaches the overbright range bloom responds to. It
        // rides a float vertex attribute for exactly that reason; a packed
        // 8-bit colour could not carry it.
        const float gain = def.alpha * cfg.intensity * vis * animScale;
        if (gain <= 0.0f) continue;

        auto bit = batchIndex.find(def.texture);
        if (bit == batchIndex.end()) {
            bit = batchIndex.emplace(def.texture, outBatches.size()).first;
            outBatches.push_back(CoronaBatch{});
            outBatches.back().texture = def.texture;
        }
        CoronaBatch &batch = outBatches[bit->second];

        const uint16_t base = static_cast<uint16_t>(batch.vertices.size());
        // 65532 vertices = 16383 coronas in one batch; past that the 16-bit
        // index buffer wraps. Start a new batch instead of corrupting this one.
        if (base > 65531) continue;

        // Nudge toward the viewer so the sprite clears its own emitter's
        // geometry before the depth fade compares against it. Only matters
        // when the fade is on; harmless either way at this magnitude.
        const Vector3 drawPos = pos - camForward * cfg.depthFadeOffset;

        const Vector3 right = camRight * radius;
        const Vector3 up    = camUp    * radius;
        const Vector3 corners[4] = {
            drawPos - right + up,  // TL
            drawPos + right + up,  // TR
            drawPos + right - up,  // BR
            drawPos - right - up,  // BL
        };
        const float uvs[4][2] = { {0,0}, {1,0}, {1,1}, {0,1} };

        for (int c = 0; c < 4; ++c) {
            CoronaBatch::Vertex v;
            v.x = corners[c].x; v.y = corners[c].y; v.z = corners[c].z;
            v.u = uvs[c][0];    v.v = uvs[c][1];
            v.r = def.color.x * gain;
            v.g = def.color.y * gain;
            v.b = def.color.z * gain;
            v.a = 1.0f;
            batch.vertices.push_back(v);
        }
        const uint16_t idx[6] = { 0, 1, 2, 0, 2, 3 };
        for (uint16_t k : idx)
            batch.indices.push_back(static_cast<uint16_t>(base + k));
    }
}

} // namespace Darkness
