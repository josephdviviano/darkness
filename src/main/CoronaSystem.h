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
#include "SubObjectPose.h"      // findVHotModelSpace — the light attachment point
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

/// The SIZE CURVE synthesized coronas use, taken from what people who actually
/// author coronas choose rather than from retail's single record.
///
/// Retail MISS5 is an outlier in the one field that matters at range. Census of
/// every P$Corona in the fan missions on hand (42 records, 13 distinct settings,
/// Ominous Bequest Gold + Equilibrium) against it:
///
///                      radiusNear   radiusFar   maxDist    growth half-angle
///   fan missions        1.0-2.5      4-18       8-64        3.6-17.9 deg
///     median            2.0          6.0        48          6.4 deg
///   retail MISS5        2.0          65.0       300         12.0 deg
///
/// The GROWTH RATE is not the problem — retail's 12 degrees sits inside normal
/// practice. `maxDist` is: 300 is six times the largest value any of those
/// authors picked. A 12-degree disc is unobjectionable while you only ever see
/// it inside 48 units, and absurd when it is still 12 degrees at 300 while the
/// lamp itself has shrunk to four pixels.
///
/// So the size curve tops out at the fan-mission median (radiusFar 6.0 reached
/// at 48 units) and stays there. `maxDist` — how far the corona is DRAWN at all
/// — keeps retail's 300, because a distant lamp glow is worth having in a dark
/// level; it just should not be a dinner plate.
inline constexpr CoronaTemplate kSynthTemplate{
    /*radiusNear*/ 2.0f,
    /*radiusFar */ 6.0f,
    /*maxDist   */ 48.0f,   // where radiusFar is reached — NOT the draw range
    /*alpha     */ 0.15f,   // retail's; fan missions run 0.20-0.75
    /*texture   */ "corona",
};

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

/// The attachment point a corona hangs off: the model's vhot with engine index
/// 1, transformed by the object's own rotation.
///
/// MEASURED, not assumed. Retail caches the anchor it resolved into the
/// P$Corona record's position field, so the engine's own answer is readable
/// off disk. Subtracting each object's P$Position leaves the offset the engine
/// applied, and every one of them is a vhot at index 1:
///
///   MISS6  obj 95/160/167/180/193 → delta z +3.980, |xy| < 0.08
///                                   = STRLANT.BIN  vhot 1 (-0.046, 0.020, 3.986)
///   MISS12 obj 1354/1356/1360/1362 → delta z +0.297, |xy| ~ 0.40
///                                   = GASLITE*.BIN vhot 1 ( 0.393, 0.073, 0.297)
///   MISS12 obj 1388/1432/1433/1628/1823/2020 → delta z +0.091, |xy| ~ 0.22
///                                   = ELECLGH.BIN  vhot 1 (-0.244,-0.020, 0.091)
///
/// Each of those three points is unique across all 1783 stock models at the
/// measured tolerance, so the match is not a coincidence of similar numbers.
/// (The residual few thousandths are the object positions being printed to one
/// decimal, not a real disagreement.)
///
/// Three details fall out of the same data and each of them is a way to get
/// this wrong:
///
///   INDEX, NOT SLOT. STRLANT.BIN stores index 3 at array slot 0 and index 1 at
///   slot 1; GASLITE2.BIN stores the same pair in the opposite order. The
///   anchors match index 1 in both, so "the first vhot" is the wrong rule and
///   would put the streetlamp's glow 2.5 units too high.
///
///   ROTATED BY THE OBJECT. Retail's one authored corona (MISS5 obj 35) caches
///   a delta of (0.020, -0.244, 0.091) against an object whose facing is the
///   quaternion (w 0.7071, z 0.7071) — 90 degrees about Z. ELECLGH's vhot 1 is
///   (-0.244, -0.020, 0.091), and rotating it by that facing gives the cached
///   delta exactly. An unrotated offset would be visibly wrong on any lamp
///   mounted at an angle.
///
///   NOT THE LIGHT OFFSET. P$Light carries its own offset field, and it is a
///   different thing: MISS6's streetlamps have offset (0,0,0) yet anchor their
///   coronas 3.98 units up. Across the whole campaign only 25 of 3325 P$Light
///   records have a non-zero offset at all, which is why anchoring on it left
///   99.2% of coronas sitting exactly on the object origin.
inline constexpr uint32_t kVHotLight = 1;

/// P$RenderType value NewDark adds for "draw the corona but not the model".
/// Absent from Thief 2's own `objectrendertype` enum (Normal 0, Not Rendered
/// 1, No Lightmap 2, Editor Only 3), so no retail object can carry it; it is
/// handled purely so fan missions that do behave correctly.
inline constexpr uint32_t kRenderTypeCoronaOnly = 4;

/// How a corona's drawn radius responds to viewer distance. The two models
/// disagree by up to 32x, so this is the single biggest lever on how coronas
/// read in a scene.
enum class CoronaDistanceModel : uint8_t {
    /// The ORIGINAL Thief 2 rule: interpolate radiusNear → radiusFar across
    /// the light's visible range. Retail's own numbers (2.0 at 0, 65.0 at 300)
    /// work out to a constant apparent angular size — a ~24 degree disc, 40%
    /// of screen height, at every distance.
    ///
    /// This is original, not a NewDark extension. Retail THIEF2.EXE v1.07
    /// carries the field labels itself:
    ///   [BIN: property-field label table @0x22e418-0x22e548, 0x40 stride
    ///    ("Corona", "radius up close", "radius at max dist",
    ///     "max. dist. visible", "alpha", "texture"), THIEF2.EXE v1.07
    ///    2000-08-24]
    /// NewDark 1.28's DromEd carries the identical strings and only appends
    /// "color" and "spot angle scale" — the two its release notes describe
    /// adding. So the growth belongs to the original engine and turning it off
    /// is a deliberate divergence.
    Engine,

    /// Constant world radius: the corona shrinks on screen exactly as the lamp
    /// does. This is what a veiling-glare disc actually does.
    ///
    /// Stiles-Holladay: the glare an eye or lens spreads around a source is
    /// L(theta) = k*E/theta^2, with E = I/d^2 the irradiance the source
    /// delivers. The visible edge of the glow is where L crosses the eye's
    /// threshold T:
    ///
    ///     k*(I/d^2) / theta_v^2 = T   =>   theta_v = sqrt(k*I/T) / d
    ///     world radius r = theta_v * d = sqrt(k*I/T)      <- no d
    ///
    /// So the world radius is INDEPENDENT of distance and proportional to
    /// sqrt(source intensity) — which is exactly the reasoning already in
    /// coronaBrightScale, applied to the other variable. The CIE 1999 glare
    /// equation's theta^-3 term would add a d^(1/3) growth, far weaker than
    /// the engine's linear one.
    ///
    /// NO SEPARATE DISTANCE DIMMING GOES WITH THIS, and adding one would be a
    /// bug. A fixed-world-size disc drawn at constant radiance already loses
    /// total flux as 1/d^2, because its solid angle does; multiplying by
    /// another 1/d^2 would give 1/d^4.
    ///
    /// Both reference implementations size billboards this way. HPL2's
    /// cBillboard holds a fixed world `mvSize` and varies only halo alpha
    /// (BillBoard.cpp); Godot's billboards are world-sized by default, with
    /// `FLAG_FIXED_SIZE` — hold screen size — an explicit opt-in meant for
    /// editor gizmos rather than light glows.
    Physical,
};

/// Where a corona's `offset` lives, which decides what has to happen to it
/// before it can be added to a world position.
enum class CoronaAnchor : uint8_t {
    /// The object's own origin. What you get when the model has no vhot 1 —
    /// the engine's fallback too, and wrong-looking on exactly the models that
    /// deserve a glow, so the load-time report counts these.
    Origin,
    /// Model space: rotated (and scaled) by the object's live transform every
    /// frame. The engine's construction — see kVHotLight.
    VHot,
    /// P$Light / P$AnimLight offset. Applied in world space, unrotated,
    /// matching how LightingSystem places the light itself. Only reached when
    /// the model has no vhot 1, and only 25 records campaign-wide are non-zero.
    LightOffset,
    /// An authored P$Corona's engine-cached world delta, used when we cannot
    /// re-derive the anchor from a vhot. Correct where the object stands, but
    /// frozen at the rotation it was cached with.
    CachedAnchor,
};

/// One corona, resolved at load. `objID` is the object it hangs off; the
/// world position is recomputed every frame from that object plus `offset`,
/// so a corona on a moving lamp moves with it.
struct CoronaDef {
    int32_t  objID       = 0;
    Vector3  offset{0.0f, 0.0f, 0.0f};
    CoronaAnchor anchor  = CoronaAnchor::Origin;
    /// Load-time P$Scale, applied to a model-space anchor before rotating it.
    /// A scaled lamp's vhot scales with its geometry — NewDark 1.28 lists
    /// "buggy transform for vhot ... when parent object is scaled" among its
    /// fixes, so the original got this wrong and the fixed behaviour is ours.
    Vector3  scale{1.0f, 1.0f, 1.0f};
    float    radiusNear  = 0.0f;
    float    radiusFar   = 0.0f;
    /// How far the corona is DRAWN. Culling only.
    float    maxDist     = 0.0f;
    /// Distance at which `radiusFar` is reached — the SIZE curve, which is a
    /// different thing from the draw range and must not be conflated with it.
    ///
    /// Splitting them fixes a real bug as well as letting the two be tuned
    /// apart. The synthesizer scales its radii by brightness, and when the
    /// interpolation ran against a brightness-scaled maxDist the scale
    /// cancelled outright:
    ///     radius(d) = 2s + (65s - 2s) * d/(300s) = 2s + 0.21*d
    /// A 12x brightness range then produced at most a 1.1x size difference
    /// past 100 units — every light in the level drew the same disc. With the
    /// size curve on its own un-scaled reference the s survives.
    float    sizeMaxDist = 0.0f;
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
    /// Anchor census, reported at load. A corona sitting on the object origin
    /// is the failure the vhot lookup exists to fix, so "how many still do"
    /// must be visible without a debugger.
    int anchorVHot     = 0;
    int anchorOrigin   = 0;
    int anchorOther    = 0;

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

/// Resolve an object's live orientation, in the same order coronaObjectPos
/// resolves its position: runtime state first, static P$Position otherwise.
/// A lamp that swings has to take its glow with it.
inline Quaternion coronaObjectRot(ObjectService *objSvc,
                                  const ObjectStateMap *objectStates,
                                  int32_t objID) {
    if (objectStates) {
        if (const ObjectState *st = objectStates->tryGet(objID))
            return st->orientation;
    }
    return objSvc->orientation(objID);
}

/// objectID -> the model the renderer draws for it.
///
/// Built from ObjectPropData::objects, which is the ONLY place a resolved
/// ModelName lands. allPlacements looks like the more convenient source and is
/// not: it is snapshotted before the inheritance walk that resolves ModelName,
/// so its `modelName` field is empty for every entry, and a lookup through it
/// silently answers "this object has no model" for the whole level.
inline std::unordered_map<int32_t, const ObjectPlacement *>
buildCoronaModelIndex(const ObjectPropData &objData) {
    std::unordered_map<int32_t, const ObjectPlacement *> idx;
    idx.reserve(objData.objects.size() * 2);
    for (const auto &o : objData.objects)
        idx[o.objID] = &o;
    return idx;
}

/// The model-space light attachment point of whatever model an object draws,
/// if it has one.
///
/// The rest pose is deliberate. A vhot on a moving part would strictly need
/// the object's live joint values, but no stock light model puts vhot 1 on a
/// moving part, and threading per-frame joints through the corona list to
/// serve zero objects would cost every corona a sub-object composition.
inline bool coronaModelVHot(const std::unordered_map<int32_t, const ObjectPlacement *> &modelIndex,
                            const std::unordered_map<std::string, ParsedBinMesh> &models,
                            int32_t objID, Vector3 &out) {
    auto pit = modelIndex.find(objID);
    if (pit == modelIndex.end() || pit->second->modelName[0] == '\0')
        return false;

    auto mit = models.find(pit->second->modelName);
    if (mit == models.end() || !mit->second.valid) return false;

    return findVHotModelSpace(mit->second, kVHotLight, nullptr, 0, out);
}

/// Load-time P$Scale, or (1,1,1) for an object we have no placement for.
inline Vector3 coronaObjectScale(const ObjectPropData &objData, int32_t objID) {
    auto pit = objData.allPlacements.find(objID);
    if (pit == objData.allPlacements.end()) return Vector3(1.0f, 1.0f, 1.0f);
    return Vector3(pit->second.scaleX, pit->second.scaleY, pit->second.scaleZ);
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
                                 const ObjectPropData &objData,
                                 const std::unordered_map<std::string, ParsedBinMesh> &models,
                                 CoronaRuntime &out) {
    if (!propSvc || !objSvc) return;

    const auto modelIndex = detail::buildCoronaModelIndex(objData);

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
        // An authored record's size curve IS its draw range — the designer
        // picked both radii against that one distance, and fan-mission authors
        // demonstrably tune it (8 to 64 units across the missions surveyed).
        // Only the synthesizer separates them.
        def.sizeMaxDist = def.maxDist;
        def.alpha      = pc.alpha;
        def.flags      = pc.flags;
        def.color      = detail::coronaLightColor(propSvc, objID);

        char tex[17] = {};
        std::memcpy(tex, pc.texture, 16);
        def.texture = tex;

        // Prefer re-deriving the anchor from the model's light vhot over the
        // engine's cached one. They agree where both exist — retail's one
        // authored corona reproduces its cached delta exactly this way — but
        // the cache is a world position frozen at bake time, so only the vhot
        // survives the object being rotated or moved.
        def.scale = detail::coronaObjectScale(objData, objID);
        Vector3 vhot;
        if (detail::coronaModelVHot(modelIndex, models, objID, vhot)) {
            def.offset = vhot;
            def.anchor = CoronaAnchor::VHot;
        } else {
            // No vhot to re-derive from: fall back to the cached anchor,
            // stored as a difference from the object's origin so the glow at
            // least follows the object rather than staying behind.
            const Vector3 objPos =
                detail::coronaObjectPos(objSvc, objectStates, objID);
            const Vector3 cached(pc.posX, pc.posY, pc.posZ);
            Vector3 delta = cached - objPos;
            // Guard against a stale cache from a since-moved object: a huge
            // delta is not a vhot, and honouring it would strand the glow
            // across the level. Vhots live inside the model's bounding box.
            if (glm::length(delta) > 16.0f)
                delta = Vector3(0.0f, 0.0f, 0.0f);
            def.offset = delta;
            def.anchor = (glm::length(delta) > 1e-4f) ? CoronaAnchor::CachedAnchor
                                                      : CoronaAnchor::Origin;
        }

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
                              const std::unordered_map<std::string, ParsedBinMesh> &models,
                              const std::unordered_map<int16_t, LightSource> &lightSources,
                              CoronaRuntime &out) {
    if (!propSvc || !objSvc) return;

    // Objects the renderer draws, and the model each one draws — the same set,
    // read two ways.
    const auto modelIndex = detail::buildCoronaModelIndex(objData);
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

    auto emit = [&](int32_t objID, float brightness, const Vector3 &lightOffset,
                    int16_t animLightNum, float animMaxBright) {
        const float scale = detail::coronaBrightScale(brightness);
        if (scale <= 0.0f) return; // brightness 0 = the light is off

        CoronaDef def;
        def.objID        = objID;

        // The model's light vhot is the engine's own answer to "where does
        // this lamp glow from", and it is usually nowhere near the origin: a
        // street lantern's is 3.99 units up its post, a wall torch's about 1.4.
        // Fall back to the light's authored offset — which 3300 of 3325 retail
        // records leave at zero — only when the model names no such point.
        def.scale = detail::coronaObjectScale(objData, objID);
        Vector3 vhot;
        if (detail::coronaModelVHot(modelIndex, models, objID, vhot)) {
            def.offset = vhot;
            def.anchor = CoronaAnchor::VHot;
        } else {
            def.offset = lightOffset;
            def.anchor = (glm::length(lightOffset) > 1e-4f)
                             ? CoronaAnchor::LightOffset
                             : CoronaAnchor::Origin;
        }

        // Radii scale with brightness; the size curve's reference distance
        // does NOT, or the scale cancels straight back out — see
        // CoronaDef::sizeMaxDist. The draw range keeps scaling, because a
        // brighter lamp genuinely should be visible from further away.
        def.radiusNear   = kSynthTemplate.radiusNear * scale;
        def.radiusFar    = kSynthTemplate.radiusFar  * scale;
        def.sizeMaxDist  = kSynthTemplate.maxDist;
        def.maxDist      = kRetailTemplate.maxDist   * scale;
        def.alpha        = kSynthTemplate.alpha;
        def.color        = detail::coronaLightColor(propSvc, objID);
        def.texture      = kSynthTemplate.texture;
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

    rt.anchorVHot = rt.anchorOrigin = rt.anchorOther = 0;
    for (const auto &d : rt.defs) {
        if (d.anchor == CoronaAnchor::VHot)        ++rt.anchorVHot;
        else if (d.anchor == CoronaAnchor::Origin) ++rt.anchorOrigin;
        else                                       ++rt.anchorOther;
    }
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
    /// Engine rule or veiling-glare physics — see CoronaDistanceModel. Note
    /// this changes only the drawn RADIUS; `maxDist` still bounds visibility
    /// either way, so a light stops being drawn at the same range.
    CoronaDistanceModel distanceModel = CoronaDistanceModel::Physical;
    /// Physical model only: the viewing distance whose size is frozen and held
    /// at every range. Bigger = bigger coronas everywhere, in proportion.
    /// 20 units is a room's width in this game's scale, so the default reads
    /// as "the size it looked from across the room".
    float physicalRefDistance = 20.0f;
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

        // A vhot anchor is model-space, so it has to ride the object's live
        // transform; every other anchor is already a world-space delta.
        Vector3 pos = detail::coronaObjectPos(objSvc, objectStates, def.objID);
        if (def.anchor == CoronaAnchor::VHot) {
            const Quaternion rot =
                detail::coronaObjectRot(objSvc, objectStates, def.objID);
            pos += rot * (def.offset * def.scale);
        } else {
            pos += def.offset;
        }

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

        // World radius. Both models walk the SAME size curve; they differ only
        // in where on it they stand.
        //
        //   engine    evaluate at the viewer's actual distance, so the radius
        //             grows and the billboard holds a constant apparent size.
        //   physical  evaluate once at a fixed reference distance and hold
        //             that world radius forever, so the corona shrinks on
        //             screen exactly as its lamp does.
        //
        // The reference distance is not a fudge — it is the free parameter the
        // physics leaves open. The veiling-glare derivation fixes the SHAPE of
        // the law (r independent of d, proportional to sqrt(I)) but its
        // constant is sqrt(k*I/T), and T is the eye's adaptation threshold.
        // Something has to set it, and "the size this corona had at a normal
        // viewing distance" is the honest way to.
        //
        // Anchoring on `radiusNear` instead — the first thing tried — was
        // wrong: that is the curve's value at d = 0, the smallest it ever
        // takes, so every corona shrank including the one at arm's length.
        const float sizeAt =
            (cfg.distanceModel == CoronaDistanceModel::Physical)
                ? cfg.physicalRefDistance
                : dist;
        const float t = (def.sizeMaxDist > 1e-4f)
                            ? std::clamp(sizeAt / def.sizeMaxDist, 0.0f, 1.0f)
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
