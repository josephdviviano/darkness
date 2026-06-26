/******************************************************************************
 *
 *    This file is part of the Darkness engine
 *
 *    SelfLitProducer — per-frame walk of objects tagged with P$SelfLit
 *    (schema label "DynamicLight") that pushes each into the
 *    DynamicLightList for the renderer to consume.
 *
 *    Mirrors the reference engine's SelfLitUpdateAll() pattern: for each
 *    concrete object with the property, resolve its current position,
 *    fold in P$LightColor (HSV) if set, and add to the dynamic list at a
 *    fixed engine-constant radius. Unlike P$Light (which is baked into
 *    the lightmap at level generation), P$SelfLit is intentionally NOT
 *    baked — the level designer tags objects whose brightness must
 *    appear at runtime rather than at bake time.
 *
 *    Called once per render frame, right after DynamicLightList::reset(),
 *    before renderObjects(). The whole list is rebuilt each frame; nothing
 *    persists across frames inside this module.
 *
 *    Future generalization: when the project switches to "fully dynamic"
 *    lighting (all P$Light objects evaluated per-frame, no lightmap bake),
 *    add a parallel walk of P$Light here using the same shape.
 *
 *****************************************************************************/

#pragma once

#include <cstdint>

#include "DarknessMath.h"
#include "DynamicLightList.h"
#include "RenderParamsParser.h"     // for hsbToRgb
#include "object/ObjectService.h"
#include "property/DarkPropertyDefs.h"
#include "property/PropertyService.h"
#include "property/TypedProperty.h"
#include "worldquery/ObjectState.h"

namespace Darkness {

/// Quick-cull radius for SelfLit dynamic lights. Matches the reference
/// engine's hardware-render value (`g_lgd3d ? 10.0 : 4.0`). DynamicLight
/// uses this as the "don't bother evaluating beyond this distance" cutoff
/// — points farther than this from the light source skip the visibility
/// raycast + falloff sum. The contribution itself is still 1/r-attenuated
/// inside the cull radius.
inline constexpr float kSelfLitRadius = 10.0f;

/// Walk every concrete object with P$SelfLit and add it to `out` as a
/// dynamic light. Skips objects with brightness == 0 (explicit disable).
/// Skips abstract objects (negative IDs) — only instantiated lights matter.
///
/// Position resolution order:
///   1. objectStates->tryGet(objID)->position  (runtime override — moving
///      doors/platforms, frob-grabbed lit objects, tweq-animated lamps)
///   2. ObjectService::position(objID)         (static P$Position fallback)
///
/// Color resolution:
///   - If P$LightColor (hue, saturation) is set, modulate via hsbToRgb().
///   - Otherwise default to white (1, 1, 1).
///
/// Brightness is the raw int32 from P$SelfLit, in the same RGB radiance
/// units as WRStaticLight::bright. The shader applies 1/r attenuation
/// during the per-object lighting pass.
///
/// Inheritance note: only objects that DIRECTLY own a P$SelfLit record
/// contribute — `getAllObjectsWithProperty` walks the storage's stored
/// IDs, not the inheritance chain. Concretes that merely inherit SelfLit
/// from an archetype are skipped. This intentionally matches the
/// reference engine's `SelfLitUpdateAll`, which iterates the property's
/// own (direct-storage) iterator. If "fully dynamic" lighting later
/// extends this producer to P$Light, that walk WILL need inheritance
/// resolution because P$Light is heavily inherited from archetypes —
/// don't carry this iteration shape over blindly.
inline void produceSelfLitLights(PropertyService *propSvc,
                                 ObjectService *objSvc,
                                 const ObjectStateMap *objectStates,
                                 DynamicLightList &out) {
    if (!propSvc || !objSvc)
        return;

    auto ids = getAllObjectsWithProperty(propSvc, "SelfLit");
    for (int objID : ids) {
        // Abstract objects (archetypes) have negative IDs and no world
        // position — skip them. Only their concrete instances matter.
        if (objID <= 0)
            continue;

        PropSelfLit sl{};
        if (!getTypedProperty<PropSelfLit>(propSvc, "SelfLit", objID, sl))
            continue;
        if (sl.brightness == 0)
            continue;  // explicit "off" — script may toggle this at runtime

        Vector3 color(1.0f, 1.0f, 1.0f);
        PropLightColor lc{};
        if (getTypedProperty<PropLightColor>(propSvc, "LightColor", objID, lc))
            color = hsbToRgb(lc.hue, lc.saturation);

        Vector3 pos;
        const ObjectState *st =
            objectStates ? objectStates->tryGet(objID) : nullptr;
        if (st)
            pos = st->position;
        else
            pos = objSvc->position(objID);

        // Bright is RGB × scalar brightness — matches WRStaticLight::bright
        // convention so the ObjectIlluminator shader path applies the same
        // 1/r falloff that static lights use.
        Vector3 bright = color * static_cast<float>(sl.brightness);
        out.add(pos, bright, kSelfLitRadius);
    }
}

}  // namespace Darkness
