/******************************************************************************
 *
 *    This file is part of the Darkness engine
 *
 *    Shared value types for the IWorldQuery interface. These are lightweight
 *    snapshot types decoupled from OPDE service internals — downstream
 *    subsystems (AI, audio, scripts) see only these types, never raw
 *    service pointers.
 *
 *****************************************************************************/

#ifndef __WORLDQUERYTYPES_H
#define __WORLDQUERYTYPES_H

#include <cstdint>

#include "Plane.h"
#include "Quaternion.h"
#include "Vector3.h"

namespace Darkness {

// ============================================================================
// ID type aliases — zero-cost documentation of intent
// ============================================================================

/// Object/entity identifier (matches ObjectService int objID)
using EntityID = int32_t;

/// Room number (matches Room::getRoomID())
using RoomID = int16_t;

/// Portal identifier (matches RoomPortal mID)
using PortalID = int32_t;

/// Link relation flavor (matches LinkService flavor param)
using LinkFlavorID = int;

// ============================================================================
// Cached handles — resolved once at init, used for O(1) lookups in hot loops
// ============================================================================

/// Opaque handle to a Property object. Avoids repeated string→Property* hash
/// lookups in tight loops (AI checking properties on 100+ entities, acoustic
/// propagation querying room properties per sound event per frame).
/// Constructed via IWorldQuery::resolveProperty().
struct PropertyHandle {
    void *_internal = nullptr; // Property* — opaque to consumers
    explicit operator bool() const { return _internal != nullptr; }
};

/// Opaque handle to a Relation object. Same caching pattern as PropertyHandle.
/// Constructed via IWorldQuery::resolveRelation().
struct RelationHandle {
    void *_internal = nullptr; // Relation* — opaque to consumers
    explicit operator bool() const { return _internal != nullptr; }
};

// ============================================================================
// Snapshot value types — consumers hold copies, never internal pointers
// ============================================================================

/// Lightweight snapshot of a room portal's essential data.
/// Copied from RoomPortal internals so consumers are decoupled from service
/// lifetimes. Used by acoustic propagation BFS and AI room scanning.
struct PortalInfo {
    PortalID id;
    RoomID nearRoom, farRoom;
    Vector3 center;
    Plane plane;
};

/// Value copy of a link record. Consumers iterate these without touching
/// the Relation's internal LinkMap.
struct LinkInfo {
    uint32_t id;
    EntityID src, dst;
    int flavor;
};

/// Result of a raycast query (stub until Task 16).
struct RayHit {
    Vector3 point;
    Vector3 normal;
    float distance;
    EntityID hitEntity; // 0 if world geometry hit
};

/// Axis-aligned bounding box.
struct BBox {
    Vector3 min, max;

    Vector3 center() const {
        return {(min.x + max.x) * 0.5f, (min.y + max.y) * 0.5f,
                (min.z + max.z) * 0.5f};
    }
};

} // namespace Darkness

#endif // __WORLDQUERYTYPES_H
