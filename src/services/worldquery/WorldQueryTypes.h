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

#include "DarknessMath.h"

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

/// Result of a raycast query.
/// Why `raycastWorld` returned what it returned.
///
/// WHY THIS EXISTS: raycastWorld's `bool` conflates two very different
/// answers — "I proved the segment is clear" and "I could not evaluate the
/// segment". It returns false when the ray ORIGIN is outside every WR cell,
/// when the cell budget is exhausted, on degenerate geometry, and on invalid
/// portal targets. A caller that reads false as "clear line of sight" will
/// see through walls in exactly those cases.
///
/// Exactly ONE path proves clear (`Clear`): the ray ending inside a cell
/// without crossing an exit plane. Two prove blocked (`Hit`). Everything
/// else is UNPROVEN — so visibility consumers should WHITELIST `Clear`
/// rather than blacklist known failures.
///
/// Only `raycastWorld` sets this; other RayHit producers (object/physics
/// raycasts) leave it `Unset`.
enum class RayStatus : uint8_t {
    Unset = 0,
    Hit,                 ///< proven blocked; hit point/normal are valid
    Clear,               ///< PROVEN clear — ray ended inside a cell
    ZeroLength,          ///< from ~= to; nothing between by definition
    // ── everything below is UNPROVEN: false does NOT mean "clear" ──
    NoStartCell,         ///< origin outside every cell — ray never traced
    DegenerateGeom,      ///< no valid exit plane found
    DiscardNoPolygon,    ///< exit plane crossed but no polygon contained the
                         ///< hit point; discarded to match the original
                         ///< engine — traversal STOPS here
    InvalidPortalTarget, ///< portal's target cell out of range ("treat as
                         ///< solid" per the traversal's own comment)
    CellBudget,          ///< MAX_RAY_CELLS exceeded before resolving
    InvalidCell,         ///< current cell went out of range mid-traversal
};

/// True only when the raycast actually resolved the segment. `!proven()`
/// means the bool answer is a guess and must not be read as visibility.
inline bool rayStatusProven(RayStatus s) {
    return s == RayStatus::Hit || s == RayStatus::Clear ||
           s == RayStatus::ZeroLength;
}

struct RayHit {
    // Every member carries an initializer on purpose. Vector3 is glm::vec3 and
    // GLM_FORCE_CTOR_INIT is deliberately NOT defined in this build, so a bare
    // `Vector3 point;` is garbage — and `RayHit hit;` would hand a producer's
    // early-return path straight to a consumer as uninitialized stack.
    //
    // This matters because RayHit is an out-param with many early returns:
    // raycastWorld alone has seven `return false` paths, and each sets only
    // `status`. Callers today all gate on the bool, so nothing reads the
    // garbage — but the next one to forget is reading a stack address. Fix the
    // struct rather than each producer; there is no default worth omitting.
    Vector3 point{0.0f};
    Vector3 normal{0.0f};
    float distance = 0.0f;
    EntityID hitEntity = 0; // 0 if world geometry hit
    int32_t textureIndex = -1; // WR polygon texture index (-1 = unknown/no texture)
    int32_t cellIdx = -1;      // WR cell where hit occurred (-1 = unknown)
    int32_t polyIdx = -1;      // WR polygon index within cell (-1 = unknown)
    /// Set by raycastWorld only (see RayStatus). Unset from other producers.
    RayStatus status = RayStatus::Unset;
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
