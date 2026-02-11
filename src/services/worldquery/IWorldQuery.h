/******************************************************************************
 *
 *    This file is part of the Darkness engine
 *
 *    IWorldQuery — the read-only facade through which every downstream
 *    subsystem (AI, audio, scripts, physics) accesses world state.
 *
 *    No subsystem reaches into another's internals; mutations go through
 *    the owning subsystem's API. This decoupling is what makes every module
 *    independently replaceable (e.g. swap DarkPhysics for Jolt, swap
 *    ClassicAIBrain for LLMAgentBrain, etc.).
 *
 *    See .claude/IWorldQuery_PLAN.md for design rationale.
 *
 *****************************************************************************/

#ifndef __IWORLDQUERY_H
#define __IWORLDQUERY_H

#include <cstring>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "WorldQueryTypes.h"

namespace Darkness {

/** @brief Pure virtual read-only interface for querying world state.
 *
 * Every downstream module receives `const IWorldQuery&` — read-only access.
 * Mutations go through the owning subsystem's API (ObjectService, LinkService,
 * etc.). The renderer accesses services directly (it is the host, not a
 * downstream consumer).
 *
 * Methods are grouped into:
 *  - Entity queries (position, orientation, name, bounds)
 *  - Property access (typed data with inheritance resolution)
 *  - Link queries (relation traversal)
 *  - Room/portal topology (spatial connectivity)
 *  - Spatial queries (radius, frustum — stubs until SpatialIndex)
 *  - Environment (light level, raycast — stubs until later phases)
 *
 * Two API styles coexist for room/portal/link queries:
 *  - Vector-returning methods for convenience (allocates)
 *  - Callback/visitor methods for hot paths (zero-allocation)
 */
class IWorldQuery {
public:
    virtual ~IWorldQuery() = default;

    // ========================================================================
    // Entity queries
    // ========================================================================

    /// Does the entity exist in the object system?
    virtual bool exists(EntityID id) const = 0;

    /// Entity position in world space (0,0,0 if abstract or not positioned)
    virtual Vector3 getPosition(EntityID id) const = 0;

    /// Entity orientation (identity if not oriented)
    virtual Quaternion getOrientation(EntityID id) const = 0;

    /// Entity symbolic name (empty string if unnamed or invalid)
    virtual std::string getName(EntityID id) const = 0;

    /// Axis-aligned bounding box in world space.
    /// Stub: returns unit box at entity position until .bin bounds available.
    virtual BBox getBounds(EntityID id) const = 0;

    // ========================================================================
    // Property access — string-based (convenience) and handle-based (hot path)
    // ========================================================================

    /// Resolve a property name to an opaque handle for O(1) lookups.
    /// Call once at subsystem init, reuse in tight loops.
    virtual PropertyHandle resolveProperty(
        const std::string &propName) const = 0;

    /// Check if entity has property (with inheritance resolution)
    virtual bool hasProperty(EntityID id,
                             const std::string &propName) const = 0;
    virtual bool hasProperty(EntityID id,
                             const PropertyHandle &handle) const = 0;

    /// Check if entity directly owns property (no inheritance)
    virtual bool ownsProperty(EntityID id,
                              const std::string &propName) const = 0;
    virtual bool ownsProperty(EntityID id,
                              const PropertyHandle &handle) const = 0;

    /// Get all entity IDs that directly store a given property
    virtual std::vector<EntityID> getAllWithProperty(
        const std::string &propName) const = 0;

    /** Get typed property value with inheritance resolution.
     * Non-virtual template (NVI pattern) — calls virtual getRawPropertyData().
     * @return std::nullopt if entity doesn't have the property or data too
     * small */
    template <typename T>
    std::optional<T> getProperty(EntityID id,
                                 const std::string &propName) const {
        PropertyHandle h = resolveProperty(propName);
        if (!h)
            return std::nullopt;
        return getProperty<T>(id, h);
    }

    /** Get typed property via cached handle (O(1) lookup, no hash map search).
     * This is the hot-path variant for AI and acoustic propagation. */
    template <typename T>
    std::optional<T> getProperty(EntityID id,
                                 const PropertyHandle &handle) const {
        size_t size = 0;
        const uint8_t *data = getRawPropertyData(id, handle, size);
        if (!data || size < sizeof(T))
            return std::nullopt;
        T result;
        std::memcpy(&result, data, sizeof(T));
        return result;
    }

    // ========================================================================
    // Link queries — string-based and handle-based
    // ========================================================================

    /// Resolve a relation name to an opaque handle for O(1) lookups.
    virtual RelationHandle resolveRelation(
        const std::string &relName) const = 0;

    /// Get all links from src in the named relation (dst=0 for any destination)
    virtual std::vector<LinkInfo> getLinks(
        EntityID src, const std::string &relName,
        EntityID dst = 0) const = 0;
    virtual std::vector<LinkInfo> getLinks(EntityID src,
                                           const RelationHandle &handle,
                                           EntityID dst = 0) const = 0;

    /// Get back-links (incoming links to dst) using the inverse relation
    virtual std::vector<LinkInfo> getBackLinks(
        EntityID dst, const std::string &relName,
        EntityID src = 0) const = 0;
    virtual std::vector<LinkInfo> getBackLinks(EntityID dst,
                                               const RelationHandle &handle,
                                               EntityID src = 0) const = 0;

    // ========================================================================
    // Room/portal topology
    // ========================================================================

    /// Find which room contains the given world-space point (or nullptr/invalid
    /// if outside all rooms)
    virtual RoomID getRoomAt(const Vector3 &pos) const = 0;

    /// Get rooms adjacent to the given room (connected via portals)
    virtual std::vector<RoomID> getAdjacentRooms(RoomID room) const = 0;

    /// Get portal snapshots for a room
    virtual std::vector<PortalInfo> getPortals(RoomID room) const = 0;

    /// Portal open fraction (0.0=closed, 1.0=fully open).
    /// Stub: returns 1.0 until door state is implemented in Phase 2.
    virtual float getPortalOpenFraction(PortalID portal) const = 0;

    /// Get entities in a room (from the specified id set)
    virtual std::vector<EntityID> queryRoom(RoomID room,
                                            size_t idset = 0) const = 0;

    // --- Callback variants (zero-allocation for acoustic/AI hot paths) ---

    /// Visit each portal of a room without allocating a vector.
    /// Used by acoustic propagation BFS through portal graph.
    virtual void forEachPortal(
        RoomID room,
        const std::function<void(const PortalInfo &)> &callback) const = 0;

    /// Visit each adjacent room without allocating a vector.
    virtual void forEachAdjacentRoom(
        RoomID room,
        const std::function<void(RoomID)> &callback) const = 0;

    /// Visit each entity in a room without allocating a vector.
    virtual void forEachEntityInRoom(
        RoomID room,
        const std::function<void(EntityID)> &callback,
        size_t idset = 0) const = 0;

    // ========================================================================
    // Spatial queries (stubs until Task 15 SpatialIndex)
    // ========================================================================

    /// Find all entities within radius of a point
    virtual std::vector<EntityID> queryRadius(const Vector3 &center,
                                              float radius) const = 0;

    /// Find all entities within a view frustum
    virtual std::vector<EntityID> queryFrustum(
        const Vector3 &origin, const Vector3 &forward, float fovRadians,
        float aspect, float nearDist, float farDist) const = 0;

    // ========================================================================
    // Environment queries (stubs until later phases)
    // ========================================================================

    /// Light level at a world position (0.0=dark, 1.0=bright).
    /// Stub: returns 1.0 until Phase 6 lighting system.
    virtual float getLightLevel(const Vector3 &pos) const = 0;

    /// Cast a ray and find the first hit.
    /// Stub: returns false until Task 16.
    virtual bool raycast(const Vector3 &from, const Vector3 &to,
                         RayHit &hit) const = 0;

protected:
    /** Virtual raw property data access — the implementation point for the
     * NVI getProperty<T>() template above. Subclasses override this to
     * delegate to their property storage system.
     * @param id Entity to query
     * @param handle Cached property handle (from resolveProperty)
     * @param outSize Filled with byte count of returned data
     * @return Pointer to raw bytes, or nullptr if not found */
    virtual const uint8_t *getRawPropertyData(
        EntityID id, const PropertyHandle &handle,
        size_t &outSize) const = 0;
};

} // namespace Darkness

#endif // __IWORLDQUERY_H
