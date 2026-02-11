/******************************************************************************
 *
 *    This file is part of the Darkness engine
 *
 *    ObjSysWorldState — concrete IWorldQuery implementation that wraps the
 *    OPDE service stack (ObjectService, PropertyService, LinkService,
 *    RoomService). This is the primary world-state facade for downstream
 *    subsystems.
 *
 *    Spatial queries (queryRadius, queryAABB, queryFrustum) are backed by a
 *    lazily-populated SpatialIndex hash grid. Raycast and light level queries
 *    return documented defaults until filled in by later tasks.
 *
 *    See .claude/IWorldQuery_PLAN.md for design rationale.
 *
 *****************************************************************************/

#ifndef __OBJSYSWORLDSTATE_H
#define __OBJSYSWORLDSTATE_H

#include <functional>

#include "IWorldQuery.h"
#include "SpatialIndex.h"
#include "ServiceCommon.h"
#include "link/LinkCommon.h"
#include "link/LinkService.h"
#include "link/Relation.h"
#include "object/ObjectService.h"
#include "property/Property.h"
#include "property/PropertyService.h"
#include "property/TypedProperty.h"
#include "room/Room.h"
#include "room/RoomPortal.h"
#include "room/RoomService.h"

namespace Darkness {

class ObjSysWorldState : public IWorldQuery {
public:
    ObjSysWorldState(ObjectService *objSvc, PropertyService *propSvc,
                     LinkService *linkSvc, RoomService *roomSvc)
        : mObjSvc(objSvc), mPropSvc(propSvc), mLinkSvc(linkSvc),
          mRoomSvc(roomSvc) {}

    // ========================================================================
    // Entity queries
    // ========================================================================

    bool exists(EntityID id) const override { return mObjSvc->exists(id); }

    Vector3 getPosition(EntityID id) const override {
        return mObjSvc->position(id);
    }

    Quaternion getOrientation(EntityID id) const override {
        return mObjSvc->orientation(id);
    }

    std::string getName(EntityID id) const override {
        return mObjSvc->getName(id);
    }

    BBox getBounds(EntityID id) const override {
        // Stub: unit box at entity position until .bin bounds available
        Vector3 pos = mObjSvc->position(id);
        return {{pos.x - 0.5f, pos.y - 0.5f, pos.z - 0.5f},
                {pos.x + 0.5f, pos.y + 0.5f, pos.z + 0.5f}};
    }

    // ========================================================================
    // Property access
    // ========================================================================

    PropertyHandle resolveProperty(const std::string &propName) const override {
        PropertyHandle h;
        h._internal = static_cast<void *>(mPropSvc->getProperty(propName));
        return h;
    }

    bool hasProperty(EntityID id,
                     const std::string &propName) const override {
        return Darkness::hasProperty(mPropSvc, propName, id);
    }

    bool hasProperty(EntityID id,
                     const PropertyHandle &handle) const override {
        auto *prop = static_cast<Property *>(handle._internal);
        if (!prop)
            return false;
        return prop->has(id);
    }

    bool ownsProperty(EntityID id,
                      const std::string &propName) const override {
        return Darkness::ownsProperty(mPropSvc, propName, id);
    }

    bool ownsProperty(EntityID id,
                      const PropertyHandle &handle) const override {
        auto *prop = static_cast<Property *>(handle._internal);
        if (!prop)
            return false;
        return prop->owns(id);
    }

    std::vector<EntityID>
    getAllWithProperty(const std::string &propName) const override {
        auto intVec = getAllObjectsWithProperty(mPropSvc, propName);
        // EntityID is int32_t, same as int on this platform, but be explicit
        std::vector<EntityID> result(intVec.begin(), intVec.end());
        return result;
    }

    // ========================================================================
    // Link queries
    // ========================================================================

    RelationHandle resolveRelation(const std::string &relName) const override {
        RelationHandle h;
        RelationPtr rel = mLinkSvc->getRelation(relName);
        // Store the raw pointer — RelationPtr is shared_ptr, kept alive by
        // LinkService for the lifetime of the relation
        h._internal = rel ? static_cast<void *>(rel.get()) : nullptr;
        return h;
    }

    std::vector<LinkInfo> getLinks(EntityID src, const std::string &relName,
                                   EntityID dst) const override {
        RelationHandle h = resolveRelation(relName);
        return getLinks(src, h, dst);
    }

    std::vector<LinkInfo> getLinks(EntityID src, const RelationHandle &handle,
                                   EntityID dst) const override {
        std::vector<LinkInfo> result;
        auto *rel = static_cast<Relation *>(handle._internal);
        if (!rel)
            return result;

        LinkQueryResultPtr qr = rel->getAllLinks(src, dst);
        if (!qr)
            return result;

        while (!qr->end()) {
            const Link &lnk = qr->next();
            result.push_back(
                {lnk.id(), static_cast<EntityID>(lnk.src()),
                 static_cast<EntityID>(lnk.dst()), lnk.flavor()});
        }
        return result;
    }

    std::vector<LinkInfo> getBackLinks(EntityID dst,
                                       const std::string &relName,
                                       EntityID src) const override {
        // Look up the inverse relation directly via the ~Name convention.
        // LinkService::createRelation() always registers both "Name" and "~Name".
        RelationHandle h = resolveRelation("~" + relName);
        if (!h)
            return {};
        return getLinks(dst, h, src);
    }

    std::vector<LinkInfo> getBackLinks(EntityID dst,
                                       const RelationHandle &handle,
                                       EntityID src) const override {
        std::vector<LinkInfo> result;
        auto *rel = static_cast<Relation *>(handle._internal);
        if (!rel)
            return result;

        // Use the inverse relation for back-link queries
        Relation *inv = rel->inverse();
        if (!inv)
            return result;

        LinkQueryResultPtr qr = inv->getAllLinks(dst, src);
        if (!qr)
            return result;

        while (!qr->end()) {
            const Link &lnk = qr->next();
            result.push_back(
                {lnk.id(), static_cast<EntityID>(lnk.src()),
                 static_cast<EntityID>(lnk.dst()), lnk.flavor()});
        }
        return result;
    }

    // ========================================================================
    // Room/portal topology
    // ========================================================================

    RoomID getRoomAt(const Vector3 &pos) const override {
        if (!mRoomSvc->isLoaded())
            return -1;
        Room *room = mRoomSvc->roomFromPoint(pos);
        return room ? room->getRoomID() : static_cast<RoomID>(-1);
    }

    std::vector<RoomID> getAdjacentRooms(RoomID room) const override {
        std::vector<RoomID> result;
        forEachAdjacentRoom(room, [&](RoomID adj) { result.push_back(adj); });
        return result;
    }

    std::vector<PortalInfo> getPortals(RoomID room) const override {
        std::vector<PortalInfo> result;
        forEachPortal(room,
                      [&](const PortalInfo &pi) { result.push_back(pi); });
        return result;
    }

    float getPortalOpenFraction(PortalID /*portal*/) const override {
        // Stub: all portals fully open until door state in Phase 2
        return 1.0f;
    }

    std::vector<EntityID> queryRoom(RoomID room,
                                    size_t idset) const override {
        std::vector<EntityID> result;
        Room *r = findRoomByRoomID(room);
        if (!r)
            return result;

        auto ids = r->getObjectIDs(idset);
        result.assign(ids.begin(), ids.end());
        return result;
    }

    // --- Callback variants ---

    void forEachPortal(
        RoomID room,
        const std::function<void(const PortalInfo &)> &callback)
        const override {
        Room *r = findRoomByRoomID(room);
        if (!r)
            return;

        for (uint32_t i = 0; i < r->getPortalCount(); ++i) {
            RoomPortal *portal = r->getPortal(i);
            if (!portal)
                continue;

            PortalInfo pi;
            pi.id = portal->getPortalID();
            pi.center = portal->getCenter();
            pi.plane = portal->getPlane();

            Room *near = portal->getNearRoom();
            Room *far = portal->getFarRoom();
            pi.nearRoom = near ? near->getRoomID() : static_cast<RoomID>(-1);
            pi.farRoom = far ? far->getRoomID() : static_cast<RoomID>(-1);

            callback(pi);
        }
    }

    void forEachAdjacentRoom(
        RoomID room,
        const std::function<void(RoomID)> &callback) const override {
        Room *r = findRoomByRoomID(room);
        if (!r)
            return;

        for (uint32_t i = 0; i < r->getPortalCount(); ++i) {
            RoomPortal *portal = r->getPortal(i);
            if (!portal)
                continue;

            Room *far = portal->getFarRoom();
            if (far) {
                callback(far->getRoomID());
            }
        }
    }

    void forEachEntityInRoom(
        RoomID room, const std::function<void(EntityID)> &callback,
        size_t idset) const override {
        Room *r = findRoomByRoomID(room);
        if (!r)
            return;

        auto ids = r->getObjectIDs(idset);
        for (int id : ids) {
            callback(static_cast<EntityID>(id));
        }
    }

    // ========================================================================
    // Spatial queries — backed by lazily-populated SpatialIndex hash grid
    // ========================================================================

    std::vector<EntityID> queryRadius(const Vector3 &center,
                                      float radius) const override {
        ensureSpatialIndex();
        return mSpatialIndex.queryRadius(center, radius);
    }

    std::vector<EntityID> queryAABB(const BBox &box) const override {
        ensureSpatialIndex();
        return mSpatialIndex.queryAABB(box);
    }

    std::vector<EntityID>
    queryFrustum(const Vector3 &origin, const Vector3 &forward,
                 float fovRadians, float aspect, float nearDist,
                 float farDist) const override {
        ensureSpatialIndex();
        BBox frustumBox =
            computeFrustumAABB(origin, forward, fovRadians, aspect,
                               nearDist, farDist);
        return mSpatialIndex.queryAABB(frustumBox);
    }

    // ========================================================================
    // Environment queries (stubs until later phases)
    // ========================================================================

    float getLightLevel(const Vector3 & /*pos*/) const override {
        return 1.0f; // Stub — will query lightmap/dynamic light in Phase 6
    }

    bool raycast(const Vector3 &from, const Vector3 &to,
                 RayHit &hit) const override {
        if (mRaycaster)
            return mRaycaster(from, to, hit);
        return false; // No raycaster injected — fallback
    }

    // ========================================================================
    // Raycaster injection — renderer injects a lambda capturing WR geometry
    // so the services layer has no dependency on WRParsedData / bgfx.
    // ========================================================================

    using RaycastFn = std::function<bool(const Vector3 &, const Vector3 &, RayHit &)>;

    /// Inject the raycaster implementation. Call once after WR data is parsed.
    void setRaycaster(RaycastFn fn) { mRaycaster = std::move(fn); }

protected:
    const uint8_t *getRawPropertyData(EntityID id,
                                      const PropertyHandle &handle,
                                      size_t &outSize) const override {
        auto *prop = static_cast<Property *>(handle._internal);
        if (!prop) {
            outSize = 0;
            return nullptr;
        }

        // Resolve inheritance: walk archetype + MetaProp chain
        int effectiveID = prop->getEffectiveID(id);
        if (effectiveID == 0) {
            outSize = 0;
            return nullptr;
        }

        DataStorage *storage = prop->getStorage();
        if (!storage) {
            outSize = 0;
            return nullptr;
        }

        return storage->getRawData(effectiveID, outSize);
    }

private:
    /// Find a Room by its room number (not object ID).
    /// Linear scan of all rooms — room counts are small (~50-200).
    Room *findRoomByRoomID(RoomID roomID) const {
        if (!mRoomSvc->isLoaded())
            return nullptr;
        return mRoomSvc->getRoomByID(roomID);
    }

    /// Populate the spatial index on first use. Iterates all entities with
    /// P$Position and inserts concrete objects (ID > 0) into the grid.
    void ensureSpatialIndex() const {
        if (mSpatialPopulated)
            return;
        mSpatialPopulated = true;

        auto ids = getAllObjectsWithProperty(mPropSvc, "Position");
        for (int id : ids) {
            if (id <= 0)
                continue; // skip archetypes
            Vector3 pos = mObjSvc->position(id);
            mSpatialIndex.insert(static_cast<EntityID>(id), pos);
        }
    }

    /// Compute a conservative AABB enclosing a view frustum.
    /// Used by queryFrustum() to reduce the query to an AABB test.
    static BBox computeFrustumAABB(const Vector3 &origin,
                                   const Vector3 &forward,
                                   float fovRadians, float aspect,
                                   float nearDist, float farDist) {
        // Build orthonormal basis from forward direction
        // Dark Engine is Z-up — use Z-up as default, fall back to Y if
        // forward is nearly vertical
        Vector3 up = {0.0f, 0.0f, 1.0f};
        float dot = forward.x * up.x + forward.y * up.y + forward.z * up.z;
        if (std::fabs(dot) > 0.99f)
            up = {0.0f, 1.0f, 0.0f}; // fallback for looking straight up/down

        // right = forward × up (normalized)
        Vector3 right = {
            forward.y * up.z - forward.z * up.y,
            forward.z * up.x - forward.x * up.z,
            forward.x * up.y - forward.y * up.x
        };
        float rightLen = std::sqrt(right.x * right.x + right.y * right.y +
                                   right.z * right.z);
        if (rightLen > 1e-6f) {
            right.x /= rightLen;
            right.y /= rightLen;
            right.z /= rightLen;
        }

        // adjusted up = right × forward
        Vector3 adjUp = {
            right.y * forward.z - right.z * forward.y,
            right.z * forward.x - right.x * forward.z,
            right.x * forward.y - right.y * forward.x
        };

        // Half-extents at near and far planes
        float halfTan = std::tan(fovRadians * 0.5f);
        float nearHW = nearDist * halfTan;
        float nearHH = nearHW / aspect;
        float farHW = farDist * halfTan;
        float farHH = farHW / aspect;

        // Compute 8 frustum corners
        Vector3 nearCenter = {
            origin.x + forward.x * nearDist,
            origin.y + forward.y * nearDist,
            origin.z + forward.z * nearDist
        };
        Vector3 farCenter = {
            origin.x + forward.x * farDist,
            origin.y + forward.y * farDist,
            origin.z + forward.z * farDist
        };

        Vector3 corners[8];
        // Near plane corners
        for (int i = 0; i < 4; ++i) {
            float sx = (i & 1) ? nearHW : -nearHW;
            float sy = (i & 2) ? nearHH : -nearHH;
            corners[i] = {
                nearCenter.x + right.x * sx + adjUp.x * sy,
                nearCenter.y + right.y * sx + adjUp.y * sy,
                nearCenter.z + right.z * sx + adjUp.z * sy
            };
        }
        // Far plane corners
        for (int i = 0; i < 4; ++i) {
            float sx = (i & 1) ? farHW : -farHW;
            float sy = (i & 2) ? farHH : -farHH;
            corners[4 + i] = {
                farCenter.x + right.x * sx + adjUp.x * sy,
                farCenter.y + right.y * sx + adjUp.y * sy,
                farCenter.z + right.z * sx + adjUp.z * sy
            };
        }

        // AABB = min/max across all 8 corners
        BBox box;
        box.min = corners[0];
        box.max = corners[0];
        for (int i = 1; i < 8; ++i) {
            if (corners[i].x < box.min.x) box.min.x = corners[i].x;
            if (corners[i].y < box.min.y) box.min.y = corners[i].y;
            if (corners[i].z < box.min.z) box.min.z = corners[i].z;
            if (corners[i].x > box.max.x) box.max.x = corners[i].x;
            if (corners[i].y > box.max.y) box.max.y = corners[i].y;
            if (corners[i].z > box.max.z) box.max.z = corners[i].z;
        }

        return box;
    }

    ObjectService *mObjSvc;
    PropertyService *mPropSvc;
    LinkService *mLinkSvc;
    RoomService *mRoomSvc;

    // Raycaster — injected by renderer, not set at construction time
    RaycastFn mRaycaster;

    // Spatial index — mutable for lazy init from const query methods
    mutable SpatialIndex mSpatialIndex{32.0f};
    mutable bool mSpatialPopulated = false;
};

} // namespace Darkness

#endif // __OBJSYSWORLDSTATE_H
