/******************************************************************************
 *
 *    This file is part of the Darkness engine
 *
 *    ObjSysWorldState — concrete IWorldQuery implementation that wraps the
 *    OPDE service stack (ObjectService, PropertyService, LinkService,
 *    RoomService). This is the primary world-state facade for downstream
 *    subsystems.
 *
 *    Methods requiring future systems (SpatialIndex, raycast, light level)
 *    return documented defaults and will be filled in by later tasks.
 *
 *    See .claude/IWorldQuery_PLAN.md for design rationale.
 *
 *****************************************************************************/

#ifndef __OBJSYSWORLDSTATE_H
#define __OBJSYSWORLDSTATE_H

#include "IWorldQuery.h"
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
    // Spatial queries (stubs until Task 15 SpatialIndex)
    // ========================================================================

    std::vector<EntityID> queryRadius(const Vector3 & /*center*/,
                                      float /*radius*/) const override {
        return {}; // Stub — will delegate to SpatialIndex in Task 15
    }

    std::vector<EntityID>
    queryFrustum(const Vector3 & /*origin*/, const Vector3 & /*forward*/,
                 float /*fovRadians*/, float /*aspect*/, float /*nearDist*/,
                 float /*farDist*/) const override {
        return {}; // Stub — will delegate to SpatialIndex in Task 15
    }

    // ========================================================================
    // Environment queries (stubs until later phases)
    // ========================================================================

    float getLightLevel(const Vector3 & /*pos*/) const override {
        return 1.0f; // Stub — will query lightmap/dynamic light in Phase 6
    }

    bool raycast(const Vector3 & /*from*/, const Vector3 & /*to*/,
                 RayHit & /*hit*/) const override {
        return false; // Stub — will delegate to ray-cell traversal in Task 16
    }

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

    ObjectService *mObjSvc;
    PropertyService *mPropSvc;
    LinkService *mLinkSvc;
    RoomService *mRoomSvc;
};

} // namespace Darkness

#endif // __OBJSYSWORLDSTATE_H
