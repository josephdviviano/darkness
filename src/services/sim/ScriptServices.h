/******************************************************************************
 *
 *    This file is part of the Darkness engine
 *    Copyright (C) 2024-2026 darkness contributors
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

// ScriptServices.h — Thin wrapper services for scripts (Task 67)
//
// Each service wraps existing OPDE services with a script-friendly API.
// No functionality is rebuilt — these are convenience facades that cache
// flavor IDs and provide simplified signatures matching the original
// Dark Engine's script service interfaces (ILinkSrv, IObjectSrv, etc.).
//
// Inspired by the openDarkEngine (OPDE) service architecture (GPLv2).
// Service API design follows Dark Engine's scrptsrv.h pattern.

#pragma once

#include <cstdint>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

#include "DarknessMath.h"
#include "dyntype/Variant.h"
#include "link/LinkCommon.h"
#include "link/LinkService.h"
#include "object/ObjectService.h"
#include "property/PropertyService.h"
#include "property/TypedProperty.h"
#include "property/DarkPropertyDefs.h"
#include "sim/DoorSystem.h"
#include "sim/TweqSystem.h"
#include "sim/MovingTerrainSystem.h"
#include "sim/MessageDispatch.h"
#include "worldquery/IWorldQuery.h"

// Forward declare AudioService (avoid heavy include)
namespace Darkness { class AudioService; }

namespace Darkness {

// ============================================================================
// LinkScriptService (Task 67b)
// ============================================================================
// Thin wrapper around OPDE's Relation CRUD. Caches flavor IDs for performance.

struct LinkScriptService {
    LinkService *linkSvc = nullptr;

    /// Get or cache a relation flavor ID by name.
    int getFlavor(const std::string &name) {
        auto it = mFlavorCache.find(name);
        if (it != mFlavorCache.end()) return it->second;
        int f = linkSvc ? linkSvc->nameToFlavor(name) : -1;
        mFlavorCache[name] = f;
        return f;
    }

    /// Create a link between two objects.
    link_id_t create(const std::string &kind, int32_t from, int32_t to) {
        if (!linkSvc) return 0;
        auto rel = linkSvc->getRelation(kind);
        if (!rel) return 0;
        return rel->create(from, to);
    }

    /// Destroy a link by ID.
    void destroy(link_id_t id) {
        if (!linkSvc) return;
        const Link *link = linkSvc->getLink(id);
        if (!link) return;
        auto rel = linkSvc->getRelation(link->flavor());
        if (rel) rel->remove(id);
    }

    /// Destroy all links of a given kind between src and dst.
    void destroyMany(const std::string &kind, int32_t from, int32_t to) {
        if (!linkSvc) return;
        int flavor = getFlavor(kind);
        if (flavor < 0) return;
        auto results = linkSvc->getAllLinks(flavor, from, to);
        if (!results) return;
        std::vector<link_id_t> toRemove;
        while (!results->end()) {
            toRemove.push_back(results->next().id());
        }
        auto rel = linkSvc->getRelation(flavor);
        if (!rel) return;
        for (auto id : toRemove) rel->remove(id);
    }

    /// Check if any link of a given kind exists between src and dst.
    bool anyExist(const std::string &kind, int32_t from, int32_t to) {
        if (!linkSvc) return false;
        int flavor = getFlavor(kind);
        if (flavor < 0) return false;
        auto link = linkSvc->getOneLink(flavor, from, to);
        return link != nullptr;
    }

    /// Get all links of a given kind from src (to=0 for any dest).
    struct LinkInfo {
        link_id_t id;
        int32_t src;
        int32_t dst;
    };

    std::vector<LinkInfo> getAll(const std::string &kind, int32_t from,
                                  int32_t to = 0) {
        std::vector<LinkInfo> result;
        if (!linkSvc) return result;
        int flavor = getFlavor(kind);
        if (flavor < 0) return result;
        auto links = linkSvc->getAllLinks(flavor, from, to);
        if (!links) return result;
        while (!links->end()) {
            const Link &l = links->next();
            result.push_back({l.id(), l.src(), l.dst()});
        }
        return result;
    }

    /// Get a single link of a given kind between src and dst.
    LinkInfo getOne(const std::string &kind, int32_t from, int32_t to = 0) {
        if (!linkSvc) return {0, 0, 0};
        int flavor = getFlavor(kind);
        if (flavor < 0) return {0, 0, 0};
        auto link = linkSvc->getOneLink(flavor, from, to);
        if (!link) return {0, 0, 0};
        return {link->id(), link->src(), link->dst()};
    }

    /// Get a link data field (e.g., ScriptParams link data).
    Variant getLinkData(link_id_t id, const std::string &field) {
        if (!linkSvc) return {};
        const Link *link = linkSvc->getLink(id);
        if (!link) return {};
        auto rel = linkSvc->getRelation(link->flavor());
        if (!rel) return {};
        return rel->getLinkField(id, field);
    }

private:
    std::unordered_map<std::string, int> mFlavorCache;
};

// ============================================================================
// ObjectScriptService (Task 67c)
// ============================================================================
// Thin wrapper around OPDE's ObjectService.

struct ObjectScriptService {
    ObjectService *objSvc = nullptr;
    const IWorldQuery *worldQuery = nullptr;

    int32_t create(int32_t archetype) {
        return objSvc ? objSvc->create(archetype) : 0;
    }

    void destroy(int32_t objID) {
        // Full destroy with broadcast — scripts may need cleanup
        // For now, mark as destroyed. Full destroy requires ObjectService support.
        if (objSvc) {
            // ObjectService doesn't have a destroy() yet — stub for Phase 6
            std::fprintf(stderr, "[ObjectScriptService] destroy(%d) stubbed\n", objID);
        }
    }

    bool exists(int32_t objID) const {
        return objSvc ? objSvc->exists(objID) : false;
    }

    std::string getName(int32_t objID) const {
        return objSvc ? objSvc->getName(objID) : "";
    }

    Vector3 position(int32_t objID) const {
        if (worldQuery) return worldQuery->getPosition(objID);
        return objSvc ? objSvc->position(objID) : Vector3(0);
    }

    Vector3 facing(int32_t objID) const {
        // Return Euler angles from ObjectState or P$Position
        // Stub: return zero facing for now
        return Vector3(0);
    }

    void teleport(int32_t objID, const Vector3 &pos, const Vector3 &facing) {
        if (!objSvc) return;
        // Convert facing (Euler) to Quaternion
        Quaternion ori = glm::quat(glm::vec3(facing.x, facing.y, facing.z));
        objSvc->teleport(objID, pos, ori, false);
    }

    bool addMetaProperty(int32_t objID, int32_t mpID) {
        if (!objSvc) return false;
        std::string name = objSvc->getName(mpID);
        return objSvc->addMetaProperty(objID, name) > 0;
    }

    bool removeMetaProperty(int32_t objID, int32_t mpID) {
        if (!objSvc) return false;
        std::string name = objSvc->getName(mpID);
        return objSvc->removeMetaProperty(objID, name) > 0;
    }

    bool inheritsFrom(int32_t objID, int32_t archetype) const {
        return objSvc ? objSvc->hasMetaProperty(objID, objSvc->getName(archetype))
                      : false;
    }
};

// ============================================================================
// PropertyScriptService (Task 67d)
// ============================================================================
// Thin wrapper around OPDE's PropertyService.

struct PropertyScriptService {
    PropertyService *propSvc = nullptr;

    Variant get(int32_t objID, const std::string &propName,
                const std::string &fieldName = "") const {
        if (!propSvc) return {};
        Variant result;
        if (fieldName.empty()) {
            // Get the first/default field
            propSvc->get(objID, propName, "", result);
        } else {
            propSvc->get(objID, propName, fieldName, result);
        }
        return result;
    }

    bool set(int32_t objID, const std::string &propName,
             const std::string &fieldName, const Variant &value) {
        return propSvc ? propSvc->set(objID, propName, fieldName, value) : false;
    }

    bool possessed(int32_t objID, const std::string &propName) const {
        return propSvc ? propSvc->has(objID, propName) : false;
    }

    bool owned(int32_t objID, const std::string &propName) const {
        return propSvc ? propSvc->owns(objID, propName) : false;
    }

    /// Get typed property data (raw binary access for complex structs).
    template <typename T>
    bool getTyped(int32_t objID, const std::string &propName, T &out) const {
        return propSvc ? getTypedProperty<T>(propSvc, propName, objID, out) : false;
    }
};

// ============================================================================
// SoundScriptService (Task 67e)
// ============================================================================
// Thin wrapper around AudioService.

struct SoundScriptService {
    AudioService *audioSvc = nullptr;

    /// Play an environmental schema (tag-based lookup). Returns handle.
    int32_t playEnvSchema(int32_t objID, const std::string &tags,
                          int32_t srcID = 0) {
        // AudioService's playEnvSchema takes a vector of SchemaTagValue.
        // Scripts pass simplified tag strings — we parse into tags here.
        // Stub: log and return invalid handle until AudioService is wired.
        std::fprintf(stderr, "[SoundScriptService] playEnvSchema(obj=%d, tags=\"%s\")\n",
                     objID, tags.c_str());
        return -1;
    }

    /// Play a named schema directly. Returns handle.
    int32_t playSchema(int32_t objID, const std::string &schemaName) {
        std::fprintf(stderr, "[SoundScriptService] playSchema(obj=%d, schema=\"%s\")\n",
                     objID, schemaName.c_str());
        return -1;
    }

    /// Halt a specific schema on an object.
    void haltSchema(int32_t objID, const std::string &schemaName) {
        std::fprintf(stderr, "[SoundScriptService] haltSchema(obj=%d, schema=\"%s\")\n",
                     objID, schemaName.c_str());
    }

    /// Halt all speech on an object.
    void haltSpeech(int32_t objID) {
        std::fprintf(stderr, "[SoundScriptService] haltSpeech(obj=%d)\n", objID);
    }
};

// ============================================================================
// LightScriptService (Task 67f)
// ============================================================================
// Wraps LightingSystem for AnimLight control.

struct LightScriptService {
    // LightingSystem is header-only with free functions; we need a reference
    // to the animLights map and the dirty-region update callback.
    // These are wired by the main loop after LightingSystem is initialized.

    /// Change an animated light's mode at runtime.
    void setMode(int32_t objID, int mode) {
        std::fprintf(stderr, "[LightScriptService] setMode(obj=%d, mode=%d)\n",
                     objID, mode);
        // Will be wired to LightingSystem's animLights map
    }

    /// Activate a light (start animation).
    void activate(int32_t objID) {
        std::fprintf(stderr, "[LightScriptService] activate(obj=%d)\n", objID);
    }

    /// Deactivate a light (stop animation, set to min brightness).
    void deactivate(int32_t objID) {
        std::fprintf(stderr, "[LightScriptService] deactivate(obj=%d)\n", objID);
    }

    /// Set direct brightness value (0.0-1.0).
    void setBrightness(int32_t objID, float brightness) {
        std::fprintf(stderr, "[LightScriptService] setBrightness(obj=%d, %.2f)\n",
                     objID, brightness);
    }
};

// ============================================================================
// DamageScriptService (Task 67g)
// ============================================================================

struct DamageScriptService {
    MessageDispatch *msgDispatch = nullptr;

    /// Destroy an object with death effects.
    void slay(int32_t objID, int32_t culprit = 0) {
        // Send Slain message to the object
        if (msgDispatch) {
            ScriptMessage msg;
            msg.to = objID;
            msg.name = "Slain";
            msg.from = culprit;
            msgDispatch->sendMessage(msg);
        }
    }

    /// Apply damage to an object.
    void damage(int32_t objID, int32_t culprit, int amount, int type = 0) {
        if (msgDispatch) {
            ScriptMessage msg;
            msg.to = objID;
            msg.name = "Damage";
            msg.from = culprit;
            msg.data = Variant(amount);
            msg.data2 = Variant(type);
            msgDispatch->sendMessage(msg);
        }
    }
};

// ============================================================================
// DataScriptService (Task 67h)
// ============================================================================
// Random number generation for scripts.

struct DataScriptService {
    /// Random float in [0, 1).
    float randFloat01() {
        return mDist(mRng);
    }

    /// Random integer in [min, max] inclusive.
    int randInt(int min, int max) {
        if (min >= max) return min;
        std::uniform_int_distribution<int> dist(min, max);
        return dist(mRng);
    }

private:
    std::mt19937 mRng{std::random_device{}()};
    std::uniform_real_distribution<float> mDist{0.0f, 1.0f};
};

// ============================================================================
// LockedScriptService (Task 67i)
// ============================================================================
// Wraps P$Locked property + NowLocked/NowUnlocked messaging.

struct LockedScriptService {
    PropertyService *propSvc = nullptr;
    MessageDispatch *msgDispatch = nullptr;

    bool isLocked(int32_t objID) const {
        if (!propSvc) return false;
        PropLocked locked{};
        if (getTypedProperty<PropLocked>(propSvc, "Locked", objID, locked))
            return locked.isLocked != 0;
        return false;
    }

    void setLocked(int32_t objID, bool locked) {
        if (!propSvc) return;
        // Set the property value
        propSvc->set(objID, "Locked", "", Variant(locked ? 1u : 0u));

        // Send NowLocked/NowUnlocked message
        if (msgDispatch) {
            ScriptMessage msg;
            msg.to = objID;
            msg.name = locked ? "NowLocked" : "NowUnlocked";
            msg.from = objID;
            msgDispatch->sendMessage(msg);
        }
    }
};

// ============================================================================
// PhysicsScriptService (Task 67j)
// ============================================================================
// Wraps future ODE integration. Initially stub.

struct PhysicsScriptService {
    /// Subscribe to physics collision messages for an object.
    void subscribeMsg(int32_t objID, uint32_t msgTypes) {
        mSubscriptions[objID] |= msgTypes;
    }

    /// Unsubscribe from physics messages.
    void unsubscribeMsg(int32_t objID, uint32_t msgTypes) {
        mSubscriptions[objID] &= ~msgTypes;
    }

    /// Check if an object is subscribed to a message type.
    bool isSubscribed(int32_t objID, uint32_t msgType) const {
        auto it = mSubscriptions.find(objID);
        return it != mSubscriptions.end() && (it->second & msgType) != 0;
    }

    // Physics message type flags (matching original engine)
    static constexpr uint32_t kCollisionMsg = 0x01;
    static constexpr uint32_t kEnterExitMsg = 0x02;
    static constexpr uint32_t kContactMsg   = 0x04;

private:
    std::unordered_map<int32_t, uint32_t> mSubscriptions;
};

// ============================================================================
// DoorScriptService (Task 67k)
// ============================================================================
// Wraps DoorSystem.

struct DoorScriptService {
    DoorSystem *doorSys = nullptr;

    DoorStatus getDoorState(int32_t objID) const {
        return doorSys ? doorSys->getStatus(objID) : kDoorClosed;
    }

    bool openDoor(int32_t objID) {
        return doorSys ? doorSys->activate(objID, kDoorDoOpen) : false;
    }

    bool closeDoor(int32_t objID) {
        return doorSys ? doorSys->activate(objID, kDoorDoClose) : false;
    }

    bool toggleDoor(int32_t objID) {
        return doorSys ? doorSys->activate(objID, kDoorToggle) : false;
    }

    bool isDoor(int32_t objID) const {
        return doorSys ? doorSys->isDoor(objID) : false;
    }

    float getOpenFraction(int32_t objID) const {
        return doorSys ? doorSys->getOpenFraction(objID) : 0.0f;
    }
};

// ============================================================================
// ContainerScriptService (Task 67l)
// ============================================================================

struct ContainerScriptService {
    LinkService *linkSvc = nullptr;

    /// Add object to a container (create Contains link).
    void add(int32_t objID, int32_t containerID) {
        if (!linkSvc) return;
        auto rel = linkSvc->getRelation("Contains");
        if (rel) rel->create(containerID, objID);
    }

    /// Remove object from a container.
    void remove(int32_t objID, int32_t containerID) {
        if (!linkSvc) return;
        int flavor = linkSvc->nameToFlavor("Contains");
        if (flavor < 0) return;
        auto link = linkSvc->getOneLink(flavor, containerID, objID);
        if (!link) return;
        auto rel = linkSvc->getRelation(flavor);
        if (rel) rel->remove(link->id());
    }

    /// Check if an object is contained in anything.
    bool isHeld(int32_t objID) const {
        if (!linkSvc) return false;
        int flavor = linkSvc->nameToFlavor("Contains");
        if (flavor < 0) return false;
        // Check for any incoming Contains link (container→objID)
        auto links = linkSvc->getAllLinks(flavor, 0, objID);
        return links && !links->end();
    }
};

// ============================================================================
// DarkGameScriptService (Task 67n)
// ============================================================================

struct DarkGameScriptService {
    /// Credit player with discovering a secret/loot object.
    void foundObject(int32_t objID) {
        std::fprintf(stderr, "[DarkGameScriptService] foundObject(%d)\n", objID);
        // Will update quest variables for loot totals
    }

    /// Trigger mission completion.
    void endMission() {
        std::fprintf(stderr, "[DarkGameScriptService] endMission()\n");
    }

    /// Check if a config key is defined.
    bool configIsDefined(const std::string &key) const {
        return false;  // Stub
    }
};

// ============================================================================
// DarkUIScriptService (Task 67p)
// ============================================================================

struct DarkUIScriptService {
    /// Display book/scroll text.
    void readBook(int32_t objID, const std::string &artName) {
        std::fprintf(stderr, "[DarkUIScriptService] readBook(obj=%d, art=\"%s\")\n",
                     objID, artName.c_str());
    }

    /// Display HUD text message.
    void textMessage(const std::string &text, uint32_t color = 0xFFFFFF,
                     float time = 5.0f) {
        std::fprintf(stderr, "[DarkUI] %s\n", text.c_str());
    }
};

} // namespace Darkness
