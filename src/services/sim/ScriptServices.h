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

#include <sstream>

#include "DarknessMath.h"
#include "LightingSystem.h"
#include "audio/AudioLog.h"
#include "audio/AudioService.h"
#include "audio/SchemaTypes.h"
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
        if (!linkSvc) {
            std::fprintf(stderr, "[FALLBACK] LinkScriptService::getFlavor: linkSvc is null, returning -1 for '%s'\n", name.c_str());
            mFlavorCache[name] = -1;
            return -1;
        }
        int f = linkSvc->nameToFlavor(name);
        if (f < 0) {
            static int warnCount = 0;
            if (warnCount++ < 20)
                std::fprintf(stderr, "[DEFAULT] LinkScriptService::getFlavor: relation '%s' not found, returning -1\n", name.c_str());
        }
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
        if (objSvc) return objSvc->position(objID);
        static int warnCount = 0;
        if (warnCount++ < 5)
            std::fprintf(stderr, "[FALLBACK] ObjectScriptService::position: no worldQuery or objSvc, returning origin for obj %d\n", objID);
        return Vector3(0);
    }

    Vector3 facing(int32_t objID) const {
        // Return Euler angles from ObjectState or P$Position
        // Stub: return zero facing for now
        static int warnCount = 0;
        if (warnCount++ < 5)
            std::fprintf(stderr, "[DEFAULT] ObjectScriptService::facing: STUB returning zero facing for obj %d\n", objID);
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
    PropertyService *propSvc = nullptr;

    // Object position lookup for 3D spatialization.
    const std::unordered_map<int32_t, ObjectPlacement> *placements = nullptr;

    /// Play an environmental schema (tag-based lookup). Returns handle.
    /// Tags are space-separated "Key Value" pairs, e.g. "Event Activate"
    /// or "Event StateChange, DirectionState Forward" (comma-separated groups).
    /// ClassTags from the object (e.g. "SwitchType BPush", "Material Metal")
    /// are automatically appended for schema matching.
    int32_t playEnvSchema(int32_t objID, const std::string &tags,
                          int32_t srcID = 0) {
        if (!audioSvc) {
            static int warnCount = 0;
            if (warnCount++ < 5)
                std::fprintf(stderr, "[FALLBACK] SoundScriptService::playEnvSchema: audioSvc is null, returning -1 for obj %d\n", objID);
            return -1;
        }

        // Parse tag string into SchemaTagValue vector
        std::vector<SchemaTagValue> tagVec;
        std::istringstream iss(tags);
        std::string key, value;
        while (iss >> key) {
            if (!key.empty() && key.back() == ',') key.pop_back();
            if (key.empty()) continue;
            if (!(iss >> value)) break;
            if (!value.empty() && value.back() == ',') value.pop_back();
            SchemaTagValue tv;
            tv.tagName = key;
            tv.enumValues.push_back(value);
            tagVec.push_back(std::move(tv));
        }

        // Append ClassTags from the object (e.g. "SwitchType BPush").
        // These are needed for schema matching — "Event Activate" alone
        // is too generic to match specific switch/button sounds.
        if (propSvc) {
            struct { uint32_t val; char text[252]; } classTags = {};
            int32_t lookupID = (srcID > 0) ? srcID : objID;
            for (const char *pname : {"ClassTags", "Class Tags", "Class Tag"}) {
                if (getTypedProperty<decltype(classTags)>(propSvc, pname, lookupID, classTags))
                    break;
            }
            std::string tagStr(classTags.text,
                strnlen(classTags.text, sizeof(classTags.text)));
            if (!tagStr.empty()) {
                std::istringstream ctIss(tagStr);
                std::string ctKey, ctVal;
                while (ctIss >> ctKey >> ctVal) {
                    SchemaTagValue tv;
                    tv.tagName = ctKey;
                    tv.enumValues.push_back(ctVal);
                    tagVec.push_back(std::move(tv));
                }
            }
        }

        // Look up object position for 3D spatialization
        Vector3 pos(0.0f);
        if (placements) {
            auto it = placements->find(objID);
            if (it != placements->end())
                pos = Vector3(it->second.x, it->second.y, it->second.z);
        }

        AUDIO_LOG("[SoundScriptService] playEnvSchema(obj=%d, tags=\"%s\", %zu total tags)\n",
                     objID, tags.c_str(), tagVec.size());
        return static_cast<int32_t>(audioSvc->playEnvSchema(tagVec, pos));
    }

    /// Play a named schema directly. Returns handle.
    int32_t playSchema(int32_t objID, const std::string &schemaName) {
        AUDIO_LOG("[SoundScriptService] playSchema(obj=%d, schema=\"%s\")\n",
                     objID, schemaName.c_str());
        return -1;
    }

    /// Halt a specific schema on an object.
    void haltSchema(int32_t objID, const std::string &schemaName) {
        AUDIO_LOG("[SoundScriptService] haltSchema(obj=%d, schema=\"%s\")\n",
                     objID, schemaName.c_str());
    }

    /// Halt all speech on an object.
    void haltSpeech(int32_t objID) {
        AUDIO_LOG("[SoundScriptService] haltSpeech(obj=%d)\n", objID);
    }
};

// ============================================================================
// LightScriptService (Task 67f)
// ============================================================================
// Wraps LightingSystem for AnimLight control.

struct LightScriptService {
    // Pointer to the mission's lightSources map (keyed by lightNum).
    // Set by the main loop after LightingSystem is initialized.
    std::unordered_map<int16_t, LightSource> *lightSources = nullptr;

    // Set true when any light is changed by a script. The render loop
    // checks and clears this to force a lightmap atlas re-blend.
    bool dirty = false;

    /// Find a LightSource by Dark Engine object ID.
    /// Returns nullptr if not found.
    LightSource *findByObjID(int32_t objID) {
        if (!lightSources) {
            static int warnCount = 0;
            if (warnCount++ < 3)
                std::fprintf(stderr, "[FALLBACK] LightScriptService::findByObjID: lightSources map not wired, returning nullptr for obj %d\n", objID);
            return nullptr;
        }
        for (auto &[lightNum, ls] : *lightSources) {
            if (ls.objectId == objID) return &ls;
        }
        return nullptr;
    }

    /// Change an animated light's mode at runtime.
    void setMode(int32_t objID, int mode) {
        LightSource *ls = findByObjID(objID);
        if (!ls) {
            std::fprintf(stderr, "[FALLBACK] LightScriptService::setMode(obj=%d, mode=%d) — light not found\n", objID, mode);
            return;
        }
        ls->mode = static_cast<uint16_t>(mode);
        ls->inactive = false;
        // Reset countdown so the new mode takes effect immediately
        ls->countdown = 0.0f;
        dirty = true;
        std::fprintf(stderr, "[LightScriptService] setMode(obj=%d, mode=%d)\n",
                     objID, mode);
    }

    /// Activate a light — set to max brightness mode and mark active.
    void activate(int32_t objID) {
        LightSource *ls = findByObjID(objID);
        if (!ls) {
            std::fprintf(stderr, "[LightScriptService] activate(obj=%d) — not found\n", objID);
            return;
        }
        float prevBright = ls->brightness;
        ls->inactive = false;
        ls->mode = ANIM_MAX_BRIGHT;
        ls->brightness = ls->maxBright;
        // Only mark dirty if brightness actually changed (avoid spurious
        // re-blend on startup when lights are already at max brightness)
        if (ls->brightness != prevBright) dirty = true;
        std::fprintf(stderr, "[LightScriptService] activate(obj=%d) lightNum=%d brightness=%.2f\n",
                     objID, ls->lightNum, ls->brightness);
    }

    /// Deactivate a light — set to zero brightness mode.
    void deactivate(int32_t objID) {
        LightSource *ls = findByObjID(objID);
        if (!ls) {
            std::fprintf(stderr, "[LightScriptService] deactivate(obj=%d) — not found\n", objID);
            return;
        }
        float prevBright = ls->brightness;
        ls->mode = ANIM_ZERO;
        ls->brightness = 0.0f;
        ls->inactive = false;  // must remain active so the zero brightness is applied
        if (ls->brightness != prevBright) dirty = true;
        std::fprintf(stderr, "[LightScriptService] deactivate(obj=%d) lightNum=%d\n",
                     objID, ls->lightNum);
    }

    /// Set direct brightness value (0.0-1.0).
    void setBrightness(int32_t objID, float brightness) {
        LightSource *ls = findByObjID(objID);
        if (!ls) {
            std::fprintf(stderr, "[FALLBACK] LightScriptService::setBrightness(obj=%d, %.2f) — light not found\n", objID, brightness);
            return;
        }
        ls->brightness = brightness;
        ls->inactive = false;
        dirty = true;
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
        if (!propSvc) {
            static int w = 0; if (w++ < 3)
                std::fprintf(stderr, "[FALLBACK] LockedScriptService::isLocked: propSvc null, returning false for obj %d\n", objID);
            return false;
        }
        PropLocked locked{};
        if (getTypedProperty<PropLocked>(propSvc, "Locked", objID, locked))
            return locked.isLocked != 0;
        return false;
    }

    void setLocked(int32_t objID, bool locked) {
        if (!propSvc) {
            static int w = 0; if (w++ < 3)
                std::fprintf(stderr, "[FALLBACK] LockedScriptService::setLocked: propSvc null for obj %d\n", objID);
            return;
        }
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
        if (!doorSys) {
            static int w = 0; if (w++ < 3)
                std::fprintf(stderr, "[FALLBACK] DoorScriptService::getDoorState: doorSys null, returning kDoorClosed for obj %d\n", objID);
            return kDoorClosed;
        }
        return doorSys->getStatus(objID);
    }

    bool openDoor(int32_t objID) {
        if (!doorSys) {
            static int w = 0; if (w++ < 3)
                std::fprintf(stderr, "[FALLBACK] DoorScriptService::openDoor: doorSys null for obj %d\n", objID);
            return false;
        }
        return doorSys->activate(objID, kDoorDoOpen);
    }

    bool closeDoor(int32_t objID) {
        if (!doorSys) {
            static int w = 0; if (w++ < 3)
                std::fprintf(stderr, "[FALLBACK] DoorScriptService::closeDoor: doorSys null for obj %d\n", objID);
            return false;
        }
        return doorSys->activate(objID, kDoorDoClose);
    }

    bool toggleDoor(int32_t objID) {
        if (!doorSys) {
            static int w = 0; if (w++ < 3)
                std::fprintf(stderr, "[FALLBACK] DoorScriptService::toggleDoor: doorSys null for obj %d\n", objID);
            return false;
        }
        return doorSys->activate(objID, kDoorToggle);
    }

    bool isDoor(int32_t objID) const {
        return doorSys ? doorSys->isDoor(objID) : false;
    }

    float getOpenFraction(int32_t objID) const {
        if (!doorSys) {
            static int w = 0; if (w++ < 3)
                std::fprintf(stderr, "[FALLBACK] DoorScriptService::getOpenFraction: doorSys null, returning 0.0 for obj %d\n", objID);
            return 0.0f;
        }
        return doorSys->getOpenFraction(objID);
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
