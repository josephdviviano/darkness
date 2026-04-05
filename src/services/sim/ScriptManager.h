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

// ScriptManager.h — Central script lifecycle manager
//
// Reads P$Scripts properties from mission data, instantiates ScriptBase
// subclasses per-object via the ScriptRegistry factory, and routes messages
// to the correct script instances. Integrates as a SimListener so scripts
// receive per-frame timer ticks and queued message delivery.
//
// Architecture:
//   - ScriptManager is a SimListener (registered after physics, before render)
//   - sendMessage(): synchronous delivery (scripts process inline)
//   - postMessage(): queued delivery (delivered next pumpMessages())
//   - Timer system: priority queue, fires Timer messages on expiry
//   - Script data: per-(objID, className, fieldName) key-value store
//   - Fallthrough: if no script handles a message, falls through to
//     MessageDispatch global handlers (backward compat during migration)
//
// Inspired by the openDarkEngine (OPDE) service architecture (GPLv2).
// Script system design follows Dark Engine's cScriptMan pattern.

#pragma once

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <queue>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "dyntype/Variant.h"
#include "property/DarkPropertyDefs.h"
#include "property/PropertyService.h"
#include "property/TypedProperty.h"
#include "sim/MessageDispatch.h"
#include "sim/ScriptBase.h"
#include "sim/SimCommon.h"
#include "link/LinkService.h"

namespace Darkness {

// Forward declarations
class PropertyService;
class LinkService;
class ObjectService;
struct IScriptServices;

// ============================================================================
// ScriptDatumTag — key for per-object, per-script-class data store
// ============================================================================

struct ScriptDatumTag {
    int32_t objID;
    std::string className;
    std::string fieldName;

    bool operator==(const ScriptDatumTag &o) const {
        return objID == o.objID && className == o.className &&
               fieldName == o.fieldName;
    }
};

struct ScriptDatumTagHash {
    size_t operator()(const ScriptDatumTag &t) const {
        size_t h1 = std::hash<int32_t>{}(t.objID);
        size_t h2 = std::hash<std::string>{}(t.className);
        size_t h3 = std::hash<std::string>{}(t.fieldName);
        return h1 ^ (h2 << 16) ^ (h3 << 32);
    }
};

// ============================================================================
// TimerEntry — pending timer in the priority queue
// ============================================================================

struct TimerEntry {
    float fireTime;            // absolute sim time to fire
    int32_t objID;             // target object
    std::string scriptClass;   // which script on the object
    std::string timerName;     // timer name (delivered in msg.data)
    Variant data;              // extra data (delivered in msg.data2)
    timer_handle handle;       // unique ID for cancellation

    // Priority queue is max-heap; we want earliest time first
    bool operator>(const TimerEntry &o) const { return fireTime > o.fireTime; }
};

// ============================================================================
// ScriptManager — central script lifecycle manager
// ============================================================================

class ScriptManager : public SimListener {
public:
    ScriptManager() = default;
    ~ScriptManager() override {
        destroyAllScripts();
    }

    // ── Initialization ──

    /// Initialize with required services. Call before instantiateScripts().
    void init(PropertyService *propSvc, LinkService *linkSvc,
              ObjectService *objSvc, MessageDispatch *msgDispatch,
              IScriptServices *services) {
        mPropSvc = propSvc;
        mLinkSvc = linkSvc;
        mObjSvc = objSvc;
        mMsgDispatch = msgDispatch;
        mServices = services;
    }

    /// Read P$Scripts from all concrete objects and instantiate script classes.
    /// Uses OPDE's inheritor system to resolve scripts from archetypes.
    void instantiateScripts() {
        if (!mPropSvc) return;

        Property *scriptProp = mPropSvc->getProperty("Scripts");
        if (!scriptProp) {
            std::fprintf(stderr, "[ScriptManager] WARNING: P$Scripts property not registered\n");
            return;
        }

        // We need to scan ALL concrete objects (positive IDs) because most
        // objects inherit P$Scripts from archetypes rather than owning it
        // directly. getTypedProperty uses the inheritor system to resolve.
        // To find all concrete objects, scan the Position property which
        // every placed object has directly.
        auto allPositioned = getAllObjectsWithProperty(mPropSvc, "Position");

        std::unordered_map<int32_t, PropScripts> objectScripts;
        for (int objID : allPositioned) {
            if (objID <= 0) continue;  // skip archetypes
            PropScripts scripts;
            // getTypedProperty resolves inheritance — walks archetype chain
            if (getTypedProperty<PropScripts>(mPropSvc, "Scripts", objID, scripts)) {
                objectScripts[objID] = scripts;
            }
        }

        int totalInstances = 0;
        int totalObjects = 0;
        int unregisteredCount = 0;
        std::unordered_map<std::string, int> unregisteredNames;

        for (auto &[objID, scripts] : objectScripts) {
            // Only instantiate scripts for concrete objects (positive IDs)
            if (objID <= 0) continue;

            // Check dontInherit: if set, only use scripts directly owned
            if (scripts.dontInherit && !ownsProperty(mPropSvc, "Scripts", objID))
                continue;

            int instancesForObj = 0;
            const char *slots[4] = {scripts.script0, scripts.script1,
                                     scripts.script2, scripts.script3};

            for (int slot = 0; slot < 4; ++slot) {
                if (slots[slot][0] == '\0') continue;

                std::string name(slots[slot]);

                ScriptFactory factory = ScriptRegistry::getFactory(name);
                if (!factory) {
                    // Track unregistered scripts for diagnostics
                    unregisteredNames[name]++;
                    unregisteredCount++;
                    continue;
                }

                ScriptBase *script = factory(objID, name, mServices, this);
                if (script) {
                    mScripts[objID].push_back(
                        std::unique_ptr<ScriptBase>(script));
                    instancesForObj++;
                    totalInstances++;
                }
            }

            if (instancesForObj > 0)
                totalObjects++;
        }

        std::fprintf(stderr,
                     "[ScriptManager] Instantiated %d script instances on %d objects"
                     " (%d unregistered references)\n",
                     totalInstances, totalObjects, unregisteredCount);

        // Log the most common unregistered script names
        if (!unregisteredNames.empty()) {
            std::vector<std::pair<std::string, int>> sorted(
                unregisteredNames.begin(), unregisteredNames.end());
            std::sort(sorted.begin(), sorted.end(),
                      [](const auto &a, const auto &b) {
                          return b.second < a.second;
                      });
            int shown = 0;
            for (const auto &[name, count] : sorted) {
                std::fprintf(stderr,
                             "  [ScriptManager] Unregistered: \"%s\" (%d refs)\n",
                             name.c_str(), count);
                if (++shown >= 40) break;
            }
        }
    }

    // ── Message delivery ──

    /// Send a message synchronously — scripts process inline, then falls
    /// through to MessageDispatch global handlers if not consumed.
    bool sendMessage(ScriptMessage &msg) {
        msg.time = mSimTime;
        msg.flags |= kSMF_MsgSent;

        bool handled = deliverToScripts(msg);

        // Fall through to MessageDispatch for backward compat
        // (global handlers for doors/tweqs/platforms still active)
        if (!handled && mMsgDispatch) {
            handled = mMsgDispatch->sendMessage(msg);
        }

        return handled;
    }

    /// Send a message with ControlDevice link propagation.
    /// Scripts get first crack; then global handlers; then link traversal.
    bool sendMessageWithLinks(ScriptMessage &msg) {
        msg.time = mSimTime;
        msg.flags |= kSMF_MsgSent;

        bool handled = deliverToScripts(msg);

        // Fall through to MessageDispatch global handlers (without re-sending)
        if (mMsgDispatch) {
            if (!handled)
                handled = mMsgDispatch->sendMessage(msg);

            // ControlDevice link propagation — done here in ScriptManager
            // so that linked TurnOn/TurnOff messages are routed through
            // sendMessage() (scripts get first crack), not directly through
            // MessageDispatch which would bypass scripts entirely.
            if (msg.name == "TurnOn" || msg.name == "TurnOff" ||
                msg.name == "FrobWorldEnd") {
                propagateControlDeviceLinks(msg.to, msg.from, msg.name);
            }
        }

        return handled;
    }

    /// Post a message for deferred delivery (next pumpMessages() call).
    void postMessage(ScriptMessage msg) {
        msg.time = mSimTime;
        mMessageQueue.push(std::move(msg));
    }

    /// Drain the message queue. Called each simStep.
    void pumpMessages() {
        // Process queue in FIFO order. New messages posted during processing
        // are delivered in the same pump (matching original engine behavior).
        while (!mMessageQueue.empty()) {
            ScriptMessage msg = std::move(mMessageQueue.front());
            mMessageQueue.pop();
            sendMessage(msg);
        }
    }

    // ── Timer system ──

    /// Set a one-shot timer. Returns a handle for cancellation.
    timer_handle setTimer(int32_t objID, const std::string &scriptClass,
                          const std::string &timerName, float delaySec,
                          Variant data = {}) {
        timer_handle h = ++mNextTimerHandle;
        mTimers.push({mSimTime + delaySec, objID, scriptClass,
                      timerName, std::move(data), h});
        return h;
    }

    /// Cancel a pending timer by handle.
    void killTimer(timer_handle h) {
        mCancelledTimers.insert(h);
    }

    /// Fire all expired timers as Timer messages. Called each simStep.
    void pumpTimers(float simTime) {
        while (!mTimers.empty() && mTimers.top().fireTime <= simTime) {
            TimerEntry entry = mTimers.top();
            mTimers.pop();

            // Skip cancelled timers
            if (mCancelledTimers.count(entry.handle)) {
                mCancelledTimers.erase(entry.handle);
                continue;
            }

            // Deliver Timer message to the specific script
            ScriptMessage msg;
            msg.to = entry.objID;
            msg.name = "Timer";
            msg.from = entry.objID;
            msg.data = Variant(entry.timerName);
            msg.data2 = entry.data;
            msg.time = simTime;
            msg.flags = kSMF_MsgSent;

            deliverToScript(entry.objID, entry.scriptClass, msg);
        }
    }

    // ── Script data store ──

    void setScriptData(int32_t objID, const std::string &className,
                       const std::string &key, Variant value) {
        mScriptData[{objID, className, key}] = std::move(value);
    }

    Variant getScriptData(int32_t objID, const std::string &className,
                          const std::string &key) const {
        auto it = mScriptData.find({objID, className, key});
        return (it != mScriptData.end()) ? it->second : Variant();
    }

    bool isScriptDataSet(int32_t objID, const std::string &className,
                         const std::string &key) const {
        return mScriptData.count({objID, className, key}) > 0;
    }

    void clearScriptData(int32_t objID, const std::string &className,
                         const std::string &key) {
        mScriptData.erase({objID, className, key});
    }

    // ── SimListener interface ──

    void simStarted() override {
        SimListener::simStarted();

        // Send BeginScript to all scripts
        for (auto &[objID, scripts] : mScripts) {
            for (auto &script : scripts) {
                ScriptMessage msg;
                msg.to = objID;
                msg.name = "BeginScript";
                msg.from = 0;
                msg.time = 0.0f;
                script->receiveMessage(msg);
            }
        }

        // Send Sim message (sim has started)
        for (auto &[objID, scripts] : mScripts) {
            for (auto &script : scripts) {
                ScriptMessage msg;
                msg.to = objID;
                msg.name = "Sim";
                msg.from = 0;
                msg.data = Variant(1);  // 1 = sim starting
                msg.time = 0.0f;
                script->receiveMessage(msg);
            }
        }
    }

    void simEnded() override {
        // Send EndScript to all scripts
        for (auto &[objID, scripts] : mScripts) {
            for (auto &script : scripts) {
                ScriptMessage msg;
                msg.to = objID;
                msg.name = "EndScript";
                msg.from = 0;
                msg.time = mSimTime;
                script->receiveMessage(msg);
            }
        }

        SimListener::simEnded();
    }

    void simStep(float simTime, float delta) override {
        SimListener::simStep(simTime, delta);
        pumpTimers(simTime);
        pumpMessages();
    }

    // ── Query ──

    /// Check if any object has scripts attached.
    bool hasScripts(int32_t objID) const {
        return mScripts.count(objID) > 0;
    }

    /// Get the number of script instances total.
    size_t getScriptCount() const {
        size_t count = 0;
        for (const auto &[id, scripts] : mScripts)
            count += scripts.size();
        return count;
    }

    /// Get all objects that have scripts.
    std::vector<int32_t> getScriptedObjects() const {
        std::vector<int32_t> result;
        result.reserve(mScripts.size());
        for (const auto &[id, scripts] : mScripts)
            result.push_back(id);
        return result;
    }

    // ── ControlDevice link propagation ──
    // Follows ControlDevice relations from srcObjID and sends TurnOn/TurnOff
    // to linked targets through ScriptManager::sendMessage() (scripts first).

    void propagateControlDeviceLinks(int32_t srcObjID, int32_t fromID,
                                      const std::string &triggerMsg) {
        std::string linkMsg;
        if (triggerMsg == "TurnOn") {
            linkMsg = "TurnOn";
        } else if (triggerMsg == "TurnOff") {
            linkMsg = "TurnOff";
        } else if (triggerMsg == "FrobWorldEnd") {
            // FrobWorldEnd on a scriptless switch: toggle state.
            // Track per-object on/off state so repeated frobs alternate.
            bool &isOn = mFrobToggleState[srcObjID];
            isOn = !isOn;
            linkMsg = isOn ? "TurnOff" : "TurnOn";
        } else {
            linkMsg = "TurnOn";
        }
        broadcastOnAllLinks(srcObjID, linkMsg, "ControlDevice", {});
    }

    // ── Link traversal helpers (used by ScriptBase::broadcastOnAllLinks) ──

    void broadcastOnAllLinks(int32_t srcObjID, const std::string &msgName,
                             const std::string &linkType, Variant data) {
        if (!mLinkSvc) return;

        int flavor = mLinkSvc->nameToFlavor(linkType);
        if (flavor < 0) return;

        LinkQueryResultPtr links = mLinkSvc->getAllLinks(flavor, srcObjID, 0);
        if (!links) return;

        while (!links->end()) {
            const Link &link = links->next();
            int32_t dstID = link.dst();
            if (dstID == 0 || dstID == srcObjID) continue;

            ScriptMessage msg;
            msg.to = dstID;
            msg.name = msgName;
            msg.from = srcObjID;
            msg.data = data;
            sendMessage(msg);
        }
    }

private:
    /// Deliver a message to all scripts on the target object.
    /// Returns true if any script returned kSR_Handled or kSR_Consumed.
    bool deliverToScripts(ScriptMessage &msg) {
        auto it = mScripts.find(msg.to);
        if (it == mScripts.end()) return false;

        bool handled = false;
        for (auto &script : it->second) {
            eScriptResult result = script->receiveMessage(msg);
            if (result == kSR_Handled || result == kSR_Consumed) {
                handled = true;
            }
            if (result == kSR_Consumed) break;
        }
        return handled;
    }

    /// Deliver a Timer message to a specific script class on an object.
    void deliverToScript(int32_t objID, const std::string &className,
                         ScriptMessage &msg) {
        auto it = mScripts.find(objID);
        if (it == mScripts.end()) return;

        for (auto &script : it->second) {
            if (script->getClassName() == className) {
                script->receiveMessage(msg);
                return;
            }
        }
    }

    /// Destroy all script instances (sends EndScript first if sim is running).
    void destroyAllScripts() {
        mScripts.clear();
    }

    // ── Services ──
    PropertyService *mPropSvc = nullptr;
    LinkService *mLinkSvc = nullptr;
    ObjectService *mObjSvc = nullptr;
    MessageDispatch *mMsgDispatch = nullptr;
    IScriptServices *mServices = nullptr;

    // ── Script instances: objID → list of scripts ──
    std::unordered_map<int32_t,
                       std::vector<std::unique_ptr<ScriptBase>>> mScripts;

    // ── Message queue (for postMessage) ──
    std::queue<ScriptMessage> mMessageQueue;

    // ── Timer system ──
    std::priority_queue<TimerEntry, std::vector<TimerEntry>,
                        std::greater<TimerEntry>> mTimers;
    std::unordered_set<timer_handle> mCancelledTimers;
    timer_handle mNextTimerHandle = 0;

    // ── Script data store ──
    std::unordered_map<ScriptDatumTag, Variant, ScriptDatumTagHash> mScriptData;

    // ── Frob toggle state for scriptless switches ──
    // Tracks on/off state per object so FrobWorldEnd alternates TurnOn/TurnOff
    // on ControlDevice links. Objects with scripts handle this themselves.
    std::unordered_map<int32_t, bool> mFrobToggleState;
};

// ============================================================================
// ScriptBase method implementations (need ScriptManager definition)
// ============================================================================

inline timer_handle ScriptBase::setOneShotTimer(const std::string &name,
                                                 float delaySec, Variant data) {
    if (!mgr) return kInvalidTimer;
    return mgr->setTimer(self, mClassName, name, delaySec, std::move(data));
}

inline void ScriptBase::killTimer(timer_handle h) {
    if (mgr) mgr->killTimer(h);
}

inline void ScriptBase::setData(const std::string &key, Variant value) {
    if (mgr) mgr->setScriptData(self, mClassName, key, std::move(value));
}

inline Variant ScriptBase::getData(const std::string &key) const {
    if (!mgr) return {};
    return mgr->getScriptData(self, mClassName, key);
}

inline bool ScriptBase::isDataSet(const std::string &key) const {
    if (!mgr) return false;
    return mgr->isScriptDataSet(self, mClassName, key);
}

inline void ScriptBase::clearData(const std::string &key) {
    if (mgr) mgr->clearScriptData(self, mClassName, key);
}

inline void ScriptBase::broadcastOnAllLinks(const std::string &msg,
                                             const std::string &linkType,
                                             Variant data) {
    if (mgr) mgr->broadcastOnAllLinks(self, msg, linkType, std::move(data));
}

} // namespace Darkness
