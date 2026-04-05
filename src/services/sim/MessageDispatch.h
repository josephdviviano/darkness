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

// MessageDispatch.h — Minimal script message routing system
//
// Implements the Dark Engine's object messaging pattern: objects send named
// messages to other objects, which respond via registered handlers. This is
// the foundation for TurnOn/TurnOff, FrobWorldEnd, DoorOpen/DoorClose, and
// the full script system (Phase 6).
//
// Architecture:
//   - Messages are {to, name, from, data, data2, data3, time, flags} tuples
//   - Data fields use OPDE Variant (7-type tagged union matching cMultiParm)
//   - Handlers register per-object or per-message-name (global)
//   - ControlDevice link traversal: when an object is activated, follow its
//     ControlDevice relations and send TurnOn/TurnOff to linked targets
//     (Thief 1/2 use "ControlDevice"; System Shock 2 uses "SwitchLink")
//   - Built-in handlers: doors respond to TurnOn/TurnOff/FrobWorldEnd
//
// All dispatch is synchronous on the main thread. No queue, no deferral.
// ScriptManager (Phase 6) adds queued/deferred delivery on top of this.

#pragma once

#include <cstdint>
#include <string>
#include <functional>
#include <unordered_map>
#include <vector>

#include "dyntype/Variant.h"
#include "worldquery/IWorldQuery.h"

namespace Darkness {

// ── Script message flags (matching Dark Engine kSMF_* constants) ──
enum ScriptMessageFlags : uint32_t {
    kSMF_None          = 0,
    kSMF_MsgSent       = 0x01,  // message has been sent (set by dispatch)
    kSMF_MsgBlock      = 0x02,  // block further dispatch after this handler
    kSMF_MsgSendToProxy = 0x04, // send to proxy object (networking)
    kSMF_MsgPostToOwner = 0x08, // post to owner (networking)
};

// ── Script message ──
// Matches the Dark Engine's sScrMsg structure. Data fields use OPDE Variant
// (equivalent to the original engine's cMultiParm). Three data slots allow
// messages to carry typed payloads (e.g., timer name + delay, damage amount
// + type, frob source + destination).
struct ScriptMessage {
    int32_t to;            // destination object ID
    std::string name;      // message name (e.g. "TurnOn", "FrobWorldEnd")
    int32_t from;          // source object ID (who sent this)
    Variant data;          // primary data (replaces old int32_t)
    Variant data2;         // secondary data
    Variant data3;         // tertiary data
    float time = 0.0f;    // sim time when message was created (seconds)
    uint32_t flags = 0;   // ScriptMessageFlags bitmask
};

// ── Message handler callback ──
// Returns true if the message was consumed (stop further dispatch).
using MessageHandler = std::function<bool(const ScriptMessage &msg)>;

// ============================================================================
// MessageDispatch — routes messages to handlers
// ============================================================================

class MessageDispatch {
public:
    MessageDispatch() = default;

    /// Initialize with world query for link traversal.
    void init(const IWorldQuery *worldQuery) {
        mWorldQuery = worldQuery;
    }

    // ── Handler registration ──

    /// Register a handler for a specific object + message name pair.
    /// Multiple handlers per object are supported (all are called in order).
    void registerHandler(int32_t objID, const std::string &msgName,
                          MessageHandler handler) {
        uint64_t key = makeKey(objID, msgName);
        mHandlers[key].push_back(std::move(handler));
    }

    /// Register a handler for ALL objects receiving a given message name.
    /// Global handlers run after per-object handlers.
    void registerGlobalHandler(const std::string &msgName,
                                MessageHandler handler) {
        mGlobalHandlers[msgName].push_back(std::move(handler));
    }

    // ── Message sending ──

    /// Send a message directly to an object.
    bool sendMessage(const ScriptMessage &msg) {
        bool handled = false;

        // Per-object handlers
        uint64_t key = makeKey(msg.to, msg.name);
        auto it = mHandlers.find(key);
        if (it != mHandlers.end()) {
            for (auto &handler : it->second) {
                if (handler(msg)) {
                    handled = true;
                    break;  // consumed
                }
            }
        }

        // Global handlers (run even if per-object handler existed but didn't consume)
        if (!handled) {
            auto git = mGlobalHandlers.find(msg.name);
            if (git != mGlobalHandlers.end()) {
                for (auto &handler : git->second) {
                    if (handler(msg)) {
                        handled = true;
                        break;
                    }
                }
            }
        }

        return handled;
    }

    /// Send a message and follow ControlDevice link relations.
    /// First sends the message to the target, then follows ControlDevice from
    /// the target to linked objects, sending TurnOn or TurnOff as appropriate.
    /// (Thief 1/2 use "ControlDevice"; System Shock 2 uses "SwitchLink".)
    bool sendMessageWithLinks(const ScriptMessage &msg) {
        bool handled = sendMessage(msg);

        // Follow ControlDevice: when an object receives an activation message,
        // propagate to linked objects. The link direction is src → dst.
        if (mWorldQuery &&
            (msg.name == "TurnOn" || msg.name == "TurnOff" ||
             msg.name == "FrobWorldEnd")) {
            propagateControlDeviceLinks(msg.to, msg.from, msg.name);
        }

        return handled;
    }

    /// Convenience: send TurnOn to an object and follow ControlDevice links.
    void turnOn(int32_t objID, int32_t fromID = 0) {
        sendMessageWithLinks({objID, "TurnOn", fromID, {}});
    }

    /// Convenience: send TurnOff to an object and follow ControlDevice links.
    void turnOff(int32_t objID, int32_t fromID = 0) {
        sendMessageWithLinks({objID, "TurnOff", fromID, {}});
    }

    /// Convenience: send FrobWorldEnd and follow ControlDevice links.
    void frobWorldEnd(int32_t objID, int32_t frobberID = 0) {
        sendMessageWithLinks({objID, "FrobWorldEnd", frobberID, {}});
    }

    /// Follow ControlDevice link relations from an object and send messages
    /// to targets. "TurnOn" and "FrobWorldEnd" send "TurnOn" to linked objects.
    /// "TurnOff" sends "TurnOff" to linked objects. One-hop only to prevent
    /// infinite loops — scripts handle multi-hop propagation themselves.
    void propagateControlDeviceLinks(int32_t srcObjID, int32_t fromID,
                                      const std::string &triggerMsg) {
        if (!mWorldQuery) return;

        // Determine what message to send to linked targets
        std::string linkMsg = (triggerMsg == "TurnOff") ? "TurnOff" : "TurnOn";

        // Query all ControlDevice relations from this object
        // (Thief 1/2 missions use "ControlDevice", not "SwitchLink")
        auto links = mWorldQuery->getLinks(srcObjID, "ControlDevice", 0);

        if (links.empty()) {
            std::fprintf(stderr, "[MsgDispatch] propagate %s from obj %d: no ControlDevice links\n",
                         triggerMsg.c_str(), srcObjID);
        }

        for (const auto &link : links) {
            int32_t targetID = static_cast<int32_t>(link.dst);
            if (targetID == 0 || targetID == srcObjID)
                continue;  // skip self-links and null

            std::fprintf(stderr, "[MsgDispatch] propagate %s: obj %d -> %s -> obj %d\n",
                         triggerMsg.c_str(), srcObjID, linkMsg.c_str(), targetID);

            // Send the message to the linked target (no further ControlDevice
            // propagation to avoid infinite loops)
            ScriptMessage targetMsg{targetID, linkMsg, fromID, {}};
            sendMessage(targetMsg);
        }
    }

    /// Hash key for (objID, msgName) pair.
    static uint64_t makeKey(int32_t objID, const std::string &name) {
        // Combine objID hash with string hash
        uint64_t h1 = std::hash<int32_t>{}(objID);
        uint64_t h2 = std::hash<std::string>{}(name);
        return h1 ^ (h2 << 32) ^ (h2 >> 32);
    }

    const IWorldQuery *mWorldQuery = nullptr;

    // Per-object handlers: key = hash(objID, msgName)
    std::unordered_map<uint64_t, std::vector<MessageHandler>> mHandlers;

    // Global handlers: key = msgName (applies to all objects)
    std::unordered_map<std::string, std::vector<MessageHandler>> mGlobalHandlers;
};

} // namespace Darkness
