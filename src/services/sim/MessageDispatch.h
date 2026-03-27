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
// eventually the full DarkScript trap/trigger system (Phase 6).
//
// Architecture:
//   - Messages are {to, name, from, data} tuples
//   - Handlers register per-object or per-message-name (global)
//   - SwitchLink traversal: when an object is activated, follow its
//     SwitchLink relations and send TurnOn/TurnOff to linked targets
//   - Built-in handlers: doors respond to TurnOn/TurnOff/FrobWorldEnd
//
// All dispatch is synchronous on the main thread. No queue, no deferral.
// This is intentionally simple — the full script VM comes in Phase 6.

#pragma once

#include <cstdint>
#include <string>
#include <functional>
#include <unordered_map>
#include <vector>

#include "worldquery/IWorldQuery.h"

namespace Darkness {

// ── Script message ──
struct ScriptMessage {
    int32_t to;            // destination object ID
    std::string name;      // message name (e.g. "TurnOn", "FrobWorldEnd")
    int32_t from;          // source object ID (who sent this)
    int32_t data;          // generic integer data (0 if unused)
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

    /// Send a message and follow SwitchLink relations.
    /// First sends the message to the target, then follows SwitchLink from
    /// the target to linked objects, sending TurnOn or TurnOff as appropriate.
    bool sendMessageWithLinks(const ScriptMessage &msg) {
        bool handled = sendMessage(msg);

        // Follow SwitchLink: when an object receives an activation message,
        // propagate to linked objects. The link direction is src → dst.
        if (mWorldQuery &&
            (msg.name == "TurnOn" || msg.name == "TurnOff" ||
             msg.name == "FrobWorldEnd")) {
            propagateSwitchLinks(msg.to, msg.from, msg.name);
        }

        return handled;
    }

    /// Convenience: send TurnOn to an object and follow SwitchLinks.
    void turnOn(int32_t objID, int32_t fromID = 0) {
        sendMessageWithLinks({objID, "TurnOn", fromID, 0});
    }

    /// Convenience: send TurnOff to an object and follow SwitchLinks.
    void turnOff(int32_t objID, int32_t fromID = 0) {
        sendMessageWithLinks({objID, "TurnOff", fromID, 0});
    }

    /// Convenience: send FrobWorldEnd and follow SwitchLinks.
    void frobWorldEnd(int32_t objID, int32_t frobberID = 0) {
        sendMessageWithLinks({objID, "FrobWorldEnd", frobberID, 0});
    }

private:
    /// Follow SwitchLink relations from an object and send messages to targets.
    /// "TurnOn" and "FrobWorldEnd" send "TurnOn" to linked objects.
    /// "TurnOff" sends "TurnOff" to linked objects.
    void propagateSwitchLinks(int32_t srcObjID, int32_t fromID,
                               const std::string &triggerMsg) {
        if (!mWorldQuery) return;

        // Determine what message to send to linked targets
        std::string linkMsg = (triggerMsg == "TurnOff") ? "TurnOff" : "TurnOn";

        // Query all SwitchLink relations from this object
        auto links = mWorldQuery->getLinks(srcObjID, "SwitchLink", 0);

        for (const auto &link : links) {
            int32_t targetID = static_cast<int32_t>(link.dst);
            if (targetID == 0 || targetID == srcObjID)
                continue;  // skip self-links and null

            // Send the message to the linked target (no further SwitchLink
            // propagation to avoid infinite loops)
            ScriptMessage targetMsg{targetID, linkMsg, fromID, 0};
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
