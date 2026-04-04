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

// GameLogic.h — Game logic scripts (Tasks 76-80)
//
// StdBook/StdScroll: readable objects
// Loot pickup: frob to collect loot
// QuestService: quest variable storage
// Lock system: key/lock matching on frob

#pragma once

#include <cstdio>
#include <string>
#include <unordered_map>

#include "sim/ScriptBase.h"
#include "sim/ScriptManager.h"
#include "sim/ScriptServices.h"
#include "sim/IScriptServices.h"
#include "sim/MessageDispatch.h"

namespace Darkness {

// ============================================================================
// StdBook — readable object (Task 76a)
// ============================================================================

class StdBook : public ScriptBase {
public:
    using ScriptBase::ScriptBase;
protected:
    void onFrobWorldEnd(ScriptMessage &msg) override {
        // Read the book art/text from P$Book property
        if (svc && svc->property) {
            Variant bookArt = svc->property->get(self, "Book", "");
            if (bookArt.type() != Variant::DV_INVALID) {
                std::string artName = bookArt.toString();
                if (svc->darkUI)
                    svc->darkUI->readBook(self, artName);
            }
        }
    }
};
REGISTER_SCRIPT(StdBook);

// StdScroll is identical behavior, different script name
class StdScroll : public StdBook {
public:
    using StdBook::StdBook;
};
REGISTER_SCRIPT(StdScroll);

// ============================================================================
// Loot pickup script (Task 77a)
// ============================================================================

class StdLoot : public ScriptBase {
public:
    using ScriptBase::ScriptBase;
protected:
    void onFrobWorldEnd(ScriptMessage &msg) override {
        // Credit the player with the loot
        if (svc && svc->darkGame)
            svc->darkGame->foundObject(self);

        // Destroy the world object (picked up into inventory)
        if (svc && svc->object)
            svc->object->destroy(self);
    }
};
REGISTER_SCRIPT(StdLoot);

// ============================================================================
// QuestService — global key-value store for mission state (Task 78)
// ============================================================================
// Not a script per se, but a service used by scripts. Holds quest variables
// loaded from QUEST_DB chunk. Broadcasts QuestChange messages on mutation.

class QuestService {
public:
    QuestService() = default;

    /// Get a quest variable value. Returns 0 if not set.
    int get(const std::string &name) const {
        auto it = mVars.find(name);
        return (it != mVars.end()) ? it->second : 0;
    }

    /// Set a quest variable and broadcast QuestChange if value changed.
    void set(const std::string &name, int value, ScriptManager *mgr = nullptr) {
        int oldValue = get(name);
        mVars[name] = value;

        if (value != oldValue && mgr) {
            // Notify all subscribers
            for (auto &[objID, subName] : mSubscriptions) {
                if (subName == name) {
                    ScriptMessage msg;
                    msg.to = objID;
                    msg.name = "QuestChange";
                    msg.from = 0;
                    msg.data = Variant(std::string(name));
                    msg.data2 = Variant(oldValue);
                    msg.data3 = Variant(value);
                    mgr->sendMessage(msg);
                }
            }
        }
    }

    /// Subscribe an object to changes of a specific quest variable.
    void subscribe(int32_t objID, const std::string &name) {
        mSubscriptions.push_back({objID, name});
    }

    /// Check if a quest variable exists.
    bool exists(const std::string &name) const {
        return mVars.count(name) > 0;
    }

    /// Load quest variables from raw chunk data.
    void loadFromChunk(const uint8_t *data, size_t size) {
        // QUEST_DB format: sequence of {name_len, name, value} entries
        // Parsing deferred until QUEST_DB chunk format is verified
        std::fprintf(stderr, "[QuestService] loadFromChunk: %zu bytes\n", size);
    }

    /// Get all variables (for debug/save).
    const std::unordered_map<std::string, int> &getAll() const { return mVars; }

private:
    std::unordered_map<std::string, int> mVars;
    std::vector<std::pair<int32_t, std::string>> mSubscriptions;
};

// ============================================================================
// TrigQVar — quest variable trigger (Task 73e, depends on Task 78)
// ============================================================================

class TrigQVar : public ScriptBase {
public:
    using ScriptBase::ScriptBase;
protected:
    void onBeginScript(ScriptMessage &) override {
        // Read the quest variable name and condition from properties
        // Subscribe to quest variable changes
        // QuestService subscription will be wired when QuestService is
        // available through IScriptServices
    }

    void onQuestChange(ScriptMessage &msg) override {
        // Evaluate condition: compare new value against condition string
        // Send TurnOn/TurnOff based on result
        int newValue = msg.data3.toInt();
        // Simplified: fire TurnOn if value > 0, TurnOff if 0
        if (newValue > 0)
            broadcastOnAllLinks("TurnOn", "ControlDevice");
        else
            broadcastOnAllLinks("TurnOff", "ControlDevice");
    }
};
REGISTER_SCRIPT(TrigQVar);

// ============================================================================
// Lock system — key/lock matching on frob (Task 80)
// ============================================================================

class StdKey : public ScriptBase {
public:
    using ScriptBase::ScriptBase;
protected:
    void onFrobToolEnd(ScriptMessage &msg) override {
        // msg.from = object being frobbed with this key
        int32_t target = msg.data.toInt();  // target object ID
        if (target == 0) return;

        // Check if this key matches the target's lock
        if (!svc || !svc->property || !svc->locked) return;

        // Read P$KeySrc (this key's properties) and P$KeyDst (target's lock)
        // For now, simplified: if target is locked, unlock it
        if (svc->locked->isLocked(target)) {
            svc->locked->setLocked(target, false);

            // Play unlock sound
            if (svc->sound)
                svc->sound->playEnvSchema(self, "Event Activate, Operation UnlockDoor", self);

            // Notify the door that it was unlocked by a tool frob
            if (mgr) {
                ScriptMessage toolMsg;
                toolMsg.to = target;
                toolMsg.name = "PlayerToolFrob";
                toolMsg.from = self;
                mgr->sendMessage(toolMsg);
            }

            std::fprintf(stderr, "[StdKey] obj %d unlocked target %d\n", self, target);
        }
    }
};
REGISTER_SCRIPT(StdKey);

} // namespace Darkness
