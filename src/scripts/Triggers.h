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

// Triggers.h — Trigger scripts (Task 73)
//
// Trigger scripts convert game events into ControlDevice broadcasts.
// Each trigger listens for a specific event (frob, slain, room enter,
// physics enter/exit, door state change, etc.) and sends TurnOn/TurnOff
// on ControlDevice links.

#pragma once

#include "sim/ScriptBase.h"
#include "sim/ScriptManager.h"
#include "sim/ScriptServices.h"
#include "sim/IScriptServices.h"

namespace Darkness {

// ============================================================================
// TrigWorldFrob — FrobWorldEnd → TurnOn on ControlDevice (Task 73a)
// ============================================================================

class TrigWorldFrob : public ScriptBase {
public:
    using ScriptBase::ScriptBase;
protected:
    void onFrobWorldEnd(ScriptMessage &) override {
        broadcastOnAllLinks("TurnOn", "ControlDevice");
    }
};
REGISTER_SCRIPT(TrigWorldFrob);

// ============================================================================
// TrigSlain — Slain → TurnOn on ControlDevice (Task 73b)
// ============================================================================

class TrigSlain : public ScriptBase {
public:
    using ScriptBase::ScriptBase;
protected:
    void onSlain(ScriptMessage &) override {
        broadcastOnAllLinks("TurnOn", "ControlDevice");
    }
};
REGISTER_SCRIPT(TrigSlain);

// ============================================================================
// TrigRoomPlayer — player room enter/exit → TurnOn/TurnOff (Task 73c)
// ============================================================================

class TrigRoomPlayer : public ScriptBase {
public:
    using ScriptBase::ScriptBase;
protected:
    void onPlayerRoomEnter(ScriptMessage &) override {
        broadcastOnAllLinks("TurnOn", "ControlDevice");
    }
    void onPlayerRoomExit(ScriptMessage &) override {
        broadcastOnAllLinks("TurnOff", "ControlDevice");
    }
};
REGISTER_SCRIPT(TrigRoomPlayer);

// ============================================================================
// TrigOBB — physics volume trigger (Task 73d)
// ============================================================================
// Sends TurnOn when first occupant enters, TurnOff when last leaves.

class TrigOBB : public ScriptBase {
public:
    using ScriptBase::ScriptBase;
protected:
    void onBeginScript(ScriptMessage &) override {
        if (svc && svc->physics)
            svc->physics->subscribeMsg(self, PhysicsScriptService::kEnterExitMsg);
        // Only seed Population on first script start. A re-begin (save/restore,
        // script reload) must preserve any in-progress occupant count;
        // otherwise objects already inside the volume would never fire TurnOff
        // and a re-entry would re-fire TurnOn.
        if (!isDataSet("Population"))
            setData("Population", Variant(0));
    }

    void onEndScript(ScriptMessage &) override {
        if (svc && svc->physics)
            svc->physics->unsubscribeMsg(self, PhysicsScriptService::kEnterExitMsg);
    }

    void onPhysEnter(ScriptMessage &msg) override {
        int pop = getData("Population").toInt();
        pop++;
        setData("Population", Variant(pop));
        if (pop == 1) {
            // First occupant — fire TurnOn
            broadcastOnAllLinks("TurnOn", "ControlDevice");
        }
    }

    void onPhysExit(ScriptMessage &msg) override {
        int pop = getData("Population").toInt();
        if (pop > 0) pop--;
        setData("Population", Variant(pop));
        if (pop == 0) {
            // Last occupant left — fire TurnOff
            broadcastOnAllLinks("TurnOff", "ControlDevice");
        }
    }
};
REGISTER_SCRIPT(TrigOBB);

// ============================================================================
// TrigOBBPlayer — player-only volume trigger variant
// ============================================================================

class TrigOBBPlayer : public TrigOBB {
public:
    using TrigOBB::TrigOBB;
    // Same as TrigOBB but PhysEnter/PhysExit messages only fire for player
    // (EdgeTriggerSystem already only tracks the player)
};
REGISTER_SCRIPT(TrigOBBPlayer);

// ============================================================================
// TrigDoorOpen — door open/close → TurnOn/TurnOff (Task 73g)
// ============================================================================

class TrigDoorOpen : public ScriptBase {
public:
    using ScriptBase::ScriptBase;
protected:
    void onDoorOpen(ScriptMessage &) override {
        broadcastOnAllLinks("TurnOn", "ControlDevice");
    }
    void onDoorClose(ScriptMessage &) override {
        broadcastOnAllLinks("TurnOff", "ControlDevice");
    }
};
REGISTER_SCRIPT(TrigDoorOpen);

// ============================================================================
// TrigUnlock — lock state change → TurnOn/TurnOff (Task 73h)
// ============================================================================

class TrigUnlock : public ScriptBase {
public:
    using ScriptBase::ScriptBase;
protected:
    void onNowUnlocked(ScriptMessage &) override {
        broadcastOnAllLinks("TurnOn", "ControlDevice");
    }
    void onNowLocked(ScriptMessage &) override {
        broadcastOnAllLinks("TurnOff", "ControlDevice");
    }
};
REGISTER_SCRIPT(TrigUnlock);

// ============================================================================
// TrigInvFrob — inventory frob → TurnOn (Task 73j)
// ============================================================================

class TrigInvFrob : public ScriptBase {
public:
    using ScriptBase::ScriptBase;
protected:
    void onFrobInvEnd(ScriptMessage &) override {
        broadcastOnAllLinks("TurnOn", "ControlDevice");
    }
};
REGISTER_SCRIPT(TrigInvFrob);

// ============================================================================
// TrigFlicker — tweq flicker completion → alternating TurnOn/TurnOff (73f)
// ============================================================================

class TrigFlicker : public ScriptBase {
public:
    using ScriptBase::ScriptBase;
protected:
    void onTweqComplete(ScriptMessage &) override {
        bool lastWasOn = isDataSet("LastOn") && getData("LastOn").toBool();
        bool sendOn = !lastWasOn;
        setData("LastOn", Variant(sendOn));
        broadcastOnAllLinks(sendOn ? "TurnOn" : "TurnOff", "ControlDevice");
    }
};
REGISTER_SCRIPT(TrigFlicker);

// ============================================================================
// TrigRoomCreature — creature room tracking (Task 73i)
// ============================================================================

class TrigRoomCreature : public ScriptBase {
public:
    using ScriptBase::ScriptBase;
protected:
    void onCreatureRoomEnter(ScriptMessage &) override {
        int pop = isDataSet("Population") ? getData("Population").toInt() : 0;
        pop++;
        setData("Population", Variant(pop));
        if (pop == 1)
            broadcastOnAllLinks("TurnOn", "ControlDevice");
    }
    void onCreatureRoomExit(ScriptMessage &) override {
        int pop = isDataSet("Population") ? getData("Population").toInt() : 0;
        if (pop > 0) pop--;
        setData("Population", Variant(pop));
        if (pop == 0)
            broadcastOnAllLinks("TurnOff", "ControlDevice");
    }
};
REGISTER_SCRIPT(TrigRoomCreature);

} // namespace Darkness
