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

// StdTrap.h — Trap base class and relay/timing trap scripts (Tasks 71-72)
//
// Reimplements the Dark Engine's trap/trigger system. StdTrap is the base
// class for all traps: it reads P$TrapFlags (Once, Invert, NoOn, NoOff),
// checks Locked state, and routes to virtual activate(). Derived traps
// implement specific behavior (relay, delay, destroy, teleport, etc.).

#pragma once

#include "sim/ScriptBase.h"
#include "sim/ScriptManager.h"
#include "sim/ScriptServices.h"
#include "sim/IScriptServices.h"

namespace Darkness {

// TrapFlags bits (matching Dark Engine TRAPF_* constants)
static constexpr uint32_t TRAPF_ONCE   = 0x01;
static constexpr uint32_t TRAPF_INVERT = 0x02;
static constexpr uint32_t TRAPF_NOON   = 0x04;
static constexpr uint32_t TRAPF_NOOFF  = 0x08;

// ============================================================================
// StdTrap — base class for all traps (Task 71a)
// ============================================================================

class StdTrap : public ScriptBase {
public:
    StdTrap(int32_t objID, const std::string &className,
            IScriptServices *services, ScriptManager *manager)
        : ScriptBase(objID, className, services, manager) {}

protected:
    /// Override in derived traps — called when activation passes all filters.
    virtual void doActivate(bool on, int32_t sender) {}

    void onTurnOn(ScriptMessage &msg) override {
        handleMessage(true, msg.from);
    }

    void onTurnOff(ScriptMessage &msg) override {
        handleMessage(false, msg.from);
    }

private:
    void handleMessage(bool on, int32_t sender) {
        // Check locked state
        if (svc && svc->locked && svc->locked->isLocked(self))
            return;

        // Read trap flags
        uint32_t flags = 0;
        if (svc && svc->property) {
            PropTrapFlags tf{};
            if (svc->property->getTyped(self, "TrapFlags", tf))
                flags = tf.flags;
        }

        // Filter: NoOn / NoOff
        if (on && (flags & TRAPF_NOON))  return;
        if (!on && (flags & TRAPF_NOOFF)) return;

        // Invert
        if (flags & TRAPF_INVERT) on = !on;

        doActivate(on, sender);

        // Once: lock after first use
        if (flags & TRAPF_ONCE) {
            if (svc && svc->locked)
                svc->locked->setLocked(self, true);
        }
    }
};

// ============================================================================
// TrapRelay — broadcasts to ControlDevice links (Task 71b)
// ============================================================================

class TrapRelay : public StdTrap {
public:
    using StdTrap::StdTrap;
protected:
    void doActivate(bool on, int32_t sender) override {
        // Optional probability filter via ScriptTiming (as percentage 0-100)
        if (svc && svc->property && svc->data) {
            Variant timing = svc->property->get(self, "ScriptTiming");
            if (timing.type() != Variant::DV_INVALID) {
                int pct = timing.toInt();
                if (pct > 0 && pct < 100) {
                    if (svc->data->randFloat01() * 100.0f > static_cast<float>(pct))
                        return;
                }
            }
        }
        broadcastOnAllLinks(on ? "TurnOn" : "TurnOff", "ControlDevice");
    }
};
REGISTER_SCRIPT(TrapRelay);

// ============================================================================
// TrapTimedRelay — delayed broadcast (Task 71c)
// ============================================================================

class TrapTimedRelay : public StdTrap {
public:
    using StdTrap::StdTrap;
protected:
    void doActivate(bool on, int32_t sender) override {
        // Cancel any pending timer
        if (isDataSet("PendingTimer")) {
            killTimer(static_cast<timer_handle>(getData("PendingTimer").toInt()));
            clearData("PendingTimer");
        }

        // Read delay from ScriptTiming (milliseconds)
        float delay = 0.0f;
        if (svc && svc->property) {
            Variant timing = svc->property->get(self, "ScriptTiming");
            if (timing.type() != Variant::DV_INVALID)
                delay = static_cast<float>(timing.toInt()) / 1000.0f;
        }

        if (delay <= 0.0f) {
            // Immediate relay
            broadcastOnAllLinks(on ? "TurnOn" : "TurnOff", "ControlDevice");
        } else {
            timer_handle h = setOneShotTimer("Relay", delay, Variant(on));
            setData("PendingTimer", Variant(static_cast<int>(h)));
        }
    }

    void onTimer(ScriptMessage &msg) override {
        if (msg.data.toString() == "Relay") {
            clearData("PendingTimer");
            bool on = msg.data2.toBool();
            broadcastOnAllLinks(on ? "TurnOn" : "TurnOff", "ControlDevice");
        }
    }
};
REGISTER_SCRIPT(TrapTimedRelay);

// ============================================================================
// TrapRevert — broadcast then auto-revert after delay (Task 71d)
// ============================================================================

class TrapRevert : public StdTrap {
public:
    using StdTrap::StdTrap;
protected:
    void doActivate(bool on, int32_t sender) override {
        broadcastOnAllLinks(on ? "TurnOn" : "TurnOff", "ControlDevice");

        // Set timer for opposite message
        float delay = 0.0f;
        if (svc && svc->property) {
            Variant timing = svc->property->get(self, "ScriptTiming");
            if (timing.type() != Variant::DV_INVALID)
                delay = static_cast<float>(timing.toInt()) / 1000.0f;
        }

        if (delay > 0.0f)
            setOneShotTimer("Revert", delay, Variant(!on));
    }

    void onTimer(ScriptMessage &msg) override {
        if (msg.data.toString() == "Revert") {
            bool on = msg.data2.toBool();
            broadcastOnAllLinks(on ? "TurnOn" : "TurnOff", "ControlDevice");
        }
    }
};
REGISTER_SCRIPT(TrapRevert);

// ============================================================================
// TrapDestroy — slays all ControlDevice-linked objects (Task 71e)
// ============================================================================

class TrapDestroy : public StdTrap {
public:
    using StdTrap::StdTrap;
protected:
    void doActivate(bool on, int32_t sender) override {
        if (!on) return;  // only activate on TurnOn
        if (!svc || !svc->link || !svc->damage) return;
        auto links = svc->link->getAll("ControlDevice", self);
        for (auto &link : links) {
            svc->damage->slay(link.dst, self);
        }
    }
};
REGISTER_SCRIPT(TrapDestroy);

// ============================================================================
// TrapTeleporter — teleports linked object to self position (Task 71f)
// ============================================================================

class TrapTeleporter : public StdTrap {
public:
    using StdTrap::StdTrap;
protected:
    void doActivate(bool on, int32_t sender) override {
        if (!on) return;
        if (!svc || !svc->link || !svc->object) return;
        auto link = svc->link->getOne("ControlDevice", self);
        if (link.dst != 0) {
            Vector3 myPos = svc->object->position(self);
            svc->object->teleport(link.dst, myPos, Vector3(0));
        }
    }
};
REGISTER_SCRIPT(TrapTeleporter);

// ============================================================================
// TrapRequireAll — AND gate (Task 72a)
// ============================================================================
// Fires TurnOn only when ALL inputs are on. Fires TurnOff when any turns off.

class TrapRequireAll : public StdTrap {
public:
    using StdTrap::StdTrap;
protected:
    void doActivate(bool on, int32_t sender) override {
        // Track per-sender state
        std::string key = "input_" + std::to_string(sender);
        setData(key, Variant(on));

        if (on) {
            // Check if ALL inputs are on
            if (!svc || !svc->link) return;
            // Get all incoming ControlDevice links (backlinks)
            // For now, track senders and check all are on
            // Simplified: if any input is off, don't fire
            bool allOn = checkAllInputs();
            if (allOn)
                broadcastOnAllLinks("TurnOn", "ControlDevice");
        } else {
            // Any input off → fire TurnOff
            broadcastOnAllLinks("TurnOff", "ControlDevice");
        }
    }

private:
    bool checkAllInputs() {
        // Check all tracked inputs
        // This is a simplified approach — in the original engine,
        // this uses Population links to track input objects
        return true;  // Simplified for now
    }
};
REGISTER_SCRIPT(TrapRequireAll);

// ============================================================================
// TrapRequireAny — OR gate (Task 72b)
// ============================================================================

class TrapRequireAny : public StdTrap {
public:
    using StdTrap::StdTrap;
protected:
    void doActivate(bool on, int32_t sender) override {
        std::string key = "input_" + std::to_string(sender);
        setData(key, Variant(on));

        if (on) {
            // Any input on → fire TurnOn
            broadcastOnAllLinks("TurnOn", "ControlDevice");
        } else {
            // Check if ALL are off before firing TurnOff
            // Simplified: fire TurnOff on any off
            broadcastOnAllLinks("TurnOff", "ControlDevice");
        }
    }
};
REGISTER_SCRIPT(TrapRequireAny);

} // namespace Darkness
