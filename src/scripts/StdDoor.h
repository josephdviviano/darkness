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

// StdDoor.h — Standard door script (Task 68)
//
// Reimplements the Dark Engine's StdDoor script behavior: doors open/close
// on frob and respond to TurnOn/TurnOff messages. Supports:
//   - Lock-aware frobbing (locked doors reject frob when closed)
//   - Auto-close timer via ScriptTiming property (milliseconds)
//   - Double-door synchronization via ScriptParams "Double" links
//   - Door state sound effects via StateChange env_tags
//   - ControlDevice broadcast on state changes
//   - NowLocked → close, NowUnlocked → open
//   - Slain → force open, remove locks
//
// DoorSystem retains physics simulation (rotation/translation) and fires
// DoorOpening/DoorClosing/DoorOpen/DoorClose/DoorHalt messages to scripts.
// This script calls svc->door to control the physical door.

#pragma once

#include "sim/ScriptBase.h"
#include "sim/ScriptManager.h"
#include "sim/ScriptServices.h"
#include "sim/IScriptServices.h"

namespace Darkness {

class StdDoor : public ScriptBase {
public:
    StdDoor(int32_t objID, const std::string &className,
            IScriptServices *services, ScriptManager *manager)
        : ScriptBase(objID, className, services, manager) {}

protected:
    // ── Lifecycle ──

    void onSim(ScriptMessage &msg) override {
        if (msg.data.toInt() == 1) {
            // Sim starting
            setData("Sim", Variant(true));
            // Double-door AI repulsion initialization deferred (needs AI)
        } else {
            clearData("Sim");
        }
    }

    // ── Activation ──

    void onTurnOn(ScriptMessage &msg) override {
        // TurnOn always opens, even if locked (matching original engine)
        if (svc && svc->door)
            svc->door->openDoor(self);
    }

    void onTurnOff(ScriptMessage &msg) override {
        // TurnOff always closes, even if locked
        if (svc && svc->door)
            svc->door->closeDoor(self);
    }

    // ── Frob ──

    void onFrobWorldEnd(ScriptMessage &msg) override {
        if (!svc || !svc->door) return;

        // Check lock state — locked doors reject frob when closed
        bool locked = false;
        if (svc->locked && svc->locked->isLocked(self) &&
            svc->door->getDoorState(self) == kDoorClosed) {
            locked = true;
        }

        if (locked) {
            // Play reject sound
            if (svc->sound)
                svc->sound->playEnvSchema(self, "Event Reject, Operation OpenDoor", self);
            return;
        }

        // Track player frob for sound tags
        // msg.from == 0 for player frob (or could check archetype)
        setData("PlayerFrob", Variant(0));

        // If halted mid-motion, resume in the opposite direction
        if (isDataSet("BeforeHalt")) {
            int before = getData("BeforeHalt").toInt();
            clearData("BeforeHalt");
            // kDoorOpening = 3 — was opening when halted, so close
            if (before == kDoorOpening)
                svc->door->closeDoor(self);
            else
                svc->door->openDoor(self);
        } else {
            svc->door->toggleDoor(self);
        }
    }

    // ── Door state events (fired by DoorSystem) ──

    void onDoorOpening(ScriptMessage &msg) override {
        // Sync double doors
        pingDoubles();

        // Credit discovery
        if (svc && svc->darkGame)
            svc->darkGame->foundObject(self);

        // Play state change sound
        playStateChangeSound(msg);

        // Broadcast on ControlDevice links
        broadcastOnAllLinks("TurnOn", "ControlDevice");
    }

    void onDoorClosing(ScriptMessage &msg) override {
        pingDoubles();
        playStateChangeSound(msg);

        // Cancel auto-close timer when manually closing
        cancelCloseTimer();

        broadcastOnAllLinks("TurnOff", "ControlDevice");
    }

    void onDoorOpen(ScriptMessage &msg) override {
        // Halt any currently playing door movement sound
        if (svc && svc->sound)
            svc->sound->haltSchema(self, "");
        playStateChangeSound(msg);

        // Start auto-close timer if ScriptTiming property exists
        checkAutoCloseTimer();
    }

    void onDoorClose(ScriptMessage &msg) override {
        if (svc && svc->sound)
            svc->sound->haltSchema(self, "");
        playStateChangeSound(msg);
    }

    void onDoorHalt(ScriptMessage &msg) override {
        // Remember which direction we were going before the halt
        setData("BeforeHalt", msg.data2);  // data2 = prevActionType
        playStateChangeSound(msg);
        checkAutoCloseTimer();
    }

    // ── Lock events ──

    void onNowLocked(ScriptMessage &msg) override {
        if (isDataSet("Sim")) {
            pingDoubles();
            if (svc && svc->door)
                svc->door->closeDoor(self);
        }
    }

    void onNowUnlocked(ScriptMessage &msg) override {
        if (isDataSet("Sim")) {
            pingDoubles();
            if (svc && svc->door)
                svc->door->openDoor(self);
        }
    }

    // ── Timer ──

    void onTimer(ScriptMessage &msg) override {
        std::string timerName = msg.data.toString();
        if (timerName == "Close") {
            clearData("CloseTimer");
            if (svc && svc->door)
                svc->door->closeDoor(self);
        }
    }

    // ── Slain ──

    void onSlain(ScriptMessage &msg) override {
        // Remove locks and force open
        if (svc && svc->link) {
            // Destroy all Lock links
            svc->link->destroyMany("Lock", self, 0);
        }

        if (svc && svc->property) {
            if (svc->property->possessed(self, "Locked")) {
                svc->property->set(self, "Locked", "", Variant(false));
            }
            if (svc->property->possessed(self, "KeyDst")) {
                // Remove key destination property
                // Property removal not yet supported — just clear it
            }
        }

        if (svc && svc->door)
            svc->door->openDoor(self);
    }

    // ── SynchUp (double door sync) ──

    void onMessage(ScriptMessage &msg) override {
        if (msg.name == "SynchUp") {
            // Synchronize with the other door in the pair
            if (!svc || !svc->door) return;

            int32_t otherDoor = msg.from;
            DoorStatus otherState = svc->door->getDoorState(otherDoor);
            DoorStatus myState = svc->door->getDoorState(self);

            // Normalize to target state (opening→open, closing→closed)
            int otherTarget = targetState(otherState);
            int myTarget = targetState(myState);

            if (myTarget != otherTarget) {
                if (otherTarget == kDoorClosed)
                    svc->door->closeDoor(self);
                else
                    svc->door->openDoor(self);
            }

            // Sync lock state
            if (svc->locked) {
                bool otherLocked = svc->locked->isLocked(otherDoor);
                bool myLocked = svc->locked->isLocked(self);
                if (myLocked != otherLocked) {
                    svc->locked->setLocked(self, otherLocked);
                }
            }
        }
        // TurnOn/TurnOff as catch-all (matching original engine's OnMessage)
        else if (msg.name == "Open") {
            if (svc && svc->door) svc->door->openDoor(self);
        }
        else if (msg.name == "Close") {
            if (svc && svc->door) svc->door->closeDoor(self);
        }
        else if (msg.name == "PlayerToolFrob") {
            setData("PlayerFrob", Variant(0));
        }
    }

private:
    /// Broadcast SynchUp to all ScriptParams-linked double doors.
    void pingDoubles() {
        broadcastOnAllLinks("SynchUp", "ScriptParams");
        // Also broadcast on inverse ScriptParams links
        broadcastOnAllLinks("SynchUp", "~ScriptParams");
    }

    /// Normalize in-progress states to their target states.
    static int targetState(DoorStatus state) {
        if (state == kDoorClosing) return kDoorClosed;
        if (state == kDoorOpening) return kDoorOpen;
        return state;
    }

    /// Play door state change sound with appropriate tags.
    void playStateChangeSound(ScriptMessage &msg) {
        if (!svc || !svc->sound) return;

        // Build tag string: "Event StateChange, OpenState <state>, OldOpenState <old>"
        static const char *stateNames[] = {"Open", "Closed", "Closing", "Opening", "Halted"};
        int newState = msg.data.toInt();
        int oldState = msg.data2.toInt();

        if (newState < 0 || newState > 4) newState = 0;
        if (oldState < 0 || oldState > 4) oldState = 0;

        std::string tags = "Event StateChange, OpenState ";
        tags += stateNames[newState];
        tags += ", OldOpenState ";
        tags += stateNames[oldState];

        // Add CreatureType Player if this was a player frob
        if (oldState != 4 && isDataSet("PlayerFrob")) {
            tags += ", CreatureType Player";
        }

        // Clear player frob data when reaching a stable state
        if (newState != kDoorClosing && newState != kDoorOpening) {
            clearData("PlayerFrob");
        }

        svc->sound->playEnvSchema(self, tags, self);
    }

    /// Check if ScriptTiming property exists and start auto-close timer.
    void checkAutoCloseTimer() {
        if (!svc || !svc->property) return;

        if (svc->property->possessed(self, "ScriptTiming")) {
            cancelCloseTimer();
            Variant timeVal = svc->property->get(self, "ScriptTiming");
            int timeMs = timeVal.toInt();
            if (timeMs > 0) {
                timer_handle h = setOneShotTimer("Close",
                                                  static_cast<float>(timeMs) / 1000.0f);
                setData("CloseTimer", Variant(static_cast<int>(h)));
            }
        }
    }

    /// Cancel any pending auto-close timer.
    void cancelCloseTimer() {
        if (isDataSet("CloseTimer")) {
            int h = getData("CloseTimer").toInt();
            killTimer(static_cast<timer_handle>(h));
            clearData("CloseTimer");
        }
    }
};

REGISTER_SCRIPT(StdDoor);

/// NonAutoDoor — door that does NOT auto open/close on lock/unlock.
class NonAutoDoor : public StdDoor {
public:
    NonAutoDoor(int32_t objID, const std::string &className,
                IScriptServices *services, ScriptManager *manager)
        : StdDoor(objID, className, services, manager) {}

protected:
    void onNowLocked(ScriptMessage &) override {}
    void onNowUnlocked(ScriptMessage &) override {}
};

REGISTER_SCRIPT(NonAutoDoor);

} // namespace Darkness
