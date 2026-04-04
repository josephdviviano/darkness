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

// StdLever.h — Standard lever, button, and two-state device scripts (Task 69)
//
// Reimplements the Dark Engine's StdController → StdTwoState → StdLever
// hierarchy and StdButton script. These are the primary player interaction
// scripts for switches and buttons.
//
// StdController: Base class with Broadcast() method that respects TrapFlags
//   and sends TurnOn/TurnOff on ControlDevice links.
// StdTwoState: Two-state device using tweq animation (forward/reverse).
//   Responds to TurnOn/TurnOff/Toggle messages.
// StdLever: Frobbing toggles tweq, broadcasts on ControlDevice after tweq
//   completes. Lock-aware.
// StdButton: Frobbing or physics collision sends TurnOn, plays sound,
//   bounces tweq.

#pragma once

#include "sim/ScriptBase.h"
#include "sim/ScriptManager.h"
#include "sim/ScriptServices.h"
#include "sim/IScriptServices.h"

namespace Darkness {

// ── Tweq direction constants (matching Dark Engine kTweqDir*) ──
static constexpr int kTweqDirForward = 0;
static constexpr int kTweqDirReverse = 1;

// ============================================================================
// StdController — base for all interactive control devices
// ============================================================================
// Provides Broadcast() that respects TrapFlags and sends TurnOn/TurnOff
// on ControlDevice links.

class StdController : public ScriptBase {
public:
    StdController(int32_t objID, const std::string &className,
                  IScriptServices *services, ScriptManager *manager)
        : ScriptBase(objID, className, services, manager) {}

protected:
    /// Broadcast TurnOn or TurnOff on ControlDevice links, respecting TrapFlags.
    void broadcast(bool on) {
        uint32_t trapFlags = 0;
        if (svc && svc->property) {
            PropTrapFlags tf{};
            if (svc->property->getTyped(self, "TrapFlags", tf))
                trapFlags = tf.flags;
        }

        // Check NoOn / NoOff flags
        if (on && (trapFlags & 0x04))  return;  // TRAPF_NOON
        if (!on && (trapFlags & 0x08)) return;  // TRAPF_NOOFF

        // Invert flag swaps sense
        if (trapFlags & 0x02) on = !on;  // TRAPF_INVERT

        broadcastOnAllLinks(on ? "TurnOn" : "TurnOff", "ControlDevice");

        if (on && svc && svc->darkGame)
            svc->darkGame->foundObject(self);

        // Once flag: lock after first activation
        if (trapFlags & 0x01) {  // TRAPF_ONCE
            if (svc && svc->locked)
                svc->locked->setLocked(self, true);
        }
    }
};

// ============================================================================
// StdTwoState — two-state device with tweq animation
// ============================================================================

class StdTwoState : public StdController {
public:
    StdTwoState(int32_t objID, const std::string &className,
                IScriptServices *services, ScriptManager *manager)
        : StdController(objID, className, services, manager) {}

protected:
    /// Override in derived classes for pre/post tweq activation hooks.
    virtual void preTweqActivate(int dir) {}
    virtual void postTweqActivate(int dir) {}

    /// Get the target state of this device (forward or reverse).
    /// Forward = active bit differs from reverse bit in tweq anim state.
    int targetState() const {
        bool active = tweqAnimStateHasVal(self, 0x01);
        bool reverse = tweqAnimStateHasVal(self, 0x02);
        return (active != reverse) ? kTweqDirForward : kTweqDirReverse;
    }

    /// Activate/reverse the tweq animation without side effects.
    void tweqActivateNoSE() {
        if (!tweqAnimStateHasVal(self, 0x01)) {
            // Tweq is OFF — play state change sound
            int targ = targetState();
            std::string tags = "Event StateChange, DirectionState ";
            tags += (targ == kTweqDirForward) ? "Forward" : "Reverse";
            if (svc && svc->sound)
                svc->sound->playEnvSchema(self, tags, self);

            preTweqActivate(targ);
        }

        // Activate the tweq (toggle direction)
        if (svc && svc->tweqSystem) {
            svc->tweqSystem->activate(self, kTweqDoDefault);
        }
    }

    /// Activate with side effects (broadcasts to ToggleNoSE on ScriptParams).
    void tweqActivate() {
        tweqActivateNoSE();
        broadcastOnAllLinks("ToggleNoSE", "ScriptParams");
    }

    /// Move towards a specific state if not already there.
    void goTowardsState(int stateId) {
        if (targetState() == stateId) return;
        tweqActivate();
    }

    // ── Message handlers ──

    void onTurnOn(ScriptMessage &msg) override {
        goTowardsState(kTweqDirForward);
    }

    void onTurnOff(ScriptMessage &msg) override {
        goTowardsState(kTweqDirReverse);
    }

    void onTweqComplete(ScriptMessage &msg) override {
        // msg.data = tweq type, msg.data2 = direction
        int dir = msg.data2.toInt();
        postTweqActivate(dir);
    }

    void onMessage(ScriptMessage &msg) override {
        if (msg.name == "GoForward")
            goTowardsState(kTweqDirForward);
        else if (msg.name == "GoBackward")
            goTowardsState(kTweqDirReverse);
        else if (msg.name == "Toggle")
            tweqActivate();
        else if (msg.name == "ToggleNoSE")
            tweqActivateNoSE();
        else if (msg.name == "SynchUp")
            goTowardsState(targetStateOf(msg.from));
    }

private:
    /// Check if a tweq anim state bit is set on an object.
    bool tweqAnimStateHasVal(int32_t objID, int mask) const {
        // Check StTweqJoints or StTweqRotate AnimS field
        if (!svc || !svc->property) return false;

        Variant animS = svc->property->get(objID, "StTweqJoints", "AnimS");
        if (animS.type() == Variant::DV_INVALID)
            animS = svc->property->get(objID, "StTweqRotate", "AnimS");
        if (animS.type() == Variant::DV_INVALID)
            return false;

        return (animS.toInt() & mask) != 0;
    }

    int targetStateOf(int32_t objID) const {
        bool active = tweqAnimStateHasVal(objID, 0x01);
        bool reverse = tweqAnimStateHasVal(objID, 0x02);
        return (active != reverse) ? kTweqDirForward : kTweqDirReverse;
    }
};

// ============================================================================
// StdLever — frobbing toggles tweq, broadcasts on completion
// ============================================================================

class StdLever : public StdTwoState {
public:
    StdLever(int32_t objID, const std::string &className,
             IScriptServices *services, ScriptManager *manager)
        : StdTwoState(objID, className, services, manager) {}

protected:
    void postTweqActivate(int dir) override {
        // Lever completed animation �� broadcast result
        if (dir == kTweqDirReverse)
            broadcast(false);  // lever up → TurnOff
        else if (dir == kTweqDirForward)
            broadcast(true);   // lever down → TurnOn
    }

    void onFrobWorldEnd(ScriptMessage &msg) override {
        // Lock-aware: locked levers send error output instead
        if (svc && svc->locked && svc->locked->isLocked(self)) {
            broadcastOnAllLinks("TurnOn", "ScriptParams");
            return;
        }
        tweqActivate();
    }
};

REGISTER_SCRIPT(StdLever);

// ============================================================================
// LeverNoChain — broadcasts ToggleNoSE instead of TurnOn/TurnOff
// ============================================================================

class LeverNoChain : public StdLever {
public:
    LeverNoChain(int32_t objID, const std::string &className,
                 IScriptServices *services, ScriptManager *manager)
        : StdLever(objID, className, services, manager) {}

protected:
    void postTweqActivate(int dir) override {
        broadcastOnAllLinks("ToggleNoSE", "ControlDevice");
    }
};

REGISTER_SCRIPT(LeverNoChain);

// ============================================================================
// StdButton — frobbing or physics collision sends TurnOn
// ============================================================================

class StdButton : public StdController {
public:
    StdButton(int32_t objID, const std::string &className,
              IScriptServices *services, ScriptManager *manager)
        : StdController(objID, className, services, manager) {}

protected:
    void onBeginScript(ScriptMessage &msg) override {
        // Subscribe to physics collision messages
        if (svc && svc->physics)
            svc->physics->subscribeMsg(self, PhysicsScriptService::kCollisionMsg);
    }

    void onEndScript(ScriptMessage &msg) override {
        if (svc && svc->physics)
            svc->physics->unsubscribeMsg(self, PhysicsScriptService::kCollisionMsg);
    }

    void onPhysCollision(ScriptMessage &msg) override {
        // Only respond to front-face collisions (submod 4)
        int collSubmod = msg.data.toInt();
        if (collSubmod == 4) {
            // Ignore AI/creature collisions
            int32_t collObj = msg.data2.toInt();
            if (svc && svc->object) {
                if (svc->object->inheritsFrom(collObj, -1 /* Avatar archetype */))
                    return;
                // Could also check Creature archetype
            }
            buttonPush();
        }
    }

    void onFrobWorldEnd(ScriptMessage &msg) override {
        buttonPush();
    }

private:
    void buttonPush() {
        // Play activation sound
        if (svc && svc->sound)
            svc->sound->playEnvSchema(self, "Event Activate", self);

        // Broadcast TurnOn on ControlDevice links
        broadcast(true);

        // Bounce the button tweq
        if (svc && svc->tweqSystem)
            svc->tweqSystem->activate(self, kTweqDoActivate);

        // Credit discovery
        if (svc && svc->darkGame)
            svc->darkGame->foundObject(self);
    }
};

REGISTER_SCRIPT(StdButton);

} // namespace Darkness
