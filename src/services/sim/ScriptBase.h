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

// ScriptBase.h — Per-object script base class
//
// Reimplements the Dark Engine's cScript base class pattern. Each object can
// have up to 4 scripts (from P$Scripts), each instantiated as a ScriptBase
// subclass. Scripts receive named messages via receiveMessage() which routes
// to type-specific virtual handlers (onTurnOn, onFrobWorldEnd, etc.).
//
// Scripts interact with the engine through IScriptServices (thin wrappers
// around existing OPDE services) and ScriptManager (for messaging/timers).
//
// Inspired by the openDarkEngine (OPDE) service architecture (GPLv2).

#pragma once

#include <cstdint>
#include <string>
#include "dyntype/Variant.h"
#include "sim/MessageDispatch.h"

namespace Darkness {

// Forward declarations
class ScriptManager;
struct IScriptServices;

// ── Script result codes ──
enum eScriptResult : int32_t {
    kSR_NoAction  = 0,  // message not handled
    kSR_Handled   = 1,  // message handled, continue dispatch
    kSR_Consumed  = 2,  // message handled, stop dispatch
};

// ── Timer handle for cancellation ──
using timer_handle = uint32_t;
static constexpr timer_handle kInvalidTimer = 0;

// ============================================================================
// ScriptBase — per-object script instance
// ============================================================================
//
// Derived classes override the on* virtual methods to respond to messages.
// The receiveMessage() dispatch uses a string→handler lookup that avoids
// a giant if/else chain while keeping the virtual override pattern clean.

class ScriptBase {
public:
    ScriptBase(int32_t objID, const std::string &className,
               IScriptServices *services, ScriptManager *manager)
        : self(objID), mClassName(className), svc(services), mgr(manager) {}

    virtual ~ScriptBase() = default;

    /// Main entry point — routes named messages to virtual handlers.
    /// Returns kSR_NoAction if unhandled, allowing fallback to global handlers.
    virtual eScriptResult receiveMessage(ScriptMessage &msg) {
        const std::string &n = msg.name;

        // Lifecycle
        if (n == "BeginScript")   { onBeginScript(msg); return kSR_Handled; }
        if (n == "EndScript")     { onEndScript(msg);   return kSR_Handled; }
        if (n == "Sim")           { onSim(msg);         return kSR_Handled; }
        if (n == "Timer")         { onTimer(msg);       return kSR_Handled; }

        // Activation
        if (n == "TurnOn")        { onTurnOn(msg);      return kSR_Handled; }
        if (n == "TurnOff")       { onTurnOff(msg);     return kSR_Handled; }

        // Frob
        if (n == "FrobWorldEnd")  { onFrobWorldEnd(msg); return kSR_Handled; }
        if (n == "FrobInvEnd")    { onFrobInvEnd(msg);   return kSR_Handled; }
        if (n == "FrobToolEnd")   { onFrobToolEnd(msg);  return kSR_Handled; }

        // Physics
        if (n == "PhysCollision") { onPhysCollision(msg); return kSR_Handled; }
        if (n == "PhysEnter")     { onPhysEnter(msg);     return kSR_Handled; }
        if (n == "PhysExit")      { onPhysExit(msg);      return kSR_Handled; }

        // Door state
        if (n == "DoorOpen")      { onDoorOpen(msg);     return kSR_Handled; }
        if (n == "DoorClose")     { onDoorClose(msg);    return kSR_Handled; }
        if (n == "DoorOpening")   { onDoorOpening(msg);  return kSR_Handled; }
        if (n == "DoorClosing")   { onDoorClosing(msg);  return kSR_Handled; }
        if (n == "DoorHalt")      { onDoorHalt(msg);     return kSR_Handled; }

        // Room
        if (n == "PlayerRoomEnter")   { onPlayerRoomEnter(msg);   return kSR_Handled; }
        if (n == "PlayerRoomExit")    { onPlayerRoomExit(msg);    return kSR_Handled; }
        if (n == "CreatureRoomEnter") { onCreatureRoomEnter(msg); return kSR_Handled; }
        if (n == "CreatureRoomExit")  { onCreatureRoomExit(msg);  return kSR_Handled; }

        // Container
        if (n == "Container")     { onContainer(msg);   return kSR_Handled; }
        if (n == "Contained")     { onContained(msg);   return kSR_Handled; }

        // Damage
        if (n == "Slain")         { onSlain(msg);       return kSR_Handled; }
        if (n == "Damage")        { onDamage(msg);      return kSR_Handled; }

        // Animation/Sound
        if (n == "TweqComplete")  { onTweqComplete(msg); return kSR_Handled; }
        if (n == "MotionEnd")     { onMotionEnd(msg);    return kSR_Handled; }
        if (n == "SchemaDone")    { onSchemaDone(msg);   return kSR_Handled; }

        // Lock
        if (n == "NowLocked")     { onNowLocked(msg);    return kSR_Handled; }
        if (n == "NowUnlocked")   { onNowUnlocked(msg);  return kSR_Handled; }

        // Quest/Elevator
        if (n == "QuestChange")           { onQuestChange(msg);           return kSR_Handled; }
        if (n == "MovingTerrainWaypoint") { onMovingTerrainWaypoint(msg); return kSR_Handled; }

        // Catch-all for any unmatched message name
        onMessage(msg);
        return kSR_NoAction;
    }

    /// Object ID this script is attached to
    int32_t getObjID() const { return self; }

    /// Script class name (e.g. "StdDoor", "AnimLight")
    const std::string &getClassName() const { return mClassName; }

protected:
    int32_t self;                  // object ID this script is attached to
    std::string mClassName;        // script class name
    IScriptServices *svc;          // engine service access
    ScriptManager *mgr;            // for sending messages / timers

    // ── Lifecycle ──
    virtual void onBeginScript(ScriptMessage &) {}
    virtual void onEndScript(ScriptMessage &) {}
    virtual void onSim(ScriptMessage &) {}
    virtual void onTimer(ScriptMessage &) {}

    // ── Activation ──
    virtual void onTurnOn(ScriptMessage &) {}
    virtual void onTurnOff(ScriptMessage &) {}

    // ── Frob ──
    virtual void onFrobWorldEnd(ScriptMessage &) {}
    virtual void onFrobInvEnd(ScriptMessage &) {}
    virtual void onFrobToolEnd(ScriptMessage &) {}

    // ── Physics ──
    virtual void onPhysCollision(ScriptMessage &) {}
    virtual void onPhysEnter(ScriptMessage &) {}
    virtual void onPhysExit(ScriptMessage &) {}

    // ── Door state ──
    virtual void onDoorOpen(ScriptMessage &) {}
    virtual void onDoorClose(ScriptMessage &) {}
    virtual void onDoorOpening(ScriptMessage &) {}
    virtual void onDoorClosing(ScriptMessage &) {}
    virtual void onDoorHalt(ScriptMessage &) {}

    // ── Room ──
    virtual void onPlayerRoomEnter(ScriptMessage &) {}
    virtual void onPlayerRoomExit(ScriptMessage &) {}
    virtual void onCreatureRoomEnter(ScriptMessage &) {}
    virtual void onCreatureRoomExit(ScriptMessage &) {}

    // ── Container ──
    virtual void onContainer(ScriptMessage &) {}
    virtual void onContained(ScriptMessage &) {}

    // ── Damage ──
    virtual void onSlain(ScriptMessage &) {}
    virtual void onDamage(ScriptMessage &) {}

    // ── Animation/Sound ──
    virtual void onTweqComplete(ScriptMessage &) {}
    virtual void onMotionEnd(ScriptMessage &) {}
    virtual void onSchemaDone(ScriptMessage &) {}

    // ── Lock ──
    virtual void onNowLocked(ScriptMessage &) {}
    virtual void onNowUnlocked(ScriptMessage &) {}

    // ── Quest/Elevator ──
    virtual void onQuestChange(ScriptMessage &) {}
    virtual void onMovingTerrainWaypoint(ScriptMessage &) {}

    // ── Catch-all ──
    virtual void onMessage(ScriptMessage &) {}

    // ── Timer API ──
    // Implemented in ScriptManager.h (inline, needs ScriptManager definition)

    /// Set a one-shot timer that fires a Timer message after delaySec seconds.
    /// The timer name is delivered in msg.data, extra data in msg.data2.
    timer_handle setOneShotTimer(const std::string &name, float delaySec,
                                  Variant data = {});

    /// Cancel a pending timer.
    void killTimer(timer_handle h);

    // ── Script data API (per-object persistent key-value store) ──
    // Key is (objID, scriptClass, fieldName). Values persist across
    // sim pause/unpause. Save/load deferred to Phase 6G.

    void setData(const std::string &key, Variant value);
    Variant getData(const std::string &key) const;
    bool isDataSet(const std::string &key) const;
    void clearData(const std::string &key);

    // ── Link broadcast helper ──
    // Sends a message to all objects connected from self via the named link type.
    void broadcastOnAllLinks(const std::string &msg, const std::string &linkType,
                             Variant data = {});
};

// ============================================================================
// Script class registry — static factory pattern
// ============================================================================
//
// Script classes register via REGISTER_SCRIPT(ClassName) macro. ScriptManager
// looks up factories by name when instantiating scripts from P$Scripts data.

using ScriptFactory = ScriptBase *(*)(int32_t objID, const std::string &className,
                                       IScriptServices *svc, ScriptManager *mgr);

class ScriptRegistry {
public:
    /// Register a script factory function. Called by REGISTER_SCRIPT macro.
    static void registerClass(const std::string &name, ScriptFactory factory) {
        getMap()[name] = factory;
    }

    /// Look up a factory by script class name. Returns nullptr if not found.
    static ScriptFactory getFactory(const std::string &name) {
        auto &m = getMap();
        auto it = m.find(name);
        return (it != m.end()) ? it->second : nullptr;
    }

    /// Get all registered script class names (for diagnostics).
    static const std::unordered_map<std::string, ScriptFactory> &getAll() {
        return getMap();
    }

private:
    static std::unordered_map<std::string, ScriptFactory> &getMap() {
        static std::unordered_map<std::string, ScriptFactory> sMap;
        return sMap;
    }
};

/// Auto-registration helper. Instantiated as a static variable by REGISTER_SCRIPT.
struct ScriptRegistrar {
    ScriptRegistrar(const std::string &name, ScriptFactory factory) {
        ScriptRegistry::registerClass(name, factory);
    }
};

/// Macro to register a script class. Place in the script's .h or .cpp file.
/// The script class must have a constructor matching ScriptBase's signature.
#define REGISTER_SCRIPT(ClassName)                                              \
    static ::Darkness::ScriptBase *ClassName##_create(                          \
        int32_t objID, const std::string &className,                            \
        ::Darkness::IScriptServices *svc, ::Darkness::ScriptManager *mgr) {     \
        return new ClassName(objID, className, svc, mgr);                       \
    }                                                                           \
    static ::Darkness::ScriptRegistrar ClassName##_reg(#ClassName,              \
                                                        ClassName##_create)

} // namespace Darkness
