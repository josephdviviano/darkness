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

// IScriptServices.h — Aggregate struct holding service interfaces for scripts
//
// Passed to ScriptBase on construction. Scripts access engine services as
// svc->link, svc->object, svc->property, etc. Services are thin wrappers
// around existing OPDE services (no functionality rebuilt).
//
// This is the equivalent of the original engine's ScriptServices aggregate
// that provided ILinkSrv, IObjectSrv, IPropertySrv, ISoundSrv, etc.
//
// Services are added incrementally as Task 67 progresses. The struct is
// append-only for ABI stability with future external script modules.

#pragma once

#include <cstdint>
#include <string>

namespace Darkness {

// Forward declarations for services (defined in Task 67)
class LinkService;
class ObjectService;
class PropertyService;
class DoorSystem;
class TweqSystem;
class MovingTerrainSystem;
class MessageDispatch;

// Forward declarations for future script service wrappers (Task 67)
struct LinkScriptService;
struct ObjectScriptService;
struct PropertyScriptService;
struct SoundScriptService;
struct LightScriptService;
struct DamageScriptService;
struct DataScriptService;
struct LockedScriptService;
struct PhysicsScriptService;
struct DoorScriptService;

// ============================================================================
// IScriptServices — aggregate of all services available to scripts
// ============================================================================
//
// Scripts access services through this aggregate. Initially holds raw OPDE
// service pointers; as Task 67 adds thin wrapper services, those replace
// the raw pointers. Fields are append-only for external module ABI stability.

struct IScriptServices {
    // ── Raw OPDE service access (available immediately) ──
    PropertyService *propertyService = nullptr;
    LinkService *linkService = nullptr;
    ObjectService *objectService = nullptr;

    // ── Sim systems ──
    DoorSystem *doorSystem = nullptr;
    TweqSystem *tweqSystem = nullptr;
    MovingTerrainSystem *movingTerrainSystem = nullptr;
    MessageDispatch *messageDispatch = nullptr;

    // ── Script service wrappers (added by Task 67, nullptr until then) ──
    LinkScriptService *link = nullptr;
    ObjectScriptService *object = nullptr;
    PropertyScriptService *property = nullptr;
    SoundScriptService *sound = nullptr;
    LightScriptService *light = nullptr;
    DamageScriptService *damage = nullptr;
    DataScriptService *data = nullptr;
    LockedScriptService *locked = nullptr;
    PhysicsScriptService *physics = nullptr;
    DoorScriptService *door = nullptr;
};

} // namespace Darkness
