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

// StdElevator.h — Standard elevator/moving terrain script (Task 70)
//
// Reimplements the Dark Engine's StdElevator: height-based waypoint seeking
// for platforms. TurnOn = seek highest waypoint, TurnOff = seek lowest.
// Manages TPathNext links dynamically to control MovingTerrainSystem.

#pragma once

#include "sim/ScriptBase.h"
#include "sim/ScriptManager.h"
#include "sim/ScriptServices.h"
#include "sim/IScriptServices.h"

namespace Darkness {

class StdElevator : public ScriptBase {
public:
    StdElevator(int32_t objID, const std::string &className,
                IScriptServices *services, ScriptManager *manager)
        : ScriptBase(objID, className, services, manager) {}

protected:
    // ── Core waypoint navigation ──

    /// Find the highest or lowest waypoint along the TPath chain.
    /// Uses Floyd's cycle detection (scout pointer) to handle loops.
    int32_t findPoint(bool highest) {
        if (!svc || !svc->link || !svc->object) return self;

        int transform = highest ? 1 : -1;

        // Find starting waypoint from TPathNext or TPathInit
        int32_t curpoint = 0;
        auto nextLink = svc->link->getOne("TPathNext", self);
        if (nextLink.dst != 0) {
            curpoint = nextLink.dst;
        } else {
            auto initLink = svc->link->getOne("TPathInit", self);
            if (initLink.dst != 0) {
                curpoint = initLink.dst;
            } else {
                std::fprintf(stderr, "[StdElevator] obj %d: no waypoint path found\n", self);
                return self;
            }
        }

        // Traverse TPath links to find highest/lowest point
        float maxHeight = transform * svc->object->position(curpoint).z;
        int32_t bestPoint = curpoint;
        int32_t scoutpt = curpoint;

        do {
            float h = transform * svc->object->position(curpoint).z;
            if (h > maxHeight) {
                maxHeight = h;
                bestPoint = curpoint;
            }

            curpoint = nextPoint(curpoint, "TPath");
            scoutpt = nextPoint(nextPoint(scoutpt, "TPath"), "TPath");
        } while (curpoint != scoutpt && curpoint != 0);

        return bestPoint;
    }

    /// Get next waypoint along a link type.
    int32_t nextPoint(int32_t curpt, const std::string &linkType) {
        if (!svc || !svc->link) return curpt;
        auto link = svc->link->getOne(linkType, curpt);
        return (link.dst != 0) ? link.dst : curpt;
    }

    /// Seek to a specific waypoint by managing TPathNext links.
    void seekPoint(int32_t dest) {
        if (dest == self || !svc || !svc->link) return;

        // Check if we're already heading to this destination
        auto existing = svc->link->getOne("TPathNext", self);
        if (existing.dst == dest) {
            // Already going there — just ensure we're active
            if (svc->movingTerrainSystem)
                svc->movingTerrainSystem->activate(self);
            return;
        }

        // Destroy existing TPathNext link
        if (existing.id != 0) {
            if (svc->movingTerrainSystem)
                svc->movingTerrainSystem->deactivate(self);
            svc->link->destroy(existing.id);
        }

        // Create new TPathNext link to destination
        svc->link->create("TPathNext", self, dest);

        // Activate movement
        if (svc->movingTerrainSystem)
            svc->movingTerrainSystem->activate(self);

        // Check for short trips (< 0.5 units) — skip sound
        Vector3 myPos = svc->object->position(self);
        Vector3 destPos = svc->object->position(dest);
        float dist = glm::length(destPos - myPos);

        if (dist > 0.5f) {
            // Notify self that we're starting (for sound scripts)
            if (mgr) {
                ScriptMessage startMsg;
                startMsg.to = self;
                startMsg.name = "Starting";
                startMsg.from = self;
                mgr->sendMessage(startMsg);
            }
        }

        if (svc->darkGame)
            svc->darkGame->foundObject(self);
    }

    /// Activate elevator: TurnOn = seek up, TurnOff = seek down.
    void activate(bool on) {
        // on=true → seek highest (TurnOn means go up)
        // on=false → seek lowest (TurnOff means go down)
        seekPoint(findPoint(on));
    }

    // ── Message handlers ──

    void onTurnOn(ScriptMessage &msg) override {
        activateWithFlags(true);
    }

    void onTurnOff(ScriptMessage &msg) override {
        activateWithFlags(false);
    }

    void onMovingTerrainWaypoint(ScriptMessage &msg) override {
        // Arrived at waypoint — broadcast on ControlDevice links
        broadcastOnAllLinks("TurnOn", "ControlDevice");

        // If at destination, deactivate
        auto dest = svc->link->getOne("TPathNext", self);
        if (dest.dst != 0) {
            Vector3 myPos = svc->object->position(self);
            Vector3 destPos = svc->object->position(dest.dst);
            float dist = glm::length(destPos - myPos);
            if (dist < 0.5f) {
                // Arrived — stop
                if (svc->movingTerrainSystem)
                    svc->movingTerrainSystem->deactivate(self);
            }
        }
    }

    void onMessage(ScriptMessage &msg) override {
        if (msg.name == "Call") {
            // Call elevator to sender's waypoint
            seekPoint(msg.from);
        }
    }

private:
    /// Activate with TrapFlags filtering.
    void activateWithFlags(bool on) {
        uint32_t trapFlags = 0;
        if (svc && svc->property) {
            PropTrapFlags tf{};
            if (svc->property->getTyped(self, "TrapFlags", tf))
                trapFlags = tf.flags;
        }

        if (on && (trapFlags & 0x04))  return;  // TRAPF_NOON
        if (!on && (trapFlags & 0x08)) return;  // TRAPF_NOOFF
        if (trapFlags & 0x02) on = !on;          // TRAPF_INVERT

        activate(on);
    }
};

REGISTER_SCRIPT(StdElevator);

} // namespace Darkness
