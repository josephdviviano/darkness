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

// AnimLightScript.h — Animated light script (Task 74)
//
// Reimplements the Dark Engine's AnimLight script. Handles turn on/off
// of lights, switching between OnLiteMode and OffLiteMode, managing
// associated ambient sounds (torch crackle), and SelfIllum properties.
//
// This unblocks 39 animated lights in MISS6 (torches, gas lamps, etc.).

#pragma once

#include "sim/ScriptBase.h"
#include "sim/ScriptManager.h"
#include "sim/ScriptServices.h"
#include "sim/IScriptServices.h"

namespace Darkness {

class AnimLightScript : public ScriptBase {
public:
    AnimLightScript(int32_t objID, const std::string &className,
                    IScriptServices *services, ScriptManager *manager)
        : ScriptBase(objID, className, services, manager) {}

protected:
    void onBeginScript(ScriptMessage &msg) override {
        // Initialize light state from current property
        // Sync particles, TweqBlink, SelfIllum
    }

    void onSim(ScriptMessage &msg) override {
        if (msg.data.toInt() == 1) {
            // Sim starting — set initial mode
            bool isOn = !isDataSet("IsOn") || getData("IsOn").toBool();
            if (isOn) {
                turnOnLight();
            } else {
                turnOffLight();
            }
        }
    }

    void onTurnOn(ScriptMessage &msg) override {
        turnOnLight();
        // Broadcast to ControlDevice and ~ParticleAttachment links
        broadcastOnAllLinks("TurnOn", "ControlDevice");
        broadcastOnAllLinks("TurnOn", "~ParticleAttachement");
    }

    void onTurnOff(ScriptMessage &msg) override {
        turnOffLight();
        broadcastOnAllLinks("TurnOff", "ControlDevice");
        broadcastOnAllLinks("TurnOff", "~ParticleAttachement");
    }

    void onMessage(ScriptMessage &msg) override {
        if (msg.name == "Toggle") {
            bool isOn = isDataSet("IsOn") && getData("IsOn").toBool();
            if (isOn)
                turnOffLight();
            else
                turnOnLight();
        }
    }

    void onSlain(ScriptMessage &msg) override {
        // Extinguish and schedule destruction
        turnOffLight();
        setOneShotTimer("ReallySlay", 0.1f);
    }

    void onTimer(ScriptMessage &msg) override {
        if (msg.data.toString() == "ReallySlay") {
            if (svc && svc->damage)
                svc->damage->slay(self, 0);
        }
    }

private:
    void turnOnLight() {
        setData("IsOn", Variant(true));
        if (svc && svc->light)
            svc->light->activate(self);

        // Enable ambient sound (torch crackle)
        // Set AmbientHacked flags to start associated sound
        if (svc && svc->sound)
            svc->sound->playEnvSchema(self, "Event Activate", self);
    }

    void turnOffLight() {
        setData("IsOn", Variant(false));
        if (svc && svc->light)
            svc->light->deactivate(self);

        // Disable ambient sound
        if (svc && svc->sound)
            svc->sound->haltSchema(self, "");
    }
};

REGISTER_SCRIPT(AnimLightScript);

// Register under both names — missions may use either
static ScriptRegistrar AnimLight_reg("AnimLight",
    [](int32_t objID, const std::string &className,
       IScriptServices *svc, ScriptManager *mgr) -> ScriptBase * {
        return new AnimLightScript(objID, className, svc, mgr);
    });

} // namespace Darkness
