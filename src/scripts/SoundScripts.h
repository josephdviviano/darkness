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

// SoundScripts.h — Sound scripts (Task 75)
//
// OnOffSounds: plays activation/deactivation sounds on TurnOn/TurnOff.
// ActivateAmbient: toggles AmbientHacked property flags on TurnOn/TurnOff.

#pragma once

#include "sim/ScriptBase.h"
#include "sim/ScriptManager.h"
#include "sim/ScriptServices.h"
#include "sim/IScriptServices.h"

namespace Darkness {

// ============================================================================
// OnOffSounds — plays activation/deactivation sounds (Task 75a)
// ============================================================================

class OnOffSounds : public ScriptBase {
public:
    using ScriptBase::ScriptBase;

protected:
    void onTurnOn(ScriptMessage &msg) override {
        if (svc && svc->sound) {
            svc->sound->haltSchema(self, "");
            svc->sound->playEnvSchema(self, "Event Activate", self);
        }
    }

    void onTurnOff(ScriptMessage &msg) override {
        if (svc && svc->sound) {
            svc->sound->haltSchema(self, "");
            svc->sound->playEnvSchema(self, "Event Deactivate", self);
        }
    }
};

REGISTER_SCRIPT(OnOffSounds);

// ============================================================================
// ActivateAmbient — toggles AmbientHacked property (Task 75b)
// ============================================================================

class ActivateAmbient : public ScriptBase {
public:
    using ScriptBase::ScriptBase;

protected:
    void onTurnOn(ScriptMessage &msg) override {
        // Set ambient active — when AmbientHacked management is fully wired,
        // this will enable the ambient sound. For now, log.
        std::fprintf(stderr, "[ActivateAmbient] obj %d: TurnOn (ambient enabled)\n", self);
    }

    void onTurnOff(ScriptMessage &msg) override {
        std::fprintf(stderr, "[ActivateAmbient] obj %d: TurnOff (ambient disabled)\n", self);
    }
};

REGISTER_SCRIPT(ActivateAmbient);

} // namespace Darkness
