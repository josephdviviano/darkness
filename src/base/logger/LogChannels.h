/******************************************************************************
 *
 *    This file is part of the darkness project
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

// Per-subsystem stderr log channels. Diagnostic prints are tagged
// ("[ODE] ...", "[TWEQ] ...") and chatty RUNTIME tags consult this
// registry before printing, so a subsystem's telemetry can be muted
// without rebuilding:
//
//     if (Darkness::logTagEnabled("ODE")) std::fprintf(stderr, ...);
//
// Config: `dev: log_mute: "ODE,TWEQ"` (comma-separated tag list).
// Console: `log_mute` lists state, `log_mute <TAG>` toggles one live.
//
// Scope rules: only tags measured as runtime-recurring get gated (the
// census lives in tools' noise runs — gate by measurement, not
// speculation). One-shot load dumps stay ungated, and [FALLBACK] must
// NEVER route through this registry — a fallback that can be silenced
// by default is a silent fallback.

#pragma once

#include <string>
#include <unordered_set>

namespace Darkness {

inline std::unordered_set<std::string> &logMutedTags() {
    static std::unordered_set<std::string> s;
    return s;
}

inline bool logTagEnabled(const char *tag) {
    return logMutedTags().count(tag) == 0;
}

inline void setLogTagEnabled(const char *tag, bool enabled) {
    if (enabled)
        logMutedTags().erase(tag);
    else
        logMutedTags().insert(tag);
}

// "ODE,TWEQ, PATH_RAW" -> mute set (whitespace-tolerant, replaces the
// current set so the config is authoritative at startup).
inline void applyLogMuteList(const std::string &csv) {
    logMutedTags().clear();
    std::string cur;
    for (char c : csv) {
        if (c == ',') {
            if (!cur.empty()) logMutedTags().insert(cur);
            cur.clear();
        } else if (c != ' ' && c != '\t') {
            cur.push_back(c);
        }
    }
    if (!cur.empty()) logMutedTags().insert(cur);
}

inline std::string logMuteListString() {
    std::string out;
    for (const auto &t : logMutedTags()) {
        if (!out.empty()) out += ",";
        out += t;
    }
    return out.empty() ? "(none)" : out;
}

} // namespace Darkness
