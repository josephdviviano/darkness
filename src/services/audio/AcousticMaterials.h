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

// Acoustic material keyword matching for texture name → material type mapping.
// Maps Dark Engine TXLIST texture names (e.g., "brick/red01", "stone/grey03")
// to material keywords via substring matching.
//
// The keyword table is sorted by length descending to prevent false partial
// matches (e.g., "car" matching before "carpet"). This header is independent
// of Steam Audio (no phonon.h dependency) so it can be tested standalone.

#pragma once

#include <algorithm>
#include <string>

namespace Darkness {

/// Sorted keyword table for acoustic material matching.
/// Longer keywords come first to prevent false substring matches.
/// Each keyword maps to one of the IPLMaterial presets in AudioService.cpp
/// (kKeywordToIPLMaterial) — see that table for the keyword→material intent;
/// this array only governs match ORDER (length-descending, first hit wins).
///
/// Thief TXLIST names use abbreviated / truncated spellings that contain none
/// of the plain English words as substrings (e.g. "Clnbrik1" has "brik" not
/// "brick"; "Wdplnk" has "wdpl" not "wood"/"plank"; "Granit2" has "granit").
/// The bulk of the entries below are Dark-Engine-abbreviation aliases that
/// point at existing presets. They were derived empirically by running every
/// TXLIST name from all shipping missions through this matcher — see the
/// `darknessHeadless txlist_audit` verb. When a new mission surfaces gaps,
/// re-run that verb, add the missing abbreviation to the correct length
/// bucket here AND a matching entry in kKeywordToIPLMaterial.
inline const char *kAcousticKeywords[] = {
    // 8 chars
    "concrete", "cragston", "granwall", "robblock", "bigblock", "stuchorz",
    "stucvert", "gratmold", "blacktre", "snowcrun",
    // 7 chars
    "ceramic", "plaster", "wdpanel", "wdplain", "filston", "bstnwal", "gstnwal",
    "terston", "blackmb", "cracked", "pagwall", "forwall", "lostcty", "artdout",
    "clnbrik", "tanbrck", "walfres", "newmech", "cavelrg", "cavetex", "wintmaw",
    "clayflo",
    // 6 chars
    "carpet", "gravel", "bronze", "wfloor", "sfloor", "mfloor", "wgrain",
    "parkay", "planks", "paghut", "wframe", "wdtrim", "criswd", "sndstn",
    "brnstn", "blustn", "rufgry", "cobbel", "colwal", "stnwal", "baswal",
    "balcon", "facade", "ddetal", "detail", "blmbri", "marble", "firepl",
    "roctex", "asfalt", "cement", "stucco", "dbalco", "border", "mosaic",
    "mtrack", "mecban", "locker", "cavrgh", "foraut", "flower", "thatch",
    "hothou", "banner", "adbook",
    // 5 chars
    "brick", "glass", "stone", "metal", "grate", "plate", "floor", "earth",
    "steel", "winwd", "winwl", "plank", "rplan", "frame", "panel", "crate",
    "creat", "bigbl", "decor", "rwall", "wallc", "nkroc", "terra", "mcobb",
    "detai", "ledge", "patio", "rstuc", "dewal", "dceil", "borde", "lathe",
    "sequo", "sandy", "grass", "plant", "cfwin", "rwind", "stain", "adwin",
    "shngl", "books", "ruber", "water",
    // 4 chars
    "wood", "rock", "tile", "metl", "dirt", "rust", "iron", "pipe", "door",
    "gate", "roof", "vine", "leaf", "bark", "wdpl", "cris", "blox", "slab",
    "crag", "mstn", "grst", "sdst", "hewn", "cobl", "brik", "marb", "ston",
    "gran", "vicn", "clmn", "bcol", "col0", "rd02", "mold", "brck", "vicm",
    "stuc", "ceil", "mech", "gear", "gird", "oven", "elev", "cave", "clay",
    "snow", "sand", "awin", "owin", "dwin", "win1", "win3", "win5", "win6",
    // 3 chars
    "ice", "rug", "hay", "mud", "brd", "wd1", "stn",
};
inline const size_t kAcousticKeywordCount =
    sizeof(kAcousticKeywords) / sizeof(kAcousticKeywords[0]);

/// Look up the matching acoustic material keyword for a texture name.
/// @param texName  Texture path from TXLIST (e.g., "brick/red01", "stone/grey03")
/// @return The matched keyword (e.g., "stone"), or "generic" if no match found.
inline std::string lookupAcousticMaterialKeyword(const std::string &texName)
{
    // Convert to lowercase for case-insensitive matching
    std::string lower = texName;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

    for (size_t i = 0; i < kAcousticKeywordCount; ++i) {
        if (lower.find(kAcousticKeywords[i]) != std::string::npos) {
            return kAcousticKeywords[i];
        }
    }
    return "generic";
}

/// Coarse material CLASS for a matched keyword — the preset family it maps to
/// in kKeywordToIPLMaterial (e.g. "stn" -> "stone", "wdpl" -> "wood"). This is
/// phonon-free so the renderer can colour surfaces by acoustic class in the
/// `show_acoustic_materials` debug overlay. Keep this table in sync with
/// kAcousticKeywords / kKeywordToIPLMaterial when adding aliases — the unit
/// test "Every acoustic keyword has a material class" asserts full coverage.
inline const struct { const char *kw; const char *cls; }
    kAcousticKeywordClass[] = {
    {"concrete","concrete"}, {"ceramic","ceramic"}, {"plaster","plaster"},
    {"carpet","carpet"}, {"gravel","gravel"}, {"bronze","metal"},
    {"wfloor","wood"}, {"sfloor","stone"}, {"mfloor","metal"},
    {"brick","brick"}, {"glass","glass"}, {"stone","stone"},
    {"metal","metal"}, {"grate","metal"}, {"plate","metal"},
    {"floor","stone"}, {"earth","dirt"}, {"steel","metal"},
    {"wood","wood"}, {"rock","stone"}, {"tile","tile"}, {"metl","metal"},
    {"dirt","dirt"}, {"rust","metal"}, {"iron","metal"}, {"pipe","metal"},
    {"door","wood"}, {"gate","metal"}, {"roof","ceramic"}, {"vine","wood"},
    {"leaf","wood"}, {"bark","wood"}, {"ice","ice"}, {"rug","carpet"},
    {"hay","carpet"}, {"mud","dirt"}, {"wdpanel","wood"},
    {"wdplain","wood"}, {"wgrain","wood"}, {"parkay","wood"},
    {"planks","wood"}, {"paghut","wood"}, {"wframe","wood"},
    {"wdtrim","wood"}, {"criswd","wood"}, {"winwd","wood"},
    {"winwl","wood"}, {"wdpl","wood"}, {"plank","wood"}, {"rplan","wood"},
    {"frame","wood"}, {"panel","wood"}, {"crate","wood"}, {"creat","wood"},
    {"brd","wood"}, {"wd1","wood"}, {"sndstn","stone"},
    {"cragston","stone"}, {"filston","stone"}, {"granwall","stone"},
    {"bstnwal","stone"}, {"gstnwal","stone"}, {"robblock","stone"},
    {"terston","stone"}, {"brnstn","stone"}, {"blustn","stone"},
    {"rufgry","stone"}, {"cobbel","stone"}, {"blackmb","stone"},
    {"cracked","stone"}, {"colwal","stone"}, {"pagwall","stone"},
    {"forwall","stone"}, {"bigblock","stone"}, {"stnwal","stone"},
    {"baswal","stone"}, {"balcon","stone"}, {"facade","stone"},
    {"ddetal","stone"}, {"detail","stone"}, {"blmbri","stone"},
    {"marble","stone"}, {"bigbl","stone"}, {"lostcty","stone"},
    {"artdout","stone"}, {"decor","stone"}, {"rwall","stone"},
    {"wallc","stone"}, {"firepl","stone"}, {"nkroc","stone"},
    {"roctex","stone"}, {"terra","stone"}, {"cris","stone"},
    {"blox","stone"}, {"slab","stone"}, {"crag","stone"}, {"mstn","stone"},
    {"grst","stone"}, {"sdst","stone"}, {"hewn","stone"}, {"cobl","stone"},
    {"mcobb","stone"}, {"brik","brick"}, {"marb","stone"},
    {"ston","stone"}, {"gran","stone"}, {"vicn","stone"}, {"clmn","stone"},
    {"bcol","stone"}, {"col0","stone"}, {"rd02","stone"},
    {"detai","stone"}, {"mold","stone"}, {"ledge","stone"},
    {"patio","stone"}, {"stn","stone"}, {"clnbrik","brick"},
    {"tanbrck","brick"}, {"brck","brick"}, {"asfalt","concrete"},
    {"cement","concrete"}, {"stuchorz","plaster"}, {"stucvert","plaster"},
    {"stucco","plaster"}, {"walfres","plaster"}, {"dbalco","plaster"},
    {"border","plaster"}, {"rstuc","plaster"}, {"dewal","plaster"},
    {"dceil","plaster"}, {"borde","plaster"}, {"vicm","plaster"},
    {"stuc","plaster"}, {"ceil","plaster"}, {"mosaic","tile"},
    {"gratmold","metal"}, {"newmech","metal"}, {"mtrack","metal"},
    {"mecban","metal"}, {"locker","metal"}, {"lathe","metal"},
    {"mech","metal"}, {"gear","metal"}, {"gird","metal"}, {"oven","metal"},
    {"elev","metal"}, {"cavelrg","rock"}, {"cavetex","rock"},
    {"cavrgh","rock"}, {"cave","rock"}, {"blacktre","bark"},
    {"sequo","bark"}, {"snowcrun","dirt"}, {"wintmaw","dirt"},
    {"clayflo","dirt"}, {"sandy","dirt"}, {"grass","dirt"},
    {"clay","dirt"}, {"snow","dirt"}, {"sand","dirt"}, {"foraut","leaf"},
    {"flower","leaf"}, {"plant","leaf"}, {"thatch","hay"},
    {"hothou","glass"}, {"cfwin","glass"}, {"rwind","glass"},
    {"stain","glass"}, {"adwin","glass"}, {"awin","glass"},
    {"owin","glass"}, {"dwin","glass"}, {"win1","glass"}, {"win3","glass"},
    {"win5","glass"}, {"win6","glass"}, {"shngl","ceramic"},
    {"banner","carpet"}, {"books","carpet"}, {"adbook","carpet"},
    {"ruber","carpet"}, {"water","ceramic"},
};
inline const size_t kAcousticKeywordClassCount =
    sizeof(kAcousticKeywordClass) / sizeof(kAcousticKeywordClass[0]);

/// Map a matched keyword to its coarse material class (see above).
/// @param keyword  A keyword returned by lookupAcousticMaterialKeyword().
/// @return class name (e.g. "stone", "wood", "metal"), or "generic" if the
///         keyword is unknown / unmapped.
inline std::string acousticMaterialClass(const std::string &keyword)
{
    for (size_t i = 0; i < kAcousticKeywordClassCount; ++i) {
        if (keyword == kAcousticKeywordClass[i].kw)
            return kAcousticKeywordClass[i].cls;
    }
    return "generic";
}

} // namespace Darkness
