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
/// Each keyword maps to one of the IPLMaterial presets in AudioService.cpp.
inline const char *kAcousticKeywords[] = {
    // 8+ chars
    "concrete",
    "ceramic",
    "plaster",
    "carpet",
    "gravel",
    "bronze",    // bronze metal fixtures
    // 5 chars
    "brick",
    "glass",
    "stone",
    "metal",
    "grate",     // metal grating (city/Gratesqr, iroGrate, etc.)
    "plate",     // metal plates (city/Plate, etc.)
    "floor",     // many Thief 2 families like "wfloor", "sfloor" are wood/stone
    "earth",
    "steel",     // steel surfaces
    // 4 chars
    "wood",
    "rock",
    "tile",
    "metl",      // Thief 2 abbreviation for metal (e.g., "metl/beam01")
    "dirt",
    "rust",      // rusty metal surfaces
    "iron",      // iron metal surfaces
    "pipe",      // metal piping
    "door",      // wooden doors (wdoor, etc.)
    "gate",      // metal gates
    "roof",      // rooftop surfaces (tile/stone)
    "vine",      // organic (plant matter — use wood properties)
    "leaf",      // organic (plant matter — use wood properties)
    "bark",      // tree bark (wood)
    // 3 chars
    "ice",
    "rug",       // rug surfaces (carpet-like)
    "hay",       // organic (straw — high absorption like carpet)
    "mud",       // wet dirt
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

} // namespace Darkness
