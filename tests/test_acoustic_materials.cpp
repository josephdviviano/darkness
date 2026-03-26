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
 *****************************************************************************/

#include <catch2/catch_test_macros.hpp>
#include "audio/AcousticMaterials.h"

using namespace Darkness;

// ════════════════════════════════════════════════════════════════════════════
// Keyword table ordering — longer keywords must come first to prevent
// false substring matches (e.g. "car" before "carpet")
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Keyword table is sorted by length descending", "[acoustic][materials]") {
    // Each keyword should be >= the length of the next keyword.
    // This ensures longer matches are tried before shorter ones.
    for (size_t i = 1; i < kAcousticKeywordCount; ++i) {
        size_t prevLen = std::strlen(kAcousticKeywords[i - 1]);
        size_t currLen = std::strlen(kAcousticKeywords[i]);
        INFO("keywords[" << (i-1) << "]=\"" << kAcousticKeywords[i-1]
             << "\" (len " << prevLen << ") should be >= keywords["
             << i << "]=\"" << kAcousticKeywords[i]
             << "\" (len " << currLen << ")");
        CHECK(prevLen >= currLen);
    }
}

// ════════════════════════════════════════════════════════════════════════════
// Core material keyword matching — verify each keyword matches itself
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("All keywords match themselves", "[acoustic][materials]") {
    for (size_t i = 0; i < kAcousticKeywordCount; ++i) {
        INFO("keyword: " << kAcousticKeywords[i]);
        CHECK(lookupAcousticMaterialKeyword(kAcousticKeywords[i])
              == kAcousticKeywords[i]);
    }
}

TEST_CASE("Case-insensitive matching", "[acoustic][materials]") {
    CHECK(lookupAcousticMaterialKeyword("STONE") == "stone");
    CHECK(lookupAcousticMaterialKeyword("Metal") == "metal");
    CHECK(lookupAcousticMaterialKeyword("BRICK") == "brick");
    CHECK(lookupAcousticMaterialKeyword("WoOd") == "wood");
}

TEST_CASE("Unrecognized texture returns generic", "[acoustic][materials]") {
    CHECK(lookupAcousticMaterialKeyword("") == "generic");
    CHECK(lookupAcousticMaterialKeyword("xyzzy") == "generic");
    CHECK(lookupAcousticMaterialKeyword("unknown_surface") == "generic");
}

// ════════════════════════════════════════════════════════════════════════════
// Thief 2 texture family names — these are the family/name paths from TXLIST
// that appear in actual Thief 2 missions. Each MUST match a non-generic
// material keyword to ensure proper acoustic behavior.
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Thief 2 brick textures match 'brick'", "[acoustic][thief2]") {
    CHECK(lookupAcousticMaterialKeyword("brick/red01") == "brick");
    CHECK(lookupAcousticMaterialKeyword("brick/grey03") == "brick");
    CHECK(lookupAcousticMaterialKeyword("brick/drk_brn1") == "brick");
    CHECK(lookupAcousticMaterialKeyword("brickwal/med01") == "brick");
}

TEST_CASE("Thief 2 stone textures match 'stone'", "[acoustic][thief2]") {
    CHECK(lookupAcousticMaterialKeyword("stone/grey01") == "stone");
    CHECK(lookupAcousticMaterialKeyword("stone/coblstn2") == "stone");
    CHECK(lookupAcousticMaterialKeyword("stone/flgston1") == "stone");
    CHECK(lookupAcousticMaterialKeyword("cutstone/arch01") == "stone");
    CHECK(lookupAcousticMaterialKeyword("oldstone/wall03") == "stone");
    CHECK(lookupAcousticMaterialKeyword("grstone/step01") == "stone");
}

TEST_CASE("Thief 2 metal textures match metal-like keywords", "[acoustic][thief2]") {
    CHECK(lookupAcousticMaterialKeyword("metal/panel01") == "metal");
    CHECK(lookupAcousticMaterialKeyword("metal/grate02") == "metal");
    CHECK(lookupAcousticMaterialKeyword("metl/beam01") == "metl");
    // "metalwal" contains both "metal" (5 chars) and "rust" — "metal" wins (longer)
    CHECK(lookupAcousticMaterialKeyword("metalwal/rust01") == "metal");
    // "rust" is also metal — both map to the same IPLMaterial
    CHECK(lookupAcousticMaterialKeyword("rust/pipe03") == "rust");
    CHECK(lookupAcousticMaterialKeyword("iron/fence01") == "iron");
}

TEST_CASE("Thief 2 wood textures match wood-like keywords", "[acoustic][thief2]") {
    // "wood/floor01" contains both "floor" (5 chars) and "wood" (4 chars) —
    // "floor" wins because it's longer. Both map to hard surface materials.
    CHECK(lookupAcousticMaterialKeyword("wood/floor01") == "floor");
    CHECK(lookupAcousticMaterialKeyword("wood/panel03") == "wood");
    CHECK(lookupAcousticMaterialKeyword("wdoor/oak01") == "door");
    // "wfloor" is a distinct 6-char keyword (wood floor) — matched before
    // generic "floor" because it's longer. Maps to wood acoustics.
    CHECK(lookupAcousticMaterialKeyword("wfloor/parq01") == "wfloor");
    CHECK(lookupAcousticMaterialKeyword("bark/tree01") == "bark");
}

TEST_CASE("Thief 2 carpet/rug textures match carpet-like", "[acoustic][thief2]") {
    CHECK(lookupAcousticMaterialKeyword("carpet/red01") == "carpet");
    CHECK(lookupAcousticMaterialKeyword("carpet/orient1") == "carpet");
    CHECK(lookupAcousticMaterialKeyword("rug/persian1") == "rug");
}

TEST_CASE("Thief 2 tile textures match tile-like keywords", "[acoustic][thief2]") {
    CHECK(lookupAcousticMaterialKeyword("tile/flr01") == "tile");
    CHECK(lookupAcousticMaterialKeyword("tile/mosaik01") == "tile");
    // "rooftile" contains both "tile" (4 chars) and "roof" (4 chars) —
    // "tile" appears first in the table at equal length, so it wins
    CHECK(lookupAcousticMaterialKeyword("rooftile/red01") == "tile");
}

TEST_CASE("Thief 2 glass textures match 'glass'", "[acoustic][thief2]") {
    CHECK(lookupAcousticMaterialKeyword("glass/window01") == "glass");
    CHECK(lookupAcousticMaterialKeyword("glass/stained2") == "glass");
}

TEST_CASE("Thief 2 dirt/earth textures match dirt-like", "[acoustic][thief2]") {
    CHECK(lookupAcousticMaterialKeyword("dirt/path01") == "dirt");
    CHECK(lookupAcousticMaterialKeyword("earth/clay01") == "earth");
    CHECK(lookupAcousticMaterialKeyword("mud/wet01") == "mud");
    CHECK(lookupAcousticMaterialKeyword("gravel/loose1") == "gravel");
}

TEST_CASE("Thief 2 outdoor/organic textures match", "[acoustic][thief2]") {
    CHECK(lookupAcousticMaterialKeyword("vine/ivy01") == "vine");
    CHECK(lookupAcousticMaterialKeyword("leaf/dead01") == "leaf");
    CHECK(lookupAcousticMaterialKeyword("hay/bale01") == "hay");
}

TEST_CASE("Thief 2 concrete/plaster textures match", "[acoustic][thief2]") {
    CHECK(lookupAcousticMaterialKeyword("concrete/wall01") == "concrete");
    CHECK(lookupAcousticMaterialKeyword("plaster/cracked1") == "plaster");
}

TEST_CASE("Thief 2 ice textures match 'ice'", "[acoustic][thief2]") {
    CHECK(lookupAcousticMaterialKeyword("vmawwin/iceclif1") == "ice");
    CHECK(lookupAcousticMaterialKeyword("ice/frozen01") == "ice");
}

TEST_CASE("Thief 2 gate/door textures match", "[acoustic][thief2]") {
    // "gate/iron01" contains both "iron" (4 chars) and "gate" (4 chars) —
    // "iron" appears first in the table at equal length, so it wins.
    // Both map to metal IPLMaterial, so the acoustic result is identical.
    CHECK(lookupAcousticMaterialKeyword("gate/iron01") == "iron");
    CHECK(lookupAcousticMaterialKeyword("door/cell01") == "door");
}

// ════════════════════════════════════════════════════════════════════════════
// Priority / false match prevention
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Carpet matches before car-like substrings", "[acoustic][materials]") {
    // "carpet" is 6 chars, checked before any hypothetical 3-char "car"
    CHECK(lookupAcousticMaterialKeyword("carpet/red01") == "carpet");
    CHECK(lookupAcousticMaterialKeyword("carpet_runner") == "carpet");
}

TEST_CASE("Concrete matches before shorter substrings", "[acoustic][materials]") {
    // "concrete" is the longest keyword — always matched first
    CHECK(lookupAcousticMaterialKeyword("concrete/wall01") == "concrete");
}

TEST_CASE("Rooftile matches tile before roof", "[acoustic][materials]") {
    // "tile" and "roof" are both 4 chars. "tile" appears first in the table
    // (sorted by insertion order at equal length), so it wins for "rooftile".
    std::string result = lookupAcousticMaterialKeyword("rooftile/red01");
    CHECK(result == "tile");
}

// ════════════════════════════════════════════════════════════════════════════
// Coverage check — known Thief 2 families that have common substring matches.
// These are "family" prefixes from TXLIST that should NOT fall back to generic.
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Common Thief 2 families do not fall back to generic", "[acoustic][thief2]") {
    // Representative families from Thief 2 missions.
    // Each should match some acoustic keyword.
    const char *families[] = {
        "brick/wall01",
        "brickwal/old01",
        "stone/grey01",
        "cutstone/arch01",
        "grstone/step01",
        "oldstone/flr01",
        "metal/panel01",
        "metl/beam01",
        "wood/floor01",
        "wdoor/oak01",
        "carpet/red01",
        "tile/flr01",
        "glass/window01",
        "dirt/path01",
        "concrete/wall01",
        "plaster/wall01",
        "iron/railing01",
        "rust/pipe01",
        "gravel/loose01",
        "hay/bale01",
        "rug/red01",
        "bark/tree01",
        "leaf/dead01",
        "ice/wall01",
        "mud/wet01",
        "vine/ivy01",
        "door/cell01",
        "gate/iron01",
        "roof/slate01",
        "earth/clay01",
        "floor/marble01",
    };

    int genericCount = 0;
    for (const char *fam : families) {
        std::string result = lookupAcousticMaterialKeyword(fam);
        INFO("family: " << fam << " -> " << result);
        if (result == "generic") {
            genericCount++;
        }
        CHECK(result != "generic");
    }

    // None of the common families should fall back to generic
    CHECK(genericCount == 0);
}
