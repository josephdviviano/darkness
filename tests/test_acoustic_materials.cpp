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
    // "wood/panel03" contains "panel" (5 chars) and "wood" (4 chars) — "panel"
    // wins (longer) and also maps to the wood preset, so the acoustic result is
    // identical. ("panel" was added to cover artdeco/Panel* wood panelling.)
    CHECK(lookupAcousticMaterialKeyword("wood/panel03") == "panel");
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

// ════════════════════════════════════════════════════════════════════════════
// Dark-Engine-abbreviation regression set. These are REAL family/name strings
// pulled from the TXLIST chunks of all shipping Thief 2 missions (via the
// `darknessHeadless txlist_audit` verb). They use Thief's truncated spellings
// (no plain English word as a substring) and previously fell through to the
// opaque-reflective fallback. Every one must now resolve to a keyword.
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Thief 2 abbreviated spellings resolve (not generic)", "[acoustic][thief2]") {
    const char *names[] = {
        // wood: planks / panels / frames / crates / boards
        "core_2/Wdplnk", "core_2/Wdplanks", "core_2/Parkay64", "core_2/Planks1",
        "artdeco/Panel07", "core_3/wd1128", "core_2/CRATE1", "core_3/creat6",
        "ancient/BRD_LADL", "ruined/RPLAN04", "vicm09/wdtrim", "vicm01/Frame1",
        "city/Winwd64", "vmaw2/paghut", "rescor_1/wgrain01",
        // stone / masonry / marble / columns
        "core_1/Granit2", "core_1/Cobls1", "core_1/Blustn", "core_1/BIGBL2",
        "core_1/Hewn3", "core_1/Rufgry", "core_2/Bstnwal1", "core_2/Marble",
        "core_2/Slab64", "core_3/grst164", "core_3/mstn1128", "artdout/BlackMB",
        "civicb2/abaswal4", "civicb2/balcon02", "civicb2/Mold07", "civicbui/col01",
        "civicbui/vicn92", "lostcty/LC07", "lostcty/WAL01", "NewKeep/nkroc03",
        "Ramirez/bigblock", "vmaw/forwall1", "vmaw/terston2", "ruined/RWALL04",
        "rescor_2/mcobb01", "rescor_1/firepl01", "ancient/SNDSTN01", "ancient/decor02a",
        // brick
        "core_1/Clnbrik1", "core_1/Blubrik1", "core_2/Tanbrck",
        // concrete
        "core_1/ASFALT", "core_1/Cement",
        // plaster / stucco / ceilings / deco walls
        "core_1/Stuc2", "core_2/Stucco", "ancient/STUCHORZ", "ancient/WALFRESA",
        "artdeco/Dewal03b", "artdeco/Dceil04", "ceilpain/Pceil01", "vicm09/wall03",
        "vicm09/VIC01M", "vicm09/m09ceil1", "mine/stucco",
        // tile / mosaic
        "core_2/Mosaic1",
        // metal: mech / mine / fixtures
        "mech/DWAL2", "mech/GEAR2S", "mech/GRATMOLD", "newmech/plpanel",
        "mine/mtrack01", "mine/elev2btn", "core_3/locker1", "Tower2/gird",
        "rescor_1/oven2",
        // rock / cave
        "cave/CAVELRG1", "core_1/Cavrgh5",
        // bark / trees
        "vmaw/sequo1", "vmawtb/Blacktre",
        // dirt / sand / clay / snow / grass
        "core_1/Sandy", "core_2/Sand", "vmaw2/clayflo", "vmaw/grassj1",
        "vmawwin/snowcrun", "vmawwin/wintmaw1",
        // organic foliage
        "rescor_1/plant04", "rescor_2/flower04", "vmaw/foraut1", "vmaw2/thatch",
        // glass / windows
        "rescor_1/hothou1", "rescor_2/stain04", "artdmisc/ADWin01", "civicb2/Awin08",
        "civicbui/owin02", "civicb2/Dwin13", "city/win1", "city/Cfwinf01",
        "ruined/RWIND06",
        // ceramic (roof shingle / water)
        "city/Blushngl", "waterhw/blin",
        // fabric / banner / books / mats
        "rescor_1/banner03", "rescor_1/books01a", "city/rubermat",
    };

    int genericCount = 0;
    for (const char *n : names) {
        std::string result = lookupAcousticMaterialKeyword(n);
        INFO("name: " << n << " -> " << result);
        if (result == "generic") genericCount++;
        CHECK(result != "generic");
    }
    CHECK(genericCount == 0);
}

TEST_CASE("Every acoustic keyword has a material class", "[acoustic][materials]") {
    // The renderer's show_acoustic_materials debug overlay colours surfaces by
    // material class. Every keyword in the matcher table must have a class
    // entry, or a mapped surface would render as "generic" (magenta) in the
    // overlay despite actually being mapped — a false alarm.
    for (size_t i = 0; i < kAcousticKeywordCount; ++i) {
        std::string cls = acousticMaterialClass(kAcousticKeywords[i]);
        INFO("keyword: " << kAcousticKeywords[i] << " -> class " << cls);
        CHECK(cls != "generic");
    }
    // Spot-check a few class resolutions.
    CHECK(acousticMaterialClass("stn")  == "stone");
    CHECK(acousticMaterialClass("wdpl") == "wood");
    CHECK(acousticMaterialClass("mech") == "metal");
    CHECK(acousticMaterialClass("vicm") == "plaster");
    CHECK(acousticMaterialClass("nope_no_such_keyword") == "generic");
}

TEST_CASE("Thief 2 abbreviation keyword resolution (pins tricky cases)",
          "[acoustic][thief2]") {
    // Wood-plank family: the 4-char "wdpl" alias covers every Wdplnk* variant.
    CHECK(lookupAcousticMaterialKeyword("core_2/Wdplnk")   == "wdpl");
    CHECK(lookupAcousticMaterialKeyword("core_2/Wdplkbnd") == "wdpl");
    // Brick: full "clnbrik" beats the generic "brik" (both → brick preset).
    CHECK(lookupAcousticMaterialKeyword("core_1/Clnbrik1") == "clnbrik");
    CHECK(lookupAcousticMaterialKeyword("core_1/Blubrik1") == "brik");
    // Masonry / stone abbreviations.
    CHECK(lookupAcousticMaterialKeyword("core_1/Granit2")  == "gran");
    CHECK(lookupAcousticMaterialKeyword("core_1/ASFALT")   == "asfalt");
    CHECK(lookupAcousticMaterialKeyword("core_1/Stuc2")    == "stuc");
    CHECK(lookupAcousticMaterialKeyword("core_1/Sandy")    == "sandy");
    CHECK(lookupAcousticMaterialKeyword("core_2/Sand")     == "sand");
    // Mech grating molding must stay metal (NOT the stone "mold" alias).
    CHECK(lookupAcousticMaterialKeyword("mech/GRATMOLD")   == "gratmold");
    // New-mech panels must stay metal (NOT the wood "panel" alias).
    CHECK(lookupAcousticMaterialKeyword("newmech/plpanel") == "newmech");
    // Window-wood shutters are wood, not glass (city/Winwd*).
    CHECK(lookupAcousticMaterialKeyword("city/Winwd64")    == "winwd");
    // Victorian-mansion family default → plaster; civic vicn → stone. Both
    // contain "vic" but only the mansion names contain "vicm".
    CHECK(lookupAcousticMaterialKeyword("vicm09/VIC01M")   == "vicm");
    CHECK(lookupAcousticMaterialKeyword("civicbui/vicn92") == "vicn");
}
