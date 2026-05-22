/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2005-2009 openDarkEngine team
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

// Animated lighting system — parses P$AnimLight properties, drives animation
// state machines per Dark Engine light modes, and provides intensity values
// for lightmap atlas blending.
//
// Architecture:
//   - LightSource: per-light animation params + runtime state
//   - parseAnimLightProperties(): reads P$AnimLight + P$Position from .mis
//   - updateLightAnimation(): per-frame state machine update
//   - buildAnimLightIndex(): builds lightnum → [(cell,poly)] reverse map
//
// Future extension points:
//   - Dynamic point lights (position + radius already stored)
//   - GPU compute blend path (replace CPU loop)
//   - Volumetric fog interaction (query lights by position/radius)

#pragma once

#include "WRChunkParser.h"
#include "property/PropertyService.h"
#include "property/DarkPropertyDefs.h"
#include "property/TypedProperty.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <unordered_map>
#include <vector>

namespace Darkness {

// ── Animation modes (from animlightmode enum in t2-types.dtype) ──

enum AnimLightMode : uint16_t {
    ANIM_FLIP          = 0,  // Instant min↔max toggle
    ANIM_SMOOTH        = 1,  // Linear fade, direction reverses at limits
    ANIM_RANDOM        = 2,  // Random brightness each cycle
    ANIM_MIN_BRIGHT    = 3,  // Static at minimum brightness
    ANIM_MAX_BRIGHT    = 4,  // Static at maximum brightness (default)
    ANIM_ZERO          = 5,  // Static at zero brightness (off)
    ANIM_BRIGHTEN      = 6,  // One-shot linear fade to max, then stop
    ANIM_DIM           = 7,  // One-shot linear fade to min, then stop
    ANIM_SEMI_RANDOM   = 8,  // Bounded random walk (smooth random)
    ANIM_FLICKER       = 9,  // Rapid random toggle with timing variance
};

// ── Light source data ──
//
// Stores both animation parameters and spatial data. Spatial data (position,
// radius) is not used for the baked overlay blend, but is stored here for
// future dynamic lighting / volumetric fog queries.

struct LightSource {
    int16_t lightNum;          // matches animMap entries in WR cells
    int32_t objectId;          // Dark Engine object ID (from property)

    // Spatial data (for future dynamic lighting / volumetric fog)
    float posX, posY, posZ;    // world position (from P$Position + offset)
    float radius;              // light radius
    float innerRadius;         // inner radius for falloff

    // Animation parameters (from P$AnimLight property)
    uint16_t mode;             // animlightmode enum (0-9)
    float brightenTime;        // seconds to reach max
    float dimTime;             // seconds to reach min
    float minBright;           // minimum brightness value
    float maxBright;           // maximum brightness value

    // Runtime state
    float brightness;          // current brightness value
    float countdown;           // seconds remaining in current phase
    bool isRising;             // true = brightening, false = dimming
    bool inactive;             // true = light is inactive/paused
    float prevIntensity;       // for dirty detection (0.0-1.0)

    // Computed at parse time per the original engine's InitModes() logic
    // (see GEN.SCR AnimLight). Scripts that turn the light on or off pick
    // the per-light on/off mode here — never the hardcoded MAX/ZERO — so
    // FLICKER, SMOOTH, RANDOM authoring survives a turn-on round-trip.
    uint16_t onLiteMode;
    uint16_t offLiteMode;
};

// Compute per-light on/off modes from the authored mode. Direct port of
// the original engine's AnimLight script InitModes() semantics:
//   mode == MIN          → off=MIN,        on=MAX            (alarm-style)
//   mode == BRIGHTEN/DIM → off=DIM,        on=BRIGHTEN       (one-shot fade pair)
//   mode == anything     → off=ZERO,       on=mode           (preserve FLICKER, etc.)
inline void computeLiteModes(LightSource &ls) {
    uint16_t mode = ls.mode;
    uint16_t offmode, onmode;

    if (mode == ANIM_MIN_BRIGHT) {
        offmode = ANIM_MIN_BRIGHT;
    } else if (mode == ANIM_BRIGHTEN || mode == ANIM_DIM) {
        offmode = ANIM_DIM;
    } else {
        offmode = ANIM_ZERO;
    }

    if (mode != offmode) {
        // Authored mode is the on-mode — this is the path that preserves
        // FLICKER, RANDOM, SEMI_RANDOM, MAX, FLIP, SMOOTH as the activated
        // animation. Anything that's not MIN/BRIGHTEN/DIM lands here.
        onmode = mode;
    } else {
        // mode == offmode means the authored mode is intrinsically "off"
        // (MIN or DIM). Fall back to the canonical opposite.
        if (offmode == ANIM_DIM)
            onmode = ANIM_BRIGHTEN;
        else  // ANIM_MIN_BRIGHT
            onmode = ANIM_MAX_BRIGHT;
    }

    ls.onLiteMode = onmode;
    ls.offLiteMode = offmode;
}

// Switch a light to a new mode and prime brightness/countdown so the
// updateLightAnimation tick picks up cleanly. Called by LightScriptService
// for runtime on/off transitions (and shared with the parser's initial
// brightness logic). Centralized so the mode → brightness mapping has one
// source of truth.
inline void setLiteMode(LightSource &ls, uint16_t newMode) {
    ls.mode = newMode;
    switch (newMode) {
        case ANIM_MIN_BRIGHT:
            ls.brightness = ls.minBright;
            break;
        case ANIM_ZERO:
            ls.brightness = 0.0f;
            break;
        case ANIM_MAX_BRIGHT:
            ls.brightness = ls.maxBright;
            break;
        case ANIM_BRIGHTEN:
            // One-shot fade up — start at min so the animation has range to run.
            ls.brightness = ls.minBright;
            ls.isRising = true;
            ls.countdown = ls.brightenTime;
            break;
        case ANIM_DIM:
            // One-shot fade down — start at max.
            ls.brightness = ls.maxBright;
            ls.isRising = false;
            ls.countdown = ls.dimTime;
            break;
        default:
            // Animating modes (FLIP, SMOOTH, RANDOM, SEMI_RANDOM, FLICKER):
            // start at max; updateLightAnimation drives subsequent values.
            ls.brightness = ls.maxBright;
            break;
    }
}

// ── AnimLight property parser ──
//
// Iterates objects that own P$AnimLight via PropertyService and builds a
// LightSource per record. Reads P$Position similarly for world position
// (light position = object position + AnimLight.offset).
//
// Layout details live in PropAnimLight (DarkPropertyDefs.h). Both this
// parser and DarknessHeadless's prop-dump memcpy from the same raw bytes
// via getTypedProperty<PropAnimLight>, so the two views cannot drift.

inline std::unordered_map<int16_t, LightSource>
parseAnimLightProperties(PropertyService *propSvc)
{
    std::unordered_map<int16_t, LightSource> lights;
    if (!propSvc) {
        std::fprintf(stderr, "[FALLBACK] parseAnimLightProperties: propSvc=null\n");
        return lights;
    }

    auto objIDs = getAllObjectsWithProperty(propSvc, "AnimLight");
    int parsed = 0;
    for (int objID : objIDs) {
        PropAnimLight prop{};
        if (!getTypedProperty<PropAnimLight>(propSvc, "AnimLight", objID, prop))
            continue;

        // Archetype records use lightNum=0 as the unauthored sentinel.
        // Skip them — concrete instances re-author their own lightNum.
        // (lightNum<0 is the original "invalid" sentinel from the engine.)
        if (prop.lightNum <= 0) continue;

        LightSource ls{};
        ls.lightNum = prop.lightNum;
        ls.objectId = objID;

        // World position = P$Position + AnimLight.offset. PropertyService
        // resolves archetype inheritance for Position automatically.
        PropPosition pos{};
        if (getTypedProperty<PropPosition>(propSvc, "Position", objID, pos)) {
            ls.posX = pos.x + prop.offsetX;
            ls.posY = pos.y + prop.offsetY;
            ls.posZ = pos.z + prop.offsetZ;
        } else {
            ls.posX = prop.offsetX;
            ls.posY = prop.offsetY;
            ls.posZ = prop.offsetZ;
            std::fprintf(stderr,
                "[FALLBACK] LightingSystem: light %d (obj %d) has no Position, "
                "using AnimLight.offset alone (%.1f,%.1f,%.1f)\n",
                (int)prop.lightNum, objID, prop.offsetX, prop.offsetY, prop.offsetZ);
        }

        ls.radius = prop.radius;
        ls.innerRadius = prop.innerRadius;
        ls.mode = prop.mode;
        ls.brightenTime = prop.brightenTime / 1000.0f; // ms → seconds
        ls.dimTime = prop.dimTime / 1000.0f;
        ls.minBright = prop.minBrightness;
        ls.maxBright = prop.maxBrightness;
        ls.isRising = (prop.rising != 0);
        ls.inactive = (prop.inactive != 0);
        ls.countdown = prop.countdown / 1000.0f;

        // Initialize brightness based on mode and state. Kept here (not via
        // setLiteMode) because the parser must respect the on-disk `rising`
        // bit for BRIGHTEN/DIM/SMOOTH state restoration — runtime activate()
        // chooses its own direction.
        switch (prop.mode) {
            case ANIM_MIN_BRIGHT:
                ls.brightness = ls.minBright;
                break;
            case ANIM_ZERO:
                ls.brightness = 0.0f;
                break;
            case ANIM_MAX_BRIGHT:
                ls.brightness = ls.maxBright;
                break;
            case ANIM_DIM:
                ls.brightness = ls.isRising ? ls.minBright : ls.maxBright;
                break;
            case ANIM_BRIGHTEN:
                ls.brightness = ls.isRising ? ls.maxBright : ls.minBright;
                break;
            default:
                ls.brightness = ls.isRising ? ls.minBright : ls.maxBright;
                break;
        }

        // Set initial countdown if not loaded from save
        if (ls.countdown <= 0.0f) {
            ls.countdown = ls.isRising ? ls.brightenTime : ls.dimTime;
        }

        if (ls.maxBright <= 0.0f) {
            std::fprintf(stderr,
                "[DEFAULT] LightingSystem: light %d maxBright=%.3f <= 0, "
                "prevIntensity defaulting to 0.0\n",
                (int)prop.lightNum, ls.maxBright);
        }
        ls.prevIntensity = (ls.maxBright > 0.0f)
            ? ls.brightness / ls.maxBright : 0.0f;

        // Compute on/off modes per the original engine's InitModes logic.
        // LightScriptService::activate/deactivate use these instead of
        // hardcoded MAX/ZERO so authored FLICKER/RANDOM/SMOOTH survive
        // a turn-on round-trip.
        computeLiteModes(ls);

        lights[prop.lightNum] = ls;
        ++parsed;
    }

    // Count animated (non-static) lights and report details
    int animatedCount = 0;
    int inactiveCount = 0;
    for (const auto &[num, ls] : lights) {
        if (ls.mode != ANIM_MAX_BRIGHT && ls.mode != ANIM_MIN_BRIGHT
            && ls.mode != ANIM_ZERO)
            animatedCount++;
        if (ls.inactive)
            inactiveCount++;
    }

    std::fprintf(stderr,
        "LightingSystem: parsed %d AnimLight properties "
        "(%d animated, %d static, %d inactive)\n",
        parsed, animatedCount, parsed - animatedCount, inactiveCount);

    return lights;
}

// ── Animation state machine ──
//
// Advances a single light's animation by dt seconds.
// Returns true if the brightness changed (for dirty detection).
// Implements all 10 Dark Engine animation modes.

inline bool updateLightAnimation(LightSource &light, float dt) {
    if (light.inactive) return false;

    float range = light.maxBright - light.minBright;
    if (range <= 0.0f) {
        static int warnCount = 0;
        if (warnCount++ < 5)
            std::fprintf(stderr, "[DEFAULT] LightingSystem::updateLightAnimation: light %d range=%.3f (max=%.3f min=%.3f) <= 0, skipping\n",
                         light.lightNum, range, light.maxBright, light.minBright);
        return false;
    }

    float prevBrightness = light.brightness;

    switch (light.mode) {
        case ANIM_FLIP: {
            // Toggle instantly between min and max when timer expires
            light.countdown -= dt;
            if (light.countdown <= 0.0f) {
                if (light.isRising) {
                    light.brightness = light.maxBright;
                    light.isRising = false;
                    light.countdown = light.dimTime;
                } else {
                    light.brightness = light.minBright;
                    light.isRising = true;
                    light.countdown = light.brightenTime;
                }
            }
            break;
        }

        case ANIM_SMOOTH: {
            // Linear interpolation, reverses direction at limits
            light.countdown -= dt;
            if (light.isRising) {
                float duration = light.brightenTime;
                if (duration > 0.0f) {
                    light.brightness += (range * dt) / duration;
                }
                if (light.countdown <= 0.0f || light.brightness >= light.maxBright) {
                    light.brightness = light.maxBright;
                    light.isRising = false;
                    light.countdown = light.dimTime;
                }
            } else {
                float duration = light.dimTime;
                if (duration > 0.0f) {
                    light.brightness -= (range * dt) / duration;
                }
                if (light.countdown <= 0.0f || light.brightness <= light.minBright) {
                    light.brightness = light.minBright;
                    light.isRising = true;
                    light.countdown = light.brightenTime;
                }
            }
            break;
        }

        case ANIM_RANDOM: {
            // Random brightness each cycle
            light.countdown -= dt;
            if (light.countdown <= 0.0f) {
                float t = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
                light.brightness = light.minBright + t * range;
                // Alternate between brighten and dim timings
                light.isRising = !light.isRising;
                light.countdown = light.isRising ? light.brightenTime : light.dimTime;
                if (light.countdown <= 0.0f) {
                    light.countdown = 0.1f; // prevent zero-timer spin
                    std::fprintf(stderr, "[DEFAULT] LightingSystem: light %d countdown reset to 0.1s (prevent zero-timer spin)\n", light.lightNum);
                }
            }
            break;
        }

        case ANIM_MIN_BRIGHT:
            // Static at minimum brightness — report change if just switched
            light.brightness = light.minBright;
            return (light.brightness != prevBrightness);

        case ANIM_MAX_BRIGHT:
            // Static at maximum brightness — report change if just switched
            light.brightness = light.maxBright;
            return (light.brightness != prevBrightness);

        case ANIM_ZERO:
            // Static at zero brightness — report change if just switched
            light.brightness = 0.0f;
            return (light.brightness != prevBrightness);

        case ANIM_BRIGHTEN: {
            // One-shot linear fade to max, then stop
            if (light.brightness < light.maxBright) {
                float duration = light.brightenTime;
                if (duration > 0.0f) {
                    light.brightness += (range * dt) / duration;
                }
                if (light.brightness >= light.maxBright) {
                    light.brightness = light.maxBright;
                }
            }
            break;
        }

        case ANIM_DIM: {
            // One-shot linear fade to min, then stop
            if (light.brightness > light.minBright) {
                float duration = light.dimTime;
                if (duration > 0.0f) {
                    light.brightness -= (range * dt) / duration;
                }
                if (light.brightness <= light.minBright) {
                    light.brightness = light.minBright;
                }
            }
            break;
        }

        case ANIM_SEMI_RANDOM: {
            // Bounded random walk — smooth random within ±60% of range per cycle
            light.countdown -= dt;
            if (light.countdown <= 0.0f) {
                float maxDelta = range * 0.6f;
                float delta = (static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX)
                               * 2.0f - 1.0f) * maxDelta;
                light.brightness += delta;
                light.brightness = std::max(light.minBright,
                                   std::min(light.maxBright, light.brightness));
                light.isRising = !light.isRising;
                light.countdown = light.isRising ? light.brightenTime : light.dimTime;
                if (light.countdown <= 0.0f)
                    light.countdown = 0.05f; // prevent zero-timer spin
            }
            break;
        }

        case ANIM_FLICKER: {
            // Rapid random toggle with ±50% timing variance
            light.countdown -= dt;
            if (light.countdown <= 0.0f) {
                if (light.isRising) {
                    light.brightness = light.maxBright;
                    light.isRising = false;
                    // Add ±50% random variance to timer
                    float variance = 0.5f + static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
                    light.countdown = light.dimTime * variance;
                } else {
                    light.brightness = light.minBright;
                    light.isRising = true;
                    float variance = 0.5f + static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
                    light.countdown = light.brightenTime * variance;
                }
                if (light.countdown <= 0.0f)
                    light.countdown = 0.02f; // prevent zero-timer spin
            }
            break;
        }

        default:
            return false;
    }

    // Clamp brightness to valid range
    light.brightness = std::max(0.0f, std::min(light.maxBright, light.brightness));

    // Return true if brightness actually changed
    return std::abs(light.brightness - prevBrightness) > 0.001f;
}

// ── Reverse index builder ──
//
// Builds a mapping: lightnum → [(cellIdx, polyIdx)]
// For each cell, walks set bits in each polygon's animflags and maps them
// to the corresponding animMap entry (which gives the lightnum).
// This tells us which atlas regions to re-blend when a light changes intensity.

inline std::unordered_map<int16_t, std::vector<std::pair<uint32_t, int>>>
buildAnimLightIndex(const WRParsedData &wr)
{
    std::unordered_map<int16_t, std::vector<std::pair<uint32_t, int>>> index;

    for (uint32_t ci = 0; ci < wr.numCells; ++ci) {
        const auto &cell = wr.cells[ci];

        for (int pi = 0; pi < cell.numTextured; ++pi) {
            uint32_t flags = cell.lightInfos[pi].animflags;
            if (flags == 0) continue;

            // Walk set bits in animflags
            while (flags) {
                int bit = __builtin_ctz(flags); // lowest set bit
                flags &= flags - 1;            // clear it

                // The bit position indexes into the cell's animMap
                if (bit < static_cast<int>(cell.animMap.size())) {
                    int16_t lightNum = cell.animMap[bit];
                    index[lightNum].emplace_back(ci, pi);
                }
            }
        }
    }

    // Report stats
    int totalMappings = 0;
    for (const auto &kv : index)
        totalMappings += static_cast<int>(kv.second.size());

    std::fprintf(stderr, "LightingSystem: %zu unique lightnums → %d polygon mappings\n",
                 index.size(), totalMappings);

    return index;
}

} // namespace Darkness
