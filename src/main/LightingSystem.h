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

#include "File.h"
#include "FileGroup.h"
#include "FileCompat.h"
#include "WRChunkParser.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
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
};

// ── AnimLight property parser ──
//
// Reads P$AnimLight records (76 bytes each) from the mission file.
// Also reads P$Position to get world position for each light object
// (useful for future volumetric/dynamic lighting).
//
// P$AnimLight on-disk layout (76 bytes):
//   int32  unk1              (offset 0)
//   float3 offset            (offset 4)
//   int32  unk2              (offset 16)
//   int16  cellindex         (offset 20)
//   int16  hitcells          (offset 22)
//   int16  lightnum          (offset 24)
//   uint16 mode              (offset 26)
//   int32  brightentime_ms   (offset 28)
//   int32  dimtime_ms        (offset 32)
//   float  minbrightness     (offset 36)
//   float  maxbrightness     (offset 40)
//   int32  unk4              (offset 44)
//   bool32 rising            (offset 48)
//   int32  countdown_ms      (offset 52)
//   bool32 inactive          (offset 56)
//   float  radius            (offset 60)
//   int32  unk5              (offset 64)
//   bool32 quadlit           (offset 68)
//   float  innerradius       (offset 72)

inline std::unordered_map<int16_t, LightSource>
parseAnimLightProperties(const std::string &misPath)
{
    std::unordered_map<int16_t, LightSource> lights;

    FilePtr fp(new StdFile(misPath, File::FILE_R));
    FileGroupPtr db(new DarkFileGroup(fp));

    // First pass: read P$Position for all objects so we can look up light positions
    std::unordered_map<int32_t, std::array<float, 3>> positions;
    if (db->hasFile("P$Position")) {
        FilePtr posFile = db->getFile("P$Position");
        while (static_cast<size_t>(posFile->tell()) + 8 <= posFile->size()) {
            uint32_t objID, dataSize;
            *posFile >> objID >> dataSize;

            if (dataSize >= 12) {
                float px, py, pz;
                *posFile >> px >> py >> pz;
                positions[static_cast<int32_t>(objID)] = {px, py, pz};
                // Skip remaining bytes in this record
                if (dataSize > 12)
                    posFile->seek(dataSize - 12, File::FSEEK_CUR);
            } else if (dataSize > 0) {
                posFile->seek(dataSize, File::FSEEK_CUR);
            }
        }
    }

    // Second pass: read P$AnimLight properties
    const char *chunkName = "P$AnimLigh"; // truncated to 11 chars (10 + null)
    if (!db->hasFile(chunkName)) {
        // Try full name in case of different truncation
        chunkName = "P$AnimLight";
        if (!db->hasFile(chunkName)) {
            std::fprintf(stderr, "LightingSystem: no P$AnimLight chunk found\n");
            return lights;
        }
    }

    FilePtr animFile = db->getFile(chunkName);

    // P$ property records: {uint32_t objID, uint32_t dataSize, data[dataSize]}
    int parsed = 0;
    while (static_cast<size_t>(animFile->tell()) + 8 <= animFile->size()) {
        uint32_t objID, dataSize;
        *animFile >> objID >> dataSize;

        if (dataSize >= 76) {
            // Read the AnimLight struct fields
            int32_t unk1;
            float offsetX, offsetY, offsetZ;
            int32_t unk2;
            int16_t cellindex, hitcells, lightnum;
            uint16_t mode;
            int32_t brightentime, dimtime;
            float minbrightness, maxbrightness;
            int32_t unk4;
            uint32_t rising;
            int32_t countdown;
            uint32_t inactive;
            float radius;
            int32_t unk5;
            uint32_t quadlit;
            float innerradius;

            *animFile >> unk1;
            *animFile >> offsetX >> offsetY >> offsetZ;
            *animFile >> unk2;
            *animFile >> cellindex >> hitcells >> lightnum;
            *animFile >> mode;
            *animFile >> brightentime >> dimtime;
            *animFile >> minbrightness >> maxbrightness;
            *animFile >> unk4;
            *animFile >> rising;
            *animFile >> countdown;
            *animFile >> inactive;
            *animFile >> radius;
            *animFile >> unk5;
            *animFile >> quadlit;
            *animFile >> innerradius;

            // Skip remaining bytes if dataSize > 76
            if (dataSize > 76)
                animFile->seek(dataSize - 76, File::FSEEK_CUR);

            // Skip lights with invalid lightnum
            if (lightnum < 0) continue;

            LightSource ls;
            ls.lightNum = lightnum;
            ls.objectId = static_cast<int32_t>(objID);

            // Get world position from P$Position + offset
            auto posIt = positions.find(ls.objectId);
            if (posIt != positions.end()) {
                ls.posX = posIt->second[0] + offsetX;
                ls.posY = posIt->second[1] + offsetY;
                ls.posZ = posIt->second[2] + offsetZ;
            } else {
                ls.posX = offsetX;
                ls.posY = offsetY;
                ls.posZ = offsetZ;
            }

            ls.radius = radius;
            ls.innerRadius = innerradius;
            ls.mode = mode;
            ls.brightenTime = brightentime / 1000.0f; // ms → seconds
            ls.dimTime = dimtime / 1000.0f;
            ls.minBright = minbrightness;
            ls.maxBright = maxbrightness;
            ls.isRising = (rising != 0);
            ls.inactive = (inactive != 0);
            ls.countdown = countdown / 1000.0f;

            // Initialize brightness based on mode and state
            switch (mode) {
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
                    // Start at max, will dim over time
                    ls.brightness = ls.isRising ? ls.minBright : ls.maxBright;
                    break;
                case ANIM_BRIGHTEN:
                    // Start at min, will brighten over time
                    ls.brightness = ls.isRising ? ls.maxBright : ls.minBright;
                    break;
                default:
                    // For animated modes, start based on rising/dimming state
                    ls.brightness = ls.isRising ? ls.minBright : ls.maxBright;
                    break;
            }

            // Set initial countdown if not loaded from save
            if (ls.countdown <= 0.0f) {
                ls.countdown = ls.isRising ? ls.brightenTime : ls.dimTime;
            }

            // Compute initial intensity for dirty tracking
            ls.prevIntensity = (ls.maxBright > 0.0f)
                ? ls.brightness / ls.maxBright : 0.0f;

            lights[lightnum] = ls;
            ++parsed;
        } else if (dataSize > 0) {
            animFile->seek(dataSize, File::FSEEK_CUR);
        }
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

    std::fprintf(stderr, "LightingSystem: parsed %d AnimLight properties (%d animated, %d static, %d inactive)\n",
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
    if (range <= 0.0f) return false;

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
                if (light.countdown <= 0.0f)
                    light.countdown = 0.1f; // prevent zero-timer spin
            }
            break;
        }

        case ANIM_MIN_BRIGHT:
            // Static at minimum brightness — no animation
            light.brightness = light.minBright;
            return false;

        case ANIM_MAX_BRIGHT:
            // Static at maximum brightness — no animation
            light.brightness = light.maxBright;
            return false;

        case ANIM_ZERO:
            // Static at zero brightness — no animation
            light.brightness = 0.0f;
            return false;

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
