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

// Global AI hearing-data chunk decoders.
//
// The Dark Engine stores two small global chunks in the mission/gam
// database that drive how AIs hear sound:
//
//   AIHearStat (48 bytes) — per-hearing-rating distance multiplier and
//     dB add. Six ratings: Deaf, VeryLow, Low, Normal, High, VeryHigh.
//     Layout on disk: 6 little-endian floats of dist_muls, followed by
//     6 little-endian int32s of db_adds (centibels). Note the type change
//     mid-chunk — db_adds are stored as signed integers in the original
//     engine, e.g. Deaf=1000000, Normal=0, VeryHigh=-1000. Decoding them
//     as floats produces denormals/NaNs.
//
//   AISNDTWK  (24 bytes) — default audible-range (in world units) for
//     each of six AI sound types: Untyped, Inform, MinorAnomaly,
//     MajorAnomaly, NonCombatHigh, Combat. Layout: 6 little-endian int32s.
//
// This file provides parsing-only helpers: a future AI hearing runtime
// will consume the parsed `AIHearingStats` and `AISoundTweaks` to gate
// AI awareness events. No runtime behavior is implemented here.

#pragma once

#include <cstdint>
#include <cstring>

namespace Darkness {

// ════════════════════════════════════════════════════════════════════════════
// Enum mirrors (kept as anonymous enums so the header has no link-time deps).
// These mirror the on-disk ordering — do NOT renumber.
// ════════════════════════════════════════════════════════════════════════════

// AI hearing rating slot index. Stored in P$AI_Hearin per-AI archetype.
enum AIHearingRating : int {
    AI_HEARING_DEAF      = 0,
    AI_HEARING_VERY_LOW  = 1,
    AI_HEARING_LOW       = 2,
    AI_HEARING_NORMAL    = 3,
    AI_HEARING_HIGH      = 4,
    AI_HEARING_VERY_HIGH = 5,
    AI_HEARING_COUNT     = 6,
};

// AI sound type slot index. Used by AISNDTWK and per-AI sound-type tables.
enum AISoundType : int {
    AI_SOUND_UNTYPED         = 0,
    AI_SOUND_INFORM          = 1,
    AI_SOUND_MINOR_ANOMALY   = 2,
    AI_SOUND_MAJOR_ANOMALY   = 3,
    AI_SOUND_NON_COMBAT_HIGH = 4,
    AI_SOUND_COMBAT          = 5,
    AI_SOUND_TYPE_COUNT      = 6,
};

// ════════════════════════════════════════════════════════════════════════════
// AIHearingStats — decoded AIHearStat chunk.
//
// For an emitted sound with apparent SPL `db` at distance `d`, an AI with
// rating `r` perceives loudness `db + db_adds[r]` after the distance is
// scaled by `dist_muls[r]`. Slot 0 (Deaf) uses `0 * dist + 1000000 dB add`
// — the AI is effectively deaf.
//
// db_adds are in centibels (0.1 dB), stored as int32 on disk. dist_muls
// are floats. The struct is plain old data, layout-matched to the on-disk
// chunk (6 floats + 6 int32s, no padding). Total size: 48 bytes.
// ════════════════════════════════════════════════════════════════════════════

struct AIHearingStats {
    float   dist_muls[AI_HEARING_COUNT];
    int32_t db_adds  [AI_HEARING_COUNT];
};

static_assert(sizeof(AIHearingStats) == 48,
              "AIHearingStats must be 48 bytes — matches AIHearStat chunk layout");

// Original engine default values (used when no AIHearStat chunk is present
// in the database). The Deaf slot's huge db_add ensures no sound is ever
// loud enough to be heard.
static constexpr AIHearingStats kDefaultAIHearingStats = {
    /* dist_muls */ { 0.0f, 0.25f, 0.65f, 1.0f, 1.5f, 3.0f },
    /* db_adds   */ { 1000000, 1000, 200, 0, -200, -1000 },
};

// ════════════════════════════════════════════════════════════════════════════
// AISoundTweaks — decoded AISNDTWK chunk.
//
// Per AI sound type, gives the default audible range in world units. An
// emitter without an explicit range falls back to this table.
// ════════════════════════════════════════════════════════════════════════════

struct AISoundTweaks {
    int32_t defaultRanges[AI_SOUND_TYPE_COUNT];
};

static_assert(sizeof(AISoundTweaks) == 24,
              "AISoundTweaks must be 24 bytes — matches AISNDTWK chunk layout");

// ════════════════════════════════════════════════════════════════════════════
// Chunk decoders. Both return false on size mismatch; on success they
// fully populate `out`.
// ════════════════════════════════════════════════════════════════════════════

// Decode a 48-byte AIHearStat chunk into `out`.
inline bool readAIHearStat(const uint8_t *bytes, size_t sz, AIHearingStats &out) {
    if (bytes == nullptr || sz != sizeof(AIHearingStats))
        return false;
    // On-disk layout: 6 little-endian floats (dist_muls) followed by 6
    // little-endian int32s (db_adds, in centibels). Struct field order
    // matches, and all supported targets are little-endian, so we can
    // memcpy each half directly into the struct.
    std::memcpy(out.dist_muls, bytes + 0,  sizeof(out.dist_muls));
    std::memcpy(out.db_adds,   bytes + 24, sizeof(out.db_adds));
    return true;
}

// Decode a 24-byte AISNDTWK chunk into `out`.
inline bool readAISndTwk(const uint8_t *bytes, size_t sz, AISoundTweaks &out) {
    if (bytes == nullptr || sz != sizeof(AISoundTweaks))
        return false;
    // 6 little-endian int32s, packed; raw memcpy on little-endian targets.
    std::memcpy(&out, bytes, sizeof(AISoundTweaks));
    return true;
}

// ════════════════════════════════════════════════════════════════════════════
// Display name helpers for debug dumps / headless inspectors.
// ════════════════════════════════════════════════════════════════════════════

inline const char *aiHearingRatingName(int r) {
    switch (r) {
        case AI_HEARING_DEAF:      return "Deaf";
        case AI_HEARING_VERY_LOW:  return "VeryLow";
        case AI_HEARING_LOW:       return "Low";
        case AI_HEARING_NORMAL:    return "Normal";
        case AI_HEARING_HIGH:      return "High";
        case AI_HEARING_VERY_HIGH: return "VeryHigh";
        default:                   return "?";
    }
}

inline const char *aiSoundTypeName(int t) {
    switch (t) {
        case AI_SOUND_UNTYPED:         return "Untyped";
        case AI_SOUND_INFORM:          return "Inform";
        case AI_SOUND_MINOR_ANOMALY:   return "MinorAnomaly";
        case AI_SOUND_MAJOR_ANOMALY:   return "MajorAnomaly";
        case AI_SOUND_NON_COMBAT_HIGH: return "NonCombatHigh";
        case AI_SOUND_COMBAT:          return "Combat";
        default:                       return "?";
    }
}

} // namespace Darkness
