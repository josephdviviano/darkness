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

// Data structures for the Dark Engine schema sound system.
// Schemas define how sounds are selected and played — volume, priority,
// looping, sample selection, tag-based matching for environmental sounds
// and AI speech.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace Darkness {

/// Audio class categories (maps to eSchemaType in original engine)
enum class SchemaAudioClass : uint8_t {
    Noise = 0,       // General sound effects
    Speech = 1,      // Voice/dialogue
    Ambient = 2,     // Ambient background sounds
    Music = 3,       // Music
    MetaUI = 4,      // UI/inventory/metagame sounds
    PlayerFeet = 5,  // Player footstep sounds
    OtherFeet = 6,   // AI/NPC footstep sounds
    Collisions = 7,  // Collision sounds
    Weapons = 8,     // Weapon sounds
    Monsters = 9     // Monster/creature sounds
};

/// Schema flags (bitfield, maps to SCH_* constants in original engine)
enum SchemaFlags : uint32_t {
    SCH_FLAG_NONE      = 0,
    SCH_RETRIGGER      = 1 << 0,   // Restart when triggered again while playing
    SCH_PAN_POS        = 1 << 1,   // Fixed pan position described
    SCH_PAN_RANGE      = 1 << 2,   // Pan randomized within range
    SCH_NO_REPEAT      = 1 << 3,   // Don't repeat the last sample
    SCH_NO_CACHE       = 1 << 4,   // Don't cache audio data
    SCH_STREAM         = 1 << 5,   // Stream from disk
    SCH_PLAY_ONCE      = 1 << 6,   // Play only once ever
    SCH_NO_COMBAT      = 1 << 7,   // Don't play during combat
    SCH_NET_AMBIENT    = 1 << 8,   // Broadcast ambient over network
    SCH_LOC_SPATIAL    = 1 << 9,   // Local-only spatial sound
};

/// Playback parameters (maps to sSchemaPlayParams, 20 bytes on disk)
struct SchemaPlayParams {
    uint32_t flags = 0;           // SchemaFlags bitfield
    int volume = -1;              // Nominal volume (-10000 to -1)
    int pan = 0;                  // Pan position or range
    int initialDelay = 0;         // Delay before start (ms)
    int fade = 0;                 // Fade in/out duration (ms)
    SchemaAudioClass audioClass = SchemaAudioClass::Noise;
    int priority = 128;           // 0-255, default 128
};

/// Loop parameters (maps to sSchemaLoopParams, 8 bytes on disk)
struct SchemaLoopParams {
    bool isLooping = false;       // Whether this schema loops at all
    bool isPoly = false;          // true = poly_loop, false = mono_loop
    uint8_t maxSamples = 1;       // Max simultaneous samples (poly_loop)
    uint16_t count = 0;           // Iteration limit (0 = infinite)
    uint16_t intervalMin = 0;     // Min delay between iterations (ms)
    uint16_t intervalMax = 0;     // Max delay between iterations (ms)
};

/// A single sound sample within a schema
struct SchemaSample {
    std::string name;             // WAV filename (bare name, no extension)
    uint8_t frequency = 1;        // Selection weight (0-255, default 1)
    std::string text;             // Optional subtitle text
};

/// A tag value for env_tag/schema_voice matching
struct SchemaTagValue {
    std::string tagName;
    std::vector<std::string> enumValues;  // For enum matching
    int rangeMin = 0;                     // For integer range matching
    int rangeMax = 0;
    bool isIntRange = false;
};

/// A complete schema entry
struct SchemaEntry {
    std::string name;
    std::string archetypeName;    // Parent archetype name (for inheritance)
    SchemaPlayParams playParams;
    SchemaLoopParams loopParams;
    std::vector<SchemaSample> samples;

    // Environmental sound matching (env_tag)
    std::vector<SchemaTagValue> envTags;

    // Speech matching (schema_voice)
    std::string voiceName;
    int voiceWeight = 0;
    std::string conceptName;
    std::vector<SchemaTagValue> voiceTags;

    // Message label for AI notification
    std::string message;

    bool hasEnvTags() const { return !envTags.empty(); }
    bool hasVoice() const { return !voiceName.empty(); }

    /// Total frequency weight across all samples
    int totalFrequency() const {
        int sum = 0;
        for (const auto &s : samples) sum += s.frequency;
        return sum;
    }
};

/// Tag definition from .spc files
struct TagDefinition {
    std::string name;
    std::vector<std::string> values;  // Allowed enum values (empty for tag_int)
    bool isIntTag = false;            // true for tag_int, false for tag
    bool isRequired = false;          // true if env_tag_required
};

/// Voice definition from .spc files
struct VoiceDefinition {
    std::string name;
    std::string archetypeName;  // Parent voice (archetype hierarchy)
};

/// Concept definition from .spc files
struct ConceptDefinition {
    std::string name;
    int priority = 0;
};

} // namespace Darkness
