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

// SpeechDatabase — decodes the Speech_DB chunk (Dark Engine speech voice
// database, v1.3). Layout, in order on disk:
//
//   cSpeechDomain
//     NameMap m_Concept    // "sleeping", "atlevelzero", ..., "playerseesfumus"
//     NameMap m_Tag        // "Event", "Fungus", "Landing", "Damage", ...
//     NameMap m_Value      // "Acquire", "Launch", "Collision", "Footstep", ...
//     int     nConceptPrio
//     int32   conceptPrio[nConceptPrio]   // per-concept priority
//     int     nTagFlags
//     int32   tagFlags[nTagFlags]         // bit 0 = int-typed tag, bit 1 = enum-typed
//   int       nVoices
//   cSpeechVoice voices[nVoices]
//     // Each voice = m_Concept.size() sequential tag databases (one per concept).
//
// Where NameMap is:
//   int32 upperBound
//   int32 lowerBound
//   int32 size
//   { char '+' | '-' ; if '+': char[16] name } * size
//
// And each tag database matches the ENV_SOUND format documented in
// EnvSoundDatabase.h. We decode all three layers and surface them through
// flat accessors so callers can join voice index + concept index + decoded
// records back to the human-readable names.

#pragma once

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <string>
#include <vector>

// Re-use the leaf record types from EnvSoundDatabase to keep the API surface
// uniform — the original engine shares one record/key type between the
// speech and env-sound tag databases.
#include "EnvSoundDatabase.h"

namespace Darkness {

class SpeechDatabase {
public:
    using KeySegment = EnvSoundDatabase::KeySegment;
    using DataRecord = EnvSoundDatabase::DataRecord;

    // A single concept-level entry inside one voice.
    // (voiceIndex, conceptIndex) join back to nameMaps for display.
    // `keyPath` is the chain of tag-keys from the concept-DB root to this
    // leaf; `data` is the list of (schemaObjID, weight) pairs stored.
    struct SpeechDBEntry {
        uint32_t                 voiceIndex   = 0;
        uint32_t                 conceptIndex = 0;
        std::vector<KeySegment>  keyPath;
        std::vector<DataRecord>  data;
    };

    SpeechDatabase() = default;
    ~SpeechDatabase() = default;

    SpeechDatabase(const SpeechDatabase &)            = delete;
    SpeechDatabase &operator=(const SpeechDatabase &) = delete;
    SpeechDatabase(SpeechDatabase &&)                 = default;
    SpeechDatabase &operator=(SpeechDatabase &&)      = default;

    // Decode the chunk. Returns false on null / zero-byte input. On a
    // structurally bad buffer, decoding stops at the first failure but
    // any prefix that parsed successfully is retained.
    bool loadFromChunk(const uint8_t *data, size_t size);
    void clear();

    bool   isLoaded() const { return mLoaded; }
    size_t rawSize()  const { return mRaw.size(); }
    const std::vector<uint8_t> &rawBytes() const { return mRaw; }

    // First N raw bytes of the chunk — fingerprint / debug aid.
    std::vector<uint8_t> headerBytes(size_t n = 16) const;

    // Name maps (cSpeechDomain): concept[], tag[], value[]. Indices match
    // those stored in the decoded entries' keyType / iData fields.
    const std::vector<std::string> &conceptNames() const { return mConcepts; }
    const std::vector<std::string> &tagNames()     const { return mTags; }
    const std::vector<std::string> &valueNames()   const { return mValues; }

    // Per-concept priority and per-tag flag arrays (parallel to the
    // corresponding name map). Empty if the chunk wasn't parsed.
    const std::vector<int32_t> &conceptPriorities() const { return mConceptPrio; }
    const std::vector<int32_t> &tagFlags()          const { return mTagFlags; }

    // Number of voices the domain declared. mEntries below only includes
    // *non-empty* concept databases per voice — index back via voiceIndex.
    uint32_t voiceCount() const { return mVoiceCount; }

    // Flat list of decoded leaves across all voices and concepts.
    const std::vector<SpeechDBEntry> &entries() const { return mEntries; }

    // Bytes the decoder couldn't reach (typically empty on a clean parse).
    const std::vector<uint8_t> &tailBytes() const { return mTail; }

    // Print summary + a hex peek to `out`. Intended for CLI use.
    void dump(std::ostream &out) const;

    // --------------------------------------------------------------------
    // Resolver helpers — turn raw KeySegment indices into readable text.
    //
    // The ENV_SOUND chunk does not embed its own name maps; the original
    // engine looks them up in the speech domain at runtime. So both the
    // sound-chunks dumper and any future selector code need to project
    // ENV_SOUND keys through *this* SpeechDatabase to get human-readable
    // tag/value names. Returns "<tag N>" / "<value N>" sentinels when the
    // index is out of range or the name slot is empty.

    // tagFlags bit layout (see Dark Engine convention): bit 0 = int-typed
    // tag (interpret 8-byte payload as {int32 min, int32 max}); bit 1 =
    // enum-typed (interpret as up to 8 value-indices terminated by 0xFF).
    static constexpr int32_t kTagFlagInt  = 0x1;
    static constexpr int32_t kTagFlagEnum = 0x2;
    // Enum-list terminator; matches the 0xFF 'unused' sentinel on disk.
    static constexpr uint8_t kEnumTerminator = 0xFF;

    bool isIntTag(uint32_t keyType) const;
    bool isEnumTag(uint32_t keyType) const;

    // Lookups. Out-of-range indices return a sentinel string like
    // "<tag 7>" / "<value 12>" rather than throwing or returning empty.
    std::string tagTypeName(uint32_t keyType) const;
    std::string valueName(uint8_t valueIdx) const;
    std::string conceptName(uint32_t conceptIdx) const;

    // Format one KeySegment as e.g. "Event=Idle,Greet" (enum-typed) or
    // "CreatureType=[-256..-1]" (int-typed). For unknown tag flags we
    // fall back to the int-range view since that's what most stock
    // Thief 2 tags are.
    std::string formatKey(const KeySegment &seg) const;

    // Comma-joined formatted key path. Empty input yields "<root>".
    std::string formatKeyPath(const std::vector<KeySegment> &path) const;

private:
    std::vector<uint8_t>       mRaw;
    std::vector<std::string>   mConcepts;
    std::vector<std::string>   mTags;
    std::vector<std::string>   mValues;
    std::vector<int32_t>       mConceptPrio;
    std::vector<int32_t>       mTagFlags;
    std::vector<SpeechDBEntry> mEntries;
    std::vector<uint8_t>       mTail;
    uint32_t                   mVoiceCount = 0;
    bool                       mLoaded     = false;
};

} // namespace Darkness
