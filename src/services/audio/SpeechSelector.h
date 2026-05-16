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

// SpeechSelector — runtime selection of a speech-schema archetype for a given
// (voice, concept, tag-query) triple, backed by a loaded SpeechDatabase.
//
// Background: Speech_DB's per-voice tag-database trees encode the original
// engine's speech-selection logic. A voice (e.g. "Karras", "Garrett") holds
// one tag-DB per concept ("Greet", "Alert", "sleeping" …). Each tag-DB
// branches on key/value constraints; leaves carry one or more
// (schemaObjID, weight) records. The original engine walked the tree with
// the caller's tag query, collected every leaf whose keyPath was satisfied
// by the query, and weighted-random-picked one of the schemas.
//
// This class is a stateless pure query layer over SpeechDatabase. It does
// not own the SpeechDatabase reference; callers must outlive any selector
// use. NO_REPEAT-style avoid-last-pick tracking is left to the caller
// (AudioService::mLastSampleIdx is keyed by schema name, which is the
// natural integration point — selecting two different schemas back-to-back
// is fine, we only avoid replaying the same sample within a schema).
//
// Randomness is injectable so tests are deterministic. Production callers
// either pass their own RNG (e.g. the AudioService-owned mt19937) or omit
// the argument to let the selector use a default-seeded RNG.

#pragma once

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "SpeechDatabase.h"

namespace Darkness {

class SpeechSelector {
public:
    /// Tag-value query input. `intValue` is used when the tag is int-typed
    /// (matched against the entry's [keyMin..keyMax] range); `enumValues`
    /// is a list of value-name indices used when the tag is enum-typed
    /// (matches if any query value also appears in the entry's enum set).
    ///
    /// Strings are used at the public boundary; internally the selector
    /// resolves tagName → tagType-index and enumValues[] → value-name-
    /// indices once per query via SpeechDatabase's name maps.
    struct TagQuery {
        std::string              tagName;
        bool                     hasInt = false;
        int32_t                  intValue = 0;
        std::vector<std::string> enumValues;
    };

    /// A single candidate (post-tag-filtering). Returned for tests +
    /// callers that want to inspect every match before picking one.
    struct Candidate {
        int32_t schemaObjID = 0;
        float   weight      = 1.0f;
    };

    /// Construct a selector bound to `db`. The reference must outlive
    /// the selector.
    explicit SpeechSelector(const SpeechDatabase &db) : mDB(db) {}

    /// Return true iff the bound speech database is loaded and ready to
    /// answer queries.
    bool isReady() const { return mDB.isLoaded(); }

    /// Resolve a voice name (P$SpchVoice contents) to a voice index for
    /// use with selectMatches / pickOne. Returns -1 when the name is
    /// not found. Voice names live in the speech domain as the *concept*
    /// name map? No — in this engine the voices have no global name map;
    /// they are identified by integer index (P$VoiceIdx) primarily. The
    /// 16-byte P$SpchVoice string is a developer-facing label and is
    /// resolved by walking the concepts ("voice0", "voice1" …) — but in
    /// stock Thief 2 these are sparse; we accept either an int or this
    /// name lookup at the AudioService layer. This helper is kept for
    /// completeness and returns -1 unconditionally if the domain has
    /// no voice-naming convention to look up against; callers should
    /// prefer P$VoiceIdx (integer).
    int32_t findVoiceByName(const std::string &voiceName) const;

    /// Enumerate every leaf whose (voiceIndex, conceptName) matches and
    /// whose keyPath is fully satisfied by the query tag set. Returns
    /// empty when the concept is unknown to the domain or no leaves
    /// satisfy the query.
    ///
    /// Matching rule (per original engine convention):
    ///   - empty keyPath always matches (default / unconditional leaf)
    ///   - for each KeySegment in the leaf's keyPath, *some* TagQuery
    ///     must have a matching tagName AND a value that satisfies the
    ///     segment's range (int-typed) or enum set (enum-typed)
    ///   - extra unmatched query tags are ignored (don't disqualify)
    ///
    /// This is "the query is a superset of the leaf's constraints" —
    /// the leaf is a partial specification, the query is the full state.
    std::vector<Candidate> selectMatches(
        int32_t voiceIndex,
        const std::string &conceptName,
        const std::vector<TagQuery> &query) const;

    /// Pick one candidate at random, weighted by `Candidate::weight`.
    /// Returns nullopt-equivalent (schemaObjID==0) when `candidates`
    /// is empty. RNG is the caller's; production code should reuse a
    /// long-lived RNG so per-call seeding doesn't reset entropy.
    Candidate pickOne(const std::vector<Candidate> &candidates,
                      std::mt19937 &rng) const;

    /// Convenience: combine selectMatches + pickOne in one call. Uses
    /// the selector-owned RNG (default-seeded). Returns a Candidate
    /// with schemaObjID == 0 on miss.
    Candidate selectOne(int32_t voiceIndex,
                        const std::string &conceptName,
                        const std::vector<TagQuery> &query);

    /// Seed the selector-owned RNG. Useful for tests that want
    /// reproducible picks without threading their own RNG through
    /// selectOne callers.
    void seed(uint32_t s) { mRng.seed(s); }

private:
    // Resolve a query tag's value-name strings to value-name indices
    // (indices into SpeechDatabase::valueNames). Returns an empty list
    // when none of the names are known. Reuses a small per-call buffer
    // so resolveTags() can be called per-leaf-key without churn.
    std::vector<uint8_t> resolveValueNames(
        const std::vector<std::string> &names) const;

    // Resolve a tag name to a tag-type index. Returns UINT32_MAX when
    // the name is unknown to the domain.
    uint32_t resolveTagName(const std::string &name) const;

    // Test whether one KeySegment is satisfied by the query.
    bool segmentMatches(const SpeechDatabase::KeySegment &seg,
                        const std::vector<TagQuery> &query) const;

    const SpeechDatabase &mDB;
    std::mt19937          mRng{std::random_device{}()};
};

} // namespace Darkness
