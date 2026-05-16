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

#include "SpeechSelector.h"

#include <cstdint>
#include <limits>

namespace Darkness {

// ── Name resolution helpers ─────────────────────────────────────────────

uint32_t SpeechSelector::resolveTagName(const std::string &name) const {
    const auto &tags = mDB.tagNames();
    for (size_t i = 0; i < tags.size(); ++i) {
        if (tags[i] == name) return static_cast<uint32_t>(i);
    }
    return std::numeric_limits<uint32_t>::max();
}

std::vector<uint8_t> SpeechSelector::resolveValueNames(
    const std::vector<std::string> &names) const
{
    std::vector<uint8_t> out;
    out.reserve(names.size());
    const auto &values = mDB.valueNames();
    for (const auto &n : names) {
        for (size_t i = 0; i < values.size() && i < 255; ++i) {
            // Skip slot 255 — kEnumTerminator sentinel is reserved.
            if (values[i] == n) {
                out.push_back(static_cast<uint8_t>(i));
                break;
            }
        }
    }
    return out;
}

int32_t SpeechSelector::findVoiceByName(const std::string &voiceName) const {
    // The speech domain in stock Thief 2 does not carry a voice-name map;
    // voices are addressed strictly by integer index. The 16-byte
    // P$SpchVoice string is the developer label and is matched at the
    // game layer (caller maintains a side table or a convention). Return
    // -1 so callers that try a name lookup fall through to a loud failure
    // path rather than silently mis-binding to voice 0.
    (void)voiceName;
    return -1;
}

// ── Per-segment matching ────────────────────────────────────────────────

bool SpeechSelector::segmentMatches(
    const SpeechDatabase::KeySegment &seg,
    const std::vector<TagQuery> &query) const
{
    // Find any query tag whose name resolves to seg.keyType. A leaf may
    // have multiple key segments; each must be satisfied by some query
    // tag, but a query tag may satisfy several segments.
    for (const auto &q : query) {
        const uint32_t qType = resolveTagName(q.tagName);
        if (qType != seg.keyType) continue;

        // Int-typed (or unflagged — we default to range for unflagged
        // tags, matching the resolver's formatKey policy and stock
        // Thief 2 convention for non-enum keys).
        if (!mDB.isEnumTag(seg.keyType)) {
            if (!q.hasInt) continue;  // wrong-shaped query value
            if (q.intValue >= seg.keyMin && q.intValue <= seg.keyMax)
                return true;
            continue;
        }

        // Enum-typed: any overlap between the query's value-name set
        // and the leaf's value-byte set satisfies the segment.
        std::vector<uint8_t> qVals = resolveValueNames(q.enumValues);
        for (uint8_t qv : qVals) {
            for (size_t i = 0; i < 8; ++i) {
                const uint8_t kv = seg.keyEnums[i];
                if (kv == SpeechDatabase::kEnumTerminator) break;
                if (kv == qv) return true;
            }
        }
    }
    return false;
}

// ── Public query API ────────────────────────────────────────────────────

std::vector<SpeechSelector::Candidate>
SpeechSelector::selectMatches(int32_t voiceIndex,
                              const std::string &conceptName,
                              const std::vector<TagQuery> &query) const
{
    std::vector<Candidate> out;
    if (!mDB.isLoaded()) return out;

    // Resolve concept name to index. Concept index lookup uses the
    // concept name map; out-of-range conceptIndex means the domain
    // does not know this concept at all.
    const auto &concepts = mDB.conceptNames();
    uint32_t conceptIndex = std::numeric_limits<uint32_t>::max();
    for (size_t i = 0; i < concepts.size(); ++i) {
        if (concepts[i] == conceptName) {
            conceptIndex = static_cast<uint32_t>(i);
            break;
        }
    }
    if (conceptIndex == std::numeric_limits<uint32_t>::max()) return out;
    if (voiceIndex < 0) return out;

    const uint32_t vIdx = static_cast<uint32_t>(voiceIndex);
    for (const auto &e : mDB.entries()) {
        if (e.voiceIndex != vIdx) continue;
        if (e.conceptIndex != conceptIndex) continue;

        // Every key segment in the leaf's path must be satisfied by the
        // query. Empty keyPath always matches (the default leaf).
        bool allMatch = true;
        for (const auto &seg : e.keyPath) {
            if (!segmentMatches(seg, query)) {
                allMatch = false;
                break;
            }
        }
        if (!allMatch) continue;

        for (const auto &d : e.data) {
            Candidate c;
            c.schemaObjID = d.schemaObjID;
            c.weight      = d.weight;
            out.push_back(c);
        }
    }
    return out;
}

SpeechSelector::Candidate SpeechSelector::pickOne(
    const std::vector<Candidate> &candidates, std::mt19937 &rng) const
{
    Candidate miss;
    if (candidates.empty()) return miss;
    if (candidates.size() == 1) return candidates[0];

    // Weighted random. Total weight is the sum across all candidates;
    // we pick a uniform value in [0, total) and walk to find the bucket.
    // Negative or zero weights collapse to uniform random to keep
    // selection robust against malformed input.
    double total = 0.0;
    for (const auto &c : candidates) total += static_cast<double>(c.weight);
    if (total <= 0.0) {
        std::uniform_int_distribution<size_t> uni(0, candidates.size() - 1);
        return candidates[uni(rng)];
    }
    std::uniform_real_distribution<double> uni(0.0, total);
    double pick = uni(rng);
    double acc  = 0.0;
    for (const auto &c : candidates) {
        acc += static_cast<double>(c.weight);
        if (pick < acc) return c;
    }
    return candidates.back();  // float-rounding fallback
}

SpeechSelector::Candidate SpeechSelector::selectOne(
    int32_t voiceIndex,
    const std::string &conceptName,
    const std::vector<TagQuery> &query)
{
    auto matches = selectMatches(voiceIndex, conceptName, query);
    return pickOne(matches, mRng);
}

} // namespace Darkness
