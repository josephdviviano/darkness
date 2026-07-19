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

#include "SpeechDatabase.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <ostream>

namespace Darkness {

namespace {

// Internal little-endian reader. Same shape as the one in
// EnvSoundDatabase.cpp; kept private to avoid coupling the two
// translation units more than necessary.
struct LeReader {
    const uint8_t *base = nullptr;
    size_t         size = 0;
    size_t         pos  = 0;
    bool           ok   = true;

    bool need(size_t n) {
        if (!ok || pos + n > size) { ok = false; return false; }
        return true;
    }
    int32_t i32() {
        if (!need(4)) return 0;
        int32_t v;
        std::memcpy(&v, base + pos, 4);
        pos += 4;
        return v;
    }
    uint32_t u32() {
        if (!need(4)) return 0;
        uint32_t v;
        std::memcpy(&v, base + pos, 4);
        pos += 4;
        return v;
    }
    float f32() {
        if (!need(4)) return 0.0f;
        float v;
        std::memcpy(&v, base + pos, 4);
        pos += 4;
        return v;
    }
    void read(void *dst, size_t n) {
        if (!need(n)) return;
        std::memcpy(dst, base + pos, n);
        pos += n;
    }
};

constexpr int32_t kMaxRecordsPerNode = 10000;
constexpr int32_t kMaxNameMapSize    = 100000;
constexpr int32_t kMaxVoices         = 1024;
constexpr size_t  kMaxTraversalDepth = 64;

// Decode a single name map. The disk shape is documented in the header.
// Length-prefixed array of 17-byte records (tag byte + optional 16-byte
// fixed name). Empty slots are written as a single '-' byte.
bool decodeNameMap(LeReader &r, std::vector<std::string> &out) {
    out.clear();
    const int32_t upper = r.i32();   // unused at runtime here
    const int32_t lower = r.i32();   // unused at runtime here
    (void)upper; (void)lower;
    const int32_t size  = r.i32();
    if (!r.ok || size < 0 || size > kMaxNameMapSize) return false;
    out.reserve(static_cast<size_t>(size));
    for (int32_t i = 0; i < size; ++i) {
        if (!r.need(1)) return false;
        const char tag = static_cast<char>(r.base[r.pos++]);
        if (tag == '+') {
            char buf[16] = {0};
            r.read(buf, 16);
            if (!r.ok) return false;
            // Names are NUL-padded fixed 16-byte fields (the engine's
            // label convention), so anything past the first NUL is
            // padding garbage we should drop.
            const size_t len = ::strnlen(buf, 16);
            out.emplace_back(buf, len);
        } else if (tag == '-') {
            // Hole in the name map — empty slot. Keep the index aligned
            // by inserting an empty placeholder.
            out.emplace_back();
        } else {
            // Anything else means we lost sync; bail.
            return false;
        }
    }
    return true;
}

// Walk one tag database node (the same serialized layout the original
// engine writes). Identical layout to
// EnvSoundDatabase's decodeNode, but the leaves get tagged with voice +
// concept indices supplied by the caller. Returns false on parse failure.
bool decodeNode(LeReader &r,
                std::vector<SpeechDatabase::KeySegment> &path,
                std::vector<SpeechDatabase::SpeechDBEntry> &out,
                uint32_t voiceIndex,
                uint32_t conceptIndex,
                size_t   depth)
{
    if (depth > kMaxTraversalDepth) return false;

    const int32_t nData = r.i32();
    if (!r.ok || nData < 0 || nData > kMaxRecordsPerNode) return false;

    if (nData > 0) {
        SpeechDatabase::SpeechDBEntry entry;
        entry.voiceIndex   = voiceIndex;
        entry.conceptIndex = conceptIndex;
        entry.keyPath      = path;
        entry.data.reserve(static_cast<size_t>(nData));
        for (int32_t i = 0; i < nData; ++i) {
            SpeechDatabase::DataRecord d;
            d.schemaObjID = r.i32();
            d.weight      = r.f32();
            if (!r.ok) return false;
            entry.data.push_back(d);
        }
        out.push_back(std::move(entry));
    }

    const int32_t nBranch = r.i32();
    if (!r.ok || nBranch < 0 || nBranch > kMaxRecordsPerNode) return false;

    for (int32_t b = 0; b < nBranch; ++b) {
        SpeechDatabase::KeySegment seg;
        seg.keyType = r.u32();
        uint8_t raw[8];
        r.read(raw, 8);
        if (!r.ok) return false;
        std::memcpy(&seg.keyMin, raw + 0, 4);
        std::memcpy(&seg.keyMax, raw + 4, 4);
        std::memcpy(seg.keyEnums, raw, 8);

        path.push_back(seg);
        if (!decodeNode(r, path, out, voiceIndex, conceptIndex, depth + 1))
            return false;
        path.pop_back();
    }
    return true;
}

} // namespace

bool SpeechDatabase::loadFromChunk(const uint8_t *data, size_t size) {
    clear();
    if (data == nullptr || size == 0) return false;

    // Capture raw bytes verbatim so callers can still rebuild the buffer
    // even when decoding fails partway.
    mRaw.assign(data, data + size);

    LeReader r{data, size, 0, true};

    // ------ speech domain ------
    if (!decodeNameMap(r, mConcepts)) {
        mTail.assign(data + r.pos, data + size);
        return false;
    }
    if (!decodeNameMap(r, mTags)) {
        mTail.assign(data + r.pos, data + size);
        return false;
    }
    if (!decodeNameMap(r, mValues)) {
        mTail.assign(data + r.pos, data + size);
        return false;
    }
    {
        const int32_t n = r.i32();
        if (!r.ok || n < 0 || n > kMaxNameMapSize) {
            mTail.assign(data + r.pos, data + size);
            return false;
        }
        mConceptPrio.resize(static_cast<size_t>(n));
        for (int32_t i = 0; i < n; ++i) mConceptPrio[i] = r.i32();
        if (!r.ok) {
            mTail.assign(data + r.pos, data + size);
            return false;
        }
    }
    {
        const int32_t n = r.i32();
        if (!r.ok || n < 0 || n > kMaxNameMapSize) {
            mTail.assign(data + r.pos, data + size);
            return false;
        }
        mTagFlags.resize(static_cast<size_t>(n));
        for (int32_t i = 0; i < n; ++i) mTagFlags[i] = r.i32();
        if (!r.ok) {
            mTail.assign(data + r.pos, data + size);
            return false;
        }
    }

    // ------ Voices ------
    const int32_t nVoices = r.i32();
    if (!r.ok || nVoices < 0 || nVoices > kMaxVoices) {
        mTail.assign(data + r.pos, data + size);
        return false;
    }
    mVoiceCount = static_cast<uint32_t>(nVoices);

    // Each voice persists one tag-DB per concept, in concept-index order.
    const size_t nConcepts = mConcepts.size();
    std::vector<KeySegment> path;
    bool ok = true;
    for (int32_t v = 0; v < nVoices && ok; ++v) {
        for (size_t c = 0; c < nConcepts; ++c) {
            if (!decodeNode(r, path, mEntries,
                            static_cast<uint32_t>(v),
                            static_cast<uint32_t>(c),
                            /*depth=*/0))
            {
                ok = false;
                break;
            }
        }
    }

    // Any unparsed remainder goes into tailBytes() for diagnostic value.
    if (r.pos < size) {
        mTail.assign(data + r.pos, data + size);
    }

    // Loaded if the domain header parsed (we have at least one concept)
    // even when the voice array decode failed mid-way; partial data is
    // still useful for cross-checking and debugging.
    mLoaded = !mConcepts.empty();
    return mLoaded;
}

void SpeechDatabase::clear() {
    mRaw.clear();
    mRaw.shrink_to_fit();
    mConcepts.clear();
    mTags.clear();
    mValues.clear();
    mConceptPrio.clear();
    mTagFlags.clear();
    mEntries.clear();
    mTail.clear();
    mVoiceCount = 0;
    mLoaded = false;
}

std::vector<uint8_t> SpeechDatabase::headerBytes(size_t n) const {
    const size_t count = std::min(n, mRaw.size());
    return std::vector<uint8_t>(mRaw.begin(), mRaw.begin() + count);
}

bool SpeechDatabase::isIntTag(uint32_t keyType) const {
    if (keyType >= mTagFlags.size()) return false;
    return (mTagFlags[keyType] & kTagFlagInt) != 0;
}

bool SpeechDatabase::isEnumTag(uint32_t keyType) const {
    if (keyType >= mTagFlags.size()) return false;
    return (mTagFlags[keyType] & kTagFlagEnum) != 0;
}

// Out-of-range or empty-slot lookups deliberately return a sentinel
// (e.g. "<tag 7>") rather than throwing or returning the empty string,
// so debug/log output is always informative without the caller having
// to special-case missing names.
std::string SpeechDatabase::tagTypeName(uint32_t keyType) const {
    if (keyType < mTags.size() && !mTags[keyType].empty())
        return mTags[keyType];
    char buf[24];
    std::snprintf(buf, sizeof(buf), "<tag %u>", keyType);
    return buf;
}

std::string SpeechDatabase::valueName(uint8_t valueIdx) const {
    if (valueIdx < mValues.size() && !mValues[valueIdx].empty())
        return mValues[valueIdx];
    char buf[24];
    std::snprintf(buf, sizeof(buf), "<value %u>", valueIdx);
    return buf;
}

std::string SpeechDatabase::conceptName(uint32_t conceptIdx) const {
    if (conceptIdx < mConcepts.size() && !mConcepts[conceptIdx].empty())
        return mConcepts[conceptIdx];
    char buf[24];
    std::snprintf(buf, sizeof(buf), "<concept %u>", conceptIdx);
    return buf;
}

std::string SpeechDatabase::formatKey(const KeySegment &seg) const {
    std::string out = tagTypeName(seg.keyType);
    if (isEnumTag(seg.keyType)) {
        out += '=';
        // Enum keys hold up to 8 value-byte indices; 0xFF terminates.
        bool first = true;
        for (size_t i = 0; i < 8; ++i) {
            const uint8_t v = seg.keyEnums[i];
            if (v == kEnumTerminator) break;
            if (!first) out += ',';
            out += valueName(v);
            first = false;
        }
        if (first) out += "<empty>"; // all-terminator payload
    } else {
        // Default to the int-range view for unflagged tags too — that's
        // how stock Thief 2 stores most non-enum keys (object-ID ranges,
        // numeric thresholds, etc.).
        char buf[64];
        std::snprintf(buf, sizeof(buf), "=[%d..%d]", seg.keyMin, seg.keyMax);
        out += buf;
    }
    return out;
}

std::string SpeechDatabase::formatKeyPath(
    const std::vector<KeySegment> &path) const
{
    if (path.empty()) return "<root>";
    std::string out;
    for (size_t i = 0; i < path.size(); ++i) {
        if (i) out += ", ";
        out += formatKey(path[i]);
    }
    return out;
}

void SpeechDatabase::dump(std::ostream &out) const {
    if (!mLoaded) {
        out << "SpeechDatabase: <not loaded>\n";
        return;
    }
    out << "SpeechDatabase: " << mRaw.size() << " bytes  "
        << "concepts=" << mConcepts.size()
        << " tags=" << mTags.size()
        << " values=" << mValues.size()
        << " voices=" << mVoiceCount
        << " entries=" << mEntries.size()
        << " tail=" << mTail.size() << '\n';

    // Hex peek of the first 16 raw bytes. snprintf used for portable,
    // locale-free hex output (matches the previous formatter's style).
    const std::vector<uint8_t> head = headerBytes(16);
    out << "  header[0.." << head.size() << "):";
    char buf[8];
    for (size_t i = 0; i < head.size(); ++i) {
        if (i == 8) out << " ";
        std::snprintf(buf, sizeof(buf), " %02x", head[i]);
        out << buf;
    }
    out << "\n";
}

} // namespace Darkness
