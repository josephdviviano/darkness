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

#include "SchemaSamplesChunk.h"

#include <cstring>

namespace Darkness {

namespace {

// Sanity limits — anything beyond these is almost certainly a parse-offset
// error, not real data. Schema sample names in Dark Engine are 8.3-style
// short identifiers (e.g. "stnft1", "fs_metal3"); 64 bytes is a generous
// upper bound. Number-of-samples is bounded in the engine's text parser
// to SCHEMA_SAMPLES_MAX (20); 256 here is a loose upper bound that still
// flags obviously-bogus values without being too restrictive.
constexpr int32_t kMaxStrLen = 64;
constexpr int32_t kMaxNumSamples = 256;

// Read a little-endian int32 from `data[off..off+4)`. Assumes the host is
// little-endian (true for all supported platforms: macOS arm64/x64, Linux
// x64, Windows x64). Caller must bounds-check.
inline int32_t readI32(const uint8_t *data, size_t off) {
    int32_t v;
    std::memcpy(&v, data + off, sizeof(v));
    return v;
}

// Heuristic: does this look like a plausible objID for a schema archetype?
// Dark Engine archetype IDs are negative (concrete=positive, archetype=
// negative), with absolute values typically in the thousands. A zero or
// implausibly large value at offset 0 strongly suggests a chunk-version
// prefix has shifted the real records by 4 bytes.
inline bool plausibleObjID(int32_t id) {
    // 0 is never a valid object ID; absolute value > 1,000,000 is well
    // beyond any reasonable archetype ID.
    if (id == 0) return false;
    int64_t abs = id < 0 ? -static_cast<int64_t>(id) : id;
    if (abs > 1000000) return false;
    return true;
}

} // namespace

bool SchemaSamplesChunk::tryParse(const uint8_t *data, size_t size, size_t start,
                                  std::vector<SchSampRecord> &outRecords) {
    outRecords.clear();
    size_t off = start;

    while (off < size) {
        // Each record needs at least 8 bytes for objID + num_samples.
        if (off + 8 > size) {
            return false; // mid-record truncation
        }

        SchSampRecord rec;
        rec.objID = readI32(data, off);
        off += 4;
        int32_t numSamples = readI32(data, off);
        off += 4;

        if (numSamples < 0 || numSamples > kMaxNumSamples) {
            return false;
        }

        rec.samples.reserve(static_cast<size_t>(numSamples));

        for (int32_t i = 0; i < numSamples; ++i) {
            if (off + 4 > size) return false;
            int32_t strLen = readI32(data, off);
            off += 4;

            // strLen of 0 is suspicious but technically representable; we
            // require at least 1 because real sample names are non-empty.
            if (strLen < 1 || strLen > kMaxStrLen) return false;

            // Need strLen bytes of name + 1 byte of freq.
            if (off + static_cast<size_t>(strLen) + 1 > size) return false;

            SchSampSample s;
            s.name.assign(reinterpret_cast<const char *>(data + off),
                          static_cast<size_t>(strLen));
            off += static_cast<size_t>(strLen);
            s.frequency = data[off];
            off += 1;

            rec.samples.push_back(std::move(s));
        }

        outRecords.push_back(std::move(rec));
    }

    return true;
}

bool SchemaSamplesChunk::loadFromChunk(const uint8_t *data, size_t size) {
    mRecords.clear();
    mLoaded = false;

    if (data == nullptr || size < 8) {
        return false;
    }

    // Heuristic for the optional 4-byte chunk-version prefix:
    //   1. Peek at the first int32 as if it were the objID of record 0.
    //   2. If it's plausible (nonzero, not absurdly large), parse from
    //      offset 0.
    //   3. Otherwise assume a version prefix and parse from offset 4.
    // This matches the documented behavior: chunks may or may not include
    // the prefix depending on how they were authored.
    int32_t firstWord = readI32(data, 0);
    std::vector<SchSampRecord> parsed;

    if (plausibleObjID(firstWord)) {
        if (tryParse(data, size, 0, parsed)) {
            mRecords = std::move(parsed);
            mLoaded = true;
            return true;
        }
        // Fall through and try with the version-prefix offset in case the
        // first parse happened to start at a plausible-looking byte pattern
        // but failed mid-record.
    }

    if (size >= 4) {
        parsed.clear();
        if (tryParse(data, size, 4, parsed)) {
            mRecords = std::move(parsed);
            mLoaded = true;
            return true;
        }
    }

    mRecords.clear();
    return false;
}

void SchemaSamplesChunk::dump(std::ostream &out) const {
    out << "SchSamp: " << mRecords.size() << " records\n";
    for (const auto &rec : mRecords) {
        out << "  objID=" << rec.objID
            << "  samples=" << rec.samples.size() << "\n";
        for (const auto &s : rec.samples) {
            out << "    " << s.name
                << "  freq=" << static_cast<int>(s.frequency) << "\n";
        }
    }
}

} // namespace Darkness
