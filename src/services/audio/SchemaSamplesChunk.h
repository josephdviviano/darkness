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

// Parser for the SchSamp chunk found in dark.gam. This chunk is the original
// engine's canonical schema-archetype -> samples mapping. We currently
// re-derive equivalent data from text .sch/.arc files via SchemaParser, but
// the SchSamp chunk is the authoritative on-disk binary form.
//
// Wire format (after an optional 4-byte chunk-version prefix at offset 0):
//
//   record:
//     int32_t  objID            // schema-archetype object ID
//     int32_t  num_samples
//     sample[num_samples]:
//       int32_t  strLen
//       char     name[strLen]   // sample name, NOT null-terminated
//       uint8_t  freq           // sample frequency / weight
//
// Records are concatenated until end-of-chunk; no overall record count.

#pragma once

#include <cstddef>
#include <cstdint>
#include <ostream>
#include <string>
#include <vector>

namespace Darkness {

struct SchSampSample {
    std::string name;
    uint8_t frequency = 0;
};

struct SchSampRecord {
    int32_t objID = 0;
    std::vector<SchSampSample> samples;
};

class SchemaSamplesChunk {
public:
    SchemaSamplesChunk() = default;

    /// Parse the raw chunk bytes (the full chunk payload, not the chunk header).
    /// Returns true on success. On any structural failure (truncation in the
    /// middle of a record, implausible field values) the parser returns false
    /// and isLoaded() remains false.
    bool loadFromChunk(const uint8_t *data, size_t size);

    bool isLoaded() const { return mLoaded; }

    const std::vector<SchSampRecord> &records() const { return mRecords; }
    size_t recordCount() const { return mRecords.size(); }

    /// Pretty-print all records to `out` in a human-readable format.
    void dump(std::ostream &out) const;

private:
    // Try to parse the buffer starting at byte offset `start`. Returns true
    // and fills `outRecords` on success; returns false on any structural
    // failure (truncation, out-of-range field).
    static bool tryParse(const uint8_t *data, size_t size, size_t start,
                         std::vector<SchSampRecord> &outRecords);

    std::vector<SchSampRecord> mRecords;
    bool mLoaded = false;
};

} // namespace Darkness
