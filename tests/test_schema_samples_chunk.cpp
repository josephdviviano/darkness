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

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#include "audio/SchemaSamplesChunk.h"

using namespace Darkness;

namespace {

// Append a little-endian int32 to `buf`.
void appendI32(std::vector<uint8_t> &buf, int32_t v) {
    uint8_t bytes[4];
    std::memcpy(bytes, &v, sizeof(v));
    buf.insert(buf.end(), bytes, bytes + 4);
}

// Append a single byte to `buf`.
void appendU8(std::vector<uint8_t> &buf, uint8_t v) { buf.push_back(v); }

// Append a sample triple: int32 strLen, raw name bytes (no null), uint8 freq.
void appendSample(std::vector<uint8_t> &buf, const std::string &name,
                  uint8_t freq) {
    appendI32(buf, static_cast<int32_t>(name.size()));
    buf.insert(buf.end(), name.begin(), name.end());
    appendU8(buf, freq);
}

// Append a record: int32 objID, int32 num_samples, then `samples` entries.
struct SampleSpec {
    std::string name;
    uint8_t freq;
};
void appendRecord(std::vector<uint8_t> &buf, int32_t objID,
                  const std::vector<SampleSpec> &samples) {
    appendI32(buf, objID);
    appendI32(buf, static_cast<int32_t>(samples.size()));
    for (const auto &s : samples)
        appendSample(buf, s.name, s.freq);
}

// Build a synthetic two-record buffer used by several tests.
std::vector<uint8_t> buildTwoRecordBuffer() {
    std::vector<uint8_t> buf;
    appendRecord(buf, -1234, {{"stnft1", 3}, {"stnft2", 5}});
    appendRecord(buf, -5678, {{"woodft1", 1}, {"woodft2", 2}, {"woodft3", 4}});
    return buf;
}

} // namespace

// ════════════════════════════════════════════════════════════════════════════
// Basic parse — two records, no version prefix
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("SchSamp: parse two records without version prefix",
          "[sch_samp][parser]") {
    auto buf = buildTwoRecordBuffer();

    SchemaSamplesChunk chunk;
    REQUIRE(chunk.loadFromChunk(buf.data(), buf.size()));
    CHECK(chunk.isLoaded());
    REQUIRE(chunk.recordCount() == 2);

    const auto &recs = chunk.records();
    CHECK(recs[0].objID == -1234);
    REQUIRE(recs[0].samples.size() == 2);
    CHECK(recs[0].samples[0].name == "stnft1");
    CHECK(recs[0].samples[0].frequency == 3);
    CHECK(recs[0].samples[1].name == "stnft2");
    CHECK(recs[0].samples[1].frequency == 5);

    CHECK(recs[1].objID == -5678);
    REQUIRE(recs[1].samples.size() == 3);
    CHECK(recs[1].samples[0].name == "woodft1");
    CHECK(recs[1].samples[0].frequency == 1);
    CHECK(recs[1].samples[1].name == "woodft2");
    CHECK(recs[1].samples[1].frequency == 2);
    CHECK(recs[1].samples[2].name == "woodft3");
    CHECK(recs[1].samples[2].frequency == 4);
}

// ════════════════════════════════════════════════════════════════════════════
// Version-prefix heuristic — same data, 4-byte version prefix prepended
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("SchSamp: skip 4-byte version prefix", "[sch_samp][parser]") {
    // Prepend a synthetic version int (1) that would look like a bogus
    // objID if interpreted as the start of record 0 — but it's small enough
    // (and the trailing bytes are large enough) that we still want the
    // parser to detect the prefix. To force the prefix path, we use a
    // version value of 0 which is never a valid objID.
    std::vector<uint8_t> bufWithPrefix;
    appendI32(bufWithPrefix, 0); // version=0, triggers prefix-skip heuristic
    auto inner = buildTwoRecordBuffer();
    bufWithPrefix.insert(bufWithPrefix.end(), inner.begin(), inner.end());

    SchemaSamplesChunk chunk;
    REQUIRE(chunk.loadFromChunk(bufWithPrefix.data(), bufWithPrefix.size()));
    REQUIRE(chunk.recordCount() == 2);

    const auto &recs = chunk.records();
    CHECK(recs[0].objID == -1234);
    CHECK(recs[1].objID == -5678);
    REQUIRE(recs[0].samples.size() == 2);
    REQUIRE(recs[1].samples.size() == 3);
    CHECK(recs[0].samples[0].name == "stnft1");
    CHECK(recs[1].samples[2].name == "woodft3");
}

// ════════════════════════════════════════════════════════════════════════════
// Bounds-checking — truncation in the middle of a record
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("SchSamp: truncated record returns isLoaded false",
          "[sch_samp][parser]") {
    auto buf = buildTwoRecordBuffer();
    // Lop off the last few bytes — the final sample is now incomplete.
    REQUIRE(buf.size() > 6);
    buf.resize(buf.size() - 6);

    SchemaSamplesChunk chunk;
    bool ok = chunk.loadFromChunk(buf.data(), buf.size());
    CHECK_FALSE(ok);
    CHECK_FALSE(chunk.isLoaded());
    CHECK(chunk.recordCount() == 0);
}

TEST_CASE("SchSamp: truncation before num_samples returns false",
          "[sch_samp][parser]") {
    // Only 6 bytes — not enough for objID(4) + num_samples(4).
    std::vector<uint8_t> buf;
    appendI32(buf, -42);
    appendU8(buf, 0);
    appendU8(buf, 0);

    SchemaSamplesChunk chunk;
    CHECK_FALSE(chunk.loadFromChunk(buf.data(), buf.size()));
    CHECK_FALSE(chunk.isLoaded());
}

TEST_CASE("SchSamp: implausibly large num_samples returns false",
          "[sch_samp][parser]") {
    std::vector<uint8_t> buf;
    appendI32(buf, -42);
    appendI32(buf, 1000000); // way over the 256 sanity bound

    SchemaSamplesChunk chunk;
    CHECK_FALSE(chunk.loadFromChunk(buf.data(), buf.size()));
}

TEST_CASE("SchSamp: implausibly large strLen returns false",
          "[sch_samp][parser]") {
    std::vector<uint8_t> buf;
    appendI32(buf, -42);
    appendI32(buf, 1);
    appendI32(buf, 5000); // sample name length way over kMaxStrLen=64

    SchemaSamplesChunk chunk;
    CHECK_FALSE(chunk.loadFromChunk(buf.data(), buf.size()));
}

TEST_CASE("SchSamp: zero-length name returns false", "[sch_samp][parser]") {
    std::vector<uint8_t> buf;
    appendI32(buf, -42);
    appendI32(buf, 1);
    appendI32(buf, 0); // strLen=0 — rejected (real names are non-empty)
    appendU8(buf, 1);

    SchemaSamplesChunk chunk;
    CHECK_FALSE(chunk.loadFromChunk(buf.data(), buf.size()));
}

TEST_CASE("SchSamp: empty buffer returns false", "[sch_samp][parser]") {
    SchemaSamplesChunk chunk;
    CHECK_FALSE(chunk.loadFromChunk(nullptr, 0));
    CHECK_FALSE(chunk.isLoaded());

    std::vector<uint8_t> tiny(2, 0);
    CHECK_FALSE(chunk.loadFromChunk(tiny.data(), tiny.size()));
}

// ════════════════════════════════════════════════════════════════════════════
// Record with zero samples is still valid
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("SchSamp: record with zero samples is valid", "[sch_samp][parser]") {
    std::vector<uint8_t> buf;
    appendRecord(buf, -99, {});
    appendRecord(buf, -100, {{"foo", 7}});

    SchemaSamplesChunk chunk;
    REQUIRE(chunk.loadFromChunk(buf.data(), buf.size()));
    REQUIRE(chunk.recordCount() == 2);
    CHECK(chunk.records()[0].objID == -99);
    CHECK(chunk.records()[0].samples.empty());
    CHECK(chunk.records()[1].objID == -100);
    REQUIRE(chunk.records()[1].samples.size() == 1);
    CHECK(chunk.records()[1].samples[0].name == "foo");
    CHECK(chunk.records()[1].samples[0].frequency == 7);
}

// ════════════════════════════════════════════════════════════════════════════
// dump() produces expected human-readable output
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("SchSamp: dump produces readable output", "[sch_samp][parser]") {
    auto buf = buildTwoRecordBuffer();
    SchemaSamplesChunk chunk;
    REQUIRE(chunk.loadFromChunk(buf.data(), buf.size()));

    std::ostringstream out;
    chunk.dump(out);
    const std::string s = out.str();

    CHECK(s.find("SchSamp: 2 records") != std::string::npos);
    CHECK(s.find("objID=-1234") != std::string::npos);
    CHECK(s.find("objID=-5678") != std::string::npos);
    CHECK(s.find("stnft1") != std::string::npos);
    CHECK(s.find("woodft3") != std::string::npos);
    CHECK(s.find("freq=3") != std::string::npos);
    CHECK(s.find("freq=4") != std::string::npos);
}
