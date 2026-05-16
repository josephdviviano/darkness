/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2024-2026 darkness contributors
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 3 of the License, or
 *    (at your option) any later version.
 *
 *****************************************************************************/

// Unit tests for the schema-property-overlay byte-decoders. Each test
// builds a synthetic record matching the on-disk dtype layout, runs it
// through the applySch* helper, and asserts the SchemaEntry receives
// the right field values. This verifies Unit A's on-disk mapping
// (PLAN.SOUND_DATA_PARSING.md) is correct.

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstdint>
#include <cstring>

#include "audio/SchemaPropertyOverrides.h"
#include "audio/SchemaTypes.h"
#include "property/DarkPropertyDefs.h"

using namespace Darkness;

namespace {

// Pack a PropSchemaPlayParams record into a 20-byte buffer in the same
// little-endian layout the property storage gives us.
std::array<uint8_t, sizeof(PropSchemaPlayParams)>
packPlayParams(uint16_t flags, uint16_t audioClass, int32_t volume,
               int32_t pan, uint32_t delay, int32_t fade)
{
    PropSchemaPlayParams pp{flags, audioClass, volume, pan, delay, fade};
    std::array<uint8_t, sizeof(PropSchemaPlayParams)> buf{};
    std::memcpy(buf.data(), &pp, sizeof(pp));
    return buf;
}

std::array<uint8_t, sizeof(PropSchemaLoopParams)>
packLoopParams(uint8_t flags, uint8_t maxSamples, int16_t loopCount,
               int16_t minInterval, int16_t maxInterval)
{
    PropSchemaLoopParams lp{flags, maxSamples, loopCount,
                            minInterval, maxInterval};
    std::array<uint8_t, sizeof(PropSchemaLoopParams)> buf{};
    std::memcpy(buf.data(), &lp, sizeof(lp));
    return buf;
}

} // namespace

TEST_CASE("PropSchemaPlayParams struct is 20 bytes", "[schema][overrides]") {
    // The on-disk dtype is 20 bytes (uint16+uint16+int32+int32+uint32+int32).
    // If this size changes our memcpy-based decoder will silently read
    // misaligned fields, so guard it here.
    CHECK(sizeof(PropSchemaPlayParams) == 20);
}

TEST_CASE("PropSchemaLoopParams struct is 8 bytes", "[schema][overrides]") {
    CHECK(sizeof(PropSchemaLoopParams) == 8);
}

TEST_CASE("PropSchPriori struct is 4 bytes", "[schema][overrides]") {
    CHECK(sizeof(PropSchPriori) == 4);
}

TEST_CASE("PropSchMsg struct is 16 bytes", "[schema][overrides]") {
    CHECK(sizeof(PropSchMsg) == 16);
}

TEST_CASE("mapDiskAudioClass remaps 1-based disk → 0-based enum",
          "[schema][overrides]") {
    SchemaAudioClass out;
    // Disk 0 = "None" → no override
    CHECK_FALSE(mapDiskAudioClass(0, out));
    // Disk 1 = "Noise" → internal Noise (0)
    REQUIRE(mapDiskAudioClass(1, out));
    CHECK(out == SchemaAudioClass::Noise);
    // Disk 2 = "Speech" → internal Speech (1)
    REQUIRE(mapDiskAudioClass(2, out));
    CHECK(out == SchemaAudioClass::Speech);
    // Disk 10 = "Monsters" → internal Monsters (9)
    REQUIRE(mapDiskAudioClass(10, out));
    CHECK(out == SchemaAudioClass::Monsters);
    // Out-of-range → no override
    CHECK_FALSE(mapDiskAudioClass(11, out));
    CHECK_FALSE(mapDiskAudioClass(0xFFFF, out));
}

TEST_CASE("applySchPlayParams overwrites all play fields",
          "[schema][overrides]") {
    SchemaEntry sch;
    // Synthetic: flags=0x7F00 (dtype default — SHARP enabled), Speech,
    // volume=-500, pan=-3000, delay=250, fade=100.
    auto rec = packPlayParams(0x7F00, 2 /* Speech */,
                              -500, -3000, 250, 100);
    applySchPlayParams(sch, rec.data());
    CHECK(sch.playParams.flags == 0x7F00u);
    CHECK((sch.playParams.flags & SCH_SHARP_FALLOFF) != 0);
    CHECK(sch.playParams.audioClass == SchemaAudioClass::Speech);
    CHECK(sch.playParams.volume == -500);
    CHECK(sch.playParams.pan == -3000);
    CHECK(sch.playParams.initialDelay == 250);
    CHECK(sch.playParams.fade == 100);
    // All "explicitly set" bits should be flipped on.
    CHECK((sch.playParams.fieldsSet & SCH_SET_VOLUME) != 0);
    CHECK((sch.playParams.fieldsSet & SCH_SET_DELAY) != 0);
    CHECK((sch.playParams.fieldsSet & SCH_SET_PAN) != 0);
    CHECK((sch.playParams.fieldsSet & SCH_SET_FADE) != 0);
    CHECK((sch.playParams.fieldsSet & SCH_SET_AUDIO_CLASS) != 0);
}

TEST_CASE("applySchPlayParams keeps default audioClass when disk class is 0",
          "[schema][overrides]") {
    SchemaEntry sch;
    sch.playParams.audioClass = SchemaAudioClass::Ambient;  // pre-set
    // Disk audio class 0 ("None") should leave audioClass alone.
    auto rec = packPlayParams(0x7F00, 0, -1, 0, 0, 0);
    applySchPlayParams(sch, rec.data());
    CHECK(sch.playParams.audioClass == SchemaAudioClass::Ambient);
}

TEST_CASE("applySchLoopParams: poly bit sets isLooping and isPoly",
          "[schema][overrides]") {
    SchemaEntry sch;
    // Poly bit (0x01) set, 4 max samples, 0 loop count, 500-1500 ms interval.
    auto rec = packLoopParams(0x01, 4, 0, 500, 1500);
    applySchLoopParams(sch, rec.data());
    CHECK(sch.loopParams.isLooping);
    CHECK(sch.loopParams.isPoly);
    CHECK(sch.loopParams.maxSamples == 4);
    CHECK(sch.loopParams.intervalMin == 500);
    CHECK(sch.loopParams.intervalMax == 1500);
}

TEST_CASE("applySchLoopParams: all-zero record leaves isLooping false",
          "[schema][overrides]") {
    SchemaEntry sch;
    auto rec = packLoopParams(0, 0, 0, 0, 0);
    applySchLoopParams(sch, rec.data());
    CHECK_FALSE(sch.loopParams.isLooping);
    CHECK_FALSE(sch.loopParams.isPoly);
    // maxSamples clamps to >= 1 even when the property record has 0.
    CHECK(sch.loopParams.maxSamples == 1);
}

TEST_CASE("applySchLoopParams: non-zero loopCount implies isLooping even "
          "without poly bit", "[schema][overrides]") {
    SchemaEntry sch;
    auto rec = packLoopParams(0x00, 1, 3, 100, 200);
    applySchLoopParams(sch, rec.data());
    CHECK(sch.loopParams.isLooping);
    CHECK_FALSE(sch.loopParams.isPoly);
    CHECK(sch.loopParams.count == 3);
}

TEST_CASE("applySchPriority overrides priority and sets the flag",
          "[schema][overrides]") {
    SchemaEntry sch;
    CHECK(sch.playParams.priority == 128);  // default
    PropSchPriori pri{255};  // Karras-style high priority
    applySchPriority(sch,
        reinterpret_cast<const uint8_t *>(&pri));
    CHECK(sch.playParams.priority == 255);
    CHECK((sch.playParams.fieldsSet & SCH_SET_PRIORITY) != 0);
}

TEST_CASE("applySchMessage copies the NUL-terminated label",
          "[schema][overrides]") {
    SchemaEntry sch;
    PropSchMsg m{};
    std::strncpy(m.label, "gotonoise", sizeof(m.label));
    applySchMessage(sch, reinterpret_cast<const uint8_t *>(&m));
    CHECK(sch.message == "gotonoise");
}

TEST_CASE("applySchMessage handles labels that fill the full 16 bytes",
          "[schema][overrides]") {
    SchemaEntry sch;
    PropSchMsg m{};
    // 16 chars with no terminator in-buffer — strnlen must cap at 16.
    std::memcpy(m.label, "abcdefghijklmnop", 16);
    applySchMessage(sch, reinterpret_cast<const uint8_t *>(&m));
    CHECK(sch.message.size() == 16);
    CHECK(sch.message == "abcdefghijklmnop");
}
