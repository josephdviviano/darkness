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
 *****************************************************************************/

#include <catch2/catch_test_macros.hpp>
#include "audio/SchemaParser.h"

using namespace Darkness;

// ════════════════════════════════════════════════════════════════════════════
// 34a: Basic data structure tests
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("SchemaEntry defaults", "[schema][types]") {
    SchemaEntry entry;
    CHECK(entry.playParams.volume == -1);
    CHECK(entry.playParams.priority == 128);
    CHECK(entry.playParams.audioClass == SchemaAudioClass::Noise);
    CHECK(entry.loopParams.isLooping == false);
    CHECK(entry.samples.empty());
    CHECK(entry.totalFrequency() == 0);
    CHECK_FALSE(entry.hasEnvTags());
    CHECK_FALSE(entry.hasVoice());
}

TEST_CASE("SchemaSample defaults", "[schema][types]") {
    SchemaSample sample;
    CHECK(sample.frequency == 1);
    CHECK(sample.name.empty());
    CHECK(sample.text.empty());
}

// ════════════════════════════════════════════════════════════════════════════
// 34b/c: Parser — basic schema parsing
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Parse empty schema", "[schema][parser]") {
    SchemaParser parser;
    REQUIRE(parser.parseString("schema empty_test"));
    CHECK(parser.schemaCount() == 1);
    auto *s = parser.findSchema("empty_test");
    REQUIRE(s != nullptr);
    CHECK(s->name == "empty_test");
    CHECK(s->samples.empty());
}

TEST_CASE("Parse schema with volume", "[schema][parser]") {
    SchemaParser parser;
    REQUIRE(parser.parseString("schema loud_sound\nvolume -500\n"));
    auto *s = parser.findSchema("loud_sound");
    REQUIRE(s != nullptr);
    CHECK(s->playParams.volume == -500);
}

TEST_CASE("Parse schema with all play params", "[schema][parser]") {
    SchemaParser parser;
    REQUIRE(parser.parseString(R"(
        schema test_full
        volume -2000
        delay 500
        pan -3000
        priority 200
        fade 100
        audio_class ambient
        no_repeat
        stream
    )"));

    auto *s = parser.findSchema("test_full");
    REQUIRE(s != nullptr);
    CHECK(s->playParams.volume == -2000);
    CHECK(s->playParams.initialDelay == 500);
    CHECK(s->playParams.pan == -3000);
    CHECK(s->playParams.priority == 200);
    CHECK(s->playParams.fade == 100);
    CHECK(s->playParams.audioClass == SchemaAudioClass::Ambient);
    CHECK((s->playParams.flags & SCH_NO_REPEAT) != 0);
    CHECK((s->playParams.flags & SCH_STREAM) != 0);
    CHECK((s->playParams.flags & SCH_PAN_POS) != 0);
}

TEST_CASE("Parse schema with samples", "[schema][parser]") {
    SchemaParser parser;
    REQUIRE(parser.parseString(R"(
        schema footstep_stone
        volume -1000
        swoosh1
        swoosh2
        swoosh3 freq 5
    )"));

    auto *s = parser.findSchema("footstep_stone");
    REQUIRE(s != nullptr);
    REQUIRE(s->samples.size() == 3);
    CHECK(s->samples[0].name == "swoosh1");
    CHECK(s->samples[0].frequency == 1);
    CHECK(s->samples[1].name == "swoosh2");
    CHECK(s->samples[2].name == "swoosh3");
    CHECK(s->samples[2].frequency == 5);
}

TEST_CASE("Parse schema with sample text", "[schema][parser]") {
    SchemaParser parser;
    REQUIRE(parser.parseString(R"(
        schema guard_hello
        volume -800
        greet1 "Hey you there!"
        greet2 "Who goes there?" freq 3
    )"));

    auto *s = parser.findSchema("guard_hello");
    REQUIRE(s != nullptr);
    REQUIRE(s->samples.size() == 2);
    CHECK(s->samples[0].text == "Hey you there!");
    CHECK(s->samples[1].text == "Who goes there?");
    CHECK(s->samples[1].frequency == 3);
}

TEST_CASE("Parse mono_loop", "[schema][parser]") {
    SchemaParser parser;
    REQUIRE(parser.parseString(R"(
        schema ambient_drip
        mono_loop 2000 5000
        drip1
        drip2
    )"));

    auto *s = parser.findSchema("ambient_drip");
    REQUIRE(s != nullptr);
    CHECK(s->loopParams.isLooping == true);
    CHECK(s->loopParams.isPoly == false);
    CHECK(s->loopParams.maxSamples == 1);
    CHECK(s->loopParams.intervalMin == 2000);
    CHECK(s->loopParams.intervalMax == 5000);
}

TEST_CASE("Parse poly_loop", "[schema][parser]") {
    SchemaParser parser;
    REQUIRE(parser.parseString(R"(
        schema ambient_birds
        poly_loop 3 1000 3000
        loop_count 10
        bird1
        bird2
    )"));

    auto *s = parser.findSchema("ambient_birds");
    REQUIRE(s != nullptr);
    CHECK(s->loopParams.isLooping == true);
    CHECK(s->loopParams.isPoly == true);
    CHECK(s->loopParams.maxSamples == 3);
    CHECK(s->loopParams.intervalMin == 1000);
    CHECK(s->loopParams.intervalMax == 3000);
    CHECK(s->loopParams.count == 10);
}

// ════════════════════════════════════════════════════════════════════════════
// 34c: Parser — comments and whitespace
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Parser handles comments", "[schema][parser]") {
    SchemaParser parser;
    REQUIRE(parser.parseString(R"(
        // This is a comment
        schema test_comments
        volume -100 // inline comment
        sample1
    )"));

    CHECK(parser.schemaCount() == 1);
    auto *s = parser.findSchema("test_comments");
    REQUIRE(s != nullptr);
    CHECK(s->playParams.volume == -100);
    CHECK(s->samples.size() == 1);
}

// ════════════════════════════════════════════════════════════════════════════
// 34c: Parser — multiple schemas in one file
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Parse multiple schemas", "[schema][parser]") {
    SchemaParser parser;
    REQUIRE(parser.parseString(R"(
        schema first
        volume -100
        sample_a

        schema second
        volume -200
        sample_b
        sample_c
    )"));

    CHECK(parser.schemaCount() == 2);
    auto *s1 = parser.findSchema("first");
    auto *s2 = parser.findSchema("second");
    REQUIRE(s1 != nullptr);
    REQUIRE(s2 != nullptr);
    CHECK(s1->playParams.volume == -100);
    CHECK(s1->samples.size() == 1);
    CHECK(s2->playParams.volume == -200);
    CHECK(s2->samples.size() == 2);
}

// ════════════════════════════════════════════════════════════════════════════
// 34d: Archetype inheritance
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Archetype inheritance — basic", "[schema][inheritance]") {
    SchemaParser parser;
    REQUIRE(parser.parseString(R"(
        schema AMBIENTS
        audio_class ambient
        volume -1

        schema drip_ambient
        archetype AMBIENTS
        mono_loop 1000 3000
        drip1
    )"));

    auto *s = parser.findSchema("drip_ambient");
    REQUIRE(s != nullptr);
    CHECK(s->playParams.audioClass == SchemaAudioClass::Ambient);
    CHECK(s->playParams.volume == -1);
    CHECK(s->loopParams.isLooping == true);
    CHECK(s->samples.size() == 1);
}

TEST_CASE("Archetype inheritance — child overrides parent", "[schema][inheritance]") {
    SchemaParser parser;
    REQUIRE(parser.parseString(R"(
        schema BASE
        volume -500
        priority 64

        schema CHILD
        archetype BASE
        volume -800
    )"));

    auto *s = parser.findSchema("CHILD");
    REQUIRE(s != nullptr);
    CHECK(s->playParams.volume == -800);  // child's explicit value
    CHECK(s->playParams.priority == 64);  // inherited from parent
}

TEST_CASE("Archetype inheritance — flags merge", "[schema][inheritance]") {
    SchemaParser parser;
    REQUIRE(parser.parseString(R"(
        schema PARENT
        no_repeat
        no_cache

        schema CHILD
        archetype PARENT
        stream
    )"));

    auto *s = parser.findSchema("CHILD");
    REQUIRE(s != nullptr);
    CHECK((s->playParams.flags & SCH_NO_REPEAT) != 0);
    CHECK((s->playParams.flags & SCH_NO_CACHE) != 0);
    CHECK((s->playParams.flags & SCH_STREAM) != 0);
}

// ════════════════════════════════════════════════════════════════════════════
// 34e: Tag and env_tag definitions
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Parse tag definition", "[schema][tags]") {
    SchemaParser parser;
    REQUIRE(parser.parseString(R"(
        tag Material Stone Metal Wood Carpet Tile Earth Gravel
    )"));

    CHECK(parser.tagCount() == 1);
    auto *t = parser.findTag("Material");
    REQUIRE(t != nullptr);
    CHECK(t->name == "Material");
    CHECK(t->values.size() == 7);
    CHECK(t->values[0] == "Stone");
    CHECK(t->values[1] == "Metal");
    CHECK(t->isIntTag == false);
}

TEST_CASE("Parse tag_int definition", "[schema][tags]") {
    SchemaParser parser;
    REQUIRE(parser.parseString("tag_int NearbyFriends"));

    auto *t = parser.findTag("NearbyFriends");
    REQUIRE(t != nullptr);
    CHECK(t->isIntTag == true);
    CHECK(t->values.empty());
}

TEST_CASE("Parse env_tag on schema", "[schema][tags]") {
    SchemaParser parser;
    REQUIRE(parser.parseString(R"(
        schema footstep_stone_player
        volume -1000
        step_stone1
        step_stone2
        env_tag (Event Footstep) (Material Stone) (CreatureType Player)
    )"));

    auto *s = parser.findSchema("footstep_stone_player");
    REQUIRE(s != nullptr);
    CHECK(s->hasEnvTags());
    REQUIRE(s->envTags.size() == 3);
    CHECK(s->envTags[0].tagName == "Event");
    CHECK(s->envTags[0].enumValues.size() == 1);
    CHECK(s->envTags[0].enumValues[0] == "Footstep");
    CHECK(s->envTags[1].tagName == "Material");
    CHECK(s->envTags[1].enumValues[0] == "Stone");
    CHECK(s->envTags[2].tagName == "CreatureType");
    CHECK(s->envTags[2].enumValues[0] == "Player");
}

TEST_CASE("Parse env_tag with integer range", "[schema][tags]") {
    SchemaParser parser;
    REQUIRE(parser.parseString(R"(
        schema range_test
        volume -500
        sample1
        env_tag (Speed 0 100)
    )"));

    auto *s = parser.findSchema("range_test");
    REQUIRE(s != nullptr);
    REQUIRE(s->envTags.size() == 1);
    CHECK(s->envTags[0].tagName == "Speed");
    CHECK(s->envTags[0].isIntRange == true);
    CHECK(s->envTags[0].rangeMin == 0);
    CHECK(s->envTags[0].rangeMax == 100);
}

TEST_CASE("findByEnvTags matching", "[schema][tags]") {
    SchemaParser parser;
    REQUIRE(parser.parseString(R"(
        schema footstep_stone
        step_stone1
        env_tag (Event Footstep) (Material Stone)

        schema footstep_metal
        step_metal1
        env_tag (Event Footstep) (Material Metal)

        schema ambient_wind
        wind1
        env_tag (Event Ambient) (Location Outdoor)
    )"));

    // Query for footstep + stone
    std::vector<SchemaTagValue> query;
    query.push_back({"Event", {"Footstep"}, 0, 0, false});
    query.push_back({"Material", {"Stone"}, 0, 0, false});

    auto matches = parser.findByEnvTags(query);
    REQUIRE(matches.size() == 1);
    CHECK(matches[0]->name == "footstep_stone");
}

// ════════════════════════════════════════════════════════════════════════════
// 34e: Voice and concept definitions
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Parse voice definition", "[schema][voice]") {
    SchemaParser parser;
    REQUIRE(parser.parseString(R"(
        voice vguard1
        voice vguard2 archetype vguard1
    )"));

    CHECK(parser.voiceCount() == 2);
}

TEST_CASE("Parse concept definition", "[schema][voice]") {
    SchemaParser parser;
    REQUIRE(parser.parseString("concept atlevelone 5"));
    CHECK(parser.conceptCount() == 1);
}

TEST_CASE("Parse schema_voice", "[schema][voice]") {
    SchemaParser parser;
    REQUIRE(parser.parseString(R"(
        schema guard_greeting
        volume -800
        greet1

        schema_voice vguard1 5 atlevelone (Sense Sight) (NearbyFriends 0 2)
    )"));

    auto *s = parser.findSchema("guard_greeting");
    REQUIRE(s != nullptr);
    CHECK(s->hasVoice());
    CHECK(s->voiceName == "vguard1");
    CHECK(s->voiceWeight == 5);
    CHECK(s->conceptName == "atlevelone");
    REQUIRE(s->voiceTags.size() == 2);
    CHECK(s->voiceTags[0].tagName == "Sense");
    CHECK(s->voiceTags[0].enumValues[0] == "Sight");
    CHECK(s->voiceTags[1].tagName == "NearbyFriends");
    CHECK(s->voiceTags[1].isIntRange == true);
    CHECK(s->voiceTags[1].rangeMin == 0);
    CHECK(s->voiceTags[1].rangeMax == 2);
}

// ════════════════════════════════════════════════════════════════════════════
// 34c: Parser — #define constants
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Parse #define substitution", "[schema][parser]") {
    SchemaParser parser;
    REQUIRE(parser.parseString(R"(
        #define LOUD_VOL -500
        #define HIGH_PRI 200

        schema defined_test
        volume LOUD_VOL
        priority HIGH_PRI
        sample1
    )"));

    auto *s = parser.findSchema("defined_test");
    REQUIRE(s != nullptr);
    CHECK(s->playParams.volume == -500);
    CHECK(s->playParams.priority == 200);
}

// ════════════════════════════════════════════════════════════════════════════
// 34i: Edge cases
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Empty input produces no schemas", "[schema][edge]") {
    SchemaParser parser;
    REQUIRE(parser.parseString(""));
    CHECK(parser.schemaCount() == 0);
}

TEST_CASE("Comments only produces no schemas", "[schema][edge]") {
    SchemaParser parser;
    REQUIRE(parser.parseString("// just comments\n// nothing here\n"));
    CHECK(parser.schemaCount() == 0);
}

TEST_CASE("Max 20 samples per schema", "[schema][edge]") {
    SchemaParser parser;
    std::string text = "schema max_samples\n";
    for (int i = 0; i < 25; i++)
        text += "sample_" + std::to_string(i) + "\n";

    REQUIRE(parser.parseString(text));
    auto *s = parser.findSchema("max_samples");
    REQUIRE(s != nullptr);
    CHECK(s->samples.size() == 20);  // Capped at SCHEMA_SAMPLES_MAX
}

TEST_CASE("Case-insensitive schema lookup", "[schema][edge]") {
    SchemaParser parser;
    REQUIRE(parser.parseString("schema MySchema\nvolume -100\n"));

    CHECK(parser.findSchema("myschema") != nullptr);
    CHECK(parser.findSchema("MYSCHEMA") != nullptr);
    CHECK(parser.findSchema("MySchema") != nullptr);
}

TEST_CASE("All audio class names", "[schema][parser]") {
    SchemaParser parser;
    const char *classes[] = {
        "noise", "speech", "ambient", "music", "metaui",
        "player_feet", "other_feet", "collisions", "weapons", "monsters"
    };

    for (int i = 0; i < 10; i++) {
        std::string text = "schema test_class_" + std::to_string(i) +
                           "\naudio_class " + classes[i] + "\n";
        REQUIRE(parser.parseString(text));
    }

    CHECK(parser.schemaCount() == 10);
    auto *s0 = parser.findSchema("test_class_0");
    REQUIRE(s0 != nullptr);
    CHECK(s0->playParams.audioClass == SchemaAudioClass::Noise);

    auto *s2 = parser.findSchema("test_class_2");
    REQUIRE(s2 != nullptr);
    CHECK(s2->playParams.audioClass == SchemaAudioClass::Ambient);

    auto *s5 = parser.findSchema("test_class_5");
    REQUIRE(s5 != nullptr);
    CHECK(s5->playParams.audioClass == SchemaAudioClass::PlayerFeet);
}

TEST_CASE("All boolean flags", "[schema][parser]") {
    SchemaParser parser;
    REQUIRE(parser.parseString(R"(
        schema all_flags
        no_repeat
        no_cache
        stream
        play_once
        no_combat
        net_ambient
        local_spatial
    )"));

    auto *s = parser.findSchema("all_flags");
    REQUIRE(s != nullptr);
    CHECK((s->playParams.flags & SCH_NO_REPEAT) != 0);
    CHECK((s->playParams.flags & SCH_NO_CACHE) != 0);
    CHECK((s->playParams.flags & SCH_STREAM) != 0);
    CHECK((s->playParams.flags & SCH_PLAY_ONCE) != 0);
    CHECK((s->playParams.flags & SCH_NO_COMBAT) != 0);
    CHECK((s->playParams.flags & SCH_NET_AMBIENT) != 0);
    CHECK((s->playParams.flags & SCH_LOC_SPATIAL) != 0);
}

TEST_CASE("Pan vs pan_range flags", "[schema][parser]") {
    SchemaParser parser;
    REQUIRE(parser.parseString(R"(
        schema pan_test
        pan -5000

        schema pan_range_test
        pan_range 3000
    )"));

    auto *sp = parser.findSchema("pan_test");
    REQUIRE(sp != nullptr);
    CHECK((sp->playParams.flags & SCH_PAN_POS) != 0);
    CHECK((sp->playParams.flags & SCH_PAN_RANGE) == 0);

    auto *sr = parser.findSchema("pan_range_test");
    REQUIRE(sr != nullptr);
    CHECK((sr->playParams.flags & SCH_PAN_POS) == 0);
    CHECK((sr->playParams.flags & SCH_PAN_RANGE) != 0);
}

TEST_CASE("env_tag_required", "[schema][tags]") {
    SchemaParser parser;
    REQUIRE(parser.parseString(R"(
        tag Event Footstep Ambient Collision
        env_tag_required Event
    )"));

    auto *t = parser.findTag("Event");
    REQUIRE(t != nullptr);
    CHECK(t->isRequired == true);
}

// ════════════════════════════════════════════════════════════════════════════
// 34f: Parse BASE.ARC from Thief 2 disc (if available)
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Parse Thief 2 BASE.ARC", "[schema][thief2][!mayfail]") {
    // This test requires the Thief 2 disc to be mounted
    const std::string arcPath =
        "/Volumes/THIEF2_INSTALL_C/EDITOR/schemas/BASE.ARC";

    SchemaParser parser;
    bool loaded = parser.loadFile(arcPath);
    if (!loaded) {
        WARN("Thief 2 disc not mounted — skipping BASE.ARC test");
        SKIP();
    }

    // BASE.ARC defines archetype schemas (no samples, just param defaults)
    CHECK(parser.schemaCount() > 0);

    // Known archetypes from BASE.ARC
    CHECK(parser.findSchema("AMBIENTS") != nullptr);
    CHECK(parser.findSchema("AI_SPEECH") != nullptr);
}

TEST_CASE("Parse Thief 2 ENVSOUND.SPC", "[schema][thief2][!mayfail]") {
    const std::string spcPath =
        "/Volumes/THIEF2_INSTALL_C/EDITOR/schemas/ENVSOUND.SPC";

    SchemaParser parser;
    bool loaded = parser.loadFile(spcPath);
    if (!loaded) {
        WARN("Thief 2 disc not mounted — skipping ENVSOUND.SPC test");
        SKIP();
    }

    // ENVSOUND.SPC defines environmental sound tags
    CHECK(parser.tagCount() > 0);

    // Known tags
    CHECK(parser.findTag("Event") != nullptr);
    CHECK(parser.findTag("Material") != nullptr);
}

TEST_CASE("Parse Thief 2 SPEECH.SPC", "[schema][thief2][!mayfail]") {
    const std::string spcPath =
        "/Volumes/THIEF2_INSTALL_C/EDITOR/schemas/SPEECH.SPC";

    SchemaParser parser;
    bool loaded = parser.loadFile(spcPath);
    if (!loaded) {
        WARN("Thief 2 disc not mounted — skipping SPEECH.SPC test");
        SKIP();
    }

    // SPEECH.SPC defines voices and concepts
    CHECK(parser.voiceCount() > 0);
    CHECK(parser.conceptCount() > 0);
}
