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

#include <algorithm>
#include <string>

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

// ════════════════════════════════════════════════════════════════════════════
// Fix verification: unterminated string, multi-level inheritance, fieldsSet
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Unterminated string reports error", "[schema][parser]") {
    SchemaParser parser;
    // Missing closing quote — should produce an error
    bool ok = parser.parseString(R"(schema bad volume -100 "unterminated)");
    CHECK_FALSE(ok);
    REQUIRE(parser.errors().size() >= 1);

    // Check that the error message mentions unterminated string
    bool found = false;
    for (const auto &e : parser.errors()) {
        if (e.find("unterminated") != std::string::npos) {
            found = true;
            break;
        }
    }
    CHECK(found);
}

TEST_CASE("Multi-level archetype inheritance (A -> B -> C)", "[schema][inheritance]") {
    SchemaParser parser;
    REQUIRE(parser.parseString(R"(
        schema GRANDPARENT
        volume -100
        priority 32
        audio_class ambient
        no_repeat

        schema PARENT
        archetype GRANDPARENT
        fade 500

        schema CHILD
        archetype PARENT
        delay 200
        child_sample1
    )"));

    auto *s = parser.findSchema("CHILD");
    REQUIRE(s != nullptr);
    // Inherited from GRANDPARENT through PARENT
    CHECK(s->playParams.volume == -100);
    CHECK(s->playParams.priority == 32);
    CHECK(s->playParams.audioClass == SchemaAudioClass::Ambient);
    CHECK((s->playParams.flags & SCH_NO_REPEAT) != 0);
    // Inherited from PARENT
    CHECK(s->playParams.fade == 500);
    // Set directly on CHILD
    CHECK(s->playParams.initialDelay == 200);
    CHECK(s->samples.size() == 1);
}

TEST_CASE("Circular archetype reference does not infinite loop", "[schema][inheritance]") {
    SchemaParser parser;
    // A references B, B references A — parser must not hang
    REQUIRE(parser.parseString(R"(
        schema ALPHA
        archetype BETA
        volume -100

        schema BETA
        archetype ALPHA
        priority 64
    )"));

    // Both should exist and have their own explicit values
    auto *a = parser.findSchema("ALPHA");
    auto *b = parser.findSchema("BETA");
    REQUIRE(a != nullptr);
    REQUIRE(b != nullptr);
    CHECK(a->playParams.volume == -100);
    CHECK(b->playParams.priority == 64);
}

TEST_CASE("Inheritance respects explicitly-set default values", "[schema][inheritance]") {
    SchemaParser parser;
    REQUIRE(parser.parseString(R"(
        schema PARENT
        volume -500
        priority 200
        pan -3000
        fade 100

        schema CHILD_DEFAULTS
        archetype PARENT
        pan 0
        priority 128
    )"));

    auto *s = parser.findSchema("CHILD_DEFAULTS");
    REQUIRE(s != nullptr);

    // Child explicitly set pan=0 and priority=128 (both happen to be defaults).
    // These must NOT be overwritten by the parent's values.
    CHECK(s->playParams.pan == 0);
    CHECK(s->playParams.priority == 128);

    // These were not set by child, so they should be inherited from parent.
    CHECK(s->playParams.volume == -500);
    CHECK(s->playParams.fade == 100);
}

// ════════════════════════════════════════════════════════════════════════════
// 34g: Parse NewBridge fan mission .sch files (7 .sch + 2 .spc + 1 .arc)
// ════════════════════════════════════════════════════════════════════════════

#ifndef FIXTURES_DIR
#define FIXTURES_DIR "tests/fixtures"
#endif

static const std::string NEWBRIDGE_DIR =
    std::string(FIXTURES_DIR) + "/newbridge_schema";

TEST_CASE("NewBridge: loadDirectory succeeds with no errors", "[schema][newbridge]") {
    SchemaParser parser;
    bool ok = parser.loadDirectory(NEWBRIDGE_DIR);

    // Dump errors for debugging
    for (const auto &e : parser.errors())
        UNSCOPED_INFO("ERROR: " << e);
    for (const auto &w : parser.warnings())
        UNSCOPED_INFO("WARNING: " << w);

    REQUIRE(ok);
    CHECK(parser.errors().empty());
}

TEST_CASE("NewBridge: schema count", "[schema][newbridge]") {
    SchemaParser parser;
    REQUIRE(parser.loadDirectory(NEWBRIDGE_DIR));
    // 7 .sch files + 3 archetypes in m20.arc = total schemas
    // Count manually: m20.arc has 3 (AMB_M20, DEVICES_M20, M20_CONV)
    // The .sch files define the rest
    CHECK(parser.schemaCount() >= 200);
    // Exact count may vary — at least all are parsed
    INFO("Total schemas parsed: " << parser.schemaCount());
}

TEST_CASE("NewBridge: tags from ENVSOUND.SPC", "[schema][newbridge]") {
    SchemaParser parser;
    REQUIRE(parser.loadDirectory(NEWBRIDGE_DIR));

    // ENVSOUND.SPC defines key tags like Event, Material, etc.
    auto *eventTag = parser.findTag("Event");
    REQUIRE(eventTag != nullptr);
    CHECK_FALSE(eventTag->isIntTag);
    CHECK(eventTag->isRequired);
    // Event has: Launch Collision Footstep Damage StateChange Activate
    //            ActiveLoop Deactivate Death Select Reject MediaTrans Climbstep Motion Create
    CHECK(eventTag->values.size() >= 10);

    // Check a tag_int exists
    auto *launchVel = parser.findTag("LaunchVel");
    if (launchVel) {
        CHECK(launchVel->isIntTag);
    }
}

TEST_CASE("NewBridge: voices and concepts from SPEECH.SPC", "[schema][newbridge]") {
    SchemaParser parser;
    REQUIRE(parser.loadDirectory(NEWBRIDGE_DIR));

    CHECK(parser.voiceCount() > 0);
    CHECK(parser.conceptCount() > 0);

    INFO("Voices: " << parser.voiceCount() << ", Concepts: " << parser.conceptCount());
}

TEST_CASE("NewBridge: simple ambient schema (m20turb1)", "[schema][newbridge]") {
    SchemaParser parser;
    REQUIRE(parser.loadDirectory(NEWBRIDGE_DIR));

    auto *s = parser.findSchema("m20turb1");
    REQUIRE(s != nullptr);
    CHECK(s->archetypeName == "DEVICES_M20");
    CHECK(s->playParams.volume == -1000);
    CHECK(s->loopParams.isLooping == true);
    CHECK(s->loopParams.isPoly == false);
    CHECK(s->loopParams.intervalMin == 0);
    CHECK(s->loopParams.intervalMax == 0);
    REQUIRE(s->samples.size() == 1);
    CHECK(s->samples[0].name == "turbinlp");
}

TEST_CASE("NewBridge: multi-sample with no_repeat and delay (m20manintten)",
          "[schema][newbridge]") {
    SchemaParser parser;
    REQUIRE(parser.loadDirectory(NEWBRIDGE_DIR));

    auto *s = parser.findSchema("m20manintten");
    REQUIRE(s != nullptr);
    CHECK(s->archetypeName == "AMB_M20");
    CHECK(s->playParams.volume == -2500);
    CHECK(s->playParams.initialDelay == 8000);
    CHECK(s->playParams.flags & SCH_NO_REPEAT);
    CHECK(s->loopParams.isLooping == true);
    CHECK(s->loopParams.intervalMin == 10000);
    CHECK(s->loopParams.intervalMax == 20000);
    REQUIRE(s->samples.size() == 5);
    CHECK(s->samples[0].name == "gr3");
    CHECK(s->samples[4].name == "gr9");
}

TEST_CASE("NewBridge: conversation schema with schema_voice (nb000)",
          "[schema][newbridge]") {
    SchemaParser parser;
    REQUIRE(parser.loadDirectory(NEWBRIDGE_DIR));

    auto *s = parser.findSchema("nb000");
    REQUIRE(s != nullptr);
    CHECK(s->archetypeName == "M20_CONV");
    CHECK(s->playParams.volume == -1);
    CHECK(s->playParams.initialDelay == 500);
    REQUIRE(s->samples.size() == 1);
    CHECK(s->samples[0].name == "nb000");

    // schema_voice vgarrett 1 nbconv0 (LineNo 1 1)
    CHECK(s->voiceName == "vgarrett");
    CHECK(s->voiceWeight == 1);
    CHECK(s->conceptName == "nbconv0");
    REQUIRE(s->voiceTags.size() == 1);
    CHECK(s->voiceTags[0].tagName == "LineNo");
    CHECK(s->voiceTags[0].isIntRange == true);
    CHECK(s->voiceTags[0].rangeMin == 1);
    CHECK(s->voiceTags[0].rangeMax == 1);
}

TEST_CASE("NewBridge: env_tag schema (blackjack_ceram)", "[schema][newbridge]") {
    SchemaParser parser;
    REQUIRE(parser.loadDirectory(NEWBRIDGE_DIR));

    auto *s = parser.findSchema("blackjack_ceram");
    REQUIRE(s != nullptr);
    CHECK(s->archetypeName == "HIT_MATERIAL");
    CHECK(s->playParams.volume == -1);
    REQUIRE(s->samples.size() == 2);
    CHECK(s->samples[0].name == "ar_body1");
    CHECK(s->samples[1].name == "ar_body2");

    // env_tag (Event Collision) (WeaponType Blackjack) (Material Ceramic)
    REQUIRE(s->envTags.size() == 3);

    // Find each tag by name (order may vary)
    bool foundEvent = false, foundWeapon = false, foundMaterial = false;
    for (const auto &tag : s->envTags) {
        if (tag.tagName == "Event") {
            CHECK(tag.enumValues.size() == 1);
            CHECK(tag.enumValues[0] == "Collision");
            foundEvent = true;
        } else if (tag.tagName == "WeaponType") {
            CHECK(tag.enumValues.size() == 1);
            CHECK(tag.enumValues[0] == "Blackjack");
            foundWeapon = true;
        } else if (tag.tagName == "Material") {
            CHECK(tag.enumValues.size() == 1);
            CHECK(tag.enumValues[0] == "Ceramic");
            foundMaterial = true;
        }
    }
    CHECK(foundEvent);
    CHECK(foundWeapon);
    CHECK(foundMaterial);
}

TEST_CASE("NewBridge: door state change env_tag with multi-value enum",
          "[schema][newbridge]") {
    SchemaParser parser;
    REQUIRE(parser.loadDirectory(NEWBRIDGE_DIR));

    // Look for the door schema with multi-value OldOpenState
    // env_tag (Event StateChange) (DoorType Wood1cem) (OpenState Open)
    //         (OldOpenState Closed Opening Closing)
    // We need to find this schema by scanning for env_tags with OldOpenState
    const SchemaEntry *doorSchema = nullptr;
    for (const auto &[key, schema] : parser.schemas()) {
        for (const auto &tag : schema.envTags) {
            if (tag.tagName == "OldOpenState" && tag.enumValues.size() >= 3) {
                doorSchema = &schema;
                break;
            }
        }
        if (doorSchema) break;
    }

    REQUIRE(doorSchema != nullptr);
    INFO("Door schema: " << doorSchema->name);

    // Verify multi-value enum was parsed
    for (const auto &tag : doorSchema->envTags) {
        if (tag.tagName == "OldOpenState") {
            CHECK(tag.enumValues.size() == 3);
            // Should contain Closed, Opening, Closing
            CHECK(std::find(tag.enumValues.begin(), tag.enumValues.end(), "Closed")
                  != tag.enumValues.end());
            CHECK(std::find(tag.enumValues.begin(), tag.enumValues.end(), "Opening")
                  != tag.enumValues.end());
            CHECK(std::find(tag.enumValues.begin(), tag.enumValues.end(), "Closing")
                  != tag.enumValues.end());
        }
    }
}

TEST_CASE("NewBridge: archetype resolution (AMB_M20 -> AMB)", "[schema][newbridge]") {
    SchemaParser parser;
    REQUIRE(parser.loadDirectory(NEWBRIDGE_DIR));

    // m20.arc defines: AMB_M20 archetype AMB
    auto *archSchema = parser.findSchema("AMB_M20");
    REQUIRE(archSchema != nullptr);
    CHECK(archSchema->archetypeName == "AMB");
}

TEST_CASE("NewBridge: creature celebration schema_voice (ab1rcel)",
          "[schema][newbridge]") {
    SchemaParser parser;
    REQUIRE(parser.loadDirectory(NEWBRIDGE_DIR));

    auto *s = parser.findSchema("ab1rcel");
    REQUIRE(s != nullptr);
    CHECK(s->archetypeName == "AI_NONE");
    CHECK(s->playParams.flags & SCH_NO_REPEAT);
    CHECK(s->samples.size() == 5);
    CHECK(s->voiceName == "vape1");
    CHECK(s->voiceWeight == 1);
    CHECK(s->conceptName == "nbritcel");
}

TEST_CASE("NewBridge: all 205 schemas have names and archetypes",
          "[schema][newbridge]") {
    SchemaParser parser;
    REQUIRE(parser.loadDirectory(NEWBRIDGE_DIR));

    int namedCount = 0;
    int withArchetype = 0;
    for (const auto &[key, schema] : parser.schemas()) {
        if (!schema.name.empty()) namedCount++;
        if (!schema.archetypeName.empty()) withArchetype++;
    }

    CHECK(namedCount == parser.schemaCount());
    // All NewBridge schemas use archetypes
    CHECK(withArchetype == parser.schemaCount());
}

// ════════════════════════════════════════════════════════════════════════════
// 34h: Round-trip tests — parse → serialize → reparse
// ════════════════════════════════════════════════════════════════════════════

// Helper: compare two SchemaEntry structs field-by-field
static void checkSchemaEqual(const SchemaEntry &a, const SchemaEntry &b,
                              const std::string &name) {
    INFO("Comparing schema: " << name);

    // Names are compared case-insensitively since parser lowercases keys
    // but stores original names
    CHECK(a.archetypeName == b.archetypeName);

    // Play params — compare only explicitly-set fields
    CHECK(a.playParams.volume == b.playParams.volume);
    CHECK(a.playParams.pan == b.playParams.pan);
    CHECK(a.playParams.initialDelay == b.playParams.initialDelay);
    CHECK(a.playParams.fade == b.playParams.fade);
    CHECK(a.playParams.priority == b.playParams.priority);
    CHECK(a.playParams.audioClass == b.playParams.audioClass);
    CHECK(a.playParams.flags == b.playParams.flags);

    // Loop params
    CHECK(a.loopParams.isLooping == b.loopParams.isLooping);
    if (a.loopParams.isLooping) {
        CHECK(a.loopParams.isPoly == b.loopParams.isPoly);
        CHECK(a.loopParams.maxSamples == b.loopParams.maxSamples);
        CHECK(a.loopParams.intervalMin == b.loopParams.intervalMin);
        CHECK(a.loopParams.intervalMax == b.loopParams.intervalMax);
    }
    CHECK(a.loopParams.count == b.loopParams.count);

    // Samples
    REQUIRE(a.samples.size() == b.samples.size());
    for (size_t i = 0; i < a.samples.size(); ++i) {
        CHECK(a.samples[i].name == b.samples[i].name);
        CHECK(a.samples[i].frequency == b.samples[i].frequency);
        CHECK(a.samples[i].text == b.samples[i].text);
    }

    // Voice binding
    CHECK(a.voiceName == b.voiceName);
    CHECK(a.voiceWeight == b.voiceWeight);
    CHECK(a.conceptName == b.conceptName);
    REQUIRE(a.voiceTags.size() == b.voiceTags.size());
    for (size_t i = 0; i < a.voiceTags.size(); ++i) {
        CHECK(a.voiceTags[i].tagName == b.voiceTags[i].tagName);
        CHECK(a.voiceTags[i].isIntRange == b.voiceTags[i].isIntRange);
        if (a.voiceTags[i].isIntRange) {
            CHECK(a.voiceTags[i].rangeMin == b.voiceTags[i].rangeMin);
            CHECK(a.voiceTags[i].rangeMax == b.voiceTags[i].rangeMax);
        } else {
            CHECK(a.voiceTags[i].enumValues == b.voiceTags[i].enumValues);
        }
    }

    // Env tags
    REQUIRE(a.envTags.size() == b.envTags.size());
    for (size_t i = 0; i < a.envTags.size(); ++i) {
        CHECK(a.envTags[i].tagName == b.envTags[i].tagName);
        CHECK(a.envTags[i].isIntRange == b.envTags[i].isIntRange);
        if (a.envTags[i].isIntRange) {
            CHECK(a.envTags[i].rangeMin == b.envTags[i].rangeMin);
            CHECK(a.envTags[i].rangeMax == b.envTags[i].rangeMax);
        } else {
            CHECK(a.envTags[i].enumValues == b.envTags[i].enumValues);
        }
    }

    // Message
    CHECK(a.message == b.message);
}

TEST_CASE("Round-trip: simple schema", "[schema][roundtrip]") {
    SchemaParser parser;
    REQUIRE(parser.parseString(R"(
        schema test_basic
        archetype SomeParent
        volume -500
        delay 100
        mono_loop 1000 2000
        no_repeat
        sample1
        sample2 freq 3
    )"));

    std::string serialized = parser.serialize();
    INFO("Serialized:\n" << serialized);

    SchemaParser parser2;
    REQUIRE(parser2.parseString(serialized));

    auto *a = parser.findSchema("test_basic");
    auto *b = parser2.findSchema("test_basic");
    REQUIRE(a != nullptr);
    REQUIRE(b != nullptr);
    checkSchemaEqual(*a, *b, "test_basic");
}

TEST_CASE("Round-trip: all fields set", "[schema][roundtrip]") {
    SchemaParser parser;
    REQUIRE(parser.parseString(R"(
        tag TestTag val1 val2 val3
        tag_int TestInt
        env_tag_required TestTag

        voice TestVoice archetype BaseVoice
        concept testconcept 42

        schema allfields
        archetype SomeBase
        volume -3000
        delay 500
        pan_range 2000
        priority 200
        fade 100
        audio_class speech
        no_repeat
        no_cache
        stream
        play_once
        no_combat
        net_ambient
        local_spatial
        poly_loop 3 500 1500
        loop_count 5
        message test_msg
        samp1 "subtitle text" freq 4
        samp2
        schema_voice TestVoice 10 testconcept (TestTag val1 val2) (TestInt 0 100)
        env_tag (TestTag val3) (TestInt 50 75)
    )"));

    std::string serialized = parser.serialize();
    INFO("Serialized:\n" << serialized);

    SchemaParser parser2;
    REQUIRE(parser2.parseString(serialized));

    auto *a = parser.findSchema("allfields");
    auto *b = parser2.findSchema("allfields");
    REQUIRE(a != nullptr);
    REQUIRE(b != nullptr);
    checkSchemaEqual(*a, *b, "allfields");

    // Also verify tags/voices/concepts survived
    CHECK(parser2.tagCount() >= 2);
    CHECK(parser2.voiceCount() >= 1);
    CHECK(parser2.conceptCount() >= 1);

    auto *tag = parser2.findTag("TestTag");
    REQUIRE(tag != nullptr);
    CHECK(tag->values.size() == 3);
    CHECK(tag->isRequired);
}

TEST_CASE("Round-trip: archetype inheritance preserved", "[schema][roundtrip]") {
    SchemaParser parser;
    REQUIRE(parser.parseString(R"(
        schema PARENT
        volume -500
        priority 200

        schema CHILD
        archetype PARENT
        delay 100
    )"));

    std::string serialized = parser.serialize();
    SchemaParser parser2;
    REQUIRE(parser2.parseString(serialized));

    auto *child = parser2.findSchema("CHILD");
    REQUIRE(child != nullptr);
    CHECK(child->archetypeName == "PARENT");
    // After reparse + re-resolve, inherited values should match
    CHECK(child->playParams.volume == -500);
    CHECK(child->playParams.initialDelay == 100);
}

TEST_CASE("Round-trip: NewBridge full directory", "[schema][roundtrip][newbridge]") {
    SchemaParser parser;
    REQUIRE(parser.loadDirectory(NEWBRIDGE_DIR));

    std::string serialized = parser.serialize();
    INFO("Serialized length: " << serialized.size() << " bytes");

    SchemaParser parser2;
    REQUIRE(parser2.parseString(serialized));

    // Same number of schemas
    CHECK(parser2.schemaCount() == parser.schemaCount());

    // Compare every schema field-by-field
    int compared = 0;
    for (const auto &[key, schema] : parser.schemas()) {
        auto *reparsed = parser2.findSchema(schema.name);
        if (reparsed) {
            checkSchemaEqual(schema, *reparsed, schema.name);
            compared++;
        } else {
            FAIL("Schema '" << schema.name << "' missing after round-trip");
        }
    }

    INFO("Compared " << compared << " schemas successfully");
    CHECK(compared == (int)parser.schemaCount());
}

// ════════════════════════════════════════════════════════════════════════════
// 34j: Error cases — malformed input, missing fields, unknown keywords
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Error: unterminated string", "[schema][errors]") {
    SchemaParser parser;
    parser.parseString("schema foo\nvolume -1\n\"never closed");
    CHECK_FALSE(parser.errors().empty());
}

TEST_CASE("Error: missing schema name", "[schema][errors]") {
    SchemaParser parser;
    // 'schema' immediately followed by another keyword
    parser.parseString("schema\nschema bar\nvolume -1\nbaz");
    // The first 'schema' should fail (next token is 'schema', not identifier)
    CHECK_FALSE(parser.errors().empty());
}

TEST_CASE("Error: empty input produces no schemas", "[schema][errors]") {
    SchemaParser parser;
    bool ok = parser.parseString("");
    CHECK(ok);
    CHECK(parser.schemaCount() == 0);
    CHECK(parser.errors().empty());
}

TEST_CASE("Error: comments only produces no schemas", "[schema][errors]") {
    SchemaParser parser;
    bool ok = parser.parseString("// just comments\n// nothing here\n");
    CHECK(ok);
    CHECK(parser.schemaCount() == 0);
    CHECK(parser.errors().empty());
}

TEST_CASE("Error: unknown top-level token produces warning", "[schema][errors]") {
    SchemaParser parser;
    parser.parseString("gobbledygook 42\nschema foo\nvolume -1\nbar");
    // 'gobbledygook' is unknown at top level — should be a warning
    CHECK_FALSE(parser.warnings().empty());
    // But the valid schema should still parse
    CHECK(parser.schemaCount() >= 1);
}

TEST_CASE("Error: duplicate schema name (last wins)", "[schema][errors]") {
    SchemaParser parser;
    parser.parseString(R"(
        schema dupe
        volume -100
        samp1

        schema dupe
        volume -200
        samp2
    )");

    auto *s = parser.findSchema("dupe");
    REQUIRE(s != nullptr);
    // Second definition overwrites first
    CHECK(s->playParams.volume == -200);
    REQUIRE(s->samples.size() == 1);
    CHECK(s->samples[0].name == "samp2");
}

TEST_CASE("Error: tag with no values is valid", "[schema][errors]") {
    SchemaParser parser;
    // A tag with no enum values — this is technically valid (empty set)
    bool ok = parser.parseString("tag EmptyTag\nschema foo\nbar");
    CHECK(ok);
    auto *tag = parser.findTag("EmptyTag");
    REQUIRE(tag != nullptr);
    CHECK(tag->values.empty());
    CHECK_FALSE(tag->isIntTag);
}

TEST_CASE("Error: negative volume is valid", "[schema][errors]") {
    SchemaParser parser;
    bool ok = parser.parseString("schema neg\nvolume -10000\nsample1");
    CHECK(ok);
    auto *s = parser.findSchema("neg");
    REQUIRE(s != nullptr);
    CHECK(s->playParams.volume == -10000);
}

TEST_CASE("Error: sample with quoted text preserved", "[schema][errors]") {
    SchemaParser parser;
    bool ok = parser.parseString(R"(
        schema dialog
        samp1 "Hello world"
        samp2 "Goodbye" freq 5
    )");
    CHECK(ok);

    auto *s = parser.findSchema("dialog");
    REQUIRE(s != nullptr);
    REQUIRE(s->samples.size() == 2);
    CHECK(s->samples[0].text == "Hello world");
    CHECK(s->samples[1].text == "Goodbye");
    CHECK(s->samples[1].frequency == 5);
}
