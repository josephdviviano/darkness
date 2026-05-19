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

// Unit tests for the multi-path ambisonics sub-source slot state machine.
// See SubSourceSlots.h for the design and PLAN.MULTI_PATH_AMBISONICS.md
// for the rollout plan. These tests exercise the main-thread slot
// assignment in isolation — no Steam Audio handles, no audio thread.

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>

#include "audio/SubSourceSlots.h"
#include "audio/VoicePool.h"
#include "room/RoomService.h"

using namespace Darkness;

namespace {

// Identity direction closure for tests: returns a sentinel IPLVector3
// encoding the path's predecessor so tests can verify the right path
// reached the right slot without doing any frame math.
IPLVector3 sentinelDir(const SoundPathRecord& p) {
    return IPLVector3{static_cast<float>(p.predecessorRoomID), 0.0f, 0.0f};
}

SoundPathRecord makePath(int32_t pred, float effDist, float door = 0.0f,
                         float realDist = -1.0f) {
    SoundPathRecord p;
    p.predecessorRoomID = pred;
    p.effectiveDistance = effDist;
    p.realDistance      = (realDist < 0.0f) ? effDist : realDist;
    p.doorBlocking      = door;
    p.virtualPosition   = Vector3{static_cast<float>(pred), 0.0f, 0.0f};
    return p;
}

SoundPropInfo propWith(std::initializer_list<SoundPathRecord> records) {
    SoundPropInfo prop;
    prop.reached = !std::empty(records);
    prop.paths.assign(records);
    if (!prop.paths.empty()) {
        prop.effectiveDistance = prop.paths.front().effectiveDistance;
        prop.realDistance      = prop.paths.front().realDistance;
        prop.doorBlocking      = prop.paths.front().doorBlocking;
        prop.virtualPosition   = prop.paths.front().virtualPosition;
    }
    return prop;
}

} // namespace

// ════════════════════════════════════════════════════════════════════════════
// Cold-start: empty slot array gets paths assigned in order
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("SubSourceSlots: cold start assigns paths to free slots",
          "[subsource][slots]") {
    std::array<SubSource, kMaxSubSources> slots{};
    SoundPropInfo prop = propWith({
        makePath(/*pred=*/5, /*effDist=*/10.0f),
        makePath(/*pred=*/9, /*effDist=*/14.0f),
    });

    updateSubSourceSlots(slots, prop, kMaxSubSources, sentinelDir);

    int activeCount = 0;
    for (const auto& s : slots) {
        if (s.state == SubSourceState::Active) ++activeCount;
    }
    CHECK(activeCount == 2);

    int s5 = findSlotByPredecessor(slots, 5);
    int s9 = findSlotByPredecessor(slots, 9);
    REQUIRE(s5 >= 0);
    REQUIRE(s9 >= 0);
    REQUIRE(s5 != s9);

    // On a cold (Free→Active) transition, currentDir is snapped to
    // targetDir so the binaural bilinear-interp prev/curr keys match
    // on the first callback. currentGain stays at 0 (ramps up on the
    // audio thread).
    CHECK(slots[s5].currentDir.x == 5.0f);
    CHECK(slots[s9].currentDir.x == 9.0f);
    CHECK(slots[s5].currentGain == 0.0f);
    CHECK(slots[s9].currentGain == 0.0f);
    CHECK(slots[s5].targetGain  == 1.0f);
    CHECK(slots[s9].targetGain  == 1.0f);
}

// ════════════════════════════════════════════════════════════════════════════
// Slot stability under BFS reorder — the entire reason for this design
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("SubSourceSlots: BFS reorder keeps each path in its original slot",
          "[subsource][slots][stability]") {
    std::array<SubSource, kMaxSubSources> slots{};

    // Frame 1: path A (pred=5) is closer, path B (pred=9) is farther.
    SoundPropInfo f1 = propWith({
        makePath(/*pred=*/5, /*effDist=*/10.0f),
        makePath(/*pred=*/9, /*effDist=*/14.0f),
    });
    updateSubSourceSlots(slots, f1, kMaxSubSources, sentinelDir);
    int slotA_initial = findSlotByPredecessor(slots, 5);
    int slotB_initial = findSlotByPredecessor(slots, 9);
    REQUIRE(slotA_initial >= 0);
    REQUIRE(slotB_initial >= 0);

    // Simulate the audio thread ramping currentGain to steady-state.
    // This is what the FREE→DRAINING→FREE cycle will need to observe
    // when the path drops out.
    for (auto& s : slots) {
        if (s.state == SubSourceState::Active) s.currentGain = 1.0f;
    }

    // Frame 2: door opens, path B becomes the closer path. SoundPath
    // records swap their sort order. The slot identity MUST follow
    // predecessorRoomID, not array index — otherwise per-slot HRTF
    // interp and LPF state get smeared across physically different
    // paths and we hear a click.
    SoundPropInfo f2 = propWith({
        makePath(/*pred=*/9, /*effDist=*/9.0f),
        makePath(/*pred=*/5, /*effDist=*/12.0f),
    });
    updateSubSourceSlots(slots, f2, kMaxSubSources, sentinelDir);

    CHECK(findSlotByPredecessor(slots, 5) == slotA_initial);
    CHECK(findSlotByPredecessor(slots, 9) == slotB_initial);
    // currentGain should be preserved — these were ACTIVE→ACTIVE, no
    // drain or cold reinit.
    CHECK(slots[slotA_initial].currentGain == 1.0f);
    CHECK(slots[slotB_initial].currentGain == 1.0f);
}

// ════════════════════════════════════════════════════════════════════════════
// Path drop-out: Active → Draining → Free
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("SubSourceSlots: vanished path drains then is reaped",
          "[subsource][slots][lifecycle]") {
    std::array<SubSource, kMaxSubSources> slots{};

    SoundPropInfo f1 = propWith({makePath(/*pred=*/5, /*effDist=*/10.0f)});
    updateSubSourceSlots(slots, f1, kMaxSubSources, sentinelDir);
    int slot = findSlotByPredecessor(slots, 5);
    REQUIRE(slot >= 0);
    REQUIRE(slots[slot].state == SubSourceState::Active);
    slots[slot].currentGain = 1.0f;  // simulate audio-thread ramp-up

    // Frame 2: path 5 vanished. Should transition to DRAINING.
    SoundPropInfo f2 = propWith({});
    updateSubSourceSlots(slots, f2, kMaxSubSources, sentinelDir);
    CHECK(slots[slot].state == SubSourceState::Draining);
    CHECK(slots[slot].targetGain == 0.0f);
    // currentGain not yet reaped — slot still holds its identity.
    CHECK(slots[slot].currentGain == 1.0f);
    CHECK(slots[slot].predecessorRoomID == 5);

    // Frame 3: audio thread has drained currentGain past the epsilon.
    slots[slot].currentGain = 0.5e-4f;  // below kSubSourceDrainEpsilon
    SoundPropInfo f3 = propWith({});
    updateSubSourceSlots(slots, f3, kMaxSubSources, sentinelDir);
    CHECK(slots[slot].state == SubSourceState::Free);
    CHECK(slots[slot].predecessorRoomID == -1);
}

// ════════════════════════════════════════════════════════════════════════════
// Path resurrection within drain window: Draining → Active without cold reinit
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("SubSourceSlots: path returning mid-drain resumes without cold reinit",
          "[subsource][slots][resurrection]") {
    std::array<SubSource, kMaxSubSources> slots{};

    updateSubSourceSlots(slots, propWith({makePath(5, 10.0f)}),
                         kMaxSubSources, sentinelDir);
    int slot = findSlotByPredecessor(slots, 5);
    REQUIRE(slot >= 0);
    slots[slot].currentGain = 1.0f;

    // Path vanishes → Draining.
    updateSubSourceSlots(slots, propWith({}), kMaxSubSources, sentinelDir);
    REQUIRE(slots[slot].state == SubSourceState::Draining);
    // Audio thread has only partially drained it.
    slots[slot].currentGain = 0.7f;

    // Same path returns. Slot is still keyed by predecessorRoomID, so
    // findSlotByPredecessor matches it (Draining slots aren't FREE);
    // it resumes ACTIVE without snapping currentGain — preserving the
    // partial decay so the audio thread can ramp back up smoothly
    // from there rather than starting over from silence.
    updateSubSourceSlots(slots, propWith({makePath(5, 10.0f)}),
                         kMaxSubSources, sentinelDir);
    CHECK(slots[slot].state == SubSourceState::Active);
    CHECK(slots[slot].targetGain == 1.0f);
    CHECK(slots[slot].currentGain == 0.7f);  // ramp continues from here
    CHECK(slots[slot].predecessorRoomID == 5);
}

// ════════════════════════════════════════════════════════════════════════════
// Capacity overflow: LRU-by-energy eviction defers the new path one frame
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("SubSourceSlots: when a slot's old path is no longer "
          "in prop.paths but a NEW predecessor needs a slot, eviction "
          "trumps the would-be drain so the new path gets considered "
          "while the old slot tail-flushes",
          "[subsource][slots][eviction]") {
    std::array<SubSource, kMaxSubSources> slots{};
    static_assert(kMaxSubSources == 4, "test assumes kMaxSubSources == 4");

    // Frame 1: fill all four slots; per-frame BFS returned {1,2,3,4}.
    updateSubSourceSlots(slots, propWith({
        makePath(1, 10.0f),
        makePath(2, 11.0f),
        makePath(3, 12.0f),
        makePath(4, 13.0f),
    }), kMaxSubSources, sentinelDir);
    for (auto& s : slots) {
        if (s.state == SubSourceState::Active) s.currentGain = 1.0f;
    }
    const int slot2 = findSlotByPredecessor(slots, 2);
    REQUIRE(slot2 >= 0);
    // Make slot2 the lowest-energy active slot so it's the eviction
    // victim in frame 2 below.
    slots[slot2].currentGain = 0.1f;

    // Frame 2: pred=2 has vanished from prop.paths AND a new pred=5
    // has appeared. Three of the original four (1, 3, 4) persist.
    //   pred=1 → reuse slot0  (seen)
    //   pred=3 → reuse slot2  (seen) — note: slot indices and pred IDs
    //                                  no longer line up after the
    //                                  cold start, that's fine.
    //   pred=4 → reuse slot3  (seen)
    //   pred=5 → no match; findFreeSlot finds NONE (slot for old
    //            pred=2 is still ACTIVE, not Free, until step 3 below
    //            would normally drain it). The eviction path picks
    //            it — already the lowest-energy slot — and drains it.
    //            pred=5 waits one frame for the drain to finish.
    updateSubSourceSlots(slots, propWith({
        makePath(1, 10.0f),
        makePath(3, 12.0f),
        makePath(4, 13.0f),
        makePath(5, 14.0f),
    }), kMaxSubSources, sentinelDir);

    CHECK(slots[slot2].state == SubSourceState::Draining);
    CHECK(slots[slot2].targetGain == 0.0f);
    CHECK(findSlotByPredecessor(slots, 5) == -1);  // pred=5 deferred
    CHECK(findSlotByPredecessor(slots, 1) >= 0);
    CHECK(findSlotByPredecessor(slots, 3) >= 0);
    CHECK(findSlotByPredecessor(slots, 4) >= 0);

    // Frame 3: audio thread finishes draining slot2. Slot is reaped
    // to FREE at the top of updateSubSourceSlots, then pred=5 takes
    // it.
    slots[slot2].currentGain = 0.5e-4f;
    updateSubSourceSlots(slots, propWith({
        makePath(1, 10.0f),
        makePath(3, 12.0f),
        makePath(4, 13.0f),
        makePath(5, 14.0f),
    }), kMaxSubSources, sentinelDir);
    const int slot5 = findSlotByPredecessor(slots, 5);
    CHECK(slot5 == slot2);
    CHECK(slots[slot5].state == SubSourceState::Active);
}

TEST_CASE("SubSourceSlots: drain-without-evict when an unused free "
          "slot is available (the lazy path)",
          "[subsource][slots][eviction][nominal]") {
    std::array<SubSource, kMaxSubSources> slots{};

    // Frame 1: cold start with 2 paths — slots 0 and 1 used; slots 2
    // and 3 remain FREE.
    updateSubSourceSlots(slots, propWith({
        makePath(1, 10.0f),
        makePath(2, 11.0f),
    }), kMaxSubSources, sentinelDir);
    int slot1 = findSlotByPredecessor(slots, 1);
    int slot2 = findSlotByPredecessor(slots, 2);
    REQUIRE(slot1 >= 0);
    REQUIRE(slot2 >= 0);
    for (auto& s : slots) {
        if (s.state == SubSourceState::Active) s.currentGain = 1.0f;
    }

    // Frame 2: paths {1, 3} — pred=2 dropped, pred=3 new.
    //   pred=1 reuses slot1.
    //   pred=3 finds NO match, takes the first FREE slot (no eviction
    //          needed). slot2's old path (pred=2) drains via step 3.
    updateSubSourceSlots(slots, propWith({
        makePath(1, 10.0f),
        makePath(3, 12.0f),
    }), kMaxSubSources, sentinelDir);

    CHECK(slots[slot2].state == SubSourceState::Draining);
    CHECK(findSlotByPredecessor(slots, 3) >= 0);
    CHECK(findSlotByPredecessor(slots, 3) != slot2);  // pred=3 took a FREE slot, not slot2
}

// ════════════════════════════════════════════════════════════════════════════
// maxN cap clamps the per-voice sub-source count
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("SubSourceSlots: maxN cap limits assigned paths",
          "[subsource][slots][cap]") {
    std::array<SubSource, kMaxSubSources> slots{};

    // 4 paths, cap=2 → only the two shortest paths get slots.
    updateSubSourceSlots(slots, propWith({
        makePath(1, 10.0f),
        makePath(2, 11.0f),
        makePath(3, 12.0f),
        makePath(4, 13.0f),
    }), /*maxN=*/2, sentinelDir);

    int active = 0;
    for (const auto& s : slots) {
        if (s.state == SubSourceState::Active) ++active;
    }
    CHECK(active == 2);
    CHECK(findSlotByPredecessor(slots, 1) >= 0);
    CHECK(findSlotByPredecessor(slots, 2) >= 0);
    CHECK(findSlotByPredecessor(slots, 3) < 0);
    CHECK(findSlotByPredecessor(slots, 4) < 0);
}

// ════════════════════════════════════════════════════════════════════════════
// drainAllSubSourceSlots: silenced regime
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("SubSourceSlots: drainAllSubSourceSlots silences every active slot",
          "[subsource][slots][drain_all]") {
    std::array<SubSource, kMaxSubSources> slots{};

    updateSubSourceSlots(slots, propWith({
        makePath(1, 10.0f),
        makePath(2, 11.0f),
    }), kMaxSubSources, sentinelDir);
    for (auto& s : slots) {
        if (s.state == SubSourceState::Active) s.currentGain = 1.0f;
    }

    drainAllSubSourceSlots(slots);
    for (const auto& s : slots) {
        CHECK(s.state != SubSourceState::Active);
    }

    // Drain and reap to FREE.
    for (auto& s : slots) {
        if (s.state == SubSourceState::Draining) s.currentGain = 0.0f;
    }
    drainAllSubSourceSlots(slots);
    for (const auto& s : slots) {
        CHECK(s.state == SubSourceState::Free);
        CHECK(s.predecessorRoomID == -1);
    }
}

// ════════════════════════════════════════════════════════════════════════════
// targetDoorBlocking: helper stores raw door blocking; audio thread converts to α
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("SubSourceSlots: targetGain encodes per-path (realDist/effDist)^2",
          "[subsource][slots][gain]") {
    std::array<SubSource, kMaxSubSources> slots{};
    // Path A: same-room (realDist = effDist) → pathGain = 1.0
    // Path B: cross-room with 20% door inflation → ratio = 10/12, pathGain ≈ 0.694
    // Path C: doorless very far → realDist = effDist again → pathGain = 1.0
    SoundPropInfo prop = propWith({
        makePath(/*pred=*/1, /*effDist=*/10.0f, /*door=*/0.0f, /*realDist=*/10.0f),
        makePath(/*pred=*/2, /*effDist=*/12.0f, /*door=*/0.2f, /*realDist=*/10.0f),
        makePath(/*pred=*/3, /*effDist=*/50.0f, /*door=*/0.0f, /*realDist=*/50.0f),
    });
    updateSubSourceSlots(slots, prop, kMaxSubSources, sentinelDir);

    const int s1 = findSlotByPredecessor(slots, 1);
    const int s2 = findSlotByPredecessor(slots, 2);
    const int s3 = findSlotByPredecessor(slots, 3);
    REQUIRE(s1 >= 0);
    REQUIRE(s2 >= 0);
    REQUIRE(s3 >= 0);

    CHECK(slots[s1].targetGain == 1.0f);
    // (10/12)^2 = 0.6944... — use approximate compare for FP safety.
    const float expectedB = (10.0f / 12.0f) * (10.0f / 12.0f);
    CHECK(std::fabs(slots[s2].targetGain - expectedB) < 1e-5f);
    CHECK(slots[s3].targetGain == 1.0f);
}

TEST_CASE("SubSourceSlots: targetDoorBlocking holds the path's raw door blocking",
          "[subsource][slots][blocking]") {
    std::array<SubSource, kMaxSubSources> slots{};
    SoundPropInfo prop = propWith({
        makePath(/*pred=*/5, /*effDist=*/10.0f, /*door=*/0.0f),
        makePath(/*pred=*/9, /*effDist=*/12.0f, /*door=*/0.6f),
    });
    updateSubSourceSlots(slots, prop, kMaxSubSources, sentinelDir);
    const int s5 = findSlotByPredecessor(slots, 5);
    const int s9 = findSlotByPredecessor(slots, 9);
    REQUIRE(s5 >= 0);
    REQUIRE(s9 >= 0);
    CHECK(slots[s5].targetDoorBlocking == 0.0f);  // open door
    CHECK(slots[s9].targetDoorBlocking == 0.6f);  // partially blocked
    // currentDoorAlpha snapped to 1.0 on cold (audio thread ramps to actual α).
    CHECK(slots[s5].currentDoorAlpha == 1.0f);
    CHECK(slots[s9].currentDoorAlpha == 1.0f);
}
