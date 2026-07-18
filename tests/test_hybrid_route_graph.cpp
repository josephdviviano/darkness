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

#include <catch2/catch_test_macros.hpp>
#include "audio/HybridRouteGraph.h"

#include <vector>

using namespace Darkness;

// Toy graph — two parallel corridors between a source room and a listener
// room, each with one door:
//
//   0 ---A--- 1 ---2 (listener side)      short route, door A at edge 0-1
//   |                  |
//   3 ---B--- 4 ------5                    long route,  door B at edge 3-4
//
// probe 0 = source anchor, probe 2 = listener anchor.
static HybridRouteGraph makeGraph() {
    std::vector<Vector3> pos = {
        {0.0f, 0.0f, 0.0f},   // 0 source side
        {10.0f, 0.0f, 0.0f},  // 1 through door A
        {20.0f, 0.0f, 0.0f},  // 2 listener side
        {0.0f, 20.0f, 0.0f},  // 3 source side (lower corridor)
        {10.0f, 20.0f, 0.0f}, // 4 through door B
        {20.0f, 20.0f, 0.0f}, // 5 listener side (lower)
    };
    std::vector<std::pair<int, int>> edges = {
        {0, 1}, {1, 2},          // short corridor (door A between 0-1)
        {0, 3}, {3, 4}, {4, 5},  // long corridor (door B between 3-4)
        {2, 5},                  // both corridors meet at the listener side
    };
    std::vector<HybridRouteGraph::DoorBox> doors = {
        {100, {5.0f, 0.0f, 0.0f}, 2.0f},   // door A on edge 0-1
        {200, {5.0f, 20.0f, 0.0f}, 2.0f},  // door B on edge 3-4
    };
    HybridRouteGraph g;
    g.build(pos, edges, doors);
    return g;
}

TEST_CASE("HybridRouteGraph builds and maps doors to edges", "[hybrid]") {
    HybridRouteGraph g = makeGraph();
    REQUIRE(g.built());
    REQUIRE(g.numProbes() == 6);
    // Exactly two edges carry a door (0-1 -> A, 3-4 -> B).
    REQUIRE(g.doorEdgeCount() == 2);
}

TEST_CASE("nearestProbe finds the closest probe", "[hybrid]") {
    HybridRouteGraph g = makeGraph();
    REQUIRE(g.nearestProbe({0.5f, 0.5f, 0.0f}) == 0);
    REQUIRE(g.nearestProbe({19.0f, 1.0f, 0.0f}) == 2);
    REQUIRE(g.nearestProbe({9.5f, 20.5f, 0.0f}) == 4);
    // Far outside the populated region still resolves.
    REQUIRE(g.nearestProbe({1000.0f, 1000.0f, 0.0f}) >= 0);
}

TEST_CASE("pathDoors: all doors open -> shortest route's door", "[hybrid]") {
    HybridRouteGraph g = makeGraph();
    auto allOpen = [](int32_t) { return 1.0f; };
    // Source near probe 0, listener near probe 2: shortest is 0-1-2 through
    // door A only.
    std::vector<int32_t> doors;
    REQUIRE(g.route({0.0f, 0.0f, 0.0f}, {20.0f, 0.0f, 0.0f}, allOpen, 0.05f, doors));
    REQUIRE(doors.size() == 1);
    REQUIRE(doors[0] == 100);   // door A
}

TEST_CASE("pathDoors: closing door A reroutes around it", "[hybrid]") {
    HybridRouteGraph g = makeGraph();
    // Door A fully closed, door B open: the only surviving route is
    // 0-3-4-5-2 through door B.
    auto aClosed = [](int32_t id) { return id == 100 ? 0.0f : 1.0f; };
    std::vector<int32_t> doors;
    REQUIRE(g.route({0.0f, 0.0f, 0.0f}, {20.0f, 0.0f, 0.0f}, aClosed, 0.05f, doors));
    REQUIRE(doors.size() == 1);
    REQUIRE(doors[0] == 200);   // rerouted through door B
}

TEST_CASE("pathDoors: partially-open door stays on the path", "[hybrid]") {
    HybridRouteGraph g = makeGraph();
    // Door A half-open (0.5) is NOT below the closed threshold, so the
    // shortest route still goes through it — the caller multiplies by 0.5.
    auto aHalf = [](int32_t id) { return id == 100 ? 0.5f : 1.0f; };
    std::vector<int32_t> doors;
    REQUIRE(g.route({0.0f, 0.0f, 0.0f}, {20.0f, 0.0f, 0.0f}, aHalf, 0.05f, doors));
    REQUIRE(doors.size() == 1);
    REQUIRE(doors[0] == 100);
}

TEST_CASE("route: both doors closed -> UNREACHABLE (returns false)", "[hybrid]") {
    HybridRouteGraph g = makeGraph();
    auto allClosed = [](int32_t) { return 0.0f; };
    std::vector<int32_t> doors;
    // false => caller gates to SILENCE, not to 1 (the 0->1 jump we fixed).
    REQUIRE_FALSE(g.route({0.0f, 0.0f, 0.0f}, {20.0f, 0.0f, 0.0f}, allClosed, 0.05f, doors));
    REQUIRE(doors.empty());
}

TEST_CASE("pathDoors: door-free route returns no doors", "[hybrid]") {
    // A single clear edge with no door.
    HybridRouteGraph g;
    std::vector<Vector3> pos = {{0, 0, 0}, {10, 0, 0}};
    std::vector<std::pair<int, int>> edges = {{0, 1}};
    g.build(pos, edges, {});
    std::vector<int32_t> doors;
    REQUIRE(g.route({0, 0, 0}, {10, 0, 0}, [](int32_t){ return 1.0f; }, 0.05f, doors));
    REQUIRE(doors.empty());
}
