/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2025 darkness contributors
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

// Door → WR portal aperture binding.
//
// The rule these tests pin down: a shut door closes the opening it actually
// fills and nothing else. The gate this replaced keyed on the ROOM_DB room
// pair, which — because a Thief doorway carries its own thin threshold room
// brush — dropped the doorway's own cells and stopped the jamb / head /
// threshold reveal from being drawn while the door was shut.

#include <catch2/catch_test_macros.hpp>

#include <vector>

#include "DoorPortalBinding.h"

using namespace Darkness;

namespace {

// One quad portal from `src` to `dst`, lying on the given plane.
void addPortal(WRParsedCell &cell, const std::vector<Vector3> &quad,
               uint16_t dst, const Vector3 &normal, float d) {
    uint8_t base = static_cast<uint8_t>(cell.vertices.size());
    for (const auto &v : quad) cell.vertices.push_back(v);

    WRPolygon poly{};
    poly.count   = 4;
    poly.plane   = static_cast<uint8_t>(cell.planes.size());
    poly.tgtCell = dst;
    cell.polygons.push_back(poly);
    cell.polyIndices.push_back({base, uint8_t(base + 1),
                                uint8_t(base + 2), uint8_t(base + 3)});

    Plane pl;
    pl.normal = normal;
    pl.d = d;
    cell.planes.push_back(pl);

    cell.numPolygons = static_cast<uint8_t>(cell.polygons.size());
    cell.numPortals  = cell.numPolygons;   // every poly here is a portal
    cell.numPlanes   = static_cast<uint8_t>(cell.planes.size());
}

// A 4 x 0.3 x 8 ft leaf standing in the x=0 plane, centred at `centre`,
// hinged on Z — the shape every Thief door model measures out to.
DoorAperture leafAt(int32_t id, const Vector3 &centre) {
    DoorAperture ap;
    ap.objID   = id;
    ap.center  = centre;
    ap.axis[0] = Vector3(1, 0, 0);   // through the leaf (thin)
    ap.axis[1] = Vector3(0, 1, 0);   // across the opening
    ap.axis[2] = Vector3(0, 0, 1);   // up
    ap.half    = Vector3(0.15f, 2.0f, 4.0f);
    ap.thinAxis = 0;
    return ap;
}

// Quad in the x = px plane, spanning [y0,y1] x [z0,z1].
std::vector<Vector3> yzQuad(float px, float y0, float y1, float z0, float z1) {
    return {Vector3(px, y0, z0), Vector3(px, y1, z0),
            Vector3(px, y1, z1), Vector3(px, y0, z1)};
}

// Quad in the z = pz plane, spanning [x0,x1] x [y0,y1].
std::vector<Vector3> xyQuad(float pz, float x0, float x1, float y0, float y1) {
    return {Vector3(x0, y0, pz), Vector3(x1, y0, pz),
            Vector3(x1, y1, pz), Vector3(x0, y1, pz)};
}

} // namespace

TEST_CASE("Door binds to the aperture it fills", "[doorportal]") {
    WRParsedData wr;
    wr.cells.resize(4);
    // The opening the leaf sits in: cell 0 <-> cell 1 across x = 0.
    addPortal(wr.cells[0], yzQuad(0.0f, -2.0f, 2.0f, -4.0f, 4.0f),
              1, Vector3(1, 0, 0), 0.0f);
    wr.numCells = 4;

    auto map = buildDoorPortalMap(wr, {leafAt(7, Vector3(0, 0, 0))});

    REQUIRE(map.size() == 1);
    auto it = map.find(cellPairKey(0, 1));
    REQUIRE(it != map.end());
    REQUIRE(it->second == std::vector<int32_t>{7});
}

TEST_CASE("Door ignores the BSP splits crowding its doorway", "[doorportal]") {
    WRParsedData wr;
    wr.cells.resize(4);
    wr.numCells = 4;

    // The real opening.
    addPortal(wr.cells[0], yzQuad(0.0f, -2.0f, 2.0f, -4.0f, 4.0f),
              1, Vector3(1, 0, 0), 0.0f);
    // A floor split directly under the leaf — inside the OBB, but its plane
    // is perpendicular to the leaf face, so it is not part of the opening.
    addPortal(wr.cells[0], xyQuad(-3.9f, -1.0f, 1.0f, -1.0f, 1.0f),
              2, Vector3(0, 0, 1), 3.9f);
    // A second doorway further along the same wall: coplanar with the leaf,
    // so only the OBB extent test can reject it.
    addPortal(wr.cells[0], yzQuad(0.0f, 9.0f, 13.0f, -4.0f, 4.0f),
              3, Vector3(1, 0, 0), 0.0f);

    auto map = buildDoorPortalMap(wr, {leafAt(7, Vector3(0, 0, 0))});

    REQUIRE(map.size() == 1);
    CHECK(map.count(cellPairKey(0, 1)) == 1);
    CHECK(map.count(cellPairKey(0, 2)) == 0);   // perpendicular split
    CHECK(map.count(cellPairKey(0, 3)) == 0);   // neighbouring doorway
}

TEST_CASE("Both leaves of a double door claim the same aperture",
          "[doorportal]") {
    WRParsedData wr;
    wr.cells.resize(2);
    wr.numCells = 2;
    // One wide opening filled by two leaves meeting in the middle.
    addPortal(wr.cells[0], yzQuad(0.0f, -4.0f, 4.0f, -4.0f, 4.0f),
              1, Vector3(1, 0, 0), 0.0f);

    auto map = buildDoorPortalMap(
        wr, {leafAt(11, Vector3(0, -2.0f, 0)), leafAt(12, Vector3(0, 2.0f, 0))});

    auto it = map.find(cellPairKey(0, 1));
    REQUIRE(it != map.end());
    REQUIRE(it->second.size() == 2);
    // Order follows door order; both must be present so the gate can require
    // every leaf to be shut before it blocks.
    CHECK(it->second[0] == 11);
    CHECK(it->second[1] == 12);
}

TEST_CASE("A door with no opening behind it claims nothing", "[doorportal]") {
    WRParsedData wr;
    wr.cells.resize(2);
    wr.numCells = 2;
    // Only opening in the level is 40 ft away.
    addPortal(wr.cells[0], yzQuad(40.0f, -2.0f, 2.0f, -4.0f, 4.0f),
              1, Vector3(1, 0, 0), -40.0f);

    auto map = buildDoorPortalMap(wr, {leafAt(7, Vector3(0, 0, 0))});

    CHECK(map.empty());   // fails open: over-renders, never deletes surface
}

TEST_CASE("Aperture binding is direction-agnostic", "[doorportal]") {
    // The same opening is stored once in each cell it joins, with opposite
    // plane normals. Both records must land on one key, or portalBFS would
    // block only the traversal that happens to enter from one side.
    WRParsedData wr;
    wr.cells.resize(2);
    wr.numCells = 2;
    addPortal(wr.cells[0], yzQuad(0.0f, -2.0f, 2.0f, -4.0f, 4.0f),
              1, Vector3(1, 0, 0), 0.0f);
    addPortal(wr.cells[1], yzQuad(0.0f, -2.0f, 2.0f, -4.0f, 4.0f),
              0, Vector3(-1, 0, 0), 0.0f);

    auto map = buildDoorPortalMap(wr, {leafAt(7, Vector3(0, 0, 0))});

    REQUIRE(map.size() == 1);
    CHECK(cellPairKey(0, 1) == cellPairKey(1, 0));
    auto it = map.find(cellPairKey(0, 1));
    REQUIRE(it != map.end());
    CHECK(it->second == std::vector<int32_t>{7});   // registered once, not twice
}

TEST_CASE("Door ignores coplanar boundaries stacked behind its opening",
          "[doorportal]") {
    // The regression that shipped in the first cut of this binding: a 2 ft
    // through-margin let a shut door claim every coplanar cell boundary near
    // its face. A BSP-split doorway stacks several of those — MISS6 door 419
    // claimed boundaries at 0.04, 0.62, 0.87, 1.37 and 1.88 ft — and blocking
    // them cut real cells out of the rooms either side, so shutting the door
    // punched fresh holes beside the doorway.
    WRParsedData wr;
    wr.cells.resize(5);
    wr.numCells = 5;

    // The opening the leaf actually fills.
    addPortal(wr.cells[0], yzQuad(0.0f, -2.0f, 2.0f, -4.0f, 4.0f),
              1, Vector3(1, 0, 0), 0.0f);
    // Cell boundaries in the rooms either side: coplanar with the leaf and
    // squarely inside its footprint, but the leaf does not obstruct them.
    addPortal(wr.cells[0], yzQuad(0.9f, -2.0f, 2.0f, -4.0f, 4.0f),
              2, Vector3(1, 0, 0), -0.9f);
    addPortal(wr.cells[0], yzQuad(-1.7f, -2.0f, 2.0f, -4.0f, 4.0f),
              3, Vector3(1, 0, 0), 1.7f);

    auto map = buildDoorPortalMap(wr, {leafAt(7, Vector3(0, 0, 0))});

    REQUIRE(map.size() == 1);
    CHECK(map.count(cellPairKey(0, 1)) == 1);
    CHECK(map.count(cellPairKey(0, 2)) == 0);
    CHECK(map.count(cellPairKey(0, 3)) == 0);
}

TEST_CASE("Offset leaf binds to its nearest opening only", "[doorportal]") {
    // A tall gate whose P$PhysDims is not centred on the leaf (MISS6 door 705:
    // only candidate 1.50 ft out, half-thickness 0.25). It must still block —
    // but only the one opening, not the stack behind it.
    WRParsedData wr;
    wr.cells.resize(4);
    wr.numCells = 4;

    addPortal(wr.cells[0], yzQuad(1.5f, -2.0f, 2.0f, -4.0f, 4.0f),
              1, Vector3(1, 0, 0), -1.5f);
    addPortal(wr.cells[0], yzQuad(2.6f, -2.0f, 2.0f, -4.0f, 4.0f),
              2, Vector3(1, 0, 0), -2.6f);

    auto map = buildDoorPortalMap(wr, {leafAt(7, Vector3(0, 0, 0))});

    REQUIRE(map.size() == 1);
    CHECK(map.count(cellPairKey(0, 1)) == 1);   // nearest opening: bound
    CHECK(map.count(cellPairKey(0, 2)) == 0);   // the one behind it: not
}
