/******************************************************************************
 *
 *    This file is part of the Darkness engine
 *    Copyright (C) 2024-2026 Darkness contributors
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

// test_subobject_joints.cpp — LGMD sub-object posing (model joints)
//
// Covers the pose maths in SubObjectPose.h and the joints tweq in TweqSystem.
// Meshes are hand-built here rather than parsed from obj.crf, so the suite
// stays asset-free.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cmath>
#include <cstring>
#include <vector>

#include "SubObjectPose.h"
#include "sim/TweqSystem.h"

using Catch::Approx;
using Darkness::BinSubObject;
using Darkness::Matrix4;
using Darkness::ParsedBinMesh;
using Darkness::Vector3;

namespace {

/// Column-major 3x3: the three arguments are the images of the part's local
/// X, Y and Z axes in its parent's space.
BinSubObject makePart(const char *name, uint8_t movement, int32_t jointIdx,
                      Vector3 colX, Vector3 colY, Vector3 colZ, Vector3 axle) {
    BinSubObject s = {};
    std::strncpy(s.name, name, sizeof(s.name) - 1);
    s.movement = movement;
    s.jointIdx = jointIdx;
    s.minRange = 0.0f;
    s.maxRange = 0.0f;
    const float rot[9] = {colX.x, colX.y, colX.z,
                          colY.x, colY.y, colY.z,
                          colZ.x, colZ.y, colZ.z};
    std::memcpy(s.rot, rot, sizeof(rot));
    s.axle[0] = axle.x; s.axle[1] = axle.y; s.axle[2] = axle.z;
    s.child = -1;
    s.next = -1;
    return s;
}

BinSubObject makeRoot() {
    BinSubObject s = {};
    std::strncpy(s.name, "root", sizeof(s.name) - 1);
    s.movement = Darkness::kSubObjStatic;
    s.jointIdx = -1;
    // The on-disk root carries an all-zero rot in every stock model; the pose
    // code must treat it as the model frame, not as a (degenerate) basis.
    s.child = -1;
    s.next = -1;
    return s;
}

Vector3 apply(const Matrix4 &m, Vector3 v) {
    return Vector3(m * glm::vec4(v, 1.0f));
}

/// A part whose local X maps to world +Z, local Y to world +X, local Z to
/// world +Y — the shape stock models use to put a hinge on a chosen axis.
ParsedBinMesh singleJointMesh(uint8_t movement, int32_t jointIdx) {
    ParsedBinMesh mesh = {};
    mesh.subObjects.push_back(makeRoot());
    mesh.subObjects.push_back(makePart("@s00bb", movement, jointIdx,
                                       Vector3(0, 0, 1),   // local X -> world +Z
                                       Vector3(1, 0, 0),   // local Y -> world +X
                                       Vector3(0, 1, 0),   // local Z -> world +Y
                                       Vector3(5, 0, 0)));
    mesh.subObjects[0].child = 1;
    return mesh;
}

} // namespace

// ── Rest pose ──

TEST_CASE("SubObjectPose: rest pose reproduces the baked transform", "[SubObject]") {
    // The old parser baked `rot * v + axle` into the vertices at load. With
    // every joint at rest the composed matrix must do exactly that, or every
    // multi-part model shifts the day this lands.
    ParsedBinMesh mesh = singleJointMesh(Darkness::kSubObjRotate, 0);
    std::vector<Matrix4> mtx;
    Darkness::composeSubObjectMatrices(mesh, nullptr, 0, mtx);

    REQUIRE(mtx.size() == 2);

    const Vector3 local(2.0f, 3.0f, 4.0f);
    const BinSubObject &s = mesh.subObjects[1];
    const Vector3 baked(
        s.rot[0]*local.x + s.rot[3]*local.y + s.rot[6]*local.z + s.axle[0],
        s.rot[1]*local.x + s.rot[4]*local.y + s.rot[7]*local.z + s.axle[1],
        s.rot[2]*local.x + s.rot[5]*local.y + s.rot[8]*local.z + s.axle[2]);

    const Vector3 posed = apply(mtx[1], local);
    REQUIRE(posed.x == Approx(baked.x));
    REQUIRE(posed.y == Approx(baked.y));
    REQUIRE(posed.z == Approx(baked.z));
}

TEST_CASE("SubObjectPose: the root is the model frame", "[SubObject]") {
    ParsedBinMesh mesh = singleJointMesh(Darkness::kSubObjRotate, 0);
    std::vector<Matrix4> mtx;
    Darkness::composeSubObjectMatrices(mesh, nullptr, 0, mtx);

    const Vector3 v = apply(mtx[0], Vector3(1.0f, 2.0f, 3.0f));
    REQUIRE(v.x == Approx(1.0f));
    REQUIRE(v.y == Approx(2.0f));
    REQUIRE(v.z == Approx(3.0f));
}

TEST_CASE("SubObjectPose: nested parts inherit the whole ancestor chain",
          "[SubObject]") {
    // The bug this replaces: the old path applied a part's OWN frame only, so
    // the grandchild of a camera or turret drew detached from the model.
    ParsedBinMesh mesh = {};
    mesh.subObjects.push_back(makeRoot());
    mesh.subObjects.push_back(makePart("@s00aa", Darkness::kSubObjRotate, 0,
                                       Vector3(1, 0, 0), Vector3(0, 1, 0),
                                       Vector3(0, 0, 1), Vector3(10, 0, 0)));
    mesh.subObjects.push_back(makePart("@s01bb", Darkness::kSubObjRotate, 1,
                                       Vector3(1, 0, 0), Vector3(0, 1, 0),
                                       Vector3(0, 0, 1), Vector3(0, 2, 0)));
    mesh.subObjects[0].child = 1;
    mesh.subObjects[1].child = 2;

    std::vector<Matrix4> mtx;
    Darkness::composeSubObjectMatrices(mesh, nullptr, 0, mtx);
    REQUIRE(mtx.size() == 3);

    const Vector3 childOrigin = apply(mtx[2], Vector3(0.0f));
    REQUIRE(childOrigin.x == Approx(10.0f));   // 0.0 under the old baked path
    REQUIRE(childOrigin.y == Approx(2.0f));
    REQUIRE(childOrigin.z == Approx(0.0f));
}

// ── Joint motion ──

TEST_CASE("SubObjectPose: a rotate joint turns about the part's local X",
          "[SubObject]") {
    ParsedBinMesh mesh = singleJointMesh(Darkness::kSubObjRotate, 0);
    float joints[6] = {90.0f, 0, 0, 0, 0, 0};   // JointPos stores DEGREES

    std::vector<Matrix4> mtx;
    Darkness::composeSubObjectMatrices(mesh, joints, 6, mtx);

    // A point one unit off the hinge, in the part's local +Y. At rest it sits
    // at world (6, 0, 0); local X maps to world +Z, so a quarter turn must
    // swing it in the world XY plane about (5, 0, 0) and keep its world Z.
    const Vector3 swung = apply(mtx[1], Vector3(0.0f, 1.0f, 0.0f));
    REQUIRE(swung.x == Approx(5.0f).margin(1e-5));
    REQUIRE(swung.y == Approx(1.0f).margin(1e-5));
    REQUIRE(swung.z == Approx(0.0f).margin(1e-5));

    // ... and the hinge itself must not move.
    const Vector3 onAxis = apply(mtx[1], Vector3(3.0f, 0.0f, 0.0f));
    REQUIRE(onAxis.x == Approx(5.0f).margin(1e-5));
    REQUIRE(onAxis.y == Approx(0.0f).margin(1e-5));
    REQUIRE(onAxis.z == Approx(3.0f).margin(1e-5));
}

TEST_CASE("SubObjectPose: rotate joints read as degrees", "[SubObject]") {
    ParsedBinMesh mesh = singleJointMesh(Darkness::kSubObjRotate, 0);

    // 360 degrees is a full turn: the part must land back on its rest pose.
    float full[6] = {360.0f, 0, 0, 0, 0, 0};
    std::vector<Matrix4> turned;
    Darkness::composeSubObjectMatrices(mesh, full, 6, turned);

    std::vector<Matrix4> rest;
    Darkness::composeSubObjectMatrices(mesh, nullptr, 0, rest);

    const Vector3 p(0.0f, 1.0f, 0.5f);
    const Vector3 a = apply(turned[1], p);
    const Vector3 b = apply(rest[1], p);
    REQUIRE(a.x == Approx(b.x).margin(1e-4));
    REQUIRE(a.y == Approx(b.y).margin(1e-4));
    REQUIRE(a.z == Approx(b.z).margin(1e-4));
}

TEST_CASE("SubObjectPose: a slide joint translates along local X in world units",
          "[SubObject]") {
    ParsedBinMesh mesh = singleJointMesh(Darkness::kSubObjSlide, 0);
    float joints[6] = {3.0f, 0, 0, 0, 0, 0};   // units, NOT degrees

    std::vector<Matrix4> mtx;
    Darkness::composeSubObjectMatrices(mesh, joints, 6, mtx);

    // Local X maps to world +Z, so a pop-up turret's top rises by 3.
    const Vector3 moved = apply(mtx[1], Vector3(0.0f));
    REQUIRE(moved.x == Approx(5.0f).margin(1e-5));
    REQUIRE(moved.y == Approx(0.0f).margin(1e-5));
    REQUIRE(moved.z == Approx(3.0f).margin(1e-5));
}

TEST_CASE("SubObjectPose: joint slots past the property's six read as rest",
          "[SubObject]") {
    // Stock models index joints up to 13 (MECLOCK2) while JointPos stores six.
    // Those parts never move — they must not read past the array either.
    ParsedBinMesh mesh = singleJointMesh(Darkness::kSubObjRotate, 13);
    float joints[6] = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f};

    std::vector<Matrix4> posed, rest;
    Darkness::composeSubObjectMatrices(mesh, joints, 6, posed);
    Darkness::composeSubObjectMatrices(mesh, nullptr, 0, rest);

    const Vector3 p(0.0f, 1.0f, 0.0f);
    REQUIRE(apply(posed[1], p).y == Approx(apply(rest[1], p).y));
    REQUIRE(Darkness::meshJointsAtRest(mesh, joints, 6));
}

TEST_CASE("SubObjectPose: authored ranges do not clamp the joint value",
          "[SubObject]") {
    // Five stock parts carry a range that excludes zero (MECLOCK2 @s12ll is
    // [-1.745, -13.352]). Clamping to it would displace them at rest, so the
    // mesh range is metadata, not a runtime limit.
    ParsedBinMesh mesh = singleJointMesh(Darkness::kSubObjRotate, 0);
    mesh.subObjects[1].minRange = -1.745f;
    mesh.subObjects[1].maxRange = -13.352f;

    std::vector<Matrix4> atZero, rest;
    float zero[6] = {0, 0, 0, 0, 0, 0};
    Darkness::composeSubObjectMatrices(mesh, zero, 6, atZero);
    Darkness::composeSubObjectMatrices(mesh, nullptr, 0, rest);

    const Vector3 p(0.0f, 1.0f, 0.0f);
    REQUIRE(apply(atZero[1], p).x == Approx(apply(rest[1], p).x));
    REQUIRE(apply(atZero[1], p).y == Approx(apply(rest[1], p).y));

    // A value outside the authored interval is still applied: half a turn
    // takes the local +Y point from world (6, 0, 0) to (4, 0, 0).
    float beyond[6] = {180.0f, 0, 0, 0, 0, 0};
    std::vector<Matrix4> posed;
    Darkness::composeSubObjectMatrices(mesh, beyond, 6, posed);
    REQUIRE(apply(posed[1], p).x == Approx(4.0f).margin(1e-4));
    REQUIRE(apply(posed[1], p).y == Approx(0.0f).margin(1e-4));
}

TEST_CASE("SubObjectPose: single-part models need no per-part transform",
          "[SubObject]") {
    ParsedBinMesh mesh = {};
    mesh.subObjects.push_back(makeRoot());
    REQUIRE_FALSE(Darkness::meshHasSubObjectTransforms(mesh));

    mesh.subObjects.push_back(makePart("@s00bb", Darkness::kSubObjRotate, 0,
                                       Vector3(1, 0, 0), Vector3(0, 1, 0),
                                       Vector3(0, 0, 1), Vector3(0.0f)));
    REQUIRE(Darkness::meshHasSubObjectTransforms(mesh));
}

TEST_CASE("SubObjectPose: meshJointsAtRest tracks only the joints the mesh uses",
          "[SubObject]") {
    ParsedBinMesh mesh = singleJointMesh(Darkness::kSubObjRotate, 2);

    float other[6] = {90.0f, 90.0f, 0.0f, 90.0f, 0, 0};
    REQUIRE(Darkness::meshJointsAtRest(mesh, other, 6));

    float mine[6] = {0, 0, 45.0f, 0, 0, 0};
    REQUIRE_FALSE(Darkness::meshJointsAtRest(mesh, mine, 6));
}

// ── Joints tweq ──

namespace {

Darkness::TweqInstance makeJointsTweq(int32_t objID, float rate,
                                      float low, float high) {
    Darkness::TweqInstance tw;
    tw.objID = objID;
    tw.type = Darkness::kTweqTypeJoints;
    tw.cfgAnim = 0;
    tw.cfgHalt = Darkness::kTweqHaltContinue;
    tw.axisCount = Darkness::kJointSlotCount;
    tw.axes[0] = {rate, low, high};
    tw.primaryAxis = 0;
    tw.active = true;
    tw.values[0] = low;
    tw.base.position = {0.0f, 0.0f, 0.0f};
    tw.base.rotation = Darkness::Matrix4(1.0f);
    tw.base.scale = {1.0f, 1.0f, 1.0f};
    // Skip the placement lookup — this test has no placement map.
    tw.hasObjectState = true;
    return tw;
}

} // namespace

TEST_CASE("TweqSystem: a joints tweq publishes its pose to ObjectState",
          "[Tweq][SubObject]") {
    Darkness::TweqSystem sys;
    Darkness::ObjectStateMap states;
    sys.injectForTest(makeJointsTweq(900, 90.0f, 0.0f, 90.0f), &states);

    sys.simStep(0.0f, 0.1f);   // 100 ms at 90 deg/s -> 90 degrees

    const Darkness::ObjectState *os = states.tryGet(900);
    REQUIRE(os != nullptr);
    REQUIRE(os->hasJoints);
    REQUIRE(os->joints[0] == Approx(90.0f).margin(1.0f));
    REQUIRE(os->joints[1] == Approx(0.0f));
}

TEST_CASE("TweqSystem: a joints tweq leaves the object transform alone",
          "[Tweq][SubObject]") {
    // Joints move parts inside the model; the object itself must not budge.
    Darkness::TweqSystem sys;
    Darkness::ObjectStateMap states;
    sys.injectForTest(makeJointsTweq(901, 45.0f, 0.0f, 90.0f), &states);

    sys.simStep(0.0f, 0.1f);

    const Darkness::ObjectState *os = states.tryGet(901);
    REQUIRE(os != nullptr);
    REQUIRE_FALSE(os->hasMatrix);
    REQUIRE(os->position.x == Approx(0.0f));
    REQUIRE(os->position.y == Approx(0.0f));
    REQUIRE(os->position.z == Approx(0.0f));
}

TEST_CASE("TweqSystem: joints tweq bounces at the configured limit",
          "[Tweq][SubObject]") {
    Darkness::TweqSystem sys;
    Darkness::ObjectStateMap states;
    auto tw = makeJointsTweq(902, 90.0f, 0.0f, 90.0f);
    tw.values[0] = 85.0f;
    sys.injectForTest(tw, &states);

    sys.simStep(0.0f, 0.1f);

    const Darkness::TweqInstance *result =
        sys.getInstanceForTest(902, Darkness::kTweqTypeJoints);
    REQUIRE(result != nullptr);
    REQUIRE(result->values[0] == Approx(90.0f));
    REQUIRE((result->axisState[0] & Darkness::kTweqStateReverse) != 0);
}

// ── Lock tweq ──

TEST_CASE("TweqSystem: lock joint selector is 1-indexed, with 0 also meaning slot 0",
          "[Tweq][SubObject]") {
    // Stock configs use 0 (lockboxl), 1 (chestloc, footlock, spinny_door) and
    // 2 (lcchest, smalsafe). chestloc's 1 must land on slot 0 — its lock plate —
    // and smalsafe's 2 on slot 1, its knob.
    REQUIRE(Darkness::TweqSystem::lockJointSlotFromConfig(0) == 0);
    REQUIRE(Darkness::TweqSystem::lockJointSlotFromConfig(1) == 0);
    REQUIRE(Darkness::TweqSystem::lockJointSlotFromConfig(2) == 1);
    REQUIRE(Darkness::TweqSystem::lockJointSlotFromConfig(3) == 2);
}

TEST_CASE("TweqSystem: a lock tweq drives only its own joint slot",
          "[Tweq][SubObject]") {
    Darkness::TweqSystem sys;
    Darkness::ObjectStateMap states;

    // Pre-pose the whole object, as JointPos seeding would.
    Darkness::ObjectState &pre = states.get(910);
    for (int i = 0; i < Darkness::ObjectState::kJointSlots; ++i)
        pre.joints[i] = 11.0f;
    pre.hasJoints = true;

    Darkness::TweqInstance tw;
    tw.objID = 910;
    tw.type = Darkness::kTweqTypeLock;
    tw.cfgHalt = Darkness::kTweqHaltContinue;
    tw.axisCount = 1;
    tw.axes[0] = {90.0f, 0.0f, 90.0f};
    tw.lockJointSlot = 2;
    tw.active = true;
    tw.values[0] = 0.0f;
    tw.base.rotation = Darkness::Matrix4(1.0f);
    tw.hasObjectState = true;
    sys.injectForTest(tw, &states);

    sys.simStep(0.0f, 0.1f);

    const Darkness::ObjectState *os = states.tryGet(910);
    REQUIRE(os != nullptr);
    REQUIRE(os->joints[2] == Approx(90.0f).margin(1.0f));
    REQUIRE(os->joints[0] == Approx(11.0f));   // untouched
    REQUIRE(os->joints[1] == Approx(11.0f));
    REQUIRE(os->joints[3] == Approx(11.0f));
}

// ── Delete tweq ──

namespace {

Darkness::TweqInstance makeDeleteTweq(int32_t objID, uint16_t rateMs,
                                      uint8_t halt) {
    Darkness::TweqInstance tw;
    tw.objID = objID;
    tw.type = Darkness::kTweqTypeDelete;
    tw.cfgHalt = halt;
    tw.cfgRate = rateMs;
    tw.active = true;
    tw.base.rotation = Darkness::Matrix4(1.0f);
    tw.hasObjectState = true;
    return tw;
}

} // namespace

TEST_CASE("TweqSystem: a delete tweq removes its object when the timer expires",
          "[Tweq]") {
    Darkness::TweqSystem sys;
    Darkness::ObjectStateMap states;
    states.get(920);   // object exists before the countdown fires
    sys.injectForTest(makeDeleteTweq(920, 500, Darkness::kTweqHaltSlay), &states);

    sys.simStep(0.0f, 0.2f);   // 200 ms — not yet
    const Darkness::TweqInstance *mid =
        sys.getInstanceForTest(920, Darkness::kTweqTypeDelete);
    REQUIRE(mid->active);
    REQUIRE((states.get(920).flags & Darkness::kObjStateDestroyed) == 0);

    sys.simStep(0.2f, 0.4f);   // 600 ms total — expired

    const Darkness::TweqInstance *done =
        sys.getInstanceForTest(920, Darkness::kTweqTypeDelete);
    REQUIRE_FALSE(done->active);
    REQUIRE((states.get(920).flags & Darkness::kObjStateDestroyed) != 0);
}

TEST_CASE("TweqSystem: destruction is announced, not just flagged", "[Tweq]") {
    // The flag alone only stops the renderer. Everything else — collision
    // bodies, the spatial index, exists() — has to be told.
    Darkness::TweqSystem sys;
    Darkness::ObjectStateMap states;
    std::vector<int32_t> destroyed;
    sys.setDestroyCallback([&destroyed](int32_t id) { destroyed.push_back(id); });
    sys.injectForTest(makeDeleteTweq(922, 100, Darkness::kTweqHaltDestroy), &states);

    sys.simStep(0.0f, 0.2f);

    REQUIRE(destroyed.size() == 1);
    REQUIRE(destroyed[0] == 922);
}

TEST_CASE("TweqSystem: a Stop-halt delete does not announce destruction",
          "[Tweq]") {
    Darkness::TweqSystem sys;
    Darkness::ObjectStateMap states;
    std::vector<int32_t> destroyed;
    sys.setDestroyCallback([&destroyed](int32_t id) { destroyed.push_back(id); });
    sys.injectForTest(makeDeleteTweq(923, 100, Darkness::kTweqHaltStop), &states);

    sys.simStep(0.0f, 0.2f);

    REQUIRE(destroyed.empty());
}

TEST_CASE("TweqSystem: a halted delete tweq leaves the object alone", "[Tweq]") {
    // halt action Stop (2) is the stock 'rat01' setting: the countdown ends,
    // the object stays.
    Darkness::TweqSystem sys;
    Darkness::ObjectStateMap states;
    states.get(921);
    sys.injectForTest(makeDeleteTweq(921, 100, Darkness::kTweqHaltStop), &states);

    sys.simStep(0.0f, 0.2f);

    REQUIRE_FALSE(sys.getInstanceForTest(921, Darkness::kTweqTypeDelete)->active);
    REQUIRE((states.get(921).flags & Darkness::kObjStateDestroyed) == 0);
}

TEST_CASE("TweqSystem: animStateBits reports the live instance state",
          "[Tweq][SubObject]") {
    // Scripts read this instead of the StTweq* property, whose field accessor
    // always failed and so always answered "no bits set".
    Darkness::TweqSystem sys;
    Darkness::ObjectStateMap states;
    auto tw = makeJointsTweq(903, 90.0f, 0.0f, 90.0f);
    tw.active = false;
    sys.injectForTest(tw, &states);

    REQUIRE((sys.animStateBits(903) & Darkness::kTweqStateOn) == 0);

    sys.activate(903, Darkness::kTweqDoActivate);
    REQUIRE((sys.animStateBits(903) & Darkness::kTweqStateOn) != 0);

    sys.activate(903, Darkness::kTweqDoReverse);
    REQUIRE((sys.animStateBits(903) & Darkness::kTweqStateReverse) != 0);
}
