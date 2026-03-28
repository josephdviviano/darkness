// Unit tests for ObjectState, ObjectStateMap, ValueChangeRequest, and
// SpatialIndex::update() — pure value types with no service dependencies.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>

#include "ValueChangeRequest.h"
#include "worldquery/ObjectState.h"
#include "worldquery/SpatialIndex.h"

using namespace Darkness;
using Catch::Approx;

static constexpr float kPi = 3.14159265f;

// ============================================================================
// ValueChangeRequest
// ============================================================================

TEST_CASE("ValueChangeRequest: set stores correct value", "[ValueChangeRequest]") {
    ValueChangeRequest<int> req;
    req.set(42);
    REQUIRE(req.requestedVal() == 42);
}

TEST_CASE("ValueChangeRequest: getIfReq returns true and sets target", "[ValueChangeRequest]") {
    ValueChangeRequest<int> req;
    req.set(99);
    int target = 0;
    REQUIRE(req.getIfReq(target) == true);
    REQUIRE(target == 99);

    // Consumed — second call returns false
    int target2 = 0;
    REQUIRE(req.getIfReq(target2) == false);
    REQUIRE(target2 == 0);  // unchanged
}

TEST_CASE("ValueChangeRequest: getIfReq returns false when not requested", "[ValueChangeRequest]") {
    ValueChangeRequest<int> req;
    int target = 77;
    REQUIRE(req.getIfReq(target) == false);
    REQUIRE(target == 77);  // unchanged
}

TEST_CASE("ValueChangeRequest: last set wins", "[ValueChangeRequest]") {
    ValueChangeRequest<int> req;
    req.set(1);
    req.set(2);
    req.set(3);
    int target = 0;
    REQUIRE(req.getIfReq(target) == true);
    REQUIRE(target == 3);
}

TEST_CASE("ValueChangeRequest: set after consume creates new request", "[ValueChangeRequest]") {
    ValueChangeRequest<int> req;
    req.set(10);
    int t1 = 0;
    req.getIfReq(t1);
    REQUIRE(t1 == 10);

    req.set(20);
    int t2 = 0;
    REQUIRE(req.getIfReq(t2) == true);
    REQUIRE(t2 == 20);
}

TEST_CASE("ValueChangeRequest: works with float", "[ValueChangeRequest]") {
    ValueChangeRequest<float> req;
    req.set(3.14f);
    float t = 0.0f;
    REQUIRE(req.getIfReq(t) == true);
    REQUIRE(t == Approx(3.14f));
}

TEST_CASE("ValueChangeRequest: works with bool", "[ValueChangeRequest]") {
    ValueChangeRequest<bool> req;
    req.set(true);
    bool t = false;
    REQUIRE(req.getIfReq(t) == true);
    REQUIRE(t == true);
}

// ============================================================================
// ObjectState
// ============================================================================

TEST_CASE("ObjectState: setTransform stores position and angles", "[ObjectState]") {
    ObjectState os;
    os.setTransform({10.0f, 20.0f, 30.0f}, 1.0f, 0.5f, 0.25f);

    REQUIRE(os.position.x == Approx(10.0f));
    REQUIRE(os.position.y == Approx(20.0f));
    REQUIRE(os.position.z == Approx(30.0f));
    REQUIRE(os.heading == Approx(1.0f));
    REQUIRE(os.pitch == Approx(0.5f));
    REQUIRE(os.bank == Approx(0.25f));

    // Quaternion should not be identity (angles are non-zero)
    Quaternion identity(1.0f, 0.0f, 0.0f, 0.0f);
    bool isIdentity = (std::abs(os.orientation.w - identity.w) < 1e-5f &&
                        std::abs(os.orientation.x - identity.x) < 1e-5f &&
                        std::abs(os.orientation.y - identity.y) < 1e-5f &&
                        std::abs(os.orientation.z - identity.z) < 1e-5f);
    REQUIRE_FALSE(isIdentity);
}

TEST_CASE("ObjectState: setAngles recomputes quaternion", "[ObjectState]") {
    ObjectState os;

    // Zero angles → identity quaternion
    os.setAngles(0.0f, 0.0f, 0.0f);
    REQUIRE(os.orientation.w == Approx(1.0f).margin(1e-5f));
    REQUIRE(os.orientation.x == Approx(0.0f).margin(1e-5f));
    REQUIRE(os.orientation.y == Approx(0.0f).margin(1e-5f));
    REQUIRE(os.orientation.z == Approx(0.0f).margin(1e-5f));

    // 90-degree heading (Z rotation) — compare against known quaternion
    os.setAngles(kPi / 2.0f, 0.0f, 0.0f);
    Matrix4 expected = glm::eulerAngleZYX(kPi / 2.0f, 0.0f, 0.0f);
    Quaternion expectedQ = glm::quat_cast(expected);
    REQUIRE(os.orientation.w == Approx(expectedQ.w).margin(1e-4f));
    REQUIRE(os.orientation.x == Approx(expectedQ.x).margin(1e-4f));
    REQUIRE(os.orientation.y == Approx(expectedQ.y).margin(1e-4f));
    REQUIRE(os.orientation.z == Approx(expectedQ.z).margin(1e-4f));
}

TEST_CASE("ObjectState: initFromBinaryRadians converts correctly", "[ObjectState]") {
    ObjectState os;

    // 16384 binary radians = 90 degrees = pi/2
    os.initFromBinaryRadians(1.0f, 2.0f, 3.0f, 16384, 0, 0);
    REQUIRE(os.position.x == Approx(1.0f));
    REQUIRE(os.position.y == Approx(2.0f));
    REQUIRE(os.position.z == Approx(3.0f));
    REQUIRE(os.heading == Approx(kPi / 2.0f).margin(1e-4f));
    REQUIRE(os.pitch == Approx(0.0f));
    REQUIRE(os.bank == Approx(0.0f));
    REQUIRE(os.scale == Vector3(1.0f, 1.0f, 1.0f));

    // Custom scale
    os.initFromBinaryRadians(0, 0, 0, 0, 0, 0, 2.0f, 3.0f, 4.0f);
    REQUIRE(os.scale == Vector3(2.0f, 3.0f, 4.0f));

    // Negative binary radians (-16384 = -90 degrees = -pi/2)
    os.initFromBinaryRadians(0, 0, 0, -16384, 0, 0);
    REQUIRE(os.heading == Approx(-kPi / 2.0f).margin(1e-4f));

    // Full rotation (32768 = 180 degrees = pi)
    os.initFromBinaryRadians(0, 0, 0, 32767, 0, 0);
    REQUIRE(os.heading == Approx(kPi).margin(0.01f));
}

// ============================================================================
// ObjectStateMap
// ============================================================================

TEST_CASE("ObjectStateMap: has/tryGet/get/set/remove/clear", "[ObjectState]") {
    ObjectStateMap map;

    // Initial state
    REQUIRE(map.has(42) == false);
    REQUIRE(map.tryGet(42) == nullptr);
    REQUIRE(map.size() == 0);

    // Set
    ObjectState state;
    state.position = {1.0f, 2.0f, 3.0f};
    map.set(42, state);
    REQUIRE(map.has(42) == true);
    REQUIRE(map.tryGet(42) != nullptr);
    REQUIRE(map.tryGet(42)->position.x == Approx(1.0f));
    REQUIRE(map.size() == 1);

    // Remove
    map.remove(42);
    REQUIRE(map.has(42) == false);
    REQUIRE(map.tryGet(42) == nullptr);
    REQUIRE(map.size() == 0);

    // Clear
    ObjectState s1, s2;
    map.set(1, s1);
    map.set(2, s2);
    REQUIRE(map.size() == 2);
    map.clear();
    REQUIRE(map.size() == 0);
    REQUIRE(map.has(1) == false);
    REQUIRE(map.has(2) == false);
}

TEST_CASE("ObjectStateMap: get() creates default entry", "[ObjectState]") {
    ObjectStateMap map;
    REQUIRE(map.has(99) == false);

    // get() auto-inserts a default ObjectState
    ObjectState &os = map.get(99);
    REQUIRE(map.has(99) == true);
    REQUIRE(map.size() == 1);

    // Default values
    REQUIRE(os.position == Vector3(0.0f));
    REQUIRE(os.heading == 0.0f);
    REQUIRE(os.scale == Vector3(1.0f, 1.0f, 1.0f));
    REQUIRE(os.flags == kObjStateActive);
}

// ============================================================================
// SpatialIndex::update()
// ============================================================================

TEST_CASE("SpatialIndex: update moves entity to new cell", "[SpatialIndex]") {
    SpatialIndex si(32.0f);
    si.insert(1, {0.0f, 0.0f, 0.0f});

    // Move far enough to cross a cell boundary
    si.update(1, {100.0f, 0.0f, 0.0f});

    auto found = si.queryRadius({100.0f, 0.0f, 0.0f}, 1.0f);
    REQUIRE(found.size() == 1);
    REQUIRE(found[0] == 1);

    auto oldPos = si.queryRadius({0.0f, 0.0f, 0.0f}, 1.0f);
    REQUIRE(oldPos.empty());
}

TEST_CASE("SpatialIndex: update no rehash when same cell", "[SpatialIndex]") {
    SpatialIndex si(32.0f);
    si.insert(1, {5.0f, 5.0f, 5.0f});
    size_t cellsBefore = si.cellCount();

    // Move within the same cell (both < 32)
    si.update(1, {6.0f, 5.0f, 5.0f});
    REQUIRE(si.cellCount() == cellsBefore);

    // Should be found at new position
    auto found = si.queryRadius({6.0f, 5.0f, 5.0f}, 0.5f);
    REQUIRE(found.size() == 1);
    REQUIRE(found[0] == 1);
}

TEST_CASE("SpatialIndex: update is no-op for uninserted entity", "[SpatialIndex]") {
    SpatialIndex si(32.0f);
    si.update(999, {50.0f, 50.0f, 50.0f});
    REQUIRE(si.size() == 0);
    auto found = si.queryRadius({50.0f, 50.0f, 50.0f}, 100.0f);
    REQUIRE(found.empty());
}

TEST_CASE("SpatialIndex: queryRadius finds entity after update", "[SpatialIndex]") {
    SpatialIndex si(32.0f);
    si.insert(1, {0.0f, 0.0f, 0.0f});
    si.insert(2, {10.0f, 0.0f, 0.0f});
    si.insert(3, {200.0f, 0.0f, 0.0f});

    // Move entity 3 close to entity 2
    si.update(3, {11.0f, 0.0f, 0.0f});

    auto nearby = si.queryRadius({10.0f, 0.0f, 0.0f}, 5.0f);
    // Should find entities 2 and 3, but not 1 (distance 10 > 5)
    REQUIRE(nearby.size() == 2);
    bool has2 = std::find(nearby.begin(), nearby.end(), 2) != nearby.end();
    bool has3 = std::find(nearby.begin(), nearby.end(), 3) != nearby.end();
    REQUIRE(has2);
    REQUIRE(has3);
}

TEST_CASE("SpatialIndex: update cleans up empty old cell", "[SpatialIndex]") {
    SpatialIndex si(32.0f);
    si.insert(1, {0.0f, 0.0f, 0.0f});
    REQUIRE(si.cellCount() == 1);

    // Move to a different cell — old cell should be cleaned up
    si.update(1, {100.0f, 0.0f, 0.0f});
    REQUIRE(si.cellCount() == 1);  // old empty cell removed, new cell created
}
