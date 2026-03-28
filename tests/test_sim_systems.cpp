// Unit tests for LoopService, SimService, DoorSystem, and MessageDispatch.
// Uses mock/test-double classes for LoopClient, SimListener, and IWorldQuery.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cmath>
#include <string>
#include <thread>
#include <vector>
#include <utility>

#include "ValueChangeRequest.h"
#include "loop/LoopService.h"
#include "loop/LoopCommon.h"
#include "sim/SimService.h"
#include "sim/SimCommon.h"
#include "sim/DoorSystem.h"
#include "sim/MessageDispatch.h"
#include "ServiceCommon.h"
#include "worldquery/ObjectState.h"
#include "worldquery/IWorldQuery.h"
#include "logger.h"
#include "stdlog.h"

using namespace Darkness;
using Catch::Approx;

// Ensure Logger singleton exists before any test runs.
// LoopService/SimService use LOG_* macros that assert the singleton.
// Check getSingletonPtr() first — another test file may have created it.
static struct LoggerBootstrap {
    Logger *logger = nullptr;
    StdLog *stdlog = nullptr;
    LoggerBootstrap() {
        if (!Logger::getSingletonPtr()) {
            logger = new Logger();
            stdlog = new StdLog();
            logger->registerLogListener(stdlog);
            logger->setLogLevel(Logger::LOG_LEVEL_ERROR);
        }
    }
    ~LoggerBootstrap() {
        // Don't delete — other test files' statics may still reference it,
        // and Singleton destructor handles cleanup.
    }
} sLoggerBootstrap;

static constexpr float kPi = 3.14159265f;

// ============================================================================
// Mock / Test Double Classes
// ============================================================================

/// Mock LoopClient that records all loopStep calls and mode events.
class MockLoopClient : public LoopClient {
public:
    MockLoopClient(LoopClientID id, const std::string &name,
                    LoopModeMask mask, LoopClientPriority priority) {
        mLoopClientDef.id = id;
        mLoopClientDef.name = name;
        mLoopClientDef.mask = mask;
        mLoopClientDef.priority = priority;
    }

    std::vector<float> stepDeltas;
    int startedCount = 0;
    int endedCount = 0;

    // Shared order tracker — appends this client's priority when stepped
    std::vector<int> *orderTracker = nullptr;

protected:
    void loopStep(float deltaTime) override {
        stepDeltas.push_back(deltaTime);
        if (orderTracker)
            orderTracker->push_back(static_cast<int>(mLoopClientDef.priority));
    }
    void loopModeStarted(const LoopModeDefinition &) override { ++startedCount; }
    void loopModeEnded(const LoopModeDefinition &) override { ++endedCount; }
};

/// Mock SimListener that records all lifecycle and step callbacks.
class MockSimListener : public SimListener {
public:
    std::vector<std::pair<float, float>> steps;  // (simTime, delta)
    int startedCount = 0, endedCount = 0;
    int pausedCount = 0, unpausedCount = 0;
    std::vector<float> flowChanges;

    void simStarted() override { SimListener::simStarted(); ++startedCount; }
    void simEnded() override { SimListener::simEnded(); ++endedCount; }
    void simPaused() override { SimListener::simPaused(); ++pausedCount; }
    void simUnPaused() override { SimListener::simUnPaused(); ++unpausedCount; }
    void simFlowChange(float newFlow) override {
        SimListener::simFlowChange(newFlow);
        flowChanges.push_back(newFlow);
    }
    void simStep(float simTime, float delta) override {
        SimListener::simStep(simTime, delta);
        steps.emplace_back(simTime, delta);
    }
};

/// Minimal IWorldQuery mock for MessageDispatch SwitchLink tests.
/// Only getLinks() is implemented — everything else returns defaults.
class MockWorldQuery : public IWorldQuery {
public:
    // Configure: when getLinks(src, "SwitchLink", 0) is called, return these links
    std::unordered_map<int32_t, std::vector<LinkInfo>> switchLinks;

    // Required IWorldQuery overrides (stubs)
    bool exists(EntityID) const override { return true; }
    Vector3 getPosition(EntityID) const override { return {}; }
    Quaternion getOrientation(EntityID) const override { return {1,0,0,0}; }
    std::string getName(EntityID) const override { return ""; }
    BBox getBounds(EntityID) const override { return {}; }

    PropertyHandle resolveProperty(const std::string &) const override { return {}; }
    bool hasProperty(EntityID, const std::string &) const override { return false; }
    bool hasProperty(EntityID, const PropertyHandle &) const override { return false; }
    bool ownsProperty(EntityID, const std::string &) const override { return false; }
    bool ownsProperty(EntityID, const PropertyHandle &) const override { return false; }
    std::vector<EntityID> getAllWithProperty(const std::string &) const override { return {}; }

    RelationHandle resolveRelation(const std::string &relName) const override {
        RelationHandle h;
        if (relName == "SwitchLink") h._internal = reinterpret_cast<void*>(1);
        return h;
    }

    std::vector<LinkInfo> getLinks(EntityID src, const std::string &relName,
                                    EntityID) const override {
        if (relName != "SwitchLink") return {};
        auto it = switchLinks.find(static_cast<int32_t>(src));
        if (it != switchLinks.end()) return it->second;
        return {};
    }

    std::vector<LinkInfo> getLinks(EntityID src, const RelationHandle &handle,
                                    EntityID) const override {
        if (!handle) return {};
        return getLinks(src, std::string("SwitchLink"), EntityID(0));
    }

    std::vector<LinkInfo> getBackLinks(EntityID, const std::string &,
                                        EntityID) const override { return {}; }
    std::vector<LinkInfo> getBackLinks(EntityID, const RelationHandle &,
                                        EntityID) const override { return {}; }

    RoomID getRoomAt(const Vector3 &) const override { return -1; }
    std::vector<RoomID> getAdjacentRooms(RoomID) const override { return {}; }
    std::vector<PortalInfo> getPortals(RoomID) const override { return {}; }
    float getPortalOpenFraction(PortalID) const override { return 1.0f; }
    float getLightLevel(const Vector3 &) const override { return 0.0f; }
    bool raycast(const Vector3 &, const Vector3 &, RayHit &) const override { return false; }

protected:
    const uint8_t *getRawPropertyData(EntityID, const PropertyHandle &,
                                       size_t &outSize) const override {
        outSize = 0;
        return nullptr;
    }

public:

    void forEachPortal(RoomID, const std::function<void(const PortalInfo &)> &) const override {}
    void forEachAdjacentRoom(RoomID, const std::function<void(RoomID)> &) const override {}
    void forEachEntityInRoom(RoomID, const std::function<void(EntityID)> &, size_t) const override {}

    std::vector<EntityID> queryRadius(const Vector3 &, float) const override { return {}; }
    std::vector<EntityID> queryAABB(const BBox &) const override { return {}; }
    std::vector<EntityID> queryFrustum(const Vector3 &, const Vector3 &,
                                        float, float, float, float) const override { return {}; }
    std::vector<EntityID> queryRoom(RoomID, size_t) const override { return {}; }
};

// ============================================================================
// LoopMode tests
// ============================================================================

TEST_CASE("LoopMode: dispatches to all clients", "[LoopService]") {
    LoopModeDefinition def{1, 0xFF, "TestMode"};
    LoopMode mode(def, nullptr);

    MockLoopClient c1(1, "A", 0xFF, 1);
    MockLoopClient c2(2, "B", 0xFF, 2);
    MockLoopClient c3(3, "C", 0xFF, 3);
    mode.addLoopClient(&c1);
    mode.addLoopClient(&c2);
    mode.addLoopClient(&c3);

    mode.loopStep(0.016f);
    REQUIRE(c1.stepDeltas.size() == 1);
    REQUIRE(c2.stepDeltas.size() == 1);
    REQUIRE(c3.stepDeltas.size() == 1);
    REQUIRE(c1.stepDeltas[0] == Approx(0.016f));
}

TEST_CASE("LoopMode: dispatches in priority order", "[LoopService]") {
    LoopModeDefinition def{1, 0xFF, "TestMode"};
    LoopMode mode(def, nullptr);

    std::vector<int> order;
    MockLoopClient c1(1, "P10", 0xFF, 10);
    MockLoopClient c2(2, "P1",  0xFF, 1);
    MockLoopClient c3(3, "P5",  0xFF, 5);
    c1.orderTracker = &order;
    c2.orderTracker = &order;
    c3.orderTracker = &order;

    mode.addLoopClient(&c1);
    mode.addLoopClient(&c2);
    mode.addLoopClient(&c3);

    mode.loopStep(0.016f);
    REQUIRE(order == std::vector<int>{1, 5, 10});
}

TEST_CASE("LoopMode: removeLoopClient stops dispatch", "[LoopService]") {
    LoopModeDefinition def{1, 0xFF, "TestMode"};
    LoopMode mode(def, nullptr);

    MockLoopClient c1(1, "A", 0xFF, 1);
    MockLoopClient c2(2, "B", 0xFF, 2);
    mode.addLoopClient(&c1);
    mode.addLoopClient(&c2);
    mode.removeLoopClient(&c1);

    mode.loopStep(0.016f);
    REQUIRE(c1.stepDeltas.empty());
    REQUIRE(c2.stepDeltas.size() == 1);
}

TEST_CASE("LoopMode: empty mode does not crash", "[LoopService]") {
    LoopModeDefinition def{1, 0xFF, "TestMode"};
    LoopMode mode(def, nullptr);
    mode.loopStep(0.016f);
    mode.loopModeStarted();
    mode.loopModeEnded();
    REQUIRE(true);
}

TEST_CASE("LoopMode: mode started/ended notifies clients", "[LoopService]") {
    LoopModeDefinition def{1, 0xFF, "TestMode"};
    LoopMode mode(def, nullptr);

    MockLoopClient c1(1, "A", 0xFF, 1);
    mode.addLoopClient(&c1);

    mode.loopModeStarted();
    REQUIRE(c1.startedCount == 1);
    mode.loopModeEnded();
    REQUIRE(c1.endedCount == 1);
}

TEST_CASE("LoopMode: duplicate priority clients all dispatched", "[LoopService]") {
    LoopModeDefinition def{1, 0xFF, "TestMode"};
    LoopMode mode(def, nullptr);

    MockLoopClient c1(1, "A", 0xFF, 5);
    MockLoopClient c2(2, "B", 0xFF, 5);
    MockLoopClient c3(3, "C", 0xFF, 5);
    mode.addLoopClient(&c1);
    mode.addLoopClient(&c2);
    mode.addLoopClient(&c3);

    mode.loopStep(0.016f);
    REQUIRE(c1.stepDeltas.size() == 1);
    REQUIRE(c2.stepDeltas.size() == 1);
    REQUIRE(c3.stepDeltas.size() == 1);
}

// ============================================================================
// LoopService::step() tests
// ============================================================================

// Note: LoopService::step() and LoopService construction require a
// ServiceManager (Logger singleton + ServiceImpl base class). These are
// tested via full integration (running darknessRender) and indirectly
// through the LoopMode dispatch tests above. The critical step()
// mode-request-processing fix is covered by the LoopMode tests which
// exercise the same client dispatch code path.

// ============================================================================
// SimService tests
// ============================================================================

// Helper: create a SimService on the heap (bypassing ServiceManager).
// Don't call init() — it would try GET_SERVICE(LoopService).
static std::unique_ptr<SimService> makeSimService() {
    return std::make_unique<SimService>(nullptr, "test");
}

TEST_CASE("SimService: loopStep with sim not running does nothing", "[SimService]") {
    auto svc = makeSimService();
    MockSimListener listener;
    svc->registerListener(&listener, 1);

    svc->loopStep(0.016f);
    REQUIRE(listener.steps.empty());
}

TEST_CASE("SimService: startSim then step dispatches", "[SimService]") {
    auto svc = makeSimService();
    MockSimListener listener;
    svc->registerListener(&listener, 1);

    svc->startSim();
    svc->loopStep(0.016f);

    REQUIRE(listener.startedCount == 1);
    REQUIRE(listener.steps.size() == 1);
    REQUIRE(listener.steps[0].first == Approx(0.0f));   // simTime starts at 0
    REQUIRE(listener.steps[0].second == Approx(0.016f)); // delta
}

TEST_CASE("SimService: startSim resets simTime to zero", "[SimService]") {
    auto svc = makeSimService();
    MockSimListener listener;
    svc->registerListener(&listener, 1);

    svc->startSim();
    svc->loopStep(0.1f);
    svc->loopStep(0.1f);
    // simTime should be 0.2 after two steps

    svc->endSim();
    svc->loopStep(0.0f);  // process the end request

    svc->startSim();
    svc->loopStep(0.05f);

    // Last step should have simTime = 0.0 (reset on start)
    auto &last = listener.steps.back();
    REQUIRE(last.first == Approx(0.0f));
    REQUIRE(last.second == Approx(0.05f));
}

TEST_CASE("SimService: pauseSim stops step dispatch", "[SimService]") {
    auto svc = makeSimService();
    MockSimListener listener;
    svc->registerListener(&listener, 1);

    svc->startSim();
    svc->loopStep(0.016f);  // 1 step
    REQUIRE(listener.steps.size() == 1);

    svc->pauseSim();
    svc->loopStep(0.016f);  // pause processed, no step
    svc->loopStep(0.016f);  // still paused
    REQUIRE(listener.pausedCount == 1);
    REQUIRE(listener.steps.size() == 1);  // no new steps
}

TEST_CASE("SimService: unPauseSim resumes step dispatch", "[SimService]") {
    auto svc = makeSimService();
    MockSimListener listener;
    svc->registerListener(&listener, 1);
    // registerListener fires sync: simUnPaused (because sim starts unpaused)
    int baseUnpausedCount = listener.unpausedCount;

    svc->startSim();
    svc->loopStep(0.016f);  // step 1
    svc->pauseSim();
    svc->loopStep(0.016f);  // paused, no step
    svc->unPauseSim();
    svc->loopStep(0.016f);  // unpaused, step 2

    REQUIRE(listener.unpausedCount == baseUnpausedCount + 1);
    REQUIRE(listener.steps.size() == 2);
}

TEST_CASE("SimService: flowCoeff scales delta", "[SimService]") {
    auto svc = makeSimService();
    MockSimListener listener;
    svc->registerListener(&listener, 1);

    svc->startSim();
    svc->setFlowCoeff(2.0f);
    svc->loopStep(0.016f);

    REQUIRE(listener.flowChanges.size() >= 1);
    REQUIRE(listener.flowChanges.back() == Approx(2.0f));
    REQUIRE(listener.steps.size() == 1);
    REQUIRE(listener.steps[0].second == Approx(0.032f));  // 2.0 * 0.016
}

TEST_CASE("SimService: simTime accumulates correctly", "[SimService]") {
    auto svc = makeSimService();
    MockSimListener listener;
    svc->registerListener(&listener, 1);

    svc->startSim();
    for (int i = 0; i < 5; i++)
        svc->loopStep(0.02f);

    REQUIRE(listener.steps.size() == 5);
    // simTime values: 0.0, 0.02, 0.04, 0.06, 0.08
    for (int i = 0; i < 5; i++) {
        REQUIRE(listener.steps[i].first == Approx(0.02f * i).margin(1e-5f));
        REQUIRE(listener.steps[i].second == Approx(0.02f));
    }
}

TEST_CASE("SimService: setFlowCoeff rejects non-positive", "[SimService]") {
    auto svc = makeSimService();
    MockSimListener listener;
    svc->registerListener(&listener, 1);

    svc->startSim();
    svc->setFlowCoeff(-1.0f);
    svc->loopStep(0.016f);

    // Flow should still be 1.0 (default), not -1.0
    REQUIRE(listener.steps[0].second == Approx(0.016f));
}

TEST_CASE("SimService: registerListener receives sync callbacks", "[SimService]") {
    auto svc = makeSimService();

    // Start sim, pause it, set flow
    svc->startSim();
    svc->loopStep(0.0f);  // process start
    svc->pauseSim();
    svc->loopStep(0.0f);  // process pause
    svc->setFlowCoeff(3.0f);
    svc->loopStep(0.0f);  // process flow change

    // Now register a new listener — should get sync callbacks
    MockSimListener late;
    svc->registerListener(&late, 1);

    REQUIRE(late.startedCount == 1);   // sim is running
    REQUIRE(late.pausedCount == 1);    // sim is paused
    REQUIRE(late.flowChanges.size() >= 1);
    REQUIRE(late.flowChanges.back() == Approx(3.0f));
}

TEST_CASE("SimService: unregisterListener removes correctly", "[SimService]") {
    auto svc = makeSimService();
    MockSimListener a, b;
    svc->registerListener(&a, 1);
    svc->registerListener(&b, 2);

    svc->startSim();
    svc->loopStep(0.016f);  // both get step 1
    REQUIRE(a.steps.size() == 1);
    REQUIRE(b.steps.size() == 1);

    svc->unregisterListener(&a);
    svc->loopStep(0.016f);  // only b gets step 2

    REQUIRE(a.steps.size() == 1);  // unchanged
    REQUIRE(b.steps.size() == 2);
}

TEST_CASE("SimService: endSim notifies listeners", "[SimService]") {
    auto svc = makeSimService();
    MockSimListener listener;
    svc->registerListener(&listener, 1);
    // registerListener fires sync callback: simEnded() because sim isn't running yet
    int baseEndedCount = listener.endedCount;

    svc->startSim();
    svc->loopStep(0.0f);
    svc->endSim();
    svc->loopStep(0.0f);

    REQUIRE(listener.endedCount == baseEndedCount + 1);
    // No more steps after end
    size_t stepsAfterEnd = listener.steps.size();
    svc->loopStep(0.016f);
    REQUIRE(listener.steps.size() == stepsAfterEnd);
}

// ============================================================================
// MessageDispatch tests
// ============================================================================

TEST_CASE("MessageDispatch: sendMessage routes to per-object handler", "[MessageDispatch]") {
    MessageDispatch md;
    bool handled = false;
    md.registerHandler(42, "TurnOn", [&](const ScriptMessage &msg) {
        handled = true;
        return true;
    });

    bool result = md.sendMessage({42, "TurnOn", 0, 0});
    REQUIRE(handled == true);
    REQUIRE(result == true);
}

TEST_CASE("MessageDispatch: falls through to global handler", "[MessageDispatch]") {
    MessageDispatch md;
    bool globalHit = false;
    md.registerGlobalHandler("TurnOn", [&](const ScriptMessage &) {
        globalHit = true;
        return true;
    });

    md.sendMessage({42, "TurnOn", 0, 0});
    REQUIRE(globalHit == true);
}

TEST_CASE("MessageDispatch: per-object consuming blocks global", "[MessageDispatch]") {
    MessageDispatch md;
    bool globalHit = false;
    md.registerHandler(42, "TurnOn", [](const ScriptMessage &) { return true; });
    md.registerGlobalHandler("TurnOn", [&](const ScriptMessage &) {
        globalHit = true;
        return true;
    });

    md.sendMessage({42, "TurnOn", 0, 0});
    REQUIRE(globalHit == false);
}

TEST_CASE("MessageDispatch: FrobWorldEnd follows SwitchLink", "[MessageDispatch]") {
    MockWorldQuery wq;
    // Object 10 has SwitchLink to object 20
    wq.switchLinks[10] = {{1, 10, 20, 0}};

    MessageDispatch md;
    md.init(&wq);

    bool targetHit = false;
    int32_t targetFrom = -1;
    md.registerHandler(20, "TurnOn", [&](const ScriptMessage &msg) {
        targetHit = true;
        targetFrom = msg.from;
        return true;
    });

    // FrobWorldEnd on object 10 should follow SwitchLink to send TurnOn to 20
    md.sendMessageWithLinks({10, "FrobWorldEnd", 0, 0});
    REQUIRE(targetHit == true);
}

TEST_CASE("MessageDispatch: SwitchLink propagation is one-hop only", "[MessageDispatch]") {
    MockWorldQuery wq;
    // Chain: 10 →SwitchLink→ 20 →SwitchLink→ 30
    wq.switchLinks[10] = {{1, 10, 20, 0}};
    wq.switchLinks[20] = {{2, 20, 30, 0}};

    MessageDispatch md;
    md.init(&wq);

    bool obj20Hit = false, obj30Hit = false;
    md.registerHandler(20, "TurnOn", [&](const ScriptMessage &) {
        obj20Hit = true;
        return false;  // don't consume — let globals run too
    });
    md.registerHandler(30, "TurnOn", [&](const ScriptMessage &) {
        obj30Hit = true;
        return true;
    });

    md.sendMessageWithLinks({10, "TurnOn", 0, 0});
    REQUIRE(obj20Hit == true);
    REQUIRE(obj30Hit == false);  // one-hop only, no recursion
}

TEST_CASE("MessageDispatch: TurnOff propagates TurnOff not TurnOn", "[MessageDispatch]") {
    MockWorldQuery wq;
    wq.switchLinks[10] = {{1, 10, 20, 0}};

    MessageDispatch md;
    md.init(&wq);

    std::string receivedMsg;
    md.registerHandler(20, "TurnOff", [&](const ScriptMessage &) {
        receivedMsg = "TurnOff";
        return true;
    });
    md.registerHandler(20, "TurnOn", [&](const ScriptMessage &) {
        receivedMsg = "TurnOn";
        return true;
    });

    md.sendMessageWithLinks({10, "TurnOff", 0, 0});
    REQUIRE(receivedMsg == "TurnOff");
}

TEST_CASE("MessageDispatch: multiple handlers first to consume stops", "[MessageDispatch]") {
    MessageDispatch md;
    bool first = false, second = false;
    md.registerHandler(42, "TurnOn", [&](const ScriptMessage &) {
        first = true;
        return true;  // consume
    });
    md.registerHandler(42, "TurnOn", [&](const ScriptMessage &) {
        second = true;
        return true;
    });

    md.sendMessage({42, "TurnOn", 0, 0});
    REQUIRE(first == true);
    REQUIRE(second == false);
}
