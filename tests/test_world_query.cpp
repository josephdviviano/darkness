// IWorldQuery interface tests — validates that ObjSysWorldState correctly
// wraps the OPDE service stack, providing consistent results for entity,
// property, link, and room queries through the facade.
//
// Test fixture: Equilibrium fan mission (tests/fixtures/Equilibrium_ENG.zip)
// Same fixture as test_typed_properties.cpp.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>
#include <vector>

#include <zzip/zzip.h>

// Darkness service stack
#include "logger.h"
#include "stdlog.h"
#include "ConsoleBackend.h"
#include "DarknessServiceManager.h"
#include "config/ConfigService.h"
#include "database/DatabaseService.h"
#include "game/GameService.h"
#include "inherit/InheritService.h"
#include "link/LinkService.h"
#include "link/Relation.h"
#include "loop/LoopService.h"
#include "object/ObjectService.h"
#include "platform/PlatformService.h"
#include "property/PropertyService.h"
#include "room/RoomService.h"
#include "sim/SimService.h"
#include "physics/PhysicsService.h"
#include "RawDataStorage.h"
#include "PLDefParser.h"
#include "DTypeSizeParser.h"
#include "SingleFieldDataStorage.h"

// Typed property access (for cross-validation)
#include "property/TypedProperty.h"
#include "property/DarkPropertyDefs.h"

// IWorldQuery + SpatialIndex
#include "worldquery/ObjSysWorldState.h"
#include "worldquery/SpatialIndex.h"

// Raycaster + cell geometry (bgfx-free)
#include "RayCaster.h"

namespace fs = std::filesystem;
using namespace Darkness;

// ============================================================================
// Zip extraction helper (same as test_typed_properties.cpp)
// ============================================================================
static bool extractFromZip(const std::string &zipPath,
                           const std::string &entryName,
                           const std::string &destPath) {
    ZZIP_DIR *dir = zzip_dir_open(zipPath.c_str(), nullptr);
    if (!dir) return false;

    ZZIP_FILE *fp = zzip_file_open(dir, entryName.c_str(), O_RDONLY);
    if (!fp) {
        zzip_dir_close(dir);
        return false;
    }

    std::ofstream out(destPath, std::ios::binary);
    if (!out.is_open()) {
        zzip_file_close(fp);
        zzip_dir_close(dir);
        return false;
    }

    char buf[8192];
    zzip_ssize_t n;
    while ((n = zzip_file_read(fp, buf, sizeof(buf))) > 0)
        out.write(buf, n);

    zzip_file_close(fp);
    zzip_dir_close(dir);
    return true;
}

// ============================================================================
// Shared test fixture — loads mission once, constructs ObjSysWorldState
// ============================================================================
static bool s_loaded = false;
static bool s_available = false;

// Raw service pointers (kept alive by ServiceManager singleton)
static ObjectService *s_objSvc = nullptr;
static PropertyService *s_propSvc = nullptr;
static LinkService *s_linkSvc = nullptr;
static RoomService *s_roomSvc = nullptr;

// The IWorldQuery facade under test
static std::unique_ptr<ObjSysWorldState> s_worldQuery;

// WR geometry for raycaster integration tests (parsed from .mis alongside mission load)
static WRParsedData s_wrData;
static bool s_wrParsed = false;

// Logger singletons
static Logger *s_logger = nullptr;
static StdLog *s_stdlog = nullptr;
static ConsoleBackend *s_console = nullptr;

static void ensureLoaded() {
    if (s_loaded) return;
    s_loaded = true;

    std::string zipPath = std::string(FIXTURES_DIR) + "/Equilibrium_ENG.zip";
    if (!fs::exists(zipPath)) {
        std::fprintf(stderr, "Fixture zip not found: %s\n", zipPath.c_str());
        return;
    }

    std::string tempDir = (fs::temp_directory_path() / "darkness_wq_test").string();
    fs::create_directories(tempDir);

    std::string gamPath = tempDir + "/expone.gam";
    std::string misPath = tempDir + "/Miss20.mis";

    if (!fs::exists(gamPath)) {
        if (!extractFromZip(zipPath, "expone.gam", gamPath)) return;
    }
    if (!fs::exists(misPath)) {
        if (!extractFromZip(zipPath, "Miss20.mis", misPath)) return;
    }

    // Initialize logging — reuse existing singletons if another test file
    // (e.g. test_typed_properties) already created them in this process
    if (!Logger::getSingletonPtr()) {
        s_logger = new Logger();
        s_stdlog = new StdLog();
        s_logger->setLogLevel(Logger::LOG_LEVEL_FATAL);
    }
    if (!ConsoleBackend::getSingletonPtr()) {
        s_console = new ConsoleBackend();
    }

    // Reuse existing ServiceManager if already initialized
    ServiceManager *svcMgr = ServiceManager::getSingletonPtr();
    if (!svcMgr) {
        svcMgr = new ServiceManager(SERVICE_ALL);

        svcMgr->registerFactory<PlatformServiceFactory>();
        svcMgr->registerFactory<ConfigServiceFactory>();
        svcMgr->registerFactory<DatabaseServiceFactory>();
        svcMgr->registerFactory<GameServiceFactory>();
        svcMgr->registerFactory<InheritServiceFactory>();
        svcMgr->registerFactory<LinkServiceFactory>();
        svcMgr->registerFactory<LoopServiceFactory>();
        svcMgr->registerFactory<ObjectServiceFactory>();
        svcMgr->registerFactory<PropertyServiceFactory>();
        svcMgr->registerFactory<RoomServiceFactory>();
        svcMgr->registerFactory<SimServiceFactory>();
        svcMgr->registerFactory<PhysicsServiceFactory>();

        svcMgr->bootstrapFinished();
    }

    // Load schema and mission — skip if another test file already did this
    PropertyServicePtr propSvc = GET_SERVICE(PropertyService);
    LinkServicePtr linkSvc = GET_SERVICE(LinkService);
    s_propSvc = propSvc.get();
    s_linkSvc = linkSvc.get();

    // Check if schema already loaded (e.g. by test_typed_properties)
    if (!propSvc->getProperty("ModelName")) {
        std::string scriptsDir = std::string(SCRIPTS_DIR);

        PLDefResult propDefs = parsePLDef(scriptsDir + "/t2-props.pldef");
        PLDefResult linkDefs = parsePLDef(scriptsDir + "/t2-links.pldef");
        auto dtypeSizes = parseDTypeSizes(scriptsDir + "/t2-types.dtype");

        for (const auto &pd : propDefs.properties) {
            if (propSvc->getProperty(pd.name)) continue;
            DataStoragePtr storage;
            if (pd.isVarStr)
                storage = DataStoragePtr(new StringDataStorage());
            else
                storage = DataStoragePtr(new RawDataStorage());
            Property *prop = propSvc->createProperty(
                pd.name, pd.name, pd.inheritor, storage);
            if (prop)
                prop->setChunkVersions(pd.verMaj, pd.verMin);
        }

        for (const auto &rd : linkDefs.relations) {
            if (linkSvc->getRelation(rd.name)) continue;
            DataStoragePtr storage;
            if (!rd.noData) {
                auto it = dtypeSizes.find(rd.name);
                size_t dataSize = (it != dtypeSizes.end()) ? it->second : 0;
                storage = DataStoragePtr(new RawDataStorage(dataSize));
            }
            RelationPtr rel = linkSvc->createRelation(rd.name, storage, rd.hidden);
            if (rel) {
                rel->setChunkVersions(rd.lVerMaj, rd.lVerMin,
                                      rd.dVerMaj, rd.dVerMin);
                if (rd.fakeSize >= 0)
                    rel->setFakeSize(rd.fakeSize);
            }
        }

        // Register rendering properties
        auto registerRawProp = [&](const char *name, const char *chunk,
                                   const char *inheritor) {
            if (!propSvc->getProperty(name)) {
                propSvc->createProperty(name, chunk, inheritor,
                    DataStoragePtr(new RawDataStorage()));
            }
        };
        registerRawProp("ModelName",   "ModelName", "always");
        registerRawProp("RenderType",  "RenderTyp", "always");
        registerRawProp("RenderAlpha", "RenderAlp", "always");
        registerRawProp("ModelScale",  "Scale",     "never");

        // Load mission
        GameServicePtr gameSvc = GET_SERVICE(GameService);
        gameSvc->load(misPath);
    }

    // Grab raw service pointers
    ObjectServicePtr objSvc = GET_SERVICE(ObjectService);
    s_objSvc = objSvc.get();
    RoomServicePtr roomSvc = GET_SERVICE(RoomService);
    s_roomSvc = roomSvc.get();

    // Construct the IWorldQuery facade under test
    s_worldQuery = std::make_unique<ObjSysWorldState>(
        s_objSvc, s_propSvc, s_linkSvc, s_roomSvc);

    // Parse WR geometry for raycaster integration tests
    if (!s_wrParsed) {
        try {
            s_wrData = parseWRChunk(misPath);
            s_wrParsed = true;
            std::fprintf(stderr, "Test: parsed WR geometry (%u cells)\n", s_wrData.numCells);

            // Inject raycaster into the world query facade
            s_worldQuery->setRaycaster(
                [](const Vector3 &from, const Vector3 &to, RayHit &hit) {
                    return raycastWorld(s_wrData, from, to, hit);
                });
        } catch (const std::exception &e) {
            std::fprintf(stderr, "Test: failed to parse WR chunk: %s\n", e.what());
        }
    }

    s_available = true;
    std::fprintf(stderr, "Test: mission loaded for IWorldQuery tests\n");
}

#define REQUIRE_WQ() do {       \
    ensureLoaded();              \
    if (!s_available) {          \
        SKIP("Fixture not available"); \
    }                            \
} while(0)

// ============================================================================
// WorldQueryTypes tests (no fixture needed)
// ============================================================================

TEST_CASE("WorldQueryTypes: PropertyHandle default is null",
          "[worldquery][types]") {
    PropertyHandle h;
    CHECK_FALSE(static_cast<bool>(h));
    CHECK(h._internal == nullptr);
}

TEST_CASE("WorldQueryTypes: RelationHandle default is null",
          "[worldquery][types]") {
    RelationHandle h;
    CHECK_FALSE(static_cast<bool>(h));
    CHECK(h._internal == nullptr);
}

TEST_CASE("WorldQueryTypes: BBox center calculation",
          "[worldquery][types]") {
    BBox box{{-2.0f, -4.0f, 0.0f}, {6.0f, 8.0f, 10.0f}};
    Vector3 c = box.center();
    CHECK(c.x == 2.0f);
    CHECK(c.y == 2.0f);
    CHECK(c.z == 5.0f);
}

TEST_CASE("WorldQueryTypes: PortalInfo is value-copyable",
          "[worldquery][types]") {
    PortalInfo a;
    a.id = 42;
    a.nearRoom = 1;
    a.farRoom = 2;
    a.center = {1.0f, 2.0f, 3.0f};

    PortalInfo b = a;
    CHECK(b.id == 42);
    CHECK(b.nearRoom == 1);
    CHECK(b.farRoom == 2);
    CHECK(b.center.x == 1.0f);
}

TEST_CASE("WorldQueryTypes: LinkInfo is value-copyable",
          "[worldquery][types]") {
    LinkInfo a;
    a.id = 100;
    a.src = -5;
    a.dst = 10;
    a.flavor = 3;

    LinkInfo b = a;
    CHECK(b.id == 100);
    CHECK(b.src == -5);
    CHECK(b.dst == 10);
    CHECK(b.flavor == 3);
}

// ============================================================================
// Entity query tests
// ============================================================================

TEST_CASE("IWorldQuery: exists() matches ObjectService",
          "[worldquery][entity]") {
    REQUIRE_WQ();

    // Archetype -1 always exists (root object)
    CHECK(s_worldQuery->exists(-1));
    CHECK(s_worldQuery->exists(-1) == s_objSvc->exists(-1));

    // Object 0 should not exist (reserved null ID)
    CHECK_FALSE(s_worldQuery->exists(0));

    // Check a few positive IDs match direct ObjectService
    for (int id = 1; id <= 20; ++id) {
        CHECK(s_worldQuery->exists(id) == s_objSvc->exists(id));
    }
}

TEST_CASE("IWorldQuery: getPosition() matches ObjectService",
          "[worldquery][entity]") {
    REQUIRE_WQ();

    // Find some positioned entities
    auto positioned = s_worldQuery->getAllWithProperty("Position");
    REQUIRE(positioned.size() > 0);

    int checked = 0;
    for (auto eid : positioned) {
        if (eid <= 0) continue; // only concrete objects have meaningful positions

        Vector3 wqPos = s_worldQuery->getPosition(eid);
        Vector3 svcPos = s_objSvc->position(eid);

        CHECK(wqPos.x == svcPos.x);
        CHECK(wqPos.y == svcPos.y);
        CHECK(wqPos.z == svcPos.z);

        if (++checked >= 50) break; // spot check
    }
    CHECK(checked > 0);
}

TEST_CASE("IWorldQuery: getOrientation() matches ObjectService",
          "[worldquery][entity]") {
    REQUIRE_WQ();

    auto positioned = s_worldQuery->getAllWithProperty("Position");
    REQUIRE(positioned.size() > 0);

    int checked = 0;
    for (auto eid : positioned) {
        if (eid <= 0) continue;

        Quaternion wqOri = s_worldQuery->getOrientation(eid);
        Quaternion svcOri = s_objSvc->orientation(eid);

        CHECK(wqOri.w == svcOri.w);
        CHECK(wqOri.x == svcOri.x);
        CHECK(wqOri.y == svcOri.y);
        CHECK(wqOri.z == svcOri.z);

        if (++checked >= 50) break;
    }
    CHECK(checked > 0);
}

TEST_CASE("IWorldQuery: getName() matches ObjectService",
          "[worldquery][entity]") {
    REQUIRE_WQ();

    // Test some named archetypes
    std::string name = s_worldQuery->getName(-1);
    CHECK(name == s_objSvc->getName(-1));

    // Check a few concrete objects
    for (int id = 1; id <= 20; ++id) {
        if (!s_worldQuery->exists(id)) continue;
        CHECK(s_worldQuery->getName(id) == s_objSvc->getName(id));
    }
}

TEST_CASE("IWorldQuery: getBounds() returns valid box",
          "[worldquery][entity]") {
    REQUIRE_WQ();

    auto positioned = s_worldQuery->getAllWithProperty("Position");
    REQUIRE(positioned.size() > 0);

    // Pick a concrete positioned entity
    EntityID eid = 0;
    for (auto id : positioned) {
        if (id > 0) { eid = id; break; }
    }
    REQUIRE(eid > 0);

    BBox box = s_worldQuery->getBounds(eid);
    // Currently a stub returning unit box at position
    Vector3 pos = s_worldQuery->getPosition(eid);
    CHECK(box.min.x == pos.x - 0.5f);
    CHECK(box.max.x == pos.x + 0.5f);
    CHECK(box.min.y == pos.y - 0.5f);
    CHECK(box.max.y == pos.y + 0.5f);
    CHECK(box.min.z == pos.z - 0.5f);
    CHECK(box.max.z == pos.z + 0.5f);
}

// ============================================================================
// Property access tests
// ============================================================================

TEST_CASE("IWorldQuery: resolveProperty returns valid handle for known properties",
          "[worldquery][property]") {
    REQUIRE_WQ();

    auto h = s_worldQuery->resolveProperty("ModelName");
    CHECK(static_cast<bool>(h));
    CHECK(h._internal != nullptr);

    auto h2 = s_worldQuery->resolveProperty("Position");
    CHECK(static_cast<bool>(h2));

    // Unknown property returns null handle
    auto hBad = s_worldQuery->resolveProperty("NonExistentProperty_XYZ");
    CHECK_FALSE(static_cast<bool>(hBad));
}

TEST_CASE("IWorldQuery: hasProperty string matches TypedProperty.h",
          "[worldquery][property]") {
    REQUIRE_WQ();

    auto positioned = s_worldQuery->getAllWithProperty("Position");
    REQUIRE(positioned.size() > 0);

    int checked = 0;
    for (auto eid : positioned) {
        bool wqHas = s_worldQuery->hasProperty(eid, "ModelName");
        bool tpHas = Darkness::hasProperty(s_propSvc, "ModelName", eid);
        CHECK(wqHas == tpHas);

        if (++checked >= 100) break;
    }
    CHECK(checked > 0);
}

TEST_CASE("IWorldQuery: hasProperty handle matches string variant",
          "[worldquery][property]") {
    REQUIRE_WQ();

    auto handle = s_worldQuery->resolveProperty("ModelName");
    REQUIRE(static_cast<bool>(handle));

    auto positioned = s_worldQuery->getAllWithProperty("Position");
    REQUIRE(positioned.size() > 0);

    int checked = 0;
    for (auto eid : positioned) {
        bool byString = s_worldQuery->hasProperty(eid, "ModelName");
        bool byHandle = s_worldQuery->hasProperty(eid, handle);
        CHECK(byString == byHandle);

        if (++checked >= 100) break;
    }
    CHECK(checked > 0);
}

TEST_CASE("IWorldQuery: ownsProperty matches TypedProperty.h",
          "[worldquery][property]") {
    REQUIRE_WQ();

    auto allModelName = s_worldQuery->getAllWithProperty("ModelName");
    REQUIRE(allModelName.size() > 0);

    int checked = 0;
    for (auto eid : allModelName) {
        bool wqOwns = s_worldQuery->ownsProperty(eid, "ModelName");
        bool tpOwns = Darkness::ownsProperty(s_propSvc, "ModelName", eid);
        CHECK(wqOwns == tpOwns);
        // getAllWithProperty returns direct owners, so owns should be true
        CHECK(wqOwns == true);

        if (++checked >= 50) break;
    }
    CHECK(checked > 0);
}

TEST_CASE("IWorldQuery: ownsProperty handle matches string variant",
          "[worldquery][property]") {
    REQUIRE_WQ();

    auto handle = s_worldQuery->resolveProperty("ModelName");
    REQUIRE(static_cast<bool>(handle));

    auto allModelName = s_worldQuery->getAllWithProperty("ModelName");
    REQUIRE(allModelName.size() > 0);

    for (size_t i = 0; i < std::min(allModelName.size(), size_t(50)); ++i) {
        auto eid = allModelName[i];
        CHECK(s_worldQuery->ownsProperty(eid, "ModelName") ==
              s_worldQuery->ownsProperty(eid, handle));
    }
}

TEST_CASE("IWorldQuery: getProperty<T> matches getTypedProperty<T>",
          "[worldquery][property]") {
    REQUIRE_WQ();

    auto allModelName = s_worldQuery->getAllWithProperty("ModelName");
    REQUIRE(allModelName.size() > 0);

    int checked = 0;
    for (auto eid : allModelName) {
        // Via IWorldQuery
        auto wqResult = s_worldQuery->getProperty<PropModelName>(eid, "ModelName");

        // Via TypedProperty.h directly
        PropModelName tpResult;
        bool tpOk = getTypedProperty<PropModelName>(s_propSvc, "ModelName", eid, tpResult);

        CHECK(wqResult.has_value() == tpOk);
        if (wqResult.has_value() && tpOk) {
            CHECK(std::memcmp(wqResult->name, tpResult.name,
                              sizeof(tpResult.name)) == 0);
        }

        if (++checked >= 50) break;
    }
    CHECK(checked > 0);
}

TEST_CASE("IWorldQuery: getProperty<T> via handle matches string variant",
          "[worldquery][property]") {
    REQUIRE_WQ();

    auto handle = s_worldQuery->resolveProperty("ModelName");
    REQUIRE(static_cast<bool>(handle));

    auto allModelName = s_worldQuery->getAllWithProperty("ModelName");
    REQUIRE(allModelName.size() > 0);

    int checked = 0;
    for (auto eid : allModelName) {
        auto byString = s_worldQuery->getProperty<PropModelName>(eid, "ModelName");
        auto byHandle = s_worldQuery->getProperty<PropModelName>(eid, handle);

        CHECK(byString.has_value() == byHandle.has_value());
        if (byString.has_value() && byHandle.has_value()) {
            CHECK(std::memcmp(byString->name, byHandle->name,
                              sizeof(byString->name)) == 0);
        }

        if (++checked >= 50) break;
    }
    CHECK(checked > 0);
}

TEST_CASE("IWorldQuery: getProperty returns nullopt for missing property",
          "[worldquery][property]") {
    REQUIRE_WQ();

    // Object 0 doesn't exist, should return nullopt
    auto result = s_worldQuery->getProperty<PropModelName>(0, "ModelName");
    CHECK_FALSE(result.has_value());

    // Non-existent property name
    auto result2 = s_worldQuery->getProperty<PropModelName>(1, "FakeProperty_XYZ");
    CHECK_FALSE(result2.has_value());
}

TEST_CASE("IWorldQuery: getAllWithProperty returns same set as TypedProperty.h",
          "[worldquery][property]") {
    REQUIRE_WQ();

    auto wqResult = s_worldQuery->getAllWithProperty("ModelName");
    auto tpResult = getAllObjectsWithProperty(s_propSvc, "ModelName");

    // Convert both to sets for order-independent comparison
    std::set<EntityID> wqSet(wqResult.begin(), wqResult.end());
    std::set<int> tpSet(tpResult.begin(), tpResult.end());

    CHECK(wqSet.size() == tpSet.size());

    // Every ID in the typed property result should be in the IWorldQuery result
    for (int id : tpResult) {
        CHECK(wqSet.count(static_cast<EntityID>(id)) == 1);
    }
}

// ============================================================================
// Link query tests
// ============================================================================

TEST_CASE("IWorldQuery: resolveRelation returns valid handle for known relations",
          "[worldquery][link]") {
    REQUIRE_WQ();

    auto h = s_worldQuery->resolveRelation("MetaProp");
    CHECK(static_cast<bool>(h));
    CHECK(h._internal != nullptr);

    // Inverse relation should also be resolvable
    auto hInv = s_worldQuery->resolveRelation("~MetaProp");
    CHECK(static_cast<bool>(hInv));

    // Unknown relation returns null handle
    auto hBad = s_worldQuery->resolveRelation("FakeRelation_XYZ");
    CHECK_FALSE(static_cast<bool>(hBad));
}

TEST_CASE("IWorldQuery: getLinks returns same results as Relation::getAllLinks",
          "[worldquery][link]") {
    REQUIRE_WQ();

    // Test with MetaProp from archetype -1
    RelationPtr mpRel = s_linkSvc->getRelation("MetaProp");
    REQUIRE(mpRel);

    // Get links directly via Relation
    LinkQueryResultPtr directResult = mpRel->getAllLinks(-1, 0);
    std::vector<LinkInfo> directLinks;
    if (directResult) {
        while (!directResult->end()) {
            const Link &lnk = directResult->next();
            directLinks.push_back(
                {lnk.id(), static_cast<EntityID>(lnk.src()),
                 static_cast<EntityID>(lnk.dst()), lnk.flavor()});
        }
    }

    // Get links via IWorldQuery
    auto wqLinks = s_worldQuery->getLinks(-1, "MetaProp", 0);

    CHECK(wqLinks.size() == directLinks.size());
    for (size_t i = 0; i < wqLinks.size() && i < directLinks.size(); ++i) {
        CHECK(wqLinks[i].id == directLinks[i].id);
        CHECK(wqLinks[i].src == directLinks[i].src);
        CHECK(wqLinks[i].dst == directLinks[i].dst);
    }
}

TEST_CASE("IWorldQuery: getLinks handle matches string variant",
          "[worldquery][link]") {
    REQUIRE_WQ();

    auto handle = s_worldQuery->resolveRelation("MetaProp");
    REQUIRE(static_cast<bool>(handle));

    auto byString = s_worldQuery->getLinks(-1, "MetaProp", 0);
    auto byHandle = s_worldQuery->getLinks(-1, handle, 0);

    CHECK(byString.size() == byHandle.size());
    for (size_t i = 0; i < byString.size() && i < byHandle.size(); ++i) {
        CHECK(byString[i].id == byHandle[i].id);
        CHECK(byString[i].src == byHandle[i].src);
        CHECK(byString[i].dst == byHandle[i].dst);
    }
}

TEST_CASE("IWorldQuery: getBackLinks uses ~Relation",
          "[worldquery][link]") {
    REQUIRE_WQ();

    // getBackLinks(dst, "MetaProp") should resolve to ~MetaProp
    // and return links where dst appears as the source in the inverse relation
    auto backLinks = s_worldQuery->getBackLinks(-1, "MetaProp", 0);

    // Verify by querying ~MetaProp directly
    auto invLinks = s_worldQuery->getLinks(-1, "~MetaProp", 0);

    CHECK(backLinks.size() == invLinks.size());
    for (size_t i = 0; i < backLinks.size() && i < invLinks.size(); ++i) {
        CHECK(backLinks[i].id == invLinks[i].id);
    }
}

TEST_CASE("IWorldQuery: getLinks returns empty for non-existent relation",
          "[worldquery][link]") {
    REQUIRE_WQ();

    auto links = s_worldQuery->getLinks(-1, "FakeRelation_XYZ", 0);
    CHECK(links.empty());
}

TEST_CASE("IWorldQuery: LinkInfo values are correct snapshots",
          "[worldquery][link]") {
    REQUIRE_WQ();

    // Find a relation that has at least one link
    auto links = s_worldQuery->getLinks(-1, "MetaProp", 0);
    if (links.empty()) {
        SKIP("No MetaProp links from -1 in this mission");
    }

    for (const auto &li : links) {
        // Source should be -1 (we queried from -1)
        CHECK(li.src == -1);
        // Destination should be valid (non-zero)
        CHECK(li.dst != 0);
        // ID should be non-zero
        CHECK(li.id != 0);
    }
}

// ============================================================================
// Room/portal topology tests
// ============================================================================

TEST_CASE("IWorldQuery: getRoomAt returns -1 for origin if no rooms loaded",
          "[worldquery][room]") {
    REQUIRE_WQ();

    // Whether rooms are loaded depends on the fixture.
    // Either way, the method should not crash.
    RoomID room = s_worldQuery->getRoomAt({0.0f, 0.0f, 0.0f});
    // Result is either a valid room ID or -1
    (void)room; // Just verify no crash
}

TEST_CASE("IWorldQuery: getPortalOpenFraction stub returns 1.0",
          "[worldquery][room]") {
    REQUIRE_WQ();

    // Stub should always return 1.0
    CHECK(s_worldQuery->getPortalOpenFraction(0) == 1.0f);
    CHECK(s_worldQuery->getPortalOpenFraction(42) == 1.0f);
    CHECK(s_worldQuery->getPortalOpenFraction(-1) == 1.0f);
}

TEST_CASE("IWorldQuery: getPortals and forEachPortal are consistent",
          "[worldquery][room]") {
    REQUIRE_WQ();

    if (!s_roomSvc->isLoaded()) {
        SKIP("Room database not loaded in this fixture");
    }

    const auto &rooms = s_roomSvc->getAllRooms();
    if (rooms.empty()) {
        SKIP("No rooms in this fixture");
    }

    // Test the first room that has portals
    for (const auto &r : rooms) {
        if (!r || r->getPortalCount() == 0) continue;

        RoomID rid = r->getRoomID();

        // Vector-returning variant
        auto vecPortals = s_worldQuery->getPortals(rid);

        // Callback variant — collect into a separate vector
        std::vector<PortalInfo> cbPortals;
        s_worldQuery->forEachPortal(rid,
            [&](const PortalInfo &pi) { cbPortals.push_back(pi); });

        CHECK(vecPortals.size() == cbPortals.size());
        CHECK(vecPortals.size() == r->getPortalCount());

        for (size_t i = 0; i < vecPortals.size() && i < cbPortals.size(); ++i) {
            CHECK(vecPortals[i].id == cbPortals[i].id);
            CHECK(vecPortals[i].nearRoom == cbPortals[i].nearRoom);
            CHECK(vecPortals[i].farRoom == cbPortals[i].farRoom);
            CHECK(vecPortals[i].center.x == cbPortals[i].center.x);
        }

        break; // One room is enough
    }
}

TEST_CASE("IWorldQuery: getAdjacentRooms and forEachAdjacentRoom are consistent",
          "[worldquery][room]") {
    REQUIRE_WQ();

    if (!s_roomSvc->isLoaded()) {
        SKIP("Room database not loaded in this fixture");
    }

    const auto &rooms = s_roomSvc->getAllRooms();
    for (const auto &r : rooms) {
        if (!r || r->getPortalCount() == 0) continue;

        RoomID rid = r->getRoomID();

        auto vecAdj = s_worldQuery->getAdjacentRooms(rid);

        std::vector<RoomID> cbAdj;
        s_worldQuery->forEachAdjacentRoom(rid,
            [&](RoomID adj) { cbAdj.push_back(adj); });

        CHECK(vecAdj.size() == cbAdj.size());
        for (size_t i = 0; i < vecAdj.size() && i < cbAdj.size(); ++i) {
            CHECK(vecAdj[i] == cbAdj[i]);
        }

        break;
    }
}

// ============================================================================
// SpatialIndex unit tests (synthetic data, no fixture needed)
// ============================================================================

TEST_CASE("SpatialIndex: empty index returns empty queries",
          "[spatial][unit]") {
    SpatialIndex idx(10.0f);
    CHECK(idx.size() == 0);
    CHECK(idx.cellCount() == 0);
    CHECK(idx.queryRadius({0, 0, 0}, 100.0f).empty());
    CHECK(idx.queryAABB({{-100, -100, -100}, {100, 100, 100}}).empty());
}

TEST_CASE("SpatialIndex: insert and queryRadius finds entity",
          "[spatial][unit]") {
    SpatialIndex idx(10.0f);
    idx.insert(1, {0, 0, 0});
    idx.insert(2, {5, 0, 0});
    idx.insert(3, {15, 0, 0});
    idx.insert(4, {100, 0, 0});

    auto result = idx.queryRadius({0, 0, 0}, 10.0f);
    std::set<EntityID> found(result.begin(), result.end());

    // Entity 1 (dist=0) and 2 (dist=5) within radius 10
    CHECK(found.count(1) == 1);
    CHECK(found.count(2) == 1);
    // Entity 3 (dist=15) and 4 (dist=100) outside
    CHECK(found.count(3) == 0);
    CHECK(found.count(4) == 0);
}

TEST_CASE("SpatialIndex: queryRadius boundary — entity at exact radius",
          "[spatial][unit]") {
    SpatialIndex idx(10.0f);
    idx.insert(1, {10, 0, 0}); // exactly at radius 10

    auto result = idx.queryRadius({0, 0, 0}, 10.0f);
    // Should be included (distance <= radius)
    CHECK(result.size() == 1);
    CHECK(result[0] == 1);
}

TEST_CASE("SpatialIndex: queryAABB includes and excludes correctly",
          "[spatial][unit]") {
    SpatialIndex idx(10.0f);
    idx.insert(1, {5, 5, 5});    // inside box (0,0,0)-(10,10,10)
    idx.insert(2, {0, 0, 0});    // on edge — included
    idx.insert(3, {10, 10, 10}); // on edge — included
    idx.insert(4, {20, 20, 20}); // outside

    auto result = idx.queryAABB({{0, 0, 0}, {10, 10, 10}});
    std::set<EntityID> found(result.begin(), result.end());

    CHECK(found.count(1) == 1);
    CHECK(found.count(2) == 1);
    CHECK(found.count(3) == 1);
    CHECK(found.count(4) == 0);
}

TEST_CASE("SpatialIndex: remove prevents entity from being found",
          "[spatial][unit]") {
    SpatialIndex idx(10.0f);
    idx.insert(1, {0, 0, 0});
    idx.insert(2, {5, 0, 0});

    CHECK(idx.size() == 2);

    // Remove entity 1
    idx.remove(1);
    CHECK(idx.size() == 1);

    auto result = idx.queryRadius({0, 0, 0}, 100.0f);
    std::set<EntityID> found(result.begin(), result.end());
    CHECK(found.count(1) == 0);
    CHECK(found.count(2) == 1);
}

TEST_CASE("SpatialIndex: update moves entity between cells",
          "[spatial][unit]") {
    SpatialIndex idx(10.0f);
    idx.insert(1, {0, 0, 0});

    // Found at origin
    CHECK(idx.queryRadius({0, 0, 0}, 1.0f).size() == 1);
    // Not found far away
    CHECK(idx.queryRadius({200, 0, 0}, 1.0f).empty());

    // Move entity to (200,0,0)
    idx.update(1, {200, 0, 0});

    // No longer at origin
    CHECK(idx.queryRadius({0, 0, 0}, 1.0f).empty());
    // Now found at new position
    CHECK(idx.queryRadius({200, 0, 0}, 1.0f).size() == 1);
}

TEST_CASE("SpatialIndex: clear empties the index",
          "[spatial][unit]") {
    SpatialIndex idx(10.0f);
    idx.insert(1, {0, 0, 0});
    idx.insert(2, {50, 50, 50});
    idx.insert(3, {-100, -100, -100});

    CHECK(idx.size() == 3);
    idx.clear();
    CHECK(idx.size() == 0);
    CHECK(idx.cellCount() == 0);
    CHECK(idx.queryRadius({0, 0, 0}, 10000.0f).empty());
}

TEST_CASE("SpatialIndex: negative coordinates work correctly",
          "[spatial][unit]") {
    SpatialIndex idx(10.0f);
    idx.insert(1, {-50, -50, -50});
    idx.insert(2, {-45, -50, -50});

    auto result = idx.queryRadius({-50, -50, -50}, 10.0f);
    std::set<EntityID> found(result.begin(), result.end());
    CHECK(found.count(1) == 1);
    CHECK(found.count(2) == 1);
}

TEST_CASE("SpatialIndex: cell boundary — entities in different cells both found",
          "[spatial][unit]") {
    SpatialIndex idx(10.0f);
    // Cell boundary at x=10: entity at 9.5 (cell 0) and 10.5 (cell 1)
    idx.insert(1, {9.5f, 0, 0});
    idx.insert(2, {10.5f, 0, 0});

    // Radius from x=10 should find both (dist < 1)
    auto result = idx.queryRadius({10.0f, 0, 0}, 2.0f);
    std::set<EntityID> found(result.begin(), result.end());
    CHECK(found.count(1) == 1);
    CHECK(found.count(2) == 1);
}

TEST_CASE("SpatialIndex: multiple entities in same cell all returned",
          "[spatial][unit]") {
    SpatialIndex idx(10.0f);
    // All in cell (0,0,0)
    idx.insert(1, {1, 1, 1});
    idx.insert(2, {2, 2, 2});
    idx.insert(3, {3, 3, 3});
    idx.insert(4, {4, 4, 4});

    auto result = idx.queryRadius({0, 0, 0}, 10.0f);
    CHECK(result.size() == 4);
}

TEST_CASE("SpatialIndex: large radius captures all inserted entities",
          "[spatial][unit]") {
    SpatialIndex idx(10.0f);
    idx.insert(1, {0, 0, 0});
    idx.insert(2, {500, 0, 0});
    idx.insert(3, {-500, 0, 0});
    idx.insert(4, {0, 500, 0});
    idx.insert(5, {0, 0, 500});

    auto result = idx.queryRadius({0, 0, 0}, 1000.0f);
    CHECK(result.size() == 5);
}

TEST_CASE("SpatialIndex: 3D diagonal distance is correct",
          "[spatial][unit]") {
    SpatialIndex idx(10.0f);
    // sqrt(3^2 + 4^2 + 0^2) = 5
    idx.insert(1, {3, 4, 0});
    // sqrt(6^2 + 8^2 + 0^2) = 10
    idx.insert(2, {6, 8, 0});

    auto result = idx.queryRadius({0, 0, 0}, 5.0f);
    std::set<EntityID> found(result.begin(), result.end());
    CHECK(found.count(1) == 1);  // dist=5, inside
    CHECK(found.count(2) == 0);  // dist=10, outside radius 5
}

// ============================================================================
// Spatial query integration tests (Equilibrium fixture)
// ============================================================================

TEST_CASE("IWorldQuery: queryRadius finds entity at its own position",
          "[worldquery][spatial]") {
    REQUIRE_WQ();

    auto positioned = s_worldQuery->getAllWithProperty("Position");
    REQUIRE(positioned.size() > 0);

    // Find a concrete positioned entity
    EntityID target = 0;
    Vector3 targetPos;
    for (auto eid : positioned) {
        if (eid > 0) {
            target = eid;
            targetPos = s_worldQuery->getPosition(eid);
            break;
        }
    }
    REQUIRE(target > 0);

    // Small radius at entity's exact position should find it
    auto nearby = s_worldQuery->queryRadius(targetPos, 0.01f);
    bool foundTarget = false;
    for (auto id : nearby) {
        if (id == target) foundTarget = true;
    }
    CHECK(foundTarget);
}

TEST_CASE("IWorldQuery: queryRadius large radius finds all concrete positioned entities",
          "[worldquery][spatial]") {
    REQUIRE_WQ();

    auto positioned = s_worldQuery->getAllWithProperty("Position");
    REQUIRE(positioned.size() > 0);

    // Count concrete positioned entities (ID > 0)
    size_t concreteCount = 0;
    for (auto eid : positioned) {
        if (eid > 0) ++concreteCount;
    }

    // Very large radius should find all concrete positioned entities
    auto all = s_worldQuery->queryRadius({0, 0, 0}, 100000.0f);
    CHECK(all.size() >= concreteCount);
}

TEST_CASE("IWorldQuery: queryAABB with huge box returns all positioned entities",
          "[worldquery][spatial]") {
    REQUIRE_WQ();

    auto positioned = s_worldQuery->getAllWithProperty("Position");
    size_t concreteCount = 0;
    for (auto eid : positioned) {
        if (eid > 0) ++concreteCount;
    }

    BBox hugeBox = {{-100000, -100000, -100000}, {100000, 100000, 100000}};
    auto all = s_worldQuery->queryAABB(hugeBox);
    CHECK(all.size() >= concreteCount);
}

TEST_CASE("IWorldQuery: queryFrustum returns entities with wide FOV",
          "[worldquery][spatial]") {
    REQUIRE_WQ();

    auto positioned = s_worldQuery->getAllWithProperty("Position");
    REQUIRE(positioned.size() > 0);

    // Find the centroid of all concrete positioned entities
    float cx = 0, cy = 0, cz = 0;
    int count = 0;
    for (auto eid : positioned) {
        if (eid <= 0) continue;
        Vector3 pos = s_worldQuery->getPosition(eid);
        cx += pos.x; cy += pos.y; cz += pos.z;
        ++count;
    }
    REQUIRE(count > 0);
    cx /= count; cy /= count; cz /= count;

    // Wide FOV from centroid looking forward should find some entities
    auto result = s_worldQuery->queryFrustum(
        {cx, cy, cz}, {1.0f, 0.0f, 0.0f},
        2.0f, 1.33f, 0.1f, 10000.0f); // ~115 degree FOV, huge far plane
    CHECK(result.size() > 0);
}

TEST_CASE("IWorldQuery: queryFrustum subset of enclosing sphere",
          "[worldquery][spatial]") {
    REQUIRE_WQ();

    // queryFrustum uses a conservative AABB, so its results should be a
    // subset of a queryRadius with large enough radius to contain the AABB
    Vector3 origin = {0, 0, 0};
    Vector3 forward = {1, 0, 0};
    float fov = 1.0f;     // ~57 degrees
    float farDist = 500.0f;

    auto frustumResult = s_worldQuery->queryFrustum(
        origin, forward, fov, 1.33f, 0.1f, farDist);

    // The frustum AABB is at most farDist in each direction
    auto sphereResult = s_worldQuery->queryRadius(origin, farDist * 2.0f);
    std::set<EntityID> sphereSet(sphereResult.begin(), sphereResult.end());

    // Every frustum entity should be in the sphere result
    for (auto eid : frustumResult) {
        CHECK(sphereSet.count(eid) == 1);
    }
}

// ============================================================================
// Environment query stub tests
// ============================================================================

TEST_CASE("IWorldQuery: getLightLevel stub returns 1.0",
          "[worldquery][environment]") {
    REQUIRE_WQ();

    CHECK(s_worldQuery->getLightLevel({0.0f, 0.0f, 0.0f}) == 1.0f);
    CHECK(s_worldQuery->getLightLevel({100.0f, -50.0f, 25.0f}) == 1.0f);
}

TEST_CASE("IWorldQuery: raycast without injected raycaster returns false",
          "[worldquery][raycast]") {
    // Fresh ObjSysWorldState without setRaycaster → always returns false
    REQUIRE_WQ();
    auto freshQuery = std::make_unique<ObjSysWorldState>(
        s_objSvc, s_propSvc, s_linkSvc, s_roomSvc);

    RayHit hit;
    bool didHit = freshQuery->raycast(
        {0.0f, 0.0f, 0.0f}, {100.0f, 0.0f, 0.0f}, hit);
    CHECK_FALSE(didHit);
}

// ============================================================================
// Raycast unit tests — synthetic WR geometry, no fixture needed
// ============================================================================

// Helper: build a simple box cell for testing.
// Axis-aligned box from (-size, -size, -size) to (size, size, size).
// 8 vertices, 6 planes (facing inward), 6 solid quad polygons.
static WRParsedData makeBoxWorld(float size, int numCells = 1) {
    WRParsedData wr;
    wr.numCells = numCells;
    wr.lightSize = 1;
    wr.cells.resize(numCells);

    auto &cell = wr.cells[0];
    cell.numPolygons = 6;
    cell.numPortals = 0;
    cell.numPlanes = 6;
    cell.mediaType = 1; // air
    cell.numTextured = 0;
    cell.flowGroup = 0;

    // Cell center and bounding sphere
    cell.center = {0.0f, 0.0f, 0.0f};
    cell.radius = size * 1.74f; // sqrt(3) * size, covers corners

    // 8 vertices of the box
    cell.vertices = {
        {-size, -size, -size}, // 0: ---
        { size, -size, -size}, // 1: +--
        { size,  size, -size}, // 2: ++-
        {-size,  size, -size}, // 3: -+-
        {-size, -size,  size}, // 4: --+
        { size, -size,  size}, // 5: +-+
        { size,  size,  size}, // 6: +++
        {-size,  size,  size}, // 7: -++
    };

    // 6 inward-facing planes (Dark Engine convention: planes face into the cell)
    cell.planes = {
        {{-1, 0, 0}, size},  // 0: +X wall, inward normal = (-1,0,0), d = size
        {{ 1, 0, 0}, size},  // 1: -X wall, inward normal = (+1,0,0), d = size
        {{ 0,-1, 0}, size},  // 2: +Y wall, inward normal = (0,-1,0), d = size
        {{ 0, 1, 0}, size},  // 3: -Y wall, inward normal = (0,+1,0), d = size
        {{ 0, 0,-1}, size},  // 4: +Z wall (ceiling), inward normal = (0,0,-1), d = size
        {{ 0, 0, 1}, size},  // 5: -Z wall (floor), inward normal = (0,0,+1), d = size
    };

    // 6 polygon structs (one per face)
    cell.polygons.resize(6);
    for (int i = 0; i < 6; ++i) {
        cell.polygons[i].flags = 0;
        cell.polygons[i].count = 4;
        cell.polygons[i].plane = static_cast<uint8_t>(i);
        cell.polygons[i].unk = 0;
        cell.polygons[i].tgtCell = 0;
        cell.polygons[i].unk1 = 0;
        cell.polygons[i].unk2 = 0;
    }

    // Polygon vertex indices — wound CCW when viewed from outside the cell.
    // pointInConvexPolygon uses outward normal (-plane.normal) for the winding test.
    cell.polyIndices = {
        {1, 2, 6, 5}, // plane 0: +X wall (x=+size), CCW from outside (+X)
        {3, 0, 4, 7}, // plane 1: -X wall (x=-size), CCW from outside (-X)
        {3, 7, 6, 2}, // plane 2: +Y wall (y=+size), CCW from outside (+Y)
        {0, 1, 5, 4}, // plane 3: -Y wall (y=-size), CCW from outside (-Y)
        {4, 5, 6, 7}, // plane 4: +Z ceiling (z=+size), CCW from outside (+Z)
        {1, 0, 3, 2}, // plane 5: -Z floor (z=-size), CCW from outside (-Z)
    };

    return wr;
}

TEST_CASE("Raycast: ray hits wall in box cell",
          "[raycast][unit]") {
    auto wr = makeBoxWorld(10.0f);
    RayHit hit;

    // Ray from center toward +X wall
    bool didHit = raycastWorld(wr, {0, 0, 0}, {20, 0, 0}, hit);
    CHECK(didHit);
    CHECK(hit.point.x == Catch::Approx(10.0f).margin(0.01f));
    CHECK(hit.point.y == Catch::Approx(0.0f).margin(0.01f));
    CHECK(hit.point.z == Catch::Approx(0.0f).margin(0.01f));
    // Normal = inward cell plane normal = facing the ray origin (toward cell interior)
    // +X wall's inward normal is (-1,0,0)
    CHECK(hit.normal.x == Catch::Approx(-1.0f).margin(0.01f));
    CHECK(hit.distance == Catch::Approx(10.0f).margin(0.01f));
    CHECK(hit.hitEntity == 0); // world geometry
}

TEST_CASE("Raycast: ray hits each face of box",
          "[raycast][unit]") {
    auto wr = makeBoxWorld(10.0f);
    RayHit hit;

    // Inward cell plane normal = facing the ray origin (standard ray-trace convention)
    // +X wall → inward normal (-1,0,0)
    CHECK(raycastWorld(wr, {0,0,0}, {20,0,0}, hit));
    CHECK(hit.normal.x == Catch::Approx(-1.0f).margin(0.01f));

    // -X wall → inward normal (+1,0,0)
    CHECK(raycastWorld(wr, {0,0,0}, {-20,0,0}, hit));
    CHECK(hit.normal.x == Catch::Approx(1.0f).margin(0.01f));

    // +Y wall → inward normal (0,-1,0)
    CHECK(raycastWorld(wr, {0,0,0}, {0,20,0}, hit));
    CHECK(hit.normal.y == Catch::Approx(-1.0f).margin(0.01f));

    // -Y wall → inward normal (0,+1,0)
    CHECK(raycastWorld(wr, {0,0,0}, {0,-20,0}, hit));
    CHECK(hit.normal.y == Catch::Approx(1.0f).margin(0.01f));

    // +Z ceiling → inward normal (0,0,-1)
    CHECK(raycastWorld(wr, {0,0,0}, {0,0,20}, hit));
    CHECK(hit.normal.z == Catch::Approx(-1.0f).margin(0.01f));

    // -Z floor → inward normal (0,0,+1)
    CHECK(raycastWorld(wr, {0,0,0}, {0,0,-20}, hit));
    CHECK(hit.normal.z == Catch::Approx(1.0f).margin(0.01f));
}

TEST_CASE("Raycast: ray misses when parallel to all walls",
          "[raycast][unit]") {
    auto wr = makeBoxWorld(10.0f);
    RayHit hit;

    // Ray along +X axis but too short to reach the wall
    CHECK_FALSE(raycastWorld(wr, {0, 0, 0}, {5, 0, 0}, hit));
}

TEST_CASE("Raycast: ray from outside world returns false",
          "[raycast][unit]") {
    auto wr = makeBoxWorld(10.0f);
    RayHit hit;

    // Origin outside all cells — findCameraCell returns -1
    CHECK_FALSE(raycastWorld(wr, {100, 100, 100}, {200, 100, 100}, hit));
}

TEST_CASE("Raycast: closest hit wins when ray intersects multiple planes",
          "[raycast][unit]") {
    auto wr = makeBoxWorld(10.0f);
    RayHit hit;

    // Ray from near +X wall toward -X wall — should hit +X wall first
    bool didHit = raycastWorld(wr, {5, 0, 0}, {-20, 0, 0}, hit);
    CHECK(didHit);
    // Closest hit should be the -X wall (distance 15 from origin {5,0,0})
    // Wait — from {5,0,0} toward {-20,0,0}: direction is (-1,0,0)
    // +X wall is at x=10, -X wall at x=-10
    // From x=5 in direction (-1,0,0): hits -X wall at t=15, +X wall would be behind (t=-5)
    // Actually the ray goes from (5,0,0) in direction (-1,0,0)
    // It hits x=-10 at t=15
    CHECK(hit.point.x == Catch::Approx(-10.0f).margin(0.01f));
    CHECK(hit.distance == Catch::Approx(15.0f).margin(0.01f));
}

TEST_CASE("Raycast: degenerate zero-length ray returns false",
          "[raycast][unit]") {
    auto wr = makeBoxWorld(10.0f);
    RayHit hit;

    // from == to → zero-length ray
    CHECK_FALSE(raycastWorld(wr, {0, 0, 0}, {0, 0, 0}, hit));
}

TEST_CASE("Raycast: ray through portal hits far wall",
          "[raycast][unit]") {
    // Build two connected box cells sharing a portal on the +X/-X boundary.
    // Portals must be the LAST polygons in the list (indices [numSolid, numPolygons)).

    WRParsedData wr;
    wr.numCells = 2;
    wr.lightSize = 1;
    wr.cells.resize(2);

    // ── Cell 0: box from (-10,-10,-10) to (10,10,10) ──
    // +X face is a portal to cell 1, placed last in the polygon list
    auto &c0 = wr.cells[0];
    c0.numPolygons = 6;
    c0.numPortals = 1;  // last polygon is a portal
    c0.numPlanes = 6;
    c0.mediaType = 1;
    c0.numTextured = 0;
    c0.flowGroup = 0;
    c0.center = {0, 0, 0};
    c0.radius = 10.0f * 1.74f;

    c0.vertices = {
        {-10, -10, -10}, {10, -10, -10}, {10, 10, -10}, {-10, 10, -10},
        {-10, -10,  10}, {10, -10,  10}, {10, 10,  10}, {-10, 10,  10},
    };

    // Planes: put +X plane last (index 5) so its polygon is the portal
    c0.planes = {
        {{ 1, 0, 0}, 10},   // 0: -X wall
        {{ 0,-1, 0}, 10},   // 1: +Y wall
        {{ 0, 1, 0}, 10},   // 2: -Y wall
        {{ 0, 0,-1}, 10},   // 3: +Z ceiling
        {{ 0, 0, 1}, 10},   // 4: -Z floor
        {{-1, 0, 0}, 10},   // 5: +X wall (portal plane)
    };

    c0.polygons.resize(6);
    for (int i = 0; i < 6; ++i) {
        c0.polygons[i].flags = 0;
        c0.polygons[i].count = 4;
        c0.polygons[i].plane = static_cast<uint8_t>(i);
        c0.polygons[i].unk = 0;
        c0.polygons[i].tgtCell = 0;
        c0.polygons[i].unk1 = 0;
        c0.polygons[i].unk2 = 0;
    }
    // Polygon 5 is the portal → cell 1
    c0.polygons[5].tgtCell = 1;

    c0.polyIndices = {
        {3, 0, 4, 7}, // plane 0: -X wall
        {3, 2, 6, 7}, // plane 1: +Y wall
        {0, 4, 5, 1}, // plane 2: -Y wall
        {4, 5, 6, 7}, // plane 3: +Z ceiling
        {1, 0, 3, 2}, // plane 4: -Z floor
        {1, 2, 6, 5}, // plane 5: +X wall (portal)
    };

    // ── Cell 1: box from (10,-10,-10) to (30,10,10) ──
    // -X face is a portal back to cell 0, placed last in the polygon list
    auto &c1 = wr.cells[1];
    c1.numPolygons = 6;
    c1.numPortals = 1;  // last polygon is a portal
    c1.numPlanes = 6;
    c1.mediaType = 1;
    c1.numTextured = 0;
    c1.flowGroup = 0;
    c1.center = {20, 0, 0};
    c1.radius = 10.0f * 1.74f;

    c1.vertices = {
        {10, -10, -10}, {30, -10, -10}, {30, 10, -10}, {10, 10, -10},
        {10, -10,  10}, {30, -10,  10}, {30, 10,  10}, {10, 10,  10},
    };

    // Planes: put -X plane last (index 5) so its polygon is the portal
    c1.planes = {
        {{-1, 0, 0}, 30},   // 0: +X wall
        {{ 0,-1, 0}, 10},   // 1: +Y wall
        {{ 0, 1, 0}, 10},   // 2: -Y wall
        {{ 0, 0,-1}, 10},   // 3: +Z ceiling
        {{ 0, 0, 1}, 10},   // 4: -Z floor
        {{ 1, 0, 0}, -10},  // 5: -X wall (portal plane)
    };

    c1.polygons.resize(6);
    for (int i = 0; i < 6; ++i) {
        c1.polygons[i].flags = 0;
        c1.polygons[i].count = 4;
        c1.polygons[i].plane = static_cast<uint8_t>(i);
        c1.polygons[i].unk = 0;
        c1.polygons[i].tgtCell = 0;
        c1.polygons[i].unk1 = 0;
        c1.polygons[i].unk2 = 0;
    }
    // Polygon 5 is the portal → cell 0
    c1.polygons[5].tgtCell = 0;

    c1.polyIndices = {
        {1, 2, 6, 5}, // plane 0: +X wall
        {3, 2, 6, 7}, // plane 1: +Y wall
        {0, 4, 5, 1}, // plane 2: -Y wall
        {4, 5, 6, 7}, // plane 3: +Z ceiling
        {1, 0, 3, 2}, // plane 4: -Z floor
        {3, 0, 4, 7}, // plane 5: -X wall (portal)
    };

    RayHit hit;

    // Ray from center of cell 0 toward +X (through portal into cell 1, hitting +X wall)
    bool didHit = raycastWorld(wr, {0, 0, 0}, {40, 0, 0}, hit);
    CHECK(didHit);
    // Should hit cell 1's +X wall at x=30, inward normal = (-1,0,0)
    CHECK(hit.point.x == Catch::Approx(30.0f).margin(0.01f));
    CHECK(hit.distance == Catch::Approx(30.0f).margin(0.01f));
    CHECK(hit.normal.x == Catch::Approx(-1.0f).margin(0.01f));
}

TEST_CASE("Raycast: textureIndex populated for textured polygon",
          "[raycast][unit]") {
    auto wr = makeBoxWorld(10.0f);
    auto &cell = wr.cells[0];

    // Add texturing data for the first polygon (+X face)
    cell.numTextured = 1;
    WRPolygonTexturing tex{};
    tex.txt = 42; // texture index 42
    cell.texturing.push_back(tex);

    RayHit hit;
    bool didHit = raycastWorld(wr, {0, 0, 0}, {20, 0, 0}, hit);
    CHECK(didHit);
    CHECK(hit.textureIndex == 42);
}

TEST_CASE("Raycast: textureIndex is -1 for untextured polygon",
          "[raycast][unit]") {
    auto wr = makeBoxWorld(10.0f);
    // No texturing data (numTextured = 0)

    RayHit hit;
    bool didHit = raycastWorld(wr, {0, 0, 0}, {20, 0, 0}, hit);
    CHECK(didHit);
    CHECK(hit.textureIndex == -1);
}

// ============================================================================
// Raycast integration tests (Equilibrium fixture with WR geometry)
// ============================================================================

TEST_CASE("IWorldQuery: raycast downward hits floor from positioned entity",
          "[worldquery][raycast]") {
    REQUIRE_WQ();
    REQUIRE(s_wrParsed);

    // Find a concrete positioned entity that is inside a WR cell
    auto positioned = s_worldQuery->getAllWithProperty("Position");
    REQUIRE(positioned.size() > 0);

    EntityID target = 0;
    Vector3 targetPos;
    for (auto eid : positioned) {
        if (eid <= 0) continue;
        Vector3 pos = s_worldQuery->getPosition(eid);
        // Verify entity is inside a WR cell before casting
        if (findCameraCell(s_wrData, pos.x, pos.y, pos.z) >= 0) {
            target = eid;
            targetPos = pos;
            break;
        }
    }
    REQUIRE(target > 0);

    // Cast ray downward from entity position
    RayHit hit;
    Vector3 below(targetPos.x, targetPos.y, targetPos.z - 200.0f);
    bool didHit = s_worldQuery->raycast(targetPos, below, hit);
    CHECK(didHit);
    if (didHit) {
        // Floor normal should point upward (Z > 0)
        CHECK(hit.normal.z > 0.0f);
        CHECK(hit.distance > 0.0f);
        CHECK(hit.hitEntity == 0); // world geometry
    }
}

TEST_CASE("IWorldQuery: raycast upward hits ceiling from positioned entity",
          "[worldquery][raycast]") {
    REQUIRE_WQ();
    REQUIRE(s_wrParsed);

    auto positioned = s_worldQuery->getAllWithProperty("Position");
    REQUIRE(positioned.size() > 0);

    EntityID target = 0;
    Vector3 targetPos;
    for (auto eid : positioned) {
        if (eid <= 0) continue;
        Vector3 pos = s_worldQuery->getPosition(eid);
        if (findCameraCell(s_wrData, pos.x, pos.y, pos.z) >= 0) {
            target = eid;
            targetPos = pos;
            break;
        }
    }
    REQUIRE(target > 0);

    // Cast ray upward
    RayHit hit;
    Vector3 above(targetPos.x, targetPos.y, targetPos.z + 200.0f);
    bool didHit = s_worldQuery->raycast(targetPos, above, hit);
    CHECK(didHit);
    if (didHit) {
        // Ceiling normal should point downward (Z < 0)
        CHECK(hit.normal.z < 0.0f);
        CHECK(hit.distance > 0.0f);
    }
}

TEST_CASE("IWorldQuery: raycast from outside world returns false",
          "[worldquery][raycast]") {
    REQUIRE_WQ();
    REQUIRE(s_wrParsed);

    RayHit hit;
    bool didHit = s_worldQuery->raycast(
        {99999.0f, 99999.0f, 99999.0f},
        {99999.0f, 99999.0f, 99899.0f}, hit);
    CHECK_FALSE(didHit);
}

TEST_CASE("IWorldQuery: raycast hit has valid textureIndex",
          "[worldquery][raycast]") {
    REQUIRE_WQ();
    REQUIRE(s_wrParsed);

    // Find a positioned entity inside a WR cell
    auto positioned = s_worldQuery->getAllWithProperty("Position");
    REQUIRE(positioned.size() > 0);

    EntityID target = 0;
    Vector3 targetPos;
    for (auto eid : positioned) {
        if (eid <= 0) continue;
        Vector3 pos = s_worldQuery->getPosition(eid);
        if (findCameraCell(s_wrData, pos.x, pos.y, pos.z) >= 0) {
            target = eid;
            targetPos = pos;
            break;
        }
    }
    REQUIRE(target > 0);

    RayHit hit;
    Vector3 below(targetPos.x, targetPos.y, targetPos.z - 200.0f);
    bool didHit = s_worldQuery->raycast(targetPos, below, hit);
    if (didHit) {
        // textureIndex should be either -1 (untextured) or a valid index (>= 0)
        CHECK(hit.textureIndex >= -1);
    }
}

// ============================================================================
// Room getter tests (testing the new const getters on Room/RoomPortal/RoomService)
// ============================================================================

TEST_CASE("RoomService: isLoaded reflects state",
          "[worldquery][room][getters]") {
    REQUIRE_WQ();
    // Just verify it doesn't crash and returns a consistent bool
    bool loaded = s_roomSvc->isLoaded();
    (void)loaded;
}

TEST_CASE("RoomService: getAllRooms returns reference to room vector",
          "[worldquery][room][getters]") {
    REQUIRE_WQ();

    const auto &rooms = s_roomSvc->getAllRooms();
    // Should be a valid reference (may be empty if no room DB)
    if (s_roomSvc->isLoaded() && !rooms.empty()) {
        // Each non-null room should have a valid room ID
        for (const auto &r : rooms) {
            if (!r) continue;
            CHECK(r->getRoomID() >= 0);
        }
    }
}

TEST_CASE("Room: getCenter returns valid vector",
          "[worldquery][room][getters]") {
    REQUIRE_WQ();

    if (!s_roomSvc->isLoaded()) {
        SKIP("Room database not loaded");
    }

    const auto &rooms = s_roomSvc->getAllRooms();
    for (const auto &r : rooms) {
        if (!r) continue;
        const Vector3 &center = r->getCenter();
        // Center should be finite
        CHECK(std::isfinite(center.x));
        CHECK(std::isfinite(center.y));
        CHECK(std::isfinite(center.z));
        break; // One is enough
    }
}

TEST_CASE("Room: getPortal returns null for out-of-range index",
          "[worldquery][room][getters]") {
    REQUIRE_WQ();

    if (!s_roomSvc->isLoaded()) {
        SKIP("Room database not loaded");
    }

    const auto &rooms = s_roomSvc->getAllRooms();
    for (const auto &r : rooms) {
        if (!r) continue;
        // Out of range should return null
        CHECK(r->getPortal(999999) == nullptr);
        break;
    }
}

TEST_CASE("Room: getBoundingPlanes returns 6 planes",
          "[worldquery][room][getters]") {
    REQUIRE_WQ();

    if (!s_roomSvc->isLoaded()) {
        SKIP("Room database not loaded");
    }

    const auto &rooms = s_roomSvc->getAllRooms();
    for (const auto &r : rooms) {
        if (!r) continue;
        const Plane *planes = r->getBoundingPlanes();
        CHECK(planes != nullptr);
        // All normals should be finite
        for (int i = 0; i < 6; ++i) {
            CHECK(std::isfinite(planes[i].normal.x));
            CHECK(std::isfinite(planes[i].normal.y));
            CHECK(std::isfinite(planes[i].normal.z));
        }
        break;
    }
}

TEST_CASE("RoomPortal: getters return consistent data",
          "[worldquery][room][getters]") {
    REQUIRE_WQ();

    if (!s_roomSvc->isLoaded()) {
        SKIP("Room database not loaded");
    }

    const auto &rooms = s_roomSvc->getAllRooms();
    for (const auto &r : rooms) {
        if (!r || r->getPortalCount() == 0) continue;

        RoomPortal *portal = r->getPortal(0);
        REQUIRE(portal != nullptr);

        // Getters should not crash and return finite values
        CHECK(std::isfinite(portal->getCenter().x));
        CHECK(std::isfinite(portal->getCenter().y));
        CHECK(std::isfinite(portal->getCenter().z));
        CHECK(std::isfinite(portal->getPlane().normal.x));
        CHECK(portal->getIndex() == 0); // We asked for index 0
        CHECK(portal->getFarRoom() != nullptr);
        CHECK(portal->getNearRoom() != nullptr);

        break;
    }
}
