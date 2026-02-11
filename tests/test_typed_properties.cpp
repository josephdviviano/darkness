// Typed property verification tests — validates that packed C++ structs in
// DarkPropertyDefs.h match the actual on-disk byte layouts in Dark Engine
// database files. Uses dual-path verification: raw P$ chunk reads vs
// PropertyService typed access.
//
// Test fixture: Equilibrium fan mission (tests/fixtures/Equilibrium_ENG.zip)
// Extracted at runtime via zziplib to a temp directory.

#include <catch2/catch_test_macros.hpp>

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
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

// Typed property access
#include "property/TypedProperty.h"
#include "property/DarkPropertyDefs.h"

// File I/O for raw chunk reading
#include "File.h"
#include "FileGroup.h"
#include "FileCompat.h"

namespace fs = std::filesystem;

// GET_SERVICE macro uses unqualified ServiceManager
using namespace Darkness;

// ============================================================================
// Zip extraction helper — extracts a single file from a zip to a dest path
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
// Shared test fixture — loads the Equilibrium mission once for all tests
// ============================================================================

// File-static state — initialized once, shared across all TEST_CASEs
static bool s_loaded = false;
static bool s_available = false;  // true if fixture files exist and loaded OK
static std::string s_gamPath;
static std::string s_misPath;
static std::string s_tempDir;

// Logger and service singletons must outlive all tests
static Darkness::Logger *s_logger = nullptr;
static Darkness::StdLog *s_stdlog = nullptr;
static Darkness::ConsoleBackend *s_console = nullptr;

// Cached service pointers
static Darkness::PropertyService *s_propSvc = nullptr;

// Raw chunk reading helper — returns map of objID → raw data bytes
static std::map<int, std::vector<uint8_t>>
readRawChunkRecords(Darkness::FileGroupPtr &db, const std::string &chunkName) {
    std::map<int, std::vector<uint8_t>> result;
    if (!db->hasFile(chunkName.c_str()))
        return result;

    Darkness::FilePtr fp = db->getFile(chunkName.c_str());

    // P$ chunks: no count header, just sequential {objID, dataSize, data[]} records
    while (static_cast<size_t>(fp->tell()) + 8 <= fp->size()) {
        uint32_t objID, dataSize;
        *fp >> objID >> dataSize;

        // Guard against corrupted data that would read past EOF
        if (static_cast<size_t>(fp->tell()) + dataSize > fp->size())
            break;

        std::vector<uint8_t> data(dataSize);
        if (dataSize > 0)
            fp->read(data.data(), dataSize);
        result[static_cast<int>(objID)] = std::move(data);
    }
    return result;
}

// Initialize the service stack and load the Equilibrium mission.
// Called at the start of each TEST_CASE — idempotent via s_loaded flag.
static void ensureMissionLoaded() {
    if (s_loaded) return;
    s_loaded = true;

    // Check that the fixture zip exists
    std::string zipPath = std::string(FIXTURES_DIR) + "/Equilibrium_ENG.zip";
    if (!fs::exists(zipPath)) {
        std::fprintf(stderr, "Fixture zip not found: %s\n", zipPath.c_str());
        return;
    }

    // Create temp directory for extracted files
    s_tempDir = (fs::temp_directory_path() / "darkness_test_fixtures").string();
    fs::create_directories(s_tempDir);

    // Extract .gam and .mis from the zip
    s_gamPath = s_tempDir + "/expone.gam";
    s_misPath = s_tempDir + "/Miss20.mis";

    if (!fs::exists(s_gamPath)) {
        if (!extractFromZip(zipPath, "expone.gam", s_gamPath)) {
            std::fprintf(stderr, "Failed to extract expone.gam from zip\n");
            return;
        }
    }
    if (!fs::exists(s_misPath)) {
        if (!extractFromZip(zipPath, "Miss20.mis", s_misPath)) {
            std::fprintf(stderr, "Failed to extract Miss20.mis from zip\n");
            return;
        }
    }

    // Initialize logging — reuse existing singletons if another test file
    // (e.g. test_world_query) already created them in this process
    if (!Darkness::Logger::getSingletonPtr()) {
        s_logger = new Darkness::Logger();
        s_stdlog = new Darkness::StdLog();
        s_logger->setLogLevel(Darkness::Logger::LOG_LEVEL_FATAL);
    }
    if (!Darkness::ConsoleBackend::getSingletonPtr()) {
        s_console = new Darkness::ConsoleBackend();
    }

    // Reuse existing ServiceManager if already initialized
    Darkness::ServiceManager *svcMgr = Darkness::ServiceManager::getSingletonPtr();
    if (!svcMgr) {
        svcMgr = new Darkness::ServiceManager(SERVICE_ALL);

        svcMgr->registerFactory<Darkness::PlatformServiceFactory>();
        svcMgr->registerFactory<Darkness::ConfigServiceFactory>();
        svcMgr->registerFactory<Darkness::DatabaseServiceFactory>();
        svcMgr->registerFactory<Darkness::GameServiceFactory>();
        svcMgr->registerFactory<Darkness::InheritServiceFactory>();
        svcMgr->registerFactory<Darkness::LinkServiceFactory>();
        svcMgr->registerFactory<Darkness::LoopServiceFactory>();
        svcMgr->registerFactory<Darkness::ObjectServiceFactory>();
        svcMgr->registerFactory<Darkness::PropertyServiceFactory>();
        svcMgr->registerFactory<Darkness::RoomServiceFactory>();
        svcMgr->registerFactory<Darkness::SimServiceFactory>();
        svcMgr->registerFactory<Darkness::PhysicsServiceFactory>();

        svcMgr->bootstrapFinished();
    }

    // Load schema and mission — skip if another test file already did this
    Darkness::PropertyServicePtr propSvc = GET_SERVICE(Darkness::PropertyService);
    s_propSvc = propSvc.get();

    if (!propSvc->getProperty("ModelName")) {
        // Schema not yet loaded — register all property/link types
        Darkness::LinkServicePtr linkSvc = GET_SERVICE(Darkness::LinkService);
        std::string scriptsDir = std::string(SCRIPTS_DIR);

        Darkness::PLDefResult propDefs = Darkness::parsePLDef(scriptsDir + "/t2-props.pldef");
        Darkness::PLDefResult linkDefs = Darkness::parsePLDef(scriptsDir + "/t2-links.pldef");
        auto dtypeSizes = Darkness::parseDTypeSizes(scriptsDir + "/t2-types.dtype");

        int propCount = 0;
        for (const auto &pd : propDefs.properties) {
            if (propSvc->getProperty(pd.name))
                continue;
            Darkness::DataStoragePtr storage;
            if (pd.isVarStr) {
                storage = Darkness::DataStoragePtr(new Darkness::StringDataStorage());
            } else {
                storage = Darkness::DataStoragePtr(new Darkness::RawDataStorage());
            }
            Darkness::Property *prop = propSvc->createProperty(
                pd.name, pd.name, pd.inheritor, storage);
            if (prop) {
                prop->setChunkVersions(pd.verMaj, pd.verMin);
                ++propCount;
            }
        }

        int relCount = 0;
        for (const auto &rd : linkDefs.relations) {
            if (linkSvc->getRelation(rd.name))
                continue;
            Darkness::DataStoragePtr storage;
            if (!rd.noData) {
                auto it = dtypeSizes.find(rd.name);
                size_t dataSize = (it != dtypeSizes.end()) ? it->second : 0;
                storage = Darkness::DataStoragePtr(new Darkness::RawDataStorage(dataSize));
            }
            Darkness::RelationPtr rel = linkSvc->createRelation(rd.name, storage, rd.hidden);
            if (rel) {
                rel->setChunkVersions(rd.lVerMaj, rd.lVerMin,
                                      rd.dVerMaj, rd.dVerMin);
                if (rd.fakeSize >= 0)
                    rel->setFakeSize(rd.fakeSize);
                ++relCount;
            }
        }

        // Register rendering properties (commented out in t2-props.pldef,
        // normally created by RenderService which the test doesn't use)
        auto registerRawProp = [&](const char *name, const char *chunk,
                                   const char *inheritor) {
            if (!propSvc->getProperty(name)) {
                propSvc->createProperty(name, chunk, inheritor,
                    Darkness::DataStoragePtr(new Darkness::RawDataStorage()));
                ++propCount;
            }
        };
        registerRawProp("ModelName",   "ModelName", "always");
        registerRawProp("RenderType",  "RenderTyp", "always");
        registerRawProp("RenderAlpha", "RenderAlp", "always");
        registerRawProp("ModelScale",  "Scale",     "never");

        std::fprintf(stderr, "Test: registered %d properties, %d relations\n",
                     propCount, relCount);

        // Load the mission database (.gam + .mis)
        Darkness::GameServicePtr gameSvc = GET_SERVICE(Darkness::GameService);
        gameSvc->load(s_misPath);
    }

    s_available = true;
    std::fprintf(stderr, "Test: mission loaded successfully\n");
}

// Helper macro: skip test if fixture is not available
#define REQUIRE_FIXTURE() do {       \
    ensureMissionLoaded();           \
    if (!s_available) {              \
        SKIP("Fixture not available: Equilibrium_ENG.zip not found or failed to load"); \
    }                                \
} while(0)

// ============================================================================
// Test cases
// ============================================================================

TEST_CASE("Service stack loads Equilibrium mission", "[properties][setup]") {
    REQUIRE_FIXTURE();

    // Verify property service has properties registered
    // The Thief 2 schema has 326 properties from pldef + 4 manually registered = 330
    // Fan missions use the same schema
    Darkness::PropertyServicePtr propSvc = GET_SERVICE(Darkness::PropertyService);
    REQUIRE(propSvc.get() != nullptr);

    // Check that we can find some well-known properties
    CHECK(propSvc->getProperty("ModelName") != nullptr);
    CHECK(propSvc->getProperty("RenderType") != nullptr);
    CHECK(propSvc->getProperty("RenderAlpha") != nullptr);
    CHECK(propSvc->getProperty("ModelScale") != nullptr);
    CHECK(propSvc->getProperty("HitPoints") != nullptr);
    CHECK(propSvc->getProperty("PhysType") != nullptr);
}

TEST_CASE("Struct sizes match on-disk data sizes", "[properties][sizes]") {
    REQUIRE_FIXTURE();

    // Open .gam and .mis directly to read raw P$ chunk records
    // and verify that all dataSize values match sizeof(PropFoo)
    Darkness::FilePtr gamFp(new Darkness::StdFile(s_gamPath, Darkness::File::FILE_R));
    Darkness::FileGroupPtr gamDb(new Darkness::DarkFileGroup(gamFp));

    Darkness::FilePtr misFp(new Darkness::StdFile(s_misPath, Darkness::File::FILE_R));
    Darkness::FileGroupPtr misDb(new Darkness::DarkFileGroup(misFp));

    // Check struct sizes against on-disk record sizes for each property type.
    // For each P$ chunk, read all records and verify every dataSize matches sizeof(Prop).
    struct SizeCheck {
        const char *chunkName;
        size_t expectedSize;
        const char *structName;
    };

    SizeCheck checks[] = {
        {"P$ModelName",  sizeof(Darkness::PropModelName),        "PropModelName"},
        {"P$RenderTyp",  sizeof(Darkness::PropRenderType),       "PropRenderType"},
        {"P$RenderAlp",  sizeof(Darkness::PropRenderAlpha),      "PropRenderAlpha"},
        {"P$Scale",      sizeof(Darkness::PropScale),            "PropScale"},
        {"P$HitPoint",   sizeof(Darkness::PropHitPoints),        "PropHitPoints"},
        {"P$PhysType",   sizeof(Darkness::PropPhysType),         "PropPhysType"},
        {"P$Light",      sizeof(Darkness::PropLight),            "PropLight"},
        {"P$LightColor", sizeof(Darkness::PropLightColor),       "PropLightColor"},
        {"P$Scripts",    sizeof(Darkness::PropScripts),          "PropScripts"},
        {"P$SchPlayPa",  sizeof(Darkness::PropSchemaPlayParams), "PropSchemaPlayParams"},
        {"P$SchLoopPa",  sizeof(Darkness::PropSchemaLoopParams), "PropSchemaLoopParams"},
        {"P$SelfIllum",  sizeof(Darkness::PropSelfIllum),        "PropSelfIllum"},
        {"P$MAX_HP",     sizeof(Darkness::PropMaxHP),            "PropMaxHP"},
    };

    for (const auto &check : checks) {
        DYNAMIC_SECTION("sizeof(" << check.structName << ") = " << check.expectedSize
                        << " matches P$ chunk records") {
            // Read from both .gam (archetypes) and .mis (concrete objects)
            auto gamRecords = readRawChunkRecords(gamDb, check.chunkName);
            auto misRecords = readRawChunkRecords(misDb, check.chunkName);

            int total = 0, mismatches = 0;
            auto verifyRecords = [&](const std::map<int, std::vector<uint8_t>> &records) {
                for (const auto &kv : records) {
                    ++total;
                    if (kv.second.size() != check.expectedSize) {
                        ++mismatches;
                        UNSCOPED_INFO("Object " << kv.first << ": dataSize="
                                      << kv.second.size() << " expected="
                                      << check.expectedSize);
                    }
                }
            };

            verifyRecords(gamRecords);
            verifyRecords(misRecords);

            // Need at least some records to be a meaningful test
            if (total > 0) {
                CHECK(mismatches == 0);
            }
            UNSCOPED_INFO(check.structName << ": " << total << " records checked");
        }
    }
}

TEST_CASE("Raw bytes match PropertyService for ModelName", "[properties][modelname]") {
    REQUIRE_FIXTURE();

    // Path A: Read raw P$ModelName chunk records directly from .gam + .mis
    Darkness::FilePtr gamFp(new Darkness::StdFile(s_gamPath, Darkness::File::FILE_R));
    Darkness::FileGroupPtr gamDb(new Darkness::DarkFileGroup(gamFp));
    Darkness::FilePtr misFp(new Darkness::StdFile(s_misPath, Darkness::File::FILE_R));
    Darkness::FileGroupPtr misDb(new Darkness::DarkFileGroup(misFp));

    auto gamRaw = readRawChunkRecords(gamDb, "P$ModelName");
    auto misRaw = readRawChunkRecords(misDb, "P$ModelName");

    // Merge all raw records (mis overrides gam for same objID)
    std::map<int, std::vector<uint8_t>> allRaw = gamRaw;
    for (const auto &kv : misRaw)
        allRaw[kv.first] = kv.second;

    REQUIRE(allRaw.size() > 0);

    // Path B: Verify PropertyService returns matching raw bytes
    int checked = 0, matched = 0;
    for (const auto &kv : allRaw) {
        int objID = kv.first;
        const auto &rawBytes = kv.second;

        // Only check objects that directly own this property
        if (!Darkness::ownsProperty(s_propSvc, "ModelName", objID))
            continue;

        size_t svcSize = 0;
        const uint8_t *svcData = Darkness::getPropertyRawData(
            s_propSvc, "ModelName", objID, svcSize);

        if (svcData && svcSize == rawBytes.size()) {
            if (std::memcmp(svcData, rawBytes.data(), svcSize) == 0) {
                ++matched;
            }
        }
        ++checked;
    }

    // At least some records should match
    CHECK(checked > 0);
    CHECK(matched == checked);

    // Sanity check: interpret a few records as PropModelName
    int sanityChecked = 0;
    for (const auto &kv : allRaw) {
        if (kv.second.size() < sizeof(Darkness::PropModelName))
            continue;

        Darkness::PropModelName mn;
        std::memcpy(&mn, kv.second.data(), sizeof(mn));

        // Name should be printable ASCII (or null)
        for (int i = 0; i < 16; ++i) {
            if (mn.name[i] == '\0') break;
            CHECK((mn.name[i] >= 0x20 && mn.name[i] <= 0x7E));
        }

        if (++sanityChecked >= 10) break;  // spot check a few
    }
}

TEST_CASE("Raw bytes match PropertyService for RenderType", "[properties][rendertype]") {
    REQUIRE_FIXTURE();

    Darkness::FilePtr gamFp(new Darkness::StdFile(s_gamPath, Darkness::File::FILE_R));
    Darkness::FileGroupPtr gamDb(new Darkness::DarkFileGroup(gamFp));
    Darkness::FilePtr misFp(new Darkness::StdFile(s_misPath, Darkness::File::FILE_R));
    Darkness::FileGroupPtr misDb(new Darkness::DarkFileGroup(misFp));

    auto gamRaw = readRawChunkRecords(gamDb, "P$RenderTyp");
    auto misRaw = readRawChunkRecords(misDb, "P$RenderTyp");

    std::map<int, std::vector<uint8_t>> allRaw = gamRaw;
    for (const auto &kv : misRaw)
        allRaw[kv.first] = kv.second;

    int checked = 0, matched = 0;
    for (const auto &kv : allRaw) {
        int objID = kv.first;
        const auto &rawBytes = kv.second;

        if (!Darkness::ownsProperty(s_propSvc, "RenderType", objID))
            continue;

        size_t svcSize = 0;
        const uint8_t *svcData = Darkness::getPropertyRawData(
            s_propSvc, "RenderType", objID, svcSize);

        if (svcData && svcSize == rawBytes.size()) {
            if (std::memcmp(svcData, rawBytes.data(), svcSize) == 0)
                ++matched;
        }
        ++checked;
    }

    CHECK(checked > 0);
    CHECK(matched == checked);

    // Sanity: mode values should be in [0, 3]
    for (const auto &kv : allRaw) {
        if (kv.second.size() < sizeof(Darkness::PropRenderType))
            continue;
        Darkness::PropRenderType rt;
        std::memcpy(&rt, kv.second.data(), sizeof(rt));
        CHECK(rt.mode <= 3);
    }
}

TEST_CASE("Raw bytes match PropertyService for Scale", "[properties][scale]") {
    REQUIRE_FIXTURE();

    Darkness::FilePtr gamFp(new Darkness::StdFile(s_gamPath, Darkness::File::FILE_R));
    Darkness::FileGroupPtr gamDb(new Darkness::DarkFileGroup(gamFp));
    Darkness::FilePtr misFp(new Darkness::StdFile(s_misPath, Darkness::File::FILE_R));
    Darkness::FileGroupPtr misDb(new Darkness::DarkFileGroup(misFp));

    auto gamRaw = readRawChunkRecords(gamDb, "P$Scale");
    auto misRaw = readRawChunkRecords(misDb, "P$Scale");

    std::map<int, std::vector<uint8_t>> allRaw = gamRaw;
    for (const auto &kv : misRaw)
        allRaw[kv.first] = kv.second;

    int checked = 0, matched = 0;
    for (const auto &kv : allRaw) {
        int objID = kv.first;
        const auto &rawBytes = kv.second;

        if (!Darkness::ownsProperty(s_propSvc, "ModelScale", objID))
            continue;

        size_t svcSize = 0;
        const uint8_t *svcData = Darkness::getPropertyRawData(
            s_propSvc, "ModelScale", objID, svcSize);

        if (svcData && svcSize == rawBytes.size()) {
            if (std::memcmp(svcData, rawBytes.data(), svcSize) == 0)
                ++matched;
        }
        ++checked;
    }

    if (checked > 0) {
        CHECK(matched == checked);
    }

    // Sanity: scale values should be finite and non-negative.
    // Some archetype scales can be 0.0 (indicating "use default 1.0" at runtime).
    for (const auto &kv : allRaw) {
        if (kv.second.size() < sizeof(Darkness::PropScale))
            continue;
        Darkness::PropScale sc;
        std::memcpy(&sc, kv.second.data(), sizeof(sc));
        CHECK(std::isfinite(sc.x));
        CHECK(std::isfinite(sc.y));
        CHECK(std::isfinite(sc.z));
        CHECK(sc.x >= 0.0f);
        CHECK(sc.y >= 0.0f);
        CHECK(sc.z >= 0.0f);
    }
}

TEST_CASE("Raw bytes match PropertyService for HitPoints", "[properties][hitpoints]") {
    REQUIRE_FIXTURE();

    Darkness::FilePtr gamFp(new Darkness::StdFile(s_gamPath, Darkness::File::FILE_R));
    Darkness::FileGroupPtr gamDb(new Darkness::DarkFileGroup(gamFp));
    Darkness::FilePtr misFp(new Darkness::StdFile(s_misPath, Darkness::File::FILE_R));
    Darkness::FileGroupPtr misDb(new Darkness::DarkFileGroup(misFp));

    auto gamRaw = readRawChunkRecords(gamDb, "P$HitPoint");
    auto misRaw = readRawChunkRecords(misDb, "P$HitPoint");

    std::map<int, std::vector<uint8_t>> allRaw = gamRaw;
    for (const auto &kv : misRaw)
        allRaw[kv.first] = kv.second;

    int checked = 0, matched = 0;
    for (const auto &kv : allRaw) {
        int objID = kv.first;
        const auto &rawBytes = kv.second;

        if (!Darkness::ownsProperty(s_propSvc, "HitPoints", objID))
            continue;

        size_t svcSize = 0;
        const uint8_t *svcData = Darkness::getPropertyRawData(
            s_propSvc, "HitPoints", objID, svcSize);

        if (svcData && svcSize == rawBytes.size()) {
            if (std::memcmp(svcData, rawBytes.data(), svcSize) == 0)
                ++matched;
        }
        ++checked;
    }

    if (checked > 0) {
        CHECK(matched == checked);
    }

    // Sanity: HP values are typically in [-1, 10000] range
    for (const auto &kv : allRaw) {
        if (kv.second.size() < sizeof(Darkness::PropHitPoints))
            continue;
        Darkness::PropHitPoints hp;
        std::memcpy(&hp, kv.second.data(), sizeof(hp));
        CHECK(hp.hp >= -1);
        CHECK(hp.hp <= 100000);
    }
}

TEST_CASE("Raw bytes match PropertyService for PhysType", "[properties][physics]") {
    REQUIRE_FIXTURE();

    Darkness::FilePtr gamFp(new Darkness::StdFile(s_gamPath, Darkness::File::FILE_R));
    Darkness::FileGroupPtr gamDb(new Darkness::DarkFileGroup(gamFp));
    Darkness::FilePtr misFp(new Darkness::StdFile(s_misPath, Darkness::File::FILE_R));
    Darkness::FileGroupPtr misDb(new Darkness::DarkFileGroup(misFp));

    auto gamRaw = readRawChunkRecords(gamDb, "P$PhysType");
    auto misRaw = readRawChunkRecords(misDb, "P$PhysType");

    std::map<int, std::vector<uint8_t>> allRaw = gamRaw;
    for (const auto &kv : misRaw)
        allRaw[kv.first] = kv.second;

    int checked = 0, matched = 0;
    for (const auto &kv : allRaw) {
        int objID = kv.first;
        const auto &rawBytes = kv.second;

        if (!Darkness::ownsProperty(s_propSvc, "PhysType", objID))
            continue;

        size_t svcSize = 0;
        const uint8_t *svcData = Darkness::getPropertyRawData(
            s_propSvc, "PhysType", objID, svcSize);

        if (svcData && svcSize == rawBytes.size()) {
            if (std::memcmp(svcData, rawBytes.data(), svcSize) == 0)
                ++matched;
        }
        ++checked;
    }

    if (checked > 0) {
        CHECK(matched == checked);
    }

    // Sanity: physmodeltype should be in [0, 3]
    // (0=OBB, 1=Sphere, 2=SphereHat, 3=None)
    for (const auto &kv : allRaw) {
        if (kv.second.size() < sizeof(Darkness::PropPhysType))
            continue;
        Darkness::PropPhysType pt;
        std::memcpy(&pt, kv.second.data(), sizeof(pt));
        CHECK(pt.type <= 3);
    }
}

TEST_CASE("Inheritance resolution works", "[properties][inheritance]") {
    REQUIRE_FIXTURE();

    // Find a concrete object (positive ID) that inherits ModelName from an
    // archetype but does NOT directly own it. This validates the Inheritor
    // system's archetype/MetaProp chain walking.
    auto allOwners = Darkness::getAllObjectsWithProperty(s_propSvc, "ModelName");

    // Separate into archetypes (negative IDs) and concrete (positive IDs)
    std::vector<int> archetypes, concrete;
    for (int id : allOwners) {
        if (id < 0)
            archetypes.push_back(id);
        else
            concrete.push_back(id);
    }

    // We need archetypes that own ModelName for inheritance to work
    REQUIRE(archetypes.size() > 0);

    // Find a concrete object that has ModelName (via inheritance) but
    // doesn't directly own it
    int inheritingObj = 0;
    // Check concrete objects that have Position (likely game objects)
    // and have ModelName via inheritance
    Darkness::FilePtr misFp(new Darkness::StdFile(s_misPath, Darkness::File::FILE_R));
    Darkness::FileGroupPtr misDb(new Darkness::DarkFileGroup(misFp));
    auto posRecords = readRawChunkRecords(misDb, "P$Position");

    for (const auto &kv : posRecords) {
        int objID = kv.first;
        if (objID <= 0) continue;

        // Has ModelName via inheritance?
        if (!Darkness::hasProperty(s_propSvc, "ModelName", objID))
            continue;

        // Does NOT directly own it?
        if (Darkness::ownsProperty(s_propSvc, "ModelName", objID))
            continue;

        inheritingObj = objID;
        break;
    }

    if (inheritingObj == 0) {
        WARN("No inheriting object found — all concrete objects own ModelName directly");
        return;
    }

    INFO("Found inheriting object: " << inheritingObj);

    // Verify inheritance resolution
    CHECK(Darkness::hasProperty(s_propSvc, "ModelName", inheritingObj));
    CHECK_FALSE(Darkness::ownsProperty(s_propSvc, "ModelName", inheritingObj));

    // Should still be able to get the property value via inheritance
    Darkness::PropModelName mn;
    REQUIRE(Darkness::getTypedProperty<Darkness::PropModelName>(
        s_propSvc, "ModelName", inheritingObj, mn));

    // The inherited name should be printable ASCII
    bool foundPrintable = false;
    for (int i = 0; i < 16; ++i) {
        if (mn.name[i] == '\0') break;
        CHECK((mn.name[i] >= 0x20 && mn.name[i] <= 0x7E));
        foundPrintable = true;
    }
    CHECK(foundPrintable);
}
