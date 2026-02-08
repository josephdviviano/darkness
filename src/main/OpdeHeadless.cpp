/******************************************************************************
 *
 *    This file is part of openDarkEngine project
 *    Copyright (C) 2005-2009 openDarkEngine team
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

// Headless entry point for openDarkEngine
// Loads and inspects Dark Engine databases (.mis, .gam, .sav) without a renderer

#include "config.h"
#include "filelog.h"
#include "logger.h"
#include "stdlog.h"
#include "ConsoleBackend.h"
#include "OpdeServiceManager.h"
#include "File.h"
#include "FileGroup.h"
#include "darkdb.h"
#include "database/DatabaseCommon.h"

// Service factories
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

// Schema loading
#include "RawDataStorage.h"
#include "SingleFieldDataStorage.h"
#include "PLDefParser.h"
#include "DTypeSizeParser.h"

#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

using namespace Opde;

// ---------- Level 1: Raw chunk inspection (no service stack) ----------

static const char *fileTypeName(uint32_t ft) {
    if (ft == DBM_FILETYPE_MIS) return "MIS (Mission)";
    if (ft == DBM_FILETYPE_GAM) return "GAM (Gamesys)";
    if (ft == DBM_FILETYPE_SAV) return "SAV (Savegame)";
    if (ft == DBM_FILETYPE_COW) return "COW (Combined)";
    if (ft == DBM_FILETYPE_VBR) return "VBR (Multibrush)";
    return "Unknown";
}

static FileGroupPtr openFileGroup(const std::string &filename) {
    FilePtr fp(new StdFile(filename, File::FILE_R));
    return FileGroupPtr(new DarkFileGroup(fp));
}

static void printInfo(const std::string &filename) {
    FileGroupPtr db = openFileGroup(filename);

    std::cout << "File: " << filename << std::endl;

    // Read FILE_TYPE
    if (db->hasFile("FILE_TYPE")) {
        FilePtr ft = db->getFile("FILE_TYPE");
        if (ft && ft->size() >= sizeof(uint32_t)) {
            uint32_t fileType = 0;
            ft->readElem(&fileType, sizeof(uint32_t));
            std::cout << "Type: " << fileTypeName(fileType)
                      << " (0x" << std::hex << fileType << std::dec << ")"
                      << std::endl;
        }
    } else {
        std::cout << "Type: (no FILE_TYPE tag)" << std::endl;
    }

    // Read parent DB references
    if (db->hasFile("MIS_FILE")) {
        FilePtr mf = db->getFile("MIS_FILE");
        size_t sz = mf->size();
        std::string parent(sz, '\0');
        mf->read(&parent[0], sz);
        // Trim trailing nulls
        while (!parent.empty() && parent.back() == '\0')
            parent.pop_back();
        std::cout << "MIS_FILE: " << parent << std::endl;
    }

    if (db->hasFile("GAM_FILE")) {
        FilePtr gf = db->getFile("GAM_FILE");
        size_t sz = gf->size();
        std::string parent(sz, '\0');
        gf->read(&parent[0], sz);
        while (!parent.empty() && parent.back() == '\0')
            parent.pop_back();
        std::cout << "GAM_FILE: " << parent << std::endl;
    }

    // Count chunks
    int count = 0;
    for (auto it = db->begin(); it != db->end(); ++it)
        ++count;

    std::cout << "Chunks: " << count << std::endl;
}

static void printChunks(const std::string &filename) {
    FileGroupPtr db = openFileGroup(filename);

    std::cout << "Chunks in " << filename << ":" << std::endl;
    std::cout << std::endl;

    int count = 0;
    for (auto it = db->begin(); it != db->end(); ++it) {
        const auto &name = it->first;
        const auto &chunk = it->second;
        size_t dataSize = chunk.file ? chunk.file->size() : 0;

        printf("  %-16s  v%u.%u  %zu bytes\n",
               name.c_str(),
               chunk.header.version_high,
               chunk.header.version_low,
               dataSize);
        ++count;
    }

    std::cout << std::endl;
    std::cout << count << " chunks total" << std::endl;
}

// ---------- Level 2: Service-based inspection ----------

static void initServices() {
    // ServiceManager is a singleton - construct it with all services enabled
    auto *svcMgr = new ServiceManager(SERVICE_ALL);

    // Register headless service factories (no render, input, draw, etc.)
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

static void loadSchema(const std::string &scriptsDir) {
    PropertyServicePtr propSvc = GET_SERVICE(PropertyService);
    LinkServicePtr linkSvc = GET_SERVICE(LinkService);

    // Parse schema definitions
    PLDefResult propDefs = parsePLDef(scriptsDir + "/t2-props.pldef");
    PLDefResult linkDefs = parsePLDef(scriptsDir + "/t2-links.pldef");
    auto dtypeSizes = parseDTypeSizes(scriptsDir + "/t2-types.dtype");

    int propCount = 0;
    for (const auto &pd : propDefs.properties) {
        // Skip built-in properties
        if (propSvc->getProperty(pd.name))
            continue;

        DataStoragePtr storage;
        if (pd.isVarStr) {
            storage = DataStoragePtr(new StringDataStorage());
        } else {
            storage = DataStoragePtr(new RawDataStorage());
        }

        Property *prop = propSvc->createProperty(
            pd.name, pd.name, pd.inheritor, storage);
        if (prop) {
            prop->setChunkVersions(pd.verMaj, pd.verMin);
            ++propCount;
        }
    }

    int relCount = 0;
    for (const auto &rd : linkDefs.relations) {
        // Skip built-in relations
        if (linkSvc->getRelation(rd.name))
            continue;

        DataStoragePtr storage;
        if (!rd.noData) {
            // Look up the data size from dtype definitions
            auto it = dtypeSizes.find(rd.name);
            size_t dataSize = (it != dtypeSizes.end()) ? it->second : 0;
            storage = DataStoragePtr(new RawDataStorage(dataSize));
        }
        // noData → null storage

        RelationPtr rel = linkSvc->createRelation(rd.name, storage, rd.hidden);
        if (rel) {
            rel->setChunkVersions(rd.lVerMaj, rd.lVerMin,
                                  rd.dVerMaj, rd.dVerMin);
            if (rd.fakeSize >= 0)
                rel->setFakeSize(rd.fakeSize);
            ++relCount;
        }
    }

    std::cout << "Schema: registered " << propCount << " properties, "
              << relCount << " relations" << std::endl;
}

static void loadDatabase(const std::string &filename) {
    GameServicePtr gameSvc = GET_SERVICE(GameService);
    gameSvc->load(filename);
}

static void printObject(ObjectServicePtr &objSvc, int id) {
    std::string name = objSvc->getName(id);
    Vector3 pos = objSvc->position(id);

    if (id < 0) {
        printf("  [%5d] archetype  %-30s\n", id, name.c_str());
    } else {
        printf("  [%5d] concrete   %-30s  pos(%.1f, %.1f, %.1f)\n",
               id, name.c_str(), pos.x, pos.y, pos.z);
    }
}

static void printObjects() {
    ObjectServicePtr objSvc = GET_SERVICE(ObjectService);

    std::cout << "Objects:" << std::endl;
    std::cout << std::endl;

    int count = 0;

    // Scan archetypes (negative IDs) — stop when we hit the BitArray boundary
    for (int id = -1; id >= -8192; --id) {
        try {
            if (!objSvc->exists(id))
                continue;
        } catch (...) {
            break;
        }
        printObject(objSvc, id);
        ++count;
    }

    // Scan concrete objects (non-negative IDs)
    for (int id = 0; id <= 8192; ++id) {
        try {
            if (!objSvc->exists(id))
                continue;
        } catch (...) {
            break;
        }
        printObject(objSvc, id);
        ++count;
    }

    std::cout << std::endl;
    std::cout << count << " objects total" << std::endl;
}

static void printProperties(int objID) {
    PropertyServicePtr propSvc = GET_SERVICE(PropertyService);
    ObjectServicePtr objSvc = GET_SERVICE(ObjectService);

    if (!objSvc->exists(objID)) {
        std::cerr << "Object " << objID << " does not exist" << std::endl;
        return;
    }

    std::string objName = objSvc->getName(objID);
    std::cout << "Properties for object " << objID
              << " (" << objName << "):" << std::endl;
    std::cout << std::endl;

    StringIteratorPtr nameIt = propSvc->getAllPropertyNames();
    int count = 0;

    while (!nameIt->end()) {
        const std::string &propName = nameIt->next();

        if (!propSvc->has(objID, propName))
            continue;

        bool owned = propSvc->owns(objID, propName);

        std::cout << "  " << propName
                  << (owned ? "" : " (inherited)") << std::endl;

        // Dump field values
        try {
            const DataFields &fields = propSvc->getFieldDesc(propName);

            for (const auto &field : fields) {
                Variant val;
                if (propSvc->get(objID, propName, field.name, val)) {
                    std::cout << "    " << field.name << " = "
                              << val.toString() << std::endl;
                }
            }
        } catch (...) {
            // Some properties may not have field descriptors
        }

        ++count;
    }

    std::cout << std::endl;
    std::cout << count << " properties" << std::endl;
}

static void printAllProperties() {
    PropertyServicePtr propSvc = GET_SERVICE(PropertyService);

    std::cout << "Property types:" << std::endl;
    std::cout << std::endl;

    StringIteratorPtr nameIt = propSvc->getAllPropertyNames();
    int count = 0;

    while (!nameIt->end()) {
        const std::string &propName = nameIt->next();
        std::cout << "  " << propName << std::endl;
        ++count;
    }

    std::cout << std::endl;
    std::cout << count << " property types" << std::endl;
}

static void printLinks(int objID) {
    LinkServicePtr linkSvc = GET_SERVICE(LinkService);
    ObjectServicePtr objSvc = GET_SERVICE(ObjectService);

    if (!objSvc->exists(objID)) {
        std::cerr << "Object " << objID << " does not exist" << std::endl;
        return;
    }

    std::string objName = objSvc->getName(objID);
    std::cout << "Links from object " << objID
              << " (" << objName << "):" << std::endl;
    std::cout << std::endl;

    StringIteratorPtr nameIt = linkSvc->getAllLinkNames();
    int count = 0;

    while (!nameIt->end()) {
        const std::string &relName = nameIt->next();

        int flavor = linkSvc->nameToFlavor(relName);
        if (flavor == 0)
            continue;

        LinkQueryResultPtr links = linkSvc->getAllLinks(flavor, objID, 0);

        while (!links->end()) {
            const Link &lnk = links->next();

            std::string dstName = objSvc->exists(lnk.dst())
                                      ? objSvc->getName(lnk.dst())
                                      : "?";

            printf("  %-20s -> [%d] %s  (id=%u)\n",
                   relName.c_str(), lnk.dst(), dstName.c_str(), lnk.id());
            ++count;
        }
    }

    std::cout << std::endl;
    std::cout << count << " links" << std::endl;
}

static void printAllLinks() {
    LinkServicePtr linkSvc = GET_SERVICE(LinkService);

    std::cout << "Link/Relation types:" << std::endl;
    std::cout << std::endl;

    StringIteratorPtr nameIt = linkSvc->getAllLinkNames();
    int count = 0;

    while (!nameIt->end()) {
        const std::string &relName = nameIt->next();
        std::cout << "  " << relName << std::endl;
        ++count;
    }

    std::cout << std::endl;
    std::cout << count << " relation types" << std::endl;
}

// ---------- Usage ----------

static void printUsage(const char *prog) {
    std::cerr << "openDarkEngine Headless Inspector" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Usage: " << prog << " <database> [command] [args] [--scripts <path>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Commands:" << std::endl;
    std::cerr << "  info              File type, parent DB chain, chunk count (default)" << std::endl;
    std::cerr << "  chunks            List all chunks with name, version, size" << std::endl;
    std::cerr << "  objects           List all objects with ID, name, position" << std::endl;
    std::cerr << "  properties [id]   List property types, or dump properties for an object" << std::endl;
    std::cerr << "  links [id]        List relation types, or dump links for an object" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Options:" << std::endl;
    std::cerr << "  --scripts <path>  Path to schema scripts directory (default: scripts/thief2)" << std::endl;
}

// ---------- Main ----------

int main(int argc, char *argv[]) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    // Parse --scripts option from anywhere in argv
    std::string scriptsDir = "scripts/thief2";
    std::vector<std::string> positionalArgs;

    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--scripts" && i + 1 < argc) {
            scriptsDir = argv[++i];
        } else {
            positionalArgs.push_back(argv[i]);
        }
    }

    if (positionalArgs.empty()) {
        printUsage(argv[0]);
        return 1;
    }

    std::string dbFile = positionalArgs[0];
    std::string command = (positionalArgs.size() >= 2) ? positionalArgs[1] : "info";

    // Set up logging - suppress for Level 1, allow for Level 2
    Logger logger;
    StdLog stdlog;

    bool needsServices = (command == "objects" ||
                          command == "properties" ||
                          command == "links");

    if (needsServices) {
        logger.registerLogListener(&stdlog);
        logger.setLogLevel(Logger::LOG_LEVEL_INFO);
    } else {
        logger.setLogLevel(Logger::LOG_LEVEL_FATAL);
    }

    ConsoleBackend console;

    try {
        if (command == "info") {
            printInfo(dbFile);
        } else if (command == "chunks") {
            printChunks(dbFile);
        } else if (command == "objects") {
            initServices();
            loadSchema(scriptsDir);
            loadDatabase(dbFile);
            printObjects();
        } else if (command == "properties") {
            initServices();
            loadSchema(scriptsDir);
            loadDatabase(dbFile);
            if (positionalArgs.size() >= 3) {
                int objID = std::stoi(positionalArgs[2]);
                printProperties(objID);
            } else {
                printAllProperties();
            }
        } else if (command == "links") {
            initServices();
            loadSchema(scriptsDir);
            loadDatabase(dbFile);
            if (positionalArgs.size() >= 3) {
                int objID = std::stoi(positionalArgs[2]);
                printLinks(objID);
            } else {
                printAllLinks();
            }
        } else {
            std::cerr << "Unknown command: " << command << std::endl;
            printUsage(argv[0]);
            return 1;
        }
    } catch (const Opde::BasicException &e) {
        std::cerr << "Error: " << e.getDetails() << std::endl;
        return 1;
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    if (needsServices) {
        logger.unregisterLogListener(&stdlog);
    }

    return 0;
}
