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

// Headless entry point for Darkness
// Loads and inspects Dark Engine databases (.mis, .gam, .sav) without a renderer

#include "config.h"
#include "filelog.h"
#include "logger.h"
#include "stdlog.h"
#include "ConsoleBackend.h"
#include "DarknessServiceManager.h"
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
#include "room/Room.h"
#include "room/RoomPortal.h"
#include "property/DarkPropertyDefs.h"
#include "DataStorage.h"
#include "sim/SimService.h"
#include "physics/PhysicsService.h"

// Schema loading
#include "RawDataStorage.h"
#include "SingleFieldDataStorage.h"
#include "PLDefParser.h"
#include "DTypeSizeParser.h"

// Sound chunk database parsers (Units D, E, F)
#include "audio/AIHearingData.h"
#include "audio/EnvSoundDatabase.h"
#include "audio/SchemaSamplesChunk.h"
#include "audio/SpeechDatabase.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

using namespace Darkness;

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

// ---------- Room geometry dump ----------
//
// `room-info <roomID> [...more]` prints OBB bounding planes (with their
// implied min/max box extents), portal list, and portal-to-portal
// distance matrix for each requested room. Use it to spot degenerate
// rooms (near-zero volume), suspicious portal placements, or chains of
// overlapping OBBs that allow sound to "skip" through near-zero
// segments in propagateSoundPath.
static void printRoomInfo(int roomID) {
    RoomServicePtr roomSvc = GET_SERVICE(RoomService);
    if (!roomSvc || !roomSvc->isLoaded()) {
        std::cerr << "room-info: RoomService not loaded\n";
        return;
    }
    ::Darkness::Room *r = roomSvc->getRoomByID(roomID);
    if (!r) {
        std::cerr << "room-info: room " << roomID << " not found\n";
        return;
    }

    Vector3 c = r->getCenter();
    std::cout << "Room " << roomID << "  center=(" << c.x << ", " << c.y
              << ", " << c.z << ")  portals=" << r->getPortalCount() << "\n";

    // OBB planes. For an axis-aligned room the 6 planes give us x/y/z
    // min/max directly via their `d` values; we print all 6 raw so
    // non-axis-aligned rooms are still readable.
    const ::Darkness::Plane *planes = r->getBoundingPlanes();
    std::cout << "  Bounding planes (inward-normal convention):\n";
    for (int p = 0; p < 6; ++p) {
        std::printf("    plane[%d] n=(%.3f, %.3f, %.3f) d=%.3f\n",
                    p, planes[p].normal.x, planes[p].normal.y, planes[p].normal.z,
                    planes[p].d);
    }

    // Portal list with far-room IDs, per-portal centers, plane + edges.
    uint32_t pc = r->getPortalCount();
    std::cout << "  Portals:\n";
    for (uint32_t i = 0; i < pc; ++i) {
        ::Darkness::RoomPortal *p = r->getPortal(i);
        if (!p) { std::cout << "    [" << i << "] (null)\n"; continue; }
        ::Darkness::Room *far = p->getFarRoom();
        Vector3 pc_ = p->getCenter();
        std::printf("    [%2u] id=%-4d  far=%-4d  center=(%.3f, %.3f, %.3f)  edges=%u  destPortalID=%d\n",
                    i, p->getPortalID(), far ? far->getRoomID() : -1,
                    pc_.x, pc_.y, pc_.z, p->getEdgeCount(), p->getDestPortalID());
        // Portal plane (normal points into far room per our convention).
        const ::Darkness::Plane &pl = p->getPlane();
        std::printf("         plane n=(%.3f,%.3f,%.3f) d=%.3f\n",
                    pl.normal.x, pl.normal.y, pl.normal.z, pl.d);
        // Edge half-planes (normals point INTO the polygon interior).
        for (uint32_t e = 0; e < p->getEdgeCount(); ++e) {
            const ::Darkness::Plane &ep = p->getEdgePlane(e);
            std::printf("         edge[%u] n=(%.3f,%.3f,%.3f) d=%.3f\n",
                        e, ep.normal.x, ep.normal.y, ep.normal.z, ep.d);
        }
    }

    // Portal-to-portal distance matrix (intra-room shortest paths from
    // one portal to another, precomputed by the level compiler).
    if (pc > 1) {
        std::cout << "  Portal-to-portal distances (intra-room):\n";
        std::cout << "        ";
        for (uint32_t j = 0; j < pc; ++j) std::printf("%-7u", j);
        std::cout << "\n";
        for (uint32_t i = 0; i < pc; ++i) {
            std::printf("    %2u: ", i);
            for (uint32_t j = 0; j < pc; ++j) {
                std::printf("%-7.2f", r->getPortalDist(i, j));
            }
            std::cout << "\n";
        }
    }
}

// ---------- Ambient sound dump ----------
//
// `ambients` lists every object with a P$AmbientHa property: schema name,
// radius, position, room ID, and flags. Use it to identify which ambient
// corresponds to "the church" and check whether its radius is plausibly
// reaching the listener's room.
struct AmbientRecord {
    int32_t objID;
    int32_t radius;
    int32_t volume;
    uint32_t flags;
    char schema[16];
};
static void printAmbients() {
    PropertyServicePtr propSvc = GET_SERVICE(PropertyService);
    ObjectServicePtr   objSvc  = GET_SERVICE(ObjectService);
    RoomServicePtr     roomSvc = GET_SERVICE(RoomService);

    ::Darkness::Property *prop = propSvc->getProperty("AmbientHa");
    if (!prop) { std::cerr << "ambients: no P$AmbientHa property registered\n"; return; }
    DataStorage *storage = prop->getStorage();
    if (!storage) { std::cerr << "ambients: AmbientHa storage null\n"; return; }

    std::cout << "ObjID  Room  Radius  Volume  Flags     Schema           Position\n";
    std::cout << "-----  ----  ------  ------  --------  ---------------  ----------\n";

    IntIteratorPtr it = storage->getAllStoredObjects();
    int total = 0;
    while (!it->end()) {
        int objID = it->next();
        if (objID <= 0) continue;  // archetypes
        size_t sz = 0;
        const uint8_t *bytes = storage->getRawData(objID, sz);
        if (!bytes || sz < 60) continue;

        AmbientRecord a{};
        std::memcpy(&a.radius, bytes, 4);
        std::memcpy(&a.volume, bytes + 4, 4);
        std::memcpy(&a.flags,  bytes + 8, 4);
        std::memcpy(a.schema,  bytes + 12, 16);
        a.objID = objID;

        Vector3 pos = objSvc ? objSvc->position(objID) : Vector3{0,0,0};
        ::Darkness::Room *room = roomSvc ? roomSvc->roomFromPoint(pos) : nullptr;
        int roomID = room ? room->getRoomID() : -1;
        std::string schemaStr(a.schema, ::strnlen(a.schema, 16));

        std::printf("%-5d  %-4d  %-6d  %-6d  %08x  %-15s  (%.1f, %.1f, %.1f)\n",
                    a.objID, roomID, a.radius, a.volume, a.flags,
                    schemaStr.c_str(), pos.x, pos.y, pos.z);
        ++total;
    }
    std::cout << "(" << total << " ambients total)" << std::endl;
}

// ---------- Sound chunk database dumpers ----------
//
// `sound-chunks` dumps every sound-related chunk loaded from the
// mission/gamesys: AMBIENT, ENV_SOUND, Speech_DB, SchSamp, AIHearStat,
// AISNDTWK. Lets the user verify that our parsers see real data.
static bool readChunkBytesHL(const FileGroupPtr &db, const std::string &name,
                              std::vector<uint8_t> &out)
{
    if (!db->hasFile(name)) return false;
    FilePtr f = db->getFile(name);
    if (!f) return false;
    file_size_t sz = f->size();
    if (sz == 0) return false;
    out.resize(static_cast<size_t>(sz));
    f->seek(0);
    f->read(out.data(), sz);
    return true;
}

static void printSoundChunks(const FileGroupPtr &db) {
    using namespace ::Darkness;
    auto hexDump = [](const std::vector<uint8_t> &b, size_t n) {
        size_t k = std::min<size_t>(b.size(), n);
        for (size_t i = 0; i < k; ++i)
            std::printf("%02x ", b[i]);
        if (b.size() > k) std::printf("...");
        std::printf("\n");
    };

    std::cout << "=== Sound chunk databases ===\n";

    // AMBIENT (mission-level)
    std::vector<uint8_t> bytes;
    if (readChunkBytesHL(db, "AMBIENT", bytes)) {
        std::cout << "AMBIENT: " << bytes.size() << " bytes  ";
        if (bytes.size() >= 4) {
            uint32_t v; std::memcpy(&v, bytes.data(), 4);
            std::printf("value=0x%08x (%u)\n", v, v);
        } else {
            std::printf("\n  hex="); hexDump(bytes, 16);
        }
    } else {
        std::cout << "AMBIENT: not present\n";
    }

    // Pre-load Speech_DB so the ENV_SOUND printer can project its raw
    // tag-type / enum-byte indices through the speech domain's name
    // maps — the ENV_SOUND chunk itself carries no name maps.
    SpeechDatabase speechDB;
    bool speechLoaded = false;
    if (readChunkBytesHL(db, "Speech_DB", bytes)) {
        speechLoaded = speechDB.loadFromChunk(bytes.data(), bytes.size());
    }

    // ENV_SOUND — decoded as a cTagDBDatabase tag tree. Each surfaced
    // entry corresponds to a key-path that resolves to one or more
    // schema ObjIDs (with matching weight). The chunk preamble carries
    // a "required tag" bitarray that gates which tag-types must appear
    // in queries.
    if (readChunkBytesHL(db, "ENV_SOUND", bytes)) {
        EnvSoundDatabase env;
        bool ok = env.loadFromChunk(bytes.data(), bytes.size());
        std::printf("ENV_SOUND: %zu bytes  loaded=%s  entries=%zu  "
                    "required-tag-bytes=%zu  tail=%zu  (names=%s)\n",
                    bytes.size(), ok ? "yes" : "no",
                    env.entries().size(),
                    env.localTagRequired().size(),
                    env.tailBytes().size(),
                    speechLoaded ? "resolved via Speech_DB" : "raw indices");
        if (ok) {
            std::cout << "  header: ";
            hexDump(env.headerBytes(16), 16);
            const size_t n = std::min<size_t>(env.entries().size(), 3);
            for (size_t i = 0; i < n; ++i) {
                const auto &e = env.entries()[i];
                std::printf("  [%zu] keys=%zu  data=%zu  first=(obj=%d w=%.2f)\n",
                            i, e.keyPath.size(), e.data.size(),
                            e.data.empty() ? 0 : e.data[0].schemaObjID,
                            e.data.empty() ? 0.0f : e.data[0].weight);
                // Project raw {keyType, payload} segments through the
                // speech domain when available; fall back to raw indices
                // when Speech_DB is missing so we always emit something.
                for (size_t k = 0; k < e.keyPath.size(); ++k) {
                    const auto &seg = e.keyPath[k];
                    if (speechLoaded) {
                        std::printf("       key[%zu] %s\n",
                                    k, speechDB.formatKey(seg).c_str());
                    } else {
                        std::printf("       key[%zu] type=%u [%d..%d]\n",
                                    k, seg.keyType, seg.keyMin, seg.keyMax);
                    }
                }
            }
        }
    } else {
        std::cout << "ENV_SOUND: not present\n";
    }

    // Speech_DB — full decode: three name maps (concept/tag/value),
    // per-concept priorities, per-tag flags, then nVoices voices each
    // holding one tag-DB per concept. Voice + concept indices in each
    // entry join back to the name maps for human-readable display.
    if (speechLoaded || readChunkBytesHL(db, "Speech_DB", bytes)) {
        // If we couldn't load above (rare), reload now so the per-block
        // bytes-size / tail metrics below still reflect a fresh decode.
        if (!speechLoaded) {
            speechLoaded = speechDB.loadFromChunk(bytes.data(), bytes.size());
        }
        const auto &sp = speechDB;
        std::printf("Speech_DB: %zu bytes  loaded=%s  concepts=%zu  tags=%zu  "
                    "values=%zu  voices=%u  entries=%zu  tail=%zu\n",
                    sp.rawSize(), speechLoaded ? "yes" : "no",
                    sp.conceptNames().size(),
                    sp.tagNames().size(),
                    sp.valueNames().size(),
                    sp.voiceCount(),
                    sp.entries().size(),
                    sp.tailBytes().size());
        if (speechLoaded) {
            std::cout << "  header: ";
            hexDump(sp.headerBytes(16), 16);
            const size_t cn = std::min<size_t>(sp.conceptNames().size(), 3);
            for (size_t i = 0; i < cn; ++i) {
                std::printf("  concept[%zu] = \"%s\"\n",
                            i, sp.conceptNames()[i].c_str());
            }
            const size_t n = std::min<size_t>(sp.entries().size(), 3);
            for (size_t i = 0; i < n; ++i) {
                const auto &e = sp.entries()[i];
                std::printf("  [%zu] voice=%u  concept=%u(%s)  keys=%zu  data=%zu\n",
                            i, e.voiceIndex, e.conceptIndex,
                            sp.conceptName(e.conceptIndex).c_str(),
                            e.keyPath.size(), e.data.size());
                if (!e.keyPath.empty()) {
                    std::printf("       path: %s\n",
                                sp.formatKeyPath(e.keyPath).c_str());
                }
                if (!e.data.empty()) {
                    std::printf("       first=(obj=%d w=%.2f)\n",
                                e.data[0].schemaObjID, e.data[0].weight);
                }
            }
        }
    } else {
        std::cout << "Speech_DB: not present\n";
    }

    // SchSamp
    if (readChunkBytesHL(db, "SchSamp", bytes)) {
        SchemaSamplesChunk ss;
        bool ok = ss.loadFromChunk(bytes.data(), bytes.size());
        std::printf("SchSamp: %zu bytes  loaded=%s  records=%zu\n",
                    bytes.size(), ok ? "yes" : "no",
                    ok ? ss.recordCount() : 0);
        if (ok && ss.recordCount() > 0) {
            // Show first three records as a sanity check.
            size_t n = std::min<size_t>(ss.recordCount(), 3);
            for (size_t i = 0; i < n; ++i) {
                const auto &r = ss.records()[i];
                std::printf("  [%zu] objID=%d samples=%zu  first=\"%s\"(freq=%u)\n",
                            i, r.objID, r.samples.size(),
                            r.samples.empty() ? "" : r.samples[0].name.c_str(),
                            r.samples.empty() ? 0 : r.samples[0].frequency);
            }
        }
    } else {
        std::cout << "SchSamp: not present\n";
    }

    // AIHearStat
    if (readChunkBytesHL(db, "AIHearStat", bytes)) {
        AIHearingStats st;
        bool ok = readAIHearStat(bytes.data(), bytes.size(), st);
        std::printf("AIHearStat: %zu bytes  parsed=%s\n",
                    bytes.size(), ok ? "yes" : "no");
        if (ok) {
            std::cout << "  rating         dist_mul   db_add\n";
            for (int i = 0; i < AI_HEARING_COUNT; ++i) {
                std::printf("  %-13s  %8.2f  %8d\n",
                            aiHearingRatingName(i),
                            st.dist_muls[i], st.db_adds[i]);
            }
        }
    } else {
        std::cout << "AIHearStat: not present\n";
    }

    // AISNDTWK
    if (readChunkBytesHL(db, "AISNDTWK", bytes)) {
        AISoundTweaks tw;
        bool ok = readAISndTwk(bytes.data(), bytes.size(), tw);
        std::printf("AISNDTWK: %zu bytes  parsed=%s\n",
                    bytes.size(), ok ? "yes" : "no");
        if (ok) {
            std::cout << "  sound type           defaultRange\n";
            for (int i = 0; i < AI_SOUND_TYPE_COUNT; ++i) {
                std::printf("  %-20s %d\n",
                            aiSoundTypeName(i), tw.defaultRanges[i]);
            }
        }
    } else {
        std::cout << "AISNDTWK: not present\n";
    }

    std::cout << "=== End sound chunk databases ===\n";
}

// `sound-desc` enumerates L$SoundDesc links (object → sound-descriptor
// archetype). Each link names a schema attached to an object event/script.
static void printSoundDescLinks() {
    using namespace ::Darkness;
    LinkServicePtr linkSvc = GET_SERVICE(LinkService);
    ObjectServicePtr objSvc = GET_SERVICE(ObjectService);
    PropertyServicePtr propSvc = GET_SERVICE(PropertyService);
    if (!linkSvc || !objSvc || !propSvc) {
        std::cerr << "sound-desc: required services unavailable\n";
        return;
    }
    RelationPtr rel = linkSvc->getRelation("SoundDescription");
    if (!rel) {
        std::cout << "sound-desc: SoundDescription relation not registered\n";
        return;
    }

    // Walk every named object (covers all link sources in practice).
    ::Darkness::Property *symProp = propSvc->getProperty("SymbolicName");
    if (!symProp || !symProp->getStorage()) {
        std::cerr << "sound-desc: SymbolicName property/storage missing\n";
        return;
    }
    IntIteratorPtr it = symProp->getStorage()->getAllStoredObjects();
    int total = 0;
    std::cout << "  Src       SrcName          Dst        DstName          LinkID\n";
    std::cout << "  --------  ---------------  ---------  ---------------  --------\n";
    while (!it->end()) {
        int src = it->next();
        if (src == 0) continue;
        LinkQueryResultPtr links = rel->getAllLinks(src, 0);
        while (!links->end()) {
            const Link &lnk = links->next();
            std::string srcName = objSvc->getName(lnk.src());
            std::string dstName = objSvc->getName(lnk.dst());
            std::printf("  %-8d  %-15s  %-9d  %-15s  %u\n",
                        lnk.src(), srcName.empty() ? "?" : srcName.c_str(),
                        lnk.dst(), dstName.empty() ? "?" : dstName.c_str(),
                        lnk.id());
            ++total;
        }
    }
    std::cout << "(" << total << " SoundDescription links)\n";
}

// ---------- Property-data inspector ----------
//
// `prop-dump <propName>` walks every object that has the named property,
// reading raw bytes and decoding them via DarkPropertyDefs.h struct
// definitions where we know the layout. Otherwise prints raw hex (up to
// 16 bytes per record). Use this to audit what's actually in dark.gam
// or a .mis file for any sound-related (or other) property.
//
// Examples:
//   prop-dump SchAttFac    → per-schema attenuation factors
//   prop-dump SchPlayPa    → per-archetype schema play-param overrides
//   prop-dump SpotAmb      → spot-ambient envelopes (inner/outer/level)
//   prop-dump LoudRoom     → per-room transmission multipliers
//   prop-dump AmbientHa    → ambient sound emitters
static std::string symNameFor(int objID) {
    ObjectServicePtr objSvc = GET_SERVICE(ObjectService);
    return objSvc ? objSvc->getName(objID) : std::string();
}

static void formatPropRecord(const std::string &propName, int objID,
                              const uint8_t *bytes, size_t sz) {
    using namespace ::Darkness;
    auto p = [&](const char *fmt, ...) {
        va_list ap; va_start(ap, fmt); std::vprintf(fmt, ap); va_end(ap);
    };
    std::string name = symNameFor(objID);
    p("  obj=%-5d %-20s sz=%-3zu  ", objID, name.empty() ? "(unnamed)" : name.c_str(), sz);

    auto readFloat = [&]() { float f; std::memcpy(&f, bytes, 4); return f; };
    auto readI32   = [&]() { int32_t i; std::memcpy(&i, bytes, 4); return i; };
    auto readU32   = [&]() { uint32_t u; std::memcpy(&u, bytes, 4); return u; };

    if (propName == "SchAttFac" && sz >= 4) {
        p("attenuation=%.3f", readFloat());
    } else if (propName == "SchPriori" && sz >= 4) {
        p("priority=%d", readI32());
    } else if (propName == "SchMsg" && sz >= 16) {
        char buf[17] = {0}; std::memcpy(buf, bytes, 16);
        p("label=\"%s\"", buf);
    } else if (propName == "SchLastSa" && sz >= 4) {
        p("lastSampleIdx=%d", readI32());
    } else if (propName == "SchPlayPa" && sz >= sizeof(PropSchemaPlayParams)) {
        PropSchemaPlayParams pp; std::memcpy(&pp, bytes, sizeof(pp));
        p("flags=0x%04x class=%u volume=%d pan=%d delay=%u fade=%d",
          pp.flags, pp.audioClass, pp.volume, pp.pan, pp.delay, pp.fade);
    } else if (propName == "SchLoopPa" && sz >= sizeof(PropSchemaLoopParams)) {
        PropSchemaLoopParams lp; std::memcpy(&lp, bytes, sizeof(lp));
        p("flags=0x%02x maxSamples=%u loopCount=%d interval=[%d,%d]",
          lp.flags, lp.maxSamples, lp.loopCount, lp.minInterval, lp.maxInterval);
    } else if (propName == "SpotAmb" && sz >= sizeof(PropSpotAmb)) {
        PropSpotAmb sa; std::memcpy(&sa, bytes, sizeof(sa));
        p("inner=%.2f outer=%.2f ambient=%.3f", sa.inner, sa.outer, sa.ambient);
    } else if (propName == "Heartbeat" && sz >= 4) {
        p("intervalMs=%d", readI32());
    } else if (propName == "MaxSpchPa" && sz >= 4) {
        p("maxPauseMs=%d", readI32());
    } else if (propName == "MinSpchPa" && sz >= 4) {
        p("minPauseMs=%d", readI32());
    } else if (propName == "AI_Hearin" && sz >= 4) {
        const char *names[] = {"VeryLow","Low","Normal","High","VeryHigh"};
        uint32_t r = readU32();
        const char *rn = (r >= 1 && r <= 5) ? names[r-1] : "?";
        p("rating=%u (%s)", r, rn);
    } else if (propName == "AI_SndTyp" && sz >= sizeof(PropAI_SndTyp)) {
        PropAI_SndTyp st; std::memcpy(&st, bytes, sizeof(st));
        char sig[33] = {0}; std::memcpy(sig, st.signal, 32);
        p("type=%u signal=\"%s\"", st.type, sig);
    } else if (propName == "VoiceIdx" && sz >= 4) {
        p("voice=%d", readI32());
    } else if (propName == "SpchVoice" && sz >= 16) {
        char buf[17] = {0}; std::memcpy(buf, bytes, 16);
        p("label=\"%s\"", buf);
    } else if (propName == "LoudRoom" && sz >= 4) {
        p("transmission=%.3f", readFloat());
    } else if (propName == "Acoustics" && sz >= 12) {
        // P$Acoustics layout: uint32 eax, int32 dampening, int32 height
        uint32_t eax; int32_t dampen, height;
        std::memcpy(&eax, bytes, 4);
        std::memcpy(&dampen, bytes + 4, 4);
        std::memcpy(&height, bytes + 8, 4);
        p("eax=%u dampening=%d height=%d", eax, dampen, height);
    } else if (propName == "AnimLight" && sz >= sizeof(PropAnimLight)) {
        // Canonical 76-byte layout — same struct LightingSystem reads.
        PropAnimLight al; std::memcpy(&al, bytes, sizeof(al));
        static const char *kModeNames[10] = {
            "FLIP","SMOOTH","RANDOM","MINBRIGHT","MAXBRIGHT",
            "ZERO","BRIGHTEN","DIM","SEMI_RANDOM","FLICKER"
        };
        const char *ms = (al.mode < 10) ? kModeNames[al.mode] : "?";
        p("ln=%d mode=%u(%s) min=%.1f max=%.1f bright=%dms dim=%dms rising=%u%s",
          al.lightNum, al.mode, ms, al.minBrightness, al.maxBrightness,
          al.brightenTime, al.dimTime, al.rising,
          al.inactive ? " INACTIVE" : "");
    } else if (propName == "AmbientHa" && sz >= 60) {
        // P$AmbientHack layout: int32 radius, int32 volume, uint32 flags,
        // char[16] schema (+ 32 bytes aux we ignore here).
        int32_t radius, volume; uint32_t flags; char schema[17] = {0};
        std::memcpy(&radius, bytes, 4);
        std::memcpy(&volume, bytes + 4, 4);
        std::memcpy(&flags,  bytes + 8, 4);
        std::memcpy(schema,  bytes + 12, 16);
        p("schema=\"%s\" radius=%d volume=%d flags=0x%04x",
          schema, radius, volume, flags);
    } else {
        // Unknown layout — dump hex (up to 24 bytes)
        size_t n = std::min<size_t>(sz, 24);
        p("hex=");
        for (size_t i = 0; i < n; ++i) p("%02x", bytes[i]);
        if (sz > n) p("...");
    }
    p("\n");
}

static void printPropDump(const std::string &propName) {
    PropertyServicePtr propSvc = GET_SERVICE(PropertyService);
    ::Darkness::Property *prop = propSvc->getProperty(propName);
    if (!prop) {
        std::cerr << "prop-dump: no property named '" << propName << "' registered\n";
        return;
    }
    DataStorage *storage = prop->getStorage();
    if (!storage) {
        std::cerr << "prop-dump: '" << propName << "' has null storage\n";
        return;
    }

    std::cout << "P$" << propName << " — objects with this property:" << std::endl;
    IntIteratorPtr it = storage->getAllStoredObjects();
    int totalConcrete = 0, totalArchetype = 0;
    while (!it->end()) {
        int objID = it->next();
        size_t sz = 0;
        const uint8_t *bytes = storage->getRawData(objID, sz);
        if (!bytes) continue;
        formatPropRecord(propName, objID, bytes, sz);
        if (objID < 0) ++totalArchetype; else ++totalConcrete;
    }
    std::cout << "(" << totalArchetype << " archetype + " << totalConcrete
              << " concrete = " << (totalArchetype + totalConcrete) << " total)"
              << std::endl;
}

// ---------- Initial door blocking gather ----------
//
// Walk every object that has a P$RotDoor or P$TransDoor property; for each
// door that starts CLOSED (status == 0), record its (room1, room2,
// soundBlocking) into a map keyed symmetrically by room ID pair.
//
// This mirrors what DarknessRender.cpp does at startup: it iterates
// DoorSystem's door list and calls AudioService::setBlockingFactor for
// each door with status==closed, blockSound>0, and room1≠room2.
// Replaying that gather here lets the headless trace see the same
// initial blocking state the runtime sees, so a closed door on the BFS
// path will inflate effectiveDistance the way it does in-game.
//
// We DON'T attempt to model runtime door open/close — that would require
// scripts + sim. The trace shows the state at level load.
struct DoorBlocker {
    int32_t objID;
    int32_t room1, room2;
    float   blocking;  // 0–1
    uint32_t status;   // 0=closed, 1=open, 2=closing, 3=opening, 4=halted
    const char *propName;
};
static std::vector<DoorBlocker> gatherInitialDoorBlocking() {
    std::vector<DoorBlocker> out;
    PropertyServicePtr propSvc = GET_SERVICE(PropertyService);

    auto walkProp = [&](const char *propName) {
        ::Darkness::Property *prop = propSvc->getProperty(propName);
        if (!prop) return;
        DataStorage *storage = prop->getStorage();
        if (!storage) return;
        IntIteratorPtr it = storage->getAllStoredObjects();
        while (!it->end()) {
            int objID = it->next();
            size_t sz = 0;
            const uint8_t *bytes = storage->getRawData(objID, sz);
            if (!bytes) continue;

            // Both RotDoor and TransDoor share the same prefix layout
            // through `room1, room2`. Use PropRotDoor for the field
            // offsets — sizeof differs but we only read fields up to
            // and including room1/room2.
            // (Could split per-type for safety; we rely on the property
            // chunks being correctly sized by the loader.)
            if (sz < sizeof(::Darkness::PropTransDoor)) continue;
            const auto *p = reinterpret_cast<const ::Darkness::PropTransDoor*>(bytes);

            DoorBlocker d{};
            d.objID    = objID;
            d.status   = p->status;
            d.blocking = p->blockSound / 100.0f;
            d.room1    = p->room1;
            d.room2    = p->room2;
            d.propName = propName;
            out.push_back(d);
        }
    };
    walkProp("RotDoor");
    walkProp("TransDoor");
    return out;
}

// ---------- Sound propagation tracing ----------
//
// `trace-path <src> <dst> [maxDist]` runs the same BFS through the room
// portal graph that AudioService uses at runtime. `src` and `dst` can be:
//   - Room IDs (positive int) — uses the room's geometric center as the
//     source/listener position.
//   - Object IDs with `o` prefix (e.g. `o332`) — uses the object's
//     ObjectService::position() as the source/listener position. The
//     room is resolved via roomFromPoint. Use this when the source is
//     an ambient sound object whose position is offset from the room
//     center (the room-center simplification can make BFS pick wrong
//     paths for ambients near a room boundary).
//
// Initial door blocking is applied (status==closed && blockSound>0). No
// LoudRoom multipliers are applied — MISS6's P$LoudRoom chunk is 0 bytes
// and most other Thief 2 missions are similar.
struct TraceEndpoint {
    Vector3 pos;
    ::Darkness::Room *room = nullptr;
    int roomID = -1;
    int objID = 0;  // 0 = room-center mode
};
static bool resolveEndpoint(const std::string &arg, TraceEndpoint &out,
                            const RoomServicePtr &roomSvc,
                            const ObjectServicePtr &objSvc)
{
    if (arg.empty()) return false;
    if (arg[0] == 'o' || arg[0] == 'O') {
        out.objID = std::stoi(arg.substr(1));
        out.pos = objSvc->position(out.objID);
        out.room = roomSvc->roomFromPoint(out.pos);
        out.roomID = out.room ? out.room->getRoomID() : -1;
        if (!out.room) {
            std::cerr << "trace-path: obj " << out.objID
                      << " at (" << out.pos.x << ", " << out.pos.y << ", "
                      << out.pos.z << ") is not inside any room\n";
            return false;
        }
    } else if (arg[0] == 'c' || arg[0] == 'C') {
        // "c:X,Y,Z" — arbitrary world coordinate. Useful for "listener at
        // the portal threshold" tests where neither room ID nor object
        // position is what you want.
        std::string body = arg.substr(arg[0] == 'c' || arg[0] == 'C' ? 1 : 0);
        if (!body.empty() && (body[0] == ':' || body[0] == '=')) body.erase(0, 1);
        float x = 0, y = 0, z = 0;
        if (std::sscanf(body.c_str(), "%f,%f,%f", &x, &y, &z) != 3) {
            std::cerr << "trace-path: bad coord '" << arg << "', expected c:X,Y,Z\n";
            return false;
        }
        out.pos = Vector3{x, y, z};
        out.room = roomSvc->roomFromPoint(out.pos);
        out.roomID = out.room ? out.room->getRoomID() : -1;
        if (!out.room) {
            std::cerr << "trace-path: coord (" << x << "," << y << "," << z
                      << ") is not inside any room\n";
            return false;
        }
    } else {
        out.roomID = std::stoi(arg);
        out.room = roomSvc->getRoomByID(out.roomID);
        if (!out.room) {
            std::cerr << "trace-path: room " << out.roomID << " not found\n";
            return false;
        }
        out.pos = out.room->getCenter();
    }
    return true;
}
static void printTracePath(const std::string &srcArg, const std::string &dstArg, float maxDist) {
    RoomServicePtr roomSvc = GET_SERVICE(RoomService);
    if (!roomSvc || !roomSvc->isLoaded()) {
        std::cerr << "trace-path: RoomService not loaded — does this database have ROOM_DB?\n";
        return;
    }
    ObjectServicePtr objSvc = GET_SERVICE(ObjectService);

    TraceEndpoint src{}, dst{};
    if (!resolveEndpoint(srcArg, src, roomSvc, objSvc)) return;
    if (!resolveEndpoint(dstArg, dst, roomSvc, objSvc)) return;

    int srcRoomID = src.roomID;
    int dstRoomID = dst.roomID;
    Vector3 srcCenter = src.pos;
    Vector3 dstCenter = dst.pos;

    std::cout << "Trace: "
              << (src.objID ? "obj " : "room ") << (src.objID ? src.objID : srcRoomID)
              << " → "
              << (dst.objID ? "obj " : "room ") << (dst.objID ? dst.objID : dstRoomID)
              << "  maxDist=" << maxDist << std::endl;
    std::cout << "  source: room=" << srcRoomID
              << "  pos=(" << srcCenter.x << ", " << srcCenter.y << ", " << srcCenter.z
              << ")  euclidean to dst: " << glm::length(dstCenter - srcCenter) << std::endl;
    std::cout << "  dest:   room=" << dstRoomID
              << "  pos=(" << dstCenter.x << ", " << dstCenter.y << ", " << dstCenter.z
              << ")" << std::endl;

    ::Darkness::Room *srcRoom = src.room;
    ::Darkness::Room *dstRoom = dst.room;
    auto dumpNeighbors = [&](::Darkness::Room *r) {
        std::cout << "  Room " << r->getRoomID() << " portals (" << r->getPortalCount() << "):";
        for (uint32_t i = 0; i < r->getPortalCount(); ++i) {
            ::Darkness::RoomPortal *p = r->getPortal(i);
            if (!p) { std::cout << " [null]"; continue; }
            ::Darkness::Room *far = p->getFarRoom();
            std::cout << "  ->" << (far ? far->getRoomID() : -1)
                      << "@" << "(" << p->getCenter().x << "," << p->getCenter().y
                      << "," << p->getCenter().z << ")";
        }
        std::cout << std::endl;
    };
    dumpNeighbors(srcRoom);
    dumpNeighbors(dstRoom);

    // Gather initial door blocking the way DarknessRender.cpp does at
    // startup: iterate every P$RotDoor / P$TransDoor, and if the door is
    // closed (status == 0) with blockSound > 0 and room1 != room2,
    // contribute blockSound/100 to the (room1, room2) pair.
    auto doors = gatherInitialDoorBlocking();
    std::unordered_map<uint64_t, float> blockingMap;
    auto packKey = [](int32_t a, int32_t b) -> uint64_t {
        // Symmetric: store both orderings so lookup works either way.
        return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32) |
               static_cast<uint32_t>(b);
    };
    int closedDoors = 0;
    for (const auto &d : doors) {
        bool closed = (d.status == 0);
        bool roomsOk = (d.room1 >= 0 && d.room2 >= 0 && d.room1 != d.room2);
        if (closed && d.blocking > 0.0f && roomsOk) {
            blockingMap[packKey(d.room1, d.room2)] = d.blocking;
            blockingMap[packKey(d.room2, d.room1)] = d.blocking;
            ++closedDoors;
        }
    }
    std::cout << "Doors: " << doors.size() << " total, " << closedDoors
              << " contributing initial blocking" << std::endl;
    // Print every closed door + verify the (room1, room2) pair actually
    // corresponds to a portal in our parsed graph. A door listing a
    // non-existent portal pair means either the door's room IDs are
    // miscomputed at level-author time, or our parser is reading
    // wrong rooms (off-by-one in the prop struct, etc.).
    auto roomsArePortalNeighbors = [&](int32_t a, int32_t b) -> bool {
        ::Darkness::Room *ra = roomSvc->getRoomByID(a);
        if (!ra) return false;
        for (uint32_t i = 0; i < ra->getPortalCount(); ++i) {
            ::Darkness::RoomPortal *p = ra->getPortal(i);
            if (!p) continue;
            ::Darkness::Room *far = p->getFarRoom();
            if (far && far->getRoomID() == b) return true;
        }
        return false;
    };
    int phantomDoors = 0;
    for (const auto &d : doors) {
        bool roomsOk = (d.room1 >= 0 && d.room2 >= 0 && d.room1 != d.room2);
        if (d.status == 0 && d.blocking > 0.0f && roomsOk) {
            bool portalExists = roomsArePortalNeighbors(d.room1, d.room2);
            std::printf("  [closed obj=%-5d %-10s] rooms=(%4d,%4d) blocking=%.2f  %s\n",
                        d.objID, d.propName, d.room1, d.room2, d.blocking,
                        portalExists ? "" : "PHANTOM (rooms not portal-neighbors)");
            if (!portalExists) ++phantomDoors;
        }
    }
    if (phantomDoors > 0) {
        std::cout << "  *** " << phantomDoors << " closed doors reference"
                  << " non-existent portal pairs ***" << std::endl;
    }

    ::Darkness::SoundPropParams params;
    params.maxDist = maxDist;
    params.doorBlocking = [&blockingMap, &packKey](int32_t a, int32_t b) -> float {
        auto it = blockingMap.find(packKey(a, b));
        return (it != blockingMap.end()) ? it->second : 0.0f;
    };
    // No LoudRoom — MISS6's P$LoudRoom chunk is empty anyway.

    std::vector<::Darkness::SoundPathHop> path;
    params.pathOut = &path;

    auto info = roomSvc->propagateSoundPath(srcCenter, dstCenter, srcRoom, dstRoom, params);

    std::cout << std::endl;
    if (!info.reached) {
        std::cout << "RESULT: UNREACHED — see [FALLBACK] PROP_FAIL on stderr for "
                     "DISCONNECTED vs MAXDIST_CUTOFF diagnosis." << std::endl;
        return;
    }

    std::cout << "RESULT: reached  realDist=" << info.realDistance
              << "  effDist=" << info.effectiveDistance
              << "  totalBlocking=" << info.totalBlocking
              << "  doorBlocking=" << info.doorBlocking
              << "  virtualPos=(" << info.virtualPosition.x << ","
              << info.virtualPosition.y << "," << info.virtualPosition.z << ")"
              << std::endl;
    std::cout << std::endl;
    std::cout << "Path (" << path.size() << " hops):" << std::endl;
    for (size_t i = 0; i < path.size(); ++i) {
        const auto &h = path[i];
        std::printf("  [%2zu] room=%-5d segDist=%7.2f  cumReal=%7.2f  cumEff=%7.2f  "
                    "doorBlk=%.3f  loudRm=%.3f  enterAt=(%.1f,%.1f,%.1f)\n",
                    i, h.roomID, h.segmentDist, h.cumRealDist, h.cumEffDist,
                    h.doorBlocking, h.loudRoom,
                    h.enterPortalCenter.x, h.enterPortalCenter.y,
                    h.enterPortalCenter.z);
    }
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
    std::cerr << "Darkness Headless Inspector" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Usage: " << prog << " <database> [command] [args] [--scripts <path>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Commands:" << std::endl;
    std::cerr << "  info              File type, parent DB chain, chunk count (default)" << std::endl;
    std::cerr << "  chunks            List all chunks with name, version, size" << std::endl;
    std::cerr << "  objects           List all objects with ID, name, position" << std::endl;
    std::cerr << "  properties [id]   List property types, or dump properties for an object" << std::endl;
    std::cerr << "  links [id]        List relation types, or dump links for an object" << std::endl;
    std::cerr << "  room-info <id> [<id> ...]" << std::endl;
    std::cerr << "                    Dump OBB planes, portal list, and portal-to-portal" << std::endl;
    std::cerr << "                    distance matrix for each given room ID." << std::endl;
    std::cerr << "  trace-path <src> <dst> [maxDist]" << std::endl;
    std::cerr << "                    Trace BFS through the portal graph from src room to dst" << std::endl;
    std::cerr << "                    room. Per-hop detail (segmentDist, cumEff, doorBlocking)." << std::endl;
    std::cerr << "                    No door/LoudRoom cost data — topology trace only." << std::endl;
    std::cerr << "                    maxDist defaults to 200." << std::endl;
    std::cerr << "  sound-chunks      Dump every sound-related chunk: AMBIENT, ENV_SOUND," << std::endl;
    std::cerr << "                    Speech_DB, SchSamp, AIHearStat, AISNDTWK." << std::endl;
    std::cerr << "  sound-desc        Enumerate L$SoundDesc links (sound-descriptor relations" << std::endl;
    std::cerr << "                    attached to objects)." << std::endl;
    std::cerr << "  prop-dump <name>  Dump every record of a named property (e.g. SchPlayPa," << std::endl;
    std::cerr << "                    SpotAmb, PrjSound, AmbientHa)." << std::endl;
    std::cerr << "  ambients          List every P$AmbientHa emitter with schema + position." << std::endl;
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
                          command == "links" ||
                          command == "trace-path" ||
                          command == "room-info" ||
                          command == "ambients" ||
                          command == "prop-dump" ||
                          command == "sound-desc");

    if (needsServices) {
        logger.registerLogListener(&stdlog);
        // Only show errors — INFO floods stderr and drowns the stdout data output.
        // Service initialization is verbose; the user wants clean data on stdout.
        logger.setLogLevel(Logger::LOG_LEVEL_ERROR);
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
        } else if (command == "prop-dump") {
            if (positionalArgs.size() < 3) {
                std::cerr << "prop-dump: need <propName> (e.g. SchAttFac, SpotAmb)\n";
                printUsage(argv[0]);
                return 1;
            }
            initServices();
            loadSchema(scriptsDir);
            (void)GET_SERVICE(RoomService);
            loadDatabase(dbFile);
            printPropDump(positionalArgs[2]);
        } else if (command == "ambients") {
            initServices();
            loadSchema(scriptsDir);
            (void)GET_SERVICE(RoomService);
            loadDatabase(dbFile);
            printAmbients();
        } else if (command == "sound-chunks") {
            // sound-chunks is service-less: open the FileGroup directly
            // and read the named chunks. No service init needed.
            FileGroupPtr db = openFileGroup(dbFile);
            printSoundChunks(db);
        } else if (command == "sound-desc") {
            initServices();
            loadSchema(scriptsDir);
            loadDatabase(dbFile);
            printSoundDescLinks();
        } else if (command == "room-info") {
            if (positionalArgs.size() < 3) {
                std::cerr << "room-info: need at least one <roomID>\n";
                printUsage(argv[0]);
                return 1;
            }
            initServices();
            loadSchema(scriptsDir);
            (void)GET_SERVICE(RoomService);
            loadDatabase(dbFile);
            for (size_t i = 2; i < positionalArgs.size(); ++i) {
                printRoomInfo(std::stoi(positionalArgs[i]));
                if (i + 1 < positionalArgs.size()) std::cout << std::endl;
            }
        } else if (command == "trace-path") {
            if (positionalArgs.size() < 4) {
                std::cerr << "trace-path: need <srcRoomID> <dstRoomID> [maxDist]\n";
                printUsage(argv[0]);
                return 1;
            }
            float maxDist = (positionalArgs.size() >= 5)
                            ? std::stof(positionalArgs[4])
                            : 200.0f;
            initServices();
            loadSchema(scriptsDir);
            // Force RoomService instantiation BEFORE loadDatabase so it
            // registers as a DatabaseListener and actually parses ROOM_DB.
            // Other services like ObjectService/PropertyService get pulled
            // in by loadSchema()'s GET_SERVICE calls; RoomService has no
            // schema dependency so without this it'd stay uninstantiated
            // and miss the onDBLoad broadcast.
            (void)GET_SERVICE(RoomService);
            loadDatabase(dbFile);
            printTracePath(positionalArgs[2], positionalArgs[3], maxDist);
        } else {
            std::cerr << "Unknown command: " << command << std::endl;
            printUsage(argv[0]);
            return 1;
        }
    } catch (const Darkness::BasicException &e) {
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
