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
#include "audio/SchemaParser.h"
#include "audio/SchemaTypes.h"
#include "audio/SpeechDatabase.h"

// Probe-plan verb dependencies — Capability A in PLAN.PROBE_DEBUG_TOOLING.
// The verb builds a real acoustic scene + queries AudioService for the
// dry-run probe plan; WR + TXLIST parsing lives here (in the renderer
// today) so the headless build doesn't pull src/main into DarknessServices.
#include "audio/AudioService.h"
#include "audio/AcousticMaterials.h"
#include "audio/AudioLog.h"
#include "audio/ProbeManager.h"
#include "WRChunkParser.h"
#include "TXListParser.h"
// findCameraCell — the WR cell lookup raycastWorld does before tracing.
// bgfx-free (see RayCaster.h), so it is safe in the headless binary.
#include "CellGeometry.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
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
    // AudioServiceFactory is lazy — registering it here costs nothing
    // for verbs that don't query AudioService, and `probe_plan` needs
    // it to be available via GET_SERVICE.
    svcMgr->registerFactory<AudioServiceFactory>();

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

// ---------- Room graph dump ----------
//
// `room_graph` prints the entire ROOM_DB portal graph in machine-readable
// line records, for offline distribution analysis (analysis/room_graph_stats.py).
// This is the data the probe-graph edge-range lever is derived from: the
// original engine's precomputed intra-room portal-to-portal hop tables
// (Room::getPortalDist — same access pattern as the maxRoomSpanFt
// derivation in AudioService::prepareProbeBakeParams) plus room centers
// and adjacency.
//
// Output records (space-separated, one per line):
//   ROOM <roomID> <cx> <cy> <cz> <portalCount>
//   PORTAL <nearRoomID> <farRoomID> <px> <py> <pz> <inradiusFt> <edgeCount>
//       one per directed portal; every physical portal appears twice
//       (once per side) — consumers de-duplicate back-links.
//       inradiusFt = distance from the portal center to its nearest bounding
//       edge plane = HALF the portal's narrow dimension. Separates real
//       architectural apertures (a 3 ft doorway ~= 1.5 ft) from ROOM_DB box
//       boundaries standing in open space (10 ft+). -1 = no edge planes.
//   PDIST <roomID> <i> <j> <dist>
//       intra-room portal-to-portal distance for portal indices i<j,
//       straight from the level compiler's precomputed matrix.
//   ROOMGRAPH_SUMMARY rooms=<n> portals_directed=<m> zero_portal_rooms=<k>
//       null_portals=<x> null_far_rooms=<y> zero_pdist_pairs=<z>
static void printRoomGraph() {
    RoomServicePtr roomSvc = GET_SERVICE(RoomService);
    if (!roomSvc || !roomSvc->isLoaded()) {
        std::cerr << "room_graph: RoomService not loaded — does this database "
                     "have ROOM_DB?\n";
        return;
    }

    const auto &rooms = roomSvc->getAllRooms();
    int roomCount = 0;
    int portalDirected = 0;
    int zeroPortalRooms = 0;
    int nullPortals = 0;
    int nullFarRooms = 0;
    // dist == 0 for i != j means the precomputed table entry is missing
    // or degenerate — counted so the offline stats can flag it.
    int zeroPDistPairs = 0;

    for (const auto &roomPtr : rooms) {
        if (!roomPtr) continue;
        ++roomCount;
        const int roomID = roomPtr->getRoomID();
        const Vector3 c = roomPtr->getCenter();
        const uint32_t pc = roomPtr->getPortalCount();
        std::printf("ROOM %d %.3f %.3f %.3f %u\n",
                    roomID, c.x, c.y, c.z, pc);
        if (pc == 0) ++zeroPortalRooms;

        for (uint32_t i = 0; i < pc; ++i) {
            ::Darkness::RoomPortal *p = roomPtr->getPortal(i);
            if (!p) { ++nullPortals; continue; }
            ::Darkness::Room *far = p->getFarRoom();
            if (!far) ++nullFarRooms;
            const Vector3 pcen = p->getCenter();
            // Aperture size = INRADIUS: the distance from the portal center
            // to its nearest bounding edge plane, i.e. HALF the portal's
            // NARROW dimension. RoomPortal stores no vertices — it is a
            // plane plus a set of bounding edge planes — so this is the
            // cheapest honest size metric available from the public API.
            //   a 3 ft doorway      -> ~1.5 ft
            //   a wide open boundary between two halves of one open volume
            //                       -> 10 ft+
            // This is what separates a REAL architectural aperture (where
            // sound diffracts around a jamb, so bend pairs earn their cost)
            // from a designer's ROOM_DB box boundary standing in mid-air
            // (where a bend pair buys nothing and just densifies the graph).
            float inradius = -1.0f;
            const uint32_t ec = p->getEdgeCount();
            for (uint32_t e = 0; e < ec; ++e) {
                const Plane &ep = p->getEdgePlane(e);
                const float n2 = glm::dot(ep.normal, ep.normal);
                if (n2 < 1e-8f) continue;   // degenerate edge plane
                const float dist = std::fabs(glm::dot(ep.normal, pcen) + ep.d)
                                 / std::sqrt(n2);
                if (inradius < 0.0f || dist < inradius) inradius = dist;
            }
            std::printf("PORTAL %d %d %.3f %.3f %.3f %.3f %u\n",
                        roomID, far ? far->getRoomID() : -1,
                        pcen.x, pcen.y, pcen.z, inradius, ec);
            ++portalDirected;
        }

        for (uint32_t i = 0; i < pc; ++i) {
            for (uint32_t j = i + 1; j < pc; ++j) {
                const float d = roomPtr->getPortalDist(i, j);
                if (d <= 0.0f) ++zeroPDistPairs;
                std::printf("PDIST %d %u %u %.3f\n", roomID, i, j, d);
            }
        }
    }

    std::printf("ROOMGRAPH_SUMMARY rooms=%d portals_directed=%d "
                "zero_portal_rooms=%d null_portals=%d null_far_rooms=%d "
                "zero_pdist_pairs=%d\n",
                roomCount, portalDirected, zeroPortalRooms, nullPortals,
                nullFarRooms, zeroPDistPairs);
}

// ---------- Probe / WR-cell containment audit ----------
//
// `probe_cell_audit` answers exactly one question: do probe positions land
// INSIDE the WR cell network that raycastWorld traverses?
//
// WHY THIS EXISTS: probes are placed from ROOM_DB (room centroids, portal
// centers) — a DIFFERENT spatial partition than the WR cells the raycaster
// walks. raycastWorld's first step is to locate the cell containing the ray
// ORIGIN; when that lookup fails it bails immediately (`if (curCell < 0)
// return false;`) WITHOUT TESTING ANY GEOMETRY, and callers read that false
// as "no hit" == clear line of sight. A probe outside the cell network
// therefore reports a clear path to every other probe in range — straight
// through solid world geometry. This audit counts that condition; it is the
// prime suspect for the pathing-graph edges observed passing through giant
// world geometry.
//
// Records (space-separated, one per line):
//   PROBE <index> <x> <y> <z> <cell>
//       real baked probe positions, read from a `.probes.*.positions.csv`
//       sidecar (index,x,y,z,radiusFt). Ground truth — needs a bake.
//   CAND room <roomID> <x> <y> <z> <cell>
//       ROOM_DB room center. NOTE: a PROXY — the real centroid probe sits at
//       floor+5 ft (or the vertical midpoint in short rooms), not the raw
//       geometric center. Indicative, not exact.
//   CAND portal <nearRoomID> <farRoomID> <x> <y> <z> <cell>
//       ROOM_DB portal center — EXACT: portal/door probes are placed at this
//       point (baseline) or flanking it along the normal (bends).
//   cell == -1 => findCameraCell found no containing cell => a ray cast FROM
//       this point is never actually traced.
//   CELLAUDIT_SUMMARY ...
//
// The CAND records need no bake, so this runs across every shipping level.
// Gap from a point to the nearest cell's bounding sphere, in engine feet.
// Distinguishes WHY a point failed findCameraCell:
//   gap <= 0  => the point is inside some cell's bounding sphere but outside
//               every cell's convex hull => it is IN SOLID, wedged in/behind
//               a wall between air cells. A bad probe: Steam Audio would also
//               trace its rays from inside geometry.
//   gap >  0  => outside every bounding sphere => deep solid, or off-map.
// (Cells are AIR volumes; solid is the absence of cells — so "outside all
// cells" always means "not in open space".)
static float nearestCellGapFt(const Darkness::WRParsedData &wr,
                              float x, float y, float z) {
    float best = 1e30f;
    for (uint32_t i = 0; i < wr.numCells; ++i) {
        const auto &c = wr.cells[i];
        const float dx = x - c.center.x;
        const float dy = y - c.center.y;
        const float dz = z - c.center.z;
        const float gap = std::sqrt(dx * dx + dy * dy + dz * dz) - c.radius;
        if (gap < best) best = gap;
    }
    return best;
}

static void printProbeCellAudit(const std::string &misPath,
                                const std::string &positionsCsv) {
    Darkness::WRParsedData wr;
    try {
        wr = Darkness::parseWRChunk(misPath);
    } catch (const std::exception &e) {
        std::fprintf(stderr,
            "probe_cell_audit: failed to parse WR chunk in '%s': %s\n",
            misPath.c_str(), e.what());
        return;
    }
    if (wr.numCells == 0) {
        std::fprintf(stderr,
            "probe_cell_audit: '%s' has 0 WR cells — nothing to test against\n",
            misPath.c_str());
        return;
    }

    int total = 0, outside = 0;
    int outsideInSolid = 0, outsideOffMap = 0;
    int roomTotal = 0, roomOutside = 0;
    int portalTotal = 0, portalOutside = 0;

    if (!positionsCsv.empty()) {
        // Ground-truth mode: audit the REAL baked probe positions.
        std::ifstream in(positionsCsv);
        if (!in) {
            std::fprintf(stderr,
                "probe_cell_audit: cannot open positions CSV '%s'\n",
                positionsCsv.c_str());
            return;
        }
        std::string line;
        while (std::getline(in, line)) {
            if (line.empty() || line[0] == '#') continue;
            if (line.compare(0, 6, "index,") == 0) continue;   // header row
            // index,x,y,z,radiusFt
            int idx = 0; float x = 0, y = 0, z = 0, r = 0;
            if (std::sscanf(line.c_str(), "%d,%f,%f,%f,%f",
                            &idx, &x, &y, &z, &r) < 4) continue;
            const int32_t cell = Darkness::findCameraCell(wr, x, y, z);
            ++total;
            if (cell < 0) {
                ++outside;
                const float gap = nearestCellGapFt(wr, x, y, z);
                if (gap <= 0.0f) ++outsideInSolid; else ++outsideOffMap;
                std::printf("PROBE %d %.3f %.3f %.3f %d gap=%.2f %s\n",
                            idx, x, y, z, cell, gap,
                            gap <= 0.0f ? "IN_SOLID" : "OFF_MAP");
            } else {
                std::printf("PROBE %d %.3f %.3f %.3f %d\n", idx, x, y, z, cell);
            }
        }
    } else {
        // No-bake mode: audit ROOM_DB-derived candidate positions.
        RoomServicePtr roomSvc = GET_SERVICE(RoomService);
        if (!roomSvc || !roomSvc->isLoaded()) {
            std::cerr << "probe_cell_audit: RoomService not loaded — does this "
                         "database have ROOM_DB?\n";
            return;
        }
        const auto &rooms = roomSvc->getAllRooms();
        for (const auto &roomPtr : rooms) {
            if (!roomPtr) continue;
            const int roomID = roomPtr->getRoomID();
            const Vector3 c = roomPtr->getCenter();
            int32_t cell = Darkness::findCameraCell(wr, c.x, c.y, c.z);
            ++total; ++roomTotal;
            if (cell < 0) {
                ++outside; ++roomOutside;
                const float gap = nearestCellGapFt(wr, c.x, c.y, c.z);
                if (gap <= 0.0f) ++outsideInSolid; else ++outsideOffMap;
                std::printf("CAND room %d %.3f %.3f %.3f %d gap=%.2f %s\n",
                            roomID, c.x, c.y, c.z, cell, gap,
                            gap <= 0.0f ? "IN_SOLID" : "OFF_MAP");
            } else {
                std::printf("CAND room %d %.3f %.3f %.3f %d\n",
                            roomID, c.x, c.y, c.z, cell);
            }

            const uint32_t pc = roomPtr->getPortalCount();
            for (uint32_t i = 0; i < pc; ++i) {
                ::Darkness::RoomPortal *p = roomPtr->getPortal(i);
                if (!p) continue;
                ::Darkness::Room *far = p->getFarRoom();
                const Vector3 pcen = p->getCenter();
                cell = Darkness::findCameraCell(wr, pcen.x, pcen.y, pcen.z);
                ++total; ++portalTotal;
                if (cell < 0) {
                    ++outside; ++portalOutside;
                    const float gap = nearestCellGapFt(wr, pcen.x, pcen.y, pcen.z);
                    if (gap <= 0.0f) ++outsideInSolid; else ++outsideOffMap;
                    std::printf("CAND portal %d %d %.3f %.3f %.3f %d gap=%.2f %s\n",
                                roomID, far ? far->getRoomID() : -1,
                                pcen.x, pcen.y, pcen.z, cell, gap,
                                gap <= 0.0f ? "IN_SOLID" : "OFF_MAP");
                } else {
                    std::printf("CAND portal %d %d %.3f %.3f %.3f %d\n",
                                roomID, far ? far->getRoomID() : -1,
                                pcen.x, pcen.y, pcen.z, cell);
                }
            }
        }
    }

    const double pct = total ? (100.0 * outside / total) : 0.0;
    std::printf("CELLAUDIT_SUMMARY cells=%u tested=%d outside=%d pct=%.2f "
                "in_solid=%d off_map=%d "
                "room_tested=%d room_outside=%d portal_tested=%d "
                "portal_outside=%d\n",
                wr.numCells, total, outside, pct,
                outsideInSolid, outsideOffMap,
                roomTotal, roomOutside, portalTotal, portalOutside);
}

// ---------- WR cell-portal aperture survey ----------
//
// `wr_portals` dumps every WR CELL PORTAL polygon with an aperture size, so
// the WR portal graph can be compared line-for-line against the ROOM_DB
// portal graph that `room_graph` dumps.
//
// WHY THIS EXISTS: ROOM_DB "portals" are boundaries between designer-drawn
// boxes. A box boundary need not correspond to any architectural opening —
// it frequently stands in mid-air, splitting one open volume in two. WR cell
// portals are the opposite: the level compiler produces them from the ACTUAL
// world geometry as the shared faces of a convex decomposition, and they are
// what raycastWorld traverses cell-to-cell. So a doorway's WR portal polygon
// IS the doorway, at its true dimensions.
//
// The convex decomposition still splits open volumes (a courtyard becomes
// many cells whose mutual portals are splitting planes, not apertures), so a
// size metric is still needed to tell a real aperture from a splitting plane.
//
// APERTURE SIZE METRIC — deliberately the same IDEA as room_graph's
// RoomPortal inradius (half the narrow dimension), so the two histograms are
// directly comparable, but computed honestly from real geometry:
//   inradiusFt = min over EDGE SEGMENTS of the distance from the polygon
//                centroid to that segment.
// RoomPortal stores only bounding PLANES, so room_graph had to use
// point-to-plane distance (an infinite plane over-estimates nothing but
// ignores the polygon's actual extent). A WR portal is a real polygon with
// VERTICES, so the distance is taken to the finite edge SEGMENT instead —
// for a convex polygon containing its centroid the two agree, and for a
// non-convex or skewed polygon the segment distance is the truthful one.
// Both answer "how far can you get from the middle of this opening before
// you hit its rim, in the tightest direction" = HALF THE NARROW DIMENSION:
//   a 3 ft doorway            -> ~1.5 ft
//   a courtyard splitting plane -> 10 ft+
// circumradiusFt (max vertex distance from the centroid) is emitted
// alongside it: inradius/circumradius separates a square aperture from a
// long thin slit, which a single number cannot.
//
// Records (space-separated, one per line):
//   WRPORTAL <cellIdx> <tgtCell> <cx> <cy> <cz> <inradiusFt> <circumradiusFt>
//            <numVerts> <areaSqFt>
//       one per portal POLYGON; every physical portal appears twice (once
//       from each side) — consumers de-duplicate by (min,max) cell pair.
//       tgtCell = -1 when the polygon names a cell outside the network.
//   WRPORTAL_SUMMARY cells=<n> portals=<m> degenerate=<k> bad_tgt=<x>
//
// Service-less: parseWRChunk reads the .mis directly, and cells carry their
// own geometry, so no ROOM_DB / service stack is involved.
static void printWRPortals(const std::string &misPath) {
    Darkness::WRParsedData wr;
    try {
        wr = Darkness::parseWRChunk(misPath);
    } catch (const std::exception &e) {
        std::fprintf(stderr,
            "wr_portals: failed to parse WR chunk in '%s': %s\n",
            misPath.c_str(), e.what());
        return;
    }
    if (wr.numCells == 0) {
        std::fprintf(stderr,
            "wr_portals: '%s' has 0 WR cells — nothing to survey\n",
            misPath.c_str());
        return;
    }

    int portalCount = 0, degenerate = 0, badTgt = 0;

    for (uint32_t ci = 0; ci < wr.numCells; ++ci) {
        const auto &cell = wr.cells[ci];
        // Portal polygons are the LAST numPortals entries of the polygon
        // list — the same slicing raycastWorld uses (`numSolid =
        // numPolygons - numPortals`, RayCaster.h).
        const int numSolid = static_cast<int>(cell.numPolygons)
                           - static_cast<int>(cell.numPortals);
        for (int pi = numSolid; pi < static_cast<int>(cell.numPolygons); ++pi) {
            if (pi < 0 || pi >= static_cast<int>(cell.polyIndices.size())) {
                ++degenerate;
                continue;
            }
            const auto &idx = cell.polyIndices[pi];
            const size_t nv = idx.size();
            if (nv < 3) { ++degenerate; continue; }

            // Centroid: plain vertex average. For the convex portal polygons
            // the compiler emits, this is interior, which is what makes the
            // "distance to the rim" reading meaningful.
            Vector3 cen(0.0f, 0.0f, 0.0f);
            bool badIdx = false;
            for (size_t v = 0; v < nv; ++v) {
                if (idx[v] >= cell.vertices.size()) { badIdx = true; break; }
                cen += cell.vertices[idx[v]];
            }
            if (badIdx) { ++degenerate; continue; }
            cen /= static_cast<float>(nv);

            float inradius = -1.0f;
            float circumradius = 0.0f;
            double area2 = 0.0;   // twice the fan-triangulated area
            for (size_t v = 0; v < nv; ++v) {
                const Vector3 &a = cell.vertices[idx[v]];
                const Vector3 &b = cell.vertices[idx[(v + 1) % nv]];

                const float cr = glm::length(a - cen);
                if (cr > circumradius) circumradius = cr;

                // Distance from the centroid to the finite edge segment ab.
                const Vector3 ab = b - a;
                const float ab2 = glm::dot(ab, ab);
                float d;
                if (ab2 < 1e-8f) {
                    d = glm::length(cen - a);   // degenerate (coincident) edge
                } else {
                    float t = glm::dot(cen - a, ab) / ab2;
                    t = glm::clamp(t, 0.0f, 1.0f);
                    d = glm::length(cen - (a + t * ab));
                }
                if (inradius < 0.0f || d < inradius) inradius = d;

                // Fan triangulation about the centroid. Portal polygons are
                // planar, so summing the triangle-cross magnitudes is exact.
                area2 += glm::length(glm::cross(a - cen, b - cen));
            }

            int32_t tgt = static_cast<int32_t>(cell.polygons[pi].tgtCell);
            if (tgt < 0 || tgt >= static_cast<int32_t>(wr.numCells)) {
                ++badTgt;
                tgt = -1;
            }

            std::printf("WRPORTAL %u %d %.3f %.3f %.3f %.3f %.3f %zu %.3f\n",
                        ci, tgt, cen.x, cen.y, cen.z,
                        inradius, circumradius, nv,
                        static_cast<float>(area2 * 0.5));
            ++portalCount;
        }
    }

    std::printf("WRPORTAL_SUMMARY cells=%u portals=%d degenerate=%d "
                "bad_tgt=%d\n",
                wr.numCells, portalCount, degenerate, badTgt);
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
//   prop-dump SpotAmb      → spotlight cones + ambient brightness (renderer)
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
    } else if (propName == "SpotAmb" && sz >= sizeof(PropSpotlightAndAmbient)) {
        PropSpotlightAndAmbient sa; std::memcpy(&sa, bytes, sizeof(sa));
        // RENDERER property: spotlight cone + ambient brightness override.
        p("innerAngleDeg=%.2f outerAngleDeg=%.2f ambientBrightness=%.3f",
          sa.innerAngleDeg, sa.outerAngleDeg, sa.ambientBrightness);
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

// ---------- probe_plan verb (Capability A in PLAN.PROBE_DEBUG_TOOLING) ----------
//
// Dry-run AudioService::bakeProbes' Darkness-side placement passes and
// print exact per-purpose probe counts WITHOUT invoking Steam Audio's
// multi-minute reflection/pathing bake. Used to iterate on probe
// placement reductions safely — see the .probes file check below for
// the CI-friendly diff-then-bake workflow.
//
// Acoustic-scene construction (WR / TXLIST parsing + buildAcousticScene)
// happens here in the headless side because adding WRChunkParser to
// DarknessServices would cross src/main → src/services include
// boundaries; the renderer (DarknessRender) is the only other caller
// and uses an inline copy for the same reason. Door OBB registration
// requires DoorSystem, which lives in the renderer init flow and is
// not replicated here — the [PROBE_PLAN] WARN banner emitted by
// AudioService::computeProbePlan surfaces this gap.

static bool buildAcousticSceneFromMissionFile(
    Darkness::AudioServicePtr audioSvc,
    const std::string &misPath)
{
    Darkness::WRParsedData wr;
    try {
        wr = Darkness::parseWRChunk(misPath);
    } catch (const std::exception &e) {
        std::fprintf(stderr,
            "[PROBE_PLAN] failed to parse WR chunk in '%s': %s\n",
            misPath.c_str(), e.what());
        return false;
    }

    Darkness::TXList txList;
    try {
        txList = Darkness::parseTXList(misPath);
    } catch (const std::exception &e) {
        // TXList parse failure is non-fatal — material lookup falls back
        // to "generic" everywhere. Probe counts are unaffected because
        // material keywords drive IR coloration, not placement.
        std::fprintf(stderr,
            "[PROBE_PLAN] TXLIST parse failed (%s) — proceeding with "
            "'generic' materials\n", e.what());
        txList = Darkness::TXList{};
    }

    Darkness::AcousticSceneData fullScene;

    // Vertex dedup hash — same quantization as DarknessRender.cpp's
    // acoustic-scene assembly to keep the IPLScene byte-identical.
    struct VertKey {
        int32_t x, y, z;
        bool operator==(const VertKey &o) const {
            return x == o.x && y == o.y && z == o.z;
        }
    };
    struct VertKeyHash {
        size_t operator()(const VertKey &k) const {
            size_t h = 0x811c9dc5u;
            h ^= static_cast<size_t>(k.x); h *= 0x01000193u;
            h ^= static_cast<size_t>(k.y); h *= 0x01000193u;
            h ^= static_cast<size_t>(k.z); h *= 0x01000193u;
            return h;
        }
    };
    std::unordered_map<VertKey, uint32_t, VertKeyHash> vertexMap;
    vertexMap.reserve(wr.numCells * 20);

    auto getVertexIndex = [&](float x, float y, float z) -> uint32_t {
        VertKey key{static_cast<int32_t>(std::round(x * 100.0f)),
                    static_cast<int32_t>(std::round(y * 100.0f)),
                    static_cast<int32_t>(std::round(z * 100.0f))};
        auto it = vertexMap.find(key);
        if (it != vertexMap.end()) return it->second;
        uint32_t idx = static_cast<uint32_t>(fullScene.vertices.size() / 3);
        fullScene.vertices.push_back(x);
        fullScene.vertices.push_back(y);
        fullScene.vertices.push_back(z);
        vertexMap[key] = idx;
        return idx;
    };

    for (uint32_t ci = 0; ci < wr.numCells; ++ci) {
        const auto &cell = wr.cells[ci];
        int numSolid = cell.numPolygons - cell.numPortals;
        for (int pi = 0; pi < cell.numPolygons; ++pi) {
            const auto &poly = cell.polygons[pi];
            bool isPortal = (pi >= numSolid);
            // Skip BSP-split phantom portals (see DarknessRender.cpp
            // comment for the full rationale; they'd otherwise produce
            // phantom reflections / blocked probe-to-probe visibility
            // rays).
            bool isNonRenderedPortal = isPortal && (pi >= cell.numTextured);
            if (isNonRenderedPortal) continue;
            if (!isPortal && pi < cell.numTextured && cell.texturing[pi].txt == 249)
                continue;
            if (!isPortal && pi < cell.numTextured) {
                uint8_t txtIdx = cell.texturing[pi].txt;
                if (txtIdx >= 247) continue;  // WATERIN / WATEROUT / BACKHACK
            }
            std::string texName = "generic";
            if (isPortal) {
                texName = "_portal";
            } else if (pi < cell.numTextured) {
                uint8_t txtIdx = cell.texturing[pi].txt;
                if (txtIdx < txList.textures.size()) {
                    const auto &entry = txList.textures[txtIdx];
                    if (!entry.fullPath.empty()) texName = entry.fullPath;
                }
            }
            if (poly.count < 3) continue;
            std::vector<uint32_t> polyVerts(poly.count);
            for (int vi = 0; vi < poly.count; ++vi) {
                uint8_t idx = cell.polyIndices[pi][vi];
                const auto &v = cell.vertices[idx];
                polyVerts[vi] = getVertexIndex(v.x, v.y, v.z);
            }
            for (int t = 1; t < poly.count - 1; ++t) {
                fullScene.indices.push_back(static_cast<int32_t>(polyVerts[0]));
                fullScene.indices.push_back(static_cast<int32_t>(polyVerts[t + 1]));
                fullScene.indices.push_back(static_cast<int32_t>(polyVerts[t]));
                fullScene.texNames.push_back(texName);
            }
            // Floor-poly probe candidate.  Dark Engine BSP plane normals
            // face inward (see CellGeometry.h); for a floor polygon (cell
            // bottom) the inward normal points +Z.  One candidate per
            // qualifying polygon; AudioService applies the height offset
            // and downstream filters.
            if (!isPortal && poly.plane < cell.planes.size()
                && cell.planes[poly.plane].normal.z > 0.5f)
            {
                Vector3 c(0.0f);
                for (int vi = 0; vi < poly.count; ++vi)
                    c += cell.vertices[cell.polyIndices[pi][vi]];
                c /= static_cast<float>(poly.count);
                fullScene.floorProbeCandidates.push_back(c);
            }
        }
    }

    size_t numTris = fullScene.indices.size() / 3;
    std::fprintf(stderr,
                 "[PROBE_PLAN] Acoustic mesh: %zu vertices, %zu triangles (%u cells)\n",
                 fullScene.vertices.size() / 3, numTris, wr.numCells);

    // Cell-centroid Z distribution. UNIFORMFLOOR raycasts down from the
    // OBB top to find floor surfaces; if the cell centroids cluster far
    // below the OBB top (e.g. a few outlier-high cells stretch the OBB),
    // the raycasts must traverse a lot of empty space to reach real
    // floors.  Surface the per-decile spread so an operator can spot
    // outlier rooms that pull the OBB top way above the playable mass.
    if (wr.numCells > 0) {
        std::vector<float> cz;
        cz.reserve(wr.numCells);
        for (uint32_t ci = 0; ci < wr.numCells; ++ci)
            cz.push_back(wr.cells[ci].center.z);
        std::sort(cz.begin(), cz.end());
        auto pct = [&](float p) -> float {
            size_t idx = std::min(cz.size() - 1,
                                  static_cast<size_t>(p * cz.size()));
            return cz[idx];
        };
        std::fprintf(stderr,
            "[PROBE_PLAN] Cell-centroid Z (ft): min=%.0f p05=%.0f p25=%.0f "
            "med=%.0f p75=%.0f p95=%.0f max=%.0f  (mesh-bound range affects "
            "UNIFORMFLOOR ray length)\n",
            cz.front(), pct(0.05f), pct(0.25f), pct(0.50f), pct(0.75f),
            pct(0.95f), cz.back());
    }

    // Reflection-probe placement-scheme comparison.  UNIFORMFLOOR assumes
    // a mostly-flat world (rays cast down from one OBB) and fails on
    // multi-level missions like miss14 where rooms span 350 ft of Z.
    // Emit counts for the alternative schemes so an operator can pick
    // the right primitive across all 15 missions before committing to a
    // rewrite.  Per-scheme probe-count estimates, with the dry-run grid
    // spacing fixed at 5 ft (the current reflection default):
    //   ROOM_CENTROID   — one probe at the center of each ROOM_DB room
    //                     (coarsest; matches per-zone EAX-style reverb)
    //   CELL_CENTROID   — one probe at each WR-cell center (one per BSP
    //                     convex polyhedron)
    //   FLOOR_POLY      — one probe per upward-facing cell polygon
    //                     (normal_z > 0.5); handles multi-floor cells
    //   FLOOR_GRID5ft   — area-driven grid sample at 5 ft spacing
    //                     within each floor polygon's XY extent
    //                     (≈ UNIFORMFLOOR density, per-cell-aware)
    {
        RoomServicePtr roomSvc = GET_SERVICE(RoomService);
        size_t roomCount = 0;
        if (roomSvc && roomSvc->isLoaded()) {
            // RoomService doesn't expose a count getter; iterate over ID range.
            // Cheap: room IDs are dense small ints starting at 0.
            for (int rid = 0; rid < 4096; ++rid) {
                if (roomSvc->getRoomByID(rid) != nullptr) ++roomCount;
            }
        }

        size_t cellCount      = wr.numCells;
        size_t floorPolyCount = 0;
        double floorAreaTotal = 0.0;
        for (uint32_t ci = 0; ci < wr.numCells; ++ci) {
            const auto &cell = wr.cells[ci];
            // Solid (non-portal) polygons are pi < numSolid.
            int numSolid = cell.numPolygons - cell.numPortals;
            for (int pi = 0; pi < numSolid; ++pi) {
                const auto &poly = cell.polygons[pi];
                if (poly.count < 3) continue;
                if (poly.plane >= cell.planes.size()) continue;
                // BSP plane normal: convention is normal points INTO the
                // cell's free space.  Upward-facing floors in engine Z-up
                // have plane.normal.z > 0.
                const Vector3 &n = cell.planes[poly.plane].normal;
                if (n.z <= 0.5f) continue;
                ++floorPolyCount;
                // Horizontal area = shoelace formula on (x,y) projection.
                // Negligible cost; sums fan-triangles for accuracy.
                double area = 0.0;
                const auto &idxs = cell.polyIndices[pi];
                const auto &v0 = cell.vertices[idxs[0]];
                for (int t = 1; t < poly.count - 1; ++t) {
                    const auto &va = cell.vertices[idxs[t]];
                    const auto &vb = cell.vertices[idxs[t + 1]];
                    double ex1 = va.x - v0.x, ey1 = va.y - v0.y;
                    double ex2 = vb.x - v0.x, ey2 = vb.y - v0.y;
                    area += 0.5 * std::fabs(ex1 * ey2 - ex2 * ey1);
                }
                floorAreaTotal += area;
            }
        }
        const double gridSpacing = 5.0;  // ft, matches default
        size_t gridProbeEst =
            static_cast<size_t>(floorAreaTotal / (gridSpacing * gridSpacing));
        std::fprintf(stderr,
            "[PROBE_PLAN] Scheme comparison (estimates):\n"
            "  ROOM_CENTROID     %6zu probes  (ROOM_DB rooms)\n"
            "  CELL_CENTROID     %6zu probes  (WR cells)\n"
            "  FLOOR_POLY        %6zu probes  (cell polys with normal.z > 0.5)\n"
            "  FLOOR_GRID5ft     %6zu probes  (floor area %.0f ft² / 25 ft²)\n",
            roomCount, cellCount, floorPolyCount, gridProbeEst,
            floorAreaTotal);
    }

    // Push the per-texture material keyword table (TXLIST → keyword)
    // BEFORE buildAcousticScene so the AudioService materials lookup
    // sees them at scene assembly time.
    {
        size_t numTex = txList.textures.size();
        std::vector<std::string> materials(numTex);
        for (size_t i = 0; i < numTex; ++i) {
            const auto &entry = txList.textures[i];
            if (!entry.fullPath.empty()) {
                materials[i] = Darkness::lookupAcousticMaterialKeyword(entry.fullPath);
            } else {
                materials[i] = "generic";
            }
        }
        audioSvc->setTextureMaterials(std::move(materials));
    }

    return audioSvc->buildAcousticScene(fullScene);
}

// ---------- sound_db verb ----------
//
// Walks the schema directory and emits one CSV row per schema with the
// fields needed to make data-driven decisions about pathing/audible
// range without re-loading every mission. Schema-level only — per-mission
// P$SchPlayPa overrides are intentionally ignored so the table reflects
// the global archetype defaults.
//
// Output: perf/sound_db/sound_metadata.csv (or path passed as positional[2])
//
// Per-schema max audible distance follows the Dark Engine formula from
// `AmbientSoundManager.cpp`:
//   maxDist_ft = max(1, (5000 + volume_cb) / 55) * attenuation_factor
// where `volume_cb` is the schema's `volume` field (centibels) and
// `attenuation_factor` is its P$SchAttFac (default 1.0, ~20 for bells).
static const char *audioClassStr(::Darkness::SchemaAudioClass c) {
    using AC = ::Darkness::SchemaAudioClass;
    switch (c) {
        case AC::Noise:      return "noise";
        case AC::Speech:     return "speech";
        case AC::Ambient:    return "ambient";
        case AC::Music:      return "music";
        case AC::MetaUI:     return "metaui";
        case AC::PlayerFeet: return "player_feet";
        case AC::OtherFeet:  return "other_feet";
        case AC::Collisions: return "collisions";
        case AC::Weapons:    return "weapons";
        case AC::Monsters:   return "monsters";
    }
    return "noise";
}

static int runSoundDbVerb(const std::string &schemasPath,
                          const std::string &outPathArg,
                          const std::string &dbFile,
                          const std::string &scriptsDir)
{
    if (schemasPath.empty()) {
        std::fprintf(stderr,
            "[SOUND_DB] --schemas <DIR> required\n");
        return 1;
    }

    ::Darkness::SchemaParser parser;
    if (!parser.loadDirectory(schemasPath)) {
        std::fprintf(stderr,
            "[SOUND_DB] failed to load schemas from %s\n",
            schemasPath.c_str());
        return 1;
    }
    std::fprintf(stderr,
        "[SOUND_DB] loaded %zu schemas from %s\n",
        parser.schemaCount(), schemasPath.c_str());

    // OPTIONAL overlay pass.  Schema .sch text files set attenuation
    // factor = 1.0 by default; the per-archetype P$SchAttFac override
    // lives in dark.gam (and per-mission .mis files).  Without these
    // overrides every schema looks like a 91 ft sound, missing the
    // ~20× boost authored for bells / long-carry SFX.  When the user
    // passes a database file (mission .mis or dark.gam), spin up the
    // service stack, load the DB, and overlay P$SchAttFac onto the
    // local SchemaParser instance so the CSV reflects real audible
    // distances.
    int attFacOverrides = 0;
    if (!dbFile.empty()) {
        initServices();
        loadSchema(scriptsDir);
        loadDatabase(dbFile);
        ::Darkness::PropertyServicePtr propSvc =
            GET_SERVICE(::Darkness::PropertyService);
        ::Darkness::ObjectServicePtr objSvc =
            GET_SERVICE(::Darkness::ObjectService);
        if (propSvc && objSvc) {
            ::Darkness::Property *attFacProp = propSvc->getProperty("SchAttFac");
            if (attFacProp && attFacProp->getStorage()) {
                auto *storage = attFacProp->getStorage();
                auto it = storage->getAllStoredObjects();
                while (!it->end()) {
                    int objID = it->next();
                    size_t sz = 0;
                    const uint8_t *bytes = storage->getRawData(objID, sz);
                    if (!bytes || sz < sizeof(float)) continue;
                    float factor;
                    std::memcpy(&factor, bytes, sizeof(float));
                    if (factor <= 0.0f) continue;
                    std::string schemaName = objSvc->getName(objID);
                    if (schemaName.empty()) continue;
                    auto *sch = parser.findSchemaMutable(schemaName);
                    if (!sch) continue;
                    sch->playParams.attenuationFactor = factor;
                    ++attFacOverrides;
                }
            }
        }
        std::fprintf(stderr,
            "[SOUND_DB] applied %d P$SchAttFac overrides from %s\n",
            attFacOverrides, dbFile.c_str());
    } else {
        std::fprintf(stderr,
            "[SOUND_DB] no database file — schema-text defaults only "
            "(all atten_factor=1.0). Pass a .mis or dark.gam as first "
            "positional to apply P$SchAttFac overrides.\n");
    }

    // Resolve output path; default into the existing perf/ tree.
    std::string outPath = outPathArg.empty()
        ? std::string("perf/sound_db/sound_metadata.csv")
        : outPathArg;
    // Create parent dirs (mkdir -p semantics). The existing `perf/`
    // siblings are gitignored already; this just makes the verb usable
    // out-of-the-box.
    {
        std::string dir = outPath;
        size_t slash = dir.find_last_of("/\\");
        if (slash != std::string::npos) {
            dir = dir.substr(0, slash);
            // Use mkdir -p via shell; portable enough for darwin/linux
            // and keeps the verb code free of POSIX directory APIs.
            std::string cmd = "mkdir -p '" + dir + "'";
            std::system(cmd.c_str());
        }
    }

    std::ofstream out(outPath);
    if (!out.is_open()) {
        std::fprintf(stderr,
            "[SOUND_DB] cannot open %s for writing\n", outPath.c_str());
        return 1;
    }

    out << "# Generated by darknessHeadless sound_db. Schema-level "
        << "defaults; per-mission P$SchPlayPa overrides not applied.\n";
    out << "schema,archetype,gain_cb,atten_factor,max_audible_ft,"
        << "audio_class,priority,is_looping,is_poly_loop,max_samples_loop,"
        << "n_samples,fade_ms,initial_delay_ms,schema_flags_hex,"
        << "has_env_tag,has_voice,sample_names\n";

    size_t emitted = 0;
    for (const auto &kv : parser.schemas()) {
        const auto &s = kv.second;
        const auto &pp = s.playParams;
        const auto &lp = s.loopParams;

        const float gainCb = static_cast<float>(pp.volume);
        float atten = pp.attenuationFactor;
        if (atten <= 0.01f) atten = 1.0f;
        float maxDist = (5000.0f + gainCb) / 55.0f;
        if (maxDist < 1.0f) maxDist = 1.0f;
        maxDist *= atten;

        // Sample names pipe-joined so the CSV stays single-row per schema.
        std::ostringstream samples;
        for (size_t i = 0; i < s.samples.size(); ++i) {
            if (i > 0) samples << "|";
            samples << s.samples[i].name;
        }

        char flagsHex[16];
        std::snprintf(flagsHex, sizeof(flagsHex), "0x%X",
                      static_cast<unsigned>(pp.flags));

        out << s.name << ","
            << s.archetypeName << ","
            << pp.volume << ","
            << std::fixed << std::setprecision(2) << atten << ","
            << std::fixed << std::setprecision(1) << maxDist << ","
            << audioClassStr(pp.audioClass) << ","
            << pp.priority << ","
            << (lp.isLooping ? 1 : 0) << ","
            << (lp.isPoly ? 1 : 0) << ","
            << static_cast<int>(lp.maxSamples) << ","
            << s.samples.size() << ","
            << pp.fade << ","
            << pp.initialDelay << ","
            << flagsHex << ","
            << (s.hasEnvTags() ? 1 : 0) << ","
            << (s.hasVoice() ? 1 : 0) << ","
            << samples.str()
            << "\n";
        ++emitted;
    }
    out.close();

    std::fprintf(stdout,
        "[SOUND_DB] wrote %zu schema rows to %s\n",
        emitted, outPath.c_str());
    return 0;
}

// ---------- txlist_audit verb ----------
//
// Enumerates every texture name in the mission's TXLIST chunk and runs
// each through lookupAcousticMaterialKeyword() — the SAME function the
// audio engine uses to map a world surface to an IPLMaterial preset.
// Any name that resolves to "generic" has no acoustic-material keyword
// and will fall through to the opaque-reflective fallback at runtime
// (corrupting reverb + occlusion for every surface using it).
//
// Output (one line per texture, deduped by name):
//   [TXLIST_AUDIT][UNMAPPED] <family/name>          ← no keyword match
//   [TXLIST_AUDIT][MAP]      <family/name> -> <kw>   ← matched keyword
//   [TXLIST_AUDIT] summary: <unmapped>/<total> unmapped
//
// Aggregate the complete cross-mission gap set with, e.g.:
//   for m in MISS*.mis; do darknessHeadless "$m" txlist_audit; done \
//     | grep UNMAPPED | awk '{print $2}' | sort -u
//
// The name passed to lookupAcousticMaterialKeyword is TXEntry.fullPath
// ("family/name", or bare "name" when the texture has no family) — the
// IDENTICAL string the runtime acoustic-mesh build feeds the lookup
// (see buildAcousticSceneFromMissionFile above). No services / no Steam
// Audio runtime required — pure TXLIST analysis.
static int runTxlistAuditVerb(const std::string &misPath)
{
    namespace D = Darkness;

    std::fprintf(stdout, "[TXLIST_AUDIT] mission=%s\n", misPath.c_str());

    D::TXList txList;
    try {
        txList = D::parseTXList(misPath);
    } catch (const std::exception &e) {
        std::fprintf(stderr,
            "[TXLIST_AUDIT] failed to parse TXLIST chunk in '%s': %s\n",
            misPath.c_str(), e.what());
        return 1;
    }

    // Dedupe by fullPath — TXLIST can list the same texture under
    // multiple indices, and we only care about distinct names.
    std::set<std::string> seen;
    size_t unmapped = 0;
    for (const auto &entry : txList.textures) {
        if (entry.fullPath.empty()) continue;        // unnamed slot
        if (!seen.insert(entry.fullPath).second) continue; // already reported

        std::string kw = D::lookupAcousticMaterialKeyword(entry.fullPath);
        if (kw == "generic") {
            ++unmapped;
            std::fprintf(stdout, "[TXLIST_AUDIT][UNMAPPED] %s\n",
                         entry.fullPath.c_str());
        } else {
            std::fprintf(stdout, "[TXLIST_AUDIT][MAP] %s -> %s\n",
                         entry.fullPath.c_str(), kw.c_str());
        }
    }

    std::fprintf(stdout, "[TXLIST_AUDIT] summary: %zu/%zu unmapped\n",
                 unmapped, seen.size());
    return 0;
}

// ---------- mesh_validate verb ----------
//
// Constructs the same AcousticSceneData that darknessRender hands to
// AudioService::buildAcousticScene, then runs a half-edge boundary
// count over its triangle list. Outputs:
//
//   <mis>.acoustic.obj           — Wavefront OBJ of the static mesh.
//   <mis>.acoustic.boundary.txt  — every boundary edge (midpoint +
//                                  endpoints + endpoint-vertex index
//                                  pair). One line per edge.
//
// A "boundary edge" is one used by exactly one triangle (no opposing
// triangle on the other side). For a watertight room mesh this should
// be zero. Non-zero boundary count means the mesh has holes — exactly
// the seam Steam Audio's single-ray `scene.isOccluded` can slip
// through while the multi-sample volumetric occlusion (rays at small
// angular offset) correctly hits the surrounding closed geometry.
//
// Edges shared by 3+ triangles ("non-manifold edges") are also
// reported because they indicate winding / portal-pair-coincidence
// artefacts that can degrade BVH precision for grazing rays.
//
// Optional positional filter args: `--at X Y Z R` reports only edges
// whose midpoint is within R feet of `(X,Y,Z)`. Useful for zooming in
// on a known cross-wall leak — e.g.
//   darknessHeadless mesh_validate MISS6.mis --at 79.9 -45.4 -47.0 30
// to look only at the corridor around the strlight_lp source.
//
// Independent of any Steam Audio runtime state — no IPLContext / no
// IPLScene created. Pure WR-input analysis.
static int runMeshValidateVerb(const std::string &misPath,
                                const Darkness::Vector3 *filterPos,
                                float filterRadiusFt)
{
    namespace D = Darkness;

    std::fprintf(stdout, "[MESH_VALIDATE] mission=%s\n", misPath.c_str());
    if (filterPos) {
        std::fprintf(stdout,
            "[MESH_VALIDATE] filter pos=(%.2f, %.2f, %.2f) radius=%.2fft\n",
            filterPos->x, filterPos->y, filterPos->z, filterRadiusFt);
    }

    D::WRParsedData wr;
    try {
        wr = D::parseWRChunk(misPath);
    } catch (const std::exception &e) {
        std::fprintf(stderr,
            "[MESH_VALIDATE] failed to parse WR chunk in '%s': %s\n",
            misPath.c_str(), e.what());
        return 1;
    }

    D::TXList txList;
    try {
        txList = D::parseTXList(misPath);
    } catch (const std::exception &) {
        // TXList parse failure is non-fatal — mesh geometry is independent
        // of material lookup.
        txList = D::TXList{};
    }

    // ── Build AcousticSceneData (same logic as
    //    buildAcousticSceneFromMissionFile above and DarknessRender.cpp's
    //    in-engine assembly). Kept inline rather than refactored so a
    //    drift in the production path is detectable by diffing this verb
    //    against the runtime mesh — the whole point of this verb is to
    //    cross-check the production mesh-build assumptions. If the
    //    production path changes, this verb's output diverges, which is
    //    the alarm we want. Future refactor: extract a single helper.
    D::AcousticSceneData fullScene;

    struct VertKey {
        int32_t x, y, z;
        bool operator==(const VertKey &o) const {
            return x == o.x && y == o.y && z == o.z;
        }
    };
    struct VertKeyHash {
        size_t operator()(const VertKey &k) const {
            size_t h = 0x811c9dc5u;
            h ^= static_cast<size_t>(k.x); h *= 0x01000193u;
            h ^= static_cast<size_t>(k.y); h *= 0x01000193u;
            h ^= static_cast<size_t>(k.z); h *= 0x01000193u;
            return h;
        }
    };
    std::unordered_map<VertKey, uint32_t, VertKeyHash> vertexMap;
    vertexMap.reserve(wr.numCells * 20);

    auto getVertexIndex = [&](float x, float y, float z) -> uint32_t {
        VertKey key{static_cast<int32_t>(std::round(x * 100.0f)),
                    static_cast<int32_t>(std::round(y * 100.0f)),
                    static_cast<int32_t>(std::round(z * 100.0f))};
        auto it = vertexMap.find(key);
        if (it != vertexMap.end()) return it->second;
        uint32_t idx = static_cast<uint32_t>(fullScene.vertices.size() / 3);
        fullScene.vertices.push_back(x);
        fullScene.vertices.push_back(y);
        fullScene.vertices.push_back(z);
        vertexMap[key] = idx;
        return idx;
    };

    // Tally why polygons are skipped so the boundary-edge count can be
    // interpreted in context: a high boundary count with lots of
    // txt>=247 skips means the holes are intentional skybox/water
    // openings (the suspect #1 identified in the cross-wall-leak
    // audit). A high boundary count without skips means real
    // T-junctions or missing tessellation in the WR data.
    size_t skipBspPhantom   = 0;
    size_t skipBackhack249  = 0;
    size_t skipTxt247Plus   = 0;
    size_t emittedSolid     = 0;
    size_t emittedPortal    = 0;

    for (uint32_t ci = 0; ci < wr.numCells; ++ci) {
        const auto &cell = wr.cells[ci];
        int numSolid = cell.numPolygons - cell.numPortals;
        for (int pi = 0; pi < cell.numPolygons; ++pi) {
            const auto &poly = cell.polygons[pi];
            bool isPortal = (pi >= numSolid);
            bool isNonRenderedPortal = isPortal && (pi >= cell.numTextured);
            if (isNonRenderedPortal) { skipBspPhantom++; continue; }
            if (!isPortal && pi < cell.numTextured && cell.texturing[pi].txt == 249) {
                skipBackhack249++;
                continue;
            }
            if (!isPortal && pi < cell.numTextured) {
                uint8_t txtIdx = cell.texturing[pi].txt;
                if (txtIdx >= 247) { skipTxt247Plus++; continue; }
            }
            if (poly.count < 3) continue;
            std::vector<uint32_t> polyVerts(poly.count);
            for (int vi = 0; vi < poly.count; ++vi) {
                uint8_t idx = cell.polyIndices[pi][vi];
                const auto &v = cell.vertices[idx];
                polyVerts[vi] = getVertexIndex(v.x, v.y, v.z);
            }
            for (int t = 1; t < poly.count - 1; ++t) {
                fullScene.indices.push_back(static_cast<int32_t>(polyVerts[0]));
                fullScene.indices.push_back(static_cast<int32_t>(polyVerts[t + 1]));
                fullScene.indices.push_back(static_cast<int32_t>(polyVerts[t]));
            }
            if (isPortal) emittedPortal++;
            else emittedSolid++;
        }
    }

    const size_t numTris = fullScene.indices.size() / 3;
    const size_t numVerts = fullScene.vertices.size() / 3;
    std::fprintf(stdout,
        "[MESH_VALIDATE] mesh: verts=%zu tris=%zu cells=%u\n"
        "[MESH_VALIDATE] emitted: solid=%zu portal=%zu\n"
        "[MESH_VALIDATE] skipped: bsp_phantom=%zu backhack(txt=249)=%zu "
        "water_etc(txt>=247)=%zu\n",
        numVerts, numTris, wr.numCells,
        emittedSolid, emittedPortal,
        skipBspPhantom, skipBackhack249, skipTxt247Plus);

    // ── Half-edge boundary counter ──
    //
    // Key = (min(u,v), max(u,v)) — undirected edge identity. Value =
    // number of triangles that contain this edge.
    //   count == 1 → boundary edge (mesh hole — single ray slips here)
    //   count == 2 → manifold interior (the desired state for solid walls)
    //   count >= 3 → non-manifold (3+ triangles sharing one edge — usually
    //                a winding artefact or duplicate polygon)
    using EdgeKey = std::pair<uint32_t, uint32_t>;
    struct EdgeKeyHash {
        size_t operator()(const EdgeKey &e) const {
            uint64_t k = (static_cast<uint64_t>(e.first) << 32)
                       | static_cast<uint64_t>(e.second);
            return std::hash<uint64_t>{}(k);
        }
    };
    std::unordered_map<EdgeKey, int, EdgeKeyHash> edgeCount;
    edgeCount.reserve(numTris * 3);

    auto addEdge = [&](uint32_t u, uint32_t v) {
        if (u > v) std::swap(u, v);
        edgeCount[EdgeKey{u, v}]++;
    };
    for (size_t t = 0; t < numTris; ++t) {
        uint32_t a = static_cast<uint32_t>(fullScene.indices[3*t + 0]);
        uint32_t b = static_cast<uint32_t>(fullScene.indices[3*t + 1]);
        uint32_t c = static_cast<uint32_t>(fullScene.indices[3*t + 2]);
        addEdge(a, b);
        addEdge(b, c);
        addEdge(c, a);
    }

    auto getVert = [&](uint32_t idx) -> D::Vector3 {
        return D::Vector3{
            fullScene.vertices[3*idx + 0],
            fullScene.vertices[3*idx + 1],
            fullScene.vertices[3*idx + 2]};
    };

    size_t boundaryCount   = 0;
    size_t nonManifoldCount = 0;
    size_t boundaryInFilter = 0;

    // Collect boundary + non-manifold for output. Filter applies only to
    // the report file, not the summary count.
    std::vector<EdgeKey> boundaryEdges;
    std::vector<EdgeKey> nonManifoldEdges;
    boundaryEdges.reserve(edgeCount.size() / 32);
    for (const auto &kv : edgeCount) {
        if (kv.second == 1) {
            boundaryCount++;
            boundaryEdges.push_back(kv.first);
        } else if (kv.second >= 3) {
            nonManifoldCount++;
            nonManifoldEdges.push_back(kv.first);
        }
    }

    std::fprintf(stdout,
        "[MESH_VALIDATE] edges: total=%zu boundary=%zu non_manifold=%zu\n",
        edgeCount.size(), boundaryCount, nonManifoldCount);

    // ── Duplicate-triangle counter ──
    //
    // Key = sorted (v0, v1, v2) — the same vertex triple regardless of
    // winding order. Value = number of triangles using this triple.
    // Coincident-coplanar triangle pairs (a rendered portal emitted by
    // BOTH adjacent cells with opposite winding) show up as count=2
    // here. Higher counts indicate a more severe pile-up (e.g. a portal
    // polygon EMITTED plus an adjacent wall that got tessellated the
    // same way from both sides — count=4).
    //
    // Perf relevance: Steam Audio's reflection sim casts thousands of
    // rays per source per iteration and each ray-triangle intersection
    // test inside a BVH leaf is O(numTrisInLeaf). Coincident triangles
    // all fall into the same spatial cell → same leaf → each ray
    // through that leaf pays the intersection-test cost for every
    // duplicate. A mesh with 30% duplicate triangles wastes ~30% of the
    // narrow-phase work in the leaves those triangles occupy.
    struct TriKey {
        uint32_t a, b, c;  // ascending: a < b < c
        bool operator==(const TriKey &o) const {
            return a == o.a && b == o.b && c == o.c;
        }
    };
    struct TriKeyHash {
        size_t operator()(const TriKey &k) const {
            uint64_t h = k.a;
            h = h * 0x100000001b3ull + k.b;
            h = h * 0x100000001b3ull + k.c;
            return std::hash<uint64_t>{}(h);
        }
    };
    std::unordered_map<TriKey, int, TriKeyHash> triCount;
    triCount.reserve(numTris);
    for (size_t t = 0; t < numTris; ++t) {
        uint32_t v[3] = {
            static_cast<uint32_t>(fullScene.indices[3*t + 0]),
            static_cast<uint32_t>(fullScene.indices[3*t + 1]),
            static_cast<uint32_t>(fullScene.indices[3*t + 2]),
        };
        // Sort ascending.
        if (v[0] > v[1]) std::swap(v[0], v[1]);
        if (v[1] > v[2]) std::swap(v[1], v[2]);
        if (v[0] > v[1]) std::swap(v[0], v[1]);
        triCount[TriKey{v[0], v[1], v[2]}]++;
    }

    // Histogram: how many triples appear 1×, 2×, 3+×.
    size_t uniqueTris  = 0;  // total distinct vertex triples
    size_t dupPair     = 0;  // triples used by exactly 2 triangles
    size_t dupTriPlus  = 0;  // triples used by 3+ triangles
    size_t dupOverhead = 0;  // sum of (count-1) across dup entries = wasted tris
    int    maxCount    = 0;
    TriKey maxCountKey{0,0,0};
    for (const auto &kv : triCount) {
        uniqueTris++;
        if (kv.second == 2) {
            dupPair++;
            dupOverhead += 1;
        } else if (kv.second >= 3) {
            dupTriPlus++;
            dupOverhead += (kv.second - 1);
        }
        if (kv.second > maxCount) {
            maxCount = kv.second;
            maxCountKey = kv.first;
        }
    }
    const double dupPct = numTris > 0
        ? 100.0 * static_cast<double>(dupOverhead) / static_cast<double>(numTris)
        : 0.0;
    std::fprintf(stdout,
        "[MESH_VALIDATE] tris: unique=%zu dup_pairs=%zu dup_triple+=%zu "
        "wasted=%zu (%.1f%% of total) max_count=%d\n",
        uniqueTris, dupPair, dupTriPlus, dupOverhead, dupPct, maxCount);
    if (maxCount > 1) {
        D::Vector3 a = getVert(maxCountKey.a);
        D::Vector3 b = getVert(maxCountKey.b);
        D::Vector3 c = getVert(maxCountKey.c);
        D::Vector3 centroid{
            (a.x + b.x + c.x) / 3.0f,
            (a.y + b.y + c.y) / 3.0f,
            (a.z + b.z + c.z) / 3.0f};
        std::fprintf(stdout,
            "[MESH_VALIDATE] worst duplicate: count=%d at centroid=(%.2f,%.2f,%.2f)\n",
            maxCount, centroid.x, centroid.y, centroid.z);
    }

    const float filterR2 = filterRadiusFt * filterRadiusFt;
    if (filterPos) {
        for (const auto &e : boundaryEdges) {
            D::Vector3 p0 = getVert(e.first);
            D::Vector3 p1 = getVert(e.second);
            D::Vector3 mid{0.5f * (p0.x + p1.x),
                           0.5f * (p0.y + p1.y),
                           0.5f * (p0.z + p1.z)};
            D::Vector3 d{mid.x - filterPos->x,
                         mid.y - filterPos->y,
                         mid.z - filterPos->z};
            if (d.x*d.x + d.y*d.y + d.z*d.z <= filterR2) boundaryInFilter++;
        }
        std::fprintf(stdout,
            "[MESH_VALIDATE] boundary edges within filter: %zu\n",
            boundaryInFilter);
    }

    // ── OBJ dump (CWD, basename of mission) ──
    // Mission directories are often read-only (mounted ISO / disk image),
    // so anchor at the working directory instead of misPath's dir.
    auto basenameOf = [](const std::string &p) {
        auto slash = p.find_last_of("/\\");
        return slash == std::string::npos ? p : p.substr(slash + 1);
    };
    const std::string base = basenameOf(misPath);
    std::string objPath = base + ".acoustic.obj";
    std::FILE *obj = std::fopen(objPath.c_str(), "w");
    if (!obj) {
        std::fprintf(stderr,
            "[MESH_VALIDATE] failed to open %s for writing\n",
            objPath.c_str());
        return 1;
    }
    std::fprintf(obj,
        "# darknessHeadless mesh_validate\n"
        "# mission: %s\n"
        "# verts=%zu tris=%zu\n",
        misPath.c_str(), numVerts, numTris);
    for (size_t v = 0; v < numVerts; ++v) {
        std::fprintf(obj, "v %.4f %.4f %.4f\n",
            fullScene.vertices[3*v + 0],
            fullScene.vertices[3*v + 1],
            fullScene.vertices[3*v + 2]);
    }
    for (size_t t = 0; t < numTris; ++t) {
        std::fprintf(obj, "f %d %d %d\n",
            fullScene.indices[3*t + 0] + 1,
            fullScene.indices[3*t + 1] + 1,
            fullScene.indices[3*t + 2] + 1);
    }
    std::fclose(obj);
    std::fprintf(stdout, "[MESH_VALIDATE] wrote %s\n", objPath.c_str());

    // ── Boundary report ──
    std::string boundPath = base + ".acoustic.boundary.txt";
    std::FILE *bf = std::fopen(boundPath.c_str(), "w");
    if (!bf) {
        std::fprintf(stderr,
            "[MESH_VALIDATE] failed to open %s for writing\n",
            boundPath.c_str());
        return 1;
    }
    std::fprintf(bf, "# darknessHeadless mesh_validate boundary edges\n");
    std::fprintf(bf, "# mission: %s\n", misPath.c_str());
    std::fprintf(bf, "# boundary edges (count==1): %zu\n", boundaryCount);
    std::fprintf(bf, "# non-manifold edges (count>=3): %zu\n", nonManifoldCount);
    if (filterPos) {
        std::fprintf(bf,
            "# filter pos=(%.2f, %.2f, %.2f) radius=%.2fft\n"
            "# boundary edges within filter: %zu\n",
            filterPos->x, filterPos->y, filterPos->z,
            filterRadiusFt, boundaryInFilter);
    }
    std::fprintf(bf,
        "# format: <kind> mid=(x,y,z) p0=(x,y,z) p1=(x,y,z) v=<a,b>\n");
    for (const auto &e : boundaryEdges) {
        D::Vector3 p0 = getVert(e.first);
        D::Vector3 p1 = getVert(e.second);
        D::Vector3 mid{0.5f * (p0.x + p1.x),
                       0.5f * (p0.y + p1.y),
                       0.5f * (p0.z + p1.z)};
        if (filterPos) {
            D::Vector3 d{mid.x - filterPos->x,
                         mid.y - filterPos->y,
                         mid.z - filterPos->z};
            if (d.x*d.x + d.y*d.y + d.z*d.z > filterR2) continue;
        }
        std::fprintf(bf,
            "BOUNDARY mid=(%.3f,%.3f,%.3f) p0=(%.3f,%.3f,%.3f) "
            "p1=(%.3f,%.3f,%.3f) v=<%u,%u>\n",
            mid.x, mid.y, mid.z,
            p0.x, p0.y, p0.z,
            p1.x, p1.y, p1.z,
            e.first, e.second);
    }
    for (const auto &e : nonManifoldEdges) {
        D::Vector3 p0 = getVert(e.first);
        D::Vector3 p1 = getVert(e.second);
        D::Vector3 mid{0.5f * (p0.x + p1.x),
                       0.5f * (p0.y + p1.y),
                       0.5f * (p0.z + p1.z)};
        if (filterPos) {
            D::Vector3 d{mid.x - filterPos->x,
                         mid.y - filterPos->y,
                         mid.z - filterPos->z};
            if (d.x*d.x + d.y*d.y + d.z*d.z > filterR2) continue;
        }
        std::fprintf(bf,
            "NONMANIFOLD count=%d mid=(%.3f,%.3f,%.3f) "
            "p0=(%.3f,%.3f,%.3f) p1=(%.3f,%.3f,%.3f) v=<%u,%u>\n",
            edgeCount[e],
            mid.x, mid.y, mid.z,
            p0.x, p0.y, p0.z,
            p1.x, p1.y, p1.z,
            e.first, e.second);
    }
    std::fclose(bf);
    std::fprintf(stdout, "[MESH_VALIDATE] wrote %s\n", boundPath.c_str());
    std::fprintf(stdout, "[MESH_VALIDATE] done\n");
    return 0;
}

static int runProbePlanVerb(const std::string &misPath,
                             const std::string &resPath,
                             const std::string &schemasPath,
                             const std::string &scriptsDir,
                             const std::vector<std::pair<std::string,std::string>> &setOverrides)
{
    (void)resPath;       // currently unused — TXLIST lives in the .mis itself,
                         // not in fam.crf. Kept for forward-compat / consistency
                         // with darknessRender's flag set.
    (void)schemasPath;   // sound schemas not consulted by probe placement;
                         // accept the flag so users can paste the
                         // darknessRender invocation verbatim.

    // Banner — emit BEFORE any heavy work so the user sees the
    // commitment up-front. Repeated at end per feedback_no_silent_fallbacks.
    std::fprintf(stdout, "[PROBE_PLAN] DRY-RUN — no .probes file written\n");

    // probe_plan is a diagnostic verb — surface the audio-log lines that
    // narrate scene bounds, OBB derivation, dedup decisions, etc. so the
    // user can debug failures (e.g. "no probes generated" on weird-shaped
    // missions) without rebuilding with a different log gate.
    ::Darkness::gAudioLogVerbose = true;

    // Refuse to overwrite an existing .probes from this verb. The verb
    // never writes one (DRY-RUN banner), but if the user expected to
    // see a file, the banner below tells them where the existing one
    // lives so CI scripts can `diff` against the existing bake.
    {
        std::string existingProbe =
            Darkness::AudioService::getProbeFilePath(misPath);
        FILE *f = std::fopen(existingProbe.c_str(), "rb");
        if (f) {
            std::fclose(f);
            std::fprintf(stdout,
                "[PROBE_PLAN] existing .probes at '%s' (left untouched — "
                "this verb never writes)\n",
                existingProbe.c_str());
        }
    }

    initServices();
    loadSchema(scriptsDir);
    (void)GET_SERVICE(RoomService);
    loadDatabase(misPath);

    Darkness::AudioServicePtr audioSvc = GET_SERVICE(Darkness::AudioService);
    if (!audioSvc) {
        std::fprintf(stderr, "[PROBE_PLAN] AudioService not available\n");
        return 1;
    }

    // Apply --set overrides BEFORE acoustic-scene build + computeProbePlan
    // so swept knobs take effect. The dispatch table is intentionally
    // narrow — only knobs that affect probe placement. Unknown paths
    // get a [FALLBACK] (per feedback_no_silent_fallbacks) rather than
    // a silent no-op, so the sweep harness can't get away with reporting
    // identical counts for every iteration if it typos the path.
    for (const auto &kv : setOverrides) {
        const std::string &p = kv.first;
        const std::string &v = kv.second;
        try {
            if (p == "audio.pathing_probes.dedup_radius_ft") {
                audioSvc->setPathingDedupRadiusFt(std::stof(v));
                std::fprintf(stderr,
                    "[PROBE_PLAN] --set %s=%.3f applied\n",
                    p.c_str(), audioSvc->getPathingDedupRadiusFt());
            } else if (p == "audio.pathing_probes.enabled") {
                bool b = (v == "1" || v == "true" || v == "yes");
                audioSvc->setProbePathingEnabled(b);
                std::fprintf(stderr,
                    "[PROBE_PLAN] --set %s=%s applied\n",
                    p.c_str(), b ? "true" : "false");
            } else if (p == "audio.pathing_probes.density") {
                // Single string→enum mapping (pathingProbeDensityFromName;
                // "high" reserved for a future Tier 2). Reject-at-parse,
                // loudly, keeping the current value (default bends).
                const auto density = Darkness::pathingProbeDensityFromName(v);
                if (density != Darkness::PathingProbeDensity::Unknown) {
                    audioSvc->setPathingProbeDensity(density);
                    std::fprintf(stderr,
                        "[PROBE_PLAN] --set %s=%s applied\n", p.c_str(),
                        Darkness::pathingProbeDensityName(density));
                } else {
                    std::fprintf(stderr,
                        "[FALLBACK] --set %s: invalid value '%s' — valid: "
                        "'baseline' | 'bends' ('high' reserved, not yet "
                        "implemented). Keeping default '%s'.\n",
                        p.c_str(), v.c_str(),
                        Darkness::pathingProbeDensityName(
                            audioSvc->getPathingProbeDensity()));
                }
            } else if (p == "audio.probes.spacing") {
                audioSvc->setProbeSpacingFt(std::stof(v));
                std::fprintf(stderr, "[PROBE_PLAN] --set %s=%s applied\n", p.c_str(), v.c_str());
            } else if (p == "audio.probes.height") {
                audioSvc->setProbeHeightFt(std::stof(v));
                std::fprintf(stderr, "[PROBE_PLAN] --set %s=%s applied\n", p.c_str(), v.c_str());
            } else if (p == "audio.probes.min_wall_clearance_ft") {
                audioSvc->setProbeMinWallClearanceFt(std::stof(v));
                std::fprintf(stderr, "[PROBE_PLAN] --set %s=%s applied\n", p.c_str(), v.c_str());
            } else if (p == "audio.probes.elevation_sparsity_mul") {
                audioSvc->setProbeElevationSparsityMul(std::stof(v));
                std::fprintf(stderr, "[PROBE_PLAN] --set %s=%s applied\n", p.c_str(), v.c_str());
            } else if (p == "audio.probes.global_dedup_radius_ft") {
                audioSvc->setProbeGlobalDedupRadiusFt(std::stof(v));
                std::fprintf(stderr, "[PROBE_PLAN] --set %s=%s applied\n", p.c_str(), v.c_str());
            } else {
                std::fprintf(stderr,
                    "[FALLBACK] --set: unknown or probe_plan-irrelevant path "
                    "'%s' — ignored (only audio.pathing_probes.* and "
                    "audio.probes.* knobs affect probe placement)\n",
                    p.c_str());
            }
        } catch (const std::exception &e) {
            std::fprintf(stderr,
                "[FALLBACK] --set: failed to parse value '%s' for %s: %s\n",
                v.c_str(), p.c_str(), e.what());
        }
    }

    if (!buildAcousticSceneFromMissionFile(audioSvc, misPath)) {
        std::fprintf(stderr,
            "[PROBE_PLAN] failed to build acoustic scene from '%s'\n",
            misPath.c_str());
        return 1;
    }

    Darkness::ProbeBakePlan plan;
    if (!audioSvc->computeProbePlan(plan)) {
        std::fprintf(stderr,
            "[PROBE_PLAN] computeProbePlan failed\n");
        return 1;
    }

    // ── Output (machine-greppable, order locked by PLAN.PROBE_DEBUG_TOOLING) ──

    // Mission name (basename only — keeps the line stable across cwd
    // changes and disk-image mounts).
    std::string missionName = misPath;
    {
        size_t slash = missionName.find_last_of("/\\");
        if (slash != std::string::npos) missionName = missionName.substr(slash + 1);
    }
    std::fprintf(stdout, "[PROBE_PLAN] mission=%s\n", missionName.c_str());

    // Reflection counts. `globalDeduped` is the dropped count; the
    // total is the survivors (`reflectionPositions.size()`).
    int reflTotal = static_cast<int>(plan.reflectionPositions.size());
    // emitterKept is the count actually added by the emitter pass
    // (after the pass's own dedup against earlier passes). We can't
    // recover the floor/elev/emitter split from positions.size() alone
    // because the global dedup mixes them — report the per-pass
    // counts (pre-globalDedup) and the final globalDeduped drop.
    std::fprintf(stdout,
        "[PROBE_PLAN] reflections: floor=%d elevation=%d emitter=%d "
        "globalDeduped=%d total=%d\n",
        plan.floorKept, plan.elevationKept, plan.emitterKept,
        plan.globalDeduped, reflTotal);

    // Pathing per-purpose counts (post-dedup, pre-adaptive-radius).
    auto getPurpose = [&](Darkness::PathingProbePurpose p) -> int {
        auto it = plan.pathingPerPurpose.find(p);
        return (it != plan.pathingPerPurpose.end()) ? it->second : 0;
    };
    int postDedup = static_cast<int>(plan.pathingKept.size());
    std::fprintf(stdout,
        "[PROBE_PLAN] pathing [density=%s]: Portal=%d PortalPair=%d "
        "DoorPair=%d Centroid=%d Emitter=%d HubFill=%d  postDedup=%d\n",
        Darkness::pathingProbeDensityName(
            audioSvc->getPathingProbeDensity()),
        getPurpose(Darkness::PathingProbePurpose::Portal),
        getPurpose(Darkness::PathingProbePurpose::PortalPair),
        getPurpose(Darkness::PathingProbePurpose::DoorPair),
        getPurpose(Darkness::PathingProbePurpose::Centroid),
        getPurpose(Darkness::PathingProbePurpose::Emitter),
        getPurpose(Darkness::PathingProbePurpose::HubFill),
        postDedup);

    std::fprintf(stdout,
        "[PROBE_PLAN] pathing dedup_dropped: %d (centroids=%d "
        "doorPairs=%d portalPairs=%d other=%d)\n",
        plan.dedupDroppedTotal, plan.dedupDroppedCentroids,
        plan.dedupDroppedDoorPairs, plan.dedupDroppedPortalPairs,
        plan.dedupDroppedOther);

    // Per-room pathing-probe distribution.  Pathing runtime cost scales
    // with edges-visited per query; edge count scales with local probe
    // density.  Rooms with many probes (hub rooms with N portals + door
    // pairs) drive worst-case `[PATHING_SLOW]` cost; rooms with zero
    // probes (small closets that lost their centroid to dedup) drive
    // `eq=[0.1,0.1,0.1]` solver sentinels for in-room voices.
    {
        RoomServicePtr roomSvc = GET_SERVICE(RoomService);
        if (roomSvc && roomSvc->isLoaded()) {
            std::map<int, int> perRoomCount;  // roomID -> probe count
            int orphanProbes = 0;             // probes outside any room
            for (const auto &cand : plan.pathingKept) {
                ::Darkness::Room *r = roomSvc->roomFromPoint(cand.position);
                if (r) perRoomCount[r->getRoomID()]++;
                else   ++orphanProbes;
            }
            // Count rooms with zero probes (loaded via getRoomByID scan,
            // matches the ROOM_CENTROID estimate from earlier scheme cmp).
            int totalRooms      = 0;
            int roomsWithZero   = 0;
            for (int rid = 0; rid < 4096; ++rid) {
                if (!roomSvc->getRoomByID(rid)) continue;
                ++totalRooms;
                if (perRoomCount.find(rid) == perRoomCount.end()) {
                    ++roomsWithZero;
                }
            }
            // Histogram of probes-per-room (only over rooms with ≥1).
            std::vector<int> counts;
            counts.reserve(perRoomCount.size());
            for (auto &kv : perRoomCount) counts.push_back(kv.second);
            std::sort(counts.begin(), counts.end());
            auto pct = [&](float p) -> int {
                if (counts.empty()) return 0;
                size_t idx = std::min(counts.size() - 1,
                                      static_cast<size_t>(p * counts.size()));
                return counts[idx];
            };
            double mean = 0.0;
            for (int c : counts) mean += c;
            if (!counts.empty()) mean /= counts.size();
            std::fprintf(stdout,
                "[PROBE_PLAN] pathing per-room: total_rooms=%d "
                "rooms_with_probes=%zu rooms_with_zero=%d orphan_probes=%d  "
                "per-room counts: mean=%.1f min=%d p25=%d med=%d p75=%d "
                "p95=%d max=%d\n",
                totalRooms, perRoomCount.size(), roomsWithZero, orphanProbes,
                mean,
                counts.empty() ? 0 : counts.front(),
                pct(0.25f), pct(0.50f), pct(0.75f), pct(0.95f),
                counts.empty() ? 0 : counts.back());
        }
    }

    // Distance histogram: for each probe (across both batches) compute
    // distance to nearest OTHER probe, bucket. Lets the user spot
    // coverage holes (lots of >=20 ft probes) or wasteful clustering
    // (lots of <2 ft probes). O(N²) but only ~2000 probes × 2000 ≈ 4M
    // ops — milliseconds.
    {
        std::vector<Darkness::Vector3> all;
        all.reserve(plan.reflectionPositions.size() + plan.pathingKept.size());
        for (const auto &p : plan.reflectionPositions) all.push_back(p);
        for (const auto &k : plan.pathingKept)         all.push_back(k.position);

        int b2 = 0, b5 = 0, b10 = 0, b20 = 0, bMax = 0;
        for (size_t i = 0; i < all.size(); ++i) {
            float bestSq = std::numeric_limits<float>::max();
            for (size_t j = 0; j < all.size(); ++j) {
                if (i == j) continue;
                Darkness::Vector3 d = all[i] - all[j];
                float dsq = d.x*d.x + d.y*d.y + d.z*d.z;
                if (dsq < bestSq) bestSq = dsq;
            }
            float dist = std::sqrt(bestSq);
            if      (dist <  2.0f) ++b2;
            else if (dist <  5.0f) ++b5;
            else if (dist < 10.0f) ++b10;
            else if (dist < 20.0f) ++b20;
            else                    ++bMax;
        }
        std::fprintf(stdout,
            "[PROBE_PLAN] distance histogram (probe-to-nearest, ft): "
            "<2:%d  <5:%d  <10:%d  <20:%d  >=20:%d\n",
            b2, b5, b10, b20, bMax);
    }

    // ── Pathing-probe local-density metric ─────────────────────────────
    // The Steam Audio pathing solver runs Dijkstra-ish search through
    // the probe-to-probe visibility graph at runtime. Cost scales with
    // edges visited per query, which scales with each probe's local
    // degree (how many other probes it can connect to). Probe COUNT
    // alone is a weak proxy — a 500-probe level spread evenly is far
    // cheaper than a 200-probe level packed into one corner.
    //
    // We compute distance-only degree at a FIXED local radius (no
    // raycast / no Steam-Audio-derived visRange). Two reasons for the
    // fixed radius:
    //   1. Steam Audio's pathing-bake `visRange` is scene-AABB×1.5
    //      (`AudioService.cpp:11424`), which for most missions exceeds
    //      the mission's own extents — degree would saturate at N-1
    //      and lose discriminating power.
    //   2. headless doesn't currently set up the BSP raycaster (that's
    //      renderer-side), so we can't apply Steam Audio's
    //      visibility-graph occlusion here. A distance-only count is an
    //      UPPER BOUND on the real visibility degree, which is exactly
    //      what we want for "is this placement too clustered?" — if
    //      probes are bunched in 3D space, no occlusion check will save
    //      the solver from candidate-edge sprawl.
    //
    // 30 ft = ~9 m = room-scale. Smaller than typical inter-room
    // distances, larger than typical compound-doorway clusters. Picks
    // up over-packing without trivially saturating.
    {
        constexpr float kLocalDensityRadiusFt = 30.0f;
        const float r2 = kLocalDensityRadiusFt * kLocalDensityRadiusFt;
        const size_t N = plan.pathingKept.size();
        std::vector<int> degree(N, 0);
        // O(N²) — at N≤1500 this is ~2M ops, sub-millisecond.
        for (size_t i = 0; i < N; ++i) {
            const auto &pi = plan.pathingKept[i].position;
            for (size_t j = i + 1; j < N; ++j) {
                Darkness::Vector3 d = pi - plan.pathingKept[j].position;
                if (d.x*d.x + d.y*d.y + d.z*d.z < r2) {
                    ++degree[i];
                    ++degree[j];
                }
            }
        }
        long long sumDeg = 0, maxDeg = 0;
        long long edges = 0;
        std::vector<int> sortedDeg = degree;
        std::sort(sortedDeg.begin(), sortedDeg.end());
        for (int d : degree) { sumDeg += d; if (d > maxDeg) maxDeg = d; }
        edges = sumDeg / 2;
        auto pct = [&](double p) -> int {
            if (sortedDeg.empty()) return 0;
            size_t idx = std::min<size_t>(
                sortedDeg.size() - 1,
                static_cast<size_t>(p * sortedDeg.size()));
            return sortedDeg[idx];
        };
        double avg = N > 0 ? static_cast<double>(sumDeg) / static_cast<double>(N) : 0.0;
        std::fprintf(stdout,
            "[PROBE_PLAN] pathing density (within %.0f ft): "
            "avg=%.1f p50=%d p95=%d max=%lld edges=%lld\n",
            kLocalDensityRadiusFt, avg, pct(0.50), pct(0.95),
            maxDeg, edges);
    }

    std::fprintf(stdout, "[PROBE_PLAN] DRY-RUN — no .probes file written\n");
    return 0;
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
    std::cerr << "  probe_cell_audit [positions.csv]" << std::endl;
    std::cerr << "                    Do probe positions land inside the WR cell network that" << std::endl;
    std::cerr << "                    raycastWorld traverses? A position outside every cell makes" << std::endl;
    std::cerr << "                    raycastWorld bail before tracing any geometry and report" << std::endl;
    std::cerr << "                    'no hit' — i.e. a clear line of sight through solid walls." << std::endl;
    std::cerr << "                    With a .probes.*.positions.csv sidecar: audits the REAL baked" << std::endl;
    std::cerr << "                    probes. Without: audits ROOM_DB room/portal centers (no bake" << std::endl;
    std::cerr << "                    needed, so it runs on any level)." << std::endl;
    std::cerr << "  room_graph        Dump the whole ROOM_DB portal graph as machine-readable" << std::endl;
    std::cerr << "                    ROOM / PORTAL / PDIST lines (room centers, portal centers," << std::endl;
    std::cerr << "                    degrees, intra-room portal-to-portal distances). Consumed" << std::endl;
    std::cerr << "                    by analysis/room_graph_stats.py." << std::endl;
    std::cerr << "  wr_portals        Dump every WR CELL portal polygon as machine-readable" << std::endl;
    std::cerr << "                    WRPORTAL lines (cell, target cell, centroid, inradius," << std::endl;
    std::cerr << "                    circumradius, vertex count, area). Unlike room_graph's" << std::endl;
    std::cerr << "                    ROOM_DB portals — designer box boundaries — these are" << std::endl;
    std::cerr << "                    compiled from the real geometry, so an aperture's size" << std::endl;
    std::cerr << "                    is the opening's true size. Needs no services." << std::endl;
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
    std::cerr << "  sound_db [outpath] Walk --schemas dir and emit a CSV of every schema's" << std::endl;
    std::cerr << "                    name, gain, attenuation factor, max audible distance," << std::endl;
    std::cerr << "                    audio class, loop state, sample count, etc." << std::endl;
    std::cerr << "                    Mission-less verb — invoke as: darknessHeadless sound_db" << std::endl;
    std::cerr << "                    [outpath] --schemas <DIR>" << std::endl;
    std::cerr << "  ambients          List every P$AmbientHa emitter with schema + position." << std::endl;
    std::cerr << "  probe_plan        Dry-run the audio probe placement passes for this mission" << std::endl;
    std::cerr << "                    and print per-purpose probe counts WITHOUT invoking the" << std::endl;
    std::cerr << "                    Steam Audio bake. Useful for iterating on probe placement" << std::endl;
    std::cerr << "                    reductions. Accepts --res / --schemas (same as darknessRender);" << std::endl;
    std::cerr << "                    no .probes file is ever written." << std::endl;
    std::cerr << "  txlist_audit      Enumerate every TXLIST texture name and report which ones" << std::endl;
    std::cerr << "                    have no acoustic-material keyword (resolve to 'generic' and" << std::endl;
    std::cerr << "                    fall through to the opaque-reflective fallback at runtime)." << std::endl;
    std::cerr << "                    Aggregate across missions: grep the [UNMAPPED] lines. No" << std::endl;
    std::cerr << "                    services or Steam Audio runtime required." << std::endl;
    std::cerr << "  mesh_validate     Validate watertightness of the acoustic mesh built from this" << std::endl;
    std::cerr << "                    mission. Writes <mis>.acoustic.obj and" << std::endl;
    std::cerr << "                    <mis>.acoustic.boundary.txt. Optional `--at X Y Z R` filter" << std::endl;
    std::cerr << "                    reports only boundary/non-manifold edges within R feet of" << std::endl;
    std::cerr << "                    (X, Y, Z). No services or Steam Audio runtime required." << std::endl;
    std::cerr << std::endl;
    std::cerr << "Options:" << std::endl;
    std::cerr << "  --scripts <path>  Path to schema scripts directory (default: scripts/thief2)" << std::endl;
    std::cerr << "  --res <path>      Thief 2 RES directory (snd.crf, fam.crf, obj.crf)." << std::endl;
    std::cerr << "                    Currently unused by probe_plan but accepted for compatibility" << std::endl;
    std::cerr << "                    with darknessRender invocations." << std::endl;
    std::cerr << "  --schemas <path>  Sound schemas directory. Same compat note as --res." << std::endl;
}

// ---------- Main ----------

int main(int argc, char *argv[]) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    // Parse --scripts / --res / --schemas options from anywhere in argv.
    // --res and --schemas are accepted (and currently ignored by every
    // verb except probe_plan) so users can paste a darknessRender
    // invocation verbatim without arg surgery.
    std::string scriptsDir = "scripts/thief2";
    std::string resPath;
    std::string schemasPath;
    // --set <yaml.path>=<value> overrides applied after AudioService init,
    // before any verb that consumes the relevant config. Currently only
    // probe_plan reads them; the dispatch table below lists supported
    // paths explicitly so unknown paths emit [FALLBACK] per
    // feedback_no_silent_fallbacks.
    std::vector<std::pair<std::string, std::string>> setOverrides;
    std::vector<std::string> positionalArgs;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--scripts" && i + 1 < argc) {
            scriptsDir = argv[++i];
        } else if (a == "--res" && i + 1 < argc) {
            resPath = argv[++i];
        } else if (a == "--schemas" && i + 1 < argc) {
            schemasPath = argv[++i];
        } else if (a == "--set" && i + 1 < argc) {
            std::string kv = argv[++i];
            auto eq = kv.find('=');
            if (eq == std::string::npos) {
                std::fprintf(stderr,
                    "[FALLBACK] --set: missing '=' in '%s' — expected --set "
                    "<yaml.path>=<value>; ignored\n", kv.c_str());
            } else {
                setOverrides.emplace_back(kv.substr(0, eq), kv.substr(eq + 1));
            }
        } else {
            positionalArgs.push_back(a);
        }
    }

    if (positionalArgs.empty()) {
        printUsage(argv[0]);
        return 1;
    }

    // sound_db can run mission-less (text-only) or with a mission/dark.gam
    // to apply P$SchAttFac + P$SchPlayPa overrides:
    //   darknessHeadless sound_db [outpath] --schemas DIR     → text-only
    //   darknessHeadless <mission|gam> sound_db [outpath]     → with overlays
    std::string dbFile;
    std::string command;
    if (positionalArgs[0] == "sound_db") {
        command = positionalArgs[0];
        // dbFile stays empty — text-only dump.
    } else {
        dbFile  = positionalArgs[0];
        command = (positionalArgs.size() >= 2) ? positionalArgs[1] : "info";
    }

    // Set up logging - suppress for Level 1, allow for Level 2
    Logger logger;
    StdLog stdlog;

    bool needsServices = (command == "objects" ||
                          command == "properties" ||
                          command == "links" ||
                          command == "trace-path" ||
                          command == "room-info" ||
                          command == "room_graph" ||
                          command == "probe_cell_audit" ||
                          command == "ambients" ||
                          command == "prop-dump" ||
                          command == "sound-desc" ||
                          command == "probe_plan");

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
        } else if (command == "room_graph") {
            initServices();
            loadSchema(scriptsDir);
            // Same rationale as trace-path: force RoomService instantiation
            // BEFORE loadDatabase so it registers as a DatabaseListener and
            // actually parses ROOM_DB.
            (void)GET_SERVICE(RoomService);
            loadDatabase(dbFile);
            printRoomGraph();
        } else if (command == "probe_cell_audit") {
            initServices();
            loadSchema(scriptsDir);
            // Same rationale as room_graph: force RoomService instantiation
            // BEFORE loadDatabase so ROOM_DB is actually parsed. (Only the
            // no-CSV candidate mode needs it, but the cost is trivial.)
            (void)GET_SERVICE(RoomService);
            loadDatabase(dbFile);
            printProbeCellAudit(dbFile,
                positionalArgs.size() > 2 ? positionalArgs[2] : std::string());
        } else if (command == "wr_portals") {
            // Service-less by design: parseWRChunk reads the .mis directly and
            // WR cells carry their own geometry — no ROOM_DB involved. That is
            // the whole point of the verb (see printWRPortals).
            printWRPortals(dbFile);
        } else if (command == "sound_db") {
            // sound_db: walks the schema directory; optionally applies
            // P$SchAttFac/SchPlayPa overrides from dbFile when present.
            //   • text-only invocation:  positionalArgs = [sound_db, outpath?]
            //   • with overlays:         positionalArgs = [dbFile, sound_db, outpath?]
            std::string outPath;
            if (dbFile.empty()) {
                if (positionalArgs.size() >= 2) outPath = positionalArgs[1];
            } else {
                if (positionalArgs.size() >= 3) outPath = positionalArgs[2];
            }
            return runSoundDbVerb(schemasPath, outPath, dbFile, scriptsDir);
        } else if (command == "probe_plan") {
            // probe_plan handles its own initServices / loadSchema /
            // ── --set overrides flow here (only probe_plan reads them today)
            // loadDatabase since it needs to interleave them with
            // acoustic-scene construction before the dry-run plan
            // computation. Returns 0 on success, 1 on failure.
            return runProbePlanVerb(dbFile, resPath, schemasPath, scriptsDir,
                                     setOverrides);
        } else if (command == "txlist_audit") {
            // txlist_audit: enumerate TXLIST texture names and report which
            // ones have no acoustic-material keyword. No services / no IPL.
            return runTxlistAuditVerb(dbFile);
        } else if (command == "mesh_validate") {
            // mesh_validate: WR-only static-mesh watertightness check.
            // Optional `--at X Y Z R` positional filter for zooming in
            // on a known leak. No services / no database / no IPL
            // required — purely structural.
            //
            // Filter parsing: scan positionalArgs for "--at"; the next
            // four entries are X, Y, Z, R as floats. Tolerate the flag
            // appearing anywhere after the mission path.
            const Darkness::Vector3 *filterPosPtr = nullptr;
            Darkness::Vector3 filterPos{0.0f, 0.0f, 0.0f};
            float filterRadiusFt = 0.0f;
            for (size_t i = 2; i < positionalArgs.size(); ++i) {
                if (positionalArgs[i] == "--at"
                    && i + 4 < positionalArgs.size()) {
                    try {
                        filterPos.x = std::stof(positionalArgs[i + 1]);
                        filterPos.y = std::stof(positionalArgs[i + 2]);
                        filterPos.z = std::stof(positionalArgs[i + 3]);
                        filterRadiusFt = std::stof(positionalArgs[i + 4]);
                        if (filterRadiusFt > 0.0f) filterPosPtr = &filterPos;
                    } catch (const std::exception &e) {
                        std::fprintf(stderr,
                            "mesh_validate: --at parse failed: %s\n",
                            e.what());
                        return 1;
                    }
                    break;
                }
            }
            return runMeshValidateVerb(dbFile, filterPosPtr, filterRadiusFt);
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
