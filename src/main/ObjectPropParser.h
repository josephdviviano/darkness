/******************************************************************************
 *
 *    This file is part of the darkness project
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

// ObjectPropParser — reads P$ModelName and P$Position chunks from Dark Engine
// .mis and .gam files, resolving archetype inheritance via MetaProp links to
// build a list of concrete objects and their world placements.
//
// Most objects in a mission file inherit their model name from their archetype
// defined in the .gam file. The inheritance chain is resolved via L$MetaProp
// links: each concrete object (positive ID) links to an archetype (negative
// ID), which may chain further to parent archetypes.
//
// Same standalone/header-only pattern as SpawnFinder.h.

#pragma once

#include "File.h"
#include "FileGroup.h"
#include "FileCompat.h"

#include <array>
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <algorithm>

namespace Darkness {

// Per-object placement data merged from P$ModelName + P$Position chunks
struct ObjectPlacement {
    int32_t objID;
    char modelName[16];  // from P$ModelName (fixed 16-byte string)
    float x, y, z;       // world position from P$Position
    int16_t heading;     // binary radians (65536 = 360 degrees)
    int16_t pitch;
    int16_t bank;
    float scaleX, scaleY, scaleZ;  // from P$Scale (default 1,1,1)
    bool hasPosition;    // false if object had model name but no position
};

// Aggregated result of parsing object properties
struct ObjectPropData {
    std::vector<ObjectPlacement> objects;       // concrete objects with models
    std::vector<std::string> uniqueModels;      // deduplicated model names
};

namespace detail {

// Read all P$ModelName records from a file group into a map.
// Records: {uint32_t objID, uint32_t dataSize=16, char name[16]}
// Accepts both positive (concrete) and negative (archetype) IDs.
inline void readModelNames(FileGroupPtr &db,
                            std::unordered_map<int32_t, std::string> &modelMap) {
    // "P$ModelName" is exactly 11 chars — fits in the 12-byte chunk name field
    const char *modelChunk = "P$ModelName";

    if (!db->hasFile(modelChunk)) return;

    FilePtr modelFile = db->getFile(modelChunk);

    while (static_cast<size_t>(modelFile->tell()) + 8 <= modelFile->size()) {
        uint32_t objID, dataSize;
        *modelFile >> objID >> dataSize;

        if (dataSize == 0) continue;

        // Read model name (fixed 16 bytes, null-terminated)
        char name[16] = {};
        size_t toRead = std::min(static_cast<size_t>(dataSize), sizeof(name));
        modelFile->read(name, toRead);
        name[15] = '\0';

        // Skip any remaining bytes if dataSize > 16
        if (dataSize > toRead) {
            modelFile->seek(static_cast<file_offset_t>(dataSize - toRead),
                            File::FSEEK_CUR);
        }

        if (name[0] != '\0') {
            // Lowercase for consistent CRF lookups
            std::string nameStr(name);
            std::transform(nameStr.begin(), nameStr.end(),
                           nameStr.begin(), ::tolower);
            int32_t signedID = static_cast<int32_t>(objID);
            modelMap[signedID] = nameStr;
        }
    }
}

// Read MetaProp links from a file group. Each link says "src inherits from dest".
// Link records: {uint32_t id, int32_t src, int32_t dest, uint16_t flavor} = 14 bytes
inline void readMetaPropLinks(FileGroupPtr &db,
                               std::unordered_map<int32_t, int32_t> &parentMap) {
    const char *linkChunk = "L$MetaProp";

    if (!db->hasFile(linkChunk)) return;

    FilePtr linkFile = db->getFile(linkChunk);
    const int LINK_SIZE = 14;

    while (static_cast<size_t>(linkFile->tell()) + LINK_SIZE <=
           linkFile->size()) {
        uint32_t id;
        int32_t src, dest;
        uint16_t flavor;
        *linkFile >> id >> src >> dest >> flavor;

        // Store the first (primary) parent link for each object.
        // If an object has multiple MetaProp links, the first one is the
        // primary archetype parent.
        if (parentMap.find(src) == parentMap.end()) {
            parentMap[src] = dest;
        }
    }
}

// Read all P$RenderTyp records from a file group into a map.
// Records: {uint32_t objID, uint32_t dataSize=4, uint32_t renderType}
// RenderType values: 0=Normal, 1=NotRendered, 2=NoLightmap, 3=EditorOnly
inline void readRenderTypes(FileGroupPtr &db,
                             std::unordered_map<int32_t, uint32_t> &renderMap) {
    const char *rtChunk = "P$RenderTyp";

    if (!db->hasFile(rtChunk)) return;

    FilePtr rtFile = db->getFile(rtChunk);

    while (static_cast<size_t>(rtFile->tell()) + 8 <= rtFile->size()) {
        uint32_t objID, dataSize;
        *rtFile >> objID >> dataSize;

        if (dataSize >= 4) {
            uint32_t renderType;
            *rtFile >> renderType;

            // Skip remaining bytes if dataSize > 4
            if (dataSize > 4) {
                rtFile->seek(static_cast<file_offset_t>(dataSize - 4),
                             File::FSEEK_CUR);
            }

            int32_t signedID = static_cast<int32_t>(objID);
            renderMap[signedID] = renderType;
        } else if (dataSize > 0) {
            rtFile->seek(static_cast<file_offset_t>(dataSize),
                         File::FSEEK_CUR);
        }
    }
}

// Walk the MetaProp inheritance chain to resolve an object's render type.
// Returns 0 (Normal) if no explicit RenderType is found in the chain.
inline uint32_t resolveRenderType(
    int32_t objID,
    const std::unordered_map<int32_t, uint32_t> &renderMap,
    const std::unordered_map<int32_t, int32_t> &parentMap)
{
    // Check direct assignment first
    auto it = renderMap.find(objID);
    if (it != renderMap.end()) return it->second;

    // Walk the inheritance chain (max depth to prevent infinite loops)
    int32_t current = objID;
    for (int depth = 0; depth < 32; ++depth) {
        auto pit = parentMap.find(current);
        if (pit == parentMap.end()) break;

        current = pit->second;
        auto rit = renderMap.find(current);
        if (rit != renderMap.end()) return rit->second;
    }

    // Default: Normal (renderable)
    return 0;
}

// Read the BRLIST chunk to find object IDs associated with editor brushes.
// BRLIST records are 76 bytes base (pack(2)), with variable face data for terrain.
// The 'primal' field (int32 at offset 4) is the brush's associated object ID.
// The 'type' field (int8 at offset 10) identifies the brush kind:
//   0=SOLID(terrain), 1=AIR, 2=WATER, -1(0xFF)=LIGHT, -4(0xFC)=FLOW, -5(0xFB)=ROOM
// All brush objects are editor constructs — not rendered as standalone objects.
// (Terrain brushes are already baked into the worldmesh WR chunk.)
inline void readBrushObjectIDs(FileGroupPtr &db,
                                std::unordered_set<int32_t> &brushObjIDs) {
    const char *brChunk = "BRLIST";
    if (!db->hasFile(brChunk)) return;

    FilePtr file = db->getFile(brChunk);
    const size_t baseSize = 76;  // fixed part of each brush record (pack(2))

    int count = 0;
    while (static_cast<size_t>(file->tell()) + baseSize <= file->size()) {
        // Read the fixed 76-byte brush header
        uint16_t id, time;
        int32_t primal;
        int16_t base;
        int8_t type;

        *file >> id >> time;
        *file >> primal;
        *file >> base >> type;

        // Skip remaining fixed fields (bytes 11..75 = 65 bytes)
        // offset 11: 1 byte padding + 12 float pos + 12 float size + 6 int16 rot
        //            + 2 int16 cur_face + 4 float snap + 18 unknown + 1 snap_grid
        //            + 1 num_faces + 1 edge + 1 vertex + 1 flags + 1 group + 4 unknown
        uint8_t padding;
        *file >> padding;  // alignment byte at offset 11

        float posX, posY, posZ, sizeX, sizeY, sizeZ;
        *file >> posX >> posY >> posZ >> sizeX >> sizeY >> sizeZ;

        int16_t rotX, rotY, rotZ, curFace;
        *file >> rotX >> rotY >> rotZ >> curFace;

        float snapSize;
        *file >> snapSize;

        // Skip 18 unknown bytes
        file->seek(static_cast<file_offset_t>(18), File::FSEEK_CUR);

        uint8_t snapGrid, numFaces, edge, vertex, flags, group;
        *file >> snapGrid >> numFaces >> edge >> vertex >> flags >> group;

        uint32_t unknown5;
        *file >> unknown5;

        // For terrain brushes (type 0), skip the per-face texture data
        if (type == 0 && numFaces > 0) {
            file->seek(static_cast<file_offset_t>(numFaces * 10),
                       File::FSEEK_CUR);
        }

        // The primal field is the object ID associated with this brush
        if (primal > 0) {
            brushObjIDs.insert(primal);
        }
        ++count;
    }

    if (count > 0) {
        std::fprintf(stderr, "BRLIST: %d brushes, %zu unique object IDs\n",
                     count, brushObjIDs.size());
    }
}

// Read all P$Scale records from a file group into a map.
// Records: {uint32_t objID, uint32_t dataSize=12, float sx, float sy, float sz}
// P$Scale stores the ModelScale as a Vector3.
inline void readScales(FileGroupPtr &db,
                        std::unordered_map<int32_t, std::array<float,3>> &scaleMap) {
    const char *scaleChunk = "P$Scale";

    if (!db->hasFile(scaleChunk)) return;

    FilePtr sf = db->getFile(scaleChunk);

    while (static_cast<size_t>(sf->tell()) + 8 <= sf->size()) {
        uint32_t objID, dataSize;
        *sf >> objID >> dataSize;

        if (dataSize >= 12) {
            float sx, sy, sz;
            *sf >> sx >> sy >> sz;

            // Skip remaining bytes if dataSize > 12
            if (dataSize > 12) {
                sf->seek(static_cast<file_offset_t>(dataSize - 12),
                         File::FSEEK_CUR);
            }

            int32_t signedID = static_cast<int32_t>(objID);
            scaleMap[signedID] = {sx, sy, sz};
        } else if (dataSize > 0) {
            sf->seek(static_cast<file_offset_t>(dataSize),
                     File::FSEEK_CUR);
        }
    }
}

// Look up an object's scale directly (no inheritance).
// Dark Engine's P$Scale property is created with kPropertyNoInherit
// (see SCALPROP.CPP), so archetype scales are NOT inherited by concrete
// objects. Archetype P$Scale values are often physics bounding box dimensions,
// not visual model scales — inheriting them causes massive distortion.
// Returns {1,1,1} if the object has no direct P$Scale record.
inline std::array<float,3> resolveScale(
    int32_t objID,
    const std::unordered_map<int32_t, std::array<float,3>> &scaleMap)
{
    auto it = scaleMap.find(objID);
    if (it != scaleMap.end()) return it->second;

    return {1.0f, 1.0f, 1.0f};
}

// Walk the MetaProp inheritance chain to find the model name for an object.
// Returns empty string if no model found in the chain.
inline std::string resolveModelName(
    int32_t objID,
    const std::unordered_map<int32_t, std::string> &modelMap,
    const std::unordered_map<int32_t, int32_t> &parentMap)
{
    // Check direct assignment first
    auto it = modelMap.find(objID);
    if (it != modelMap.end()) return it->second;

    // Walk the inheritance chain (max depth to prevent infinite loops)
    int32_t current = objID;
    for (int depth = 0; depth < 32; ++depth) {
        auto pit = parentMap.find(current);
        if (pit == parentMap.end()) break;

        current = pit->second;
        auto mit = modelMap.find(current);
        if (mit != modelMap.end()) return mit->second;
    }

    return {};
}

// Read the GAM_FILE path from a .mis file group
inline std::string readGamFilePath(FileGroupPtr &db) {
    if (!db->hasFile("GAM_FILE")) return {};

    FilePtr gf = db->getFile("GAM_FILE");
    size_t sz = gf->size();
    std::string path(sz, '\0');
    gf->read(&path[0], sz);
    // Trim trailing nulls
    while (!path.empty() && path.back() == '\0')
        path.pop_back();
    return path;
}

} // namespace detail

// Parse object placement data from a .mis file.
//
// Algorithm:
// 1. Open the .mis file, read GAM_FILE path, open the .gam file
// 2. Read P$ModelName from both .gam (archetypes) and .mis (overrides)
// 3. Read L$MetaProp links from both files (inheritance chain)
// 4. Read P$Position from .mis (concrete object positions)
// 5. For each concrete object with a position, resolve its model name
//    by walking the MetaProp inheritance chain
// 6. Collect unique model names for batch loading
//
// gamPath: explicit path to .gam file, or empty to auto-detect from GAM_FILE chunk
inline ObjectPropData parseObjectProps(const std::string &misPath,
                                        const std::string &gamPath = {}) {
    ObjectPropData result;

    // Combined maps from both .gam and .mis
    std::unordered_map<int32_t, std::string> modelMap;
    std::unordered_map<int32_t, int32_t> parentMap;
    std::unordered_map<int32_t, uint32_t> renderTypeMap;
    std::unordered_map<int32_t, std::array<float,3>> scaleMap;
    std::unordered_set<int32_t> brushObjIDs;  // object IDs from BRLIST (editor brushes)

    // ── Load .gam file (archetype definitions) ──
    FilePtr misFp(new StdFile(misPath, File::FILE_R));
    FileGroupPtr misDb(new DarkFileGroup(misFp));

    // Determine .gam path: use explicit path, or read from GAM_FILE chunk
    std::string resolvedGamPath = gamPath;
    if (resolvedGamPath.empty()) {
        std::string gamRef = detail::readGamFilePath(misDb);
        if (!gamRef.empty()) {
            // GAM_FILE path is relative to the .mis file's directory
            // (or sometimes just a filename like "dark.gam")
            std::string misDir;
            size_t lastSlash = misPath.find_last_of("/\\");
            if (lastSlash != std::string::npos) {
                misDir = misPath.substr(0, lastSlash + 1);
            }
            resolvedGamPath = misDir + gamRef;
            std::fprintf(stderr, "ObjectPropParser: GAM_FILE -> %s\n",
                         resolvedGamPath.c_str());
        }
    }

    if (!resolvedGamPath.empty()) {
        try {
            FilePtr gamFp(new StdFile(resolvedGamPath, File::FILE_R));
            FileGroupPtr gamDb(new DarkFileGroup(gamFp));

            detail::readModelNames(gamDb, modelMap);
            detail::readMetaPropLinks(gamDb, parentMap);
            detail::readRenderTypes(gamDb, renderTypeMap);
            detail::readScales(gamDb, scaleMap);

            std::fprintf(stderr, "ObjectPropParser: loaded %zu archetype models, "
                         "%zu parent links, %zu render types, %zu scales from .gam\n",
                         modelMap.size(), parentMap.size(), renderTypeMap.size(),
                         scaleMap.size());
        } catch (const std::exception &e) {
            std::fprintf(stderr, "ObjectPropParser: failed to load .gam (%s): %s\n",
                         resolvedGamPath.c_str(), e.what());
        }
    }

    // ── Load .mis file (concrete objects + overrides) ──
    size_t prevModels = modelMap.size();
    size_t prevLinks = parentMap.size();

    detail::readModelNames(misDb, modelMap);
    detail::readMetaPropLinks(misDb, parentMap);
    size_t prevRT = renderTypeMap.size();
    detail::readRenderTypes(misDb, renderTypeMap);
    size_t prevScales = scaleMap.size();
    detail::readScales(misDb, scaleMap);

    // Read BRLIST to identify editor brush objects (room/terrain/flow brushes)
    // that should not be rendered as standalone objects
    detail::readBrushObjectIDs(misDb, brushObjIDs);

    std::fprintf(stderr, "ObjectPropParser: +%zu models, +%zu links, +%zu render types, "
                 "+%zu scales from .mis\n",
                 modelMap.size() - prevModels, parentMap.size() - prevLinks,
                 renderTypeMap.size() - prevRT, scaleMap.size() - prevScales);

    // ── Read positions from P$Position ──
    const char *posChunk = "P$Position";

    std::unordered_map<int32_t, ObjectPlacement> posMap;

    if (misDb->hasFile(posChunk)) {
        FilePtr posFile = misDb->getFile(posChunk);

        while (static_cast<size_t>(posFile->tell()) + 8 <= posFile->size()) {
            uint32_t objID, dataSize;
            *posFile >> objID >> dataSize;

            int32_t signedID = static_cast<int32_t>(objID);

            // On-disk P$Position: float[3] pos (12) + int32_t cell (4)
            // + int16_t bank/pitch/heading (6) = 22 bytes (angvec tx,ty,tz)
            if (signedID > 0 && dataSize >= 22) {
                float px, py, pz;
                *posFile >> px >> py >> pz;

                int32_t cell;
                *posFile >> cell;

                // On-disk order: bank, pitch, heading (= angvec tx, ty, tz)
                // DarkDBPropPosition.facing is SCoord {x,y,z} = {tx,ty,tz}
                // where tx=bank(X-rot), ty=pitch(Y-rot), tz=heading(Z-rot)
                int16_t bank, pitch, heading;
                *posFile >> bank >> pitch >> heading;

                if (dataSize > 22) {
                    posFile->seek(static_cast<file_offset_t>(dataSize - 22),
                                  File::FSEEK_CUR);
                }

                ObjectPlacement p = {};
                p.objID = signedID;
                p.x = px; p.y = py; p.z = pz;
                p.heading = heading;
                p.pitch = pitch;
                p.bank = bank;
                p.hasPosition = true;
                posMap[signedID] = p;
            } else {
                if (dataSize > 0) {
                    posFile->seek(static_cast<file_offset_t>(dataSize),
                                  File::FSEEK_CUR);
                }
            }
        }
    } else {
        std::fprintf(stderr, "ObjectPropParser: no %s chunk found\n", posChunk);
    }

    std::fprintf(stderr, "ObjectPropParser: %zu concrete objects with positions\n",
                 posMap.size());

    // ── Merge: resolve model names via inheritance for each positioned object ──
    std::unordered_set<std::string> uniqueSet;
    int resolved = 0, unresolved = 0, filtered = 0;

    for (auto &kv : posMap) {
        int32_t id = kv.first;
        const ObjectPlacement &pos = kv.second;

        // Check render type — skip non-renderable objects (lights, markers, etc.)
        // 0=Normal, 1=NotRendered, 2=NoLightmap, 3=EditorOnly
        uint32_t rt = detail::resolveRenderType(id, renderTypeMap, parentMap);
        if (rt == 1 || rt == 3) {
            ++filtered;
            continue;
        }

        // Skip editor brush objects (from BRLIST) — room brushes, terrain brushes,
        // flow brushes, etc. are architectural geometry already baked into the
        // worldmesh or used for room/sound definitions, not standalone objects
        if (brushObjIDs.count(id)) {
            ++filtered;
            continue;
        }

        // Resolve model name through inheritance chain
        std::string name = detail::resolveModelName(id, modelMap, parentMap);

        if (name.empty()) {
            ++unresolved;
            continue;
        }

        ObjectPlacement placement = pos;
        std::strncpy(placement.modelName, name.c_str(),
                     sizeof(placement.modelName) - 1);
        placement.modelName[15] = '\0';

        // Look up scale directly on this object (P$Scale does not inherit)
        auto scale = detail::resolveScale(id, scaleMap);
        placement.scaleX = scale[0];
        placement.scaleY = scale[1];
        placement.scaleZ = scale[2];

        result.objects.push_back(placement);
        uniqueSet.insert(name);
        ++resolved;
    }

    // Build sorted unique model list
    result.uniqueModels.assign(uniqueSet.begin(), uniqueSet.end());
    std::sort(result.uniqueModels.begin(), result.uniqueModels.end());

    std::fprintf(stderr, "ObjectPropParser: %d resolved, %d unresolved, "
                 "%d filtered (non-renderable) (%zu unique models)\n",
                 resolved, unresolved, filtered, result.uniqueModels.size());

    return result;
}

} // namespace Darkness
