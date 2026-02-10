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

// ObjectPropParser — builds a list of concrete objects with world placements
// for rendering. Uses PropertyService for property resolution (ModelName,
// RenderType, Scale, RenderAlpha) with proper inheritance via the Inheritor
// system. P$Position is read directly from the .mis file because the built-in
// PositionPropertyStorage uses a different in-memory format (Quaternion vs
// raw int16 angles needed by the renderer's rotation matrix builder).
//
// BRLIST is also read directly — it's a non-property chunk identifying
// editor brush objects that should not be rendered.

#pragma once

#include "File.h"
#include "FileGroup.h"
#include "FileCompat.h"
#include "property/PropertyService.h"
#include "property/TypedProperty.h"
#include "property/DarkPropertyDefs.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <unordered_set>
#include <unordered_map>
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
    float renderAlpha;   // from P$RenderAlp (0.0=invisible, 1.0=opaque, default 1.0)
    bool hasPosition;    // false if object had model name but no position
};

// Aggregated result of parsing object properties
struct ObjectPropData {
    std::vector<ObjectPlacement> objects;       // concrete objects with models
    std::vector<std::string> uniqueModels;      // deduplicated model names
};

namespace detail {

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

} // namespace detail

// Parse object placement data from a .mis file using PropertyService for
// property resolution (ModelName, RenderType, Scale, RenderAlpha) with
// proper inheritance via the Inheritor system.
//
// PropertyService must have all properties loaded (GameService::load()
// handles the recursive .gam + .mis database loading).
//
// P$Position is still read directly from the .mis file because the built-in
// PositionPropertyStorage stores a different in-memory format (32-byte
// sPositionProp with Quaternion) than the on-disk format (22-byte
// PropPosition with int16 binary-radian angles). The renderer needs the
// raw int16 angles for its rotation matrix builder.
//
// BRLIST is also read directly — it's a non-property chunk.
inline ObjectPropData parseObjectProps(PropertyService *propSvc,
                                        const std::string &misPath) {
    ObjectPropData result;

    // Open .mis file for P$Position and BRLIST (still needed for direct reads)
    FilePtr misFp(new StdFile(misPath, File::FILE_R));
    FileGroupPtr misDb(new DarkFileGroup(misFp));

    // Read BRLIST to identify editor brush objects (room/terrain/flow brushes)
    // that should not be rendered as standalone objects
    std::unordered_set<int32_t> brushObjIDs;
    detail::readBrushObjectIDs(misDb, brushObjIDs);

    // ── Read positions from P$Position ──
    // Read directly from file because PositionPropertyStorage uses a different
    // in-memory format (Quaternion) than the on-disk format (int16 angles).
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
                p.renderAlpha = 1.0f;  // default opaque, resolved later
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

    // ── Merge: resolve properties via PropertyService for each positioned object ──
    std::unordered_set<std::string> uniqueSet;
    int resolved = 0, unresolved = 0, filtered = 0;

    for (auto &kv : posMap) {
        int32_t id = kv.first;
        const ObjectPlacement &pos = kv.second;

        // Check render type — skip non-renderable objects (lights, markers, etc.)
        // Inheritance is resolved by PropertyService's Inheritor system.
        // 0=Normal, 1=NotRendered, 2=NoLightmap, 3=EditorOnly
        PropRenderType rt;
        if (getTypedProperty<PropRenderType>(propSvc, "RenderType", id, rt)) {
            if (rt.mode == 1 || rt.mode == 3) {
                ++filtered;
                continue;
            }
        }

        // Skip editor brush objects (from BRLIST) — room brushes, terrain brushes,
        // flow brushes, etc. are architectural geometry already baked into the
        // worldmesh or used for room/sound definitions, not standalone objects
        if (brushObjIDs.count(id)) {
            ++filtered;
            continue;
        }

        // Resolve model name through PropertyService inheritance chain.
        // The Inheritor walks archetype + MetaProp links automatically.
        PropModelName mn;
        if (!getTypedProperty<PropModelName>(propSvc, "ModelName", id, mn)) {
            ++unresolved;
            continue;
        }

        // Lowercase for consistent CRF lookups
        std::string name(mn.name);
        if (name.empty() || name[0] == '\0') {
            ++unresolved;
            continue;
        }
        // Ensure null-termination for names that fill all 16 bytes
        name.resize(std::strlen(mn.name));
        std::transform(name.begin(), name.end(), name.begin(), ::tolower);

        ObjectPlacement placement = pos;
        std::strncpy(placement.modelName, name.c_str(),
                     sizeof(placement.modelName) - 1);
        placement.modelName[15] = '\0';

        // Look up scale directly on this object (P$Scale does not inherit).
        // Dark Engine's P$Scale property uses kPropertyNoInherit — archetype
        // scales are physics bounding box dimensions, not visual model scales.
        // We use ownsProperty() to check direct ownership only.
        placement.scaleX = 1.0f;
        placement.scaleY = 1.0f;
        placement.scaleZ = 1.0f;
        if (ownsProperty(propSvc, "ModelScale", id)) {
            PropScale scale;
            if (getTypedProperty<PropScale>(propSvc, "ModelScale", id, scale)) {
                placement.scaleX = scale.x;
                placement.scaleY = scale.y;
                placement.scaleZ = scale.z;
            }
        }

        // Resolve render alpha through PropertyService inheritance.
        // RenderAlpha DOES inherit ("always" inheritance mode).
        PropRenderAlpha alpha;
        if (getTypedProperty<PropRenderAlpha>(propSvc, "RenderAlpha", id, alpha)) {
            // Clamp to valid range
            placement.renderAlpha = std::max(0.0f, std::min(1.0f, alpha.alpha));
        }

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
