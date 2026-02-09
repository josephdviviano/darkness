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

// Standalone spawn-point finder — reads L$PlayerFactory and P$Position chunks
// from a Dark Engine .mis file to locate the player starting position.
// Uses same DarkFileGroup infrastructure as WRChunkParser.h.

#pragma once

#include "File.h"
#include "FileGroup.h"
#include "FileCompat.h"

#include <cstdint>
#include <cmath>
#include <cstdio>
#include <string>

namespace Darkness {

// Result of spawn point search
struct SpawnInfo {
    float x, y, z;    // position
    float yaw;         // facing angle in radians (from heading in binary radians)
    bool found;        // false = no spawn found, caller should use centroid
};

// Find the player spawn point from a .mis file's chunk database.
//
// Algorithm:
// 1. Read "L$PlayerFac" chunk (truncated from "L$PlayerFactory") to find the
//    StartingPoint object ID. Each link is 14 bytes: {id, src, dest, flavor}.
//    The src field is the StartingPoint concrete object.
// 2. Read "P$Position" chunk to find that object's position. Each record is
//    variable-length: {uint32_t objID, uint32_t size, uint8_t data[size]}.
//    On-disk position data is 22 bytes: {float[3] pos, int32_t cell,
//    int16_t heading, int16_t bank, int16_t pitch}.
//    Angles are binary radians (65536 = 360 degrees).
// 3. Extract yaw from the heading angle.
inline SpawnInfo findSpawnPoint(const std::string &misPath) {
    SpawnInfo info = {0, 0, 0, 0, false};

    FilePtr fp(new StdFile(misPath, File::FILE_R));
    FileGroupPtr db(new DarkFileGroup(fp));

    // Step 1: Find StartingPoint object ID from L$PlayerFactory links
    // Chunk name truncated to 11 chars: "L$PlayerFac"
    const char *linkChunk = "L$PlayerFac";

    if (!db->hasFile(linkChunk)) {
        std::fprintf(stderr, "SpawnFinder: no %s chunk found\n", linkChunk);
        return info;
    }

    FilePtr linkFile = db->getFile(linkChunk);

    // Each link record is 14 bytes: {uint32_t id, int32_t src, int32_t dest, uint16_t flavor}
    const int LINK_STRUCT_SIZE = 14;
    int32_t startingPointID = 0;
    bool foundLink = false;

    // Read link records until EOF to find the first PlayerFactory link.
    // The src field is the concrete StartingPoint object ID.
    while (static_cast<size_t>(linkFile->tell()) + LINK_STRUCT_SIZE <=
           linkFile->size()) {
        uint32_t id;
        int32_t src, dest;
        uint16_t flavor;
        *linkFile >> id >> src >> dest >> flavor;

        // Take the first link — its src is the StartingPoint object
        startingPointID = src;
        foundLink = true;
        break;
    }

    if (!foundLink) {
        std::fprintf(stderr, "SpawnFinder: L$PlayerFac chunk is empty\n");
        return info;
    }

    std::fprintf(stderr, "SpawnFinder: StartingPoint object ID = %d\n",
                 startingPointID);

    // Step 2: Find that object's position in P$Position
    const char *posChunk = "P$Position";

    if (!db->hasFile(posChunk)) {
        std::fprintf(stderr, "SpawnFinder: no %s chunk found\n", posChunk);
        return info;
    }

    FilePtr posFile = db->getFile(posChunk);

    // Property records: {uint32_t objID, uint32_t size, uint8_t data[size]}
    // Scan through all records looking for our StartingPoint object ID
    while (static_cast<size_t>(posFile->tell()) + 8 <=
           posFile->size()) {
        uint32_t objID, dataSize;
        *posFile >> objID >> dataSize;

        // Check if this is our target object (compare as signed)
        // On-disk format: float[3] pos (12) + int32_t cell (4) + int16_t b/p/h (6) = 22 bytes
        if (static_cast<int32_t>(objID) == startingPointID && dataSize >= 22) {
            float px, py, pz;
            *posFile >> px >> py >> pz;

            int32_t cell;
            *posFile >> cell;

            // Facing angles in binary radians (65536 = 360 degrees)
            // On-disk order: bank, pitch, heading (= angvec tx, ty, tz)
            int16_t bank, pitch, heading;
            *posFile >> bank >> pitch >> heading;

            // Convert heading to radians for camera yaw
            float yaw = static_cast<float>(heading) * (2.0f * 3.14159265f / 65536.0f);

            info.x = px;
            info.y = py;
            info.z = pz;
            info.yaw = yaw;
            info.found = true;

            std::fprintf(stderr,
                "SpawnFinder: found position (%.1f, %.1f, %.1f) heading=%.1f deg\n",
                px, py, pz, static_cast<float>(heading) * 360.0f / 65536.0f);

            return info;
        }

        // Skip this record's data
        if (dataSize > 0) {
            posFile->seek(dataSize, File::FSEEK_CUR);
        }
    }

    std::fprintf(stderr, "SpawnFinder: object %d not found in P$Position\n",
                 startingPointID);
    return info;
}

} // namespace Darkness
