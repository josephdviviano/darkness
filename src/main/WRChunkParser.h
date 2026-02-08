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

// Standalone WR/WRRGB chunk parser — reads cell geometry from .mis files
// using only OpdeBase primitives (File, FileGroup). No Ogre, no service stack.

#pragma once

#include "File.h"
#include "FileGroup.h"
#include "FileCompat.h"
#include "Vector3.h"
#include "Plane.h"

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace Opde {

// ── Packed structs (copied from WRTypes.h, avoiding LightmapAtlas dependency) ──

#pragma pack(push, 1)

struct WRHeader {
    uint32_t unk;
    uint32_t numCells;

    friend File &operator>>(File &st, WRHeader &ch) {
        st >> ch.unk >> ch.numCells;
        return st;
    }
};

struct WRCellHeader {
    uint8_t numVertices;
    uint8_t numPolygons;
    uint8_t numTextured;
    uint8_t numPortals;
    uint8_t numPlanes;
    uint8_t mediaType;
    uint8_t cellFlags;
    uint32_t nxn;
    uint16_t polymapSize;
    uint8_t numAnimLights;
    uint8_t flowGroup;
    Vector3 center;
    float radius;

    friend File &operator>>(File &st, WRCellHeader &ch) {
        st >> ch.numVertices >> ch.numPolygons >> ch.numTextured
           >> ch.numPortals >> ch.numPlanes >> ch.mediaType >> ch.cellFlags
           >> ch.nxn >> ch.polymapSize >> ch.numAnimLights >> ch.flowGroup
           >> ch.center >> ch.radius;
        return st;
    }
};

struct WRPolygon {
    uint8_t flags;
    uint8_t count;
    uint8_t plane;
    uint8_t unk;
    uint16_t tgtCell;
    uint8_t unk1;
    uint8_t unk2;

    friend File &operator>>(File &st, WRPolygon &ch) {
        st >> ch.flags >> ch.count >> ch.plane >> ch.unk >> ch.tgtCell
           >> ch.unk1 >> ch.unk2;
        return st;
    }
};

struct WRPolygonTexturing {
    Vector3 axisU;
    Vector3 axisV;
    int16_t u;
    int16_t v;
    uint8_t txt;
    uint8_t originVertex;
    uint16_t unk;
    float scale;
    Vector3 center;

    friend File &operator>>(File &st, WRPolygonTexturing &ch) {
        st >> ch.axisU >> ch.axisV >> ch.u >> ch.v >> ch.txt
           >> ch.originVertex >> ch.unk >> ch.scale >> ch.center;
        return st;
    }
};

struct WRLightInfo {
    int16_t u;
    int16_t v;
    uint16_t lx;
    uint8_t ly;
    uint8_t lx8;
    uint32_t staticLmapPtr;
    uint32_t dynamicLmapPtr;
    uint32_t animflags;

    friend File &operator>>(File &st, WRLightInfo &ch) {
        st >> ch.u >> ch.v >> ch.lx >> ch.ly >> ch.lx8 >> ch.staticLmapPtr
           >> ch.dynamicLmapPtr >> ch.animflags;
        return st;
    }
};

#pragma pack(pop)

// ── Output structures ──

struct WRParsedCell {
    uint8_t numPolygons, numPortals, numPlanes, mediaType;
    uint8_t numTextured;
    Vector3 center;
    float radius;
    std::vector<Vector3> vertices;
    std::vector<WRPolygon> polygons;
    std::vector<WRPolygonTexturing> texturing; // size = numTextured
    std::vector<std::vector<uint8_t>> polyIndices;
    std::vector<Plane> planes;
    std::vector<int16_t> animMap;                      // numAnimLights entries
    std::vector<WRLightInfo> lightInfos;               // numTextured entries
    std::vector<std::vector<uint8_t>> staticLightmaps; // raw pixel data per polygon
    // animLightmaps[polyIdx][overlayIdx] = raw pixel data (same size as static lmap)
    // Overlays are in bit order of animflags (lowest set bit first)
    std::vector<std::vector<std::vector<uint8_t>>> animLightmaps;
};

struct WRParsedData {
    uint32_t numCells;
    int lightSize; // 1 for WR (grayscale), 2 for WRRGB (xBGR 5:5:5)
    std::vector<WRParsedCell> cells;
};

// ── Helpers ──

inline int countBits(uint32_t src) {
    src = (src & 0x55555555) + ((src >> 1) & 0x55555555);
    src = (src & 0x33333333) + ((src >> 2) & 0x33333333);
    src = (src + (src >> 4)) & 0x0f0f0f0f;
    src = (src + (src >> 8));
    src = (src + (src >> 16));
    return src & 0xff;
}

// ── Parser ──

inline WRParsedData parseWRChunk(const std::string &misPath) {
    FilePtr fp(new StdFile(misPath, File::FILE_R));
    FileGroupPtr db(new DarkFileGroup(fp));

    int lightSize = 1;
    FilePtr chunk;

    if (db->hasFile("WRRGB")) {
        chunk = db->getFile("WRRGB");
        lightSize = 2;
    } else if (db->hasFile("WR")) {
        chunk = db->getFile("WR");
        lightSize = 1;
    } else {
        throw std::runtime_error("No WR or WRRGB chunk found in " + misPath);
    }

    WRHeader hdr;
    *chunk >> hdr;

    WRParsedData result;
    result.numCells = hdr.numCells;
    result.lightSize = lightSize;
    result.cells.resize(hdr.numCells);

    for (uint32_t ci = 0; ci < hdr.numCells; ++ci) {
        WRCellHeader ch;
        *chunk >> ch;

        WRParsedCell &cell = result.cells[ci];
        cell.numPolygons = ch.numPolygons;
        cell.numPortals = ch.numPortals;
        cell.numPlanes = ch.numPlanes;
        cell.mediaType = ch.mediaType;
        cell.center = ch.center;
        cell.radius = ch.radius;

        // 1. Vertices
        cell.vertices.resize(ch.numVertices);
        for (auto &v : cell.vertices) *chunk >> v;

        // 2. Polygon headers
        cell.polygons.resize(ch.numPolygons);
        for (auto &p : cell.polygons) *chunk >> p;

        // 3. Texturing info
        cell.numTextured = ch.numTextured;
        cell.texturing.resize(ch.numTextured);
        for (auto &t : cell.texturing) *chunk >> t;

        // 4. Polygon index map
        uint32_t numIndices;
        *chunk >> numIndices;

        cell.polyIndices.resize(ch.numPolygons);
        for (int p = 0; p < ch.numPolygons; ++p) {
            cell.polyIndices[p].resize(cell.polygons[p].count);
            chunk->read(cell.polyIndices[p].data(), cell.polygons[p].count);
        }

        // 5. Planes
        cell.planes.resize(ch.numPlanes);
        for (auto &pl : cell.planes) *chunk >> pl;

        // 6a. anim_map: numAnimLights × int16_t (store for future animated light support)
        cell.animMap.resize(ch.numAnimLights);
        for (auto &am : cell.animMap) { int16_t val; *chunk >> val; am = val; }

        // 6b. lightmap descriptors
        cell.lightInfos.resize(ch.numTextured);
        for (auto &li : cell.lightInfos) *chunk >> li;

        // 6c. lightmap pixel data — read static + animated overlays
        cell.staticLightmaps.resize(ch.numTextured);
        cell.animLightmaps.resize(ch.numTextured);
        for (int i = 0; i < ch.numTextured; ++i) {
            int lmSize = cell.lightInfos[i].lx * cell.lightInfos[i].ly * lightSize;
            cell.staticLightmaps[i].resize(lmSize);
            chunk->read(cell.staticLightmaps[i].data(), lmSize);
            // Read animated overlay lightmaps (one per set bit in animflags)
            int animCount = countBits(cell.lightInfos[i].animflags);
            cell.animLightmaps[i].resize(animCount);
            for (int a = 0; a < animCount; ++a) {
                cell.animLightmaps[i][a].resize(lmSize);
                chunk->read(cell.animLightmaps[i][a].data(), lmSize);
            }
        }

        // 6d. light_indices (skip for Phase 1)
        uint32_t lightCount;
        *chunk >> lightCount;
        if (lightCount > 0)
            chunk->seek(lightCount * 2, File::FSEEK_CUR);
    }

    return result;
}

} // namespace Opde
