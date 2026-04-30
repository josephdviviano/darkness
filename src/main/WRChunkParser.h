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
// using only DarknessBase primitives (File, FileGroup). No Ogre, no service stack.

#pragma once

#include "File.h"
#include "FileGroup.h"
#include "FileCompat.h"
#include "DarknessMath.h"

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace Darkness {

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
    uint8_t flowGroup;  // 0=no flow, else flow group number for water animation
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
    // Static lights potentially affecting this cell. Element 0 holds the count;
    // elements [1..count] index into WRParsedData::staticLights. Built at level
    // bake; treated as a precomputed visibility filter (object lighting still
    // raycasts per-light to handle dynamic occlusion changes from doors etc.).
    std::vector<uint16_t> lightIndices;
};

// One entry in the static light table (Dark Engine "RGB lighting" build layout,
// 48 bytes on disk). Located at end of the WR/WRRGB chunk, after the per-cell
// stream. Slot 0 is reserved for the sun (overwritten per-evaluation by the
// object lighting code; on-disk values for slot 0 are scratch).
//
// `bright` is pre-divided by the bake-time light_scale (32.0) for non-sun
// slots. The sun's brightness is patched at runtime from RENDPARAMS' computed
// sun_scaled_rgb (which is NOT pre-divided).
//
// `inner` and `outer` are cosines of half-angles (inner > outer); inner = -1.0
// is the omni-directional sentinel (no spotlight cone applied).
struct WRStaticLight {
    Vector3 loc;
    Vector3 dir;
    Vector3 bright;
    float   inner;
    float   outer;
    float   radius;
};
// Lock the on-disk byte layout for the RGB build. Vector3 = glm::vec3 has
// no tail padding (12 B / 4 B align), so the natural struct layout matches
// the original engine's mls_multi_light record exactly. If a future GLM
// upgrade switches to padded vec3, this assert catches it before we read
// garbage from disk.
static_assert(sizeof(WRStaticLight) == 48,
              "WRStaticLight must be exactly 48 bytes to match Dark Engine "
              "RGB-build mls_multi_light on-disk layout");

struct WRParsedData {
    uint32_t numCells;
    int lightSize; // 1 for WR (grayscale), 2 for WRRGB (xBGR 5:5:5)
    std::vector<WRParsedCell> cells;
    // Static light table (active count + entries). On disk: int32 num_light,
    // int32 num_dyn, then 768 fixed-size entries followed by 32 scratch
    // entries. We keep only the active slice [0..numStaticLights).
    int32_t                    numStaticLights = 0;
    std::vector<WRStaticLight> staticLights;
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
        cell.flowGroup = ch.flowGroup;
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

        // 6d. light_indices: int32 num (= active count + 1) followed by num
        // ushorts. Element 0 is the count itself; elements [1..count] are
        // indices into the static light table. Empty cells (e.g. fully solid
        // or sealed-off) have num == 0.
        uint32_t numLightIndices;
        *chunk >> numLightIndices;
        if (numLightIndices > 0) {
            cell.lightIndices.resize(numLightIndices);
            chunk->read(cell.lightIndices.data(),
                        static_cast<size_t>(numLightIndices) * sizeof(uint16_t));
        }
    }

    // After the per-cell stream, the engine writes a BSP tree used by the
    // portal-cell hit tests, immediately followed by the static light table.
    //
    // BSP layout:
    //   int32  numExtraPlanes
    //   { vec3 normal; float plane_constant; } extras[numExtraPlanes]      (16 B each)
    //   int32  bspTreeSize
    //   { uint parent_index; int cell_id; int plane_id;
    //     uint inside_index; uint outside_index; } nodes[bspTreeSize]      (20 B each)
    //
    // We don't currently use the BSP tree (our portal traversal uses cells
    // directly), so we skip it. Layout discovered by tracing the original
    // engine's WriteWR -> wrBspTreeWrite path.
    {
        int32_t numExtraPlanes = 0;
        *chunk >> numExtraPlanes;
        if (numExtraPlanes > 0)
            chunk->seek(static_cast<size_t>(numExtraPlanes) * 16,
                        File::FSEEK_CUR);

        int32_t bspTreeSize = 0;
        *chunk >> bspTreeSize;
        if (bspTreeSize > 0)
            chunk->seek(static_cast<size_t>(bspTreeSize) * 20,
                        File::FSEEK_CUR);
    }

    // Static light table. Layout at end of chunk:
    //   int32  num_light    (active light count, including slot 0 = sun)
    //   int32  num_dyn      (always 0 in saved data)
    //   WRStaticLight light_data[768]   (MAX_STATIC; only [0..num_light) active)
    //   WRStaticLight light_this[32]    (scratch buffer; skipped)
    //   int32  num_anim_light_to_cell   (mapping table for the lightmap path)
    //   <anim_light_to_cell entries>    (skipped — handled by LightingSystem)
    //
    // We read all 768 entries (so disk offsets stay valid for any later chunk
    // reads) but surface only the active slice via numStaticLights.
    //
    // The on-disk record layout is RGB-build only (48 bytes per entry).
    // Grayscale-build missions (Thief 1) use a different 32-byte layout —
    // we don't currently support those for object lighting. The ObjectIlluminator
    // still works; the per-object output just won't include cell-light
    // contributions for those missions.
    constexpr int kMaxStaticLights = 768;
    constexpr int kLightThisCount  = 32;
    if (lightSize != 2) {
        // Grayscale WR: skip the static light section entirely; object
        // lighting will use ambient + sun + ExtraLight only.
        std::fprintf(stderr,
            "[WR] grayscale WR chunk — static light table not parsed "
            "(object lighting falls back to ambient+sun)\n");
        return result;
    }
    {
        int32_t numLight = 0, numDyn = 0;
        *chunk >> numLight >> numDyn;
        result.numStaticLights = numLight;

        result.staticLights.resize(kMaxStaticLights);
        chunk->read(result.staticLights.data(),
                    static_cast<size_t>(kMaxStaticLights) * sizeof(WRStaticLight));

        // Skip the scratch buffer.
        chunk->seek(static_cast<size_t>(kLightThisCount) * sizeof(WRStaticLight),
                    File::FSEEK_CUR);

        // Trim to active slice for memory hygiene; capacity stays.
        if (numLight >= 0 && numLight <= kMaxStaticLights)
            result.staticLights.resize(numLight);
    }

    return result;
}

} // namespace Darkness
