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

// DarknessRendererExtended.h — High-level builders and feature-specific data
//
// Mesh construction, environment parsing (fog, sky, water), and feature-specific
// data structures that assemble geometry and parse mission-specific chunks.

#pragma once

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <vector>
#include <string>
#include <algorithm>
#include <map>
#include <unordered_map>

#include "DarknessRendererCore.h"
#include "LightmapAtlas.h"

// Darkness File I/O (for chunk parsing)
#include "File.h"
#include "FileGroup.h"

namespace Darkness {

// ── Water flow data ──
// Forward declarations needed by buildWaterMesh and parseFlowData.

// FLOW_TEX chunk: 256 entries mapping flow group index to water textures.
// in_texture/out_texture are runtime-allocated palette indices — NOT usable as
// TXLIST indices. Instead, the name field holds the texture base name which maps
// to fam.crf paths: "water/<name>in.PCX" (air side) / "water/<name>out.PCX".
static constexpr int MAX_FLOW_GROUPS = 256;

struct FlowTexEntry {
    int16_t inTexture;     // runtime palette index (NOT a TXLIST index)
    int16_t outTexture;    // runtime palette index (NOT a TXLIST index)
    char    name[28];      // texture base name (e.g. "gr" → water/grin.PCX)
};
static_assert(sizeof(FlowTexEntry) == 32, "FLOW_TEX entry must be 32 bytes");

// CELL_MOTION chunk contains two sequential arrays:
// 1. PortalCellMotion[256] — current state (center, angle, flags, axis)
// 2. sMedMoCellMotion[256] — velocities (center_change, angle_change)
// The state stores the current animation position; the velocities are per-second deltas.
#pragma pack(push, 1)
struct PortalCellMotion {
    float    centerX, centerY, centerZ; // rotation pivot / UV origin in world space
    uint16_t angle;                     // current rotation angle (binary radians, 65536 = 360°)
    int32_t  inMotion;                  // 0=disabled, 1=enabled
    uint8_t  majorAxis;                 // water plane: 0=X, 1=Y, 2=Z (default Z for horizontal)
    uint8_t  pad1;
    uint8_t  pad2;
};
static_assert(sizeof(PortalCellMotion) == 21, "PortalCellMotion must be 21 bytes");

struct MedMoCellMotion {
    float    centerChangeX, centerChangeY, centerChangeZ; // UV scroll velocity (world units/sec)
    uint16_t angleChange;  // rotation velocity (binary radians/sec, cast to int16_t for sign)
};
static_assert(sizeof(MedMoCellMotion) == 14, "MedMoCellMotion must be 14 bytes");
#pragma pack(pop)

// Parsed flow data for the mission — texture mapping and animation velocities.
struct FlowData {
    FlowTexEntry      textures[MAX_FLOW_GROUPS];
    PortalCellMotion  state[MAX_FLOW_GROUPS];     // current animation state
    MedMoCellMotion   velocity[MAX_FLOW_GROUPS];  // animation velocities (per second)
    bool hasFlowTex    = false;  // true if FLOW_TEX chunk was found
    bool hasCellMotion = false;  // true if CELL_MOTION chunk was found
};

// ── Build flat-shaded mesh (no textures) ──

inline FlatMesh buildFlatMesh(const Darkness::WRParsedData &wr) {
    FlatMesh mesh;

    // Sun direction for Lambertian shading
    float sunDir[3] = { 0.3f, 0.8f, 0.4f };
    float sunLen = std::sqrt(sunDir[0]*sunDir[0] + sunDir[1]*sunDir[1] + sunDir[2]*sunDir[2]);
    sunDir[0] /= sunLen; sunDir[1] /= sunLen; sunDir[2] /= sunLen;

    float sumX = 0, sumY = 0, sumZ = 0;
    int cellCount = 0;

    // Temporary per-(cell, texture) triangle lists for portal culling support
    // Key: (cellID << 8) | txtIndex — cell in high bits for sort order
    std::map<uint64_t, std::vector<uint32_t>> cellTexTriangles;

    for (uint32_t ci = 0; ci < wr.numCells; ++ci) {
        const auto &cell = wr.cells[ci];

        sumX += cell.center.x;
        sumY += cell.center.y;
        sumZ += cell.center.z;
        ++cellCount;

        int numSolid = cell.numPolygons - cell.numPortals;

        for (int pi = 0; pi < numSolid; ++pi) {
            const auto &poly = cell.polygons[pi];
            const auto &indices = cell.polyIndices[pi];

            if (poly.count < 3) continue;

            // Skip sky polygons (texture 249 = BACKHACK_IDX) — these are holes
            // that should reveal the sky dome rendered behind world geometry
            if (pi < cell.numTextured && cell.texturing[pi].txt == 249)
                continue;

            // Get plane normal for flat shading
            Darkness::Plane plane;
            if (poly.plane < cell.planes.size())
                plane = cell.planes[poly.plane];

            Darkness::Vector3 n = glm::normalize(plane.normal);
            float dot = n.x * sunDir[0] + n.y * sunDir[1] + n.z * sunDir[2];
            float brightness = std::max(dot, 0.0f) * 0.85f + 0.15f;

            // Color based on cell media type
            uint32_t color;
            if (cell.mediaType == 2) {
                // Water: blue tint
                color = packABGR(brightness * 0.4f, brightness * 0.5f, brightness * 0.8f);
            } else {
                // Air: warm stone beige
                color = packABGR(brightness * 0.75f, brightness * 0.65f, brightness * 0.5f);
            }

            // Emit vertices for this polygon
            uint32_t baseVertex = static_cast<uint32_t>(mesh.vertices.size());

            for (int vi = 0; vi < poly.count; ++vi) {
                uint8_t idx = indices[vi];
                if (idx >= cell.vertices.size()) continue;
                const auto &v = cell.vertices[idx];
                mesh.vertices.push_back({v.x, v.y, v.z, color});
            }

            // Fan-triangulate: (0, t+1, t) — matches original WRCell.cpp winding
            // Group by (cellID, 0) since flat mesh has no texture index
            uint64_t key = (static_cast<uint64_t>(ci) << 8);
            auto &triList = cellTexTriangles[key];
            for (int t = 1; t < poly.count - 1; ++t) {
                triList.push_back(baseVertex);
                triList.push_back(baseVertex + t + 1);
                triList.push_back(baseVertex + t);
            }
        }
    }

    // Build sorted index buffer: groups sorted by (cellID, txtIndex)
    // std::map is already sorted by key
    for (auto &kv : cellTexTriangles) {
        uint32_t cellID = static_cast<uint32_t>(kv.first >> 8);
        uint8_t txtIdx = static_cast<uint8_t>(kv.first & 0xFF);
        CellTextureGroup grp;
        grp.cellID = cellID;
        grp.txtIndex = txtIdx;
        grp.firstIndex = static_cast<uint32_t>(mesh.indices.size());
        grp.numIndices = static_cast<uint32_t>(kv.second.size());
        mesh.groups.push_back(grp);
        mesh.indices.insert(mesh.indices.end(), kv.second.begin(), kv.second.end());
    }

    if (cellCount > 0) {
        mesh.cx = sumX / cellCount;
        mesh.cy = sumY / cellCount;
        mesh.cz = sumZ / cellCount;
    } else {
        mesh.cx = mesh.cy = mesh.cz = 0;
    }

    return mesh;
}

// ── Build water mesh with flow group mapping ──
// Water portals are portal polygons with the 0x10 flag that connect air cells
// (mediaType 1) to water cells (mediaType 2). Portals with 0x10 between air
// cells are room brush zone boundaries and should not be rendered as water.
// Only emit from air cells (mediaType==1) to avoid double-rendering — the
// same portal polygon exists in both the air and water cell.
// Texture source priority:
//   1. FLOW_TEX chunk (via cell.flowGroup) — canonical water texture source
//   2. Polygon texturing data (fallback if no FLOW_TEX or flowGroup==0)
//   3. Flat blue-green vertex color (if no texture available)

inline WorldMesh buildWaterMesh(const Darkness::WRParsedData &wr,
                                const std::unordered_map<uint8_t, TexDimensions> &texDims,
                                const FlowData &flowData,
                                const std::unordered_map<uint8_t, TexDimensions> &flowTexDims) {
    WorldMesh mesh;
    // Semi-transparent dark blue-green — fallback for non-textured water portals
    uint32_t waterColor = packABGR(0.2f, 0.32f, 0.4f, 0.35f);
    // Semi-transparent white — lets texture colors show through for textured water
    uint32_t texWaterColor = packABGR(1.0f, 1.0f, 1.0f, 0.35f);
    int waterPolyCount = 0;

    // Composite key for grouping triangles by texture source.
    // Flow-textured water uses 0x8000 | flowGroup (high bit set).
    // TXLIST-textured water uses (txtIndex << 8) | flowGroup.
    // This ensures each texture source gets its own draw group.
    std::unordered_map<uint16_t, std::vector<uint32_t>> texTriangles;
    // Track flow group and texture index per composite key
    std::unordered_map<uint16_t, uint8_t> keyFlowGroup;
    std::unordered_map<uint16_t, uint8_t> keyTxtIndex;

    for (uint32_t ci = 0; ci < wr.numCells; ++ci) {
        const auto &cell = wr.cells[ci];
        // Only emit from air cells to avoid double-rendering at portal boundaries
        if (cell.mediaType != 1) continue;
        int numSolid = cell.numPolygons - cell.numPortals;

        for (int pi = numSolid; pi < cell.numPolygons; ++pi) {
            const auto &poly = cell.polygons[pi];
            const auto &indices = cell.polyIndices[pi];
            if (poly.count < 3) continue;
            // Water surface detection: portal polygon must have the water flag
            // (bit 0x10) AND the target cell must be a water cell (mediaType 2).
            // Portal polygons with 0x10 between air cells are room brush zone
            // boundaries — invisible in the original engine, not water surfaces.
            if (!(poly.flags & 0x10)) continue;
            if (poly.tgtCell < wr.numCells &&
                wr.cells[poly.tgtCell].mediaType != 2) continue;

            // Resolve flow group: water portals connect an air cell to a water cell.
            // The flowGroup is on the water cell (target), not the air cell (source).
            uint8_t cellFlowGroup = 0;
            bool hasFlowTex = false;
            if (poly.tgtCell < wr.numCells) {
                cellFlowGroup = wr.cells[poly.tgtCell].flowGroup;
                if (cellFlowGroup > 0) {
                    hasFlowTex = flowTexDims.count(cellFlowGroup) > 0;
                }
            }

            // Determine texture: prefer flow texture (loaded by name from fam.crf),
            // then polygon texturing from TXLIST, then flat color.
            bool hasPolyTex = (pi < cell.numTextured);
            uint8_t polyTxtIdx = 0;
            if (hasPolyTex) {
                polyTxtIdx = cell.texturing[pi].txt;
                if (polyTxtIdx == 0 || polyTxtIdx == 249)
                    hasPolyTex = false;
            }

            // Flow texture takes priority — polygon texture is fallback
            uint8_t txtIdx = hasFlowTex ? 0 : polyTxtIdx;
            bool isTextured = hasFlowTex || (txtIdx > 0);

            float texW = 64.0f, texH = 64.0f;
            if (hasFlowTex) {
                // Dimensions from flow texture loaded by name from fam.crf
                auto fit = flowTexDims.find(cellFlowGroup);
                if (fit != flowTexDims.end()) {
                    texW = static_cast<float>(fit->second.w);
                    texH = static_cast<float>(fit->second.h);
                }
            } else if (isTextured) {
                auto it = texDims.find(txtIdx);
                if (it != texDims.end()) {
                    texW = static_cast<float>(it->second.w);
                    texH = static_cast<float>(it->second.h);
                }
            }

            uint32_t baseVertex = static_cast<uint32_t>(mesh.vertices.size());

            if (isTextured && hasPolyTex) {
                // Textured water with polygon UV data — use projected UVs
                const auto &tex = cell.texturing[pi];
                Darkness::Vector3 origin(0, 0, 0);
                if (tex.originVertex < indices.size()) {
                    uint8_t oi = indices[tex.originVertex];
                    if (oi < cell.vertices.size())
                        origin = cell.vertices[oi];
                }

                float mag2_u = glm::length2(tex.axisU);
                float mag2_v = glm::length2(tex.axisV);
                float sh_u = tex.u / 4096.0f;
                float sh_v = tex.v / 4096.0f;
                float dotp = glm::dot(tex.axisU, tex.axisV);

                for (int vi = 0; vi < poly.count; ++vi) {
                    uint8_t idx = indices[vi];
                    if (idx >= cell.vertices.size()) continue;
                    const auto &coord = cell.vertices[idx];

                    Darkness::Vector3 tmp = coord - origin;
                    float u, v;

                    if (std::abs(dotp) < 1e-6f) {
                        u = glm::dot(tex.axisU, tmp) / mag2_u + sh_u;
                        v = glm::dot(tex.axisV, tmp) / mag2_v + sh_v;   // why is this one a member function?
                    } else {
                        float corr = 1.0f / (mag2_u * mag2_v - dotp * dotp);
                        float cu = corr * mag2_v;
                        float cv = corr * mag2_u;
                        float cross = corr * dotp;
                        float pu = glm::dot(tex.axisU, tmp);
                        float pv = glm::dot(tex.axisV, tmp);  // why is this one a member function?
                        u = pu * cu - pv * cross + sh_u;
                        v = pv * cv - pu * cross + sh_v;
                    }

                    // Normalize: projection is in 64-texel units, scale to 0..1 UV
                    u /= (texW / 64.0f);
                    v /= (texH / 64.0f);

                    mesh.vertices.push_back({coord.x, coord.y, coord.z, texWaterColor, u, v});
                }
            } else if (isTextured) {
                // Textured water from FLOW_TEX but no polygon UV data —
                // compute planar UVs by projecting onto the water plane.
                // Dark Engine uses 4 world units per texture repeat (TEXTURE_UNIT_LENGTH).
                constexpr float TEX_SCALE = 4.0f;
                for (int vi = 0; vi < poly.count; ++vi) {
                    uint8_t idx = indices[vi];
                    if (idx >= cell.vertices.size()) continue;
                    const auto &coord = cell.vertices[idx];

                    // Project position onto the two axes perpendicular to the
                    // water plane. Most water is Z-up (horizontal), so use X,Y.
                    float u = coord.x / (texW * TEX_SCALE / 64.0f);
                    float v = coord.y / (texH * TEX_SCALE / 64.0f);

                    mesh.vertices.push_back({coord.x, coord.y, coord.z, texWaterColor, u, v});
                }
            } else {
                // Non-textured water portal: flat color, no UVs
                for (int vi = 0; vi < poly.count; ++vi) {
                    uint8_t idx = indices[vi];
                    if (idx >= cell.vertices.size()) continue;
                    const auto &v = cell.vertices[idx];
                    mesh.vertices.push_back({v.x, v.y, v.z, waterColor, 0.0f, 0.0f});
                }
            }

            // Fan triangulation: same winding as buildFlatMesh
            // Composite key: flow-textured uses 0x8000|flowGroup, TXLIST-textured
            // uses (txtIdx<<8)|flowGroup to avoid key collisions.
            uint16_t key;
            if (hasFlowTex) {
                key = 0x8000 | static_cast<uint16_t>(cellFlowGroup);
            } else {
                key = (static_cast<uint16_t>(txtIdx) << 8) | cellFlowGroup;
            }
            keyFlowGroup[key] = cellFlowGroup;
            keyTxtIndex[key] = txtIdx;
            auto &triList = texTriangles[key];
            for (int t = 1; t < poly.count - 1; ++t) {
                triList.push_back(baseVertex);
                triList.push_back(baseVertex + t + 1);
                triList.push_back(baseVertex + t);
            }
            ++waterPolyCount;
        }
    }

    // Build sorted index buffer with texture groups
    std::vector<uint16_t> sortedKeys;
    for (auto &kv : texTriangles)
        sortedKeys.push_back(kv.first);
    std::sort(sortedKeys.begin(), sortedKeys.end());

    for (uint16_t key : sortedKeys) {
        auto &triList = texTriangles[key];
        CellTextureGroup grp;
        grp.cellID = 0; // water mesh is not portal-culled
        grp.txtIndex = keyTxtIndex[key];
        grp.flowGroup = keyFlowGroup[key];
        grp.firstIndex = static_cast<uint32_t>(mesh.indices.size());
        grp.numIndices = static_cast<uint32_t>(triList.size());
        mesh.groups.push_back(grp);
        mesh.indices.insert(mesh.indices.end(), triList.begin(), triList.end());
    }

    mesh.cx = mesh.cy = mesh.cz = 0;
    // Count flow-textured vs TXLIST-textured vs flat water groups
    int flowTexGroups = 0, txlistTexGroups = 0, flatGroups = 0;
    for (const auto &g : mesh.groups) {
        if (g.flowGroup > 0) ++flowTexGroups;
        else if (g.txtIndex > 0) ++txlistTexGroups;
        else ++flatGroups;
    }
    std::fprintf(stderr, "Water mesh: %d polygons, %zu vertices, %zu indices, %zu groups"
                 " (%d flow-textured, %d txlist, %d flat)\n",
                 waterPolyCount, mesh.vertices.size(), mesh.indices.size(), mesh.groups.size(),
                 flowTexGroups, txlistTexGroups, flatGroups);
    return mesh;
}

// ── Build textured mesh with UV coordinates ──

inline WorldMesh buildTexturedMesh(const Darkness::WRParsedData &wr,
                                   const std::unordered_map<uint8_t, TexDimensions> &texDims) {
    WorldMesh mesh;

    // Sun direction for Lambertian shading
    float sunDir[3] = { 0.3f, 0.8f, 0.4f };
    float sunLen = std::sqrt(sunDir[0]*sunDir[0] + sunDir[1]*sunDir[1] + sunDir[2]*sunDir[2]);
    sunDir[0] /= sunLen; sunDir[1] /= sunLen; sunDir[2] /= sunLen;

    float sumX = 0, sumY = 0, sumZ = 0;
    int cellCount = 0;

    // Temporary per-(cell, texture) triangle lists for portal culling support
    // Key: (cellID << 8) | txtIndex — cell in high bits for sort order
    std::map<uint64_t, std::vector<uint32_t>> cellTexTriangles;

    for (uint32_t ci = 0; ci < wr.numCells; ++ci) {
        const auto &cell = wr.cells[ci];

        sumX += cell.center.x;
        sumY += cell.center.y;
        sumZ += cell.center.z;
        ++cellCount;

        int numSolid = cell.numPolygons - cell.numPortals;

        for (int pi = 0; pi < numSolid; ++pi) {
            const auto &poly = cell.polygons[pi];
            const auto &indices = cell.polyIndices[pi];

            if (poly.count < 3) continue;

            Darkness::Plane plane;
            if (poly.plane < cell.planes.size())
                plane = cell.planes[poly.plane];

            Darkness::Vector3 n = glm::normalize(plane.normal);
            float dot = n.x * sunDir[0] + n.y * sunDir[1] + n.z * sunDir[2];
            float brightness = std::max(dot, 0.0f) * 0.85f + 0.15f;

            // Determine if this polygon is textured
            bool isTextured = (pi < cell.numTextured);
            uint8_t txtIdx = 0;
            float texW = 64.0f, texH = 64.0f;

            if (isTextured) {
                const auto &tex = cell.texturing[pi];
                txtIdx = tex.txt;

                // Skip sky polygons entirely — they become holes for the sky dome
                if (txtIdx == 249)
                    continue;

                // Null texture falls through to flat shading
                if (txtIdx == 0)
                    isTextured = false;
            }

            if (isTextured) {
                auto it = texDims.find(txtIdx);
                if (it != texDims.end()) {
                    texW = static_cast<float>(it->second.w);
                    texH = static_cast<float>(it->second.h);
                }
            }

            // Neutral white lighting for textured, warm tint for flat
            uint32_t color;
            if (isTextured) {
                color = packABGR(brightness, brightness, brightness);
            } else if (cell.mediaType == 2) {
                color = packABGR(brightness * 0.4f, brightness * 0.5f, brightness * 0.8f);
            } else {
                color = packABGR(brightness * 0.75f, brightness * 0.65f, brightness * 0.5f);
            }

            uint32_t baseVertex = static_cast<uint32_t>(mesh.vertices.size());

            // UV computation (ref: WRCell.cpp:145-212)
            if (isTextured) {
                const auto &tex = cell.texturing[pi];
                Darkness::Vector3 origin(0, 0, 0);
                if (tex.originVertex < indices.size()) {
                    uint8_t oi = indices[tex.originVertex];
                    if (oi < cell.vertices.size())
                        origin = cell.vertices[oi];
                }

                float mag2_u = glm::length2(tex.axisU);
                float mag2_v = glm::length2(tex.axisV);
                float sh_u = tex.u / 4096.0f;
                float sh_v = tex.v / 4096.0f;
                float dotp = glm::dot(tex.axisU, tex.axisV);

                for (int vi = 0; vi < poly.count; ++vi) {
                    uint8_t idx = indices[vi];
                    if (idx >= cell.vertices.size()) continue;
                    const auto &coord = cell.vertices[idx];

                    Darkness::Vector3 tmp = coord - origin;
                    float u, v;

                    if (std::abs(dotp) < 1e-6f) {
                        // Orthogonal axes — direct projection
                        u = glm::dot(tex.axisU, tmp) / mag2_u + sh_u;
                        v = glm::dot(tex.axisV, tmp) / mag2_v + sh_v;
                    } else {
                        // Non-orthogonal correction (WRCell.cpp:196-203)
                        float corr = 1.0f / (mag2_u * mag2_v - dotp * dotp);
                        float cu = corr * mag2_v;
                        float cv = corr * mag2_u;
                        float cross = corr * dotp;
                        float pu = glm::dot(tex.axisU, tmp);
                        float pv = glm::dot(tex.axisV, tmp);
                        u = pu * cu - pv * cross + sh_u;
                        v = pv * cv - pu * cross + sh_v;
                    }

                    // Normalize: projection is in 64-texel units, scale to 0..1 UV
                    u /= (texW / 64.0f);
                    v /= (texH / 64.0f);

                    mesh.vertices.push_back({coord.x, coord.y, coord.z, color, u, v});
                }
            } else {
                for (int vi = 0; vi < poly.count; ++vi) {
                    uint8_t idx = indices[vi];
                    if (idx >= cell.vertices.size()) continue;
                    const auto &coord = cell.vertices[idx];
                    mesh.vertices.push_back({coord.x, coord.y, coord.z, color, 0.0f, 0.0f});
                }
            }

            // Fan-triangulate, grouped by (cellID, txtIndex) for portal culling
            uint8_t key = isTextured ? txtIdx : 0; // 0 = flat group
            uint64_t cellTexKey = (static_cast<uint64_t>(ci) << 8) | key;
            auto &triList = cellTexTriangles[cellTexKey];
            for (int t = 1; t < poly.count - 1; ++t) {
                triList.push_back(baseVertex);
                triList.push_back(baseVertex + t + 1);
                triList.push_back(baseVertex + t);
            }
        }
    }

    // Build sorted index buffer: groups sorted by (cellID, txtIndex)
    // std::map is already sorted by key
    for (auto &kv : cellTexTriangles) {
        uint32_t cellID = static_cast<uint32_t>(kv.first >> 8);
        uint8_t txtIdx = static_cast<uint8_t>(kv.first & 0xFF);
        CellTextureGroup grp;
        grp.cellID = cellID;
        grp.txtIndex = txtIdx;
        grp.firstIndex = static_cast<uint32_t>(mesh.indices.size());
        grp.numIndices = static_cast<uint32_t>(kv.second.size());
        mesh.groups.push_back(grp);
        mesh.indices.insert(mesh.indices.end(), kv.second.begin(), kv.second.end());
    }

    if (cellCount > 0) {
        mesh.cx = sumX / cellCount;
        mesh.cy = sumY / cellCount;
        mesh.cz = sumZ / cellCount;
    } else {
        mesh.cx = mesh.cy = mesh.cz = 0;
    }

    return mesh;
}

// ── Build lightmapped mesh with dual UV channels ──

inline LightmappedMesh buildLightmappedMesh(
    const Darkness::WRParsedData &wr,
    const std::unordered_map<uint8_t, TexDimensions> &texDims,
    const Darkness::LightmapAtlasSet &lmAtlas)
{
    LightmappedMesh mesh;

    float sumX = 0, sumY = 0, sumZ = 0;
    int cellCount = 0;

    // Temporary per-(cell, texture) triangle lists for portal culling support
    std::map<uint64_t, std::vector<uint32_t>> cellTexTriangles;

    auto findWrap = [](float x) -> float {
        if (x >= 0) return -64.0f * static_cast<int>(x / 64.0f);
        else return -64.0f * (-1 + static_cast<int>(x / 64.0f));
    };

    for (uint32_t ci = 0; ci < wr.numCells; ++ci) {
        const auto &cell = wr.cells[ci];

        sumX += cell.center.x;
        sumY += cell.center.y;
        sumZ += cell.center.z;
        ++cellCount;

        int numSolid = cell.numPolygons - cell.numPortals;

        for (int pi = 0; pi < numSolid; ++pi) {
            const auto &poly = cell.polygons[pi];
            const auto &indices = cell.polyIndices[pi];

            if (poly.count < 3) continue;

            bool isTextured = (pi < cell.numTextured);
            uint8_t txtIdx = 0;
            float texW = 64.0f, texH = 64.0f;

            if (isTextured) {
                const auto &tex = cell.texturing[pi];
                txtIdx = tex.txt;

                // Skip sky polygons entirely — they become holes for the sky dome
                if (txtIdx == 249)
                    continue;

                // Null texture falls through to flat shading
                if (txtIdx == 0)
                    isTextured = false;
            }

            if (isTextured) {
                auto it = texDims.find(txtIdx);
                if (it != texDims.end()) {
                    texW = static_cast<float>(it->second.w);
                    texH = static_cast<float>(it->second.h);
                }
            }

            // Check if this polygon has lightmap data
            bool hasLightmap = isTextured && pi < static_cast<int>(cell.lightInfos.size())
                && cell.lightInfos[pi].lx > 0 && cell.lightInfos[pi].ly > 0
                && ci < lmAtlas.entries.size()
                && pi < static_cast<int>(lmAtlas.entries[ci].size());

            uint32_t baseVertex = static_cast<uint32_t>(mesh.vertices.size());

            if (isTextured) {
                const auto &tex = cell.texturing[pi];
                Darkness::Vector3 origin(0, 0, 0);
                if (tex.originVertex < indices.size()) {
                    uint8_t oi = indices[tex.originVertex];
                    if (oi < cell.vertices.size())
                        origin = cell.vertices[oi];
                }

                float mag2_u = glm::length2(tex.axisU);
                float mag2_v = glm::length2(tex.axisV);
                float sh_u = tex.u / 4096.0f;
                float sh_v = tex.v / 4096.0f;
                float dotp = glm::dot(tex.axisU, tex.axisV);

                // Lightmap shift and entry (if available)
                const Darkness::WRLightInfo *li = nullptr;
                const Darkness::LmapEntry *lmEntry = nullptr;
                float lsh_u = 0, lsh_v = 0;

                if (hasLightmap) {
                    li = &cell.lightInfos[pi];
                    lmEntry = &lmAtlas.entries[ci][pi];
                    lsh_u = (0.5f - li->u) + tex.u / 1024.0f;
                    lsh_v = (0.5f - li->v) + tex.v / 1024.0f;
                }

                // First pass: compute raw lightmap UVs to find min for wrapping
                struct VertexData {
                    float x, y, z;
                    float diffU, diffV;
                    float lmU, lmV;
                };
                std::vector<VertexData> verts(poly.count);

                for (int vi = 0; vi < poly.count; ++vi) {
                    uint8_t idx = indices[vi];
                    if (idx >= cell.vertices.size()) { verts[vi] = {}; continue; }
                    const auto &coord = cell.vertices[idx];
                    verts[vi].x = coord.x;
                    verts[vi].y = coord.y;
                    verts[vi].z = coord.z;

                    Darkness::Vector3 tmp = coord - origin;
                    float pu = glm::dot(tex.axisU, tmp);
                    float pv = glm::dot(tex.axisV, tmp);
                    float projU, projV;

                    if (std::abs(dotp) < 1e-6f) {
                        projU = pu / mag2_u;
                        projV = pv / mag2_v;
                    } else {
                        float corr = 1.0f / (mag2_u * mag2_v - dotp * dotp);
                        float cu = corr * mag2_v;
                        float cv = corr * mag2_u;
                        float cross = corr * dotp;
                        projU = pu * cu - pv * cross;
                        projV = pv * cv - pu * cross;
                    }

                    // Diffuse UV
                    verts[vi].diffU = (projU + sh_u) / (texW / 64.0f);
                    verts[vi].diffV = (projV + sh_v) / (texH / 64.0f);

                    // Lightmap UV (raw, before wrapping)
                    if (hasLightmap) {
                        verts[vi].lmU = 4.0f * projU + lsh_u;
                        verts[vi].lmV = 4.0f * projV + lsh_v;
                    }
                }

                if (hasLightmap) {
                    // Find min lightmap UV for wrap correction
                    float minLmU = verts[0].lmU, minLmV = verts[0].lmV;
                    for (int vi = 1; vi < poly.count; ++vi) {
                        if (verts[vi].lmU < minLmU) minLmU = verts[vi].lmU;
                        if (verts[vi].lmV < minLmV) minLmV = verts[vi].lmV;
                    }

                    float wrapU = findWrap(minLmU);
                    float wrapV = findWrap(minLmV);

                    float invLx = 1.0f / static_cast<float>(li->lx);
                    float invLy = 1.0f / static_cast<float>(li->ly);

                    for (int vi = 0; vi < poly.count; ++vi) {
                        float lmU = (verts[vi].lmU + wrapU) * invLx;
                        float lmV = (verts[vi].lmV + wrapV) * invLy;

                        // Remap to atlas coordinates
                        float atlasU = lmU * lmEntry->atlasSU + lmEntry->atlasU;
                        float atlasV = lmV * lmEntry->atlasSV + lmEntry->atlasV;

                        mesh.vertices.push_back({
                            verts[vi].x, verts[vi].y, verts[vi].z,
                            verts[vi].diffU, verts[vi].diffV,
                            atlasU, atlasV
                        });
                    }
                } else {
                    // No lightmap — point at white fallback pixel in atlas
                    const Darkness::LmapEntry &fallback = (ci < lmAtlas.entries.size() && !lmAtlas.entries[ci].empty())
                        ? lmAtlas.entries[ci][0] : Darkness::LmapEntry{0, 0, 0, 0, 0, 0, 0, 0, 0};

                    for (int vi = 0; vi < poly.count; ++vi) {
                        mesh.vertices.push_back({
                            verts[vi].x, verts[vi].y, verts[vi].z,
                            verts[vi].diffU, verts[vi].diffV,
                            fallback.atlasU, fallback.atlasV
                        });
                    }
                }
            } else {
                // Non-textured polygon — flat fallback
                for (int vi = 0; vi < poly.count; ++vi) {
                    uint8_t idx = indices[vi];
                    if (idx >= cell.vertices.size()) continue;
                    const auto &coord = cell.vertices[idx];
                    mesh.vertices.push_back({coord.x, coord.y, coord.z, 0.0f, 0.0f, 0.0f, 0.0f});
                }
            }

            // Fan-triangulate, grouped by (cellID, txtIndex) for portal culling
            uint8_t key = isTextured ? txtIdx : 0;
            uint64_t cellTexKey = (static_cast<uint64_t>(ci) << 8) | key;
            auto &triList = cellTexTriangles[cellTexKey];
            for (int t = 1; t < poly.count - 1; ++t) {
                triList.push_back(baseVertex);
                triList.push_back(baseVertex + t + 1);
                triList.push_back(baseVertex + t);
            }
        }
    }

    // Build sorted index buffer: groups sorted by (cellID, txtIndex)
    for (auto &kv : cellTexTriangles) {
        uint32_t cellID = static_cast<uint32_t>(kv.first >> 8);
        uint8_t txtIdx = static_cast<uint8_t>(kv.first & 0xFF);
        CellTextureGroup grp;
        grp.cellID = cellID;
        grp.txtIndex = txtIdx;
        grp.firstIndex = static_cast<uint32_t>(mesh.indices.size());
        grp.numIndices = static_cast<uint32_t>(kv.second.size());
        mesh.groups.push_back(grp);
        mesh.indices.insert(mesh.indices.end(), kv.second.begin(), kv.second.end());
    }

    if (cellCount > 0) {
        mesh.cx = sumX / cellCount;
        mesh.cy = sumY / cellCount;
        mesh.cz = sumZ / cellCount;
    } else {
        mesh.cx = mesh.cy = mesh.cz = 0;
    }

    return mesh;
}

// ── Object fallback cube ──

// Build a small unit cube as PosColorVertex (fallback for missing models)
inline void buildFallbackCube(std::vector<PosColorVertex> &verts,
                               std::vector<uint32_t> &indices,
                               uint32_t color) {
    // 8 vertices of a unit cube centered at origin (±0.5)
    const float h = 0.5f;
    const float p[8][3] = {
        {-h,-h,-h}, { h,-h,-h}, { h, h,-h}, {-h, h,-h},
        {-h,-h, h}, { h,-h, h}, { h, h, h}, {-h, h, h}
    };
    uint32_t base = static_cast<uint32_t>(verts.size());
    for (int i = 0; i < 8; ++i) {
        verts.push_back({p[i][0], p[i][1], p[i][2], color});
    }
    // 12 triangles (6 faces x 2)
    const uint32_t idx[36] = {
        0,2,1, 0,3,2, // front (-Z)
        4,5,6, 4,6,7, // back (+Z)
        0,1,5, 0,5,4, // bottom
        2,3,7, 2,7,6, // top
        0,4,7, 0,7,3, // left
        1,2,6, 1,6,5  // right
    };
    for (int i = 0; i < 36; ++i) {
        indices.push_back(base + idx[i]);
    }
}

// ── Fog parsing ──

// Parse the FOG chunk from a .mis file if present.
// Format: DarkDBChunkFOG = { int32 red, int32 green, int32 blue, float distance }
inline FogParams parseFogChunk(const char *misPath) {
    FogParams fog = { false, 0.0f, 0.0f, 0.0f, 1.0f };

    try {
        Darkness::FilePtr fp(new Darkness::StdFile(misPath, Darkness::File::FILE_R));
        Darkness::FileGroupPtr db(new Darkness::DarkFileGroup(fp));

        if (!db->hasFile("FOG")) {
            std::fprintf(stderr, "No FOG chunk — fog disabled\n");
            return fog;
        }

        Darkness::FilePtr chunk = db->getFile("FOG");

        int32_t red, green, blue;
        float distance;
        chunk->readElem(&red, sizeof(red));
        chunk->readElem(&green, sizeof(green));
        chunk->readElem(&blue, sizeof(blue));
        chunk->readElem(&distance, sizeof(distance));

        if (distance > 0.0f) {
            fog.enabled = true;
            fog.r = static_cast<float>(red) / 255.0f;
            fog.g = static_cast<float>(green) / 255.0f;
            fog.b = static_cast<float>(blue) / 255.0f;
            fog.distance = distance;
            std::fprintf(stderr, "FOG: color=(%d,%d,%d) distance=%.1f\n",
                         red, green, blue, distance);
        } else {
            std::fprintf(stderr, "FOG: chunk present but distance=0 — fog disabled\n");
        }
    } catch (const std::exception &e) {
        std::fprintf(stderr, "Failed to read FOG: %s (fog disabled)\n", e.what());
    }

    return fog;
}

// ── Water flow data parsing ──

// Parse FLOW_TEX and CELL_MOTION chunks from a .mis file.
// FLOW_TEX: 256 × 32-byte entries mapping flow group to water texture names.
// CELL_MOTION: two sequential arrays — PortalCellMotion[256] (state) then
// MedMoCellMotion[256] (velocities). The velocities give the per-second scroll
// and rotation rates for each flow group, as set by the level designer.
inline FlowData parseFlowData(const char *misPath) {
    FlowData data = {};

    try {
        Darkness::FilePtr fp(new Darkness::StdFile(misPath, Darkness::File::FILE_R));
        Darkness::FileGroupPtr db(new Darkness::DarkFileGroup(fp));

        // Parse FLOW_TEX: 256 × 32 bytes = 8192 bytes
        if (db->hasFile("FLOW_TEX")) {
            Darkness::FilePtr chunk = db->getFile("FLOW_TEX");
            for (int i = 0; i < MAX_FLOW_GROUPS; ++i) {
                chunk->readElem(&data.textures[i], sizeof(FlowTexEntry));
            }
            data.hasFlowTex = true;

            // Log flow groups with non-empty texture names
            int namedCount = 0;
            for (int i = 1; i < MAX_FLOW_GROUPS; ++i) {
                if (data.textures[i].name[0] != '\0') {
                    char safeName[29] = {};
                    std::memcpy(safeName, data.textures[i].name, 28);
                    std::fprintf(stderr, "  flow group %d: name='%s' (palette idx in=%d out=%d)\n",
                                 i, safeName, data.textures[i].inTexture, data.textures[i].outTexture);
                    ++namedCount;
                }
            }
            std::fprintf(stderr, "FLOW_TEX: %d named flow groups\n", namedCount);
        } else {
            std::fprintf(stderr, "No FLOW_TEX chunk — water textures from polygon data only\n");
        }

        // Parse CELL_MOTION: PortalCellMotion[256] (21 bytes each) then
        // MedMoCellMotion[256] (14 bytes each) = 256*(21+14) = 8960 bytes
        if (db->hasFile("CELL_MOTION")) {
            Darkness::FilePtr chunk = db->getFile("CELL_MOTION");
            // First array: current state
            for (int i = 0; i < MAX_FLOW_GROUPS; ++i) {
                chunk->readElem(&data.state[i], sizeof(PortalCellMotion));
            }
            // Second array: velocities (scroll + rotation speeds)
            for (int i = 0; i < MAX_FLOW_GROUPS; ++i) {
                chunk->readElem(&data.velocity[i], sizeof(MedMoCellMotion));
            }
            data.hasCellMotion = true;

            // Log flow groups with non-zero state (center or angle set by the level)
            int motionCount = 0;
            for (int i = 1; i < MAX_FLOW_GROUPS; ++i) {
                const auto &st = data.state[i];
                const auto &vel = data.velocity[i];
                bool hasState = (st.centerX != 0 || st.centerY != 0 || st.centerZ != 0 || st.angle != 0);
                bool hasVelocity = (vel.centerChangeX != 0 || vel.centerChangeY != 0 ||
                                    vel.centerChangeZ != 0 || vel.angleChange != 0);
                if (hasState || hasVelocity) {
                    float angDeg = st.angle * 360.0f / 65536.0f;
                    float angVelDeg = static_cast<float>(static_cast<int16_t>(vel.angleChange))
                                      * 360.0f / 65536.0f;
                    std::fprintf(stderr, "  motion %d: center=(%.1f,%.1f,%.1f) angle=%.1f° "
                                 "vel=(%.2f,%.2f,%.2f)/s rot=%.1f°/s axis=%d\n",
                                 i, st.centerX, st.centerY, st.centerZ, angDeg,
                                 vel.centerChangeX, vel.centerChangeY, vel.centerChangeZ,
                                 angVelDeg, st.majorAxis);
                    ++motionCount;
                }
            }
            std::fprintf(stderr, "CELL_MOTION: %d groups with non-zero state/velocity\n", motionCount);
        } else {
            std::fprintf(stderr, "No CELL_MOTION chunk — using default water animation\n");
        }
    } catch (const std::exception &e) {
        std::fprintf(stderr, "Failed to read flow data: %s\n", e.what());
    }

    return data;
}

// ── Sky dome ──

// Sky rendering parameters — either from SKYOBJVAR chunk or defaults
struct SkyParams {
    bool enabled;           // useNewSky from SKYOBJVAR (true if chunk present)
    bool fog;               // whether the sky should be affected by fog
    int numLatPoints;       // latitude rings (default 8)
    int numLonPoints;       // longitude segments (default 24)
    float dipAngle;         // how far below horizon to extend (degrees, default 10)

    // 5-stop vertex colour gradient (pole → dip below horizon)
    // Each stop is {r, g, b} in 0..1 range
    float poleColor[3];           // zenith (straight up)
    float fortyFiveColor[3];      // 45 degrees from pole
    float seventyColor[3];        // 70 degrees from pole
    float horizonColor[3];        // at the horizon (90 degrees)
    float dipColor[3];            // below horizon
};

// Default sky colours — dark blue-grey for Thief 2's nighttime atmosphere
inline SkyParams defaultSkyParams() {
    SkyParams p;
    p.enabled = true;
    p.fog = true;  // default: sky affected by fog (overridden by SKYOBJVAR if present)
    p.numLatPoints = 8;
    p.numLonPoints = 24;
    p.dipAngle = 10.0f;

    // Deep blue zenith fading to grey horizon
    p.poleColor[0]       = 0.05f; p.poleColor[1]       = 0.05f; p.poleColor[2]       = 0.20f;
    p.fortyFiveColor[0]  = 0.10f; p.fortyFiveColor[1]  = 0.15f; p.fortyFiveColor[2]  = 0.35f;
    p.seventyColor[0]    = 0.30f; p.seventyColor[1]    = 0.40f; p.seventyColor[2]    = 0.60f;
    p.horizonColor[0]    = 0.50f; p.horizonColor[1]    = 0.55f; p.horizonColor[2]    = 0.60f;
    p.dipColor[0]        = 0.25f; p.dipColor[1]        = 0.25f; p.dipColor[2]        = 0.30f;

    return p;
}

// Parse SKYOBJVAR chunk from a .mis file if present.
// Uses the DarkFileGroup chunk API to read the binary struct directly.
inline SkyParams parseSkyObjVar(const char *misPath) {
    SkyParams params = defaultSkyParams();

    try {
        Darkness::FilePtr fp(new Darkness::StdFile(misPath, Darkness::File::FILE_R));
        Darkness::FileGroupPtr db(new Darkness::DarkFileGroup(fp));

        if (!db->hasFile("SKYOBJVAR")) {
            std::fprintf(stderr, "No SKYOBJVAR chunk — using default sky colours\n");
            return params;
        }

        Darkness::FilePtr chunk = db->getFile("SKYOBJVAR");

        // Read the binary struct: matches DarkDBChunkSKYOBJVAR from DarkDBDefs.h
        // Fields: enable(4), fog(4), atmos_radius(4), earth_radius(4),
        //         lat_count(4), lon_count(4), dip_angle(4),
        //         5 × Color(12) = 60, glow_color(12),
        //         glow_lat(4), glow_lon(4), glow_ang(4), glow_scale(4),
        //         glow_method(4), clip_lat(4)
        // Total: 120 bytes
        uint32_t enable, fog;
        float atmosRadius, earthRadius;
        int32_t latCount, lonCount;
        float dipAngle;
        float colors[5][3]; // pole, 45, 70, horizon, dip
        float glowColor[3], glowLat, glowLon, glowAng, glowScale;
        uint32_t glowMethod;
        float clipLat;

        chunk->readElem(&enable, sizeof(enable));
        chunk->readElem(&fog, sizeof(fog));
        chunk->readElem(&atmosRadius, sizeof(atmosRadius));
        chunk->readElem(&earthRadius, sizeof(earthRadius));
        chunk->readElem(&latCount, sizeof(latCount));
        chunk->readElem(&lonCount, sizeof(lonCount));
        chunk->readElem(&dipAngle, sizeof(dipAngle));

        // Read 5 control colours (pole, 45, 70, horizon, dip)
        for (int i = 0; i < 5; ++i) {
            chunk->readElem(&colors[i][0], sizeof(float));
            chunk->readElem(&colors[i][1], sizeof(float));
            chunk->readElem(&colors[i][2], sizeof(float));
        }

        // Read glow parameters (not used yet but consume for completeness)
        chunk->readElem(&glowColor[0], sizeof(float));
        chunk->readElem(&glowColor[1], sizeof(float));
        chunk->readElem(&glowColor[2], sizeof(float));
        chunk->readElem(&glowLat, sizeof(float));
        chunk->readElem(&glowLon, sizeof(float));
        chunk->readElem(&glowAng, sizeof(float));
        chunk->readElem(&glowScale, sizeof(float));
        chunk->readElem(&glowMethod, sizeof(glowMethod));
        chunk->readElem(&clipLat, sizeof(float));

        if (enable) {
            params.enabled = true;
            params.fog = (fog != 0);
            params.numLatPoints = (latCount > 2 && latCount < 32) ? latCount : 8;
            params.numLonPoints = (lonCount > 4 && lonCount < 64) ? lonCount : 24;
            params.dipAngle = dipAngle;

            // Copy control colours
            std::memcpy(params.poleColor, colors[0], sizeof(float) * 3);
            std::memcpy(params.fortyFiveColor, colors[1], sizeof(float) * 3);
            std::memcpy(params.seventyColor, colors[2], sizeof(float) * 3);
            std::memcpy(params.horizonColor, colors[3], sizeof(float) * 3);
            std::memcpy(params.dipColor, colors[4], sizeof(float) * 3);

            std::fprintf(stderr, "SKYOBJVAR: lat=%d lon=%d dip=%.1f\n",
                         params.numLatPoints, params.numLonPoints, params.dipAngle);
        } else {
            std::fprintf(stderr, "SKYOBJVAR: new sky disabled, using defaults\n");
        }
    } catch (const std::exception &e) {
        std::fprintf(stderr, "Failed to read SKYOBJVAR: %s (using defaults)\n", e.what());
    }

    return params;
}

// Sky dome mesh data — a vertex-coloured hemisphere
struct SkyDome {
    std::vector<PosColorVertex> vertices;
    std::vector<uint16_t> indices;
};

// Build a vertex-coloured hemisphere for sky rendering.
// Latitude goes from 0 (pole/zenith) to 90+dipAngle degrees (below horizon).
// Colour is interpolated through 5 control stops matching Dark Engine's sky system.
inline SkyDome buildSkyDome(const SkyParams &sky) {
    SkyDome dome;

    const float PI = 3.14159265f;
    const float DEG2RAD = PI / 180.0f;

    int numLat = sky.numLatPoints;   // rings from pole to below-horizon
    int numLon = sky.numLonPoints;   // segments around

    // Total angular range: 0 (pole) to 90 + dipAngle (below horizon)
    float maxLat = (90.0f + sky.dipAngle) * DEG2RAD;

    // Radius — large enough that it feels infinitely far (no depth test anyway)
    const float radius = 1000.0f;

    // 5-stop gradient: pole(0°), 45°, 70°, 90°(horizon), 90+dip°
    // Stored as {latDegrees, r, g, b}
    struct ColorStop { float lat; float r, g, b; };
    ColorStop stops[5] = {
        {  0.0f, sky.poleColor[0],       sky.poleColor[1],       sky.poleColor[2]       },
        { 45.0f, sky.fortyFiveColor[0],  sky.fortyFiveColor[1],  sky.fortyFiveColor[2]  },
        { 70.0f, sky.seventyColor[0],    sky.seventyColor[1],    sky.seventyColor[2]    },
        { 90.0f, sky.horizonColor[0],    sky.horizonColor[1],    sky.horizonColor[2]    },
        { 90.0f + sky.dipAngle,
                  sky.dipColor[0],        sky.dipColor[1],        sky.dipColor[2]        },
    };

    // Interpolate colour at a given latitude (degrees)
    auto lerpColor = [&](float latDeg) -> uint32_t {
        // Find the surrounding stops
        int i = 0;
        for (; i < 4; ++i) {
            if (latDeg <= stops[i + 1].lat) break;
        }
        i = std::min(i, 3);

        float range = stops[i + 1].lat - stops[i].lat;
        float t = (range > 0.001f) ? (latDeg - stops[i].lat) / range : 0.0f;
        t = std::max(0.0f, std::min(1.0f, t));

        float r = stops[i].r + t * (stops[i + 1].r - stops[i].r);
        float g = stops[i].g + t * (stops[i + 1].g - stops[i].g);
        float b = stops[i].b + t * (stops[i + 1].b - stops[i].b);
        return packABGR(r, g, b);
    };

    // Vertex 0: pole (zenith, Z-up)
    dome.vertices.push_back({ 0.0f, 0.0f, radius, lerpColor(0.0f) });

    // Latitude rings: numLat-1 rings between pole and max latitude
    for (int lat = 1; lat < numLat; ++lat) {
        float latFrac = static_cast<float>(lat) / static_cast<float>(numLat - 1);
        float latRad = latFrac * maxLat;
        float latDeg = latFrac * (90.0f + sky.dipAngle);

        float sinLat = std::sin(latRad);
        float cosLat = std::cos(latRad);

        uint32_t color = lerpColor(latDeg);

        for (int lon = 0; lon < numLon; ++lon) {
            float lonRad = static_cast<float>(lon) / static_cast<float>(numLon) * 2.0f * PI;
            float x = radius * sinLat * std::cos(lonRad);
            float y = radius * sinLat * std::sin(lonRad);
            float z = radius * cosLat;
            dome.vertices.push_back({ x, y, z, color });
        }
    }

    // Indices — pole fan (vertex 0 connects to first ring)
    for (int lon = 0; lon < numLon; ++lon) {
        int next = (lon + 1) % numLon;
        dome.indices.push_back(0);
        dome.indices.push_back(static_cast<uint16_t>(1 + lon));
        dome.indices.push_back(static_cast<uint16_t>(1 + next));
    }

    // Quad strips between consecutive latitude rings
    for (int lat = 1; lat < numLat - 1; ++lat) {
        int ringA = 1 + (lat - 1) * numLon;
        int ringB = 1 + lat * numLon;

        for (int lon = 0; lon < numLon; ++lon) {
            int next = (lon + 1) % numLon;

            // Two triangles per quad
            dome.indices.push_back(static_cast<uint16_t>(ringA + lon));
            dome.indices.push_back(static_cast<uint16_t>(ringB + lon));
            dome.indices.push_back(static_cast<uint16_t>(ringB + next));

            dome.indices.push_back(static_cast<uint16_t>(ringA + lon));
            dome.indices.push_back(static_cast<uint16_t>(ringB + next));
            dome.indices.push_back(static_cast<uint16_t>(ringA + next));
        }
    }

    std::fprintf(stderr, "Sky dome: %zu vertices, %zu indices (%zu triangles)\n",
                 dome.vertices.size(), dome.indices.size(), dome.indices.size() / 3);

    return dome;
}

// ── Textured skybox (old sky system) ──

// Per-face draw range within the skybox cube's index buffer
struct SkyboxFace {
    std::string key;       // "n", "s", "e", "w", "t"
    uint32_t firstIndex;
    uint32_t indexCount;   // always 6
};

// Skybox cube mesh — 5 textured quads (N/S/E/W/T) viewed from inside
struct SkyboxCube {
    std::vector<PosColorUVVertex> vertices;
    std::vector<uint16_t> indices;
    std::vector<SkyboxFace> faces;
};

// Build 5 textured quads for the skybox cube.
// Each face is a quad with UV-mapped texture, white vertex colour (texture
// appears unmodified), and CW winding from inside for CULL_CCW compatibility.
// Z-up coordinate system: N=-X, S=+X, E=+Y, W=-Y, T=+Z.
inline SkyboxCube buildSkyboxCube() {
    SkyboxCube cube;

    const float h = 500.0f; // half-size — arbitrary, no depth test
    const uint32_t white = 0xFFFFFFFFu; // ABGR white

    // Each face: 4 vertices (TL, TR, BL, BR) + 6 indices (2 triangles)
    // Winding order: TL→BL→TR, TR→BL→BR (CW from inside)
    struct FaceDef {
        const char *key;
        float verts[4][3]; // TL, TR, BL, BR positions
    };

    // Face vertex definitions — CW winding from inside the cube.
    // Dark Engine skybox camera angles per face:
    //   'n' heading=180° → -X wall, 'e' heading=90° → +Y wall,
    //   's' heading=0°   → +X wall, 'w' heading=270° → -Y wall,
    //   't' pitch=-90°   → +Z wall (top)
    // UV orientation: u increases in camera-right direction, v increases downward.
    // Camera right at each heading from our viewer's Camera::move():
    //   heading=0°: right=-Y, heading=90°: right=+X,
    //   heading=180°: right=+Y, heading=270°: right=-X
    const int NUM_FACES = 5;
    FaceDef faceDefs[NUM_FACES];

    // North (-X wall): heading=180°, camera right=+Y, up=+Z
    // Screen: left=-Y, right=+Y, top=+Z, bottom=-Z
    faceDefs[0].key = "n";
    faceDefs[0].verts[0][0] = -h; faceDefs[0].verts[0][1] = -h; faceDefs[0].verts[0][2] =  h; // TL
    faceDefs[0].verts[1][0] = -h; faceDefs[0].verts[1][1] =  h; faceDefs[0].verts[1][2] =  h; // TR
    faceDefs[0].verts[2][0] = -h; faceDefs[0].verts[2][1] = -h; faceDefs[0].verts[2][2] = -h; // BL
    faceDefs[0].verts[3][0] = -h; faceDefs[0].verts[3][1] =  h; faceDefs[0].verts[3][2] = -h; // BR

    // East (+Y wall): heading=90°, camera right=+X, up=+Z
    // Screen: left=-X, right=+X, top=+Z, bottom=-Z
    faceDefs[1].key = "e";
    faceDefs[1].verts[0][0] = -h; faceDefs[1].verts[0][1] =  h; faceDefs[1].verts[0][2] =  h; // TL
    faceDefs[1].verts[1][0] =  h; faceDefs[1].verts[1][1] =  h; faceDefs[1].verts[1][2] =  h; // TR
    faceDefs[1].verts[2][0] = -h; faceDefs[1].verts[2][1] =  h; faceDefs[1].verts[2][2] = -h; // BL
    faceDefs[1].verts[3][0] =  h; faceDefs[1].verts[3][1] =  h; faceDefs[1].verts[3][2] = -h; // BR

    // South (+X wall): heading=0°, camera right=-Y, up=+Z
    // Screen: left=+Y, right=-Y, top=+Z, bottom=-Z
    faceDefs[2].key = "s";
    faceDefs[2].verts[0][0] =  h; faceDefs[2].verts[0][1] =  h; faceDefs[2].verts[0][2] =  h; // TL
    faceDefs[2].verts[1][0] =  h; faceDefs[2].verts[1][1] = -h; faceDefs[2].verts[1][2] =  h; // TR
    faceDefs[2].verts[2][0] =  h; faceDefs[2].verts[2][1] =  h; faceDefs[2].verts[2][2] = -h; // BL
    faceDefs[2].verts[3][0] =  h; faceDefs[2].verts[3][1] = -h; faceDefs[2].verts[3][2] = -h; // BR

    // West (-Y wall): heading=270°, camera right=-X, up=+Z
    // Screen: left=+X, right=-X, top=+Z, bottom=-Z
    faceDefs[3].key = "w";
    faceDefs[3].verts[0][0] =  h; faceDefs[3].verts[0][1] = -h; faceDefs[3].verts[0][2] =  h; // TL
    faceDefs[3].verts[1][0] = -h; faceDefs[3].verts[1][1] = -h; faceDefs[3].verts[1][2] =  h; // TR
    faceDefs[3].verts[2][0] =  h; faceDefs[3].verts[2][1] = -h; faceDefs[3].verts[2][2] = -h; // BL
    faceDefs[3].verts[3][0] = -h; faceDefs[3].verts[3][1] = -h; faceDefs[3].verts[3][2] = -h; // BR

    // Top (+Z wall): heading=90° then pitch up 90°, camera right=+X
    // Screen top = -Y (what was behind when facing +Y), bottom = +Y
    faceDefs[4].key = "t";
    faceDefs[4].verts[0][0] = -h; faceDefs[4].verts[0][1] = -h; faceDefs[4].verts[0][2] =  h; // TL
    faceDefs[4].verts[1][0] =  h; faceDefs[4].verts[1][1] = -h; faceDefs[4].verts[1][2] =  h; // TR
    faceDefs[4].verts[2][0] = -h; faceDefs[4].verts[2][1] =  h; faceDefs[4].verts[2][2] =  h; // BL
    faceDefs[4].verts[3][0] =  h; faceDefs[4].verts[3][1] =  h; faceDefs[4].verts[3][2] =  h; // BR

    for (const auto &fd : faceDefs) {
        SkyboxFace face;
        face.key = fd.key;
        face.firstIndex = static_cast<uint32_t>(cube.indices.size());
        face.indexCount = 6;

        uint16_t base = static_cast<uint16_t>(cube.vertices.size());

        // TL(0,0), TR(1,0), BL(0,1), BR(1,1)
        cube.vertices.push_back({ fd.verts[0][0], fd.verts[0][1], fd.verts[0][2], white, 0.0f, 0.0f });
        cube.vertices.push_back({ fd.verts[1][0], fd.verts[1][1], fd.verts[1][2], white, 1.0f, 0.0f });
        cube.vertices.push_back({ fd.verts[2][0], fd.verts[2][1], fd.verts[2][2], white, 0.0f, 1.0f });
        cube.vertices.push_back({ fd.verts[3][0], fd.verts[3][1], fd.verts[3][2], white, 1.0f, 1.0f });

        // Two triangles (CW from inside — matches dome convention):
        // TL→TR→BL, TR→BR→BL
        cube.indices.push_back(base + 0);
        cube.indices.push_back(base + 1);
        cube.indices.push_back(base + 2);

        cube.indices.push_back(base + 1);
        cube.indices.push_back(base + 3);
        cube.indices.push_back(base + 2);

        cube.faces.push_back(std::move(face));
    }

    std::fprintf(stderr, "Skybox cube: %zu vertices, %zu indices (%zu faces)\n",
                 cube.vertices.size(), cube.indices.size(), cube.faces.size());

    return cube;
}

} // namespace Darkness
