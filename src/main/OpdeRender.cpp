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

// World geometry viewer with baked lightmaps and object mesh rendering

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <vector>
#include <string>
#include <algorithm>
#include <chrono>
#include <unordered_map>
#include <unordered_set>

#include <SDL.h>
#include <SDL_syswm.h>

#include <bgfx/bgfx.h>
#include <bgfx/platform.h>
#include <bx/math.h>

#include "../shaders/basic_shader.h"
#include "../shaders/textured_shader.h"
#include "../shaders/lightmapped_shader.h"
#include "WRChunkParser.h"
#include "TXListParser.h"
#include "CRFTextureLoader.h"
#include "CRFModelLoader.h"
#include "LightmapAtlas.h"
#include "SpawnFinder.h"
#include "LightingSystem.h"
#include "ObjectPropParser.h"
#include "BinMeshParser.h"

static const int WINDOW_WIDTH  = 1280;
static const int WINDOW_HEIGHT = 720;

// ── Vertex layouts ──

struct PosColorVertex {
    float x, y, z;
    uint32_t abgr;

    static bgfx::VertexLayout layout;

    static void init() {
        layout.begin()
            .add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
            .add(bgfx::Attrib::Color0, 4, bgfx::AttribType::Uint8, true)
            .end();
    }
};

bgfx::VertexLayout PosColorVertex::layout;

struct PosColorUVVertex {
    float x, y, z;
    uint32_t abgr;
    float u, v;

    static bgfx::VertexLayout layout;

    static void init() {
        layout.begin()
            .add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
            .add(bgfx::Attrib::Color0, 4, bgfx::AttribType::Uint8, true)
            .add(bgfx::Attrib::TexCoord0, 2, bgfx::AttribType::Float)
            .end();
    }
};

bgfx::VertexLayout PosColorUVVertex::layout;

struct PosUV2Vertex {
    float x, y, z;
    float u0, v0; // diffuse texture UV
    float u1, v1; // lightmap atlas UV

    static bgfx::VertexLayout layout;

    static void init() {
        layout.begin()
            .add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
            .add(bgfx::Attrib::TexCoord0, 2, bgfx::AttribType::Float)
            .add(bgfx::Attrib::TexCoord1, 2, bgfx::AttribType::Float)
            .end();
    }
};

bgfx::VertexLayout PosUV2Vertex::layout;

// ── Texture group for batched draw calls ──

struct TextureGroup {
    uint8_t txtIndex;
    uint32_t firstIndex;
    uint32_t numIndices;
};

// ── World mesh (textured) ──

struct WorldMesh {
    std::vector<PosColorUVVertex> vertices;
    std::vector<uint32_t> indices;
    std::vector<TextureGroup> groups; // sorted by texture index
    float cx, cy, cz;
};

// ── Flat-only mesh (fallback) ──

struct FlatMesh {
    std::vector<PosColorVertex> vertices;
    std::vector<uint32_t> indices;
    float cx, cy, cz;
};

struct LightmappedMesh {
    std::vector<PosUV2Vertex> vertices;
    std::vector<uint32_t> indices;
    std::vector<TextureGroup> groups;
    float cx, cy, cz;
};

static uint32_t packABGR(float r, float g, float b) {
    uint8_t ri = static_cast<uint8_t>(std::min(std::max(r, 0.0f), 1.0f) * 255.0f);
    uint8_t gi = static_cast<uint8_t>(std::min(std::max(g, 0.0f), 1.0f) * 255.0f);
    uint8_t bi = static_cast<uint8_t>(std::min(std::max(b, 0.0f), 1.0f) * 255.0f);
    return 0xff000000u | (bi << 16) | (gi << 8) | ri;
}

// ── Build flat-shaded mesh (no textures) ──

static FlatMesh buildFlatMesh(const Opde::WRParsedData &wr) {
    FlatMesh mesh;

    // Sun direction for Lambertian shading
    float sunDir[3] = { 0.3f, 0.8f, 0.4f };
    float sunLen = std::sqrt(sunDir[0]*sunDir[0] + sunDir[1]*sunDir[1] + sunDir[2]*sunDir[2]);
    sunDir[0] /= sunLen; sunDir[1] /= sunLen; sunDir[2] /= sunLen;

    float sumX = 0, sumY = 0, sumZ = 0;
    int cellCount = 0;

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
            Opde::Plane plane;
            if (poly.plane < cell.planes.size())
                plane = cell.planes[poly.plane];

            Opde::Vector3 n = plane.normal.normalisedCopy();
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

            // Fan-triangulate: (0, t+1, t) — matches OPDE WRCell.cpp winding
            for (int t = 1; t < poly.count - 1; ++t) {
                mesh.indices.push_back(baseVertex);
                mesh.indices.push_back(baseVertex + t + 1);
                mesh.indices.push_back(baseVertex + t);
            }
        }
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

// ── Build textured mesh with UV coordinates ──

struct TexDimensions {
    uint32_t w, h;
};

static WorldMesh buildTexturedMesh(const Opde::WRParsedData &wr,
                                   const std::unordered_map<uint8_t, TexDimensions> &texDims) {
    WorldMesh mesh;

    // Sun direction for Lambertian shading
    float sunDir[3] = { 0.3f, 0.8f, 0.4f };
    float sunLen = std::sqrt(sunDir[0]*sunDir[0] + sunDir[1]*sunDir[1] + sunDir[2]*sunDir[2]);
    sunDir[0] /= sunLen; sunDir[1] /= sunLen; sunDir[2] /= sunLen;

    float sumX = 0, sumY = 0, sumZ = 0;
    int cellCount = 0;

    // Temporary per-texture triangle lists; key = texture index
    std::unordered_map<uint8_t, std::vector<uint32_t>> texTriangles;

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

            Opde::Plane plane;
            if (poly.plane < cell.planes.size())
                plane = cell.planes[poly.plane];

            Opde::Vector3 n = plane.normal.normalisedCopy();
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
                Opde::Vector3 origin(0, 0, 0);
                if (tex.originVertex < indices.size()) {
                    uint8_t oi = indices[tex.originVertex];
                    if (oi < cell.vertices.size())
                        origin = cell.vertices[oi];
                }

                float mag2_u = tex.axisU.squaredLength();
                float mag2_v = tex.axisV.squaredLength();
                float sh_u = tex.u / 4096.0f;
                float sh_v = tex.v / 4096.0f;
                float dotp = tex.axisU.dotProduct(tex.axisV);

                for (int vi = 0; vi < poly.count; ++vi) {
                    uint8_t idx = indices[vi];
                    if (idx >= cell.vertices.size()) continue;
                    const auto &coord = cell.vertices[idx];

                    Opde::Vector3 tmp = coord - origin;
                    float u, v;

                    if (std::abs(dotp) < 1e-6f) {
                        // Orthogonal axes — direct projection
                        u = tex.axisU.dotProduct(tmp) / mag2_u + sh_u;
                        v = tex.axisV.dotProduct(tmp) / mag2_v + sh_v;
                    } else {
                        // Non-orthogonal correction (WRCell.cpp:196-203)
                        float corr = 1.0f / (mag2_u * mag2_v - dotp * dotp);
                        float cu = corr * mag2_v;
                        float cv = corr * mag2_u;
                        float cross = corr * dotp;
                        float pu = tex.axisU.dotProduct(tmp);
                        float pv = tex.axisV.dotProduct(tmp);
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

            // Fan-triangulate
            uint8_t key = isTextured ? txtIdx : 0; // 0 = flat group
            auto &triList = texTriangles[key];
            for (int t = 1; t < poly.count - 1; ++t) {
                triList.push_back(baseVertex);
                triList.push_back(baseVertex + t + 1);
                triList.push_back(baseVertex + t);
            }
        }
    }

    // Build sorted index buffer: all groups sorted by texture index
    // Put flat group (key=0) first
    std::vector<uint8_t> sortedKeys;
    for (auto &kv : texTriangles)
        sortedKeys.push_back(kv.first);
    std::sort(sortedKeys.begin(), sortedKeys.end());

    for (uint8_t key : sortedKeys) {
        auto &triList = texTriangles[key];
        TextureGroup grp;
        grp.txtIndex = key;
        grp.firstIndex = static_cast<uint32_t>(mesh.indices.size());
        grp.numIndices = static_cast<uint32_t>(triList.size());
        mesh.groups.push_back(grp);
        mesh.indices.insert(mesh.indices.end(), triList.begin(), triList.end());
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

static LightmappedMesh buildLightmappedMesh(
    const Opde::WRParsedData &wr,
    const std::unordered_map<uint8_t, TexDimensions> &texDims,
    const Opde::LightmapAtlasSet &lmAtlas)
{
    LightmappedMesh mesh;

    float sumX = 0, sumY = 0, sumZ = 0;
    int cellCount = 0;

    std::unordered_map<uint8_t, std::vector<uint32_t>> texTriangles;

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
                Opde::Vector3 origin(0, 0, 0);
                if (tex.originVertex < indices.size()) {
                    uint8_t oi = indices[tex.originVertex];
                    if (oi < cell.vertices.size())
                        origin = cell.vertices[oi];
                }

                float mag2_u = tex.axisU.squaredLength();
                float mag2_v = tex.axisV.squaredLength();
                float sh_u = tex.u / 4096.0f;
                float sh_v = tex.v / 4096.0f;
                float dotp = tex.axisU.dotProduct(tex.axisV);

                // Lightmap shift and entry (if available)
                const Opde::WRLightInfo *li = nullptr;
                const Opde::LmapEntry *lmEntry = nullptr;
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

                    Opde::Vector3 tmp = coord - origin;
                    float pu = tex.axisU.dotProduct(tmp);
                    float pv = tex.axisV.dotProduct(tmp);
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
                    const Opde::LmapEntry &fallback = (ci < lmAtlas.entries.size() && !lmAtlas.entries[ci].empty())
                        ? lmAtlas.entries[ci][0] : Opde::LmapEntry{0, 0, 0, 0, 0, 0, 0, 0, 0};

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

            uint8_t key = isTextured ? txtIdx : 0;
            auto &triList = texTriangles[key];
            for (int t = 1; t < poly.count - 1; ++t) {
                triList.push_back(baseVertex);
                triList.push_back(baseVertex + t + 1);
                triList.push_back(baseVertex + t);
            }
        }
    }

    // Build sorted index buffer
    std::vector<uint8_t> sortedKeys;
    for (auto &kv : texTriangles)
        sortedKeys.push_back(kv.first);
    std::sort(sortedKeys.begin(), sortedKeys.end());

    for (uint8_t key : sortedKeys) {
        auto &triList = texTriangles[key];
        TextureGroup grp;
        grp.txtIndex = key;
        grp.firstIndex = static_cast<uint32_t>(mesh.indices.size());
        grp.numIndices = static_cast<uint32_t>(triList.size());
        mesh.groups.push_back(grp);
        mesh.indices.insert(mesh.indices.end(), triList.begin(), triList.end());
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

// ── Object mesh rendering ──

// Per-material submesh draw range within an object model's index buffer
struct ObjectSubMeshGPU {
    uint32_t firstIndex;
    uint32_t indexCount;
    std::string matName;   // lowercase material name, for texture lookup
    bool textured;         // true = Opde::MD_MAT_TMAP, false = Opde::MD_MAT_COLOR
};

// GPU buffers for a single unique model (shared by all instances)
struct ObjectModelGPU {
    bgfx::VertexBufferHandle vbh = BGFX_INVALID_HANDLE;
    bgfx::IndexBufferHandle ibh = BGFX_INVALID_HANDLE;
    std::vector<ObjectSubMeshGPU> subMeshes;
    bool valid = false;
};

// Generate a stable colour from a model name hash (for flat-shaded objects)
static uint32_t colorFromName(const std::string &name) {
    // Simple hash -> hue mapping for distinct per-model colours
    uint32_t hash = 5381;
    for (char c : name) hash = hash * 33 + static_cast<uint8_t>(c);

    // HSV -> RGB with S=0.6, V=0.85 for pleasant pastel tones
    float hue = static_cast<float>(hash % 360);
    float s = 0.6f, v = 0.85f;
    float c = v * s;
    float x = c * (1.0f - std::fabs(std::fmod(hue / 60.0f, 2.0f) - 1.0f));
    float m = v - c;
    float r, g, b;
    if (hue < 60)       { r = c; g = x; b = 0; }
    else if (hue < 120) { r = x; g = c; b = 0; }
    else if (hue < 180) { r = 0; g = c; b = x; }
    else if (hue < 240) { r = 0; g = x; b = c; }
    else if (hue < 300) { r = x; g = 0; b = c; }
    else                { r = c; g = 0; b = x; }
    return packABGR(r + m, g + m, b + m);
}

// Build a small unit cube as PosColorVertex (fallback for missing models)
static void buildFallbackCube(std::vector<PosColorVertex> &verts,
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

// ── Sky dome ──

// Sky rendering parameters — either from SKYOBJVAR chunk or defaults
struct SkyParams {
    bool enabled;           // useNewSky from SKYOBJVAR (true if chunk present)
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
static SkyParams defaultSkyParams() {
    SkyParams p;
    p.enabled = true;
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
static SkyParams parseSkyObjVar(const char *misPath) {
    SkyParams params = defaultSkyParams();

    try {
        Opde::FilePtr fp(new Opde::StdFile(misPath, Opde::File::FILE_R));
        Opde::FileGroupPtr db(new Opde::DarkFileGroup(fp));

        if (!db->hasFile("SKYOBJVAR")) {
            std::fprintf(stderr, "No SKYOBJVAR chunk — using default sky colours\n");
            return params;
        }

        Opde::FilePtr chunk = db->getFile("SKYOBJVAR");

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
static SkyDome buildSkyDome(const SkyParams &sky) {
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
static SkyboxCube buildSkyboxCube() {
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

// Build a 4x4 model matrix from position + binary-radian angles.
// Dark Engine uses heading/pitch/bank in binary radians (65536 = 360 degrees).
//
// Dark Engine coordinate system (Z-up):
//   heading = rotation around Z axis  (angvec.tz)
//   pitch   = rotation around Y axis  (angvec.ty)
//   bank    = rotation around X axis  (angvec.tx)
//
// Dark Engine rotation order: R = Rx(bank) * Ry(pitch) * Rz(heading)
// bx::mtxRotateXYZ(ax, ay, az) = Rx(ax) * Ry(ay) * Rz(az) — same order.
static void buildModelMatrix(float *mtx, float x, float y, float z,
                              int16_t heading, int16_t pitch, int16_t bank,
                              float sx = 1.0f, float sy = 1.0f, float sz = 1.0f) {
    const float angScale = 2.0f * 3.14159265f / 65536.0f;

    float h = static_cast<float>(heading) * angScale;
    float p = static_cast<float>(pitch)   * angScale;
    float b = static_cast<float>(bank)    * angScale;

    // Build rotation matrix matching the Dark Engine rotation convention:
    //   M_DE = Rz(heading) * Ry(pitch) * Rx(bank)  (column-vector, column-major)
    // where bank=X-rot, pitch=Y-rot, heading=Z-rot (Z-up coordinate system).
    // bx stores row-major; Metal reads as column-major (= transpose), which
    // effectively reverses the rotation direction. Negating all angles
    // compensates, producing the correct Dark Engine rotation on the GPU.
    bx::mtxRotateXYZ(mtx, -b, -p, -h);

    // Apply scale in local model space: M = T * R * S
    // bx uses row-vector convention (v * M), so to apply model-space scale
    // before rotation, we multiply: M_bx = Scale_rows * R_bx.
    // After bgfx's transposition for the GPU (column-vector: M^T * v),
    // this becomes R_gpu * S * v — scale first, then rotate. Correct.
    mtx[ 0] *= sx;  mtx[ 1] *= sx;  mtx[ 2] *= sx;  // row 0 *= sx
    mtx[ 4] *= sy;  mtx[ 5] *= sy;  mtx[ 6] *= sy;  // row 1 *= sy
    mtx[ 8] *= sz;  mtx[ 9] *= sz;  mtx[10] *= sz;  // row 2 *= sz

    mtx[12] = x;
    mtx[13] = y;
    mtx[14] = z;
}

// ── Fly Camera ──

struct Camera {
    float pos[3];
    float yaw;
    float pitch;

    void init(float x, float y, float z) {
        pos[0] = x; pos[1] = y; pos[2] = z;
        yaw = 0; pitch = 0;
    }

    void getViewMatrix(float *mtx) const {
        // Dark Engine uses Z-up: X=right, Y=forward, Z=up.
        // Forward direction from yaw (heading about Z) and pitch (tilt up/down).
        // At yaw=0: forward = +X (Dark Engine convention — identity rotation faces +X).
        float cosPitch = std::cos(pitch);
        float fwd[3] = {
            std::cos(yaw) * cosPitch,   // X
            std::sin(yaw) * cosPitch,   // Y
            std::sin(pitch)              // Z (up/down tilt)
        };

        float at[3] = { pos[0] + fwd[0], pos[1] + fwd[1], pos[2] + fwd[2] };

        bx::Vec3 eye = { pos[0], pos[1], pos[2] };
        bx::Vec3 target = { at[0], at[1], at[2] };
        bx::Vec3 upV = { 0.0f, 0.0f, 1.0f };  // Z is up in Dark Engine

        bx::mtxLookAt(mtx, eye, target, upV, bx::Handedness::Right);
    }

    // Sky view matrix: rotation only, no translation.
    // Sky dome follows camera rotation but appears infinitely far away.
    void getSkyViewMatrix(float *mtx) const {
        float cosPitch = std::cos(pitch);
        float fwd[3] = {
            std::cos(yaw) * cosPitch,
            std::sin(yaw) * cosPitch,
            std::sin(pitch)
        };

        // Look from origin — sky dome is centered at (0,0,0)
        bx::Vec3 eye = { 0.0f, 0.0f, 0.0f };
        bx::Vec3 target = { fwd[0], fwd[1], fwd[2] };
        bx::Vec3 upV = { 0.0f, 0.0f, 1.0f };

        bx::mtxLookAt(mtx, eye, target, upV, bx::Handedness::Right);
    }

    void move(float forward, float right, float up) {
        // Dark Engine Z-up: yaw rotates in XY plane, Z is vertical.
        float cosPitch = std::cos(pitch);
        float fwd[3] = {
            std::cos(yaw) * cosPitch,   // X
            std::sin(yaw) * cosPitch,   // Y
            std::sin(pitch)              // Z
        };
        // Right vector: forward × up in right-handed Z-up = -Y at yaw=0
        float rt[3] = {
            std::sin(yaw),   // X
           -std::cos(yaw),   // Y
            0.0f              // Z (horizontal movement only)
        };

        pos[0] += fwd[0] * forward + rt[0] * right;
        pos[1] += fwd[1] * forward + rt[1] * right;
        pos[2] += fwd[2] * forward + up;  // Z is vertical
    }
};

static void printHelp() {
    std::fprintf(stderr,
        "opdeRender — Dark Engine world geometry viewer\n"
        "\n"
        "Usage:\n"
        "  opdeRender <mission.mis> [--res <path>] [--lm-scale <N>]\n"
        "\n"
        "Options:\n"
        "  --res <path>   Path to Thief 2 RES directory containing fam.crf.\n"
        "                 Enables lightmapped+textured rendering. Without this\n"
        "                 flag, geometry is rendered with flat Lambertian shading.\n"
        "  --lm-scale <N> Lightmap upscale factor (1-8, default 1).\n"
        "                 1 = vintage (original blocky lightmaps).\n"
        "                 2/4/8 = progressively smoother shadows via bicubic\n"
        "                 interpolation. Higher values use more atlas memory.\n"
        "  --no-objects   Disable object mesh rendering (world geometry only).\n"
        "  --force-flicker Force all animated lights to flicker mode (debug).\n"
        "  --help         Show this help message.\n"
        "\n"
        "Controls:\n"
        "  WASD           Move forward/left/back/right\n"
        "  Mouse          Look around\n"
        "  Space/LShift   Move up/down\n"
        "  Q/E            Move up/down (alternate)\n"
        "  Ctrl           Sprint (3x speed)\n"
        "  Scroll wheel   Adjust movement speed (shown in title bar)\n"
        "  Home           Teleport to player spawn point\n"
        "  Esc            Quit\n"
        "\n"
        "Resource setup:\n"
        "  The --res path should point to a directory containing fam.crf, which\n"
        "  holds the PCX textures used by Dark Engine levels. This can come from:\n"
        "\n"
        "  1. Mounted ISO (macOS):\n"
        "     hdiutil mount ../disk_images/thief_2_disk_1.iso\n"
        "     --res /Volumes/THIEF2_INSTALL_C/THIEF2/RES\n"
        "\n"
        "  2. GOG/Steam install directory:\n"
        "     --res /path/to/Thief2/RES\n"
        "\n"
        "  3. Any directory containing fam.crf\n"
        "\n"
        "Examples:\n"
        "  # Flat-shaded (no external resources needed):\n"
        "  opdeRender path/to/miss6.mis\n"
        "\n"
        "  # Textured, using mounted Thief 2 disc:\n"
        "  opdeRender path/to/miss6.mis --res /Volumes/THIEF2_INSTALL_C/THIEF2/RES\n"
        "\n"
        "  # Textured, using GOG install:\n"
        "  opdeRender path/to/miss6.mis --res ~/GOG/Thief2/RES\n"
    );
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        printHelp();
        return 1;
    }

    const char *misPath = argv[1];
    std::string resPath;
    int lmScale = 1;
    bool showObjects = true;
    bool forceFlicker = false;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            printHelp();
            return 0;
        }
        if (std::strcmp(argv[i], "--res") == 0 && i + 1 < argc) {
            resPath = argv[++i];
        }
        if (std::strcmp(argv[i], "--lm-scale") == 0 && i + 1 < argc) {
            lmScale = std::atoi(argv[++i]);
            if (lmScale < 1) lmScale = 1;
            if (lmScale > 8) lmScale = 8;
        }
        if (std::strcmp(argv[i], "--no-objects") == 0) {
            showObjects = false;
        }
        if (std::strcmp(argv[i], "--force-flicker") == 0) {
            forceFlicker = true;
        }
    }

    bool texturedMode = !resPath.empty();

    // Extract mission base name (e.g. "miss6" from "path/to/miss6.mis")
    // for constructing skybox texture filenames like "skyhw/miss6n.PCX"
    std::string missionName;
    {
        std::string p(misPath);
        size_t slash = p.find_last_of("/\\");
        std::string base = (slash != std::string::npos) ? p.substr(slash + 1) : p;
        size_t dot = base.find('.');
        missionName = (dot != std::string::npos) ? base.substr(0, dot) : base;
        // Lowercase for case-insensitive CRF matching
        std::transform(missionName.begin(), missionName.end(), missionName.begin(),
                       [](unsigned char c) { return std::tolower(c); });
    }

    // Parse WR geometry
    std::fprintf(stderr, "Loading WR geometry from %s...\n", misPath);

    Opde::WRParsedData wrData;
    try {
        wrData = Opde::parseWRChunk(misPath);
    } catch (const std::exception &e) {
        std::fprintf(stderr, "Failed to parse WR chunk: %s\n", e.what());
        return 1;
    }

    std::fprintf(stderr, "Loaded %u cells\n", wrData.numCells);

    // Find player spawn point from L$PlayerFactory + P$Position chunks
    Opde::SpawnInfo spawnInfo = Opde::findSpawnPoint(misPath);

    // Parse animated light properties from mission database
    auto lightSources = Opde::parseAnimLightProperties(misPath);

    // Build reverse index: lightnum → list of (cellIdx, polyIdx) affected
    auto animLightIndex = Opde::buildAnimLightIndex(wrData);

    // Ensure all lightnums referenced in WR data have a LightSource entry.
    // Lights without P$AnimLight properties default to mode 4 (max brightness).
    for (const auto &kv : animLightIndex) {
        if (lightSources.find(kv.first) == lightSources.end()) {
            Opde::LightSource ls = {};
            ls.lightNum = kv.first;
            ls.mode = Opde::ANIM_MAX_BRIGHT;
            ls.maxBright = 1.0f;
            ls.minBright = 0.0f;
            ls.brightness = 1.0f;
            ls.prevIntensity = 1.0f;
            lightSources[kv.first] = ls;
        }
    }

    // Apply --force-flicker: override all lights to flicker mode for debugging
    if (forceFlicker) {
        for (auto &[num, ls] : lightSources) {
            ls.mode = Opde::ANIM_FLICKER;
            ls.inactive = false;
            ls.minBright = 0.0f;
            ls.maxBright = 1.0f;
            ls.brightenTime = 0.15f;
            ls.dimTime = 0.15f;
            ls.brightness = ls.maxBright;
            ls.countdown = 0.1f;
            ls.isRising = false;
        }
        std::fprintf(stderr, "Force-flicker: all %zu lights set to flicker mode\n",
                     lightSources.size());
    }

    // Animated light diagnostics
    {
        int modeCounts[10] = {};
        int inactiveCount = 0;
        int fromMIS = 0, fromDefault = 0;
        for (const auto &[num, ls] : lightSources) {
            if (ls.mode < 10) modeCounts[ls.mode]++;
            if (ls.inactive) inactiveCount++;
            if (ls.objectId != 0) fromMIS++; else fromDefault++;
        }
        std::fprintf(stderr, "Animated lights: %zu sources (%d from MIS, %d defaulted), "
                     "%zu indexed lightnums\n",
                     lightSources.size(), fromMIS, fromDefault, animLightIndex.size());
        const char *modeNames[] = {
            "flip", "smooth", "random", "min_bright", "max_bright",
            "zero", "brighten", "dim", "semi_random", "flicker"
        };
        for (int m = 0; m < 10; ++m) {
            if (modeCounts[m] > 0)
                std::fprintf(stderr, "  mode %d (%s): %d lights\n",
                             m, modeNames[m], modeCounts[m]);
        }
        if (inactiveCount > 0)
            std::fprintf(stderr, "  inactive: %d lights\n", inactiveCount);
    }

    // Parse sky dome parameters from SKYOBJVAR chunk (if present)
    SkyParams skyParams = parseSkyObjVar(misPath);
    SkyDome skyDome = buildSkyDome(skyParams);

    // Parse TXLIST if in textured mode
    Opde::TXList txList;
    if (texturedMode) {
        try {
            txList = Opde::parseTXList(misPath);
            std::fprintf(stderr, "TXLIST: %zu textures, %zu families\n",
                         txList.textures.size(), txList.families.size());
        } catch (const std::exception &e) {
            std::fprintf(stderr, "Failed to parse TXLIST: %s (falling back to flat)\n",
                         e.what());
            texturedMode = false;
        }
    }

    // Collect unique texture indices used by world geometry
    std::unordered_set<uint8_t> usedTextures;
    if (texturedMode) {
        for (const auto &cell : wrData.cells) {
            for (int pi = 0; pi < cell.numTextured; ++pi) {
                uint8_t txt = cell.texturing[pi].txt;
                if (txt != 0 && txt != 249)
                    usedTextures.insert(txt);
            }
        }
        std::fprintf(stderr, "Unique texture indices used: %zu\n", usedTextures.size());
    }

    // Load textures from CRF
    std::unordered_map<uint8_t, Opde::DecodedImage> loadedTextures;
    std::unordered_map<uint8_t, TexDimensions> texDims;

    if (texturedMode) {
        Opde::CRFTextureLoader loader(resPath);
        if (!loader.isOpen()) {
            std::fprintf(stderr, "CRF not available, falling back to flat shading\n");
            texturedMode = false;
        } else {
            int loaded = 0;
            for (uint8_t idx : usedTextures) {
                if (idx >= txList.textures.size()) continue;
                const auto &entry = txList.textures[idx];
                auto img = loader.loadTexture(entry.family, entry.name);
                texDims[idx] = { img.width, img.height };
                loadedTextures[idx] = std::move(img);
                ++loaded;
            }
            std::fprintf(stderr, "Loaded %d/%zu textures from CRF\n",
                         loaded, usedTextures.size());
        }
    }

    // ── Load skybox face textures (old sky system) ──
    // Missions without SKYOBJVAR use a textured skybox with per-mission PCX
    // textures in fam.crf under skyhw/ (e.g. skyhw/miss6n.PCX for north face).
    std::unordered_map<std::string, Opde::DecodedImage> skyboxImages;
    bool hasSkybox = false;

    if (texturedMode) {
        Opde::CRFTextureLoader skyLoader(resPath);
        if (skyLoader.isOpen()) {
            // 5 faces: n=north(+Y), s=south(-Y), e=east(+X), w=west(-X), t=top(+Z)
            const char *suffixes[] = { "n", "s", "e", "w", "t" };
            int loaded = 0;
            for (const char *suf : suffixes) {
                std::string texName = missionName + suf;
                auto img = skyLoader.loadTexture("skyhw", texName);
                // Real texture is larger than the 8x8 fallback checkerboard
                if (img.width > 8 || img.height > 8) {
                    skyboxImages[suf] = std::move(img);
                    ++loaded;
                }
            }
            // Skybox available if at least the 4 side faces loaded (top optional)
            hasSkybox = skyboxImages.count("n") && skyboxImages.count("s")
                     && skyboxImages.count("e") && skyboxImages.count("w");
            if (hasSkybox) {
                std::fprintf(stderr, "Skybox: loaded %d/5 faces for %s (textured skybox active)\n",
                             loaded, missionName.c_str());
                for (auto &kv : skyboxImages) {
                    std::fprintf(stderr, "  face '%s': %ux%u\n",
                                 kv.first.c_str(), kv.second.width, kv.second.height);
                }
            } else if (loaded > 0) {
                std::fprintf(stderr, "Skybox: partial load (%d faces), falling back to dome\n", loaded);
            }
        }
    }

    // ── Parse object placements from .mis ──

    Opde::ObjectPropData objData;
    if (showObjects) {
        try {
            objData = Opde::parseObjectProps(misPath);
        } catch (const std::exception &e) {
            std::fprintf(stderr, "Failed to parse object props: %s\n", e.what());
            showObjects = false;
        }
        if (objData.objects.empty()) {
            std::fprintf(stderr, "No objects to render\n");
            showObjects = false;
        }
    }

    // Load .bin models from obj.crf (if --res provided and objects enabled)
    std::unordered_map<std::string, Opde::ParsedBinMesh> parsedModels;

    if (showObjects && !resPath.empty()) {
        Opde::CRFModelLoader modelLoader(resPath);
        if (modelLoader.isOpen()) {
            int loaded = 0, failed = 0;
            for (const auto &name : objData.uniqueModels) {
                auto binData = modelLoader.loadModel(name);
                if (binData.empty()) {
                    ++failed;
                    continue;
                }
                try {
                    auto mesh = Opde::parseBinModel(binData.data(), binData.size());
                    if (mesh.valid) {
                        parsedModels[name] = std::move(mesh);
                        ++loaded;
                    } else {
                        // Log first few failures for debugging
                        if (failed < 5) {
                            // Show magic header of failed file
                            char hdr[5] = {};
                            if (binData.size() >= 4)
                                std::memcpy(hdr, binData.data(), 4);
                            std::fprintf(stderr, "  model '%s': parse failed "
                                         "(size=%zu, magic='%s')\n",
                                         name.c_str(), binData.size(), hdr);
                        }
                        ++failed;
                    }
                } catch (const std::exception &e) {
                    // Some .bin files may be AI meshes (LGMM) or corrupt
                    if (failed < 5) {
                        std::fprintf(stderr, "  model '%s': exception: %s\n",
                                     name.c_str(), e.what());
                    }
                    ++failed;
                }
            }
            std::fprintf(stderr, "Loaded %d/%zu models from obj.crf (%d failed)\n",
                         loaded, objData.uniqueModels.size(), failed);
        } else {
            std::fprintf(stderr, "obj.crf not available, using fallback cubes\n");
        }
    }

    // ── Load object textures from txt16.crf (16-bit) with txt.crf fallback ──

    // Collect unique MD_MAT_TMAP material names from all parsed models
    std::unordered_set<std::string> objMatNames;
    for (const auto &kv : parsedModels) {
        for (const auto &mat : kv.second.materials) {
            if (mat.type == Opde::MD_MAT_TMAP) {
                // Lowercase the name for case-insensitive matching
                std::string lname(mat.name);
                std::transform(lname.begin(), lname.end(), lname.begin(),
                               [](unsigned char c) { return std::tolower(c); });
                objMatNames.insert(lname);
            }
        }
    }

    // Load object textures from obj.crf (txt16/ and txt/ subdirectories inside it).
    // Dark Engine stores object textures as GIF/PCX files within obj.crf, not in
    // separate txt16.crf/txt.crf archives.
    std::unordered_map<std::string, Opde::DecodedImage> objTexImages;

    if (!objMatNames.empty() && !resPath.empty()) {
        // Reuse obj.crf for texture lookup (same archive that holds .bin models)
        Opde::CRFTextureLoader objTexLoader(resPath, "obj.crf");

        int loaded = 0;
        for (const auto &name : objMatNames) {
            if (!objTexLoader.isOpen()) break;
            auto img = objTexLoader.loadObjectTexture(name);
            // Check if we got a real texture (not the 8x8 fallback checkerboard)
            if (img.width > 8 || img.height > 8) {
                objTexImages[name] = std::move(img);
                ++loaded;
            }
        }
        std::fprintf(stderr, "Loaded %d/%zu object textures from obj.crf\n",
                     loaded, objMatNames.size());
    }

    // ── SDL2 + bgfx init ──

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window *window = SDL_CreateWindow(
        "darkness — lightmapped renderer",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WINDOW_WIDTH, WINDOW_HEIGHT,
        SDL_WINDOW_SHOWN | SDL_WINDOW_ALLOW_HIGHDPI
    );

    if (!window) {
        std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_SysWMinfo wmi;
    SDL_VERSION(&wmi.version);
    if (!SDL_GetWindowWMInfo(window, &wmi)) {
        std::fprintf(stderr, "SDL_GetWindowWMInfo failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    bgfx::renderFrame(); // single-threaded mode

    bgfx::Init bInit;
    bInit.type = bgfx::RendererType::Metal;
    bInit.resolution.width  = WINDOW_WIDTH;
    bInit.resolution.height = WINDOW_HEIGHT;
    bInit.resolution.reset  = BGFX_RESET_VSYNC;

#if BX_PLATFORM_OSX
    bInit.platformData.nwh = wmi.info.cocoa.window;
#elif BX_PLATFORM_LINUX
    bInit.platformData.ndt = wmi.info.x11.display;
    bInit.platformData.nwh = (void *)(uintptr_t)wmi.info.x11.window;
#elif BX_PLATFORM_WINDOWS
    bInit.platformData.nwh = wmi.info.win.window;
#endif

    if (!bgfx::init(bInit)) {
        std::fprintf(stderr, "bgfx::init failed\n");
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    // View 0: Sky pass — clears colour + depth, renders sky dome with no depth writes
    bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
                        0x1a1a2eFF, 1.0f, 0);
    bgfx::setViewRect(0, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);

    // View 1: World + objects pass — clears depth only, preserves sky colour
    bgfx::setViewClear(1, BGFX_CLEAR_DEPTH, 0, 1.0f, 0);
    bgfx::setViewRect(1, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);

    PosColorVertex::init();
    PosColorUVVertex::init();
    PosUV2Vertex::init();

    // ── Shaders ──

    // Flat-color program (existing basic shader)
    bgfx::ShaderHandle flatVsh = bgfx::createShader(
        bgfx::makeRef(vs_basic_metal, sizeof(vs_basic_metal))
    );
    bgfx::ShaderHandle flatFsh = bgfx::createShader(
        bgfx::makeRef(fs_basic_metal, sizeof(fs_basic_metal))
    );
    bgfx::ProgramHandle flatProgram = bgfx::createProgram(flatVsh, flatFsh, true);

    // Textured program
    bgfx::ShaderHandle texVsh = bgfx::createShader(
        bgfx::makeRef(vs_textured_metal, sizeof(vs_textured_metal))
    );
    bgfx::ShaderHandle texFsh = bgfx::createShader(
        bgfx::makeRef(fs_textured_metal, sizeof(fs_textured_metal))
    );
    bgfx::ProgramHandle texturedProgram = bgfx::createProgram(texVsh, texFsh, true);

    // Lightmapped program
    bgfx::ProgramHandle lightmappedProgram = bgfx::createProgram(
        bgfx::createShader(bgfx::makeRef(vs_lightmapped_metal, sizeof(vs_lightmapped_metal))),
        bgfx::createShader(bgfx::makeRef(fs_lightmapped_metal, sizeof(fs_lightmapped_metal))),
        true);

    bgfx::UniformHandle s_texColor = bgfx::createUniform("s_texColor", bgfx::UniformType::Sampler);
    bgfx::UniformHandle s_texLightmap = bgfx::createUniform("s_texLightmap", bgfx::UniformType::Sampler);

    // ── Build lightmap atlas (if textured mode) ──

    bool lightmappedMode = false;
    Opde::LightmapAtlasSet lmAtlasSet;
    std::vector<bgfx::TextureHandle> lightmapAtlasHandles;

    if (texturedMode) {
        lmAtlasSet = Opde::buildLightmapAtlases(wrData, lmScale);
        if (!lmAtlasSet.atlases.empty()) {
            lightmappedMode = true;

            // Initial blend pass: apply animated overlays at initial intensities.
            // buildLightmapAtlases() only wrote static lightmaps into the atlas.
            // We must blend in overlay contributions so lights at max brightness
            // (mode 4, the default) have correct initial appearance.
            if (!animLightIndex.empty()) {
                // Compute initial intensities for all lights
                std::unordered_map<int16_t, float> initIntensities;
                for (const auto &[lightNum, light] : lightSources) {
                    initIntensities[lightNum] = (light.maxBright > 0.0f)
                        ? light.brightness / light.maxBright : 0.0f;
                }

                // Collect unique (cell, poly) pairs — a polygon may appear under
                // multiple lightnums, but blendAnimatedLightmap re-blends all
                // overlays at once, so we only need to call it once per polygon.
                std::unordered_set<uint64_t> blendedSet;
                int blendedPolys = 0;
                for (const auto &[lightNum, polys] : animLightIndex) {
                    for (const auto &[ci, pi] : polys) {
                        uint64_t key = (static_cast<uint64_t>(ci) << 32)
                                      | static_cast<uint32_t>(pi);
                        if (!blendedSet.insert(key).second) continue;

                        Opde::blendAnimatedLightmap(
                            lmAtlasSet.atlases[0], wrData, ci, pi,
                            lmAtlasSet.entries[ci][pi],
                            initIntensities, lmScale);
                        ++blendedPolys;
                    }
                }
                std::fprintf(stderr, "Initial lightmap blend: %d polygons\n",
                             blendedPolys);
            }

            for (const auto &atlas : lmAtlasSet.atlases) {
                // Create texture WITHOUT initial data so it stays mutable —
                // bgfx treats textures with initial mem as immutable.
                // We upload via updateTexture2D immediately after creation.
                // Point filtering so --lm-scale 1 gives the original blocky/vintage
                // look. Higher lm-scale values bake bicubic smoothing into the
                // atlas texels, providing progressively smoother lighting.
                bgfx::TextureHandle th = bgfx::createTexture2D(
                    static_cast<uint16_t>(atlas.size),
                    static_cast<uint16_t>(atlas.size),
                    false, 1, bgfx::TextureFormat::RGBA8,
                    BGFX_SAMPLER_MIN_POINT | BGFX_SAMPLER_MAG_POINT
                    | BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP);
                // Upload initial atlas data
                const bgfx::Memory *mem = bgfx::copy(atlas.rgba.data(),
                    static_cast<uint32_t>(atlas.rgba.size()));
                bgfx::updateTexture2D(th, 0, 0, 0, 0,
                    static_cast<uint16_t>(atlas.size),
                    static_cast<uint16_t>(atlas.size), mem);
                lightmapAtlasHandles.push_back(th);
            }
            std::fprintf(stderr, "Created %zu lightmap atlas GPU texture(s)\n",
                         lightmapAtlasHandles.size());
        }
    }

    // ── Build geometry and create GPU buffers ──

    float camX, camY, camZ;
    bgfx::VertexBufferHandle vbh;
    bgfx::IndexBufferHandle ibh;

    LightmappedMesh lmMesh;
    WorldMesh worldMesh;
    FlatMesh flatMesh;

    // Create bgfx texture handles
    std::unordered_map<uint8_t, bgfx::TextureHandle> textureHandles;

    if (lightmappedMode) {
        lmMesh = buildLightmappedMesh(wrData, texDims, lmAtlasSet);
        camX = lmMesh.cx; camY = lmMesh.cy; camZ = lmMesh.cz;

        std::fprintf(stderr, "Geometry (lightmapped): %zu vertices, %zu indices (%zu triangles), %zu texture groups\n",
                     lmMesh.vertices.size(), lmMesh.indices.size(),
                     lmMesh.indices.size() / 3, lmMesh.groups.size());

        if (lmMesh.vertices.empty()) {
            std::fprintf(stderr, "No geometry to render\n");
            bgfx::shutdown();
            SDL_DestroyWindow(window);
            SDL_Quit();
            return 1;
        }

        const bgfx::Memory *vbMem = bgfx::copy(
            lmMesh.vertices.data(),
            static_cast<uint32_t>(lmMesh.vertices.size() * sizeof(PosUV2Vertex))
        );
        vbh = bgfx::createVertexBuffer(vbMem, PosUV2Vertex::layout);

        const bgfx::Memory *ibMem = bgfx::copy(
            lmMesh.indices.data(),
            static_cast<uint32_t>(lmMesh.indices.size() * sizeof(uint32_t))
        );
        ibh = bgfx::createIndexBuffer(ibMem, BGFX_BUFFER_INDEX32);

        // Create bgfx textures from loaded images
        for (auto &kv : loadedTextures) {
            uint8_t idx = kv.first;
            const auto &img = kv.second;
            const bgfx::Memory *mem = bgfx::copy(img.rgba.data(),
                static_cast<uint32_t>(img.rgba.size()));
            textureHandles[idx] = bgfx::createTexture2D(
                static_cast<uint16_t>(img.width),
                static_cast<uint16_t>(img.height),
                false, 1, bgfx::TextureFormat::RGBA8,
                BGFX_SAMPLER_MIN_POINT | BGFX_SAMPLER_MAG_POINT
                | BGFX_SAMPLER_U_MIRROR | BGFX_SAMPLER_V_MIRROR,
                mem
            );
        }
        std::fprintf(stderr, "Created %zu GPU textures\n", textureHandles.size());
    } else if (texturedMode) {
        worldMesh = buildTexturedMesh(wrData, texDims);
        camX = worldMesh.cx; camY = worldMesh.cy; camZ = worldMesh.cz;

        std::fprintf(stderr, "Geometry (textured): %zu vertices, %zu indices (%zu triangles), %zu texture groups\n",
                     worldMesh.vertices.size(), worldMesh.indices.size(),
                     worldMesh.indices.size() / 3, worldMesh.groups.size());

        if (worldMesh.vertices.empty()) {
            std::fprintf(stderr, "No geometry to render\n");
            bgfx::shutdown();
            SDL_DestroyWindow(window);
            SDL_Quit();
            return 1;
        }

        const bgfx::Memory *vbMem = bgfx::copy(
            worldMesh.vertices.data(),
            static_cast<uint32_t>(worldMesh.vertices.size() * sizeof(PosColorUVVertex))
        );
        vbh = bgfx::createVertexBuffer(vbMem, PosColorUVVertex::layout);

        const bgfx::Memory *ibMem = bgfx::copy(
            worldMesh.indices.data(),
            static_cast<uint32_t>(worldMesh.indices.size() * sizeof(uint32_t))
        );
        ibh = bgfx::createIndexBuffer(ibMem, BGFX_BUFFER_INDEX32);

        for (auto &kv : loadedTextures) {
            uint8_t idx = kv.first;
            const auto &img = kv.second;
            const bgfx::Memory *mem = bgfx::copy(img.rgba.data(),
                static_cast<uint32_t>(img.rgba.size()));
            textureHandles[idx] = bgfx::createTexture2D(
                static_cast<uint16_t>(img.width),
                static_cast<uint16_t>(img.height),
                false, 1, bgfx::TextureFormat::RGBA8,
                BGFX_SAMPLER_MIN_POINT | BGFX_SAMPLER_MAG_POINT
                | BGFX_SAMPLER_U_MIRROR | BGFX_SAMPLER_V_MIRROR,
                mem
            );
        }
        std::fprintf(stderr, "Created %zu GPU textures\n", textureHandles.size());
    } else {
        flatMesh = buildFlatMesh(wrData);
        camX = flatMesh.cx; camY = flatMesh.cy; camZ = flatMesh.cz;

        std::fprintf(stderr, "Geometry (flat): %zu vertices, %zu indices (%zu triangles)\n",
                     flatMesh.vertices.size(), flatMesh.indices.size(),
                     flatMesh.indices.size() / 3);

        if (flatMesh.vertices.empty()) {
            std::fprintf(stderr, "No geometry to render\n");
            bgfx::shutdown();
            SDL_DestroyWindow(window);
            SDL_Quit();
            return 1;
        }

        const bgfx::Memory *vbMem = bgfx::copy(
            flatMesh.vertices.data(),
            static_cast<uint32_t>(flatMesh.vertices.size() * sizeof(PosColorVertex))
        );
        vbh = bgfx::createVertexBuffer(vbMem, PosColorVertex::layout);

        const bgfx::Memory *ibMem = bgfx::copy(
            flatMesh.indices.data(),
            static_cast<uint32_t>(flatMesh.indices.size() * sizeof(uint32_t))
        );
        ibh = bgfx::createIndexBuffer(ibMem, BGFX_BUFFER_INDEX32);
    }

    // ── Create GPU buffers for object meshes ──

    // Per-unique-model GPU buffers (PosColorUVVertex for textured rendering)
    std::unordered_map<std::string, ObjectModelGPU> objModelGPU;

    // Object texture GPU handles (keyed by lowercase material name)
    std::unordered_map<std::string, bgfx::TextureHandle> objTextureHandles;

    // Fallback cube: a single shared VBH/IBH for models not found in obj.crf
    bgfx::VertexBufferHandle fallbackCubeVBH = BGFX_INVALID_HANDLE;
    bgfx::IndexBufferHandle fallbackCubeIBH = BGFX_INVALID_HANDLE;
    uint32_t fallbackCubeIndexCount = 0;

    if (showObjects) {
        // Build fallback cube
        {
            std::vector<PosColorVertex> cubeVerts;
            std::vector<uint32_t> cubeIndices;
            buildFallbackCube(cubeVerts, cubeIndices, 0xff808080u);
            fallbackCubeIndexCount = static_cast<uint32_t>(cubeIndices.size());
            fallbackCubeVBH = bgfx::createVertexBuffer(
                bgfx::copy(cubeVerts.data(),
                    static_cast<uint32_t>(cubeVerts.size() * sizeof(PosColorVertex))),
                PosColorVertex::layout);
            fallbackCubeIBH = bgfx::createIndexBuffer(
                bgfx::copy(cubeIndices.data(),
                    static_cast<uint32_t>(cubeIndices.size() * sizeof(uint32_t))),
                BGFX_BUFFER_INDEX32);
        }

        // Create bgfx texture handles from loaded object texture images
        for (auto &kv : objTexImages) {
            const auto &img = kv.second;
            const bgfx::Memory *mem = bgfx::copy(img.rgba.data(),
                static_cast<uint32_t>(img.rgba.size()));
            objTextureHandles[kv.first] = bgfx::createTexture2D(
                static_cast<uint16_t>(img.width),
                static_cast<uint16_t>(img.height),
                false, 1, bgfx::TextureFormat::RGBA8,
                BGFX_SAMPLER_MIN_POINT | BGFX_SAMPLER_MAG_POINT
                | BGFX_SAMPLER_U_MIRROR | BGFX_SAMPLER_V_MIRROR,
                mem);
        }
        if (!objTextureHandles.empty()) {
            std::fprintf(stderr, "Created %zu object texture GPU handles\n",
                         objTextureHandles.size());
        }

        // Create GPU buffers for each parsed .bin model
        for (auto &kv : parsedModels) {
            const std::string &name = kv.first;
            const Opde::ParsedBinMesh &mesh = kv.second;

            if (mesh.vertices.empty() || mesh.indices.empty()) continue;

            // Build a vertex-to-material map so we can colour vertices appropriately.
            // Each submesh covers a range of indices; map each index to its material.
            std::vector<int> vertexMatIndex(mesh.vertices.size(), -1);
            for (const auto &sm : mesh.subMeshes) {
                int mi = sm.matIndex;
                for (uint32_t ii = sm.firstIndex; ii < sm.firstIndex + sm.indexCount; ++ii) {
                    if (ii < mesh.indices.size()) {
                        uint32_t vi = mesh.indices[ii];
                        if (vi < vertexMatIndex.size()) {
                            vertexMatIndex[vi] = mi;
                        }
                    }
                }
            }

            // Fallback colour for non-textured models or Opde::MD_MAT_COLOR materials
            uint32_t fallbackColor = colorFromName(name);

            // Build set of material indices that have actually-loaded textures,
            // so we only assign white to vertices that will be textured at draw time.
            std::unordered_set<int> loadedMatIndices;
            for (int mi = 0; mi < static_cast<int>(mesh.materials.size()); ++mi) {
                if (mesh.materials[mi].type == Opde::MD_MAT_TMAP) {
                    std::string lname(mesh.materials[mi].name);
                    std::transform(lname.begin(), lname.end(), lname.begin(),
                                   [](unsigned char c) { return std::tolower(c); });
                    if (objTexImages.count(lname)) {
                        loadedMatIndices.insert(mi);
                    }
                }
            }

            // Convert BinVert -> PosColorUVVertex
            std::vector<PosColorUVVertex> gpuVerts(mesh.vertices.size());
            for (size_t i = 0; i < mesh.vertices.size(); ++i) {
                const auto &bv = mesh.vertices[i];

                // Apply simple directional lighting using the vertex normal
                float dot = bv.nx * 0.3f + bv.ny * 0.8f + bv.nz * 0.4f;
                float brightness = std::max(dot, 0.0f) * 0.7f + 0.3f;

                // Determine vertex colour based on material type
                uint32_t vertColor;
                int mi = vertexMatIndex[i];
                if (mi >= 0 && loadedMatIndices.count(mi)) {
                    // Textured material with loaded texture: white * brightness
                    // (texture will modulate the final colour at draw time)
                    vertColor = packABGR(brightness, brightness, brightness);
                } else if (mi >= 0 && mi < static_cast<int>(mesh.materials.size()) &&
                           mesh.materials[mi].type == Opde::MD_MAT_COLOR) {
                    // Solid-colour material: BGRA from material data * brightness
                    const uint8_t *c = mesh.materials[mi].colour;
                    vertColor = packABGR(c[2] / 255.0f * brightness,   // R (colour is BGRA)
                                         c[1] / 255.0f * brightness,   // G
                                         c[0] / 255.0f * brightness);  // B
                } else {
                    // No loaded texture, unknown material, or palette — use hash colour
                    uint8_t r = fallbackColor & 0xFF;
                    uint8_t g = (fallbackColor >> 8) & 0xFF;
                    uint8_t b = (fallbackColor >> 16) & 0xFF;
                    vertColor = packABGR(r / 255.0f * brightness,
                                         g / 255.0f * brightness,
                                         b / 255.0f * brightness);
                }

                gpuVerts[i] = {
                    bv.x, bv.y, bv.z,
                    vertColor,
                    bv.u, bv.v
                };
            }

            ObjectModelGPU gpu;
            gpu.vbh = bgfx::createVertexBuffer(
                bgfx::copy(gpuVerts.data(),
                    static_cast<uint32_t>(gpuVerts.size() * sizeof(PosColorUVVertex))),
                PosColorUVVertex::layout);
            gpu.ibh = bgfx::createIndexBuffer(
                bgfx::copy(mesh.indices.data(),
                    static_cast<uint32_t>(mesh.indices.size() * sizeof(uint32_t))),
                BGFX_BUFFER_INDEX32);

            // Build per-submesh GPU draw info from parsed submeshes
            for (const auto &sm : mesh.subMeshes) {
                ObjectSubMeshGPU gsm;
                gsm.firstIndex = sm.firstIndex;
                gsm.indexCount = sm.indexCount;
                gsm.textured = false;
                if (sm.matIndex >= 0 && sm.matIndex < static_cast<int>(mesh.materials.size())) {
                    const auto &mat = mesh.materials[sm.matIndex];
                    gsm.textured = (mat.type == Opde::MD_MAT_TMAP);
                    // Lowercase material name for texture lookup
                    gsm.matName = mat.name;
                    std::transform(gsm.matName.begin(), gsm.matName.end(),
                                   gsm.matName.begin(),
                                   [](unsigned char c) { return std::tolower(c); });
                }
                gpu.subMeshes.push_back(gsm);
            }

            gpu.valid = true;
            objModelGPU[name] = std::move(gpu);
        }
        std::fprintf(stderr, "Object GPU buffers: %zu models + fallback cube\n",
                     objModelGPU.size());
    }

    // ── Create sky dome GPU buffers ──
    bgfx::VertexBufferHandle skyVBH = BGFX_INVALID_HANDLE;
    bgfx::IndexBufferHandle skyIBH = BGFX_INVALID_HANDLE;
    uint32_t skyIndexCount = 0;

    if (!skyDome.vertices.empty()) {
        skyVBH = bgfx::createVertexBuffer(
            bgfx::copy(skyDome.vertices.data(),
                static_cast<uint32_t>(skyDome.vertices.size() * sizeof(PosColorVertex))),
            PosColorVertex::layout);
        skyIBH = bgfx::createIndexBuffer(
            bgfx::copy(skyDome.indices.data(),
                static_cast<uint32_t>(skyDome.indices.size() * sizeof(uint16_t))));
        skyIndexCount = static_cast<uint32_t>(skyDome.indices.size());
    }

    // ── Create textured skybox GPU buffers (old sky system) ──
    bgfx::VertexBufferHandle skyboxVBH = BGFX_INVALID_HANDLE;
    bgfx::IndexBufferHandle skyboxIBH = BGFX_INVALID_HANDLE;
    SkyboxCube skyboxCube;
    std::unordered_map<std::string, bgfx::TextureHandle> skyboxTexHandles;

    if (hasSkybox) {
        skyboxCube = buildSkyboxCube();

        skyboxVBH = bgfx::createVertexBuffer(
            bgfx::copy(skyboxCube.vertices.data(),
                static_cast<uint32_t>(skyboxCube.vertices.size() * sizeof(PosColorUVVertex))),
            PosColorUVVertex::layout);
        skyboxIBH = bgfx::createIndexBuffer(
            bgfx::copy(skyboxCube.indices.data(),
                static_cast<uint32_t>(skyboxCube.indices.size() * sizeof(uint16_t))));

        // Create GPU textures for each loaded skybox face
        for (auto &kv : skyboxImages) {
            const auto &img = kv.second;
            const bgfx::Memory *mem = bgfx::copy(img.rgba.data(),
                static_cast<uint32_t>(img.rgba.size()));
            skyboxTexHandles[kv.first] = bgfx::createTexture2D(
                static_cast<uint16_t>(img.width),
                static_cast<uint16_t>(img.height),
                false, 1, bgfx::TextureFormat::RGBA8,
                BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP,
                mem);
        }
        std::fprintf(stderr, "Skybox GPU: %zu face textures created\n",
                     skyboxTexHandles.size());
    }

    const char *modeStr = lightmappedMode ? "lightmapped" :
                          texturedMode ? "textured" : "flat-shaded";
    std::fprintf(stderr, "Render window opened (%dx%d, Metal, %s)\n",
                 WINDOW_WIDTH, WINDOW_HEIGHT, modeStr);

    // Use spawn point if found, otherwise fall back to centroid
    float spawnX = camX, spawnY = camY, spawnZ = camZ;
    float spawnYaw = 0.0f;
    if (spawnInfo.found) {
        spawnX = spawnInfo.x;
        spawnY = spawnInfo.y;
        spawnZ = spawnInfo.z;
        spawnYaw = spawnInfo.yaw;
        std::fprintf(stderr, "Camera at spawn (%.1f, %.1f, %.1f)\n",
                     spawnX, spawnY, spawnZ);
    } else {
        std::fprintf(stderr, "Camera at centroid (%.1f, %.1f, %.1f)\n",
                     camX, camY, camZ);
    }

    std::fprintf(stderr, "Controls: WASD=move, mouse=look, Space/LShift=up/down, "
                 "scroll=speed, Home=spawn, Esc=quit\n");

    Camera cam;
    cam.init(spawnX, spawnY, spawnZ);
    cam.yaw = spawnYaw;

    SDL_SetRelativeMouseMode(SDL_TRUE);

    float moveSpeed = 20.0f; // adjustable via scroll wheel
    const float MOUSE_SENS = 0.002f;
    const float PI = 3.14159265f;

    // Helper to update window title with current speed
    auto updateTitle = [&]() {
        char title[128];
        std::snprintf(title, sizeof(title), "darkness — %s [speed: %.1f]",
                      modeStr, moveSpeed);
        SDL_SetWindowTitle(window, title);
    };
    updateTitle();

    uint64_t renderState = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
                         | BGFX_STATE_WRITE_Z | BGFX_STATE_DEPTH_TEST_LESS
                         | BGFX_STATE_CULL_CW;

    auto lastTime = std::chrono::high_resolution_clock::now();

    bool running = true;
    while (running) {
        auto now = std::chrono::high_resolution_clock::now();
        float dt = std::chrono::duration<float>(now - lastTime).count();
        lastTime = now;
        dt = std::min(dt, 0.1f);

        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT) {
                running = false;
            } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_ESCAPE) {
                running = false;
            } else if (ev.type == SDL_MOUSEMOTION) {
                cam.yaw   -= ev.motion.xrel * MOUSE_SENS;
                cam.pitch -= ev.motion.yrel * MOUSE_SENS;
                cam.pitch = std::max(-PI * 0.49f, std::min(PI * 0.49f, cam.pitch));
            } else if (ev.type == SDL_MOUSEWHEEL) {
                // Scroll wheel adjusts movement speed: 1.5x per tick
                if (ev.wheel.y > 0) {
                    for (int i = 0; i < ev.wheel.y; ++i)
                        moveSpeed *= 1.5f;
                } else if (ev.wheel.y < 0) {
                    for (int i = 0; i < -ev.wheel.y; ++i)
                        moveSpeed /= 1.5f;
                }
                moveSpeed = std::max(1.0f, std::min(500.0f, moveSpeed));
                updateTitle();
            } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_HOME) {
                // Teleport back to spawn point
                cam.pos[0] = spawnX;
                cam.pos[1] = spawnY;
                cam.pos[2] = spawnZ;
                cam.yaw = spawnYaw;
                cam.pitch = 0;
                std::fprintf(stderr, "Teleported to spawn (%.1f, %.1f, %.1f)\n",
                             spawnX, spawnY, spawnZ);
            }
        }

        // Keyboard movement
        const Uint8 *keys = SDL_GetKeyboardState(nullptr);
        float forward = 0, right = 0, up = 0;
        float speed = moveSpeed;
        if (keys[SDL_SCANCODE_LCTRL] || keys[SDL_SCANCODE_RCTRL])
            speed *= 3.0f;

        if (keys[SDL_SCANCODE_W]) forward += speed * dt;
        if (keys[SDL_SCANCODE_S]) forward -= speed * dt;
        if (keys[SDL_SCANCODE_D]) right   += speed * dt;
        if (keys[SDL_SCANCODE_A]) right   -= speed * dt;
        if (keys[SDL_SCANCODE_SPACE])  up += speed * dt;
        if (keys[SDL_SCANCODE_LSHIFT]) up -= speed * dt;
        if (keys[SDL_SCANCODE_Q]) up += speed * dt;
        if (keys[SDL_SCANCODE_E]) up -= speed * dt;

        cam.move(forward, right, up);

        // ── Animated lightmap update ──
        // Advance all light animation timers and re-blend changed lightmaps
        if (lightmappedMode && !lmAtlasSet.atlases.empty()) {
            bool anyLightChanged = false;
            std::unordered_map<int16_t, float> currentIntensities;

            for (auto &[lightNum, light] : lightSources) {
                bool changed = Opde::updateLightAnimation(light, dt);
                float intensity = (light.maxBright > 0.0f)
                    ? light.brightness / light.maxBright : 0.0f;
                currentIntensities[lightNum] = intensity;
                if (changed) anyLightChanged = true;
            }

            // Re-blend changed lightmaps into atlas CPU buffer
            if (anyLightChanged) {
                for (auto &[lightNum, light] : lightSources) {
                    float intensity = currentIntensities[lightNum];
                    if (std::abs(intensity - light.prevIntensity) < 0.002f) continue;
                    light.prevIntensity = intensity;

                    auto it = animLightIndex.find(lightNum);
                    if (it == animLightIndex.end()) continue;

                    for (auto &[ci, pi] : it->second) {
                        Opde::blendAnimatedLightmap(
                            lmAtlasSet.atlases[0], wrData, ci, pi,
                            lmAtlasSet.entries[ci][pi],
                            currentIntensities, lmScale);
                    }
                }

                // Upload full atlas to GPU
                const auto &atlas = lmAtlasSet.atlases[0];
                const bgfx::Memory *mem = bgfx::copy(
                    atlas.rgba.data(), static_cast<uint32_t>(atlas.rgba.size()));
                bgfx::updateTexture2D(lightmapAtlasHandles[0], 0, 0, 0, 0,
                    static_cast<uint16_t>(atlas.size),
                    static_cast<uint16_t>(atlas.size), mem);
            }
        }

        // Set up projection matrix (shared by both views)
        float proj[16];
        bx::mtxProj(proj, 60.0f,
                     float(WINDOW_WIDTH) / float(WINDOW_HEIGHT),
                     0.1f, 5000.0f,
                     bgfx::getCaps()->homogeneousDepth,
                     bx::Handedness::Right);

        // ── View 0: Sky pass ──
        // Sky rendered with rotation-only view (no translation),
        // no depth write/test — appears infinitely far behind everything.
        {
            float skyView[16];
            cam.getSkyViewMatrix(skyView);
            bgfx::setViewTransform(0, skyView, proj);

            float skyModel[16];
            bx::mtxIdentity(skyModel);

            // Cull CCW (not CW) because the camera is inside the cube/sphere —
            // we see the back faces, so cull the outward-facing front faces.
            uint64_t skyState = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
                              | BGFX_STATE_CULL_CCW;

            if (hasSkybox && bgfx::isValid(skyboxVBH)) {
                // Textured skybox (old sky system) — render each face with its texture
                for (const auto &face : skyboxCube.faces) {
                    auto texIt = skyboxTexHandles.find(face.key);
                    if (texIt == skyboxTexHandles.end()) continue;

                    bgfx::setTransform(skyModel);
                    bgfx::setVertexBuffer(0, skyboxVBH);
                    bgfx::setIndexBuffer(skyboxIBH, face.firstIndex, face.indexCount);
                    bgfx::setState(skyState);
                    bgfx::setTexture(0, s_texColor, texIt->second);
                    bgfx::submit(0, texturedProgram);
                }
            } else if (bgfx::isValid(skyVBH)) {
                // Procedural dome (new sky system) — vertex-coloured hemisphere
                bgfx::setTransform(skyModel);
                bgfx::setVertexBuffer(0, skyVBH);
                bgfx::setIndexBuffer(skyIBH);
                bgfx::setState(skyState);
                bgfx::submit(0, flatProgram);
            }
        }

        // ── View 1: World + objects pass ──
        float view[16];
        cam.getViewMatrix(view);
        bgfx::setViewTransform(1, view, proj);

        // Identity model transform for world geometry
        float model[16];
        bx::mtxIdentity(model);

        if (lightmappedMode) {
            for (const auto &grp : lmMesh.groups) {
                bgfx::setTransform(model);
                bgfx::setVertexBuffer(0, vbh);
                bgfx::setIndexBuffer(ibh, grp.firstIndex, grp.numIndices);
                bgfx::setState(renderState);

                if (grp.txtIndex == 0) {
                    bgfx::submit(1, flatProgram);
                } else {
                    auto it = textureHandles.find(grp.txtIndex);
                    if (it != textureHandles.end()) {
                        bgfx::setTexture(0, s_texColor, it->second);
                        if (!lightmapAtlasHandles.empty())
                            bgfx::setTexture(1, s_texLightmap, lightmapAtlasHandles[0]);
                        bgfx::submit(1, lightmappedProgram);
                    } else {
                        bgfx::submit(1, flatProgram);
                    }
                }
            }
        } else if (texturedMode) {
            for (const auto &grp : worldMesh.groups) {
                bgfx::setTransform(model);
                bgfx::setVertexBuffer(0, vbh);
                bgfx::setIndexBuffer(ibh, grp.firstIndex, grp.numIndices);
                bgfx::setState(renderState);

                if (grp.txtIndex == 0) {
                    bgfx::submit(1, flatProgram);
                } else {
                    auto it = textureHandles.find(grp.txtIndex);
                    if (it != textureHandles.end()) {
                        bgfx::setTexture(0, s_texColor, it->second);
                        bgfx::submit(1, texturedProgram);
                    } else {
                        bgfx::submit(1, flatProgram);
                    }
                }
            }
        } else {
            bgfx::setTransform(model);
            bgfx::setVertexBuffer(0, vbh);
            bgfx::setIndexBuffer(ibh);
            bgfx::setState(renderState);
            bgfx::submit(1, flatProgram);
        }

        // ── Draw object meshes (per-submesh for textured/solid materials) ──
        if (showObjects) {
            for (const auto &obj : objData.objects) {
                if (!obj.hasPosition) continue;

                // Compute per-object model matrix from position + angles
                float objMtx[16];
                buildModelMatrix(objMtx, obj.x, obj.y, obj.z,
                                 obj.heading, obj.pitch, obj.bank,
                                 obj.scaleX, obj.scaleY, obj.scaleZ);

                // Find the GPU model for this object's model name
                std::string modelName(obj.modelName);
                auto it = objModelGPU.find(modelName);

                if (it != objModelGPU.end() && it->second.valid) {
                    const auto &gpuModel = it->second;

                    // Draw each submesh with appropriate shader/texture
                    for (const auto &sm : gpuModel.subMeshes) {
                        if (sm.indexCount == 0) continue;

                        bgfx::setTransform(objMtx);
                        bgfx::setVertexBuffer(0, gpuModel.vbh);
                        bgfx::setIndexBuffer(gpuModel.ibh, sm.firstIndex, sm.indexCount);
                        bgfx::setState(renderState);

                        if (sm.textured) {
                            // Look up object texture by material name
                            auto texIt = objTextureHandles.find(sm.matName);
                            if (texIt != objTextureHandles.end()) {
                                bgfx::setTexture(0, s_texColor, texIt->second);
                                bgfx::submit(1, texturedProgram);
                            } else {
                                // Texture not loaded — fall back to flat shading
                                // (vertex colour is white*brightness, so it will appear grey)
                                bgfx::submit(1, flatProgram);
                            }
                        } else {
                            // Solid-colour submesh — vertex colour carries material colour
                            bgfx::submit(1, flatProgram);
                        }
                    }
                } else {
                    // Fallback: small coloured cube for missing models
                    bgfx::setTransform(objMtx);
                    bgfx::setVertexBuffer(0, fallbackCubeVBH);
                    bgfx::setIndexBuffer(fallbackCubeIBH);
                    bgfx::setState(renderState);
                    bgfx::submit(1, flatProgram);
                }
            }
        }

        bgfx::frame();
    }

    // Cleanup — textured skybox buffers
    if (bgfx::isValid(skyboxVBH)) bgfx::destroy(skyboxVBH);
    if (bgfx::isValid(skyboxIBH)) bgfx::destroy(skyboxIBH);
    for (auto &kv : skyboxTexHandles)
        bgfx::destroy(kv.second);

    // Cleanup — sky dome buffers
    if (bgfx::isValid(skyVBH)) bgfx::destroy(skyVBH);
    if (bgfx::isValid(skyIBH)) bgfx::destroy(skyIBH);

    // Cleanup — object GPU buffers and textures
    for (auto &kv : objModelGPU) {
        if (bgfx::isValid(kv.second.vbh)) bgfx::destroy(kv.second.vbh);
        if (bgfx::isValid(kv.second.ibh)) bgfx::destroy(kv.second.ibh);
    }
    for (auto &kv : objTextureHandles) {
        bgfx::destroy(kv.second);
    }
    if (bgfx::isValid(fallbackCubeVBH)) bgfx::destroy(fallbackCubeVBH);
    if (bgfx::isValid(fallbackCubeIBH)) bgfx::destroy(fallbackCubeIBH);

    // Cleanup — world geometry and textures
    for (auto &h : lightmapAtlasHandles)
        bgfx::destroy(h);
    for (auto &kv : textureHandles)
        bgfx::destroy(kv.second);
    bgfx::destroy(s_texLightmap);
    bgfx::destroy(s_texColor);

    bgfx::destroy(ibh);
    bgfx::destroy(vbh);
    bgfx::destroy(flatProgram);
    bgfx::destroy(texturedProgram);
    bgfx::destroy(lightmappedProgram);

    bgfx::shutdown();
    SDL_DestroyWindow(window);
    SDL_Quit();

    std::fprintf(stderr, "Clean shutdown.\n");
    return 0;
}
