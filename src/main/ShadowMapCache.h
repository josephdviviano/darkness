/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2024-2026 darkness contributors
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

// Shadow-map cache — S1 of PLAN.HIGH_RES_SHADOWS.md: the GPU side of the
// shadow oracle. Six R32F 2D faces per omni light, tiled into one atlas
// (ShadowFaceMath.h owns the face geometry; this header owns the bgfx
// plumbing: targets, view ids, the bounded LRU pool, the caster mesh, the
// readback cross-check and the debug HUD).
//
// The pool's validity test is HPL2's `iRenderer::ShadowMapNeedsUpdate`
// pattern, ported with attribution (GPLv3):
//   ../AmnesiaTheDarkDescent/HPL2/core/sources/graphics/Renderer.cpp:783-818
// — light identity, transform (position), radius, spot params, and a
// caster-set comparison (`ShadowCastersAreUnchanged`). The caster term is
// carried as a hash field from day one because it is what makes S3's
// door-event invalidation cheap; until S3 lands it is always 0 (world
// geometry only, and the world never changes).
//
// CASTER GEOMETRY IS ITS OWN MESH, not the render mesh. The lightmapped
// world mesh excludes sky polygons (texture 249 becomes a hole for the sky
// dome) and flat/null-textured polygons — but `raycastWorld`, the CPU
// oracle the S1 acceptance compares against (and the oracle the shipped
// lumel bake was verified against), stops at ALL solid polygons. The
// shadow pass therefore triangulates every solid WR polygon into a
// position-only buffer of its own: occluder set == BSP solid set, by
// construction rather than by hoping the render mesh happens to match.

#pragma once

#include "ShadowFaceMath.h"
#include "WRChunkParser.h"
#include "LightmapBake.h"   // physicalReachRadius, bakeRand01, kSubQuantThreshold
#include "RayCaster.h"      // the CPU oracle for the acceptance cross-check
#include "CellGeometry.h"   // findCameraCell — "is this light inside the world"
#include "PostProcess.h"    // kViewShadowFaces / kViewShadowDebug ownership
#include "../services/physics/ObjectCollisionGeometry.h"  // S4b dynamic casters

#include <bgfx/bgfx.h>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cstdio>
#include <vector>

namespace Darkness {

static_assert(kShadowFaceCount == 6,
              "PostProcess.h reserves view ids as kShadowMaxPoolSlots * 6");

// Position-only caster vertex.
struct ShadowCasterVertex {
    float x, y, z;

    inline static bgfx::VertexLayout layout;

    static void init() {
        layout.begin()
            .add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
            .end();
    }
};

// NEGATIVE-ID REGISTRY for pool slots. Static lights use their table
// index (> 0); every transient user owns a disjoint negative range so
// slots can never collide across subsystems:
//   -1000..-1999     LIVE EMITTERS (RuntimeState::liveEmitters —
//                    moving light-emitting objects; -1001 = the player
//                    lantern / console test vehicle)
//   -2000 - lightIdx S4c promoted light, CURRENT-pose faces
//   -3000 - lightIdx S4c promoted light, FROZEN baked-pose faces (pinned)
//   -9001            S1 cross-check dynamic leg
//   -9100 and down   --door-diff-diag harness (released after each run)
// Add new users here BEFORE picking an id.
//
// One pool slot: which light's six faces its tiles hold, and the state
// those faces were rendered from (the HPL2 validity fields).
struct ShadowSlot {
    int      lightIdx  = -1;          // -1 = never used
    // S4c: a pinned slot holds a FROZEN reference (door casters at the
    // last-baked pose) for a differential live light. Ensure/LRU must
    // never steal or re-render it — its whole value is that it does NOT
    // track the moving door.
    bool     pinned    = false;
    Vector3  pos{0.0f};               // light position at render time
    Vector3  bright{0.0f};            // brightness at render time (reach input)
    float    reach     = 0.0f;        // reach the faces were normalised by
    uint64_t casterHash = 0;          // S3 hook: moving-caster set (0 = world only)
    uint32_t lastUsed  = 0;           // frameIndex of last ensure — LRU key
    bool     rendered  = false;
};

struct ShadowMapCache {
    // ── Geometry of the atlas ──
    int   faceSize    = 256;          // texels per face edge
    int   slotCount   = kShadowMaxPoolSlots;
    int   tilesPerRow = 0;
    int   atlasW = 0, atlasH = 0;
    float nearZ = 0.05f;              // matches the bake's surface offset scale

    // ── GPU resources ──
    bgfx::TextureHandle     atlasTex     = BGFX_INVALID_HANDLE; // R32F, RT
    bgfx::TextureHandle     depthTex     = BGFX_INVALID_HANDLE; // D32F, RT
    bgfx::FrameBufferHandle fb           = BGFX_INVALID_HANDLE;
    bgfx::ProgramHandle     depthProgram = BGFX_INVALID_HANDLE; // owned
    bgfx::ProgramHandle     debugProgram = BGFX_INVALID_HANDLE; // owned (HUD)
    bgfx::ProgramHandle     clearProgram = BGFX_INVALID_HANDLE; // owned (tile clear)
    bgfx::UniformHandle     u_shadowLightPos = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle     u_shadowFaceMtx  = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle     s_shadowAtlas    = BGFX_INVALID_HANDLE;
    bgfx::VertexBufferHandle casterVbh  = BGFX_INVALID_HANDLE;
    bgfx::IndexBufferHandle  casterIbh  = BGFX_INVALID_HANDLE;
    uint32_t                 casterIndexCount = 0;
    bgfx::VertexBufferHandle clearQuadVbh = BGFX_INVALID_HANDLE;
    // Per-cell index ranges into casterIbh (first index, count) — the
    // mesh is built cell-contiguous precisely so face renders can draw
    // only the cells a face can see instead of the whole mission (at 32
    // moving lights the full-mesh draw was ~6M tris/frame).
    std::vector<std::pair<uint32_t, uint32_t>> cellIndexRange;

    // ── Per-light derived state (index-parallel with wr.staticLights) ──
    // Reach via physicalReachRadius with the SAME anchor/emitter parameters
    // the bake uses (kSubQuantThreshold is shared — handoff invariant #8).
    // Slot 0 (sun) and lights that resolve to no cell get reach 0: they
    // cast nothing under light→surface semantics (open item, handoff §4.7).
    // lightAnchor is the throw-derived per-light anchor table
    // (physicalAnchors) — the ONE authority ObjectIlluminator's falloff
    // mirror reads too.
    std::vector<float>   lightReach;
    std::vector<float>   lightAnchor;
    std::vector<int32_t> lightCell;

    // ── Pool ──
    std::vector<ShadowSlot> slots;

    // ── S4b dynamic casters (door leaves) ──
    // Bodies whose OBBs join every face render as boxes, and whose
    // poses feed the validity hash — a light re-renders its faces when
    // a door inside its reach moves. Set once after the door census.
    const ObjectCollisionWorld *dynCasterWorld = nullptr;
    std::vector<uint32_t>       dynCasterBodies;

    // Retained dynamic VBs for the caster boxes. These were transient
    // buffers, and the per-frame transient pool DROPPED them under
    // pressure (silently, until 2026-08-08): a face rendered without
    // its doors bakes door-transparent overlays and feeds the S4c
    // differential a phantom across the light's whole reach — the
    // "artifacts where the door isn't" bug. Shadow correctness must not
    // share a budget with cosmetic transient users. Ring, one entry per
    // face-render CALL (all 6 faces of a call share one buffer): bgfx
    // dynamic-VB updates are last-writer-wins within a frame, so
    // concurrent renders need distinct buffers. Entries created lazily.
    static constexpr uint32_t kDynBoxRing = 32;
    std::vector<bgfx::DynamicVertexBufferHandle> dynBoxVbh;
    uint32_t dynBoxCursor = 0;

    // ── Stats (cumulative; the startup line and the HUD read these) ──
    uint32_t facesRendered = 0;
    uint32_t facesCulled   = 0;
    uint32_t lightRenders  = 0;

    bool valid() const { return bgfx::isValid(fb); }
};

// ── Caster mesh ─────────────────────────────────────────────────────────────
// Every solid polygon of every cell, fan-triangulated, positions only.
// Includes sky (249) and flat/null-textured polygons — see the header
// comment for why this must NOT reuse the render mesh.
inline void buildShadowCasterMesh(
    const WRParsedData &wr, std::vector<ShadowCasterVertex> &verts,
    std::vector<uint32_t> &indices,
    std::vector<std::pair<uint32_t, uint32_t>> *cellRanges = nullptr) {
    if (cellRanges) cellRanges->assign(wr.numCells, {0u, 0u});
    for (uint32_t ci = 0; ci < wr.numCells; ++ci) {
        const uint32_t cellFirst = static_cast<uint32_t>(indices.size());
        const auto &cell = wr.cells[ci];
        const int numSolid = cell.numPolygons - cell.numPortals;
        for (int pi = 0; pi < numSolid; ++pi) {
            const auto &poly = cell.polygons[pi];
            const auto &idx  = cell.polyIndices[pi];
            if (poly.count < 3) continue;
            const uint32_t base = static_cast<uint32_t>(verts.size());
            int emitted = 0;
            for (int k = 0; k < poly.count && k < static_cast<int>(idx.size());
                 ++k) {
                if (idx[k] >= cell.vertices.size()) continue;
                const Vector3 &p = cell.vertices[idx[k]];
                verts.push_back({p.x, p.y, p.z});
                ++emitted;
            }
            for (int k = 1; k + 1 < emitted; ++k) {
                indices.push_back(base);
                indices.push_back(base + k);
                indices.push_back(base + k + 1);
            }
        }
        if (cellRanges)
            (*cellRanges)[ci] = {
                cellFirst,
                static_cast<uint32_t>(indices.size()) - cellFirst};
    }
}

// ── Init / destroy ──────────────────────────────────────────────────────────
// `anchor`, `emitterA`, `brightScale` MUST be the bake's falloff parameters
// (cfg.rebakeFalloffAnchor / cfg.rebakeEmitter / BakeFormula::brightScale)
// so the reach spheres here agree with the bake's — invariant #8. The two
// programs are created by the caller (embedded-shader table access lives
// with the other program creation in DarknessRenderInit.h); the cache owns
// and destroys them.
inline void initShadowMapCache(ShadowMapCache &c, const WRParsedData &wr,
                               float anchor, float emitterA, float brightScale,
                               float throwAlpha, int faceSize,
                               bgfx::ProgramHandle depthProgram,
                               bgfx::ProgramHandle debugProgram,
                               bgfx::ProgramHandle clearProgram =
                                   BGFX_INVALID_HANDLE) {
    c.faceSize  = std::max(64, faceSize);
    c.slotCount = kShadowMaxPoolSlots;

    const int tiles = c.slotCount * kShadowFaceCount;
    c.tilesPerRow = 1;
    while (c.tilesPerRow * c.tilesPerRow < tiles) ++c.tilesPerRow;
    const int rows = (tiles + c.tilesPerRow - 1) / c.tilesPerRow;
    c.atlasW = c.tilesPerRow * c.faceSize;
    c.atlasH = rows * c.faceSize;

    // Per-light reach + containing cell — CPU-only, computed BEFORE the GPU
    // caps gate below: ObjectIlluminator's physical-falloff mirror reads
    // this table, and object lighting must stay coherent with the re-baked
    // walls even on a backend that cannot host the shadow atlas.
    const size_t n = wr.staticLights.size();
    c.lightReach.assign(n, 0.0f);
    c.lightCell.assign(n, -1);
    // Throw-derived per-light anchors — the same table the bake builds
    // (physicalAnchors), so reach spheres, faces and object lighting all
    // agree on each light's intensity.
    const bool throwMode = throwAlpha > 0.0f;
    c.lightAnchor = physicalAnchors(wr, throwAlpha, anchor);
    int inWorld = 0, noCell = 0, reachable = 0;
    for (size_t li = 1; li < n; ++li) {   // slot 0 = sun: no omni faces
        const WRStaticLight &L = wr.staticLights[li];
        c.lightCell[li] = findCameraCell(wr, L.loc.x, L.loc.y, L.loc.z);
        if (c.lightCell[li] < 0) { ++noCell; continue; }
        ++inWorld;
        c.lightReach[li] = physicalReachRadius(
            L, c.lightAnchor[li], emitterA, brightScale,
            /*clampToAuthored=*/!throwMode);
        if (c.lightReach[li] > 0.0f) ++reachable;
    }

    const bgfx::Caps *caps = bgfx::getCaps();
    const bool r32fFb = (caps->formats[bgfx::TextureFormat::R32F] &
                         BGFX_CAPS_FORMAT_TEXTURE_FRAMEBUFFER) != 0;
    if (!r32fFb || c.atlasW > static_cast<int>(caps->limits.maxTextureSize) ||
        c.atlasH > static_cast<int>(caps->limits.maxTextureSize)) {
        std::fprintf(stderr,
            "[FALLBACK] shadow cache: backend cannot host the face atlas "
            "(R32F-renderable=%d, %dx%d vs max %u) — shadow maps DISABLED. "
            "S2+ stages will have no oracle on this machine.\n",
            r32fFb ? 1 : 0, c.atlasW, c.atlasH, caps->limits.maxTextureSize);
        // The caller handed us the programs; take ownership so teardown is
        // uniform even on the disabled path.
        c.depthProgram = depthProgram;
        c.debugProgram = debugProgram;
        c.clearProgram = clearProgram;
        return;
    }

    // Point sampling: the shadow test compares ONE stored distance against
    // one computed distance — filtering across a depth discontinuity would
    // invent occluders that exist in no direction (PCF, when it comes,
    // filters the COMPARISON RESULTS, never the distances).
    const uint64_t sampFlags = BGFX_SAMPLER_MIN_POINT | BGFX_SAMPLER_MAG_POINT |
                               BGFX_SAMPLER_MIP_POINT | BGFX_SAMPLER_U_CLAMP |
                               BGFX_SAMPLER_V_CLAMP;
    c.atlasTex = bgfx::createTexture2D(
        static_cast<uint16_t>(c.atlasW), static_cast<uint16_t>(c.atlasH),
        false, 1, bgfx::TextureFormat::R32F, BGFX_TEXTURE_RT | sampFlags);
    c.depthTex = bgfx::createTexture2D(
        static_cast<uint16_t>(c.atlasW), static_cast<uint16_t>(c.atlasH),
        false, 1, bgfx::TextureFormat::D32F, BGFX_TEXTURE_RT_WRITE_ONLY);
    const bgfx::TextureHandle att[2] = {c.atlasTex, c.depthTex};
    c.fb = bgfx::createFrameBuffer(2, att, false);

    c.depthProgram = depthProgram;
    c.debugProgram = debugProgram;
    c.clearProgram = clearProgram;
    c.u_shadowLightPos = bgfx::createUniform("u_shadowLightPos",
                                             bgfx::UniformType::Vec4);
    c.u_shadowFaceMtx = bgfx::createUniform("u_shadowFaceMtx",
                                            bgfx::UniformType::Mat4);
    c.s_shadowAtlas = bgfx::createUniform("s_texColor",
                                          bgfx::UniformType::Sampler);

    // Caster mesh.
    ShadowCasterVertex::init();
    std::vector<ShadowCasterVertex> verts;
    std::vector<uint32_t> indices;
    buildShadowCasterMesh(wr, verts, indices, &c.cellIndexRange);
    if (!indices.empty()) {
        c.casterVbh = bgfx::createVertexBuffer(
            bgfx::copy(verts.data(),
                       static_cast<uint32_t>(verts.size() *
                                             sizeof(ShadowCasterVertex))),
            ShadowCasterVertex::layout);
        c.casterIbh = bgfx::createIndexBuffer(
            bgfx::copy(indices.data(),
                       static_cast<uint32_t>(indices.size() * sizeof(uint32_t))),
            BGFX_BUFFER_INDEX32);
        c.casterIndexCount = static_cast<uint32_t>(indices.size());
    }

    // Tile-clear quad: unit NDC quad, placed per draw by the remap in
    // u_shadowFaceMtx (see vs_shadow_clear).
    {
        const ShadowCasterVertex q[6] = {
            {-1.0f, -1.0f, 0.0f}, {1.0f, -1.0f, 0.0f}, {1.0f, 1.0f, 0.0f},
            {-1.0f, -1.0f, 0.0f}, {1.0f, 1.0f, 0.0f}, {-1.0f, 1.0f, 0.0f}};
        c.clearQuadVbh = bgfx::createVertexBuffer(
            bgfx::copy(q, sizeof(q)), ShadowCasterVertex::layout);
    }

    c.slots.assign(c.slotCount, ShadowSlot{});

    std::fprintf(stderr,
        "Shadow cache: %dx%d R32F atlas (+D32F) = %.0f MB, %d slots x 6 "
        "faces @ %d^2 | casters %u tris (%zu solid-poly verts, incl. sky + "
        "flat) | lights: %d in-world (%d castable, %d embedded-in-solid "
        "cast nothing)\n",
        c.atlasW, c.atlasH,
        2.0 * c.atlasW * c.atlasH * 4.0 / (1024.0 * 1024.0),
        c.slotCount, c.faceSize, c.casterIndexCount / 3, verts.size(),
        inWorld, reachable, noCell);
}

inline void destroyShadowMapCache(ShadowMapCache &c) {
    for (auto &h : c.dynBoxVbh)
        if (bgfx::isValid(h)) { bgfx::destroy(h); h = BGFX_INVALID_HANDLE; }
    c.dynBoxVbh.clear();
    if (bgfx::isValid(c.clearProgram)) {
        bgfx::destroy(c.clearProgram);
        c.clearProgram = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(c.clearQuadVbh)) {
        bgfx::destroy(c.clearQuadVbh);
        c.clearQuadVbh = BGFX_INVALID_HANDLE;
    }
    if (bgfx::isValid(c.u_shadowFaceMtx)) {
        bgfx::destroy(c.u_shadowFaceMtx);
        c.u_shadowFaceMtx = BGFX_INVALID_HANDLE;
    }
    auto kill = [](auto &h) {
        if (bgfx::isValid(h)) { bgfx::destroy(h); h.idx = bgfx::kInvalidHandle; }
    };
    kill(c.fb);
    kill(c.atlasTex);
    kill(c.depthTex);
    kill(c.casterVbh);
    kill(c.casterIbh);
    kill(c.depthProgram);
    kill(c.debugProgram);
    kill(c.u_shadowLightPos);
    kill(c.s_shadowAtlas);
    c.slots.clear();
}

// Pose hash of the dynamic casters within a light's reach — the
// `ShadowCastersAreUnchanged` term of the HPL2 validity pattern, finally
// fed. FNV over body index + world transform floats.
inline uint64_t shadowDynamicCasterHash(const ShadowMapCache &c,
                                        const Vector3 &pos, float reach) {
    if (!c.dynCasterWorld) return 0;
    uint64_t h = 1469598103934665603ull;
    auto mix = [&h](const void *p, size_t n) {
        const uint8_t *b = static_cast<const uint8_t *>(p);
        for (size_t i = 0; i < n; ++i)
            h = (h ^ b[i]) * 1099511628211ull;
    };
    const auto &bodies = c.dynCasterWorld->getBodies();
    for (uint32_t bi : c.dynCasterBodies) {
        if (bi >= bodies.size()) continue;
        const ObjectCollisionBody &b = bodies[bi];
        if (b.removed) continue;
        const float maxEdge = std::max(
            {b.edgeLengths.x, b.edgeLengths.y, b.edgeLengths.z});
        if (glm::length(b.worldPos - pos) - maxEdge > reach) continue;
        mix(&bi, sizeof(bi));
        mix(&b.worldPos, sizeof(b.worldPos));
        mix(&b.rotation, sizeof(b.rotation));
    }
    return h;
}

// Is the segment blocked by a dynamic caster? The CPU cross-check's door
// term — the same bodies the GPU faces draw, so the two oracles keep
// answering the same question.
inline bool shadowDynamicSegmentBlocked(const ShadowMapCache &c,
                                        const Vector3 &a, const Vector3 &b) {
    if (!c.dynCasterWorld) return false;
    for (uint32_t bi : c.dynCasterBodies)
        if (c.dynCasterWorld->segmentHitsBody(a, b, bi)) return true;
    return false;
}

// ── Face rendering ──────────────────────────────────────────────────────────

// S4c: a dynamic caster drawn at a pose OTHER than its body's current one
// — the frozen-reference faces render the promoted door at the pose the
// baked overlay was last baked at, not where the leaf is now.
struct ShadowCasterPoseOverride {
    uint32_t  bodyIdx;
    Vector3   worldPos;
    glm::mat3 rotation;
};

// Render the six faces of `lightIdx` into `slot`'s tiles. Faces whose 90°
// frustum misses every cell bounding sphere within the light's reach are
// not drawn — their tiles are still CLEARED (to 1.0 = "no occluder"),
// because a culled face's tile would otherwise keep a previous occupant's
// distances and lookups into it would read another light's scene.
// Core face render for a light at an arbitrary position — static lights
// and transients (live emitters, S4c door promotions) share it.
inline void renderShadowFacesAt(ShadowMapCache &c, const WRParsedData &wr,
                                const Vector3 &lightPos, float reach,
                                int slot,
                                const std::vector<ShadowCasterPoseOverride>
                                    *poseOv = nullptr) {
    const float farZ = std::max(reach, c.nearZ * 2.0f);
    const bool homoDepth = bgfx::getCaps()->homogeneousDepth;

    // Candidate cells once per light (ascending index — the range merge
    // below depends on it); per-face frustum test at draw time.
    std::vector<uint32_t> nearCells;
    for (uint32_t ci = 0; ci < wr.numCells; ++ci) {
        const WRParsedCell &cell = wr.cells[ci];
        if (glm::length(cell.center - lightPos) - cell.radius <= reach)
            nearCells.push_back(ci);
    }

    // S4b: dynamic casters (door leaves) within reach, drawn as transient
    // boxes into every face after the static mesh. Overridden bodies
    // (S4c frozen references) take their pose from the override, not the
    // live body.
    auto casterPose = [&](uint32_t bi, const ObjectCollisionBody &b,
                          Vector3 &pos, glm::mat3 &rot) {
        pos = b.worldPos;
        rot = b.rotation;
        if (poseOv)
            for (const ShadowCasterPoseOverride &ov : *poseOv)
                if (ov.bodyIdx == bi) {
                    pos = ov.worldPos;
                    rot = ov.rotation;
                    return;
                }
    };
    // Box soup for the in-reach doors, built ONCE per call (identical
    // for all six faces) and uploaded to a retained ring buffer — never
    // the per-frame transient pool (see dynBoxVbh).
    uint32_t dynBoxCount = 0;
    bgfx::DynamicVertexBufferHandle dynVbh = BGFX_INVALID_HANDLE;
    if (c.dynCasterWorld) {
        std::vector<ShadowCasterVertex> soup;
        const auto &bodies = c.dynCasterWorld->getBodies();
        for (uint32_t bi : c.dynCasterBodies) {
            if (bi >= bodies.size() || bodies[bi].removed) continue;
            const ObjectCollisionBody &b = bodies[bi];
            Vector3 bp; glm::mat3 br;
            casterPose(bi, b, bp, br);
            const float maxEdge = std::max(
                {b.edgeLengths.x, b.edgeLengths.y, b.edgeLengths.z});
            if (glm::length(bp - lightPos) - maxEdge > reach) continue;
            const Vector3 he = b.edgeLengths * 0.5f;
            const Vector3 ax = Vector3(br[0]) * he.x;
            const Vector3 ay = Vector3(br[1]) * he.y;
            const Vector3 az = Vector3(br[2]) * he.z;
            Vector3 corner[8];
            for (int k = 0; k < 8; ++k)
                corner[k] = bp +
                    ax * ((k & 1) ? 1.0f : -1.0f) +
                    ay * ((k & 2) ? 1.0f : -1.0f) +
                    az * ((k & 4) ? 1.0f : -1.0f);
            // 12 triangles over the 6 box faces (winding irrelevant —
            // the pass renders two-sided).
            static const int tri[36] = {
                0,1,3, 0,3,2,  4,6,7, 4,7,5,
                0,4,5, 0,5,1,  2,3,7, 2,7,6,
                0,2,6, 0,6,4,  1,5,7, 1,7,3};
            for (int k = 0; k < 36; ++k) {
                const Vector3 &pq = corner[tri[k]];
                soup.push_back({pq.x, pq.y, pq.z});
            }
        }
        if (!soup.empty()) {
            if (c.dynBoxVbh.empty())
                c.dynBoxVbh.assign(ShadowMapCache::kDynBoxRing,
                                   BGFX_INVALID_HANDLE);
            auto &slot2 = c.dynBoxVbh[c.dynBoxCursor];
            c.dynBoxCursor =
                (c.dynBoxCursor + 1) % ShadowMapCache::kDynBoxRing;
            if (!bgfx::isValid(slot2))
                slot2 = bgfx::createDynamicVertexBuffer(
                    static_cast<uint32_t>(soup.size()),
                    ShadowCasterVertex::layout,
                    BGFX_BUFFER_ALLOW_RESIZE);
            if (bgfx::isValid(slot2)) {
                bgfx::update(slot2, 0,
                             bgfx::copy(soup.data(),
                                        static_cast<uint32_t>(
                                            soup.size() *
                                            sizeof(ShadowCasterVertex))));
                dynVbh = slot2;
                dynBoxCount = static_cast<uint32_t>(soup.size());
            } else {
                std::fprintf(stderr,
                    "[FALLBACK] shadow face render: dynamic caster VB "
                    "unavailable — %zu box vert(s) MISSING from slot "
                    "%d\n", soup.size(), slot);
            }
        }
    }

    // ── The single shadow view ──
    // Every face of every slot renders here (the per-slot view scheme hit
    // bgfx's 256-view budget at 15 differentials). Viewport = the whole
    // atlas; each draw places its tile via a clip-space remap composed
    // into u_shadowFaceMtx plus a matching scissor. Sequential mode:
    // the clear quad MUST precede its tile's geometry.
    const bgfx::ViewId vid = kViewShadowFaces;
    bgfx::setViewFrameBuffer(vid, c.fb);
    bgfx::setViewRect(vid, 0, 0, static_cast<uint16_t>(c.atlasW),
                      static_cast<uint16_t>(c.atlasH));
    bgfx::setViewClear(vid, BGFX_CLEAR_NONE);
    bgfx::setViewMode(vid, bgfx::ViewMode::Sequential);

    const float lp[4] = {lightPos.x, lightPos.y, lightPos.z,
                         reach > 0.0f ? 1.0f / reach : 0.0f};

    for (int face = 0; face < kShadowFaceCount; ++face) {
        int ox = 0, oy = 0;
        shadowAtlasTileOrigin(slot, face, c.tilesPerRow, c.faceSize, ox, oy);

        // Clip-space tile placement: tile-local NDC → this tile's patch
        // of the full-atlas viewport (top-left origin, +y up in NDC —
        // the same mapping setViewRect used to provide).
        Matrix4 remap(1.0f);
        remap[0][0] = static_cast<float>(c.faceSize) / c.atlasW;
        remap[1][1] = static_cast<float>(c.faceSize) / c.atlasH;
        remap[3][0] =
            (2.0f * ox + c.faceSize) / static_cast<float>(c.atlasW) - 1.0f;
        remap[3][1] =
            1.0f - (2.0f * oy + c.faceSize) / static_cast<float>(c.atlasH);

        // Tile clear: quad through the remap alone at far depth
        // (per-view clears cannot scope to a tile). Runs for CULLED
        // faces too — their tiles must read 1.0, not a previous
        // occupant's distances.
        bgfx::setScissor(static_cast<uint16_t>(ox),
                         static_cast<uint16_t>(oy),
                         static_cast<uint16_t>(c.faceSize),
                         static_cast<uint16_t>(c.faceSize));
        bgfx::setUniform(c.u_shadowFaceMtx, glm::value_ptr(remap));
        bgfx::setVertexBuffer(0, c.clearQuadVbh);
        bgfx::setState(BGFX_STATE_WRITE_R | BGFX_STATE_WRITE_Z |
                       BGFX_STATE_DEPTH_TEST_ALWAYS);
        bgfx::submit(vid, c.clearProgram);

        // Per-face visible cells → merged index ranges. Gap-tolerant
        // merge bounds the draw count; if the face still needs too many
        // ranges, one full-mesh draw is never WORSE than the old path.
        std::vector<std::pair<uint32_t, uint32_t>> ranges;
        for (uint32_t ci : nearCells) {
            const WRParsedCell &cell = wr.cells[ci];
            if (!shadowFaceSeesSphere(lightPos, face, cell.center,
                                      cell.radius, reach))
                continue;
            if (ci >= c.cellIndexRange.size()) continue;
            const auto &r = c.cellIndexRange[ci];
            if (r.second == 0) continue;
            if (!ranges.empty() &&
                r.first - (ranges.back().first + ranges.back().second) <=
                    4096)
                ranges.back().second =
                    r.first + r.second - ranges.back().first;
            else
                ranges.push_back(r);
        }
        if (ranges.empty() || c.casterIndexCount == 0) {
            ++c.facesCulled;
            continue;   // tile stays cleared = 1.0
        }
        if (ranges.size() > 24)
            ranges.assign(1, {0u, c.casterIndexCount});

        const Matrix4 view = shadowFaceView(lightPos, face);
        // The backend's clip convention decides the projection variant —
        // render with the wrong one and the backend clips half the depth
        // range away (see shadowFaceProjZO). The UV math is identical in
        // both; only clip/ordering z differs.
        const Matrix4 proj = homoDepth ? shadowFaceProj(c.nearZ, farZ)
                                       : shadowFaceProjZO(c.nearZ, farZ);
        const Matrix4 faceMtx = remap * proj * view;

        for (const auto &r : ranges) {
            bgfx::setScissor(static_cast<uint16_t>(ox),
                             static_cast<uint16_t>(oy),
                             static_cast<uint16_t>(c.faceSize),
                             static_cast<uint16_t>(c.faceSize));
            bgfx::setUniform(c.u_shadowFaceMtx, glm::value_ptr(faceMtx));
            bgfx::setUniform(c.u_shadowLightPos, lp);
            bgfx::setVertexBuffer(0, c.casterVbh);
            bgfx::setIndexBuffer(c.casterIbh, r.first, r.second);
            // Two-sided: the raycaster stops at solid polygons regardless
            // of approach side, so the caster mesh must too. Depth test
            // resolves the closest surface; R carries linear distance.
            bgfx::setState(BGFX_STATE_WRITE_R | BGFX_STATE_WRITE_Z |
                           BGFX_STATE_DEPTH_TEST_LESS);
            bgfx::submit(vid, c.depthProgram);
        }

        // Dynamic casters: the ring-buffered box soup built above.
        if (dynBoxCount > 0 && bgfx::isValid(dynVbh)) {
            bgfx::setScissor(static_cast<uint16_t>(ox),
                             static_cast<uint16_t>(oy),
                             static_cast<uint16_t>(c.faceSize),
                             static_cast<uint16_t>(c.faceSize));
            bgfx::setUniform(c.u_shadowFaceMtx, glm::value_ptr(faceMtx));
            bgfx::setUniform(c.u_shadowLightPos, lp);
            bgfx::setVertexBuffer(0, dynVbh, 0, dynBoxCount);
            bgfx::setState(BGFX_STATE_WRITE_R | BGFX_STATE_WRITE_Z |
                           BGFX_STATE_DEPTH_TEST_LESS);
            bgfx::submit(vid, c.depthProgram);
        }
        ++c.facesRendered;
    }
}

// Static-light wrapper: bookkeeping + the caster-pose validity hash.
inline void renderShadowFaces(ShadowMapCache &c, const WRParsedData &wr,
                              int lightIdx, int slot, uint32_t frameIndex) {
    const WRStaticLight &L = wr.staticLights[lightIdx];
    const float reach = c.lightReach[lightIdx];
    renderShadowFacesAt(c, wr, L.loc, reach, slot);
    ShadowSlot &s = c.slots[slot];
    s.lightIdx = lightIdx;
    s.pos = L.loc;
    s.bright = L.bright;
    s.reach = reach;
    s.casterHash = shadowDynamicCasterHash(c, L.loc, reach);
    s.lastUsed = frameIndex;
    s.rendered = true;
    ++c.lightRenders;
}

// Ensure `lightIdx` has valid faces in the pool; returns its slot, or -1
// if the light cannot cast (no reach / outside the world / cache disabled).
// Validity is the HPL2 `ShadowMapNeedsUpdate` check (attribution in the
// header comment): identity + transform + radius + caster set. Static
// lights currently only go stale through S3/S4 (casters, position jitter),
// but the fields are compared, not assumed — a light whose table entry
// moved or re-brightened re-renders without any new code.
inline int ensureShadowLight(ShadowMapCache &c, const WRParsedData &wr,
                             int lightIdx, uint32_t frameIndex) {
    if (!c.valid()) return -1;
    if (lightIdx <= 0 ||
        lightIdx >= static_cast<int>(wr.staticLights.size())) return -1;
    if (c.lightReach[lightIdx] <= 0.0f || c.lightCell[lightIdx] < 0) return -1;

    const WRStaticLight &L = wr.staticLights[lightIdx];

    int freeSlot = -1, lruSlot = 0;
    uint32_t lruAge = 0xffffffffu;
    for (int i = 0; i < c.slotCount; ++i) {
        ShadowSlot &s = c.slots[i];
        if (s.lightIdx == lightIdx) {
            const bool valid = s.rendered && s.pos == L.loc &&
                               s.bright == L.bright &&
                               s.reach == c.lightReach[lightIdx] &&
                               s.casterHash == shadowDynamicCasterHash(
                                   c, L.loc, c.lightReach[lightIdx]);
            if (valid) {
                s.lastUsed = frameIndex;
                return i;
            }
            renderShadowFaces(c, wr, lightIdx, i, frameIndex);
            return i;
        }
        if (s.pinned) continue;   // frozen reference — never steal
        if (s.lightIdx < 0 && freeSlot < 0) freeSlot = i;
        if (s.lastUsed < lruAge) { lruAge = s.lastUsed; lruSlot = i; }
    }
    if (freeSlot < 0 && lruAge == 0xffffffffu) return -1;  // all pinned

    const int slot = freeSlot >= 0 ? freeSlot : lruSlot;
    renderShadowFaces(c, wr, lightIdx, slot, frameIndex);
    return slot;
}

// Transient lights (live emitters, S4c door promotions when a light
// must track a moving emitter) — identified by caller-chosen NEGATIVE
// ids so they can never collide with static-table indices. A moving
// light re-renders every frame (pos changes); a parked one goes valid
// like any other slot.
inline int ensureDynamicShadowLight(ShadowMapCache &c,
                                    const WRParsedData &wr, int id,
                                    const Vector3 &pos, float reach,
                                    uint32_t frameIndex) {
    if (!c.valid() || id >= 0 || reach <= 0.0f) return -1;
    int freeSlot = -1, lruSlot = 0;
    uint32_t lruAge = 0xffffffffu;
    for (int i = 0; i < c.slotCount; ++i) {
        ShadowSlot &s = c.slots[i];
        if (s.lightIdx == id) {
            const bool valid = s.rendered && s.pos == pos &&
                               s.reach == reach &&
                               s.casterHash ==
                                   shadowDynamicCasterHash(c, pos, reach);
            if (valid) {
                s.lastUsed = frameIndex;
                return i;
            }
            renderShadowFacesAt(c, wr, pos, reach, i);
            s.lightIdx = id;
            s.pos = pos;
            s.bright = Vector3(0.0f);
            s.reach = reach;
            s.casterHash = shadowDynamicCasterHash(c, pos, reach);
            s.lastUsed = frameIndex;
            s.rendered = true;
            ++c.lightRenders;
            return i;
        }
        if (s.pinned) continue;   // frozen reference — never steal
        if (s.lightIdx < 0 && s.lightIdx != id && freeSlot < 0 &&
            !s.rendered)
            freeSlot = i;
        if (s.lastUsed < lruAge) { lruAge = s.lastUsed; lruSlot = i; }
    }
    if (freeSlot < 0 && lruAge == 0xffffffffu) return -1;  // all pinned
    const int slot = freeSlot >= 0 ? freeSlot : lruSlot;
    renderShadowFacesAt(c, wr, pos, reach, slot);
    ShadowSlot &s = c.slots[slot];
    s.lightIdx = id;
    s.pos = pos;
    s.bright = Vector3(0.0f);
    s.reach = reach;
    s.casterHash = shadowDynamicCasterHash(c, pos, reach);
    s.lastUsed = frameIndex;
    s.rendered = true;
    ++c.lightRenders;
    return slot;
}

// ── S4c frozen references ───────────────────────────────────────────────────
// A frozen slot holds a promoted light's faces with the door casters at
// the LAST-BAKED pose (poseOv) — the reference the differential live term
// subtracts. It renders ONCE per acquire and is pinned: the ensure
// functions must neither steal nor refresh it (its hash is deliberately
// left at 0 — no validity machinery applies; the owner re-acquires when
// the baked pose changes and releases when the promotion ends).
inline int acquireFrozenShadowSlot(ShadowMapCache &c, const WRParsedData &wr,
                                   int id, const Vector3 &pos, float reach,
                                   const std::vector<ShadowCasterPoseOverride>
                                       &poseOv,
                                   uint32_t frameIndex) {
    if (!c.valid() || id >= 0 || reach <= 0.0f) return -1;
    int freeSlot = -1, lruSlot = -1, existing = -1;
    uint32_t lruAge = 0xffffffffu;
    for (int i = 0; i < c.slotCount; ++i) {
        ShadowSlot &s = c.slots[i];
        if (s.lightIdx == id) { existing = i; break; }
        if (s.pinned) continue;
        if (s.lightIdx < 0 && freeSlot < 0) freeSlot = i;
        if (s.lastUsed < lruAge) { lruAge = s.lastUsed; lruSlot = i; }
    }
    const int slot = existing >= 0 ? existing
                   : freeSlot >= 0 ? freeSlot : lruSlot;
    if (slot < 0) return -1;   // pool entirely pinned
    renderShadowFacesAt(c, wr, pos, reach, slot, &poseOv);
    ShadowSlot &s = c.slots[slot];
    s.lightIdx = id;
    s.pinned = true;
    s.pos = pos;
    s.bright = Vector3(0.0f);
    s.reach = reach;
    s.casterHash = 0;
    s.lastUsed = frameIndex;
    s.rendered = true;
    ++c.lightRenders;
    return slot;
}

inline void releaseShadowSlot(ShadowMapCache &c, int id) {
    for (ShadowSlot &s : c.slots)
        if (s.lightIdx == id) {
            s = ShadowSlot{};
            return;
        }
}

// Keep a pinned/held slot warm in the LRU without touching its faces.
inline void touchShadowSlot(ShadowMapCache &c, int slot,
                            uint32_t frameIndex) {
    if (slot >= 0 && slot < c.slotCount)
        c.slots[slot].lastUsed = frameIndex;
}

// ── CPU-side atlas lookup (readback consumers: the cross-check) ────────────
// Mirrors what the S4 shader lookup will do, against a CPU copy of the
// atlas. Returns the stored reach-normalised distance at the nearest texel,
// or -1 if the point projects outside its covering face (cannot happen for
// points chosen by shadowFaceForDirection, kept as a guard).
//
// Row convention: shadowAtlasTileOrigin and setViewRect share bgfx's
// top-left origin, and clip-space +y (our v) rasterises to the TOP of the
// viewport. readTexture returns rows in texture-memory order, which is
// top-down on Metal/D3D/Vulkan but BOTTOM-UP on OpenGL — the
// `originBottomLeft` flip below. Getting this wrong reads a mirrored face
// and the cross-check collapses toward 50%, which is exactly the failure
// the cross-check exists to catch.
inline float shadowAtlasSampleCPU(const ShadowMapCache &c,
                                  const std::vector<float> &atlasData,
                                  int slot, const Vector3 &lightPos,
                                  const Vector3 &point) {
    const int face = shadowFaceForDirection(point - lightPos);
    float u = 0.0f, v = 0.0f;
    if (!shadowFaceUV(lightPos, face, point, u, v)) return -1.0f;
    int ox = 0, oy = 0;
    shadowAtlasTileOrigin(slot, face, c.tilesPerRow, c.faceSize, ox, oy);
    const int fs = c.faceSize;
    int tx = std::min(fs - 1, std::max(0, static_cast<int>(u * fs)));
    int tyTop = std::min(fs - 1, std::max(0, static_cast<int>((1.0f - v) * fs)));
    int row = oy + tyTop;                      // visual (top-left) row
    if (bgfx::getCaps()->originBottomLeft)
        row = c.atlasH - 1 - row;              // GL texture memory is bottom-up
    return atlasData[static_cast<size_t>(row) * c.atlasW + ox + tx];
}

// Blocking whole-atlas readback. Runs its own bgfx frames until the data
// lands, so it is for STARTUP diagnostics (the cross-check), never the
// frame loop. Returns false (loudly) when the backend cannot blit+read.
inline bool readShadowAtlasBlocking(const ShadowMapCache &c,
                                    std::vector<float> &out) {
    const bgfx::Caps *caps = bgfx::getCaps();
    if (!(caps->supported & BGFX_CAPS_TEXTURE_BLIT) ||
        !(caps->supported & BGFX_CAPS_TEXTURE_READ_BACK)) {
        std::fprintf(stderr,
            "[FALLBACK] shadow readback: backend lacks TEXTURE_BLIT/"
            "READ_BACK — cross-check unavailable on this machine.\n");
        return false;
    }
    bgfx::TextureHandle staging = bgfx::createTexture2D(
        static_cast<uint16_t>(c.atlasW), static_cast<uint16_t>(c.atlasH),
        false, 1, bgfx::TextureFormat::R32F,
        BGFX_TEXTURE_BLIT_DST | BGFX_TEXTURE_READ_BACK);
    // The blit lives on kViewShadowDebug: after every face render, so face
    // submits from THIS frame are in the atlas before the copy runs.
    bgfx::blit(kViewShadowDebug, staging, 0, 0, c.atlasTex, 0, 0,
               static_cast<uint16_t>(c.atlasW),
               static_cast<uint16_t>(c.atlasH));
    out.assign(static_cast<size_t>(c.atlasW) * c.atlasH, 0.0f);
    const uint32_t ready = bgfx::readTexture(staging, out.data());
    uint32_t cur = bgfx::frame();
    while (cur < ready) cur = bgfx::frame();
    bgfx::destroy(staging);
    return true;
}

// ── S1 acceptance: the readback cross-check ────────────────────────────────
// N random (point, light) pairs: face-lookup distance compare vs
// raycastWorld, the two independent implementations of the same visibility
// question. Acceptance ≥ 99% agreement, with disagreements ATTRIBUTED —
// the bias is then tuned against the printed sweep rather than by eye
// (plan §S1). Deterministic (bakeRand01), so a regression is a diff, not
// a mood.
inline void runShadowCrossCheck(ShadowMapCache &c, const WRParsedData &wr,
                                int pairCount) {
    if (!c.valid()) {
        std::fprintf(stderr, "[SHADOW_XCHECK] cache invalid — skipped\n");
        return;
    }

    // Castable lights, stride-sampled across the table so the batch spans
    // the mission rather than clustering in whatever cell the table starts
    // in. One pool's worth per run.
    std::vector<int> candidates;
    for (int li = 1; li < static_cast<int>(wr.staticLights.size()); ++li)
        if (c.lightReach[li] > 0.0f && c.lightCell[li] >= 0)
            candidates.push_back(li);
    if (candidates.empty()) {
        std::fprintf(stderr, "[SHADOW_XCHECK] no castable lights — skipped\n");
        return;
    }
    const int nLights = std::min<int>(c.slotCount,
                                      static_cast<int>(candidates.size()));
    std::vector<int> sample;
    for (int k = 0; k < nLights; ++k)
        sample.push_back(
            candidates[static_cast<size_t>(k) * candidates.size() / nLights]);

    std::vector<int> slotOf(sample.size(), -1);
    for (size_t k = 0; k < sample.size(); ++k)
        slotOf[k] = ensureShadowLight(c, wr, sample[k], /*frameIndex=*/k + 1);

    std::vector<float> atlas;
    if (!readShadowAtlasBlocking(c, atlas)) return;

    // Bias sweep, world units. The comparison is
    //   occludedGPU := stored*reach < dist(P) - bias
    const float biases[] = {0.0f, 0.1f, 0.25f, 0.5f, 1.0f, 2.0f};
    constexpr int kNumBiases = 6;
    long agree[kNumBiases] = {0};
    long total = 0, unproven = 0, outsideFace = 0;
    // Attribution at bias index 0 (0.0u): both oracles read the SAME
    // geometry here, so zero bias wins by construction (bias exists for
    // rasterized consumers comparing a surface against its own texel — S2
    // tunes it against ITS acceptance check). Classes: resolution-limited
    // (within two texel footprints at that distance), sky-polygon hits,
    // other.
    constexpr int kAttribBias = 0;
    long disResolution = 0, disSky = 0, disOther = 0;
    // Occluder-distance error where BOTH oracles saw a surface before P:
    // how well the map measures the same surface the raycaster hit.
    std::vector<float> occErr;

    uint32_t rng = 0xC0FFEEu;
    const int perLight = std::max(1, pairCount / static_cast<int>(sample.size()));
    for (size_t k = 0; k < sample.size(); ++k) {
        const int li = sample[k];
        const int slot = slotOf[k];
        if (slot < 0) continue;
        const WRStaticLight &L = wr.staticLights[li];
        const float reach = c.lightReach[li];
        for (int m = 0; m < perLight; ++m) {
            // Uniform direction (rejection), distance across the reach span.
            Vector3 dir;
            float len2;
            do {
                dir = Vector3(bakeRand01(rng) * 2.0f - 1.0f,
                              bakeRand01(rng) * 2.0f - 1.0f,
                              bakeRand01(rng) * 2.0f - 1.0f);
                len2 = glm::dot(dir, dir);
            } while (len2 < 1e-4f || len2 > 1.0f);
            dir *= 1.0f / std::sqrt(len2);
            const float dist = c.nearZ * 4.0f +
                bakeRand01(rng) * (reach * 0.98f - c.nearZ * 4.0f);
            const Vector3 P = L.loc + dir * dist;

            RayHit hit;
            raycastWorld(wr, L.loc, P, hit, nullptr, c.lightCell[li]);
            if (!rayStatusProven(hit.status)) { ++unproven; continue; }
            // Dynamic casters (door leaves) block the CPU oracle exactly
            // as they block the GPU faces — the S4b requirement that the
            // cross-check keep both sides answering the same question.
            const bool occCPU = (hit.status == RayStatus::Hit) ||
                                shadowDynamicSegmentBlocked(c, L.loc, P);

            const float stored = shadowAtlasSampleCPU(c, atlas, slot, L.loc, P);
            if (stored < 0.0f) { ++outsideFace; continue; }
            const float storedDist = stored * reach;

            ++total;
            bool agreeAtAttrib = false;
            for (int b = 0; b < kNumBiases; ++b) {
                const bool occGPU = storedDist < dist - biases[b];
                if (occGPU == occCPU) {
                    ++agree[b];
                    if (b == kAttribBias) agreeAtAttrib = true;
                }
            }
            if (!agreeAtAttrib) {
                // One texel of this face at the stored distance covers
                // 2*d/faceSize world units; within two of those, the map
                // simply cannot distinguish the surface from P.
                const float texelFootprint =
                    2.0f * std::max(storedDist, dist) / c.faceSize;
                if (std::abs(dist - storedDist) < 2.0f * texelFootprint)
                    ++disResolution;
                else if (occCPU && hit.textureIndex == 249)
                    ++disSky;
                else
                    ++disOther;
            }
            if (occCPU && storedDist < reach * 0.999f)
                occErr.push_back(std::abs(storedDist - hit.distance));
        }
    }

    if (total == 0) {
        std::fprintf(stderr, "[SHADOW_XCHECK] no comparable pairs "
                             "(unproven %ld, outside-face %ld)\n",
                     unproven, outsideFace);
        return;
    }

    std::fprintf(stderr,
        "[SHADOW_XCHECK] %d lights (of %zu castable), %ld pairs compared "
        "(%ld CPU-unproven excluded, %ld outside-face) | faces rendered %u, "
        "culled %u\n",
        nLights, candidates.size(), total, unproven, outsideFace,
        c.facesRendered, c.facesCulled);
    std::fprintf(stderr, "[SHADOW_XCHECK] bias sweep:");
    int best = 0;
    for (int b = 0; b < kNumBiases; ++b) {
        std::fprintf(stderr, "  %.2fu=%.2f%%", biases[b],
                     100.0 * agree[b] / total);
        if (agree[b] > agree[best]) best = b;
    }
    std::fprintf(stderr, "\n");
    const double bestPct = 100.0 * agree[best] / total;
    std::fprintf(stderr,
        "[SHADOW_XCHECK] best bias %.2fu -> %.2f%% agreement — acceptance "
        ">=99%%: %s\n",
        biases[best], bestPct, bestPct >= 99.0 ? "PASS" : "FAIL");
    const long disTotal = disResolution + disSky + disOther;
    std::fprintf(stderr,
        "[SHADOW_XCHECK] disagreements @%.2fu bias: %ld — %ld within map "
        "resolution, %ld sky-polygon hits, %ld other\n",
        biases[kAttribBias], disTotal, disResolution, disSky, disOther);
    if (!occErr.empty()) {
        std::sort(occErr.begin(), occErr.end());
        std::fprintf(stderr,
            "[SHADOW_XCHECK] occluder-distance |map - ray| (%zu CPU-occluded "
            "pairs): p50 %.3fu  p95 %.3fu  max %.3fu\n",
            occErr.size(), occErr[occErr.size() / 2],
            occErr[occErr.size() * 95 / 100], occErr.back());
    }

    // ── Dynamic-light leg (S4b) ──
    // ensureDynamicShadowLight + the atlas content + the CPU lookup that
    // MIRRORS the shader path, validated the same way: park a transient
    // light at the first sampled light's position (with the same reach)
    // and compare. A failure here indicts the dynamic ensure path or the
    // slot addressing; a pass here with no shadows ON SCREEN indicts the
    // shader mirror or its uniforms.
    if (!sample.empty()) {
        const int li = sample[0];
        const Vector3 dpos = wr.staticLights[li].loc;
        const float dreach = c.lightReach[li];
        const int dslot = ensureDynamicShadowLight(c, wr, -9001, dpos,
                                                   dreach, 999999u);
        if (dslot < 0) {
            std::fprintf(stderr,
                "[SHADOW_XCHECK] dynamic leg: ensureDynamicShadowLight "
                "FAILED (slot -1)\n");
        } else if (readShadowAtlasBlocking(c, atlas)) {
            long dagree = 0, dtotal = 0;
            uint32_t drng = 0xBEEF01u;
            for (int m = 0; m < 2000; ++m) {
                Vector3 dir;
                float len2;
                do {
                    dir = Vector3(bakeRand01(drng) * 2.0f - 1.0f,
                                  bakeRand01(drng) * 2.0f - 1.0f,
                                  bakeRand01(drng) * 2.0f - 1.0f);
                    len2 = glm::dot(dir, dir);
                } while (len2 < 1e-4f || len2 > 1.0f);
                dir *= 1.0f / std::sqrt(len2);
                const float dist = c.nearZ * 4.0f +
                    bakeRand01(drng) * (dreach * 0.98f - c.nearZ * 4.0f);
                const Vector3 P = dpos + dir * dist;
                RayHit hit;
                raycastWorld(wr, dpos, P, hit, nullptr, c.lightCell[li]);
                if (!rayStatusProven(hit.status)) continue;
                const bool occCPU = (hit.status == RayStatus::Hit) ||
                                    shadowDynamicSegmentBlocked(c, dpos, P);
                const float stored =
                    shadowAtlasSampleCPU(c, atlas, dslot, dpos, P);
                if (stored < 0.0f) continue;
                ++dtotal;
                if ((stored * dreach < dist) == occCPU) ++dagree;
            }
            std::fprintf(stderr,
                "[SHADOW_XCHECK] dynamic leg: slot %d, %ld pairs, "
                "%.2f%% agreement\n",
                dslot, dtotal,
                dtotal ? 100.0 * dagree / dtotal : 0.0);
        }
    }
}

// ── Debug HUD ───────────────────────────────────────────────────────────────
// Draws one light's six face tiles as a 3x2 grid of grayscale quads on the
// right of the screen (S1 acceptance: "a debug view that draws one light's
// depth faces"). Draws on kViewShadowDebug: after the face renders, direct
// to the backbuffer — a diagnostic overlay, deliberately outside the tone
// pipeline, like the debug text.
inline void submitShadowDebugHud(ShadowMapCache &c, int slot,
                                 uint32_t screenW, uint32_t screenH) {
    if (!c.valid() || slot < 0 || slot >= c.slotCount) return;
    if (!bgfx::isValid(c.debugProgram)) return;

    struct HudVertex {   // matches vs_textured's inputs (pos, color0, uv)
        float x, y, z;
        uint32_t abgr;
        float u, v;
    };
    static bgfx::VertexLayout hudLayout;
    static bool hudLayoutInit = false;
    if (!hudLayoutInit) {
        hudLayout.begin()
            .add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
            .add(bgfx::Attrib::Color0, 4, bgfx::AttribType::Uint8, true)
            .add(bgfx::Attrib::TexCoord0, 2, bgfx::AttribType::Float)
            .end();
        hudLayoutInit = true;
    }

    bgfx::setViewFrameBuffer(kViewShadowDebug, BGFX_INVALID_HANDLE);
    bgfx::setViewRect(kViewShadowDebug, 0, 0, static_cast<uint16_t>(screenW),
                      static_cast<uint16_t>(screenH));
    bgfx::setViewClear(kViewShadowDebug, BGFX_CLEAR_NONE);
    // Screen-space ortho, y-down, [0,1]² — quad coords below are fractions
    // of the window.
    const Matrix4 ident(1.0f);
    const bool homoDepth = bgfx::getCaps()->homogeneousDepth;
    const Matrix4 proj = homoDepth
        ? glm::orthoRH_NO(0.0f, 1.0f, 1.0f, 0.0f, -1.0f, 1.0f)
        : glm::orthoRH_ZO(0.0f, 1.0f, 1.0f, 0.0f, -1.0f, 1.0f);
    bgfx::setViewTransform(kViewShadowDebug, glm::value_ptr(ident),
                           glm::value_ptr(proj));

    // 3 columns x 2 rows on the right edge: +X -X +Y / -Y +Z -Z.
    const float tileW = 0.10f;
    const float gap = 0.005f;
    const float x0 = 1.0f - 3.0f * (tileW + gap);
    const float y0 = 0.03f;
    // Screen tiles are square fractions of WIDTH; correct the height by the
    // aspect so faces render square.
    const float tileH = tileW * (screenW > 0 && screenH > 0
        ? static_cast<float>(screenW) / static_cast<float>(screenH) : 1.0f);

    if (bgfx::getAvailTransientVertexBuffer(24, hudLayout) < 24) return;
    bgfx::TransientVertexBuffer tvb;
    bgfx::TransientIndexBuffer tib;
    bgfx::allocTransientVertexBuffer(&tvb, 24, hudLayout);
    bgfx::allocTransientIndexBuffer(&tib, 36);
    HudVertex *v = reinterpret_cast<HudVertex *>(tvb.data);
    uint16_t *ix = reinterpret_cast<uint16_t *>(tib.data);

    for (int face = 0; face < kShadowFaceCount; ++face) {
        int ox = 0, oy = 0;
        shadowAtlasTileOrigin(slot, face, c.tilesPerRow, c.faceSize, ox, oy);
        float u0 = static_cast<float>(ox) / c.atlasW;
        float v0 = static_cast<float>(oy) / c.atlasH;
        float u1 = static_cast<float>(ox + c.faceSize) / c.atlasW;
        float v1 = static_cast<float>(oy + c.faceSize) / c.atlasH;
        if (bgfx::getCaps()->originBottomLeft) {
            // GL stores the render target bottom-up; flip so the HUD shows
            // every backend the same image.
            v0 = 1.0f - v0;
            v1 = 1.0f - v1;
        }
        const int col = face % 3, rowi = face / 3;
        const float qx = x0 + col * (tileW + gap);
        const float qy = y0 + rowi * (tileH + gap);
        const uint32_t white = 0xffffffffu;
        const int b = face * 4;
        v[b + 0] = {qx, qy, 0.0f, white, u0, v0};
        v[b + 1] = {qx + tileW, qy, 0.0f, white, u1, v0};
        v[b + 2] = {qx + tileW, qy + tileH, 0.0f, white, u1, v1};
        v[b + 3] = {qx, qy + tileH, 0.0f, white, u0, v1};
        const int e = face * 6;
        ix[e + 0] = static_cast<uint16_t>(b + 0);
        ix[e + 1] = static_cast<uint16_t>(b + 1);
        ix[e + 2] = static_cast<uint16_t>(b + 2);
        ix[e + 3] = static_cast<uint16_t>(b + 0);
        ix[e + 4] = static_cast<uint16_t>(b + 2);
        ix[e + 5] = static_cast<uint16_t>(b + 3);
    }

    bgfx::setVertexBuffer(0, &tvb);
    bgfx::setIndexBuffer(&tib);
    bgfx::setTexture(0, c.s_shadowAtlas, c.atlasTex);
    bgfx::setState(BGFX_STATE_WRITE_RGB);
    bgfx::submit(kViewShadowDebug, c.debugProgram);
}

} // namespace Darkness
