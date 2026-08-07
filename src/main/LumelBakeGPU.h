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

// S2 lumel-bake engine (PLAN.HIGH_RES_SHADOWS "S2 promoted to the
// door-event engine"): run the bake's own formula on the GPU — zero
// bounces, visibility from the S1 face atlas — to modify the baked
// lightmaps in real time. This header holds the acceptance-first slice:
// GPU resources, the single-rect submit, and the startup SELF-TEST that
// diffs a GPU-baked overlay against the CPU bake of the same rect
// (`--lumel-bake-test`, `[LUMEL_BAKE]` lines). The event integration
// replaces DoorShadowSystem's CPU rays once this agrees.
//
// The lumel-to-world map is affine (LumelGrid::at), so a poly bakes as
// ONE quad with three uniform vectors — no bake mesh needed for the
// per-poly path. The fragment shader (fs_lumel_bake.sc) shares
// live_lights.sh's shadow lookup, so the S1 mirror is exercised by the
// same code path the world shader uses.

#pragma once

#include "ShadowMapCache.h"
#include "LightmapBake.h"

#include <bgfx/bgfx.h>
#include <glm/gtc/type_ptr.hpp>

#include <cstdio>
#include <vector>

namespace Darkness {

// One in-flight GPU bake batch: packed rects submitted to the scratch RT,
// blitted to staging, read back ~2 frames later. Single slot (v1): a new
// batch starts only after the previous collected — an event of ~700 rects
// still lands in ~150 ms end-to-end vs 3 s of CPU rays.
struct LumelBakeBatch {
    struct Item {
        size_t recIdx = 0;
        size_t ovK = 0;
        int16_t lightIdx = 0;
        int x = 0, y = 0, w = 0, h = 0;
    };
    std::vector<Item> items;
    std::vector<uint8_t> pixels;   // staging readback, scratch W*H*4
    uint32_t readyFrame = 0;
    bool inFlight = false;
};

struct LumelBakeGPU {
    bgfx::ProgramHandle program = BGFX_INVALID_HANDLE;   // owned
    bgfx::UniformHandle u_bakeOrigin = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_bakeSpanU  = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_bakeSpanV  = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_bakeNormal = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_bakeLight  = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_bakeColor  = BGFX_INVALID_HANDLE;
    // live_lights.sh's shared uniforms, set for THIS view too.
    bgfx::UniformHandle u_liveFalloff    = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_liveShadowInfo = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle s_liveShadowAtlas = BGFX_INVALID_HANDLE;

    // Scratch RT + staging for the batched event path.
    static constexpr int kScratchSize = 1024;
    bgfx::TextureHandle scratchTex = BGFX_INVALID_HANDLE;
    bgfx::FrameBufferHandle scratchFb = BGFX_INVALID_HANDLE;
    bgfx::TextureHandle stagingTex = BGFX_INVALID_HANDLE;
    LumelBakeBatch batch;

    bool valid() const {
        return bgfx::isValid(program) && bgfx::isValid(scratchFb);
    }
};

inline void initLumelBakeGPU(LumelBakeGPU &g, bgfx::ProgramHandle program) {
    g.program = program;
    g.u_bakeOrigin = bgfx::createUniform("u_bakeOrigin",
                                         bgfx::UniformType::Vec4);
    g.u_bakeSpanU = bgfx::createUniform("u_bakeSpanU",
                                        bgfx::UniformType::Vec4);
    g.u_bakeSpanV = bgfx::createUniform("u_bakeSpanV",
                                        bgfx::UniformType::Vec4);
    g.u_bakeNormal = bgfx::createUniform("u_bakeNormal",
                                         bgfx::UniformType::Vec4);
    g.u_bakeLight = bgfx::createUniform("u_bakeLight",
                                        bgfx::UniformType::Vec4);
    g.u_bakeColor = bgfx::createUniform("u_bakeColor",
                                        bgfx::UniformType::Vec4);
    g.u_liveFalloff = bgfx::createUniform("u_liveFalloff",
                                          bgfx::UniformType::Vec4);
    g.u_liveShadowInfo = bgfx::createUniform("u_liveShadowInfo",
                                             bgfx::UniformType::Vec4);
    g.s_liveShadowAtlas = bgfx::createUniform("s_liveShadowAtlas",
                                              bgfx::UniformType::Sampler);
    g.scratchTex = bgfx::createTexture2D(
        LumelBakeGPU::kScratchSize, LumelBakeGPU::kScratchSize, false, 1,
        bgfx::TextureFormat::RGBA8,
        BGFX_TEXTURE_RT | BGFX_SAMPLER_MIN_POINT | BGFX_SAMPLER_MAG_POINT);
    g.scratchFb = bgfx::createFrameBuffer(1, &g.scratchTex, false);
    g.stagingTex = bgfx::createTexture2D(
        LumelBakeGPU::kScratchSize, LumelBakeGPU::kScratchSize, false, 1,
        bgfx::TextureFormat::RGBA8,
        BGFX_TEXTURE_BLIT_DST | BGFX_TEXTURE_READ_BACK);
}

// Configure the bake view over the persistent scratch for this frame's
// submits. Call once per frame BEFORE any submitLumelRect.
inline void beginLumelBatchView(const LumelBakeGPU &g) {
    bgfx::setViewFrameBuffer(kViewLumelBake, g.scratchFb);
    bgfx::setViewRect(kViewLumelBake, 0, 0, LumelBakeGPU::kScratchSize,
                      LumelBakeGPU::kScratchSize);
    bgfx::setViewClear(kViewLumelBake, BGFX_CLEAR_COLOR, 0x000000ff);
    const Matrix4 ident(1.0f);
    const float sz = static_cast<float>(LumelBakeGPU::kScratchSize);
    const Matrix4 proj = bgfx::getCaps()->homogeneousDepth
        ? glm::orthoRH_NO(0.0f, sz, sz, 0.0f, -1.0f, 1.0f)
        : glm::orthoRH_ZO(0.0f, sz, sz, 0.0f, -1.0f, 1.0f);
    bgfx::setViewTransform(kViewLumelBake, glm::value_ptr(ident),
                           glm::value_ptr(proj));
}

// Blit + readback kick for this frame's batch (call after the submits).
// Returns the bgfx frame at which `batch.pixels` is valid.
inline uint32_t kickLumelBatchReadback(LumelBakeGPU &g) {
    bgfx::blit(kViewLumelRead, g.stagingTex, 0, 0, g.scratchTex, 0, 0,
               LumelBakeGPU::kScratchSize, LumelBakeGPU::kScratchSize);
    g.batch.pixels.assign(static_cast<size_t>(LumelBakeGPU::kScratchSize) *
                              LumelBakeGPU::kScratchSize * 4,
                          0);
    return bgfx::readTexture(g.stagingTex, g.batch.pixels.data());
}

inline void destroyLumelBakeGPU(LumelBakeGPU &g) {
    auto kill = [](auto &h) {
        if (bgfx::isValid(h)) { bgfx::destroy(h); h.idx = bgfx::kInvalidHandle; }
    };
    kill(g.program);
    kill(g.u_bakeOrigin);
    kill(g.u_bakeSpanU);
    kill(g.u_bakeSpanV);
    kill(g.u_bakeNormal);
    kill(g.u_bakeLight);
    kill(g.u_bakeColor);
    kill(g.u_liveFalloff);
    kill(g.u_liveShadowInfo);
    kill(g.s_liveShadowAtlas);
    kill(g.scratchFb);
    kill(g.scratchTex);
    kill(g.stagingTex);
}

// Submit one poly rect: a quad at [x0,y0]..[x0+w,y0+h] of the target (the
// view supplies an ortho over the RT in pixels), lumel UVs 0..1, uniforms
// carrying the affine lumel->world map and one light. `slot` is the
// light's S1 pool slot (its faces must be current — the caller ensures).
struct LumelQuadVertex {
    float x, y, z;
    float u, v;
    inline static bgfx::VertexLayout layout;
    static void init() {
        layout.begin()
            .add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
            .add(bgfx::Attrib::TexCoord0, 2, bgfx::AttribType::Float)
            .end();
    }
};

inline bool submitLumelRect(const LumelBakeGPU &g, const ShadowMapCache &sc,
                            const LumelGrid &grid, float surfaceOffset,
                            const Vector3 &lightPos, float reach,
                            const Vector3 &colorK, int slot,
                            float emitterA, int x0, int y0) {
    using QuadVertex = LumelQuadVertex;
    static bool layoutInit = false;
    if (!layoutInit) { QuadVertex::init(); layoutInit = true; }

    if (bgfx::getAvailTransientVertexBuffer(4, QuadVertex::layout) < 4)
        return false;
    bgfx::TransientVertexBuffer tvb;
    bgfx::TransientIndexBuffer tib;
    bgfx::allocTransientVertexBuffer(&tvb, 4, QuadVertex::layout);
    bgfx::allocTransientIndexBuffer(&tib, 6);
    auto *v = reinterpret_cast<QuadVertex *>(tvb.data);
    const float x1 = static_cast<float>(x0 + grid.lx);
    const float y1 = static_cast<float>(y0 + grid.ly);
    v[0] = {static_cast<float>(x0), static_cast<float>(y0), 0, 0.0f, 0.0f};
    v[1] = {x1, static_cast<float>(y0), 0, 1.0f, 0.0f};
    v[2] = {x1, y1, 0, 1.0f, 1.0f};
    v[3] = {static_cast<float>(x0), y1, 0, 0.0f, 1.0f};
    auto *ix = reinterpret_cast<uint16_t *>(tib.data);
    ix[0] = 0; ix[1] = 1; ix[2] = 2; ix[3] = 0; ix[4] = 2; ix[5] = 3;

    // Affine map with the half-lumel centre offset folded into the origin:
    // fragment centres sit at (i+0.5)/lx, so origin shifts back by half a
    // step (mirror of LumelGrid::at at ss=1).
    const Vector3 origin =
        grid.origin + grid.axisU * (grid.baseU - 0.5f * grid.stepU)
                    + grid.axisV * (grid.baseV - 0.5f * grid.stepV);
    const Vector3 spanU = grid.axisU * (grid.stepU * grid.lx);
    const Vector3 spanV = grid.axisV * (grid.stepV * grid.ly);

    const float o[4] = {origin.x, origin.y, origin.z, surfaceOffset};
    const float su[4] = {spanU.x, spanU.y, spanU.z, 0};
    const float sv[4] = {spanV.x, spanV.y, spanV.z, 0};
    const float nn[4] = {grid.normal.x, grid.normal.y, grid.normal.z, 0};
    const float li[4] = {lightPos.x, lightPos.y, lightPos.z, reach * reach};
    const float co[4] = {colorK.x, colorK.y, colorK.z,
                         static_cast<float>(slot)};
    const float fall[4] = {emitterA * emitterA, 0, 0, 0};
    const float info[4] = {
        static_cast<float>(sc.tilesPerRow),
        sc.atlasW > 0 ? float(sc.faceSize) / float(sc.atlasW) : 0.0f,
        sc.atlasH > 0 ? float(sc.faceSize) / float(sc.atlasH) : 0.0f,
        bgfx::getCaps()->originBottomLeft ? 1.0f : 0.0f};
    bgfx::setUniform(g.u_bakeOrigin, o);
    bgfx::setUniform(g.u_bakeSpanU, su);
    bgfx::setUniform(g.u_bakeSpanV, sv);
    bgfx::setUniform(g.u_bakeNormal, nn);
    bgfx::setUniform(g.u_bakeLight, li);
    bgfx::setUniform(g.u_bakeColor, co);
    bgfx::setUniform(g.u_liveFalloff, fall);
    bgfx::setUniform(g.u_liveShadowInfo, info);
    bgfx::setTexture(3, g.s_liveShadowAtlas, sc.atlasTex);
    bgfx::setVertexBuffer(0, &tvb);
    bgfx::setIndexBuffer(&tib);
    bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A);
    bgfx::submit(kViewLumelBake, g.program);
    return true;
}

// ── The acceptance self-test ────────────────────────────────────────────────
// GPU-bake ONE door-overlay rect and diff it against the CPU bake of the
// same rect at the same sampling (penumbra 1, supersample 1 — hard
// shadows both sides so the comparison isolates the FORMULA + the S1
// lookup, not penumbra style). Blocking; startup only.
inline void runLumelBakeSelfTest(LumelBakeGPU &g, ShadowMapCache &sc,
                                 const WRParsedData &wr,
                                 const RenderParams &rp,
                                 const BakeFormula &formula,
                                 const std::vector<int32_t> &lightCells,
                                 const std::vector<RebakedAnimPoly> &recs,
                                 const std::vector<uint8_t> &extraLights,
                                 int density) {
    if (!g.valid() || !sc.valid()) {
        std::fprintf(stderr, "[LUMEL_BAKE] resources invalid — skipped\n");
        return;
    }
    // Pick the first rec with a visible door overlay.
    size_t recIdx = recs.size();
    size_t ovK = 0;
    int16_t lightIdx = -1;
    for (size_t r = 0; r < recs.size() && recIdx == recs.size(); ++r) {
        const RebakedAnimPoly &rec = recs[r];
        for (size_t k = 0; k < rec.overlayLightIdx.size(); ++k) {
            const int16_t li = rec.overlayLightIdx[k];
            if (li > 0 && static_cast<size_t>(li) < extraLights.size() &&
                extraLights[li] && k < rec.overlays.size()) {
                uint8_t peak = 0;
                for (uint8_t b : rec.overlays[k]) peak = std::max(peak, b);
                if (peak >= 24) {
                    recIdx = r;
                    ovK = k;
                    lightIdx = li;
                    break;
                }
            }
        }
    }
    if (recIdx >= recs.size()) {
        std::fprintf(stderr,
            "[LUMEL_BAKE] no visible door overlay to test — skipped\n");
        return;
    }
    const RebakedAnimPoly &rec = recs[recIdx];

    // CPU reference at hard-shadow sampling.
    BakeFormula refF = formula;
    refF.penumbraSamples = 1;
    refF.supersample = 1;
    refF.includeAmbient = false;
    refF.includeSun = false;
    std::vector<uint8_t> mask(wr.staticLights.size(), 0);
    mask[static_cast<size_t>(lightIdx)] = 1;
    const std::vector<uint16_t> lightList = {
        1, static_cast<uint16_t>(lightIdx)};
    BakeStats stats;
    std::vector<Vector3> cpuLumels;
    if (!bakePolygon(wr, rp, rec.ci, rec.pi, refF, lightCells, cpuLumels,
                     stats, nullptr, mask.data(), nullptr, &lightList) ||
        static_cast<int>(cpuLumels.size()) != rec.w * rec.h) {
        std::fprintf(stderr, "[LUMEL_BAKE] CPU reference bake failed\n");
        return;
    }

    // GPU: current faces for the light (door-inclusive), then the rect.
    const int slot = ensureShadowLight(sc, wr, lightIdx, 424242u);
    if (slot < 0) {
        std::fprintf(stderr, "[LUMEL_BAKE] no shadow slot — skipped\n");
        return;
    }
    const LumelGrid grid = buildLumelGrid(wr, rec.ci, rec.pi, density);
    if (!grid.valid || grid.lx != rec.w || grid.ly != rec.h) {
        std::fprintf(stderr,
            "[LUMEL_BAKE] grid mismatch (%dx%d vs rec %dx%d) — skipped\n",
            grid.lx, grid.ly, rec.w, rec.h);
        return;
    }

    const uint16_t rtW = static_cast<uint16_t>(grid.lx);
    const uint16_t rtH = static_cast<uint16_t>(grid.ly);
    bgfx::TextureHandle rt = bgfx::createTexture2D(
        rtW, rtH, false, 1, bgfx::TextureFormat::RGBA8,
        BGFX_TEXTURE_RT | BGFX_SAMPLER_MIN_POINT | BGFX_SAMPLER_MAG_POINT);
    bgfx::FrameBufferHandle fb = bgfx::createFrameBuffer(1, &rt, false);
    bgfx::TextureHandle staging = bgfx::createTexture2D(
        rtW, rtH, false, 1, bgfx::TextureFormat::RGBA8,
        BGFX_TEXTURE_BLIT_DST | BGFX_TEXTURE_READ_BACK);

    bgfx::setViewFrameBuffer(kViewLumelBake, fb);
    bgfx::setViewRect(kViewLumelBake, 0, 0, rtW, rtH);
    bgfx::setViewClear(kViewLumelBake, BGFX_CLEAR_COLOR, 0x000000ff);
    const Matrix4 ident(1.0f);
    const Matrix4 proj = bgfx::getCaps()->homogeneousDepth
        ? glm::orthoRH_NO(0.0f, float(rtW), float(rtH), 0.0f, -1.0f, 1.0f)
        : glm::orthoRH_ZO(0.0f, float(rtW), float(rtH), 0.0f, -1.0f, 1.0f);
    bgfx::setViewTransform(kViewLumelBake, glm::value_ptr(ident),
                           glm::value_ptr(proj));

    // Per-light colour: bright x brightScale x K_i (the formula's anchor
    // table is the throw-intensity authority).
    const WRStaticLight &L = wr.staticLights[lightIdx];
    const float anchor = formula.anchorFor(lightIdx);
    const float a2 = formula.emitterRadius * formula.emitterRadius;
    const float K = (anchor * anchor + a2) / anchor;
    const Vector3 colorK = L.bright * (formula.brightScale * K);
    const float reach = sc.lightReach[lightIdx];

    if (!submitLumelRect(g, sc, grid, formula.surfaceOffset, L.loc, reach,
                         colorK, slot, formula.emitterRadius, 0, 0)) {
        std::fprintf(stderr, "[LUMEL_BAKE] submit failed\n");
        bgfx::destroy(fb); bgfx::destroy(rt); bgfx::destroy(staging);
        return;
    }
    bgfx::blit(kViewLumelRead, staging, 0, 0, rt, 0, 0, rtW, rtH);
    std::vector<uint8_t> pixels(static_cast<size_t>(rtW) * rtH * 4);
    const uint32_t ready = bgfx::readTexture(staging, pixels.data());
    uint32_t cur = bgfx::frame();
    while (cur < ready) cur = bgfx::frame();

    // Compare (GPU RGBA8 vs CPU floats clamped to the same 8-bit scale).
    // The top-left GPU row is lumel row 0 on Metal/D3D; GL readback is
    // bottom-up — same flip the shadow-atlas readback applies.
    const bool flip = bgfx::getCaps()->originBottomLeft;
    long total = 0, within2 = 0;
    double sumAbs = 0.0;
    int maxD = 0;
    for (int y = 0; y < rec.h; ++y) {
        const int ry = flip ? (rec.h - 1 - y) : y;
        for (int x = 0; x < rec.w; ++x) {
            const size_t gi = (static_cast<size_t>(ry) * rtW + x) * 4;
            const Vector3 &cf = cpuLumels[static_cast<size_t>(y) * rec.w + x];
            for (int ch = 0; ch < 3; ++ch) {
                const float cv = std::min(1.0f, std::max(0.0f, cf[ch]));
                const int cpu8 = static_cast<int>(cv * 255.0f + 0.5f);
                const int gpu8 = pixels[gi + ch];
                const int d = std::abs(cpu8 - gpu8);
                sumAbs += d;
                maxD = std::max(maxD, d);
                if (d <= 2) ++within2;
                ++total;
            }
        }
    }
    std::fprintf(stderr,
        "[LUMEL_BAKE] cell %u poly %d light %d (%dx%d, slot %d): "
        "mean |d| %.2f/255, max %d, %.2f%% within 2/255 %s\n",
        rec.ci, rec.pi, lightIdx, rec.w, rec.h, slot,
        total ? sumAbs / total : 0.0, maxD,
        total ? 100.0 * within2 / total : 0.0,
        (total && 100.0 * within2 / total >= 95.0) ? "— PASS" : "— FAIL");

    bgfx::destroy(fb);
    bgfx::destroy(rt);
    bgfx::destroy(staging);
}

} // namespace Darkness
