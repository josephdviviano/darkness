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
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <queue>

#include <SDL.h>
#include <SDL_syswm.h>

#include <bgfx/bgfx.h>
#include <bgfx/platform.h>
#include <bx/math.h>

#include "../shaders/embedded_shaders.h"
#include "WRChunkParser.h"
#include "TXListParser.h"
#include "CRFTextureLoader.h"
#include "CRFModelLoader.h"
#include "LightmapAtlas.h"
#include "SpawnFinder.h"
#include "LightingSystem.h"
#include "ObjectPropParser.h"
#include "BinMeshParser.h"
#include "RenderConfig.h"
#include "RayCaster.h"
#include "DebugConsole.h"

// Logging (must be initialized before ServiceManager)
#include "logger.h"
#include "stdlog.h"
#include "ConsoleBackend.h"

// Service stack for typed property access
#include "DarknessServiceManager.h"
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
#include "sim/SimService.h"
#include "physics/PhysicsService.h"
#include "RawDataStorage.h"
#include "PLDefParser.h"
#include "DTypeSizeParser.h"
#include "SingleFieldDataStorage.h"
#include "worldquery/ObjSysWorldState.h"

// TODO: Make these configurable via command-line or config file
static const int WINDOW_WIDTH  = 1280;
static const int WINDOW_HEIGHT = 720;

// ── Renderer modules ──
// Core: vertex formats, texture pipeline, camera, collision, culling, fog data
// Extended: mesh builders, water system, sky, fog parsing, fallback cube
// State: data-holding structs for renderer decomposition
#include "DarknessRendererCore.h"
#include "DarknessRendererExtended.h"
#include "DarknessRenderState.h"

using namespace Darkness;

#include "DarknessRenderInit.h"

static void printHelp() {
    std::fprintf(stderr,
        "darknessRender — Dark Engine world geometry viewer\n"
        "\n"
        "Usage:\n"
        "  darknessRender <mission.mis> [--res <path>] [--config <path>] [--lightmap-filtering <mode>]\n"
        "\n"
        "Options:\n"
        "  --res <path>   Path to Thief 2 RES directory containing fam.crf.\n"
        "                 Enables lightmapped+textured rendering. Without this\n"
        "                 flag, geometry is rendered with flat Lambertian shading.\n"
        "  --lightmap-filtering <mode>\n"
        "                 Lightmap filtering mode: bilinear (default) or bicubic.\n"
        "                 bilinear = GPU hardware filtering (fast, matches original engine).\n"
        "                 bicubic  = cubic B-spline shader (smoother, minimal GPU cost).\n"
        "  --no-objects   Disable object mesh rendering (world geometry only).\n"
        "  --no-cull      Start with portal culling disabled (see all geometry).\n"
        "  --collision    Start with camera collision enabled (clip to world).\n"
        "  --filter       Start with bilinear texture filtering (default: point/crispy).\n"
        "  --force-flicker Force all animated lights to flicker mode (debug).\n"
        "  --linear-mips  Gamma-correct mipmap generation (sRGB linearization).\n"
        "  --sharp-mips   Sharpen mip levels to preserve detail at distance.\n"
        "  --config <path> Path to YAML config file (default: ./darknessRender.yaml).\n"
        "  --help         Show this help message.\n"
        "\n"
        "Controls:\n"
        "  WASD           Move forward/left/back/right\n"
        "  Mouse          Look around\n"
        "  Space/LShift   Move up/down\n"
        "  Q/E            Move up/down (alternate)\n"
        "  Ctrl           Sprint (3x speed)\n"
        "  Scroll wheel   Adjust movement speed (shown in title bar)\n"
        "  ` (backtick)   Open settings console\n"
        "  Home           Teleport to player spawn point\n"
        "  Esc            Quit\n"
        "\n"
        "Debug shortcuts (hold Backspace + key):\n"
        "  BS+C           Toggle portal culling on/off\n"
        "  BS+F           Cycle texture filtering (point/bilinear/trilinear/aniso)\n"
        "  BS+L           Toggle lightmap filtering (bilinear/bicubic)\n"
        "  BS+V           Toggle camera collision (clip/noclip)\n"
        "  BS+M/N         Cycle model isolation (next/prev)\n"
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
        "  darknessRender path/to/miss6.mis\n"
        "\n"
        "  # Textured, using mounted Thief 2 disc:\n"
        "  darknessRender path/to/miss6.mis --res /Volumes/THIEF2_INSTALL_C/THIEF2/RES\n"
        "\n"
        "  # Textured, using GOG install:\n"
        "  darknessRender path/to/miss6.mis --res ~/GOG/Thief2/RES\n"
    );
}

// ── Render-loop functions ──
// Each function handles one pass of the per-frame render loop.
// Parameters are explicit locals from main() — will be consolidated into
// structs in Steps 11-13.

// Render sky dome or textured skybox into View 0 (no depth write/test).
static void renderSky(
    const Darkness::FrameContext &fc,
    const Darkness::BuiltMeshes &meshes,
    const Darkness::GPUResources &gpu,
    const Darkness::MissionData &mission,
    const Darkness::RuntimeState &state)
{
    float skyView[16];
    state.cam.getSkyViewMatrix(skyView);
    bgfx::setViewTransform(0, skyView, fc.proj);

    float skyModel[16];
    bx::mtxIdentity(skyModel);

    // Cull CCW (not CW) because the camera is inside the cube/sphere —
    // we see the back faces, so cull the outward-facing front faces.
    uint64_t skyState = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
                      | BGFX_STATE_CULL_CCW;

    // Inline sky fog uniform helper
    float opaqueParams[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
    auto setFogSky = [&]() {
        bgfx::setUniform(gpu.u_fogColor, fc.fogColorArr);
        bgfx::setUniform(gpu.u_fogParams, fc.skyFogArr);
        bgfx::setUniform(gpu.u_objectParams, opaqueParams);
    };

    if (mission.hasSkybox && bgfx::isValid(gpu.skyboxVBH)) {
        // Textured skybox (old sky system) — render each face with its texture
        for (const auto &face : meshes.skyboxCube.faces) {
            auto texIt = gpu.skyboxTexHandles.find(face.key);
            if (texIt == gpu.skyboxTexHandles.end()) continue;

            setFogSky();
            bgfx::setTransform(skyModel);
            bgfx::setVertexBuffer(0, gpu.skyboxVBH);
            bgfx::setIndexBuffer(gpu.skyboxIBH, face.firstIndex, face.indexCount);
            bgfx::setState(skyState);
            bgfx::setTexture(0, gpu.s_texColor, texIt->second, fc.skySampler);
            bgfx::submit(0, gpu.texturedProgram);
        }
    } else if (bgfx::isValid(gpu.skyVBH)) {
        // Procedural dome (new sky system) — vertex-coloured hemisphere
        setFogSky();
        bgfx::setTransform(skyModel);
        bgfx::setVertexBuffer(0, gpu.skyVBH);
        bgfx::setIndexBuffer(gpu.skyIBH);
        bgfx::setState(skyState);
        bgfx::submit(0, gpu.flatProgram);
    }
}

// Render world geometry into View 1 — lightmapped, textured, or flat-shaded.
// Iterates per-cell draw groups, skipping cells not in the visible set.
static void renderWorld(
    const Darkness::FrameContext &fc,
    const Darkness::BuiltMeshes &meshes,
    const Darkness::GPUResources &gpu,
    const Darkness::MissionData &mission,
    const Darkness::RuntimeState &state)
{
    float model[16];
    bx::mtxIdentity(model);

    float opaqueParams[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
    auto setFogOn = [&]() {
        bgfx::setUniform(gpu.u_fogColor, fc.fogColorArr);
        bgfx::setUniform(gpu.u_fogParams, fc.fogOnArr);
        bgfx::setUniform(gpu.u_objectParams, opaqueParams);
    };

    auto isCellVisible = [&](uint32_t cellID) -> bool {
        if (!state.portalCulling) return true;
        return fc.visibleCells.count(cellID) > 0;
    };

    if (meshes.lightmappedMode) {
        for (const auto &grp : meshes.lmMesh.groups) {
            if (!isCellVisible(grp.cellID)) continue;

            setFogOn();
            bgfx::setTransform(model);
            bgfx::setVertexBuffer(0, gpu.vbh);
            bgfx::setIndexBuffer(gpu.ibh, grp.firstIndex, grp.numIndices);
            bgfx::setState(fc.renderState);

            if (grp.txtIndex == 0) {
                bgfx::submit(1, gpu.flatProgram);
            } else {
                auto it = gpu.textureHandles.find(grp.txtIndex);
                if (it != gpu.textureHandles.end()) {
                    bgfx::setTexture(0, gpu.s_texColor, it->second, fc.texSampler);
                    if (!gpu.lightmapAtlasHandles.empty())
                        bgfx::setTexture(1, gpu.s_texLightmap, gpu.lightmapAtlasHandles[0]);

                    // Select lightmap program: bilinear (hardware) or bicubic (shader)
                    bgfx::ProgramHandle lmProg = gpu.lightmappedProgram;
                    if (state.lightmapFiltering == 1 && bgfx::isValid(gpu.lightmappedBicubicProgram)) {
                        lmProg = gpu.lightmappedBicubicProgram;
                        // Pass atlas dimensions for texel-space calculations in bicubic shader
                        float sz[4] = { float(gpu.lmAtlasSet.atlases[0].size),
                                        float(gpu.lmAtlasSet.atlases[0].size), 0, 0 };
                        bgfx::setUniform(gpu.u_lmAtlasSize, sz);
                    }
                    bgfx::submit(1, lmProg);
                } else {
                    bgfx::submit(1, gpu.flatProgram);
                }
            }
        }
    } else if (mission.texturedMode) {
        for (const auto &grp : meshes.worldMesh.groups) {
            if (!isCellVisible(grp.cellID)) continue;

            setFogOn();
            bgfx::setTransform(model);
            bgfx::setVertexBuffer(0, gpu.vbh);
            bgfx::setIndexBuffer(gpu.ibh, grp.firstIndex, grp.numIndices);
            bgfx::setState(fc.renderState);

            if (grp.txtIndex == 0) {
                bgfx::submit(1, gpu.flatProgram);
            } else {
                auto it = gpu.textureHandles.find(grp.txtIndex);
                if (it != gpu.textureHandles.end()) {
                    bgfx::setTexture(0, gpu.s_texColor, it->second, fc.texSampler);
                    bgfx::submit(1, gpu.texturedProgram);
                } else {
                    bgfx::submit(1, gpu.flatProgram);
                }
            }
        }
    } else {
        for (const auto &grp : meshes.flatMesh.groups) {
            if (!isCellVisible(grp.cellID)) continue;

            setFogOn();
            bgfx::setTransform(model);
            bgfx::setVertexBuffer(0, gpu.vbh);
            bgfx::setIndexBuffer(gpu.ibh, grp.firstIndex, grp.numIndices);
            bgfx::setState(fc.renderState);
            bgfx::submit(1, gpu.flatProgram);
        }
    }
}

// Render water surfaces into View 1 — alpha-blended, no depth write, double-sided.
// Rendered last so all opaque geometry is already in the depth buffer.
static void renderWater(
    const Darkness::FrameContext &fc,
    const Darkness::BuiltMeshes &meshes,
    const Darkness::GPUResources &gpu,
    const Darkness::RuntimeState &state)
{
    if (!meshes.hasWater) return;

    float identity[16];
    bx::mtxIdentity(identity);

    // Alpha blend, depth test (read) but no depth write, no face culling
    uint64_t waterState = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
        | BGFX_STATE_DEPTH_TEST_LESS
        | BGFX_STATE_BLEND_ALPHA;
    // No BGFX_STATE_WRITE_Z — water doesn't occlude geometry behind it
    // No BGFX_STATE_CULL_* — water is visible from both sides

    float opaqueParams[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
    auto setFogOn = [&]() {
        bgfx::setUniform(gpu.u_fogColor, fc.fogColorArr);
        bgfx::setUniform(gpu.u_fogParams, fc.fogOnArr);
        bgfx::setUniform(gpu.u_objectParams, opaqueParams);
    };

    for (const auto &grp : meshes.waterMesh.groups) {
        setFogOn();
        bgfx::setTransform(identity);
        bgfx::setVertexBuffer(0, gpu.waterVBH);
        bgfx::setIndexBuffer(gpu.waterIBH, grp.firstIndex, grp.numIndices);
        bgfx::setState(waterState);

        if (grp.txtIndex != 0 || grp.flowGroup > 0) {
            // Textured water: water shader with vertex displacement + UV distortion
            // u_waterParams: x=elapsed time, y=scroll speed, z=wave amplitude, w=UV distortion
            float wp[4] = { state.waterElapsed, state.waterScrollSpeed, state.waveAmplitude, state.uvDistortion };
            bgfx::setUniform(gpu.u_waterParams, wp);

            // u_waterFlow: x=rotation speed, y=use_world_uv flag, z=tex_unit_len
            // Flow-textured water: vertex shader computes UVs from world position
            // with rotation (matching Dark Engine's flow animation system).
            // TXLIST-textured water: uses pre-computed UVs from mesh.
            bool isFlowTextured = (grp.flowGroup > 0);
            float useWorldUV = isFlowTextured ? 1.0f : 0.0f;
            constexpr float TEX_UNIT_LEN = 4.0f; // 4 world units per texture repeat
            float wf[4] = { state.waterRotation, useWorldUV, TEX_UNIT_LEN, 0.0f };
            bgfx::setUniform(gpu.u_waterFlow, wf);

            // Resolve texture: flow texture by group first, then TXLIST by index
            bgfx::TextureHandle tex = BGFX_INVALID_HANDLE;
            if (grp.flowGroup > 0) {
                auto fit = gpu.flowTextureHandles.find(grp.flowGroup);
                if (fit != gpu.flowTextureHandles.end())
                    tex = fit->second;
            }
            if (!bgfx::isValid(tex) && grp.txtIndex != 0) {
                auto it = gpu.textureHandles.find(grp.txtIndex);
                if (it != gpu.textureHandles.end())
                    tex = it->second;
            }

            if (bgfx::isValid(tex)) {
                bgfx::setTexture(0, gpu.s_texColor, tex, fc.texSampler);
                bgfx::submit(1, gpu.waterProgram);
            } else {
                bgfx::submit(1, gpu.flatProgram);
            }
        } else {
            // Non-textured water: flat blue-green from vertex color
            bgfx::submit(1, gpu.flatProgram);
        }
    }
}

// Render debug raycast visualization + HUD text into View 2.
// Casts a ray from camera forward, draws cross at hit point, normal line,
// and HUD text overlay with hit details.
static void renderDebugOverlay(
    const Darkness::FrameContext &fc,
    const Darkness::GPUResources &gpu,
    const Darkness::MissionData &mission,
    const Darkness::RuntimeState &state)
{
    if (!state.showRaycast) return;

    // Set up view 2 with same transform as view 1
    bgfx::setViewTransform(2, fc.view, fc.proj);

    // Compute camera forward direction (same as Camera::getViewMatrix)
    float cosPitch = std::cos(state.cam.pitch);
    float fwdX = std::cos(state.cam.yaw) * cosPitch;
    float fwdY = std::sin(state.cam.yaw) * cosPitch;
    float fwdZ = std::sin(state.cam.pitch);

    // Ray from camera position, extending forward up to 500 world units
    constexpr float RAY_VIS_DIST = 500.0f;
    Darkness::Vector3 rayFrom(state.cam.pos[0], state.cam.pos[1], state.cam.pos[2]);
    Darkness::Vector3 rayTo(state.cam.pos[0] + fwdX * RAY_VIS_DIST,
                            state.cam.pos[1] + fwdY * RAY_VIS_DIST,
                            state.cam.pos[2] + fwdZ * RAY_VIS_DIST);

    Darkness::RayHit rayHit;
    bool rayDidHit = Darkness::raycastWorld(mission.wrData, rayFrom, rayTo, rayHit);

    // Draw debug lines using bgfx transient vertex buffers.
    // Hit: normal (2) + cross (6) + offset ray (2) = 10.  Miss: ray (2).
    constexpr uint32_t MAX_DEBUG_VERTS = 10;
    bgfx::TransientVertexBuffer tvb;
    if (bgfx::getAvailTransientVertexBuffer(MAX_DEBUG_VERTS, Darkness::PosColorVertex::layout) >= MAX_DEBUG_VERTS) {
        bgfx::allocTransientVertexBuffer(&tvb, MAX_DEBUG_VERTS, Darkness::PosColorVertex::layout);
        auto *verts = reinterpret_cast<Darkness::PosColorVertex *>(tvb.data);
        uint32_t numVerts = 0;

        // Helper: emit one debug vertex
        auto addVert = [&](float vx, float vy, float vz, uint32_t color) {
            verts[numVerts].x = vx;
            verts[numVerts].y = vy;
            verts[numVerts].z = vz;
            verts[numVerts].abgr = color;
            ++numVerts;
        };

        // Colors (ABGR format for bgfx)
        uint32_t green  = 0xFF00FF00;  // hit ray segment
        uint32_t yellow = 0xFF00FFFF;  // miss ray segment
        uint32_t cyan   = 0xFFFFFF00;  // surface normal
        uint32_t red    = 0xFF0000FF;  // hit marker cross

        if (rayDidHit) {
            // Cyan line: surface normal at hit point (4 unit length)
            constexpr float NORM_LEN = 4.0f;
            addVert(rayHit.point.x, rayHit.point.y, rayHit.point.z, cyan);
            addVert(rayHit.point.x + rayHit.normal.x * NORM_LEN,
                    rayHit.point.y + rayHit.normal.y * NORM_LEN,
                    rayHit.point.z + rayHit.normal.z * NORM_LEN, cyan);

            // Red cross at hit point (3 axes, 2 unit half-size — visible at distance)
            constexpr float CROSS = 2.0f;
            addVert(rayHit.point.x - CROSS, rayHit.point.y, rayHit.point.z, red);
            addVert(rayHit.point.x + CROSS, rayHit.point.y, rayHit.point.z, red);
            addVert(rayHit.point.x, rayHit.point.y - CROSS, rayHit.point.z, red);
            addVert(rayHit.point.x, rayHit.point.y + CROSS, rayHit.point.z, red);
            addVert(rayHit.point.x, rayHit.point.y, rayHit.point.z - CROSS, red);
            addVert(rayHit.point.x, rayHit.point.y, rayHit.point.z + CROSS, red);

            // Green line: from hit point pulled back slightly toward camera,
            // offset perpendicular so it's not collinear with the view ray.
            // This gives a visible green "approach" line near the hit.
            constexpr float PULLBACK = 5.0f;
            constexpr float OFFSET = 0.3f;
            // Camera right vector for perpendicular offset
            float rtX = std::sin(state.cam.yaw);
            float rtY = -std::cos(state.cam.yaw);
            addVert(rayHit.point.x - fwdX * PULLBACK + rtX * OFFSET,
                    rayHit.point.y - fwdY * PULLBACK + rtY * OFFSET,
                    rayHit.point.z - fwdZ * PULLBACK, green);
            addVert(rayHit.point.x, rayHit.point.y, rayHit.point.z, green);
        } else {
            // Yellow line: camera → ray end (visible against sky/void)
            addVert(state.cam.pos[0], state.cam.pos[1], state.cam.pos[2], yellow);
            addVert(rayTo.x, rayTo.y, rayTo.z, yellow);
        }

        // Submit to view 2 (debug overlay) — no depth test so lines
        // render on top of world geometry
        float identity[16];
        bx::mtxIdentity(identity);
        bgfx::setTransform(identity);
        bgfx::setVertexBuffer(0, &tvb, 0, numVerts);
        uint64_t lineState = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
                           | BGFX_STATE_PT_LINES;
        bgfx::setState(lineState);
        float opaqueParams[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
        bgfx::setUniform(gpu.u_fogColor, fc.fogColorArr);
        bgfx::setUniform(gpu.u_fogParams, fc.fogOnArr);
        bgfx::setUniform(gpu.u_objectParams, opaqueParams);
        bgfx::submit(2, gpu.flatProgram);
    }

    // HUD text overlay showing raycast results
    bgfx::setDebug(BGFX_DEBUG_TEXT);
    bgfx::dbgTextClear();

    // Screen-center crosshair (160 cols x 45 rows at 1280x720)
    uint8_t cross_attr = 0x0E; // yellow on black
    bgfx::dbgTextPrintf(79, 22, cross_attr, "+");

    uint8_t hud_attr = 0x0F; // white on black
    uint8_t val_attr = 0x0A; // green on black

    bgfx::dbgTextPrintf(2, 1, hud_attr, "RAYCAST DEBUG (Backspace+R to toggle)");

    if (rayDidHit) {
        bgfx::dbgTextPrintf(2, 3, val_attr, "Hit:     YES");
        bgfx::dbgTextPrintf(2, 4, val_attr, "Dist:    %.2f", rayHit.distance);
        bgfx::dbgTextPrintf(2, 5, val_attr, "Point:   (%.2f, %.2f, %.2f)",
            rayHit.point.x, rayHit.point.y, rayHit.point.z);
        bgfx::dbgTextPrintf(2, 6, val_attr, "Normal:  (%.3f, %.3f, %.3f)",
            rayHit.normal.x, rayHit.normal.y, rayHit.normal.z);
        bgfx::dbgTextPrintf(2, 7, val_attr, "TexIdx:  %d",
            rayHit.textureIndex);
        // Show texture name if available from TXLIST
        if (rayHit.textureIndex >= 0 &&
            static_cast<size_t>(rayHit.textureIndex) < mission.txList.textures.size()) {
            const auto &tex = mission.txList.textures[rayHit.textureIndex];
            bgfx::dbgTextPrintf(2, 8, val_attr, "Texture: %s/%s",
                tex.family.c_str(), tex.name.c_str());
        }
    } else {
        bgfx::dbgTextPrintf(2, 3, 0x0C, "Hit:     NO (miss)");
    }

    // Camera info line
    int32_t camCellIdx = findCameraCell(mission.wrData, state.cam.pos[0], state.cam.pos[1], state.cam.pos[2]);
    bgfx::dbgTextPrintf(2, 10, hud_attr, "Camera:  (%.2f, %.2f, %.2f)  cell=%d",
        state.cam.pos[0], state.cam.pos[1], state.cam.pos[2], camCellIdx);
}

// ── Object rendering ──
// Two-pass rendering for correct transparency compositing:
//   Pass 1: all opaque submeshes (matTrans == 0 AND renderAlpha == 1.0)
//   Pass 2: all translucent submeshes (matTrans > 0 OR renderAlpha < 1.0)
// A single object (e.g. a window) can have both opaque submeshes (frame)
// and translucent submeshes (glass pane), so transparency is per-submesh.
//
// Final alpha = (1.0 - material.trans) * object.renderAlpha
// where material.trans is from .bin mat_extra (0=opaque, 0.3=glass)
// and object.renderAlpha is from P$RenderAlp property (1.0=opaque)
static void renderObjects(
    const Darkness::FrameContext &fc,
    const Darkness::GPUResources &gpu,
    const Darkness::MissionData &mission,
    const Darkness::RuntimeState &state)
{
    if (!state.showObjects) return;

    // Alpha-blend state for translucent submeshes
    uint64_t translucentState = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
                              | BGFX_STATE_WRITE_Z | BGFX_STATE_DEPTH_TEST_LESS
                              | BGFX_STATE_CULL_CW
                              | BGFX_STATE_BLEND_ALPHA;

    // Helper: set fog uniforms (bgfx clears uniforms after each submit)
    float opaqueParams[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
    auto setFogOn = [&]() {
        bgfx::setUniform(gpu.u_fogColor, fc.fogColorArr);
        bgfx::setUniform(gpu.u_fogParams, fc.fogOnArr);
        bgfx::setUniform(gpu.u_objectParams, opaqueParams);
    };

    // Helper: draw one object's submeshes, filtering by opacity.
    // When opaquePass=true, draws only opaque submeshes with fc.renderState.
    // When opaquePass=false, draws only translucent submeshes with translucentState.
    auto drawObjectSubmeshes = [&](size_t oi, bool opaquePass) {
        const auto &obj = mission.objData.objects[oi];
        if (!obj.hasPosition) return;

        // Portal culling: skip objects in non-visible cells.
        if (state.portalCulling && oi < mission.objCellIDs.size()) {
            int32_t objCell = mission.objCellIDs[oi];
            if (objCell >= 0 && fc.visibleCells.count(static_cast<uint32_t>(objCell)) == 0)
                return;
        }

        // Compute per-object model matrix from position + angles
        float objMtx[16];
        buildModelMatrix(objMtx, obj.x, obj.y, obj.z,
                         obj.heading, obj.pitch, obj.bank,
                         obj.scaleX, obj.scaleY, obj.scaleZ);

        std::string modelName(obj.modelName);

        // Model isolation mode: skip objects that don't match the isolated model
        if (state.isolateModelIdx >= 0 && state.isolateModelIdx < (int)state.sortedModelNames.size()) {
            if (modelName != state.sortedModelNames[state.isolateModelIdx])
                return;
        }

        auto it = gpu.objModelGPU.find(modelName);

        if (it != gpu.objModelGPU.end() && it->second.valid) {
            const auto &gpuModel = it->second;

            for (const auto &sm : gpuModel.subMeshes) {
                if (sm.indexCount == 0) continue;

                // Determine if this submesh is translucent:
                // either the material has translucency or the object has RenderAlpha
                bool isTranslucent = (sm.matTrans > 0.0f) || (obj.renderAlpha < 1.0f);
                if (opaquePass == isTranslucent) continue;  // wrong pass

                // Compute final alpha: (1 - matTrans) * renderAlpha
                // matTrans convention: 0=opaque, 0.3=30% transparent glass
                float finalAlpha = (1.0f - sm.matTrans) * obj.renderAlpha;
                float objAlpha[4] = { finalAlpha, 0.0f, 0.0f, 0.0f };

                uint64_t drawState = opaquePass ? fc.renderState : translucentState;

                bgfx::setUniform(gpu.u_fogColor, fc.fogColorArr);
                bgfx::setUniform(gpu.u_fogParams, fc.fogOnArr);
                bgfx::setUniform(gpu.u_objectParams, objAlpha);
                bgfx::setTransform(objMtx);
                bgfx::setVertexBuffer(0, gpuModel.vbh);
                bgfx::setIndexBuffer(gpuModel.ibh, sm.firstIndex, sm.indexCount);
                bgfx::setState(drawState);

                if (sm.textured) {
                    auto texIt = gpu.objTextureHandles.find(sm.matName);
                    if (texIt != gpu.objTextureHandles.end()) {
                        bgfx::setTexture(0, gpu.s_texColor, texIt->second, fc.texSampler);
                        bgfx::submit(1, gpu.texturedProgram);
                    } else {
                        bgfx::submit(1, gpu.flatProgram);
                    }
                } else {
                    bgfx::submit(1, gpu.flatProgram);
                }
            }
        } else if (state.showFallbackCubes && opaquePass) {
            // Fallback cubes are always opaque
            setFogOn();
            bgfx::setTransform(objMtx);
            bgfx::setVertexBuffer(0, gpu.fallbackCubeVBH);
            bgfx::setIndexBuffer(gpu.fallbackCubeIBH);
            bgfx::setState(fc.renderState);
            bgfx::submit(1, gpu.flatProgram);
        }
    };

    // Pass 1: opaque submeshes of all objects
    for (size_t oi = 0; oi < mission.objData.objects.size(); ++oi) {
        drawObjectSubmeshes(oi, true);
    }
    // Pass 2: translucent submeshes — rendered after all opaque geometry
    for (size_t oi = 0; oi < mission.objData.objects.size(); ++oi) {
        drawObjectSubmeshes(oi, false);
    }
}


// ── Update window title bar ──
// Displays current render mode, speed, culling stats, filter mode, and model isolation info.
static void updateTitleBar(SDL_Window *window, const Darkness::RuntimeState &state) {
    char title[512];
    const char *filterNames[] = { "point", "bilinear", "trilinear", "aniso" };
    const char *filterStr = filterNames[state.filterMode % 4];
    const char *lmNames[] = { "bilinear", "bicubic" };
    const char *lmStr = lmNames[state.lightmapFiltering % 2];

    // Build model isolation suffix if active
    char isoSuffix[128] = "";
    if (state.isolateModelIdx >= 0 && state.isolateModelIdx < (int)state.sortedModelNames.size()) {
        const auto &isoName = state.sortedModelNames[state.isolateModelIdx];
        auto cit = state.modelInstanceCounts.find(isoName);
        int cnt = (cit != state.modelInstanceCounts.end()) ? cit->second : 0;
        std::snprintf(isoSuffix, sizeof(isoSuffix),
            " [MODEL %d/%zu: %s x%d]",
            state.isolateModelIdx + 1, state.sortedModelNames.size(),
            isoName.c_str(), cnt);
    }

    const char *clipStr = state.cameraCollision ? "clip" : "noclip";

    if (state.portalCulling) {
        std::snprintf(title, sizeof(title),
            "darkness — %s [speed: %.1f] [cull: %u/%u cells] [%s] [lm:%s] [%s]%s",
            state.modeStr, state.moveSpeed, state.cullVisibleCells, state.cullTotalCells, filterStr, lmStr, clipStr, isoSuffix);
    } else {
        std::snprintf(title, sizeof(title),
            "darkness — %s [speed: %.1f] [cull: OFF] [%s] [lm:%s] [%s]%s",
            state.modeStr, state.moveSpeed, filterStr, lmStr, clipStr, isoSuffix);
    }
    SDL_SetWindowTitle(window, title);
}

// ── Register debug console settings ──
// Binds 11 runtime-changeable settings to the debug console (opened with backtick).
// Lambdas capture state by reference and update the title bar when relevant settings change.
static void registerConsoleSettings(
    Darkness::DebugConsole &dbgConsole,
    Darkness::RuntimeState &state,
    SDL_Window *window)
{
    // Helper lambda that captures window+state for title bar refresh
    auto refreshTitle = [window, &state]() { updateTitleBar(window, state); };

    dbgConsole.addCategorical("filter_mode",
        {"point", "bilinear", "trilinear", "anisotropic"},
        [&state]() { return state.filterMode; },
        [&state, refreshTitle](int v) { state.filterMode = v; refreshTitle(); });

    dbgConsole.addCategorical("lightmap_filtering",
        {"bilinear", "bicubic"},
        [&state]() { return state.lightmapFiltering; },
        [&state, refreshTitle](int v) { state.lightmapFiltering = v; refreshTitle(); });

    dbgConsole.addBool("portal_culling",
        [&state]() { return state.portalCulling; },
        [&state, refreshTitle](bool v) { state.portalCulling = v; refreshTitle(); });

    dbgConsole.addBool("camera_collision",
        [&state]() { return state.cameraCollision; },
        [&state, refreshTitle](bool v) { state.cameraCollision = v; refreshTitle(); });

    dbgConsole.addBool("show_objects",
        [&state]() { return state.showObjects; },
        [&state](bool v) { state.showObjects = v; });

    dbgConsole.addBool("show_fallback_cubes",
        [&state]() { return state.showFallbackCubes; },
        [&state](bool v) { state.showFallbackCubes = v; });

    dbgConsole.addBool("show_raycast",
        [&state]() { return state.showRaycast; },
        [&state](bool v) { state.showRaycast = v; });

    dbgConsole.addFloat("move_speed", 1.0f, 500.0f,
        [&state]() { return state.moveSpeed; },
        [&state, refreshTitle](float v) { state.moveSpeed = v; refreshTitle(); });

    dbgConsole.addFloat("wave_amplitude", 0.0f, 5.0f,
        [&state]() { return state.waveAmplitude; },
        [&state](float v) { state.waveAmplitude = v; });

    dbgConsole.addFloat("uv_distortion", 0.0f, 0.5f,
        [&state]() { return state.uvDistortion; },
        [&state](float v) { state.uvDistortion = v; });

    dbgConsole.addFloat("water_rotation", 0.0f, 0.5f,
        [&state]() { return state.waterRotation; },
        [&state](float v) { state.waterRotation = v; });

    dbgConsole.addFloat("water_scroll", 0.0f, 1.0f,
        [&state]() { return state.waterScrollSpeed; },
        [&state](float v) { state.waterScrollSpeed = v; });
}

// ── Event handling ──
// Process SDL events: quit, mouse look, scroll-wheel speed, keyboard shortcuts.
// Mutates camera, runtime flags, and model isolation state.
static void handleEvents(
    Darkness::RuntimeState &state,
    Darkness::DebugConsole &dbgConsole,
    SDL_Window *window)
{
    static constexpr float MOUSE_SENS = 0.002f;
    static constexpr float PI = 3.14159265f;

    SDL_Event ev;
    while (SDL_PollEvent(&ev)) {
        // Debug console gets first crack at all events
        if (dbgConsole.handleEvent(ev)) continue;

        if (ev.type == SDL_QUIT) {
            state.running = false;
        } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_ESCAPE) {
            state.running = false;
        } else if (ev.type == SDL_MOUSEMOTION) {
            state.cam.yaw   -= ev.motion.xrel * MOUSE_SENS;
            state.cam.pitch -= ev.motion.yrel * MOUSE_SENS;
            state.cam.pitch = std::max(-PI * 0.49f, std::min(PI * 0.49f, state.cam.pitch));
        } else if (ev.type == SDL_MOUSEWHEEL) {
            // Scroll wheel adjusts movement speed: 1.5x per tick
            if (ev.wheel.y > 0) {
                for (int i = 0; i < ev.wheel.y; ++i)
                    state.moveSpeed *= 1.5f;
            } else if (ev.wheel.y < 0) {
                for (int i = 0; i < -ev.wheel.y; ++i)
                    state.moveSpeed /= 1.5f;
            }
            state.moveSpeed = std::max(1.0f, std::min(500.0f, state.moveSpeed));
            updateTitleBar(window, state);
        } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_HOME) {
            // Teleport back to spawn point
            state.cam.pos[0] = state.spawnX;
            state.cam.pos[1] = state.spawnY;
            state.cam.pos[2] = state.spawnZ;
            state.cam.yaw = state.spawnYaw;
            state.cam.pitch = 0;
            std::fprintf(stderr, "Teleported to spawn (%.1f, %.1f, %.1f)\n",
                         state.spawnX, state.spawnY, state.spawnZ);
        } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_c
                   && SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_BACKSPACE]) {
            // Backspace+C: Toggle portal culling
            state.portalCulling = !state.portalCulling;
            std::fprintf(stderr, "Portal culling: %s\n",
                         state.portalCulling ? "ON" : "OFF");
        } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_f
                   && SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_BACKSPACE]) {
            // Backspace+F: Cycle texture filtering
            state.filterMode = (state.filterMode + 1) % 4;
            const char *filterNames[] = {
                "point (crispy)", "bilinear", "trilinear", "anisotropic"
            };
            std::fprintf(stderr, "Texture filtering: %s\n", filterNames[state.filterMode]);
        } else if (ev.type == SDL_KEYDOWN
                   && (ev.key.keysym.sym == SDLK_m || ev.key.keysym.sym == SDLK_n)
                   && SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_BACKSPACE]) {
            // Model isolation: M = next model, N = previous model.
            // Cycles through sorted model names, isolating one at a time.
            // Past the last/first model returns to "show all" mode.
            bool forward = (ev.key.keysym.sym == SDLK_m);
            if (state.sortedModelNames.empty()) {
                std::fprintf(stderr, "No models loaded for isolation\n");
            } else {
                if (forward) {
                    state.isolateModelIdx++;
                    if (state.isolateModelIdx >= static_cast<int>(state.sortedModelNames.size()))
                        state.isolateModelIdx = -1;
                } else {
                    state.isolateModelIdx--;
                    if (state.isolateModelIdx < -1)
                        state.isolateModelIdx = static_cast<int>(state.sortedModelNames.size()) - 1;
                }
                if (state.isolateModelIdx < 0) {
                    std::fprintf(stderr, "Model isolation: OFF (showing all)\n");
                } else {
                    const auto &isoName = state.sortedModelNames[state.isolateModelIdx];
                    auto cit = state.modelInstanceCounts.find(isoName);
                    int cnt = (cit != state.modelInstanceCounts.end()) ? cit->second : 0;
                    std::fprintf(stderr, "Isolating model [%d/%zu]: '%s' (%d instances)\n",
                                 state.isolateModelIdx + 1, state.sortedModelNames.size(),
                                 isoName.c_str(), cnt);
                }
                updateTitleBar(window, state);
            }
        } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_v
                   && SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_BACKSPACE]) {
            // Backspace+V: Toggle camera collision with world geometry
            state.cameraCollision = !state.cameraCollision;
            std::fprintf(stderr, "Camera collision: %s\n",
                         state.cameraCollision ? "ON (clip)" : "OFF (noclip)");
            updateTitleBar(window, state);
        } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_l
                   && SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_BACKSPACE]) {
            // Backspace+L: Toggle lightmap filtering (bilinear ↔ bicubic)
            state.lightmapFiltering = (state.lightmapFiltering + 1) % 2;
            const char *lmNames[] = { "bilinear", "bicubic" };
            std::fprintf(stderr, "Lightmap filtering: %s\n", lmNames[state.lightmapFiltering]);
            updateTitleBar(window, state);
        } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_r
                   && SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_BACKSPACE]) {
            // Backspace+R: Toggle debug raycast visualization
            state.showRaycast = !state.showRaycast;
            std::fprintf(stderr, "Raycast debug: %s\n",
                         state.showRaycast ? "ON" : "OFF");
        }
    }
}

// ── Movement update ──
// WASD + vertical movement, suppressed while debug console is open.
// Applies camera collision against world geometry when enabled.
static void updateMovement(
    float dt, Darkness::RuntimeState &state,
    const Darkness::MissionData &mission,
    const Darkness::DebugConsole &dbgConsole)
{
    if (dbgConsole.isOpen()) return;

    const Uint8 *keys = SDL_GetKeyboardState(nullptr);
    float forward = 0, right = 0, up = 0;
    float speed = state.moveSpeed;
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

    // Save position before movement for collision revert
    float oldPos[3] = { state.cam.pos[0], state.cam.pos[1], state.cam.pos[2] };

    state.cam.move(forward, right, up);

    // Camera collision: constrain sphere within world cell planes
    if (state.cameraCollision) {
        applyCameraCollision(mission.wrData, oldPos, state.cam.pos);
    }
}

// ── Animated lightmap update ──
// Advance all light animation timers and re-blend changed lightmaps into
// the atlas CPU buffer, then upload the updated atlas to the GPU.
static void updateLightmaps(
    float dt, const Darkness::BuiltMeshes &meshes,
    Darkness::MissionData &mission, Darkness::GPUResources &gpu)
{
    if (!meshes.lightmappedMode || gpu.lmAtlasSet.atlases.empty()) return;

    bool anyLightChanged = false;
    std::unordered_map<int16_t, float> currentIntensities;

    for (auto &[lightNum, light] : mission.lightSources) {
        bool changed = Darkness::updateLightAnimation(light, dt);
        float intensity = (light.maxBright > 0.0f)
            ? light.brightness / light.maxBright : 0.0f;
        currentIntensities[lightNum] = intensity;
        if (changed) anyLightChanged = true;
    }

    // Re-blend changed lightmaps into atlas CPU buffer, tracking dirty region
    if (anyLightChanged) {
        const auto &atlas = gpu.lmAtlasSet.atlases[0];
        int dirtyX0 = atlas.size, dirtyY0 = atlas.size;
        int dirtyX1 = 0, dirtyY1 = 0;

        for (auto &[lightNum, light] : mission.lightSources) {
            float intensity = currentIntensities[lightNum];
            if (std::abs(intensity - light.prevIntensity) < 0.002f) continue;
            light.prevIntensity = intensity;

            auto it = mission.animLightIndex.find(lightNum);
            if (it == mission.animLightIndex.end()) continue;

            for (auto &[ci, pi] : it->second) {
                const auto &entry = gpu.lmAtlasSet.entries[ci][pi];
                Darkness::blendAnimatedLightmap(
                    gpu.lmAtlasSet.atlases[0], mission.wrData, ci, pi,
                    entry, currentIntensities);

                // Expand dirty region to include this lightmap + its 2px padding
                int x0 = std::max(0, entry.pixelX - 2);
                int y0 = std::max(0, entry.pixelY - 2);
                int x1 = std::min(atlas.size, entry.pixelX + entry.pixelW + 2);
                int y1 = std::min(atlas.size, entry.pixelY + entry.pixelH + 2);
                dirtyX0 = std::min(dirtyX0, x0);
                dirtyY0 = std::min(dirtyY0, y0);
                dirtyX1 = std::max(dirtyX1, x1);
                dirtyY1 = std::max(dirtyY1, y1);
            }
        }

        // Upload only the dirty sub-rectangle to GPU
        if (dirtyX1 > dirtyX0 && dirtyY1 > dirtyY0) {
            int dw = dirtyX1 - dirtyX0;
            int dh = dirtyY1 - dirtyY0;
            std::vector<uint8_t> sub(dw * dh * 4);
            for (int y = 0; y < dh; ++y) {
                std::memcpy(&sub[y * dw * 4],
                            &atlas.rgba[((dirtyY0 + y) * atlas.size + dirtyX0) * 4],
                            dw * 4);
            }
            const bgfx::Memory *mem = bgfx::copy(sub.data(), static_cast<uint32_t>(sub.size()));
            bgfx::updateTexture2D(gpu.lightmapAtlasHandles[0], 0, 0,
                static_cast<uint16_t>(dirtyX0), static_cast<uint16_t>(dirtyY0),
                static_cast<uint16_t>(dw), static_cast<uint16_t>(dh), mem);
        }
    }
}

// ── Frame preparation ──
// Compute per-frame matrices, fog uniforms, sampler flags, underwater state,
// portal culling, and bgfx view transforms. Returns a FrameContext consumed
// by each render pass within a single frame.
static Darkness::FrameContext prepareFrame(
    Darkness::RuntimeState &state,
    const Darkness::MissionData &mission)
{
    using namespace Darkness;
    FrameContext fc{};

    // Projection matrix (shared by sky and world views)
    bx::mtxProj(fc.proj, 60.0f,
                 float(WINDOW_WIDTH) / float(WINDOW_HEIGHT),
                 0.1f, 5000.0f,
                 bgfx::getCaps()->homogeneousDepth,
                 bx::Handedness::Right);

    // Texture sampler flags for the current filtering mode (shared by both views).
    // UINT32_MAX = use texture's baked POINT flags (default).
    // Other modes override with explicit sampler flags per draw call.
    // Two variants: MIRROR wrap (world/object textures), CLAMP wrap (skybox).
    switch (state.filterMode) {
    case 0: // Point: use texture's baked POINT flags
        fc.texSampler = UINT32_MAX;
        fc.skySampler = UINT32_MAX;
        break;
    case 1: // Bilinear: linear min/mag, point mip
        fc.texSampler = BGFX_SAMPLER_U_MIRROR | BGFX_SAMPLER_V_MIRROR
                      | BGFX_SAMPLER_MIP_POINT;
        fc.skySampler = BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP
                      | BGFX_SAMPLER_MIP_POINT;
        break;
    case 2: // Trilinear: linear min/mag/mip
        fc.texSampler = BGFX_SAMPLER_U_MIRROR | BGFX_SAMPLER_V_MIRROR;
        fc.skySampler = BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP;
        break;
    case 3: // Anisotropic: aniso min/mag, linear mip
        fc.texSampler = BGFX_SAMPLER_U_MIRROR | BGFX_SAMPLER_V_MIRROR
                      | BGFX_SAMPLER_MIN_ANISOTROPIC | BGFX_SAMPLER_MAG_ANISOTROPIC;
        fc.skySampler = BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP
                      | BGFX_SAMPLER_MIN_ANISOTROPIC | BGFX_SAMPLER_MAG_ANISOTROPIC;
        break;
    default:
        fc.texSampler = UINT32_MAX;
        fc.skySampler = UINT32_MAX;
        break;
    }

    // ── Underwater detection ──
    // Check if the camera is inside a water cell (mediaType==2).
    // When submerged, override fog with short-range blue-green water fog.
    fc.underwater = (getCameraMediaType(mission.wrData, state.cam.pos[0], state.cam.pos[1], state.cam.pos[2]) == 2);

    // Water fog defaults: dark blue-green tint, short visibility range
    static constexpr float waterFogR = 0.10f, waterFogG = 0.18f, waterFogB = 0.25f;
    static constexpr float waterFogDist = 80.0f;

    // ── Fog uniform values (reused before every bgfx::submit) ──
    // bgfx uniforms are per-submit: cleared after each draw call.
    // We set them before every submit so all shaders receive fog data.
    // When underwater, override with water fog regardless of FOG chunk.
    if (fc.underwater) {
        fc.fogColorArr[0] = waterFogR; fc.fogColorArr[1] = waterFogG;
        fc.fogColorArr[2] = waterFogB; fc.fogColorArr[3] = 1.0f;
        fc.fogOnArr[0] = 1.0f; fc.fogOnArr[1] = waterFogDist;
        fc.fogOnArr[2] = 0.0f; fc.fogOnArr[3] = 0.0f;
    } else {
        fc.fogColorArr[0] = mission.fogParams.r; fc.fogColorArr[1] = mission.fogParams.g;
        fc.fogColorArr[2] = mission.fogParams.b; fc.fogColorArr[3] = 1.0f;
        fc.fogOnArr[0] = mission.fogParams.enabled ? 1.0f : 0.0f;
        fc.fogOnArr[1] = mission.fogParams.distance;
        fc.fogOnArr[2] = 0.0f; fc.fogOnArr[3] = 0.0f;
    }
    fc.fogOffArr[0] = 0.0f; fc.fogOffArr[1] = 1.0f;
    fc.fogOffArr[2] = 0.0f; fc.fogOffArr[3] = 0.0f;

    // Sky fog: underwater always fogs sky; otherwise respect SKYOBJVAR.fog
    fc.skyFogged = fc.underwater || (mission.fogParams.enabled && mission.skyParams.fog);
    fc.skyFogArr = fc.skyFogged ? fc.fogOnArr : fc.fogOffArr;

    // Update clear color per-frame for underwater tinting
    if (fc.underwater) {
        uint8_t cr = static_cast<uint8_t>(waterFogR * 255.0f);
        uint8_t cg = static_cast<uint8_t>(waterFogG * 255.0f);
        uint8_t cb = static_cast<uint8_t>(waterFogB * 255.0f);
        uint32_t waterClear = (uint32_t(cr) << 24) | (uint32_t(cg) << 16)
                            | (uint32_t(cb) << 8) | 0xFF;
        bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
                            waterClear, 1.0f, 0);
    } else {
        bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
                            state.skyClearColor, 1.0f, 0);
    }

    // View 1 transform (world + objects)
    state.cam.getViewMatrix(fc.view);
    bgfx::setViewTransform(1, fc.view, fc.proj);

    // ── Portal culling: determine visible cells ──
    // Build view-projection matrix and extract frustum planes for portal tests.
    // When culling is disabled, visibleCells remains empty (skip filtering).
    if (state.portalCulling) {
        float vp[16];
        bx::mtxMul(vp, fc.view, fc.proj);
        ViewFrustum frustum;
        frustum.extractFromVP(vp);

        int32_t camCell = findCameraCell(mission.wrData, state.cam.pos[0], state.cam.pos[1], state.cam.pos[2]);
        fc.visibleCells = portalBFS(mission.wrData, mission.cellPortals, camCell, frustum,
                                     state.cam.pos[0], state.cam.pos[1], state.cam.pos[2]);
        state.cullVisibleCells = static_cast<uint32_t>(fc.visibleCells.size());
    }

    // Opaque geometry render state (constant each frame)
    fc.renderState = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
                   | BGFX_STATE_WRITE_Z | BGFX_STATE_DEPTH_TEST_LESS
                   | BGFX_STATE_CULL_CW;

    return fc;
}





int main(int argc, char *argv[]) {
    if (argc < 2) {
        printHelp();
        return 1;
    }

    // Parse config: hardcoded defaults → YAML file → CLI overrides
    Darkness::RenderConfig cfg;

    // First CLI pass: extract --config path (and detect --help early)
    Darkness::CliResult cli = Darkness::applyCliOverrides(argc, argv, cfg);

    if (cli.helpRequested) {
        printHelp();
        return 0;
    }

    if (!cli.misPath) {
        std::fprintf(stderr, "Error: no mission file specified.\n\n");
        printHelp();
        return 1;
    }

    // Load YAML config (defaults to ./darknessRender.yaml if no --config flag)
    std::string configPath = cli.configPath.empty() ? "darknessRender.yaml" : cli.configPath;
    Darkness::loadConfigFromYAML(configPath, cfg);

    // Re-apply CLI so flags always win over YAML values
    cli = Darkness::applyCliOverrides(argc, argv, cfg);

    // All CPU-side parsed mission content and mutable runtime state.
    Darkness::MissionData mission;
    Darkness::RuntimeState state;

    // Unpack mutable config into state structs
    const char *misPath    = cli.misPath;
    std::string resPath    = cli.resPath;
    state.showObjects       = cfg.showObjects;
    state.showFallbackCubes = cfg.showFallbackCubes;
    state.portalCulling     = cfg.portalCulling;
    state.cameraCollision   = cfg.cameraCollision;
    state.filterMode        = cfg.filterMode;
    state.lightmapFiltering = cfg.lightmapFiltering;
    state.waveAmplitude    = cfg.waveAmplitude;
    state.uvDistortion     = cfg.uvDistortion;
    state.waterRotation    = cfg.waterRotation;
    state.waterScrollSpeed = cfg.waterScrollSpeed;

    mission.texturedMode = !resPath.empty();

    // ── Initialize logging (required before ServiceManager) ──
    Darkness::Logger logger;
    Darkness::StdLog stdlog;
    logger.setLogLevel(Darkness::Logger::LOG_LEVEL_FATAL);
    Darkness::ConsoleBackend console;

    // ── Initialize service stack, load database, construct IWorldQuery ──
    std::string scriptsDir = "scripts/thief2";
    auto worldQuery = initServiceStack(misPath, scriptsDir);

    // ── Load mission data: WR geometry, portals, spawn, lights, sky, fog, flow ──
    if (!loadMissionData(misPath, cfg.forceFlicker, mission))
        return 1;

    // Inject raycaster into the world query facade — enables ray-vs-world
    // queries (AI line-of-sight, physics, sound occlusion) via portal traversal
    worldQuery->setRaycaster(
        [&mission](const Darkness::Vector3 &from, const Darkness::Vector3 &to,
                  Darkness::RayHit &hit) {
            return Darkness::raycastWorld(mission.wrData, from, to, hit);
        });

    // ── Load world textures: TXLIST, fam.crf textures, flow textures, skybox ──
    loadWorldTextures(misPath, resPath, mission);

    // ── Load object assets: properties, .bin models, textures from obj.crf ──
    loadObjectAssets(misPath, resPath, cfg, mission, state);

    // ── SDL2 + bgfx init ──
    Darkness::GPUResources gpu;
    Darkness::BuiltMeshes meshes;

    // state.skyClearColor set by initWindow
    SDL_Window *window = initWindow(mission.fogParams, state.skyClearColor);
    if (!window) return 1;

    // ── Create all GPU resources: shaders, lightmap atlas, world/object/sky buffers ──
    float camX, camY, camZ;
    if (!createGPUResources(mission, cfg, state.showObjects, meshes, gpu,
                            camX, camY, camZ)) {
        shutdownWindow(window);
        return 1;
    }

    // ── Initialize runtime state: mode string, model isolation, spawn/camera ──
    initRuntimeState(mission, meshes, gpu, camX, camY, camZ, state);

    // ── Debug console for runtime settings management ──
    // Opened with backtick (`), provides tab-completion and value editing.
    Darkness::DebugConsole dbgConsole;
    registerConsoleSettings(dbgConsole, state, window);

    updateTitleBar(window, state);

    auto lastTime = std::chrono::high_resolution_clock::now();
    // state.waterElapsed, state.running have defaults from struct initializer
    while (state.running) {
        auto now = std::chrono::high_resolution_clock::now();
        float dt = std::chrono::duration<float>(now - lastTime).count();
        lastTime = now;
        dt = std::min(dt, 0.1f);
        state.waterElapsed += dt;

        handleEvents(state, dbgConsole, window);

        updateMovement(dt, state, mission, dbgConsole);

        updateLightmaps(dt, meshes, mission, gpu);

        // ── Prepare frame: matrices, fog, samplers, culling ──
        auto fc = prepareFrame(state, mission);
        updateTitleBar(window, state);

        // ── View 0: Sky pass ──
        renderSky(fc, meshes, gpu, mission, state);

        // ── View 1: World geometry ──
        renderWorld(fc, meshes, gpu, mission, state);

        // ── Object meshes ──
        renderObjects(fc, gpu, mission, state);

        // ── Water surfaces ──
        renderWater(fc, meshes, gpu, state);

        // ── Debug raycast visualization (view 2) ──
        renderDebugOverlay(fc, gpu, mission, state);

        // Debug console overlay (no-op when closed)
        dbgConsole.render();

        bgfx::frame();
    }

    destroyGPUResources(gpu);
    shutdownWindow(window);

    std::fprintf(stderr, "Clean shutdown.\n");
    return 0;
}
