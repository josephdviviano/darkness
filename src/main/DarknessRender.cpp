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
#include <functional>
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

static void printHelp() {
    std::fprintf(stderr,
        "darknessRender — Dark Engine world geometry viewer\n"
        "\n"
        "Usage:\n"
        "  darknessRender <mission.mis> [--res <path>] [--config <path>] [--lm-scale <N>]\n"
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
    const Darkness::Camera &cam, const float proj[16],
    bool hasSkybox,
    bgfx::VertexBufferHandle skyboxVBH, bgfx::IndexBufferHandle skyboxIBH,
    const Darkness::SkyboxCube &skyboxCube,
    const std::unordered_map<std::string, bgfx::TextureHandle> &skyboxTexHandles,
    bgfx::VertexBufferHandle skyVBH, bgfx::IndexBufferHandle skyIBH,
    bgfx::ProgramHandle flatProgram, bgfx::ProgramHandle texturedProgram,
    bgfx::UniformHandle s_texColor,
    bgfx::UniformHandle u_fogColor, bgfx::UniformHandle u_fogParams,
    bgfx::UniformHandle u_objectParams,
    const float fogColorArr[4], const float *skyFogArr,
    uint32_t skySampler)
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

    // Inline sky fog uniform helper
    float opaqueParams[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
    auto setFogSky = [&]() {
        bgfx::setUniform(u_fogColor, fogColorArr);
        bgfx::setUniform(u_fogParams, skyFogArr);
        bgfx::setUniform(u_objectParams, opaqueParams);
    };

    if (hasSkybox && bgfx::isValid(skyboxVBH)) {
        // Textured skybox (old sky system) — render each face with its texture
        for (const auto &face : skyboxCube.faces) {
            auto texIt = skyboxTexHandles.find(face.key);
            if (texIt == skyboxTexHandles.end()) continue;

            setFogSky();
            bgfx::setTransform(skyModel);
            bgfx::setVertexBuffer(0, skyboxVBH);
            bgfx::setIndexBuffer(skyboxIBH, face.firstIndex, face.indexCount);
            bgfx::setState(skyState);
            bgfx::setTexture(0, s_texColor, texIt->second, skySampler);
            bgfx::submit(0, texturedProgram);
        }
    } else if (bgfx::isValid(skyVBH)) {
        // Procedural dome (new sky system) — vertex-coloured hemisphere
        setFogSky();
        bgfx::setTransform(skyModel);
        bgfx::setVertexBuffer(0, skyVBH);
        bgfx::setIndexBuffer(skyIBH);
        bgfx::setState(skyState);
        bgfx::submit(0, flatProgram);
    }
}

// Render world geometry into View 1 — lightmapped, textured, or flat-shaded.
// Iterates per-cell draw groups, skipping cells not in the visible set.
static void renderWorld(
    bool lightmappedMode, bool texturedMode,
    bool portalCulling, const std::unordered_set<uint32_t> &visibleCells,
    const Darkness::LightmappedMesh &lmMesh,
    const Darkness::WorldMesh &worldMesh,
    const Darkness::FlatMesh &flatMesh,
    bgfx::VertexBufferHandle vbh, bgfx::IndexBufferHandle ibh,
    bgfx::ProgramHandle flatProgram, bgfx::ProgramHandle texturedProgram,
    bgfx::ProgramHandle lightmappedProgram,
    bgfx::UniformHandle s_texColor, bgfx::UniformHandle s_texLightmap,
    bgfx::UniformHandle u_fogColor, bgfx::UniformHandle u_fogParams,
    bgfx::UniformHandle u_objectParams,
    const float fogColorArr[4], const float fogOnArr[4],
    const std::unordered_map<uint8_t, bgfx::TextureHandle> &textureHandles,
    const std::vector<bgfx::TextureHandle> &lightmapAtlasHandles,
    uint32_t texSampler, uint64_t renderState)
{
    float model[16];
    bx::mtxIdentity(model);

    float opaqueParams[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
    auto setFogOn = [&]() {
        bgfx::setUniform(u_fogColor, fogColorArr);
        bgfx::setUniform(u_fogParams, fogOnArr);
        bgfx::setUniform(u_objectParams, opaqueParams);
    };

    auto isCellVisible = [&](uint32_t cellID) -> bool {
        if (!portalCulling) return true;
        return visibleCells.count(cellID) > 0;
    };

    if (lightmappedMode) {
        for (const auto &grp : lmMesh.groups) {
            if (!isCellVisible(grp.cellID)) continue;

            setFogOn();
            bgfx::setTransform(model);
            bgfx::setVertexBuffer(0, vbh);
            bgfx::setIndexBuffer(ibh, grp.firstIndex, grp.numIndices);
            bgfx::setState(renderState);

            if (grp.txtIndex == 0) {
                bgfx::submit(1, flatProgram);
            } else {
                auto it = textureHandles.find(grp.txtIndex);
                if (it != textureHandles.end()) {
                    bgfx::setTexture(0, s_texColor, it->second, texSampler);
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
            if (!isCellVisible(grp.cellID)) continue;

            setFogOn();
            bgfx::setTransform(model);
            bgfx::setVertexBuffer(0, vbh);
            bgfx::setIndexBuffer(ibh, grp.firstIndex, grp.numIndices);
            bgfx::setState(renderState);

            if (grp.txtIndex == 0) {
                bgfx::submit(1, flatProgram);
            } else {
                auto it = textureHandles.find(grp.txtIndex);
                if (it != textureHandles.end()) {
                    bgfx::setTexture(0, s_texColor, it->second, texSampler);
                    bgfx::submit(1, texturedProgram);
                } else {
                    bgfx::submit(1, flatProgram);
                }
            }
        }
    } else {
        for (const auto &grp : flatMesh.groups) {
            if (!isCellVisible(grp.cellID)) continue;

            setFogOn();
            bgfx::setTransform(model);
            bgfx::setVertexBuffer(0, vbh);
            bgfx::setIndexBuffer(ibh, grp.firstIndex, grp.numIndices);
            bgfx::setState(renderState);
            bgfx::submit(1, flatProgram);
        }
    }
}

// Render water surfaces into View 1 — alpha-blended, no depth write, double-sided.
// Rendered last so all opaque geometry is already in the depth buffer.
static void renderWater(
    bool hasWater,
    const Darkness::WorldMesh &waterMesh,
    bgfx::VertexBufferHandle waterVBH, bgfx::IndexBufferHandle waterIBH,
    bgfx::ProgramHandle flatProgram, bgfx::ProgramHandle waterProgram,
    bgfx::UniformHandle s_texColor,
    bgfx::UniformHandle u_fogColor, bgfx::UniformHandle u_fogParams,
    bgfx::UniformHandle u_objectParams,
    bgfx::UniformHandle u_waterParams, bgfx::UniformHandle u_waterFlow,
    const float fogColorArr[4], const float fogOnArr[4],
    const std::unordered_map<uint8_t, bgfx::TextureHandle> &textureHandles,
    const std::unordered_map<uint8_t, bgfx::TextureHandle> &flowTextureHandles,
    uint32_t texSampler,
    float waterElapsed, float waterScrollSpeed,
    float waveAmplitude, float uvDistortion, float waterRotation)
{
    if (!hasWater) return;

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
        bgfx::setUniform(u_fogColor, fogColorArr);
        bgfx::setUniform(u_fogParams, fogOnArr);
        bgfx::setUniform(u_objectParams, opaqueParams);
    };

    for (const auto &grp : waterMesh.groups) {
        setFogOn();
        bgfx::setTransform(identity);
        bgfx::setVertexBuffer(0, waterVBH);
        bgfx::setIndexBuffer(waterIBH, grp.firstIndex, grp.numIndices);
        bgfx::setState(waterState);

        if (grp.txtIndex != 0 || grp.flowGroup > 0) {
            // Textured water: water shader with vertex displacement + UV distortion
            // u_waterParams: x=elapsed time, y=scroll speed, z=wave amplitude, w=UV distortion
            float wp[4] = { waterElapsed, waterScrollSpeed, waveAmplitude, uvDistortion };
            bgfx::setUniform(u_waterParams, wp);

            // u_waterFlow: x=rotation speed, y=use_world_uv flag, z=tex_unit_len
            // Flow-textured water: vertex shader computes UVs from world position
            // with rotation (matching Dark Engine's flow animation system).
            // TXLIST-textured water: uses pre-computed UVs from mesh.
            bool isFlowTextured = (grp.flowGroup > 0);
            float useWorldUV = isFlowTextured ? 1.0f : 0.0f;
            constexpr float TEX_UNIT_LEN = 4.0f; // 4 world units per texture repeat
            float wf[4] = { waterRotation, useWorldUV, TEX_UNIT_LEN, 0.0f };
            bgfx::setUniform(u_waterFlow, wf);

            // Resolve texture: flow texture by group first, then TXLIST by index
            bgfx::TextureHandle tex = BGFX_INVALID_HANDLE;
            if (grp.flowGroup > 0) {
                auto fit = flowTextureHandles.find(grp.flowGroup);
                if (fit != flowTextureHandles.end())
                    tex = fit->second;
            }
            if (!bgfx::isValid(tex) && grp.txtIndex != 0) {
                auto it = textureHandles.find(grp.txtIndex);
                if (it != textureHandles.end())
                    tex = it->second;
            }

            if (bgfx::isValid(tex)) {
                bgfx::setTexture(0, s_texColor, tex, texSampler);
                bgfx::submit(1, waterProgram);
            } else {
                bgfx::submit(1, flatProgram);
            }
        } else {
            // Non-textured water: flat blue-green from vertex color
            bgfx::submit(1, flatProgram);
        }
    }
}

// Render debug raycast visualization + HUD text into View 2.
// Casts a ray from camera forward, draws cross at hit point, normal line,
// and HUD text overlay with hit details.
static void renderDebugOverlay(
    bool showRaycast,
    const Darkness::Camera &cam, const float view[16], const float proj[16],
    const Darkness::WRParsedData &wrData, const Darkness::TXList &txList,
    bgfx::ProgramHandle flatProgram,
    bgfx::UniformHandle u_fogColor, bgfx::UniformHandle u_fogParams,
    bgfx::UniformHandle u_objectParams,
    const float fogColorArr[4], const float fogOnArr[4])
{
    if (!showRaycast) return;

    // Set up view 2 with same transform as view 1
    bgfx::setViewTransform(2, view, proj);

    // Compute camera forward direction (same as Camera::getViewMatrix)
    float cosPitch = std::cos(cam.pitch);
    float fwdX = std::cos(cam.yaw) * cosPitch;
    float fwdY = std::sin(cam.yaw) * cosPitch;
    float fwdZ = std::sin(cam.pitch);

    // Ray from camera position, extending forward up to 500 world units
    constexpr float RAY_VIS_DIST = 500.0f;
    Darkness::Vector3 rayFrom(cam.pos[0], cam.pos[1], cam.pos[2]);
    Darkness::Vector3 rayTo(cam.pos[0] + fwdX * RAY_VIS_DIST,
                            cam.pos[1] + fwdY * RAY_VIS_DIST,
                            cam.pos[2] + fwdZ * RAY_VIS_DIST);

    Darkness::RayHit rayHit;
    bool rayDidHit = Darkness::raycastWorld(wrData, rayFrom, rayTo, rayHit);

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
            float rtX = std::sin(cam.yaw);
            float rtY = -std::cos(cam.yaw);
            addVert(rayHit.point.x - fwdX * PULLBACK + rtX * OFFSET,
                    rayHit.point.y - fwdY * PULLBACK + rtY * OFFSET,
                    rayHit.point.z - fwdZ * PULLBACK, green);
            addVert(rayHit.point.x, rayHit.point.y, rayHit.point.z, green);
        } else {
            // Yellow line: camera → ray end (visible against sky/void)
            addVert(cam.pos[0], cam.pos[1], cam.pos[2], yellow);
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
        bgfx::setUniform(u_fogColor, fogColorArr);
        bgfx::setUniform(u_fogParams, fogOnArr);
        bgfx::setUniform(u_objectParams, opaqueParams);
        bgfx::submit(2, flatProgram);
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
            static_cast<size_t>(rayHit.textureIndex) < txList.textures.size()) {
            const auto &tex = txList.textures[rayHit.textureIndex];
            bgfx::dbgTextPrintf(2, 8, val_attr, "Texture: %s/%s",
                tex.family.c_str(), tex.name.c_str());
        }
    } else {
        bgfx::dbgTextPrintf(2, 3, 0x0C, "Hit:     NO (miss)");
    }

    // Camera info line
    int32_t camCellIdx = findCameraCell(wrData, cam.pos[0], cam.pos[1], cam.pos[2]);
    bgfx::dbgTextPrintf(2, 10, hud_attr, "Camera:  (%.2f, %.2f, %.2f)  cell=%d",
        cam.pos[0], cam.pos[1], cam.pos[2], camCellIdx);
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
    bool showObjects, bool showFallbackCubes,
    bool portalCulling, const std::unordered_set<uint32_t> &visibleCells,
    int isolateModelIdx, const std::vector<std::string> &sortedModelNames,
    const ObjectPropData &objData,
    const std::vector<int32_t> &objCellIDs,
    const std::unordered_map<std::string, ObjectModelGPU> &objModelGPU,
    const std::unordered_map<std::string, bgfx::TextureHandle> &objTextureHandles,
    bgfx::VertexBufferHandle fallbackCubeVBH, bgfx::IndexBufferHandle fallbackCubeIBH,
    bgfx::ProgramHandle flatProgram, bgfx::ProgramHandle texturedProgram,
    bgfx::UniformHandle s_texColor,
    bgfx::UniformHandle u_fogColor, bgfx::UniformHandle u_fogParams,
    bgfx::UniformHandle u_objectParams,
    const float fogColorArr[4], const float fogOnArr[4],
    uint32_t texSampler, uint64_t renderState)
{
    if (!showObjects) return;

    // Alpha-blend state for translucent submeshes
    uint64_t translucentState = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
                              | BGFX_STATE_WRITE_Z | BGFX_STATE_DEPTH_TEST_LESS
                              | BGFX_STATE_CULL_CW
                              | BGFX_STATE_BLEND_ALPHA;

    // Helper: set fog uniforms (bgfx clears uniforms after each submit)
    float opaqueParams[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
    auto setFogOn = [&]() {
        bgfx::setUniform(u_fogColor, fogColorArr);
        bgfx::setUniform(u_fogParams, fogOnArr);
        bgfx::setUniform(u_objectParams, opaqueParams);
    };

    // Helper: draw one object's submeshes, filtering by opacity.
    // When opaquePass=true, draws only opaque submeshes with renderState.
    // When opaquePass=false, draws only translucent submeshes with translucentState.
    auto drawObjectSubmeshes = [&](size_t oi, bool opaquePass) {
        const auto &obj = objData.objects[oi];
        if (!obj.hasPosition) return;

        // Portal culling: skip objects in non-visible cells.
        if (portalCulling && oi < objCellIDs.size()) {
            int32_t objCell = objCellIDs[oi];
            if (objCell >= 0 && visibleCells.count(static_cast<uint32_t>(objCell)) == 0)
                return;
        }

        // Compute per-object model matrix from position + angles
        float objMtx[16];
        buildModelMatrix(objMtx, obj.x, obj.y, obj.z,
                         obj.heading, obj.pitch, obj.bank,
                         obj.scaleX, obj.scaleY, obj.scaleZ);

        std::string modelName(obj.modelName);

        // Model isolation mode: skip objects that don't match the isolated model
        if (isolateModelIdx >= 0 && isolateModelIdx < (int)sortedModelNames.size()) {
            if (modelName != sortedModelNames[isolateModelIdx])
                return;
        }

        auto it = objModelGPU.find(modelName);

        if (it != objModelGPU.end() && it->second.valid) {
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

                uint64_t state = opaquePass ? renderState : translucentState;

                bgfx::setUniform(u_fogColor, fogColorArr);
                bgfx::setUniform(u_fogParams, fogOnArr);
                bgfx::setUniform(u_objectParams, objAlpha);
                bgfx::setTransform(objMtx);
                bgfx::setVertexBuffer(0, gpuModel.vbh);
                bgfx::setIndexBuffer(gpuModel.ibh, sm.firstIndex, sm.indexCount);
                bgfx::setState(state);

                if (sm.textured) {
                    auto texIt = objTextureHandles.find(sm.matName);
                    if (texIt != objTextureHandles.end()) {
                        bgfx::setTexture(0, s_texColor, texIt->second, texSampler);
                        bgfx::submit(1, texturedProgram);
                    } else {
                        bgfx::submit(1, flatProgram);
                    }
                } else {
                    bgfx::submit(1, flatProgram);
                }
            }
        } else if (showFallbackCubes && opaquePass) {
            // Fallback cubes are always opaque
            setFogOn();
            bgfx::setTransform(objMtx);
            bgfx::setVertexBuffer(0, fallbackCubeVBH);
            bgfx::setIndexBuffer(fallbackCubeIBH);
            bgfx::setState(renderState);
            bgfx::submit(1, flatProgram);
        }
    };

    // Pass 1: opaque submeshes of all objects
    for (size_t oi = 0; oi < objData.objects.size(); ++oi) {
        drawObjectSubmeshes(oi, true);
    }
    // Pass 2: translucent submeshes — rendered after all opaque geometry
    for (size_t oi = 0; oi < objData.objects.size(); ++oi) {
        drawObjectSubmeshes(oi, false);
    }
}

// ── Event handling ──
// Process SDL events: quit, mouse look, scroll-wheel speed, keyboard shortcuts.
// Mutates camera, runtime flags, and model isolation state.
static void handleEvents(
    bool &running, Camera &cam, float &moveSpeed,
    bool &portalCulling, int &filterMode,
    int &isolateModelIdx, bool &cameraCollision, bool &showRaycast,
    float spawnX, float spawnY, float spawnZ, float spawnYaw,
    const std::vector<std::string> &sortedModelNames,
    const std::unordered_map<std::string, int> &modelInstanceCounts,
    Darkness::DebugConsole &dbgConsole,
    std::function<void()> updateTitle)
{
    static constexpr float MOUSE_SENS = 0.002f;
    static constexpr float PI = 3.14159265f;

    SDL_Event ev;
    while (SDL_PollEvent(&ev)) {
        // Debug console gets first crack at all events
        if (dbgConsole.handleEvent(ev)) continue;

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
        } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_c
                   && SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_BACKSPACE]) {
            // Backspace+C: Toggle portal culling
            portalCulling = !portalCulling;
            std::fprintf(stderr, "Portal culling: %s\n",
                         portalCulling ? "ON" : "OFF");
        } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_f
                   && SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_BACKSPACE]) {
            // Backspace+F: Cycle texture filtering
            filterMode = (filterMode + 1) % 4;
            const char *filterNames[] = {
                "point (crispy)", "bilinear", "trilinear", "anisotropic"
            };
            std::fprintf(stderr, "Texture filtering: %s\n", filterNames[filterMode]);
        } else if (ev.type == SDL_KEYDOWN
                   && (ev.key.keysym.sym == SDLK_m || ev.key.keysym.sym == SDLK_n)
                   && SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_BACKSPACE]) {
            // Model isolation: M = next model, N = previous model.
            // Cycles through sorted model names, isolating one at a time.
            // Past the last/first model returns to "show all" mode.
            bool forward = (ev.key.keysym.sym == SDLK_m);
            if (sortedModelNames.empty()) {
                std::fprintf(stderr, "No models loaded for isolation\n");
            } else {
                if (forward) {
                    isolateModelIdx++;
                    if (isolateModelIdx >= static_cast<int>(sortedModelNames.size()))
                        isolateModelIdx = -1;
                } else {
                    isolateModelIdx--;
                    if (isolateModelIdx < -1)
                        isolateModelIdx = static_cast<int>(sortedModelNames.size()) - 1;
                }
                if (isolateModelIdx < 0) {
                    std::fprintf(stderr, "Model isolation: OFF (showing all)\n");
                } else {
                    const auto &isoName = sortedModelNames[isolateModelIdx];
                    auto cit = modelInstanceCounts.find(isoName);
                    int cnt = (cit != modelInstanceCounts.end()) ? cit->second : 0;
                    std::fprintf(stderr, "Isolating model [%d/%zu]: '%s' (%d instances)\n",
                                 isolateModelIdx + 1, sortedModelNames.size(),
                                 isoName.c_str(), cnt);
                }
                updateTitle();
            }
        } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_v
                   && SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_BACKSPACE]) {
            // Backspace+V: Toggle camera collision with world geometry
            cameraCollision = !cameraCollision;
            std::fprintf(stderr, "Camera collision: %s\n",
                         cameraCollision ? "ON (clip)" : "OFF (noclip)");
            updateTitle();
        } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_r
                   && SDL_GetKeyboardState(nullptr)[SDL_SCANCODE_BACKSPACE]) {
            // Backspace+R: Toggle debug raycast visualization
            showRaycast = !showRaycast;
            std::fprintf(stderr, "Raycast debug: %s\n",
                         showRaycast ? "ON" : "OFF");
        }
    }
}

// ── Movement update ──
// WASD + vertical movement, suppressed while debug console is open.
// Applies camera collision against world geometry when enabled.
static void updateMovement(
    float dt, Camera &cam, float moveSpeed,
    bool cameraCollision, const WRParsedData &wrData,
    const Darkness::DebugConsole &dbgConsole)
{
    if (dbgConsole.isOpen()) return;

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

    // Save position before movement for collision revert
    float oldPos[3] = { cam.pos[0], cam.pos[1], cam.pos[2] };

    cam.move(forward, right, up);

    // Camera collision: constrain sphere within world cell planes
    if (cameraCollision) {
        applyCameraCollision(wrData, oldPos, cam.pos);
    }
}

// ── Animated lightmap update ──
// Advance all light animation timers and re-blend changed lightmaps into
// the atlas CPU buffer, then upload the updated atlas to the GPU.
static void updateLightmaps(
    float dt, bool lightmappedMode,
    std::unordered_map<int16_t, LightSource> &lightSources,
    const std::unordered_map<int16_t, std::vector<std::pair<uint32_t, int>>> &animLightIndex,
    LightmapAtlasSet &lmAtlasSet,
    const std::vector<bgfx::TextureHandle> &lightmapAtlasHandles,
    const WRParsedData &wrData, int lmScale)
{
    if (!lightmappedMode || lmAtlasSet.atlases.empty()) return;

    bool anyLightChanged = false;
    std::unordered_map<int16_t, float> currentIntensities;

    for (auto &[lightNum, light] : lightSources) {
        bool changed = Darkness::updateLightAnimation(light, dt);
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
                Darkness::blendAnimatedLightmap(
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

// ── Frame preparation ──
// Compute per-frame matrices, fog uniforms, sampler flags, underwater state,
// portal culling, and bgfx view transforms. Returns a FrameContext consumed
// by each render pass within a single frame.
static Darkness::FrameContext prepareFrame(
    const Camera &cam, int filterMode,
    bool portalCulling, uint32_t skyClearColor,
    const WRParsedData &wrData,
    const std::vector<std::vector<CellPortalInfo>> &cellPortals,
    const FogParams &fogParams, const SkyParams &skyParams,
    uint32_t &outCullVisibleCells)
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
    switch (filterMode) {
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
    fc.underwater = (getCameraMediaType(wrData, cam.pos[0], cam.pos[1], cam.pos[2]) == 2);

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
        fc.fogColorArr[0] = fogParams.r; fc.fogColorArr[1] = fogParams.g;
        fc.fogColorArr[2] = fogParams.b; fc.fogColorArr[3] = 1.0f;
        fc.fogOnArr[0] = fogParams.enabled ? 1.0f : 0.0f;
        fc.fogOnArr[1] = fogParams.distance;
        fc.fogOnArr[2] = 0.0f; fc.fogOnArr[3] = 0.0f;
    }
    fc.fogOffArr[0] = 0.0f; fc.fogOffArr[1] = 1.0f;
    fc.fogOffArr[2] = 0.0f; fc.fogOffArr[3] = 0.0f;

    // Sky fog: underwater always fogs sky; otherwise respect SKYOBJVAR.fog
    fc.skyFogged = fc.underwater || (fogParams.enabled && skyParams.fog);
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
                            skyClearColor, 1.0f, 0);
    }

    // View 1 transform (world + objects)
    cam.getViewMatrix(fc.view);
    bgfx::setViewTransform(1, fc.view, fc.proj);

    // ── Portal culling: determine visible cells ──
    // Build view-projection matrix and extract frustum planes for portal tests.
    // When culling is disabled, visibleCells remains empty (skip filtering).
    if (portalCulling) {
        float vp[16];
        bx::mtxMul(vp, fc.view, fc.proj);
        ViewFrustum frustum;
        frustum.extractFromVP(vp);

        int32_t camCell = findCameraCell(wrData, cam.pos[0], cam.pos[1], cam.pos[2]);
        fc.visibleCells = portalBFS(wrData, cellPortals, camCell, frustum,
                                     cam.pos[0], cam.pos[1], cam.pos[2]);
        outCullVisibleCells = static_cast<uint32_t>(fc.visibleCells.size());
    }

    // Opaque geometry render state (constant each frame)
    fc.renderState = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
                   | BGFX_STATE_WRITE_Z | BGFX_STATE_DEPTH_TEST_LESS
                   | BGFX_STATE_CULL_CW;

    return fc;
}

// ── Cleanup functions ──
// Destroy all bgfx GPU resources created during initialization.
static void destroyGPUResources(
    bgfx::VertexBufferHandle vbh, bgfx::IndexBufferHandle ibh,
    bgfx::ProgramHandle flatProgram, bgfx::ProgramHandle texturedProgram,
    bgfx::ProgramHandle lightmappedProgram, bgfx::ProgramHandle waterProgram,
    bgfx::UniformHandle s_texColor, bgfx::UniformHandle s_texLightmap,
    bgfx::UniformHandle u_waterParams, bgfx::UniformHandle u_waterFlow,
    bgfx::UniformHandle u_fogColor, bgfx::UniformHandle u_fogParams,
    bgfx::UniformHandle u_objectParams,
    std::unordered_map<uint8_t, bgfx::TextureHandle> &textureHandles,
    std::unordered_map<uint8_t, bgfx::TextureHandle> &flowTextureHandles,
    std::vector<bgfx::TextureHandle> &lightmapAtlasHandles,
    bgfx::VertexBufferHandle waterVBH, bgfx::IndexBufferHandle waterIBH,
    bgfx::VertexBufferHandle skyboxVBH, bgfx::IndexBufferHandle skyboxIBH,
    std::unordered_map<std::string, bgfx::TextureHandle> &skyboxTexHandles,
    bgfx::VertexBufferHandle skyVBH, bgfx::IndexBufferHandle skyIBH,
    std::unordered_map<std::string, Darkness::ObjectModelGPU> &objModelGPU,
    std::unordered_map<std::string, bgfx::TextureHandle> &objTextureHandles,
    bgfx::VertexBufferHandle fallbackCubeVBH, bgfx::IndexBufferHandle fallbackCubeIBH)
{
    // Water surface buffers and flow textures
    if (bgfx::isValid(waterVBH)) bgfx::destroy(waterVBH);
    if (bgfx::isValid(waterIBH)) bgfx::destroy(waterIBH);
    for (auto &kv : flowTextureHandles)
        bgfx::destroy(kv.second);

    // Textured skybox buffers
    if (bgfx::isValid(skyboxVBH)) bgfx::destroy(skyboxVBH);
    if (bgfx::isValid(skyboxIBH)) bgfx::destroy(skyboxIBH);
    for (auto &kv : skyboxTexHandles)
        bgfx::destroy(kv.second);

    // Sky dome buffers
    if (bgfx::isValid(skyVBH)) bgfx::destroy(skyVBH);
    if (bgfx::isValid(skyIBH)) bgfx::destroy(skyIBH);

    // Object GPU buffers and textures
    for (auto &kv : objModelGPU) {
        if (bgfx::isValid(kv.second.vbh)) bgfx::destroy(kv.second.vbh);
        if (bgfx::isValid(kv.second.ibh)) bgfx::destroy(kv.second.ibh);
    }
    for (auto &kv : objTextureHandles) {
        bgfx::destroy(kv.second);
    }
    if (bgfx::isValid(fallbackCubeVBH)) bgfx::destroy(fallbackCubeVBH);
    if (bgfx::isValid(fallbackCubeIBH)) bgfx::destroy(fallbackCubeIBH);

    // World geometry and textures
    for (auto &h : lightmapAtlasHandles)
        bgfx::destroy(h);
    for (auto &kv : textureHandles)
        bgfx::destroy(kv.second);
    bgfx::destroy(s_texLightmap);
    bgfx::destroy(s_texColor);
    bgfx::destroy(u_waterParams);
    bgfx::destroy(u_waterFlow);
    bgfx::destroy(u_fogColor);
    bgfx::destroy(u_fogParams);
    bgfx::destroy(u_objectParams);

    bgfx::destroy(ibh);
    bgfx::destroy(vbh);
    bgfx::destroy(flatProgram);
    bgfx::destroy(texturedProgram);
    bgfx::destroy(lightmappedProgram);
    bgfx::destroy(waterProgram);
}

// Initialize SDL2 window and bgfx rendering context.
// Sets up 3 views: sky (0), world+objects (1), debug overlay (2).
// Returns the SDL window, or nullptr on failure.
static SDL_Window *initWindow(const Darkness::FogParams &fogParams,
                               uint32_t &outSkyClearColor) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return nullptr;
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
        return nullptr;
    }

    SDL_SysWMinfo wmi;
    SDL_VERSION(&wmi.version);
    if (!SDL_GetWindowWMInfo(window, &wmi)) {
        std::fprintf(stderr, "SDL_GetWindowWMInfo failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return nullptr;
    }

    bgfx::renderFrame(); // single-threaded mode

    bgfx::Init bInit;
    bInit.type = bgfx::RendererType::Count; // auto-detect: Metal/D3D11/OpenGL/Vulkan
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
        return nullptr;
    }

    // View 0: Sky pass — clears colour + depth, renders sky dome with no depth writes
    // When fog is enabled, use fog colour as clear colour so uncovered sky matches
    outSkyClearColor = 0x1a1a2eFF;
    if (fogParams.enabled) {
        uint8_t fr = static_cast<uint8_t>(fogParams.r * 255.0f);
        uint8_t fg = static_cast<uint8_t>(fogParams.g * 255.0f);
        uint8_t fb = static_cast<uint8_t>(fogParams.b * 255.0f);
        outSkyClearColor = (uint32_t(fr) << 24) | (uint32_t(fg) << 16) | (uint32_t(fb) << 8) | 0xFF;
    }
    bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
                        outSkyClearColor, 1.0f, 0);
    bgfx::setViewRect(0, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);

    // View 1: World + objects pass — clears depth only, preserves sky colour
    bgfx::setViewClear(1, BGFX_CLEAR_DEPTH, 0, 1.0f, 0);
    bgfx::setViewRect(1, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);

    // View 2: Debug overlay — no clear, renders on top of view 1.
    // Separate view ensures debug lines are drawn AFTER all world geometry,
    // regardless of bgfx's internal draw call sorting within a view.
    bgfx::setViewClear(2, BGFX_CLEAR_NONE, 0, 1.0f, 0);
    bgfx::setViewRect(2, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);

    Darkness::PosColorVertex::init();
    Darkness::PosColorUVVertex::init();
    Darkness::PosUV2Vertex::init();

    return window;
}

// Shut down bgfx and SDL2 window.
static void shutdownWindow(SDL_Window *window) {
    bgfx::shutdown();
    SDL_DestroyWindow(window);
    SDL_Quit();
}

// ── Initialize service stack and load mission database ──
// Creates the ServiceManager with all 12 services, registers property/relation
// schemas from PLDef/DType files, loads the mission database (.gam + .mis),
// constructs the IWorldQuery facade, and runs verification diagnostics.
// Logger objects must be constructed before calling this (they live in main).
static std::unique_ptr<Darkness::ObjSysWorldState> initServiceStack(
    const char *misPath, const std::string &scriptsDir)
{
    // Create service manager with all 12 services
    {
        auto *svcMgr = new Darkness::ServiceManager(SERVICE_ALL);

        svcMgr->registerFactory<Darkness::PlatformServiceFactory>();
        svcMgr->registerFactory<Darkness::ConfigServiceFactory>();
        svcMgr->registerFactory<Darkness::DatabaseServiceFactory>();
        svcMgr->registerFactory<Darkness::GameServiceFactory>();
        svcMgr->registerFactory<Darkness::InheritServiceFactory>();
        svcMgr->registerFactory<Darkness::LinkServiceFactory>();
        svcMgr->registerFactory<Darkness::LoopServiceFactory>();
        svcMgr->registerFactory<Darkness::ObjectServiceFactory>();
        svcMgr->registerFactory<Darkness::PropertyServiceFactory>();
        svcMgr->registerFactory<Darkness::RoomServiceFactory>();
        svcMgr->registerFactory<Darkness::SimServiceFactory>();
        svcMgr->registerFactory<Darkness::PhysicsServiceFactory>();

        svcMgr->bootstrapFinished();
    }

    // Load schema definitions (property types, link relations)
    {
        Darkness::PropertyServicePtr propSvc = GET_SERVICE(Darkness::PropertyService);
        Darkness::LinkServicePtr linkSvc = GET_SERVICE(Darkness::LinkService);

        Darkness::PLDefResult propDefs = Darkness::parsePLDef(scriptsDir + "/t2-props.pldef");
        Darkness::PLDefResult linkDefs = Darkness::parsePLDef(scriptsDir + "/t2-links.pldef");
        auto dtypeSizes = Darkness::parseDTypeSizes(scriptsDir + "/t2-types.dtype");

        int propCount = 0;
        for (const auto &pd : propDefs.properties) {
            if (propSvc->getProperty(pd.name))
                continue;
            Darkness::DataStoragePtr storage;
            if (pd.isVarStr) {
                storage = Darkness::DataStoragePtr(new Darkness::StringDataStorage());
            } else {
                storage = Darkness::DataStoragePtr(new Darkness::RawDataStorage());
            }
            Darkness::Property *prop = propSvc->createProperty(
                pd.name, pd.name, pd.inheritor, storage);
            if (prop) {
                prop->setChunkVersions(pd.verMaj, pd.verMin);
                ++propCount;
            }
        }

        int relCount = 0;
        for (const auto &rd : linkDefs.relations) {
            if (linkSvc->getRelation(rd.name))
                continue;
            Darkness::DataStoragePtr storage;
            if (!rd.noData) {
                auto it = dtypeSizes.find(rd.name);
                size_t dataSize = (it != dtypeSizes.end()) ? it->second : 0;
                storage = Darkness::DataStoragePtr(new Darkness::RawDataStorage(dataSize));
            }
            Darkness::RelationPtr rel = linkSvc->createRelation(rd.name, storage, rd.hidden);
            if (rel) {
                rel->setChunkVersions(rd.lVerMaj, rd.lVerMin,
                                      rd.dVerMaj, rd.dVerMin);
                if (rd.fakeSize >= 0)
                    rel->setFakeSize(rd.fakeSize);
                ++relCount;
            }
        }

        // Register rendering properties that are commented out in t2-props.pldef
        // (normally created by RenderService, which the standalone viewer doesn't use).
        // These must exist before database loading so their P$ chunks get read.
        auto registerRawProp = [&](const char *name, const char *chunk,
                                   const char *inheritor) {
            if (!propSvc->getProperty(name)) {
                propSvc->createProperty(name, chunk, inheritor,
                    Darkness::DataStoragePtr(new Darkness::RawDataStorage()));
                ++propCount;
            }
        };
        registerRawProp("ModelName",   "ModelName", "always");
        registerRawProp("RenderType",  "RenderTyp", "always");
        registerRawProp("RenderAlpha", "RenderAlp", "always");
        // Scale uses "never" inheritor — kPropertyNoInherit in the original engine.
        // Archetype scales are physics bounding boxes, not visual model scales.
        registerRawProp("ModelScale",  "Scale",     "never");

        std::fprintf(stderr, "Schema: registered %d properties, %d relations\n",
                     propCount, relCount);
    }

    // Load the mission database (.gam + .mis) via service stack.
    // GameService::load() recursively loads the parent .gam first, then the .mis.
    // This populates PropertyService with all P$ data and LinkService with all L$ data.
    {
        Darkness::GameServicePtr gameSvc = GET_SERVICE(Darkness::GameService);
        gameSvc->load(misPath);
    }

    // Construct IWorldQuery facade — the read-only interface for downstream subsystems.
    // Currently used for verification only; will be passed to AI, audio, scripts.
    Darkness::ObjectServicePtr objSvcPtr = GET_SERVICE(Darkness::ObjectService);
    Darkness::PropertyServicePtr propSvcPtr = GET_SERVICE(Darkness::PropertyService);
    Darkness::LinkServicePtr linkSvcPtr = GET_SERVICE(Darkness::LinkService);
    Darkness::RoomServicePtr roomSvcPtr = GET_SERVICE(Darkness::RoomService);

    auto worldQuery = std::make_unique<Darkness::ObjSysWorldState>(
        objSvcPtr.get(), propSvcPtr.get(), linkSvcPtr.get(), roomSvcPtr.get());

    // Verification: exercise IWorldQuery methods to ensure correctness
    {
        int entityCount = 0, positionedCount = 0, roomCount = 0, linkCount = 0;

        // Count positioned entities via property access
        auto positionedIDs = worldQuery->getAllWithProperty("Position");
        for (auto eid : positionedIDs) {
            if (!worldQuery->exists(eid))
                continue;
            ++entityCount;

            Darkness::Vector3 pos = worldQuery->getPosition(eid);
            // Verify consistency: getPosition matches getProperty<T> raw data
            if (pos.x != 0.0f || pos.y != 0.0f || pos.z != 0.0f)
                ++positionedCount;
        }

        // Count rooms and their portals
        if (roomSvcPtr->isLoaded()) {
            const auto &rooms = roomSvcPtr->getAllRooms();
            for (const auto &r : rooms) {
                if (!r)
                    continue;
                ++roomCount;
            }
        }

        // Count links via MetaProp relation — test back links (incoming to -1).
        // Archetype -1 is the root; MetaProp links go TO it from other archetypes.
        // Uses string-based getBackLinks which resolves ~MetaProp internally.
        {
            auto backLinks = worldQuery->getBackLinks(-1, "MetaProp", 0);
            linkCount = static_cast<int>(backLinks.size());
        }

        // Test property handle caching (hot-path variant)
        auto posHandle = worldQuery->resolveProperty("Position");
        int handleHits = 0;
        if (posHandle) {
            for (auto eid : positionedIDs) {
                if (worldQuery->hasProperty(eid, posHandle))
                    ++handleHits;
            }
        }

        std::fprintf(stderr,
                     "IWorldQuery: verified %d entities (%d positioned), "
                     "%d rooms, %d MetaProp backlinks to -1, "
                     "handle cache: %d/%zu hits\n",
                     entityCount, positionedCount, roomCount, linkCount,
                     handleHits, positionedIDs.size());
    }

    return worldQuery;
}

// ── Load mission data from .mis file ──
// Parses WR geometry, portal graph, spawn point, animated lights, sky/fog/flow
// parameters, and extracts mission base name. Returns false if WR parse fails.
static bool loadMissionData(const char *misPath, bool forceFlicker,
                            Darkness::MissionData &mission)
{
    // Extract mission base name (e.g. "miss6" from "path/to/miss6.mis")
    // for constructing skybox texture filenames like "skyhw/miss6n.PCX"
    {
        std::string p(misPath);
        size_t slash = p.find_last_of("/\\");
        std::string base = (slash != std::string::npos) ? p.substr(slash + 1) : p;
        size_t dot = base.find('.');
        mission.missionName = (dot != std::string::npos) ? base.substr(0, dot) : base;
        // Lowercase for case-insensitive CRF matching
        std::transform(mission.missionName.begin(), mission.missionName.end(),
                       mission.missionName.begin(),
                       [](unsigned char c) { return std::tolower(c); });
    }

    // Parse WR geometry
    std::fprintf(stderr, "Loading WR geometry from %s...\n", misPath);
    try {
        mission.wrData = Darkness::parseWRChunk(misPath);
    } catch (const std::exception &e) {
        std::fprintf(stderr, "Failed to parse WR chunk: %s\n", e.what());
        return false;
    }
    std::fprintf(stderr, "Loaded %u cells\n", mission.wrData.numCells);

    // Build portal adjacency graph for portal culling
    mission.cellPortals = buildPortalGraph(mission.wrData);
    {
        int totalPortals = 0;
        for (const auto &pl : mission.cellPortals)
            totalPortals += static_cast<int>(pl.size());
        std::fprintf(stderr, "Portal graph: %d portals across %u cells\n",
                     totalPortals, mission.wrData.numCells);
    }

    // Find player spawn point from L$PlayerFactory + P$Position chunks
    mission.spawnInfo = Darkness::findSpawnPoint(misPath);

    // Parse animated light properties from mission database
    mission.lightSources = Darkness::parseAnimLightProperties(misPath);

    // Build reverse index: lightnum → list of (cellIdx, polyIdx) affected
    mission.animLightIndex = Darkness::buildAnimLightIndex(mission.wrData);

    // Ensure all lightnums referenced in WR data have a LightSource entry.
    // Lights without P$AnimLight properties default to mode 4 (max brightness).
    for (const auto &kv : mission.animLightIndex) {
        if (mission.lightSources.find(kv.first) == mission.lightSources.end()) {
            Darkness::LightSource ls = {};
            ls.lightNum = kv.first;
            ls.mode = Darkness::ANIM_MAX_BRIGHT;
            ls.maxBright = 1.0f;
            ls.minBright = 0.0f;
            ls.brightness = 1.0f;
            ls.prevIntensity = 1.0f;
            mission.lightSources[kv.first] = ls;
        }
    }

    // Apply --force-flicker: override all lights to flicker mode for debugging
    if (forceFlicker) {
        for (auto &[num, ls] : mission.lightSources) {
            ls.mode = Darkness::ANIM_FLICKER;
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
                     mission.lightSources.size());
    }

    // Animated light diagnostics
    {
        int modeCounts[10] = {};
        int inactiveCount = 0;
        int fromMIS = 0, fromDefault = 0;
        for (const auto &[num, ls] : mission.lightSources) {
            if (ls.mode < 10) modeCounts[ls.mode]++;
            if (ls.inactive) inactiveCount++;
            if (ls.objectId != 0) fromMIS++; else fromDefault++;
        }
        std::fprintf(stderr, "Animated lights: %zu sources (%d from MIS, %d defaulted), "
                     "%zu indexed lightnums\n",
                     mission.lightSources.size(), fromMIS, fromDefault,
                     mission.animLightIndex.size());
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
    mission.skyParams = parseSkyObjVar(misPath);
    mission.skyDome = buildSkyDome(mission.skyParams);

    // Parse global fog parameters from FOG chunk (if present)
    mission.fogParams = parseFogChunk(misPath);

    // Parse water flow data — FLOW_TEX (texture mapping) and CELL_MOTION (animation state)
    mission.flowData = parseFlowData(misPath);

    return true;
}

// ── Load world textures from CRF archives ──
// Parses TXLIST, loads world textures from fam.crf, water flow textures,
// and skybox face textures. All images stored in mission for later GPU upload.
static void loadWorldTextures(const char *misPath, const std::string &resPath,
                              Darkness::MissionData &mission)
{
    // Parse TXLIST if in textured mode
    if (mission.texturedMode) {
        try {
            mission.txList = Darkness::parseTXList(misPath);
            std::fprintf(stderr, "TXLIST: %zu textures, %zu families\n",
                         mission.txList.textures.size(), mission.txList.families.size());
        } catch (const std::exception &e) {
            std::fprintf(stderr, "Failed to parse TXLIST: %s (falling back to flat)\n",
                         e.what());
            mission.texturedMode = false;
        }
    }

    // Collect unique texture indices used by world geometry
    std::unordered_set<uint8_t> usedTextures;
    if (mission.texturedMode) {
        for (const auto &cell : mission.wrData.cells) {
            for (int pi = 0; pi < cell.numTextured; ++pi) {
                uint8_t txt = cell.texturing[pi].txt;
                if (txt != 0 && txt != 249)
                    usedTextures.insert(txt);
            }
        }
        // Note: FLOW_TEX texture indices are runtime palette positions, NOT TXLIST
        // indices. Water textures are loaded separately by name from fam.crf below.
        std::fprintf(stderr, "Unique texture indices used: %zu\n", usedTextures.size());
    }

    // Load world textures from fam.crf (indexed by TXLIST)
    if (mission.texturedMode) {
        Darkness::CRFTextureLoader loader(resPath);
        if (!loader.isOpen()) {
            std::fprintf(stderr, "CRF not available, falling back to flat shading\n");
            mission.texturedMode = false;
        } else {
            int loaded = 0;
            for (uint8_t idx : usedTextures) {
                if (idx >= mission.txList.textures.size()) continue;
                const auto &entry = mission.txList.textures[idx];
                auto img = loader.loadTexture(entry.family, entry.name);
                mission.texDims[idx] = { img.width, img.height };
                mission.loadedTextures[idx] = std::move(img);
                ++loaded;
            }
            std::fprintf(stderr, "Loaded %d/%zu textures from CRF\n",
                         loaded, usedTextures.size());
        }
    }

    // Load water flow textures from fam.crf by name.
    // FLOW_TEX name field (e.g. "gr") maps to "water/<name>in.PCX" for the
    // air-side texture and "water/<name>out.PCX" for the underwater side.
    // Keyed by flow group index (1-255).
    if (mission.texturedMode && mission.flowData.hasFlowTex) {
        Darkness::CRFTextureLoader loader(resPath);
        if (loader.isOpen()) {
            // Collect unique flow groups used by cells
            std::unordered_set<uint8_t> usedFlowGroups;
            for (const auto &cell : mission.wrData.cells) {
                if (cell.flowGroup > 0)
                    usedFlowGroups.insert(cell.flowGroup);
            }

            int loaded = 0;
            for (uint8_t fg : usedFlowGroups) {
                const auto &fe = mission.flowData.textures[fg];
                if (fe.name[0] == '\0') continue;

                // Extract base name, trimming trailing nulls/spaces
                std::string baseName(fe.name, strnlen(fe.name, 28));
                while (!baseName.empty() && (baseName.back() == ' ' || baseName.back() == '\0'))
                    baseName.pop_back();
                if (baseName.empty()) continue;

                // Air-side texture: "water/<name>in" (e.g. "gr" → "water/grin")
                std::string inName = baseName + "in";
                auto img = loader.loadTexture("water", inName);

                // Check if we got a real texture (not the 8x8 fallback)
                if (img.width > 8 || img.height > 8) {
                    std::fprintf(stderr, "Flow group %d: loaded water/%s.PCX (%ux%u)\n",
                                 fg, inName.c_str(), img.width, img.height);
                    mission.flowTexDims[fg] = { img.width, img.height };
                    mission.flowLoadedTextures[fg] = std::move(img);
                    ++loaded;
                } else {
                    std::fprintf(stderr, "Flow group %d: water/%s.PCX not found, trying waterhw/\n",
                                 fg, inName.c_str());
                    // Some missions may use WATERHW family instead
                    auto img2 = loader.loadTexture("waterhw", inName);
                    if (img2.width > 8 || img2.height > 8) {
                        std::fprintf(stderr, "Flow group %d: loaded waterhw/%s.PCX (%ux%u)\n",
                                     fg, inName.c_str(), img2.width, img2.height);
                        mission.flowTexDims[fg] = { img2.width, img2.height };
                        mission.flowLoadedTextures[fg] = std::move(img2);
                        ++loaded;
                    } else {
                        std::fprintf(stderr, "Flow group %d: no water texture found for '%s'\n",
                                     fg, baseName.c_str());
                    }
                }
            }
            if (loaded > 0) {
                std::fprintf(stderr, "Loaded %d flow group water textures from CRF\n", loaded);
            }
        }
    }

    // Load skybox face textures (old sky system).
    // Missions without SKYOBJVAR use a textured skybox with per-mission PCX
    // textures in fam.crf under skyhw/ (e.g. skyhw/miss6n.PCX for north face).
    if (mission.texturedMode) {
        Darkness::CRFTextureLoader skyLoader(resPath);
        if (skyLoader.isOpen()) {
            // 5 faces: n=north(+Y), s=south(-Y), e=east(+X), w=west(-X), t=top(+Z)
            const char *suffixes[] = { "n", "s", "e", "w", "t" };
            int loaded = 0;
            for (const char *suf : suffixes) {
                std::string texName = mission.missionName + suf;
                auto img = skyLoader.loadTexture("skyhw", texName);
                // Real texture is larger than the 8x8 fallback checkerboard
                if (img.width > 8 || img.height > 8) {
                    mission.skyboxImages[suf] = std::move(img);
                    ++loaded;
                }
            }
            // Skybox available if at least the 4 side faces loaded (top optional)
            mission.hasSkybox = mission.skyboxImages.count("n") && mission.skyboxImages.count("s")
                     && mission.skyboxImages.count("e") && mission.skyboxImages.count("w");
            if (mission.hasSkybox) {
                std::fprintf(stderr, "Skybox: loaded %d/5 faces for %s (textured skybox active)\n",
                             loaded, mission.missionName.c_str());
                for (auto &kv : mission.skyboxImages) {
                    std::fprintf(stderr, "  face '%s': %ux%u\n",
                                 kv.first.c_str(), kv.second.width, kv.second.height);
                }
            } else if (loaded > 0) {
                std::fprintf(stderr, "Skybox: partial load (%d faces), falling back to dome\n", loaded);
            }
        }
    }
}

// ── Load object assets: properties, .bin models, textures from obj.crf ──
// Parses object placements via PropertyService, precomputes per-object cell IDs
// for portal culling, loads .bin meshes and object textures from obj.crf.
static void loadObjectAssets(const char *misPath, const std::string &resPath,
                             const Darkness::RenderConfig &cfg,
                             const Darkness::WRParsedData &wrData,
                             Darkness::MissionData &mission,
                             Darkness::RuntimeState &state)
{
    // Parse object placements from .mis via PropertyService
    if (state.showObjects) {
        try {
            Darkness::PropertyServicePtr propSvc = GET_SERVICE(Darkness::PropertyService);
            mission.objData = Darkness::parseObjectProps(propSvc.get(), misPath,
                                                    cfg.debugObjects);
        } catch (const std::exception &e) {
            std::fprintf(stderr, "Failed to parse object props: %s\n", e.what());
            state.showObjects = false;
        }
        if (mission.objData.objects.empty()) {
            std::fprintf(stderr, "No objects to render\n");
            state.showObjects = false;
        }
    }

    // Precompute which cell each object is in for portal culling.
    // Objects don't move (yet), so this is a one-time lookup at load time.
    // -1 = outside all cells (always rendered to avoid popping).
    if (state.showObjects) {
        mission.objCellIDs.resize(mission.objData.objects.size());
        for (size_t i = 0; i < mission.objData.objects.size(); ++i) {
            const auto &obj = mission.objData.objects[i];
            if (obj.hasPosition) {
                mission.objCellIDs[i] = findCameraCell(wrData, obj.x, obj.y, obj.z);
            } else {
                mission.objCellIDs[i] = -1;
            }
        }
    }

    // Load .bin models from obj.crf (if --res provided and objects enabled)
    if (state.showObjects && !resPath.empty()) {
        Darkness::CRFModelLoader modelLoader(resPath);
        if (modelLoader.isOpen()) {
            int loaded = 0, failed = 0;
            for (const auto &name : mission.objData.uniqueModels) {
                auto binData = modelLoader.loadModel(name);
                if (binData.empty()) {
                    ++failed;
                    continue;
                }
                try {
                    auto mesh = Darkness::parseBinModel(binData.data(), binData.size());
                    if (mesh.valid) {
                        mission.parsedModels[name] = std::move(mesh);
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
                         loaded, mission.objData.uniqueModels.size(), failed);
        } else {
            std::fprintf(stderr, "obj.crf not available, using fallback cubes\n");
        }
    }

    // Load object textures from obj.crf (txt16/ and txt/ subdirectories inside it).
    // Dark Engine stores object textures as GIF/PCX files within obj.crf, not in
    // separate txt16.crf/txt.crf archives.

    // Collect unique MD_MAT_TMAP material names from all parsed models
    std::unordered_set<std::string> objMatNames;
    for (const auto &kv : mission.parsedModels) {
        for (const auto &mat : kv.second.materials) {
            if (mat.type == Darkness::MD_MAT_TMAP) {
                // Lowercase the name for case-insensitive matching
                std::string lname(mat.name);
                std::transform(lname.begin(), lname.end(), lname.begin(),
                               [](unsigned char c) { return std::tolower(c); });
                objMatNames.insert(lname);
            }
        }
    }

    if (!objMatNames.empty() && !resPath.empty()) {
        // Reuse obj.crf for texture lookup (same archive that holds .bin models)
        Darkness::CRFTextureLoader objTexLoader(resPath, "obj.crf");

        int loaded = 0;
        for (const auto &name : objMatNames) {
            if (!objTexLoader.isOpen()) break;
            auto img = objTexLoader.loadObjectTexture(name);
            // Check if we got a real texture (not the 8x8 fallback checkerboard)
            if (img.width > 8 || img.height > 8) {
                mission.objTexImages[name] = std::move(img);
                ++loaded;
            }
        }
        std::fprintf(stderr, "Loaded %d/%zu object textures from obj.crf\n",
                     loaded, objMatNames.size());
    }
}

// ── Create all GPU resources: shaders, lightmap atlas, world/object/sky buffers ──
// Builds mesh data from MissionData, uploads to bgfx, creates shader programs
// and uniform handles. Returns false if world geometry is empty (fatal).
// centroidX/Y/Z receive the world geometry centroid for camera init.
static bool createGPUResources(const Darkness::MissionData &mission,
                               const Darkness::RenderConfig &cfg,
                               bool showObjects,
                               Darkness::BuiltMeshes &meshes,
                               Darkness::GPUResources &gpu,
                               float &centroidX, float &centroidY, float &centroidZ)
{
    int lmScale = cfg.lmScale;
    bool linearMips = cfg.linearMips;
    bool sharpMips = cfg.sharpMips;

    // ── Shaders ──
    // Load from cross-platform embedded shader table (auto-selects Metal/D3D/GL/Vulkan)
    bgfx::RendererType::Enum rendererType = bgfx::getRendererType();

    // Flat-color program
    gpu.flatProgram = bgfx::createProgram(
        bgfx::createEmbeddedShader(s_embeddedShaders, rendererType, "vs_basic"),
        bgfx::createEmbeddedShader(s_embeddedShaders, rendererType, "fs_basic"),
        true);

    // Textured program
    gpu.texturedProgram = bgfx::createProgram(
        bgfx::createEmbeddedShader(s_embeddedShaders, rendererType, "vs_textured"),
        bgfx::createEmbeddedShader(s_embeddedShaders, rendererType, "fs_textured"),
        true);

    // Lightmapped program
    gpu.lightmappedProgram = bgfx::createProgram(
        bgfx::createEmbeddedShader(s_embeddedShaders, rendererType, "vs_lightmapped"),
        bgfx::createEmbeddedShader(s_embeddedShaders, rendererType, "fs_lightmapped"),
        true);

    // Water program: vertex displacement + textured fragment with UV distortion
    gpu.waterProgram = bgfx::createProgram(
        bgfx::createEmbeddedShader(s_embeddedShaders, rendererType, "vs_water"),
        bgfx::createEmbeddedShader(s_embeddedShaders, rendererType, "fs_water"),
        true);

    gpu.s_texColor = bgfx::createUniform("s_texColor", bgfx::UniformType::Sampler);
    gpu.s_texLightmap = bgfx::createUniform("s_texLightmap", bgfx::UniformType::Sampler);
    gpu.u_waterParams = bgfx::createUniform("u_waterParams", bgfx::UniformType::Vec4);
    gpu.u_waterFlow = bgfx::createUniform("u_waterFlow", bgfx::UniformType::Vec4);
    gpu.u_fogColor = bgfx::createUniform("u_fogColor", bgfx::UniformType::Vec4);
    gpu.u_fogParams = bgfx::createUniform("u_fogParams", bgfx::UniformType::Vec4);
    // Per-object params: x = alpha (1.0 = opaque, < 1.0 = translucent via RenderAlpha property)
    gpu.u_objectParams = bgfx::createUniform("u_objectParams", bgfx::UniformType::Vec4);

    // ── Build lightmap atlas (if textured mode) ──

    if (mission.texturedMode) {
        gpu.lmAtlasSet = Darkness::buildLightmapAtlases(mission.wrData, lmScale);
        if (!gpu.lmAtlasSet.atlases.empty()) {
            meshes.lightmappedMode = true;

            // Initial blend pass: apply animated overlays at initial intensities.
            // buildLightmapAtlases() only wrote static lightmaps into the atlas.
            // We must blend in overlay contributions so lights at max brightness
            // (mode 4, the default) have correct initial appearance.
            if (!mission.animLightIndex.empty()) {
                // Compute initial intensities for all lights
                std::unordered_map<int16_t, float> initIntensities;
                for (const auto &[lightNum, light] : mission.lightSources) {
                    initIntensities[lightNum] = (light.maxBright > 0.0f)
                        ? light.brightness / light.maxBright : 0.0f;
                }

                // Collect unique (cell, poly) pairs — a polygon may appear under
                // multiple lightnums, but blendAnimatedLightmap re-blends all
                // overlays at once, so we only need to call it once per polygon.
                std::unordered_set<uint64_t> blendedSet;
                int blendedPolys = 0;
                for (const auto &[lightNum, polys] : mission.animLightIndex) {
                    for (const auto &[ci, pi] : polys) {
                        uint64_t key = (static_cast<uint64_t>(ci) << 32)
                                      | static_cast<uint32_t>(pi);
                        if (!blendedSet.insert(key).second) continue;

                        Darkness::blendAnimatedLightmap(
                            gpu.lmAtlasSet.atlases[0], mission.wrData, ci, pi,
                            gpu.lmAtlasSet.entries[ci][pi],
                            initIntensities, lmScale);
                        ++blendedPolys;
                    }
                }
                std::fprintf(stderr, "Initial lightmap blend: %d polygons\n",
                             blendedPolys);
            }

            for (const auto &atlas : gpu.lmAtlasSet.atlases) {
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
                gpu.lightmapAtlasHandles.push_back(th);
            }
            std::fprintf(stderr, "Created %zu lightmap atlas GPU texture(s)\n",
                         gpu.lightmapAtlasHandles.size());
        }
    }

    // ── Build geometry and create GPU buffers ──

    if (meshes.lightmappedMode) {
        meshes.lmMesh = buildLightmappedMesh(mission.wrData, mission.texDims, gpu.lmAtlasSet);
        centroidX = meshes.lmMesh.cx; centroidY = meshes.lmMesh.cy; centroidZ = meshes.lmMesh.cz;

        std::fprintf(stderr, "Geometry (lightmapped): %zu vertices, %zu indices (%zu triangles), %zu texture groups\n",
                     meshes.lmMesh.vertices.size(), meshes.lmMesh.indices.size(),
                     meshes.lmMesh.indices.size() / 3, meshes.lmMesh.groups.size());

        if (meshes.lmMesh.vertices.empty()) {
            std::fprintf(stderr, "No geometry to render\n");
            return false;
        }

        const bgfx::Memory *vbMem = bgfx::copy(
            meshes.lmMesh.vertices.data(),
            static_cast<uint32_t>(meshes.lmMesh.vertices.size() * sizeof(PosUV2Vertex))
        );
        gpu.vbh = bgfx::createVertexBuffer(vbMem, PosUV2Vertex::layout);

        const bgfx::Memory *ibMem = bgfx::copy(
            meshes.lmMesh.indices.data(),
            static_cast<uint32_t>(meshes.lmMesh.indices.size() * sizeof(uint32_t))
        );
        gpu.ibh = bgfx::createIndexBuffer(ibMem, BGFX_BUFFER_INDEX32);

        // Create bgfx textures with full mip chains from loaded images
        for (const auto &kv : mission.loadedTextures) {
            uint8_t idx = kv.first;
            const auto &img = kv.second;
            gpu.textureHandles[idx] = createMipmappedTexture(
                img.rgba.data(), img.width, img.height,
                BGFX_SAMPLER_MIN_POINT | BGFX_SAMPLER_MAG_POINT
                | BGFX_SAMPLER_U_MIRROR | BGFX_SAMPLER_V_MIRROR,
                MipAlphaMode::ALPHA_TEST, linearMips, sharpMips);
        }
        std::fprintf(stderr, "Created %zu GPU textures (mipmapped)\n", gpu.textureHandles.size());

        // Create GPU textures for flow group water textures (loaded by name)
        // Water textures are fully opaque (alpha blending via vertex color),
        // so use ALPHA_BLEND (no coverage preservation needed).
        for (const auto &kv : mission.flowLoadedTextures) {
            uint8_t fg = kv.first;
            const auto &img = kv.second;
            gpu.flowTextureHandles[fg] = createMipmappedTexture(
                img.rgba.data(), img.width, img.height,
                BGFX_SAMPLER_U_MIRROR | BGFX_SAMPLER_V_MIRROR,
                MipAlphaMode::ALPHA_BLEND, linearMips, sharpMips);
        }
        if (!gpu.flowTextureHandles.empty()) {
            std::fprintf(stderr, "Created %zu flow water GPU textures\n", gpu.flowTextureHandles.size());
        }
    } else if (mission.texturedMode) {
        meshes.worldMesh = buildTexturedMesh(mission.wrData, mission.texDims);
        centroidX = meshes.worldMesh.cx; centroidY = meshes.worldMesh.cy; centroidZ = meshes.worldMesh.cz;

        std::fprintf(stderr, "Geometry (textured): %zu vertices, %zu indices (%zu triangles), %zu texture groups\n",
                     meshes.worldMesh.vertices.size(), meshes.worldMesh.indices.size(),
                     meshes.worldMesh.indices.size() / 3, meshes.worldMesh.groups.size());

        if (meshes.worldMesh.vertices.empty()) {
            std::fprintf(stderr, "No geometry to render\n");
            return false;
        }

        const bgfx::Memory *vbMem = bgfx::copy(
            meshes.worldMesh.vertices.data(),
            static_cast<uint32_t>(meshes.worldMesh.vertices.size() * sizeof(PosColorUVVertex))
        );
        gpu.vbh = bgfx::createVertexBuffer(vbMem, PosColorUVVertex::layout);

        const bgfx::Memory *ibMem = bgfx::copy(
            meshes.worldMesh.indices.data(),
            static_cast<uint32_t>(meshes.worldMesh.indices.size() * sizeof(uint32_t))
        );
        gpu.ibh = bgfx::createIndexBuffer(ibMem, BGFX_BUFFER_INDEX32);

        for (const auto &kv : mission.loadedTextures) {
            uint8_t idx = kv.first;
            const auto &img = kv.second;
            gpu.textureHandles[idx] = createMipmappedTexture(
                img.rgba.data(), img.width, img.height,
                BGFX_SAMPLER_MIN_POINT | BGFX_SAMPLER_MAG_POINT
                | BGFX_SAMPLER_U_MIRROR | BGFX_SAMPLER_V_MIRROR,
                MipAlphaMode::ALPHA_TEST, linearMips, sharpMips);
        }
        std::fprintf(stderr, "Created %zu GPU textures (mipmapped)\n", gpu.textureHandles.size());
    } else {
        meshes.flatMesh = buildFlatMesh(mission.wrData);
        centroidX = meshes.flatMesh.cx; centroidY = meshes.flatMesh.cy; centroidZ = meshes.flatMesh.cz;

        std::fprintf(stderr, "Geometry (flat): %zu vertices, %zu indices (%zu triangles)\n",
                     meshes.flatMesh.vertices.size(), meshes.flatMesh.indices.size(),
                     meshes.flatMesh.indices.size() / 3);

        if (meshes.flatMesh.vertices.empty()) {
            std::fprintf(stderr, "No geometry to render\n");
            return false;
        }

        const bgfx::Memory *vbMem = bgfx::copy(
            meshes.flatMesh.vertices.data(),
            static_cast<uint32_t>(meshes.flatMesh.vertices.size() * sizeof(PosColorVertex))
        );
        gpu.vbh = bgfx::createVertexBuffer(vbMem, PosColorVertex::layout);

        const bgfx::Memory *ibMem = bgfx::copy(
            meshes.flatMesh.indices.data(),
            static_cast<uint32_t>(meshes.flatMesh.indices.size() * sizeof(uint32_t))
        );
        gpu.ibh = bgfx::createIndexBuffer(ibMem, BGFX_BUFFER_INDEX32);
    }

    // ── Build water surface mesh from portal polygons ──
    meshes.waterMesh = buildWaterMesh(mission.wrData, mission.texDims, mission.flowData, mission.flowTexDims);
    meshes.hasWater = !meshes.waterMesh.vertices.empty();

    if (meshes.hasWater) {
        gpu.waterVBH = bgfx::createVertexBuffer(
            bgfx::copy(meshes.waterMesh.vertices.data(),
                        static_cast<uint32_t>(meshes.waterMesh.vertices.size() * sizeof(PosColorUVVertex))),
            PosColorUVVertex::layout);
        gpu.waterIBH = bgfx::createIndexBuffer(
            bgfx::copy(meshes.waterMesh.indices.data(),
                        static_cast<uint32_t>(meshes.waterMesh.indices.size() * sizeof(uint32_t))),
            BGFX_BUFFER_INDEX32);
    }

    // ── Create GPU buffers for object meshes ──

    if (showObjects) {
        // Build fallback cube
        {
            std::vector<PosColorVertex> cubeVerts;
            std::vector<uint32_t> cubeIndices;
            buildFallbackCube(cubeVerts, cubeIndices, 0xff808080u);
            gpu.fallbackCubeIndexCount = static_cast<uint32_t>(cubeIndices.size());
            gpu.fallbackCubeVBH = bgfx::createVertexBuffer(
                bgfx::copy(cubeVerts.data(),
                    static_cast<uint32_t>(cubeVerts.size() * sizeof(PosColorVertex))),
                PosColorVertex::layout);
            gpu.fallbackCubeIBH = bgfx::createIndexBuffer(
                bgfx::copy(cubeIndices.data(),
                    static_cast<uint32_t>(cubeIndices.size() * sizeof(uint32_t))),
                BGFX_BUFFER_INDEX32);
        }

        // Create bgfx texture handles with mip chains from loaded object texture images
        for (const auto &kv : mission.objTexImages) {
            const auto &img = kv.second;
            gpu.objTextureHandles[kv.first] = createMipmappedTexture(
                img.rgba.data(), img.width, img.height,
                BGFX_SAMPLER_MIN_POINT | BGFX_SAMPLER_MAG_POINT
                | BGFX_SAMPLER_U_MIRROR | BGFX_SAMPLER_V_MIRROR,
                MipAlphaMode::ALPHA_TEST, linearMips, sharpMips);
        }
        if (!gpu.objTextureHandles.empty()) {
            std::fprintf(stderr, "Created %zu object texture GPU handles\n",
                         gpu.objTextureHandles.size());
        }

        // Create GPU buffers for each parsed .bin model
        for (const auto &kv : mission.parsedModels) {
            const std::string &name = kv.first;
            const Darkness::ParsedBinMesh &mesh = kv.second;

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

            // Fallback colour for non-textured models or Darkness::MD_MAT_COLOR materials
            uint32_t fallbackColor = colorFromName(name);

            // Build set of material indices that have actually-loaded textures,
            // so we only assign white to vertices that will be textured at draw time.
            std::unordered_set<int> loadedMatIndices;
            for (int mi = 0; mi < static_cast<int>(mesh.materials.size()); ++mi) {
                if (mesh.materials[mi].type == Darkness::MD_MAT_TMAP) {
                    std::string lname(mesh.materials[mi].name);
                    std::transform(lname.begin(), lname.end(), lname.begin(),
                                   [](unsigned char c) { return std::tolower(c); });
                    if (mission.objTexImages.count(lname)) {
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
                           mesh.materials[mi].type == Darkness::MD_MAT_COLOR) {
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

            ObjectModelGPU objGPUEntry;
            objGPUEntry.vbh = bgfx::createVertexBuffer(
                bgfx::copy(gpuVerts.data(),
                    static_cast<uint32_t>(gpuVerts.size() * sizeof(PosColorUVVertex))),
                PosColorUVVertex::layout);
            objGPUEntry.ibh = bgfx::createIndexBuffer(
                bgfx::copy(mesh.indices.data(),
                    static_cast<uint32_t>(mesh.indices.size() * sizeof(uint32_t))),
                BGFX_BUFFER_INDEX32);

            // Build per-submesh GPU draw info from parsed submeshes
            for (const auto &sm : mesh.subMeshes) {
                ObjectSubMeshGPU gsm;
                gsm.firstIndex = sm.firstIndex;
                gsm.indexCount = sm.indexCount;
                gsm.textured = false;
                gsm.matTrans = 0.0f;
                if (sm.matIndex >= 0 && sm.matIndex < static_cast<int>(mesh.materials.size())) {
                    const auto &mat = mesh.materials[sm.matIndex];
                    gsm.textured = (mat.type == Darkness::MD_MAT_TMAP);
                    gsm.matTrans = mat.trans;  // per-material translucency
                    // Lowercase material name for texture lookup
                    gsm.matName = mat.name;
                    std::transform(gsm.matName.begin(), gsm.matName.end(),
                                   gsm.matName.begin(),
                                   [](unsigned char c) { return std::tolower(c); });
                }
                objGPUEntry.subMeshes.push_back(gsm);
            }

            objGPUEntry.valid = true;
            gpu.objModelGPU[name] = std::move(objGPUEntry);
        }
        std::fprintf(stderr, "Object GPU buffers: %zu models + fallback cube\n",
                     gpu.objModelGPU.size());

        // Count translucent objects/materials for diagnostics
        int translucentObjCount = 0;
        int translucentMatCount = 0;
        for (const auto &obj : mission.objData.objects) {
            if (obj.renderAlpha < 1.0f) ++translucentObjCount;
        }
        for (const auto &kv : gpu.objModelGPU) {
            if (!kv.second.valid) continue;
            for (const auto &sm : kv.second.subMeshes) {
                if (sm.matTrans > 0.0f) ++translucentMatCount;
            }
        }
        if (translucentObjCount > 0 || translucentMatCount > 0) {
            std::fprintf(stderr, "Translucent: %d objects (RenderAlpha), "
                         "%d material submeshes (mat_extra)\n",
                         translucentObjCount, translucentMatCount);
        }
    }

    // ── Create sky dome GPU buffers ──

    if (!mission.skyDome.vertices.empty()) {
        gpu.skyVBH = bgfx::createVertexBuffer(
            bgfx::copy(mission.skyDome.vertices.data(),
                static_cast<uint32_t>(mission.skyDome.vertices.size() * sizeof(PosColorVertex))),
            PosColorVertex::layout);
        gpu.skyIBH = bgfx::createIndexBuffer(
            bgfx::copy(mission.skyDome.indices.data(),
                static_cast<uint32_t>(mission.skyDome.indices.size() * sizeof(uint16_t))));
        gpu.skyIndexCount = static_cast<uint32_t>(mission.skyDome.indices.size());
    }

    // ── Create textured skybox GPU buffers (old sky system) ──

    if (mission.hasSkybox) {
        meshes.skyboxCube = buildSkyboxCube();

        gpu.skyboxVBH = bgfx::createVertexBuffer(
            bgfx::copy(meshes.skyboxCube.vertices.data(),
                static_cast<uint32_t>(meshes.skyboxCube.vertices.size() * sizeof(PosColorUVVertex))),
            PosColorUVVertex::layout);
        gpu.skyboxIBH = bgfx::createIndexBuffer(
            bgfx::copy(meshes.skyboxCube.indices.data(),
                static_cast<uint32_t>(meshes.skyboxCube.indices.size() * sizeof(uint16_t))));

        // Create GPU textures with mip chains for each loaded skybox face
        // Skybox faces are fully opaque so use ALPHA_BLEND (no coverage preservation)
        for (const auto &kv : mission.skyboxImages) {
            const auto &img = kv.second;
            gpu.skyboxTexHandles[kv.first] = createMipmappedTexture(
                img.rgba.data(), img.width, img.height,
                BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP,
                MipAlphaMode::ALPHA_BLEND, linearMips, sharpMips);
        }
        std::fprintf(stderr, "Skybox GPU: %zu face textures created\n",
                     gpu.skyboxTexHandles.size());
    }

    return true;
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

    // Unpack into local variables — rest of file uses these unchanged
    const char *misPath    = cli.misPath;
    std::string resPath    = cli.resPath;
    int  lmScale           = cfg.lmScale;
    state.showObjects       = cfg.showObjects;
    state.showFallbackCubes = cfg.showFallbackCubes;
    bool forceFlicker      = cfg.forceFlicker;
    state.portalCulling     = cfg.portalCulling;
    state.cameraCollision   = cfg.cameraCollision;
    state.filterMode        = cfg.filterMode;
    bool linearMips        = cfg.linearMips;
    bool sharpMips         = cfg.sharpMips;
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
    if (!loadMissionData(misPath, forceFlicker, mission))
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
    loadObjectAssets(misPath, resPath, cfg, mission.wrData, mission, state);

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

    const char *modeStr = meshes.lightmappedMode ? "lightmapped" :
                          mission.texturedMode ? "textured" : "flat-shaded";
    std::fprintf(stderr, "Render window opened (%dx%d, %s, %s)\n",
                 WINDOW_WIDTH, WINDOW_HEIGHT,
                 bgfx::getRendererName(bgfx::getRendererType()), modeStr);
    std::fprintf(stderr, "Portal culling: %s (toggle with C key)\n",
                 state.portalCulling ? "ON" : "OFF");

    // ── Model isolation state for debugging ──
    // Sorted list of model names that have loaded GPU data, with instance counts.
    // Press N to cycle through, isolating one model at a time for identification.
    if (state.showObjects) {
        // Count instances per model name
        for (const auto &obj : mission.objData.objects) {
            if (!obj.hasPosition) continue;
            std::string mn(obj.modelName);
            state.modelInstanceCounts[mn]++;
        }

        // Build sorted list of models that have GPU data (loaded successfully)
        for (const auto &kv : gpu.objModelGPU) {
            if (kv.second.valid)
                state.sortedModelNames.push_back(kv.first);
        }
        std::sort(state.sortedModelNames.begin(), state.sortedModelNames.end());

        // Count unloaded models (fallback cube candidates)
        int unloadedCount = 0;
        for (const auto &kv : state.modelInstanceCounts) {
            if (gpu.objModelGPU.find(kv.first) == gpu.objModelGPU.end() || !gpu.objModelGPU[kv.first].valid)
                ++unloadedCount;
        }
        std::fprintf(stderr, "Model isolation: %zu loaded, %d unloaded (M=next, N=prev)\n",
                     state.sortedModelNames.size(), unloadedCount);
    }

    // Use spawn point if found, otherwise fall back to centroid
    state.spawnX = camX; state.spawnY = camY; state.spawnZ = camZ;
    state.spawnYaw = 0.0f;
    if (mission.spawnInfo.found) {
        state.spawnX = mission.spawnInfo.x;
        state.spawnY = mission.spawnInfo.y;
        state.spawnZ = mission.spawnInfo.z;
        state.spawnYaw = mission.spawnInfo.yaw;
        std::fprintf(stderr, "Camera at spawn (%.1f, %.1f, %.1f)\n",
                     state.spawnX, state.spawnY, state.spawnZ);
    } else {
        std::fprintf(stderr, "Camera at centroid (%.1f, %.1f, %.1f)\n",
                     camX, camY, camZ);
    }

    std::fprintf(stderr, "Controls: WASD=move, mouse=look, Space/LShift=up/down, "
                 "scroll=speed, `=console, Home=spawn, BS+C/F/V/M/N/R=debug, Esc=quit\n");

    // state.cam initialized below
    state.cam.init(state.spawnX, state.spawnY, state.spawnZ);
    state.cam.yaw = state.spawnYaw;

    SDL_SetRelativeMouseMode(SDL_TRUE);

    // state.moveSpeed has default 20.0f from struct initializer

    // Portal culling stats for title bar display
    state.cullTotalCells = mission.wrData.numCells;

    // Helper to update window title with current speed, culling, and filter stats
    auto updateTitle = [&]() {
        char title[512];
        const char *filterNames[] = { "point", "bilinear", "trilinear", "aniso" };
        const char *filterStr = filterNames[state.filterMode % 4];

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
                "darkness — %s [speed: %.1f] [cull: %u/%u cells] [%s] [%s]%s",
                modeStr, state.moveSpeed, state.cullVisibleCells, state.cullTotalCells, filterStr, clipStr, isoSuffix);
        } else {
            std::snprintf(title, sizeof(title),
                "darkness — %s [speed: %.1f] [cull: OFF] [%s] [%s]%s",
                modeStr, state.moveSpeed, filterStr, clipStr, isoSuffix);
        }
        SDL_SetWindowTitle(window, title);
    };
    updateTitle();

    // ── Debug console for runtime settings management ──
    // Opened with backtick (`), provides tab-completion and value editing.
    Darkness::DebugConsole dbgConsole;

    dbgConsole.addCategorical("filter_mode",
        {"point", "bilinear", "trilinear", "anisotropic"},
        [&]() { return state.filterMode; },
        [&](int v) { state.filterMode = v; updateTitle(); });

    dbgConsole.addBool("portal_culling",
        [&]() { return state.portalCulling; },
        [&](bool v) { state.portalCulling = v; updateTitle(); });

    dbgConsole.addBool("camera_collision",
        [&]() { return state.cameraCollision; },
        [&](bool v) { state.cameraCollision = v; updateTitle(); });

    dbgConsole.addBool("show_objects",
        [&]() { return state.showObjects; },
        [&](bool v) { state.showObjects = v; });

    dbgConsole.addBool("show_fallback_cubes",
        [&]() { return state.showFallbackCubes; },
        [&](bool v) { state.showFallbackCubes = v; });

    dbgConsole.addBool("show_raycast",
        [&]() { return state.showRaycast; },
        [&](bool v) { state.showRaycast = v; });

    dbgConsole.addFloat("move_speed", 1.0f, 500.0f,
        [&]() { return state.moveSpeed; },
        [&](float v) { state.moveSpeed = v; updateTitle(); });

    dbgConsole.addFloat("wave_amplitude", 0.0f, 5.0f,
        [&]() { return state.waveAmplitude; },
        [&](float v) { state.waveAmplitude = v; });

    dbgConsole.addFloat("uv_distortion", 0.0f, 0.5f,
        [&]() { return state.uvDistortion; },
        [&](float v) { state.uvDistortion = v; });

    dbgConsole.addFloat("water_rotation", 0.0f, 0.5f,
        [&]() { return state.waterRotation; },
        [&](float v) { state.waterRotation = v; });

    dbgConsole.addFloat("water_scroll", 0.0f, 1.0f,
        [&]() { return state.waterScrollSpeed; },
        [&](float v) { state.waterScrollSpeed = v; });

    auto lastTime = std::chrono::high_resolution_clock::now();
    // state.waterElapsed, state.running have defaults from struct initializer
    while (state.running) {
        auto now = std::chrono::high_resolution_clock::now();
        float dt = std::chrono::duration<float>(now - lastTime).count();
        lastTime = now;
        dt = std::min(dt, 0.1f);
        state.waterElapsed += dt;

        handleEvents(state.running, state.cam, state.moveSpeed,
                     state.portalCulling, state.filterMode,
                     state.isolateModelIdx, state.cameraCollision, state.showRaycast,
                     state.spawnX, state.spawnY, state.spawnZ, state.spawnYaw,
                     state.sortedModelNames, state.modelInstanceCounts,
                     dbgConsole, updateTitle);

        updateMovement(dt, state.cam, state.moveSpeed, state.cameraCollision, mission.wrData, dbgConsole);

        updateLightmaps(dt, meshes.lightmappedMode,
                        mission.lightSources, mission.animLightIndex,
                        gpu.lmAtlasSet, gpu.lightmapAtlasHandles,
                        mission.wrData, lmScale);

        // ── Prepare frame: matrices, fog, samplers, culling ──
        auto fc = prepareFrame(state.cam, state.filterMode,
                               state.portalCulling, state.skyClearColor,
                               mission.wrData, mission.cellPortals,
                               mission.fogParams, mission.skyParams,
                               state.cullVisibleCells);
        updateTitle();

        // ── View 0: Sky pass ──
        renderSky(state.cam, fc.proj, mission.hasSkybox,
                  gpu.skyboxVBH, gpu.skyboxIBH, meshes.skyboxCube, gpu.skyboxTexHandles,
                  gpu.skyVBH, gpu.skyIBH, gpu.flatProgram, gpu.texturedProgram,
                  gpu.s_texColor, gpu.u_fogColor, gpu.u_fogParams, gpu.u_objectParams,
                  fc.fogColorArr, fc.skyFogArr, fc.skySampler);

        // ── View 1: World geometry ──
        renderWorld(meshes.lightmappedMode, mission.texturedMode,
                    state.portalCulling, fc.visibleCells,
                    meshes.lmMesh, meshes.worldMesh, meshes.flatMesh,
                    gpu.vbh, gpu.ibh, gpu.flatProgram, gpu.texturedProgram, gpu.lightmappedProgram,
                    gpu.s_texColor, gpu.s_texLightmap,
                    gpu.u_fogColor, gpu.u_fogParams, gpu.u_objectParams,
                    fc.fogColorArr, fc.fogOnArr,
                    gpu.textureHandles, gpu.lightmapAtlasHandles,
                    fc.texSampler, fc.renderState);

        // ── Object meshes ──
        renderObjects(state.showObjects, state.showFallbackCubes,
                      state.portalCulling, fc.visibleCells,
                      state.isolateModelIdx, state.sortedModelNames,
                      mission.objData, mission.objCellIDs,
                      gpu.objModelGPU, gpu.objTextureHandles,
                      gpu.fallbackCubeVBH, gpu.fallbackCubeIBH,
                      gpu.flatProgram, gpu.texturedProgram,
                      gpu.s_texColor, gpu.u_fogColor, gpu.u_fogParams, gpu.u_objectParams,
                      fc.fogColorArr, fc.fogOnArr,
                      fc.texSampler, fc.renderState);

        // ── Water surfaces ──
        renderWater(meshes.hasWater, meshes.waterMesh, gpu.waterVBH, gpu.waterIBH,
                    gpu.flatProgram, gpu.waterProgram,
                    gpu.s_texColor, gpu.u_fogColor, gpu.u_fogParams, gpu.u_objectParams,
                    gpu.u_waterParams, gpu.u_waterFlow,
                    fc.fogColorArr, fc.fogOnArr,
                    gpu.textureHandles, gpu.flowTextureHandles, fc.texSampler,
                    state.waterElapsed, state.waterScrollSpeed,
                    state.waveAmplitude, state.uvDistortion, state.waterRotation);

        // ── Debug raycast visualization (view 2) ──
        renderDebugOverlay(state.showRaycast, state.cam, fc.view, fc.proj, mission.wrData, mission.txList,
                           gpu.flatProgram, gpu.u_fogColor, gpu.u_fogParams, gpu.u_objectParams,
                           fc.fogColorArr, fc.fogOnArr);

        // Debug console overlay (no-op when closed)
        dbgConsole.render();

        bgfx::frame();
    }

    destroyGPUResources(
        gpu.vbh, gpu.ibh, gpu.flatProgram, gpu.texturedProgram, gpu.lightmappedProgram, gpu.waterProgram,
        gpu.s_texColor, gpu.s_texLightmap, gpu.u_waterParams, gpu.u_waterFlow,
        gpu.u_fogColor, gpu.u_fogParams, gpu.u_objectParams,
        gpu.textureHandles, gpu.flowTextureHandles, gpu.lightmapAtlasHandles,
        gpu.waterVBH, gpu.waterIBH, gpu.skyboxVBH, gpu.skyboxIBH, gpu.skyboxTexHandles,
        gpu.skyVBH, gpu.skyIBH, gpu.objModelGPU, gpu.objTextureHandles,
        gpu.fallbackCubeVBH, gpu.fallbackCubeIBH);
    shutdownWindow(window);

    std::fprintf(stderr, "Clean shutdown.\n");
    return 0;
}
