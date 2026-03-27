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
#include <unistd.h> // dup2, fileno

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
#include "audio/AudioService.h"
#include "audio/AcousticMaterials.h"
#include "motion/MotionService.h"
#include "RawDataStorage.h"
#include "PLDefParser.h"
#include "DTypeSizeParser.h"
#include "SingleFieldDataStorage.h"
#include "worldquery/ObjSysWorldState.h"
#include "sim/DoorSystem.h"
#include "FrobSystem.h"
#include "FunctionalLoopClient.h"

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
        "  --physics-rate <hz>\n"
        "                 Physics timestep: 12 (vintage), 60 (modern/default), 120 (ultra).\n"
        "  --show-fallback Show colored cubes for objects with missing models.\n"
        "  --wave-amp <f> Water wave amplitude, 0.0-10.0 (default: 0.3).\n"
        "  --uv-distort <f> Water UV distortion, 0.0-0.1 (default: 0.015).\n"
        "  --water-rot <f> Water UV rotation speed in rad/s, 0.0-1.0 (default: 0.015).\n"
        "  --water-scroll <f> Water UV scroll speed, 0.0-1.0 (default: 0.05).\n"
        "  --step-log     Log stair step diagnostics to stderr ([STEP] prefix).\n"
        "  --debug-objects Dump per-object filtering diagnostics to stderr.\n"
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
        "Debug console (` backtick to open):\n"
        "  portal_culling     Toggle portal culling on/off\n"
        "  filter_mode        Cycle texture filtering (point/bilinear/trilinear/aniso)\n"
        "  lightmap_filtering Toggle lightmap filtering (bilinear/bicubic)\n"
        "  camera_collision   Toggle camera collision (clip/noclip) [fly mode only]\n"
        "  physics_mode       Toggle physics mode (walk with gravity/fly noclip)\n"
        "  isolate_model      Cycle model isolation\n"
        "  physics_log        Toggle physics diagnostic log (physics_log.csv)\n"
        "  show_raycast       Toggle raycast debug visualization\n"
        "  refl_enabled       Toggle audio reflections (convolution reverb)\n"
        "\n"
        "Physics mode controls (when physics_mode is on):\n"
        "  WASD           Walk forward/strafe\n"
        "  Space           Jump (when on ground)\n"
        "  C               Crouch (toggle)\n"
        "  LShift          Sneak/creep (hold, 0.5x speed)\n"
        "  Q/E             Lean left/right\n"
        "  Ctrl            Sprint/run (2x speed)\n"
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
// Parameters are struct references defined in DarknessRenderState.h.

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
            // with rotation.
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
    if (!state.showRaycast && !state.showAcousticMesh) return;

    // Set up view 2 with same transform as view 1
    bgfx::setViewTransform(2, fc.view, fc.proj);

    // ── Raycast debug lines ──
    if (state.showRaycast) {
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
    // Acoustic mesh wireframe overlay
    if (state.showAcousticMesh && bgfx::isValid(state.acousticVBH)) {
        float identity[16];
        bx::mtxIdentity(identity);
        bgfx::setTransform(identity);
        bgfx::setVertexBuffer(0, state.acousticVBH, 0, state.acousticLineCount);
        uint64_t wireState = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
                           | BGFX_STATE_PT_LINES
                           | BGFX_STATE_BLEND_ALPHA
                           | BGFX_STATE_DEPTH_TEST_LESS;
        bgfx::setState(wireState);
        float noFog[4] = {0, 0, 0, 0};
        bgfx::setUniform(gpu.u_fogColor, noFog);
        bgfx::setUniform(gpu.u_fogParams, noFog);
        float opaqueParams[4] = {1.0f, 0.0f, 0.0f, 0.0f};
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

    bgfx::dbgTextPrintf(2, 1, hud_attr, "RAYCAST DEBUG (` > show_raycast to toggle)");

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
    } // end showRaycast
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

        // Portal culling: skip objects in non-visible cells, unless the
        // object's world-space AABB still intersects the rendering frustum.
        // This prevents large objects from popping when their center-of-mass
        // cell passes behind the camera but part of the mesh is still visible.
        if (state.portalCulling && oi < mission.objCellIDs.size()) {
            int32_t objCell = mission.objCellIDs[oi];
            if (objCell >= 0 && fc.visibleCells.count(static_cast<uint32_t>(objCell)) == 0) {
                // Cell not visible — check if model AABB intersects frustum
                std::string mname(obj.modelName);
                auto mit = mission.parsedModels.find(mname);
                if (mit == mission.parsedModels.end()) return;
                const auto &mesh = mit->second;

                // Compute conservative world-space AABB from model bbox + position.
                // Ignores rotation for speed — the resulting box is larger than
                // the true oriented bbox, so we never cull something visible.
                // Use runtime position if available (moving doors/platforms).
                float cx = obj.x, cy = obj.y, cz = obj.z;
                float csx = obj.scaleX, csy = obj.scaleY, csz = obj.scaleZ;
                const Darkness::ObjectState *cullState = state.objectStates
                    ? state.objectStates->tryGet(obj.objID) : nullptr;
                if (cullState) {
                    cx = cullState->position.x; cy = cullState->position.y; cz = cullState->position.z;
                    csx = cullState->scale.x; csy = cullState->scale.y; csz = cullState->scale.z;
                }
                float halfX = (mesh.bboxMax[0] - mesh.bboxMin[0]) * 0.5f * csx;
                float halfY = (mesh.bboxMax[1] - mesh.bboxMin[1]) * 0.5f * csy;
                float halfZ = (mesh.bboxMax[2] - mesh.bboxMin[2]) * 0.5f * csz;
                float extent = std::max({halfX, halfY, halfZ});  // sphere-ish bound
                if (!fc.objFrustum.testAABB(
                        cx - extent, cy - extent, cz - extent,
                        cx + extent, cy + extent, cz + extent))
                    return;
            }
        }

        // Compute per-object model matrix. If the object has a runtime state
        // override (door opening, platform moving, tweq animating), use that
        // transform instead of the static P$Position from the .mis file.
        float objMtx[16];
        const Darkness::ObjectState *objState = state.objectStates
            ? state.objectStates->tryGet(obj.objID) : nullptr;
        if (objState && !(objState->flags & Darkness::kObjStateDestroyed)) {
            if (objState->flags & Darkness::kObjStateHidden)
                return;  // skip hidden objects (blink tweq, etc.)
            // Build matrix from runtime state angles (already in radians)
            const float negH = -objState->heading;
            const float negP = -objState->pitch;
            const float negB = -objState->bank;
            bx::mtxRotateXYZ(objMtx, negB, negP, negH);
            objMtx[ 0] *= objState->scale.x; objMtx[ 1] *= objState->scale.x; objMtx[ 2] *= objState->scale.x;
            objMtx[ 4] *= objState->scale.y; objMtx[ 5] *= objState->scale.y; objMtx[ 6] *= objState->scale.y;
            objMtx[ 8] *= objState->scale.z; objMtx[ 9] *= objState->scale.z; objMtx[10] *= objState->scale.z;
            objMtx[12] = objState->position.x;
            objMtx[13] = objState->position.y;
            objMtx[14] = objState->position.z;
        } else {
            buildModelMatrix(objMtx, obj.x, obj.y, obj.z,
                             obj.heading, obj.pitch, obj.bank,
                             obj.scaleX, obj.scaleY, obj.scaleZ);
        }

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

    // Physics mode shows "walk" / camera collision shows "clip" / default "noclip"
    const char *moveStr = state.physicsMode ? "walk" :
                          state.cameraCollision ? "clip" : "noclip";

    // In physics mode, show velocity and ground state for debugging
    char physSuffix[128] = "";
    if (state.physicsMode && state.physics) {
        Darkness::Vector3 vel = state.physics->getPlayerVelocity();
        float speed = std::sqrt(vel.x * vel.x + vel.y * vel.y);
        const char *groundStr = state.physics->isPlayerOnGround() ? "ground" : "air";
        // Show player mode + speed modifier
        const auto &pp = state.physics->getPlayerPhysics();
        static const char *modeNames[] = {
            "stand", "crouch", "swim", "climb", "carry", "slide", "jump", "dead"
        };
        int modeIdx = static_cast<int>(pp.getMode());
        const char *modeName = (modeIdx >= 0 && modeIdx < 8) ? modeNames[modeIdx] : "?";
        const char *speedMode = pp.isSneaking() ? "sneak" :
                                pp.isRunning()  ? "run" : modeName;
        std::snprintf(physSuffix, sizeof(physSuffix),
            " [%s %s %.1f u/s cell:%d %.0fHz]", groundStr, speedMode, speed,
            state.physics->getPlayerCell(), state.physics->getPhysicsHz());
    }

    if (state.portalCulling) {
        std::snprintf(title, sizeof(title),
            "darkness — %s [speed: %.1f] [cull: %u/%u cells] [%s] [lm:%s] [%s]%s%s",
            state.modeStr, state.moveSpeed, state.cullVisibleCells, state.cullTotalCells, filterStr, lmStr, moveStr, isoSuffix, physSuffix);
    } else {
        std::snprintf(title, sizeof(title),
            "darkness — %s [speed: %.1f] [cull: OFF] [%s] [lm:%s] [%s]%s%s",
            state.modeStr, state.moveSpeed, filterStr, lmStr, moveStr, isoSuffix, physSuffix);
    }
    SDL_SetWindowTitle(window, title);
}

// ── Register debug console settings ──
// Binds 15 runtime-changeable settings to the debug console (opened with backtick).
// Lambdas capture state by reference and update the title bar when relevant settings change.
static void registerConsoleSettings(
    Darkness::DebugConsole &dbgConsole,
    Darkness::RuntimeState &state,
    SDL_Window *window,
    const std::string &misPath = "")
{
    // Helper lambda that captures window+state for title bar refresh
    auto refreshTitle = [window, &state]() { updateTitleBar(window, state); };

    // ── Rendering ──

    dbgConsole.setGroup("Rendering");

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
        [&state, refreshTitle](bool v) { state.portalCulling = v; refreshTitle(); },
        "BFS portal traversal culling (reduces draw calls)");

    dbgConsole.addBool("show_objects",
        [&state]() { return state.showObjects; },
        [&state](bool v) { state.showObjects = v; },
        "Render object meshes (.bin models from obj.crf)");

    dbgConsole.addBool("show_fallback_cubes",
        [&state]() { return state.showFallbackCubes; },
        [&state](bool v) { state.showFallbackCubes = v; },
        "Show colored cubes for objects with missing models");

    dbgConsole.addBool("show_raycast",
        [&state]() { return state.showRaycast; },
        [&state](bool v) { state.showRaycast = v; },
        "Debug ray visualization from camera center");

    dbgConsole.addBool("show_acoustic_mesh",
        [&state]() { return state.showAcousticMesh; },
        [&state](bool v) { state.showAcousticMesh = v; },
        "Cyan wireframe overlay of the acoustic scene geometry");

    // Model isolation: "all" = show everything, then one entry per loaded model name.
    // sortedModelNames is populated during init before this registration runs.
    {
        std::vector<std::string> modelOpts = {"all"};
        modelOpts.insert(modelOpts.end(),
                         state.sortedModelNames.begin(),
                         state.sortedModelNames.end());
        dbgConsole.addCategorical("isolate_model", modelOpts,
            [&state]() { return state.isolateModelIdx + 1; },  // -1 → 0 ("all")
            [&state, window](int v) {
                state.isolateModelIdx = v - 1;  // 0 → -1 ("all")
                if (state.isolateModelIdx >= 0 &&
                    state.isolateModelIdx < static_cast<int>(state.sortedModelNames.size())) {
                    const auto &isoName = state.sortedModelNames[state.isolateModelIdx];
                    auto cit = state.modelInstanceCounts.find(isoName);
                    int cnt = (cit != state.modelInstanceCounts.end()) ? cit->second : 0;
                    std::fprintf(stderr, "Isolating model [%d/%zu]: '%s' (%d instances)\n",
                                 state.isolateModelIdx + 1, state.sortedModelNames.size(),
                                 isoName.c_str(), cnt);
                } else {
                    std::fprintf(stderr, "Model isolation: OFF (showing all)\n");
                }
                updateTitleBar(window, state);
            });
    }

    // ── Movement ──

    dbgConsole.setGroup("Movement");

    dbgConsole.addBool("physics_mode",
        [&state]() { return state.physicsMode; },
        [&state, refreshTitle](bool v) {
            state.physicsMode = v;
            state.crouchToggled = false;
            if (v && state.physics) {
                Darkness::Vector3 bodyPos(state.cam.pos[0], state.cam.pos[1], state.cam.pos[2]);
                state.physics->setPlayerPosition(bodyPos);
                state.physics->setPlayerYaw(state.cam.yaw);
            }
            refreshTitle();
        },
        "Walk mode (on) vs fly/noclip mode (off)");

    dbgConsole.addCategorical("physics_rate",
        {"12.5Hz (vintage)", "60Hz (modern)", "120Hz (ultra)"},
        [&state]() {
            if (!state.physics) return 1;
            float hz = state.physics->getPhysicsHz();
            if (hz < 20.0f) return 0;       // vintage
            if (hz > 100.0f) return 2;       // ultra
            return 1;                         // modern
        },
        [&state, refreshTitle](int v) {
            if (!state.physics) return;
            auto &pp = state.physics->getPlayerPhysics();
            if (v == 0) pp.setTimestep(Darkness::VINTAGE);
            else if (v == 2) pp.setTimestep(Darkness::ULTRA);
            else pp.setTimestep(Darkness::MODERN);
            refreshTitle();
        });

    dbgConsole.addBool("camera_collision",
        [&state]() { return state.cameraCollision; },
        [&state, refreshTitle](bool v) { state.cameraCollision = v; refreshTitle(); },
        "Sphere collision against world geometry (noclip when off)");

    dbgConsole.addFloat("move_speed", 1.0f, 500.0f,
        [&state]() { return state.moveSpeed; },
        [&state, refreshTitle](float v) { state.moveSpeed = v; refreshTitle(); },
        "Camera/fly movement speed (world units/sec)");

    dbgConsole.addBool("step_log",
        [&state]() { return state.physics ? state.physics->getPlayerPhysics().stepLogEnabled() : false; },
        [&state](bool v) { if (state.physics) state.physics->getPlayerPhysics().setStepLog(v); },
        "Log stair-step diagnostics to stderr ([STEP] prefix)");

    dbgConsole.addBool("physics_log",
        [&state]() { return state.physics ? state.physics->getPlayerPhysics().isLogging() : false; },
        [&state](bool v) {
            if (!state.physics) return;
            auto &player = state.physics->getPlayerPhysics();
            if (v) player.startLog("physics_log.csv");
            else   player.stopLog();
        },
        "Write per-timestep physics data to physics_log.csv");

    // ── Water ──

    dbgConsole.setGroup("Water");

    dbgConsole.addFloat("wave_amplitude", 0.0f, 5.0f,
        [&state]() { return state.waveAmplitude; },
        [&state](float v) { state.waveAmplitude = v; },
        "Water vertex wave displacement (0=flat)");

    dbgConsole.addFloat("uv_distortion", 0.0f, 0.5f,
        [&state]() { return state.uvDistortion; },
        [&state](float v) { state.uvDistortion = v; },
        "Water UV texture wobble strength");

    dbgConsole.addFloat("water_rotation", 0.0f, 0.5f,
        [&state]() { return state.waterRotation; },
        [&state](float v) { state.waterRotation = v; },
        "Water UV rotation speed (rad/s)");

    dbgConsole.addFloat("water_scroll", 0.0f, 1.0f,
        [&state]() { return state.waterScrollSpeed; },
        [&state](float v) { state.waterScrollSpeed = v; },
        "Water UV scroll speed (world units/s)");

    // ── Audio ──

    dbgConsole.setGroup("Audio");

    dbgConsole.addBool("portal_routing",
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? svc->getPortalRoutingEnabled() : true;
        },
        [](bool v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setPortalRoutingEnabled(v);
        },
        "Route sound through doorways via portal graph (indirect paths)");

    dbgConsole.addBool("probe_pathing",
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? svc->getProbePathingEnabled() : true;
        },
        [](bool v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setProbePathingEnabled(v);
        },
        "Baked probe diffraction/pathing (when probe data available)");

    dbgConsole.addBool("refl_enabled",
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? svc->getReflectionsEnabled() : false;
        },
        [](bool v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setReflectionsEnabled(v);
        },
        "Real-time convolution reverb (per-source room reflections)");

    dbgConsole.addFloat("refl_rays", 128.0f, 8192.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? static_cast<float>(svc->getReflectionNumRays()) : 4096.0f;
        },
        [](float v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setReflectionNumRays(static_cast<int>(v));
        },
        "Rays per reflection sim step (background thread)");

    dbgConsole.addFloat("refl_bounces", 1.0f, 8.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? static_cast<float>(svc->getReflectionNumBounces()) : 4.0f;
        },
        [](float v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setReflectionNumBounces(static_cast<int>(v));
        },
        "Bounces per ray (more = multi-room reverb propagation)");

    dbgConsole.addFloat("refl_duration", 0.5f, 4.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? svc->getReflectionDuration() : 2.0f;
        },
        [](float v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setReflectionDuration(v);
        },
        "Max reverb tail length in seconds");

    dbgConsole.addFloat("refl_throttle", 1.0f, 32.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? static_cast<float>(svc->getReflectionThrottle()) : 4.0f;
        },
        [](float v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setReflectionThrottle(static_cast<int>(v));
        },
        "Run reflection sim every Nth frame (higher = less CPU)");

    dbgConsole.addFloat("refl_tri_count", 0.0f, 999999.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? static_cast<float>(svc->getAcousticSceneTriCount()) : 0.0f;
        },
        [](float) { /* read-only */ },
        "Triangles in acoustic scene (read-only)");

    dbgConsole.addFloat("refl_max_voices", 1.0f, 32.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? static_cast<float>(svc->getMaxReflectionVoices()) : 4.0f;
        },
        [](float v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (svc) svc->setMaxReflectionVoices(static_cast<int>(v));
        },
        "Max active voices with convolution reverb");

    dbgConsole.addFloat("refl_sample_rate", 0.0f, 48000.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? static_cast<float>(svc->getReflectionSampleRate()) : 48000.0f;
        },
        [](float) { /* read-only */ },
        "Reflection pipeline sample rate (read-only, set via YAML)");

    dbgConsole.addFloat("refl_ambi_order", 0.0f, 1.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? static_cast<float>(svc->getAmbisonicsOrder()) : 0.0f;
        },
        [](float) { /* read-only */ },
        "Ambisonics order: 0=omnidirectional 1=directional (read-only, set via YAML)");

    dbgConsole.addFloat("transmission_scale", 0.1f, 100.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? svc->getTransmissionScale() : 10.0f;
        },
        [](float) { /* read-only — set via YAML, requires scene rebuild */ },
        "Material transmission multiplier (1=physical, 10=audible through walls, set via YAML)");

    // Bake probes: set to "on" to trigger re-baking
    dbgConsole.addBool("bake_probes",
        []() { return false; },  // always shows "off" (it's an action, not a state)
        [misPath](bool v) {
            if (!v) return;
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (!svc) return;
            std::string probePath = Darkness::AudioService::getProbeFilePath(misPath);
            std::fprintf(stderr, "Re-baking probes → %s\n", probePath.c_str());
            if (svc->bakeProbes(probePath)) {
                svc->loadProbes(probePath);
            }
        },
        "Re-bake acoustic probes (blocking, ~10-60s). Auto-baked on first run.");

    dbgConsole.addFloat("probe_count", 0.0f, 99999.0f,
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? static_cast<float>(svc->getProbeCount()) : 0.0f;
        },
        [](float) { /* read-only */ },
        "Number of loaded acoustic probes (read-only)");

    dbgConsole.addBool("record_audio",
        []() {
            auto svc = GET_SERVICE(Darkness::AudioService);
            return svc ? svc->isRecordingAudio() : false;
        },
        [](bool v) {
            auto svc = GET_SERVICE(Darkness::AudioService);
            if (!svc) return;
            if (v) svc->startAudioRecording();
            else   svc->stopAudioRecording();
        },
        "Record final audio output to WAV + position CSV for debugging");

    // ── Interaction ──

    dbgConsole.setGroup("Interaction");

    dbgConsole.addFloat("frob_distance", 1.0f, 30.0f,
        [&state]() {
            return state.frobSystem ? state.frobSystem->getFrobDistance()
                                    : Darkness::kDefaultFrobDistance;
        },
        [&state](float v) {
            if (state.frobSystem) state.frobSystem->setFrobDistance(v);
        },
        "Maximum interaction distance (original engine default: 8.0)");
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
        } else if (ev.type == SDL_WINDOWEVENT) {
            // Track window focus to avoid rendering to an invalid Metal drawable
            // when the user Command+Tabs away (macOS) or Alt+Tabs (Windows/Linux).
            if (ev.window.event == SDL_WINDOWEVENT_FOCUS_LOST) {
                state.windowFocused = false;
            } else if (ev.window.event == SDL_WINDOWEVENT_FOCUS_GAINED) {
                state.windowFocused = true;
            }
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
            // Also teleport physics player if physics mode is active
            if (state.physics) {
                Darkness::Vector3 spawnPos(state.spawnX, state.spawnY, state.spawnZ);
                state.physics->setPlayerPosition(spawnPos);
                state.physics->setPlayerYaw(state.spawnYaw);
            }
            std::fprintf(stderr, "Teleported to spawn (%.1f, %.1f, %.1f)\n",
                         state.spawnX, state.spawnY, state.spawnZ);
        } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_c
                   && state.physicsMode) {
            // C in physics mode: toggle crouch on/off
            state.crouchToggled = !state.crouchToggled;
        } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_SPACE
                   && !ev.key.repeat && state.physicsMode && state.physics) {
            // Space in physics mode: jump (edge-triggered, not continuous).
            // The original Dark Engine fires jump once per key-press event.
            // SDL_KEYDOWN with key.repeat filtered out ensures we only jump
            // on the initial press, not on OS key-repeat while held.
            state.physics->playerJump();
        } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_p
                   && !ev.key.repeat) {
            // P: toggle simulation pause. Physics, doors, tweqs, and platforms
            // freeze while audio and rendering continue.
            Darkness::SimServicePtr simSvc = GET_SERVICE(Darkness::SimService);
            if (simSvc->isSimRunning()) {
                if (simSvc->isSimPaused()) {
                    simSvc->unPauseSim();
                    std::fprintf(stderr, "Simulation UNPAUSED\n");
                } else {
                    simSvc->pauseSim();
                    std::fprintf(stderr, "Simulation PAUSED\n");
                }
            }
        } else if (ev.type == SDL_MOUSEBUTTONDOWN
                   && ev.button.button == SDL_BUTTON_RIGHT
                   && state.frobSystem) {
            // Right-click: frob the object under the crosshair
            if (state.frobSystem->hasTarget()) {
                state.frobSystem->executeFrob();
            }
        } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_g
                   && !ev.key.repeat && state.doorSystem) {
            // G: toggle the nearest door (debug interaction).
            // Finds the closest door to the camera and toggles it open/closed.
            Darkness::Vector3 camPos(state.cam.pos[0], state.cam.pos[1], state.cam.pos[2]);
            auto doorIDs = state.doorSystem->getAllDoorIDs();
            int32_t nearestID = 0;
            float nearestDist = 1e9f;
            for (int32_t id : doorIDs) {
                const auto *door = state.doorSystem->getDoor(id);
                if (!door) continue;
                float dx = door->basePosition.x - camPos.x;
                float dy = door->basePosition.y - camPos.y;
                float dz = door->basePosition.z - camPos.z;
                float dist = dx*dx + dy*dy + dz*dz;
                if (dist < nearestDist) {
                    nearestDist = dist;
                    nearestID = id;
                }
            }
            if (nearestID != 0) {
                state.doorSystem->activate(nearestID, Darkness::kDoorToggle);
                std::fprintf(stderr, "Toggled door %d (dist=%.1f)\n",
                             nearestID, std::sqrt(nearestDist));
            }
        }
    }
}

// ── Movement update ──
// WASD + vertical movement, suppressed while debug console is open.
// Two modes:
//   Fly mode (default): free camera with optional sphere collision (camera_collision)
//   Physics mode: gravity, ground detection, constraint collision (physics_mode)
static void updateMovement(
    float dt, Darkness::RuntimeState &state,
    const Darkness::MissionData &mission,
    const Darkness::DebugConsole &dbgConsole)
{
    if (dbgConsole.isOpen()) return;

    const Uint8 *keys = SDL_GetKeyboardState(nullptr);

    // ── Physics mode: player walks with gravity and collision ──
    if (state.physicsMode && state.physics) {
        // Normalized movement input [-1, 1] for forward/strafe
        float forward = 0.0f, right = 0.0f;
        if (keys[SDL_SCANCODE_W]) forward += 1.0f;
        if (keys[SDL_SCANCODE_S]) forward -= 1.0f;
        if (keys[SDL_SCANCODE_D]) right   += 1.0f;
        if (keys[SDL_SCANCODE_A]) right   -= 1.0f;

        state.physics->setPlayerMovement(forward, right);
        state.physics->setPlayerYaw(state.cam.yaw);

        // Speed modes: run (Ctrl, 2x) and sneak (LShift, 0.5x).
        // Sneak takes priority — can't run and sneak simultaneously.
        state.physics->setPlayerRunning(
            keys[SDL_SCANCODE_LCTRL] || keys[SDL_SCANCODE_RCTRL]);
        state.physics->setPlayerSneaking(keys[SDL_SCANCODE_LSHIFT] != 0);

        // Jump: handled by SDL_KEYDOWN event (edge-triggered, not held).
        // The original Dark Engine fires jump on key-press events, not per-frame
        // polling. This prevents jump chaining when spacebar is held — the player
        // must release and re-press space to jump again after landing.

        // Crouch with C key (toggle on/off, handled in event loop)
        state.physics->setPlayerCrouching(state.crouchToggled);

        // Lean with Q/E — lateral camera offset, physics body stays in place.
        // In physics mode Q/E lean instead of moving vertically.
        int leanDir = 0;
        if (keys[SDL_SCANCODE_Q]) leanDir -= 1;
        if (keys[SDL_SCANCODE_E]) leanDir += 1;
        state.physics->getPlayerPhysics().setLeanDirection(leanDir);

        // Feed camera pitch to physics for diagnostic logging
        state.physics->getPlayerPhysics().setCameraPitch(state.cam.pitch);

        // Step the physics simulation (respects sim pause — when paused,
        // player movement input is accepted but not simulated, so the player
        // freezes in place). SimService applies flow coefficient for slow-mo.
        {
            Darkness::SimServicePtr simSvc = GET_SERVICE(Darkness::SimService);
            float simDt = dt;
            if (simSvc->isSimRunning()) {
                if (simSvc->isSimPaused())
                    simDt = 0.0f;
                else
                    simDt = dt * simSvc->getFlowCoeff();
            }
            state.physics->step(simDt);
        }

        // Read back eye position and lean tilt from physics into camera
        Darkness::Vector3 eye = state.physics->getPlayerEyePosition();

        // DEBUG: detect large per-frame eye position jumps (visible teleportation)
        {
            float dx = eye.x - state.cam.pos[0];
            float dy = eye.y - state.cam.pos[1];
            float dz = eye.z - state.cam.pos[2];
            float eyeJump = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (eyeJump > 2.0f && state.cam.pos[0] != 0.0f) { // skip first frame
                fprintf(stderr, "[EYE-JUMP] dt=%.4f  dist=%.2f  eye=(%.1f,%.1f,%.1f)->(%.1f,%.1f,%.1f)  "
                    "cell=%d  body=(%.1f,%.1f,%.1f)\n",
                    dt, eyeJump,
                    state.cam.pos[0], state.cam.pos[1], state.cam.pos[2],
                    eye.x, eye.y, eye.z,
                    state.physics->getPlayerCell(),
                    state.physics->getPlayerPosition().x,
                    state.physics->getPlayerPosition().y,
                    state.physics->getPlayerPosition().z);
            }
        }

        state.cam.pos[0] = eye.x;
        state.cam.pos[1] = eye.y;
        state.cam.pos[2] = eye.z;
        state.cam.roll = state.physics->getPlayerPhysics().getLeanTilt();
        return;
    }

    // ── Fly mode: free camera with optional collision ──
    state.cam.roll = 0.0f;  // no lean tilt in fly mode
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

    // Ensure view 1's depth clear always executes, even if portal culling
    // produces zero visible cells (e.g., camera between cells). Without this,
    // bgfx skips the clear for views with no submissions, leaving stale depth
    // values that cause far geometry to flash through near geometry.
    bgfx::touch(1);

    // ── Portal culling: determine visible cells ──
    // Build view-projection matrix and extract frustum planes for portal tests.
    // When culling is disabled, visibleCells remains empty (skip filtering).
    // Guard band (world units) expands the frustum slightly to prevent large
    // objects from popping at screen edges due to cell-based culling.
    if (state.portalCulling) {
        // Build a wider projection for culling than for rendering.
        // 120-degree vertical FOV (~160° horizontal at 16:9) gives generous
        // margin so large objects aren't culled while still partially visible.
        constexpr float CULL_FOV = 120.0f;
        float cullProj[16];
        bx::mtxProj(cullProj, CULL_FOV,
                     float(WINDOW_WIDTH) / float(WINDOW_HEIGHT),
                     0.1f, 5000.0f,
                     bgfx::getCaps()->homogeneousDepth,
                     bx::Handedness::Right);
        float vp[16];
        bx::mtxMul(vp, fc.view, cullProj);
        ViewFrustum frustum;
        frustum.extractFromVP(vp);

        int32_t camCell = findCameraCell(mission.wrData, state.cam.pos[0], state.cam.pos[1], state.cam.pos[2]);
        fc.visibleCells = portalBFS(mission.wrData, mission.cellPortals, camCell, frustum,
                                     state.cam.pos[0], state.cam.pos[1], state.cam.pos[2]);
        state.cullVisibleCells = static_cast<uint32_t>(fc.visibleCells.size());

        // Build a tighter frustum from the actual rendering projection for
        // per-object AABB tests. Objects whose cells are culled by the wide
        // portal frustum still get rendered if their world-space AABB
        // intersects this rendering frustum.
        float renderVP[16];
        bx::mtxMul(renderVP, fc.view, fc.proj);
        fc.objFrustum.extractFromVP(renderVP);
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

    // Route all engine diagnostics to stdout so standard pipes and redirects work:
    //   ./darknessRender ... | grep FRICTION
    //   ./darknessRender ... > log.txt
    // The render binary is interactive (SDL window), so stdout carries no structured
    // data. Merging stderr into stdout makes all fprintf(stderr, ...) output
    // easily filterable. Line-buffering ensures output appears immediately.
    setvbuf(stdout, nullptr, _IOLBF, 0);
    dup2(fileno(stdout), fileno(stderr));

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

    // Create physics world from parsed WR cell geometry.
    // Must be created after loadMissionData() so mission.wrData is valid.
    // The physics instance lives for the duration of the program; wrData
    // must outlive it (both are in main's scope).
    // Map config physicsRate to PhysicsTimestep preset:
    //   <= 12 → VINTAGE (12.5Hz, 3 collision iters, authentic Dark Engine feel)
    //   >= 120 → ULTRA (120Hz, 1 iter, high-fidelity)
    //   else → MODERN (60Hz, 1 iter, default)
    auto physTimestep = (cfg.physicsRate <= 12)  ? Darkness::VINTAGE
                      : (cfg.physicsRate >= 120) ? Darkness::ULTRA
                      : Darkness::MODERN;
    state.physics = std::make_unique<Darkness::DarkPhysics>(mission.wrData, physTimestep);

    // ── Query player physics properties from archetype inheritance chain ──
    // The starting point object inherits P$PhysAttr/P$PhysDims from the
    // Avatar archetype in dark.gam. Values override PlayerPhysics defaults.
    if (mission.spawnInfo.found && mission.spawnInfo.objectID != 0) {
        Darkness::PropertyServicePtr propSvc = GET_SERVICE(Darkness::PropertyService);
        Darkness::PlayerPhysicsConfig cfg;

        // Read P$PhysAttr — may be 48 bytes (without poreSize) or 52 bytes
        Darkness::PropPhysAttr attr = {};
        size_t rawSize = 0;
        const uint8_t *rawData = Darkness::getPropertyRawData(
            propSvc.get(), "PhysAttr", mission.spawnInfo.objectID, rawSize);
        if (rawData && rawSize >= 20) {
            // Copy what's available (handles both 48 and 52-byte variants)
            std::memcpy(&attr, rawData, std::min(rawSize, sizeof(attr)));
            cfg.gravityScale = attr.gravity / 100.0f;
            cfg.mass = attr.mass;
            cfg.density = attr.density;
            cfg.elasticity = attr.elasticity;
            std::fprintf(stderr, "P$PhysAttr (obj %d): gravity=%.1f%% mass=%.1f "
                         "density=%.1f elasticity=%.2f friction=%.3f\n",
                         mission.spawnInfo.objectID, attr.gravity, attr.mass,
                         attr.density, attr.elasticity, attr.friction);
        } else {
            std::fprintf(stderr, "P$PhysAttr: not found for obj %d (using defaults)\n",
                         mission.spawnInfo.objectID);
        }

        // Read P$PhysDims (52 bytes) — sphere radii and submodel offsets
        Darkness::PropPhysDims dims = {};
        if (Darkness::getTypedProperty<Darkness::PropPhysDims>(
                propSvc.get(), "PhysDims", mission.spawnInfo.objectID, dims)) {
            if (dims.radius[0] > 0.0f) cfg.headRadius = dims.radius[0];
            if (dims.radius[1] > 0.0f) cfg.bodyRadius = dims.radius[1];
            cfg.headOffsetZ = dims.offset1Z;
            cfg.bodyOffsetZ = dims.offset2Z;
            std::fprintf(stderr, "P$PhysDims (obj %d): r0=%.2f r1=%.2f "
                         "off1=(%.2f,%.2f,%.2f) off2=(%.2f,%.2f,%.2f)\n",
                         mission.spawnInfo.objectID,
                         dims.radius[0], dims.radius[1],
                         dims.offset1X, dims.offset1Y, dims.offset1Z,
                         dims.offset2X, dims.offset2Y, dims.offset2Z);
        } else {
            std::fprintf(stderr, "P$PhysDims: not found for obj %d (using defaults)\n",
                         mission.spawnInfo.objectID);
        }

        state.physics->applyPlayerConfig(cfg);
    }

    // Enable stair step diagnostics if --step-log was passed
    if (cfg.stepLog && state.physics) {
        state.physics->getPlayerPhysics().setStepLog(true);
        std::fprintf(stderr, "Stair step logging enabled (--step-log)\n");
    }

    // ── Load motion capture data from motions.crf ──
    // motions.crf is present in ALL Dark Engine games (Thief 1/2, System Shock 2).
    // Currently loaded for future NPC animation (Phase 4 AI). The player camera uses
    // pre-scripted offset poses (matching the original engine's PlayerMotionTable),
    // NOT raw motion capture data — mocap is only for third-person creature animation.
    std::unique_ptr<Darkness::MotionService> motionSvc;
    if (!resPath.empty()) {
        motionSvc = std::make_unique<Darkness::MotionService>();
        if (motionSvc->loadFromCRF(resPath)) {
            std::fprintf(stderr, "Motion: loaded %d clips from motions.crf (for future NPC use)\n",
                         motionSvc->clipCount());
        } else {
            std::fprintf(stderr, "WARNING: failed to load motions.crf from %s\n"
                                 "         NPC animation will not be available.\n",
                         resPath.c_str());
        }
    }

    // ── Load sound resources from snd.crf ──
    if (!resPath.empty()) {
        Darkness::AudioServicePtr audioSvc = GET_SERVICE(Darkness::AudioService);
        if (!audioSvc->loadSoundResources(resPath, cli.schemasPath)) {
            std::fprintf(stderr, "WARNING: failed to load snd.crf from %s\n"
                                 "         Sound playback will not be available.\n",
                         resPath.c_str());
        }
    }

    // Parse TXLIST early — needed by the acoustic mesh builder for material
    // keyword matching. The full texture loading (CRF I/O, GPU upload) happens
    // later in loadWorldTextures(), but we need the texture name strings now.
    if (mission.texturedMode && mission.txList.textures.empty()) {
        try {
            mission.txList = Darkness::parseTXList(misPath);
            std::fprintf(stderr, "TXLIST (early parse for acoustics): %zu textures\n",
                         mission.txList.textures.size());
        } catch (const std::exception &e) {
            std::fprintf(stderr, "Early TXLIST parse failed: %s\n", e.what());
        }
    }

    // ── Build Steam Audio acoustic scene from WR world geometry ──
    // Build once at load time, no per-frame rebuilds needed.
    {
        Darkness::AudioServicePtr audioSvc = GET_SERVICE(Darkness::AudioService);

        const auto &wr = mission.wrData;
        Darkness::AcousticSceneData fullScene;

        // Vertex deduplication: hash map from quantized position to vertex index.
        // Adjacent cells share vertices at polygon edges — without dedup, each
        // polygon pushes its own copy, inflating the vertex buffer ~3x.
        // Quantize to 0.01 units to merge coincident vertices robustly.
        struct VertKey {
            int32_t x, y, z;
            bool operator==(const VertKey &o) const {
                return x == o.x && y == o.y && z == o.z;
            }
        };
        struct VertKeyHash {
            size_t operator()(const VertKey &k) const {
                // FNV-1a inspired hash for 3 ints
                size_t h = 0x811c9dc5u;
                h ^= static_cast<size_t>(k.x); h *= 0x01000193u;
                h ^= static_cast<size_t>(k.y); h *= 0x01000193u;
                h ^= static_cast<size_t>(k.z); h *= 0x01000193u;
                return h;
            }
        };
        std::unordered_map<VertKey, uint32_t, VertKeyHash> vertexMap;
        vertexMap.reserve(wr.numCells * 20);  // rough estimate

        // Helper: get or insert a deduplicated vertex, returns index
        auto getVertexIndex = [&](float x, float y, float z) -> uint32_t {
            // Quantize to 0.01 units for robust merging
            VertKey key{static_cast<int32_t>(std::round(x * 100.0f)),
                        static_cast<int32_t>(std::round(y * 100.0f)),
                        static_cast<int32_t>(std::round(z * 100.0f))};
            auto it = vertexMap.find(key);
            if (it != vertexMap.end())
                return it->second;
            uint32_t idx = static_cast<uint32_t>(fullScene.vertices.size() / 3);
            fullScene.vertices.push_back(x);
            fullScene.vertices.push_back(y);
            fullScene.vertices.push_back(z);
            vertexMap[key] = idx;
            return idx;
        };

        for (uint32_t ci = 0; ci < wr.numCells; ++ci) {
            const auto &cell = wr.cells[ci];
            int numSolid = cell.numPolygons - cell.numPortals;

            for (int pi = 0; pi < cell.numPolygons; ++pi) {
                const auto &poly = cell.polygons[pi];

                bool isPortal = (pi >= numSolid);

                // The WR polygon ordering within each cell is:
                //   [0, numSolid)        — solid walls (textured, physical)
                //   [numSolid, numTex)   — rendered portals (water/glass boundaries)
                //   [numTex, numPolys)   — non-rendered portals (BSP splits)
                //
                // Non-rendered portals are BSP subdivision artifacts — invisible
                // boundaries created when the BSP compiler splits large cells.
                // In outdoor areas, these create a lattice of phantom surfaces
                // floating in mid-air. They must be EXCLUDED from the acoustic
                // mesh to prevent phantom reflections.
                bool isNonRenderedPortal = isPortal && (pi >= cell.numTextured);
                if (isNonRenderedPortal)
                    continue;

                // Skip sky polygons (BACKHACK_IDX = 249) — only solid polys have textures
                if (!isPortal && pi < cell.numTextured && cell.texturing[pi].txt == 249)
                    continue;

                // Also skip water boundary textures (WATERIN_IDX=247, WATEROUT_IDX=248)
                // — these are medium transitions, not acoustically reflective surfaces
                if (!isPortal && pi < cell.numTextured) {
                    uint8_t txtIdx = cell.texturing[pi].txt;
                    if (txtIdx >= 247)  // WATERIN, WATEROUT, BACKHACK — all non-physical
                        continue;
                }

                // Determine texture name for material lookup.
                // Rendered portal polygons (water/glass boundaries) use the _portal
                // material. Solid walls use their TXLIST texture for material matching.
                std::string texName = "generic";
                if (isPortal) {
                    texName = "_portal";
                } else if (pi < cell.numTextured) {
                    uint8_t txtIdx = cell.texturing[pi].txt;
                    if (txtIdx < mission.txList.textures.size()) {
                        const auto &entry = mission.txList.textures[txtIdx];
                        if (!entry.fullPath.empty())
                            texName = entry.fullPath;
                    }
                }

                // Skip degenerate polygons
                if (poly.count < 3)
                    continue;

                // Collect deduplicated vertex indices for this polygon
                std::vector<uint32_t> polyVerts(poly.count);
                for (int vi = 0; vi < poly.count; ++vi) {
                    uint8_t idx = cell.polyIndices[pi][vi];
                    const auto &v = cell.vertices[idx];
                    polyVerts[vi] = getVertexIndex(v.x, v.y, v.z);
                }

                // Fan-triangulate the polygon
                for (int t = 1; t < poly.count - 1; ++t) {
                    fullScene.indices.push_back(
                        static_cast<int32_t>(polyVerts[0]));
                    fullScene.indices.push_back(
                        static_cast<int32_t>(polyVerts[t + 1]));
                    fullScene.indices.push_back(
                        static_cast<int32_t>(polyVerts[t]));
                    fullScene.texNames.push_back(texName);
                }
            }
        }

        size_t numTris = fullScene.indices.size() / 3;
        size_t numVerts = fullScene.vertices.size() / 3;
        std::fprintf(stderr, "Acoustic mesh: %zu vertices, %zu triangles (%u cells)\n",
                     numVerts, numTris, wr.numCells);

        // Apply audio config before building the acoustic scene
        audioSvc->setReflectionRateDivisor(cfg.reflectionRateDivisor);
        audioSvc->setConvolutionWorkerCount(cfg.convolutionWorkers);
        audioSvc->setAmbisonicsOrder(cfg.ambisonicsOrder);
        audioSvc->setMaxReflectionVoices(cfg.maxReflectionVoices);
        audioSvc->setReflectionNumRays(cfg.reflectionNumRays);
        audioSvc->setReflectionNumBounces(cfg.reflectionNumBounces);
        audioSvc->setReflectionDuration(cfg.reflectionDuration);
        audioSvc->setReflectionThrottle(cfg.reflectionThrottle);
        audioSvc->setTransmissionScale(cfg.transmissionScale);
        audioSvc->setAbsorptionScale(cfg.absorptionScale);
        audioSvc->setDiffuseSamples(cfg.diffuseSamples);
        audioSvc->setBakeDiffuseSamples(cfg.bakeDiffuseSamples);
        audioSvc->setOcclusionRadius(cfg.occlusionRadius);
        audioSvc->setOcclusionSamples(cfg.occlusionSamples);
        audioSvc->setPortalRoutingEnabled(cfg.portalRouting);
        audioSvc->setProbePathingEnabled(cfg.probePathing);
        audioSvc->setReflectionsEnabled(cfg.realtimeReflections);

        // DSP chain config (soft limiter, compressor, EQ, ducking)
        audioSvc->setDSPLimiterEnabled(cfg.dspLimiter);
        audioSvc->setDSPLimiterKnee(cfg.dspLimiterKnee);
        audioSvc->setDSPCompressorEnabled(cfg.dspCompressor);
        audioSvc->setDSPCompThreshold(cfg.dspCompThreshold);
        audioSvc->setDSPCompRatio(cfg.dspCompRatio);
        audioSvc->setDSPEQEnabled(cfg.dspEQ);
        audioSvc->setDSPEQFreq(cfg.dspEQFreq);
        audioSvc->setDSPEQGain(cfg.dspEQGain);
        audioSvc->setDSPDuckingEnabled(cfg.dspDucking);
        audioSvc->setDSPDuckAmount(cfg.dspDuckAmount);

        if (!audioSvc->buildAcousticScene(fullScene)) {
            std::fprintf(stderr, "WARNING: failed to build acoustic scene\n"
                                 "         Steam Audio spatialization disabled.\n");
        }

        // Try to load baked probe data from the darkness install folder.
        // If no probes exist, auto-bake them (with a progress bar).
        {
            std::string probePath = Darkness::AudioService::getProbeFilePath(misPath);
            if (!audioSvc->loadProbes(probePath)) {
                // No baked probes — need to bake.
                // This is done BEFORE the render loop starts, so we can use
                // a simple bgfx debug text progress bar during baking.
                state.probeBakePath = probePath;
                state.probeBakeNeeded = true;
            }
        }

        // Store acoustic mesh data for debug wireframe overlay.
        // GPU buffer is created later after bgfx::init.
        state.acousticVerts = fullScene.vertices;
        state.acousticIndices = fullScene.indices;
    }

    // Inject raycaster into the world query facade — enables ray-vs-world
    // queries (AI line-of-sight, physics, sound occlusion) via portal traversal
    worldQuery->setRaycaster(
        [&mission](const Darkness::Vector3 &from, const Darkness::Vector3 &to,
                  Darkness::RayHit &hit) {
            return Darkness::raycastWorld(mission.wrData, from, to, hit);
        });

    // Give the renderer access to the mutable object state map (owned by worldQuery)
    state.objectStates = &worldQuery->objectStates();

    // ── Initialize door system ──
    // Scans for P$RotDoor and P$TransDoor properties, creates DoorState entries,
    // and initializes ObjectState transforms for each door.
    Darkness::DoorSystem doorSystem;
    {
        Darkness::PropertyServicePtr propSvc = GET_SERVICE(Darkness::PropertyService);
        Darkness::ObjectServicePtr objSvc = GET_SERVICE(Darkness::ObjectService);
        doorSystem.init(propSvc.get(), objSvc.get(), state.objectStates);
    }
    state.doorSystem = &doorSystem;

    // ── Initialize frob system ──
    // Casts a short ray from the camera each frame to find the nearest frobbable
    // object (doors, switches, pickups). Right-click triggers the frob action.
    Darkness::FrobSystem frobSystem;
    {
        Darkness::PropertyServicePtr propSvc = GET_SERVICE(Darkness::PropertyService);
        Darkness::ObjectServicePtr objSvc = GET_SERVICE(Darkness::ObjectService);
        Darkness::ObjectCollisionWorld *ocw =
            state.physics ? state.physics->getObjectCollisionWorld() : nullptr;
        frobSystem.init(propSvc.get(), objSvc.get(), &doorSystem, ocw);
    }
    state.frobSystem = &frobSystem;

    // ── Load world textures: TXLIST, fam.crf textures, flow textures, skybox ──
    loadWorldTextures(misPath, resPath, mission);

    // ── Build per-texture friction table from P$Friction on texture archetypes ──
    // The original engine maps each TXLIST texture index to a "t_fam/<family>/<name>"
    // archetype in dark.gam, then reads P$Friction (float, default 1.0) from that
    // archetype (with MetaProp inheritance). We build a flat lookup table at load time
    // to avoid string lookups per physics step.
    //
    // TODO: The shipping Thief 2 gamesys only has 3 P$Friction records (city/roof,
    // city/rooftile at 0.15, and object 2 at 0.0). Ice/snow textures (vmawwin/iceclif1
    // etc.) have no friction override — verify whether the original engine has additional
    // friction sources beyond P$Friction (e.g. texture-name heuristics, per-mission
    // overrides, or Climbability affecting friction). May need a name-based fallback
    // or manual friction table for ice/snow surfaces if gameplay testing reveals issues.
    {
        Darkness::ObjectServicePtr objSvc = GET_SERVICE(Darkness::ObjectService);
        Darkness::PropertyServicePtr propSvc = GET_SERVICE(Darkness::PropertyService);
        size_t numTex = mission.txList.textures.size();
        mission.frictionTable.assign(numTex, 1.0f);

        int nonDefault = 0;
        for (size_t i = 0; i < numTex; ++i) {
            const auto &entry = mission.txList.textures[i];
            if (entry.family.empty() && entry.name.empty())
                continue;

            // Texture archetypes in dark.gam are named "t_fam/<family>/<name>"
            // with original case preserved (objectNamed is case-sensitive).
            std::string archName = "t_fam/" + entry.fullPath;

            int archID = objSvc->named(archName);
            if (archID == 0) continue;  // no archetype found for this texture

            float friction = 1.0f;
            if (Darkness::getTypedProperty<float>(propSvc.get(), "Friction", archID, friction)) {
                if (std::fabs(friction - 1.0f) > 0.001f) {
                    mission.frictionTable[i] = friction;
                    ++nonDefault;
                    std::fprintf(stderr, "[FRICTION] %s: %.2f (idx %zu)\n",
                                 archName.c_str(), friction, i);
                }
            }
        }
        std::fprintf(stderr, "[FRICTION] %d textures with non-default friction out of %zu loaded\n",
                     nonDefault, numTex);

        // Pass friction table to player physics
        if (state.physics) {
            state.physics->setFrictionTable(mission.frictionTable);
        }
    }

    // ── Build per-texture climbability table from P$Climbabil ──
    // Same pattern as friction table above. P$Climbabil is a single float
    // "factor" on texture archetypes. Non-zero values boost friction on steep
    // surfaces (helping the player walk up ramps) — this is NOT a climb-mode
    // trigger. The original engine disabled terrain wall climbing in shipping
    // Thief 2; only OBB objects with climbable_sides support actual climbing.
    {
        Darkness::ObjectServicePtr objSvc = GET_SERVICE(Darkness::ObjectService);
        Darkness::PropertyServicePtr propSvc = GET_SERVICE(Darkness::PropertyService);
        size_t numTex = mission.txList.textures.size();
        mission.climbabilityTable.assign(numTex, 0.0f);

        int nonDefault = 0;
        for (size_t i = 0; i < numTex; ++i) {
            const auto &entry = mission.txList.textures[i];
            if (entry.family.empty() && entry.name.empty())
                continue;

            std::string archName = "t_fam/" + entry.fullPath;
            int archID = objSvc->named(archName);
            if (archID == 0) continue;

            float climbability = 0.0f;
            if (Darkness::getTypedProperty<float>(propSvc.get(), "Climbabil", archID, climbability)) {
                if (climbability > 0.001f) {
                    mission.climbabilityTable[i] = climbability;
                    ++nonDefault;
                    std::fprintf(stderr, "[CLIMBABILITY] %s: %.2f (idx %zu)\n",
                                 archName.c_str(), climbability, i);
                }
            }
        }
        std::fprintf(stderr, "[CLIMBABILITY] %d textures with non-zero climbability out of %zu loaded\n",
                     nonDefault, numTex);

        // Pass climbability table to player physics
        if (state.physics) {
            state.physics->setClimbabilityTable(mission.climbabilityTable);
        }
    }

    // ── Build per-texture material keyword table for footstep sounds ──
    // Maps each TXLIST texture index → material keyword ("stone", "metal", etc.)
    // using substring matching on texture family/name paths.
    {
        Darkness::AudioServicePtr audioSvc = GET_SERVICE(Darkness::AudioService);
        size_t numTex = mission.txList.textures.size();
        std::vector<std::string> materials(numTex);
        for (size_t i = 0; i < numTex; ++i) {
            const auto &entry = mission.txList.textures[i];
            if (!entry.fullPath.empty()) {
                materials[i] = Darkness::lookupAcousticMaterialKeyword(entry.fullPath);
            } else {
                materials[i] = "generic";
            }
        }
        audioSvc->setTextureMaterials(std::move(materials));

        // Register footstep and landing callbacks with physics
        if (state.physics) {
            state.physics->setFootstepCallback(
                [audioSvc](const Darkness::Vector3 &pos, float speed, int matIdx) {
                    audioSvc->playFootstep(pos, speed, matIdx);
                });
            state.physics->setLandingCallback(
                [audioSvc](const Darkness::Vector3 &pos, float fallSpeed, int matIdx) {
                    audioSvc->playLanding(pos, fallSpeed, matIdx);
                });
        }
    }

    // ── Load object assets: properties, .bin models, textures from obj.crf ──
    loadObjectAssets(misPath, resPath, cfg, mission, state);

    // ── Build object collision bodies from .bin bounding boxes ──
    // Creates OBB/sphere collision bodies for placed objects (crates, furniture,
    // doors) so the player can't walk through them. Uses P$PhysType properties
    // to determine shape type and P$PhysDims for optional dimension overrides.
    if (state.physics && !mission.objData.objects.empty()) {
        Darkness::PropertyServicePtr propSvc = GET_SERVICE(Darkness::PropertyService);
        state.physics->buildObjectCollision(
            mission.objData.objects, mission.parsedModels, propSvc.get());
    }

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

    // ── Create acoustic mesh debug wireframe buffer (after bgfx init) ──
    if (!state.acousticIndices.empty()) {
        size_t numTris = state.acousticIndices.size() / 3;
        std::vector<Darkness::PosColorVertex> lineVerts;
        lineVerts.reserve(numTris * 6);
        uint32_t wireColor = 0x80FFFF00;  // ABGR: semi-transparent cyan
        for (size_t t = 0; t < numTris; ++t) {
            int32_t i0 = state.acousticIndices[t * 3 + 0];
            int32_t i1 = state.acousticIndices[t * 3 + 1];
            int32_t i2 = state.acousticIndices[t * 3 + 2];
            float x0 = state.acousticVerts[i0*3], y0 = state.acousticVerts[i0*3+1], z0 = state.acousticVerts[i0*3+2];
            float x1 = state.acousticVerts[i1*3], y1 = state.acousticVerts[i1*3+1], z1 = state.acousticVerts[i1*3+2];
            float x2 = state.acousticVerts[i2*3], y2 = state.acousticVerts[i2*3+1], z2 = state.acousticVerts[i2*3+2];
            lineVerts.push_back({x0,y0,z0,wireColor}); lineVerts.push_back({x1,y1,z1,wireColor});
            lineVerts.push_back({x1,y1,z1,wireColor}); lineVerts.push_back({x2,y2,z2,wireColor});
            lineVerts.push_back({x2,y2,z2,wireColor}); lineVerts.push_back({x0,y0,z0,wireColor});
        }
        state.acousticLineCount = static_cast<uint32_t>(lineVerts.size());
        const bgfx::Memory *mem = bgfx::copy(lineVerts.data(),
            static_cast<uint32_t>(lineVerts.size() * sizeof(Darkness::PosColorVertex)));
        state.acousticVBH = bgfx::createVertexBuffer(mem, Darkness::PosColorVertex::layout);
        std::fprintf(stderr, "Acoustic debug mesh: %zu tris, %u line verts\n",
                     numTris, state.acousticLineCount);
        // Free CPU-side data now that GPU buffer is created
        state.acousticVerts.clear();
        state.acousticVerts.shrink_to_fit();
        state.acousticIndices.clear();
        state.acousticIndices.shrink_to_fit();
    }

    // ── Initialize runtime state: mode string, model isolation, spawn/camera ──
    initRuntimeState(mission, meshes, gpu, camX, camY, camZ, state);

    // Initialize physics player at spawn position.
    // Use the camera position (eye level) directly as initial body center.
    // Gravity will pull the player down to the floor within a fraction of a
    // second, settling them at the correct standing height.  Subtracting
    // HEAD_OFFSET here would risk placing the body center below the floor
    // (outside all WR cells), which disables collision entirely.
    if (state.physics) {
        Darkness::Vector3 bodyPos(state.cam.pos[0], state.cam.pos[1], state.cam.pos[2]);
        state.physics->setPlayerPosition(bodyPos);
        state.physics->setPlayerYaw(state.cam.yaw);
        std::fprintf(stderr, "Physics: player spawned at body (%.1f, %.1f, %.1f)\n",
                     bodyPos.x, bodyPos.y, bodyPos.z);
    }

    // ── Debug console for runtime settings management ──
    // Opened with backtick (`), provides tab-completion and value editing.
    Darkness::DebugConsole dbgConsole;
    registerConsoleSettings(dbgConsole, state, window, misPath);

    updateTitleBar(window, state);

    // ── Auto-bake probes if needed (with progress bar) ──
    if (state.probeBakeNeeded) {
        Darkness::AudioServicePtr audioSvc = GET_SERVICE(Darkness::AudioService);
        if (audioSvc) {
            std::atomic<float> bakeProgress{0.0f};
            std::atomic<bool> bakeDone{false};
            bool bakeSuccess = false;

            // Run baking on a background thread so we can render progress
            std::thread bakeThread([&]() {
                bakeSuccess = audioSvc->bakeProbes(state.probeBakePath,
                                                    &bakeProgress);
                bakeDone.store(true, std::memory_order_release);
            });

            // Render progress bar until baking completes.
            // The bake runs 3 sequential stages, each reporting 0→1 progress:
            //   Stage 0: Probe placement + pathing visibility (fast, ~10s)
            //   Stage 1: Pathing shortest paths (fast, ~10s)
            //   Stage 2: Reflection IR baking (slow, minutes)
            // We detect stage transitions when progress drops and map to
            // overall 0→100% with weighted allocation (stages 0+1 = 20%, stage 2 = 80%).
            int bakeStage = 0;
            float lastProg = 0.0f;
            const char *stageNames[] = {
                "Baking pathing visibility...",
                "Computing shortest paths...",
                "Baking reflection IRs (this takes a while)..."
            };
            // Weight allocation: pathing stages are fast, reflection is slow
            const float stageStart[] = {0.0f, 0.10f, 0.20f};
            const float stageWeight[] = {0.10f, 0.10f, 0.80f};

            while (!bakeDone.load(std::memory_order_acquire)) {
                SDL_Event ev;
                while (SDL_PollEvent(&ev)) {
                    if (ev.type == SDL_QUIT) {
                        state.running = false;
                        break;
                    }
                }
                if (!state.running) break;

                float rawProg = bakeProgress.load(std::memory_order_relaxed);
                // Detect stage transition: progress drops back toward 0
                if (rawProg < lastProg - 0.1f && bakeStage < 2) bakeStage++;
                lastProg = rawProg;

                // Map to overall progress with weighted stages
                float prog = stageStart[bakeStage] + rawProg * stageWeight[bakeStage];
                prog = std::min(prog, 1.0f);
                int pct = static_cast<int>(prog * 100.0f);

                // Render progress bar using bgfx debug text
                bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
                                   0x1a1a2eFF, 1.0f, 0);
                bgfx::setViewRect(0, 0, 0, 1280, 720);
                bgfx::touch(0);

                bgfx::setDebug(BGFX_DEBUG_TEXT);
                bgfx::dbgTextClear();

                // Title + stage name
                bgfx::dbgTextPrintf(2, 10, 0x0f, "DARKNESS ENGINE");
                bgfx::dbgTextPrintf(2, 12, 0x07, "Baking acoustic probes (stage %d/3):",
                                    bakeStage + 1);
                bgfx::dbgTextPrintf(2, 13, 0x07, "  %s", stageNames[bakeStage]);

                // Progress bar (40 chars wide)
                int barWidth = 40;
                int filled = static_cast<int>(prog * barWidth);
                char bar[64] = {};
                for (int i = 0; i < barWidth; ++i)
                    bar[i] = (i < filled) ? '#' : '-';
                bar[barWidth] = '\0';

                bgfx::dbgTextPrintf(2, 15, 0x0a, "[%s] %d%%", bar, pct);
                bgfx::dbgTextPrintf(2, 17, 0x08, "This only happens once per mission.");
                bgfx::dbgTextPrintf(2, 18, 0x08, "Probe data will be cached in:");
                bgfx::dbgTextPrintf(2, 19, 0x08, "  %s", state.probeBakePath.c_str());

                bgfx::frame();
                SDL_Delay(50);  // ~20fps for the progress display
            }

            bakeThread.join();

            if (bakeSuccess && state.running) {
                audioSvc->loadProbes(state.probeBakePath);
                std::fprintf(stderr, "Probe baking complete — %d probes loaded\n",
                             audioSvc->getProbeCount());
            }

            bgfx::setDebug(0);  // disable debug text
        }
        state.probeBakeNeeded = false;
    }

    // ── Set up LoopService for priority-ordered frame dispatch ──
    //
    // Execution order per frame (by LoopClient priority):
    //   1. InputClient     (priority 1)    — SDL events, movement, listener xform
    //   2. SimService      (priority 50)   — sim time, pause, dispatches to SimListeners
    //   3. AudioService    (priority 100)  — voice management, Steam Audio sim kick
    //   4. RenderClient    (priority 1024) — lightmaps, all render passes, bgfx::frame
    //
    // SimService and AudioService register themselves during bootstrapFinished().
    // We create the LoopMode, register our two FunctionalLoopClients, then drive
    // the loop with loopService->step().
    Darkness::LoopServicePtr loopSvc = GET_SERVICE(Darkness::LoopService);
    Darkness::SimServicePtr simSvc = GET_SERVICE(Darkness::SimService);

    {
        Darkness::LoopModeDefinition gameMode;
        gameMode.id = 1;
        gameMode.mask = LOOPMODE_INPUT | LOOPMODE_RENDER;
        gameMode.name = "GameRunning";
        loopSvc->createLoopMode(gameMode);
        loopSvc->requestLoopMode(gameMode.id);
        // Force the pending mode request to take effect immediately
        // (normally happens at start of run(), but we use step())
        loopSvc->step();
    }

    // Wire continuous audio blocking updates during door animation
    doorSystem.setAudioBlockingCallback(
        [](int32_t room1, int32_t room2, float factor) {
            Darkness::AudioServicePtr audioSvc = GET_SERVICE(Darkness::AudioService);
            if (audioSvc)
                audioSvc->setBlockingFactor(room1, room2, factor);
        });

    // Apply initial blocking for all doors that start closed
    {
        Darkness::AudioServicePtr audioSvc = GET_SERVICE(Darkness::AudioService);
        for (int32_t id : doorSystem.getAllDoorIDs()) {
            const auto *door = doorSystem.getDoor(id);
            if (door && door->status == Darkness::kDoorClosed &&
                door->soundBlocking > 0.0f &&
                door->room1 >= 0 && door->room2 >= 0 &&
                door->room1 != door->room2) {
                audioSvc->setBlockingFactor(door->room1, door->room2,
                                             door->soundBlocking);
            }
        }
    }

    // Register DoorSystem as a SimListener so it receives simStep() calls.
    // Priority 10 = before physics (which would be 20+ when registered).
    simSvc->registerListener(&doorSystem, 10);

    // Set up door event callback: update audio blocking and log status changes.
    // When a door starts opening, remove sound blocking. When it finishes
    // closing, apply full blocking. During animation, blocking scales with
    // the open fraction (updated per-frame below).
    doorSystem.setEventCallback([](int32_t objID, Darkness::DoorStatus status,
                                    const Darkness::DoorState &door) {
        const char *statusNames[] = {"CLOSED", "OPEN", "CLOSING", "OPENING", "HALT"};
        const char *name = (status >= 0 && status <= 4) ? statusNames[status] : "?";
        std::fprintf(stderr, "Door %d: %s (room %d<->%d, blocking=%.0f%%)\n",
                     objID, name, door.room1, door.room2, door.soundBlocking * 100.0f);

        // Update audio blocking on door state transitions
        if (door.room1 >= 0 && door.room2 >= 0 && door.room1 != door.room2) {
            Darkness::AudioServicePtr audioSvc = GET_SERVICE(Darkness::AudioService);
            if (audioSvc) {
                if (status == Darkness::kDoorClosed) {
                    // Door fully closed: apply maximum blocking
                    audioSvc->setBlockingFactor(door.room1, door.room2,
                                                 door.soundBlocking);
                } else if (status == Darkness::kDoorOpen) {
                    // Door fully open: remove blocking
                    audioSvc->setBlockingFactor(door.room1, door.room2, 0.0f);
                }
                // Opening/closing transitions: blocking updated per-frame below
            }
        }
    });

    // Start simulation time — SimListeners will begin receiving simStep()
    simSvc->startSim();

    // InputClient: SDL events, player movement, audio listener transform.
    // Runs first (priority 1) so camera position is current before audio/render.
    Darkness::FunctionalLoopClient inputClient(
        LOOPCLIENT_ID_INPUT, "InputClient",
        LOOPMODE_INPUT | LOOPMODE_RENDER, LOOPCLIENT_PRIORITY_INPUT,
        [&](float dt) {
            state.waterElapsed += dt;

            handleEvents(state, dbgConsole, window);

            // When unfocused, skip movement but still process events
            // (so we detect refocus). Render client handles the idle case.
            if (!state.windowFocused)
                return;

            updateMovement(dt, state, mission, dbgConsole);

            // Update frob target — cast ray from camera to find nearest frobbable object
            if (state.frobSystem) {
                state.frobSystem->update(state.cam);
            }

            // Set audio listener position before AudioService::loopStep runs
            Darkness::AudioServicePtr audioSvc = GET_SERVICE(Darkness::AudioService);
            audioSvc->setListenerTransform(
                Darkness::Vector3(state.cam.pos[0], state.cam.pos[1], state.cam.pos[2]),
                state.cam.yaw, state.cam.pitch);

            if (state.physics) {
                audioSvc->setPlayerInWater(
                    state.physics->getPlayerPhysics().isInWater());
            }
        });

    // RenderClient: lightmaps, all render passes, bgfx::frame().
    // Runs last (priority 1024).
    Darkness::FunctionalLoopClient renderClient(
        LOOPCLIENT_ID_RENDERER, "RenderClient",
        LOOPMODE_RENDER, LOOPCLIENT_PRIORITY_RENDERER,
        [&](float dt) {
            // When the window loses focus (Command+Tab on macOS, Alt+Tab elsewhere),
            // skip rendering entirely. The Metal drawable is invalid while backgrounded
            // and submitting draw calls would segfault. We still call bgfx::frame() with
            // no submissions to keep the internal state ticking, and sleep briefly to
            // avoid burning CPU in a tight poll loop.
            if (!state.windowFocused) {
                bgfx::frame();
                SDL_Delay(16);  // ~60Hz idle polling rate
                return;
            }

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

            // Frob target indicator — show object name at screen center
            if (state.frobSystem && state.frobSystem->hasTarget()) {
                const auto &target = state.frobSystem->getTarget();
                // bgfx debug text: row 14 (near center), centered horizontally
                bgfx::dbgTextPrintf(30, 14, 0x0f, "[%s]  (%.1f)",
                                    target.name.c_str(), target.distance);
            }

            // Debug console overlay (no-op when closed)
            dbgConsole.render();

            bgfx::frame();
        });

    loopSvc->addLoopClient(&inputClient);
    loopSvc->addLoopClient(&renderClient);

    // Note: AudioService already registered itself as a LoopClient during
    // bootstrapFinished() (priority 100). It receives loopStep(dt) from
    // LoopService — the manual audioSvc->updateAudio(dt) call is removed.
    // SimService registered itself in init() (priority 50).

    // ── Main loop — LoopService drives frame dispatch ──
    while (state.running && !loopSvc->isTerminationRequested()) {
        loopSvc->step();
    }

    // Clean up LoopClients and SimListeners before state is destroyed
    loopSvc->removeLoopClient(&inputClient);
    loopSvc->removeLoopClient(&renderClient);
    simSvc->unregisterListener(&doorSystem);
    simSvc->endSim();

    destroyGPUResources(gpu);
    shutdownWindow(window);

    std::fprintf(stderr, "Clean shutdown.\n");
    return 0;
}
