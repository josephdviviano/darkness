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

// DarknessRenderState.h — Data-holding structs for the renderer
//
// Groups the ~80 local variables from main() into logical structs:
//   MissionData  — CPU-side parsed mission content (geometry, textures, objects)
//   BuiltMeshes  — CPU mesh data between parsing and GPU upload
//   GPUResources — All bgfx handles (programs, uniforms, buffers, textures)
//   RuntimeState — Mutable per-frame state (camera, config toggles, debug)
//   FrameContext — Per-frame computed values (matrices, fog, culling results)
//
// These structs contain no logic — they are pure data aggregates used by
// the init, render-loop, and cleanup functions.

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>

#include <bgfx/bgfx.h>

#include "DarknessRendererCore.h"
#include "DarknessRendererExtended.h"
#include "LightmapAtlas.h"
#include "LightingSystem.h"
#include "ObjectPropParser.h"
#include "BinMeshParser.h"
#include "SpawnFinder.h"
#include "TXListParser.h"
#include "PCXDecoder.h"
#include "physics/DarkPhysics.h"

namespace Darkness {

// ── MissionData — CPU-side parsed mission content ──
// Everything loaded from the .mis/.gam files and CRF archives before
// any GPU resources are created.

struct MissionData {
    // World geometry + portal graph
    WRParsedData                                      wrData;
    std::vector<std::vector<CellPortalInfo>>          cellPortals;

    // Texture catalog
    TXList                                            txList;
    std::unordered_map<uint8_t, DecodedImage>         loadedTextures;
    std::unordered_map<uint8_t, TexDimensions>        texDims;

    // Water flow textures
    std::unordered_map<uint8_t, DecodedImage>         flowLoadedTextures;
    std::unordered_map<uint8_t, TexDimensions>        flowTexDims;
    FlowData                                          flowData;

    // Sky
    SkyParams                                         skyParams;
    SkyDome                                           skyDome;
    std::unordered_map<std::string, DecodedImage>     skyboxImages;
    bool                                              hasSkybox = false;

    // Fog
    FogParams                                         fogParams;

    // Animated lights
    std::unordered_map<int16_t, LightSource>          lightSources;
    std::unordered_map<int16_t, std::vector<std::pair<uint32_t, int>>> animLightIndex;

    // Objects
    ObjectPropData                                    objData;
    std::vector<int32_t>                              objCellIDs;
    std::unordered_map<std::string, ParsedBinMesh>    parsedModels;
    std::unordered_map<std::string, DecodedImage>     objTexImages;

    // Mission metadata
    SpawnInfo                                         spawnInfo;
    std::string                                       missionName;
    bool                                              texturedMode = false;
};

// ── BuiltMeshes — CPU mesh data between parsing and GPU upload ──
// Intermediate mesh representations built from MissionData, consumed
// during GPU resource creation and kept alive for per-frame draw calls
// (the groups vector drives CellTextureGroup-based rendering).

struct BuiltMeshes {
    LightmappedMesh  lmMesh;
    WorldMesh         worldMesh;
    FlatMesh          flatMesh;
    WorldMesh         waterMesh;
    SkyboxCube        skyboxCube;
    bool              lightmappedMode = false;
    bool              hasWater = false;
};

// ── GPUResources — All bgfx handles ──
// Every GPU object created during initialization. Grouped here so
// cleanup can destroy everything in one pass.

struct GPUResources {
    // Shader programs (5)
    bgfx::ProgramHandle flatProgram                = BGFX_INVALID_HANDLE;
    bgfx::ProgramHandle texturedProgram            = BGFX_INVALID_HANDLE;
    bgfx::ProgramHandle lightmappedProgram         = BGFX_INVALID_HANDLE;
    bgfx::ProgramHandle lightmappedBicubicProgram  = BGFX_INVALID_HANDLE;
    bgfx::ProgramHandle waterProgram               = BGFX_INVALID_HANDLE;

    // Uniforms (8)
    bgfx::UniformHandle s_texColor     = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle s_texLightmap  = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_waterParams  = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_waterFlow    = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_fogColor     = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_fogParams    = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_objectParams = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_lmAtlasSize  = BGFX_INVALID_HANDLE;

    // World geometry
    bgfx::VertexBufferHandle  vbh = BGFX_INVALID_HANDLE;
    bgfx::IndexBufferHandle   ibh = BGFX_INVALID_HANDLE;
    std::unordered_map<uint8_t, bgfx::TextureHandle>       textureHandles;
    std::unordered_map<uint8_t, bgfx::TextureHandle>       flowTextureHandles;

    // Lightmap atlas
    LightmapAtlasSet                                       lmAtlasSet;
    std::vector<bgfx::TextureHandle>                       lightmapAtlasHandles;

    // Water
    bgfx::VertexBufferHandle  waterVBH = BGFX_INVALID_HANDLE;
    bgfx::IndexBufferHandle   waterIBH = BGFX_INVALID_HANDLE;

    // Objects
    std::unordered_map<std::string, ObjectModelGPU>        objModelGPU;
    std::unordered_map<std::string, bgfx::TextureHandle>   objTextureHandles;
    bgfx::VertexBufferHandle  fallbackCubeVBH = BGFX_INVALID_HANDLE;
    bgfx::IndexBufferHandle   fallbackCubeIBH = BGFX_INVALID_HANDLE;
    uint32_t                  fallbackCubeIndexCount = 0;

    // Sky dome
    bgfx::VertexBufferHandle  skyVBH = BGFX_INVALID_HANDLE;
    bgfx::IndexBufferHandle   skyIBH = BGFX_INVALID_HANDLE;
    uint32_t                  skyIndexCount = 0;

    // Skybox
    bgfx::VertexBufferHandle  skyboxVBH = BGFX_INVALID_HANDLE;
    bgfx::IndexBufferHandle   skyboxIBH = BGFX_INVALID_HANDLE;
    std::unordered_map<std::string, bgfx::TextureHandle>   skyboxTexHandles;
};

// ── RuntimeState — Mutable per-frame state ──
// Camera, debug toggles, runtime config, and anything that changes
// frame-to-frame during the render loop.

struct RuntimeState {
    Camera cam;
    float  moveSpeed = 20.0f;
    float  spawnX = 0, spawnY = 0, spawnZ = 0, spawnYaw = 0;
    float  waterElapsed = 0.0f;
    uint32_t skyClearColor = 0x1a1a2eFF;
    bool   running = true;

    // Debug/isolation
    int  isolateModelIdx = -1;
    bool showRaycast = false;
    std::vector<std::string> sortedModelNames;
    std::unordered_map<std::string, int> modelInstanceCounts;

    // Mutable config (toggled at runtime via keys/console)
    bool  showObjects = true;
    bool  showFallbackCubes = false;
    bool  portalCulling = true;
    bool  cameraCollision = false;
    int   filterMode = 0;
    int   lightmapFiltering = 0;  // 0=bilinear (default), 1=bicubic
    float waveAmplitude = 1.0f;
    float uvDistortion = 1.0f;
    float waterRotation = 1.0f;
    float waterScrollSpeed = 1.0f;

    // Render mode flags (set during init, read in loop)
    bool lightmappedMode = false;
    bool texturedMode = false;
    bool hasWater = false;
    bool hasSkybox = false;

    // Physics simulation (created after WR data is parsed, nullptr until then)
    // In physics mode the camera position is driven by PlayerPhysics::getEyePosition().
    // In fly mode, old noclip/clip behavior is preserved.
    std::unique_ptr<DarkPhysics> physics;
    bool physicsMode = true;  // physics (walk) mode on by default; BS+P toggles to fly

    // Mode description string for title bar (points to string literal)
    const char *modeStr = "flat-shaded";

    // Portal culling stats (for title bar)
    uint32_t cullVisibleCells = 0;
    uint32_t cullTotalCells = 0;
};

// ── FrameContext — Per-frame computed values ──
// Returned by prepareFrame(), consumed by each render pass within
// a single frame. Not preserved across frames.

struct FrameContext {
    float proj[16];
    float view[16];
    float skyView[16];
    float fogColorArr[4];
    float fogOnArr[4];
    float fogOffArr[4];
    float *skyFogArr = nullptr;   // points to fogOnArr or fogOffArr
    uint32_t texSampler = 0;
    uint32_t skySampler = 0;
    bool underwater = false;
    bool skyFogged = false;
    std::unordered_set<uint32_t> visibleCells;
    uint64_t renderState = 0;     // BGFX_STATE flags for opaque geometry
};

} // namespace Darkness
