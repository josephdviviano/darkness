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
#include <cstdio>
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
#include "RenderParamsParser.h"
#include "ObjectIllumination.h"
#include "DynamicLightList.h"
#include "physics/DarkPhysics.h"

namespace Darkness {

// Forward declarations
class ObjectStateMap;
class DoorSystem;
class MovingTerrainSystem;
class FrobSystem;
class GrabSystem;
class MessageDispatch;
class ScriptManager;

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

    // Per-mission ambient + sun lighting parameters (from RENDPARAMS chunk).
    RenderParams                                      renderParams;

    // Animated lights
    std::unordered_map<int16_t, LightSource>          lightSources;
    std::unordered_map<int16_t, std::vector<std::pair<uint32_t, int>>> animLightIndex;
    // Map from animated lightnum to its slot in WRParsedData::staticLights.
    // Built at load by position-matching LightSource.pos against
    // staticLights[i].loc. Lets the per-frame anim update modulate the
    // per-object lighting path (so torches blinking off dim nearby objects).
    std::unordered_map<int16_t, int32_t>              animLightToStaticIdx;

    // Objects
    ObjectPropData                                    objData;
    std::vector<int32_t>                              objCellIDs;
    std::unordered_map<std::string, ParsedBinMesh>    parsedModels;
    std::unordered_map<std::string, DecodedImage>     objTexImages;

    // Per-texture terrain friction (indexed by TXLIST texture index, default 1.0).
    // Built from P$Friction on texture archetype objects in dark.gam.
    std::vector<float>                                frictionTable;

    // Cell→room mapping for door visibility blocking (indexed by cellID, -1 if unmapped)
    std::vector<int32_t>                              cellToRoom;

    // Per-texture climbability factor (indexed by TXLIST texture index, default 0.0).
    // Built from P$Climbabil on texture archetype objects in dark.gam.
    // Non-zero values boost friction on steep surfaces (NOT a climb-mode trigger).
    std::vector<float>                                climbabilityTable;

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
    // Shader programs
    bgfx::ProgramHandle flatProgram                = BGFX_INVALID_HANDLE;
    bgfx::ProgramHandle texturedProgram            = BGFX_INVALID_HANDLE;
    bgfx::ProgramHandle lightmappedProgram         = BGFX_INVALID_HANDLE;
    bgfx::ProgramHandle lightmappedBicubicProgram  = BGFX_INVALID_HANDLE;
    bgfx::ProgramHandle waterProgram               = BGFX_INVALID_HANDLE;
    // Per-vertex Lambertian object programs — alternative to the scalar
    // texturedProgram / flatProgram pair when per-vertex shading is on.
    bgfx::ProgramHandle texturedPerVertexProgram   = BGFX_INVALID_HANDLE;
    bgfx::ProgramHandle basicPerVertexProgram      = BGFX_INVALID_HANDLE;

    // Uniforms
    bgfx::UniformHandle s_texColor     = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle s_texLightmap  = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_waterParams  = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_waterFlow    = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_fogColor     = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_fogParams    = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_objectParams = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_objectLight  = BGFX_INVALID_HANDLE;  // .rgb = per-object lighting tint (scalar path)
    bgfx::UniformHandle u_lmAtlasSize  = BGFX_INVALID_HANDLE;

    // Per-object light array uniforms (per-vertex path). Sized at
    // creation by kObjectLightCap (single source of truth in
    // shaders/object_lighting_constants.h, mirrored to C++ in
    // ObjectIllumination.h).
    bgfx::UniformHandle u_objectLightCount  = BGFX_INVALID_HANDLE;  // .x = count
    bgfx::UniformHandle u_objectAmbient     = BGFX_INVALID_HANDLE;  // .rgb = ambient + ExtraLight
    bgfx::UniformHandle u_objectLightLoc    = BGFX_INVALID_HANDLE;  // vec4 × cap
    bgfx::UniformHandle u_objectLightDir    = BGFX_INVALID_HANDLE;  // vec4 × cap
    bgfx::UniformHandle u_objectLightBright = BGFX_INVALID_HANDLE;  // vec4 × cap

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
    bool   windowFocused = true;  // false when Command+Tab or otherwise backgrounded

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
    bool  debugAnimLightmaps = false; // tint animated lightmap polys magenta
    bool  forceFlicker = false;       // override all animated lights to flicker mode
    float waveAmplitude = 1.0f;
    float uvDistortion = 1.0f;
    float waterRotation = 1.0f;
    float waterScrollSpeed = 1.0f;

    // Render mode flags (set during init, read in loop)
    bool lightmappedMode = false;
    bool texturedMode = false;
    bool hasWater = false;
    bool hasSkybox = false;

    // Per-object lighting (Dark Engine convention: ambient + cell-light-list
    // sum + sun + P$ExtraLight). When disabled, objects render at their
    // material color (the historical behavior). Toggle via debug console.
    //
    // Marked `mutable` because the illuminator carries an internal shadow
    // cache that compute() updates as a transparent optimization; the render
    // pipeline passes RuntimeState by const ref everywhere, so this lets the
    // const draw lambdas still poke the cache.
    mutable ObjectIlluminator objectIlluminator;
    bool objectLightingEnabled = true;

    // Per-vertex Lambertian shading for objects. When true, the renderer
    // uploads the cell's visible light array to the GPU and the vertex
    // shader does cos(angle)/dist per vertex; when false, the scalar
    // tint path runs (one RGB value uniform across the whole object).
    // Per-vertex produces front/back contrast for objects near point
    // lights (an AI walking past a torch); the scalar path is flatter
    // but cheaper and faithful to the original engine's CLUT path.
    bool perVertexObjectLightingEnabled = true;

    // Scratch buffer for buildLightArray() output. Held as state to
    // avoid reallocating ~96 vec4s every per-object draw call. Not
    // thread-safe; the renderer is single-threaded.
    mutable GPULightArray gpuLightScratch;

    // Per-frame dynamic light registry. Reset at the start of each render
    // frame; gameplay systems push transient lights (player flashlight,
    // fire arrows, mage spells, creature glow) before object draws begin.
    DynamicLightList dynamicLights;

    // Physics simulation (created after WR data is parsed, nullptr until then)
    // In physics mode the camera position is driven by PlayerPhysics::getEyePosition().
    // In fly mode, old noclip/clip behavior is preserved.
    std::unique_ptr<DarkPhysics> physics;
    bool physicsMode = true;  // physics (walk) mode on by default; BS+P toggles to fly
    bool crouchToggled = false;  // crouch toggle state (C key toggles on/off)

    // Mutable object state — non-owning pointer to the ObjectStateMap owned
    // by ObjSysWorldState. Game systems (doors, tweqs, platforms) write here;
    // the renderer reads it to override static P$Position transforms.
    ObjectStateMap *objectStates = nullptr;

    // Door system — non-owning pointer, for debug interaction (G key toggles nearest door)
    DoorSystem *doorSystem = nullptr;

    // Moving terrain system — non-owning pointer, for platform velocity queries
    MovingTerrainSystem *movingTerrainSystem = nullptr;

    // Frob system — non-owning pointer, for player interaction with objects
    FrobSystem *frobSystem = nullptr;

    // Grab system — non-owning pointer, for held-object physics manipulation
    // (carry / drop / throw). Decoupled from FrobSystem so future grip modes
    // (e.g. Amnesia-style hinge drag) can extend without touching frob.
    GrabSystem *grabSystem = nullptr;

    // Message dispatch — non-owning pointer, for routing frob/TurnOn/TurnOff
    // through global handlers (which enforce lock checks, etc.)
    MessageDispatch *messageDispatch = nullptr;
    ScriptManager *scriptManager = nullptr;

    // Mode description string for title bar (points to string literal)
    const char *modeStr = "flat-shaded";

    // Frob highlight (Dark Engine convention: additive brightness boost)
    // Fades in/out over ~130ms matching the original engine.
    int32_t frobHighlightObjID = 0;   // currently highlighted object (0 = none)
    float frobHighlightLevel = 0.0f;  // current highlight intensity (0.0-0.47)

    // Portal culling stats (for title bar)
    uint32_t cullVisibleCells = 0;
    uint32_t cullTotalCells = 0;

    // Probe baking state (auto-bake on first run)
    bool probeBakeNeeded = false;
    std::string probeBakePath;

    // ── Head/viewport log (per-render-frame CSV) ──
    // Records the actual camera state at display rate, complementary to the
    // per-fixed-step physics_log. Used for diagnosing head-bob smoothness and
    // animation snap that only manifest at the render rate (between physics
    // steps). Enabled via --head-log <path> or the head_log console toggle.
    FILE    *headLog = nullptr;
    uint64_t headLogStartUs   = 0;      // monotonic epoch for relative wall time
    uint64_t headLogPrevUs    = 0;      // previous-frame timestamp (for frameDt)
    uint64_t headLogPrevSteps = 0;      // previous-frame PlayerPhysics step count
    uint64_t headLogPrevPPF   = 0;      // previous-frame PPF cancellation count
    uint32_t headLogFlushCtr  = 0;      // flush every N rows

    // ── View-punch displacement bookkeeping ──
    // mPunchAngle from PlayerPhysics is an absolute offset from neutral (the
    // current spring displacement), not a per-frame delta. Adding it to
    // state.cam.pitch every render frame would integrate the offset and run
    // pitch off to infinity — we have to subtract last frame's contribution
    // before adding this frame's. Roll has no equivalent because cam.roll is
    // overwritten each frame by getLeanTilt() before the punch is applied.
    float lastAppliedPunchPitch = 0.0f;

    // Debug: acoustic mesh wireframe overlay
    bool showAcousticMesh = false;
    std::vector<float> acousticVerts;     // x,y,z flat array
    std::vector<int32_t> acousticIndices; // triangle indices
    bgfx::VertexBufferHandle acousticVBH = BGFX_INVALID_HANDLE;
    bgfx::IndexBufferHandle  acousticIBH = BGFX_INVALID_HANDLE;
    uint32_t acousticLineCount = 0;

    // Debug: acoustic probe overlay — scatter a colored cube at every baked
    // probe position (queried from AudioService) and flash a marker at the
    // listener while any player-emitted voice (footstep / land) is sounding.
    // Single toggle: the marker only makes sense alongside the probe grid,
    // since the whole point is correlating listener-to-probe distance with
    // perceived footstep loudness.
    bool  showProbes        = false;
    float probeMarkerSize   = 1.0f;   // half-extent of the cube in feet

    // Debug: room/portal wireframe overlay. Per-room edges are computed at
    // startup from the room's 6 bounding planes and the portal-edge planes
    // (see RoomDebugViz.h). Colors are deterministic per-room-ID so adjacent
    // rooms read as visually distinct.
    bool showRooms = false;
    // Debug: camera world-space position readout on the HUD. Off by
    // default; toggle via `show_pos` in the debug console. Useful when
    // navigating to a specific coordinate (e.g. for audio-propagation
    // verification at a known room/portal boundary).
    bool showPos = false;
    // Debug: portal polygons rendered in light pink, independent of the
    // full room/portal wireframe (`show_rooms`). Off by default; toggle
    // via `show_portals`. Lets you trace exactly which polygons the BFS
    // path is threading — small doorways vs large open atria are
    // immediately visible. Drawn always-on-top with alpha blending.
    bool showPortals = false;
    // Debug: per-voice spatial visualization. Draws an arrow from each
    // active voice's source position to its virtualPosition (where the
    // propagation system tells Steam Audio the sound is coming from),
    // then a second segment to the listener. Off by default; toggle via
    // `show_vpos`. Lets you watch the BFS anchor jump frame-to-frame
    // and correlate it with audible volume/direction changes.
    bool showVPos = false;

    // One entry per labelled point per room. We push the room's center
    // (used to flag the listener's current room with a "*" suffix) plus
    // every polytope corner (so wherever you look at a wireframe, a
    // nearby label tells you which room ID that wireframe belongs to —
    // essential for manually auditing overlapping subdivisions of a
    // single visual room).
    struct RoomLabel {
        Vector3 pos;
        int16_t roomID;
        bool    isCenter;   // true → primary label, eligible for "*" current marker
    };

    // Per-room CPU-side debug geometry. Built once at level load (the
    // room database is static for the life of the mission) and consumed
    // each frame by the closest-N cull in renderDebugOverlay /
    // renderRoomLabelsOverlay. Splitting by room (rather than baking
    // everything into one big VBH up front) lets the renderer keep
    // visual clutter down by only emitting the rooms nearest the camera.
    struct PerRoomDebug {
        Vector3 center;                          // for distance sort
        int16_t roomID;
        std::vector<PosColorVertex> roomLines;   // colored by room-ID hash
        std::vector<PosColorVertex> portalLines; // uniform light pink
        std::vector<RoomLabel>      labels;      // center + corner labels
    };
    std::vector<PerRoomDebug> roomDebug;

    // Cap on number of rooms drawn into the show_rooms / show_portals
    // overlays. The camera-nearest N rooms (by center distance) are kept;
    // the rest are culled. 0 means "no limit, draw everything". Default
    // 20 is tuned against typical Thief 2 missions where ~200 rooms make
    // an uncapped overlay unreadable. Exposed via `debug_room_max_count`
    // in the debug console.
    int debugRoomMaxCount = 20;
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
    ViewFrustum objFrustum;       // Rendering-FOV frustum for per-object AABB tests
    uint64_t renderState = 0;     // BGFX_STATE flags for opaque geometry
};

} // namespace Darkness
