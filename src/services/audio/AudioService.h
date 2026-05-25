/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2005-2009 openDarkEngine team
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

#ifndef __AUDIOSERVICE_H
#define __AUDIOSERVICE_H

#include "config.h"

#include "DarknessService.h"
#include "DarknessServiceFactory.h"
#include "ServiceCommon.h"
#include "SharedPtr.h"
#include "DarknessMath.h"
#include "database/DatabaseCommon.h"
#include "loop/LoopCommon.h"
#include "audio/AudioDSPChain.h"
#include "audio/AudioUnits.h"
#include "audio/SchemaTypes.h"
#include "room/RoomService.h"  // SoundPropInfo / SoundPropParams / SoundPathHop
#include "worldquery/WorldQueryTypes.h"  // RayHit (for diagnostic raycaster setter)

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Forward declarations for audio backend types (defined in .cpp via C headers)
struct ma_engine;
struct ma_device;
// Steam Audio uses DECLARE_OPAQUE_HANDLE: typedef struct _IPLFoo_t* IPLFoo
struct _IPLContext_t;
typedef _IPLContext_t* IPLContext;
struct _IPLHRTF_t;
typedef _IPLHRTF_t* IPLHRTF;
struct _IPLScene_t;
typedef _IPLScene_t* IPLScene;
struct _IPLStaticMesh_t;
typedef _IPLStaticMesh_t* IPLStaticMesh;
struct _IPLInstancedMesh_t;
typedef _IPLInstancedMesh_t* IPLInstancedMesh;
struct _IPLSimulator_t;
typedef _IPLSimulator_t* IPLSimulator;
struct _IPLReflectionMixer_t;
typedef _IPLReflectionMixer_t* IPLReflectionMixer;
struct _IPLAmbisonicsDecodeEffect_t;
typedef _IPLAmbisonicsDecodeEffect_t* IPLAmbisonicsDecodeEffect;
struct _IPLSource_t;
typedef _IPLSource_t* IPLSource;
struct _IPLProbeBatch_t;
typedef _IPLProbeBatch_t* IPLProbeBatch;
struct _IPLPathEffect_t;
typedef _IPLPathEffect_t* IPLPathEffect;

// Forward declarations for the reflection-sim / convolution-pool subsystems
// (defined in ReflectionSimulator.h / ConvolutionWorkerPool.h respectively —
// included from AudioService.cpp, not this header, to keep the include
// footprint narrow).

namespace Darkness {

// Forward declarations for room system types
class Room;
class RoomPortal;

// Probe-bake plumbing structs (defined in ProbeManager.h). Forward-
// declared here so prepareProbeBakeParams / computeProbePlan can name
// them in their signatures without dragging ProbeManager.h's IPL
// forward decls into this header. Full include lives in AudioService.cpp.
struct ProbeBakeParams;
struct ProbeBakePlan;

// Owned subsystem types — full definitions live in their own headers.
class ReflectionSimulator;
class PathingSimulator;
class ConvolutionWorkerPool;
struct ConvolutionWorker;

// Forward declarations for sound resource types (defined in CRFSoundLoader.h)
class CRFSoundLoader;
class SoundCache;
struct SoundData;
class SchemaParser;

// Forward declaration — defined in AmbientSoundManager.h
class AmbientSoundManager;

/// Data transfer struct for world geometry → Steam Audio acoustic scene.
/// Filled by the render binary from WR chunk data, passed to AudioService.
struct AcousticSceneData {
    /// World-space vertex positions (3 floats per vertex: x, y, z)
    std::vector<float> vertices;
    /// Triangle indices (3 per triangle, indexing into vertices)
    std::vector<int32_t> indices;
    /// Per-triangle texture name from TXLIST (family/name path, for material lookup)
    std::vector<std::string> texNames;
};

// Forward declarations for AI hearing data structs (defined in audio/AIHearingData.h)
struct AIHearingStats;
struct AISoundTweaks;

// Forward declaration — defined in sim/DoorSystem.h. The audio service consumes
// these to register door geometry with Steam Audio's acoustic scene so closed
// doors block sound via geometry-aware path validation.
struct DoorAudioGeometry;

/// Event published by AudioService whenever a sound is emitted into the
/// world. Listeners (AI hearing subsystem, debug overlays, recording
/// systems) subscribe via registerSoundEmissionListener(). Carries the
/// minimum metadata needed for AI awareness gating; the underlying voice
/// is not exposed (callers should treat this as fire-and-forget).
struct SoundEmissionEvent {
    int32_t     emitterObjID;   ///< Who emitted (-1 for player / world-anchored)
    Vector3     position;       ///< World-space emit position
    std::string schemaName;     ///< Schema name as defined in .sch files
    int32_t     soundType;      ///< AI sound type slot (0=Untyped..5=Combat)
    float       baseRange;      ///< Unmodified audible range in world units
    float       gainDb;         ///< Base loudness in centibels (negative = quieter)
};

/// Listener callback signature for sound-emission notifications.
using SoundEmissionListener = std::function<void(const SoundEmissionEvent &)>;

/// Handle to an active sound (returned by play functions, used to halt/query)
using SoundHandle = int32_t;

/// Invalid sound handle sentinel
constexpr SoundHandle SOUND_HANDLE_INVALID = -1;

/// Active voice — owns WAV data, miniaudio decoder + sound (defined in AudioService.cpp)
struct ActiveVoice;

/// Debug snapshot of one voice's spatial state — what the propagation
/// system thinks "where" the voice is. Used by the renderer's
/// show_vpos overlay to draw per-voice path arrows for diagnosis of
/// audible jumps. Captured on the main thread from the cached prop
/// result; no audio-thread synchronisation needed.
struct VoiceSpatialSnapshot {
    std::string schemaName;       ///< schema that spawned this voice
    Vector3     sourcePos;        ///< actual world position
    Vector3     virtualPos;       ///< prop.virtualPosition (centroid — where Steam Audio's *reflection-send* sees the source; with multi-path the per-slot direct path uses each path's own virtualPosition)
    bool        reached;          ///< prop.reached
    bool        usePortal;        ///< true when cross-room (virtualPos != sourcePos)
    bool        isAmbient;        ///< true for AMB_ENVIRONMENTAL ambients
    /// Per-portal anchor bend points along the primary path, in
    /// source→listener order. Empty for same-room / clean-threaded
    /// paths. The renderer connects these with line segments
    /// (source → bend₀ → bend₁ → … → bendₙ → listener) to show what
    /// the propagation algorithm thinks the sound's geometric path
    /// actually is.
    std::vector<Vector3> chain;
    /// One entry per ACTIVE propagation path (Phase 4 multi-path:
    /// the audio renderer mixes up to kMaxSubSources of these
    /// simultaneously). `endpoint` is the path's last anchor (the
    /// per-slot virtualPosition); `chain` is the bend chain that
    /// got the sound from source to that endpoint (empty for
    /// clean-threaded / same-room paths). The renderer overlays
    /// these in distinct colors per slot so the visualisation
    /// matches what's actually summed into the audio bus —
    /// otherwise the user sees only the primary path's chain (the
    /// single `chain` field above) and can't tell what the other
    /// slots are doing.
    struct PathSnapshot {
        Vector3              endpoint;
        std::vector<Vector3> chain;
        /// Propagation backend that produced this path. show_vpos
        /// renders cell-graph chains as solid lines and (when the
        /// Phase-2 Steam Audio integration lands) Steam Audio chains
        /// as dashed lines.
        PropagationBackend   backend = PropagationBackend::RoomGraph;
    };
    std::vector<PathSnapshot> paths;
    /// Per-voice divergence-overlay data. `primaryEffectiveDistance`
    /// is the cell-graph result driving audio; `secondaryEffectiveDistance`
    /// is the alternate backend's result when available (Phase 2). In
    /// Phase 1 secondaryEffectiveDistance is always 0 and
    /// hasSecondaryBackend is false — the overlay code reads these
    /// fields and skips the dB-delta colour-coding when no comparison
    /// is available.
    float primaryEffectiveDistance   = 0.0f;
    float secondaryEffectiveDistance = 0.0f;
    bool  hasSecondaryBackend        = false;
};

/// Voice classification, set at startVoice time so the per-class default
/// directParams (distanceAttenuation = 1.0, occlusion-disabled flags etc.)
/// can be applied inside createVoiceSource — i.e. before ma_sound_start
/// kicks off audio-thread playback. Without this, the audio thread reads
/// the silent initVoiceDSP defaults for the first ~21 ms callback after
/// voice creation, since the per-frame loopStep override doesn't fire
/// until the next frame.
enum class VoiceClass {
    Normal,         ///< NPC dialogue, world FX, gunshots, etc.
    PlayerEmitted,  ///< Footsteps, landings — source ≈ listener; no occlusion / distance
    Ambient,        ///< Environmental ambients — distance handled by ambient system
};

/// On-disk layout of P$AmbientHack property (60 bytes).
/// Ambient sound objects in the mission use this to define looping environmental sounds.
#pragma pack(push, 1)
struct PropAmbientHack {
    int32_t radius;           ///< Propagation radius in world units
    int32_t volume;           ///< Volume (millibels, same as schema volume)
    uint32_t flags;           ///< AmbientHackFlags bitfield
    char schema[16];          ///< Schema name (null-terminated)
    char aux1[16];            ///< Auxiliary data (unused)
    char aux2[16];            ///< Auxiliary data (unused)
};
#pragma pack(pop)

/// AmbientHackedFlags bitfield values
enum AmbientHackFlags : uint32_t {
    AMB_ENVIRONMENTAL = 0x01, ///< Room-based environmental ambient
    AMB_NO_SHARP_CURVE = 0x02,///< Smooth volume falloff
    AMB_TURNED_OFF = 0x04,    ///< Initially disabled
    AMB_ONCE_ONLY = 0x08,     ///< Play once then stop
    AMB_MUSIC = 0x10,         ///< Music track (separate volume control)
    AMB_SYNCH = 0x20,         ///< Synchronize with other ambients
    AMB_NO_FADE = 0x40,       ///< No fade in/out
    AMB_DESTROY_OBJ = 0x80,   ///< Destroy object after playing
    AMB_AUTO_OFF = 0x100,     ///< Auto-disable when out of range
};

/// AmbientSound is owned by AmbientSoundManager (see AmbientSoundManager.h).
/// The manager is responsible for parsing P$AmbientHack and per-frame voice
/// lifecycle. Loudness shaping is delegated to Steam Audio's per-voice DSP
/// chain. (P$SpotAmb was previously mis-loaded here as a second ambient
/// encoding; it's actually a renderer SpotlightAndAmbient property — see
/// PropSpotlightAndAmbient in DarkPropertyDefs.h.)

/// Maximum simultaneous active voices (matches Dark Engine's limit)
constexpr int MAX_ACTIVE_VOICES = 64;

/// Default maximum voices with reverb (convolution) enabled simultaneously.
/// Per-voice convolution is expensive. Remaining voices use direct path only
/// (HRTF + distance attenuation + occlusion), which is cheap.
/// Tunable at runtime via setReverbVoices().
constexpr int DEFAULT_REVERB_VOICES = 16;

/// Default maximum sound propagation distance (world units)
constexpr float SOUND_MAX_DIST = 200.0f;

/// SoundPropInfo / SoundPathHop / SoundPropParams are declared in
/// room/RoomService.h — they describe portal-graph propagation, a
/// room-system concern that AudioService consumes.

/** @brief Audio service — manages all sound playback, schema resolution,
 *  Steam Audio spatialization, and AI sound propagation.
 *
 *  Lifecycle:
 *    init()             — acquire service dependencies
 *    bootstrapFinished() — register as DatabaseListener + LoopClient
 *    onDBLoad()         — load schemas, build acoustic scene
 *    loopStep()         — per-frame voice management + simulation
 *    onDBDrop()         — release mission audio state
 *    shutdown()         — unregister listeners, release backends
 */
class AudioService : public ServiceImpl<AudioService>,
                     public DatabaseListener,
                     public LoopClient {
    // AmbientSoundManager is an extracted subsystem that owns ambient
    // lifecycle (P$AmbientHack). It needs to reach this service's
    // private state (mVoicePool, mReflectionMixNode,
    // mSchemaParser, startVoice, haltSound, publishSoundEmission,
    // mListenerPos) through its back-pointer — befriending keeps that
    // access narrow and avoids widening the public API.
    friend class AmbientSoundManager;

public:
    /// Forward declaration — defined in AudioService.cpp (needs public access
    /// for the free-function audio-thread callback reflectionMixNodeProcess)
    struct ReflectionMixNode;

    AudioService(ServiceManager *manager, const std::string &name);
    ~AudioService() override;

    // ── Public API (stubs — wired up in later tasks) ──

    /** Play a schema by name at a world position (one-shot or looping).
     *  @param schemaName  Schema name (as defined in .sch files)
     *  @param position    World-space position
     *  @return Sound handle, or SOUND_HANDLE_INVALID on failure */
    SoundHandle playSchema(const std::string &schemaName,
                           const Vector3 &position);

    /** Play a schema attached to a game object (tracks object position).
     *  @param schemaName  Schema name
     *  @param objID       Object ID to attach to
     *  @return Sound handle, or SOUND_HANDLE_INVALID on failure */
    SoundHandle playSchemaOnObj(const std::string &schemaName, int objID);

    /** Play a sound matching environment tags at a world position.
     *  General-purpose schema query: finds schemas whose env_tags match the
     *  provided tag set, selects a sample, and plays it at the given position.
     *  Used by doors, NPCs, traps, and any system that triggers sounds via
     *  the Dark Engine's env_tag matching system.
     *  @param tags     Environment tags to match (e.g., Event=StateChange, OpenState=Opening)
     *  @param position World-space position for 3D audio
     *  @return Sound handle, or SOUND_HANDLE_INVALID if no matching schema found */
    SoundHandle playEnvSchema(const std::vector<SchemaTagValue> &tags,
                              const Vector3 &position,
                              bool bypassPortalBlocking = false);

    /** Halt a specific active sound.
     *  @param handle  Sound handle from playSchema/playSchemaOnObj
     *  @param fadeMs  Fade-out duration in milliseconds. Default 15 ms is the
     *                 minimum needed to suppress click/pop from an abrupt
     *                 cut. Use a longer fade (~200 ms) when halting an
     *                 ambient that has gracefully aged out of audible range
     *                 so the voice fades out rather than cuts. */
    void haltSound(SoundHandle handle, int fadeMs = 15);

    /** Halt all active sounds (e.g. on mission unload). */
    void haltAll();

    /** Set the portal blocking factor between two rooms.
     *  Used by door open/close logic to control AI sound propagation.
     *  @param room1   First room ID
     *  @param room2   Second room ID
     *  @param factor  0.0 = fully open, 1.0 = fully blocked */
    void setBlockingFactor(int room1, int room2, float factor);

    /** Get the portal blocking factor between two rooms.
     *  @return Blocking factor (0.0–1.0), or 0.0 if not set */
    float getBlockingFactor(int room1, int room2) const;

    /** Register every door's audio geometry with the acoustic scene.
     *  Builds one `IPLStaticMesh` + `IPLInstancedMesh` per door (mesh in the
     *  door's model-local frame, instance at the door's current world pose)
     *  so Steam Audio's geometry-aware pathing-validation can reject baked
     *  path edges that pass through closed doors.
     *
     *  Must be called AFTER `buildAcousticScene` (mIplScene must exist) and
     *  AFTER DoorSystem::init has populated door state. Doors with zero
     *  edgeLengths are skipped at the DoorSystem side; this call expects a
     *  pre-filtered list. Subsequent transform updates flow through
     *  `setDoorTransform`. */
    void registerDoorGeometry(const std::vector<DoorAudioGeometry> &doors);

    /** Push a door's current world transform into the acoustic scene. The
     *  transform should be the door's pose WITHOUT scale baked in — the
     *  door's audio static mesh already encodes its final world dimensions
     *  in its vertex coordinates (see DoorAudioGeometry). Setting a
     *  transform marks the scene as needing a commit; the commit is
     *  coalesced once per loopStep after reflection-sim is drained. */
    void setDoorTransform(int32_t doorObjID, const Matrix4 &worldTransform);

    /** Debug overlay accessor — returns world-space triangle data for all
     *  registered doors. Used by the renderer's `show_door_geometry`
     *  wireframe overlay to visualize geometry the acoustic scene sees
     *  beyond the static BSP mesh. Each `DoorMesh` packs vertices already
     *  transformed into world space (engine feet, ready for direct GPU
     *  upload) plus the index list. Recomputed live — door pose changes
     *  reach the overlay on the next frame. Returns an empty vector when
     *  no doors are registered or the acoustic scene isn't built. */
    struct DebugDoorMesh {
        int32_t                objID = 0;
        std::vector<float>     worldVertices;  ///< flat float3 array, engine feet
        std::vector<uint32_t>  indices;
    };
    std::vector<DebugDoorMesh> getDoorGeometryForDebug() const;

    /** Load sound resources from a Thief 2 RES directory.
     *  Opens snd.crf and prepares the sound cache.
     *  Called from DarknessRender.cpp after services are bootstrapped.
     *  @param resPath     Path to Thief 2 RES directory (containing snd.crf)
     *  @param schemasPath Optional explicit path to schema files directory.
     *                     If empty, searches EDITOR/SCHEMA and EDITOR/schemas
     *                     relative to the RES directory's parent.
     *  @return true if snd.crf opened successfully */
    bool loadSoundResources(const std::string &resPath,
                            const std::string &schemasPath = "");

    /** Build Steam Audio acoustic scene from world geometry.
     *  Called from the render binary after WR chunks are parsed.
     *  Creates IPLScene + IPLStaticMesh + IPLSimulator for ray-traced acoustics.
     *  @param data  World geometry — vertices, triangle indices, per-triangle texture names
     *  @return true if scene built successfully */
    bool buildAcousticScene(const AcousticSceneData &data);


    /** Update the listener (player camera) position and orientation.
     *  Called each frame from the render binary after camera/physics update.
     *  @param pos    World-space listener position
     *  @param yaw    Heading rotation (radians, counterclockwise from +X in Z-up)
     *  @param pitch  Tilt rotation (radians, positive = looking up) */
    void setListenerTransform(const Vector3 &pos, float yaw, float pitch);

    /** Propagate sound from source to listener through the portal graph.
     *  Uses Dijkstra-style BFS through room portals, applying per-portal
     *  blocking factors (from doors) to compute effective distance.
     *  @param sourcePos    World-space sound source position
     *  @param listenerPos  World-space listener/AI position
     *  @param maxDist      Maximum propagation distance (default 200 world units)
     *  @return Propagation info (reached=false if sound can't reach listener) */
    SoundPropInfo propagateSound(const Vector3 &sourcePos,
                                 const Vector3 &listenerPos,
                                 float maxDist = SOUND_MAX_DIST) const;

    /** Set the per-texture material keyword table (indexed by TXLIST texture index).
     *  Built from world texture names via AcousticMaterials keyword matching.
     *  Called from the render binary after TXLIST is parsed.
     *  @param materials  Per-texture material keywords (e.g., "stone", "metal", "wood") */
    void setTextureMaterials(std::vector<std::string> materials);

    /** Install a BSP-aware line-of-sight callback for sound propagation.
     *  The callback must return true iff the segment a→b is unobstructed
     *  by world geometry. When provided, propagateSoundPath runs per-bend
     *  BSP validation during chain reconstruction: a blocked segment
     *  triggers a search for an alternate bend position on the same
     *  portal polygon, and a path that can't be made clear is dropped.
     *
     *  Renderer-supplied (BSP data lives in the renderer). Call once
     *  after the service is bootstrapped. */
    void setSoundPathLineOfSightFn(
        std::function<bool(const Vector3 &a, const Vector3 &b)> fn);

    /** Diagnostic-only raycaster: returns true on hit, populates `hit`.
     *
     *  Used by the [PATH] periodic|spike diagnostic to answer "does our own
     *  BSP raycaster agree that the line from a SPIKE'd voice to its nearest
     *  probe is blocked?" — distinguishes "source in solid / wall in the
     *  way" from "Steam Audio mesh interpretation differs from ours". Not
     *  on any hot path; only fires inside an already-gated log block.
     *
     *  Renderer-supplied (WR data lives in the renderer). Call once after
     *  the service is bootstrapped. Safe to leave unset — the diagnostic
     *  reports -1 in that case. */
    using AudioRaycastFn = std::function<bool(const Vector3 &from,
                                              const Vector3 &to,
                                              RayHit &hit)>;
    void setRaycaster(AudioRaycastFn fn) { mRaycaster = std::move(fn); }

    /** Play a footstep sound for a specific material and speed.
     *  Selects a schema via env_tag matching: (Event Footstep) + (Material <keyword>).
     *  When in water, overrides material with water-specific schema tags.
     *  Called from the physics footstep callback.
     *  @param pos         World-space foot position
     *  @param speed       Horizontal movement speed (world units/sec)
     *  @param textureIdx  Ground texture index (TXLIST) for material lookup */
    void playFootstep(const Vector3 &pos, float speed, int textureIdx);

    /** Play a landing impact sound for the given material and fall speed.
     *  Selects a schema via env_tag matching: (Event Footstep) (Landing True)
     *  (CreatureType Player) (Material <keyword>).
     *  Volume scales with fall velocity — harder falls are louder.
     *  @param pos         World-space foot position
     *  @param fallSpeed   Downward velocity at impact (positive, world units/sec)
     *  @param textureIdx  Ground texture index (TXLIST) for material lookup */
    void playLanding(const Vector3 &pos, float fallSpeed, int textureIdx);

    /** Set the player's water state for footstep material override.
     *  When in water, footsteps use water splash schemas instead of ground material.
     *  @param inWater  true if the player's feet are submerged */
    void setPlayerInWater(bool inWater) { mPlayerInWater = inWater; }

    /** Toggle convolution reflections on/off (for A/B comparison). */
    void setReflectionsEnabled(bool enabled) { mReflectionsEnabled = enabled; }
    bool getReflectionsEnabled() const { return mReflectionsEnabled; }

    // ── Tunable reflection parameters (exposed for console binding) ──
    //
    // Realtime params drive the per-frame `iplSimulatorRunReflections` step
    // and the per-voice `iplReflectionEffectApply` IR length. Bake params
    // drive the offline `iplReflectionsBakerBake` pass and are persisted into
    // the .probes cache file — bake changes require a re-bake.

    /// Reflection effect algorithm: HYBRID-only. The pipeline always runs
    /// Steam Audio's `IPL_REFLECTIONEFFECTTYPE_HYBRID`: an early convolution
    /// head (length = `hybridTransitionTime`) followed by a parametric tail
    /// driven by `reverbTimes[3]` (RT60) baked into the probe data — these
    /// correspond to the game's EAX-style reverb presets. Parametric tails
    /// cannot beat from per-frame IR crossfade stacking, which is the failure
    /// mode targeted by PLAN.HYBRID_REVERB.md. CONVOLUTION and PARAMETRIC
    /// modes were removed once HYBRID had matched both perceptually with
    /// strictly less CPU. (Matches Steam Audio's Unity/Unreal integrations,
    /// which also ship HYBRID-only at a 1.0 s default head length.)

    /// Length (seconds) of the convolution portion of the IR. Must be < the
    /// IR duration — Steam Audio's `HybridReverbEstimator` crashes otherwise.
    /// Default 1.0 s matches the Unity/Unreal reference integrations.
    void  setHybridTransitionTime(float s) { mHybridTransitionTime = std::max(0.1f, std::min(s, 8.0f)); }
    float getHybridTransitionTime() const  { return mHybridTransitionTime; }

    /// Fraction of `hybridTransitionTime` used for the convolution↔
    /// parametric crossfade. Anchored at the END of the transition.
    void  setHybridOverlapPercent(float f) { mHybridOverlapPercent = std::max(0.0f, std::min(f, 1.0f)); }
    float getHybridOverlapPercent() const  { return mHybridOverlapPercent; }

    // Realtime simulation params (used by iplSimulatorRunReflections,
    // iplReflectionEffectCreate's irSize, and the per-frame ambisonics
    // pipeline).
    void setRealtimeNumRays(int n) { mRealtimeNumRays = std::max(128, std::min(n, 8192)); }
    int  getRealtimeNumRays() const { return mRealtimeNumRays; }
    void setRealtimeNumBounces(int n) { mRealtimeNumBounces = std::max(1, std::min(n, 8)); }
    int  getRealtimeNumBounces() const { return mRealtimeNumBounces; }
    void setRealtimeDuration(float d) { mRealtimeDuration = std::max(0.5f, std::min(d, 4.0f)); }
    float getRealtimeDuration() const { return mRealtimeDuration; }
    void setRealtimeDiffuseSamples(int n) { mRealtimeDiffuseSamples = std::max(16, std::min(n, 256)); }
    int  getRealtimeDiffuseSamples() const { return mRealtimeDiffuseSamples; }

    // Offline bake params (used by iplReflectionsBakerBake).
    void setBakeNumRays(int n) { mBakeNumRays = std::max(1024, std::min(n, 65536)); }
    int  getBakeNumRays() const { return mBakeNumRays; }
    void setBakeNumBounces(int n) { mBakeNumBounces = std::max(1, std::min(n, 64)); }
    int  getBakeNumBounces() const { return mBakeNumBounces; }
    void setBakeDuration(float d) { mBakeDuration = std::max(0.5f, std::min(d, 8.0f)); }
    float getBakeDuration() const { return mBakeDuration; }
    void setBakeDiffuseSamples(int n) { mBakeDiffuseSamples = std::max(32, std::min(n, 4096)); }
    int  getBakeDiffuseSamples() const { return mBakeDiffuseSamples; }
    void setBakeAmbisonicsOrder(int n) { mBakeAmbisonicsOrder = std::max(0, std::min(n, 3)); }
    int  getBakeAmbisonicsOrder() const { return mBakeAmbisonicsOrder; }

    void setReflectionThrottle(int n);
    int  getReflectionThrottle() const;

    // [REFLECTIONS] Cap on total reverb voices (realtime + baked combined).
    // Every reverb voice runs a per-source convolution regardless of mode,
    // so this is the worker-pool CPU governor. 0 disables all reverb
    // convolution entirely (fully dry). To run baked-only, keep this
    // positive and set reverb_voices_realtime = 0.
    // Out-of-line because we propagate the new value to the convolution
    // pool's per-worker cap (Fix B, 2026-05-24), and the pool is only
    // forward-declared here.
    void setReverbVoices(int n);
    int  getReverbVoices() const { return mReverbVoices; }

    // [REALTIME] Of the reverb voices above, how many may run with
    // realtime ray-traced IRs. 0 = baked-only (all eligible voices route
    // through baked-probe reverb). Any positive value <= reverb_voices
    // splits the budget: that many realtime slots, remainder for baked.
    void setReverbVoicesRealtime(int n) {
        mReverbVoicesRealtime = std::max(0, std::min(n, MAX_ACTIVE_VOICES));
    }
    int  getReverbVoicesRealtime() const { return mReverbVoicesRealtime; }

    void setTransmissionScale(float s) { mTransmissionScale = std::max(0.1f, std::min(s, 100.0f)); }
    float getTransmissionScale() const { return mTransmissionScale; }

    void setAbsorptionScale(float s) { mAbsorptionScale = std::max(0.01f, std::min(s, 10.0f)); }
    float getAbsorptionScale() const { return mAbsorptionScale; }

    // Occlusion tuning — storage + clamps live in AudioOcclusion.
    // These facades preserve the historical public AudioService API so
    // RenderConfig and DebugConsole keep working unchanged.
    void  setOcclusionRadius(float r);
    float getOcclusionRadius() const;
    void  setOcclusionSamples(int n);
    int   getOcclusionSamples() const;

    /// Get the current listener position (for door sound placement, etc.)
    Vector3 getListenerPos() const { return mListenerPos; }

    /// Debug snapshot of every active voice's spatial state — source
    /// position, virtual position (what Steam Audio sees), and reached
    /// flag. Renderer overlay (`show_vpos`) consumes this to draw
    /// per-voice path arrows for diagnosing audible jumps. Captured
    /// from the cached prop result on the main thread; no audio-thread
    /// synchronisation needed.
    std::vector<VoiceSpatialSnapshot> getVoiceSpatialSnapshots() const;

    /// Returns the P$PrjSound schema name for `objID`, walking the
    /// MetaProp inheritance chain so concrete projectile instances
    /// resolve to their archetype's schema. Returns empty when no
    /// projectile sound is set anywhere in the chain. Projectile-spawn
    /// code should consult this when emitting a projectile to play
    /// the right flight/impact schema.
    std::string getProjectileSound(int32_t objID) const;

    /// Play an AI speech utterance for the given concept (e.g. "Greet",
    /// "Alert", "atlevelzero") on the emitter object. The voice is
    /// resolved from the emitter's P$VoiceIdx (preferred — int32) or
    /// P$SpchVoice (16-byte label) via the MetaProp inheritance chain;
    /// AIs almost always carry these on archetypes, not instances.
    ///
    /// `tags` is an optional tag-value query forwarded to the
    /// SpeechSelector (e.g. {Event: Idle, Damage: <int>}). Matching
    /// follows the original engine's "query is a superset of the leaf's
    /// constraints" rule.
    ///
    /// Returns the SoundHandle of the started voice, or
    /// SOUND_HANDLE_INVALID on miss (no voice set, unknown concept,
    /// no matching schema, schema lookup failed, sample failed to load).
    /// On success, also publishes a SoundEmissionEvent so AI hearing /
    /// debug overlays can observe the utterance.
    ///
    /// Fire-and-forget — the returned handle is informational; callers
    /// that want to halt the voice can pass it to haltSound, but most
    /// will simply let it play out.
    SoundHandle playSpeech(int32_t emitterObjID,
                           const std::string &conceptName,
                           const std::vector<SchemaTagValue> &tags = {});

    // Propagation layer toggles (all on by default)
    void setPortalRoutingEnabled(bool v) { mPortalRoutingEnabled = v; }
    bool getPortalRoutingEnabled() const { return mPortalRoutingEnabled; }
    void setProbePathingEnabled(bool v) { mProbePathingEnabled = v; }
    bool getProbePathingEnabled() const { return mProbePathingEnabled; }

    // ── Master bus DSP chain + mixer + spatialization + door LPF ──
    //
    // Storage and clamps live in AudioDSPChain (see AudioDSPChain.h). These
    // facades preserve the historical public AudioService API used by
    // RenderConfig and DebugConsole so callers don't need any changes.
    // setMaster/Direct/ReflectionGain stay out-of-line because they also
    // poke the live ReflectionMixNode (private to AudioService.cpp).

    /// Configure the soft limiter (prevents digital clipping)
    void setDSPLimiterEnabled(bool v) { mDSPChain->setDSPLimiterEnabled(v); }
    void setDSPLimiterKnee(float k)   { mDSPChain->setDSPLimiterKnee(k); }

    /// Configure the master bus compressor (tames transients)
    void setDSPCompressorEnabled(bool v) { mDSPChain->setDSPCompressorEnabled(v); }
    void setDSPCompThreshold(float t)    { mDSPChain->setDSPCompThreshold(t); }
    void setDSPCompRatio(float r)        { mDSPChain->setDSPCompRatio(r); }
    void setDSPCompAttackMs(float ms)    { mDSPChain->setDSPCompAttackMs(ms); }
    void setDSPCompReleaseMs(float ms)   { mDSPChain->setDSPCompReleaseMs(ms); }

    /// Configure the low-shelf EQ (bass boost/cut)
    void setDSPEQEnabled(bool v) { mDSPChain->setDSPEQEnabled(v); }
    void setDSPEQFreq(float f)   { mDSPChain->setDSPEQFreq(f); }
    void setDSPEQGain(float g)   { mDSPChain->setDSPEQGain(g); }
    void setDSPEQQ(float q)      { mDSPChain->setDSPEQQ(q); }

    /// Configure the ambient ducking system (disabled by default)
    void setDSPDuckingEnabled(bool v)   { mDSPChain->setDSPDuckingEnabled(v); }
    void setDSPDuckAmount(float a)      { mDSPChain->setDSPDuckAmount(a); }
    void setDSPDuckAttackMs(float ms)   { mDSPChain->setDSPDuckAttackMs(ms); }
    void setDSPDuckReleaseMs(float ms)  { mDSPChain->setDSPDuckReleaseMs(ms); }

    /// Configure the wet-bus tape saturation. See AudioDSPChain for semantics.
    void setDSPWetSaturationEnabled(bool v) { mDSPChain->setDSPWetSaturationEnabled(v); }
    bool getDSPWetSaturationEnabled() const { return mDSPChain->getDSPWetSaturationEnabled(); }
    void setDSPWetSaturationDrive(float d)  { mDSPChain->setDSPWetSaturationDrive(d); }
    float getDSPWetSaturationDrive() const  { return mDSPChain->getDSPWetSaturationDrive(); }

    // ── Mixer / global gains ──
    // Setters push live to the running mix node (if active) so console tweaks
    // take effect immediately without restarting the audio pipeline.
    void  setMasterGain(float g);
    float getMasterGain() const { return mDSPChain->getMasterGain(); }
    void  setReflectionGain(float g);
    float getReflectionGain() const { return mDSPChain->getReflectionGain(); }
    /// Dry-bus multiplier (direct path). Independent of master + reflection
    /// gains — lets you tune the direct/indirect ratio for "around the corner"
    /// audibility without raising overall volume.
    void  setDirectGain(float g);
    float getDirectGain() const { return mDSPChain->getDirectGain(); }
    void  setReflectionRampMs(float ms) { mDSPChain->setReflectionRampMs(ms); }
    float getReflectionRampMs() const { return mDSPChain->getReflectionRampMs(); }

    // ── Spatialization (HRTF + distance model) ──
    /// Must be set BEFORE bootstrapFinished()/init — used during HRTF creation.
    void setHRTFVolume(float v) { mDSPChain->setHRTFVolume(v); }
    /// "nearest" or "bilinear". Must be set BEFORE per-source effects are created.
    void setHRTFInterpolation(const std::string& s) { mDSPChain->setHRTFInterpolation(s); }
    void setSpatialBlend(float b) { mDSPChain->setSpatialBlend(b); }

    // ── Propagation tuning ──
    // Setters republish to the audio thread so runtime tweaks (e.g. via the
    // debug console) take effect on the next audio callback.
    void  setPropagationMaxDist(float d) { mPropagationMaxDist = std::max(10.0f, std::min(d, 5000.0f)); }
    float getPropagationMaxDist() const { return mPropagationMaxDist; }
    void  setPropMaxPaths(uint32_t n) {
        mPropMaxPaths = std::max<uint32_t>(1u, std::min<uint32_t>(n, 4u));
    }
    uint32_t getPropMaxPaths() const { return mPropMaxPaths; }
    void  setPropMaxPathDiff(float d) {
        mPropMaxPathDiff = std::max(0.0f, std::min(d, 50.0f));
    }
    float getPropMaxPathDiff() const { return mPropMaxPathDiff; }
    void  setDoorLpfOpenHz(float hz)    { mDSPChain->setDoorLpfOpenHz(hz); }
    float getDoorLpfOpenHz() const      { return mDSPChain->getDoorLpfOpenHz(); }
    void  setDoorLpfBlockedHz(float hz) { mDSPChain->setDoorLpfBlockedHz(hz); }
    float getDoorLpfBlockedHz() const   { return mDSPChain->getDoorLpfBlockedHz(); }
    void  setPropMinAttenuation(float a){ mDSPChain->setPropMinAttenuation(a); }
    float getPropMinAttenuation() const { return mDSPChain->getPropMinAttenuation(); }

    /** Runtime multiplier on the scalar gain produced by
     *  `eqCoeffsToDspMapping`. 1.0 = identity (baked behaviour). Values
     *  above 1 make through-portal sound louder than the bake would
     *  imply — useful when the baked pathing visibility threshold or
     *  probe coverage feels too aggressive and re-baking is expensive.
     *  Does NOT affect the LPF blocking factor; only amplitude.
     *  Range: [0.1, 10.0]. */
    void  setPathingGainScale(float s) { mPathingGainScale = std::max(0.1f, std::min(s, 10.0f)); }
    float getPathingGainScale() const  { return mPathingGainScale; }

    /** Companion knob to `setPathingGainScale`, but for the door-LPF
     *  closure factor. The legacy mapping `blocking = 1 − eqHigh`
     *  closes the LPF to ~400 Hz for typical cross-room ambients
     *  (eqHigh ≈ 0.02), producing unrecognisably muffled output.
     *  Multiplier in [0, 1]; lower values keep the LPF more open.
     *  Does NOT affect gain. Range: [0.0, 1.0]. */
    void  setPathingBlockingScale(float s) { mPathingBlockingScale = std::max(0.0f, std::min(s, 1.0f)); }
    float getPathingBlockingScale() const  { return mPathingBlockingScale; }

    /** Per-band weights that control how Steam Audio's 3-band eqCoeffs
     *  collapse into the scalar portalAttenuation gain. Applied as
     *  `gain = wL·eqLow + wM·eqMid + wH·eqHigh`, then multiplied by
     *  `pathingGainScale`. Default {0.25, 0.50, 0.25} (mid-heavy
     *  perceptual weighting). Weights are NOT auto-normalised — sums
     *  other than 1.0 act as an additional flat boost or cut. Each
     *  component clamped to [0, 1] at the setter. */
    void  setPathingGainBandWeights(float low, float mid, float high) {
        auto clamp01 = [](float v) {
            if (!std::isfinite(v) || v < 0.0f) return 0.0f;
            return v > 1.0f ? 1.0f : v;
        };
        mPathingGainWeightLow  = clamp01(low);
        mPathingGainWeightMid  = clamp01(mid);
        mPathingGainWeightHigh = clamp01(high);
    }
    float getPathingGainWeightLow()  const { return mPathingGainWeightLow;  }
    float getPathingGainWeightMid()  const { return mPathingGainWeightMid;  }
    float getPathingGainWeightHigh() const { return mPathingGainWeightHigh; }

    /** Minimum interval (seconds) between successive Steam Audio
     *  pathing-simulation updates. iplSimulatorRunPathing runs
     *  synchronously on the main loop thread; throttling here also
     *  skips the per-voice iplSourceSetInputs(...PATHING) staging on
     *  the same frames. Output reads (eqCoeffs → portalAttenuation /
     *  portalBlocking) keep running every frame and use Steam Audio's
     *  last-cached outputs, so audio stays smooth between sim runs.
     *  Default 0.1 s (10 Hz) matches the Unity/Unreal integration
     *  defaults. 0.0 = update every frame (legacy / A-B diagnostic).
     *  Range: [0.0, 1.0]. */
    void  setPathingUpdateInterval(float s) {
        mPathingUpdateInterval = std::max(0.0f, std::min(s, 1.0f));
    }
    float getPathingUpdateInterval() const { return mPathingUpdateInterval; }

    // ── Ambient tuning (facades — forwarded to AmbientSoundManager) ──
    void setAmbHysteresisStartMul(float m);
    void setAmbHysteresisStopMul(float m);
    void setAmbDefaultPriority(int p);
    // Per-voice spatialBlend override applied to AMB_ENVIRONMENTAL ambients
    // at activation time. 1.0 = full HRTF point-source pan; 0.0 = mono
    // passthrough. Lower values make room ambients (wind, church) feel
    // less like they emit from a single point. Object-attached ambients
    // (no AMB_ENVIRONMENTAL flag) ignore this and stay at full HRTF.
    void setAmbEnvironmentalSpatialBlend(float b);
    /// Global linear volume multiplier on every ambient voice (1.0 = no
    /// change). Compensates for the loudness re-baseline introduced when
    /// Steam Audio became the sole player-audio propagation authority.
    /// Tuned by ear at the YAML layer.
    void setAmbGlobalVolumeScale(float s);

    // ── Performance tuning (some MUST be set BEFORE buildAcousticScene) ──
    void setMaxActiveVoices(int n) { mMaxActiveVoicesCfg = std::max(8, std::min(n, 256)); }
    int  getMaxActiveVoices() const { return mMaxActiveVoicesCfg; }
    void setSimMaxOcclusionSamples(int n) { mSimMaxOcclusionSamplesCfg = std::max(4, std::min(n, 256)); }

    // ── Thread budget (merged convolution + sim) ──
    // Total threads dedicated to reverb work, split between the per-voice
    // convolution worker pool and Steam Audio's ray-trace simulator. The
    // invariant `conv_workers + sim_threads == reverb_threads` is enforced
    // at init time — guarantees no over-allocation regardless of how the
    // user sets the share.
    //
    // 0 = auto: total = max(2, hwconc - 2). Reserves 2 cores for main +
    // audio threads.
    void  setReverbThreads(int n) { mReverbThreadsCfg = std::max(0, std::min(n, 64)); }
    int   getReverbThreads() const { return mReverbThreadsCfg; }
    // Fraction of the reverb-thread budget assigned to convolution
    // workers (vs the ray-trace simulator). -1 = auto: chosen at init
    // based on whether realtime is enabled (baked-only configs bias
    // toward convolution since the sim is largely idle work in that
    // mode; realtime configs bias toward sim since the cycle is the
    // expensive one). Any value in [0.0, 1.0] is honored literally.
    void  setReverbThreadsConvShare(float share) {
        mReverbThreadsConvShareCfg = (share < 0.0f) ? -1.0f
                                                    : std::max(0.0f, std::min(share, 1.0f));
    }
    float getReverbThreadsConvShare() const { return mReverbThreadsConvShareCfg; }

    /// "default" or "embree". Embree requires a Steam Audio build with
    /// Embree linked; AudioService falls back to "default" with a loud
    /// [FALLBACK] stderr message if iplSceneCreate refuses the embree
    /// scene type.
    void setSceneType(const std::string& s) { mSceneTypeCfg = (s == "embree" ? "embree" : "default"); }

    // Audio engine (must be set BEFORE bootstrapFinished())
    void setAudioSampleRate(int r) { mAudioSampleRateCfg = r; }
    void setAudioFrameSize(int n)  { mAudioFrameSizeCfg = std::max(256, std::min(n, 4096)); }
    void setSoundCacheMB(int mb)   { mSoundCacheMBCfg = std::max(4, std::min(mb, 1024)); }

    /// Publish settings used on the audio thread (HRTF interp, spatial blend,
    /// door LPF, propagation min, distance model) into thread-safe storage.
    /// Call after configuration setters so the next audio callback picks them up.
    void publishAudioThreadParams();

    /** Bake acoustic probes for the current scene.
     *  Generates probes on a uniform floor grid, bakes pathing visibility,
     *  and saves to the specified file. Blocking call (~10-60 seconds).
     *  Progress is reported via the atomic float (0.0-1.0).
     *  @param outputPath  File path for the .probes output
     *  @param progress    Atomic float updated with bake progress (optional)
     *  @param spacing     Grid spacing in world units (default 5.0)
     *  @param height      Probe height above floor (default 5 world units)
     *  @return true if baking succeeded */
    bool bakeProbes(const std::string &outputPath,
                    std::atomic<float> *progress = nullptr,
                    float spacing = -1.0f, float height = -1.0f);

    /** Dry-run the Darkness-side probe placement passes and report
     *  predicted per-purpose probe counts WITHOUT invoking Steam
     *  Audio's bake. Used by `darknessHeadless probe_plan` (Capability
     *  A in PLAN.PROBE_DEBUG_TOOLING).
     *
     *  Does the same params-setup as `bakeProbes` (snapshot tuning
     *  state → ProbeBakeParams) plus the pathing-batch dedup that
     *  lives in this class, then delegates the geometry-aware
     *  placement work to `ProbeManager::computeBakePlan`. The
     *  per-pass / per-purpose counters in `out` are filled byte-for-
     *  byte the same as a live bake would produce — no rounding
     *  divergence, since the bake and the dry-run share the
     *  `computeReflectionPlacements` / `computePathingPlacements`
     *  helpers.
     *
     *  Returns false if the acoustic scene is not built or if the
     *  reflection placement pass produced zero probes (same as
     *  bakeProbes' failure modes). */
    bool computeProbePlan(ProbeBakePlan &out);

    /** Load baked probe data from disk and register with simulator.
     *  @param probePath  Path to .probes file
     *  @return true if loaded successfully */
    bool loadProbes(const std::string &probePath);

    /** @return number of loaded probes (0 if none) */
    int getProbeCount() const;

    /** Bake-time grid configuration. Takes effect on the next bakeProbes()
     *  call — does NOT relocate existing probes. Range-clamped to keep CPU
     *  cost reasonable. Forwards to the underlying ProbeManager. */
    void setProbeSpacingFt(float ft);
    float getProbeSpacingFt() const;
    void setProbeHeightFt(float ft);
    float getProbeHeightFt() const;

    /** Extra elevation tier (engine feet, above each floor probe) that
     *  bakeProbes will replicate the floor grid at, so wall-mounted /
     *  ceiling-mounted emitters route through pathing probes near their
     *  actual height. Empty vector = floor-only (legacy). */
    void setProbeElevations(std::vector<float> heights);

    /** Minimum clearance, in engine feet, between any baked probe and
     *  the nearest VERTICAL wall of its containing room. Bake-time
     *  filter only — candidates within this margin are dropped before
     *  any reflection rays are cast. 0 disables the clearance check.
     *  Floor and ceiling planes are intentionally excluded: probes are
     *  placed a fixed `height` above the floor by design, and short
     *  outdoor rooms (rooftops, balconies) often have a low audio-
     *  ceiling plane that would otherwise reject every probe inside
     *  them. The inside-solid (no-room) check always runs regardless
     *  of clearance value. Clamped to [0, 50] at the setter. */
    void setProbeMinWallClearanceFt(float ft);
    float getProbeMinWallClearanceFt() const { return mProbeMinWallClearanceFt; }

    /** Elevation-tier sparsity multiplier (see ProbeBakeParams). 1.0 =
     *  1:1 with floor probes (legacy), 2.0 = 2×2 binning (1:4), etc.
     *  Clamped to [1.0, 8.0]. Requires re-bake. */
    void setProbeElevationSparsityMul(float mul) {
        mProbeElevationSparsityMul = std::max(1.0f, std::min(mul, 8.0f));
    }
    float getProbeElevationSparsityMul() const { return mProbeElevationSparsityMul; }

    /** Global dedup pass radius in engine feet (see ProbeBakeParams).
     *  Probes within this distance of an earlier-kept probe (in
     *  placement order: floor → elevation → portal → emitter) get
     *  dropped. 0 = disabled. Clamped to [0.0, 10.0]. Requires re-bake. */
    void setProbeGlobalDedupRadiusFt(float ft) {
        mProbeGlobalDedupRadiusFt = std::max(0.0f, std::min(ft, 10.0f));
    }
    float getProbeGlobalDedupRadiusFt() const { return mProbeGlobalDedupRadiusFt; }

    /** Toggle the sparse ROOM_PORTAL pathing probe batch. When true (the
     *  default), bakeProbes produces a second IPLProbeBatch with one
     *  probe per Room centroid + two probes per RoomPortal. When false,
     *  the .probes file contains only the dense reflection batch and
     *  runtime Steam Audio pathing is effectively disabled (synthetic
     *  bypass branch). Takes effect on the next bake. */
    void setPathingProbeBatchEnabled(bool enabled) {
        mProbePathingBatchEnabled = enabled;
    }
    bool getPathingProbeBatchEnabled() const { return mProbePathingBatchEnabled; }

    /** Proximity dedup radius for the pathing batch (engine feet). Applied
     *  after all pathing-candidate emission (portals → centroids →
     *  emitters). Tuned independently from the reflection batch's
     *  global_dedup_radius_ft. 0 disables the pass. Takes effect on
     *  the next bake. */
    void setPathingDedupRadiusFt(float ft) {
        mProbePathingDedupRadiusFt = std::max(0.0f, std::min(ft, 30.0f));
    }
    float getPathingDedupRadiusFt() const { return mProbePathingDedupRadiusFt; }

    /** Reflection-bake skip. When true, the next bakeProbes() call carries
     *  the existing `.probes` file's reflection section forward verbatim
     *  and only re-bakes the pathing batch. Speeds iteration on pathing-
     *  only placement tweaks (reflection bake is multi-minute; pathing
     *  bake is seconds). Hard-fails the bake if the existing file has no
     *  reflection section. Set from `--skip-reflection-bake` CLI flag /
     *  `audio.reflections.bake_skip` YAML key. Default false. */
    void setReflectionBakeSkip(bool skip) { mReflectionBakeSkip = skip; }
    bool getReflectionBakeSkip() const { return mReflectionBakeSkip; }

    /** Snapshot of probe positions in feet (engine units). Populated by
     *  bakeProbes() and loadProbes(); empty if no probes are loaded. Used
     *  by the renderer to draw a debug overlay. The vector is rebuilt on
     *  every bake/load, so cache by index — values do not change between
     *  re-bakes. */
    const std::vector<Vector3> &getProbePositions() const;

    /** Pathing-batch probe positions (feet). Empty if no pathing batch
     *  is loaded (e.g. audio.pathing_probes.enabled = false in yaml).
     *  Parallel API to getProbePositions(), which returns the reflection
     *  batch — the two batches are independent and have different counts. */
    const std::vector<Vector3> &getPathingProbePositions() const;

    /** Per-pathing-probe debug visualization data. One entry per pathing
     *  probe (parallel to getPathingProbePositions). `roomID` is set via
     *  RoomService::roomFromPoint at call-time (so it picks up runtime
     *  room edits) and falls back to -1 when the probe sits in BSP void
     *  or RoomService is unavailable — the debug overlay filters those
     *  out so they never render. Used by `show_probe_radius` to draw
     *  influence spheres filtered to the camera-nearest rooms. */
    struct PathingProbeViz {
        Vector3 position{0.0f, 0.0f, 0.0f};
        float   radiusFt = 0.0f;
        int32_t roomID   = -1;
    };
    std::vector<PathingProbeViz> getPathingProbeViz() const;

    /** One edge in the pathing-probe visibility graph, with neighborhood
     *  activity payload (Capability C, PLAN.PROBE_DEBUG_TOOLING.md).
     *
     *  HONESTY NOTE: `edgeBlock` is NOT a per-edge result from Steam
     *  Audio's solver — the public C API does not expose that. It is
     *  a neighborhood-activity heatmap: across all active voices whose
     *  source position is within `visRadius` of either endpoint A or B
     *  of this edge, we report `max(1 - V.eqCoeffs.mid)`. Reads the
     *  same `lastGoodPathParams` snapshot the per-voice path effect
     *  already caches each frame — no extra Steam Audio API calls.
     *
     *  hasNeighbor=false means no active voice is in either endpoint's
     *  influence radius → renderer draws this edge as dim gray
     *  (background topology) instead of color-coding it. */
    struct PathingEdgeViz {
        Vector3 posA{0.0f, 0.0f, 0.0f};
        Vector3 posB{0.0f, 0.0f, 0.0f};
        float   edgeBlock  = 0.0f;
        bool    hasNeighbor = false;
    };

    /** Per-voice source→listener arrow for Layer-3 of the pathing-graph
     *  overlay. One entry per spatial voice with a successful pathing
     *  solve; `eqMid` is the mid-band coefficient straight off
     *  `lastGoodPathParams.eqCoeffs[1]`. Renderer colors the arrow by
     *  this value. */
    struct VoiceArrowViz {
        Vector3 source{0.0f, 0.0f, 0.0f};
        Vector3 listener{0.0f, 0.0f, 0.0f};
        float   eqMid     = 1.0f;
        bool    everSolved = false;
        bool    isAmbient = false;
    };

    /** Aggregate the pathing adjacency (static, from ProbeManager) with
     *  per-frame voice EQ outputs into one renderable list. Called once
     *  per frame from the main loop (NOT the audio callback — per
     *  feedback_threading_architecture). Returns empty when no pathing
     *  batch is loaded or the adjacency hasn't been built yet; the HUD
     *  reports that state to the user. */
    std::vector<PathingEdgeViz> getPathingEdgeViz() const;

    /** Per-voice arrow snapshot for Layer-3. Called once per frame from
     *  the main loop. Same threading rules as getPathingEdgeViz. */
    std::vector<VoiceArrowViz> getVoiceArrowViz() const;

    /** Trigger the one-shot O(N²) pathing-adjacency build on the
     *  ProbeManager. Idempotent — calls clearPathingAdjacency first.
     *  Safe to call before the raycaster is wired (the inner method
     *  emits [VIZ_FALLBACK] in that case). Invoked automatically after
     *  loadProbes / bakeProbes; callers can also trigger from a debug
     *  console binding if they re-injected the raycaster mid-session. */
    void rebuildPathingAdjacency();

    /** Per-probe reachability classification, parallel to
     *  getProbePositions(). Populated by classifyProbeReachability();
     *  empty until that runs. Used by the renderer to tint the probe
     *  overlay so we can preview which probes a future reachability
     *  filter would prune before committing to a re-bake. */
    enum class ProbeFate : uint8_t {
        Kept        = 0,  ///< In a room reachable from a mapper-placed object
        NoRoom      = 1,  ///< roomFromPoint returned null (BSP void)
        Unreachable = 2,  ///< Has a room, but no mapper-placed object's
                          ///< portal-graph component touches it
        Isolated    = 3,  ///< Spatially isolated — no other probe within
                          ///< Steam Audio's probe-visibility radius
                          ///< (= probeSpacingFt × probeIsolationMul, default
                          ///< 1.5x). Steam Audio's pathing solver cannot
                          ///< form a visibility edge from a probe without
                          ///< nearby neighbors, so any source associated
                          ///< with it returns the 0.1f sentinel forever.
                          ///< Heuristic — does NOT test line-of-sight: a
                          ///< probe with nearby neighbors but blocked LOS
                          ///< will still classify as Kept yet fail at
                          ///< runtime. Mostly catches isolated emitter-
                          ///< anchored probes whose nudge landed away from
                          ///< the rest of the grid.
    };
    const std::vector<ProbeFate> &getProbeFates() const { return mProbeFates; }

    /** Recompute mProbeFates by:
     *    1) Collecting every concrete (positive-ID) object whose P$Position
     *       is directly owned (not inherited) and whose enclosing room is
     *       non-null. Each such room is a BFS seed.
     *    2) Multi-seed BFS over the room-portal graph (Room::getPortal(i)
     *       ->getFarRoom()), unioning all reached rooms into a single
     *       playable set. This covers levels with teleporter-only-
     *       reachable subgraphs — every gameplay component contains at
     *       least one mapper-placed seed.
     *    3) For each probe: roomFromPoint(pos) → null = NoRoom; not in
     *       playable set = Unreachable; else = Kept.
     *  @return number of probes that would be pruned (NoRoom + Unreachable). */
    size_t classifyProbeReachability();

    /** Diagnostic: is any player-emitted voice currently making sound?
     *  Used by the renderer to flash a listener-position marker so a user
     *  can visually correlate footstep loudness with where they are
     *  standing relative to baked probes. Cheap atomic load. */
    bool isPlayerEmittedVoiceActive() const { return mPlayerEmittedActive.load(std::memory_order_relaxed); }

    /** Start/stop recording the final audio output to WAV + position CSV.
     *  Files are written to the current working directory:
     *  - darkness_audio_capture.wav (48kHz stereo float32)
     *  - darkness_audio_positions.csv (timestamp, x, y, z, yaw, pitch, voices) */
    void startAudioRecording();
    void stopAudioRecording();
    bool isRecordingAudio() const;

    /** Get the standard probe file path for a mission.
     *  Stored in ~/darkness/{gameName}/baked_probes/{missionName}.probes
     *  Creates directories if they don't exist. */
    static std::string getProbeFilePath(const std::string &misPath,
                                         const std::string &gameName = "thief2");

    /** Reflection convolution rate divisor (1=full 48kHz, 2=half 24kHz, 4=quarter 12kHz).
     *  Must be set BEFORE buildAcousticScene() — cannot change at runtime.
     *  Higher divisors reduce per-voice convolution cost proportionally.
     *  Reverb is perceptually transparent at 24kHz; 12kHz cuts above 6kHz
     *  but reverb tails are dominated by frequencies below 4kHz anyway. */
    void setReflectionRateDivisor(int div);
    int  getReflectionRateDivisor() const;

    /// Backward-compatible wrapper for the old bool API
    void setHalfRateReflections(bool enabled);
    bool getHalfRateReflections() const;

    // Convolution worker count / simulator thread count are no longer
    // independent knobs — they are derived at init time from
    // mReverbThreadsCfg + mReverbThreadsConvShareCfg (see setReverbThreads
    // / setReverbThreadsConvShare above). Getters expose the derived
    // values for diagnostics.
    int  getConvolutionWorkerCount() const { return mConvolutionWorkerCount; }
    int  getSimulatorThreadCount() const { return mSimulatorThreadCount; }

    /** Ambisonics order for reflection convolution (0 or 1).
     *  Must be set BEFORE buildAcousticScene() — cannot change at runtime.
     *  Order 0 = 1 channel (omnidirectional reverb), 4x cheaper per voice.
     *  Order 1 = 4 channels (directional reverb), more spatial detail in tails.
     *  Direct path HRTF is unaffected — spatial positioning stays full quality. */
    // Ambisonic order: 0 = omnidirectional (1ch), 1 = directional (4ch),
    // 2 = (9ch), 3 = (16ch). Memory + decode cost scale with (N+1)^2 channels.
    void setAmbisonicsOrder(int order) { mAmbisonicsOrder = std::max(0, std::min(order, 3)); }
    int  getAmbisonicsOrder() const { return mAmbisonicsOrder; }

    /** @return the actual reflection pipeline sample rate (24000 or 48000) */
    uint32_t getReflectionSampleRate() const { return mReflectionSampleRate; }

    /** @return number of triangles in the current acoustic scene */
    int  getAcousticSceneTriCount() const { return mAcousticTriCount; }

    // ── Global AI hearing data accessors (Unit C) ──

    /** Copy the parsed AIHearStat chunk into `out`.
     *  @return true if the gamesys contained an AIHearStat chunk and `out`
     *          was populated; false when no chunk was loaded (caller should
     *          fall back to kDefaultAIHearingStats from AIHearingData.h). */
    bool getAIHearingStats(AIHearingStats &out) const;

    /** Copy the parsed AISNDTWK chunk into `out`.
     *  @return true if the gamesys contained an AISNDTWK chunk and `out`
     *          was populated; false otherwise. */
    bool getAISoundTweaks(AISoundTweaks &out) const;

    // ── Sound emission pub/sub ──

    /** Register a listener that fires whenever a sound is emitted into the
     *  world. Listeners are called synchronously on the main thread from
     *  whatever code site published the event; keep callbacks cheap. */
    void registerSoundEmissionListener(SoundEmissionListener cb);

    // ── Audio perf-capture JSONL sink (PLAN.AUDIO_PROFILING.md §1.1 §1.2) ──

    /** Open the per-run audio_perf.jsonl artifact file. Idempotent — the
     *  second call is a no-op (the first wins). Path is built as
     *      ./perf/<mission>/<utc_iso>__<perfLabel>/audio_perf.jsonl
     *  Directories are created as needed. On success, immediately writes
     *  the `event:"run.meta"` first line (git SHA, build timestamp,
     *  start time, full audio.* config snapshot, device params, cli_argv).
     *
     *  RT-safety: ALL writes happen on the main thread inside
     *  dumpAudioStatusPeriodic / openPerfJsonl / closePerfJsonl. The audio
     *  callback never touches the file (per feedback_threading_architecture).
     *
     *  The caller is responsible for serializing the audio config snapshot
     *  into JSON (we don't pull RenderConfig.h into the services layer to
     *  avoid yaml-cpp leakage). `audioConfigJson` is dropped into the
     *  run.meta record verbatim under key `"audio"`; it MUST be a valid
     *  JSON object literal beginning with `{` and ending with `}`.
     *
     *  @param missionName  filename without extension (e.g. "miss6")
     *  @param missionPath  full path passed on CLI, recorded as `mis_path`
     *  @param perfLabel    --perf-label value, must be FS-safe
     *  @param cliArgv      space-joined argv for the `cli_argv` record
     *  @param audioConfigJson  pre-serialized `{...}` object of audio.* keys
     *  @return true if the file opened successfully */
    bool openPerfJsonl(const std::string& missionName,
                       const std::string& missionPath,
                       const std::string& perfLabel,
                       const std::string& cliArgv,
                       const std::string& audioConfigJson);

    /** Flush and close the JSONL sink. Safe to call even if never opened.
     *  Called from shutdown() and from main() on `--exit-after-seconds`. */
    void closePerfJsonl();

    /** True if openPerfJsonl succeeded and the file is still open. */
    bool isPerfJsonlOpen() const;

protected:
    // ── Service lifecycle ──

    bool init() override;
    void bootstrapFinished() override;
    void shutdown() override;

    // ── DatabaseListener ──

    void onDBLoad(const FileGroupPtr &db, uint32_t curmask) override;
    void onDBSave(const FileGroupPtr &db, uint32_t tgtmask) override;
    void onDBDrop(uint32_t dropmask) override;

    // ── LoopClient ──

    void loopStep(float deltaTime) override;

private:
    /// Build the `ProbeBakeParams` that the next bake (or dry-run plan)
    /// would consume. Snapshots all bake-time tuning state into
    /// `outParams`, builds the pathing-batch candidate list (rooms +
    /// portals + emitter anchors), and runs the AudioService-side
    /// proximity dedup on those candidates. If `planCounters` is
    /// non-null, the dedup pass populates its dedup counters
    /// (used by the `darknessHeadless probe_plan` verb to report what
    /// the dedup pass dropped).
    ///
    /// `spacingOverride` / `heightOverride` mirror `bakeProbes`' optional
    /// args — passed through to `outParams.spacingFtOverride` /
    /// `heightFtOverride`. -1.0f = use the ProbeManager's configured
    /// values.
    ///
    /// Called by both `bakeProbes` (planCounters=null) and
    /// `computeProbePlan` (planCounters=&plan), so the two share an
    /// identical params-building code path.
    void prepareProbeBakeParams(ProbeBakeParams &outParams,
                                ProbeBakePlan *planCounters,
                                float spacingOverride,
                                float heightOverride);

    // ── Service dependencies ──

    DatabaseServicePtr mDbService;
    RoomServicePtr mRoomService;
    PropertyServicePtr mPropertyService;
    ObjectServicePtr mObjectService;

    // ── Sound propagation through portal graph ──
    //
    // The portal blocking factor map (was mBlockingFactors), the per-room
    // LoudRoom transmission map (was mRoomTransmission), and the BFS
    // forwarder into RoomService::propagateSoundPath all live in
    // SoundPropagation. AudioService keeps thin facades (propagateSound,
    // setBlockingFactor, getBlockingFactor) that forward to it; the
    // per-voice loopStep path reads / mutates it directly via this owner.
    std::unique_ptr<class SoundPropagation> mSoundPropagation;

    // ── Diagnostic raycaster (renderer-injected) ──
    // Used by the [PATH] periodic|spike block to verify whether a SPIKE'd
    // voice's nearest-probe LOS is actually blocked by world geometry.
    // Not on any hot path; only fires inside the gated diagnostic block.
    AudioRaycastFn mRaycaster;

    // ── Volumetric occlusion configuration ──
    //
    // Radius (engine feet) + sample count for Steam Audio's volumetric
    // occlusion model. setOcclusionRadius / setOcclusionSamples (and the
    // public getters) forward to this. The clamp on radius is widened to
    // [0.1, 200] (was [0.3, 30]) — preserved here.
    std::unique_ptr<class AudioOcclusion> mAudioOcclusion;

    // Voice handle allocation now lives in VoicePool (mVoicePool->allocate()).
    // The previous mNextHandle counter moved with it.

    // ── Audio backends ──

    /// miniaudio engine (heap-allocated to keep ma_engine out of header)
    ma_engine *mMaEngine = nullptr;

    /// miniaudio playback device. We own the device explicitly (rather than
    /// letting ma_engine_init create one for us) so we can set
    /// `ma_device_config.periods` — the engine config exposes only
    /// `periodSizeInFrames`, leaving period COUNT at miniaudio's default of 3.
    /// Two periods at the same period size halves the device-buffer latency
    /// component (the largest contributor to total output latency on our
    /// pipeline). See AudioService.cpp `initMiniaudio` for the wiring.
    ma_device *mMaDevice = nullptr;

    /// Steam Audio context
    IPLContext mIplContext = nullptr;

    /// Steam Audio HRTF for binaural rendering
    IPLHRTF mIplHrtf = nullptr;

    /// Whether audio backends initialized successfully
    bool mAudioReady = false;

    /// Actual device sample rate (detected at init, used by Steam Audio).
    /// Initial value is the engine default from AudioUnits.h — overwritten
    /// at init by the negotiated device rate.
    uint32_t mDeviceSampleRate = kDefaultDeviceSampleRate;

    /// Audio processing frame size (aligned with Steam Audio). Initial
    /// value is the engine default from AudioUnits.h — overwritten at
    /// init by the YAML-configured `audio.performance.frame_size`.
    uint32_t mFrameSize = kDefaultDeviceFrameSize;

    // ── Sound resource loading ──

    /// CRF sound loader (opens snd.crf via zziplib)
    std::unique_ptr<CRFSoundLoader> mSoundLoader;

    /// LRU cache for recently-used decoded sounds (default 64MB budget)
    std::unique_ptr<SoundCache> mSoundCache;

    /// Negative cache of sample names that have already failed to load.
    /// Avoids re-attempting the CRF lookup every frame for missing assets
    /// (e.g. mission references a schema/sample not present in snd.crf),
    /// and gates the load-failure log to a single message per sample.
    std::unordered_set<std::string> mFailedSamples;

    /// Schema parser (.sch/.spc/.arc files — sound selection rules)
    std::unique_ptr<SchemaParser> mSchemaParser;

    // ── Active voice management ──

    /// Active-voice pool — owns the handle→voice map and handle allocator.
    /// Voices store their own WAV data, miniaudio decoder, and sound object;
    /// the pool's cleanupFinished() sweep runs each frame from loopStep().
    /// AudioService still owns voice STARTUP (createVoiceSource / initVoiceDSP
    /// / ma_sound_start) and the removeVoiceSource() side of TEARDOWN — the
    /// pool just owns lifetime + iteration.
    std::unique_ptr<class VoicePool> mVoicePool;

    /// Per-frame sweep — drops voices whose end-callback fired and routes
    /// each through removeVoiceSource() + a [VOICE_PEAK] summary log.
    /// Implemented as a thin facade over VoicePool::cleanupFinished(hook).
    void cleanupFinishedVoices();

    /// Random number generator for sample selection
    std::mt19937 mRng{std::random_device{}()};

    /// Per-schema last sample index for NO_REPEAT behavior
    std::unordered_map<std::string, int> mLastSampleIdx;

    // ── Ambient sound management ──
    //
    // P$AmbientHack lifecycle (load, per-frame volume updates with
    // hysteresis + falloff) lives in AmbientSoundManager. This service
    // owns the manager and forwards tuning setters to it; it also
    // exposes the manager to the friend class via the back-pointer in
    // AmbientSoundManager.
    std::unique_ptr<AmbientSoundManager> mAmbientManager;

    /// Periodic (~5 s) audio status dump — voice counts, callback budget,
    /// per-ambient diagnostic rows. Called from loopStep(). Used to live
    /// at the head of updateAmbientVolumes() before the extraction; now
    /// it reads ambient state from mAmbientManager.
    void dumpAudioStatusPeriodic();

    /// Per-frame ducking envelope update — counts non-ambient SFX voices
    /// and ramps the global duck multiplier toward its target. Read by
    /// the AmbientSoundManager update path (and elsewhere) when applying
    /// per-voice ducking. Was previously the tail of updateAmbientVolumes.
    void updateAmbientDuckingEnvelope();

    /// Overlay per-archetype schema property overrides from the gamesys
    /// onto our parsed SchemaEntry table. Reads P$SchPlayPa (play params:
    /// flags, audio class, volume, pan, delay, fade), P$SchLoopPa (loop
    /// params: flags, max samples, loop count, interval), P$SchPriori
    /// (priority override) and P$SchMsg (AI message label). Matches the
    /// schema-archetype object to a SchemaEntry by SymName.
    void loadSchemaPropertyOverrides();

    /// Load auxiliary per-schema / per-room sound data from mission +
    /// gamesys properties (P$SchAttFac, schema overlays, P$PrjSound /
    /// P$Heartbeat / P$SchLastSa, P$LoudRoom, P$Acoustics verification),
    /// then trigger AmbientSoundManager::loadAmbientSounds(). Called
    /// from loadSoundResources after the schema parser is ready.
    void loadAuxiliarySoundData();

    // ── Voice-state accessors exposed to AmbientSoundManager ──
    //
    // ActiveVoice and ReflectionMixNode are defined inside AudioService.cpp
    // (their definitions pull in miniaudio + Steam Audio implementation
    // details). The AmbientSoundManager only needs a small, well-defined
    // slice of voice state — these helpers keep that surface narrow and
    // mean AmbientSoundManager.cpp doesn't need to see the full structs.
    //
    // Returns true iff `handle` resolves to a live voice in the pool.
    bool voiceExists(SoundHandle handle) const;
    // Set the voice's per-source BFS-termination distance. Used by the
    // ambient/spot loaders to bake the per-source max audible distance.
    void voiceSetMaxAudibleDist(SoundHandle handle, float maxDist);
    // Per-voice distance-attenuation rolloff factor (P$SchAttFac).
    // Default 1.0 → minDistance=1m in Steam Audio's INVERSEDISTANCE model
    // (= DEFAULT model behaviour). Factor > 1 keeps the sound at full
    // volume out to N meters before 1/d falloff. Clamped [0.1, 100].
    void voiceSetAttenuationFactor(SoundHandle handle, float factor);
    // Override the voice's HRTF/mono spatialBlend (atomic, audio-thread
    // safe). Used for AMB_ENVIRONMENTAL ambients.
    void voiceSetSpatialBlendOverride(SoundHandle handle, float blend);
    // Apply a final ma_sound_set_volume() to the voice. AmbientSoundManager
    // computes the linear gain and applies the duck multiplier; this
    // helper crosses the ActiveVoice/miniaudio boundary in one shot.
    void voiceSetLinearVolume(SoundHandle handle, float linearVol);
    // Current global ducking multiplier (1.0 = no duck). Returns 1.0 if
    // ducking is disabled or the mix node isn't ready.
    float currentDuckGain() const;

    // ── Miscellaneous per-archetype sound data (Unit G) ──

    /// P$PrjSound — schema name to play for a projectile archetype's
    /// flight/impact sound. Keyed by archetype objID (negative). The
    /// projectile-spawning code consults this map when emitting a
    /// projectile to pick the right flight schema.
    std::unordered_map<int32_t, std::string> mProjectileSounds;

    /// P$Heartbeat — interval (ms) between heartbeat sounds for a given
    /// object archetype/instance. Keyed by objID. Default-zero entries
    /// are not stored. Used by health-low / AI-frightened heartbeat
    /// schemas; the runtime consults this to pace the heartbeat trigger.
    std::unordered_map<int32_t, int32_t> mHeartbeatIntervals;

    /// P$SchLastSa — last-played sample index per schema archetype.
    /// Runtime-mutable: written when a schema plays, restored from save
    /// data on load. Drives SCH_NO_REPEAT behavior across save/load.
    /// Keyed by schema name (SymName of the archetype object).
    std::unordered_map<std::string, int32_t> mSchemaLastSampleSaved;

    /// Load the small per-archetype sound props above from mission data.
    /// Called from loadAmbientSounds() after the schema-override pass.
    void loadMiscSoundProperties();

    // ── Mission-level sound data (Unit H) ──

    /// AMBIENT chunk — single int32 holding the ObjID of the
    /// environmental ambient (the single global "background music-like"
    /// ambient slot) that was active at save time. 0 means "no
    /// environmental ambient was active". The original engine wrote it
    /// from AmbientSave() to support save-game resume; pristine shipping
    /// missions ship with 0 (only one Thief 2 mission, miss14, ships
    /// with a nonzero pre-set env ambient).
    ///
    /// This field is captured for future SAV-file resume + the
    /// miss14-style level-start env ambient case; it is NOT a level-wide
    /// enable flag.
    bool mHasAmbientChunk = false;
    int32_t mEnvAmbientObjID = 0;

    /// L$SoundDesc links — sound-descriptor relations attached to objects
    /// (e.g. triggered/scripted sounds distinct from P$AmbientHack). Each
    /// entry is a (source-object, destination-object, link-id) tuple.
    /// The destination is typically a sound-descriptor archetype carrying
    /// the schema name; runtime triggers walk these on script events.
    struct SoundDescLink {
        int32_t src;
        int32_t dst;
        uint32_t linkID;
    };
    std::vector<SoundDescLink> mSoundDescLinks;

    /// Load mission-level sound data (AMBIENT chunk + L$SoundDesc).
    /// Called from onDBLoad after the ROOM_EAX block.
    void loadMissionSoundData(const FileGroupPtr &db);

    // ── Sound chunk databases (Units D, E, F) ──

    /// Compiled environmental-sound tag database from the ENV_SOUND
    /// chunk in dark.gam. First-pass: capture raw bytes; full decoding
    /// of the binary tag-tree is deferred.
    std::unique_ptr<class EnvSoundDatabase> mEnvSoundDB;

    /// Compiled speech database from the Speech_DB chunk in dark.gam.
    /// First-pass: capture raw bytes; full decoding of voice/concept
    /// trees is deferred.
    std::unique_ptr<class SpeechDatabase> mSpeechDB;

    /// Schema → samples mapping from the SchSamp chunk in dark.gam.
    /// Canonical compiled form of what we currently re-derive from .sch
    /// files. Loaded for cross-check + to cover gam-only schemas.
    std::unique_ptr<class SchemaSamplesChunk> mSchemaSamplesDB;

    /// Global AI hearing-stats / sound-tweaks chunks (Unit C). Both
    /// optional; populated only when the gamesys contains the chunks.
    /// Storage uses the parser's value types; default-constructed when
    /// absent. See audio/AIHearingData.h for layout.
    bool mHasAIHearingStats = false;
    bool mHasAISoundTweaks = false;
    /// Backing storage as raw bytes so we don't pull AIHearingData.h
    /// into this header (keeps the include surface narrow).
    uint8_t mAIHearingStatsBytes[48] = {0};
    uint8_t mAISoundTweaksBytes[24] = {0};

    /// Load all global sound chunk databases (ENV_SOUND, Speech_DB,
    /// SchSamp, AIHearStat, AISNDTWK). Called from onDBLoad.
    void loadSoundChunkDatabases(const FileGroupPtr &db);

    // ── Texture material mapping (for footstep schema selection) ──
    //
    // Per-room LoudRoom transmission factors moved to SoundPropagation
    // (mSoundPropagation->setRoomTransmission / getRoomTransmission).
    // Used in propagation BFS as the per-room transmission multiplier.

    /// Per-texture material keyword (indexed by TXLIST texture index).
    /// E.g., "stone", "metal", "wood", "carpet", "generic" (default).
    std::vector<std::string> mTextureMaterials;

    // ── Dark Engine room acoustic data (for verification/comparison) ──

    /// Per-room EAX reverb preset index from ROOM_EAX chunk (0-25).
    /// Maps room ID → EAX preset index. Parsed for verification logging
    /// against Steam Audio's physics-based reverb. Not used for rendering.
    std::unordered_map<int32_t, uint32_t> mRoomEAXPresets;

    /// On-disk layout of P$Acoustics property (12 bytes).
    /// Stores the EAX reverb type, dampening, and height parameters that the
    /// original Dark Engine passed to Creative's EAX hardware reverb API.
    #pragma pack(push, 1)
    struct PropAcoustics {
        uint32_t eax;        ///< EAX preset index (0-25, see kEAXPresetNames)
        int32_t dampening;   ///< Room dampening factor
        int32_t height;      ///< Room height override
    };
    #pragma pack(pop)

    // ── Steam Audio acoustic scene ──

    /// Acoustic scene built from WR world geometry
    IPLScene mIplScene = nullptr;

    /// Static mesh within the scene (world geometry + material assignments)
    IPLStaticMesh mIplStaticMesh = nullptr;

    /// Per-door dynamic acoustic geometry. Each entry owns:
    ///   - a sub-scene containing the door's local-space OBB mesh
    ///   - a static mesh inside that sub-scene (geometry + material)
    ///   - an instanced mesh in the main scene that references the sub-scene
    /// We keep our own reference to the sub-scene for the lifetime of the
    /// instance: Steam Audio's docs do not explicitly state that
    /// iplInstancedMeshCreate retains the sub-scene, so releasing it
    /// immediately after iplInstancedMeshAdd risks a dangling reference at
    /// the next scene-commit (seen as a boot-time hang on some builds).
    struct DoorAudioInstance {
        IPLScene         subScene      = nullptr;
        IPLStaticMesh    staticMesh    = nullptr;
        IPLInstancedMesh instancedMesh = nullptr;
        /// Cached for the show_door_geometry debug overlay. Local-space
        /// (door-relative) vertex floats + index triples, plus the current
        /// world transform. Updated by setDoorTransform alongside the IPL
        /// instance transform so the overlay sees doors in their live pose.
        std::vector<float>    localVertices;
        std::vector<int32_t>  indices;
        Matrix4               worldTransform{1.0f};
        /// Cached world-space AABB of the door mesh in ENGINE coordinates
        /// (Z-up feet). Recomputed from localVertices × worldTransform by
        /// registerDoorGeometry and setDoorTransform. Used by the
        /// [PATH_PROBE_DOOR] diagnostic to check whether a baked probe sits
        /// inside or near a door's footprint (the "probe-as-sound-transport
        /// over a closed door" failure mode).
        Vector3 worldAABBmin{0.0f, 0.0f, 0.0f};
        Vector3 worldAABBmax{0.0f, 0.0f, 0.0f};
    };
    std::unordered_map<int32_t, DoorAudioInstance> mDoorAudioInstances;

    /// Recompute worldAABBmin/max for a single door instance from its cached
    /// localVertices × worldTransform. Called by registerDoorGeometry on
    /// initial registration and by setDoorTransform on every transform push.
    /// Engine coordinates (Z-up feet) — same frame as voice->worldPos and
    /// ProbeManager::getProbePositions, so the [PATH_PROBE_DOOR] diagnostic
    /// can compare directly without engine↔IPL conversion.
    void recomputeDoorWorldAABB(DoorAudioInstance &inst) const;

    /// Set by setDoorTransform; consumed (and cleared) at the top of the
    /// next loopStep. Coalesces multiple per-frame door transform updates
    /// into one BVH refit.
    std::atomic<bool> mSceneNeedsCommit{false};

    /// Reflection simulation owner — wraps the Steam Audio reflection
    /// IPLSimulator handle, its dedicated background worker thread, the
    /// deferred source-add/remove queues, throttle counter, rate divisor,
    /// demote hysteresis and the Stage 2.2 demote fallback. See
    /// ReflectionSimulator.h for the full threading contract.
    std::unique_ptr<ReflectionSimulator> mReflectionSim;

    /// Simulator for direct path (distance attenuation, occlusion, air
    /// absorption, transmission). Runs synchronously on the main thread in
    /// `loopStep`, so its source list is never iterated concurrently —
    /// `iplSourceAdd` / `iplSourceRemove` are always safe to call inline.
    /// Shares `mIplScene` with the reflection simulator (refcounted via
    /// `iplSceneRetain`); world geometry is not duplicated.
    IPLSimulator mDirectSimulator = nullptr;

    /// Pathing simulator owner — wraps the Steam Audio pathing IPLSimulator
    /// handle and its dedicated background worker thread that pumps
    /// iplSimulatorRunPathing. Split off from the direct sim to keep the
    /// 50–11000 ms pathing iteration off the main loop. See
    /// PathingSimulator.h for the full threading contract.
    std::unique_ptr<PathingSimulator> mPathingSim;

    /// True once the loaded probe batch has been attached to the pathing
    /// simulator. Steam Audio's pathing solver looks up the nearest probe
    /// to each source/listener via the batches registered on the pathing
    /// simulator handle (separate from the reflection simulator's batch
    /// list). Idempotent: attached once per probe-load, with
    /// iplProbeBatchRetain so the ProbeManager's release path (which
    /// releases on the reflection sim) doesn't free the batch from under
    /// us.
    bool mPathingProbeBatchAdded = false;

    /// Whether an acoustic scene is currently active (built and committed).
    /// Atomic as a defensive measure — currently only accessed from the main
    /// thread, but the reflection sim thread could plausibly need it in future.
    std::atomic<bool> mSceneReady{false};

    // ── Reflection pipeline (per-voice reflection effect → ambisonics → binaural) ──
    //
    // The pre-Phase-3 design held a shared IPLReflectionMixer here and routed
    // per-voice convolution into it. PLAN.HYBRID_REVERB.md Phase 3 dropped
    // the mixer entirely: Steam Audio's mixer overload only handles
    // CONVOLUTION/TAN and rejects HYBRID, so we sum per-voice ambisonics
    // manually inside each ConvolutionSubWorker. (Now that we ship HYBRID-
    // only, the mixer overload is permanently out of reach — the per-worker
    // manual summation is the only architecture.) The ambisonics decoder
    // below is still needed (one global decode of the summed ambisonics to
    // binaural stereo).

    /// Ambisonics decode effect — converts the per-worker-summed reflection
    /// ambisonics to binaural stereo via HRTF.
    IPLAmbisonicsDecodeEffect mIplAmbiDecodeEffect = nullptr;

    /// Whether reflections are enabled (toggled at runtime with R key)
    bool mReflectionsEnabled = true;

    /// Global reflection mix node — sits between per-voice DSP nodes and the
    /// engine endpoint. Reads the convolution worker's output and adds to direct.
    std::unique_ptr<ReflectionMixNode> mReflectionMixNode;

    /// Off-thread convolution worker pool — owns the ConvolutionWorker (with
    /// its K sub-worker threads) that consume per-voice mono snapshots,
    /// run iplReflectionEffectApply against the reflection IRs, decode
    /// ambisonics to binaural stereo, and write double-buffered output for
    /// the reflection mix node to read on the next callback.
    std::unique_ptr<ConvolutionWorkerPool> mConvolutionPool;

    // ── Tunable reflection parameters ──
    //
    // Split into realtime + bake groups: realtime drives every-frame
    // iplSimulatorRunReflections + per-voice iplReflectionEffectApply, bake
    // drives the one-shot iplReflectionsBakerBake and is persisted into the
    // .probes cache file. Changes to bake params require a re-bake to take
    // effect; changes to realtime params take effect on the next callback.

    /// Length (seconds) of the convolution head of the HYBRID IR. The
    /// parametric tail thereafter is driven by `reverbTimes[3]` (RT60) from
    /// the baked probe data. Steam Audio crashes if this exceeds the IR
    /// duration, so we enforce mRealtimeDuration > mHybridTransitionTime with
    /// a small margin in initReflectionPipeline. Default 1.0 s matches Steam
    /// Audio's Unity / Unreal reference integrations.
    float mHybridTransitionTime = 1.0f;

    /// Fraction of `mHybridTransitionTime` used for the convolution↔
    /// parametric crossfade, anchored at the END of the transition.
    float mHybridOverlapPercent = 0.25f;

    // Realtime simulation params (used per audio frame).
    int   mRealtimeNumRays         = 1024;  ///< Rays per realtime sim step (128–8192)
    int   mRealtimeNumBounces      = 4;     ///< Bounces per ray (1–8)
    float mRealtimeDuration        = 2.0f;  ///< Realtime IR duration in seconds (0.5–4.0)
    int   mRealtimeDiffuseSamples  = 32;    ///< Diffuse samples per bounce (16–256)

    // Offline bake params (one-shot per mission).
    int   mBakeNumRays             = 4096;  ///< Rays per bake step (1024–65536)
    int   mBakeNumBounces          = 8;     ///< Bake bounces (1–64)
    float mBakeDuration            = 4.0f;  ///< Bake IR duration in seconds (>= realtime)
    int   mBakeDiffuseSamples      = 256;   ///< Bake diffuse samples (32–4096)
    int   mBakeAmbisonicsOrder     = 1;     ///< Bake ambisonic order (0–3)

    int mReverbVoices         = DEFAULT_REVERB_VOICES;
    // Subset of mReverbVoices that runs realtime ray-traced IRs. 0 =
    // baked-only (the user's default). Set by RenderConfig wiring.
    int mReverbVoicesRealtime = 0;
    int mAcousticTriCount = 0;         ///< Triangles in current acoustic scene

    /// Global multiplier for material transmission coefficients.
    /// 1.0 = physically accurate, 10.0 = audible through walls (game-friendly).
    float mTransmissionScale = 10.0f;

    /// Global multiplier for material absorption coefficients.
    /// Values < 1.0 make surfaces more reflective (less absorption per bounce).
    /// Values > 1.0 make surfaces more absorptive (deader rooms).
    /// 1.0 = physically accurate. 0.5 = half absorption (twice as reflective).
    float mAbsorptionScale = 1.0f;

    /// Per-mission extra probe placement, snapshot at config-load time and
    /// consumed by `bakeProbes`. `mProbeElevations` adds elevated copies
    /// of the floor grid (heights in engine feet). Takes effect only on
    /// the NEXT bake.
    std::vector<float> mProbeElevations = { 10.0f };

    /// Minimum probe-to-wall clearance (engine feet) enforced at bake
    /// time. Probes whose containing room reports a nearer plane are
    /// dropped before any reflection rays are cast. 0 = clearance check
    /// disabled (inside-solid rejection still runs).  Tunable from YAML
    /// (audio.probes.min_wall_clearance_ft) and clamped at the setter.
    float              mProbeMinWallClearanceFt = 5.0f;

    /// Elevation-tier sparsity multiplier; see setProbeElevationSparsityMul.
    /// Default 2.0 = 2×2 binning = 1:4 ratio of elevation to floor probes.
    float              mProbeElevationSparsityMul = 2.0f;

    /// Global dedup pass radius (engine feet); see setProbeGlobalDedupRadiusFt.
    /// Default 2.0 = catches obvious overlaps without trimming the grid.
    float              mProbeGlobalDedupRadiusFt = 2.0f;

    /// Whether to bake a second probe batch (sparse ROOM_PORTAL graph)
    /// for Steam Audio pathing. Default true. When false, the .probes
    /// file contains the reflection batch only and runtime pathing is
    /// effectively disabled (synthetic-bypass branch). Set from yaml
    /// (`audio.pathing_probes.enabled`).
    bool               mProbePathingBatchEnabled = true;

    /// Proximity dedup radius for the PATHING batch (engine feet). 0
    /// disables the pass. Tuned independently from the reflection
    /// batch's global_dedup_radius_ft. Default 10 ft — empirically
    /// drops most compound-doorway clusters in Thief 2 levels without
    /// collapsing legitimately distinct rooms. Set from yaml
    /// (`audio.pathing_probes.dedup_radius_ft`).
    float              mProbePathingDedupRadiusFt = 10.0f;

    /// Reflection-bake skip flag. When true, the next bakeProbes() call
    /// carries the existing `.probes` reflection section forward verbatim
    /// and only re-bakes the pathing batch. See setReflectionBakeSkip()
    /// for the full rationale. Default false. Set from yaml
    /// (`audio.reflections.bake_skip`) and `--skip-reflection-bake` CLI.
    /// Also drives the once-per-30s [REFL_SKIP] staleness reminder in
    /// dumpAudioStatusPeriodic.
    bool               mReflectionBakeSkip = false;

    // Volumetric occlusion sphere radius + sample count moved to
    // AudioOcclusion (mAudioOcclusion). The setters/getters above are
    // facades that forward into it. Larger radius = smoother transitions
    // around corners; the door/local-sound floor in AudioService.cpp
    // raises radius to 16 ft for skipPortalRouting voices so doorways
    // aren't over-occluded by narrow frames.

    /// Propagation layer toggles (debug — all on by default)
    bool mPortalRoutingEnabled = true;   ///< Portal-graph routing through doorways
    bool mProbePathingEnabled = true;    ///< Baked probe diffraction (when available)

    /// Pathing-simulation throttle. iplSimulatorRunPathing runs on the
    /// main loop thread; mPathingUpdateInterval bounds its rate so we
    /// don't pay full CPU cost every frame. mPathingAccumSec accumulates
    /// deltaTime across loopStep calls; mPathingDueThisStep is set true
    /// for the current loopStep whenever the accumulator crosses the
    /// interval (and the accumulator resets). Per-voice
    /// iplSourceSetInputs(...PATHING) calls + the iplSimulatorRunPathing
    /// invocation are gated on this flag. Default 0.1 s (10 Hz) matches
    /// Unity/Unreal Steam Audio integration defaults. 0.0 = every frame.
    float mPathingUpdateInterval = 0.1f;
    float mPathingAccumSec       = 0.0f;
    bool  mPathingDueThisStep    = false;

    // ── Master bus DSP chain + mixer + spatialization + door LPF config ──
    //
    // Limiter / compressor / EQ / ducker tuning, the global master/direct/
    // reflection gains, the HRTF/spatial-blend/distance-model knobs and the
    // door LPF + propagation-min-attenuation values all live in
    // AudioDSPChain. AudioService still seeds the live ReflectionMixNode
    // from these values inside initReflectionPipeline().
    std::unique_ptr<AudioDSPChain> mDSPChain = std::make_unique<AudioDSPChain>();

    // ── Propagation tuning (portal graph) ──
    float    mPropagationMaxDist  = 200.0f;
    // N-path BFS knobs. Default 4 = matches the multi-path-ambisonics
    // sub-source slot cap (kMaxSubSources, see VoicePool.h), so the
    // BFS delivers enough paths to keep every reachable doorway
    // simultaneously active. Under the legacy default of 2 the slot
    // assigner could only ever drive two slots ACTIVE, and the SET of
    // kept paths swapped as the listener moved, producing audible
    // loudness jumps when a third reachable path entered or left the
    // set (the 40× distAtt swing diagnosed at the miss6 (68,56,-36)
    // m06wingedM source when the listener traverses the three-rooms-
    // side-by-side region). Setters clamp to legal ranges; values
    // plumbed through SoundPropParams in propagateSound.
    uint32_t mPropMaxPaths        = 4;
    // maxPathDiff is the effective-distance window for "alternate path
    // worth keeping": at each BFS-reached room, any path whose
    // effDist is ≥ primary's effDist + maxPathDiff is PRUNED — and
    // because the pruning runs mid-BFS (at intermediate rooms, not
    // only at the listener room), alternates die early and never get
    // a chance to reach the listener. The legacy default of 10 came
    // from the original engine's kMaxDistDiff and was tuned for a
    // single-source renderer where alternates only modulated a
    // centroid by a few percent. For the multi-path renderer a tight
    // window discards physically-meaningful alternates whose
    // direction-of-arrival differs sharply from the primary —
    // producing the "sound goes into the wall" perception when the
    // primary path's anchor lands at a wall-edge but a longer
    // alternate enters through a clearly-open doorway. 50 (the
    // setter's clamp ceiling) keeps essentially every distinct-
    // predecessor alternate within the propagation horizon active,
    // so the per-slot renderer always has the unblocked doorway as
    // one of its sub-sources.
    float    mPropMaxPathDiff     = 50.0f;

    /// Multiplier on the scalar gain emitted by `eqCoeffsToDspMapping`.
    /// 1.0 = identity. Tunable at runtime via yaml/console to compensate
    /// for under-amplitude baked pathing without re-baking.
    float    mPathingGainScale    = 1.0f;

    /// Multiplier on the LPF blocking factor emitted by
    /// `eqCoeffsToDspMapping`. Range [0, 1]. 1.0 = identity (legacy
    /// behaviour). Lower values keep the door-LPF more open so distant
    /// cross-room ambients don't drop to ~400 Hz cutoff just because
    /// their eqHigh band is near zero.
    float    mPathingBlockingScale = 1.0f;

    /// Per-band weights applied when collapsing Steam Audio's eqCoeffs
    /// into the scalar portalAttenuation gain. Default {0.25, 0.50, 0.25}
    /// matches the legacy hardcoded mid-heavy perceptual weighting.
    /// Components clamped to [0, 1] at the setter; sum is NOT normalised
    /// (callers use unit-sum weights for level-neutral shaping; sums >1
    /// produce a flat boost).
    float    mPathingGainWeightLow  = 0.25f;
    float    mPathingGainWeightMid  = 0.50f;
    float    mPathingGainWeightHigh = 0.25f;

    // ── Ambient tuning (P$AmbientHack) ──
    // Moved to AmbientSoundManager — the setAmb* facade methods on this
    // service forward to mAmbientManager.

    // ── Performance/infrastructure (must be set BEFORE init/buildAcousticScene) ──
    int         mMaxActiveVoicesCfg        = MAX_ACTIVE_VOICES;
    int         mSimMaxOcclusionSamplesCfg = 32;
    // Thread budget for reverb work (convolution + sim combined). The
    // share knob divides it; the actual derived counts land in
    // mConvolutionWorkerCount / mSimulatorThreadCount at init time.
    // 0 = auto: hwconc - 2 (reserve 2 cores for main + audio).
    int         mReverbThreadsCfg          = 0;
    // -1 = auto split (depends on whether realtime is enabled). Else
    // literal fraction in [0.0, 1.0] of the budget assigned to
    // convolution workers; remainder goes to simulator threads.
    float       mReverbThreadsConvShareCfg = -1.0f;
    std::string mSceneTypeCfg              = "default"; // "default" or "embree"
    int         mAudioSampleRateCfg        = static_cast<int>(kDefaultDeviceSampleRate);
    int         mAudioFrameSizeCfg         = 512;
    int         mSoundCacheMBCfg           = 64;

    // ── Baked probe pathing ──
    //
    // The probe batch, position mirror, and bake/load orchestration live in
    // ProbeManager (audio/ProbeManager.h). AudioService keeps thin facades
    // for bakeProbes/loadProbes and the spacing/height/positions accessors.

    std::unique_ptr<class ProbeManager> mProbeManager;

    /// Per-probe reachability classification, parallel to
    /// mProbeManager->getProbePositions(). Cleared on every load/bake and
    /// recomputed by classifyProbeReachability(). Used only by the debug
    /// probe overlay — runtime sim ignores it. See ProbeFate for buckets.
    std::vector<ProbeFate> mProbeFates;

    /// True while any voice flagged playerEmitted is producing sound. Set by
    /// loopStep() on the main thread, read by the renderer for the debug
    /// listener-marker overlay. Atomic so renderer can read without locking.
    std::atomic<bool> mPlayerEmittedActive{false};

    /// Scene bounding box (computed during buildAcousticScene)
    Vector3 mSceneMin{0, 0, 0};
    Vector3 mSceneMax{0, 0, 0};

    /// Derived counts populated at initReflectionPipeline time from
    /// mReverbThreadsCfg + mReverbThreadsConvShareCfg. Sum == budget.
    /// Rate divisor lives on mReflectionSim now.
    int mConvolutionWorkerCount = 0;
    int mSimulatorThreadCount   = 0;

    /// Ambisonics order for reflections (0 = 1 channel, 1 = 4 channels).
    /// Order 0 is 4x cheaper per voice. Set before buildAcousticScene().
    int mAmbisonicsOrder = 0;
    int mAmbisonicsChannels = 1;  ///< (mAmbisonicsOrder+1)^2, computed at init

    /// Reflection pipeline sample rate and frame size (derived from
    /// mReflectionRateDivisor). Initial values are engine defaults from
    /// AudioUnits.h — overwritten at init once the rate divisor is known.
    uint32_t mReflectionSampleRate = kDefaultDeviceSampleRate;
    uint32_t mReflectionFrameSize = kDefaultDeviceFrameSize;

    // ── Background simulation threads ──
    //
    // Two SEPARATE threads for direct and reflection simulation so that the
    // expensive reflection ray trace never blocks the latency-critical direct sim:
    // - Direct sim (occlusion/attenuation): runs every frame, cheap, <2ms
    // - Reflection sim (ray-traced reverb): runs throttled, expensive, 50-200ms
    //
    // Source mutations (add/remove/commit) must wait for BOTH threads to be idle.
    //
    // The reflection-side thread + mutex + CV + want/running/shutdown atomics,
    // the deferred source-add/remove queues, the throttle counter, the rate
    // divisor and the active-source counter ALL live on mReflectionSim.

    // Room-explicit propagateSound is now SoundPropagation's responsibility
    // and takes a RoomID (int32_t) — see SoundPropagation::propagateSound.
    // Resolving Room* from an ID at call time eliminates the latent
    // dangling-pointer hazard the old `Room *` overload had if the room
    // database was rebuilt between caller capture and BFS.

    // ── Listener state (updated each frame from render binary) ──

    Vector3 mListenerPos{0.0f, 0.0f, 0.0f};
    float mListenerYaw = 0.0f;
    float mListenerPitch = 0.0f;

    /// Cached listener room — updated incrementally each frame via portal
    /// adjacency instead of brute-force roomFromPoint every frame.
    Room *mListenerRoom = nullptr;

    /// Whether the player's feet are currently in water (for footstep override)
    bool mPlayerInWater = false;

    // ── Sound emission pub/sub (Unit AI-hearing runtime) ──

    /// Registered listeners. Called synchronously inside publishSoundEmission.
    std::vector<SoundEmissionListener> mSoundEmissionListeners;

    /// Dispatch a sound emission event to all registered listeners.
    /// Called from audio code whenever a new voice/ambient/footstep starts.
    void publishSoundEmission(const SoundEmissionEvent &ev);

    // ── Audio perf-capture JSONL sink state (PLAN.AUDIO_PROFILING.md §1.1) ──
    //
    // Per-run audio_perf.jsonl artifact file. Owned by AudioService so that
    // the once-per-window snapshot inside dumpAudioStatusPeriodic can fan
    // out to both stderr (AUDIO_LOG) AND the file with NO extra
    // snapshotAndReset(true) call — see PLAN.AUDIO_PROFILING.md §1.7 #1
    // (histogram-ownership pitfall guard, lesson from commit bdeb98b).
    //
    // mPerfJsonlFile is the canonical write target; nullptr = never opened
    // or already closed. mPerfJsonlPath is kept for one-shot init logging.
    // mPerfJsonlStartedAt is the UTC time captured at open; embedded into
    // the run.meta header AND used to compute relative `ts_ms` on every
    // perf.window record (so timelines align with the actual run start,
    // not the program's hardcoded steady_clock epoch).
    std::FILE*                                  mPerfJsonlFile = nullptr;
    std::string                                 mPerfJsonlPath;
    std::chrono::steady_clock::time_point       mPerfJsonlStartedAt;

    // Private helpers
    bool initMiniaudio();
    void shutdownMiniaudio();
    bool initSteamAudio();
    void shutdownSteamAudio();
    void destroyAcousticScene();
    bool initReflectionPipeline();
    void destroyReflectionPipeline();
    void createVoiceSource(ActiveVoice &voice);
    void removeVoiceSource(ActiveVoice &voice);

    /// Reflection-source demote fallback (stage 2.2):
    /// Stage 2.2 demote fallback now lives on ReflectionSimulator
    /// (mReflectionSim->demoteVoice). loopStep calls it directly when a
    /// voice has stayed outside the top-N pool for the configured
    /// hysteresis frames.
    void initVoiceDSP(ActiveVoice &voice);
    int selectSample(const std::string &schemaName, int sampleCount, int totalFreq,
                     const int *frequencies);
    SoundHandle startVoice(const std::string &schemaName, const std::string &sampleName,
                           const Vector3 &position, int priority, bool looping,
                           int objID, float volume = 1.0f,
                           VoiceClass cls = VoiceClass::Normal);
    bool evictLowestPriority(int newPriority);
};

/// Factory for AudioService
class AudioServiceFactory : public ServiceFactory {
public:
    AudioServiceFactory();
    ~AudioServiceFactory() {};

    Service *createInstance(ServiceManager *manager) override;
    const std::string &getName() override;
    const uint getMask() override;
    const size_t getSID() override;

private:
    static const std::string mName;
};

} // namespace Darkness

#endif // __AUDIOSERVICE_H
