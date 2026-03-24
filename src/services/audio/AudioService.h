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

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// Forward declarations for audio backend types (defined in .cpp via C headers)
struct ma_engine;
// Steam Audio uses DECLARE_OPAQUE_HANDLE: typedef struct _IPLFoo_t* IPLFoo
struct _IPLContext_t;
typedef _IPLContext_t* IPLContext;
struct _IPLHRTF_t;
typedef _IPLHRTF_t* IPLHRTF;
struct _IPLScene_t;
typedef _IPLScene_t* IPLScene;
struct _IPLStaticMesh_t;
typedef _IPLStaticMesh_t* IPLStaticMesh;
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

namespace Darkness {

// Forward declarations for sound resource types (defined in CRFSoundLoader.h)
class CRFSoundLoader;
class SoundCache;
struct SoundData;
class SchemaParser;

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

/// Handle to an active sound (returned by play functions, used to halt/query)
using SoundHandle = int32_t;

/// Invalid sound handle sentinel
constexpr SoundHandle SOUND_HANDLE_INVALID = -1;

/// Active voice — owns WAV data, miniaudio decoder + sound (defined in AudioService.cpp)
struct ActiveVoice;

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

/// Tracked ambient sound instance (attached to a mission object)
struct AmbientSound {
    int objID = 0;                    ///< Mission object ID
    std::string schemaName;           ///< Schema to play
    Vector3 position{0, 0, 0};       ///< World-space position
    float radius = 0.0f;             ///< Propagation radius
    int32_t volume = -1;              ///< Volume in millibels
    uint32_t flags = 0;               ///< AmbientHackFlags
    SoundHandle handle = SOUND_HANDLE_INVALID; ///< Active voice handle (if playing)
    float smoothedEffDist = 1e9f;     ///< Temporally smoothed effective distance (EMA)
    float fadeInTimer = 0.0f;         ///< Seconds remaining for fade-in ramp (0 = done)
};

/// Maximum simultaneous active voices (matches Dark Engine's limit)
constexpr int MAX_ACTIVE_VOICES = 64;

/// Default maximum voices with reflection convolution enabled simultaneously.
/// Per-voice convolution is expensive. Remaining voices use direct path only
/// (HRTF + distance attenuation + occlusion), which is cheap.
/// Tunable at runtime via setMaxReflectionVoices().
constexpr int DEFAULT_MAX_REFLECTION_VOICES = 2;

/// Default maximum sound propagation distance (world units)
constexpr float SOUND_MAX_DIST = 200.0f;

/// Result of sound propagation through the portal graph.
/// Describes how a sound reaches a listener after traversing portals and doors.
struct SoundPropInfo {
    float effectiveDistance = 0.0f;   ///< Distance with blocking penalties applied
    float realDistance = 0.0f;        ///< Actual physical distance through portal chain
    float totalBlocking = 0.0f;      ///< Cumulative blocking factor [0,1] (max of all portals)
    Vector3 virtualPosition{0, 0, 0}; ///< Where the sound appears to come from (last portal center)
    bool reached = false;            ///< Whether the sound can reach the listener
};

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

    /** Halt a specific active sound.
     *  @param handle  Sound handle from playSchema/playSchemaOnObj */
    void haltSound(SoundHandle handle);

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

    void setReflectionNumRays(int n) { mReflectionNumRays = std::max(128, std::min(n, 8192)); }
    int  getReflectionNumRays() const { return mReflectionNumRays; }

    void setReflectionNumBounces(int n) { mReflectionNumBounces = std::max(1, std::min(n, 8)); }
    int  getReflectionNumBounces() const { return mReflectionNumBounces; }

    void setReflectionDuration(float d) { mReflectionDuration = std::max(0.5f, std::min(d, 4.0f)); }
    float getReflectionDuration() const { return mReflectionDuration; }

    void setReflectionThrottle(int n) { mReflectionThrottle = std::max(1, std::min(n, 32)); }
    int  getReflectionThrottle() const { return mReflectionThrottle; }

    void setMaxReflectionVoices(int n) { mMaxReflectionVoices = std::max(1, std::min(n, MAX_ACTIVE_VOICES)); }
    int  getMaxReflectionVoices() const { return mMaxReflectionVoices; }

    void setTransmissionScale(float s) { mTransmissionScale = std::max(0.1f, std::min(s, 100.0f)); }
    float getTransmissionScale() const { return mTransmissionScale; }

    // Propagation layer toggles (all on by default)
    void setPortalRoutingEnabled(bool v) { mPortalRoutingEnabled = v; }
    bool getPortalRoutingEnabled() const { return mPortalRoutingEnabled; }
    void setProbePathingEnabled(bool v) { mProbePathingEnabled = v; }
    bool getProbePathingEnabled() const { return mProbePathingEnabled; }

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
                    float spacing = 5.0f, float height = 5.0f);

    /** Load baked probe data from disk and register with simulator.
     *  @param probePath  Path to .probes file
     *  @return true if loaded successfully */
    bool loadProbes(const std::string &probePath);

    /** @return number of loaded probes (0 if none) */
    int getProbeCount() const;

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

    /** Whether reflection convolution runs at half sample rate (24kHz).
     *  Must be set BEFORE buildAcousticScene() — cannot change at runtime.
     *  Half-rate halves convolution cost per voice, allowing more reflection
     *  voices for the same CPU budget. Reverb quality is perceptually
     *  transparent at 24kHz (12kHz bandwidth captures all reverb character). */
    void setHalfRateReflections(bool enabled) { mHalfRateReflections = enabled; }
    bool getHalfRateReflections() const { return mHalfRateReflections; }

    /** Ambisonics order for reflection convolution (0 or 1).
     *  Must be set BEFORE buildAcousticScene() — cannot change at runtime.
     *  Order 0 = 1 channel (omnidirectional reverb), 4x cheaper per voice.
     *  Order 1 = 4 channels (directional reverb), more spatial detail in tails.
     *  Direct path HRTF is unaffected — spatial positioning stays full quality. */
    void setAmbisonicsOrder(int order) { mAmbisonicsOrder = std::max(0, std::min(order, 1)); }
    int  getAmbisonicsOrder() const { return mAmbisonicsOrder; }

    /** @return the actual reflection pipeline sample rate (24000 or 48000) */
    uint32_t getReflectionSampleRate() const { return mReflectionSampleRate; }

    /** @return number of triangles in the current acoustic scene */
    int  getAcousticSceneTriCount() const { return mAcousticTriCount; }

    /** Per-frame audio update — voice cleanup, Steam Audio simulation step.
     *  Called from the render binary's main loop (LoopService is not used
     *  in the render binary; this provides a direct entry point).
     *  @param deltaTime  Frame delta in seconds */
    void updateAudio(float deltaTime);

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
    // ── Service dependencies ──

    DatabaseServicePtr mDbService;
    RoomServicePtr mRoomService;
    PropertyServicePtr mPropertyService;
    ObjectServicePtr mObjectService;

    // ── Portal blocking factors for AI hearing propagation ──
    // Key: (room1 << 16) | room2 (bidirectional — stored both ways)
    std::unordered_map<uint32_t, float> mBlockingFactors;

    /// Next sound handle to assign
    SoundHandle mNextHandle = 0;

    // ── Audio backends ──

    /// miniaudio engine (heap-allocated to keep ma_engine out of header)
    ma_engine *mMaEngine = nullptr;

    /// Steam Audio context
    IPLContext mIplContext = nullptr;

    /// Steam Audio HRTF for binaural rendering
    IPLHRTF mIplHrtf = nullptr;

    /// Whether audio backends initialized successfully
    bool mAudioReady = false;

    /// Actual device sample rate (detected at init, used by Steam Audio)
    uint32_t mDeviceSampleRate = 48000;

    /// Audio processing frame size (aligned with Steam Audio)
    uint32_t mFrameSize = 1024;

    // ── Sound resource loading ──

    /// CRF sound loader (opens snd.crf via zziplib)
    std::unique_ptr<CRFSoundLoader> mSoundLoader;

    /// LRU cache for recently-used decoded sounds (default 64MB budget)
    std::unique_ptr<SoundCache> mSoundCache;

    /// Schema parser (.sch/.spc/.arc files — sound selection rules)
    std::unique_ptr<SchemaParser> mSchemaParser;

    // ── Active voice management ──

    /// Map of active voices (handle → voice). Voices own their WAV data,
    /// miniaudio decoder, and sound object. Cleaned up in loopStep().
    std::unordered_map<SoundHandle, std::unique_ptr<ActiveVoice>> mVoices;

    /// Remove finished voices from the active map
    void cleanupFinishedVoices();

    /// Random number generator for sample selection
    std::mt19937 mRng{std::random_device{}()};

    /// Per-schema last sample index for NO_REPEAT behavior
    std::unordered_map<std::string, int> mLastSampleIdx;

    // ── Ambient sound management ──

    /// Active ambient sounds parsed from P$AmbientHack properties
    std::vector<AmbientSound> mAmbients;

    /// Load ambient sound objects from mission data
    void loadAmbientSounds();

    /// Update ambient volumes based on listener distance (per-frame)
    void updateAmbientVolumes();

    // ── Texture material mapping (for footstep schema selection) ──

    /// Per-room LoudRoom transmission factor (keyed by room ID, default 1.0).
    /// Values < 1.0 dampen sound passing through the room, > 1.0 amplify.
    /// Parsed from P$LoudRoom property on room objects.
    std::unordered_map<int32_t, float> mRoomTransmission;

    /// Per-texture material keyword (indexed by TXLIST texture index).
    /// E.g., "stone", "metal", "wood", "carpet", "generic" (default).
    std::vector<std::string> mTextureMaterials;

    // ── Steam Audio acoustic scene ──

    /// Acoustic scene built from WR world geometry
    IPLScene mIplScene = nullptr;

    /// Static mesh within the scene (world geometry + material assignments)
    IPLStaticMesh mIplStaticMesh = nullptr;

    /// Simulator for direct/reflection/reverb computation
    IPLSimulator mIplSimulator = nullptr;

    /// Whether an acoustic scene is currently active (built and committed)
    bool mSceneReady = false;

    /// Deferred simulator commit flag — set when sources are added/removed,
    /// committed once per frame in loopStep() before simulation runs.
    bool mSimulatorDirty = false;

    // ── Reflection pipeline (convolution reverb → ambisonics → binaural) ──

    /// Shared reflection mixer — accumulates convolution output from all
    /// per-voice reflection effects into a single ambisonics buffer.
    IPLReflectionMixer mIplReflectionMixer = nullptr;

    /// Ambisonics decode effect — converts accumulated reflection ambisonics
    /// (from the mixer) to binaural stereo via HRTF.
    IPLAmbisonicsDecodeEffect mIplAmbiDecodeEffect = nullptr;

    /// Whether reflections are enabled (toggled at runtime with R key)
    bool mReflectionsEnabled = true;

    /// Global reflection mix node — sits between per-voice DSP nodes and the
    /// engine endpoint. Retrieves mixed reflection ambisonics, decodes to
    /// binaural stereo, and adds to the direct audio stream.
    std::unique_ptr<ReflectionMixNode> mReflectionMixNode;

    /// Frame counter for throttling reflection simulation (every Nth frame)
    int mReflectionFrameCounter = 0;

    // ── Tunable reflection parameters ──

    int mReflectionNumRays = 1024;     ///< Rays per simulation step (128–8192)
    int mReflectionNumBounces = 4;     ///< Bounces per ray (1–8)
    float mReflectionDuration = 2.0f;  ///< Max reverb tail in seconds (0.5–4.0)
    int mReflectionThrottle = 4;       ///< Run every Nth frame (1–32)
    int mMaxReflectionVoices = DEFAULT_MAX_REFLECTION_VOICES;
    int mAcousticTriCount = 0;         ///< Triangles in current acoustic scene

    /// Global multiplier for material transmission coefficients.
    /// 1.0 = physically accurate, 10.0 = audible through walls (game-friendly).
    float mTransmissionScale = 10.0f;

    /// Propagation layer toggles (debug — all on by default)
    bool mPortalRoutingEnabled = true;   ///< Portal-graph routing through doorways
    bool mProbePathingEnabled = true;    ///< Baked probe diffraction (when available)

    // ── Baked probe pathing ──

    IPLProbeBatch mIplProbeBatch = nullptr;  ///< Loaded probe data (positions + baked paths + reflections)
    int mProbeCount = 0;                     ///< Number of probes in the batch
    bool mProbesHaveReflections = false;     ///< True if loaded probes contain baked reflection IRs

    /// Scene bounding box (computed during buildAcousticScene)
    Vector3 mSceneMin{0, 0, 0};
    Vector3 mSceneMax{0, 0, 0};

    /// Half-rate reflection mode: convolution at 24kHz instead of 48kHz.
    /// Set before buildAcousticScene(). Halves convolution cost per voice.
    bool mHalfRateReflections = false;

    /// Ambisonics order for reflections (0 = 1 channel, 1 = 4 channels).
    /// Order 0 is 4x cheaper per voice. Set before buildAcousticScene().
    int mAmbisonicsOrder = 0;
    int mAmbisonicsChannels = 1;  ///< (mAmbisonicsOrder+1)^2, computed at init

    /// Reflection pipeline sample rate and frame size (derived from mHalfRateReflections)
    uint32_t mReflectionSampleRate = 48000;
    uint32_t mReflectionFrameSize = 1024;

    // ── Background simulation threads ──
    //
    // Two separate threads for direct and reflection simulation:
    // - Direct sim (occlusion/attenuation): runs every frame, cheap, latency-critical
    // - Reflection sim (ray-traced reverb): runs throttled, expensive, latency-tolerant
    //
    // Source mutations (add/remove/commit) must wait for BOTH threads to be idle.

    /// Persistent simulation worker thread — handles both direct and reflection
    /// simulation requests via condition variable signaling. Eliminates per-frame
    /// std::thread creation overhead (60 pthread_create/join cycles per second).
    std::thread mSimWorkerThread;
    std::mutex mSimWorkerMutex;
    std::condition_variable mSimWorkerCV;

    /// Work requests for the sim worker (protected by mSimWorkerMutex)
    bool mSimWorkerWantDirect = false;
    bool mSimWorkerWantReflections = false;

    /// Completion flags (atomic, read by main thread without lock)
    std::atomic<bool> mDirectSimRunning{false};
    std::atomic<bool> mReflectionSimRunning{false};

    /// Signals worker thread to shut down
    std::atomic<bool> mSimWorkerShutdown{false};

    /// Worker thread entry point
    void simWorkerMain();

    /// IPL sources awaiting deferred removal (accumulated while any sim thread is busy).
    std::vector<IPLSource> mPendingSourceRemovals;

    /// IPL sources awaiting deferred add (accumulated while any sim thread is busy).
    std::vector<IPLSource> mPendingSourceAdds;

    /// Join the background reflection thread if it's running.
    /// Must be called before any source mutation (add/remove/commit)
    /// to prevent Steam Audio from accessing freed source data.
    void waitForReflectionThread();

    // ── Listener state (updated each frame from render binary) ──

    Vector3 mListenerPos{0.0f, 0.0f, 0.0f};
    float mListenerYaw = 0.0f;
    float mListenerPitch = 0.0f;

    /// Whether the player's feet are currently in water (for footstep override)
    bool mPlayerInWater = false;

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
    void initVoiceDSP(ActiveVoice &voice);
    int selectSample(const std::string &schemaName, int sampleCount, int totalFreq,
                     const int *frequencies);
    SoundHandle startVoice(const std::string &schemaName, const std::string &sampleName,
                           const Vector3 &position, int priority, bool looping,
                           int objID, float volume = 1.0f);
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
