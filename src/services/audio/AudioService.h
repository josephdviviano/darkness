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
#include "audio/SchemaTypes.h"
#include "room/RoomService.h"  // SoundPropInfo / SoundPropParams / SoundPathHop

#include <atomic>
#include <condition_variable>
#include <cstdint>
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

// Forward declaration for off-thread convolution worker (defined in AudioService.cpp)

namespace Darkness {

// Forward declarations for room system types
class Room;
class RoomPortal;

// Forward declaration for off-thread convolution worker (defined in AudioService.cpp)
struct ConvolutionWorker;

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

/// Tracked ambient sound instance (attached to a mission object)
struct AmbientSound {
    int objID = 0;                    ///< Mission object ID
    std::string schemaName;           ///< Schema to play
    Vector3 position{0, 0, 0};       ///< World-space position
    float radius = 0.0f;             ///< Propagation radius
    int32_t volume = -1;              ///< Volume in millibels
    uint32_t flags = 0;               ///< AmbientHackFlags
    /// Per-ambient SHARP-falloff flag, captured from the schema's
    /// SCH_SHARP_FALLOFF (= original engine's SFXFLG_SHARP) at load time.
    /// True → use the 4th-power curve `(d/r)^4` in updateAmbientVolumes;
    /// false → linear `(d/r)`. SHARP is the original engine's default
    /// for ambients (most use it); schemas opt out via an explicit
    /// `flags` directive that clears bit 12.
    bool isSharp = true;
    /// Per-schema attenuation-factor divisor for the volume formula,
    /// loaded from P$SchAttFac at startup (default 1.0). Higher values
    /// make this schema fall off less aggressively. Examples in MISS6:
    /// m06bell=20.0, KARRAS_SCRIPTED=3.0, AI_CONV=1.7, HIT_EXPLOSION=2.0.
    float attenuationFactor = 1.0f;
    SoundHandle handle = SOUND_HANDLE_INVALID; ///< Active voice handle (if playing)
};

/// Spot ambient — alternative ambient encoding with a hard inner/outer
/// falloff envelope (loaded from P$SpotAmb). Unlike AmbientSound (single
/// radius, linear/SHARP falloff), spot ambients are flat-volume within
/// the inner radius, linearly fading to silence between inner and outer,
/// and silent beyond. The original engine ties these to specific scene
/// objects (e.g. mech floor lamps in MISS6).
struct SpotAmbient {
    int objID = 0;
    std::string schemaName;       ///< Schema name (resolved from emitter object's archetype)
    Vector3 position{0, 0, 0};
    float inner = 0.0f;           ///< Distance inside which volume = level
    float outer = 0.0f;           ///< Distance beyond which volume = 0
    float level = 1.0f;           ///< Volume scalar at d <= inner
    SoundHandle handle = SOUND_HANDLE_INVALID;
};

/// Maximum simultaneous active voices (matches Dark Engine's limit)
constexpr int MAX_ACTIVE_VOICES = 64;

/// Default maximum voices with reflection convolution enabled simultaneously.
/// Per-voice convolution is expensive. Remaining voices use direct path only
/// (HRTF + distance attenuation + occlusion), which is cheap.
/// Tunable at runtime via setMaxReflectionVoices().
constexpr int DEFAULT_MAX_REFLECTION_VOICES = 16;

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

    void setAbsorptionScale(float s) { mAbsorptionScale = std::max(0.01f, std::min(s, 10.0f)); }
    float getAbsorptionScale() const { return mAbsorptionScale; }

    void setDiffuseSamples(int n) { mDiffuseSamples = std::max(16, std::min(n, 256)); }
    void setBakeDiffuseSamples(int n) { mBakeDiffuseSamples = std::max(32, std::min(n, 512)); }

    void setOcclusionRadius(float r) { mOcclusionRadius = std::max(0.3f, std::min(r, 30.0f)); }
    float getOcclusionRadius() const { return mOcclusionRadius; }
    void setOcclusionSamples(int n) { mOcclusionSamples = std::max(4, std::min(n, 64)); }
    int  getOcclusionSamples() const { return mOcclusionSamples; }

    /// Get the current listener position (for door sound placement, etc.)
    Vector3 getListenerPos() const { return mListenerPos; }

    // Propagation layer toggles (all on by default)
    void setPortalRoutingEnabled(bool v) { mPortalRoutingEnabled = v; }
    bool getPortalRoutingEnabled() const { return mPortalRoutingEnabled; }
    void setProbePathingEnabled(bool v) { mProbePathingEnabled = v; }
    bool getProbePathingEnabled() const { return mProbePathingEnabled; }

    // ── Master bus DSP chain config (applied when reflection pipeline initializes) ──

    /// Configure the soft limiter (prevents digital clipping)
    void setDSPLimiterEnabled(bool v) { mDSPLimiterEnabled = v; }
    void setDSPLimiterKnee(float k) { mDSPLimiterKnee = std::max(0.5f, std::min(k, 0.95f)); }

    /// Configure the master bus compressor (tames transients)
    void setDSPCompressorEnabled(bool v) { mDSPCompressorEnabled = v; }
    void setDSPCompThreshold(float t) { mDSPCompThreshold = std::max(-30.0f, std::min(t, 0.0f)); }
    void setDSPCompRatio(float r) { mDSPCompRatio = std::max(1.5f, std::min(r, 10.0f)); }
    void setDSPCompAttackMs(float ms) { mDSPCompAttackMs = std::max(1.0f, std::min(ms, 100.0f)); }
    void setDSPCompReleaseMs(float ms) { mDSPCompReleaseMs = std::max(50.0f, std::min(ms, 2000.0f)); }

    /// Configure the low-shelf EQ (bass boost/cut)
    void setDSPEQEnabled(bool v) { mDSPEQEnabled = v; }
    void setDSPEQFreq(float f) { mDSPEQFreq = std::max(60.0f, std::min(f, 500.0f)); }
    void setDSPEQGain(float g) { mDSPEQGain = std::max(-6.0f, std::min(g, 6.0f)); }
    void setDSPEQQ(float q) { mDSPEQQ = std::max(0.3f, std::min(q, 2.0f)); }

    /// Configure the ambient ducking system (disabled by default)
    void setDSPDuckingEnabled(bool v) { mDSPDuckingEnabled = v; }
    void setDSPDuckAmount(float a) { mDSPDuckAmount = std::max(0.1f, std::min(a, 1.0f)); }
    void setDSPDuckAttackMs(float ms) { mDSPDuckAttackMs = std::max(10.0f, std::min(ms, 500.0f)); }
    void setDSPDuckReleaseMs(float ms) { mDSPDuckReleaseMs = std::max(50.0f, std::min(ms, 5000.0f)); }

    // ── Mixer / global gains ──
    // Setters push live to the running mix node (if active) so console tweaks
    // take effect immediately without restarting the audio pipeline.
    void  setMasterGain(float g);
    float getMasterGain() const { return mMasterGain; }
    void  setReflectionGain(float g);
    float getReflectionGain() const { return mReflectionGain; }
    /// Dry-bus multiplier (direct path). Independent of master + reflection
    /// gains — lets you tune the direct/indirect ratio for "around the corner"
    /// audibility without raising overall volume.
    void  setDirectGain(float g);
    float getDirectGain() const { return mDirectGain; }
    void  setReflectionRampMs(float ms) { mReflectionRampMs = std::max(1.0f, std::min(ms, 1000.0f)); }
    float getReflectionRampMs() const { return mReflectionRampMs; }

    // ── Spatialization (HRTF + distance model) ──
    /// Must be set BEFORE bootstrapFinished()/init — used during HRTF creation.
    void setHRTFVolume(float v) { mHRTFVolume = std::max(0.0f, std::min(v, 4.0f)); }
    /// "nearest" or "bilinear". Must be set BEFORE per-source effects are created.
    void setHRTFInterpolation(const std::string& s) { mHRTFInterpolation = (s == "nearest" ? "nearest" : "bilinear"); }
    void setSpatialBlend(float b) { mSpatialBlend = std::max(0.0f, std::min(b, 1.0f)); }
    /// "default" or "inverse_distance"
    void setDistanceModel(const std::string& s) { mDistanceModel = (s == "inverse_distance" ? "inverse_distance" : "default"); }

    // ── Propagation tuning ──
    // Setters republish to the audio thread so runtime tweaks (e.g. via the
    // debug console) take effect on the next audio callback.
    void  setPropagationMaxDist(float d) { mPropagationMaxDist = std::max(10.0f, std::min(d, 5000.0f)); }
    float getPropagationMaxDist() const { return mPropagationMaxDist; }
    void  setDoorLpfOpenHz(float hz) {
        mDoorLpfOpenHz = std::max(1000.0f, std::min(hz, 24000.0f));
        publishAudioThreadParams();
    }
    float getDoorLpfOpenHz() const { return mDoorLpfOpenHz; }
    void  setDoorLpfBlockedHz(float hz) {
        mDoorLpfBlockedHz = std::max(100.0f, std::min(hz, 10000.0f));
        publishAudioThreadParams();
    }
    float getDoorLpfBlockedHz() const { return mDoorLpfBlockedHz; }
    void  setPropMinAttenuation(float a) {
        mPropMinAttenuation = std::max(0.0f, std::min(a, 0.1f));
        publishAudioThreadParams();
    }
    float getPropMinAttenuation() const { return mPropMinAttenuation; }

    // ── Ambient tuning ──
    void setAmbHysteresisStartMul(float m) { mAmbHysteresisStartMul = std::max(1.0f, std::min(m, 5.0f)); }
    void setAmbHysteresisStopMul(float m)  { mAmbHysteresisStopMul  = std::max(1.0f, std::min(m, 5.0f)); }
    /// "linear" or "quadratic"
    void setAmbFalloffCurve(const std::string& s) { mAmbFalloffCurve = (s == "linear" ? "linear" : "quadratic"); }
    void setAmbDefaultPriority(int p) { mAmbDefaultPriority = std::max(0, std::min(p, 255)); }
    // Per-voice spatialBlend override applied to AMB_ENVIRONMENTAL ambients
    // at activation time. 1.0 = full HRTF point-source pan; 0.0 = mono
    // passthrough. Lower values make room ambients (wind, church) feel
    // less like they emit from a single point. Object-attached ambients
    // (no AMB_ENVIRONMENTAL flag) ignore this and stay at full HRTF.
    void setAmbEnvironmentalSpatialBlend(float b) {
        mAmbEnvironmentalSpatialBlend = std::max(0.0f, std::min(b, 1.0f));
    }

    // ── Performance tuning (some MUST be set BEFORE buildAcousticScene) ──
    void setMaxActiveVoices(int n) { mMaxActiveVoicesCfg = std::max(8, std::min(n, 256)); }
    int  getMaxActiveVoices() const { return mMaxActiveVoicesCfg; }
    void setSimulatorThreads(int n) { mSimulatorThreadsCfg = std::max(0, std::min(n, 64)); }
    void setSimMaxOcclusionSamples(int n) { mSimMaxOcclusionSamplesCfg = std::max(4, std::min(n, 256)); }
    void setSimMaxRays(int n) { mSimMaxRaysCfg = std::max(128, std::min(n, 16384)); }
    // Direct simulator's source cap — runs synchronously and per-source cost is
    // ~constant, so this can be set much higher than the reflection cap.
    void setDirectMaxSources(int n) { mDirectMaxSourcesCfg = std::max(4, std::min(n, 1024)); }
    // Reflection simulator's source cap — convolution + IR memory scale with
    // this, so it's the expensive one. Keep modest unless quality budget allows.
    void setReflectionMaxSources(int n) { mReflectionMaxSourcesCfg = std::max(4, std::min(n, 256)); }
    // Number of consecutive frames a Normal voice must stay out of the top-N
    // reflection candidate pool before its reflection source is released
    // and the voice goes fully dry for the rest of its life. This is a
    // sparing fallback — default is high (~10 s at 60 fps) so it fires
    // only when the convolution/sim budget is under real pressure.
    void setReflectionDemoteHysteresisFrames(int n) { mReflectionDemoteHysteresisCfg = std::max(1, std::min(n, 3600)); }
    /// "default" or "embree"
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

    /** Load baked probe data from disk and register with simulator.
     *  @param probePath  Path to .probes file
     *  @return true if loaded successfully */
    bool loadProbes(const std::string &probePath);

    /** @return number of loaded probes (0 if none) */
    int getProbeCount() const;

    /** Bake-time grid configuration. Takes effect on the next bakeProbes()
     *  call — does NOT relocate existing probes. Range-clamped to keep CPU
     *  cost reasonable. */
    void setProbeSpacingFt(float ft) { mProbeSpacingFt = std::max(1.0f, std::min(ft, 20.0f)); }
    float getProbeSpacingFt() const { return mProbeSpacingFt; }
    void setProbeHeightFt(float ft) { mProbeHeightFt = std::max(0.5f, std::min(ft, 20.0f)); }
    float getProbeHeightFt() const { return mProbeHeightFt; }

    /** Snapshot of probe positions in feet (engine units). Populated by
     *  bakeProbes() and loadProbes(); empty if no probes are loaded. Used
     *  by the renderer to draw a debug overlay. The vector is rebuilt on
     *  every bake/load, so cache by index — values do not change between
     *  re-bakes. */
    const std::vector<Vector3> &getProbePositions() const { return mProbePositions; }

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
    void setReflectionRateDivisor(int div) { mReflectionRateDivisor = (div >= 4) ? 4 : (div >= 2) ? 2 : 1; }
    int  getReflectionRateDivisor() const { return mReflectionRateDivisor; }

    /// Backward-compatible wrapper for the old bool API
    void setHalfRateReflections(bool enabled) { mReflectionRateDivisor = enabled ? 2 : 1; }
    bool getHalfRateReflections() const { return mReflectionRateDivisor >= 2; }

    /** Number of parallel convolution worker threads (0=auto, based on hardware).
     *  Must be set BEFORE buildAcousticScene() — cannot change at runtime.
     *  More workers enable more simultaneous convolution voices. */
    void setConvolutionWorkerCount(int n) { mConvolutionWorkerCount = std::max(0, std::min(n, 16)); }
    int  getConvolutionWorkerCount() const { return mConvolutionWorkerCount; }

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

    /// Negative cache of sample names that have already failed to load.
    /// Avoids re-attempting the CRF lookup every frame for missing assets
    /// (e.g. mission references a schema/sample not present in snd.crf),
    /// and gates the load-failure log to a single message per sample.
    std::unordered_set<std::string> mFailedSamples;

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

    /// Overlay per-archetype schema property overrides from the gamesys
    /// onto our parsed SchemaEntry table. Reads P$SchPlayPa (play params:
    /// flags, audio class, volume, pan, delay, fade), P$SchLoopPa (loop
    /// params: flags, max samples, loop count, interval), P$SchPriori
    /// (priority override) and P$SchMsg (AI message label). Matches the
    /// schema-archetype object to a SchemaEntry by SymName.
    void loadSchemaPropertyOverrides();

    // ── Spot ambients (Unit B) ──

    /// Active spot-ambient instances parsed from P$SpotAmb properties.
    /// Unlike P$AmbientHack (single radius, linear/SHARP falloff), spot
    /// ambients have inner/outer envelope: full volume inside `inner`,
    /// linear fade to 0 between `inner` and `outer`, silent beyond.
    std::vector<SpotAmbient> mSpotAmbients;

    /// Load spot ambients from mission data (P$SpotAmb on objects).
    void loadSpotAmbients();

    /// Update spot-ambient voice volumes based on listener distance.
    /// Mirrors updateAmbientVolumes for the envelope-falloff model.
    void updateSpotAmbientVolumes();

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

    /// Raw 4-byte AMBIENT chunk contents (loaded as uint32). Mission-
    /// level ambient enable flag or similar — the original engine wrote
    /// this per-mission to control whether the ambient layer is active.
    /// Empty (no value) when the mission has no AMBIENT chunk.
    bool mHasAmbientChunk = false;
    uint32_t mAmbientChunkValue = 0;

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

    /// Simulator for ray-traced reflection/reverb computation. Runs on a
    /// background thread (`reflectionSimWorkerMain`); source-add/remove
    /// must be deferred via `mPendingSourceAdds` / `mPendingSourceRemovals`
    /// when the worker is running, since `iplSimulatorRunReflections`
    /// internally iterates the simulator's source list.
    IPLSimulator mReflectionSimulator = nullptr;

    /// Simulator for direct path (distance attenuation, occlusion, air
    /// absorption, transmission). Runs synchronously on the main thread in
    /// `loopStep`, so its source list is never iterated concurrently —
    /// `iplSourceAdd` / `iplSourceRemove` are always safe to call inline.
    /// Shares `mIplScene` with the reflection simulator (refcounted via
    /// `iplSceneRetain`); world geometry is not duplicated.
    IPLSimulator mDirectSimulator = nullptr;

    /// Whether an acoustic scene is currently active (built and committed).
    /// Atomic as a defensive measure — currently only accessed from the main
    /// thread, but the reflection sim thread could plausibly need it in future.
    std::atomic<bool> mSceneReady{false};

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
    /// engine endpoint. Reads the convolution worker's output and adds to direct.
    std::unique_ptr<ReflectionMixNode> mReflectionMixNode;

    /// Off-thread convolution worker — processes all per-voice reflection
    /// convolution on a dedicated thread with double-buffered stereo output.
    std::unique_ptr<ConvolutionWorker> mConvolutionWorker;

    /// Wait for the convolution worker to finish its current frame.
    /// Must be called before releasing any IPLReflectionEffect.
    void waitForConvolutionWorker();

    /// Convolution sub-worker thread entry point (one per parallel worker).
    void convolutionSubWorkerMain(int workerIdx);

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

    /// Global multiplier for material absorption coefficients.
    /// Values < 1.0 make surfaces more reflective (less absorption per bounce).
    /// Values > 1.0 make surfaces more absorptive (deader rooms).
    /// 1.0 = physically accurate. 0.5 = half absorption (twice as reflective).
    float mAbsorptionScale = 1.0f;

    /// Diffuse scattering samples for real-time reflection simulation (16-256)
    int mDiffuseSamples = 64;
    /// Diffuse scattering samples for probe baking (32-512, higher=smoother)
    int mBakeDiffuseSamples = 128;

    /// Volumetric occlusion source sphere radius (engine feet — converted to
    /// meters at the IPL boundary). Larger = smoother transitions around corners,
    /// smaller = tighter response. ~5 ft = lamp/small source, ~10 = small machine,
    /// ~16 = large machinery; the door/local-sound floor in AudioService.cpp
    /// raises this to 16 ft for skipPortalRouting voices so doorways aren't
    /// over-occluded by narrow frames.
    float mOcclusionRadius = 10.0f;
    /// Number of ray samples for volumetric occlusion (4-64).
    /// More samples = smoother gradient, higher CPU cost per source.
    int mOcclusionSamples = 16;

    /// Propagation layer toggles (debug — all on by default)
    bool mPortalRoutingEnabled = true;   ///< Portal-graph routing through doorways
    bool mProbePathingEnabled = true;    ///< Baked probe diffraction (when available)

    // ── Master bus DSP chain config (stored until reflection pipeline init) ──

    bool  mDSPLimiterEnabled = true;
    float mDSPLimiterKnee = 0.8f;
    bool  mDSPCompressorEnabled = true;
    float mDSPCompThreshold = -15.0f;
    float mDSPCompRatio = 3.0f;
    float mDSPCompAttackMs = 10.0f;
    float mDSPCompReleaseMs = 250.0f;
    bool  mDSPEQEnabled = true;
    float mDSPEQFreq = 120.0f;
    float mDSPEQGain = 3.0f;
    float mDSPEQQ    = 0.707f;
    bool  mDSPDuckingEnabled = false;
    float mDSPDuckAmount = 0.5f;
    float mDSPDuckAttackMs = 50.0f;
    float mDSPDuckReleaseMs = 500.0f;

    // ── Mixer (global gains) ──
    float mMasterGain        = 1.0f;
    float mDirectGain        = 1.0f;
    float mReflectionGain    = 1.0f;
    float mReflectionRampMs  = 10.0f;

    // ── Spatialization (HRTF + distance attenuation) ──
    float       mHRTFVolume        = 1.0f;
    std::string mHRTFInterpolation = "bilinear"; // "nearest" or "bilinear"
    float       mSpatialBlend      = 1.0f;
    std::string mDistanceModel     = "default";  // "default" or "inverse_distance"

    // ── Propagation tuning (portal graph + door blocking) ──
    float mPropagationMaxDist  = 200.0f;
    float mDoorLpfOpenHz       = 20000.0f;
    float mDoorLpfBlockedHz    = 800.0f;
    float mPropMinAttenuation  = 0.001f;

    // ── Ambient tuning (P$AmbientHack) ──
    float       mAmbEnvironmentalSpatialBlend = 0.3f;
    float       mAmbHysteresisStartMul = 1.5f;
    float       mAmbHysteresisStopMul  = 2.0f;
    std::string mAmbFalloffCurve       = "quadratic"; // "linear" or "quadratic"
    int         mAmbDefaultPriority    = 64;

    // ── Performance/infrastructure (must be set BEFORE init/buildAcousticScene) ──
    int         mMaxActiveVoicesCfg        = MAX_ACTIVE_VOICES;
    int         mSimulatorThreadsCfg       = 0;       // 0 = auto (hwconc-2)
    int         mSimMaxOcclusionSamplesCfg = 32;
    int         mSimMaxRaysCfg             = 4096;
    // Source-cap split: direct sim is cheap per source so it gets a larger
    // pool by default; reflection sim is the expensive one and stays modest.
    // Stage 2.2 (lazy reflection-source allocation) will let many more voices
    // exist with direct-only spatialization while only top-N voices hold a
    // reflection source.
    int         mDirectMaxSourcesCfg       = 256;
    int         mReflectionMaxSourcesCfg   = 32;
    // Demote fallback (stage 2.2): every voice starts with a reflection
    // source. When a Normal voice has stayed outside the top-N reflection
    // candidate pool for this many consecutive frames, its reflection
    // source is released and the voice plays dry for the remainder of
    // its life. Default ≈10 s at 60 fps — intentionally high so this is
    // a sparing fallback for genuinely long-lived "stuck distant" voices,
    // not the steady-state path. See AudioService.cpp
    // demoteFromRealtimeReflection and the loopStep demote pass.
    int         mReflectionDemoteHysteresisCfg = 600;
    std::string mSceneTypeCfg              = "default"; // "default" or "embree"
    int         mAudioSampleRateCfg        = 48000;
    int         mAudioFrameSizeCfg         = 1024;
    int         mSoundCacheMBCfg           = 64;

    // ── Baked probe pathing ──

    IPLProbeBatch mIplProbeBatch = nullptr;  ///< Loaded probe data (positions + baked paths + reflections)
    int mProbeCount = 0;                     ///< Number of probes in the batch
    bool mProbesHaveReflections = false;     ///< True if loaded probes contain baked reflection IRs

    /// Grid parameters used at bake time. Read by bakeProbes() if no explicit
    /// spacing/height argument is provided. Live-tunable via console but only
    /// take effect on the next re-bake.
    float mProbeSpacingFt = 5.0f;
    float mProbeHeightFt  = 5.0f;

    /// Probe positions in feet (engine units). Populated by bakeProbes() and
    /// loaded from a sidecar file by loadProbes(). Used purely for debug
    /// overlay rendering — Steam Audio holds the canonical copy internally.
    std::vector<Vector3> mProbePositions;

    /// True while any voice flagged playerEmitted is producing sound. Set by
    /// loopStep() on the main thread, read by the renderer for the debug
    /// listener-marker overlay. Atomic so renderer can read without locking.
    std::atomic<bool> mPlayerEmittedActive{false};

    /// Scene bounding box (computed during buildAcousticScene)
    Vector3 mSceneMin{0, 0, 0};
    Vector3 mSceneMax{0, 0, 0};

    /// Reflection rate divisor: 1=full (48kHz), 2=half (24kHz), 4=quarter (12kHz).
    /// Set before buildAcousticScene(). Higher divisors reduce per-voice cost.
    int mReflectionRateDivisor = 2;

    /// Number of parallel convolution worker threads (0=auto).
    int mConvolutionWorkerCount = 0;

    /// Ambisonics order for reflections (0 = 1 channel, 1 = 4 channels).
    /// Order 0 is 4x cheaper per voice. Set before buildAcousticScene().
    int mAmbisonicsOrder = 0;
    int mAmbisonicsChannels = 1;  ///< (mAmbisonicsOrder+1)^2, computed at init

    /// Reflection pipeline sample rate and frame size (derived from mReflectionRateDivisor)
    uint32_t mReflectionSampleRate = 48000;
    uint32_t mReflectionFrameSize = 1024;

    // ── Background simulation threads ──
    //
    // Two SEPARATE threads for direct and reflection simulation so that the
    // expensive reflection ray trace never blocks the latency-critical direct sim:
    // - Direct sim (occlusion/attenuation): runs every frame, cheap, <2ms
    // - Reflection sim (ray-traced reverb): runs throttled, expensive, 50-200ms
    //
    // Source mutations (add/remove/commit) must wait for BOTH threads to be idle.

    /// Reflection simulation worker — runs ray-traced reverb on a throttled schedule.
    /// Latency-tolerant: reverb tails change slowly with listener movement.
    std::thread mReflectionSimThread;
    std::mutex mReflectionSimMutex;
    std::condition_variable mReflectionSimCV;
    bool mReflectionSimWant = false;  ///< protected by mReflectionSimMutex
    std::atomic<bool> mReflectionSimRunning{false};
    std::atomic<bool> mReflectionSimShutdown{false};
    void reflectionSimWorkerMain();

    /// IPL sources awaiting deferred removal (accumulated while any sim thread is busy).
    std::vector<IPLSource> mPendingSourceRemovals;

    /// IPL sources awaiting deferred add (accumulated while any sim thread is busy).
    std::vector<IPLSource> mPendingSourceAdds;

    /// Count of voices that currently hold a real-time reflection source
    /// (promoted). Incremented in promoteToRealtimeReflection, decremented
    /// in demoteFromRealtimeReflection / removeVoiceSource. Atomic so the
    /// debug console / perf overlay can read without locking. Validates
    /// against leaks in the promote/demote state machine — should equal the
    /// number of voices in mVoices with a non-null reflectionSource.
    std::atomic<int> mActiveReflectionSources{0};

    /// Join the background reflection thread if it's running.
    /// Must be called before any source mutation (add/remove/commit)
    /// to prevent Steam Audio from accessing freed source data.
    void waitForReflectionThread();

    // ── Portal blending for smooth room transitions ──

    /// Per-frame portal blend state. When the listener stands in a doorway
    /// between two rooms, this provides a smooth positional blend between
    /// the propagation results from both rooms, eliminating the flicker from
    /// roomFromPoint alternating between rooms at the boundary.
    struct PortalBlendState {
        bool        active = false;     ///< True when listener is near a portal
        Room       *roomA = nullptr;    ///< Primary room (from roomFromPoint)
        Room       *roomB = nullptr;    ///< Secondary room (far side of portal)
        float       blend = 0.0f;       ///< 0.0 = fully roomA, 1.0 = fully roomB
    };
    PortalBlendState mPortalBlend;

    /// Compute portal blend state for the current listener position.
    /// Called once per frame at the start of loopStep().
    void computePortalBlend();

    /// Propagate sound with portal blending. Uses mPortalBlend to interpolate
    /// between both rooms when the listener straddles a portal boundary.
    SoundPropInfo propagateSoundBlended(const Vector3 &sourcePos,
                                         float maxDist = SOUND_MAX_DIST) const;

    /// Room-explicit propagateSound overload (bypasses internal roomFromPoint).
    SoundPropInfo propagateSound(const Vector3 &sourcePos,
                                  const Vector3 &listenerPos,
                                  Room *sourceRoom, Room *listenerRoom,
                                  float maxDist = SOUND_MAX_DIST) const;

    // ── Listener state (updated each frame from render binary) ──

    Vector3 mListenerPos{0.0f, 0.0f, 0.0f};
    float mListenerYaw = 0.0f;
    float mListenerPitch = 0.0f;

    /// Cached listener room — updated incrementally each frame via portal
    /// adjacency instead of brute-force roomFromPoint every frame.
    Room *mListenerRoom = nullptr;

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

    /// Reflection-source demote fallback (stage 2.2):
    /// Every voice is created with a reflectionSource in createVoiceSource
    /// so the baseline path is "voice has reverb" (baked by default,
    /// upgraded to realtime when it enters top-N via inputs.baked = false).
    /// This helper releases the reflection source for a voice that has
    /// stayed outside the top-N reflection candidate pool for
    /// `mReflectionDemoteHysteresisCfg` consecutive frames — a sparing
    /// fallback that trims `mSourceData[0]` iteration cost when convolution
    /// or sim budget is genuinely under pressure. Once demoted the voice
    /// plays dry for the remainder of its life. PlayerEmitted and Ambient
    /// voices are excluded; the caller in loopStep filters them out.
    /// Release uses `mPendingSourceRemovals` when the sim thread is busy.
    void demoteFromRealtimeReflection(ActiveVoice &voice);
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
