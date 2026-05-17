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
#include "audio/SchemaTypes.h"
#include "room/RoomService.h"  // SoundPropInfo / SoundPropParams / SoundPathHop

#include <atomic>
#include <condition_variable>
#include <cstdint>
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
    Vector3     virtualPos;       ///< prop.virtualPosition (where Steam Audio thinks the source is)
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

/// AmbientSound and SpotAmbient structs are owned by AmbientSoundManager
/// (see AmbientSoundManager.h). The manager is responsible for parsing
/// P$AmbientHack / P$SpotAmb, per-frame voice lifecycle, and applying the
/// unified AmbientVolumeModel.

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
    // AmbientSoundManager is an extracted subsystem that owns ambient
    // lifecycle (P$AmbientHack + P$SpotAmb). It needs to reach this
    // service's private state (mVoicePool, mReflectionMixNode,
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

    void setOcclusionRadius(float r) { mOcclusionRadius = std::max(0.1f, std::min(r, 200.0f)); }
    float getOcclusionRadius() const { return mOcclusionRadius; }
    void setOcclusionSamples(int n) { mOcclusionSamples = std::max(4, std::min(n, 64)); }
    int  getOcclusionSamples() const { return mOcclusionSamples; }

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
    /// "default" or "inverse_distance"
    void setDistanceModel(const std::string& s) { mDSPChain->setDistanceModel(s); }

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

    // ── Ambient tuning (facades — forwarded to AmbientSoundManager) ──
    void setAmbHysteresisStartMul(float m);
    void setAmbHysteresisStopMul(float m);
    /// "linear" or "quadratic"
    void setAmbFalloffCurve(const std::string &s);
    void setAmbDefaultPriority(int p);
    // Per-voice spatialBlend override applied to AMB_ENVIRONMENTAL ambients
    // at activation time. 1.0 = full HRTF point-source pan; 0.0 = mono
    // passthrough. Lower values make room ambients (wind, church) feel
    // less like they emit from a single point. Object-attached ambients
    // (no AMB_ENVIRONMENTAL flag) ignore this and stay at full HRTF.
    void setAmbEnvironmentalSpatialBlend(float b);

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
     *  cost reasonable. Forwards to the underlying ProbeManager. */
    void setProbeSpacingFt(float ft);
    float getProbeSpacingFt() const;
    void setProbeHeightFt(float ft);
    float getProbeHeightFt() const;

    /** Snapshot of probe positions in feet (engine units). Populated by
     *  bakeProbes() and loadProbes(); empty if no probes are loaded. Used
     *  by the renderer to draw a debug overlay. The vector is rebuilt on
     *  every bake/load, so cache by index — values do not change between
     *  re-bakes. */
    const std::vector<Vector3> &getProbePositions() const;

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

    // Voice handle allocation now lives in VoicePool (mVoicePool->allocate()).
    // The previous mNextHandle counter moved with it.

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
    // P$AmbientHack and P$SpotAmb lifecycle (load, per-frame volume
    // updates with hysteresis + falloff) lives in AmbientSoundManager.
    // This service owns the manager and forwards tuning setters to it;
    // it also exposes the manager to the friend class via the back-
    // pointer in AmbientSoundManager.
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
    /// then trigger AmbientSoundManager::loadAmbientSounds() and the
    /// spot-ambient loader. Called from loadSoundResources after the
    /// schema parser is ready.
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
    // Falloff distance to use for ambient volume computation: prefers the
    // BFS `cachedProp.effectiveDistance` when reached, otherwise returns
    // `fallbackEuclideanDist`. Returns the input fallback when the voice
    // is not found (no caller currently relies on that path, but it keeps
    // the helper total).
    float voiceFalloffDistance(SoundHandle handle, float fallbackEuclideanDist) const;
    // Set the voice's per-source BFS-termination distance. Used by the
    // ambient/spot loaders to bake the per-source max audible distance.
    void voiceSetMaxAudibleDist(SoundHandle handle, float maxDist);
    // Override the voice's HRTF/mono spatialBlend (atomic, audio-thread
    // safe). Used for AMB_ENVIRONMENTAL ambients + spot ambients.
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
    /// ambient slot, distinct from spot ambients) that was active at
    /// save time. 0 means "no environmental ambient was active". The
    /// original engine wrote it from AmbientSave() to support save-game
    /// resume; pristine shipping missions ship with 0 (only one Thief 2
    /// mission, miss14, ships with a nonzero pre-set env ambient).
    ///
    /// Spot ambients (P$SpotAmb, owned by AmbientSoundManager) are
    /// orthogonal — they live on individual objects and are started by
    /// player proximity, not gated by this value. This field is captured
    /// for future SAV-file resume + the miss14-style level-start env
    /// ambient case; it is NOT a level-wide enable flag.
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
    // N-path BFS knobs. Default 2 = original Dark Engine behavior (the
    // dual-predecessor scheme + MergeSounds). Setters clamp to legal
    // ranges; values plumbed through SoundPropParams in propagateSound.
    uint32_t mPropMaxPaths        = 2;
    float    mPropMaxPathDiff     = 10.0f;

    // ── Ambient tuning (P$AmbientHack) ──
    // Moved to AmbientSoundManager — the setAmb* facade methods on this
    // service forward to mAmbientManager.

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
    //
    // The probe batch, position mirror, and bake/load orchestration live in
    // ProbeManager (audio/ProbeManager.h). AudioService keeps thin facades
    // for bakeProbes/loadProbes and the spacing/height/positions accessors.

    std::unique_ptr<class ProbeManager> mProbeManager;

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
    /// number of voices in mVoicePool with a non-null reflectionSource.
    std::atomic<int> mActiveReflectionSources{0};

    /// Join the background reflection thread if it's running.
    /// Must be called before any source mutation (add/remove/commit)
    /// to prevent Steam Audio from accessing freed source data.
    void waitForReflectionThread();

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

    // ── Sound emission pub/sub (Unit AI-hearing runtime) ──

    /// Registered listeners. Called synchronously inside publishSoundEmission.
    std::vector<SoundEmissionListener> mSoundEmissionListeners;

    /// Dispatch a sound emission event to all registered listeners.
    /// Called from audio code whenever a new voice/ambient/footstep starts.
    void publishSoundEmission(const SoundEmissionEvent &ev);

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
