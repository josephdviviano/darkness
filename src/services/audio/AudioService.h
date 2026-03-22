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

#include <cstdint>
#include <memory>
#include <string>
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

namespace Darkness {

// Forward declarations for sound resource types (defined in CRFSoundLoader.h)
class CRFSoundLoader;
class SoundCache;
struct SoundData;

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

/// Active voice — owns WAV data, miniaudio decoder + sound (defined in AudioService.cpp)
struct ActiveVoice;

/// Handle to an active sound (returned by play functions, used to halt/query)
using SoundHandle = int32_t;

/// Invalid sound handle sentinel
constexpr SoundHandle SOUND_HANDLE_INVALID = -1;

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
     *  @param resPath  Path to Thief 2 RES directory (containing snd.crf)
     *  @return true if snd.crf opened successfully */
    bool loadSoundResources(const std::string &resPath);

    /** Build Steam Audio acoustic scene from world geometry.
     *  Called from the render binary after WR chunks are parsed.
     *  Creates IPLScene + IPLStaticMesh + IPLSimulator for ray-traced acoustics.
     *  @param data  World geometry — vertices, triangle indices, per-triangle texture names
     *  @return true if scene built successfully */
    bool buildAcousticScene(const AcousticSceneData &data);

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

    // ── Sound resource loading ──

    /// CRF sound loader (opens snd.crf via zziplib)
    std::unique_ptr<CRFSoundLoader> mSoundLoader;

    /// LRU cache for recently-used decoded sounds (default 64MB budget)
    std::unique_ptr<SoundCache> mSoundCache;

    // ── Active voice management ──

    /// Map of active voices (handle → voice). Voices own their WAV data,
    /// miniaudio decoder, and sound object. Cleaned up in loopStep().
    std::unordered_map<SoundHandle, std::unique_ptr<ActiveVoice>> mVoices;

    /// Remove finished voices from the active map
    void cleanupFinishedVoices();

    // ── Steam Audio acoustic scene ──

    /// Acoustic scene built from WR world geometry
    IPLScene mIplScene = nullptr;

    /// Static mesh within the scene (world geometry + material assignments)
    IPLStaticMesh mIplStaticMesh = nullptr;

    /// Simulator for direct/reflection/reverb computation
    IPLSimulator mIplSimulator = nullptr;

    /// Whether an acoustic scene is currently active (built and committed)
    bool mSceneReady = false;

    // Private helpers
    bool initMiniaudio();
    void shutdownMiniaudio();
    bool initSteamAudio();
    void shutdownSteamAudio();
    void destroyAcousticScene();
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
