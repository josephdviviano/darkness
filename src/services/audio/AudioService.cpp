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

#include "AudioService.h"
#include "CRFSoundLoader.h"
#include "ServiceCommon.h"
#include <algorithm>
#include <atomic>
#include "DarknessServiceManager.h"
#include "database/DatabaseService.h"
#include "loop/LoopService.h"
#include "room/RoomService.h"
#include "property/PropertyService.h"
#include "logger.h"

// miniaudio — single-header C library, implementation compiled here
#define MINIAUDIO_IMPLEMENTATION
#include <miniaudio.h>

// Steam Audio C API
#include <phonon.h>

namespace Darkness {

/*----------------------------------------------------*/
/*---------- Acoustic Material Preset Table ----------*/
/*----------------------------------------------------*/

/// Steam Audio material properties for common surface types.
/// Values are per-frequency-band: {400 Hz, 2.5 kHz, 15 kHz}.
/// Based on published acoustic absorption/transmission coefficients
/// (see NOTES.AUDIO_ENGINE.md for sources and reasoning).
struct AcousticMaterialEntry {
    const char *keyword;   // substring to match in texture names
    IPLMaterial material;
};

// Sorted by keyword length descending to prevent false matches
// (e.g. "car" matching before "carpet"). See NOTES.AUDIO_ENGINE.md.
static const AcousticMaterialEntry kAcousticMaterials[] = {
    {"concrete", {{ 0.05f, 0.07f, 0.08f }, 0.05f, { 0.015f, 0.015f, 0.015f }}},
    {"ceramic",  {{ 0.01f, 0.02f, 0.02f }, 0.05f, { 0.060f, 0.044f, 0.011f }}},
    {"plaster",  {{ 0.12f, 0.06f, 0.04f }, 0.05f, { 0.056f, 0.056f, 0.004f }}},
    {"carpet",   {{ 0.24f, 0.69f, 0.73f }, 0.05f, { 0.020f, 0.005f, 0.003f }}},
    {"gravel",   {{ 0.60f, 0.70f, 0.80f }, 0.60f, { 0.031f, 0.012f, 0.008f }}},
    {"brick",    {{ 0.03f, 0.04f, 0.07f }, 0.05f, { 0.015f, 0.015f, 0.015f }}},
    {"glass",    {{ 0.06f, 0.03f, 0.02f }, 0.05f, { 0.060f, 0.044f, 0.011f }}},
    {"stone",    {{ 0.13f, 0.20f, 0.24f }, 0.20f, { 0.015f, 0.002f, 0.001f }}},
    {"metal",    {{ 0.20f, 0.07f, 0.06f }, 0.05f, { 0.250f, 0.190f, 0.080f }}},
    {"wood",     {{ 0.11f, 0.07f, 0.06f }, 0.05f, { 0.070f, 0.014f, 0.005f }}},
    {"rock",     {{ 0.13f, 0.20f, 0.24f }, 0.20f, { 0.015f, 0.002f, 0.001f }}},
    {"tile",     {{ 0.01f, 0.02f, 0.02f }, 0.05f, { 0.060f, 0.044f, 0.011f }}},
    {"dirt",     {{ 0.60f, 0.70f, 0.80f }, 0.60f, { 0.031f, 0.012f, 0.008f }}},
    {"ice",      {{ 0.01f, 0.02f, 0.02f }, 0.05f, { 0.060f, 0.044f, 0.011f }}},
};
static const size_t kAcousticMaterialCount =
    sizeof(kAcousticMaterials) / sizeof(kAcousticMaterials[0]);

// Default material for unmatched textures
static const IPLMaterial kGenericMaterial =
    {{ 0.10f, 0.20f, 0.30f }, 0.05f, { 0.100f, 0.050f, 0.030f }};

/// Look up an acoustic material by texture name via substring matching.
/// Longer keywords are checked first to avoid false partial matches.
static IPLMaterial lookupAcousticMaterial(const std::string &texName)
{
    // Convert to lowercase for matching
    std::string lower = texName;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

    for (size_t i = 0; i < kAcousticMaterialCount; ++i) {
        if (lower.find(kAcousticMaterials[i].keyword) != std::string::npos) {
            return kAcousticMaterials[i].material;
        }
    }
    return kGenericMaterial;
}

/*----------------------------------------------------*/
/*-------------- Active Voice Management -------------*/
/*----------------------------------------------------*/

/// An active voice playing through miniaudio. Owns the WAV data, decoder,
/// and sound object. Non-copyable/non-movable because ma_decoder and ma_sound
/// contain internal pointers that would dangle after a move.
///
/// Thread safety: the `finished` flag is set atomically by miniaudio's end
/// callback (audio thread) and read by cleanupFinishedVoices (main thread).
/// This avoids polling ma_sound_at_end() across threads.
struct ActiveVoice {
    SoundData data;        // WAV bytes — must outlive decoder (pointer dependency)
    ma_decoder decoder;    // Decodes WAV → PCM (data source for ma_sound)
    ma_sound sound;        // Playback control (volume, position, state)
    SoundHandle handle = SOUND_HANDLE_INVALID;
    bool initialized = false;
    std::atomic<bool> finished{false};  // Set by end callback on audio thread

    ActiveVoice() {
        std::memset(&decoder, 0, sizeof(decoder));
        std::memset(&sound, 0, sizeof(sound));
    }

    ~ActiveVoice() {
        if (initialized) {
            ma_sound_uninit(&sound);
            ma_decoder_uninit(&decoder);
        }
    }

    // Non-copyable, non-movable (ma_decoder/ma_sound have internal pointers)
    ActiveVoice(const ActiveVoice &) = delete;
    ActiveVoice &operator=(const ActiveVoice &) = delete;
    ActiveVoice(ActiveVoice &&) = delete;
    ActiveVoice &operator=(ActiveVoice &&) = delete;
};

/// miniaudio end callback — called on the audio thread when a sound finishes.
/// Sets the atomic finished flag so the main thread can safely clean up.
static void onSoundEnd(void *pUserData, ma_sound * /*pSound*/)
{
    auto *voice = static_cast<ActiveVoice *>(pUserData);
    if (voice) {
        voice->finished.store(true, std::memory_order_release);
    }
}

/*----------------------------------------------------*/
/*------------------ Audio Service -------------------*/
/*----------------------------------------------------*/
template <>
const size_t ServiceImpl<AudioService>::SID = __SERVICE_ID_AUDIO;

AudioService::AudioService(ServiceManager *manager, const std::string &name)
    : ServiceImpl<AudioService>(manager, name)
{
    // LoopClient definition — runs after input, before renderer
    mLoopClientDef.id = LOOPCLIENT_ID_AUDIO;
    mLoopClientDef.mask = LOOPMODE_INPUT | LOOPMODE_RENDER;
    mLoopClientDef.name = "AudioService";
    mLoopClientDef.priority = LOOPCLIENT_PRIORITY_AUDIO;
}

//------------------------------------------------------
AudioService::~AudioService()
{
    // Voices must be destroyed before the engine (they reference it internally)
    mVoices.clear();
    // Release acoustic scene before the Steam Audio context
    destroyAcousticScene();
    // Ensure backends are shut down even if shutdown() wasn't called
    shutdownSteamAudio();
    shutdownMiniaudio();
}

// ── miniaudio backend ──

//------------------------------------------------------
bool AudioService::initMiniaudio()
{
    mMaEngine = new ma_engine();

    ma_engine_config config = ma_engine_config_init();
    config.channels = 2;           // stereo output
    config.sampleRate = 44100;     // standard sample rate
    config.noDevice = MA_FALSE;    // create output device

    ma_result result = ma_engine_init(&config, mMaEngine);
    if (result != MA_SUCCESS) {
        LOG_ERROR("AudioService: miniaudio init failed (error %d)", result);
        delete mMaEngine;
        mMaEngine = nullptr;
        return false;
    }

    LOG_INFO("AudioService: miniaudio initialized (44100 Hz stereo)");
    return true;
}

//------------------------------------------------------
void AudioService::shutdownMiniaudio()
{
    if (mMaEngine) {
        ma_engine_uninit(mMaEngine);
        delete mMaEngine;
        mMaEngine = nullptr;
        LOG_INFO("AudioService: miniaudio shut down");
    }
}

// ── Steam Audio backend ──

//------------------------------------------------------
bool AudioService::initSteamAudio()
{
    // Create Steam Audio context
    IPLContextSettings contextSettings{};
    contextSettings.version = STEAMAUDIO_VERSION;

    IPLerror err = iplContextCreate(&contextSettings, &mIplContext);
    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("AudioService: Steam Audio context creation failed (error %d)", err);
        return false;
    }

    // Create HRTF for binaural rendering
    IPLAudioSettings audioSettings{};
    audioSettings.samplingRate = 44100;
    audioSettings.frameSize = 1024;  // ~23ms at 44100Hz

    IPLHRTFSettings hrtfSettings{};
    hrtfSettings.type = IPL_HRTFTYPE_DEFAULT;

    err = iplHRTFCreate(mIplContext, &audioSettings, &hrtfSettings, &mIplHrtf);
    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("AudioService: Steam Audio HRTF creation failed (error %d)", err);
        iplContextRelease(&mIplContext);
        mIplContext = nullptr;
        return false;
    }

    LOG_INFO("AudioService: Steam Audio initialized (HRTF ready)");
    return true;
}

//------------------------------------------------------
void AudioService::shutdownSteamAudio()
{
    if (mIplHrtf) {
        iplHRTFRelease(&mIplHrtf);
        mIplHrtf = nullptr;
    }
    if (mIplContext) {
        iplContextRelease(&mIplContext);
        mIplContext = nullptr;
        LOG_INFO("AudioService: Steam Audio shut down");
    }
}

// ── Sound resource loading ──

//------------------------------------------------------
bool AudioService::loadSoundResources(const std::string &resPath)
{
    // Create sound loader (opens snd.crf)
    mSoundLoader = std::make_unique<CRFSoundLoader>(resPath);
    if (!mSoundLoader->isOpen()) {
        LOG_ERROR("AudioService: Failed to open snd.crf from %s", resPath.c_str());
        mSoundLoader.reset();
        return false;
    }

    // Create LRU sound cache (64MB budget)
    mSoundCache = std::make_unique<SoundCache>();

    LOG_INFO("AudioService: Sound resources loaded from %s", resPath.c_str());
    return true;
}

// ── Steam Audio acoustic scene ──

//------------------------------------------------------
bool AudioService::buildAcousticScene(const AcousticSceneData &data)
{
    if (!mIplContext) {
        LOG_ERROR("AudioService: cannot build acoustic scene — Steam Audio not initialized");
        return false;
    }

    // Destroy any existing scene from a previous mission
    destroyAcousticScene();

    size_t numVertices = data.vertices.size() / 3;
    size_t numTriangles = data.indices.size() / 3;

    if (numVertices == 0 || numTriangles == 0) {
        LOG_INFO("AudioService: empty geometry — skipping acoustic scene");
        return false;
    }

    // Step 1: Create IPLScene (Steam Audio's built-in CPU raytracer)
    IPLSceneSettings sceneSettings{};
    sceneSettings.type = IPL_SCENETYPE_DEFAULT;

    IPLerror err = iplSceneCreate(mIplContext, &sceneSettings, &mIplScene);
    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("AudioService: iplSceneCreate failed (error %d)", err);
        return false;
    }

    // Step 2: Build material palette from unique texture names
    // Map each unique texture name to a material index
    std::unordered_map<std::string, int32_t> texToMaterialIdx;
    std::vector<IPLMaterial> materials;

    for (const auto &texName : data.texNames) {
        if (texToMaterialIdx.find(texName) == texToMaterialIdx.end()) {
            texToMaterialIdx[texName] = static_cast<int32_t>(materials.size());
            materials.push_back(lookupAcousticMaterial(texName));
        }
    }

    // Ensure at least one material (generic fallback)
    if (materials.empty()) {
        materials.push_back(kGenericMaterial);
    }

    // Build per-triangle material index array
    std::vector<IPLint32> materialIndices;
    materialIndices.reserve(numTriangles);
    for (const auto &texName : data.texNames) {
        auto it = texToMaterialIdx.find(texName);
        materialIndices.push_back(it != texToMaterialIdx.end() ? it->second : 0);
    }

    // Step 3: Convert vertex data to IPLVector3 array
    // (IPLVector3 is {float x, y, z} — same layout as our flat array)
    std::vector<IPLVector3> iplVertices(numVertices);
    for (size_t i = 0; i < numVertices; ++i) {
        iplVertices[i] = {data.vertices[i * 3],
                          data.vertices[i * 3 + 1],
                          data.vertices[i * 3 + 2]};
    }

    // Convert index data to IPLTriangle array
    std::vector<IPLTriangle> iplTriangles(numTriangles);
    for (size_t i = 0; i < numTriangles; ++i) {
        iplTriangles[i].indices[0] = data.indices[i * 3];
        iplTriangles[i].indices[1] = data.indices[i * 3 + 1];
        iplTriangles[i].indices[2] = data.indices[i * 3 + 2];
    }

    // Step 4: Create static mesh with geometry and material assignments
    IPLStaticMeshSettings meshSettings{};
    meshSettings.numVertices = static_cast<IPLint32>(numVertices);
    meshSettings.numTriangles = static_cast<IPLint32>(numTriangles);
    meshSettings.numMaterials = static_cast<IPLint32>(materials.size());
    meshSettings.vertices = iplVertices.data();
    meshSettings.triangles = iplTriangles.data();
    meshSettings.materialIndices = materialIndices.data();
    meshSettings.materials = materials.data();

    err = iplStaticMeshCreate(mIplScene, &meshSettings, &mIplStaticMesh);
    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("AudioService: iplStaticMeshCreate failed (error %d)", err);
        iplSceneRelease(&mIplScene);
        mIplScene = nullptr;
        return false;
    }

    // Step 5: Add the mesh to the scene and commit (builds BVH acceleration)
    iplStaticMeshAdd(mIplStaticMesh, mIplScene);
    iplSceneCommit(mIplScene);

    // Step 6: Create the simulator for direct occlusion + reflections + reverb
    IPLSimulationSettings simSettings{};
    simSettings.flags = static_cast<IPLSimulationFlags>(
        IPL_SIMULATIONFLAGS_DIRECT | IPL_SIMULATIONFLAGS_REFLECTIONS);
    simSettings.sceneType = IPL_SCENETYPE_DEFAULT;
    simSettings.reflectionType = IPL_REFLECTIONEFFECTTYPE_CONVOLUTION;
    simSettings.maxNumOcclusionSamples = 32;
    simSettings.maxNumRays = 4096;       // rays per simulation step
    simSettings.numDiffuseSamples = 32;
    simSettings.maxDuration = 2.0f;      // max reverb tail (seconds)
    simSettings.maxOrder = 1;            // ambisonics order
    simSettings.maxNumSources = 32;      // voice pool size
    simSettings.numThreads = 2;          // parallel ray tracing
    simSettings.samplingRate = 44100;
    simSettings.frameSize = 1024;

    err = iplSimulatorCreate(mIplContext, &simSettings, &mIplSimulator);
    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("AudioService: iplSimulatorCreate failed (error %d)", err);
        destroyAcousticScene();
        return false;
    }

    // Bind the scene to the simulator
    iplSimulatorSetScene(mIplSimulator, mIplScene);
    iplSimulatorCommit(mIplSimulator);

    mSceneReady = true;

    LOG_INFO("AudioService: acoustic scene built — %zu vertices, %zu triangles, "
             "%zu materials", numVertices, numTriangles, materials.size());
    return true;
}

//------------------------------------------------------
void AudioService::destroyAcousticScene()
{
    mSceneReady = false;

    if (mIplSimulator) {
        iplSimulatorRelease(&mIplSimulator);
        mIplSimulator = nullptr;
    }
    if (mIplStaticMesh) {
        iplStaticMeshRelease(&mIplStaticMesh);
        mIplStaticMesh = nullptr;
    }
    if (mIplScene) {
        iplSceneRelease(&mIplScene);
        mIplScene = nullptr;
    }
}

// ── Service lifecycle ──

//------------------------------------------------------
bool AudioService::init()
{
    return true;
}

//------------------------------------------------------
void AudioService::bootstrapFinished()
{
    // Acquire service dependencies
    mDbService = GET_SERVICE(DatabaseService);
    mRoomService = GET_SERVICE(RoomService);
    mPropertyService = GET_SERVICE(PropertyService);

    // Register as a database listener — load after rooms are ready
    mDbService->registerListener(this, DBP_AUDIO);

    // Register as a loop client for per-frame updates
    auto loopSvc = GET_SERVICE(LoopService);
    loopSvc->addLoopClient(this);

    // Initialize audio backends
    bool maOk = initMiniaudio();
    bool saOk = initSteamAudio();
    mAudioReady = maOk && saOk;

    if (mAudioReady) {
        LOG_INFO("AudioService: fully initialized");
    } else {
        LOG_ERROR("AudioService: initialized with errors (miniaudio=%s, steam_audio=%s)",
                  maOk ? "ok" : "FAILED", saOk ? "ok" : "FAILED");
    }
}

//------------------------------------------------------
void AudioService::shutdown()
{
    haltAll();

    // Release sound resources
    mSoundCache.reset();
    mSoundLoader.reset();

    // Shut down audio backends
    shutdownSteamAudio();
    shutdownMiniaudio();
    mAudioReady = false;

    // Unregister loop client
    auto loopSvc = GET_SERVICE(LoopService);
    loopSvc->removeLoopClient(this);

    // Unregister database listener
    if (mDbService) {
        mDbService->unregisterListener(this);
        mDbService.reset();
    }

    mRoomService.reset();
    mPropertyService.reset();

    LOG_INFO("AudioService: shut down");
}

//------------------------------------------------------
void AudioService::onDBLoad(const FileGroupPtr &db, uint32_t curmask)
{
    if (!(curmask & DBM_MIS_DATA))
        return;

    // TODO (Task 34): Load and parse .sch schema files
    // NOTE: Steam Audio scene is built via buildAcousticScene() called from DarknessRender.cpp
    // TODO (Task 39): Load ambient sound properties (P$AmbientHack)

    // Test sound playback — load and play a sound effect from snd.crf
    // to verify the full pipeline (CRF → WAV → miniaudio → speakers)
    if (mSoundLoader && mAudioReady) {
        static const char *testNames[] = {"swoosh1", "bowpull", "gate1",
                                          "doormtl1", "swordhi1"};
        for (const char *name : testNames) {
            SoundData snd = mSoundLoader->loadSound(name);
            if (!snd.valid())
                continue;

            auto voice = std::make_unique<ActiveVoice>();
            voice->data = std::move(snd);
            voice->handle = mNextHandle++;

            // Decode WAV from memory → PCM
            ma_decoder_config cfg = ma_decoder_config_init(ma_format_f32, 0, 0);
            if (ma_decoder_init_memory(voice->data.wavData.data(),
                                       voice->data.wavData.size(),
                                       &cfg, &voice->decoder) != MA_SUCCESS) {
                LOG_ERROR("AudioService: failed to decode test sound '%s'", name);
                continue;
            }

            // Create sound from decoder (no spatialization for this test)
            if (ma_sound_init_from_data_source(
                    mMaEngine, &voice->decoder,
                    MA_SOUND_FLAG_NO_SPATIALIZATION, nullptr,
                    &voice->sound) != MA_SUCCESS) {
                ma_decoder_uninit(&voice->decoder);
                LOG_ERROR("AudioService: failed to init test sound '%s'", name);
                continue;
            }

            voice->initialized = true;

            // Register end callback so the audio thread signals completion
            // via the atomic flag (avoids cross-thread ma_sound_at_end polling)
            ma_sound_set_end_callback(&voice->sound, onSoundEnd, voice.get());

            ma_result startResult = ma_sound_start(&voice->sound);
            if (startResult != MA_SUCCESS) {
                LOG_ERROR("AudioService: failed to start test sound '%s' (error %d)",
                          name, startResult);
                continue;
            }

            LOG_INFO("AudioService: playing test sound '%s' (%zu bytes)",
                     name, voice->data.sizeBytes());
            mVoices[voice->handle] = std::move(voice);
            break; // Only play one test sound
        }
    }

    LOG_INFO("AudioService: mission loaded");
}

//------------------------------------------------------
void AudioService::onDBSave(const FileGroupPtr &db, uint32_t tgtmask)
{
    // Audio state is not saved — nothing to do
}

//------------------------------------------------------
void AudioService::onDBDrop(uint32_t dropmask)
{
    if (!(dropmask & DBM_MIS_DATA))
        return;

    haltAll();
    mBlockingFactors.clear();
    mNextHandle = 0;

    // Flush the sound cache on mission unload (sounds may differ between missions)
    if (mSoundCache) {
        mSoundCache->clear();
    }

    // Release Steam Audio acoustic scene
    destroyAcousticScene();

    // TODO (Task 34): Clear schema database

    LOG_INFO("AudioService: mission audio state cleared");
}

//------------------------------------------------------
void AudioService::loopStep(float deltaTime)
{
    if (!mAudioReady)
        return;

    // Remove voices that have finished playback
    cleanupFinishedVoices();

    // TODO (Task 36): Update active voices — update Steam Audio source
    //   positions for moving objects, run simulation step
}

//------------------------------------------------------
void AudioService::cleanupFinishedVoices()
{
    // Only check the atomic flag (set by the audio thread's end callback).
    // This avoids cross-thread calls to ma_sound_at_end().
    for (auto it = mVoices.begin(); it != mVoices.end();) {
        if (it->second->finished.load(std::memory_order_acquire)) {
            it = mVoices.erase(it);
        } else {
            ++it;
        }
    }
}

// ── Public API stubs ──

//------------------------------------------------------
SoundHandle AudioService::playSchema(const std::string &schemaName,
                                     const Vector3 &position)
{
    if (!mAudioReady)
        return SOUND_HANDLE_INVALID;

    // TODO (Task 36): Resolve schema, pick sample, start playback
    LOG_DEBUG("AudioService::playSchema('%s') — not yet implemented",
              schemaName.c_str());
    return SOUND_HANDLE_INVALID;
}

//------------------------------------------------------
SoundHandle AudioService::playSchemaOnObj(const std::string &schemaName,
                                          int objID)
{
    if (!mAudioReady)
        return SOUND_HANDLE_INVALID;

    // TODO (Task 36): Resolve schema, attach to object, start playback
    LOG_DEBUG("AudioService::playSchemaOnObj('%s', %d) — not yet implemented",
              schemaName.c_str(), objID);
    return SOUND_HANDLE_INVALID;
}

//------------------------------------------------------
void AudioService::haltSound(SoundHandle handle)
{
    if (handle == SOUND_HANDLE_INVALID)
        return;

    auto it = mVoices.find(handle);
    if (it != mVoices.end()) {
        if (it->second->initialized) {
            ma_sound_stop(&it->second->sound);
        }
        mVoices.erase(it);
    }
}

//------------------------------------------------------
void AudioService::haltAll()
{
    // Stop all active sounds before destroying voices
    for (auto &[handle, voice] : mVoices) {
        if (voice->initialized) {
            ma_sound_stop(&voice->sound);
        }
    }
    mVoices.clear();
    mNextHandle = 0;
}

//------------------------------------------------------
void AudioService::setBlockingFactor(int room1, int room2, float factor)
{
    // Store bidirectionally so lookup works in either direction
    uint32_t key1 = (static_cast<uint32_t>(room1) << 16) |
                     static_cast<uint32_t>(room2 & 0xFFFF);
    uint32_t key2 = (static_cast<uint32_t>(room2) << 16) |
                     static_cast<uint32_t>(room1 & 0xFFFF);

    if (factor <= 0.0f) {
        // Remove blocking (fully open)
        mBlockingFactors.erase(key1);
        mBlockingFactors.erase(key2);
    } else {
        mBlockingFactors[key1] = factor;
        mBlockingFactors[key2] = factor;
    }
}

//------------------------------------------------------
float AudioService::getBlockingFactor(int room1, int room2) const
{
    uint32_t key = (static_cast<uint32_t>(room1) << 16) |
                    static_cast<uint32_t>(room2 & 0xFFFF);
    auto it = mBlockingFactors.find(key);
    return (it != mBlockingFactors.end()) ? it->second : 0.0f;
}

/*---------------------- Factory ----------------------*/
const std::string AudioServiceFactory::mName = "AudioService";

AudioServiceFactory::AudioServiceFactory() : ServiceFactory() {}

const std::string &AudioServiceFactory::getName() { return mName; }

const uint AudioServiceFactory::getMask()
{
    return SERVICE_ENGINE | SERVICE_DATABASE_LISTENER;
}

const size_t AudioServiceFactory::getSID() { return AudioService::SID; }

Service *AudioServiceFactory::createInstance(ServiceManager *manager)
{
    return new AudioService(manager, mName);
}

} // namespace Darkness
