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
    // TODO (Task 35): Build Steam Audio IPLScene from world geometry
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

    // TODO (Task 35): Destroy Steam Audio scene
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
    //   positions for moving objects
    // TODO (Task 35): Run Steam Audio simulation step
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
