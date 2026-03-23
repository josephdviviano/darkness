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
#include "AcousticMaterials.h"
#include "CRFSoundLoader.h"
#include "SchemaParser.h"
#include "SchemaTypes.h"
#include "ServiceCommon.h"
#include <algorithm>
#include <atomic>
#include <queue>
#include "DarknessServiceManager.h"
#include "database/DatabaseService.h"
#include "loop/LoopService.h"
#include "object/ObjectService.h"
#include "room/Room.h"
#include "room/RoomPortal.h"
#include "room/RoomService.h"
#include "property/PropertyService.h"
#include "property/TypedProperty.h"
#include "logger.h"

#include <unordered_set>

// miniaudio — single-header C library, implementation compiled here
#define MINIAUDIO_IMPLEMENTATION
#include <miniaudio.h>

// Steam Audio C API
#include <phonon.h>

namespace Darkness {

/*----------------------------------------------------*/
/*---------- Acoustic Material Preset Table ----------*/
/*----------------------------------------------------*/

/// Steam Audio material properties indexed by keyword.
/// Uses the keyword table from AcousticMaterials.h for substring matching,
/// then maps matched keywords to IPLMaterial values here.
/// Values from published acoustic absorption/transmission coefficients
/// (see NOTES.AUDIO_ENGINE.md for sources and reasoning).
static const std::unordered_map<std::string, IPLMaterial> kKeywordToIPLMaterial = {
    //                             absorption{lo, mid, hi}     scat   transmission{lo, mid, hi}
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
    // Aliases — keywords from AcousticMaterials.h that map to the same materials
    {"floor",    {{ 0.13f, 0.20f, 0.24f }, 0.20f, { 0.015f, 0.002f, 0.001f }}}, // → stone
    {"earth",    {{ 0.60f, 0.70f, 0.80f }, 0.60f, { 0.031f, 0.012f, 0.008f }}}, // → dirt
    {"metl",     {{ 0.20f, 0.07f, 0.06f }, 0.05f, { 0.250f, 0.190f, 0.080f }}}, // → metal (Thief 2 abbreviation)
    {"rust",     {{ 0.20f, 0.07f, 0.06f }, 0.05f, { 0.250f, 0.190f, 0.080f }}}, // → metal
    {"iron",     {{ 0.20f, 0.07f, 0.06f }, 0.05f, { 0.250f, 0.190f, 0.080f }}}, // → metal
    {"door",     {{ 0.11f, 0.07f, 0.06f }, 0.05f, { 0.070f, 0.014f, 0.005f }}}, // → wood
    {"gate",     {{ 0.20f, 0.07f, 0.06f }, 0.05f, { 0.250f, 0.190f, 0.080f }}}, // → metal
    {"roof",     {{ 0.01f, 0.02f, 0.02f }, 0.05f, { 0.060f, 0.044f, 0.011f }}}, // → tile
    {"vine",     {{ 0.11f, 0.07f, 0.06f }, 0.05f, { 0.070f, 0.014f, 0.005f }}}, // → wood
    {"leaf",     {{ 0.11f, 0.07f, 0.06f }, 0.05f, { 0.070f, 0.014f, 0.005f }}}, // → wood
    {"bark",     {{ 0.11f, 0.07f, 0.06f }, 0.05f, { 0.070f, 0.014f, 0.005f }}}, // → wood
    {"rug",      {{ 0.24f, 0.69f, 0.73f }, 0.05f, { 0.020f, 0.005f, 0.003f }}}, // → carpet
    {"hay",      {{ 0.24f, 0.69f, 0.73f }, 0.05f, { 0.020f, 0.005f, 0.003f }}}, // → carpet
    {"mud",      {{ 0.60f, 0.70f, 0.80f }, 0.60f, { 0.031f, 0.012f, 0.008f }}}, // → dirt
};

// Default material for unmatched textures
static const IPLMaterial kGenericMaterial =
    {{ 0.10f, 0.20f, 0.30f }, 0.05f, { 0.100f, 0.050f, 0.030f }};

/// Look up an IPLMaterial by texture name via keyword substring matching.
/// Uses the shared keyword table from AcousticMaterials.h for matching,
/// then maps the keyword to IPLMaterial properties.
/// Logs unmatched textures once per name for development diagnostics.
static IPLMaterial lookupAcousticMaterial(const std::string &texName)
{
    std::string keyword = lookupAcousticMaterialKeyword(texName);

    if (keyword == "generic") {
        // Log each unmatched texture name once to help identify keyword gaps
        static std::unordered_set<std::string> sLoggedUnmatched;
        if (sLoggedUnmatched.insert(texName).second) {
            LOG_INFO("AudioService: no acoustic keyword match for texture '%s' "
                     "— using generic material", texName.c_str());
        }
        return kGenericMaterial;
    }

    auto it = kKeywordToIPLMaterial.find(keyword);
    return (it != kKeywordToIPLMaterial.end()) ? it->second : kGenericMaterial;
}

/*----------------------------------------------------*/
/*----------- Steam Audio DSP Processing Node --------*/
/*----------------------------------------------------*/

/// Forward declare the process callback for the custom miniaudio node vtable.
static void steamAudioNodeProcess(ma_node* pNode, const float** ppFramesIn,
                                   ma_uint32* pFrameCountIn,
                                   float** ppFramesOut, ma_uint32* pFrameCountOut);

/// Custom miniaudio node for Steam Audio DSP processing.
/// Sits between a voice's ma_sound (mono) and the engine endpoint (stereo).
/// Applies IPLDirectEffect (frequency-dependent attenuation) followed by
/// IPLBinauralEffect (HRTF spatialization) on the audio thread.
struct SteamAudioDSPNode {
    ma_node_base base;  // Must be first member (miniaudio node graph requirement)

    // Steam Audio effects (per-voice, processed on audio thread)
    IPLDirectEffect directEffect = nullptr;
    IPLBinauralEffect binauralEffect = nullptr;

    // Shared reference (NOT owned — AudioService manages HRTF lifetime)
    IPLHRTF hrtf = nullptr;

    // Processing parameters — written by main thread in loopStep(),
    // read by audio thread in process callback. Potential tearing on
    // partial writes is acceptable: worst case is slightly wrong audio
    // for a single frame (~23ms), which is imperceptible.
    IPLDirectEffectParams directParams{};
    IPLVector3 direction{1.0f, 0.0f, 0.0f};  // listener-to-source (listener-local frame)

    // Scratch buffers for deinterleaved Steam Audio processing
    // (allocated once at init, never reallocated — safe for audio thread)
    std::vector<float> monoScratch;   // mono channel for direct effect in/out
    std::vector<float> stereoL;       // left channel (binaural output)
    std::vector<float> stereoR;       // right channel (binaural output)

    int frameSize = 1024;
    bool effectsReady = false;       // true when effects + node are initialized
    bool nodeInitialized = false;    // true when ma_node_init succeeded

    // Diagnostics (written by audio thread, read by main thread)
    std::atomic<uint64_t> callCount{0};     // how many times process() was called
    std::atomic<float> peakInput{0.0f};     // peak input level seen
    std::atomic<float> peakOutput{0.0f};    // peak output level seen
    std::atomic<uint32_t> lastFrameCount{0}; // last frame count delivered by miniaudio
    std::atomic<float> lastAtten{1.0f};     // last attenuation factor applied
};

/// vtable for the Steam Audio DSP custom node
static ma_node_vtable sSteamAudioNodeVtable = {
    steamAudioNodeProcess,   // onProcess
    nullptr,                 // onGetRequiredInputFrameCount (1:1 input/output ratio)
    1,                       // inputBusCount  (stereo from ma_sound)
    1,                       // outputBusCount (stereo to engine endpoint)
    0                        // flags
};

/// Temporary debug flag: when true, bypass Steam Audio effects and just pass
/// mono input through as dual-mono stereo. Helps isolate node graph issues.
/// DSP bypass level (0=full pipeline, 3=mono passthrough, 4=stereo passthrough)
static std::atomic<int> sDSPBypassLevel{0};

/// Audio-thread callback: applies Steam Audio direct + binaural effects.
/// Reads mono from ma_sound, outputs stereo binaural to engine endpoint.
static void steamAudioNodeProcess(ma_node* pNode, const float** ppFramesIn,
                                   ma_uint32* pFrameCountIn,
                                   float** ppFramesOut, ma_uint32* pFrameCountOut)
{
    auto* node = reinterpret_cast<SteamAudioDSPNode*>(pNode);
    ma_uint32 frameCount = *pFrameCountOut;

    // Limit to available input frames and our buffer size
    if (pFrameCountIn[0] < frameCount)
        frameCount = pFrameCountIn[0];
    if (frameCount > static_cast<ma_uint32>(node->frameSize))
        frameCount = static_cast<ma_uint32>(node->frameSize);

    // Track diagnostics
    node->callCount.fetch_add(1, std::memory_order_relaxed);
    node->lastFrameCount.store(frameCount, std::memory_order_relaxed);

    float* stereoOut = ppFramesOut[0];  // interleaved stereo output

    if (!node->effectsReady || frameCount == 0 || !ppFramesIn[0]) {
        // Output silence
        std::memset(stereoOut, 0, frameCount * 2 * sizeof(float));
        pFrameCountIn[0] = frameCount;
        *pFrameCountOut = frameCount;
        return;
    }

    // Input is interleaved stereo from ma_sound (engine outputs stereo).
    // Downmix to mono for Steam Audio processing.
    const float* stereoIn = ppFramesIn[0];
    float* mono = node->monoScratch.data();

    float peakIn = 0.0f;
    for (ma_uint32 i = 0; i < frameCount; ++i) {
        mono[i] = (stereoIn[i * 2] + stereoIn[i * 2 + 1]) * 0.5f;
        float a = std::fabs(mono[i]);
        if (a > peakIn) peakIn = a;
    }
    node->peakInput.store(peakIn, std::memory_order_relaxed);

    int bypassLevel = sDSPBypassLevel.load(std::memory_order_relaxed);

    // Level 4: stereo passthrough (no processing at all)
    if (bypassLevel >= 4) {
        std::memcpy(stereoOut, stereoIn, frameCount * 2 * sizeof(float));
        node->peakOutput.store(peakIn, std::memory_order_relaxed);
        pFrameCountIn[0] = frameCount;
        *pFrameCountOut = frameCount;
        return;
    }

    // Level 3: mono passthrough (downmix, then output as dual-mono)
    if (bypassLevel >= 3) {
        for (ma_uint32 i = 0; i < frameCount; ++i) {
            stereoOut[i * 2]     = mono[i];
            stereoOut[i * 2 + 1] = mono[i];
        }
        node->peakOutput.store(peakIn, std::memory_order_relaxed);
        pFrameCountIn[0] = frameCount;
        *pFrameCountOut = frameCount;
        return;
    }

    // Levels 0-2: Steam Audio processing
    //
    // Architecture: run binaural HRTF on the raw mono input, then apply direct
    // effect attenuation as a simple post-multiply on the stereo output.
    //
    // Level 0: binaural + direct attenuation (full pipeline)
    // Level 1: direct attenuation only (no binaural — mono passthrough)
    // Level 2: binaural only (no direct attenuation)
    bool runBinaural = (bypassLevel == 0 || bypassLevel == 2) && node->binauralEffect;
    bool runAtten = (bypassLevel == 0 || bypassLevel == 1);

    // Apply binaural effect (mono → stereo HRTF spatialization).
    if (runBinaural) {
        float* binauralMonoPtr = mono;
        IPLAudioBuffer binauralIn{};
        binauralIn.numChannels = 1;
        binauralIn.numSamples = static_cast<IPLint32>(frameCount);
        binauralIn.data = &binauralMonoPtr;

        float* chL = node->stereoL.data();
        float* chR = node->stereoR.data();
        float* stereoChannels[2] = {chL, chR};

        IPLAudioBuffer outBuf{};
        outBuf.numChannels = 2;
        outBuf.numSamples = static_cast<IPLint32>(frameCount);
        outBuf.data = stereoChannels;

        IPLBinauralEffectParams binParams{};
        binParams.direction = node->direction;
        binParams.interpolation = IPL_HRTFINTERPOLATION_BILINEAR;
        binParams.spatialBlend = 1.0f;
        binParams.hrtf = node->hrtf;
        binParams.peakDelays = nullptr;

        iplBinauralEffectApply(node->binauralEffect, &binParams, &binauralIn, &outBuf);

        // Apply direct effect attenuation as post-multiply on stereo output.
        // This gives us distance falloff, occlusion, and air absorption
        // without chaining the IPLDirectEffect into the binaural input.
        if (runAtten) {
            float atten = node->directParams.distanceAttenuation
                        * node->directParams.occlusion;
            float airAvg = (node->directParams.airAbsorption[0]
                          + node->directParams.airAbsorption[1]
                          + node->directParams.airAbsorption[2]) / 3.0f;
            atten *= airAvg;
            if (atten < 0.001f) atten = 0.001f;
            node->lastAtten.store(atten, std::memory_order_relaxed);
            for (ma_uint32 i = 0; i < frameCount; ++i) {
                chL[i] *= atten;
                chR[i] *= atten;
            }
        }

        // Interleave deinterleaved stereo → miniaudio interleaved output
        float peakOut = 0.0f;
        for (ma_uint32 i = 0; i < frameCount; ++i) {
            stereoOut[i * 2]     = chL[i];
            stereoOut[i * 2 + 1] = chR[i];
            float a = std::fabs(chL[i]);
            if (a > peakOut) peakOut = a;
            a = std::fabs(chR[i]);
            if (a > peakOut) peakOut = a;
        }
        node->peakOutput.store(peakOut, std::memory_order_relaxed);
    } else {
        // No binaural — duplicate mono to both channels, apply direct attenuation
        float atten = 1.0f;
        if (runAtten) {
            atten = node->directParams.distanceAttenuation
                  * node->directParams.occlusion;
            float airAvg = (node->directParams.airAbsorption[0]
                          + node->directParams.airAbsorption[1]
                          + node->directParams.airAbsorption[2]) / 3.0f;
            atten *= airAvg;
            if (atten < 0.001f) atten = 0.001f;
        }
        float peakOut = 0.0f;
        for (ma_uint32 i = 0; i < frameCount; ++i) {
            float s = mono[i] * atten;
            stereoOut[i * 2]     = s;
            stereoOut[i * 2 + 1] = s;
            float a = std::fabs(s);
            if (a > peakOut) peakOut = a;
        }
        node->peakOutput.store(peakOut, std::memory_order_relaxed);
    }

    pFrameCountIn[0] = frameCount;
    *pFrameCountOut = frameCount;
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

    // Voice management metadata
    std::string schemaName;            // Schema that spawned this voice
    int priority = 128;                // 0-255, higher = more important
    int objID = 0;                     // Object ID if attached (0 = positional)

    // Steam Audio simulation source (nullptr if scene not ready or non-spatial)
    IPLSource iplSource = nullptr;

    // World-space position for spatial audio (updated for moving objects)
    Vector3 worldPos{0.0f, 0.0f, 0.0f};

    // Steam Audio DSP node — applies direct + binaural effects in audio thread.
    // Connected between ma_sound output and engine endpoint in the node graph.
    SteamAudioDSPNode dspNode;

    ActiveVoice() {
        std::memset(&decoder, 0, sizeof(decoder));
        std::memset(&sound, 0, sizeof(sound));
    }

    ~ActiveVoice() {
        // Stop playback first to prevent audio thread from accessing resources
        if (initialized) {
            ma_sound_stop(&sound);
        }

        // Disconnect and destroy DSP node (detaches from graph, waits for audio thread)
        if (dspNode.nodeInitialized) {
            ma_node_uninit(&dspNode.base, nullptr);
            dspNode.nodeInitialized = false;
        }
        if (dspNode.directEffect) {
            iplDirectEffectRelease(&dspNode.directEffect);
            dspNode.directEffect = nullptr;
        }
        if (dspNode.binauralEffect) {
            iplBinauralEffectRelease(&dspNode.binauralEffect);
            dspNode.binauralEffect = nullptr;
        }
        dspNode.effectsReady = false;

        // Destroy sound and decoder
        if (initialized) {
            ma_sound_uninit(&sound);
            ma_decoder_uninit(&decoder);
        }

        // Release IPLSource (removeVoiceSource should have been called, but be safe)
        if (iplSource) {
            iplSourceRelease(&iplSource);
        }
    }

    // Non-copyable, non-movable (ma_decoder/ma_sound/ma_node have internal pointers)
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

    // Use 48kHz — the standard rate that Steam Audio's HRTF supports.
    // miniaudio handles resampling to the device's native rate (e.g. 96kHz).
    ma_engine_config config = ma_engine_config_init();
    config.channels = 2;           // stereo output
    config.sampleRate = 48000;     // 48kHz for Steam Audio HRTF compatibility
    config.noDevice = MA_FALSE;    // create output device
    config.listenerCount = 1;      // one listener for 3D spatialization
    config.periodSizeInFrames = 1024; // power-of-2 period for Steam Audio HRTF compatibility

    ma_result result = ma_engine_init(&config, mMaEngine);
    if (result != MA_SUCCESS) {
        LOG_ERROR("AudioService: miniaudio init failed (error %d)", result);
        std::fprintf(stderr, "AudioService: miniaudio init FAILED (error %d)\n", result);
        delete mMaEngine;
        mMaEngine = nullptr;
        return false;
    }

    // Store the engine sample rate (48kHz) for Steam Audio to match.
    // The device may run at a different native rate (e.g. 96kHz);
    // miniaudio resamples internally.
    mDeviceSampleRate = ma_engine_get_sample_rate(mMaEngine);

    // Detect the actual audio processing frame size from the device.
    // Must match Steam Audio's frameSize for correct buffer processing.
    ma_device *device = ma_engine_get_device(mMaEngine);
    if (device) {
        ma_uint32 periodSize = device->playback.internalPeriodSizeInFrames;
        if (periodSize > 0) {
            mFrameSize = periodSize;
        }
        std::fprintf(stderr, "AudioService: miniaudio engine %u Hz → device '%s' @ %u Hz, %u ch, "
                     "period=%u frames\n",
                     mDeviceSampleRate, device->playback.name,
                     device->sampleRate, device->playback.channels,
                     mFrameSize);
    }

    LOG_INFO("AudioService: miniaudio initialized");
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

    // Create HRTF for binaural rendering (must match device sample rate)
    IPLAudioSettings audioSettings{};
    audioSettings.samplingRate = static_cast<IPLint32>(mDeviceSampleRate);
    audioSettings.frameSize = static_cast<IPLint32>(mFrameSize);

    IPLHRTFSettings hrtfSettings{};
    hrtfSettings.type = IPL_HRTFTYPE_DEFAULT;
    hrtfSettings.volume = 1.0f;  // 1.0 = use HRTF data as-is (0.0 = silence!)

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
bool AudioService::loadSoundResources(const std::string &resPath,
                                      const std::string &schemasPath)
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

    // Load schema files (.spc, .arc, .sch).
    // Thief 2 stores schemas on disc 2 at EDITOR/SCHEMA/ or disc 1 at EDITOR/schemas/.
    // If --schemas path is provided, use it directly. Otherwise search known locations.
    mSchemaParser = std::make_unique<SchemaParser>();
    bool schemasLoaded = false;

    // Explicit path from --schemas CLI flag
    if (!schemasPath.empty()) {
        if (mSchemaParser->loadDirectory(schemasPath)) {
            std::fprintf(stderr, "AudioService: loaded %zu schemas from %s\n",
                         mSchemaParser->schemaCount(), schemasPath.c_str());
            schemasLoaded = true;
        }
    }

    // Search known Thief 2 schema locations relative to RES parent directory
    if (!schemasLoaded) {
        const char *searchPaths[] = {
            "/../EDITOR/SCHEMA",   // Disc 2 layout (uppercase)
            "/../EDITOR/schemas",  // Disc 1 layout (lowercase)
        };
        for (const char *suffix : searchPaths) {
            std::string dir = resPath + suffix;
            if (mSchemaParser->loadDirectory(dir)) {
                std::fprintf(stderr, "AudioService: loaded %zu schemas from %s\n",
                             mSchemaParser->schemaCount(), dir.c_str());
                schemasLoaded = true;
                break;
            }
        }
    }

    if (!schemasLoaded) {
        std::fprintf(stderr, "AudioService: no schema files found — use --schemas <path>\n");
        mSchemaParser.reset();
    }

    LOG_INFO("AudioService: Sound resources loaded from %s", resPath.c_str());
    std::fprintf(stderr, "AudioService: loaded %zu schemas, sound resources ready\n",
                 mSchemaParser ? mSchemaParser->schemaCount() : 0);

    // Now that schemas and sound loader are ready, load ambient sounds
    // from mission data (P$AmbientHack properties parsed during onDBLoad).
    loadAmbientSounds();
    std::fprintf(stderr, "AudioService: %zu ambient sounds, %zu active voices\n",
                 mAmbients.size(), mVoices.size());

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

    if (data.texNames.size() != numTriangles) {
        LOG_ERROR("AudioService: texNames count (%zu) != triangle count (%zu)",
                  data.texNames.size(), numTriangles);
        return false;
    }

    // Wrap scene construction in try-catch so that any C++ exception
    // (e.g. from std::vector allocation) cleans up partially-created
    // Steam Audio objects instead of leaking them.
    try {
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
            destroyAcousticScene();
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
        std::fprintf(stderr, "AudioService: acoustic scene built (%zu tris, %zu mats)\n",
                     numTriangles, materials.size());
        return true;

    } catch (const std::exception &e) {
        LOG_ERROR("AudioService: exception building acoustic scene: %s", e.what());
        destroyAcousticScene();
        return false;
    } catch (...) {
        LOG_ERROR("AudioService: unknown exception building acoustic scene");
        destroyAcousticScene();
        return false;
    }
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
    mObjectService = GET_SERVICE(ObjectService);

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
        std::fprintf(stderr, "AudioService: fully initialized (miniaudio + Steam Audio)\n");
    } else {
        LOG_ERROR("AudioService: initialized with errors (miniaudio=%s, steam_audio=%s)",
                  maOk ? "ok" : "FAILED", saOk ? "ok" : "FAILED");
        std::fprintf(stderr, "AudioService: INIT FAILED (miniaudio=%s, steam_audio=%s)\n",
                     maOk ? "ok" : "FAILED", saOk ? "ok" : "FAILED");
    }
}

//------------------------------------------------------
void AudioService::shutdown()
{
    haltAll();
    mAmbients.clear();

    // Release sound resources
    mSoundCache.reset();
    mSoundLoader.reset();

    // Release acoustic scene before the Steam Audio context it depends on
    destroyAcousticScene();

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
    mObjectService.reset();
    mSchemaParser.reset();

    LOG_INFO("AudioService: shut down");
}

//------------------------------------------------------
void AudioService::onDBLoad(const FileGroupPtr &db, uint32_t curmask)
{
    if (!(curmask & DBM_MIS_DATA))
        return;

    // NOTE: Sound resources, acoustic scene, and ambient sounds are loaded from
    // DarknessRender.cpp after this point (loadSoundResources → loadAmbientSounds).
    // onDBLoad just records that mission data is available.

    LOG_INFO("AudioService: mission data loaded");
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

    // Clear ambient sounds (voices already halted above)
    mAmbients.clear();

    // Clear per-schema state (NO_REPEAT tracking) and texture materials
    mLastSampleIdx.clear();
    mTextureMaterials.clear();

    LOG_INFO("AudioService: mission audio state cleared");
}

//------------------------------------------------------
void AudioService::updateAudio(float deltaTime)
{
    loopStep(deltaTime);
}

//------------------------------------------------------
void AudioService::setListenerTransform(const Vector3 &pos, float yaw, float pitch)
{
    mListenerPos = pos;
    mListenerYaw = yaw;
    mListenerPitch = pitch;

    // Also update miniaudio's 3D listener position
    if (mMaEngine) {
        ma_engine_listener_set_position(mMaEngine, 0, pos.x, pos.y, pos.z);

        // Compute forward and up vectors for miniaudio listener
        // Dark Engine Z-up: forward = (cosY*cosP, sinY*cosP, sinP)
        float cosY = std::cos(yaw), sinY = std::sin(yaw);
        float cosP = std::cos(pitch), sinP = std::sin(pitch);
        ma_engine_listener_set_direction(mMaEngine, 0,
                                         cosY * cosP, sinY * cosP, sinP);
        ma_engine_listener_set_world_up(mMaEngine, 0, 0.0f, 0.0f, 1.0f);
    }
}

//------------------------------------------------------
void AudioService::loopStep(float deltaTime)
{
    if (!mAudioReady)
        return;

    // Remove voices that have finished playback
    cleanupFinishedVoices();

    // Run Steam Audio simulation for all active sources
    if (mSceneReady && mIplSimulator && !mVoices.empty()) {
        // Step 1: Set listener position/orientation for the simulator
        float cosY = std::cos(mListenerYaw), sinY = std::sin(mListenerYaw);
        float cosP = std::cos(mListenerPitch), sinP = std::sin(mListenerPitch);

        IPLCoordinateSpace3 listenerCoord{};
        listenerCoord.origin = {mListenerPos.x, mListenerPos.y, mListenerPos.z};
        listenerCoord.ahead  = {cosY * cosP, sinY * cosP, sinP};
        listenerCoord.right  = {sinY, -cosY, 0.0f};
        // up = cross(right, ahead) for right-handed Z-up
        listenerCoord.up     = {-cosY * sinP, -sinY * sinP, cosP};

        IPLSimulationSharedInputs sharedInputs{};
        sharedInputs.listener = listenerCoord;
        sharedInputs.numRays = 4096;
        sharedInputs.numBounces = 4;
        sharedInputs.duration = 2.0f;
        sharedInputs.order = 1;
        sharedInputs.irradianceMinDistance = 1.0f;
        iplSimulatorSetSharedInputs(mIplSimulator,
            static_cast<IPLSimulationFlags>(
                IPL_SIMULATIONFLAGS_DIRECT | IPL_SIMULATIONFLAGS_REFLECTIONS),
            &sharedInputs);

        // Step 2: Update per-source positions
        for (auto &[handle, voice] : mVoices) {
            if (!voice->iplSource)
                continue;

            IPLCoordinateSpace3 sourceCoord{};
            sourceCoord.origin = {voice->worldPos.x, voice->worldPos.y, voice->worldPos.z};
            // Sources are omnidirectional — orientation doesn't matter for direct sim,
            // but Steam Audio requires a valid coordinate frame
            sourceCoord.ahead = {1.0f, 0.0f, 0.0f};
            sourceCoord.right = {0.0f, 1.0f, 0.0f};
            sourceCoord.up    = {0.0f, 0.0f, 1.0f};

            IPLSimulationInputs inputs{};
            inputs.flags = static_cast<IPLSimulationFlags>(
                IPL_SIMULATIONFLAGS_DIRECT | IPL_SIMULATIONFLAGS_REFLECTIONS);
            inputs.directFlags = static_cast<IPLDirectSimulationFlags>(
                IPL_DIRECTSIMULATIONFLAGS_DISTANCEATTENUATION |
                IPL_DIRECTSIMULATIONFLAGS_AIRABSORPTION |
                IPL_DIRECTSIMULATIONFLAGS_OCCLUSION |
                IPL_DIRECTSIMULATIONFLAGS_TRANSMISSION);
            inputs.source = sourceCoord;
            inputs.distanceAttenuationModel.type = IPL_DISTANCEATTENUATIONTYPE_DEFAULT;
            inputs.airAbsorptionModel.type = IPL_AIRABSORPTIONTYPE_DEFAULT;
            inputs.occlusionType = IPL_OCCLUSIONTYPE_RAYCAST;
            inputs.numOcclusionSamples = 16;
            inputs.numTransmissionRays = 8;

            iplSourceSetInputs(voice->iplSource,
                static_cast<IPLSimulationFlags>(
                    IPL_SIMULATIONFLAGS_DIRECT | IPL_SIMULATIONFLAGS_REFLECTIONS),
                &inputs);
        }

        // Step 3: Run the simulation
        iplSimulatorRunDirect(mIplSimulator);
        // TODO: Also run reflections when reflection DSP pipeline is ready
        // iplSimulatorRunReflections(mIplSimulator);

        // Step 4: Read back simulation results and feed to DSP nodes.
        // The DSP node's process callback (audio thread) uses these params
        // for frequency-dependent direct effects + HRTF binaural rendering.
        for (auto &[handle, voice] : mVoices) {
            if (!voice->iplSource)
                continue;

            IPLSimulationOutputs outputs{};
            iplSourceGetOutputs(voice->iplSource,
                static_cast<IPLSimulationFlags>(IPL_SIMULATIONFLAGS_DIRECT),
                &outputs);

            if (voice->dspNode.effectsReady) {
                // Copy simulation outputs directly to DSP node params.
                // outputs.direct IS an IPLDirectEffectParams — just set the flags
                // to tell the effect which components to apply.
                voice->dspNode.directParams = outputs.direct;
                voice->dspNode.directParams.flags = static_cast<IPLDirectEffectFlags>(
                    IPL_DIRECTEFFECTFLAGS_APPLYDISTANCEATTENUATION |
                    IPL_DIRECTEFFECTFLAGS_APPLYAIRABSORPTION |
                    IPL_DIRECTEFFECTFLAGS_APPLYOCCLUSION |
                    IPL_DIRECTEFFECTFLAGS_APPLYTRANSMISSION);
                voice->dspNode.directParams.transmissionType =
                    IPL_TRANSMISSIONTYPE_FREQDEPENDENT;

                // Compute listener-to-source direction in listener's local frame
                // for HRTF binaural rendering
                Vector3 toSource = voice->worldPos - mListenerPos;
                float dist = glm::length(toSource);
                if (dist > 0.001f) {
                    toSource /= dist;
                    // Listener basis vectors (Z-up right-handed)
                    Vector3 right(sinY, -cosY, 0.0f);
                    Vector3 ahead(cosY * cosP, sinY * cosP, sinP);
                    Vector3 up(-cosY * sinP, -sinY * sinP, cosP);

                    // Project world direction onto listener-local axes.
                    // Steam Audio convention: +X=right, +Y=up, -Z=ahead,
                    // so negate the ahead component for correct front/back.
                    voice->dspNode.direction = {
                        glm::dot(toSource, right),    // x = right
                        glm::dot(toSource, up),       // y = up
                        -glm::dot(toSource, ahead)    // z = -ahead (Steam Audio: -Z = forward)
                    };
                }
            } else {
                // Fallback: simple volume scaling (no DSP pipeline available)
                float volume = outputs.direct.distanceAttenuation *
                               outputs.direct.occlusion;
                ma_sound_set_volume(&voice->sound, volume);
            }
        }
    }

    // Update ambient sound volumes based on listener distance
    updateAmbientVolumes();
}

//------------------------------------------------------
void AudioService::cleanupFinishedVoices()
{
    // Only check the atomic flag (set by the audio thread's end callback).
    // This avoids cross-thread calls to ma_sound_at_end().
    for (auto it = mVoices.begin(); it != mVoices.end();) {
        if (it->second->finished.load(std::memory_order_acquire)) {
            removeVoiceSource(*it->second);
            it = mVoices.erase(it);
        } else {
            ++it;
        }
    }
}

//------------------------------------------------------
void AudioService::createVoiceSource(ActiveVoice &voice)
{
    if (!mIplSimulator || !mSceneReady)
        return;

    IPLSourceSettings srcSettings{};
    srcSettings.flags = static_cast<IPLSimulationFlags>(
        IPL_SIMULATIONFLAGS_DIRECT | IPL_SIMULATIONFLAGS_REFLECTIONS);

    IPLerror err = iplSourceCreate(mIplSimulator, &srcSettings, &voice.iplSource);
    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("AudioService: iplSourceCreate failed (error %d)", err);
        voice.iplSource = nullptr;
        return;
    }

    iplSourceAdd(voice.iplSource, mIplSimulator);
    iplSimulatorCommit(mIplSimulator);
}

//------------------------------------------------------
void AudioService::removeVoiceSource(ActiveVoice &voice)
{
    if (!voice.iplSource)
        return;

    if (mIplSimulator) {
        iplSourceRemove(voice.iplSource, mIplSimulator);
        iplSimulatorCommit(mIplSimulator);
    }
    // Release the source handle here (not in ~ActiveVoice) so lifecycle is
    // self-contained and ordering relative to simulator destruction doesn't matter.
    iplSourceRelease(&voice.iplSource);
    // voice.iplSource is now nullptr (iplSourceRelease nulls the pointer)
}

//------------------------------------------------------
void AudioService::initVoiceDSP(ActiveVoice &voice)
{
    if (!mIplContext || !mIplHrtf || !mMaEngine)
        return;

    auto &dsp = voice.dspNode;
    dsp.hrtf = mIplHrtf;
    dsp.frameSize = static_cast<int>(mFrameSize);

    // Allocate processing buffers (once, never reallocated — audio thread safe)
    dsp.monoScratch.resize(dsp.frameSize);
    dsp.stereoL.resize(dsp.frameSize);
    dsp.stereoR.resize(dsp.frameSize);

    // Create IPLDirectEffect (per-voice, frequency-dependent 3-band EQ)
    // Must match device sample rate so effects process audio correctly.
    IPLAudioSettings audioSettings{};
    audioSettings.samplingRate = static_cast<IPLint32>(mDeviceSampleRate);
    audioSettings.frameSize = dsp.frameSize;

    IPLDirectEffectSettings directSettings{};
    directSettings.numChannels = 1;  // mono processing

    IPLerror err = iplDirectEffectCreate(mIplContext, &audioSettings,
                                          &directSettings, &dsp.directEffect);
    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("AudioService: iplDirectEffectCreate failed (error %d)", err);
        return;
    }

    // Create IPLBinauralEffect (mono → stereo HRTF spatialization)
    IPLBinauralEffectSettings binauralSettings{};
    binauralSettings.hrtf = mIplHrtf;

    err = iplBinauralEffectCreate(mIplContext, &audioSettings,
                                   &binauralSettings, &dsp.binauralEffect);
    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("AudioService: iplBinauralEffectCreate failed (error %d)", err);
        iplDirectEffectRelease(&dsp.directEffect);
        dsp.directEffect = nullptr;
        return;
    }

    // Initialize custom miniaudio node (stereo input → stereo output).
    // Input is stereo because ma_sound always outputs in the engine's channel format.
    // The process callback downmixes to mono for Steam Audio, then re-spatializes to stereo.
    ma_uint32 inputChannels[1] = {2};   // 1 input bus, 2 channels (stereo from ma_sound)
    ma_uint32 outputChannels[1] = {2};  // 1 output bus, 2 channels (stereo to endpoint)

    ma_node_config nodeConfig = ma_node_config_init();
    nodeConfig.vtable = &sSteamAudioNodeVtable;
    nodeConfig.inputBusCount = 1;
    nodeConfig.outputBusCount = 1;
    nodeConfig.pInputChannels = inputChannels;
    nodeConfig.pOutputChannels = outputChannels;

    ma_result result = ma_node_init(ma_engine_get_node_graph(mMaEngine),
                                     &nodeConfig, nullptr, &dsp.base);
    if (result != MA_SUCCESS) {
        LOG_ERROR("AudioService: ma_node_init failed for DSP node (error %d)", result);
        iplBinauralEffectRelease(&dsp.binauralEffect);
        dsp.binauralEffect = nullptr;
        iplDirectEffectRelease(&dsp.directEffect);
        dsp.directEffect = nullptr;
        return;
    }
    dsp.nodeInitialized = true;

    // Set default direct params (unity — no attenuation until simulation updates)
    dsp.directParams.flags = static_cast<IPLDirectEffectFlags>(
        IPL_DIRECTEFFECTFLAGS_APPLYDISTANCEATTENUATION |
        IPL_DIRECTEFFECTFLAGS_APPLYAIRABSORPTION |
        IPL_DIRECTEFFECTFLAGS_APPLYOCCLUSION |
        IPL_DIRECTEFFECTFLAGS_APPLYTRANSMISSION);
    dsp.directParams.transmissionType = IPL_TRANSMISSIONTYPE_FREQDEPENDENT;
    dsp.directParams.distanceAttenuation = 1.0f;
    dsp.directParams.airAbsorption[0] = 1.0f;
    dsp.directParams.airAbsorption[1] = 1.0f;
    dsp.directParams.airAbsorption[2] = 1.0f;
    dsp.directParams.occlusion = 1.0f;
    dsp.directParams.transmission[0] = 1.0f;
    dsp.directParams.transmission[1] = 1.0f;
    dsp.directParams.transmission[2] = 1.0f;

    dsp.effectsReady = true;

    // Connect the node graph: ma_sound → DSP node → engine endpoint
    ma_result r1 = ma_node_attach_output_bus(&voice.sound, 0, &dsp.base, 0);
    ma_result r2 = ma_node_attach_output_bus(&dsp.base, 0, ma_engine_get_endpoint(mMaEngine), 0);

    if (r1 != MA_SUCCESS || r2 != MA_SUCCESS) {
        LOG_ERROR("AudioService: DSP node attach failed: sound→dsp=%d, dsp→endpoint=%d",
                  r1, r2);
    }

    LOG_DEBUG("AudioService: Steam Audio DSP pipeline initialized for voice %d",
              voice.handle);
}

// ── Voice creation helpers ──

/// Convert Dark Engine volume (millibels, -10000 to -1) to linear 0.0–1.0.
/// -1 = full volume (~1.0), -10000 = -100 dB (silence).
static float schemaVolumeToLinear(int volume)
{
    if (volume >= 0) return 1.0f;
    if (volume <= -10000) return 0.0f;
    // millibels → dB → linear: 10^(volume/2000)
    return std::pow(10.0f, volume / 2000.0f);
}

//------------------------------------------------------
int AudioService::selectSample(const std::string &schemaName, int sampleCount,
                                int totalFreq, const int *frequencies)
{
    if (sampleCount <= 0)
        return -1;
    if (sampleCount == 1)
        return 0;

    // Weighted random selection based on frequency values
    std::uniform_int_distribution<int> dist(0, totalFreq - 1);
    int roll = dist(mRng);

    int cumulative = 0;
    for (int i = 0; i < sampleCount; ++i) {
        cumulative += frequencies[i];
        if (roll < cumulative)
            return i;
    }
    return sampleCount - 1;  // fallback (shouldn't happen)
}

//------------------------------------------------------
bool AudioService::evictLowestPriority(int newPriority)
{
    // Find the voice with the lowest priority (strictly lower than new)
    SoundHandle lowestHandle = SOUND_HANDLE_INVALID;
    int lowestPriority = newPriority;

    for (const auto &[handle, voice] : mVoices) {
        if (voice->priority < lowestPriority) {
            lowestPriority = voice->priority;
            lowestHandle = handle;
        }
    }

    if (lowestHandle == SOUND_HANDLE_INVALID)
        return false;  // All existing voices have equal or higher priority

    LOG_DEBUG("AudioService: evicting voice %d (priority %d) for new voice (priority %d)",
              lowestHandle, lowestPriority, newPriority);
    haltSound(lowestHandle);
    return true;
}

//------------------------------------------------------
SoundHandle AudioService::startVoice(const std::string &schemaName,
                                      const std::string &sampleName,
                                      const Vector3 &position,
                                      int priority, bool looping, int objID)
{
    if (!mMaEngine || !mSoundLoader)
        return SOUND_HANDLE_INVALID;

    // Enforce voice limit — evict lowest priority if full
    if (static_cast<int>(mVoices.size()) >= MAX_ACTIVE_VOICES) {
        if (!evictLowestPriority(priority)) {
            LOG_DEBUG("AudioService: voice pool full, cannot play '%s' (priority %d)",
                      schemaName.c_str(), priority);
            return SOUND_HANDLE_INVALID;
        }
    }

    // Load WAV data (check cache first, then CRF)
    SoundData snd;
    if (mSoundCache) {
        const SoundData *cached = mSoundCache->get(sampleName);
        if (cached) {
            snd = *cached;
        }
    }
    if (!snd.valid()) {
        snd = mSoundLoader->loadSound(sampleName);
        if (!snd.valid()) {
            LOG_ERROR("AudioService: failed to load sample '%s' for schema '%s'",
                      sampleName.c_str(), schemaName.c_str());
            return SOUND_HANDLE_INVALID;
        }
        if (mSoundCache) {
            mSoundCache->put(sampleName, snd);
        }
    }

    auto voice = std::make_unique<ActiveVoice>();
    voice->data = std::move(snd);
    voice->handle = mNextHandle++;
    voice->schemaName = schemaName;
    voice->priority = priority;
    voice->objID = objID;
    voice->worldPos = position;

    // Decode WAV from memory → PCM
    ma_decoder_config cfg = ma_decoder_config_init(ma_format_f32, 0, 0);
    if (ma_decoder_init_memory(voice->data.wavData.data(),
                               voice->data.wavData.size(),
                               &cfg, &voice->decoder) != MA_SUCCESS) {
        LOG_ERROR("AudioService: failed to decode sample '%s'", sampleName.c_str());
        return SOUND_HANDLE_INVALID;
    }

    // Create sound without default attachment (routed through DSP node)
    if (ma_sound_init_from_data_source(
            mMaEngine, &voice->decoder,
            MA_SOUND_FLAG_NO_SPATIALIZATION |
            MA_SOUND_FLAG_NO_DEFAULT_ATTACHMENT, nullptr,
            &voice->sound) != MA_SUCCESS) {
        ma_decoder_uninit(&voice->decoder);
        LOG_ERROR("AudioService: failed to init sound for '%s'", sampleName.c_str());
        return SOUND_HANDLE_INVALID;
    }

    voice->initialized = true;

    // Set looping if schema says so
    if (looping) {
        ma_sound_set_looping(&voice->sound, MA_TRUE);
    }

    // Set up Steam Audio DSP pipeline (direct + binaural effects)
    initVoiceDSP(*voice);

    // If DSP setup failed or not available, connect directly to endpoint
    if (!voice->dspNode.effectsReady) {
        ma_node_attach_output_bus(&voice->sound, 0,
                                  ma_engine_get_endpoint(mMaEngine), 0);
    }

    // Create Steam Audio source for spatial simulation
    createVoiceSource(*voice);

    ma_result startResult = ma_sound_start(&voice->sound);
    if (startResult != MA_SUCCESS) {
        LOG_ERROR("AudioService: failed to start sound '%s' (error %d)",
                  sampleName.c_str(), startResult);
        return SOUND_HANDLE_INVALID;
    }

    // Register end callback AFTER successful start (avoids dangling pointer if start fails)
    ma_sound_set_end_callback(&voice->sound, onSoundEnd, voice.get());

    SoundHandle h = voice->handle;
    LOG_DEBUG("AudioService: playing '%s' sample '%s' at (%.1f, %.1f, %.1f) priority=%d",
              schemaName.c_str(), sampleName.c_str(),
              position.x, position.y, position.z, priority);
    mVoices[h] = std::move(voice);
    return h;
}

// ── Public API ──

//------------------------------------------------------
SoundHandle AudioService::playSchema(const std::string &schemaName,
                                     const Vector3 &position)
{
    if (!mAudioReady || !mSchemaParser)
        return SOUND_HANDLE_INVALID;

    const SchemaEntry *schema = mSchemaParser->findSchema(schemaName);
    if (!schema) {
        LOG_DEBUG("AudioService::playSchema: schema '%s' not found", schemaName.c_str());
        return SOUND_HANDLE_INVALID;
    }

    if (schema->samples.empty()) {
        LOG_DEBUG("AudioService::playSchema: schema '%s' has no samples", schemaName.c_str());
        return SOUND_HANDLE_INVALID;
    }

    // Select a sample using frequency-weighted random selection
    std::vector<int> freqs;
    freqs.reserve(schema->samples.size());
    for (const auto &s : schema->samples)
        freqs.push_back(s.frequency);

    int totalFreq = schema->totalFrequency();
    if (totalFreq <= 0)
        return SOUND_HANDLE_INVALID;

    int idx = selectSample(schemaName, static_cast<int>(schema->samples.size()),
                           totalFreq, freqs.data());
    if (idx < 0 || idx >= static_cast<int>(schema->samples.size()))
        return SOUND_HANDLE_INVALID;

    // NO_REPEAT: avoid repeating the last sample
    if ((schema->playParams.flags & SCH_NO_REPEAT) && schema->samples.size() > 1) {
        auto it = mLastSampleIdx.find(schemaName);
        if (it != mLastSampleIdx.end() && it->second == idx) {
            // Re-roll once to avoid repeat
            idx = selectSample(schemaName, static_cast<int>(schema->samples.size()),
                               totalFreq, freqs.data());
            if (idx < 0 || idx >= static_cast<int>(schema->samples.size()))
                return SOUND_HANDLE_INVALID;
        }
    }
    mLastSampleIdx[schemaName] = idx;

    const SchemaSample &sample = schema->samples[idx];
    bool looping = schema->loopParams.isLooping;

    SoundHandle h = startVoice(schemaName, sample.name, position,
                               schema->playParams.priority, looping, 0);

    // Apply schema volume to the voice
    if (h != SOUND_HANDLE_INVALID && mVoices.count(h)) {
        float vol = schemaVolumeToLinear(schema->playParams.volume);
        ma_sound_set_volume(&mVoices[h]->sound, vol);
    }

    return h;
}

//------------------------------------------------------
SoundHandle AudioService::playSchemaOnObj(const std::string &schemaName,
                                          int objID)
{
    if (!mAudioReady || !mObjectService)
        return SOUND_HANDLE_INVALID;

    // Look up object position via ObjectService
    Vector3 pos = mObjectService->position(objID);

    // Play the schema at the object's position, attached to the object
    if (!mSchemaParser)
        return SOUND_HANDLE_INVALID;

    const SchemaEntry *schema = mSchemaParser->findSchema(schemaName);
    if (!schema || schema->samples.empty())
        return SOUND_HANDLE_INVALID;

    std::vector<int> freqs;
    freqs.reserve(schema->samples.size());
    for (const auto &s : schema->samples)
        freqs.push_back(s.frequency);

    int totalFreq = schema->totalFrequency();
    if (totalFreq <= 0)
        return SOUND_HANDLE_INVALID;

    int idx = selectSample(schemaName, static_cast<int>(schema->samples.size()),
                           totalFreq, freqs.data());
    if (idx < 0 || idx >= static_cast<int>(schema->samples.size()))
        return SOUND_HANDLE_INVALID;

    if ((schema->playParams.flags & SCH_NO_REPEAT) && schema->samples.size() > 1) {
        auto it = mLastSampleIdx.find(schemaName);
        if (it != mLastSampleIdx.end() && it->second == idx) {
            idx = selectSample(schemaName, static_cast<int>(schema->samples.size()),
                               totalFreq, freqs.data());
            if (idx < 0 || idx >= static_cast<int>(schema->samples.size()))
                return SOUND_HANDLE_INVALID;
        }
    }
    mLastSampleIdx[schemaName] = idx;

    const SchemaSample &sample = schema->samples[idx];
    bool looping = schema->loopParams.isLooping;

    SoundHandle h = startVoice(schemaName, sample.name, pos,
                               schema->playParams.priority, looping, objID);

    if (h != SOUND_HANDLE_INVALID && mVoices.count(h)) {
        float vol = schemaVolumeToLinear(schema->playParams.volume);
        ma_sound_set_volume(&mVoices[h]->sound, vol);
    }

    return h;
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
        removeVoiceSource(*it->second);
        mVoices.erase(it);
    }
}

//------------------------------------------------------
void AudioService::haltAll()
{
    // Remove all Steam Audio sources before destroying voices
    for (auto &[handle, voice] : mVoices) {
        if (voice->initialized) {
            ma_sound_stop(&voice->sound);
        }
        removeVoiceSource(*voice);
    }
    mVoices.clear();
    mNextHandle = 0;
}

// ── Ambient sound system ──

//------------------------------------------------------
void AudioService::loadAmbientSounds()
{
    if (!mPropertyService || !mObjectService || !mAudioReady)
        return;

    // Find all objects with P$AmbientHack (property name "AmbientHa" in the gamesys)
    auto objIDs = getAllObjectsWithProperty(mPropertyService.get(), "AmbientHa");

    int loaded = 0;
    for (int objID : objIDs) {
        // Only process concrete objects (positive IDs), not archetypes
        if (objID <= 0)
            continue;

        size_t rawSize = 0;
        const uint8_t *raw = getPropertyRawData(
            mPropertyService.get(), "AmbientHa", objID, rawSize);
        if (!raw || rawSize < sizeof(PropAmbientHack))
            continue;

        const auto *prop = reinterpret_cast<const PropAmbientHack *>(raw);

        // Skip turned-off ambients
        if (prop->flags & AMB_TURNED_OFF)
            continue;

        AmbientSound amb;
        amb.objID = objID;
        amb.schemaName = std::string(prop->schema,
            strnlen(prop->schema, sizeof(prop->schema)));
        amb.radius = static_cast<float>(prop->radius);
        amb.volume = prop->volume;
        amb.flags = prop->flags;
        amb.position = mObjectService->position(objID);

        if (amb.schemaName.empty())
            continue;

        // Register ambient data only — voices are started/stopped dynamically
        // in updateAmbientVolumes() based on listener distance.
        mAmbients.push_back(std::move(amb));
        ++loaded;
    }

    if (loaded > 0) {
        LOG_INFO("AudioService: registered %d ambient sounds from P$AmbientHack", loaded);
    }
}

//------------------------------------------------------
void AudioService::updateAmbientVolumes()
{
    if (mAmbients.empty())
        return;

    // Debug: periodic status dump (once per ~5 seconds)
    static float debugTimer = 0.0f;
    debugTimer += 1.0f / 60.0f;  // approximate
    if (debugTimer >= 5.0f) {
        debugTimer = 0.0f;
        int playing = 0;
        for (const auto &amb : mAmbients) {
            if (amb.handle != SOUND_HANDLE_INVALID && mVoices.count(amb.handle))
                ++playing;
        }
        std::fprintf(stderr, "[Audio] %zu voices, %d/%zu ambients playing, "
                     "listener=(%.0f,%.0f,%.0f)\n",
                     mVoices.size(), playing, mAmbients.size(),
                     mListenerPos.x, mListenerPos.y, mListenerPos.z);
    }

    for (auto &amb : mAmbients) {
        float dist = glm::length(mListenerPos - amb.position);
        bool inRange = (amb.radius > 0.0f && dist < amb.radius);

        if (inRange) {
            // Start voice if not already playing
            if (amb.handle == SOUND_HANDLE_INVALID) {
                bool isLooping = !(amb.flags & AMB_ONCE_ONLY);
                if (mSchemaParser) {
                    const SchemaEntry *schema = mSchemaParser->findSchema(amb.schemaName);
                    if (schema && !schema->samples.empty()) {
                        const SchemaSample &sample = schema->samples[0];
                        amb.handle = startVoice(amb.schemaName, sample.name, amb.position,
                                                schema->playParams.priority, isLooping, amb.objID);
                    }
                }
                // Fallback: try loading schema name as a raw sound
                if (amb.handle == SOUND_HANDLE_INVALID) {
                    amb.handle = startVoice(amb.schemaName, amb.schemaName, amb.position,
                                            64, isLooping, amb.objID);
                }
            }

            // Update volume based on distance
            if (amb.handle != SOUND_HANDLE_INVALID) {
                auto it = mVoices.find(amb.handle);
                if (it == mVoices.end()) {
                    amb.handle = SOUND_HANDLE_INVALID;
                    continue;
                }

                float falloff = 1.0f - (dist / amb.radius);
                if (!(amb.flags & AMB_NO_FADE)) {
                    falloff *= falloff;  // quadratic curve for natural fade
                }
                float baseVol = schemaVolumeToLinear(amb.volume);
                ma_sound_set_volume(&it->second->sound, baseVol * falloff);
            }
        } else {
            // Out of range — stop voice to free the slot and DSP resources
            if (amb.handle != SOUND_HANDLE_INVALID) {
                haltSound(amb.handle);
                amb.handle = SOUND_HANDLE_INVALID;
            }
        }
    }
}

// ── Footstep material system ──

//------------------------------------------------------
void AudioService::setTextureMaterials(std::vector<std::string> materials)
{
    mTextureMaterials = std::move(materials);
    LOG_INFO("AudioService: %zu texture materials set", mTextureMaterials.size());
}

//------------------------------------------------------
void AudioService::playFootstep(const Vector3 &pos, float speed, int textureIdx)
{
    if (!mAudioReady || !mSchemaParser)
        return;

    // Determine material for footstep sound selection.
    // Water overrides ground material when the player's feet are submerged.
    std::string material;
    bool isWater = mPlayerInWater;

    if (isWater) {
        material = "Liquid";  // Schema material name for water surfaces
    } else {
        // Look up acoustic keyword from texture index, then map to schema Material name.
        // AcousticMaterials returns internal keywords (e.g., "concrete", "brick", "dirt")
        // but schemas use Dark Engine material names (e.g., "Stone", "Earth", "Metal").
        std::string keyword = "generic";
        if (textureIdx >= 0 && textureIdx < static_cast<int>(mTextureMaterials.size())) {
            const std::string &m = mTextureMaterials[textureIdx];
            if (!m.empty()) keyword = m;
        }

        // Map acoustic keywords → schema Material enum values
        if (keyword == "stone" || keyword == "concrete" || keyword == "brick" ||
            keyword == "plaster" || keyword == "rock" || keyword == "floor" ||
            keyword == "roof") {
            material = "Stone";
        } else if (keyword == "metal" || keyword == "metl" || keyword == "rust" ||
                   keyword == "iron" || keyword == "gate" || keyword == "grate" ||
                   keyword == "plate" || keyword == "pipe" || keyword == "steel" ||
                   keyword == "bronze") {
            material = "Metal";
        } else if (keyword == "wood" || keyword == "door" || keyword == "bark") {
            material = "Wood";
        } else if (keyword == "tile") {
            material = "Tile";
        } else if (keyword == "ceramic") {
            material = "Ceramic";
        } else if (keyword == "glass") {
            material = "Glass";
        } else if (keyword == "carpet" || keyword == "rug") {
            material = "Carpet";
        } else if (keyword == "gravel") {
            material = "Gravel";
        } else if (keyword == "earth" || keyword == "dirt" || keyword == "mud") {
            material = "Earth";
        } else if (keyword == "ice") {
            material = "Ice";
        } else if (keyword == "hay" || keyword == "vine" || keyword == "leaf") {
            material = "Vegetation";
        } else {
            material = "Stone";  // safe default — most common surface in Thief
        }
    }

    // Build env_tag query to match schemas like:
    //   env_tag (Event Footstep) (CreatureType Player) (Material Stone)
    // CreatureType Player is required — all player footstep schemas include it.
    // For water footsteps, also adds (MediaLevel Foot) tag.
    std::vector<SchemaTagValue> query;
    {
        SchemaTagValue eventTag;
        eventTag.tagName = "Event";
        eventTag.enumValues.push_back("Footstep");
        query.push_back(std::move(eventTag));

        // CreatureType Player — required by all player footstep schemas
        SchemaTagValue creatureTag;
        creatureTag.tagName = "CreatureType";
        creatureTag.enumValues.push_back("Player");
        query.push_back(std::move(creatureTag));

        SchemaTagValue matTag;
        matTag.tagName = "Material";
        matTag.enumValues.push_back(material);
        query.push_back(std::move(matTag));

        // Water footsteps add MediaLevel tag for splash/wading distinction
        if (isWater) {
            SchemaTagValue mediaTag;
            mediaTag.tagName = "MediaLevel";
            mediaTag.enumValues.push_back("Foot");
            query.push_back(std::move(mediaTag));
        }
    }

    // Find matching schemas via env_tag matching
    auto matches = mSchemaParser->findByEnvTags(query);
    if (matches.empty()) {
        LOG_DEBUG("AudioService: no footstep schema for material '%s' (texIdx=%d)",
                  material.c_str(), textureIdx);
        return;
    }

    // Pick the first matching schema (highest priority if multiple match)
    const SchemaEntry *schema = matches[0];

    // Scale volume by movement speed:
    //   creep (~3.75 u/s) → quiet (0.3)
    //   walk  (~7.5 u/s)  → normal (0.6)
    //   run   (~15 u/s)   → loud (1.0)
    float speedFactor = std::clamp(speed / 15.0f, 0.1f, 1.0f);
    float baseVol = schemaVolumeToLinear(schema->playParams.volume);
    float finalVol = baseVol * speedFactor;

    // Select sample and play
    if (schema->samples.empty())
        return;

    std::vector<int> freqs;
    freqs.reserve(schema->samples.size());
    for (const auto &s : schema->samples)
        freqs.push_back(s.frequency);

    int totalFreq = schema->totalFrequency();
    if (totalFreq <= 0)
        return;

    int idx = selectSample(schema->name, static_cast<int>(schema->samples.size()),
                           totalFreq, freqs.data());
    if (idx < 0 || idx >= static_cast<int>(schema->samples.size()))
        return;

    // NO_REPEAT for footsteps
    if ((schema->playParams.flags & SCH_NO_REPEAT) && schema->samples.size() > 1) {
        auto it = mLastSampleIdx.find(schema->name);
        if (it != mLastSampleIdx.end() && it->second == idx) {
            idx = selectSample(schema->name, static_cast<int>(schema->samples.size()),
                               totalFreq, freqs.data());
            if (idx < 0 || idx >= static_cast<int>(schema->samples.size()))
                return;
        }
    }
    mLastSampleIdx[schema->name] = idx;

    const SchemaSample &sample = schema->samples[idx];

    SoundHandle h = startVoice(schema->name, sample.name, pos,
                               schema->playParams.priority, false, 0);

    if (h != SOUND_HANDLE_INVALID && mVoices.count(h)) {
        ma_sound_set_volume(&mVoices[h]->sound, finalVol);
    }
}

//------------------------------------------------------
void AudioService::playLanding(const Vector3 &pos, float fallSpeed, int textureIdx)
{
    if (!mAudioReady || !mSchemaParser)
        return;

    // Determine material (same mapping as playFootstep)
    std::string material;
    bool isWater = mPlayerInWater;

    if (isWater) {
        material = "Liquid";
    } else {
        std::string keyword = "generic";
        if (textureIdx >= 0 && textureIdx < static_cast<int>(mTextureMaterials.size())) {
            const std::string &m = mTextureMaterials[textureIdx];
            if (!m.empty()) keyword = m;
        }

        if (keyword == "stone" || keyword == "concrete" || keyword == "brick" ||
            keyword == "plaster" || keyword == "rock" || keyword == "floor" ||
            keyword == "roof") {
            material = "Stone";
        } else if (keyword == "metal" || keyword == "metl" || keyword == "rust" ||
                   keyword == "iron" || keyword == "gate" || keyword == "grate" ||
                   keyword == "plate" || keyword == "pipe" || keyword == "steel" ||
                   keyword == "bronze") {
            material = "Metal";
        } else if (keyword == "wood" || keyword == "door" || keyword == "bark") {
            material = "Wood";
        } else if (keyword == "tile") {
            material = "Tile";
        } else if (keyword == "ceramic") {
            material = "Ceramic";
        } else if (keyword == "glass") {
            material = "Glass";
        } else if (keyword == "carpet" || keyword == "rug") {
            material = "Carpet";
        } else if (keyword == "gravel") {
            material = "Gravel";
        } else if (keyword == "earth" || keyword == "dirt" || keyword == "mud") {
            material = "Earth";
        } else if (keyword == "ice") {
            material = "Ice";
        } else if (keyword == "hay" || keyword == "vine" || keyword == "leaf") {
            material = "Vegetation";
        } else {
            material = "Stone";
        }
    }

    // Build env_tag query: (Event Footstep) (Landing True) (CreatureType Player) (Material ...)
    std::vector<SchemaTagValue> query;
    {
        SchemaTagValue eventTag;
        eventTag.tagName = "Event";
        eventTag.enumValues.push_back("Footstep");
        query.push_back(std::move(eventTag));

        SchemaTagValue landingTag;
        landingTag.tagName = "Landing";
        landingTag.enumValues.push_back("True");
        query.push_back(std::move(landingTag));

        SchemaTagValue creatureTag;
        creatureTag.tagName = "CreatureType";
        creatureTag.enumValues.push_back("Player");
        query.push_back(std::move(creatureTag));

        SchemaTagValue matTag;
        matTag.tagName = "Material";
        matTag.enumValues.push_back(material);
        query.push_back(std::move(matTag));

        if (isWater) {
            SchemaTagValue mediaTag;
            mediaTag.tagName = "MediaLevel";
            mediaTag.enumValues.push_back("Foot");
            query.push_back(std::move(mediaTag));
        }
    }

    auto matches = mSchemaParser->findByEnvTags(query);
    if (matches.empty()) {
        LOG_DEBUG("AudioService: no landing schema for material '%s'", material.c_str());
        return;
    }

    const SchemaEntry *schema = matches[0];

    // Volume scales with fall velocity — harder falls are louder.
    // Dark Engine uses SCH_ADD_VOLUME: the computed volume is added to the
    // schema's base volume. Formula: playVol = baseVol + volMul * (fallSpeed - cutoff)
    // Higher fallSpeed → less negative playVol → louder overall.
    constexpr int LANDING_BASE_VOL = -1000;  // millibels
    constexpr int LANDING_VOL_MUL = 250;     // millibels per unit/sec
    constexpr float LANDING_CUTOFF_VEL = 2.0f;
    int playVol = LANDING_BASE_VOL +
        LANDING_VOL_MUL * static_cast<int>(fallSpeed - LANDING_CUTOFF_VEL);
    if (playVol > -1) playVol = -1;

    // Additive: schema volume + velocity-scaled volume (both in millibels)
    int totalMillibels = schema->playParams.volume + playVol;
    if (totalMillibels > -1) totalMillibels = -1;

    float finalVol = schemaVolumeToLinear(totalMillibels);

    if (schema->samples.empty())
        return;

    // Landing schemas typically have a single sample (e.g., ftroc_j)
    const SchemaSample &sample = schema->samples[0];

    SoundHandle h = startVoice(schema->name, sample.name, pos,
                               schema->playParams.priority, false, 0);

    if (h != SOUND_HANDLE_INVALID && mVoices.count(h)) {
        ma_sound_set_volume(&mVoices[h]->sound, finalVol);
    }
}

// ── Sound propagation through portal graph (AI hearing) ──

//------------------------------------------------------
SoundPropInfo AudioService::propagateSound(const Vector3 &sourcePos,
                                            const Vector3 &listenerPos,
                                            float maxDist) const
{
    SoundPropInfo result;

    if (!mRoomService || !mRoomService->isLoaded())
        return result;

    // Find the rooms containing source and listener
    Room *sourceRoom = mRoomService->roomFromPoint(sourcePos);
    Room *listenerRoom = mRoomService->roomFromPoint(listenerPos);

    if (!sourceRoom || !listenerRoom)
        return result;

    // Same room — direct line of sight, no portal traversal needed
    if (sourceRoom == listenerRoom) {
        float dist = glm::length(listenerPos - sourcePos);
        if (dist <= maxDist) {
            result.reached = true;
            result.realDistance = dist;
            result.effectiveDistance = dist;
            result.totalBlocking = 0.0f;
            result.virtualPosition = sourcePos;
        }
        return result;
    }

    // Dijkstra-style BFS through portal graph, ordered by effective distance.
    // Each entry tracks cumulative real distance, effective distance (with blocking
    // penalties), and the last portal crossed (for virtual position).
    struct BFSEntry {
        Room *room;
        float realDist;         // Cumulative real distance to room entry point
        float effectiveDist;    // Cumulative effective distance (with blocking penalties)
        float maxBlocking;      // Highest blocking factor encountered so far
        Vector3 entryPoint;     // Where we entered this room (portal center or source)
        Vector3 lastPortalPos;  // Center of the last portal crossed (for virtual position)

        // Priority queue: smallest effective distance first
        bool operator>(const BFSEntry &o) const {
            return effectiveDist > o.effectiveDist;
        }
    };

    std::priority_queue<BFSEntry, std::vector<BFSEntry>, std::greater<BFSEntry>> pq;
    std::unordered_map<int32_t, float> bestDist;  // roomID → best effective distance seen

    // Seed with the source room
    BFSEntry start;
    start.room = sourceRoom;
    start.realDist = 0.0f;
    start.effectiveDist = 0.0f;
    start.maxBlocking = 0.0f;
    start.entryPoint = sourcePos;
    start.lastPortalPos = sourcePos;
    pq.push(start);
    bestDist[sourceRoom->getRoomID()] = 0.0f;

    while (!pq.empty()) {
        BFSEntry cur = pq.top();
        pq.pop();

        // Skip if we already found a better path to this room
        auto it = bestDist.find(cur.room->getRoomID());
        if (it != bestDist.end() && cur.effectiveDist > it->second)
            continue;

        // Check if we've reached the listener's room
        if (cur.room == listenerRoom) {
            // Add final segment from room entry point to listener
            float finalSeg = glm::length(listenerPos - cur.entryPoint);
            result.reached = true;
            result.realDistance = cur.realDist + finalSeg;
            result.effectiveDistance = cur.effectiveDist + finalSeg;
            result.totalBlocking = cur.maxBlocking;
            // Virtual position: the last portal center, or source if same-room
            result.virtualPosition = (cur.lastPortalPos == sourcePos)
                                     ? sourcePos : cur.lastPortalPos;
            return result;
        }

        // Explore portals from the current room
        uint32_t portalCount = cur.room->getPortalCount();
        for (uint32_t i = 0; i < portalCount; ++i) {
            RoomPortal *portal = cur.room->getPortal(i);
            if (!portal) continue;

            Room *nextRoom = portal->getFarRoom();
            if (!nextRoom) continue;

            // Distance from current entry point to this portal center
            float segDist = glm::length(portal->getCenter() - cur.entryPoint);
            float newRealDist = cur.realDist + segDist;

            // Get blocking factor for this portal (keyed by room pair)
            float blocking = getBlockingFactor(cur.room->getRoomID(),
                                                nextRoom->getRoomID());

            // Dark Engine munged distance formula:
            // effectiveDist = realDist + (maxDist - realDist) * blockingFactor
            // Applied per-portal: the penalty increases the effective distance
            float penalty = (maxDist - newRealDist) * blocking;
            if (penalty < 0.0f) penalty = 0.0f;
            float newEffDist = cur.effectiveDist + segDist + penalty;

            // Prune if beyond max propagation distance
            if (newEffDist > maxDist)
                continue;

            float newMaxBlocking = std::max(cur.maxBlocking, blocking);

            // Only explore if this is a better path to the next room
            int32_t nextID = nextRoom->getRoomID();
            auto bestIt = bestDist.find(nextID);
            if (bestIt != bestDist.end() && newEffDist >= bestIt->second)
                continue;
            bestDist[nextID] = newEffDist;

            BFSEntry next;
            next.room = nextRoom;
            next.realDist = newRealDist;
            next.effectiveDist = newEffDist;
            next.maxBlocking = newMaxBlocking;
            next.entryPoint = portal->getCenter();
            next.lastPortalPos = portal->getCenter();
            pq.push(next);
        }
    }

    // Sound could not reach the listener within maxDist
    return result;
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
