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

    // Reflection convolution effect (per-voice, feeds into shared mixer)
    IPLReflectionEffect reflectionEffect = nullptr;

    // Shared reference to the reflection mixer (NOT owned — AudioService manages)
    IPLReflectionMixer reflectionMixer = nullptr;

    // Reflection simulation output params (written by main thread from simulator)
    IPLReflectionEffectParams reflectionParams{};
    bool reflectionsActive = false;

    // Scratch buffers for deinterleaved Steam Audio processing
    // (allocated once at init, never reallocated — safe for audio thread)
    std::vector<float> monoScratch;   // mono channel for direct effect in/out
    std::vector<float> stereoL;       // left channel (binaural output)
    std::vector<float> stereoR;       // right channel (binaural output)

    // Per-voice ambisonics scratch for reflection convolution output
    // (required by iplReflectionEffectApply even when using mixer accumulation)
    std::vector<float> ambiScratch0;  // W channel
    std::vector<float> ambiScratch1;  // Y channel
    std::vector<float> ambiScratch2;  // Z channel
    std::vector<float> ambiScratch3;  // X channel

    // Half-rate decimation scratch (used when reflection pipeline runs at 24kHz)
    std::vector<float> decimatedMono;

    int frameSize = 1024;
    int reflectionFrameSize = 1024;  // half of frameSize in half-rate mode
    bool halfRate = false;
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
    MA_NODE_FLAG_CONTINUOUS_PROCESSING  // Keep processing after input exhausted (reverb tails)
};

// Forward-declare the reflection mix node process callback
static void reflectionMixNodeProcess(ma_node* pNode, const float** ppFramesIn,
                                      ma_uint32* pFrameCountIn,
                                      float** ppFramesOut, ma_uint32* pFrameCountOut);

/// Global reflection mix node — one per AudioService.
/// Sits between all per-voice DSP nodes and the engine endpoint.
/// Retrieves accumulated reflection convolution output from the shared mixer,
/// decodes ambisonics to binaural stereo via HRTF, and adds to the direct signal.
struct AudioService::ReflectionMixNode {
    ma_node_base base{};  // Must be first member (miniaudio node graph requirement)

    // Shared handles (NOT owned — AudioService manages lifetimes)
    IPLReflectionMixer mixer = nullptr;
    IPLAmbisonicsDecodeEffect ambiDecodeEffect = nullptr;
    IPLHRTF hrtf = nullptr;

    // Listener orientation for ambisonics decode (written by main thread each frame).
    // Must be initialized to a valid coordinate space — zero vectors cause crashes
    // in iplAmbisonicsDecodeEffectApply before the first loopStep sets the real values.
    IPLCoordinateSpace3 listenerOrientation = {
        {1.0f, 0.0f, 0.0f},  // right  = +X
        {0.0f, 0.0f, 1.0f},  // up     = +Z (Dark Engine Z-up)
        {0.0f, 1.0f, 0.0f},  // ahead  = +Y
        {0.0f, 0.0f, 0.0f}   // origin = world origin
    };

    // Scratch buffers for ambisonics processing (order 1 = 4 channels)
    std::vector<float> ambiCh0;  // W channel
    std::vector<float> ambiCh1;  // Y channel
    std::vector<float> ambiCh2;  // Z channel
    std::vector<float> ambiCh3;  // X channel
    std::vector<float> decodedL; // binaural left
    std::vector<float> decodedR; // binaural right

    int frameSize = 1024;             // engine frame size (48kHz)
    int reflectionFrameSize = 1024;   // reflection frame size (24kHz or 48kHz)
    int ambiChannels = 1;             // ambisonics channel count (1 for order 0, 4 for order 1)
    int ambiOrder = 0;                // ambisonics order (0 or 1)
    bool halfRate = false;            // true when running reflections at half sample rate
    bool ready = false;               // true once pipeline is initialized
    bool simulationRan = false;       // true after first iplSimulatorRunReflections completes
    bool nodeInitialized = false;
};

/// vtable for the global reflection mix node
static ma_node_vtable sReflectionMixNodeVtable = {
    reflectionMixNodeProcess,
    nullptr,    // onGetRequiredInputFrameCount
    1,          // inputBusCount  (stereo sum from all per-voice DSP nodes)
    1,          // outputBusCount (stereo to engine endpoint)
    0           // flags
};

/// Temporary debug flag: when true, bypass Steam Audio effects and just pass
/// mono input through as dual-mono stereo. Helps isolate node graph issues.
/// DSP bypass level (0=full pipeline, 3=mono passthrough, 4=stereo passthrough)
static std::atomic<int> sDSPBypassLevel{0};

// ── Audio thread profiling (written by audio thread, read by main thread) ──
// Tracks peak time spent in audio callbacks to detect buffer underruns.
static std::atomic<float> sPerVoicePeakUs{0.0f};    // peak per-voice DSP time (µs)
static std::atomic<float> sMixNodePeakUs{0.0f};      // peak global mix node time (µs)
static std::atomic<float> sTotalCallbackPeakUs{0.0f}; // peak total audio callback time (µs)
static std::atomic<int>   sPerVoiceCallCount{0};      // voices processed in last period
static std::atomic<float> sWaitThreadPeakMs{0.0f};    // peak waitForReflectionThread time (ms)
static std::atomic<float> sCommitPeakMs{0.0f};         // peak iplSimulatorCommit time (ms)

// Accumulates per-voice time within a single audio callback cycle.
// Reset by the mix node after reading. Not truly thread-safe between
// multiple voice callbacks in the same cycle, but gives a good estimate.
static std::atomic<float> sCallbackAccumUs{0.0f};

/// Audio-thread callback: applies Steam Audio direct + binaural effects.
/// Reads mono from ma_sound, outputs stereo binaural to engine endpoint.
static void steamAudioNodeProcess(ma_node* pNode, const float** ppFramesIn,
                                   ma_uint32* pFrameCountIn,
                                   float** ppFramesOut, ma_uint32* pFrameCountOut)
{
    auto t0 = std::chrono::steady_clock::now();
    auto* node = reinterpret_cast<SteamAudioDSPNode*>(pNode);
    ma_uint32 frameCount = *pFrameCountOut;

    // Clamp to available input and our buffer size (1:1 input/output ratio)
    if (pFrameCountIn[0] < frameCount)
        frameCount = pFrameCountIn[0];
    if (frameCount > static_cast<ma_uint32>(node->frameSize))
        frameCount = static_cast<ma_uint32>(node->frameSize);

    // Track diagnostics
    node->callCount.fetch_add(1, std::memory_order_relaxed);
    node->lastFrameCount.store(frameCount, std::memory_order_relaxed);

    float* stereoOut = ppFramesOut[0];  // interleaved stereo output

    if (!node->effectsReady || frameCount == 0) {
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
    if (stereoIn && frameCount > 0) {
        for (ma_uint32 i = 0; i < frameCount; ++i) {
            mono[i] = (stereoIn[i * 2] + stereoIn[i * 2 + 1]) * 0.5f;
            float a = std::fabs(mono[i]);
            if (a > peakIn) peakIn = a;
        }
    } else {
        std::memset(mono, 0, frameCount * sizeof(float));
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

        // Apply reflection convolution — feeds attenuated mono input through
        // per-voice convolution IR and accumulates the ambisonics result into
        // the shared mixer. We use the attenuated mono (matching the direct
        // output volume) so that reverb level is proportional to the source's
        // perceived loudness. The IR encodes reflection paths but not source
        // distance — without this, nearby sounds would have disproportionately
        // quiet reverb relative to their direct signal.
        if (node->reflectionsActive && node->reflectionEffect && node->reflectionMixer
            && node->reflectionParams.irSize > 0) {
            // Apply attenuation to mono before convolution (same factor as direct path)
            float reflAtten = node->lastAtten.load(std::memory_order_relaxed);
            if (runAtten && reflAtten < 1.0f) {
                for (ma_uint32 i = 0; i < frameCount; ++i)
                    mono[i] *= reflAtten;
            }

            // Steam Audio requires EXACTLY reflectionFrameSize samples per call.
            // In half-rate mode, decimate mono input from 48kHz to 24kHz.
            ma_uint32 reflFrames = static_cast<ma_uint32>(node->reflectionFrameSize);
            float* reflMonoPtr = mono;
            if (node->halfRate && !node->decimatedMono.empty()) {
                float* dec = node->decimatedMono.data();
                ma_uint32 pairs = std::min(frameCount / 2, reflFrames);
                for (ma_uint32 i = 0; i < pairs; ++i)
                    dec[i] = (mono[i * 2] + mono[i * 2 + 1]) * 0.5f;
                // Zero-pad if frameCount was short
                for (ma_uint32 i = pairs; i < reflFrames; ++i)
                    dec[i] = 0.0f;
                reflMonoPtr = dec;
            } else if (frameCount < reflFrames) {
                // Non-half-rate but short frame — pad mono with zeros
                for (ma_uint32 i = frameCount; i < reflFrames; ++i)
                    mono[i] = 0.0f;
            }

            IPLAudioBuffer reflIn{};
            reflIn.numChannels = 1;
            reflIn.numSamples = static_cast<IPLint32>(reflFrames);
            reflIn.data = &reflMonoPtr;

            // Output buffer required by Steam Audio even when mixer accumulates.
            // Channel count depends on ambisonics order (1 for order 0, 4 for order 1).
            float* ambiChPtrs[4] = {
                node->ambiScratch0.data(),
                !node->ambiScratch1.empty() ? node->ambiScratch1.data() : nullptr,
                !node->ambiScratch2.empty() ? node->ambiScratch2.data() : nullptr,
                !node->ambiScratch3.empty() ? node->ambiScratch3.data() : nullptr
            };
            IPLAudioBuffer reflOut{};
            reflOut.numChannels = node->reflectionParams.numChannels;
            reflOut.numSamples = static_cast<IPLint32>(reflFrames);
            reflOut.data = ambiChPtrs;

            iplReflectionEffectApply(node->reflectionEffect,
                                      &node->reflectionParams,
                                      &reflIn, &reflOut, node->reflectionMixer);
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

    // Profile: track peak per-voice DSP time and accumulate for total callback
    auto t1 = std::chrono::steady_clock::now();
    float us = std::chrono::duration<float, std::micro>(t1 - t0).count();
    float prev = sPerVoicePeakUs.load(std::memory_order_relaxed);
    if (us > prev) sPerVoicePeakUs.store(us, std::memory_order_relaxed);
    // Accumulate per-voice time for total callback measurement (C++17 CAS loop)
    float oldAccum = sCallbackAccumUs.load(std::memory_order_relaxed);
    while (!sCallbackAccumUs.compare_exchange_weak(oldAccum, oldAccum + us,
                                                     std::memory_order_relaxed)) {}
    sPerVoiceCallCount.fetch_add(1, std::memory_order_relaxed);
}

/// Audio-thread callback for the global reflection mix node.
/// Receives summed direct stereo from all per-voice DSP nodes (via miniaudio bus),
/// retrieves accumulated reflection convolution output from the shared mixer,
/// decodes from ambisonics to binaural stereo, and adds to the direct signal.
static void reflectionMixNodeProcess(ma_node* pNode, const float** ppFramesIn,
                                      ma_uint32* pFrameCountIn,
                                      float** ppFramesOut, ma_uint32* pFrameCountOut)
{
    auto mixT0 = std::chrono::steady_clock::now();
    auto* node = reinterpret_cast<AudioService::ReflectionMixNode*>(pNode);
    ma_uint32 frameCount = *pFrameCountOut;

    if (pFrameCountIn[0] < frameCount)
        frameCount = pFrameCountIn[0];
    if (frameCount > static_cast<ma_uint32>(node->frameSize))
        frameCount = static_cast<ma_uint32>(node->frameSize);

    float* stereoOut = ppFramesOut[0];
    const float* stereoIn = ppFramesIn[0];

    // Always pass through the direct audio (from per-voice DSP nodes)
    if (stereoIn && stereoOut && frameCount > 0)
        std::memcpy(stereoOut, stereoIn, frameCount * 2 * sizeof(float));

    // If reflection pipeline not ready or no simulation has run yet, just pass through.
    // Calling iplReflectionMixerApply before any simulation data exists can crash.
    if (!node->ready || !node->simulationRan || !node->mixer || !node->ambiDecodeEffect) {
        pFrameCountIn[0] = frameCount;
        *pFrameCountOut = frameCount;
        return;
    }

    // Steam Audio requires EXACTLY reflectionFrameSize samples per call.
    // Always use the configured size, not frameCount/2 (which may not match
    // if miniaudio delivers a non-standard frame count).
    ma_uint32 reflFrameCount = static_cast<ma_uint32>(node->reflectionFrameSize);

    // Step 1: Retrieve accumulated reflection ambisonics from the mixer.
    // Channel count depends on ambisonics order (1 for order 0, 4 for order 1).
    int nch = node->ambiChannels;
    float* ambiChannels[4] = {
        node->ambiCh0.data(),
        nch > 1 ? node->ambiCh1.data() : nullptr,
        nch > 2 ? node->ambiCh2.data() : nullptr,
        nch > 3 ? node->ambiCh3.data() : nullptr
    };

    IPLAudioBuffer ambiOut{};
    ambiOut.numChannels = nch;
    ambiOut.numSamples = static_cast<IPLint32>(reflFrameCount);
    ambiOut.data = ambiChannels;

    IPLReflectionEffectParams mixerParams{};
    mixerParams.type = IPL_REFLECTIONEFFECTTYPE_CONVOLUTION;
    mixerParams.numChannels = nch;

    iplReflectionMixerApply(node->mixer, &mixerParams, &ambiOut);

    // Step 2: Decode ambisonics to binaural stereo via HRTF
    float* decodedChannels[2] = {node->decodedL.data(), node->decodedR.data()};

    IPLAudioBuffer decodedBuf{};
    decodedBuf.numChannels = 2;
    decodedBuf.numSamples = static_cast<IPLint32>(reflFrameCount);
    decodedBuf.data = decodedChannels;

    IPLAmbisonicsDecodeEffectParams decodeParams{};
    decodeParams.order = node->ambiOrder;
    decodeParams.hrtf = node->hrtf;
    decodeParams.orientation = node->listenerOrientation;
    decodeParams.binaural = IPL_TRUE;

    iplAmbisonicsDecodeEffectApply(node->ambiDecodeEffect, &decodeParams,
                                   &ambiOut, &decodedBuf);

    // Step 3: Additive mix — reflection reverb on top of direct audio.
    // In half-rate mode, upsample the decoded stereo from reflFrameCount
    // to frameCount using linear interpolation (fine for diffuse reverb).
    // Clamp output to frameCount to avoid writing past the output buffer.
    if (node->halfRate) {
        ma_uint32 outSamples = std::min(reflFrameCount * 2, frameCount);
        ma_uint32 pairs = outSamples / 2;
        for (ma_uint32 i = 0; i < pairs; ++i) {
            float l0 = decodedChannels[0][i];
            float r0 = decodedChannels[1][i];
            float l1 = (i + 1 < reflFrameCount) ? decodedChannels[0][i + 1] : l0;
            float r1 = (i + 1 < reflFrameCount) ? decodedChannels[1][i + 1] : r0;
            // Each half-rate sample maps to 2 full-rate samples
            stereoOut[i * 4]     += l0;
            stereoOut[i * 4 + 1] += r0;
            if (i * 2 + 1 < outSamples) {
                stereoOut[i * 4 + 2] += (l0 + l1) * 0.5f;
                stereoOut[i * 4 + 3] += (r0 + r1) * 0.5f;
            }
        }
    } else {
        ma_uint32 outSamples = std::min(reflFrameCount, frameCount);
        for (ma_uint32 i = 0; i < outSamples; ++i) {
            stereoOut[i * 2]     += decodedChannels[0][i];
            stereoOut[i * 2 + 1] += decodedChannels[1][i];
        }
    }

    pFrameCountIn[0] = frameCount;
    *pFrameCountOut = frameCount;

    // Profile: track peak mix node time and total callback time
    auto mixT1 = std::chrono::steady_clock::now();
    float mixUs = std::chrono::duration<float, std::micro>(mixT1 - mixT0).count();
    float prevMix = sMixNodePeakUs.load(std::memory_order_relaxed);
    if (mixUs > prevMix) sMixNodePeakUs.store(mixUs, std::memory_order_relaxed);
    // Total = all per-voice DSP time + this mix node time
    float totalUs = sCallbackAccumUs.exchange(0.0f, std::memory_order_relaxed) + mixUs;
    float prevTotal = sTotalCallbackPeakUs.load(std::memory_order_relaxed);
    if (totalUs > prevTotal) sTotalCallbackPeakUs.store(totalUs, std::memory_order_relaxed);
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

    // Reverb tail timer: when the source audio finishes, the voice stays alive
    // to let the per-voice convolution tail ring out. The DSP callback feeds
    // silence into the convolution during this period. When the timer expires,
    // the voice is truly finished and can be cleaned up.
    bool sourceEnded = false;    // true after ma_sound reaches end
    float tailTimer = 0.0f;     // seconds remaining for reverb tail

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
        // Mark DSP as inactive FIRST — tells the audio thread to stop using
        // this node's effects immediately (before we uninit/release them).
        dspNode.effectsReady = false;
        dspNode.reflectionsActive = false;

        // Stop playback — use immediate stop here since ma_node_uninit below
        // will block until the audio thread finishes, providing a clean boundary.
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
        if (dspNode.reflectionEffect) {
            iplReflectionEffectRelease(&dspNode.reflectionEffect);
            dspNode.reflectionEffect = nullptr;
        }
        dspNode.reflectionMixer = nullptr;  // shared ref, not owned

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
/// Marks sourceEnded so the main thread can start the reverb tail timer.
/// The voice stays alive during the tail period so the per-voice convolution
/// continues feeding its IR tail into the reflection mixer.
static void onSoundEnd(void *pUserData, ma_sound * /*pSound*/)
{
    auto *voice = static_cast<ActiveVoice *>(pUserData);
    if (voice) {
        // Always defer to main thread via sourceEnded — the main thread
        // decides whether to start a tail timer or mark finished based on
        // whether the voice has a reflection effect (not just reflectionsActive,
        // which changes per-frame based on distance ranking).
        voice->sourceEnded = true;
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
                IPLMaterial mat = lookupAcousticMaterial(texName);
                // Apply transmission scale — higher values make sounds more
                // audible through walls (game-friendly vs physically accurate)
                if (mTransmissionScale != 1.0f) {
                    for (int b = 0; b < 3; ++b)
                        mat.transmission[b] = std::min(1.0f,
                            mat.transmission[b] * mTransmissionScale);
                }
                materials.push_back(mat);
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

        // Compute reflection pipeline rate — half-rate uses 24kHz (half of 48kHz)
        // for convolution, halving per-voice FFT cost while keeping HRTF at 48kHz.
        if (mHalfRateReflections) {
            mReflectionSampleRate = mDeviceSampleRate / 2;
            mReflectionFrameSize = mFrameSize / 2;
        } else {
            mReflectionSampleRate = mDeviceSampleRate;
            mReflectionFrameSize = mFrameSize;
        }

        // Step 6: Create the simulator for direct occlusion + reflections + reverb.
        // The simulator uses the reflection sample rate — IRs are generated at this
        // rate, which must match the reflection effects and mixer.
        IPLSimulationSettings simSettings{};
        simSettings.flags = static_cast<IPLSimulationFlags>(
            IPL_SIMULATIONFLAGS_DIRECT | IPL_SIMULATIONFLAGS_REFLECTIONS);
        simSettings.sceneType = IPL_SCENETYPE_DEFAULT;
        simSettings.reflectionType = IPL_REFLECTIONEFFECTTYPE_CONVOLUTION;
        simSettings.maxNumOcclusionSamples = 32;
        simSettings.maxNumRays = 4096;       // max rays (runtime uses mReflectionNumRays)
        simSettings.numDiffuseSamples = 32;
        simSettings.maxDuration = mReflectionDuration;
        simSettings.maxOrder = mAmbisonicsOrder;
        simSettings.maxNumSources = 32;      // voice pool size
        simSettings.numThreads = 2;          // parallel ray tracing
        simSettings.samplingRate = static_cast<IPLint32>(mReflectionSampleRate);
        simSettings.frameSize = static_cast<IPLint32>(mReflectionFrameSize);

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
        mAcousticTriCount = static_cast<int>(numTriangles);

        // Create the reflection pipeline (mixer + ambisonics decode + mix node)
        // This is optional — if it fails, direct effects still work
        std::fprintf(stderr, "REFL: about to call initReflectionPipeline()\n");
        if (!initReflectionPipeline()) {
            LOG_ERROR("AudioService: reflection pipeline init failed — "
                      "direct effects only");
        }
        std::fprintf(stderr, "REFL: initReflectionPipeline returned\n");

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
bool AudioService::initReflectionPipeline()
{
    std::fprintf(stderr, "REFL: initReflectionPipeline enter\n");
    if (!mIplContext || !mIplHrtf || !mMaEngine || !mIplSimulator)
        return false;

    // Reflection pipeline uses the reflection sample rate (24kHz or 48kHz).
    // This rate was computed in buildAcousticScene and must match the simulator.
    IPLAudioSettings audioSettings{};
    audioSettings.samplingRate = static_cast<IPLint32>(mReflectionSampleRate);
    audioSettings.frameSize = static_cast<IPLint32>(mReflectionFrameSize);

    // irSize = maxDuration * samplingRate (matches simulator settings)
    IPLint32 irSize = static_cast<IPLint32>(mReflectionDuration * mReflectionSampleRate);
    // Compute ambisonics channel count: (order+1)^2. Order 0=1ch, order 1=4ch.
    mAmbisonicsChannels = (mAmbisonicsOrder + 1) * (mAmbisonicsOrder + 1);
    IPLint32 numAmbiChannels = static_cast<IPLint32>(mAmbisonicsChannels);

    std::fprintf(stderr, "REFL: creating mixer (irSize=%d, channels=%d, rate=%d, frame=%d%s)\n",
                 irSize, numAmbiChannels,
                 audioSettings.samplingRate, audioSettings.frameSize,
                 mHalfRateReflections ? ", half-rate" : "");

    // Create shared reflection mixer — accumulates all per-voice convolution outputs
    IPLReflectionEffectSettings reflSettings{};
    reflSettings.type = IPL_REFLECTIONEFFECTTYPE_CONVOLUTION;
    reflSettings.irSize = irSize;
    reflSettings.numChannels = numAmbiChannels;

    IPLerror err = iplReflectionMixerCreate(mIplContext, &audioSettings,
                                             &reflSettings, &mIplReflectionMixer);
    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("AudioService: iplReflectionMixerCreate failed (error %d)", err);
        return false;
    }
    std::fprintf(stderr, "REFL: mixer created OK\n");

    // Create ambisonics decode effect (ambisonics → binaural stereo via HRTF).
    // This also runs at the reflection rate — output is upsampled in the mix node.
    IPLAmbisonicsDecodeEffectSettings decodeSettings{};
    decodeSettings.speakerLayout.type = IPL_SPEAKERLAYOUTTYPE_STEREO;
    decodeSettings.hrtf = mIplHrtf;
    decodeSettings.maxOrder = mAmbisonicsOrder;

    err = iplAmbisonicsDecodeEffectCreate(mIplContext, &audioSettings,
                                           &decodeSettings, &mIplAmbiDecodeEffect);
    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("AudioService: iplAmbisonicsDecodeEffectCreate failed (error %d)", err);
        iplReflectionMixerRelease(&mIplReflectionMixer);
        mIplReflectionMixer = nullptr;
        return false;
    }

    // Create global reflection mix node in the miniaudio node graph.
    // The node operates at the engine's 48kHz rate; it internally handles
    // resampling when the reflection pipeline runs at half rate.
    mReflectionMixNode = std::make_unique<ReflectionMixNode>();
    auto& rmn = *mReflectionMixNode;
    rmn.mixer = mIplReflectionMixer;
    rmn.ambiDecodeEffect = mIplAmbiDecodeEffect;
    rmn.hrtf = mIplHrtf;
    rmn.frameSize = static_cast<int>(mFrameSize);
    rmn.reflectionFrameSize = static_cast<int>(mReflectionFrameSize);
    rmn.halfRate = mHalfRateReflections;
    rmn.ambiChannels = mAmbisonicsChannels;
    rmn.ambiOrder = mAmbisonicsOrder;

    // Allocate scratch buffers at the reflection frame size.
    // Only allocate channels needed for the configured ambisonics order.
    rmn.ambiCh0.resize(mReflectionFrameSize, 0.0f);
    if (mAmbisonicsChannels > 1) rmn.ambiCh1.resize(mReflectionFrameSize, 0.0f);
    if (mAmbisonicsChannels > 2) rmn.ambiCh2.resize(mReflectionFrameSize, 0.0f);
    if (mAmbisonicsChannels > 3) rmn.ambiCh3.resize(mReflectionFrameSize, 0.0f);
    rmn.decodedL.resize(mReflectionFrameSize, 0.0f);
    rmn.decodedR.resize(mReflectionFrameSize, 0.0f);

    // Initialize the miniaudio node
    ma_uint32 inputChannels[1] = {2};
    ma_uint32 outputChannels[1] = {2};

    ma_node_config nodeConfig = ma_node_config_init();
    nodeConfig.vtable = &sReflectionMixNodeVtable;
    nodeConfig.inputBusCount = 1;
    nodeConfig.outputBusCount = 1;
    nodeConfig.pInputChannels = inputChannels;
    nodeConfig.pOutputChannels = outputChannels;

    ma_result result = ma_node_init(ma_engine_get_node_graph(mMaEngine),
                                     &nodeConfig, nullptr, &rmn.base);
    if (result != MA_SUCCESS) {
        LOG_ERROR("AudioService: ReflectionMixNode init failed (error %d)", result);
        mReflectionMixNode.reset();
        iplAmbisonicsDecodeEffectRelease(&mIplAmbiDecodeEffect);
        mIplAmbiDecodeEffect = nullptr;
        iplReflectionMixerRelease(&mIplReflectionMixer);
        mIplReflectionMixer = nullptr;
        return false;
    }
    rmn.nodeInitialized = true;
    std::fprintf(stderr, "REFL: node initialized, attaching to endpoint\n");

    // Connect: reflection mix node → engine endpoint
    ma_result r = ma_node_attach_output_bus(&rmn.base, 0,
                                             ma_engine_get_endpoint(mMaEngine), 0);
    if (r != MA_SUCCESS) {
        LOG_ERROR("AudioService: ReflectionMixNode attach failed (error %d)", r);
        ma_node_uninit(&rmn.base, nullptr);
        mReflectionMixNode.reset();
        iplAmbisonicsDecodeEffectRelease(&mIplAmbiDecodeEffect);
        mIplAmbiDecodeEffect = nullptr;
        iplReflectionMixerRelease(&mIplReflectionMixer);
        mIplReflectionMixer = nullptr;
        return false;
    }

    rmn.ready = true;

    std::fprintf(stderr, "AudioService: reflection pipeline initialized "
                 "(convolution, order %d (%dch), IR %d samples, %uHz%s, max %d voices)\n",
                 mAmbisonicsOrder, mAmbisonicsChannels,
                 irSize, mReflectionSampleRate,
                 mHalfRateReflections ? " half-rate" : "",
                 mMaxReflectionVoices);
    return true;
}

//------------------------------------------------------
void AudioService::destroyReflectionPipeline()
{
    // Wait for any in-flight background reflection simulation to finish
    mReflectionShutdown.store(true, std::memory_order_release);
    if (mReflectionThread.joinable())
        mReflectionThread.join();

    // Flush deferred IPL source removals before destroying the simulator
    if (!mPendingSourceRemovals.empty() && mIplSimulator) {
        for (auto &src : mPendingSourceRemovals) {
            iplSourceRemove(src, mIplSimulator);
            iplSourceRelease(&src);
        }
        mPendingSourceRemovals.clear();
    }

    // Order matters: mix node references mixer/decode, so destroy it first
    if (mReflectionMixNode) {
        mReflectionMixNode->ready = false;
        if (mReflectionMixNode->nodeInitialized) {
            ma_node_uninit(&mReflectionMixNode->base, nullptr);
        }
        mReflectionMixNode.reset();
    }

    if (mIplAmbiDecodeEffect) {
        iplAmbisonicsDecodeEffectRelease(&mIplAmbiDecodeEffect);
        mIplAmbiDecodeEffect = nullptr;
    }
    if (mIplReflectionMixer) {
        iplReflectionMixerRelease(&mIplReflectionMixer);
        mIplReflectionMixer = nullptr;
    }
}

//------------------------------------------------------
void AudioService::destroyAcousticScene()
{
    mSceneReady = false;

    // Destroy reflection pipeline before simulator (it references simulator sources)
    destroyReflectionPipeline();

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

    // Source mutations (add/remove/commit) must not happen while the background
    // simulation thread is running — iplSimulatorRunDirect/Reflections holds
    // internal pointers to committed sources. Instead of BLOCKING the main thread
    // (which caused 200+ms stalls), we DEFER mutations until the sim finishes.
    bool simBusy = mReflectionSimRunning.load(std::memory_order_acquire);

    if (!simBusy) {
        // Background sim is idle — safe to mutate sources
        if (mReflectionThread.joinable())
            mReflectionThread.join();

        // Flush deferred IPL source removals (accumulated while sim was busy)
        if (!mPendingSourceRemovals.empty() && mIplSimulator) {
            for (auto &src : mPendingSourceRemovals) {
                iplSourceRemove(src, mIplSimulator);
                iplSourceRelease(&src);
            }
            mPendingSourceRemovals.clear();
        }

        // Remove voices that have finished playback
        cleanupFinishedVoices();

        // Batch-commit any pending source additions/removals before simulation.
        if (mSimulatorDirty && mIplSimulator) {
            auto ct0 = std::chrono::steady_clock::now();
            iplSimulatorCommit(mIplSimulator);
            auto ct1 = std::chrono::steady_clock::now();
            float cMs = std::chrono::duration<float, std::milli>(ct1 - ct0).count();
            float prevC = sCommitPeakMs.load(std::memory_order_relaxed);
            if (cMs > prevC) sCommitPeakMs.store(cMs, std::memory_order_relaxed);
            mSimulatorDirty = false;
        }
    } else {
        // Sim is running — still clean up finished voices to free slots for
        // new sounds (e.g., footsteps). IPL source removal is deferred
        // automatically by removeVoiceSource when sim is busy.
        cleanupFinishedVoices();
    }

    // Run Steam Audio simulation for all active sources
    if (mSceneReady && mIplSimulator && !mVoices.empty()) {
        float cosY = std::cos(mListenerYaw), sinY = std::sin(mListenerYaw);
        float cosP = std::cos(mListenerPitch), sinP = std::sin(mListenerPitch);

        IPLCoordinateSpace3 listenerCoord{};
        listenerCoord.origin = {mListenerPos.x, mListenerPos.y, mListenerPos.z};
        listenerCoord.ahead  = {cosY * cosP, sinY * cosP, sinP};
        listenerCoord.right  = {sinY, -cosY, 0.0f};
        listenerCoord.up     = {-cosY * sinP, -sinY * sinP, cosP};

        // Update listener orientation for ambisonics decode (audio thread reads this)
        if (mReflectionMixNode && mReflectionMixNode->ready) {
            mReflectionMixNode->listenerOrientation = listenerCoord;
        }

        // Steps 1-3: Set simulation inputs and launch background sim.
        // All iplSimulator/iplSource calls must happen when the sim thread is
        // idle to prevent data races with iplSimulatorRunDirect/Reflections.
        if (!simBusy) {
            // Step 1: Set listener position for the simulator
            IPLSimulationSharedInputs sharedInputs{};
            sharedInputs.listener = listenerCoord;
            sharedInputs.numRays = mReflectionNumRays;
            sharedInputs.numBounces = mReflectionNumBounces;
            sharedInputs.duration = mReflectionDuration;
            sharedInputs.order = mAmbisonicsOrder;
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

            // Step 3: Launch simulation on background thread
            bool runReflections = mReflectionsEnabled && mIplReflectionMixer
                && (++mReflectionFrameCounter >= mReflectionThrottle);
            if (runReflections)
                mReflectionFrameCounter = 0;

            mReflectionSimRunning.store(true, std::memory_order_release);
            mReflectionThread = std::thread([this, runReflections]() {
                iplSimulatorRunDirect(mIplSimulator);
                if (runReflections) {
                    iplSimulatorRunReflections(mIplSimulator);
                    if (mReflectionMixNode)
                        mReflectionMixNode->simulationRan = true;
                }
                mReflectionSimRunning.store(false, std::memory_order_release);
            });
        }

        // Step 4: Read back simulation results and feed to DSP nodes.
        // Results are from the previous frame's simulation (1-frame latency) since
        // we moved both direct and reflection sim to the background thread.
        // This is imperceptible for both attenuation and reverb changes.
        // Steam Audio's triple buffer ensures iplSourceGetOutputs returns the
        // latest completed results without blocking.

        // Select the N closest voices for reflection convolution.
        // Per-voice convolution is expensive (~98M FLOPs per voice per audio
        // frame with a 96K IR), so we limit to mMaxReflectionVoices sources.
        // Remaining voices get direct path only (HRTF + occlusion), which is
        // cheap and still sounds great.
        struct VoiceDist {
            SoundHandle handle;
            float distSq;
        };
        std::vector<VoiceDist> reflCandidates;
        if (mReflectionsEnabled) {
            reflCandidates.reserve(mVoices.size());
            for (auto &[h, v] : mVoices) {
                // Skip tail voices — they've finished playing and are just
                // ringing out their convolution tail. Don't let them steal
                // reflection slots from voices that are actively producing audio.
                if (v->sourceEnded)
                    continue;
                if (v->iplSource && v->dspNode.effectsReady && v->dspNode.reflectionEffect) {
                    Vector3 delta = v->worldPos - mListenerPos;
                    reflCandidates.push_back({h, glm::dot(delta, delta)});
                }
            }
            // Partial sort: only need the N closest
            if (static_cast<int>(reflCandidates.size()) > mMaxReflectionVoices) {
                std::partial_sort(reflCandidates.begin(),
                                  reflCandidates.begin() + mMaxReflectionVoices,
                                  reflCandidates.end(),
                                  [](const VoiceDist &a, const VoiceDist &b) {
                                      return a.distSq < b.distSq;
                                  });
                reflCandidates.resize(mMaxReflectionVoices);
            }
        }

        for (auto &[handle, voice] : mVoices) {
            if (!voice->iplSource)
                continue;

            IPLSimulationOutputs outputs{};
            IPLSimulationFlags getFlags = static_cast<IPLSimulationFlags>(
                IPL_SIMULATIONFLAGS_DIRECT |
                (mReflectionsEnabled ? IPL_SIMULATIONFLAGS_REFLECTIONS : 0));
            iplSourceGetOutputs(voice->iplSource, getFlags, &outputs);

            if (voice->dspNode.effectsReady) {
                // Copy simulation outputs to DSP node params. Guard against
                // invalid/zero outputs from the simulation — before the first
                // background sim run, outputs may contain zeros which would
                // silence the voice. Keep defaults (unity) until sim catches up.
                if (outputs.direct.distanceAttenuation > 0.0f) {
                    voice->dspNode.directParams = outputs.direct;
                    voice->dspNode.directParams.flags = static_cast<IPLDirectEffectFlags>(
                        IPL_DIRECTEFFECTFLAGS_APPLYDISTANCEATTENUATION |
                        IPL_DIRECTEFFECTFLAGS_APPLYAIRABSORPTION |
                        IPL_DIRECTEFFECTFLAGS_APPLYOCCLUSION |
                        IPL_DIRECTEFFECTFLAGS_APPLYTRANSMISSION);
                    voice->dspNode.directParams.transmissionType =
                        IPL_TRANSMISSIONTYPE_FREQDEPENDENT;
                }

                // Only enable reflection convolution for the N closest voices
                bool isReflVoice = false;
                for (const auto &rc : reflCandidates) {
                    if (rc.handle == handle) { isReflVoice = true; break; }
                }
                if (isReflVoice) {
                    voice->dspNode.reflectionParams = outputs.reflections;
                    voice->dspNode.reflectionParams.type =
                        IPL_REFLECTIONEFFECTTYPE_CONVOLUTION;
                    voice->dspNode.reflectionParams.numChannels = mAmbisonicsChannels;
                    voice->dspNode.reflectionsActive = true;
                } else {
                    voice->dspNode.reflectionsActive = false;
                }

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

    // Tick reverb tail timers for voices whose source audio has ended.
    // The voice stays alive during the tail so the per-voice convolution
    // continues feeding its IR tail into the reflection mixer.
    for (auto &[handle, voice] : mVoices) {
        if (voice->sourceEnded && !voice->finished.load(std::memory_order_relaxed)) {
            if (voice->tailTimer <= 0.0f) {
                // Start the tail timer if this voice has a reflection effect.
                // Check reflectionEffect (was it created with one?) not
                // reflectionsActive (is it in the top N right now?) — a voice
                // may have been active earlier and accumulated convolution state.
                if (voice->dspNode.reflectionEffect) {
                    // Use half the IR duration — the tail decays exponentially,
                    // so most energy is in the first half. Keeps voice slots free.
                    voice->tailTimer = mReflectionDuration * 0.5f;
                    // Ensure convolution stays active during the tail so the
                    // IR rings out naturally (feed silence → convolution outputs tail)
                    voice->dspNode.reflectionsActive = true;
                    std::fprintf(stderr, "[VOICE] TAIL_START h=%d '%s' tail=%.1fs\n",
                                 handle, voice->schemaName.c_str(), voice->tailTimer);
                } else {
                    voice->finished.store(true, std::memory_order_release);
                    std::fprintf(stderr, "[VOICE] END_NOTAIL h=%d '%s'\n",
                                 handle, voice->schemaName.c_str());
                }
            } else {
                voice->tailTimer -= deltaTime;
                if (voice->tailTimer <= 0.0f) {
                    voice->finished.store(true, std::memory_order_release);
                    std::fprintf(stderr, "[VOICE] TAIL_DONE h=%d '%s'\n",
                                 handle, voice->schemaName.c_str());
                }
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
            std::fprintf(stderr, "[VOICE] CLEANUP h=%d '%s' srcEnded=%d tail=%.1f\n",
                         it->second->handle, it->second->schemaName.c_str(),
                         it->second->sourceEnded ? 1 : 0, it->second->tailTimer);
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
    mSimulatorDirty = true;
}

//------------------------------------------------------
void AudioService::waitForReflectionThread()
{
    // If the background reflection thread is running, wait for it to finish.
    // This prevents Steam Audio's internal data structures from being accessed
    // while we modify the source list (add/remove/commit).
    if (mReflectionThread.joinable()) {
        mReflectionThread.join();
    }
    mReflectionSimRunning.store(false, std::memory_order_release);
}

//------------------------------------------------------
void AudioService::removeVoiceSource(ActiveVoice &voice)
{
    if (!voice.iplSource)
        return;

    // If the background sim thread is running, we can't safely call
    // iplSourceRemove (it races with iplSimulatorRunDirect/Reflections).
    // Defer the removal — queue the IPL source handle for later cleanup.
    if (mReflectionSimRunning.load(std::memory_order_acquire)) {
        mPendingSourceRemovals.push_back(voice.iplSource);
        voice.iplSource = nullptr;  // detach from voice (we own it now)
        mSimulatorDirty = true;
        return;
    }

    if (mIplSimulator) {
        iplSourceRemove(voice.iplSource, mIplSimulator);
        mSimulatorDirty = true;
    }
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
    dsp.reflectionFrameSize = static_cast<int>(mReflectionFrameSize);
    dsp.halfRate = mHalfRateReflections;

    // Allocate processing buffers (once, never reallocated — audio thread safe)
    dsp.monoScratch.resize(dsp.frameSize);
    dsp.stereoL.resize(dsp.frameSize);
    dsp.stereoR.resize(dsp.frameSize);
    dsp.ambiScratch0.resize(mReflectionFrameSize, 0.0f);
    if (mAmbisonicsChannels > 1) dsp.ambiScratch1.resize(mReflectionFrameSize, 0.0f);
    if (mAmbisonicsChannels > 2) dsp.ambiScratch2.resize(mReflectionFrameSize, 0.0f);
    if (mAmbisonicsChannels > 3) dsp.ambiScratch3.resize(mReflectionFrameSize, 0.0f);
    if (mHalfRateReflections)
        dsp.decimatedMono.resize(mReflectionFrameSize, 0.0f);

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

    // Create per-voice reflection convolution effect (optional — only if mixer exists).
    // Uses the reflection sample rate (24kHz or 48kHz) to match the mixer and simulator.
    if (mIplReflectionMixer) {
        IPLint32 irSize = static_cast<IPLint32>(mReflectionDuration * mReflectionSampleRate);

        IPLAudioSettings reflAudioSettings{};
        reflAudioSettings.samplingRate = static_cast<IPLint32>(mReflectionSampleRate);
        reflAudioSettings.frameSize = static_cast<IPLint32>(mReflectionFrameSize);

        IPLReflectionEffectSettings reflSettings{};
        reflSettings.type = IPL_REFLECTIONEFFECTTYPE_CONVOLUTION;
        reflSettings.irSize = irSize;
        reflSettings.numChannels = static_cast<IPLint32>(mAmbisonicsChannels);

        err = iplReflectionEffectCreate(mIplContext, &reflAudioSettings,
                                         &reflSettings, &dsp.reflectionEffect);
        if (err == IPL_STATUS_SUCCESS) {
            dsp.reflectionMixer = mIplReflectionMixer;
        } else {
            LOG_ERROR("AudioService: iplReflectionEffectCreate failed (error %d) "
                      "— direct effects only for this voice", err);
            // Continue without reflections for this voice
        }
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
        if (dsp.reflectionEffect) {
            iplReflectionEffectRelease(&dsp.reflectionEffect);
            dsp.reflectionEffect = nullptr;
        }
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

    // Connect the node graph BEFORE setting effectsReady — prevents the audio
    // thread from trying to process through an unconnected node.
    //   ma_sound → DSP node → ReflectionMixNode → engine endpoint
    // If no reflection mix node, fall back to DSP node → engine endpoint
    ma_node* outputTarget = (mReflectionMixNode && mReflectionMixNode->nodeInitialized)
        ? reinterpret_cast<ma_node*>(&mReflectionMixNode->base)
        : static_cast<ma_node*>(ma_engine_get_endpoint(mMaEngine));

    ma_result r1 = ma_node_attach_output_bus(&voice.sound, 0, &dsp.base, 0);
    ma_result r2 = ma_node_attach_output_bus(&dsp.base, 0, outputTarget, 0);

    if (r1 != MA_SUCCESS || r2 != MA_SUCCESS) {
        LOG_ERROR("AudioService: DSP node attach failed: sound→dsp=%d, dsp→target=%d",
                  r1, r2);
        // Don't set effectsReady — the fallback in startVoice will
        // connect the sound directly to the endpoint
        return;
    }

    // Only mark ready AFTER the graph is fully wired — the audio thread
    // checks this flag before using any Steam Audio effects.
    dsp.effectsReady = true;
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

    std::fprintf(stderr, "[VOICE] EVICT h=%d pri=%d for new pri=%d\n",
                 lowestHandle, lowestPriority, newPriority);
    haltSound(lowestHandle);
    return true;
}

//------------------------------------------------------
SoundHandle AudioService::startVoice(const std::string &schemaName,
                                      const std::string &sampleName,
                                      const Vector3 &position,
                                      int priority, bool looping,
                                      int objID, float volume)
{
    if (!mMaEngine || !mSoundLoader)
        return SOUND_HANDLE_INVALID;

    // Enforce voice limit — evict lowest priority if full
    if (static_cast<int>(mVoices.size()) >= MAX_ACTIVE_VOICES) {
        if (!evictLowestPriority(priority)) {
            std::fprintf(stderr, "[VOICE] POOL_FULL cannot play '%s' pri=%d voices=%zu\n",
                         schemaName.c_str(), priority, mVoices.size());
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

    // DIAGNOSTIC: set to true to bypass Steam Audio entirely and test
    // raw miniaudio playback. If sounds are still cut off without Steam Audio,
    // the bug is in miniaudio/decoder/data, not in our DSP pipeline.
    static const bool sBypassSteamAudio = (std::getenv("DARKNESS_NO_STEAMAUDIO") != nullptr);

    if (!sBypassSteamAudio) {
        // Set up Steam Audio DSP pipeline (direct + binaural effects)
        initVoiceDSP(*voice);
    }

    // If DSP setup failed, not available, or bypassed — connect directly to endpoint
    if (!voice->dspNode.effectsReady) {
        if (sBypassSteamAudio) {
            std::fprintf(stderr, "[VOICE] BYPASS '%s' — Steam Audio disabled via env\n",
                         schemaName.c_str());
        } else {
            std::fprintf(stderr, "[VOICE] DSP_FAIL '%s' sample='%s' — raw bypass to endpoint\n",
                         schemaName.c_str(), sampleName.c_str());
        }
        ma_node_attach_output_bus(&voice->sound, 0,
                                  ma_engine_get_endpoint(mMaEngine), 0);
    }

    // Create Steam Audio source for spatial simulation
    if (!sBypassSteamAudio)
        createVoiceSource(*voice);

    // Set volume BEFORE starting playback — prevents the audio thread from
    // pulling the first frames at a wrong volume level.
    ma_sound_set_volume(&voice->sound, volume);

    ma_result startResult = ma_sound_start(&voice->sound);
    if (startResult != MA_SUCCESS) {
        LOG_ERROR("AudioService: failed to start sound '%s' (error %d)",
                  sampleName.c_str(), startResult);
        return SOUND_HANDLE_INVALID;
    }

    // Register end callback AFTER successful start (avoids dangling pointer if start fails)
    ma_sound_set_end_callback(&voice->sound, onSoundEnd, voice.get());

    SoundHandle h = voice->handle;

    // Diagnostic: check decoded audio properties
    ma_uint64 totalFrames = 0;
    ma_decoder_get_length_in_pcm_frames(&voice->decoder, &totalFrames);
    ma_uint32 decRate = voice->decoder.outputSampleRate;
    ma_uint32 decCh = voice->decoder.outputChannels;
    float durMs = (decRate > 0) ? (totalFrames * 1000.0f / decRate) : 0.0f;

    std::fprintf(stderr, "[VOICE] START h=%d '%s' sample='%s' pri=%d voices=%zu "
                 "wavBytes=%zu decoded=%lluframes %uHz %uch %.0fms\n",
                 h, schemaName.c_str(), sampleName.c_str(),
                 priority, mVoices.size() + 1,
                 voice->data.wavData.size(), (unsigned long long)totalFrames,
                 decRate, decCh, durMs);
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

    float vol = schemaVolumeToLinear(schema->playParams.volume);
    SoundHandle h = startVoice(schemaName, sample.name, position,
                               schema->playParams.priority, looping, 0, vol);
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

    float vol = schemaVolumeToLinear(schema->playParams.volume);
    SoundHandle h = startVoice(schemaName, sample.name, pos,
                               schema->playParams.priority, looping, objID, vol);
    return h;
}

//------------------------------------------------------
void AudioService::haltSound(SoundHandle handle)
{
    if (handle == SOUND_HANDLE_INVALID)
        return;

    auto it = mVoices.find(handle);
    if (it != mVoices.end()) {
        auto &voice = *it->second;
        std::fprintf(stderr, "[VOICE] HALT h=%d '%s' hasRefl=%d\n",
                     handle, voice.schemaName.c_str(),
                     voice.dspNode.reflectionEffect ? 1 : 0);
        if (voice.initialized) {
            // Fade out over 15ms to prevent click/pop from abrupt stop.
            // miniaudio schedules the stop after the fade completes.
            ma_sound_stop_with_fade_in_milliseconds(&voice.sound, 15);
        }
        // Don't destroy immediately — mark sourceEnded so the tail timer
        // can let any reverb ring out naturally. For voices without a
        // reflection effect, the tail timer will mark finished immediately.
        voice.sourceEnded = true;
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
        // Read and reset audio thread profiling counters
        float voiceUs = sPerVoicePeakUs.exchange(0.0f, std::memory_order_relaxed);
        float mixUs   = sMixNodePeakUs.exchange(0.0f, std::memory_order_relaxed);
        float totalUs = sTotalCallbackPeakUs.exchange(0.0f, std::memory_order_relaxed);
        int   voiceCalls = sPerVoiceCallCount.exchange(0, std::memory_order_relaxed);
        float waitMs  = sWaitThreadPeakMs.exchange(0.0f, std::memory_order_relaxed);
        float commitMs = sCommitPeakMs.exchange(0.0f, std::memory_order_relaxed);
        (void)voiceCalls;

        // Audio budget: 1024 samples @ 48kHz = 21333µs per callback.
        float budgetUs = (static_cast<float>(mFrameSize) / mDeviceSampleRate) * 1e6f;
        float loadPct = (totalUs / budgetUs) * 100.0f;

        int reflVoices = 0;
        int tailVoices = 0;
        for (auto &[h, v] : mVoices) {
            if (v->dspNode.reflectionsActive) ++reflVoices;
            if (v->sourceEnded && !v->finished.load(std::memory_order_relaxed)) ++tailVoices;
        }

        std::fprintf(stderr, "[Audio] %zu voices (%d refl, %d tail), %d/%zu ambients | "
                     "total=%.0f/%.0fµs (%.0f%%) peak_voice=%.0fµs mix=%.0fµs | "
                     "main: wait=%.1fms commit=%.1fms\n",
                     mVoices.size(), reflVoices, tailVoices, playing, mAmbients.size(),
                     totalUs, budgetUs, loadPct,
                     voiceUs, mixUs,
                     waitMs, commitMs);
    }

    for (auto &amb : mAmbients) {
        float dist = glm::length(mListenerPos - amb.position);
        // Hysteresis: start at radius, stop at radius * 1.1.
        // Prevents voice churn when listener hovers near the boundary —
        // each start/stop creates/destroys IPL effects and triggers
        // iplSimulatorCommit, which is expensive with many ambients.
        bool alreadyPlaying = (amb.handle != SOUND_HANDLE_INVALID);
        float stopRadius = amb.radius * 1.5f;
        bool inRange = (amb.radius > 0.0f &&
                        (alreadyPlaying ? dist < stopRadius : dist < amb.radius));

        if (inRange) {
            // Start voice if not already playing
            if (amb.handle == SOUND_HANDLE_INVALID) {
                bool isLooping = !(amb.flags & AMB_ONCE_ONLY);
                if (mSchemaParser) {
                    const SchemaEntry *schema = mSchemaParser->findSchema(amb.schemaName);
                    if (schema && !schema->samples.empty()) {
                        const SchemaSample &sample = schema->samples[0];
                        float ambVol = schemaVolumeToLinear(schema->playParams.volume);
                        amb.handle = startVoice(amb.schemaName, sample.name, amb.position,
                                                schema->playParams.priority, isLooping,
                                                amb.objID, ambVol);
                    }
                }
                // Fallback: try loading schema name as a raw sound
                if (amb.handle == SOUND_HANDLE_INVALID) {
                    amb.handle = startVoice(amb.schemaName, amb.schemaName, amb.position,
                                            64, isLooping, amb.objID,
                                            schemaVolumeToLinear(amb.volume));
                }
            }

            // Update volume based on distance
            if (amb.handle != SOUND_HANDLE_INVALID) {
                auto it = mVoices.find(amb.handle);
                if (it == mVoices.end()) {
                    amb.handle = SOUND_HANDLE_INVALID;
                    continue;
                }

                float falloff = std::max(0.0f, 1.0f - (dist / amb.radius));
                if (!(amb.flags & AMB_NO_FADE)) {
                    falloff *= falloff;  // quadratic curve for natural fade
                }
                // Only apply schema base volume here — Steam Audio's DSP handles
                // distance attenuation via simulation. Don't double-attenuate.
                float baseVol = schemaVolumeToLinear(amb.volume);
                if (it->second->dspNode.effectsReady) {
                    // DSP active: set base volume only, let Steam Audio handle distance
                    ma_sound_set_volume(&it->second->sound, baseVol);
                } else {
                    // No DSP: apply manual distance falloff
                    ma_sound_set_volume(&it->second->sound, baseVol * falloff);
                }
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
                               schema->playParams.priority, false, 0, finalVol);

    if (h != SOUND_HANDLE_INVALID && mVoices.count(h)) {
        // Diagnostic: count concurrent footstep voices (including tails)
        int footActive = 0, footTail = 0;
        for (auto &[fh, fv] : mVoices) {
            if (fv->schemaName.find("foot_") == 0 || fv->schemaName.find("land_") == 0) {
                if (fv->sourceEnded) ++footTail;
                else ++footActive;
            }
        }
        std::fprintf(stderr, "[FOOT] h=%d '%s' vol=%.2f spd=%.1f active=%d tail=%d\n",
                     h, sample.name.c_str(), finalVol, speed, footActive, footTail);
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
                               schema->playParams.priority, false, 0, finalVol);
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
