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
#include <sys/stat.h>
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
// Acoustic material properties: absorption, scattering, and transmission per
// frequency band (low ~400Hz, mid ~2.5kHz, high ~15kHz).
//
// Scattering coefficients updated from published ISO 17497 measurements and
// ODEON/Treble Technologies guidelines (Rindel, Zeng, Christensen 2006;
// Cox & D'Antonio "Acoustic Absorbers and Diffusers"). Steam Audio's built-in
// presets use 0.05 for all materials, which is a placeholder — real surfaces
// scatter significantly more, especially rough medieval stone and brick.
//
// Absorption and transmission values from published coefficient tables.
static const std::unordered_map<std::string, IPLMaterial> kKeywordToIPLMaterial = {
    //                             absorption{lo, mid, hi}     scat   transmission{lo, mid, hi}
    {"concrete", {{ 0.05f, 0.07f, 0.08f }, 0.20f, { 0.015f, 0.015f, 0.015f }}},  // rough concrete (ODEON: 0.10-0.20)
    {"ceramic",  {{ 0.01f, 0.02f, 0.02f }, 0.05f, { 0.060f, 0.044f, 0.011f }}},  // smooth glazed (specular)
    {"plaster",  {{ 0.12f, 0.06f, 0.04f }, 0.10f, { 0.056f, 0.056f, 0.004f }}},  // old rough plaster (ODEON: 0.05-0.10)
    {"carpet",   {{ 0.24f, 0.69f, 0.73f }, 0.10f, { 0.020f, 0.005f, 0.003f }}},  // flat carpet (geometrically smooth)
    {"gravel",   {{ 0.60f, 0.70f, 0.80f }, 0.60f, { 0.031f, 0.012f, 0.008f }}},  // highly irregular surface
    {"brick",    {{ 0.03f, 0.04f, 0.07f }, 0.15f, { 0.015f, 0.015f, 0.015f }}},  // open-joint brickwork (ODEON: 0.10-0.20)
    {"glass",    {{ 0.06f, 0.03f, 0.02f }, 0.05f, { 0.060f, 0.044f, 0.011f }}},  // smooth flat glass (very specular)
    {"stone",    {{ 0.13f, 0.20f, 0.24f }, 0.35f, { 0.015f, 0.002f, 0.001f }}},  // rough medieval stone (ODEON: 0.25-0.35 for coursed masonry)
    {"metal",    {{ 0.20f, 0.07f, 0.06f }, 0.10f, { 0.250f, 0.190f, 0.080f }}},  // smooth sheet metal
    {"wood",     {{ 0.11f, 0.07f, 0.06f }, 0.15f, { 0.070f, 0.014f, 0.005f }}},  // planks with gaps (Treble: 0.10-0.20)
    {"rock",     {{ 0.13f, 0.20f, 0.24f }, 0.35f, { 0.015f, 0.002f, 0.001f }}},  // rough natural rock
    {"tile",     {{ 0.01f, 0.02f, 0.02f }, 0.05f, { 0.060f, 0.044f, 0.011f }}},  // smooth glazed tile
    {"dirt",     {{ 0.60f, 0.70f, 0.80f }, 0.50f, { 0.031f, 0.012f, 0.008f }}},  // packed earth
    {"ice",      {{ 0.01f, 0.02f, 0.02f }, 0.10f, { 0.060f, 0.044f, 0.011f }}},  // rough/granular ice
    // Portal polygons — high transmission so sound passes through doorways
    {"_portal",  {{ 0.01f, 0.01f, 0.01f }, 0.05f, { 0.950f, 0.950f, 0.950f }}},
    // Floor family prefixes — checked before generic "floor" keyword
    {"wfloor",   {{ 0.11f, 0.07f, 0.06f }, 0.15f, { 0.070f, 0.014f, 0.005f }}},  // → wood
    {"sfloor",   {{ 0.13f, 0.20f, 0.24f }, 0.35f, { 0.015f, 0.002f, 0.001f }}},  // → stone
    {"mfloor",   {{ 0.20f, 0.07f, 0.06f }, 0.10f, { 0.250f, 0.190f, 0.080f }}},  // → metal
    // Aliases
    {"bronze",   {{ 0.20f, 0.07f, 0.06f }, 0.10f, { 0.250f, 0.190f, 0.080f }}},  // → smooth metal
    {"grate",    {{ 0.20f, 0.07f, 0.06f }, 0.35f, { 0.250f, 0.190f, 0.080f }}},  // → perforated metal (high scatter)
    {"plate",    {{ 0.20f, 0.07f, 0.06f }, 0.10f, { 0.250f, 0.190f, 0.080f }}},  // → smooth metal
    {"steel",    {{ 0.20f, 0.07f, 0.06f }, 0.10f, { 0.250f, 0.190f, 0.080f }}},  // → smooth metal
    {"pipe",     {{ 0.20f, 0.07f, 0.06f }, 0.30f, { 0.250f, 0.190f, 0.080f }}},  // → cylindrical (moderate scatter)
    {"floor",    {{ 0.13f, 0.20f, 0.24f }, 0.20f, { 0.015f, 0.002f, 0.001f }}},  // → stone (generic fallback)
    {"earth",    {{ 0.60f, 0.70f, 0.80f }, 0.50f, { 0.031f, 0.012f, 0.008f }}},  // → dirt
    {"metl",     {{ 0.20f, 0.07f, 0.06f }, 0.10f, { 0.250f, 0.190f, 0.080f }}},  // → metal (Thief 2 abbreviation)
    {"rust",     {{ 0.20f, 0.07f, 0.06f }, 0.20f, { 0.250f, 0.190f, 0.080f }}},  // → corroded metal (rougher)
    {"iron",     {{ 0.20f, 0.07f, 0.06f }, 0.15f, { 0.250f, 0.190f, 0.080f }}},  // → wrought iron (moderate)
    {"door",     {{ 0.11f, 0.07f, 0.06f }, 0.15f, { 0.070f, 0.014f, 0.005f }}},  // → wood door with panels
    {"gate",     {{ 0.20f, 0.07f, 0.06f }, 0.35f, { 0.250f, 0.190f, 0.080f }}},  // → metal gate (irregular)
    {"roof",     {{ 0.01f, 0.02f, 0.02f }, 0.10f, { 0.060f, 0.044f, 0.011f }}},  // → roof tile (slightly rough)
    {"vine",     {{ 0.11f, 0.07f, 0.06f }, 0.40f, { 0.070f, 0.014f, 0.005f }}},  // → organic (very irregular)
    {"leaf",     {{ 0.11f, 0.07f, 0.06f }, 0.40f, { 0.070f, 0.014f, 0.005f }}},  // → organic (very irregular)
    {"bark",     {{ 0.11f, 0.07f, 0.06f }, 0.30f, { 0.070f, 0.014f, 0.005f }}},  // → rough bark
    {"rug",      {{ 0.24f, 0.69f, 0.73f }, 0.10f, { 0.020f, 0.005f, 0.003f }}},  // → flat textile
    {"hay",      {{ 0.24f, 0.69f, 0.73f }, 0.50f, { 0.020f, 0.005f, 0.003f }}},  // → loose organic (very irregular)
    {"mud",      {{ 0.60f, 0.70f, 0.80f }, 0.50f, { 0.031f, 0.012f, 0.008f }}},  // → wet irregular surface
};

// Default material for unmatched textures — moderate scattering as recommended
// by Treble Technologies (minimum 0.10 to prevent ray trapping).
static const IPLMaterial kGenericMaterial =
    {{ 0.10f, 0.20f, 0.30f }, 0.20f, { 0.100f, 0.050f, 0.030f }};

/// Look up an IPLMaterial by texture name via keyword substring matching.
/// Uses the shared keyword table from AcousticMaterials.h for matching,
/// then maps the keyword to IPLMaterial properties.
/// Logs unmatched textures once per name for development diagnostics.
static IPLMaterial lookupAcousticMaterial(const std::string &texName)
{
    // Check for exact texture name match first (e.g., "_portal" sentinel).
    // This allows special internal names to bypass keyword substring matching.
    auto exact = kKeywordToIPLMaterial.find(texName);
    if (exact != kKeywordToIPLMaterial.end())
        return exact->second;

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

    // Portal-based sound propagation (alternative to direct line-of-sight).
    // When a sound is occluded but reachable through connected rooms (doorways,
    // corridors), the portal path provides attenuation and direction from the
    // last portal the sound passed through.
    bool usePortalRouting = false;
    bool skipAttenuation = false;        // true for player-emitted sounds (footsteps, within 5 units)
    float portalAttenuation = 0.0f;      // inverse-square from effective distance
    IPLVector3 portalDirection{1.0f, 0.0f, 0.0f}; // direction toward virtual source (portal center)

    // Reflection convolution effect (per-voice, feeds into shared mixer)
    IPLReflectionEffect reflectionEffect = nullptr;

    // Shared reference to the reflection mixer (NOT owned — AudioService manages)
    IPLReflectionMixer reflectionMixer = nullptr;

    // Reflection simulation output params (written by main thread from simulator)
    IPLReflectionEffectParams reflectionParams{};
    std::atomic<bool> reflectionsActive{false};

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
    std::atomic<bool> effectsReady{false};   // true when effects + node are initialized
    bool nodeInitialized = false;            // true when ma_node_init succeeded (main thread only)

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

    // Reflection gain ramp — smoothly fades in reflection output to prevent
    // crackling on initial activation. Ramps from 0→1 over ~10ms (480 samples
    // at 48kHz). Once at 1.0, stays there (subsequent IR updates are handled
    // by Steam Audio's internal crossfading).
    float reflGain = 0.0f;           // current gain (0=silent, 1=full)
    float reflGainTarget = 1.0f;     // target gain
    static constexpr float kReflRampRate = 1.0f / 480.0f; // per-sample increment (~10ms at 48kHz)
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

// ── Debug audio recording ──
// Captures the final stereo output to a WAV file for offline analysis.
// Toggled via the debug console (record_audio setting).
static std::atomic<bool> sRecording{false};
static FILE *sRecordFile = nullptr;
static uint32_t sRecordSamples = 0;  // total stereo samples written
static std::mutex sRecordMutex;

static void startRecording(uint32_t sampleRate) {
    std::lock_guard<std::mutex> lock(sRecordMutex);
    if (sRecordFile) return;

    std::string path = "darkness_audio_capture.wav";
    sRecordFile = std::fopen(path.c_str(), "wb");
    if (!sRecordFile) {
        std::fprintf(stderr, "[RECORD] Failed to open %s\n", path.c_str());
        return;
    }

    // Write a placeholder WAV header (44 bytes) — updated on close
    uint8_t header[44] = {};
    std::memcpy(header, "RIFF", 4);
    std::memcpy(header + 8, "WAVE", 4);
    std::memcpy(header + 12, "fmt ", 4);
    uint32_t fmtSize = 16;
    std::memcpy(header + 16, &fmtSize, 4);
    uint16_t audioFmt = 3;  // IEEE float
    std::memcpy(header + 20, &audioFmt, 2);
    uint16_t channels = 2;
    std::memcpy(header + 22, &channels, 2);
    std::memcpy(header + 24, &sampleRate, 4);
    uint32_t byteRate = sampleRate * 2 * 4;  // stereo float32
    std::memcpy(header + 28, &byteRate, 4);
    uint16_t blockAlign = 8;  // 2 channels × 4 bytes
    std::memcpy(header + 32, &blockAlign, 2);
    uint16_t bitsPerSample = 32;
    std::memcpy(header + 34, &bitsPerSample, 2);
    std::memcpy(header + 36, "data", 4);
    // data size placeholder (updated on close)
    std::fwrite(header, 1, 44, sRecordFile);
    sRecordSamples = 0;

    // Also start a position log
    FILE *posFile = std::fopen("darkness_audio_positions.csv", "w");
    if (posFile) {
        std::fprintf(posFile, "timestamp_ms,x,y,z,yaw,pitch,voices,refl_voices\n");
        std::fclose(posFile);
    }

    std::fprintf(stderr, "[RECORD] Started: %s (48kHz stereo float32)\n", path.c_str());
    sRecording.store(true, std::memory_order_release);
}

static void stopRecording() {
    std::lock_guard<std::mutex> lock(sRecordMutex);
    sRecording.store(false, std::memory_order_release);
    if (!sRecordFile) return;

    // Fix up WAV header with actual sizes
    uint32_t dataSize = sRecordSamples * 2 * 4;  // stereo float32
    uint32_t riffSize = 36 + dataSize;
    std::fseek(sRecordFile, 4, SEEK_SET);
    std::fwrite(&riffSize, 4, 1, sRecordFile);
    std::fseek(sRecordFile, 40, SEEK_SET);
    std::fwrite(&dataSize, 4, 1, sRecordFile);
    std::fclose(sRecordFile);
    sRecordFile = nullptr;

    float durSec = sRecordSamples / 48000.0f;
    std::fprintf(stderr, "[RECORD] Stopped: %u samples (%.1f seconds)\n",
                 sRecordSamples, durSec);
}

// Called from the audio thread (reflectionMixNodeProcess) to capture output
static void recordAudioFrame(const float *stereoInterleaved, uint32_t frameCount) {
    if (!sRecording.load(std::memory_order_relaxed)) return;
    std::lock_guard<std::mutex> lock(sRecordMutex);
    if (!sRecordFile) return;
    std::fwrite(stereoInterleaved, sizeof(float), frameCount * 2, sRecordFile);
    sRecordSamples += frameCount;
}

// ── Audio thread profiling (written by audio thread, read by main thread) ──
// Tracks peak time spent in audio callbacks to detect buffer underruns.
static std::atomic<float> sPerVoicePeakUs{0.0f};    // peak per-voice DSP time (µs)
static std::atomic<float> sMixNodePeakUs{0.0f};      // peak global mix node time (µs)
static std::atomic<float> sTotalCallbackPeakUs{0.0f}; // peak total audio callback time (µs)
static std::atomic<int>   sPerVoiceCallCount{0};      // voices processed in last period
static std::atomic<float> sCommitPeakMs{0.0f};         // peak iplSimulatorCommit time (ms)

// ── Main thread + sim worker profiling ──
static std::atomic<float> sLoopStepPeakMs{0.0f};      // peak loopStep total time (ms)
static std::atomic<float> sDirectSimPeakMs{0.0f};     // peak iplSimulatorRunDirect time (ms)
static std::atomic<float> sReflSimPeakMs{0.0f};       // peak iplSimulatorRunReflections time (ms)
static std::atomic<int>   sPortalRoutingTotalUs{0};     // accumulated portal routing time per dump (µs, int)
static std::atomic<int>   sPortalRoutingCount{0};      // portal routing calls per dump
static std::atomic<int>   sVoicesCreated{0};           // voices started since last dump
static std::atomic<int>   sVoicesDestroyed{0};         // voices cleaned up since last dump
static std::atomic<int>   sReflFramesRun{0};           // reflection sim steps since last dump

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

    if (!node->effectsReady.load(std::memory_order_acquire) || frameCount == 0) {
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

        // Apply attenuation from the best available propagation path.
        // Three paths are considered (highest energy wins):
        //   1. Direct line-of-sight (Steam Audio: distance × occlusion × air)
        //   2. Portal routing (through doorways/corridors, effective distance)
        //   3. Transmission through walls (part of direct sim, very quiet)
        if (runAtten && !node->skipAttenuation) {
            float airAvg = (node->directParams.airAbsorption[0]
                          + node->directParams.airAbsorption[1]
                          + node->directParams.airAbsorption[2]) / 3.0f;

            // Apply wall transmission — when a sound is behind geometry, Steam
            // Audio computes how much energy passes through the intervening
            // material (frequency-dependent). The listener hears the unoccluded
            // portion (occlusion) plus what leaks through (transmission).
            // When fully occluded (occlusion=0), only transmission remains.
            // When not occluded (occlusion=1), transmission has no effect.
            float transAvg = (node->directParams.transmission[0]
                            + node->directParams.transmission[1]
                            + node->directParams.transmission[2]) / 3.0f;
            float directAtten = node->directParams.distanceAttenuation
                              * (node->directParams.occlusion + transAvg * (1.0f - node->directParams.occlusion))
                              * airAvg;

            // Architecture B routing:
            // - Cross-room voices: portal attenuation × last-segment occlusion.
            //   The IPLSource was positioned at the virtual position (doorway),
            //   so directAtten reflects occlusion from doorway to listener.
            // - Same-room / no-room voices: Steam Audio direct path only.
            //   Wall occlusion is trusted — if Steam Audio says there's a wall,
            //   the sound is muffled regardless of portal graph reachability.
            //   (The portal floor was previously overriding legitimate wall
            //   occlusion for sounds behind walls that happened to be portal-
            //   reachable through a long doorway chain.)
            float atten = directAtten;
            if (node->usePortalRouting) {
                atten = node->portalAttenuation * directAtten;
            }

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
            // Feed the UNATTENUATED mono signal into the convolution.
            // The reflection IR encodes the energy from all reflected paths
            // (wall bounces, ceiling reflections, around-corner diffraction).
            // These paths are independent of the direct-path occlusion — a lamp
            // behind a wall has its direct path blocked, but its reflected energy
            // (bouncing off nearby walls and through doorways) should still be
            // audible. Pre-attenuating the convolution input by the direct-path
            // occlusion would silence the reflected paths too.
            //
            // Distance attenuation is still relevant (a distant source produces
            // less reflected energy), so we apply distanceAttenuation only,
            // without occlusion/transmission.
            float reflAtten = node->directParams.distanceAttenuation;
            if (reflAtten < 1.0f) {
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
    // Apply a gain ramp to prevent crackling when the IR updates. The gain
    // smoothly ramps from 0→1 over ~10ms on first activation and after each
    // simulation update. This eliminates discontinuities at IR transitions.
    // In half-rate mode, upsample the decoded stereo from reflFrameCount
    // to frameCount using linear interpolation (fine for diffuse reverb).
    if (node->halfRate) {
        ma_uint32 outSamples = std::min(reflFrameCount * 2, frameCount);
        ma_uint32 pairs = outSamples / 2;
        for (ma_uint32 i = 0; i < pairs; ++i) {
            // Ramp gain toward target (2 output samples per iteration)
            if (node->reflGain < node->reflGainTarget)
                node->reflGain = std::min(node->reflGain + node->kReflRampRate * 2.0f, node->reflGainTarget);
            float g = node->reflGain;

            float l0 = decodedChannels[0][i] * g;
            float r0 = decodedChannels[1][i] * g;
            float l1 = ((i + 1 < reflFrameCount) ? decodedChannels[0][i + 1] : decodedChannels[0][i]) * g;
            float r1 = ((i + 1 < reflFrameCount) ? decodedChannels[1][i + 1] : decodedChannels[1][i]) * g;
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
            if (node->reflGain < node->reflGainTarget)
                node->reflGain = std::min(node->reflGain + node->kReflRampRate, node->reflGainTarget);
            float g = node->reflGain;
            stereoOut[i * 2]     += decodedChannels[0][i] * g;
            stereoOut[i * 2 + 1] += decodedChannels[1][i] * g;
        }
    }

    // Clamp output to [-1, 1] — HRTF convolution can produce samples > 1.0
    // (confirmed by Steam Audio issue #350). Without clamping, the DAC clips
    // harshly, producing audible click artifacts.
    for (ma_uint32 i = 0; i < frameCount * 2; ++i) {
        if (stereoOut[i] > 1.0f) stereoOut[i] = 1.0f;
        else if (stereoOut[i] < -1.0f) stereoOut[i] = -1.0f;
    }

    pFrameCountIn[0] = frameCount;
    *pFrameCountOut = frameCount;

    // Capture final output for debug recording (if active)
    if (stereoOut && frameCount > 0)
        recordAudioFrame(stereoOut, frameCount);

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
    std::atomic<bool> sourceEnded{false};  // true after ma_sound reaches end (set by audio thread)
    float tailTimer = 0.0f;     // seconds remaining for reverb tail

    // Voice management metadata
    std::string schemaName;            // Schema that spawned this voice
    int priority = 128;                // 0-255, higher = more important
    int objID = 0;                     // Object ID if attached (0 = positional)
    bool playerEmitted = false;        // true for footsteps/landing — skip DSP attenuation

    // Re-propagation throttle: nearby sounds update every frame, distant
    // sounds every 8-16 frames. Matching the original engine's adaptive
    // update frequency (FramesUntilUpdate = dist/10, clamped to [1,16]).
    int propagationCountdown = 0;      // frames until next propagateSound call
    SoundPropInfo cachedProp{};        // last propagation result (reused between updates)

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
        dspNode.effectsReady.store(false, std::memory_order_release);
        dspNode.reflectionsActive.store(false, std::memory_order_release);

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
        voice->sourceEnded.store(true, std::memory_order_release);
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
                // Apply absorption scale — lower values make surfaces more
                // reflective (richer reverb, more echo). 0.5 = half absorption.
                if (mAbsorptionScale != 1.0f) {
                    for (int b = 0; b < 3; ++b)
                        mat.absorption[b] = std::min(1.0f,
                            mat.absorption[b] * mAbsorptionScale);
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

        // Step 3: Convert vertex data to IPLVector3 array and compute scene bounds
        // (IPLVector3 is {float x, y, z} — same layout as our flat array)
        mSceneMin = Vector3( 1e9f,  1e9f,  1e9f);
        mSceneMax = Vector3(-1e9f, -1e9f, -1e9f);
        std::vector<IPLVector3> iplVertices(numVertices);
        for (size_t i = 0; i < numVertices; ++i) {
            float x = data.vertices[i * 3];
            float y = data.vertices[i * 3 + 1];
            float z = data.vertices[i * 3 + 2];
            iplVertices[i] = {x, y, z};
            mSceneMin = glm::min(mSceneMin, Vector3(x, y, z));
            mSceneMax = glm::max(mSceneMax, Vector3(x, y, z));
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
        simSettings.numDiffuseSamples = mDiffuseSamples;
        simSettings.maxDuration = mReflectionDuration;
        simSettings.maxOrder = mAmbisonicsOrder;
        simSettings.maxNumSources = 32;      // voice pool size
        // Use most available cores for parallel ray tracing.
        // Reserve 2 cores for main + audio threads.
        unsigned int hwThreads = std::thread::hardware_concurrency();
        simSettings.numThreads = std::max(2u, hwThreads > 2 ? hwThreads - 2 : 2u);
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
    // Wait for any in-flight simulation tasks to finish on the worker thread
    waitForReflectionThread();

    // Flush deferred IPL source adds — never added to simulator, just release
    for (auto &src : mPendingSourceAdds) {
        iplSourceRelease(&src);
    }
    mPendingSourceAdds.clear();

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

    // Release probe batch before simulator (it's registered with the simulator)
    if (mIplProbeBatch) {
        if (mIplSimulator)
            iplSimulatorRemoveProbeBatch(mIplSimulator, mIplProbeBatch);
        iplProbeBatchRelease(&mIplProbeBatch);
        mIplProbeBatch = nullptr;
        mProbeCount = 0;
    }

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

    // Start the persistent simulation worker thread
    if (mAudioReady) {
        mSimWorkerShutdown.store(false, std::memory_order_relaxed);
        mSimWorkerThread = std::thread(&AudioService::simWorkerMain, this);
    }

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

    // Shut down the simulation worker thread before destroying the scene
    mSimWorkerShutdown.store(true, std::memory_order_release);
    mSimWorkerCV.notify_one();
    if (mSimWorkerThread.joinable())
        mSimWorkerThread.join();

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
    mListenerRoom = nullptr;
    mRoomTransmission.clear();
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

    auto loopStepStart = std::chrono::steady_clock::now();

    // Compute portal blend state for this frame (once, reused by all propagation calls)
    computePortalBlend();

    // Source mutations (add/remove/commit) must not happen while the background
    // simulation thread is running — iplSimulatorRunDirect/Reflections holds
    // internal pointers to committed sources. Instead of BLOCKING the main thread
    // (which caused 200+ms stalls), we DEFER mutations until the sim finishes.
    // Source mutations (add/remove/commit) require the direct sim thread to be idle.
    // The reflection sim can run concurrently with source input updates because
    // Steam Audio uses double-buffering: setInputs writes to the staging buffer,
    // while runReflections reads from the committed (active) buffer.
    // Only commit() copies staging → active, so commit must wait for both threads.
    bool directBusy = mDirectSimRunning.load(std::memory_order_acquire);
    bool reflBusy = mReflectionSimRunning.load(std::memory_order_acquire);

    // Source mutations (add/remove) need both sim tasks idle because they modify
    // the source list that both sim tasks reference.
    // Commit (staging→active copy) needs both idle because RunReflections reads active.
    bool canMutate = !directBusy && !reflBusy;

    if (canMutate) {

        // Flush deferred IPL source adds
        if (!mPendingSourceAdds.empty() && mIplSimulator) {
            for (auto &src : mPendingSourceAdds) {
                iplSourceAdd(src, mIplSimulator);
            }
            mPendingSourceAdds.clear();
            mSimulatorDirty = true;
        }

        // Flush deferred IPL source removals
        if (!mPendingSourceRemovals.empty() && mIplSimulator) {
            for (auto &src : mPendingSourceRemovals) {
                iplSourceRemove(src, mIplSimulator);
                iplSourceRelease(&src);
            }
            mPendingSourceRemovals.clear();
        }
    }

    // Commit can happen whenever BOTH threads are idle — the most common window
    // is right after the direct sim finishes (every frame) and before the next
    // reflection sim launches (every Nth frame). If the reflection sim takes
    // longer than the throttle interval, commits are deferred, but the fallback
    // attenuation above ensures voices are never silent while waiting.
    if (canMutate && mSimulatorDirty && mIplSimulator) {
        auto ct0 = std::chrono::steady_clock::now();
        iplSimulatorCommit(mIplSimulator);
        auto ct1 = std::chrono::steady_clock::now();
        float cMs = std::chrono::duration<float, std::milli>(ct1 - ct0).count();
        float prevC = sCommitPeakMs.load(std::memory_order_relaxed);
        if (cMs > prevC) sCommitPeakMs.store(cMs, std::memory_order_relaxed);
        mSimulatorDirty = false;
    }

    // Always clean up finished voices (defers IPL removal if sim busy)
    cleanupFinishedVoices();

    // Run Steam Audio simulation for all active sources
    if (mSceneReady && mIplSimulator && !mVoices.empty()) {
        float cosY = std::cos(mListenerYaw), sinY = std::sin(mListenerYaw);
        float cosP = std::cos(mListenerPitch), sinP = std::sin(mListenerPitch);

        // Listener coordinate frame (used for both IPL and HRTF direction)
        Vector3 right(sinY, -cosY, 0.0f);
        Vector3 ahead(cosY * cosP, sinY * cosP, sinP);
        Vector3 up(-cosY * sinP, -sinY * sinP, cosP);

        IPLCoordinateSpace3 listenerCoord{};
        listenerCoord.origin = {mListenerPos.x, mListenerPos.y, mListenerPos.z};
        listenerCoord.ahead  = {ahead.x, ahead.y, ahead.z};
        listenerCoord.right  = {right.x, right.y, right.z};
        listenerCoord.up     = {up.x, up.y, up.z};

        // Update listener orientation for ambisonics decode (audio thread reads this)
        if (mReflectionMixNode && mReflectionMixNode->ready) {
            mReflectionMixNode->listenerOrientation = listenerCoord;
        }

        // Set simulation inputs and launch direct sim.
        // setInputs/setSharedInputs write to the staging buffer (safe while
        // reflection sim reads from the committed buffer). Only need direct
        // sim thread to be idle — reflection sim can run concurrently.
        // Reflection voice ranking — declared here so it's visible to both
        // the setInputs block (Step 2) and the output reading block (Step 4).
        struct VoiceDist {
            SoundHandle handle;
            float distSq;
        };
        std::vector<VoiceDist> reflCandidates;
        std::unordered_set<SoundHandle> reflCandidateSet;

        if (!directBusy) {
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

            // Step 2a: Pre-compute reflection voice ranking (top-N closest).
            // Done before setInputs so we can set inputs.baked for non-top-N voices.
            if (mReflectionsEnabled) {
                reflCandidates.reserve(mVoices.size());
                for (auto &[h, v] : mVoices) {
                    if (v->sourceEnded)
                        continue;
                    if (v->iplSource && v->dspNode.effectsReady && v->dspNode.reflectionEffect) {
                        Vector3 delta = v->worldPos - mListenerPos;
                        reflCandidates.push_back({h, glm::dot(delta, delta)});
                    }
                }
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

            // Build a set for O(1) lookup of top-N membership
            for (const auto &rc : reflCandidates)
                reflCandidateSet.insert(rc.handle);

            // Baked reflection identifier (for non-top-N voices using probe reverb)
            IPLBakedDataIdentifier bakedReflId{};
            bakedReflId.type = IPL_BAKEDDATATYPE_REFLECTIONS;
            bakedReflId.variation = IPL_BAKEDDATAVARIATION_REVERB;

            // Step 2b: Run portal propagation and set source positions.
            // Architecture B: for cross-room voices, set the IPLSource position
            // to the virtual position (last portal anchor) so Steam Audio traces
            // rays from the doorway, not from behind a wall. Same-room voices
            // use their real position. Unreachable voices skip simulation.
            // Use the portal blend's primary room for stable cross-room detection.
            // When blending is active, roomFromPoint may flicker between rooms,
            // but mPortalBlend.roomA is always the primary room for this frame.
            // Update cached listener room incrementally:
            // 1. If portal blend is active, use the blend's primary room (stable)
            // 2. Otherwise, check if still inside the cached room (fast, 6 plane tests)
            // 3. If not, check adjacent rooms via portal adjacency (fast, ~4 rooms)
            // 4. Only fall back to full brute-force scan if adjacency fails
            if (mPortalBlend.active) {
                mListenerRoom = mPortalBlend.roomA;
            } else if (mListenerRoom && mListenerRoom->isInside(mListenerPos)) {
                // Still in the same room — no change needed
            } else if (mListenerRoom) {
                // Moved out of cached room — check adjacent rooms first
                bool found = false;
                for (uint32_t i = 0; i < mListenerRoom->getPortalCount(); ++i) {
                    RoomPortal *portal = mListenerRoom->getPortal(i);
                    if (portal && portal->getFarRoom() &&
                        portal->getFarRoom()->isInside(mListenerPos)) {
                        mListenerRoom = portal->getFarRoom();
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    // Adjacent rooms didn't match — full scan
                    mListenerRoom = mRoomService ? mRoomService->roomFromPoint(mListenerPos) : nullptr;
                }
            } else {
                // No cached room — full scan
                mListenerRoom = mRoomService ? mRoomService->roomFromPoint(mListenerPos) : nullptr;
            }
            Room *listenerRoom = mListenerRoom;

            for (auto &[handle, voice] : mVoices) {
                if (!voice->iplSource)
                    continue;

                // Player-emitted sounds (footsteps, landing) bypass DSP
                // attenuation — the player's own sounds are never wall-occluded.
                voice->dspNode.skipAttenuation = voice->playerEmitted;

                // Run portal propagation to determine reachability and path.
                // Throttled: nearby voices update every frame, distant voices
                // every 8-16 frames. Matching the original engine's adaptive
                // update frequency. Uses cached result between updates.
                SoundPropInfo prop{};
                bool isCrossRoom = false;
                if (mPortalRoutingEnabled && !voice->sourceEnded && !voice->playerEmitted) {
                    if (voice->propagationCountdown <= 0) {
                        auto prT0 = std::chrono::steady_clock::now();
                        prop = propagateSoundBlended(voice->worldPos);
                        auto prT1 = std::chrono::steady_clock::now();
                        float prUs = std::chrono::duration<float, std::micro>(prT1 - prT0).count();
                        sPortalRoutingTotalUs.fetch_add(static_cast<int>(prUs), std::memory_order_relaxed);
                        sPortalRoutingCount.fetch_add(1, std::memory_order_relaxed);

                        voice->cachedProp = prop;

                        // Set next update interval based on distance.
                        // 0 = update every frame (very close). 16 = every 16th frame (~267ms).
                        // Matches original engine: FramesUntilUpdate = dist/10, clamped [0,16].
                        float dist = prop.reached ? prop.effectiveDistance : 200.0f;
                        voice->propagationCountdown = std::max(0, std::min(16,
                            static_cast<int>(dist / 10.0f)));
                    } else {
                        // Use cached result from previous propagation
                        prop = voice->cachedProp;
                        voice->propagationCountdown--;
                    }

                    // Determine if the voice is in a different room from the listener.
                    // Minimum distance threshold: sounds very close to the listener
                    // (footsteps, player-emitted sounds) are always same-room even if
                    // roomFromPoint returns different rooms at floor/ceiling boundaries.
                    Room *srcRoom = mRoomService ? mRoomService->roomFromPoint(voice->worldPos) : nullptr;
                    float srcListenerDist = glm::length(voice->worldPos - mListenerPos);
                    isCrossRoom = (srcRoom && listenerRoom && srcRoom != listenerRoom
                                   && srcListenerDist > 5.0f);
                }

                // Store propagation result on DSP node for the audio callback
                if (prop.reached) {
                    float portalAtten = 1.0f / (1.0f + prop.effectiveDistance *
                                                        prop.effectiveDistance * 0.001f);
                    voice->dspNode.portalAttenuation = portalAtten;
                    voice->dspNode.usePortalRouting = isCrossRoom;

                    // For cross-room voices, compute HRTF direction toward the
                    // virtual position (last portal anchor / doorway edge)
                    if (isCrossRoom) {
                        Vector3 toPortal = prop.virtualPosition - mListenerPos;
                        float portalDist = glm::length(toPortal);
                        if (portalDist > 0.001f) {
                            toPortal /= portalDist;
                            voice->dspNode.direction = {
                                glm::dot(toPortal, right),
                                glm::dot(toPortal, up),
                                -glm::dot(toPortal, ahead)
                            };
                        }
                    }
                } else {
                    voice->dspNode.usePortalRouting = false;
                    voice->dspNode.portalAttenuation = 0.0f;
                }

                // Architecture B: for cross-room voices, set the IPLSource
                // position to the virtual position (last portal anchor).
                // Steam Audio traces direct-path rays from the doorway, giving
                // correct frequency-dependent occlusion for the final visible
                // segment. Same-room voices use their real position for full
                // same-room occlusion (pillars, furniture, etc.).
                IPLCoordinateSpace3 sourceCoord{};
                if (isCrossRoom && prop.reached) {
                    sourceCoord.origin = {prop.virtualPosition.x,
                                          prop.virtualPosition.y,
                                          prop.virtualPosition.z};
                } else {
                    sourceCoord.origin = {voice->worldPos.x,
                                          voice->worldPos.y,
                                          voice->worldPos.z};
                }
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

                // Non-top-N voices use baked probe reflections instead of
                // real-time ray tracing.
                bool isTopN = reflCandidateSet.count(handle) > 0;
                if (!isTopN && mProbesHaveReflections) {
                    inputs.baked = IPL_TRUE;
                    inputs.bakedDataIdentifier = bakedReflId;
                }

                iplSourceSetInputs(voice->iplSource,
                    static_cast<IPLSimulationFlags>(
                        IPL_SIMULATIONFLAGS_DIRECT | IPL_SIMULATIONFLAGS_REFLECTIONS),
                    &inputs);
            }

            // Step 3: Signal the persistent sim worker thread.
            // Direct sim runs every frame, reflection sim runs throttled.
            bool wantDirect = true;
            bool wantReflections = mReflectionsEnabled && mIplReflectionMixer
                && !reflBusy
                && (++mReflectionFrameCounter >= mReflectionThrottle);
            if (wantReflections)
                mReflectionFrameCounter = 0;

            // Set running flags BEFORE signaling (worker clears them on completion)
            mDirectSimRunning.store(true, std::memory_order_release);
            if (wantReflections)
                mReflectionSimRunning.store(true, std::memory_order_release);

            {
                std::lock_guard<std::mutex> lock(mSimWorkerMutex);
                mSimWorkerWantDirect = wantDirect;
                if (wantReflections)
                    mSimWorkerWantReflections = true;
            }
            mSimWorkerCV.notify_one();
        }

        // Step 4: Read back simulation results and feed to DSP nodes.
        // Results are from the previous frame's simulation (1-frame latency) since
        // we moved both direct and reflection sim to the background thread.
        // This is imperceptible for both attenuation and reverb changes.
        // Steam Audio's triple buffer ensures iplSourceGetOutputs returns the
        // latest completed results without blocking.

        // reflCandidates and reflCandidateSet were already computed in Step 2a
        // (before iplSourceSetInputs) so non-top-N voices could be set to baked mode.

        // Track total convolution voices against the audio callback budget.
        // Tail voices (sourceEnded, still ringing out reverb) are counted first
        // since they already have reflectionsActive=true and can't be cheaply
        // disabled mid-convolution. Remaining slots go to active voices: top-N
        // closest first, then baked voices fill remaining up to mMaxReflectionVoices.
        int tailConvolutionCount = 0;
        for (auto &[h, v] : mVoices) {
            if (v->sourceEnded && !v->finished.load(std::memory_order_relaxed)
                && v->dspNode.reflectionsActive.load(std::memory_order_relaxed)) {
                ++tailConvolutionCount;
            }
        }
        int activeConvolutionCount = tailConvolutionCount;

        for (auto &[handle, voice] : mVoices) {
            if (!voice->iplSource)
                continue;

            IPLSimulationOutputs outputs{};
            IPLSimulationFlags getFlags = static_cast<IPLSimulationFlags>(
                IPL_SIMULATIONFLAGS_DIRECT |
                (mReflectionsEnabled ? IPL_SIMULATIONFLAGS_REFLECTIONS : 0));
            iplSourceGetOutputs(voice->iplSource, getFlags, &outputs);

            if (voice->dspNode.effectsReady) {
                // Copy simulation outputs to DSP node params. If the simulator
                // hasn't processed this source yet (zero attenuation = uncommitted),
                // compute a distance-based fallback so the voice is never silent
                // while waiting for the sim to catch up.
                if (outputs.direct.distanceAttenuation > 0.0f) {
                    voice->dspNode.directParams = outputs.direct;
                    voice->dspNode.directParams.flags = static_cast<IPLDirectEffectFlags>(
                        IPL_DIRECTEFFECTFLAGS_APPLYDISTANCEATTENUATION |
                        IPL_DIRECTEFFECTFLAGS_APPLYAIRABSORPTION |
                        IPL_DIRECTEFFECTFLAGS_APPLYOCCLUSION |
                        IPL_DIRECTEFFECTFLAGS_APPLYTRANSMISSION);
                    voice->dspNode.directParams.transmissionType =
                        IPL_TRANSMISSIONTYPE_FREQDEPENDENT;
                } else {
                    // Sim hasn't committed this source yet — use distance fallback
                    // so the voice plays at a reasonable volume immediately.
                    float dist = glm::length(voice->worldPos - mListenerPos);
                    float fallbackAtten = 1.0f / (1.0f + dist * dist * 0.001f);
                    voice->dspNode.directParams.distanceAttenuation = fallbackAtten;
                    voice->dspNode.directParams.occlusion = 1.0f;
                    voice->dspNode.directParams.airAbsorption[0] = 1.0f;
                    voice->dspNode.directParams.airAbsorption[1] = 1.0f;
                    voice->dspNode.directParams.airAbsorption[2] = 1.0f;
                }

                // Enable reflection convolution up to the global budget.
                // Top-N closest voices (real-time) get priority, then baked
                // voices fill remaining slots. Total convolution voices are
                // capped at mMaxReflectionVoices to stay within the audio
                // callback budget (~1.5ms per convolution voice).
                bool isReflVoice = reflCandidateSet.count(handle) > 0;
                bool canAffordConvolution = (activeConvolutionCount < mMaxReflectionVoices);
                bool hasBakedData = mProbesHaveReflections
                                    && outputs.reflections.irSize > 0
                                    && voice->dspNode.reflectionEffect;
                bool enableRefl = isReflVoice || (hasBakedData && canAffordConvolution);

                if (enableRefl) {
                    voice->dspNode.reflectionParams = outputs.reflections;
                    voice->dspNode.reflectionParams.type =
                        IPL_REFLECTIONEFFECTTYPE_CONVOLUTION;
                    voice->dspNode.reflectionParams.numChannels = mAmbisonicsChannels;

                    // Per-source LOD: reduce IR length for distant voices.
                    // Nearby voices get full IR, distant voices get shorter tails.
                    // This reduces convolution cost proportionally to distance.
                    float voiceDist = glm::length(voice->worldPos - mListenerPos);
                    int maxIrSize = voice->dspNode.reflectionParams.irSize;
                    if (maxIrSize > 0 && voiceDist > 20.0f) {
                        // Linear ramp: full IR at 20 units, quarter IR at 140+ units
                        float lodScale = std::max(0.25f, 1.0f - (voiceDist - 20.0f) / 120.0f);
                        int lodIrSize = static_cast<int>(maxIrSize * lodScale);
                        lodIrSize = (lodIrSize / static_cast<int>(mReflectionFrameSize))
                                  * static_cast<int>(mReflectionFrameSize);
                        if (lodIrSize < static_cast<int>(mReflectionFrameSize))
                            lodIrSize = static_cast<int>(mReflectionFrameSize);
                        voice->dspNode.reflectionParams.irSize = lodIrSize;
                    }

                    voice->dspNode.reflectionsActive.store(true, std::memory_order_release);
                    ++activeConvolutionCount;
                } else {
                    voice->dspNode.reflectionsActive.store(false, std::memory_order_relaxed);
                }

                // Compute listener-to-source direction in listener's local frame
                // for HRTF binaural rendering. Cross-room voices already had
                // their direction set toward the portal in Step 2b — skip them.
                if (!voice->dspNode.usePortalRouting) {
                Vector3 toSource = voice->worldPos - mListenerPos;
                float dist = glm::length(toSource);
                if (dist > 0.001f) {
                    toSource /= dist;
                    // Project world direction onto listener-local axes.
                    // Steam Audio convention: +X=right, +Y=up, -Z=ahead,
                    // so negate the ahead component for correct front/back.
                    voice->dspNode.direction = {
                        glm::dot(toSource, right),    // x = right
                        glm::dot(toSource, up),       // y = up
                        -glm::dot(toSource, ahead)    // z = -ahead (Steam Audio: -Z = forward)
                    };
                }
                } // end !usePortalRouting direction update

                // Portal routing is now done in Step 2b (before iplSourceSetInputs).
                // The HRTF direction and portalAttenuation are already set on
                // the DSP node. Cross-room voices also have their IPLSource
                // position overridden to the virtual position.
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
                if (voice->dspNode.reflectionEffect &&
                    voice->dspNode.reflectionsActive.load(std::memory_order_relaxed)) {
                    // Only start a reverb tail if this voice was actually running
                    // convolution (was in the top-N or had a baked slot). Voices
                    // that were dry don't need a tail — there's no convolution
                    // state to ring out.
                    voice->tailTimer = mReflectionDuration;
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

    // loopStep total time (main thread)
    {
        auto loopStepEnd = std::chrono::steady_clock::now();
        float ms = std::chrono::duration<float, std::milli>(loopStepEnd - loopStepStart).count();
        float prev = sLoopStepPeakMs.load(std::memory_order_relaxed);
        if (ms > prev) sLoopStepPeakMs.store(ms, std::memory_order_relaxed);
    }

    // Write position data to CSV if recording is active
    if (sRecording.load(std::memory_order_relaxed)) {
        static auto recordStart = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        float ms = std::chrono::duration<float, std::milli>(now - recordStart).count();
        int reflCount = 0;
        for (auto &[h, v] : mVoices)
            if (v->dspNode.reflectionsActive) ++reflCount;
        FILE *posFile = std::fopen("darkness_audio_positions.csv", "a");
        if (posFile) {
            std::fprintf(posFile, "%.1f,%.2f,%.2f,%.2f,%.4f,%.4f,%zu,%d\n",
                         ms, mListenerPos.x, mListenerPos.y, mListenerPos.z,
                         mListenerYaw, mListenerPitch,
                         mVoices.size(), reflCount);
            std::fclose(posFile);
        }
    }
}

//------------------------------------------------------
void AudioService::simWorkerMain()
{
    // Persistent worker thread for Steam Audio simulation.
    // Waits on a condition variable for work requests, avoiding the overhead
    // of creating/destroying a std::thread every frame (60 pthread_create/join
    // cycles per second).
    while (true) {
        bool doDirect = false;
        bool doReflections = false;

        {
            std::unique_lock<std::mutex> lock(mSimWorkerMutex);
            mSimWorkerCV.wait(lock, [this] {
                return mSimWorkerWantDirect || mSimWorkerWantReflections
                       || mSimWorkerShutdown.load(std::memory_order_relaxed);
            });

            if (mSimWorkerShutdown.load(std::memory_order_relaxed))
                break;

            doDirect = mSimWorkerWantDirect;
            doReflections = mSimWorkerWantReflections;
            mSimWorkerWantDirect = false;
            mSimWorkerWantReflections = false;
        }

        if (doDirect) {
            auto t0 = std::chrono::steady_clock::now();
            iplSimulatorRunDirect(mIplSimulator);
            auto t1 = std::chrono::steady_clock::now();
            float ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
            float prev = sDirectSimPeakMs.load(std::memory_order_relaxed);
            if (ms > prev) sDirectSimPeakMs.store(ms, std::memory_order_relaxed);
            mDirectSimRunning.store(false, std::memory_order_release);
        }
        if (doReflections) {
            auto t0 = std::chrono::steady_clock::now();
            iplSimulatorRunReflections(mIplSimulator);
            auto t1 = std::chrono::steady_clock::now();
            float ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
            float prev = sReflSimPeakMs.load(std::memory_order_relaxed);
            if (ms > prev) sReflSimPeakMs.store(ms, std::memory_order_relaxed);
            sReflFramesRun.fetch_add(1, std::memory_order_relaxed);
            if (mReflectionMixNode)
                mReflectionMixNode->simulationRan = true;
            mReflectionSimRunning.store(false, std::memory_order_release);
        }
    }
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
            sVoicesDestroyed.fetch_add(1, std::memory_order_relaxed);
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

    // iplSourceAdd modifies the simulator's source list, which both sim threads
    // iterate. Defer the add if either thread is running to prevent races.
    if (mDirectSimRunning.load(std::memory_order_acquire) ||
        mReflectionSimRunning.load(std::memory_order_acquire)) {
        mPendingSourceAdds.push_back(voice.iplSource);
    } else {
        iplSourceAdd(voice.iplSource, mIplSimulator);
    }
    mSimulatorDirty = true;
}

//------------------------------------------------------
void AudioService::waitForReflectionThread()
{
    // Spin-wait for both simulation tasks to complete on the worker thread.
    // This is called infrequently (voice add/remove, shutdown) and the sim
    // tasks are short (~2-5ms direct, ~10-50ms reflections), so spinning
    // is acceptable. The worker thread stays alive — we just wait for its
    // current work to finish.
    while (mDirectSimRunning.load(std::memory_order_acquire) ||
           mReflectionSimRunning.load(std::memory_order_acquire)) {
        std::this_thread::yield();
    }
}

//------------------------------------------------------
void AudioService::removeVoiceSource(ActiveVoice &voice)
{
    if (!voice.iplSource)
        return;

    // If either background sim thread is running, we can't safely call
    // iplSourceRemove (it races with iplSimulatorRunDirect/Reflections).
    // Defer the removal — queue the IPL source handle for later cleanup.
    if (mDirectSimRunning.load(std::memory_order_acquire) ||
        mReflectionSimRunning.load(std::memory_order_acquire)) {
        // If this source was deferred for add but never actually added,
        // just release it directly — no need to defer a removal.
        auto addIt = std::find(mPendingSourceAdds.begin(),
                               mPendingSourceAdds.end(), voice.iplSource);
        if (addIt != mPendingSourceAdds.end()) {
            mPendingSourceAdds.erase(addIt);
            iplSourceRelease(&voice.iplSource);
        } else {
            mPendingSourceRemovals.push_back(voice.iplSource);
            voice.iplSource = nullptr;  // detach from voice (we own it now)
            mSimulatorDirty = true;
        }
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
    // Release ordering ensures all preceding stores (effect pointers, node
    // wiring) are visible before the audio thread sees effectsReady=true.
    dsp.effectsReady.store(true, std::memory_order_release);
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
        // Clean up the IPL source we just added to the simulator to prevent
        // a phantom source that wastes simulation compute forever.
        removeVoiceSource(*voice);
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
    sVoicesCreated.fetch_add(1, std::memory_order_relaxed);
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
        voice.sourceEnded.store(true, std::memory_order_release);
    }
}

//------------------------------------------------------
void AudioService::haltAll()
{
    // Wait for both sim threads to finish — they hold pointers to committed
    // sources that we're about to destroy. Without this, ~ActiveVoice can
    // release IPL sources while iplSimulatorRunDirect/Reflections still uses them.
    waitForReflectionThread();

    // Remove all Steam Audio sources before destroying voices.
    // Since we just joined both threads, removeVoiceSource will take the
    // immediate path (no deferral needed).
    for (auto &[handle, voice] : mVoices) {
        if (voice->initialized) {
            ma_sound_stop(&voice->sound);
        }
        removeVoiceSource(*voice);
    }
    mVoices.clear();

    // Flush any pending deferred adds/removals. Pending adds were never added
    // to the simulator — just release them. Pending removals were already
    // detached from their voices — remove from simulator and release.
    for (auto &src : mPendingSourceAdds) {
        iplSourceRelease(&src);
    }
    mPendingSourceAdds.clear();

    if (mIplSimulator) {
        for (auto &src : mPendingSourceRemovals) {
            iplSourceRemove(src, mIplSimulator);
            iplSourceRelease(&src);
        }
    }
    mPendingSourceRemovals.clear();

    mNextHandle = 0;
}

// ── Ambient sound system ──

//------------------------------------------------------
void AudioService::loadAmbientSounds()
{
    if (!mPropertyService || !mObjectService || !mAudioReady)
        return;

    // Load per-room LoudRoom transmission factors from room object properties.
    // LoudRoom is a single float (default 1.0) that multiplicatively scales
    // sound energy passing through a room during portal propagation.
    // Values < 1.0 dampen (closets, padded rooms), > 1.0 amplify (marble halls).
    mRoomTransmission.clear();
    if (mRoomService) {
        const auto &rooms = mRoomService->getAllRooms();
        int loudRoomCount = 0;
        for (const auto &room : rooms) {
            if (!room) continue;
            int32_t objID = room->getObjectID();
            size_t rawSize = 0;
            const uint8_t *raw = getPropertyRawData(
                mPropertyService.get(), "LoudRoom", objID, rawSize);
            if (raw && rawSize >= sizeof(float)) {
                float transmission;
                std::memcpy(&transmission, raw, sizeof(float));
                if (transmission != 1.0f) {  // only store non-default values
                    mRoomTransmission[room->getRoomID()] = transmission;
                    ++loudRoomCount;
                }
            }
        }
        if (loudRoomCount > 0) {
            std::fprintf(stderr, "AudioService: loaded %d LoudRoom transmission factors\n",
                         loudRoomCount);
        }
    }

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
        float commitMs = sCommitPeakMs.exchange(0.0f, std::memory_order_relaxed);

        // Read and reset main thread + sim worker profiling counters
        float loopMs  = sLoopStepPeakMs.exchange(0.0f, std::memory_order_relaxed);
        float directMs = sDirectSimPeakMs.exchange(0.0f, std::memory_order_relaxed);
        float reflMs  = sReflSimPeakMs.exchange(0.0f, std::memory_order_relaxed);
        int   portalUs = sPortalRoutingTotalUs.exchange(0, std::memory_order_relaxed);
        int   portalCalls = sPortalRoutingCount.exchange(0, std::memory_order_relaxed);
        int   created = sVoicesCreated.exchange(0, std::memory_order_relaxed);
        int   destroyed = sVoicesDestroyed.exchange(0, std::memory_order_relaxed);
        int   reflFrames = sReflFramesRun.exchange(0, std::memory_order_relaxed);

        // Audio budget: 1024 samples @ 48kHz = 21333µs per callback.
        float budgetUs = (static_cast<float>(mFrameSize) / mDeviceSampleRate) * 1e6f;
        float loadPct = (totalUs / budgetUs) * 100.0f;

        int reflVoices = 0;
        int tailVoices = 0;
        for (auto &[h, v] : mVoices) {
            if (v->dspNode.reflectionsActive) ++reflVoices;
            if (v->sourceEnded && !v->finished.load(std::memory_order_relaxed)) ++tailVoices;
        }

        // Effective rays/sec (rays * reflection sim steps run in this dump period)
        float raysPerSec = (reflFrames * mReflectionNumRays) / 5.0f;  // 5s dump interval

        // Log active ambient details to diagnose distant sound leaks
        for (const auto &amb : mAmbients) {
            if (amb.handle != SOUND_HANDLE_INVALID && mVoices.count(amb.handle)) {
                float d = glm::length(mListenerPos - amb.position);
                auto it = mVoices.find(amb.handle);
                if (it == mVoices.end()) continue;
                auto &v = *it->second;
                float atten = v.dspNode.lastAtten.load(std::memory_order_relaxed);
                bool portal = v.dspNode.usePortalRouting;
                float pAtten = v.dspNode.portalAttenuation;
                std::fprintf(stderr, "  [AMB] '%s' dist=%.0f rad=%.0f atten=%.3f portal=%d(%.3f) pos=(%.0f,%.0f,%.0f)\n",
                             amb.schemaName.c_str(), d, amb.radius, atten,
                             portal?1:0, pAtten,
                             amb.position.x, amb.position.y, amb.position.z);
            }
        }

        float portalAvgUs = portalCalls > 0 ? static_cast<float>(portalUs) / portalCalls : 0.0f;

        std::fprintf(stderr, "[Audio] %zu voices (%d refl, %d tail), %d/%zu ambients | "
                     "cb: total=%.0f/%.0fµs (%.0f%%) voice=%.0fµs mix=%.0fµs | "
                     "main: loop=%.1fms commit=%.1fms portal=%.0fµs(%dcalls,avg=%.0f) | "
                     "sim: direct=%.1fms refl=%.1fms(%dsteps) rays/s=%.0f | "
                     "churn: +%d/-%d\n",
                     mVoices.size(), reflVoices, tailVoices, playing, mAmbients.size(),
                     totalUs, budgetUs, loadPct, voiceUs, mixUs,
                     loopMs, commitMs, static_cast<float>(portalUs), portalCalls, portalAvgUs,
                     directMs, reflMs, reflFrames, raysPerSec,
                     created, destroyed);
    }

    for (auto &amb : mAmbients) {
        // Ambient sound activation uses EUCLIDEAN distance, matching the
        // original Dark Engine's ambient system (AMBIENT.C). The room portal
        // system was never used for ambient activation — ambients work purely
        // on radius. Room routing handles cross-room propagation for voices
        // in the voice update loop (Step 2b), not here.
        //
        // Start voices at 1.5x radius (before audible) so the distance-based
        // volume curve fades in smoothly. Stop at 2x radius (hysteresis).
        float dist = glm::length(mListenerPos - amb.position);
        float startRadius = amb.radius * 1.5f;
        float stopRadius = amb.radius * 2.0f;

        bool alreadyPlaying = (amb.handle != SOUND_HANDLE_INVALID);
        bool inRange = (amb.radius > 0.0f &&
                        (alreadyPlaying ? dist < stopRadius : dist < startRadius));

        if (inRange) {
            // Start voice if not already playing
            if (amb.handle == SOUND_HANDLE_INVALID) {
                bool isLooping = !(amb.flags & AMB_ONCE_ONLY);
                if (mSchemaParser) {
                    const SchemaEntry *schema = mSchemaParser->findSchema(amb.schemaName);
                    if (schema && !schema->samples.empty()) {
                        const SchemaSample &sample = schema->samples[0];
                        // Start at volume 0 — the volume update below will set the
                        // correct level this same frame. Prevents pop-in at full volume
                        // before Steam Audio has simulated this source.
                        amb.handle = startVoice(amb.schemaName, sample.name, amb.position,
                                                schema->playParams.priority, isLooping,
                                                amb.objID, 0.0f);
                    }
                }
                // Fallback: try loading schema name as a raw sound
                if (amb.handle == SOUND_HANDLE_INVALID) {
                    amb.handle = startVoice(amb.schemaName, amb.schemaName, amb.position,
                                            64, isLooping, amb.objID, 0.0f);
                }
            }

            // Update volume — purely position-based, no time-based fades.
            // Volume curve: full volume at dist=0, fades to 0 at dist=radius.
            // Beyond radius (in the start/stop management zone), volume is 0
            // but the voice stays alive so it can fade back in smoothly when
            // the player approaches again.
            if (amb.handle != SOUND_HANDLE_INVALID) {
                auto it = mVoices.find(amb.handle);
                if (it == mVoices.end()) {
                    amb.handle = SOUND_HANDLE_INVALID;
                    continue;
                }

                float baseVol = schemaVolumeToLinear(amb.volume);
                float distFactor = std::max(0.0f, 1.0f - (dist / amb.radius));
                if (!(amb.flags & AMB_NO_FADE)) {
                    distFactor *= distFactor;  // quadratic for natural rolloff
                }
                ma_sound_set_volume(&it->second->sound, baseVol * distFactor);
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
    // Boost footstep volume to compensate for Steam Audio's distance
    // attenuation at close range (source at feet ~3 units from listener).
    // Without this, the DSP callback's directAtten multiplies the already
    // speed-scaled volume, making footsteps sound too distant.
    float finalVol = baseVol * speedFactor * 2.0f;

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

    // Mark as player-emitted so the DSP callback skips attenuation.
    // The player's own footsteps should always be audible at the volume
    // set by the footstep system — never occluded by floor geometry.
    if (h != SOUND_HANDLE_INVALID && mVoices.count(h)) {
        mVoices[h]->playerEmitted = true;
    }

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

    // Mark as player-emitted — landing impacts are never occluded
    if (h != SOUND_HANDLE_INVALID && mVoices.count(h)) {
        mVoices[h]->playerEmitted = true;
    }
}

// ── Portal blend: smooth room transitions ──

//------------------------------------------------------
void AudioService::computePortalBlend()
{
    mPortalBlend = {};  // reset to inactive

    if (!mRoomService || !mRoomService->isLoaded())
        return;

    Room *primaryRoom = mRoomService->roomFromPoint(mListenerPos);
    if (!primaryRoom)
        return;

    constexpr float kBlendRadius = 3.0f;

    // Find the closest portal the listener is near enough to trigger blending.
    float bestAbsDist = kBlendRadius;
    RoomPortal *bestPortal = nullptr;
    float bestSignedDist = 0.0f;

    uint32_t portalCount = primaryRoom->getPortalCount();
    for (uint32_t i = 0; i < portalCount; ++i) {
        RoomPortal *portal = primaryRoom->getPortal(i);
        if (!portal || !portal->getFarRoom())
            continue;

        // Signed distance from listener to portal plane.
        // After our d-negation on load: positive = inside (near-room side).
        float signedDist = portal->getPlane().getDistance(mListenerPos);
        float absDist = std::fabs(signedDist);

        if (absDist >= bestAbsDist)
            continue;

        // Check edge containment: is the listener within the portal's
        // 2D footprint? isInside() tests edge planes only.
        if (!portal->isInside(mListenerPos))
            continue;

        bestAbsDist = absDist;
        bestPortal = portal;
        bestSignedDist = signedDist;
    }

    if (!bestPortal)
        return;

    // Blend weight from portal plane signed distance.
    // signedDist > 0: listener on near-room side → blend toward roomA.
    // signedDist < 0: listener crossed to far side → blend toward roomB.
    // signedDist = 0: on the portal plane → 50/50 blend.
    float blend = 0.5f - bestSignedDist / (2.0f * kBlendRadius);
    blend = std::max(0.0f, std::min(1.0f, blend));

    mPortalBlend.active = true;
    mPortalBlend.roomA  = primaryRoom;
    mPortalBlend.roomB  = bestPortal->getFarRoom();
    mPortalBlend.blend  = blend;
}

//------------------------------------------------------
SoundPropInfo AudioService::propagateSoundBlended(const Vector3 &sourcePos,
                                                   float maxDist) const
{
    if (!mPortalBlend.active) {
        return propagateSound(sourcePos, mListenerPos, maxDist);
    }

    // Resolve source room once (shared between both calls)
    Room *sourceRoom = mRoomService ? mRoomService->roomFromPoint(sourcePos) : nullptr;

    // Propagate assuming listener is in roomA
    SoundPropInfo propA = propagateSound(sourcePos, mListenerPos,
                                          sourceRoom, mPortalBlend.roomA, maxDist);
    // Propagate assuming listener is in roomB
    SoundPropInfo propB = propagateSound(sourcePos, mListenerPos,
                                          sourceRoom, mPortalBlend.roomB, maxDist);

    float t = mPortalBlend.blend;

    // If only one path reached, use it
    if (propA.reached && !propB.reached) return propA;
    if (!propA.reached && propB.reached) return propB;
    if (!propA.reached && !propB.reached) return propA;

    // Blend the results
    SoundPropInfo blended;
    blended.reached = true;
    blended.effectiveDistance = propA.effectiveDistance * (1.0f - t) + propB.effectiveDistance * t;
    blended.realDistance = propA.realDistance * (1.0f - t) + propB.realDistance * t;
    blended.totalBlocking = propA.totalBlocking * (1.0f - t) + propB.totalBlocking * t;
    blended.virtualPosition = propA.virtualPosition * (1.0f - t) + propB.virtualPosition * t;
    return blended;
}

// ── Sound propagation through portal graph ──

//------------------------------------------------------
SoundPropInfo AudioService::propagateSound(const Vector3 &sourcePos,
                                            const Vector3 &listenerPos,
                                            float maxDist) const
{
    if (!mRoomService || !mRoomService->isLoaded())
        return {};

    Room *sourceRoom = mRoomService->roomFromPoint(sourcePos);
    Room *listenerRoom = mRoomService->roomFromPoint(listenerPos);

    // If source or listener is outside all room OBBs, the 5-arg overload
    // handles it with a euclidean distance fallback. This matches the original
    // Dark Engine behavior: objects outside the room database cannot propagate
    // sound through the room portal graph. The ambient system uses euclidean
    // distance directly (matching AMBIENT.C), so this fallback only affects
    // non-ambient voices (footsteps, one-shots) that happen to be outside rooms.
    return propagateSound(sourcePos, listenerPos, sourceRoom, listenerRoom, maxDist);
}

//------------------------------------------------------
SoundPropInfo AudioService::propagateSound(const Vector3 &sourcePos,
                                            const Vector3 &listenerPos,
                                            Room *sourceRoom, Room *listenerRoom,
                                            float maxDist) const
{
    SoundPropInfo result;

    if (!sourceRoom || !listenerRoom) {
        // Both rooms should be resolved by the 3-arg wrapper's nearest-room
        // fallback. If we still have NULL here (no rooms loaded at all), use
        // euclidean distance as a last resort.
        float dist = glm::length(listenerPos - sourcePos);
        if (dist <= maxDist) {
            result.reached = true;
            result.realDistance = dist;
            result.effectiveDistance = dist;
            result.totalBlocking = 0.0f;
            result.virtualPosition = sourcePos;
        }

        static int sFailCount = 0;
        if (sFailCount < 5) {
            std::fprintf(stderr, "[PORTAL] propagateSound: NULL room after fallback "
                         "src=(%.1f,%.1f,%.1f)→%s lst=(%.1f,%.1f,%.1f)→%s dist=%.1f\n",
                         sourcePos.x, sourcePos.y, sourcePos.z,
                         sourceRoom ? "OK" : "NULL",
                         listenerPos.x, listenerPos.y, listenerPos.z,
                         listenerRoom ? "OK" : "NULL", dist);
            // Find the closest room to the listener and dump its data
            if (sFailCount == 0) {
                auto &rooms = mRoomService->getAllRooms();
                std::fprintf(stderr, "[PORTAL] %zu rooms loaded.\n", rooms.size());

                // Find closest room center to listener
                float bestDist = 1e9f;
                Room *bestRoom = nullptr;
                for (auto &rp : rooms) {
                    if (!rp) continue;
                    float d = glm::length(rp->getCenter() - listenerPos);
                    if (d < bestDist) { bestDist = d; bestRoom = rp.get(); }
                }
                if (bestRoom) {
                    auto c = bestRoom->getCenter();
                    std::fprintf(stderr, "[PORTAL] Closest room %d: center=(%.1f,%.1f,%.1f) dist=%.1f\n",
                                 bestRoom->getRoomID(), c.x, c.y, c.z, bestDist);
                    const Plane *planes = bestRoom->getBoundingPlanes();
                    for (int p = 0; p < 6; ++p) {
                        float dist = planes[p].getDistance(listenerPos);
                        std::fprintf(stderr, "[PORTAL]   plane[%d]: n=(%.3f,%.3f,%.3f) d=%.3f dist_to_listener=%.3f %s\n",
                                     p, planes[p].normal.x, planes[p].normal.y, planes[p].normal.z,
                                     planes[p].d, dist, dist < 0 ? "OUTSIDE" : "inside");
                    }
                }
            }
            sFailCount++;
        }
        return result;
    }

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

    // ── Dijkstra BFS through room portal graph ──
    // Uses the precomputed portal-to-portal distance matrix within each room
    // (from ROOM_DB) instead of euclidean entry-point-to-portal distances.
    // Tracks enter-portal indices for matrix lookup and path reconstruction.
    // Based on the Dark Engine's cRoomPropAgent::PropagateBF algorithm.
    struct BFSEntry {
        Room    *room;
        float    realDist;        // Cumulative geometric distance
        float    effectiveDist;   // With blocking penalties applied
        float    cumulativeTransmission; // Product of (1 - blocking) per portal
        int32_t  enterPortalIdx;  // Portal index we entered this room through (-1 = source room)
        Room    *prevRoom;        // Previous room (for path reconstruction)

        bool operator>(const BFSEntry &o) const {
            return effectiveDist > o.effectiveDist;
        }
    };

    std::priority_queue<BFSEntry, std::vector<BFSEntry>, std::greater<BFSEntry>> pq;

    // Reuse visited map across calls (thread_local retains bucket memory)
    thread_local std::unordered_map<int32_t, float> bestDist;
    bestDist.clear();

    // For path reconstruction: store the BFS entry that reached each room
    thread_local std::unordered_map<int32_t, BFSEntry> reachedFrom;
    reachedFrom.clear();

    // Seed with source room
    BFSEntry start{};
    start.room = sourceRoom;
    start.realDist = 0.0f;
    start.effectiveDist = 0.0f;
    start.cumulativeTransmission = 1.0f;  // full transmission (no blocking)
    start.enterPortalIdx = -1;
    start.prevRoom = nullptr;
    pq.push(start);
    bestDist[sourceRoom->getRoomID()] = 0.0f;

    BFSEntry listenerEntry{};
    bool foundListener = false;

    while (!pq.empty()) {
        BFSEntry cur = pq.top();
        pq.pop();

        auto it = bestDist.find(cur.room->getRoomID());
        if (it != bestDist.end() && cur.effectiveDist > it->second)
            continue;

        reachedFrom[cur.room->getRoomID()] = cur;

        // Reached listener's room
        if (cur.room == listenerRoom) {
            listenerEntry = cur;
            foundListener = true;
            break;
        }

        // Explore portals
        uint32_t portalCount = cur.room->getPortalCount();
        for (uint32_t i = 0; i < portalCount; ++i) {
            // Don't go back through the portal we entered from
            if (static_cast<int32_t>(i) == cur.enterPortalIdx)
                continue;

            RoomPortal *portal = cur.room->getPortal(i);
            if (!portal) continue;

            Room *nextRoom = portal->getFarRoom();
            if (!nextRoom) continue;

            // Distance: use precomputed portal-to-portal matrix for intra-room
            // distance. For the source room (enterPortalIdx == -1), use euclidean
            // from source position to exit portal center.
            float segDist;
            if (cur.enterPortalIdx >= 0) {
                segDist = cur.room->getPortalDist(i, static_cast<uint32_t>(cur.enterPortalIdx));
            } else {
                segDist = glm::length(portal->getCenter() - sourcePos);
            }
            float newRealDist = cur.realDist + segDist;

            // Blocking: Dark Engine formula — dist += (maxDist - dist) * blocking.
            // No artificial per-portal penalty: room portals are real architectural
            // doorways, so open portals have zero blocking (unlike BSP cell boundaries).
            float blocking = getBlockingFactor(cur.room->getRoomID(),
                                                nextRoom->getRoomID());
            float newEffDist = cur.effectiveDist + segDist;
            if (blocking > 0.0f && newEffDist < maxDist) {
                newEffDist += (maxDist - newEffDist) * blocking;
            }

            if (newEffDist > maxDist)
                continue;

            // Multiplicative transmission: each portal reduces the cumulative
            // transmission by its blocking factor. Two doors each blocking 50%
            // give total transmission 0.5 * 0.5 = 0.25 (not max(0.5, 0.5) = 0.5).
            // This matches the Dark Engine's FindSoundPath blocking formula.
            float newTransmission = cur.cumulativeTransmission * (1.0f - blocking);

            // LoudRoom: per-room transmission multiplier from P$LoudRoom property.
            // Applied when entering the next room. Default 1.0 (no effect).
            auto loudIt = mRoomTransmission.find(nextRoom->getRoomID());
            if (loudIt != mRoomTransmission.end()) {
                newTransmission *= loudIt->second;
            }

            int32_t nextID = nextRoom->getRoomID();
            auto bestIt = bestDist.find(nextID);
            if (bestIt != bestDist.end() && newEffDist >= bestIt->second)
                continue;
            bestDist[nextID] = newEffDist;

            // Find the entry portal index in the next room. Use the stored
            // destination portal ID (from ROOM_DB) for O(1) lookup instead of
            // linear search. This correctly handles rooms with multiple portals
            // to the same neighbor.
            int32_t farPortalIdx = -1;
            int32_t destPortalID = portal->getDestPortalID();
            for (uint32_t j = 0; j < nextRoom->getPortalCount(); ++j) {
                RoomPortal *p = nextRoom->getPortal(j);
                if (p && p->getPortalID() == destPortalID) {
                    farPortalIdx = static_cast<int32_t>(j);
                    break;
                }
            }

            BFSEntry next{};
            next.room = nextRoom;
            next.realDist = newRealDist;
            next.effectiveDist = newEffDist;
            next.cumulativeTransmission = newTransmission;
            next.enterPortalIdx = farPortalIdx;
            next.prevRoom = cur.room;
            pq.push(next);
        }
    }

    if (!foundListener)
        return result;  // unreachable within maxDist

    // ── Path reconstruction + portal edge projection ──
    // Walk backward from listener room to source room, collecting the portal
    // sequence. Then run a forward pass with anchor projection to find the
    // shortest geometric path that threads through all portal openings.
    // Based on the Dark Engine's FindSoundPath algorithm.

    // Reconstruct portal sequence (reversed: listener→source, then flip)
    struct PortalInfo {
        RoomPortal *portal;
        Room       *fromRoom;
    };
    std::vector<PortalInfo> portalChain;
    {
        Room *walkRoom = listenerRoom;
        while (walkRoom != sourceRoom) {
            auto it = reachedFrom.find(walkRoom->getRoomID());
            if (it == reachedFrom.end())
                break;  // shouldn't happen if foundListener is true

            const BFSEntry &entry = it->second;
            if (!entry.prevRoom)
                break;

            // Find the portal from prevRoom to walkRoom
            for (uint32_t i = 0; i < entry.prevRoom->getPortalCount(); ++i) {
                RoomPortal *p = entry.prevRoom->getPortal(i);
                if (p && p->getFarRoom() == walkRoom) {
                    portalChain.push_back({p, entry.prevRoom});
                    break;
                }
            }
            walkRoom = entry.prevRoom;
        }
        // Reverse so portalChain[0] is nearest to source
        std::reverse(portalChain.begin(), portalChain.end());
    }

    // Forward-pass anchor projection: cast rays from source through each
    // portal toward the next portal (or listener). When a ray misses a portal,
    // project onto the nearest edge to create an anchor point where sound
    // bends around the doorframe.
    struct Anchor {
        Vector3 pos;
        bool valid = false;
    };
    std::vector<Anchor> anchors(portalChain.size());

    Vector3 leadPt = sourcePos;  // current "leading" point
    for (size_t i = 0; i < portalChain.size(); ++i) {
        // Target: next portal center, or listener if this is the last portal
        Vector3 target = (i + 1 < portalChain.size())
                         ? portalChain[i + 1].portal->getCenter()
                         : listenerPos;

        Vector3 dir = target - leadPt;

        if (!portalChain[i].portal->raycast(leadPt, dir)) {
            // Ray missed — project onto nearest portal edge
            Vector3 projPt;
            if (portalChain[i].portal->getRaycastProjection(leadPt, dir, projPt)) {
                anchors[i].pos = projPt;
                anchors[i].valid = true;
                leadPt = projPt;
            }
        }
        // If raycast succeeds, no anchor needed — path goes straight through
    }

    // Compute path distance through anchor points
    float pathDist = 0.0f;
    int lastAnchor = -1;
    Vector3 virtualPos = sourcePos;

    for (size_t i = 0; i < anchors.size(); ++i) {
        if (anchors[i].valid) {
            if (lastAnchor < 0) {
                pathDist += glm::length(anchors[i].pos - sourcePos);
            } else {
                pathDist += glm::length(anchors[i].pos - anchors[lastAnchor].pos);
            }
            lastAnchor = static_cast<int>(i);
            virtualPos = anchors[i].pos;
        }
    }

    // Final segment: last anchor (or source) to listener
    if (lastAnchor < 0) {
        // Direct LOS through all portals (rare but possible for aligned rooms)
        pathDist += glm::length(listenerPos - sourcePos);
        virtualPos = sourcePos;
    } else {
        pathDist += glm::length(listenerPos - anchors[lastAnchor].pos);
        virtualPos = anchors[lastAnchor].pos;
    }

    // Apply blocking to the path distance (Dark Engine munged distance formula).
    // totalBlocking = 1 - cumulativeTransmission (0 = no blocking, 1 = fully blocked).
    // The original formula: dist += (maxDist - dist) * totalBlocking.
    float totalBlocking = 1.0f - listenerEntry.cumulativeTransmission;
    float effDist = pathDist;
    if (totalBlocking > 0.0f && effDist < maxDist) {
        effDist += (maxDist - effDist) * totalBlocking;
    }

    if (effDist <= maxDist) {
        result.reached = true;
        result.realDistance = pathDist;
        result.effectiveDistance = effDist;
        result.totalBlocking = totalBlocking;
        result.virtualPosition = virtualPos;
    }

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
    if (it != mBlockingFactors.end())
        return it->second;
    // Open portals have zero blocking. Room portals are real architectural
    // doorways (not BSP cell boundaries), so traversing an open doorway has
    // no artificial penalty. Door blocking is set explicitly via
    // setBlockingFactor() when doors close.
    return 0.0f;
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

// ── Probe baking and loading ──

//------------------------------------------------------
int AudioService::getProbeCount() const
{
    return mProbeCount;
}

void AudioService::startAudioRecording()
{
    startRecording(mDeviceSampleRate);
}

void AudioService::stopAudioRecording()
{
    stopRecording();
}

bool AudioService::isRecordingAudio() const
{
    return sRecording.load(std::memory_order_relaxed);
}

//------------------------------------------------------
std::string AudioService::getProbeFilePath(const std::string &misPath,
                                            const std::string &gameName)
{
    // Extract mission filename without extension
    std::string missionName;
    auto lastSlash = misPath.find_last_of("/\\");
    std::string filename = (lastSlash != std::string::npos)
        ? misPath.substr(lastSlash + 1) : misPath;
    auto dotPos = filename.rfind('.');
    missionName = (dotPos != std::string::npos)
        ? filename.substr(0, dotPos) : filename;

    // Convert to lowercase for consistency
    for (auto &c : missionName) c = static_cast<char>(std::tolower(c));

    // Build path: ~/darkness/{gameName}/baked_probes/{missionName}.probes
    const char *home = std::getenv("HOME");
    if (!home) home = ".";
    std::string dir = std::string(home) + "/darkness/" + gameName + "/baked_probes";

    // Create directories (mkdir -p equivalent)
    std::string pathSoFar;
    for (size_t i = 0; i < dir.size(); ++i) {
        if (dir[i] == '/' || i == dir.size() - 1) {
            pathSoFar = dir.substr(0, i + 1);
#ifdef _WIN32
            _mkdir(pathSoFar.c_str());
#else
            mkdir(pathSoFar.c_str(), 0755);
#endif
        }
    }

    return dir + "/" + missionName + ".probes";
}

bool AudioService::bakeProbes(const std::string &outputPath,
                               std::atomic<float> *progress,
                               float spacing, float height)
{
    if (!mIplContext || !mIplScene) {
        LOG_ERROR("AudioService: cannot bake probes — no acoustic scene");
        return false;
    }

    std::fprintf(stderr, "Baking probes: spacing=%.1f height=%.1f ...\n", spacing, height);

    // Step 1: Generate probes on a uniform floor grid
    IPLProbeArray probeArray = nullptr;
    IPLerror err = iplProbeArrayCreate(mIplContext, &probeArray);
    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("AudioService: iplProbeArrayCreate failed (%d)", err);
        return false;
    }

    // Use the actual acoustic scene bounding box for probe generation.
    // The OBB transform maps a unit cube [-0.5, 0.5]³ to world space.
    Vector3 center = (mSceneMin + mSceneMax) * 0.5f;
    Vector3 extent = mSceneMax - mSceneMin;
    // Add a small margin to ensure probes cover the edges
    extent += Vector3(spacing * 2.0f);

    IPLMatrix4x4 transform{};
    transform.elements[0][0] = extent.x;   // X scale
    transform.elements[1][1] = extent.y;   // Y scale
    transform.elements[2][2] = extent.z;   // Z scale
    transform.elements[3][0] = center.x;   // X translation
    transform.elements[3][1] = center.y;   // Y translation
    transform.elements[3][2] = center.z;   // Z translation
    transform.elements[3][3] = 1.0f;

    std::fprintf(stderr, "Scene bounds: (%.0f,%.0f,%.0f)-(%.0f,%.0f,%.0f) extent=(%.0f,%.0f,%.0f)\n",
                 mSceneMin.x, mSceneMin.y, mSceneMin.z,
                 mSceneMax.x, mSceneMax.y, mSceneMax.z,
                 extent.x, extent.y, extent.z);

    IPLProbeGenerationParams genParams{};
    genParams.type = IPL_PROBEGENERATIONTYPE_UNIFORMFLOOR;
    genParams.spacing = spacing;
    genParams.height = height;
    genParams.transform = transform;

    iplProbeArrayGenerateProbes(probeArray, mIplScene, &genParams);

    int numProbes = iplProbeArrayGetNumProbes(probeArray);
    std::fprintf(stderr, "Generated %d probes (spacing=%.1f, height=%.1f)\n",
                 numProbes, spacing, height);

    if (numProbes == 0) {
        LOG_ERROR("AudioService: no probes generated — check scene geometry");
        iplProbeArrayRelease(&probeArray);
        return false;
    }

    // Step 2: Create probe batch and add the generated probes
    IPLProbeBatch probeBatch = nullptr;
    err = iplProbeBatchCreate(mIplContext, &probeBatch);
    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("AudioService: iplProbeBatchCreate failed (%d)", err);
        iplProbeArrayRelease(&probeArray);
        return false;
    }

    iplProbeBatchAddProbeArray(probeBatch, probeArray);
    iplProbeBatchCommit(probeBatch);
    iplProbeArrayRelease(&probeArray);  // batch now owns the probe data

    // Step 3: Bake pathing data (visibility graph + shortest paths)
    std::fprintf(stderr, "Baking pathing data for %d probes...\n", numProbes);

    IPLBakedDataIdentifier pathId{};
    pathId.type = IPL_BAKEDDATATYPE_PATHING;
    pathId.variation = IPL_BAKEDDATAVARIATION_DYNAMIC;

    IPLPathBakeParams bakeParams{};
    bakeParams.scene = mIplScene;
    bakeParams.probeBatch = probeBatch;
    bakeParams.identifier = pathId;
    bakeParams.numSamples = 4;       // visibility test samples (rays = numSamples²)
    bakeParams.radius = spacing;     // probe influence radius
    bakeParams.threshold = 0.1f;     // visibility threshold
    bakeParams.visRange = 200.0f;    // max visibility distance (SOUND_MAX_DIST)
    bakeParams.pathRange = 200.0f;   // max path length
    bakeParams.numThreads = 4;       // parallel baking

    auto bakeStart = std::chrono::steady_clock::now();

    iplPathBakerBake(mIplContext, &bakeParams,
        [](IPLfloat32 p, void *userData) {
            auto *prog = static_cast<std::atomic<float> *>(userData);
            if (prog) prog->store(p, std::memory_order_relaxed);
        },
        progress);

    auto bakeEnd = std::chrono::steady_clock::now();
    float bakeSec = std::chrono::duration<float>(bakeEnd - bakeStart).count();
    std::fprintf(stderr, "Pathing bake complete: %d probes in %.1f seconds\n",
                 numProbes, bakeSec);

    // Step 3b: Bake reflection IRs at each probe position.
    // This pre-computes reverb impulse responses so that voices outside the
    // real-time top-N can use baked reverb instead of being dry.
    // Uses REVERB variation — one bake covers all sources (listener-position-based).
    std::fprintf(stderr, "Baking reflection IRs for %d probes (rays=%d bounces=%d duration=%.1fs)...\n",
                 numProbes, mReflectionNumRays, mReflectionNumBounces, mReflectionDuration);

    IPLBakedDataIdentifier reflId{};
    reflId.type = IPL_BAKEDDATATYPE_REFLECTIONS;
    reflId.variation = IPL_BAKEDDATAVARIATION_REVERB;

    unsigned int hwThreads = std::thread::hardware_concurrency();
    int bakeThreads = std::max(2u, hwThreads > 2 ? hwThreads - 2 : 2u);

    IPLReflectionsBakeParams reflBakeParams{};
    reflBakeParams.scene = mIplScene;
    reflBakeParams.probeBatch = probeBatch;
    reflBakeParams.sceneType = IPL_SCENETYPE_DEFAULT;
    reflBakeParams.identifier = reflId;
    reflBakeParams.bakeFlags = static_cast<IPLReflectionsBakeFlags>(
        IPL_REFLECTIONSBAKEFLAGS_BAKECONVOLUTION);
    reflBakeParams.numRays = mReflectionNumRays;
    reflBakeParams.numDiffuseSamples = mBakeDiffuseSamples;
    reflBakeParams.numBounces = mReflectionNumBounces;
    reflBakeParams.simulatedDuration = mReflectionDuration;
    reflBakeParams.savedDuration = mReflectionDuration;  // save full IR
    reflBakeParams.order = mAmbisonicsOrder;
    reflBakeParams.numThreads = bakeThreads;
    reflBakeParams.irradianceMinDistance = 1.0f;

    auto reflBakeStart = std::chrono::steady_clock::now();

    iplReflectionsBakerBake(mIplContext, &reflBakeParams,
        [](IPLfloat32 p, void *userData) {
            auto *prog = static_cast<std::atomic<float> *>(userData);
            if (prog) prog->store(p, std::memory_order_relaxed);
        },
        progress);

    auto reflBakeEnd = std::chrono::steady_clock::now();
    float reflBakeSec = std::chrono::duration<float>(reflBakeEnd - reflBakeStart).count();
    std::fprintf(stderr, "Reflection bake complete: %d probes in %.1f seconds\n",
                 numProbes, reflBakeSec);

    // Step 4: Serialize to disk (includes both pathing and reflection layers)
    IPLSerializedObjectSettings soSettings{};
    soSettings.data = nullptr;
    soSettings.size = 0;

    IPLSerializedObject serializedObject = nullptr;
    err = iplSerializedObjectCreate(mIplContext, &soSettings, &serializedObject);
    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("AudioService: iplSerializedObjectCreate failed (%d)", err);
        iplProbeBatchRelease(&probeBatch);
        return false;
    }

    iplProbeBatchSave(probeBatch, serializedObject);

    IPLsize dataSize = iplSerializedObjectGetSize(serializedObject);
    IPLbyte *data = iplSerializedObjectGetData(serializedObject);

    FILE *f = std::fopen(outputPath.c_str(), "wb");
    if (!f) {
        LOG_ERROR("AudioService: failed to open '%s' for writing", outputPath.c_str());
        iplSerializedObjectRelease(&serializedObject);
        iplProbeBatchRelease(&probeBatch);
        return false;
    }

    std::fwrite(data, 1, dataSize, f);
    std::fclose(f);

    std::fprintf(stderr, "Saved %d probes to '%s' (%zu bytes)\n",
                 numProbes, outputPath.c_str(), static_cast<size_t>(dataSize));

    iplSerializedObjectRelease(&serializedObject);
    iplProbeBatchRelease(&probeBatch);
    return true;
}

//------------------------------------------------------
bool AudioService::loadProbes(const std::string &probePath)
{
    if (!mIplContext || !mIplSimulator) {
        LOG_ERROR("AudioService: cannot load probes — no context/simulator");
        return false;
    }

    // Release any previously loaded probes
    if (mIplProbeBatch) {
        waitForReflectionThread();
        iplSimulatorRemoveProbeBatch(mIplSimulator, mIplProbeBatch);
        iplSimulatorCommit(mIplSimulator);
        iplProbeBatchRelease(&mIplProbeBatch);
        mIplProbeBatch = nullptr;
        mProbeCount = 0;
    }

    // Read file into memory
    FILE *f = std::fopen(probePath.c_str(), "rb");
    if (!f) {
        LOG_INFO("AudioService: no probe file at '%s' — pathing disabled",
                 probePath.c_str());
        return false;
    }

    std::fseek(f, 0, SEEK_END);
    size_t fileSize = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);

    std::vector<uint8_t> fileData(fileSize);
    std::fread(fileData.data(), 1, fileSize, f);
    std::fclose(f);

    // Deserialize
    IPLSerializedObjectSettings soSettings{};
    soSettings.data = reinterpret_cast<IPLbyte *>(fileData.data());
    soSettings.size = static_cast<IPLsize>(fileSize);

    IPLSerializedObject serializedObject = nullptr;
    IPLerror err = iplSerializedObjectCreate(mIplContext, &soSettings, &serializedObject);
    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("AudioService: iplSerializedObjectCreate failed (%d) loading '%s'",
                  err, probePath.c_str());
        return false;
    }

    err = iplProbeBatchLoad(mIplContext, serializedObject, &mIplProbeBatch);
    iplSerializedObjectRelease(&serializedObject);

    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("AudioService: iplProbeBatchLoad failed (%d) for '%s'",
                  err, probePath.c_str());
        return false;
    }

    iplProbeBatchCommit(mIplProbeBatch);
    mProbeCount = iplProbeBatchGetNumProbes(mIplProbeBatch);

    // Register with simulator
    waitForReflectionThread();
    iplSimulatorAddProbeBatch(mIplSimulator, mIplProbeBatch);
    iplSimulatorCommit(mIplSimulator);

    // Check if the probe batch contains baked reflection IRs.
    // If so, non-top-N voices can use baked reverb instead of being dry.
    IPLBakedDataIdentifier reflId{};
    reflId.type = IPL_BAKEDDATATYPE_REFLECTIONS;
    reflId.variation = IPL_BAKEDDATAVARIATION_REVERB;
    IPLsize reflDataSize = iplProbeBatchGetDataSize(mIplProbeBatch, &reflId);
    mProbesHaveReflections = (reflDataSize > 0);

    std::fprintf(stderr, "AudioService: loaded %d probes from '%s' "
                 "(reflections=%s, refl_size=%zu bytes)\n",
                 mProbeCount, probePath.c_str(),
                 mProbesHaveReflections ? "yes" : "no",
                 static_cast<size_t>(reflDataSize));
    return true;
}

} // namespace Darkness
