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
#include "AIHearingData.h"
#include "AmbientSoundManager.h"
#include "AudioLog.h"
#include "AudioMetering.h"
#include "AudioOcclusion.h"
#include "AcousticMaterials.h"
#include "ConvolutionWorkerPool.h"
#include "CRFSoundLoader.h"
#include "EnvSoundDatabase.h"
#include "ProbeFile.h"
#include "ProbeManager.h"
#include "ReflectionSimulator.h"
#include "SchemaParser.h"
#include "SchemaPropertyOverrides.h"
#include "SchemaSamplesChunk.h"
#include "SchemaTypes.h"
#include "SoundPropagation.h"
#include "SpeechDatabase.h"
#include "SteamAudioPathing.h"
#include "SpeechSelector.h"
#include "SubSourceSlots.h"
#include "VoicePool.h"
#include "ServiceCommon.h"
#include <algorithm>
#include <atomic>
#include <queue>
#include <sys/stat.h>
#include "DarknessServiceManager.h"
#include "database/DatabaseService.h"
#include "link/LinkService.h"
#include "link/Relation.h"
#include "loop/LoopService.h"
#include "object/ObjectService.h"
#include "room/Room.h"
#include "room/RoomPortal.h"
#include "room/RoomService.h"
#include "property/DarkPropertyDefs.h"
#include "property/PropertyService.h"
#include "property/TypedProperty.h"
#include "logger.h"

#include <unordered_set>
#include <cmath>

// miniaudio — single-header C library, implementation compiled here
#define MINIAUDIO_IMPLEMENTATION
#include <miniaudio.h>

// Steam Audio C API
#include <phonon.h>

namespace Darkness {

/*----------------------------------------------------*/
/*---------- EAX Reverb Preset Reference Table -------*/
/*----------------------------------------------------*/

/// EAX 2.0 reverb preset names and approximate decay times (seconds).
/// The original Dark Engine stored a per-room preset index (0-25) in the
/// ROOM_EAX chunk and P$Acoustics property, passing it to Creative's EAX
/// hardware reverb API. We parse these for verification: comparing the
/// designer's intended reverb character against what Steam Audio's physics-
/// based convolution produces from the actual room geometry and materials.
///
/// Decay times from the EAX 2.0 specification (Creative Technology, 1999).
struct EAXPresetInfo {
    const char *name;
    float decayTime;  // approximate T60 in seconds
};
static const EAXPresetInfo kEAXPresets[26] = {
    {"Generic",         1.49f},
    {"PaddedCell",      0.17f},
    {"Room",            0.40f},
    {"Bathroom",        1.49f},
    {"LivingRoom",      0.50f},
    {"StoneRoom",       2.31f},
    {"Auditorium",      4.32f},
    {"ConcertHall",     3.92f},
    {"Cave",            2.91f},
    {"Arena",           7.24f},
    {"Hangar",         10.05f},
    {"CarpetedHallway", 0.30f},
    {"Hallway",         1.49f},
    {"StoneCorridor",   2.70f},
    {"Alley",           1.49f},
    {"Forest",          1.49f},
    {"City",            1.49f},
    {"Mountains",       1.49f},
    {"Quarry",          1.49f},
    {"Plain",           1.49f},
    {"ParkingLot",      1.65f},
    {"SewerPipe",       2.81f},
    {"UnderWater",      1.49f},
    {"Drugged",         8.39f},
    {"Dizzy",          17.23f},
    {"Psychotic",       7.56f},
};

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
/*----- NaN/Inf guards and denormal-flush helpers ----*/
/*----------------------------------------------------*/
//
// The audio pipeline contains many IIR filters (master EQ biquad, per-voice
// door-blocking LPF, compressor envelope) whose state evolves recursively:
// a single non-finite sample sticks in the state forever and silences every
// subsequent sample on that channel.  The downstream compressor / limiter
// stages have asymmetric NaN behavior (`std::max(NaN, x)` typically returns
// `x`; `NaN > knee` is false), so a NaN'd left channel can persist while
// the right channel continues to play — exactly the "left channel goes
// silent" symptom we observe in the field.
//
// The producers of non-finite samples are not fully known (Steam Audio HRTF
// edge cases, large transient clicks/pops driving the IIRs into overshoot,
// 0/0 in occasional coefficient computations).  Rather than chase every
// producer, we sanitize at every state-mutating boundary: per-voice DSP
// output, per-voice IIR state, master mix input, post-EQ state, post-
// compressor state, and the final output.  Each guard zeros the offending
// sample (or resets the IIR state to a safe value) and emits a rate-limited
// log so the producer can still be tracked down.

/// Sanitize a deinterleaved float buffer in-place.  Replaces any non-finite
/// sample with 0.  Returns true if at least one sample was replaced.
/// Hot path — called once per buffer per audio callback.
static inline bool audioSanitizeBuffer(float *buf, std::size_t n) {
    if (!buf) return false;
    bool anyBad = false;
    for (std::size_t i = 0; i < n; ++i) {
        if (!std::isfinite(buf[i])) {
            buf[i] = 0.0f;
            anyBad = true;
        }
    }
    return anyBad;
}

/// Sanitize an interleaved stereo buffer in-place (length = frames*2 samples).
/// Identical to audioSanitizeBuffer; named separately for code clarity at
/// call sites that operate on interleaved data.
static inline bool audioSanitizeInterleaved(float *interleaved, std::size_t frames) {
    return audioSanitizeBuffer(interleaved, frames * 2);
}

/// Sanitize a single scalar (typically an IIR state value).  Returns the
/// sanitized value: original if finite, otherwise `fallback`.
static inline float audioSanitizeScalar(float v, float fallback = 0.0f) {
    return std::isfinite(v) ? v : fallback;
}

/// Enable flush-to-zero and denormals-are-zero on the calling thread.
/// Denormal arithmetic does not produce NaN, but it does cause severe
/// performance cliffs (10-100× slowdown) inside IIRs whose state decays
/// toward zero — which then back-pressures the audio callback and produces
/// dropouts that look indistinguishable from NaN-silenced channels.  Worth
/// enabling on every thread that runs DSP code.
///
/// macOS arm64: set FPCR bits FZ (24) and the per-input flush bit (FZ16,
/// bit 19).  x86 (Intel macOS, Linux, Windows): use the SSE MXCSR macros.
/// Both calls are idempotent; safe to invoke at the top of every audio
/// callback (cost is a single register read/write).
static inline void audioEnableDenormalFlush() {
#if defined(__aarch64__) || defined(_M_ARM64)
    uint64_t fpcr;
    __asm__ __volatile__("mrs %0, fpcr" : "=r"(fpcr));
    fpcr |= (uint64_t(1) << 24);  // FZ
    __asm__ __volatile__("msr fpcr, %0" : : "r"(fpcr));
#elif defined(__SSE__) || defined(_M_IX86) || defined(_M_X64)
    // _MM_SET_FLUSH_ZERO_MODE / _MM_SET_DENORMALS_ZERO_MODE require xmmintrin.h
    // and pmmintrin.h, which we don't include at the top of this file.  Use
    // the raw stmxcsr/ldmxcsr to avoid pulling them in.
    unsigned int mxcsr;
    __asm__ __volatile__("stmxcsr %0" : "=m"(mxcsr));
    mxcsr |= (1u << 15) | (1u << 6);  // FTZ | DAZ
    __asm__ __volatile__("ldmxcsr %0" : : "m"(mxcsr));
#endif
}

/*----------------------------------------------------*/
/*----------- Gain-staging meters --------------------*/
/*----------------------------------------------------*/
//
// One meter per major DSP stage in the master bus.  Each meter accumulates
// per-block peak (max |sample|) and per-block sum-of-squares (for RMS) into
// a running window, flushed periodically by the mix node as a [GAIN] log
// line per stage.  Clipping at the output is almost always due to one
// specific stage pushing too hot — the per-stage peak/RMS delta points
// directly at which one (e.g. EQ shelf gain too aggressive, compressor
// makeup over-compensating, master gain set above the headroom budget).
//
// dB conversion uses a -120 dB floor so silent windows don't print "-inf".
// L and R are tracked separately because asymmetric loudness is informative
// (e.g. binaural pan to far right pushes R hot while L stays quiet).
//
// StageMeter struct moved to AudioMetering.h so the per-sub-worker meters
// inside ConvolutionWorkerPool can share the same accumulator without
// dragging this struct's full definition out of the AudioService TU.

/*----------------------------------------------------*/
/*----------- Steam Audio DSP Processing Node --------*/
/*----------------------------------------------------*/

/// Forward declare the process callback for the custom miniaudio node vtable.
static void steamAudioNodeProcess(ma_node* pNode, const float** ppFramesIn,
                                   ma_uint32* pFrameCountIn,
                                   float** ppFramesOut, ma_uint32* pFrameCountOut);

// Forward declaration (full definition below, after ReflectionMixNode)
struct ConvolutionWorker;

// SteamAudioDSPNode and ActiveVoice now live in VoicePool.h. They had to
// move together because ActiveVoice embeds a SteamAudioDSPNode by value;
// VoicePool owns the voice map + handle allocator. AudioService still owns
// startup (createVoiceSource / initVoiceDSP / ma_sound_start) and teardown
// (removeVoiceSource) for each voice — only the data location moved.
//
// The process callback steamAudioNodeProcess() and the vtable below remain
// here so the audio-thread DSP path stays in this TU (alongside its
// counterpart reflectionMixNodeProcess and the ConvolutionWorker).

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

    // Listener position snapshot in engine feet (NOT meters), used only by the
    // [WET_BUS] diagnostic so the periodic log line can correlate wet-bus
    // amplitude with where the listener is standing. Written by main thread
    // each frame; read by audio thread without locking. The fields are
    // independent floats so a torn read at most produces a wrong number in a
    // log message — not worth the cost of stricter sync for a diagnostic.
    float listenerPosX = 0.0f;
    float listenerPosY = 0.0f;
    float listenerPosZ = 0.0f;

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
    std::atomic<bool> ready{false};   // true once pipeline is initialized
    std::atomic<bool> simulationRan{false};  // true after first iplSimulatorRunReflections completes
    bool nodeInitialized = false;
    ConvolutionWorker *convWorker = nullptr; // pointer to the off-thread worker

    // Reflection gain ramp — smoothly fades in reflection output to prevent
    // crackling on initial activation. Ramps from 0→1 over ~10ms (480 samples
    // at 48kHz). Once at 1.0, stays there (subsequent IR updates are handled
    // by Steam Audio's internal crossfading).
    float reflGain = 0.0f;           // current gain (0=silent, 1=full)
    float reflGainTarget = 1.0f;     // target = mixer.reflection_gain (set in initDSP)
    float reflRampRate = 1.0f / 480.0f; // per-sample increment (set in initDSP from mixer.reflection_ramp_ms)
    float directGain = 1.0f;         // dry-bus multiplier (applied during direct passthrough)
    float masterGain = 1.0f;         // post-limiter master multiplier

    // ── Wet-bus tape saturation ──
    // Soft tanh saturator applied to the summed wet bus AFTER reflGain ramp
    // and BEFORE the wet path is added to the dry stereoOut. Models analog
    // tape / phonograph compression: nearly transparent for quiet reverb,
    // harmonic coloration at moderate levels, brick-wall ceiling at loud
    // peaks. The Thief gothic-steampunk aesthetic benefits from intentional
    // analog character on reverb, and the same curve doubles as a graceful
    // clip-handler for pathological hot probes (a player inside a small
    // metal container with an ambient inside) without polluting the dry
    // signal.
    //
    // Curve: out = tanh(in * drive) / drive
    //   - drive=1.0 (default): nearly linear small-signal, soft ceiling at ±1
    //   - drive=2-4: audible tape-like warmth, peak capped at 1/drive
    //   - drive=5-10: phonograph character, heavy compression and harmonics
    // Unity small-signal gain: tanh(x*d)/d ≈ x for small x. Peak ceiling: 1/d.
    bool  wetSaturationEnabled = false;
    float wetSaturationDrive   = 1.0f;
    // Scratch buffers used to sum sub-worker wet output before saturation,
    // so the soft tanh sees the full wet contribution (post reflGain ramp)
    // rather than per-sub-worker contributions. Sized at frameSize in
    // initReflectionPipeline.
    std::vector<float> wetScratchL;
    std::vector<float> wetScratchR;

    // ── Master bus DSP chain ──
    // Processing order: Low-shelf EQ → Compressor → Soft Limiter
    // Each stage can be independently enabled/disabled via config.

    // Soft limiter — replaces hard clamp with smooth tanh saturation.
    // Transparent below the threshold knee, smoothly curves to ±1.0 above.
    bool limiterEnabled = true;
    float limiterKnee = 0.8f;  // linear threshold where limiting begins

    // Master bus compressor — tames transients to preserve dynamic detail.
    // Uses linked stereo peak envelope follower with configurable attack/release.
    bool compressorEnabled = true;
    float compThresholdDb = -15.0f;  // dBFS threshold
    float compThresholdLin = 0.178f; // linear threshold (computed from dB in initDSP)
    float compRatio = 3.0f;         // compression ratio (3:1)
    float compAttackMs = 10.0f;     // attack time in ms (config-driven; pushed before initDSP)
    float compReleaseMs = 250.0f;   // release time in ms (config-driven; pushed before initDSP)
    float compAttackCoeff = 0.0f;   // per-sample attack smoothing coefficient
    float compReleaseCoeff = 0.0f;  // per-sample release smoothing coefficient
    float compMakeupDb = 0.0f;      // makeup gain in dB (auto-computed from threshold/ratio)
    float compMakeupLin = 1.0f;     // linear makeup gain
    float compEnvelope = 0.0f;      // envelope follower state (shared stereo)

    // Low-shelf EQ — adds weight to the soundscape (+3dB at 120Hz default).
    // Uses biquad filter (Audio EQ Cookbook, Robert Bristow-Johnson).
    bool eqEnabled = true;
    float eqFreq = 120.0f;       // center frequency (Hz)
    float eqGainDb = 3.0f;       // shelf gain (dB)
    float eqQ = 0.707f;          // filter Q (Butterworth)
    // Biquad coefficients (computed at init from freq/gain/Q/sampleRate)
    float eqB0 = 1.0f, eqB1 = 0.0f, eqB2 = 0.0f;
    float eqA1 = 0.0f, eqA2 = 0.0f;
    // Per-channel biquad state (Direct Form II Transposed)
    float eqZ1L = 0.0f, eqZ2L = 0.0f;  // left channel
    float eqZ1R = 0.0f, eqZ2R = 0.0f;  // right channel

    // NaN/Inf guard counters for the master DSP chain (audio thread).
    // Each location that sanitizes non-finite samples or resets a stuck
    // IIR state increments its own counter; we log the first few
    // occurrences so the producer can be tracked.
    std::atomic<uint32_t> nanCountInput{0};   // input from per-voice dry sum
    std::atomic<uint32_t> nanCountWet{0};     // input from reflection wet bus
    std::atomic<uint32_t> nanCountEq{0};      // post-EQ biquad state reset
    std::atomic<uint32_t> nanCountComp{0};    // compressor envelope reset
    std::atomic<uint32_t> nanCountOutput{0};  // final output stage clamp

    // Gain-staging meters — one per stage of the master DSP chain.
    // Read & written from the audio thread only (no atomic — single writer,
    // single reader on the same thread).  Flushed every ~1.4 s as a
    // [GAIN] log block (see kGainLogIntervalCallbacks).
    StageMeter meterDryIn;     // post per-voice sum, before wet add
    StageMeter meterPostWet;   // after wet bus add
    StageMeter meterPostEq;    // after EQ biquad
    StageMeter meterPostComp;  // after compressor
    StageMeter meterPostGain;  // after master gain (pre-limiter)
    StageMeter meterPostLim;   // after limiter (== device input)
    // Per-block totals over the same window.
    uint64_t   meterClipCount = 0;    // |x| > 0.99 at FINAL output (true clip danger)
    uint64_t   meterLimitHits = 0;    // samples that hit the soft-limiter knee (> knee or < -knee)
    uint64_t   meterTotalSamples = 0; // denominator for hit-rate percent
    uint64_t   meterBlocksSinceLog = 0;

    // Ducking system — ducks ambient sounds when SFX voices are active.
    // Disabled by default. When enabled, ambient voice volumes are multiplied
    // by duckGain, which smoothly transitions based on SFX activity.
    bool duckingEnabled = false;
    float duckAmount = 0.5f;       // target ambient volume multiplier during SFX
    float duckAttackMs = 50.0f;    // ducking attack in ms (config-driven; pushed before initDSP)
    float duckReleaseMs = 500.0f;  // ducking release in ms (config-driven; pushed before initDSP)
    float duckAttackCoeff = 0.0f;  // per-frame attack coefficient
    float duckReleaseCoeff = 0.0f; // per-frame release coefficient
    float duckGain = 1.0f;         // current ducking multiplier (1.0 = no duck)

    /// Initialize DSP chain coefficients from sample rate. Must be called after
    /// frameSize is set and before the first audio callback.
    void initDSP(uint32_t sampleRate) {
        // Compressor attack/release coefficients (per-sample exponential).
        // Times come from config (default 10ms attack, 250ms release).
        float attackTimeSec = compAttackMs * 0.001f;
        float releaseTimeSec = compReleaseMs * 0.001f;
        compAttackCoeff = 1.0f - std::exp(-1.0f / (static_cast<float>(sampleRate) * attackTimeSec));
        compReleaseCoeff = 1.0f - std::exp(-1.0f / (static_cast<float>(sampleRate) * releaseTimeSec));

        // Auto makeup gain: compensate for the gain reduction at threshold.
        // At threshold, gain reduction = (threshold - threshold) * (1 - 1/ratio) = 0.
        // At 0 dBFS, gain reduction = (0 - threshold) * (1 - 1/ratio).
        // We add half that reduction back as makeup to keep perceived loudness.
        compThresholdLin = std::pow(10.0f, compThresholdDb / 20.0f);
        compMakeupDb = -compThresholdDb * (1.0f - 1.0f / compRatio) * 0.5f;
        compMakeupLin = std::pow(10.0f, compMakeupDb / 20.0f);

        // Low-shelf EQ biquad coefficients (Audio EQ Cookbook, R. Bristow-Johnson)
        float w0 = 2.0f * 3.14159265f * eqFreq / static_cast<float>(sampleRate);
        float A = std::pow(10.0f, eqGainDb / 40.0f);
        float alpha = std::sin(w0) / (2.0f * eqQ);
        float cosW0 = std::cos(w0);
        float sqrtA2alpha = 2.0f * std::sqrt(A) * alpha;

        float b0 = A * ((A + 1.0f) - (A - 1.0f) * cosW0 + sqrtA2alpha);
        float b1 = 2.0f * A * ((A - 1.0f) - (A + 1.0f) * cosW0);
        float b2 = A * ((A + 1.0f) - (A - 1.0f) * cosW0 - sqrtA2alpha);
        float a0 = (A + 1.0f) + (A - 1.0f) * cosW0 + sqrtA2alpha;
        float a1 = -2.0f * ((A - 1.0f) + (A + 1.0f) * cosW0);
        float a2 = (A + 1.0f) + (A - 1.0f) * cosW0 - sqrtA2alpha;

        // Normalize by a0
        float invA0 = 1.0f / a0;
        eqB0 = b0 * invA0;
        eqB1 = b1 * invA0;
        eqB2 = b2 * invA0;
        eqA1 = a1 * invA0;
        eqA2 = a2 * invA0;

        // Ducking coefficients (per-frame, not per-sample — called from loopStep).
        // Times come from config (default 50ms attack, 500ms release).
        // Per-frame coefficient assumes a ~60fps loop step.
        duckAttackCoeff = 1.0f - std::exp(-1.0f / (60.0f * duckAttackMs * 0.001f));
        duckReleaseCoeff = 1.0f - std::exp(-1.0f / (60.0f * duckReleaseMs * 0.001f));
    }
};

// ConvolutionSubWorker, ConvolutionWorker, and the drainConvolutionWorker
// helper that ~ActiveVoice calls all live in ConvolutionWorkerPool.h/.cpp
// now. The reflection mix node and the per-voice DSP node use the worker
// via the ConvolutionWorker* pointer threaded in during init — see
// node->convWorker / dsp.convWorker on the audio thread.

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

// ── Engine ↔ Steam Audio unit conversion ──
// The Dark Engine convention is 1 world unit = 1 foot (player is 6 units tall,
// gravity is 32 units/s² ≈ 9.8 m/s² in feet). Steam Audio's IPL API has no
// "scale" knob: distances are interpreted as METERS by its physics models —
// air absorption is computed per-meter, IRs scale with path length, distance
// attenuation curves treat the input value as meters. Feeding raw feet makes
// the virtual scene look ~3.28× larger than reality, with over-attenuated
// highs, exaggerated reverb tails, and wrong reflection delays.
//
// Fix: every position, geometry vertex, and distance-shaped parameter that
// crosses into the IPL API gets scaled by kFeetToMeters. Scalars that stay
// inside the engine (our portal-graph distances, effectiveDistance, etc.)
// remain in feet — the conversion is strictly at the IPL boundary.
//
// The dimensionless quantities (occlusion sample counts, ray counts, material
// absorption coefficients, frequencies in Hz, durations in seconds) are
// untouched: they were never affected by the unit mismatch.
constexpr float kFeetToMeters = 0.3048f;
constexpr float kMetersToFeet = 1.0f / kFeetToMeters;

// ── Engine ↔ Steam Audio coordinate-system bridge ──
//
// Dark Engine uses a right-handed Z-up system: +X right, +Y forward, +Z up.
// At yaw=0 the listener faces engine +X (this is how the camera basis is
// built — see `Vector3 ahead(cosY*cosP, sinY*cosP, sinP)`).
//
// Steam Audio uses a right-handed Y-up system (phonon.h L221): +X right,
// +Y up, -Z ahead.  At the IPL default orientation (right=+X, up=+Y,
// ahead=-Z), the listener faces IPL -Z.
//
// We choose the mapping so the engine's neutral facing direction (yaw=0,
// engine +X) lands on the IPL neutral facing direction (-Z).  That keeps
// the listener at IPL's "easy case" orientation when the player has not
// yawed and avoids loading every ambisonics decode with a non-identity
// SH-field rotation.  The mapping:
//
//   engine +X (yaw=0 ahead) → IPL -Z (IPL ahead)
//   engine +Y (engine fwd)  → IPL -X (IPL left)
//   engine +Z (up)          → IPL +Y (up)
//
// In coordinate form: engine (x, y, z) → IPL (-y, z, -x).  Determinant
// of the rotation matrix is +1 (handedness preserved).
//
// All other IPL operations (BVH ray queries, reflection sim, occlusion)
// are agnostic about the world up direction as long as listener, sources,
// and mesh share a frame.  Probe generation with UNIFORMFLOOR is NOT
// agnostic: it raycasts along its own -Y axis to find floors, so the
// transformed scene MUST be Y-up.  Everything that crosses the IPL
// boundary goes through these helpers; distances/scalars are unchanged.
inline IPLVector3 engineToIplPos(const Vector3 &p) {
    return { -p.y * kFeetToMeters,
              p.z * kFeetToMeters,
             -p.x * kFeetToMeters };
}
inline IPLVector3 engineToIplDir(const Vector3 &d) {
    // Directions skip the metric scale; basis vectors must remain unit-length.
    return { -d.y, d.z, -d.x };
}
inline Vector3 iplToEnginePos(const IPLVector3 &p) {
    return Vector3(-p.z * kMetersToFeet,
                   -p.x * kMetersToFeet,
                    p.y * kMetersToFeet);
}

// ── Audio-thread tunables ──
// Namespace-scope atomics published by AudioService setters and read on the
// audio thread. These belong to per-voice processing paths that don't have
// direct access to AudioService members; using atomics avoids data races
// even though the values are typically only set once at startup from
// RenderConfig.
//   sHrtfInterpolation: 0 = nearest, 1 = bilinear
//   sDistanceModel:     0 = default, 1 = inverse_distance
//
// These have external linkage so AudioDSPChain.cpp's publish helper can
// write to them via extern declarations; the audio callback in this TU
// continues to read them with no extra indirection.
std::atomic<int>   sHrtfInterpolation{1};
std::atomic<float> sSpatialBlend{1.0f};
std::atomic<float> sDoorLpfOpenHz{20000.0f};
std::atomic<float> sDoorLpfBlockedHz{800.0f};
std::atomic<float> sPropMinAttenuation{0.001f};
std::atomic<int>   sDistanceModel{0};
/// Engine sample rate published for audio-thread DSP that needs it (door LPF, etc.)
static std::atomic<uint32_t> sEngineSampleRate{48000};

/// Runtime cap on the number of simultaneous sub-source slots assigned per
/// voice. Independent of mPropMaxPaths (which caps how many paths the BFS
/// keeps); the effective per-voice cap is min(mPropMaxPaths, sMaxSubSources).
/// Phase 4 default = 4. Per-path occlusion via per-slot IPLSource +
/// iplSimulatorRunDirect makes higher path counts physically meaningful
/// (each path contributes its own distance attenuation, air absorption,
/// volumetric occlusion, and transmission). Clamped against kMaxSubSources
/// at the slot-assignment call site.
static std::atomic<uint32_t> sMaxSubSources{4};

// ── Steam Audio per-voice gain-staging meters ──
//
// Aggregated across ALL active per-voice DSP nodes (so a single hot voice
// shows up in the max/peak even if other voices are quiet).  Written by
// the audio thread inside steamAudioNodeProcess — miniaudio runs the
// per-voice callbacks serially on the audio thread, so plain non-atomic
// fields are safe.  Read & reset by the master mix node every [GAIN]
// window (same thread, same callback chain).
//
// Resonance in the iplDirectEffect 3-band EQ shows up as a spike in
// sSaMeterDirectOut peak/RMS *above* sSaMeterDirectIn.  Resonance in the
// HRTF binaural is rare but shows up the same way at sSaMeterBinaural.
// A spike at sSaMeterPostLpf above sSaMeterBinaural means the door-LPF
// IIR is misbehaving (state corruption, or a degenerate cutoff value
// turning the 1-pole filter into a resonator).
static StageMeter sSaMeterDirectIn;   // mono input to iplDirectEffectApply
static StageMeter sSaMeterDirectOut;  // mono output of iplDirectEffectApply
static StageMeter sSaMeterBinaural;   // stereo output of iplBinauralEffectApply
static StageMeter sSaMeterPostLpf;    // stereo post per-voice LPF + portal atten

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
        AUDIO_LOG( "[RECORD] Failed to open %s\n", path.c_str());
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

    AUDIO_LOG( "[RECORD] Started: %s (48kHz stereo float32)\n", path.c_str());
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
    AUDIO_LOG( "[RECORD] Stopped: %u samples (%.1f seconds)\n",
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
// Updated only when audio_log profiling is enabled — see AudioLog.h
// (gAudioLogVerbose gates both textual logging and these perf counters).
// Each producer reads the flag once per callback / loop iteration into a
// local, then branches per write site to avoid 11 atomic RMWs in the hot
// path when profiling is off.  Readers (the periodic dumper) always run;
// they'll just observe stale 0s when profiling was off.  We deliberately
// do NOT zero counters on toggle-off — that would race with the audio
// thread.
static std::atomic<float> sPerVoicePeakUs{0.0f};    // peak per-voice DSP time (µs)
static std::atomic<float> sMixNodePeakUs{0.0f};      // peak global mix node time (µs)
static std::atomic<float> sTotalCallbackPeakUs{0.0f}; // peak total audio callback time (µs)
static std::atomic<int>   sPerVoiceCallCount{0};      // voices processed in last period
static std::atomic<float> sCommitPeakMs{0.0f};         // peak iplSimulatorCommit time (ms)

// ── Main thread + sim worker profiling ──
static std::atomic<float> sLoopStepPeakMs{0.0f};      // peak loopStep total time (ms)
static std::atomic<float> sDirectSimPeakMs{0.0f};     // peak iplSimulatorRunDirect time (ms)
// sReflSimPeakMs and sReflFramesRun moved to ReflectionSimulator.cpp (the
// only thread that writes them). Declared extern here so the periodic dump
// can still read+exchange them.
extern std::atomic<float> sReflSimPeakMs;
extern std::atomic<int>   sReflFramesRun;
static std::atomic<int>   sPortalRoutingTotalUs{0};     // accumulated portal routing time per dump (µs, int)
static std::atomic<int>   sPortalRoutingCount{0};      // portal routing calls per dump
static std::atomic<int>   sVoicesCreated{0};           // voices started since last dump
static std::atomic<int>   sVoicesDestroyed{0};         // voices cleaned up since last dump

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
    // Read the profiling flag once per callback. When false, skip all the
    // chrono captures and atomic RMWs that would otherwise update the perf
    // counters at the bottom of this function.
    const bool profOn = ::Darkness::gAudioLogVerbose;
    auto t0 = profOn ? std::chrono::steady_clock::now()
                     : std::chrono::steady_clock::time_point{};
    // Keep denormals from creeping into IIR state across many small samples —
    // a flushed-zero subnormal is mathematically the same as zero but avoids
    // the 10-100× microcode slowdown that can starve the audio callback.
    audioEnableDenormalFlush();
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

    // Recover IIR state from any non-finite values carried over from the
    // previous callback.  Without this, a single bad sample anywhere in the
    // pipeline could silence this voice for the rest of its lifetime —
    // door-LPF state in particular is a 1-pole IIR that NaN-locks the moment
    // the state goes bad.  Cheap (six scalar checks per voice per callback)
    // and worth the worst-case one-callback discontinuity vs. permanent
    // silence.
    {
        // Per-voice WET-bus LPF state (the DRY LPF state lives per-slot
        // in SubSource and is sanitized inside the slot loop).
        bool badLpf = !std::isfinite(node->reflSendLpfState);
        bool badRamp = !std::isfinite(node->currentPortalAtten)
                    || !std::isfinite(node->currentDoorAlpha);
        if (badLpf) {
            node->reflSendLpfState = 0.0f;
            uint32_t n = node->nanCountLpf.fetch_add(1, std::memory_order_relaxed);
            if (n < 4) {
                AUDIO_LOG("[NAN_GUARD] node=%p reflSendLpfState reset to 0 "
                          "(occurrence %u)\n",
                          static_cast<void*>(node), n + 1);
            }
        }
        if (badRamp) {
            // Reset to safe defaults: full attenuation passthrough, open door.
            node->currentPortalAtten = 1.0f;
            node->currentDoorAlpha   = 1.0f;
            uint32_t n = node->nanCountRamp.fetch_add(1, std::memory_order_relaxed);
            if (n < 4) {
                AUDIO_LOG("[NAN_GUARD] node=%p portalAtten/doorAlpha ramp "
                          "reset to passthrough (occurrence %u)\n",
                          static_cast<void*>(node), n + 1);
            }
        }
    }

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
    // Architecture: apply IPLDirectEffect (frequency-dependent 3-band attenuation
    // for distance, air absorption, occlusion, transmission) to mono, then run
    // binaural HRTF for spatialization. Portal routing is a separate post-HRTF
    // scalar multiply (geometric, frequency-independent).
    //
    // Level 0: direct effect + binaural (full pipeline)
    // Level 1: direct attenuation only (no binaural — mono passthrough)
    // Level 2: binaural only (no direct attenuation)
    // ── Multi-path slot pipeline gates ──
    // Phase 2: the audio callback iterates non-Free sub-source slots and
    // sums each slot's binaural output into stereoL/R. The legacy single
    // direct/binaural effects on SteamAudioDSPNode are gone; each slot
    // carries its own pair (pre-allocated in initVoiceDSP). Slot 0 is
    // the only slot that's normally ACTIVE under the Phase 2 cap=1
    // policy, so steady-state output equals the legacy single-source
    // pipeline. Transition frames briefly see slot 0 DRAINING with a
    // second slot ACTIVE; both run their effects, gain-snap (Phase 2)
    // or per-sample crossfade (Phase 3) produces clickless handover.
    bool runBinaural = (bypassLevel == 0 || bypassLevel == 2)
                       && node->subSources[0].binauralEffect;
    bool runAtten = (bypassLevel == 0 || bypassLevel == 1);

    // ── Memory-ordering investigation ──
    // Snapshot directParams as the audio thread sees it RIGHT NOW, before
    // any effect runs.  We know what the main thread wrote last:
    //   • player-emitted voices: distanceAttenuation = 1.0
    //   • everything else: distanceAttenuation = IPL-computed value (varies)
    // If the audio thread's read disagrees with the main thread's last
    // write, we have a memory-ordering issue — directParams is a plain
    // struct with no atomic / barrier protection.  Both main-thread writes
    // (struct copy + override) and audio-thread reads could be reordered
    // by the compiler or CPU.
    //
    // Logs on the first 5 callbacks of each voice — enough to see the
    // value evolve as the main thread catches up.  Audio-thread address
    // included so we can correlate logs with specific voices (see VOICE_PEAK
    // for the human-readable name; both logs share the node ptr).
    {
        int cb = node->lifetimeFrameCount.load(std::memory_order_relaxed);
        if (cb < 5) {
            float seen = node->directParams.distanceAttenuation;
            float aa0  = node->directParams.airAbsorption[0];
            float aa1  = node->directParams.airAbsorption[1];
            float aa2  = node->directParams.airAbsorption[2];
            AUDIO_LOG("[DPARAM_READ] node=%p cb=%d skipAtten=%d distAtt=%.4f "
                      "airAbs=(%.3f,%.3f,%.3f) flags=0x%x\n",
                      static_cast<void*>(node), cb,
                      node->skipAttenuation ? 1 : 0,
                      seen, aa0, aa1, aa2,
                      static_cast<unsigned>(node->directParams.flags));
        }
    }

    // ─────────────────────────────────────────────────────────────────
    // Per-slot dry-path pipeline (Phase 3a)
    //
    // Each sub-source slot runs its own directEffect → binaural → LPF
    // chain and accumulates into voice-level stereoL/R with per-sample
    // gain ramping. Slot 0 is the only ACTIVE slot under the Phase 3
    // cap=1 policy (Phase 3b will raise it). Transition frames briefly
    // see slot 0 DRAINING + slot N ACTIVE; the gain ramp + the slot's
    // own LPF state produce a clickless crossfade between paths.
    //
    // The LPF is per-slot (slot.targetDoorBlocking → α, slot.lpfStateL/R)
    // so a path through a closed door gets its OWN spectral coloration
    // independent of other paths. The per-voice door LPF that used to
    // run post-summation is gone; the post-summation pass now only
    // applies the per-voice portalAttenuation ramp (still scalar) and
    // computes the per-voice currentDoorAlpha for the reflection-send
    // bus.
    // ─────────────────────────────────────────────────────────────────
    if (runBinaural) {
        float* chL = node->stereoL.data();
        float* chR = node->stereoR.data();
        std::memset(chL, 0, frameCount * sizeof(float));
        std::memset(chR, 0, frameCount * sizeof(float));

        float* slotL = node->subSlotStereoL.data();
        float* slotR = node->subSlotStereoR.data();
        float* directOutPtr = node->directEffectOut.data();

        const float sampleRate = static_cast<float>(
            sEngineSampleRate.load(std::memory_order_relaxed));
        const float rampSamples = std::max(1.0f, sampleRate * 0.010f);
        const float maxStep = 1.0f / rampSamples;

        // Door-LPF cutoff formula constants — same for every slot in
        // this callback (depend on global config + sample rate, not
        // per-slot blocking).
        const float openHz    = sDoorLpfOpenHz.load(std::memory_order_relaxed);
        const float blockedHz = sDoorLpfBlockedHz.load(std::memory_order_relaxed);
        const float nyquist   = 0.5f * sampleRate;
        const float hzRatio   = blockedHz / openHz;

        // Mono input peak (independent of slots — same input).
        float monoInPeak = 0.0f;
        for (ma_uint32 i = 0; i < frameCount; ++i) {
            float a = std::fabs(mono[i]);
            if (a > monoInPeak) monoInPeak = a;
        }
        if (runAtten && !node->skipAttenuation) {
            sSaMeterDirectIn.measureMono(mono, frameCount);
        }

        float monoOutPeak = 0.0f;
        bool anyDirectRan = false;

        for (int slotIdx = 0; slotIdx < kMaxSubSources; ++slotIdx) {
            SubSource& slot = node->subSources[slotIdx];
            if (slot.state == SubSourceState::Free) continue;
            if (!slot.binauralEffect) continue;

            // Snap currentDir to targetDir at callback boundary.
            // Steam Audio's internal bilinear interpolation handles
            // smoothing between this callback's direction and the
            // previous one; we don't ramp direction per sample.
            slot.currentDir = slot.targetDir;
            if (!std::isfinite(slot.currentDir.x)
                || !std::isfinite(slot.currentDir.y)
                || !std::isfinite(slot.currentDir.z)) {
                slot.currentDir = {0.0f, 0.0f, -1.0f};
            }

            // ── Per-slot direct effect (Phase 4: per-path params) ──
            //
            // Each slot's directEffect runs with its OWN
            // IPLDirectEffectParams from this slot's IPLSource —
            // populated by iplSimulatorRunDirect using the slot's
            // path virtualPosition. So a slot routed through an open
            // doorway and a slot routed through a partially-walled
            // path get genuinely distinct distance attenuation, air
            // absorption, occlusion, and transmission spectra. The
            // legacy single-source node->directParams (sourced from
            // voice->directSource at the centroid prop.virtualPosition)
            // is still maintained for the reflection-send path below
            // but is no longer consumed here.
            float* binauralMono = mono;
            if (runAtten && !node->skipAttenuation && slot.directEffect) {
                float* directInPtr = mono;
                IPLAudioBuffer directIn{};
                directIn.numChannels = 1;
                directIn.numSamples = static_cast<IPLint32>(frameCount);
                directIn.data = &directInPtr;
                IPLAudioBuffer directOut{};
                directOut.numChannels = 1;
                directOut.numSamples = static_cast<IPLint32>(frameCount);
                directOut.data = &directOutPtr;

                iplDirectEffectApply(slot.directEffect,
                                     &slot.targetDirectParams,
                                     &directIn, &directOut);

                if (audioSanitizeBuffer(directOutPtr, frameCount)) {
                    uint32_t n = node->nanCountDirect.fetch_add(
                        1, std::memory_order_relaxed);
                    if (n < 4) {
                        AUDIO_LOG("[NAN_GUARD] node=%p slot=%d directEffect "
                                  "output had non-finite samples "
                                  "(occurrence %u) "
                                  "distAtt=%.4f occl=%.4f trans=(%.3f,%.3f,%.3f) "
                                  "airAbs=(%.3f,%.3f,%.3f)\n",
                                  static_cast<void*>(node), slotIdx, n + 1,
                                  slot.targetDirectParams.distanceAttenuation,
                                  slot.targetDirectParams.occlusion,
                                  slot.targetDirectParams.transmission[0],
                                  slot.targetDirectParams.transmission[1],
                                  slot.targetDirectParams.transmission[2],
                                  slot.targetDirectParams.airAbsorption[0],
                                  slot.targetDirectParams.airAbsorption[1],
                                  slot.targetDirectParams.airAbsorption[2]);
                    }
                }
                binauralMono = directOutPtr;
                anyDirectRan = true;

                for (ma_uint32 i = 0; i < frameCount; ++i) {
                    float a = std::fabs(directOutPtr[i]);
                    if (a > monoOutPeak) monoOutPeak = a;
                }
            }

            // ── Per-slot binaural (uses slot.currentDir). ──
            float* binauralMonoPtr = binauralMono;
            IPLAudioBuffer binauralIn{};
            binauralIn.numChannels = 1;
            binauralIn.numSamples = static_cast<IPLint32>(frameCount);
            binauralIn.data = &binauralMonoPtr;

            float* slotChans[2] = {slotL, slotR};
            IPLAudioBuffer slotOut{};
            slotOut.numChannels = 2;
            slotOut.numSamples = static_cast<IPLint32>(frameCount);
            slotOut.data = slotChans;

            IPLBinauralEffectParams binParams{};
            binParams.direction = slot.currentDir;
            binParams.interpolation =
                (sHrtfInterpolation.load(std::memory_order_relaxed) == 0)
                    ? IPL_HRTFINTERPOLATION_NEAREST
                    : IPL_HRTFINTERPOLATION_BILINEAR;
            binParams.spatialBlend =
                sSpatialBlend.load(std::memory_order_relaxed)
                * node->spatialBlendOverride.load(std::memory_order_relaxed);
            binParams.hrtf = node->hrtf;
            binParams.peakDelays = nullptr;

            iplBinauralEffectApply(slot.binauralEffect, &binParams,
                                   &binauralIn, &slotOut);

            bool badL = audioSanitizeBuffer(slotL, frameCount);
            bool badR = audioSanitizeBuffer(slotR, frameCount);
            if (badL || badR) {
                uint32_t n = node->nanCountBinaural.fetch_add(
                    1, std::memory_order_relaxed);
                if (n < 4) {
                    AUDIO_LOG("[NAN_GUARD] node=%p slot=%d binaural output had "
                              "non-finite samples L=%d R=%d (occurrence %u) "
                              "dir=(%.3f,%.3f,%.3f) spatialBlend=%.3f\n",
                              static_cast<void*>(node), slotIdx,
                              badL ? 1 : 0, badR ? 1 : 0, n + 1,
                              binParams.direction.x, binParams.direction.y,
                              binParams.direction.z, binParams.spatialBlend);
                }
            }

            // ── Per-slot LPF α target (Maekawa-ish 1-pole IIR
            // coefficient derived from slot.targetDoorBlocking). One
            // pow() per slot per callback; cheap. ──
            float blocking = audioSanitizeScalar(slot.targetDoorBlocking, 0.0f);
            if      (blocking < 0.0f) blocking = 0.0f;
            else if (blocking > 1.0f) blocking = 1.0f;
            float cutoff = openHz * std::pow(hzRatio, blocking);
            if      (cutoff >= 0.99f * nyquist) cutoff = 0.99f * nyquist;
            else if (cutoff < 1.0f)              cutoff = 1.0f;
            float omega = std::tan(3.14159265f * cutoff / sampleRate);
            float alphaTarget = audioSanitizeScalar(omega / (1.0f + omega), 1.0f);
            if      (alphaTarget < 0.0f) alphaTarget = 0.0f;
            else if (alphaTarget > 1.0f) alphaTarget = 1.0f;

            // Defence in depth: recover any non-finite ramped state.
            if (!std::isfinite(slot.currentGain))      slot.currentGain      = 0.0f;
            if (!std::isfinite(slot.currentDoorAlpha)) slot.currentDoorAlpha = 1.0f;
            if (!std::isfinite(slot.lpfStateL))        slot.lpfStateL        = 0.0f;
            if (!std::isfinite(slot.lpfStateR))        slot.lpfStateR        = 0.0f;

            // ── Per-sample ramp + LPF + accumulate into chL/chR. ──
            float curGain  = slot.currentGain;
            float curAlpha = slot.currentDoorAlpha;
            const float targetGain = slot.targetGain;
            float lpfL = slot.lpfStateL;
            float lpfR = slot.lpfStateR;

            for (ma_uint32 i = 0; i < frameCount; ++i) {
                // Slew gain toward target (~10 ms slew rate). The slot
                // ramps up from 0 on cold allocation and down to 0 on
                // drain — that's the click-prevention crossfade between
                // paths during transitions.
                float dg = targetGain - curGain;
                if      (dg >  maxStep) dg =  maxStep;
                else if (dg < -maxStep) dg = -maxStep;
                curGain += dg;

                // Slew door-LPF α toward target.
                float dAlpha = alphaTarget - curAlpha;
                if      (dAlpha >  maxStep) dAlpha =  maxStep;
                else if (dAlpha < -maxStep) dAlpha = -maxStep;
                curAlpha += dAlpha;

                // Per-slot 1-pole IIR door LPF.
                lpfL += curAlpha * (slotL[i] - lpfL);
                lpfR += curAlpha * (slotR[i] - lpfR);

                // Accumulate × per-slot gain. Open-door, full-gain slot
                // contributes its full binaural output; draining slot
                // ramps to zero contribution; multi-active slots in
                // Phase 3b sum coherently.
                chL[i] += lpfL * curGain;
                chR[i] += lpfR * curGain;
            }

            slot.currentGain      = curGain;
            slot.currentDoorAlpha = curAlpha;
            slot.lpfStateL        = lpfL;
            slot.lpfStateR        = lpfR;
        }

        // Per-stage peak diagnostics (aggregated across slots — same
        // semantics as legacy [VOICE_PEAK] reads).
        if (anyDirectRan) {
            sSaMeterDirectOut.measureMono(directOutPtr, frameCount);
            float prev = node->monoInPeak.load(std::memory_order_relaxed);
            while (monoInPeak > prev && !node->monoInPeak.compare_exchange_weak(
                       prev, monoInPeak, std::memory_order_relaxed)) {}
            prev = node->monoOutPeak.load(std::memory_order_relaxed);
            while (monoOutPeak > prev && !node->monoOutPeak.compare_exchange_weak(
                       prev, monoOutPeak, std::memory_order_relaxed)) {}

            float atten = node->directParams.distanceAttenuation
                        * node->directParams.occlusion
                        * node->portalAttenuation;
            float minAtten = sPropMinAttenuation.load(std::memory_order_relaxed);
            if (atten < minAtten) atten = minAtten;
            node->lastAtten.store(atten, std::memory_order_relaxed);
        }

        // HRTF headroom trim — applied to summed output (linearly
        // commutes with summation, so this is equivalent to trimming
        // each slot's binaural output independently).
        constexpr float kHrtfTrim = 0.7079458f;  // 10^(-3/20) = -3 dBFS
        for (ma_uint32 i = 0; i < frameCount; ++i) {
            chL[i] *= kHrtfTrim;
            chR[i] *= kHrtfTrim;
        }
        sSaMeterBinaural.measureDeinterleaved(chL, chR, frameCount);

        // ── Per-voice ramp updates for reflection-send (Phase 4) ──
        //
        // With per-slot IPLSources providing per-path distance
        // attenuation and per-slot LPF providing per-path door
        // coloration, the dry chL/chR have already absorbed both. The
        // legacy post-summation `chL *= currentPortalAtten` and per-
        // voice door LPF would now double-attenuate, so the dry apply
        // is gone. We keep the ramps themselves running, however, so
        // node->currentPortalAtten and node->currentDoorAlpha stay in
        // sync with their targets — the reflection-send block below
        // still reads both for its centroid-based mono scaling and
        // wet-bus LPF coloration (the reverb tail is single-source,
        // centroid is the right summary).
        if (runAtten && !node->skipAttenuation) {
            float portalTarget = audioSanitizeScalar(node->portalAttenuation, 1.0f);
            float minAtten = sPropMinAttenuation.load(std::memory_order_relaxed);
            if (portalTarget < minAtten) portalTarget = minAtten;

            float voiceBlocking = audioSanitizeScalar(node->portalBlocking, 0.0f);
            if      (voiceBlocking < 0.0f) voiceBlocking = 0.0f;
            else if (voiceBlocking > 1.0f) voiceBlocking = 1.0f;
            float voiceCutoff = openHz * std::pow(hzRatio, voiceBlocking);
            if      (voiceCutoff >= 0.99f * nyquist) voiceCutoff = 0.99f * nyquist;
            else if (voiceCutoff < 1.0f)             voiceCutoff = 1.0f;
            float voiceOmega = std::tan(3.14159265f * voiceCutoff / sampleRate);
            float voiceAlphaTarget = audioSanitizeScalar(voiceOmega / (1.0f + voiceOmega), 1.0f);
            if      (voiceAlphaTarget < 0.0f) voiceAlphaTarget = 0.0f;
            else if (voiceAlphaTarget > 1.0f) voiceAlphaTarget = 1.0f;

            float curAtten = node->currentPortalAtten;
            float curAlpha = node->currentDoorAlpha;

            for (ma_uint32 i = 0; i < frameCount; ++i) {
                float da = portalTarget - curAtten;
                if      (da >  maxStep) da =  maxStep;
                else if (da < -maxStep) da = -maxStep;
                curAtten += da;

                float dAlpha = voiceAlphaTarget - curAlpha;
                if      (dAlpha >  maxStep) dAlpha =  maxStep;
                else if (dAlpha < -maxStep) dAlpha = -maxStep;
                curAlpha += dAlpha;
            }

            node->currentPortalAtten = curAtten;
            node->currentDoorAlpha   = curAlpha;
        }

        // Gain-staging: post per-voice LPF + portal attenuation (the
        // signal that gets summed into the dry bus).  Peak here above
        // binaural peak indicates the door-LPF IIR is resonating
        // (degenerate cutoff coefficient or stale state).  Below
        // binaural peak by a few dB is the expected behaviour with a
        // partial door close.
        sSaMeterPostLpf.measureDeinterleaved(chL, chR, frameCount);

        // Apply reflection convolution — feeds attenuated mono input through
        // per-voice convolution IR and accumulates the ambisonics result into
        // the shared mixer. We use the attenuated mono (matching the direct
        // output volume) so that reverb level is proportional to the source's
        // perceived loudness. The IR encodes reflection paths but not source
        // distance — without this, nearby sounds would have disproportionately
        // quiet reverb relative to their direct signal.
        // Snapshot mono for the off-thread convolution worker.
        // The worker processes all voice convolutions in parallel with the
        // next audio callback, writing to a double-buffered stereo output.
        if (node->reflectionsActive && node->reflectionEffect
            && node->reflectionParams.irSize > 0 && node->convWorker) {
            auto &cw = *node->convWorker;
            int w = cw.writeIdx.load(std::memory_order_relaxed);
            int slotIdx = cw.stagingCount[w]++;
            if (slotIdx < ConvolutionWorker::kMaxSlots) {
                auto &slot = cw.staging[w][slotIdx];

                // Apply distance attenuation to mono. Steam Audio's
                // direct-path occlusion (walls between source and listener)
                // is INTENTIONALLY not folded in — wall occlusion only
                // blocks the line-of-sight path; reflected energy can still
                // arrive via room bounces and shouldn't be silenced.
                //
                // Door blocking is different: a closed door is a physical
                // seal between rooms, so the reverb tail from a source on
                // the other side should be both quieter and muffled. We
                // multiply in the post-ramp portalAttenuation (volume) and
                // — further below — pre-LPF the mono signal with the same
                // door alpha used for the dry path. Both `currentPortalAtten`
                // and `currentDoorAlpha` were just updated by the dry-path
                // block above; reuse them so the wet bus tracks the dry bus
                // exactly through transitions (door swinging closed/open).
                //
                // If runAtten is false (bypassLevel == 2 — binaural-only
                // debug path), skip both. Matches the dry-path bypass.
                float reflAtten = node->directParams.distanceAttenuation;
                float reflSendAlpha = 1.0f;  // 1.0 = passthrough LPF
                if (runAtten && !node->skipAttenuation) {
                    reflAtten     *= node->currentPortalAtten;
                    reflSendAlpha  = node->currentDoorAlpha;
                }

                // Diagnostic: for footstep voices, log the convolution input
                // amplitude on the first few audio callbacks so we can confirm
                // the dry signal actually reaches Steam Audio. Bounded to ~3
                // frames per voice so the log stays readable.
                if (node->isFootstepDiag) {
                    int n = node->reflInputLogCount.fetch_add(1, std::memory_order_relaxed);
                    if (n < 3) {
                        float monoPeak = 0.0f;
                        for (ma_uint32 i = 0; i < frameCount; ++i)
                            monoPeak = std::max(monoPeak, std::fabs(mono[i]));
                        AUDIO_LOG("[FOOT_REFL_IN] frame=%d monoPeak=%.4f distAtten=%.3f "
                                  "reflAtten=%.4f scaledPeak=%.4f irSize=%d\n",
                                  n, monoPeak,
                                  node->directParams.distanceAttenuation,
                                  reflAtten, monoPeak * reflAtten,
                                  node->reflectionParams.irSize);
                    }
                }
                ma_uint32 reflFrames = static_cast<ma_uint32>(node->reflectionFrameSize);

                // 1-pole IIR LPF state for the reflection-send mono. Same
                // formula as the dry-path stereo LPF:
                //   y[n] = α·x[n] + (1-α)·y[n-1]
                // α = 1 collapses to passthrough (open door), so the
                // unconditional run is free when no door blocks the path.
                float reflLpf = node->reflSendLpfState;
                if (node->rateDivisor > 1 && !node->decimatedMono.empty()) {
                    // Decimate from device rate to reflection rate (averaging groups)
                    float *dec = node->decimatedMono.data();
                    int div = node->rateDivisor;
                    float invDiv = 1.0f / static_cast<float>(div);
                    ma_uint32 outFrames = std::min(frameCount / static_cast<ma_uint32>(div), reflFrames);
                    for (ma_uint32 i = 0; i < outFrames; ++i) {
                        float sum = 0.0f;
                        for (int d = 0; d < div; ++d)
                            sum += mono[i * div + d];
                        float in = sum * invDiv * reflAtten;
                        reflLpf += reflSendAlpha * (in - reflLpf);
                        dec[i] = reflLpf;
                    }
                    for (ma_uint32 i = outFrames; i < reflFrames; ++i)
                        dec[i] = 0.0f;
                    std::memcpy(slot.mono.data(), dec, reflFrames * sizeof(float));
                } else {
                    // Copy mono with portal attenuation + door LPF applied
                    for (ma_uint32 i = 0; i < std::min(frameCount, reflFrames); ++i) {
                        float in = mono[i] * reflAtten;
                        reflLpf += reflSendAlpha * (in - reflLpf);
                        slot.mono[i] = reflLpf;
                    }
                    for (ma_uint32 i = frameCount; i < reflFrames; ++i)
                        slot.mono[i] = 0.0f;
                }
                node->reflSendLpfState = reflLpf;

                slot.effect = node->reflectionEffect;
                slot.validityToken = node->validityToken;  // shared_ptr copy — prevents use-after-free
                slot.params = node->reflectionParams;
                slot.reflFrameSize = node->reflectionFrameSize;
                slot.active = true;
                slot.isFootstepDiag = node->isFootstepDiag;  // propagate for worker log

                // Per-voice reverb-send peak. Measures the mono buffer
                // *after* reflAtten scaling — i.e. exactly the signal
                // handed to iplReflectionEffectApply. Sampled+reset by
                // the 5 s [AMB] dump on the main thread; combined with
                // the [WET_BUS] global IR ratio, this lets us attribute
                // runaway reverb to specific voices.
                {
                    float sPeak = 0.0f;
                    for (int i = 0; i < node->reflectionFrameSize; ++i) {
                        float a = std::fabs(slot.mono[i]);
                        if (a > sPeak) sPeak = a;
                    }
                    float prev = node->reflSendPeak.load(std::memory_order_relaxed);
                    while (sPeak > prev && !node->reflSendPeak.compare_exchange_weak(
                               prev, sPeak, std::memory_order_relaxed)) {}
                }
            }
        }

        // Interleave deinterleaved stereo → miniaudio interleaved output.
        // Track per-channel peaks separately so the [DRY_BAL] diagnostic
        // below can flag asymmetric HRTF/binaural output.  An imbalance
        // here points at the dry path (binaural direction, HRTF dataset)
        // — wet-bus imbalance instead shows in the [WET_BUS] log.
        float peakOut = 0.0f;
        float peakL = 0.0f;
        float peakR = 0.0f;
        for (ma_uint32 i = 0; i < frameCount; ++i) {
            stereoOut[i * 2]     = chL[i];
            stereoOut[i * 2 + 1] = chR[i];
            float aL = std::fabs(chL[i]);
            float aR = std::fabs(chR[i]);
            if (aL > peakL) peakL = aL;
            if (aR > peakR) peakR = aR;
        }
        peakOut = std::max(peakL, peakR);
        node->peakOutput.store(peakOut, std::memory_order_relaxed);

        // Per-voice lifetime peak (atomic max).  Used by [VOICE_PEAK] log at
        // CLEANUP.  Compare-and-swap loop because std::atomic<float> doesn't
        // have fetch_max; this is uncontested in practice (only one audio
        // thread writes per voice).
        {
            int prevFrames = node->lifetimeFrameCount.fetch_add(1, std::memory_order_relaxed);
            float prevL = node->lifetimePeakL.load(std::memory_order_relaxed);
            // Snapshot direction whenever we update the L peak — gives us
            // the listener-local source direction at the moment the voice
            // was loudest.  If identical voices produce different lifetime
            // peaks AND different direction-at-peak vectors, the listener
            // is moving during voice playback and HRTF gain is varying.
            bool updated = false;
            while (peakL > prevL && !node->lifetimePeakL.compare_exchange_weak(
                       prevL, peakL, std::memory_order_relaxed)) { /* retry */ }
            if (peakL >= node->lifetimePeakL.load(std::memory_order_relaxed) - 1e-9f
                && peakL > 0.0f) {
                updated = true;
            }
            float prevR = node->lifetimePeakR.load(std::memory_order_relaxed);
            while (peakR > prevR && !node->lifetimePeakR.compare_exchange_weak(
                       prevR, peakR, std::memory_order_relaxed)) { /* retry */ }
            if (updated) {
                node->directionAtPeakX.store(node->direction.x, std::memory_order_relaxed);
                node->directionAtPeakY.store(node->direction.y, std::memory_order_relaxed);
                node->directionAtPeakZ.store(node->direction.z, std::memory_order_relaxed);
            }
            // Record first-callback peak exactly once.
            if (prevFrames == 0) {
                node->firstCallbackPeakL.store(peakL, std::memory_order_relaxed);
                node->firstCallbackPeakR.store(peakR, std::memory_order_relaxed);
            }
        }

        // Periodic dry-bus balance log (per voice).  Only fires when the
        // voice is producing audio above a noise floor; rate-limited to
        // ~once per 64 callbacks (≈1.4 s @ 1024@48k) so the spam doesn't
        // drown the rest of the log when many voices are active.
        if (peakOut > 0.001f) {
            int n = node->dryBalLogCount.fetch_add(1, std::memory_order_relaxed);
            if ((n & 0x3F) == 0) {
                float ratio = (peakL + peakR > 1e-6f)
                              ? (peakR - peakL) / (peakR + peakL)
                              : 0.0f;
                AUDIO_LOG("[DRY_BAL] peakL=%.4f peakR=%.4f balance=%+.2f "
                          "(neg=L-heavy, pos=R-heavy) dir=(%.2f,%.2f,%.2f)\n",
                          peakL, peakR, ratio,
                          node->direction.x, node->direction.y, node->direction.z);
            }
        }
    } else {
        // No binaural — bypass level 1 (direct only) or level 2 with
        // binaural effects unavailable. Run slot 0's direct effect
        // standalone (mirrors the legacy "direct then output mono"
        // shape) and emit the filtered mono as dual mono. Skipped if
        // the voice opted out of attenuation (player-emitted) or has
        // no direct effect handle.
        float* monoOut = mono;
        if (runAtten && !node->skipAttenuation
            && node->subSources[0].directEffect) {
            float* directInPtr = mono;
            IPLAudioBuffer directIn{};
            directIn.numChannels = 1;
            directIn.numSamples = static_cast<IPLint32>(frameCount);
            directIn.data = &directInPtr;
            float* directOutPtr = node->directEffectOut.data();
            IPLAudioBuffer directOut{};
            directOut.numChannels = 1;
            directOut.numSamples = static_cast<IPLint32>(frameCount);
            directOut.data = &directOutPtr;
            iplDirectEffectApply(node->subSources[0].directEffect,
                                 &node->directParams,
                                 &directIn, &directOut);
            audioSanitizeBuffer(directOutPtr, frameCount);
            monoOut = directOutPtr;
        }
        float peakOut = 0.0f;
        for (ma_uint32 i = 0; i < frameCount; ++i) {
            float s = monoOut[i];
            stereoOut[i * 2]     = s;
            stereoOut[i * 2 + 1] = s;
            float a = std::fabs(s);
            if (a > peakOut) peakOut = a;
        }
        node->peakOutput.store(peakOut, std::memory_order_relaxed);
    }

    pFrameCountIn[0] = frameCount;
    *pFrameCountOut = frameCount;

    // Profile: track peak per-voice DSP time and accumulate for total callback.
    // Skipped entirely when audio_log profiling is off — see AudioLog.h.
    if (profOn) {
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
}

/// Audio-thread callback for the global reflection mix node.
/// Receives summed direct stereo from all per-voice DSP nodes (via miniaudio bus),
/// retrieves accumulated reflection convolution output from the shared mixer,
/// decodes from ambisonics to binaural stereo, and adds to the direct signal.
static void reflectionMixNodeProcess(ma_node* pNode, const float** ppFramesIn,
                                      ma_uint32* pFrameCountIn,
                                      float** ppFramesOut, ma_uint32* pFrameCountOut)
{
    // Read the profiling flag once per callback. When false, skip all the
    // chrono captures and atomic RMWs that update the perf counters below.
    const bool profOn = ::Darkness::gAudioLogVerbose;
    auto mixT0 = profOn ? std::chrono::steady_clock::now()
                        : std::chrono::steady_clock::time_point{};
    audioEnableDenormalFlush();
    auto* node = reinterpret_cast<AudioService::ReflectionMixNode*>(pNode);
    ma_uint32 frameCount = *pFrameCountOut;

    if (pFrameCountIn[0] < frameCount)
        frameCount = pFrameCountIn[0];
    if (frameCount > static_cast<ma_uint32>(node->frameSize))
        frameCount = static_cast<ma_uint32>(node->frameSize);

    float* stereoOut = ppFramesOut[0];
    const float* stereoIn = ppFramesIn[0];

    // Always pass through the direct audio (from per-voice DSP nodes).
    // directGain scales just the dry path so the direct/indirect ratio
    // can be tuned independently of master + reflection gains.
    if (stereoIn && stereoOut && frameCount > 0) {
        float dg = node->directGain;
        if (dg == 1.0f) {
            std::memcpy(stereoOut, stereoIn, frameCount * 2 * sizeof(float));
        } else {
            for (ma_uint32 i = 0; i < frameCount * 2; ++i)
                stereoOut[i] = stereoIn[i] * dg;
        }
    }

    // Sanitize the summed dry input before any IIR-bearing stage touches it.
    // This is the last opportunity to catch a non-finite sample from any
    // per-voice DSP node before it NaN-locks the master EQ biquad — once
    // eqZ1L/eqZ2L go non-finite, every subsequent left sample is NaN for
    // the rest of the session.
    if (stereoOut && frameCount > 0) {
        if (audioSanitizeInterleaved(stereoOut, frameCount)) {
            uint32_t n = node->nanCountInput.fetch_add(1, std::memory_order_relaxed);
            if (n < 4) {
                AUDIO_LOG("[NAN_GUARD] mixNode dry input had non-finite "
                          "samples (occurrence %u)\n", n + 1);
            }
        }
        // Gain-staging stage 1: peak + RMS of the post-per-voice-DSP sum
        // (no wet bus yet, no master DSP).  This is the "starting level"
        // against which every downstream stage's contribution is judged.
        node->meterDryIn.measure(stereoOut, frameCount);
    }

    // If reflection pipeline not ready, just pass through.  Still measure
    // post-wet (== dry, since there is no wet) so the log line is uniform.
    if (!node->ready || !node->simulationRan) {
        if (stereoOut && frameCount > 0)
            node->meterPostWet.measure(stereoOut, frameCount);
        pFrameCountIn[0] = frameCount;
        *pFrameCountOut = frameCount;
        return;
    }

    // Read all sub-workers' front buffers (previous frame's decoded reverb)
    // and sum into the wet scratch buffers. The dedicated scratch path is
    // necessary so the tape saturator (below) sees the FULL summed wet bus
    // contribution rather than per-sub-worker pieces — saturating each piece
    // independently would clip transients that, summed, would cancel.
    ConvolutionWorker *cw = node->convWorker;
    float wetPeakPreSat  = 0.0f;  // diagnostic: peak |wet| before saturation
    float wetPeakL = 0.0f;        // diagnostic: peak |wet stereo| after saturation
    float wetPeakR = 0.0f;
    int   wetWorkersHit = 0;      // diagnostic: how many sub-workers actually contributed
    int   wetStaleFronts = 0;     // diagnostic: how many sub-workers had a stale front buffer
    if (cw) {
        ma_uint32 outSamples = std::min(static_cast<ma_uint32>(cw->frameSize), frameCount);

        // Clear scratch — accumulating across sub-workers below.
        if (node->wetScratchL.size() >= outSamples
            && node->wetScratchR.size() >= outSamples) {
            std::memset(node->wetScratchL.data(), 0, outSamples * sizeof(float));
            std::memset(node->wetScratchR.data(), 0, outSamples * sizeof(float));
        }

        // Per-sub-worker front-buffer freshness diagnostic: count how
        // often the mix node reads a `front` that the worker hasn't
        // flipped since the previous callback. Non-zero `stale` means
        // the worker's output write is lagging past the audio-callback
        // boundary — the wet bus repeats the previous frame, audible
        // as low-frequency amplitude modulation ("beating").
        // Note: workersReading is decremented as soon as the worker
        // finishes reading staging, well before frontIdx is flipped,
        // so the existing [CONV_DROP] log doesn't catch this case.
        static uint64_t sLastConsumedSeq[ConvolutionWorker::kMaxSlots] = {};
        for (auto &subPtr : cw->workers) {
            auto &sub = *subPtr;
            if (!sub.hasProducedOutput.load(std::memory_order_acquire)) continue;
            ++wetWorkersHit;
            int front = sub.frontIdx.load(std::memory_order_acquire);

            // Freshness: has processedSeq advanced since the last callback?
            // Indexed by sub-worker position in cw->workers; bounded by
            // kMaxSlots which is always >> reasonable worker counts.
            size_t wkIdx = (&subPtr - &cw->workers[0]);
            if (wkIdx < ConvolutionWorker::kMaxSlots) {
                uint64_t p = sub.processedSeq.load(std::memory_order_acquire);
                if (p == sLastConsumedSeq[wkIdx]) ++wetStaleFronts;
                sLastConsumedSeq[wkIdx] = p;
            }

            // Sum into scratch. Per-sample finite check on the wet buffers
            // — a NaN slipped through by the convolution worker would
            // otherwise infect the wet scratch and propagate downstream.
            // Replace bad samples with 0 silently (the wet-workersHit log
            // already surfaces wet bus health).
            //
            // No gain ramp: early reflections need the full transient
            // edge intact, so the wet bus is summed at unity. Any global
            // wet-bus level adjustment belongs downstream of this point.
            bool wetSawBad = false;
            for (ma_uint32 i = 0; i < outSamples; ++i) {
                float lRaw = sub.stereoL[front][i];
                float rRaw = sub.stereoR[front][i];
                float l = std::isfinite(lRaw) ? lRaw : (wetSawBad = true, 0.0f);
                float r = std::isfinite(rRaw) ? rRaw : (wetSawBad = true, 0.0f);
                node->wetScratchL[i] += l;
                node->wetScratchR[i] += r;
            }
            if (wetSawBad) {
                uint32_t n = node->nanCountWet.fetch_add(1, std::memory_order_relaxed);
                if (n < 4) {
                    AUDIO_LOG("[NAN_GUARD] mixNode wet sub-worker output had "
                              "non-finite samples (occurrence %u)\n", n + 1);
                }
            }
        }

        // Apply optional tape/phonograph saturation to the summed wet bus.
        // out = tanh(in * drive) / drive. Unity small-signal gain, peak
        // capped at ±(1/drive). Above the soft knee, samples are smoothly
        // compressed toward the ceiling — graceful for pathological hot
        // probes (player inside a small metal container with ambient inside)
        // and a stylistic aesthetic for the Thief tape/phonograph feel.
        // When disabled or drive==1, runs nothing — keeps the path metric-
        // equivalent to the original passthrough for benchmarking.
        if (node->wetSaturationEnabled && node->wetSaturationDrive > 1.0f
            && node->wetScratchL.size() >= outSamples) {
            float d    = node->wetSaturationDrive;
            float invD = 1.0f / d;
            for (ma_uint32 i = 0; i < outSamples; ++i) {
                float pre = std::max(std::fabs(node->wetScratchL[i]),
                                     std::fabs(node->wetScratchR[i]));
                if (pre > wetPeakPreSat) wetPeakPreSat = pre;
                node->wetScratchL[i] = std::tanh(node->wetScratchL[i] * d) * invD;
                node->wetScratchR[i] = std::tanh(node->wetScratchR[i] * d) * invD;
            }
        } else {
            // Saturator disabled — diagnostic pre/post peaks are identical.
            for (ma_uint32 i = 0; i < outSamples; ++i) {
                float pre = std::max(std::fabs(node->wetScratchL[i]),
                                     std::fabs(node->wetScratchR[i]));
                if (pre > wetPeakPreSat) wetPeakPreSat = pre;
            }
        }

        // Add saturated wet to the (already dry-loaded) stereoOut, and
        // measure post-saturation peaks for the [WET_BUS] log.
        for (ma_uint32 i = 0; i < outSamples; ++i) {
            float l = node->wetScratchL[i];
            float r = node->wetScratchR[i];
            stereoOut[i * 2]     += l;
            stereoOut[i * 2 + 1] += r;
            wetPeakL = std::max(wetPeakL, std::fabs(l));
            wetPeakR = std::max(wetPeakR, std::fabs(r));
        }
    }
    // Gain-staging stage 2: post-wet sum.  The delta from meterDryIn shows
    // how much energy the reflection bus is adding on top of the dry path.
    if (stereoOut && frameCount > 0)
        node->meterPostWet.measure(stereoOut, frameCount);

    // Diagnostic: log the summed wet bus peak ~once per second so we can see
    // whether reverb is actually reaching the output. Independent of which
    // voices are convolving — if wet peaks are non-zero, reverb exists in the
    // mix; if zero or near-zero, no reverb is being produced (workers behind,
    // IRs empty, or all voices below floor).
    {
        static std::atomic<int> sWetLogTick{0};
        // Aggregate sub-worker freshness across the 64-callback window so the
        // [WET_BUS] dump shows total stale vs fresh front-buffer reads. A
        // non-zero stale count means the convolution worker's output write
        // is lagging past at least one audio-callback boundary — the wet bus
        // is reading a `front` that hasn't been flipped since last callback,
        // so the same wet stereo content is being summed twice in a row.
        // That is the structural cause of audible LF amplitude modulation
        // ("beating") that the existing [CONV_DROP] check does NOT detect
        // (workersReading is dropped after the staging read, well before
        // the worker finishes its output write).
        static std::atomic<int> sStaleAccum{0};
        static std::atomic<int> sFreshAccum{0};
        if (cw) {
            int fresh = std::max(0, wetWorkersHit - wetStaleFronts);
            sStaleAccum.fetch_add(wetStaleFronts, std::memory_order_relaxed);
            sFreshAccum.fetch_add(fresh,          std::memory_order_relaxed);
        }
        int tick = sWetLogTick.fetch_add(1, std::memory_order_relaxed);
        if ((tick & 0x3F) == 0) {  // every ~64 callbacks ≈ 1.4s @ 1024@48k
            // Convert peak to dBFS so the dynamic range is readable (raw 0.0006 vs
            // 0.04 looks alike at a glance; -64 dB vs -28 dB does not). dB floor
            // at -120 dB so silent-bus frames don't print "-inf".
            float peakStereo = std::max(wetPeakL, wetPeakR);
            float peakDb = (peakStereo > 1e-6f) ? 20.0f * std::log10(peakStereo) : -120.0f;
            float preDb = (wetPeakPreSat > 1e-6f) ? 20.0f * std::log10(wetPeakPreSat) : -120.0f;
            float satDeltaDb = preDb - peakDb;  // 0 = passthrough, >0 = saturator working
            int staleW = sStaleAccum.exchange(0, std::memory_order_relaxed);
            int freshW = sFreshAccum.exchange(0, std::memory_order_relaxed);
            int totalW = staleW + freshW;
            float stalePct = (totalW > 0) ? (100.0f * staleW / totalW) : 0.0f;
            AUDIO_LOG("[WET_BUS] peakL=%.4f peakR=%.4f peakDb=%.1f preDb=%.1f satΔ=%.1fdB "
                      "drive=%.2f workersHit=%d stale=%d fresh=%d stale%%=%.1f "
                      "listenerPos=(%.1f,%.1f,%.1f)\n",
                      wetPeakL, wetPeakR, peakDb, preDb, satDeltaDb,
                      node->wetSaturationEnabled ? node->wetSaturationDrive : 1.0f,
                      wetWorkersHit, staleW, freshW, stalePct,
                      node->listenerPosX, node->listenerPosY, node->listenerPosZ);
        }
    }

    // Distribute voice slots to sub-workers and signal them.
    // Snapshot the listener orientation for ambisonics decode.
    if (cw) {
        cw->listenerOrientation = node->listenerOrientation;

        // Triple-buffered staging swap.
        //
        // Buffer layout (3 staging slots):
        //   • writeIdx  — audio thread is filling this one over the current callback
        //   • prevRead  — workers' most recently published target; workers either
        //                 are reading it right now, or already finished and are
        //                 idle waiting for the next signal
        //   • the third buffer — guaranteed free under normal operation, becomes
        //                 the next writeIdx
        //
        // After the swap: the buffer we just filled (`w`) is published as the
        // new read target. The third buffer becomes the new writeIdx. workers
        // are signaled. Under one-frame-behind worker lag this is still safe
        // because the new writeIdx is neither `w` (workers' new target) nor the
        // previous read buffer (slow workers' lingering target).
        //
        // Catastrophic-backlog fallback: if any worker is two-or-more signals
        // behind (frameSeq − processedSeq ≥ 2), three buffers are no longer
        // enough to cover the live read positions. Fall through to the legacy
        // drop path. In practice this only fires under sustained CONV_LAG.
        int w = cw->writeIdx.load(std::memory_order_relaxed);
        int prevRead = cw->currentReadBuf;

        bool workersTwoBehind = false;
        for (auto &subPtr : cw->workers) {
            uint64_t fs = subPtr->frameSeq.load(std::memory_order_relaxed);
            uint64_t ps = subPtr->processedSeq.load(std::memory_order_relaxed);
            if (fs >= ps + 2) { workersTwoBehind = true; break; }
        }

        if (workersTwoBehind) {
            // Catastrophic backlog — drop this frame to avoid overwriting a
            // staging buffer a still-running worker is reading. Same diagnostic
            // shape as the legacy double-buffer drop, but should be rare.
            static std::atomic<int> sFrameDropCount{0};
            static std::atomic<int> sVoicesDroppedTotal{0};
            sFrameDropCount.fetch_add(1, std::memory_order_relaxed);
            sVoicesDroppedTotal.fetch_add(cw->stagingCount[w],
                                           std::memory_order_relaxed);
            int dc = sFrameDropCount.load(std::memory_order_relaxed);
            if ((dc & 0xF) == 0) {
                int vd = sVoicesDroppedTotal.load(std::memory_order_relaxed);
                AUDIO_LOG("[CONV_DROP] frames=%d voices_lost=%d (workers ≥2 frames behind)\n",
                          dc, vd);
            }
            cw->stagingCount[w] = 0;
        } else {
            int count = cw->stagingCount[w];
            cw->readyCount.store(count, std::memory_order_relaxed);

            // Pick the third buffer: not `w` (about to become read target),
            // not `prevRead` (slow workers may still be on it). First-frame
            // bootstrap: prevRead == -1 — pick any other buffer.
            int newW;
            if (prevRead < 0) {
                newW = (w + 1) % ConvolutionWorker::kStagingBuffers;
            } else {
                newW = (0 + 1 + 2) - w - prevRead;  // the remaining index
            }
            cw->stagingCount[newW] = 0;
            cw->writeIdx.store(newW, std::memory_order_release);

            // Distribute voice slots to sub-workers (round-robin)
            int numW = cw->numWorkers;
            for (int wk = 0; wk < numW; ++wk)
                cw->workers[wk]->assignedCount = 0;
            for (int i = 0; i < count && i < ConvolutionWorker::kMaxSlots; ++i) {
                int wk = i % numW;
                auto &sub = *cw->workers[wk];
                sub.assignedSlots[sub.assignedCount++] = i;
            }

            // Publish the new read target before signaling so workers see it
            // through the frameSeq acquire barrier.
            cw->currentReadBuf = w;

            cw->workersReading.store(numW, std::memory_order_release);

            // Signal all sub-workers that new data is available
            for (auto &subPtr : cw->workers)
                subPtr->frameSeq.fetch_add(1, std::memory_order_release);
        }
    }

    // ── Master bus DSP chain ──
    // Processing order: Low-shelf EQ → Compressor → Soft Limiter
    // HRTF convolution can produce samples > 1.0 (confirmed by Steam Audio
    // issue #350). The DSP chain shapes dynamics and prevents digital clipping
    // while preserving the natural character of the soundscape.

    // Stage 1: Low-shelf EQ — adds weight to low frequencies (biquad filter).
    // Direct Form II Transposed: y[n] = b0*x[n] + z1; z1 = b1*x[n] - a1*y[n] + z2; z2 = b2*x[n] - a2*y[n]
    //
    // The biquad state (eqZ1L/eqZ2L/eqZ1R/eqZ2R) is the canonical place
    // where a single bad sample silences a channel for the rest of the
    // session — the recurrence locks NaN into the IIR.  Recover at the
    // top of the stage if state went non-finite between callbacks, and
    // recheck after to catch in-stage corruption.
    if (node->eqEnabled) {
        bool eqStateBad = !std::isfinite(node->eqZ1L) || !std::isfinite(node->eqZ2L)
                       || !std::isfinite(node->eqZ1R) || !std::isfinite(node->eqZ2R);
        if (eqStateBad) {
            node->eqZ1L = node->eqZ2L = 0.0f;
            node->eqZ1R = node->eqZ2R = 0.0f;
            uint32_t n = node->nanCountEq.fetch_add(1, std::memory_order_relaxed);
            if (n < 4) {
                AUDIO_LOG("[NAN_GUARD] mixNode EQ state pre-reset to 0 "
                          "(occurrence %u)\n", n + 1);
            }
        }
        for (ma_uint32 i = 0; i < frameCount; ++i) {
            // Left channel
            float xL = stereoOut[i * 2];
            float yL = node->eqB0 * xL + node->eqZ1L;
            node->eqZ1L = node->eqB1 * xL - node->eqA1 * yL + node->eqZ2L;
            node->eqZ2L = node->eqB2 * xL - node->eqA2 * yL;
            stereoOut[i * 2] = yL;

            // Right channel
            float xR = stereoOut[i * 2 + 1];
            float yR = node->eqB0 * xR + node->eqZ1R;
            node->eqZ1R = node->eqB1 * xR - node->eqA1 * yR + node->eqZ2R;
            node->eqZ2R = node->eqB2 * xR - node->eqA2 * yR;
            stereoOut[i * 2 + 1] = yR;
        }
        // Post-stage state check — catches a NaN introduced mid-loop
        // (e.g. by a denormal-edge sample interacting with stale state
        // that was finite but pathological).  Same recovery as above.
        if (!std::isfinite(node->eqZ1L) || !std::isfinite(node->eqZ2L)
         || !std::isfinite(node->eqZ1R) || !std::isfinite(node->eqZ2R)) {
            node->eqZ1L = node->eqZ2L = 0.0f;
            node->eqZ1R = node->eqZ2R = 0.0f;
            uint32_t n = node->nanCountEq.fetch_add(1, std::memory_order_relaxed);
            if (n < 4) {
                AUDIO_LOG("[NAN_GUARD] mixNode EQ state post-reset to 0 "
                          "(occurrence %u)\n", n + 1);
            }
            // Output may also contain non-finite samples written before
            // the state went bad — sanitize to keep the compressor input
            // clean.
            audioSanitizeInterleaved(stereoOut, frameCount);
        }
    }
    // Gain-staging stage 3: post-EQ.  Delta from post-wet shows the shelf
    // gain's contribution; if RMS jumps several dB here the EQ gain setting
    // is eating your headroom.
    if (stereoOut && frameCount > 0)
        node->meterPostEq.measure(stereoOut, frameCount);

    // Stage 2: Compressor — linked stereo peak envelope follower.
    // Tames transients to keep loud sounds (explosions, machinery) from drowning
    // out subtle ambient detail. Attack ~10ms, release ~250ms.
    //
    // Gain computation uses the linear domain to avoid per-sample log10/pow
    // (which caused 4ms mix node spikes). The compressor transfer function
    // in linear: gain = (threshold / envelope)^(1 - 1/ratio) when envelope
    // exceeds threshold. This is mathematically equivalent to the dB formulation
    // but uses a single powf call with a precomputed exponent.
    if (node->compressorEnabled) {
        // Reset stuck envelope at the start of the stage.  compEnvelope is
        // also a recurrence; a NaN here would `>` always false, never
        // recover, and zero out the gain forever.
        if (!std::isfinite(node->compEnvelope)) {
            node->compEnvelope = 0.0f;
            uint32_t n = node->nanCountComp.fetch_add(1, std::memory_order_relaxed);
            if (n < 4) {
                AUDIO_LOG("[NAN_GUARD] mixNode compressor envelope reset to 0 "
                          "(occurrence %u)\n", n + 1);
            }
        }

        float threshLin = node->compThresholdLin;
        float exponent = 1.0f - 1.0f / node->compRatio;  // e.g., 2/3 for 3:1
        float makeup = node->compMakeupLin;

        for (ma_uint32 i = 0; i < frameCount; ++i) {
            float sL = stereoOut[i * 2];
            float sR = stereoOut[i * 2 + 1];

            // Linked stereo peak detection.  std::max(NaN, x) is platform-
            // dependent (typically returns x), so an upstream NaN that
            // slipped past our earlier guards would silently disable
            // envelope tracking on the NaN'd channel and leave the clean
            // channel compressed wrong.  Force-sanitize each sample here
            // so the envelope follower always sees finite input.
            if (!std::isfinite(sL)) sL = 0.0f;
            if (!std::isfinite(sR)) sR = 0.0f;
            float peak = std::max(std::fabs(sL), std::fabs(sR));

            // Envelope follower (attack/release asymmetric smoothing)
            if (peak > node->compEnvelope)
                node->compEnvelope += node->compAttackCoeff * (peak - node->compEnvelope);
            else
                node->compEnvelope += node->compReleaseCoeff * (peak - node->compEnvelope);

            // Gain reduction in linear domain: no log10/pow10 needed.
            // gain = (threshold / envelope)^exponent when envelope > threshold.
            float totalGain = makeup;
            if (node->compEnvelope > threshLin) {
                totalGain *= std::pow(threshLin / node->compEnvelope, exponent);
            }

            stereoOut[i * 2]     = sL * totalGain;
            stereoOut[i * 2 + 1] = sR * totalGain;
        }
    }
    // Gain-staging stage 4: post-compressor.  Compare against post-EQ:
    // peak should be lower (transients tamed), RMS should be similar or
    // higher (makeup gain restoring loudness).  If post-comp RMS is well
    // above post-EQ RMS the makeup gain is over-compensating.
    if (stereoOut && frameCount > 0)
        node->meterPostComp.measure(stereoOut, frameCount);

    // Stage 3a: Master gain — applied PRE-limiter so that boosting above 1.0
    // is reined in by the limiter rather than clipping.
    if (node->masterGain != 1.0f) {
        float mg = node->masterGain;
        for (ma_uint32 i = 0; i < frameCount * 2; ++i)
            stereoOut[i] *= mg;
    }
    // Gain-staging stage 5: post-master-gain.  This is the signal handed
    // to the limiter.  Peak well above 0 dBFS here means the limiter is
    // doing the heavy lifting; consider trimming masterGain or the comp
    // makeup instead so the limiter only catches occasional transients.
    if (stereoOut && frameCount > 0)
        node->meterPostGain.measure(stereoOut, frameCount);

    // Stage 3b: Soft limiter — smooth tanh saturation above the knee threshold.
    // Transparent below the knee (default 0.8), smoothly curves to ±1.0 above.
    // Replaces the old hard clamp which produced audible click artifacts.
    // The limiter's comparisons (`x > knee`, `x < -knee`) are *both* false
    // for NaN, so NaN samples pass through unchanged.  We add an explicit
    // finite check so the limiter is also a NaN clamp.
    if (node->limiterEnabled) {
        float knee = node->limiterKnee;
        float range = 1.0f - knee;
        float invRange = 1.0f / range;
        uint64_t limHits = 0;  // count of samples that triggered the limiter
        for (ma_uint32 i = 0; i < frameCount * 2; ++i) {
            float x = stereoOut[i];
            if (!std::isfinite(x))
                stereoOut[i] = 0.0f;
            else if (x > knee) {
                stereoOut[i] = knee + range * std::tanh((x - knee) * invRange);
                ++limHits;
            }
            else if (x < -knee) {
                stereoOut[i] = -(knee + range * std::tanh((-x - knee) * invRange));
                ++limHits;
            }
        }
        node->meterLimitHits += limHits;
    } else {
        // Safety hard clamp as absolute last resort (should not be reached
        // if limiter is enabled, but guards against runaway signals).
        // Same NaN consideration as the limiter branch above.
        uint64_t limHits = 0;
        for (ma_uint32 i = 0; i < frameCount * 2; ++i) {
            if (!std::isfinite(stereoOut[i])) stereoOut[i] = 0.0f;
            else if (stereoOut[i] > 1.0f) { stereoOut[i] = 1.0f; ++limHits; }
            else if (stereoOut[i] < -1.0f) { stereoOut[i] = -1.0f; ++limHits; }
        }
        node->meterLimitHits += limHits;
    }

    // Near-clip counter: any sample with |x| > 0.99 at the *output*.  With
    // the soft limiter engaged this should be zero in normal operation —
    // any non-zero reading means the limiter wasn't able to bring a sample
    // back below the danger threshold, which is the symptom of red-lining.
    {
        uint64_t clips = 0;
        for (ma_uint32 i = 0; i < frameCount * 2; ++i) {
            if (std::fabs(stereoOut[i]) > 0.99f) ++clips;
        }
        node->meterClipCount += clips;
    }

    // Final defence-in-depth pass: by here every IIR-bearing stage has
    // been guarded, but a future regression in the master chain could
    // still leak NaN/Inf to the audio device.  One last finite-clamp on
    // the output buffer ensures Core Audio / WASAPI / ALSA never sees a
    // non-finite sample — the symptom that motivated all of this work.
    if (stereoOut && frameCount > 0) {
        if (audioSanitizeInterleaved(stereoOut, frameCount)) {
            uint32_t n = node->nanCountOutput.fetch_add(1, std::memory_order_relaxed);
            if (n < 4) {
                AUDIO_LOG("[NAN_GUARD] mixNode final output had non-finite "
                          "samples (occurrence %u) — earlier guards leaked\n",
                          n + 1);
            }
        }
        // Gain-staging stage 6: final output (== device input).  Compare
        // peak against post-gain peak to see how hard the limiter is
        // working; compare RMS against post-gain RMS to see how much
        // perceived loudness the limiter is costing.
        node->meterPostLim.measure(stereoOut, frameCount);
        node->meterTotalSamples += frameCount * 2;
    }

    // Emit a [GAIN] block periodically so the operator can tune the YAML
    // knobs (eq.gain_db, comp.threshold_db / ratio, master_gain, limiter
    // knee) toward "loud and oppressive without redlining."  One log block
    // per ~1.4 seconds (64 callbacks at 1024 frames @ 48 kHz) is dense
    // enough to follow during play, sparse enough not to flood the log.
    constexpr uint64_t kGainLogIntervalCallbacks = 64;
    ++node->meterBlocksSinceLog;
    if (node->meterBlocksSinceLog >= kGainLogIntervalCallbacks) {
        const float secs =
            static_cast<float>(node->meterDryIn.sampleCount) /
            static_cast<float>(sEngineSampleRate.load(std::memory_order_relaxed));
        // One line per stage so each is greppable in isolation.
        // Format: stage_name | peak L/R dBFS | RMS L/R dBFS
        auto emit = [&](const char *name, const StageMeter &m) {
            AUDIO_LOG("[GAIN] %-11s pk=%+6.1f/%+6.1f dB  rms=%+6.1f/%+6.1f dB\n",
                      name,
                      m.peakDbL(), m.peakDbR(),
                      m.rmsDbL(), m.rmsDbR());
        };
        AUDIO_LOG("[GAIN] === %llu callbacks (%.2fs) — chain analysis ===\n",
                  static_cast<unsigned long long>(node->meterBlocksSinceLog), secs);
        // Steam Audio per-voice dry path.  Numbers here aggregate across
        // every active voice in the window — a single hot voice pulls
        // the peak up even if other voices are quiet.  Watch for
        // sa_directO > sa_directI (3-band EQ resonating), sa_binaural >
        // sa_directO (HRTF overshoot, Steam Audio #350), or
        // sa_postLpf > sa_binaural (door-LPF IIR resonating).
        emit("sa_directI", sSaMeterDirectIn);
        emit("sa_directO", sSaMeterDirectOut);
        emit("sa_binaural",sSaMeterBinaural);
        emit("sa_postLpf", sSaMeterPostLpf);
        // Steam Audio wet path (reflection convolution + ambisonics
        // decode).  Combine all sub-workers' meters into a unified
        // view so the [GAIN] line shows the wet bus before it lands
        // in the master mix node's wet sum.  Watch for sa_reflW >>
        // sa_reflIn (IR has resonant peaks — room-mode buildup in the
        // baked reflections) or sa_decodeO >> sa_reflW (ambisonics
        // decode HRTF amplifying excessively).
        StageMeter combinedReflIn, combinedReflW, combinedDecodeOut;
        if (node->convWorker) {
            for (auto &subPtr : node->convWorker->workers) {
                combinedReflIn.mergeIn(subPtr->saMeterReflIn);
                combinedReflW.mergeIn(subPtr->saMeterReflW);
                combinedDecodeOut.mergeIn(subPtr->saMeterDecodeOut);
            }
        }
        emit("sa_reflI",  combinedReflIn);
        emit("sa_reflW",  combinedReflW);
        emit("sa_decodeO",combinedDecodeOut);
        // Master bus stages (the chain inside this mix node).
        emit("dry_in",   node->meterDryIn);
        emit("post_wet", node->meterPostWet);
        emit("post_eq",  node->meterPostEq);
        emit("post_comp",node->meterPostComp);
        emit("post_gain",node->meterPostGain);
        emit("post_lim", node->meterPostLim);
        // Activity counters — limiter hits & near-clip samples.  The
        // hit-rate gives "how often was the limiter audibly engaged";
        // clip count > 0 means a sample escaped the limiter (or the
        // limiter is disabled and the safety clamp kicked in) — that's
        // the red-line we want to drive to zero.
        const double totalDen = node->meterTotalSamples
            ? static_cast<double>(node->meterTotalSamples) : 1.0;
        const double limPct = 100.0 *
            static_cast<double>(node->meterLimitHits) / totalDen;
        const double clipPct = 100.0 *
            static_cast<double>(node->meterClipCount) / totalDen;
        AUDIO_LOG("[GAIN] limiter hits=%llu (%.3f%% of samples)  "
                  "near-clip (|x|>0.99)=%llu (%.4f%%)\n",
                  static_cast<unsigned long long>(node->meterLimitHits), limPct,
                  static_cast<unsigned long long>(node->meterClipCount), clipPct);

        // Reset everything for the next window so values reflect recent
        // activity rather than monotonic lifetime peaks.  Per-voice SA
        // meters are file-scope statics; sub-worker meters live on the
        // workers themselves.  Both are written from the audio / worker
        // threads — resetting here from the mix-node callback is safe
        // because the mix node fires *after* all per-voice callbacks
        // for the same buffer cycle, and conv-worker reset is the
        // standard accept-tearing diagnostic convention.
        sSaMeterDirectIn.reset();
        sSaMeterDirectOut.reset();
        sSaMeterBinaural.reset();
        sSaMeterPostLpf.reset();
        if (node->convWorker) {
            for (auto &subPtr : node->convWorker->workers) {
                subPtr->saMeterReflIn.reset();
                subPtr->saMeterReflW.reset();
                subPtr->saMeterDecodeOut.reset();
            }
        }
        node->meterDryIn.reset();
        node->meterPostWet.reset();
        node->meterPostEq.reset();
        node->meterPostComp.reset();
        node->meterPostGain.reset();
        node->meterPostLim.reset();
        node->meterClipCount = 0;
        node->meterLimitHits = 0;
        node->meterTotalSamples = 0;
        node->meterBlocksSinceLog = 0;
    }

    pFrameCountIn[0] = frameCount;
    *pFrameCountOut = frameCount;

    // Capture final output for debug recording (if active)
    if (stereoOut && frameCount > 0)
        recordAudioFrame(stereoOut, frameCount);

    // Profile: track peak mix node time and total callback time.
    // Skipped entirely when audio_log profiling is off — see AudioLog.h.
    if (profOn) {
        auto mixT1 = std::chrono::steady_clock::now();
        float mixUs = std::chrono::duration<float, std::micro>(mixT1 - mixT0).count();
        float prevMix = sMixNodePeakUs.load(std::memory_order_relaxed);
        if (mixUs > prevMix) sMixNodePeakUs.store(mixUs, std::memory_order_relaxed);
        // Total = all per-voice DSP time + this mix node time.
        // Note: we drain sCallbackAccumUs only when profiling is on, so when
        // profiling toggles off the accumulator may hold a stale residual.
        // It will be drained on the next on-cycle; readers always see a
        // freshly-exchanged value, so no harm done.
        float totalUs = sCallbackAccumUs.exchange(0.0f, std::memory_order_relaxed) + mixUs;
        float prevTotal = sTotalCallbackPeakUs.load(std::memory_order_relaxed);
        if (totalUs > prevTotal) sTotalCallbackPeakUs.store(totalUs, std::memory_order_relaxed);
    }
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
// ActiveVoice's full definition lives in VoicePool.h; its destructor body
// is in VoicePool.cpp. AudioService still hosts the helper that ~ActiveVoice
// needs to drain pending convolution-worker frames (drainConvolutionWorker,
// defined below where ConvolutionWorker is fully visible).

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

    // Construct the active-voice pool. Owns the handle→voice map and the
    // monotonic handle allocator. AudioService still drives startup +
    // per-voice teardown — see startVoice / removeVoiceSource.
    mVoicePool = std::make_unique<VoicePool>();

    // Construct the ambient subsystem manager. Wired with a back-pointer
    // to this service for access to the schema parser, voice map, voice
    // start/halt helpers, listener position, and reflection-mix node
    // (for the ducking multiplier). All ambient/spot-ambient lifecycle
    // and per-frame volume updates run through this object.
    mAmbientManager = std::make_unique<AmbientSoundManager>(this);

    // Construct the volumetric occlusion configuration owner. Holds
    // radius (engine feet) + sample count for Steam Audio's volumetric
    // occlusion model. RenderConfig calls setOcclusionRadius /
    // setOcclusionSamples before init/scene build so the values are
    // ready when loopStep first pushes inputs to the simulator. The
    // IPLScene handle is plumbed in later (in buildAcousticScene).
    mAudioOcclusion = std::make_unique<AudioOcclusion>();

    // Construct the reflection-sim subsystem. Its background thread is
    // started later in bootstrapFinished (after audio backends init); the
    // IPLSimulator handle is plugged in by buildAcousticScene. The
    // convolution-worker drain hook is wired here so demoteVoice can
    // safely release reflection effects mid-life.
    mReflectionSim = std::make_unique<ReflectionSimulator>();
    mReflectionSim->setConvolutionDrainHook([this]() {
        if (mConvolutionPool) mConvolutionPool->waitForCompletion();
    });
    mReflectionSim->setSimulationRanHook([this]() {
        if (mReflectionMixNode) mReflectionMixNode->simulationRan = true;
    });

    // SoundPropagation requires RoomService, which is acquired in
    // bootstrapFinished() — construct there.
}

//------------------------------------------------------
AudioService::~AudioService()
{
    // Voices must be destroyed before the engine (they reference it internally)
    if (mVoicePool) mVoicePool->clear();
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

    // Engine sample rate is configurable (default 48kHz).
    // 48kHz is preferred — Steam Audio's HRTF dataset is native at 48kHz and any
    // other rate forces internal resampling. miniaudio still resamples between
    // engine and device rates if those differ.
    ma_engine_config config = ma_engine_config_init();
    config.channels = 2;           // stereo output (hardcoded — binaural HRTF requires stereo)
    config.sampleRate = static_cast<ma_uint32>(mAudioSampleRateCfg);
    config.noDevice = MA_FALSE;    // create output device
    config.listenerCount = 1;      // one listener for 3D spatialization
    config.periodSizeInFrames = static_cast<ma_uint32>(mAudioFrameSizeCfg);

    ma_result result = ma_engine_init(&config, mMaEngine);
    if (result != MA_SUCCESS) {
        LOG_ERROR("AudioService: miniaudio init failed (error %d)", result);
        AUDIO_LOG( "AudioService: miniaudio init FAILED (error %d)\n", result);
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
        AUDIO_LOG( "AudioService: miniaudio engine %u Hz → device '%s' @ %u Hz, %u ch, "
                     "period=%u frames\n",
                     mDeviceSampleRate, device->playback.name,
                     device->sampleRate, device->playback.channels,
                     mFrameSize);
    }

    // Publish engine sample rate to audio-thread DSP that needs it (door LPF).
    sEngineSampleRate.store(mDeviceSampleRate, std::memory_order_relaxed);

    LOG_INFO("AudioService: miniaudio initialized");
    return true;
}

//------------------------------------------------------
// Live-tunable mixer gains. Setters update the AudioDSPChain member AND, if
// the reflection mix node is alive, push the new value into it so the change
// takes effect on the next audio callback. Plain float writes match the
// lock-free update pattern used elsewhere in this file (e.g.
// listenerOrientation). The clamp + storage lives in AudioDSPChain; the
// live node poke stays here because ReflectionMixNode is private to this TU.
void AudioService::setMasterGain(float g)
{
    mDSPChain->setMasterGain(g);
    if (mReflectionMixNode) mReflectionMixNode->masterGain = mDSPChain->getMasterGain();
}

void AudioService::setReflectionGain(float g)
{
    mDSPChain->setReflectionGain(g);
    if (mReflectionMixNode) mReflectionMixNode->reflGainTarget = mDSPChain->getReflectionGain();
}

void AudioService::setDirectGain(float g)
{
    mDSPChain->setDirectGain(g);
    if (mReflectionMixNode) mReflectionMixNode->directGain = mDSPChain->getDirectGain();
}

// ── ReflectionSimulator facades ──
// The throttle/rate-divisor/demote-hysteresis state moved to the
// ReflectionSimulator subsystem (see audio/ReflectionSimulator.h). These
// facades preserve the public AudioService API so RenderConfig / DebugConsole
// keep working unchanged. The setHalfRateReflections / getHalfRateReflections
// pair is kept as a deprecated wrapper for the old bool API.
void AudioService::setReflectionThrottle(int n)
{
    if (mReflectionSim) mReflectionSim->setThrottle(n);
}
int AudioService::getReflectionThrottle() const
{
    return mReflectionSim ? mReflectionSim->getThrottle() : 4;
}
void AudioService::setReflectionRateDivisor(int div)
{
    if (mReflectionSim) mReflectionSim->setRateDivisor(div);
}
int AudioService::getReflectionRateDivisor() const
{
    return mReflectionSim ? mReflectionSim->getRateDivisor() : 2;
}
void AudioService::setHalfRateReflections(bool enabled)
{
    if (mReflectionSim) mReflectionSim->setRateDivisor(enabled ? 2 : 1);
}
bool AudioService::getHalfRateReflections() const
{
    return mReflectionSim ? mReflectionSim->getRateDivisor() >= 2 : true;
}
void AudioService::setReflectionDemoteHysteresisFrames(int n)
{
    if (mReflectionSim)
        mReflectionSim->setDemoteHysteresis(std::max(1, std::min(n, 3600)));
}

//------------------------------------------------------
// Forwards to AudioDSPChain, which writes to the file-scope atomics defined
// above (sHrtfInterpolation / sSpatialBlend / sDoorLpfOpenHz /
// sDoorLpfBlockedHz / sPropMinAttenuation / sDistanceModel) via extern
// declarations in AudioDSPChain.cpp. Keeping the atomic definitions in this
// TU means the audio callback continues to read them with no extra
// indirection.
void AudioService::publishAudioThreadParams()
{
    mDSPChain->publishAudioThreadParams();
}

//------------------------------------------------------
// Ambient tuning facades — forward to AmbientSoundManager with the same
// clamps the previous inline setters applied. Manager is constructed in
// the AudioService constructor and lives for the service's lifetime, so
// these are always safe to call.
void AudioService::setAmbHysteresisStartMul(float m)
{
    if (mAmbientManager)
        mAmbientManager->setHysteresisStartMul(std::max(1.0f, std::min(m, 5.0f)));
}

void AudioService::setAmbHysteresisStopMul(float m)
{
    if (mAmbientManager)
        mAmbientManager->setHysteresisStopMul(std::max(1.0f, std::min(m, 5.0f)));
}

void AudioService::setAmbFalloffCurve(const std::string &s)
{
    if (mAmbientManager)
        mAmbientManager->setFalloffCurve(s);
}

void AudioService::setAmbDefaultPriority(int p)
{
    if (mAmbientManager)
        mAmbientManager->setDefaultPriority(std::max(0, std::min(p, 255)));
}

void AudioService::setAmbEnvironmentalSpatialBlend(float b)
{
    if (mAmbientManager)
        mAmbientManager->setEnvironmentalSpatialBlend(
            std::max(0.0f, std::min(b, 1.0f)));
}

//------------------------------------------------------
// Voice-state accessors used by AmbientSoundManager. Defined here (rather
// than inline in the header) so AmbientSoundManager.cpp doesn't need to
// see the full ActiveVoice / ReflectionMixNode struct definitions.
bool AudioService::voiceExists(SoundHandle handle) const
{
    return mVoicePool && mVoicePool->exists(handle);
}

float AudioService::voiceFalloffDistance(SoundHandle handle,
                                         float fallbackEuclideanDist) const
{
    if (!mVoicePool) return fallbackEuclideanDist;
    const ActiveVoice *v = mVoicePool->find(handle);
    if (!v) return fallbackEuclideanDist;
    if (v->cachedProp.reached)
        return v->cachedProp.effectiveDistance;
    return fallbackEuclideanDist;
}

void AudioService::voiceSetMaxAudibleDist(SoundHandle handle, float maxDist)
{
    if (!mVoicePool) return;
    ActiveVoice *v = mVoicePool->find(handle);
    if (!v) return;
    v->maxAudibleDist = maxDist;
}

void AudioService::voiceSetSpatialBlendOverride(SoundHandle handle, float blend)
{
    if (!mVoicePool) return;
    ActiveVoice *v = mVoicePool->find(handle);
    if (!v) return;
    v->dspNode.spatialBlendOverride.store(blend, std::memory_order_relaxed);
}

void AudioService::voiceSetLinearVolume(SoundHandle handle, float linearVol)
{
    if (!mVoicePool) return;
    ActiveVoice *v = mVoicePool->find(handle);
    if (!v) return;
    ma_sound_set_volume(&v->sound, linearVol);
}

float AudioService::currentDuckGain() const
{
    if (mReflectionMixNode && mReflectionMixNode->duckingEnabled)
        return mReflectionMixNode->duckGain;
    return 1.0f;
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
    hrtfSettings.volume = mDSPChain->getHRTFVolume();  // 1.0 = HRTF as-is; 0.0 = silence!

    err = iplHRTFCreate(mIplContext, &audioSettings, &hrtfSettings, &mIplHrtf);
    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("AudioService: Steam Audio HRTF creation failed (error %d)", err);
        iplContextRelease(&mIplContext);
        mIplContext = nullptr;
        return false;
    }

    // Probe baking/loading lives in ProbeManager (audio/ProbeManager.h).
    // It needs the IPL context and a hook to quiesce the reflection-sim
    // worker before any probe-batch mutation.
    {
        ProbeManagerDeps deps;
        deps.context = mIplContext;
        deps.waitForReflectionThread = [this]() {
            if (mReflectionSim) mReflectionSim->waitForCompletion();
        };
        mProbeManager = std::make_unique<ProbeManager>(std::move(deps));
    }

    LOG_INFO("AudioService: Steam Audio initialized (HRTF ready)");
    return true;
}

//------------------------------------------------------
void AudioService::shutdownSteamAudio()
{
    // ProbeManager owns the probe batch handle, which holds an internal
    // reference to the IPL context — release it before the context.
    mProbeManager.reset();
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

    // Create LRU sound cache (configurable budget; default 64MB)
    mSoundCache = std::make_unique<SoundCache>(
        static_cast<size_t>(mSoundCacheMBCfg) * 1024u * 1024u);

    // Load schema files (.spc, .arc, .sch).
    // Thief 2 stores schemas on disc 2 at EDITOR/SCHEMA/ or disc 1 at EDITOR/schemas/.
    // If --schemas path is provided, use it directly. Otherwise search known locations.
    mSchemaParser = std::make_unique<SchemaParser>();
    bool schemasLoaded = false;

    // Explicit path from --schemas CLI flag
    if (!schemasPath.empty()) {
        if (mSchemaParser->loadDirectory(schemasPath)) {
            AUDIO_LOG( "AudioService: loaded %zu schemas from %s\n",
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
                AUDIO_LOG( "AudioService: loaded %zu schemas from %s\n",
                             mSchemaParser->schemaCount(), dir.c_str());
                schemasLoaded = true;
                break;
            }
        }
    }

    if (!schemasLoaded) {
        // Hard failure: schemas are required. Without them, every ambient
        // and event-driven schema lookup falls through to a name-as-sample
        // fallback that always misses (samples in snd.crf don't share
        // schema names), so the level is silent and the log fills with
        // load-error retries. The renderer caller treats false as fatal.
        LOG_ERROR("AudioService: no schema directory found.\n"
                  "  Pass --schemas <path> to the EDITOR/SCHEMA directory,\n"
                  "  or place the schemas next to RES at %s/../EDITOR/SCHEMA.",
                  resPath.c_str());
        mSchemaParser.reset();
        mSoundLoader.reset();
        mSoundCache.reset();
        return false;
    }

    LOG_INFO("AudioService: Sound resources loaded from %s", resPath.c_str());
    AUDIO_LOG( "AudioService: loaded %zu schemas, sound resources ready\n",
                 mSchemaParser ? mSchemaParser->schemaCount() : 0);

    // Now that schemas and sound loader are ready, load auxiliary sound
    // data (per-schema attenuation, schema property overlays, misc per-
    // archetype sound props, P$LoudRoom transmission factors, P$Acoustics
    // verification) — then hand off to AmbientSoundManager for P$AmbientHack
    // and P$SpotAmb parsing.
    loadAuxiliarySoundData();
    AUDIO_LOG( "AudioService: %zu ambient sounds, %zu active voices\n",
                 mAmbientManager ? mAmbientManager->getAmbients().size() : 0,
                 mVoicePool->size());

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
    // Resolve scene type once for both the scene and the simulator below.
    IPLSceneType sceneTypeEnum = IPL_SCENETYPE_DEFAULT;
    if (mSceneTypeCfg == "embree") {
        // Embree only takes effect if Steam Audio was built with embree support;
        // otherwise the IPL fall back is to DEFAULT internally.
        sceneTypeEnum = IPL_SCENETYPE_EMBREE;
    }

    try {
        // Step 1: Create IPLScene (Steam Audio's built-in CPU raytracer)
        IPLSceneSettings sceneSettings{};
        sceneSettings.type = sceneTypeEnum;

        IPLerror err = iplSceneCreate(mIplContext, &sceneSettings, &mIplScene);
        if (err != IPL_STATUS_SUCCESS) {
            LOG_ERROR("AudioService: iplSceneCreate failed (error %d)", err);
            return false;
        }

        // Hand the scene handle to AudioOcclusion so future scene-aware
        // occlusion paths (e.g. out-of-band ray casts) can reach it
        // without needing access to AudioService's private state.
        if (mAudioOcclusion) mAudioOcclusion->setScene(mIplScene);

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

        // Step 3: Convert vertex data to IPLVector3 array and compute scene bounds.
        // Engine vertices are in feet; IPL expects meters AND a Y-up axis
        // (engineToIplPos handles both at the boundary).  mSceneMin/Max
        // stay in engine-native Z-up feet for use by render-side code; the
        // probe-OBB transform built later remaps them to IPL space.
        mSceneMin = Vector3( 1e9f,  1e9f,  1e9f);
        mSceneMax = Vector3(-1e9f, -1e9f, -1e9f);
        std::vector<IPLVector3> iplVertices(numVertices);
        for (size_t i = 0; i < numVertices; ++i) {
            Vector3 v(data.vertices[i * 3],
                      data.vertices[i * 3 + 1],
                      data.vertices[i * 3 + 2]);
            // Bounds in engine units (feet, Z-up)
            mSceneMin = glm::min(mSceneMin, v);
            mSceneMax = glm::max(mSceneMax, v);
            // Vertex shipped to IPL in meters, Y-up — UNIFORMFLOOR raycasts
            // along IPL -Y, which must point at engine-Z floors for probes
            // to land correctly.
            iplVertices[i] = engineToIplPos(v);
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

        // Compute reflection pipeline rate — reduced rate lowers per-voice FFT
        // cost while keeping HRTF at full device rate (48kHz).
        // Divisor 1=full rate, 2=half (24kHz), 4=quarter (12kHz). Rate divisor
        // is owned by ReflectionSimulator now (see initial setRateDivisor()
        // in bootstrapFinished()).
        int rateDivisor = mReflectionSim ? mReflectionSim->getRateDivisor() : 2;
        mReflectionSampleRate = mDeviceSampleRate / static_cast<uint32_t>(rateDivisor);
        mReflectionFrameSize = mFrameSize / static_cast<uint32_t>(rateDivisor);

        // Step 6: Create the simulator for direct occlusion + reflections + reverb.
        // The simulator uses the reflection sample rate — IRs are generated at this
        // rate, which must match the reflection effects and mixer.
        IPLSimulationSettings simSettings{};
        simSettings.flags = static_cast<IPLSimulationFlags>(
            IPL_SIMULATIONFLAGS_DIRECT | IPL_SIMULATIONFLAGS_REFLECTIONS);
        simSettings.sceneType = sceneTypeEnum;
        // Reflection algorithm: convolution / hybrid / parametric. The
        // simulator-side type must match the per-effect type and the
        // per-call `params.type` (phonon.h enforces this) — we set all three
        // from `mReflectionType` and a small translation table.
        simSettings.reflectionType = (mReflectionType == ReflectionType::Hybrid)
            ? IPL_REFLECTIONEFFECTTYPE_HYBRID
            : (mReflectionType == ReflectionType::Parametric)
                ? IPL_REFLECTIONEFFECTTYPE_PARAMETRIC
                : IPL_REFLECTIONEFFECTTYPE_CONVOLUTION;
        // Sim caps — these are upper bounds; runtime uses mRealtimeNumRays etc.
        // maxNumRays must be >= the runtime ray count; auto-bump if user set both.
        simSettings.maxNumOcclusionSamples = mSimMaxOcclusionSamplesCfg;
        simSettings.maxNumRays = std::max(mSimMaxRaysCfg, mRealtimeNumRays);
        simSettings.numDiffuseSamples = mRealtimeDiffuseSamples;
        simSettings.maxDuration = mRealtimeDuration;
        simSettings.maxOrder = mAmbisonicsOrder;
        simSettings.maxNumSources = mReflectionMaxSourcesCfg;
        // Ray-trace thread count: 0 = auto (reserve 2 cores for main + audio).
        if (mSimulatorThreadsCfg > 0) {
            simSettings.numThreads = mSimulatorThreadsCfg;
        } else {
            unsigned int hwThreads = std::thread::hardware_concurrency();
            simSettings.numThreads = std::max(2u, hwThreads > 2 ? hwThreads - 2 : 2u);
        }
        simSettings.samplingRate = static_cast<IPLint32>(mReflectionSampleRate);
        simSettings.frameSize = static_cast<IPLint32>(mReflectionFrameSize);

        IPLSimulator reflectionSimHandle = nullptr;
        err = iplSimulatorCreate(mIplContext, &simSettings, &reflectionSimHandle);
        if (err != IPL_STATUS_SUCCESS) {
            LOG_ERROR("AudioService: iplSimulatorCreate failed (error %d)", err);
            destroyAcousticScene();
            return false;
        }

        // Install the freshly-created simulator handle onto the
        // ReflectionSimulator subsystem so its worker thread can pump it.
        if (mReflectionSim) mReflectionSim->setSimulator(reflectionSimHandle);

        // Bind the scene to the reflection simulator
        iplSimulatorSetScene(reflectionSimHandle, mIplScene);
        iplSimulatorCommit(reflectionSimHandle);

        // Step 6b: Create the direct-only simulator. Runs synchronously on
        // the main thread in loopStep, so it has no background source-list
        // iteration — iplSourceAdd is always immediately safe and a newly
        // created voice gets correct directParams from the next loopStep
        // (≈one frame), instead of after a full reflection cycle (200-800
        // ms) when racing the reflection sim. See HANDOFF.AUDIO_VOICE_INIT.md
        // for the underlying race this addresses.
        //
        // Reflection-only fields (maxNumRays, numDiffuseSamples, maxDuration,
        // maxOrder, samplingRate, frameSize, reflectionType) are left at
        // zero — Steam Audio skips allocating those buffers when REFLECTIONS
        // is not in the flags. Direct cap is decoupled from the reflection cap
        // (typically much larger — direct sim cost is ~constant per source).
        IPLSimulationSettings directSettings{};
        // DIRECT carries distance/occlusion/transmission/air-absorption;
        // PATHING carries the probe-graph baked path lookup that feeds the
        // per-voice eqCoeffs gain + LPF for player audio. Both run on the
        // same direct simulator handle so iplSimulatorRunDirect /
        // iplSimulatorRunPathing share the staged source list — no
        // cross-thread synchronisation needed since loopStep runs both
        // synchronously on the main thread.
        directSettings.flags = static_cast<IPLSimulationFlags>(
            IPL_SIMULATIONFLAGS_DIRECT | IPL_SIMULATIONFLAGS_PATHING);
        directSettings.sceneType = sceneTypeEnum;
        directSettings.maxNumOcclusionSamples = mSimMaxOcclusionSamplesCfg;
        directSettings.maxNumSources = mDirectMaxSourcesCfg;
        // Probe-to-probe visibility sampling count for pathing — when a
        // baked path is occluded by dynamic geometry and findAlternatePaths
        // re-runs path finding, this controls how many rays test each
        // candidate edge. 16 matches the benchmark prototype's setting;
        // higher = smoother dynamic obstruction at proportionally higher
        // cost.
        directSettings.numVisSamples = 16;
        // Direct sim runs on the main thread so internal worker threads
        // here are mostly for parallel occlusion ray-traces. Match the
        // reflection sim's choice for now.
        directSettings.numThreads = simSettings.numThreads;

        err = iplSimulatorCreate(mIplContext, &directSettings, &mDirectSimulator);
        if (err != IPL_STATUS_SUCCESS) {
            LOG_ERROR("AudioService: direct iplSimulatorCreate failed (error %d)", err);
            destroyAcousticScene();
            return false;
        }

        // Share the scene with the direct simulator. iplSceneRetain bumps
        // the refcount so destroyAcousticScene's iplSceneRelease still
        // brings it down to zero (one retain per simulator + the original
        // create reference).
        iplSceneRetain(mIplScene);
        iplSimulatorSetScene(mDirectSimulator, mIplScene);
        iplSimulatorCommit(mDirectSimulator);

        mSceneReady = true;
        mAcousticTriCount = static_cast<int>(numTriangles);

        // Create the reflection pipeline (mixer + ambisonics decode + mix node)
        // This is optional — if it fails, direct effects still work
        AUDIO_LOG( "REFL: about to call initReflectionPipeline()\n");
        if (!initReflectionPipeline()) {
            LOG_ERROR("AudioService: reflection pipeline init failed — "
                      "direct effects only");
        }
        AUDIO_LOG( "REFL: initReflectionPipeline returned\n");

        LOG_INFO("AudioService: acoustic scene built — %zu vertices, %zu triangles, "
                 "%zu materials", numVertices, numTriangles, materials.size());
        AUDIO_LOG( "AudioService: acoustic scene built (%zu tris, %zu mats)\n",
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
    AUDIO_LOG( "REFL: initReflectionPipeline enter\n");
    IPLSimulator reflectionSimHandle = mReflectionSim ? mReflectionSim->simulator() : nullptr;
    if (!mIplContext || !mIplHrtf || !mMaEngine || !reflectionSimHandle)
        return false;
    const int rateDivisor = mReflectionSim->getRateDivisor();

    // Reflection pipeline uses the reflection sample rate (24kHz or 48kHz).
    // This rate was computed in buildAcousticScene and must match the simulator.
    IPLAudioSettings audioSettings{};
    audioSettings.samplingRate = static_cast<IPLint32>(mReflectionSampleRate);
    audioSettings.frameSize = static_cast<IPLint32>(mReflectionFrameSize);

    // irSize = realtime duration * sampling rate (matches simulator settings).
    // For hybrid, this covers the WHOLE IR (convolution portion plus a small
    // margin for the parametric crossfade) — Steam Audio crashes if
    // hybridReverbTransitionTime > irSize/samplingRate, so we keep
    // mRealtimeDuration > mHybridTransitionTime as policy.
    IPLint32 irSize = static_cast<IPLint32>(mRealtimeDuration * mReflectionSampleRate);
    // Compute ambisonics channel count: (order+1)^2. Order 0=1ch, order 1=4ch.
    mAmbisonicsChannels = (mAmbisonicsOrder + 1) * (mAmbisonicsOrder + 1);
    IPLint32 numAmbiChannels = static_cast<IPLint32>(mAmbisonicsChannels);

    AUDIO_LOG( "REFL: creating mixer (irSize=%d, channels=%d, rate=%d, frame=%d, 1/%d rate)\n",
                 irSize, numAmbiChannels,
                 audioSettings.samplingRate, audioSettings.frameSize,
                 rateDivisor);

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
    AUDIO_LOG( "REFL: mixer created OK\n");

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
    // Wet saturation scratch (full engine frame size — saturator runs after
    // sub-worker output is upsampled to device rate via the reflection mix).
    rmn.wetScratchL.resize(mFrameSize, 0.0f);
    rmn.wetScratchR.resize(mFrameSize, 0.0f);

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
    AUDIO_LOG( "REFL: node initialized, attaching to endpoint\n");

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

    // Apply DSP chain config from AudioDSPChain (set by RenderConfig).
    // Must be done BEFORE setting ready=true so the audio callback never sees
    // uninitialized coefficients (ready + simulationRan gate the DSP path).
    rmn.limiterEnabled    = mDSPChain->getDSPLimiterEnabled();
    rmn.limiterKnee       = mDSPChain->getDSPLimiterKnee();
    rmn.compressorEnabled = mDSPChain->getDSPCompressorEnabled();
    rmn.compThresholdDb   = mDSPChain->getDSPCompThreshold();
    rmn.compRatio         = mDSPChain->getDSPCompRatio();
    rmn.compAttackMs      = mDSPChain->getDSPCompAttackMs();
    rmn.compReleaseMs     = mDSPChain->getDSPCompReleaseMs();
    rmn.eqEnabled         = mDSPChain->getDSPEQEnabled();
    rmn.eqFreq            = mDSPChain->getDSPEQFreq();
    rmn.eqGainDb          = mDSPChain->getDSPEQGain();
    rmn.eqQ               = mDSPChain->getDSPEQQ();
    rmn.duckingEnabled    = mDSPChain->getDSPDuckingEnabled();
    rmn.duckAmount        = mDSPChain->getDSPDuckAmount();
    rmn.duckAttackMs      = mDSPChain->getDSPDuckAttackMs();
    rmn.duckReleaseMs     = mDSPChain->getDSPDuckReleaseMs();
    rmn.wetSaturationEnabled = mDSPChain->getDSPWetSaturationEnabled();
    rmn.wetSaturationDrive   = mDSPChain->getDSPWetSaturationDrive();

    // Mixer config — global gains + reflection ramp time.
    // reflRampRate is the per-sample increment such that a full 0→1 ramp takes
    // getReflectionRampMs() milliseconds at the current sample rate.
    rmn.masterGain     = mDSPChain->getMasterGain();
    rmn.directGain     = mDSPChain->getDirectGain();
    rmn.reflGainTarget = mDSPChain->getReflectionGain();
    {
        float rampSamples = (mDSPChain->getReflectionRampMs() * 0.001f) * static_cast<float>(mDeviceSampleRate);
        rmn.reflRampRate = (rampSamples > 1.0f) ? (1.0f / rampSamples) : 1.0f;
    }

    // Initialize the master bus DSP chain (EQ, compressor, limiter coefficients).
    // Must be called AFTER pushing config — initDSP reads compAttackMs/eqQ/etc.
    rmn.initDSP(mDeviceSampleRate);
    AUDIO_LOG( "REFL: DSP chain initialized (limiter=%s knee=%.2f, compressor=%s %.0fdB/%g:1, eq=%s %gHz/%+gdB, ducking=%s)\n",
                 rmn.limiterEnabled ? "on" : "off", rmn.limiterKnee,
                 rmn.compressorEnabled ? "on" : "off", rmn.compThresholdDb, rmn.compRatio,
                 rmn.eqEnabled ? "on" : "off", rmn.eqFreq, rmn.eqGainDb,
                 rmn.duckingEnabled ? "on" : "off");

    // Set ready AFTER DSP coefficients are fully written — ensures the audio
    // callback never executes the DSP chain with default/uninitialized values.
    rmn.ready = true;

    // NOTE: rmn.convWorker is set AFTER the worker pool is created (below).

    // Determine number of sub-workers
    int numWorkers;
    if (mConvolutionWorkerCount > 0) {
        numWorkers = mConvolutionWorkerCount;
    } else {
        // Auto: use available cores minus 3 (main + audio + sim), minimum 1
        unsigned int hwThreads = std::thread::hardware_concurrency();
        numWorkers = std::max(1, static_cast<int>(hwThreads) - 3);
    }

    // Hand off to the ConvolutionWorkerPool — it creates the pool's
    // ConvolutionWorker struct, allocates per-sub-worker Steam Audio
    // pipelines (own mixer + decoder), and spawns the threads.
    ConvolutionWorkerPool::Config poolCfg;
    poolCfg.context = mIplContext;
    poolCfg.hrtf = mIplHrtf;
    poolCfg.frameSize = static_cast<int>(mFrameSize);
    poolCfg.reflectionFrameSize = static_cast<int>(mReflectionFrameSize);
    poolCfg.rateDivisor = rateDivisor;
    poolCfg.ambiChannels = mAmbisonicsChannels;
    poolCfg.ambiOrder = mAmbisonicsOrder;
    poolCfg.numWorkers = numWorkers;
    poolCfg.audioSettings = audioSettings;
    poolCfg.reflSettings = reflSettings;
    mConvolutionPool = std::make_unique<ConvolutionWorkerPool>();
    if (!mConvolutionPool->init(poolCfg)) {
        mConvolutionPool.reset();
    }

    if (mConvolutionPool) {
        // Wire the mix node to the worker (pool exposes the raw struct
        // pointer so the audio-thread fast path stays unchanged).
        rmn.convWorker = mConvolutionPool->worker();

        AUDIO_LOG( "REFL: convolution started (%d sub-workers, off-thread)\n",
                     mConvolutionPool->numWorkers());
    }

    AUDIO_LOG( "AudioService: reflection pipeline initialized "
                 "(convolution, order %d (%dch), IR %d samples, %uHz, 1/%d rate, max %d voices%s)\n",
                 mAmbisonicsOrder, mAmbisonicsChannels,
                 irSize, mReflectionSampleRate, rateDivisor,
                 mMaxReflectionVoices,
                 mConvolutionPool ? ", off-thread" : ", on-thread fallback");
    return true;
}

//------------------------------------------------------
void AudioService::destroyReflectionPipeline()
{
    // Shut down all convolution sub-workers before destroying any Steam Audio objects
    if (mConvolutionPool) {
        mConvolutionPool->shutdown();
        mConvolutionPool.reset();
    }

    // Wait for any in-flight simulation tasks to finish on the worker thread
    if (mReflectionSim) {
        mReflectionSim->waitForCompletion();
        // Flush deferred IPL source adds — never added to simulator, just release
        mReflectionSim->releasePendingAdds();
        // Flush deferred IPL source removals before destroying the simulator
        mReflectionSim->flushPendingRemovals();
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

    // Release probe batch before simulator (it's registered with the simulator).
    // The batch may be attached to BOTH the reflection sim (for baked reverb
    // lookups) AND the direct sim (for Steam Audio pathing). Detach from the
    // direct sim first, releasing the iplProbeBatchRetain we took at
    // loadProbes time, then hand off to ProbeManager which detaches from the
    // reflection sim and releases the create-time reference.
    if (mProbeManager) {
        IPLProbeBatch batch = mProbeManager->getProbeBatch();
        if (batch && mDirectProbeBatchAdded && mDirectSimulator) {
            iplSimulatorRemoveProbeBatch(mDirectSimulator, batch);
            iplSimulatorCommit(mDirectSimulator);
            iplProbeBatchRelease(&batch);  // drops our retain (batch is aliased,
                                           // ProbeManager still holds the original
                                           // create-time reference)
        }
        mDirectProbeBatchAdded = false;
        if (mReflectionSim) {
            mProbeManager->releaseBatch(mReflectionSim->simulator());
        }
    }

    if (mReflectionSim && mReflectionSim->simulator()) {
        IPLSimulator handle = mReflectionSim->simulator();
        iplSimulatorRelease(&handle);
        mReflectionSim->setSimulator(nullptr);
    }
    // Direct simulator runs synchronously on the main thread, so it has
    // no in-flight worker to wait for. Release after the reflection sim
    // (order is irrelevant; this just keeps the parallel structure
    // visible). The scene we attached to it was retained at create time;
    // iplSceneRelease below drops both the original create-time reference
    // and the retain — refcount reaches zero and the scene is destroyed.
    if (mDirectSimulator) {
        iplSimulatorRelease(&mDirectSimulator);
        mDirectSimulator = nullptr;
    }

    if (mIplStaticMesh) {
        iplStaticMeshRelease(&mIplStaticMesh);
        mIplStaticMesh = nullptr;
    }
    if (mIplScene) {
        // We hold two references on the scene: one from iplSceneCreate, one
        // from iplSceneRetain in initAcousticScene (the second simulator's
        // reference). Release both so the refcount reaches zero. Pass a
        // local handle the first time so iplSceneRelease's null-on-deref
        // doesn't clobber mIplScene before the second call.
        IPLScene aliased = mIplScene;
        iplSceneRelease(&aliased);            // drops the retain
        iplSceneRelease(&mIplScene);          // drops the original create-ref
    }

    // Drop the dangling-pointer aliased scene handle that AudioOcclusion
    // cached at build time so it doesn't survive across rebuilds.
    if (mAudioOcclusion) mAudioOcclusion->setScene(nullptr);
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

    // Sound propagation owns the per-portal door blocking map + per-room
    // LoudRoom transmission map and forwards propagateSound() into
    // RoomService::propagateSoundPath. It can only be constructed once
    // RoomService is resolved.
    mSoundPropagation = std::make_unique<SoundPropagation>(mRoomService.get());

    // Register as a database listener — load after rooms are ready
    mDbService->registerListener(this, DBP_AUDIO);

    // Register as a loop client for per-frame updates
    auto loopSvc = GET_SERVICE(LoopService);
    loopSvc->addLoopClient(this);

    // Initialize audio backends
    bool maOk = initMiniaudio();
    bool saOk = initSteamAudio();
    mAudioReady = maOk && saOk;

    // Start the reflection simulation worker thread (background, latency-tolerant).
    // Direct sim runs synchronously on the main thread for same-frame occlusion.
    if (mAudioReady && mReflectionSim) {
        mReflectionSim->start();
    }

    if (mAudioReady) {
        LOG_INFO("AudioService: fully initialized");
        AUDIO_LOG( "AudioService: fully initialized (miniaudio + Steam Audio)\n");
    } else {
        LOG_ERROR("AudioService: initialized with errors (miniaudio=%s, steam_audio=%s)",
                  maOk ? "ok" : "FAILED", saOk ? "ok" : "FAILED");
        AUDIO_LOG( "AudioService: INIT FAILED (miniaudio=%s, steam_audio=%s)\n",
                     maOk ? "ok" : "FAILED", saOk ? "ok" : "FAILED");
    }
}

//------------------------------------------------------
void AudioService::shutdown()
{
    haltAll();
    if (mAmbientManager) mAmbientManager->clear();

    // Release sound resources
    mSoundCache.reset();
    mSoundLoader.reset();

    // Shut down the reflection simulation worker thread before destroying the scene
    if (mReflectionSim) mReflectionSim->stop();

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
    // onDBLoad fires once per loaded FileGroup with `curmask` indicating
    // which data classes that file contributed:
    //   • DBM_OBJTREE_GAMESYS — the gam (dark.gam) load. Global tables
    //     live here: ENV_SOUND, Speech_DB, SchSamp, AIHearStat, AISNDTWK.
    //   • DBM_MIS_DATA        — the mis load. Per-mission chunks live
    //     here: ROOM_EAX, AMBIENT, L$SoundDesc.
    // We dispatch per-mask so that loading a mission (which fires both
    // events in sequence) populates everything, and loading the gam
    // standalone still populates the global tables.

    // ── Gamesys-level data ───────────────────────────────────────────
    // The global sound chunk databases are stored in dark.gam, so we
    // load them on the GAMESYS pass. Some missions also embed copies
    // (e.g. modified schemas), so we still try the MIS pass below if
    // GAMESYS didn't surface them — `loadSoundChunkDatabases` is
    // idempotent and `db->hasFile` returns false for missing chunks.
    if (curmask & DBM_OBJTREE_GAMESYS) {
        loadSoundChunkDatabases(db);
    }

    // Everything below is mission-level — bail if this isn't the MIS
    // pass.
    if (!(curmask & DBM_MIS_DATA))
        return;

    // NOTE: Sound resources, acoustic scene, and ambient sounds are loaded from
    // DarknessRender.cpp after this point (loadSoundResources → loadAmbientSounds).
    // onDBLoad just records that mission data is available.

    // Parse ROOM_EAX chunk — per-room EAX reverb preset indices.
    // The original Dark Engine stored these for Creative's hardware EAX reverb.
    // We parse them for verification logging: comparing the designer's intended
    // reverb character against Steam Audio's physics-based convolution result.
    mRoomEAXPresets.clear();
    if (db->hasFile("ROOM_EAX")) {
        FilePtr eaxFile = db->getFile("ROOM_EAX");
        uint32_t count = 0;
        *eaxFile >> count;
        if (count > 0 && count < 1024) {  // sanity check
            for (uint32_t i = 0; i < count; ++i) {
                uint32_t eaxIdx = 0;
                *eaxFile >> eaxIdx;
                if (eaxIdx < 26)
                    mRoomEAXPresets[static_cast<int32_t>(i)] = eaxIdx;
            }
            AUDIO_LOG( "AudioService: parsed ROOM_EAX chunk — %u rooms with EAX presets\n", count);
        }
    } else {
        AUDIO_LOG( "AudioService: no ROOM_EAX chunk in mission (EAX presets unavailable)\n");
    }

    // Retry global sound databases against the MIS FileGroup in case
    // a mission overrides one (the call is idempotent — chunks already
    // loaded on the GAMESYS pass remain unless overwritten).
    loadSoundChunkDatabases(db);

    // Parse mission-level sound data: AMBIENT chunk + L$SoundDesc links.
    loadMissionSoundData(db);

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
    if (mSoundPropagation) mSoundPropagation->clear();
    mListenerRoom = nullptr;
    mRoomEAXPresets.clear();
    if (mVoicePool) mVoicePool->resetAllocator();

    // Flush the sound cache on mission unload (sounds may differ between missions)
    if (mSoundCache) {
        mSoundCache->clear();
    }

    // Release Steam Audio acoustic scene
    destroyAcousticScene();

    // Clear ambient sounds (voices already halted above)
    if (mAmbientManager) mAmbientManager->clear();

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

    // Read profiling flag once per loopStep. When off, all the perf-counter
    // writes scattered through this function and its callees become no-ops.
    // See AudioLog.h for the toggle.
    const bool profOn = ::Darkness::gAudioLogVerbose;
    auto loopStepStart = profOn ? std::chrono::steady_clock::now()
                                : std::chrono::steady_clock::time_point{};

    // Update ambient volumes early so newly created ambient voices exist before
    // the simulation runs. This ensures same-frame occlusion for new ambients —
    // voices created after the sim would miss occlusion for their first frame.
    // The audio status dump fires first to capture top-of-frame state, then
    // the manager updates voices, then the ducking envelope ramps for the
    // next frame's per-voice duck multiplier (matching the pre-extraction
    // order, when all three lived inside AudioService::updateAmbientVolumes).
    dumpAudioStatusPeriodic();
    if (mAmbientManager) {
        mAmbientManager->updateAmbientVolumes();
        mAmbientManager->updateSpotAmbientVolumes();
    }
    updateAmbientDuckingEnvelope();

    // Source mutations (add/remove/commit) can race with the background reflection
    // sim thread, so we defer them until it's idle. Direct sim runs synchronously
    // on the main thread, so it's never concurrent with mutations.
    // Steam Audio uses double-buffering: setInputs writes to the staging buffer,
    // while runReflections reads from the committed (active) buffer.
    // Only commit() copies staging → active, so commit must wait for reflections.
    bool reflBusy = mReflectionSim && mReflectionSim->isRunning();
    bool canMutate = !reflBusy;
    IPLSimulator reflectionSimHandle = mReflectionSim ? mReflectionSim->simulator() : nullptr;

    if (canMutate && mReflectionSim) {

        // Flush deferred IPL source adds / removals via ReflectionSimulator.
        mReflectionSim->flushPendingAdds();
        mReflectionSim->flushPendingRemovals();
    }

    // Commit copies staging → active buffer. Must wait for reflection sim to
    // be idle since it reads from the active buffer. Direct sim runs inline
    // (after commit), so no conflict there.
    if (canMutate && mReflectionSim && mReflectionSim->isSimulatorDirty() && reflectionSimHandle) {
        if (profOn) {
            auto ct0 = std::chrono::steady_clock::now();
            mReflectionSim->commitIfDirty();
            auto ct1 = std::chrono::steady_clock::now();
            float cMs = std::chrono::duration<float, std::milli>(ct1 - ct0).count();
            float prevC = sCommitPeakMs.load(std::memory_order_relaxed);
            if (cMs > prevC) sCommitPeakMs.store(cMs, std::memory_order_relaxed);
        } else {
            mReflectionSim->commitIfDirty();
        }
    }

    // Always clean up finished voices (defers IPL removal if sim busy)
    cleanupFinishedVoices();

    // Publish a one-bit "any player-emitted voice is making sound" flag so the
    // renderer can flash a listener-position marker in sync with footstep
    // audibility. Scan is O(voices) but voices ≤ MAX_ACTIVE_VOICES (64) and we
    // already iterate them in this function — the marginal cost is negligible.
    // `sourceEnded` flips to true once miniaudio drains the decoder; voices
    // hang around briefly after that for the reverb tail, which we
    // intentionally exclude so the marker tracks the original-sample
    // duration (matches what the user *hears* as "the footstep").
    {
        bool anyActive = false;
        for (const auto &[handle, voice] : mVoicePool->voices()) {
            if (voice && voice->playerEmitted &&
                !voice->sourceEnded.load(std::memory_order_relaxed)) {
                anyActive = true;
                break;
            }
        }
        mPlayerEmittedActive.store(anyActive, std::memory_order_relaxed);
    }

    // Run Steam Audio simulation for all active sources
    if (mSceneReady && reflectionSimHandle && !mVoicePool->empty()) {
        float cosY = std::cos(mListenerYaw), sinY = std::sin(mListenerYaw);
        float cosP = std::cos(mListenerPitch), sinP = std::sin(mListenerPitch);

        // Listener coordinate frame in engine (Z-up) space.
        Vector3 right(sinY, -cosY, 0.0f);
        Vector3 ahead(cosY * cosP, sinY * cosP, sinP);
        Vector3 up(-cosY * sinP, -sinY * sinP, cosP);

        // Cross the IPL boundary: feet→meters AND Z-up→Y-up.  All other
        // IPL coordinates (mesh, sources, probe transform) use the same
        // remap so the relative geometry IPL sees is internally consistent;
        // HRTF binaural panning still works because the listener basis is
        // converted along with everything else.
        IPLCoordinateSpace3 listenerCoord{};
        listenerCoord.origin = engineToIplPos(mListenerPos);
        listenerCoord.ahead  = engineToIplDir(ahead);
        listenerCoord.right  = engineToIplDir(right);
        listenerCoord.up     = engineToIplDir(up);

        // Update listener orientation for ambisonics decode (audio thread reads this)
        if (mReflectionMixNode && mReflectionMixNode->ready) {
            mReflectionMixNode->listenerOrientation = listenerCoord;
            // Also publish listener position in engine feet for the
            // [WET_BUS] diagnostic. Same loose-sync pattern as
            // listenerOrientation — single-writer, periodic read.
            mReflectionMixNode->listenerPosX = mListenerPos.x;
            mReflectionMixNode->listenerPosY = mListenerPos.y;
            mReflectionMixNode->listenerPosZ = mListenerPos.z;
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

        {
            // Step 1: Set listener position for the simulator
            IPLSimulationSharedInputs sharedInputs{};
            sharedInputs.listener = listenerCoord;
            sharedInputs.numRays = mRealtimeNumRays;
            sharedInputs.numBounces = mRealtimeNumBounces;
            sharedInputs.duration = mRealtimeDuration;
            sharedInputs.order = mAmbisonicsOrder;
            sharedInputs.irradianceMinDistance = 1.0f;
            // Listener pose goes to both simulators; reflection params
            // (numRays/numBounces/duration/order/irradianceMinDistance) are
            // only consumed when the REFLECTIONS flag is set, so passing the
            // populated struct to the direct sim with only the DIRECT flag
            // is safe — Steam Audio ignores the unused fields. Per phonon.h,
            // these calls require no synchronisation between them.
            iplSimulatorSetSharedInputs(mDirectSimulator,
                IPL_SIMULATIONFLAGS_DIRECT, &sharedInputs);
            iplSimulatorSetSharedInputs(reflectionSimHandle,
                IPL_SIMULATIONFLAGS_REFLECTIONS, &sharedInputs);

            // Step 2a: Pre-compute reflection voice ranking (top-N closest).
            // Done before setInputs so we can set inputs.baked for non-top-N voices.
            //
            // Player-emitted voices (footsteps, landing impacts) are EXCLUDED
            // from the top-N pool — they always use the baked-probe reverb path
            // instead. Rationale:
            //   • Source ≈ listener for player-emitted sounds, so the room IR
            //     at listener position (what baked reverb captures) is an
            //     excellent approximation of the source-position IR.
            //   • The real-time ray-trace sim takes ~200-400 ms per step.
            //     Footsteps last ~200 ms, so without baked routing many
            //     footsteps end before their IR is computed — they vanish
            //     from the indirect path entirely.
            //   • Baked lookup is O(1) and frees real-time sim budget for
            //     voices whose source position genuinely matters (NPC dialogue,
            //     gunshots in other corridors, etc.).
            // Requires probes to be loaded (`mProbeManager->hasReflections()`); if not,
            // player-emitted voices fall through to the same path as everything
            // else and the timing race may reappear.
            if (mReflectionsEnabled) {
                // Tail voices (source ended, reverb still ringing) keep their
                // convolution slot until the IR finishes — they're carried over
                // from previous frames and can't be cheaply disabled mid-
                // convolution without an audible click.  They consume budget
                // that would otherwise go to new top-N voices, so we count
                // them BEFORE selecting top-N and shrink the top-N cap by
                // tailCount.  Without this step, top-N + tails together
                // overshoot mMaxReflectionVoices, which is exactly the
                // "17–21 refl with max=16" behaviour the perf log shows.
                int tailCount = 0;
                for (auto &[h, v] : mVoicePool->voices()) {
                    if (v->sourceEnded
                        && !v->finished.load(std::memory_order_relaxed)
                        && v->dspNode.reflectionsActive.load(std::memory_order_relaxed))
                    {
                        ++tailCount;
                    }
                }
                int topNCap = std::max(0, mMaxReflectionVoices - tailCount);

                reflCandidates.reserve(mVoicePool->size());
                for (auto &[h, v] : mVoicePool->voices()) {
                    if (v->sourceEnded)
                        continue;
                    if (v->playerEmitted && mProbeManager->hasReflections())
                        continue;  // routed via baked-probe path below
                    // Voices demoted by the stage 2.2 fallback have
                    // reflectionSource == nullptr — exclude them from the
                    // top-N pool (they've gone fully dry until the voice
                    // ends; no need to re-rank them every frame).
                    if (v->reflectionSource && v->dspNode.effectsReady && v->dspNode.reflectionEffect) {
                        Vector3 delta = v->worldPos - mListenerPos;
                        reflCandidates.push_back({h, glm::dot(delta, delta)});
                    }
                }
                if (static_cast<int>(reflCandidates.size()) > topNCap) {
                    std::partial_sort(reflCandidates.begin(),
                                      reflCandidates.begin() + topNCap,
                                      reflCandidates.end(),
                                      [](const VoiceDist &a, const VoiceDist &b) {
                                          return a.distSq < b.distSq;
                                      });
                    reflCandidates.resize(topNCap);
                }
            }

            // Build a set for O(1) lookup of top-N membership
            for (const auto &rc : reflCandidates)
                reflCandidateSet.insert(rc.handle);

            // Stage 2.2 — demote-only fallback pass.
            //
            // Every voice starts with a reflectionSource (eager allocation
            // in createVoiceSource), so by default it routes through baked
            // reverb. The per-frame inputs.baked flip below upgrades the
            // closest top-N voices to ray-traced realtime reflections.
            //
            // The demote fallback below is reserved for voices that have
            // sat out of top-N for mReflectionDemoteHysteresisCfg
            // consecutive frames — i.e., genuinely long-lived voices that
            // have stayed distant/quiet for a long time. Releasing their
            // source trims mSourceData[0] iteration cost in the reflection
            // sim cycle. The cost: those voices then play dry (no realtime
            // and no baked) for the rest of their lifetime. Default
            // hysteresis (≈10 s at 60 fps) is intentionally high so this
            // only fires when convolution/sim budget is genuinely under
            // pressure — the steady-state assumption is that every voice
            // keeps its baked reverb for life.
            //
            // Decisions are made against the live mListenerPos every frame,
            // not frozen at spawn — a voice that's far at startVoice can
            // be close 10 seconds later (and vice versa) in a large level.
            //
            // PlayerEmitted and Ambient voices are excluded — PE uses baked
            // routing as its steady-state path, Ambient voices are
            // long-lived and intentionally part of the room ambience.
            if (mReflectionsEnabled && reflectionSimHandle && mReflectionSim) {
                int hysteresis = mReflectionSim->getDemoteHysteresis();
                for (auto &[h, v] : mVoicePool->voices()) {
                    if (v->sourceEnded) continue;
                    if (v->playerEmitted) continue;  // baked routing, never demoted
                    if (v->isAmbient)     continue;  // long-lived, never demoted
                    if (!v->dspNode.effectsReady) continue;
                    if (!v->reflectionSource) continue;  // already demoted

                    bool inTopN = reflCandidateSet.count(h) > 0;
                    if (inTopN) {
                        v->framesOutOfTopN = 0;
                    } else if (++v->framesOutOfTopN >= hysteresis) {
                        mReflectionSim->demoteVoice(*v);
                    }
                }
            }

            // Baked reflection identifier (for non-top-N voices using probe reverb)
            IPLBakedDataIdentifier bakedReflId{};
            bakedReflId.type = IPL_BAKEDDATATYPE_REFLECTIONS;
            bakedReflId.variation = IPL_BAKEDDATAVARIATION_REVERB;

            // Step 2b: Run portal propagation and set source positions.
            // Architecture B: for cross-room voices, set the IPLSource position
            // to the virtual position (last portal anchor) so Steam Audio traces
            // rays from the doorway, not from behind a wall. Same-room voices
            // use their real position. Unreachable voices skip simulation.
            //
            // Listener-room update: canonical RoomService::updatedRoom
            // query — always disambiguates overlapping room OBBs via
            // portal planes (the disambiguator polarity fix from the
            // 2026-05 cleanup made this deterministic). Previous
            // implementation also gated on a portal-blend state to
            // sustain the room assignment near boundaries; that blend
            // has been removed because its hysteresis produced
            // asymmetric "louder walking away" volume artifacts —
            // matches the original engine which has no propagation
            // hysteresis. See NOTES.PROJECT.md "Sound Propagation
            // Model".
            if (mRoomService) {
                mListenerRoom = mRoomService->updatedRoom(mListenerRoom, mListenerPos);
            } else {
                mListenerRoom = nullptr;
            }
            Room *listenerRoom = mListenerRoom;

            for (auto &[handle, voice] : mVoicePool->voices()) {
                // createVoiceSource always sets both sources together (or
                // neither, on init failure), so checking directSource here
                // covers both. The reflectionSource may be in mPendingSourceAdds
                // — that's handled separately below at the per-voice
                // outputs read.
                if (!voice->directSource)
                    continue;

                // Defensive per-frame reset of skipAttenuation. No voice
                // class currently sets this to true — the prior workaround
                // for player-emitted voices was needed when the unified
                // simulator's defer-flush race silently broke the surgical
                // direct-flag overrides; with the split simulator the
                // overrides are read by the audio thread on the very first
                // callback after createVoiceSource.
                voice->dspNode.skipAttenuation = false;

                // Decide which propagation backend drives this voice this
                // frame. Player audio (everything in mVoicePool) uses Steam
                // Audio's pathing simulator when the probe batch is attached
                // to the direct sim AND mProbePathingEnabled is true — gives
                // a smooth, continuous 3-band eqCoeffs envelope instead of
                // the room-graph BFS's discrete same-room/cross-room snap
                // at portal crossings. Player-emitted voices (footsteps,
                // landings) are excluded — source is approximately the
                // listener, so the pathing graph collapses to a self-loop
                // with eqCoeffs ≡ 1.0 anyway, and running it would just
                // churn the same direct sim. Setting
                // audio.propagation.probe_pathing: false in YAML (or
                // toggling probe_pathing in the debug console) flips this
                // off at runtime, forcing the BFS branch back — used for
                // A/B testing the two backends without re-baking probes or
                // restarting the renderer. AI hearing is unaffected
                // regardless: AIHearingService still calls
                // RoomService::propagateSoundPath directly (room-BFS), and
                // its discrete heard/not-heard semantics tolerate the BFS's
                // step change.
                bool useSteamAudioPathing = mDirectProbeBatchAdded
                                          && mProbePathingEnabled
                                          && !voice->playerEmitted
                                          && !voice->skipPortalRouting;

                // Run portal propagation to determine reachability and path.
                // Throttled: nearby voices update every frame, distant voices
                // every 8-16 frames. Matching the original engine's adaptive
                // update frequency. Uses cached result between updates.
                //
                // Player-emitted voices (footsteps, landings) are always in
                // the same "room" as the listener by definition — the source
                // IS the player. Skipping portal propagation entirely avoids
                // head↔feet straddling: foot-level sound emitter vs head-level
                // listener can flicker into different rooms at floor or ceiling
                // boundaries, falsely tripping isCrossRoom and routing the
                // voice through a portal anchor with door-LPF on the dry path.
                // Direct path stays at full volume; baked reflection routing
                // handles the wet path independently. This skip is the single
                // defense against that flicker — there is no longer a
                // belt-and-suspenders distance threshold below.
                SoundPropInfo prop{};
                bool isCrossRoom = false;
                if (!useSteamAudioPathing && mPortalRoutingEnabled
                    && !voice->sourceEnded
                    && !voice->skipPortalRouting && !voice->playerEmitted) {
                    // Run portal propagation every frame. Per-voice BFS averages
                    // ~3 µs (see `[Audio] portal=...avg=` stats), so ~20 voices ×
                    // 60 Hz = ~60 µs/frame total — negligible. The previous
                    // adaptive-throttle approach (FramesUntilUpdate = dist/10,
                    // capped at 16) was a 12.5 Hz-era CPU optimization. At 60+ Hz
                    // it produced 200–2000 ms of audible latency between
                    // listener-room changes and the voice's gain following, which
                    // is well above the perceptual threshold for ambient cues.
                    auto prT0 = profOn ? std::chrono::steady_clock::now()
                                       : std::chrono::steady_clock::time_point{};
                    // Use the voice's per-source maxAudibleDist instead
                    // of the global cap: BFS won't bother exploring
                    // paths longer than the schema's effective radius,
                    // so e.g. a wind ambient with radius=25 stops
                    // propagating at ~25 ft of portal-graph path even
                    // through open archways. Matches Dark Engine
                    // m_MaxDistance per-source semantics.
                    // Inline propagation call so we can capture the bend
                    // chain into voice->cachedChain for the show_vpos
                    // debug overlay. The wrapper propagateSound() doesn't
                    // expose params.chainOut.
                    if (mRoomService && mSoundPropagation) {
                        // Resolve source/listener rooms now so we can pass
                        // their IDs through to SoundPropagation. -1 means
                        // "outside all rooms" (euclidean fallback).
                        Room *srcRoomP = mRoomService->roomFromPoint(voice->worldPos);
                        Room *lstRoomP = mRoomService->roomFromPoint(mListenerPos);
                        int32_t srcID = srcRoomP ? srcRoomP->getRoomID() : -1;
                        int32_t lstID = lstRoomP ? lstRoomP->getRoomID() : -1;
                        // Build the params struct ourselves so we can plug
                        // the per-voice chainOut accumulator in (used by
                        // the show_vpos debug overlay). SoundPropagation
                        // overwrites doorBlocking / loudRoom with its own
                        // callbacks before forwarding to RoomService.
                        SoundPropParams pp;
                        pp.maxDist     = voice->maxAudibleDist;
                        pp.maxPaths    = mPropMaxPaths;
                        pp.maxPathDiff = mPropMaxPathDiff;
                        pp.chainOut    = &voice->cachedChain;
                        prop = mSoundPropagation->propagateSoundWithParams(
                            voice->worldPos, mListenerPos, srcID, lstID, pp);
                    } else {
                        prop = propagateSound(voice->worldPos, mListenerPos,
                                              voice->maxAudibleDist);
                    }
                    if (profOn) {
                        auto prT1 = std::chrono::steady_clock::now();
                        float prUs = std::chrono::duration<float, std::micro>(prT1 - prT0).count();
                        sPortalRoutingTotalUs.fetch_add(static_cast<int>(prUs), std::memory_order_relaxed);
                        sPortalRoutingCount.fetch_add(1, std::memory_order_relaxed);
                    }

                    voice->cachedProp = prop;

                    // Determine if the voice is in a different room from the listener.
                    // No physical-distance threshold here: a previous 5-ft gate was
                    // leftover defense against player-source head/feet straddling,
                    // which is now handled cleanly by the voice->playerEmitted skip
                    // above. With the gate in place, crossing 5 ft from a cross-room
                    // source caused two audible pops — an HRTF direction snap (real
                    // source → portal anchor) and a mute cliff when BFS failed
                    // (full volume → silence). Letting cross-room status track the
                    // actual room assignment removes both discontinuities.
                    Room *srcRoom = mRoomService ? mRoomService->roomFromPoint(voice->worldPos) : nullptr;
                    isCrossRoom = (srcRoom && listenerRoom && srcRoom != listenerRoom);
                }

                // Store propagation result on DSP node for the audio callback.
                //
                // portalAttenuation is a *continuous* excess-path
                // attenuation. The previous formula (1/(1+effDist²·0.001))
                // was gated by isCrossRoom, so it switched on at the portal
                // boundary, producing an audible pop. The new formula uses
                // the ratio of straight-line distance to portal-graph path,
                // which is:
                //   • 1.0 for same-room voices (real == effective)
                //   • < 1.0 only when the BFS path detours through portals,
                //   • continuous in the limit at the room boundary because
                //     a chain that's geometrically clean enough to threadle
                //     end-to-end produces real ≈ effective, yielding a
                //     ratio close to 1.0 just like the same-room case.
                //     Real/effective and door blocking themselves change
                //     discretely at the regime switch though — that
                //     residual step is one of the symptoms tracked in
                //     HANDOFF.SOUND_PROPAGATION_REALISM.
                if (useSteamAudioPathing) {
                    // Steam Audio pathing branch.
                    //
                    // portalAttenuation + portalBlocking will be overwritten
                    // from the per-voice pathing eqCoeffs read after
                    // iplSimulatorRunPathing fires in the simulation-step
                    // block below. Pre-seed neutral defaults so the audio
                    // thread reads stable values during this frame's
                    // window between dspNode write here and the eqCoeffs
                    // overwrite a few hundred microseconds later.
                    //
                    // usePortalRouting stays false: HRTF direction always
                    // points at the real source position, which the audio
                    // callback writes via the existing !usePortalRouting
                    // branch (toSource = voice->worldPos − mListenerPos).
                    // Pointing at "first bend in the pathing chain" is a
                    // Phase 5 refinement and tracked there.
                    voice->dspNode.usePortalRouting = false;
                    voice->dspNode.portalAttenuation = 1.0f;
                    voice->dspNode.portalBlocking    = 0.0f;

                    // maxAudibleDist hard cutoff — same authored value
                    // that gates AI hearing's BFS, so a wind ambient with
                    // radius=25 still goes silent at 25 ft of straight-
                    // line distance. Steam Audio's distance attenuation
                    // would taper to ~0 at this range anyway, but the
                    // schema-authored radius is treated as an absolute
                    // boundary in the original engine.
                    float lineDist = glm::length(voice->worldPos - mListenerPos);
                    if (lineDist > voice->maxAudibleDist) {
                        voice->dspNode.portalAttenuation = 0.0f;
                        voice->dspNode.portalBlocking    = 1.0f;
                    }
                } else if (prop.reached) {
                    // LPF cutoff is driven by *door* blocking only — not by
                    // the combined door+LoudRoom transmission loss
                    // (`totalBlocking`). LoudRoom is a level-designer
                    // omnidirectional attenuator per room subdivision and
                    // should not muffle the spectrum; routing it to the
                    // LPF cutoff produced audibly-muffled cross-subdivision
                    // ambients even when no door was in the path.
                    voice->dspNode.portalBlocking = prop.doorBlocking;
                    voice->dspNode.usePortalRouting = isCrossRoom;

                    if (prop.effectiveDistance > 0.001f) {
                        float ratio = prop.realDistance / prop.effectiveDistance;
                        // Clamp to [0,1]. The ratio cannot exceed 1 in
                        // theory (effectiveDistance ≥ realDistance for any
                        // physical path), but floating-point noise near the
                        // portal threshold can produce a hair above 1.0.
                        if (ratio > 1.0f) ratio = 1.0f;
                        if (ratio < 0.0f) ratio = 0.0f;
                        voice->dspNode.portalAttenuation = ratio * ratio;
                    } else {
                        // Listener is essentially on top of the source —
                        // no detour, full volume.
                        voice->dspNode.portalAttenuation = 1.0f;
                    }

                    // For cross-room voices, compute HRTF direction toward the
                    // virtual position (last portal anchor / doorway edge).
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
                } else if (isCrossRoom) {
                    // BFS failed to find a portal path between the source
                    // and listener rooms. Two distinct underlying causes:
                    //
                    //   (a) The source's effDist exceeded `maxDist` while
                    //       accumulating door-blocking penalties — there
                    //       IS a path, just heavily blocked. Silence +
                    //       full LPF is the correct outcome (matches the
                    //       Dark Engine "beyond the propagation horizon"
                    //       semantic).
                    //   (b) The source and listener are in disconnected
                    //       subgraphs of the portal graph. In a shipped
                    //       level this should not happen — it would mean
                    //       sounds are inaudible in places they should
                    //       reach, which the original engine would have
                    //       exhibited too. So if we see (b) on a real
                    //       mission, it indicates a parse or logic bug
                    //       on our end — chase that bug rather than
                    //       silently letting the audio through, which
                    //       would mask the root cause.
                    //
                    // Both branches collapse to the same audio result —
                    // a silenced, fully-muffled voice — but we log (b)
                    // so it surfaces during development.
                    voice->dspNode.portalAttenuation = 0.0f;
                    voice->dspNode.portalBlocking = 1.0f;
                    voice->dspNode.usePortalRouting = false;
                } else {
                    // No room data, no cross-room flag — treat as a fully
                    // unattenuated direct-path voice (Steam Audio handles
                    // distance/occlusion on its own).
                    voice->dspNode.usePortalRouting = false;
                    voice->dspNode.portalAttenuation = 1.0f;
                    voice->dspNode.portalBlocking = 0.0f;
                }

                // ── Phase 1: multi-path sub-source slot fan-out ──
                //
                // Populate voice->dspNode.subSources[] from this frame's
                // propagation paths so per-slot DSP state stays attached
                // to the physical path it represents (keyed by
                // SoundPathRecord::predecessorRoomID). The audio thread
                // does NOT consume these slots yet — the legacy single-
                // source pipeline above is still authoritative. The fan-
                // out runs in parallel so we can verify slot-assignment
                // stability against real BFS output before Phase 2 wires
                // up the audio-thread consumer. See
                // PLAN.MULTI_PATH_AMBISONICS.md for the rollout plan.
                //
                // The direction closure converts world-space
                // listener→path delta into IPL listener-local frame —
                // same right/up/ahead basis used by the legacy
                // node->direction write above so Phase 2 can drop the
                // legacy field without a frame shift.
                auto computeDirForPath = [&](const SoundPathRecord& p) -> IPLVector3 {
                    Vector3 toPath = p.virtualPosition - mListenerPos;
                    float   dist   = glm::length(toPath);
                    if (dist < 0.001f) {
                        return IPLVector3{0.0f, 0.0f, -1.0f};  // ahead fallback
                    }
                    toPath /= dist;
                    return IPLVector3{
                        glm::dot(toPath, right),
                        glm::dot(toPath, up),
                        -glm::dot(toPath, ahead)
                    };
                };

                const int maxN = static_cast<int>(std::min<uint32_t>(
                    sMaxSubSources.load(std::memory_order_relaxed),
                    static_cast<uint32_t>(kMaxSubSources)));

                if (prop.reached && !prop.paths.empty()) {
                    updateSubSourceSlots(voice->dspNode.subSources, prop,
                                         maxN, computeDirForPath);
                } else if (isCrossRoom) {
                    // BFS-failed cross-room: legitimately silenced.
                    // Drain any active slots.
                    drainAllSubSourceSlots(voice->dspNode.subSources);
                } else {
                    // No propagation result available (player-emitted
                    // voice, skipPortalRouting voice, portal routing
                    // globally disabled, or voice without room data).
                    // Treat as a direct-line same-room source so slot 0
                    // stays active and the audio callback's per-slot
                    // loop still produces output. Synthesize a single
                    // SoundPathRecord at voice->worldPos with no door
                    // blocking — matches the legacy
                    // (!usePortalRouting) audio behaviour where the
                    // dry path runs at full volume with direction =
                    // (worldPos - listenerPos).
                    SoundPropInfo synth;
                    synth.reached = true;
                    SoundPathRecord rec;
                    rec.predecessorRoomID = -1;   // same-room sentinel
                    rec.virtualPosition   = voice->worldPos;
                    Vector3 vd            = voice->worldPos - mListenerPos;
                    float   dist          = glm::length(vd);
                    rec.realDistance      = dist;
                    rec.effectiveDistance = dist;
                    rec.doorBlocking      = 0.0f;
                    rec.totalBlocking     = 0.0f;
                    synth.paths.push_back(rec);
                    synth.virtualPosition   = rec.virtualPosition;
                    synth.realDistance      = rec.realDistance;
                    synth.effectiveDistance = rec.effectiveDistance;
                    updateSubSourceSlots(voice->dspNode.subSources, synth,
                                         maxN, computeDirForPath);
                }

                // Architecture B: set the IPLSource position to
                // `prop.virtualPosition`. This collapses to the right thing
                // in every regime by construction:
                //   • Same-room (sourceRoom == listenerRoom): propagateSound
                //     returns virtualPosition == sourcePos, so Steam Audio
                //     sees the real source and traces occlusion rays through
                //     same-room geometry (pillars, furniture, etc.).
                //   • Cross-room reachable: virtualPosition is the last
                //     portal center along the BFS path, so Steam Audio
                //     traces rays from the doorway — the "occlusion-via-
                //     virtual-source" trick that makes cross-room voices
                //     audible without ray-tracing through walls.
                //   • BFS-failed cross-room (!prop.reached, isCrossRoom):
                //     virtualPosition is the default {0,0,0}. The voice is
                //     silenced by portalAttenuation = 0 from Change 2, so
                //     the bogus source position never reaches the listener
                //     audibly. We still fall back to worldPos here so the
                //     direction recompute downstream sees a meaningful
                //     vector if the BFS recovers next frame.
                //   • !prop.reached && !isCrossRoom (no room data at all):
                //     fall back to the real worldPos and let Steam Audio
                //     handle distance/occlusion in isolation.
                //
                // Edge case: virtualPosition can step at the moment the
                // listener crosses a room boundary, because the regime
                // switches from "real source" (same-room) to "portal
                // anchor" (cross-room) in one frame. roomFromPoint's
                // OBB+portal-plane disambiguator keeps that switch
                // deterministic — no per-frame flicker — but the step
                // is still audible as a small panning shift at the
                // doorway. Smoothing it out is one of the items
                // tracked in HANDOFF.SOUND_PROPAGATION_REALISM
                // (research direction 1, "smooth the regime
                // transition").
                // Engine→IPL conversion (feet→meters AND Z-up→Y-up) happens
                // via engineToIplPos so the source sits in the same frame
                // as the listener and mesh.
                IPLCoordinateSpace3 sourceCoord{};
                sourceCoord.origin = prop.reached
                    ? engineToIplPos(prop.virtualPosition)
                    : engineToIplPos(voice->worldPos);
                // Voices have no inherent orientation in this engine; pick an
                // arbitrary right-handed IPL-space basis (X right, Y up,
                // -Z ahead per phonon.h convention).  Steam Audio uses this
                // only for directional sources, which we don't drive.
                sourceCoord.ahead = { 0.0f,  0.0f, -1.0f };
                sourceCoord.right = { 1.0f,  0.0f,  0.0f };
                sourceCoord.up    = { 0.0f,  1.0f,  0.0f };

                IPLSimulationInputs inputs{};
                // DIRECT + PATHING share the same direct simulator handle;
                // REFLECTIONS lives on its own simulator. Pathing fields
                // below (pathingProbes, visRadius, visThreshold, visRange,
                // pathingOrder, enableValidation, findAlternatePaths) are
                // only read when the PATHING flag is set on the
                // iplSourceSetInputs call below, so the same struct can be
                // pushed to the reflection simulator with REFLECTIONS only
                // — Steam Audio ignores fields not selected by the flag
                // argument.
                inputs.flags = static_cast<IPLSimulationFlags>(
                    IPL_SIMULATIONFLAGS_DIRECT
                    | IPL_SIMULATIONFLAGS_REFLECTIONS
                    | IPL_SIMULATIONFLAGS_PATHING);
                // Player-emitted voices (footsteps, landings) emit from the
                // player's own body — there is no "wall" between the player's
                // feet and ears, so occlusion + transmission are skipped at
                // the Steam Audio API level. Distance attenuation, air
                // absorption, and reflections still run normally so footsteps
                // produce reverb the same way as any other voice.
                IPLDirectSimulationFlags directFlags = static_cast<IPLDirectSimulationFlags>(
                    IPL_DIRECTSIMULATIONFLAGS_DISTANCEATTENUATION |
                    IPL_DIRECTSIMULATIONFLAGS_AIRABSORPTION);
                if (!voice->playerEmitted) {
                    directFlags = static_cast<IPLDirectSimulationFlags>(
                        directFlags
                        | IPL_DIRECTSIMULATIONFLAGS_OCCLUSION
                        | IPL_DIRECTSIMULATIONFLAGS_TRANSMISSION);
                }
                inputs.directFlags = directFlags;
                inputs.source = sourceCoord;
                inputs.distanceAttenuationModel.type =
                    (sDistanceModel.load(std::memory_order_relaxed) == 1)
                        ? IPL_DISTANCEATTENUATIONTYPE_INVERSEDISTANCE
                        : IPL_DISTANCEATTENUATIONTYPE_DEFAULT;
                inputs.airAbsorptionModel.type = IPL_AIRABSORPTIONTYPE_DEFAULT;
                // Volumetric occlusion models the source as a sphere — as the
                // sphere partially disappears behind a corner, occlusion ramps
                // smoothly from 0 to 1 instead of snapping instantly (RAYCAST).
                // The 16 samples are points within the sphere used for ray tests.
                inputs.occlusionType = IPL_OCCLUSIONTYPE_VOLUMETRIC;
                // Volumetric occlusion samples N points from a sphere
                // CENTERED ON THE SOURCE and casts rays from each to the
                // listener. Fraction of unblocked rays = unoccluded
                // fraction.
                //
                // Sphere radius is controlled entirely by the YAML knob
                // `audio.occlusion.radius` (clamped 0.3–30 at config
                // load). No runtime floor — the value you set in the
                // config is exactly the value used.
                inputs.occlusionRadius = mAudioOcclusion->getRadiusMeters();
                inputs.numOcclusionSamples = mAudioOcclusion->getSamples();
                inputs.numTransmissionRays = 8;

                // Voices route to baked-probe reflections when:
                //   • They are not in the top-N closest (real-time budget
                //     reserved for the most audible source-position-sensitive
                //     voices), OR
                //   • They are player-emitted (footsteps, landings) — see the
                //     candidate-selection comment above for rationale: source
                //     ≈ listener so listener-position baked IR is correct, and
                //     it sidesteps the real-time sim's 200-400 ms latency
                //     which is longer than a footstep.
                bool isTopN = reflCandidateSet.count(handle) > 0;
                bool useBaked = (!isTopN || voice->playerEmitted)
                                && mProbeManager->hasReflections();
                if (useBaked) {
                    inputs.baked = IPL_TRUE;
                    inputs.bakedDataIdentifier = bakedReflId;
                }

                // Steam Audio pathing inputs — used only when the probe
                // batch has been attached to mDirectSimulator and the
                // voice is not player-emitted (player-emitted voices
                // emit from the listener, so the pathing graph collapses
                // to a self-loop with eqCoeffs ≡ 1.0; running it is wasted
                // work). visRange is the per-voice maximum audible
                // distance (the schema-authored radius), so Steam Audio
                // stops exploring long detours through the probe graph
                // beyond what the schema considers audible. pathingOrder
                // = 0 keeps the path effect mono — we drive the gain +
                // LPF through portalAttenuation / portalBlocking on the
                // existing DSP chain rather than as a spatialized
                // ambisonic field.
                if (mDirectProbeBatchAdded && mProbePathingEnabled
                    && !voice->playerEmitted) {
                    inputs.pathingProbes      = mProbeManager->getProbeBatch();
                    inputs.visRadius          = 0.5f;   // meters
                    inputs.visThreshold       = 0.1f;
                    inputs.visRange           = voice->maxAudibleDist * kFeetToMeters;
                    inputs.pathingOrder       = 0;
                    inputs.enableValidation   = IPL_TRUE;
                    inputs.findAlternatePaths = IPL_TRUE;
                }

                // Push inputs to both simulators. Each call only consumes
                // the fields relevant to its flag, so the same struct works
                // for both. Per phonon.h, no synchronisation is required
                // between the two calls — Steam Audio internally separates
                // direct- and reflection-side staging buffers.
                iplSourceSetInputs(voice->directSource,
                    IPL_SIMULATIONFLAGS_DIRECT, &inputs);
                // Pathing inputs ride on the same direct simulator handle
                // (we configured it with DIRECT|PATHING flags). The
                // setInputs call selects which subset of fields the
                // simulator consumes via the flags argument.
                if (mDirectProbeBatchAdded && mProbePathingEnabled
                    && !voice->playerEmitted) {
                    iplSourceSetInputs(voice->directSource,
                        IPL_SIMULATIONFLAGS_PATHING, &inputs);
                }
                // Stage 2.2: Normal voices without a reflectionSource
                // (not yet promoted, or recently demoted) have nothing on
                // the reflection side to update. Skip the call to avoid
                // dereferencing a null IPLSource inside Steam Audio.
                if (voice->reflectionSource) {
                    iplSourceSetInputs(voice->reflectionSource,
                        IPL_SIMULATIONFLAGS_REFLECTIONS, &inputs);
                }

                // Per-slot direct-source inputs (Phase 4 multi-path).
                // Each ACTIVE slot's source is positioned at its path's
                // virtualPosition — the doorway anchor for cross-room
                // paths, sourcePos for same-room. All other simulation
                // parameters (distance model, air absorption, occlusion
                // type/radius/samples, transmission rays) are inherited
                // from the voice-level `inputs` above so per-slot
                // sampling uses the same configuration knobs. The flags
                // word is copied verbatim — including playerEmitted's
                // OCCL/TRANS skip — because those are per-voice
                // characteristics that apply to every path.
                //
                // Draining slots intentionally retain their last-set
                // inputs: their gain is ramping down, the audio thread
                // skips reading their outputs, so refreshing inputs
                // would be wasted work.
                for (auto &slot : voice->dspNode.subSources) {
                    if (slot.state != SubSourceState::Active) continue;
                    if (!slot.directSource) continue;
                    Vector3 slotWorldPos = voice->worldPos;
                    for (const auto &p : prop.paths) {
                        if (p.predecessorRoomID == slot.predecessorRoomID) {
                            slotWorldPos = p.virtualPosition;
                            break;
                        }
                    }
                    IPLSimulationInputs slotInputs = inputs;
                    slotInputs.source.origin = engineToIplPos(slotWorldPos);
                    iplSourceSetInputs(slot.directSource,
                        IPL_SIMULATIONFLAGS_DIRECT, &slotInputs);
                }
            }

            // Step 3: Run direct sim and signal reflection thread.
            // Direct sim runs synchronously every frame.
            // Reflection sim runs on background thread, throttled.
            // The throttle counter / divisor live on ReflectionSimulator now.
            bool wantReflections = mReflectionsEnabled && mIplReflectionMixer
                && !reflBusy && mReflectionSim
                && mReflectionSim->throttleTickAndConsume();

            // Run direct sim synchronously on the main thread (2-5ms).
            // Same-frame results: every voice (including newly created ones)
            // gets real occlusion/distance/air absorption before the audio
            // callback sees them. Uses mDirectSimulator so the run never
            // contends with the reflection sim's background thread, and so
            // newly added direct sources are processed from frame 1 (no
            // defer-flush race for the direct path).
            {
                if (profOn) {
                    auto dt0 = std::chrono::steady_clock::now();
                    iplSimulatorRunDirect(mDirectSimulator);
                    auto dt1 = std::chrono::steady_clock::now();
                    float dMs = std::chrono::duration<float, std::milli>(dt1 - dt0).count();
                    float prevD = sDirectSimPeakMs.load(std::memory_order_relaxed);
                    if (dMs > prevD) sDirectSimPeakMs.store(dMs, std::memory_order_relaxed);
                } else {
                    iplSimulatorRunDirect(mDirectSimulator);
                }
            }

            // Run Steam Audio pathing on the same direct-sim handle. Cheap
            // when no probe batch is attached (mDirectProbeBatchAdded =
            // false) — Steam Audio skips per-source pathing work since
            // pathingProbes is null for every source. Driven by the
            // per-voice eqCoeffs read in the output block below. Also
            // bypassed entirely when mProbePathingEnabled is off
            // (audio.propagation.probe_pathing: false), so flipping the
            // toggle saves the full pathing-sim cost rather than just
            // discarding the outputs.
            if (mDirectProbeBatchAdded && mProbePathingEnabled) {
                iplSimulatorRunPathing(mDirectSimulator);
            }

            // Signal reflection sim (throttled, latency-tolerant, background thread)
            if (wantReflections && mReflectionSim) {
                mReflectionSim->signal();
            }
        }

        // Step 4: Read back simulation results and feed to DSP nodes.
        // Direct sim results are same-frame (ran synchronously above).
        // Reflection results are from the previous frame (background thread),
        // which is imperceptible since reverb tails change slowly.

        // reflCandidates and reflCandidateSet were already computed in Step 2a
        // (before iplSourceSetInputs) so non-top-N voices could be set to baked mode.

        // Track total convolution voices against the audio callback budget.
        // Tail voices (sourceEnded, still ringing out reverb) are counted first
        // since they already have reflectionsActive=true and can't be cheaply
        // disabled mid-convolution. Remaining slots go to active voices: top-N
        // closest first, then baked voices fill remaining up to mMaxReflectionVoices.
        int tailConvolutionCount = 0;
        for (auto &[h, v] : mVoicePool->voices()) {
            if (v->sourceEnded && !v->finished.load(std::memory_order_relaxed)
                && v->dspNode.reflectionsActive.load(std::memory_order_relaxed)) {
                ++tailConvolutionCount;
            }
        }
        int activeConvolutionCount = tailConvolutionCount;

        for (auto &[handle, voice] : mVoicePool->voices()) {
            if (!voice->directSource)
                continue;

            // Read direct outputs unconditionally — direct sim has no
            // background iteration, so directSource is always in the
            // simulator and outputs.direct is populated from this frame's
            // iplSimulatorRunDirect call above.
            IPLSimulationOutputs outputs{};
            iplSourceGetOutputs(voice->directSource,
                IPL_SIMULATIONFLAGS_DIRECT, &outputs);

            // Reflection outputs are only meaningful for sources that have
            // actually been added to the reflection simulator. If the source
            // is still pending an add (because the reflection sim was busy
            // at createVoiceSource time), iplSourceGetOutputs would return
            // uninitialized values. Keep the existing dspNode.reflectionParams
            // until the source is flushed and the sim processes it.
            bool isReflectionPending = !voice->reflectionSource
                || (mReflectionSim && mReflectionSim->isAddPending(voice->reflectionSource));
            if (mReflectionsEnabled && !isReflectionPending) {
                iplSourceGetOutputs(voice->reflectionSource,
                    IPL_SIMULATIONFLAGS_REFLECTIONS, &outputs);
            }

            // Steam Audio pathing → portalAttenuation + portalBlocking.
            //
            // Same gating as the setInputs block in Step 2a: voice must
            // not be player-emitted, must not be skipPortalRouting'd, and
            // the probe batch must be attached to the direct simulator.
            // When all three hold, eqCoeffs[3] is a smooth 3-band
            // attenuation envelope along the resolved probe-graph path —
            // 1.0 unobstructed, trending toward 0.0 as the path is
            // increasingly attenuated.
            //
            // Mapping (matches PLAN.PLAYER_AUDIO_STEAM_PATHING.md
            // "eqCoeffs → audio mapping"):
            //   gain   = 0.25·low + 0.50·mid + 0.25·high  → portalAttenuation
            //   t      = 1 − high                          → portalBlocking
            //
            // Skip when the voice's listener-source distance exceeds
            // maxAudibleDist (the schema-authored hard cutoff applied in
            // the Step 2a setInputs block above zeroed portalAttenuation
            // already). Skip when sourceEnded (no fresh source content;
            // the convolution tail is still ringing out the previous
            // pathing state).
            bool useSteamAudioPathing = mDirectProbeBatchAdded
                                      && mProbePathingEnabled
                                      && !voice->playerEmitted
                                      && !voice->skipPortalRouting;
            if (useSteamAudioPathing && !voice->sourceEnded) {
                float lineDist = glm::length(voice->worldPos - mListenerPos);
                if (lineDist <= voice->maxAudibleDist) {
                    IPLSimulationOutputs pathingOut{};
                    iplSourceGetOutputs(voice->directSource,
                        IPL_SIMULATIONFLAGS_PATHING, &pathingOut);

                    // ── Closed-door blocking layer (PLAN.* "Option A") ──
                    //
                    // Steam Audio's pathing graph is baked once, so it
                    // doesn't see runtime door state changes. We re-introduce
                    // door-state-dependent muffling here by querying the
                    // canonical room-pair blocking factor map (updated by
                    // door open/close logic via setBlockingFactor) for the
                    // source-room ↔ listener-room edge. The factor varies
                    // continuously with door angle (the door's JointPos
                    // interpolates 0..1), so the resulting LPF cutoff and
                    // gain reduction track door state smoothly.
                    //
                    // Implemented as a hashmap lookup rather than a BSP
                    // raycast (the spec's literal Option A): the raycast's
                    // "hit a closed door object → look up factor" collapses
                    // to the same outcome via the room-pair edge, which is
                    // the canonical place door logic writes its blocking
                    // value. O(1), so no Euclidean pre-filter against the
                    // nearest door is needed; we only enter this branch
                    // for cross-room voices anyway.
                    //
                    // Failure mode parallels the spec's raycast variant: a
                    // closed door that's NOT on the direct source ↔
                    // listener room boundary (sound routes around it) is
                    // not gated — correct geometrically, since the
                    // probe-graph path already routed around it.
                    float doorBlocking = 0.0f;
                    if (mSoundPropagation && mRoomService) {
                        Room *srcR = mRoomService->roomFromPoint(voice->worldPos);
                        Room *lstR = mRoomService->roomFromPoint(mListenerPos);
                        if (srcR && lstR && srcR != lstR) {
                            doorBlocking = mSoundPropagation->getBlockingFactor(
                                srcR->getRoomID(), lstR->getRoomID());
                        }
                    }

                    PathingDspMapping m = eqCoeffsToDspMapping(
                        pathingOut.pathing.eqCoeffs[0],
                        pathingOut.pathing.eqCoeffs[1],
                        pathingOut.pathing.eqCoeffs[2],
                        doorBlocking,
                        mPathingGainScale);

                    voice->dspNode.portalAttenuation = m.gain;
                    voice->dspNode.portalBlocking    = m.blocking;
                }
            }

            if (voice->dspNode.effectsReady) {
                // Always update HRTF direction — it doesn't depend on simulation
                // outputs and pending voices still need correct spatialization.
                if (!voice->dspNode.usePortalRouting) {
                    Vector3 toSource = voice->worldPos - mListenerPos;
                    float dist = glm::length(toSource);
                    if (dist > 0.001f) {
                        toSource /= dist;
                        // Project world direction onto listener-local axes.
                        // Steam Audio convention: +X=right, +Y=up, -Z=ahead,
                        // so negate the ahead component for correct front/back.
                        float dirX = glm::dot(toSource, right);
                        float dirY = glm::dot(toSource, up);
                        float dirZ = -glm::dot(toSource, ahead);
                        voice->dspNode.direction = { dirX, dirY, dirZ };
                    }
                }

                // Direct path is always written — direct sim ran this frame
                // for this source unconditionally. The old `if (isPending)
                // continue;` guard is gone: it was only needed because the
                // unified simulator coupled direct and reflection adds, so
                // a deferred reflection add silently broke the direct read
                // too. With split simulators that coupling is gone.
                voice->dspNode.directParams = outputs.direct;

                // For environmental ambient voices, SKIP distance attenuation —
                // the ambient system handles distance via ma_sound_set_volume
                // with a designer-controlled radius curve. Object ambients and
                // normal voices get full Steam Audio distance attenuation.
                if (voice->isAmbient) {
                    voice->dspNode.directParams.flags = static_cast<IPLDirectEffectFlags>(
                        IPL_DIRECTEFFECTFLAGS_APPLYAIRABSORPTION |
                        IPL_DIRECTEFFECTFLAGS_APPLYOCCLUSION |
                        IPL_DIRECTEFFECTFLAGS_APPLYTRANSMISSION);
                    // Override distanceAttenuation to 1.0 so the reflection
                    // convolution input isn't distance-scaled either
                    voice->dspNode.directParams.distanceAttenuation = 1.0f;
                } else if (voice->playerEmitted) {
                    // Player's own sounds (footsteps, landings) — skip
                    // occlusion + transmission (no "wall" between feet and
                    // ears) AND override distanceAttenuation to 1.0 so the
                    // dry path is full volume and the convolution input
                    // isn't distance-scaled. The room's reflection IR
                    // already encodes natural attenuation along bounce
                    // paths; layering source-listener distance on top would
                    // double-count it. Symmetric with the isAmbient branch.
                    voice->dspNode.directParams.flags = static_cast<IPLDirectEffectFlags>(
                        IPL_DIRECTEFFECTFLAGS_APPLYDISTANCEATTENUATION |
                        IPL_DIRECTEFFECTFLAGS_APPLYAIRABSORPTION);
                    voice->dspNode.directParams.distanceAttenuation = 1.0f;
                } else {
                    voice->dspNode.directParams.flags = static_cast<IPLDirectEffectFlags>(
                        IPL_DIRECTEFFECTFLAGS_APPLYDISTANCEATTENUATION |
                        IPL_DIRECTEFFECTFLAGS_APPLYAIRABSORPTION |
                        IPL_DIRECTEFFECTFLAGS_APPLYOCCLUSION |
                        IPL_DIRECTEFFECTFLAGS_APPLYTRANSMISSION);
                }
                voice->dspNode.directParams.transmissionType =
                    IPL_TRANSMISSIONTYPE_FREQDEPENDENT;

                // pathing_gain_scale knob — boost the dry-path distance
                // attenuation when Steam Audio pathing reports the voice
                // is substantially obstructed. The portalAttenuation
                // value computed above (= m.gain × pathing_gain_scale)
                // is currently only applied to the wet bus (see the
                // reflection-send `reflAtten *= currentPortalAtten` at
                // around AudioService.cpp:1334) — the dry path is driven
                // by IPL's DirectEffect, which reads distanceAttenuation
                // and occlusion but knows nothing about pathing eqCoeffs.
                //
                // Result without this block: wet bus responds to the
                // scale knob, dry path does not, which masks the knob
                // as "no audible change" for ambient sources whose wet
                // tail is small relative to the dry signal.
                //
                // The < 0.95 threshold scopes the boost to genuinely
                // obstructed voices (cross-room / occluded line-of-
                // sight). Same-room voices have m.gain ≈ 1 and don't
                // need lifting — without the gate, raising the scale
                // would brighten in-room voices too, which is not what
                // a "cross-room intelligibility" knob should do.
                if (useSteamAudioPathing && !voice->sourceEnded
                    && voice->dspNode.portalAttenuation < 0.95f) {
                    voice->dspNode.directParams.distanceAttenuation *= mPathingGainScale;
                }

                // ── Per-slot direct outputs (Phase 4 multi-path) ──
                //
                // Read each ACTIVE slot's iplSimulatorRunDirect outputs
                // into slot.targetDirectParams so the audio thread's
                // per-slot directEffect sees per-PATH distance
                // attenuation, air absorption, occlusion, and
                // transmission. The same per-class flag overrides as
                // the voice-level block above apply to every slot —
                // ambient and player-emitted voices keep
                // distanceAttenuation = 1.0 (the ambient/player volume
                // logic owns scaling on the dry path) and skip the
                // occlusion / transmission flags that don't physically
                // apply to them. Draining slots keep their last-set
                // params; the audio thread will continue running
                // direct effect on them with the previous frame's
                // params while their currentGain ramps to 0.
                for (auto &slot : voice->dspNode.subSources) {
                    if (slot.state != SubSourceState::Active) continue;
                    if (!slot.directSource) continue;
                    IPLSimulationOutputs slotOutputs{};
                    iplSourceGetOutputs(slot.directSource,
                        IPL_SIMULATIONFLAGS_DIRECT, &slotOutputs);
                    slot.targetDirectParams = slotOutputs.direct;
                    if (voice->isAmbient) {
                        slot.targetDirectParams.flags = static_cast<IPLDirectEffectFlags>(
                            IPL_DIRECTEFFECTFLAGS_APPLYAIRABSORPTION |
                            IPL_DIRECTEFFECTFLAGS_APPLYOCCLUSION |
                            IPL_DIRECTEFFECTFLAGS_APPLYTRANSMISSION);
                        slot.targetDirectParams.distanceAttenuation = 1.0f;
                    } else if (voice->playerEmitted) {
                        slot.targetDirectParams.flags = static_cast<IPLDirectEffectFlags>(
                            IPL_DIRECTEFFECTFLAGS_APPLYDISTANCEATTENUATION |
                            IPL_DIRECTEFFECTFLAGS_APPLYAIRABSORPTION);
                        slot.targetDirectParams.distanceAttenuation = 1.0f;
                    } else {
                        slot.targetDirectParams.flags = static_cast<IPLDirectEffectFlags>(
                            IPL_DIRECTEFFECTFLAGS_APPLYDISTANCEATTENUATION |
                            IPL_DIRECTEFFECTFLAGS_APPLYAIRABSORPTION |
                            IPL_DIRECTEFFECTFLAGS_APPLYOCCLUSION |
                            IPL_DIRECTEFFECTFLAGS_APPLYTRANSMISSION);
                    }
                    slot.targetDirectParams.transmissionType =
                        IPL_TRANSMISSIONTYPE_FREQDEPENDENT;
                }

                // Diagnostic: log Steam Audio direct params for door sounds
                if (voice->skipPortalRouting) {
                    const auto &dp = voice->dspNode.directParams;
                    AUDIO_LOG( "[DOOR_SND] h=%u '%s' distAtten=%.3f "
                                 "occl=%.3f trans=(%.2f,%.2f,%.2f) "
                                 "portalRoute=%d portalAtten=%.3f blocking=%.3f\n",
                                 handle, voice->schemaName.c_str(),
                                 dp.distanceAttenuation, dp.occlusion,
                                 dp.transmission[0], dp.transmission[1], dp.transmission[2],
                                 (int)voice->dspNode.usePortalRouting,
                                 voice->dspNode.portalAttenuation,
                                 voice->dspNode.portalBlocking);
                }

                // Enable reflection convolution up to the global budget.
                // Top-N closest voices (real-time) get priority, then baked
                // voices fill remaining slots. Total convolution voices are
                // capped at mMaxReflectionVoices to stay within the audio
                // callback budget (~1.5ms per convolution voice).
                //
                // Note: reflCandidates was already shrunk to
                // (mMaxReflectionVoices − tailCount) above, so in normal flow
                // an isReflVoice voice always has slot budget here.  We still
                // gate by canAffordConvolution as defence-in-depth — if the
                // cap was lowered at runtime, or a tail voice was missed in
                // the pre-count, this prevents overshoot.
                bool isReflVoice = reflCandidateSet.count(handle) > 0;
                bool canAffordConvolution = (activeConvolutionCount < mMaxReflectionVoices);
                bool hasBakedData = mProbeManager->hasReflections()
                                    && outputs.reflections.irSize > 0
                                    && voice->dspNode.reflectionEffect;
                // Pending reflection sources have no valid sim outputs yet
                // (we never called iplSourceGetOutputs on them above), so
                // outputs.reflections.irSize is zero for them — but
                // isReflVoice can still be true. Gate on !isReflectionPending
                // to prevent activating convolution with empty IR data and
                // wasting a slot on a source that the sim hasn't seen yet.
                bool enableRefl = canAffordConvolution
                                  && !isReflectionPending
                                  && (isReflVoice || hasBakedData);

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

                    // Diagnostic: confirm footstep voices entered the
                    // convolution path and capture the IR / direct params at
                    // activation. One-shot per voice so the log is readable
                    // even when many footsteps fire in quick succession.
                    if (voice->dspNode.isFootstepDiag && !voice->loggedReflActivationMain) {
                        voice->loggedReflActivationMain = true;
                        float voiceDist = glm::length(voice->worldPos - mListenerPos);
                        AUDIO_LOG("[FOOT_REFL_ON] h=%u '%s' dist=%.1f irSize=%d distAtten=%.3f "
                                  "occl=%.3f trans=(%.2f,%.2f,%.2f) src=%s ambiCh=%d\n",
                                  handle, voice->schemaName.c_str(), voiceDist,
                                  voice->dspNode.reflectionParams.irSize,
                                  voice->dspNode.directParams.distanceAttenuation,
                                  voice->dspNode.directParams.occlusion,
                                  voice->dspNode.directParams.transmission[0],
                                  voice->dspNode.directParams.transmission[1],
                                  voice->dspNode.directParams.transmission[2],
                                  isReflVoice ? "topN" : "baked",
                                  static_cast<int>(mAmbisonicsChannels));
                    }
                } else {
                    voice->dspNode.reflectionsActive.store(false, std::memory_order_relaxed);

                    // Diagnostic: log footsteps that were rejected from the
                    // convolution path, so we can tell "didn't get a slot" from
                    // "got a slot but no audible reverb."
                    if (voice->dspNode.isFootstepDiag && !voice->loggedReflActivationMain) {
                        voice->loggedReflActivationMain = true;
                        float voiceDist = glm::length(voice->worldPos - mListenerPos);
                        // Reason hierarchy:
                        //   topN-cap-full  → was top-N but budget was eaten by tails / runtime cap drop
                        //   baked-cap-full → had baked IR but cap was already full
                        //   not-topN-no-baked → not top-N AND no baked data → silently dry
                        const char *reason =
                            isReflVoice       ? "topN-cap-full" :
                            hasBakedData      ? "baked-cap-full" :
                                                "not-topN-no-baked";
                        AUDIO_LOG("[FOOT_REFL_OFF] h=%u '%s' dist=%.1f reason=%s "
                                  "irSize=%d slots=%d/%d hasBaked=%d\n",
                                  handle, voice->schemaName.c_str(), voiceDist,
                                  reason,
                                  outputs.reflections.irSize,
                                  activeConvolutionCount, mMaxReflectionVoices,
                                  hasBakedData ? 1 : 0);
                    }
                }

                // Direction update moved above isPending guard so all voices
                // (including pending ones) get correct HRTF spatialization.

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
    for (auto &[handle, voice] : mVoicePool->voices()) {
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
                    voice->tailTimer = mRealtimeDuration;
                    AUDIO_LOG( "[VOICE] TAIL_START h=%d '%s' tail=%.1fs\n",
                                 handle, voice->schemaName.c_str(), voice->tailTimer);
                } else {
                    voice->finished.store(true, std::memory_order_release);
                    AUDIO_LOG( "[VOICE] END_NOTAIL h=%d '%s'\n",
                                 handle, voice->schemaName.c_str());
                }
            } else {
                voice->tailTimer -= deltaTime;
                if (voice->tailTimer <= 0.0f) {
                    voice->finished.store(true, std::memory_order_release);
                    AUDIO_LOG( "[VOICE] TAIL_DONE h=%d '%s'\n",
                                 handle, voice->schemaName.c_str());
                }
            }
        }
    }

    // (ambient volumes updated at top of loopStep, before simulation)

    // loopStep total time (main thread). Skipped when audio_log is off —
    // see AudioLog.h.
    if (profOn) {
        auto loopStepEnd = std::chrono::steady_clock::now();
        float ms = std::chrono::duration<float, std::milli>(loopStepEnd - loopStepStart).count();
        float prev = sLoopStepPeakMs.load(std::memory_order_relaxed);
        if (ms > prev) sLoopStepPeakMs.store(ms, std::memory_order_relaxed);
    }

    // Periodic per-voice state snapshot (~1 Hz, gated behind audio_log).
    // Single line per voice covering everything Steam Audio is doing with
    // it: HRTF direction, propagation BFS state, cross-room routing,
    // reflection convolution path, dry-path attenuation values.
    //
    // Designed for diagnosing two specific question classes:
    //   1. "Why doesn't this sound seem to come from where the source is?"
    //      → check dir(LR,UD,FB). For footsteps should be ≈ (small, -1, ~0).
    //      Wrong direction = bug in the projection code. Right direction
    //      but no spatial cue = HRTF dataset limitation (below-listener
    //      sources are sparsely measured in standard binaural datasets).
    //   2. "Why doesn't this sound propagate from another room?"
    //      → check prop.reached / effD / block / xRoom. reached=0 for a
    //      sound that should be audible means the BFS isn't finding the
    //      path. effD much larger than crow-flies = routing through the
    //      wrong portal. block=1.0 = treated as fully blocked.
    {
        static float voiceDumpTimer = 0.0f;
        voiceDumpTimer += deltaTime;
        if (voiceDumpTimer >= 1.0f && Darkness::gAudioLogVerbose) {
            voiceDumpTimer = 0.0f;
            int16_t listenerRoomId = (mListenerRoom != nullptr)
                ? mListenerRoom->getRoomID() : -1;
            AUDIO_LOG("[VOICE_DUMP] listener pos=(%.0f,%.0f,%.0f) rm=%d "
                      "yaw=%.2f pitch=%.2f voices=%zu\n",
                      mListenerPos.x, mListenerPos.y, mListenerPos.z,
                      static_cast<int>(listenerRoomId),
                      mListenerYaw, mListenerPitch, mVoicePool->size());
            for (auto &[handle, voice] : mVoicePool->voices()) {
                Room *srcRoom = mRoomService
                    ? mRoomService->roomFromPoint(voice->worldPos) : nullptr;
                int16_t srcRoomId = (srcRoom != nullptr) ? srcRoom->getRoomID() : -1;
                Vector3 toListener = voice->worldPos - mListenerPos;
                float d = glm::length(toListener);
                const auto &p = voice->cachedProp;
                const auto &dir = voice->dspNode.direction;
                const auto &dp = voice->dspNode.directParams;
                bool reflActive = voice->dspNode.reflectionsActive.load(std::memory_order_relaxed);
                // We don't track per-voice baked-vs-realtime status directly;
                // infer "baked likely" if the voice is excluded from the
                // top-N candidate pool (playerEmitted with probes) or
                // simply not in the top-N this frame.
                const char *occlStr =
                    voice->playerEmitted ? "skip" : "active";
                // Calibration diagnostic: compute what the original
                // Dark Engine's volume formula would produce for this
                // voice's path, and log it next to our actual output
                // gain. If `oursLin` is materially below `origLin` the
                // voice is over-attenuated (Steam Audio distAtt +
                // engine portalAtt stacking past the original's single-
                // centibel-curve attenuation). If much higher, under-
                // attenuated. See APPSFX.CPP:964 + PSNDINST.CPP:1844.
                float origLin = 0.0f;
                float oursLin = 0.0f;
                if (mSchemaParser) {
                    const SchemaEntry *sch =
                        mSchemaParser->findSchema(voice->schemaName);
                    if (sch) {
                        int gainCb = sch->playParams.volume;
                        float attenFactor = sch->playParams.attenuationFactor;
                        if (attenFactor < 0.01f) attenFactor = 1.0f;
                        bool isSharp = (sch->playParams.flags & 0x1000) != 0;  // SFXFLG_SHARP
                        constexpr float kAtnFactor = 55.0f;
                        float mScaleDist = (5000.0f + gainCb) / kAtnFactor;
                        float mMaxDist   = mScaleDist * attenFactor;
                        float pathDist   = p.reached ? p.effectiveDistance : 1e9f;
                        float objDist;
                        if (isSharp) {
                            float t = (mMaxDist > 0.01f) ? (pathDist / mMaxDist) : 0.0f;
                            objDist = (t*t*t*t) * mScaleDist;
                        } else {
                            objDist = (mMaxDist > 0.01f)
                                          ? (pathDist / mMaxDist) * mScaleDist
                                          : pathDist;
                        }
                        objDist /= attenFactor;
                        float volumeCb = gainCb - objDist * kAtnFactor;
                        // cb→linear: 1 cb = 0.01 dB → linear = 10^(cb/2000).
                        origLin = std::pow(10.0f, volumeCb * 0.0005f);
                    }
                    // Our actual: schema_linear × portalAtten × distAtt.
                    // (HRTF + reflection contributions sit on top of this
                    //  but are direction/timing, not gain.)
                    if (sch) {
                        float schemaLin = std::pow(
                            10.0f, sch->playParams.volume * 0.0005f);
                        oursLin = schemaLin
                                * voice->dspNode.portalAttenuation
                                * dp.distanceAttenuation;
                    }
                }
                AUDIO_LOG("[VOICE] h=%u '%s' pE=%d amb=%d "
                          "pos=(%.0f,%.0f,%.0f) d=%.1f rm=%d | "
                          "prop: reached=%d effD=%.1f block=%.2f vPos=(%.0f,%.0f,%.0f) | "
                          "dir(LR,UD,FB)=(%+.2f,%+.2f,%+.2f) usePortal=%d "
                          "portalAtt=%.3f portalBlk=%.2f | "
                          "refl: active=%d irSize=%d | "
                          "dry: distAtt=%.3f occl=%s ended=%d | "
                          "calib: ours=%.4f orig=%.4f ratio=%.2f\n",
                          handle, voice->schemaName.c_str(),
                          voice->playerEmitted ? 1 : 0,
                          voice->isAmbient ? 1 : 0,
                          voice->worldPos.x, voice->worldPos.y, voice->worldPos.z,
                          d, static_cast<int>(srcRoomId),
                          p.reached ? 1 : 0, p.effectiveDistance, p.totalBlocking,
                          p.virtualPosition.x, p.virtualPosition.y, p.virtualPosition.z,
                          dir.x, dir.y, dir.z,
                          voice->dspNode.usePortalRouting ? 1 : 0,
                          voice->dspNode.portalAttenuation,
                          voice->dspNode.portalBlocking,
                          reflActive ? 1 : 0,
                          voice->dspNode.reflectionParams.irSize,
                          dp.distanceAttenuation, occlStr,
                          voice->sourceEnded.load(std::memory_order_relaxed) ? 1 : 0,
                          oursLin, origLin,
                          origLin > 1e-9f ? (oursLin / origLin) : 0.0f);
            }
        }
    }

    // Write position data to CSV if recording is active
    if (sRecording.load(std::memory_order_relaxed)) {
        static auto recordStart = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        float ms = std::chrono::duration<float, std::milli>(now - recordStart).count();
        int reflCount = 0;
        for (auto &[h, v] : mVoicePool->voices())
            if (v->dspNode.reflectionsActive) ++reflCount;
        FILE *posFile = std::fopen("darkness_audio_positions.csv", "a");
        if (posFile) {
            std::fprintf(posFile, "%.1f,%.2f,%.2f,%.2f,%.4f,%.4f,%zu,%d\n",
                         ms, mListenerPos.x, mListenerPos.y, mListenerPos.z,
                         mListenerYaw, mListenerPitch,
                         mVoicePool->size(), reflCount);
            std::fclose(posFile);
        }
    }
}

// AudioService::reflectionSimWorkerMain moved to
// ReflectionSimulator::workerMain (audio/ReflectionSimulator.cpp).
//
// AudioService::convolutionSubWorkerMain moved to
// ConvolutionWorkerPool::subWorkerMain (audio/ConvolutionWorkerPool.cpp).
//
// AudioService::waitForReflectionThread moved to
// ReflectionSimulator::waitForCompletion.
//
// AudioService::waitForConvolutionWorker moved to
// ConvolutionWorkerPool::waitForCompletion.
//
// AudioService::demoteFromRealtimeReflection moved to
// ReflectionSimulator::demoteVoice.


//------------------------------------------------------
void AudioService::cleanupFinishedVoices()
{
    if (!mVoicePool) return;
    // Facade — delegate iteration + erase to VoicePool, run the per-voice
    // diagnostic + IPL teardown via the hook. The pool checks each voice's
    // `finished` atomic (set by the audio thread's end callback) so we still
    // avoid cross-thread calls to ma_sound_at_end().
    std::size_t removed = mVoicePool->cleanupFinished(
        [this](ActiveVoice &v) {
            AUDIO_LOG( "[VOICE] CLEANUP h=%d '%s' srcEnded=%d tail=%.1f\n",
                         v.handle, v.schemaName.c_str(),
                         v.sourceEnded ? 1 : 0, v.tailTimer);
            // Lifetime-peak summary.  Captures the absolute loudest sample
            // the voice ever produced, not just whatever frame the
            // rate-limited per-voice DRY_BAL diagnostic happened to sample.
            // Logged once per voice at the moment of cleanup.  Useful for
            // diagnosing "footsteps fading in/out across jumps" — if max
            // peak varies wildly between identical voices, the DSP path
            // (HRTF, direct effect, master gain) is non-deterministic.
            float lpL = v.dspNode.lifetimePeakL.load(std::memory_order_relaxed);
            float lpR = v.dspNode.lifetimePeakR.load(std::memory_order_relaxed);
            int   lpN = v.dspNode.lifetimeFrameCount.load(std::memory_order_relaxed);
            float mIn  = v.dspNode.monoInPeak.load(std::memory_order_relaxed);
            float mOut = v.dspNode.monoOutPeak.load(std::memory_order_relaxed);
            float dpx = v.dspNode.directionAtPeakX.load(std::memory_order_relaxed);
            float dpy = v.dspNode.directionAtPeakY.load(std::memory_order_relaxed);
            float dpz = v.dspNode.directionAtPeakZ.load(std::memory_order_relaxed);
            float lpMax = std::max(lpL, lpR);
            float lpDb = (lpMax > 1e-6f) ? 20.0f * std::log10(lpMax) : -120.0f;
            // Stage ratios — if monoIn varies, decode/volume is the source.
            // If monoIn is consistent but monoOut varies, iplDirectEffect.
            // If monoOut is consistent but stereo varies, iplBinauralEffect.
            float dirMonoOut = (mIn  > 1e-6f) ? (mOut  / mIn)  : 0.0f;
            float dirStereo  = (mOut > 1e-6f) ? (lpMax / mOut) : 0.0f;
            AUDIO_LOG("[VOICE_PEAK] h=%d '%s' "
                      "monoIn=%.4f monoOut=%.4f stereoMax=%.4f stereoDb=%.1f "
                      "directGain=%.3f binGain=%.3f "
                      "dirAtPeak=(%.2f,%.2f,%.2f) "
                      "frames=%d pE=%d\n",
                      v.handle, v.schemaName.c_str(),
                      mIn, mOut, lpMax, lpDb,
                      dirMonoOut, dirStereo,
                      dpx, dpy, dpz, lpN,
                      v.playerEmitted ? 1 : 0);
            removeVoiceSource(v);
        });

    // Voice-lifecycle perf counter — only updated when audio_log
    // profiling is on (see AudioLog.h).
    if (removed > 0 && ::Darkness::gAudioLogVerbose)
        sVoicesDestroyed.fetch_add(static_cast<int>(removed),
                                   std::memory_order_relaxed);
}

//------------------------------------------------------
void AudioService::createVoiceSource(ActiveVoice &voice)
{
    IPLSimulator reflectionSimHandle = mReflectionSim ? mReflectionSim->simulator() : nullptr;
    if (!mDirectSimulator || !reflectionSimHandle || !mSceneReady)
        return;

    // ── Direct source ──
    // Created against mDirectSimulator. Direct sim runs synchronously on the
    // main thread so its source list is never iterated concurrently —
    // iplSourceAdd is always immediately safe. The voice's directParams are
    // populated by the next iplSimulatorRunDirect (≈one frame after creation),
    // so newly spawned voices are audible at the right level from their first
    // (or at worst second) audio callback.
    {
        IPLSourceSettings srcSettings{};
        srcSettings.flags = IPL_SIMULATIONFLAGS_DIRECT;

        IPLerror err = iplSourceCreate(mDirectSimulator, &srcSettings, &voice.directSource);
        if (err != IPL_STATUS_SUCCESS) {
            LOG_ERROR("AudioService: direct iplSourceCreate failed (error %d)", err);
            voice.directSource = nullptr;
            return;
        }
        iplSourceAdd(voice.directSource, mDirectSimulator);
        // No mSimulatorDirty for the direct sim — its commit happens inline
        // immediately below (no concurrency concern).
        iplSimulatorCommit(mDirectSimulator);
    }

    // ── Reflection source ──
    // Created eagerly for every voice so it can route through baked-probe
    // reverb (inputs.baked = true) by default. The per-frame top-N pass in
    // loopStep flips the baked flag off to upgrade the closest N voices to
    // ray-traced realtime reflections (see the inputs.baked logic later in
    // loopStep). Steam Audio's baked path iterates mSourceData[0] internally,
    // so a voice without a source-in-simulator would get neither realtime
    // NOR baked reverb — every voice starts with one so the baseline is
    // "voice has reverb."
    //
    // iplSourceAdd modifies the simulator's source list, which the
    // reflection sim thread iterates every cycle (200-800 ms). When that
    // thread is running, we MUST defer the add — adding mid-iteration would
    // race inside Steam Audio.
    //
    // Voices may later release their source via ReflectionSimulator::demoteVoice
    // — a sparing fallback that fires only after a voice has stayed out of
    // the top-N pool for the demote-hysteresis frame count. Default ≈10 s
    // means the demote is reserved for genuinely long-lived "stuck-distant"
    // voices; short-lived voices keep their baked reverb for their full
    // lifetime.
    {
        IPLSourceSettings srcSettings{};
        srcSettings.flags = IPL_SIMULATIONFLAGS_REFLECTIONS;

        IPLerror err = iplSourceCreate(reflectionSimHandle, &srcSettings, &voice.reflectionSource);
        if (err != IPL_STATUS_SUCCESS) {
            LOG_ERROR("AudioService: reflection iplSourceCreate failed (error %d)", err);
            // Direct source already created above; don't leak it. Caller
            // will treat the voice as having no spatial audio.
            iplSourceRemove(voice.directSource, mDirectSimulator);
            iplSimulatorCommit(mDirectSimulator);
            iplSourceRelease(&voice.directSource);
            voice.reflectionSource = nullptr;
            return;
        }

        if (mReflectionSim->isRunning()) {
            mReflectionSim->queueSourceAdd(voice.reflectionSource);
        } else {
            iplSourceAdd(voice.reflectionSource, reflectionSimHandle);
        }
        mReflectionSim->setSimulatorDirty();
        mReflectionSim->incrementActiveSources();
    }

    // ── Per-slot direct sources (multi-path ambisonics, Phase 4) ──
    //
    // Each sub-source slot owns its own IPLSource in the direct
    // simulator. Per-frame loopStep positions each ACTIVE slot's
    // source at that slot's path virtualPosition and writes the
    // appropriate simulation flags. iplSimulatorRunDirect then
    // computes per-path distance attenuation, air absorption,
    // occlusion, and transmission — read back into
    // slot.targetDirectParams and consumed by the per-slot
    // directEffect in the audio callback. The voice-level
    // voice.directSource above stays for reflection-related
    // bookkeeping (top-N ranking, reflection-send mono scaling) and
    // is no longer the source of the dry path's directParams.
    //
    // Pre-allocate ALL kMaxSubSources slots up front so the audio
    // thread never has to wait on Steam Audio source create/destroy
    // (which is main-thread-only). Idle slots have stale inputs;
    // iplSimulatorRunDirect still processes them but the outputs go
    // unread until the slot becomes ACTIVE.
    {
        IPLSourceSettings srcSettings{};
        srcSettings.flags = IPL_SIMULATIONFLAGS_DIRECT;
        int builtSlotSources = 0;
        for (auto &slot : voice.dspNode.subSources) {
            IPLerror err = iplSourceCreate(mDirectSimulator, &srcSettings,
                                           &slot.directSource);
            if (err != IPL_STATUS_SUCCESS) {
                LOG_ERROR("AudioService: per-slot iplSourceCreate failed "
                          "(error %d) at slot %d", err, builtSlotSources);
                // Roll back this slot's source and any previously-built
                // slot sources, plus the voice-level direct + reflection
                // handles, so the voice ends up with no Steam Audio state
                // (matches the legacy create-all-or-create-none policy).
                slot.directSource = nullptr;
                for (auto &s2 : voice.dspNode.subSources) {
                    if (s2.directSource) {
                        iplSourceRemove(s2.directSource, mDirectSimulator);
                        iplSourceRelease(&s2.directSource);
                    }
                }
                iplSimulatorCommit(mDirectSimulator);
                iplSourceRemove(voice.directSource, mDirectSimulator);
                iplSourceRelease(&voice.directSource);
                if (voice.reflectionSource) {
                    if (mReflectionSim && mReflectionSim->isRunning()) {
                        mReflectionSim->queueSourceRemove(voice.reflectionSource);
                        voice.reflectionSource = nullptr;
                    } else {
                        if (mReflectionSim && mReflectionSim->simulator()) {
                            iplSourceRemove(voice.reflectionSource,
                                            mReflectionSim->simulator());
                        }
                        iplSourceRelease(&voice.reflectionSource);
                    }
                }
                iplSimulatorCommit(mDirectSimulator);
                return;
            }
            iplSourceAdd(slot.directSource, mDirectSimulator);
            ++builtSlotSources;
        }
        // One commit after batching all kMaxSubSources adds, same as the
        // voice-level direct source's single commit above.
        iplSimulatorCommit(mDirectSimulator);
    }

    // Per-class default directParams are set in initVoiceDSP — see the
    // long comment there for why it has to happen there (audio thread races
    // the gap between initVoiceDSP and createVoiceSource).
}

//------------------------------------------------------
void AudioService::removeVoiceSource(ActiveVoice &voice)
{
    if (!voice.directSource && !voice.reflectionSource)
        return;

    // Invalidate this voice's effects BEFORE waiting for the worker.
    // The worker checks effectsReady before using the effect pointer, so
    // setting it false here ensures any in-flight processing skips this voice.
    // This closes the race where the worker hasn't started yet but has a
    // staging snapshot containing this voice's effect pointer.
    voice.dspNode.effectsReady.store(false, std::memory_order_release);
    voice.dspNode.reflectionsActive.store(false, std::memory_order_release);

    // Wait for the convolution worker to finish processing all current frames.
    if (mConvolutionPool) mConvolutionPool->waitForCompletion();

    // ── Direct source + per-slot direct sources ──
    // No background thread iterates the direct simulator, so removal is
    // always immediately safe. Batch the per-slot removes with the
    // voice-level remove and commit once at the end.
    if (mDirectSimulator) {
        if (voice.directSource) {
            iplSourceRemove(voice.directSource, mDirectSimulator);
        }
        for (auto &slot : voice.dspNode.subSources) {
            if (slot.directSource) {
                iplSourceRemove(slot.directSource, mDirectSimulator);
            }
        }
        iplSimulatorCommit(mDirectSimulator);
        if (voice.directSource) {
            iplSourceRelease(&voice.directSource);
        }
        for (auto &slot : voice.dspNode.subSources) {
            if (slot.directSource) {
                iplSourceRelease(&slot.directSource);
            }
        }
    }

    // ── Reflection source ──
    // Same defer-flush dance as before, just narrowed to reflectionSource.
    if (!voice.reflectionSource)
        return;

    // If this reflection source was deferred for add but never actually
    // added to the simulator, just release it directly. This check must
    // run BEFORE the sim-busy check — otherwise when sim threads are idle
    // we'd call iplSourceRemove on a source that was never iplSourceAdd'ed,
    // crashing inside Steam Audio (pointer authentication failure / SIGSEGV).
    if (mReflectionSim && mReflectionSim->removeFromPendingAdds(voice.reflectionSource)) {
        iplSourceRelease(&voice.reflectionSource);
        return;
    }

    // If the reflection sim thread is running, we can't safely call
    // iplSourceRemove (it races with iplSimulatorRunReflections).
    // Defer the removal — queue the IPL source handle for later cleanup.
    if (mReflectionSim && mReflectionSim->isRunning()) {
        mReflectionSim->queueSourceRemove(voice.reflectionSource);
        voice.reflectionSource = nullptr;  // detach from voice (we own it now)
        mReflectionSim->setSimulatorDirty();
        return;
    }

    if (mReflectionSim && mReflectionSim->simulator()) {
        iplSourceRemove(voice.reflectionSource, mReflectionSim->simulator());
        mReflectionSim->setSimulatorDirty();
    }
    iplSourceRelease(&voice.reflectionSource);
    if (mReflectionSim) mReflectionSim->decrementActiveSources();
}

//------------------------------------------------------
void AudioService::initVoiceDSP(ActiveVoice &voice)
{
    if (!mIplContext || !mIplHrtf || !mMaEngine)
        return;

    auto &dsp = voice.dspNode;
    const int rateDivisor = mReflectionSim ? mReflectionSim->getRateDivisor() : 2;
    dsp.hrtf = mIplHrtf;
    dsp.frameSize = static_cast<int>(mFrameSize);
    dsp.reflectionFrameSize = static_cast<int>(mReflectionFrameSize);
    dsp.rateDivisor = rateDivisor;

    // Allocate processing buffers (once, never reallocated — audio thread safe)
    dsp.monoScratch.resize(dsp.frameSize);
    dsp.directEffectOut.resize(dsp.frameSize, 0.0f);
    dsp.stereoL.resize(dsp.frameSize);
    dsp.stereoR.resize(dsp.frameSize);
    dsp.subSlotStereoL.resize(dsp.frameSize, 0.0f);
    dsp.subSlotStereoR.resize(dsp.frameSize, 0.0f);
    dsp.ambiScratch0.resize(mReflectionFrameSize, 0.0f);
    if (mAmbisonicsChannels > 1) dsp.ambiScratch1.resize(mReflectionFrameSize, 0.0f);
    if (mAmbisonicsChannels > 2) dsp.ambiScratch2.resize(mReflectionFrameSize, 0.0f);
    if (mAmbisonicsChannels > 3) dsp.ambiScratch3.resize(mReflectionFrameSize, 0.0f);
    if (rateDivisor > 1)
        dsp.decimatedMono.resize(mReflectionFrameSize, 0.0f);

    // Create IPLDirectEffect (per-voice, frequency-dependent 3-band EQ)
    // Must match device sample rate so effects process audio correctly.
    IPLAudioSettings audioSettings{};
    audioSettings.samplingRate = static_cast<IPLint32>(mDeviceSampleRate);
    audioSettings.frameSize = dsp.frameSize;

    IPLDirectEffectSettings directSettings{};
    directSettings.numChannels = 1;  // mono processing

    IPLBinauralEffectSettings binauralSettings{};
    binauralSettings.hrtf = mIplHrtf;

    // ── Per-slot direct + binaural effects (multi-path sub-source slots) ──
    //
    // Each of the kMaxSubSources slots owns its own pair of IPL effects so
    // that the per-slot HRTF interpolation state and direct-effect smoothing
    // state stay attached to the physical path mapped to that slot (keyed by
    // SoundPathRecord::predecessorRoomID in the slot fan-out — see
    // SubSourceSlots.h). All slot effects are pre-allocated up front so the
    // audio thread never needs to create / destroy IPL handles. In Phase 2
    // only one slot is ever ACTIVE at a time (sMaxSubSources clamped to 1),
    // but allocating all of them keeps the lifecycle uniform and lets
    // transitions briefly use a second slot for the draining tail without
    // touching the IPL handle pool from the audio thread.
    //
    // On any per-slot creation failure we tear down the slots we've already
    // built and bail out — matches the legacy single-effect "create or
    // refuse the voice" policy.
    IPLerror err = IPL_STATUS_SUCCESS;
    int      builtSlots = 0;
    for (int i = 0; i < kMaxSubSources; ++i) {
        SubSource &slot = dsp.subSources[i];

        err = iplDirectEffectCreate(mIplContext, &audioSettings,
                                    &directSettings, &slot.directEffect);
        if (err != IPL_STATUS_SUCCESS) {
            LOG_ERROR("AudioService: iplDirectEffectCreate failed for slot %d "
                      "(error %d)", i, err);
            break;
        }

        err = iplBinauralEffectCreate(mIplContext, &audioSettings,
                                      &binauralSettings, &slot.binauralEffect);
        if (err != IPL_STATUS_SUCCESS) {
            LOG_ERROR("AudioService: iplBinauralEffectCreate failed for slot %d "
                      "(error %d)", i, err);
            iplDirectEffectRelease(&slot.directEffect);
            slot.directEffect = nullptr;
            break;
        }

        ++builtSlots;
    }

    if (err != IPL_STATUS_SUCCESS) {
        // Roll back the slots we did build so the voice is left with no
        // partially-initialised IPL effects (matches legacy: create-all-
        // or-create-none).
        for (int i = 0; i < builtSlots; ++i) {
            SubSource &slot = dsp.subSources[i];
            if (slot.binauralEffect) {
                iplBinauralEffectRelease(&slot.binauralEffect);
                slot.binauralEffect = nullptr;
            }
            if (slot.directEffect) {
                iplDirectEffectRelease(&slot.directEffect);
                slot.directEffect = nullptr;
            }
        }
        return;
    }

    // Pre-warm each slot's binaural effect's overlap-add history buffer with
    // one frame of silence. Without this, the first real audio frame for a
    // new voice gets attenuated by ~50% because overlap-add convolution
    // against a zero-history buffer drops the leading samples.
    //
    // Audible only on short transients (footsteps, impacts, glass breaks)
    // where the leading edge IS the perceived sound. Long voices fade in
    // imperceptibly. Voices spawn at arbitrary phases within the audio
    // buffer cycle, so the impact transient lands in the first frame for
    // some voices and in subsequent frames for others — producing an A/B
    // alternation in perceived loudness unrelated to source position.
    //
    // One-time ~50 µs per slot at voice creation; negligible.
    {
        std::vector<float> warmMono(mFrameSize, 0.0f);
        std::vector<float> warmL(mFrameSize, 0.0f);
        std::vector<float> warmR(mFrameSize, 0.0f);
        float *warmInPtr = warmMono.data();
        IPLAudioBuffer warmIn{};
        warmIn.numChannels = 1;
        warmIn.numSamples = static_cast<IPLint32>(mFrameSize);
        warmIn.data = &warmInPtr;

        float *warmOutChans[2] = {warmL.data(), warmR.data()};
        IPLAudioBuffer warmOut{};
        warmOut.numChannels = 2;
        warmOut.numSamples = static_cast<IPLint32>(mFrameSize);
        warmOut.data = warmOutChans;

        IPLBinauralEffectParams warmParams{};
        warmParams.direction = {0.0f, 0.0f, -1.0f};  // ahead, neutral
        warmParams.interpolation = IPL_HRTFINTERPOLATION_BILINEAR;
        warmParams.spatialBlend = 1.0f;
        warmParams.hrtf = mIplHrtf;
        warmParams.peakDelays = nullptr;
        for (auto &slot : dsp.subSources) {
            if (slot.binauralEffect) {
                iplBinauralEffectApply(slot.binauralEffect, &warmParams,
                                       &warmIn, &warmOut);
            }
        }
    }

    // Create per-voice reflection convolution effect (optional — only if mixer exists).
    // Uses the reflection sample rate (24kHz or 48kHz) to match the mixer and simulator.
    if (mIplReflectionMixer) {
        IPLint32 irSize = static_cast<IPLint32>(mRealtimeDuration * mReflectionSampleRate);

        IPLAudioSettings reflAudioSettings{};
        reflAudioSettings.samplingRate = static_cast<IPLint32>(mReflectionSampleRate);
        reflAudioSettings.frameSize = static_cast<IPLint32>(mReflectionFrameSize);

        IPLReflectionEffectSettings reflSettings{};
        reflSettings.type = (mReflectionType == ReflectionType::Hybrid)
            ? IPL_REFLECTIONEFFECTTYPE_HYBRID
            : (mReflectionType == ReflectionType::Parametric)
                ? IPL_REFLECTIONEFFECTTYPE_PARAMETRIC
                : IPL_REFLECTIONEFFECTTYPE_CONVOLUTION;
        reflSettings.irSize = irSize;
        reflSettings.numChannels = static_cast<IPLint32>(mAmbisonicsChannels);

        err = iplReflectionEffectCreate(mIplContext, &reflAudioSettings,
                                         &reflSettings, &dsp.reflectionEffect);
        if (err == IPL_STATUS_SUCCESS) {
            dsp.reflectionMixer = mIplReflectionMixer;
            dsp.convWorker = mConvolutionPool ? mConvolutionPool->worker() : nullptr;
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
        // Roll back all per-slot IPL effects pre-allocated above. Same
        // create-all-or-create-none policy that the per-slot loop itself
        // enforces — if the node graph can't be wired up we leave the
        // voice with no Steam Audio state at all.
        for (auto &slot : dsp.subSources) {
            if (slot.binauralEffect) {
                iplBinauralEffectRelease(&slot.binauralEffect);
                slot.binauralEffect = nullptr;
            }
            if (slot.directEffect) {
                iplDirectEffectRelease(&slot.directEffect);
                slot.directEffect = nullptr;
            }
        }
        return;
    }
    dsp.nodeInitialized = true;

    // Per-class default directParams — written here BEFORE the audio graph
    // is wired (a few lines below) so the audio thread reads correct values
    // from its very first callback. This must happen here, not in
    // createVoiceSource, because:
    //   1. ma_node_attach_output_bus + effectsReady=true (further below)
    //      let the audio thread fire processCallback immediately.
    //   2. Empirical evidence (see [DPARAM_READ] cb=0/cb=1 vs [CVS_DEFAULT]
    //      log ordering) shows the audio thread can fire ≥2 callbacks for
    //      a fresh node before startVoice's call to createVoiceSource even
    //      starts — long enough to expose ~42 ms of wrong-volume audio.
    // voice.playerEmitted / voice.isAmbient are set by startVoice's switch
    // statement BEFORE this function runs (see VoiceClass plumbing).
    //
    // Player-emitted (footsteps, landings): source ≈ listener, so distance
    // attenuation and occlusion are physically meaningless. Match the
    // per-frame loopStep override at line ~3286: distAtt = 1.0,
    // flags = APPLYDISTANCEATTENUATION | APPLYAIRABSORPTION (the DIST
    // flag is harmless because we set distAtt = 1.0 — keeps the audio
    // thread's effective gain identical between this default and the
    // loopStep override on subsequent frames).
    //
    // Ambient: distance handled by the ambient system's volume curve, not
    // Steam Audio. Match the per-frame loopStep override: distAtt = 1.0,
    // flags = AIRABSORPTION | OCCLUSION | TRANSMISSION (no DIST so the
    // ambient system's volume curve is the only attenuation).
    //
    // Regular positional: silent default (distAtt = 0.0,
    // flags = APPLYDISTANCEATTENUATION | APPLYOCCLUSION). The first audio
    // callback may produce silence for ~21 ms before the next loopStep
    // populates real direct-sim values; with the split simulator this is
    // bounded to one loopStep regardless of reflection-sim state.
    dsp.directParams.transmissionType = IPL_TRANSMISSIONTYPE_FREQDEPENDENT;
    if (voice.playerEmitted) {
        dsp.directParams.flags = static_cast<IPLDirectEffectFlags>(
            IPL_DIRECTEFFECTFLAGS_APPLYDISTANCEATTENUATION |
            IPL_DIRECTEFFECTFLAGS_APPLYAIRABSORPTION);
        dsp.directParams.distanceAttenuation = 1.0f;
    } else if (voice.isAmbient) {
        dsp.directParams.flags = static_cast<IPLDirectEffectFlags>(
            IPL_DIRECTEFFECTFLAGS_APPLYAIRABSORPTION |
            IPL_DIRECTEFFECTFLAGS_APPLYOCCLUSION |
            IPL_DIRECTEFFECTFLAGS_APPLYTRANSMISSION);
        dsp.directParams.distanceAttenuation = 1.0f;
    } else {
        dsp.directParams.flags = static_cast<IPLDirectEffectFlags>(
            IPL_DIRECTEFFECTFLAGS_APPLYDISTANCEATTENUATION |
            IPL_DIRECTEFFECTFLAGS_APPLYOCCLUSION);
        dsp.directParams.distanceAttenuation = 0.0f;
    }
    dsp.directParams.airAbsorption[0] = 1.0f;
    dsp.directParams.airAbsorption[1] = 1.0f;
    dsp.directParams.airAbsorption[2] = 1.0f;
    dsp.directParams.occlusion = 0.0f;
    dsp.directParams.transmission[0] = 0.0f;
    dsp.directParams.transmission[1] = 0.0f;
    dsp.directParams.transmission[2] = 0.0f;

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

    // Create the shared validity token for the convolution worker.
    // This is a heap-allocated atomic<bool> shared between the voice and any
    // staging slots that reference its reflectionEffect. When the voice is
    // destroyed, ~ActiveVoice sets the token to false; the worker's shared_ptr
    // copy keeps the token alive so it can be safely checked without UB.
    dsp.validityToken = std::make_shared<std::atomic<bool>>(true);

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

    for (const auto &[handle, voice] : mVoicePool->voices()) {
        if (voice->priority < lowestPriority) {
            lowestPriority = voice->priority;
            lowestHandle = handle;
        }
    }

    if (lowestHandle == SOUND_HANDLE_INVALID)
        return false;  // All existing voices have equal or higher priority

    AUDIO_LOG( "[VOICE] EVICT h=%d pri=%d for new pri=%d\n",
                 lowestHandle, lowestPriority, newPriority);
    haltSound(lowestHandle);
    return true;
}

//------------------------------------------------------
SoundHandle AudioService::startVoice(const std::string &schemaName,
                                      const std::string &sampleName,
                                      const Vector3 &position,
                                      int priority, bool looping,
                                      int objID, float volume,
                                      VoiceClass cls)
{
    if (!mMaEngine || !mSoundLoader)
        return SOUND_HANDLE_INVALID;

    // Enforce voice limit — evict lowest priority if full.
    // Effective cap is min(config, compile-time max for array sizing).
    int effectiveCap = std::min(mMaxActiveVoicesCfg, MAX_ACTIVE_VOICES);
    if (static_cast<int>(mVoicePool->size()) >= effectiveCap) {
        if (!evictLowestPriority(priority)) {
            AUDIO_LOG( "[VOICE] POOL_FULL cannot play '%s' pri=%d voices=%zu\n",
                         schemaName.c_str(), priority, mVoicePool->size());
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
        // Skip if this sample has already failed once — avoids re-walking
        // the CRF index every frame for ambients that reference missing
        // assets (e.g. mission .sch points at a sample not in snd.crf).
        if (mFailedSamples.count(sampleName))
            return SOUND_HANDLE_INVALID;

        snd = mSoundLoader->loadSound(sampleName);
        if (!snd.valid()) {
            LOG_ERROR("AudioService: failed to load sample '%s' for schema '%s'",
                      sampleName.c_str(), schemaName.c_str());
            mFailedSamples.insert(sampleName);
            return SOUND_HANDLE_INVALID;
        }
        if (mSoundCache) {
            mSoundCache->put(sampleName, snd);
        }
    }

    auto voice = std::make_unique<ActiveVoice>();
    voice->data = std::move(snd);
    voice->handle = mVoicePool->allocate();
    voice->schemaName = schemaName;
    voice->priority = priority;

    // Diagnostic: tag footstep / landing voices for reflection-path tracing.
    // Used by main- and audio-thread one-shot logs to confirm whether transient
    // sounds reach the convolution effect. Schema names follow Dark Engine
    // conventions: "foot_*" for ground footsteps, "land_*" for landing impacts.
    bool isFoot = (schemaName.compare(0, 5, "foot_") == 0
                   || schemaName.compare(0, 5, "land_") == 0);
    voice->dspNode.isFootstepDiag = isFoot;
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

    // Apply voice classification BEFORE initVoiceDSP / createVoiceSource so
    // the per-class default directParams written in createVoiceSource see
    // the right voice.playerEmitted / voice.isAmbient values. Otherwise the
    // first audio callback after this voice is created reads the silent
    // initVoiceDSP defaults (distAtt=0, flags=0x9), and only the second
    // callback sees the per-frame loopStep override take effect — producing
    // ~21 ms of wrong-volume audio at the attack of every transient.
    switch (cls) {
        case VoiceClass::PlayerEmitted: voice->playerEmitted = true; break;
        case VoiceClass::Ambient:       voice->isAmbient     = true; break;
        case VoiceClass::Normal:        break;
    }

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
            AUDIO_LOG( "[VOICE] BYPASS '%s' — Steam Audio disabled via env\n",
                         schemaName.c_str());
        } else {
            AUDIO_LOG( "[VOICE] DSP_FAIL '%s' sample='%s' — raw bypass to endpoint\n",
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

    AUDIO_LOG( "[VOICE] START h=%d '%s' sample='%s' pri=%d voices=%zu "
                 "wavBytes=%zu decoded=%lluframes %uHz %uch %.0fms\n",
                 h, schemaName.c_str(), sampleName.c_str(),
                 priority, mVoicePool->size() + 1,
                 voice->data.wavData.size(), (unsigned long long)totalFrames,
                 decRate, decCh, durMs);
    mVoicePool->insert(h, std::move(voice));
    // Voice-lifecycle perf counter — only updated when audio_log profiling
    // is on (see AudioLog.h).
    if (::Darkness::gAudioLogVerbose)
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
SoundHandle AudioService::playEnvSchema(const std::vector<SchemaTagValue> &tags,
                                         const Vector3 &position,
                                         bool bypassPortalBlocking)
{
    if (!mAudioReady || !mSchemaParser)
        return SOUND_HANDLE_INVALID;

    auto matches = mSchemaParser->findByEnvTags(tags);
    if (matches.empty())
        return SOUND_HANDLE_INVALID;

    // Use the first matching schema (highest priority match)
    const SchemaEntry *schema = matches[0];
    if (schema->samples.empty())
        return SOUND_HANDLE_INVALID;

    // Weighted random sample selection (same as playSchema)
    std::vector<int> freqs;
    freqs.reserve(schema->samples.size());
    for (const auto &s : schema->samples)
        freqs.push_back(s.frequency);

    int totalFreq = schema->totalFrequency();
    if (totalFreq <= 0)
        return SOUND_HANDLE_INVALID;

    int idx = selectSample(schema->name, static_cast<int>(schema->samples.size()),
                           totalFreq, freqs.data());
    if (idx < 0 || idx >= static_cast<int>(schema->samples.size()))
        return SOUND_HANDLE_INVALID;

    if ((schema->playParams.flags & SCH_NO_REPEAT) && schema->samples.size() > 1) {
        auto it = mLastSampleIdx.find(schema->name);
        if (it != mLastSampleIdx.end() && it->second == idx) {
            idx = selectSample(schema->name, static_cast<int>(schema->samples.size()),
                               totalFreq, freqs.data());
            if (idx < 0 || idx >= static_cast<int>(schema->samples.size()))
                return SOUND_HANDLE_INVALID;
        }
    }
    mLastSampleIdx[schema->name] = idx;

    const SchemaSample &sample = schema->samples[idx];
    bool looping = schema->loopParams.isLooping;
    float vol = schemaVolumeToLinear(schema->playParams.volume);

    // Local sounds (door open/close) get a volume boost to compensate for
    // Steam Audio's geometry-based distance attenuation which stacks on top
    // of the schema's authored volume. The original engine had no such
    // additional attenuation layer. 2x boost (~+6dB) restores audibility.
    if (bypassPortalBlocking) {
        vol = std::min(vol * 2.0f, 1.0f);
    }

    SoundHandle h = startVoice(schema->name, sample.name, position,
                               schema->playParams.priority, looping, 0, vol);
    // Local sounds skip portal-based blocking so they're audible from both
    // sides of a door. Steam Audio HRTF and distance attenuation still apply.
    if (ActiveVoice *dv = (bypassPortalBlocking ? mVoicePool->find(h) : nullptr)) {
        dv->skipPortalRouting = true;
        AUDIO_LOG( "[DOOR_SND] CREATE h=%u '%s' vol=%.3f "
                     "schemaVol=%d pos=(%.1f,%.1f,%.1f)\n",
                     h, schema->name.c_str(), vol,
                     schema->playParams.volume,
                     position.x, position.y, position.z);
    }
    return h;
}

//------------------------------------------------------
void AudioService::haltSound(SoundHandle handle, int fadeMs)
{
    if (handle == SOUND_HANDLE_INVALID)
        return;

    ActiveVoice *vp = mVoicePool ? mVoicePool->find(handle) : nullptr;
    if (vp) {
        auto &voice = *vp;
        AUDIO_LOG( "[VOICE] HALT h=%d '%s' hasRefl=%d fadeMs=%d\n",
                     handle, voice.schemaName.c_str(),
                     voice.dspNode.reflectionEffect ? 1 : 0, fadeMs);
        if (voice.initialized) {
            // Fade out before stopping. Default 15 ms suppresses click/pop
            // from an abrupt cut; callers that gracefully retire a voice
            // (e.g. an ambient aged out of audible range) pass a longer
            // fade so the listener perceives a smooth fade-out instead of
            // a hard cut. miniaudio schedules the stop after the fade
            // completes.
            if (fadeMs < 1) fadeMs = 1;
            ma_sound_stop_with_fade_in_milliseconds(&voice.sound,
                                                    static_cast<ma_uint64>(fadeMs));
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
    if (mReflectionSim) mReflectionSim->waitForCompletion();

    // Remove all Steam Audio sources before destroying voices.
    // Since we just joined both threads, removeVoiceSource will take the
    // immediate path (no deferral needed). The pool's clear() runs the
    // hook once per voice and then drops the map.
    if (mVoicePool) {
        mVoicePool->clear([this](ActiveVoice &voice) {
            if (voice.initialized) {
                ma_sound_stop(&voice.sound);
            }
            removeVoiceSource(voice);
        });
    }

    // Flush any pending deferred adds/removals. Pending adds were never added
    // to the simulator — just release them. Pending removals were already
    // detached from their voices — remove from simulator and release.
    if (mReflectionSim) {
        mReflectionSim->releasePendingAdds();
        mReflectionSim->flushPendingRemovals();
    }

    if (mVoicePool) mVoicePool->resetAllocator();
}

// ── Ambient sound system ──

//------------------------------------------------------
// Auxiliary per-schema / per-room sound data loaders. The actual
// P$AmbientHack and P$SpotAmb parsing has moved to AmbientSoundManager;
// this function does the work that previously preceded it in the same
// load step so the call ordering (and any cross-coupling — e.g. SchAttFac
// must apply before ambient parsing so its attenuationFactor reaches the
// AmbientSound records) is preserved.
void AudioService::loadAuxiliarySoundData()
{
    if (!mPropertyService || !mObjectService || !mAudioReady)
        return;

    // Load per-schema attenuation factor overrides from P$SchAttFac. The
    // property exists on schema-archetype objects in dark.gam; the
    // property's float value becomes the schema's volume-falloff
    // divisor (default 1.0). Schemas with values > 1.0 fall off less
    // aggressively — e.g. MISS6's m06bell has 20.0, so the bell stays
    // audible across essentially its full radius instead of dropping
    // -50 dB at the edge. AI dialog uses ~1.7, weapon hits 1.5–3, etc.
    //
    // Match the schema-archetype object to our parsed SchemaEntry by
    // the object's SymName (case-insensitive). Ignore objects we don't
    // recognize (some schema archetypes in dark.gam may not have a
    // corresponding .sch file in the schema dir we loaded).
    if (mSchemaParser) {
        Property *attFacProp = mPropertyService->getProperty("SchAttFac");
        if (attFacProp && attFacProp->getStorage()) {
            DataStorage *storage = attFacProp->getStorage();
            IntIteratorPtr it = storage->getAllStoredObjects();
            int applied = 0;
            while (!it->end()) {
                int objID = it->next();
                size_t sz = 0;
                const uint8_t *bytes = storage->getRawData(objID, sz);
                if (!bytes || sz < sizeof(float)) continue;
                float factor;
                std::memcpy(&factor, bytes, sizeof(float));
                if (factor <= 0.0f) continue;  // sanity

                std::string schemaName = mObjectService->getName(objID);
                if (schemaName.empty()) continue;
                SchemaEntry *sch = mSchemaParser->findSchemaMutable(schemaName);
                if (!sch) continue;
                sch->playParams.attenuationFactor = factor;
                ++applied;
            }
            AUDIO_LOG("AudioService: applied P$SchAttFac to %d schemas\n",
                      applied);
        }
    }

    // Overlay per-archetype schema property overrides from dark.gam onto
    // our SchemaEntry table. These properties carry authoritative play /
    // loop / priority / message data for schemas; .sch files are an
    // authoring surface, but the gamesys stores the canonical values.
    loadSchemaPropertyOverrides();

    // Load miscellaneous sound properties (P$PrjSound, P$Heartbeat,
    // P$SchLastSa). Per-archetype data the runtime consults at specific
    // moments (projectile spawn, heartbeat schema trigger, no-repeat
    // sample selection across save/load).
    loadMiscSoundProperties();

    // Load spot ambients (P$SpotAmb) via AmbientSoundManager. These are
    // an alternative ambient encoding with an inner/outer falloff envelope
    // distinct from the single-radius P$AmbientHack model.
    if (mAmbientManager) mAmbientManager->loadSpotAmbients();

    // Load per-room LoudRoom transmission factors from room object properties.
    // LoudRoom is a single float (default 1.0) that multiplicatively scales
    // sound energy passing through a room during portal propagation.
    // Values < 1.0 dampen (closets, padded rooms), > 1.0 amplify (marble halls).
    // Storage now lives in SoundPropagation — clearing happens at the same
    // call site as mBlockingFactors (onDBDrop and clear()).
    if (mRoomService && mSoundPropagation) {
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
                    mSoundPropagation->setRoomTransmission(
                        room->getRoomID(), transmission);
                    ++loudRoomCount;
                }
            }
        }
        if (loudRoomCount > 0) {
            AUDIO_LOG( "AudioService: loaded %d LoudRoom transmission factors\n",
                         loudRoomCount);
        }

        // Load P$Acoustics property per room and log comparison with ROOM_EAX presets.
        // This verifies that Steam Audio's physics-based reverb produces results
        // comparable to the original designers' EAX preset intentions.
        if (!mRoomEAXPresets.empty()) {
            AUDIO_LOG( "\n=== Room Acoustic Verification (EAX presets vs Steam Audio) ===\n");
            AUDIO_LOG( "  Room | ObjID | EAX Preset         | Decay(s) | Damp | Height | LoudRoom\n");
            AUDIO_LOG( "  -----|-------|--------------------|---------:|-----:|-------:|---------\n");
            for (const auto &room : rooms) {
                if (!room) continue;
                int16_t roomID = room->getRoomID();
                int32_t objID = room->getObjectID();

                // Look up EAX preset from ROOM_EAX chunk
                uint32_t eaxIdx = 0;  // default: Generic
                auto eaxIt = mRoomEAXPresets.find(roomID);
                if (eaxIt != mRoomEAXPresets.end())
                    eaxIdx = eaxIt->second;
                const char *presetName = (eaxIdx < 26) ? kEAXPresets[eaxIdx].name : "Unknown";
                float decayTime = (eaxIdx < 26) ? kEAXPresets[eaxIdx].decayTime : 0.0f;

                // Try to load P$Acoustics for additional dampening/height info
                int32_t dampening = 0;
                int32_t height = 0;
                size_t acRawSize = 0;
                const uint8_t *acRaw = getPropertyRawData(
                    mPropertyService.get(), "Acoustics", objID, acRawSize);
                if (acRaw && acRawSize >= sizeof(PropAcoustics)) {
                    PropAcoustics ac;
                    std::memcpy(&ac, acRaw, sizeof(PropAcoustics));
                    dampening = ac.dampening;
                    height = ac.height;
                    // Use the property's EAX index if different from ROOM_EAX chunk
                    // (P$Acoustics on the room object can override the chunk value)
                    if (ac.eax < 26 && ac.eax != eaxIdx) {
                        eaxIdx = ac.eax;
                        presetName = kEAXPresets[eaxIdx].name;
                        decayTime = kEAXPresets[eaxIdx].decayTime;
                    }
                }

                // LoudRoom transmission factor (already loaded above)
                float loudRoom = mSoundPropagation
                                     ? mSoundPropagation->getRoomTransmission(roomID)
                                     : 1.0f;

                AUDIO_LOG( "  %4d | %5d | %-18s | %7.2f  | %4d | %6d | %.2f\n",
                             roomID, objID, presetName, decayTime, dampening, height, loudRoom);
            }
            AUDIO_LOG( "=== End Room Acoustic Verification ===\n\n");
        }
    }

    // Hand off P$AmbientHack parsing to the AmbientSoundManager. Per-schema
    // attenuation factors (applied above) are baked into each AmbientSound
    // record at parse time, so order matters: SchAttFac → manager load.
    if (mAmbientManager) mAmbientManager->loadAmbientSounds();
}

//------------------------------------------------------
void AudioService::loadSchemaPropertyOverrides()
{
    if (!mPropertyService || !mObjectService || !mSchemaParser)
        return;

    // Generic walker: iterate every (objID, raw bytes) in a property's
    // storage, look up the schema-archetype object by SymName, and call
    // the per-record callback if (a) the object has a known SymName and
    // (b) the byte size is large enough for the record type. Records on
    // unnamed objects (or with stale names) are silently skipped, matching
    // the original engine's tolerance.
    using ApplyFn = void(*)(SchemaEntry &, const uint8_t *);
    auto applyOverlay =
        [&](const char *propName, size_t minBytes, ApplyFn apply) -> int {
        Property *prop = mPropertyService->getProperty(propName);
        if (!prop || !prop->getStorage())
            return 0;
        DataStorage *storage = prop->getStorage();
        IntIteratorPtr it = storage->getAllStoredObjects();
        int applied = 0;
        int unmatched = 0;
        while (!it->end()) {
            int objID = it->next();
            size_t sz = 0;
            const uint8_t *bytes = storage->getRawData(objID, sz);
            if (!bytes || sz < minBytes) continue;
            std::string schemaName = mObjectService->getName(objID);
            if (schemaName.empty()) { ++unmatched; continue; }
            SchemaEntry *sch = mSchemaParser->findSchemaMutable(schemaName);
            if (!sch) { ++unmatched; continue; }
            apply(*sch, bytes);
            ++applied;
        }
        if (applied > 0 || unmatched > 0) {
            AUDIO_LOG("AudioService: [SCH_OVERRIDE] %s applied=%d unmatched=%d\n",
                      propName, applied, unmatched);
        }
        return applied;
    };

    // P$SchPlayPa (20 bytes) — schema play parameters. The on-disk dtype
    // default for `flags` is 0x7F00 (includes SFXFLG_SHARP at bit 12), so
    // this overlay normally confirms the SHARP-falloff default; schemas
    // with an explicit override (e.g. cleared SHARP) end up using that.
    applyOverlay("SchPlayPa", sizeof(PropSchemaPlayParams),
                 &applySchPlayParams);

    // P$SchLoopPa (8 bytes) — schema loop parameters. Some archetypes
    // declare looping only via this property, so we infer isLooping from
    // any non-default field; see applySchLoopParams for the rules.
    applyOverlay("SchLoopPa", sizeof(PropSchemaLoopParams),
                 &applySchLoopParams);

    // P$SchPriori (4 bytes) — per-schema priority for voice arbitration.
    applyOverlay("SchPriori", sizeof(PropSchPriori),
                 &applySchPriority);

    // P$SchMsg (16 bytes) — AI message label fired when this schema
    // triggers. E.g. `break_glass` → "gotonoise" so AIs investigate.
    applyOverlay("SchMsg", sizeof(PropSchMsg),
                 &applySchMessage);
}

//------------------------------------------------------
void AudioService::loadMiscSoundProperties()
{
    if (!mPropertyService || !mObjectService)
        return;

    mProjectileSounds.clear();
    mHeartbeatIntervals.clear();
    mSchemaLastSampleSaved.clear();

    // P$PrjSound — varstr schema name for a projectile archetype's
    // flight/impact sound. Property type=varstr means the raw bytes are
    // the string itself; the original engine stores either a length
    // prefix + chars or a raw NUL-terminated buffer (we tolerate both
    // by stripping any trailing NULs and trimming control bytes).
    if (Property *prj = mPropertyService->getProperty("PrjSound")) {
        if (DataStorage *storage = prj->getStorage()) {
            IntIteratorPtr it = storage->getAllStoredObjects();
            int loaded = 0;
            while (!it->end()) {
                int objID = it->next();
                size_t sz = 0;
                const uint8_t *bytes = storage->getRawData(objID, sz);
                if (!bytes || sz == 0) continue;
                // Strip any trailing NULs / non-printable trailing bytes
                // so the resulting std::string is exactly the schema
                // name. We don't trim leading bytes since some varstr
                // encodings begin with a length prefix; in that case the
                // first byte may be a non-printable count, and the rest
                // is the string. Heuristic: if the first byte is < 32
                // (control) and the second byte is printable, assume it
                // is a length prefix and skip it.
                size_t off = 0;
                if (sz >= 2 && bytes[0] < 32 && bytes[1] >= 32)
                    off = 1;
                size_t end = sz;
                while (end > off && (bytes[end - 1] == 0 ||
                                     bytes[end - 1] < 32))
                    --end;
                if (end <= off) continue;
                std::string name(reinterpret_cast<const char *>(bytes + off),
                                 end - off);
                mProjectileSounds[objID] = std::move(name);
                ++loaded;
            }
            if (loaded > 0)
                AUDIO_LOG("AudioService: loaded P$PrjSound for %d projectile "
                          "archetype(s)\n", loaded);
        }
    }

    // P$Heartbeat — int32 ms between heartbeats for an object that owns
    // a heartbeat schema. The MISS6 mission has no instances; the data
    // lives in dark.gam archetype records (e.g. AI fear states).
    if (Property *hb = mPropertyService->getProperty("Heartbeat")) {
        if (DataStorage *storage = hb->getStorage()) {
            IntIteratorPtr it = storage->getAllStoredObjects();
            int loaded = 0;
            while (!it->end()) {
                int objID = it->next();
                size_t sz = 0;
                const uint8_t *bytes = storage->getRawData(objID, sz);
                if (!bytes || sz < sizeof(int32_t)) continue;
                int32_t intervalMs;
                std::memcpy(&intervalMs, bytes, sizeof(int32_t));
                if (intervalMs <= 0) continue;  // 0/negative = inactive
                mHeartbeatIntervals[objID] = intervalMs;
                ++loaded;
            }
            if (loaded > 0)
                AUDIO_LOG("AudioService: loaded P$Heartbeat for %d "
                          "object(s)\n", loaded);
        }
    }

    // P$SchLastSa — int32 last-played sample index per schema. Saved
    // at mission save, restored on load — drives SCH_NO_REPEAT across
    // save/load boundaries. We populate mLastSampleIdx (the runtime
    // tracker used by sample selection) directly so the very next play
    // of the schema avoids the just-loaded sample. Empty in fresh
    // missions; non-empty after a save→load.
    if (Property *last = mPropertyService->getProperty("SchLastSa")) {
        if (DataStorage *storage = last->getStorage()) {
            IntIteratorPtr it = storage->getAllStoredObjects();
            int loaded = 0;
            while (!it->end()) {
                int objID = it->next();
                size_t sz = 0;
                const uint8_t *bytes = storage->getRawData(objID, sz);
                if (!bytes || sz < sizeof(int32_t)) continue;
                int32_t idx;
                std::memcpy(&idx, bytes, sizeof(int32_t));
                std::string schemaName = mObjectService->getName(objID);
                if (schemaName.empty()) continue;
                mSchemaLastSampleSaved[schemaName] = idx;
                mLastSampleIdx[schemaName] = idx;
                ++loaded;
            }
            if (loaded > 0)
                AUDIO_LOG("AudioService: restored P$SchLastSa for %d "
                          "schema(s)\n", loaded);
        }
    }
}

//------------------------------------------------------
std::vector<VoiceSpatialSnapshot> AudioService::getVoiceSpatialSnapshots() const
{
    std::vector<VoiceSpatialSnapshot> out;
    out.reserve(mVoicePool->size());
    for (const auto &kv : mVoicePool->voices()) {
        const ActiveVoice *v = kv.second.get();
        if (!v || !v->initialized) continue;
        VoiceSpatialSnapshot snap;
        snap.schemaName = v->schemaName;
        snap.sourcePos  = v->worldPos;
        // virtualPos is the propagation anchor — what Steam Audio sees
        // as the source location. When unreached or when reached=true
        // but virtualPos defaulted to sourcePos (same-room or
        // clean-threading short-circuit), this equals sourcePos.
        snap.virtualPos = v->cachedProp.reached
                              ? v->cachedProp.virtualPosition
                              : v->worldPos;
        snap.reached    = v->cachedProp.reached;
        snap.usePortal  = v->cachedProp.reached
                       && (glm::length(snap.virtualPos - snap.sourcePos) > 0.1f);
        snap.isAmbient  = v->isAmbient;
        snap.chain      = v->cachedChain;
        // Per-path snapshots — populated from the propagation result so
        // the show_vpos overlay can render every active sub-source slot
        // independently (the legacy single chain showed only paths[0]).
        // For the multi-path renderer to look "right" visually, the
        // overlay needs to draw what each slot is actually doing.
        snap.paths.reserve(v->cachedProp.paths.size());
        for (const auto &p : v->cachedProp.paths) {
            VoiceSpatialSnapshot::PathSnapshot ps;
            ps.endpoint = p.virtualPosition;
            ps.chain    = p.chain;
            ps.backend  = p.backend;
            snap.paths.push_back(std::move(ps));
        }
        // Divergence overlay fields. Primary effective distance is the
        // merged scalar from the cell-graph (or whichever backend the
        // service has selected). Secondary is the side-by-side result
        // when available — null in Phase 1 of the cell-graph migration.
        snap.primaryEffectiveDistance = v->cachedProp.effectiveDistance;
        if (v->cachedProp.secondaryBackendResult
            && v->cachedProp.secondaryBackendResult->reached) {
            snap.secondaryEffectiveDistance =
                v->cachedProp.secondaryBackendResult->effectiveDistance;
            snap.hasSecondaryBackend = true;
        }
        out.push_back(std::move(snap));
    }
    return out;
}

//------------------------------------------------------
std::string AudioService::getProjectileSound(int32_t objID) const
{
    if (!mPropertyService) return std::string();

    // Walk the MetaProp inheritance chain to find the archetype that
    // actually stores the schema name. Property::getEffectiveID handles
    // the chain walk for us; for concrete projectile instances spawned
    // from an archetype, this maps back to the negative archetype ID
    // where mProjectileSounds keeps the entry.
    Property *prj = mPropertyService->getProperty("PrjSound");
    if (!prj) return std::string();
    int effective = prj->getEffectiveID(objID);
    if (effective == 0) return std::string();
    auto it = mProjectileSounds.find(effective);
    if (it == mProjectileSounds.end()) return std::string();
    return it->second;
}

//------------------------------------------------------
// AI speech utterance — Speech_DB-driven schema selection + playback.
//
// Resolves the emitter's voice (P$VoiceIdx preferred — raw int32 voice
// index; falls back to P$SpchVoice for completeness even though stock
// Thief 2 lacks a voice-name lookup table in Speech_DB) via the MetaProp
// inheritance chain, queries the SpeechSelector for matching schemas,
// resolves the chosen schema-archetype objID → schema name via
// ObjectService, starts a voice via the normal schema-play path, and
// publishes a SoundEmissionEvent so AIHearingService sees it.
//
// Fire-and-forget: returns SoundHandle for callers that want to halt;
// most callers ignore the return value.
SoundHandle AudioService::playSpeech(int32_t emitterObjID,
                                     const std::string &conceptName,
                                     const std::vector<SchemaTagValue> &tags)
{
    if (!mAudioReady || !mSchemaParser) return SOUND_HANDLE_INVALID;
    if (!mPropertyService || !mObjectService) return SOUND_HANDLE_INVALID;
    if (!mSpeechDB || !mSpeechDB->isLoaded()) {
        // Loud fallback — speech DB never loaded means every playSpeech
        // call is silently a no-op without this. Log once per missing
        // concept by appending the concept name to the message.
        std::fprintf(stderr,
            "[FALLBACK] AudioService::playSpeech: Speech_DB not loaded; "
            "cannot play concept '%s' on obj %d\n",
            conceptName.c_str(), emitterObjID);
        return SOUND_HANDLE_INVALID;
    }

    // ── Resolve voice index ───────────────────────────────────────────
    //
    // P$VoiceIdx is the canonical source on AI archetypes (e.g. Karras=2).
    // We walk the MetaProp chain via Property::getEffectiveID so concrete
    // AI instances resolve to the archetype that actually carries the
    // property. P$SpchVoice is the legacy 16-byte label — we read it for
    // completeness, but the stock Speech_DB does not embed a voice-name
    // lookup, so the label path is currently always "no match" and falls
    // back to the loud-fallback branch below.
    int32_t voiceIndex = -1;
    if (Property *prop = mPropertyService->getProperty("VoiceIdx")) {
        int eff = prop->getEffectiveID(emitterObjID);
        if (eff != 0) {
            if (DataStorage *st = prop->getStorage()) {
                size_t sz = 0;
                const uint8_t *bytes = st->getRawData(eff, sz);
                if (bytes && sz >= sizeof(int32_t)) {
                    int32_t v;
                    std::memcpy(&v, bytes, sizeof(int32_t));
                    voiceIndex = v;
                }
            }
        }
    }
    if (voiceIndex < 0) {
        // Try P$SpchVoice — informational; resolved by selector if the
        // speech domain ever ships with a voice-name map.
        if (Property *prop = mPropertyService->getProperty("SpchVoice")) {
            int eff = prop->getEffectiveID(emitterObjID);
            if (eff != 0) {
                if (DataStorage *st = prop->getStorage()) {
                    size_t sz = 0;
                    const uint8_t *bytes = st->getRawData(eff, sz);
                    if (bytes && sz >= 16) {
                        char buf[17] = {0};
                        std::memcpy(buf, bytes, 16);
                        SpeechSelector tmp(*mSpeechDB);
                        voiceIndex = tmp.findVoiceByName(
                            std::string(buf, ::strnlen(buf, 16)));
                    }
                }
            }
        }
    }
    if (voiceIndex < 0) {
        std::fprintf(stderr,
            "[FALLBACK] AudioService::playSpeech: obj %d has no P$VoiceIdx "
            "(nor a resolvable P$SpchVoice) for concept '%s'\n",
            emitterObjID, conceptName.c_str());
        return SOUND_HANDLE_INVALID;
    }

    // ── Translate SchemaTagValue → SpeechSelector::TagQuery ───────────
    //
    // SchemaTagValue is the .sch-parser flavour (enumValues+rangeMin/Max);
    // SpeechSelector::TagQuery is the runtime flavour (intValue|enumValues).
    // The bridge keeps the selector's API independent of schema-parser
    // types — the selector is purely about Speech_DB shape.
    std::vector<SpeechSelector::TagQuery> query;
    query.reserve(tags.size());
    for (const auto &t : tags) {
        SpeechSelector::TagQuery q;
        q.tagName = t.tagName;
        if (t.isIntRange) {
            // Caller-supplied int ranges collapse to the midpoint when
            // passed to a leaf range check; we just pass the min through
            // since stock Thief 2 query sites always set min==max for
            // exact-value integer tags (e.g. Damage=37).
            q.hasInt   = true;
            q.intValue = t.rangeMin;
        }
        q.enumValues = t.enumValues;
        query.push_back(std::move(q));
    }

    // ── Run the selector ─────────────────────────────────────────────
    SpeechSelector sel(*mSpeechDB);
    sel.seed(mRng());  // re-seed from the AudioService RNG so picks vary
    auto matches = sel.selectMatches(voiceIndex, conceptName, query);
    if (matches.empty()) {
        std::fprintf(stderr,
            "[FALLBACK] AudioService::playSpeech: no Speech_DB match for "
            "voice=%d concept='%s' (%zu query tags) on obj %d\n",
            voiceIndex, conceptName.c_str(), tags.size(), emitterObjID);
        return SOUND_HANDLE_INVALID;
    }
    auto pick = sel.pickOne(matches, mRng);
    if (pick.schemaObjID == 0) return SOUND_HANDLE_INVALID;

    // ── Resolve schema-archetype objID → schema name → SchemaEntry ───
    //
    // The selector returns the schema-archetype object ID baked into
    // Speech_DB. ObjectService maps it back to its SymName (the schema
    // name as registered in the .sch parser).
    std::string schemaName = mObjectService->getName(pick.schemaObjID);
    if (schemaName.empty()) {
        std::fprintf(stderr,
            "[FALLBACK] AudioService::playSpeech: schema archetype %d has "
            "no SymName (voice=%d concept='%s')\n",
            pick.schemaObjID, voiceIndex, conceptName.c_str());
        return SOUND_HANDLE_INVALID;
    }
    const SchemaEntry *schema = mSchemaParser->findSchema(schemaName);
    if (!schema || schema->samples.empty()) {
        std::fprintf(stderr,
            "[FALLBACK] AudioService::playSpeech: schema '%s' (obj %d) not "
            "registered or has no samples\n",
            schemaName.c_str(), pick.schemaObjID);
        return SOUND_HANDLE_INVALID;
    }

    // ── Pick a sample and start the voice ────────────────────────────
    //
    // Mirrors playSchema: frequency-weighted random sample selection with
    // SCH_NO_REPEAT honored via mLastSampleIdx (keyed by schema name).
    std::vector<int> freqs;
    freqs.reserve(schema->samples.size());
    for (const auto &s : schema->samples) freqs.push_back(s.frequency);
    int totalFreq = schema->totalFrequency();
    if (totalFreq <= 0) return SOUND_HANDLE_INVALID;

    int idx = selectSample(schemaName,
                           static_cast<int>(schema->samples.size()),
                           totalFreq, freqs.data());
    if (idx < 0 || idx >= static_cast<int>(schema->samples.size()))
        return SOUND_HANDLE_INVALID;
    if ((schema->playParams.flags & SCH_NO_REPEAT)
        && schema->samples.size() > 1)
    {
        auto it = mLastSampleIdx.find(schemaName);
        if (it != mLastSampleIdx.end() && it->second == idx) {
            idx = selectSample(schemaName,
                               static_cast<int>(schema->samples.size()),
                               totalFreq, freqs.data());
            if (idx < 0 || idx >= static_cast<int>(schema->samples.size()))
                return SOUND_HANDLE_INVALID;
        }
    }
    mLastSampleIdx[schemaName] = idx;

    const SchemaSample &sample = schema->samples[idx];

    // Use the emitter's live position (never frozen at spawn) so AI
    // speech follows the emitter as it moves between rooms.
    Vector3 pos = mObjectService->position(emitterObjID);
    float vol  = schemaVolumeToLinear(schema->playParams.volume);
    bool looping = schema->loopParams.isLooping;

    SoundHandle h = startVoice(schemaName, sample.name, pos,
                               schema->playParams.priority, looping,
                               emitterObjID, vol, VoiceClass::Normal);
    if (h == SOUND_HANDLE_INVALID) return h;

    // ── Publish to AI hearing / debug listeners ──────────────────────
    //
    // soundType=0 (Untyped) — speech doesn't carry a P$AI_SndTyp by
    // default; AI hearing will treat it as untyped and apply default
    // range. The schema's authored volume (millibels) is forwarded as
    // gainDb so listeners can apply per-type loudness biasing. baseRange
    // is the schema's nominal radius if exposed; we pass the propagation
    // cap as a conservative default — AIHearingService re-derives its
    // own range from the AISNDTWK ranges anyway.
    SoundEmissionEvent ev{};
    ev.emitterObjID = emitterObjID;
    ev.position     = pos;
    ev.schemaName   = schemaName;
    ev.soundType    = 0;
    ev.baseRange    = mPropagationMaxDist;
    ev.gainDb       = static_cast<float>(schema->playParams.volume);
    publishSoundEmission(ev);
    return h;
}

//------------------------------------------------------
// loadSpotAmbients() and updateSpotAmbientVolumes() moved to
// AmbientSoundManager — see audio/AmbientSoundManager.cpp.

//------------------------------------------------------
// Helper: read the entire contents of a chunk file into a byte vector.
// Returns true on success, false if the chunk doesn't exist or is empty.
static bool readChunkBytes(const FileGroupPtr &db, const std::string &name,
                            std::vector<uint8_t> &out)
{
    if (!db->hasFile(name))
        return false;
    FilePtr f = db->getFile(name);
    if (!f)
        return false;
    file_size_t sz = f->size();
    if (sz == 0)
        return false;
    out.resize(static_cast<size_t>(sz));
    f->seek(0);
    f->read(out.data(), sz);
    return true;
}

void AudioService::loadSoundChunkDatabases(const FileGroupPtr &db)
{
    // Idempotent: only the chunks actually present in this FileGroup
    // are reparsed. Chunks loaded on a prior pass (e.g. the GAMESYS
    // pass) survive the subsequent MIS pass intact when the MIS doesn't
    // include them. This lets onDBLoad fire twice during a mission load
    // (once for dark.gam, once for the mis) without clobbering global
    // tables.

    // ENV_SOUND — compiled environmental-sound tag database.
    {
        std::vector<uint8_t> bytes;
        if (readChunkBytes(db, "ENV_SOUND", bytes)) {
            auto fresh = std::make_unique<EnvSoundDatabase>();
            if (fresh->loadFromChunk(bytes.data(), bytes.size())) {
                mEnvSoundDB = std::move(fresh);
                AUDIO_LOG("AudioService: parsed ENV_SOUND chunk (%zu bytes)\n",
                          bytes.size());
            } else {
                AUDIO_LOG("AudioService: failed to parse ENV_SOUND chunk "
                          "(%zu bytes) — keeping prior state\n", bytes.size());
            }
        }
    }

    // Speech_DB — compiled speech voice/concept database.
    {
        std::vector<uint8_t> bytes;
        if (readChunkBytes(db, "Speech_DB", bytes)) {
            auto fresh = std::make_unique<SpeechDatabase>();
            if (fresh->loadFromChunk(bytes.data(), bytes.size())) {
                mSpeechDB = std::move(fresh);
                AUDIO_LOG("AudioService: parsed Speech_DB chunk (%zu bytes)\n",
                          bytes.size());
            }
        }
    }

    // SchSamp — canonical schema → samples mapping.
    {
        std::vector<uint8_t> bytes;
        if (readChunkBytes(db, "SchSamp", bytes)) {
            auto fresh = std::make_unique<SchemaSamplesChunk>();
            if (fresh->loadFromChunk(bytes.data(), bytes.size())) {
                mSchemaSamplesDB = std::move(fresh);
                AUDIO_LOG("AudioService: parsed SchSamp chunk: %zu records\n",
                          mSchemaSamplesDB->recordCount());
            } else {
                AUDIO_LOG("AudioService: failed to parse SchSamp chunk — "
                          "keeping prior state\n");
            }
        }
    }

    // AIHearStat — per-rating distance/dB hearing-stats globals.
    {
        std::vector<uint8_t> bytes;
        if (readChunkBytes(db, "AIHearStat", bytes)) {
            AIHearingStats stats;
            if (readAIHearStat(bytes.data(), bytes.size(), stats)) {
                std::memcpy(mAIHearingStatsBytes, &stats, sizeof(stats));
                mHasAIHearingStats = true;
                AUDIO_LOG("AudioService: parsed AIHearStat chunk "
                          "(dist_muls[normal]=%.2f, db_adds[normal]=%d)\n",
                          stats.dist_muls[AI_HEARING_NORMAL],
                          stats.db_adds[AI_HEARING_NORMAL]);
            }
        }
    }

    // AISNDTWK — per-sound-type default audible-range globals.
    {
        std::vector<uint8_t> bytes;
        if (readChunkBytes(db, "AISNDTWK", bytes)) {
            AISoundTweaks tweaks;
            if (readAISndTwk(bytes.data(), bytes.size(), tweaks)) {
                std::memcpy(mAISoundTweaksBytes, &tweaks, sizeof(tweaks));
                mHasAISoundTweaks = true;
                AUDIO_LOG("AudioService: parsed AISNDTWK chunk "
                          "(ranges: untyped=%d combat=%d)\n",
                          tweaks.defaultRanges[AI_SOUND_UNTYPED],
                          tweaks.defaultRanges[AI_SOUND_COMBAT]);
            }
        }
    }
}

//------------------------------------------------------
void AudioService::loadMissionSoundData(const FileGroupPtr &db)
{
    // AMBIENT (4 bytes) — single int32 ObjID of the environmental
    // ambient that was active at save time. 0 == none. NOT a level-wide
    // enable flag (spot ambients are independent and always run on
    // proximity). All 15 shipping Thief 2 missions ship with 0 except
    // miss14, which ships with a nonzero env-ambient objID pre-set as
    // the level-start environmental sound. Resume / level-start playback
    // of that one ambient is deferred until the env-ambient runtime
    // lands (currently only spot ambients are wired up).
    mHasAmbientChunk = false;
    mEnvAmbientObjID = 0;
    if (db->hasFile("AMBIENT")) {
        FilePtr f = db->getFile("AMBIENT");
        if (f && f->size() >= sizeof(int32_t)) {
            int32_t value = 0;
            f->seek(0);
            *f >> value;
            mEnvAmbientObjID = value;
            mHasAmbientChunk = true;
            AUDIO_LOG("AudioService: AMBIENT chunk envObjID=%d%s\n",
                      value,
                      value == 0 ? " (no pre-set env ambient)" : "");
        }
    }

    // L$SoundDesc — sound-descriptor links. Relation::getAllLinks
    // requires a non-zero source, so we walk every object known to the
    // SymbolicName property storage (which covers every named object,
    // concrete + archetype) and query per-src. Objects without a
    // SymbolicName won't be visited; in practice every link source is
    // named, so this is sufficient for the data-capture pass.
    mSoundDescLinks.clear();
    LinkServicePtr linkSvc = GET_SERVICE(LinkService);
    if (linkSvc && mPropertyService) {
        RelationPtr rel = linkSvc->getRelation("SoundDescription");
        if (rel) {
            auto srcIDs = getAllObjectsWithProperty(
                mPropertyService.get(), "SymbolicName");
            for (int src : srcIDs) {
                if (src == 0) continue;
                LinkQueryResultPtr links = rel->getAllLinks(src, 0);
                while (!links->end()) {
                    const Link &lnk = links->next();
                    mSoundDescLinks.push_back(
                        SoundDescLink{lnk.src(), lnk.dst(), lnk.id()});
                }
            }
            if (!mSoundDescLinks.empty()) {
                AUDIO_LOG("AudioService: enumerated %zu L$SoundDesc link(s)\n",
                          mSoundDescLinks.size());
            }
        }
    }
}

//------------------------------------------------------
// Periodic (~5 s) audio status dump. Used to live at the top of
// updateAmbientVolumes() before the AmbientSoundManager extraction; now
// it's a standalone method called from loopStep() so it can read ambient
// state through mAmbientManager. Behaviour preserved bit-for-bit: same
// 5-second timer, same counter resets, same line format.
void AudioService::dumpAudioStatusPeriodic()
{
    // No ambients = no work and no log row. Matches the original
    // updateAmbientVolumes early-out (which gated the whole function on
    // mAmbients.empty()).
    if (!mAmbientManager || mAmbientManager->getAmbients().empty())
        return;

    const auto &ambients = mAmbientManager->getAmbients();

    static float debugTimer = 0.0f;
    debugTimer += 1.0f / 60.0f;  // approximate
    if (debugTimer < 5.0f)
        return;
    debugTimer = 0.0f;

    int playing = mAmbientManager->activeAmbientVoiceCount();

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
    for (auto &[h, v] : mVoicePool->voices()) {
        if (v->dspNode.reflectionsActive) ++reflVoices;
        if (v->sourceEnded && !v->finished.load(std::memory_order_relaxed)) ++tailVoices;
    }

    // Effective rays/sec (rays * reflection sim steps run in this dump period)
    float raysPerSec = (reflFrames * mRealtimeNumRays) / 5.0f;  // 5s dump interval

    // Compute listener's nearest probe once for the whole dump — every
    // ambient row references the same value (their reverb send convolves
    // with the listener-anchored IR, not a per-source one).
    int   listenerNearestProbe = -1;
    float listenerNearestDist  = -1.0f;
    const auto &probePositions = mProbeManager->getProbePositions();
    for (size_t i = 0; i < probePositions.size(); ++i) {
        float d = glm::length(probePositions[i] - mListenerPos);
        if (listenerNearestProbe < 0 || d < listenerNearestDist) {
            listenerNearestProbe = static_cast<int>(i);
            listenerNearestDist  = d;
        }
    }

    // Log active ambient details to diagnose distant sound leaks AND to
    // attribute wet-bus energy spikes to specific ambient voices.
    // reflIn is the peak mono signal this voice handed to convolution
    // since the last dump (resets here). Pair with [WET_BUS]'s peakDb /
    // reflGain and the listener/source nearestProbe distances to identify
    // the "ambient + small-space probe" pathological combination:
    // a high reflIn on a voice whose source AND the listener are both
    // close to the same probe, with [WET_BUS] peakDb spiking, confirms
    // the IR-energy hypothesis.
    for (const auto &amb : ambients) {
        if (amb.handle != SOUND_HANDLE_INVALID && mVoicePool->exists(amb.handle)) {
            float d = glm::length(mListenerPos - amb.position);
            ActiveVoice *vp = mVoicePool->find(amb.handle);
            if (!vp) continue;
            auto &v = *vp;
            float atten = v.dspNode.lastAtten.load(std::memory_order_relaxed);
            // pAtten is now a continuous excess-path multiplier:
            //   1.000 → same-room (no detour)
            //   0.001-0.999 → cross-room reachable, detour scales by ratio²
            //   0.000 → cross-room unreachable (BFS exhausted maxDist)
            // The `xRoom` flag indicates the cross-room HRTF direction
            // override is in effect for this voice this frame.
            bool xRoom = v.dspNode.usePortalRouting;
            float pAtten = v.dspNode.portalAttenuation;
            float pBlock = v.dspNode.portalBlocking;
            // Peak reverb-send this voice contributed since last dump.
            // Read-and-reset so the next dump period is independent.
            float reflIn = v.dspNode.reflSendPeak.exchange(0.0f,
                               std::memory_order_relaxed);
            // Source's nearest probe — small distance here means the
            // ambient sits inside a probe's catchment, which combined
            // with a small listener-to-probe distance is the worst case
            // for runaway reverb.
            int   srcNearestProbe = -1;
            float srcNearestDist  = -1.0f;
            for (size_t i = 0; i < probePositions.size(); ++i) {
                float dp = glm::length(probePositions[i] - amb.position);
                if (srcNearestProbe < 0 || dp < srcNearestDist) {
                    srcNearestProbe = static_cast<int>(i);
                    srcNearestDist  = dp;
                }
            }
            AUDIO_LOG( "  [AMB] '%s' dist=%.0f rad=%.0f atten=%.3f "
                         "pAtten=%.3f pBlock=%.2f xRoom=%d pos=(%.0f,%.0f,%.0f) "
                         "reflIn=%.4f lProbe=%d d=%.1f sProbe=%d d=%.1f\n",
                         amb.schemaName.c_str(), d, amb.radius, atten,
                         pAtten, pBlock, xRoom?1:0,
                         amb.position.x, amb.position.y, amb.position.z,
                         reflIn,
                         listenerNearestProbe, listenerNearestDist,
                         srcNearestProbe, srcNearestDist);
        }
    }

    float portalAvgUs = portalCalls > 0 ? static_cast<float>(portalUs) / portalCalls : 0.0f;

    AUDIO_LOG( "[Audio] %zu voices (%d refl, %d tail), %d/%zu ambients | "
                 "cb: total=%.0f/%.0fµs (%.0f%%) voice=%.0fµs mix=%.0fµs | "
                 "main: loop=%.1fms commit=%.1fms portal=%.0fµs(%dcalls,avg=%.0f) | "
                 "sim: direct=%.1fms refl=%.1fms(%dsteps) rays/s=%.0f | "
                 "churn: +%d/-%d",
                 mVoicePool->size(), reflVoices, tailVoices, playing, ambients.size(),
                 totalUs, budgetUs, loadPct, voiceUs, mixUs,
                 loopMs, commitMs, static_cast<float>(portalUs), portalCalls, portalAvgUs,
                 directMs, reflMs, reflFrames, raysPerSec,
                 created, destroyed);
    // Append convolution sub-worker stats if active
    if (mConvolutionPool && mConvolutionPool->isActive()) {
        ConvolutionWorker *cw = mConvolutionPool->worker();
        float maxMs = 0.0f;
        for (auto &subPtr : cw->workers) {
            float ms = subPtr->peakMs.exchange(0.0f, std::memory_order_relaxed);
            if (ms > maxMs) maxMs = ms;
        }
        AUDIO_LOG( " | conv: %.1fms (%dw)", maxMs, cw->numWorkers);
    }
    AUDIO_LOG( "\n");
}

//------------------------------------------------------
// Per-frame ducking envelope ramp. When SFX voices are playing, ambient
// volumes are reduced to keep dialog, footsteps, and other important
// sounds prominent. The duck envelope smoothly transitions between
// normal and ducked states to avoid pumping artifacts. The duckGain
// multiplier is applied inline by AmbientSoundManager when setting each
// ambient's volume to avoid compounding volume decay from read-back-and-
// multiply. Used to live at the tail of updateAmbientVolumes() before
// the extraction; preserved bit-for-bit.
void AudioService::updateAmbientDuckingEnvelope()
{
    // Gate by ambient activity to match the original behaviour: when
    // there are zero ambients the old updateAmbientVolumes early-out
    // skipped this block, so we replicate that to avoid changing the
    // duckGain trajectory in ambient-free scenes.
    if (!mAmbientManager || mAmbientManager->getAmbients().empty())
        return;

    if (mReflectionMixNode && mReflectionMixNode->duckingEnabled) {
        auto &rmn = *mReflectionMixNode;

        // Count active non-ambient SFX voices
        int sfxVoices = 0;
        for (auto &[h, v] : mVoicePool->voices()) {
            if (!v->isAmbient && !v->sourceEnded.load(std::memory_order_relaxed))
                ++sfxVoices;
        }

        // Smooth envelope toward target
        float target = (sfxVoices > 0) ? rmn.duckAmount : 1.0f;
        float coeff = (target < rmn.duckGain) ? rmn.duckAttackCoeff : rmn.duckReleaseCoeff;
        rmn.duckGain += coeff * (target - rmn.duckGain);
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
    // Player-emitted sounds have skipAttenuation=true, so no Steam Audio
    // distance/occlusion is applied. Volume is purely speed-dependent.
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

    // Pass VoiceClass::PlayerEmitted so startVoice sets voice.playerEmitted
    // BEFORE createVoiceSource runs. The per-class default directParams
    // (distAtt=1.0, AIRABS-only flags) are then in place from the very
    // first audio callback — no skipAttenuation bypass needed, no ~21 ms
    // wrong-volume window at the attack.
    SoundHandle h = startVoice(schema->name, sample.name, pos,
                               schema->playParams.priority, false, 0, finalVol,
                               VoiceClass::PlayerEmitted);

    if (h != SOUND_HANDLE_INVALID && mVoicePool->exists(h)) {
        // Diagnostic: count concurrent footstep voices (including tails)
        int footActive = 0, footTail = 0;
        for (auto &[fh, fv] : mVoicePool->voices()) {
            if (fv->schemaName.find("foot_") == 0 || fv->schemaName.find("land_") == 0) {
                if (fv->sourceEnded) ++footTail;
                else ++footActive;
            }
        }
        // Snapshot of the closest baked probe at the moment of the footstep.
        // If probe interpolation is the cause of A/B amplitude swings, the
        // pattern should show up as "footsteps loud when nearestDist is small,
        // quiet when nearestDist is large." Skipped when no probes are loaded
        // (e.g. --no-probes) — nearestIdx stays -1.
        int   nearestIdx  = -1;
        float nearestDist = -1.0f;
        const auto &footProbePositions = mProbeManager->getProbePositions();
        for (size_t i = 0; i < footProbePositions.size(); ++i) {
            float d = glm::length(footProbePositions[i] - mListenerPos);
            if (nearestIdx < 0 || d < nearestDist) {
                nearestIdx = static_cast<int>(i);
                nearestDist = d;
            }
        }
        // listenerPos included so this event log can be cross-referenced
        // against the periodic [WET_BUS] line — together they let you build
        // an offline position→wet-amplitude scatter for probe-coverage debug.
        AUDIO_LOG( "[FOOT] h=%d '%s' vol=%.2f spd=%.1f active=%d tail=%d "
                   "listenerPos=(%.1f,%.1f,%.1f) nearestProbe=%d dist=%.2f\n",
                     h, sample.name.c_str(), finalVol, speed, footActive, footTail,
                     mListenerPos.x, mListenerPos.y, mListenerPos.z,
                     nearestIdx, nearestDist);
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

    // VoiceClass::PlayerEmitted — see playFootstep for rationale.
    SoundHandle h = startVoice(schema->name, sample.name, pos,
                               schema->playParams.priority, false, 0, finalVol,
                               VoiceClass::PlayerEmitted);

    if (h != SOUND_HANDLE_INVALID && mVoicePool->exists(h)) {
        // Match the [FOOT] event log: capture listener position at the impact
        // so we can map landing audibility to where in the level the player is.
        AUDIO_LOG("[LAND] h=%d '%s' vol=%.2f fallSpd=%.1f "
                  "listenerPos=(%.1f,%.1f,%.1f)\n",
                  h, sample.name.c_str(), finalVol, fallSpeed,
                  mListenerPos.x, mListenerPos.y, mListenerPos.z);
    }
}

// ── Sound propagation through portal graph (facades over SoundPropagation) ──

//------------------------------------------------------
SoundPropInfo AudioService::propagateSound(const Vector3 &sourcePos,
                                            const Vector3 &listenerPos,
                                            float maxDist) const
{
    if (!mSoundPropagation)
        return {};
    return mSoundPropagation->propagateSound(
        sourcePos, listenerPos, maxDist, mPropMaxPaths, mPropMaxPathDiff);
}

//------------------------------------------------------
void AudioService::setBlockingFactor(int room1, int room2, float factor)
{
    if (mSoundPropagation)
        mSoundPropagation->setBlockingFactor(room1, room2, factor);
}

//------------------------------------------------------
void AudioService::setSoundPathLineOfSightFn(
    std::function<bool(const Vector3 &a, const Vector3 &b)> fn)
{
    if (mSoundPropagation)
        mSoundPropagation->setLineOfSightFn(std::move(fn));
}

//------------------------------------------------------
float AudioService::getBlockingFactor(int room1, int room2) const
{
    return mSoundPropagation ? mSoundPropagation->getBlockingFactor(room1, room2)
                             : 0.0f;
}

//------------------------------------------------------
void AudioService::setOcclusionRadius(float r)
{
    if (mAudioOcclusion) mAudioOcclusion->setRadius(r);
}

//------------------------------------------------------
float AudioService::getOcclusionRadius() const
{
    return mAudioOcclusion ? mAudioOcclusion->getRadius() : 10.0f;
}

//------------------------------------------------------
void AudioService::setOcclusionSamples(int n)
{
    if (mAudioOcclusion) mAudioOcclusion->setSamples(n);
}

//------------------------------------------------------
int AudioService::getOcclusionSamples() const
{
    return mAudioOcclusion ? mAudioOcclusion->getSamples() : 16;
}

// ── Global AI hearing data accessors (Unit C) ──
//
// Both copy the stored raw bytes into the caller-supplied struct, returning
// false when no chunk was loaded (the gamesys did not contain the chunk in
// onDBLoad). On false the caller should fall back to kDefaultAIHearingStats
// from audio/AIHearingData.h.

//------------------------------------------------------
bool AudioService::getAIHearingStats(AIHearingStats &out) const
{
    if (!mHasAIHearingStats)
        return false;
    return readAIHearStat(mAIHearingStatsBytes,
                          sizeof(mAIHearingStatsBytes), out);
}

//------------------------------------------------------
bool AudioService::getAISoundTweaks(AISoundTweaks &out) const
{
    if (!mHasAISoundTweaks)
        return false;
    return readAISndTwk(mAISoundTweaksBytes,
                        sizeof(mAISoundTweaksBytes), out);
}

// ── Sound emission pub/sub ──

//------------------------------------------------------
void AudioService::registerSoundEmissionListener(SoundEmissionListener cb)
{
    if (cb)
        mSoundEmissionListeners.push_back(std::move(cb));
}

//------------------------------------------------------
void AudioService::publishSoundEmission(const SoundEmissionEvent &ev)
{
    // Synchronous dispatch on the main thread — listeners must be cheap.
    for (auto &cb : mSoundEmissionListeners) {
        if (cb)
            cb(ev);
    }
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

// ── Probe baking and loading (facades over ProbeManager) ──

//------------------------------------------------------
int AudioService::getProbeCount() const
{
    return mProbeManager ? mProbeManager->getProbeCount() : 0;
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
    return ProbeManager::getProbeFilePath(misPath, gameName);
}

//------------------------------------------------------
void  AudioService::setProbeSpacingFt(float ft)
{
    if (mProbeManager) mProbeManager->setProbeSpacingFt(ft);
}

float AudioService::getProbeSpacingFt() const
{
    return mProbeManager ? mProbeManager->getProbeSpacingFt() : 5.0f;
}

void  AudioService::setProbeHeightFt(float ft)
{
    if (mProbeManager) mProbeManager->setProbeHeightFt(ft);
}

float AudioService::getProbeHeightFt() const
{
    return mProbeManager ? mProbeManager->getProbeHeightFt() : 5.0f;
}

void AudioService::setProbeElevations(std::vector<float> heights)
{
    mProbeElevations = std::move(heights);
}

void AudioService::setProbePortalRings(bool enabled)
{
    mProbePortalRings = enabled;
}

const std::vector<Vector3> &AudioService::getProbePositions() const
{
    static const std::vector<Vector3> kEmpty;
    return mProbeManager ? mProbeManager->getProbePositions() : kEmpty;
}

bool AudioService::bakeProbes(const std::string &outputPath,
                               std::atomic<float> *progress,
                               float spacing, float height)
{
    if (!mProbeManager || !mIplScene) {
        LOG_ERROR("AudioService: cannot bake probes — no acoustic scene");
        return false;
    }

    // Snapshot the bake-time tuning into ProbeBakeParams. The override
    // sentinels (negative) are forwarded so ProbeManager falls back to its
    // own configured spacing/height.
    ProbeBakeParams params;
    params.sceneMin              = mSceneMin;
    params.sceneMax              = mSceneMax;
    params.propagationMaxDist    = mPropagationMaxDist;
    params.bakeNumRays           = mBakeNumRays;
    params.bakeNumBounces        = mBakeNumBounces;
    params.bakeDuration          = mBakeDuration;
    params.bakeDiffuseSamples    = mBakeDiffuseSamples;
    params.simulatorThreads      = mSimulatorThreadsCfg;
    params.ambisonicsOrder       = mBakeAmbisonicsOrder;
    params.sceneType             = mSceneTypeCfg;
    params.spacingFtOverride     = spacing;
    params.heightFtOverride      = height;
    params.additionalElevations  = mProbeElevations;

    // Build per-portal axial anchors from RoomService at bake time.
    // For each portal we contribute up to two probe candidates:
    //   center + normal * axialOffsetFt   (into room A)
    //   center - normal * axialOffsetFt   (into room B)
    // ProbeManager applies a proximity dedup against the floor +
    // elevation tiers so anchors that fall on top of existing grid
    // probes are dropped before the bake.
    //
    // Two filters live here on the audio side because they need
    // RoomService (the portal+room graph) which ProbeManager doesn't:
    //
    //   1. Canonical-orientation dedup. Each shared doorway portal is
    //      stored in BOTH adjoining rooms' portal lists. We only emit
    //      when `portal->getPortalID() < portal->getDestPortalID()` so
    //      a single ProbeAxis represents the pair. (The previous ring
    //      design emitted both directions, which doubled every doorway
    //      because the basis symmetry made the 4 ring positions match
    //      exactly between the two passes.)
    //
    //   2. Sky-portal filter. Tall outdoor cells extending to the
    //      skybox produce portals at the geometric center of their
    //      shared face — which is up in the sky, far from any floor.
    //      On miss6 ~250 portal centroids sit above z=+5 ft (well above
    //      every playable floor in the level). We approximate each
    //      adjoining room's floor Z by projecting the room center onto
    //      the most upward-facing bounding plane, then skip the portal
    //      if its centroid is more than `kSkyThresholdFt` above the
    //      lower of the two floors. The lower bound is permissive — we
    //      only drop portals that are unrealistically high above BOTH
    //      adjoining floors.
    if (mProbePortalRings && mRoomService) {
        // 30 ft above floor is well clear of any realistic indoor or
        // mezzanine portal — typical room ceilings are 8–16 ft. Outdoor
        // sky portals in miss6 sit ~70 ft above their room floors.
        constexpr float kSkyThresholdFt = 30.0f;

        auto roomFloorZ = [](Room *r) -> float {
            // Bounding planes face inward (positive distance = inside).
            // The most upward-facing plane is the room's floor. Solve
            // `dot(normal, p) + d = 0` for z at the room's center (x,y).
            // For axis-aligned rooms this is exact; for OBB-rotated
            // rooms it gives the floor height under the center, which
            // is good enough for a "is this portal in the sky?" check.
            const Plane *planes = r->getBoundingPlanes();
            int   floorIdx = 0;
            float bestNz   = -1e9f;
            for (int i = 0; i < 6; ++i) {
                if (planes[i].normal.z > bestNz) {
                    bestNz   = planes[i].normal.z;
                    floorIdx = i;
                }
            }
            const Plane &fp = planes[floorIdx];
            if (std::abs(fp.normal.z) < 1e-4f) return r->getCenter().z;
            const Vector3 c = r->getCenter();
            return -(fp.normal.x * c.x + fp.normal.y * c.y + fp.d) / fp.normal.z;
        };

        const auto &rooms = mRoomService->getAllRooms();
        int totalPortals = 0, skippedDup = 0, skippedSky = 0, emitted = 0;
        for (const auto &roomPtr : rooms) {
            if (!roomPtr) continue;
            const uint32_t portalCount = roomPtr->getPortalCount();
            for (uint32_t i = 0; i < portalCount; ++i) {
                RoomPortal *portal = roomPtr->getPortal(i);
                if (!portal) continue;
                ++totalPortals;

                // Canonical orientation: keep the lower portal ID only.
                // For self-loop portals (shouldn't exist but be safe)
                // myID == destID — keep them.
                const int32_t myID   = portal->getPortalID();
                const int32_t destID = portal->getDestPortalID();
                if (myID > destID) { ++skippedDup; continue; }

                const Vector3 center = portal->getCenter();
                const Plane  &plane  = portal->getPlane();
                Vector3 normal{plane.normal.x, plane.normal.y, plane.normal.z};
                float nLen = glm::length(normal);
                if (nLen < 1e-4f) continue;
                normal /= nLen;

                // Sky-portal filter: compare centroid Z to the floors of
                // both adjoining rooms. Use the lower (more permissive)
                // floor as the reference so legitimate high-mezzanine
                // portals survive when one adjoining room has a low floor.
                Room *roomA = roomPtr.get();
                Room *roomB = portal->getFarRoom();
                float bottomA = roomFloorZ(roomA);
                float bottomB = roomB ? roomFloorZ(roomB) : bottomA;
                float minBottom = std::min(bottomA, bottomB);
                if (center.z > minBottom + kSkyThresholdFt) {
                    ++skippedSky;
                    continue;
                }

                ProbeBakeParams::PortalAxis axis;
                axis.center = center;
                axis.normal = normal;
                params.portalAxes.push_back(axis);
                ++emitted;
            }
        }
        AUDIO_LOG("Portal-axis emit: %d portals walked, %d back-direction "
                  "dups skipped, %d sky portals skipped, %d emitted "
                  "(→ up to %d probe candidates pre-dedup)\n",
                  totalPortals, skippedDup, skippedSky, emitted, emitted * 2);
    }

    bool ok = mProbeManager->bakeProbes(mIplScene, outputPath, params, progress);
    if (ok) {
        // Classify the freshly-baked probes so the overlay can show which
        // ones a future reachability filter would prune. Cheap and only
        // the debug overlay reads the result.
        classifyProbeReachability();
    }
    return ok;
}


//------------------------------------------------------
bool AudioService::loadProbes(const std::string &probePath)
{
    if (!mProbeManager) {
        LOG_ERROR("AudioService: cannot load probes — ProbeManager not initialized");
        return false;
    }
    // If a previous probe batch is still attached to the direct simulator,
    // detach it BEFORE ProbeManager swaps in a new batch. ProbeManager's
    // loadProbes only knows about the reflection simulator, so its release
    // path won't touch the direct sim; leaving the old batch attached
    // produces a dangling reference once ProbeManager releases its
    // create-time ref. Symmetric with destroyAcousticScene's teardown.
    if (mDirectProbeBatchAdded && mDirectSimulator && mProbeManager) {
        IPLProbeBatch oldBatch = mProbeManager->getProbeBatch();
        if (oldBatch) {
            iplSimulatorRemoveProbeBatch(mDirectSimulator, oldBatch);
            iplSimulatorCommit(mDirectSimulator);
            iplProbeBatchRelease(&oldBatch);  // drops our retain
        }
        mDirectProbeBatchAdded = false;
    }

    bool ok = mProbeManager->loadProbes(probePath,
        mReflectionSim ? mReflectionSim->simulator() : nullptr);
    if (!ok) return false;

    // Attach the freshly loaded probe batch to the direct simulator too,
    // so Steam Audio's pathing can walk the probe graph from each voice's
    // source to the listener. Retain so ProbeManager's release path on the
    // reflection simulator side doesn't free the batch out from under us.
    if (mDirectSimulator) {
        IPLProbeBatch batch = mProbeManager->getProbeBatch();
        if (batch) {
            iplProbeBatchRetain(batch);
            iplSimulatorAddProbeBatch(mDirectSimulator, batch);
            iplSimulatorCommit(mDirectSimulator);
            mDirectProbeBatchAdded = true;
            AUDIO_LOG("AudioService: attached probe batch (%d probes) to "
                      "direct simulator for pathing\n",
                      mProbeManager->getProbeCount());
        }
    }
    // Classify reachability so the debug overlay can preview which probes
    // a future bake-time filter would prune. Cheap (a few ms even for
    // thousands of probes) and only the overlay reads the result.
    classifyProbeReachability();
    return true;
}


//------------------------------------------------------
size_t AudioService::classifyProbeReachability()
{
    mProbeFates.clear();
    if (!mProbeManager) return 0;

    const auto &positions = mProbeManager->getProbePositions();
    if (positions.empty()) return 0;

    if (!mRoomService) {
        // Without rooms we can't classify anything sensibly. Mark every
        // probe NoRoom so the overlay still has parallel data.
        mProbeFates.assign(positions.size(), ProbeFate::NoRoom);
        AUDIO_LOG("classifyProbeReachability: no RoomService — flagged all "
                  "%zu probes NoRoom\n",
                  positions.size());
        return positions.size();
    }

    mProbeFates.assign(positions.size(), ProbeFate::Kept);

    // Step 1: collect seed rooms — every room that contains at least one
    // concrete (mapper-placed) object that directly owns a P$Position.
    // `owns` (not `has`) skips archetype inheritance, so archetypes whose
    // position is (0,0,0) don't seed BFS in some unroomed void cell. This
    // seeding strategy is what handles disconnected playable subgraphs
    // (e.g. teleporter-only-reachable areas): every gameplay component
    // must contain at least one mapper-placed object, so it self-seeds.
    std::unordered_set<Room *> seedRooms;
    if (mObjectService && mPropertyService) {
        Property *propPos = mPropertyService->getProperty("Position");
        if (propPos && propPos->getStorage()) {
            IntIteratorPtr it = propPos->getStorage()->getAllStoredObjects();
            while (it && !it->end()) {
                int id = it->next();
                if (id <= 0) continue;            // skip archetypes
                if (!mObjectService->exists(id)) continue;
                Vector3 pos = mObjectService->position(id);
                Room *r = mRoomService->roomFromPoint(pos);
                if (r) seedRooms.insert(r);
            }
        }
    }

    if (seedRooms.empty()) {
        AUDIO_LOG("[FALLBACK] classifyProbeReachability: no seed rooms "
                  "found (ObjectService=%p PropertyService=%p) — every "
                  "probe with a room will be marked Unreachable. Overlay "
                  "may show entire level red.\n",
                  static_cast<void *>(mObjectService.get()),
                  static_cast<void *>(mPropertyService.get()));
    }

    // Step 2: multi-seed BFS over the room-portal graph. `playable` is the
    // union of all rooms reachable from any seed.
    std::unordered_set<Room *> playable;
    std::queue<Room *> frontier;
    for (Room *seed : seedRooms) {
        if (playable.insert(seed).second) frontier.push(seed);
    }
    while (!frontier.empty()) {
        Room *cur = frontier.front();
        frontier.pop();
        const uint32_t pc = cur->getPortalCount();
        for (uint32_t i = 0; i < pc; ++i) {
            RoomPortal *p = cur->getPortal(i);
            if (!p) continue;
            Room *neighbor = p->getFarRoom();
            if (!neighbor) continue;
            if (playable.insert(neighbor).second) frontier.push(neighbor);
        }
    }

    // Step 3: per-probe classification.
    size_t kept = 0, noRoom = 0, unreach = 0;
    for (size_t i = 0; i < positions.size(); ++i) {
        Room *r = mRoomService->roomFromPoint(positions[i]);
        if (!r) {
            mProbeFates[i] = ProbeFate::NoRoom;
            ++noRoom;
        } else if (playable.count(r) == 0) {
            mProbeFates[i] = ProbeFate::Unreachable;
            ++unreach;
        } else {
            mProbeFates[i] = ProbeFate::Kept;
            ++kept;
        }
    }

    AUDIO_LOG("classifyProbeReachability: seeds=%zu rooms → %zu playable; "
              "probes kept=%zu, noRoom=%zu, unreachable=%zu (would prune "
              "%zu / %zu = %.1f%%)\n",
              seedRooms.size(), playable.size(),
              kept, noRoom, unreach,
              noRoom + unreach, positions.size(),
              positions.size() ? 100.0f * (noRoom + unreach) / positions.size() : 0.0f);

    return noRoom + unreach;
}


} // namespace Darkness
