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
#include "LatencyHistogram.h"
#include "PathingSimulator.h"
#include "ProbeFile.h"
#include "ProbeManager.h"
#include "ReflectionSimulator.h"
#include "WetBusBeatDetector.h"
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
#include "sim/DoorAudioGeometry.h"  // DoorAudioGeometry passed to registerDoorGeometry
#include "logger.h"

#include <unordered_set>
#include <cmath>
#include <thread>

#if defined(__APPLE__)
#  include <pthread.h>
#  include <sys/qos.h>
#endif

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

    // Shared handles (NOT owned — AudioService manages lifetimes).
    // No reflection mixer here: the post-Phase-3 worker pool sums per-voice
    // ambisonics manually (Steam Audio's reflection mixer rejects
    // HYBRID/PARAMETRIC modes).
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

    int frameSize = static_cast<int>(kDefaultDeviceFrameSize);  // engine frame size
    int reflectionFrameSize = static_cast<int>(kDefaultDeviceFrameSize);  // reflection frame size (post rate-divisor)
    int ambiChannels = 1;             // ambisonics channel count (1 for order 0, 4 for order 1)
    int ambiOrder = 0;                // ambisonics order (0 or 1)
    // Sync-in-callback wait deadline. Set at init from
    // 0.70 * callback_period_ms (frameSize / sampleRate). When workers
    // overrun this, the wet bus falls through to last-frame's output
    // (which is what the pre-sync pipeline did unconditionally). Larger
    // values give workers more headroom but bring the audio thread
    // closer to the underrun cliff; 0.70 leaves ~30 % of the period for
    // master DSP + node-graph overhead.
    float syncDeadlineMs = 7.5f;
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
//
// kFeetToMeters / kMetersToFeet live in AudioUnits.h so every audio file
// resolves to the same literal — see that header for the consolidation
// rationale.

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

// Convert a 4x4 affine transform from engine space (Z-up, feet) to IPL space
// (Y-up, meters) for IPLInstancedMeshSettings / iplInstancedMeshUpdateTransform.
//
// Derivation: if x_ipl = P * x_engine where P maps positions engine→IPL, then
// for an engine transform M_e we need M_i = P * M_e * P^-1. P decomposes as
// (kFeetToMeters * Q) where Q is the basis permutation+sign engine→IPL. The
// feet/meter scaling cancels in the linear part (P * R_e * P^-1 = Q * R_e *
// Q^-1) and survives only in the translation column.
//
// Concretely: column j of R_ipl equals engineToIplDir(R_engine applied to the
// engine vector that maps to IPL basis j). Walking IPL bases (1,0,0), (0,1,0),
// (0,0,1) through iplToEnginePos (without metric scale) gives engine vectors
// (0,-1,0), (0,0,1), (-1,0,0) — so R_ipl.col[0] = eToIDir(-R_e.col[1]),
// R_ipl.col[1] = eToIDir(R_e.col[2]), R_ipl.col[2] = eToIDir(-R_e.col[0]).
// Translation goes through engineToIplPos (metric scale included).
inline IPLMatrix4x4 engineToIplMatrix(const Matrix4 &m) {
    auto eToI = [](const Vector3 &d) -> Vector3 {
        return Vector3(-d.y, d.z, -d.x);
    };
    // GLM column-major: m[col][row]. Extract engine columns.
    Vector3 col0_e(m[0][0], m[0][1], m[0][2]);
    Vector3 col1_e(m[1][0], m[1][1], m[1][2]);
    Vector3 col2_e(m[2][0], m[2][1], m[2][2]);
    Vector3 trans_e(m[3][0], m[3][1], m[3][2]);

    Vector3 col0_i = eToI(-col1_e);
    Vector3 col1_i = eToI( col2_e);
    Vector3 col2_i = eToI(-col0_e);
    Vector3 trans_i(-trans_e.y * kFeetToMeters,
                     trans_e.z * kFeetToMeters,
                    -trans_e.x * kFeetToMeters);

    IPLMatrix4x4 r{};
    // IPLMatrix4x4 elements[row][col] is row-major.
    r.elements[0][0] = col0_i.x; r.elements[0][1] = col1_i.x;
    r.elements[0][2] = col2_i.x; r.elements[0][3] = trans_i.x;
    r.elements[1][0] = col0_i.y; r.elements[1][1] = col1_i.y;
    r.elements[1][2] = col2_i.y; r.elements[1][3] = trans_i.y;
    r.elements[2][0] = col0_i.z; r.elements[2][1] = col1_i.z;
    r.elements[2][2] = col2_i.z; r.elements[2][3] = trans_i.z;
    r.elements[3][0] = 0.0f;     r.elements[3][1] = 0.0f;
    r.elements[3][2] = 0.0f;     r.elements[3][3] = 1.0f;
    return r;
}

// ── Audio-thread tunables ──
// Namespace-scope atomics published by AudioService setters and read on the
// audio thread. These belong to per-voice processing paths that don't have
// direct access to AudioService members; using atomics avoids data races
// even though the values are typically only set once at startup from
// RenderConfig.
//   sHrtfInterpolation: 0 = nearest, 1 = bilinear
//
// These have external linkage so AudioDSPChain.cpp's publish helper can
// write to them via extern declarations; the audio callback in this TU
// continues to read them with no extra indirection.
std::atomic<int>   sHrtfInterpolation{1};
std::atomic<float> sSpatialBlend{1.0f};
std::atomic<float> sDoorLpfOpenHz{20000.0f};
std::atomic<float> sDoorLpfBlockedHz{800.0f};
std::atomic<float> sPropMinAttenuation{0.001f};
/// Engine sample rate published for audio-thread DSP that needs it (door LPF, etc.)
static std::atomic<uint32_t> sEngineSampleRate{kDefaultDeviceSampleRate};

/// Phase 4: listener orientation published for audio-thread
/// iplPathEffectApply. Written by the main thread in `loopStep` each
/// frame (right after `listenerCoord` is built); read by the audio
/// thread inside steamAudioNodeProcess. Loose-sync: an audio callback
/// may see a one-frame-stale orientation, which is consistent with the
/// reflection-mix-node's listenerOrientation field that uses the same
/// pattern. Initialized to IPL's neutral facing direction (-Z ahead)
/// so the first audio callback before the first loopStep sees a valid
/// coordinate space.
static IPLCoordinateSpace3 sPathListenerCoord = {
    {1.0f, 0.0f, 0.0f},  // right = +X
    {0.0f, 1.0f, 0.0f},  // up    = +Y (IPL Y-up)
    {0.0f, 0.0f, -1.0f}, // ahead = -Z (IPL convention)
    {0.0f, 0.0f, 0.0f}   // origin
};
/// Audio-thread-visible HRTF pointer for iplPathEffectApply (when
/// binaural=true). Published by AudioService::init at HRTF creation,
/// read by the audio thread. The HRTF handle is owned by AudioService;
/// this is a non-owning shared reference.
static std::atomic<IPLHRTF> sPathHrtf{nullptr};

/// Audio-thread-visible runtime ambisonics order for iplPathEffectApply.
/// Must equal the path effect's create-time IPLPathEffectSettings::maxOrder
/// (which we set to mAmbisonicsOrder); Steam Audio's internal rotate stage
/// asserts the encoded SH buffer's channel count == numCoeffsForOrder(order).
/// Published once at init alongside sPathHrtf; the audio thread reads it
/// when staging the per-call IPLPathEffectParams::order so the cached
/// pathTargetParams / lastGoodPathParams replays still carry a valid order
/// even before the first solve writes one.
static std::atomic<int> sPathAmbisonicsOrder{0};

/// Silent-voice convolution skip threshold (in audio callbacks). Derived
/// via `reflSilentSkipFrames()` (AudioUnits.h) from the live
/// `realtime.duration`, reflection sample rate, and reflection frame
/// size. Updated by `recomputeReflSilentSkipFrames()` at every event
/// that changes any of those inputs (init, YAML reload, debug-console
/// edit) so the audio callback never reads a stale literal.
std::atomic<int> sReflSilentSkipFrames{192};

/// Runtime cap on the number of simultaneous sub-source slots assigned per
/// voice. Independent of mPropMaxPaths (which caps how many paths the BFS
/// keeps); the effective per-voice cap is min(mPropMaxPaths, sMaxSubSources).
/// Phase 4 default = 4. Per-path occlusion via per-slot IPLSource +
/// iplSimulatorRunDirect makes higher path counts physically meaningful
/// (each path contributes its own distance attenuation, air absorption,
/// volumetric occlusion, and transmission). Clamped against kMaxSubSources
/// at the slot-assignment call site.
static std::atomic<uint32_t> sMaxSubSources{4};

/// Runtime IR-length clamp (samples at the reflection rate). 0 = disabled
/// (use whatever irSize the simulator / baked probe produced). >0 = cap
/// per-voice `slot.params.irSize` to this value before handing it to
/// iplReflectionEffectApply. Steam Audio convolves with the first N
/// samples of the IR, so a smaller value means cheaper convolution per
/// voice — useful for A/B-testing the perceptual cost of shorter IRs
/// without re-baking. Set from AudioService::setRuntimeIrClampMs (which
/// converts ms → samples using the reflection sample rate).
static std::atomic<int> sRuntimeIrClampSamples{0};

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

// ── Per-stage latency histograms ──
//
// Companions to the scalar peak counters above. The peak fields catch
// the worst single iteration since the last reset (useful for "did we
// ever overrun?"); the histograms capture the full p50/p95/p99
// distribution over the dump window (useful for "where is the budget
// actually going?"). Writers (audio thread + sim thread) record into
// these via ScopedLatencyTimer / .record(); the main-thread periodic
// dump consumes them via snapshotAndReset(). Gated by gAudioLogVerbose
// at every call site so production cost is zero.
static LatencyHistogram sPerfDspNodeMs;       // SteamAudioDSPNode per-voice callback
static LatencyHistogram sPerfReflMixMs;       // ReflectionMixNode callback (audio thread)
// Sync-in-callback: time the audio thread spends blocked at the top of the
// reflection mix node waiting for convolution workers to finish processing
// the CURRENT callback's mono input. Eliminates the 1-callback dry/wet gap
// the async pipeline produces; the deadline-fallback path is still present
// so a worker overrun doesn't underrun the device — it just gracefully
// reverts to "use last frame's wet" for that one callback.
static LatencyHistogram sPerfSyncWaitMs;
// Counter: number of callbacks where the sync wait expired without all
// workers reaching their target processedSeq. Logged alongside SYNC_WAIT
// in the periodic perf dump.
static std::atomic<int> sSyncTimeoutCount{0};
// sPerfReflSimMs lives in ReflectionSimulator.cpp next to its only writer
// (iplSimulatorRunReflections) — same extern pattern as sReflSimPeakMs.
extern LatencyHistogram sPerfReflSimMs;

// ── Inter-callback period histogram (H1' of perf-tuning) ───────────────
//
// Records the wall-clock gap between two consecutive entries to
// reflectionMixNodeProcess. The nominal period at 1024 @ 48 kHz is
// 21.333 ms; we want to know how tight that distribution actually is.
// If p99 is materially shorter than p50 (i.e. CoreAudio occasionally
// fires the callback faster than expected), it explains how all 4
// workers can simultaneously appear stale despite having ~10 ms of slack
// on iter time — they may have less than that on some pairs of callbacks.
// Writer: audio thread (one). Reader: main-thread periodic dump.
static LatencyHistogram sPerfInterCallbackMs;
static std::atomic<long long> sLastCallbackNs{0};

// ── Wet-bus beat detector ──
//
// Audio thread pushes one envelope sample (stereo peak) per reflection
// mix-node callback. Main-thread periodic dump linearises the ring and
// autocorrelates over the 1–5 Hz band. Hot for diagnosing the residual
// amplitude-modulation artefact that PLAN.HYBRID_REVERB.md targets.
static WetBusBeatDetector sWetBeat;

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
    // The audio callback iterates non-Free sub-source slots and sums
    // each slot's binaural output into stereoL/R. Each slot owns its
    // own direct + binaural effect pair (pre-allocated in initVoiceDSP).
    // In SA mode only slot 0 is ever ACTIVE; in BFS mode up to
    // kMaxSubSources slots run simultaneously for multi-portal voices.
    // See PLAN.MULTI_PATH_SA_MIGRATION.md for the regime split.
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
                    // Drop the direct effect's internal IIR / EQ state so
                    // the next call doesn't keep producing NaN from the
                    // same poisoned coefficients (matches the binaural
                    // reset below). The direct effect's IIR cascade
                    // (occlusion + air absorption + transmission EQ)
                    // accumulates state per-call; resetting clears the
                    // history without reallocation.
                    iplDirectEffectReset(slot.directEffect);
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
                // Break the NaN cascade. The sanitize above replaces the
                // output buffer with zeros, but Steam Audio's binaural
                // effect keeps its OWN internal FFT delay line + HRIR
                // overlap-add history — once those are poisoned, every
                // subsequent call produces NaN again from the same stale
                // state. Resetting drops the internal buffers to zero so
                // the next call starts clean. Audibly this is a single
                // pop at the reset point (output goes from sanitize-zero
                // to fresh-output discontinuously), which is preferable
                // to a perpetual stream of NaN→zero substitutions —
                // perceptually a continuous crackle exactly like a DAW
                // running with too-small buffers (the user's report).
                //
                // No-allocation, audio-thread-safe — `iplBinauralEffectReset`
                // is documented to be cheap and lock-free.
                iplBinauralEffectReset(slot.binauralEffect);
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
            // Single audio-thread writer per voice (see [VOICE_PEAK] note
            // below); CAS retry loop is uncontested by design, so a plain
            // store-if-greater suffices and avoids the RMW.
            if (monoInPeak  > node->monoInPeak.load(std::memory_order_relaxed))
                node->monoInPeak.store(monoInPeak,  std::memory_order_relaxed);
            if (monoOutPeak > node->monoOutPeak.load(std::memory_order_relaxed))
                node->monoOutPeak.store(monoOutPeak, std::memory_order_relaxed);

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

        // ── Phase 4: iplPathEffectApply (Steam Audio sole authority) ──
        //
        // Run the per-voice path effect against the mono input and mix
        // its spatialized binaural output additively into the dry bus
        // (Valve Unity reference pattern — wet bus on top of direct
        // binaural; the direct binaural is the dry path, the path
        // effect's output is the "routed around obstacles" supplement).
        //
        // Skipped when:
        //   • voice has no path effect (player-emitted voices — created
        //     null in initVoiceDSP because the player IS the listener),
        //   • runAtten is off (debug bypass level 2),
        //   • the main thread has marked the path target invalid
        //     (sentinel + never-solved, out-of-range, or pathing
        //     disabled for this voice).
        // The invariant cache (§5.1) handles sentinel-flicker: when
        // the main thread sees a sentinel AND the voice has previously
        // solved, it replays the last-good params into
        // pathTargetParams + leaves pathTargetValid==true so the audio
        // thread keeps producing the frozen routed signal across the
        // flicker — exactly what the eqCoeffs cache did before Phase 4,
        // just at the IPLPathEffectParams level instead of the scalar
        // level.
        if (runAtten && !node->skipAttenuation
            && node->pathEffect
            && node->pathTargetValid.load(std::memory_order_acquire)
            && !node->pathOutL.empty()
            && !node->pathOutR.empty()) {
            // Pull the latest main-thread-staged params. POD copy — the
            // shCoeffs pointer remains valid (Steam Audio owns it across
            // pathing iterations).
            IPLPathEffectParams pp = node->pathTargetParams;
            // Listener pose + HRTF + binaural flag are per-callback
            // state owned by the audio thread; the main thread can't
            // know them at staging time. Populate now from the audio-
            // thread-visible globals (sPathListenerCoord and
            // sPathHrtf).
            pp.binaural    = IPL_TRUE;
            pp.hrtf        = sPathHrtf.load(std::memory_order_acquire);
            pp.listener    = sPathListenerCoord;
            // normalizeEQ: divide eqCoeffs by max(eqCoeffs) before
            // applying. Counteracts overly-aggressive band drops when
            // the solver returns a small-but-non-sentinel value across
            // all three bands. Conservative default per phonon.h.
            pp.normalizeEQ = IPL_TRUE;
            // order MUST equal the path effect's create-time maxOrder
            // (= mAmbisonicsOrder). Steam Audio's internal rotate stage
            // asserts the encoded SH buffer's channel count matches
            // numCoeffsForOrder(params.order). The solver normally writes
            // this when pathingOrder is set on the source inputs (we set
            // it to mAmbisonicsOrder when staging IPLSimulationInputs),
            // but we override here as a belt-and-suspenders for any stale
            // cached params struct that predates the runtime order —
            // including lastGoodPathParams entries cached before a
            // runtime ambisonics_order change, and the early-startup
            // window before the first solve completes.
            pp.order       = sPathAmbisonicsOrder.load(std::memory_order_acquire);

            // pp.hrtf may legitimately be null during shutdown — skip
            // the apply rather than crash inside Steam Audio.
            if (pp.hrtf) {
                float* monoInPtr = mono;
                IPLAudioBuffer pathIn{};
                pathIn.numChannels = 1;
                pathIn.numSamples  = static_cast<IPLint32>(frameCount);
                pathIn.data        = &monoInPtr;

                float* pathOutChans[2] = {node->pathOutL.data(),
                                          node->pathOutR.data()};
                IPLAudioBuffer pathOut{};
                pathOut.numChannels = 2;
                pathOut.numSamples  = static_cast<IPLint32>(frameCount);
                pathOut.data        = pathOutChans;

                iplPathEffectApply(node->pathEffect, &pp,
                                   &pathIn, &pathOut);

                // Sanitize before additive mix so a runaway NaN doesn't
                // poison the dry bus permanently.
                bool badPathL = audioSanitizeBuffer(node->pathOutL.data(), frameCount);
                bool badPathR = audioSanitizeBuffer(node->pathOutR.data(), frameCount);
                if (badPathL || badPathR) {
                    // Reset the path effect's internal FFT/EQ state to
                    // break any persistent NaN cascade. Cheap per
                    // phonon.h; one audible pop at reset vs continuous
                    // grain.
                    iplPathEffectReset(node->pathEffect);
                }

                // Additive mix into the voice's dry stereo bus.
                for (ma_uint32 i = 0; i < frameCount; ++i) {
                    chL[i] += node->pathOutL[i];
                    chR[i] += node->pathOutR[i];
                }
            }
        }

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
        // Reflection apply gate.
        //   • CONVOLUTION / HYBRID: need a real IR (irSize > 0). HYBRID
        //     additionally consults reverbTimes for the parametric tail,
        //     but the convolution front still requires irSize.
        //   • PARAMETRIC: no IR ever exists — the apply call uses only
        //     reverbTimes. Gate solely on the type field.
        const bool reflParamsValid =
            (node->reflectionParams.type == IPL_REFLECTIONEFFECTTYPE_PARAMETRIC)
                ? true
                : (node->reflectionParams.irSize > 0);
        if (node->reflectionsActive && node->reflectionEffect
            && reflParamsValid && node->convWorker) {
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

                // Sanitize the reflection-send mono buffer BEFORE handing it
                // off to the convolution worker. A single NaN/Inf sample here
                // gets convolved against an IR of tens of thousands of taps
                // (~52800 at 48 kHz for ambisonics-order-1) before the wet-bus
                // NaN guard catches it post-effect — by then a long patch of
                // wet output is already corrupted (silence-replacement looks
                // like broadband grain after IR smearing). Sanitizing the
                // *input* contains the blast radius to one sample and surfaces
                // the upstream producer via [NAN_GUARD] so the real source
                // (reflAtten, reflSendAlpha, or the mono pre-LPF) can be
                // tracked down. Replaces non-finite samples with 0 in-place.
                if (audioSanitizeBuffer(slot.mono.data(),
                                        static_cast<std::size_t>(node->reflectionFrameSize))) {
                    uint32_t n = node->nanCountReflInput.fetch_add(
                        1, std::memory_order_relaxed);
                    if (n < 4) {
                        AUDIO_LOG("[NAN_GUARD] node=%p reflection input had "
                                  "non-finite samples before convolution "
                                  "(occurrence %u) reflAtten=%.4f "
                                  "reflSendAlpha=%.4f reflLpfState=%.6g\n",
                                  static_cast<void*>(node), n + 1,
                                  reflAtten, reflSendAlpha,
                                  static_cast<double>(node->reflSendLpfState));
                    }
                    // If the LPF state itself went non-finite, it would
                    // re-poison every subsequent callback. Reset it to a
                    // safe value so the corruption can't persist across
                    // callbacks even if the upstream producer keeps emitting
                    // bad samples.
                    if (!std::isfinite(node->reflSendLpfState))
                        node->reflSendLpfState = 0.0f;
                }

                slot.effect = node->reflectionEffect;
                // The shared_ptr keeps the validity-atomic alive past the
                // owning ActiveVoice's destruction (workers may still read
                // it between iterations — see ~ActiveVoice in VoicePool.cpp).
                // Voice→slot assignment is stable in steady state, so the
                // common case is "slot already holds this voice's token"
                // and we can skip the assignment entirely. Avoiding the
                // copy avoids two atomic ref-count RMWs per voice per
                // callback (~10 voices × 93 Hz = ~2 k atomics/sec); only
                // matters cumulatively, but free to eliminate.
                if (slot.validityToken.get() != node->validityToken.get())
                    slot.validityToken = node->validityToken;
                slot.params = node->reflectionParams;
                // Runtime IR clamp: cap the convolution length to a
                // configurable maximum (in reflection-rate samples). 0 =
                // disabled. Steam Audio convolves the first irSize samples
                // of the effect's IR, so reducing irSize directly reduces
                // per-voice CPU at iplReflectionEffectApply time. Both
                // baked-probe and realtime-simulated voices flow through
                // here, so a single knob covers both. Live-tunable from
                // setRuntimeIrClampMs (sampled atomically every frame).
                {
                    int clampSamples = sRuntimeIrClampSamples.load(
                        std::memory_order_relaxed);
                    if (clampSamples > 0 && slot.params.irSize > clampSamples) {
                        // One-shot log per clamp engagement: prints first
                        // ~8 times to confirm the knob is reaching the apply
                        // path. Throttled hard so it doesn't flood under
                        // many active voices.
                        static std::atomic<int> sClampLogCount{0};
                        int n = sClampLogCount.fetch_add(1, std::memory_order_relaxed);
                        if (n < 8) {
                            AUDIO_LOG("[IR_CLAMP] before=%d after=%d "
                                      "(clamp=%d samples) #%d\n",
                                      slot.params.irSize, clampSamples,
                                      clampSamples, n + 1);
                        }
                        slot.params.irSize = clampSamples;
                    }
                }
                slot.reflFrameSize = node->reflectionFrameSize;

                // ── Silent-voice apply elision ───────────────────────────────
                //
                // After the reflection-send mono has been written above (with
                // reflAtten + door LPF applied), peek the buffer for any
                // nonzero sample. Tail voices in the convolution pool
                // (sourceEnded but reflectionsActive=true while their reverb
                // rings out) — and even active voices whose distance
                // attenuation has driven the send to ~zero — feed all-zero
                // mono into iplReflectionEffectApply, which costs the same
                // CPU as a non-silent voice but produces no useful output
                // once the IR delay line has drained.
                //
                // We feed zeros through apply() for kReflSilentSkipFrames
                // consecutive callbacks first — long enough that the
                // convolution's internal delay line (≈ irSize samples) and
                // the FDN parametric tail have fully decayed. Past that
                // point the worker can safely skip the apply: there is no
                // stale history left that the next non-zero input would
                // smear against incorrectly. We signal the skip by setting
                // slot.active = false; the worker's existing slot loop in
                // ConvolutionWorkerPool.cpp already short-circuits on that.
                //
                // The skip restores per-callback worker iter time to the
                // pre-CONV_LAG envelope: ~N_audible_voices × apply_cost
                // rather than N_total_voices × apply_cost. Each IR-update
                // crossfade roughly doubles apply_cost, so the savings are
                // most pronounced exactly at the sim-cycle boundaries where
                // the [CONV_LAG] overruns previously fired.
                //
                // Threshold derives from the live `realtime.duration` knob
                // via reflSilentSkipFrames() in AudioUnits.h: we feed zeros
                // for `(IR length in callbacks) × 2 + 1` callbacks so both
                // the conv delay-line and the FDN parametric tail drain
                // before the apply is elided. Raising `realtime.duration`
                // automatically widens the skip window — no manual update
                // needed.
                {
                    const int kReflSilentSkipFrames =
                        sReflSilentSkipFrames.load(std::memory_order_relaxed);
                    constexpr float kSilenceEpsilon = 1.0e-6f;
                    float reflInPeak = 0.0f;
                    for (ma_uint32 i = 0; i < reflFrames; ++i) {
                        const float a = std::fabs(slot.mono[i]);
                        if (a > reflInPeak) reflInPeak = a;
                    }
                    if (reflInPeak > kSilenceEpsilon) {
                        node->framesSinceNonzeroReflInput = 0;
                    } else if (node->framesSinceNonzeroReflInput
                               < kReflSilentSkipFrames + 1) {
                        // Cap the counter at threshold+1 so it doesn't
                        // wrap around for very-long-silent voices.
                        ++node->framesSinceNonzeroReflInput;
                    }
                    slot.active =
                        (node->framesSinceNonzeroReflInput <= kReflSilentSkipFrames);
                }
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
                    // Single audio-thread writer per voice — non-CAS store.
                    if (sPeak > node->reflSendPeak.load(std::memory_order_relaxed))
                        node->reflSendPeak.store(sPeak, std::memory_order_relaxed);
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
        // CLEANUP.  std::atomic<float> doesn't have fetch_max, but this is
        // single-writer (only one audio thread writes per voice), so a plain
        // store-if-greater replaces the CAS retry — same result, no RMW.
        {
            int prevFrames = node->lifetimeFrameCount.fetch_add(1, std::memory_order_relaxed);
            float prevL = node->lifetimePeakL.load(std::memory_order_relaxed);
            // Snapshot direction whenever we update the L peak — gives us
            // the listener-local source direction at the moment the voice
            // was loudest.  If identical voices produce different lifetime
            // peaks AND different direction-at-peak vectors, the listener
            // is moving during voice playback and HRTF gain is varying.
            bool updated = false;
            if (peakL > prevL) {
                node->lifetimePeakL.store(peakL, std::memory_order_relaxed);
                if (peakL > 0.0f) updated = true;
            }
            float prevR = node->lifetimePeakR.load(std::memory_order_relaxed);
            if (peakR > prevR)
                node->lifetimePeakR.store(peakR, std::memory_order_relaxed);
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
        // Feed the per-stage histogram — same data as `us` above, but in
        // a form the [PERF] dump can derive p50/p95/p99 from.
        sPerfDspNodeMs.record(us / 1000.0);
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

    // H1' inter-callback period. Measured even when profiling is off
    // since it's just one timestamp + one subtract — far cheaper than the
    // existing FTZ enable below. The first sample (sLastCallbackNs==0)
    // is skipped to avoid recording an enormous fake interval relative
    // to the steady_clock epoch.
    {
        const long long nowNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        const long long lastNs = sLastCallbackNs.exchange(
            nowNs, std::memory_order_relaxed);
        if (lastNs > 0) {
            const long long deltaNs = nowNs - lastNs;
            if (deltaNs > 0) {
                sPerfInterCallbackMs.record(
                    static_cast<double>(deltaNs) / 1.0e6);
            }
        }
    }
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

    ConvolutionWorker *cw = node->convWorker;

    // ── Sync-in-callback: kick off convolution for the CURRENT callback's
    // mono input and wait for it before reading the wet bus. ───────────
    //
    // Before this change the worker handoff happened AFTER the wet bus
    // read, so workers ran asynchronously on the next callback period and
    // the wet bus we summed was always one callback period (~10.7 ms at
    // 512/48k) behind the dry path — a static pre-delay added to every
    // reflection, indistinguishable from "every space is a small room."
    //
    // Now: per-voice DSP nodes have already written their mono into
    // staging[writeIdx] by the time we get here. We assign slots + signal
    // workers immediately, then wait (bounded by `syncDeadlineMs`) for
    // their processedSeq to advance before reading the front buffers.
    // Workers process in parallel with each other on their own threads —
    // the audio thread blocks only for the longest sub-worker's iter.
    //
    // Deadline fallback: if the wait expires, we fall through and read
    // whatever's currently in the front buffers (== the previous frame's
    // output, same as the pre-change behaviour). One callback of stale
    // wet is far better than an underrun. The existing [WET_BUS] staleness
    // tracking naturally flags these events; the SYNC_WAIT histogram +
    // sSyncTimeoutCount counter give per-window rates.
    uint64_t targetSeq[64] = {0};
    int numSignalled = 0;
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
        // drop path. With sync-in-callback this should essentially never fire
        // — the wait loop blocks the audio thread until workers catch up — but
        // it's retained as a safety net for shutdown / deadline-expiry paths.
        int w = cw->writeIdx.load(std::memory_order_relaxed);
        int prevRead = cw->currentReadBuf;

        bool workersTwoBehind = false;
        for (auto &subPtr : cw->workers) {
            uint64_t fs = subPtr->frameSeq.load(std::memory_order_relaxed);
            uint64_t ps = subPtr->processedSeq.load(std::memory_order_relaxed);
            if (fs >= ps + 2) { workersTwoBehind = true; break; }
        }

        if (workersTwoBehind) {
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

            // Stamp the signal time on each worker BEFORE bumping frameSeq.
            // The worker's acquire-load on frameSeq makes this write visible
            // to the worker, which uses it to compute the signal→pickup
            // latency for the H2' diagnostic.
            const long long signalNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            for (auto &subPtr : cw->workers)
                subPtr->signalTimeNs.store(signalNs, std::memory_order_release);

            // Signal all sub-workers and snapshot their target seq for the
            // wait loop below. The frameSeq bump is the canonical seq
            // advance (acquire/release pair with the worker's predicate
            // check); the notify_one wakes the worker if it's currently
            // in cv.wait(). Order matters: bump frameSeq BEFORE notify so
            // the worker's predicate check inside cv.wait sees the new
            // value if it wakes due to our notify (vs spurious wake).
            const int maxSnap = sizeof(targetSeq) / sizeof(targetSeq[0]);
            for (auto &subPtr : cw->workers) {
                uint64_t newSeq = subPtr->frameSeq.fetch_add(
                    1, std::memory_order_release) + 1;
                subPtr->wakeCv.notify_one();
                if (numSignalled < maxSnap)
                    targetSeq[numSignalled++] = newSeq;
            }
        }
    }

    // ── Bounded wait for sub-workers ───────────────────────────────────
    //
    // Wait until each signalled sub-worker's processedSeq reaches its
    // target value, bounded by `node->syncDeadlineMs`. Workers run their
    // DSP on dedicated threads in parallel; the audio thread's only job
    // here is to wait the longest sub-worker out.
    //
    // Previous design yield-polled with std::this_thread::yield() inside
    // the deadline window. On macOS, yield doesn't put the thread to
    // sleep — it just gives up the rest of the quantum — so when any
    // worker was preempted (typical when the reflection sim cycle ends
    // and 6 sim threads all become non-runnable simultaneously) the
    // audio thread would burn the full deadline yield-spinning before
    // falling back to stale wet. Observed in [PERF sync_wait]: p50 ≈
    // half the callback period, periodic timeout bursts of 12–82
    // callbacks per window.
    //
    // Condvar wait_until lets the audio thread actually sleep, frees
    // the core for the workers, and wakes the instant any worker
    // bumps its processedSeq + notify_one. Predicate re-checks every
    // worker so a single notify suffices regardless of which worker
    // signalled.
    auto syncT0 = std::chrono::steady_clock::now();
    bool syncTimedOut = false;
    if (cw && numSignalled > 0) {
        const auto deadline = syncT0 + std::chrono::microseconds(
            static_cast<long long>(node->syncDeadlineMs * 1000.0f));
        std::unique_lock<std::mutex> lk(cw->doneMtx);
        bool ok = cw->doneCv.wait_until(lk, deadline, [&]() {
            for (int i = 0; i < numSignalled; ++i) {
                if (cw->workers[i]->processedSeq.load(
                        std::memory_order_acquire) < targetSeq[i])
                    return false;
            }
            return true;
        });
        syncTimedOut = !ok;
    }
    {
        auto syncT1 = std::chrono::steady_clock::now();
        double waitMs = std::chrono::duration<double, std::milli>(
            syncT1 - syncT0).count();
        sPerfSyncWaitMs.record(waitMs);
        if (syncTimedOut)
            sSyncTimeoutCount.fetch_add(1, std::memory_order_relaxed);
    }

    // Read all sub-workers' front buffers (now CURRENT callback's decoded
    // reverb, if the sync wait succeeded — otherwise the previous frame's
    // output as a graceful fallback) and sum into the wet scratch buffers.
    // The dedicated scratch path is necessary so the tape saturator (below)
    // sees the FULL summed wet bus contribution rather than per-sub-worker
    // pieces — saturating each piece independently would clip transients
    // that, summed, would cancel.
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
                const bool wasStale = (p == sLastConsumedSeq[wkIdx]);
                if (wasStale) {
                    ++wetStaleFronts;
                    // H3' per-stale-event gap log. The gap = now − the
                    // signal time of the most recent frameSeq bump for
                    // this worker. If the gap is comparable to one
                    // callback period (~21 ms), the worker is one frame
                    // behind because it didn't finish iter N before we
                    // arrived to do iter N+1's freshness check. If it's
                    // much larger, the worker was scheduled out for
                    // multiple periods. Throttled to ~1 of every 32
                    // events so a sustained backlog doesn't drown the log.
                    static std::atomic<uint32_t> sStaleLogCount{0};
                    uint32_t n = sStaleLogCount.fetch_add(1, std::memory_order_relaxed);
                    if ((n & 0x1F) == 0) {
                        const long long signalNs = sub.signalTimeNs.load(
                            std::memory_order_acquire);
                        const long long nowNs = std::chrono::duration_cast<
                            std::chrono::nanoseconds>(
                                std::chrono::steady_clock::now().time_since_epoch()).count();
                        const double gapMs = (signalNs > 0)
                            ? static_cast<double>(nowNs - signalNs) / 1.0e6
                            : -1.0;
                        AUDIO_LOG("[STALE_GAP] w=%zu since_signal_ms=%.3f "
                                  "processedSeq=%llu lastConsumed=%llu (occurrence #%u)\n",
                                  wkIdx, gapMs,
                                  (unsigned long long)p,
                                  (unsigned long long)sLastConsumedSeq[wkIdx],
                                  n + 1);
                    }
                }
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

        // Apply global reflection-bus gain + first-activation fade-in ramp.
        // `reflGainTarget` is the YAML `audio.mixer.reflection_gain` value
        // (updated live by setMixerSettings). `reflGain` ramps from 0 toward
        // the target at `reflRampRate` per sample so the first time reverb
        // engages it fades in over `reflection_ramp_ms`, then stays at the
        // target. Once at target the loop is a single multiply per sample.
        // The previous wet-bus refactor (triple-buffered staging) dropped
        // this multiply entirely — reflection_gain stopped affecting the
        // output because nothing multiplied by it anywhere. Applied here
        // (after sub-worker sum, before saturator) so the saturator sees
        // the gain-scaled signal that's about to hit the dry bus.
        if (node->wetScratchL.size() >= outSamples) {
            float g = node->reflGain;
            const float target = node->reflGainTarget;
            const float step = node->reflRampRate;
            for (ma_uint32 i = 0; i < outSamples; ++i) {
                if (g < target) {
                    g += step;
                    if (g > target) g = target;
                }
                node->wetScratchL[i] *= g;
                node->wetScratchR[i] *= g;
            }
            node->reflGain = g;
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

        // H3 stale-burst tracker (callback-level, not worker-frame-level).
        //
        // The stale% above counts worker-frames (workers × callbacks). It
        // tells us the rate but not the temporal pattern. A "burst" here
        // is a run of consecutive callbacks where at least one sub-worker
        // had a stale front buffer — exactly the pattern that produces
        // audible amplitude modulation on the wet bus (held-flat held
        // for K callbacks in a row → step discontinuity perceived as a
        // K × callbackPeriod-long modulation).
        //
        // sCurrentBurstLen — running burst length, resets on first clean
        //                    callback. Atomic only for hygiene; only the
        //                    mix node (one thread) writes it.
        // sBurstCount      — number of bursts started in the current
        //                    [WET_BUS] window. burstCount / window =
        //                    burst rate.
        // sMaxBurstLen     — longest burst seen in the window. A long
        //                    burst (≥3 consecutive degraded callbacks)
        //                    means the worker pool fell that far behind.
        // sDegradedCb /
        // sCleanCb         — callbacks classified at the callback level.
        //                    Different from staleAccum/freshAccum which
        //                    are worker-frame totals.
        static std::atomic<int> sCurrentBurstLen{0};
        static std::atomic<int> sBurstCount{0};
        static std::atomic<int> sMaxBurstLen{0};
        static std::atomic<int> sDegradedCb{0};
        static std::atomic<int> sCleanCb{0};
        if (cw) {
            int fresh = std::max(0, wetWorkersHit - wetStaleFronts);
            sStaleAccum.fetch_add(wetStaleFronts, std::memory_order_relaxed);
            sFreshAccum.fetch_add(fresh,          std::memory_order_relaxed);

            const bool cbDegraded = (wetStaleFronts > 0);
            if (cbDegraded) {
                int newLen = sCurrentBurstLen.fetch_add(
                    1, std::memory_order_relaxed) + 1;
                if (newLen == 1)
                    sBurstCount.fetch_add(1, std::memory_order_relaxed);
                int curMax = sMaxBurstLen.load(std::memory_order_relaxed);
                while (newLen > curMax
                       && !sMaxBurstLen.compare_exchange_weak(
                            curMax, newLen, std::memory_order_relaxed)) {
                    // curMax updated by CAS on failure; retry.
                }
                sDegradedCb.fetch_add(1, std::memory_order_relaxed);
            } else {
                sCurrentBurstLen.store(0, std::memory_order_relaxed);
                sCleanCb.fetch_add(1, std::memory_order_relaxed);
            }
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

            // H3 burst stats. NOTE: do NOT reset sCurrentBurstLen — a
            // burst may straddle the log window boundary. The other
            // counters are window-local and reset here.
            int bursts     = sBurstCount.exchange(0, std::memory_order_relaxed);
            int maxBurst   = sMaxBurstLen.exchange(0, std::memory_order_relaxed);
            int degradedCb = sDegradedCb.exchange(0, std::memory_order_relaxed);
            int cleanCb    = sCleanCb.exchange(0, std::memory_order_relaxed);
            int totalCb    = degradedCb + cleanCb;
            float cbDegradedPct = (totalCb > 0)
                ? (100.0f * degradedCb / totalCb) : 0.0f;

            AUDIO_LOG("[WET_BUS] peakL=%.4f peakR=%.4f peakDb=%.1f preDb=%.1f satΔ=%.1fdB "
                      "drive=%.2f workersHit=%d stale=%d fresh=%d stale%%=%.1f "
                      "bursts=%d maxBurst=%d cbDeg=%d/%d (%.1f%%) "
                      "listenerPos=(%.1f,%.1f,%.1f)\n",
                      wetPeakL, wetPeakR, peakDb, preDb, satDeltaDb,
                      node->wetSaturationEnabled ? node->wetSaturationDrive : 1.0f,
                      wetWorkersHit, staleW, freshW, stalePct,
                      bursts, maxBurst, degradedCb, totalCb, cbDegradedPct,
                      node->listenerPosX, node->listenerPosY, node->listenerPosZ);
        }
    }

    // (Worker signal + sync wait have moved to the top of the mix node,
    // before the wet bus read. See the "Sync-in-callback" block earlier.)

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
        // Feed the per-stage histogram for the mix node.
        sPerfReflMixMs.record(mixUs / 1000.0);
    }

    // Beat detector: push one envelope sample per mix-node callback,
    // independent of the gAudioLogVerbose gate so the detector keeps
    // tracking even when verbose perf logging is off. Single producer
    // (audio thread), single consumer (main-thread periodic dump). The
    // envelope sample is the stereo peak post-saturation — exactly the
    // signal that reaches the engine endpoint, so any 1–5 Hz amplitude
    // modulation visible to the listener is visible to the autocorrelator.
    {
        float envPeak = 0.0f;
        if (stereoOut) {
            for (ma_uint32 i = 0; i < frameCount; ++i) {
                float l = std::fabs(stereoOut[i * 2]);
                float r = std::fabs(stereoOut[i * 2 + 1]);
                envPeak = std::max(envPeak, std::max(l, r));
            }
        }
        sWetBeat.push(envPeak);
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

    // Construct the pathing-sim subsystem. Same lifecycle pattern as the
    // reflection sim: thread starts in bootstrapFinished, simulator handle
    // plugged in by buildAcousticScene. Moves iplSimulatorRunPathing off
    // the main loop — the worst observed iteration was 11+ seconds on
    // MISS6 with dynamic door geometry invalidating baked edges.
    mPathingSim = std::make_unique<PathingSimulator>();

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

// Device data callback. We own the playback device ourselves (instead of
// letting ma_engine_init build one) so we can set `ma_device_config.periods`
// — the engine config does not expose period count, so its internal device
// is always created at MA_DEFAULT_PERIODS (= 3). A lower period count
// directly cuts device-buffer latency, which dominates the total
// output-latency budget. The trade-off is less slack for the OS scheduler
// before underrun; periods=2 matches what FMOD/Wwise use on PC by default.
static void engineDeviceDataCallback(ma_device *pDevice, void *pFramesOut,
                                     const void *pFramesIn, ma_uint32 frameCount)
{
    (void)pFramesIn;
    auto *pEngine = static_cast<ma_engine*>(pDevice->pUserData);
    if (pEngine)
        ma_engine_read_pcm_frames(pEngine, pFramesOut, frameCount, nullptr);
}

//------------------------------------------------------
bool AudioService::initMiniaudio()
{
    mMaEngine = new ma_engine();

    // Build our own playback device first so we can pin `periods`. The
    // device config has the field we need (line 7108 of miniaudio.h); the
    // engine config does not. Without this override miniaudio defaults to
    // MA_DEFAULT_PERIODS = 3, which adds ~64 ms of device-buffer latency at
    // 1024-frame periods. periods=2 halves that to ~43 ms.
    ma_device_config deviceConfig = ma_device_config_init(ma_device_type_playback);
    deviceConfig.playback.format     = ma_format_f32;
    deviceConfig.playback.channels   = 2;          // stereo (HRTF requires it)
    deviceConfig.sampleRate          = static_cast<ma_uint32>(mAudioSampleRateCfg);
    deviceConfig.periodSizeInFrames  = static_cast<ma_uint32>(mAudioFrameSizeCfg);
    deviceConfig.periods             = 2;          // tighter latency than default 3
    deviceConfig.performanceProfile  = ma_performance_profile_low_latency;
    deviceConfig.dataCallback        = engineDeviceDataCallback;
    deviceConfig.pUserData           = mMaEngine;
    deviceConfig.noPreSilencedOutputBuffer = MA_TRUE;  // we fill every frame
    deviceConfig.noClip              = MA_TRUE;        // engine clips itself

    mMaDevice = new ma_device();
    ma_result result = ma_device_init(nullptr, &deviceConfig, mMaDevice);
    if (result != MA_SUCCESS) {
        LOG_ERROR("AudioService: ma_device_init failed (error %d)", result);
        AUDIO_LOG("AudioService: ma_device_init FAILED (error %d)\n", result);
        delete mMaDevice;
        mMaDevice = nullptr;
        delete mMaEngine;
        mMaEngine = nullptr;
        return false;
    }

    // Engine sample rate is configurable (default 48kHz).
    // 48kHz is preferred — Steam Audio's HRTF dataset is native at 48kHz and any
    // other rate forces internal resampling. miniaudio still resamples between
    // engine and device rates if those differ.
    ma_engine_config config = ma_engine_config_init();
    config.channels = 2;           // stereo output (hardcoded — binaural HRTF requires stereo)
    config.sampleRate = static_cast<ma_uint32>(mAudioSampleRateCfg);
    config.listenerCount = 1;      // one listener for 3D spatialization
    config.periodSizeInFrames = static_cast<ma_uint32>(mAudioFrameSizeCfg);
    // Pre-built device: engine skips its own device creation and uses ours.
    // The data callback we installed above forwards into ma_engine_read_pcm_frames.
    config.pDevice = mMaDevice;

    result = ma_engine_init(&config, mMaEngine);
    if (result != MA_SUCCESS) {
        LOG_ERROR("AudioService: miniaudio init failed (error %d)", result);
        AUDIO_LOG( "AudioService: miniaudio init FAILED (error %d)\n", result);
        ma_device_uninit(mMaDevice);
        delete mMaDevice;
        mMaDevice = nullptr;
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
        // LOG_INFO (not AUDIO_LOG) so we always capture the engine↔device
        // rate relationship — critical for diagnosing rate-mismatch
        // resampling artefacts, and needed for triage even when the
        // verbose audio log is off. Includes `internalPeriods` so the
        // current device-buffer latency is computable from the log:
        //   buffer_ms = (period_frames * periods) / sample_rate * 1000
        ma_uint32 periodCount = device->playback.internalPeriods;
        float bufferMs = 0.0f;
        if (device->sampleRate > 0)
            bufferMs = 1000.0f * static_cast<float>(periodSize * periodCount)
                     / static_cast<float>(device->sampleRate);
        LOG_INFO("AudioService: miniaudio engine %u Hz \xe2\x86\x92 device '%s' @ %u Hz, "
                 "%u ch, period=%u frames, periods=%u (buffer=%.1f ms)",
                 mDeviceSampleRate, device->playback.name,
                 device->sampleRate, device->playback.channels,
                 mFrameSize, periodCount, bufferMs);
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
void AudioService::setRuntimeIrClampMs(float ms)
{
    // Clamp to [0, 4000] — 4 s is the longest IR Steam Audio supports in
    // any of our paths, so values beyond that are a typo. 0 disables.
    mRuntimeIrClampMs = std::max(0.0f, std::min(ms, 4000.0f));
    // Convert ms → reflection-rate samples and publish to the atomic the
    // audio thread reads at staging-slot setup. If the reflection sample
    // rate isn't known yet (call before initReflectionPipeline), the
    // conversion uses 0 → clamp disabled until the pipeline init re-runs
    // this conversion via a second setRuntimeIrClampMs from the config
    // pump on subsequent loop steps.
    int samples = 0;
    if (mReflectionSampleRate > 0 && mRuntimeIrClampMs > 0.0f) {
        samples = static_cast<int>(0.001f * mRuntimeIrClampMs
            * static_cast<float>(mReflectionSampleRate));
        if (samples < 1) samples = 1;
    }
    sRuntimeIrClampSamples.store(samples, std::memory_order_relaxed);
    LOG_INFO("AudioService: runtime IR clamp set to %.1f ms = %d samples "
             "(refl rate=%u Hz)", mRuntimeIrClampMs, samples,
             mReflectionSampleRate);
}
bool AudioService::getHalfRateReflections() const
{
    return mReflectionSim ? mReflectionSim->getRateDivisor() >= 2 : true;
}
//------------------------------------------------------
// Forwards to AudioDSPChain, which writes to the file-scope atomics defined
// above (sHrtfInterpolation / sSpatialBlend / sDoorLpfOpenHz /
// sDoorLpfBlockedHz / sPropMinAttenuation) via extern declarations in
// AudioDSPChain.cpp. Keeping the atomic definitions in this TU means the
// audio callback continues to read them with no extra indirection.
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

void AudioService::setAmbGlobalVolumeScale(float s)
{
    if (mAmbientManager)
        mAmbientManager->setGlobalVolumeScale(std::max(0.0f, std::min(s, 4.0f)));
}

//------------------------------------------------------
// Voice-state accessors used by AmbientSoundManager. Defined here (rather
// than inline in the header) so AmbientSoundManager.cpp doesn't need to
// see the full ActiveVoice / ReflectionMixNode struct definitions.
bool AudioService::voiceExists(SoundHandle handle) const
{
    return mVoicePool && mVoicePool->exists(handle);
}

void AudioService::voiceSetMaxAudibleDist(SoundHandle handle, float maxDist)
{
    if (!mVoicePool) return;
    ActiveVoice *v = mVoicePool->find(handle);
    if (!v) return;
    v->maxAudibleDist = maxDist;
}

void AudioService::voiceSetAttenuationFactor(SoundHandle handle, float factor)
{
    if (!mVoicePool) return;
    ActiveVoice *v = mVoicePool->find(handle);
    if (!v) return;
    // Clamp to a sane range. Steam Audio's INVERSEDISTANCE minDistance
    // is in meters, so factor=0.1 → 0.1m (effectively no full-volume zone)
    // and factor=100 → 100m (sound essentially never attenuates with
    // distance). The Dark Engine ships factors in [1, ~20].
    v->attenuationFactor = std::max(0.1f, std::min(factor, 100.0f));
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
    // Engine before device: ma_engine_uninit only tears down node graph and
    // listeners — it does NOT uninit pDevice when one was supplied
    // externally (see ma_engine_init source). We own the device's lifetime.
    if (mMaEngine) {
        ma_engine_uninit(mMaEngine);
        delete mMaEngine;
        mMaEngine = nullptr;
    }
    if (mMaDevice) {
        ma_device_uninit(mMaDevice);
        delete mMaDevice;
        mMaDevice = nullptr;
    }
    LOG_INFO("AudioService: miniaudio shut down");
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
    // Phase 4: publish the HRTF handle for the audio thread's per-voice
    // iplPathEffectApply (binaural=true). Audio thread reads via atomic
    // load; main thread writes once at init and once at shutdown.
    sPathHrtf.store(mIplHrtf, std::memory_order_release);
    // Publish the runtime ambisonics order so the audio thread can stamp
    // it onto every IPLPathEffectParams it passes to iplPathEffectApply —
    // must equal IPLPathEffectSettings::maxOrder used at create time, or
    // Steam Audio's internal rotate stage asserts.
    sPathAmbisonicsOrder.store(mAmbisonicsOrder, std::memory_order_release);

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
        // Clear the audio-thread-visible pointer BEFORE releasing the
        // backing handle so a callback racing the shutdown sees null
        // (and skips iplPathEffectApply) rather than a dangling handle.
        sPathHrtf.store(nullptr, std::memory_order_release);
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
        sceneTypeEnum = IPL_SCENETYPE_EMBREE;
    }

    try {
        // Step 1: Create IPLScene (Steam Audio's built-in CPU raytracer).
        // If the user asked for embree and Steam Audio wasn't built with
        // embree linked, iplSceneCreate returns an error — fall back to
        // the default raytracer with a loud [FALLBACK] message so the
        // user notices and either fixes the build or drops the knob.
        IPLSceneSettings sceneSettings{};
        sceneSettings.type = sceneTypeEnum;

        IPLerror err = iplSceneCreate(mIplContext, &sceneSettings, &mIplScene);
        if (err != IPL_STATUS_SUCCESS && sceneTypeEnum == IPL_SCENETYPE_EMBREE) {
            std::fprintf(stderr,
                "[FALLBACK] iplSceneCreate(embree) failed (error %d) — Steam Audio "
                "was not built with Embree support. Falling back to scene_type=default. "
                "Set audio.performance.scene_type: default in darknessRender.yaml to "
                "silence this warning.\n",
                err);
            sceneTypeEnum = IPL_SCENETYPE_DEFAULT;
            mSceneTypeCfg = "default";
            sceneSettings.type = sceneTypeEnum;
            err = iplSceneCreate(mIplContext, &sceneSettings, &mIplScene);
        }
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
        // Re-publish the runtime IR clamp now that mReflectionSampleRate
        // is known. The setter call from RenderConfig wiring at startup
        // ran before this, when sample rate was still 0 → the atomic
        // would have been left at 0 (clamp disabled) even if a non-zero
        // ms value was configured. Re-running with the same ms now
        // converts to a real sample count.
        setRuntimeIrClampMs(mRuntimeIrClampMs);

        // Re-publish the silent-voice-skip threshold for the same reason.
        // It is derived from mRealtimeDuration + the rates we just settled
        // above; if it weren't republished here, the audio callback would
        // continue reading the bootstrap default until the next call to
        // setRealtimeDuration / setRateDivisor.
        sReflSilentSkipFrames.store(
            reflSilentSkipFrames(mRealtimeDuration,
                                 static_cast<int>(mReflectionSampleRate),
                                 static_cast<int>(mReflectionFrameSize)),
            std::memory_order_relaxed);

        // ── Reverb-thread budget split ──
        //
        // Total budget: mReverbThreadsCfg (0 = auto: hwconc - 2, reserving
        // 2 cores for main + audio). Split between convolution workers and
        // ray-trace simulator threads using mReverbThreadsConvShareCfg.
        //
        // Auto-split policy (-1): bias toward sim in both modes. Empirically
        // (see SESSION_LOGS 2026-05-20) the sim is the bottleneck regardless
        // of whether realtime voices consume its output: per-cycle cost is
        // hundreds of ms and scales with maxNumRays + numDiffuseSamples,
        // both of which are paid up front at simulator-create time.
        // Convolution has plenty of headroom — for the typical 12-16 voice
        // load each apply is ~2-3 ms and parallelizes trivially across
        // workers, so 3-4 workers is enough to stay well under the 21 ms
        // callback budget. Realtime configs bias slightly more toward
        // convolution because realtime voices add per-voice apply work.
        //
        // The invariant `conv + sim == total` is enforced here so the user
        // cannot over-allocate regardless of how the share is set.
        {
            int totalThreads;
            if (mReverbThreadsCfg > 0) {
                totalThreads = mReverbThreadsCfg;
            } else {
                unsigned int hwThreads = std::thread::hardware_concurrency();
                totalThreads = static_cast<int>(std::max(4u,
                    hwThreads > 2 ? hwThreads - 2 : 2u));
            }
            // Each side needs at least 1; clamp to a sensible floor.
            if (totalThreads < 2) totalThreads = 2;

            float convShare;
            if (mReverbThreadsConvShareCfg < 0.0f) {
                convShare = (mReverbVoicesRealtime == 0) ? 0.35f : 0.45f;
            } else {
                convShare = mReverbThreadsConvShareCfg;
            }
            mConvolutionWorkerCount = std::max(1,
                static_cast<int>(static_cast<float>(totalThreads) * convShare + 0.5f));
            mSimulatorThreadCount   = std::max(1, totalThreads - mConvolutionWorkerCount);
            // Re-adjust if rounding pushed convolution above total - 1.
            if (mConvolutionWorkerCount + mSimulatorThreadCount > totalThreads) {
                mConvolutionWorkerCount = totalThreads - mSimulatorThreadCount;
            }
            AUDIO_LOG("[REVERB_THREADS] budget=%d conv=%d sim=%d (share=%.2f%s, realtime=%d)\n",
                      totalThreads, mConvolutionWorkerCount, mSimulatorThreadCount,
                      convShare, mReverbThreadsConvShareCfg < 0.0f ? " auto" : "",
                      mReverbVoicesRealtime);
        }

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
        // Sim caps. maxNumRays must be >= whichever pipeline branch (realtime
        // or bake) will fire the most rays per cycle; auto-derived rather
        // than a separate knob.
        simSettings.maxNumOcclusionSamples = mSimMaxOcclusionSamplesCfg;
        simSettings.maxNumRays = std::max(mRealtimeNumRays, mBakeNumRays);
        simSettings.numDiffuseSamples = mRealtimeDiffuseSamples;
        simSettings.maxDuration = mRealtimeDuration;
        simSettings.maxOrder = mAmbisonicsOrder;
        // Re-publish the runtime ambisonics order for the audio thread.
        // initSteamAudio publishes this too, but at that point
        // mAmbisonicsOrder is still the default (the level-load code path
        // calls setAmbisonicsOrder AFTER bootstrapFinished → initSteamAudio
        // runs, so the early publish always sees 0). The path effect is
        // created later in initVoiceDSP with maxOrder=mAmbisonicsOrder, and
        // the audio thread must stamp params.order to match — re-publish
        // here, when mAmbisonicsOrder is final and the simulator agrees.
        sPathAmbisonicsOrder.store(mAmbisonicsOrder, std::memory_order_release);
        // Reflection-sim source pool — every reverb voice (realtime or
        // baked) registers a source here, so it must be >= reverb_voices.
        // The 8 floor keeps a tiny safety margin for cap=0 edge cases.
        simSettings.maxNumSources = std::max(8, mReverbVoices);
        simSettings.numThreads = mSimulatorThreadCount;
        simSettings.samplingRate = static_cast<IPLint32>(mReflectionSampleRate);
        simSettings.frameSize = static_cast<IPLint32>(mReflectionFrameSize);

        // Spawn the reflection simulator from a temporary thread already
        // at QOS_CLASS_UTILITY so any internal worker pool Steam Audio
        // creates here inherits UTILITY at create time. macOS QoS
        // propagates from the spawning thread at the moment of thread
        // creation; if we called iplSimulatorCreate from the main thread
        // (QOS_CLASS_DEFAULT) and demoted only our wrapper later, Steam
        // Audio's internal sim workers would be stuck at DEFAULT —
        // negating the demotion done in ReflectionSimulator::workerMain.
        // The empty join below ensures the temporary thread completes
        // before we touch reflectionSimHandle. Belt-and-suspenders with
        // the wrapper-thread demotion: if Steam Audio creates the pool
        // lazily on first run() instead of eagerly here, the wrapper's
        // UTILITY still wins.
        IPLSimulator reflectionSimHandle = nullptr;
        err = IPL_STATUS_FAILURE;
        {
            std::thread creator([&]() {
#if defined(__APPLE__)
                pthread_set_qos_class_self_np(QOS_CLASS_UTILITY, 0);
                qos_class_t qos = QOS_CLASS_UNSPECIFIED;
                int relPri = 0;
                if (pthread_get_qos_class_np(pthread_self(), &qos, &relPri) == 0) {
                    AUDIO_LOG("[SIM_QOS] simulator-create thread qos=%u rel=%d\n",
                              qos, relPri);
                }
#endif
                err = iplSimulatorCreate(mIplContext, &simSettings, &reflectionSimHandle);
            });
            creator.join();
        }
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
        //
        // Pathing was previously bundled onto this simulator (DIRECT|PATHING)
        // but its iplSimulatorRunPathing iteration was observed to stall
        // 50–11000 ms on MISS6. It now lives on a separate mPathingSimulator
        // pumped by PathingSimulator's background worker — see below.
        IPLSimulationSettings directSettings{};
        directSettings.flags = IPL_SIMULATIONFLAGS_DIRECT;
        directSettings.sceneType = sceneTypeEnum;
        directSettings.maxNumOcclusionSamples = mSimMaxOcclusionSamplesCfg;
        // Direct-sim source pool — every voice (regardless of reverb
        // status) registers a direct source. Cost is ~constant per
        // source so we size to the voice cap with a 64 floor.
        directSettings.maxNumSources = std::max(64, mMaxActiveVoicesCfg);
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

        // Step 6c: Create the pathing-only simulator. Iterated by
        // PathingSimulator's background worker thread so the 50–11000 ms
        // iplSimulatorRunPathing cost stays off the main loop.
        //
        // PATHING owns the probe-graph baked path lookup that feeds the
        // per-voice eqCoeffs gain + LPF for cross-room audio. The probe
        // batch attaches to this simulator (not the direct one) — see
        // loadProbes. numVisSamples = 4 matches Steam Audio's Unity bake
        // convention (16 rays per validation test); the runtime stays
        // symmetric with the bake to avoid sentinel reads from edge-set
        // mismatches.
        {
            IPLSimulationSettings pathingSettings{};
            pathingSettings.flags = IPL_SIMULATIONFLAGS_PATHING;
            pathingSettings.sceneType = sceneTypeEnum;
            pathingSettings.maxNumSources = std::max(64, mMaxActiveVoicesCfg);
            // maxOrder MUST equal IPLPathEffectSettings::maxOrder used at
            // per-voice path-effect create time (both = mAmbisonicsOrder).
            // The pathing solver writes pathingOut.pathing.shCoeffs sized to
            // (maxOrder+1)^2; Steam Audio's internal rotate stage in
            // iplPathEffectApply then asserts the encoded SH buffer's
            // channel count matches numCoeffsForOrder(params.order). Phase
            // 4 inherited a zero-init maxOrder from the pre-path-effect
            // implementation — invisible at runtime ambisonics_order=0
            // (default trivially holds), crashed inside Steam Audio at
            // ambisonics_order>0.
            pathingSettings.maxOrder = mAmbisonicsOrder;
            // Probe-to-probe visibility sampling count for pathing edge
            // validation. Steam Audio's Unity integration defaults to 4
            // for the BAKE equivalent (`bakingVisibilitySamples`); the
            // runtime is conventionally aligned with the bake — 4 here
            // matches the shipping convention. Higher values (16 = 256
            // rays per edge via the N² rule) are 16× more expensive per
            // edge for no perceptible quality gain on static-geometry-
            // dominant scenes with a handful of dynamic doors.
            pathingSettings.numVisSamples = 4;
            // Worker thread already runs at UTILITY QoS; internal Steam
            // Audio worker threads spawned for ray-traces will inherit
            // that QoS. Match the reflection-sim choice for now.
            pathingSettings.numThreads = simSettings.numThreads;

            IPLSimulator pathingSimHandle = nullptr;
            err = iplSimulatorCreate(mIplContext, &pathingSettings, &pathingSimHandle);
            if (err != IPL_STATUS_SUCCESS) {
                LOG_ERROR("AudioService: pathing iplSimulatorCreate failed (error %d)", err);
                destroyAcousticScene();
                return false;
            }

            // Retain the scene for the pathing simulator (same pattern as
            // the direct sim above — refcount balances against the
            // iplSceneRelease in destroyAcousticScene).
            iplSceneRetain(mIplScene);
            iplSimulatorSetScene(pathingSimHandle, mIplScene);
            iplSimulatorCommit(pathingSimHandle);

            if (mPathingSim) mPathingSim->setSimulator(pathingSimHandle);
        }

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
    //
    // Hard config validation: Steam Audio crashes if
    // hybridReverbTransitionTime > irDuration. Earlier this code silently
    // clamped the transition down; that hid the misconfig and made the
    // user's actual setting irrelevant. Refuse to init the reflection
    // pipeline instead so the user fixes the YAML.
    // Steam Audio crashes if hybrid_transition_time > IR duration; keep a
    // 0.1s safety margin. Use a small epsilon to absorb float32 precision
    // noise (e.g. 2.1f - 0.1f == 1.9999999f, which would falsely reject the
    // documented 2.0/2.1 config). 1e-3f = 1 ms is large enough to absorb a
    // few ULP of subtraction error and small enough to not change the real
    // safety contract (still ~99 ms margin vs the IR duration).
    constexpr float kHybridMarginSlack = 1e-3f;
    if (mReflectionType == ReflectionType::Hybrid
        && mHybridTransitionTime > mRealtimeDuration - 0.1f + kHybridMarginSlack) {
        LOG_ERROR(
            "AudioService: reflections.hybrid_transition_time (%.2fs) >= "
            "reflections.realtime.duration (%.2fs) - 0.1s margin — Steam "
            "Audio's hybrid reverb would crash on the first apply. Reflection "
            "pipeline NOT initialized. Fix darknessRender.yaml: raise "
            "reflections.realtime.duration above hybrid_transition_time + "
            "0.1s, or lower hybrid_transition_time.",
            mHybridTransitionTime, mRealtimeDuration);
        return false;
    }

    // Hard config validation: the bake order must be >= the runtime
    // (realtime) order. Steam Audio's per-frame IPLReflectionEffectParams
    // accepts numChannels <= the effect's create-time numChannels (per
    // phonon.h IPLReflectionEffectParams::numChannels: "May be less than
    // the number of channels specified when creating the effect, in which
    // case CPU usage will be reduced"). When the bake order matches or
    // exceeds the runtime order, the runtime processes the first
    // (runtime_order+1)^2 channels of each baked IR — extra higher-order
    // baked channels are simply ignored at apply time. The reverse (bake
    // order < runtime order) leaves the runtime effect handle expecting
    // more channels than the baked IRs carry, which Steam Audio cannot
    // synthesise. Reject only that case.
    if (mBakeAmbisonicsOrder < mAmbisonicsOrder) {
        LOG_ERROR(
            "AudioService: reflections.bake.ambisonics_order (%d) < "
            "reflections.ambisonics_order (%d) — runtime would expect more "
            "ambisonic channels than the bake produced. Reflection pipeline "
            "NOT initialized. Fix darknessRender.yaml: raise "
            "reflections.bake.ambisonics_order to at least the runtime value, "
            "or lower reflections.ambisonics_order.",
            mBakeAmbisonicsOrder, mAmbisonicsOrder);
        return false;
    }

    IPLint32 irSize = static_cast<IPLint32>(mRealtimeDuration * mReflectionSampleRate);
    // Compute ambisonics channel count: (order+1)^2. Order 0=1ch, order 1=4ch.
    mAmbisonicsChannels = (mAmbisonicsOrder + 1) * (mAmbisonicsOrder + 1);
    IPLint32 numAmbiChannels = static_cast<IPLint32>(mAmbisonicsChannels);

    AUDIO_LOG( "REFL: creating ambi decoder (irSize=%d, channels=%d, rate=%d, frame=%d, 1/%d rate, type=%s)\n",
                 irSize, numAmbiChannels,
                 audioSettings.samplingRate, audioSettings.frameSize,
                 rateDivisor,
                 mReflectionType == ReflectionType::Hybrid     ? "hybrid"     :
                 mReflectionType == ReflectionType::Parametric ? "parametric" : "convolution");

    // Note: the global IPLReflectionMixer is intentionally NOT created.
    // The pre-Phase-3 design accumulated per-voice convolution into a shared
    // mixer; PLAN.HYBRID_REVERB.md Phase 3 dropped it because Steam Audio's
    // mixer overload of iplReflectionEffectApply is restricted to
    // CONVOLUTION/TAN and rejects HYBRID/PARAMETRIC. The sub-workers now
    // sum per-voice ambisonics manually, which works for all three modes.

    // Create ambisonics decode effect (ambisonics → binaural stereo via HRTF).
    // This also runs at the reflection rate — output is upsampled in the mix node.
    IPLAmbisonicsDecodeEffectSettings decodeSettings{};
    decodeSettings.speakerLayout.type = IPL_SPEAKERLAYOUTTYPE_STEREO;
    decodeSettings.hrtf = mIplHrtf;
    decodeSettings.maxOrder = mAmbisonicsOrder;

    IPLerror err = iplAmbisonicsDecodeEffectCreate(mIplContext, &audioSettings,
                                           &decodeSettings, &mIplAmbiDecodeEffect);
    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("AudioService: iplAmbisonicsDecodeEffectCreate failed (error %d)", err);
        return false;
    }

    // Create global reflection mix node in the miniaudio node graph.
    // The node operates at the engine's 48kHz rate; it internally handles
    // resampling when the reflection pipeline runs at half rate.
    mReflectionMixNode = std::make_unique<ReflectionMixNode>();
    auto& rmn = *mReflectionMixNode;
    rmn.ambiDecodeEffect = mIplAmbiDecodeEffect;
    rmn.hrtf = mIplHrtf;
    rmn.frameSize = static_cast<int>(mFrameSize);
    rmn.reflectionFrameSize = static_cast<int>(mReflectionFrameSize);
    rmn.ambiChannels = mAmbisonicsChannels;
    rmn.ambiOrder = mAmbisonicsOrder;
    // Sync-in-callback deadline: 70 % of the audio callback period. At
    // 512 frames / 48 kHz that's ~7.47 ms, leaving ~3.2 ms for the rest
    // of the mix node + master DSP chain inside one ~10.7 ms callback.
    if (mDeviceSampleRate > 0)
        rmn.syncDeadlineMs =
            0.70f * 1000.0f * static_cast<float>(mFrameSize)
                            / static_cast<float>(mDeviceSampleRate);

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

    // Sub-worker count was computed earlier in buildAcousticScene from the
    // unified reverb-thread budget split (mReverbThreadsCfg /
    // mReverbThreadsConvShareCfg). See [REVERB_THREADS] log line.
    int numWorkers = mConvolutionWorkerCount;

    // Hand off to the ConvolutionWorkerPool — it creates the pool's
    // ConvolutionWorker struct, allocates per-sub-worker Steam Audio
    // pipelines (decoder + scratch + output buffers), and spawns threads.
    // reflSettings is forwarded for backwards-compat: post-Phase-3 the pool
    // no longer creates a per-sub-worker mixer, but the field is still in
    // the Config struct in case future modes need it.
    IPLReflectionEffectSettings poolReflSettings{};
    poolReflSettings.type = (mReflectionType == ReflectionType::Hybrid)
        ? IPL_REFLECTIONEFFECTTYPE_HYBRID
        : (mReflectionType == ReflectionType::Parametric)
            ? IPL_REFLECTIONEFFECTTYPE_PARAMETRIC
            : IPL_REFLECTIONEFFECTTYPE_CONVOLUTION;
    poolReflSettings.irSize = irSize;
    poolReflSettings.numChannels = numAmbiChannels;
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
    poolCfg.reflSettings = poolReflSettings;
    // ── Rate / framing summary ─────────────────────────────────────────
    // One-shot diagnostic so we never have to fish for these numbers
    // across multiple init log lines. The four rates that matter for the
    // reflection pipeline are:
    //   • mDeviceSampleRate        — what we configured miniaudio's
    //                                engine to expect (= what Steam Audio
    //                                is told to run at). miniaudio
    //                                resamples to the actual hardware
    //                                rate transparently.
    //   • mReflectionSampleRate    — convolution rate inside the worker
    //                                (= device rate ÷ rateDivisor).
    //   • mFrameSize               — audio thread frame size (samples).
    //                                Period in ms = mFrameSize ÷
    //                                mDeviceSampleRate × 1000.
    //   • mReflectionFrameSize     — worker output frame (samples at
    //                                reflection rate).
    {
        const double devicePeriodMs = (mDeviceSampleRate > 0)
            ? (1000.0 * static_cast<double>(mFrameSize)
                       / static_cast<double>(mDeviceSampleRate))
            : 0.0;
        // Use LOG_INFO (always-on logger) rather than AUDIO_LOG so this
        // fires even when gAudioLogVerbose hasn't been set true yet — the
        // YAML `audio_log: true` toggle takes effect after this init path,
        // so AUDIO_LOG here would silently no-op.
        LOG_INFO("[RATES] device=%u Hz reflection=%u Hz "
                 "frame=%u smp (%.3f ms nominal) reflFrame=%u smp "
                 "ambiOrder=%d ambiCh=%d rateDiv=%d workers=%d",
                 mDeviceSampleRate, mReflectionSampleRate,
                 mFrameSize, devicePeriodMs,
                 mReflectionFrameSize,
                 mAmbisonicsOrder, mAmbisonicsChannels,
                 rateDivisor, numWorkers);
    }

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
                 mReverbVoices,
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
}

//------------------------------------------------------
void AudioService::destroyAcousticScene()
{
    mSceneReady = false;

    // Destroy reflection pipeline before simulator (it references simulator sources)
    destroyReflectionPipeline();

    // Drain the pathing-sim worker so we can mutate its state safely. The
    // pathing sim is the one that holds our probe batch (post-PT-13), so
    // any release path that touches the batch must wait for the worker to
    // finish its current iteration first.
    if (mPathingSim) {
        mPathingSim->waitForCompletion();
        // Pending adds never landed — release the handles directly so
        // we don't leak them with the simulator handle gone.
        mPathingSim->releasePendingAdds();
        // Pending removes still need to be applied + released so each
        // iplSourceRelease balances its iplSourceCreate.
        mPathingSim->flushPendingRemovals();
    }

    // Release probe batch before simulator (it's registered with the simulator).
    // The batch may be attached to BOTH the reflection sim (for baked reverb
    // lookups) AND the pathing sim (for Steam Audio pathing). Detach from the
    // pathing sim first, releasing the iplProbeBatchRetain we took at
    // loadProbes time, then hand off to ProbeManager which detaches from the
    // reflection sim and releases the create-time reference.
    if (mProbeManager) {
        IPLProbeBatch batch = mProbeManager->getProbeBatch();
        IPLSimulator pathingHandle = mPathingSim ? mPathingSim->simulator() : nullptr;
        if (batch && mPathingProbeBatchAdded && pathingHandle) {
            iplSimulatorRemoveProbeBatch(pathingHandle, batch);
            iplSimulatorCommit(pathingHandle);
            iplProbeBatchRelease(&batch);  // drops our retain (batch is aliased,
                                           // ProbeManager still holds the original
                                           // create-time reference)
        }
        mPathingProbeBatchAdded = false;
        if (mReflectionSim) {
            mProbeManager->releaseBatch(mReflectionSim->simulator());
        }
    }

    if (mReflectionSim && mReflectionSim->simulator()) {
        IPLSimulator handle = mReflectionSim->simulator();
        iplSimulatorRelease(&handle);
        mReflectionSim->setSimulator(nullptr);
    }
    // Pathing simulator — worker already drained above. Release the
    // handle now that the probe batch is detached. The scene we attached
    // to it was retained at create time; the iplSceneRelease below drops
    // both that retain and the original create reference.
    if (mPathingSim && mPathingSim->simulator()) {
        IPLSimulator handle = mPathingSim->simulator();
        iplSimulatorRelease(&handle);
        mPathingSim->setSimulator(nullptr);
    }
    // Direct simulator runs synchronously on the main thread, so it has
    // no in-flight worker to wait for. Release after the other sims
    // (order is irrelevant; this just keeps the parallel structure
    // visible). The scene we attached to it was retained at create time;
    // iplSceneRelease below drops both the original create-time reference
    // and the retain — refcount reaches zero and the scene is destroyed.
    if (mDirectSimulator) {
        iplSimulatorRelease(&mDirectSimulator);
        mDirectSimulator = nullptr;
    }

    // Tear down per-door dynamic meshes before the scene. Each instanced mesh
    // must be removed from the scene before its static mesh is released, or
    // Steam Audio's refcount won't reach zero. We don't commit between
    // removals — the scene is being destroyed; the final iplSceneRelease drops
    // any uncommitted state.
    if (!mDoorAudioInstances.empty()) {
        for (auto &[id, inst] : mDoorAudioInstances) {
            if (inst.instancedMesh) {
                iplInstancedMeshRemove(inst.instancedMesh, mIplScene);
                iplInstancedMeshRelease(&inst.instancedMesh);
                inst.instancedMesh = nullptr;
            }
            if (inst.staticMesh) {
                iplStaticMeshRelease(&inst.staticMesh);
                inst.staticMesh = nullptr;
            }
            if (inst.subScene) {
                iplSceneRelease(&inst.subScene);
                inst.subScene = nullptr;
            }
        }
        mDoorAudioInstances.clear();
    }
    mSceneNeedsCommit.store(false, std::memory_order_relaxed);

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

// ── Door dynamic geometry (geometry-aware door blocking) ──
//
// Each door registers a small OBB-shaped IPLStaticMesh in its model-local
// frame plus an IPLInstancedMesh that places that mesh into the world. At
// runtime, DoorSystem's audio-mesh callback pushes new transforms through
// setDoorTransform — that's enough for Steam Audio's pathing-validation
// (iplSimulatorRunPathing's ray-cast pass with `enableValidation = TRUE`) to
// reject baked path edges that pass through closed doors. Open doors carry
// the same mesh out of the doorway, so the BVH simply doesn't intersect the
// portal column anymore and the same baked edge re-validates.
//
// The mesh material is "door" from the keyword table (wood, low transmission
// across all bands) so even transmitted sound is meaningfully attenuated.
//
//------------------------------------------------------
void AudioService::registerDoorGeometry(const std::vector<DoorAudioGeometry> &doors)
{
    if (!mIplScene) {
        std::fprintf(stderr,
            "[DOOR_AUDIO] registerDoorGeometry: scene not built — skipping\n");
        return;
    }
    if (doors.empty()) {
        std::fprintf(stderr,
            "[DOOR_AUDIO] registerDoorGeometry: no doors to register\n");
        return;
    }
    std::fprintf(stderr,
        "[DOOR_AUDIO] registerDoorGeometry: registering %zu doors...\n",
        doors.size());

    // Steam Audio's docs warn that iplSceneCommit cannot run concurrently with
    // any simulation function. Static/instanced-mesh adds defer their effect
    // until commit, so the add calls themselves are safe, but the commit at
    // the end is not. Drain BOTH simulation workers before touching scene
    // state — pathing and reflection each iterate the same shared scene.
    // At boot this is a no-op (no voices yet → both sims idle), but cheap
    // insurance.
    if (mReflectionSim) mReflectionSim->waitForCompletion();
    if (mPathingSim)    mPathingSim->waitForCompletion();

    // Cache the "door" material once. Falls back to generic if the keyword
    // table is missing the entry (kept defensive — table is in this TU).
    IPLMaterial doorMaterial = lookupAcousticMaterial("door");

    int created = 0;
    int skipped = 0;
    for (const auto &g : doors) {
        if (g.localVertices.size() < 9 || g.indices.size() < 3 ||
            (g.localVertices.size() % 3) != 0 || (g.indices.size() % 3) != 0) {
            std::fprintf(stderr,
                "[FALLBACK] registerDoorGeometry: door %d has malformed mesh "
                "(verts=%zu, indices=%zu) — skipping\n",
                g.objID, g.localVertices.size(), g.indices.size());
            ++skipped;
            continue;
        }

        const size_t numVerts = g.localVertices.size() / 3;
        const size_t numTris  = g.indices.size() / 3;

        // Convert vertices from engine local-feet to IPL local-meters. Same
        // engineToIplPos used for world geometry; for a static mesh in a
        // local frame this just orients the mesh consistently with the
        // instanced-mesh transform (engineToIplMatrix below).
        std::vector<IPLVector3> iplVerts(numVerts);
        for (size_t i = 0; i < numVerts; ++i) {
            Vector3 v(g.localVertices[i * 3 + 0],
                      g.localVertices[i * 3 + 1],
                      g.localVertices[i * 3 + 2]);
            iplVerts[i] = engineToIplPos(v);
        }

        std::vector<IPLTriangle> iplTris(numTris);
        for (size_t i = 0; i < numTris; ++i) {
            iplTris[i].indices[0] = g.indices[i * 3 + 0];
            iplTris[i].indices[1] = g.indices[i * 3 + 1];
            iplTris[i].indices[2] = g.indices[i * 3 + 2];
        }

        // One material per door mesh ("door" → wood). Single-material mesh,
        // so materialIndices is all-zero (initialized by IPLint32 default).
        std::vector<IPLint32> matIdx(numTris, 0);
        IPLMaterial mats[1] = { doorMaterial };

        // Local-frame static mesh held by a 1-shot single-mesh subScene.
        // Steam Audio's IPLInstancedMesh references an entire IPLScene, not
        // an IPLStaticMesh directly — we build a tiny per-door scene that
        // contains the static mesh, then instance THAT into the world scene.
        IPLSceneSettings subSettings{};
        subSettings.type = IPL_SCENETYPE_DEFAULT;  // doors don't need embree
        IPLScene subScene = nullptr;
        if (iplSceneCreate(mIplContext, &subSettings, &subScene) != IPL_STATUS_SUCCESS) {
            std::fprintf(stderr,
                "[FALLBACK] registerDoorGeometry: iplSceneCreate(sub) failed for door %d\n",
                g.objID);
            ++skipped;
            continue;
        }

        IPLStaticMeshSettings meshSettings{};
        meshSettings.numVertices    = static_cast<IPLint32>(numVerts);
        meshSettings.numTriangles   = static_cast<IPLint32>(numTris);
        meshSettings.numMaterials   = 1;
        meshSettings.vertices       = iplVerts.data();
        meshSettings.triangles      = iplTris.data();
        meshSettings.materialIndices = matIdx.data();
        meshSettings.materials      = mats;

        IPLStaticMesh sMesh = nullptr;
        if (iplStaticMeshCreate(subScene, &meshSettings, &sMesh) != IPL_STATUS_SUCCESS) {
            std::fprintf(stderr,
                "[FALLBACK] registerDoorGeometry: iplStaticMeshCreate failed for door %d\n",
                g.objID);
            iplSceneRelease(&subScene);
            ++skipped;
            continue;
        }
        iplStaticMeshAdd(sMesh, subScene);
        iplSceneCommit(subScene);  // commit the sub-scene's own BVH

        // Instance the sub-scene into the main scene at the door's current
        // world pose. The transform converts engine Z-up feet to IPL Y-up
        // meters (see engineToIplMatrix derivation above).
        IPLInstancedMeshSettings instSettings{};
        instSettings.subScene  = subScene;
        instSettings.transform = engineToIplMatrix(g.worldTransform);

        IPLInstancedMesh iMesh = nullptr;
        if (iplInstancedMeshCreate(mIplScene, &instSettings, &iMesh) != IPL_STATUS_SUCCESS) {
            std::fprintf(stderr,
                "[FALLBACK] registerDoorGeometry: iplInstancedMeshCreate failed for door %d\n",
                g.objID);
            iplStaticMeshRelease(&sMesh);
            iplSceneRelease(&subScene);
            ++skipped;
            continue;
        }
        iplInstancedMeshAdd(iMesh, mIplScene);

        // Hold onto the sub-scene reference for the lifetime of the
        // instanced mesh. Steam Audio's docs do not guarantee that
        // iplInstancedMeshCreate retains the sub-scene, and an early
        // iplSceneRelease has manifested as a boot-time hang on some
        // builds (BVH walks following a dangling pointer). We release it
        // in destroyAcousticScene instead.
        DoorAudioInstance inst;
        inst.subScene      = subScene;
        inst.staticMesh    = sMesh;
        inst.instancedMesh = iMesh;
        mDoorAudioInstances[g.objID] = inst;
        ++created;
    }

    // Single commit on the main scene rebuilds its BVH once for all the new
    // instances. Cheaper than committing per-add.
    std::fprintf(stderr,
        "[DOOR_AUDIO] registerDoorGeometry: committing scene "
        "(created=%d, skipped=%d)...\n", created, skipped);
    iplSceneCommit(mIplScene);

    std::fprintf(stderr,
        "[DOOR_AUDIO] registerDoorGeometry: done — %d instanced meshes "
        "(of %zu doors)\n", created, doors.size());
}

//------------------------------------------------------
void AudioService::setDoorTransform(int32_t doorObjID, const Matrix4 &worldTransform)
{
    auto it = mDoorAudioInstances.find(doorObjID);
    if (it == mDoorAudioInstances.end() || !it->second.instancedMesh || !mIplScene) {
        // Door not registered for audio (e.g. zero edgeLengths) — silently
        // no-op. The DoorSystem callback fires for every animating door
        // regardless, so this path is hit per-frame for un-registered doors.
        return;
    }
    IPLMatrix4x4 ipl = engineToIplMatrix(worldTransform);
    iplInstancedMeshUpdateTransform(it->second.instancedMesh, mIplScene, ipl);
    // Coalesce into one iplSceneCommit per loopStep (see loopStep header).
    mSceneNeedsCommit.store(true, std::memory_order_release);

    // Diagnostic: log the first few transform pushes per door so we can
    // verify the animation callback is firing. Rate-limited to keep the
    // log readable while doors animate over many frames.
    {
        static std::unordered_map<int32_t, int> sCountByDoor;
        int &c = sCountByDoor[doorObjID];
        if (c < 4 || (c % 32) == 0) {
            std::fprintf(stderr,
                "[DOOR_AUDIO] setDoorTransform obj=%d trans=(%.2f,%.2f,%.2f) "
                "(call #%d)\n",
                doorObjID,
                worldTransform[3][0], worldTransform[3][1], worldTransform[3][2],
                c + 1);
        }
        ++c;
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
    // Start the pathing simulation worker thread. iplSimulatorRunPathing
    // has been observed at 11+ seconds on MISS6 when dynamic door OBBs
    // invalidate baked path edges and findAlternatePaths explores its
    // search space — keeping it on the main thread froze the render loop.
    if (mAudioReady && mPathingSim) {
        mPathingSim->start();
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

    // Shut down both simulation worker threads before destroying the scene.
    // Order between them doesn't matter (they don't share state); the only
    // requirement is that both stop BEFORE destroyAcousticScene tears down
    // the simulator handles + scene.
    if (mReflectionSim) mReflectionSim->stop();
    if (mPathingSim)    mPathingSim->stop();

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

    // Steam Audio pathing-simulation throttle. iplSimulatorRunPathing runs
    // synchronously on this thread and is CPU-heavy; running it every loopStep
    // can stall the audio callback (the Unity/Unreal integrations expose an
    // equivalent "Simulation Update Interval" knob for the same reason).
    // Accumulate deltaTime and only mark the pathing step "due" once the
    // configured interval elapses. mPathingDueThisStep is read by the
    // per-voice pathing-input staging loop and by the iplSimulatorRunPathing
    // invocation further below. mPathingUpdateInterval == 0 means update every
    // frame (legacy behaviour; preserved as an A-B diagnostic).
    //
    // This previously lived in updateAudio(), which was the AudioService's
    // own per-frame entrypoint. updateAudio() was decommissioned when the
    // service moved to LoopService-driven loopStep() (DarknessRender.cpp
    // comment "the manual audioSvc->updateAudio(dt) call is removed"), but
    // the throttle update was left behind in the old function — so
    // mPathingDueThisStep stayed false forever, iplSimulatorRunPathing
    // never ran, and every source's pathing eqCoeffs stayed at Steam
    // Audio's create-time default of 0.1f (per simulation_data.cpp:120-124).
    if (mPathingUpdateInterval <= 0.0f) {
        mPathingDueThisStep = true;
        mPathingAccumSec    = 0.0f;
    } else {
        mPathingAccumSec += deltaTime;
        mPathingDueThisStep = (mPathingAccumSec >= mPathingUpdateInterval);
        if (mPathingDueThisStep) mPathingAccumSec = 0.0f;
    }

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

    // Source mutations (add/remove/commit) can race with the background
    // reflection AND pathing sim threads, so we defer them until BOTH are
    // idle. Direct sim runs synchronously on the main thread, so it's never
    // concurrent with mutations. Steam Audio uses double-buffering:
    // setInputs writes to the staging buffer, while runReflections /
    // runPathing read from the committed (active) buffer. Only commit()
    // copies staging → active, so commit must wait for both workers.
    bool reflBusy  = mReflectionSim && mReflectionSim->isRunning();
    bool pathBusy  = mPathingSim    && mPathingSim->isRunning();
    bool canMutate = !reflBusy && !pathBusy;
    IPLSimulator reflectionSimHandle = mReflectionSim ? mReflectionSim->simulator() : nullptr;
    IPLSimulator pathingSimHandle    = mPathingSim    ? mPathingSim->simulator()    : nullptr;

    if (canMutate && mReflectionSim) {

        // Flush deferred IPL source adds / removals via ReflectionSimulator.
        mReflectionSim->flushPendingAdds();
        mReflectionSim->flushPendingRemovals();
    }
    if (canMutate && mPathingSim) {
        // Same defer-flush dance for the pathing simulator's pending
        // source-add/remove queues.
        mPathingSim->flushPendingAdds();
        mPathingSim->flushPendingRemovals();
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
    // Pathing-sim commit. Same constraint as the reflection sim: cannot
    // overlap with iplSimulatorRunPathing on the worker thread.
    if (canMutate && mPathingSim && mPathingSim->isSimulatorDirty() && pathingSimHandle) {
        mPathingSim->commitIfDirty();
    }

    // Coalesced acoustic-scene commit for door dynamic geometry.
    // setDoorTransform queues iplInstancedMeshUpdateTransform calls and flips
    // mSceneNeedsCommit. iplSceneCommit (BVH refit) cannot run concurrently
    // with any simulation function per Steam Audio's API contract — so we
    // gate on `canMutate` (same flag the reflection-sim source-mutation
    // commit above uses): if the reflection sim is currently running we
    // skip and try next frame, rather than block on waitForCompletion().
    // Door transforms are sticky in mSceneNeedsCommit, so postponing a
    // frame just delays validation by ~16 ms — imperceptible. Pathing and
    // direct sims run synchronously later in this same loopStep, so they
    // always observe a committed scene.
    if (mIplScene && mSceneNeedsCommit.load(std::memory_order_acquire)) {
        if (canMutate) {
            auto c0 = std::chrono::steady_clock::now();
            iplSceneCommit(mIplScene);
            auto c1 = std::chrono::steady_clock::now();
            mSceneNeedsCommit.store(false, std::memory_order_release);

            static std::atomic<int> sCommitCount{0};
            int n = sCommitCount.fetch_add(1, std::memory_order_relaxed);
            if (n < 8 || (n % 32) == 0) {
                float ms = std::chrono::duration<float, std::milli>(c1 - c0).count();
                std::fprintf(stderr,
                    "[DOOR_AUDIO] iplSceneCommit (door BVH refit) took "
                    "%.2f ms (call #%d)\n", ms, n + 1);
            }
        } else {
            // Refl-sim busy. Track how long the door commit has been
            // deferred so we can spot the pathological case where it
            // never commits (refl-sim always running). Logs every 64
            // deferred frames.
            static std::atomic<int> sCommitDeferred{0};
            int d = sCommitDeferred.fetch_add(1, std::memory_order_relaxed);
            if ((d % 64) == 0) {
                std::fprintf(stderr,
                    "[DOOR_AUDIO] scene commit deferred — reflection sim "
                    "busy (deferred #%d times so far)\n", d + 1);
            }
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

        // Phase 4: publish the listener coord for the audio thread's
        // per-voice iplPathEffectApply. Loose-sync — single writer
        // (main thread, here), multiple readers (audio thread, once
        // per voice per callback). The struct is small (12 floats);
        // a torn read at most produces one slightly-off binaural
        // sample window for one voice for one callback.
        sPathListenerCoord = listenerCoord;

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
            sharedInputs.irradianceMinDistance = kIrradianceMinDistanceMeters;
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
            // Pathing also needs the listener pose per-frame. Without
            // this call, iplSimulatorRunPathing has no listener position,
            // silently skips all sources, and iplSourceGetOutputs returns
            // the source's SimulationData::pathingOutputs.eq init value
            // of [0.1, 0.1, 0.1] (see Steam Audio simulation_data.cpp).
            // Now lives on the separate pathing simulator handle owned
            // by mPathingSim — both flags-on-this-simulator (here:
            // PATHING) require their per-frame SetSharedInputs call
            // even though only one is set.
            if (pathingSimHandle) {
                iplSimulatorSetSharedInputs(pathingSimHandle,
                    IPL_SIMULATIONFLAGS_PATHING, &sharedInputs);
            }

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
            // ── Sticky reflection-slot allocation ──
            //
            // Replaces the pre-2026-05 per-frame top-N partial_sort + demote
            // hysteresis pass. The old design re-ranked every voice by distSq
            // every frame and let voices flip in/out of top-N as small
            // distance changes shuffled the ranking; each flip changed a
            // voice's IR mode (baked-probe lookup ↔ real-time ray-traced),
            // producing the low-frequency wet-bus wobble that the hybrid-
            // reverb migration was supposed to kill.
            //
            // The new contract:
            //   1. Each eligible voice's realtime-vs-baked decision is
            //      made ONCE, on its first loopStep with valid reflection
            //      outputs (isReflectionPending=false), and retained for
            //      the voice's entire lifetime including the reverb tail.
            //   2. Slots are released only when sourceEnded + tailTimer
            //      expired. No mid-playback eviction.
            //   3. If all slots are full when a new eligible voice spawns,
            //      the new voice goes baked-only for life. Accepted trade-
            //      off vs the per-frame mode-flip artefact.
            //
            // Eligibility for realtime:
            //   - !playerEmitted        (player ≈ listener, baked is correct)
            //   - !isAmbient            (long-lived, would hog slots forever)
            //   - has a reflectionSource AND effectsReady AND reflectionEffect
            //
            // PlayerEmitted + Ambient voices fall through to the baked-probe
            // path below (inputs.baked = IPL_TRUE).
            if (mReflectionsEnabled && reflectionSimHandle && mReflectionSim) {
                // Step 1: count currently-owned slots, including tail voices.
                int slotsHeld = 0;
                for (auto &[h, v] : mVoicePool->voices()) {
                    if (v->reflSlotOwned) ++slotsHeld;
                }
                // Realtime sticky-slot pool size = `mReverbVoicesRealtime`, which
                // defaults to mReverbVoices but can be set lower (or 0)
                // to reserve total-convolution budget for baked voices only.
                int slotsAvailable = std::max(0, mReverbVoicesRealtime - slotsHeld);

                // Step 2: collect undecided candidates eligible for promotion.
                std::vector<VoiceDist> newCandidates;
                newCandidates.reserve(mVoicePool->size());
                for (auto &[h, v] : mVoicePool->voices()) {
                    if (v->reflSlotDecided) continue;             // already decided
                    if (v->sourceEnded)     continue;             // ended before decision
                    if (v->playerEmitted)   continue;             // always baked
                    if (v->isAmbient)       continue;             // always baked
                    if (!v->reflectionSource)                continue;
                    if (!v->dspNode.effectsReady)            continue;
                    if (!v->dspNode.reflectionEffect)        continue;
                    // Pending sim-add → wait until the sim has the source
                    // before committing. Avoids deciding before the voice
                    // has any IR to render.
                    if (mReflectionSim->isAddPending(v->reflectionSource)) continue;
                    Vector3 delta = v->worldPos - mListenerPos;
                    newCandidates.push_back({h, glm::dot(delta, delta)});
                }

                // Step 3: when multiple voices are eligible in the same
                // frame and slots are scarce, the closest get priority.
                // After this, the decision sticks regardless of later
                // distance changes — by design.
                if (!newCandidates.empty()) {
                    std::sort(newCandidates.begin(), newCandidates.end(),
                              [](const VoiceDist &a, const VoiceDist &b) {
                                  return a.distSq < b.distSq;
                              });
                    for (auto &cand : newCandidates) {
                        ActiveVoice *v = mVoicePool->find(cand.handle);
                        if (!v) continue;
                        if (slotsAvailable > 0) {
                            v->reflSlotOwned   = true;
                            v->reflSlotDecided = true;
                            --slotsAvailable;
                        } else {
                            v->reflSlotOwned   = false;
                            v->reflSlotDecided = true;
                        }
                    }
                }

                // Step 4: release slots from voices whose source has ended
                // AND whose reverb tail has expired. tailTimer is ticked
                // down in a later loop further below in loopStep; we look
                // at sourceEnded + (tailTimer <= 0) which is exactly the
                // "audibly silent, safe to demote" condition. demoteVoice
                // releases the IPLSource (frees a slot in the sim too).
                for (auto &[h, v] : mVoicePool->voices()) {
                    if (!v->reflSlotOwned) continue;
                    if (!v->sourceEnded)   continue;          // still playing
                    if (v->tailTimer > 0.0f) continue;        // tail still ringing
                    v->reflSlotOwned = false;
                    if (v->reflectionSource) mReflectionSim->demoteVoice(*v);
                }

                // Step 5: build the set used downstream for inputs.baked /
                // enableRefl decisions. Membership == "owns a realtime
                // slot right now."
                for (auto &[h, v] : mVoicePool->voices()) {
                    if (v->reflSlotOwned) reflCandidateSet.insert(h);
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

                // Player-audio portal-eq backend toggle. When true, Steam
                // Audio's pathing simulator drives portalAttenuation /
                // portalBlocking via 3-band eqCoeffs. When false (legacy
                // mode), the room-BFS branch below fills the same fields
                // from effectiveDistance/realDistance. The non-pathing
                // Steam Audio DSP chain (direct effect, HRTF, reflections,
                // air absorption, occlusion/transmission) runs in both
                // modes — this gate is only the portal-eq source. AI
                // hearing is unaffected: AIHearingService calls
                // RoomService::propagateSoundPath directly, independent
                // of this gate. Player-emitted voices (footsteps, landings)
                // sit in the same room as the listener by definition, so
                // both backends short-circuit on the playerEmitted flag.
                bool useSteamAudioPathing = mPathingProbeBatchAdded
                                          && mProbePathingEnabled
                                          && !voice->playerEmitted
                                          && !voice->skipPortalRouting;

                // Legacy BFS path — only reachable when probe_pathing: false.
                // Steam Audio is the shipping default routing authority for
                // player audio; this branch exists for A/B comparison and
                // may be removed in a future revision. The room-BFS fills
                // portalAttenuation / portalBlocking via the prop.reached
                // branches below; gated off when Steam Audio pathing owns
                // those fields so the two backends don't compete to write
                // the same dspNode fields.
                SoundPropInfo prop{};
                bool isCrossRoom = false;
                if (!useSteamAudioPathing && mPortalRoutingEnabled
                    && !voice->sourceEnded
                    && !voice->skipPortalRouting && !voice->playerEmitted) {
                    // BFS every frame (~3 µs/voice). The cap is decoupled
                    // from per-voice audibility radius: maxDist inside
                    // propagateSoundPath is both the BFS depth bound AND
                    // the door-blocking saturation target, so tying them
                    // together made small-radius voices unreachable after
                    // one or two portal hops.
                    auto prT0 = profOn ? std::chrono::steady_clock::now()
                                       : std::chrono::steady_clock::time_point{};
                    if (mRoomService && mSoundPropagation) {
                        Room *srcRoomP = mRoomService->roomFromPoint(voice->worldPos);
                        Room *lstRoomP = mRoomService->roomFromPoint(mListenerPos);
                        int32_t srcID = srcRoomP ? srcRoomP->getRoomID() : -1;
                        int32_t lstID = lstRoomP ? lstRoomP->getRoomID() : -1;
                        // Inline params so we can plug chainOut in (used
                        // by the show_vpos debug overlay; the
                        // propagateSound wrapper doesn't expose it).
                        SoundPropParams pp;
                        pp.maxDist     = std::max(voice->maxAudibleDist, mPropagationMaxDist);
                        pp.maxPaths    = mPropMaxPaths;
                        pp.maxPathDiff = mPropMaxPathDiff;
                        pp.chainOut    = &voice->cachedChain;
                        prop = mSoundPropagation->propagateSoundWithParams(
                            voice->worldPos, mListenerPos, srcID, lstID, pp);
                    } else {
                        prop = propagateSound(voice->worldPos, mListenerPos,
                                              std::max(voice->maxAudibleDist, mPropagationMaxDist));
                    }
                    if (profOn) {
                        auto prT1 = std::chrono::steady_clock::now();
                        float prUs = std::chrono::duration<float, std::micro>(prT1 - prT0).count();
                        sPortalRoutingTotalUs.fetch_add(static_cast<int>(prUs), std::memory_order_relaxed);
                        sPortalRoutingCount.fetch_add(1, std::memory_order_relaxed);
                    }

                    voice->cachedProp = prop;

                    // Cross-room flag tracks actual room assignment — no
                    // distance gate. The playerEmitted skip above handles
                    // head/feet straddling for player-sourced voices.
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
                // Phase 4 — Steam Audio sole authority for player audio
                // routing + attenuation. The legacy BFS-driven branches
                // (prop.reached / cross-room failure / door-blocking
                // doorBlocking → portalBlocking → LPF cutoff) no longer
                // feed the player DSP: those scalars are now driven by
                // Steam Audio's path effect (geometry-aware
                // diffraction + door OBB validation in the per-frame
                // pathing solver, applied on the audio thread via
                // iplPathEffectApply).
                //
                // The DSP-side scalars portalAttenuation / portalBlocking
                // are kept at neutral 1.0 / 0.0 so the wet-bus
                // reflection-send carries unattenuated reverb (the
                // reflection IR already encodes the room's acoustic
                // character). Distance attenuation is owned by Steam
                // Audio's IPLDirectEffect via the direct simulator's
                // INVERSEDISTANCE model. HRTF direction is driven by
                // the standard !usePortalRouting branch downstream
                // (toSource = voice->worldPos − mListenerPos), pointing
                // at the real source position.
                voice->dspNode.usePortalRouting  = false;
                voice->dspNode.portalAttenuation = 1.0f;
                voice->dspNode.portalBlocking    = 0.0f;

                // maxAudibleDist hard cutoff — same authored value that
                // gates AI hearing's BFS, so a wind ambient with
                // radius=25 still goes silent at 25 ft of straight-line
                // distance. Steam Audio's distance attenuation would
                // taper to ~0 at this range anyway, but the schema-
                // authored radius is treated as an absolute boundary in
                // the original engine. Implementation: invalidate the
                // path-effect target (audio thread skips the path
                // effect) AND silence the dry bus via portalAttenuation
                // = 0.0. The wet-bus reflection-send tracks
                // portalAttenuation, so reverb also silences past the
                // boundary.
                {
                    float lineDistTo = glm::length(voice->worldPos - mListenerPos);
                    if (lineDistTo > voice->maxAudibleDist) {
                        voice->dspNode.portalAttenuation = 0.0f;
                    }
                }
                // The BFS `prop` and `isCrossRoom` are still populated
                // above (when probe_pathing is off) — they drive the
                // show_vpos diagnostic overlay (cachedProp), the [VOICE]
                // periodic log, and the legacy sub-source slot fan-out
                // below. They have zero effect on the player DSP signal
                // path. AI hearing reads
                // RoomService::propagateSoundPath directly
                // (AIHearingService::onSoundEmitted), unaffected by this
                // branch.

                // ── Sub-source slot fan-out ──
                //
                // Steam Audio mode (`probe_pathing: true`, shipping default):
                // Steam Audio's pathing solver folds multi-path propagation
                // into a single eqCoeffs[3] + combined ambisonic SH field,
                // not into N independent (direction, gain) tuples. The
                // multi-path-multi-portal data model the original Phase-2
                // design assumed cannot be reconstructed from SA's output.
                // We force slot-0-only: a synthesised single path at
                // voice->worldPos, with slots 1-3 drained. Steam Audio's
                // eqCoeffs (via portalAttenuation / portalBlocking) carries
                // the cross-room attenuation work upstream of this slot;
                // same-room voices end up at eqCoeffs ≈ 1.
                //
                // Legacy BFS mode (`probe_pathing: false`): the BFS branch
                // above populated prop.paths with up to maxPaths discrete
                // routes; multi-portal scenes get true multi-direction
                // HRTF fan-out via updateSubSourceSlots.
                //
                // Player-emitted + skipPortalRouting voices stay slot-0-only
                // in both modes — source ≈ listener, portal routing N/A.
                //
                // See PLAN.MULTI_PATH_SA_MIGRATION.md for the rationale.
                // PLAN.MULTI_PATH_AMBISONICS.md (the original Phase-1..4
                // design) is superseded by this for the SA-mode regime;
                // Phases 2-4 remain valid for the legacy BFS mode.
                auto computeDirForPath = [&](const SoundPathRecord& p) -> IPLVector3 {
                    Vector3 toPath = p.virtualPosition - mListenerPos;
                    float   dist   = glm::length(toPath);
                    if (dist < kDistanceEpsilonFt) {
                        return IPLVector3{0.0f, 0.0f, -1.0f};  // ahead fallback
                    }
                    toPath /= dist;
                    return IPLVector3{
                        glm::dot(toPath, right),
                        glm::dot(toPath, up),
                        -glm::dot(toPath, ahead)
                    };
                };

                // Synthesised single-path record at the source's actual
                // world position. Used for the slot-0-only path
                // (SA-mode, player-emitted, skipPortalRouting, and the
                // legacy "no room data" fallback). Reuse here to avoid
                // duplicating the construction across branches.
                auto makeSyntheticSinglePath = [&]() -> SoundPropInfo {
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
                    return synth;
                };

                if (useSteamAudioPathing
                    || voice->playerEmitted
                    || voice->skipPortalRouting) {
                    // Slot-0-only. maxN=1 causes updateSubSourceSlots to
                    // drain slots 1-3 (if any are ACTIVE from a prior
                    // BFS-mode session, or from a runtime toggle) and
                    // keep slot 0 ACTIVE with the synthesised path.
                    SoundPropInfo synth = makeSyntheticSinglePath();
                    updateSubSourceSlots(voice->dspNode.subSources, synth,
                                         /*maxN=*/1, computeDirForPath);
                } else {
                    // Legacy BFS mode: BFS produced prop.paths (or it
                    // didn't reach). Use the configured sMaxSubSources
                    // clamp.
                    const int maxN = static_cast<int>(std::min<uint32_t>(
                        sMaxSubSources.load(std::memory_order_relaxed),
                        static_cast<uint32_t>(kMaxSubSources)));
                    if (prop.reached && !prop.paths.empty()) {
                        updateSubSourceSlots(voice->dspNode.subSources, prop,
                                             maxN, computeDirForPath);
                    } else if (isCrossRoom) {
                        // BFS-failed cross-room: legitimately silenced.
                        drainAllSubSourceSlots(voice->dspNode.subSources);
                    } else {
                        // BFS didn't run (no room data, etc.) — same
                        // synthesised single-path fallback as SA mode.
                        SoundPropInfo synth = makeSyntheticSinglePath();
                        updateSubSourceSlots(voice->dspNode.subSources, synth,
                                             maxN, computeDirForPath);
                    }
                }

                // Phase 4 — Steam Audio sole authority: pin the IPL source
                // position to the REAL source position for EVERY voice. The
                // legacy cross-room branch that pinned to
                // `prop.virtualPosition` (the last portal anchor along the
                // BFS chain) is gone. With IPLPathEffect now driving
                // cross-room routing (occlusion through walls via
                // enableValidation + findAlternatePaths against the door
                // OBBs), the direct simulator wants the real geometry
                // between source and listener — anchoring at a doorway
                // turned an occluded source into an unoccluded one (the
                // pathing solver did the diffraction; the direct sim
                // independently saw clean line-of-sight from the anchor
                // and skipped the wall transmission).
                //
                // Engine→IPL conversion (feet→meters AND Z-up→Y-up) happens
                // via engineToIplPos so the source sits in the same frame
                // as the listener and mesh.
                IPLCoordinateSpace3 sourceCoord{};
                sourceCoord.origin = engineToIplPos(voice->worldPos);
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
                // Steam Audio distance model: always INVERSEDISTANCE with
                // minDistance scaled by the schema's P$SchAttFac
                // (attenuationFactor). attenuationFactor=1 → minDistance=1m
                // (matches DEFAULT model behaviour); attenuationFactor=20
                // → minDistance=20m, keeping a sound at full volume out to
                // 20m before 1/d falloff. Per-voice; default factor 1.0 set
                // at voice creation in VoicePool, schemas with non-default
                // P$SchAttFac override via AmbientSoundManager → voiceSetAttenuationFactor.
                inputs.distanceAttenuationModel.type =
                    IPL_DISTANCEATTENUATIONTYPE_INVERSEDISTANCE;
                inputs.distanceAttenuationModel.minDistance =
                    1.0f * voice->attenuationFactor;
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

                // Hybrid/parametric tuning, always populated. reverbScale[]
                // defaults to {0,0,0} on zero-init — which silences the
                // parametric tail entirely. Set to {1,1,1} so Steam Audio
                // uses the simulated RT60 unmodified across all three bands.
                // The two hybrid fields are only consulted when the effect
                // type is HYBRID (Steam Audio ignores them otherwise), so
                // it's safe to set unconditionally. Plan target:
                // transitionTime=2.0 s, overlap=0.25 → 0..1.5 s pure
                // convolution, 1.5..2.0 s blend, 2.0+ s pure parametric.
                inputs.reverbScale[0] = 1.0f;
                inputs.reverbScale[1] = 1.0f;
                inputs.reverbScale[2] = 1.0f;
                inputs.hybridReverbTransitionTime = mHybridTransitionTime;
                inputs.hybridReverbOverlapPercent = mHybridOverlapPercent;

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
                // Throttled: pathing input staging is skipped on frames
                // when mPathingDueThisStep is false. The matching
                // iplSourceSetInputs(...PATHING) call and the
                // iplSimulatorRunPathing call below are gated on the
                // same flag, so skipping the stamping here costs
                // nothing (Steam Audio reads pathing fields only on
                // the PATHING setInputs path).
                // Per-voice audibility gate. Voices outside their
                // schema-authored maxAudibleDist are inaudible regardless
                // of pathing — skip the per-source staging (and the
                // corresponding cost in iplSimulatorRunPathing) for them.
                //
                // We do NOT short-circuit on "same room" here. The room
                // graph is too coarse-grained for that to be a reliable
                // proxy for "no portal between source and listener" —
                // many open spaces (halls, courtyards, outdoor areas)
                // span multiple Room cells but should sonically behave
                // as a single space, and the previous same-room hack
                // would incorrectly skip pathing for voices across those
                // cells. Let Steam Audio's solver decide; for true
                // unobstructed paths it returns eqCoeffs ≈ 1.0 and the
                // mapping naturally produces full passthrough.
                float voicePathingDist = glm::length(voice->worldPos - mListenerPos);
                bool  pathingWanted    = (voicePathingDist <= voice->maxAudibleDist);
                if (mPathingProbeBatchAdded && mProbePathingEnabled
                    && mPathingDueThisStep
                    && !voice->playerEmitted
                    && pathingWanted) {
                    inputs.pathingProbes      = mProbeManager->getProbeBatch();
                    // visRadius and visThreshold are derived from the
                    // shared pathing helpers in SteamAudioPathing.h so
                    // they stay locked to the bake-time parameters
                    // (ProbeManager.cpp). A runtime value below the bake
                    // radius causes iplSimulatorRunPathing to fail
                    // finding an entry probe for sources placed
                    // off-probe, returning eqCoeffs=[0,0,0] — audible as
                    // per-callback choppy gaps in the reflection wet bus.
                    inputs.visRadius          = pathingVisRadiusMeters(
                        mProbeManager->getProbeSpacingFt());
                    inputs.visThreshold       = kPathingVisThreshold;
                    // visRange = maximum probe-to-probe distance the
                    // solver will consider for a single edge. MUST equal
                    // the bake-time value or the runtime considers edges
                    // that don't exist in the graph (too high) /
                    // silently rejects baked edges (too low).
                    //
                    // Matches Steam Audio's Unity convention of allowing
                    // the solver to walk the full propagation graph;
                    // per-frame cost is controlled via `numVisSamples`
                    // and `visThreshold`, NOT by truncating the graph.
                    inputs.visRange = mPropagationMaxDist * kFeetToMeters;
                    // pathingOrder MUST equal the path effect's create-time
                    // maxOrder (= mAmbisonicsOrder). The solver writes
                    // pathingOut.pathing.shCoeffs sized to (pathingOrder+1)^2
                    // and pathingOut.pathing.order = pathingOrder. The path
                    // effect's internal ambisonics-rotate stage asserts
                    // `in.numChannels() == numCoeffsForOrder(params.order)`,
                    // so the per-call params.order must match the channel
                    // count of the encoded SH buffer. Phase 4 inherited a
                    // legacy pathingOrder=0 from the pre-path-effect
                    // implementation (where pathing only drove eqCoeffs +
                    // scalar collapse) — at runtime ambisonics_order > 0
                    // that mismatch crashed inside iplPathEffectApply.
                    inputs.pathingOrder       = mAmbisonicsOrder;
                    inputs.enableValidation   = IPL_TRUE;
                    // findAlternatePaths ON.
                    //
                    // Required for correctness with dynamic door OBBs in
                    // the acoustic scene. enableValidation alone discards
                    // any baked path whose edge ray hits a door OBB; with
                    // findAlternatePaths off, no replacement is sought and
                    // the source's pathingState.eq stays at the 0.1f
                    // sentinel — observed in practice as nearby sources
                    // (24–37 ft, sProbe/lProbe well inside visRadius)
                    // returning persistent sentinels because their
                    // shortest baked path happens to skirt a door OBB
                    // somewhere along the way.
                    //
                    // Cost was the reason this was OFF earlier (boot-time
                    // main-thread freeze). Two changes reduce it to
                    // acceptable now:
                    //   • numVisSamples dropped from 16 to 4 in the
                    //     direct-sim settings — 16× fewer rays per
                    //     candidate edge during alternate-path search.
                    //   • numVisSamples = 4 matches Steam Audio's
                    //     Unity bake convention, so runtime and bake
                    //     stay aligned.
                    //
                    // If [PATHING_SLOW] returns and blocks the main
                    // thread, the next move is PLAN.PATHING_WORKER.md —
                    // run the solver on a background thread so any
                    // per-call cost becomes acceptable.
                    inputs.findAlternatePaths = IPL_TRUE;
                }

                // Push inputs to each simulator. Every voice has a
                // directSource (in mDirectSimulator); non-player-emitted
                // voices also have a pathingSource (in the pathing sim)
                // and reflection-eligible voices have a reflectionSource
                // (in the reflection sim). Per phonon.h, no
                // synchronisation is required between these calls — Steam
                // Audio internally separates per-source staging buffers,
                // and the staging-vs-active double buffer is safe to
                // write from the main thread even while a worker is
                // iterating (validated against Unity + Unreal reference
                // integrations).
                iplSourceSetInputs(voice->directSource,
                    IPL_SIMULATIONFLAGS_DIRECT, &inputs);
                // Pathing inputs go to the per-voice pathingSource (in
                // the pathing simulator). Throttled by mPathingDueThisStep
                // — we only stamp fresh inputs when the throttle has
                // elapsed and the worker is about to be signalled. No
                // !isRunning() gate: per N2 (RM.PLAN.PATHING_WORKER.md),
                // iplSourceSetInputs is per-frame-safe via Steam Audio's
                // internal double buffering, and both reference
                // integrations call it every frame while the sim thread
                // is mid-iteration.
                //
                // The gate here must match the populate block above
                // EXACTLY. Letting a voice through here without going
                // through the populate path means Steam Audio reads
                // `inputs.pathingProbes` as the default-initialised null
                // pointer and segfaults inside the next runPathing.
                if (mPathingProbeBatchAdded && mProbePathingEnabled
                    && mPathingDueThisStep
                    && !voice->playerEmitted
                    && pathingWanted
                    && voice->pathingSource) {
                    iplSourceSetInputs(voice->pathingSource,
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
            // The reflection pipeline is considered ready when both the
            // ambisonics decoder and the convolution worker pool are alive
            // (post-Phase-3 the global IPLReflectionMixer is gone — the
            // worker pool sums ambisonics manually).
            bool wantReflections = mReflectionsEnabled
                && mIplAmbiDecodeEffect && mConvolutionPool && mConvolutionPool->isActive()
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

            // Signal the pathing-sim worker thread to run one
            // iplSimulatorRunPathing iteration. Cheap when no probe batch
            // is attached (mPathingProbeBatchAdded = false) — Steam Audio
            // skips per-source pathing work since pathingProbes is null
            // for every source. Bypassed entirely when mProbePathingEnabled
            // is off (audio.propagation.probe_pathing: false).
            //
            // Throttled by mPathingDueThisStep (audio.propagation.
            // pathing_update_interval, default 0.1 s / 10 Hz). On skipped
            // frames the previous run's eqCoeffs remain cached on each
            // source, so the output read further down still produces
            // stable portalAttenuation/portalBlocking values.
            //
            // The actual iplSimulatorRunPathing call (and its
            // [PATHING_SLOW] timing log) now lives in
            // PathingSimulator::workerMain — keeping it off the main
            // thread eliminates the 50–11000 ms loop-stall hitches
            // observed on MISS6 with dynamic door geometry.
            {
                const bool gateProbeBatch = mPathingProbeBatchAdded;
                const bool gateEnable     = mProbePathingEnabled;
                const bool gateDue        = mPathingDueThisStep;
                const bool wantPathing    = gateProbeBatch && gateEnable && gateDue;
                if (wantPathing && mPathingSim) {
                    // PT-18 [PATHING_LAG] diagnostic. If we want to signal
                    // a fresh iteration but the worker is still running
                    // the previous one, the throttle interval is shorter
                    // than the worker's actual iteration cost — eq-coeffs
                    // will be stale for one extra throttle window.
                    // Rate-limit (first 16 + every 64th thereafter).
                    if (mPathingSim->isRunning()) {
                        static std::atomic<int> sPathingLagCount{0};
                        int lc = sPathingLagCount.fetch_add(1, std::memory_order_relaxed);
                        if (lc < 16 || (lc % 64) == 0) {
                            std::fprintf(stderr,
                                "[PATHING_LAG] worker still running when "
                                "throttle elapsed (interval=%.3fs, "
                                "occurrence #%d) — eqCoeffs will be one "
                                "interval stale this cycle\n",
                                mPathingUpdateInterval, lc + 1);
                        }
                    }
                    mPathingSim->signal();
                }
                // DIAG: confirm iplSimulatorRunPathing is actually
                // firing. eqCoeffs reading back as the Steam Audio
                // default 0.1f (per simulation_data.cpp:120-124, source
                // pathingState.eq[i] is initialized to 0.1f at source
                // create time) means the solver never wrote new values
                // — either runPathing isn't being called, or it's
                // being called but the source isn't in its solve set.
                static std::atomic<int> sRunPathingLogCount{0};
                int n = sRunPathingLogCount.fetch_add(1, std::memory_order_relaxed);
                if (n < 8) {
                    AUDIO_LOG("[RUN_PATHING] called=%d probeBatchAdded=%d "
                              "enabled=%d due=%d updateInterval=%.3f "
                              "accumSec=%.3f (occurrence #%d)\n",
                              wantPathing ? 1 : 0,
                              gateProbeBatch ? 1 : 0,
                              gateEnable ? 1 : 0,
                              gateDue ? 1 : 0,
                              mPathingUpdateInterval, mPathingAccumSec,
                              n + 1);
                }
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
        // closest first, then baked voices fill remaining up to mReverbVoices.
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
            bool useSteamAudioPathing = mPathingProbeBatchAdded
                                      && mProbePathingEnabled
                                      && !voice->playerEmitted
                                      && !voice->skipPortalRouting
                                      && voice->pathingSource != nullptr;
            if (useSteamAudioPathing && !voice->sourceEnded) {
                float lineDist = glm::length(voice->worldPos - mListenerPos);

                // Mirror the setInputs gate from earlier in this loopStep.
                // Voices we skipped staging for (out-of-range) must NOT
                // read back eqCoeffs from the source — the solver never
                // wrote new values, so getOutputs returns the source's
                // pristine create-time pathingState.eq[i] = 0.1f sentinel.
                // Feeding that into eqCoeffsToDspMapping produces gain
                // ≈ 0.5 and blocking ≈ 0.36 — every skipped voice becomes
                // audibly quiet and muffled (the "ambient jitter"
                // pattern). Treat skipped voices as fully unobstructed.
                if (lineDist > voice->maxAudibleDist) {
                    // Out-of-range: skip the iplSourceGetOutputs read
                    // entirely. The solver never staged this voice's
                    // pathing inputs (it's skipped on the input-staging
                    // side too) so getOutputs would return the
                    // pristine 0.1f sentinel. Phase 4: also invalidate
                    // the audio-thread path target so iplPathEffectApply
                    // is bypassed this callback. Distance attenuation
                    // downstream in iplDirectEffect already silences
                    // the dry bus for out-of-range voices.
                    voice->dspNode.pathTargetValid.store(
                        false, std::memory_order_release);
                } else {
                    // iplSourceGetOutputs is safe from any thread; reads
                    // the most recently committed sim result. No gating
                    // on `mPathingSim->isRunning()` — the worker writes
                    // outputs at the end of each iteration, and any read
                    // mid-iteration returns the previous iteration's
                    // cached values (which is exactly the throttle-skip
                    // behaviour the legacy synchronous path also relied
                    // on).
                    IPLSimulationOutputs pathingOut{};
                    iplSourceGetOutputs(voice->pathingSource,
                        IPL_SIMULATIONFLAGS_PATHING, &pathingOut);

                    // Sentinel detection. Steam Audio initializes
                    // pathingState.eq[i] to the bit-identical literal
                    // 0.1f (0x3DCCCCCD) at source-create time, and
                    // leaves it untouched if either (a) the source has
                    // not yet had a successful pathing solve (startup
                    // window — up to one throttle interval after voice
                    // spawn) or (b) the bake graph has no reachable
                    // path between the source and listener entry probes
                    // (sparse bake / disconnected components / all
                    // candidate paths invalidated). Feeding 0.1 in each
                    // band into eqCoeffsToDspMapping produces gain ≈
                    // 0.5 and blocking ≈ 0.36 — the audible "ambient
                    // jitter" artefact (every new or unreachable voice
                    // sounds quiet and muffled for the duration of the
                    // condition).
                    //
                    // Treat sentinel reads as "no pathing data available
                    // — fall back to passthrough". Distance attenuation
                    // and HRTF direction are still applied by the
                    // direct path downstream, so the voice sounds
                    // correct on its own line of sight; we just don't
                    // attenuate based on a phantom obstruction.
                    //
                    // Bit-pattern check (not float-equality) because
                    // the sentinel is a known literal that we want to
                    // catch precisely. Real eqCoeffs of exactly 0.1f
                    // in all three bands would be vanishingly unlikely
                    // and would round-trip through the mapping
                    // identically to passthrough anyway.
                    bool pathingSentinel = false;
                    {
                        uint32_t b0, b1, b2;
                        std::memcpy(&b0, &pathingOut.pathing.eqCoeffs[0], 4);
                        std::memcpy(&b1, &pathingOut.pathing.eqCoeffs[1], 4);
                        std::memcpy(&b2, &pathingOut.pathing.eqCoeffs[2], 4);
                        constexpr uint32_t kSentinel = 0x3DCCCCCD; // bits of 0.1f
                        pathingSentinel = (b0 == kSentinel && b1 == kSentinel && b2 == kSentinel);
                    }
                    // Per-voice frame-counter book-keeping for sentinel-
                    // cause diagnostics. Incremented every output read so
                    // the [PATHING_SENTINEL] log can report:
                    //   • spawn-age in loopSteps (startup window?)
                    //   • frames-since-last-solve (steady sentinel or
                    //     transient flicker?)
                    // Reset of `loopStepsSinceLastSolve` happens in the
                    // non-sentinel branch below.
                    voice->loopStepsSinceSpawn++;
                    if (pathingSentinel) voice->loopStepsSinceLastSolve++;

                    if (pathingSentinel) {
                        // One-shot diagnostic so we can distinguish
                        // startup-window sentinels (logs once per new
                        // voice, then quiet) from persistent
                        // unreachable-bake-edge sentinels (the voice
                        // keeps emitting until halted — actionable
                        // bake / probe-graph diagnostic) from flicker
                        // events (a voice that has solved before but
                        // is now intermittently sentinel — points at
                        // non-deterministic validation or scene
                        // mutation between pathing-sim calls).
                        static std::atomic<int> sSentinelLogCount{0};
                        int n = sSentinelLogCount.fetch_add(1, std::memory_order_relaxed);
                        if (n < 32 || (n % 256) == 0) {
                            std::fprintf(stderr,
                                "[PATHING_SENTINEL] h=%u '%s' dist=%.1f "
                                "everSolved=%d sinceSpawn=%u sinceSolve=%u "
                                "dueThisStep=%d — using cached or bypass "
                                "[#%d]\n",
                                handle, voice->schemaName.c_str(),
                                lineDist,
                                voice->pathingEverSolved ? 1 : 0,
                                voice->loopStepsSinceSpawn,
                                voice->loopStepsSinceLastSolve,
                                mPathingDueThisStep ? 1 : 0,
                                n + 1);
                        }
                    } else if (!voice->pathingEverSolved) {
                        // First non-sentinel read for this voice — flip
                        // the flag and emit a one-shot log so the
                        // startup-window vs. persistent-sentinel
                        // distinction is visible per-voice.
                        voice->pathingEverSolved = true;
                        static std::atomic<int> sFirstSolveLogCount{0};
                        int n = sFirstSolveLogCount.fetch_add(1, std::memory_order_relaxed);
                        if (n < 64 || (n % 256) == 0) {
                            std::fprintf(stderr,
                                "[PATHING_FIRST_SOLVE] h=%u '%s' dist=%.1f "
                                "eq=[%.3f,%.3f,%.3f] sProbe=%d lProbe=%d [#%d]\n",
                                handle, voice->schemaName.c_str(), lineDist,
                                pathingOut.pathing.eqCoeffs[0],
                                pathingOut.pathing.eqCoeffs[1],
                                pathingOut.pathing.eqCoeffs[2],
                                /*sProbe=*/-1, /*lProbe=*/-1, n + 1);
                        }
                    }

                    // ── Closed-door blocking (geometry-aware via Steam Audio) ──
                    //
                    // Doors register `IPLInstancedMesh` geometry with the
                    // acoustic scene (see registerDoorGeometry). Steam
                    // Audio's pathing solver runs ray-cast validation on
                    // every baked path edge (`enableValidation = TRUE` in
                    // the per-voice setInputs above); edges that intersect
                    // a closed door's geometry fail validation, and
                    // `findAlternatePaths = TRUE` searches for a
                    // geometrically-valid alternate. If none exists the
                    // resulting eqCoeffs collapse toward zero. This makes
                    // door blocking physically accurate and route-aware:
                    // closing a door in a room with no alternate route
                    // silences cross-room sound; closing a door in a room
                    // with an alternate corridor only partially attenuates
                    // (sound reroutes around). No per-frame BFS needed.
                    //
                    // The `setBlockingFactor` map is still maintained by
                    // DoorSystem for AIHearingService and the legacy
                    // room-BFS path (when `probe_pathing = false`), but is
                    // not consulted here.

                    // Phase 4 — Steam Audio sole authority for player
                    // routing. The legacy eqCoeffsToDspMapping(...) →
                    // (portalAttenuation, portalBlocking) translation is
                    // gone; Steam Audio's per-frame
                    // `IPLPathEffectParams` (in pathingOut.pathing) is
                    // staged for the audio thread to consume via
                    // iplPathEffectApply directly. The path effect
                    // produces routing-aware spatialized binaural that
                    // mixes additively into the dry bus — no manual
                    // 3-band → scalar collapse and no separate door LPF
                    // in the player path.
                    //
                    // Note on shCoeffs lifetime: Steam Audio writes
                    // `IPLPathEffectParams.shCoeffs` as a pointer into
                    // its own internal staging buffer; the pointer is
                    // stable until the next pathing-sim iteration.
                    // We copy the params struct verbatim (POD copy
                    // includes the pointer) and the audio thread reads
                    // it back without locking. Per phonon.h, this
                    // matches Valve's Unity integration pattern.
                    // (validation flicker, probe-graph misses) that
                    // produce audible choppy gaps in the wet bus.
                    // Periodic log every N pathing calls for cadence,
                    // plus always-on SPIKE log when eqCoeffs sum drops
                    // below the threshold (the canonical signature of a
                    // failed Steam Audio pathing solve).
                    {
                        const float eqL = pathingOut.pathing.eqCoeffs[0];
                        const float eqM = pathingOut.pathing.eqCoeffs[1];
                        const float eqH = pathingOut.pathing.eqCoeffs[2];
                        const float eqSum = eqL + eqM + eqH;
                        static std::atomic<uint64_t> sPathLogCount{0};
                        const uint64_t cc = sPathLogCount.fetch_add(
                            1, std::memory_order_relaxed);
                        const bool periodic = (cc % 256) == 0;
                        const bool spike    = (eqSum < 1.5f);
                        if (periodic || spike) {
                            // Print exact bits as hex so we can spot
                            // suspicious sentinel patterns (e.g. all
                            // bands holding the bit-identical 0x3DCCCCCD
                            // = 0.1f literal) vs naturally varying
                            // visibility values.
                            uint32_t eqLBits, eqMBits, eqHBits;
                            std::memcpy(&eqLBits, &eqL, 4);
                            std::memcpy(&eqMBits, &eqM, 4);
                            std::memcpy(&eqHBits, &eqH, 4);

                            // Nearest-probe diagnostic. The hypothesis is
                            // that Steam Audio's source-probe lookup
                            // silently skips sources whose distance to the
                            // nearest probe exceeds an internal radius
                            // (derived from bake spacing). When that
                            // happens, pathing never runs for the source
                            // and SimulationData::pathingOutputs.eq stays
                            // at the init-default 0.1 across all bands —
                            // matching every SPIKE we currently see.
                            // sProbeD = source→nearest-probe distance (ft)
                            // lProbeD = listener→nearest-probe distance
                            // sProbeIdx / lProbeIdx = probe indices for
                            // cross-referencing with the probe overlay.
                            // Lookup is O(N_probes) but only runs inside
                            // this periodic|spike-gated branch so cost is
                            // bounded; never enters the per-frame hot
                            // path. mProbeManager null-checked because
                            // pathing-gated branch above already requires
                            // mPathingProbeBatchAdded, but be defensive.
                            int   sProbeIdx = -1, lProbeIdx = -1;
                            float sProbeD = -1.0f, lProbeD = -1.0f;
                            if (mProbeManager) {
                                const auto &probePos = mProbeManager->getProbePositions();
                                if (!probePos.empty()) {
                                    float sBestSq = std::numeric_limits<float>::max();
                                    float lBestSq = std::numeric_limits<float>::max();
                                    for (size_t i = 0; i < probePos.size(); ++i) {
                                        Vector3 ds = probePos[i] - voice->worldPos;
                                        float sq = glm::dot(ds, ds);
                                        if (sq < sBestSq) { sBestSq = sq; sProbeIdx = static_cast<int>(i); }
                                        Vector3 dl = probePos[i] - mListenerPos;
                                        float lq = glm::dot(dl, dl);
                                        if (lq < lBestSq) { lBestSq = lq; lProbeIdx = static_cast<int>(i); }
                                    }
                                    sProbeD = std::sqrt(sBestSq);
                                    lProbeD = std::sqrt(lBestSq);
                                }
                            }
                            // doorBlk dropped from the format string: door
                                      // blocking is now handled by Steam Audio's
                                      // geometry-aware path validation (instanced
                                      // door meshes in the IPLScene), not by the
                                      // per-frame BFS lookup that used to populate
                                      // this field.
                            // Phase 4: gain/blk scalars removed from the
                            // format string. Path-effect output is no
                            // longer a (gain, LPF) collapse — the raw
                            // eqCoeffs feed iplPathEffectApply directly
                            // on the audio thread. The eqCoeffs themselves
                            // remain the load-bearing diagnostic.
                            AUDIO_LOG("[PATH] h=%u '%s' dist=%.1f "
                                      "eq=[%.6f,%.6f,%.6f] "
                                      "eqBits=[%08x,%08x,%08x] "
                                      "srcW=(%.1f,%.1f,%.1f) "
                                      "lstW=(%.1f,%.1f,%.1f) "
                                      "sProbe=%d d=%.1f lProbe=%d d=%.1f%s\n",
                                      handle, voice->schemaName.c_str(),
                                      lineDist, eqL, eqM, eqH,
                                      eqLBits, eqMBits, eqHBits,
                                      voice->worldPos.x, voice->worldPos.y, voice->worldPos.z,
                                      mListenerPos.x, mListenerPos.y, mListenerPos.z,
                                      sProbeIdx, sProbeD, lProbeIdx, lProbeD,
                                      spike ? " SPIKE" : "");
                        }
                    }

                    // Phase 4 — per-voice last-good IPLPathEffectParams
                    // cache (§5.1 of PLAN.AUDIO_REALTIME_ARCHITECTURE.md
                    // moved from scalar-mapping level to params level).
                    //
                    //   (1) Sentinel + `pathingEverSolved`: replay the
                    //       cached last-good IPLPathEffectParams into the
                    //       voice's audio-thread target. iplPathEffectApply
                    //       sees the same params it saw on the most recent
                    //       successful solve — no level / pan / EQ pop
                    //       across the flicker.
                    //   (2) Sentinel + !`pathingEverSolved`: synthetic
                    //       passthrough. Mark the target as invalid so
                    //       the audio thread skips iplPathEffectApply
                    //       entirely; the voice falls back to dry-binaural
                    //       only (acoustically equivalent to "fully
                    //       unobstructed", which is the right answer for
                    //       a brand-new same-room voice in the startup
                    //       window). Steam Audio's iplPathEffectApply
                    //       does not document a well-defined behaviour
                    //       for the all-0.1f sentinel, so bypassing is
                    //       safer than feeding a known-bad params struct.
                    //   (3) Non-sentinel: copy the resolved params into
                    //       the lastGoodPathParams cache AND the per-
                    //       voice audio-thread target, then mark the
                    //       target valid + reset sinceLastSolve.
                    if (pathingSentinel) {
                        if (voice->pathingEverSolved) {
                            voice->dspNode.pathTargetParams =
                                voice->lastGoodPathParams;
                            voice->dspNode.pathTargetValid.store(
                                true, std::memory_order_release);
                        } else {
                            voice->dspNode.pathTargetValid.store(
                                false, std::memory_order_release);
                        }
                    } else {
                        // pathingOut.pathing already carries eqCoeffs[3]
                        // and shCoeffs pointer + order. Copy the params
                        // struct verbatim; Steam Audio owns the shCoeffs
                        // pointer's lifetime (stable across iterations
                        // until the next runPathing). Listener pose + HRTF
                        // fields are filled in by the audio thread (they
                        // depend on per-callback listener-orientation
                        // state).
                        voice->dspNode.pathTargetParams = pathingOut.pathing;
                        voice->dspNode.pathTargetValid.store(
                            true, std::memory_order_release);
                        // Cache for sentinel-replay on the next flicker.
                        voice->lastGoodPathParams = pathingOut.pathing;
                        voice->loopStepsSinceLastSolve = 0;
                    }

                    // Phase 4: portalAttenuation / portalBlocking are no
                    // longer driven by BFS or by the eqCoeffs→scalar
                    // mapping. Steam Audio's path effect handles routing
                    // + EQ on the audio thread directly; the dry path is
                    // un-attenuated portal-wise and the wet-bus
                    // reflection-send keeps its LPF wide open (so reverb
                    // through a closed door is not double-muffled — the
                    // path effect's pathing already encodes the route).
                    voice->dspNode.portalAttenuation = 1.0f;
                    voice->dspNode.portalBlocking    = 0.0f;

                    // ── Diagnostic: catch per-pathing-tick eq-jitter ──
                    //
                    // The legacy [PATH_JUMP] log tracked the scalar
                    // (portalAttenuation, portalBlocking) collapse the
                    // eqCoeffsToDspMapping produced. Phase 4 removed
                    // that collapse; we now track the underlying
                    // eqCoeffs vector itself, with the same "first 16
                    // + every 32nd" rate limit. Cache replay still
                    // engages (pathTargetParams freezes), so jumps
                    // here correspond to non-sentinel reads with
                    // genuinely different solver outputs — the audible
                    // signature of a moving listener crossing a path-
                    // diffraction discontinuity.
                    {
                        static std::unordered_map<SoundHandle,
                            std::array<float, 3>> sPrevEq;
                        static std::unordered_map<SoundHandle, int> sJumpCount;
                        const auto &cur = voice->dspNode.pathTargetParams;
                        auto it = sPrevEq.find(handle);
                        if (it != sPrevEq.end()) {
                            float d0 = std::fabs(cur.eqCoeffs[0] - it->second[0]);
                            float d1 = std::fabs(cur.eqCoeffs[1] - it->second[1]);
                            float d2 = std::fabs(cur.eqCoeffs[2] - it->second[2]);
                            float maxD = std::max({d0, d1, d2});
                            // 0.05 ≈ −0.4 dB step on any one band.
                            if (maxD > 0.05f) {
                                int n = ++sJumpCount[handle];
                                if (n <= 16 || (n % 32) == 0) {
                                    std::fprintf(stderr,
                                        "[PATH_JUMP] h=%u '%s' "
                                        "eq [%.3f,%.3f,%.3f]→[%.3f,%.3f,%.3f] "
                                        "(maxΔ=%.3f) sentinel=%d [#%d]\n",
                                        handle, voice->schemaName.c_str(),
                                        it->second[0], it->second[1], it->second[2],
                                        cur.eqCoeffs[0], cur.eqCoeffs[1], cur.eqCoeffs[2],
                                        maxD, pathingSentinel ? 1 : 0, n);
                                }
                            }
                        }
                        sPrevEq[handle] = {cur.eqCoeffs[0],
                                           cur.eqCoeffs[1],
                                           cur.eqCoeffs[2]};
                    }
                }
            }

            if (voice->dspNode.effectsReady) {
                // Always update HRTF direction — it doesn't depend on simulation
                // outputs and pending voices still need correct spatialization.
                if (!voice->dspNode.usePortalRouting) {
                    Vector3 toSource = voice->worldPos - mListenerPos;
                    float dist = glm::length(toSource);
                    if (dist > kDistanceEpsilonFt) {
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

                // Phase 4: the `pathing_gain_scale` knob applied to the
                // dry path's `distanceAttenuation` is removed. That
                // boost compensated for the BFS→scalar→DSP mapping's
                // tendency to under-amplify cross-room voices when the
                // eqCoeffs collapse to a scalar. With Steam Audio's
                // IPLPathEffect now driving cross-room audio directly
                // (additive wet bus on top of dry binaural), the dry
                // path and the routed path are independent — boosting
                // distanceAttenuation would over-amplify the
                // direct-line wall-transmission signal. The knob still
                // exists in the public API (no setter removed) for
                // backwards compat; it just no longer has a consumer
                // in the player DSP chain.

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
                // capped at mReverbVoices to stay within the audio
                // callback budget (~1.5ms per convolution voice).
                //
                // Note: reflCandidates was already shrunk to
                // (mReverbVoices − tailCount) above, so in normal flow
                // an isReflVoice voice always has slot budget here.  We still
                // gate by canAffordConvolution as defence-in-depth — if the
                // cap was lowered at runtime, or a tail voice was missed in
                // the pre-count, this prevents overshoot.
                bool isReflVoice = reflCandidateSet.count(handle) > 0;
                bool canAffordConvolution = (activeConvolutionCount < mReverbVoices);
                // Pure parametric mode produces no IR — Steam Audio's
                // parametric reflection effect uses only reverbTimes[3]
                // (per phonon.h: ir/irSize/eq/delay are CONVOLUTION/HYBRID
                // -only fields). For baked sources the simulator never
                // populates outputs.reflections.ir or irSize when
                // reflectionType == PARAMETRIC; reverbTimes come from the
                // probe batch via iplProbeBatchGetReverb in the pin block
                // below. So the "has reflection data available" gate
                // splits by mode: parametric needs only probe data;
                // convolution / hybrid still need a real IR.
                const bool isParametricMode =
                    (mReflectionType == ReflectionType::Parametric);
                bool hasBakedData = voice->dspNode.reflectionEffect
                    && mProbeManager->hasReflections()
                    && (isParametricMode || outputs.reflections.irSize > 0);
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
                    // ── Per-voice IR pinning (2026-05-20) ──
                    //
                    // First time a voice has valid IR data, COPY the full IR
                    // payload into voice->pinnedIRData and pin the params
                    // struct (with .ir pointing at our copy) for the voice's
                    // entire lifetime. Subsequent sim cycles produce updated
                    // IRs in outputs.reflections, but we ignore them — keeping
                    // the convolution effect's params.ir pointer stable
                    // eliminates the Steam-Audio-internal IR crossfade that
                    // produces the ~5 Hz amplitude pulse ("beating") in the
                    // wet bus. See VoicePool.h "Pinned per-voice IR" docs.
                    // Pin-eligibility split by reflection algorithm:
                    //   • CONVOLUTION / HYBRID: need a valid IR handle from
                    //     the simulator (irSize > 0 && ir != nullptr).
                    //   • PARAMETRIC: no IR is ever produced — pin as soon
                    //     as a probe batch with parametric data is loaded,
                    //     since the apply call needs only reverbTimes.
                    const bool hasConvIR = outputs.reflections.irSize > 0
                        && outputs.reflections.ir != nullptr;
                    const bool canPinParametric = isParametricMode
                        && mProbeManager && mProbeManager->hasReflections();
                    if (!voice->reflectionIRPinned
                        && (hasConvIR || canPinParametric)) {
                        // outputs.reflections.ir is an opaque
                        // IPLReflectionEffectIR handle owned by Steam
                        // Audio; we hold the handle as-is and never
                        // refresh it. We override several fields from
                        // our known-good config values rather than
                        // trusting whatever Steam Audio happened to
                        // leave in outputs.reflections — those output
                        // fields may not be populated on the first
                        // cycle, and pinning a wrong value sticks for
                        // the voice's lifetime.
                        //
                        voice->pinnedParams = outputs.reflections;
                        voice->pinnedParams.type =
                            (mReflectionType == ReflectionType::Hybrid)
                                ? IPL_REFLECTIONEFFECTTYPE_HYBRID
                                : (mReflectionType == ReflectionType::Parametric)
                                    ? IPL_REFLECTIONEFFECTTYPE_PARAMETRIC
                                    : IPL_REFLECTIONEFFECTTYPE_CONVOLUTION;
                        voice->pinnedParams.numChannels = mAmbisonicsChannels;
                        // delay = 0: parametric FDN runs from t=0,
                        // co-decaying with the convolution.
                        //
                        // Steam Audio's hybrid design (per phonon.h
                        // docs + hybrid_reverb_estimator.cpp) is that
                        // delay should be `rampStart =
                        // (1-overlap)·transitionTime·Fs`, at which
                        // point an IR-domain equal-power crossfade is
                        // supposed to mask the parametric onset: the
                        // conv IR is multiplied by sqrt(alpha) in the
                        // ramp window [rampStart, rampEnd], and the
                        // parametric's expected impulse response is
                        // subtracted from channel 0 of the conv IR so
                        // the sum stays energy-flat. Past rampEnd the
                        // conv IR is zeroed and parametric carries
                        // the tail alone. Steam Audio's own simulator
                        // computes this same value and writes it to
                        // outputs.reflections.delay.
                        //
                        // Empirically that doesn't work in our setup
                        // — every non-zero delay we've tried (rampStart
                        // = 18000, rampEnd = 24000, irSize = 26400)
                        // is audible as a distinct "second reverb
                        // onset" at t=delay. Suspected reasons:
                        //   • The IR-side cancellation only touches
                        //     channel 0 (W). Channels 1..N (X/Y/Z and
                        //     up) are faded by sqrt(alpha) but not
                        //     cancelled, so the spatial character of
                        //     the wet field jumps even when energy
                        //     stays flat.
                        //   • Energy-flat ≠ perceptually-flat: the
                        //     timbre of an FDN onset is distinguishable
                        //     from the truncated room IR even at
                        //     matched RMS.
                        // Driving the parametric from t=0 sidesteps
                        // both — the two signals co-decay from the
                        // same instant, so there's no second event
                        // to localize in time.
                        //
                        // Tradeoff: in the [0, rampStart] window the
                        // wet bus carries both the full early-
                        // reflection IR AND the parametric's initial
                        // FDN response, which is technically "doubled
                        // wet energy." In practice this manifests as
                        // a slightly wetter onset, not a second-event
                        // artifact, and is what was measured to sound
                        // best when the user A/B'd it against
                        // delay=rampStart and delay=rampEnd.
                        //
                        // Requires reverbTimes to be populated below
                        // (via iplProbeBatchGetReverb) — with
                        // RT60=0 Steam Audio's parametric default-
                        // fallback rings at full amplitude and
                        // produces a very different "doubled reverb"
                        // symptom.
                        voice->pinnedParams.delay = 0;

                        // Per-call processing length. `irSize` in the params
                        // struct is the per-call "samples to process" knob
                        // (≤ effect's create-time irSize). For CONVOLUTION/
                        // HYBRID this controls how much of the pinned IR is
                        // convolved each call. For PARAMETRIC there is no
                        // IR — but Steam Audio still consults this field as
                        // the per-call processing length for the FDN: pass
                        // 0 and apply() returns silence regardless of how
                        // well-populated reverbTimes are (verified
                        // empirically: pinned reverbTimes correct, footstep
                        // mono peak ~0.3 reaching the worker, wet output
                        // peak exactly 0.0). For baked CONVOLUTION/HYBRID
                        // the outputs.reflections copy already carries a
                        // valid irSize so we leave it alone; for PARAMETRIC
                        // we force the create-time effect length so apply
                        // produces full-length output.
                        if (isParametricMode) {
                            voice->pinnedParams.irSize = static_cast<IPLint32>(
                                mRealtimeDuration
                                * static_cast<float>(mReflectionSampleRate));
                        }

                        // Fetch baked reverbTimes from the nearest probe
                        // to the listener. Steam Audio does NOT populate
                        // outputs.reflections.reverbTimes for baked
                        // sources — we have to query the probe batch
                        // directly via iplProbeBatchGetReverbTimes.
                        // Without this, reverbTimes stays at the
                        // outputs default (observed: all zeros), which
                        // makes the parametric reverb in hybrid mode
                        // run with undefined / default-fallback decay,
                        // producing the audible "second reverb" the
                        // user heard after IR pinning landed.
                        //
                        // We use the listener-nearest probe (not the
                        // source-nearest) because reverbTimes describe
                        // "how long does the room ring at this point
                        // in space" — that's the LISTENER's experience
                        // of the room's RT60.
                        int probeLookupIdx = -1;
                        if (mProbeManager && mProbeManager->hasReflections()) {
                            IPLProbeBatch pb = mProbeManager->getProbeBatch();
                            const auto &probePos = mProbeManager->getProbePositions();
                            if (pb && !probePos.empty()) {
                                int nearestIdx = 0;
                                float nearestDistSq = std::numeric_limits<float>::max();
                                for (size_t i = 0; i < probePos.size(); ++i) {
                                    Vector3 d = probePos[i] - mListenerPos;
                                    float dsq = glm::dot(d, d);
                                    if (dsq < nearestDistSq) {
                                        nearestDistSq = dsq;
                                        nearestIdx = static_cast<int>(i);
                                    }
                                }
                                IPLBakedDataIdentifier reflId{};
                                reflId.type = IPL_BAKEDDATATYPE_REFLECTIONS;
                                reflId.variation = IPL_BAKEDDATAVARIATION_REVERB;
                                iplProbeBatchGetReverb(pb, &reflId, nearestIdx,
                                    voice->pinnedParams.reverbTimes);
                                probeLookupIdx = nearestIdx;
                            }
                        }

                        voice->reflectionIRPinned = true;
                        AUDIO_LOG("[PINNED_IR] h=%u '%s' irSize=%d numChannels=%d "
                                  "delay=%d reverbTimes=[%.3f,%.3f,%.3f] "
                                  "eq=[%.3f,%.3f,%.3f] probeLookupIdx=%d "
                                  "hasReflections=%d probeCount=%zu\n",
                                  handle, voice->schemaName.c_str(),
                                  outputs.reflections.irSize,
                                  outputs.reflections.numChannels,
                                  voice->pinnedParams.delay,
                                  voice->pinnedParams.reverbTimes[0],
                                  voice->pinnedParams.reverbTimes[1],
                                  voice->pinnedParams.reverbTimes[2],
                                  voice->pinnedParams.eq[0],
                                  voice->pinnedParams.eq[1],
                                  voice->pinnedParams.eq[2],
                                  probeLookupIdx,
                                  mProbeManager ? mProbeManager->hasReflections() : -1,
                                  mProbeManager ? mProbeManager->getProbePositions().size() : 0);
                    }

                    if (voice->reflectionIRPinned) {
                        // Use the pinned params unconditionally — no further
                        // ir-pointer or sample updates ever happen for this
                        // voice. This is the architectural invariant that
                        // suppresses the per-sim-cycle beating artefact.
                        voice->dspNode.reflectionParams = voice->pinnedParams;
                    } else {
                        // Pre-pin (first ever loopStep for this voice with
                        // outputs.reflections still pending): copy through
                        // current outputs unchanged. The next cycle should
                        // pin proper data.
                        voice->dspNode.reflectionParams = outputs.reflections;
                        voice->dspNode.reflectionParams.type =
                            (mReflectionType == ReflectionType::Hybrid)
                                ? IPL_REFLECTIONEFFECTTYPE_HYBRID
                                : (mReflectionType == ReflectionType::Parametric)
                                    ? IPL_REFLECTIONEFFECTTYPE_PARAMETRIC
                                    : IPL_REFLECTIONEFFECTTYPE_CONVOLUTION;
                        voice->dspNode.reflectionParams.numChannels = mAmbisonicsChannels;
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
                                  activeConvolutionCount, mReverbVoices,
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

    // ── [ROOM_ACOUSTICS] diagnostic ──
    //
    // Periodic log line comparing the designer-tagged EAX preset's T60
    // against Steam Audio's geometry-derived per-band RT60 for the
    // listener's room. Fires on room change with a 2 s anti-flicker
    // throttle. Informational only — does not change behaviour. Surfaces
    // mismatches that would benefit from a per-source `reverbScale[3]`
    // correction (which the simulation inputs above already plumb).
    if (mAudioReady && mListenerRoom && !mRoomEAXPresets.empty()) {
        static int sLastLoggedRoomId = INT32_MIN;
        static auto sLastLoggedTime  = std::chrono::steady_clock::now();
        int currentRoomId = static_cast<int>(mListenerRoom->getRoomID());
        auto now = std::chrono::steady_clock::now();
        auto sinceLastLog =
            std::chrono::duration_cast<std::chrono::seconds>(now - sLastLoggedTime).count();
        bool roomChanged = (currentRoomId != sLastLoggedRoomId);
        if (roomChanged && sinceLastLog >= 2) {
            auto eaxIt = mRoomEAXPresets.find(static_cast<int32_t>(currentRoomId));
            if (eaxIt != mRoomEAXPresets.end() && eaxIt->second < 26) {
                uint32_t eaxIdx = eaxIt->second;
                const char *eaxName = kEAXPresets[eaxIdx].name;
                float eaxT60        = kEAXPresets[eaxIdx].decayTime;
                // Find an active reflection voice and use its reverbTimes
                // as the derived value. Steam Audio populates these per
                // source from the simulator's geometry trace + parametric
                // reverb estimation; they're the same numbers the hybrid
                // tail uses, so the comparison is apples-to-apples.
                float rtLow = 0.0f, rtMid = 0.0f, rtHigh = 0.0f;
                bool found = false;
                for (auto &[h, v] : mVoicePool->voices()) {
                    // Accept any active reflection voice whose reverbTimes
                    // are populated. CONVOLUTION/HYBRID populate irSize too;
                    // PARAMETRIC has irSize == 0 by design but still carries
                    // valid reverbTimes — gate on the type field, not irSize.
                    if (!v->dspNode.reflectionsActive.load(std::memory_order_relaxed))
                        continue;
                    const auto &rp = v->dspNode.reflectionParams;
                    const bool hasData =
                        (rp.type == IPL_REFLECTIONEFFECTTYPE_PARAMETRIC)
                            ? (rp.reverbTimes[0] > 0.0f
                               || rp.reverbTimes[1] > 0.0f
                               || rp.reverbTimes[2] > 0.0f)
                            : (rp.irSize > 0);
                    if (hasData) {
                        rtLow  = rp.reverbTimes[0];
                        rtMid  = rp.reverbTimes[1];
                        rtHigh = rp.reverbTimes[2];
                        found  = true;
                        break;
                    }
                }
                if (found) {
                    AUDIO_LOG("[ROOM_ACOUSTICS] room=%d eax=%s (T60=%.2fs) "
                              "derived_rt60_band=(%.2f, %.2f, %.2f)s\n",
                              currentRoomId, eaxName, eaxT60,
                              rtLow, rtMid, rtHigh);
                } else {
                    AUDIO_LOG("[ROOM_ACOUSTICS] room=%d eax=%s (T60=%.2fs) "
                              "derived_rt60_band=(no active reflection voice yet)\n",
                              currentRoomId, eaxName, eaxT60);
                }
                sLastLoggedRoomId = currentRoomId;
                sLastLoggedTime   = now;
            }
        }
    }

    // Tick reverb tail timers for voices whose source audio has ended.
    // The voice stays alive during the tail so the per-voice convolution
    // continues feeding its IR tail.
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
                    // Clear reflectionsActive at tail end. Without this the
                    // staging guard in reflectionMixNodeProcess
                    // (`if (node->reflectionsActive && node->reflectionEffect
                    // && node->reflectionParams.irSize > 0 ...)`) keeps
                    // matching every callback until the voice is fully
                    // destroyed — so the convolution worker pool continues
                    // applying iplReflectionEffectApply to a long-dead
                    // voice. Over a busy session this accumulates: every
                    // ended voice's slot stays allocated and consuming
                    // CPU until destroyVoice runs, easily piling to the
                    // full MAX_ACTIVE_VOICES (= 32) regardless of the
                    // reverb_voices config cap. With this clear the
                    // staging guard rejects the voice on the very next
                    // callback after the tail finishes — same callback
                    // the slot-release loop at line ~4419 frees
                    // reflSlotOwned, in lockstep.
                    voice->dspNode.reflectionsActive.store(
                        false, std::memory_order_release);
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
    //
    // Source flags must include every simulation type the source will
    // ever participate in — Steam Audio uses these to decide which
    // per-source state slots to allocate. The direct simulator now
    // carries DIRECT only (pathing moved to a separate simulator
    // iterated by a background worker — see PathingSimulator.h); the
    // pathing-side source is created below as `voice.pathingSource`.
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

    // ── Pathing source ──
    // Created against the pathing simulator. Player-emitted voices skip
    // pathing entirely (source ≈ listener so the probe-graph collapses
    // to a self-loop and the iteration cost is wasted). Same defer-add
    // pattern as the reflection source: if the worker is mid-iteration
    // we can't safely mutate the source list, so queue the add until
    // the next idle frame in loopStep.
    if (!voice.playerEmitted && mPathingSim && mPathingSim->simulator()) {
        IPLSourceSettings srcSettings{};
        srcSettings.flags = IPL_SIMULATIONFLAGS_PATHING;

        IPLerror err = iplSourceCreate(mPathingSim->simulator(), &srcSettings,
                                       &voice.pathingSource);
        if (err != IPL_STATUS_SUCCESS) {
            LOG_ERROR("AudioService: pathing iplSourceCreate failed (error %d)", err);
            voice.pathingSource = nullptr;
            // Continue without pathing — direct path still works and the
            // per-frame setInputs / getOutputs blocks skip on null source.
        } else if (mPathingSim->isRunning()) {
            mPathingSim->queueSourceAdd(voice.pathingSource);
            mPathingSim->setSimulatorDirty();
        } else {
            iplSourceAdd(voice.pathingSource, mPathingSim->simulator());
            mPathingSim->setSimulatorDirty();
        }
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
            // Direct + pathing sources already created above; don't leak
            // them. Caller will treat the voice as having no spatial
            // audio.
            iplSourceRemove(voice.directSource, mDirectSimulator);
            iplSimulatorCommit(mDirectSimulator);
            iplSourceRelease(&voice.directSource);
            if (voice.pathingSource && mPathingSim) {
                // Symmetric defer-flush dance with the create path above.
                if (mPathingSim->removeFromPendingAdds(voice.pathingSource)) {
                    iplSourceRelease(&voice.pathingSource);
                } else if (mPathingSim->isRunning()) {
                    mPathingSim->queueSourceRemove(voice.pathingSource);
                    voice.pathingSource = nullptr;
                    mPathingSim->setSimulatorDirty();
                } else if (mPathingSim->simulator()) {
                    iplSourceRemove(voice.pathingSource, mPathingSim->simulator());
                    iplSourceRelease(&voice.pathingSource);
                    mPathingSim->setSimulatorDirty();
                } else {
                    iplSourceRelease(&voice.pathingSource);
                }
            }
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
    // Pre-allocate sub-source slots. In SA mode (probe_pathing: true,
    // shipping default) only slot 0 is ever ACTIVE per the
    // PLAN.MULTI_PATH_SA_MIGRATION.md decision, so we skip slots 1-3
    // to save ~3× per-voice IPLSource + IPLDirectEffect + IPLBinauralEffect
    // memory and the eager source-register / simulator-commit cost. In
    // BFS mode (probe_pathing: false) all four are needed for multi-
    // portal fan-out. Teardown (~ActiveVoice and removeVoiceSource)
    // skips slots with null IPL handles so a partial allocation is safe.
    //
    // mProbePathingEnabled is captured at voice spawn; runtime toggle of
    // probe_pathing is not supported by the slot lifecycle today (would
    // require re-allocating slots on flip). No setter exists so this
    // can't happen in practice; document the constraint here if a
    // setter is added later.
    const int slotsToAlloc = mProbePathingEnabled ? 1 : kMaxSubSources;
    {
        IPLSourceSettings srcSettings{};
        srcSettings.flags = IPL_SIMULATIONFLAGS_DIRECT;
        int builtSlotSources = 0;
        for (int slotIdx = 0; slotIdx < slotsToAlloc; ++slotIdx) {
            auto &slot = voice.dspNode.subSources[slotIdx];
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
                // Pathing source rollback. Same defer-flush dance as
                // reflectionSource — the worker may be mid-iteration
                // and we can't safely call iplSourceRemove while it
                // runs.
                if (voice.pathingSource && mPathingSim) {
                    if (mPathingSim->removeFromPendingAdds(voice.pathingSource)) {
                        iplSourceRelease(&voice.pathingSource);
                    } else if (mPathingSim->isRunning()) {
                        mPathingSim->queueSourceRemove(voice.pathingSource);
                        voice.pathingSource = nullptr;
                        mPathingSim->setSimulatorDirty();
                    } else if (mPathingSim->simulator()) {
                        iplSourceRemove(voice.pathingSource, mPathingSim->simulator());
                        iplSourceRelease(&voice.pathingSource);
                        mPathingSim->setSimulatorDirty();
                    } else {
                        iplSourceRelease(&voice.pathingSource);
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
    if (!voice.directSource && !voice.reflectionSource && !voice.pathingSource)
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

    // ── Pathing source ──
    // Same defer-flush dance as the reflection source below — the
    // pathing-sim worker may be mid-iteration and iplSourceRemove can't
    // safely race iplSimulatorRunPathing on the same handle.
    if (voice.pathingSource && mPathingSim) {
        // If this pathing source was deferred for add but never actually
        // added, just release it directly. (Mirror of the
        // removeFromPendingAdds check on the reflection side below — same
        // hazard: calling iplSourceRemove on a never-added source crashes
        // inside Steam Audio.)
        if (mPathingSim->removeFromPendingAdds(voice.pathingSource)) {
            iplSourceRelease(&voice.pathingSource);
        } else if (mPathingSim->isRunning()) {
            mPathingSim->queueSourceRemove(voice.pathingSource);
            voice.pathingSource = nullptr;
            mPathingSim->setSimulatorDirty();
        } else if (mPathingSim->simulator()) {
            iplSourceRemove(voice.pathingSource, mPathingSim->simulator());
            iplSourceRelease(&voice.pathingSource);
            mPathingSim->setSimulatorDirty();
        } else {
            iplSourceRelease(&voice.pathingSource);
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
    // Each sub-source slot owns its own pair of IPL effects so per-slot
    // HRTF interpolation + direct-effect smoothing stays attached to the
    // physical path mapped to that slot. In SA mode (probe_pathing: true)
    // only slot 0 is ever ACTIVE, so we allocate just one slot's worth;
    // in BFS mode (probe_pathing: false) we allocate all kMaxSubSources
    // for multi-portal fan-out. See PLAN.MULTI_PATH_SA_MIGRATION.md.
    //
    // Audio callback's per-slot loop skips slots with null binauralEffect
    // (SubSource defaults state=Free + null IPL handles), so a partial
    // allocation is safe. On any creation failure within the allocated
    // range we tear down the slots we've already built and bail out.
    const int slotsToAlloc = mProbePathingEnabled ? 1 : kMaxSubSources;
    IPLerror err = IPL_STATUS_SUCCESS;
    int      builtSlots = 0;
    for (int i = 0; i < slotsToAlloc; ++i) {
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

    // Create per-voice reflection effect. Gated on the convolution worker
    // pool being alive (post-Phase-3 the global mixer is gone — readiness is
    // signalled by mConvolutionPool->isActive() instead). The reflection
    // sample rate (24kHz or 48kHz) must match the simulator's.
    if (mConvolutionPool && mConvolutionPool->isActive()) {
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
            dsp.convWorker = mConvolutionPool->worker();
        } else {
            LOG_ERROR("AudioService: iplReflectionEffectCreate failed (error %d) "
                      "— direct effects only for this voice", err);
            // Continue without reflections for this voice
        }
    }

    // Phase 4: per-voice IPLPathEffect (Steam Audio sole authority for
    // player routing + attenuation). Created for every non-player-emitted
    // voice and consumed on the audio thread inside steamAudioNodeProcess
    // — its output is the routing-aware spatialized signal that REPLACES
    // the old `eqCoeffsToDspMapping` manual portal-attenuation + LPF stage.
    //
    // Settings match Valve's Unity reference integration:
    //   • spatialize = IPL_TRUE — output is HRTF-binaural stereo that
    //     mixes additively into the dry bus on top of the direct path
    //     binaural. Avoids a second ambisonics-decode stage and keeps the
    //     per-voice DSP graph linear.
    //   • maxOrder = mAmbisonicsOrder — bounds the SH-field-size memory
    //     up-front; the per-frame `IPLPathEffectParams.order` may be
    //     lower for cheaper CPU.
    //
    // Player-emitted voices skip this entirely — the player IS the
    // listener, the pathing graph collapses to a self-loop, and the
    // path effect would have nothing to route. removeVoiceSource /
    // ~ActiveVoice both null-check before releasing so this asymmetry
    // is safe.
    if (!voice.playerEmitted && mIplHrtf) {
        IPLPathEffectSettings pathSettings{};
        pathSettings.maxOrder       = mAmbisonicsOrder;
        pathSettings.spatialize     = IPL_TRUE;
        pathSettings.speakerLayout.type = IPL_SPEAKERLAYOUTTYPE_STEREO;
        pathSettings.hrtf           = mIplHrtf;

        IPLerror perr = iplPathEffectCreate(mIplContext, &audioSettings,
                                            &pathSettings, &dsp.pathEffect);
        if (perr != IPL_STATUS_SUCCESS) {
            LOG_ERROR("AudioService: iplPathEffectCreate failed (error %d) "
                      "— direct path only for this voice", perr);
            dsp.pathEffect = nullptr;
        } else {
            // Pre-size the per-voice path-effect output scratch (stereo
            // because spatialize=TRUE renders binaural). The audio thread
            // mixes these buffers additively into the voice's accumulated
            // stereoL/stereoR after the per-slot binaural pipeline. Sized
            // to mFrameSize × 2 channels — one allocation, never realloc'd.
            dsp.pathOutL.assign(mFrameSize, 0.0f);
            dsp.pathOutR.assign(mFrameSize, 0.0f);
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
        // Symmetric rollback for the path effect added in Phase 4.
        if (dsp.pathEffect) {
            iplPathEffectRelease(&dsp.pathEffect);
            dsp.pathEffect = nullptr;
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

    // Phase 4 diagnostic: prove at runtime that every voice (including
    // ambients that used to be pinned to `prop.virtualPosition` for the
    // cross-room case) now spawns with the IPL source position equal to
    // the real source. `realPos` and `pos` are identical here at spawn
    // — they diverged only inside the per-frame setInputs block on the
    // old code path. Rate-limited (first 16 + every 64th) so a level
    // with hundreds of ambients doesn't flood the log.
    {
        static std::atomic<int> sPlayerVoicePosLogCount{0};
        int n = sPlayerVoicePosLogCount.fetch_add(1, std::memory_order_relaxed);
        if (n < 16 || (n % 64) == 0) {
            std::fprintf(stdout,
                "[PLAYER_VOICE_POS] vid=%d obj=%d pos=(%.2f,%.2f,%.2f) "
                "realPos=(%.2f,%.2f,%.2f) schema='%s' [#%d]\n",
                voice->handle, objID,
                position.x, position.y, position.z,
                voice->worldPos.x, voice->worldPos.y, voice->worldPos.z,
                schemaName.c_str(), n + 1);
        }
    }

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
    // Chain / paths data here is BFS-driven (cachedProp), so the show_vpos
    // overlay's path visualisation is empty when probe_pathing is true (the
    // shipping default) and the BFS branch is gated off for player audio.
    // A Steam-Audio-driven path overlay is a separate future task.
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

    // ── [PERF] per-stage latency percentiles ──
    //
    // Snapshot+reset every histogram so each [PERF] line shows the
    // distribution over the just-finished 5 s window. Per-sub-worker
    // histograms are merged into a single bin-summed histogram before
    // computing percentiles — that gives the worker-pool aggregate
    // (apples-to-apples with "is the pool keeping up?"). Sub-worker
    // imbalance is observable via the existing scalar peakMs +
    // [CONV_LAG] log.
    //
    // Empty windows (n=0) emit n=0 and dashes — useful sentinel for
    // "this stage never ran" (e.g. no convolution voices = no apply/sum).
    auto dspP    = sPerfDspNodeMs.snapshotAndReset();
    auto mixP    = sPerfReflMixMs.snapshotAndReset();
    auto simP    = sPerfReflSimMs.snapshotAndReset();
    LatencyHistogram::Percentiles applyP{}, sumP{}, decP{}, upP{}, iterP{};
    // H2 per-iter aggregate histograms — see ConvolutionSubWorker for rationale.
    LatencyHistogram::Percentiles maxApplyP{}, sumApplyP{}, residualP{};
    // H2' signal→pickup latency (worker side).
    LatencyHistogram::Percentiles pickupP{};
    if (mConvolutionPool && mConvolutionPool->isActive()) {
        ConvolutionWorker *cw = mConvolutionPool->worker();
        // Snapshot each sub-worker histogram and accumulate bin counts
        // into temp histograms. Use record() N times would be O(n) per
        // bucket — but n can be large, so we cheat by recording a single
        // representative sample per bucket. Cleaner: a dedicated "merge"
        // helper, but for the [PERF] line we just want one set of pcts
        // per stage. Approach: pick the worker with the largest sample
        // count and use its percentiles as the pool representative,
        // accumulating total `n` across all workers for context.
        // (Per-worker breakdown still available via existing peakMs.)
        for (auto &subPtr : cw->workers) {
            auto a = subPtr->perfApplyMs.snapshotAndReset();
            auto s = subPtr->perfSumMs.snapshotAndReset();
            auto d = subPtr->perfDecodeMs.snapshotAndReset();
            auto u = subPtr->perfUpsampleMs.snapshotAndReset();
            auto it = subPtr->perfIterMs.snapshotAndReset();
            auto mx = subPtr->perfMaxApplyMs.snapshotAndReset();
            auto sa = subPtr->perfSumApplyMs.snapshotAndReset();
            auto rs = subPtr->perfResidualMs.snapshotAndReset();
            auto pk = subPtr->perfSignalPickupMs.snapshotAndReset();
            // Take the percentile values of the worker with the most
            // samples — most representative of pool behaviour under load.
            if (a.n   > applyP.n)     applyP    = a;
            if (s.n   > sumP.n)       sumP      = s;
            if (d.n   > decP.n)       decP      = d;
            if (u.n   > upP.n)        upP       = u;
            if (it.n  > iterP.n)      iterP     = it;
            if (mx.n  > maxApplyP.n)  maxApplyP = mx;
            if (sa.n  > sumApplyP.n)  sumApplyP = sa;
            if (rs.n  > residualP.n)  residualP = rs;
            if (pk.n  > pickupP.n)    pickupP   = pk;
        }
    }
    // H1' inter-callback period (one writer = audio thread).
    auto interCbP = sPerfInterCallbackMs.snapshotAndReset();
    auto fmtMs = [](double v) -> double { return v; };  // identity, leave for future tuning
    AUDIO_LOG(
        "[PERF audio] dsp_node n=%llu p50/p95/p99=%.3f/%.3f/%.3f ms"
        " | refl_mix n=%llu p50/p95/p99=%.3f/%.3f/%.3f ms"
        " | refl_sim n=%llu p50/p95/p99=%.1f/%.1f/%.1f ms\n",
        (unsigned long long)dspP.n, fmtMs(dspP.p50), fmtMs(dspP.p95), fmtMs(dspP.p99),
        (unsigned long long)mixP.n, fmtMs(mixP.p50), fmtMs(mixP.p95), fmtMs(mixP.p99),
        (unsigned long long)simP.n, fmtMs(simP.p50), fmtMs(simP.p95), fmtMs(simP.p99));
    AUDIO_LOG(
        "[PERF worker] apply n=%llu p50/p95/p99=%.3f/%.3f/%.3f ms"
        " | sum n=%llu p50/p95/p99=%.3f/%.3f/%.3f ms"
        " | decode n=%llu p50/p95/p99=%.3f/%.3f/%.3f ms"
        " | upsample n=%llu p50/p95/p99=%.3f/%.3f/%.3f ms"
        " | iter n=%llu p50/p95/p99=%.3f/%.3f/%.3f ms\n",
        (unsigned long long)applyP.n, fmtMs(applyP.p50), fmtMs(applyP.p95), fmtMs(applyP.p99),
        (unsigned long long)sumP.n,   fmtMs(sumP.p50),   fmtMs(sumP.p95),   fmtMs(sumP.p99),
        (unsigned long long)decP.n,   fmtMs(decP.p50),   fmtMs(decP.p95),   fmtMs(decP.p99),
        (unsigned long long)upP.n,    fmtMs(upP.p50),    fmtMs(upP.p95),    fmtMs(upP.p99),
        (unsigned long long)iterP.n,  fmtMs(iterP.p50),  fmtMs(iterP.p95),  fmtMs(iterP.p99));

    // H2 per-iter aggregate distribution (apply-distribution + residual).
    //
    // sum_apply  — convolution work the worker actually does per iter.
    //              Compare against iter p99: if sum_apply ≈ iter, DSP is
    //              the whole cost. If sum_apply ≪ iter, look at residual.
    // max_apply  — worst single apply() call within an iter. If max ≈
    //              sum_apply, one expensive voice is the bottleneck.
    //              If max ≪ sum_apply / assignCount, costs are balanced
    //              — attack per-voice cost (rate_divisor, order) not
    //              assignment policy.
    // residual   — iter − (sumApply + sumStage + decode + upsample). All
    //              non-DSP time: memset, atomics, buffer flip, scheduler
    //              preemption. residual p99 ≫ p50 = thread scheduling
    //              jitter (H1) or contention; residual p50 large = static
    //              overhead (mempset/atomics) we can attack.
    AUDIO_LOG(
        "[PERF worker_iter] sum_apply n=%llu p50/p95/p99=%.3f/%.3f/%.3f ms"
        " | max_apply n=%llu p50/p95/p99=%.3f/%.3f/%.3f ms"
        " | residual n=%llu p50/p95/p99=%.3f/%.3f/%.3f ms\n",
        (unsigned long long)sumApplyP.n, fmtMs(sumApplyP.p50), fmtMs(sumApplyP.p95), fmtMs(sumApplyP.p99),
        (unsigned long long)maxApplyP.n, fmtMs(maxApplyP.p50), fmtMs(maxApplyP.p95), fmtMs(maxApplyP.p99),
        (unsigned long long)residualP.n, fmtMs(residualP.p50), fmtMs(residualP.p95), fmtMs(residualP.p99));

    // H1' / H2' — signal pipeline timing.
    //
    // inter_cb    — wall-clock between consecutive audio callbacks.
    //               Nominal at 1024 @ 48 kHz: 21.333 ms. If p99 ≪ p50,
    //               CoreAudio is firing some callbacks early (e.g. two
    //               in quick succession) and no worker can possibly
    //               advance between them — that would explain the
    //               all-4-stale-or-none pattern in [WET_BUS].
    //
    // signal_pickup — wall-clock from `frameSeq++` on the mix node side
    //               to the worker's observation of the new seq on its
    //               next yield-poll cycle. Lower bound on how long a
    //               worker spent unscheduled (or yield-pollling) before
    //               picking up its work. Large p99 here = the
    //               yield()-based wake path is the bottleneck → look at
    //               replacing it with a condvar / semaphore.
    AUDIO_LOG(
        "[PERF timing] inter_cb n=%llu p50/p95/p99=%.3f/%.3f/%.3f ms"
        " | signal_pickup n=%llu p50/p95/p99=%.3f/%.3f/%.3f ms\n",
        (unsigned long long)interCbP.n, fmtMs(interCbP.p50), fmtMs(interCbP.p95), fmtMs(interCbP.p99),
        (unsigned long long)pickupP.n,  fmtMs(pickupP.p50),  fmtMs(pickupP.p95),  fmtMs(pickupP.p99));

    // sync_wait — wall-clock time the audio thread spent at the top of
    // the reflection mix node waiting for sub-workers to finish processing
    // the CURRENT callback's mono input (sync-in-callback). p50 ≈ longest
    // worker iter time + signal/pickup overhead. Compare against
    // node->syncDeadlineMs (default 70 % of callback period) — p99
    // approaching the deadline means the pool is close to falling back
    // to last-frame's wet, which would reintroduce the 1-callback gap.
    // timeouts: number of callbacks in the window where the wait expired
    // without all workers reaching their target.
    {
        auto syncP = sPerfSyncWaitMs.snapshotAndReset();
        int timeouts = sSyncTimeoutCount.exchange(0, std::memory_order_relaxed);
        float deadlineMs = (mReflectionMixNode)
            ? mReflectionMixNode->syncDeadlineMs : 0.0f;
        AUDIO_LOG(
            "[PERF sync_wait] n=%llu p50/p95/p99=%.3f/%.3f/%.3f ms"
            " | timeouts=%d (deadline=%.2f ms)\n",
            (unsigned long long)syncP.n,
            fmtMs(syncP.p50), fmtMs(syncP.p95), fmtMs(syncP.p99),
            timeouts, static_cast<double>(deadlineMs));
    }

    // ── [BEAT] wet-bus autocorrelation ──
    //
    // The detector pushed one envelope sample per reflection-mix-node
    // callback, so `frameRateHz` here = device-rate / device-frame-size.
    // Scan the 1–5 Hz lag band for periodic structure; emit a single
    // line with the peak autocorrelation, its lag, the corresponding
    // frequency, and a YES/NO flag against the threshold.
    //
    // YES: wet bus has a rhythmic amplitude modulation in the
    // beating band. Triage:
    //   • If acFreqHz lands at 1/reflectionThrottle  → per-frame IR
    //     crossfade stack (original bug — hybrid should suppress)
    //   • If acFreqHz lands at 1/footstep_cadence    → expected, ignore
    //   • If acFreqHz lands at simRate/throttle      → sim cycle leakage
    // NO: no audible amplitude rhythm; either silent or random.
    {
        const float envFrameRateHz =
            (mFrameSize > 0)
                ? static_cast<float>(mDeviceSampleRate) / static_cast<float>(mFrameSize)
                : 46.875f;
        auto beat = sWetBeat.analyze(envFrameRateHz);
        AUDIO_LOG(
            "[BEAT] env_mean=%.5f env_rms=%.5f ac_peak=%.3f ac_lag=%.3fs ac_freq=%.2fHz "
            "n=%d beating=%s\n",
            beat.envMean, beat.envRMS, beat.acPeak, beat.acLagSec, beat.acFreqHz,
            beat.samples, beat.beating ? "YES" : "no");
    }
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

void AudioService::setProbeMinWallClearanceFt(float ft)
{
    // Negative or NaN → treat as disabled (0). Upper clamp guards
    // against a YAML typo (e.g. metres mistaken for feet) that would
    // reject every probe in the level.
    if (!(ft >= 0.0f)) ft = 0.0f;
    if (ft > 50.0f)    ft = 50.0f;
    mProbeMinWallClearanceFt = ft;
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
    params.simulatorThreads      = mSimulatorThreadCount;
    params.ambisonicsOrder       = mBakeAmbisonicsOrder;
    params.sceneType             = mSceneTypeCfg;
    params.spacingFtOverride     = spacing;
    params.heightFtOverride      = height;
    params.additionalElevations  = mProbeElevations;
    params.elevationSparsityMul  = mProbeElevationSparsityMul;
    params.globalDedupRadiusFt   = mProbeGlobalDedupRadiusFt;

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

    // Validity filter applied to every probe candidate. Two rejection
    // criteria, both producing the same outcome (probe is dropped from
    // the batch before any reflection rays are cast):
    //
    //   1. No containing room → the candidate sits in BSP void or
    //      inside solid geometry. Steam Audio's UNIFORMFLOOR generation
    //      raycasts down to find a floor, which mostly avoids underground
    //      placements, but elevation tiers and portal anchors don't get
    //      that guarantee. roomFromPoint is the cheapest reliable
    //      "inside the level" oracle we have.
    //
    //   2. Within `minClearanceFt` of the nearest room wall. Wall-
    //      adjacent probes capture a single dominant first-bounce
    //      reflection at t = 2·distance/c, which comb-filters input
    //      audio at multiples of c/(2·distance). Pulling probes inward
    //      from walls trades a small coverage gap near room edges for
    //      far better-conditioned IRs everywhere else. We approximate
    //      "distance to nearest wall" with min(positive distance over
    //      the 6 room bounding planes) — exact for box-shaped rooms,
    //      a slight over-estimate for L-shaped ones. Good enough for a
    //      bake-time filter; if we ever need true per-cell clearance we
    //      can swap in CollisionGeometry::sphereVsCellPolygons here.
    //
    // Filter is constructed inline so it captures live tuning state.
    // Empty lambda = legacy behaviour (every candidate accepted).
    const float minClearanceFt = mProbeMinWallClearanceFt;
    if (mRoomService) {
        params.minWallClearanceFt = minClearanceFt;
        params.probeFilter =
            [this, minClearanceFt](const Vector3 &p) -> ProbeFilterDecision {
                ProbeFilterDecision d;
                Room *r = mRoomService->roomFromPoint(p);
                if (!r) {                              // (1) inside solid / void
                    d.result = ProbeFilterResult::Reject;
                    return d;
                }
                if (minClearanceFt <= 0.0f) {          // clearance disabled
                    d.result = ProbeFilterResult::Accept;
                    return d;
                }
                // (2) Lateral wall clearance only. Planes face inward,
                //     so distance > 0 = inside; smallest positive
                //     distance over the VERTICAL planes is the wall
                //     clearance. See ProbeManager.h for the per-result
                //     semantics; Reject = no recovery, Nudge = displace
                //     by `nudgeDir × nudgeDistFt` and re-evaluate.
                //
                //     Floor/ceiling planes (|normal.z| > 0.5) are
                //     excluded by design:
                //       - The probe sits a fixed `height` above the
                //         floor (default 5 ft = the default clearance).
                //         Including the floor plane would place every
                //         probe exactly at the threshold, where float-
                //         precision jitter rejects half of them.
                //       - Outdoor rooftop / balcony rooms often have an
                //         audio-ceiling plane only a few feet above the
                //         floor (it bounds the audio room, not the
                //         physical sky). Including it would reject
                //         every probe in those rooms even though they
                //         sit in wide-open space.
                //     The 0.5 threshold = 30° off vertical; Dark Engine
                //     room OBBs are axis-aligned, so |normal.z| is
                //     either ~0 (wall) or ~1 (floor/ceiling) — the cut
                //     is unambiguous in practice.
                const Plane *planes = r->getBoundingPlanes();
                float minDist = 1e9f;
                int   minIdx  = -1;
                int   wallPlanes = 0;
                for (int i = 0; i < 6; ++i) {
                    if (std::abs(planes[i].normal.z) > 0.5f) continue;
                    float dist = planes[i].getDistance(p);
                    if (dist < minDist) { minDist = dist; minIdx = i; }
                    ++wallPlanes;
                }
                // Degenerate case: room has no vertical walls (rare —
                // possibly a malformed top/bottom-only bounds). Without
                // walls to be adjacent to, accept the probe.
                if (wallPlanes == 0 || minIdx < 0) {
                    d.result = ProbeFilterResult::Accept;
                    return d;
                }
                if (minDist >= minClearanceFt) {
                    d.result = ProbeFilterResult::Accept;
                    return d;
                }
                // Too close to a wall — nudge inward along the wall's
                // inward normal by the missing clearance. ProbeManager
                // adds a fixed 0.5 ft margin and re-evaluates.
                d.result      = ProbeFilterResult::Nudge;
                d.nudgeDir    = planes[minIdx].normal;
                d.nudgeDistFt = minClearanceFt - minDist;
                return d;
            };
    } else {
        // No RoomService = no way to validate anything; let everything
        // through and log so the operator notices. This is the same
        // posture classifyProbeReachability takes below.
        AUDIO_LOG("[FALLBACK] bakeProbes: no RoomService — probe filter "
                  "disabled, every candidate will be baked\n");
    }

    bool ok = mProbeManager->bakeProbes(mIplScene, outputPath, params, progress);
    if (ok) {
        // Classify the freshly-baked probes so the overlay can show
        // residual bad ones (e.g. unreachable disconnected rooms — those
        // get past the filter because roomFromPoint returns a valid
        // room, just one no playable seed reaches). With the filter
        // active on a fresh bake this should be a small minority; if it
        // isn't, the filter or seed-room logic needs another look.
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
    // If a previous probe batch is still attached to the pathing simulator,
    // detach it BEFORE ProbeManager swaps in a new batch. ProbeManager's
    // loadProbes only knows about the reflection simulator, so its release
    // path won't touch the pathing sim; leaving the old batch attached
    // produces a dangling reference once ProbeManager releases its
    // create-time ref. Symmetric with destroyAcousticScene's teardown.
    //
    // The pathing-sim worker must be drained before mutating its probe
    // batch list — iplSimulatorRemoveProbeBatch is not safe to race
    // iplSimulatorRunPathing on the same handle.
    if (mPathingProbeBatchAdded && mPathingSim && mPathingSim->simulator() && mProbeManager) {
        IPLProbeBatch oldBatch = mProbeManager->getProbeBatch();
        if (oldBatch) {
            mPathingSim->waitForCompletion();
            iplSimulatorRemoveProbeBatch(mPathingSim->simulator(), oldBatch);
            iplSimulatorCommit(mPathingSim->simulator());
            iplProbeBatchRelease(&oldBatch);  // drops our retain
        }
        mPathingProbeBatchAdded = false;
    }

    bool ok = mProbeManager->loadProbes(probePath,
        mReflectionSim ? mReflectionSim->simulator() : nullptr);
    if (!ok) return false;

    // Attach the freshly loaded probe batch to the pathing simulator
    // too, so Steam Audio's pathing solver can walk the probe graph from
    // each voice's source to the listener. Retain so ProbeManager's
    // release path on the reflection simulator side doesn't free the
    // batch out from under us.
    if (mPathingSim && mPathingSim->simulator()) {
        IPLProbeBatch batch = mProbeManager->getProbeBatch();
        if (batch) {
            // Drain the worker before mutating the probe-batch list (same
            // reason as the detach above).
            mPathingSim->waitForCompletion();
            iplProbeBatchRetain(batch);
            iplSimulatorAddProbeBatch(mPathingSim->simulator(), batch);
            iplSimulatorCommit(mPathingSim->simulator());
            mPathingProbeBatchAdded = true;
            AUDIO_LOG("AudioService: attached probe batch (%d probes) to "
                      "pathing simulator for pathing\n",
                      mProbeManager->getProbeCount());
        }
    }
    // Classify reachability so the debug overlay can preview which
    // probes the bake-time filter would now reject. With the filter
    // active on a fresh bake the bad count should be ~0 (only the
    // reachability check still finds disconnected components, and those
    // are rare). If we load an OLD .probes file baked before the filter
    // existed, the bad count can be a large fraction of total — warn
    // the operator so they know a re-bake will clean things up.
    size_t bad = classifyProbeReachability();
    const auto &positionsForCheck = mProbeManager->getProbePositions();
    if (!positionsForCheck.empty()) {
        const float badFrac = static_cast<float>(bad)
                            / static_cast<float>(positionsForCheck.size());
        if (badFrac > 0.25f) {
            std::fprintf(stderr,
                "[STALE_PROBES] '%s' contains %zu/%zu (%.0f%%) probes "
                "that the current filter would reject (inside solid or "
                "wall-adjacent). The file was likely baked before "
                "audio.probes.min_wall_clearance_ft was set. Re-bake from "
                "the debug console (`bake_probes`) for clean reverb — "
                "voices snapping to bad probes may sound dropout-prone "
                "or comb-filtered until you do.\n",
                probePath.c_str(), bad, positionsForCheck.size(),
                100.0f * badFrac);
        }
    }
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
