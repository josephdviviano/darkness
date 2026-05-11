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
#include "AudioLog.h"
#include "AcousticMaterials.h"
#include "CRFSoundLoader.h"
#include "ProbeFile.h"
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
/*----------- Steam Audio DSP Processing Node --------*/
/*----------------------------------------------------*/

/// Forward declare the process callback for the custom miniaudio node vtable.
static void steamAudioNodeProcess(ma_node* pNode, const float** ppFramesIn,
                                   ma_uint32* pFrameCountIn,
                                   float** ppFramesOut, ma_uint32* pFrameCountOut);

// Forward declaration (full definition below, after ReflectionMixNode)
struct ConvolutionWorker;

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
    // ── Footstep reflection diagnostic ──
    // Set true at startVoice time for "foot_*" / "land_*" schemas. The audio
    // and main threads both gate one-shot diagnostic logs on this so we can
    // confirm whether transient sounds reach the convolution path. Cleared
    // when the voice is destroyed.
    bool isFootstepDiag = false;
    std::atomic<int> reflInputLogCount{0};  // audio thread: limit log spam to first few frames
    float portalAttenuation = 0.0f;      // inverse-square from effective distance
    float portalBlocking = 0.0f;         // 0.0=open, 1.0=fully blocked (for LPF)
    IPLVector3 portalDirection{1.0f, 0.0f, 0.0f}; // direction toward virtual source (portal center)

    // Per-voice low-pass filter state for door blocking (audio thread only).
    // Simulates high-frequency absorption through closed doors.
    float lpfStateL = 0.0f;
    float lpfStateR = 0.0f;

    // Reflection convolution effect (per-voice, feeds into shared mixer)
    IPLReflectionEffect reflectionEffect = nullptr;

    // Shared reference to the reflection mixer (NOT owned — AudioService manages)
    // Only used if convolution worker is not active (on-thread fallback).
    IPLReflectionMixer reflectionMixer = nullptr;

    // Pointer to the off-thread convolution worker (set during initVoiceDSP)
    ConvolutionWorker *convWorker = nullptr;

    // Reflection simulation output params (written by main thread from simulator)
    IPLReflectionEffectParams reflectionParams{};
    std::atomic<bool> reflectionsActive{false};

    // Scratch buffers for deinterleaved Steam Audio processing
    // (allocated once at init, never reallocated — safe for audio thread)
    std::vector<float> monoScratch;   // mono channel (raw downmix, preserved for convolution)
    std::vector<float> directEffectOut; // mono output from iplDirectEffectApply (filtered)
    std::vector<float> stereoL;       // left channel (binaural output)
    std::vector<float> stereoR;       // right channel (binaural output)

    // Per-voice ambisonics scratch for reflection convolution output
    // (required by iplReflectionEffectApply even when using mixer accumulation)
    std::vector<float> ambiScratch0;  // W channel
    std::vector<float> ambiScratch1;  // Y channel
    std::vector<float> ambiScratch2;  // Z channel
    std::vector<float> ambiScratch3;  // X channel

    // Decimation scratch (used when reflection pipeline runs at reduced rate)
    std::vector<float> decimatedMono;

    int frameSize = 1024;
    int reflectionFrameSize = 1024;  // frameSize / rateDivisor
    int rateDivisor = 1;  // 1=full, 2=half, 4=quarter
    std::atomic<bool> effectsReady{false};   // true when effects + node are initialized
    bool nodeInitialized = false;            // true when ma_node_init succeeded (main thread only)

    // Shared validity token for the convolution worker. Heap-allocated and
    // reference-counted so the worker can safely check it even after the
    // ActiveVoice that created it has been destroyed. The audio callback
    // copies the shared_ptr into the staging slot; ~ActiveVoice sets the
    // bool to false but does NOT need to wait for the worker to see it —
    // the shared_ptr prevents use-after-free of the token itself.
    std::shared_ptr<std::atomic<bool>> validityToken;

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

/// Per-thread convolution sub-worker. Each sub-worker owns its own Steam Audio
/// mixer/decoder pipeline and processes a subset of voice slots assigned by
/// the mix node callback. Completely independent — no shared mutable state
/// between sub-workers during processing.
struct ConvolutionSubWorker {
    // Own Steam Audio pipeline (not thread-safe — one per worker)
    IPLReflectionMixer mixer = nullptr;
    IPLAmbisonicsDecodeEffect ambiDecodeEffect = nullptr;

    // Own scratch buffers for ambisonics processing
    std::vector<float> ambiCh0, ambiCh1, ambiCh2, ambiCh3;
    std::vector<float> voiceAmbi0, voiceAmbi1, voiceAmbi2, voiceAmbi3;
    std::vector<float> decodedL, decodedR;

    // Own double-buffered stereo output (deinterleaved)
    std::vector<float> stereoL[2], stereoR[2];
    std::atomic<int> frontIdx{0};
    std::atomic<bool> hasProducedOutput{false};

    // Assignment: indices into shared staging[readBuf].
    // Written by mix node BEFORE signal, read by worker AFTER signal.
    // The atomic frameSeq provides the acquire-release barrier.
    int assignedSlots[MAX_ACTIVE_VOICES];
    int assignedCount = 0;

    // Thread control
    std::thread thread;
    std::atomic<uint64_t> frameSeq{0};       // incremented by mix node to signal new data
    std::atomic<uint64_t> processedSeq{0};   // last frameSeq the worker finished
    std::atomic<bool> shutdown{false};
    std::atomic<float> peakMs{0.0f};
};

/// Off-thread convolution worker with K parallel sub-workers.
/// The audio callback writes voice mono snapshots to shared staging buffers.
/// The mix node callback distributes voice slots across sub-workers (round-robin)
/// and signals them. Each sub-worker processes its subset independently and
/// writes to its own double-buffered stereo output. The mix node reads and
/// sums all sub-worker outputs on the next callback. No barrier sync needed.
struct ConvolutionWorker {
    // Per-voice mono input snapshots (filled by audio thread, read by sub-workers)
    struct VoiceSlot {
        std::vector<float> mono;          // processed mono data (post-distance-atten, post-decimate)
        IPLReflectionEffect effect = nullptr;
        IPLReflectionEffectParams params{};
        // Shared validity token — reference-counted so workers can safely
        // dereference it even after the owning ActiveVoice has been destroyed.
        std::shared_ptr<std::atomic<bool>> validityToken;
        int reflFrameSize = 0;
        bool active = false;
        // Diagnostic: tag forwarded from the voice's DSP node so the worker
        // can log the wet-output peak only for footstep voices, keeping the
        // log focused.
        bool isFootstepDiag = false;
    };
    static constexpr int kMaxSlots = MAX_ACTIVE_VOICES;

    // Double-buffered voice slots: audio thread writes to staging[writeIdx],
    // sub-workers read from staging[readBuf]. Swapped atomically via writeIdx.
    VoiceSlot staging[2][kMaxSlots];
    int stagingCount[2] = {0, 0};        // count per buffer
    std::atomic<int> writeIdx{0};        // audio thread writes to this index
    std::atomic<int> readyCount{0};      // count of the buffer just completed

    // Track how many sub-workers are still reading the staging buffer.
    // Mix node checks this before swapping — if > 0, drops the frame.
    std::atomic<int> workersReading{0};

    // Which staging buffer sub-workers should read (plain int — visibility
    // guaranteed by the acquire-release on sub-worker frameSeq signals).
    int currentReadBuf = -1;

    // Sub-workers (K parallel threads, each with own mixer/decoder/output).
    // unique_ptr because ConvolutionSubWorker contains std::atomic (non-movable).
    std::vector<std::unique_ptr<ConvolutionSubWorker>> workers;
    int numWorkers = 1;

    // Shared read-only state (set at init, never modified during processing)
    IPLHRTF hrtf = nullptr;
    IPLContext context = nullptr;
    int frameSize = 1024;
    int reflectionFrameSize = 1024;
    int ambiChannels = 1;
    int ambiOrder = 0;
    int rateDivisor = 2;  // 1=full, 2=half, 4=quarter

    // Listener orientation snapshot (written by mix node, read by all workers)
    IPLCoordinateSpace3 listenerOrientation = {
        {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f}
    };
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

// ── Audio-thread tunables ──
// File-scope atomics published by AudioService setters and read on the audio
// thread. These belong to per-voice processing paths that don't have direct
// access to AudioService members; using atomics avoids data races even though
// the values are typically only set once at startup from RenderConfig.
//   sHrtfInterpolation: 0 = nearest, 1 = bilinear
//   sDistanceModel:     0 = default, 1 = inverse_distance
static std::atomic<int>   sHrtfInterpolation{1};
static std::atomic<float> sSpatialBlend{1.0f};
static std::atomic<float> sDoorLpfOpenHz{20000.0f};
static std::atomic<float> sDoorLpfBlockedHz{800.0f};
static std::atomic<float> sPropMinAttenuation{0.001f};
static std::atomic<int>   sDistanceModel{0};
/// Engine sample rate published for audio-thread DSP that needs it (door LPF, etc.)
static std::atomic<uint32_t> sEngineSampleRate{48000};

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
    // Architecture: apply IPLDirectEffect (frequency-dependent 3-band attenuation
    // for distance, air absorption, occlusion, transmission) to mono, then run
    // binaural HRTF for spatialization. Portal routing is a separate post-HRTF
    // scalar multiply (geometric, frequency-independent).
    //
    // Level 0: direct effect + binaural (full pipeline)
    // Level 1: direct attenuation only (no binaural — mono passthrough)
    // Level 2: binaural only (no direct attenuation)
    bool runBinaural = (bypassLevel == 0 || bypassLevel == 2) && node->binauralEffect;
    bool runAtten = (bypassLevel == 0 || bypassLevel == 1);

    // Apply Steam Audio direct effect — frequency-dependent 3-band EQ covering
    // distance attenuation, air absorption, occlusion, and wall transmission.
    // Processes mono input to a separate output buffer so the raw mono stays
    // pristine for convolution staging (reverb uses unfiltered mono × distance).
    float* binauralMono = mono;  // default: feed raw mono to HRTF
    if (runAtten && !node->skipAttenuation && node->directEffect) {
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

        iplDirectEffectApply(node->directEffect, &node->directParams,
                             &directIn, &directOut);
        binauralMono = directOutPtr;  // feed filtered mono to HRTF

        // Store scalar attenuation estimate for debug logging and
        // reflection voice distance ranking (broadband approximation)
        float atten = node->directParams.distanceAttenuation
                    * node->directParams.occlusion;
        if (node->usePortalRouting) {
            atten *= node->portalAttenuation;
        }
        float minAtten = sPropMinAttenuation.load(std::memory_order_relaxed);
        if (atten < minAtten) atten = minAtten;
        node->lastAtten.store(atten, std::memory_order_relaxed);
    }

    // Apply binaural effect (mono → stereo HRTF spatialization).
    // Uses filtered mono (post-direct-effect) so frequency-dependent
    // attenuation is spatialized correctly to the listener's head.
    if (runBinaural) {
        float* binauralMonoPtr = binauralMono;
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
        binParams.interpolation = (sHrtfInterpolation.load(std::memory_order_relaxed) == 0)
                                  ? IPL_HRTFINTERPOLATION_NEAREST
                                  : IPL_HRTFINTERPOLATION_BILINEAR;
        binParams.spatialBlend = sSpatialBlend.load(std::memory_order_relaxed);
        binParams.hrtf = node->hrtf;
        binParams.peakDelays = nullptr;

        iplBinauralEffectApply(node->binauralEffect, &binParams, &binauralIn, &outBuf);

        // Apply portal routing — geometric distance through doorway chain.
        // This is frequency-independent (not acoustic material filtering)
        // and separate from the direct effect applied above.
        if (runAtten && !node->skipAttenuation && node->usePortalRouting) {
            float portalScale = node->portalAttenuation;
            float minAtten = sPropMinAttenuation.load(std::memory_order_relaxed);
            if (portalScale < minAtten) portalScale = minAtten;
            for (ma_uint32 i = 0; i < frameCount; ++i) {
                chL[i] *= portalScale;
                chR[i] *= portalScale;
            }

            // Low-pass filter for door blocking — closed doors absorb high
            // frequencies more than low frequencies, producing a muffled
            // quality. Uses a 1-pole IIR LPF whose cutoff decreases with
            // increasing blocking factor along an exponential map between
            // open and blocked cutoffs (config-driven; defaults 20kHz→800Hz).
            float blocking = node->portalBlocking;
            if (blocking > 0.01f) {
                float openHz    = sDoorLpfOpenHz.load(std::memory_order_relaxed);
                float blockedHz = sDoorLpfBlockedHz.load(std::memory_order_relaxed);
                float cutoff = openHz * std::pow(blockedHz / openHz, blocking);
                float sampleRate = static_cast<float>(
                    sEngineSampleRate.load(std::memory_order_relaxed));
                float rc = 1.0f / (2.0f * 3.14159265f * cutoff);
                float dt = 1.0f / sampleRate;
                float alpha = dt / (rc + dt);

                for (ma_uint32 i = 0; i < frameCount; ++i) {
                    node->lpfStateL += alpha * (chL[i] - node->lpfStateL);
                    chL[i] = node->lpfStateL;
                    node->lpfStateR += alpha * (chR[i] - node->lpfStateR);
                    chR[i] = node->lpfStateR;
                }
            }
        }

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

                // Apply distance attenuation to mono (no occlusion — reflected
                // paths are independent of direct-path wall blocking)
                float reflAtten = node->directParams.distanceAttenuation;

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
                        dec[i] = sum * invDiv * reflAtten;
                    }
                    for (ma_uint32 i = outFrames; i < reflFrames; ++i)
                        dec[i] = 0.0f;
                    std::memcpy(slot.mono.data(), dec, reflFrames * sizeof(float));
                } else {
                    // Copy mono with distance attenuation applied
                    for (ma_uint32 i = 0; i < std::min(frameCount, reflFrames); ++i)
                        slot.mono[i] = mono[i] * reflAtten;
                    for (ma_uint32 i = frameCount; i < reflFrames; ++i)
                        slot.mono[i] = 0.0f;
                }

                slot.effect = node->reflectionEffect;
                slot.validityToken = node->validityToken;  // shared_ptr copy — prevents use-after-free
                slot.params = node->reflectionParams;
                slot.reflFrameSize = node->reflectionFrameSize;
                slot.active = true;
                slot.isFootstepDiag = node->isFootstepDiag;  // propagate for worker log
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
        // No binaural — duplicate filtered mono to both channels.
        // Direct effect was already applied above (binauralMono points to
        // filtered output if attenuation is active).
        float peakOut = 0.0f;
        for (ma_uint32 i = 0; i < frameCount; ++i) {
            float s = binauralMono[i];
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

    // If reflection pipeline not ready, just pass through.
    if (!node->ready || !node->simulationRan) {
        pFrameCountIn[0] = frameCount;
        *pFrameCountOut = frameCount;
        return;
    }

    // Read all sub-workers' front buffers (previous frame's decoded reverb)
    // and sum into the output. Each sub-worker double-buffers independently.
    ConvolutionWorker *cw = node->convWorker;
    float wetPeakL = 0.0f;  // diagnostic: peak |wet stereo| this block
    float wetPeakR = 0.0f;
    int   wetWorkersHit = 0;  // diagnostic: how many sub-workers actually contributed
    if (cw) {
        ma_uint32 outSamples = std::min(static_cast<ma_uint32>(cw->frameSize), frameCount);
        for (auto &subPtr : cw->workers) {
            auto &sub = *subPtr;
            if (!sub.hasProducedOutput.load(std::memory_order_acquire)) continue;
            ++wetWorkersHit;
            int front = sub.frontIdx.load(std::memory_order_acquire);

            // Additive mix with gain ramp
            for (ma_uint32 i = 0; i < outSamples; ++i) {
                if (node->reflGain < node->reflGainTarget)
                    node->reflGain = std::min(node->reflGain + node->reflRampRate, node->reflGainTarget);
                float g = node->reflGain;
                float l = sub.stereoL[front][i] * g;
                float r = sub.stereoR[front][i] * g;
                stereoOut[i * 2]     += l;
                stereoOut[i * 2 + 1] += r;
                wetPeakL = std::max(wetPeakL, std::fabs(l));
                wetPeakR = std::max(wetPeakR, std::fabs(r));
            }
        }
    }
    // Diagnostic: log the summed wet bus peak ~once per second so we can see
    // whether reverb is actually reaching the output. Independent of which
    // voices are convolving — if wet peaks are non-zero, reverb exists in the
    // mix; if zero or near-zero, no reverb is being produced (workers behind,
    // IRs empty, or all voices below floor).
    {
        static std::atomic<int> sWetLogTick{0};
        int tick = sWetLogTick.fetch_add(1, std::memory_order_relaxed);
        if ((tick & 0x3F) == 0) {  // every ~64 callbacks ≈ 1.4s @ 1024@48k
            AUDIO_LOG("[WET_BUS] peakL=%.4f peakR=%.4f workersHit=%d/3 reflGain=%.2f\n",
                      wetPeakL, wetPeakR, wetWorkersHit, node->reflGain);
        }
    }

    // Distribute voice slots to sub-workers and signal them.
    // Snapshot the listener orientation for ambisonics decode.
    if (cw) {
        cw->listenerOrientation = node->listenerOrientation;

        // Swap staging buffers: the write buffer (just filled by voice callbacks)
        // becomes the read buffer for sub-workers. Start a fresh write buffer.
        // Guard: if ANY sub-worker is still reading, skip the swap and drop
        // this frame's staging data to prevent corruption.
        int w = cw->writeIdx.load(std::memory_order_relaxed);
        int newW = 1 - w;
        if (cw->workersReading.load(std::memory_order_acquire) > 0) {
            // Sub-workers still reading — drop this frame.
            // Diagnostic: count how often we drop and how many voices were
            // sacrificed. Logged once per ~5 s alongside the perf summary.
            static std::atomic<int> sFrameDropCount{0};
            static std::atomic<int> sVoicesDroppedTotal{0};
            sFrameDropCount.fetch_add(1, std::memory_order_relaxed);
            sVoicesDroppedTotal.fetch_add(cw->stagingCount[w],
                                           std::memory_order_relaxed);
            // Periodic dump (cheap — only fires every 256 drops).
            int dc = sFrameDropCount.load(std::memory_order_relaxed);
            if ((dc & 0xFF) == 0) {
                int vd = sVoicesDroppedTotal.load(std::memory_order_relaxed);
                AUDIO_LOG("[CONV_DROP] frames=%d voices_lost=%d (workers behind)\n",
                          dc, vd);
            }
            cw->stagingCount[w] = 0;
        } else {
            int count = cw->stagingCount[w];
            cw->readyCount.store(count, std::memory_order_relaxed);
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

            // Set which staging buffer sub-workers should read.
            // This write is visible to workers via the frameSeq acquire barrier.
            cw->currentReadBuf = w;  // the buffer we just swapped away from

            // Pre-set workersReading BEFORE signaling to close the TOCTOU gap.
            // Without this, a sub-worker could wake from the signal but not yet
            // increment workersReading, letting the next mix node callback see 0
            // and swap the buffer out from under it.
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
    if (node->eqEnabled) {
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
    }

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
        float threshLin = node->compThresholdLin;
        float exponent = 1.0f - 1.0f / node->compRatio;  // e.g., 2/3 for 3:1
        float makeup = node->compMakeupLin;

        for (ma_uint32 i = 0; i < frameCount; ++i) {
            float sL = stereoOut[i * 2];
            float sR = stereoOut[i * 2 + 1];

            // Linked stereo peak detection
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

    // Stage 3a: Master gain — applied PRE-limiter so that boosting above 1.0
    // is reined in by the limiter rather than clipping.
    if (node->masterGain != 1.0f) {
        float mg = node->masterGain;
        for (ma_uint32 i = 0; i < frameCount * 2; ++i)
            stereoOut[i] *= mg;
    }

    // Stage 3b: Soft limiter — smooth tanh saturation above the knee threshold.
    // Transparent below the knee (default 0.8), smoothly curves to ±1.0 above.
    // Replaces the old hard clamp which produced audible click artifacts.
    if (node->limiterEnabled) {
        float knee = node->limiterKnee;
        float range = 1.0f - knee;
        float invRange = 1.0f / range;
        for (ma_uint32 i = 0; i < frameCount * 2; ++i) {
            float x = stereoOut[i];
            if (x > knee)
                stereoOut[i] = knee + range * std::tanh((x - knee) * invRange);
            else if (x < -knee)
                stereoOut[i] = -(knee + range * std::tanh((-x - knee) * invRange));
        }
    } else {
        // Safety hard clamp as absolute last resort (should not be reached
        // if limiter is enabled, but guards against runaway signals)
        for (ma_uint32 i = 0; i < frameCount * 2; ++i) {
            if (stereoOut[i] > 1.0f) stereoOut[i] = 1.0f;
            else if (stereoOut[i] < -1.0f) stereoOut[i] = -1.0f;
        }
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
    bool skipPortalRouting = false;    // true for door sounds — use Steam Audio but skip portal blocking
    bool isAmbient = false;            // true for ambient voices — distance handled by ambient system
    bool loggedReflActivationMain = false;  // diagnostic: footstep convolution-activation log fired once

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

        // Invalidate the shared validity token. The convolution worker holds
        // a shared_ptr copy, so the token stays alive — but reads as false,
        // causing the worker to skip this voice's staged data. This is safe
        // even if the worker checks it after this ActiveVoice is freed.
        if (dspNode.validityToken) {
            dspNode.validityToken->store(false, std::memory_order_release);
        }

        // Stop playback — use immediate stop here since ma_node_uninit below
        // will block until the audio thread finishes, providing a clean boundary.
        if (initialized) {
            ma_sound_stop(&sound);
        }

        // Disconnect and destroy DSP node (detaches from graph, waits for audio thread).
        // After this returns, the audio callback will NOT be called again for this
        // node, so no new staging snapshots will reference this voice's effect.
        if (dspNode.nodeInitialized) {
            ma_node_uninit(&dspNode.base, nullptr);
            dspNode.nodeInitialized = false;
        }

        // Wait for ALL convolution sub-workers to finish pending frames before
        // releasing the reflection effect. Uses processedSeq counter for
        // correctness (no TOCTOU gap).
        if (dspNode.convWorker && dspNode.reflectionEffect) {
            auto &cw = *dspNode.convWorker;
            auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
            for (auto &subPtr : cw.workers) {
                auto &sub = *subPtr;
                uint64_t target = sub.frameSeq.load(std::memory_order_acquire);
                while (sub.processedSeq.load(std::memory_order_acquire) < target) {
                    if (sub.shutdown.load(std::memory_order_relaxed)) break;
                    if (std::chrono::steady_clock::now() >= deadline) {
                        AUDIO_LOG( "[AUDIO] WARNING: ~ActiveVoice worker drain "
                                     "timed out — releasing effect anyway\n");
                        break;
                    }
                    std::this_thread::yield();
                }
            }
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
// Live-tunable mixer gains. Setters update the AudioService member AND, if the
// reflection mix node is alive, push the new value into it so the change takes
// effect on the next audio callback. Plain float writes match the lock-free
// update pattern used elsewhere in this file (e.g. listenerOrientation).
void AudioService::setMasterGain(float g)
{
    mMasterGain = std::max(0.0f, std::min(g, 4.0f));
    if (mReflectionMixNode) mReflectionMixNode->masterGain = mMasterGain;
}

void AudioService::setReflectionGain(float g)
{
    mReflectionGain = std::max(0.0f, std::min(g, 4.0f));
    if (mReflectionMixNode) mReflectionMixNode->reflGainTarget = mReflectionGain;
}

void AudioService::setDirectGain(float g)
{
    mDirectGain = std::max(0.0f, std::min(g, 4.0f));
    if (mReflectionMixNode) mReflectionMixNode->directGain = mDirectGain;
}

//------------------------------------------------------
void AudioService::publishAudioThreadParams()
{
    sHrtfInterpolation.store(mHRTFInterpolation == "nearest" ? 0 : 1,
                             std::memory_order_relaxed);
    sSpatialBlend.store(mSpatialBlend, std::memory_order_relaxed);
    sDoorLpfOpenHz.store(mDoorLpfOpenHz, std::memory_order_relaxed);
    sDoorLpfBlockedHz.store(mDoorLpfBlockedHz, std::memory_order_relaxed);
    sPropMinAttenuation.store(mPropMinAttenuation, std::memory_order_relaxed);
    sDistanceModel.store(mDistanceModel == "inverse_distance" ? 1 : 0,
                         std::memory_order_relaxed);
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
    hrtfSettings.volume = mHRTFVolume;  // 1.0 = HRTF as-is; 0.0 = silence!

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

    // Now that schemas and sound loader are ready, load ambient sounds
    // from mission data (P$AmbientHack properties parsed during onDBLoad).
    loadAmbientSounds();
    AUDIO_LOG( "AudioService: %zu ambient sounds, %zu active voices\n",
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
        // Engine vertices are in feet; IPL expects meters. Scale at the boundary
        // so Steam Audio sees physically-correct dimensions for air absorption,
        // reverb decay, and reflection delays.
        // mSceneMin/mSceneMax stay in FEET — they're used for our own probe-extent
        // computations and are converted to meters at the IPL transform.
        mSceneMin = Vector3( 1e9f,  1e9f,  1e9f);
        mSceneMax = Vector3(-1e9f, -1e9f, -1e9f);
        std::vector<IPLVector3> iplVertices(numVertices);
        for (size_t i = 0; i < numVertices; ++i) {
            float x = data.vertices[i * 3];
            float y = data.vertices[i * 3 + 1];
            float z = data.vertices[i * 3 + 2];
            // Bounds in engine units (feet)
            mSceneMin = glm::min(mSceneMin, Vector3(x, y, z));
            mSceneMax = glm::max(mSceneMax, Vector3(x, y, z));
            // Vertex shipped to IPL in meters
            iplVertices[i] = {x * kFeetToMeters, y * kFeetToMeters, z * kFeetToMeters};
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
        // Divisor 1=full rate, 2=half (24kHz), 4=quarter (12kHz).
        mReflectionSampleRate = mDeviceSampleRate / static_cast<uint32_t>(mReflectionRateDivisor);
        mReflectionFrameSize = mFrameSize / static_cast<uint32_t>(mReflectionRateDivisor);

        // Step 6: Create the simulator for direct occlusion + reflections + reverb.
        // The simulator uses the reflection sample rate — IRs are generated at this
        // rate, which must match the reflection effects and mixer.
        IPLSimulationSettings simSettings{};
        simSettings.flags = static_cast<IPLSimulationFlags>(
            IPL_SIMULATIONFLAGS_DIRECT | IPL_SIMULATIONFLAGS_REFLECTIONS);
        simSettings.sceneType = sceneTypeEnum;
        simSettings.reflectionType = IPL_REFLECTIONEFFECTTYPE_CONVOLUTION;
        // Sim caps — these are upper bounds; runtime uses mReflectionNumRays etc.
        // maxNumRays must be >= the runtime ray count; auto-bump if user set both.
        simSettings.maxNumOcclusionSamples = mSimMaxOcclusionSamplesCfg;
        simSettings.maxNumRays = std::max(mSimMaxRaysCfg, mReflectionNumRays);
        simSettings.numDiffuseSamples = mDiffuseSamples;
        simSettings.maxDuration = mReflectionDuration;
        simSettings.maxOrder = mAmbisonicsOrder;
        simSettings.maxNumSources = mSimMaxSourcesCfg;
        // Ray-trace thread count: 0 = auto (reserve 2 cores for main + audio).
        if (mSimulatorThreadsCfg > 0) {
            simSettings.numThreads = mSimulatorThreadsCfg;
        } else {
            unsigned int hwThreads = std::thread::hardware_concurrency();
            simSettings.numThreads = std::max(2u, hwThreads > 2 ? hwThreads - 2 : 2u);
        }
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

    AUDIO_LOG( "REFL: creating mixer (irSize=%d, channels=%d, rate=%d, frame=%d, 1/%d rate)\n",
                 irSize, numAmbiChannels,
                 audioSettings.samplingRate, audioSettings.frameSize,
                 mReflectionRateDivisor);

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

    // Apply DSP chain config from AudioService members (set by RenderConfig).
    // Must be done BEFORE setting ready=true so the audio callback never sees
    // uninitialized coefficients (ready + simulationRan gate the DSP path).
    rmn.limiterEnabled = mDSPLimiterEnabled;
    rmn.limiterKnee = mDSPLimiterKnee;
    rmn.compressorEnabled = mDSPCompressorEnabled;
    rmn.compThresholdDb = mDSPCompThreshold;
    rmn.compRatio = mDSPCompRatio;
    rmn.compAttackMs = mDSPCompAttackMs;
    rmn.compReleaseMs = mDSPCompReleaseMs;
    rmn.eqEnabled = mDSPEQEnabled;
    rmn.eqFreq = mDSPEQFreq;
    rmn.eqGainDb = mDSPEQGain;
    rmn.eqQ = mDSPEQQ;
    rmn.duckingEnabled = mDSPDuckingEnabled;
    rmn.duckAmount = mDSPDuckAmount;
    rmn.duckAttackMs = mDSPDuckAttackMs;
    rmn.duckReleaseMs = mDSPDuckReleaseMs;

    // Mixer config — global gains + reflection ramp time.
    // reflRampRate is the per-sample increment such that a full 0→1 ramp takes
    // mReflectionRampMs milliseconds at the current sample rate.
    rmn.masterGain = mMasterGain;
    rmn.directGain = mDirectGain;
    rmn.reflGainTarget = mReflectionGain;
    {
        float rampSamples = (mReflectionRampMs * 0.001f) * static_cast<float>(mDeviceSampleRate);
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

    // NOTE: rmn.convWorker is set AFTER the worker is created (below).

    // Create the off-thread convolution worker with K parallel sub-workers.
    // Each sub-worker owns its own IPLReflectionMixer and IPLAmbisonicsDecodeEffect.
    mConvolutionWorker = std::make_unique<ConvolutionWorker>();
    auto &cw = *mConvolutionWorker;
    cw.context = mIplContext;
    cw.hrtf = mIplHrtf;
    cw.frameSize = static_cast<int>(mFrameSize);
    cw.reflectionFrameSize = static_cast<int>(mReflectionFrameSize);
    cw.rateDivisor = mReflectionRateDivisor;
    cw.ambiChannels = mAmbisonicsChannels;
    cw.ambiOrder = mAmbisonicsOrder;

    // Determine number of sub-workers
    if (mConvolutionWorkerCount > 0) {
        cw.numWorkers = mConvolutionWorkerCount;
    } else {
        // Auto: use available cores minus 3 (main + audio + sim), minimum 1
        unsigned int hwThreads = std::thread::hardware_concurrency();
        cw.numWorkers = std::max(1, static_cast<int>(hwThreads) - 3);
    }

    // Allocate shared per-voice mono staging buffers (both double-buffer sides)
    for (int b = 0; b < 2; ++b) {
        for (int i = 0; i < ConvolutionWorker::kMaxSlots; ++i) {
            cw.staging[b][i].mono.resize(mReflectionFrameSize, 0.0f);
        }
    }

    // Create K sub-workers, each with own mixer + decoder + scratch + output
    for (int i = 0; i < cw.numWorkers; ++i)
        cw.workers.push_back(std::make_unique<ConvolutionSubWorker>());
    bool allWorkersOk = true;
    for (int wk = 0; wk < cw.numWorkers; ++wk) {
        auto &sub = *cw.workers[wk];

        // Create sub-worker's own reflection mixer
        err = iplReflectionMixerCreate(mIplContext, &audioSettings,
                                        &reflSettings, &sub.mixer);
        if (err != IPL_STATUS_SUCCESS) {
            LOG_ERROR("AudioService: sub-worker %d mixer create failed (%d)", wk, err);
            allWorkersOk = false;
            break;
        }

        // Create sub-worker's own ambisonics decode effect
        err = iplAmbisonicsDecodeEffectCreate(mIplContext, &audioSettings,
                                               &decodeSettings, &sub.ambiDecodeEffect);
        if (err != IPL_STATUS_SUCCESS) {
            LOG_ERROR("AudioService: sub-worker %d decode create failed (%d)", wk, err);
            iplReflectionMixerRelease(&sub.mixer);
            sub.mixer = nullptr;
            allWorkersOk = false;
            break;
        }

        // Allocate double-buffered stereo output
        for (int b = 0; b < 2; ++b) {
            sub.stereoL[b].resize(mFrameSize, 0.0f);
            sub.stereoR[b].resize(mFrameSize, 0.0f);
        }

        // Allocate scratch buffers
        sub.ambiCh0.resize(mReflectionFrameSize, 0.0f);
        if (mAmbisonicsChannels > 1) sub.ambiCh1.resize(mReflectionFrameSize, 0.0f);
        if (mAmbisonicsChannels > 2) sub.ambiCh2.resize(mReflectionFrameSize, 0.0f);
        if (mAmbisonicsChannels > 3) sub.ambiCh3.resize(mReflectionFrameSize, 0.0f);
        sub.decodedL.resize(mReflectionFrameSize, 0.0f);
        sub.decodedR.resize(mReflectionFrameSize, 0.0f);
        sub.voiceAmbi0.resize(mReflectionFrameSize, 0.0f);
        if (mAmbisonicsChannels > 1) sub.voiceAmbi1.resize(mReflectionFrameSize, 0.0f);
        if (mAmbisonicsChannels > 2) sub.voiceAmbi2.resize(mReflectionFrameSize, 0.0f);
        if (mAmbisonicsChannels > 3) sub.voiceAmbi3.resize(mReflectionFrameSize, 0.0f);
    }

    if (!allWorkersOk) {
        // Clean up partially created sub-workers
        for (auto &subPtr : cw.workers) {
            if (subPtr->ambiDecodeEffect)
                iplAmbisonicsDecodeEffectRelease(&subPtr->ambiDecodeEffect);
            if (subPtr->mixer)
                iplReflectionMixerRelease(&subPtr->mixer);
        }
        mConvolutionWorker.reset();
    }

    if (mConvolutionWorker) {
        // Spawn sub-worker threads
        for (int wk = 0; wk < cw.numWorkers; ++wk) {
            cw.workers[wk]->shutdown.store(false, std::memory_order_relaxed);
            cw.workers[wk]->thread = std::thread([this, wk]() { convolutionSubWorkerMain(wk); });
        }

        // Wire the mix node to the worker
        rmn.convWorker = mConvolutionWorker.get();

        AUDIO_LOG( "REFL: convolution started (%d sub-workers, off-thread)\n",
                     cw.numWorkers);
    }

    AUDIO_LOG( "AudioService: reflection pipeline initialized "
                 "(convolution, order %d (%dch), IR %d samples, %uHz, 1/%d rate, max %d voices%s)\n",
                 mAmbisonicsOrder, mAmbisonicsChannels,
                 irSize, mReflectionSampleRate, mReflectionRateDivisor,
                 mMaxReflectionVoices,
                 mConvolutionWorker ? ", off-thread" : ", on-thread fallback");
    return true;
}

//------------------------------------------------------
void AudioService::destroyReflectionPipeline()
{
    // Shut down all convolution sub-workers before destroying any Steam Audio objects
    if (mConvolutionWorker) {
        for (auto &subPtr : mConvolutionWorker->workers) {
            subPtr->shutdown.store(true, std::memory_order_release);
            subPtr->frameSeq.fetch_add(1, std::memory_order_release);  // wake it
        }
        for (auto &subPtr : mConvolutionWorker->workers) {
            if (subPtr->thread.joinable())
                subPtr->thread.join();
            // Release sub-worker-owned Steam Audio objects
            if (subPtr->ambiDecodeEffect)
                iplAmbisonicsDecodeEffectRelease(&subPtr->ambiDecodeEffect);
            if (subPtr->mixer)
                iplReflectionMixerRelease(&subPtr->mixer);
        }
        mConvolutionWorker.reset();
    }

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

    // Start the reflection simulation worker thread (background, latency-tolerant).
    // Direct sim runs synchronously on the main thread for same-frame occlusion.
    if (mAudioReady) {
        mReflectionSimShutdown.store(false, std::memory_order_relaxed);
        mReflectionSimThread = std::thread(&AudioService::reflectionSimWorkerMain, this);
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
    mAmbients.clear();

    // Release sound resources
    mSoundCache.reset();
    mSoundLoader.reset();

    // Shut down the reflection simulation worker thread before destroying the scene
    mReflectionSimShutdown.store(true, std::memory_order_release);
    mReflectionSimCV.notify_one();
    if (mReflectionSimThread.joinable())
        mReflectionSimThread.join();

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
    mRoomEAXPresets.clear();
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

    // Update ambient volumes early so newly created ambient voices exist before
    // the simulation runs. This ensures same-frame occlusion for new ambients —
    // voices created after the sim would miss occlusion for their first frame.
    updateAmbientVolumes();

    // Source mutations (add/remove/commit) can race with the background reflection
    // sim thread, so we defer them until it's idle. Direct sim runs synchronously
    // on the main thread, so it's never concurrent with mutations.
    // Steam Audio uses double-buffering: setInputs writes to the staging buffer,
    // while runReflections reads from the committed (active) buffer.
    // Only commit() copies staging → active, so commit must wait for reflections.
    bool reflBusy = mReflectionSimRunning.load(std::memory_order_acquire);
    bool canMutate = !reflBusy;

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

    // Commit copies staging → active buffer. Must wait for reflection sim to
    // be idle since it reads from the active buffer. Direct sim runs inline
    // (after commit), so no conflict there.
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

        // Listener origin → IPL in meters (engine stores feet).
        // Direction vectors are unit-length, no scaling needed.
        IPLCoordinateSpace3 listenerCoord{};
        listenerCoord.origin = {mListenerPos.x * kFeetToMeters,
                                mListenerPos.y * kFeetToMeters,
                                mListenerPos.z * kFeetToMeters};
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

        {
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
            // Requires probes to be loaded (`mProbesHaveReflections`); if not,
            // player-emitted voices fall through to the same path as everything
            // else and the timing race may reappear.
            if (mReflectionsEnabled) {
                reflCandidates.reserve(mVoices.size());
                for (auto &[h, v] : mVoices) {
                    if (v->sourceEnded)
                        continue;
                    if (v->playerEmitted && mProbesHaveReflections)
                        continue;  // routed via baked-probe path below
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

                // Player-emitted voices (footsteps, landing) run the per-voice
                // DSP node like every other voice — direct attenuation, HRTF,
                // and reflection convolution staging all happen normally.
                // What's different for them is configured at the simulator
                // input level below: occlusion + transmission disabled, dist
                // attenuation forced to 1.0, baked reflections routed through
                // the listener-position probe IR, and (just below) portal
                // routing skipped. Together those replace the older broad
                // "skipAttenuation skips the entire DSP block" bypass which
                // had been silently preventing footsteps from reaching the
                // convolution path as a side-effect.
                voice->dspNode.skipAttenuation = false;

                // Run portal propagation to determine reachability and path.
                // Throttled: nearby voices update every frame, distant voices
                // every 8-16 frames. Matching the original engine's adaptive
                // update frequency. Uses cached result between updates.
                //
                // Player-emitted voices (footsteps, landings) are always in
                // the same "room" as the listener by definition — the source
                // IS the player. Skipping portal propagation entirely avoids
                // a 953d322-class bug: head↔feet distance is ~5-6 ft (just
                // over the 5-unit same-room threshold below), so at floor or
                // ceiling room boundaries a footstep can flicker into
                // isCrossRoom=true, get its source moved to a portal anchor,
                // and end up portal-attenuated + door-LPF'd into silence on
                // the dry path. Direct path stays at full volume; baked
                // reflection routing handles the wet path independently.
                SoundPropInfo prop{};
                bool isCrossRoom = false;
                if (mPortalRoutingEnabled && !voice->sourceEnded
                    && !voice->skipPortalRouting && !voice->playerEmitted) {
                    if (voice->propagationCountdown <= 0) {
                        auto prT0 = std::chrono::steady_clock::now();
                        prop = propagateSoundBlended(voice->worldPos,
                                                     mPropagationMaxDist);
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
                    voice->dspNode.portalBlocking = prop.totalBlocking;
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
                // Source origin → IPL in meters (engine positions are in feet).
                // Cross-room sounds project from the last portal's center
                // (prop.virtualPosition) so distance attenuation reflects the
                // longer path. Both feet, both converted here.
                IPLCoordinateSpace3 sourceCoord{};
                if (isCrossRoom && prop.reached) {
                    sourceCoord.origin = {prop.virtualPosition.x * kFeetToMeters,
                                          prop.virtualPosition.y * kFeetToMeters,
                                          prop.virtualPosition.z * kFeetToMeters};
                } else {
                    sourceCoord.origin = {voice->worldPos.x * kFeetToMeters,
                                          voice->worldPos.y * kFeetToMeters,
                                          voice->worldPos.z * kFeetToMeters};
                }
                sourceCoord.ahead = {1.0f, 0.0f, 0.0f};
                sourceCoord.right = {0.0f, 1.0f, 0.0f};
                sourceCoord.up    = {0.0f, 0.0f, 1.0f};

                IPLSimulationInputs inputs{};
                inputs.flags = static_cast<IPLSimulationFlags>(
                    IPL_SIMULATIONFLAGS_DIRECT | IPL_SIMULATIONFLAGS_REFLECTIONS);
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
                // Door/local sounds use a larger occlusion sphere so they aren't
                // over-occluded by narrow door frames. The larger sphere means more
                // rays see around edges, producing gentle partial occlusion instead
                // of heavy blocking. Works correctly for both player and NPC doors.
                // Engine radius is in feet (matches occlusion_radius YAML key);
                // convert to meters at the IPL boundary.
                // 16 ft floor ≈ 4.88 m at IPL — generous enough for typical
                // doorways (~3 ft wide) so a partially-open door produces gentle
                // ramping rather than abrupt blocking.
                float occRadiusFt = voice->skipPortalRouting
                    ? std::max(mOcclusionRadius, 16.0f)
                    : mOcclusionRadius;
                inputs.occlusionRadius = occRadiusFt * kFeetToMeters;
                inputs.numOcclusionSamples = mOcclusionSamples;
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
                                && mProbesHaveReflections;
                if (useBaked) {
                    inputs.baked = IPL_TRUE;
                    inputs.bakedDataIdentifier = bakedReflId;
                }

                iplSourceSetInputs(voice->iplSource,
                    static_cast<IPLSimulationFlags>(
                        IPL_SIMULATIONFLAGS_DIRECT | IPL_SIMULATIONFLAGS_REFLECTIONS),
                    &inputs);
            }

            // Step 3: Run direct sim and signal reflection thread.
            // Direct sim runs synchronously every frame.
            // Reflection sim runs on background thread, throttled.
            bool wantReflections = mReflectionsEnabled && mIplReflectionMixer
                && !reflBusy
                && (++mReflectionFrameCounter >= mReflectionThrottle);
            if (wantReflections)
                mReflectionFrameCounter = 0;

            // Run direct sim synchronously on the main thread (2-5ms).
            // Same-frame results: every voice (including newly created ones)
            // gets real occlusion/distance/air absorption before the audio
            // callback sees them. No fallback path needed.
            {
                auto dt0 = std::chrono::steady_clock::now();
                iplSimulatorRunDirect(mIplSimulator);
                auto dt1 = std::chrono::steady_clock::now();
                float dMs = std::chrono::duration<float, std::milli>(dt1 - dt0).count();
                float prevD = sDirectSimPeakMs.load(std::memory_order_relaxed);
                if (dMs > prevD) sDirectSimPeakMs.store(dMs, std::memory_order_relaxed);
            }

            // Signal reflection sim (throttled, latency-tolerant, background thread)
            if (wantReflections) {
                mReflectionSimRunning.store(true, std::memory_order_release);
                {
                    std::lock_guard<std::mutex> lock(mReflectionSimMutex);
                    mReflectionSimWant = true;
                }
                mReflectionSimCV.notify_one();
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
                // Skip sources that haven't been added to the simulator yet.
                // When the reflection sim is busy at voice creation time,
                // iplSourceAdd is deferred to mPendingSourceAdds. Until it's
                // flushed and the sim processes the source, iplSourceGetOutputs
                // returns uninitialized values. Keep silent initVoiceDSP defaults.
                bool isPending = std::find(mPendingSourceAdds.begin(),
                                           mPendingSourceAdds.end(),
                                           voice->iplSource) != mPendingSourceAdds.end();

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

                if (isPending)
                    continue;

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
                        AUDIO_LOG("[FOOT_REFL_OFF] h=%u '%s' dist=%.1f reason=%s "
                                  "irSize=%d slots=%d/%d hasBaked=%d\n",
                                  handle, voice->schemaName.c_str(), voiceDist,
                                  isReflVoice ? "no-cap" :
                                  (hasBakedData ? "no-cap-baked" :
                                   (canAffordConvolution ? "not-topN-no-baked" : "cap-full")),
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
void AudioService::reflectionSimWorkerMain()
{
    // Dedicated thread for reflection simulation (ray-traced reverb).
    // Runs every Nth frame, can take 50-200ms — latency-tolerant because
    // reverb tails change slowly with listener movement.
    // Separated from direct sim so it never blocks occlusion updates.
    while (true) {
        {
            std::unique_lock<std::mutex> lock(mReflectionSimMutex);
            mReflectionSimCV.wait(lock, [this] {
                return mReflectionSimWant
                       || mReflectionSimShutdown.load(std::memory_order_relaxed);
            });
            if (mReflectionSimShutdown.load(std::memory_order_relaxed))
                break;
            mReflectionSimWant = false;
        }

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

//------------------------------------------------------
void AudioService::convolutionSubWorkerMain(int workerIdx)
{
    if (!mConvolutionWorker) return;
    auto &cw = *mConvolutionWorker;
    if (workerIdx < 0 || workerIdx >= static_cast<int>(cw.workers.size())) return;
    auto &sub = *cw.workers[workerIdx];
    uint64_t lastSeq = 0;

    while (!sub.shutdown.load(std::memory_order_relaxed)) {
        // Wait for the mix node to signal new data
        uint64_t seq = sub.frameSeq.load(std::memory_order_acquire);
        if (seq == lastSeq) {
            std::this_thread::yield();
            continue;
        }
        lastSeq = seq;

        auto t0 = std::chrono::steady_clock::now();

        // workersReading was pre-set by the mix node before signaling us.
        // We only decrement when done — no increment needed here.

        int readBuf = cw.currentReadBuf;  // visible via frameSeq acquire barrier
        int assignCount = sub.assignedCount;  // likewise
        int back = 1 - sub.frontIdx.load(std::memory_order_relaxed);

        // Clear this sub-worker's back stereo buffers
        std::memset(sub.stereoL[back].data(), 0, cw.frameSize * sizeof(float));
        std::memset(sub.stereoR[back].data(), 0, cw.frameSize * sizeof(float));

        // Diagnostic: report worker iteration entry state. If assignCount=0
        // we know workers are running but receiving no work. If >0 but the
        // ANY_SLOT_WET diagnostic doesn't fire, slots are failing the early
        // continue checks.
        {
            static std::atomic<int> sWorkerEntryLogCount{0};
            int n = sWorkerEntryLogCount.fetch_add(1, std::memory_order_relaxed);
            if (n < 24) {
                int activeSlots = 0;
                for (int j = 0; j < assignCount; ++j) {
                    int i = sub.assignedSlots[j];
                    if (i < 0 || i >= ConvolutionWorker::kMaxSlots) continue;
                    auto &slot = cw.staging[readBuf][i];
                    if (slot.active) activeSlots++;
                }
                AUDIO_LOG("[WORKER_ENTRY] w=%d readBuf=%d assignCount=%d activeSlots=%d\n",
                          workerIdx, readBuf, assignCount, activeSlots);
            }
        }

        // Process assigned voice slots through this sub-worker's mixer
        int nch = cw.ambiChannels;
        for (int j = 0; j < assignCount; ++j) {
            int i = sub.assignedSlots[j];
            if (i < 0 || i >= ConvolutionWorker::kMaxSlots) continue;
            auto &slot = cw.staging[readBuf][i];
            if (!slot.active || !slot.effect || slot.params.irSize <= 0)
                continue;
            if (slot.validityToken && !slot.validityToken->load(std::memory_order_acquire))
                continue;

            // Build input buffer from the mono snapshot
            float *monoPtr = slot.mono.data();
            IPLAudioBuffer reflIn{};
            reflIn.numChannels = 1;
            reflIn.numSamples = static_cast<IPLint32>(slot.reflFrameSize);
            reflIn.data = &monoPtr;

            // Build output buffer (sub-worker-owned per-voice ambisonics scratch)
            float *ambiPtrs[4] = {
                sub.voiceAmbi0.data(),
                !sub.voiceAmbi1.empty() ? sub.voiceAmbi1.data() : nullptr,
                !sub.voiceAmbi2.empty() ? sub.voiceAmbi2.data() : nullptr,
                !sub.voiceAmbi3.empty() ? sub.voiceAmbi3.data() : nullptr
            };
            IPLAudioBuffer reflOut{};
            reflOut.numChannels = slot.params.numChannels;
            reflOut.numSamples = static_cast<IPLint32>(slot.reflFrameSize);
            reflOut.data = ambiPtrs;

            // Apply convolution, accumulating into this sub-worker's mixer
            iplReflectionEffectApply(slot.effect, &slot.params,
                                      &reflIn, &reflOut, sub.mixer);

            // Diagnostic: log every Nth slot the worker processes so we can
            // tell apart "worker not seeing slot" vs. "worker seeing slot but
            // isFootstepDiag flag not propagated." Reports the flag value.
            {
                static std::atomic<int> sAnySlotLogCount{0};
                int n = sAnySlotLogCount.fetch_add(1, std::memory_order_relaxed);
                if (n < 24) {
                    float inPeak = 0.0f;
                    for (int s = 0; s < slot.reflFrameSize; ++s)
                        inPeak = std::max(inPeak, std::fabs(slot.mono[s]));
                    float wPeak = 0.0f;
                    for (int s = 0; s < slot.reflFrameSize; ++s)
                        wPeak = std::max(wPeak, std::fabs(sub.voiceAmbi0[s]));
                    AUDIO_LOG("[ANY_SLOT_WET] w=%d slotIdx=%d isFootDiag=%d "
                              "inPeak=%.4f wOutPeak=%.4f irSize=%d nch=%d\n",
                              workerIdx, i, slot.isFootstepDiag ? 1 : 0,
                              inPeak, wPeak,
                              slot.params.irSize, slot.params.numChannels);
                }
            }

            // Diagnostic: for footstep slots, report input + output peak
            // amplitudes so we can confirm convolution is actually producing
            // audible wet signal. The W (omnidirectional) ambisonics channel
            // is the right metric for "total wet energy."
            if (slot.isFootstepDiag) {
                static std::atomic<int> sFootWetLogCount{0};
                int n = sFootWetLogCount.fetch_add(1, std::memory_order_relaxed);
                if (n < 12) {  // ~12 sample frames is enough to characterize
                    float inPeak = 0.0f;
                    for (int s = 0; s < slot.reflFrameSize; ++s)
                        inPeak = std::max(inPeak, std::fabs(slot.mono[s]));
                    float wPeak = 0.0f;
                    for (int s = 0; s < slot.reflFrameSize; ++s)
                        wPeak = std::max(wPeak, std::fabs(sub.voiceAmbi0[s]));
                    AUDIO_LOG("[FOOT_REFL_WET] worker=%d inPeak=%.4f wOutPeak=%.4f "
                              "irSize=%d nch=%d ratio=%.3f\n",
                              workerIdx, inPeak, wPeak,
                              slot.params.irSize, slot.params.numChannels,
                              inPeak > 1e-6f ? wPeak / inPeak : 0.0f);
                }
            }
        }

        // Release staging slot references for this sub-worker's assigned slots.
        // Each sub-worker operates on disjoint slots (round-robin) — no contention.
        for (int j = 0; j < assignCount; ++j) {
            int i = sub.assignedSlots[j];
            if (i < 0 || i >= ConvolutionWorker::kMaxSlots) continue;
            auto &slot = cw.staging[readBuf][i];
            slot.effect = nullptr;
            slot.validityToken.reset();
            slot.active = false;
        }

        // Done reading shared staging
        cw.workersReading.fetch_sub(1, std::memory_order_acq_rel);

        // Extract accumulated ambisonics from this sub-worker's mixer
        ma_uint32 reflFrameCount = static_cast<ma_uint32>(cw.reflectionFrameSize);
        float *ambiChannels[4] = {
            sub.ambiCh0.data(),
            nch > 1 ? sub.ambiCh1.data() : nullptr,
            nch > 2 ? sub.ambiCh2.data() : nullptr,
            nch > 3 ? sub.ambiCh3.data() : nullptr
        };
        IPLAudioBuffer ambiOut{};
        ambiOut.numChannels = nch;
        ambiOut.numSamples = static_cast<IPLint32>(reflFrameCount);
        ambiOut.data = ambiChannels;

        IPLReflectionEffectParams mixerParams{};
        mixerParams.type = IPL_REFLECTIONEFFECTTYPE_CONVOLUTION;
        mixerParams.numChannels = nch;
        iplReflectionMixerApply(sub.mixer, &mixerParams, &ambiOut);

        // Decode ambisonics to binaural stereo
        float *decodedPtrs[2] = {sub.decodedL.data(), sub.decodedR.data()};
        IPLAudioBuffer decodedBuf{};
        decodedBuf.numChannels = 2;
        decodedBuf.numSamples = static_cast<IPLint32>(reflFrameCount);
        decodedBuf.data = decodedPtrs;

        IPLAmbisonicsDecodeEffectParams decodeParams{};
        decodeParams.order = cw.ambiOrder;
        decodeParams.hrtf = cw.hrtf;
        decodeParams.orientation = cw.listenerOrientation;
        decodeParams.binaural = IPL_TRUE;
        iplAmbisonicsDecodeEffectApply(sub.ambiDecodeEffect, &decodeParams,
                                        &ambiOut, &decodedBuf);

        // Write decoded stereo to this sub-worker's back buffer.
        // Handle upsampling from reflection rate to device rate.
        int div = cw.rateDivisor;
        if (div > 1) {
            // Linear interpolation upsample by rateDivisor
            ma_uint32 inFrames = std::min(reflFrameCount,
                static_cast<ma_uint32>(cw.frameSize / div));
            for (ma_uint32 j = 0; j < inFrames; ++j) {
                float l0 = sub.decodedL[j];
                float r0 = sub.decodedR[j];
                float l1 = (j + 1 < reflFrameCount) ? sub.decodedL[j + 1] : l0;
                float r1 = (j + 1 < reflFrameCount) ? sub.decodedR[j + 1] : r0;
                for (int d = 0; d < div; ++d) {
                    float t = static_cast<float>(d) / static_cast<float>(div);
                    sub.stereoL[back][j * div + d] = l0 + (l1 - l0) * t;
                    sub.stereoR[back][j * div + d] = r0 + (r1 - r0) * t;
                }
            }
        } else {
            ma_uint32 outSamples = std::min(reflFrameCount, static_cast<ma_uint32>(cw.frameSize));
            std::memcpy(sub.stereoL[back].data(), sub.decodedL.data(), outSamples * sizeof(float));
            std::memcpy(sub.stereoR[back].data(), sub.decodedR.data(), outSamples * sizeof(float));
        }

        // Swap: make back buffer the new front
        sub.frontIdx.store(back, std::memory_order_release);
        sub.hasProducedOutput.store(true, std::memory_order_release);

        auto t1 = std::chrono::steady_clock::now();
        float ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
        float prev = sub.peakMs.load(std::memory_order_relaxed);
        if (ms > prev) sub.peakMs.store(ms, std::memory_order_relaxed);

        // Signal that this frame's data has been fully processed
        sub.processedSeq.store(lastSeq, std::memory_order_release);
    }
}

//------------------------------------------------------
void AudioService::waitForConvolutionWorker()
{
    if (!mConvolutionWorker) return;
    auto &cw = *mConvolutionWorker;

    // Wait until ALL sub-workers have finished processing ALL frames signaled so far.
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    for (auto &subPtr : cw.workers) {
        auto &sub = *subPtr;
        uint64_t target = sub.frameSeq.load(std::memory_order_acquire);
        while (sub.processedSeq.load(std::memory_order_acquire) < target) {
            if (sub.shutdown.load(std::memory_order_relaxed)) break;
            if (std::chrono::steady_clock::now() >= deadline) {
                AUDIO_LOG( "[AUDIO] WARNING: convolution sub-worker wait timed out "
                             "(target=%llu, processed=%llu)\n",
                             (unsigned long long)target,
                             (unsigned long long)sub.processedSeq.load(std::memory_order_relaxed));
                break;
            }
            std::this_thread::yield();
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
            AUDIO_LOG( "[VOICE] CLEANUP h=%d '%s' srcEnded=%d tail=%.1f\n",
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

    // iplSourceAdd modifies the simulator's source list, which the reflection
    // sim thread iterates. Defer the add if it's running to prevent races.
    // Direct sim runs inline on the main thread, so no conflict.
    if (mReflectionSimRunning.load(std::memory_order_acquire)) {
        mPendingSourceAdds.push_back(voice.iplSource);
    } else {
        iplSourceAdd(voice.iplSource, mIplSimulator);
    }
    mSimulatorDirty = true;
}

//------------------------------------------------------
void AudioService::waitForReflectionThread()
{
    // Spin-wait for the reflection sim to complete on its background thread.
    // Called infrequently (voice removal, shutdown). Direct sim runs inline
    // on the main thread so it's never concurrent here.
    while (mReflectionSimRunning.load(std::memory_order_acquire)) {
        std::this_thread::yield();
    }
}

//------------------------------------------------------
void AudioService::removeVoiceSource(ActiveVoice &voice)
{
    if (!voice.iplSource)
        return;

    // Invalidate this voice's effects BEFORE waiting for the worker.
    // The worker checks effectsReady before using the effect pointer, so
    // setting it false here ensures any in-flight processing skips this voice.
    // This closes the race where the worker hasn't started yet but has a
    // staging snapshot containing this voice's effect pointer.
    voice.dspNode.effectsReady.store(false, std::memory_order_release);
    voice.dspNode.reflectionsActive.store(false, std::memory_order_release);

    // Wait for the worker to finish processing all current frames.
    waitForConvolutionWorker();

    // If this source was deferred for add but never actually added to the
    // simulator, just release it directly. This check must run BEFORE the
    // sim-busy check — otherwise when sim threads are idle we'd call
    // iplSourceRemove on a source that was never iplSourceAdd'ed, crashing
    // inside Steam Audio (pointer authentication failure / SIGSEGV).
    auto addIt = std::find(mPendingSourceAdds.begin(),
                           mPendingSourceAdds.end(), voice.iplSource);
    if (addIt != mPendingSourceAdds.end()) {
        mPendingSourceAdds.erase(addIt);
        iplSourceRelease(&voice.iplSource);
        return;
    }

    // If the reflection sim thread is running, we can't safely call
    // iplSourceRemove (it races with iplSimulatorRunReflections).
    // Defer the removal — queue the IPL source handle for later cleanup.
    // Direct sim runs inline on the main thread, so no conflict.
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
    dsp.rateDivisor = mReflectionRateDivisor;

    // Allocate processing buffers (once, never reallocated — audio thread safe)
    dsp.monoScratch.resize(dsp.frameSize);
    dsp.directEffectOut.resize(dsp.frameSize, 0.0f);
    dsp.stereoL.resize(dsp.frameSize);
    dsp.stereoR.resize(dsp.frameSize);
    dsp.ambiScratch0.resize(mReflectionFrameSize, 0.0f);
    if (mAmbisonicsChannels > 1) dsp.ambiScratch1.resize(mReflectionFrameSize, 0.0f);
    if (mAmbisonicsChannels > 2) dsp.ambiScratch2.resize(mReflectionFrameSize, 0.0f);
    if (mAmbisonicsChannels > 3) dsp.ambiScratch3.resize(mReflectionFrameSize, 0.0f);
    if (mReflectionRateDivisor > 1)
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

    // Pre-warm the binaural effect's overlap-add history buffer with one
    // frame of silence. Without this, the first real audio frame for a new
    // voice gets attenuated by ~50% because overlap-add convolution against
    // a zero-history buffer drops the leading samples.
    //
    // Audible only on short transients (footsteps, impacts, glass breaks)
    // where the leading edge IS the perceived sound. Long voices fade in
    // imperceptibly. Voices spawn at arbitrary phases within the audio
    // buffer cycle, so the impact transient lands in the first frame for
    // some voices and in subsequent frames for others — producing an A/B
    // alternation in perceived loudness unrelated to source position.
    //
    // One-time ~50 µs cost at voice creation; negligible.
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
        iplBinauralEffectApply(dsp.binauralEffect, &warmParams, &warmIn, &warmOut);
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
            dsp.convWorker = mConvolutionWorker.get();
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

    // Set default direct params to SILENT — new voices stay inaudible until
    // the simulation provides real occlusion/distance values. This prevents
    // a 1-frame pop at full volume for voices created after the sim runs.
    // The simulation overwrites these in loopStep once it processes the source.
    dsp.directParams.flags = static_cast<IPLDirectEffectFlags>(
        IPL_DIRECTEFFECTFLAGS_APPLYDISTANCEATTENUATION |
        IPL_DIRECTEFFECTFLAGS_APPLYOCCLUSION);
    dsp.directParams.transmissionType = IPL_TRANSMISSIONTYPE_FREQDEPENDENT;
    dsp.directParams.distanceAttenuation = 0.0f;
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

    for (const auto &[handle, voice] : mVoices) {
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
                                      int objID, float volume)
{
    if (!mMaEngine || !mSoundLoader)
        return SOUND_HANDLE_INVALID;

    // Enforce voice limit — evict lowest priority if full.
    // Effective cap is min(config, compile-time max for array sizing).
    int effectiveCap = std::min(mMaxActiveVoicesCfg, MAX_ACTIVE_VOICES);
    if (static_cast<int>(mVoices.size()) >= effectiveCap) {
        if (!evictLowestPriority(priority)) {
            AUDIO_LOG( "[VOICE] POOL_FULL cannot play '%s' pri=%d voices=%zu\n",
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
    voice->handle = mNextHandle++;
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
    if (bypassPortalBlocking && h != SOUND_HANDLE_INVALID && mVoices.count(h)) {
        mVoices[h]->skipPortalRouting = true;
        AUDIO_LOG( "[DOOR_SND] CREATE h=%u '%s' vol=%.3f "
                     "schemaVol=%d pos=(%.1f,%.1f,%.1f)\n",
                     h, schema->name.c_str(), vol,
                     schema->playParams.volume,
                     position.x, position.y, position.z);
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
        auto &voice = *it->second;
        AUDIO_LOG( "[VOICE] HALT h=%d '%s' hasRefl=%d\n",
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
                float loudRoom = 1.0f;
                auto lrIt = mRoomTransmission.find(roomID);
                if (lrIt != mRoomTransmission.end())
                    loudRoom = lrIt->second;

                AUDIO_LOG( "  %4d | %5d | %-18s | %7.2f  | %4d | %6d | %.2f\n",
                             roomID, objID, presetName, decayTime, dampening, height, loudRoom);
            }
            AUDIO_LOG( "=== End Room Acoustic Verification ===\n\n");
        }
    }

    // Find all objects with P$AmbientHack (property name "AmbientHa" in the gamesys)
    auto objIDs = getAllObjectsWithProperty(mPropertyService.get(), "AmbientHa");

    // Track schemas we've already warned about so we emit one warning per
    // unique missing name regardless of how many objects reference it.
    // (e.g. miss6.mis references the undefined schema "m06subson" on 7 objects.)
    std::unordered_set<std::string> warnedMissing;

    int loaded = 0;
    int skipped = 0;
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

        // Validate that the referenced schema exists, or — failing that —
        // that snd.crf contains a raw WAV with the same name (the runtime
        // fallback path in updateAmbientVolumes also tries this). If both
        // miss, the original game data has a dangling reference; drop the
        // ambient so we don't churn through a doomed lookup every frame.
        bool schemaExists = mSchemaParser &&
                            mSchemaParser->findSchema(amb.schemaName) != nullptr;
        bool rawSampleExists = mSoundLoader &&
                               mSoundLoader->hasSound(amb.schemaName);
        if (!schemaExists && !rawSampleExists) {
            if (warnedMissing.insert(amb.schemaName).second) {
                LOG_ERROR("AudioService: AmbientHack on obj %d references "
                          "unknown schema/sample '%s' — skipping",
                          objID, amb.schemaName.c_str());
            }
            ++skipped;
            continue;
        }

        // Register ambient data only — voices are started/stopped dynamically
        // in updateAmbientVolumes() based on listener distance.
        mAmbients.push_back(std::move(amb));
        ++loaded;
    }

    if (loaded > 0) {
        LOG_INFO("AudioService: registered %d ambient sounds from P$AmbientHack", loaded);
    }
    if (skipped > 0) {
        LOG_INFO("AudioService: skipped %d ambient sound(s) with missing schemas/samples",
                 skipped);
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
                AUDIO_LOG( "  [AMB] '%s' dist=%.0f rad=%.0f atten=%.3f portal=%d(%.3f) pos=(%.0f,%.0f,%.0f)\n",
                             amb.schemaName.c_str(), d, amb.radius, atten,
                             portal?1:0, pAtten,
                             amb.position.x, amb.position.y, amb.position.z);
            }
        }

        float portalAvgUs = portalCalls > 0 ? static_cast<float>(portalUs) / portalCalls : 0.0f;

        AUDIO_LOG( "[Audio] %zu voices (%d refl, %d tail), %d/%zu ambients | "
                     "cb: total=%.0f/%.0fµs (%.0f%%) voice=%.0fµs mix=%.0fµs | "
                     "main: loop=%.1fms commit=%.1fms portal=%.0fµs(%dcalls,avg=%.0f) | "
                     "sim: direct=%.1fms refl=%.1fms(%dsteps) rays/s=%.0f | "
                     "churn: +%d/-%d",
                     mVoices.size(), reflVoices, tailVoices, playing, mAmbients.size(),
                     totalUs, budgetUs, loadPct, voiceUs, mixUs,
                     loopMs, commitMs, static_cast<float>(portalUs), portalCalls, portalAvgUs,
                     directMs, reflMs, reflFrames, raysPerSec,
                     created, destroyed);
        // Append convolution sub-worker stats if active
        if (mConvolutionWorker) {
            float maxMs = 0.0f;
            for (auto &subPtr : mConvolutionWorker->workers) {
                float ms = subPtr->peakMs.exchange(0.0f, std::memory_order_relaxed);
                if (ms > maxMs) maxMs = ms;
            }
            AUDIO_LOG( " | conv: %.1fms (%dw)", maxMs, mConvolutionWorker->numWorkers);
        }
        AUDIO_LOG( "\n");
    }

    for (auto &amb : mAmbients) {
        // Ambient sound activation uses EUCLIDEAN distance, matching the
        // original Dark Engine's ambient system (AMBIENT.C). The room portal
        // system was never used for ambient activation — ambients work purely
        // on radius. Room routing handles cross-room propagation for voices
        // in the voice update loop (Step 2b), not here.
        //
        // Start voices BEFORE the source becomes audible so the distance-based
        // volume curve fades in smoothly. Stop at a wider hysteresis radius
        // to avoid rapid start/stop oscillation when the listener hovers near
        // the boundary. Multipliers come from audio.ambient config.
        float dist = glm::length(mListenerPos - amb.position);
        float startRadius = amb.radius * mAmbHysteresisStartMul;
        float stopRadius = amb.radius * mAmbHysteresisStopMul;

        bool alreadyPlaying = (amb.handle != SOUND_HANDLE_INVALID);
        bool inRange = (amb.radius > 0.0f &&
                        (alreadyPlaying ? dist < stopRadius : dist < startRadius));

        if (inRange) {
            // Start voice if not already playing
            bool justCreated = false;
            if (amb.handle == SOUND_HANDLE_INVALID) {
                bool isLooping = !(amb.flags & AMB_ONCE_ONLY);
                if (mSchemaParser) {
                    const SchemaEntry *schema = mSchemaParser->findSchema(amb.schemaName);
                    if (schema && !schema->samples.empty()) {
                        const SchemaSample &sample = schema->samples[0];
                        // Start at volume 0 — stays silent until the sim has
                        // processed this source (next frame) so occlusion is
                        // applied before the voice is audible.
                        amb.handle = startVoice(amb.schemaName, sample.name, amb.position,
                                                schema->playParams.priority, isLooping,
                                                amb.objID, 0.0f);
                    }
                }
                // Environmental ambients use the ambient system's radius curve
                // for distance. Object ambients use Steam Audio distance.
                bool isEnvironmental = (amb.flags & AMB_ENVIRONMENTAL) != 0;
                if (amb.handle != SOUND_HANDLE_INVALID && mVoices.count(amb.handle)) {
                    mVoices[amb.handle]->isAmbient = isEnvironmental;
                    justCreated = true;
                }
                // Fallback: try loading schema name as a raw sound
                if (amb.handle == SOUND_HANDLE_INVALID) {
                    amb.handle = startVoice(amb.schemaName, amb.schemaName, amb.position,
                                            mAmbDefaultPriority, isLooping, amb.objID, 0.0f);
                    if (amb.handle != SOUND_HANDLE_INVALID && mVoices.count(amb.handle)) {
                        mVoices[amb.handle]->isAmbient = isEnvironmental;
                        justCreated = true;
                    }
                }
            }

            // Update volume.
            // Just-created voices stay at 0 — silent defaults in initVoiceDSP
            // ensure iplDirectEffectApply outputs silence until the sim provides
            // real occlusion/distance data (next frame).
            if (!justCreated && amb.handle != SOUND_HANDLE_INVALID) {
                auto it = mVoices.find(amb.handle);
                if (it == mVoices.end()) {
                    amb.handle = SOUND_HANDLE_INVALID;
                    continue;
                }

                bool isEnvironmental = (amb.flags & AMB_ENVIRONMENTAL) != 0;
                float baseVol = schemaVolumeToLinear(amb.volume);
                float distFactor;
                if (isEnvironmental) {
                    distFactor = std::max(0.0f, 1.0f - (dist / amb.radius));
                    // Falloff curve from audio.ambient.falloff_curve.
                    // AMB_NO_FADE flag forces linear regardless of config.
                    bool useQuadratic = (mAmbFalloffCurve == "quadratic")
                                        && !(amb.flags & AMB_NO_FADE);
                    if (useQuadratic)
                        distFactor *= distFactor;
                } else {
                    // Object ambients — Steam Audio handles distance.
                    distFactor = 1.0f;
                }

                // Apply ducking multiplier inline (not as a separate pass)
                // to avoid compounding volume decay across frames.
                float duck = (mReflectionMixNode && mReflectionMixNode->duckingEnabled)
                             ? mReflectionMixNode->duckGain : 1.0f;
                ma_sound_set_volume(&it->second->sound, baseVol * distFactor * duck);
            }
        } else {
            // Out of range — stop voice to free the slot and DSP resources
            if (amb.handle != SOUND_HANDLE_INVALID) {
                haltSound(amb.handle);
                amb.handle = SOUND_HANDLE_INVALID;
            }
        }
    }

    // ── Ducking system (disabled by default) ──
    // When SFX voices are playing, ambient volumes are reduced to keep dialog,
    // footsteps, and other important sounds prominent. The duck envelope smoothly
    // transitions between normal and ducked states to avoid pumping artifacts.
    // The duckGain multiplier is applied inline above (when setting each ambient's
    // volume) to avoid compounding volume decay from read-back-and-multiply.
    if (mReflectionMixNode && mReflectionMixNode->duckingEnabled) {
        auto &rmn = *mReflectionMixNode;

        // Count active non-ambient SFX voices
        int sfxVoices = 0;
        for (auto &[h, v] : mVoices) {
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
        AUDIO_LOG( "[FOOT] h=%d '%s' vol=%.2f spd=%.1f active=%d tail=%d\n",
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
    if (!mRoomService || !mRoomService->isLoaded()) {
        mPortalBlend = {};
        return;
    }

    constexpr float kBlendRadius = 3.0f;

    // If blend is already active, KEEP the existing roomA/roomB assignment
    // and only update the blend weight. This prevents the room assignment from
    // flickering frame-to-frame in narrow hallways where roomFromPoint returns
    // different rooms on each frame.
    //
    // Sustain path: only check plane distance, NOT polygon containment.
    // The initial detection validates isInside() to pick the right portal,
    // but the sustain path must be lenient — the listener can slide along
    // a wall near a portal and temporarily leave the polygon projection
    // while still being in the blend zone. Checking isInside here caused
    // blend to drop and re-engage every frame, oscillating all attenuation.
    // Use a wider exit threshold (2x blend radius) for hysteresis.
    constexpr float kExitRadius = kBlendRadius * 2.0f;
    if (mPortalBlend.active && mPortalBlend.roomA && mPortalBlend.roomB) {
        RoomPortal *activePortal = nullptr;
        float activeDist = 0.0f;

        // Search roomA's portals for one connecting to roomB
        for (uint32_t i = 0; i < mPortalBlend.roomA->getPortalCount(); ++i) {
            RoomPortal *p = mPortalBlend.roomA->getPortal(i);
            if (p && p->getFarRoom() == mPortalBlend.roomB) {
                float d = std::fabs(p->getPlane().getDistance(mListenerPos));
                // Sustain: only plane distance, no isInside — wider exit zone
                if (d < kExitRadius) {
                    activePortal = p;
                    activeDist = p->getPlane().getDistance(mListenerPos);
                    break;
                }
            }
        }

        if (activePortal) {
            // Still in the blend zone — update weight only, keep rooms pinned.
            // Use the original blend radius for the weight calculation so the
            // blend reaches 0/1 at the boundary. Beyond kBlendRadius the weight
            // clamps to 0 or 1, but the blend stays active until kExitRadius.
            float blend = 0.5f - activeDist / (2.0f * kBlendRadius);
            mPortalBlend.blend = std::max(0.0f, std::min(1.0f, blend));
            return;
        }

        // Left the exit zone — deactivate and fall through to fresh detection
        mPortalBlend = {};
    }

    // Fresh portal detection: find the closest portal to the listener
    Room *primaryRoom = mRoomService->roomFromPoint(mListenerPos);
    if (!primaryRoom) {
        mPortalBlend = {};
        return;
    }

    float bestAbsDist = kBlendRadius;
    RoomPortal *bestPortal = nullptr;
    float bestSignedDist = 0.0f;

    uint32_t portalCount = primaryRoom->getPortalCount();
    for (uint32_t i = 0; i < portalCount; ++i) {
        RoomPortal *portal = primaryRoom->getPortal(i);
        if (!portal || !portal->getFarRoom())
            continue;

        float signedDist = portal->getPlane().getDistance(mListenerPos);
        float absDist = std::fabs(signedDist);

        if (absDist >= bestAbsDist)
            continue;

        if (!portal->isInside(mListenerPos))
            continue;

        bestAbsDist = absDist;
        bestPortal = portal;
        bestSignedDist = signedDist;
    }

    if (!bestPortal) {
        mPortalBlend = {};
        return;
    }

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
            AUDIO_LOG( "[PORTAL] propagateSound: NULL room after fallback "
                         "src=(%.1f,%.1f,%.1f)→%s lst=(%.1f,%.1f,%.1f)→%s dist=%.1f\n",
                         sourcePos.x, sourcePos.y, sourcePos.z,
                         sourceRoom ? "OK" : "NULL",
                         listenerPos.x, listenerPos.y, listenerPos.z,
                         listenerRoom ? "OK" : "NULL", dist);
            // Find the closest room to the listener and dump its data
            if (sFailCount == 0) {
                auto &rooms = mRoomService->getAllRooms();
                AUDIO_LOG( "[PORTAL] %zu rooms loaded.\n", rooms.size());

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
                    AUDIO_LOG( "[PORTAL] Closest room %d: center=(%.1f,%.1f,%.1f) dist=%.1f\n",
                                 bestRoom->getRoomID(), c.x, c.y, c.z, bestDist);
                    const Plane *planes = bestRoom->getBoundingPlanes();
                    for (int p = 0; p < 6; ++p) {
                        float dist = planes[p].getDistance(listenerPos);
                        AUDIO_LOG( "[PORTAL]   plane[%d]: n=(%.3f,%.3f,%.3f) d=%.3f dist_to_listener=%.3f %s\n",
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

    AUDIO_LOG( "Baking probes: spacing=%.1f height=%.1f ...\n", spacing, height);

    // Step 1: Generate probes on a uniform floor grid
    IPLProbeArray probeArray = nullptr;
    IPLerror err = iplProbeArrayCreate(mIplContext, &probeArray);
    if (err != IPL_STATUS_SUCCESS) {
        LOG_ERROR("AudioService: iplProbeArrayCreate failed (%d)", err);
        return false;
    }

    // Use the actual acoustic scene bounding box for probe generation.
    // The OBB transform maps a unit cube [-0.5, 0.5]³ to world space.
    // mSceneMin/Max are in FEET; the IPL transform and probe spacing must be
    // in METERS so probe positions land at the same physical points as the
    // (already-converted) static mesh vertices.
    Vector3 center = (mSceneMin + mSceneMax) * 0.5f;
    Vector3 extent = mSceneMax - mSceneMin;
    // Add a small margin (in feet, matching `spacing`) to ensure probes cover edges
    extent += Vector3(spacing * 2.0f);

    IPLMatrix4x4 transform{};
    transform.elements[0][0] = extent.x * kFeetToMeters; // X scale (m)
    transform.elements[1][1] = extent.y * kFeetToMeters; // Y scale (m)
    transform.elements[2][2] = extent.z * kFeetToMeters; // Z scale (m)
    transform.elements[3][0] = center.x * kFeetToMeters; // X translation (m)
    transform.elements[3][1] = center.y * kFeetToMeters; // Y translation (m)
    transform.elements[3][2] = center.z * kFeetToMeters; // Z translation (m)
    transform.elements[3][3] = 1.0f;

    AUDIO_LOG( "Scene bounds (ft): (%.0f,%.0f,%.0f)-(%.0f,%.0f,%.0f) extent=(%.0f,%.0f,%.0f)\n",
                 mSceneMin.x, mSceneMin.y, mSceneMin.z,
                 mSceneMax.x, mSceneMax.y, mSceneMax.z,
                 extent.x, extent.y, extent.z);

    IPLProbeGenerationParams genParams{};
    genParams.type = IPL_PROBEGENERATIONTYPE_UNIFORMFLOOR;
    // spacing/height are passed in feet; IPL grid is in meters.
    genParams.spacing = spacing * kFeetToMeters;
    genParams.height  = height  * kFeetToMeters;
    genParams.transform = transform;

    iplProbeArrayGenerateProbes(probeArray, mIplScene, &genParams);

    int numProbes = iplProbeArrayGetNumProbes(probeArray);
    AUDIO_LOG( "Generated %d probes (spacing=%.1f, height=%.1f)\n",
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
    AUDIO_LOG( "Baking pathing data for %d probes...\n", numProbes);

    IPLBakedDataIdentifier pathId{};
    pathId.type = IPL_BAKEDDATATYPE_PATHING;
    pathId.variation = IPL_BAKEDDATAVARIATION_DYNAMIC;

    // Pathing bake — all distance-shaped fields cross into IPL, so feet → meters.
    // `spacing` and `mPropagationMaxDist` live in engine feet; convert here.
    IPLPathBakeParams bakeParams{};
    bakeParams.scene = mIplScene;
    bakeParams.probeBatch = probeBatch;
    bakeParams.identifier = pathId;
    bakeParams.numSamples = 4;                                   // visibility test samples (rays = numSamples²)
    bakeParams.radius = spacing * kFeetToMeters;                 // probe influence radius (m)
    bakeParams.threshold = 0.1f;                                 // visibility threshold (dimensionless)
    bakeParams.visRange  = mPropagationMaxDist * kFeetToMeters;  // max visibility distance (m)
    bakeParams.pathRange = mPropagationMaxDist * kFeetToMeters;  // max path length (m)
    bakeParams.numThreads = 4;                                   // parallel baking

    auto bakeStart = std::chrono::steady_clock::now();

    iplPathBakerBake(mIplContext, &bakeParams,
        [](IPLfloat32 p, void *userData) {
            auto *prog = static_cast<std::atomic<float> *>(userData);
            if (prog) prog->store(p, std::memory_order_relaxed);
        },
        progress);

    auto bakeEnd = std::chrono::steady_clock::now();
    float bakeSec = std::chrono::duration<float>(bakeEnd - bakeStart).count();
    AUDIO_LOG( "Pathing bake complete: %d probes in %.1f seconds\n",
                 numProbes, bakeSec);

    // Step 3b: Bake reflection IRs at each probe position.
    // This pre-computes reverb impulse responses so that voices outside the
    // real-time top-N can use baked reverb instead of being dry.
    // Uses REVERB variation — one bake covers all sources (listener-position-based).
    AUDIO_LOG( "Baking reflection IRs for %d probes (rays=%d bounces=%d duration=%.1fs)...\n",
                 numProbes, mReflectionNumRays, mReflectionNumBounces, mReflectionDuration);

    IPLBakedDataIdentifier reflId{};
    reflId.type = IPL_BAKEDDATATYPE_REFLECTIONS;
    reflId.variation = IPL_BAKEDDATAVARIATION_REVERB;

    unsigned int hwThreads = std::thread::hardware_concurrency();
    int bakeThreads = (mSimulatorThreadsCfg > 0)
        ? mSimulatorThreadsCfg
        : std::max(2u, hwThreads > 2 ? hwThreads - 2 : 2u);

    IPLReflectionsBakeParams reflBakeParams{};
    reflBakeParams.scene = mIplScene;
    reflBakeParams.probeBatch = probeBatch;
    reflBakeParams.sceneType = (mSceneTypeCfg == "embree")
        ? IPL_SCENETYPE_EMBREE : IPL_SCENETYPE_DEFAULT;
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
    // irradianceMinDistance is in METERS (Steam Audio convention) — clamps the
    // inner radius of the irradiance integral to avoid singularities for sources
    // very close to a probe. 1.0m ≈ 3.28 ft is a sensible physical default.
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
    AUDIO_LOG( "Reflection bake complete: %d probes in %.1f seconds\n",
                 numProbes, reflBakeSec);

    // Step 4: Serialize to memory buffer
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

    // Step 5: Round-trip validation — deserialize in memory and verify probe count
    // matches what we baked. Catches serialization bugs before they hit disk.
    {
        IPLSerializedObjectSettings rtSettings{};
        rtSettings.data = data;
        rtSettings.size = dataSize;
        IPLSerializedObject rtObj = nullptr;
        IPLProbeBatch rtBatch = nullptr;
        err = iplSerializedObjectCreate(mIplContext, &rtSettings, &rtObj);
        if (err != IPL_STATUS_SUCCESS) {
            LOG_ERROR("AudioService: round-trip validation failed — "
                      "iplSerializedObjectCreate returned %d", err);
            iplSerializedObjectRelease(&serializedObject);
            iplProbeBatchRelease(&probeBatch);
            return false;
        }
        err = iplProbeBatchLoad(mIplContext, rtObj, &rtBatch);
        iplSerializedObjectRelease(&rtObj);
        if (err != IPL_STATUS_SUCCESS) {
            LOG_ERROR("AudioService: round-trip validation failed — "
                      "iplProbeBatchLoad returned %d", err);
            iplSerializedObjectRelease(&serializedObject);
            iplProbeBatchRelease(&probeBatch);
            return false;
        }
        int rtCount = iplProbeBatchGetNumProbes(rtBatch);
        iplProbeBatchRelease(&rtBatch);
        if (rtCount != numProbes) {
            LOG_ERROR("AudioService: round-trip validation failed — "
                      "expected %d probes, deserialized %d", numProbes, rtCount);
            iplSerializedObjectRelease(&serializedObject);
            iplProbeBatchRelease(&probeBatch);
            return false;
        }
        AUDIO_LOG( "Round-trip validation passed (%d probes)\n", rtCount);
    }

    // Step 6: Write to disk with integrity header (atomic tmp+rename)
    bool writeOk = writeProbeFile(
        outputPath,
        reinterpret_cast<const uint8_t *>(data),
        static_cast<size_t>(dataSize),
        static_cast<uint32_t>(numProbes));

    iplSerializedObjectRelease(&serializedObject);
    iplProbeBatchRelease(&probeBatch);

    if (!writeOk) {
        LOG_ERROR("AudioService: failed to write probe file '%s'", outputPath.c_str());
        return false;
    }

    AUDIO_LOG( "Saved %d probes to '%s' (%zu bytes + %zu header)\n",
                 numProbes, outputPath.c_str(), static_cast<size_t>(dataSize),
                 kProbeFileHeaderSize);
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

    // Load and validate the probe file (header + CRC check)
    ProbeFileHeader hdr;
    std::vector<uint8_t> payload;
    ProbeFileStatus status = loadProbeFile(probePath, hdr, payload);

    if (status == ProbeFileStatus::FileNotFound) {
        LOG_INFO("AudioService: no probe file at '%s' — pathing disabled",
                 probePath.c_str());
        return false;
    }

    if (status != ProbeFileStatus::Ok) {
        LOG_ERROR("AudioService: probe file '%s' failed validation: %s — "
                  "delete and re-bake",
                  probePath.c_str(), probeFileStatusString(status));
        return false;
    }

    // Deserialize the validated payload
    IPLSerializedObjectSettings soSettings{};
    soSettings.data = reinterpret_cast<IPLbyte *>(payload.data());
    soSettings.size = static_cast<IPLsize>(payload.size());

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

    // Cross-check: probe count from header must match what Steam Audio loaded
    if (mProbeCount != static_cast<int>(hdr.probeCount)) {
        LOG_ERROR("AudioService: probe count mismatch — header says %u, "
                  "Steam Audio loaded %d — discarding",
                  hdr.probeCount, mProbeCount);
        iplProbeBatchRelease(&mIplProbeBatch);
        mIplProbeBatch = nullptr;
        mProbeCount = 0;
        return false;
    }

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

    AUDIO_LOG( "AudioService: loaded %d probes from '%s' "
                 "(reflections=%s, refl_size=%zu bytes, crc=0x%08x)\n",
                 mProbeCount, probePath.c_str(),
                 mProbesHaveReflections ? "yes" : "no",
                 static_cast<size_t>(reflDataSize), hdr.crc32);
    return true;
}

} // namespace Darkness
