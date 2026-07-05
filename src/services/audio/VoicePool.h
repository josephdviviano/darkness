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

#ifndef __VOICEPOOL_H
#define __VOICEPOOL_H

// VoicePool — owns the active-voice lifecycle. Encapsulates ActiveVoice
// (PCM data + miniaudio decoder/sound + Steam Audio DSP node), the
// handle→voice map, monotonic handle allocation, and the per-frame
// finished-voice sweep. AudioService still owns voice startup
// (createVoiceSource, initVoiceDSP, ma_sound_start) and the side-effects
// of teardown (detaching Steam Audio sources from simulators) via the
// cleanup hook.
//
// Threading: pool is main-thread only. The audio thread follows captured
// ActiveVoice* pointers set at startup; miniaudio's end-callback writes
// sourceEnded/finished atomically and the main-thread sweep reads them.

#include "DarknessMath.h"
#include "AudioService.h"        // SoundHandle, SOUND_HANDLE_INVALID
#include "AudioUnits.h"          // kDefaultDeviceFrameSize
#include "CRFSoundLoader.h"      // SoundData (ActiveVoice owns one)
#include "room/RoomService.h"    // SoundPropInfo

#include <miniaudio.h>
#include <phonon.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace Darkness {

// Opaque to VoicePool — only used as a raw pointer in SteamAudioDSPNode.
struct ConvolutionWorker;

// ── Multi-path ambisonics sub-source state machine ──
// Each voice may render up to kMaxSubSources directional contributions
// (one per propagation path). Slots are keyed by predecessorRoomID so
// per-slot DSP state stays attached to the path it represents across BFS
// reorders.
//   FREE     — no path assigned, DSP idle
//   ACTIVE   — path assigned, normal processing
//   DRAINING — path removed/evicted, targetGain ramping to 0; DSP keeps
//              running so the LPF tail flushes. Returns to FREE when
//              currentGain decays below 1e-4.
enum class SubSourceState : uint8_t {
    Free     = 0,
    Active   = 1,
    Draining = 2,
};

struct SubSource {
    // Per-slot IPL handles. directSource is registered with the direct
    // simulator (DirectSimulator worker) so iplSimulatorRunDirect
    // computes per-path distance attenuation, air absorption, occlusion,
    // transmission from THIS path's virtual position (doorway anchor for
    // cross-room, real source for same-room). Pre-allocated in
    // initVoiceDSP, released in ~ActiveVoice.
    IPLSource             directSource       = nullptr;
    IPLDirectEffect       directEffect       = nullptr;
    IPLBinauralEffect     binauralEffect     = nullptr;

    // Direct-sim outputs for this slot. Default-initialised so a brand-new
    // slot still produces sensible passthrough (silent) audio until the
    // first real sim result lands.
    IPLDirectEffectParams targetDirectParams{};

    int32_t        predecessorRoomID = -1;   // -1 sentinel = FREE
    SubSourceState state             = SubSourceState::Free;

    // Audio-thread ramp targets (main thread writes once per frame).
    // No normalization: Steam Audio's per-path inv-d² carries amplitude;
    // targetGain is a unit-scale fade for slot transitions only.
    // targetDoorBlocking is raw [0,1]; audio thread converts to LPF α
    // per callback.
    IPLVector3 targetDir{1.0f, 0.0f, 0.0f};
    float      targetGain         = 0.0f;
    float      targetDoorBlocking = 0.0f;

    IPLVector3 currentDir{1.0f, 0.0f, 0.0f};
    float      currentGain      = 0.0f;
    float      currentDoorAlpha = 1.0f;  // LPF α (1.0 = passthrough)

    // Per-slot door LPF IIR memory (stereo, post-binaural).
    float lpfStateL = 0.0f;
    float lpfStateR = 0.0f;
};

// Match SoundPropParams::maxPaths clamp [1,4].
inline constexpr int kMaxSubSources = 4;

/// Custom miniaudio node — Steam Audio DSP processing between a voice's
/// ma_sound (mono) and the engine endpoint (stereo). The process callback
/// itself lives in AudioService.cpp.
struct SteamAudioDSPNode {
    ma_node_base base;  // must be first (miniaudio node graph requirement)

    IPLHRTF hrtf = nullptr;  // shared ref; HRTF lifetime owned by AudioService

    // Written by main thread in loopStep, read by audio thread. Partial-
    // write tearing is acceptable: worst case is one slightly wrong
    // ~23ms frame.
    IPLDirectEffectParams directParams{};
    IPLVector3 direction{1.0f, 0.0f, 0.0f};  // listener-to-source (listener-local frame)

    // True iff this voice's HRTF direction was computed toward a portal
    // anchor (cross-room) rather than the real source. NOT a gate on
    // portal-attenuation or door-LPF — both apply continuously every
    // callback, with portalAttenuation=1.0 + portalBlocking=0.0
    // collapsing to no-op for same-room voices.
    bool usePortalRouting = false;
    bool skipAttenuation = false;        // true for player-emitted sounds (footsteps within 5 units)

    // Per-voice override for IPLBinauralEffectParams::spatialBlend.
    // 1.0 = full HRTF; 0.0 = mono passthrough. Used by AMB_ENVIRONMENTAL
    // voices to feel less like a point source.
    std::atomic<float> spatialBlendOverride{1.0f};

    // Footstep reflection diagnostics. Set at startVoice for "foot_*" /
    // "land_*" schemas; one-shot logs gate on this.
    bool isFootstepDiag = false;

    // Voice identity forwarded to ConvolutionWorker::VoiceSlot so the
    // reflection workers can label [REFLECTION_VOICE] diagnostics. The
    // schema c-string points into the owning ActiveVoice's std::string;
    // the worker's iteration is bounded by removeVoiceSource draining
    // workers before voice destruction, so the pointer is stable.
    int         voiceHandle      = -1;
    const char *voiceSchemaCStr  = nullptr;
    std::atomic<int> reflInputLogCount{0};
    std::atomic<int> dryBalLogCount{0};

    // Per-voice lifetime peak tracking (atomic max on every audio callback,
    // logged once at CLEANUP as [VOICE_PEAK]). Captures the loudest sample
    // across the voice's entire lifetime — more reliable than rate-limited
    // sampling which can land on decay tails.
    std::atomic<float> lifetimePeakL{0.0f};
    std::atomic<float> lifetimePeakR{0.0f};
    std::atomic<int>   lifetimeFrameCount{0};
    // Captured exactly once on first callback — distinguishes "lucky frame"
    // peaks from "transient phase" peaks.
    std::atomic<float> firstCallbackPeakL{0.0f};
    std::atomic<float> firstCallbackPeakR{0.0f};
    // Pipeline-stage peaks: monoInPeak (raw decoded × volume, pre-DSP),
    // monoOutPeak (post iplDirectEffect), stereoMax (= lifetimePeak, post
    // binaural). Localises which stage introduces variation.
    std::atomic<float> monoInPeak{0.0f};
    std::atomic<float> monoOutPeak{0.0f};
    // Direction at the lifetime-peak callback — reveals listener-pose flutter
    // if it varies between voices that should share direction.
    std::atomic<float> directionAtPeakX{0.0f};
    std::atomic<float> directionAtPeakY{0.0f};
    std::atomic<float> directionAtPeakZ{0.0f};

    // Excess-path attenuation through the portal graph.
    //   same-room:        1.0
    //   cross-room reachable: (realDist/effDist)² ∈ (0,1]
    //   cross-room unreachable: 0.0
    // Applied unconditionally — same-room is a no-op multiply, so there is
    // no discontinuity at the portal boundary.
    float portalAttenuation = 1.0f;
    float portalBlocking = 0.0f;         // 0=open, 1=fully blocked (for LPF)
    IPLVector3 portalDirection{1.0f, 0.0f, 0.0f}; // direction toward virtual source (portal centre)

    // Per-voice LPF state for door blocking on the WET (reflection-send)
    // bus. The dry bus uses per-slot LPF state inside each SubSource
    // (slot.lpfStateL/R) driven by the path's individual blocking. Wet
    // bus stays single-source via centroid prop.doorBlocking since
    // reflections are inherently diffuse.
    float reflSendLpfState = 0.0f;

    // Audio-thread counter: consecutive callbacks with silent reflection-send
    // input. Used to elide iplReflectionEffectApply once the conv delay line
    // and parametric FDN have decayed to zero. Threshold must be ≥ the IR
    // length in callbacks so the delay line fully drains before skipping —
    // skipping earlier would freeze stale content in the delay line.
    //
    // INITIALISED AT 0: voices start in the active path. Voices born
    // genuinely silent (out-of-range / pathing-isolated) will count up
    // to kReflSilentSkipFrames+1 and elide naturally. Initializing high
    // (INT_MAX/2) was tried briefly (2026-05-26) to skip the wasted-apply
    // window on never-audible spawns; reverted because the CPU savings
    // were small with the post-hysteresis ambient-population reduction,
    // and the symmetric init makes the lifecycle easier to reason about.
    int framesSinceNonzeroReflInput = 0;

    // Per-voice ramp state for portal-routing scalars (audio thread only).
    // Slews toward main-thread targets at a fixed rate so abrupt frame-to-
    // frame target changes resolve smoothly within ~10ms. Initialised to
    // open/unattenuated so a new voice doesn't ramp up from silence.
    float currentPortalAtten = 1.0f;
    float currentDoorAlpha   = 1.0f;  // 1.0 = passthrough (open door)

    // Multi-path ambisonics sub-source slots. Main thread writes assignments
    // + ramp targets in loopStep keyed by predecessorRoomID. Audio thread
    // reads to run per-path direct + binaural effects and sum coherently.
    std::array<SubSource, kMaxSubSources> subSources{};

    // Per-voice reflection effect. Output flows through the convolution
    // worker pool; the worker pool sums ambisonics manually.
    IPLReflectionEffect reflectionEffect = nullptr;
    ConvolutionWorker *convWorker = nullptr;

    IPLReflectionEffectParams reflectionParams{};
    std::atomic<bool> reflectionsActive{false};

    // ── IR-energy elision groundwork (measurement only) ──
    //
    // Mean-square energy of this voice's wet W (omnidirectional, channel-0)
    // convolution OUTPUT, measured by the convolution sub-worker right after
    // iplReflectionEffectApply and written back here (worker writes via the
    // raw pointer VoiceSlot::outEnergyW, guarded by the same validityToken
    // drain that protects the rest of the slot's voice-derived data). Read on
    // the main thread for the [REFL_IR_TICK] diagnostic.
    //
    // Why OUTPUT energy, not IR energy: Steam Audio's reflection IR
    // (outputs.reflections.ir) is an OPAQUE handle (IPLReflectionEffectIR) —
    // its samples are not exposed (see ConvolutionWorkerPool.cpp comment at the
    // [REFL_FIRST_APPLY] block). The convolution output W channel is the only
    // available ground truth for "how much reverb energy this voice produces."
    //
    // This is the foundation for eliding the convolution of voices whose reverb
    // is inaudible — specifically the STATICSOURCE occluded-emitter case where
    // the input mono is non-zero (no pathing gate) but the self-occluding baked
    // IR yields silent output, which the existing input-silence counter cannot
    // detect. It is OBSERVATIONAL ONLY today: it does NOT yet gate elision,
    // because gating on output energy requires a periodic re-probe (an elided
    // voice stops being measured, so it could never un-elide when its IR
    // becomes audible again). That re-probe loop is specified in
    // HANDOFF.STATICSOURCE.md §4.4 and lands with the STATICSOURCE work.
    //
    // Default 0.0 (never measured yet). Relaxed atomic.
    std::atomic<float> reflectionOutEnergyW{0.0f};

    // Per-voice IPLPathEffect (Phase 4 — Steam Audio sole authority for
    // player routing + attenuation). Created in initVoiceDSP for non-
    // player-emitted voices, released in removeVoiceSource/~ActiveVoice
    // symmetric to reflectionEffect. Configured with spatialize=IPL_TRUE +
    // binaural per Valve's Unity reference; output is HRTF-binaural stereo
    // that mixes additively into the dry bus on top of the direct binaural
    // path. Null for player-emitted voices (the player IS the listener; the
    // pathing graph collapses to a self-loop and the effect has nothing to
    // route).
    IPLPathEffect pathEffect = nullptr;

    // Audio-thread mirror of ActiveVoice::pathingSource. The audio
    // thread calls iplSourceGetOutputs on this directly inside its
    // path-effect block — same pattern as Valve's Unity / Unreal /
    // FMOD / Wwise reference plugins. Reading on the audio thread
    // (rather than staging a copy from the main thread) avoids the
    // cross-thread tear on IPLPathEffectParams that the staged-copy
    // pattern had.
    //
    // Lifetime: set by createVoiceSource right after iplSourceCreate
    // succeeds; never modified after that. The release of this handle
    // is deferred to ~ActiveVoice (AFTER ma_node_uninit drains the
    // audio thread), so the audio thread can safely dereference
    // without racing the main thread's removeVoiceSource call. The
    // simulator's source-list membership is still toggled by
    // removeVoiceSource via iplSourceRemove / queueSourceRemove —
    // only the iplSourceRelease (which frees the IPL object) is
    // deferred.
    IPLSource pathingSource = nullptr;

    // Gates iplPathEffectApply on the audio thread. Starts false; the
    // main thread sets it true once the pathing source is live (after
    // the first non-skipped loopStep) and false in the out-of-range
    // and pending-source-add branches. When false, the audio thread
    // skips the path effect and only the dry binaural reaches output.
    std::atomic<bool> pathTargetValid{false};

    // Stereo scratch for iplPathEffectApply output. Allocated once in
    // initVoiceDSP at frameSize × 2; mixed additively into the voice's
    // dry stereoL/stereoR after the per-slot binaural pipeline.
    std::vector<float> pathOutL;
    std::vector<float> pathOutR;

    // Scratch buffers (allocated once at init, never reallocated — safe
    // for audio thread).
    std::vector<float> monoScratch;        // raw downmix, preserved for convolution
    std::vector<float> directEffectOut;    // mono output from iplDirectEffectApply
    std::vector<float> stereoL;            // accumulated binaural across slots
    std::vector<float> stereoR;
    // Per-slot binaural scratch — reused across slots since iteration is
    // sequential.
    std::vector<float> subSlotStereoL;
    std::vector<float> subSlotStereoR;
    // Ambisonics scratch for reflection convolution (W/Y/Z/X).
    std::vector<float> ambiScratch0;
    std::vector<float> ambiScratch1;
    std::vector<float> ambiScratch2;
    std::vector<float> ambiScratch3;
    // Used when reflection pipeline runs at reduced rate.
    std::vector<float> decimatedMono;

    int frameSize = static_cast<int>(kDefaultDeviceFrameSize);
    int reflectionFrameSize = static_cast<int>(kDefaultDeviceFrameSize);  // frameSize / rateDivisor
    int rateDivisor = 1;  // 1=full, 2=half, 4=quarter
    std::atomic<bool> effectsReady{false};   // true when effects + node initialised
    bool nodeInitialized = false;            // true when ma_node_init succeeded (main thread)

    // Shared validity token for the convolution worker. The shared_ptr
    // prevents use-after-free of the token itself even after ~ActiveVoice
    // sets the bool to false.
    std::shared_ptr<std::atomic<bool>> validityToken;

    // Diagnostics (audio thread writes, main thread reads).
    std::atomic<uint64_t> callCount{0};
    std::atomic<float> peakInput{0.0f};
    std::atomic<float> peakOutput{0.0f};
    std::atomic<uint32_t> lastFrameCount{0};
    std::atomic<float> lastAtten{1.0f};

    // Per-voice peak of mono signal staged for convolution (= reverb-send
    // level post reflAtten). Atomic max written every callback; read+reset
    // by the 5s [AMB] dump to attribute wet-bus energy.
    std::atomic<float> reflSendPeak{0.0f};

    // NaN/Inf guard counters (audio thread). Each location that sanitises
    // non-finite samples increments its own counter; logs are rate-limited
    // per voice.
    std::atomic<uint32_t> nanCountDirect{0};
    std::atomic<uint32_t> nanCountBinaural{0};
    std::atomic<uint32_t> nanCountLpf{0};
    std::atomic<uint32_t> nanCountRamp{0};
    // Catches NaN/Inf in slot.mono after the reflSend LPF and *before*
    // iplReflectionEffectApply. A single NaN here gets convolved against
    // the full IR, spreading grain across the wet bus.
    std::atomic<uint32_t> nanCountReflInput{0};
};

/// An active voice playing through miniaudio. Non-copyable/non-movable
/// because ma_decoder/ma_sound/ma_node carry internal pointers.
/// Threading: `finished` is set atomically by miniaudio's end callback
/// (audio thread) and read by cleanupFinished (main thread).
struct ActiveVoice {
    SoundData data;        // WAV bytes — must outlive decoder
    ma_decoder decoder;
    ma_sound sound;
    SoundHandle handle = SOUND_HANDLE_INVALID;
    bool initialized = false;
    std::atomic<bool> finished{false};

    // Voice stays alive after source audio ends to let the per-voice
    // convolution tail ring out (silence fed into convolution until
    // timer expires).
    std::atomic<bool> sourceEnded{false};
    float tailTimer = 0.0f;

    std::string schemaName;
    int priority = 128;                // 0-255, higher = more important
    int objID = 0;                     // 0 = positional
    bool playerEmitted = false;        // footsteps/landing — skip DSP attenuation
    bool skipPortalRouting = false;    // door sounds — Steam Audio yes, portal blocking no
    bool isAmbient = false;            // distance handled by ambient system
    bool loggedReflActivationMain = false;

    // Per-voice max audible portal-graph distance — BFS terminates beyond
    // this. Ambient voices override at startVoice with a schema-radius-
    // derived value so a wind ambient with radius=25 doesn't propagate
    // 200ft through open corridors.
    float maxAudibleDist = 200.0f;

    // Per-voice distance-attenuation rolloff factor, sourced from the
    // schema's P$SchAttFac (Dark Engine attenuationFactor). Drives
    // Steam Audio's IPLDistanceAttenuationModel::minDistance: minDistance
    // = 1m × attenuationFactor, so attenuationFactor=20 keeps the sound
    // at full volume out to 20m before 1/d falloff kicks in. Default 1.0
    // matches Steam Audio's DEFAULT model (1m full-volume zone). Set at
    // voice spawn from the schema (via voiceSetAttenuationFactor) and
    // read every frame in the per-voice setInputs block.
    float attenuationFactor = 1.0f;

    // Last propagation result — diagnostic only, recomputed every frame.
    SoundPropInfo cachedProp{};
    // Per-portal anchor bend points along the primary path (source→listener).
    // Empty for clean-threaded paths. Diagnostic only — surfaced via
    // getVoiceSpatialSnapshots() for the renderer's show_vpos overlay.
    std::vector<Vector3> cachedChain;

    // Steam Audio sim sources split across three simulators so the direct
    // path is never blocked by the reflection or pathing sim background
    // iterations. All nullptr if scene not ready or non-spatial. The
    // pathing source is also nullptr for `playerEmitted` voices (the
    // player IS the listener, so the pathing graph collapses to a
    // self-loop and the iteration cost is wasted).
    IPLSource directSource     = nullptr;  // owned by mDirectSim
    IPLSource reflectionSource = nullptr;  // owned by mReflectionSim
    IPLSource pathingSource    = nullptr;  // owned by mPathingSim

    // Reflection-simulator completed-cycle counter at the moment THIS
    // voice's reflectionSource was actually added to the simulator
    // (immediate path: createVoiceSource right after iplSourceAdd;
    // deferred path: flushPendingAdds callback). The simCycleAfterAdd
    // gate in AudioService::loopStep refuses to copy outputs.reflections
    // into dspNode.reflectionParams until
    // ReflectionSimulator::completedCycles() exceeds this value —
    // guaranteeing the simulator has run at least one full iteration
    // containing this source. Before that point, outputs.reflections.ir
    // may carry a non-null handle with all-zero content (simulator "no
    // output yet" state); shipping that produced ~1 callback of
    // permanently-silent wet bus on voice spawn — [REFL_FIRST_APPLY]
    // irNorm=0 was the smoking gun.
    uint64_t reflectionSimCycleAtAdd = 0;

    // Direct-simulator counterpart of reflectionSimCycleAtAdd (PR 1b —
    // the direct sim now also runs on a background worker, so the same
    // "outputs unwritten until one full iteration has contained this
    // source" race exists). Captured at iplSourceAdd time (immediate
    // path) or DirectSimulator::flushPendingAdds callback (deferred
    // path). The harvest pass in loopStep keeps the initVoiceDSP
    // per-class default directParams (and the slots' silent defaults)
    // until DirectSimulator::completedCycles() exceeds this value —
    // without the gate, the first harvest after an add read back the
    // SimulationData create-time seeds (neutral distanceAttenuation =
    // 1.0), audibly popping distant spawns at full volume for a frame.
    uint64_t directSimCycleAtAdd = 0;

    // Per-voice volumetric-occlusion sphere radius (engine feet).
    // Computed in createVoiceSource by raycasting from the source
    // position in N uniformly-distributed directions; the radius is
    // capped so the sphere doesn't extend past the nearest wall
    // surface in any direction, with a small absolute floor and an
    // upper cap from the global AudioOcclusion::getRadius() value.
    // Feeds IPLSimulationInputs::occlusionRadius in the per-voice
    // setInputs block in loopStep (replacing the prior single global
    // value). Negative sentinel means "not yet computed" — the
    // setInputs block falls back to the global radius in that case
    // (covers the headless-no-raycaster path).
    float occlusionRadiusFt = -1.0f;

    // World-space position (updated for moving objects).
    Vector3 worldPos{0.0f, 0.0f, 0.0f};

    // Steam Audio DSP node — direct + binaural in audio thread, between
    // ma_sound output and the engine endpoint.
    SteamAudioDSPNode dspNode;

    ActiveVoice() {
        std::memset(&decoder, 0, sizeof(decoder));
        std::memset(&sound, 0, sizeof(sound));
    }

    // Out-of-line in VoicePool.cpp: marks DSP node inactive, drains the
    // convolution worker, releases Steam Audio effects, tears down
    // ma_sound + decoder.
    ~ActiveVoice();

    ActiveVoice(const ActiveVoice &) = delete;
    ActiveVoice &operator=(const ActiveVoice &) = delete;
    ActiveVoice(ActiveVoice &&) = delete;
    ActiveVoice &operator=(ActiveVoice &&) = delete;
};

/// Owns the active-voice map and handle allocator. Single-threaded (main
/// thread only) — audio thread reads/writes already-captured ActiveVoice*.
class VoicePool {
public:
    using Map = std::unordered_map<SoundHandle, std::unique_ptr<ActiveVoice>>;

    /// Cleanup hook — invoked exactly once per voice being removed (by
    /// cleanupFinished/remove/clear) BEFORE the unique_ptr is destroyed.
    /// Where AudioService runs per-voice diagnostics and detaches Steam
    /// Audio sources from simulators. May be empty.
    using CleanupHook = std::function<void(ActiveVoice &)>;

    VoicePool() = default;
    ~VoicePool() = default;

    VoicePool(const VoicePool &) = delete;
    VoicePool &operator=(const VoicePool &) = delete;
    VoicePool(VoicePool &&) = delete;
    VoicePool &operator=(VoicePool &&) = delete;

    SoundHandle allocate() { return mNextHandle++; }

    /// Reset the handle allocator — called from haltAll() so post-reset
    /// numbering starts at zero (keeps diagnostic logs readable).
    void resetAllocator() { mNextHandle = 0; }

    Map       &voices()       { return mVoices; }
    const Map &voices() const { return mVoices; }

    ActiveVoice *find(SoundHandle h) {
        if (h == SOUND_HANDLE_INVALID) return nullptr;
        auto it = mVoices.find(h);
        return (it == mVoices.end()) ? nullptr : it->second.get();
    }
    const ActiveVoice *find(SoundHandle h) const {
        if (h == SOUND_HANDLE_INVALID) return nullptr;
        auto it = mVoices.find(h);
        return (it == mVoices.end()) ? nullptr : it->second.get();
    }

    bool exists(SoundHandle h) const {
        return h != SOUND_HANDLE_INVALID && mVoices.count(h) > 0;
    }

    void insert(SoundHandle h, std::unique_ptr<ActiveVoice> voice) {
        mVoices[h] = std::move(voice);
    }

    /// Remove one voice. Hook runs BEFORE the unique_ptr is destroyed
    /// (i.e. while still discoverable by raw pointer).
    bool remove(SoundHandle h, CleanupHook hook = {});

    /// Drop every voice. Hook runs for each in unspecified order.
    /// Does NOT touch the handle allocator.
    void clear(CleanupHook hook = {});

    std::size_t size()  const { return mVoices.size(); }
    bool        empty() const { return mVoices.empty(); }

    /// Sweep finished voices. Hook runs for each removed voice in
    /// iteration order. Returns the number removed.
    std::size_t cleanupFinished(CleanupHook hook = {});

private:
    Map mVoices;
    SoundHandle mNextHandle = 0;
};

} // namespace Darkness

#endif // __VOICEPOOL_H
