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

// VoicePool — owns the active-voice lifecycle that used to live directly on
// AudioService. The pool encapsulates:
//   • the ActiveVoice struct (PCM data + miniaudio decoder/sound + Steam
//     Audio DSP node) — fully defined here because every translation unit
//     that needs to touch voice state pulls in this header
//   • the handle→voice map
//   • monotonic handle allocation
//   • the per-frame "drop voices whose end-callback fired" sweep
//
// AudioService still owns voice STARTUP (createVoiceSource, initVoiceDSP,
// ma_sound_start) and per-voice DESTRUCTION beyond the destructor (the
// removeVoiceSource side of the teardown that detaches Steam Audio sources
// from the simulators). The pool's cleanupFinished() accepts a hook so the
// caller can perform those side-effects + diagnostic logging in the order
// already established before extraction.
//
// Threading: the pool is touched only from the main thread. The audio
// thread never iterates the voice map — it follows captured ActiveVoice*
// pointers set during voice startup. miniaudio's end-callback writes to
// ActiveVoice::sourceEnded / ActiveVoice::finished atomically; the pool's
// cleanup sweep reads those atomics on the main thread.

#include "DarknessMath.h"
#include "AudioService.h"        // for SoundHandle, SOUND_HANDLE_INVALID
#include "CRFSoundLoader.h"      // for SoundData (full def — ActiveVoice owns one)
#include "room/RoomService.h"    // for SoundPropInfo

// miniaudio: ActiveVoice embeds ma_decoder and ma_sound by value (decoder
// owns internal buffers; ma_sound owns playback state). The implementation
// macro must NOT be defined in this header — only AudioService.cpp owns the
// miniaudio implementation TU.
#include <miniaudio.h>

// Steam Audio C API — ActiveVoice carries IPLSource handles and a
// SteamAudioDSPNode whose members are mostly IPL* types.
#include <phonon.h>

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

// Forward declarations — ConvolutionWorker is opaque to VoicePool (we only
// hold a raw pointer); SteamAudioDSPNode references it.
struct ConvolutionWorker;

/// Custom miniaudio node for Steam Audio DSP processing.
/// Sits between a voice's ma_sound (mono) and the engine endpoint (stereo).
/// Applies IPLDirectEffect (frequency-dependent attenuation) followed by
/// IPLBinauralEffect (HRTF spatialization) on the audio thread.
///
/// Defined here (rather than in AudioService.cpp) because ActiveVoice
/// embeds one by value and ActiveVoice is now owned by VoicePool. The
/// implementation of the node's process callback still lives in
/// AudioService.cpp — only the layout moves.
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
    // True iff this voice's HRTF direction was computed toward the portal
    // anchor (cross-room) rather than toward the real source position.
    // Used by the post-sim block downstream to skip the same-room direction
    // recompute. Note: this is NOT a gate on portal-attenuation or
    // door-LPF anymore — both are applied continuously every callback,
    // with portalAttenuation = 1.0 and portalBlocking = 0.0 collapsing to
    // a no-op for same-room voices. Renaming to keep semantic clear.
    bool usePortalRouting = false;
    bool skipAttenuation = false;        // true for player-emitted sounds (footsteps, within 5 units)

    // Per-voice override for IPLBinauralEffectParams::spatialBlend.
    // 1.0 = full HRTF panning (point-source localization).
    // 0.0 = mono passthrough (no directional cue).
    // Intermediate values cross-fade between the two. Used by Ambient
    // voices flagged AMB_ENVIRONMENTAL to feel less like a point source
    // (wind, church reverberance, room tone). Object-attached ambients
    // and Normal voices should leave this at 1.0 to preserve their
    // directional cue. Default 1.0 matches the previous global behavior;
    // loopStep writes this to the per-class value once at activation.
    std::atomic<float> spatialBlendOverride{1.0f};
    // ── Footstep reflection diagnostic ──
    // Set true at startVoice time for "foot_*" / "land_*" schemas. The audio
    // and main threads both gate one-shot diagnostic logs on this so we can
    // confirm whether transient sounds reach the convolution path. Cleared
    // when the voice is destroyed.
    bool isFootstepDiag = false;
    std::atomic<int> reflInputLogCount{0};  // audio thread: limit log spam to first few frames
    std::atomic<int> dryBalLogCount{0};     // audio thread: rate-limit per-voice dry-bus L/R balance log

    // Per-voice lifetime peak tracking.  Updated on every audio callback
    // (atomic max), read once on cleanup.  Captures the absolute loudest
    // sample the voice produced across its entire lifetime — unlike the
    // rate-limited DRY_BAL log which samples one callback per voice and
    // catches whatever frame timing happens to align with, producing
    // misleading "voice N was quiet" data when really we just sampled the
    // decay tail of N's waveform.  Logged at CLEANUP as [VOICE_PEAK].
    std::atomic<float> lifetimePeakL{0.0f};
    std::atomic<float> lifetimePeakR{0.0f};
    std::atomic<int>   lifetimeFrameCount{0};  // audio callbacks this voice processed
    // First-callback peak — captured exactly once on the audio thread.
    // Differentiates "voice's max came on frame 1 and we got lucky/unlucky"
    // from "voice's max came on frame N where N depends on transient phase."
    // If firstCallbackPeak ≈ lifetimePeak, the loudness lives in the leading
    // edge (= filter-state-sensitive).  If firstCallbackPeak ≪ lifetimePeak,
    // the loud part comes later and the variation is elsewhere.
    std::atomic<float> firstCallbackPeakL{0.0f};
    std::atomic<float> firstCallbackPeakR{0.0f};
    // Pipeline stage peaks.  Each stage's max sample across the voice's
    // lifetime.  Lets us localize the variation: if stages 1 and 2 are
    // consistent but stage 3 varies, the bug is in stage 3.
    //   monoInPeak  — raw decoded WAV after ma_sound × volume (pre-DSP)
    //   monoOutPeak — after iplDirectEffect (post air-abs + dist-atten)
    //   stereoMax   — same as lifetimePeak (post HRTF binaural)
    std::atomic<float> monoInPeak{0.0f};
    std::atomic<float> monoOutPeak{0.0f};
    // Direction passed to binaural on the callback that produced the
    // lifetime peak. If this varies between voices that should have the
    // same direction, the variation is upstream (listener pose flutter).
    std::atomic<float> directionAtPeakX{0.0f};
    std::atomic<float> directionAtPeakY{0.0f};
    std::atomic<float> directionAtPeakZ{0.0f};
    // Excess-path attenuation through the portal graph.
    //   Same-room voices  → 1.0 (no extra attenuation; Steam Audio handles distance).
    //   Cross-room reachable → (realDistance/effectiveDistance)² ∈ (0,1].
    //   Cross-room unreachable → 0.0 (silenced; BFS could not connect rooms).
    // Applied unconditionally in the audio callback — same-room voices are a
    // no-op multiply by 1.0, so there is no discontinuity at the portal
    // boundary. The previous formula 1/(1+effDist²·0.001) created a hard
    // popping transition because it attenuated by absolute path length and
    // was gated by a binary `usePortalRouting` flag that flipped at the
    // portal threshold.
    float portalAttenuation = 1.0f;
    float portalBlocking = 0.0f;         // 0.0=open, 1.0=fully blocked (for LPF)
    IPLVector3 portalDirection{1.0f, 0.0f, 0.0f}; // direction toward virtual source (portal center)

    // Per-voice low-pass filter state for door blocking (audio thread only).
    // Simulates high-frequency absorption through closed doors.
    float lpfStateL = 0.0f;
    float lpfStateR = 0.0f;

    // Per-voice ramp state for portal-routing scalars (audio thread only).
    // The main thread writes `portalAttenuation` / `portalBlocking` once per
    // frame (~16 ms); the audio thread used to apply those as constants over
    // the next ~21 ms callback, producing audible steps when the target
    // jumped (room boundary crossings, BFS path flips, one-sided fallbacks).
    // We now slew toward the target at a fixed sample-rate-derived rate so
    // even abrupt frame-to-frame target changes resolve smoothly within a
    // ~10 ms window. The "current" values lag the target by at most one ramp
    // window; once they reach it, the multiply is a no-op vs. the old
    // direct-target read.
    //   currentPortalAtten ← node->portalAttenuation
    //   currentDoorAlpha   ← α(portalBlocking) where α is the 1-pole LPF coef
    // Initialized to the open / unattenuated values so a brand-new voice
    // doesn't ramp up from silence on its first callback.
    float currentPortalAtten = 1.0f;
    float currentDoorAlpha   = 1.0f;  // 1.0 = passthrough (open door)

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

    // Per-voice peak of the mono signal staged for convolution
    // (= the actual reverb-send level for this voice, post reflAtten scaling).
    // Atomic max written every audio callback; read+reset by the main thread
    // 5 s [AMB] dump so we can attribute wet-bus energy to specific voices
    // and correlate with the [WET_BUS] global IR ratio. For ambient voices
    // distanceAttenuation is forced to 1.0, so this equals the raw mono peak
    // — i.e. how loud the ambient is feeding the convolver.
    std::atomic<float> reflSendPeak{0.0f};

    // NaN/Inf guard counters (audio thread).  Each location that sanitizes
    // non-finite samples increments its own counter; we log the first few
    // occurrences per voice so the producer can be tracked down without
    // flooding the log on a sustained NaN burst.
    std::atomic<uint32_t> nanCountDirect{0};   // post iplDirectEffect
    std::atomic<uint32_t> nanCountBinaural{0}; // post iplBinauralEffect
    std::atomic<uint32_t> nanCountLpf{0};      // door-LPF state reset
    std::atomic<uint32_t> nanCountRamp{0};     // portal-atten / door-alpha ramp reset
};

/// An active voice playing through miniaudio. Owns the WAV data, decoder,
/// and sound object. Non-copyable/non-movable because ma_decoder and ma_sound
/// contain internal pointers that would dangle after a move.
///
/// Thread safety: the `finished` flag is set atomically by miniaudio's end
/// callback (audio thread) and read by VoicePool::cleanupFinished (main thread).
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

    // Per-voice maximum audible portal-graph distance. The BFS in
    // RoomService::propagateSoundPath terminates when accumulated
    // effective distance exceeds this value — anything beyond is
    // treated as unreachable. Defaults to the global mPropagationMaxDist;
    // ambient voices override this at startVoice time with a value
    // derived from their schema radius (so a wind ambient with radius=25
    // doesn't propagate 200 ft through open corridors). Matches the
    // Dark Engine `m_MaxDistance` per-source convention.
    float maxAudibleDist = 200.0f;

    // Last propagation result, retained only so external readers (debug
    // dumps, listener-room transition queries) can read the most recent
    // path without re-running BFS. The path itself is recomputed every
    // frame in loopStep; nothing here gates that.
    SoundPropInfo cachedProp{};
    // Per-portal anchor bend points along the primary path, in
    // source→listener order. Empty for clean-threaded paths (no bends
    // needed). Populated by propagateSoundPath's chainOut hook and
    // surfaced through getVoiceSpatialSnapshots() for the renderer's
    // show_vpos overlay. Diagnostic only — propagation itself does not
    // consume this.
    std::vector<Vector3> cachedChain;

    // Steam Audio simulation sources. Split across two simulators so the
    // direct path is never blocked by the reflection sim's background
    // iteration (see HANDOFF.AUDIO_VOICE_INIT.md for the underlying race).
    // Both are nullptr if scene not ready or non-spatial.
    IPLSource directSource     = nullptr;  // owned by mDirectSimulator
    IPLSource reflectionSource = nullptr;  // owned by mReflectionSimulator

    // Lazy reflection-source state machine (stage 2.2): Normal voices only
    // hold a reflectionSource while they are in (or recently in) the top-N
    // reflection-candidate pool, recomputed every frame against the live
    // listener position. This counter tracks consecutive frames the voice
    // has been out of top-N; once it reaches mReflectionDemoteHysteresisCfg
    // the voice is demoted and reflectionSource is released. Reset to 0
    // every frame the voice is in top-N. PlayerEmitted / Ambient voices
    // are excluded from the dance and this counter is unused for them.
    int framesOutOfTopN = 0;

    // World-space position for spatial audio (updated for moving objects)
    Vector3 worldPos{0.0f, 0.0f, 0.0f};

    // Steam Audio DSP node — applies direct + binaural effects in audio thread.
    // Connected between ma_sound output and engine endpoint in the node graph.
    SteamAudioDSPNode dspNode;

    ActiveVoice() {
        std::memset(&decoder, 0, sizeof(decoder));
        std::memset(&sound, 0, sizeof(sound));
    }

    // Destructor performs the audio-thread-safe teardown: marks the DSP node
    // inactive, drains the convolution worker, releases Steam Audio effects,
    // and tears down the miniaudio sound + decoder. Defined in VoicePool.cpp
    // (out-of-line) so this header doesn't pull in any further dependencies.
    ~ActiveVoice();

    // Non-copyable, non-movable (ma_decoder/ma_sound/ma_node have internal pointers)
    ActiveVoice(const ActiveVoice &) = delete;
    ActiveVoice &operator=(const ActiveVoice &) = delete;
    ActiveVoice(ActiveVoice &&) = delete;
    ActiveVoice &operator=(ActiveVoice &&) = delete;
};

/// Owns the active-voice map and handle allocator. Single-threaded (main
/// thread only) — the audio thread never touches the pool directly; it only
/// reads/writes already-captured ActiveVoice* pointers.
class VoicePool {
public:
    using Map = std::unordered_map<SoundHandle, std::unique_ptr<ActiveVoice>>;

    /// Cleanup hook signature. Invoked exactly once per voice being removed
    /// (by cleanupFinished/remove/clear) BEFORE the unique_ptr is destroyed.
    /// The hook is where AudioService runs its per-voice diagnostics and the
    /// removeVoiceSource() side of teardown (detaching Steam Audio sources
    /// from the simulators). The hook may be empty.
    using CleanupHook = std::function<void(ActiveVoice &)>;

    VoicePool() = default;
    ~VoicePool() = default;

    // Non-copyable, non-movable — the pool owns ma_decoder/ma_sound-bearing
    // ActiveVoices and exposes raw pointers to the audio thread via startVoice.
    VoicePool(const VoicePool &) = delete;
    VoicePool &operator=(const VoicePool &) = delete;
    VoicePool(VoicePool &&) = delete;
    VoicePool &operator=(VoicePool &&) = delete;

    /// Allocate the next monotonic SoundHandle. Wraps after 2^31, but in
    /// practice handles are recycled long before that.
    SoundHandle allocate() { return mNextHandle++; }

    /// Reset the handle allocator. Called from haltAll() so the post-reset
    /// session starts numbering from zero again — keeps diagnostic logs
    /// readable across mission boundaries.
    void resetAllocator() { mNextHandle = 0; }

    /// Direct access to the underlying map. Used by call sites that iterate
    /// all voices (per-frame propagation, debug dumps, halt-all). Read-only
    /// access for callers that only need to enumerate; non-const for callers
    /// that mutate per-voice state via the unique_ptr.
    Map       &voices()       { return mVoices; }
    const Map &voices() const { return mVoices; }

    /// Look up a voice by handle. Returns nullptr if absent.
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

    /// True iff `h` resolves to a live voice.
    bool exists(SoundHandle h) const {
        return h != SOUND_HANDLE_INVALID && mVoices.count(h) > 0;
    }

    /// Insert a newly-constructed voice under handle `h`. Overwrites any
    /// existing entry (callers ensure the handle came from allocate()).
    void insert(SoundHandle h, std::unique_ptr<ActiveVoice> voice) {
        mVoices[h] = std::move(voice);
    }

    /// Remove one voice. The cleanup hook (if non-empty) is invoked with a
    /// reference to the ActiveVoice BEFORE the unique_ptr is destroyed —
    /// i.e. while the voice is still discoverable by raw pointer. Returns
    /// true iff a voice was removed.
    bool remove(SoundHandle h, CleanupHook hook = {});

    /// Drop every voice. Equivalent to a remove() per entry; the hook runs
    /// for each voice in unspecified iteration order. Leaves the map empty
    /// but does NOT touch the handle allocator (callers that want a clean
    /// numbering call resetAllocator() separately).
    void clear(CleanupHook hook = {});

    std::size_t size()  const { return mVoices.size(); }
    bool        empty() const { return mVoices.empty(); }

    /// Sweep finished voices. A voice is "finished" when its `finished`
    /// atomic has been set by the tail-timer / end-callback path on the
    /// main thread. The hook runs for each removed voice, in iteration
    /// order. Returns the number of voices removed.
    std::size_t cleanupFinished(CleanupHook hook = {});

private:
    /// Map of active voices (handle → voice). Voices own their WAV data,
    /// miniaudio decoder, and sound object.
    Map mVoices;

    /// Next sound handle to assign. Reset on haltAll() via resetAllocator().
    SoundHandle mNextHandle = 0;
};

} // namespace Darkness

#endif // __VOICEPOOL_H
