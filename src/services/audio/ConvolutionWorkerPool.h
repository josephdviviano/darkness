/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2024-2026 darkness contributors
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 3 of the License, or
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

#ifndef __CONVOLUTION_WORKER_POOL_H
#define __CONVOLUTION_WORKER_POOL_H

/// @file ConvolutionWorkerPool.h
/// Off-thread reflection convolution pool — consumes per-voice mono input
/// snapshots produced by the audio thread, applies the convolution IRs
/// returned by ReflectionSimulator (ambisonics), decodes ambisonics to
/// binaural stereo and writes the result into double-buffered staging
/// outputs that the reflection mix node reads on the next callback.
///
/// Extracted from AudioService. Each sub-worker owns its own Steam Audio
/// pipeline (mixer + ambisonics decoder + scratch buffers) so processing
/// is fully independent across workers — no shared mutable state during
/// iteration. Per-voice slot assignment is round-robin and is signalled
/// from the mix node callback via acquire-release `frameSeq` increments.
///
/// THREADING — PRESERVED from the pre-extraction implementation:
///
///   • Each ConvolutionSubWorker has its OWN background thread, so the
///     pool runs N+1 threads (N sub-workers, plus the audio thread that
///     drives the staging-buffer swap inside the reflection mix node).
///   • Audio thread writes per-voice mono into `staging[writeIdx][slotIdx]`
///     while sub-workers read from `staging[readBuf][slotIdx]` for the
///     PREVIOUS frame. Staging is **triple-buffered**: at each swap the
///     mix node picks the buffer that is neither the one just filled
///     (which becomes the new read target) nor the one workers might
///     still be on. Workers are allowed to fall one frame behind without
///     forcing the audio thread to drop a swap. Only if any worker is
///     two-or-more signals behind (frameSeq − processedSeq ≥ 2) does the
///     mix node fall back to the legacy drop path, since three buffers
///     can no longer cover the three simultaneously-live read positions.
///   • Sub-worker wake-up uses an atomic `frameSeq` with release/acquire
///     ordering; the corresponding `processedSeq` is bumped at the end of
///     each iteration so wait loops (drainConvolutionWorker,
///     waitForCompletion) can converge.
///   • Per-sub-worker stereo output is double-buffered (front/back) so the
///     mix node can read the front buffer while the worker fills the back
///     buffer; the swap is a single `frontIdx.store(release)`.
///   • Each sub-worker's `hasProducedOutput` flag latches once the worker
///     has produced its first frame; the mix node skips workers that
///     haven't started yet so we don't sum from a still-zeroed buffer.

#include "AudioService.h"   // MAX_ACTIVE_VOICES

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

// Steam Audio opaque handle types — mirror the typedefs from AudioService.h
// so callers don't need to drag in phonon.h.
struct _IPLContext_t;
typedef _IPLContext_t* IPLContext;
struct _IPLHRTF_t;
typedef _IPLHRTF_t* IPLHRTF;
struct _IPLReflectionMixer_t;
typedef _IPLReflectionMixer_t* IPLReflectionMixer;
struct _IPLAmbisonicsDecodeEffect_t;
typedef _IPLAmbisonicsDecodeEffect_t* IPLAmbisonicsDecodeEffect;
struct _IPLReflectionEffect_t;
typedef _IPLReflectionEffect_t* IPLReflectionEffect;

// IPL types defined inside phonon.h. We need their full definitions for
// the structs below (ConvolutionWorker.VoiceSlot.params, listenerOrientation).
// Those structs are defined inline below; we include phonon.h in the .cpp
// translation units that touch them rather than from this header. To keep
// the header self-contained we forward-include phonon.h only when callers
// explicitly need the full types — but VoiceSlot stores IPLReflectionEffectParams
// by value, which means we DO need the full layout. The simplest path is to
// include phonon.h from this header; consumers already include
// AudioService.h which transitively includes other Steam Audio headers
// via phonon.h.
#include <phonon.h>

#include "AudioMetering.h"  // StageMeter

namespace Darkness {

// ── Per-thread convolution sub-worker ──────────────────────────────────
//
// Each sub-worker owns its own Steam Audio mixer/decoder pipeline and
// processes a subset of voice slots assigned by the mix node callback.
// Completely independent — no shared mutable state between sub-workers
// during processing.
struct ConvolutionSubWorker {
    // Own Steam Audio ambisonics decoder (not thread-safe — one per worker).
    // The reflection MIXER was dropped during PLAN.HYBRID_REVERB.md Phase 3:
    // Steam Audio's mixer only supports CONVOLUTION and TAN, not HYBRID or
    // PARAMETRIC. The sub-worker now accumulates per-voice ambisonics
    // manually into ambiCh*, which works for all three modes (convolution,
    // hybrid, parametric).
    IPLAmbisonicsDecodeEffect ambiDecodeEffect = nullptr;

    // Own scratch buffers for ambisonics processing.
    //   ambiCh{0..3}      — per-worker accumulator (sum of all per-voice
    //                       outputs assigned to this worker for this frame).
    //   voiceAmbi{0..3}   — per-voice output of one iplReflectionEffectApply
    //                       call. Cleared/overwritten by every voice.
    //   decodedL/R         — stereo binaural after ambisonics decode.
    std::vector<float> ambiCh0, ambiCh1, ambiCh2, ambiCh3;
    std::vector<float> voiceAmbi0, voiceAmbi1, voiceAmbi2, voiceAmbi3;
    std::vector<float> decodedL, decodedR;

    // Own double-buffered stereo output (deinterleaved)
    std::vector<float> stereoL[2], stereoR[2];
    std::atomic<int> frontIdx{0};
    std::atomic<bool> hasProducedOutput{false};

    // Last decoded sample of the previous reflection frame, kept as state
    // for the rateDivisor>1 upsampler so the bridge between consecutive
    // frames is a continuous piecewise-linear curve instead of a held-flat
    // sample + step discontinuity at every reflection-frame boundary
    // (audible as a low buzz at reflRate / reflFrameSize Hz).
    float prevDecodedTailL = 0.0f;
    float prevDecodedTailR = 0.0f;

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

    // Gain-staging meters (per sub-worker, written only by this worker
    // thread, read by the master mix node at [GAIN] dump time with
    // accept-tearing semantics — diagnostic, no correctness impact).
    // The mix node folds all sub-worker meters into a combined view
    // before logging via StageMeter::mergeIn.
    StageMeter saMeterReflIn;     // mono input to iplReflectionEffectApply
    StageMeter saMeterReflW;      // W (omni) channel of convolution output
    StageMeter saMeterDecodeOut;  // stereo output of iplAmbisonicsDecodeEffect
};

// ── Off-thread convolution worker with K parallel sub-workers ──────────
//
// The audio callback writes voice mono snapshots to shared staging buffers.
// The mix node callback distributes voice slots across sub-workers
// (round-robin) and signals them. Each sub-worker processes its subset
// independently and writes to its own double-buffered stereo output. The
// mix node reads and sums all sub-worker outputs on the next callback.
// No barrier sync needed.
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

    // Triple-buffered voice slots. Rotation pattern at each swap:
    //   • writeIdx (just filled by audio thread) → becomes new currentReadBuf
    //   • currentReadBuf (workers' previous target, may still be in flight)
    //     → becomes the next writeIdx
    //   • the remaining buffer is the one workers were NOT on
    // This way the audio thread never has to drop a swap just because a
    // worker is finishing the previous frame. Drops only happen if any
    // worker is ≥ 2 frames behind (catastrophic backlog), since then all
    // three buffers are simultaneously live.
    static constexpr int kStagingBuffers = 3;
    VoiceSlot staging[kStagingBuffers][kMaxSlots];
    int stagingCount[kStagingBuffers] = {0, 0, 0};  // count per buffer
    std::atomic<int> writeIdx{0};        // audio thread writes to this index
    std::atomic<int> readyCount{0};      // count of the buffer just completed

    // Number of sub-workers that have woken on the current signal but
    // have not yet finished their iteration. Decremented to 0 by workers
    // after they release their staging slot references. Vestigial for
    // safety (drop logic) since the two-frames-behind check on
    // frameSeq/processedSeq is now the primary back-pressure signal;
    // kept for shutdown drain diagnostics.
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

/// Drain all sub-workers' pending frames. Called from ~ActiveVoice (defined
/// in VoicePool.cpp) before releasing the IPLReflectionEffect — the worker
/// holds a pointer to that effect and must not be mid-iteration when it's
/// released.
///
/// Returns true on clean drain, false if the deadline expired (caller logs
/// and continues — the validity-token mechanism still keeps the worker safe
/// from reading freed memory, this just bounds the post-cleanup tail).
bool drainConvolutionWorker(ConvolutionWorker *cw, int deadlineMs);

// ── Pool front-end ─────────────────────────────────────────────────────
//
// AudioService constructs one of these inside initReflectionPipeline().
// It owns the ConvolutionWorker (and its sub-worker threads), exposes the
// worker pointer for the reflection mix node + per-voice DSP node staging,
// and provides shutdown/drain helpers.
class ConvolutionWorkerPool {
public:
    /// Configuration captured from RenderConfig / AudioService at init.
    struct Config {
        IPLContext context = nullptr;
        IPLHRTF    hrtf = nullptr;
        int        frameSize = 1024;
        int        reflectionFrameSize = 1024;
        int        rateDivisor = 2;
        int        ambiChannels = 1;
        int        ambiOrder = 0;
        /// Number of sub-workers (>0). Auto-derivation happens in the
        /// caller before construction.
        int        numWorkers = 1;
        /// IPL audio settings (sampling rate + frame size) used to create
        /// each sub-worker's IPLReflectionMixer + IPLAmbisonicsDecodeEffect.
        IPLAudioSettings audioSettings{};
        /// Reflection-effect settings — irSize + numChannels.
        IPLReflectionEffectSettings reflSettings{};
    };

    ConvolutionWorkerPool();
    ~ConvolutionWorkerPool();

    ConvolutionWorkerPool(const ConvolutionWorkerPool&) = delete;
    ConvolutionWorkerPool& operator=(const ConvolutionWorkerPool&) = delete;

    /// Construct the underlying ConvolutionWorker, allocate per-sub-worker
    /// Steam Audio pipelines, spawn the threads. Returns false on any
    /// IPL allocation failure (caller logs + falls back to on-thread).
    bool init(const Config &cfg);

    /// Block until every sub-worker has finished all frames signalled so far.
    /// Spin-wait with a 500ms deadline; logs a warning on timeout.
    void waitForCompletion();

    /// Shut down all sub-workers and release per-worker Steam Audio
    /// resources. Idempotent.
    void shutdown();

    /// Pointer to the underlying worker struct (used by the reflection
    /// mix node + per-voice DSP nodes for staging-buffer hand-off). May
    /// return nullptr if init() has not been called or failed.
    ConvolutionWorker* worker() { return mWorker.get(); }
    const ConvolutionWorker* worker() const { return mWorker.get(); }

    /// True if init() succeeded and the worker pool is operational.
    bool isActive() const { return mWorker != nullptr; }

    /// Convenience: number of sub-workers actually spawned.
    int numWorkers() const { return mWorker ? mWorker->numWorkers : 0; }

private:
    /// Per-thread main — wakes on frameSeq increment, processes assigned
    /// slots, runs ambisonics decode, writes deinterleaved stereo to the
    /// back buffer, flips frontIdx, bumps processedSeq.
    void subWorkerMain(int workerIdx);

    std::unique_ptr<ConvolutionWorker> mWorker;
};

} // namespace Darkness

#endif // __CONVOLUTION_WORKER_POOL_H
