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
/// Off-thread reflection convolution pool. The audio thread snapshots per-
/// voice mono into shared staging buffers; the mix node distributes voice
/// slots round-robin across N sub-workers and signals them by bumping
/// `frameSeq` (release). Each sub-worker owns its own Steam Audio pipeline
/// (no shared mutable state during iteration), applies the convolution IRs,
/// decodes ambisonics to binaural stereo, and writes a double-buffered
/// output that the mix node reads on the next callback.
///
/// THREADING:
///   • N sub-worker threads + audio thread.
///   • Staging is **triple-buffered** so workers may lag one frame; only
///     when any worker is ≥2 signals behind does the mix node fall back to
///     the drop path.
///   • Wake uses acquire-release atomics + condvar; processedSeq is bumped
///     at iteration end so drain/waitForCompletion can converge.
///   • Per-worker stereo output is double-buffered; swap is a single
///     frontIdx.store(release).

#include "AudioService.h"   // MAX_ACTIVE_VOICES
#include "AudioUnits.h"     // kDefaultDeviceFrameSize

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

// Steam Audio opaque types — mirror typedefs from AudioService.h so callers
// don't need to drag in phonon.h.
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

// VoiceSlot stores IPLReflectionEffectParams by value → need the full layout.
#include <phonon.h>

#include "AudioMetering.h"  // StageMeter
#include "LatencyHistogram.h"  // per-stage CPU profiling

namespace Darkness {

/// Per-thread sub-worker. Owns its own Steam Audio ambisonics decoder +
/// scratch buffers — fully independent across workers. The reflection mixer
/// was dropped (it only supports CONVOLUTION/TAN); workers accumulate
/// ambisonics manually into ambiCh* so HYBRID and PARAMETRIC also work.
struct ConvolutionSubWorker {
    IPLAmbisonicsDecodeEffect ambiDecodeEffect = nullptr;

    // Per-worker accumulator (sum of this worker's voice outputs this frame).
    std::vector<float> ambiCh0, ambiCh1, ambiCh2, ambiCh3;
    // Per-voice output of one iplReflectionEffectApply (overwritten per voice).
    std::vector<float> voiceAmbi0, voiceAmbi1, voiceAmbi2, voiceAmbi3;
    // Stereo binaural after ambisonics decode.
    std::vector<float> decodedL, decodedR;

    // Double-buffered stereo output (deinterleaved).
    std::vector<float> stereoL[2], stereoR[2];
    std::atomic<int> frontIdx{0};
    std::atomic<bool> hasProducedOutput{false};

    // Delay-line history for the polyphase upsampler — (subLen-1) samples
    // of the previous reflection frame per channel. Empty when rateDivisor=1.
    std::vector<float> prevDecodedL;
    std::vector<float> prevDecodedR;

    // Slot assignment: indices into shared staging[readBuf]. Written by mix
    // node BEFORE signal, read by worker AFTER signal — atomic frameSeq
    // provides the acquire-release barrier.
    int assignedSlots[MAX_ACTIVE_VOICES];
    int assignedCount = 0;

    std::thread thread;
    std::atomic<uint64_t> frameSeq{0};      // mix node bumps to signal new data
    std::atomic<uint64_t> processedSeq{0};  // last frameSeq this worker finished
    std::atomic<bool> shutdown{false};
    std::atomic<float> peakMs{0.0f};

    // Worker sleeps on wakeCv when frameSeq==lastSeq && !shutdown. cv.wait
    // re-checks the predicate so a missed notify is harmless. Replaces the
    // previous yield-poll which missed signals during macOS scheduler stalls.
    std::mutex wakeMtx;
    std::condition_variable wakeCv;

    // Gain-staging meters (written only by this worker thread; read by the
    // mix node at [GAIN] dump time with accept-tearing semantics).
    StageMeter saMeterReflIn;     // mono input to iplReflectionEffectApply
    StageMeter saMeterReflW;      // W (omni) channel of convolution output
    StageMeter saMeterDecodeOut;  // stereo output of iplAmbisonicsDecodeEffect

    // Per-stage latency histograms (one writer per histogram). AudioService
    // periodic dump merges across sub-workers before computing percentiles.
    LatencyHistogram perfApplyMs;     // iplReflectionEffectApply per voice
    LatencyHistogram perfSumMs;       // manual ambisonics sum per voice
    LatencyHistogram perfDecodeMs;    // iplAmbisonicsDecodeEffectApply per frame
    LatencyHistogram perfUpsampleMs;  // rate-divisor upsample + bridge per frame
    LatencyHistogram perfIterMs;      // total per-iteration time

    // Per-iter apply distribution:
    //   perfSumApplyMs  — sum of apply() calls in one iter (worker's actual
    //                     convolution work, comparable to iter time)
    //   perfMaxApplyMs  — max single apply() call (single dominant voice
    //                     vs balanced load)
    //   perfResidualMs  — iter − DSP. Non-DSP time: memset, atomic store,
    //                     buffer flip, scheduler preemption. Large p99 →
    //                     bottleneck is not DSP, yaml knobs won't help.
    LatencyHistogram perfMaxApplyMs;
    LatencyHistogram perfSumApplyMs;
    LatencyHistogram perfResidualMs;

    // Signal→pickup latency. Mix node writes at frameSeq bump; worker reads
    // when it observes the new seq. Captures wait-loop + scheduler delay.
    // ns since steady_clock epoch.
    std::atomic<long long> signalTimeNs{0};
    LatencyHistogram perfSignalPickupMs;

    // Per-stage NaN-guard counters. ambiCh* sanitised BEFORE decode (a NaN
    // in W becomes NaN in both stereo channels post-decode); decodedL/R
    // sanitised AFTER decode (catches decoder-introduced NaNs). Per-sample
    // replacement with 0 produces audible clicks rather than full silence.
    std::atomic<uint32_t> nanCountAmbi{0};
    std::atomic<uint32_t> nanCountDecode{0};

    // ── Voice-eviction schema buffer length ──
    //
    // Per-sub-worker prevActive* arrays were promoted to the pool level
    // (ConvolutionWorker::prevPoolActive*) in 2026-05-24 so worker-boundary
    // migrations stop firing false-positive [REFL_EVICT] events. Only the
    // schema-name buffer length constant remains here because both the
    // pool-level arrays AND the per-voice DSP staging slot (for forwarding
    // the schema string to the audio thread) reference it.
    static constexpr int kSchemaNameBufLen = 24;

    // ── First-apply tracking (T0.2) ──
    //
    // First time we see a voice handle in any slot across this sub-worker
    // we emit `[REFL_FIRST_APPLY]` so the orchestrator can distinguish
    // "IR unpopulated for short-lived voices" from "convolution state
    // isn't initialised on the first apply for transients". The entry
    // is removed from the set on slot eviction so each voice incarnation
    // gets one log line.
    //
    // Tracked by voice handle so a voice that bounces across sub-workers
    // doesn't double-log within the same sub-worker — but it can log
    // once per sub-worker (acceptable; distinct sub-workers see distinct
    // first-apply states).
    static constexpr int kFirstApplyTrackCap = MAX_ACTIVE_VOICES * 2;
    int  firstAppliedHandles[kFirstApplyTrackCap];
    int  firstAppliedCount = 0;

    // ── Queue-depth for T2.3 [PERF refl_worker] ──
    //
    // perfQueueDepth — assignCount sampled at iter entry, one observation
    //                  per worker iter (so percentiles count iters, not
    //                  voices). Surfaces the "8/8 voices fed to two
    //                  workers" vs "4/4 each" imbalance. The existing
    //                  perfApplyMs histogram covers the apply wall-clock
    //                  half of T2.3.
    LatencyHistogram perfQueueDepth;
};

/// Off-thread convolution worker with K parallel sub-workers. Audio
/// callback writes voice mono to shared staging; mix node distributes
/// round-robin and signals workers; each sub-worker writes its own
/// double-buffered stereo; mix node sums all sub-worker outputs on the
/// next callback. No barrier sync needed.
struct ConvolutionWorker {
    /// Per-voice mono input snapshot (audio thread writes, sub-workers read).
    struct VoiceSlot {
        std::vector<float> mono;          // post-distance-atten, post-decimate
        IPLReflectionEffect effect = nullptr;
        IPLReflectionEffectParams params{};
        // Reference-counted so workers can safely dereference even after
        // the owning ActiveVoice has been destroyed.
        std::shared_ptr<std::atomic<bool>> validityToken;
        int reflFrameSize = 0;
        bool active = false;
        // Forwarded from the voice's DSP node — gates per-voice wet-peak log
        // to footstep voices.
        bool isFootstepDiag = false;
        // Forwarded by the audio thread for the [REFLECTION_VOICE]
        // diagnostic. The schemaCStr pointer references the owning
        // ActiveVoice's std::string; safe within the worker's iteration
        // because removeVoiceSource waits for the worker to drain before
        // destroying the voice (validityToken pattern).
        int voiceHandle = -1;
        const char *schemaCStr = nullptr;
        // IR-energy elision groundwork (measurement only): points at the
        // owning node's reflectionOutEnergyW atomic. After applying the
        // reflection effect, the sub-worker writes the mean-square energy of
        // the wet W (channel-0) output here so the main thread can observe how
        // much reverb each voice actually produces. Lifetime is guarded by the
        // same validityToken drain as schemaCStr — removeVoiceSource drains
        // the worker before destroying the voice/node. Null = don't write.
        std::atomic<float> *outEnergyW = nullptr;
    };
    static constexpr int kMaxSlots = MAX_ACTIVE_VOICES;

    // Triple-buffered staging. At each swap: writeIdx (just filled) becomes
    // new currentReadBuf; the previous read buffer becomes writeIdx; the
    // third buffer is the one workers were NOT on. Audio thread never has
    // to drop a swap unless any worker is ≥2 frames behind (catastrophic
    // backlog — all three buffers simultaneously live).
    static constexpr int kStagingBuffers = 3;
    VoiceSlot staging[kStagingBuffers][kMaxSlots];
    int stagingCount[kStagingBuffers] = {0, 0, 0};
    std::atomic<int> writeIdx{0};
    std::atomic<int> readyCount{0};

    // Sub-workers active on the current signal (workers decrement to 0 after
    // releasing staging slot references). Vestigial for safety since the
    // ≥2-frames-behind check on frameSeq/processedSeq is the primary
    // back-pressure signal; retained for shutdown drain diagnostics.
    std::atomic<int> workersReading{0};

    // Which staging buffer sub-workers should read. Plain int — visibility
    // guaranteed by acquire-release on sub-worker frameSeq signals.
    int currentReadBuf = -1;

    // Sub-workers (K parallel threads). unique_ptr because
    // ConvolutionSubWorker contains std::atomic (non-movable).
    std::vector<std::unique_ptr<ConvolutionSubWorker>> workers;
    int numWorkers = 1;

    // ── Per-worker slot cap (Fix B, 2026-05-24) ──
    //
    // Hard ceiling on assignedCount for any single sub-worker per audio
    // callback. Derived from `mReverbVoices` (the runtime convolution
    // budget) divided by `numWorkers` with ceiling rounding, so the union
    // of per-worker caps still covers the full budget. Enforced in the
    // round-robin distribution loop in
    // AudioService::reflectionMixNodeProcess.
    //
    // Default kMaxSlots (= MAX_ACTIVE_VOICES) effectively disables the
    // cap until AudioService::setPerWorkerSlotCap is called from
    // initReflectionPipeline / setReverbVoices.
    //
    // Per the memory rule (no silent fallbacks): if the round-robin loop
    // would assign a slot to a worker that has hit this cap, the slot is
    // dropped and a [FALLBACK] line is emitted identifying the full
    // worker. The dropped slot is effectively a [REFL_DROP] of one voice
    // for one callback — same outcome as the existing catastrophic-
    // backlog branch, but caused by per-worker rather than total
    // overflow.
    int perWorkerSlotCap = kMaxSlots;

    // Shared read-only state (set at init, immutable during processing).
    IPLHRTF hrtf = nullptr;
    IPLContext context = nullptr;
    int frameSize = static_cast<int>(kDefaultDeviceFrameSize);
    int reflectionFrameSize = static_cast<int>(kDefaultDeviceFrameSize);
    int ambiChannels = 1;
    int ambiOrder = 0;
    int rateDivisor = 2;  // 1=full, 2=half, 4=quarter

    // Polyphase upsample FIR (designed once at init from rateDivisor).
    // Layout: [phase * subLen + tap]. Empty + subLen==0 when rateDivisor=1
    // (passthrough memcpy). Replaces per-sample linear interpolation
    // which had ~13 dB image rejection at source-Nyquist → audible grain
    // in the upper output band.
    std::vector<float> upsampleCoeffs;
    int upsampleSubLen = 0;
    // Audio callback period (ms) = frameSize/sampleRate * 1000. Used to
    // size the [REFL_LAG] threshold + report in the log.
    float callbackPeriodMs = 21.333f;

    // Listener orientation snapshot (mix node writes, all workers read).
    IPLCoordinateSpace3 listenerOrientation = {
        {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f}
    };

    // Worker→audio-thread completion signal. Replaces the previous yield-
    // poll which burned the full sync deadline whenever any worker stalled.
    // Single shared CV — audio thread is the only waiter, so notify_one
    // from any worker wakes the predicate re-check which loads every
    // worker's processedSeq atomically.
    std::mutex             doneMtx;
    std::condition_variable doneCv;

    // Counter for catastrophic-backlog frame drops + slot evictions. Both
    // are counters, not histograms: a [PERF refl_worker] cadence of 1 Hz
    // can read-and-reset them. Updated:
    //   • droppedFramesTotal — by the mix node when workersTwoBehind=true
    //                          (AudioService.cpp catastrophic-backlog
    //                          path). The pool exposes a setter so
    //                          AudioService can bump without taking a
    //                          dependency on the pool's internal struct.
    //   • slotEvictionsTotal — by the worker when a slot's voiceHandle
    //                          changes across iters (see T1.4).
    std::atomic<uint64_t> droppedFramesTotal{0};
    std::atomic<uint64_t> slotEvictionsTotal{0};

    // ── Pool-level voice-eviction tracking (Fix A, 2026-05-24) ──
    //
    // Previously each ConvolutionSubWorker maintained its own prevActive*
    // set and diff. That fired false [REFL_EVICT] events whenever a voice
    // legitimately migrated across worker stripes (load-balancing /
    // priority reorder): the voice left worker A's stripe (counted as an
    // eviction by A) and joined worker B's (silent there but the A→B
    // crossing produced a log line). A 15k-line MISS06 run showed 883
    // events with only ~57 actual pinned voices — same handles citing
    // 50+ "evictions" each.
    //
    // The correct definition: a voice is evicted only when its handle
    // disappears from the UNION of all sub-workers' assigned-slot sets.
    // That union equals the current iter's stagingCount[currentReadBuf]
    // slots (since the round-robin distribution covers every staging
    // slot). The sweep is run ONCE per audio-callback iter on the audio
    // thread, AFTER round-robin slot assignment and BEFORE the frameSeq
    // bump that signals sub-workers (so any cross-thread state updates
    // we make here — e.g. clearing firstAppliedHandles on sub-workers —
    // are visible to the workers through the frameSeq acquire barrier).
    //
    // Storage requirements (audio thread is the single writer; sub-workers
    // never touch these arrays):
    //   prevPoolActiveCount    — valid entries in the arrays below.
    //   prevPoolActiveHandles  — voice handles present in the union last
    //                            iter (unordered).
    //   prevPoolActiveSchemas  — parallel array of schema-name copies for
    //                            the eviction log line; the owning
    //                            ActiveVoice may be destroyed between
    //                            iters so we cannot keep raw const-char*.
    int  prevPoolActiveCount = 0;
    int  prevPoolActiveHandles[MAX_ACTIVE_VOICES];
    char prevPoolActiveSchemas[MAX_ACTIVE_VOICES]
                              [ConvolutionSubWorker::kSchemaNameBufLen];

    /// Pool-wide voice-eviction sweep (Fix A, 2026-05-24). Called by the
    /// audio thread once per callback AFTER round-robin slot assignment
    /// and BEFORE bumping per-sub-worker frameSeq. Compares the union of
    /// active voice handles across `staging[writeBuf][0..count-1]` to the
    /// previous iter's pool-wide set and emits one `[REFL_EVICT]` line
    /// per genuinely-departed voice. Worker-boundary moves are silent
    /// because the union view ignores which worker holds which slot.
    ///
    /// RT-safety: no malloc, stack-only scratch (bounded by kMaxSlots),
    /// O(N²) diff over N≤64. Updates this->prevPoolActive* + bumps
    /// slotEvictionsTotal + clears firstAppliedHandles on all sub-workers
    /// for evicted handles (safe because workers are sleeping on wakeCv
    /// at this point in the callback).
    void sweepEvictionsRT(int writeBuf, int count);
};

/// Drain all sub-workers' pending frames. Called from ~ActiveVoice before
/// releasing the IPLReflectionEffect — the worker holds a pointer to that
/// effect and must not be mid-iteration when it's released. Returns true
/// on clean drain, false on deadline expiry (the validity token still
/// keeps the worker safe; this just bounds the post-cleanup tail).
bool drainConvolutionWorker(ConvolutionWorker *cw, int deadlineMs);

/// Pool front-end. AudioService constructs one in initReflectionPipeline().
class ConvolutionWorkerPool {
public:
    struct Config {
        IPLContext context = nullptr;
        IPLHRTF    hrtf = nullptr;
        int        frameSize = static_cast<int>(kDefaultDeviceFrameSize);
        int        reflectionFrameSize = static_cast<int>(kDefaultDeviceFrameSize);
        int        rateDivisor = 2;
        int        ambiChannels = 1;
        int        ambiOrder = 0;
        int        numWorkers = 1;
        IPLAudioSettings audioSettings{};
        IPLReflectionEffectSettings reflSettings{};
    };

    ConvolutionWorkerPool();
    ~ConvolutionWorkerPool();

    ConvolutionWorkerPool(const ConvolutionWorkerPool&) = delete;
    ConvolutionWorkerPool& operator=(const ConvolutionWorkerPool&) = delete;

    /// Construct the worker, allocate per-sub-worker IPL pipelines, spawn
    /// threads. False on any IPL alloc failure (caller falls back to
    /// on-thread).
    bool init(const Config &cfg);

    /// Block until every sub-worker has finished all frames signalled so
    /// far. Spin-wait with a 500ms deadline; warns on timeout.
    void waitForCompletion();

    /// Idempotent shutdown.
    void shutdown();

    ConvolutionWorker* worker() { return mWorker.get(); }
    const ConvolutionWorker* worker() const { return mWorker.get(); }

    /// Main-thread-paced perf log emitter for T2.3 [PERF refl_worker].
    /// Caller invokes once per main-loop tick; we self-throttle to ~1 Hz
    /// using a monotonic timestamp so the log cadence is stable even when
    /// the caller's tick rate fluctuates. Reads percentiles from every
    /// sub-worker's perfQueueDepth and emits one log line.
    /// Idempotent if called before mWorker is constructed (early-return).
    ///
    /// NOTE: applyMs is owned by [PERF worker] in
    /// AudioService::dumpAudioStatusPeriodic (which drains perfApplyMs via
    /// snapshotAndReset earlier in the same dump cycle). This method must
    /// NOT touch perfApplyMs — doing so would double-empty the histogram
    /// and produce 0.000-ms percentiles in whichever line runs second.
    void pollPerfPeriodic();

    /// Bumped by the mix node (AudioService.cpp catastrophic-backlog
    /// branch) when N voice slots were thrown away because workers were
    /// >=2 frames behind. Plain counter; pollPerfPeriodic exchanges to 0
    /// each emission window so the next window is independent.
    void recordDroppedVoices(int n) {
        if (n <= 0 || !mWorker) return;
        mWorker->droppedFramesTotal.fetch_add(
            static_cast<uint64_t>(n), std::memory_order_relaxed);
    }

    bool isActive() const { return mWorker != nullptr; }
    int numWorkers() const { return mWorker ? mWorker->numWorkers : 0; }

    /// Set the per-worker slot cap (Fix B, 2026-05-24). `reverbVoices` is
    /// the global runtime convolution budget; the per-worker ceiling is
    /// ceil(reverbVoices / numWorkers). Safe to call from any thread; the
    /// audio callback reads the value with relaxed ordering each frame.
    /// Idempotent if called before `init()`.
    void setPerWorkerSlotCap(int reverbVoices) {
        if (!mWorker) return;
        const int n = mWorker->numWorkers > 0 ? mWorker->numWorkers : 1;
        int cap = (reverbVoices + n - 1) / n;
        if (cap < 1) cap = 1;
        if (cap > ConvolutionWorker::kMaxSlots) cap = ConvolutionWorker::kMaxSlots;
        mWorker->perWorkerSlotCap = cap;
    }

private:
    /// Per-thread main — wakes on frameSeq, processes assigned slots, runs
    /// ambisonics decode, writes back stereo, flips frontIdx, bumps
    /// processedSeq.
    void subWorkerMain(int workerIdx);

    std::unique_ptr<ConvolutionWorker> mWorker;
};

} // namespace Darkness

#endif
