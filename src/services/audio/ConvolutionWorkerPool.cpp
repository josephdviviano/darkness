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

#include "ConvolutionWorkerPool.h"

#include "AudioLog.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>

#include <phonon.h>

#include "logger.h"

namespace {

// I0 series — converges fast for β ≤ 10; 20 terms gives <1e-9 relative
// error. Used only at pool init.
double besselI0(double x)
{
    const double y = 0.5 * x;
    double term = 1.0;
    double sum = 1.0;
    for (int k = 1; k <= 20; ++k) {
        term *= y / static_cast<double>(k);
        const double inc = term * term;
        sum += inc;
        if (inc < 1.0e-12 * sum) break;
    }
    return sum;
}

// Design a Kaiser-windowed-sinc low-pass and lay it out in polyphase order.
//   subLen      — taps per sub-filter
//   div         — upsample ratio; total = subLen * div
//   cutoffNorm  — cutoff normalised to fs_out (must be < 0.5/div)
//   beta        — Kaiser β (≈8 → ~75 dB stopband)
//   out         — out[phase * subLen + tap]; tap=0 is x[k], tap=subLen-1 is
//                 x[k-(subLen-1)]
// Each sub-filter is scaled to sum ≈1 → unity DC gain after interpolation.
void designUpsampleFIR(int subLen, int div, double cutoffNorm, double beta,
                       std::vector<float>& out)
{
    const int totalTaps = subLen * div;
    out.assign(static_cast<size_t>(totalTaps), 0.0f);
    if (totalTaps < 2 || div < 2 || subLen < 2) return;

    // Build the flat linear-phase prototype.
    std::vector<double> h(static_cast<size_t>(totalTaps), 0.0);
    const double M    = static_cast<double>(totalTaps - 1);
    const double i0B  = besselI0(beta);
    double sum        = 0.0;
    for (int n = 0; n < totalTaps; ++n) {
        const double x = static_cast<double>(n) - 0.5 * M;
        double sinc;
        if (std::abs(x) < 1.0e-12) {
            sinc = 2.0 * cutoffNorm;
        } else {
            sinc = std::sin(2.0 * M_PI * cutoffNorm * x) / (M_PI * x);
        }
        const double r   = 2.0 * static_cast<double>(n) / M - 1.0;
        const double arg = beta * std::sqrt(std::max(0.0, 1.0 - r * r));
        const double w   = besselI0(arg) / i0B;
        h[n] = sinc * w;
        sum += h[n];
    }

    const double scale = (sum > 1.0e-12) ? (static_cast<double>(div) / sum) : 1.0;

    // Polyphase rearrange: out[p*subLen + t] = h[t*div + p]. Tap-major
    // inside each phase walks input history sequentially (cache-friendly).
    for (int p = 0; p < div; ++p) {
        for (int t = 0; t < subLen; ++t) {
            const double v = h[t * div + p] * scale;
            out[static_cast<size_t>(p) * subLen + t] = static_cast<float>(v);
        }
    }
}

} // namespace

// Thread naming + QoS introspection — macOS can place threads on E-cores
// (~2-3× slower for FFT work), so [WORKER_QOS] confirms each worker's class.
#if defined(__APPLE__)
#  include <pthread.h>
#  include <pthread/qos.h>
#elif defined(__linux__)
#  include <pthread.h>
#  include <sched.h>
#endif

namespace Darkness {

// Replace non-finite samples with 0; returns true if any were replaced.
// Mirrors AudioService.cpp's master-DSP helper so workers stop bad samples
// at source instead of leaving them for the downstream guard.
static inline bool audioSanitizeBuffer(float* buf, std::size_t n)
{
    if (!buf) return false;
    bool sawBad = false;
    for (std::size_t i = 0; i < n; ++i) {
        if (!std::isfinite(buf[i])) {
            buf[i] = 0.0f;
            sawBad = true;
        }
    }
    return sawBad;
}

// Enable FTZ — long-tail IRs decaying to denormals can stall the worker
// 50×. Mirrors AudioService.cpp's audio-thread helper (private to that TU).
static inline void audioEnableDenormalFlush()
{
#if defined(__aarch64__) || defined(_M_ARM64)
    uint64_t fpcr;
    __asm__ __volatile__("mrs %0, fpcr" : "=r"(fpcr));
    fpcr |= (uint64_t(1) << 24);  // FZ
    __asm__ __volatile__("msr fpcr, %0" : : "r"(fpcr));
#elif defined(__SSE__) || defined(_M_IX86) || defined(_M_X64)
    unsigned int mxcsr;
    __asm__ __volatile__("stmxcsr %0" : "=m"(mxcsr));
    mxcsr |= (1u << 15) | (1u << 6);  // FTZ | DAZ
    __asm__ __volatile__("ldmxcsr %0" : : "m"(mxcsr));
#endif
}

//------------------------------------------------------
// ConvolutionWorker::sweepEvictionsRT (Fix A, 2026-05-24)
//
// Pool-wide voice-eviction diff, called once per audio-callback iter on
// the audio thread. Promoted from the per-sub-worker sweep that fired
// false [REFL_EVICT] events on worker-boundary moves (the 759ed eviction
// fix's blind spot).
//
// RT-safety contract:
//   • No malloc, no mutex, no condvar.
//   • Stack scratch bounded by kMaxSlots (= MAX_ACTIVE_VOICES = 64).
//   • snprintf for schema-name copy — same cost as [FOOT_REFL_IN] etc.
//     that already run on this thread.
//   • firstAppliedHandles[] on each sub-worker is normally read/written
//     by the worker thread only. We write it here BEFORE the audio
//     thread bumps frameSeq (the wake signal); the worker's acquire on
//     frameSeq is the release-paired barrier, so our store is visible
//     by the time the worker next inspects firstAppliedHandles.
void ConvolutionWorker::sweepEvictionsRT(int writeBuf, int count)
{
    if (writeBuf < 0 || writeBuf >= kStagingBuffers) return;
    if (count < 0) count = 0;
    if (count > kMaxSlots) count = kMaxSlots;

    int  curHandles[kMaxSlots];
    char curSchemas[kMaxSlots]
                   [ConvolutionSubWorker::kSchemaNameBufLen];
    int  curCount = 0;
    for (int i = 0; i < count; ++i) {
        auto &slot = staging[writeBuf][i];
        if (!slot.active) continue;
        int h = slot.voiceHandle;
        if (h < 0) continue;
        // De-duplicate: under the current per-voice DSP design a handle
        // can appear in at most one staging slot per iter, but we guard
        // against duplicates so a future packer change can't silently
        // skew the eviction count.
        bool seen = false;
        for (int c = 0; c < curCount; ++c) {
            if (curHandles[c] == h) { seen = true; break; }
        }
        if (seen) continue;
        if (curCount >= kMaxSlots) {
            // Defensive: unreachable under count<=kMaxSlots.
            AUDIO_LOG("[FALLBACK] sweepEvictionsRT curHandles overflow "
                      "curCount=%d kMaxSlots=%d\n",
                      curCount, kMaxSlots);
            break;
        }
        curHandles[curCount] = h;
        const char *schema = slot.schemaCStr ? slot.schemaCStr : "";
        std::snprintf(curSchemas[curCount],
                      ConvolutionSubWorker::kSchemaNameBufLen,
                      "%s", schema);
        ++curCount;
    }

    // For each handle in last iter's pool-wide set, is it still present
    // in this iter's set? Absent = genuine eviction (voice ended, fell
    // below the global active cap, etc.). Worker-boundary moves are
    // silent because the union view ignores which worker holds which
    // slot — this is the whole point of the Fix A refactor.
    for (int p = 0; p < prevPoolActiveCount; ++p) {
        int prevH = prevPoolActiveHandles[p];
        if (prevH < 0) continue;
        bool stillPresent = false;
        for (int c = 0; c < curCount; ++c) {
            if (curHandles[c] == prevH) { stillPresent = true; break; }
        }
        if (!stillPresent) {
            slotEvictionsTotal.fetch_add(1, std::memory_order_relaxed);
            AUDIO_LOG("[REFL_EVICT] voice h=%d schema='%s'\n",
                      prevH, prevPoolActiveSchemas[p]);
            // Clear the evicted handle from every sub-worker's
            // firstAppliedHandles so a future re-appearance re-emits
            // [REFL_FIRST_APPLY]. Safe per the RT contract above.
            for (auto &subPtr : workers) {
                ConvolutionSubWorker &sub = *subPtr;
                for (int k = 0; k < sub.firstAppliedCount; ++k) {
                    if (sub.firstAppliedHandles[k] == prevH) {
                        sub.firstAppliedHandles[k] =
                            sub.firstAppliedHandles[--sub.firstAppliedCount];
                        break;
                    }
                }
            }
        }
    }

    // Save current set as the new prev set. Copy schemas now (before the
    // owning ActiveVoice can be destroyed between iters) so the next
    // iter's eviction log line still has the schema name available.
    prevPoolActiveCount = curCount;
    for (int c = 0; c < curCount; ++c) {
        prevPoolActiveHandles[c] = curHandles[c];
        std::memcpy(prevPoolActiveSchemas[c], curSchemas[c],
                    ConvolutionSubWorker::kSchemaNameBufLen);
    }
}

//------------------------------------------------------
bool drainConvolutionWorker(ConvolutionWorker *cw, int deadlineMs)
{
    if (!cw) return true;
    auto deadline = std::chrono::steady_clock::now()
                  + std::chrono::milliseconds(deadlineMs);
    for (auto &subPtr : cw->workers) {
        auto &sub = *subPtr;
        uint64_t target = sub.frameSeq.load(std::memory_order_acquire);
        while (sub.processedSeq.load(std::memory_order_acquire) < target) {
            if (sub.shutdown.load(std::memory_order_relaxed)) break;
            if (std::chrono::steady_clock::now() >= deadline)
                return false;
            std::this_thread::yield();
        }
    }
    return true;
}

//------------------------------------------------------
ConvolutionWorkerPool::ConvolutionWorkerPool() = default;

//------------------------------------------------------
ConvolutionWorkerPool::~ConvolutionWorkerPool()
{
    shutdown();
}

//------------------------------------------------------
bool ConvolutionWorkerPool::init(const Config &cfg)
{
    if (mWorker) {
        LOG_ERROR("ConvolutionWorkerPool: init called when already initialized");
        return false;
    }
    if (cfg.numWorkers <= 0) {
        LOG_ERROR("ConvolutionWorkerPool: numWorkers must be > 0 (got %d)", cfg.numWorkers);
        return false;
    }

    mWorker = std::make_unique<ConvolutionWorker>();
    auto &cw = *mWorker;
    cw.context = cfg.context;
    cw.hrtf = cfg.hrtf;
    cw.frameSize = cfg.frameSize;
    cw.reflectionFrameSize = cfg.reflectionFrameSize;
    cw.rateDivisor = cfg.rateDivisor;
    cw.droppedFramesTotal.store(0, std::memory_order_relaxed);
    cw.slotEvictionsTotal.store(0, std::memory_order_relaxed);

    // Pool-level eviction tracking (Fix A, 2026-05-24): start with empty
    // prev-active set so the first iter's "everything is new" diff yields
    // zero evictions. Arrays pre-sized at MAX_ACTIVE_VOICES so the audio-
    // thread sweep never allocates. Schema buffers zeroed defensively.
    cw.prevPoolActiveCount = 0;
    for (int i = 0; i < ConvolutionWorker::kMaxSlots; ++i) {
        cw.prevPoolActiveHandles[i] = -1;
        cw.prevPoolActiveSchemas[i][0] = '\0';
    }
    // perWorkerSlotCap defaults to kMaxSlots (effectively unbounded). The
    // owner (AudioService) calls setPerWorkerSlotCap after init to tighten
    // it to ceil(mReverbVoices / numWorkers).

    // Polyphase upsample FIR for rateDivisor > 1. 16 taps × β=8 → ~75 dB
    // stopband. Cutoff 0.45/div on the output axis leaves a clean
    // transition before the image at 0.5/div. Group delay ≈0.32 ms at
    // div=2/48 kHz — inaudible vs dry-path latency.
    if (cw.rateDivisor > 1) {
        constexpr int kUpsampleSubLen = 16;
        cw.upsampleSubLen = kUpsampleSubLen;
        const double cutoffNorm = 0.45 / static_cast<double>(cw.rateDivisor);
        designUpsampleFIR(kUpsampleSubLen, cw.rateDivisor, cutoffNorm, /*beta=*/8.0,
                          cw.upsampleCoeffs);
    } else {
        cw.upsampleSubLen = 0;
        cw.upsampleCoeffs.clear();
    }
    cw.ambiChannels = cfg.ambiChannels;
    cw.ambiOrder = cfg.ambiOrder;
    cw.numWorkers = cfg.numWorkers;
    if (cfg.audioSettings.samplingRate > 0 && cfg.frameSize > 0) {
        cw.callbackPeriodMs = 1000.0f
            * static_cast<float>(cfg.frameSize)
            / static_cast<float>(cfg.audioSettings.samplingRate);
    }

    // Allocate shared per-voice mono staging buffers (triple-buffered).
    for (int b = 0; b < ConvolutionWorker::kStagingBuffers; ++b) {
        for (int i = 0; i < ConvolutionWorker::kMaxSlots; ++i) {
            cw.staging[b][i].mono.resize(cfg.reflectionFrameSize, 0.0f);
        }
        // Reverb wet-bus accumulation buffer (PR C part 2) — voices
        // without a per-voice reflection effect sum their sends here;
        // triple-buffered in lockstep with the staging slots (same
        // writeIdx). Allocated unconditionally: 3 × reflectionFrameSize
        // floats is trivial, and the bus must exist for runtime flips of
        // reverb_voices_realtime → 0.
        cw.busAccumMono[b].assign(cfg.reflectionFrameSize, 0.0f);
    }

    // Build ambisonics decode settings: stereo binaural via the shared HRTF.
    IPLAmbisonicsDecodeEffectSettings decodeSettings{};
    decodeSettings.speakerLayout.type = IPL_SPEAKERLAYOUTTYPE_STEREO;
    decodeSettings.hrtf = cfg.hrtf;
    decodeSettings.maxOrder = cfg.ambiOrder;

    IPLAudioSettings audioSettings = cfg.audioSettings;
    (void)cfg.reflSettings;  // was for the dropped mixer

    // K sub-workers, each with own decoder + scratch + output. Per-voice
    // ambisonics is accumulated manually into sub.ambiCh* (the reflection
    // mixer was dropped — it only supports CONVOLUTION/TAN).
    for (int i = 0; i < cfg.numWorkers; ++i)
        cw.workers.push_back(std::make_unique<ConvolutionSubWorker>());

    bool allWorkersOk = true;
    for (int wk = 0; wk < cfg.numWorkers; ++wk) {
        auto &sub = *cw.workers[wk];

        IPLerror err = iplAmbisonicsDecodeEffectCreate(cfg.context, &audioSettings,
                                               &decodeSettings, &sub.ambiDecodeEffect);
        if (err != IPL_STATUS_SUCCESS) {
            LOG_ERROR("ConvolutionWorkerPool: sub-worker %d decode create failed (%d)", wk, err);
            allWorkersOk = false;
            break;
        }

        // Allocate double-buffered stereo output (device sample rate)
        for (int b = 0; b < 2; ++b) {
            sub.stereoL[b].resize(cfg.frameSize, 0.0f);
            sub.stereoR[b].resize(cfg.frameSize, 0.0f);
        }

        // Scratch buffers (reflection rate)
        sub.ambiCh0.resize(cfg.reflectionFrameSize, 0.0f);
        if (cfg.ambiChannels > 1) sub.ambiCh1.resize(cfg.reflectionFrameSize, 0.0f);
        if (cfg.ambiChannels > 2) sub.ambiCh2.resize(cfg.reflectionFrameSize, 0.0f);
        if (cfg.ambiChannels > 3) sub.ambiCh3.resize(cfg.reflectionFrameSize, 0.0f);
        sub.decodedL.resize(cfg.reflectionFrameSize, 0.0f);
        sub.decodedR.resize(cfg.reflectionFrameSize, 0.0f);
        sub.voiceAmbi0.resize(cfg.reflectionFrameSize, 0.0f);
        if (cfg.ambiChannels > 1) sub.voiceAmbi1.resize(cfg.reflectionFrameSize, 0.0f);
        if (cfg.ambiChannels > 2) sub.voiceAmbi2.resize(cfg.reflectionFrameSize, 0.0f);
        if (cfg.ambiChannels > 3) sub.voiceAmbi3.resize(cfg.reflectionFrameSize, 0.0f);

        // Upsampler history — (subLen-1) decoded samples from the previous
        // reflection frame so the FIR has continuous input at boundaries.
        if (cw.upsampleSubLen > 1) {
            sub.prevDecodedL.assign(cw.upsampleSubLen - 1, 0.0f);
            sub.prevDecodedR.assign(cw.upsampleSubLen - 1, 0.0f);
        }

        // First-apply set starts empty; refilled lazily as voices appear.
        // (Per-sub-worker eviction tracking was moved to the pool level —
        // see ConvolutionWorker::prevPoolActive* — so the diff observes
        // the union of all sub-workers' assigned slots and worker-boundary
        // migrations no longer fire false [REFL_EVICT] events.)
        sub.firstAppliedCount = 0;
    }

    if (!allWorkersOk) {
        // Clean up partially created sub-workers
        for (auto &subPtr : cw.workers) {
            if (subPtr->ambiDecodeEffect)
                iplAmbisonicsDecodeEffectRelease(&subPtr->ambiDecodeEffect);
        }
        mWorker.reset();
        return false;
    }

    // Spawn sub-worker threads
    for (int wk = 0; wk < cfg.numWorkers; ++wk) {
        cw.workers[wk]->shutdown.store(false, std::memory_order_relaxed);
        cw.workers[wk]->thread = std::thread([this, wk]() { subWorkerMain(wk); });
    }
    return true;
}

//------------------------------------------------------
// T2.3 — periodic [PERF refl_worker] emission. Self-throttled to ~1 Hz;
// caller is expected to invoke from the main-thread loop step at any
// rate >=1 Hz. We snapshot+reset every sub-worker's queue-depth
// histogram and pick the worker with the most samples as the pool
// representative — matches the convention used by the existing
// [PERF worker] line. The dropped-frames + eviction counters are flat
// counters: dropped is exchanged-to-zero here so each emission window
// is independent; evictions is read (not exchanged) because the
// ProbeManager refl_cache line may want the same value within the
// same 1 s window.
//
// applyMs is owned by [PERF worker] in AudioService::dumpAudioStatusPeriodic;
// this method only owns the per-worker queue/dropped/evictions counters.
// Draining perfApplyMs here would double-empty the histogram (AudioService
// drains it first within the same dump cycle), so this method would always
// see n=0 and report applyMs=0.000 — exactly the bug T2.3 originally exposed.
void ConvolutionWorkerPool::pollPerfPeriodic()
{
    if (!mWorker) return;
    auto &cw = *mWorker;

    static std::chrono::steady_clock::time_point sLast{};
    auto now = std::chrono::steady_clock::now();
    if (sLast.time_since_epoch().count() != 0) {
        auto ms = std::chrono::duration<double, std::milli>(now - sLast).count();
        if (ms < 1000.0) return;
    }
    sLast = now;

    LatencyHistogram::Percentiles queueP{};
    for (auto &subPtr : cw.workers) {
        auto q = subPtr->perfQueueDepth.snapshotAndReset();
        if (q.n > queueP.n) queueP = q;
    }

    uint64_t dropped =
        cw.droppedFramesTotal.exchange(0, std::memory_order_relaxed);
    uint64_t evictions =
        cw.slotEvictionsTotal.load(std::memory_order_relaxed);

    AUDIO_LOG(
        "[PERF refl_worker] queue_n=%llu queue_p50=%.1f p95=%.1f p99=%.1f"
        " dropped=%llu evictions=%llu\n",
        static_cast<unsigned long long>(queueP.n),
        queueP.p50, queueP.p95, queueP.p99,
        static_cast<unsigned long long>(dropped),
        static_cast<unsigned long long>(evictions));
}

//------------------------------------------------------
void ConvolutionWorkerPool::shutdown()
{
    if (!mWorker) return;
    for (auto &subPtr : mWorker->workers) {
        subPtr->shutdown.store(true, std::memory_order_release);
        subPtr->frameSeq.fetch_add(1, std::memory_order_release);  // wake it
        subPtr->wakeCv.notify_one();  // condvar wake (yield-poll → cv since 2026-05)
    }
    for (auto &subPtr : mWorker->workers) {
        if (subPtr->thread.joinable())
            subPtr->thread.join();
        if (subPtr->ambiDecodeEffect)
            iplAmbisonicsDecodeEffectRelease(&subPtr->ambiDecodeEffect);
    }
    mWorker.reset();
}

//------------------------------------------------------
void ConvolutionWorkerPool::waitForCompletion()
{
    if (!mWorker) return;
    auto &cw = *mWorker;

    // Wait until ALL sub-workers have finished processing ALL frames signaled so far.
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    for (auto &subPtr : cw.workers) {
        auto &sub = *subPtr;
        uint64_t target = sub.frameSeq.load(std::memory_order_acquire);
        while (sub.processedSeq.load(std::memory_order_acquire) < target) {
            if (sub.shutdown.load(std::memory_order_relaxed)) break;
            if (std::chrono::steady_clock::now() >= deadline) {
                AUDIO_LOG("[AUDIO] WARNING: convolution sub-worker wait timed out "
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
void ConvolutionWorkerPool::subWorkerMain(int workerIdx)
{
    if (!mWorker) return;
    auto &cw = *mWorker;
    if (workerIdx < 0 || workerIdx >= static_cast<int>(cw.workers.size())) return;
    auto &sub = *cw.workers[workerIdx];
    uint64_t lastSeq = 0;

    // Convolution sub-workers run their own DSP (iplReflectionEffectApply,
    // iplAmbisonicsDecodeEffectApply) on dedicated threads.  Enable FTZ so
    // the per-thread MXCSR / FPCR matches the audio thread's denormal
    // handling — otherwise long-tail IRs can decay into denormals and
    // produce performance cliffs that look indistinguishable from broken
    // audio.
    audioEnableDenormalFlush();

    // Promote worker thread to USER_INTERACTIVE — the same QoS CoreAudio
    // uses for its callback thread, so the scheduler treats us as
    // user-perceptible-latency (P-core preference, low jitter). Spawned
    // threads inherit QOS_CLASS_DEFAULT from the main thread; at DEFAULT
    // a worker can sit unscheduled for >21 ms while DSP iter time is only
    // 5-12 ms, producing wet-bus amplitude modulation.
    {
        char threadName[16];
        std::snprintf(threadName, sizeof(threadName), "conv-w%d", workerIdx);
#if defined(__APPLE__)
        pthread_setname_np(threadName);
        int qosSetRc = pthread_set_qos_class_self_np(
            QOS_CLASS_USER_INTERACTIVE, 0);
        qos_class_t qos = QOS_CLASS_UNSPECIFIED;
        int relPri = 0;
        if (pthread_get_qos_class_np(pthread_self(), &qos, &relPri) == 0) {
            AUDIO_LOG("[WORKER_QOS] w=%d name=%s qos=%u rel=%d setRc=%d\n",
                      workerIdx, threadName,
                      static_cast<unsigned>(qos), relPri, qosSetRc);
        } else {
            AUDIO_LOG("[WORKER_QOS] w=%d name=%s qos=query-failed setRc=%d\n",
                      workerIdx, threadName, qosSetRc);
        }
#elif defined(__linux__)
        // No safe QoS equivalent on Linux (SCHED_FIFO requires CAP_SYS_NICE).
        pthread_setname_np(pthread_self(), threadName);
        int cpu = sched_getcpu();
        int policy = sched_getscheduler(0);
        AUDIO_LOG("[WORKER_QOS] w=%d name=%s startCpu=%d schedPolicy=%d\n",
                  workerIdx, threadName, cpu, policy);
#else
        AUDIO_LOG("[WORKER_QOS] w=%d name=%s (no introspection on this platform)\n",
                  workerIdx, threadName);
#endif
    }

    while (!sub.shutdown.load(std::memory_order_relaxed)) {
        // Wait for the mix node's frameSeq bump. Condvar wake with re-check
        // under lock — missed notify_one is harmless. Replaces yield-poll
        // which could miss bumps during macOS scheduler stalls.
        uint64_t seq;
        {
            std::unique_lock<std::mutex> lock(sub.wakeMtx);
            sub.wakeCv.wait(lock, [&] {
                return sub.frameSeq.load(std::memory_order_acquire) != lastSeq
                       || sub.shutdown.load(std::memory_order_relaxed);
            });
            if (sub.shutdown.load(std::memory_order_relaxed)) break;
            seq = sub.frameSeq.load(std::memory_order_acquire);
        }
        lastSeq = seq;

        auto t0 = std::chrono::steady_clock::now();

        // Signal→pickup latency. Captures wakeup + OS dispatch + cache
        // rewarm — none of which is in iter time. Skip while signalTimeNs=0
        // (bootstrap) to avoid recording a huge fake latency.
        const long long signalNs = sub.signalTimeNs.load(std::memory_order_acquire);
        if (signalNs > 0) {
            const long long pickupNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
                t0.time_since_epoch()).count() - signalNs;
            if (pickupNs >= 0) {
                sub.perfSignalPickupMs.record(static_cast<double>(pickupNs) / 1.0e6);
            }
        }

        // Per-iter accumulators feeding apply-distribution histograms.
        double iterApplySumMs    = 0.0;
        double iterApplyMaxMs    = 0.0;
        double iterSumStageMs    = 0.0;
        double iterDecodeMs      = 0.0;
        double iterUpsampleMs    = 0.0;

        // workersReading pre-set by mix node; we only decrement when done.
        int readBuf = cw.currentReadBuf;       // visible via frameSeq acquire
        int assignCount = sub.assignedCount;   // likewise
        int back = 1 - sub.frontIdx.load(std::memory_order_relaxed);

        // T2.3 queue depth: per-iter snapshot of assigned-slot count.
        // This is the "jobs waiting at iter start" for this sub-worker —
        // a proxy for queue depth that doesn't require a real submit-side
        // sample (the mix node's stagingCount is what feeds this, and
        // round-robin distributes evenly across workers).
        sub.perfQueueDepth.record(static_cast<double>(assignCount));

        std::memset(sub.stereoL[back].data(), 0, cw.frameSize * sizeof(float));
        std::memset(sub.stereoR[back].data(), 0, cw.frameSize * sizeof(float));

        // ── Voice-eviction sweep moved to the audio thread (Fix A, 2026-05-24) ──
        //
        // Previously each sub-worker maintained its own prev-active set and
        // diff. Voices that legitimately migrated across worker stripes (e.g.
        // load-balancing reorder) fired false [REFL_EVICT] events — one per
        // worker the voice left. A 15k-line MISS06 run showed 883 events
        // with only ~57 actual pinned voices.
        //
        // The diff now runs in AudioService::reflectionMixNodeProcess (one
        // owner, pool-wide visibility, runs once per audio callback before
        // bumping frameSeq). The audio thread updates
        // ConvolutionWorker::prevPoolActive* and the per-sub-worker
        // firstAppliedHandles cleanup via the same acquire-release barrier
        // workers already use to pick up assignedSlots.

        // Iter entry diagnostic — distinguishes "no work" from "work but
        // slots fail the early-continue checks".
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

        // Per-worker ambisonics accumulator — cleared per frame, then each
        // voice's apply output is summed in (replaces the dropped mixer).
        int nch = cw.ambiChannels;
        std::uint32_t reflFrameCount = static_cast<std::uint32_t>(cw.reflectionFrameSize);
        if (!sub.ambiCh0.empty())
            std::memset(sub.ambiCh0.data(), 0, reflFrameCount * sizeof(float));
        if (nch > 1 && !sub.ambiCh1.empty())
            std::memset(sub.ambiCh1.data(), 0, reflFrameCount * sizeof(float));
        if (nch > 2 && !sub.ambiCh2.empty())
            std::memset(sub.ambiCh2.data(), 0, reflFrameCount * sizeof(float));
        if (nch > 3 && !sub.ambiCh3.empty())
            std::memset(sub.ambiCh3.data(), 0, reflFrameCount * sizeof(float));

        // Per-voice apply → per-voice scratch → per-worker accumulator.
        for (int j = 0; j < assignCount; ++j) {
            int i = sub.assignedSlots[j];
            if (i < 0 || i >= ConvolutionWorker::kMaxSlots) continue;
            auto &slot = cw.staging[readBuf][i];
            // HYBRID: convolution head always needs a real IR (irSize > 0).
            // The parametric tail is driven by pinned reverbTimes alongside
            // that same IR — no separate gate needed.
            const bool slotParamsValid = (slot.params.irSize > 0);
            if (!slot.active || !slot.effect || !slotParamsValid)
                continue;
            if (slot.validityToken && !slot.validityToken->load(std::memory_order_acquire))
                continue;

            float *monoPtr = slot.mono.data();
            IPLAudioBuffer reflIn{};
            reflIn.numChannels = 1;
            reflIn.numSamples = static_cast<IPLint32>(slot.reflFrameSize);
            reflIn.data = &monoPtr;

            // Per-voice ambisonics scratch. mixer=nullptr writes directly
            // here. numChannels comes from slot.params and may be lower
            // than the effect's create-time max (CPU saver for distant
            // voices).
            float *voicePtrs[4] = {
                sub.voiceAmbi0.data(),
                (nch > 1 && !sub.voiceAmbi1.empty()) ? sub.voiceAmbi1.data() : nullptr,
                (nch > 2 && !sub.voiceAmbi2.empty()) ? sub.voiceAmbi2.data() : nullptr,
                (nch > 3 && !sub.voiceAmbi3.empty()) ? sub.voiceAmbi3.data() : nullptr
            };
            // Defensive clamp: numChannels must not exceed nch or the sum
            // loop below dereferences null voicePtrs entries.
            IPLAudioBuffer voiceOut{};
            voiceOut.numChannels = std::min(slot.params.numChannels, nch);
            voiceOut.numSamples  = static_cast<IPLint32>(slot.reflFrameSize);
            voiceOut.data        = voicePtrs;

            // Per-voice input level; saMeterReflW (post-sum) vs this is
            // the IR-added-energy ratio.
            sub.saMeterReflIn.measureMono(monoPtr, slot.reflFrameSize);

            // Apply: mixer=nullptr is the only overload that supports
            // HYBRID. Explicit timing feeds per-iter sum/max.
            {
                auto applyT0 = std::chrono::steady_clock::now();
                iplReflectionEffectApply(slot.effect, &slot.params,
                                          &reflIn, &voiceOut, /*mixer=*/nullptr);
                auto applyT1 = std::chrono::steady_clock::now();
                double applyMs = std::chrono::duration<double, std::milli>(
                    applyT1 - applyT0).count();
                sub.perfApplyMs.record(applyMs);
                iterApplySumMs += applyMs;
                if (applyMs > iterApplyMaxMs) iterApplyMaxMs = applyMs;
            }

            // ── IR-energy elision groundwork (measurement only) ──
            // Report the mean-square energy of the wet W (channel-0) output
            // back to the owning node so the main thread can observe how much
            // reverb this voice actually produces. This is the only available
            // ground truth for reflection energy — the IR itself is an opaque
            // IPLReflectionEffectIR handle. OBSERVATIONAL ONLY: nothing gates
            // elision on it yet (that needs a periodic re-probe so an elided
            // voice can be re-measured — see HANDOFF.STATICSOURCE.md §4.4).
            // validityToken was checked at the top of this slot's processing
            // and the worker-drain guarantees the node outlives this iteration.
            if (slot.outEnergyW) {
                const float *w = sub.voiceAmbi0.data();
                double e = 0.0;
                for (int s = 0; s < slot.reflFrameSize; ++s)
                    e += static_cast<double>(w[s]) * w[s];
                const float meanSq = slot.reflFrameSize > 0
                    ? static_cast<float>(e / slot.reflFrameSize)
                    : 0.0f;
                slot.outEnergyW->store(meanSq, std::memory_order_relaxed);
            }

            // ── T0.2 [REFL_FIRST_APPLY] — first apply per (voice, slot) ──
            //
            // Emit once per (this sub-worker, voice handle). The log line
            // captures the L1 norm of the ambisonic-W (channel 0) output
            // so the reader can distinguish:
            //   • IR exists but is silent (irNorm ≈ 0, irNumSamples > 0,
            //                              reflStatePtr non-null)
            //   • IR is null (reflStatePtr=0 — paired [FALLBACK] line)
            //   • IR is healthy (irNorm large, follow-up [REFLECTION_VOICE]
            //                    shows non-zero ratio)
            // We use the wet output (rather than reading the IR directly
            // — Steam Audio doesn't expose the IR samples) as a proxy:
            // an IR that produces zero output for a non-zero input on
            // its very first frame is almost certainly an unpopulated IR
            // or an uninitialised reflection-effect state (the T0.2
            // hypothesis).
            //
            // Limited to ~MAX_ACTIVE_VOICES*2 distinct handles per sub-
            // worker over the lifetime of the run. When the set fills,
            // new voices are still logged but old entries are not pruned
            // — the log goes quiet on the next eviction, which is fine.
            {
                int handle = slot.voiceHandle;
                bool already = false;
                for (int k = 0; k < sub.firstAppliedCount; ++k) {
                    if (sub.firstAppliedHandles[k] == handle) {
                        already = true;
                        break;
                    }
                }
                if (!already && handle >= 0) {
                    if (sub.firstAppliedCount < ConvolutionSubWorker::kFirstApplyTrackCap)
                        sub.firstAppliedHandles[sub.firstAppliedCount++] = handle;

                    double irNorm = 0.0;
                    const float *ch0 = sub.voiceAmbi0.data();
                    for (int s = 0; s < slot.reflFrameSize; ++s) {
                        irNorm += static_cast<double>(std::fabs(ch0[s]));
                    }
                    const char *schemaForLog = slot.schemaCStr
                                              ? slot.schemaCStr
                                              : "(unknown)";
                    AUDIO_LOG("[REFL_FIRST_APPLY] schema='%s' slot=%d "
                              "irNorm=%.6e irNumSamples=%d reflStatePtr=%p\n",
                              schemaForLog, i, irNorm, slot.params.irSize,
                              reinterpret_cast<void*>(slot.effect));

                    // Loud no-fallback announce per project memory
                    // feedback_no_silent_fallbacks: a null reflection-
                    // effect handle reached the worker — that should be
                    // impossible (the mix node gates on
                    // node->reflectionEffect) but if it ever happens we
                    // want a [FALLBACK] line, not a silent skip.
                    if (!slot.effect) {
                        std::fprintf(stderr,
                            "[FALLBACK] reflection state null for voice %d "
                            "(slot=%d schema=%s)\n",
                            handle, i, schemaForLog);
                    }
                }
            }

            // Sum per-voice output into per-worker accumulator.
            {
                auto sumT0 = std::chrono::steady_clock::now();
                const int voiceCh = std::min(slot.params.numChannels, nch);
                for (int c = 0; c < voiceCh; ++c) {
                    const float *src = voicePtrs[c];
                    float *dst = (c == 0) ? sub.ambiCh0.data()
                               : (c == 1) ? (sub.ambiCh1.empty() ? nullptr : sub.ambiCh1.data())
                               : (c == 2) ? (sub.ambiCh2.empty() ? nullptr : sub.ambiCh2.data())
                                          : (sub.ambiCh3.empty() ? nullptr : sub.ambiCh3.data());
                    if (!src || !dst) continue;
                    for (std::uint32_t s = 0; s < reflFrameCount; ++s)
                        dst[s] += src[s];
                }
                auto sumT1 = std::chrono::steady_clock::now();
                double sumMs = std::chrono::duration<double, std::milli>(
                    sumT1 - sumT0).count();
                sub.perfSumMs.record(sumMs);
                iterSumStageMs += sumMs;
            }

            // Per-Nth-slot diagnostic — peak per-voice is now directly
            // accessible in voiceAmbi0 (was hidden by the dropped mixer).
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

            // Per-voice [REFLECTION_VOICE] diagnostic — periodic dump of
            // every slot occupant's in/wet peak so the user can see at a
            // glance which voices are getting reflection processing AND
            // whether their wet output is non-silent. Helps diagnose the
            // "I have dry off but only some voices come through" class
            // of question: silent slots reveal weak baked IRs (probable
            // V8-class isolation) without requiring a re-bake. Rate-
            // limited at the per-worker level — each worker iteration
            // covers ~1/4 of the live slot set, so 1-of-N gating per
            // worker still produces a complete picture across a few
            // seconds of log.
            {
                static std::atomic<int> sReflVoiceLogCount{0};
                int n = sReflVoiceLogCount.fetch_add(1, std::memory_order_relaxed);
                // ~once per (period × workers × slots) iterations. With 4
                // workers × ~4 active slots × 100 callbacks/s ≈ 1600
                // iterations/s, period=256 → about 6 emissions/s across
                // all voices. Comfortable for log readability.
                if ((n & 0xFF) == 0) {
                    float inPeak = 0.0f;
                    for (int s = 0; s < slot.reflFrameSize; ++s)
                        inPeak = std::max(inPeak, std::fabs(slot.mono[s]));
                    float wPeak = 0.0f;
                    for (int s = 0; s < slot.reflFrameSize; ++s)
                        wPeak = std::max(wPeak, std::fabs(sub.voiceAmbi0[s]));
                    float inDb = inPeak > 1e-6f
                               ? 20.0f * std::log10(inPeak) : -120.0f;
                    float wDb  = wPeak  > 1e-6f
                               ? 20.0f * std::log10(wPeak)  : -120.0f;
                    const char *schema = slot.schemaCStr
                                       ? slot.schemaCStr
                                       : "(unknown)";
                    AUDIO_LOG("[REFLECTION_VOICE] h=%d schema='%s' "
                              "inPeak=%.4f (%.1fdB) wOutPeak=%.4f (%.1fdB) "
                              "ratio=%.3f irSize=%d nch=%d w=%d slot=%d\n",
                              slot.voiceHandle, schema,
                              inPeak, inDb, wPeak, wDb,
                              inPeak > 1e-6f ? wPeak / inPeak : 0.0f,
                              slot.params.irSize, slot.params.numChannels,
                              workerIdx, i);
                }
            }

            if (slot.isFootstepDiag) {
                static std::atomic<int> sFootWetLogCount{0};
                int n = sFootWetLogCount.fetch_add(1, std::memory_order_relaxed);
                if (n < 12) {
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

        // Release staging slot refs (disjoint across sub-workers — no contention).
        for (int j = 0; j < assignCount; ++j) {
            int i = sub.assignedSlots[j];
            if (i < 0 || i >= ConvolutionWorker::kMaxSlots) continue;
            auto &slot = cw.staging[readBuf][i];
            slot.effect = nullptr;
            slot.validityToken.reset();
            slot.active = false;
        }

        cw.workersReading.fetch_sub(1, std::memory_order_acq_rel);

        // ambiCh* already holds the per-worker summed ambisonics.
        float *ambiChannels[4] = {
            sub.ambiCh0.data(),
            nch > 1 ? sub.ambiCh1.data() : nullptr,
            nch > 2 ? sub.ambiCh2.data() : nullptr,
            nch > 3 ? sub.ambiCh3.data() : nullptr
        };
        IPLAudioBuffer ambiOut{};
        ambiOut.numChannels = nch;
        ambiOut.numSamples  = static_cast<IPLint32>(reflFrameCount);
        ambiOut.data        = ambiChannels;

        // Per-worker W (ambiCh0 = 0th SH = total omni energy). Peak well
        // above sa_reflI is the IR's resonance signature.
        if (!sub.ambiCh0.empty())
            sub.saMeterReflW.measureMono(sub.ambiCh0.data(), reflFrameCount);

        // Sanitise before decode — a single W-channel NaN becomes NaN in
        // both stereo channels post-decode.
        {
            bool sawBad = false;
            if (!sub.ambiCh0.empty())
                sawBad |= audioSanitizeBuffer(sub.ambiCh0.data(), reflFrameCount);
            if (nch > 1 && !sub.ambiCh1.empty())
                sawBad |= audioSanitizeBuffer(sub.ambiCh1.data(), reflFrameCount);
            if (nch > 2 && !sub.ambiCh2.empty())
                sawBad |= audioSanitizeBuffer(sub.ambiCh2.data(), reflFrameCount);
            if (nch > 3 && !sub.ambiCh3.empty())
                sawBad |= audioSanitizeBuffer(sub.ambiCh3.data(), reflFrameCount);
            if (sawBad) {
                uint32_t n = sub.nanCountAmbi.fetch_add(
                    1, std::memory_order_relaxed);
                if (n < 4) {
                    AUDIO_LOG("[NAN_GUARD] w=%d ambiCh* had non-finite samples "
                              "BEFORE ambisonics decode (occurrence %u) "
                              "assignCount=%d nch=%d\n",
                              workerIdx, n + 1, assignCount, nch);
                }
            }
        }

        // Ambisonics → binaural stereo.
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
        {
            auto decT0 = std::chrono::steady_clock::now();
            iplAmbisonicsDecodeEffectApply(sub.ambiDecodeEffect, &decodeParams,
                                            &ambiOut, &decodedBuf);
            auto decT1 = std::chrono::steady_clock::now();
            iterDecodeMs = std::chrono::duration<double, std::milli>(
                decT1 - decT0).count();
            sub.perfDecodeMs.record(iterDecodeMs);
        }

        // Sanitise decoded stereo before it lands in the front buffer the
        // master mix node reads. Mix node has its own guard but stopping
        // it here pins decoder-vs-accumulator as the NaN producer.
        {
            bool badL = audioSanitizeBuffer(sub.decodedL.data(), reflFrameCount);
            bool badR = audioSanitizeBuffer(sub.decodedR.data(), reflFrameCount);
            if (badL || badR) {
                uint32_t n = sub.nanCountDecode.fetch_add(
                    1, std::memory_order_relaxed);
                if (n < 4) {
                    AUDIO_LOG("[NAN_GUARD] w=%d decoded stereo had non-finite "
                              "samples AFTER ambisonics decode (occurrence %u) "
                              "L=%d R=%d assignCount=%d\n",
                              workerIdx, n + 1, badL ? 1 : 0, badR ? 1 : 0,
                              assignCount);
                }
            }
        }

        // Post-decode stereo. Mix node merges across sub-workers for the
        // unified [GAIN] wet bus level.
        sub.saMeterDecodeOut.measureDeinterleaved(
            sub.decodedL.data(), sub.decodedR.data(), reflFrameCount);

        // Decoded stereo → back buffer, upsampling from reflection rate
        // to device rate. Explicit timing feeds the per-iter residual calc.
        auto upT0 = std::chrono::steady_clock::now();
        int div = cw.rateDivisor;
        if (div > 1) {
            // Polyphase FIR upsample. Layout: coeffs[p*subLen + t], phase
            // p in 0..div-1, tap t in 0..subLen-1. For output index
            // k*div+p, sub-filter p convolves x[k], x[k-1], ..., the
            // first (subLen-1) drawn from prevDecoded*, rest from decoded*.
            const int subLen = cw.upsampleSubLen;
            const std::uint32_t inFrames = std::min(reflFrameCount,
                static_cast<std::uint32_t>(cw.frameSize / div));
            if (inFrames > 0 && subLen > 1) {
                const float *coefs = cw.upsampleCoeffs.data();
                const float *prevL = sub.prevDecodedL.data();
                const float *prevR = sub.prevDecodedR.data();
                const float *dL    = sub.decodedL.data();
                const float *dR    = sub.decodedR.data();
                float       *outL  = sub.stereoL[back].data();
                float       *outR  = sub.stereoR[back].data();
                const int    histN = subLen - 1;

                for (std::uint32_t k = 0; k < inFrames; ++k) {
                    const int kI = static_cast<int>(k);
                    for (int p = 0; p < div; ++p) {
                        const float *subCoef = coefs + p * subLen;
                        float accL = 0.0f;
                        float accR = 0.0f;
                        // idx<0 indexes prev* (oldest-first: prev[0] is the
                        // oldest sample, prev[histN-1] the most recent).
                        for (int t = 0; t < subLen; ++t) {
                            const int idx = kI - t;
                            const float xL = (idx >= 0)
                                ? dL[idx]
                                : prevL[histN + idx];
                            const float xR = (idx >= 0)
                                ? dR[idx]
                                : prevR[histN + idx];
                            accL += subCoef[t] * xL;
                            accR += subCoef[t] * xR;
                        }
                        outL[static_cast<size_t>(k) * div + p] = accL;
                        outR[static_cast<size_t>(k) * div + p] = accR;
                    }
                }

                // Roll delay line. New history = last histN samples of
                // [old_history, new_block].
                const int F = static_cast<int>(inFrames);
                if (F >= histN) {
                    for (int i = 0; i < histN; ++i) {
                        sub.prevDecodedL[i] = dL[F - histN + i];
                        sub.prevDecodedR[i] = dR[F - histN + i];
                    }
                } else {
                    const int keep = histN - F;
                    for (int i = 0; i < keep; ++i) {
                        sub.prevDecodedL[i] = sub.prevDecodedL[i + F];
                        sub.prevDecodedR[i] = sub.prevDecodedR[i + F];
                    }
                    for (int i = 0; i < F; ++i) {
                        sub.prevDecodedL[keep + i] = dL[i];
                        sub.prevDecodedR[keep + i] = dR[i];
                    }
                }
            }
        } else {
            std::uint32_t outSamples = std::min(reflFrameCount, static_cast<std::uint32_t>(cw.frameSize));
            std::memcpy(sub.stereoL[back].data(), sub.decodedL.data(), outSamples * sizeof(float));
            std::memcpy(sub.stereoR[back].data(), sub.decodedR.data(), outSamples * sizeof(float));
        }
        // Buffer-flip atomics + processedSeq are intentionally outside the
        // upsample span — they accrue to perfResidualMs.
        {
            auto upT1 = std::chrono::steady_clock::now();
            iterUpsampleMs = std::chrono::duration<double, std::milli>(
                upT1 - upT0).count();
            sub.perfUpsampleMs.record(iterUpsampleMs);
        }

        // Swap.
        sub.frontIdx.store(back, std::memory_order_release);
        sub.hasProducedOutput.store(true, std::memory_order_release);

        auto t1 = std::chrono::steady_clock::now();
        float ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
        float prev = sub.peakMs.load(std::memory_order_relaxed);
        if (ms > prev) sub.peakMs.store(ms, std::memory_order_relaxed);
        sub.perfIterMs.record(static_cast<double>(ms));

        // Skip sum/max histograms when assignCount=0 — zero-fill would
        // dominate the p50 and mask the real distribution. perfResidualMs
        // is always recorded (the non-DSP floor exists every iter).
        if (assignCount > 0) {
            sub.perfSumApplyMs.record(iterApplySumMs);
            sub.perfMaxApplyMs.record(iterApplyMaxMs);
        }
        const double iterTotalMs       = static_cast<double>(ms);
        const double knownStagesMs     = iterApplySumMs + iterSumStageMs
                                       + iterDecodeMs + iterUpsampleMs;
        const double residualMs        = (iterTotalMs > knownStagesMs)
                                       ? (iterTotalMs - knownStagesMs)
                                       : 0.0;
        sub.perfResidualMs.record(residualMs);

        // Iter-overrun diagnostic. Sync-in-callback waits on processedSeq,
        // so an overrun forces a sync-wait timeout in the mix node.
        // Threshold = 80% of callback period (before the mix node's 70%
        // wait deadline). Throttled to once per 64 overruns per worker.
        const float cbMs = cw.callbackPeriodMs;
        const float lagThresholdMs = 0.80f * cbMs;
        if (ms > lagThresholdMs) {
            static std::atomic<uint32_t> sLagLogCount[16] = {};
            int idx = workerIdx & 0xF;
            uint32_t n = sLagLogCount[idx].fetch_add(1, std::memory_order_relaxed);
            if ((n & 0x3F) == 0) {
                AUDIO_LOG("[REFL_LAG] w=%d ms=%.2f assignCount=%d "
                          "(overrun #%u, cb=%.2fms thr=%.2fms)\n",
                          workerIdx, ms, assignCount, n + 1,
                          cbMs, lagThresholdMs);
            }
        }

        // Signal completion — wakes the mix node's sync-wait CV. Lock
        // contention is briefly possible only mid-predicate-check.
        {
            std::lock_guard<std::mutex> lk(mWorker->doneMtx);
            sub.processedSeq.store(lastSeq, std::memory_order_release);
        }
        mWorker->doneCv.notify_one();
    }
}

} // namespace Darkness
