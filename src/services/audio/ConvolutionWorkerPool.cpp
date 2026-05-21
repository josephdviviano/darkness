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

// Thread naming + QoS introspection for the one-shot [WORKER_QOS] log.
// On macOS the OS scheduler can place threads on either P-cores (fast) or
// E-cores (~2-3× slower for FFT-bound work), so confirming each worker's
// scheduling class is the first step in diagnosing iter-time tail latency.
#if defined(__APPLE__)
#  include <pthread.h>
#  include <pthread/qos.h>
#elif defined(__linux__)
#  include <pthread.h>
#  include <sched.h>
#endif

namespace Darkness {

// ── Sample-sanitization helper ────────────────────────────────────────
//
// The convolution worker can emit non-finite samples when an IR contains
// a denormal-edge condition or a NaN slipped into the convolution input.
// Mirror the inline helper that AudioService.cpp uses for its master DSP
// chain so the worker can stop bad samples at the source rather than
// rely on the mix node's downstream guard.
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

// ── Denormal flush (worker threads) ───────────────────────────────────
//
// Long-tail IRs decaying into denormals are the canonical "audio worker
// suddenly takes 50× longer to process a frame" pitfall. Mirror the FTZ
// helper that AudioService.cpp installs on the audio thread — we don't
// share the symbol since it's a local helper inside that TU.
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
    cw.ambiChannels = cfg.ambiChannels;
    cw.ambiOrder = cfg.ambiOrder;
    cw.numWorkers = cfg.numWorkers;
    // Cache the audio callback period so [CONV_LAG] can size its threshold
    // (default 80 % of the period) and report the period in its log line.
    // Falls back to the historical 21.333 ms (1024 @ 48 kHz) if the
    // settings struct is unfilled.
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
    }

    // Build ambisonics decode settings: stereo binaural via the shared HRTF.
    IPLAmbisonicsDecodeEffectSettings decodeSettings{};
    decodeSettings.speakerLayout.type = IPL_SPEAKERLAYOUTTYPE_STEREO;
    decodeSettings.hrtf = cfg.hrtf;
    decodeSettings.maxOrder = cfg.ambiOrder;

    IPLAudioSettings audioSettings = cfg.audioSettings;
    (void)cfg.reflSettings;  // reflSettings was used for the mixer; mixer is gone.

    // Create K sub-workers, each with own decoder + scratch + output.
    // Mixers were dropped in PLAN.HYBRID_REVERB.md Phase 3 — Steam Audio's
    // reflection mixer only supports CONVOLUTION/TAN, not HYBRID/PARAMETRIC.
    // We accumulate per-voice ambisonics manually into sub.ambiCh* in
    // subWorkerMain, which works for all three modes.
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

    // Promote the worker thread to a real-time-grade scheduling class
    // BEFORE doing any other work, so the first iter benefits.
    //
    // Spawned threads inherit QoS from the caller (the audio init path on
    // the main thread, which lands at QOS_CLASS_DEFAULT = 21 on macOS).
    // At DEFAULT the scheduler can preempt the worker on equal footing
    // with the render thread; under load that has been observed to leave
    // a worker un-scheduled for >21 ms (one audio-callback period), at
    // which point the mix node reads the previous frame and the wet bus
    // is held flat — audible amplitude modulation on the reverb. Diagnosis
    // pinned this on the H1 hypothesis in HANDOFF.AUDIO_PERFORMANCE_TUNING.md:
    // iter time is 5–12 ms (well under the 21 ms budget) yet bursts of
    // 1–4 stale callbacks still occur, which can only be a wait-for-signal
    // scheduling gap, not a DSP overrun.
    //
    // USER_INTERACTIVE is what CoreAudio uses for its own callback thread;
    // matching that here puts the workers at the same priority tier and
    // tells the macOS scheduler these are user-perceptible-latency tasks
    // (→ P-core preference, low scheduler-jitter). The CoreAudio thread
    // itself is unaffected by our setting — we're only raising the workers
    // to its level, not above. No realtime-budget syscall is involved
    // (cf. `thread_policy_set` with `THREAD_TIME_CONSTRAINT_POLICY`) since
    // that requires per-frame budget hints we'd have to keep in sync with
    // the audio-callback period.
    //
    // We log the post-set QoS via `pthread_get_qos_class_np` so the
    // [WORKER_QOS] log line in production output confirms the value stuck
    // (a silent failure would leave qos=21 visible and reveal it).
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
        // Linux has no QoS equivalent; the closest path is SCHED_FIFO/RR
        // via pthread_setschedparam, which requires CAP_SYS_NICE or root
        // and is unsafe to enable by default. The same wobble symptom
        // would need to be confirmed on Linux before adding a privileged
        // priority bump there.
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
        // Wait for the mix node to signal new data. Condvar-based wake
        // (replaces previous yield-poll loop, 2026-05). yield-poll spun
        // the worker thread, and during macOS scheduler stalls — most
        // notably at iplSimulatorRunReflections cycle completion when
        // 6 sim threads all become non-runnable at once — the worker
        // could miss a frameSeq bump for an entire callback period
        // (~21 ms). Because the worker then jumps straight to the
        // latest seq without processing the missed frame, the wet bus
        // mix node observed a stale `processedSeq` for that callback
        // and replayed the previous 21.3 ms of reverb output —
        // producing the per-sim-cycle "thump" / wobble artefact.
        // The cv.wait predicate re-checks frameSeq atomically under
        // lock, so a missed notify_one (worker not in wait at the
        // moment of notify) is harmless. See NOTES.PROJECT.md
        // "Reflection sim ↔ conv worker contention pattern".
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

        // H2' signal→pickup latency. The mix node wrote signalTimeNs
        // immediately before bumping frameSeq; the acquire load above
        // makes that write visible here. The gap captures yield-poll
        // wakeup latency + OS dispatch + cache rewarm — none of which
        // is included in iter time (which starts at t0). signalTimeNs=0
        // is the bootstrap value before the first bump; skip the record
        // until the producer has actually written one to avoid recording
        // an enormous fake latency.
        const long long signalNs = sub.signalTimeNs.load(std::memory_order_acquire);
        if (signalNs > 0) {
            const long long pickupNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
                t0.time_since_epoch()).count() - signalNs;
            if (pickupNs >= 0) {
                sub.perfSignalPickupMs.record(static_cast<double>(pickupNs) / 1.0e6);
            }
        }

        // Per-iteration accumulators for H2 (apply-distribution histograms).
        // These mirror the existing per-call histograms but aggregate over
        // the iter so we can attribute the iter-time tail to either the
        // accumulated apply cost, a single dominant voice, or non-DSP
        // residual (memset/atomics/scheduler). Cost: a few doubles in
        // automatic storage + a few add/max ops per voice; negligible.
        double iterApplySumMs    = 0.0;
        double iterApplyMaxMs    = 0.0;
        double iterSumStageMs    = 0.0;
        double iterDecodeMs      = 0.0;
        double iterUpsampleMs    = 0.0;

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

        // Per-worker ambisonics accumulator. Cleared each frame, then we add
        // each voice's iplReflectionEffectApply output into it. Replaces the
        // pre-Phase-3 iplReflectionMixer accumulation path (which Steam Audio
        // forbids for HYBRID/PARAMETRIC).
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

        // Process assigned voice slots — one iplReflectionEffectApply per
        // voice into per-voice scratch (voiceAmbi*), then sum into the
        // per-worker accumulator (ambiCh*).
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

            // Build output buffer pointing at the per-voice ambisonics
            // scratch. iplReflectionEffectApply with mixer=nullptr writes
            // the result directly here. numChannels here is the per-call
            // count from slot.params, which may be lower than the effect's
            // create-time max (used as a CPU saver for distant voices).
            float *voicePtrs[4] = {
                sub.voiceAmbi0.data(),
                (nch > 1 && !sub.voiceAmbi1.empty()) ? sub.voiceAmbi1.data() : nullptr,
                (nch > 2 && !sub.voiceAmbi2.empty()) ? sub.voiceAmbi2.data() : nullptr,
                (nch > 3 && !sub.voiceAmbi3.empty()) ? sub.voiceAmbi3.data() : nullptr
            };
            // Defensive clamp: numChannels MUST NOT exceed the worker's
            // own channel count (nch). The audio thread populates this
            // from mAmbisonicsChannels which equals nch in current use,
            // but if a future staging path forgets to set numChannels or
            // a wrong-mode params struct sneaks through, the loop below
            // would dereference null entries in voicePtrs. Cheap min().
            IPLAudioBuffer voiceOut{};
            voiceOut.numChannels = std::min(slot.params.numChannels, nch);
            voiceOut.numSamples  = static_cast<IPLint32>(slot.reflFrameSize);
            voiceOut.data        = voicePtrs;

            // Gain-staging: per-voice input level to convolution. The
            // convolution IR can be tens of thousands of samples long and
            // accumulates a ringing tail; the saMeterReflW (measured after
            // the per-worker sum below) vs saMeterReflIn ratio captures
            // how much energy the IRs are adding for the current
            // listener/source set.
            sub.saMeterReflIn.measureMono(monoPtr, slot.reflFrameSize);

            // Apply the reflection effect into the per-voice scratch
            // (mixer=nullptr → direct buffer output). This is the only
            // overload that supports HYBRID/PARAMETRIC; the mixer overload
            // is restricted to CONVOLUTION/TAN.
            //
            // Explicit timing (rather than ScopedLatencyTimer) so the
            // elapsed value also feeds the per-iter sum/max accumulators
            // for the perfSumApplyMs / perfMaxApplyMs histograms.
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

            // Sum per-voice output into the per-worker accumulator.
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

            // Diagnostic: log every Nth slot the worker processes. With
            // manual-sum mode the per-voice peak is directly available in
            // voiceAmbi0 (was inaccessible under the mixer path).
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

        // Build the IPLAudioBuffer view of the summed ambisonics. ambiCh*
        // already holds the per-worker total — no mixer-extract call needed.
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

        // Gain-staging: per-worker W (ambiCh0 = 0th SH coefficient = total
        // omni energy). A peak here significantly above sa_reflI is the
        // resonance signature for the convolution IR.
        if (!sub.ambiCh0.empty())
            sub.saMeterReflW.measureMono(sub.ambiCh0.data(), reflFrameCount);

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
        {
            auto decT0 = std::chrono::steady_clock::now();
            iplAmbisonicsDecodeEffectApply(sub.ambiDecodeEffect, &decodeParams,
                                            &ambiOut, &decodedBuf);
            auto decT1 = std::chrono::steady_clock::now();
            iterDecodeMs = std::chrono::duration<double, std::milli>(
                decT1 - decT0).count();
            sub.perfDecodeMs.record(iterDecodeMs);
        }

        // Sanitize the decoded stereo from the ambisonics decoder before it
        // lands in the front buffer that the master mix node reads.  A NaN
        // here would otherwise propagate into stereoOut and poison the
        // master EQ.  The mix node has its own guard for this (nanCountWet)
        // but stopping it at the source is cheaper and gives the worker
        // thread credit for the detection in the log.
        audioSanitizeBuffer(sub.decodedL.data(), reflFrameCount);
        audioSanitizeBuffer(sub.decodedR.data(), reflFrameCount);

        // Gain-staging: post-decode stereo.  The mix node combines this
        // with all sub-workers' meters via StageMeter::mergeIn so the
        // [GAIN] dump shows a unified wet bus level.  Peak above the
        // saMeterReflW peak indicates the ambisonics decoder + HRTF is
        // amplifying — usually fine and expected (HRTF can boost) but
        // useful to watch alongside the dry binaural meter.
        sub.saMeterDecodeOut.measureDeinterleaved(
            sub.decodedL.data(), sub.decodedR.data(), reflFrameCount);

        // Write decoded stereo to this sub-worker's back buffer.
        // Handle upsampling from reflection rate to device rate.
        //
        // Explicit timing (rather than ScopedLatencyTimer) so the elapsed
        // value also feeds the per-iter residual-time calculation below.
        auto upT0 = std::chrono::steady_clock::now();
        int div = cw.rateDivisor;
        if (div > 1) {
            // Linear interpolation upsample by rateDivisor.
            //
            // Anchoring: the device-rate sample at position j*div should
            // equal the reflection-rate sample decodedL[j-1] (i.e. each
            // reflection sample defines the START of its own div-sample
            // group, and the slope to the next reflection sample fills
            // the remainder).  To keep the bridge between consecutive
            // reflection frames continuous, we hold the last decoded
            // sample of the previous frame as state in
            // prevDecodedTail{L,R} and use it as the "left anchor" for
            // the first output group of this frame.  Cost: one
            // reflection-rate sample of latency (~42 µs at 24 kHz),
            // inaudible.  Without this, the previous code held the
            // final decoded sample flat for div samples and then
            // started the next frame at decodedL[0] — a step
            // discontinuity at every frame boundary, audible as a low
            // buzz at reflRate / reflFrameSize Hz.
            std::uint32_t inFrames = std::min(reflFrameCount,
                static_cast<std::uint32_t>(cw.frameSize / div));
            if (inFrames > 0) {
                // Bridge from prev-frame tail → this frame's first sample.
                {
                    float l0 = sub.prevDecodedTailL, l1 = sub.decodedL[0];
                    float r0 = sub.prevDecodedTailR, r1 = sub.decodedR[0];
                    for (int d = 0; d < div; ++d) {
                        float t = static_cast<float>(d) / static_cast<float>(div);
                        sub.stereoL[back][d] = l0 + (l1 - l0) * t;
                        sub.stereoR[back][d] = r0 + (r1 - r0) * t;
                    }
                }
                // Middle groups: decodedL[j] → decodedL[j+1], written to
                // device-rate positions (j+1)*div .. (j+2)*div - 1.
                for (std::uint32_t j = 0; j + 1 < inFrames; ++j) {
                    float l0 = sub.decodedL[j],   l1 = sub.decodedL[j + 1];
                    float r0 = sub.decodedR[j],   r1 = sub.decodedR[j + 1];
                    for (int d = 0; d < div; ++d) {
                        float t = static_cast<float>(d) / static_cast<float>(div);
                        sub.stereoL[back][(j + 1) * div + d] = l0 + (l1 - l0) * t;
                        sub.stereoR[back][(j + 1) * div + d] = r0 + (r1 - r0) * t;
                    }
                }
                // Save tail sample for the next frame's bridge.
                sub.prevDecodedTailL = sub.decodedL[inFrames - 1];
                sub.prevDecodedTailR = sub.decodedR[inFrames - 1];
            }
        } else {
            std::uint32_t outSamples = std::min(reflFrameCount, static_cast<std::uint32_t>(cw.frameSize));
            std::memcpy(sub.stereoL[back].data(), sub.decodedL.data(), outSamples * sizeof(float));
            std::memcpy(sub.stereoR[back].data(), sub.decodedR.data(), outSamples * sizeof(float));
        }
        // Close the upsample timing span (opened just before `int div = ...`).
        // The buffer-flip atomics + processedSeq store below are intentionally
        // outside this span — they're accounted for in the perfResidualMs
        // bucket via `iter − (sum stages)` below.
        {
            auto upT1 = std::chrono::steady_clock::now();
            iterUpsampleMs = std::chrono::duration<double, std::milli>(
                upT1 - upT0).count();
            sub.perfUpsampleMs.record(iterUpsampleMs);
        }

        // Swap: make back buffer the new front
        sub.frontIdx.store(back, std::memory_order_release);
        sub.hasProducedOutput.store(true, std::memory_order_release);

        auto t1 = std::chrono::steady_clock::now();
        float ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
        float prev = sub.peakMs.load(std::memory_order_relaxed);
        if (ms > prev) sub.peakMs.store(ms, std::memory_order_relaxed);
        // Also feed the per-iteration histogram — peakMs gives the worst
        // single iteration since the last reset; perfIterMs gives the full
        // distribution (p50/p95/p99) over the window.
        sub.perfIterMs.record(static_cast<double>(ms));

        // H2: per-iter apply-distribution + residual histograms.
        //
        // Only record sum/max when this iter actually processed voices —
        // an iter with assignCount=0 has iterApplySumMs==0 and would
        // dominate the p50 with zeros, masking the real distribution.
        // perfResidualMs is always recorded: it captures the non-DSP
        // floor (memset + entry diag + atomics + buffer flip + scheduler
        // preemption) which exists every iter regardless of voice count.
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

        // Diagnostic: log iterations that risk overrunning the audio
        // callback period (frameSize / sampleRate). Sync-in-callback means
        // the audio thread waits for our processedSeq before reading wet —
        // so an overrun directly forces a deadline timeout in the mix
        // node, which the [PERF sync_wait] "timeouts" counter records.
        // Threshold = 80 % of callback period; warns before the mix
        // node's 70 % wait deadline so we can attribute timeouts to a
        // specific worker. Throttled to ~once per 64 overruns per
        // sub-worker so the log stays usable when the pipeline is
        // consistently behind.
        const float cbMs = cw.callbackPeriodMs;
        const float lagThresholdMs = 0.80f * cbMs;
        if (ms > lagThresholdMs) {
            static std::atomic<uint32_t> sLagLogCount[16] = {};
            int idx = workerIdx & 0xF;
            uint32_t n = sLagLogCount[idx].fetch_add(1, std::memory_order_relaxed);
            if ((n & 0x3F) == 0) {
                AUDIO_LOG("[CONV_LAG] w=%d ms=%.2f assignCount=%d "
                          "(overrun #%u, cb=%.2fms thr=%.2fms)\n",
                          workerIdx, ms, assignCount, n + 1,
                          cbMs, lagThresholdMs);
            }
        }

        // Signal that this frame's data has been fully processed
        sub.processedSeq.store(lastSeq, std::memory_order_release);
    }
}

} // namespace Darkness
