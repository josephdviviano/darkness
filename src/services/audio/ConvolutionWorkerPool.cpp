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
#include <cstring>
#include <thread>

#include <phonon.h>

#include "logger.h"

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
    IPLReflectionEffectSettings reflSettings = cfg.reflSettings;

    // Create K sub-workers, each with own mixer + decoder + scratch + output
    for (int i = 0; i < cfg.numWorkers; ++i)
        cw.workers.push_back(std::make_unique<ConvolutionSubWorker>());

    bool allWorkersOk = true;
    for (int wk = 0; wk < cfg.numWorkers; ++wk) {
        auto &sub = *cw.workers[wk];

        IPLerror err = iplReflectionMixerCreate(cfg.context, &audioSettings,
                                                 &reflSettings, &sub.mixer);
        if (err != IPL_STATUS_SUCCESS) {
            LOG_ERROR("ConvolutionWorkerPool: sub-worker %d mixer create failed (%d)", wk, err);
            allWorkersOk = false;
            break;
        }

        err = iplAmbisonicsDecodeEffectCreate(cfg.context, &audioSettings,
                                               &decodeSettings, &sub.ambiDecodeEffect);
        if (err != IPL_STATUS_SUCCESS) {
            LOG_ERROR("ConvolutionWorkerPool: sub-worker %d decode create failed (%d)", wk, err);
            iplReflectionMixerRelease(&sub.mixer);
            sub.mixer = nullptr;
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
            if (subPtr->mixer)
                iplReflectionMixerRelease(&subPtr->mixer);
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
    }
    for (auto &subPtr : mWorker->workers) {
        if (subPtr->thread.joinable())
            subPtr->thread.join();
        if (subPtr->ambiDecodeEffect)
            iplAmbisonicsDecodeEffectRelease(&subPtr->ambiDecodeEffect);
        if (subPtr->mixer)
            iplReflectionMixerRelease(&subPtr->mixer);
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

            // Gain-staging: record the mono input level to convolution
            // (per voice, this sub-worker only).  The convolution IR can
            // be tens of thousands of samples long and accumulates a
            // ringing tail; the saMeterReflW (measured AFTER the mixer
            // apply, below) vs saMeterReflIn ratio captures how much
            // energy the IRs are adding for the current listener/source
            // set.
            sub.saMeterReflIn.measureMono(monoPtr, slot.reflFrameSize);

            // Apply convolution, accumulating into this sub-worker's
            // mixer.  Important: when the 5th arg (mixer) is non-null,
            // Steam Audio writes the per-voice result *into the mixer*
            // and leaves reflOut/ambiPtrs untouched — so we cannot
            // measure per-voice W here.  W is measured once after
            // iplReflectionMixerApply extracts the summed result below.
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
        std::uint32_t reflFrameCount = static_cast<std::uint32_t>(cw.reflectionFrameSize);
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

        // Gain-staging: now that the mixer has extracted the summed
        // per-voice convolution into ambiCh0..3, measure the W channel
        // (ambiCh0 == 0th SH coefficient == total omni energy).  This
        // is the correct measurement point — measuring inside the
        // iplReflectionEffectApply loop reads stale data because that
        // call routes the per-voice result into the mixer when the
        // mixer arg is non-null.  A peak here significantly above
        // sa_reflI is the resonance signature for the convolution IR.
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
        iplAmbisonicsDecodeEffectApply(sub.ambiDecodeEffect, &decodeParams,
                                        &ambiOut, &decodedBuf);

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

        // Swap: make back buffer the new front
        sub.frontIdx.store(back, std::memory_order_release);
        sub.hasProducedOutput.store(true, std::memory_order_release);

        auto t1 = std::chrono::steady_clock::now();
        float ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
        float prev = sub.peakMs.load(std::memory_order_relaxed);
        if (ms > prev) sub.peakMs.store(ms, std::memory_order_relaxed);

        // Diagnostic: log iterations that risk overrunning the audio
        // callback period (1024 @ 48 kHz = ~21.3 ms).  When ms exceeds
        // the threshold, the mix node on the next callback is likely to
        // read a still-stale `front` buffer because the worker hasn't
        // flipped frontIdx yet — the front-buffer freshness counter in
        // [WET_BUS] will rise in lockstep.  Throttled to ~once per 64
        // overruns per sub-worker so the log stays usable when the
        // pipeline is consistently behind.
        if (ms > 18.0f) {
            static std::atomic<uint32_t> sLagLogCount[16] = {};
            int idx = workerIdx & 0xF;
            uint32_t n = sLagLogCount[idx].fetch_add(1, std::memory_order_relaxed);
            if ((n & 0x3F) == 0) {
                AUDIO_LOG("[CONV_LAG] w=%d ms=%.2f assignCount=%d "
                          "(overrun #%u, cb=21.3ms)\n",
                          workerIdx, ms, assignCount, n + 1);
            }
        }

        // Signal that this frame's data has been fully processed
        sub.processedSeq.store(lastSeq, std::memory_order_release);
    }
}

} // namespace Darkness
