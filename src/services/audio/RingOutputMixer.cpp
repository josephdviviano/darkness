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

// RingOutputMixer.cpp — see RingOutputMixer.h for the design + threading
// contract (PLAN.AUDIO_PERF.md PR D).

#include "RingOutputMixer.h"

#include <chrono>
#include <cstdio>
#include <cstring>

#include "logger.h"

#if defined(__APPLE__)
#  include <pthread.h>
#  include <pthread/qos.h>
#elif defined(__linux__)
#  include <pthread.h>
#  include <sched.h>
#endif

// Denormal / flush-to-zero for the mixer thread. Same helper as the audio
// callback path (AudioService.cpp) and the convolution sub-workers
// (ConvolutionWorkerPool.cpp): FTZ/DAZ is a PER-THREAD FPCR/MXCSR setting,
// so the thread that now runs the whole node graph must enable it itself or
// long IIR/convolution tails decay into denormals and hit the 10-100×
// microcode slowdown that this thread exists to avoid.
#if defined(__aarch64__) || defined(_M_ARM64)
static inline void ringMixerEnableDenormalFlush()
{
    uint64_t fpcr;
    __asm__ __volatile__("mrs %0, fpcr" : "=r"(fpcr));
    fpcr |= (1ull << 24); // FZ — flush denormals to zero
    __asm__ __volatile__("msr fpcr, %0" :: "r"(fpcr));
}
#elif defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP >= 1)
#  include <xmmintrin.h>
#  include <pmmintrin.h>
static inline void ringMixerEnableDenormalFlush()
{
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
}
#else
static inline void ringMixerEnableDenormalFlush() {}
#endif

// Save/restore variant for the start() prefill: topUp() renders the whole
// node graph on the CALLING thread (before the mixer thread exists), and
// FTZ/DAZ is per-thread state — enable it for the prefill, then put the
// caller's FP environment back exactly as found (the main thread must not
// silently inherit flush-to-zero for the rest of the session).
#if defined(__aarch64__) || defined(_M_ARM64)
static inline uint64_t ringMixerSaveFpEnv()
{
    uint64_t fpcr;
    __asm__ __volatile__("mrs %0, fpcr" : "=r"(fpcr));
    return fpcr;
}
static inline void ringMixerRestoreFpEnv(uint64_t fpcr)
{
    __asm__ __volatile__("msr fpcr, %0" :: "r"(fpcr));
}
#elif defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP >= 1)
static inline uint64_t ringMixerSaveFpEnv()
{
    return (static_cast<uint64_t>(_MM_GET_FLUSH_ZERO_MODE()) << 32)
         | static_cast<uint64_t>(_MM_GET_DENORMALS_ZERO_MODE());
}
static inline void ringMixerRestoreFpEnv(uint64_t env)
{
    _MM_SET_FLUSH_ZERO_MODE(static_cast<unsigned>(env >> 32));
    _MM_SET_DENORMALS_ZERO_MODE(static_cast<unsigned>(env & 0xFFFFFFFFu));
}
#else
static inline uint64_t ringMixerSaveFpEnv() { return 0; }
static inline void ringMixerRestoreFpEnv(uint64_t) {}
#endif

namespace Darkness {

RingOutputMixer::~RingOutputMixer()
{
    stop();
}

//------------------------------------------------------
bool RingOutputMixer::start(ma_engine* engine, ma_uint32 blockFrames,
                            ma_uint32 channels, ma_uint32 sampleRate,
                            float marginMs)
{
    if (mActive.load(std::memory_order_acquire)) {
        std::fprintf(stderr,
            "[FALLBACK] RingOutputMixer: start() while already active — "
            "second start ignored\n");
        return false;
    }
    if (!engine || blockFrames == 0 || channels == 0 || sampleRate == 0) {
        std::fprintf(stderr,
            "[FALLBACK] RingOutputMixer: invalid start parameters "
            "(engine=%p block=%u ch=%u rate=%u)\n",
            static_cast<void*>(engine), blockFrames, channels, sampleRate);
        return false;
    }

    mEngine      = engine;
    mBlockFrames = blockFrames;
    mChannels    = channels;
    mSampleRate  = sampleRate;

    // Fill target. Floor of one block: the top-up loop's quantum is a whole
    // engine block, so a sub-block margin would still oscillate in [margin,
    // margin+block) — a floor keeps the accounting honest about that.
    ma_uint32 marginFrames = static_cast<ma_uint32>(
        (static_cast<double>(marginMs) * sampleRate) / 1000.0 + 0.5);
    if (marginFrames < blockFrames)
        marginFrames = blockFrames;
    mMarginFrames = marginFrames;

    // Capacity: fill target + one block of top-up overshoot + two blocks of
    // slack so the producer never finds the ring too full for a whole block
    // in steady state (the top-up loop stops as soon as fill ≥ margin).
    const ma_uint32 capacityFrames = mMarginFrames + 3 * mBlockFrames;

    ma_result rr = ma_pcm_rb_init(ma_format_f32, channels, capacityFrames,
                                  nullptr, nullptr, &mRb);
    if (rr != MA_SUCCESS) {
        std::fprintf(stderr,
            "[FALLBACK] RingOutputMixer: ma_pcm_rb_init(%u frames) failed "
            "(error %d) — ring mixer unavailable\n", capacityFrames, rr);
        return false;
    }
    mRbInit = true;

    mStaging.assign(static_cast<size_t>(mBlockFrames) * mChannels, 0.0f);

    // Reset metrics for this activation.
    mLowWatermark.store(kNoReads, std::memory_order_relaxed);
    mUnderrunEventsWindow.store(0, std::memory_order_relaxed);
    mUnderrunFramesWindow.store(0, std::memory_order_relaxed);
    mUnderrunEventsTotal.store(0, std::memory_order_relaxed);
    mLastXrunLogNs.store(0, std::memory_order_relaxed);

    // mRun gates the PRODUCER (topUp) as a whole, not just the thread loop —
    // set before the prefill so the same code path serves both.
    mRun.store(true, std::memory_order_release);

    // Prefill to the margin on the calling thread — the mixer thread does
    // not exist yet and the device has not started, so this thread is
    // (briefly) the single producer and the engine's only reader. The first
    // device callback then finds a full ring even if the OS is slow to
    // schedule the new thread.
    //
    // FTZ around the prefill: only threadMain() enables denormal flushing
    // and it is per-thread state, so without this the prefill blocks render
    // with denormals enabled — on a restart path with decaying IIR/
    // convolution tails that is the 10-100x microcode penalty landing on
    // the main thread at the exact moment we are racing to fill the ring.
    // The caller's FP environment is restored afterwards.
    {
        const uint64_t savedFpEnv = ringMixerSaveFpEnv();
        ringMixerEnableDenormalFlush();
        topUp();
        ringMixerRestoreFpEnv(savedFpEnv);
    }

    try {
        mThread = std::thread(&RingOutputMixer::threadMain, this);
    } catch (const std::exception& e) {
        std::fprintf(stderr,
            "[FALLBACK] RingOutputMixer: mixer thread spawn failed (%s) — "
            "ring mixer unavailable\n", e.what());
        mRun.store(false, std::memory_order_release);
        ma_pcm_rb_uninit(&mRb);
        mRbInit = false;
        return false;
    }

    mActive.store(true, std::memory_order_release);
    return true;
}

//------------------------------------------------------
void RingOutputMixer::stop()
{
    // Idempotent: joinable() covers both "never started" and "already
    // stopped". The caller (AudioService::shutdownMiniaudio) guarantees the
    // device thread has left deviceRead() before this runs.
    mRun.store(false, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lk(mWakeMutex);
        // Lock held so the wake can't slip between the thread's predicate
        // check and its wait — standard CV shutdown handshake.
    }
    mWakeCv.notify_all();
    if (mThread.joinable())
        mThread.join();
    if (mRbInit) {
        ma_pcm_rb_uninit(&mRb);
        mRbInit = false;
    }
    mActive.store(false, std::memory_order_release);
    mEngine = nullptr;
}

//------------------------------------------------------
void RingOutputMixer::deviceRead(float* out, ma_uint32 frameCount)
{
    // RT path: two ring-pointer atomics, a memcpy, and (only on the failure
    // path) counter bumps + a rate-limited stderr line. Never blocks.
    const ma_uint32 fill = ma_pcm_rb_available_read(&mRb);

    // Low-watermark (CAS-min, relaxed — single writer, main-thread reader).
    uint32_t prevMin = mLowWatermark.load(std::memory_order_relaxed);
    while (fill < prevMin
           && !mLowWatermark.compare_exchange_weak(
                  prevMin, fill, std::memory_order_relaxed)) {}

    ma_uint32 remaining = (frameCount < fill) ? frameCount : fill;
    const ma_uint32 got = remaining;
    float* dst = out;
    while (remaining > 0) {
        ma_uint32 grab = remaining;
        void* src = nullptr;
        // acquire_read can return fewer frames than requested when the read
        // region wraps the ring end — loop until done.
        if (ma_pcm_rb_acquire_read(&mRb, &grab, &src) != MA_SUCCESS
            || grab == 0) {
            break;
        }
        std::memcpy(dst, src,
                    static_cast<size_t>(grab) * mChannels * sizeof(float));
        ma_pcm_rb_commit_read(&mRb, grab);
        dst += static_cast<size_t>(grab) * mChannels;
        remaining -= grab;
    }
    const ma_uint32 copied = got - remaining;

    if (copied < frameCount) {
        // Underrun: the mixer thread fell behind. Fill the shortfall with
        // silence (the device must receive a full buffer — noPreSilenced
        // output) and report LOUDLY, rate-limited to 1 line/s so a sustained
        // starve can't flood stderr from the RT thread.
        const ma_uint32 deficit = frameCount - copied;
        std::memset(out + static_cast<size_t>(copied) * mChannels, 0,
                    static_cast<size_t>(deficit) * mChannels * sizeof(float));
        mUnderrunEventsWindow.fetch_add(1, std::memory_order_relaxed);
        mUnderrunFramesWindow.fetch_add(deficit, std::memory_order_relaxed);
        const uint32_t total =
            mUnderrunEventsTotal.fetch_add(1, std::memory_order_relaxed) + 1;

        const long long nowNs =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
        long long last = mLastXrunLogNs.load(std::memory_order_relaxed);
        if (nowNs - last > 1000000000LL
            && mLastXrunLogNs.compare_exchange_strong(
                   last, nowNs, std::memory_order_relaxed)) {
            std::fprintf(stderr,
                "[XRUN] ring underrun: needed %u, had %u (#%u)\n",
                frameCount, fill, total);
        }
    }
}

//------------------------------------------------------
RingOutputMixer::WindowStats RingOutputMixer::drainWindowStats()
{
    WindowStats s;
    s.lowWatermarkFrames =
        mLowWatermark.exchange(kNoReads, std::memory_order_relaxed);
    s.underrunEvents =
        mUnderrunEventsWindow.exchange(0, std::memory_order_relaxed);
    s.underrunFrames =
        mUnderrunFramesWindow.exchange(0, std::memory_order_relaxed);
    return s;
}

//------------------------------------------------------
void RingOutputMixer::topUp()
{
    while (mRun.load(std::memory_order_acquire)) {
        if (ma_pcm_rb_available_read(&mRb) >= mMarginFrames)
            break;   // fill target reached
        if (ma_pcm_rb_available_write(&mRb) < mBlockFrames)
            break;   // no room for a whole block (capacity sizing makes this rare)

        // Render one engine block. This is where the ENTIRE mix graph runs:
        // per-voice Steam Audio DSP nodes, the reflection mix node (incl.
        // its convolution-worker sync wait), and the master chain.
        const auto t0 = std::chrono::steady_clock::now();
        ma_engine_read_pcm_frames(mEngine, mStaging.data(), mBlockFrames,
                                  nullptr);
        const auto t1 = std::chrono::steady_clock::now();
        mRenderMs.record(
            std::chrono::duration<double, std::milli>(t1 - t0).count());

        // Publish the block into the ring (wrap-aware acquire/commit loop,
        // same shape as the WAV-capture tap's writer side).
        const float* src = mStaging.data();
        ma_uint32 remaining = mBlockFrames;
        while (remaining > 0) {
            ma_uint32 grab = remaining;
            void* dst = nullptr;
            if (ma_pcm_rb_acquire_write(&mRb, &grab, &dst) != MA_SUCCESS
                || grab == 0) {
                // Cannot happen while the free-space precheck above holds
                // (single producer); loud if the invariant ever breaks.
                std::fprintf(stderr,
                    "[FALLBACK] RingOutputMixer: ring write refused with "
                    "%u frames pending — dropping rendered audio\n",
                    remaining);
                break;
            }
            std::memcpy(dst, src,
                        static_cast<size_t>(grab) * mChannels * sizeof(float));
            ma_pcm_rb_commit_write(&mRb, grab);
            src += static_cast<size_t>(grab) * mChannels;
            remaining -= grab;
        }
    }
}

//------------------------------------------------------
void RingOutputMixer::threadMain()
{
    ringMixerEnableDenormalFlush();

    // Promote to USER_INTERACTIVE on macOS — the same QoS CoreAudio gives
    // its own IO thread and the same class the convolution sub-workers use
    // (see ConvolutionWorkerPool.cpp): the render deadline here is
    // user-perceptible (ring margin), so the scheduler must treat this
    // thread as latency-critical (P-core preference, low wake jitter).
    {
        // LOG_INFO (not AUDIO_LOG): this thread starts inside
        // bootstrapFinished, before --audio-log has flipped
        // gAudioLogVerbose, and the one-shot QoS confirmation must always
        // be capturable (the conv workers' [WORKER_QOS] lines start late
        // enough that AUDIO_LOG works for them).
        const char* threadName = "ring-mixer";
#if defined(__APPLE__)
        pthread_setname_np(threadName);
        int qosSetRc = pthread_set_qos_class_self_np(
            QOS_CLASS_USER_INTERACTIVE, 0);
        qos_class_t qos = QOS_CLASS_UNSPECIFIED;
        int relPri = 0;
        if (pthread_get_qos_class_np(pthread_self(), &qos, &relPri) == 0) {
            LOG_INFO("[MIXER_QOS] name=%s qos=%u rel=%d setRc=%d",
                     threadName, static_cast<unsigned>(qos), relPri,
                     qosSetRc);
        } else {
            LOG_INFO("[MIXER_QOS] name=%s qos=query-failed setRc=%d",
                     threadName, qosSetRc);
        }
#elif defined(__linux__)
        // No safe QoS equivalent on Linux (SCHED_FIFO requires
        // CAP_SYS_NICE); rely on the short poll cadence + CFS.
        pthread_setname_np(pthread_self(), threadName);
        LOG_INFO("[MIXER_QOS] name=%s startCpu=%d schedPolicy=%d",
                 threadName, sched_getcpu(), sched_getscheduler(0));
#else
        LOG_INFO("[MIXER_QOS] name=%s (no QoS control on this platform)",
                 threadName);
#endif
    }

    // Pump loop: top the ring up to the margin, then nap briefly. The 2 ms
    // poll bounds how stale the fill check can get between wakes; with the
    // default margin (two engine blocks) the ring tolerates a render peak
    // plus several missed polls before the device would see a shortfall.
    // A condvar timed wait (not plain sleep) so stop() can interrupt the
    // nap immediately.
    while (mRun.load(std::memory_order_acquire)) {
        topUp();
        std::unique_lock<std::mutex> lk(mWakeMutex);
        mWakeCv.wait_for(lk, std::chrono::milliseconds(2), [this] {
            return !mRun.load(std::memory_order_acquire);
        });
    }
}

} // namespace Darkness
