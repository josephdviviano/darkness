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

#ifndef __RING_OUTPUT_MIXER_H
#define __RING_OUTPUT_MIXER_H

/// @file RingOutputMixer.h
/// Ring-fed mixer thread that decouples mix-graph rendering from the audio
/// device callback (PLAN.AUDIO_PERF.md PR D).
///
/// WHY: the miniaudio node graph (per-voice Steam Audio DSP, reflection wet
/// bus, master chain) used to render synchronously inside the device data
/// callback. The engine budgets against the negotiated callback period
/// (frame_size / sample_rate, ~21.3 ms at 1024 @ 48 kHz) but the HAL's real
/// IO cycle can be shorter (measured ~10.67 ms on CoreAudio devices whose
/// internal rate is 96 kHz), so steady-state render peaks of 7-13 ms sat at
/// 50-120 % of the TRUE budget and the device verifiably starved at 14-17
/// voices (66-90 % of frames delivered = audible crackle). This class moves
/// the render onto a dedicated high-QoS thread that keeps a lock-free SPSC
/// ring topped up to a configurable margin; the device callback becomes a
/// bounded-time ring drain (memcpy + silence-fill on shortfall).
///
/// The ring is miniaudio's `ma_pcm_rb` — the same lock-free single-producer /
/// single-consumer PCM ring the WAV-capture tap already uses, just pointed
/// the other way (thread produces, device callback consumes).
///
/// Threading model:
///   • MIXER THREAD (`threadMain`, owned here, USER_INTERACTIVE QoS on
///     macOS): the ONLY caller of `ma_engine_read_pcm_frames` while active.
///     Every node-graph process callback (steamAudioNodeProcess,
///     reflectionMixNodeProcess incl. its convolution-worker doneCv wait,
///     master DSP chain) therefore executes on this thread. It REPLACES the
///     device thread as "the audio thread" for every main↔audio-thread
///     contract in AudioService (dspNode field staging, lock-free atomics,
///     pending-source gates) — still exactly one consumer thread, so those
///     contracts hold unchanged. Producer side of the ring.
///   • DEVICE THREAD (CoreAudio/backend RT thread): calls `deviceRead()`
///     only — ring consumer. Never blocks, never renders, never touches the
///     engine or any DSP state. On shortfall it fills silence, bumps the
///     underrun counters and emits a rate-limited `[XRUN]` stderr line (per
///     feedback_no_silent_fallbacks: never silent, never blocking).
///   • MAIN THREAD: `start()` / `stop()` lifecycle (device stopped or not
///     yet started around both — enforced by AudioService), and the
///     per-window metric drains (`drainWindowStats`, `renderHistogram`).
///     `start()` prefills the ring to the margin from the calling thread
///     BEFORE the mixer thread exists and before the device starts, so the
///     single-producer invariant holds and the first device callbacks find
///     a full ring.
///   • Metric counters are relaxed atomics (single writer each); the
///     histogram is the shared multi-writer-safe LatencyHistogram.

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

#include <miniaudio.h>

#include "LatencyHistogram.h"

namespace Darkness {

class RingOutputMixer {
public:
    /// Sentinel for "no device reads happened this window" in
    /// `WindowStats::lowWatermarkFrames`.
    static constexpr uint32_t kNoReads = 0xFFFFFFFFu;

    RingOutputMixer() = default;
    ~RingOutputMixer();

    // Non-copyable, non-movable (owns a thread + a ring the device reads).
    RingOutputMixer(const RingOutputMixer&) = delete;
    RingOutputMixer& operator=(const RingOutputMixer&) = delete;

    /// Allocate the ring, prefill it to the margin (rendered on the CALLING
    /// thread — the mixer thread does not exist yet), then spawn the mixer
    /// thread. Must be called before the device starts consuming.
    ///   engine       — initialized ma_engine whose graph we pump. Not owned.
    ///   blockFrames  — engine render quantum (must equal the Steam Audio /
    ///                  DSP-node frameSize so per-node cadence is unchanged).
    ///   channels     — interleaved f32 channel count (2 — HRTF is stereo).
    ///   sampleRate   — engine/device callback rate (ring frames are 1:1
    ///                  with device-callback frames).
    ///   marginMs     — ring fill target. The mixer tops the ring back up to
    ///                  at least this much buffered audio each wake.
    /// Returns false (with a loud stderr line) on ring-alloc or thread-spawn
    /// failure; the caller falls back to legacy in-callback rendering.
    bool start(ma_engine* engine, ma_uint32 blockFrames, ma_uint32 channels,
               ma_uint32 sampleRate, float marginMs);

    /// Join the mixer thread and free the ring. The caller must guarantee
    /// the device is no longer inside deviceRead() (AudioService stops the
    /// device and spins its in-flight guard first). Idempotent.
    void stop();

    /// True between a successful start() and stop().
    bool active() const { return mActive.load(std::memory_order_acquire); }

    /// DEVICE-THREAD ONLY. Copy up to `frameCount` frames from the ring into
    /// `out`; silence-fill and count any shortfall. Honors arbitrary
    /// frameCount (backends may aggregate periods). Never blocks.
    void deviceRead(float* out, ma_uint32 frameCount);

    /// Per-perf-window metric drain (main thread). Counters reset to zero /
    /// kNoReads on read so each window is independent.
    struct WindowStats {
        uint32_t lowWatermarkFrames = kNoReads; ///< min ring fill seen at deviceRead entry
        uint32_t underrunEvents     = 0;        ///< deviceRead calls that came up short
        uint64_t underrunFrames     = 0;        ///< total silence frames inserted
    };
    WindowStats drainWindowStats();

    /// Mixer-thread render-time histogram (one record per engine-block
    /// render). Drained by the main-thread periodic dump like every other
    /// audio LatencyHistogram.
    LatencyHistogram& renderHistogram() { return mRenderMs; }

    /// Effective ring fill target in frames / ms (post-clamp), for the init
    /// latency-accounting log.
    ma_uint32 marginFrames() const { return mMarginFrames; }
    float marginMs() const {
        return mSampleRate ? 1000.0f * static_cast<float>(mMarginFrames)
                                 / static_cast<float>(mSampleRate)
                           : 0.0f;
    }

private:
    void threadMain();
    /// Render engine blocks into the ring until fill ≥ margin (or the ring
    /// can't take a whole block). Mixer thread + start()-prefill only.
    void topUp();

    ma_engine*  mEngine      = nullptr;
    ma_pcm_rb   mRb{};
    bool        mRbInit      = false;
    ma_uint32   mBlockFrames = 0;
    ma_uint32   mChannels    = 0;
    ma_uint32   mSampleRate  = 0;
    ma_uint32   mMarginFrames = 0;
    std::vector<float> mStaging;   ///< mixer-thread-only render scratch

    std::thread             mThread;
    std::atomic<bool>       mRun{false};    ///< mixer-thread run gate
    std::atomic<bool>       mActive{false}; ///< start()/stop() lifecycle flag
    std::mutex              mWakeMutex;     ///< pairs with mWakeCv (stop wake)
    std::condition_variable mWakeCv;

    // ── metrics ──
    LatencyHistogram      mRenderMs;              ///< per-block ma_engine_read cost
    std::atomic<uint32_t> mLowWatermark{kNoReads};///< device thread writes (CAS min)
    std::atomic<uint32_t> mUnderrunEventsWindow{0};
    std::atomic<uint64_t> mUnderrunFramesWindow{0};
    std::atomic<uint32_t> mUnderrunEventsTotal{0};///< lifetime, for the [XRUN] "#k"
    std::atomic<long long> mLastXrunLogNs{0};     ///< [XRUN] rate limiter (1/s)
};

} // namespace Darkness

#endif // __RING_OUTPUT_MIXER_H
