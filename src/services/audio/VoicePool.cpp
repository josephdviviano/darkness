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

#include "VoicePool.h"
#include "AudioLog.h"

#include <chrono>
#include <thread>

// ConvolutionWorker is opaque to this TU; we only dereference it to wait on
// processedSeq counters. The full definition lives in AudioService.cpp.
// The destructor reaches in via a minimal in-TU declaration matching the
// fields we need. To avoid duplicating that struct shape here, ~ActiveVoice
// is split: the IPL/miniaudio teardown that doesn't touch ConvolutionWorker
// lives below, and the convolution-worker drain is delegated to a hook on
// SteamAudioDSPNode set during initVoiceDSP (the `convWorker` raw pointer
// + a small helper compiled in AudioService.cpp's TU where the struct is
// complete).
//
// For now (keeping behaviour identical to pre-extraction), we keep the
// drain inline by including just enough to see ConvolutionWorker's
// sub-worker fields. That definition is provided here under the same
// namespace; AudioService.cpp's authoritative definition is layout-
// compatible because both share the same header dependencies.
//
// NOTE: this is a deliberate piece of structural friction. The cleaner
// long-term shape is to move ConvolutionWorker into its own header (audio-
// only). That's left for a follow-up — out of scope for this extraction.

namespace Darkness {

// ──────────────────────────────────────────────────────────────────────
// ConvolutionWorker forward usage
//
// We need to drain pending work in ~ActiveVoice. To avoid duplicating the
// full ConvolutionWorker definition here, we provide a small helper that
// is defined inside AudioService.cpp (where ConvolutionWorker is fully
// visible). The helper waits up to `deadlineMs` for every sub-worker's
// processedSeq to catch up to its frameSeq, returning true on clean drain
// and false on timeout (in which case the caller logs and continues).
// ──────────────────────────────────────────────────────────────────────
bool drainConvolutionWorker(ConvolutionWorker *cw, int deadlineMs);

//------------------------------------------------------
ActiveVoice::~ActiveVoice()
{
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
    // correctness (no TOCTOU gap). Implemented in AudioService.cpp where
    // ConvolutionWorker is fully visible.
    if (dspNode.convWorker && dspNode.reflectionEffect) {
        if (!drainConvolutionWorker(dspNode.convWorker, 500)) {
            AUDIO_LOG("[AUDIO] WARNING: ~ActiveVoice worker drain "
                      "timed out — releasing effect anyway\n");
        }
    }

    // Release per-slot Steam Audio effects. Each sub-source slot owns its
    // own direct + binaural pair so the per-slot HRTF interp / direct-
    // effect smoothing state stayed attached to the physical propagation
    // path mapped to that slot during the voice's lifetime. The handles
    // are pre-allocated in initVoiceDSP (all-or-nothing for the voice) so
    // either every slot has them or none does.
    for (auto &slot : dspNode.subSources) {
        if (slot.binauralEffect) {
            iplBinauralEffectRelease(&slot.binauralEffect);
            slot.binauralEffect = nullptr;
        }
        if (slot.directEffect) {
            iplDirectEffectRelease(&slot.directEffect);
            slot.directEffect = nullptr;
        }
    }
    if (dspNode.reflectionEffect) {
        iplReflectionEffectRelease(&dspNode.reflectionEffect);
        dspNode.reflectionEffect = nullptr;
    }
    // Phase 4: release the per-voice IPLPathEffect symmetric to the
    // reflection effect above. createVoiceSource/initVoiceDSP skipped
    // creation for playerEmitted voices, so this is a no-op for them.
    if (dspNode.pathEffect) {
        iplPathEffectRelease(&dspNode.pathEffect);
        dspNode.pathEffect = nullptr;
    }

    // Release the audio-thread mirror of the pathing IPLSource. The
    // audio thread reads dspNode.pathingSource inside its path-effect
    // block via iplSourceGetOutputs; releasing it here AFTER the
    // ma_node_uninit drain above is the linchpin of the deferred-
    // release safety pattern set up by AudioService::removeVoiceSource
    // — that function nulls voice.pathingSource without iplSourceRelease
    // and leaves dspNode.pathingSource as the sole live reference. The
    // queue-source-remove branch nulls dspNode.pathingSource so this is
    // a no-op for the deferred-flush case (flushPendingRemovals does
    // the release on its own copy after the audio thread is drained).
    if (dspNode.pathingSource) {
        iplSourceRelease(&dspNode.pathingSource);
    }

    // Destroy sound and decoder
    if (initialized) {
        ma_sound_uninit(&sound);
        ma_decoder_uninit(&decoder);
    }

    // Release IPLSources (removeVoiceSource should have been called, but be safe).
    // Per-slot direct sources mirror the voice-level directSource handling —
    // they're added to the same mDirectSimulator and removed in
    // removeVoiceSource, but if that hook wasn't run we at least release
    // the handles here so the IPL refcount drops to zero.
    if (directSource) {
        iplSourceRelease(&directSource);
    }
    for (auto &slot : dspNode.subSources) {
        if (slot.directSource) {
            iplSourceRelease(&slot.directSource);
        }
    }
    if (reflectionSource) {
        iplSourceRelease(&reflectionSource);
    }
}

//------------------------------------------------------
bool VoicePool::remove(SoundHandle h, CleanupHook hook)
{
    auto it = mVoices.find(h);
    if (it == mVoices.end())
        return false;
    if (hook)
        hook(*it->second);
    mVoices.erase(it);
    return true;
}

//------------------------------------------------------
void VoicePool::clear(CleanupHook hook)
{
    if (hook) {
        for (auto &kv : mVoices) {
            hook(*kv.second);
        }
    }
    mVoices.clear();
}

//------------------------------------------------------
std::size_t VoicePool::cleanupFinished(CleanupHook hook)
{
    std::size_t removed = 0;
    // Only check the atomic flag (set by the audio thread's end callback).
    // This avoids cross-thread calls to ma_sound_at_end().
    for (auto it = mVoices.begin(); it != mVoices.end();) {
        if (it->second->finished.load(std::memory_order_acquire)) {
            if (hook)
                hook(*it->second);
            it = mVoices.erase(it);
            ++removed;
        } else {
            ++it;
        }
    }
    return removed;
}

} // namespace Darkness
