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

#ifndef __SUBSOURCESLOTS_H
#define __SUBSOURCESLOTS_H

// Main-thread helpers for the multi-path ambisonics sub-source state
// machine. Each voice carries an array<SubSource, kMaxSubSources> in its
// SteamAudioDSPNode (see VoicePool.h). This header provides the pure-data
// slot-assignment logic that keeps slot identity stable across BFS
// reorders (keyed by SoundPathRecord::predecessorRoomID) and runs the
// FREE/ACTIVE/DRAINING state machine that prevents per-slot DSP-state
// smearing — without this the binaural effect's HRTF interpolation and
// the per-slot door LPF would carry state across physically different
// paths whenever the BFS sort order swapped between frames.
//
// The free functions are templated on a direction-computation callable
// so the helper is decoupled from listener-pose / IPL-frame math; tests
// pass an identity lambda, production passes a closure over the
// listener's right/up/ahead vectors. Audio thread is not involved here
// — these helpers run in main-thread loopStep only.

#include "VoicePool.h"               // SubSource, SubSourceState, kMaxSubSources
#include "room/RoomService.h"        // SoundPropInfo, SoundPathRecord

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace Darkness {

/// Threshold below which a DRAINING slot's currentGain is considered
/// fully decayed and the slot is reclaimed to FREE. Chosen well below
/// audibility (≈ -80 dB) so a slot waiting to be reused doesn't carry
/// residual envelope into its next assignment.
inline constexpr float kSubSourceDrainEpsilon = 1.0e-4f;

/// Returns the index of an ACTIVE/DRAINING slot whose predecessorRoomID
/// matches `pred`, or -1 if no such slot exists. Walks the small fixed
/// array linearly — kMaxSubSources is tiny enough that anything fancier
/// would be slower.
inline int findSlotByPredecessor(
    const std::array<SubSource, kMaxSubSources>& slots, int32_t pred)
{
    for (int i = 0; i < static_cast<int>(slots.size()); ++i) {
        if (slots[i].state != SubSourceState::Free
            && slots[i].predecessorRoomID == pred) {
            return i;
        }
    }
    return -1;
}

/// Returns the index of any FREE slot, or -1 if all slots are occupied.
inline int findFreeSlot(const std::array<SubSource, kMaxSubSources>& slots)
{
    for (int i = 0; i < static_cast<int>(slots.size()); ++i) {
        if (slots[i].state == SubSourceState::Free) return i;
    }
    return -1;
}

/// Returns the index of the ACTIVE slot with the lowest currentGain —
/// the LRU-by-energy victim for capacity-overflow eviction. Returns -1
/// when no ACTIVE slot exists (in which case DRAINING slots are still
/// in their drain window and the caller should let the next frame
/// finish them off before assigning the new path).
///
/// We intentionally do NOT consider DRAINING slots as eviction
/// candidates — they're already on their way out and double-assigning
/// a draining slot would interrupt its tail flush, defeating the
/// click-prevention design.
inline int findLowestEnergyActiveSlot(
    const std::array<SubSource, kMaxSubSources>& slots)
{
    int   best = -1;
    float bestGain = std::numeric_limits<float>::infinity();
    for (int i = 0; i < static_cast<int>(slots.size()); ++i) {
        if (slots[i].state != SubSourceState::Active) continue;
        if (slots[i].currentGain < bestGain) {
            bestGain = slots[i].currentGain;
            best     = i;
        }
    }
    return best;
}

/// Per-frame slot maintenance. Drives the FREE / ACTIVE / DRAINING
/// state machine for one voice from this frame's propagation result.
///
/// Behaviour summary (matches PLAN.MULTI_PATH_AMBISONICS.md):
///   1. Each path in `prop.paths[0..maxN)` finds or claims a slot
///      keyed by predecessorRoomID so per-slot DSP state stays
///      attached to the physical path it represents across BFS
///      reorders.
///   2. New paths take a FREE slot, snap currentDir/currentDoorAlpha
///      to their targets (so the binaural bilinear-interp prev/curr
///      keys match on the cold callback), and let currentGain ramp
///      up from 0 — the resulting fade-in over the audio-thread ramp
///      window prevents clicks on slot allocation.
///   3. Capacity overflow evicts the lowest-energy ACTIVE slot via
///      transition to DRAINING; the new path waits one frame for the
///      drain to complete.
///   4. ACTIVE slots not present in this frame's paths transition to
///      DRAINING; their LPF tail flushes naturally as currentGain
///      ramps down (the audio thread keeps running DSP on draining
///      slots).
///   5. DRAINING slots whose currentGain has decayed below
///      kSubSourceDrainEpsilon are reaped back to FREE.
///
/// The `computeDir` callable converts a path's virtualPosition into
/// an IPLVector3 in whichever frame the audio thread expects (IPL
/// listener-local for production; identity / arbitrary for tests).
/// Signature: `IPLVector3 computeDir(const SoundPathRecord& p)`.
template <class ComputeDirFn>
inline void updateSubSourceSlots(
    std::array<SubSource, kMaxSubSources>& slots,
    const SoundPropInfo&                   prop,
    int                                    maxN,
    ComputeDirFn&&                         computeDir)
{
    const int slotCount = static_cast<int>(slots.size());
    if (maxN < 0)         maxN = 0;
    if (maxN > slotCount) maxN = slotCount;

    // Step 1 — reap any DRAINING slot whose tail has finished flushing
    // BEFORE we try to allocate new paths. This frees up capacity that
    // would otherwise force an unnecessary eviction.
    //
    // Reset audio-thread ramped state at the same time so the next
    // cold assignment to this slot starts from a clean slate (current
    // values match what a fresh slot would see). The audio thread is
    // not concurrently writing currentGain etc. at this point because
    // the slot has just drained to silence — once we mark it FREE the
    // audio thread skips it entirely on subsequent callbacks.
    for (auto& s : slots) {
        if (s.state == SubSourceState::Draining
            && std::fabs(s.currentGain) < kSubSourceDrainEpsilon) {
            s.state              = SubSourceState::Free;
            s.predecessorRoomID  = -1;
            s.currentGain        = 0.0f;
            s.currentDoorAlpha   = 1.0f;
            s.lpfStateL          = 0.0f;
            s.lpfStateR          = 0.0f;
        }
    }

    // Step 2 — assign each propagation path to a slot. Track which
    // slots got touched so unmatched ACTIVE slots can be marked
    // DRAINING below.
    std::array<bool, kMaxSubSources> seen{};

    const int pathN = std::min<int>(static_cast<int>(prop.paths.size()), maxN);
    for (int i = 0; i < pathN; ++i) {
        const SoundPathRecord& p = prop.paths[i];

        // 2a. Reuse the slot already representing this physical path,
        //     if any. This is the entire reason slots are keyed by
        //     predecessorRoomID: it makes per-slot DSP state (HRTF
        //     interp, occlusion smoothing, LPF memory) stick with the
        //     path it belongs to across BFS sort-order swaps.
        int slot = findSlotByPredecessor(slots, p.predecessorRoomID);

        // 2b. Otherwise take a FREE slot.
        if (slot < 0) slot = findFreeSlot(slots);

        // 2c. Otherwise evict the lowest-energy ACTIVE slot via
        //     transition to DRAINING. The new path does NOT take the
        //     slot this frame — it waits for the drain to finish so
        //     the outgoing tenant can flush its tail without an
        //     abrupt cutoff (which would click). The next frame's
        //     reap pass (Step 1) will free the slot and the path
        //     will land naturally.
        if (slot < 0) {
            const int victim = findLowestEnergyActiveSlot(slots);
            if (victim >= 0) {
                slots[victim].targetGain = 0.0f;
                slots[victim].state      = SubSourceState::Draining;
            }
            // Skip this path this frame; no slot to write into.
            continue;
        }

        SubSource& s    = slots[slot];
        const bool cold = (s.state == SubSourceState::Free);

        // 2d. Update targets.
        //
        //  • targetDir — the per-path direction the audio thread will
        //    pass to the binaural HRTF.
        //  • targetGain — per-path portal-attenuation factor. Steam
        //    Audio's iplSimulatorRunDirect places this slot's source at
        //    the path's virtualPosition (doorway anchor for cross-room,
        //    sourcePos for same-room) and computes inv-d² at THAT
        //    distance — which for cross-room is much shorter than the
        //    real path length (because the portal anchor is close to
        //    the listener). The per-path (realDist/effDist)² factor
        //    compensates so the final amplitude reflects the actual
        //    portal-graph path length (with door + LoudRoom
        //    inflation). Same-room paths have realDist == effDist so
        //    targetGain collapses to 1.0 and no compensation is
        //    applied. Multi-path voices sum per-slot signals coherently
        //    — there is no normalization (the chain-energy power sum
        //    in PLAN.SOUND_PROPAGATION.md §3 falls out by
        //    construction).
        //  • targetDoorBlocking — raw [0,1] door blocking; the audio
        //    thread converts to a 1-pole-IIR LPF α per callback.
        float pathGain = 1.0f;
        if (p.effectiveDistance > 0.001f) {
            float ratio = p.realDistance / p.effectiveDistance;
            if      (ratio < 0.0f) ratio = 0.0f;
            else if (ratio > 1.0f) ratio = 1.0f;
            pathGain = ratio * ratio;
        }
        s.targetDir          = computeDir(p);
        s.targetGain         = pathGain;
        s.targetDoorBlocking = p.doorBlocking;

        if (cold) {
            // Snap current = target direction so the binaural effect's
            // bilinear interpolation has matching prev/curr direction
            // keys on its first callback after allocation. currentGain
            // intentionally stays 0 — the ramp from silence up to
            // targetGain is what prevents the click on slot allocation.
            //
            // currentDoorAlpha starts at 1.0 (passthrough) so the slot
            // begins with no spectral coloration and the audio thread
            // ramps it toward whatever the actual door alpha is over
            // ~10 ms. lpfStateL/R are zeroed so no IIR memory carries
            // across from the previous tenant of this slot.
            s.currentDir       = s.targetDir;
            s.currentDoorAlpha = 1.0f;
            s.lpfStateL        = 0.0f;
            s.lpfStateR        = 0.0f;
        }

        s.predecessorRoomID = p.predecessorRoomID;
        s.state             = SubSourceState::Active;
        seen[slot]          = true;
    }

    // Step 3 — any ACTIVE slot not seen this frame loses its path;
    // transition to DRAINING. The audio thread keeps running DSP on
    // it until currentGain decays past the epsilon, at which point
    // Step 1 (next frame) reaps it.
    for (int i = 0; i < slotCount; ++i) {
        SubSource& s = slots[i];
        if (s.state == SubSourceState::Active && !seen[i]) {
            s.targetGain = 0.0f;
            s.state      = SubSourceState::Draining;
        }
    }
}

/// Special-case: the voice has no reached paths at all this frame
/// (prop.reached == false, or prop.paths empty). All ACTIVE slots
/// transition to DRAINING; DRAINING slots continue draining. Used for
/// the unreached / disconnected-cross-room regime where the entire
/// voice is being silenced.
inline void drainAllSubSourceSlots(
    std::array<SubSource, kMaxSubSources>& slots)
{
    for (auto& s : slots) {
        if (s.state == SubSourceState::Active) {
            s.targetGain = 0.0f;
            s.state      = SubSourceState::Draining;
        }
        if (s.state == SubSourceState::Draining
            && std::fabs(s.currentGain) < kSubSourceDrainEpsilon) {
            s.state             = SubSourceState::Free;
            s.predecessorRoomID = -1;
        }
    }
}

} // namespace Darkness

#endif // __SUBSOURCESLOTS_H
