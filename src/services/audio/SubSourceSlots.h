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
// machine. Each voice carries array<SubSource, kMaxSubSources> in its
// SteamAudioDSPNode. Slots are keyed by SoundPathRecord::predecessorRoomID
// so per-slot DSP state (HRTF interp, LPF memory) stays attached to the
// physical path it represents across BFS reorders. FREE/ACTIVE/DRAINING
// state machine prevents DSP-state smearing across different paths.
//
// Templated on a direction-computation callable so tests can pass an
// identity lambda. Main-thread only — audio thread is not involved.

#include "AudioUnits.h"              // kDistanceEpsilonFt
#include "VoicePool.h"               // SubSource, SubSourceState, kMaxSubSources
#include "room/RoomService.h"        // SoundPropInfo, SoundPathRecord

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace Darkness {

/// Drain threshold (≈ -80 dB) — well below audibility so reclaimed slots
/// don't carry residual envelope into their next assignment.
inline constexpr float kSubSourceDrainEpsilon = 1.0e-4f;

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

inline int findFreeSlot(const std::array<SubSource, kMaxSubSources>& slots)
{
    for (int i = 0; i < static_cast<int>(slots.size()); ++i) {
        if (slots[i].state == SubSourceState::Free) return i;
    }
    return -1;
}

/// Lowest-currentGain ACTIVE slot (LRU-by-energy victim). DRAINING slots
/// are skipped — evicting them would interrupt the tail flush.
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

/// Per-frame slot maintenance. Drives FREE/ACTIVE/DRAINING for one voice.
/// `computeDir(SoundPathRecord)` returns the IPLVector3 in whichever
/// frame the audio thread expects (listener-local in production).
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

    // Reap fully-drained slots first so they're available for new paths
    // this frame (otherwise the eviction path takes a slot unnecessarily).
    // Audio thread isn't writing here — drained slots have already gone
    // silent and the next callback will skip them once marked FREE.
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

    std::array<bool, kMaxSubSources> seen{};

    const int pathN = std::min<int>(static_cast<int>(prop.paths.size()), maxN);
    for (int i = 0; i < pathN; ++i) {
        const SoundPathRecord& p = prop.paths[i];

        // Prefer reusing the slot already representing this physical path
        // so DSP state sticks across BFS sort-order swaps.
        int slot = findSlotByPredecessor(slots, p.predecessorRoomID);
        if (slot < 0) slot = findFreeSlot(slots);

        // Capacity overflow: evict lowest-energy ACTIVE → DRAINING. The
        // new path waits one frame for the drain to finish so the
        // outgoing tenant can flush its tail without clicking.
        if (slot < 0) {
            const int victim = findLowestEnergyActiveSlot(slots);
            if (victim >= 0) {
                slots[victim].targetGain = 0.0f;
                slots[victim].state      = SubSourceState::Draining;
            }
            continue;
        }

        SubSource& s    = slots[slot];
        const bool cold = (s.state == SubSourceState::Free);

        // Per-path gain compensates for Steam Audio placing the virtual
        // source at the doorway anchor (cross-room) — the (realDist/
        // effDist)² factor scales back to the actual portal-graph path
        // length. Same-room paths collapse to 1.0.
        float pathGain = 1.0f;
        if (p.effectiveDistance > kDistanceEpsilonFt) {
            float ratio = p.realDistance / p.effectiveDistance;
            if      (ratio < 0.0f) ratio = 0.0f;
            else if (ratio > 1.0f) ratio = 1.0f;
            pathGain = ratio * ratio;
        }
        s.targetDir          = computeDir(p);
        s.targetGain         = pathGain;
        s.targetDoorBlocking = p.doorBlocking;

        if (cold) {
            // Snap current=target dir so the binaural bilinear interp
            // has matching prev/curr keys on the first callback. Leave
            // currentGain=0 so the ramp from silence prevents clicks.
            s.currentDir       = s.targetDir;
            s.currentDoorAlpha = 1.0f;
            s.lpfStateL        = 0.0f;
            s.lpfStateR        = 0.0f;
        }

        s.predecessorRoomID = p.predecessorRoomID;
        s.state             = SubSourceState::Active;
        seen[slot]          = true;
    }

    // ACTIVE slots not present this frame transition to DRAINING. Audio
    // thread keeps running DSP on them until currentGain decays past
    // epsilon, at which point next frame's reap reclaims them.
    for (int i = 0; i < slotCount; ++i) {
        SubSource& s = slots[i];
        if (s.state == SubSourceState::Active && !seen[i]) {
            s.targetGain = 0.0f;
            s.state      = SubSourceState::Draining;
        }
    }
}

/// Drain all slots — used when the voice has no reached paths this frame.
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
