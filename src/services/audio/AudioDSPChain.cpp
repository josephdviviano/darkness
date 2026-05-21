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

#include "AudioDSPChain.h"

#include <algorithm>
#include <atomic>

namespace Darkness {

// File-scope atomics that the audio thread reads. Defined in AudioService.cpp
// (option (a) from the extraction brief — keeping them in the translation
// unit that holds the audio callback). We publish to them via extern
// declarations so AudioDSPChain doesn't have to know about the audio
// backend.
extern std::atomic<int>   sHrtfInterpolation;
extern std::atomic<float> sSpatialBlend;
extern std::atomic<float> sDoorLpfOpenHz;
extern std::atomic<float> sDoorLpfBlockedHz;
extern std::atomic<float> sPropMinAttenuation;

// ── Master bus DSP chain config ──

void AudioDSPChain::setDSPLimiterKnee(float k)
{
    mDSPLimiterKnee = std::max(0.5f, std::min(k, 0.95f));
}

void AudioDSPChain::setDSPWetSaturationDrive(float d)
{
    // Drive range: 1.0 = effectively transparent soft brick-wall at ±1.0;
    // 2-4 = audible tape warmth; 5-10 = heavy phonograph character.
    // Above 10 the effect is so colored it stops sounding like reverb at all.
    mDSPWetSaturationDrive = std::max(1.0f, std::min(d, 10.0f));
}

void AudioDSPChain::setDSPCompThreshold(float t)
{
    mDSPCompThreshold = std::max(-30.0f, std::min(t, 0.0f));
}

void AudioDSPChain::setDSPCompRatio(float r)
{
    mDSPCompRatio = std::max(1.5f, std::min(r, 10.0f));
}

void AudioDSPChain::setDSPCompAttackMs(float ms)
{
    mDSPCompAttackMs = std::max(1.0f, std::min(ms, 100.0f));
}

void AudioDSPChain::setDSPCompReleaseMs(float ms)
{
    mDSPCompReleaseMs = std::max(50.0f, std::min(ms, 2000.0f));
}

void AudioDSPChain::setDSPEQFreq(float f)
{
    mDSPEQFreq = std::max(60.0f, std::min(f, 500.0f));
}

void AudioDSPChain::setDSPEQGain(float g)
{
    mDSPEQGain = std::max(-6.0f, std::min(g, 6.0f));
}

void AudioDSPChain::setDSPEQQ(float q)
{
    mDSPEQQ = std::max(0.3f, std::min(q, 2.0f));
}

void AudioDSPChain::setDSPDuckAmount(float a)
{
    mDSPDuckAmount = std::max(0.1f, std::min(a, 1.0f));
}

void AudioDSPChain::setDSPDuckAttackMs(float ms)
{
    mDSPDuckAttackMs = std::max(10.0f, std::min(ms, 500.0f));
}

void AudioDSPChain::setDSPDuckReleaseMs(float ms)
{
    mDSPDuckReleaseMs = std::max(50.0f, std::min(ms, 5000.0f));
}

// ── Mixer / global gains ──
//
// Setters here merely clamp + store. The live ReflectionMixNode poke is
// handled by AudioService's facade so the node type stays private to the
// audio TU.

void AudioDSPChain::setMasterGain(float g)
{
    mMasterGain = std::max(0.0f, std::min(g, 4.0f));
}

void AudioDSPChain::setDirectGain(float g)
{
    mDirectGain = std::max(0.0f, std::min(g, 4.0f));
}

void AudioDSPChain::setReflectionGain(float g)
{
    mReflectionGain = std::max(0.0f, std::min(g, 4.0f));
}

void AudioDSPChain::setReflectionRampMs(float ms)
{
    mReflectionRampMs = std::max(1.0f, std::min(ms, 1000.0f));
}

// ── Spatialization ──

void AudioDSPChain::setHRTFVolume(float v)
{
    mHRTFVolume = std::max(0.0f, std::min(v, 4.0f));
}

void AudioDSPChain::setHRTFInterpolation(const std::string& s)
{
    mHRTFInterpolation = (s == "nearest" ? "nearest" : "bilinear");
}

void AudioDSPChain::setSpatialBlend(float b)
{
    mSpatialBlend = std::max(0.0f, std::min(b, 1.0f));
}

// ── Propagation tuning ──

void AudioDSPChain::setDoorLpfOpenHz(float hz)
{
    // Widened lower bound: was 1000 Hz; now 200 Hz to allow very-muffled
    // "open" doors for stylized environments and tests.
    mDoorLpfOpenHz = std::max(200.0f, std::min(hz, 24000.0f));
    publishAudioThreadParams();
}

void AudioDSPChain::setDoorLpfBlockedHz(float hz)
{
    mDoorLpfBlockedHz = std::max(100.0f, std::min(hz, 10000.0f));
    publishAudioThreadParams();
}

void AudioDSPChain::setPropMinAttenuation(float a)
{
    // Widened upper bound: was 0.1; now 1.0 so callers can effectively
    // disable propagation by saturating the minimum-attenuation floor.
    mPropMinAttenuation = std::max(0.0f, std::min(a, 1.0f));
    publishAudioThreadParams();
}

// ── Audio-thread parameter publish ──

void AudioDSPChain::publishAudioThreadParams() const
{
    sHrtfInterpolation.store(mHRTFInterpolation == "nearest" ? 0 : 1,
                             std::memory_order_relaxed);
    sSpatialBlend.store(mSpatialBlend, std::memory_order_relaxed);
    sDoorLpfOpenHz.store(mDoorLpfOpenHz, std::memory_order_relaxed);
    sDoorLpfBlockedHz.store(mDoorLpfBlockedHz, std::memory_order_relaxed);
    sPropMinAttenuation.store(mPropMinAttenuation, std::memory_order_relaxed);
}

} // namespace Darkness
