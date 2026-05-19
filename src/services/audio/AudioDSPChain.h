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

#ifndef __AUDIODSPCHAIN_H
#define __AUDIODSPCHAIN_H

#include <string>

namespace Darkness {

/// Owns the master-bus DSP-chain configuration (soft limiter, compressor,
/// low-shelf EQ, ducker), the global mixer gains (master / direct /
/// reflection), the spatialization knobs (HRTF interp + spatial blend +
/// distance model) and the propagation tuning that the audio thread reads
/// (door LPF cutoffs, minimum attenuation). Extracted from AudioService to
/// keep that god-class focused on lifetime + scene orchestration.
///
/// All setters clamp inputs to the same legal ranges that AudioService used
/// previously; AudioService keeps public facades that forward to this class
/// so call sites (RenderConfig / DebugConsole) need no changes.
///
/// AudioDSPChain does NOT own the live miniaudio ReflectionMixNode — that
/// stays inside AudioService.cpp. AudioService reads these members during
/// initReflectionPipeline() to seed the node and during the live-tunable
/// gain setters (set{Master,Direct,Reflection}Gain) to push updates into
/// the running node.
class AudioDSPChain {
public:
    AudioDSPChain() = default;

    // ── Master bus DSP chain config (applied when reflection pipeline initializes) ──

    /// Configure the soft limiter (prevents digital clipping)
    void setDSPLimiterEnabled(bool v) { mDSPLimiterEnabled = v; }
    bool getDSPLimiterEnabled() const { return mDSPLimiterEnabled; }
    void setDSPLimiterKnee(float k);
    float getDSPLimiterKnee() const { return mDSPLimiterKnee; }

    /// Configure the master bus compressor (tames transients)
    void setDSPCompressorEnabled(bool v) { mDSPCompressorEnabled = v; }
    bool getDSPCompressorEnabled() const { return mDSPCompressorEnabled; }
    void setDSPCompThreshold(float t);
    float getDSPCompThreshold() const { return mDSPCompThreshold; }
    void setDSPCompRatio(float r);
    float getDSPCompRatio() const { return mDSPCompRatio; }
    void setDSPCompAttackMs(float ms);
    float getDSPCompAttackMs() const { return mDSPCompAttackMs; }
    void setDSPCompReleaseMs(float ms);
    float getDSPCompReleaseMs() const { return mDSPCompReleaseMs; }

    /// Configure the low-shelf EQ (bass boost/cut)
    void setDSPEQEnabled(bool v) { mDSPEQEnabled = v; }
    bool getDSPEQEnabled() const { return mDSPEQEnabled; }
    void setDSPEQFreq(float f);
    float getDSPEQFreq() const { return mDSPEQFreq; }
    void setDSPEQGain(float g);
    float getDSPEQGain() const { return mDSPEQGain; }
    void setDSPEQQ(float q);
    float getDSPEQQ() const { return mDSPEQQ; }

    /// Configure the wet-bus tape saturation (analog character on reverb path).
    /// Applies a soft tanh saturator to the summed wet bus before it's mixed
    /// with the dry path. Designed to model analog tape / phonograph
    /// compression — nearly transparent for quiet reverb, harmonic
    /// coloration at moderate levels, brick-wall ceiling at loud peaks.
    /// Doubles as a graceful clip-handler for pathological hot probes
    /// (small enclosed rooms with ambients) without polluting the dry signal.
    void setDSPWetSaturationEnabled(bool v) { mDSPWetSaturationEnabled = v; }
    bool getDSPWetSaturationEnabled() const { return mDSPWetSaturationEnabled; }
    void setDSPWetSaturationDrive(float d);
    float getDSPWetSaturationDrive() const { return mDSPWetSaturationDrive; }

    /// Configure the ambient ducking system (disabled by default)
    void setDSPDuckingEnabled(bool v) { mDSPDuckingEnabled = v; }
    bool getDSPDuckingEnabled() const { return mDSPDuckingEnabled; }
    void setDSPDuckAmount(float a);
    float getDSPDuckAmount() const { return mDSPDuckAmount; }
    void setDSPDuckAttackMs(float ms);
    float getDSPDuckAttackMs() const { return mDSPDuckAttackMs; }
    void setDSPDuckReleaseMs(float ms);
    float getDSPDuckReleaseMs() const { return mDSPDuckReleaseMs; }

    // ── Mixer / global gains ──
    //
    // These setters merely clamp and stash. AudioService's facade poke also
    // pushes the new value into the live ReflectionMixNode so changes take
    // effect on the next audio callback — that side-effect stays in
    // AudioService.cpp since the node type is private to that translation
    // unit.
    void  setMasterGain(float g);
    float getMasterGain() const { return mMasterGain; }
    void  setDirectGain(float g);
    float getDirectGain() const { return mDirectGain; }
    void  setReflectionGain(float g);
    float getReflectionGain() const { return mReflectionGain; }
    void  setReflectionRampMs(float ms);
    float getReflectionRampMs() const { return mReflectionRampMs; }

    // ── Spatialization (HRTF + distance model) ──
    /// Must be set BEFORE bootstrapFinished()/init — used during HRTF creation.
    void setHRTFVolume(float v);
    float getHRTFVolume() const { return mHRTFVolume; }
    /// "nearest" or "bilinear". Must be set BEFORE per-source effects are created.
    void setHRTFInterpolation(const std::string& s);
    const std::string& getHRTFInterpolation() const { return mHRTFInterpolation; }
    void setSpatialBlend(float b);
    float getSpatialBlend() const { return mSpatialBlend; }
    /// "default" or "inverse_distance"
    void setDistanceModel(const std::string& s);
    const std::string& getDistanceModel() const { return mDistanceModel; }

    // ── Propagation tuning that the audio thread reads ──
    //
    // The "live" setters re-publish to the audio-thread atomics in
    // AudioService.cpp so the next audio callback picks up the new value.
    void  setDoorLpfOpenHz(float hz);
    float getDoorLpfOpenHz() const { return mDoorLpfOpenHz; }
    void  setDoorLpfBlockedHz(float hz);
    float getDoorLpfBlockedHz() const { return mDoorLpfBlockedHz; }
    void  setPropMinAttenuation(float a);
    float getPropMinAttenuation() const { return mPropMinAttenuation; }

    /// Republish HRTF interp / spatial blend / door LPF / propagation min /
    /// distance model into the file-scope atomics that the audio thread
    /// reads. Idempotent and cheap; called from setters and after bulk
    /// configuration in RenderConfig apply.
    void publishAudioThreadParams() const;

private:
    // ── Master bus DSP chain config ──
    bool  mDSPLimiterEnabled = true;
    float mDSPLimiterKnee = 0.8f;
    bool  mDSPCompressorEnabled = true;
    float mDSPCompThreshold = -15.0f;
    float mDSPCompRatio = 3.0f;
    float mDSPCompAttackMs = 10.0f;
    float mDSPCompReleaseMs = 250.0f;
    bool  mDSPEQEnabled = true;
    float mDSPEQFreq = 120.0f;
    float mDSPEQGain = 3.0f;
    float mDSPEQQ    = 0.707f;
    bool  mDSPDuckingEnabled = false;
    float mDSPDuckAmount = 0.5f;
    float mDSPDuckAttackMs = 50.0f;
    float mDSPDuckReleaseMs = 500.0f;
    bool  mDSPWetSaturationEnabled = false;
    float mDSPWetSaturationDrive = 1.0f;

    // ── Mixer (global gains) ──
    float mMasterGain        = 1.0f;
    float mDirectGain        = 1.0f;
    float mReflectionGain    = 1.0f;
    float mReflectionRampMs  = 10.0f;

    // ── Spatialization (HRTF + distance attenuation) ──
    float       mHRTFVolume        = 1.0f;
    std::string mHRTFInterpolation = "bilinear"; // "nearest" or "bilinear"
    float       mSpatialBlend      = 1.0f;
    std::string mDistanceModel     = "default";  // "default" or "inverse_distance"

    // ── Propagation tuning (door LPF + minimum attenuation) ──
    float    mDoorLpfOpenHz       = 20000.0f;
    float    mDoorLpfBlockedHz    = 800.0f;
    float    mPropMinAttenuation  = 0.001f;
};

} // namespace Darkness

#endif // __AUDIODSPCHAIN_H
