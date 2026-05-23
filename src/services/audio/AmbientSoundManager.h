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

#ifndef __AMBIENTSOUNDMANAGER_H
#define __AMBIENTSOUNDMANAGER_H

#include "DarknessMath.h"
#include "AudioService.h"  // for SoundHandle, SOUND_HANDLE_INVALID, AmbientHackFlags

#include <cstdint>
#include <string>
#include <vector>

namespace Darkness {

class AudioService;

/// Tracked ambient sound instance (attached to a mission object), parsed
/// from P$AmbientHack. Owned by AmbientSoundManager.
struct AmbientSound {
    int objID = 0;                    ///< Mission object ID
    std::string schemaName;           ///< Schema to play
    Vector3 position{0, 0, 0};        ///< World-space position
    float radius = 0.0f;              ///< Propagation radius
    int32_t volume = -1;              ///< Volume in millibels
    uint32_t flags = 0;               ///< AmbientHackFlags
    /// Schema's SCH_SHARP_FALLOFF flag (parsed from P$SchPlayPa).
    /// Preserved-but-unused since loudness shaping moved to Steam Audio
    /// — kept on the struct in case a future tuning pass maps it onto
    /// Steam Audio rolloff parameters.
    bool isSharp = true;
    /// P$SchAttFac. Preserved-but-unused since the per-radius centibel
    /// falloff curve retired in favour of Steam Audio's distance model
    /// — a future tuning pass will map this onto rolloffFactor.
    float attenuationFactor = 1.0f;
    SoundHandle handle = SOUND_HANDLE_INVALID; ///< Active voice handle (if playing)
    /// Sticky one-way memo: set to true the first frame both startVoice
    /// attempts (schema sample then raw schemaName) return
    /// SOUND_HANDLE_INVALID. Gates the per-frame retry block in
    /// updateAmbientVolumes so a permanently unresolvable ambient does
    /// not re-emit its [FALLBACK] log every host tick. Never cleared —
    /// the missing-sample condition does not change at runtime (CRF
    /// resources are immutable), so we transition once and stay.
    bool resolutionFailed = false;
};

// NOTE: there was previously a `SpotAmbient` struct here, plus a parallel
// loader / per-frame lifecycle, that interpreted P$SpotAmb as a second
// audio-ambient encoding (inner/outer/level). That was a misidentification
// — P$SpotAmb is a renderer "SpotlightAndAmbient" property, not audio. The
// audio path has been removed; the property struct now lives at
// PropSpotlightAndAmbient in DarkPropertyDefs.h, awaiting renderer wiring.
// See TASKS.TODO.md "SpotlightAndAmbient lighting".

/// Manages ambient sound lifecycle: parses P$AmbientHack from mission data,
/// starts/stops voices based on listener distance with hysteresis, and
/// applies a per-voice linear volume each frame. Loudness shaping (distance
/// / portal / occlusion) is delegated to Steam Audio's per-voice DSP chain.
/// Owned by AudioService; constructed in the service's constructor and torn
/// down with it.
class AmbientSoundManager {
public:
    explicit AmbientSoundManager(AudioService *host);
    ~AmbientSoundManager();

    AmbientSoundManager(const AmbientSoundManager &) = delete;
    AmbientSoundManager &operator=(const AmbientSoundManager &) = delete;

    /// Load ambient sound objects (P$AmbientHack) from mission data. The
    /// caller is responsible for ensuring sound resources, the schema
    /// parser, and the property/object services are ready first.
    void loadAmbientSounds();

    /// Per-frame update for P$AmbientHack ambients: starts/stops voices
    /// based on hysteresis around the authored radius and applies the
    /// authored (centibel) volume + duck + global scale once per frame.
    void updateAmbientVolumes();

    /// Drop all parsed ambient state. Voices are NOT halted here — the
    /// caller (AudioService::shutdown / onDBDrop) is expected to have
    /// already called haltAll() before clearing the bookkeeping.
    void clear();

    /// Asymmetric hysteresis multipliers — start >= 1.0, stop > start.
    /// Prevents start/stop oscillation near the radius boundary.
    void setHysteresisStartMul(float m) { mHysteresisStartMul = m; }
    void setHysteresisStopMul(float m)  { mHysteresisStopMul  = m; }
    float getHysteresisStartMul() const { return mHysteresisStartMul; }
    float getHysteresisStopMul() const  { return mHysteresisStopMul; }

    void setDefaultPriority(int p) { mDefaultPriority = p; }
    int  getDefaultPriority() const { return mDefaultPriority; }

    void setEnvironmentalSpatialBlend(float b) { mEnvironmentalSpatialBlend = b; }
    float getEnvironmentalSpatialBlend() const { return mEnvironmentalSpatialBlend; }

    /// Global multiplier applied to every ambient voice's per-frame linear
    /// volume. Compensates for the loudness re-baseline introduced when
    /// Steam Audio became the sole player-audio propagation authority (no
    /// more centibel falloff curve over schema radius). Default 1.0 = no
    /// change; designers tune by ear.
    void setGlobalVolumeScale(float s) { mGlobalVolumeScale = (s < 0.0f ? 0.0f : s); }
    float getGlobalVolumeScale() const { return mGlobalVolumeScale; }

    /// Read-only view (used for the global audio status dump in
    /// AudioService::loopStep and other diagnostics).
    const std::vector<AmbientSound> &getAmbients() const { return mAmbients; }

    /// Live ambient voice count — number of P$AmbientHack ambients with
    /// an active voice this frame. Used by the audio status dump.
    int activeAmbientVoiceCount() const;

private:
    AudioService *mHost;  ///< back-pointer, never null after construction

    /// Active ambient sounds parsed from P$AmbientHack properties.
    std::vector<AmbientSound> mAmbients;

    // ── Tuning (mirrors of the previous AudioService fields) ──
    float       mEnvironmentalSpatialBlend = 0.3f;
    float       mHysteresisStartMul        = 1.5f;
    float       mHysteresisStopMul         = 2.0f;
    int         mDefaultPriority           = 64;
    float       mGlobalVolumeScale         = 1.0f;
};

} // namespace Darkness

#endif // __AMBIENTSOUNDMANAGER_H
