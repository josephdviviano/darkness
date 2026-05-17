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
    /// Per-ambient SHARP-falloff flag, captured from the schema's
    /// SCH_SHARP_FALLOFF (= original engine's SFXFLG_SHARP) at load time.
    /// True → use the 4th-power curve `(d/r)^4` in updateAmbientVolumes;
    /// false → linear `(d/r)`. SHARP is the original engine's default
    /// for ambients (most use it); schemas opt out via an explicit
    /// `flags` directive that clears bit 12.
    bool isSharp = true;
    /// Per-schema attenuation-factor divisor for the volume formula,
    /// loaded from P$SchAttFac at startup (default 1.0). Higher values
    /// make this schema fall off less aggressively. Examples in MISS6:
    /// m06bell=20.0, KARRAS_SCRIPTED=3.0, AI_CONV=1.7, HIT_EXPLOSION=2.0.
    float attenuationFactor = 1.0f;
    SoundHandle handle = SOUND_HANDLE_INVALID; ///< Active voice handle (if playing)
};

/// Spot ambient — alternative ambient encoding with a hard inner/outer
/// falloff envelope (loaded from P$SpotAmb). Unlike AmbientSound (single
/// radius, linear/SHARP falloff), spot ambients are flat-volume within
/// the inner radius, linearly fading to silence between inner and outer,
/// and silent beyond. The original engine ties these to specific scene
/// objects (e.g. mech floor lamps in MISS6).
struct SpotAmbient {
    int objID = 0;
    std::string schemaName;       ///< Schema name (resolved from emitter object's archetype)
    Vector3 position{0, 0, 0};
    float inner = 0.0f;           ///< Distance inside which volume = level
    float outer = 0.0f;           ///< Distance beyond which volume = 0
    float level = 1.0f;           ///< Volume scalar at d <= inner
    SoundHandle handle = SOUND_HANDLE_INVALID;
};

/// Falloff curves for ambient volume models.
///   Linear    — `falloffPct = (d - inner) / (outer - inner)` (saturated to [0,1])
///   Quartic   — `falloffPct = ((d - inner) / (outer - inner))^4`
///               (= the original engine's SHARP curve; near-full volume across
///               most of the radius, falling quickly only near the edge).
enum class FalloffCurve {
    Linear,
    Quartic,
};

/// Unified ambient volume envelope. Returns a normalised "falloff
/// percentage" in [0,1] where 0 means fully inside (d <= inner, voice at
/// authored gain) and 1 means fully outside (d >= outer, voice silent).
/// The two ambient subsystems apply this falloff value to their own
/// gain-domain formulae:
///   • P$AmbientHack uses `volumeCb = gain - falloffPct * (5000 + gain) / atten`
///     with inner = 0, outer = radius, and Linear or Quartic from the schema's
///     SCH_SHARP_FALLOFF bit.
///   • P$SpotAmb uses `targetGain = level * (1 - falloffPct)` (a complement of
///     the normalised falloff, in level-domain). inner/outer come from the
///     property; the curve is always Linear (matches the original engine's
///     inner/outer envelope).
///
/// Behaviour parity with the prior two implementations is preserved
/// exactly: when inner == 0, Linear yields `d/outer` and Quartic yields
/// `(d/outer)^4`, matching the previous P$AmbientHack code paths. The
/// previous P$SpotAmb code path computed `(outer - d)/(outer - inner)`
/// (= 1 - linearFalloff), which equals `1 - computeFalloff(...,Linear)`.
struct AmbientVolumeModel {
    static float computeFalloff(float distance, float inner, float outer,
                                FalloffCurve curve);
};

/// Manages ambient sound lifecycle: parses P$AmbientHack and P$SpotAmb
/// from mission data, starts/stops voices based on listener distance with
/// hysteresis, applies per-voice volume each frame via the unified
/// falloff model. Owned by AudioService; constructed in the service's
/// constructor and torn down with it.
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

    /// Load spot ambients (P$SpotAmb) from mission data.
    void loadSpotAmbients();

    /// Per-frame update for P$AmbientHack ambients: starts/stops voices
    /// based on hysteresis around the authored radius and updates their
    /// volume via the gain-domain falloff formula.
    void updateAmbientVolumes();

    /// Per-frame update for P$SpotAmb spot ambients: starts/stops voices
    /// based on the inner/outer envelope and updates their volume.
    void updateSpotAmbientVolumes();

    /// Drop all parsed ambient/spot-ambient state. Voices are NOT halted
    /// here — the caller (AudioService::shutdown / onDBDrop) is expected
    /// to have already called haltAll() before clearing the bookkeeping.
    void clear();

    /// Tunable hysteresis multipliers. start >= 1.0, stop > start
    /// recommended (the original engine's hysteresis is asymmetric to
    /// avoid start/stop oscillation when the listener hovers near the
    /// boundary). Clamped by the public AudioService API.
    void setHysteresisStartMul(float m) { mHysteresisStartMul = m; }
    void setHysteresisStopMul(float m)  { mHysteresisStopMul  = m; }
    float getHysteresisStartMul() const { return mHysteresisStartMul; }
    float getHysteresisStopMul() const  { return mHysteresisStopMul; }

    /// "linear" or "quadratic" — string passthrough kept for compatibility
    /// with the previous AudioService API. The actual per-ambient curve
    /// is chosen from the schema's SCH_SHARP_FALLOFF bit, not this
    /// setting; this remains for future use.
    void setFalloffCurve(const std::string &s) {
        mFalloffCurve = (s == "linear" ? "linear" : "quadratic");
    }
    const std::string &getFalloffCurve() const { return mFalloffCurve; }

    void setDefaultPriority(int p) { mDefaultPriority = p; }
    int  getDefaultPriority() const { return mDefaultPriority; }

    void setEnvironmentalSpatialBlend(float b) { mEnvironmentalSpatialBlend = b; }
    float getEnvironmentalSpatialBlend() const { return mEnvironmentalSpatialBlend; }

    /// Read-only views (used for the global audio status dump in
    /// AudioService::loopStep and other diagnostics).
    const std::vector<AmbientSound> &getAmbients() const { return mAmbients; }
    const std::vector<SpotAmbient> &getSpotAmbients() const { return mSpotAmbients; }

    /// Live ambient voice count — number of P$AmbientHack ambients with
    /// an active voice this frame. Used by the audio status dump.
    int activeAmbientVoiceCount() const;

private:
    AudioService *mHost;  ///< back-pointer, never null after construction

    /// Active ambient sounds parsed from P$AmbientHack properties.
    std::vector<AmbientSound> mAmbients;

    /// Active spot-ambient instances parsed from P$SpotAmb properties.
    std::vector<SpotAmbient> mSpotAmbients;

    // ── Tuning (mirrors of the previous AudioService fields) ──
    float       mEnvironmentalSpatialBlend = 0.3f;
    float       mHysteresisStartMul        = 1.5f;
    float       mHysteresisStopMul         = 2.0f;
    std::string mFalloffCurve              = "quadratic";
    int         mDefaultPriority           = 64;
};

} // namespace Darkness

#endif // __AMBIENTSOUNDMANAGER_H
