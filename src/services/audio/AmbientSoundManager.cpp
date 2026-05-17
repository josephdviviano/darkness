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

#include "AmbientSoundManager.h"

#include "AudioLog.h"
#include "AudioService.h"
#include "CRFSoundLoader.h"
#include "SchemaParser.h"
#include "SchemaTypes.h"

#include "object/ObjectService.h"
#include "property/DarkPropertyDefs.h"
#include "property/PropertyService.h"
#include "property/TypedProperty.h"
#include "logger.h"

#include <cmath>
#include <cstring>
#include <unordered_set>

namespace Darkness {

//------------------------------------------------------
float AmbientVolumeModel::computeFalloff(float distance, float inner,
                                         float outer, FalloffCurve curve)
{
    // Inner/outer envelope, normalised to [0,1] across the fade band.
    // d <= inner → 0   (fully audible, no falloff applied)
    // d >= outer → 1   (silenced)
    // in band   → curve-shaped fraction
    if (outer <= inner) {
        // Degenerate envelope: a single step at `outer`. Avoids divide-by-zero.
        return (distance >= outer) ? 1.0f : 0.0f;
    }
    if (distance <= inner) return 0.0f;
    if (distance >= outer) return 1.0f;
    float t = (distance - inner) / (outer - inner);
    switch (curve) {
        case FalloffCurve::Linear:
            return t;
        case FalloffCurve::Quartic:
            // (d/r)^4 in the inner==0 case — matches the original engine's
            // SHARP-falloff curve. Equivalent to a four-multiply chain
            // since `t` is already in [0,1].
            return t * t * t * t;
    }
    return t;  // unreachable; satisfies non-void warning
}

//------------------------------------------------------
AmbientSoundManager::AmbientSoundManager(AudioService *host)
    : mHost(host)
{
}

//------------------------------------------------------
AmbientSoundManager::~AmbientSoundManager() = default;

//------------------------------------------------------
void AmbientSoundManager::clear()
{
    mAmbients.clear();
    mSpotAmbients.clear();
}

//------------------------------------------------------
int AmbientSoundManager::activeAmbientVoiceCount() const
{
    int playing = 0;
    for (const auto &amb : mAmbients) {
        if (mHost->voiceExists(amb.handle))
            ++playing;
    }
    return playing;
}

//------------------------------------------------------
void AmbientSoundManager::loadAmbientSounds()
{
    if (!mHost->mPropertyService || !mHost->mObjectService || !mHost->mAudioReady)
        return;

    // Find all objects with P$AmbientHack (property name "AmbientHa" in the gamesys)
    auto objIDs = getAllObjectsWithProperty(mHost->mPropertyService.get(),
                                            "AmbientHa");

    // Track schemas we've already warned about so we emit one warning per
    // unique missing name regardless of how many objects reference it.
    // (e.g. miss6.mis references the undefined schema "m06subson" on 7 objects.)
    std::unordered_set<std::string> warnedMissing;

    int loaded = 0;
    int skipped = 0;
    for (int objID : objIDs) {
        // Only process concrete objects (positive IDs), not archetypes
        if (objID <= 0)
            continue;

        size_t rawSize = 0;
        const uint8_t *raw = getPropertyRawData(
            mHost->mPropertyService.get(), "AmbientHa", objID, rawSize);
        if (!raw || rawSize < sizeof(PropAmbientHack))
            continue;

        const auto *prop = reinterpret_cast<const PropAmbientHack *>(raw);

        // Skip turned-off ambients
        if (prop->flags & AMB_TURNED_OFF)
            continue;

        AmbientSound amb;
        amb.objID = objID;
        amb.schemaName = std::string(prop->schema,
            strnlen(prop->schema, sizeof(prop->schema)));
        amb.radius = static_cast<float>(prop->radius);
        amb.volume = prop->volume;
        amb.flags = prop->flags;
        amb.position = mHost->mObjectService->position(objID);
        // Capture per-schema falloff parameters at ambient creation.
        // SHARP is the original engine's default for schemas (P$SchPlayPa
        // dtype default is 0x7F00, which includes bit 12 = SFXFLG_SHARP).
        // The .sch parser also defaults SchemaPlayParams.flags to
        // SCH_SHARP_FALLOFF, so any schema without an explicit `flags`
        // directive that clears bit 12 ends up with SHARP. The
        // attenuation factor comes from P$SchAttFac (applied earlier on
        // schema load).
        if (mHost->mSchemaParser) {
            if (const SchemaEntry *sch = mHost->mSchemaParser->findSchema(amb.schemaName)) {
                amb.isSharp = (sch->playParams.flags & SCH_SHARP_FALLOFF) != 0;
                amb.attenuationFactor = sch->playParams.attenuationFactor;
            }
        }

        if (amb.schemaName.empty())
            continue;

        // Validate that the referenced schema exists, or — failing that —
        // that snd.crf contains a raw WAV with the same name (the runtime
        // fallback path in updateAmbientVolumes also tries this). If both
        // miss, the original game data has a dangling reference; drop the
        // ambient so we don't churn through a doomed lookup every frame.
        bool schemaExists = mHost->mSchemaParser &&
                            mHost->mSchemaParser->findSchema(amb.schemaName) != nullptr;
        bool rawSampleExists = mHost->mSoundLoader &&
                               mHost->mSoundLoader->hasSound(amb.schemaName);
        if (!schemaExists && !rawSampleExists) {
            if (warnedMissing.insert(amb.schemaName).second) {
                LOG_ERROR("AudioService: AmbientHack on obj %d references "
                          "unknown schema/sample '%s' — skipping",
                          objID, amb.schemaName.c_str());
            }
            ++skipped;
            continue;
        }

        // Register ambient data only — voices are started/stopped dynamically
        // in updateAmbientVolumes() based on listener distance.
        mAmbients.push_back(std::move(amb));
        ++loaded;
    }

    if (loaded > 0) {
        LOG_INFO("AudioService: registered %d ambient sounds from P$AmbientHack", loaded);
    }
    if (skipped > 0) {
        LOG_INFO("AudioService: skipped %d ambient sound(s) with missing schemas/samples",
                 skipped);
    }
}

//------------------------------------------------------
void AmbientSoundManager::loadSpotAmbients()
{
    if (!mHost->mPropertyService || !mHost->mObjectService)
        return;

    mSpotAmbients.clear();

    Property *prop = mHost->mPropertyService->getProperty("SpotAmb");
    if (!prop || !prop->getStorage())
        return;
    DataStorage *storage = prop->getStorage();
    IntIteratorPtr it = storage->getAllStoredObjects();

    int loaded = 0;
    int skippedNoSchema = 0;
    while (!it->end()) {
        int objID = it->next();
        if (objID <= 0) continue;  // archetypes only carry templates

        size_t sz = 0;
        const uint8_t *bytes = storage->getRawData(objID, sz);
        if (!bytes || sz < sizeof(PropSpotAmb)) continue;
        PropSpotAmb sa;
        std::memcpy(&sa, bytes, sizeof(sa));

        // Sanity: outer >= inner > 0, level >= 0.
        if (sa.outer <= 0.0f || sa.outer < sa.inner || sa.ambient < 0.0f)
            continue;

        SpotAmbient se;
        se.objID = objID;
        se.position = mHost->mObjectService->position(objID);
        se.inner = sa.inner;
        se.outer = sa.outer;
        se.level = sa.ambient;

        // Resolve the schema name. The original engine ties a spot
        // ambient to whatever ambient schema the object would otherwise
        // play — typically the object's P$AmbientHack (concrete or
        // inherited). If the object also carries an AmbientHa property
        // we reuse its schema field; otherwise this spot ambient has no
        // schema mapping yet (logged for diagnostics — the data is still
        // captured so a follow-up resolver can complete it).
        size_t ahSize = 0;
        const uint8_t *ahRaw = getPropertyRawData(
            mHost->mPropertyService.get(), "AmbientHa", objID, ahSize);
        if (ahRaw && ahSize >= sizeof(PropAmbientHack)) {
            const auto *ah = reinterpret_cast<const PropAmbientHack *>(ahRaw);
            se.schemaName.assign(ah->schema,
                strnlen(ah->schema, sizeof(ah->schema)));
        }
        if (se.schemaName.empty())
            ++skippedNoSchema;

        mSpotAmbients.push_back(std::move(se));
        ++loaded;
    }

    if (loaded > 0) {
        AUDIO_LOG("AudioService: loaded %d spot ambient(s) (P$SpotAmb), "
                  "%d without resolved schema name\n",
                  loaded, skippedNoSchema);
    }
}

//------------------------------------------------------
void AmbientSoundManager::updateAmbientVolumes()
{
    if (mAmbients.empty())
        return;

    for (auto &amb : mAmbients) {
        // Ambient sound activation uses EUCLIDEAN distance, matching the
        // original Dark Engine's ambient system. The room portal system was
        // never used for ambient activation — ambients work purely on radius.
        // Room routing handles cross-room propagation for voices in the
        // voice update loop (Step 2b), not here.
        //
        // Start voices BEFORE the source becomes audible so the distance-based
        // volume curve fades in smoothly. Stop at a wider hysteresis radius
        // to avoid rapid start/stop oscillation when the listener hovers near
        // the boundary. Multipliers come from audio.ambient config.
        float dist = glm::length(mHost->mListenerPos - amb.position);
        float startRadius = amb.radius * mHysteresisStartMul;
        float stopRadius  = amb.radius * mHysteresisStopMul;

        bool alreadyPlaying = (amb.handle != SOUND_HANDLE_INVALID);
        bool inRange = (amb.radius > 0.0f &&
                        (alreadyPlaying ? dist < stopRadius : dist < startRadius));

        if (inRange) {
            // Start voice if not already playing
            bool justCreated = false;
            bool isEnvironmental = (amb.flags & AMB_ENVIRONMENTAL) != 0;
            if (amb.handle == SOUND_HANDLE_INVALID) {
                bool isLooping = !(amb.flags & AMB_ONCE_ONLY);
                // Environmental ambients use the ambient system's radius
                // curve for distance; object ambients use Steam Audio
                // distance. Decide BEFORE startVoice so the VoiceClass tag
                // is set before createVoiceSource applies its per-class
                // default directParams — otherwise the first audio callback
                // reads the silent initVoiceDSP defaults (~21 ms gap).
                VoiceClass cls = isEnvironmental ? VoiceClass::Ambient
                                                 : VoiceClass::Normal;
                if (mHost->mSchemaParser) {
                    const SchemaEntry *schema =
                        mHost->mSchemaParser->findSchema(amb.schemaName);
                    if (schema && !schema->samples.empty()) {
                        const SchemaSample &sample = schema->samples[0];
                        // Start at volume 0 — stays silent until the sim has
                        // processed this source (next frame) so occlusion is
                        // applied before the voice is audible.
                        amb.handle = mHost->startVoice(amb.schemaName, sample.name,
                                                      amb.position,
                                                      schema->playParams.priority,
                                                      isLooping, amb.objID, 0.0f, cls);
                    }
                }
                if (mHost->voiceExists(amb.handle)) {
                    justCreated = true;
                }
                // Fallback: try loading schema name as a raw sound
                if (amb.handle == SOUND_HANDLE_INVALID) {
                    amb.handle = mHost->startVoice(amb.schemaName, amb.schemaName,
                                                  amb.position, mDefaultPriority,
                                                  isLooping, amb.objID, 0.0f, cls);
                    if (mHost->voiceExists(amb.handle)) {
                        justCreated = true;
                    }
                }

                // Set the voice's per-source max audible distance from
                // the ambient's radius. BFS terminates beyond this, so
                // the sound naturally stops propagating once the portal-
                // graph path exceeds the source's effective reach.
                //
                // Per-source BFS-termination distance, matching the
                // original Dark Engine's per-source attenuation:
                //   SFX_MaxDist(gain) = (5000 + gain) / attenuation_factor
                //   m_MaxDistance = SFX_MaxDist(gain) * atten_factor
                // with attenuation_factor = 55 (engine-config default).
                //
                // This is the IMPLICIT phantom-portal filter in the
                // original engine: a typical ambient with gain = -3000
                // (-30 dB) and atten_factor = 1.0 gets m_MaxDistance =
                // (5000 - 3000) / 55 * 1.0 ≈ 36.4 ft. BFS exhausts that
                // budget before reaching distant listeners through
                // phantom chains, naturally silencing audio that
                // shouldn't propagate.
                //
                // History: this used to be `amb.radius * 2.0f`, which
                // for a typical-radius ambient produces ~110 ft — about
                // 3× the original's per-source budget for the same
                // schema. The radius gate worked for the obvious "wind
                // ambient with radius=25" case but allowed long phantom
                // chains for higher-radius ambients (cathedral mech-
                // angels, etc.) to reach listeners they shouldn't.
                if (mHost->voiceExists(amb.handle)) {
                    constexpr float kAttenuationFactor = 55.0f;
                    // Re-look-up the schema; the outer-scope `schema`
                    // pointer is out of scope here. Gain is in centibels
                    // (negative = attenuated, 0 = full). Some ambients
                    // spawn through the raw-sound fallback and have no
                    // schema — default to gain=0 for those.
                    int gainCb = 0;
                    if (mHost->mSchemaParser) {
                        const SchemaEntry *sch =
                            mHost->mSchemaParser->findSchema(amb.schemaName);
                        if (sch) gainCb = sch->playParams.volume;
                    }
                    const float gain = static_cast<float>(gainCb);
                    const float attenFactor = (amb.attenuationFactor > 0.01f)
                                                 ? amb.attenuationFactor
                                                 : 1.0f;
                    float sfxMaxDist = (5000.0f + gain) / kAttenuationFactor;
                    if (sfxMaxDist < 1.0f) sfxMaxDist = 1.0f;
                    mHost->voiceSetMaxAudibleDist(amb.handle,
                                                  sfxMaxDist * attenFactor);
                }

                // Per-voice spatialBlend override for room-attached
                // ambients (AMB_ENVIRONMENTAL: wind, church reverberance,
                // generic "room tone"). Lowering binaural spatialBlend
                // mixes the HRTF output with mono passthrough so the
                // sound feels diffuse rather than coming from a single
                // point in space. Object-attached ambients (no
                // AMB_ENVIRONMENTAL flag) skip this override and keep the
                // default 1.0 so they stay directional (e.g. an audible
                // generator hum should still point at the generator).
                //
                // Setting once at activation is correct: the override is
                // an atomic that the audio thread reads every callback,
                // and the value is constant for the voice's lifetime
                // (the AMB_ENVIRONMENTAL flag doesn't change post-spawn).
                // The audio thread may fire one callback before this
                // store lands (~21 ms of full HRTF before the diffuse
                // override takes effect); ambients start at volume 0 and
                // ramp up in updateAmbientVolumes, so any leakage during
                // that window is below the audibility threshold anyway.
                if (justCreated && isEnvironmental
                    && mHost->voiceExists(amb.handle)) {
                    mHost->voiceSetSpatialBlendOverride(
                        amb.handle, mEnvironmentalSpatialBlend);
                }

                // Fan out a sound-emission event so AI hearing /
                // diagnostic listeners can observe the new ambient.
                // soundType defaults to Untyped (0); ambients aren't
                // typed in P$AmbientHack. gainDb uses the schema-
                // authored volume (millibels — caller can interpret).
                if (justCreated && amb.handle != SOUND_HANDLE_INVALID) {
                    SoundEmissionEvent ev{};
                    ev.emitterObjID = amb.objID;
                    ev.position     = amb.position;
                    ev.schemaName   = amb.schemaName;
                    ev.soundType    = 0; // Untyped — ambients don't carry an AI sound type
                    ev.baseRange    = amb.radius;
                    ev.gainDb       = static_cast<float>(amb.volume);
                    mHost->publishSoundEmission(ev);
                }
            }

            // Update volume.
            // Just-created voices stay at 0 — silent defaults in initVoiceDSP
            // ensure iplDirectEffectApply outputs silence until the sim provides
            // real occlusion/distance data (next frame).
            if (!justCreated && amb.handle != SOUND_HANDLE_INVALID) {
                if (!mHost->voiceExists(amb.handle)) {
                    amb.handle = SOUND_HANDLE_INVALID;
                    continue;
                }

                // Distance-attenuated volume, per the original engine's
                // formula. Combines the schema-authored gain (centibels)
                // with the BFS-munged path distance against the ambient's
                // authored radius:
                //
                //   volume_centibels = gain - falloffPct * (5000 + gain)
                //   falloffPct       = (d/r)        for default (linear)
                //                    = (d/r)^4      for SCH_SHARP_FALLOFF
                //   linearAmplitude  = 10^(volume_centibels / 2000)
                //
                // At d=0: volume_centibels = gain (the schema's authored
                // level — full volume of the sample). At d=radius:
                // volume_centibels = gain - (5000 + gain), i.e. -50 dB
                // relative to a gain=0 reference. The curve is linear in
                // centibels (= linear in dB) so the rate of change is
                // constant, with no cliff near radius.
                //
                // SHARP-tagged ambients use (d/r)^4 instead: near-full
                // volume across most of the radius, falling quickly only
                // near the edge. The original engine sets SFXFLG_SHARP on
                // sound effects where the designer wants tighter
                // localization (e.g. small machine hums) vs. ambient room
                // tones that need to fill a large volume.
                //
                // Fallback to Euclidean when the voice's BFS path hasn't
                // resolved (cachedProp.reached == false; e.g. first frame
                // before propagation runs).
                float falloffDist = mHost->voiceFalloffDistance(amb.handle, dist);
                // AMB_NO_FADE forces linear regardless of schema's SHARP
                // flag — the level designer explicitly tagged this
                // ambient as "no fade", so we use the gentlest curve.
                bool useSharp = amb.isSharp && !(amb.flags & AMB_NO_FADE);
                FalloffCurve curve = useSharp ? FalloffCurve::Quartic
                                              : FalloffCurve::Linear;
                // Unified envelope: inner=0, outer=radius — d <= 0 gives 0
                // (full gain), d >= radius gives 1 (full attenuation step).
                // Matches the previous explicit (d/r) and (d/r)^4 code paths
                // bit-for-bit when inner == 0.
                float falloffPct = AmbientVolumeModel::computeFalloff(
                    falloffDist, /*inner=*/0.0f, /*outer=*/amb.radius, curve);
                float gainCb = static_cast<float>(amb.volume);
                // The original engine's m_ScaleDistance term works out to
                // (5000+gain) once the global attenuation_factor cancels.
                // We replicate that here so the per-radius dB drop is
                // independent of the global tunable (which we don't expose
                // — we apply the original engine's value of 55 implicitly).
                //
                // Per-schema attenuation factor (from P$SchAttFac) is a
                // divisor on the per-radius dB drop: factor=2.0 halves
                // the drop (so -25 dB at radius instead of -50), factor=20
                // (e.g. m06bell) means only ~-2.5 dB at radius. Default
                // 1.0 = no effect.
                float attenuation = amb.attenuationFactor > 0.0f
                                    ? amb.attenuationFactor : 1.0f;
                float volumeCb = gainCb
                                 - (falloffPct * (5000.0f + gainCb)) / attenuation;
                float linearVol = (volumeCb <= -10000.0f)
                                  ? 0.0f
                                  : std::pow(10.0f, volumeCb / 2000.0f);

                // Apply ducking multiplier inline (not as a separate pass)
                // to avoid compounding volume decay across frames.
                float duck = mHost->currentDuckGain();
                mHost->voiceSetLinearVolume(amb.handle, linearVol * duck);
            }
        } else {
            // Out of range — stop voice to free the slot and DSP resources.
            // Use a long fade (200 ms) so a graceful "aged out of audible
            // radius" retirement is heard as a fade-out rather than a hard
            // cut. Start of voice already ramps up smoothly (volume = 0
            // at creation, distFactor evolves from 0 over the first few
            // frames), so this restores symmetry on the trailing edge.
            if (amb.handle != SOUND_HANDLE_INVALID) {
                mHost->haltSound(amb.handle, /*fadeMs=*/200);
                amb.handle = SOUND_HANDLE_INVALID;
            }
        }
    }
}

//------------------------------------------------------
void AmbientSoundManager::updateSpotAmbientVolumes()
{
    // Per-frame envelope evaluation and voice lifecycle for spot ambients.
    // Mirrors updateAmbientVolumes' start/stop/update pattern but uses the
    // hard inner/outer envelope (P$SpotAmb) instead of the radius-based
    // SHARP/linear curve.
    //
    // Envelope (per the original engine):
    //   d <= inner          → volume = level
    //   inner < d < outer   → volume = level * (outer - d) / (outer - inner)
    //   d >= outer          → volume = 0
    //
    // `level` is interpreted as a millibel gain (same convention as
    // AmbientSound::volume). Positive values are clamped to unity by
    // schemaVolumeToLinear; negative values attenuate. When envVol > 0 a
    // voice is started/maintained; at envVol == 0 (d ≥ outer) the voice
    // is halted with a short fade.
    if (mSpotAmbients.empty())
        return;

    // Throttle the diagnostic log so it doesn't spam every frame.
    static float spotDebugTimer = 0.0f;
    spotDebugTimer += 1.0f / 60.0f;  // rough per-frame increment
    bool emitLog = (spotDebugTimer > 5.0f);
    if (emitLog) spotDebugTimer = 0.0f;

    int audible = 0;
    int playing = 0;
    for (auto &se : mSpotAmbients) {
        float d = glm::distance(mHost->mListenerPos, se.position);

        // Unified envelope: Linear curve in [inner..outer], converted to
        // the level-domain target by complement (target = level * (1 - pct))
        // so d == inner → level, d == outer → 0. Bit-for-bit equivalent to
        // the previous `level * (outer - d) / (outer - inner)` expression.
        float falloffPct = AmbientVolumeModel::computeFalloff(
            d, se.inner, se.outer, FalloffCurve::Linear);
        float envVol = se.level * (1.0f - falloffPct);

        if (envVol > 0.0f) ++audible;

        if (envVol > 0.0f) {
            // In range — ensure a voice is playing and update its volume.
            bool justCreated = false;
            if (se.handle == SOUND_HANDLE_INVALID) {
                if (se.schemaName.empty()) {
                    // Unresolved schema — can't start anything. Skip
                    // (already counted in the load-time diagnostic).
                    continue;
                }
                if (!mHost->mSchemaParser) continue;
                const SchemaEntry *schema =
                    mHost->mSchemaParser->findSchema(se.schemaName);
                if (!schema || schema->samples.empty()) continue;
                const SchemaSample &sample = schema->samples[0];
                // Spot ambients are looping by definition (they describe a
                // sustained sound bubble around an emitter). Use the
                // Ambient voice class so they get the same diffuse
                // spatialBlend treatment as room-attached ambients.
                bool isLooping = true;
                VoiceClass cls = VoiceClass::Ambient;
                // Start at volume 0 — the volume is set below on the same
                // frame (the silent-defaults-until-occlusion concern that
                // applies to AmbientHack-style ambients doesn't apply
                // here: the envelope has already established audibility).
                se.handle = mHost->startVoice(se.schemaName, sample.name,
                                              se.position,
                                              schema->playParams.priority,
                                              isLooping, se.objID, 0.0f, cls);
                if (mHost->voiceExists(se.handle)) {
                    justCreated = true;
                    // Cap BFS propagation at the outer envelope — past
                    // outer the source is silent anyway.
                    mHost->voiceSetMaxAudibleDist(se.handle, se.outer);
                    // Match AMB_ENVIRONMENTAL diffuse-spatialBlend
                    // treatment so the source feels diffuse rather
                    // than pinpoint.
                    mHost->voiceSetSpatialBlendOverride(
                        se.handle, mEnvironmentalSpatialBlend);
                }
            }

            if (se.handle != SOUND_HANDLE_INVALID) {
                if (!mHost->voiceExists(se.handle)) {
                    // Voice died (e.g. priority eviction). Clear and let
                    // the next frame attempt re-creation.
                    se.handle = SOUND_HANDLE_INVALID;
                    continue;
                }
                // Interpret envVol as a millibel gain (matches
                // AmbientSound::volume convention). Apply duck multiplier
                // inline, same as updateAmbientVolumes.
                // Equivalent to the file-static schemaVolumeToLinear in
                // AudioService.cpp — duplicated here to avoid widening
                // the AudioService API surface. millibels → linear:
                //   -1 (or 0+) = full volume, -10000 = silence,
                //   otherwise  10^(volume/2000).
                int volMb = static_cast<int>(envVol);
                float linearVol;
                if (volMb >= 0)              linearVol = 1.0f;
                else if (volMb <= -10000)    linearVol = 0.0f;
                else                         linearVol = std::pow(10.0f, volMb / 2000.0f);
                float duck = mHost->currentDuckGain();
                mHost->voiceSetLinearVolume(se.handle, linearVol * duck);
                if (!justCreated) ++playing;
                else ++playing;
            }
        } else {
            // Out of envelope — stop the voice if one is playing. Use a
            // long fade so the trailing edge is graceful (mirrors the
            // 200ms used in updateAmbientVolumes).
            if (se.handle != SOUND_HANDLE_INVALID) {
                mHost->haltSound(se.handle, /*fadeMs=*/200);
                se.handle = SOUND_HANDLE_INVALID;
            }
        }
    }
    if (emitLog && audible > 0) {
        AUDIO_LOG("AudioService: [SPOT_AMB] %d/%zu spot ambient(s) audible "
                  "(%d voice(s) active)\n",
                  audible, mSpotAmbients.size(), playing);
    }
}

} // namespace Darkness
