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
#include <cstdio>
#include <cstring>
#include <unordered_set>

namespace Darkness {

namespace {

/// Look up authored schema volume in centibels. Returns 0 when the schema
/// is unresolved (matches the previous behaviour of treating "no schema
/// found" as gain=0).
int schemaVolumeCb(SchemaParser *parser, const std::string &name)
{
    if (!parser) return 0;
    const SchemaEntry *sch = parser->findSchema(name);
    return sch ? sch->playParams.volume : 0;
}

/// Centibels → linear amplitude with -10000 clamped to silence.
float centibelsToLinear(float cb)
{
    if (cb <= -10000.0f) return 0.0f;
    return std::pow(10.0f, cb / 2000.0f);
}

/// Millibel-style integer gain → linear amplitude. Positive values clamp
/// to unity (matches the SpotAmb level convention).
float milliBelIntToLinear(int volMb)
{
    if (volMb >= 0)              return 1.0f;
    if (volMb <= -10000)         return 0.0f;
    return std::pow(10.0f, volMb / 2000.0f);
}

} // anonymous namespace

//------------------------------------------------------
float AmbientVolumeModel::computeFalloff(float distance, float inner,
                                         float outer, FalloffCurve curve)
{
    // Inner/outer envelope normalised to [0,1]. d<=inner → 0 (full volume),
    // d>=outer → 1 (silent), in-band → curve-shaped fraction.
    if (outer <= inner) {
        return (distance >= outer) ? 1.0f : 0.0f;  // degenerate envelope: step at `outer`
    }
    if (distance <= inner) return 0.0f;
    if (distance >= outer) return 1.0f;
    const float t = (distance - inner) / (outer - inner);
    switch (curve) {
        case FalloffCurve::Linear:  return t;
        case FalloffCurve::Quartic: return t * t * t * t;  // original engine's SHARP curve
    }
    return t;
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

    // Objects with P$AmbientHack (property name "AmbientHa" in the gamesys).
    auto objIDs = getAllObjectsWithProperty(mHost->mPropertyService.get(),
                                            "AmbientHa");

    // One warning per unique missing schema across all references.
    std::unordered_set<std::string> warnedMissing;

    int loaded = 0;
    int skipped = 0;
    for (int objID : objIDs) {
        if (objID <= 0) continue;  // archetypes only carry templates

        size_t rawSize = 0;
        const uint8_t *raw = getPropertyRawData(
            mHost->mPropertyService.get(), "AmbientHa", objID, rawSize);
        if (!raw || rawSize < sizeof(PropAmbientHack))
            continue;

        const auto *prop = reinterpret_cast<const PropAmbientHack *>(raw);
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

        // Per-schema falloff params. SHARP is the engine's default for ambients
        // (P$SchPlayPa dtype default 0x7F00 includes bit 12 = SFXFLG_SHARP);
        // schemas opt out via an explicit `flags` directive that clears it.
        if (mHost->mSchemaParser) {
            if (const SchemaEntry *sch = mHost->mSchemaParser->findSchema(amb.schemaName)) {
                amb.isSharp = (sch->playParams.flags & SCH_SHARP_FALLOFF) != 0;
                amb.attenuationFactor = sch->playParams.attenuationFactor;
            }
        }

        if (amb.schemaName.empty())
            continue;

        // Drop ambients with no resolvable schema AND no raw-sample fallback,
        // so we don't churn through doomed lookups every frame.
        const bool schemaExists = mHost->mSchemaParser &&
                                  mHost->mSchemaParser->findSchema(amb.schemaName) != nullptr;
        const bool rawSampleExists = mHost->mSoundLoader &&
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
        if (objID <= 0) continue;

        size_t sz = 0;
        const uint8_t *bytes = storage->getRawData(objID, sz);
        if (!bytes || sz < sizeof(PropSpotAmb)) continue;
        PropSpotAmb sa;
        std::memcpy(&sa, bytes, sizeof(sa));

        if (sa.outer <= 0.0f || sa.outer < sa.inner || sa.ambient < 0.0f)
            continue;

        SpotAmbient se;
        se.objID = objID;
        se.position = mHost->mObjectService->position(objID);
        se.inner = sa.inner;
        se.outer = sa.outer;
        se.level = sa.ambient;

        // The schema name is borrowed from the object's P$AmbientHack
        // (concrete or inherited). If absent, the spot ambient is captured
        // but won't start (logged for diagnostics).
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
        // Ambient activation uses Euclidean distance — radius-based; portal
        // routing handles cross-room propagation downstream. Start radius is
        // narrower than stop radius (asymmetric hysteresis) to prevent
        // start/stop oscillation when the listener hovers near the boundary.
        const float dist = glm::length(mHost->mListenerPos - amb.position);
        const float startRadius = amb.radius * mHysteresisStartMul;
        const float stopRadius  = amb.radius * mHysteresisStopMul;

        const bool alreadyPlaying = (amb.handle != SOUND_HANDLE_INVALID);
        const bool inRange = (amb.radius > 0.0f &&
                              (alreadyPlaying ? dist < stopRadius : dist < startRadius));

        if (!inRange) {
            // Out of range — fade out (mirror the smooth ramp-up at start).
            if (amb.handle != SOUND_HANDLE_INVALID) {
                mHost->haltSound(amb.handle, /*fadeMs=*/200);
                amb.handle = SOUND_HANDLE_INVALID;
            }
            continue;
        }

        const bool isEnvironmental = (amb.flags & AMB_ENVIRONMENTAL) != 0;
        bool justCreated = false;

        if (amb.handle == SOUND_HANDLE_INVALID) {
            const bool isLooping = !(amb.flags & AMB_ONCE_ONLY);
            // Decide VoiceClass BEFORE startVoice so createVoiceSource applies
            // the per-class default directParams on the first audio callback
            // (otherwise it reads silent initVoiceDSP defaults for ~21 ms).
            const VoiceClass cls = isEnvironmental ? VoiceClass::Ambient
                                                   : VoiceClass::Normal;

            const SchemaEntry *schema = mHost->mSchemaParser
                ? mHost->mSchemaParser->findSchema(amb.schemaName) : nullptr;

            if (schema && !schema->samples.empty()) {
                amb.handle = mHost->startVoice(amb.schemaName, schema->samples[0].name,
                                               amb.position,
                                               schema->playParams.priority,
                                               isLooping, amb.objID, 0.0f, cls);
            }
            if (amb.handle == SOUND_HANDLE_INVALID) {
                // Fallback: try the schema name as a raw sample.
                std::fprintf(stderr,
                    "[FALLBACK] AmbientSoundManager: schema '%s' on obj %d "
                    "had no usable samples — trying raw sample of the same name\n",
                    amb.schemaName.c_str(), amb.objID);
                amb.handle = mHost->startVoice(amb.schemaName, amb.schemaName,
                                               amb.position, mDefaultPriority,
                                               isLooping, amb.objID, 0.0f, cls);
            }
            justCreated = mHost->voiceExists(amb.handle);

            if (justCreated) {
                // Per-source BFS-termination distance, matching the original
                // Dark Engine's per-source attenuation:
                //   SFX_MaxDist(gain) = (5000 + gain) / 55
                //   m_MaxDistance     = SFX_MaxDist(gain) * atten_factor
                // The implicit phantom-portal filter: a -3000cB ambient with
                // atten_factor=1.0 gets ~36 ft, so BFS exhausts before
                // reaching distant listeners through phantom chains.
                constexpr float kAttenuationFactor = 55.0f;
                const int   gainCb = schemaVolumeCb(mHost->mSchemaParser.get(),
                                                   amb.schemaName);
                const float gain   = static_cast<float>(gainCb);
                const float atten  = (amb.attenuationFactor > 0.01f)
                                     ? amb.attenuationFactor : 1.0f;
                float sfxMaxDist = (5000.0f + gain) / kAttenuationFactor;
                if (sfxMaxDist < 1.0f) sfxMaxDist = 1.0f;
                mHost->voiceSetMaxAudibleDist(amb.handle, sfxMaxDist * atten);

                // AMB_ENVIRONMENTAL: diffuse spatial blend (wind, room tone)
                // — mixes HRTF with mono passthrough so the source feels
                // ambient rather than pinpoint. Object-attached ambients
                // keep 1.0 (full HRTF) so a generator hum still points at
                // the generator. The audio thread reads the atomic every
                // callback; one early callback may use 1.0 before this
                // store lands, but ambients ramp up from volume 0 so the
                // leakage is below audibility.
                if (isEnvironmental) {
                    mHost->voiceSetSpatialBlendOverride(
                        amb.handle, mEnvironmentalSpatialBlend);
                }

                // Fan out an emission event so AI hearing observes the new
                // ambient. gainDb = AmbientHack override + schema-authored
                // (both centibels, additive) — matches the player-audible
                // gain in the volume formula below.
                SoundEmissionEvent ev{};
                ev.emitterObjID = amb.objID;
                ev.position     = amb.position;
                ev.schemaName   = amb.schemaName;
                ev.soundType    = 0;  // ambients aren't AI-typed
                ev.baseRange    = amb.radius;
                ev.gainDb       = static_cast<float>(amb.volume + gainCb);
                mHost->publishSoundEmission(ev);
            }
        }

        // Just-created voices stay at 0 — silent defaults in initVoiceDSP
        // hold iplDirectEffectApply silent until the sim provides real
        // occlusion/distance data next frame.
        if (justCreated || amb.handle == SOUND_HANDLE_INVALID)
            continue;
        if (!mHost->voiceExists(amb.handle)) {
            amb.handle = SOUND_HANDLE_INVALID;
            continue;
        }

        // Distance-attenuated volume, original engine formula:
        //   vol_cb       = gain - falloffPct * (5000 + gain) / atten_factor
        //   falloffPct   = (d/r)   linear (default), (d/r)^4 SHARP
        //   linearVol    = 10^(vol_cb / 2000)
        // At d=0 vol_cb = gain; at d=radius vol_cb = gain - (5000+gain)/atten
        // (= -50 dB for atten=1, gain=0). AMB_NO_FADE forces linear regardless
        // of schema's SHARP flag — designer explicitly tagged "no fade".
        const float falloffDist = mHost->voiceFalloffDistance(amb.handle, dist);
        const bool  useSharp    = amb.isSharp && !(amb.flags & AMB_NO_FADE);
        const FalloffCurve curve = useSharp ? FalloffCurve::Quartic
                                            : FalloffCurve::Linear;
        const float falloffPct = AmbientVolumeModel::computeFalloff(
            falloffDist, /*inner=*/0.0f, /*outer=*/amb.radius, curve);

        // Combine schema-authored gain with the per-object AmbientHack override
        // (both centibels — additive = multiplicative on linear amplitudes).
        // Without this, schemas authored at -1700cB (≈-17dB) play at unity.
        const float gainCb = static_cast<float>(
            amb.volume + schemaVolumeCb(mHost->mSchemaParser.get(), amb.schemaName));
        // Per-schema attenuation factor (P$SchAttFac) divides the per-radius
        // dB drop. Default 1.0 = original (-50 dB at radius for gain=0).
        const float attenuation = amb.attenuationFactor > 0.0f
                                  ? amb.attenuationFactor : 1.0f;
        const float volumeCb = gainCb - (falloffPct * (5000.0f + gainCb)) / attenuation;
        const float linearVol = centibelsToLinear(volumeCb);

        // Ducking applied inline so it doesn't compound across frames.
        mHost->voiceSetLinearVolume(amb.handle, linearVol * mHost->currentDuckGain());
    }
}

//------------------------------------------------------
void AmbientSoundManager::updateSpotAmbientVolumes()
{
    // Per-frame envelope evaluation + lifecycle for P$SpotAmb. Uses the
    // hard inner/outer envelope:
    //   d <= inner → volume = level
    //   in band   → volume = level * (outer - d) / (outer - inner)
    //   d >= outer → volume = 0
    // `level` is millibel gain (same convention as AmbientSound::volume).
    if (mSpotAmbients.empty())
        return;

    // Diagnostic log throttle.
    static float spotDebugTimer = 0.0f;
    spotDebugTimer += 1.0f / 60.0f;
    const bool emitLog = (spotDebugTimer > 5.0f);
    if (emitLog) spotDebugTimer = 0.0f;

    int audible = 0;
    int playing = 0;
    for (auto &se : mSpotAmbients) {
        const float d = glm::distance(mHost->mListenerPos, se.position);

        // Unified envelope (Linear curve), converted to level-domain target
        // by complement: target = level * (1 - pct).
        const float falloffPct = AmbientVolumeModel::computeFalloff(
            d, se.inner, se.outer, FalloffCurve::Linear);
        const float envVol = se.level * (1.0f - falloffPct);

        if (envVol <= 0.0f) {
            // Out of envelope — fade out.
            if (se.handle != SOUND_HANDLE_INVALID) {
                mHost->haltSound(se.handle, /*fadeMs=*/200);
                se.handle = SOUND_HANDLE_INVALID;
            }
            continue;
        }

        ++audible;

        bool justCreated = false;
        if (se.handle == SOUND_HANDLE_INVALID) {
            if (se.schemaName.empty()) continue;  // already counted at load
            if (!mHost->mSchemaParser) continue;
            const SchemaEntry *schema = mHost->mSchemaParser->findSchema(se.schemaName);
            if (!schema || schema->samples.empty()) continue;

            // Spot ambients are looping by definition + Ambient class for
            // diffuse spatialBlend treatment.
            se.handle = mHost->startVoice(se.schemaName, schema->samples[0].name,
                                          se.position, schema->playParams.priority,
                                          /*isLooping=*/true, se.objID, 0.0f,
                                          VoiceClass::Ambient);
            if (mHost->voiceExists(se.handle)) {
                justCreated = true;
                // Cap BFS at outer envelope — past outer the source is silent.
                mHost->voiceSetMaxAudibleDist(se.handle, se.outer);
                mHost->voiceSetSpatialBlendOverride(se.handle, mEnvironmentalSpatialBlend);
            }
        }

        if (se.handle == SOUND_HANDLE_INVALID) continue;
        if (!mHost->voiceExists(se.handle)) {
            // Voice died (priority eviction) — clear, retry next frame.
            se.handle = SOUND_HANDLE_INVALID;
            continue;
        }

        // Combine envVol (millibel) with schema-authored gain (centibel).
        const int schemaVolCb = schemaVolumeCb(mHost->mSchemaParser.get(), se.schemaName);
        const int volMb = static_cast<int>(envVol) + schemaVolCb;
        const float linearVol = milliBelIntToLinear(volMb);
        mHost->voiceSetLinearVolume(se.handle, linearVol * mHost->currentDuckGain());
        ++playing;
        (void)justCreated;
    }
    if (emitLog && audible > 0) {
        AUDIO_LOG("AudioService: [SPOT_AMB] %d/%zu spot ambient(s) audible "
                  "(%d voice(s) active)\n",
                  audible, mSpotAmbients.size(), playing);
    }
}

} // namespace Darkness
