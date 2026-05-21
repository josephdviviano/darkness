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
        // Ambient spawn/halt gate (CPU-side voice lifecycle): Euclidean
        // distance against schema radius with asymmetric hysteresis.
        // `amb.radius` is the authored CPU spawn boundary only — loudness
        // shaping is delegated to Steam Audio (distance, portal, occlusion,
        // air absorption), so the per-radius falloff curve is no longer
        // applied below.
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

        // Voice loudness: schema-authored gain (centibels) + per-object
        // AmbientHack override, converted to linear amplitude once.
        // Steam Audio's per-voice DSP chain (iplDirectEffect distance
        // attenuation + portalAttenuation + portalBlocking + air
        // absorption + occlusion/transmission + HRTF) owns all distance
        // / portal / occlusion shaping downstream — the Dark Engine
        // centibel falloff curve over schema radius double-attenuated
        // against that chain and confused authoring.
        //
        // amb.attenuationFactor (P$SchAttFac) is preserved on the
        // struct but no longer consumed here — future tuning pass will
        // map it onto Steam Audio's rolloffFactor.
        const float gainCb = static_cast<float>(
            amb.volume + schemaVolumeCb(mHost->mSchemaParser.get(), amb.schemaName));
        const float linearVol = centibelsToLinear(gainCb);

        // Ducking applied inline so it doesn't compound across frames.
        // mGlobalVolumeScale compensates for the ambient re-baseline
        // introduced when Steam Audio became the sole player-audio
        // propagation authority — one knob in YAML rather than per-schema.
        mHost->voiceSetLinearVolume(amb.handle,
            linearVol * mHost->currentDuckGain() * mGlobalVolumeScale);
    }
}

//------------------------------------------------------
void AmbientSoundManager::updateSpotAmbientVolumes()
{
    // Per-frame lifecycle for P$SpotAmb. The authored inner/outer envelope
    // is now used as a hard spawn/halt boundary only (d >= outer → halt);
    // the in-band linear fade is delegated to Steam Audio's per-voice DSP
    // chain (distance attenuation + portal/occlusion). Loudness is the
    // authored level + schema gain, applied once.
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

        // Spawn/halt gate: voice is live whenever the listener is inside
        // the authored outer envelope. Level<=0 spots never spawn.
        const bool inEnvelope = (se.outer > 0.0f && d < se.outer && se.level > 0.0f);

        if (!inEnvelope) {
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

        // Authored level (millibel) + schema-authored gain (centibel),
        // converted to linear amplitude once. Steam Audio shapes distance
        // / portal / occlusion downstream.
        const int schemaVolCb = schemaVolumeCb(mHost->mSchemaParser.get(), se.schemaName);
        const int volMb = static_cast<int>(se.level) + schemaVolCb;
        const float linearVol = milliBelIntToLinear(volMb);
        mHost->voiceSetLinearVolume(se.handle,
            linearVol * mHost->currentDuckGain() * mGlobalVolumeScale);
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
