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

                // Push the schema's P$SchAttFac into Steam Audio's distance
                // model as a per-voice rolloff factor (mapped to
                // INVERSEDISTANCE minDistance × atten — sound stays at full
                // volume out to N meters before 1/d falloff). Default 1.0
                // matches Steam Audio's DEFAULT model. Schemas like m06bell
                // ship factors up to ~20 for long-carry sounds.
                mHost->voiceSetAttenuationFactor(amb.handle, atten);

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
        // Steam Audio's per-voice DSP chain owns all distance / portal
        // / occlusion shaping downstream:
        //   • iplDirectEffect — distance attenuation (INVERSEDISTANCE
        //     model with minDistance = 1m × P$SchAttFac), air
        //     absorption, volumetric occlusion against world
        //     geometry + door OBBs, frequency-dependent transmission
        //   • iplPathEffect   — routed-around-obstacles wet bus
        //     (3-band EQ from eqCoeffs + ambisonic SH direction)
        //   • portalAttenuation / portalBlocking — derived from the
        //     path solver's eqCoeffs (AudioService.cpp post-getOutputs
        //     block); these drive the reflection-send door blocking
        //     (reverb tail attenuates + muffles for sources behind
        //     closed doors)
        //   • iplReflectionEffect — room reverb tail
        //
        // No distance term is applied here at the miniaudio source-
        // volume layer because Steam Audio's INVERSEDISTANCE model
        // already does that on the dry path, the path effect does it
        // internally on the wet pathing bus, and the reverb-send
        // multiplies in directParams.distanceAttenuation. Adding a
        // ma_sound_set_volume distance curve here would double-
        // attenuate every downstream pathway.
        //
        // NOTE — distance-curved ma_sound_set_volume IS the right
        // approach for the AI-hearing / BFS portal-graph propagation
        // system (a separate subsystem in AIHearingService that
        // operates on the legacy Dark Engine portal-graph propagation
        // model). That system runs entirely outside Steam Audio and
        // does its own per-source distance + door attenuation in the
        // BFS graph traversal. Don't confuse the two pipelines: BFS
        // is for AI listeners (NPCs hearing the player), Steam Audio
        // is for the player listener (hearing the world).
        //
        // amb.attenuationFactor (P$SchAttFac) is plumbed into Steam
        // Audio's INVERSEDISTANCE distance model at voice spawn (see
        // voiceSetAttenuationFactor call earlier). Not re-read per-
        // frame.
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

} // namespace Darkness
