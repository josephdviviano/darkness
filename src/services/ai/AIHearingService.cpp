/******************************************************************************
 *
 *    This file is part of the darkness project
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

#include "AIHearingService.h"

#include "DarknessServiceManager.h"
#include "audio/AudioService.h"
#include "audio/AIHearingData.h"
#include "object/ObjectService.h"
#include "property/Property.h"
#include "property/PropertyService.h"
#include "property/TypedProperty.h"
#include "logger.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace Darkness {

template <> const size_t ServiceImpl<AIHearingService>::SID = __SERVICE_ID_AI_HEARING;

//------------------------------------------------------
AIHearingService::AIHearingService(ServiceManager *manager,
                                   const std::string &name)
    : ServiceImpl<AIHearingService>(manager, name)
{
}

//------------------------------------------------------
AIHearingService::~AIHearingService() {}

//------------------------------------------------------
bool AIHearingService::init()
{
    // No work in init — service handles are acquired in bootstrapFinished
    // so that AudioService is guaranteed to be fully constructed first.
    return true;
}

//------------------------------------------------------
void AIHearingService::bootstrapFinished()
{
    // Acquire service dependencies. AudioService is created after this
    // service in the renderer init order (see DarknessRenderInit.h), so
    // we must defer this acquisition until bootstrapFinished.
    mAudioService    = GET_SERVICE(AudioService);
    mPropertyService = GET_SERVICE(PropertyService);
    mObjectService   = GET_SERVICE(ObjectService);

    if (!mAudioService || !mPropertyService || !mObjectService) {
        LOG_ERROR("AIHearingService: missing dependency (audio=%p prop=%p obj=%p)",
                  (void *)mAudioService.get(),
                  (void *)mPropertyService.get(),
                  (void *)mObjectService.get());
        return;
    }

    // Cache the global hearing tables. These don't change after the gamesys
    // is loaded (P$AI_Hearin etc. are per-archetype, but the global
    // dist_muls / db_adds / defaultRanges tables come from singleton chunks).
    mHasStats  = mAudioService->getAIHearingStats(mStats);
    mHasTweaks = mAudioService->getAISoundTweaks(mTweaks);

    // Fallback announcements — loud, single-line per the project's no-silent-
    // fallback policy. Use kDefaultAIHearingStats from AIHearingData.h.
    if (!mHasStats) {
        std::fprintf(stderr,
            "[FALLBACK] AIHearingService: AIHearStat chunk not loaded — "
            "using kDefaultAIHearingStats (Deaf=0x dist, Normal=1.0x dist, "
            "VeryHigh=3.0x dist).\n");
        mStats = kDefaultAIHearingStats;
    }
    if (!mHasTweaks) {
        std::fprintf(stderr,
            "[FALLBACK] AIHearingService: AISNDTWK chunk not loaded — "
            "using zero-init default sound-type ranges.\n");
        // Zero-init defaults — AISoundTweaks has no kDefault counterpart in
        // the data header, so we just leave it zeroed.
        for (int i = 0; i < AI_SOUND_TYPE_COUNT; ++i)
            mTweaks.defaultRanges[i] = 0;
    }

    // Subscribe to sound emission events. The handler captures `this` by raw
    // pointer; the service outlives the AudioService here (AudioService
    // un-publishes on shutdown), so capture-by-pointer is safe.
    mAudioService->registerSoundEmissionListener(
        [this](const SoundEmissionEvent &ev) { onSoundEmitted(ev); });

    LOG_INFO("AIHearingService: bootstrapped (stats=%s, tweaks=%s)",
             mHasStats  ? "loaded" : "DEFAULT",
             mHasTweaks ? "loaded" : "DEFAULT");
}

//------------------------------------------------------
void AIHearingService::shutdown()
{
    mAudioService.reset();
    mPropertyService.reset();
    mObjectService.reset();
}

//------------------------------------------------------
bool AIHearingService::isAudible(const AIHearingStats &stats,
                                 const AISoundTweaks &tweaks,
                                 int rating,
                                 int soundType,
                                 float baseRange,
                                 float distance)
{
    // Deaf AIs (rating slot 0) never hear anything — matches the original
    // engine's hearing-rating semantics (dist_muls[0] = 0).
    if (rating == AI_HEARING_DEAF)
        return false;

    // Clamp inputs to valid table indices so the caller doesn't have to.
    if (rating    < 0 || rating    >= AI_HEARING_COUNT)   return false;
    if (soundType < 0 || soundType >= AI_SOUND_TYPE_COUNT) soundType = 0;

    // Effective audible range. The rating multiplier scales the emitter's
    // authored range; the sound-type tweak is added on top to capture the
    // global "Combat sounds carry farther than Untyped" intuition. We treat
    // negative type tweaks as zero — the original engine's default-range
    // table is non-negative for the slots we ship.
    float typeBonus = static_cast<float>(tweaks.defaultRanges[soundType]);
    if (typeBonus < 0.0f) typeBonus = 0.0f;

    float effectiveRange = baseRange * stats.dist_muls[rating] + typeBonus;

    return distance <= effectiveRange;
}

//------------------------------------------------------
void AIHearingService::onSoundEmitted(const SoundEmissionEvent &ev)
{
    if (!mPropertyService || !mObjectService)
        return;

    // Lazy refresh of the global tables. bootstrapFinished runs before
    // AudioService::onDBLoad parses the AIHearStat / AISNDTWK chunks,
    // so on the first emission we may still be on default values. Try
    // again here — once the chunks have been loaded the AudioService
    // getters return true and we lock the tables in.
    if (!mHasStats && mAudioService) {
        if (mAudioService->getAIHearingStats(mStats)) {
            mHasStats = true;
            LOG_INFO("AIHearingService: AIHearStat chunk now available — "
                     "switching from defaults to mission-supplied table");
        }
    }
    if (!mHasTweaks && mAudioService) {
        if (mAudioService->getAISoundTweaks(mTweaks)) {
            mHasTweaks = true;
            LOG_INFO("AIHearingService: AISNDTWK chunk now available — "
                     "switching from defaults to mission-supplied table");
        }
    }

    // Enumerate every concrete AI: walk all objects with P$Position and
    // filter to those that have P$AI_Hearin via inheritance. P$AI_Hearin
    // itself is per-archetype in dark.gam, so getAllObjectsWithProperty
    // returns archetypes (negative IDs) without world positions. Concrete
    // mission AIs (positive IDs with P$Position) inherit AI_Hearin through
    // the MetaProp chain, and hasProperty() resolves that chain.
    Property *hearinProp = mPropertyService->getProperty("AI_Hearin");
    if (!hearinProp)
        return;

    auto positioned = getAllObjectsWithProperty(mPropertyService.get(), "Position");

    std::vector<int> aiIDs;
    aiIDs.reserve(positioned.size());
    for (int id : positioned) {
        // has() walks the archetype + MetaProp chain.
        if (hearinProp->has(id))
            aiIDs.push_back(id);
    }
    size_t concreteCount = aiIDs.size();

    // Also include archetype holders (for future archetype-level diagnostics,
    // though without positions they'll be filtered below).
    auto archetypeHolders =
        getAllObjectsWithProperty(mPropertyService.get(), "AI_Hearin");
    for (int id : archetypeHolders) {
        // De-dup — most won't be in `aiIDs` because archetypes don't have
        // P$Position, but we want belt-and-braces.
        if (std::find(aiIDs.begin(), aiIDs.end(), id) == aiIDs.end())
            aiIDs.push_back(id);
    }

    static bool firstCall = true;
    if (firstCall) {
        firstCall = false;
        LOG_INFO("AIHearingService: first emission — %zu AIs in scope "
                 "(concrete=%zu, archetype-only=%zu)",
                 aiIDs.size(), concreteCount,
                 aiIDs.size() - concreteCount);
    }

    auto &allRatingHolders = aiIDs;

    for (int aiID : allRatingHolders) {
        if (aiID == 0) continue;

        // Read the AI's hearing rating. P$AI_Hearin stores a single int32.
        // Use raw-data access since the property is one int (no struct).
        size_t sz = 0;
        const uint8_t *raw = getPropertyRawData(
            mPropertyService.get(), "AI_Hearin", aiID, sz);
        if (!raw || sz < sizeof(int32_t))
            continue;

        int32_t rating = 0;
        std::memcpy(&rating, raw, sizeof(int32_t));

        // Position lookup. ObjectService::position() returns (0,0,0) for
        // objects without P$Position — typically archetypes. Skip those:
        // they're abstract templates that don't exist in world space.
        Vector3 aiPos = mObjectService->position(aiID);
        bool isAbstract = (aiPos.x == 0.0f && aiPos.y == 0.0f && aiPos.z == 0.0f);
        if (isAbstract)
            continue;

        float d = glm::distance(aiPos, ev.position);

        // The effective range computation lives in the pure helper so it
        // can be unit-tested without standing up the service stack.
        bool heard = isAudible(mStats, mTweaks, rating, ev.soundType,
                               ev.baseRange, d);

        // Throttle log volume — every "heard=yes" line is interesting, but
        // misses can flood the log on a level with many AIs. Print every
        // Nth miss only.
        bool shouldLog = heard;
        if (!heard) {
            if ((mMissCounter++ % kMissLogStride) == 0)
                shouldLog = true;
        }
        if (!shouldLog) continue;

        // Re-derive effective range for the log line so the operator sees
        // both the threshold and the actual distance.
        float typeBonus = static_cast<float>(
            mTweaks.defaultRanges[ev.soundType >= 0 && ev.soundType < AI_SOUND_TYPE_COUNT
                                    ? ev.soundType : 0]);
        if (typeBonus < 0.0f) typeBonus = 0.0f;
        float effRange = ev.baseRange
                       * mStats.dist_muls[rating >= 0 && rating < AI_HEARING_COUNT ? rating : 0]
                       + typeBonus;

        std::string aiName = mObjectService->getName(aiID);
        if (aiName.empty())
            aiName = "<unnamed>";

        LOG_INFO("[AI_HEAR] schema=%s emitter=%d pos=(%.1f,%.1f,%.1f) "
                 "AI=%d(%s) rating=%d(%s) d=%.1f range=%.1f heard=%s",
                 ev.schemaName.c_str(), ev.emitterObjID,
                 ev.position.x, ev.position.y, ev.position.z,
                 aiID, aiName.c_str(),
                 rating, aiHearingRatingName(rating),
                 d, effRange,
                 heard ? "yes" : "no");
    }
}

/*---------------------- Factory ----------------------*/

const std::string AIHearingServiceFactory::mName = "AIHearingService";

AIHearingServiceFactory::AIHearingServiceFactory() : ServiceFactory() {}

const std::string &AIHearingServiceFactory::getName() { return mName; }

const uint AIHearingServiceFactory::getMask() { return SERVICE_ENGINE; }

const size_t AIHearingServiceFactory::getSID() { return AIHearingService::SID; }

Service *AIHearingServiceFactory::createInstance(ServiceManager *manager)
{
    return new AIHearingService(manager, mName);
}

} // namespace Darkness
