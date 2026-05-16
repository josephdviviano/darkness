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

#ifndef __AIHEARINGSERVICE_H
#define __AIHEARINGSERVICE_H

#include "DarknessService.h"
#include "DarknessServiceFactory.h"
#include "ServiceCommon.h"
#include "audio/AIHearingData.h"
#include "audio/AudioService.h"

#include <cstdint>
#include <string>

namespace Darkness {

/**
 *  @brief AI hearing subsystem — observation-only first pass.
 *
 *  Subscribes to AudioService's sound-emission events and, for each emission,
 *  walks every AI archetype/concrete object that has a P$AI_Hearin property,
 *  computes whether that AI *would* hear the sound, and emits a structured
 *  log line. Pure observation — no AI behavior plumbing yet.
 *
 *  The audibility decision combines three data sources:
 *    - The AI's per-archetype hearing rating (0..5) from P$AI_Hearin.
 *    - The global AIHearingStats table (dist_muls + db_adds per rating),
 *      loaded from the AIHearStat chunk via AudioService::getAIHearingStats.
 *    - The global AISoundTweaks table (defaultRanges per sound type), loaded
 *      from the AISNDTWK chunk via AudioService::getAISoundTweaks.
 *
 *  When either global chunk is missing the service falls back to
 *  kDefaultAIHearingStats and a zero-extended AISoundTweaks (the runtime
 *  announces the fallback once at bootstrap; see implementation).
 */
class AIHearingService : public ServiceImpl<AIHearingService> {
public:
    AIHearingService(ServiceManager *manager, const std::string &name);
    ~AIHearingService() override;

    /// Pure free function exposed for unit testing. Given the global tables
    /// and an emission's (soundType, baseRange, distance) plus an AI rating,
    /// decide audibility using a simple in-range check:
    ///
    ///   effective_range = baseRange * dist_muls[rating]
    ///                   + max(0, defaultRanges[soundType])  // type tweak
    ///   audible        = (distance <= effective_range)
    ///                 && (rating != AI_HEARING_DEAF)
    ///
    /// This first pass deliberately ignores the db_adds gain curve — the
    /// gain semantics are subtle and will land in a follow-up. Distance-
    /// based gating already captures the bulk of the original engine's
    /// hearing behavior.
    static bool isAudible(const AIHearingStats &stats,
                          const AISoundTweaks &tweaks,
                          int rating,
                          int soundType,
                          float baseRange,
                          float distance);

protected:
    bool init() override;
    void bootstrapFinished() override;
    void shutdown() override;

private:
    /// Event handler — invoked synchronously by AudioService whenever a
    /// new sound emission is published.
    void onSoundEmitted(const SoundEmissionEvent &ev);

    /// Cached service handles (acquired in bootstrapFinished).
    AudioServicePtr     mAudioService;
    PropertyServicePtr  mPropertyService;
    ObjectServicePtr    mObjectService;

    /// Global hearing tables, cached at bootstrap time. The tables don't
    /// change after the gamesys is loaded, so caching avoids the per-event
    /// AudioService → memcpy round-trip. Falls back to kDefaultAIHearingStats
    /// and a zero-init AISoundTweaks when the chunks are missing.
    AIHearingStats mStats{};
    AISoundTweaks  mTweaks{};
    bool           mHasStats  = false;
    bool           mHasTweaks = false;

    /// Throttling counter for "heard=no" log lines. We only log a miss for
    /// every Nth AI to keep MISS6 output manageable (~80 AIs × N emissions
    /// per second would saturate the log otherwise).
    uint64_t mMissCounter = 0;

    /// Log every Nth miss. "heard=yes" lines are always emitted.
    static constexpr uint64_t kMissLogStride = 64;
};

/// Factory for AIHearingService.
class AIHearingServiceFactory : public ServiceFactory {
public:
    AIHearingServiceFactory();
    ~AIHearingServiceFactory() override {}

    Service *createInstance(ServiceManager *manager) override;
    const std::string &getName() override;
    const uint getMask() override;
    const size_t getSID() override;

private:
    static const std::string mName;
};

} // namespace Darkness

#endif // __AIHEARINGSERVICE_H
