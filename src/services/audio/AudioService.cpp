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

#include "AudioService.h"
#include "ServiceCommon.h"
#include "DarknessServiceManager.h"
#include "database/DatabaseService.h"
#include "loop/LoopService.h"
#include "room/RoomService.h"
#include "property/PropertyService.h"
#include "logger.h"

namespace Darkness {

/*----------------------------------------------------*/
/*------------------ Audio Service -------------------*/
/*----------------------------------------------------*/
template <>
const size_t ServiceImpl<AudioService>::SID = __SERVICE_ID_AUDIO;

AudioService::AudioService(ServiceManager *manager, const std::string &name)
    : ServiceImpl<AudioService>(manager, name)
{
    // LoopClient definition — runs after input, before renderer
    mLoopClientDef.id = LOOPCLIENT_ID_AUDIO;
    mLoopClientDef.mask = LOOPMODE_INPUT | LOOPMODE_RENDER;
    mLoopClientDef.name = "AudioService";
    mLoopClientDef.priority = LOOPCLIENT_PRIORITY_AUDIO;
}

//------------------------------------------------------
AudioService::~AudioService()
{
}

//------------------------------------------------------
bool AudioService::init()
{
    return true;
}

//------------------------------------------------------
void AudioService::bootstrapFinished()
{
    // Acquire service dependencies
    mDbService = GET_SERVICE(DatabaseService);
    mRoomService = GET_SERVICE(RoomService);
    mPropertyService = GET_SERVICE(PropertyService);

    // Register as a database listener — load after rooms are ready
    mDbService->registerListener(this, DBP_AUDIO);

    // Register as a loop client for per-frame updates
    auto loopSvc = GET_SERVICE(LoopService);
    loopSvc->addLoopClient(this);

    LOG_INFO("AudioService: initialized");
}

//------------------------------------------------------
void AudioService::shutdown()
{
    haltAll();

    // Unregister loop client
    auto loopSvc = GET_SERVICE(LoopService);
    loopSvc->removeLoopClient(this);

    // Unregister database listener
    if (mDbService) {
        mDbService->unregisterListener(this);
        mDbService.reset();
    }

    mRoomService.reset();
    mPropertyService.reset();

    LOG_INFO("AudioService: shut down");
}

//------------------------------------------------------
void AudioService::onDBLoad(const FileGroupPtr &db, uint32_t curmask)
{
    if (!(curmask & DBM_MIS_DATA))
        return;

    // TODO (Task 34): Load and parse .sch schema files
    // TODO (Task 35): Build Steam Audio IPLScene from world geometry
    // TODO (Task 39): Load ambient sound properties (P$AmbientHack)

    LOG_INFO("AudioService: mission loaded (audio setup pending)");
}

//------------------------------------------------------
void AudioService::onDBSave(const FileGroupPtr &db, uint32_t tgtmask)
{
    // Audio state is not saved — nothing to do
}

//------------------------------------------------------
void AudioService::onDBDrop(uint32_t dropmask)
{
    if (!(dropmask & DBM_MIS_DATA))
        return;

    haltAll();
    mBlockingFactors.clear();
    mNextHandle = 0;

    // TODO (Task 35): Destroy Steam Audio scene
    // TODO (Task 34): Clear schema database

    LOG_INFO("AudioService: mission audio state cleared");
}

//------------------------------------------------------
void AudioService::loopStep(float deltaTime)
{
    // TODO (Task 36): Update active voices — check for finished sounds,
    //   update Steam Audio source positions for moving objects
    // TODO (Task 35): Run Steam Audio simulation step
}

// ── Public API stubs ──

//------------------------------------------------------
SoundHandle AudioService::playSchema(const std::string &schemaName,
                                     const Vector3 &position)
{
    // TODO (Task 36): Resolve schema, pick sample, start playback
    LOG_DEBUG("AudioService::playSchema('%s') — not yet implemented",
              schemaName.c_str());
    return SOUND_HANDLE_INVALID;
}

//------------------------------------------------------
SoundHandle AudioService::playSchemaOnObj(const std::string &schemaName,
                                          int objID)
{
    // TODO (Task 36): Resolve schema, attach to object, start playback
    LOG_DEBUG("AudioService::playSchemaOnObj('%s', %d) — not yet implemented",
              schemaName.c_str(), objID);
    return SOUND_HANDLE_INVALID;
}

//------------------------------------------------------
void AudioService::haltSound(SoundHandle handle)
{
    if (handle == SOUND_HANDLE_INVALID)
        return;

    // TODO (Task 36): Stop the specific active voice
    LOG_DEBUG("AudioService::haltSound(%d) — not yet implemented", handle);
}

//------------------------------------------------------
void AudioService::haltAll()
{
    // TODO (Task 36): Stop all active voices
    mNextHandle = 0;
}

//------------------------------------------------------
void AudioService::setBlockingFactor(int room1, int room2, float factor)
{
    // Store bidirectionally so lookup works in either direction
    uint32_t key1 = (static_cast<uint32_t>(room1) << 16) |
                     static_cast<uint32_t>(room2 & 0xFFFF);
    uint32_t key2 = (static_cast<uint32_t>(room2) << 16) |
                     static_cast<uint32_t>(room1 & 0xFFFF);

    if (factor <= 0.0f) {
        // Remove blocking (fully open)
        mBlockingFactors.erase(key1);
        mBlockingFactors.erase(key2);
    } else {
        mBlockingFactors[key1] = factor;
        mBlockingFactors[key2] = factor;
    }
}

//------------------------------------------------------
float AudioService::getBlockingFactor(int room1, int room2) const
{
    uint32_t key = (static_cast<uint32_t>(room1) << 16) |
                    static_cast<uint32_t>(room2 & 0xFFFF);
    auto it = mBlockingFactors.find(key);
    return (it != mBlockingFactors.end()) ? it->second : 0.0f;
}

/*---------------------- Factory ----------------------*/
const std::string AudioServiceFactory::mName = "AudioService";

AudioServiceFactory::AudioServiceFactory() : ServiceFactory() {}

const std::string &AudioServiceFactory::getName() { return mName; }

const uint AudioServiceFactory::getMask()
{
    return SERVICE_ENGINE | SERVICE_DATABASE_LISTENER;
}

const size_t AudioServiceFactory::getSID() { return AudioService::SID; }

Service *AudioServiceFactory::createInstance(ServiceManager *manager)
{
    return new AudioService(manager, mName);
}

} // namespace Darkness
