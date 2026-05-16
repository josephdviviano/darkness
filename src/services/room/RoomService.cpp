/******************************************************************************
 *
 *    This file is part of openDarkEngine project
 *    Copyright (C) 2005-2009 openDarkEngine team
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
 *	  $Id$
 *
 *****************************************************************************/

#include "RoomService.h"
#include <algorithm>
#include <cstdio>
#include <limits>
#include <queue>
#include <string>
#include <unordered_set>
#include "FileGroup.h"
#include "DarknessException.h"
#include "DarknessServiceManager.h"
#include "Room.h"
#include "RoomPortal.h"
#include "ServiceCommon.h"
#include "database/DatabaseService.h"
#include "logger.h"

namespace Darkness {

/*----------------------------------------------------*/
/*-------------------- RoomService -------------------*/
/*----------------------------------------------------*/
template <> const size_t ServiceImpl<RoomService>::SID = __SERVICE_ID_ROOM;

RoomService::RoomService(ServiceManager *manager, const std::string &name)
    : ServiceImpl<Darkness::RoomService>(manager, name), mRoomsOk(false) {}

//------------------------------------------------------
RoomService::~RoomService() { clear(); }

//------------------------------------------------------
Room *RoomService::getRoomByID(int32_t id) { return mRoomsByID[id]; }

//------------------------------------------------------
Room *RoomService::findObjRoom(size_t idset, int objID, const Vector3 &pos) {
    Room *r = roomFromPoint(pos);

    if (r) {
        // re-assign the object
        _detachObjRoom(idset, objID, NULL);
        _attachObjRoom(idset, objID, r);
    }

    return r;
}

//------------------------------------------------------
Room *RoomService::roomFromPoint(const Vector3 &pos) {
    // Collect all rooms containing this point (OBBs can overlap at portals).
    // The original Dark Engine handles up to 8 overlapping rooms and uses
    // portal-plane disambiguation to find the correct one.
    Room *candidates[8];
    int numCandidates = 0;

    for (const std::unique_ptr<Room> &r : mRooms) {
        if (r->obbContains(pos)) {
            if (numCandidates < 8)
                candidates[numCandidates++] = r.get();
        }
    }

    if (numCandidates == 0)
        return nullptr;
    if (numCandidates == 1)
        return candidates[0];

    // Disambiguate overlapping rooms by portal planes.
    // For each pair of candidate rooms, find the portal between them and
    // check which side of the portal plane the point falls on. The room
    // on the point's side wins; the other is eliminated.
    // Pairwise elimination: for each pair, find the shared portal and use
    // its plane to determine which room the point belongs to.
    for (int i = 0; i < numCandidates; ++i) {
        if (!candidates[i]) continue;
        for (int j = i + 1; j < numCandidates; ++j) {
            if (!candidates[j]) continue;

            Room *roomA = candidates[i];
            Room *roomB = candidates[j];
            for (uint32_t p = 0; p < roomA->getPortalCount(); ++p) {
                RoomPortal *portal = roomA->getPortal(p);
                if (!portal) continue;
                if (portal->getFarRoom() == roomB) {
                    // Portal plane separates the two rooms.
                    //
                    // CONVENTION (per Dark Engine): the portal plane's
                    // positive side is the FAR room side. A point with
                    // positive getDistance is on the OTHER side of the
                    // portal from the owner — i.e. inside the far room.
                    //
                    // So: positive distance → eliminate the OWNER (roomA),
                    // keep the far room (roomB). Negative distance →
                    // eliminate the far room, keep the owner.
                    //
                    // An earlier version of this code had the opposite
                    // polarity. That made the entire 3-4-ft-thick OBB
                    // overlap region around every portal resolve to the
                    // wrong room, which in turn made BFS pick a longer
                    // path through that region than the listener's
                    // actual geometric position warranted. As the
                    // listener walked through the overlap, the
                    // anchor-projected pathDist jumped by ~17 ft and
                    // ambient volume stepped by 2-3 dB — perceived as
                    // "the sound drops dramatically the moment I cross
                    // into another room." Fixing the polarity makes the
                    // overlap region resolve to the same room the
                    // listener was last firmly in, eliminating the step.
                    float dist = portal->getPlane().getDistance(pos);
                    if (dist >= 0.0f) {
                        candidates[i] = nullptr;  // eliminate roomA (owner / near)
                    } else {
                        candidates[j] = nullptr;  // eliminate roomB (far)
                    }
                    break;
                }
            }
            // If roomA was eliminated, stop testing it against further rooms
            if (!candidates[i]) break;
        }
    }

    // Return the first surviving candidate
    for (int i = 0; i < numCandidates; ++i) {
        if (candidates[i])
            return candidates[i];
    }
    return nullptr;
}

//------------------------------------------------------
Room *RoomService::updatedRoom(Room * /*cached*/, const Vector3 &pos) {
    // Today this is a one-line forward to roomFromPoint. The `cached`
    // argument is reserved for a future spatial-cache optimization
    // (memoize by quantized position, evict on first miss). Until then,
    // computing the disambiguated answer from scratch every frame is
    // the only way to guarantee correctness when room OBBs overlap.
    //
    // Cost: O(numRooms × 6 plane tests) + a small pairwise portal pass.
    // On Thief 2 mission scales (~260 rooms) this is ~3 µs/call — fully
    // affordable at per-frame rates for the listener and every voice.
    return roomFromPoint(pos);
}

//------------------------------------------------------
void RoomService::updateObjRoom(size_t idset, int objID, const Vector3 &pos) {
    // Refresh the object's room membership via the canonical query.
    // The previous implementation early-outed on `r->isInside(pos)`, which
    // is the raw OBB test and returns true for *every* room whose OBB
    // contains `pos` — so once an object sat in a portal-overlap region,
    // the cached pointer never updated even after it physically left.
    // Routing through updatedRoom forces a disambiguated answer each call.
    Room *prev = getCurrentObjRoom(idset, objID);
    Room *curr = updatedRoom(prev, pos);

    if (curr == prev)
        return;

    if (prev) {
        _detachObjRoom(idset, objID, prev);
        // TODO: Broadcast the transition
    }
    if (curr) {
        _attachObjRoom(idset, objID, curr);
        // TODO: Broadcast the obj. entered
    }
}

//------------------------------------------------------
bool RoomService::init() { return true; }

//------------------------------------------------------
void RoomService::bootstrapFinished() {
    mDbService = GET_SERVICE(DatabaseService);
    mDbService->registerListener(this, DBP_ROOM);
}

//------------------------------------------------------
void RoomService::shutdown() {
    if (mDbService) {
        mDbService->unregisterListener(this);
        mDbService.reset();
    }
}

//------------------------------------------------------
void RoomService::clear() {
    mRoomsByID.clear();
    mRooms.clear();
    mRoomsOk = false;
}

//------------------------------------------------------
void RoomService::onDBLoad(const FileGroupPtr &db, uint32_t curmask) {
    LOG_INFO("RoomService::onDBLoad called.");

    if (!(curmask & DBM_OBJTREE_CONCRETE)) // May not be it but seems to fit
        return;

    // to be sure
    clear();

    if (!db->hasFile("ROOM_DB")) {
        LOG_FATAL("RoomService: Database '%d' should contain ROOM_DB by mask, "
                  "but doesn't",
                  db->getName().c_str());
        return;
    }

    // load rooms Ok flag, then room count
    uint32_t roomsOk;
    uint32_t count;

    FilePtr rdb = db->getFile("ROOM_DB");

    *rdb >> roomsOk;

    if (!roomsOk) {
        LOG_ERROR("RoomService: Database '%d' had RoomOK false",
                  db->getName().c_str());
        mRoomsOk = false;
        return;
    }

    mRoomsOk = true;

    *rdb >> count;

    LOG_INFO("RoomService: ROOM_DB contains %u rooms", count);

    // construct array of rooms as needed
    mRooms.resize(count);

    // Two phase load... first we construct them
    for (size_t rn = 0; rn < count; ++rn)
        mRooms[rn].reset(new Room(this));

    // Phase 2 - read every room's data. Portals capture src/dest room IDs
    // but do NOT yet resolve them to Room pointers (those rooms may not
    // be in mRoomsByID yet — see RoomPortal::read).
    for (size_t rn = 0; rn < count; ++rn) {
        LOG_DEBUG("RoomService: Loading room %d", rn);
        std::unique_ptr<Room> &r = mRooms[rn];
        assert(r);
        r->read(rdb);
        mRoomsByID[r->getRoomID()] = r.get(); // weak ptrs here
    }

    // Phase 3 - now that every Room is registered in mRoomsByID, link
    // each portal's src/dest pointers. Doing this inline during read()
    // silently nulled out every forward reference (room with lower index
    // pointing to a room with higher index), which collapsed the entire
    // room-portal graph: cross-room sound propagation BFS could never
    // leave the source room, all cross-room voices got muted as
    // unreachable.
    for (size_t rn = 0; rn < count; ++rn) {
        mRooms[rn]->linkPortals();
    }

    // Diagnostic: dump every room's portal connectivity to the log
    // after linkPortals() resolves the pointers. Format:
    //   [ROOM_GRAPH] Room <id> center=(x,y,z) → [n1, n2, ...]
    // Use this to verify that the parsed portal graph matches what the
    // level designer intended — e.g. that a room you expect to be a
    // neighbor of another actually is. Useful for chasing parse or
    // disambiguation bugs that lead to BFS failures.
    {
        LOG_INFO("RoomService: connectivity dump (room → neighbors)");
        for (size_t rn = 0; rn < count; ++rn) {
            Room *r = mRooms[rn].get();
            if (!r) continue;
            std::string neighbors;
            uint32_t pc = r->getPortalCount();
            for (uint32_t pi = 0; pi < pc; ++pi) {
                RoomPortal *portal = r->getPortal(pi);
                if (!portal) continue;
                Room *dst = portal->getFarRoom();
                if (!dst) {
                    if (!neighbors.empty()) neighbors += ", ";
                    neighbors += "NULL";
                    continue;
                }
                if (!neighbors.empty()) neighbors += ", ";
                neighbors += std::to_string(dst->getRoomID());
            }
            Vector3 c = r->getCenter();
            LOG_INFO("[ROOM_GRAPH] Room %d center=(%.1f,%.1f,%.1f) "
                     "portals=%u → [%s]",
                     r->getRoomID(), c.x, c.y, c.z, pc,
                     neighbors.empty() ? "(none)" : neighbors.c_str());
        }
    }
}

//------------------------------------------------------
void RoomService::onDBSave(const FileGroupPtr &db, uint32_t tgtmask) {
    LOG_INFO("RoomService::onDBSave called.");

    if (!(tgtmask & DBM_MIS_DATA))
        return;

    uint32_t roomsOk = mRoomsOk ? 1 : 0;
    uint32_t count = mRooms.size();

    if (!mRoomsOk)
        count = 0;

    FilePtr rdb = db->getFile("ROOM_DB");
    *rdb << roomsOk << count;

    if (!mRoomsOk)
        return;

    for (size_t rn = 0; rn < count; ++rn) {
        mRooms[rn]->write(rdb);
    }
}

//------------------------------------------------------
void RoomService::onDBDrop(uint32_t dropmask) {
    LOG_INFO("RoomService::onDBDrop called.");

    if (!(dropmask & DBM_MIS_DATA))
        return;

    clear();
}

//------------------------------------------------------
void RoomService::_attachObjRoom(size_t idset, int objID, Room *room) {
    assert(room);
    LOG_VERBOSE("RoomService: IDset %u Object %d attaching (new room %d)",
                idset, objID, (int)room->getRoomID());
    room->attachObj(idset, objID);
    setCurrentObjRoom(idset, objID, room);
    // TODO: listener notification
}

//------------------------------------------------------
void RoomService::_detachObjRoom(size_t idset, int objID, Room *current) {
    if (!current)
        current = getCurrentObjRoom(idset, objID);

    if (current) {
        LOG_VERBOSE("RoomService: IDset %u Object %d detaching (old room %d)",
                    idset, objID, (int)current->getRoomID());

        current->detachObj(idset, objID);

        if (mIDSets.size() > idset) {
            mIDSets[idset].erase(objID);
        }
    }
    // TODO: listener notification
}

//------------------------------------------------------
Room *RoomService::getCurrentObjRoom(size_t idset, int objID) const {
    // get the idset in question
    if (mIDSets.size() > idset) {
        RoomsByID::const_iterator it = mIDSets[idset].find(objID);

        if (it != mIDSets[idset].end())
            return it->second;
    }

    return NULL;
}

//------------------------------------------------------
void RoomService::setCurrentObjRoom(size_t idset, int objID, Room *room) {
    // ensure the idset exists
    if (mIDSets.size() <= idset) {
        mIDSets.resize(idset + 1);
    }

    mIDSets[idset][objID] = room;
}

//------------------------------------------------------
SoundPropInfo RoomService::propagateSoundPath(const Vector3 &sourcePos,
                                               const Vector3 &listenerPos,
                                               Room *sourceRoom,
                                               Room *listenerRoom,
                                               const SoundPropParams &params) const
{
    SoundPropInfo result;
    const float maxDist = params.maxDist;

    // Helper: clear path output (only appended on a successful propagation)
    if (params.pathOut)
        params.pathOut->clear();

    if (!sourceRoom || !listenerRoom) {
        // Both rooms should be resolved by the caller's nearest-room
        // fallback. If we still have NULL here (no rooms loaded at all),
        // use euclidean distance as a last resort.
        float dist = glm::length(listenerPos - sourcePos);
        if (dist <= maxDist) {
            result.reached = true;
            result.realDistance = dist;
            result.effectiveDistance = dist;
            result.totalBlocking = 0.0f;
            result.virtualPosition = sourcePos;
            if (params.pathOut) {
                SoundPathHop hop{};
                hop.roomID = sourceRoom ? sourceRoom->getRoomID()
                          : (listenerRoom ? listenerRoom->getRoomID() : -1);
                hop.segmentDist = 0.0f;
                hop.cumRealDist = dist;
                hop.cumEffDist  = dist;
                hop.loudRoom    = 1.0f;
                hop.enterPortalCenter = sourcePos;
                params.pathOut->push_back(hop);
            }
        }

        static int sFailCount = 0;
        if (sFailCount < 5) {
            std::fprintf(stderr,
                "[PORTAL] propagateSoundPath: NULL room after fallback "
                "src=(%.1f,%.1f,%.1f)→%s lst=(%.1f,%.1f,%.1f)→%s dist=%.1f\n",
                sourcePos.x, sourcePos.y, sourcePos.z,
                sourceRoom ? "OK" : "NULL",
                listenerPos.x, listenerPos.y, listenerPos.z,
                listenerRoom ? "OK" : "NULL", dist);
            if (sFailCount == 0) {
                std::fprintf(stderr, "[PORTAL] %zu rooms loaded.\n", mRooms.size());
                float bestDist = 1e9f;
                Room *bestRoom = nullptr;
                for (auto &rp : mRooms) {
                    if (!rp) continue;
                    float d = glm::length(rp->getCenter() - listenerPos);
                    if (d < bestDist) { bestDist = d; bestRoom = rp.get(); }
                }
                if (bestRoom) {
                    auto c = bestRoom->getCenter();
                    std::fprintf(stderr,
                        "[PORTAL] Closest room %d: center=(%.1f,%.1f,%.1f) dist=%.1f\n",
                        bestRoom->getRoomID(), c.x, c.y, c.z, bestDist);
                    const Plane *planes = bestRoom->getBoundingPlanes();
                    for (int p = 0; p < 6; ++p) {
                        float dst = planes[p].getDistance(listenerPos);
                        std::fprintf(stderr,
                            "[PORTAL]   plane[%d]: n=(%.3f,%.3f,%.3f) d=%.3f "
                            "dist_to_listener=%.3f %s\n",
                            p, planes[p].normal.x, planes[p].normal.y, planes[p].normal.z,
                            planes[p].d, dst, dst < 0 ? "OUTSIDE" : "inside");
                    }
                }
            }
            sFailCount++;
        }
        return result;
    }

    // Same room — direct line of sight, no portal traversal needed
    if (sourceRoom == listenerRoom) {
        float dist = glm::length(listenerPos - sourcePos);
        if (dist <= maxDist) {
            result.reached = true;
            result.realDistance = dist;
            result.effectiveDistance = dist;
            result.totalBlocking = 0.0f;
            result.virtualPosition = sourcePos;
            if (params.pathOut) {
                SoundPathHop hop{};
                hop.roomID = sourceRoom->getRoomID();
                hop.segmentDist = 0.0f;
                hop.cumRealDist = dist;
                hop.cumEffDist = dist;
                hop.loudRoom = 1.0f;
                hop.enterPortalCenter = sourcePos;
                params.pathOut->push_back(hop);
            }
        }
        return result;
    }

    // ── Dijkstra BFS through room portal graph ──
    // Uses the precomputed portal-to-portal distance matrix within each
    // room (from ROOM_DB) instead of euclidean entry-point-to-portal
    // distances. Tracks enter-portal indices for matrix lookup and path
    // reconstruction. Based on the Dark Engine's cRoomPropAgent::PropagateBF
    // algorithm.
    struct BFSEntry {
        Room    *room;
        float    realDist;
        float    effectiveDist;
        float    cumulativeTransmission;     // door × LoudRoom
        float    cumulativeDoorTransmission; // door only — drives the LPF cutoff
        int32_t  enterPortalIdx;             // portal we entered through (-1 = source)
        Room    *prevRoom;                   // BFS predecessor
        Vector3  enterPortalCenter;          // portal center, or sourcePos for source room

        bool operator>(const BFSEntry &o) const {
            return effectiveDist > o.effectiveDist;
        }
    };

    std::priority_queue<BFSEntry, std::vector<BFSEntry>, std::greater<BFSEntry>> pq;
    thread_local std::unordered_map<int32_t, float> bestDist;
    bestDist.clear();
    thread_local std::unordered_map<int32_t, BFSEntry> reachedFrom;
    reachedFrom.clear();

    const auto &blockingFn = params.doorBlocking;
    const auto &loudFn     = params.loudRoom;

    BFSEntry start{};
    start.room = sourceRoom;
    start.realDist = 0.0f;
    start.effectiveDist = 0.0f;
    start.cumulativeTransmission     = 1.0f;
    start.cumulativeDoorTransmission = 1.0f;
    start.enterPortalIdx = -1;
    start.prevRoom = nullptr;
    start.enterPortalCenter = sourcePos;
    pq.push(start);
    bestDist[sourceRoom->getRoomID()] = 0.0f;

    BFSEntry listenerEntry{};
    bool foundListener = false;

    while (!pq.empty()) {
        BFSEntry cur = pq.top();
        pq.pop();

        auto it = bestDist.find(cur.room->getRoomID());
        if (it != bestDist.end() && cur.effectiveDist > it->second)
            continue;

        reachedFrom[cur.room->getRoomID()] = cur;

        if (cur.room == listenerRoom) {
            listenerEntry = cur;
            foundListener = true;
            break;
        }

        uint32_t portalCount = cur.room->getPortalCount();
        for (uint32_t i = 0; i < portalCount; ++i) {
            if (static_cast<int32_t>(i) == cur.enterPortalIdx)
                continue;

            RoomPortal *portal = cur.room->getPortal(i);
            if (!portal) continue;

            Room *nextRoom = portal->getFarRoom();
            if (!nextRoom) continue;

            float segDist;
            if (cur.enterPortalIdx >= 0) {
                segDist = cur.room->getPortalDist(i, static_cast<uint32_t>(cur.enterPortalIdx));
            } else {
                segDist = glm::length(portal->getCenter() - sourcePos);
            }
            float newRealDist = cur.realDist + segDist;

            float blocking = blockingFn ? blockingFn(cur.room->getRoomID(),
                                                     nextRoom->getRoomID())
                                        : 0.0f;
            float newEffDist = cur.effectiveDist + segDist;
            if (blocking > 0.0f && newEffDist < maxDist) {
                newEffDist += (maxDist - newEffDist) * blocking;
            }

            if (newEffDist > maxDist)
                continue;

            float doorTransmissionFactor = (1.0f - blocking);
            float newDoorTransmission = cur.cumulativeDoorTransmission * doorTransmissionFactor;
            float newTransmission     = cur.cumulativeTransmission     * doorTransmissionFactor;

            float loud = loudFn ? loudFn(nextRoom->getRoomID()) : 1.0f;
            newTransmission *= loud;

            int32_t nextID = nextRoom->getRoomID();
            auto bestIt = bestDist.find(nextID);
            if (bestIt != bestDist.end() && newEffDist >= bestIt->second)
                continue;
            bestDist[nextID] = newEffDist;

            int32_t farPortalIdx = -1;
            int32_t destPortalID = portal->getDestPortalID();
            for (uint32_t j = 0; j < nextRoom->getPortalCount(); ++j) {
                RoomPortal *p = nextRoom->getPortal(j);
                if (p && p->getPortalID() == destPortalID) {
                    farPortalIdx = static_cast<int32_t>(j);
                    break;
                }
            }

            BFSEntry next{};
            next.room = nextRoom;
            next.realDist = newRealDist;
            next.effectiveDist = newEffDist;
            next.cumulativeTransmission     = newTransmission;
            next.cumulativeDoorTransmission = newDoorTransmission;
            next.enterPortalIdx = farPortalIdx;
            next.prevRoom = cur.room;
            next.enterPortalCenter = portal->getCenter();
            pq.push(next);
        }
    }

    if (!foundListener) {
        // BFS exhausted the queue without reaching the listener's room.
        // Either the source and listener rooms are in disconnected portal
        // graph components, or every path inflated effectiveDist past
        // maxDist via accumulated door blocking. Log loudly per the
        // "no silent fallbacks" principle.
        int32_t srcID = sourceRoom->getRoomID();
        int32_t lstID = listenerRoom->getRoomID();
        auto bestIt = bestDist.find(lstID);
        float bestToListener = (bestIt != bestDist.end()) ? bestIt->second : -1.0f;
        thread_local std::unordered_set<uint64_t> reportedPairs;
        uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(srcID)) << 32)
                       | static_cast<uint32_t>(lstID);
        if (reportedPairs.insert(key).second) {
            std::fprintf(stderr,
                "[FALLBACK] PROP_FAIL src=(%.1f,%.1f,%.1f)→rm%d "
                "lst=(%.1f,%.1f,%.1f)→rm%d visited=%zu maxDist=%.1f "
                "bestEffToLst=%.2f (%s) — caller will mute voice\n",
                sourcePos.x, sourcePos.y, sourcePos.z, srcID,
                listenerPos.x, listenerPos.y, listenerPos.z, lstID,
                reachedFrom.size(), maxDist, bestToListener,
                bestToListener < 0 ? "DISCONNECTED" : "MAXDIST_CUTOFF");
        }
        return result;
    }

    // ── Path reconstruction + anchor projection ──
    // Walk backward from listener room to source via prevRoom pointers to
    // build the portal chain. Then run a forward pass with raycast +
    // edge-projection to find the shortest geometric path that threads
    // through every portal opening. Based on the Dark Engine's
    // FindSoundPath algorithm.
    struct PortalInfo {
        RoomPortal *portal;
        Room       *fromRoom;
    };
    std::vector<PortalInfo> portalChain;
    {
        Room *walkRoom = listenerRoom;
        while (walkRoom != sourceRoom) {
            auto it = reachedFrom.find(walkRoom->getRoomID());
            if (it == reachedFrom.end()) break;
            const BFSEntry &entry = it->second;
            if (!entry.prevRoom) break;
            for (uint32_t i = 0; i < entry.prevRoom->getPortalCount(); ++i) {
                RoomPortal *p = entry.prevRoom->getPortal(i);
                if (p && p->getFarRoom() == walkRoom) {
                    portalChain.push_back({p, entry.prevRoom});
                    break;
                }
            }
            walkRoom = entry.prevRoom;
        }
        std::reverse(portalChain.begin(), portalChain.end());
    }

    struct Anchor {
        Vector3 pos;
        bool valid = false;
    };
    std::vector<Anchor> anchors(portalChain.size());

    Vector3 leadPt = sourcePos;
    for (size_t i = 0; i < portalChain.size(); ++i) {
        Vector3 target = (i + 1 < portalChain.size())
                         ? portalChain[i + 1].portal->getCenter()
                         : listenerPos;
        Vector3 dir = target - leadPt;
        if (!portalChain[i].portal->raycast(leadPt, dir)) {
            Vector3 projPt;
            if (portalChain[i].portal->getRaycastProjection(leadPt, dir, projPt)) {
                anchors[i].pos = projPt;
                anchors[i].valid = true;
                leadPt = projPt;
            }
        }
    }

    float pathDist = 0.0f;
    int lastAnchor = -1;
    Vector3 virtualPos = sourcePos;
    for (size_t i = 0; i < anchors.size(); ++i) {
        if (anchors[i].valid) {
            if (lastAnchor < 0) {
                pathDist += glm::length(anchors[i].pos - sourcePos);
            } else {
                pathDist += glm::length(anchors[i].pos - anchors[lastAnchor].pos);
            }
            lastAnchor = static_cast<int>(i);
            virtualPos = anchors[i].pos;
        }
    }
    if (lastAnchor < 0) {
        pathDist += glm::length(listenerPos - sourcePos);
        virtualPos = sourcePos;
    } else {
        pathDist += glm::length(listenerPos - anchors[lastAnchor].pos);
        virtualPos = anchors[lastAnchor].pos;
    }

    // Apply blocking to path distance (Dark Engine munged-distance formula).
    // totalBlocking = 1 - cumulativeTransmission. Includes both door
    // blocking and LoudRoom multipliers — both inflate the *perceived*
    // distance for volume-attenuation purposes.
    float totalBlocking = 1.0f - listenerEntry.cumulativeTransmission;
    float effDist = pathDist;
    if (totalBlocking > 0.0f && effDist < maxDist) {
        effDist += (maxDist - effDist) * totalBlocking;
    }
    // Door-only blocking drives the LPF cutoff downstream. LoudRoom is
    // excluded so sound passing between LoudRoom-modified subdivisions of
    // one open space gets quieter but not muffled.
    float doorBlocking = 1.0f - listenerEntry.cumulativeDoorTransmission;

    if (effDist <= maxDist) {
        result.reached = true;
        result.realDistance = pathDist;
        result.effectiveDistance = effDist;
        result.totalBlocking = totalBlocking;
        result.doorBlocking  = doorBlocking;
        result.virtualPosition = virtualPos;
    }

    // ── Optional path-detail emission (diagnostic) ──
    if (params.pathOut && result.reached) {
        // Walk the reachedFrom chain forward from source to listener.
        std::vector<BFSEntry> chain;
        Room *walkRoom = listenerRoom;
        while (walkRoom) {
            auto it = reachedFrom.find(walkRoom->getRoomID());
            if (it == reachedFrom.end()) break;
            chain.push_back(it->second);
            if (it->second.prevRoom == nullptr || walkRoom == sourceRoom) break;
            walkRoom = it->second.prevRoom;
        }
        std::reverse(chain.begin(), chain.end());

        for (size_t i = 0; i < chain.size(); ++i) {
            const BFSEntry &entry = chain[i];
            SoundPathHop hop{};
            hop.roomID = entry.room->getRoomID();
            hop.cumRealDist = entry.realDist;
            hop.cumEffDist  = entry.effectiveDist;
            hop.enterPortalCenter = entry.enterPortalCenter;

            // Per-hop door blocking / LoudRoom: derived from the ratio of
            // cumulative transmission factors between this hop and the
            // previous. The source room (i == 0) has no entry portal.
            if (i == 0) {
                hop.doorBlocking = 0.0f;
                hop.loudRoom     = 1.0f;
                hop.segmentDist  = 0.0f;
            } else {
                const BFSEntry &prev = chain[i - 1];
                float doorRatio = (prev.cumulativeDoorTransmission > 1e-9f)
                                  ? (entry.cumulativeDoorTransmission / prev.cumulativeDoorTransmission)
                                  : 0.0f;
                float combinedRatio = (prev.cumulativeTransmission > 1e-9f)
                                      ? (entry.cumulativeTransmission / prev.cumulativeTransmission)
                                      : 0.0f;
                hop.doorBlocking = 1.0f - doorRatio;
                hop.loudRoom = (doorRatio > 1e-9f) ? (combinedRatio / doorRatio) : 1.0f;
                hop.segmentDist = entry.realDist - prev.realDist;
            }
            params.pathOut->push_back(hop);
        }
    }

    return result;
}

//-------------------------- Factory implementation
const std::string RoomServiceFactory::mName = "RoomService";

RoomServiceFactory::RoomServiceFactory() : ServiceFactory() {}

const std::string &RoomServiceFactory::getName() { return mName; }

const uint RoomServiceFactory::getMask() { return SERVICE_ENGINE; }

const size_t RoomServiceFactory::getSID() { return RoomService::SID; }

Service *RoomServiceFactory::createInstance(ServiceManager *manager) {
    return new RoomService(manager, mName);
}

} // namespace Darkness
