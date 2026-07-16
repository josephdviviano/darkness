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
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
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
    if (params.chainOut)
        params.chainOut->clear();

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
            // Synthesize a single-path record so multi-path consumers see
            // a consistent shape even on the euclidean fallback.
            SoundPathRecord rec;
            rec.effectiveDistance = dist;
            rec.realDistance      = dist;
            rec.totalBlocking     = 0.0f;
            rec.doorBlocking      = 0.0f;
            rec.virtualPosition   = sourcePos;
            rec.predecessorRoomID = -1;
            result.paths.push_back(rec);
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
            SoundPathRecord rec;
            rec.effectiveDistance = dist;
            rec.realDistance      = dist;
            rec.totalBlocking     = 0.0f;
            rec.doorBlocking      = 0.0f;
            rec.virtualPosition   = sourcePos;
            rec.predecessorRoomID = -1;
            result.paths.push_back(rec);
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

    // ── Dijkstra-style N-path BFS through room portal graph ──
    //
    // Hop cost is the distance between consecutive "lead points" — the
    // closest point on each portal polygon to the previous one, computed
    // via RoomPortal::closestPointOnPolygon. This replaces the legacy
    // portal-CENTER-to-CENTER heuristic that systematically overestimated
    // costs for large portals: when a level subdivides one open volume
    // into two rooms, the shared "portal" polygon is the entire shared
    // face. Its center can be far from the source even when the nearest
    // point on the polygon is right under the source. The center-based
    // BFS would then pick a chain that started with a smaller, closer
    // portal — and produce massive effective-distance inflation when
    // anchor projection then tried to thread the geometry through that
    // wrong chain.
    //
    // Generalizes the Dark Engine's dual-predecessor scheme
    // (a hardcoded two-path merge) to N configurable
    // paths; N=1 single-shortest, N=2 reproduces the original engine.
    struct BFSEntry {
        Room    *room;
        float    realDist;
        float    effectiveDist;
        float    cumulativeTransmission;     // door × LoudRoom
        float    cumulativeDoorTransmission; // door only — drives the LPF cutoff
        int32_t  enterPortalIdx;             // portal we entered through (-1 = source)
        Room    *prevRoom;                   // BFS predecessor — path-class distinctor
        Vector3  enterLeadPt;                // closest-point on entry portal to the
                                             // previous lead pt; sourcePos for source room
        Vector3  enterPortalCenter;          // portal CENTER (diagnostic only — pathOut emits this)

        bool operator>(const BFSEntry &o) const {
            return effectiveDist > o.effectiveDist;
        }
    };

    const uint32_t maxPaths    = std::max<uint32_t>(1u, std::min<uint32_t>(params.maxPaths, 4u));
    const float    maxPathDiff = params.maxPathDiff;

    std::priority_queue<BFSEntry, std::vector<BFSEntry>, std::greater<BFSEntry>> pq;
    // Per-room: up to maxPaths entries with DISTINCT prevRoom IDs, sorted
    // by effectiveDist ascending. Front is the primary (loudest) path;
    // alternates trail. The same prevRoom never appears twice — same-prev
    // duplicates collapse to the better one.
    thread_local std::unordered_map<int32_t, std::vector<BFSEntry>> reachedFrom;
    reachedFrom.clear();
    // Mirrors paths.front().effectiveDist for the cutoff comparison; kept
    // separately so the distance-difference filter is a single map lookup.
    thread_local std::unordered_map<int32_t, float> bestDist;
    bestDist.clear();

    const auto &blockingFn = params.doorBlocking;
    const auto &loudFn     = params.loudRoom;

    // Insertion helper. Returns true if cand was accepted into
    // reachedFrom[cand.room] — caller should then push it onto the PQ.
    // Rules:
    //   (1) Same-prev entry exists ⇒ keep the better effDist.
    //   (2) Different prev ⇒ accept if within maxPathDiff of the current
    //       primary AND either there's room (size < maxPaths) or it beats
    //       the worst entry.
    // After any change, paths are re-sorted and entries no longer within
    // maxPathDiff of the new primary are pruned.
    auto tryAccept = [&](const BFSEntry &cand) -> bool {
        int32_t rid = cand.room->getRoomID();
        int32_t candPrev = cand.prevRoom ? cand.prevRoom->getRoomID() : -1;
        auto &paths = reachedFrom[rid];

        bool changed = false;
        bool sameFound = false;
        for (auto &existing : paths) {
            int32_t existPrev = existing.prevRoom ? existing.prevRoom->getRoomID() : -1;
            if (existPrev == candPrev) {
                sameFound = true;
                if (cand.effectiveDist < existing.effectiveDist) {
                    existing = cand;
                    changed = true;
                }
                break;
            }
        }

        if (!sameFound) {
            auto bestIt = bestDist.find(rid);
            if (bestIt != bestDist.end() &&
                cand.effectiveDist >= bestIt->second + maxPathDiff)
                return false;

            if (paths.size() < maxPaths) {
                paths.push_back(cand);
                changed = true;
            } else {
                auto worst = std::max_element(paths.begin(), paths.end(),
                    [](const BFSEntry &a, const BFSEntry &b) {
                        return a.effectiveDist < b.effectiveDist;
                    });
                if (cand.effectiveDist < worst->effectiveDist) {
                    *worst = cand;
                    changed = true;
                }
            }
        }

        if (!changed) return false;

        std::sort(paths.begin(), paths.end(),
            [](const BFSEntry &a, const BFSEntry &b) {
                return a.effectiveDist < b.effectiveDist;
            });
        const float newPrimary = paths.front().effectiveDist;
        if (paths.size() > 1) {
            paths.erase(std::remove_if(paths.begin() + 1, paths.end(),
                [newPrimary, maxPathDiff](const BFSEntry &e) {
                    return e.effectiveDist >= newPrimary + maxPathDiff;
                }), paths.end());
        }
        bestDist[rid] = newPrimary;
        return true;
    };

    BFSEntry start{};
    start.room = sourceRoom;
    start.realDist = 0.0f;
    start.effectiveDist = 0.0f;
    start.cumulativeTransmission     = 1.0f;
    start.cumulativeDoorTransmission = 1.0f;
    start.enterPortalIdx = -1;
    start.prevRoom = nullptr;
    start.enterLeadPt        = sourcePos;
    start.enterPortalCenter  = sourcePos;
    tryAccept(start);
    pq.push(start);

    while (!pq.empty()) {
        BFSEntry cur = pq.top();
        pq.pop();

        // Early termination: once the cheapest queue entry exceeds the
        // listener's primary effective distance by more than maxPathDiff,
        // no future pop can produce a path-class within the merge
        // threshold. With N=1 + maxPathDiff = 10 this still lets the
        // listener's primary stabilize before breaking; with N>1 it lets
        // alternates surface before we stop.
        auto lstBestIt = bestDist.find(listenerRoom->getRoomID());
        if (lstBestIt != bestDist.end() &&
            cur.effectiveDist > lstBestIt->second + maxPathDiff)
            break;

        uint32_t portalCount = cur.room->getPortalCount();
        for (uint32_t i = 0; i < portalCount; ++i) {
            if (static_cast<int32_t>(i) == cur.enterPortalIdx)
                continue;

            RoomPortal *portal = cur.room->getPortal(i);
            if (!portal) continue;

            Room *nextRoom = portal->getFarRoom();
            if (!nextRoom) continue;

            // Closest-point hop: resolve the lead point on this portal
            // polygon to the previous lead point. Cost is the Euclidean
            // distance between the two lead points. Replaces the legacy
            // portal-center-to-portal-center matrix lookup, which can
            // overestimate dramatically for large portals (whole-face
            // shared interfaces between open-air rooms).
            Vector3 nextLeadPt = portal->closestPointOnPolygon(cur.enterLeadPt);
            float segDist = glm::length(nextLeadPt - cur.enterLeadPt);

            float newRealDist = cur.realDist + segDist;

            float blocking = blockingFn ? blockingFn(cur.room->getRoomID(),
                                                     nextRoom->getRoomID())
                                        : 0.0f;
            float newEffDist = cur.effectiveDist + segDist;
            if (blocking > 0.0f && newEffDist < maxDist) {
                newEffDist += (maxDist - newEffDist) * blocking;
            }

            // When this hop lands in the listener room, include the final
            // segment from the entry lead-point to the listener position.
            // Without this, BFS would treat two chains as equal-cost even
            // if one chain enters listenerRoom adjacent to the listener
            // and the other enters far across the room — making chain
            // selection blind to the listener's actual position.
            if (nextRoom == listenerRoom) {
                float finalSeg = glm::length(listenerPos - nextLeadPt);
                newRealDist += finalSeg;
                newEffDist  += finalSeg;
            }

            if (newEffDist > maxDist)
                continue;

            float doorTransmissionFactor = (1.0f - blocking);
            float newDoorTransmission = cur.cumulativeDoorTransmission * doorTransmissionFactor;
            float newTransmission     = cur.cumulativeTransmission     * doorTransmissionFactor;

            float loud = loudFn ? loudFn(nextRoom->getRoomID()) : 1.0f;
            newTransmission *= loud;

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
            next.enterLeadPt        = nextLeadPt;
            next.enterPortalCenter  = portal->getCenter();
            if (tryAccept(next))
                pq.push(next);
        }
    }

    auto listenerIt = reachedFrom.find(listenerRoom->getRoomID());
    const bool foundListener = (listenerIt != reachedFrom.end()
                                && !listenerIt->second.empty());

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

    // ── Per-path reconstruction + bidirectional anchor projection ──
    //
    // For each listener-room entry, walk back through prevRoom pointers
    // (picking the primary at each intermediate room) to reconstruct the
    // portal chain. Then compute the geometric path through that chain
    // using a bidirectional closest-point pass:
    //
    //   1. Clean-threading short-circuit: if the source→listener line
    //      pierces every polygon inside the polygon, the path is direct
    //      Euclidean — no bend, virtualPosition = sourcePos. This is the
    //      common case for open-air subdivisions where multiple rooms
    //      share a wide-open polygon (e.g. MISS6 rooms 61↔207).
    //
    //   2. Otherwise: forward pass resolves F_i = closestPointOnPolygon(
    //      portal_i, F_(i-1)) starting from source; backward pass resolves
    //      B_i = closestPointOnPolygon(portal_i, B_(i+1)) starting from
    //      listener. The per-polygon bend point is the midpoint of F_i
    //      and B_i — this is the projection of the (still hypothetical)
    //      straight source→listener line onto the polygon, anchored to
    //      the real source and listener positions rather than to either
    //      end alone. Within ~3% of the optimal single-bend path in
    //      practice; never produces the corner snaps the legacy
    //      sequential-edge-clamp `getRaycastProjection` did.
    struct PortalInfo {
        RoomPortal *portal;
        Room       *fromRoom;
    };

    const std::vector<BFSEntry> listenerEntries = listenerIt->second;

    auto reconstructChain = [&](const BFSEntry &lstEntry) {
        std::vector<PortalInfo> chain;
        Room *walkRoom   = listenerRoom;
        BFSEntry curEntry = lstEntry;
        while (walkRoom != sourceRoom) {
            if (!curEntry.prevRoom) break;
            for (uint32_t i = 0; i < curEntry.prevRoom->getPortalCount(); ++i) {
                RoomPortal *p = curEntry.prevRoom->getPortal(i);
                if (p && p->getFarRoom() == walkRoom) {
                    chain.push_back({p, curEntry.prevRoom});
                    break;
                }
            }
            Room *prev = curEntry.prevRoom;
            walkRoom = prev;
            if (walkRoom == sourceRoom) break;
            auto pit = reachedFrom.find(prev->getRoomID());
            if (pit == reachedFrom.end() || pit->second.empty()) break;
            // Take the primary path at the predecessor — gives a stable,
            // deterministic walk-back. Full per-class chain tracking is
            // deferred (sufficient for SP-1 MVP's two-distinct-doorway
            // case; an SP-1-full enhancement could plumb a path-class
            // index through BFSEntry to make all N walk-backs independent).
            curEntry = pit->second.front();
        }
        std::reverse(chain.begin(), chain.end());
        return chain;
    };

    std::vector<SoundPathRecord> pathRecords;
    pathRecords.reserve(listenerEntries.size());
    // Track the primary path's hop chain for the optional pathOut diagnostic.
    std::vector<BFSEntry> primaryHopChain;

    // Helper: does the source→listener line cross every polygon in the
    // chain inside the polygon? If yes, the path is purely Euclidean —
    // no anchor bends needed, no inflation. Most cross-room sound paths
    // in open architecture (Thief atria, cathedrals, courtyards) thread
    // cleanly through every doorway/opening and should produce direct
    // straight-line distance.
    auto lineThreadsAllPolygons = [&](const std::vector<PortalInfo> &chain) {
        Vector3 sToL = listenerPos - sourcePos;
        constexpr float kEps = 1e-4f;
        for (const auto &info : chain) {
            const Plane &pl = info.portal->getPlane();
            float denom = glm::dot(pl.normal, sToL);
            if (std::fabs(denom) < 1e-7f) return false;  // parallel to portal
            float t = -pl.getDistance(sourcePos) / denom;
            if (t <= 0.0f || t >= 1.0f) return false;    // crossing outside [source, listener]
            Vector3 crossPt = sourcePos + sToL * t;
            for (uint32_t e = 0; e < info.portal->getEdgeCount(); ++e) {
                if (info.portal->getEdgePlane(e).getDistance(crossPt) > kEps)
                    return false;
            }
        }
        return true;
    };

    for (size_t pi = 0; pi < listenerEntries.size(); ++pi) {
        const BFSEntry &lstEntry = listenerEntries[pi];
        std::vector<PortalInfo> portalChain = reconstructChain(lstEntry);

        float   pathDist;
        Vector3 virtualPos;
        std::vector<Vector3> P;  // bend points (empty for clean-threaded)

        // The "clean threading" short-circuit (direct Euclidean line)
        // is accepted ONLY when either no portals lie on the chain OR
        // the straight source→listener line passes through every portal
        // polygon AND is BSP-clear. The BSP check guards against the
        // case where a straight line technically threads every portal
        // polygon but still pierces a wall sitting between the portals
        // (rare in well-authored geometry, common around offset
        // doorways).
        bool tryDirect = portalChain.empty() || lineThreadsAllPolygons(portalChain);
        if (tryDirect && params.losClear) {
            tryDirect = params.losClear(sourcePos, listenerPos);
        }

        if (tryDirect) {
            // Clean threading — direct Euclidean distance, no bend
            pathDist   = glm::length(listenerPos - sourcePos);
            virtualPos = sourcePos;
        } else if (portalChain.empty()) {
            // Same-component but no portal chain to bend through (BFS
            // shouldn't actually reach here cross-room, but defensively
            // skip).
            continue;
        } else {
            // Bidirectional closest-point pass + midpoint bend per
            // polygon. Each bend lies on the segment between the
            // source-side projection (F) and the listener-side projection
            // (B) on the polygon; the midpoint approximates the optimal
            // single-bend point. Avoids the corner-snap of legacy edge
            // clamping and stays anchored to real source/listener
            // positions rather than relying on portal centers.
            const size_t N = portalChain.size();
            std::vector<Vector3> F(N), B(N);
            P.resize(N);

            // Forward pass
            Vector3 leadPt = sourcePos;
            for (size_t i = 0; i < N; ++i) {
                F[i] = portalChain[i].portal->closestPointOnPolygon(leadPt);
                leadPt = F[i];
            }
            // Backward pass
            Vector3 trailPt = listenerPos;
            for (size_t i = N; i-- > 0; ) {
                B[i] = portalChain[i].portal->closestPointOnPolygon(trailPt);
                trailPt = B[i];
            }
            for (size_t i = 0; i < N; ++i) {
                P[i] = (F[i] + B[i]) * 0.5f;
            }

            // ── BSP-aware bend refinement ──
            //
            // The bidirectional midpoint above is geometrically optimal
            // but doesn't know about world geometry — a bend can land at
            // a point on the portal polygon where the segment to its
            // neighbour pierces a wall. When losClear is supplied, walk
            // every bend and verify both incoming and outgoing segments
            // are BSP-clear. For each blocked bend, try alternate
            // candidate positions on the portal polygon (forward
            // projection, backward projection, polygon center, the
            // projection of the straight prev→next line onto the
            // polygon). If any candidate produces clear segments on
            // both sides, swap it in. Iterate up to a few passes
            // because refining bend i may invalidate bend i-1 or i+1.
            //
            // A path that can't be refined to BSP-clear gets dropped
            // entirely — better to mute the voice than render audio
            // along a wall-piercing line.
            bool pathBSPClear = true;
            if (params.losClear) {
                auto refineBend = [&](size_t i,
                                      const Vector3 &prevPt,
                                      const Vector3 &nextPt,
                                      Vector3 &outPt) -> bool {
                    RoomPortal *portal = portalChain[i].portal;
                    // Candidate positions in preference order. Start
                    // with the current midpoint (no-op if already
                    // clear), then alternate projections, then portal
                    // center, then the projection of the straight
                    // prev→next line onto this polygon.
                    Vector3 candidates[5];
                    candidates[0] = P[i];
                    candidates[1] = F[i];
                    candidates[2] = B[i];
                    candidates[3] = portal->getCenter();
                    {
                        const Plane &plane = portal->getPlane();
                        Vector3 dir = nextPt - prevPt;
                        float denom = glm::dot(plane.normal, dir);
                        Vector3 projected;
                        if (std::fabs(denom) > 1e-7f) {
                            float t = -plane.getDistance(prevPt) / denom;
                            projected = prevPt + dir * t;
                            projected = portal->closestPointOnPolygon(projected);
                        } else {
                            projected = portal->getCenter();
                        }
                        candidates[4] = projected;
                    }
                    for (const Vector3 &c : candidates) {
                        if (params.losClear(prevPt, c)
                            && params.losClear(c, nextPt)) {
                            outPt = c;
                            return true;
                        }
                    }
                    return false;
                };

                for (int pass = 0; pass < 3; ++pass) {
                    bool changed  = false;
                    bool allClear = true;
                    for (size_t i = 0; i < N; ++i) {
                        Vector3 prev = (i == 0) ? sourcePos : P[i - 1];
                        Vector3 next = (i + 1 < N) ? P[i + 1] : listenerPos;
                        if (params.losClear(prev, P[i])
                            && params.losClear(P[i], next)) continue;
                        Vector3 refined;
                        if (refineBend(i, prev, next, refined)) {
                            if (glm::length(refined - P[i]) > 1e-4f) {
                                P[i] = refined;
                                changed = true;
                            }
                        } else {
                            allClear = false;
                        }
                    }
                    if (allClear) break;
                    if (!changed) break;
                }
                // Final verification — drop the path if any segment
                // remains BSP-blocked after refinement.
                for (size_t i = 0; i < N && pathBSPClear; ++i) {
                    Vector3 prev = (i == 0) ? sourcePos : P[i - 1];
                    Vector3 next = (i + 1 < N) ? P[i + 1] : listenerPos;
                    if (!params.losClear(prev, P[i])
                        || !params.losClear(P[i], next)) {
                        pathBSPClear = false;
                    }
                }
            }
            if (!pathBSPClear) continue;

            pathDist = glm::length(P.front() - sourcePos);
            for (size_t i = 1; i < N; ++i) {
                pathDist += glm::length(P[i] - P[i - 1]);
            }
            pathDist += glm::length(listenerPos - P.back());
            virtualPos = P.back();
        }

        // Apply blocking to path distance (Dark Engine munged-distance formula).
        // totalBlocking = 1 - cumulativeTransmission. Includes both door
        // blocking and LoudRoom multipliers — both inflate the *perceived*
        // distance for volume-attenuation purposes.
        float totalBlocking_p = 1.0f - lstEntry.cumulativeTransmission;
        float effDist_p = pathDist;
        if (totalBlocking_p > 0.0f && effDist_p < maxDist) {
            effDist_p += (maxDist - effDist_p) * totalBlocking_p;
        }
        // Door-only blocking drives the LPF cutoff downstream. LoudRoom is
        // excluded so sound passing between LoudRoom-modified subdivisions of
        // one open space gets quieter but not muffled.
        float doorBlocking_p = 1.0f - lstEntry.cumulativeDoorTransmission;

        if (effDist_p > maxDist) continue;

        SoundPathRecord rec;
        rec.effectiveDistance = effDist_p;
        rec.realDistance      = pathDist;
        rec.totalBlocking     = totalBlocking_p;
        rec.doorBlocking      = doorBlocking_p;
        rec.virtualPosition   = virtualPos;
        rec.predecessorRoomID = lstEntry.prevRoom
                                  ? lstEntry.prevRoom->getRoomID() : -1;
        rec.chain             = std::move(P);
        pathRecords.push_back(std::move(rec));

        // pi==0 is the primary (lowest effectiveDist by sort order).
        if (pi == 0) {
            // Walk forward through the same BFSEntry chain to populate
            // primaryHopChain for the diagnostic pathOut emission.
            std::vector<BFSEntry> chain;
            Room *walkRoom    = listenerRoom;
            BFSEntry curEntry = lstEntry;
            while (true) {
                chain.push_back(curEntry);
                if (!curEntry.prevRoom || walkRoom == sourceRoom) break;
                Room *prev = curEntry.prevRoom;
                walkRoom = prev;
                auto pit = reachedFrom.find(prev->getRoomID());
                if (pit == reachedFrom.end() || pit->second.empty()) break;
                curEntry = pit->second.front();
                if (walkRoom == sourceRoom) {
                    chain.push_back(curEntry);
                    break;
                }
            }
            std::reverse(chain.begin(), chain.end());
            primaryHopChain = std::move(chain);
        }
    }

    if (pathRecords.empty()) {
        // All reconstructed paths exceeded maxDist after anchor projection
        // (geometric path turned out longer than BFS estimate). Treat as
        // unreached.
        return result;
    }

    // pathRecords is naturally ordered by effectiveDist ascending because
    // listenerEntries was sorted in tryAccept; the per-path anchor
    // projection can only inflate effDist (geometric ≥ BFS estimate is
    // not strictly guaranteed, but in practice the projection adds
    // distance for corner-snap cases — re-sort defensively to keep the
    // pathRecords[0]-is-primary invariant clean for downstream code).
    std::sort(pathRecords.begin(), pathRecords.end(),
        [](const SoundPathRecord &a, const SoundPathRecord &b) {
            return a.effectiveDistance < b.effectiveDistance;
        });

    // Diagnostic chain output: take the bend chain of the merge primary
    // (lowest post-projection effDist). This is the chain that drives
    // Steam Audio's source coordinate via the merged virtualPosition;
    // the renderer's show_vpos overlay visualises THIS chain.
    if (params.chainOut && !pathRecords.empty()) {
        *params.chainOut = pathRecords.front().chain;
    }

    // ── Merge: min effDist drives scalar fields; inv-d² weighted vPos. ──
    // Matches the original engine's sound-merge policy of
    // taking the loudest contribution for volume + LPF, while letting the
    // virtual position blend so panning reflects multi-opening geometry.
    const SoundPathRecord &best = pathRecords.front();
    Vector3 vPosNum(0.0f, 0.0f, 0.0f);
    float   vPosDen = 0.0f;
    for (const auto &p : pathRecords) {
        float d2 = std::max(p.effectiveDistance * p.effectiveDistance, 0.01f);
        float w  = 1.0f / d2;
        vPosNum += p.virtualPosition * w;
        vPosDen += w;
    }
    Vector3 mergedVirtualPos = (vPosDen > 0.0f)
                                  ? (vPosNum / vPosDen)
                                  : best.virtualPosition;

    result.reached          = true;
    result.realDistance     = best.realDistance;
    result.effectiveDistance = best.effectiveDistance;
    result.totalBlocking    = best.totalBlocking;
    result.doorBlocking     = best.doorBlocking;
    result.virtualPosition  = mergedVirtualPos;
    result.paths            = std::move(pathRecords);

    // ── Optional path-detail emission (diagnostic) ──
    // Emits the PRIMARY path's hop chain only. Multi-path consumers read
    // result.paths for the full per-path breakdown.
    if (params.pathOut && result.reached) {
        for (size_t i = 0; i < primaryHopChain.size(); ++i) {
            const BFSEntry &entry = primaryHopChain[i];
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
                const BFSEntry &prev = primaryHopChain[i - 1];
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
