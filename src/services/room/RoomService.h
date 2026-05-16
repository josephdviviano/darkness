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

#ifndef __ROOMSERVICE_H
#define __ROOMSERVICE_H

#include <vector>
#include <memory>
#include <functional>
#include <unordered_map>

#include "DarkCommon.h"
#include "DarknessService.h"
#include "DarknessServiceFactory.h"
#include "RoomCommon.h"
#include "ServiceCommon.h"
#include "SharedPtr.h"
#include "DarknessMath.h"
#include "config.h"
#include "database/DatabaseCommon.h"

namespace Darkness {

/// Default propagation cutoff in world units; matches the long-standing
/// AudioService::SOUND_MAX_DIST default. Hoisted here so non-audio callers
/// of propagateSoundPath (e.g. the headless trace-path tool) don't need to
/// pull in the audio header just to share a sensible default.
constexpr float kSoundMaxDist = 200.0f;

/// One reconstructed path's contribution to a multi-path propagation
/// result. Each instance describes a topologically distinct route the
/// sound took from source to listener room (e.g., two parallel doorways
/// would yield two records). The merged scalar fields on SoundPropInfo
/// summarize across these records for callers that don't care about
/// per-path detail.
struct SoundPathRecord {
    /// Effective distance for this path (with door + LoudRoom inflation).
    float effectiveDistance = 0.0f;
    /// Geometric path distance through the portal chain.
    float realDistance = 0.0f;
    /// Combined transmission loss [0,1] — doors + LoudRoom.
    float totalBlocking = 0.0f;
    /// Door-only transmission loss [0,1].
    float doorBlocking = 0.0f;
    /// Last anchor on this path (the sound's apparent origin direction
    /// for spatialization if this path were chosen alone).
    Vector3 virtualPosition{0, 0, 0};
    /// Room ID of the listener room's predecessor along this path —
    /// distinguishes path-classes when two paths converge at the listener.
    /// -1 if source == listener (same-room short-circuit).
    int32_t predecessorRoomID = -1;
};

/// Result of sound propagation through the portal graph.
/// Describes how a sound reaches a listener after traversing portals and doors.
/// Scalar fields below are merged across all paths in `paths` (see
/// RoomService::propagateSoundPath); single-path callers can ignore the
/// vector and consume the merged scalars unchanged. The original Dark
/// Engine kept up to two predecessor paths per room (cBFRoomInfo's
/// previous_room_2 + MergeSounds); SP-1 generalizes to N configurable
/// paths.
struct SoundPropInfo {
    /// Merged effective distance (min over paths). Drives volume.
    float effectiveDistance = 0.0f;
    /// Merged real distance (from the path with min effectiveDistance).
    float realDistance = 0.0f;
    /// Merged combined transmission loss [0,1] (from min-eff path).
    float totalBlocking = 0.0f;
    /// Merged door-only transmission loss [0,1] (from min-eff path).
    /// Drives the LPF cutoff downstream.
    float doorBlocking = 0.0f;
    /// Merged virtual position: inverse-distance² weighted average of
    /// each path's anchor. Collapses to a single path's anchor when
    /// only one path exists; produces a directional average when
    /// multiple openings simultaneously route sound to the listener.
    Vector3 virtualPosition{0, 0, 0};
    /// Whether the sound can reach the listener at all.
    bool reached = false;
    /// Per-path detail. Empty when `reached == false`. Single entry for
    /// N=1 / same-room / disconnected cases. Sorted by effectiveDistance
    /// ascending (paths[0] is the loudest).
    std::vector<SoundPathRecord> paths;
};

/// One hop in the BFS path from source room to listener room. Populated
/// only when SoundPropParams::pathOut is non-null on the call. The first
/// entry is always the source room (with enterPortalCenter = sourcePos,
/// segmentDist = 0); the last entry is the listener room. Useful for
/// diagnostic tools that want to see *how* the BFS reached the listener.
struct SoundPathHop {
    int32_t roomID;
    /// Distance traversed within this room before exiting through the next
    /// portal (0 for the listener room — the final segment is recorded as
    /// part of the previous hop's exit).
    float segmentDist;
    /// Door blocking factor [0,1] applied when ENTERING this room. 0 for
    /// the source room.
    float doorBlocking;
    /// LoudRoom multiplier applied to ENTERING this room (1.0 = no effect).
    float loudRoom;
    /// Cumulative real (geometric) distance from source up to this room.
    float cumRealDist;
    /// Cumulative effective (blocking-inflated) distance up to this room.
    float cumEffDist;
    /// Center of the portal the BFS used to enter this room — equal to
    /// sourcePos for the source room.
    Vector3 enterPortalCenter;
};

/// Optional parameters for RoomService::propagateSoundPath. Callbacks let
/// the caller supply runtime cost data (door blocking from currently-closed
/// doors, LoudRoom multipliers from P$LoudRoom properties) without baking
/// those concerns into RoomService itself. Either callback may be left
/// unset — defaults match a "fully open / no LoudRoom" topology trace.
struct SoundPropParams {
    /// Maximum propagation distance (effectiveDistance cutoff).
    float maxDist = kSoundMaxDist;
    /// Per-portal door blocking factor [0,1]: 0 = fully open, 1 = fully
    /// closed. Should be symmetric in its two arguments. If null, all
    /// portals are treated as open.
    std::function<float(int32_t fromRoomID, int32_t toRoomID)> doorBlocking;
    /// Per-room LoudRoom transmission multiplier. 1.0 = no effect; values
    /// < 1.0 reduce transmission. If null, every room contributes factor
    /// 1.0.
    std::function<float(int32_t roomID)> loudRoom;
    /// Maximum number of simultaneous BFS paths kept per listener room.
    /// 1 = today's single-shortest-path behavior; 2 = the original Dark
    /// Engine's dual-predecessor scheme (cBFRoomInfo::previous_room_2);
    /// up to 4 supported. Clamped to [1, 4] inside propagateSoundPath.
    /// Default 2 reproduces the original engine.
    uint32_t maxPaths = 2;
    /// An alternate path is kept only if its effective distance is
    /// within this many world units of the primary (best) path. Beyond
    /// this, alternates contribute too little to the merge to matter.
    /// Default 10.0 matches the original engine's kMaxDistDiff.
    float maxPathDiff = 10.0f;
    /// Diagnostic accumulator. If non-null, the BFS path from source to
    /// listener is appended to this vector (one entry per visited room).
    /// For multi-path runs, this receives the primary (lowest-effDist)
    /// chain only. Cleared before write. Only populated on success.
    std::vector<SoundPathHop> *pathOut = nullptr;
};

/** @brief Room service - service providing a Room database.
 * Room database is a high level system that classifies object positions into
 * 'rooms'.
 * @note This service listens to object positions
 * @note This service is a source of script messages
 */
class RoomService : public ServiceImpl<RoomService>,
                    DatabaseListener {
public:
    /// Constructor
    RoomService(ServiceManager *manager, const std::string &name);

    /// Destructor
    ~RoomService() override;
    Room *getRoomByID(int32_t id);

    /** Reassigns the object's room if a room is sucessfully found
     * @param idset The id set to use (typically 0-objects, 1-AI)
     * @param objID The id of the object to track down
     * @param pos The position of the object
     * @return Room pointer if room was found, NULL otherwise
     */
    Room *findObjRoom(size_t idset, int objID, const Vector3 &pos);

    /** Finds the room which encloses the specified point.
     *
     * This is the canonical room-membership query. Room OBBs overlap by
     * design at portal boundaries, so multiple rooms may pass the raw
     * OBB containment test for one point — `roomFromPoint` collects all
     * such candidates and disambiguates them using the portal plane
     * between each pair. Always prefer this over `Room::obbContains` for
     * "is this point in this room" questions.
     *
     * @param pos The position to locate room for
     * @return Room pointer if the position is enclosed in a room, NULL
     * otherwise
     */
    Room *roomFromPoint(const Vector3 &pos);

    /** Refreshes a cached room pointer for an object that may have moved.
     *
     * Returns the current room for `pos`, identical to `roomFromPoint(pos)`.
     * The `cached` argument is accepted so callers can express intent
     * ("I had this pointer last frame, give me the current one") and so a
     * future spatial-cache optimization has a place to live without
     * touching call sites. Today this is a thin wrapper.
     *
     * Callers should compare the returned pointer to their previous cached
     * value to detect room transitions; do NOT use `Room::obbContains` on
     * the cached pointer as a "still in the same room" shortcut — that is
     * the broken pattern this API replaces, and it silently lies whenever
     * the listener stands inside two overlapping room OBBs.
     *
     * @param cached Previous cached room pointer (may be NULL or stale)
     * @param pos Current position of the object/listener
     * @return Current room for `pos`, or NULL if not in any room.
     */
    Room *updatedRoom(Room *cached, const Vector3 &pos);

    /** Updates the object's room (preferably incrementally without doing room
     * search)
     * @param idset The id set to use (typically 0-objects, 1-AI)
     * @param objID The id of the object to track down
     * @param pos The position of the object
     */
    void updateObjRoom(size_t idset, int objID, const Vector3 &pos);

    /** Attaches the specified object to a room instance
     * @param idset the object id set
     * @param objID the object to attach
     * @param room the room to attach the object to
     * */
    void _attachObjRoom(size_t idset, int objID, Room *room);

    /** Detaches the object from a room (be it attached to some)
     * @param idset the object id set
     * @param objID the object id to track down and detach
     * @param current the current room, or NULL if it should be searched for
     */
    void _detachObjRoom(size_t idset, int objID, Room *current = NULL);

    /** Returns the current room the object is attached to
     * @param idset the object id set
     * @param objID the object id
     * @return Room pointer or NULL if the object is not attached
     * */
    Room *getCurrentObjRoom(size_t idset, int objID) const;

    // --- Read-only accessors for IWorldQuery ---

    /// Get all rooms (read-only). Returns empty vector if rooms not loaded.
    const std::vector<std::unique_ptr<Room>> &getAllRooms() const {
        return mRooms;
    }

    /// Check if the room database is loaded and valid
    bool isLoaded() const { return mRoomsOk; }

    /** Propagate sound from source to listener through the portal graph.
     *
     * Uses Dijkstra-style BFS through room portals with the precomputed
     * portal-to-portal distance matrix inside each room (from ROOM_DB).
     * Cost callbacks supplied via `params` plug in runtime data:
     *   - `doorBlocking(a,b)` — door blocking factor for the portal from
     *     room a to room b; inflates effectiveDistance and is also tracked
     *     separately as a door-only transmission product for the LPF.
     *   - `loudRoom(r)`      — LoudRoom multiplier for room r; folded into
     *     combined transmission but not into the door-only product.
     * Either callback may be unset for a topology-only trace (no door
     * blocking, no LoudRoom).
     *
     * If `params.pathOut` is non-null, the BFS path from source to listener
     * is also appended there — one entry per visited room — for diagnostic
     * tracing.
     *
     * @param sourcePos     World-space source position
     * @param listenerPos   World-space listener position
     * @param sourceRoom    Room enclosing sourcePos (caller resolves)
     * @param listenerRoom  Room enclosing listenerPos (caller resolves)
     * @param params        Cost callbacks + optional path accumulator
     * @return Reached/blocked propagation info; `reached=false` if no path
     *         within `params.maxDist` exists.
     */
    SoundPropInfo propagateSoundPath(const Vector3 &sourcePos,
                                      const Vector3 &listenerPos,
                                      Room *sourceRoom,
                                      Room *listenerRoom,
                                      const SoundPropParams &params) const;

protected:
    void setCurrentObjRoom(size_t idset, int objID, Room *room);

    // service related
    bool init();
    void bootstrapFinished();
    void shutdown();

    void clear();

    /** Database load callback
     * @see DatabaseListener::onDBLoad */
    void onDBLoad(const FileGroupPtr &db, uint32_t curmask);

    /** Database save callback
     * @see DatabaseListener::onDBSave */
    void onDBSave(const FileGroupPtr &db, uint32_t tgtmask);

    /** Database drop callback
     * @see DatabaseListener::onDBDrop */
    void onDBDrop(uint32_t dropmask);

private:
    typedef std::unordered_map<int32_t, Room *> RoomsByID; // weak ptrs to rooms
    typedef std::vector<RoomsByID> ObjectIDSets;

    /// Database service
    DatabaseServicePtr mDbService;

    /// Array of all rooms
    std::vector<std::unique_ptr<Room>> mRooms;

    /// Map of rooms by their ID
    RoomsByID mRoomsByID;

    /// Indicates the room database is loaded/ok
    bool mRoomsOk;

    /// Sets of object id's
    ObjectIDSets mIDSets;
};

/// Factory for the RoomService objects
class RoomServiceFactory : public ServiceFactory {
public:
    RoomServiceFactory();
    ~RoomServiceFactory(){};

    /** Creates a RoomService instance */
    Service *createInstance(ServiceManager *manager);

    const std::string &getName() override;
    const uint getMask() override;
    const size_t getSID() override;

private:
    static const std::string mName;
};
} // namespace Darkness

#endif
