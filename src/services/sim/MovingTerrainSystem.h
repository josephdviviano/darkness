/******************************************************************************
 *
 *    This file is part of the Darkness engine
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

// MovingTerrainSystem.h — Elevator and platform simulation
//
// Implements the Dark Engine moving terrain system: objects that follow
// waypoint paths defined by TPath link chains. Platforms move at a configured
// speed, optionally pausing at each waypoint before advancing to the next.
//
// Architecture:
//   - MovingTerrainSystem is a SimListener: receives simStep() from SimService
//   - Each platform has a PlatformState with current position, velocity,
//     target waypoint, and pause timer
//   - PlatformState writes to ObjectStateMap so the renderer sees updated
//     transforms
//   - Activation via TurnOn/TurnOff script messages (buttons, levers, etc.)
//
// Data model:
//   - P$MovingTerrain property: marks an object as a moving platform
//   - TPathInit link: platform → first waypoint (no data)
//   - TPathNext link: platform → current target waypoint (runtime, no data)
//   - TPath link: waypoint → waypoint, carries {speed, pausetime, pathlimit}
//
// Inspired by the openDarkEngine (OPDE) service architecture (GPLv2).
// Moving terrain algorithm reimplemented from Dark Engine behavior analysis.

#pragma once

#include <cstdint>
#include <cmath>
#include <cstring>
#include <unordered_map>
#include <functional>

#include "DarknessMath.h"
#include "SimCommon.h"
#include "object/ObjectService.h"
#include "property/DarkPropertyDefs.h"
#include "property/PropertyService.h"
#include "property/TypedProperty.h"
#include "link/LinkService.h"
#include "link/Relation.h"
#include "worldquery/ObjectState.h"

namespace Darkness {

// Forward declarations
class PropertyService;
class ObjectStateMap;
class MessageDispatch;

// ── Platform movement status ──
enum PlatformStatus : int32_t {
    kPlatformStopped = 0,  // Not moving, waiting for activation
    kPlatformMoving  = 1,  // Moving toward current waypoint
    kPlatformPaused  = 2,  // Pausing at a waypoint before continuing
};

// ── Per-waypoint cached data (from TPath link) ──
struct WaypointSegment {
    int32_t waypointID = 0;      // Destination waypoint object ID
    Vector3 position;            // World position of this waypoint
    float speed = 0.0f;          // Movement speed TO this waypoint (units/sec)
    float pauseDuration = 0.0f;  // Pause time AT this waypoint (seconds)
    bool pathLimit = false;      // Enforce hard position limit
};

// ── Per-platform runtime state ──
struct PlatformState {
    int32_t objID = 0;
    PlatformStatus status = kPlatformStopped;

    // Waypoint graph — ordered list of waypoints forming the path.
    // For non-looping paths (most elevators), the platform travels to the
    // last waypoint and stops. On re-activation it reverses direction.
    std::vector<WaypointSegment> waypoints;
    int currentWaypointIdx = 0;  // Index into waypoints[] we're heading toward
    bool isLoop = false;         // True if the path loops back to the start
    int direction = 1;           // +1 = forward through waypoints, -1 = reverse

    // Movement state
    Vector3 currentPos;          // Current world position
    Vector3 velocity;            // Current movement velocity vector
    float speed = 0.0f;          // Current segment speed

    // Pause state
    float pauseTimer = 0.0f;     // Countdown timer (seconds remaining)

    // Base transform (from P$Position at load time)
    SimTransform base;
    bool hasObjectState = false;

    // Whether this platform was flagged as active at load time
    bool initiallyActive = false;
};

// ── Callback for collision body updates during platform animation ──
using PlatformCollisionCallback = std::function<void(int32_t objID,
                                                      const Matrix4 &worldMatrix)>;

// ============================================================================
// MovingTerrainSystem — manages all moving platforms in the level
// ============================================================================

class MovingTerrainSystem : public SimListener {
public:
    MovingTerrainSystem() = default;

    // ── Initialization ──

    /// Scan for all objects with P$MovingTerrain and build waypoint graphs
    /// from TPathInit and TPath link chains. Call after level load.
    void init(PropertyService *propSvc, ObjectService *objSvc,
              LinkService *linkSvc, ObjectStateMap *objectStates,
              const std::unordered_map<int32_t, ObjPlacementInfo> *placements = nullptr) {
        mObjectStates = objectStates;
        mObjSvc = objSvc;
        mPlacements = placements;

        // Get the TPath relation for link data queries
        RelationPtr tpathRel = linkSvc->getRelation("TPath");

        // Scan all positioned concrete objects for P$MovingTerrain.
        // The property is typically on the archetype (e.g. "Elevator") and
        // inherited by concrete objects via MetaProp, so we iterate placements
        // and check getTypedProperty (which walks the inheritance chain).
        if (!mPlacements) {
            std::fprintf(stderr, "MovingTerrainSystem: no placement data — skipping\n");
            return;
        }
        int totalWaypoints = 0;
        for (const auto &[id, placement] : *mPlacements) {
            if (id <= 0) continue;  // Skip archetypes

            PropMovingTerrain prop;
            if (!getTypedProperty<PropMovingTerrain>(propSvc, "MovingTer", id, prop))
                continue;

            PlatformState plat;
            plat.objID = id;
            plat.initiallyActive = (prop.active != 0);

            // Get base transform from placement data
            initBaseTransform(plat, propSvc, id);

            // Build waypoint graph by following TPathInit → TPath chain
            if (!buildWaypointGraph(plat, linkSvc, tpathRel.get())) {
                std::fprintf(stderr, "  MovingTerrain %d: no valid waypoint path, skipping\n", id);
                continue;
            }

            totalWaypoints += static_cast<int>(plat.waypoints.size());

            // Set initial position — platform starts at its P$Position location
            plat.currentPos = plat.base.position;

            // If initially active, start moving toward first waypoint
            if (plat.initiallyActive && !plat.waypoints.empty()) {
                startMovingToWaypoint(plat, 0);
            }

            std::fprintf(stderr, "  MovingTerrain %d: %zu waypoints, loop=%d, active=%d, "
                         "pos=(%.1f,%.1f,%.1f)\n",
                         id, plat.waypoints.size(),
                         plat.isLoop ? 1 : 0,
                         plat.initiallyActive ? 1 : 0,
                         plat.currentPos.x, plat.currentPos.y, plat.currentPos.z);

            mPlatforms[id] = std::move(plat);
        }

        std::fprintf(stderr, "MovingTerrainSystem: %zu platforms, %d total waypoints\n",
                     mPlatforms.size(), totalWaypoints);
    }

    // ── Platform control ──

    /// Activate a platform: start moving along its waypoint path.
    bool activate(int32_t objID) {
        auto it = mPlatforms.find(objID);
        if (it == mPlatforms.end()) {
            static int w = 0; if (w++ < 5)
                std::fprintf(stderr, "[FALLBACK] MovingTerrainSystem::activate: obj %d not a platform\n", objID);
            return false;
        }

        PlatformState &plat = it->second;
        if (plat.waypoints.empty()) {
            std::fprintf(stderr, "[FALLBACK] MovingTerrainSystem::activate: obj %d has no waypoints\n", objID);
            return false;
        }

        // If already moving or paused, do nothing
        if (plat.status != kPlatformStopped) return true;

        ensureObjectState(plat);

        // Compute next waypoint in current direction
        int numWP = static_cast<int>(plat.waypoints.size());
        int nextIdx = plat.currentWaypointIdx + plat.direction;
        if (plat.isLoop) {
            nextIdx = ((nextIdx % numWP) + numWP) % numWP;
        } else {
            nextIdx = std::clamp(nextIdx, 0, numWP - 1);
        }
        startMovingToWaypoint(plat, nextIdx);
        return true;
    }

    /// Deactivate a platform: stop movement immediately.
    bool deactivate(int32_t objID) {
        auto it = mPlatforms.find(objID);
        if (it == mPlatforms.end()) return false;

        PlatformState &plat = it->second;
        plat.status = kPlatformStopped;
        plat.velocity = Vector3(0.0f);
        return true;
    }

    /// Toggle platform: activate if stopped, deactivate if moving.
    bool toggle(int32_t objID) {
        auto it = mPlatforms.find(objID);
        if (it == mPlatforms.end()) return false;

        PlatformState &plat = it->second;
        if (plat.status == kPlatformStopped)
            return activate(objID);
        else
            return deactivate(objID);
    }

    /// Check if an object is a moving platform.
    bool isPlatform(int32_t objID) const {
        return mPlatforms.find(objID) != mPlatforms.end();
    }

    /// Get the current velocity of a platform (for platform riding).
    /// Returns nullptr if not a platform or not moving.
    const Vector3 *getVelocity(int32_t objID) const {
        auto it = mPlatforms.find(objID);
        if (it == mPlatforms.end()) return nullptr;
        if (it->second.status != kPlatformMoving) return nullptr;
        return &it->second.velocity;
    }

    /// Set callback for collision body updates during platform animation.
    void setCollisionUpdateCallback(PlatformCollisionCallback cb) {
        mCollisionUpdateCb = std::move(cb);
    }

    /// Set MessageDispatch for sending script messages on waypoint arrival.
    void setMessageDispatch(MessageDispatch *msgDispatch) {
        mMsgDispatch = msgDispatch;
    }

    /// Get all platform IDs (for debug enumeration).
    std::vector<int32_t> getAllPlatformIDs() const {
        std::vector<int32_t> ids;
        ids.reserve(mPlatforms.size());
        for (const auto &[id, _] : mPlatforms) ids.push_back(id);
        return ids;
    }

    // ── SimListener interface ──

    void simStep(float simTime, float delta) override {
        if (delta <= 0.0f) return;
        ++mFrameCount;
        for (auto &[id, plat] : mPlatforms) {
            switch (plat.status) {
            case kPlatformMoving:
                updateMoving(plat, delta);
                // Log position every 60 frames (~1 second)
                if (mFrameCount % 60 == 0) {
                    const WaypointSegment &tgt = plat.waypoints[plat.currentWaypointIdx];
                    float dist = glm::length(tgt.position - plat.currentPos);
                    std::fprintf(stderr, "  [MT] %d: MOVING idx=%d/%zu dir=%d "
                                 "pos=(%.1f,%.1f,%.1f) tgt=(%.1f,%.1f,%.1f) dist=%.1f spd=%.1f\n",
                                 id, plat.currentWaypointIdx,
                                 plat.waypoints.size(), plat.direction,
                                 plat.currentPos.x, plat.currentPos.y, plat.currentPos.z,
                                 tgt.position.x, tgt.position.y, tgt.position.z,
                                 dist, plat.speed);
                }
                break;
            case kPlatformPaused:
                updatePaused(plat, delta);
                break;
            case kPlatformStopped:
                // Nothing to do
                break;
            }
        }
    }

private:
    // ── Initialization helpers ──

    /// Get the platform's base transform from ObjectPlacement data or P$Position.
    void initBaseTransform(PlatformState &plat, PropertyService *propSvc,
                           int32_t objID) {
        static constexpr float kAngScale = 2.0f * 3.14159265f / 65536.0f;

        if (mPlacements) {
            auto it = mPlacements->find(objID);
            if (it != mPlacements->end()) {
                const auto &pl = it->second;
                plat.base.position = Vector3(pl.x, pl.y, pl.z);
                plat.base.scale = Vector3(pl.sx, pl.sy, pl.sz);

                // Build rotation matching Dark Engine convention:
                // Rz(heading) * Ry(pitch) * Rx(bank)
                float h   = static_cast<float>(pl.heading) * kAngScale;
                float pit = static_cast<float>(pl.pitch)   * kAngScale;
                float b   = static_cast<float>(pl.bank)    * kAngScale;
                plat.base.rotation = Matrix4(glm::mat3(
                    glm::eulerAngleZYX(h, pit, b)));
                return;
            }
        }

        // Fallback: use ObjectService position
        if (mObjSvc) {
            plat.base.position = mObjSvc->position(objID);
        }
    }

    /// Build the waypoint graph by following TPathInit → TPath link chain.
    /// Returns false if no valid path found.
    bool buildWaypointGraph(PlatformState &plat, LinkService *linkSvc,
                            Relation *tpathRel) {
        // Find the starting waypoint via TPathInit link
        int tpathInitFlavor = linkSvc->nameToFlavor("TPathInit");
        if (tpathInitFlavor < 0) {
            std::fprintf(stderr, "[FALLBACK] MovingTerrainSystem: TPathInit relation not registered\n");
            return false;
        }

        LinkQueryResultPtr initLinks = linkSvc->getAllLinks(
            tpathInitFlavor, plat.objID, 0);
        if (!initLinks || initLinks->end()) {
            std::fprintf(stderr, "[FALLBACK] MovingTerrainSystem: obj %d has no TPathInit link (no starting waypoint)\n", plat.objID);
            return false;
        }

        const Link &initLink = initLinks->next();
        int32_t firstWaypoint = initLink.dst();

        // Follow the TPath link chain from the starting waypoint
        int tpathFlavor = linkSvc->nameToFlavor("TPath");
        if (tpathFlavor < 0) {
            std::fprintf(stderr, "[FALLBACK] MovingTerrainSystem: TPath relation not registered\n");
            return false;
        }

        int32_t currentWP = firstWaypoint;
        std::unordered_map<int32_t, bool> visited;  // cycle detection

        while (currentWP != 0 && visited.find(currentWP) == visited.end()) {
            visited[currentWP] = true;

            WaypointSegment seg;
            seg.waypointID = currentWP;

            // Get waypoint position from ObjectService
            if (mObjSvc) {
                seg.position = mObjSvc->position(currentWP);
            }

            // Find the TPath link FROM the previous waypoint TO this one
            // to get movement parameters. For the first waypoint, look for
            // a TPath link from any source to this waypoint.
            // Actually, the TPath link data describes the segment TO the
            // destination waypoint. We get it by finding the TPath link
            // that leads to currentWP.
            seg.speed = 5.0f;        // Default speed if no link data
            seg.pauseDuration = 0.0f;
            seg.pathLimit = false;

            // For the first waypoint, read TPath data from the platform's
            // TPathInit context. For subsequent waypoints, read from the
            // TPath link that connects the previous waypoint to this one.
            if (plat.waypoints.empty()) {
                // First waypoint: try to read TPath link from platform → first WP
                // (some levels use TPath link on platform itself)
                readTPathData(seg, linkSvc, tpathRel, plat.objID, currentWP);
            } else {
                int32_t prevWP = plat.waypoints.back().waypointID;
                readTPathData(seg, linkSvc, tpathRel, prevWP, currentWP);
            }

            plat.waypoints.push_back(seg);

            // Find next waypoint in chain via TPath link FROM currentWP
            LinkQueryResultPtr nextLinks = linkSvc->getAllLinks(
                tpathFlavor, currentWP, 0);
            if (!nextLinks || nextLinks->end()) {
                // End of chain — check if it loops back to the first waypoint
                break;
            }

            const Link &nextLink = nextLinks->next();
            int32_t nextWP = nextLink.dst();

            // If the next waypoint is the first one, we have a loop
            if (nextWP == firstWaypoint) {
                plat.isLoop = true;
                break;
            }

            currentWP = nextWP;
        }

        return !plat.waypoints.empty();
    }

    /// Read TPath link data (speed, pausetime, pathlimit) for a segment.
    /// TPath link data is stored as raw bytes matching the PropTPath struct
    /// (16 bytes: float speed, int32 pausetime, uint32 pathlimit, int32 zero).
    /// The OPDE RawDataStorage doesn't support field-level access, so we
    /// read the raw bytes and interpret them directly.
    void readTPathData(WaypointSegment &seg, LinkService * /*linkSvc*/,
                       Relation *tpathRel, int32_t srcObj, int32_t dstObj) {
        if (!tpathRel) return;

        // Find the TPath link from src to dst
        LinkQueryResultPtr links = tpathRel->getAllLinks(srcObj, dstObj);
        if (!links || links->end()) return;

        const Link &lnk = links->next();
        link_id_t linkID = lnk.id();

        // Read raw link data bytes and interpret as PropTPath
        size_t dataSize = 0;
        const uint8_t *data = tpathRel->getRawLinkData(linkID, dataSize);
        if (!data || dataSize < sizeof(PropTPath)) {
            static int w = 0; if (w++ < 10)
                std::fprintf(stderr, "[DEFAULT] MovingTerrainSystem: TPath link %d->%d has no data (%zu bytes), using default speed=5.0\n",
                             srcObj, dstObj, dataSize);
            return;
        }

        PropTPath tpath;
        std::memcpy(&tpath, data, sizeof(PropTPath));

        if (tpath.speed > 0.0f)
            seg.speed = tpath.speed;
        seg.pauseDuration = static_cast<float>(tpath.pausetime) / 1000.0f;  // ms → sec
        seg.pathLimit = (tpath.pathlimit != 0);
    }

    // ── Movement helpers ──

    /// Start moving toward the given waypoint index.
    void startMovingToWaypoint(PlatformState &plat, int waypointIdx) {
        if (plat.waypoints.empty()) return;

        plat.currentWaypointIdx = waypointIdx % static_cast<int>(plat.waypoints.size());
        const WaypointSegment &target = plat.waypoints[plat.currentWaypointIdx];

        Vector3 direction = target.position - plat.currentPos;
        float dist = glm::length(direction);

        if (dist < 0.01f) {
            // Already at waypoint — trigger arrival immediately
            hitWaypoint(plat);
            return;
        }

        plat.speed = target.speed;
        plat.velocity = glm::normalize(direction) * target.speed;
        plat.status = kPlatformMoving;

        ensureObjectState(plat);
    }

    /// Per-frame update for a moving platform.
    void updateMoving(PlatformState &plat, float dt) {
        const WaypointSegment &target = plat.waypoints[plat.currentWaypointIdx];
        Vector3 toTarget = target.position - plat.currentPos;
        float distRemaining = glm::length(toTarget);

        // How far we'd move this frame
        float stepDist = plat.speed * dt;

        if (stepDist >= distRemaining) {
            // Reached (or overshot) the waypoint — snap to it
            plat.currentPos = target.position;
            plat.velocity = Vector3(0.0f);
            applyTransform(plat);
            hitWaypoint(plat);
        } else {
            // Normal movement: advance toward target
            plat.currentPos += plat.velocity * dt;

            // Recompute velocity direction to correct drift from floating-point
            // accumulation (keeps the platform on a straight line to the target)
            Vector3 newToTarget = target.position - plat.currentPos;
            float newDist = glm::length(newToTarget);
            if (newDist > 0.01f) {
                plat.velocity = glm::normalize(newToTarget) * plat.speed;
            }

            applyTransform(plat);
        }
    }

    /// Per-frame update for a paused platform.
    void updatePaused(PlatformState &plat, float dt) {
        plat.pauseTimer -= dt;
        if (plat.pauseTimer <= 0.0f) {
            // Pause complete — advance to next waypoint using direction
            int numWP = static_cast<int>(plat.waypoints.size());
            int nextIdx = plat.currentWaypointIdx + plat.direction;
            if (plat.isLoop) {
                nextIdx = ((nextIdx % numWP) + numWP) % numWP;
            } else if (nextIdx < 0 || nextIdx >= numWP) {
                // Reached end during pause — stop and reverse
                plat.direction = -plat.direction;
                plat.status = kPlatformStopped;
                plat.velocity = Vector3(0.0f);
                return;
            }
            startMovingToWaypoint(plat, nextIdx);
        }
    }

    /// Called when the platform reaches a waypoint.
    void hitWaypoint(PlatformState &plat) {
        const WaypointSegment &wp = plat.waypoints[plat.currentWaypointIdx];
        int numWP = static_cast<int>(plat.waypoints.size());

        std::fprintf(stderr, "  MovingTerrain %d: reached waypoint %d "
                     "(idx %d/%d, dir=%d) pos=(%.1f,%.1f,%.1f) pause=%.1fs\n",
                     plat.objID, wp.waypointID,
                     plat.currentWaypointIdx, numWP, plat.direction,
                     wp.position.x, wp.position.y, wp.position.z,
                     wp.pauseDuration);

        // Send script messages via MessageDispatch
        sendWaypointMessages(plat, wp);

        // Compute next waypoint index based on direction
        int nextIdx = plat.currentWaypointIdx + plat.direction;

        // Check for end-of-path
        if (!plat.isLoop && (nextIdx < 0 || nextIdx >= numWP)) {
            // Non-looping path: reached the end — stop and reverse direction
            // so the next activation goes back the other way
            plat.direction = -plat.direction;
            plat.status = kPlatformStopped;
            plat.velocity = Vector3(0.0f);
            std::fprintf(stderr, "  MovingTerrain %d: stopped at end, "
                         "direction reversed to %d\n",
                         plat.objID, plat.direction);
            return;
        }

        // Looping path: wrap around
        if (plat.isLoop) {
            nextIdx = ((nextIdx % numWP) + numWP) % numWP;
        }

        if (wp.pauseDuration > 0.0f) {
            // Pause at this waypoint, then auto-continue after the timer
            plat.status = kPlatformPaused;
            plat.velocity = Vector3(0.0f);
            plat.pauseTimer = wp.pauseDuration;
        } else {
            // Continue immediately to next waypoint.
            // NOTE: In the original engine, scripts (e.g. StdElevator)
            // deactivate the platform on waypoint arrival when pause=0.
            // Without the script system (Phase 6), platforms with pause=0
            // will loop continuously. This is correct engine behavior —
            // stopping requires script-driven deactivation.
            startMovingToWaypoint(plat, nextIdx);
        }
    }

    /// Send MovingTerrainWaypoint and WaypointReached script messages.
    void sendWaypointMessages(const PlatformState &plat,
                              const WaypointSegment &wp) {
        if (!mMsgDispatch) return;

        // These messages are sent via MessageDispatch for script handlers
        // to react to (e.g. triggering sounds, opening doors, etc.)
        // MovingTerrainWaypoint → platform object
        // WaypointReached → waypoint object
        // Implementation deferred to 57e when MessageDispatch is wired up
    }

    // ── Transform helpers ──

    /// Ensure the platform has an ObjectState entry (created on-demand).
    void ensureObjectState(PlatformState &plat) {
        if (plat.hasObjectState || !mObjectStates) return;
        plat.hasObjectState = true;
        applyTransform(plat);
    }

    /// Write the platform's current position to ObjectState.
    /// Platforms translate but don't rotate (the original engine locks rotation),
    /// so the transform is: T(currentPos) * R_base * S_base
    void applyTransform(PlatformState &plat) {
        if (!mObjectStates) return;

        // Compute the offset from base position
        Vector3 offset = plat.currentPos - plat.base.position;

        // Build world matrix: T(currentPos) * R_base * S_base
        Matrix4 worldTranslate = glm::translate(Matrix4(1.0f), plat.currentPos);
        Matrix4 scaleMat = glm::scale(Matrix4(1.0f), plat.base.scale);
        Matrix4 fullGlm = worldTranslate * plat.base.rotation * scaleMat;

        applyModelMatrix(*mObjectStates, plat.objID, fullGlm,
                         plat.currentPos, plat.base.scale);

        // Update collision body
        if (mCollisionUpdateCb) {
            mCollisionUpdateCb(plat.objID, fullGlm);
        }
    }

    // ── Data ──
    std::unordered_map<int32_t, PlatformState> mPlatforms;
    ObjectStateMap *mObjectStates = nullptr;
    ObjectService *mObjSvc = nullptr;
    const std::unordered_map<int32_t, ObjPlacementInfo> *mPlacements = nullptr;
    PlatformCollisionCallback mCollisionUpdateCb;
    MessageDispatch *mMsgDispatch = nullptr;
    uint32_t mFrameCount = 0;
};

} // namespace Darkness
