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

// EdgeTriggerSystem.h — Volume trigger simulation
//
// Implements the Dark Engine edge trigger system: objects with the
// P$PhysAttr.edgeTrigger flag that detect player entry/exit and send
// PhysEnter/PhysExit script messages. Edge triggers are pass-through
// volumes — they generate no physical collision response.
//
// The trigger volume is the object's OBB (oriented bounding box) from
// the collision system. Player position is tested against each edge
// trigger OBB each frame; transitions are detected as enter/exit events.
//
// Inspired by the openDarkEngine (OPDE) service architecture (GPLv2).
// Edge trigger algorithm reimplemented from Dark Engine behavior analysis.

#pragma once

#include <cstdint>
#include <cmath>
#include <cstring>
#include <unordered_set>
#include <functional>

#include "DarknessMath.h"
#include "SimCommon.h"
#include "MessageDispatch.h"
#include "physics/ObjectCollisionGeometry.h"

namespace Darkness {

// ── Player position query callback ──
using PlayerPositionCallback = std::function<Vector3()>;

// ============================================================================
// EdgeTriggerSystem — manages all edge trigger volumes in the level
// ============================================================================

class EdgeTriggerSystem : public SimListener {
public:
    EdgeTriggerSystem() = default;

    // ── Initialization ──

    /// Scan the ObjectCollisionWorld for all bodies with isEdgeTrigger=true.
    /// Call after buildObjectCollision() has populated collision bodies.
    void init(const ObjectCollisionWorld *ocw) {
        mCollisionWorld = ocw;
        if (!ocw) return;

        // Collect all edge trigger object IDs
        const auto &bodies = ocw->getBodies();
        for (size_t i = 0; i < bodies.size(); ++i) {
            if (bodies[i].isEdgeTrigger) {
                mTriggerIDs.push_back(bodies[i].objID);
            }
        }

        std::fprintf(stderr, "EdgeTriggerSystem: %zu edge triggers\n",
                     mTriggerIDs.size());
    }

    // ── Callbacks ──

    void setPlayerPositionCallback(PlayerPositionCallback cb) {
        mPlayerPosCb = std::move(cb);
    }

    void setMessageDispatch(MessageDispatch *msgDispatch) {
        mMsgDispatch = msgDispatch;
    }

    // ── SimListener interface ──

    void simStep(float simTime, float delta) override {
        if (delta <= 0.0f || !mCollisionWorld || !mPlayerPosCb) return;

        Vector3 playerPos = mPlayerPosCb();

        for (int32_t objID : mTriggerIDs) {
            const ObjectCollisionBody *body = mCollisionWorld->findBodyByObjID(objID);
            if (!body) continue;

            bool inside = isPointInsideOBB(playerPos, *body);
            bool wasInside = (mInsideTriggers.find(objID) != mInsideTriggers.end());

            if (inside && !wasInside) {
                // Player just entered this trigger volume
                mInsideTriggers.insert(objID);
                std::fprintf(stderr, "  EdgeTrigger %d: player ENTERED\n", objID);

                if (mMsgDispatch) {
                    // Send PhysEnter message to the trigger object
                    mMsgDispatch->sendMessage({objID, "PhysEnter", 0, {}});
                    // Also send TurnOn via ControlDevice links for basic
                    // functionality without the script system
                    mMsgDispatch->turnOn(objID, objID);
                }

            } else if (!inside && wasInside) {
                // Player just exited this trigger volume
                mInsideTriggers.erase(objID);
                std::fprintf(stderr, "  EdgeTrigger %d: player EXITED\n", objID);

                if (mMsgDispatch) {
                    // Send PhysExit message to the trigger object
                    mMsgDispatch->sendMessage({objID, "PhysExit", 0, {}});
                    // Also send TurnOff via ControlDevice links
                    mMsgDispatch->turnOff(objID, objID);
                }
            }
        }
    }

private:
    /// Test if a point is inside an OBB using slab decomposition.
    /// The OBB is defined by center position, half-extents (edgeLengths/2),
    /// and a 3x3 rotation matrix whose columns are the OBB local axes.
    static bool isPointInsideOBB(const Vector3 &point,
                                  const ObjectCollisionBody &body) {
        // Vector from OBB center to point
        Vector3 d = point - body.worldPos;

        // Half-extents (edgeLengths are full extents)
        Vector3 halfExtents = body.edgeLengths * 0.5f;

        // Project d onto each local axis and check against half-extent.
        // body.rotation columns are the OBB's local X, Y, Z axes.
        // Add a small epsilon (0.01 units) for hysteresis matching the
        // original engine's inside_epsilon for edge triggers.
        constexpr float EPSILON = 0.01f;

        for (int i = 0; i < 3; ++i) {
            Vector3 axis = body.rotation[i];  // column i = local axis i
            float proj = glm::dot(d, axis);
            if (std::abs(proj) > halfExtents[i] + EPSILON)
                return false;
        }
        return true;
    }

    // ── Data ──
    const ObjectCollisionWorld *mCollisionWorld = nullptr;
    std::vector<int32_t> mTriggerIDs;            // all edge trigger object IDs
    std::unordered_set<int32_t> mInsideTriggers;  // triggers player is currently inside
    PlayerPositionCallback mPlayerPosCb;
    MessageDispatch *mMsgDispatch = nullptr;
};

} // namespace Darkness
