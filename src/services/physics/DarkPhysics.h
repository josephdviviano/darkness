/******************************************************************************
 *
 *    This file is part of the Darkness engine
 *    Copyright (C) 2024-2026 Darkness contributors
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

/******************************************************************************
 *
 *    DarkPhysics — concrete IPhysicsWorld implementation.
 *
 *    Hybrid architecture:
 *    - Player body: managed by PlayerPhysics
 *      (custom sphere-polygon collision constraint-based model)
 *    - World objects: managed by ODE rigid body dynamics (Task 26, deferred)
 *
 *    Current Phase 2 scope:
 *    - Player movement with collision, gravity, ground detection
 *    - Raycast delegation to the existing portal-graph BFS raycaster
 *    - Body management stubs for ODE integration in Task 26
 *
 *    Raycast and sweepSphere delegate to the existing RayCaster.h which
 *    uses BFS portal graph traversal — efficient for typical Thief 2
 *    missions with ~200-1500 cells.
 *
 *****************************************************************************/

#ifndef __DARKPHYSICS_H
#define __DARKPHYSICS_H

#include <memory>
#include <unordered_map>

#include "IPhysicsWorld.h"
#include "CollisionGeometry.h"
#include "PlayerPhysics.h"
#include "RayCaster.h"

namespace Darkness {

/// Concrete IPhysicsWorld implementation for the Darkness engine.
/// Owns the CollisionGeometry, PlayerPhysics, and (in Task 26) ODE world.
class DarkPhysics : public IPhysicsWorld {
public:
    /// Construct the physics world from parsed WR cell data.
    /// Takes ownership of the collision geometry built from wr.
    /// The WRParsedData must outlive this object.
    explicit DarkPhysics(const WRParsedData &wr)
        : mCollision(wr)
        , mPlayer(mCollision)
        , mGravity(0.0f, 0.0f, -PlayerPhysics::GRAVITY)
    {
    }

    ~DarkPhysics() override = default;

    // ── World management ──

    void step(float dt) override {
        // Step player physics
        mPlayer.step(dt, mContactCb);

        // TODO (Task 26): Step ODE world for non-player bodies
        // dWorldQuickStep(mODEWorld, PlayerPhysics::FIXED_DT);
    }

    void addBody(EntityID id, const PhysicsBodyDesc &desc) override {
        // TODO (Task 26): Create ODE body + geom from desc
        // For now, store the descriptor for later ODE integration
        mBodyDescs[id] = desc;
    }

    void removeBody(EntityID id) override {
        // TODO (Task 26): Destroy ODE body + geom
        mBodyDescs.erase(id);
    }

    void setGravity(const Vector3 &g) override {
        mGravity = g;
        // Update player gravity magnitude from Z component
        // (Z-up: gravity is negative Z)
        mPlayer.setGravityMagnitude(std::fabs(g.z));

        // TODO (Task 26): dWorldSetGravity(mODEWorld, g.x, g.y, g.z);
    }

    // ── Force / impulse ──

    void applyForce(EntityID id, const Vector3 &force) override {
        // TODO (Task 26): dBodyAddForce on ODE body
        (void)id; (void)force;
    }

    void applyImpulse(EntityID id, const Vector3 &impulse) override {
        // TODO (Task 26): velocity change via dBodySetLinearVel
        (void)id; (void)impulse;
    }

    // ── Spatial queries ──

    bool raycast(const Vector3 &from, const Vector3 &to,
                 RayHit *hit) const override {
        if (!hit) {
            // Just test for any hit
            RayHit tmp;
            return raycastWorld(mCollision.getWR(), from, to, tmp);
        }
        return raycastWorld(mCollision.getWR(), from, to, *hit);
    }

    bool sweepSphere(const Vector3 &from, const Vector3 &to,
                     float radius, RayHit *hit) const override {
        // Phase 2 approximation: offset the ray by radius along the
        // direction and do a standard raycast. This is conservative
        // (may miss narrow gaps) but fast. Full sphere sweep deferred.
        Vector3 dir = to - from;
        float len = glm::length(dir);
        if (len < 1e-6f)
            return false;
        dir /= len;

        // Shorten the ray by radius at both ends for conservative estimate
        Vector3 offsetFrom = from + dir * radius;
        Vector3 offsetTo   = to   - dir * radius;
        if (glm::dot(offsetTo - offsetFrom, dir) <= 0.0f)
            return false; // ray too short for sphere

        RayHit tmp;
        if (raycastWorld(mCollision.getWR(), offsetFrom, offsetTo, tmp)) {
            if (hit) {
                *hit = tmp;
                // Adjust distance back to account for offset
                hit->distance += radius;
            }
            return true;
        }
        return false;
    }

    std::vector<EntityID> overlapSphere(const Vector3 &center,
                                        float radius) const override {
        // TODO (Task 26): Query ODE space or SpatialIndex
        (void)center; (void)radius;
        return {};
    }

    // ── Body state ──

    Vector3 getPosition(EntityID id) const override {
        // TODO (Task 26): dBodyGetPosition for ODE bodies
        auto it = mBodyDescs.find(id);
        if (it != mBodyDescs.end())
            return it->second.position;
        return Vector3(0.0f);
    }

    Vector3 getVelocity(EntityID id) const override {
        // TODO (Task 26): dBodyGetLinearVel for ODE bodies
        (void)id;
        return Vector3(0.0f);
    }

    void setPosition(EntityID id, const Vector3 &pos) override {
        // TODO (Task 26): dBodySetPosition for ODE bodies
        auto it = mBodyDescs.find(id);
        if (it != mBodyDescs.end())
            it->second.position = pos;
    }

    void setVelocity(EntityID id, const Vector3 &vel) override {
        // TODO (Task 26): dBodySetLinearVel for ODE bodies
        (void)id; (void)vel;
    }

    // ── Collision events ──

    void setContactCallback(ContactCallback cb) override {
        mContactCb = std::move(cb);
    }

    // ── Player-specific ──

    Vector3 getPlayerPosition() const override {
        return mPlayer.getPosition();
    }

    Vector3 getPlayerEyePosition() const override {
        return mPlayer.getEyePosition();
    }

    void setPlayerPosition(const Vector3 &pos) override {
        mPlayer.setPosition(pos);
    }

    void setPlayerMovement(float forward, float right) override {
        mPlayer.setMovement(forward, right);
    }

    void setPlayerYaw(float yaw) override {
        mPlayer.setYaw(yaw);
    }

    void playerJump() override {
        mPlayer.jump();
    }

    void setPlayerCrouching(bool crouching) override {
        mPlayer.setCrouching(crouching);
    }

    void setPlayerSneaking(bool sneaking) override {
        mPlayer.setSneaking(sneaking);
    }

    void setPlayerRunning(bool running) override {
        mPlayer.setRunning(running);
    }

    bool isPlayerOnGround() const override {
        return mPlayer.isOnGround();
    }

    int32_t getPlayerCell() const override {
        return mPlayer.getCell();
    }

    Vector3 getPlayerVelocity() const override {
        return mPlayer.getVelocity();
    }

    void setFootstepCallback(FootstepCallback cb) override {
        mFootstepCb = std::move(cb);
        // TODO: Fire from PlayerPhysics stride triggering
    }

    int getPlayerMode() const override {
        return static_cast<int>(mPlayer.getMode());
    }

    bool isPlayerMantling() const override {
        return mPlayer.isMantling();
    }

    void disablePlayerMotion() override {
        mPlayer.disableMotion();
    }

    void enablePlayerMotion() override {
        mPlayer.enableMotion();
    }

    // ── Direct access for renderer integration ──

    /// Access the collision geometry (for renderer camera collision fallback)
    const CollisionGeometry &getCollisionGeometry() const { return mCollision; }

    /// Access the player physics (for debug display, etc.)
    const PlayerPhysics &getPlayerPhysics() const { return mPlayer; }
    PlayerPhysics &getPlayerPhysics() { return mPlayer; }

private:
    CollisionGeometry mCollision;    // world collision geometry from WR cells
    PlayerPhysics mPlayer;           // custom player movement simulation

    Vector3 mGravity;                // world gravity vector
    ContactCallback mContactCb;      // optional contact notification callback
    FootstepCallback mFootstepCb;    // optional footstep sound callback

    // Stored body descriptors for deferred ODE integration (Task 26)
    std::unordered_map<EntityID, PhysicsBodyDesc> mBodyDescs;

    // TODO (Task 26): ODE world state
    // dWorldID mODEWorld = nullptr;
    // dSpaceID mODESpace = nullptr;
    // dJointGroupID mODEContacts = nullptr;
    // std::unordered_map<EntityID, dBodyID> mODEBodies;
    // std::unordered_map<EntityID, dGeomID> mODEGeoms;
};

} // namespace Darkness

#endif // __DARKPHYSICS_H
