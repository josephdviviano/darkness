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
 *    IPhysicsWorld — the physics interface through which the renderer and
 *    downstream subsystems interact with the physics simulation.
 *
 *    Design: hybrid physics model.
 *    - Player movement uses custom sphere-vs-cell-polygon collision
 *      (constraint-based response, not impulse-based).
 *    - World objects use ODE rigid body dynamics (Task 26).
 *    - Player-specific convenience methods wrap the internal PlayerPhysics
 *      state machine; the general body API handles everything else.
 *
 *    Consumers:
 *    - Renderer: step(), player position/eye for camera, cell for culling
 *    - Audio (Phase 3): contact callbacks for impact sounds
 *    - AI (Phase 4): contact callbacks for footstep loudness
 *    - Scripts (Phase 5): contact callbacks for pressure plates, traps
 *
 *****************************************************************************/

#ifndef __IPHYSICSWORLD_H
#define __IPHYSICSWORLD_H

#include <cstdint>
#include <functional>
#include <vector>

#include "DarknessMath.h"
#include "worldquery/WorldQueryTypes.h"

namespace Darkness {

// ============================================================================
// Shape and body type enums
// ============================================================================

/// Collision shape primitive type
enum class PhysShapeType {
    Sphere,     // radius only
    Capsule,    // radius + halfHeight (cylindrical section)
    Box,        // halfExtents (3D)
    Mesh        // ODE trimesh for complex static geometry
};

/// Physics body simulation type
enum class PhysBodyType {
    Static,     // immovable world geometry, doors in closed state
    Kinematic,  // script-driven (elevators, moving platforms)
    Dynamic     // fully simulated (thrown objects, physics puzzles)
};

// ============================================================================
// PhysicsBodyDesc — describes a physics body to be created
// ============================================================================

/// Complete description for creating a physics body via addBody().
/// Shape parameters are union-like: only the fields relevant to the
/// chosen PhysShapeType are used.
struct PhysicsBodyDesc {
    PhysShapeType shape = PhysShapeType::Sphere;
    PhysBodyType  type  = PhysBodyType::Dynamic;

    // Shape parameters (depends on shape type)
    float radius     = 1.0f;       // Sphere, Capsule
    float halfHeight = 0.0f;       // Capsule (half-height of cylindrical section)
    Vector3 halfExtents{1.0f};     // Box

    // Material properties
    float mass        = 1.0f;
    float friction    = 0.5f;
    float restitution = 0.0f;      // 0 = no bounce
    float density     = 0.9f;      // for buoyancy calculations

    // Initial state
    Vector3 position{0.0f};
    Quaternion orientation{1.0f, 0.0f, 0.0f, 0.0f};

    // Behavior flags
    bool gravity = true;           // affected by world gravity
    bool sensor  = false;          // generates contacts but no physics response (triggers)
};

// ============================================================================
// ContactEvent — collision notification data
// ============================================================================

/// Fired during step() for each contact pair. The collision response
/// (pushing bodies apart) happens regardless of whether a callback is
/// registered — this event is for observation only.
///
/// Phase 2: callback is optional (nullptr = no notifications).
/// Phase 3+: audio uses materialA/B for impact sound selection.
/// Phase 4+: AI uses player-ground contacts for footstep loudness.
/// Phase 5+: scripts use contacts for pressure plates and trap triggers.
struct ContactEvent {
    EntityID bodyA;
    EntityID bodyB;
    Vector3 point;                 // world-space contact point
    Vector3 normal;                // contact normal (A → B direction)
    float depth;                   // penetration depth (positive)
    int32_t materialA = -1;        // texture index for surface material lookup
    int32_t materialB = -1;
};

/// Callback type for contact notifications
using ContactCallback = std::function<void(const ContactEvent &)>;

/// Callback type for footstep sound events.
/// Fired when the player's foot travel distance exceeds the stride threshold.
/// pos = player foot position, speed = horizontal speed, materialIdx = ground texture
using FootstepCallback = std::function<void(const Vector3 &pos, float speed, int materialIdx)>;

// ============================================================================
// IPhysicsWorld — the physics interface
// ============================================================================

class IPhysicsWorld {
public:
    virtual ~IPhysicsWorld() = default;

    // ── World management ──

    /// Advance the simulation by dt seconds. Internally uses fixed timestep
    /// accumulation (60 Hz). Player and ODE bodies step together.
    virtual void step(float dt) = 0;

    /// Create a physics body for the given entity. Shape, mass, and initial
    /// state come from the descriptor. Player body is created internally
    /// and should not use this method.
    virtual void addBody(EntityID id, const PhysicsBodyDesc &desc) = 0;

    /// Remove a physics body. No-op if entity has no body.
    virtual void removeBody(EntityID id) = 0;

    /// Set world gravity vector. Default is (0, 0, -40) for Z-up.
    virtual void setGravity(const Vector3 &g) = 0;

    // ── Force / impulse application ──

    /// Apply a continuous force (integrated over dt). For throwing, wind, etc.
    virtual void applyForce(EntityID id, const Vector3 &force) = 0;

    /// Apply an instantaneous impulse (velocity change). For impacts, jumps.
    virtual void applyImpulse(EntityID id, const Vector3 &impulse) = 0;

    // ── Spatial queries ──

    /// Cast a ray from 'from' to 'to'. Returns true if any geometry is hit,
    /// filling 'hit' with the closest intersection. Delegates to the portal
    /// graph BFS raycaster for world geometry.
    virtual bool raycast(const Vector3 &from, const Vector3 &to,
                         RayHit *hit) const = 0;

    /// Sweep a sphere along a ray. Returns true if the sphere hits geometry.
    /// Phase 2: approximated via raycaster with radius offset.
    virtual bool sweepSphere(const Vector3 &from, const Vector3 &to,
                             float radius, RayHit *hit) const = 0;

    /// Find all entities whose physics bodies overlap a sphere.
    /// Useful for area-of-effect queries, explosion damage, etc.
    virtual std::vector<EntityID> overlapSphere(const Vector3 &center,
                                                float radius) const = 0;

    // ── Body state queries ──

    /// Get world-space position of a physics body (center of mass).
    virtual Vector3 getPosition(EntityID id) const = 0;

    /// Get linear velocity of a physics body.
    virtual Vector3 getVelocity(EntityID id) const = 0;

    /// Teleport a physics body to a new position. Clears velocity.
    virtual void setPosition(EntityID id, const Vector3 &pos) = 0;

    /// Set linear velocity directly. For scripted movement.
    virtual void setVelocity(EntityID id, const Vector3 &vel) = 0;

    // ── Collision events ──

    /// Register a callback for contact notifications. Pass nullptr to disable.
    /// The callback is invoked during step() for each contact pair.
    virtual void setContactCallback(ContactCallback cb) = 0;

    // ── Player-specific convenience methods ──
    // These wrap the internal PlayerPhysics state machine.
    // The player is a special body (PHYS_MDL_PLAYER) not managed by ODE.

    /// Player body center position (body sphere, not eye)
    virtual Vector3 getPlayerPosition() const = 0;

    /// Player eye position (head sphere center — above body center)
    virtual Vector3 getPlayerEyePosition() const = 0;

    /// Teleport player to a new position. Used for spawn and level transitions.
    virtual void setPlayerPosition(const Vector3 &pos) = 0;

    /// Set player movement input. forward > 0 = forward, right > 0 = strafe right.
    /// Values are normalized to [-1, 1] range (actual speed comes from constants).
    /// The player's yaw determines the world-space movement direction.
    virtual void setPlayerMovement(float forward, float right) = 0;

    /// Set the player's look direction (yaw in radians). Used by the camera
    /// to orient movement relative to where the player is looking.
    virtual void setPlayerYaw(float yaw) = 0;

    /// Initiate a jump if the player is on the ground.
    virtual void playerJump() = 0;

    /// Toggle crouching. Reduces player height and movement speed.
    /// Cannot uncrouch if ceiling is too low.
    virtual void setPlayerCrouching(bool crouching) = 0;

    /// Toggle sneaking (creep mode). Reduces movement speed by 0.5x.
    /// Cannot stack with running — sneaking takes priority.
    virtual void setPlayerSneaking(bool sneaking) = 0;

    /// Toggle running (sprint). Doubles movement speed.
    /// If sneaking is also active, sneaking takes priority.
    virtual void setPlayerRunning(bool running) = 0;

    /// True if the player is standing on a walkable surface.
    virtual bool isPlayerOnGround() const = 0;

    /// Current WR cell index containing the player. Used for portal culling.
    virtual int32_t getPlayerCell() const = 0;

    /// Player linear velocity (for footstep timing, sound, etc.)
    virtual Vector3 getPlayerVelocity() const = 0;

    /// Register a callback for footstep sound events. Called when the player's
    /// foot travel distance exceeds the stride threshold (distance-based).
    virtual void setFootstepCallback(FootstepCallback cb) = 0;

    /// Query the player's current mode (Stand, Crouch, Swim, Jump, etc.)
    virtual int getPlayerMode() const = 0;

    /// True if the player is in the middle of a mantle animation.
    virtual bool isPlayerMantling() const = 0;

    /// Disable all player motion (for cutscenes, death sequences).
    virtual void disablePlayerMotion() = 0;

    /// Re-enable player motion after being disabled.
    virtual void enablePlayerMotion() = 0;
};

} // namespace Darkness

#endif // __IPHYSICSWORLD_H
