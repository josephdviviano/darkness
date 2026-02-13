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
 *    PlayerPhysics — custom player movement simulation with sphere-polygon
 *    collision with constraint-based response.
 *
 *    Aims to use a multi-sphere player model (head, body, knee,
 *    shin, foot — 5 spheres total, all r=1.2) with a spring-connected
 *    head for natural head bob. Our Phase 2 implementation starts with a
 *    simplified 2-sphere model (body + head) and adds complexity
 *    progressively. The collision response uses normal removal
 *    (constraint projection), not impulse-based resolution.
 *
 *    Constants derived:
 *    - Walk speed: 11.0 units/sec, Run: 22.0, Creep: 5.5
 *    - Player body sphere radius: 1.2 units
 *    - Head sphere offset: +4.8 units above body center
 *    - Gravity: ~40 units/sec² (runtime-configurable)
 *    - Fixed timestep: 60 Hz (1/60 sec per physics step)
 *
 *    See NOTES.SOURCE.md for full physics constants documentation.
 *
 *****************************************************************************/

#ifndef __PLAYERPHYSICS_H
#define __PLAYERPHYSICS_H

#include <cmath>
#include <cstdint>

#include "DarknessMath.h"

namespace Darkness {

// Forward declarations
class CollisionGeometry;
struct ContactEvent;
using ContactCallback = std::function<void(const ContactEvent &)>;

/// Player movement state flags
enum class PlayerMoveState {
    OnGround,       // walking / standing on a solid surface
    Falling,        // airborne, gravity pulling down
    Swimming,       // in water volume (mediaType == 2)
    Climbing        // on a climbable surface (rope, ladder)
};

/// Player physics simulation — custom sphere-polygon collision.
/// Owns the player's position, velocity, and movement state.
/// Updated each frame by DarkPhysics::step().
class PlayerPhysics {
public:
    // ── Player physics constants ──
    // Values from previous analysis (Task 20)

    // Sphere radii — all spheres share the same radius in the original engine
    static constexpr float SPHERE_RADIUS = 1.2f;

    // Sphere center offsets relative to body center (Z-up)
    // Full 5-sphere model: HEAD +1.8, BODY -0.6, KNEE -2.6, SHIN -2.2, FOOT -3.0
    // Phase 2 simplified: body sphere at center, head sphere above
    static constexpr float HEAD_OFFSET    = 4.8f;    // head center above body center
    static constexpr float STAND_HEIGHT   = 6.0f;    // total standing height
    static constexpr float CROUCH_HEIGHT  = 3.0f;    // total crouching height
    static constexpr float CROUCH_HEAD_OFFSET = 2.4f; // head offset when crouching

    // Movement speeds (world units/sec)
    static constexpr float WALK_SPEED     = 11.0f;
    static constexpr float RUN_SPEED      = 22.0f;
    static constexpr float CREEP_SPEED    = 5.5f;
    static constexpr float SIDESTEP_SPEED = 7.7f;
    static constexpr float BACKWARD_SPEED = 5.5f;
    static constexpr float JUMP_IMPULSE   = 14.0f;   // upward velocity on jump

    // Speed multipliers for movement modes
    static constexpr float CROUCH_MULT    = 0.6f;
    static constexpr float SWIM_MULT      = 0.7f;
    static constexpr float CLIMB_MULT     = 0.5f;

    // Physics parameters
    static constexpr float GRAVITY        = 40.0f;   // units/sec² (runtime-adjustable)
    static constexpr float FIXED_DT       = 1.0f / 60.0f;  // 60 Hz physics timestep
    static constexpr float MAX_SLOPE_COS  = 0.766f;  // cos(~40°), max walkable slope
    static constexpr float STAIR_HEIGHT   = 4.0f;    // max auto-step height
    static constexpr float GROUND_SNAP    = 0.5f;    // snap-to-ground tolerance

    // Collision iteration count — matching the renderer's camera collision
    static constexpr int COLLISION_ITERS  = 3;

    // Air control factor — reduced horizontal control while airborne
    static constexpr float AIR_CONTROL    = 0.3f;

    // Spring-connected head
    static constexpr float HEAD_SPRING_TENSION = 0.6f;
    static constexpr float HEAD_SPRING_DAMPING = 0.02f;

    // ── Construction ──

    /// Create player physics with reference to collision geometry.
    /// The collision geometry must outlive this object.
    explicit PlayerPhysics(const CollisionGeometry &collision);

    // ── Main simulation ──

    /// Advance the player simulation by dt seconds.
    /// Internally steps in FIXED_DT increments. Performs gravity, movement
    /// integration, collision response, and ground detection.
    /// contactCb may be nullptr if no contact notifications are needed.
    void step(float dt, const ContactCallback &contactCb);

    // ── Input ──

    /// Set movement input (normalized to [-1, 1]).
    /// forward > 0 = forward, right > 0 = strafe right.
    void setMovement(float forward, float right);

    /// Set the player's look direction yaw (radians).
    /// Movement is oriented relative to this yaw.
    void setYaw(float yaw);

    /// Initiate a jump (only if on ground).
    void jump();

    /// Set crouching state. Cannot uncrouch if ceiling blocks.
    void setCrouching(bool crouching);

    // ── State queries ──

    /// Body center position (not eye position)
    const Vector3 &getPosition() const { return mPosition; }

    /// Eye position (head sphere center — above body center)
    Vector3 getEyePosition() const;

    /// Linear velocity
    const Vector3 &getVelocity() const { return mVelocity; }

    /// Current WR cell index (-1 if outside all cells)
    int32_t getCell() const { return mCellIdx; }

    /// Is the player standing on a walkable surface?
    bool isOnGround() const { return mMoveState == PlayerMoveState::OnGround; }

    /// Current movement state
    PlayerMoveState getMoveState() const { return mMoveState; }

    /// Is the player crouching?
    bool isCrouching() const { return mCrouching; }

    // ── Teleport ──

    /// Set player position directly (for spawn, teleport).
    /// Also resets velocity and updates cell index.
    void setPosition(const Vector3 &pos);

    /// Set gravity magnitude (units/sec²). Default is GRAVITY.
    void setGravityMagnitude(float g) { mGravityMag = g; }

private:
    // ── Internal simulation steps ──

    /// Single fixed-timestep physics step
    void fixedStep(const ContactCallback &contactCb);

    /// Apply gravity to velocity (if not on ground)
    void applyGravity();

    /// Compute desired horizontal velocity from input + yaw
    Vector3 computeDesiredVelocity() const;

    /// Apply movement input to velocity (ground vs air control)
    void applyMovement();

    /// Integrate position from velocity
    void integrate();

    /// Run sphere-vs-cell collision and constraint projection
    void resolveCollisions(const ContactCallback &contactCb);

    /// Detect ground contact and update movement state
    void detectGround();

    /// Snap player to ground when walking downhill
    void snapToGround();

    /// Update cell index from current position
    void updateCell();

    // ── State ──

    const CollisionGeometry &mCollision;  // world collision geometry (not owned)

    Vector3 mPosition{0.0f};       // body center position
    Vector3 mVelocity{0.0f};       // linear velocity
    float mYaw = 0.0f;            // look direction (radians)
    float mInputForward = 0.0f;   // movement input [-1, 1]
    float mInputRight   = 0.0f;   // strafe input [-1, 1]

    int32_t mCellIdx = -1;        // current WR cell index
    PlayerMoveState mMoveState = PlayerMoveState::Falling;

    bool mCrouching    = false;   // currently crouching
    bool mWantsCrouch  = false;   // crouch input pressed
    bool mJumpRequested = false;  // jump pending next step

    float mGravityMag = GRAVITY;  // current gravity magnitude
    float mTimeAccum  = 0.0f;     // fixed-timestep accumulator

    // Head spring state (for natural head bob)
    float mHeadOffset     = HEAD_OFFSET;  // current head offset
    float mHeadVelocity   = 0.0f;         // head spring velocity

    // Ground contact normal from last collision pass
    Vector3 mGroundNormal{0.0f, 0.0f, 1.0f};
};

} // namespace Darkness

#endif // __PLAYERPHYSICS_H
