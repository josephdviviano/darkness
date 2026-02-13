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
 *    collision and constraint-based response. Header-only (inline),
 *    matching CellGeometry.h / RayCaster.h / CollisionGeometry.h pattern.
 *
 *    Phase 2 uses a simplified 2-sphere model (body + head) with the
 *    full 5-sphere model planned for later. Collision response uses
 *    normal removal (constraint projection), not impulse-based resolution.
 *
 *    Constants:
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
#include <functional>
#include <vector>

#include "DarknessMath.h"
#include "CollisionGeometry.h"
#include "IPhysicsWorld.h"

namespace Darkness {

/// Player movement state
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

    // Sphere radii — all spheres use the same radius
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

    // Friction deceleration rate (units/sec² when no input on ground)
    static constexpr float GROUND_FRICTION = 60.0f;

    // ── Construction ──

    /// Create player physics with reference to collision geometry.
    /// The collision geometry must outlive this object.
    explicit PlayerPhysics(const CollisionGeometry &collision)
        : mCollision(collision) {}

    // ── Main simulation ──

    /// Advance the player simulation by dt seconds.
    /// Internally steps in FIXED_DT increments. Performs gravity, movement
    /// integration, collision response, and ground detection.
    /// contactCb may be empty if no contact notifications are needed.
    inline void step(float dt, const ContactCallback &contactCb) {
        mTimeAccum += dt;

        // Fixed timestep accumulation — step at 60 Hz
        while (mTimeAccum >= FIXED_DT) {
            mTimeAccum -= FIXED_DT;
            fixedStep(contactCb);
        }
    }

    // ── Input ──

    /// Set movement input (normalized to [-1, 1]).
    /// forward > 0 = forward, right > 0 = strafe right.
    inline void setMovement(float forward, float right) {
        mInputForward = forward;
        mInputRight = right;
    }

    /// Set the player's look direction yaw (radians).
    /// Movement is oriented relative to this yaw.
    inline void setYaw(float yaw) { mYaw = yaw; }

    /// Initiate a jump (only if on ground).
    inline void jump() { mJumpRequested = true; }

    /// Set crouching state. Cannot un-crouch if ceiling blocks.
    inline void setCrouching(bool crouching) { mWantsCrouch = crouching; }

    // ── State queries ──

    /// Body center position (not eye position)
    const Vector3 &getPosition() const { return mPosition; }

    /// Eye position (head sphere center — above body center)
    inline Vector3 getEyePosition() const {
        return mPosition + Vector3(0.0f, 0.0f, mHeadOffset);
    }

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
    /// If findCell fails at the given position, tries eye level as fallback
    /// (the caller often passes the camera/eye position as body center,
    /// and the cell geometry may not extend below the floor).
    inline void setPosition(const Vector3 &pos) {
        mPosition = pos;
        mVelocity = Vector3(0.0f);
        mCellIdx = mCollision.findCell(pos);
        if (mCellIdx < 0) {
            // Try at eye level — the camera position is usually inside a cell
            // even when the body center (below eye) would be outside
            mCellIdx = mCollision.findCell(pos + Vector3(0.0f, 0.0f, HEAD_OFFSET));
        }
        if (mCellIdx < 0) {
            // Last resort: try slightly above the given position
            mCellIdx = mCollision.findCell(pos + Vector3(0.0f, 0.0f, SPHERE_RADIUS));
        }
        mMoveState = PlayerMoveState::Falling; // will detect ground on next step
    }

    /// Set gravity magnitude (units/sec²). Default is GRAVITY.
    void setGravityMagnitude(float g) { mGravityMag = g; }

private:
    // ── Internal simulation steps ──

    /// Single fixed-timestep physics step
    inline void fixedStep(const ContactCallback &contactCb) {
        // 1. Handle jump request
        if (mJumpRequested) {
            if (mMoveState == PlayerMoveState::OnGround) {
                mVelocity.z = JUMP_IMPULSE;
                mMoveState = PlayerMoveState::Falling;
            }
            mJumpRequested = false;
        }

        // 2. Handle crouch state
        updateCrouch();

        // 3. Apply gravity (if airborne)
        applyGravity();

        // 4. Apply horizontal movement from input
        applyMovement();

        // 5. Integrate position
        integrate();

        // 6. Update cell index BEFORE collision — if integration moved
        //    the player through a portal, collision must test against
        //    the new cell's walls, not the old cell's.
        updateCell();

        // 7. Resolve collisions via constraint projection
        resolveCollisions(contactCb);

        // 8. Detect ground and update movement state
        detectGround();

        // 9. Snap to ground when walking on slopes (prevents hopping)
        if (mMoveState == PlayerMoveState::OnGround) {
            snapToGround();
        }

        // 10. Update cell index again (collision may have shifted position)
        updateCell();

        // 11. Update head spring (natural head bob)
        updateHeadSpring();
    }

    /// Apply gravity to velocity (if not on ground)
    inline void applyGravity() {
        if (mMoveState != PlayerMoveState::OnGround) {
            // Z-up: gravity pulls downward
            mVelocity.z -= mGravityMag * FIXED_DT;
        }
    }

    /// Compute desired horizontal velocity from input + yaw.
    /// Forward direction is determined by the player's yaw angle.
    /// Z-up coordinate system: forward = (cos(yaw), sin(yaw), 0)
    inline Vector3 computeDesiredVelocity() const {
        if (std::fabs(mInputForward) < 0.001f && std::fabs(mInputRight) < 0.001f)
            return Vector3(0.0f);

        // Forward and right vectors in Z-up coordinate system
        // Forward = (cos(yaw), sin(yaw), 0)
        // Right = (sin(yaw), -cos(yaw), 0)
        float cosYaw = std::cos(mYaw);
        float sinYaw = std::sin(mYaw);
        Vector3 forward(cosYaw, sinYaw, 0.0f);
        Vector3 right(sinYaw, -cosYaw, 0.0f);

        // Determine speed based on movement direction
        float forwardSpeed = WALK_SPEED;
        float strafeSpeed  = SIDESTEP_SPEED;

        // Backward movement is slower
        if (mInputForward < 0.0f)
            forwardSpeed = BACKWARD_SPEED;

        // Crouching reduces speed
        if (mCrouching) {
            forwardSpeed *= CROUCH_MULT;
            strafeSpeed  *= CROUCH_MULT;
        }

        Vector3 desired = forward * (mInputForward * forwardSpeed)
                        + right   * (mInputRight   * strafeSpeed);

        return desired;
    }

    /// Apply movement input to velocity (ground vs air control)
    inline void applyMovement() {
        Vector3 desired = computeDesiredVelocity();

        if (mMoveState == PlayerMoveState::OnGround) {
            // On ground: directly set horizontal velocity to desired.
            // Project desired velocity onto ground plane so walking on slopes
            // doesn't fight gravity.
            if (mGroundNormal.z > MAX_SLOPE_COS) {
                // Walkable slope — project movement onto ground plane
                // Remove the component along the ground normal from desired
                float dot = glm::dot(desired, mGroundNormal);
                desired -= mGroundNormal * dot;
            }

            // Apply friction when no input: decelerate toward zero
            float desiredLen = glm::length(desired);
            if (desiredLen < 0.01f) {
                // No input — apply friction to slow down
                float speed = std::sqrt(mVelocity.x * mVelocity.x +
                                        mVelocity.y * mVelocity.y);
                if (speed > 0.01f) {
                    float decel = GROUND_FRICTION * FIXED_DT;
                    float newSpeed = std::max(0.0f, speed - decel);
                    float ratio = newSpeed / speed;
                    mVelocity.x *= ratio;
                    mVelocity.y *= ratio;
                }
            } else {
                // Has input — set velocity directly from slope-projected desired.
                // Z must be SET (not +=) to prevent accumulation on slopes —
                // adding desired.z each frame would launch the player off ramps.
                mVelocity.x = desired.x;
                mVelocity.y = desired.y;
                mVelocity.z = desired.z;
            }
        } else {
            // Airborne: reduced horizontal control (air strafe)
            float desiredLen = glm::length(desired);
            if (desiredLen > 0.01f) {
                // Apply air control as acceleration toward desired velocity
                Vector3 accel = desired * AIR_CONTROL;
                mVelocity.x += accel.x * FIXED_DT;
                mVelocity.y += accel.y * FIXED_DT;
            }
        }
    }

    /// Integrate position from velocity
    inline void integrate() {
        mPosition += mVelocity * FIXED_DT;
    }

    /// Run sphere-vs-cell collision and constraint projection.
    /// Uses CollisionGeometry::resolveCollisions() which matches
    /// the camera collision algorithm (iterative push along contact normals).
    inline void resolveCollisions(const ContactCallback &contactCb) {
        std::vector<SphereContact> contacts;
        mCollision.resolveCollisions(mPosition, SPHERE_RADIUS, mCellIdx,
                                     contacts, COLLISION_ITERS);

        // Fire contact callbacks for downstream consumers (audio, AI, scripts)
        if (contactCb) {
            for (const auto &c : contacts) {
                ContactEvent event;
                event.bodyA = -1; // player entity (archetype ID, placeholder)
                event.bodyB = 0;  // world geometry
                event.point = mPosition - c.normal * (SPHERE_RADIUS - c.penetration);
                event.normal = c.normal;
                event.depth = c.penetration;
                event.materialA = -1;
                event.materialB = c.textureIdx;
                contactCb(event);
            }
        }

        // Remove velocity components that go into contact surfaces.
        // This is the constraint-based response: for each contact normal,
        // remove the velocity component pointing into the surface.
        for (const auto &c : contacts) {
            float vn = glm::dot(mVelocity, c.normal);
            if (vn < 0.0f) {
                // Velocity points into the surface — remove that component
                mVelocity -= c.normal * vn;
            }
        }

        // Check for ground contacts in this iteration's results.
        // A ground contact has an upward-facing normal (normal.z > MAX_SLOPE_COS)
        mLastContacts = std::move(contacts);
    }

    /// Detect ground contact and update movement state.
    /// Uses contact normals from the last collision pass — if any contact
    /// has an upward-facing normal (z > MAX_SLOPE_COS), player is on ground.
    inline void detectGround() {
        bool onGround = false;

        // Check contacts from collision resolution
        for (const auto &c : mLastContacts) {
            if (c.normal.z > MAX_SLOPE_COS) {
                onGround = true;
                mGroundNormal = c.normal;
                break;
            }
        }

        // If no contacts found, do a ground probe
        if (!onGround && mCellIdx >= 0) {
            Vector3 probeNormal;
            if (mCollision.groundTest(mPosition, SPHERE_RADIUS, mCellIdx,
                                      GROUND_SNAP, probeNormal)) {
                if (probeNormal.z > MAX_SLOPE_COS) {
                    onGround = true;
                    mGroundNormal = probeNormal;
                }
            }
        }

        // Update movement state
        if (onGround) {
            if (mMoveState == PlayerMoveState::Falling) {
                // Landing — zero vertical velocity
                if (mVelocity.z < 0.0f)
                    mVelocity.z = 0.0f;
            }
            mMoveState = PlayerMoveState::OnGround;
        } else {
            if (mMoveState == PlayerMoveState::OnGround) {
                // Just walked off an edge — start falling
                mMoveState = PlayerMoveState::Falling;
            }
        }
    }

    /// Snap player to ground when walking on slopes.
    /// Prevents "hopping" when walking downhill and keeps the player
    /// glued to the surface when walking uphill.
    /// This is only called when mMoveState == OnGround, so jump velocity
    /// (which immediately sets state to Falling) never reaches here.
    inline void snapToGround() {
        if (mCellIdx < 0)
            return;

        Vector3 probeNormal;
        if (mCollision.groundTest(mPosition, SPHERE_RADIUS, mCellIdx,
                                  GROUND_SNAP * 2.0f, probeNormal)) {
            // Ground is close below — push down to contact
            Vector3 testPos = mPosition;
            testPos.z -= GROUND_SNAP;
            auto contacts = mCollision.sphereVsCellPolygons(testPos, SPHERE_RADIUS, mCellIdx);
            for (const auto &c : contacts) {
                if (c.normal.z > MAX_SLOPE_COS) {
                    // Snap to this surface
                    mPosition.z -= (GROUND_SNAP - c.penetration);
                    if (mVelocity.z < 0.0f)
                        mVelocity.z = 0.0f;
                    break;
                }
            }
        }
    }

    /// Update cell index from current position
    inline void updateCell() {
        int32_t newCell = mCollision.findCell(mPosition);
        if (newCell >= 0)
            mCellIdx = newCell;
        // If newCell < 0, keep the old cell (player might be at a boundary)
    }

    /// Update crouch state — transition between standing and crouching
    inline void updateCrouch() {
        if (mWantsCrouch && !mCrouching) {
            // Start crouching
            mCrouching = true;
            mHeadOffset = CROUCH_HEAD_OFFSET;
        } else if (!mWantsCrouch && mCrouching) {
            // Try to uncrouch — check if there's room above
            // Test head sphere at standing height for collision
            Vector3 standingHeadPos = mPosition + Vector3(0.0f, 0.0f, HEAD_OFFSET);
            auto headContacts = mCollision.sphereVsCellPolygons(
                standingHeadPos, SPHERE_RADIUS, mCellIdx);

            // Only uncrouch if head has no collisions at standing height
            if (headContacts.empty()) {
                mCrouching = false;
                mHeadOffset = HEAD_OFFSET;
            }
            // else: ceiling too low, stay crouched
        }
    }

    /// Update head spring — provides natural head bob.
    /// The head is connected to the body by a spring; when the body
    /// moves vertically (stairs, landing), the head lags behind slightly.
    inline void updateHeadSpring() {
        float targetOffset = mCrouching ? CROUCH_HEAD_OFFSET : HEAD_OFFSET;

        // Spring force toward target offset
        float displacement = targetOffset - mHeadOffset;
        float springForce = displacement * HEAD_SPRING_TENSION;

        // Damping force to prevent oscillation
        float dampingForce = -mHeadVelocity * HEAD_SPRING_DAMPING;

        // Integrate spring
        mHeadVelocity += (springForce + dampingForce);
        mHeadOffset += mHeadVelocity;

        // Clamp to prevent wild oscillation
        float minOffset = targetOffset - 1.0f;
        float maxOffset = targetOffset + 1.0f;
        mHeadOffset = std::max(minOffset, std::min(maxOffset, mHeadOffset));
    }

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

    // Contacts from last collision resolution (used by detectGround)
    std::vector<SphereContact> mLastContacts;
};

} // namespace Darkness

#endif // __PLAYERPHYSICS_H
