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
 *    Uses a 5-sphere player model:
 *    HEAD (+1.8), BODY (-0.6), SHIN (-2.2), KNEE (-2.6), FOOT (-3.0)
 *    All spheres have radius 1.2, rigidly attached to a single body center.
 *    Stairs are handled naturally by lower sphere contacts generating
 *    upward-component normals — no explicit stair-step logic needed.
 *
 *    Head bob is driven by discrete motion poses — target offsets that the
 *    head spring tracks. Each stride activates a new pose target, and the
 *    3D spring naturally smooths the transitions, creating realistic bob
 *    with organic acceleration/deceleration. This matches the original
 *    Dark Engine's player head physics system.
 *
 *    Constants:
 *    - Walk speed: 11.0 units/sec, Run: 22.0, Creep: 5.5
 *    - All sphere radius: 1.2 units
 *    - 5 sphere offsets from body center (Z-up): +1.8, -0.6, -2.2, -2.6, -3.0
 *    - Gravity: ~32 units/sec² (runtime-configurable)
 *    - Fixed timestep: 60 Hz (1/60 sec per physics step)
 *
 *    See NOTES.SOURCE.md for full physics constants documentation.
 *
 *****************************************************************************/

#ifndef __PLAYERPHYSICS_H
#define __PLAYERPHYSICS_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <vector>

#include "DarknessMath.h"
#include "CollisionGeometry.h"
#include "IPhysicsWorld.h"
#include "RayCaster.h"

namespace Darkness {

/// Player mode — high-level state machine.
/// Determines speed multipliers, motion pose configuration, and physics behavior.
/// Replaces the separate PlayerMoveState enum and mCrouching boolean.
enum class PlayerMode {
    Stand,      // 0: Normal walking/running (on ground)
    Crouch,     // 1: Crouched movement (0.6× speed)
    Swim,       // 2: In water (0.7× speed, 0.7× rotation)
    Climb,      // 3: On climbable surface (0.5× trans, 0.7× rot)
    BodyCarry,  // 4: Carrying a body (1.0× speed, heavy bob)
    Slide,      // 5: Sliding on smooth surface
    Jump,       // 6: Airborne (jumping/falling)
    Dead,       // 7: No movement
    NumModes
};

/// Mantle animation state — 5 phases from ledge detection to standing up.
enum class MantleState {
    None,       // not mantling
    Hold,       // Phase 1: hold position, suppress physics (MANTLE_HOLD_TIME)
    Rise,       // Phase 2: move vertically toward ledge via spring
    Forward,    // Phase 3: move horizontally onto ledge (MANTLE_FWD_TIME)
    StandUp,    // Phase 4: restore normal standing offsets
    Complete    // Phase 5: wait for ground contact, then return to normal mode
};

/// Player physics simulation — custom 5-sphere-polygon collision.
/// Owns the player's position, velocity, and movement state.
/// Updated each frame by DarkPhysics::step().
class PlayerPhysics {
public:
    // ── Player physics constants ──
    // 5-sphere player model (HEAD, BODY, SHIN, KNEE, FOOT) with fixed radius and vertical offsets.

    // Sphere radii — all 5 spheres use the same radius
    static constexpr float SPHERE_RADIUS = 1.2f;

    // Player height parameters
    static constexpr float STAND_HEIGHT   = 6.0f;    // total standing height
    static constexpr float CROUCH_HEIGHT  = 3.0f;    // total crouching height

    // 5-sphere center offsets relative to body center (Z-up).
    // Standing on flat ground: FOOT bottom at ground → FOOT center at ground + 1.2
    // → body center at ground + 4.2 → eye (HEAD center) at ground + 6.0.
    static constexpr float HEAD_OFFSET_Z  =  1.8f;   // (STAND_HEIGHT/2) - SPHERE_RADIUS
    static constexpr float BODY_OFFSET_Z  = -0.6f;   // (STAND_HEIGHT/2) - 3*SPHERE_RADIUS
    static constexpr float SHIN_OFFSET_Z  = -2.2f;   // -(STAND_HEIGHT * 11/30)
    static constexpr float KNEE_OFFSET_Z  = -2.6f;   // -(STAND_HEIGHT * 13/30)
    static constexpr float FOOT_OFFSET_Z  = -3.0f;   // -(STAND_HEIGHT/2)

    // Number of collision spheres and offset array for iteration
    static constexpr int NUM_SPHERES = 5;
    static constexpr float SPHERE_OFFSETS[NUM_SPHERES] = {
        HEAD_OFFSET_Z,   // 0: HEAD   +1.8
        BODY_OFFSET_Z,   // 1: BODY   -0.6
        SHIN_OFFSET_Z,   // 2: SHIN   -2.2
        KNEE_OFFSET_Z,   // 3: KNEE   -2.6
        FOOT_OFFSET_Z    // 4: FOOT   -3.0
    };

    // Crouch scale — all sphere offsets scale by CROUCH_HEIGHT/STAND_HEIGHT when crouching.
    // Body center drops to keep feet on ground.
    static constexpr float CROUCH_SCALE = CROUCH_HEIGHT / STAND_HEIGHT; // 0.5
    // Body center shift when crouching: (1 - CROUCH_SCALE) * FOOT_OFFSET_Z
    // = 0.5 * (-3.0) = -1.5 (body center drops 1.5 units to keep feet planted)
    static constexpr float CROUCH_CENTER_DROP = (1.0f - CROUCH_SCALE) * FOOT_OFFSET_Z;

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
    static constexpr float GRAVITY        = 32.0f;   // units/sec² (runtime-adjustable)
    static constexpr float FIXED_DT       = 1.0f / 60.0f;  // 60 Hz physics timestep
    // Slope handling — friction-based.
    // GROUND_NORMAL_MIN: any surface with normal.z above this counts as "ground" for
    // state/landing purposes. The 5-sphere model and movement control handle the rest.
    // SLIDE_THRESHOLD: below this normal.z, movement control is progressively reduced,
    // causing the player to slide down steep slopes under gravity.
    static constexpr float GROUND_NORMAL_MIN = 0.1f;   // cos(~84°) — any upward surface
    static constexpr float SLIDE_THRESHOLD   = 0.64f;  // cos(~50°) — below this, sliding

    // Collision iteration count — matching the renderer's camera collision
    static constexpr int COLLISION_ITERS  = 3;

    // Landing detection thresholds.
    static constexpr float LANDING_MIN_VEL  = 2.0f;    // minimum downward speed for landing pose
    static constexpr float LANDING_MIN_TIME = 0.2f;    // minimum 200ms between landing events

    // Mantle system — 3-ray detection + 5-state animation
    static constexpr float MANTLE_UP_DIST   = 3.5f;    // upward headroom check distance
    static constexpr float MANTLE_FWD_DIST  = 2.4f;    // forward ledge reach (radius × 2)
    static constexpr float MANTLE_DOWN_DIST = 7.0f;    // downward surface scan distance
    static constexpr float MANTLE_HOLD_TIME = 0.3f;    // Phase 1: hold & compress duration
    static constexpr float MANTLE_FWD_TIME  = 0.4f;    // Phase 3: forward movement duration

    // Acceleration-based movement — replaces direct velocity setting.
    // Impulse-based control (PHCTRL); these values create similar ramp-up/ramp-down curves.
    // At 120 units/sec² accel, walk speed (11) reached in ~0.09 seconds.
    // At 80 units/sec² decel, full stop from walk in ~0.14 seconds.
    static constexpr float MOVEMENT_ACCEL = 120.0f;  // ground acceleration (units/sec²)
    static constexpr float MOVEMENT_DECEL = 80.0f;   // ground deceleration (units/sec²)

    // Air acceleration fraction — airborne steering uses this fraction of
    // ground acceleration. The original engine uses the same impulse formula
    // with friction=0 and Z removed from input.
    static constexpr float AIR_ACCEL_FRAC = 0.3f;

    // Spring-connected head (smooths vertical motion, creates natural head bob).
    // Formula: vel = displacement * tension + vel * damping
    // Damping=0.02 means only 2% of previous velocity is retained per frame
    // (98% decay — very heavy damping, no oscillation).
    // The Z-axis (vertical) gets an additional 0.5 multiplier for less springy vertical motion.
    static constexpr float HEAD_SPRING_TENSION = 0.6f;
    static constexpr float HEAD_SPRING_DAMPING = 0.02f;
    static constexpr float HEAD_SPRING_Z_SCALE = 0.5f;  // vertical axis half-strength
    static constexpr float HEAD_SPRING_CAP     = 25.0f;  // max spring velocity

    // Eye position above head submodel center.
    // Camera is 0.8 units above the head sphere center for natural eye height.
    static constexpr float EYE_ABOVE_HEAD     = 0.8f;

    // Note: GROUND_FRICTION removed — replaced by MOVEMENT_DECEL (acceleration model)

    // ── Per-mode speed multipliers ──
    // trans = translation speed scale, rot = rotation speed scale.
    struct ModeSpeedScale { float trans; float rot; };
    static constexpr ModeSpeedScale MODE_SPEEDS[static_cast<int>(PlayerMode::NumModes)] = {
        {1.0f, 1.0f},  // Stand:     100% trans, 100% rot
        {0.6f, 0.6f},  // Crouch:    60% trans,  60% rot
        {0.7f, 0.7f},  // Swim:      70% trans,  70% rot
        {0.5f, 0.7f},  // Climb:     50% trans,  70% rot
        {1.0f, 1.0f},  // BodyCarry: 100% (set from script)
        {1.0f, 1.0f},  // Slide:     100%
        {1.0f, 1.0f},  // Jump:      100% (air control handles actual speed)
        {0.0f, 0.0f},  // Dead:      disabled
    };

    // ── Motion pose system ──
    // The drives head bob through discrete motion poses —
    // target head positions that the 3D spring tracks. Each stride activates a
    // new pose, and the spring naturally smooths the transitions.
    //
    // Pose offsets are in player-local coordinates:
    //   fwd = forward displacement, lat = lateral (right positive), vert = vertical (down negative)
    //
    // Stride triggering uses velocity-dependent footstep distance:
    //   < 5 u/s: 2.5 units/step, 5–15 u/s: lerp to 4.0, > 15 u/s: 4.0.

    /// Motion pose data — target head displacement from stance rest position.
    struct MotionPoseData {
        float duration;    // blend time from current pose to this target (seconds)
        float holdTime;    // hold at target before allowing next pose (seconds)
        float fwd;         // forward displacement
        float lat;         // lateral displacement (positive = right)
        float vert;        // vertical displacement (negative = dip)
    };

    // ── Pose table — complete set of player motion poses ──
    //
    // Normal walking/idle (submodel 0 head offset):
    static constexpr MotionPoseData POSE_NORMAL       = {0.8f,  0.0f,  0.0f,   0.0f,    0.0f};

    // Walking strides — each stride is chained as {footfall, recovery} via the motion
    // queue, creating a natural up-down bounce. Duration (0.6s) is longer than the
    // stride interval (~0.31s at walk speed), so each new stride interrupts the previous
    // mid-blend. The head spring smooths these interrupted transitions into organic
    // walking motion — rapid convergence toward footfall target, then interrupted by the
    // next stride before fully reaching it. This matches the Dark Engine's stride system.
    static constexpr MotionPoseData POSE_STRIDE_LEFT  = {0.6f,  0.01f, 0.0f,  -0.1f,   -0.4f};
    static constexpr MotionPoseData POSE_STRIDE_RIGHT = {0.6f,  0.01f, 0.0f,   0.1f,   -0.4f};

    // Crouching strides — slightly wider sway, deeper vertical offset from crouch height.
    // Vert is relative to POSE_CROUCH (-2.02), so -2.50 = 0.48 below crouch rest.
    static constexpr MotionPoseData POSE_CRAWL_LEFT   = {0.6f,  0.01f, 0.0f,  -0.15f,  -2.50f};
    static constexpr MotionPoseData POSE_CRAWL_RIGHT  = {0.6f,  0.01f, 0.0f,   0.15f,  -2.50f};

    // Crouching idle — dip from crouch height change:
    static constexpr MotionPoseData POSE_CROUCH       = {0.8f,  0.0f,  0.0f,   0.0f,   -2.02f};

    // Landing impact — instantaneous dip on landing (dur=0 = snap), held briefly:
    static constexpr MotionPoseData POSE_JUMP_LAND    = {0.0f,  0.1f,  0.0f,   0.0f,   -0.5f};

    // Body carry mode (carrying a body or heavy object):
    // Heavier bob — deeper dip and more lateral sway while carrying.
    static constexpr MotionPoseData POSE_CARRY_IDLE   = {0.8f,  0.0f,  0.0f,   0.0f,   -0.8f};
    static constexpr MotionPoseData POSE_CARRY_LEFT   = {0.6f,  0.01f, 0.0f,  -0.5f,   -1.5f};
    static constexpr MotionPoseData POSE_CARRY_RIGHT  = {0.6f,  0.01f, 0.0f,   0.15f,  -1.1f};

    // Weapon swing (head bob during sword/blackjack attacks):
    static constexpr MotionPoseData POSE_WEAPON_SWING       = {0.6f,  0.01f, 0.8f,  0.0f,   0.0f};
    static constexpr MotionPoseData POSE_WEAPON_SWING_CROUCH = {0.6f, 0.01f, 0.8f,  0.0f,  -2.02f};

    // Standing lean — 1.5s blend, no hold (lean via motion pose system):
    // lat is in player-local coords: positive = right. So lean LEFT = negative lat.
    static constexpr MotionPoseData POSE_LEAN_LEFT    = {1.5f,  0.0f,  0.0f,  -2.2f,    0.0f};
    static constexpr MotionPoseData POSE_LEAN_RIGHT   = {1.5f,  0.0f,  0.0f,   2.2f,    0.0f};
    static constexpr MotionPoseData POSE_LEAN_FWD     = {1.5f,  0.0f,  2.2f,   0.0f,    0.0f};

    // Crouching lean — reduced distance + vertical drop:
    static constexpr MotionPoseData POSE_CLNLEAN_LEFT  = {1.5f, 0.0f, 0.0f,  -1.7f,   -2.0f};
    static constexpr MotionPoseData POSE_CLNLEAN_RIGHT = {1.5f, 0.0f, 0.0f,   1.7f,   -2.0f};
    static constexpr MotionPoseData POSE_CLNLEAN_FWD   = {1.5f, 0.0f, 1.7f,   0.0f,   -2.0f};

    // ── Per-mode motion configuration ──
    // Maps each PlayerMode to its rest, left stride, and right stride poses.
    // Modes without walking bob use the rest pose for all three slots.
    struct ModeMotionConfig {
        const MotionPoseData *restPose;     // idle pose for this mode
        const MotionPoseData *strideLeft;   // left foot stride
        const MotionPoseData *strideRight;  // right foot stride
    };

    // Inline helper — constexpr array not viable with pointer-to-static-constexpr
    // in C++17, so use a lookup function instead.
    static inline ModeMotionConfig getModeMotion(PlayerMode mode) {
        switch (mode) {
        case PlayerMode::Stand:     return {&POSE_NORMAL,     &POSE_STRIDE_LEFT,  &POSE_STRIDE_RIGHT};
        case PlayerMode::Crouch:    return {&POSE_CROUCH,     &POSE_CRAWL_LEFT,   &POSE_CRAWL_RIGHT};
        case PlayerMode::Swim:      return {&POSE_NORMAL,     &POSE_NORMAL,        &POSE_NORMAL};
        case PlayerMode::Climb:     return {&POSE_NORMAL,     &POSE_NORMAL,        &POSE_NORMAL};
        case PlayerMode::BodyCarry: return {&POSE_CARRY_IDLE, &POSE_CARRY_LEFT,    &POSE_CARRY_RIGHT};
        case PlayerMode::Slide:     return {&POSE_NORMAL,     &POSE_NORMAL,        &POSE_NORMAL};
        case PlayerMode::Jump:      return {&POSE_NORMAL,     &POSE_NORMAL,        &POSE_NORMAL};
        case PlayerMode::Dead:      return {&POSE_NORMAL,     &POSE_NORMAL,        &POSE_NORMAL};
        default:                    return {&POSE_NORMAL,     &POSE_NORMAL,        &POSE_NORMAL};
        }
    }

    // Stride distance thresholds (velocity-dependent footstep formula)
    static constexpr float STRIDE_DIST_MIN   = 2.5f;   // footstep distance at creep speed
    static constexpr float STRIDE_DIST_MAX   = 4.0f;   // footstep distance at run speed
    static constexpr float STRIDE_SPEED_LOW  = 5.0f;   // speed where stride distance starts growing
    static constexpr float STRIDE_SPEED_HIGH = 15.0f;  // speed where stride distance reaches max

    // Leaning — visual-only camera offset, physics body stays in place.
    // Lean is purely cosmetic, no physics body movement.
    // Lean parameters — lean is driven through motion poses (POSE_LEAN_*/POSE_CLNLEAN_*).
    // The head spring provides natural easing (no separate blend timer needed).
    // Distances match the motion pose lateral offsets:
    //   Standing lean: 2.2 units lateral (POSE_LEAN_LEFT/RIGHT)
    //   Crouching lean: 1.7 units lateral + 2.0 units vertical drop (POSE_CLNLEAN_*)
    static constexpr float LEAN_DISTANCE        = 2.2f;   // standing lean lateral offset (world units)
    static constexpr float CROUCH_LEAN_DISTANCE = 1.7f;   // crouching lean lateral offset (world units)
    static constexpr float LEAN_TILT            = 0.2618f; // max camera roll (~15 degrees, radians)

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

    /// Set sneaking (creep) state. Reduces movement speed by 0.5x.
    /// Cannot stack with running — sneaking takes priority.
    inline void setSneaking(bool sneaking) { mSneaking = sneaking; }

    /// Set running state. Doubles movement speed (22 units/sec).
    /// If sneaking is also active, sneaking takes priority (no running).
    inline void setRunning(bool running) { mRunning = running; }

    /// Disable all motion (for cutscenes, death, etc.)
    /// When disabled, movement input is ignored, poses stop updating,
    /// and gravity still applies but player cannot control velocity.
    inline void disableMotion() { mMotionDisabled = true; }

    /// Re-enable motion after being disabled.
    inline void enableMotion()  { mMotionDisabled = false; }

    /// Check if motion is currently disabled.
    bool isMotionDisabled() const { return mMotionDisabled; }

    /// Set lean direction: -1 = left, 0 = center, +1 = right.
    /// Lean is visual-only — the physics body stays in place while the
    /// camera offsets laterally, with collision limiting the lean distance.
    /// Lean is driven through the motion pose system (POSE_LEAN_* or POSE_CLNLEAN_*),
    /// and the head spring provides natural easing. While leaning, stride bob is
    /// suppressed — lean poses take priority in the chaining state machine.
    inline void setLeanDirection(int dir) {
        if (dir == mLeanDir) return;  // no change
        mLeanDir = dir;

        if (dir < 0) {
            // Lean left — select standing or crouching variant
            activatePose(isCrouching() ? POSE_CLNLEAN_LEFT : POSE_LEAN_LEFT);
        } else if (dir > 0) {
            // Lean right
            activatePose(isCrouching() ? POSE_CLNLEAN_RIGHT : POSE_LEAN_RIGHT);
        } else {
            // Centering — return to mode's rest pose
            activatePose(*getModeMotion(mCurrentMode).restPose);
        }
    }

    // ── State queries ──

    /// Body center position (not eye position).
    /// In the 5-sphere model, body center is at ground + 4.2 on flat ground
    /// (higher than the old 2-sphere model's ground + 1.2).
    const Vector3 &getPosition() const { return mPosition; }

    /// Eye position — computed from body center + 3D head spring output.
    /// The 3D spring tracks motion pose targets (stride bob, landing dip),
    /// producing natural forward/lateral/vertical displacement smoothed by
    /// spring dynamics. Lean offset is applied on top with collision limiting.
    ///
    /// Spring displacement is in player-local coordinates:
    ///   X = forward (cos(yaw), sin(yaw), 0)
    ///   Y = lateral/right (sin(yaw), -cos(yaw), 0)
    ///   Z = vertical (0, 0, 1)
    inline Vector3 getEyePosition() const {
        // Base eye Z = body center + head offset + gEyeLoc + spring vertical displacement
        float headZ = isCrouching() ? HEAD_OFFSET_Z * CROUCH_SCALE : HEAD_OFFSET_Z;
        Vector3 eye = mPosition + Vector3(0.0f, 0.0f, headZ + EYE_ABOVE_HEAD + mSpringPos.z);

        float sinYaw = std::sin(mYaw);
        float cosYaw = std::cos(mYaw);

        // Apply spring forward displacement (player-local → world)
        if (std::fabs(mSpringPos.x) > 0.001f) {
            eye.x += cosYaw * mSpringPos.x;
            eye.y += sinYaw * mSpringPos.x;
        }

        // Apply lateral displacement (player-local → world).
        // Uses collision-limited mLeanAmount instead of raw mSpringPos.y.
        // When not leaning, mLeanAmount == mSpringPos.y (stride bob only, no collision).
        // When leaning, mLeanAmount may be reduced by wall collision in updateLean().
        // Right vector in Z-up: (sin(yaw), -cos(yaw), 0)
        if (std::fabs(mLeanAmount) > 0.001f) {
            eye.x += sinYaw * mLeanAmount;
            eye.y -= cosYaw * mLeanAmount;
        }
        return eye;
    }

    /// Camera roll/bank angle from leaning (radians).
    /// Positive = tilting right, negative = tilting left.
    /// Proportional to the collision-limited lean extent (mLeanAmount),
    /// which is derived from the spring's lateral displacement each frame.
    inline float getLeanTilt() const {
        float maxDist = isCrouching() ? CROUCH_LEAN_DISTANCE : LEAN_DISTANCE;
        if (maxDist < 0.01f) return 0.0f;
        return (mLeanAmount / maxDist) * LEAN_TILT;
    }

    /// Linear velocity
    const Vector3 &getVelocity() const { return mVelocity; }

    /// Current WR cell index (-1 if outside all cells)
    int32_t getCell() const { return mCellIdx; }

    /// Is the player standing on a walkable surface?
    /// Ground modes: Stand, Crouch, BodyCarry, Slide (not Jump, Swim, Climb, Dead)
    bool isOnGround() const {
        return mCurrentMode == PlayerMode::Stand ||
               mCurrentMode == PlayerMode::Crouch ||
               mCurrentMode == PlayerMode::BodyCarry ||
               mCurrentMode == PlayerMode::Slide;
    }

    /// Current player mode
    PlayerMode getMode() const { return mCurrentMode; }

    /// Is the player crouching?
    bool isCrouching() const { return mCurrentMode == PlayerMode::Crouch; }

    /// Is the player sneaking (creeping)?
    bool isSneaking() const { return mSneaking; }

    /// Is the player running?
    bool isRunning() const { return mRunning; }

    /// Is the player currently mantling? (Stub — implemented in Phase 5)
    bool isMantling() const { return mMantling; }

    // ── Teleport ──

    /// Set player position directly (for spawn, teleport).
    /// Also resets velocity and updates cell index.
    /// If findCell fails at the given position, tries head and foot level
    /// as fallbacks (the caller often passes the camera/eye position,
    /// and the cell geometry may not extend below the floor).
    inline void setPosition(const Vector3 &pos) {
        mPosition = pos;
        mVelocity = Vector3(0.0f);
        mCellIdx = mCollision.findCell(pos);
        if (mCellIdx < 0) {
            // Try at head sphere level (body center might be placed at eye height)
            mCellIdx = mCollision.findCell(pos + Vector3(0.0f, 0.0f, HEAD_OFFSET_Z));
        }
        if (mCellIdx < 0) {
            // Try at foot sphere level (body center is higher in 5-sphere model,
            // so the foot sphere position might be inside the cell)
            mCellIdx = mCollision.findCell(pos + Vector3(0.0f, 0.0f, FOOT_OFFSET_Z));
        }
        if (mCellIdx < 0) {
            // Last resort: try slightly above the given position
            mCellIdx = mCollision.findCell(pos + Vector3(0.0f, 0.0f, SPHERE_RADIUS));
        }
        mCurrentMode = PlayerMode::Jump; // will detect ground on next step
    }

    /// Set gravity magnitude (units/sec²). Default is GRAVITY.
    void setGravityMagnitude(float g) { mGravityMag = g; }

private:
    // ── Internal simulation steps ──

    /// Single fixed-timestep physics step
    inline void fixedStep(const ContactCallback &contactCb) {
        // Advance simulation time (for landing throttle, animation timing, etc.)
        mSimTime += FIXED_DT;

        // ── Mantle takes over the full step when active ──
        // During mantling, normal movement/gravity/collision are suppressed.
        // The mantle state machine drives position directly.
        if (mMantling) {
            updateMantle();
            updateCell();
            updateMotionPose();
            updateHeadSpring();
            updateLean();
            return;
        }

        // 1. Handle jump request (can jump from any ground mode)
        if (mJumpRequested && !mMotionDisabled) {
            if (isOnGround()) {
                mVelocity.z = JUMP_IMPULSE;
                mCurrentMode = PlayerMode::Jump;
                // Reset motion to rest pose — the original locks motion to
                // kMoNormal during jump mode (no stride bob while airborne).
                // Also reset stride distance so landing doesn't carry stale state.
                activatePose(POSE_NORMAL);
                mStrideDist = 0.0f;
            }
        }
        mJumpRequested = false;

        // 2. Handle mode transitions (crouch, swim, etc.)
        if (!mMotionDisabled)
            updateModeTransitions();

        // 3. Apply gravity (if airborne — always, even when disabled)
        applyGravity();

        // 4. Apply horizontal movement from input (skip when disabled)
        if (!mMotionDisabled)
            applyMovement();

        // 5. Integrate position
        integrate();

        // 6. Update cell index BEFORE collision — if integration moved
        //    the player through a portal, collision must test against
        //    the new cell's walls, not the old cell's.
        updateCell();

        // 7. Resolve collisions via 5-sphere constraint projection
        resolveCollisions(contactCb);

        // 8. Detect ground and update movement state
        detectGround();

        // 9. Update cell index again (collision may have shifted position)
        // Note: no snap-to-ground here. The original Dark Engine relies purely on
        // gravity + collision + constraint response to keep the player grounded.
        // An explicit snap was causing jitter (collision pushes up, snap pulls down)
        // and fighting with uphill movement.
        updateCell();

        // 10. Check for mantle opportunity when airborne and pressing forward
        if (mCurrentMode == PlayerMode::Jump && mInputForward > 0.1f && !mMotionDisabled) {
            checkMantle();
        }

        // 11. Update motion pose system (discrete stride-driven pose targets)
        updateMotionPose();

        // 12. Update 3D head spring (tracks pose targets with spring dynamics)
        updateHeadSpring();

        // 13. Update lean (visual-only lateral camera offset with collision)
        updateLean();
    }

    /// Apply gravity to velocity (if not on ground)
    inline void applyGravity() {
        if (!isOnGround()) {
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

        // Determine speed based on movement direction and speed modifiers.
        // Speed toggle hierarchy: sneak (0.5x) takes priority over run (2x).
        // Crouch multiplier (0.6x) stacks with either speed mode.
        float forwardSpeed = WALK_SPEED;
        float strafeSpeed  = SIDESTEP_SPEED;

        // Backward movement is slower
        if (mInputForward < 0.0f)
            forwardSpeed = BACKWARD_SPEED;

        // Speed mode: sneak (creep 0.5x) or run (sprint 2x)
        if (mSneaking) {
            forwardSpeed *= 0.5f;
            strafeSpeed  *= 0.5f;
        } else if (mRunning) {
            forwardSpeed *= 2.0f;
            strafeSpeed  *= 2.0f;
        }

        // Apply mode speed multiplier (crouch=0.6, swim=0.7, climb=0.5, etc.)
        // Stacks with sneak/run for effective speed calculation.
        float modeScale = MODE_SPEEDS[static_cast<int>(mCurrentMode)].trans;
        forwardSpeed *= modeScale;
        strafeSpeed  *= modeScale;

        Vector3 desired = forward * (mInputForward * forwardSpeed)
                        + right   * (mInputRight   * strafeSpeed);

        return desired;
    }

    /// Apply movement input to velocity — acceleration-based control.
    /// Instead of setting velocity directly, accelerates toward the desired
    /// velocity. This creates natural ramp-up/ramp-down curves impulse-based control system.
    ///
    /// Ground: accelerate toward desired at MOVEMENT_ACCEL, decelerate
    /// at MOVEMENT_DECEL when no input. Slope projection prevents
    /// fighting gravity on inclines.
    ///
    /// Air: same acceleration model at reduced rate (AIR_ACCEL_FRAC),
    /// with Z removed from input (can't add vertical speed while airborne).
    inline void applyMovement() {
        Vector3 desired = computeDesiredVelocity();

        if (isOnGround()) {
            // Project desired velocity onto ground plane so walking on slopes
            // doesn't fight gravity. Always project when on any ground surface.
            if (mGroundNormal.z > GROUND_NORMAL_MIN) {
                float dot = glm::dot(desired, mGroundNormal);
                desired -= mGroundNormal * dot;
            }

            // Friction-based slope handling:
            // On steep slopes (normal.z < SLIDE_THRESHOLD), progressively reduce
            // movement control. This lets the 5-sphere model and gravity naturally
            // determine what's walkable vs what causes sliding — no hard angle cutoff.
            if (mGroundNormal.z < SLIDE_THRESHOLD) {
                // Linear ramp: full control at SLIDE_THRESHOLD, zero at GROUND_NORMAL_MIN
                float slopeFactor = (mGroundNormal.z - GROUND_NORMAL_MIN)
                                  / (SLIDE_THRESHOLD - GROUND_NORMAL_MIN);
                slopeFactor = std::max(0.0f, std::min(1.0f, slopeFactor));
                desired *= slopeFactor;
            }

            float desiredLen = glm::length(desired);
            if (desiredLen < 0.01f) {
                // No input — decelerate toward zero
                float hSpeed = std::sqrt(mVelocity.x * mVelocity.x +
                                         mVelocity.y * mVelocity.y);
                if (hSpeed > 0.01f) {
                    float decel = MOVEMENT_DECEL * FIXED_DT;
                    float newSpeed = std::max(0.0f, hSpeed - decel);
                    float ratio = newSpeed / hSpeed;
                    mVelocity.x *= ratio;
                    mVelocity.y *= ratio;
                }
            } else {
                // Has input — accelerate toward desired velocity.
                // Compute acceleration direction from current horizontal velocity
                // toward desired. This creates a natural ramp-up curve.
                Vector3 hVel(mVelocity.x, mVelocity.y, 0.0f);
                Vector3 delta = desired - hVel;
                float deltaLen = glm::length(delta);

                if (deltaLen > 0.01f) {
                    // Accelerate toward desired velocity
                    float accelMag = MOVEMENT_ACCEL * FIXED_DT;
                    if (accelMag > deltaLen) {
                        // Would overshoot — snap to desired
                        mVelocity.x = desired.x;
                        mVelocity.y = desired.y;
                    } else {
                        Vector3 accelDir = delta / deltaLen;
                        mVelocity.x += accelDir.x * accelMag;
                        mVelocity.y += accelDir.y * accelMag;
                    }
                }

                // Apply slope Z component — SET (not +=) to prevent accumulation
                // on slopes that would launch the player off ramps.
                mVelocity.z = desired.z;
            }
        } else {
            // Airborne: acceleration-based steering with Z removed from input.
            // Uses the same impulse formula with friction=0
            // but removes vertical input — can't gain altitude while airborne.
            Vector3 airDesired(desired.x, desired.y, 0.0f);
            float airDesiredLen = glm::length(airDesired);

            if (airDesiredLen > 0.01f) {
                Vector3 hVel(mVelocity.x, mVelocity.y, 0.0f);
                Vector3 delta = airDesired - hVel;
                float deltaLen = glm::length(delta);

                if (deltaLen > 0.01f) {
                    float airAccel = MOVEMENT_ACCEL * AIR_ACCEL_FRAC * FIXED_DT;
                    if (airAccel > deltaLen) {
                        mVelocity.x = airDesired.x;
                        mVelocity.y = airDesired.y;
                    } else {
                        Vector3 accelDir = delta / deltaLen;
                        mVelocity.x += accelDir.x * airAccel;
                        mVelocity.y += accelDir.y * airAccel;
                    }
                }
            }
        }
    }

    /// Integrate position from velocity
    inline void integrate() {
        mPosition += mVelocity * FIXED_DT;
    }

    /// Resolve collisions using the 5-sphere model.
    /// For each iteration, tests all 5 sphere centers against cell polygons,
    /// accumulates contacts, and pushes the body center out of penetrations.
    /// The 5-sphere model naturally handles stairs: lower spheres (FOOT, KNEE,
    /// SHIN) contact step edges with normals that have upward components,
    /// lifting the player up and over without explicit stair-step logic.
    inline void resolveCollisions(const ContactCallback &contactCb) {
        Vector3 origPos = mPosition;
        int32_t origCell = mCellIdx;

        // Compute current sphere offsets (standing or crouching)
        float scale = isCrouching() ? CROUCH_SCALE : 1.0f;

        mLastContacts.clear();

        for (int iter = 0; iter < COLLISION_ITERS; ++iter) {
            if (mCellIdx < 0)
                break;

            std::vector<SphereContact> iterContacts;

            for (int s = 0; s < NUM_SPHERES; ++s) {
                Vector3 sphereCenter = mPosition + Vector3(0.0f, 0.0f, SPHERE_OFFSETS[s] * scale);

                // Test against the body center's cell
                auto contacts = mCollision.sphereVsCellPolygons(
                    sphereCenter, SPHERE_RADIUS, mCellIdx);

                // Test the sphere's own cell if it differs from body center's cell
                // (handles straddling portal boundaries, e.g. feet in one cell, head in another)
                int32_t sphereCell = mCollision.findCell(sphereCenter);
                if (sphereCell >= 0 && sphereCell != mCellIdx) {
                    auto adj = mCollision.sphereVsCellPolygons(
                        sphereCenter, SPHERE_RADIUS, sphereCell);
                    contacts.insert(contacts.end(), adj.begin(), adj.end());
                }

                // Test adjacent cells via portals from the body center's cell
                const auto &cell = mCollision.getWR().cells[mCellIdx];
                int numSolid = cell.numPolygons - cell.numPortals;
                for (int pi = numSolid; pi < cell.numPolygons; ++pi) {
                    const auto &poly = cell.polygons[pi];
                    if (poly.count < 3) continue;
                    const auto &plane = cell.planes[poly.plane];
                    float dist = plane.getDistance(sphereCenter);
                    if (dist < SPHERE_RADIUS) {
                        int32_t tgtCell = static_cast<int32_t>(poly.tgtCell);
                        if (tgtCell >= 0 && tgtCell != mCellIdx && tgtCell != sphereCell
                            && tgtCell < static_cast<int32_t>(mCollision.getWR().numCells)) {
                            auto adj = mCollision.sphereVsCellPolygons(
                                sphereCenter, SPHERE_RADIUS, tgtCell);
                            contacts.insert(contacts.end(), adj.begin(), adj.end());
                        }
                    }
                }

                iterContacts.insert(iterContacts.end(), contacts.begin(), contacts.end());
            }

            if (iterContacts.empty())
                break; // No contacts — done

            // Push body center out of penetrating polygons.
            // De-duplicate by normal direction: when multiple spheres hit the same
            // wall (dot > 0.99), use only the maximum penetration depth. Without
            // this, 3 spheres hitting one wall would triple the push, causing
            // jittery over-correction on the next frame.
            std::vector<std::pair<Vector3, float>> pushes; // {normal, maxPenetration}
            for (const auto &c : iterContacts) {
                bool merged = false;
                for (auto &p : pushes) {
                    if (glm::dot(c.normal, p.first) > 0.99f) {
                        // Same wall — keep the deeper penetration
                        p.second = std::max(p.second, c.penetration);
                        merged = true;
                        break;
                    }
                }
                if (!merged) {
                    pushes.push_back({c.normal, c.penetration});
                }
            }
            for (const auto &p : pushes) {
                mPosition += p.first * p.second;
            }

            // Accumulate contacts for ground detection and velocity removal
            mLastContacts.insert(mLastContacts.end(),
                                 iterContacts.begin(), iterContacts.end());

            // Push may have moved through a portal — update cell
            int32_t newCell = mCollision.findCell(mPosition);
            if (newCell >= 0) {
                mCellIdx = newCell;
            } else {
                // Pushed outside all cells — revert to original position
                mPosition = origPos;
                mCellIdx = origCell;
                return;
            }
        }

        // Final validation — body center must still be in a valid cell
        if (mCollision.findCell(mPosition) < 0) {
            mPosition = origPos;
            mCellIdx = origCell;
        }

        // Fire contact callbacks for downstream consumers (audio, AI, scripts)
        if (contactCb) {
            for (const auto &c : mLastContacts) {
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

        // Constraint-based velocity response.
        //
        // 1 surface  — remove the normal velocity component (slide along plane)
        // 2 surfaces — project velocity onto the edge between them (slide along crease)
        // 3+ surfaces — project onto edge, validate direction; if reversed, fully stop
        //
        // This prevents the over-removal bug where independent per-normal subtraction
        // could push velocity *through* an adjacent surface.

        // Collect unique constraint normals (dot > 0.99 = same plane)
        std::vector<Vector3> constraints;
        constraints.reserve(mLastContacts.size());
        for (const auto &c : mLastContacts) {
            // Only constrain if velocity is moving into the surface
            float vn = glm::dot(mVelocity, c.normal);
            if (vn >= 0.0f) continue;

            bool duplicate = false;
            for (const auto &cn : constraints) {
                if (glm::dot(c.normal, cn) > 0.99f) { duplicate = true; break; }
            }
            if (!duplicate)
                constraints.push_back(c.normal);
        }

        if (constraints.size() == 1) {
            // Single surface — remove velocity component along the normal
            float vn = glm::dot(mVelocity, constraints[0]);
            if (vn < 0.0f)
                mVelocity -= constraints[0] * vn;

        } else if (constraints.size() == 2) {
            // Two surfaces (crease/wedge) — slide along the edge between them
            Vector3 edge = glm::cross(constraints[0], constraints[1]);
            float edgeLen = glm::length(edge);
            if (edgeLen > 1e-6f) {
                edge /= edgeLen;
                mVelocity = edge * glm::dot(mVelocity, edge);
            } else {
                // Nearly parallel normals — degenerate, treat as single surface
                float vn = glm::dot(mVelocity, constraints[0]);
                if (vn < 0.0f)
                    mVelocity -= constraints[0] * vn;
            }

        } else if (constraints.size() >= 3) {
            // Corner (3+ surfaces) — project onto edge, validate direction
            Vector3 origVel = mVelocity;
            Vector3 edge = glm::cross(constraints[0], constraints[1]);
            float edgeLen = glm::length(edge);
            if (edgeLen > 1e-6f) {
                edge /= edgeLen;
                Vector3 projected = edge * glm::dot(mVelocity, edge);
                // If projected velocity reverses original direction, player is fully blocked
                if (glm::dot(projected, origVel) < 0.0f)
                    mVelocity = Vector3(0.0f);
                else
                    mVelocity = projected;
            } else {
                mVelocity = Vector3(0.0f);
            }
        }
    }

    /// Detect ground contact and update movement state.
    /// Any contact with an upward-facing normal (z > GROUND_NORMAL_MIN) counts
    /// as ground. Movement control on steep slopes is handled separately by
    /// applyMovement() via SLIDE_THRESHOLD — this only determines ground/air state.
    /// Ground probe uses FOOT sphere position for accurate floor detection.
    inline void detectGround() {
        bool onGround = false;

        // Check contacts from collision resolution (any sphere's ground contact counts)
        for (const auto &c : mLastContacts) {
            if (c.normal.z > GROUND_NORMAL_MIN) {
                onGround = true;
                mGroundNormal = c.normal;
                break;
            }
        }

        // If no contacts found from collision AND the player is not ascending,
        // probe slightly below the FOOT sphere to detect ground. This prevents
        // briefly entering Jump mode when walking over small bumps or down slopes
        // where the collision pass didn't generate contacts. The probe only
        // DETECTS ground (updates state), it doesn't move the player — gravity
        // and collision handle the actual positioning.
        // Skip when velocity.z > 0.5 to avoid cancelling a jump (the probe
        // would find the floor just left).
        if (!onGround && mCellIdx >= 0 && mVelocity.z <= 0.5f) {
            float footOffset = isCrouching() ? (FOOT_OFFSET_Z * CROUCH_SCALE) : FOOT_OFFSET_Z;
            Vector3 footCenter = mPosition + Vector3(0.0f, 0.0f, footOffset);
            Vector3 probeNormal;
            // Small probe distance — just enough to catch one frame of gravity drop
            // at 60Hz: 0.5 * g * dt² = 0.5 * 32 * (1/60)² ≈ 0.004 units.
            // Use 0.1 units for a comfortable margin.
            constexpr float GROUND_PROBE_DIST = 0.1f;
            if (mCollision.groundTest(footCenter, SPHERE_RADIUS, mCellIdx,
                                      GROUND_PROBE_DIST, probeNormal)) {
                if (probeNormal.z > GROUND_NORMAL_MIN) {
                    onGround = true;
                    mGroundNormal = probeNormal;
                }
            }
        }

        // Update movement state — mode transitions for ground/air
        if (onGround) {
            if (mCurrentMode == PlayerMode::Jump) {
                // Landing — zero vertical velocity and optionally activate
                // landing bump pose. The bump only triggers if the player was
                // falling fast enough (LANDING_MIN_VEL) and enough time has
                // elapsed since the last landing (LANDING_MIN_TIME).
                float fallSpeed = -mVelocity.z;  // positive when falling
                if (mVelocity.z < 0.0f)
                    mVelocity.z = 0.0f;

                if (fallSpeed > LANDING_MIN_VEL &&
                    (mSimTime - mLastLandingTime) > LANDING_MIN_TIME) {
                    activatePose(POSE_JUMP_LAND);
                    mLandingActive = true;
                    mLastLandingTime = mSimTime;
                }

                // Return to crouch or stand based on input
                mCurrentMode = mWantsCrouch ? PlayerMode::Crouch : PlayerMode::Stand;
            }
            // If already in a ground mode, stay in it (crouch transitions
            // are handled by updateModeTransitions)
        } else {
            if (isOnGround()) {
                // Just walked off an edge — start falling.
                // Reset motion to rest pose (no stride bob while airborne).
                mCurrentMode = PlayerMode::Jump;
                activatePose(POSE_NORMAL);
                mStrideDist = 0.0f;
            }
        }
    }

    /// Update lean — collision-limited lean amount derived from spring position.
    ///
    /// Lean is now driven through the motion pose system: setLeanDirection()
    /// activates POSE_LEAN_* poses, and the head spring naturally converges
    /// toward the lean target with organic easing. This replaces the separate
    /// cosine-eased interpolation that was previously used.
    ///
    /// This function reads the spring's lateral displacement (mSpringPos.y),
    /// checks for wall collisions at the leaned head position, and stores
    /// the collision-limited result in mLeanAmount for use by getEyePosition()
    /// and getLeanTilt().
    inline void updateLean() {
        // Read lean from spring lateral displacement
        mLeanAmount = mSpringPos.y;

        // Collision check at leaned head position — prevent leaning into walls.
        // Test the head sphere at the leaned position; if it collides, reduce
        // lean to the maximum collision-free distance.
        if (std::fabs(mLeanAmount) > 0.01f && mCellIdx >= 0) {
            float sinYaw = std::sin(mYaw);
            float cosYaw = std::cos(mYaw);
            float leanBaseZ = (isCrouching() ? HEAD_OFFSET_Z * CROUCH_SCALE : HEAD_OFFSET_Z) + mSpringPos.z;
            Vector3 leanedHead = mPosition + Vector3(0.0f, 0.0f, leanBaseZ);
            leanedHead.x += sinYaw * mLeanAmount;
            leanedHead.y -= cosYaw * mLeanAmount;

            auto contacts = mCollision.sphereVsCellPolygons(
                leanedHead, SPHERE_RADIUS, mCellIdx);
            if (!contacts.empty()) {
                // Find the maximum safe lean by binary search / step back.
                // Simple approach: halve the lean until no collision.
                for (int i = 0; i < 4; ++i) {
                    mLeanAmount *= 0.5f;
                    leanedHead = mPosition + Vector3(0.0f, 0.0f, leanBaseZ);
                    leanedHead.x += sinYaw * mLeanAmount;
                    leanedHead.y -= cosYaw * mLeanAmount;
                    contacts = mCollision.sphereVsCellPolygons(
                        leanedHead, SPHERE_RADIUS, mCellIdx);
                    if (contacts.empty())
                        break;
                }
                // If still colliding after 4 halves (lean < 6% of original),
                // zero the lean completely.
                if (!contacts.empty())
                    mLeanAmount = 0.0f;
            }
        }
    }

    /// Check for mantleable ledge using 3-ray detection.
    /// Called each fixedStep when the player is airborne and pressing forward.
    ///
    /// Detection phases:
    /// 1. Cast UP from head — must NOT hit (headroom check)
    /// 2. Cast FORWARD from top — must NOT hit (clear path to ledge)
    /// 3. Cast DOWN from forward position — MUST hit (ledge surface)
    inline bool checkMantle() {
        if (mCurrentMode != PlayerMode::Jump || mInputForward < 0.1f || mMantling)
            return false;

        float heightScale = isCrouching() ? CROUCH_SCALE : 1.0f;
        Vector3 headPos = mPosition + Vector3(0.0f, 0.0f, HEAD_OFFSET_Z * heightScale);

        // Phase 1: upward — check headroom
        Vector3 upTarget = headPos + Vector3(0.0f, 0.0f, MANTLE_UP_DIST);
        RayHit hit;
        if (raycastWorld(mCollision.getWR(), headPos, upTarget, hit))
            return false;  // ceiling too low

        // Phase 2: forward — check clear path
        float cosY = std::cos(mYaw);
        float sinY = std::sin(mYaw);
        Vector3 fwd(cosY, sinY, 0.0f);
        Vector3 fwdTarget = upTarget + fwd * MANTLE_FWD_DIST;
        if (raycastWorld(mCollision.getWR(), upTarget, fwdTarget, hit))
            return false;  // wall in the way

        // Phase 3: downward — find ledge surface
        Vector3 downTarget = fwdTarget + Vector3(0.0f, 0.0f, -MANTLE_DOWN_DIST);
        if (!raycastWorld(mCollision.getWR(), fwdTarget, downTarget, hit))
            return false;  // no ledge found

        // Found a mantleable surface — compute target position
        // Place body center above the ledge surface with FOOT sphere on the ledge
        float footZ = isCrouching() ? FOOT_OFFSET_Z * CROUCH_SCALE : FOOT_OFFSET_Z;
        mMantleTarget = hit.point;
        mMantleTarget.z -= footZ;  // body center above ledge (FOOT_OFFSET_Z is negative)
        mMantleTarget.z += SPHERE_RADIUS;  // surface + sphere radius clearance

        startMantle();
        return true;
    }

    /// Begin the mantle animation — 5-state machine.
    inline void startMantle() {
        mMantling = true;
        mMantleState = MantleState::Hold;
        mMantleTimer = 0.0f;
        mMantleStartPos = mPosition;
    }

    /// Update the mantle state machine each fixedStep.
    /// During mantling, normal movement and gravity are suppressed.
    /// The position is driven directly by the state machine.
    inline void updateMantle() {
        if (!mMantling) return;

        mMantleTimer += FIXED_DT;

        switch (mMantleState) {
        case MantleState::Hold:
            // Phase 1: hold position, suppress physics
            mVelocity = Vector3(0.0f);
            if (mMantleTimer >= MANTLE_HOLD_TIME) {
                mMantleState = MantleState::Rise;
                mMantleTimer = 0.0f;
            }
            break;

        case MantleState::Rise:
            // Phase 2: rise vertically toward target Z via spring-like motion
            mVelocity = Vector3(0.0f);
            {
                float targetZ = mMantleTarget.z;
                float dz = targetZ - mPosition.z;
                if (std::fabs(dz) < 0.1f) {
                    // Close enough — snap and advance
                    mPosition.z = targetZ;
                    mMantleState = MantleState::Forward;
                    mMantleTimer = 0.0f;
                } else {
                    // Move toward target at spring-controlled rate
                    float riseSpeed = dz * HEAD_SPRING_TENSION / FIXED_DT * 0.5f;
                    riseSpeed = std::max(-20.0f, std::min(20.0f, riseSpeed));
                    mPosition.z += riseSpeed * FIXED_DT;
                }
            }
            break;

        case MantleState::Forward:
            // Phase 3: move horizontally onto ledge
            mVelocity = Vector3(0.0f);
            {
                float dx = mMantleTarget.x - mPosition.x;
                float dy = mMantleTarget.y - mPosition.y;
                float hDist = std::sqrt(dx * dx + dy * dy);
                if (hDist < 0.1f || mMantleTimer >= MANTLE_FWD_TIME) {
                    mPosition.x = mMantleTarget.x;
                    mPosition.y = mMantleTarget.y;
                    mMantleState = MantleState::StandUp;
                    mMantleTimer = 0.0f;
                } else {
                    float frac = FIXED_DT / std::max(0.01f, MANTLE_FWD_TIME - mMantleTimer + FIXED_DT);
                    mPosition.x += dx * frac;
                    mPosition.y += dy * frac;
                }
            }
            break;

        case MantleState::StandUp:
            // Phase 4: restore normal standing — spring handles the visual transition
            mVelocity = Vector3(0.0f);
            mMantleState = MantleState::Complete;
            mMantleTimer = 0.0f;
            break;

        case MantleState::Complete:
            // Phase 5: wait for ground contact, then end mantle
            // Apply gravity so the player settles onto the ledge
            mVelocity.z -= mGravityMag * FIXED_DT;
            if (isOnGround() || mMantleTimer > 1.0f) {
                // Done mantling — return to normal mode
                mMantling = false;
                mMantleState = MantleState::None;
                mCurrentMode = mWantsCrouch ? PlayerMode::Crouch : PlayerMode::Stand;
                mVelocity = Vector3(0.0f);
            }
            break;

        default:
            mMantling = false;
            mMantleState = MantleState::None;
            break;
        }
    }

    /// Update cell index from current position
    inline void updateCell() {
        int32_t newCell = mCollision.findCell(mPosition);
        if (newCell >= 0)
            mCellIdx = newCell;
        // If newCell < 0, keep the old cell (player might be at a boundary)
    }

    /// Update mode transitions — handles crouch, jump→ground, ground→jump, swim.
    /// Replaces the old updateModeTransitions(). Body center shifts and spring compensation
    /// for crouch/uncrouch are preserved — just driven by mode transitions now.
    inline void updateModeTransitions() {
        // Crouch spring compensation: when the body center shifts and the base
        // offset changes (standing→crouching), the spring Z must absorb the full
        // eye height difference so the eye stays at the same world position.
        // The spring then naturally converges back to zero, creating a smooth
        // crouch/uncrouch transition.
        //
        // Total eye shift = body center shift + base offset change
        //   = CROUCH_CENTER_DROP + (HEAD_OFFSET_Z * CROUCH_SCALE - HEAD_OFFSET_Z)
        //   = -1.5 + (0.9 - 1.8) = -2.4
        // Spring must compensate by +2.4 (crouching) or -2.4 (un-crouching).
        static constexpr float CROUCH_EYE_SHIFT =
            HEAD_OFFSET_Z - CROUCH_CENTER_DROP - HEAD_OFFSET_Z * CROUCH_SCALE;
            // = 1.8 - (-1.5) - 0.9 = 2.4

        // ── Crouch transitions (only on ground modes) ──
        if (isOnGround()) {
            if (mWantsCrouch && !isCrouching()) {
                // Start crouching — body center drops to keep feet on ground
                mCurrentMode = PlayerMode::Crouch;
                mPosition.z += CROUCH_CENTER_DROP; // drops by 1.5 (negative value)
                // Compensate spring Z to keep eye at same world position.
                // Spring will converge back to zero → smooth crouch transition.
                mSpringPos.z += CROUCH_EYE_SHIFT;
            } else if (!mWantsCrouch && isCrouching()) {
                // Try to un-crouch — check if there's room above.
                // Test HEAD sphere at standing height (body center raised back up)
                Vector3 futureCenter = mPosition;
                futureCenter.z -= CROUCH_CENTER_DROP; // raise center by 1.5
                Vector3 standingHeadPos = futureCenter + Vector3(0.0f, 0.0f, HEAD_OFFSET_Z);
                auto headContacts = mCollision.sphereVsCellPolygons(
                    standingHeadPos, SPHERE_RADIUS, mCellIdx);

                // Only un-crouch if head has no collisions at standing height
                if (headContacts.empty()) {
                    mCurrentMode = PlayerMode::Stand;
                    mPosition.z -= CROUCH_CENTER_DROP; // raise center by 1.5
                    // Compensate spring Z for reverse transition
                    mSpringPos.z -= CROUCH_EYE_SHIFT;
                }
                // else: ceiling too low, stay crouched
            }
        }

        // ── Swim detection — enter/exit water based on cell media type ──
        if (mCellIdx >= 0) {
            uint8_t media = mCollision.getMediaType(mPosition);
            if (media == 2 && mCurrentMode != PlayerMode::Swim) {
                mCurrentMode = PlayerMode::Swim;
            } else if (media != 2 && mCurrentMode == PlayerMode::Swim) {
                mCurrentMode = mWantsCrouch ? PlayerMode::Crouch : PlayerMode::Stand;
            }
        }
    }

    /// Activate a new motion pose — captures current offset as blend start,
    /// sets new target, and begins linear interpolation toward it.
    /// The head spring provides organic ease-in/ease-out on top of the
    /// constant-velocity linear blend.
    inline void activatePose(const MotionPoseData &pose) {
        mPoseStart = mPoseCurrent;  // capture current position as blend origin
        mPoseEnd = Vector3(pose.fwd, pose.lat, pose.vert);
        mPoseDuration = pose.duration;
        mPoseHoldTime = pose.holdTime;
        mPoseTimer = 0.0f;
        mPoseHolding = false;
    }

    /// Activate the next stride pose (alternates left/right).
    /// Each stride is chained as {footfall, recovery} via the motion queue.
    /// The footfall dips the camera down; the recovery returns it toward the
    /// mode's rest height. This creates the natural up-down bounce of walking.
    /// Uses the per-mode motion config table — each mode has its own stride poses.
    inline void activateStride() {
        auto motionCfg = getModeMotion(mCurrentMode);

        // Build a recovery pose targeting the mode's rest position with short duration.
        // Stored as member so the pointer remains valid for the motion queue.
        mStrideRecovery = {0.12f, 0.0f,
                           motionCfg.restPose->fwd,
                           motionCfg.restPose->lat,
                           motionCfg.restPose->vert};

        if (mStrideIsLeft) {
            activatePoseList({motionCfg.strideLeft, &mStrideRecovery});
        } else {
            activatePoseList({motionCfg.strideRight, &mStrideRecovery});
        }
        mStrideIsLeft = !mStrideIsLeft;
    }

    /// Activate a sequence of poses as a LIFO queue.
    /// The first pose activates immediately; the rest are pushed onto the queue
    /// in reverse order (last pushed = first popped). When each pose completes,
    /// the next is popped from the queue. Allow for compound motions (weapon swing + recovery,
    /// mantle phases, etc.).
    inline void activatePoseList(std::initializer_list<const MotionPoseData*> poses) {
        mMotionQueue.clear();
        auto it = poses.begin();
        if (it == poses.end()) return;

        // Force-activate the first pose immediately
        activatePose(**it);
        ++it;

        // Push remaining in reverse order (LIFO: last pushed = first popped)
        for (auto end = poses.end(); it != end; ++it)
            mMotionQueue.push_back(*it);
        std::reverse(mMotionQueue.begin(), mMotionQueue.end());
    }

    /// Compute velocity-dependent footstep distance.
    inline float computeFootstepDist(float hSpeed) const {
        if (hSpeed < STRIDE_SPEED_LOW)
            return STRIDE_DIST_MIN;
        if (hSpeed > STRIDE_SPEED_HIGH)
            return STRIDE_DIST_MAX;
        return STRIDE_DIST_MIN
             + (STRIDE_DIST_MAX - STRIDE_DIST_MIN)
             * ((hSpeed - STRIDE_SPEED_LOW) / (STRIDE_SPEED_HIGH - STRIDE_SPEED_LOW));
    }

    /// Update motion pose system — distance-based stride triggering.
    ///
    /// Strides trigger when the foot's accumulated travel distance exceeds
    /// the velocity-dependent footstep distance (computeFootstepDist).
    /// The pose blend animation plays over its duration independently.
    /// Foot submodel distance trigger.
    ///
    /// Velocity thresholds:
    ///   > 1.0 units/sec — start tracking foot distance
    ///   > 1.5 units/sec — trigger head bob motion (stride activation)
    ///
    /// When the player stops, the current pose blend completes and
    /// transitions to the mode's rest pose.
    inline void updateMotionPose() {
        float hSpeed = std::sqrt(mVelocity.x * mVelocity.x +
                                  mVelocity.y * mVelocity.y);
        // Lean suppresses stride bob — lean poses take priority over stride.
        // When leaning, the spring's lateral axis is driven by the lean offset,
        // so stride lateral bob would conflict.
        bool isLeaning = (mLeanDir != 0);
        bool isWalking = (isOnGround() && hSpeed > 1.5f && !isLeaning);

        // Advance pose timer
        mPoseTimer += FIXED_DT;

        // Compute current interpolated pose offset
        bool poseReady = false;  // true when current pose blend+hold is complete
        if (mPoseHolding) {
            // At target, holding — mPoseCurrent stays at mPoseEnd
            mPoseCurrent = mPoseEnd;
            // Check if hold time expired → pose is ready for next transition
            if (mPoseTimer >= mPoseHoldTime) {
                poseReady = true;
            }
        } else if (mPoseDuration <= 0.0f) {
            // Instant pose (dur=0, e.g. landing bump) — snap to target, start hold
            mPoseCurrent = mPoseEnd;
            mPoseHolding = true;
            mPoseTimer = 0.0f;
        } else {
            // Linear interpolation from start to target over the full duration.
            // A cumulative progressive blend  (current += (target - current) * (t/T) )converges
            // faster at higher framerates. At 20-30fps, common in the early 2000s, this produced
            // smooth motion, but at our fixed 60Hz physics rate it converges ~99% in 1/3 of the
            // duration, creating a "snap then hold" pattern.
            // Linear interpolation (current = start + (target - start) * t/T) is framerate-independent
            // and uses the full duration — the head spring provides organic acceleration/deceleration.
            if (mPoseTimer >= mPoseDuration) {
                // Blend complete — snap to target, enter hold phase
                mPoseCurrent = mPoseEnd;
                mPoseHolding = true;
                mPoseTimer = 0.0f;
            } else {
                float t = mPoseTimer / mPoseDuration;
                mPoseCurrent = mPoseStart + (mPoseEnd - mPoseStart) * t;
            }
        }

        // ── Distance-based stride accumulation ──
        // Accumulate foot travel distance when moving above tracking threshold.
        // This drives stride triggering — footstep animation is distance-based,
        // not time-based, foot submodel tracking.
        bool strideTriggered = false;
        if (isOnGround() && hSpeed > 1.0f) {
            mStrideDist += hSpeed * FIXED_DT;
            float footstepDist = computeFootstepDist(hSpeed);
            if (isWalking && mStrideDist >= footstepDist) {
                strideTriggered = true;
                mStrideDist -= footstepDist;  // carry over excess distance
            }
        } else {
            mStrideDist = 0.0f;
        }

        // Per-mode rest pose (idle position for current mode)
        const MotionPoseData &restPose = *getModeMotion(mCurrentMode).restPose;

        // ── Pose chaining state machine ──
        //
        // Priority order:
        // 1. Motion queue — if a queued sequence is active, pop next on completion
        // 2. Landing bump — must complete before stride resumes
        // 3. Walking strides — distance-triggered
        // 4. Idle — return to mode's rest pose

        // Motion queue takes priority — compound motions (weapon swing, mantle)
        // play through their full sequence regardless of walking state.
        if (!mMotionQueue.empty() && poseReady) {
            activatePose(*mMotionQueue.back());
            mMotionQueue.pop_back();
        }
        // Landing bump takes priority over normal strides
        else if (mLandingActive) {
            if (poseReady) {
                mLandingActive = false;
                // After landing, resume walking or return to idle
                if (isWalking) {
                    activateStride();
                    mStrideDist = 0.0f;
                } else {
                    activatePose(restPose);
                }
            }
        }
        // Walking: trigger strides based on foot travel distance
        else if (isWalking) {
            if (!mWasWalking) {
                // Just started walking — activate first stride immediately
                activateStride();
                mStrideDist = 0.0f;
            } else if (strideTriggered) {
                // Foot traveled enough distance — activate next stride
                activateStride();
            }
        }
        // Not walking: return to mode's rest pose when current pose completes
        else {
            if (mWasWalking && !mLandingActive) {
                // Just stopped walking — let current blend finish, then go to rest.
                // If already holding (blend done), transition immediately.
                if (poseReady) {
                    activatePose(restPose);
                }
                // If still blending, the pose will complete naturally,
                // then poseReady fires next frame and we'll catch it below.
            } else if (poseReady && glm::length(mPoseEnd) > 0.01f) {
                // Idle, current non-zero pose completed — return to rest
                activatePose(restPose);
            }
        }

        mWasWalking = isWalking;
    }

    /// Update 3D head spring.
    ///
    /// The spring tracks the motion pose's interpolated offset in player-local
    /// coordinates {forward, lateral, vertical}. The formula:
    ///
    ///   tension_eff = tension / dt     (= 0.6 / (1/60) = 36.0 at 60fps)
    ///   damping_eff = damping + (1 - damping) * dt   (= 0.02 + 0.98/60 ≈ 0.0363)
    ///
    ///   vel = displacement * tension_eff
    ///   vel.z *= 0.5        // vertical axis half-strength
    ///   vel += prev_vel * damping_eff
    ///
    ///   if (|vel| > cap) vel *= cap / |vel|
    ///   pos += vel * dt
    ///
    /// At 60fps this converges ~60% per frame horizontally, ~30% vertically.
    /// Very overdamped — no oscillation, smooth and natural.
    inline void updateHeadSpring() {
        // Target = current interpolated motion pose offset
        Vector3 target = mPoseCurrent;

        // Displacement from current spring position to target
        Vector3 disp = target - mSpringPos;

        // Spring dt clamping — protection against lag spikes and near-zero
        // timesteps that could destabilize the spring.
        float springDt = FIXED_DT;
        if (springDt > 0.05f) springDt = 0.05f * 0.6f;  // lag spike protection
        if (springDt < 0.001f) springDt = 0.001f;        // division-by-zero protection

        // Effective tension: tension / dt — scales displacement into velocity
        // At 60fps: 0.6 / (1/60) = 36.0
        float tensionEff = HEAD_SPRING_TENSION / springDt;

        // Effective damping: retains a tiny fraction of previous velocity
        // At 60fps: 0.02 + 0.98 * (1/60) ≈ 0.0363
        float dampingEff = HEAD_SPRING_DAMPING + (1.0f - HEAD_SPRING_DAMPING) * springDt;

        // New velocity = displacement-proportional term + damped previous velocity
        Vector3 newVel = disp * tensionEff;
        newVel.z *= HEAD_SPRING_Z_SCALE;  // vertical axis half-strength
        newVel += mSpringVel * dampingEff;

        // Velocity cap — prevents extreme values during large displacements
        // (e.g. crouch transition, landing bump)
        float velMag = glm::length(newVel);
        if (velMag > HEAD_SPRING_CAP) {
            newVel *= HEAD_SPRING_CAP / velMag;
        }

        mSpringVel = newVel;
        mSpringPos += mSpringVel * springDt;

        // Clamp to sane range — ±3.0 accommodates crouch transition where
        // body center shifts ±1.5 and the spring must track the large
        // displacement back to zero.
        mSpringPos.x = std::max(-3.0f, std::min(3.0f, mSpringPos.x));
        mSpringPos.y = std::max(-3.0f, std::min(3.0f, mSpringPos.y));
        mSpringPos.z = std::max(-3.0f, std::min(3.0f, mSpringPos.z));
    }

    // ── State ──

    const CollisionGeometry &mCollision;  // world collision geometry (not owned)

    Vector3 mPosition{0.0f};       // body center position (ground + 4.2 on flat ground)
    Vector3 mVelocity{0.0f};       // linear velocity
    float mYaw = 0.0f;            // look direction (radians)
    float mInputForward = 0.0f;   // movement input [-1, 1]
    float mInputRight   = 0.0f;   // strafe input [-1, 1]

    int32_t mCellIdx = -1;        // current WR cell index
    PlayerMode mCurrentMode = PlayerMode::Jump;

    bool mWantsCrouch    = false;   // crouch input pressed (drives Stand↔Crouch transition)
    bool mJumpRequested  = false;  // jump pending next step
    bool mSneaking       = false;   // sneaking (creep mode, 0.5x speed)
    bool mRunning        = false;   // running (2x speed)
    bool mMotionDisabled = false;   // motion disabled (cutscenes, death)
    bool mMantling       = false;   // mantle animation in progress
    MantleState mMantleState = MantleState::None;  // current mantle phase
    float mMantleTimer   = 0.0f;   // time in current mantle phase
    Vector3 mMantleTarget{0.0f};   // target position for mantle completion
    Vector3 mMantleStartPos{0.0f}; // position at mantle start

    float mGravityMag = GRAVITY;  // current gravity magnitude
    float mTimeAccum  = 0.0f;     // fixed-timestep accumulator
    float mSimTime    = 0.0f;     // total simulation time (for landing throttle, etc.)
    float mLastLandingTime = -1.0f; // time of last landing event (for throttling)

    // ── 3D head spring state ──
    // Spring tracks motion pose targets in player-local coords {fwd, lat, vert}.
    // Position is displacement from the stance rest offset; the spring naturally
    // converges toward the current pose target.
    Vector3 mSpringPos{0.0f};     // current spring displacement {fwd, lat, vert}
    Vector3 mSpringVel{0.0f};     // spring velocity

    // ── Motion pose state ──
    // Discrete stride-driven pose system. Each stride triggers a new target
    // that the spring tracks. Linear interpolation from start to end over
    // the pose duration; the head spring adds organic ease-in/ease-out.
    Vector3 mPoseStart{0.0f};     // blend start offset (captured in activatePose)
    Vector3 mPoseEnd{0.0f};       // blend target offset {fwd, lat, vert}
    Vector3 mPoseCurrent{0.0f};   // current interpolated pose offset
    float mPoseTimer    = 0.0f;   // elapsed time in current blend/hold
    float mPoseDuration = 0.8f;   // current blend duration (from pose data)
    float mPoseHoldTime = 0.0f;   // current hold time at target
    bool  mPoseHolding  = true;   // true = holding at target, false = blending

    // Stride tracking — distance-based triggering.
    // Strides activate when foot travel distance exceeds computeFootstepDist().
    bool  mStrideIsLeft  = true;  // which foot is next (alternates each stride)
    bool  mWasWalking    = false; // was the player walking last frame (edge detection)
    bool  mLandingActive = false; // true while landing bump pose is active
    float mStrideDist    = 0.0f;  // accumulated horizontal foot travel distance

    // Stride recovery pose — built dynamically in activateStride() targeting the
    // mode's rest position with short duration. Stored as member so the pointer
    // remains valid for the motion queue.
    MotionPoseData mStrideRecovery = {0.12f, 0.0f, 0.0f, 0.0f, 0.0f};

    // Motion queue — LIFO stack for chaining multi-pose sequences.
    // When a pose completes, the next is popped from the back.
    // Used for compound motions (weapon swing → recovery, mantle phases, etc.)
    std::vector<const MotionPoseData*> mMotionQueue;

    // Ground contact normal from last collision pass
    Vector3 mGroundNormal{0.0f, 0.0f, 1.0f};

    // Lean state — driven through motion poses (spring provides easing)
    int   mLeanDir        = 0;    // lean direction: -1=left, 0=center, +1=right
    float mLeanAmount     = 0.0f; // collision-limited lateral offset (derived from spring each frame)

    // Contacts from last collision resolution (used by detectGround and velocity removal)
    std::vector<SphereContact> mLastContacts;
};

} // namespace Darkness

#endif // __PLAYERPHYSICS_H
