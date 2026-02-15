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
 *    TODO: Player physics (headbob, lean, anything that depends on dt) is ONLY
 *          vintage correct at 12.5hz, at higher rendering rates, these actions
 *          can seem exaggerated. We will leave them as is for now and revisit
 *          later.
 *
 *    Uses a 5-submodel player model:
 *    HEAD (+1.8, r=1.2), BODY (-0.6, r=1.2) — real collision spheres
 *    SHIN (-2.2, r=0), KNEE (-2.6, r=0), FOOT (-3.0, r=0) — point detectors
 *    HEAD and BODY handle wall/ceiling collision; the lower three are zero-radius
 *    point detectors that sense floor contact and ground slope. All five are
 *    rigidly attached to a single body center.
 *
 *    Head bob is driven by discrete motion poses — target offsets that the
 *    head spring tracks. Each stride activates a new pose target, and the
 *    3D spring naturally smooths the transitions, creating realistic bob
 *    with organic acceleration/deceleration.
 *
 *    Constants:
 *    - Walk speed: 11.0 units/sec, Run: 22.0, Creep: 5.5
 *    - HEAD/BODY sphere radius: 1.2, KNEE/SHIN/FOOT: 0.0 (point detectors)
 *    - 5 submodel offsets from body center (Z-up): +1.8, -0.6, -2.2, -2.6, -3.0
 *    - Gravity: ~32 units/sec² (runtime-configurable)
 *    - Configurable timestep: 12.5/60/120 Hz via PhysicsTimestep presets
 *
 *    See NOTES.SOURCE.md for full physics constants documentation.
 *
 *****************************************************************************/

#ifndef __PLAYERPHYSICS_H
#define __PLAYERPHYSICS_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
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
    // 5-submodel player model (HEAD, BODY, SHIN, KNEE, FOOT) with vertical offsets.
    // HEAD and BODY are real collision spheres (radius 1.2).
    // SHIN, KNEE, FOOT are zero-radius point detectors — they sense floor/ground
    // contacts but have no physical volume. Only 2 of 5 submodels are true spheres.

    // Collision sphere radius for HEAD and BODY submodels
    static constexpr float SPHERE_RADIUS = 1.2f;

    // Player height parameters
    static constexpr float STAND_HEIGHT   = 6.0f;    // total standing height

    // 5-sphere center offsets relative to body center (Z-up).
    // Standing on flat ground: FOOT point at ground (r=0) → body center at ground + 3.0
    // → HEAD center at ground + 4.8 → eye at ground + 5.6.
    static constexpr float HEAD_OFFSET_Z  =  1.8f;   // (STAND_HEIGHT/2) - SPHERE_RADIUS
    static constexpr float BODY_OFFSET_Z  = -0.6f;   // (STAND_HEIGHT/2) - 3*SPHERE_RADIUS
    static constexpr float SHIN_OFFSET_Z  = -2.2f;   // -(STAND_HEIGHT * 11/30)
    static constexpr float KNEE_OFFSET_Z  = -2.6f;   // -(STAND_HEIGHT * 13/30)
    static constexpr float FOOT_OFFSET_Z  = -3.0f;   // -(STAND_HEIGHT/2)

    // Number of submodels and offset array for iteration
    static constexpr int NUM_SPHERES = 5;
    static constexpr float SPHERE_OFFSETS[NUM_SPHERES] = {
        HEAD_OFFSET_Z,   // 0: HEAD   +1.8
        BODY_OFFSET_Z,   // 1: BODY   -0.6
        SHIN_OFFSET_Z,   // 2: SHIN   -2.2
        KNEE_OFFSET_Z,   // 3: KNEE   -2.6
        FOOT_OFFSET_Z    // 4: FOOT   -3.0
    };

    // Per-submodel collision radii.
    // HEAD and BODY are real collision spheres that push the body away from walls.
    // SHIN, KNEE, FOOT are point detectors (radius 0.0) — they sense floor/terrain
    // contacts but have no physical volume. The point detectors participate in
    // collision response (pushing the body up when they penetrate the floor) but
    // don't inflate the player's width at leg level.
    static constexpr float SPHERE_RADII[NUM_SPHERES] = {
        SPHERE_RADIUS,   // 0: HEAD   — real collision sphere
        SPHERE_RADIUS,   // 1: BODY   — real collision sphere
        0.0f,            // 2: SHIN   — point detector
        0.0f,            // 3: KNEE   — point detector
        0.0f,            // 4: FOOT   — point detector
    };

    // Per-submodel crouch offsets.
    // During crouch, HEAD and BODY physically drop by these amounts. SHIN/KNEE/FOOT
    // stay at their standing offsets (feet don't move during crouch). The body center
    // position stays the same — only submodel positions change.
    //
    //   Crouch:     HEAD {0, 0, -2.02},  BODY {0, 0, -1.0}
    //   CrawlLeft:  HEAD {0, -0.15, -2.5}, BODY {0, 0, -1.0}
    //   CrawlRIght: HEAD {0, 0.15, -2.5},  BODY {0, 0, -1.0}
    static constexpr float CROUCH_HEAD_DROP = -2.02f;  // HEAD sphere drops 2.02 units
    static constexpr float CROUCH_BODY_DROP = -1.0f;   // BODY sphere drops 1.0 unit

    // Per-submodel crouch collision offsets (added to SPHERE_OFFSETS during crouch).
    // HEAD and BODY drop for ceiling clearance, lower submodels stay put.
    static constexpr float CROUCH_OFFSETS[NUM_SPHERES] = {
        CROUCH_HEAD_DROP,  // 0: HEAD   drops by -2.02
        CROUCH_BODY_DROP,  // 1: BODY   drops by -1.0
        0.0f,              // 2: SHIN   stays at -2.2
        0.0f,              // 3: KNEE   stays at -2.6
        0.0f,              // 4: FOOT   stays at -3.0
    };

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
    // Slope handling — friction-based.
    // GROUND_NORMAL_MIN: any surface with normal.z above this counts as "ground" for
    // state/landing purposes. The 5-sphere model and movement control handle the rest.
    // SLIDE_THRESHOLD: below this normal.z, movement control is progressively reduced,
    // causing the player to slide down steep slopes under gravity.
    static constexpr float GROUND_NORMAL_MIN = 0.1f;   // cos(~84°) — any upward surface
    static constexpr float SLIDE_THRESHOLD   = 0.64f;  // cos(~50°) — below this, sliding

    // Collision iteration count — now set per-preset via PhysicsTimestep.collisionIters.
    // Vintage (12.5Hz): 3 iterations.
    // Modern/Ultra: 1 iteration (higher Hz compensates with more steps/sec).

    // Landing detection thresholds.
    static constexpr float LANDING_MIN_VEL  = 2.0f;    // minimum downward speed for landing pose
    static constexpr float LANDING_MIN_TIME = 0.2f;    // minimum 200ms between landing events

    // Mantle system — 3-ray detection + 5-state animation
    static constexpr float MANTLE_UP_DIST   = 3.5f;    // upward headroom check distance
    static constexpr float MANTLE_FWD_DIST  = 2.4f;    // forward ledge reach (radius × 2)
    static constexpr float MANTLE_DOWN_DIST = 7.0f;    // downward surface scan distance
    static constexpr float MANTLE_HOLD_TIME = 0.3f;    // Phase 1: hold & compress duration
    static constexpr float MANTLE_FWD_TIME  = 0.4f;    // Phase 3: forward movement duration

    // Movement control — exponential convergence impulse system.
    // Instead of constant acceleration/deceleration,
    // velocity converges toward the desired value exponentially each step.
    //
    // The convergence rate alpha = CONTROL_MULTIPLIER * contactFriction * dt / VELOCITY_RATE
    // where contactFriction = FRICTION_FACTOR * groundNormal.z * gravityMag.
    // On flat ground: contactFriction = 0.03 * 1.0 * 32.0 = 0.96
    //   alpha at 60Hz = 11.0 * 0.96 * 0.0167 = 0.176 per step
    //   alpha at 12.5Hz = 11.0 * 0.96 * 0.08 = 0.845 per step
    //
    // This gives exponential approach with time constant tau = 1/(11*0.96) = 0.095s.
    // Deceleration uses the same formula with desired=0 (no separate decel constant).
    // Mass cancels in the derivation: rate = 11 * mass * friction / velocity_rate,
    // then alpha = rate * dt / mass = 11 * friction * dt / velocity_rate.
    static constexpr float CONTROL_MULTIPLIER = 11.0f;
    static constexpr float FRICTION_FACTOR    = 0.03f;
    static constexpr float VELOCITY_RATE      = 1.0f;

    // Air control fraction.
    static constexpr float AIR_CONTROL_FRAC = 0.3f;

    /// Physics timestep configuration — bundles rate-dependent parameters.
    /// Spring constants are NOT here (they're identical across all presets).
    struct PhysicsTimestep {
        float fixedDt;          // seconds per physics step
        int   collisionIters;   // constraint projection iterations per step
    };

    static constexpr PhysicsTimestep VINTAGE = { 1.0f / 12.5f, 3 };
    static constexpr PhysicsTimestep MODERN  = { 1.0f / 60.0f, 1 };
    static constexpr PhysicsTimestep ULTRA   = { 1.0f / 120.0f, 1 };

    // ── Head spring constants
    //
    // The spring uses dt-dependent scaling with a clamped spring dt:
    //   springDt = min(dt, 0.05)               (clamps to 50ms)
    //   tension_eff = BASE_TENSION / springDt   (at 12.5Hz: 0.6/0.05 = 12.0)
    //   damping_eff = BASE_DAMPING + (1-BASE_DAMPING) * springDt  (at 12.5Hz: 0.069)
    //   vel = displacement * tension_eff + old_vel * damping_eff
    //   vel.z *= Z_SCALE                       (Z-axis half-strength)
    //   pos += vel * dt                        (position uses FULL dt, not clamped)
    //
    // The dt clamping at 50ms makes the spring stiffer at 12.5Hz (80ms steps)
    // than a naive implementation: tension_eff=12.0 instead of 7.5 (+60%),
    // damping_eff=0.069 instead of 0.098 (-30%).
    //
    // Note: this formula is NOT rate-independent (behavior differs at 60Hz).
    // Once the 12.5Hz behavior is confirmed correct, a rate-independent
    // analytical solution can be derived to match at all timesteps.
    static constexpr float HEAD_SPRING_BASE_TENSION = 0.6f;   // spring tension constant
    static constexpr float HEAD_SPRING_BASE_DAMPING = 0.02f;  // velocity retention base
    static constexpr float HEAD_SPRING_Z_SCALE      = 0.5f;   // Z-axis half-strength
    static constexpr float HEAD_SPRING_VEL_CAP      = 25.0f;  // max spring velocity magnitude
    static constexpr float HEAD_SPRING_MAX_DT       = 0.05f;  // spring dt cap (50ms) — clamps dt for spring calc

    // Eye position above head submodel center.
    // Camera is 0.8 units above the head sphere center for natural eye height.
    static constexpr float EYE_ABOVE_HEAD     = 0.8f;

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
    // Motion coordinator blends offsets
    // progressively toward targets over a duration. The head spring then tracks
    // the blended offset, adding overshoot and organic smoothing. Both systems
    // work together: the blend shapes the target trajectory, the spring adds
    // dynamic response.
    //
    // Durations: 0.6s for strides, 0.8s for
    // mode transitions (stand/crouch), 1.5s for lean (slow easing). Landing
    // and weapon impacts are instant (0.0s) for sharp feel.
    //
    // Normal walking/idle (submodel 0 head offset):
    static constexpr MotionPoseData POSE_NORMAL       = {0.8f,  0.0f,  0.0f,   0.0f,    0.0f};

    // Walking strides — progressive blend over 0.6s.
    // The motion coordinator blends mPoseCurrent toward the stride target over the
    // duration, then the head spring tracks mPoseCurrent with spring dynamics.
    // Both the blend curve and the spring's overshoot/lag contribute to natural bob.
    //
    // Both strides use identical vertical dip (-0.4) and symmetric lateral sway (±0.1).
    // Lateral sign convention: original Y=LEFT-positive → our lat=RIGHT-positive,
    // so left stride sways right (+0.1) and right stride sways left (-0.1),
    // pushing the body AWAY from the stepping foot.
    static constexpr MotionPoseData POSE_STRIDE_LEFT  = {0.6f,  0.01f, 0.0f,   0.1f,   -0.4f};
    static constexpr MotionPoseData POSE_STRIDE_RIGHT = {0.6f,  0.01f, 0.0f,  -0.1f,   -0.4f};

    // Crouching strides — slightly wider sway, deeper vertical offset from crouch height.
    // Vert is relative to POSE_CROUCH (-2.02). Both crawl strides use identical vertical
    // dip (-2.5) and symmetric
    // lateral sway (±0.15), with same sign convention as standing strides.
    static constexpr MotionPoseData POSE_CRAWL_LEFT   = {0.6f,  0.01f, 0.0f,   0.15f,  -2.5f};
    static constexpr MotionPoseData POSE_CRAWL_RIGHT  = {0.6f,  0.01f, 0.0f,  -0.15f,  -2.5f};

    // Crouching idle — blend over 0.8s for smooth crouch transition:
    static constexpr MotionPoseData POSE_CROUCH       = {0.8f,  0.0f,  0.0f,   0.0f,   -2.02f};

    // Landing impact — instantaneous dip on landing (dur=0 = snap), held briefly:
    static constexpr MotionPoseData POSE_JUMP_LAND    = {0.0f,  0.1f,  0.0f,   0.0f,   -0.5f};

    // Body carry mode (carrying a body or heavy object):
    // Heavier bob — deeper dip and more lateral sway while carrying.
    static constexpr MotionPoseData POSE_CARRY_IDLE   = {0.8f,  0.0f,  0.0f,   0.0f,   -0.8f};
    static constexpr MotionPoseData POSE_CARRY_LEFT   = {0.6f,  0.0f,  0.0f,   0.5f,   -1.5f};
    static constexpr MotionPoseData POSE_CARRY_RIGHT  = {0.6f,  0.0f,  0.0f,  -0.15f,  -1.1f};

    // Weapon swing (head bob during sword/blackjack attacks):
    // Instant target — spring overshoot creates natural recoil feel.
    static constexpr MotionPoseData POSE_WEAPON_SWING       = {0.0f,  0.0f, 0.8f,  0.0f,   0.0f};
    static constexpr MotionPoseData POSE_WEAPON_SWING_CROUCH = {0.0f, 0.0f, 0.8f,  0.0f,  -2.02f};

    // Standing lean — 1.5s progressive blend for smooth easing into lean position.
    // Collision-limited via updateLean().
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

    // Stride distance:
    //   vel < 5.0:  footstep_dist = 2.5
    //   vel > 15.0: footstep_dist = 4.0  (hard cap, NOT the formula's 5.5)
    //   else:       footstep_dist = 2.5 + 3.0 * ((vel - 5.0) / 10.0)
    // Note: the formula reaches 5.5 at vel=15, but the cap overrides to 4.0
    // for vel > 15.0.
    static constexpr float STRIDE_DIST_BASE   = 2.5f;   // base footstep distance
    static constexpr float STRIDE_DIST_RANGE  = 3.0f;   // additional distance at max speed
    static constexpr float STRIDE_SPEED_LOW   = 5.0f;   // speed where stride distance starts growing
    static constexpr float STRIDE_SPEED_RANGE = 10.0f;  // velocity range (15.0 - 5.0)
    static constexpr float STRIDE_DIST_CAP    = 4.0f;   // hard cap at vel > 15.0

    // Leaning — visual-only camera offset, physics body stays in place.
    // Lean is purely cosmetic, no physics body movement.
    // Lean parameters — lean is driven through motion poses (POSE_LEAN_*/POSE_CLNLEAN_*).
    // The head spring provides natural easing (no separate blend timer needed).
    // Distances match the motion pose lateral offsets:
    //   Standing lean: 2.2 units lateral (POSE_LEAN_LEFT/RIGHT)
    //   Crouching lean: 1.7 units lateral + 2.0 units vertical drop (POSE_CLNLEAN_*)
    static constexpr float LEAN_DISTANCE        = 2.2f;   // standing lean lateral offset (world units)
    static constexpr float CROUCH_LEAN_DISTANCE = 1.7f;   // crouching lean lateral offset (world units)
    static constexpr float LEAN_TILT            = 0.087f;  // max camera roll (~5 degrees, radians)

    // ── Construction ──

    /// Create player physics with reference to collision geometry.
    /// The collision geometry must outlive this object.
    /// Optionally accepts a PhysicsTimestep preset (default: MODERN = 60Hz).
    explicit PlayerPhysics(const CollisionGeometry &collision,
                           const PhysicsTimestep &ts = MODERN)
        : mCollision(collision), mTimestep(ts) {}

    ~PlayerPhysics() { stopLog(); }

    /// Change the active physics timestep preset at runtime.
    void setTimestep(const PhysicsTimestep &ts) { mTimestep = ts; }

    /// Get the active physics timestep configuration.
    const PhysicsTimestep& getTimestep() const { return mTimestep; }

    /// Get the physics update rate in Hz (= 1 / fixedDt).
    float getPhysicsHz() const { return 1.0f / mTimestep.fixedDt; }

    // ── Diagnostic logging ──
    // Per-timestep CSV logging for debugging head bob, stride triggering, and
    // spring dynamics. Writes one row per fixedStep() call with full physics state.
    // Enabled by startLog()/stopLog(). The renderer calls setCameraPitch() each
    // frame so the log includes the full camera orientation.

    /// Start writing per-timestep diagnostic log to the given file path.
    /// Creates (or truncates) the file and writes a CSV header.
    inline void startLog(const char *path) {
        stopLog();  // close any previous log
        mLogFile = std::fopen(path, "w");
        if (!mLogFile) {
            std::fprintf(stderr, "[PlayerPhysics] Failed to open log: %s\n", path);
            return;
        }
        std::fprintf(mLogFile,
            "simTime,dt,mode,sneaking,running,"
            "posX,posY,posZ,velX,velY,velZ,hSpeed,"
            "eyeX,eyeY,eyeZ,yaw,camPitch,"
            "springPosX,springPosY,springPosZ,"
            "springVelX,springVelY,springVelZ,"
            "poseTargetX,poseTargetY,poseTargetZ,"
            "poseStartX,poseStartY,poseStartZ,"
            "poseTimer,poseDur,poseHolding,"
            "strideDist,strideIsLeft,leanDir,leanAmount,"
            "cell,inputFwd,inputRight\n");
        std::fflush(mLogFile);
        std::fprintf(stderr, "[PlayerPhysics] Logging to: %s\n", path);
    }

    /// Stop diagnostic logging and close the file.
    inline void stopLog() {
        if (mLogFile) {
            std::fclose(mLogFile);
            mLogFile = nullptr;
            std::fprintf(stderr, "[PlayerPhysics] Log stopped\n");
        }
    }

    /// Is logging currently active?
    bool isLogging() const { return mLogFile != nullptr; }

    /// Set the camera pitch from the renderer (for logging — physics doesn't own pitch).
    void setCameraPitch(float pitch) { mCamPitch = pitch; }

    /// Get string name for the current player mode (for logging/display).
    static const char* modeName(PlayerMode mode) {
        switch (mode) {
            case PlayerMode::Stand:     return "Stand";
            case PlayerMode::Crouch:    return "Crouch";
            case PlayerMode::Swim:      return "Swim";
            case PlayerMode::Climb:     return "Climb";
            case PlayerMode::BodyCarry: return "BodyCarry";
            case PlayerMode::Slide:     return "Slide";
            case PlayerMode::Jump:      return "Jump";
            case PlayerMode::Dead:      return "Dead";
            default:                    return "Unknown";
        }
    }

    // ── Main simulation ──

    /// Advance the player simulation by dt seconds.
    /// Internally steps in fixedDt increments. Performs gravity, movement
    /// integration, collision response, and ground detection.
    /// contactCb may be empty if no contact notifications are needed.
    inline void step(float dt, const ContactCallback &contactCb) {
        mTimeAccum += dt;

        // Validate timestep config — recover from invalid values
        if (mTimestep.fixedDt <= 0.0f || mTimestep.fixedDt > 1.0f) {
            mTimestep = MODERN;
        }

        // Fixed timestep accumulation — step at configured Hz.
        // Before stepping, snapshot eye position for render interpolation.
        // After the loop, compute interpolation alpha from accumulator remainder.
        int stepCount = 0;
        while (mTimeAccum >= mTimestep.fixedDt) {
            // Snapshot previous eye state BEFORE each physics step so we always
            // have the state from one step behind the current state.
            mPrevEyePos   = computeRawEyePos();
            mPrevLeanTilt = computeRawLeanTilt();

            mTimeAccum -= mTimestep.fixedDt;
            fixedStep(contactCb);
            ++stepCount;
            // Safety: cap at 10 steps per frame to prevent spiral of death
            if (stepCount >= 10) {
                mTimeAccum = 0.0f;
                break;
            }
        }

        // Compute render interpolation alpha from accumulator remainder.
        // alpha=0 means "show previous state", alpha=1 means "show current state".
        // This MUST run every frame, not just when a physics step occurred — on
        // frames between physics steps the accumulator still grows, and alpha must
        // reflect progress through the interval so the camera moves smoothly.
        // At 12.5Hz with 60fps, ~4 of 5 frames have no physics step; without
        // unconditional alpha update the camera would freeze between steps.
        mInterpAlpha = mTimeAccum / mTimestep.fixedDt;
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

    /// Eye position — interpolated between previous and current physics states.
    /// Uses the accumulator remainder from step() to smoothly blend between the
    /// last two computed physics positions. This is essential for low-Hz presets
    /// (12.5Hz vintage) where the physics timestep is much coarser than the
    /// display refresh rate. Without interpolation, the camera visibly jumps
    /// between physics positions.
    ///
    /// Uses "Fix Your Timestep" interpolation with frame fraction.
    inline Vector3 getEyePosition() const {
        Vector3 current = computeRawEyePos();
        return glm::mix(mPrevEyePos, current, mInterpAlpha);
    }

    /// Camera roll/bank angle from leaning (radians), interpolated.
    /// Positive = tilting right, negative = tilting left.
    /// Proportional to the collision-limited lean extent (mLeanAmount),
    /// which is derived from the spring's lateral displacement each frame.
    inline float getLeanTilt() const {
        float current = computeRawLeanTilt();
        return mPrevLeanTilt + (current - mPrevLeanTilt) * mInterpAlpha;
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

        // Reset spring state — teleporting should not carry old spring velocity
        // into the new position, which would cause oscillation on arrival.
        mSpringPos = Vector3(0.0f);
        mSpringVel = Vector3(0.0f);
        mPoseCurrent = Vector3(0.0f);

        // Initialize interpolation state to current position so there's no
        // lerp glitch on the first frame after a teleport/spawn.
        mPrevEyePos   = computeRawEyePos();
        mPrevLeanTilt = computeRawLeanTilt();
        mInterpAlpha  = 1.0f;
    }

    /// Set gravity magnitude (units/sec²). Default is GRAVITY.
    void setGravityMagnitude(float g) { mGravityMag = g; }

private:
    // ── Internal simulation steps ──

    /// Single fixed-timestep physics step
    inline void fixedStep(const ContactCallback &contactCb) {
        // Advance simulation time (for landing throttle, animation timing, etc.)
        mSimTime += mTimestep.fixedDt;

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
                // Reset stride bob — locks motion to POSE_NORMAL during jump mode.
                // DON'T override an active lean.
                if (mLeanDir == 0)
                    activatePose(POSE_NORMAL);
                mStrideDist = 0.0f;
            }
        }
        mJumpRequested = false;

        // 2. Handle mode transitions (crouch, swim, etc.)
        if (!mMotionDisabled)
            updateModeTransitions();

        // 3. Update motion pose system (stride targets, progressive blend).
        updateMotionPose();

        // 4. Apply gravity (if airborne — always, even when disabled)
        applyGravity();

        // 5. Apply horizontal movement from input (skip when disabled)
        if (!mMotionDisabled)
            applyMovement();

        // 6. Update head spring — runs BEFORE collision
        updateHeadSpring();

        // 7. Pre-constrain velocity against known contact surfaces from the
        //    previous frame. This removes velocity going into walls BEFORE
        //    position integration, so the player slides along surfaces instead
        //    of moving into them and being pushed back out.
        constrainVelocity();

        // 8. Integrate position
        integrate();

        // 9. Update cell index BEFORE collision — if integration moved
        //    the player through a portal, collision must test against
        //    the new cell's walls, not the old cell's.
        updateCell();

        // 10. Resolve collisions via 5-sphere constraint projection.
        //     Position correction for any remaining penetrations (new contacts,
        //     cell transitions). Also refreshes mLastContacts for the next
        //     frame's constrainVelocity() call.
        resolveCollisions(contactCb);

        // 11. Detect ground and update movement state
        detectGround();

        // 12. Update cell index again (collision may have shifted position)
        // Relies purely on gravity + collision + constraint response to keep
        // the player grounded. An explicit snap was causing jitter (collision
        // pushes up, snap pulls down) and fighting with uphill movement.
        updateCell();

        // 13. Check for mantle opportunity when airborne and pressing forward
        if (mCurrentMode == PlayerMode::Jump && mInputForward > 0.1f && !mMotionDisabled) {
            checkMantle();
        }

        // 14. Update lean (visual-only lateral camera offset with collision)
        updateLean();

        // 15. Write diagnostic log row (if logging enabled)
        writeLogRow();
    }

    /// Apply gravity to velocity (if not on ground)
    inline void applyGravity() {
        if (!isOnGround()) {
            // Z-up: gravity pulls downward
            mVelocity.z -= mGravityMag * mTimestep.fixedDt;
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

    /// Apply movement control — exponential velocity convergence impulse system:
    ///
    ///   contactFriction = FRICTION_FACTOR * groundNormal.z * gravityMag
    ///   alpha = CONTROL_MULTIPLIER * contactFriction * dt / VELOCITY_RATE
    ///   vel += (desired - vel) * alpha
    ///
    /// This creates exponential approach to desired velocity with tau ~0.095s on
    /// flat ground. When desired=0 (no input), velocity decays exponentially
    /// toward zero — no separate deceleration constant needed.
    ///
    inline void applyMovement() {
        Vector3 desired = computeDesiredVelocity();
        const float dt = mTimestep.fixedDt;

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
                float slopeFactor = (mGroundNormal.z - GROUND_NORMAL_MIN)
                                  / (SLIDE_THRESHOLD - GROUND_NORMAL_MIN);
                slopeFactor = std::max(0.0f, std::min(1.0f, slopeFactor));
                desired *= slopeFactor;
            }

            // Compute convergence rate from ground friction.
            // contactFriction = FRICTION_FACTOR * groundNormal.z * gravityMag
            // On flat ground: 0.03 * 1.0 * 32.0 = 0.96
            float contactFriction = FRICTION_FACTOR * mGroundNormal.z * mGravityMag;
            float alpha = CONTROL_MULTIPLIER * contactFriction * dt / VELOCITY_RATE;
            alpha = std::min(alpha, 1.0f);  // cap at 1.0 (instant convergence)

            // Apply convergence to horizontal velocity.
            // Preserves desired.z for slope projection (SET, not converge, to avoid
            // accumulation on slopes that launches player off ramps).
            float prevZ = mVelocity.z;
            Vector3 hVel(mVelocity.x, mVelocity.y, 0.0f);
            Vector3 hDesired(desired.x, desired.y, 0.0f);

            // Exponential convergence: vel += (desired - vel) * alpha
            Vector3 hNew = hVel + (hDesired - hVel) * alpha;
            mVelocity.x = hNew.x;
            mVelocity.y = hNew.y;

            // Z component: set from slope projection (not converged)
            mVelocity.z = desired.z != 0.0f ? desired.z : prevZ;
        } else {
            // Airborne: should have zero air control (friction=0).
            Vector3 airDesired(desired.x, desired.y, 0.0f);
            Vector3 hVel(mVelocity.x, mVelocity.y, 0.0f);

            // Use flat-ground friction for air control base rate
            float airAlpha = CONTROL_MULTIPLIER * (FRICTION_FACTOR * 1.0f * mGravityMag)
                           * AIR_CONTROL_FRAC * dt / VELOCITY_RATE;
            airAlpha = std::min(airAlpha, 1.0f);

            Vector3 hNew = hVel + (airDesired - hVel) * airAlpha;
            mVelocity.x = hNew.x;
            mVelocity.y = hNew.y;
            // Z unchanged — gravity handles it
        }
    }

    /// Integrate position from velocity
    inline void integrate() {
        mPosition += mVelocity * mTimestep.fixedDt;
    }

    /// Resolve collisions using the 5-sphere model.
    /// For each iteration, tests all 5 submodel positions against cell polygons,
    /// accumulates contacts, and pushes the body center out of penetrations.
    /// HEAD and BODY are real spheres (r=1.2) for wall/ceiling collision.
    /// Pre-constrain velocity against known contact surfaces.
    ///
    /// Uses contacts from the PREVIOUS frame's collision resolution to remove
    /// velocity components going into known walls/floor. This prevents the body
    /// from moving into surfaces and needing to be pushed back out, reducing
    /// jitter.
    ///
    /// The position-correction pass in resolveCollisions() still runs after
    /// integration to handle new contacts and penetrations.
    inline void constrainVelocity() {
        if (mLastContacts.empty()) return;

        // Collect unique constraint normals from previous frame's contacts
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

        // Apply constraint-based velocity removal
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
                if (glm::dot(projected, origVel) < 0.0f)
                    mVelocity = Vector3(0.0f);
                else
                    mVelocity = projected;
            } else {
                mVelocity = Vector3(0.0f);
            }
        }
    }

    /// Resolve collisions using the 5-sphere model.
    /// For each iteration, tests all 5 submodel positions against cell polygons,
    /// accumulates contacts, and pushes the body center out of penetrations.
    /// HEAD and BODY are real spheres (r=1.2) for wall/ceiling collision.
    /// SHIN, KNEE, FOOT are point detectors (r=0) — they sense floor contacts
    /// and generate upward normals that keep the player above the ground.
    inline void resolveCollisions(const ContactCallback &contactCb) {
        Vector3 origPos = mPosition;
        int32_t origCell = mCellIdx;

        // Compute current sphere offsets — during crouch, HEAD and BODY get
        // additional downward offsets.
        bool crouching = isCrouching();

        mLastContacts.clear();

        for (int iter = 0; iter < mTimestep.collisionIters; ++iter) {
            if (mCellIdx < 0)
                break;

            std::vector<SphereContact> iterContacts;

            for (int s = 0; s < NUM_SPHERES; ++s) {
                float sphereR = SPHERE_RADII[s];
                float offsetZ = SPHERE_OFFSETS[s] + (crouching ? CROUCH_OFFSETS[s] : 0.0f);
                Vector3 sphereCenter = mPosition + Vector3(0.0f, 0.0f, offsetZ);

                // Test against the body center's cell
                auto contacts = mCollision.sphereVsCellPolygons(
                    sphereCenter, sphereR, mCellIdx);

                // Test the submodel's own cell if it differs from body center's cell
                // (handles straddling portal boundaries, e.g. feet in one cell, head in another)
                int32_t sphereCell = mCollision.findCell(sphereCenter);
                if (sphereCell >= 0 && sphereCell != mCellIdx) {
                    auto adj = mCollision.sphereVsCellPolygons(
                        sphereCenter, sphereR, sphereCell);
                    contacts.insert(contacts.end(), adj.begin(), adj.end());
                }

                // Test adjacent cells via portals from the body center's cell.
                // For point detectors (radius=0), the portal distance check uses
                // the POINT_MAX_PENETRATION threshold to avoid missing floor contacts
                // in adjacent cells near portal boundaries.
                constexpr float POINT_PORTAL_REACH = 0.5f;
                float portalReach = (sphereR > 0.001f) ? sphereR : POINT_PORTAL_REACH;
                const auto &cell = mCollision.getWR().cells[mCellIdx];
                int numSolid = cell.numPolygons - cell.numPortals;
                for (int pi = numSolid; pi < cell.numPolygons; ++pi) {
                    const auto &poly = cell.polygons[pi];
                    if (poly.count < 3) continue;
                    const auto &plane = cell.planes[poly.plane];
                    float dist = plane.getDistance(sphereCenter);
                    if (dist < portalReach) {
                        int32_t tgtCell = static_cast<int32_t>(poly.tgtCell);
                        if (tgtCell >= 0 && tgtCell != mCellIdx && tgtCell != sphereCell
                            && tgtCell < static_cast<int32_t>(mCollision.getWR().numCells)) {
                            auto adj = mCollision.sphereVsCellPolygons(
                                sphereCenter, sphereR, tgtCell);
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

            // Push may have moved through a portal — update cell.
            // Use portal-based tracking (not brute-force) to avoid jumping to
            // an overlapping but topologically-unconnected cell mid-resolution.
            {
                bool cellUpdated = false;
                // Check if still in current cell
                const auto &curC = mCollision.getWR().cells[mCellIdx];
                bool inCur = true;
                for (const auto &pl : curC.planes) {
                    if (pl.getDistance(mPosition) < -0.1f) { inCur = false; break; }
                }
                if (inCur) {
                    cellUpdated = true;
                } else {
                    // Check portal-connected cells
                    int nSolid = curC.numPolygons - curC.numPortals;
                    for (int ppi = nSolid; ppi < curC.numPolygons; ++ppi) {
                        int32_t tc = static_cast<int32_t>(curC.polygons[ppi].tgtCell);
                        if (tc < 0 || tc >= static_cast<int32_t>(mCollision.getWR().numCells))
                            continue;
                        const auto &tgtC = mCollision.getWR().cells[tc];
                        bool inTgt = true;
                        for (const auto &pl : tgtC.planes) {
                            if (pl.getDistance(mPosition) < -0.1f) { inTgt = false; break; }
                        }
                        if (inTgt) {
                            mCellIdx = tc;
                            cellUpdated = true;
                            break;
                        }
                    }
                }
                if (!cellUpdated) {
                    // Brute-force fallback
                    int32_t newCell = mCollision.findCell(mPosition);
                    if (newCell >= 0) {
                        mCellIdx = newCell;
                    } else {
                        // Pushed outside all cells.
                        mPosition = origPos;
                        mCellIdx = origCell;
                        return;
                    }
                }
            }
        }

        // Final validation — body center must still be in a valid cell
        if (mCollision.findCell(mPosition) < 0) {
            mPosition = origPos;
            mCellIdx = origCell;
        }

        // Fire contact callbacks for downstream consumers (audio, AI, scripts).
        // Contact point is approximate — we don't track which submodel generated
        // each contact, so we use SPHERE_RADIUS (HEAD/BODY radius) as the offset.
        // For point detector contacts, this places the event slightly above the
        // actual contact, which is acceptable for audio footstep triggers.
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
        // probe slightly below the FOOT point detector to detect ground. This
        // prevents briefly entering Jump mode when walking over small bumps or
        // down slopes where the collision pass didn't generate contacts. The
        // probe only DETECTS ground (updates state), it doesn't move the player
        // — gravity and collision handle the actual positioning.
        // Skip when velocity.z > 0.5 to avoid cancelling a jump (the probe
        // would find the floor just left).
        if (!onGround && mCellIdx >= 0 && mVelocity.z <= 0.5f) {
            // FOOT stays at its standing offset during crouch (no crouch drop for legs)
            float footOffset = FOOT_OFFSET_Z;
            Vector3 footCenter = mPosition + Vector3(0.0f, 0.0f, footOffset);
            Vector3 probeNormal;
            // Small probe distance — just enough to catch one frame of gravity drop
            // at 60Hz: 0.5 * g * dt² = 0.5 * 32 * (1/60)² ≈ 0.004 units.
            // Use 0.1 units for a comfortable margin.
            // FOOT is a point detector (radius=0), so groundTest uses point-test mode.
            constexpr float GROUND_PROBE_DIST = 0.1f;
            constexpr float FOOT_RADIUS = SPHERE_RADII[4]; // 0.0 (point detector)
            if (mCollision.groundTest(footCenter, FOOT_RADIUS, mCellIdx,
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

                // Return to crouch or stand based on input.
                // Activate the appropriate rest pose so the head spring drives
                // toward the correct height (POSE_CROUCH = -2.02 or POSE_NORMAL = 0).
                mCurrentMode = mWantsCrouch ? PlayerMode::Crouch : PlayerMode::Stand;
                if (!mLandingActive && mLeanDir == 0) {
                    activatePose(*getModeMotion(mCurrentMode).restPose);
                }
            }
            // If already in a ground mode, stay in it (crouch transitions
            // are handled by updateModeTransitions)
        } else {
            if (isOnGround()) {
                // Just walked off an edge — start falling.
                // Reset stride bob (no stride while airborne), but preserve
                // lean.
                mCurrentMode = PlayerMode::Jump;
                if (mLeanDir == 0)
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
        // Instead of binary-search halving (which can snap to zero when near a
        // wall), use the contact penetration depth to compute the exact maximum
        // safe lean distance. The lean holds at the wall surface rather than
        // snapping back to standing.
        if (std::fabs(mLeanAmount) > 0.01f && mCellIdx >= 0) {
            float sinYaw = std::sin(mYaw);
            float cosYaw = std::cos(mYaw);
            // HEAD collision position for lean: standing offset + crouch drop (if any)
            float headOffset = HEAD_OFFSET_Z + (isCrouching() ? CROUCH_HEAD_DROP : 0.0f);
            float leanBaseZ = headOffset + mSpringPos.z;
            Vector3 leanedHead = mPosition + Vector3(0.0f, 0.0f, leanBaseZ);
            leanedHead.x += sinYaw * mLeanAmount;
            leanedHead.y -= cosYaw * mLeanAmount;

            // Test HEAD sphere (r=1.2) at the leaned position
            auto contacts = mCollision.sphereVsCellPolygons(
                leanedHead, SPHERE_RADIUS, mCellIdx);
            if (!contacts.empty()) {
                // Compute the lean direction in world space (unit vector pointing
                // from the player's center toward the leaned head position).
                float leanSign = (mLeanAmount > 0.0f) ? 1.0f : -1.0f;
                Vector3 leanDir(sinYaw * leanSign, -cosYaw * leanSign, 0.0f);

                // Find the largest pushback required along the lean direction
                // across all contacts. This tells us exactly how far to pull
                // the head back to clear all walls.
                float maxPushback = 0.0f;
                for (const auto &c : contacts) {
                    // Project the contact push (normal * penetration) onto the
                    // negative lean direction — how much of the wall push opposes
                    // the lean.
                    float pushInLeanDir = glm::dot(c.normal * c.penetration, -leanDir);
                    if (pushInLeanDir > maxPushback)
                        maxPushback = pushInLeanDir;
                }

                // Reduce lean by the pushback + small margin to prevent jitter.
                // The lean holds at exactly the wall distance rather than zeroing.
                constexpr float LEAN_WALL_MARGIN = 0.05f;
                float absLean = std::fabs(mLeanAmount);
                absLean = std::max(0.0f, absLean - maxPushback - LEAN_WALL_MARGIN);
                mLeanAmount = absLean * leanSign;
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

        // HEAD position for mantle check — apply crouch drop if crouching
        float headZ = HEAD_OFFSET_Z + (isCrouching() ? CROUCH_HEAD_DROP : 0.0f);
        Vector3 headPos = mPosition + Vector3(0.0f, 0.0f, headZ);

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

        // Found a mantleable surface — compute target position.
        // Place body center above the ledge so the FOOT point detector sits on
        // the surface. FOOT stays at its standing offset during crouch (no crouch
        // drop for legs), so use FOOT_OFFSET_Z directly.
        mMantleTarget = hit.point;
        mMantleTarget.z -= FOOT_OFFSET_Z;  // body center above ledge (FOOT_OFFSET_Z is negative)

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

        mMantleTimer += mTimestep.fixedDt;

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
                    // Move toward target via exponential convergence (rate-independent).
                    // 4.46/sec converges ~90% in 0.5s — quick enough for the mantle animation.
                    static constexpr float MANTLE_RISE_RATE = 4.46f;
                    float blend = 1.0f - std::exp(-MANTLE_RISE_RATE * mTimestep.fixedDt);
                    float step = dz * blend;
                    float maxStep = 20.0f * mTimestep.fixedDt;  // velocity cap equivalent
                    step = std::max(-maxStep, std::min(maxStep, step));
                    mPosition.z += step;
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
                    float frac = mTimestep.fixedDt / std::max(0.01f, MANTLE_FWD_TIME - mMantleTimer + mTimestep.fixedDt);
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
            mVelocity.z -= mGravityMag * mTimestep.fixedDt;
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

    /// Update cell index from current position.
    ///
    /// Uses portal-graph-based cell tracking rather than brute-force search.
    /// Tracks cell transitions through portal plane crossings, which avoids
    /// teleporting to geometrically-overlapping but topologically-unconnected
    /// cells (e.g., water cells that overlap air cells).
    ///
    /// Algorithm:
    ///   1. If still inside current cell, stay (most common case)
    ///   2. Check portal-connected cells (one hop from current cell)
    ///   3. Fallback: brute-force search (for spawn, teleport, etc.)
    inline void updateCell() {
        const auto &wr = mCollision.getWR();

        // No current cell — brute-force search (initial placement, teleport)
        if (mCellIdx < 0 || mCellIdx >= static_cast<int32_t>(wr.numCells)) {
            int32_t newCell = mCollision.findCell(mPosition);
            if (newCell >= 0)
                mCellIdx = newCell;
            return;
        }

        // 1. Check if still inside current cell (fast path — usual case)
        const auto &curCell = wr.cells[mCellIdx];
        bool stillInside = true;
        for (const auto &plane : curCell.planes) {
            if (plane.getDistance(mPosition) < -0.1f) {
                stillInside = false;
                break;
            }
        }
        if (stillInside)
            return;

        // 2. Check portal-connected cells — prefer topological neighbors.
        // Only transition to cells reachable via a portal from the current cell.
        // This prevents jumping to overlapping water/air cells or distant cells
        // whose bounding spheres happen to contain the player's position.
        int numSolid = curCell.numPolygons - curCell.numPortals;
        for (int pi = numSolid; pi < curCell.numPolygons; ++pi) {
            int32_t tgtCell = static_cast<int32_t>(curCell.polygons[pi].tgtCell);
            if (tgtCell < 0 || tgtCell >= static_cast<int32_t>(wr.numCells))
                continue;

            const auto &tgt = wr.cells[tgtCell];
            bool inside = true;
            for (const auto &plane : tgt.planes) {
                if (plane.getDistance(mPosition) < -0.1f) {
                    inside = false;
                    break;
                }
            }
            if (inside) {
                mCellIdx = tgtCell;
                return;
            }
        }

        // 3. Fallback: brute-force search (handles cases where portal graph
        // doesn't reach — shouldn't happen in normal gameplay but covers
        // edge cases like respawn or level geometry quirks).
        int32_t newCell = mCollision.findCell(mPosition);
        if (newCell >= 0) {
            mCellIdx = newCell;
        }
        // If still -1, keep old cell (player at boundary, will resolve next frame)
    }

    /// Update mode transitions — handles crouch, jump→ground, ground→jump, swim.
    /// Replaces the old updateModeTransitions(). Body center shifts and spring compensation
    /// for crouch/uncrouch are preserved — just driven by mode transitions now.
    inline void updateModeTransitions() {
        // Crouch uses per-submodel offsets (HEAD -2.02, BODY -1.0)
        // The body center stays in place — no
        // body center drop needed. The motion pose system (POSE_CROUCH) drives
        // mSpringPos.z toward -2.02, creating a smooth visual eye drop.
        // The collision system applies CROUCH_OFFSETS[] to physically lower HEAD
        // and BODY spheres for ceiling clearance.

        // ── Crouch transitions (only on ground modes) ──
        if (isOnGround()) {
            if (mWantsCrouch && !isCrouching()) {
                // Start crouching — body center stays in place.
                // Explicitly activate POSE_CROUCH so the head spring drives the
                // eye downward smoothly. Without this, the pose system's idle
                // check (glm::length(mPoseEnd) > 0.01f) would fail when
                // transitioning from POSE_NORMAL (all zeros) and the camera
                // would never actually lower.
                mCurrentMode = PlayerMode::Crouch;
                if (mLeanDir == 0)
                    activatePose(POSE_CROUCH);
            } else if (!mWantsCrouch && isCrouching()) {
                // Try to un-crouch — check if there's room above.
                // Test HEAD sphere at standing height (no crouch offset applied)
                Vector3 standingHeadPos = mPosition + Vector3(0.0f, 0.0f, HEAD_OFFSET_Z);
                auto headContacts = mCollision.sphereVsCellPolygons(
                    standingHeadPos, SPHERE_RADIUS, mCellIdx);

                // Only un-crouch if head has no collisions at standing height
                if (headContacts.empty()) {
                    mCurrentMode = PlayerMode::Stand;
                    // Activate standing rest pose so the spring drives eye back up
                    if (mLeanDir == 0)
                        activatePose(POSE_NORMAL);
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
    /// sets new target, and begins progressive blend toward it.
    /// The head spring provides organic smoothing on top of the blend curve.
    ///
    /// Same-motion guard: if the new target matches the current target and the
    /// pose is already blending/holding, skip re-activation.
    inline void activatePose(const MotionPoseData &pose) {
        Vector3 newTarget(pose.fwd, pose.lat, pose.vert);

        // Skip if already targeting the same offset (same-motion guard).
        // Exception: if the pose just finished holding (poseReady), allow
        // re-activation so stride→rest→stride cycling works correctly.
        if (glm::length(mPoseEnd - newTarget) < 0.001f && !mPoseHolding) {
            return;  // already blending toward this target
        }

        mPoseStart = mPoseCurrent;  // capture current position as blend origin
        mPoseEnd = newTarget;
        mPoseDuration = pose.duration;
        mPoseHoldTime = pose.holdTime;
        mPoseTimer = 0.0f;
        mPoseHolding = false;

        // NOTE: Do NOT clear mMotionQueue here.
        // Clearing here would break compound motions (weapon swing → recovery)
        // if a stride or rest interrupt fires mid-sequence.
    }

    /// Activate the next stride pose (alternates left/right).
    ///
    /// Blends toward stride target over 0.6s using progressive interpolation. The head
    /// spring provides organic dynamics (overshoot, lag) on top of the blend.
    ///
    inline void activateStride() {
        auto motionCfg = getModeMotion(mCurrentMode);
        const MotionPoseData *stride = mStrideIsLeft
            ? motionCfg.strideLeft : motionCfg.strideRight;

        // Activate stride directly — captures current pose as blend start,
        // begins progressive blend toward stride target. Each new stride
        // interrupts the previous blend naturally via activatePose().
        activatePose(*stride);

        mStrideIsLeft = !mStrideIsLeft;
        mLastStrideSimTime = mSimTime;
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
        // low speed
        if (hSpeed < STRIDE_SPEED_LOW)
            return STRIDE_DIST_BASE;
        // max speed
        if (hSpeed > STRIDE_SPEED_LOW + STRIDE_SPEED_RANGE)
            return STRIDE_DIST_CAP;
        // middle range of speeds
        return STRIDE_DIST_BASE + STRIDE_DIST_RANGE
             * ((hSpeed - STRIDE_SPEED_LOW) / STRIDE_SPEED_RANGE);
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
        mPoseTimer += mTimestep.fixedDt;

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
            // Instant pose (dur=0) — snap to target, start hold.
            // Used for landing impacts and weapon swings where the spring should
            // handle all smoothing from an instant displacement.
            mPoseCurrent = mPoseEnd;
            mPoseHolding = true;
            mPoseTimer = 0.0f;
        } else {
            // Progressive blend
            // Formula: curOffset += (targOffset - curOffset) * (timeActive / timeDuration)
            // This is NOT linear interpolation — it's a cumulative blend where each
            // step applies a fraction of the remaining distance. The fraction grows
            // with time (timeActive/timeDuration increases each step), creating an
            // accelerating approach curve that reaches the target at t=duration.
            // At the original 12.5Hz rate with 0.6s duration, this produces ~7 blend
            // steps with progressively larger increments.
            if (mPoseTimer >= mPoseDuration) {
                mPoseCurrent = mPoseEnd;
                mPoseHolding = true;
                mPoseTimer = 0.0f;
            } else {
                float blendFrac = mPoseTimer / mPoseDuration;
                mPoseCurrent += (mPoseEnd - mPoseCurrent) * blendFrac;
            }
        }

        // ── Distance-based stride accumulation ──
        // Accumulate foot travel distance when moving above tracking threshold.
        // This drives stride triggering — footstep animation is distance-based,
        // not time-based, foot submodel tracking.
        bool strideTriggered = false;
        if (isOnGround() && hSpeed > 1.0f) {
            mStrideDist += hSpeed * mTimestep.fixedDt;
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
        // Pop BEFORE activatePose — activatePose() clears the entire queue
        // internally, so pop_back after clear would be UB on an empty vector.
        if (!mMotionQueue.empty() && poseReady) {
            const MotionPoseData *nextQueued = mMotionQueue.back();
            mMotionQueue.pop_back();
            activatePose(*nextQueued);
        }
        // Landing bump takes priority over normal strides
        else if (mLandingActive) {
            if (poseReady) {
                mLandingActive = false;
                // After landing, resume walking or return to idle.
                // If leaning, re-activate the lean pose — the original gates
                // rest pose activation on !IsLeaning(), so lean survives landing.
                if (isWalking) {
                    activateStride();
                    mStrideDist = 0.0f;
                } else if (isLeaning) {
                    // Re-activate lean pose so spring target stays at lean offset
                    activatePose(isCrouching()
                        ? (mLeanDir < 0 ? POSE_CLNLEAN_LEFT : POSE_CLNLEAN_RIGHT)
                        : (mLeanDir < 0 ? POSE_LEAN_LEFT : POSE_LEAN_RIGHT));
                } else {
                    activatePose(restPose);
                }
            }
        }
        // Walking: trigger strides based on foot travel distance.
        // Between strides, a rest-condition interrupt fires ~100ms after
        // each stride activation. This interrupts the stride's progressive
        // blend at ~30% completion, producing the characteristic subtle
        // walking bob. Without this, strides chain directly and the full
        // stride offset is reached, creating ~3× too much head bob.
        else if (isWalking) {
            if (!mWasWalking) {
                // Just started walking — activate first stride immediately
                activateStride();
                mStrideDist = 0.0f;
            } else if (strideTriggered) {
                // Foot traveled enough distance — activate next stride.
                activateStride();
            } else if ((mSimTime - mLastStrideSimTime) > 0.1f &&
                       mStrideDist > computeFootstepDist(hSpeed) * 0.5f) {
                // Rest-condition interrupt: fires activatePose(restPose) when >100ms
                // has elapsed since the last
                // stride event AND the foot has traveled past the half-footstep
                // distance. This creates a stride→rest→stride→rest oscillation
                // pattern where each stride is interrupted at ~30% blend, keeping
                // effective head bob amplitude to roughly 1/3 of the full stride
                // target offset.
                activatePose(restPose);
            }
        }
        // Not walking: return to mode's rest pose when current pose completes.
        // Exception: while leaning (mLeanDir != 0), hold at the lean target
        // indefinitely — the lean pose should persist until the key is released.
        // setLeanDirection(0) explicitly activates the rest pose on key release.
        else {
            if (mWasWalking && !mLandingActive && !isLeaning) {
                // Just stopped walking — immediately interrupt any in-progress
                // stride blend with the rest pose. Fires
                // activatePose(restPose) immediately when velocity drops below
                // the stride threshold (the velocity_mag < 1.5 OR-clause in
                // the rest-condition check bypasses the distance requirement).
                // Same-motion guard in activatePose() prevents re-triggering
                // on subsequent idle frames.
                activatePose(restPose);
            } else if (poseReady && !isLeaning) {
                // Idle — check if current pose differs from mode's rest pose.
                // Activates restPose when: (a) a non-zero pose completed (e.g.
                // landing bump returning to idle), or (b) the mode changed and
                // the rest pose is different from the current target (e.g.
                // switching from Stand (vert=0) to Crouch (vert=-2.02)).
                Vector3 restTarget(restPose.fwd, restPose.lat, restPose.vert);
                if (glm::length(mPoseEnd - restTarget) > 0.01f) {
                    activatePose(restPose);
                }
            }
        }

        mWasWalking = isWalking;
    }

    /// Compute the raw (un-interpolated) eye position from current physics state.
    /// This is the body center + head offset + eye-above-head + spring displacement
    /// (in player-local coords rotated by yaw) + collision-limited lean.
    /// Called by getEyePosition() for interpolation and by step() to snapshot state.
    inline Vector3 computeRawEyePos() const {
        // Head Z offset — always HEAD_OFFSET_Z. During crouch, the motion pose system
        // (POSE_CROUCH vert=-2.02) drives mSpringPos.z down, lowering the eye naturally.
        // No body center shift or offset scaling needed.
        Vector3 eye = mPosition + Vector3(0.0f, 0.0f, HEAD_OFFSET_Z + EYE_ABOVE_HEAD + mSpringPos.z);

        float sinYaw = std::sin(mYaw);
        float cosYaw = std::cos(mYaw);

        // Spring forward displacement (mSpringPos.x = forward axis in player-local coords)
        if (std::fabs(mSpringPos.x) > 0.001f) {
            eye.x += cosYaw * mSpringPos.x;
            eye.y += sinYaw * mSpringPos.x;
        }

        // Collision-limited lateral offset (combines stride sway + lean).
        // Both stride lateral sway and deliberate lean flow through mLeanAmount,
        // which is set from mSpringPos.y in updateLean() and collision-limited
        // for wall clipping. Camera roll tracks mLeanAmount proportionally via
        // computeRawLeanTilt() — stride sway produces ~0.2° roll (imperceptible).
        if (std::fabs(mLeanAmount) > 0.001f) {
            eye.x += sinYaw * mLeanAmount;
            eye.y -= cosYaw * mLeanAmount;
        }

        return eye;
    }

    /// Compute the raw (un-interpolated) lean tilt from current physics state.
    /// Returns camera roll angle proportional to collision-limited lean extent.
    /// Camera roll is derived
    /// from the HEAD submodel's lateral displacement (mLeanAmount), not gated
    /// on a lean-active flag. This means stride sway (±0.1 units) produces
    /// ~0.2° of roll — imperceptible.
    /// Lean (±2.2 units) produces the full ~5° roll, and on key release the
    /// spring smoothly decays mLeanAmount back to zero, giving smooth roll return.
    inline float computeRawLeanTilt() const {
        float maxDist = isCrouching() ? CROUCH_LEAN_DISTANCE : LEAN_DISTANCE;
        if (maxDist < 0.01f) return 0.0f;
        return (mLeanAmount / maxDist) * LEAN_TILT;
    }

    /// Exact analytical step for one axis of a damped harmonic oscillator.
    /// State: pos (position), vel (velocity). Target: equilibrium point.
    /// Uses the matrix exponential of the continuous-time ODE:
    ///   x'' + 2ζω₀x' + ω₀²x = 0
    /// This is exact — no Euler integration error. At any dt the trajectory
    /// is identical to the continuous ODE solution. Rate-independent by construction.
    ///
    /// Handles both underdamped (ζ<1, sin/cos) and overdamped (ζ>1, sinh/cosh).
    /// CURRENTLY UNUSED — kept for future rate-independent spring implementation.
    /// Once the direct discrete formula is verified at 12.5Hz, this can replace
    /// it with properly derived ω₀/ζ values for rate-independent behavior.
    static inline void stepSpringAxis(float &pos, float &vel, float target,
                                      float omega0, float zeta, float dt) {
        float d = pos - target;     // displacement from equilibrium
        float v = vel;              // current velocity

        float sigma = zeta * omega0;  // decay rate
        float e = std::exp(-sigma * dt);

        if (zeta < 1.0f) {
            // Underdamped: oscillatory solution with sin/cos
            float wd = omega0 * std::sqrt(1.0f - zeta * zeta);
            float cosWd = std::cos(wd * dt);
            float sinWd = std::sin(wd * dt);
            float sigmaOverWd = sigma / wd;
            float omega0SqOverWd = (omega0 * omega0) / wd;

            pos = target + e * (d * (cosWd + sigmaOverWd * sinWd)
                              + v * (sinWd / wd));
            vel = e * (d * (-omega0SqOverWd * sinWd)
                     + v * (cosWd - sigmaOverWd * sinWd));
        } else {
            // Overdamped: non-oscillatory solution with sinh/cosh
            float wd = omega0 * std::sqrt(zeta * zeta - 1.0f);
            float chWd = std::cosh(wd * dt);
            float shWd = std::sinh(wd * dt);
            float sigmaOverWd = sigma / wd;
            float omega0SqOverWd = (omega0 * omega0) / wd;

            pos = target + e * (d * (chWd + sigmaOverWd * shWd)
                              + v * (shWd / wd));
            vel = e * (d * (-omega0SqOverWd * shWd)
                     + v * (chWd - sigmaOverWd * shWd));
        }
    }

    /// Update 3D head spring — direct discrete implementation - spring formula (dt-dependent):
    ///   springDt = min(dt, 0.05)               (clamps spring dt to 50ms)
    ///   tension_eff = BASE_TENSION / springDt   (higher rate → lower tension)
    ///   damping_eff = BASE_DAMPING + (1.0 - BASE_DAMPING) * springDt
    ///   vel = displacement * tension_eff        (target-tracking velocity)
    ///   vel.z *= Z_SCALE                        (Z-axis half-strength, hardcoded)
    ///   vel = vel + old_vel * damping_eff        (blend new with retained old)
    ///   pos += vel * dt                          (integrate with FULL dt, not clamped)
    ///
    /// The dt clamping means at 12.5Hz (dt=0.08), the spring uses springDt=0.05:
    ///   tension_eff = 0.6/0.05 = 12.0  (not 7.5)
    ///   damping_eff = 0.02 + 0.98*0.05 = 0.069  (not 0.098)
    /// This makes the spring 60% stiffer and 30% less damped than without clamping.
    /// Position integration still uses the full physics dt for correct displacement.
    ///
    /// Note: This formula is NOT rate-independent — behavior differs at 60Hz vs 12.5Hz.
    /// For now we use it at all rates; rate-independent analytical stepping can be
    /// added later once the behavior is confirmed correct at the original 12.5Hz rate.
    inline void updateHeadSpring() {
        const float dt = mTimestep.fixedDt;
        Vector3 target = mPoseCurrent;

        // Clamps the spring dt to 50ms max. This affects tension/damping
        // calculation but NOT position integration (which uses full dt).
        const float springDt = std::min(dt, HEAD_SPRING_MAX_DT);

        // Compute dt-dependent spring parameters with clamped dt
        float tensionEff = HEAD_SPRING_BASE_TENSION / springDt;
        float dampingEff = HEAD_SPRING_BASE_DAMPING
                         + (1.0f - HEAD_SPRING_BASE_DAMPING) * springDt;

        // Displacement from target → spring velocity (target-seeking term)
        Vector3 disp = target - mSpringPos;
        Vector3 newVel = disp * tensionEff;

        // Z-axis half-strength — makes vertical spring slower/overdamped.
        // This is what creates the dominant vertical head bob: the Z spring
        // is always in transit during walking because it approaches so slowly.
        newVel.z *= HEAD_SPRING_Z_SCALE;

        // Blend with previous velocity (damping_eff < 1 = mostly new, little old)
        // At 12.5Hz with clamping: dampingEff=0.069, so 93% new + 7% old
        mSpringVel = newVel + mSpringVel * dampingEff;

        // Velocity cap — 25.0
        float velMag = glm::length(mSpringVel);
        if (velMag > HEAD_SPRING_VEL_CAP) {
            mSpringVel *= HEAD_SPRING_VEL_CAP / velMag;
        }

        // Integrate position
        mSpringPos += mSpringVel * dt;

        // Clamp to sane range — prevents spring runaway on large target jumps.
        // ±4.0 accommodates crouch transition (2.02 units) plus overshoot.
        mSpringPos = glm::clamp(mSpringPos, Vector3(-4.0f), Vector3(4.0f));
    }

    // ── State ──

    const CollisionGeometry &mCollision;  // world collision geometry (not owned)
    PhysicsTimestep mTimestep = MODERN;  // active timestep configuration

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
    // Direct discrete spring using a dt-dependent formula.
    // Tracks motion pose targets in player-local coords {fwd, lat, vert}.
    // XY axes are near-critical; Z (with 0.5× tension) is overdamped.
    // The slow Z approach creates dominant vertical head bob during walking.
    Vector3 mSpringPos{0.0f};     // current spring position {fwd, lat, vert}
    Vector3 mSpringVel{0.0f};     // spring velocity (used by direct discrete formula)

    // ── Motion pose state ──
    // Stride-driven pose system using progressive blend + head spring.
    // A rest-condition interrupt fires ~100ms after each stride activation,
    // creating stride→rest→stride oscillation. Each stride is interrupted at
    // ~30% blend, limiting effective bob amplitude. The head spring adds
    // dynamic response on top (overshoot, lag).
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

    // Stride timing — last stride activation time for diagnostics/logging
    float mLastStrideSimTime   = 0.0f;  // simTime when last stride was activated

    // Motion queue — LIFO stack for chaining multi-pose sequences.
    // When a pose completes, the next is popped from the back.
    // Used for compound motions (weapon swing → recovery, mantle phases, etc.)
    std::vector<const MotionPoseData*> mMotionQueue;

    // Ground contact normal from last collision pass
    Vector3 mGroundNormal{0.0f, 0.0f, 1.0f};

    // ── Render interpolation state ──
    // "Fix Your Timestep" interpolation — smooth rendering between fixed physics steps.
    // Interpolates with frame fraction to blend
    // between previous and current physics states. At low Hz (12.5Hz vintage), without
    // interpolation the camera visibly snaps between physics positions.
    // mInterpAlpha = accumulator remainder / fixedDt, in [0, 1).
    Vector3 mPrevEyePos{0.0f};    // eye position from before the last physics step
    float   mPrevLeanTilt = 0.0f; // lean tilt from before the last physics step
    float   mInterpAlpha  = 1.0f; // interpolation fraction (1.0 = show current state)

    // Lean state — driven through motion poses (spring provides easing)
    int   mLeanDir        = 0;    // lean direction: -1=left, 0=center, +1=right
    float mLeanAmount     = 0.0f; // collision-limited lateral offset (derived from spring each frame)

    // Contacts from last collision resolution (used by detectGround and velocity removal)
    std::vector<SphereContact> mLastContacts;

    // ── Diagnostic logging state ──
    FILE *mLogFile = nullptr;   // per-timestep CSV log file (null = disabled)
    float mCamPitch = 0.0f;    // camera pitch from renderer (for logging only)

    /// Write one CSV row with full physics state. Called at end of fixedStep().
    inline void writeLogRow() {
        if (!mLogFile) return;

        float hSpeed = std::sqrt(mVelocity.x * mVelocity.x +
                                  mVelocity.y * mVelocity.y);
        Vector3 eye = computeRawEyePos();

        std::fprintf(mLogFile,
            "%.4f,%.5f,%s,%d,%d,"           // simTime,dt,mode,sneaking,running
            "%.3f,%.3f,%.3f,"               // posX,posY,posZ
            "%.3f,%.3f,%.3f,%.3f,"          // velX,velY,velZ,hSpeed
            "%.4f,%.4f,%.4f,"               // eyeX,eyeY,eyeZ
            "%.4f,%.4f,"                    // yaw,camPitch
            "%.5f,%.5f,%.5f,"              // springPosX,springPosY,springPosZ
            "%.5f,%.5f,%.5f,"              // springVelX,springVelY,springVelZ
            "%.5f,%.5f,%.5f,"              // poseTargetX,poseTargetY,poseTargetZ
            "%.5f,%.5f,%.5f,"              // poseStartX,poseStartY,poseStartZ
            "%.4f,%.4f,%d,"                // poseTimer,poseDur,poseHolding
            "%.3f,%d,%d,%.4f,"              // strideDist,strideIsLeft,leanDir,leanAmount
            "%d,%.2f,%.2f\n",               // cell,inputFwd,inputRight
            mSimTime, mTimestep.fixedDt, modeName(mCurrentMode),
            (int)mSneaking, (int)mRunning,
            mPosition.x, mPosition.y, mPosition.z,
            mVelocity.x, mVelocity.y, mVelocity.z, hSpeed,
            eye.x, eye.y, eye.z,
            mYaw, mCamPitch,
            mSpringPos.x, mSpringPos.y, mSpringPos.z,
            mSpringVel.x, mSpringVel.y, mSpringVel.z,
            mPoseCurrent.x, mPoseCurrent.y, mPoseCurrent.z,
            mPoseStart.x, mPoseStart.y, mPoseStart.z,
            mPoseTimer, mPoseDuration, (int)mPoseHolding,
            mStrideDist, (int)mStrideIsLeft, mLeanDir, mLeanAmount,
            mCellIdx, mInputForward, mInputRight);

        // Flush periodically so data isn't lost on crash
        static int flushCounter = 0;
        if (++flushCounter >= 10) {
            std::fflush(mLogFile);
            flushCounter = 0;
        }
    }
};

} // namespace Darkness

#endif // __PLAYERPHYSICS_H
