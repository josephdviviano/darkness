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

/// Mantle animation state machine — mirrors Dark Engine 5-state machine.
///
/// State sequence:
///   Hold → Rise → Forward → FinalRise → Complete
///
///
/// Virtual head position (mMantleHeadPos) driven by the same spring formula used
/// for head bob (computeSpringVelocity). The camera tracks this virtual head during
/// mantling, making body position changes invisible. During the "compressed ball"
/// phases (Forward, FinalRise), all 5 sub-model offsets are zeroed so the player
/// fits through tight gaps.

enum class MantleState {
    None,       // not mantling
    Hold,       // State 1: freeze position, suppress physics (0.3s)
    Rise,       // State 2: rise body to intermediate height (head peeks above ledge)
    Forward,    // State 3: "compressed ball" moves horizontally onto ledge (0.4s)
    FinalRise,  // State 4: "compressed ball" rises to final standing height
    Complete    // State 5: expand, find floor, restore normal mode (immediate)
};

/// Player physics simulation — custom 5-submodel polygon collision (2 real spheres + 3 point detectors).
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

    // Crouch submodel offsets are now driven by the pose system (MotionPoseData).
    // During crouch, HEAD drops via mPoseCurrent.z (e.g. POSE_CROUCH.vert = -2.02)
    // and BODY drops via mBodyPoseCurrent.z (e.g. POSE_CROUCH.bodyVert = -1.0).
    // Both blend smoothly over the pose duration (0.8s for crouch transition),
    // SHIN/KNEE/FOOT always use their standing offsets (no pose offsets).
    //
    // Reference crouch offsets (now in pose table):
    //   Crouch:     HEAD {0, 0, -2.02},  BODY {0, 0, -1.0}
    //   CrawlLeft:  HEAD {0, -0.15, -2.5}, BODY {0, 0, -1.0}
    //   CrawlRight: HEAD {0, 0.15, -2.5},  BODY {0, 0, -1.0}

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
    // Slope handling — purely friction-based.
    // GROUND_NORMAL_MIN: any surface with normal.z above this counts as "ground" for
    // state/landing purposes. The 5-sphere model and movement control handle the rest.
    // There is NO hard walkable slope angle. Friction = 0.03 * normal.z * gravity
    // naturally decreases on steeper slopes, reducing movement control and allowing
    // gravity's slope-parallel component to dominate. Sliding is emergent.
    static constexpr float GROUND_NORMAL_MIN = 0.01f;  // near-zero epsilon, matches original engine's normal.z > 0

    // Jump requires friction > JUMP_MIN_FRICTION. On flat ground friction = 0.96;
    // the threshold 0.5 corresponds to slopes steeper than ~58° (normal.z < 0.52).
    static constexpr float JUMP_MIN_FRICTION = 0.5f;

    // Grace period before entering Jump after losing ground contact.
    // Derived from freefall time to travel 0.1 units (contact break distance):
    // t = sqrt(2 * 0.1 / 32) ≈ 0.079s.
    static constexpr float GROUND_GRACE_DURATION = 0.08f;

    // Contact persistence — breaks terrain contacts at 0.1 units
    // separation or 5.0 u/s relative velocity. The grace period above approximates
    // the distance threshold for ground contacts. This age-based system complements
    // it for all contacts (walls, ceilings, etc.). Contacts not re-detected within
    // CONTACT_MAX_AGE frames are culled from the persistent set.
    static constexpr int CONTACT_MAX_AGE = 3;

    // Collision iteration count — now set per-preset via PhysicsTimestep.collisionIters.
    // Vintage (12.5Hz): 3 iterations.
    // Modern/Ultra: 1 iteration (higher Hz compensates with more steps/sec).

    // Stair stepping — 3-phase raycast algorithm for automatically stepping up
    // small ledges during ground movement. Triggered when FOOT/SHIN/KNEE contacts
    // have a steep/vertical normal (z < STEP_WALL_THRESHOLD), meaning the player
    // hit a wall-like surface at foot level.
    //
    // Phase 1: Probe UP to check clearance above the ledge
    // Phase 2: Probe FORWARD (along velocity) to check space on the ledge
    // Phase 3: Probe DOWN to find the surface to stand on
    //
    // The lift height is computed from the downward probe hit, clamped to a
    // minimum of STEP_MIN_ZDELTA to handle very small ledges that would
    // otherwise be missed. An additional STEP_CLEARANCE is added for safety.
    static constexpr float STEP_WALL_THRESHOLD = 0.4f;   // normal.z below this = "wall" (steep surface)
    static constexpr float STEP_UP_DIST        = 2.0f;   // upward probe distance (units)
    static constexpr float STEP_FWD_SCALE      = 0.01f;  // forward probe = velocity * this
    static constexpr float STEP_UP_EPSILON     = 0.01f;  // UP ray start offset above foot (collision skin)
    static constexpr float STEP_MIN_ZDELTA     = 0.3f;   // minimum lift height (units)
    static constexpr float STEP_CLEARANCE      = 0.02f;  // additional clearance above step surface

    // Landing detection thresholds.
    static constexpr float LANDING_MIN_VEL  = 2.0f;    // minimum downward speed for landing pose
    static constexpr float LANDING_MIN_TIME = 0.2f;    // minimum 200ms between landing events

    // Mantle system — 3-ray detection + 5-state animation.
    // Detection: UP → FWD → DOWN raycasts, then forward surface finding from HEAD/BODY/FOOT.
    // Animation: Hold(0.3s) → Rise(to intermediate) → Forward(0.4s) → FinalRise → Complete.
    static constexpr float MANTLE_UP_DIST   = 3.5f;    // upward headroom check distance
    static constexpr float MANTLE_FWD_DIST  = 2.4f;    // forward ledge reach (radius × 2)
    static constexpr float MANTLE_DOWN_DIST = 7.0f;    // downward surface scan distance
    static constexpr float MANTLE_HOLD_TIME = 0.3f;    // State 1: hold duration
    static constexpr float MANTLE_FWD_TIME  = 0.4f;    // State 3: forward movement duration
    static constexpr float MANTLE_PULLBACK  = 0.55f;   // pull back from wall (approx half radius)
    static constexpr float MANTLE_CONVERGE  = 0.0001f; // distance² convergence threshold (= 0.01 units)

    // Movement control — force/mass-based system with separate ground friction.
    //
    // Accumulates forces and integrates:
    //   rate = min(CONTROL_MULTIPLIER * PLAYER_MASS * friction / VELOCITY_RATE, MAX_CONTROL_RATE)
    //   ctrl_accel = (target_vel - current_vel) * rate
    //   new_vel = vel + ctrl_accel * dt / PLAYER_MASS
    //
    // For the velocity-controlled player, mass cancels in the derivation:
    //  alpha = rate * dt / mass = min(11 * friction / vrate, 2000/mass) * dt
    // This gives exponential convergence with time constant tau = 1/(11*0.96) = 0.095s.
    // Deceleration uses the same formula with desired=0 (no separate decel constant).
    //
    // Ground friction is computed separately by computeGroundFriction(), which sums
    // contributions from all ground contacts with per-terrain scaling. This replaces
    // the inline single-normal calculation used previously.
    //
    // On flat ground: friction = 0.03 * 1.0 * 32.0 = 0.96
    //   rate = 11 * 180 * 0.96 / 1.0 = 1900.8 (below 2000 cap)
    //   alpha at 60Hz = 1900.8 * 0.0167 / 180 = 0.176 per step
    //   alpha at 12.5Hz = 1900.8 * 0.08 / 180 = 0.845 per step
    static constexpr float CONTROL_MULTIPLIER = 11.0f;
    static constexpr float FRICTION_FACTOR    = 0.03f;
    static constexpr float VELOCITY_RATE      = 1.0f;

    // Player mass — used in force/mass calculations. Mass cancels in the
    // velocity-control path (rate = 11*mass*friction/vrate, accel = ctrl/mass),
    // so this only matters for the rate cap and future non-controlled states.
    static constexpr float PLAYER_MASS = 180.0f;

    // Maximum control rate (units/sec²). Limits control acceleration on
    // high-friction surfaces.
    // Flat ground: rate = 11 * 180 * 0.96 / 1.0 = 1900.8 (below cap).
    static constexpr float MAX_CONTROL_RATE = 2000.0f;  // Caps just above ground rate.

    // Z-axis friction multiplier for separate friction forces.
    // Extra vertical friction (40%) aids landing deceleration. Applied
    // in the non-velocity-controlled friction path (future: mantling, ragdoll).
    static constexpr float Z_FRICTION_MULT = 1.4f;

    // Base friction in water — provides drag even without ground contacts.
    // Air has base_friction = 0 (no drag), water = 0.3.
    static constexpr float WATER_BASE_FRICTION = 0.3f;

    // Water control force scaling — all control forces halved when submerged.
    // Makes water movement feel sluggish and heavy.
    static constexpr float WATER_CONTROL_SCALE = 0.5f;

    // Jump impulse scaling in water (half-strength jumps from pool floor/treading water).
    static constexpr float WATER_JUMP_SCALE = 0.5f;

    // Air control fraction — scales ground-equivalent friction for airborne steering.
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

    /// Configuration from P$PhysAttr and P$PhysDims properties.
    /// All fields default to the hardcoded constants. Call applyConfig() to override.
    struct PlayerPhysicsConfig {
        float gravityScale = 1.0f;      // PhysAttr.gravity / 100.0
        float mass         = PLAYER_MASS;
        float density      = 0.9f;      // buoyancy factor (<1 floats, 1.0 neutral, >1 sinks)
        float elasticity   = 0.0f;      // bounce coefficient (future)
        float headRadius   = SPHERE_RADIUS;
        float bodyRadius   = SPHERE_RADIUS;
        float headOffsetZ  = HEAD_OFFSET_Z;
        float bodyOffsetZ  = BODY_OFFSET_Z;
    };

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

    /// Compute spring velocity from the dt-dependent formula.
    /// Returns new velocity; caller integrates position (pos += vel * dt).
    /// This separation allows callers to modify velocity before integration
    /// (e.g., mantle velocity multipliers in states 3/4).
    static inline Vector3 computeSpringVelocity(
        const Vector3& pos, const Vector3& oldVel, const Vector3& target,
        float tension, float damping, float zScale, float velCap, float dt)
    {
        float springDt = std::clamp(dt, 0.001f, HEAD_SPRING_MAX_DT);
        float tensionEff = tension / springDt;
        float dampingEff = damping + (1.0f - damping) * springDt;

        Vector3 disp = target - pos;
        Vector3 newVel = disp * tensionEff;
        newVel.z *= zScale;
        newVel = newVel + oldVel * dampingEff;

        float mag = glm::length(newVel);
        if (mag > velCap) newVel *= velCap / mag;
        return newVel;
    }

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

    /// Motion pose data — target head and body displacement from stance rest position.
    /// Animates both submodels through a single pose blend system. Body offsets appear only in
    /// crouch-related poses (crouch idle, crawl strides, crouch lean, weapon swing crouch).
    struct MotionPoseData {
        float duration;    // blend time from current pose to this target (seconds)
        float holdTime;    // hold at target before allowing next pose (seconds)
        float fwd;         // HEAD forward displacement
        float lat;         // HEAD lateral displacement (positive = right)
        float vert;        // HEAD vertical displacement (negative = dip)
        float bodyFwd;     // BODY forward displacement
        float bodyLat;     // BODY lateral displacement (positive = right)
        float bodyVert;    // BODY vertical displacement (negative = dip)
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
    //                                                   dur    hold   fwd     lat      vert     bFwd   bLat   bVert
    static constexpr MotionPoseData POSE_NORMAL       = {0.8f,  0.0f,  0.0f,   0.0f,    0.0f,    0.0f,  0.0f,  0.0f};

    // Walking strides — progressive blend over 0.6s.
    // The motion coordinator blends mPoseCurrent toward the stride target over the
    // duration, then the head spring tracks mPoseCurrent with spring dynamics.
    // Both the blend curve and the spring's overshoot/lag contribute to natural bob.
    //
    // Both strides use identical vertical dip (-0.4) and symmetric lateral sway (±0.1).
    // Lateral sign convention: original Y=LEFT-positive → our lat=RIGHT-positive,
    // so left stride sways right (+0.1) and right stride sways left (-0.1),
    // pushing the body AWAY from the stepping foot.
    static constexpr MotionPoseData POSE_STRIDE_LEFT  = {0.6f,  0.01f, 0.0f,   0.1f,   -0.4f,   0.0f,  0.0f,  0.0f};
    static constexpr MotionPoseData POSE_STRIDE_RIGHT = {0.6f,  0.01f, 0.0f,  -0.1f,   -0.4f,   0.0f,  0.0f,  0.0f};

    // Crouching strides — slightly wider sway, deeper vertical offset from crouch height.
    // Vert is relative to POSE_CROUCH (-2.02). Both crawl strides use identical vertical
    // dip (-2.5) and symmetric
    // lateral sway (±0.15), with same sign convention as standing strides.
    static constexpr MotionPoseData POSE_CRAWL_LEFT   = {0.6f,  0.01f, 0.0f,   0.15f,  -2.5f,   0.0f,  0.0f, -1.0f};
    static constexpr MotionPoseData POSE_CRAWL_RIGHT  = {0.6f,  0.01f, 0.0f,  -0.15f,  -2.5f,   0.0f,  0.0f, -1.0f};

    // Crouching idle — blend over 0.8s for smooth crouch transition:
    static constexpr MotionPoseData POSE_CROUCH       = {0.8f,  0.0f,  0.0f,   0.0f,   -2.02f,  0.0f,  0.0f, -1.0f};

    // Landing impact — instantaneous dip on landing (dur=0 = snap), held briefly:
    static constexpr MotionPoseData POSE_JUMP_LAND    = {0.0f,  0.1f,  0.0f,   0.0f,   -0.5f,   0.0f,  0.0f,  0.0f};

    // Body carry mode (carrying a body or heavy object):
    // Heavier bob — deeper dip and more lateral sway while carrying.
    static constexpr MotionPoseData POSE_CARRY_IDLE   = {0.8f,  0.0f,  0.0f,   0.0f,   -0.8f,   0.0f,  0.0f,  0.0f};
    static constexpr MotionPoseData POSE_CARRY_LEFT   = {0.6f,  0.01f, 0.0f,   0.5f,   -1.5f,   0.0f,  0.0f,  0.0f};
    static constexpr MotionPoseData POSE_CARRY_RIGHT  = {0.6f,  0.01f, 0.0f,  -0.15f,  -1.1f,   0.0f,  0.0f,  0.0f};

    // Weapon swing (head bob during sword/blackjack attacks):
    // Instant target — spring overshoot creates natural recoil feel.
    static constexpr MotionPoseData POSE_WEAPON_SWING       = {0.0f,  0.0f, 0.8f,  0.0f,   0.0f,    0.0f,  0.0f,  0.0f};
    static constexpr MotionPoseData POSE_WEAPON_SWING_CROUCH = {0.0f, 0.0f, 0.8f,  0.0f,  -2.02f,  0.8f,  0.0f, -1.0f};

    // Standing lean — 1.5s progressive blend for smooth easing into lean position.
    // Collision-limited via updateLean().
    // lat is in player-local coords: positive = right. So lean LEFT = negative lat.
    static constexpr MotionPoseData POSE_LEAN_LEFT    = {1.5f,  0.0f,  0.0f,  -2.2f,    0.0f,   0.0f,  0.0f,  0.0f};
    static constexpr MotionPoseData POSE_LEAN_RIGHT   = {1.5f,  0.0f,  0.0f,   2.2f,    0.0f,   0.0f,  0.0f,  0.0f};
    static constexpr MotionPoseData POSE_LEAN_FWD     = {1.5f,  0.0f,  2.2f,   0.0f,    0.0f,   0.0f,  0.0f,  0.0f};

    // Crouching lean — reduced distance + vertical drop:
    static constexpr MotionPoseData POSE_CLNLEAN_LEFT  = {1.5f, 0.0f, 0.0f,  -1.7f,   -2.0f,   0.0f,  0.0f, -1.0f};
    static constexpr MotionPoseData POSE_CLNLEAN_RIGHT = {1.5f, 0.0f, 0.0f,   1.7f,   -2.0f,   0.0f,  0.0f, -1.0f};
    static constexpr MotionPoseData POSE_CLNLEAN_FWD   = {1.5f, 0.0f, 1.7f,   0.0f,   -2.0f,   0.0f,  0.0f, -1.0f};

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

    // Non-copyable, non-movable: holds raw FILE* and reference member
    PlayerPhysics(const PlayerPhysics &) = delete;
    PlayerPhysics &operator=(const PlayerPhysics &) = delete;
    PlayerPhysics(PlayerPhysics &&) = delete;
    PlayerPhysics &operator=(PlayerPhysics &&) = delete;

    /// Change the active physics timestep preset at runtime.
    void setTimestep(const PhysicsTimestep &ts) { mTimestep = ts; }

    /// Get the active physics timestep configuration.
    const PhysicsTimestep& getTimestep() const { return mTimestep; }

    /// Get the physics update rate in Hz (= 1 / fixedDt).
    float getPhysicsHz() const { return 1.0f / mTimestep.fixedDt; }

    /// Apply physics configuration from P$PhysAttr / P$PhysDims properties.
    /// Values override the hardcoded defaults. Call once after construction.
    void applyConfig(const PlayerPhysicsConfig &cfg) {
        mMass = cfg.mass;
        mDensity = std::max(cfg.density, 0.01f);  // clamp to prevent division by zero
        mElasticity = cfg.elasticity;
        // Clamp radii to minimum 0.1 to prevent zero-division in collision
        mSphereRadii[0] = std::max(cfg.headRadius, 0.1f);
        mSphereRadii[1] = std::max(cfg.bodyRadius, 0.1f);
        // SHIN/KNEE/FOOT stay at zero radius (point detectors, not in P$PhysDims)
        mSphereOffsetsBase[0] = cfg.headOffsetZ;
        mSphereOffsetsBase[1] = cfg.bodyOffsetZ;
        // SHIN/KNEE/FOOT offsets stay at static defaults (not in P$PhysDims)
        setGravityMagnitude(GRAVITY * cfg.gravityScale);
    }

    /// Get the current gravity magnitude (units/sec²).
    float getGravityMagnitude() const { return mGravityMag; }

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
            "bodyPoseX,bodyPoseY,bodyPoseZ,"
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

    /// Enable/disable stair step diagnostics to stderr ([STEP] prefix).
    void setStepLog(bool enable) { mStepLog = enable; }
    bool stepLogEnabled() const { return mStepLog; }

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
    inline void setYaw(float yaw) {
        mYaw = yaw;
        mCosYaw = std::cos(yaw);
        mSinYaw = std::sin(yaw);
    }

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

    /// Check if the player's center of gravity is in a water cell.
    /// Used for buoyancy, jump scaling, and mode transitions.
    bool isInWater() const {
        if (mCellIdx < 0) return false;
        return mCollision.getMediaType(mPosition) == 2;
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
            mCellIdx = mCollision.findCell(pos + Vector3(0.0f, 0.0f, mSphereOffsetsBase[0]));
        }
        if (mCellIdx < 0) {
            // Try at foot sphere level (body center is higher in 5-sphere model,
            // so the foot sphere position might be inside the cell)
            mCellIdx = mCollision.findCell(pos + Vector3(0.0f, 0.0f, mSphereOffsetsBase[4]));
        }
        if (mCellIdx < 0) {
            // Last resort: try slightly above the given position
            mCellIdx = mCollision.findCell(pos + Vector3(0.0f, 0.0f, mSphereRadii[0]));
        }
        mCurrentMode = PlayerMode::Jump; // will detect ground on next step
        mGroundGraceTimer = 0.0f;
        mGroundGraceActive = false;
        mPersistentContacts.clear();

        // Reset spring state — teleporting should not carry old spring velocity
        // into the new position, which would cause oscillation on arrival.
        mSpringPos = Vector3(0.0f);
        mSpringVel = Vector3(0.0f);
        mPoseCurrent = Vector3(0.0f);
        mBodyPoseCurrent = Vector3(0.0f);
        mBodyPoseEnd = Vector3(0.0f);

        // Initialize interpolation state to current position so there's no
        // lerp glitch on the first frame after a teleport/spawn.
        mPrevEyePos   = computeRawEyePos();
        mPrevLeanTilt = computeRawLeanTilt();
        mInterpAlpha  = 1.0f;
    }

    /// Set gravity magnitude (units/sec²). Default is GRAVITY.
    void setGravityMagnitude(float g) { mGravityMag = g; }

    /// Register a callback for footstep sound events. Fired each time a stride
    /// triggers (distance-based). The callback receives the player's foot position,
    /// horizontal speed, and ground texture index for material-based sound selection.
    /// Phase 3 Audio will use this to play surface-appropriate footstep sounds.
    void setFootstepCallback(FootstepCallback cb) { mFootstepCb = std::move(cb); }

    /// Callback for object collision testing. Called per physics step during
    /// resolveCollisions(), after WR polygon tests but before push de-duplication.
    /// Receives the 5 submodel sphere centers and radii, the player's current
    /// portal cell, and an output vector to append SphereContact results into.
    ///
    /// Object contacts use cellIdx = -1 as sentinel (distinguishing them from WR
    /// contacts), and polyIdx encodes (bodyIndex << 4) | (faceIdx & 0xF).
    ///
    /// ODE UPGRADE: When ODE rigid bodies are added, this callback pattern would
    /// be replaced by registering the player as a kinematic dGeomID and using
    /// dNearCallback to generate contacts between player and object geoms.
    using ObjectCollisionCallback = std::function<void(
        const Vector3 *sphereCenters,
        const float *sphereRadii,
        int numSpheres,
        int32_t playerCell,
        std::vector<SphereContact> &outContacts)>;

    /// Register a callback for player-vs-object collision. Set by DarkPhysics
    /// to bridge PlayerPhysics with ObjectCollisionWorld.
    void setObjectCollisionCallback(ObjectCollisionCallback cb) {
        mObjectCollisionCb = std::move(cb);
    }

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
        // Requires friction > 0.5 to jump — prevents jumping on steep slopes where
        // the player has no traction (above ~58°).
        // When friction is between 0.5 and 1.0, jump velocity is scaled down
        // proportionally, making slope-edge jumps weaker.
        if (mJumpRequested && !mMotionDisabled) {
            if (isOnGround()) {
                float jumpFriction = computeGroundFriction();
                if (jumpFriction > JUMP_MIN_FRICTION) {
                    // Remove vertical velocity component, boost horizontal by 5%,
                    // add jump impulse upward, then scale the ENTIRE velocity by
                    // friction when friction <= 1.0. On slopes, both the
                    // jump height AND horizontal momentum are reduced.
                    mVelocity.z = 0.0f;
                    mVelocity.x *= 1.05f;
                    mVelocity.y *= 1.05f;
                    mVelocity.z = JUMP_IMPULSE;

                    // Weaker jumps when standing on a submerged surface (pool floor).
                    // Water resistance halves the jump impulse.
                    if (isInWater())
                        mVelocity.z *= WATER_JUMP_SCALE;

                    // Scale entire velocity by friction on slopes
                    if (jumpFriction <= 1.0f) {
                        mVelocity *= jumpFriction;
                    }
                    mCurrentMode = PlayerMode::Jump;
                    // Cancel any active ground grace — we're definitely airborne now
                    mGroundGraceTimer = 0.0f;
                    mGroundGraceActive = false;
                    // Reset stride bob — locks motion to POSE_NORMAL during jump mode.
                    // DON'T override an active lean.
                    if (mLeanDir == 0)
                        activatePose(POSE_NORMAL);
                    mStrideDist = 0.0f;
                }
            } else if (mCurrentMode == PlayerMode::Swim) {
                // Water jump — vertical kick from treading water.
                // Original engine allows jumping when on_ground OR in_water.
                // No friction check (no slope in water), always half-strength.
                mVelocity.z = JUMP_IMPULSE * WATER_JUMP_SCALE;
                mCurrentMode = PlayerMode::Jump;
                mGroundGraceTimer = 0.0f;
                mGroundGraceActive = false;
                if (mLeanDir == 0)
                    activatePose(POSE_NORMAL);
                mStrideDist = 0.0f;
            }
        }
        // NOTE: mJumpRequested is cleared after mantle check (step 13) so that
        // pressing jump while airborne can trigger a mantle.
        // Requires a fresh jump press to initiate mantling — holding forward
        // while airborne is not sufficient.

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

        // 10b. Stair stepping — after collision resolution finds wall contacts
        //      at leg level, try to step up small ledges. Must run before
        //      detectGround() so the stepped position is used for ground state.
        //      Only when grounded and moving horizontally.
        tryStairStep();

        // 11. Detect ground and update movement state
        detectGround();

        // 12. Update cell index again (collision may have shifted position)
        // Relies purely on gravity + collision + constraint response to keep
        // the player grounded. An explicit snap was causing jitter (collision
        // pushes up, snap pulls down) and fighting with uphill movement.
        updateCell();

        // 13. Check for mantle opportunity — requires jump key press while airborne.
        // Triggers mantle only on a fresh jump input, not from
        // just being airborne and holding forward. This prevents auto-mantling when
        // running into walls or stepping off ledges.
        if (mCurrentMode == PlayerMode::Jump && mJumpRequested
            && mInputForward > 0.1f && !mMotionDisabled) {
            checkMantle();
        }
        mJumpRequested = false;  // clear after both jump (step 1) and mantle (step 13)

        // 14. Update lean (visual-only lateral camera offset with collision)
        updateLean();

        // 15. Write diagnostic log row (if logging enabled)
        writeLogRow();
    }

    /// Apply gravity to velocity (if not on ground, or during grace period).
    /// During the ground grace period, the player stays in a ground mode for
    /// movement control (friction, stride bob) but gravity still applies so
    /// walking off a ledge transitions naturally into a fall. Without this,
    /// the player would hover for the grace duration before starting to drop.
    inline void applyGravity() {
        if (!isOnGround() || mGroundGraceActive) {
            // Z-up: gravity pulls downward (uses scaled gravity for per-room variation)
            mVelocity.z -= mGravityMag * mTimestep.fixedDt;
        }
        // Buoyancy: always active when COG is in water, regardless of movement mode.
        // Uses raw GRAVITY constant (not gravity-scaled) —
        // buoyancy and room gravity scaling are independent systems.
        // With density=0.9: buoyancy = 32/0.9 = 35.6 > gravity = 32 → player floats up.
        if (isInWater()) {
            mVelocity.z += (GRAVITY / mDensity) * mTimestep.fixedDt;
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
        float cosYaw = mCosYaw;
        float sinYaw = mSinYaw;
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

    /// Compute ground friction from the best ground contact.
    ///
    /// Uses the most upward-facing contact normal from mLastContacts to compute:
    ///   friction = FRICTION_FACTOR * terrainScale * bestNormal.z * gravity
    ///
    /// Re-detects contacts each frame, so mLastContacts may be empty
    /// when the player stands stably without penetration. In that case, we fall
    /// back to mGroundNormal (set by detectGround()'s probe), which persists
    /// from the previous frame's ground detection.
    ///
    /// Sums friction from all upward-facing contacts. Multiple
    /// floor contacts (e.g., BODY + FOOT on the same surface) contribute
    /// additive friction, which typically pushes the control rate to the 2000
    /// cap on walkable slopes. This ensures the player has full control
    /// authority up to about 69°.
    ///
    /// Falls back to WATER_BASE_FRICTION when swimming with no ground contacts.
    /// Returns 0 in air (no contacts, not swimming).
    inline float computeGroundFriction() const {
        // Sum friction from all upward-facing contacts — Multiple submodels touching
        // the same floor surface contribute additive friction. This is critical: on
        // a 60° slope, single-contact
        // friction = 0.48 (below the 0.5 jump gate), but summed friction = ~1.44
        // (well above), matching the original's behavior where the player can
        // still jump on 60° slopes.
        float totalFriction = 0.0f;
        for (const auto &c : mLastContacts) {
            if (c.normal.z > 0.0f) {
                float terrainScale = getTerrainFriction(c.textureIdx);
                totalFriction += FRICTION_FACTOR * terrainScale * c.normal.z * mGravityMag;
            }
        }

        // Fallback: if no upward contacts in mLastContacts but we ARE on ground,
        // use mGroundNormal from detectGround()'s probe. This handles the common
        // case where the player stands stably without generating collision contacts
        // (probe detects ground but doesn't populate mLastContacts).
        if (totalFriction == 0.0f && isOnGround() && mGroundNormal.z > GROUND_NORMAL_MIN) {
            totalFriction = FRICTION_FACTOR * mGroundNormal.z * mGravityMag;
        }

        // Water base friction provides drag when submerged with no ground contacts
        if (totalFriction == 0.0f && mCurrentMode == PlayerMode::Swim) {
            totalFriction = WATER_BASE_FRICTION;
        }
        return totalFriction;
    }

    /// Per-terrain friction multiplier (default 1.0).
    /// Infrastructure for future terrain property lookup from texture index.
    /// When terrain properties are loaded, this can map textureIdx to the
    /// per-texture friction scale from the level's property data.
    inline float getTerrainFriction(int32_t /*textureIdx*/) const {
        return 1.0f;
    }

    /// Apply movement — force/mass-based control with separate ground friction.
    ///
    /// For velocity-controlled movement (walking/running/crouching):
    ///   friction = sum of contact friction contributions (computeGroundFriction)
    ///   rate = min(11 * mass * friction / velocity_rate, 2000)
    ///   alpha = rate * dt / mass   (mass cancels, giving exponential convergence)
    ///   vel += (desired - vel) * alpha
    ///
    /// This creates exponential approach to desired velocity with tau ~0.095s on
    /// flat ground. When desired=0 (no input), velocity decays exponentially
    /// toward zero — no separate deceleration constant needed.
    ///
    inline void applyMovement() {
        Vector3 desired = computeDesiredVelocity();
        const float dt = mTimestep.fixedDt;

        // Compute friction from all ground contacts (replaces inline single-normal calc)
        float friction = computeGroundFriction();

        if (isOnGround() && !mGroundGraceActive) {
            // Project desired velocity onto ground plane so walking on slopes
            // doesn't fight gravity. Always project when on any ground surface.
            if (mGroundNormal.z > GROUND_NORMAL_MIN) {
                float dot = glm::dot(desired, mGroundNormal);
                desired -= mGroundNormal * dot;
            }

            // No additional slope friction penalty — the base friction formula
            // (friction = 0.03 * normal.z * gravity) already provides slope-dependent friction. Steeper
            // slopes naturally have lower friction_pct (= normal.z), reducing
            // control authority and allowing gravity to dominate. Sliding is
            // emergent from this single formula, not from an explicit threshold.

            // Control rate: rate = min(11 * mass * friction / velocity_rate, 2000)
            // Alpha = rate * dt / mass — mass cancels, giving exponential convergence.
            // Friction is SUMMED across all submodel contacts.
            // On flat ground with ~3 contacts: friction ≈ 2.88, rate = capped at 2000.
            float rate = CONTROL_MULTIPLIER * mMass * friction / VELOCITY_RATE;
            rate = std::min(rate, MAX_CONTROL_RATE);
            float alpha = rate * dt / mMass;
            alpha = std::min(alpha, 1.0f);  // safety cap: prevent overshoot at low framerates

            // Apply convergence to horizontal velocity.
            // Preserves desired.z for slope projection (SET, not converge, to avoid
            // accumulation on slopes that launches player off ramps).
            float prevZ = mVelocity.z;
            Vector3 hVel(mVelocity.x, mVelocity.y, 0.0f);
            Vector3 hDesired(desired.x, desired.y, 0.0f);

            // Force-based convergence: vel += (desired - vel) * alpha
            Vector3 hNew = hVel + (hDesired - hVel) * alpha;

            // Velocity reversal check — if convergence would reverse horizontal
            // velocity direction when decelerating (desired ~= 0), stop instead.
            // Prevents oscillation around zero from overshooting.
            if (glm::length2(hDesired) < 0.001f && glm::dot(hNew, hVel) < 0.0f) {
                hNew = Vector3(0.0f);
            }

            mVelocity.x = hNew.x;
            mVelocity.y = hNew.y;

            // Z component: set from slope projection (not converged)
            mVelocity.z = desired.z != 0.0f ? desired.z : prevZ;
        } else if (isOnGround()) {
            // Grace period: ground mode for friction/stride but no slope
            // projection. The player is slightly above the ground and falling
            // under gravity (applyGravity applies during grace). Slope
            // projection would remove horizontal velocity based on a stale
            // mGroundNormal, causing the player to plummet straight down at
            // ledge edges instead of arcing naturally.
            //
            // Use full ground friction for movement control so the player
            // doesn't suddenly lose traction during grace.
            float rate = CONTROL_MULTIPLIER * mMass * friction / VELOCITY_RATE;
            rate = std::min(rate, MAX_CONTROL_RATE);
            float alpha = rate * dt / mMass;
            alpha = std::min(alpha, 1.0f);

            Vector3 hVel(mVelocity.x, mVelocity.y, 0.0f);
            Vector3 hDesired(desired.x, desired.y, 0.0f);
            Vector3 hNew = hVel + (hDesired - hVel) * alpha;

            if (glm::length2(hDesired) < 0.001f && glm::dot(hNew, hVel) < 0.0f) {
                hNew = Vector3(0.0f);
            }

            mVelocity.x = hNew.x;
            mVelocity.y = hNew.y;
            // Z unchanged — gravity handles it during grace
        } else if (mCurrentMode == PlayerMode::Swim) {
            // ── Swimming: 3D movement with look-direction Z control ──
            //
            // Unlike air/ground paths which strip Z, swimming uses the camera pitch
            // to project forward movement into 3D. Looking up and pressing forward
            // swims upward; looking down swims downward.
            //
            // Build 3D desired velocity using camera pitch for Z component.
            // forward3D = (cos(yaw)*cos(pitch), sin(yaw)*cos(pitch), sin(pitch))
            // right stays horizontal = (sin(yaw), -cos(yaw), 0)
            float cosPitch = std::cos(mCamPitch);
            float sinPitch = std::sin(mCamPitch);
            Vector3 forward3D(mCosYaw * cosPitch, mSinYaw * cosPitch, sinPitch);
            Vector3 right(mSinYaw, -mCosYaw, 0.0f);

            // Apply mode speed scaling (swim = 0.7x) and speed modifiers
            float modeScale = MODE_SPEEDS[static_cast<int>(mCurrentMode)].trans;
            float fwdSpeed = WALK_SPEED * modeScale;
            float strSpeed = SIDESTEP_SPEED * modeScale;

            if (mInputForward < 0.0f) fwdSpeed = BACKWARD_SPEED * modeScale;
            if (mSneaking) { fwdSpeed *= 0.5f; strSpeed *= 0.5f; }
            else if (mRunning) { fwdSpeed *= 2.0f; strSpeed *= 2.0f; }

            Vector3 swimDesired = forward3D * (mInputForward * fwdSpeed)
                                + right     * (mInputRight   * strSpeed);

            // Control rate — flat friction, no velocity-dependent scaling.
            // Velocity-dependent drag is in the friction-force
            // path which is bypassed for velocity-controlled objects (the player).
            // Player swim control rate: 11 * mass * 0.3 / 1.0 = 594 at mass=180.
            float rate = CONTROL_MULTIPLIER * mMass * WATER_BASE_FRICTION / VELOCITY_RATE;
            rate = std::min(rate, MAX_CONTROL_RATE);
            float alpha = rate * WATER_CONTROL_SCALE * dt / mMass;
            alpha = std::min(alpha, 1.0f);

            // 3D velocity convergence (no Z stripping — full 3D control)
            mVelocity += (swimDesired - mVelocity) * alpha;

        } else {
            // Airborne: limited air control.
            // In air, contact friction = 0 and base_friction = 0, so we use
            // flat-ground equivalent friction scaled by AIR_CONTROL_FRAC.
            Vector3 airDesired(desired.x, desired.y, 0.0f);
            Vector3 hVel(mVelocity.x, mVelocity.y, 0.0f);

            float airFriction = FRICTION_FACTOR * 1.0f * mGravityMag;  // flat-ground equivalent
            float airRate = CONTROL_MULTIPLIER * mMass * airFriction / VELOCITY_RATE;
            airRate = std::min(airRate, MAX_CONTROL_RATE);
            float airAlpha = airRate * AIR_CONTROL_FRAC * dt / mMass;

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

        // Collect unique constraint normals from previous frame's contacts.
        // Bounded by de-duplication — at most ~5 unique wall directions.
        static constexpr int MAX_CONSTRAINTS = 8;
        Vector3 constraintBuf[MAX_CONSTRAINTS];
        int constraintCount = 0;

        for (const auto &c : mLastContacts) {
            // Only constrain if velocity is moving into the surface
            float vn = glm::dot(mVelocity, c.normal);
            if (vn >= 0.0f) continue;

            bool duplicate = false;
            for (int i = 0; i < constraintCount; ++i) {
                if (glm::dot(c.normal, constraintBuf[i]) > 0.99f) { duplicate = true; break; }
            }
            if (!duplicate && constraintCount < MAX_CONSTRAINTS)
                constraintBuf[constraintCount++] = c.normal;
        }

        // Apply constraint-based velocity removal
        if (constraintCount == 1) {
            // Single surface — remove velocity component along the normal
            float vn = glm::dot(mVelocity, constraintBuf[0]);
            if (vn < 0.0f)
                mVelocity -= constraintBuf[0] * vn;

        } else if (constraintCount == 2) {
            // Two surfaces (crease/wedge) — slide along the edge between them
            Vector3 edge = glm::cross(constraintBuf[0], constraintBuf[1]);
            float edgeLen = glm::length(edge);
            if (edgeLen > 1e-6f) {
                edge /= edgeLen;
                mVelocity = edge * glm::dot(mVelocity, edge);
            } else {
                float vn = glm::dot(mVelocity, constraintBuf[0]);
                if (vn < 0.0f)
                    mVelocity -= constraintBuf[0] * vn;
            }

        } else if (constraintCount >= 3) {
            // Corner (3+ surfaces) — project onto edge, validate direction
            Vector3 origVel = mVelocity;
            Vector3 edge = glm::cross(constraintBuf[0], constraintBuf[1]);
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

        // Compute current sphere offsets — HEAD and BODY offsets are driven
        // by the pose blend system (mPoseCurrent.z for HEAD, mBodyPoseCurrent.z
        // for BODY). This replaces the old static CROUCH_OFFSETS[] lookup,
        // giving smooth collision sphere transitions during crouch/uncrouch.

        // Age all persistent contacts — increment frame counter before new detection.
        // Contacts re-detected this frame will have their age reset to 0.
        for (auto &c : mPersistentContacts)
            c.age++;

        mLastContacts.clear();
        mLastContacts.reserve(16);

        for (int iter = 0; iter < mTimestep.collisionIters; ++iter) {
            if (mCellIdx < 0)
                break;

            mIterContacts.clear();

            for (int s = 0; s < NUM_SPHERES; ++s) {
                float sphereR = mSphereRadii[s];
                // HEAD and BODY get pose-driven vertical offsets for smooth crouch transitions.
                // SHIN/KNEE/FOOT (s >= 2) always use their standing offsets.
                float poseOffsetZ = 0.0f;
                if (s == 0) poseOffsetZ = mPoseCurrent.z;        // HEAD
                else if (s == 1) poseOffsetZ = mBodyPoseCurrent.z; // BODY
                float offsetZ = mSphereOffsetsBase[s] + poseOffsetZ;
                Vector3 sphereCenter = mPosition + Vector3(0.0f, 0.0f, offsetZ);

                // Track contact count before this submodel's tests so we can
                // tag all new contacts with the submodel index afterwards.
                size_t contactsBefore = mIterContacts.size();

                // Test against the body center's cell
                mCollision.sphereVsCellPolygons(
                    sphereCenter, sphereR, mCellIdx, mIterContacts);

                // Test the submodel's own cell if it differs from body center's cell
                // (handles straddling portal boundaries, e.g. feet in one cell, head in another)
                int32_t sphereCell = mCollision.findCell(sphereCenter);
                if (sphereCell >= 0 && sphereCell != mCellIdx) {
                    mCollision.sphereVsCellPolygons(
                        sphereCenter, sphereR, sphereCell, mIterContacts);
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
                            mCollision.sphereVsCellPolygons(
                                sphereCenter, sphereR, tgtCell, mIterContacts);
                        }
                    }
                }

                // Tag all new contacts from this submodel with its index.
                // Enables per-submodel filtering in tryStairStep() (leg-level
                // contacts only) and detectGround() (FOOT recovery).
                for (size_t ci = contactsBefore; ci < mIterContacts.size(); ++ci) {
                    mIterContacts[ci].submodelIdx = static_cast<int8_t>(s);
                }
            }

            // ── Object collision pass ──
            // After testing WR cell polygons for all 5 submodels, test against
            // placed object collision bodies (crates, furniture, doors, etc.).
            // The callback is set by DarkPhysics to invoke ObjectCollisionWorld's
            // testPlayerSpheres(), which appends SphereContact results (with
            // cellIdx=-1 sentinel) into mIterContacts alongside the WR contacts.
            if (mObjectCollisionCb) {
                // Build sphere center array from current body position + offsets
                // (same computation as the per-sphere loop above)
                Vector3 sphereCenters[NUM_SPHERES];
                for (int s = 0; s < NUM_SPHERES; ++s) {
                    float poseOffsetZ = 0.0f;
                    if (s == 0) poseOffsetZ = mPoseCurrent.z;
                    else if (s == 1) poseOffsetZ = mBodyPoseCurrent.z;
                    float offsetZ = mSphereOffsetsBase[s] + poseOffsetZ;
                    sphereCenters[s] = mPosition + Vector3(0.0f, 0.0f, offsetZ);
                }
                mObjectCollisionCb(sphereCenters, mSphereRadii,
                                   NUM_SPHERES, mCellIdx, mIterContacts);
            }

            if (mIterContacts.empty())
                break; // No contacts — done

            // Push body center out of penetrating polygons.
            // De-duplicate by normal direction: when multiple spheres hit the same
            // wall (dot > 0.99), use only the maximum penetration depth. Without
            // this, 3 spheres hitting one wall would triple the push, causing
            // jittery over-correction on the next frame.
            mPushes.clear();
            for (const auto &c : mIterContacts) {
                bool merged = false;
                for (auto &p : mPushes) {
                    if (glm::dot(c.normal, p.first) > 0.99f) {
                        // Same wall — keep the deeper penetration
                        p.second = std::max(p.second, c.penetration);
                        merged = true;
                        break;
                    }
                }
                if (!merged) {
                    mPushes.push_back({c.normal, c.penetration});
                }
            }
            for (const auto &p : mPushes) {
                mPosition += p.first * p.second;
            }

            // Accumulate contacts for ground detection and velocity removal
            mLastContacts.insert(mLastContacts.end(),
                                 mIterContacts.begin(), mIterContacts.end());

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

        // ── Contact persistence: merge newly detected contacts into persistent set ──
        // Match by polygon identity (cellIdx + polyIdx). Matched contacts get their
        // age reset to 0 and normal/penetration refreshed. Unmatched new contacts
        // are added. Stale contacts (not re-detected within CONTACT_MAX_AGE frames)
        // are culled.
        for (const auto &nc : mLastContacts) {
            bool matched = false;
            for (auto &pc : mPersistentContacts) {
                if (pc.cellIdx == nc.cellIdx && pc.polyIdx == nc.polyIdx) {
                    // Same polygon — refresh with latest detection data
                    pc.normal = nc.normal;
                    pc.penetration = nc.penetration;
                    pc.textureIdx = nc.textureIdx;
                    pc.age = 0;
                    matched = true;
                    break;
                }
            }
            if (!matched) {
                SphereContact pc = nc;
                pc.age = 0;
                mPersistentContacts.push_back(pc);

                // ── Damped bounce impulse on first contact with object OBBs ──
                // The original engine applies a small elastic bounce (0.02 scale)
                // when the player first collides with an object. This makes the
                // response feel slightly elastic rather than perfectly rigid, but
                // the 0.02 factor is nearly imperceptible — the player barely
                // bounces off crates/furniture. WR polygon contacts (cellIdx >= 0)
                // are constraint-only with no bounce (walls feel perfectly rigid).
                //
                // ODE UPGRADE: This impulse would be replaced by ODE contact joint
                // surface parameters (dContact.surface.bounce = 0.02,
                // dContact.surface.bounce_vel = kBreakObjectContactVel).
                if (nc.cellIdx == -1) {
                    float dp = glm::dot(mVelocity, nc.normal);
                    if (dp < 0.0f) {
                        // Remove normal component (dp < 0 = into surface) and
                        // reflect 2% back out. Factor = 1.0 (zero) + 0.02 (bounce).
                        constexpr float kBounceScale = 0.02f;  // matches kOBBBounceScale
                        mVelocity -= nc.normal * dp * (1.0f + kBounceScale);
                    }
                }
            }
        }

        // Cull stale contacts not re-detected within the persistence window
        mPersistentContacts.erase(
            std::remove_if(mPersistentContacts.begin(), mPersistentContacts.end(),
                           [](const SphereContact &c) {
                               return c.age > CONTACT_MAX_AGE;
                           }),
            mPersistentContacts.end());

        // Feed persistent contacts to all downstream consumers (detectGround,
        // computeGroundFriction, constrainVelocity). This replaces the transient
        // mLastContacts with the richer persistent set that includes both fresh
        // contacts (age=0) and recently-maintained contacts (age 1..MAX_AGE).
        mLastContacts = mPersistentContacts;

        // Fire contact callbacks for downstream consumers (audio, AI, scripts).
        // Only fire for freshly detected contacts (age=0) to prevent duplicate
        // events from persistent contacts that weren't re-detected this frame.
        // Contact point is approximate — we don't track which submodel generated
        // each contact, so we use the HEAD/BODY radius as the offset.
        if (contactCb) {
            for (const auto &c : mLastContacts) {
                if (c.age > 0) continue;  // skip persistent-only contacts
                ContactEvent event;
                event.bodyA = -1; // player entity (archetype ID, placeholder)
                event.bodyB = 0;  // world geometry
                event.point = mPosition - c.normal * (mSphereRadii[0] - c.penetration);
                event.normal = c.normal;
                event.depth = c.penetration;
                event.materialA = -1;
                event.materialB = c.textureIdx;
                contactCb(event);
            }
        }

    }

    /// Try to step up a small ledge — 3-phase raycast algorithm.
    ///
    /// Called after resolveCollisions() when the player is on the ground and
    /// has contacts from FOOT/SHIN/KNEE submodels with steep/vertical normals
    /// (normal.z < STEP_WALL_THRESHOLD). These contacts indicate the player
    /// hit a wall-like surface at leg level — potentially a steppable ledge.
    ///
    ///   Phase 1 — UP: Probe upward from foot to check headroom above the ledge.
    ///             If blocked by ceiling/geometry, step is too tall.
    ///   Phase 2 — FWD: Probe forward (along horizontal velocity) at the lifted
    ///             height. If blocked, ledge is too narrow or wall continues.
    ///   Phase 3 — DOWN: Probe downward from the lifted+forward position to find
    ///             the ledge surface. If no surface found, it's a gap.
    ///
    /// On success, lifts the player by max(STEP_MIN_ZDELTA, hit_z - foot_z) +
    /// STEP_CLEARANCE, then validates the BODY sphere at the new height.
    /// If BODY collides at the stepped position, the step is aborted.
    ///
    /// Returns true if the step was applied (position updated).
    inline bool tryStairStep() {
        // Only attempt while grounded and moving horizontally
        if (!isOnGround())
            return false;

        // Use the player's INTENDED movement direction/speed, not mVelocity.
        // constrainVelocity() (step 7) removes velocity into the riser wall
        // before we get here, so mVelocity's horizontal component is often ~0.
        // The desired velocity from input is unaffected by wall constraints.
        Vector3 desired = computeDesiredVelocity();
        Vector3 hDesired(desired.x, desired.y, 0.0f);
        float hSpeed = glm::length(hDesired);
        if (hSpeed < 0.1f)
            return false;

        Vector3 moveDir = hDesired / hSpeed;

        // Check if any leg-level contact has a steep/vertical normal (wall-like
        // surface). Head & Body wall contacts (from walls above step height) should NOT
        // trigger stepping. No directional filter — the forward probe handles that.
        // Foot position — the lowest point of the player model
        float footOffsetZ = mSphereOffsetsBase[4]; // FOOT = -3.0
        Vector3 footPos = mPosition + Vector3(0.0f, 0.0f, footOffsetZ);

        bool hasWallContact = false;
        Vector3 bestWallNormal(0.0f);
        for (const auto &c : mLastContacts) {
            // Only trigger on FACE contacts from leg-level terrain submodels.
            // The Dark Engine distinguishes face, edge, and vertex contact types
            // and only triggers stair stepping on face contacts. Edge contacts
            // at polygon boundaries (e.g. wall-slope intersections) have ambiguous
            // normals and can cause spurious stepping on slopes.
            // Object contacts (submodelIdx=-1) are excluded — stepping over
            // world objects requires additional validation (object type, climbable
            // surface checks) that is not yet implemented.
            if (std::fabs(c.normal.z) < STEP_WALL_THRESHOLD
                && c.submodelIdx >= 2
                && !c.isEdge) {
                hasWallContact = true;
                bestWallNormal = c.normal;
                break;
            }
        }

        // Fallback: if no wall contact in mLastContacts, cast a short forward
        // ray from the FOOT position to detect step risers that the collision
        // system missed (e.g. point detector at polygon edge gap). This is a
        // safety net for Fix #2's edge detection which still has coverage gaps.
        if (!hasWallContact) {
            RayHit fwdProbe;
            Vector3 fwdProbeEnd = footPos + moveDir * SPHERE_RADIUS; // 1.2 units ahead
            if (raycastWorld(mCollision.getWR(), footPos, fwdProbeEnd, fwdProbe)
                && std::fabs(fwdProbe.normal.z) < STEP_WALL_THRESHOLD) {
                hasWallContact = true;
                bestWallNormal = fwdProbe.normal;
            }
        }
        if (!hasWallContact)
            return false;

        if (mStepLog) {
            std::fprintf(stderr, "[STEP] ── attempt ── pos=(%.2f,%.2f,%.2f) footZ=%.2f "
                "hSpeed=%.2f dir=(%.2f,%.2f) wallN=(%.2f,%.2f,%.2f) contacts=%zu\n",
                mPosition.x, mPosition.y, mPosition.z, footPos.z,
                hSpeed, moveDir.x, moveDir.y,
                bestWallNormal.x, bestWallNormal.y, bestWallNormal.z,
                mLastContacts.size());
            // Dump per-contact details with submodel index for diagnostics
            for (size_t ci = 0; ci < mLastContacts.size(); ++ci) {
                const auto &c = mLastContacts[ci];
                std::fprintf(stderr, "[STEP]   contact[%zu]: sub=%d n=(%.2f,%.2f,%.2f) pen=%.3f cell=%d poly=%d\n",
                    ci, (int)c.submodelIdx,
                    c.normal.x, c.normal.y, c.normal.z,
                    c.penetration, c.cellIdx, c.polyIdx);
            }
        }

        // ── Phase 1: Raycast UP ──
        // Cast from foot upward by STEP_UP_DIST (2.0). The ray traverses
        // portal boundaries, potentially entering the stair cell above.
        // We capture the terminal cell — the cell the ray ended up in after
        // portal traversal — and pass it as a hint to subsequent phases.
        //
        // Start the ray slightly above the foot (STEP_UP_EPSILON = 0.01).
        // The point detectors maintain a small separation
        // from surfaces; our collision resolves to exactly dist=0, so
        // the UP ray self-intersects at t=0 with the floor polygon.
        // This minimal offset (0.01 units) reproduces the original
        // engine's natural skin behavior without reducing headroom.
        Vector3 upStart = footPos + Vector3(0.0f, 0.0f, STEP_UP_EPSILON);
        Vector3 upPos = footPos + Vector3(0.0f, 0.0f, STEP_UP_DIST);
        RayHit stepHit;
        int32_t cellHint = -1;

        if (raycastWorld(mCollision.getWR(), upStart, upPos, stepHit, &cellHint)) {
            if (mStepLog) std::fprintf(stderr, "[STEP]   P1 UP FAIL: hit at Z=%.2f (ceil)\n",
                stepHit.point.z);
            return false; // ceiling too low
        }

        // ── Phase 2: Raycast FORWARD ──
        // Probe forward at the lifted height to check clearance above the
        // step ledge. The forward distance is velocity * 0.01. This works because
        // the cell hint from Phase 1 already places us in the stair cell.
        float fwdDist = hSpeed * STEP_FWD_SCALE;
        Vector3 fwdPos = upPos + moveDir * fwdDist;

        int32_t fwdCellHint = cellHint;
        if (raycastWorld(mCollision.getWR(), upPos, fwdPos, stepHit, &fwdCellHint, cellHint)) {
            if (mStepLog) std::fprintf(stderr, "[STEP]   P2 FWD FAIL: hit at (%.2f,%.2f,%.2f) fwdDist=%.4f\n",
                stepHit.point.x, stepHit.point.y, stepHit.point.z, fwdDist);
            return false; // wall in the way at the lifted height
        }

        if (mStepLog) {
            std::fprintf(stderr, "[STEP]   P1+P2 OK: fwdDist=%.4f fwdPos=(%.2f,%.2f,%.2f) "
                "cellHint=%d->%d\n",
                fwdDist, fwdPos.x, fwdPos.y, fwdPos.z, cellHint, fwdCellHint);
        }

        // ── Phase 3: Raycast DOWN ──
        // Cast straight down from the lifted+forward position to find the
        // step tread surface - propagates
        // cell context through all 3 phases via Location hints. Our BFS
        // raycast can't traverse vertical portals with a vertical ray
        // (denom ≈ 0 for parallel normal/direction), so when the hint
        // points to a tiny riser cell, the DOWN ray gets stuck there.
        //
        // To match the original behavior: try with the propagated hint
        // first (fast path), then fall back to findCameraCell(fwdPos)
        // which determines the correct containing cell geometrically.
        // The cast distance (4.0 = 2 × STEP_UP_DIST) handles downward steps as well.
        static constexpr float STEP_DOWN_DIST = 4.0f;
        Vector3 downTarget = fwdPos - Vector3(0.0f, 0.0f, STEP_DOWN_DIST);

        if (!raycastWorld(mCollision.getWR(), fwdPos, downTarget, stepHit, nullptr, fwdCellHint)) {
            // Hint-based raycast failed — the hint likely points to a
            // riser cell with only vertical portals. Retry using
            // findCameraCell to locate the correct cell for fwdPos.
            if (!raycastWorld(mCollision.getWR(), fwdPos, downTarget, stepHit)) {
                if (mStepLog) std::fprintf(stderr, "[STEP]   P3 DOWN FAIL: no surface found (hint=%d, cell-find also failed)\n",
                    fwdCellHint);
                return false;
            }
            if (mStepLog) std::fprintf(stderr, "[STEP]   P3 DOWN: cell-find fallback succeeded\n");
        }

        float hitZ = stepHit.point.z;
        Vector3 stepGroundNormal = stepHit.normal;
        bool foundGround = (stepGroundNormal.z > GROUND_NORMAL_MIN);

        if (mStepLog) {
            std::fprintf(stderr, "[STEP]   P3 DOWN: hitZ=%.3f n=(%.2f,%.2f,%.2f) dist=%.3f "
                "ground=%d hint=%d\n",
                hitZ, stepGroundNormal.x, stepGroundNormal.y, stepGroundNormal.z,
                stepHit.distance, (int)foundGround, fwdCellHint);
        }

        if (!foundGround) {
            if (mStepLog) std::fprintf(stderr, "[STEP]   P3 REJECT: hit surface is not ground (nZ=%.2f)\n",
                stepGroundNormal.z);
            return false;
        }

        // Compute lift: z_delta = max(0.3, hit_z - foot_z) + 0.02
        float zDelta = hitZ - footPos.z;
        if (zDelta < STEP_MIN_ZDELTA)
            zDelta = STEP_MIN_ZDELTA;
        zDelta += STEP_CLEARANCE;

        // Don't step if the lift would exceed the probe range
        if (zDelta > STEP_UP_DIST + STEP_CLEARANCE) {
            if (mStepLog) std::fprintf(stderr, "[STEP]   REJECT: zDelta=%.3f > max=%.3f (too high)\n",
                zDelta, STEP_UP_DIST + STEP_CLEARANCE);
            return false;
        }

        if (mStepLog) {
            std::fprintf(stderr, "[STEP]   P3 OK: hitZ=%.2f zDelta=%.3f (raw=%.3f clamped to min=%.1f +clearance=%.2f)\n",
                hitZ, zDelta, hitZ - footPos.z, STEP_MIN_ZDELTA, STEP_CLEARANCE);
        }

        // ── Validate: test BODY sphere at the stepped position ──
        // Effectively only BODY is validated at the stepped height.
        // We validate BODY (index 1) to ensure it fits at the new height.
        Vector3 steppedPos = mPosition;
        steppedPos.z += zDelta;

        int32_t steppedCell = mCollision.findCell(steppedPos);
        if (steppedCell < 0) {
            if (mStepLog) std::fprintf(stderr, "[STEP]   VALIDATE FAIL: steppedPos=(%.2f,%.2f,%.2f) no cell\n",
                steppedPos.x, steppedPos.y, steppedPos.z);
            return false;
        }

        {
            float bodyOffsetZ = mSphereOffsetsBase[1] + mBodyPoseCurrent.z;
            Vector3 bodyCenter = steppedPos + Vector3(0.0f, 0.0f, bodyOffsetZ);

            mStepScratchContacts.clear();
            mCollision.sphereVsCellPolygons(bodyCenter, SPHERE_RADIUS,
                                             steppedCell, mStepScratchContacts);
            int32_t bodyCell = mCollision.findCell(bodyCenter);
            if (bodyCell >= 0 && bodyCell != steppedCell) {
                mCollision.sphereVsCellPolygons(bodyCenter, SPHERE_RADIUS,
                                                 bodyCell, mStepScratchContacts);
            }

            for (const auto &c : mStepScratchContacts) {
                // Wall or ceiling collision at the stepped position — abort.
                // Allow floor contacts since we expect to rest on the step.
                if (c.normal.z < STEP_WALL_THRESHOLD && c.penetration > 0.01f) {
                    if (mStepLog) {
                        std::fprintf(stderr, "[STEP]   VALIDATE FAIL: BODY collision n=(%.2f,%.2f,%.2f) pen=%.3f "
                            "bodyCenter=(%.2f,%.2f,%.2f)\n",
                            c.normal.x, c.normal.y, c.normal.z, c.penetration,
                            bodyCenter.x, bodyCenter.y, bodyCenter.z);
                    }
                    return false;
                }
            }

            if (mStepLog) {
                std::fprintf(stderr, "[STEP]   VALIDATE OK: bodyCenter=(%.2f,%.2f,%.2f) bodyCell=%d contacts=%zu\n",
                    bodyCenter.x, bodyCenter.y, bodyCenter.z, bodyCell >= 0 ? bodyCell : steppedCell,
                    mStepScratchContacts.size());
            }
        }

        // ── Validate HEAD sphere at its CURRENT position ──
        // HEAD validation tests at the current world position, not the stepped position.
        // This is correct because HEAD won't actually move during the step; the
        // spring will gradually pull it up over subsequent frames.
        {
            float headOffsetZ = mSphereOffsetsBase[0] + mPoseCurrent.z;
            Vector3 headCenter = mPosition + Vector3(0.0f, 0.0f, headOffsetZ);

            mStepScratchContacts.clear();
            mCollision.sphereVsCellPolygons(headCenter, SPHERE_RADIUS,
                                             steppedCell, mStepScratchContacts);
            int32_t headCell = mCollision.findCell(headCenter);
            if (headCell >= 0 && headCell != steppedCell) {
                mCollision.sphereVsCellPolygons(headCenter, SPHERE_RADIUS,
                                                 headCell, mStepScratchContacts);
            }

            for (const auto &c : mStepScratchContacts) {
                // Wall or ceiling collision at the stepped position — abort.
                // Same rejection criteria as BODY: floor contacts are OK.
                if (c.normal.z < STEP_WALL_THRESHOLD && c.penetration > 0.01f) {
                    if (mStepLog) {
                        std::fprintf(stderr, "[STEP]   VALIDATE FAIL: HEAD collision n=(%.2f,%.2f,%.2f) pen=%.3f "
                            "headCenter=(%.2f,%.2f,%.2f)\n",
                            c.normal.x, c.normal.y, c.normal.z, c.penetration,
                            headCenter.x, headCenter.y, headCenter.z);
                    }
                    return false;
                }
            }

            if (mStepLog) {
                std::fprintf(stderr, "[STEP]   VALIDATE OK: headCenter=(%.2f,%.2f,%.2f) headCell=%d contacts=%zu\n",
                    headCenter.x, headCenter.y, headCenter.z, headCell >= 0 ? headCell : steppedCell,
                    mStepScratchContacts.size());
            }
        }

        // ── Accept the step ──
        if (mStepLog) {
            std::fprintf(stderr, "[STEP]   ACCEPTED: pos (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f) "
                "zDelta=%.3f cell %d->%d\n",
                mPosition.x, mPosition.y, mPosition.z,
                steppedPos.x, steppedPos.y, steppedPos.z,
                zDelta, mCellIdx, steppedCell);
        }

        mPosition = steppedPos;
        mCellIdx = steppedCell;
        mVelocity.z = 0.0f;
        mGroundNormal = stepGroundNormal;

        // HEAD spring compensation — keep camera at its current world position
        // and let the spring smoothly pull it up to the new equilibrium.
        // Since eye_z = mPosition.z + HEAD_OFFSET_Z + EYE_ABOVE_HEAD + mSpringPos.z,
        // offsetting mSpringPos.z by -zDelta cancels the position change for
        // the camera. The spring then naturally blends back toward mPoseCurrent.
        mSpringPos.z -= zDelta;

        // Destroy all terrain contacts on KNEE/SHIN/FOOT (submodels 2-4) and
        // create a new FOOT contact at the step surface. This matches the
        // DestroyAllTerrainContacts for the three leg sub-models, then
        // CreateTerrainContact on FOOT with the discovered surface polygon.
        mLastContacts.erase(
            std::remove_if(mLastContacts.begin(), mLastContacts.end(),
                           [](const SphereContact &c) {
                               return c.submodelIdx >= 2 || c.submodelIdx < 0;
                           }),
            mLastContacts.end());

        // Synthesize a new FOOT ground contact at the step surface.
        // This gives the next frame correct ground
        // state without requiring a new collision pass.
        {
            SphereContact footContact;
            footContact.normal = stepGroundNormal;
            footContact.penetration = 0.01f; // nominal contact
            footContact.cellIdx = steppedCell;
            footContact.polyIdx = -1; // synthetic (no exact poly from raycast)
            footContact.textureIdx = stepHit.textureIndex;
            footContact.age = 0;
            footContact.submodelIdx = 4; // FOOT
            mLastContacts.push_back(footContact);
        }

        // Mode transition — forces Stand mode after a
        // successful step unless the player is already in Stand, Crouch, Swim,
        // or Dead. This prevents stepping while in Climb, Slide, etc.
        if (mCurrentMode != PlayerMode::Stand &&
            mCurrentMode != PlayerMode::Crouch &&
            mCurrentMode != PlayerMode::Swim &&
            mCurrentMode != PlayerMode::Dead) {
            mCurrentMode = PlayerMode::Stand;
        }

        // Break climb state — always calls BreakClimb
        // after a successful step to cancel any active climbing.
        if (mCurrentMode == PlayerMode::Climb) {
            mCurrentMode = PlayerMode::Stand;
        }

        return true;
    }

    /// Detect ground contact and update movement state.
    /// Any contact with an upward-facing normal (z > GROUND_NORMAL_MIN) counts
    /// as ground. Movement control on steep slopes is handled by the friction
    /// formula (friction = 0.03 * normal.z * gravity) — this only determines
    /// ground/air state.
    /// Ground probe uses FOOT sphere position for accurate floor detection.
    inline void detectGround() {
        bool onGround = false;

        // Check contacts from collision resolution (any sphere's ground contact counts).
        // Track the most upward-facing ground normal and its texture index for
        // footstep material lookup (Phase 3 Audio).
        float bestGroundZ = -1.0f;
        for (const auto &c : mLastContacts) {
            if (c.normal.z > GROUND_NORMAL_MIN) {
                onGround = true;
                if (c.normal.z > bestGroundZ) {
                    bestGroundZ = c.normal.z;
                    mGroundNormal = c.normal;
                    mGroundTextureIdx = c.textureIdx;
                }
            }
        }

        // If no contacts found from collision AND the player is not ascending,
        // probe along the last known ground normal to detect ground. This
        // prevents briefly entering Jump mode when walking over small bumps or
        // down slopes where the collision pass didn't generate contacts. The
        // probe only DETECTS ground (updates state), it doesn't move the player
        // — gravity and collision handle the actual positioning.
        //
        // The probe direction follows the inverse of the last known ground
        // normal rather than always probing straight down. On slopes, this
        // follows the surface, giving the full 0.1-unit probe reach along the
        // surface normal. A straight-down probe on a 45° slope only reaches
        // 0.07 effective units along the normal. Falls back to -Z when no
        // valid ground normal exists (e.g. first frame after level load).
        //
        // Skip when velocity.z > 0.5 to avoid cancelling a jump (the probe
        // would find the floor just left).
        if (!onGround && mCellIdx >= 0 && mVelocity.z <= 0.5f) {
            // FOOT stays at its standing offset during crouch (no crouch drop for legs)
            float footOffset = mSphereOffsetsBase[4];
            Vector3 footCenter = mPosition + Vector3(0.0f, 0.0f, footOffset);
            Vector3 probeNormal;
            int32_t probeTextureIdx = -1;
            constexpr float GROUND_PROBE_DIST = 0.1f;
            float footRadius = mSphereRadii[4]; // 0.0 (point detector)

            // Probe along the inverse of the last known ground normal.
            // On flat ground this is (0,0,-1), identical to the old behavior.
            Vector3 probeDir = -mGroundNormal;
            // Safety: if mGroundNormal has been corrupted or is near-horizontal,
            // fall back to straight down
            if (mGroundNormal.z < GROUND_NORMAL_MIN) {
                probeDir = Vector3(0.0f, 0.0f, -1.0f);
            }

            if (mCollision.groundTest(footCenter, footRadius, mCellIdx,
                                      GROUND_PROBE_DIST, probeDir,
                                      probeNormal, probeTextureIdx,
                                      mGroundProbeContacts)) {
                if (probeNormal.z > GROUND_NORMAL_MIN) {
                    onGround = true;
                    mGroundNormal = probeNormal;
                    mGroundTextureIdx = probeTextureIdx;
                }
            }
        }

        // FOOT contact recovery — when the player is on ground but FOOT
        // (submodel 4) has no ground contact in mLastContacts, probe downward
        // from the FOOT position to re-acquire the tread below. This handles
        // the transition between stair treads where FOOT briefly loses contact
        // as it moves between adjacent polygons. The synthesized contact enables
        // tryStairStep() to detect the next riser contact properly.
        if (onGround) {
            bool footHasGround = false;
            for (const auto &c : mLastContacts) {
                if (c.submodelIdx == 4 && c.normal.z > GROUND_NORMAL_MIN) {
                    footHasGround = true;
                    break;
                }
            }
            if (!footHasGround) {
                float footOffset = mSphereOffsetsBase[4]; // -3.0
                Vector3 footCenter = mPosition + Vector3(0.0f, 0.0f, footOffset);
                Vector3 probeEnd = footCenter - Vector3(0.0f, 0.0f, 0.5f); // 0.5 units down
                RayHit footProbe;
                if (raycastWorld(mCollision.getWR(), footCenter, probeEnd, footProbe)
                    && footProbe.normal.z > GROUND_NORMAL_MIN) {
                    SphereContact fc;
                    fc.normal = footProbe.normal;
                    fc.penetration = 0.01f;
                    fc.cellIdx = mCellIdx;
                    fc.polyIdx = -1; // synthetic contact
                    fc.textureIdx = footProbe.textureIndex;
                    fc.submodelIdx = 4; // FOOT
                    mLastContacts.push_back(fc);
                }
            }
        }

        // Update movement state — mode transitions for ground/air
        if (onGround) {
            // Reset grace state whenever ground is positively detected
            mGroundGraceTimer = 0.0f;
            mGroundGraceActive = false;

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

                // If landing in water, go directly to Swim mode to prevent
                // 1-frame Stand flicker (detectGround runs before updateModeTransitions).
                if (isInWater()) {
                    mCurrentMode = PlayerMode::Swim;
                } else {
                    // Return to crouch or stand based on input.
                    // Activate the appropriate rest pose so the head spring drives
                    // toward the correct height (POSE_CROUCH = -2.02 or POSE_NORMAL = 0).
                    mCurrentMode = mWantsCrouch ? PlayerMode::Crouch : PlayerMode::Stand;
                    if (!mLandingActive && mLeanDir == 0) {
                        activatePose(*getModeMotion(mCurrentMode).restPose);
                    }
                }
            }
            // If already in a ground mode, stay in it (crouch transitions
            // are handled by updateModeTransitions)
        } else {
            if (isOnGround()) {
                // Ground lost while in a ground mode — apply grace period.
                // This prevents single-frame air transitions on bumps and slopes
                // where the collision pass and probe both fail momentarily.
                //
                // During grace, gravity still applies (see applyGravity) so
                // walking off a ledge produces natural fall behavior. The grace
                // only affects mode state — the player stays in Stand/Crouch
                // for movement friction and stride bob continuity.
                if (mGroundGraceTimer <= 0.0f) {
                    // First frame without ground — start the grace timer
                    mGroundGraceTimer = GROUND_GRACE_DURATION;
                    mGroundGraceActive = true;
                } else {
                    // Grace counting down
                    mGroundGraceTimer -= mTimestep.fixedDt;
                    if (mGroundGraceTimer <= 0.0f) {
                        // Grace expired — transition to Jump
                        mGroundGraceTimer = 0.0f;
                        mGroundGraceActive = false;
                        mCurrentMode = PlayerMode::Jump;
                        if (mLeanDir == 0)
                            activatePose(POSE_NORMAL);
                        mStrideDist = 0.0f;
                    }
                    // else: still in grace period, stay in ground mode
                }
            }
            // If already in Jump mode, stay in Jump mode
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
            float sinYaw = mSinYaw;
            float cosYaw = mCosYaw;
            // HEAD collision position for lean: standing offset + pose-driven drop
            float headOffset = mSphereOffsetsBase[0] + mPoseCurrent.z;
            float leanBaseZ = headOffset + mSpringPos.z;
            Vector3 leanedHead = mPosition + Vector3(0.0f, 0.0f, leanBaseZ);
            leanedHead.x += sinYaw * mLeanAmount;
            leanedHead.y -= cosYaw * mLeanAmount;

            // Test HEAD sphere at the leaned position
            std::vector<SphereContact> contacts;
            mCollision.sphereVsCellPolygons(
                leanedHead, mSphereRadii[0], mCellIdx, contacts);
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

    /// Test whether the player model can fit at a given position without
    /// penetrating world geometry. Tests all 5 submodels (2 real spheres +
    /// 3 point detectors) at their standing offsets against cell polygons.
    ///
    /// Returns true if no submodel has a penetrating contact at testPos.
    inline bool canPlayerFitAt(const Vector3 &testPos) const {
        int32_t testCell = mCollision.findCell(testPos);
        if (testCell < 0)
            return false;  // outside all cells — not valid

        std::vector<SphereContact> contacts;
        for (int s = 0; s < NUM_SPHERES; ++s) {
            float sphereR = mSphereRadii[s];
            float offsetZ = mSphereOffsetsBase[s];  // standing offsets (not crouched)
            Vector3 sphereCenter = testPos + Vector3(0.0f, 0.0f, offsetZ);

            contacts.clear();
            mCollision.sphereVsCellPolygons(sphereCenter, sphereR, testCell, contacts);

            // Also check the submodel's own cell if different
            int32_t sphereCell = mCollision.findCell(sphereCenter);
            if (sphereCell >= 0 && sphereCell != testCell) {
                mCollision.sphereVsCellPolygons(sphereCenter, sphereR, sphereCell, contacts);
            }

            if (!contacts.empty())
                return false;  // penetration detected — player doesn't fit
        }

        return true;
    }

    /// Check for mantleable ledge using 3-ray detection.
    /// Called each fixedStep when the player is airborne and pressing forward.
    ///
    /// Detection algorithm:
    /// 1. Cast UP from head — must NOT hit (headroom check, 3.5 units)
    /// 2. Cast FORWARD from top — must NOT hit (clear path, radius×2 = 2.4)
    /// 3. Cast DOWN from forward position — MUST hit (ledge surface, 7.0 units)
    /// 4. Find forward surface by casting forward from HEAD/BODY/FOOT
    ///    (determines XY target, pulled back 0.55 from wall)
    ///
    /// Rise target Z places body center at an INTERMEDIATE height — just enough
    /// for the head sphere to peek above the ledge lip. The full standing height
    /// is reached later via the FinalRise phase after forward compression.
    ///
    /// riseZ = ledge.z + radius*1.02 - HEAD_OFFSET + 1.0
    inline bool checkMantle() {
        if (mCurrentMode != PlayerMode::Jump || mInputForward < 0.1f || mMantling)
            return false;

        // HEAD position for mantle check — apply pose-driven drop (smooth during crouch)
        float headZ = mSphereOffsetsBase[0] + mPoseCurrent.z;
        Vector3 headPos = mPosition + Vector3(0.0f, 0.0f, headZ);

        // Phase 1: upward — check headroom (3.5 units above head)
        Vector3 upTarget = headPos + Vector3(0.0f, 0.0f, MANTLE_UP_DIST);
        RayHit hit;
        if (raycastWorld(mCollision.getWR(), headPos, upTarget, hit))
            return false;  // ceiling too low

        // Forward direction (horizontal, unit length, from cached yaw)
        Vector3 fwd(mCosYaw, mSinYaw, 0.0f);
        Vector3 moveDir = fwd * MANTLE_FWD_DIST;  // radius × 2 = 2.4 units

        // Phase 2: forward — check clear path above the ledge
        Vector3 fwdTarget = upTarget + moveDir;
        if (raycastWorld(mCollision.getWR(), upTarget, fwdTarget, hit))
            return false;  // wall in the way

        // Phase 3: downward — find ledge surface (7.0 units down)
        Vector3 downTarget = fwdTarget + Vector3(0.0f, 0.0f, -MANTLE_DOWN_DIST);
        if (!raycastWorld(mCollision.getWR(), fwdTarget, downTarget, hit))
            return false;  // no ledge found

        float ledgeZ = hit.point.z;

        // Phase 4: Find forward surface to determine XY target.
        // Casts forward from HEAD, then BODY, then FOOT (with fallbacks)
        // to find the wall face. The target XY is pulled back from the wall by 0.55.
        // This ensures the player ends up close to (but not inside) the wall.
        Vector3 fwdHitPoint;
        bool foundFwdSurface = false;

        // Try HEAD → BODY → FOOT forward ray to find wall surface
        const float fwdSurfaceOffsets[3] = {
            mSphereOffsetsBase[0], mSphereOffsetsBase[1], mSphereOffsetsBase[4]
        };
        for (int i = 0; i < 3 && !foundFwdSurface; ++i) {
            Vector3 spherePos = mPosition + Vector3(0.0f, 0.0f, fwdSurfaceOffsets[i]);
            Vector3 sphereFwd = spherePos + moveDir;
            if (raycastWorld(mCollision.getWR(), spherePos, sphereFwd, hit)) {
                fwdHitPoint = hit.point;
                foundFwdSurface = true;
            }
        }

        // Compute rise target — INTERMEDIATE height for head peek.
        // Z = ledge.z + (radius * 1.02) - HEAD_POS + 1.0
        // This places the body center so the head sphere peeks just above the ledge.
        // Head will be at riseZ + HEAD_OFFSET_Z = ledge.z + 0.424 + 1.8 = ledge.z + 2.224
        float riseZ = ledgeZ + (mSphereRadii[0] * 1.02f) - mSphereOffsetsBase[0] + 1.0f;

        if (foundFwdSurface) {
            // Pull back from wall by MANTLE_PULLBACK (0.55) in movement direction
            mMantleTarget = fwdHitPoint - fwd * MANTLE_PULLBACK;
            mMantleTarget.z = riseZ;
        } else {
            // Fallback: use body XY position as last resort.
            mMantleTarget = Vector3(mPosition.x, mPosition.y, riseZ);
        }

        // Validate: can the player fit at the target position?
        // Checks is_valid at the target, and also at
        // target + forward + down (standing on the ledge beyond the lip).
        // We test all 5 submodels (2 spheres + 3 point detectors) against
        // the cell geometry at the proposed position.
        Vector3 standingTarget = mMantleTarget;
        standingTarget.z = ledgeZ - mSphereOffsetsBase[4];  // final standing body center
        if (!canPlayerFitAt(standingTarget)) {
            return false;  // can't fit at target — abort mantle
        }

        // Also check one step forward and below (standing on the ledge surface).
        // delta = target + movement_dir, delta.z += -PLAYER_FOOT_POS
        Vector3 beyondTarget = standingTarget + moveDir;
        beyondTarget.z = ledgeZ - mSphereOffsetsBase[4];
        if (!canPlayerFitAt(beyondTarget)) {
            return false;  // can't fit standing on the ledge — abort mantle
        }

        startMantle();
        return true;
    }

    /// Begin the mantle animation — 5-state machine.
    /// Initializes the virtual head at the current HEAD position so the
    /// camera sees no discontinuity when mantling begins.
    inline void startMantle() {
        mMantling = true;
        mMantleState = MantleState::Hold;
        mMantleTimer = 0.0f;
        mMantleCompressed = false;
        // Virtual head starts at current head position
        mMantleHeadPos = mPosition + Vector3(0.0f, 0.0f, mSphereOffsetsBase[0]);
        mMantleHeadVel = Vector3(0.0f);
    }

    /// Update the mantle state machine each fixedStep.
    /// During mantling, normal movement/gravity/collision are suppressed.
    /// A virtual head position (mMantleHeadPos) is driven by the spring
    /// formula. The camera tracks this head, so body teleports at state
    /// transitions are invisible to the player.
    ///
    /// Uses computeSpringVelocity() with per-state tension and velocity
    /// multipliers to emulate spring-driven head submodel behavior during mantling.
    inline void updateMantle() {
        if (!mMantling) return;

        const float dt = mTimestep.fixedDt;
        mMantleTimer += dt;

        switch (mMantleState) {

        case MantleState::Hold:
            // State 1: Freeze position for MANTLE_HOLD_TIME (0.3s).
            // Spring holds head in place — target = current head position.
            // Body velocity zeroed.
            mVelocity = Vector3(0.0f);
            {
                // Spring holds head at current position (target = self)
                Vector3 holdTarget = mMantleHeadPos;
                mMantleHeadVel = computeSpringVelocity(
                    mMantleHeadPos, mMantleHeadVel, holdTarget,
                    HEAD_SPRING_BASE_TENSION, HEAD_SPRING_BASE_DAMPING,
                    HEAD_SPRING_Z_SCALE, HEAD_SPRING_VEL_CAP, dt);
                mMantleHeadPos += mMantleHeadVel * dt;
            }
            if (mMantleTimer >= MANTLE_HOLD_TIME) {
                mMantleState = MantleState::Rise;
                mMantleTimer = 0.0f;
            }
            break;

        case MantleState::Rise:
            // State 2: Spring drives head toward rise target with adaptive tension.
            // Head target = (mantleTarget.xy, riseZ + HEAD_OFFSET_Z) — head is
            // HEAD_OFFSET_Z above the body center target.
            // Tension modulated by inverse-distance-squared (faster far, slower near).
            mVelocity = Vector3(0.0f);
            {
                // Head target: body center target + head offset
                Vector3 headTarget(mMantleTarget.x, mMantleTarget.y,
                                   mMantleTarget.z + mSphereOffsetsBase[0]);

                // Adaptive tension: inverse-distance-squared modulation
                // Far away (d² > 1): tension decreases → natural acceleration
                // Close (d² < 1): tension = full base → smooth approach
                Vector3 diff = headTarget - mMantleHeadPos;
                float d2 = glm::dot(diff, diff);
                float tensionScale = (d2 > 1.0f) ? std::min(1.0f, 1.0f / d2) : 1.0f;
                float tension = tensionScale * HEAD_SPRING_BASE_TENSION;

                mMantleHeadVel = computeSpringVelocity(
                    mMantleHeadPos, mMantleHeadVel, headTarget,
                    tension, HEAD_SPRING_BASE_DAMPING,
                    HEAD_SPRING_Z_SCALE, HEAD_SPRING_VEL_CAP, dt);
                mMantleHeadPos += mMantleHeadVel * dt;

                // Check convergence — head reached rise target
                Vector3 remain = headTarget - mMantleHeadPos;
                if (glm::dot(remain, remain) < MANTLE_CONVERGE) {
                    // Rise→Forward transition: "compress" the player model.
                    // Body teleports to head position (invisible to camera since
                    // camera tracks mMantleHeadPos, not mPosition).
                    mMantleHeadPos = headTarget;  // snap head exactly
                    mPosition = mMantleHeadPos;   // body catches up to head
                    mMantleCompressed = true;

                    // Compute forward target: current position + facing × fwd distance.
                    // In compressed state, head = body (no offset).
                    Vector3 fwd(mCosYaw, mSinYaw, 0.0f);
                    mMantleTarget = mPosition + fwd * MANTLE_FWD_DIST;

                    // Reset spring velocity for fresh forward movement
                    mMantleHeadVel = Vector3(0.0f);

                    mMantleState = MantleState::Forward;
                    mMantleTimer = 0.0f;
                }
            }
            break;

        case MantleState::Forward:
            // State 3: "Compressed ball" moves forward onto the ledge (0.4s).
            // Spring with reduced tension (0.012) + 10× velocity multiplier.
            // Head = body during compressed state (no offset).
            mVelocity = Vector3(0.0f);
            {
                // Forward target — only XY matters, Z stays at current
                Vector3 fwdTarget = mMantleTarget;
                fwdTarget.z = mMantleHeadPos.z;  // hold Z constant

                // Reduced tension for forward phase (0.02 × base = 0.012)
                static constexpr float FWD_TENSION_SCALE = 0.02f;
                float tension = FWD_TENSION_SCALE * HEAD_SPRING_BASE_TENSION;

                mMantleHeadVel = computeSpringVelocity(
                    mMantleHeadPos, mMantleHeadVel, fwdTarget,
                    tension, HEAD_SPRING_BASE_DAMPING,
                    HEAD_SPRING_Z_SCALE, HEAD_SPRING_VEL_CAP, dt);

                // 10× velocity multiplier applied AFTER spring cap
                mMantleHeadVel *= 10.0f;
                mMantleHeadPos += mMantleHeadVel * dt;

                // Track body with head during compressed state
                mPosition = mMantleHeadPos;

                // Check convergence or timeout
                float dx = mMantleTarget.x - mMantleHeadPos.x;
                float dy = mMantleTarget.y - mMantleHeadPos.y;
                float hDist2 = dx * dx + dy * dy;

                if (mMantleTimer >= MANTLE_FWD_TIME || hDist2 < MANTLE_CONVERGE) {
                    // Forward→FinalRise transition.
                    // Snap body to forward target XY
                    mPosition.x = mMantleTarget.x;
                    mPosition.y = mMantleTarget.y;
                    mMantleHeadPos.x = mPosition.x;
                    mMantleHeadPos.y = mPosition.y;

                    // Restore head offset — camera jumps +1.8 (same as original
                    // engine's 3→4 transition where HEAD offset is restored).
                    mMantleHeadPos.z += mSphereOffsetsBase[0];

                    // FinalRise body target: body rises
                    // (-footOffset - headRadius - 1.0)
                    float rise2Height =
                        -mSphereOffsetsBase[4] - mSphereRadii[0] - 1.0f;
                    mMantleTarget = mPosition + Vector3(0.0f, 0.0f, rise2Height);

                    // Reset spring velocity for fresh rise
                    mMantleHeadVel = Vector3(0.0f);

                    mMantleState = MantleState::FinalRise;
                    mMantleTimer = 0.0f;
                }
            }
            break;

        case MantleState::FinalRise:
            // State 4: Spring drives head up to final standing height.
            // Same reduced tension as Forward + adaptive velocity multiplier
            // min(50, max(30, 30/d²)) applied AFTER spring cap.
            mVelocity = Vector3(0.0f);
            {
                // Head target = body target + head offset
                Vector3 headTarget = mMantleTarget + Vector3(0.0f, 0.0f, mSphereOffsetsBase[0]);

                // Same reduced tension as Forward phase
                static constexpr float RISE2_TENSION_SCALE = 0.02f;
                float tension = RISE2_TENSION_SCALE * HEAD_SPRING_BASE_TENSION;

                mMantleHeadVel = computeSpringVelocity(
                    mMantleHeadPos, mMantleHeadVel, headTarget,
                    tension, HEAD_SPRING_BASE_DAMPING,
                    HEAD_SPRING_Z_SCALE, HEAD_SPRING_VEL_CAP, dt);

                // Adaptive velocity multiplier: min(50, max(30, 30/d² (matching original order)
                Vector3 remain = headTarget - mMantleHeadPos;
                float d2 = glm::dot(remain, remain);
                float velMult = std::min(50.0f, std::max(30.0f, 30.0f / std::max(d2, 0.01f)));
                mMantleHeadVel *= velMult;

                mMantleHeadPos += mMantleHeadVel * dt;

                // Body tracks head minus offset during FinalRise
                mPosition.x = mMantleHeadPos.x;
                mPosition.y = mMantleHeadPos.y;
                mPosition.z = mMantleHeadPos.z - mSphereOffsetsBase[0];

                // Check convergence
                remain = headTarget - mMantleHeadPos;
                if (glm::dot(remain, remain) < MANTLE_CONVERGE) {
                    // Snap to target
                    mPosition = mMantleTarget;
                    mMantleHeadPos = headTarget;

                    // Raycast down to find actual floor surface
                    Vector3 rayFrom = mPosition;
                    Vector3 rayTo = mPosition;
                    rayTo.z += mSphereOffsetsBase[4] - 1.0f;  // scan below feet
                    RayHit floorHit;
                    if (raycastWorld(mCollision.getWR(), rayFrom, rayTo, floorHit)) {
                        // Place body center so foot is on floor + 0.1 margin
                        mPosition.z = floorHit.point.z - mSphereOffsetsBase[4] + 0.1f;
                    }

                    // Seed the normal head spring to bridge the camera transition.
                    // mSpringPos.z compensates so the normal eye formula produces the
                    // same eye height that the mantle head was at. The spring then
                    // naturally decays to 0, creating a smooth transition back to
                    // normal camera behavior.
                    mSpringPos.z = mMantleHeadPos.z - (mPosition.z + mSphereOffsetsBase[0]);

                    mMantleState = MantleState::Complete;
                    mMantleTimer = 0.0f;
                }
            }
            break;

        case MantleState::Complete:
            // State 5: Restore normal mode.
            // Run a collision resolution pass BEFORE returning to normal physics.
            // During mantling, the body is driven directly through geometry by the
            // spring-based state machine (no collision checks in states 1-4). If the
            // body ends up embedded in walls or ceilings, the first regular fixedStep
            // would detect deep penetration and produce a large position correction
            // that "launches" the player. Resolving here with a no-op contact callback
            // gently pushes the body out of any embedding.
            mMantling = false;
            mMantleCompressed = false;
            mMantleState = MantleState::None;
            mCurrentMode = mWantsCrouch ? PlayerMode::Crouch : PlayerMode::Stand;
            mVelocity = Vector3(0.0f);
            updateCell();
            resolveCollisions([](const ContactEvent&) {});
            updateCell();
            break;

        default:
            mMantling = false;
            mMantleCompressed = false;
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
        // The body center stays in place — no body center drop needed.
        // The pose system (POSE_CROUCH) drives both HEAD (mPoseCurrent.z → -2.02)
        // and BODY (mBodyPoseCurrent.z → -1.0) smoothly over the blend duration.
        // The collision system uses these blended offsets to physically lower
        // HEAD and BODY spheres for ceiling clearance.

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
                Vector3 standingHeadPos = mPosition + Vector3(0.0f, 0.0f, mSphereOffsetsBase[0]);
                std::vector<SphereContact> headContacts;
                mCollision.sphereVsCellPolygons(
                    standingHeadPos, mSphereRadii[0], mCellIdx, headContacts);

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
        Vector3 newBodyTarget(pose.bodyFwd, pose.bodyLat, pose.bodyVert);

        // Skip if already targeting the same offset (same-motion guard).
        // Check both HEAD and BODY targets — if both match, no re-activation needed.
        // Exception: if the pose just finished holding (poseReady), allow
        // re-activation so stride→rest→stride cycling works correctly.
        if (glm::length(mPoseEnd - newTarget) < 0.001f &&
            glm::length(mBodyPoseEnd - newBodyTarget) < 0.001f &&
            !mPoseHolding) {
            return;  // already blending toward this target
        }

        mPoseStart = mPoseCurrent;  // capture current HEAD position as blend origin
        mPoseEnd = newTarget;
        mBodyPoseEnd = newBodyTarget;
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

        // Fire footstep callback — Phase 3 Audio uses this for surface-appropriate
        // footstep sounds. Position is at the FOOT point detector (ground contact),
        // speed is horizontal velocity magnitude, material is the ground texture
        // index from the last collision pass.
        if (mFootstepCb) {
            Vector3 footPos = mPosition + Vector3(0.0f, 0.0f, mSphereOffsetsBase[4]);
            float hSpeed = std::sqrt(mVelocity.x * mVelocity.x +
                                     mVelocity.y * mVelocity.y);
            mFootstepCb(footPos, hSpeed, mGroundTextureIdx);
        }

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

        // Compute current interpolated pose offset for HEAD and BODY.
        // BODY shares the same timer/duration as HEAD.
        bool poseReady = false;  // true when current pose blend+hold is complete
        if (mPoseHolding) {
            // At target, holding — HEAD and BODY stay at their targets
            mPoseCurrent = mPoseEnd;
            mBodyPoseCurrent = mBodyPoseEnd;
            // Check if hold time expired → pose is ready for next transition
            if (mPoseTimer >= mPoseHoldTime) {
                poseReady = true;
            }
        } else if (mPoseDuration <= 0.0f) {
            // Instant pose (dur=0) — snap HEAD and BODY to target, start hold.
            // Used for landing impacts and weapon swings where the spring should
            // handle all smoothing from an instant displacement.
            mPoseCurrent = mPoseEnd;
            mBodyPoseCurrent = mBodyPoseEnd;
            mPoseHolding = true;
            mPoseTimer = 0.0f;
        } else {
            // Progressive blend for HEAD and BODY
            // Formula: curOffset += (targOffset - curOffset) * (timeActive / timeDuration)
            // This is NOT linear interpolation — it's a cumulative blend where each
            // step applies a fraction of the remaining distance. The fraction grows
            // with time (timeActive/timeDuration increases each step), creating an
            // accelerating approach curve that reaches the target at t=duration.
            // At the original 12.5Hz rate with 0.6s duration, this produces ~7 blend
            // steps with progressively larger increments.
            if (mPoseTimer >= mPoseDuration) {
                mPoseCurrent = mPoseEnd;
                mBodyPoseCurrent = mBodyPoseEnd;
                mPoseHolding = true;
                mPoseTimer = 0.0f;
            } else {
                float blendFrac = mPoseTimer / mPoseDuration;
                mPoseCurrent += (mPoseEnd - mPoseCurrent) * blendFrac;
                mBodyPoseCurrent += (mBodyPoseEnd - mBodyPoseCurrent) * blendFrac;
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
                // If leaning, re-activate the lean pose — gates
                // rest pose activation on is_leaning, so lean survives landing.
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
        // During mantling, camera tracks the spring-driven virtual head.
        // mMantleHeadPos is driven by the mantle spring each step — the camera
        // sees smooth motion while the body may teleport at state transitions.
        if (mMantling) {
            return mMantleHeadPos + Vector3(0.0f, 0.0f, EYE_ABOVE_HEAD);
        }

        // Head Z offset — always HEAD_OFFSET_Z. During crouch, the motion pose system
        // (POSE_CROUCH vert=-2.02) drives mSpringPos.z down, lowering the eye naturally.
        // No body center shift or offset scaling needed.
        Vector3 eye = mPosition + Vector3(0.0f, 0.0f, mSphereOffsetsBase[0] + EYE_ABOVE_HEAD + mSpringPos.z);

        // Use cached sin/cos (updated in setYaw() and fixedStep())
        float sinYaw = mSinYaw;
        float cosYaw = mCosYaw;

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

        // Compute spring velocity using the shared helper.
        // Uses the standard head spring parameters (tension=0.6, damping=0.02,
        // Z-scale=0.5, cap=25.0). The helper handles dt clamping internally.
        mSpringVel = computeSpringVelocity(
            mSpringPos, mSpringVel, target,
            HEAD_SPRING_BASE_TENSION, HEAD_SPRING_BASE_DAMPING,
            HEAD_SPRING_Z_SCALE, HEAD_SPRING_VEL_CAP, dt);

        // Integrate position (uses full dt, not clamped spring dt)
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
    float mCosYaw = 1.0f;         // cached cos(mYaw), updated per step and setYaw()
    float mSinYaw = 0.0f;         // cached sin(mYaw), updated per step and setYaw()
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

    // ── Mantle spring state ──
    // Virtual head position driven by the spring during mantling.
    // Camera tracks this instead of mPosition + HEAD_OFFSET_Z.
    Vector3 mMantleHeadPos{0.0f};      // spring-driven virtual head position
    Vector3 mMantleHeadVel{0.0f};      // virtual head velocity (spring state)
    bool    mMantleCompressed = false;  // true during Forward/FinalRise (offsets zeroed)

    // ── Overridable physics parameters (from P$PhysAttr/P$PhysDims or defaults) ──
    float mMass = PLAYER_MASS;
    float mDensity = 0.9f;          // buoyancy density — 0.9 (positive)
    float mElasticity = 0.0f;
    float mSphereRadii[NUM_SPHERES] = {
        SPHERE_RADIUS, SPHERE_RADIUS, 0.0f, 0.0f, 0.0f
    };
    float mSphereOffsetsBase[NUM_SPHERES] = {
        HEAD_OFFSET_Z, BODY_OFFSET_Z, SHIN_OFFSET_Z, KNEE_OFFSET_Z, FOOT_OFFSET_Z
    };

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
    Vector3 mPoseStart{0.0f};     // HEAD blend start offset (captured in activatePose)
    Vector3 mPoseEnd{0.0f};       // HEAD blend target offset {fwd, lat, vert}
    Vector3 mPoseCurrent{0.0f};   // HEAD current interpolated pose offset
    Vector3 mBodyPoseEnd{0.0f};       // BODY blend target offset {fwd, lat, vert}
    Vector3 mBodyPoseCurrent{0.0f};   // BODY current blended offset
    float mPoseTimer    = 0.0f;   // elapsed time in current blend/hold (shared HEAD+BODY)
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

    // Ground contact normal and texture from last collision pass
    Vector3 mGroundNormal{0.0f, 0.0f, 1.0f};
    int32_t mGroundTextureIdx = -1;  // texture index of ground surface (for footstep material)

    // Ground grace period state — prevents instant Jump transitions on bumps
    // and slopes where collision + probe both fail momentarily.
    float mGroundGraceTimer = 0.0f;     // countdown timer (seconds)
    bool  mGroundGraceActive = false;   // true during the grace window

    // Scratch buffer for groundTest() probe — avoids per-step heap allocation
    std::vector<SphereContact> mGroundProbeContacts;
    // Scratch buffers for tryStairStep() probes — avoids per-step heap allocation
    std::vector<SphereContact> mStepScratchContacts;
    std::vector<int32_t>       mStepBFSQueue;    // BFS cell queue for multi-hop portal traversal
    std::vector<bool>          mStepBFSVisited;  // visited flags for BFS (sized to numCells)

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

    // Contacts from last collision resolution (used by detectGround and velocity removal).
    // After the persistence merge, this contains both freshly detected contacts (age=0)
    // and still-valid persistent contacts (age 1..CONTACT_MAX_AGE).
    std::vector<SphereContact> mLastContacts;

    // Persistent contact buffer — survives across frames with aging. Contacts are
    // matched by (cellIdx, polyIdx) identity. Stale contacts are culled after
    // CONTACT_MAX_AGE frames without re-detection.
    std::vector<SphereContact> mPersistentContacts;

    // Pre-allocated scratch buffers for resolveCollisions() — avoids per-step heap allocs
    std::vector<SphereContact> mIterContacts;                // per-iteration contact accumulator
    std::vector<std::pair<Vector3, float>> mPushes;          // de-duplicated push normals

    // ── Callbacks ──
    FootstepCallback mFootstepCb;            // footstep sound event (Phase 3 Audio stub)
    ObjectCollisionCallback mObjectCollisionCb;  // player-vs-object collision (Task 26)

    // ── Diagnostic logging state ──
    FILE *mLogFile = nullptr;   // per-timestep CSV log file (null = disabled)
    bool  mStepLog = false;     // stair step diagnostics to stderr ([STEP] prefix)
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
            "%.5f,%.5f,%.5f,"              // bodyPoseX,bodyPoseY,bodyPoseZ
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
            mBodyPoseCurrent.x, mBodyPoseCurrent.y, mBodyPoseCurrent.z,
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
