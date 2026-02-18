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
 *    PlayerPhysicsConstants — enums, structs, constants, and static helpers
 *    for the player physics simulation. Separated from PlayerPhysics.h so
 *    that DarkPhysics.h and DarknessRender.cpp can reference types and
 *    presets without pulling in the full implementation.
 *
 *****************************************************************************/

#ifndef __PLAYERPHYSICSCONSTANTS_H
#define __PLAYERPHYSICSCONSTANTS_H

#include <algorithm>
#include <cmath>
#include "DarknessMath.h"

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

// ── Player physics constants ──

// 5-submodel player model (HEAD, BODY, SHIN, KNEE, FOOT) with vertical offsets. HEAD and BODY
// are real collision spheres (radius 1.2). SHIN/KNEE/FOOT are zero-radius point detectors —
// they sense floor/ground contacts but have no physical volume.

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

// Per-submodel collision radii. HEAD/BODY are real spheres that push the body away from walls.
// SHIN/KNEE/FOOT are point detectors (radius 0.0) — they participate in collision response
// (pushing the body up when they penetrate the floor) but don't inflate player width.
static constexpr float SPHERE_RADII[NUM_SPHERES] = {
    SPHERE_RADIUS,   // 0: HEAD   — real collision sphere
    SPHERE_RADIUS,   // 1: BODY   — real collision sphere
    0.0f,            // 2: SHIN   — point detector
    0.0f,            // 3: KNEE   — point detector
    0.0f,            // 4: FOOT   — point detector
};

// Crouch offsets are driven by the pose system (MotionPoseData). HEAD drops via mPoseCurrent.z
// (POSE_CROUCH.vert=-2.02) and BODY via mBodyPoseCurrent.z (bodyVert=-1.0), blending smoothly
// over the pose duration (0.8s). SHIN/KNEE/FOOT always use standing offsets (no pose offsets).
// Reference crouch offsets (in pose table): Crouch HEAD=-2.02/BODY=-1.0,
// CrawlLeft HEAD=-2.5/BODY=-1.0, CrawlRight HEAD=-2.5/BODY=-1.0.

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
// Slope handling — purely friction-based. GROUND_NORMAL_MIN: any surface with normal.z above this
// counts as "ground" for state/landing. There is NO hard walkable slope angle — friction
// (0.03 * normal.z * gravity) naturally decreases on steeper slopes, reducing movement control
// and allowing gravity's slope-parallel component to dominate. Sliding is emergent.
static constexpr float GROUND_NORMAL_MIN = 0.01f;  // near-zero epsilon, matches original engine's normal.z > 0

// Jump requires friction > JUMP_MIN_FRICTION. Flat ground friction=0.96; threshold 0.5
// corresponds to slopes steeper than ~58° (normal.z < 0.52).
static constexpr float JUMP_MIN_FRICTION = 0.5f;

// Grace period before entering Jump after losing ground contact. Derived from freefall time to
// travel 0.1 units (contact break distance): t = sqrt(2 * 0.1 / 32) ≈ 0.079s.
static constexpr float GROUND_GRACE_DURATION = 0.08f;

// Contact persistence — breaks terrain contacts at 0.1 units separation or 5.0 u/s relative
// velocity. This age-based system complements the grace period for all contacts (walls,
// ceilings, etc.). Contacts not re-detected within CONTACT_MAX_AGE frames are culled.
static constexpr int CONTACT_MAX_AGE = 3;

// Collision iteration count — per-preset via PhysicsTimestep.collisionIters. Vintage (12.5Hz): 3
// iterations. Modern/Ultra: 1 iteration (higher Hz compensates with more steps/sec).

// Stair stepping — 3-phase raycast algorithm for stepping up small ledges during ground movement.
// Triggered when FOOT/SHIN/KNEE contacts have steep/vertical normal (z < STEP_WALL_THRESHOLD).
// Phase 1: UP (clearance), Phase 2: FORWARD (space on ledge), Phase 3: DOWN (find surface).
// Lift = max(STEP_MIN_ZDELTA, hit_z - foot_z) + STEP_CLEARANCE.
static constexpr float STEP_WALL_THRESHOLD = 0.4f;   // normal.z below this = "wall" (steep surface)
static constexpr float STEP_UP_DIST        = 2.0f;   // upward probe distance (units)
static constexpr float STEP_FWD_SCALE      = 0.01f;  // forward probe = velocity * this
static constexpr float STEP_UP_EPSILON     = 0.01f;  // UP ray start offset above foot (collision skin)
static constexpr float STEP_MIN_ZDELTA     = 0.3f;   // minimum lift height (units)
static constexpr float STEP_CLEARANCE      = 0.02f;  // additional clearance above step surface

static constexpr float LANDING_MIN_VEL  = 2.0f;    // minimum downward speed for landing pose
static constexpr float LANDING_MIN_TIME = 0.2f;    // minimum 200ms between landing events

// Mantle system — 3-ray detection (UP→FWD→DOWN + forward surface finding from HEAD/BODY/FOOT)
// + 5-state animation: Hold(0.3s) → Rise(intermediate) → Forward(0.4s) → FinalRise → Complete.
static constexpr float MANTLE_UP_DIST   = 3.5f;    // upward headroom check distance
static constexpr float MANTLE_FWD_DIST  = 2.4f;    // forward ledge reach (radius × 2)
static constexpr float MANTLE_DOWN_DIST = 7.0f;    // downward surface scan distance
static constexpr float MANTLE_HOLD_TIME = 0.3f;    // State 1: hold duration
static constexpr float MANTLE_FWD_TIME  = 0.4f;    // State 3: forward movement duration
static constexpr float MANTLE_PULLBACK  = 0.55f;   // pull back from wall (approx half radius)
static constexpr float MANTLE_CONVERGE  = 0.0001f; // distance² convergence threshold (= 0.01 units)

// ── Climbing parameters ──
// OBB climbing — player attaches to objects with climbable_sides bitmask (ladders, etc.).
// Forward input = climb up, backward = climb down, strafe = lateral along wall.
// Gravity is suppressed during climbing; downward velocity is zeroed when no downward
// input is given. The original Dark Engine disabled terrain wall climbing ("don't climb
// on walls, spidey") — only OBB objects with climbable_sides support climbing.
//
// Climbing base speeds: the original engine uses "slow" input speeds (half of normal)
// as the base, then applies the 0.5× mode scale on top. Effective speeds:
//   forward: 5.5 × 0.5 = 2.75,  backward: 2.75 × 0.5 = 1.375,  strafe: 3.85 × 0.5 = 1.925
static constexpr float CLIMB_FWD_SPEED     = 5.5f;   // half of WALK_SPEED
static constexpr float CLIMB_BACK_SPEED    = 2.75f;   // half of BACKWARD_SPEED
static constexpr float CLIMB_STR_SPEED     = 3.85f;   // half of SIDESTEP_SPEED
static constexpr float CLIMB_JUMP_SCALE    = 5.0f;   // velocity impulse on jump-off
static constexpr float CLIMB_JUMP_REFLECT  = 0.5f;   // damping for reflected jump direction

// Movement control — force/mass-based system. See applyMovement() for full derivation.
static constexpr float CONTROL_MULTIPLIER = 11.0f;
static constexpr float FRICTION_FACTOR    = 0.03f;
static constexpr float VELOCITY_RATE      = 1.0f;

// Player mass — cancels in velocity-control path (rate = 11*mass*friction/vrate, accel =
// ctrl/mass), so this only matters for the rate cap and future non-controlled states.
static constexpr float PLAYER_MASS = 180.0f;

// Maximum control rate (units/sec²). Limits acceleration on high-friction surfaces.
// Flat ground: rate = 11 * 180 * 0.96 / 1.0 = 1900.8 (below cap).
static constexpr float MAX_CONTROL_RATE = 2000.0f;  // Caps just above ground rate.

// Base friction in water — provides drag even without ground contacts.
// Air has base_friction = 0 (no drag), water = 0.3.
static constexpr float WATER_BASE_FRICTION = 0.3f;

// Water control force scaling — all control forces halved when submerged.
// Makes water movement feel sluggish and heavy.
static constexpr float WATER_CONTROL_SCALE = 0.5f;

// Jump impulse scaling in water (half-strength jumps from pool floor/treading water).
static constexpr float WATER_JUMP_SCALE = 0.5f;

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

// ── Head spring constants — dt-dependent formula with clamped spring dt:
//   springDt = min(dt, 0.05),  tension_eff = BASE_TENSION / springDt,
//   damping_eff = BASE_DAMPING + (1-BASE_DAMPING) * springDt,
//   vel = displacement * tension_eff + old_vel * damping_eff,  vel.z *= Z_SCALE,
//   pos += vel * dt  (full dt, not clamped)
// At 12.5Hz (80ms): tension_eff=12.0 (not 7.5, +60%), damping_eff=0.069 (not 0.098, -30%).
// dt clamping makes the spring stiffer than naive implementation at low Hz.
// Note: NOT rate-independent (behavior differs at 60Hz). Once 12.5Hz is confirmed correct,
// a rate-independent analytical solution can be derived.
static constexpr float HEAD_SPRING_BASE_TENSION = 0.6f;   // spring tension constant
static constexpr float HEAD_SPRING_BASE_DAMPING = 0.02f;  // velocity retention base
static constexpr float HEAD_SPRING_Z_SCALE      = 0.5f;   // Z-axis half-strength
static constexpr float HEAD_SPRING_VEL_CAP      = 25.0f;  // max spring velocity magnitude
static constexpr float HEAD_SPRING_MAX_DT       = 0.05f;  // spring dt cap (50ms) — clamps dt for spring calc

/// Compute spring velocity from the dt-dependent formula. Returns new velocity; caller integrates
/// position (pos += vel * dt). Separation allows velocity modification before integration
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
// trans = translation speed scale.
struct ModeSpeedScale { float trans; };
static constexpr ModeSpeedScale MODE_SPEEDS[static_cast<int>(PlayerMode::NumModes)] = {
    {1.0f},  // Stand:     100% trans
    {0.6f},  // Crouch:    60% trans
    {0.7f},  // Swim:      70% trans
    {0.5f},  // Climb:     50% trans
    {1.0f},  // BodyCarry: 100% (set from script)
    {1.0f},  // Slide:     100%
    {1.0f},  // Jump:      100% (air control handles actual speed)
    {0.0f},  // Dead:      disabled
};

// ── Motion pose system ──
// Drives head bob through discrete motion poses — target head positions that the 3D spring
// tracks. Each stride activates a new pose; the spring naturally smooths transitions.
// Pose offsets in player-local coords: fwd, lat (right+), vert (down-).
// Stride distance: <5 u/s → 2.5, 5–15 u/s → lerp to 4.0, >15 u/s → 4.0.

/// Motion pose data — target head and body displacement from stance rest position. Animates both
/// submodels through a single blend system. Body offsets only in crouch-related poses.
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
// Motion coordinator blends offsets progressively toward targets; the head spring then tracks
// the blended offset, adding overshoot and organic smoothing. Durations: 0.6s strides, 0.8s
// mode transitions, 1.5s lean (slow easing), 0.0s landing/weapon impacts (sharp feel).
//
// Normal walking/idle (submodel 0 head offset):
//                                                   dur    hold   fwd     lat      vert     bFwd   bLat   bVert
static constexpr MotionPoseData POSE_NORMAL       = {0.8f,  0.0f,  0.0f,   0.0f,    0.0f,    0.0f,  0.0f,  0.0f};

// Walking strides — progressive blend over 0.6s, with spring dynamics adding natural bob.
// Both strides: identical vertical dip (-0.4), symmetric lateral sway (±0.1).
// Sign convention: original Y=LEFT-positive → our lat=RIGHT-positive, so left stride sways
// right (+0.1) and right sways left (-0.1), pushing the body AWAY from the stepping foot.
static constexpr MotionPoseData POSE_STRIDE_LEFT  = {0.6f,  0.01f, 0.0f,   0.1f,   -0.4f,   0.0f,  0.0f,  0.0f};
static constexpr MotionPoseData POSE_STRIDE_RIGHT = {0.6f,  0.01f, 0.0f,  -0.1f,   -0.4f,   0.0f,  0.0f,  0.0f};

// Crouching strides — wider sway (±0.15), deeper vert (-2.5 relative to crouch height).
// Same sign convention as standing strides.
static constexpr MotionPoseData POSE_CRAWL_LEFT   = {0.6f,  0.01f, 0.0f,   0.15f,  -2.5f,   0.0f,  0.0f, -1.0f};
static constexpr MotionPoseData POSE_CRAWL_RIGHT  = {0.6f,  0.01f, 0.0f,  -0.15f,  -2.5f,   0.0f,  0.0f, -1.0f};

// Crouching idle — 0.8s blend for smooth crouch transition:
static constexpr MotionPoseData POSE_CROUCH       = {0.8f,  0.0f,  0.0f,   0.0f,   -2.02f,  0.0f,  0.0f, -1.0f};

// Landing impact — instantaneous dip on landing (dur=0 = snap), held briefly:
static constexpr MotionPoseData POSE_JUMP_LAND    = {0.0f,  0.1f,  0.0f,   0.0f,   -0.5f,   0.0f,  0.0f,  0.0f};

// Body carry mode — heavier bob (deeper dip + more lateral sway while carrying).
static constexpr MotionPoseData POSE_CARRY_IDLE   = {0.8f,  0.0f,  0.0f,   0.0f,   -0.8f,   0.0f,  0.0f,  0.0f};
static constexpr MotionPoseData POSE_CARRY_LEFT   = {0.6f,  0.01f, 0.0f,   0.5f,   -1.5f,   0.0f,  0.0f,  0.0f};
static constexpr MotionPoseData POSE_CARRY_RIGHT  = {0.6f,  0.01f, 0.0f,  -0.15f,  -1.1f,   0.0f,  0.0f,  0.0f};

// Weapon swing — instant target, spring overshoot creates natural recoil feel.
static constexpr MotionPoseData POSE_WEAPON_SWING       = {0.0f,  0.0f, 0.8f,  0.0f,   0.0f,    0.0f,  0.0f,  0.0f};
static constexpr MotionPoseData POSE_WEAPON_SWING_CROUCH = {0.0f, 0.0f, 0.8f,  0.0f,  -2.02f,  0.8f,  0.0f, -1.0f};

// Standing lean — 1.5s blend, collision-limited via updateLean(). lat positive = right.
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

// Stride distance: <5 u/s → 2.5, 5–15 u/s → 2.5+3.0*((vel-5)/10), >15 u/s → 4.0 (hard cap).
// Note: formula would reach 5.5 at vel=15 but the cap overrides to 4.0.
static constexpr float STRIDE_DIST_BASE   = 2.5f;   // base footstep distance
static constexpr float STRIDE_DIST_RANGE  = 3.0f;   // additional distance at max speed
static constexpr float STRIDE_SPEED_LOW   = 5.0f;   // speed where stride distance starts growing
static constexpr float STRIDE_SPEED_RANGE = 10.0f;  // velocity range (15.0 - 5.0)
static constexpr float STRIDE_DIST_CAP    = 4.0f;   // hard cap at vel > 15.0

// Lean parameters — visual-only camera offset driven through motion poses (POSE_LEAN_*/
// POSE_CLNLEAN_*). Head spring provides natural easing. Standing: 2.2 lateral, Crouching:
// 1.7 lateral + 2.0 vertical drop.
static constexpr float LEAN_DISTANCE        = 2.2f;   // standing lean lateral offset (world units)
static constexpr float CROUCH_LEAN_DISTANCE = 1.7f;   // crouching lean lateral offset (world units)
static constexpr float LEAN_TILT            = 0.087f;  // max camera roll (~5 degrees, radians)

/// Compute velocity-dependent footstep distance.
static inline float computeFootstepDist(float hSpeed) {
    if (hSpeed < STRIDE_SPEED_LOW) return STRIDE_DIST_BASE;
    if (hSpeed > STRIDE_SPEED_LOW + STRIDE_SPEED_RANGE) return STRIDE_DIST_CAP;
    return STRIDE_DIST_BASE + STRIDE_DIST_RANGE * ((hSpeed - STRIDE_SPEED_LOW) / STRIDE_SPEED_RANGE);
}

} // namespace Darkness

#endif // __PLAYERPHYSICSCONSTANTS_H
