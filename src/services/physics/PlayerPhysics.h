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
 *    Uses a 5-submodel player model (2 real collision spheres + 3 point detectors).
 *    See PlayerPhysicsConstants.h for submodel offsets, speeds, and all physics constants.
 *
 *    Head bob is driven by discrete motion poses — target offsets that the head spring
 *    tracks. Each stride activates a new pose target, and the 3D spring naturally smooths
 *    the transitions, creating realistic bob with organic acceleration/deceleration.
 *
 *    See NOTES.SOURCE.md for full physics constants documentation.
 *
 *****************************************************************************/

#ifndef __PLAYERPHYSICS_H
#define __PLAYERPHYSICS_H

#include <cstdint>
#include <cstdio>
#include <functional>
#include <vector>

#include "PlayerPhysicsConstants.h"
#include "CollisionGeometry.h"
#include "ObjectCollisionGeometry.h"
#include "IPhysicsWorld.h"
#include "RayCaster.h"

namespace Darkness {

/// Per-frame velocity constraint — rebuilt from validated contacts each frame.
/// Matches the original Dark Engine's tConstraint { ObjID cause; mxs_vector dir; }.
/// Constraints are ephemeral (cleared + rebuilt every frame); contacts are persistent.
struct VelocityConstraint {
    Vector3 normal;      // constraint direction (prevents motion in this direction)
    int32_t objectId;    // which object caused this constraint (-1 = terrain)
};

/// Player physics simulation — custom 5-submodel polygon collision (2 real spheres + 3 point detectors).
/// Owns the player's position, velocity, and movement state.
/// Updated each frame by DarkPhysics::step().
class PlayerPhysics {
public:
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
        if (cfg.density < 0.01f)
            std::fprintf(stderr, "[DEFAULT] PlayerPhysics: density=%.4f clamped to 0.01 (prevent zero-division)\n", cfg.density);
        mDensity = std::max(cfg.density, 0.01f);
        // Clamp radii to minimum 0.1 to prevent zero-division in collision
        if (cfg.headRadius < 0.1f)
            std::fprintf(stderr, "[DEFAULT] PlayerPhysics: headRadius=%.4f clamped to 0.1\n", cfg.headRadius);
        mSphereRadii[0] = std::max(cfg.headRadius, 0.1f);
        if (cfg.bodyRadius < 0.1f)
            std::fprintf(stderr, "[DEFAULT] PlayerPhysics: bodyRadius=%.4f clamped to 0.1\n", cfg.bodyRadius);
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
            "bodyPoseX,bodyPoseY,bodyPoseZ,"
            "poseTimer,poseDur,poseHolding,"
            "strideDist,strideIsLeft,leanDir,leanAmount,"
            "cell,inputFwd,inputRight\n");
        std::fflush(mLogFile);
        mLogFlushCounter = 0;
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

    /// Enable/disable climbing diagnostics to stderr ([CLIMB] prefix).
    void setClimbLog(bool enable) { mClimbLog = enable; }

    /// True if the player is actively climbing an OBB (ladder, climbable surface).
    bool isClimbing() const { return mClimbingObjId >= 0; }

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
            std::fprintf(stderr, "[FALLBACK] PlayerPhysics: invalid timestep fixedDt=%.4f, resetting to MODERN (60Hz)\n", mTimestep.fixedDt);
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

    /// Set lean direction: -1 = left, 0 = center, +1 = right. Lean is visual-only — the physics
    /// body stays in place while the camera offsets laterally, collision-limited by wall checks.
    /// Driven through motion poses (POSE_LEAN_*/POSE_CLNLEAN_*) with spring easing. While
    /// leaning, stride bob is suppressed — lean poses take priority.
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

    /// Body center position (not eye position). In the 5-sphere model, body center is at
    /// ground + 4.2 on flat ground (higher than the old 2-sphere model's ground + 1.2).
    const Vector3 &getPosition() const { return mPosition; }

    /// Eye position — interpolated between previous and current physics states using "Fix Your
    /// Timestep" frame-fraction blending. Essential for low-Hz presets (12.5Hz vintage) where
    /// the physics timestep is much coarser than display refresh — without interpolation, the
    /// camera visibly jumps between physics positions.
    inline Vector3 getEyePosition() const {
        Vector3 current = computeRawEyePos();
        return glm::mix(mPrevEyePos, current, mInterpAlpha);
    }

    /// Camera roll/bank angle from leaning (radians), interpolated. Positive = tilting right.
    /// Proportional to spring lateral displacement (mSpringPos.y).
    inline float getLeanTilt() const {
        float current = computeRawLeanTilt();
        return mPrevLeanTilt + (current - mPrevLeanTilt) * mInterpAlpha;
    }

    /// Linear velocity
    const Vector3 &getVelocity() const { return mVelocity; }

    /// Last frame's contacts (for push system to inspect object collisions).
    const std::vector<SphereContact> &getContacts() const { return mContacts; }

    /// Object ID of the surface the player's feet are touching (-1 = none/terrain cell).
    /// Matches original Dark Engine's GetGroundObj(). Used for footstep sounds.
    int32_t getGroundObjID() const { return mGroundObjID; }

    /// Forward direction (X-Y plane, from yaw). Z-up coordinate system.
    Vector3 getForward() const {
        return Vector3(mCosYaw, mSinYaw, 0.0f);
    }

    /// Right direction (X-Y plane, perpendicular to forward). Z-up.
    Vector3 getRight() const {
        return Vector3(mSinYaw, -mCosYaw, 0.0f);
    }

    /// Apply a velocity impulse to the player (instant velocity addition).
    /// Used for jumps, water exit, etc. For object-impact knockback prefer
    /// applyKnockback below — that channel bleeds in over many ticks for a
    /// lagged shove instead of a one-frame teleport.
    void applyImpulse(const Vector3 &impulse) {
        mVelocity += impulse;
    }

    /// Apply a knockback impulse from an external impact (thrown object,
    /// explosion). The impulse is accumulated into mPendingKnockback and
    /// transferred to mVelocity exponentially over several ticks by
    /// integrateKnockback(), so the player feels a smooth shove instead of
    /// a one-frame jolt. Multiple impulses in the same frame are summed.
    void applyKnockback(const Vector3 &impulse) {
        mPendingKnockback += impulse;
    }

    // ── View punch — spring-driven camera kick from object impacts ──
    //
    // Source Engine style: angular velocity impulse decays via a damped
    // spring. Tuned slow + slightly overdamped so a hit produces a soft,
    // lagged camera tilt that crawls back to neutral without overshoot.

    /// Add angular velocity impulse to the view punch spring.
    void addViewPunch(const Vector3 &angleVel) {
        mPunchAngleVel += angleVel;
    }

    /// Update the view punch spring (called per frame, not per physics step).
    void updateViewPunch(float dt) {
        if (glm::length(mPunchAngle) < 1e-6f &&
            glm::length(mPunchAngleVel) < 1e-6f) return;
        float spring = std::min(PUNCH_SPRING * dt, 2.0f);
        mPunchAngleVel -= mPunchAngle * spring;
        mPunchAngleVel *= (1.0f - PUNCH_DAMPING * dt);
        mPunchAngle += mPunchAngleVel * dt;
    }

    /// Get current view punch offset (pitch, yaw, roll in radians).
    const Vector3 &getViewPunch() const { return mPunchAngle; }

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

    /// Check if the player is standing on a specific object (ground contact).
    /// Used by PressurePlateSystem to detect player weight on plates.
    bool isStandingOnObject(int32_t objID) const {
        if (!isOnGround()) return false;
        for (const auto &c : mContacts) {
            if (c.objectId == objID && c.normal.z > GROUND_NORMAL_MIN)
                return true;
        }
        return false;
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

    // ── Head-bob / pose-spring accessors (for per-render-frame diagnostic logging) ──
    /// Current head-spring displacement {fwd, lat, vert} in player-local coords.
    const Vector3 &getSpringPos() const { return mSpringPos; }
    /// Currently-blended pose offset (head spring's target this frame).
    const Vector3 &getPoseCurrent() const { return mPoseCurrent; }
    /// End target of the current pose blend (target the spring is chasing).
    const Vector3 &getPoseEnd() const { return mPoseEnd; }
    /// Horizontal speed magnitude (XY plane only).
    float getHorizontalSpeed() const {
        return std::sqrt(mVelocity.x * mVelocity.x + mVelocity.y * mVelocity.y);
    }
    /// Fix-your-timestep blend factor (accumulator remainder / fixedDt) used by
    /// getEyePosition() to interpolate between mPrevEyePos and computeRawEyePos().
    float getInterpAlpha() const { return mInterpAlpha; }
    /// Monotonic fixed-step counter — increments once per fixedStep() call.
    /// Caller can take deltas to count how many physics steps fell in a render frame.
    uint64_t getTotalFixedSteps() const { return mTotalFixedSteps; }
    /// Monotonic PPF-cancel counter — increments every time the anti-fall probe
    /// cancelled this step's movement. Useful for diagnosing stuck-on-floor cases.
    uint64_t getPPFCancels() const { return mPPFCancels; }
    /// Current movement input (set by renderer via setMovement/setPlayerMovement).
    float getInputForward() const { return mInputForward; }
    float getInputRight() const { return mInputRight; }

    /// Previous-tick eye snapshot — the source of the render interpolation
    /// lerp. mix(getPrevEyePos(), getRawEyePos(), getInterpAlpha()) reproduces
    /// what getEyePosition() returns. Exposed for diagnostic logging.
    const Vector3 &getPrevEyePos() const { return mPrevEyePos; }

    /// Current-tick (un-interpolated) eye position. The target of the render
    /// interpolation lerp. Returns the same value computeRawEyePos() uses
    /// inside getEyePosition().
    Vector3 getRawEyePos() const { return computeRawEyePos(); }

    /// Per-frame world-space displacement applied to the eye when the HEAD
    /// sphere bumps a wall (left over from the most recent fixed step).
    /// Stays nonzero only while the player is leaning into a wall.
    const Vector3 &getHeadClamp() const { return mHeadClamp; }

    /// Pending knockback impulse — accumulated by applyKnockback() and bled
    /// into mVelocity exponentially each fixedStep by integrateKnockback.
    /// Diagnostic exposure so the log can show how much shove is queued.
    const Vector3 &getPendingKnockback() const { return mPendingKnockback; }

    // ── Teleport ──

    /// Set player position directly (for spawn, teleport). Resets velocity and updates cell index.
    /// If findCell fails at the given position, tries head and foot level as fallbacks (the
    /// caller often passes camera/eye position, and cell geometry may not extend below the floor).
    inline void setPosition(const Vector3 &pos) {
        mPosition = pos;
        mEndPosition = pos;  // sync so resolveCollisions doesn't teleport to stale {0,0,0}
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
        mContacts.clear();
        mConstraints.clear();

        // Teleport is semantically equivalent to an airborne transition: snapshot
        // the foot position so the stride tracker doesn't see a huge cur_dist²
        // between the old mLastFootLoc and the new spawn/teleport position.
        leaveGround();

        // Reset spring state — teleporting should not carry old spring velocity
        // into the new position, which would cause oscillation on arrival.
        mSpringPos     = Vector3(0.0f);
        mSpringVel     = Vector3(0.0f);
        mSpringPrevPos = Vector3(0.0f);
        mSpringPrevVel = Vector3(0.0f);
        mSpringNextPos = Vector3(0.0f);
        mSpringNextVel = Vector3(0.0f);
        mSpringAccum   = 0.0f;
        mPoseStart = Vector3(0.0f);
        mPoseCurrent = Vector3(0.0f);
        mPoseEnd = Vector3(0.0f);
        mBodyPoseStart = Vector3(0.0f);
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

    /// Register a callback for footstep sound events. Fired each stride trigger (distance-based).
    /// Receives foot position, horizontal speed, and ground texture index for material-based
    /// sound selection. Phase 3 Audio will use this to play surface-appropriate sounds.
    void setFootstepCallback(FootstepCallback cb) { mFootstepCb = std::move(cb); }

    /// Register a callback for landing impact sound events. Fired when the player
    /// transitions from airborne (Jump mode) to ground contact with sufficient velocity.
    /// Receives foot position, downward fall speed, and ground texture index.
    void setLandingCallback(LandingCallback cb) { mLandingCb = std::move(cb); }

    /// Callback for object collision testing. Called per physics step during resolveCollisions(),
    /// after WR polygon tests but before push de-duplication. Receives 5 submodel sphere centers
    /// and radii, player's current portal cell, and output vector to append SphereContact results.
    /// Object contacts use cellIdx=-1 sentinel; polyIdx encodes (bodyIndex<<4)|(faceIdx&0xF).
    ///
    /// ODE UPGRADE: Would be replaced by registering the player as a kinematic dGeomID and
    /// using dNearCallback to generate contacts between player and object geoms.
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

    /// Set the object collision world pointer for object stair stepping.
    /// Allows tryStairStep() to look up OBB bodies for ray-vs-OBB tests
    /// and read climbable/edge-trigger properties.
    void setObjectCollisionWorld(const ObjectCollisionWorld *ocw) {
        mObjectWorld = ocw;
    }

    /// Callback that returns true if an object is pushable (skip stair stepping).
    /// Set by ObjectPushSystem so the player pushes objects instead of climbing over.
    using IsPushableCallback = std::function<bool(int32_t objID)>;
    void setIsPushableCallback(IsPushableCallback cb) {
        mIsPushableCb = std::move(cb);
    }

    /// Set per-texture friction lookup table (indexed by TXLIST texture index).
    /// Built at mission load from P$Friction on texture archetypes in dark.gam.
    void setFrictionTable(const std::vector<float> &table) {
        mFrictionTable = table;
    }

    /// Set per-texture climbability lookup table (indexed by TXLIST texture index).
    /// Built at mission load from P$Climbabil on texture archetypes in dark.gam.
    /// Climbability boosts friction on steep surfaces (not a climb-mode trigger).
    void setClimbabilityTable(const std::vector<float> &table) {
        mClimbabilityTable = table;
    }

    /// Callback to query platform velocity for moving terrain / elevator riding.
    /// Returns nullptr if the object is not a moving platform, or a pointer to
    /// its current velocity vector if it is. The pointer is valid for the current
    /// frame only.
    using PlatformVelocityCallback = std::function<const Vector3 *(int32_t objID)>;

    /// Register a callback to query platform velocities. Set by the main loop
    /// to bridge PlayerPhysics with MovingTerrainSystem.
    void setPlatformVelocityCallback(PlatformVelocityCallback cb) {
        mPlatformVelocityCb = std::move(cb);
    }

private:
    // ── Internal simulation steps ──

    /// Bleed pending knockback into mVelocity. Each tick transfers an
    /// exponential fraction of the pending impulse, so the shove from a
    /// thrown object spreads across multiple ticks instead of arriving as
    /// a one-frame velocity jump. Time constant ≈ 1 / kRate so most of the
    /// impulse delivers within ~3 / kRate seconds. Friction in applyMovement
    /// then drags the transferred velocity back to zero over walk-decel time,
    /// which is the desired "lagged shove that fades" feel.
    inline void integrateKnockback() {
        if (glm::length(mPendingKnockback) < 1e-4f) {
            mPendingKnockback = Vector3(0.0f);
            return;
        }
        constexpr float kRate = 4.0f;  // 1 / time_constant in seconds
        float k = 1.0f - std::exp(-kRate * mTimestep.fixedDt);
        Vector3 transferred = mPendingKnockback * k;
        mVelocity += transferred;
        mPendingKnockback -= transferred;
    }

    /// Called on every airborne transition (last floor contact destroyed, grace-timer
    /// expired, or teleport). Matches original Dark Engine LeaveGround: clears ground
    /// object and snapshots the current FOOT position / sim time into the stride tracker
    /// so cur_dist² starts near zero on the first airborne frame. Without this, a stale
    /// mLastFootLoc (e.g. constructor default or last-stride location from before the
    /// ledge) produces a huge cur_dist² that fires a stride every frame — the in-Jump
    /// `canUpdate` guard then prevents refresh, so the spam never stops.
    inline void leaveGround() {
        mGroundObjID = -1;
        mLastFootLoc = mPosition + Vector3(0.0f, 0.0f, mSphereOffsetsBase[4]);
        mLastFootTime = mSimTime;
    }

    /// Single fixed-timestep physics step
    inline void fixedStep(const ContactCallback &contactCb) {
        // Advance simulation time (for landing throttle, animation timing, etc.)
        mSimTime += mTimestep.fixedDt;
        ++mTotalFixedSteps;

        // ── Mantle takes over the full step when active ──
        // During mantling, normal movement/gravity/collision are suppressed.
        // The mantle state machine drives position directly.
        if (mMantling) {
            updateMantle();
            updateCell();
            updateMotionPose();
            updateHeadSpring();
            return;
        }

        // 1. Handle jump request (can jump from any ground mode)
        // Requires friction > 0.5 to jump — prevents jumping on steep slopes where
        // the player has no traction (above ~58°).
        // When friction is between 0.5 and 1.0, jump velocity is scaled down
        // proportionally, making slope-edge jumps weaker.
        if (mJumpRequested && !mMotionDisabled) {
            if (mCurrentMode == PlayerMode::Climb) {
                // Jump off climbing surface — reflected/projected impulse away from wall
                breakClimb(true);
                // Don't clear mJumpRequested yet — mantle check at step 13 may use it
            } else if (isOnGround()) {
                float jumpFriction = computeGroundFriction();

                // Diagnostic: when jumping off a non-terrain surface (e.g. a
                // crate), dump per-contact friction breakdown so we can see
                // why the jump feels lower than off solid floor. Compare with
                // a regular floor jump in the same log to spot the delta.
                if (mGroundObjID >= 0) {
                    int upContacts = 0;
                    float maxNZ = 0.0f, minNZ = 1.0f;
                    for (const auto &c : mContacts) {
                        if (c.normal.z <= 0.0f) continue;
                        ++upContacts;
                        if (c.normal.z > maxNZ) maxNZ = c.normal.z;
                        if (c.normal.z < minNZ) minNZ = c.normal.z;
                    }
                    std::fprintf(stderr,
                        "[JUMP-DIAG] from obj=%d friction=%.3f upContacts=%d "
                        "normalZ=[%.3f..%.3f] threshold=%.3f scale=%.3f%s\n",
                        mGroundObjID, jumpFriction, upContacts, minNZ, maxNZ,
                        JUMP_MIN_FRICTION,
                        (jumpFriction <= 1.0f) ? jumpFriction : 1.0f,
                        (jumpFriction <= JUMP_MIN_FRICTION) ? " REJECTED" : "");
                }

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
                    mLastFootTime = -1.0f;
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
                mLastFootTime = -1.0f;
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

        // 4. Reset model time tracking for this frame.
        //    Matches original StartFrame which resets collision state each frame.
        mModelTime = 0.0f;

        // 4b. Validate existing contacts — destroys stale/invalid contacts.
        //     Matches original: ConstrainFromTerrain (phcore.cpp lines 262-368)
        //     runs in UpdateDynamics BEFORE UpdateModelTransDynamics.
        validateContacts();

        // 5. Apply forces to velocity: gravity + movement input.
        //    Original: UpdateModelTransDynamics accumulates all forces (gravity,
        //    buoyancy, friction, control) then integrates in one step. We match
        //    by running gravity, then movement (which integrates friction), then
        //    constraint — so pre-constraint vz is typically gravity impulse
        //    partially cancelled by the friction Z-boost, not raw gravity×dt.
        //    Snapshot pre-gravity velocity so the friction drag-scale clamp sees
        //    the same magnitude the original engine does (|velocity|, not
        //    |velocity+gravity×dt|). Diverges only during motion, but kept
        //    unconditionally for clarity.
        mPreGravityVelocity = mVelocity;
        applyGravity();
        // Post-gravity, pre-movement snapshot — surfaced by the [CONSTRAIN] log
        // so the friction contribution to vz is legible alongside the final value.
        mPreMovementVelocity = mVelocity;
        if (!mMotionDisabled)
            applyMovement();

        // 5b. Bleed pending knockback into mVelocity. Object-impact handlers
        //     (DarkPhysics::handlePlayerDynamicContact) push impulses into
        //     mPendingKnockback rather than mVelocity directly so the response
        //     spreads across many ticks — a lagged shove instead of a one-tick
        //     velocity jump. Runs after applyMovement so player input does not
        //     starve the knockback transfer; constrainVelocity below still
        //     removes any component going into a wall.
        integrateKnockback();

        // 6. Constrain velocity against validated contacts.
        //    Matches original: ApplyConstraints (phmod.cpp line 1861) removes
        //    velocity components going into surfaces AFTER dynamics.
        constrainVelocity();

        // 7. Update head spring — uses current pose target + spring dynamics
        updateHeadSpring();

        // 8. Compute end position WITHOUT moving mPosition.
        //    Matches original: UpdateModel → UpdateTargetLocation →
        //    UpdateEndLocation computes EndLocationVec = LocationVec + velocity*dt.
        //    Position (LocationVec) does NOT advance until UpdatePositions at
        //    frame end. Collision detection sweeps from LocationVec to EndLocationVec.
        mPrevPosition = mPosition;  // save for head spring / mantle reference
        mEndPosition = mPosition + mVelocity * mTimestep.fixedDt;

        // 8b. Anti-fall probe (Dark Engine convention): at sub-walk speeds on ground,
        //     raycast 9 units down from the target foot; if nothing is found, cancel
        //     this frame's movement so the player can't shuffle off an edge at creep
        //     speed. Speed threshold: 0.9× walk (stand) or 0.6× walk (crouch).
        //
        //     Speed compare is 3D (|v|² vs threshold²) so a player dropping at the
        //     edge of a ledge isn't saved just because horizontal speed alone is low.
        //     The ray origin is lifted ~0.5 units above the foot (with the ray length
        //     extended to match) — the original probes from a submodel-owned foot
        //     location that sits slightly above the contact surface, and our body-
        //     center-derived origin can land a hair below the floor before the
        //     post-collision ground-snap happens, producing false "no ground" hits.
        if (isOnGround() && mCellIdx >= 0) {
            // Threshold is the per-mode walk-speed cap (WALK_SPEED × mode_trans_scale)
            // times the mode-specific anti-fall multiplier (0.9 stand / 0.6 crouch).
            // The mode-scale MUST be baked in — omitting it makes crouch fire whenever
            // the player is below un-scaled walk speed, i.e. always during crouching.
            float modeScale  = MODE_SPEEDS[static_cast<int>(mCurrentMode)].trans;
            float antiFallMul = (mCurrentMode == PlayerMode::Crouch) ? 0.6f : 0.9f;
            float threshold  = WALK_SPEED * modeScale * antiFallMul;
            if (glm::length2(mVelocity) < threshold * threshold) {
                constexpr float PROBE_ORIGIN_LIFT = 0.5f;
                Vector3 origin = mEndPosition +
                    Vector3(0.0f, 0.0f, mSphereOffsetsBase[4] + PROBE_ORIGIN_LIFT);
                Vector3 end = origin -
                    Vector3(0.0f, 0.0f, 9.0f + PROBE_ORIGIN_LIFT);
                // Pass the player's current cell as starting hint (Dark Engine
                // convention). Without this, the raycaster's fallback linear
                // scan can land on a neighbour cell whose local floor polygon
                // doesn't cover the origin's (x,y) — the portal traversal then
                // misses the floor and PPF false-fires on open interior ground.
                RayHit fallHit;
                int32_t terminalCell = -1;
                bool found = raycastWorld(mCollision.getWR(), origin, end, fallHit,
                                          &terminalCell,
                                          /*startCellHint=*/mCellIdx);
                if (!found) {
                    // No ground within 9 units below target foot — cancel movement
                    mEndPosition = mPosition;
                    ++mPPFCancels;
                    if (mStepLog && mPPFCancels <= 40) {
                        std::fprintf(stderr,
                            "[PPF] miss: origin=(%.3f,%.3f,%.3f) end=(%.3f,%.3f,%.3f) "
                            "startCell=%d terminalCell=%d mode=%s hSpd=%.2f\n",
                            origin.x, origin.y, origin.z, end.x, end.y, end.z,
                            mCellIdx, terminalCell, modeName(mCurrentMode),
                            horizontalSpeed());
                    }
                } else if (mStepLog && mPPFCancels == 0 && (mTotalFixedSteps % 240) == 0) {
                    // Occasional sanity print of a successful PPF probe (4 Hz)
                    std::fprintf(stderr,
                        "[PPF] hit: origin=(%.3f,%.3f,%.3f) hit=(%.3f,%.3f,%.3f) "
                        "distance=%.3f startCell=%d terminalCell=%d\n",
                        origin.x, origin.y, origin.z,
                        fallHit.point.x, fallHit.point.y, fallHit.point.z,
                        fallHit.distance, mCellIdx, terminalCell);
                }
            }
        }

        // 9. Resolve collisions: sweep from mPosition to mEndPosition,
        //    IntegrateToCollision + CheckStep/Bounce, then commit
        //    mPosition = mEndPosition (UpdatePositions equivalent).
        resolveCollisions(contactCb);

        // 10. Update cell after collision resolution committed position
        updateCell();

        // 11. Climbing — detect/maintain/break climb on climbable OBB objects.
        //      Must run after collision resolution (needs fresh contacts) and after
        //      stair step (stair step breaks climb). Before detectGround so climbing
        //      prevents ground transition.
        if (!mMantling && !mMotionDisabled) {
            if (mCurrentMode == PlayerMode::Climb)
                checkClimbContinuation();
            else if (mCurrentMode != PlayerMode::Swim)
                checkClimb();
        }

        // 11. Detect ground and update movement state
        detectGround();

        // 11b. Ground breaks climb — if we've reached ground while climbing,
        // transition to standing. This handles climbing down to the bottom
        // of a ladder or losing the surface while near the floor.
        if (mCurrentMode == PlayerMode::Climb && isOnGround())
            breakClimb(false);

        // 11c. Platform riding — if standing on a moving terrain object,
        // add platform velocity to player position (reference frame tracking).
        // The original Dark Engine adds a velocity constraint for the contact
        // normal component and tracks the reference frame object. Our simplified
        // version directly applies the platform displacement each step.
        updatePlatformRiding();

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

        // 14. Write diagnostic log row (if logging enabled)
        writeLogRow();
    }

    // ── Implementation includes ──
    // Method bodies split by concern into .inl files for maintainability.
    // These are included inside the class body and have full access to members.
    #include "PlayerPhysics_Movement.inl"
    #include "PlayerPhysics_Collision.inl"
    #include "PlayerPhysics_StairStep.inl"
    #include "PlayerPhysics_Mantle.inl"
    #include "PlayerPhysics_Climb.inl"
    #include "PlayerPhysics_MotionPose.inl"
    #include "PlayerPhysics_Diagnostics.inl"

    // ── State ──

    const CollisionGeometry &mCollision;  // world collision geometry (not owned)
    PhysicsTimestep mTimestep = MODERN;  // active timestep configuration

    Vector3 mPosition{0.0f};       // body center position (LocationVec equivalent)
    Vector3 mEndPosition{0.0f};    // projected end position (EndLocationVec equivalent)
    Vector3 mPrevPosition{0.0f};   // position before frame (for head spring, mantle)
    Vector3 mVelocity{0.0f};       // linear velocity
    // Transient snapshots used by friction and diagnostics to see velocity state
    // mid-step. mPreGravityVelocity is the velocity before any current-frame
    // forces are added — its magnitude feeds the friction drag-scale clamp so
    // drag depends on the previous-frame speed rather than the post-gravity
    // speed. mPreMovementVelocity is post-gravity, pre-movement, printed by
    // the [CONSTRAIN] log so the gravity-vs-friction contribution is legible.
    Vector3 mPreGravityVelocity{0.0f};
    Vector3 mPreMovementVelocity{0.0f};
    // Monotonic fixed-step counter — exposed via getTotalFixedSteps(). The renderer
    // reads this each frame and takes deltas so the head log knows how many physics
    // steps actually occurred in a given render frame (useful for diagnosing bob
    // artefacts that correlate with render-rate-over-physics-rate ratio).
    uint64_t mTotalFixedSteps = 0;
    // Monotonic counter for PPF (anti-fall probe) cancellations. A stuck player
    // shows up as this counter incrementing once per step while position stays fixed.
    uint64_t mPPFCancels = 0;
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

    // ── Climbing state ──
    // Active when player is attached to a climbable OBB (ladder, climbable surface).
    // Gravity is suppressed; forward/backward input remapped to up/down along the wall.
    int32_t mClimbingObjId = -1;       // Object ID being climbed (-1 = not climbing)
    Vector3 mClimbFaceNormal{0.0f};    // world-space normal of the climbable face
    int     mClimbFaceIdx = -1;        // which OBB face (0-5)
    bool    mClimbLog = false;         // diagnostics to stderr

    // ── Mantle spring state ──
    // Virtual head position driven by the spring during mantling.
    // Camera tracks this instead of mPosition + HEAD_OFFSET_Z.
    Vector3 mMantleHeadPos{0.0f};      // spring-driven virtual head position
    Vector3 mMantleHeadVel{0.0f};      // virtual head velocity (spring state)
    Vector3 mMantleStartPos{0.0f};     // body center at mantle start (for abort fallback)
    bool    mMantleCompressed = false;  // true during Forward/FinalRise (offsets zeroed)

    // ── Overridable physics parameters (from P$PhysAttr/P$PhysDims or defaults) ──
    float mMass = PLAYER_MASS;
    float mElasticity = 1.0f;       // model elasticity (default 1.0, from P$PhysAttr)
    float mDensity = 0.9f;          // buoyancy density — 0.9 (positive)
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

    // ── 3D head spring state (virtualised at 12.5 Hz) ──
    // The original discrete spring formula is advanced at SPRING_NATIVE_DT (0.08 s)
    // regardless of physics rate. mSpringPrev/Next cache the virtual sample state;
    // mSpringAccum is the wall-time offset since the last virtual sample crossing.
    // mSpringPos/Vel are the *displayed* values produced by cubic-Hermite
    // interpolation between Prev and Next using their cached velocities as slopes.
    // See feedback_exact_player_dynamics.md for the design principle.
    Vector3 mSpringPos{0.0f};       // displayed spring position (Hermite-interpolated)
    Vector3 mSpringVel{0.0f};       // exposed virt-next velocity (for diagnostic log parity)
    Vector3 mSpringPrevPos{0.0f};   // position at virtual sample N
    Vector3 mSpringPrevVel{0.0f};   // velocity at virtual sample N
    Vector3 mSpringNextPos{0.0f};   // position at virtual sample N+1
    Vector3 mSpringNextVel{0.0f};   // velocity at virtual sample N+1
    float   mSpringAccum = 0.0f;    // seconds since virtual sample N, in [0, SPRING_NATIVE_DT)

    // ── Motion pose state ──
    // Stride-driven pose system: linear blend (mPoseStart → mPoseEnd over
    // mPoseDuration) + head spring for organic smoothing. mPoseStart is
    // snapshotted from mPoseCurrent at activatePose() so mid-blend interrupts
    // restart cleanly from the current interpolated point rather than chasing
    // a moving target.
    Vector3 mPoseStart{0.0f};     // HEAD blend start offset (captured on activatePose)
    Vector3 mPoseEnd{0.0f};       // HEAD blend target offset {fwd, lat, vert}
    Vector3 mPoseCurrent{0.0f};   // HEAD current interpolated pose offset
    Vector3 mBodyPoseStart{0.0f};     // BODY blend start offset
    Vector3 mBodyPoseEnd{0.0f};       // BODY blend target offset
    Vector3 mBodyPoseCurrent{0.0f};   // BODY current blended offset
    float mPoseTimer    = 0.0f;   // elapsed time in current blend/hold (shared HEAD+BODY)
    float mPoseDuration = 0.8f;   // current blend duration (from pose data)
    float mPoseHoldTime = 0.0f;   // current hold time at target
    bool  mPoseHolding  = true;   // true = holding at target, false = blending

    // Stride tracking — distance-based triggering.
    // Strides activate when foot travel distance exceeds computeFootstepDist().
    bool  mStrideIsLeft  = true;  // which foot is next (alternates each stride)
    bool  mLandingActive = false; // true while landing bump pose is active

    // Stride tracking — absolute position-based, matching original Dark Engine
    // (PLYRMOV.CPP m_LastFootLoc / m_LastFootTime). Distance is computed each
    // frame as dist2(mLastFootLoc, currentFootLoc). Strides fire when distance
    // exceeds the velocity-dependent threshold. Refreshed to current foot
    // position on every airborne transition via leaveGround() — without this,
    // stale values (constructor default or last-stride location from before a
    // ledge) produce a huge cur_dist² and trigger a stride every frame while
    // the in-Jump canUpdate guard prevents refresh.
    Vector3 mLastFootLoc{0.0f};      // 3D foot position at last stride
    float   mLastFootTime = -1.0f;   // sim time at last stride (-1 = no stride yet)
    float   mLastStrideSimTime = 0.0f;  // simTime when last stride was activated

    // Ground contact normal, texture, and object from last collision pass.
    // mGroundObjID matches original Dark Engine's m_GroundObj — tracks which terrain
    // or object the player's feet are touching. Updated during validateContacts()
    // on FOOT contact transitions. Used for footstep sound placement and material lookup.
    Vector3 mGroundNormal{0.0f, 0.0f, 1.0f};
    int32_t mGroundTextureIdx = -1;  // texture index of ground surface (for footstep material)
    int32_t mGroundObjID = -1;       // object ID of ground surface (-1 = none/terrain cell)

    // Model time tracking — how much of the current frame has been consumed by
    // position integration. Matches original Dark Engine's pDynamics->GetCurrentTime()
    // (PHCORE.CPP line 5114). Used by IntegrateToCollision to compute remaining
    // integration time: integration_time = collision_time - mModelTime. When
    // mModelTime equals the frame's total dt, integration_time = 0 and the model
    // is already at its end position — cascade collisions use the current position
    // without additional backup. Reset to 0 at frame start.
    float mModelTime = 0.0f;

    // Platform riding state — tracks which moving platform the player is standing on.
    // When grounded on a moving terrain object, its velocity is added to the player's
    // position each frame (reference frame tracking). On platform departure, the
    // platform velocity is inherited into the player's velocity for smooth detachment.
    int32_t mPlatformObjID = 0;          // object ID of platform (0 = not on a platform)
    Vector3 mPlatformVelocity{0.0f};     // platform's velocity last frame

    // Ground grace period state — prevents instant Jump transitions on bumps
    // and slopes where collision + probe both fail momentarily.
    float mGroundGraceTimer = 0.0f;     // countdown timer (seconds)
    bool  mGroundGraceActive = false;   // true during the grace window

    // Scratch buffer for groundTest() probe — avoids per-step heap allocation
    std::vector<SphereContact> mGroundProbeContacts;
    // Scratch buffer for tryStairStep() probes — avoids per-step heap allocation
    std::vector<SphereContact> mStepScratchContacts;

    // ── Render interpolation state ──
    // "Fix Your Timestep" interpolation — blends previous/current physics states with frame
    // fraction. At 12.5Hz vintage, without interpolation camera visibly snaps between steps.
    // mInterpAlpha = accumulator remainder / fixedDt, in [0, 1).
    Vector3 mPrevEyePos{0.0f};    // eye position from before the last physics step
    float   mPrevLeanTilt = 0.0f; // lean tilt from before the last physics step
    float   mInterpAlpha  = 1.0f; // interpolation fraction (1.0 = show current state)

    // Lean state — driven through motion poses (spring provides easing).
    // Lateral camera offset is read directly from mSpringPos.y; wall pushback
    // flows through the body collision pipeline rather than a per-frame clamp.
    int   mLeanDir        = 0;    // lean direction: -1=left, 0=center, +1=right

    // Live contact list — validated each frame by validateContacts(), new contacts
    // added during resolveCollisions(). Contacts are persistent collision records
    // that survive across frames (destroyed only when validation fails).
    std::vector<SphereContact> mContacts;

    // Per-frame velocity constraints — rebuilt from validated contacts each frame
    // by validateContacts(). Constraints are ephemeral velocity normals that prevent
    // penetration. Matches original Dark Engine separation: contacts persist in
    // g_PhysContactLinks, constraints rebuilt in ConstrainFromTerrain/Objects each frame.
    std::vector<VelocityConstraint> mConstraints;

    // Pre-allocated scratch buffers for resolveCollisions() — avoids per-step heap allocs
    std::vector<SphereContact> mFreshContacts;               // per-frame fresh contact accumulator
    std::vector<SphereContact> mIterContacts;                // per-iteration contact accumulator
    std::vector<std::pair<Vector3, float>> mPushes;          // de-duplicated body push normals
    std::vector<std::pair<Vector3, float>> mHeadPushes;      // de-duplicated HEAD-only push normals

    // World-space displacement applied to the HEAD sphere position to prevent it
    // penetrating walls. Reset each step in resolveCollisions(); accumulated from
    // HEAD-submodel wall contacts. Read by computeRawEyePos / computeRawLeanTilt
    // to clamp the rendered camera offset, and by the next iter's HEAD-sphere
    // collision test. Reproduces the original engine's submodel collision response
    // (wall push adjusts HEAD's m_position only, body stays put).
    Vector3 mHeadClamp{0.0f};

    // ── Per-texture friction table (indexed by TXLIST texture index, default 1.0) ──
    std::vector<float> mFrictionTable;

    // ── Per-texture climbability table (indexed by TXLIST texture index, default 0.0) ──
    // Climbability boosts friction on steep surfaces — NOT a climb-mode trigger.
    std::vector<float> mClimbabilityTable;

    // ── View punch spring (Source Engine style camera kick) ──
    // Spring-driven angular offset from object impacts. Tuned slow and
    // slightly overdamped (ω₀≈4.7 rad/s, ζ≈1.17): a hit tilts the camera
    // and it crawls back to neutral over ~1 s with no oscillation. The
    // earlier values (SPRING=65, DAMPING=9; ω₀≈8, ζ≈0.56) were stiff and
    // underdamped — a noticeable snap that felt twitchy on light impacts.
    static constexpr float PUNCH_DAMPING = 11.0f;
    static constexpr float PUNCH_SPRING  = 22.0f;
    Vector3 mPunchAngle{0.0f};      // current angular offset (pitch, yaw, roll)
    Vector3 mPunchAngleVel{0.0f};   // angular velocity of the spring

    // ── Knockback channel ──
    // Pending velocity impulse from object impacts, bled into mVelocity
    // exponentially each fixedStep by integrateKnockback. Smooths the
    // shove from being hit so the player slides instead of teleporting.
    Vector3 mPendingKnockback{0.0f};

    // ── Callbacks ──
    FootstepCallback mFootstepCb;            // footstep sound event (stride-based)
    LandingCallback mLandingCb;              // landing impact sound event
    ObjectCollisionCallback mObjectCollisionCb;  // player-vs-object collision (Task 26)
    IsPushableCallback mIsPushableCb;              // skip stair step for pushable objects
    PlatformVelocityCallback mPlatformVelocityCb;  // moving terrain velocity query (Task 57)
    const ObjectCollisionWorld *mObjectWorld = nullptr;  // for OBB lookup in object stepping

    // ── Diagnostic logging state ──
    FILE *mLogFile = nullptr;   // per-timestep CSV log file (null = disabled)
    int   mLogFlushCounter = 0; // flush every N rows (avoids data loss on crash)
    bool  mStepLog = false;     // stair step diagnostics to stderr ([STEP] prefix)
    float mCamPitch = 0.0f;    // camera pitch from renderer (for logging only)
};

} // namespace Darkness

#endif // __PLAYERPHYSICS_H
