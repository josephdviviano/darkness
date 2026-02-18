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
        mDensity = std::max(cfg.density, 0.01f);  // clamp to prevent division by zero
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
    /// Proportional to collision-limited lean extent (mLeanAmount), derived from spring lateral
    /// displacement each frame.
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

    /// Set player position directly (for spawn, teleport). Resets velocity and updates cell index.
    /// If findCell fails at the given position, tries head and foot level as fallbacks (the
    /// caller often passes camera/eye position, and cell geometry may not extend below the floor).
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

    /// Register a callback for footstep sound events. Fired each stride trigger (distance-based).
    /// Receives foot position, horizontal speed, and ground texture index for material-based
    /// sound selection. Phase 3 Audio will use this to play surface-appropriate sounds.
    void setFootstepCallback(FootstepCallback cb) { mFootstepCb = std::move(cb); }

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
            if (mCurrentMode == PlayerMode::Climb) {
                // Jump off climbing surface — reflected/projected impulse away from wall
                breakClimb(true);
                // Don't clear mJumpRequested yet — mantle check at step 13 may use it
            } else if (isOnGround()) {
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

        // 10c. Climbing — detect/maintain/break climb on climbable OBB objects.
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
    bool    mMantleCompressed = false;  // true during Forward/FinalRise (offsets zeroed)

    // ── Overridable physics parameters (from P$PhysAttr/P$PhysDims or defaults) ──
    float mMass = PLAYER_MASS;
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

    // ── 3D head spring state ──
    // Direct discrete spring (dt-dependent formula) tracking pose targets in player-local coords
    // {fwd, lat, vert}. XY axes near-critical; Z (0.5× tension) overdamped → dominant vertical bob.
    Vector3 mSpringPos{0.0f};     // current spring position {fwd, lat, vert}
    Vector3 mSpringVel{0.0f};     // spring velocity (used by direct discrete formula)

    // ── Motion pose state ──
    // Stride-driven pose system: progressive blend + head spring. Rest-condition interrupt fires
    // ~100ms after each stride, creating stride→rest→stride oscillation at ~30% blend depth.
    // The head spring adds dynamic response on top (overshoot, lag).
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
    // Scratch buffer for tryStairStep() probes — avoids per-step heap allocation
    std::vector<SphereContact> mStepScratchContacts;

    // ── Render interpolation state ──
    // "Fix Your Timestep" interpolation — blends previous/current physics states with frame
    // fraction. At 12.5Hz vintage, without interpolation camera visibly snaps between steps.
    // mInterpAlpha = accumulator remainder / fixedDt, in [0, 1).
    Vector3 mPrevEyePos{0.0f};    // eye position from before the last physics step
    float   mPrevLeanTilt = 0.0f; // lean tilt from before the last physics step
    float   mInterpAlpha  = 1.0f; // interpolation fraction (1.0 = show current state)

    // Lean state — driven through motion poses (spring provides easing)
    int   mLeanDir        = 0;    // lean direction: -1=left, 0=center, +1=right
    float mLeanAmount     = 0.0f; // collision-limited lateral offset (derived from spring each frame)

    // Contacts from last collision resolution (used by detectGround and velocity removal). After
    // persistence merge, contains fresh (age=0) and maintained (age 1..CONTACT_MAX_AGE) contacts.
    std::vector<SphereContact> mLastContacts;

    // Persistent contact buffer — survives across frames with aging. Matched by (cellIdx, polyIdx)
    // identity; stale contacts culled after CONTACT_MAX_AGE frames without re-detection.
    std::vector<SphereContact> mPersistentContacts;

    // Pre-allocated scratch buffers for resolveCollisions() — avoids per-step heap allocs
    std::vector<SphereContact> mIterContacts;                // per-iteration contact accumulator
    std::vector<std::pair<Vector3, float>> mPushes;          // de-duplicated push normals

    // ── Per-texture friction table (indexed by TXLIST texture index, default 1.0) ──
    std::vector<float> mFrictionTable;

    // ── Per-texture climbability table (indexed by TXLIST texture index, default 0.0) ──
    // Climbability boosts friction on steep surfaces — NOT a climb-mode trigger.
    std::vector<float> mClimbabilityTable;

    // ── Callbacks ──
    FootstepCallback mFootstepCb;            // footstep sound event (Phase 3 Audio stub)
    ObjectCollisionCallback mObjectCollisionCb;  // player-vs-object collision (Task 26)
    const ObjectCollisionWorld *mObjectWorld = nullptr;  // for OBB lookup in object stepping

    // ── Diagnostic logging state ──
    FILE *mLogFile = nullptr;   // per-timestep CSV log file (null = disabled)
    bool  mStepLog = false;     // stair step diagnostics to stderr ([STEP] prefix)
    float mCamPitch = 0.0f;    // camera pitch from renderer (for logging only)
};

} // namespace Darkness

#endif // __PLAYERPHYSICS_H
