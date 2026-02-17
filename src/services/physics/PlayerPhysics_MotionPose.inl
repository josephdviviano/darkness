// Included inside PlayerPhysics class body — do not include standalone.

    /// Activate a new motion pose — captures current offset as blend start, sets new target.
    /// Head spring provides organic smoothing on top. Same-motion guard skips re-activation
    /// if already blending/holding toward the same target.
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

    /// Activate the next stride pose (alternates left/right). Progressive blend over 0.6s
    /// with head spring dynamics (overshoot, lag).
    inline void activateStride() {
        auto motionCfg = getModeMotion(mCurrentMode);
        const MotionPoseData *stride = mStrideIsLeft
            ? motionCfg.strideLeft : motionCfg.strideRight;

        // Each new stride interrupts the previous blend naturally via activatePose().
        activatePose(*stride);

        // Fire footstep callback (Phase 3 Audio) — FOOT position, horizontal speed, ground texture.
        if (mFootstepCb) {
            Vector3 footPos = mPosition + Vector3(0.0f, 0.0f, mSphereOffsetsBase[4]);
            float hSpeed = horizontalSpeed();
            mFootstepCb(footPos, hSpeed, mGroundTextureIdx);
        }

        mStrideIsLeft = !mStrideIsLeft;
        mLastStrideSimTime = mSimTime;
    }



    /// Update motion pose system — distance-based stride triggering. Strides trigger when foot
    /// travel exceeds computeFootstepDist(). Velocity thresholds: >1.0 u/s tracking, >1.5 u/s
    /// stride activation. When stopped, blend completes and transitions to rest pose.
    inline void updateMotionPose() {
        float hSpeed = horizontalSpeed();
        // Lean suppresses stride bob (spring lateral axis driven by lean offset; stride would conflict).
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
            // Progressive blend: curOffset += (target - cur) * (timer / duration). NOT linear interp
            // — cumulative blend with accelerating approach. At 12.5Hz with 0.6s duration: ~7
            // blend steps with progressively larger increments.
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

        // ── Distance-based stride accumulation (not time-based) ──
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
        // Walking: distance-triggered strides with rest-condition interrupt ~100ms after each
        // stride. Interrupts blend at ~30% → characteristic subtle bob (without this: ~3× too much).
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
                // Rest-condition interrupt: >100ms since stride AND past half-footstep distance.
                // Creates stride→rest oscillation, limiting effective bob to ~1/3 of stride target.
                activatePose(restPose);
            }
        }
        // Not walking: return to rest pose. Exception: while leaning, hold at lean target
        // indefinitely — setLeanDirection(0) activates rest pose on key release.
        else {
            if (mWasWalking && !mLandingActive && !isLeaning) {
                // Just stopped walking — immediately interrupt stride with rest pose. Same-motion
                // guard in activatePose() prevents re-triggering on subsequent idle frames.
                activatePose(restPose);
            } else if (poseReady && !isLeaning) {
                // Idle — activate rest pose if current target differs (e.g. landing bump
                // returning to idle, or mode changed Stand→Crouch).
                Vector3 restTarget(restPose.fwd, restPose.lat, restPose.vert);
                if (glm::length(mPoseEnd - restTarget) > 0.01f) {
                    activatePose(restPose);
                }
            }
        }

        mWasWalking = isWalking;
    }

    /// Compute raw (un-interpolated) eye position: body center + head offset + eye-above-head
    /// + spring displacement (player-local, rotated by yaw) + collision-limited lean.
    inline Vector3 computeRawEyePos() const {
        // During mantling, camera tracks spring-driven virtual head (smooth despite body teleports).
        if (mMantling) {
            return mMantleHeadPos + Vector3(0.0f, 0.0f, EYE_ABOVE_HEAD);
        }

        // Head Z offset always HEAD_OFFSET_Z. Crouch lowers eye via mSpringPos.z (POSE_CROUCH).
        Vector3 eye = mPosition + Vector3(0.0f, 0.0f, mSphereOffsetsBase[0] + EYE_ABOVE_HEAD + mSpringPos.z);

        // Use cached sin/cos (updated in setYaw() and fixedStep())
        float sinYaw = mSinYaw;
        float cosYaw = mCosYaw;

        // Spring forward displacement (mSpringPos.x = forward axis in player-local coords)
        if (std::fabs(mSpringPos.x) > 0.001f) {
            eye.x += cosYaw * mSpringPos.x;
            eye.y += sinYaw * mSpringPos.x;
        }

        // Collision-limited lateral offset (stride sway + lean flow through mLeanAmount, set from
        // mSpringPos.y in updateLean()). Stride sway produces ~0.2° roll (imperceptible).
        if (std::fabs(mLeanAmount) > 0.001f) {
            eye.x += sinYaw * mLeanAmount;
            eye.y -= cosYaw * mLeanAmount;
        }

        return eye;
    }

    /// Compute raw lean tilt — camera roll proportional to mLeanAmount (not gated on lean-active).
    /// Stride sway (±0.1u) → ~0.2° roll (imperceptible); lean (±2.2u) → full ~5° roll.
    /// On key release, spring decays mLeanAmount to zero → smooth roll return.
    inline float computeRawLeanTilt() const {
        float maxDist = isCrouching() ? CROUCH_LEAN_DISTANCE : LEAN_DISTANCE;
        if (maxDist < 0.01f) return 0.0f;
        return (mLeanAmount / maxDist) * LEAN_TILT;
    }

    /// Update lean — collision-limited lean derived from spring lateral displacement (mSpringPos.y).
    /// Lean driven through pose system (setLeanDirection→POSE_LEAN_*) with spring easing. Checks
    /// wall collisions at leaned head position and stores result in mLeanAmount.
    inline void updateLean() {
        // Read lean from spring lateral displacement
        mLeanAmount = mSpringPos.y;

        // Collision check — use contact penetration depth for exact maximum safe lean distance.
        // Lean holds at wall surface rather than snapping back to standing.
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
                // Lean direction in world space (center → leaned head).
                float leanSign = (mLeanAmount > 0.0f) ? 1.0f : -1.0f;
                Vector3 leanDir(sinYaw * leanSign, -cosYaw * leanSign, 0.0f);

                // Find largest pushback along lean direction to clear all walls.
                float maxPushback = 0.0f;
                for (const auto &c : contacts) {
                    // Project the contact push (normal * penetration) onto the
                    // negative lean direction — how much of the wall push opposes
                    // the lean.
                    float pushInLeanDir = glm::dot(c.normal * c.penetration, -leanDir);
                    if (pushInLeanDir > maxPushback)
                        maxPushback = pushInLeanDir;
                }

                // Reduce lean by pushback + margin — holds at wall distance rather than zeroing.
                constexpr float LEAN_WALL_MARGIN = 0.05f;
                float absLean = std::fabs(mLeanAmount);
                absLean = std::max(0.0f, absLean - maxPushback - LEAN_WALL_MARGIN);
                mLeanAmount = absLean * leanSign;
            }
        }
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

    /// Update 3D head spring — direct discrete dt-dependent formula. See
    /// PlayerPhysicsConstants.h for the full formula derivation and dt clamping behavior.
    /// Note: NOT rate-independent (behavior differs at 60Hz vs 12.5Hz).
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
