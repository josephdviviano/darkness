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

        mPoseEnd = newTarget;
        mBodyPoseEnd = newBodyTarget;
        mPoseDuration = pose.duration;
        mPoseHoldTime = pose.holdTime;
        mPoseTimer = 0.0f;
        mPoseHolding = false;

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
        // Offset footstep laterally to alternate left/right foot for spatial audio immersion.
        // The "right" vector in Z-up is (sin(yaw), -cos(yaw), 0); left foot gets negative offset.
        // Crouched stance is slightly wider than standing for a subtle but noticeable effect.
        if (mStepLog) {
            std::fprintf(stderr, "[FOOTSTEP] stride at simT=%.3f pos=(%.1f,%.1f,%.1f) hSpd=%.1f tex=%d\n",
                mSimTime, mPosition.x, mPosition.y, mPosition.z,
                horizontalSpeed(), mGroundTextureIdx);
        }
        if (mFootstepCb) {
            constexpr float FOOT_OFFSET_STAND  = 0.8f;  // lateral offset from center (standing)
            constexpr float FOOT_OFFSET_CROUCH = 1.0f;  // wider stance when crouched

            float lateralDist = isCrouching() ? FOOT_OFFSET_CROUCH : FOOT_OFFSET_STAND;
            // mStrideIsLeft = head bobs left = RIGHT foot strikes ground (and vice versa)
            float sign = mStrideIsLeft ? 1.0f : -1.0f;
            Vector3 right(mSinYaw, -mCosYaw, 0.0f);

            Vector3 footPos = mPosition + Vector3(0.0f, 0.0f, mSphereOffsetsBase[4])
                            + right * (sign * lateralDist);
            float hSpeed = horizontalSpeed();
            mFootstepCb(footPos, hSpeed, mGroundTextureIdx);
        }

        mStrideIsLeft = !mStrideIsLeft;  // Flips stride for next time.
        mLastStrideSimTime = mSimTime;
    }



    /// Update motion pose system — distance-based stride triggering. Strides trigger when foot
    /// travel exceeds computeFootstepDist(). Velocity thresholds: >1.0 u/s tracking, >1.5 u/s
    /// stride activation. When stopped, blend completes and transitions to rest pose.
    inline void updateMotionPose() {
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

        // ── Absolute-position stride system (matching original PLYRMOV.CPP) ──
        // The original tracks the 3D foot position at the last stride and computes
        // distance to the current foot position each frame. Strides fire when this
        // distance exceeds the velocity-dependent threshold. The position persists
        // through brief airborne moments, preventing double footsteps on stairs.
        float speed = glm::length(mVelocity);  // 3D speed (original uses mx_mag_vec)
        Vector3 footLoc = mPosition + Vector3(0.0f, 0.0f, mSphereOffsetsBase[4]);
        float curDist2 = glm::length2(mLastFootLoc - footLoc);
        float footstepDist = computeFootstepDist(speed);
        bool isLeaning = (mLeanDir != 0);

        // Per-mode rest pose
        const MotionPoseData &restPose = *getModeMotion(mCurrentMode).restPose;

        // Landing bump takes priority — must complete before stride resumes
        if (mLandingActive) {
            if (poseReady) {
                mLandingActive = false;
                if (isLeaning) {
                    activatePose(isCrouching()
                        ? (mLeanDir < 0 ? POSE_CLNLEAN_LEFT : POSE_CLNLEAN_RIGHT)
                        : (mLeanDir < 0 ? POSE_LEAN_LEFT : POSE_LEAN_RIGHT));
                } else {
                    activatePose(restPose);
                }
            }
        }
        // Stride distance check (original PLYRMOV.CPP lines 197-240)
        else if (speed > 1.0f && (mLastFootTime < 0.0f || curDist2 > footstepDist * footstepDist)) {
            bool onGround = isOnGround();

            // Update foot location/time — but NOT in Jump mode, and Stand/Crouch
            // require on_ground. Original (PLYRMOV.CPP lines 203-207):
            // if ((mode != kPM_Jump) && ((mode != kPM_Stand) || on_ground))
            bool canUpdate = (mCurrentMode != PlayerMode::Jump) &&
                             (mCurrentMode == PlayerMode::Swim ||
                              mCurrentMode == PlayerMode::Climb ||
                              onGround);
            if (canUpdate) {
                mLastFootLoc = footLoc;
                mLastFootTime = mSimTime;
            }

            // Stride activation — requires speed > 1.5 and ground for Stand/Crouch
            if (speed > 1.5f && !isLeaning) {
                bool doStride = true;
                // Stand/Crouch must be on ground for head bob
                if (mCurrentMode == PlayerMode::Stand ||
                    mCurrentMode == PlayerMode::Crouch) {
                    if (!onGround) doStride = false;
                }
                if (doStride) {
                    if (mStepLog) std::fprintf(stderr, "[STRIDE] distance trigger (dist2=%.2f thresh2=%.2f)\n",
                        curDist2, footstepDist * footstepDist);
                    activateStride();
                }
            }
        }
        // Rest interrupt (original PLYRMOV.CPP lines 243-247):
        // >100ms since last stride AND velocity > 0.1 AND
        // (past half stride distance OR velocity < 1.5)
        else if (mLastFootTime >= 0.0f &&
                 (mSimTime - mLastFootTime) > 0.1f &&
                 speed > 0.1f &&
                 (curDist2 > (footstepDist * 0.5f) * (footstepDist * 0.5f) || speed < 1.5f)) {
            activatePose(restPose);
        }
        // Idle — return to rest pose when not walking and pose is ready
        else if (poseReady && !isLeaning) {
            Vector3 restTarget(restPose.fwd, restPose.lat, restPose.vert);
            if (glm::length(mPoseEnd - restTarget) > 0.01f) {
                activatePose(restPose);
            }
        }

        // Velocity stop (original PLYRMOV.CPP line 250-251):
        // Reset foot time when velocity drops below 1.0 — next movement triggers
        // immediate stride (mLastFootTime < 0 flag).
        if (speed < 1.0f)
            mLastFootTime = -1.0f;
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
    /// Lean driven through pose system (setLeanDirection→POSE_LEAN_*) with spring easing.
    /// Sweeps HEAD sphere along the lean path testing both world geometry and objects
    /// (doors, crates) for continuous collision coverage. Stores result in mLeanAmount.
    inline void updateLean() {
        // Read lean from spring lateral displacement
        mLeanAmount = mSpringPos.y;

        // Collision check — sweep HEAD sphere along the lean path testing both
        // world geometry and objects. Samples in steps no larger than sphere radius
        // for continuous coverage of thin obstacles (e.g. doors).
        // Lean holds at wall/object surface rather than snapping back to standing.
        if (std::fabs(mLeanAmount) > 0.01f && mCellIdx >= 0) {
            float sinYaw = mSinYaw;
            float cosYaw = mCosYaw;
            // HEAD collision position for lean: standing offset + pose-driven drop
            float headOffset = mSphereOffsetsBase[0] + mPoseCurrent.z;
            float leanBaseZ = headOffset + mSpringPos.z;
            // Base (unleaned) head position
            Vector3 baseHead = mPosition + Vector3(0.0f, 0.0f, leanBaseZ);

            float leanSign = (mLeanAmount > 0.0f) ? 1.0f : -1.0f;
            Vector3 leanDir(sinYaw * leanSign, -cosYaw * leanSign, 0.0f);

            float absLean = std::fabs(mLeanAmount);
            float sphereR = mSphereRadii[0];
            constexpr float LEAN_WALL_MARGIN = 0.05f;

            // Sample lean path in steps no larger than sphere radius. Each sample's
            // sphere overlaps the previous, giving continuous coverage that catches
            // thin objects (doors) an endpoint-only test would miss.
            int numSteps = std::max(1, static_cast<int>(std::ceil(absLean / sphereR)));
            float safeLean = absLean;

            for (int i = 1; i <= numSteps; ++i) {
                float sampleDist = absLean * float(i) / float(numSteps);
                Vector3 sampleHead = baseHead + leanDir * sampleDist;

                // Find cell at sample position for broadphase accuracy
                int32_t sampleCell = mCollision.findCell(sampleHead);
                if (sampleCell < 0) {
                    // Outside world geometry — lean can't extend this far
                    safeLean = std::min(safeLean, std::max(0.0f,
                        sampleDist - sphereR));
                    break;
                }

                std::vector<SphereContact> contacts;

                // Test against world geometry
                mCollision.sphereVsCellPolygons(
                    sampleHead, sphereR, sampleCell, contacts);
                // Also check body center's cell if different (portal boundary)
                if (sampleCell != mCellIdx) {
                    mCollision.sphereVsCellPolygons(
                        sampleHead, sphereR, mCellIdx, contacts);
                }

                // Test against objects (doors, crates, furniture)
                if (mObjectWorld) {
                    mObjectWorld->testPlayerSpheres(
                        &sampleHead, &sphereR, 1, sampleCell, contacts);
                }

                if (!contacts.empty()) {
                    // Find maximum pushback opposing the lean at this sample.
                    // Project each contact's push (normal * penetration) onto the
                    // negative lean direction — how much the wall opposes the lean.
                    float maxPushback = 0.0f;
                    for (const auto &c : contacts) {
                        float pushInLeanDir =
                            glm::dot(c.normal * c.penetration, -leanDir);
                        if (pushInLeanDir > maxPushback)
                            maxPushback = pushInLeanDir;
                    }
                    // Safe lean at this sample: distance minus pushback minus margin.
                    // Minimum across all samples handles thin objects detected only
                    // at one intermediate position.
                    float safeLeanHere = sampleDist - maxPushback - LEAN_WALL_MARGIN;
                    safeLean = std::min(safeLean, safeLeanHere);
                }
            }

            mLeanAmount = std::max(0.0f, safeLean) * leanSign;
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
    /// TODO: Use to implement a rate-independent spring for the head — would require retuning spring
    /// parameters (currently tuned for direct discrete) but would eliminate dt-dependent behavior.
    /// This is currently unused.
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
