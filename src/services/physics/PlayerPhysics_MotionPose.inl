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

        // Snapshot the CURRENT interpolated offsets as the blend-start anchor so the
        // per-step blend is a true linear lerp from start to end. A mid-blend
        // re-activation therefore cleanly restarts from wherever the blend has
        // reached — no cumulative under-shoot from chasing a moving target.
        mPoseStart = mPoseCurrent;
        mBodyPoseStart = mBodyPoseCurrent;

        mPoseEnd = newTarget;
        mBodyPoseEnd = newBodyTarget;
        mPoseDuration = pose.duration;
        mPoseHoldTime = pose.holdTime;
        mPoseTimer = 0.0f;
        mPoseHolding = false;

        // Re-anchor the virtualised head spring's virt-grid to this activation
        // so the spring sees the new pose target immediately. Without this,
        // the pre-activation virt-next persists for up to SPRING_NATIVE_DT,
        // and on large target changes (crouch enter/exit) the camera visibly
        // continues the old trajectory for that window before reversing.
        // No-op at physicsDt = SPRING_NATIVE_DT (accumulator is already 0).
        snapSpringVirtualGridToNow();
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
        //
        // Sound gating matches the Dark Engine convention: only Stand/Crouch on ground
        // produce audible footsteps. Jump/Climb/etc. are silent — the original engine
        // activates stride motion separately from playing the sound, and only the Stand
        // and Crouch modes (plus Swim, which uses a different sound) have an audible
        // case. The stride motion (head-bob) still fires here — only the callback is gated.
        bool audibleStride = (mCurrentMode == PlayerMode::Stand ||
                              mCurrentMode == PlayerMode::Crouch) &&
                             isOnGround();
        if (mStepLog && audibleStride) {
            std::fprintf(stderr, "[FOOTSTEP] stride at simT=%.3f pos=(%.1f,%.1f,%.1f) hSpd=%.1f tex=%d\n",
                mSimTime, mPosition.x, mPosition.y, mPosition.z,
                horizontalSpeed(), mGroundTextureIdx);
        }
        if (mFootstepCb && audibleStride) {
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



    /// Rate-independent blend-fraction helper — reproduces the Dark Engine's
    /// 12.5Hz cumulative pose-blend shape at any physics rate.
    ///
    /// The native formula
    ///   cur_{n+1} = cur_n + (1 - cur_n) * ((n+1) * dt / dur)
    /// produces a front-loaded S-curve (fast initial rise, gentle settle)
    /// when iterated at dt=0.08s. Iterated at dt=0.0167s (60Hz) it snaps to
    /// target in ~100ms instead of 600ms — losing the 12.5Hz character we
    /// want to preserve.
    ///
    /// Closed form of the recurrence (with k = dur/dt_native):
    ///   1 - cur_n = ∏_{i=1}^{n} (1 - i/k)
    /// We evaluate this at wall-clock time regardless of physics rate by
    /// letting n index virtual 12.5Hz steps and interpolating via cubic
    /// Hermite between them (central-difference slopes; forward-difference
    /// at the n=0 boundary). C¹ smooth — no slope kinks at virtual-step
    /// boundaries — which matters because the underdamped head-bob spring
    /// would otherwise ring at its natural ~4Hz on every kink, producing
    /// HF jitter on the main bob at 60 Hz. Exact at sample points, so at
    /// 12.5Hz this degenerates to the original discrete behaviour.
    ///
    /// Caller applies `cur = mix(blendStart, blendEnd, frac)`.
    static inline float cumulativeBlendFrac(float timeActive, float duration) {
        constexpr float DT_NATIVE = 0.08f;   // 12.5Hz native physics step
        if (duration <= 0.0f || timeActive >= duration) return 1.0f;

        const float k        = duration / DT_NATIVE;
        const float virtStep = timeActive / DT_NATIVE;
        const int   n        = static_cast<int>(std::floor(virtStep));
        const float frac     = virtStep - static_cast<float>(n);

        // Walk the product up to virtual step n, caching the previous partial
        // so om_prev = omc(n-1) is available without a divide. omc(0) = 1.
        float om_prev = 1.0f;
        float om_n    = 1.0f;
        for (int i = 1; i <= n; ++i) {
            om_prev = om_n;
            om_n   *= (1.0f - static_cast<float>(i) / k);
        }
        const float om_np1 = om_n   * (1.0f - static_cast<float>(n + 1) / k);
        const float om_np2 = om_np1 * (1.0f - static_cast<float>(n + 2) / k);

        // Slopes in virtual-step parameter space. Forward difference at the
        // n=0 boundary (no omc(-1)); central difference elsewhere.
        const float m_n   = (n == 0) ? (om_np1 - om_n)
                                     : (om_np1 - om_prev) * 0.5f;
        const float m_np1 = (om_np2 - om_n) * 0.5f;

        // Cubic Hermite basis for t = frac in [0, 1].
        const float t2  = frac * frac;
        const float t3  = t2 * frac;
        const float h00 =  2.0f * t3 - 3.0f * t2 + 1.0f;
        const float h10 =         t3 - 2.0f * t2 + frac;
        const float h01 = -2.0f * t3 + 3.0f * t2;
        const float h11 =         t3 -        t2;

        const float omcSmooth = h00 * om_n   + h10 * m_n
                              + h01 * om_np1 + h11 * m_np1;
        return 1.0f - omcSmooth;
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
            // Rate-independent blend reproducing the Dark Engine's 12.5Hz
            // cumulative pose-blend shape (front-loaded S-curve). The native
            // formula is severely rate-dependent — run at 60Hz it snaps to
            // target far faster than at 12.5Hz — so cumulativeBlendFrac()
            // virtualises the discrete recurrence at 12.5Hz and samples it
            // at wall-clock time, producing the same camera trajectory at
            // any physics rate. mPoseStart/mBodyPoseStart were captured at
            // activatePose() so mid-blend re-activations restart cleanly
            // from the interpolated point rather than chasing a moving
            // target.
            if (mPoseTimer >= mPoseDuration) {
                mPoseCurrent = mPoseEnd;
                mBodyPoseCurrent = mBodyPoseEnd;
                mPoseHolding = true;
                mPoseTimer = 0.0f;
            } else {
                float blendFrac = cumulativeBlendFrac(mPoseTimer, mPoseDuration);
                mPoseCurrent     = glm::mix(mPoseStart,     mPoseEnd,     blendFrac);
                mBodyPoseCurrent = glm::mix(mBodyPoseStart, mBodyPoseEnd, blendFrac);
            }
        }

        // ── Absolute-position stride system (Dark Engine convention) ──
        // Track the 3D foot position at the last stride and compute distance
        // to the current foot position each frame. Strides fire when this
        // distance exceeds the velocity-dependent threshold. The position
        // persists through brief airborne moments, preventing double footsteps
        // on stairs.
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
        // Stride distance check — fires when the foot has traveled past the
        // velocity-dependent threshold from the last stride's foot position.
        else if (speed > 1.0f && (mLastFootTime < 0.0f || curDist2 > footstepDist * footstepDist)) {
            bool onGround = isOnGround();

            // Update foot location/time — but NOT in Jump mode, and Stand/Crouch
            // require on_ground.
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
        // Rest interrupt — Dark Engine convention: fire rest either when the
        // foot has travelled past half the stride distance since the last
        // stride, OR when the player is slowing to a stop (speed < 1.5). The
        // half-distance branch produces the stride→rest→stride "double-pump"
        // that gives walk its heavier amplitude than run — at run speeds the
        // next stride pre-empts rest before it can settle, so the double-pump
        // collapses naturally. Removing this branch makes walk feel as light
        // as run, losing the speed-dependent bob character.
        else if (mLastFootTime >= 0.0f &&
                 (mSimTime - mLastFootTime) > 0.1f &&
                 speed > 0.1f &&
                 (curDist2 > (footstepDist * 0.5f) * (footstepDist * 0.5f) ||
                  speed < 1.5f)) {
            activatePose(restPose);
        }
        // Idle — return to rest pose when not walking and pose is ready
        else if (poseReady && !isLeaning) {
            Vector3 restTarget(restPose.fwd, restPose.lat, restPose.vert);
            if (glm::length(mPoseEnd - restTarget) > 0.01f) {
                activatePose(restPose);
            }
        }

        // Velocity stop: reset foot time when velocity drops below 1.0 so that
        // the next movement triggers an immediate stride (via mLastFootTime < 0).
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

        // Lateral offset from spring lateral displacement (stride sway + lean).
        // Stride sway produces ~0.2° roll (imperceptible); lean (±2.2u) produces
        // full ~5° roll. Wall collisions feed back via the body collision pipeline:
        // when the head submodel's swept sphere hits a wall it pushes the body
        // away, so leaning into a wall slides the body rather than clamping the
        // visible offset against the wall surface.
        if (std::fabs(mSpringPos.y) > 0.001f) {
            eye.x += sinYaw * mSpringPos.y;
            eye.y -= cosYaw * mSpringPos.y;
        }

        return eye;
    }

    /// Compute raw lean tilt — camera roll proportional to spring lateral displacement.
    /// Stride sway (±0.1u) → ~0.2° roll (imperceptible); lean (±2.2u) → full ~5° roll.
    /// On key release, spring decays to zero → smooth roll return.
    inline float computeRawLeanTilt() const {
        float maxDist = isCrouching() ? CROUCH_LEAN_DISTANCE : LEAN_DISTANCE;
        if (maxDist < 0.01f) return 0.0f;
        return (mSpringPos.y / maxDist) * LEAN_TILT;
    }

    /// Compute the pose-blend target at an arbitrary wall-clock time — used
    /// when advancing the virtualised spring at a 12.5 Hz virtual sample
    /// boundary that doesn't coincide with the current physics step.
    ///
    /// Returns mPoseEnd when the query time is past the current blend's
    /// duration, mPoseStart when it's before activation, and the smooth
    /// Hermite-interpolated blend value in between. The activation time of
    /// the current blend is (mSimTime - mPoseTimer); queryTime may be in
    /// either direction from mSimTime.
    ///
    /// IMPORTANT: when mPoseHolding is true, the pose is at mPoseEnd. The
    /// blend-complete transition resets mPoseTimer to 0 which makes
    /// activationTime == mSimTime, so any past queryTime would otherwise
    /// map to a negative relativeTime and return the stale mPoseStart
    /// (producing spurious wrong-direction spring motion on the virt-step
    /// immediately after hold starts). The holding-branch short-circuit
    /// is the fix.
    inline Vector3 poseTargetAtTime(float queryTime) const {
        if (mPoseHolding) return mPoseEnd;
        const float activationTime = mSimTime - mPoseTimer;
        const float relativeTime   = queryTime - activationTime;
        if (relativeTime <= 0.0f) return mPoseStart;
        if (relativeTime >= mPoseDuration || mPoseDuration <= 0.0f)
            return mPoseEnd;
        const float frac = cumulativeBlendFrac(relativeTime, mPoseDuration);
        return glm::mix(mPoseStart, mPoseEnd, frac);
    }

    /// Snap the virtualised head spring's virtual grid to the current
    /// wall-clock time. Captures the currently-displayed spring position
    /// as the new Prev sample, recomputes the Prev velocity from the
    /// spring formula against the NEW pose target, then advances one
    /// virtual step to populate Next. Called by activatePose() so the
    /// spring responds to a new pose target immediately instead of
    /// continuing the pre-activation virt-next for up to SPRING_NATIVE_DT.
    ///
    /// Why the Prev velocity is recomputed (not captured from the Hermite
    /// derivative as it once was): the Hermite derivative describes the
    /// SHAPE of the prior interpolation curve, not the spring's physical
    /// response to the current target. When a pose activation reverses
    /// target direction (e.g. stride→rest at every footfall), the
    /// derivative still points in the OLD direction. The first-virt-span
    /// Hermite curve then blends an old-direction Prev velocity with a
    /// new-direction Next velocity, producing a small (~0.001 u) extremum
    /// bump that reads as visible jitter at footfall. Recomputing Prev
    /// velocity from the formula at the new target makes Prev/Next
    /// velocities co-directional, so the curve is monotonic.
    ///
    /// Position continuity is preserved exactly (Prev pos = display pos);
    /// velocity is allowed to slope-kink at the snap moment, which is
    /// imperceptible at the small target deltas of stride/rest cycling.
    ///
    /// At physicsDt = SPRING_NATIVE_DT this is effectively a no-op — the
    /// accumulator is already 0 right after each virt-step advance, so
    /// snapping captures the state that was just-computed. The correction
    /// scales up with physics rate as required by the design principle in
    /// feedback_exact_player_dynamics.md.
    inline void snapSpringVirtualGridToNow() {
        // Recompute Prev velocity from the spring formula against the new
        // target (evaluated at the snap moment, not 80 ms ahead). The
        // existing mSpringNextVel — the formula's output for the prior
        // virt step — is the best available "old velocity" input; with
        // damping retention ~0.07 it contributes only weakly, so the new
        // velocity is dominated by the displacement-toward-target term and
        // points the right way under any target reversal.
        const Vector3 targetNow = poseTargetAtTime(mSimTime);
        const Vector3 newPrevVel = computeSpringVelocity(
            mSpringPos, mSpringNextVel, targetNow,
            HEAD_SPRING_BASE_TENSION, HEAD_SPRING_BASE_DAMPING,
            HEAD_SPRING_Z_SCALE, HEAD_SPRING_VEL_CAP, SPRING_NATIVE_DT);

        // Prime mSpringNext so that advanceSpringVirtualStep's next→prev
        // shift preserves our captured state as the new Prev. The accumulator
        // reset anchors the virt grid to now: new Prev is at mSimTime, new
        // Next will be at mSimTime + SPRING_NATIVE_DT. advance internally
        // evaluates the pose target at new-next's time, matching the 12.5 Hz
        // discrete semantics.
        mSpringNextPos = mSpringPos;   // display value from last updateHeadSpring
        mSpringNextVel = newPrevVel;
        mSpringAccum   = 0.0f;
        advanceSpringVirtualStep(mSimTime);
    }

    /// Advance the virtual spring state (mSpringPrev → mSpringNext) by one
    /// 12.5 Hz native step. Uses the original Dark Engine discrete formula
    /// (computeSpringVelocity + pos += vel*dt) with dt = SPRING_NATIVE_DT.
    ///
    /// The target is evaluated at the NEW NEXT's wall-clock time
    /// (newPrevTime + SPRING_NATIVE_DT), matching the 12.5 Hz discrete
    /// semantics where each tick advances velocity using the pose target
    /// that has been updated this tick (i.e., the pose at the end of the
    /// step, not the start). Using the start-of-step target instead would
    /// cause two consecutive virt steps after a pose activation to reuse
    /// the same target — stalling the spring's response for one virt period
    /// and producing a visible lateral/vertical bump when the Hermite
    /// interpolant between prev (accelerating) and next (near-stationary)
    /// curves into a local extremum.
    inline void advanceSpringVirtualStep(float newPrevTime) {
        // Shift: current next becomes new prev; we'll compute a new next.
        mSpringPrevPos = mSpringNextPos;
        mSpringPrevVel = mSpringNextVel;

        const float   newNextTime = newPrevTime + SPRING_NATIVE_DT;
        const Vector3 target      = poseTargetAtTime(newNextTime);

        mSpringNextVel = computeSpringVelocity(
            mSpringPrevPos, mSpringPrevVel, target,
            HEAD_SPRING_BASE_TENSION, HEAD_SPRING_BASE_DAMPING,
            HEAD_SPRING_Z_SCALE, HEAD_SPRING_VEL_CAP, SPRING_NATIVE_DT);
        mSpringNextPos = mSpringPrevPos + mSpringNextVel * SPRING_NATIVE_DT;
    }

    /// Update 3D head spring — virtualised at 12.5 Hz.
    ///
    /// The spring state advances at SPRING_NATIVE_DT (0.08 s) regardless of
    /// the physics rate. Between virtual samples the displayed mSpringPos is
    /// cubic-Hermite interpolated using Prev/Next position and velocity — C¹
    /// smooth, with no continuous-ODE wrong-direction artifacts on target
    /// reversals. At physicsDt = SPRING_NATIVE_DT this is a no-op (identity
    /// with the original formula run at physics rate).
    ///
    /// Rationale: running the original rate-dependent discrete formula at
    /// 60 Hz produces different dynamics than 12.5 Hz; fitting an analytical
    /// continuous ODE preserves the 12.5 Hz samples but introduces between-
    /// sample wrong-direction motion that reads as HF jitter at 60 Hz.
    /// Virtualising at the native rate and interpolating smoothly for
    /// display avoids both failure modes. See feedback_exact_player_dynamics.md.
    inline void updateHeadSpring() {
        mSpringAccum += mTimestep.fixedDt;

        // Advance as many virtual steps as the accumulator has crossed. At
        // physics rates slower than 12.5 Hz this loop runs multiple times
        // per physics step; at 12.5 Hz it runs exactly once; at 60/120 Hz
        // it typically runs 0 times most steps and 1 time every 4-5 steps.
        while (mSpringAccum >= SPRING_NATIVE_DT) {
            mSpringAccum -= SPRING_NATIVE_DT;
            // Wall-clock time of the virtual sample we're computing. It sits
            // in the past relative to mSimTime by mSpringAccum seconds (the
            // residual after subtracting this step's SPRING_NATIVE_DT).
            const float virtSampleTime = mSimTime - mSpringAccum;
            advanceSpringVirtualStep(virtSampleTime);
        }

        // Cubic Hermite interpolation for display. Parameter s in [0, 1) is
        // how far into the current virtual interval the wall clock sits.
        // Velocities are scaled by SPRING_NATIVE_DT so the Hermite basis
        // matches standard [0,1] parameterisation.
        //
        // Per-axis Fritsch–Carlson monotonic clipping is applied to the
        // endpoint velocities BEFORE the Hermite eval. The original 12.5 Hz
        // spring formula can produce |V_prev| ≫ |V_next| when target reverses
        // (e.g. stride→rest at a footfall), and the unclipped Hermite curve
        // then dips a fraction of a unit past the Next sample before climbing
        // back to it — visible as a small jitter at every bob extremum.
        // Clipping limits the slope magnitudes so the curve stays monotonic
        // between Prev and Next when the data itself is monotonic, while
        // leaving non-monotonic spans (e.g. across a true bob extremum)
        // untouched. Sample positions and velocities at virt-grid points are
        // unchanged — only the between-sample curvature is constrained.
        const float s  = mSpringAccum / SPRING_NATIVE_DT;
        const float s2 = s * s;
        const float s3 = s2 * s;
        const float h00 =  2.0f * s3 - 3.0f * s2 + 1.0f;
        const float h10 =         s3 - 2.0f * s2 + s;
        const float h01 = -2.0f * s3 + 3.0f * s2;
        const float h11 =         s3 -        s2;

        Vector3 prevVelClipped = mSpringPrevVel;
        Vector3 nextVelClipped = mSpringNextVel;
        for (int i = 0; i < 3; ++i) {
            const float deltaPos = mSpringNextPos[i] - mSpringPrevPos[i];
            // Skip when endpoints coincide — secant slope is zero/undefined,
            // any velocity is "non-monotonic" and the Hermite stays close to
            // the shared endpoint regardless.
            if (std::fabs(deltaPos) < 1e-6f) continue;
            const float secant = deltaPos / SPRING_NATIVE_DT;
            const float alpha  = mSpringPrevVel[i] / secant;
            const float beta   = mSpringNextVel[i] / secant;
            // Same-sign-as-secant test: if either endpoint velocity opposes
            // the secant, the segment crosses an extremum and Hermite must
            // be free to curve through it.
            if (alpha < 0.0f || beta < 0.0f) continue;
            const float r2 = alpha * alpha + beta * beta;
            if (r2 > 9.0f) {
                const float tau = 3.0f / std::sqrt(r2);
                prevVelClipped[i] = tau * mSpringPrevVel[i];
                nextVelClipped[i] = tau * mSpringNextVel[i];
            }
        }

        mSpringPos = h00 * mSpringPrevPos
                   + h10 * prevVelClipped * SPRING_NATIVE_DT
                   + h01 * mSpringNextPos
                   + h11 * nextVelClipped * SPRING_NATIVE_DT;

        // Clamp to sane range — prevents runaway on large target jumps.
        // ±4.0 accommodates crouch transition (2.02 units) plus overshoot.
        mSpringPos = glm::clamp(mSpringPos, Vector3(-4.0f), Vector3(4.0f));

        // Expose the virt-next velocity so the per-step diagnostic log has
        // something coherent to record; displayed motion derives from Prev/Next
        // and the Hermite basis, not from this single velocity snapshot.
        mSpringVel = mSpringNextVel;
    }
