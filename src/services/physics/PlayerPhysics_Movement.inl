// Included inside PlayerPhysics class body — do not include standalone.

    /// Apply gravity to velocity. Matches original engine's UpdateModelTransDynamics
    /// (phcore.cpp lines 1417-1421) which applies gravity EVERY frame unconditionally
    /// (except climbing/mantling). Floor contacts remove the downward component via
    /// constrainVelocity() (matching original's ApplyConstraints at line 1861).
    /// The downward velocity component during the frame is essential for swept collision
    /// to detect stair risers below the foot — without it, the swept path is purely
    /// horizontal and misses the riser wall.
    ///
    /// Note on pre-constraint vz: when the player is stationary on flat ground with
    /// no input, applyMovement()'s friction branch runs between gravity and constraint.
    /// That branch applies a 3D friction force with a 1.4× Z-boost which partially
    /// cancels the gravity impulse, so the vz reaching constrainVelocity() is smaller
    /// than gravity×dt. At 60Hz with default constants the pre-constraint vz is
    /// ≈ −0.354 rather than the raw −0.533 (≈ 2/3 ratio). The constraint then zeroes
    /// whichever residual arrives. This three-layer cancellation (gravity → friction
    /// → constraint) is the original engine's design, not a scaling artefact.
    inline void applyGravity() {
        // No gravity while climbing or mantling. Climbing friction handles wall traction instead.
        if (mCurrentMode == PlayerMode::Climb || mMantling)
            return;

        // Original (PHCORE.CPP lines 1413-1421): gravity added to sum_forces.
        // When in water, ALL forces halved first (line 1413: sum_forces *= 0.5).
        // Then buoyancy added (line 1424-1427).
        float gravForce = mGravityMag;
        float buoyForce = 0.0f;
        if (isInWater()) {
            // D8: Water force halving — original halves ALL forces in water
            gravForce *= 0.5f;
            // Buoyancy: (1/density) * gravity * mass / mass = (1/density) * gravity
            buoyForce = (GRAVITY / mDensity) * 0.5f;  // also halved by water
        }
        mVelocity.z -= gravForce * mTimestep.fixedDt;
        if (buoyForce > 0.0f) {
            mVelocity.z += buoyForce * mTimestep.fixedDt;
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

        // Clamp diagonal movement: when forward + strafe are combined,
        // the vector magnitude exceeds the forward speed (sqrt(fwd²+str²)).
        // Cap to forward speed so diagonal is never faster than straight.
        // Inspired by Godot's Input::get_vector() unit-circle clamping.
        float mag = glm::length(desired);
        if (mag > forwardSpeed)
            desired *= (forwardSpeed / mag);

        return desired;
    }

    /// Horizontal speed magnitude (XY plane only).
    inline float horizontalSpeed() const {
        return std::sqrt(mVelocity.x * mVelocity.x + mVelocity.y * mVelocity.y);
    }

    /// Compute ground friction from upward-facing contacts. Sums friction from all contacts
    /// (BODY + FOOT on same surface → additive, pushing control rate to ~2000 cap on walkable
    /// slopes). Falls back to mGroundNormal if no contacts (stable standing), then to
    /// WATER_BASE_FRICTION in swim mode. Returns 0 in air.
    inline float computeGroundFriction() const {
        // Sum friction from all upward-facing contacts. Friction is continuous and
        // proportional to normal.z (original engine formula: 0.03 * normal.z * 32).
        // Steep slopes contribute less friction naturally — no hard cutoff needed.
        // Gravity overcomes friction on slopes steeper than ~45°, causing sliding.
        float totalFriction = 0.0f;
        for (const auto &c : mContacts) {
            if (c.normal.z > 0.0f) {
                float terrainScale = getTerrainFriction(c.textureIdx);
                totalFriction += FRICTION_FACTOR * terrainScale * c.normal.z * mGravityMag;
            }
        }

        // Fallback: use mGroundNormal from detectGround()'s probe when player stands stably
        // without generating collision contacts.
        if (totalFriction == 0.0f && isOnGround() && mGroundNormal.z > GROUND_NORMAL_MIN) {
            totalFriction = FRICTION_FACTOR * mGroundNormal.z * mGravityMag;
        }

        // Climbability boosts friction on steep surfaces.
        // With climbability >= 1.0, friction is raised to full ground-level friction,
        // allowing the player to walk on otherwise un-walkable steep slopes.
        // This is NOT a climb-mode trigger — just a friction multiplier.
        float maxFriction = FRICTION_FACTOR * mGravityMag;
        if (totalFriction < maxFriction) {
            float climbFactor = 0.0f;
            for (const auto &c : mContacts) {
                if (c.normal.z > 0.0f)
                    climbFactor = std::max(climbFactor, getTerrainClimbability(c.textureIdx));
            }
            if (climbFactor > 0.0f)
                totalFriction += (maxFriction - totalFriction) * std::min(climbFactor, 1.0f);
        }

        // Water base friction provides drag when submerged with no ground contacts
        if (totalFriction == 0.0f && mCurrentMode == PlayerMode::Swim) {
            totalFriction = WATER_BASE_FRICTION;
        }
        return totalFriction;
    }

    /// Per-terrain friction multiplier from P$Friction on texture archetypes in dark.gam.
    /// Lookup table built at mission load (setFrictionTable). Default 1.0 for textures
    /// without explicit friction or if table not loaded.
    inline float getTerrainFriction(int32_t textureIdx) const {
        if (textureIdx >= 0 && textureIdx < static_cast<int32_t>(mFrictionTable.size()))
            return mFrictionTable[textureIdx];
        return 1.0f;
    }

    /// Per-terrain climbability multiplier from P$Climbabil on texture archetypes.
    /// Climbability boosts friction on steep surfaces — NOT a climb-mode trigger.
    /// Default 0.0 for textures without explicit climbability or if table not loaded.
    inline float getTerrainClimbability(int32_t textureIdx) const {
        if (textureIdx >= 0 && textureIdx < static_cast<int32_t>(mClimbabilityTable.size()))
            return mClimbabilityTable[textureIdx];
        return 0.0f;
    }

    /// Apply movement — matches original UpdateModelTransDynamics + ControlVelocity.
    ///
    /// Original flow (PHCORE.CPP lines 1349-1509, PHCTRL.CPP lines 130-220):
    ///   1. ControlVelocity() computes ctrl_accel = (desired - current) * rate
    ///   2. ctrl_accel added to sum_forces (acceleration accumulator)
    ///   3. Gravity added to sum_forces
    ///   4. ideal_velocity = velocity + (sum_forces / mass) * dt
    ///   5. If velocity-controlled: NO friction (actual = ideal)
    ///   6. If NOT controlled: friction applied, actual = velocity + (sum_forces_with_friction / mass) * dt
    ///   7. Reversal check: if dot(actual, ideal) < 0 → actual = 0
    ///
    /// Key: friction is ONLY applied when player has NO movement input (D16).
    /// With active input, control acceleration handles all speed regulation.
    inline void applyMovement() {
        Vector3 desired = computeDesiredVelocity();
        const float dt = mTimestep.fixedDt;

        // Compute friction from all ground contacts
        float friction = computeGroundFriction();

        // Determine if player has active velocity control (movement input)
        // Original: IsVelocityControlled() returns true when any axis has input
        bool velocityControlled = (std::fabs(mInputForward) > 0.001f ||
                                   std::fabs(mInputRight) > 0.001f);

        if (isOnGround() && !mGroundGraceActive) {
            // Project desired velocity onto ground plane so walking on slopes
            // doesn't fight gravity.
            if (mGroundNormal.z > GROUND_NORMAL_MIN) {
                float dot = glm::dot(desired, mGroundNormal);
                desired -= mGroundNormal * dot;
            }

            // Control acceleration: rate * (desired - current)
            // Original (PHCTRL.CPP line 211): rate = 11 * mass * friction / velocity_rate
            float rate = CONTROL_MULTIPLIER * mMass * friction / VELOCITY_RATE;
            rate = std::min(rate, MAX_CONTROL_RATE);

            float prevZ = mVelocity.z;
            Vector3 hVel(mVelocity.x, mVelocity.y, 0.0f);
            Vector3 hDesired(desired.x, desired.y, 0.0f);

            // Control acceleration added to forces (matching original PHCTRL.CPP line 214)
            Vector3 ctrlAccel = (hDesired - hVel) * rate;

            // Integrate forces: ideal_velocity = velocity + (ctrl_accel / mass) * dt
            Vector3 hIdeal = hVel + ctrlAccel * (dt / mMass);

            Vector3 hActual;
            if (velocityControlled) {
                // Original (PHCORE.CPP line 1465): friction SKIPPED when velocity-controlled.
                // Control acceleration alone regulates speed toward desired.
                hActual = hIdeal;
            } else {
                // No input — apply friction to decelerate.
                // Original PHCORE.CPP: friction is 3D, opposes ideal_velocity direction,
                // magnitude = friction_amt * mass * gravity * drag_scale
                float frictionMag = friction * mMass * mGravityMag;

                // Velocity-dependent drag (D3): scale = clamp(50 * |vel| / mass, 0.25, 10)
                // Original uses PRE-gravity velocity magnitude here (sum_forces hasn't
                // been integrated yet when this scale is computed). Matching that so
                // drag isn't inflated by the current frame's gravity kick during descent.
                Vector3 vel3D(hVel.x, hVel.y, prevZ);
                float dragScale = std::clamp(
                    50.0f * glm::length(mPreGravityVelocity) / mMass, 0.25f, 10.0f);
                frictionMag *= dragScale;

                // Use 3D ideal velocity for friction direction (original is fully 3D)
                Vector3 ideal3D(hIdeal.x, hIdeal.y, prevZ);

                if (glm::length2(ideal3D) > 1e-8f) {
                    Vector3 frictionForce = -glm::normalize(ideal3D) * frictionMag;

                    // Z-axis friction boost (D4): PHCORE.CPP lines 1474-1477
                    bool onPlatform = (mPlatformObjID != 0);
                    if (onPlatform && ideal3D.z < 0.0f)
                        frictionForce.z = 0.0f;
                    else
                        frictionForce.z *= 1.4f;

                    Vector3 actual3D = vel3D + (Vector3(ctrlAccel.x, ctrlAccel.y, 0.0f) + frictionForce) * (dt / mMass);

                    // Reversal check (D2): if friction reversed direction, zero velocity
                    if (glm::dot(actual3D, ideal3D) < 0.0f)
                        actual3D = Vector3(0.0f);

                    hActual = Vector3(actual3D.x, actual3D.y, 0.0f);
                    prevZ = actual3D.z;
                } else {
                    hActual = Vector3(0.0f);
                    prevZ = 0.0f;
                }
            }

            mVelocity.x = hActual.x;
            mVelocity.y = hActual.y;
            mVelocity.z = desired.z != 0.0f ? desired.z : prevZ;
        } else if (isOnGround()) {
            // Grace period: same control model but no slope projection.
            float rate = CONTROL_MULTIPLIER * mMass * friction / VELOCITY_RATE;
            rate = std::min(rate, MAX_CONTROL_RATE);

            Vector3 hVel(mVelocity.x, mVelocity.y, 0.0f);
            Vector3 hDesired(desired.x, desired.y, 0.0f);
            Vector3 ctrlAccel = (hDesired - hVel) * rate;
            Vector3 hIdeal = hVel + ctrlAccel * (dt / mMass);

            Vector3 hActual;
            if (velocityControlled) {
                hActual = hIdeal;
            } else {
                // 3D friction with Z-axis boost, same as main ground path
                float frictionMag = friction * mMass * mGravityMag;
                float prevZ = mVelocity.z;
                Vector3 vel3D(hVel.x, hVel.y, prevZ);
                // Pre-gravity velocity magnitude — see main ground path for rationale.
                float dragScale = std::clamp(
                    50.0f * glm::length(mPreGravityVelocity) / mMass, 0.25f, 10.0f);
                frictionMag *= dragScale;

                Vector3 ideal3D(hIdeal.x, hIdeal.y, prevZ);

                if (glm::length2(ideal3D) > 1e-8f) {
                    Vector3 frictionForce = -glm::normalize(ideal3D) * frictionMag;

                    // Z-axis friction boost (D4): PHCORE.CPP lines 1474-1477
                    bool onPlatform = (mPlatformObjID != 0);
                    if (onPlatform && ideal3D.z < 0.0f)
                        frictionForce.z = 0.0f;
                    else
                        frictionForce.z *= 1.4f;

                    Vector3 actual3D = vel3D + (Vector3(ctrlAccel.x, ctrlAccel.y, 0.0f) + frictionForce) * (dt / mMass);
                    if (glm::dot(actual3D, ideal3D) < 0.0f)
                        actual3D = Vector3(0.0f);

                    hActual = Vector3(actual3D.x, actual3D.y, 0.0f);
                    mVelocity.z = actual3D.z;
                } else {
                    hActual = Vector3(0.0f);
                    mVelocity.z = 0.0f;
                }
            }

            mVelocity.x = hActual.x;
            mVelocity.y = hActual.y;
        } else if (mCurrentMode == PlayerMode::Swim) {
            // ── Swimming: 3D movement with look-direction Z control ──
            // Unlike air/ground which strip Z, swimming uses camera pitch for 3D movement.
            // forward3D = (cos(yaw)*cos(pitch), sin(yaw)*cos(pitch), sin(pitch))
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

            // Clamp diagonal swim speed to forward speed
            float swimMag = glm::length(swimDesired);
            if (swimMag > fwdSpeed)
                swimDesired *= (fwdSpeed / swimMag);

            // Flat friction (no velocity-dependent drag — that's in the friction-force path, bypassed
            // for velocity-controlled objects). Swim control rate: 11 * 180 * 0.3 / 1.0 = 594.
            float rate = CONTROL_MULTIPLIER * mMass * WATER_BASE_FRICTION / VELOCITY_RATE;
            rate = std::min(rate, MAX_CONTROL_RATE);
            float alpha = rate * WATER_CONTROL_SCALE * dt / mMass;
            alpha = std::min(alpha, 1.0f);

            // 3D velocity convergence (no Z stripping — full 3D control)
            mVelocity += (swimDesired - mVelocity) * alpha;

        } else if (mCurrentMode == PlayerMode::Climb) {
            // ── Climbing: movement follows player's look direction ──
            // The original engine uses the player's combined head+body facing
            // direction. When looking upward while pressing forward, the control
            // velocity has both an upward Z component and a forward-into-wall
            // component. Gravity is suppressed (handled in applyGravity).

            // Look up the climbing OBB (needed for non-climbable face pushback)
            const ObjectCollisionBody *body = mObjectWorld
                ? mObjectWorld->findBodyByObjID(mClimbingObjId) : nullptr;

            // Player's 3D look direction from yaw + pitch (gravity-stripping
            // is bypassed for climbers — the vertical component is preserved)
            float cosPitch = std::cos(mCamPitch);
            float sinPitch = std::sin(mCamPitch);
            Vector3 lookDir(mCosYaw * cosPitch, mSinYaw * cosPitch, sinPitch);

            // Right vector (horizontal only, perpendicular to yaw)
            Vector3 rightDir(mSinYaw, -mCosYaw, 0.0f);

            // Speed: uses "slow" base speeds (half of normal)
            // for climbing input, then the 0.5× mode scale halves again.
            // Effective: forward=2.75, backward=1.375, strafe=1.925 u/s.
            float modeScale = MODE_SPEEDS[static_cast<int>(mCurrentMode)].trans;
            float fwdSpeed = CLIMB_FWD_SPEED * modeScale;
            float strSpeed = CLIMB_STR_SPEED * modeScale;
            if (mInputForward < 0.0f) fwdSpeed = CLIMB_BACK_SPEED * modeScale;
            if (mSneaking)      { fwdSpeed *= 0.5f; strSpeed *= 0.5f; }
            else if (mRunning)  { fwdSpeed *= 2.0f; strSpeed *= 2.0f; }

            Vector3 climbDesired = lookDir  * (mInputForward * fwdSpeed)
                                 + rightDir * (mInputRight   * strSpeed);

            // Clamp diagonal climb speed to forward speed
            float climbMag = glm::length(climbDesired);
            if (climbMag > fwdSpeed)
                climbDesired *= (fwdSpeed / climbMag);

            // Full ground-level friction for wall traction
            float friction = FRICTION_FACTOR * mGravityMag;
            float rate = CONTROL_MULTIPLIER * mMass * friction / VELOCITY_RATE;
            rate = std::min(rate, MAX_CONTROL_RATE);
            float alpha = rate * dt / mMass;
            alpha = std::min(alpha, 1.0f);

            mVelocity += (climbDesired - mVelocity) * alpha;

            // Wall-plane constraint: remove velocity component going into or away
            // from the climbing surface.
            float normalVel = glm::dot(mVelocity, mClimbFaceNormal);
            mVelocity -= mClimbFaceNormal * normalVel;

            // Non-climbable face pushback: if the player has moved past a non-climbable
            // face (e.g. the top of a ladder), remove velocity going further past it.
            // This prevents climbing over the top — the player must jump to dismount.
            if (body) {
                Vector3 offset = mPosition - body->worldPos;
                Vector3 halfExt = body->edgeLengths * 0.5f;
                for (int face = 0; face < 6; ++face) {
                    if (body->climbableSides & (1 << face))
                        continue;  // climbable faces don't push back
                    Vector3 faceN = getOBBFaceNormal(*body, face);
                    int ax = face % 3;
                    float proj = glm::dot(offset, body->rotation[ax]);
                    float dist = (face < 3) ? (proj - halfExt[ax]) : (-proj - halfExt[ax]);
                    if (dist > 0.0f) {
                        float vn = glm::dot(mVelocity, faceN);
                        if (vn > 0.0f)
                            mVelocity -= faceN * vn;
                    }
                }
            }

            // Zero downward velocity when desired velocity is non-negative Z.
            // The original checks control_velocity.z >= 0: if looking down while
            // pressing forward (negative climbDesired.z), descent is allowed.
            if (mVelocity.z < 0.0f && climbDesired.z >= 0.0f)
                mVelocity.z = 0.0f;

        } else {
            // Airborne: no air control.
        }
    }

    /// Direct contact transition — maintains foot-ground contact when the foot
    /// slides off one polygon onto an adjacent one. Matches original Dark Engine
    /// ConstrainFromTerrain (phcore.cpp lines 262-368).
    ///
    /// When the foot has a floor contact from the previous frame, check if the foot
    /// is still on that polygon. If not, raycast 0.1 units INTO the old surface to
    /// find the adjacent polygon. If found, create a new contact immediately —
    /// no gravity-fall-then-collide cycle needed. This is critical for smooth stair
    /// descent: the foot transitions directly from one tread to the next.
    ///
    /// Validate existing contacts each frame, matching original Dark Engine's
    /// ConstrainFromTerrain (phcore.cpp lines 262-368). For each contact in
    /// mContacts, check if the submodel is still on the polygon (distance +
    /// point-in-polygon). Valid contacts are kept for constrainVelocity().
    /// Invalid contacts are destroyed immediately (no age-based persistence).
    ///
    /// For FOOT floor contacts that fail validation, attempt a transition
    /// raycast (0.1 units along -normal) to find the adjacent floor polygon.
    /// This handles coplanar polygon-to-polygon transitions on flat surfaces.
    /// For stair descent (0.5-unit drops), the probe is too short — the contact
    /// is destroyed and gravity handles the drop naturally.
    static constexpr float kBreakTerrainContactDist = 0.1f;  // original: kBreakTerrainContactDist
    static constexpr float kContactTransitionDist = 0.1f;    // original: look_offset scale

    inline void validateContacts() {
        // Clear per-frame constraints FIRST — rebuilt from validated contacts below.
        // Matches original Dark Engine ClearConstraints() (PHMOD.H lines 1276-1282)
        // called at the start of each frame before ConstrainFromTerrain/Objects.
        // Must happen before the early return so stale constraints don't persist
        // when player is outside collision geometry.
        mConstraints.clear();

        if (mCellIdx < 0) return;

        // Iterate backwards for safe erasure
        for (int ci = static_cast<int>(mContacts.size()) - 1; ci >= 0; --ci) {
            auto &c = mContacts[ci];

            // Reset fresh flag — will be set to true again during collision
            // detection if the contact is re-detected this frame.
            c.fresh = false;

            // ── Object contact validation (ConstrainFromObjects, phcore.cpp lines 443-611) ──
            // Checks surface distance (PhysGetObjsNorm) and geometric containment
            // (SubModOnPoly with grow_poly=TRUE). Breaks contact if distance exceeds
            // kBreakObjectContactDist (0.2) or sphere has slid off the OBB face.
            // Refreshes contact normal from current OBB orientation.
            if (c.cellIdx < 0) {
                if (!mObjectWorld) {
                    mContacts.erase(mContacts.begin() + ci);
                    continue;
                }
                const auto *body = mObjectWorld->findBodyByObjID(c.objectId);
                if (!body) {
                    mContacts.erase(mContacts.begin() + ci);
                    continue;
                }

                // Compute submodel position for this contact. mSpringPos is player-local
                // (fwd, lat, vert) — rotate fwd/lat into world XY; vert is world Z.
                // mHeadClamp captures last frame's HEAD wall response so contact validation
                // tests at the actually-clamped HEAD position.
                int subIdx = std::clamp(static_cast<int>(c.submodelIdx), 0, NUM_SPHERES - 1);
                float poseOZ = 0.0f;
                Vector3 springOff(0.0f);
                if (subIdx == 0) {
                    poseOZ = mSpringPos.z;
                    springOff = Vector3(
                        mCosYaw * mSpringPos.x + mSinYaw * mSpringPos.y,
                        mSinYaw * mSpringPos.x - mCosYaw * mSpringPos.y,
                        0.0f) + mHeadClamp;
                } else if (subIdx == 1) poseOZ = mBodyPoseCurrent.z;
                float offZ = mSphereOffsetsBase[subIdx] + poseOZ;
                Vector3 subPos = mPosition + Vector3(0.0f, 0.0f, offZ) + springOff;
                float subRadius = mSphereRadii[subIdx];

                int faceIdx = c.polyIdx & 0xF;

                // Box-shaped bodies: OBB and SphereHat (the latter is a sphere
                // with a flat-plane cap in the original engine; we model it as
                // an OBB built from the model bbox).
                const bool boxLike =
                    (body->shapeType == CollisionShapeType::OBB ||
                     body->shapeType == CollisionShapeType::SphereHat);
                if (boxLike && faceIdx <= 5) {
                    // OBB face validation
                    Vector3 faceNormal = getOBBFaceNormal(*body, faceIdx);
                    int axis = faceIdx % 3;
                    float halfExtent = body->edgeLengths[axis] * 0.5f;
                    Vector3 faceCenter = body->worldPos + faceNormal * halfExtent;

                    // Surface distance: dot(sphereCenter, faceNormal) - planeConst - radius
                    // Matches original PhysGetObjsNorm → GetSphereVsOBBNormal (phmodutl.cpp lines 92-107)
                    float centerDist = glm::dot(subPos - faceCenter, faceNormal);
                    float surfaceDist = centerDist - subRadius;

                    if (surfaceDist > kBreakObjectContactDist) {
                        mContacts.erase(mContacts.begin() + ci);
                        continue;
                    }

                    // On-face check: SubModOnPoly with grow_poly=TRUE (phcore.cpp line 488)
                    // Projects sphere center onto face tangent axes, checks against
                    // halfExtent + expansion. Expansion = sphereRadius for sphere submodels.
                    Vector3 toSub = subPos - body->worldPos;
                    int axis1 = (axis + 1) % 3;
                    int axis2 = (axis + 2) % 3;
                    float d1 = glm::dot(toSub, body->rotation[axis1]);
                    float d2 = glm::dot(toSub, body->rotation[axis2]);
                    float halfE1 = body->edgeLengths[axis1] * 0.5f;
                    float halfE2 = body->edgeLengths[axis2] * 0.5f;
                    float expand = (subRadius > 0.001f) ? subRadius : 0.1f;

                    if (std::fabs(d1) > halfE1 + expand + 0.01f ||
                        std::fabs(d2) > halfE2 + expand + 0.01f) {
                        mContacts.erase(mContacts.begin() + ci);
                        continue;
                    }

                    // Refresh normal from current OBB orientation.
                    // Original recomputes via PhysGetObjsNorm each frame and uses
                    // the fresh normal for AddConstraint (phcore.cpp line 543).
                    c.normal = faceNormal;

                } else if (body->shapeType == CollisionShapeType::Sphere) {
                    // Sphere object: center-to-center distance check
                    float dist = glm::length(subPos - body->worldPos)
                                 - subRadius - body->sphereRadius;
                    if (dist > kBreakObjectContactDist) {
                        mContacts.erase(mContacts.begin() + ci);
                        continue;
                    }
                } else {
                    // Unknown shape or invalid face index — destroy
                    mContacts.erase(mContacts.begin() + ci);
                    continue;
                }

                // Contact valid — create constraint from it.
                // Matches original ConstrainFromObjects (PHCORE.CPP line 543):
                // pModel->AddConstraint(pModel2->GetObjID(), i, normal)
                mConstraints.push_back({c.normal, c.objectId});
                // SetGroundObj (D20): track ground surface for FOOT floor contacts on objects.
                // Matches original (PHCORE.CPP line 5537):
                // g_pPlayerMovement->SetGroundObj(pOBBModel->GetObjID())
                if (c.submodelIdx == 4 && c.normal.z > GROUND_NORMAL_MIN)
                    mGroundObjID = c.objectId;
                continue;
            }

            // Validate terrain contact: distance + point-in-polygon
            if (c.cellIdx >= static_cast<int32_t>(mCollision.getWR().numCells)) {
                mContacts.erase(mContacts.begin() + ci);
                continue;
            }
            const auto &cell = mCollision.getWR().cells[c.cellIdx];
            if (c.polyIdx < 0 || c.polyIdx >= cell.numPolygons) {
                mContacts.erase(mContacts.begin() + ci);
                continue;
            }

            // Compute submodel position for this contact (rotate spring fwd/lat into
            // world XY; apply mHeadClamp so HEAD contact validation matches the
            // clamped position established by last frame's collision response).
            float poseOffsetZ = 0.0f;
            Vector3 springOff2(0.0f);
            if (c.submodelIdx == 0) {
                poseOffsetZ = mSpringPos.z;
                springOff2 = Vector3(
                    mCosYaw * mSpringPos.x + mSinYaw * mSpringPos.y,
                    mSinYaw * mSpringPos.x - mCosYaw * mSpringPos.y,
                    0.0f) + mHeadClamp;
            } else if (c.submodelIdx == 1) poseOffsetZ = mBodyPoseCurrent.z;
            int subIdx = std::clamp(static_cast<int>(c.submodelIdx), 0, NUM_SPHERES - 1);
            float offsetZ = mSphereOffsetsBase[subIdx] + poseOffsetZ;
            Vector3 subPos = mPosition + Vector3(0.0f, 0.0f, offsetZ) + springOff2;

            const auto &plane = cell.planes[cell.polygons[c.polyIdx].plane];
            float distToPlane = plane.getDistance(subPos);

            bool closeToPlane = (std::fabs(distToPlane) <= kBreakTerrainContactDist);

            if (c.isEdge) {
                // Edge/vertex contacts: distance-only validation (no point-in-polygon).
                // Matches original ConstrainFromTerrain (PHCORE.CPP lines 370-438):
                // TerrainDistance calls cEdgeContact::GetDist (PHCONTCT.CPP lines 425-452)
                // which computes the closest-point-on-segment distance each frame.
                // Algorithm: project subPos onto the edge line. If past either endpoint,
                // use endpoint distance. Otherwise use perpendicular distance via cross product.
                float subRadius = mSphereRadii[subIdx];
                float edgeDist;
                if (glm::length2(c.edgeStart) > 1e-6f || glm::length2(c.edgeEnd) > 1e-6f) {
                    // Proper closest-point-on-segment distance (original GetDist formula)
                    Vector3 v1c = subPos - c.edgeStart;
                    Vector3 v2c = subPos - c.edgeEnd;
                    Vector3 edgeVec = c.edgeStart - c.edgeEnd;

                    if (glm::dot(v1c, edgeVec) > 0.0f) {
                        // Past start vertex — use distance to start
                        edgeDist = glm::length(subPos - c.edgeStart) - subRadius;
                    } else if (glm::dot(v2c, edgeVec) < 0.0f) {
                        // Past end vertex — use distance to end
                        edgeDist = glm::length(subPos - c.edgeEnd) - subRadius;
                    } else {
                        // Between vertices — perpendicular distance via cross product
                        // ||cross(v1c, edge)|| / ||edge|| = distance from line
                        Vector3 crossProd = glm::cross(v1c, edgeVec);
                        float edgeLenSq = glm::dot(edgeVec, edgeVec);
                        edgeDist = (edgeLenSq > 1e-12f)
                            ? std::sqrt(glm::dot(crossProd, crossProd) / edgeLenSq) - subRadius
                            : glm::length(v1c) - subRadius;
                    }
                } else {
                    // Edge endpoints not stored — fall back to plane distance
                    edgeDist = std::fabs(distToPlane) - subRadius;
                }
                if (edgeDist < kBreakTerrainContactDist) {
                    mConstraints.push_back({c.normal, c.objectId});
                    continue;
                }
            } else {
                // Face contacts: distance + point-in-polygon validation.
                // Matches original ConstrainFromTerrain (PHCORE.CPP lines 269-368):
                // Face contacts require SubModOnPoly() in addition to distance check.
                bool insidePoly = false;
                if (closeToPlane) {
                    Vector3 projected = subPos - plane.normal * distToPlane;
                    Vector3 windNormal = -plane.normal;
                    insidePoly = pointInConvexPolygon(projected, cell.vertices,
                                                      cell.polyIndices[c.polyIdx],
                                                      windNormal);
                }

                if (closeToPlane && insidePoly) {
                    // Contact still valid — create constraint from it.
                    // Matches original (PHCORE.CPP line 288):
                    // pModel->AddConstraint(pFaceContact->GetObjID(), i, pFaceContact->GetNormal())
                    mConstraints.push_back({c.normal, c.objectId});
                    // SetGroundObj (D20): track ground surface for FOOT floor contacts.
                    // Matches original (PHCORE.CPP line 329):
                    // g_pPlayerMovement->SetGroundObj(faceContact.GetObjID())
                    if (c.submodelIdx == 4 && c.normal.z > GROUND_NORMAL_MIN)
                        mGroundObjID = c.objectId;
                    continue;
                }
            }

            // Contact invalid — try transition for FOOT floor contacts
            bool transitioned = false;
            if (c.submodelIdx == 4 && c.normal.z > GROUND_NORMAL_MIN) {
                // FOOT floor contact: attempt transition raycast
                Vector3 footPos = subPos;
                int32_t footCell = mCollision.findCell(footPos);
                if (footCell < 0) footCell = c.cellIdx;

                Vector3 probeEnd = footPos - c.normal * kContactTransitionDist;
                RayHit transitionHit;
                bool hit = raycastWorld(mCollision.getWR(), footPos, probeEnd,
                                        transitionHit, nullptr, footCell);

                // Reject self-hits
                if (hit && transitionHit.cellIdx == c.cellIdx &&
                    transitionHit.polyIdx == c.polyIdx) {
                    hit = false;
                }

                if (hit && transitionHit.normal.z > GROUND_NORMAL_MIN) {
                    // Transition to adjacent floor polygon
                    c.normal = transitionHit.normal;
                    c.penetration = 0.01f;
                    c.cellIdx = transitionHit.cellIdx >= 0 ? transitionHit.cellIdx : c.cellIdx;
                    c.polyIdx = transitionHit.polyIdx;
                    c.textureIdx = transitionHit.textureIndex;
                    c.fresh = true;
                    transitioned = true;

                    // Create constraint from transitioned contact
                    mConstraints.push_back({c.normal, c.objectId});
                    // SetGroundObj (D20): update ground surface on FOOT transition
                    mGroundObjID = c.objectId;

                    if (mStepLog) {
                        std::fprintf(stderr, "[STEP-TRANSITION] foot contact transition: "
                            "-> cell=%d poly=%d n=(%.2f,%.2f,%.2f)\n",
                            c.cellIdx, c.polyIdx,
                            c.normal.x, c.normal.y, c.normal.z);
                    }
                }
            }

            // No step-descent probe — matches original engine. Stair descent is
            // handled naturally: foot walks off tread edge, contact breaks, gravity
            // drops the player, swept test catches the lower tread within a few frames.

            if (!transitioned) {
                // Destroy contact — matches original DestroyTerrainContact
                bool wasFoot = (c.submodelIdx == 4 && c.normal.z > GROUND_NORMAL_MIN);
                if (mStepLog && wasFoot) {
                    std::fprintf(stderr, "[STEP-TRANSITION] contact DESTROYED: "
                        "cell=%d poly=%d (gravity takes over)\n",
                        c.cellIdx, c.polyIdx);
                }
                mContacts.erase(mContacts.begin() + ci);

                // LeaveGround (D19): when the last FOOT floor contact is destroyed,
                // immediately transition to Jump. Original (PHCORE.CPP line 360-361):
                // if (!pModel->GetFaceContacts(PLAYER_FOOT, &pDummy))
                //     g_pPlayerMovement->LeaveGround();
                if (wasFoot && isOnGround()) {
                    bool hasFootFloor = false;
                    for (const auto &rc : mContacts) {
                        if (rc.submodelIdx == 4 && rc.normal.z > GROUND_NORMAL_MIN) {
                            hasFootFloor = true;
                            break;
                        }
                    }
                    if (!hasFootFloor) {
                        mCurrentMode = PlayerMode::Jump;
                        mGroundGraceTimer = 0.0f;
                        mGroundGraceActive = false;
                        leaveGround();
                    }
                }
            }
        }
    }

    /// Collision backup factor — original Dark Engine kPartialBackupAmt = 0.9
    /// (phcore.cpp line 5010, "nasty hack to try to avoid epsilon issues").
    /// Used ONLY in IntegrateToCollision (backing up to collision point) and
    /// the Integrate() collision-context function (lines 5018/5035). NOT used
    /// in normal position integration — UpdateTargetLocation (PHMOD.CPP line 1871)
    /// uses pure position = position + velocity * dt.
    static constexpr float kPartialBackupAmt = 0.9f;

    /// Terrain bounce elasticity — original Dark Engine kTerrainBounce (PHCONST.H line 73).
    /// Combined with model elasticity: dampen = model_elasticity * kTerrainBounce.
    /// Player default elasticity = 1.0, so dampen = 0.1 (10% energy return on bounce).
    static constexpr float kTerrainBounce = 0.1f;

    /// Integrate position from velocity. Matches original UpdateTargetLocation
    /// (PHMOD.CPP line 1871): position = position + velocity * dt.
    /// No backup factor — kPartialBackupAmt is only for collision backup contexts.
    inline void integrate() {
        mPosition += mVelocity * mTimestep.fixedDt;
    }

    /// Detect ground contact and update movement state. Any contact with normal.z >
    /// GROUND_NORMAL_MIN counts as ground. Slope movement is friction-based — this only
    /// determines ground/air state. Ground probe uses FOOT position for accurate detection.
    inline void detectGround() {
        bool onGround = false;

        // Check contacts from collision resolution. Track most upward-facing ground normal and
        // texture index for footstep material lookup (Phase 3 Audio).
        // Prefer fresh contacts over maintained contacts for texture/normal selection.
        // Fresh contacts (just detected this frame) are more accurate for material.
        float bestGroundZ = -1.0f;
        bool bestIsFresh = false;
        for (const auto &c : mContacts) {
            if (c.normal.z > GROUND_NORMAL_MIN) {
                onGround = true;
                bool isFresh = c.fresh;
                // Fresh contacts always win over maintained. Among same freshness, pick highest Z.
                if ((isFresh && !bestIsFresh) ||
                    (isFresh == bestIsFresh && c.normal.z > bestGroundZ)) {
                    bestGroundZ = c.normal.z;
                    bestIsFresh = isFresh;
                    mGroundNormal = c.normal;
                    mGroundTextureIdx = c.textureIdx;
                }
            }
        }

        // If no collision contacts and player not ascending, probe along the last known ground
        // normal to detect ground. Prevents briefly entering Jump on bumps/slopes where collision
        // didn't generate contacts. Probe only detects (no position change). Direction follows
        // inverse ground normal (on 45° slope: full 0.1u reach vs 0.07u for straight-down).
        // Falls back to -Z when no valid normal exists. Skip when vel.z > 0.5 (jump).
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

        // FOOT contact recovery — when on ground but FOOT has no ground contact, probe downward
        // to re-acquire the tread below. Handles stair tread transitions where FOOT briefly loses
        // contact between adjacent polygons. Synthesized contact enables tryStairStep().
        if (onGround) {
            bool footHasGround = false;
            for (const auto &c : mContacts) {
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
                    mContacts.push_back(fc);
                }
            }
        }

        // Update movement state — mode transitions for ground/air
        if (onGround) {
            // Reset grace state whenever ground is positively detected
            mGroundGraceTimer = 0.0f;
            mGroundGraceActive = false;

            if (mCurrentMode == PlayerMode::Jump) {
                // Landing — zero vertical velocity, optionally activate landing bump pose if
                // falling fast enough (LANDING_MIN_VEL) and cooldown elapsed (LANDING_MIN_TIME).
                float fallSpeed = -mVelocity.z;  // positive when falling
                if (mVelocity.z < 0.0f)
                    mVelocity.z = 0.0f;

                if (fallSpeed > LANDING_MIN_VEL &&
                    (mSimTime - mLastLandingTime) > LANDING_MIN_TIME) {
                    activatePose(POSE_JUMP_LAND);
                    mLandingActive = true;
                    mLastLandingTime = mSimTime;

                    // Fire landing callback for impact sound
                    if (mLandingCb) {
                        Vector3 footPos = mPosition + Vector3(0.0f, 0.0f, mSphereOffsetsBase[4]);
                        mLandingCb(footPos, fallSpeed, mGroundTextureIdx);
                    }
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
                // Ground lost while in ground mode — apply grace period. Prevents single-frame
                // air transitions on bumps/slopes. During grace, gravity still applies (natural
                // fall behavior); only mode state is affected (stays Stand/Crouch for friction).
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
                        leaveGround();
                    }
                    // else: still in grace period, stay in ground mode
                }
            }
            // If already in Jump mode, stay in Jump mode
        }
    }

    /// Update cell index from current position using portal-graph-based tracking. Avoids
    /// teleporting to overlapping but topologically-unconnected cells (e.g., water/air overlap).
    /// Algorithm: 1) still in current cell (fast path), 2) portal-connected neighbors,
    /// 3) brute-force fallback (spawn/teleport).
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

    /// Update mode transitions — handles crouch, swim entry/exit. Body center shifts and spring
    /// compensation for crouch/uncrouch are driven by the pose system.
    inline void updateModeTransitions() {
        // Crouch: body center stays in place, pose system drives HEAD (-2.02) and BODY (-1.0)
        // smoothly over blend duration. Collision uses blended offsets for ceiling clearance.

        // ── Crouch transitions (only on ground modes) ──
        // Dark Engine convention: the rest-motion activation that drives the eye
        // up/down is GATED by the leaning state — when the player is leaning, the
        // current lean motion stays active across mode changes. The pose target is
        // not re-activated, so the visible eye height does not change on a
        // C-press while leaning. The crouch becomes visible later, when lean is
        // released: setLeanDirection(0) activates the mode's rest pose
        // (POSE_CROUCH or POSE_NORMAL), and the spring smoothly blends the
        // current lean offset toward that rest target.
        //
        // An earlier port-only attempt re-activated POSE_CLNLEAN_*/POSE_LEAN_* on
        // mode change mid-lean to make the crouch immediately visible. That
        // re-activation re-anchored the head spring's virtual grid
        // (snapSpringVirtualGridToNow) and produced a 1-frame camera discontinuity
        // — a "jitter" — every time the C key was tapped while leaning. Match the
        // original instead.
        if (isOnGround()) {
            if (mWantsCrouch && !isCrouching()) {
                mCurrentMode = PlayerMode::Crouch;
                if (mLeanDir == 0)
                    activatePose(POSE_CROUCH);
                // else: lean motion stays active; rest motion deferred until
                // setLeanDirection(0) fires on lean release.
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
                    if (mLeanDir == 0)
                        activatePose(POSE_NORMAL);
                    // else: lean motion stays active across the mode change.
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

    // ── Platform riding ──
    //
    // Moving terrain (elevators, platforms) carry the player by adding the
    // platform's velocity to the player position each physics step. The
    // original Dark Engine tracks a "reference frame object" and adds a
    // velocity constraint along the contact normal. Our simplified version
    // directly displaces the player by the platform's velocity * dt.
    //
    // On platform departure (ground contact lost), the platform's velocity
    // is inherited into the player's velocity for smooth detachment — the
    // player keeps moving in the direction the platform was traveling.

    inline void updatePlatformRiding() {
        if (!mPlatformVelocityCb) return;

        // Scan ground contacts for a moving terrain object
        int32_t newPlatformID = 0;
        for (const auto &c : mContacts) {
            if (c.objectId > 0 && c.normal.z > GROUND_NORMAL_MIN) {
                // Check if this object is a moving platform
                const Vector3 *platVel = mPlatformVelocityCb(c.objectId);
                if (platVel) {
                    newPlatformID = c.objectId;
                    mPlatformVelocity = *platVel;
                    break;
                }
            }
        }

        if (newPlatformID != 0) {
            // On a platform — apply platform displacement this step
            mPosition += mPlatformVelocity * mTimestep.fixedDt;

            // Suppress downward friction when platform is moving upward.
            // Without this, friction fights the upward velocity and the
            // player "sinks" through a rising platform.
            if (mPlatformVelocity.z < 0.0f) {
                // Platform descending — remove downward velocity component
                // beyond what the platform provides, so the player descends
                // with the platform instead of floating
                if (mVelocity.z < mPlatformVelocity.z)
                    mVelocity.z = mPlatformVelocity.z;
            }

            mPlatformObjID = newPlatformID;

        } else if (mPlatformObjID != 0) {
            // Just left the platform — inherit platform velocity into
            // player velocity for smooth detachment
            mVelocity += mPlatformVelocity;
            mPlatformObjID = 0;
            mPlatformVelocity = Vector3(0.0f);
        }
    }
