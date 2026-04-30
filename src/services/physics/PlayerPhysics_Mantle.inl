// Included inside PlayerPhysics class body — do not include standalone.

    /// Combined ray-vs-WR + ray-vs-OBB raycast for mantle. Returns true if
    /// either world geometry or any registered OBB is intersected, and fills
    /// `hit` with whichever is closest. Crates / chests / containers live in
    /// ObjectCollisionWorld (not WR), so a world-only raycast misses them
    /// entirely — that's why pre-fix mantle never engaged on dynamic objects.
    inline bool raycastMantle(const Vector3 &start, const Vector3 &end,
                              RayHit &hit) const {
        bool hitAny = raycastWorld(mCollision.getWR(), start, end, hit);

        if (!mObjectWorld) return hitAny;

        Vector3 segment = end - start;
        float segLen = glm::length(segment);
        if (segLen < 1e-6f) return hitAny;

        // Track closest hit by parametric t along the ray. WR hit's distance
        // is in world units; convert to t for apples-to-apples comparison.
        float bestT = hitAny ? (hit.distance / segLen) : 1.0f;

        const auto &bodies = mObjectWorld->getBodies();
        for (const auto &body : bodies) {
            if (body.skipPlayerCollision) continue;
            // Mantle climbs onto box-like surfaces; sphere-shaped objects
            // (barrels, balls) aren't valid ledge surfaces.
            if (body.shapeType != CollisionShapeType::OBB &&
                body.shapeType != CollisionShapeType::SphereHat)
                continue;

            RayOBBResult r = rayVsOBB(start, end, body);
            if (!r.hit) continue;
            if (r.t >= bestT) continue;

            bestT = r.t;
            hit.point = r.point;
            hit.distance = r.t * segLen;
            // Clear WR-specific fields so callers don't pick up stale state.
            hit.cellIdx = -1;
            hit.polyIdx = -1;
            hit.textureIndex = -1;
            // Approximate surface normal via the OBB's hit face. faceIdx
            // 0..2 = +X/+Y/+Z, 3..5 = -X/-Y/-Z (rotation-matrix columns).
            int axis = r.faceIdx % 3;
            Vector3 n = body.rotation[axis];
            if (r.faceIdx >= 3) n = -n;
            hit.normal = n;
            hit.hitEntity = body.objID;
            hitAny = true;
        }
        return hitAny;
    }

    /// Check for mantleable ledge using 3-ray detection (airborne + pressing forward).
    /// 1. UP from head (3.5u headroom), 2. FWD from top (2.4u clear path),
    /// 3. DOWN from fwd pos (7.0u ledge surface), 4. FWD from HEAD/BODY/FOOT (XY target).
    /// Rise target is INTERMEDIATE (head peeks above lip): riseZ = ledge.z + r*1.02 -
    /// HEAD_OFFSET + 1.0. Full standing height via FinalRise after forward compression.
    /// Each phase tests both WR cell geometry and OBB objects so dynamic crates,
    /// chests, etc. count as valid ledges (and as obstructions overhead).
    inline bool checkMantle() {
        if (mCurrentMode != PlayerMode::Jump || mInputForward < 0.1f || mMantling)
            return false;

        // HEAD position for mantle check — apply pose-driven drop (smooth during crouch)
        float headZ = mSphereOffsetsBase[0] + mPoseCurrent.z;
        Vector3 headPos = mPosition + Vector3(0.0f, 0.0f, headZ);

        // Phase 1: upward — check headroom (3.5 units above head)
        Vector3 upTarget = headPos + Vector3(0.0f, 0.0f, MANTLE_UP_DIST);
        RayHit hit;
        if (raycastMantle(headPos, upTarget, hit))
            return false;  // ceiling too low

        // Forward direction (horizontal, unit length, from cached yaw)
        Vector3 fwd(mCosYaw, mSinYaw, 0.0f);
        Vector3 moveDir = fwd * MANTLE_FWD_DIST;  // radius × 2 = 2.4 units

        // Phase 2: forward — check clear path above the ledge
        Vector3 fwdTarget = upTarget + moveDir;
        if (raycastMantle(upTarget, fwdTarget, hit))
            return false;  // wall in the way

        // Phase 3: downward — find ledge surface (7.0 units down)
        Vector3 downTarget = fwdTarget + Vector3(0.0f, 0.0f, -MANTLE_DOWN_DIST);
        if (!raycastMantle(fwdTarget, downTarget, hit))
            return false;  // no ledge found

        float ledgeZ = hit.point.z;

        // Phase 4: Find forward surface (HEAD→BODY→FOOT fallbacks). Target XY pulled back 0.55
        // from wall to stay close but not inside.
        Vector3 fwdHitPoint;
        bool foundFwdSurface = false;

        // Try HEAD → BODY → FOOT forward ray to find wall surface
        const float fwdSurfaceOffsets[3] = {
            mSphereOffsetsBase[0], mSphereOffsetsBase[1], mSphereOffsetsBase[4]
        };
        for (int i = 0; i < 3 && !foundFwdSurface; ++i) {
            Vector3 spherePos = mPosition + Vector3(0.0f, 0.0f, fwdSurfaceOffsets[i]);
            Vector3 sphereFwd = spherePos + moveDir;
            if (raycastMantle(spherePos, sphereFwd, hit)) {
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

        // Validate: can the player fit at the target? Also check standing on the ledge beyond
        // the lip (all 5 submodels vs cell geometry).
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
        mMantleStartPos = mPosition;  // save for abort fallback
        // Virtual head starts at current head position
        mMantleHeadPos = mPosition + Vector3(0.0f, 0.0f, mSphereOffsetsBase[0]);
        mMantleHeadVel = Vector3(0.0f);
    }

    /// Abort a mantle in progress — restore player to pre-mantle position and resume
    /// normal physics. Called when the mantle path is blocked by a wall or object.
    inline void abortMantle() {
        mPosition = mMantleStartPos;
        mVelocity = Vector3(0.0f);
        mMantling = false;
        mMantleCompressed = false;
        mMantleState = MantleState::None;
        mCurrentMode = mWantsCrouch ? PlayerMode::Crouch : PlayerMode::Stand;
        // Sync mEndPosition with the restored start position so resolveCollisions
        // doesn't drag the body back to a stale projected end (see Complete state).
        mEndPosition = mPosition;
        updateCell();
        resolveCollisions([](const ContactEvent&) {});
        updateCell();
    }

    /// Check if mantle movement is blocked by world geometry or objects.
    /// Tests HEAD sphere at the new position; returns true if any contact
    /// significantly opposes the movement direction.
    inline bool isMantlePathBlocked(const Vector3 &oldHead, const Vector3 &newHead) const {
        int32_t cell = mCollision.findCell(newHead);
        if (cell < 0) return true;  // outside world geometry

        std::vector<SphereContact> contacts;
        float headR = mSphereRadii[0];
        mCollision.sphereVsCellPolygons(newHead, headR, cell, contacts);
        if (mObjectWorld) {
            mObjectWorld->testPlayerSpheres(&newHead, &headR, 1, cell, contacts);
        }
        if (contacts.empty()) return false;

        // Check if any contact opposes the movement direction with significant
        // penetration. This filters out floor/ceiling scrape contacts that are
        // expected during mantle (e.g. rolling over a ledge lip).
        Vector3 moveDir = newHead - oldHead;
        float moveDist = glm::length(moveDir);
        if (moveDist < 0.001f) return false;
        moveDir /= moveDist;

        for (const auto &c : contacts) {
            if (glm::dot(c.normal, -moveDir) > 0.3f && c.penetration > 0.1f) {
                return true;
            }
        }
        return false;
    }

    /// Update mantle state machine each fixedStep. Normal movement/gravity/collision suppressed.
    /// Virtual head (mMantleHeadPos) driven by spring formula — camera tracks this, so body
    /// teleports at state transitions are invisible. Per-state tension/velocity multipliers.
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
            // State 2: Spring drives head toward rise target (+ HEAD_OFFSET_Z). Adaptive tension
            // modulated by inverse-distance-squared (faster far, slower near).
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

                Vector3 oldHead = mMantleHeadPos;
                mMantleHeadVel = computeSpringVelocity(
                    mMantleHeadPos, mMantleHeadVel, headTarget,
                    tension, HEAD_SPRING_BASE_DAMPING,
                    HEAD_SPRING_Z_SCALE, HEAD_SPRING_VEL_CAP, dt);
                mMantleHeadPos += mMantleHeadVel * dt;

                // Abort if head hits wall or object during rise
                if (isMantlePathBlocked(oldHead, mMantleHeadPos)) {
                    abortMantle();
                    return;
                }

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

                Vector3 oldHead = mMantleHeadPos;
                mMantleHeadVel = computeSpringVelocity(
                    mMantleHeadPos, mMantleHeadVel, fwdTarget,
                    tension, HEAD_SPRING_BASE_DAMPING,
                    HEAD_SPRING_Z_SCALE, HEAD_SPRING_VEL_CAP, dt);

                // 10× velocity multiplier applied AFTER spring cap
                mMantleHeadVel *= 10.0f;
                mMantleHeadPos += mMantleHeadVel * dt;

                // Track body with head during compressed state
                mPosition = mMantleHeadPos;

                // Abort if compressed ball hits wall or object
                if (isMantlePathBlocked(oldHead, mMantleHeadPos)) {
                    abortMantle();
                    return;
                }

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

                Vector3 oldHead = mMantleHeadPos;
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

                // Abort if player hits wall or object during final rise
                if (isMantlePathBlocked(oldHead, mMantleHeadPos)) {
                    abortMantle();
                    return;
                }

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
            // State 5: Restore normal mode. Run collision resolution to push body out of any
            // embedding from states 1-4 (which drive position directly through geometry). Without
            // this, the first regular fixedStep would detect deep penetration and "launch" player.
            mMantling = false;
            mMantleCompressed = false;
            mMantleState = MantleState::None;
            mCurrentMode = mWantsCrouch ? PlayerMode::Crouch : PlayerMode::Stand;
            mVelocity = Vector3(0.0f);
            // Sync mEndPosition: applyMovement was suppressed during mantling so
            // mEndPosition still holds the pre-mantle value. resolveCollisions's
            // first line is `mPosition = mEndPosition`, which would teleport us
            // back to the pre-jump location. The mantle state machine drove
            // mPosition directly, so the new mPosition IS the intended end pos.
            mEndPosition = mPosition;
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
