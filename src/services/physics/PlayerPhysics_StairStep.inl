// Included inside PlayerPhysics class body — do not include standalone.
//
// Stair stepping — replicates the original Dark Engine's CheckStep + PostCollisionUpdate
// pipeline exactly. Called DURING resolveCollisions() when a leg-level wall contact is
// detected. On success, replaces the normal bounce response (caller does `continue`).
//
// Original engine flow:
//   1. Detect leg wall contact (normal.z < 0.4) during collision resolution
//   2. CheckStep: 3-phase raycast (UP→FWD→DOWN), validate BODY sphere, apply position
//   3. PostCollisionUpdate: re-run physics pipeline for remaining frame time
//      (zero accel → rebuild constraints → controls → dynamics → integrate → re-collide)
//   4. Return from collision (skip bounce response)

    /// Try to step up from contacts in the current collision iteration.
    /// Matches original Dark Engine CheckStep exactly:
    /// - Trigger: leg contact with fabs(normal.z) < 0.4 (no directional filter)
    /// - UP ray: 2 units, avatar ignores ceiling
    /// - FWD ray: velocity * 0.01
    /// - DOWN ray: 4 units
    /// - z_delta: clamp to min 0.3, add 0.02, no max cap
    /// - Validate: ANY body collision = fail (no threshold)
    /// - Apply: lift position, destroy leg contacts, create FOOT contact
    /// - Re-integrate: run gravity→movement→constrain→integrate→collide for remaining dt
    inline bool tryStairStepFromContacts(
        const std::vector<SphereContact> &contacts, float remainingDt)
    {
        // Movement direction from desired velocity.
        // Original uses pModel->GetVelocity(subModId) at collision time (pre-constraint).
        // Since we're inside the collision loop, desired velocity is the closest match.
        Vector3 desired = computeDesiredVelocity();
        Vector3 hDesired(desired.x, desired.y, 0.0f);
        float hSpeed = glm::length(hDesired);
        if (hSpeed < 0.1f)
            return false;

        Vector3 moveDir = hDesired / hSpeed;

        // ── Find trigger: leg-level wall contact ──
        // Original: fabs(normal.z) < 0.4, FOOT/SHIN/KNEE, terrain face only.
        // NO directional filter — original steps on any wall regardless of direction.
        float footOffsetZ = mSphereOffsetsBase[4]; // FOOT = -3.0
        Vector3 footPos = mPosition + Vector3(0.0f, 0.0f, footOffsetZ);

        bool hasWallContact = false;
        Vector3 bestWallNormal(0.0f);
        for (const auto &c : contacts) {
            if (c.submodelIdx < 2) continue;          // only SHIN/KNEE/FOOT
            if (c.objectId >= 0) continue;             // terrain only
            if (c.isEdge) continue;                    // face contacts only
            if (std::fabs(c.normal.z) >= STEP_WALL_THRESHOLD) continue;

            hasWallContact = true;
            bestWallNormal = c.normal;
            break;
        }

        if (!hasWallContact)
            return tryObjectStairStep(moveDir, hSpeed, footPos);

        if (mStepLog) {
            std::fprintf(stderr, "[STEP] attempt pos=(%.2f,%.2f,%.2f) footZ=%.2f "
                "spd=%.1f dir=(%.2f,%.2f) wallN=(%.2f,%.2f,%.2f)\n",
                mPosition.x, mPosition.y, mPosition.z, footPos.z,
                hSpeed, moveDir.x, moveDir.y,
                bestWallNormal.x, bestWallNormal.y, bestWallNormal.z);
        }

        // ── Phase 1: Raycast UP (2 units from foot) ──
        // Original: avatar IGNORES ceiling collision (phcore.cpp line 5365).
        // Clamp UP position to ceiling if hit, fail only if no headroom at all.
        Vector3 upStart = footPos + Vector3(0.0f, 0.0f, STEP_UP_EPSILON);
        Vector3 upPos = footPos + Vector3(0.0f, 0.0f, STEP_UP_DIST);
        RayHit stepHit;
        int32_t cellHint = -1;

        bool upHit = raycastWorld(mCollision.getWR(), upStart, upPos, stepHit, &cellHint);
        if (upHit) {
            upPos.z = stepHit.point.z - 0.01f;
            if (upPos.z <= footPos.z + 0.1f) {
                if (mStepLog) std::fprintf(stderr, "[STEP]   FAIL: ceiling too low (%.2f)\n", upPos.z);
                return false;
            }
        }

        // ── Phase 2: Raycast FORWARD (velocity * 0.01) ──
        float fwdDist = hSpeed * STEP_FWD_SCALE;
        Vector3 fwdPos = upPos + moveDir * fwdDist;

        int32_t fwdCellHint = cellHint;
        if (raycastWorld(mCollision.getWR(), upPos, fwdPos, stepHit, &fwdCellHint, cellHint)) {
            if (mStepLog) std::fprintf(stderr, "[STEP]   P2 FWD FAIL\n");
            return false;
        }

        // ── Phase 3: Raycast DOWN (4 units) ──
        // Try multiple cell hints to handle thin riser cells.
        static constexpr float STEP_DOWN_DIST = 4.0f;
        Vector3 downTarget = fwdPos - Vector3(0.0f, 0.0f, STEP_DOWN_DIST);

        bool downOk = raycastWorld(mCollision.getWR(), fwdPos, downTarget, stepHit, nullptr, fwdCellHint);
        if (!downOk)
            downOk = raycastWorld(mCollision.getWR(), fwdPos, downTarget, stepHit, nullptr, mCellIdx);
        if (!downOk)
            downOk = raycastWorld(mCollision.getWR(), fwdPos, downTarget, stepHit);
        if (!downOk) {
            if (mStepLog) std::fprintf(stderr, "[STEP]   P3 DOWN FAIL\n");
            return false;
        }

        float hitZ = stepHit.point.z;
        Vector3 stepGroundNormal = stepHit.normal;

        // Landing must be walkable (prevents stepping onto steep slopes)
        if (stepGroundNormal.z < WALKABLE_SLOPE_THRESHOLD) {
            if (mStepLog) std::fprintf(stderr, "[STEP]   P3 REJECT: steep (nZ=%.2f)\n", stepGroundNormal.z);
            return false;
        }

        // ── Compute step height ──
        // Original: z_delta = hit_z - foot_z, clamp to min 0.3, add 0.02.
        // NO max cap — original relies on BODY validation only.
        float zDelta = hitZ - footPos.z;
        if (zDelta < STEP_MIN_ZDELTA)
            zDelta = STEP_MIN_ZDELTA;
        zDelta += STEP_CLEARANCE;

        // Sanity: don't step higher than the UP probe reached
        if (zDelta > (upPos.z - footPos.z) + STEP_CLEARANCE) {
            if (mStepLog) std::fprintf(stderr, "[STEP]   REJECT: above UP probe (%.3f)\n", zDelta);
            return false;
        }

        if (mStepLog) {
            std::fprintf(stderr, "[STEP]   P3 OK: hitZ=%.2f raw=%.3f zDelta=%.3f\n",
                hitZ, hitZ - footPos.z, zDelta);
        }

        // ── Validate: test BODY sphere at stepped position ──
        // Original: CheckTerrainCollision for sphere submodels. ANY collision = fail.
        Vector3 steppedPos = mPosition;
        steppedPos.z += zDelta;

        int32_t steppedCell = mCollision.findCell(steppedPos);
        if (steppedCell < 0) {
            if (mStepLog) std::fprintf(stderr, "[STEP]   VALIDATE FAIL: no cell\n");
            return false;
        }

        {
            float bodyOffsetZ = mSphereOffsetsBase[1] + mBodyPoseCurrent.z;
            Vector3 bodyCenter = steppedPos + Vector3(0.0f, 0.0f, bodyOffsetZ);
            mStepScratchContacts.clear();
            mCollision.sphereVsCellPolygons(bodyCenter, SPHERE_RADIUS,
                                             steppedCell, mStepScratchContacts);
            int32_t bodyCell = mCollision.findCell(bodyCenter);
            if (bodyCell >= 0 && bodyCell != steppedCell)
                mCollision.sphereVsCellPolygons(bodyCenter, SPHERE_RADIUS,
                                                 bodyCell, mStepScratchContacts);
            // Original: ANY collision = fail (no penetration threshold)
            for (const auto &c : mStepScratchContacts) {
                if (c.penetration > 0.0f) {
                    if (mStepLog) std::fprintf(stderr, "[STEP]   VALIDATE FAIL: BODY pen=%.3f\n", c.penetration);
                    return false;
                }
            }
        }

        // ── Accept the step ──
        if (mStepLog) {
            std::fprintf(stderr, "[STEP]   ACCEPTED: (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f) dZ=%.3f\n",
                mPosition.x, mPosition.y, mPosition.z,
                steppedPos.x, steppedPos.y, steppedPos.z, zDelta);
        }

        mPosition = steppedPos;
        mCellIdx = steppedCell;
        mGroundNormal = stepGroundNormal;

        // HEAD spring compensation — offset so camera stays at current world Z,
        // then spring naturally blends up (smooth camera step transition).
        mSpringPos.z -= zDelta;

        // Destroy leg contacts and create synthetic FOOT contact.
        // Matches original: DestroyAllTerrainContacts(KNEE/SHIN/FOOT) +
        // CreateTerrainContact(FOOT, new_poly).
        mLastContacts.erase(
            std::remove_if(mLastContacts.begin(), mLastContacts.end(),
                           [](const SphereContact &c) { return c.submodelIdx >= 2; }),
            mLastContacts.end());

        {
            SphereContact footContact;
            footContact.normal = stepGroundNormal;
            footContact.penetration = 0.01f;
            footContact.cellIdx = steppedCell;
            footContact.polyIdx = -1;
            footContact.textureIdx = stepHit.textureIndex;
            footContact.age = 0;
            footContact.submodelIdx = 4;
            mLastContacts.push_back(footContact);
        }

        // Mode transition
        if (mCurrentMode != PlayerMode::Stand &&
            mCurrentMode != PlayerMode::Crouch &&
            mCurrentMode != PlayerMode::Swim &&
            mCurrentMode != PlayerMode::Dead)
            mCurrentMode = PlayerMode::Stand;
        if (mCurrentMode == PlayerMode::Climb)
            mCurrentMode = PlayerMode::Stand;

        // ── PostCollisionUpdate: re-integrate for remaining frame time ──
        // Original engine re-runs the full physics pipeline after a step:
        //   ZeroAcceleration → ClearConstraints → ConstrainFromTerrain/Objects →
        //   UpdateModelControls → UpdateModelDynamics → UpdateModel → Re-collide
        // This is critical for smooth stepping: the player immediately gets real
        // floor contacts, velocity from gravity+friction, and forward movement.
        if (remainingDt > 0.0001f) {
            postStepReIntegrate(remainingDt);
        }

        return true;
    }

    /// Re-integration after a successful stair step, matching the original engine's
    /// PostCollisionUpdate. Re-runs the core physics pipeline for the remaining
    /// frame time so the player doesn't sit at the stepped position with stale
    /// velocity/contacts until the next frame.
    inline void postStepReIntegrate(float dt) {
        // 1. Apply gravity for remaining time
        //    Original: UpdateModelDynamics applies gravity internally.
        //    We need it before movement so friction calculation is correct.
        if (!isOnGround() || mGroundGraceActive) {
            mVelocity.z -= mGravityMag * dt;
        }

        // 2. Apply movement control for remaining time
        //    Original: UpdateModelControls applies player input.
        //    Uses the NEW contacts (synthetic foot) for friction calculation.
        applyMovement();

        // 3. Constrain velocity against new contacts
        //    Original: ClearConstraints + ConstrainFromTerrain + ApplyConstraints.
        //    The synthetic foot contact prevents downward velocity.
        constrainVelocity();

        // 4. Integrate position for remaining time
        //    Original: UpdateModel integrates velocity into position.
        mPosition += mVelocity * dt;

        // 5. Update cell
        int32_t newCell = mCollision.findCell(mPosition);
        if (newCell >= 0) mCellIdx = newCell;

        // 6. Re-run collision detection at new position
        //    Original: CheckModelTerrainCollisions + CheckModelObjectCollisions.
        //    This generates real floor contacts (replacing the synthetic one)
        //    and handles any new penetrations from the re-integration movement.
        //    Note: this can trigger another step (cascade), matching original behavior.
        //    We don't pass a contactCb here — object collision testing will happen
        //    on the next full frame. Terrain collision is sufficient for stability.
        mIterContacts.clear();
        for (int s = 0; s < NUM_SPHERES; ++s) {
            float poseOffsetZ = 0.0f;
            if (s == 0) poseOffsetZ = mPoseCurrent.z;
            else if (s == 1) poseOffsetZ = mBodyPoseCurrent.z;
            float offsetZ = mSphereOffsetsBase[s] + poseOffsetZ;
            Vector3 sphereCenter = mPosition + Vector3(0.0f, 0.0f, offsetZ);

            size_t contactsBefore = mIterContacts.size();
            mCollision.sphereVsCellPolygons(sphereCenter, mSphereRadii[s],
                                             mCellIdx, mIterContacts);
            int32_t sphereCell = mCollision.findCell(sphereCenter);
            if (sphereCell >= 0 && sphereCell != mCellIdx)
                mCollision.sphereVsCellPolygons(sphereCenter, mSphereRadii[s],
                                                 sphereCell, mIterContacts);
            for (size_t ci = contactsBefore; ci < mIterContacts.size(); ++ci)
                mIterContacts[ci].submodelIdx = static_cast<int8_t>(s);
        }

        // Push out of any penetrations from re-integration
        if (!mIterContacts.empty()) {
            mPushes.clear();
            for (const auto &c : mIterContacts) {
                Vector3 pushNormal = c.normal;
                // Steep surface flattening (same as main collision loop)
                if (pushNormal.z > 0.0f && pushNormal.z < WALKABLE_SLOPE_THRESHOLD) {
                    pushNormal.z = 0.0f;
                    float len = glm::length(pushNormal);
                    if (len > 0.001f) pushNormal /= len;
                    else continue;
                }
                bool merged = false;
                for (auto &p : mPushes) {
                    if (glm::dot(c.normal, p.first) > 0.99f) {
                        p.second = std::max(p.second, c.penetration);
                        merged = true;
                        break;
                    }
                }
                if (!merged)
                    mPushes.push_back({pushNormal, c.penetration});
            }
            for (const auto &p : mPushes)
                mPosition += p.first * p.second;

            // Accumulate real contacts for next frame's constrainVelocity
            mLastContacts.insert(mLastContacts.end(),
                                 mIterContacts.begin(), mIterContacts.end());
        }

        // Update cell after push-out
        newCell = mCollision.findCell(mPosition);
        if (newCell >= 0) mCellIdx = newCell;
    }

    /// Object stair stepping — step onto OBB world objects (crates, tables, etc).
    /// Same structure as terrain stepping but uses ray-vs-OBB tests.
    inline bool tryObjectStairStep(const Vector3 &moveDir, float hSpeed,
                                    const Vector3 &footPos) {
        if (!mObjectWorld)
            return false;

        const ObjectCollisionBody *stepOBB = nullptr;
        int triggerFaceIdx = -1;

        for (const auto &c : mLastContacts) {
            if (c.objectId < 0 || c.submodelIdx < 2)
                continue;
            const ObjectCollisionBody *body = mObjectWorld->findBodyByObjID(c.objectId);
            if (!body || body->shapeType != CollisionShapeType::OBB)
                continue;
            if (body->climbableSides != 0) continue;
            if (body->isEdgeTrigger) continue;
            if (mIsPushableCb && mIsPushableCb(c.objectId)) continue;

            int faceIdx = c.polyIdx & 0xF;
            if (faceIdx > 5) continue;

            Vector3 faceNormal = getOBBFaceNormal(*body, faceIdx);
            // Object stepping: nZ < 0.4 (no fabs — matches original)
            if (faceNormal.z < 0.4f) {
                stepOBB = body;
                triggerFaceIdx = faceIdx;
                break;
            }
        }

        if (!stepOBB)
            return false;

        if (mStepLog) {
            std::fprintf(stderr, "[STEP-OBJ] attempt objID=%d pos=(%.2f,%.2f,%.2f) footZ=%.2f\n",
                stepOBB->objID, mPosition.x, mPosition.y, mPosition.z, footPos.z);
        }

        // Phase 1: UP (avatar ignores ceiling)
        Vector3 upStart = footPos + Vector3(0.0f, 0.0f, STEP_UP_EPSILON);
        Vector3 upEnd = footPos + Vector3(0.0f, 0.0f, STEP_UP_DIST);
        RayOBBResult upResult = rayVsOBB(upStart, upEnd, *stepOBB);
        if (upResult.hit) {
            upEnd.z = upResult.point.z - 0.01f;
            if (upEnd.z <= footPos.z + 0.1f)
                return false;
        }

        // Phase 2: FORWARD
        float fwdDist = hSpeed * STEP_FWD_SCALE;
        Vector3 fwdEnd = upEnd + moveDir * fwdDist;
        RayOBBResult fwdResult = rayVsOBB(upEnd, fwdEnd, *stepOBB);
        if (fwdResult.hit) return false;

        // Phase 3: DOWN against OBB
        static constexpr float OBJ_STEP_DOWN_DIST = 4.0f;
        Vector3 downEnd = fwdEnd - Vector3(0.0f, 0.0f, OBJ_STEP_DOWN_DIST);
        RayOBBResult downResult = rayVsOBB(fwdEnd, downEnd, *stepOBB);
        if (!downResult.hit) return false;

        float hitZ = downResult.point.z;
        int topFaceIdx = downResult.faceIdx;
        Vector3 topNormal = getOBBFaceNormal(*stepOBB, topFaceIdx);

        // Compute lift (same as terrain: min 0.3, + 0.02, no max)
        float zDelta = hitZ - footPos.z;
        if (zDelta < STEP_MIN_ZDELTA) zDelta = STEP_MIN_ZDELTA;
        zDelta += STEP_CLEARANCE;

        // Sanity: don't step above UP probe
        if (zDelta > (upEnd.z - footPos.z) + STEP_CLEARANCE) return false;

        // Validate BODY
        Vector3 steppedPos = mPosition;
        steppedPos.z += zDelta;
        int32_t steppedCell = mCollision.findCell(steppedPos);
        if (steppedCell < 0) return false;

        {
            float bodyOffsetZ = mSphereOffsetsBase[1] + mBodyPoseCurrent.z;
            Vector3 bodyCenter = steppedPos + Vector3(0.0f, 0.0f, bodyOffsetZ);
            mStepScratchContacts.clear();
            mCollision.sphereVsCellPolygons(bodyCenter, SPHERE_RADIUS,
                                             steppedCell, mStepScratchContacts);
            for (const auto &c : mStepScratchContacts) {
                if (c.penetration > 0.0f) return false;
            }
        }

        if (mStepLog) {
            std::fprintf(stderr, "[STEP-OBJ] ACCEPTED: objID=%d dZ=%.3f\n",
                stepOBB->objID, zDelta);
        }

        mPosition = steppedPos;
        mCellIdx = steppedCell;
        mGroundNormal = topNormal;
        mSpringPos.z -= zDelta;

        {
            SphereContact objContact;
            objContact.normal = topNormal;
            objContact.penetration = 0.01f;
            objContact.cellIdx = -1;
            objContact.polyIdx = -1;
            objContact.textureIdx = -1;
            objContact.age = 0;
            objContact.submodelIdx = 4;
            objContact.objectId = stepOBB->objID;
            mLastContacts.push_back(objContact);
        }

        if (mCurrentMode != PlayerMode::Stand &&
            mCurrentMode != PlayerMode::Crouch &&
            mCurrentMode != PlayerMode::Swim &&
            mCurrentMode != PlayerMode::Dead)
            mCurrentMode = PlayerMode::Stand;
        if (mCurrentMode == PlayerMode::Climb)
            mCurrentMode = PlayerMode::Stand;

        return true;
    }
