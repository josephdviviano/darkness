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
        const std::vector<SphereContact> &contacts, float remainingDt,
        const Vector3 &backupPos)
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
        // Use backupPos (backed up to ~collision point) matching original engine's
        // IntegrateToCollision (phcore.cpp lines 5120-5121, kPartialBackupAmt=0.9).
        float footOffsetZ = mSphereOffsetsBase[4]; // FOOT = -3.0
        Vector3 footPos = backupPos + Vector3(0.0f, 0.0f, footOffsetZ);

        bool hasWallContact = false;
        Vector3 bestWallNormal(0.0f);
        for (const auto &c : contacts) {
            if (c.submodelIdx < 2) continue;          // only SHIN/KNEE/FOOT
            // Dark Engine object ID convention: concrete objects (crates, doors, NPCs)
            // have positive IDs (>= 0). Terrain/world geometry contacts use -1 as a
            // sentinel — there's no "object", just BSP cell polygons. So this filter
            // skips object contacts and keeps only terrain.
            if (c.objectId >= 0) continue;
            if (c.isEdge) continue;                    // face contacts only
            if (std::fabs(c.normal.z) >= STEP_WALL_THRESHOLD) continue;

            hasWallContact = true;
            bestWallNormal = c.normal;
            break;
        }

        if (!hasWallContact)
            return tryObjectStairStep(moveDir, hSpeed, footPos, remainingDt);

        if (mStepLog) {
            std::fprintf(stderr, "[STEP] attempt pos=(%.3f,%.3f,%.3f) backupPos=(%.3f,%.3f,%.3f) "
                "footPos=(%.3f,%.3f,%.3f)\n",
                mPosition.x, mPosition.y, mPosition.z,
                backupPos.x, backupPos.y, backupPos.z,
                footPos.x, footPos.y, footPos.z);
            std::fprintf(stderr, "[STEP]   spd=%.2f dir=(%.3f,%.3f) wallN=(%.3f,%.3f,%.3f)\n",
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
        if (mStepLog) {
            std::fprintf(stderr, "[STEP]   P1 UP: (%.3f,%.3f,%.3f)->(%.3f,%.3f,%.3f) hit=%d",
                upStart.x, upStart.y, upStart.z, upPos.x, upPos.y, upPos.z, upHit);
            if (upHit) std::fprintf(stderr, " hitPt=(%.3f,%.3f,%.3f)", stepHit.point.x, stepHit.point.y, stepHit.point.z);
            std::fprintf(stderr, "\n");
        }
        if (upHit) {
            upPos.z = stepHit.point.z - 0.01f;
            if (upPos.z <= footPos.z + 0.1f) {
                if (mStepLog) std::fprintf(stderr, "[STEP]   FAIL: ceiling too low (%.3f)\n", upPos.z);
                return false;
            }
        }

        // ── Phase 2: Raycast FORWARD (velocity * 0.01) ──
        float fwdDist = hSpeed * STEP_FWD_SCALE;
        Vector3 fwdPos = upPos + moveDir * fwdDist;

        if (mStepLog) {
            std::fprintf(stderr, "[STEP]   P2 FWD: (%.3f,%.3f,%.3f)->(%.3f,%.3f,%.3f) dist=%.3f\n",
                upPos.x, upPos.y, upPos.z, fwdPos.x, fwdPos.y, fwdPos.z, fwdDist);
        }
        int32_t fwdCellHint = cellHint;
        if (raycastWorld(mCollision.getWR(), upPos, fwdPos, stepHit, &fwdCellHint, cellHint)) {
            if (mStepLog) std::fprintf(stderr, "[STEP]   P2 FWD FAIL: hit at (%.3f,%.3f,%.3f)\n",
                stepHit.point.x, stepHit.point.y, stepHit.point.z);
            return false;
        }

        // ── Phase 3: Raycast DOWN (4 units) ──
        // Try multiple cell hints to handle thin riser cells.
        static constexpr float STEP_DOWN_DIST = 4.0f;
        Vector3 downTarget = fwdPos - Vector3(0.0f, 0.0f, STEP_DOWN_DIST);

        if (mStepLog) {
            std::fprintf(stderr, "[STEP]   P3 DOWN: (%.3f,%.3f,%.3f)->(%.3f,%.3f,%.3f)\n",
                fwdPos.x, fwdPos.y, fwdPos.z, downTarget.x, downTarget.y, downTarget.z);
        }
        bool downOk = raycastWorld(mCollision.getWR(), fwdPos, downTarget, stepHit, nullptr, fwdCellHint);
        if (!downOk)
            downOk = raycastWorld(mCollision.getWR(), fwdPos, downTarget, stepHit, nullptr, mCellIdx);
        if (!downOk)
            downOk = raycastWorld(mCollision.getWR(), fwdPos, downTarget, stepHit);
        if (!downOk) {
            if (mStepLog) std::fprintf(stderr, "[STEP]   P3 DOWN FAIL: no hit in any cell\n");
            return false;
        }

        float hitZ = stepHit.point.z;
        Vector3 stepGroundNormal = stepHit.normal;

        // Original engine does NOT check walkability here — it only validates
        // via BODY sphere collision.  Reject only surfaces pointing downward
        // (hit the underside of geometry), which can't be stood on.
        if (stepGroundNormal.z <= 0.0f) {
            if (mStepLog) std::fprintf(stderr, "[STEP]   P3 REJECT: downward face (nZ=%.2f)\n", stepGroundNormal.z);
            return false;
        }

        // ── Compute step height ──
        // Original: z_delta = hit_z - foot_z, clamp to min 0.3, add 0.02.
        // NO max cap — original relies on BODY sphere validation only.
        float zDelta = hitZ - footPos.z;
        if (zDelta < STEP_MIN_ZDELTA)
            zDelta = STEP_MIN_ZDELTA;
        zDelta += STEP_CLEARANCE;

        if (mStepLog) {
            std::fprintf(stderr, "[STEP]   P3 OK: hitZ=%.3f hitN=(%.3f,%.3f,%.3f) raw=%.3f zDelta=%.3f\n",
                hitZ, stepGroundNormal.x, stepGroundNormal.y, stepGroundNormal.z,
                hitZ - footPos.z, zDelta);
        }

        // ── Validate: test sphere submodels at stepped position ──
        // Original engine loops NumSubModels(), checks each sphere type.
        // Spring-connected submodels (HEAD, index 0) are validated at their CURRENT
        // position (not the stepped position), because the original sets:
        //   if (GetSpringTension(i) > 0) SetEndLocationVec(i, GetLocationVec(i));
        // This means HEAD trivially passes (it's already at its current position).
        // Only non-spring spheres (BODY) are validated at the stepped position.
        // Point detectors (SHIN/KNEE/FOOT, radius 0.0) are skipped (no volume).
        // Use backupPos as the base for the stepped position, matching the original
        // engine where CheckStep runs from the backed-up collision point.
        Vector3 steppedPos = backupPos;
        steppedPos.z += zDelta;

        int32_t steppedCell = mCollision.findCell(steppedPos);
        if (steppedCell < 0) {
            if (mStepLog) std::fprintf(stderr, "[STEP]   VALIDATE FAIL: no cell at steppedPos=(%.3f,%.3f,%.3f)\n",
                steppedPos.x, steppedPos.y, steppedPos.z);
            return false;
        }

        if (mStepLog) {
            std::fprintf(stderr, "[STEP]   VALIDATE: steppedPos=(%.3f,%.3f,%.3f) cell=%d\n",
                steppedPos.x, steppedPos.y, steppedPos.z, steppedCell);
        }

        for (int s = 0; s < NUM_SPHERES; ++s) {
            // Skip non-sphere submodels (point detectors have radius 0)
            if (mSphereRadii[s] < 0.001f)
                continue;

            // Skip HEAD (index 0) — spring-connected, validated at current position
            // which it already occupies (trivially passes). Original engine behavior.
            if (s == 0)
                continue;

            float poseOffsetZ = 0.0f;
            if (s == 1) poseOffsetZ = mBodyPoseCurrent.z; // BODY
            float offsetZ = mSphereOffsetsBase[s] + poseOffsetZ;
            Vector3 sphereCenter = steppedPos + Vector3(0.0f, 0.0f, offsetZ);

            mStepScratchContacts.clear();
            mCollision.sphereVsCellPolygons(sphereCenter, mSphereRadii[s],
                                             steppedCell, mStepScratchContacts);
            int32_t sphereCell = mCollision.findCell(sphereCenter);
            if (sphereCell >= 0 && sphereCell != steppedCell)
                mCollision.sphereVsCellPolygons(sphereCenter, mSphereRadii[s],
                                                 sphereCell, mStepScratchContacts);

            // Reject if penetration exceeds STEP_VALIDATE_EPS (0.001, matches
            // original engine's SPHR_EPSILON from SPHRCST.CPP line 57). The original
            // uses a swept spherecast with implicit time-based tolerance; our static
            // test needs an explicit penetration threshold to filter floating-point
            // noise from exact-touch cases (sphere surface tangent to a polygon).
            for (const auto &c : mStepScratchContacts) {
                if (c.penetration > STEP_VALIDATE_EPS) {
                    if (mStepLog) {
                        Vector3 sc = steppedPos + Vector3(0.0f, 0.0f,
                            mSphereOffsetsBase[s] + (s == 1 ? mBodyPoseCurrent.z : 0.0f));
                        std::fprintf(stderr, "[STEP]   VALIDATE FAIL: sub%d pen=%.6f "
                            "sphereC=(%.3f,%.3f,%.3f) r=%.1f cell=%d "
                            "cN=(%.3f,%.3f,%.3f) cCell=%d cPoly=%d\n",
                            s, c.penetration,
                            sc.x, sc.y, sc.z, mSphereRadii[s], steppedCell,
                            c.normal.x, c.normal.y, c.normal.z,
                            c.cellIdx, c.polyIdx);
                    }
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
        mGroundTextureIdx = stepHit.textureIndex;  // tread texture from DOWN raycast

        // HEAD spring compensation — offset so camera stays at current world Z,
        // then spring naturally blends up (smooth camera step transition).
        mSpringPos.z -= zDelta;

        // Destroy leg contacts and create synthetic FOOT contact.
        // Matches original: DestroyAllTerrainContacts(KNEE/SHIN/FOOT) +
        // CreateTerrainContact(FOOT, new_poly).
        mContacts.erase(
            std::remove_if(mContacts.begin(), mContacts.end(),
                           [](const SphereContact &c) { return c.submodelIdx >= 2; }),
            mContacts.end());

        {
            SphereContact footContact;
            footContact.normal = stepGroundNormal;
            footContact.penetration = 0.01f;
            footContact.cellIdx = stepHit.cellIdx >= 0 ? stepHit.cellIdx : steppedCell;
            footContact.polyIdx = stepHit.polyIdx;
            footContact.textureIdx = stepHit.textureIndex;
            footContact.fresh = true;
            footContact.submodelIdx = 4;
            mContacts.push_back(footContact);
            // SetGroundObj (D20): update ground surface on stair step.
            // Matches original (PHCORE.CPP line 5511):
            // g_pPlayerMovement->SetGroundObj(new_contact.GetObjID())
            mGroundObjID = footContact.objectId;
        }

        // Break any active climb state before mode transition.
        // Original calls BreakClimb(objID, FALSE, FALSE) here.
        if (mCurrentMode == PlayerMode::Climb) {
            mCurrentMode = PlayerMode::Stand;
            mClimbingObjId = -1;  // release climb target
            mClimbFaceIdx = -1;
            mClimbFaceNormal = Vector3(0.0f);
        }

        // Mode transition — match original: non-Stand/Crouch/Swim/Dead → Stand
        if (mCurrentMode != PlayerMode::Stand &&
            mCurrentMode != PlayerMode::Crouch &&
            mCurrentMode != PlayerMode::Swim &&
            mCurrentMode != PlayerMode::Dead)
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
        if (mStepLog) {
            std::fprintf(stderr, "[STEP-REINT] dt=%.4f pos=(%.3f,%.3f,%.3f) vel=(%.3f,%.3f,%.3f)\n",
                dt, mPosition.x, mPosition.y, mPosition.z,
                mVelocity.x, mVelocity.y, mVelocity.z);
        }

        // 1. Rebuild constraints from current contacts.
        //    Original PostCollisionUpdate (phcore.cpp lines 4131-4136):
        //      ClearConstraints()
        //      ConstrainFromObjects(i) + ConstrainFromTerrain(i) for each submodel
        //    The synthetic FOOT contact created by CheckStep needs to produce a
        //    floor constraint so gravity doesn't pull the player through the tread.
        //    Without this, constrainVelocity() uses stale constraints from the
        //    frame start — missing the new tread's floor constraint.
        mConstraints.clear();
        for (const auto &c : mContacts) {
            if (c.isEdge) {
                // Edge: distance-only check (simplified — contact just created)
                mConstraints.push_back({c.normal, c.objectId});
            } else {
                // Face: trust the contact is valid (just created by CheckStep or
                // validated at frame start). Original rebuilds from persistent
                // contacts without re-validating distance/polygon.
                mConstraints.push_back({c.normal, c.objectId});
            }
        }

        // 2. Apply gravity for remaining time — unconditionally.
        //    Original: PostCollisionUpdate (phcore.cpp line 4142) calls
        //    UpdateModelDynamics which applies gravity every frame (lines 1417-1421).
        mVelocity.z -= mGravityMag * dt;

        // 3. Apply movement control for remaining time
        //    Original: UpdateModelControls applies player input.
        applyMovement();

        // 4. Constrain velocity against rebuilt contacts.
        //    Original: ApplyConstraints (phmod.cpp line 1861) removes velocity
        //    into surfaces. The new tread's floor constraint removes downward
        //    velocity from gravity, preventing the player from falling through.
        constrainVelocity();

        if (mStepLog) {
            std::fprintf(stderr, "[STEP-REINT] post-constrain vel=(%.3f,%.3f,%.3f)\n",
                mVelocity.x, mVelocity.y, mVelocity.z);
        }

        // 4. Integrate position for remaining time.
        //    Matches original: PostCollisionUpdate → UpdateModel →
        //    UpdateTargetLocation (PHMOD.CPP line 1871) uses pure velocity * dt.
        //    No kPartialBackupAmt — that's only for IntegrateToCollision backup.
        Vector3 preReintegratePos = mPosition;  // save for swept test
        mPosition += mVelocity * dt;

        if (mStepLog) {
            std::fprintf(stderr, "[STEP-REINT] post-integrate pos=(%.3f,%.3f,%.3f)\n",
                mPosition.x, mPosition.y, mPosition.z);
        }

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
        Vector3 sphereCenters[NUM_SPHERES];
        for (int s = 0; s < NUM_SPHERES; ++s) {
            float poseOffsetZ = 0.0f;
            if (s == 0) poseOffsetZ = mPoseCurrent.z;
            else if (s == 1) poseOffsetZ = mBodyPoseCurrent.z;
            float offsetZ = mSphereOffsetsBase[s] + poseOffsetZ;
            Vector3 sphereCenter = mPosition + Vector3(0.0f, 0.0f, offsetZ);
            Vector3 oldSphereCenter = preReintegratePos + Vector3(0.0f, 0.0f, offsetZ);
            sphereCenters[s] = sphereCenter;

            size_t contactsBefore = mIterContacts.size();

            // Collision dispatch by submodel type — must match the main collision
            // loop (PlayerPhysics_Collision.inl). Original PostCollisionUpdate
            // (phcore.cpp line 4150) calls CheckModelTerrainCollisions which uses
            // the same dispatch: sphere submodels → SphrSpherecastStatic, point
            // submodels → PortalRaycast.
            if (mSphereRadii[s] > 0.001f) {
                // ── Sphere path (HEAD/BODY): static + swept sphere tests ──
                mCollision.sphereVsCellPolygons(sphereCenter, mSphereRadii[s],
                                                 mCellIdx, mIterContacts);
                int32_t sphereCell = mCollision.findCell(sphereCenter);
                if (sphereCell >= 0 && sphereCell != mCellIdx)
                    mCollision.sphereVsCellPolygons(sphereCenter, mSphereRadii[s],
                                                     sphereCell, mIterContacts);

                // Swept test from pre-integration to post-integration position.
                mCollision.sweptSphereVsCellPolygons(
                    oldSphereCenter, sphereCenter, mSphereRadii[s], mCellIdx, mIterContacts);
                if (sphereCell >= 0 && sphereCell != mCellIdx)
                    mCollision.sweptSphereVsCellPolygons(
                        oldSphereCenter, sphereCenter, mSphereRadii[s], sphereCell, mIterContacts);
                // Also test adjacent cells from submodel's cell
                int32_t portalBase = (sphereCell >= 0) ? sphereCell : mCellIdx;
                if (portalBase >= 0 && portalBase < static_cast<int32_t>(mCollision.getWR().numCells)) {
                    const auto &pc = mCollision.getWR().cells[portalBase];
                    int nSolid = pc.numPolygons - pc.numPortals;
                    for (int pi = nSolid; pi < pc.numPolygons; ++pi) {
                        int32_t tgt = static_cast<int32_t>(pc.polygons[pi].tgtCell);
                        if (tgt >= 0 && tgt != mCellIdx && tgt != sphereCell
                            && tgt < static_cast<int32_t>(mCollision.getWR().numCells)) {
                            mCollision.sweptSphereVsCellPolygons(
                                oldSphereCenter, sphereCenter, mSphereRadii[s], tgt, mIterContacts);
                        }
                    }
                }
            } else {
                // ── Point path (SHIN/KNEE/FOOT): PortalRaycast ──
                // Matches original: point submodels use PortalRaycast (line-segment
                // old→new) for portal-traversing polygon detection. This is critical
                // for detecting riser faces across cell boundaries during cascade.
                RayHit pointHit;
                if (raycastWorld(mCollision.getWR(), oldSphereCenter, sphereCenter, pointHit)) {
                    Vector3 delta = sphereCenter - oldSphereCenter;
                    float rayLen = glm::length(delta);
                    float hitTime = (rayLen > 1e-6f) ? pointHit.distance / rayLen : 0.5f;
                    hitTime = std::clamp(hitTime, 0.0f, 1.0f);

                    SphereContact contact;
                    contact.normal = pointHit.normal;
                    float endDist = glm::dot(pointHit.normal, sphereCenter - pointHit.point);
                    contact.penetration = std::max(-endDist, 0.0f);
                    contact.cellIdx = pointHit.cellIdx >= 0 ? pointHit.cellIdx : mCellIdx;
                    contact.polyIdx = pointHit.polyIdx;
                    contact.textureIdx = pointHit.textureIndex;
                    contact.time = hitTime;
                    contact.isEdge = false;  // face contact (kPC_TerrainFace)
                    mIterContacts.push_back(contact);
                }
            }

            for (size_t ci = contactsBefore; ci < mIterContacts.size(); ++ci)
                mIterContacts[ci].submodelIdx = static_cast<int8_t>(s);
        }

        // Object collision pass — previously missing from re-integration.
        // Original engine's PostCollisionUpdate re-checks both terrain AND objects.
        if (mObjectCollisionCb) {
            mObjectCollisionCb(sphereCenters, mSphereRadii,
                               NUM_SPHERES, mCellIdx, mIterContacts);
        }

        if (mStepLog) {
            std::fprintf(stderr, "[STEP-REINT] re-collision: %zu contacts, pos=(%.3f,%.3f,%.3f)\n",
                mIterContacts.size(), mPosition.x, mPosition.y, mPosition.z);
        }

        // ── Cascade collision processing (matching original PostCollisionUpdate) ──
        // Original: CheckModelTerrainCollisions queues new collisions, ResolveCollisions
        // processes them with IntegrateToCollision + CheckStep. The model_time tracking
        // ensures IntegrateToCollision only advances by the remaining time fraction.
        // We check for riser contacts and attempt CheckStep before doing push-out.
        if (!mIterContacts.empty()) {
            // Check for riser contacts that should trigger cascade stair step
            float earliestRiserTime = 2.0f;
            int bestRiserIdx = -1;
            for (int ci = 0; ci < static_cast<int>(mIterContacts.size()); ++ci) {
                const auto &c = mIterContacts[ci];
                if (c.submodelIdx < 2) continue;
                if (std::fabs(c.normal.z) >= STEP_WALL_THRESHOLD) continue;
                if (c.isEdge) continue;
                float t = c.time >= 0.0f ? c.time : 0.5f;
                // Skip contacts at sweep start (foot ON the plane)
                if (c.submodelIdx >= 2 && c.time >= 0.0f) {
                    float footOff = mSphereOffsetsBase[c.submodelIdx];
                    Vector3 footSt = preReintegratePos + Vector3(0.0f, 0.0f, footOff);
                    float sDist = glm::dot(c.normal, footSt - c.hitPoint);
                    if (sDist <= 0.0f) continue;
                }
                if (t < earliestRiserTime) {
                    earliestRiserTime = t;
                    bestRiserIdx = ci;
                }
            }

            if (mStepLog) {
                std::fprintf(stderr, "[STEP-REINT] cascade search: %zu contacts, bestRiser=%d\n",
                    mIterContacts.size(), bestRiserIdx);
                for (int ci = 0; ci < static_cast<int>(mIterContacts.size()); ++ci) {
                    const auto &c = mIterContacts[ci];
                    std::fprintf(stderr, "[STEP-REINT]   [%d] sub=%d n=(%.2f,%.2f,%.2f) t=%.3f edge=%d\n",
                        ci, c.submodelIdx, c.normal.x, c.normal.y, c.normal.z, c.time, c.isEdge);
                }
            }

            if (bestRiserIdx >= 0) {
                // Cascade CheckStep — matches original PostCollisionUpdate → ResolveCollisions
                float cascadeFrac = std::clamp(earliestRiserTime, 0.0f, 1.0f);
                float cascadeRemaining = dt * (1.0f - cascadeFrac);
                cascadeRemaining = std::max(cascadeRemaining, 0.0001f);

                // IntegrateToCollision for point submodel (use hit location)
                const auto &rc = mIterContacts[bestRiserIdx];
                Vector3 cascadeBackup;
                if (rc.submodelIdx >= 2) {
                    float footOff = mSphereOffsetsBase[rc.submodelIdx];
                    Vector3 footPos = preReintegratePos + Vector3(0.0f, 0.0f, footOff);
                    Vector3 footBk = footPos + (rc.hitPoint - footPos) * kPartialBackupAmt;
                    cascadeBackup = footBk - Vector3(0.0f, 0.0f, footOff);
                } else {
                    cascadeBackup = preReintegratePos + mVelocity *
                        (cascadeFrac * dt * kPartialBackupAmt);
                }

                if (mStepLog) {
                    std::fprintf(stderr, "[STEP-REINT] cascade riser sub=%d n=(%.2f,%.2f,%.2f) "
                        "remaining=%.4f\n", rc.submodelIdx,
                        rc.normal.x, rc.normal.y, rc.normal.z, cascadeRemaining);
                }

                if (tryStairStepFromContacts(mIterContacts, cascadeRemaining, cascadeBackup)) {
                    // Cascade step succeeded — postStepReIntegrate already called recursively
                    return;
                }

                // Cascade step failed — bounce and continue
                float dp = glm::dot(mVelocity, rc.normal);
                if (dp < 0.0f) {
                    float dampen = mElasticity * kTerrainBounce;
                    mVelocity -= rc.normal * dp * (1.0f + dampen);
                }
            }

            // No push-out in PostCollisionUpdate — the original engine does NOT
            // do position += normal * penetration during collision resolution.
            // Collision response is purely velocity-based (bounce reflection)
            // plus position advancement (IntegrateToCollision). Push-out only
            // exists in the main resolveCollisions loop, not in the post-step
            // re-integration path. See PHCORE.CPP lines 4122-4152.
            //
            // We still accumulate contacts for next frame's constrainVelocity,
            // but do NOT modify mPosition based on penetration.
            // Accumulate contacts for next frame's constrainVelocity
            mFreshContacts.insert(mFreshContacts.end(),
                                 mIterContacts.begin(), mIterContacts.end());
        }

        // Update cell after push-out
        int32_t finalCell = mCollision.findCell(mPosition);
        if (finalCell >= 0) mCellIdx = finalCell;

        if (mStepLog) {
            std::fprintf(stderr, "[STEP-REINT] final pos=(%.3f,%.3f,%.3f) cell=%d\n",
                mPosition.x, mPosition.y, mPosition.z, mCellIdx);
        }
    }

    /// Object stair stepping — step onto OBB world objects (crates, tables, etc).
    /// Same structure as terrain stepping but uses ray-vs-OBB tests.
    inline bool tryObjectStairStep(const Vector3 &moveDir, float hSpeed,
                                    const Vector3 &footPos, float remainingDt) {
        if (!mObjectWorld)
            return false;

        const ObjectCollisionBody *stepOBB = nullptr;
        int triggerFaceIdx = -1;

        for (const auto &c : mContacts) {
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

        // Compute lift (same as terrain: min 0.3, + 0.02, no max cap —
        // original relies on sphere validation only)
        float zDelta = hitZ - footPos.z;
        if (zDelta < STEP_MIN_ZDELTA) zDelta = STEP_MIN_ZDELTA;
        zDelta += STEP_CLEARANCE;

        // Validate sphere submodels at stepped position (skip HEAD — spring-connected)
        Vector3 steppedPos = mPosition;
        steppedPos.z += zDelta;
        int32_t steppedCell = mCollision.findCell(steppedPos);
        if (steppedCell < 0) return false;

        for (int s = 0; s < NUM_SPHERES; ++s) {
            if (mSphereRadii[s] < 0.001f)
                continue;
            if (s == 0) continue; // HEAD: spring-connected, validated at current pos (trivially passes)
            float poseOffsetZ = 0.0f;
            if (s == 1) poseOffsetZ = mBodyPoseCurrent.z;
            float offsetZ = mSphereOffsetsBase[s] + poseOffsetZ;
            Vector3 sphereCenter = steppedPos + Vector3(0.0f, 0.0f, offsetZ);
            mStepScratchContacts.clear();
            mCollision.sphereVsCellPolygons(sphereCenter, mSphereRadii[s],
                                             steppedCell, mStepScratchContacts);
            int32_t sphereCell = mCollision.findCell(sphereCenter);
            if (sphereCell >= 0 && sphereCell != steppedCell)
                mCollision.sphereVsCellPolygons(sphereCenter, mSphereRadii[s],
                                                 sphereCell, mStepScratchContacts);
            for (const auto &c : mStepScratchContacts) {
                if (c.penetration > STEP_VALIDATE_EPS) return false;
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
            objContact.fresh = true;
            objContact.submodelIdx = 4;
            objContact.objectId = stepOBB->objID;
            mContacts.push_back(objContact);
            // SetGroundObj (D20): update ground surface on object stair step.
            // Matches original (PHCORE.CPP line 5537):
            // g_pPlayerMovement->SetGroundObj(pOBBModel->GetObjID())
            mGroundObjID = stepOBB->objID;
        }

        if (mCurrentMode != PlayerMode::Stand &&
            mCurrentMode != PlayerMode::Crouch &&
            mCurrentMode != PlayerMode::Swim &&
            mCurrentMode != PlayerMode::Dead)
            mCurrentMode = PlayerMode::Stand;
        if (mCurrentMode == PlayerMode::Climb)
            mCurrentMode = PlayerMode::Stand;

        // PostCollisionUpdate: re-integrate for remaining frame time.
        // Previously missing — object steps got no physics re-run, causing
        // asymmetric behavior vs terrain steps.
        if (remainingDt > 0.0001f) {
            postStepReIntegrate(remainingDt);
        }

        return true;
    }

