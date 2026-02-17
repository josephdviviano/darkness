// Included inside PlayerPhysics class body — do not include standalone.

    /// Try to step up a small ledge — 3-phase raycast algorithm. Called when FOOT/SHIN/KNEE
    /// contacts have steep normals (z < STEP_WALL_THRESHOLD, wall-like surface at leg level).
    /// Phase 1 (UP): headroom check. Phase 2 (FWD): ledge space check. Phase 3 (DOWN): find
    /// surface. Lift = max(STEP_MIN_ZDELTA, hit_z-foot_z) + STEP_CLEARANCE. Validates BODY/HEAD
    /// spheres at stepped position; aborts on collision. Returns true if step was applied.
    inline bool tryStairStep() {
        // Only attempt while grounded and moving horizontally
        if (!isOnGround())
            return false;

        // Use the player's INTENDED movement direction/speed, not mVelocity.
        // constrainVelocity() (step 7) removes velocity into the riser wall
        // before we get here, so mVelocity's horizontal component is often ~0.
        // The desired velocity from input is unaffected by wall constraints.
        Vector3 desired = computeDesiredVelocity();
        Vector3 hDesired(desired.x, desired.y, 0.0f);
        float hSpeed = glm::length(hDesired);
        if (hSpeed < 0.1f)
            return false;

        Vector3 moveDir = hDesired / hSpeed;

        // Check for steep/vertical normal at leg level (not HEAD/BODY — walls above step height
        // don't trigger stepping). No directional filter — forward probe handles that.
        float footOffsetZ = mSphereOffsetsBase[4]; // FOOT = -3.0
        Vector3 footPos = mPosition + Vector3(0.0f, 0.0f, footOffsetZ);

        bool hasWallContact = false;
        Vector3 bestWallNormal(0.0f);
        for (const auto &c : mLastContacts) {
            // Only FACE contacts from leg-level terrain submodels. Dark Engine only triggers stair
            // stepping on face contacts — edge contacts at polygon boundaries have ambiguous normals
            // and cause spurious stepping. Object contacts excluded (separate OBB stepping path).
            if (std::fabs(c.normal.z) < STEP_WALL_THRESHOLD
                && c.submodelIdx >= 2
                && c.objectId < 0
                && !c.isEdge) {
                hasWallContact = true;
                bestWallNormal = c.normal;
                break;
            }
        }

        // Fallback: short forward ray from FOOT to detect step risers that collision missed
        // (e.g. point detector at polygon edge gap).
        if (!hasWallContact) {
            RayHit fwdProbe;
            Vector3 fwdProbeEnd = footPos + moveDir * SPHERE_RADIUS; // 1.2 units ahead
            if (raycastWorld(mCollision.getWR(), footPos, fwdProbeEnd, fwdProbe)
                && std::fabs(fwdProbe.normal.z) < STEP_WALL_THRESHOLD) {
                hasWallContact = true;
                bestWallNormal = fwdProbe.normal;
            }
        }
        if (!hasWallContact) {
            // No terrain wall contact — try object stepping path instead.
            // Object stepping uses ray-vs-OBB tests (not terrain raycasts) and
            // has different post-step actions (no terrain contact destruction).
            return tryObjectStairStep(moveDir, hSpeed, footPos);
        }

        if (mStepLog) {
            std::fprintf(stderr, "[STEP] ── attempt ── pos=(%.2f,%.2f,%.2f) footZ=%.2f "
                "hSpeed=%.2f dir=(%.2f,%.2f) wallN=(%.2f,%.2f,%.2f) contacts=%zu\n",
                mPosition.x, mPosition.y, mPosition.z, footPos.z,
                hSpeed, moveDir.x, moveDir.y,
                bestWallNormal.x, bestWallNormal.y, bestWallNormal.z,
                mLastContacts.size());
            // Dump per-contact details with submodel index for diagnostics
            for (size_t ci = 0; ci < mLastContacts.size(); ++ci) {
                const auto &c = mLastContacts[ci];
                std::fprintf(stderr, "[STEP]   contact[%zu]: sub=%d n=(%.2f,%.2f,%.2f) pen=%.3f cell=%d poly=%d\n",
                    ci, (int)c.submodelIdx,
                    c.normal.x, c.normal.y, c.normal.z,
                    c.penetration, c.cellIdx, c.polyIdx);
            }
        }

        // ── Phase 1: Raycast UP ──
        // Cast from foot upward by STEP_UP_DIST (2.0), capturing terminal cell hint for
        // subsequent phases. Start offset by STEP_UP_EPSILON (0.01) to avoid self-intersecting
        // with the floor polygon at t=0 (point detectors resolve to exactly dist=0).
        Vector3 upStart = footPos + Vector3(0.0f, 0.0f, STEP_UP_EPSILON);
        Vector3 upPos = footPos + Vector3(0.0f, 0.0f, STEP_UP_DIST);
        RayHit stepHit;
        int32_t cellHint = -1;

        if (raycastWorld(mCollision.getWR(), upStart, upPos, stepHit, &cellHint)) {
            if (mStepLog) std::fprintf(stderr, "[STEP]   P1 UP FAIL: hit at Z=%.2f (ceil)\n",
                stepHit.point.z);
            return false; // ceiling too low
        }

        // ── Phase 2: Raycast FORWARD ──
        // Probe forward at lifted height (distance = velocity * 0.01) using Phase 1 cell hint.
        float fwdDist = hSpeed * STEP_FWD_SCALE;
        Vector3 fwdPos = upPos + moveDir * fwdDist;

        int32_t fwdCellHint = cellHint;
        if (raycastWorld(mCollision.getWR(), upPos, fwdPos, stepHit, &fwdCellHint, cellHint)) {
            if (mStepLog) std::fprintf(stderr, "[STEP]   P2 FWD FAIL: hit at (%.2f,%.2f,%.2f) fwdDist=%.4f\n",
                stepHit.point.x, stepHit.point.y, stepHit.point.z, fwdDist);
            return false; // wall in the way at the lifted height
        }

        if (mStepLog) {
            std::fprintf(stderr, "[STEP]   P1+P2 OK: fwdDist=%.4f fwdPos=(%.2f,%.2f,%.2f) "
                "cellHint=%d->%d\n",
                fwdDist, fwdPos.x, fwdPos.y, fwdPos.z, cellHint, fwdCellHint);
        }

        // ── Phase 3: Raycast DOWN ──
        // Find step tread surface. BFS raycast can't traverse vertical portals with a vertical
        // ray (denom ≈ 0), so when hint points to a tiny riser cell the ray gets stuck.
        // Try propagated hint first, fall back to findCameraCell(fwdPos). Cast distance 4.0
        // (2× STEP_UP_DIST) handles downward steps.
        static constexpr float STEP_DOWN_DIST = 4.0f;
        Vector3 downTarget = fwdPos - Vector3(0.0f, 0.0f, STEP_DOWN_DIST);

        if (!raycastWorld(mCollision.getWR(), fwdPos, downTarget, stepHit, nullptr, fwdCellHint)) {
            // Hint-based raycast failed — the hint likely points to a
            // riser cell with only vertical portals. Retry using
            // findCameraCell to locate the correct cell for fwdPos.
            if (!raycastWorld(mCollision.getWR(), fwdPos, downTarget, stepHit)) {
                if (mStepLog) std::fprintf(stderr, "[STEP]   P3 DOWN FAIL: no surface found (hint=%d, cell-find also failed)\n",
                    fwdCellHint);
                return false;
            }
            if (mStepLog) std::fprintf(stderr, "[STEP]   P3 DOWN: cell-find fallback succeeded\n");
        }

        float hitZ = stepHit.point.z;
        Vector3 stepGroundNormal = stepHit.normal;
        bool foundGround = (stepGroundNormal.z > GROUND_NORMAL_MIN);

        if (mStepLog) {
            std::fprintf(stderr, "[STEP]   P3 DOWN: hitZ=%.3f n=(%.2f,%.2f,%.2f) dist=%.3f "
                "ground=%d hint=%d\n",
                hitZ, stepGroundNormal.x, stepGroundNormal.y, stepGroundNormal.z,
                stepHit.distance, (int)foundGround, fwdCellHint);
        }

        if (!foundGround) {
            if (mStepLog) std::fprintf(stderr, "[STEP]   P3 REJECT: hit surface is not ground (nZ=%.2f)\n",
                stepGroundNormal.z);
            return false;
        }

        // Compute lift: z_delta = max(0.3, hit_z - foot_z) + 0.02
        float zDelta = hitZ - footPos.z;
        if (zDelta < STEP_MIN_ZDELTA)
            zDelta = STEP_MIN_ZDELTA;
        zDelta += STEP_CLEARANCE;

        // Don't step if the lift would exceed the probe range
        if (zDelta > STEP_UP_DIST + STEP_CLEARANCE) {
            if (mStepLog) std::fprintf(stderr, "[STEP]   REJECT: zDelta=%.3f > max=%.3f (too high)\n",
                zDelta, STEP_UP_DIST + STEP_CLEARANCE);
            return false;
        }

        if (mStepLog) {
            std::fprintf(stderr, "[STEP]   P3 OK: hitZ=%.2f zDelta=%.3f (raw=%.3f clamped to min=%.1f +clearance=%.2f)\n",
                hitZ, zDelta, hitZ - footPos.z, STEP_MIN_ZDELTA, STEP_CLEARANCE);
        }

        // ── Validate: test BODY sphere at the stepped position ──
        // Effectively only BODY is validated at the stepped height.
        // We validate BODY (index 1) to ensure it fits at the new height.
        Vector3 steppedPos = mPosition;
        steppedPos.z += zDelta;

        int32_t steppedCell = mCollision.findCell(steppedPos);
        if (steppedCell < 0) {
            if (mStepLog) std::fprintf(stderr, "[STEP]   VALIDATE FAIL: steppedPos=(%.2f,%.2f,%.2f) no cell\n",
                steppedPos.x, steppedPos.y, steppedPos.z);
            return false;
        }

        {
            float bodyOffsetZ = mSphereOffsetsBase[1] + mBodyPoseCurrent.z;
            Vector3 bodyCenter = steppedPos + Vector3(0.0f, 0.0f, bodyOffsetZ);

            mStepScratchContacts.clear();
            mCollision.sphereVsCellPolygons(bodyCenter, SPHERE_RADIUS,
                                             steppedCell, mStepScratchContacts);
            int32_t bodyCell = mCollision.findCell(bodyCenter);
            if (bodyCell >= 0 && bodyCell != steppedCell) {
                mCollision.sphereVsCellPolygons(bodyCenter, SPHERE_RADIUS,
                                                 bodyCell, mStepScratchContacts);
            }

            for (const auto &c : mStepScratchContacts) {
                // Wall or ceiling collision at the stepped position — abort.
                // Allow floor contacts since we expect to rest on the step.
                if (c.normal.z < STEP_WALL_THRESHOLD && c.penetration > 0.01f) {
                    if (mStepLog) {
                        std::fprintf(stderr, "[STEP]   VALIDATE FAIL: BODY collision n=(%.2f,%.2f,%.2f) pen=%.3f "
                            "bodyCenter=(%.2f,%.2f,%.2f)\n",
                            c.normal.x, c.normal.y, c.normal.z, c.penetration,
                            bodyCenter.x, bodyCenter.y, bodyCenter.z);
                    }
                    return false;
                }
            }

            if (mStepLog) {
                std::fprintf(stderr, "[STEP]   VALIDATE OK: bodyCenter=(%.2f,%.2f,%.2f) bodyCell=%d contacts=%zu\n",
                    bodyCenter.x, bodyCenter.y, bodyCenter.z, bodyCell >= 0 ? bodyCell : steppedCell,
                    mStepScratchContacts.size());
            }
        }

        // ── Validate HEAD sphere at its CURRENT position ──
        // HEAD validation tests at the current world position, not the stepped position.
        // This is correct because HEAD won't actually move during the step; the
        // spring will gradually pull it up over subsequent frames.
        {
            float headOffsetZ = mSphereOffsetsBase[0] + mPoseCurrent.z;
            Vector3 headCenter = mPosition + Vector3(0.0f, 0.0f, headOffsetZ);

            mStepScratchContacts.clear();
            mCollision.sphereVsCellPolygons(headCenter, SPHERE_RADIUS,
                                             steppedCell, mStepScratchContacts);
            int32_t headCell = mCollision.findCell(headCenter);
            if (headCell >= 0 && headCell != steppedCell) {
                mCollision.sphereVsCellPolygons(headCenter, SPHERE_RADIUS,
                                                 headCell, mStepScratchContacts);
            }

            for (const auto &c : mStepScratchContacts) {
                // Wall or ceiling collision at the stepped position — abort.
                // Same rejection criteria as BODY: floor contacts are OK.
                if (c.normal.z < STEP_WALL_THRESHOLD && c.penetration > 0.01f) {
                    if (mStepLog) {
                        std::fprintf(stderr, "[STEP]   VALIDATE FAIL: HEAD collision n=(%.2f,%.2f,%.2f) pen=%.3f "
                            "headCenter=(%.2f,%.2f,%.2f)\n",
                            c.normal.x, c.normal.y, c.normal.z, c.penetration,
                            headCenter.x, headCenter.y, headCenter.z);
                    }
                    return false;
                }
            }

            if (mStepLog) {
                std::fprintf(stderr, "[STEP]   VALIDATE OK: headCenter=(%.2f,%.2f,%.2f) headCell=%d contacts=%zu\n",
                    headCenter.x, headCenter.y, headCenter.z, headCell >= 0 ? headCell : steppedCell,
                    mStepScratchContacts.size());
            }
        }

        // ── Accept the step ──
        if (mStepLog) {
            std::fprintf(stderr, "[STEP]   ACCEPTED: pos (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f) "
                "zDelta=%.3f cell %d->%d\n",
                mPosition.x, mPosition.y, mPosition.z,
                steppedPos.x, steppedPos.y, steppedPos.z,
                zDelta, mCellIdx, steppedCell);
        }

        mPosition = steppedPos;
        mCellIdx = steppedCell;
        mVelocity.z = 0.0f;
        mGroundNormal = stepGroundNormal;

        // HEAD spring compensation — offset mSpringPos.z by -zDelta to keep camera at current
        // world Z. Spring naturally blends back toward mPoseCurrent (smooth step-up for camera).
        mSpringPos.z -= zDelta;

        // Destroy leg terrain contacts (submodels 2-4) and create new FOOT contact at step surface.
        // Matches the original engine's DestroyAllTerrainContacts + CreateTerrainContact pattern.
        mLastContacts.erase(
            std::remove_if(mLastContacts.begin(), mLastContacts.end(),
                           [](const SphereContact &c) {
                               return c.submodelIdx >= 2 || c.submodelIdx < 0;
                           }),
            mLastContacts.end());

        // Synthesize FOOT ground contact at step surface for correct ground state next frame.
        {
            SphereContact footContact;
            footContact.normal = stepGroundNormal;
            footContact.penetration = 0.01f; // nominal contact
            footContact.cellIdx = steppedCell;
            footContact.polyIdx = -1; // synthetic (no exact poly from raycast)
            footContact.textureIdx = stepHit.textureIndex;
            footContact.age = 0;
            footContact.submodelIdx = 4; // FOOT
            mLastContacts.push_back(footContact);
        }

        // Mode transition — force Stand unless already Stand/Crouch/Swim/Dead.
        if (mCurrentMode != PlayerMode::Stand &&
            mCurrentMode != PlayerMode::Crouch &&
            mCurrentMode != PlayerMode::Swim &&
            mCurrentMode != PlayerMode::Dead) {
            mCurrentMode = PlayerMode::Stand;
        }

        // Break climb state — always calls BreakClimb
        // after a successful step to cancel any active climbing.
        if (mCurrentMode == PlayerMode::Climb) {
            mCurrentMode = PlayerMode::Stand;
        }

        return true;
    }

    /// Object stair stepping — step onto OBB world objects (crates, tables, etc). Called when no
    /// terrain wall contact found. Scans for OBB contacts from leg-level submodels, then runs
    /// 3-phase ray-vs-OBB (UP/FWD/DOWN) against the triggering OBB (not terrain). Validation at
    /// stepped position is terrain-only.
    /// Key differences from terrain: rayVsOBB not raycasts, nZ<0.4 (no fabs), climbable OBBs
    /// suppressed, edge triggers skipped, no leg contact destruction, OBB face becomes ground.
    inline bool tryObjectStairStep(const Vector3 &moveDir, float hSpeed,
                                    const Vector3 &footPos) {
        if (!mObjectWorld)
            return false;

        // ── Scan for OBB wall contacts from leg-level submodels ──
        // Criteria: object contact, SHIN/KNEE/FOOT, not climbable, not edge trigger,
        // face normal Z < 0.4 (wall-like; no fabs — differs from terrain).
        const ObjectCollisionBody *stepOBB = nullptr;
        int triggerFaceIdx = -1;

        for (const auto &c : mLastContacts) {
            if (c.objectId < 0 || c.submodelIdx < 2)
                continue;

            const ObjectCollisionBody *body = mObjectWorld->findBodyByObjID(c.objectId);
            if (!body || body->shapeType != CollisionShapeType::OBB)
                continue;

            // Climbable OBBs — suppress stepping entirely.
            // The climbing system handles these objects (future task).
            if (body->climbableSides != 0)
                continue;

            // Edge triggers — not physical objects, skip.
            if (body->isEdgeTrigger)
                continue;

            // Get the OBB face index from the encoded polyIdx.
            // polyIdx = (bodyIndex << 4) | (faceIdx & 0xF)
            int faceIdx = c.polyIdx & 0xF;
            if (faceIdx > 5)
                continue; // sphere contact (0xF), not a valid face

            Vector3 faceNormal = getOBBFaceNormal(*body, faceIdx);

            // Wall-like face check: nZ < 0.4 (no fabs — object stepping
            // only triggers on faces whose normal points more sideways than up).
            if (faceNormal.z < 0.4f) {
                stepOBB = body;
                triggerFaceIdx = faceIdx;
                break;
            }
        }

        if (!stepOBB)
            return false;

        if (mStepLog) {
            std::fprintf(stderr, "[STEP-OBJ] ── attempt ── objID=%d pos=(%.2f,%.2f,%.2f) "
                "footZ=%.2f hSpeed=%.2f dir=(%.2f,%.2f) face=%d\n",
                stepOBB->objID, mPosition.x, mPosition.y, mPosition.z,
                footPos.z, hSpeed, moveDir.x, moveDir.y, triggerFaceIdx);
        }

        // ── Phase 1: Ray UP against OBB ──
        // Cast from foot upward by STEP_UP_DIST (2.0). If the ray hits the
        // OBB, the step is too tall (obstacle above).
        Vector3 upStart = footPos + Vector3(0.0f, 0.0f, STEP_UP_EPSILON);
        Vector3 upEnd = footPos + Vector3(0.0f, 0.0f, STEP_UP_DIST);

        RayOBBResult upResult = rayVsOBB(upStart, upEnd, *stepOBB);
        if (upResult.hit) {
            if (mStepLog) std::fprintf(stderr, "[STEP-OBJ]   P1 UP FAIL: OBB hit at Z=%.2f\n",
                upResult.point.z);
            return false;
        }

        // ── Phase 2: Ray FORWARD against OBB ──
        // Probe forward from the lifted position. Distance = speed * 0.01.
        float fwdDist = hSpeed * STEP_FWD_SCALE;
        Vector3 fwdEnd = upEnd + moveDir * fwdDist;

        RayOBBResult fwdResult = rayVsOBB(upEnd, fwdEnd, *stepOBB);
        if (fwdResult.hit) {
            if (mStepLog) std::fprintf(stderr, "[STEP-OBJ]   P2 FWD FAIL: OBB hit at (%.2f,%.2f,%.2f)\n",
                fwdResult.point.x, fwdResult.point.y, fwdResult.point.z);
            return false;
        }

        if (mStepLog) {
            std::fprintf(stderr, "[STEP-OBJ]   P1+P2 OK: fwdDist=%.4f fwdEnd=(%.2f,%.2f,%.2f)\n",
                fwdDist, fwdEnd.x, fwdEnd.y, fwdEnd.z);
        }

        // ── Phase 3: Ray DOWN against OBB ──
        // Cast downward from lifted+forward position by 4.0 units (2× STEP_UP_DIST).
        // Must hit the OBB to find the landing surface (the top face).
        static constexpr float OBJ_STEP_DOWN_DIST = 4.0f;
        Vector3 downEnd = fwdEnd - Vector3(0.0f, 0.0f, OBJ_STEP_DOWN_DIST);

        RayOBBResult downResult = rayVsOBB(fwdEnd, downEnd, *stepOBB);
        if (!downResult.hit) {
            if (mStepLog) std::fprintf(stderr, "[STEP-OBJ]   P3 DOWN FAIL: no OBB surface found\n");
            return false;
        }

        int topFaceIdx = downResult.faceIdx;
        float hitZ = downResult.point.z;
        Vector3 topNormal = getOBBFaceNormal(*stepOBB, topFaceIdx);

        if (mStepLog) {
            std::fprintf(stderr, "[STEP-OBJ]   P3 DOWN: hitZ=%.3f face=%d n=(%.2f,%.2f,%.2f)\n",
                hitZ, topFaceIdx, topNormal.x, topNormal.y, topNormal.z);
        }

        // Compute lift: z_delta = max(0.3, hit_z - foot_z) + 0.02
        float zDelta = hitZ - footPos.z;
        if (zDelta < STEP_MIN_ZDELTA)
            zDelta = STEP_MIN_ZDELTA;
        zDelta += STEP_CLEARANCE;

        if (zDelta > STEP_UP_DIST + STEP_CLEARANCE) {
            if (mStepLog) std::fprintf(stderr, "[STEP-OBJ]   REJECT: zDelta=%.3f > max=%.3f\n",
                zDelta, STEP_UP_DIST + STEP_CLEARANCE);
            return false;
        }

        if (mStepLog) {
            std::fprintf(stderr, "[STEP-OBJ]   P3 OK: hitZ=%.2f zDelta=%.3f\n", hitZ, zDelta);
        }

        // ── Validate: terrain-only sphere tests at stepped position ──
        // Same as terrain stepping: check BODY and HEAD spheres for terrain
        // collisions at the new height. No OBB re-check needed.
        Vector3 steppedPos = mPosition;
        steppedPos.z += zDelta;

        int32_t steppedCell = mCollision.findCell(steppedPos);
        if (steppedCell < 0) {
            if (mStepLog) std::fprintf(stderr, "[STEP-OBJ]   VALIDATE FAIL: no cell at steppedPos\n");
            return false;
        }

        // BODY validation
        {
            float bodyOffsetZ = mSphereOffsetsBase[1] + mBodyPoseCurrent.z;
            Vector3 bodyCenter = steppedPos + Vector3(0.0f, 0.0f, bodyOffsetZ);

            mStepScratchContacts.clear();
            mCollision.sphereVsCellPolygons(bodyCenter, SPHERE_RADIUS,
                                             steppedCell, mStepScratchContacts);
            int32_t bodyCell = mCollision.findCell(bodyCenter);
            if (bodyCell >= 0 && bodyCell != steppedCell) {
                mCollision.sphereVsCellPolygons(bodyCenter, SPHERE_RADIUS,
                                                 bodyCell, mStepScratchContacts);
            }

            for (const auto &c : mStepScratchContacts) {
                if (c.normal.z < STEP_WALL_THRESHOLD && c.penetration > 0.01f) {
                    if (mStepLog) {
                        std::fprintf(stderr, "[STEP-OBJ]   VALIDATE FAIL: BODY terrain collision "
                            "n=(%.2f,%.2f,%.2f) pen=%.3f\n",
                            c.normal.x, c.normal.y, c.normal.z, c.penetration);
                    }
                    return false;
                }
            }
        }

        // HEAD validation at current position (spring-connected, won't move)
        {
            float headOffsetZ = mSphereOffsetsBase[0] + mPoseCurrent.z;
            Vector3 headCenter = mPosition + Vector3(0.0f, 0.0f, headOffsetZ);

            mStepScratchContacts.clear();
            mCollision.sphereVsCellPolygons(headCenter, SPHERE_RADIUS,
                                             steppedCell, mStepScratchContacts);
            int32_t headCell = mCollision.findCell(headCenter);
            if (headCell >= 0 && headCell != steppedCell) {
                mCollision.sphereVsCellPolygons(headCenter, SPHERE_RADIUS,
                                                 headCell, mStepScratchContacts);
            }

            for (const auto &c : mStepScratchContacts) {
                if (c.normal.z < STEP_WALL_THRESHOLD && c.penetration > 0.01f) {
                    if (mStepLog) {
                        std::fprintf(stderr, "[STEP-OBJ]   VALIDATE FAIL: HEAD terrain collision "
                            "n=(%.2f,%.2f,%.2f) pen=%.3f\n",
                            c.normal.x, c.normal.y, c.normal.z, c.penetration);
                    }
                    return false;
                }
            }
        }

        // ── Accept the object step ──
        if (mStepLog) {
            std::fprintf(stderr, "[STEP-OBJ]   ACCEPTED: objID=%d pos (%.2f,%.2f,%.2f) -> "
                "(%.2f,%.2f,%.2f) zDelta=%.3f face=%d\n",
                stepOBB->objID,
                mPosition.x, mPosition.y, mPosition.z,
                steppedPos.x, steppedPos.y, steppedPos.z,
                zDelta, topFaceIdx);
        }

        mPosition = steppedPos;
        mCellIdx = steppedCell;
        mVelocity.z = 0.0f;
        mGroundNormal = topNormal;

        // HEAD spring compensation (same as terrain stepping)
        mSpringPos.z -= zDelta;

        // Object stepping does NOT destroy leg terrain contacts.
        // Only create a new object contact for FOOT standing on the OBB top face.
        {
            SphereContact objContact;
            objContact.normal = topNormal;
            objContact.penetration = 0.01f;
            objContact.cellIdx = -1; // object contact sentinel
            objContact.polyIdx = -1; // synthetic
            objContact.textureIdx = -1;
            objContact.age = 0;
            objContact.submodelIdx = 4; // FOOT
            objContact.objectId = stepOBB->objID;
            mLastContacts.push_back(objContact);
        }

        // Mode transition — same as terrain stepping
        if (mCurrentMode != PlayerMode::Stand &&
            mCurrentMode != PlayerMode::Crouch &&
            mCurrentMode != PlayerMode::Swim &&
            mCurrentMode != PlayerMode::Dead) {
            mCurrentMode = PlayerMode::Stand;
        }

        // Break climb state
        if (mCurrentMode == PlayerMode::Climb) {
            mCurrentMode = PlayerMode::Stand;
        }

        return true;
    }
