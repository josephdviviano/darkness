// Included inside PlayerPhysics class body — do not include standalone.

    /// Pre-constrain velocity against per-frame constraints (rebuilt from validated contacts
    /// by validateContacts()). Removes velocity components going into known surfaces,
    /// preventing the body from moving into walls/floor and needing to be pushed back out.
    /// Matches original Dark Engine's ApplyConstraints (PHMOD.CPP lines 1053-1118).
    inline void constrainVelocity() {
        if (mConstraints.empty()) return;

        if (mStepLog) {
            // Print both post-gravity (pre-movement) and post-movement velocity so the
            // friction contribution to vz is legible. On stationary ground the two
            // usually differ by the 1.4×-boosted friction Z-push that partially cancels
            // gravity before this function zeroes the residual.
            std::fprintf(stderr,
                "[CONSTRAIN] %zu constraints, vel=(%.2f,%.2f,%.2f) preMove.z=%.2f\n",
                mConstraints.size(),
                mVelocity.x, mVelocity.y, mVelocity.z,
                mPreMovementVelocity.z);
            for (const auto &con : mConstraints) {
                std::fprintf(stderr, "[CONSTRAIN]   n=(%.2f,%.2f,%.2f) obj=%d\n",
                    con.normal.x, con.normal.y, con.normal.z, con.objectId);
            }
        }

        // Collect constraint normals from mConstraints (built by validateContacts).
        // Matches original PhysConstrain (PHUTILS.CPP lines 147-154): extracts
        // direction vectors from tConstraint list for PhysRemNormComp.
        static constexpr int MAX_CONSTRAINTS = 12;
        Vector3 constraintBuf[MAX_CONSTRAINTS];
        int constraintCount = 0;

        for (const auto &con : mConstraints) {
            if (constraintCount >= MAX_CONSTRAINTS) break;
            // De-duplicate by normal direction to prevent double-removal
            bool duplicate = false;
            for (int i = 0; i < constraintCount; ++i) {
                if (glm::dot(con.normal, constraintBuf[i]) > 0.99f) { duplicate = true; break; }
            }
            if (!duplicate)
                constraintBuf[constraintCount++] = con.normal;
        }

        // PhysRemNormComp — faithful reimplementation (PHUTILS.CPP lines 26-113).
        // Removes velocity components going into constrained surfaces.
        //
        // The original ALWAYS routes through the N-constraint solver for 2+ constraints.
        // The N-solver filters by actual violation first (dot <= 0), then decides whether
        // to use the 2-constraint crease slide or sequential single-constraint removal.
        // We must NOT short-circuit to the crease slide for 2 constraints — a floor + wall
        // combination where velocity only violates one surface must apply single removal,
        // not crease projection (which would lock movement to the floor/wall edge).
        if (constraintCount == 1) {
            // Single constraint: remove component along normal (lines 115-122).
            float vn = glm::dot(mVelocity, constraintBuf[0]);
            if (vn < 0.0f)
                mVelocity -= constraintBuf[0] * vn;

        } else if (constraintCount >= 2) {
            // N-constraint solver (lines 26-113): filter, test 2-pairs, sequential fallback.
            Vector3 origVel = mVelocity;

            // Step 1: Find which constraints are actually violated (dot <= 0).
            int realCount = 0;
            int realIdx1 = -1, realIdx2 = -1;
            for (int i = 0; i < constraintCount; ++i) {
                if (glm::dot(mVelocity, constraintBuf[i]) <= 0.0f) {
                    realCount++;
                    if (realCount > 2) break;
                    if (realIdx1 < 0) realIdx1 = i;
                    else realIdx2 = i;
                }
            }

            // Step 2: If exactly 2 violated, test if one alone suffices (lines 63-81).
            if (realCount == 2) {
                Vector3 testVel = mVelocity;
                // Try removing constraint 1 only
                float vn1 = glm::dot(testVel, constraintBuf[realIdx1]);
                if (vn1 < 0.0f) testVel -= constraintBuf[realIdx1] * vn1;
                // Check if constraint 2 is still violated
                if (glm::dot(testVel, constraintBuf[realIdx2]) <= 0.0f) {
                    // Constraint 1 alone didn't suffice. Try constraint 2 alone.
                    testVel = mVelocity;
                    float vn2 = glm::dot(testVel, constraintBuf[realIdx2]);
                    if (vn2 < 0.0f) testVel -= constraintBuf[realIdx2] * vn2;
                    if (glm::dot(testVel, constraintBuf[realIdx1]) <= 0.0f) {
                        // Neither alone suffices — apply both via crease slide
                        applyTwoConstraints(constraintBuf[realIdx1], constraintBuf[realIdx2]);
                        return; // original returns here (line 78)
                    }
                }
                // Fall through: one constraint sufficed, apply all individually below
            }

            // Step 3: Sequential single-constraint pass (lines 84-88).
            // Each violated constraint is removed individually via PhysRemNormComp.
            for (int i = 0; i < constraintCount; ++i) {
                float vn = glm::dot(mVelocity, constraintBuf[i]);
                if (vn < 0.0f)
                    mVelocity -= constraintBuf[i] * vn;
            }

            // Step 4: Backward check (lines 94-101).
            if (glm::dot(mVelocity, origVel) < 0.0f)
                mVelocity = Vector3(0.0f);

            // Step 5: Final validation — ensure no constraint still violated (lines 104-112).
            for (int i = 0; i < constraintCount; ++i) {
                if (glm::dot(mVelocity, constraintBuf[i]) < -0.0001f) {
                    mVelocity = Vector3(0.0f);
                    break;
                }
            }
        }
    }

    /// Two-constraint removal — crease/wedge slide.
    /// Matches original PhysRemNormComp 2-vector overload (PHUTILS.CPP lines 124-143).
    inline void applyTwoConstraints(const Vector3 &n0, const Vector3 &n1) {
        Vector3 edge = glm::cross(n0, n1);
        float edgeLenSq = glm::dot(edge, edge);
        if (edgeLenSq > 1e-12f) {
            edge /= std::sqrt(edgeLenSq);
            mVelocity = edge * glm::dot(mVelocity, edge);
        } else {
            // Parallel normals — apply both constraints separately (lines 134-135).
            float vn0 = glm::dot(mVelocity, n0);
            if (vn0 < 0.0f) mVelocity -= n0 * vn0;
            float vn1 = glm::dot(mVelocity, n1);
            if (vn1 < 0.0f) mVelocity -= n1 * vn1;
        }
    }

    /// Resolve collisions using the 5-sphere model. For each iteration, tests all 5 submodel
    /// positions against cell polygons, accumulates contacts, and pushes the body center out.
    /// HEAD/BODY are real spheres (r=1.2) for wall/ceiling; SHIN/KNEE/FOOT are point detectors
    /// (r=0) that sense floor contacts and generate upward normals for ground support.
    inline void resolveCollisions(const ContactCallback &contactCb) {
        Vector3 origPos = mPosition;
        int32_t origCell = mCellIdx;

        // ── UpdatePositions: commit end position ──
        // Advance mPosition to the intended end position. The iterative push-out
        // loop will adjust from here if there are penetrations.
        mPosition = mEndPosition;

        // Update cell for the new position — only if valid
        {
            int32_t endCell = mCollision.findCell(mPosition);
            if (endCell >= 0) mCellIdx = endCell;
        }

        // Fresh contacts from this frame's collision detection are accumulated in
        // mFreshContacts. After collision resolution, they're merged into mContacts
        // (deduplicated by cellIdx+polyIdx).
        mFreshContacts.clear();
        mFreshContacts.reserve(16);

        for (int iter = 0; iter < mTimestep.collisionIters; ++iter) {
            if (mCellIdx < 0)
                break;

            mIterContacts.clear();

            for (int s = 0; s < NUM_SPHERES; ++s) {
                float sphereR = mSphereRadii[s];
                // HEAD and BODY get pose-driven vertical offsets for smooth crouch transitions.
                // SHIN/KNEE/FOOT (s >= 2) always use their standing offsets.
                float poseOffsetZ = 0.0f;
                Vector3 springOffset(0.0f);     // base spring offset (start position)
                Vector3 springEndOffset(0.0f);  // spring offset at sweep endpoint
                if (s == 0) {
                    // HEAD: use spring position for start, spring position + velocity*dt for end.
                    // Matches original Dark Engine (PHMOD.CPP lines 1576-1586):
                    // HEAD EndLocationVec = HEAD position + spring_velocity * dt.
                    // The spring velocity contribution extends the sweep to where the
                    // head is actually moving, so collisions are detected along the
                    // spring-driven path (important during crouch transitions/bob).
                    poseOffsetZ = mSpringPos.z;
                    springOffset = Vector3(mSpringPos.x, mSpringPos.y, 0.0f);
                    springEndOffset = springOffset + Vector3(mSpringVel.x, mSpringVel.y, mSpringVel.z) * mTimestep.fixedDt;
                } else if (s == 1) {
                    poseOffsetZ = mBodyPoseCurrent.z; // BODY
                }
                float offsetZ = mSphereOffsetsBase[s] + poseOffsetZ;
                // Swept test from pre-commit position to current (pushed) position.
                // On the first iteration this sweeps from origPos to mEndPosition.
                // On subsequent iterations the swept distance is zero (static only).
                // HEAD uses spring end offset (includes velocity*dt) for the new position.
                Vector3 sphereCenter = mPosition + Vector3(0.0f, 0.0f, offsetZ) + springEndOffset;
                Vector3 oldSphereCenter = origPos + Vector3(0.0f, 0.0f, offsetZ) + springOffset;

                // Track contact count to tag new contacts with submodel index.
                size_t contactsBefore = mIterContacts.size();

                // ── Collision dispatch by submodel type ──
                // Matches original Dark Engine (PHMODSPH.CPP lines 132-282):
                // - Sphere submodels (HEAD/BODY, radius > 0): SphrSpherecastStatic
                //   (swept sphere from old→new + static overlap)
                // - Point submodels (SHIN/KNEE/FOOT, radius = 0): PortalRaycast
                //   (line-segment raycast from old→new position)
                //
                // The original's point submodel collision (PHMODSPH.CPP line 174-240)
                // uses PortalRaycast to detect polygon crossings. On hit, it creates
                // a kPC_TerrainFace collision event with the crossing time. This is how
                // stair riser contacts are detected for CheckStep triggering.

                if (sphereR > 0.001f) {
                    // ── Sphere path (HEAD/BODY): static + swept sphere tests ──

                    // Static test at new position
                    mCollision.sphereVsCellPolygons(
                        sphereCenter, sphereR, mCellIdx, mIterContacts);

                    int32_t sphereCell = mCollision.findCell(sphereCenter);
                    if (sphereCell >= 0 && sphereCell != mCellIdx) {
                        mCollision.sphereVsCellPolygons(
                            sphereCenter, sphereR, sphereCell, mIterContacts);
                    }

                    // Portal adjacency
                    int32_t portalBaseCell = (sphereCell >= 0) ? sphereCell : mCellIdx;
                    if (portalBaseCell >= 0 && portalBaseCell < static_cast<int32_t>(mCollision.getWR().numCells)) {
                        const auto &portalCell = mCollision.getWR().cells[portalBaseCell];
                        int numSolidPortal = portalCell.numPolygons - portalCell.numPortals;
                        for (int pi = numSolidPortal; pi < portalCell.numPolygons; ++pi) {
                            const auto &poly = portalCell.polygons[pi];
                            if (poly.count < 3) continue;
                            const auto &plane = portalCell.planes[poly.plane];
                            float dist = plane.getDistance(sphereCenter);
                            if (dist < sphereR) {
                                int32_t tgtCell = static_cast<int32_t>(poly.tgtCell);
                                if (tgtCell >= 0 && tgtCell != mCellIdx && tgtCell != sphereCell
                                    && tgtCell != portalBaseCell
                                    && tgtCell < static_cast<int32_t>(mCollision.getWR().numCells)) {
                                    mCollision.sphereVsCellPolygons(
                                        sphereCenter, sphereR, tgtCell, mIterContacts);
                                }
                            }
                        }
                    }

                    // Swept sphere from old→new
                    int32_t oldSphereCell = mCollision.findCell(oldSphereCenter);
                    mCollision.sweptSphereVsCellPolygons(
                        oldSphereCenter, sphereCenter, sphereR, mCellIdx, mIterContacts);
                    if (sphereCell >= 0 && sphereCell != mCellIdx)
                        mCollision.sweptSphereVsCellPolygons(
                            oldSphereCenter, sphereCenter, sphereR, sphereCell, mIterContacts);
                    if (oldSphereCell >= 0 && oldSphereCell != mCellIdx && oldSphereCell != sphereCell)
                        mCollision.sweptSphereVsCellPolygons(
                            oldSphereCenter, sphereCenter, sphereR, oldSphereCell, mIterContacts);
                    if (portalBaseCell >= 0 && portalBaseCell < static_cast<int32_t>(mCollision.getWR().numCells)) {
                        const auto &portalCell = mCollision.getWR().cells[portalBaseCell];
                        int numSolidPortal = portalCell.numPolygons - portalCell.numPortals;
                        for (int pi = numSolidPortal; pi < portalCell.numPolygons; ++pi) {
                            const auto &poly = portalCell.polygons[pi];
                            if (poly.count < 3) continue;
                            int32_t tgtCell = static_cast<int32_t>(poly.tgtCell);
                            if (tgtCell >= 0 && tgtCell != mCellIdx && tgtCell != sphereCell
                                && tgtCell != oldSphereCell && tgtCell != portalBaseCell
                                && tgtCell < static_cast<int32_t>(mCollision.getWR().numCells)) {
                                mCollision.sweptSphereVsCellPolygons(
                                    oldSphereCenter, sphereCenter, sphereR, tgtCell, mIterContacts);
                            }
                        }
                    }

                } else {
                    // ── Point path (SHIN/KNEE/FOOT): PortalRaycast ──
                    // Matches original PHMODSPH.CPP lines 174-240: point-type submodels
                    // use PortalRaycast (line-segment old→new) instead of sphere sweep.
                    // On hit, creates a kPC_TerrainFace collision with crossing time.
                    // This is how stair riser contacts are generated for CheckStep.
                    RayHit pointHit;
                    if (raycastWorld(mCollision.getWR(), oldSphereCenter, sphereCenter, pointHit)) {
                        // Compute parametric collision time [0,1]
                        Vector3 delta = sphereCenter - oldSphereCenter;
                        float rayLen = glm::length(delta);
                        float hitTime = (rayLen > 1e-6f) ? pointHit.distance / rayLen : 0.5f;
                        hitTime = std::clamp(hitTime, 0.0f, 1.0f);

                        SphereContact contact;
                        contact.normal = pointHit.normal;
                        // Penetration = how far past the surface the endpoint is.
                        // For a raycast hit, compute distance from endpoint to hit plane.
                        float endDist = glm::dot(pointHit.normal, sphereCenter - pointHit.point);
                        contact.penetration = std::max(-endDist, 0.0f);
                        contact.cellIdx = pointHit.cellIdx >= 0 ? pointHit.cellIdx : mCellIdx;
                        contact.polyIdx = pointHit.polyIdx;
                        contact.textureIdx = pointHit.textureIndex;
                        contact.time = hitTime;
                        contact.isEdge = false;  // face contact (kPC_TerrainFace)
                        contact.hitPoint = pointHit.point;  // store for IntegrateToCollision
                        mIterContacts.push_back(contact);
                    }
                }

                // Tag contacts with submodel index for tryStairStep() and detectGround().
                for (size_t ci = contactsBefore; ci < mIterContacts.size(); ++ci) {
                    mIterContacts[ci].submodelIdx = static_cast<int8_t>(s);
                }
            }

            // ── Object collision pass ──
            // Test against placed object collision bodies (crates, furniture, doors). The callback
            // invokes ObjectCollisionWorld::testPlayerSpheres(), appending SphereContact results
            // (cellIdx=-1 sentinel) into mIterContacts alongside WR contacts.
            if (mObjectCollisionCb) {
                // Build sphere center array from current body position + offsets
                // (same computation as the per-sphere loop above)
                Vector3 sphereCenters[NUM_SPHERES];
                for (int s = 0; s < NUM_SPHERES; ++s) {
                    float poseOffsetZ = 0.0f;
                    Vector3 sOff(0.0f);
                    if (s == 0) { poseOffsetZ = mSpringPos.z; sOff = Vector3(mSpringPos.x, mSpringPos.y, 0.0f); }
                    else if (s == 1) poseOffsetZ = mBodyPoseCurrent.z;
                    float offsetZ = mSphereOffsetsBase[s] + poseOffsetZ;
                    sphereCenters[s] = mPosition + Vector3(0.0f, 0.0f, offsetZ) + sOff;
                }
                mObjectCollisionCb(sphereCenters, mSphereRadii,
                                   NUM_SPHERES, mCellIdx, mIterContacts);
            }

            if (mIterContacts.empty())
                break; // No contacts — done

            // ── Stair stepping via time-ordered collision processing ──
            // Matches original Dark Engine: ResolveCollisions (phcore.cpp lines 6276-6305)
            // finds the earliest collision, calls IntegrateToCollision to back up the
            // model to the collision point (90% along trajectory), then CheckStep.
            //
            // Find the earliest leg-level riser contact with a valid collision time.
            // The swept test stores time ∈ [0,1] for contacts detected during the
            // sweep from mPrevPosition → mPosition.
            {   // No ground-state prerequisite for stair stepping — match original
                // engine (phcore.cpp line 6134). Original checks ONLY: terrain face,
                // steep normal (fabs(z) < 0.4), leg submodel (foot/shin/knee).
                // NO isOnGround() guard. CheckStep's own validation (3-phase raycast
                // + BODY sphere check) rejects invalid steps.

                // ── Diagnostic: dump all 5 submodel positions, velocity, contacts ──
                if (mStepLog) {
                    static const char *subNames[NUM_SPHERES] = {"HEAD", "BODY", "SHIN", "KNEE", "FOOT"};
                    std::fprintf(stderr, "[STEP-DIAG] === Frame iter=%d contacts=%zu ===\n",
                        iter, mIterContacts.size());
                    std::fprintf(stderr, "[STEP-DIAG] prevPos=(%.3f,%.3f,%.3f) pos=(%.3f,%.3f,%.3f)\n",
                        mPrevPosition.x, mPrevPosition.y, mPrevPosition.z,
                        mPosition.x, mPosition.y, mPosition.z);
                    std::fprintf(stderr, "[STEP-DIAG] vel=(%.3f,%.3f,%.3f)\n",
                        mVelocity.x, mVelocity.y, mVelocity.z);
                    for (int si = 0; si < NUM_SPHERES; ++si) {
                        float poz = 0.0f;
                        if (si == 0) poz = mPoseCurrent.z;
                        else if (si == 1) poz = mBodyPoseCurrent.z;
                        float offZ = mSphereOffsetsBase[si] + poz;
                        Vector3 sc = mPosition + Vector3(0.0f, 0.0f, offZ);
                        std::fprintf(stderr, "[STEP-DIAG]   %s[%d] r=%.1f absPos=(%.3f,%.3f,%.3f)\n",
                            subNames[si], si, mSphereRadii[si], sc.x, sc.y, sc.z);
                    }
                    // Contact summary
                    int floorCt = 0, wallCt = 0, sweptCt = 0;
                    for (const auto &cc : mIterContacts) {
                        if (cc.time >= 0.0f) sweptCt++;
                        else if (std::fabs(cc.normal.z) >= STEP_WALL_THRESHOLD) floorCt++;
                        else wallCt++;
                    }
                    std::fprintf(stderr, "[STEP-DIAG] contacts: floor=%d wall=%d swept=%d\n",
                        floorCt, wallCt, sweptCt);
                    // Each riser candidate
                    for (int ci = 0; ci < static_cast<int>(mIterContacts.size()); ++ci) {
                        const auto &cc = mIterContacts[ci];
                        if (cc.submodelIdx < 2) continue;
                        if (std::fabs(cc.normal.z) >= STEP_WALL_THRESHOLD) continue;
                        std::fprintf(stderr, "[STEP-DIAG]   riser[%d] sub=%d n=(%.3f,%.3f,%.3f) "
                            "pen=%.3f t=%.3f edge=%d obj=%d\n",
                            ci, cc.submodelIdx, cc.normal.x, cc.normal.y, cc.normal.z,
                            cc.penetration, cc.time, cc.isEdge, cc.objectId);
                    }
                }

                float earliestTime = 2.0f;  // > 1.0 = no valid swept contact found
                int bestContactIdx = -1;
                for (int ci = 0; ci < static_cast<int>(mIterContacts.size()); ++ci) {
                    const auto &c = mIterContacts[ci];
                    if (c.submodelIdx < 2) continue;          // only SHIN/KNEE/FOOT
                    if (std::fabs(c.normal.z) >= STEP_WALL_THRESHOLD) continue;
                    if (c.isEdge) continue;  // face contacts only (kPC_TerrainFace)
                    // Prefer swept contacts (time >= 0) — they have accurate timing.
                    // Static contacts (time < 0) use fallback timing.
                    float t = c.time;
                    if (t < 0.0f) t = 0.5f;  // static contact: estimate mid-frame
                    // No startDist filter — matches original engine (PHCORE.CPP line 6134).
                    // Any swept riser collision triggers CheckStep regardless of whether
                    // the foot starts behind or at the riser plane. CheckStep's own
                    // 3-phase raycast + sphere validation handles rejection.
                    if (t < earliestTime) {
                        earliestTime = t;
                        bestContactIdx = ci;
                    }
                }

                if (bestContactIdx >= 0) {
                    // Compute remaining frame time from collision time.
                    // Original: PostCollisionUpdate receives dt - pClsn->GetTime()
                    // (phcore.cpp line 6143).
                    float collisionTimeFrac = std::clamp(earliestTime, 0.0f, 1.0f);
                    float collisionTime = mTimestep.fixedDt * collisionTimeFrac;
                    float remainingDt = mTimestep.fixedDt - collisionTime;
                    remainingDt = std::max(remainingDt, 0.0001f);

                    // Track model time — matches original's SetCurrentTime(t0 + dt)
                    // in IntegrateToCollision (PHCORE.CPP line 5170). This prevents
                    // double-integration in cascade collisions.
                    mModelTime = collisionTime;

                    // IntegrateToCollision: compute the collision backup position.
                    // Original (PHCORE.CPP lines 5087-5201) has TWO paths:
                    //   - POINT submodels (line 5145): use hit location directly
                    //       new_pos = currentPos + (hitLoc - currentPos) * 0.9
                    //   - SPHERE submodels (line 5120): use velocity integration
                    //       new_pos = currentPos + velocity * integration_time * 0.9
                    // origPos is the LocationVec at frame start (before UpdatePositions).
                    const auto &bestContact = mIterContacts[bestContactIdx];
                    Vector3 backupPos;
                    if (bestContact.submodelIdx >= 2) {
                        // Point submodel (SHIN/KNEE/FOOT): backup toward hit location.
                        // Original uses double-precision (PHCORE.CPP line 5145-5151):
                        //   movement = (hitLoc - GetLocationVec(i)) * integration_backup
                        //   new_pos = GetLocationVec(i) + movement
                        float footOffsetZ = mSphereOffsetsBase[bestContact.submodelIdx];
                        Vector3 footPos = origPos + Vector3(0.0f, 0.0f, footOffsetZ);
                        Vector3 footBackup = footPos + (bestContact.hitPoint - footPos) * kPartialBackupAmt;
                        // Convert foot backup to body center backup (subtract foot offset)
                        backupPos = footBackup - Vector3(0.0f, 0.0f, footOffsetZ);
                    } else {
                        // Sphere submodel: velocity-based integration
                        backupPos = origPos + mVelocity *
                            (collisionTimeFrac * mTimestep.fixedDt * kPartialBackupAmt);
                    }

                    bool stepped = tryStairStepFromContacts(mIterContacts, remainingDt, backupPos);
                    if (stepped) {
                        // Step succeeded + re-integration done — skip push-out.
                        // Matches original: PostCollisionUpdate called, collision consumed.
                        break;
                    }

                    // ResolveBounce: when CheckStep fails, reflect velocity off the riser.
                    // Matches original Dark Engine flow (phcore.cpp lines 6199-6262):
                    //   1. CheckTerrainContact — if low velocity, create persistent contact
                    //   2. BounceObject — reflect velocity off surface
                    //   3. PostCollisionUpdate — re-integrate for remaining time
                    {
                        const auto &riserContact = mIterContacts[bestContactIdx];
                        float dp = glm::dot(mVelocity, riserContact.normal);

                        // CheckTerrainContact (phcore.cpp lines 4305-4378):
                        // Original: dp = dot(vel, normal) * elasticity; if |dp| < 5.0.
                        // This means |dot| * elasticity < 5.0, i.e. |dot| < 5.0 / elasticity.
                        // Higher elasticity → lower threshold → fewer sticking contacts.
                        static constexpr float kStickingThreshold = 5.0f;
                        float dpScaled = std::fabs(dp) * mElasticity;
                        if (dp < 0.0f && dpScaled < kStickingThreshold) {
                            // Create persistent contact from this collision
                            SphereContact stickContact = riserContact;
                            stickContact.fresh = true;
                            mFreshContacts.push_back(stickContact);
                        }

                        // BounceObject (phcore.cpp lines 4211-4248):
                        // dp = dot(velocity, normal); if (dp < 0) vel -= normal * dp * (1 + dampen)
                        if (dp < 0.0f) {
                            float dampen = mElasticity * kTerrainBounce;
                            mVelocity -= riserContact.normal * dp * (1.0f + dampen);
                            if (mStepLog) {
                                std::fprintf(stderr, "[STEP] BOUNCE: dp=%.3f n=(%.2f,%.2f,%.2f) "
                                    "dampen=%.2f vel->(%.2f,%.2f,%.2f)\n",
                                    dp, riserContact.normal.x, riserContact.normal.y, riserContact.normal.z,
                                    dampen, mVelocity.x, mVelocity.y, mVelocity.z);
                            }
                        }

                        // PostCollisionUpdate: re-run physics pipeline for remaining time
                        // (phcore.cpp lines 6254-6260).
                        if (remainingDt > 0.0001f) {
                            postStepReIntegrate(remainingDt);
                        }
                        break;  // collision consumed by bounce + re-integration
                    }
                }
            }

            // Push body center out of penetrations. De-duplicate by normal direction (dot > 0.99):
            // keep only max penetration per wall. Without this, 3 spheres hitting one wall would
            // triple the push, causing jittery over-correction.
            mPushes.clear();
            for (const auto &c : mIterContacts) {
                Vector3 pushNormal = c.normal;

                // Steep surface handling: surfaces with normal.z > 0 but below
                // the walkable threshold (45°) act as walls — flatten their push
                // normal to horizontal. This prevents the player from riding up
                // steep surfaces using velocity: the position push-out is purely
                // horizontal, and constrainVelocity() will also see the flattened
                // normal, removing only horizontal velocity into the wall.
                // Matches original Dark Engine behavior where steep surfaces
                // triggered sliding, not climbing.
                if (pushNormal.z > 0.0f && pushNormal.z < WALKABLE_SLOPE_THRESHOLD) {
                    pushNormal.z = 0.0f;
                    float len = glm::length(pushNormal);
                    if (len > 0.001f)
                        pushNormal /= len;
                    else
                        continue;  // nearly vertical — no useful horizontal push
                }

                if (c.objectId >= 0 && mIsPushableCb && mIsPushableCb(c.objectId)) {
                    static int dbgCount = 0;
                    if (dbgCount++ < 40)
                        std::fprintf(stderr, "[PUSH-COLLIDE] obj=%d n=(%.2f,%.2f,%.2f) pen=%.3f sub=%d\n",
                                     c.objectId, c.normal.x, c.normal.y, c.normal.z,
                                     c.penetration, c.submodelIdx);
                }

                bool merged = false;
                for (auto &p : mPushes) {
                    if (glm::dot(c.normal, p.first) > 0.99f) {
                        // Same wall — keep the deeper penetration
                        p.second = std::max(p.second, c.penetration);
                        merged = true;
                        break;
                    }
                }
                if (!merged) {
                    mPushes.push_back({pushNormal, c.penetration});
                }
            }
            for (const auto &p : mPushes) {
                mPosition += p.first * p.second;
            }

            // Accumulate fresh contacts for later merge into mContacts
            mFreshContacts.insert(mFreshContacts.end(),
                                  mIterContacts.begin(), mIterContacts.end());

            // Push may have moved through a portal — update cell.
            // Use portal-based tracking (not brute-force) to avoid jumping to
            // an overlapping but topologically-unconnected cell mid-resolution.
            {
                bool cellUpdated = false;
                // Check if still in current cell
                const auto &curC = mCollision.getWR().cells[mCellIdx];
                bool inCur = true;
                for (const auto &pl : curC.planes) {
                    if (pl.getDistance(mPosition) < -0.1f) { inCur = false; break; }
                }
                if (inCur) {
                    cellUpdated = true;
                } else {
                    // Check portal-connected cells
                    int nSolid = curC.numPolygons - curC.numPortals;
                    for (int ppi = nSolid; ppi < curC.numPolygons; ++ppi) {
                        int32_t tc = static_cast<int32_t>(curC.polygons[ppi].tgtCell);
                        if (tc < 0 || tc >= static_cast<int32_t>(mCollision.getWR().numCells))
                            continue;
                        const auto &tgtC = mCollision.getWR().cells[tc];
                        bool inTgt = true;
                        for (const auto &pl : tgtC.planes) {
                            if (pl.getDistance(mPosition) < -0.1f) { inTgt = false; break; }
                        }
                        if (inTgt) {
                            mCellIdx = tc;
                            cellUpdated = true;
                            break;
                        }
                    }
                }
                if (!cellUpdated) {
                    // Brute-force fallback
                    int32_t newCell = mCollision.findCell(mPosition);
                    if (newCell >= 0) {
                        mCellIdx = newCell;
                    } else {
                        // Push-out moved outside all cells. Undo THIS iteration's
                        // push-out only — don't reset the entire frame's movement.
                        // The original engine tracks cells through portal crossings
                        // and doesn't have this failure mode. Our brute-force
                        // findCell can fail near thin cell boundaries.
                        // Revert push-out by going back to pre-push position.
                        mPosition = mEndPosition;
                        // Try to find cell at the un-pushed position
                        newCell = mCollision.findCell(mPosition);
                        if (newCell >= 0) {
                            mCellIdx = newCell;
                        }
                        // Don't return — continue to contact merge
                        break;  // exit iteration loop, proceed to merge
                    }
                }
            }
        }

        // ── move_backup: pull position back along movement direction ──
        // Matches original UpdatePositions (PHCORE.CPP lines 6437-6455): after all
        // collision resolution is complete, pull the final position back by
        // min(0.01, moveLen) along the movement direction. This keeps the next
        // frame's spherecaster epsilon happy — prevents landing exactly on a polygon
        // plane where float precision could cause missed contacts.
        // Applied AFTER collision resolution, matching original placement.
        {
            Vector3 moveVec = mPosition - origPos;
            float moveLen = glm::length(moveVec);
            if (moveLen > 1e-7f) {
                float backupDist = std::min(0.01f, moveLen);
                mPosition -= (moveVec / moveLen) * backupDist;
            }
        }

        // Final validation — body center must still be in a valid cell.
        // If move_backup pushed us outside, try mEndPosition (before backup).
        // Only reset to origPos as absolute last resort.
        if (mCollision.findCell(mPosition) < 0) {
            mPosition = mEndPosition;
        }
        if (mCollision.findCell(mPosition) < 0) {
            mPosition = origPos;
            mCellIdx = origCell;
        }

        // ── Merge fresh contacts into mContacts ──
        // Matches original Dark Engine: contacts are created during collision
        // resolution (CheckTerrainContact, phcore.cpp lines 4305-4378) and
        // persist until explicitly destroyed by ConstrainFromTerrain validation.
        // No age-based persistence — contacts are validated each frame.
        //
        // Deduplicate by (cellIdx, polyIdx): if a fresh contact matches an
        // existing validated contact, refresh the data. Otherwise add it.
        for (auto &nc : mFreshContacts) {
            bool matched = false;
            for (auto &ec : mContacts) {
                if (ec.cellIdx == nc.cellIdx && ec.polyIdx == nc.polyIdx) {
                    // Same polygon — refresh with latest detection data.
                    // Matches original: re-creates the entire contact via
                    // CreateTerrainContact(), ensuring all fields match.
                    ec.normal = nc.normal;
                    ec.penetration = nc.penetration;
                    ec.textureIdx = nc.textureIdx;
                    ec.submodelIdx = nc.submodelIdx;
                    ec.isEdge = nc.isEdge;
                    ec.hitPoint = nc.hitPoint;
                    ec.time = nc.time;
                    ec.fresh = true;
                    matched = true;
                    break;
                }
            }
            if (!matched) {
                nc.fresh = true;
                mContacts.push_back(nc);

                // ── Damped bounce on first object OBB contact ──
                // Small elastic bounce (0.02 scale) — nearly imperceptible but makes
                // objects feel slightly elastic vs perfectly rigid. WR contacts
                // (cellIdx >= 0) have no bounce.
                if (nc.cellIdx == -1) {
                    float dp = glm::dot(mVelocity, nc.normal);
                    if (dp < 0.0f) {
                        constexpr float kBounceScale = 0.02f;
                        mVelocity -= nc.normal * dp * (1.0f + kBounceScale);
                    }
                }
            }
        }

        // Fire contact callbacks for newly-detected contacts only.
        // Matches original: contact events fire on creation, not maintenance.
        if (contactCb) {
            for (const auto &c : mContacts) {
                if (!c.fresh) continue;  // skip maintained contacts
                ContactEvent event;
                event.bodyA = -1; // player entity (archetype ID, placeholder)
                event.bodyB = 0;  // world geometry
                event.point = mPosition - c.normal * (mSphereRadii[0] - c.penetration);
                event.normal = c.normal;
                event.depth = c.penetration;
                event.materialA = -1;
                event.materialB = c.textureIdx;
                contactCb(event);
            }
        }

    }

    /// Test whether the player model can fit at a given position without penetrating world
    /// geometry. Tests all 5 submodels at standing offsets. Returns true if no penetration.
    inline bool canPlayerFitAt(const Vector3 &testPos) const {
        int32_t testCell = mCollision.findCell(testPos);
        if (testCell < 0)
            return false;  // outside all cells — not valid

        std::vector<SphereContact> contacts;
        for (int s = 0; s < NUM_SPHERES; ++s) {
            float sphereR = mSphereRadii[s];
            float offsetZ = mSphereOffsetsBase[s];  // standing offsets (not crouched)
            Vector3 sphereCenter = testPos + Vector3(0.0f, 0.0f, offsetZ);

            contacts.clear();
            mCollision.sphereVsCellPolygons(sphereCenter, sphereR, testCell, contacts);

            // Also check the submodel's own cell if different
            int32_t sphereCell = mCollision.findCell(sphereCenter);
            if (sphereCell >= 0 && sphereCell != testCell) {
                mCollision.sphereVsCellPolygons(sphereCenter, sphereR, sphereCell, contacts);
            }

            // Also check objects (doors, crates, furniture) — prevents mantling
            // through objects that only exist as collision bodies, not WR geometry.
            if (mObjectWorld && sphereR > 0.0f) {
                mObjectWorld->testPlayerSpheres(
                    &sphereCenter, &sphereR, 1, testCell, contacts);
            }

            if (!contacts.empty())
                return false;  // penetration detected — player doesn't fit
        }

        return true;
    }
