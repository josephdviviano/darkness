// Included inside PlayerPhysics class body — do not include standalone.

    /// Pre-constrain velocity against known contact surfaces from the previous frame. Removes
    /// velocity components going into known walls/floor, preventing the body from moving into
    /// surfaces and needing to be pushed back out (reduces jitter). The position-correction pass
    /// in resolveCollisions() still runs after integration to handle new contacts.
    inline void constrainVelocity() {
        if (mContacts.empty()) return;

        if (mStepLog && !mContacts.empty()) {
            std::fprintf(stderr, "[CONSTRAIN] %zu contacts, vel=(%.2f,%.2f,%.2f)\n",
                mContacts.size(), mVelocity.x, mVelocity.y, mVelocity.z);
            for (const auto &c : mContacts) {
                std::fprintf(stderr, "[CONSTRAIN]   sub=%d n=(%.2f,%.2f,%.2f) cell=%d poly=%d fresh=%d\n",
                    c.submodelIdx, c.normal.x, c.normal.y, c.normal.z,
                    c.cellIdx, c.polyIdx, c.fresh);
            }
        }

        // Collect unique constraint normals from validated contacts.
        // Bounded by de-duplication — at most ~5 unique wall directions.
        static constexpr int MAX_CONSTRAINTS = 8;
        Vector3 constraintBuf[MAX_CONSTRAINTS];
        int constraintCount = 0;

        for (const auto &c : mContacts) {
            // Use FULL normals for velocity constraints (matches original engine's
            // AddConstraint which stores the unmodified surface normal). This removes
            // velocity going INTO the surface in 3D, allowing natural sliding along it.
            // The position push-out (resolveCollisions) separately uses horizontal-only
            // normals for steep surfaces to prevent upward lift.
            float vn = glm::dot(mVelocity, c.normal);
            if (vn >= 0.0f) continue;

            bool duplicate = false;
            for (int i = 0; i < constraintCount; ++i) {
                if (glm::dot(c.normal, constraintBuf[i]) > 0.99f) { duplicate = true; break; }
            }
            if (!duplicate && constraintCount < MAX_CONSTRAINTS)
                constraintBuf[constraintCount++] = c.normal;
        }

        // Apply constraint-based velocity removal
        if (constraintCount == 1) {
            // Single surface — remove velocity component along the normal
            float vn = glm::dot(mVelocity, constraintBuf[0]);
            if (vn < 0.0f)
                mVelocity -= constraintBuf[0] * vn;

        } else if (constraintCount == 2) {
            // Two surfaces (crease/wedge) — slide along the edge between them
            Vector3 edge = glm::cross(constraintBuf[0], constraintBuf[1]);
            float edgeLen = glm::length(edge);
            if (edgeLen > 1e-6f) {
                edge /= edgeLen;
                mVelocity = edge * glm::dot(mVelocity, edge);
            } else {
                float vn = glm::dot(mVelocity, constraintBuf[0]);
                if (vn < 0.0f)
                    mVelocity -= constraintBuf[0] * vn;
            }

        } else if (constraintCount >= 3) {
            // Corner (3+ surfaces) — project onto edge, validate direction
            Vector3 origVel = mVelocity;
            Vector3 edge = glm::cross(constraintBuf[0], constraintBuf[1]);
            float edgeLen = glm::length(edge);
            if (edgeLen > 1e-6f) {
                edge /= edgeLen;
                Vector3 projected = edge * glm::dot(mVelocity, edge);
                if (glm::dot(projected, origVel) < 0.0f)
                    mVelocity = Vector3(0.0f);
                else
                    mVelocity = projected;
            } else {
                mVelocity = Vector3(0.0f);
            }
        }
    }

    /// Resolve collisions using the 5-sphere model. For each iteration, tests all 5 submodel
    /// positions against cell polygons, accumulates contacts, and pushes the body center out.
    /// HEAD/BODY are real spheres (r=1.2) for wall/ceiling; SHIN/KNEE/FOOT are point detectors
    /// (r=0) that sense floor contacts and generate upward normals for ground support.
    inline void resolveCollisions(const ContactCallback &contactCb) {
        Vector3 origPos = mPosition;
        int32_t origCell = mCellIdx;

        // ── UpdatePositions: commit end position with move_backup ──
        // Matches original (PHCORE.CPP lines 6437-6455): commits EndLocationVec →
        // LocationVec, but pulls the position back by min(0.01, moveLen) along the
        // movement direction. This keeps the spherecaster's epsilon happy by ensuring
        // the committed position doesn't land exactly on a polygon plane.
        {
            Vector3 moveVec = mEndPosition - mPosition;
            float moveLen = glm::length(moveVec);
            if (moveLen > 1e-7f) {
                float backupDist = std::min(0.01f, moveLen);
                mPosition = mEndPosition - (moveVec / moveLen) * backupDist;
            } else {
                mPosition = mEndPosition;
            }
        }

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
                if (s == 0) poseOffsetZ = mPoseCurrent.z;        // HEAD
                else if (s == 1) poseOffsetZ = mBodyPoseCurrent.z; // BODY
                float offsetZ = mSphereOffsetsBase[s] + poseOffsetZ;
                Vector3 sphereCenter = mPosition + Vector3(0.0f, 0.0f, offsetZ);
                // Swept test from pre-commit position to current (pushed) position.
                // On the first iteration this sweeps from origPos to mEndPosition.
                // On subsequent iterations the swept distance is zero (static only).
                Vector3 oldSphereCenter = origPos + Vector3(0.0f, 0.0f, offsetZ);

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
                    if (s == 0) poseOffsetZ = mPoseCurrent.z;
                    else if (s == 1) poseOffsetZ = mBodyPoseCurrent.z;
                    float offsetZ = mSphereOffsetsBase[s] + poseOffsetZ;
                    sphereCenters[s] = mPosition + Vector3(0.0f, 0.0f, offsetZ);
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
                    // Skip contacts where the submodel starts ON or BEHIND the plane.
                    // Matches original: PortalRaycast with use_zero_epsilon=TRUE (used for
                    // point submodel collision, PHMODSPH.CPP line 198) has the check
                    // `if (start_dist > 0.0)` at WRCAST.C line 770 — planes at distance
                    // ≤ 0 from the ray start are skipped. This prevents re-detecting a
                    // riser the foot was placed on by a previous stair step.
                    // We check the contact's hitPoint: if the foot's start position is
                    // not strictly in front of the hit plane, skip the contact.
                    if (c.submodelIdx >= 2 && c.time >= 0.0f) {
                        float footOffsetZ = mSphereOffsetsBase[c.submodelIdx];
                        Vector3 footStart = origPos + Vector3(0.0f, 0.0f, footOffsetZ);
                        float startDist = glm::dot(c.normal, footStart - c.hitPoint);
                        if (startDist <= 0.0f) continue;  // foot starts ON or behind the plane
                    }
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
                    float remainingDt = mTimestep.fixedDt * (1.0f - collisionTimeFrac);
                    remainingDt = std::max(remainingDt, 0.0001f);

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
                        // If velocity into the surface is low, create a persistent contact.
                        // Original threshold: |dp| < 5.0 * elasticity. This prevents the
                        // player from drifting into the wall next frame.
                        static constexpr float kStickingThreshold = 5.0f;
                        if (dp < 0.0f && std::fabs(dp) < kStickingThreshold * mElasticity) {
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
                        // Pushed outside all cells.
                        mPosition = origPos;
                        mCellIdx = origCell;
                        return;
                    }
                }
            }
        }

        // Final validation — body center must still be in a valid cell
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
                    // Same polygon — refresh with latest detection data
                    ec.normal = nc.normal;
                    ec.penetration = nc.penetration;
                    ec.textureIdx = nc.textureIdx;
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
