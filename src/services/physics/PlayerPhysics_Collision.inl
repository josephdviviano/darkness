// Included inside PlayerPhysics class body — do not include standalone.

    /// Pre-constrain velocity against known contact surfaces from the previous frame. Removes
    /// velocity components going into known walls/floor, preventing the body from moving into
    /// surfaces and needing to be pushed back out (reduces jitter). The position-correction pass
    /// in resolveCollisions() still runs after integration to handle new contacts.
    inline void constrainVelocity() {
        if (mLastContacts.empty()) return;

        // Collect unique constraint normals from previous frame's contacts.
        // Bounded by de-duplication — at most ~5 unique wall directions.
        static constexpr int MAX_CONSTRAINTS = 8;
        Vector3 constraintBuf[MAX_CONSTRAINTS];
        int constraintCount = 0;

        for (const auto &c : mLastContacts) {
            // Only constrain if velocity is moving into the surface
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

        // HEAD/BODY offsets are pose-driven (mPoseCurrent.z, mBodyPoseCurrent.z) for smooth
        // collision sphere transitions during crouch/uncrouch.

        // Age persistent contacts — re-detected ones get age reset to 0.
        for (auto &c : mPersistentContacts)
            c.age++;

        mLastContacts.clear();
        mLastContacts.reserve(16);

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

                // Track contact count to tag new contacts with submodel index.
                size_t contactsBefore = mIterContacts.size();

                // Test against the body center's cell
                mCollision.sphereVsCellPolygons(
                    sphereCenter, sphereR, mCellIdx, mIterContacts);

                // Test the submodel's own cell if it differs from body center's cell
                // (handles straddling portal boundaries, e.g. feet in one cell, head in another)
                int32_t sphereCell = mCollision.findCell(sphereCenter);
                if (sphereCell >= 0 && sphereCell != mCellIdx) {
                    mCollision.sphereVsCellPolygons(
                        sphereCenter, sphereR, sphereCell, mIterContacts);
                }

                // Test adjacent cells via portals. For point detectors (r=0), use
                // POINT_PORTAL_REACH to avoid missing floor contacts near portal boundaries.
                constexpr float POINT_PORTAL_REACH = 0.5f;
                float portalReach = (sphereR > 0.001f) ? sphereR : POINT_PORTAL_REACH;
                const auto &cell = mCollision.getWR().cells[mCellIdx];
                int numSolid = cell.numPolygons - cell.numPortals;
                for (int pi = numSolid; pi < cell.numPolygons; ++pi) {
                    const auto &poly = cell.polygons[pi];
                    if (poly.count < 3) continue;
                    const auto &plane = cell.planes[poly.plane];
                    float dist = plane.getDistance(sphereCenter);
                    if (dist < portalReach) {
                        int32_t tgtCell = static_cast<int32_t>(poly.tgtCell);
                        if (tgtCell >= 0 && tgtCell != mCellIdx && tgtCell != sphereCell
                            && tgtCell < static_cast<int32_t>(mCollision.getWR().numCells)) {
                            mCollision.sphereVsCellPolygons(
                                sphereCenter, sphereR, tgtCell, mIterContacts);
                        }
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

            // Push body center out of penetrations. De-duplicate by normal direction (dot > 0.99):
            // keep only max penetration per wall. Without this, 3 spheres hitting one wall would
            // triple the push, causing jittery over-correction.
            mPushes.clear();
            for (const auto &c : mIterContacts) {
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
                    mPushes.push_back({c.normal, c.penetration});
                }
            }
            for (const auto &p : mPushes) {
                mPosition += p.first * p.second;
            }

            // Accumulate contacts for ground detection and velocity removal
            mLastContacts.insert(mLastContacts.end(),
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

        // ── Contact persistence: merge new contacts into persistent set ──
        // Match by (cellIdx, polyIdx). Matched: age reset + data refreshed. Unmatched: added.
        // Stale (age > CONTACT_MAX_AGE): culled.
        for (const auto &nc : mLastContacts) {
            bool matched = false;
            for (auto &pc : mPersistentContacts) {
                if (pc.cellIdx == nc.cellIdx && pc.polyIdx == nc.polyIdx) {
                    // Same polygon — refresh with latest detection data
                    pc.normal = nc.normal;
                    pc.penetration = nc.penetration;
                    pc.textureIdx = nc.textureIdx;
                    pc.age = 0;
                    matched = true;
                    break;
                }
            }
            if (!matched) {
                SphereContact pc = nc;
                pc.age = 0;
                mPersistentContacts.push_back(pc);

                // ── Damped bounce on first object OBB contact ──
                // Small elastic bounce (0.02 scale) — nearly imperceptible but makes objects feel
                // slightly elastic vs perfectly rigid. WR contacts (cellIdx >= 0) have no bounce.
                // ODE UPGRADE: replaced by dContact.surface.bounce = 0.02.
                if (nc.cellIdx == -1) {
                    float dp = glm::dot(mVelocity, nc.normal);
                    if (dp < 0.0f) {
                        // Remove normal component (dp < 0 = into surface) and
                        // reflect 2% back out. Factor = 1.0 (zero) + 0.02 (bounce).
                        constexpr float kBounceScale = 0.02f;  // matches kOBBBounceScale
                        mVelocity -= nc.normal * dp * (1.0f + kBounceScale);
                    }
                }
            }
        }

        // Cull stale contacts not re-detected within the persistence window
        mPersistentContacts.erase(
            std::remove_if(mPersistentContacts.begin(), mPersistentContacts.end(),
                           [](const SphereContact &c) {
                               return c.age > CONTACT_MAX_AGE;
                           }),
            mPersistentContacts.end());

        // Replace transient mLastContacts with persistent set (fresh + maintained contacts) for
        // all downstream consumers (detectGround, computeGroundFriction, constrainVelocity).
        mLastContacts = mPersistentContacts;

        // Fire contact callbacks for downstream consumers (audio, AI, scripts). Only fire for
        // fresh contacts (age=0) to prevent duplicates. Contact point is approximate (HEAD radius).
        if (contactCb) {
            for (const auto &c : mLastContacts) {
                if (c.age > 0) continue;  // skip persistent-only contacts
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

            if (!contacts.empty())
                return false;  // penetration detected — player doesn't fit
        }

        return true;
    }
