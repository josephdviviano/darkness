// Included inside PlayerPhysics class body — do not include standalone.
//
// OBB climbing — player attaches to objects with climbable_sides bitmask (ladders,
// climbable architecture). Terrain wall climbing is disabled, matching the original
// Dark Engine behavior. Forward input = climb up, backward = climb down, strafe = lateral.
// Gravity is suppressed; the player must actively climb to maintain position.

    /// Check for climbable OBB contacts and enter climbing mode.
    /// Called from fixedStep() after collision resolution when NOT already climbing.
    /// Scans mLastContacts for object contacts on climbable OBB faces that the
    /// player is facing toward (dot(forward, faceNormal) <= 0).
    inline bool checkClimb() {
        if (!mObjectWorld)
            return false;

        // Player's facing direction (XY plane)
        Vector3 forward(mCosYaw, mSinYaw, 0.0f);

        for (const auto &c : mLastContacts) {
            // Only object contacts (terrain contacts have objectId < 0)
            if (c.objectId < 0 || c.cellIdx != -1)
                continue;

            // Look up the OBB body
            const auto *body = mObjectWorld->findBodyByObjID(c.objectId);
            if (!body || body->climbableSides == 0)
                continue;

            // Decode which face was hit (encoded in lower 4 bits of polyIdx)
            int faceIdx = c.polyIdx & 0xF;
            if (faceIdx < 0 || faceIdx > 5)
                continue;

            // Check if this face is marked climbable
            if ((body->climbableSides & (1 << faceIdx)) == 0)
                continue;

            // Compute world-space face normal
            Vector3 faceNormal = getOBBFaceNormal(*body, faceIdx);

            // Facing check: player must be facing toward the surface (not away)
            if (glm::dot(forward, faceNormal) > 0.0f)
                continue;

            // Enter climbing mode
            mClimbingObjId = c.objectId;
            mClimbFaceNormal = faceNormal;
            mClimbFaceIdx = faceIdx;
            mCurrentMode = PlayerMode::Climb;

            // Remove velocity component going into the wall (preserve tangential).
            // The original engine applies face constraints on entry rather than
            // zeroing all velocity, avoiding a "hitch" when grabbing a ladder at speed.
            float normalVel = glm::dot(mVelocity, faceNormal);
            if (normalVel < 0.0f)
                mVelocity -= faceNormal * normalVel;

            // Cancel any active lean
            if (mLeanDir != 0) {
                mLeanDir = 0;
                mLeanAmount = 0.0f;
                activatePose(POSE_NORMAL);
            }

            if (mClimbLog) {
                std::fprintf(stderr, "[CLIMB] Enter: objID=%d face=%d "
                    "normal=(%.2f,%.2f,%.2f) pos=(%.1f,%.1f,%.1f)\n",
                    c.objectId, faceIdx,
                    faceNormal.x, faceNormal.y, faceNormal.z,
                    mPosition.x, mPosition.y, mPosition.z);
            }
            return true;
        }
        return false;
    }

    /// Verify the player is still close enough to the climbable surface.
    /// Uses a distance-based leash (2.0 units past any OBB face) rather than
    /// contact-based detection. This slack is what lets
    /// the player slide over the top of a ladder naturally (forward+upward
    /// momentum from looking up carries them over the edge while still climbing
    /// with gravity disabled).
    ///
    /// Also updates the face normal if the player has moved to an adjacent face
    /// (contact-based face tracking).
    inline void checkClimbContinuation() {
        if (mClimbingObjId < 0)
            return;

        const auto *body = mObjectWorld
            ? mObjectWorld->findBodyByObjID(mClimbingObjId) : nullptr;
        if (!body) {
            breakClimb(false);
            return;
        }

        // Distance-based leash: check how far the player body is past each
        // OBB face plane. If > 2.0 units past any face, break the climb.
        // This replaces contact-based break detection and gives the player
        // room to slide over the top edge of the OBB.
        static constexpr float CLIMB_LEASH = 2.0f;

        Vector3 offset = mPosition - body->worldPos;
        Vector3 halfExt = body->edgeLengths * 0.5f;

        // Project offset onto each local axis and check distance past face
        for (int axis = 0; axis < 3; ++axis) {
            float proj = glm::dot(offset, body->rotation[axis]);
            float dist = std::fabs(proj) - halfExt[axis];
            if (dist > CLIMB_LEASH) {
                if (mClimbLog) {
                    std::fprintf(stderr, "[CLIMB] Leash exceeded: axis=%d dist=%.2f > %.1f "
                        "— breaking climb on objID=%d\n",
                        axis, dist, CLIMB_LEASH, mClimbingObjId);
                }
                breakClimb(false);
                return;
            }
        }

        // Update face normal from contacts — supports multi-face blending for smooth
        // corner transitions. The original engine uses weighted edge constraints when
        // the player is in front of 2 faces, producing a blended edge normal.
        bool hasCurrentContact = false;
        Vector3 blendedNormal(0.0f);
        int lastFace = -1;
        int faceCount = 0;

        for (const auto &c : mLastContacts) {
            if (c.objectId != mClimbingObjId || c.cellIdx != -1)
                continue;

            hasCurrentContact = true;

            int faceIdx = c.polyIdx & 0xF;
            if (faceIdx < 0 || faceIdx > 5)
                continue;

            if ((body->climbableSides & (1 << faceIdx)) == 0)
                continue;

            // Accumulate normals for blending (handles edge/corner cases)
            Vector3 faceN = getOBBFaceNormal(*body, faceIdx);
            if (faceCount == 0 || faceIdx != lastFace) {
                blendedNormal += faceN;
                lastFace = faceIdx;
                ++faceCount;
            }
        }

        if (faceCount > 0) {
            float len = glm::length(blendedNormal);
            if (len > 0.001f)
                mClimbFaceNormal = blendedNormal / len;
            mClimbFaceIdx = lastFace;
        }

        // Adjacent OBB transfer: if we have no contacts on the current climbing
        // object but ARE touching a different climbable OBB, transfer directly.
        if (!hasCurrentContact) {
            for (const auto &c : mLastContacts) {
                if (c.objectId <= 0 || c.objectId == mClimbingObjId || c.cellIdx != -1)
                    continue;

                const auto *newBody = mObjectWorld->findBodyByObjID(c.objectId);
                if (!newBody || newBody->climbableSides == 0)
                    continue;

                int newFace = c.polyIdx & 0xF;
                if (newFace < 0 || newFace > 5)
                    continue;
                if ((newBody->climbableSides & (1 << newFace)) == 0)
                    continue;

                // Transfer to new climbing object
                mClimbingObjId = c.objectId;
                mClimbFaceIdx = newFace;
                mClimbFaceNormal = getOBBFaceNormal(*newBody, newFace);
                if (mClimbLog) {
                    std::fprintf(stderr, "[CLIMB] Transfer to objID=%d face=%d "
                        "normal=(%.2f,%.2f,%.2f)\n",
                        c.objectId, newFace,
                        mClimbFaceNormal.x, mClimbFaceNormal.y, mClimbFaceNormal.z);
                }
                return;
            }
        }
    }

    /// Exit climbing mode.
    /// If jumping: compute jump-off direction by reflecting facing off the climbing
    /// surface normal. If facing away from wall, jump forward; if facing toward wall,
    /// reflect and dampen.
    inline void breakClimb(bool jumping) {
        if (mClimbLog) {
            std::fprintf(stderr, "[CLIMB] Break: jumping=%s objID=%d\n",
                jumping ? "true" : "false", mClimbingObjId);
        }

        if (jumping) {
            // Applies the NORMAL jump velocity first
            // (strip vertical, boost horizontal 5%, add 14 up), then adds
            // the climb jump-off impulse on top.

            // Step 1: Normal jump — strip vertical, boost horizontal, add upward
            mVelocity.z = 0.0f;                    // remove vertical component
            mVelocity *= 1.05f;                     // 5% horizontal boost
            mVelocity.z += JUMP_IMPULSE;            // 14.0 upward

            // Step 2: Determine jump direction — check for near-top vault first
            Vector3 forward(mCosYaw, mSinYaw, 0.0f);
            float facingDot = glm::dot(forward, mClimbFaceNormal);

            // Near-top-of-OBB vault detection: if the player's projection along
            // the OBB's Z axis is within 1.0 distance-squared of the top point,
            // apply a strong forward push ("jump_thru") to vault over the top.
            // This is the key mechanism for dismounting ladders at the top.
            bool jumpThru = false;
            const auto *body = mObjectWorld
                ? mObjectWorld->findBodyByObjID(mClimbingObjId) : nullptr;
            if (body) {
                Vector3 offset = mPosition - body->worldPos;
                Vector3 topPt = body->rotation[2] * (body->edgeLengths.z * 0.5f);
                Vector3 projPt = body->rotation[2] * glm::dot(offset, body->rotation[2]);
                if (glm::length2(topPt - projPt) < 1.0f)
                    jumpThru = true;
            }

            Vector3 jumpDir;
            if (jumpThru) {
                // Near top of OBB: vault forward with strong push + slight upward.
                // Magnitude ~2.0, scaled by CLIMB_JUMP_SCALE(5.0) = ~10 forward.
                jumpDir = forward * 2.0f + Vector3(0.0f, 0.0f, 0.1f);
                if (mClimbLog) {
                    std::fprintf(stderr, "[CLIMB] jump_thru vault: forward push\n");
                }
            } else if (facingDot > 0.0f) {
                // Facing away from surface — jump in facing direction
                // forward is already unit length, no normalize needed
                jumpDir = forward;
            } else {
                // Facing toward surface — reflect off normal, dampen by 0.5×
                jumpDir = forward - mClimbFaceNormal * (facingDot * 2.0f);
                jumpDir *= CLIMB_JUMP_REFLECT;
            }

            // Step 3: Add climb impulse (not normalized — magnitude matters)
            mVelocity += jumpDir * CLIMB_JUMP_SCALE;
            mCurrentMode = PlayerMode::Jump;

            // Cancel ground grace so we're treated as fully airborne
            mGroundGraceTimer = 0.0f;
            mGroundGraceActive = false;
        } else {
            // Non-jumping exit: if on ground, stand; otherwise fall
            if (isOnGround())
                mCurrentMode = PlayerMode::Stand;
            else
                mCurrentMode = PlayerMode::Jump;
        }

        // Clear climbing state
        mClimbingObjId = -1;
        mClimbFaceIdx = -1;
        mClimbFaceNormal = Vector3(0.0f);

        // Reset stride system
        if (mLeanDir == 0)
            activatePose(POSE_NORMAL);
        mStrideDist = 0.0f;
    }
