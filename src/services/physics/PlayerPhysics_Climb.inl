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

            // Zero velocity on climb entry — start fresh from wall
            mVelocity = Vector3(0.0f);

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

        // Update face normal from contacts if player moved to a different face
        for (const auto &c : mLastContacts) {
            if (c.objectId != mClimbingObjId || c.cellIdx != -1)
                continue;

            int faceIdx = c.polyIdx & 0xF;
            if (faceIdx < 0 || faceIdx > 5)
                continue;

            if ((body->climbableSides & (1 << faceIdx)) == 0)
                continue;

            if (faceIdx != mClimbFaceIdx) {
                mClimbFaceNormal = getOBBFaceNormal(*body, faceIdx);
                mClimbFaceIdx = faceIdx;
                if (mClimbLog) {
                    std::fprintf(stderr, "[CLIMB] Face change: face=%d "
                        "normal=(%.2f,%.2f,%.2f)\n",
                        faceIdx, mClimbFaceNormal.x, mClimbFaceNormal.y,
                        mClimbFaceNormal.z);
                }
            }
            break;
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
            // the climb jump-off impulse on top. This gives strong horizontal
            // momentum when jumping off ladders.

            // Step 1: Normal jump — strip vertical, boost horizontal, add upward
            mVelocity.z = 0.0f;                    // remove vertical component
            mVelocity *= 1.05f;                     // 5% horizontal boost
            mVelocity.z += JUMP_IMPULSE;            // 14.0 upward

            // Step 2: Compute climb jump-off direction from facing vs surface
            Vector3 forward(mCosYaw, mSinYaw, 0.0f);
            float facingDot = glm::dot(forward, mClimbFaceNormal);

            Vector3 jumpDir;
            if (facingDot > 0.0f) {
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
