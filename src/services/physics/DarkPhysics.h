/******************************************************************************
 *
 *    This file is part of the Darkness engine
 *    Copyright (C) 2024-2026 Darkness contributors
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *****************************************************************************/

/******************************************************************************
 *
 *    DarkPhysics — concrete IPhysicsWorld implementation.
 *
 *    Hybrid architecture:
 *    - Player body: managed by PlayerPhysics
 *      (custom sphere-polygon collision constraint-based model)
 *    - World objects: managed by ODE rigid body dynamics (Task 26, deferred)
 *
 *    Current Phase 2 scope:
 *    - Player movement with collision, gravity, ground detection
 *    - Raycast delegation to the existing portal-graph BFS raycaster
 *    - Body management stubs for ODE integration in Task 26
 *
 *    Raycast and sweepSphere delegate to the existing RayCaster.h which
 *    uses BFS portal graph traversal — efficient for typical Thief 2
 *    missions with ~200-1500 cells.
 *
 *****************************************************************************/

#ifndef __DARKPHYSICS_H
#define __DARKPHYSICS_H

#include <memory>
#include <unordered_map>

#include <ode/ode.h>

#include "IPhysicsWorld.h"
#include "CollisionGeometry.h"
#include "ObjectCollisionGeometry.h"
#include "PlayerPhysics.h"
#include "RayCaster.h"
#include "sim/SimCommon.h"
#include "sim/ObjectPushSystem.h"
#include "property/TypedProperty.h"

namespace Darkness {

// Forward declaration
class ObjectPushSystem;

/// Concrete IPhysicsWorld implementation for the Darkness engine.
/// Owns the CollisionGeometry, PlayerPhysics, and (in Task 26) ODE world.
class DarkPhysics : public IPhysicsWorld {
public:
    /// Construct the physics world from parsed WR cell data.
    /// Takes ownership of the collision geometry built from wr.
    /// Optionally accepts a PhysicsTimestep preset (default: MODERN = 60Hz).
    /// The WRParsedData must outlive this object.
    explicit DarkPhysics(const WRParsedData &wr,
                         const PhysicsTimestep &ts = MODERN)
        : mCollision(wr)
        , mPlayer(mCollision, ts)
        , mGravity(0.0f, 0.0f, -GRAVITY)
    {
        initODE();
    }

    ~DarkPhysics() override {
        shutdownODE();
    }

    // ── World management ──

    void step(float dt) override {
        // Step player physics (custom 5-sphere constraint model)
        mPlayer.step(dt, mContactCb);

        // Sync player capsule position for ODE collision detection
        if (mPlayerGeom) {
            Vector3 pos = mPlayer.getPosition();
            dGeomSetPosition(mPlayerGeom, pos.x, pos.y, pos.z);
        }

        // Step ODE world with fixed timestep accumulator (same rate as player physics)
        if (mODEWorld) {
            float fixedDt = mPlayer.getTimestep().fixedDt;
            mODEAccum += dt;
            int steps = 0;
            while (mODEAccum >= fixedDt && steps < 10) {
                dSpaceCollide(mODESpace, this, &odeNearCallback);
                dWorldQuickStep(mODEWorld, fixedDt);
                dJointGroupEmpty(mODEContacts);
                mODEAccum -= fixedDt;
                ++steps;
            }
            if (steps >= 10) mODEAccum = 0.0f;  // prevent spiral of death
        }

        // Sync dynamic bodies back to collision geometry + renderer
        syncDynamicBodies();

        // Feed player-object contacts to push system for kinematic pushing
        if (mPushSystem) {
            mPushSystem->processPlayerContacts(
                mPlayer.getLastContacts(), mPlayer.getVelocity());
        }

        // Update view punch spring
        mPlayer.updateViewPunch(dt);
    }

    void addBody(EntityID id, const PhysicsBodyDesc &desc) override {
        // TODO (Task 26): Create ODE body + geom from desc
        // For now, store the descriptor for later ODE integration
        mBodyDescs[id] = desc;
    }

    void removeBody(EntityID id) override {
        // TODO (Task 26): Destroy ODE body + geom
        mBodyDescs.erase(id);
    }

    void setGravity(const Vector3 &g) override {
        mGravity = g;
        mPlayer.setGravityMagnitude(std::fabs(g.z));
        if (mODEWorld)
            dWorldSetGravity(mODEWorld, g.x, g.y, g.z);
    }

    // ── Force / impulse ──

    void applyForce(EntityID id, const Vector3 &force) override {
        // TODO (Task 26): dBodyAddForce on ODE body
        (void)id; (void)force;
    }

    void applyImpulse(EntityID id, const Vector3 &impulse) override {
        // TODO (Task 26): velocity change via dBodySetLinearVel
        (void)id; (void)impulse;
    }

    // ── Spatial queries ──

    bool raycast(const Vector3 &from, const Vector3 &to,
                 RayHit *hit) const override {
        if (!hit) {
            // Just test for any hit
            RayHit tmp;
            return raycastWorld(mCollision.getWR(), from, to, tmp);
        }
        return raycastWorld(mCollision.getWR(), from, to, *hit);
    }

    bool sweepSphere(const Vector3 &from, const Vector3 &to,
                     float radius, RayHit *hit) const override {
        // Phase 2 approximation: offset the ray by radius along the
        // direction and do a standard raycast. This is conservative
        // (may miss narrow gaps) but fast. Full sphere sweep deferred.
        Vector3 dir = to - from;
        float len = glm::length(dir);
        if (len < 1e-6f)
            return false;
        dir /= len;

        // Shorten the ray by radius at both ends for conservative estimate
        Vector3 offsetFrom = from + dir * radius;
        Vector3 offsetTo   = to   - dir * radius;
        if (glm::dot(offsetTo - offsetFrom, dir) <= 0.0f)
            return false; // ray too short for sphere

        RayHit tmp;
        if (raycastWorld(mCollision.getWR(), offsetFrom, offsetTo, tmp)) {
            if (hit) {
                *hit = tmp;
                // Adjust distance back to account for offset
                hit->distance += radius;
            }
            return true;
        }
        return false;
    }

    std::vector<EntityID> overlapSphere(const Vector3 &center,
                                        float radius) const override {
        // TODO (Task 26): Query ODE space or SpatialIndex
        (void)center; (void)radius;
        return {};
    }

    // ── Body state ──

    Vector3 getPosition(EntityID id) const override {
        // TODO (Task 26): dBodyGetPosition for ODE bodies
        auto it = mBodyDescs.find(id);
        if (it != mBodyDescs.end())
            return it->second.position;
        return Vector3(0.0f);
    }

    Vector3 getVelocity(EntityID id) const override {
        // TODO (Task 26): dBodyGetLinearVel for ODE bodies
        (void)id;
        return Vector3(0.0f);
    }

    void setPosition(EntityID id, const Vector3 &pos) override {
        // TODO (Task 26): dBodySetPosition for ODE bodies
        auto it = mBodyDescs.find(id);
        if (it != mBodyDescs.end())
            it->second.position = pos;
    }

    void setVelocity(EntityID id, const Vector3 &vel) override {
        // TODO (Task 26): dBodySetLinearVel for ODE bodies
        (void)id; (void)vel;
    }

    // ── Collision events ──

    void setContactCallback(ContactCallback cb) override {
        mContactCb = std::move(cb);
    }

    // ── Player-specific ──

    Vector3 getPlayerPosition() const override {
        return mPlayer.getPosition();
    }

    Vector3 getPlayerEyePosition() const override {
        return mPlayer.getEyePosition();
    }

    void setPlayerPosition(const Vector3 &pos) override {
        mPlayer.setPosition(pos);
    }

    void setPlayerMovement(float forward, float right) override {
        mPlayer.setMovement(forward, right);
    }

    void setPlayerYaw(float yaw) override {
        mPlayer.setYaw(yaw);
    }

    void playerJump() override {
        mPlayer.jump();
    }

    void setPlayerCrouching(bool crouching) override {
        mPlayer.setCrouching(crouching);
    }

    void setPlayerSneaking(bool sneaking) override {
        mPlayer.setSneaking(sneaking);
    }

    void setPlayerRunning(bool running) override {
        mPlayer.setRunning(running);
    }

    bool isPlayerOnGround() const override {
        return mPlayer.isOnGround();
    }

    int32_t getPlayerCell() const override {
        return mPlayer.getCell();
    }

    Vector3 getPlayerVelocity() const override {
        return mPlayer.getVelocity();
    }

    void setFootstepCallback(FootstepCallback cb) override {
        mFootstepCb = std::move(cb);
        // Forward to PlayerPhysics where stride events fire
        mPlayer.setFootstepCallback(mFootstepCb);
    }

    void setLandingCallback(LandingCallback cb) override {
        mLandingCb = std::move(cb);
        mPlayer.setLandingCallback(mLandingCb);
    }

    int getPlayerMode() const override {
        return static_cast<int>(mPlayer.getMode());
    }

    bool isPlayerMantling() const override {
        return mPlayer.isMantling();
    }

    bool isPlayerClimbing() const override {
        return mPlayer.isClimbing();
    }

    void disablePlayerMotion() override {
        mPlayer.disableMotion();
    }

    void enablePlayerMotion() override {
        mPlayer.enableMotion();
    }

    /// Set per-texture friction lookup table (indexed by TXLIST texture index).
    /// Built at mission load from P$Friction on texture archetypes in dark.gam.
    void setFrictionTable(const std::vector<float> &table) {
        mPlayer.setFrictionTable(table);
    }

    /// Set per-texture climbability lookup table (indexed by TXLIST texture index).
    /// Built at mission load from P$Climbabil on texture archetypes in dark.gam.
    /// Non-zero values boost friction on steep surfaces (not a climb-mode trigger).
    void setClimbabilityTable(const std::vector<float> &table) {
        mPlayer.setClimbabilityTable(table);
    }

    /// Apply player physics configuration from P$PhysAttr and P$PhysDims properties.
    /// This overrides the default hardcoded constants with per-object values from
    /// the archetype inheritance chain. Called once during level load.
    void applyPlayerConfig(const PlayerPhysicsConfig &cfg) {
        mPlayer.applyConfig(cfg);
        // Keep mGravity vector consistent with the new gravity magnitude
        mGravity = Vector3(0.0f, 0.0f, -mPlayer.getGravityMagnitude());
    }

    // ── Object collision (Task 26) ──

    /// Build collision bodies for placed world objects (crates, furniture, doors).
    /// Called once during level load after object assets are parsed.
    /// Creates an ObjectCollisionWorld from .bin bounding boxes and P$PhysType
    /// properties, then wires the collision callback into PlayerPhysics.
    ///
    /// ODE UPGRADE: When ODE rigid bodies are added, this method would also
    /// create dGeomIDs for each static object (dCreateBox/dCreateSphere) in
    /// the dSpaceID, and dynamic objects would additionally get dBodyIDs with
    /// dMassSetBox/dMassSetSphere. The custom player collision callback would
    /// remain (proven correct), while ODE dNearCallback handles object-vs-object.
    void buildObjectCollision(
        const std::vector<ObjectPlacement> &objects,
        const std::unordered_map<std::string, ParsedBinMesh> &models,
        PropertyService *propSvc)
    {
        mObjectCollision = std::make_unique<ObjectCollisionWorld>();
        mObjectCollision->build(objects, models, propSvc, mCollision.getWR());

        size_t bodyCount = mObjectCollision->bodyCount();
        if (bodyCount == 0) {
            std::fprintf(stderr, "Physics: no object collision bodies created\n");
            return;
        }

        std::fprintf(stderr, "Physics: %zu object collision bodies created (%zu cells)\n",
                     bodyCount, mObjectCollision->cellCount());

        // Wire the object collision callback into PlayerPhysics.
        // The callback captures mObjectCollision by raw pointer (safe because
        // DarkPhysics owns both the ObjectCollisionWorld and the PlayerPhysics,
        // and their lifetimes are identical).
        //
        // ODE UPGRADE: This callback wiring would remain for player-vs-static-object
        // collision. For dynamic objects, ODE dNearCallback would handle them
        // separately, with the player registered as a kinematic dGeomID.
        ObjectCollisionWorld *ocw = mObjectCollision.get();
        mPlayer.setObjectCollisionCallback(
            [ocw](const Vector3 *centers, const float *radii,
                  int numSpheres, int32_t playerCell,
                  std::vector<SphereContact> &outContacts) {
                ocw->testPlayerSpheres(centers, radii, numSpheres,
                                       playerCell, outContacts);
                // Player push of dynamic objects is deferred to Phase 6
                // (script system) which will call makeDynamic() on specific
                // objects and wire push via the velocity transfer algorithm.
            });
        // Set direct pointer for object stair stepping (ray-vs-OBB lookups).
        mPlayer.setObjectCollisionWorld(ocw);
    }

    /// Number of active object collision bodies (for diagnostics).
    size_t objectCollisionBodyCount() const {
        return mObjectCollision ? mObjectCollision->bodyCount() : 0;
    }

    /// Set the object push system for kinematic pushing (Task 61).
    /// Called during main loop initialization, after ObjectPushSystem::init().
    void setPushSystem(ObjectPushSystem *pushSys) { mPushSystem = pushSys; }

    // ── Direct access for renderer integration ──

    /// Get the physics update rate in Hz (convenience delegate to PlayerPhysics).
    float getPhysicsHz() const { return mPlayer.getPhysicsHz(); }

    /// Access the collision geometry (for renderer camera collision fallback)
    const CollisionGeometry &getCollisionGeometry() const { return mCollision; }

    /// Access the object collision world (for frob ray-vs-OBB, debug display).
    /// Returns nullptr if no object collision has been built.
    ObjectCollisionWorld *getObjectCollisionWorld() {
        return mObjectCollision.get();
    }

    /// Access the player physics (for debug display, etc.)
    const PlayerPhysics &getPlayerPhysics() const { return mPlayer; }
    PlayerPhysics &getPlayerPhysics() { return mPlayer; }

private:
    // ── ODE initialization and shutdown ──

    void initODE() {
        static bool sODEInitialized = false;
        if (!sODEInitialized) {
            dInitODE2(0);
            sODEInitialized = true;
        }

        mODEWorld = dWorldCreate();
        mODESpace = dHashSpaceCreate(nullptr);
        mODEContacts = dJointGroupCreate(0);

        // World parameters — Z-up, matching Dark Engine gravity
        dWorldSetGravity(mODEWorld, 0.0, 0.0, -GRAVITY);

        // Solver parameters for stable stacking
        dWorldSetERP(mODEWorld, 0.8);                     // stiff error correction
        dWorldSetCFM(mODEWorld, 1e-4);                     // slight softness for stability
        dWorldSetContactSurfaceLayer(mODEWorld, 0.01);     // 1cm penetration tolerance
        dWorldSetQuickStepNumIterations(mODEWorld, 20);    // solver iterations

        // Auto-sleep for stable resting contacts (critical for stacked crates).
        // Higher thresholds than typical ODE defaults so objects settle quickly.
        dWorldSetAutoDisableFlag(mODEWorld, 1);
        dWorldSetAutoDisableLinearThreshold(mODEWorld, 0.05);
        dWorldSetAutoDisableAngularThreshold(mODEWorld, 0.05);
        dWorldSetAutoDisableSteps(mODEWorld, 5);
        dWorldSetAutoDisableTime(mODEWorld, 0.5);  // sleep after 0.5s of low activity

        // Aggressive damping so objects settle quickly on trimesh surfaces.
        // Without this, objects micro-bounce on trimesh contact points
        // and never reach the auto-disable velocity threshold.
        dWorldSetLinearDamping(mODEWorld, 0.05);
        dWorldSetAngularDamping(mODEWorld, 0.2);
        dWorldSetLinearDampingThreshold(mODEWorld, 0.01);
        dWorldSetAngularDampingThreshold(mODEWorld, 0.01);

        std::fprintf(stderr, "ODE world created (gravity=%.1f, ERP=0.8, CFM=1e-4, "
                     "angularDamping=0.05)\n", -GRAVITY);
    }

    void shutdownODE() {
        // Destroy world trimesh
        if (mWorldTrimesh) { dGeomDestroy(mWorldTrimesh); mWorldTrimesh = nullptr; }
        if (mWorldTrimeshData) { dGeomTriMeshDataDestroy(mWorldTrimeshData); mWorldTrimeshData = nullptr; }

        // Destroy player geom
        if (mPlayerGeom) { dGeomDestroy(mPlayerGeom); mPlayerGeom = nullptr; }

        // Destroy all dynamic bodies (must happen before geom destruction)
        for (auto &[id, body] : mODEBodies)
            dBodyDestroy(body);
        mODEBodies.clear();

        // Destroy all object geoms
        for (auto &[id, geom] : mODEGeoms)
            dGeomDestroy(geom);
        mODEGeoms.clear();

        // Free per-geom data
        for (auto *gd : mGeomDataPtrs)
            delete gd;
        mGeomDataPtrs.clear();
        mODEScales.clear();

        // Destroy ODE world
        if (mODEContacts) { dJointGroupDestroy(mODEContacts); mODEContacts = nullptr; }
        if (mODESpace) { dSpaceDestroy(mODESpace); mODESpace = nullptr; }
        if (mODEWorld) { dWorldDestroy(mODEWorld); mODEWorld = nullptr; }
        dCloseODE();
    }

    // ── ODE collision callback ──

    /// Maximum contact points per geom pair. 4 is sufficient for stable
    /// box-on-box stacking; more contacts = more solver work.
    static constexpr int MAX_ODE_CONTACTS = 4;

    // Player push uses the original engine's velocity transfer algorithm:
    // mass-weighted average along contact normal, directly setting the
    // object's velocity. No ODE forces — much more stable than impulses.

    /// Sentinel value stored in player geom's dGeomSetData
    static constexpr intptr_t PLAYER_GEOM_TAG = -999;

    /// Per-geom data stored via dGeomSetData for nearCallback material lookup.
    struct ODEGeomData {
        int32_t objID;
        float friction;
        float elasticity;
    };

    /// ODE near callback — called for each potentially colliding geom pair.
    /// Creates temporary contact joints for overlapping bodies.
    /// Detects player-vs-dynamic contacts and routes impulse to PlayerPhysics.
    static void odeNearCallback(void *data, dGeomID o1, dGeomID o2) {
        auto *self = static_cast<DarkPhysics *>(data);

        // Skip space-space collisions (not used with flat hash space)
        if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) return;

        // Check if either geom is the player via ODEGeomData
        auto *gd1 = static_cast<ODEGeomData *>(dGeomGetData(o1));
        auto *gd2 = static_cast<ODEGeomData *>(dGeomGetData(o2));
        bool isPlayer1 = (gd1 && gd1->objID == static_cast<int32_t>(PLAYER_GEOM_TAG));
        bool isPlayer2 = (gd2 && gd2->objID == static_cast<int32_t>(PLAYER_GEOM_TAG));

        // Player-vs-static: skip (handled by custom PlayerPhysics collision)
        if (isPlayer1 || isPlayer2) {
            dGeomID otherGeom = isPlayer1 ? o2 : o1;
            dBodyID dynBody = dGeomGetBody(otherGeom);
            if (!dynBody) return;  // static geom — custom system handles it

            // Player-vs-dynamic: extract hit reaction (Task 61 will add dynamic bodies)
            self->handlePlayerDynamicContact(otherGeom, dynBody);
            return;
        }

        // Normal object-vs-object collision
        dBodyID b1 = dGeomGetBody(o1);
        dBodyID b2 = dGeomGetBody(o2);

        // Skip if both are static (no dynamics to solve)
        if (!b1 && !b2) return;

        // Read per-object material properties from ODEGeomData
        float mu = 0.8f, bounce = 0.1f;
        auto *d1 = static_cast<ODEGeomData *>(dGeomGetData(o1));
        auto *d2 = static_cast<ODEGeomData *>(dGeomGetData(o2));
        if (d1 && d2 && d1->objID != PLAYER_GEOM_TAG && d2->objID != PLAYER_GEOM_TAG) {
            // Average friction, max elasticity (bouncier surface wins)
            mu = (d1->friction + d2->friction) * 0.5f;
            bounce = std::max(d1->elasticity, d2->elasticity);
        }

        // Generate contact points
        dContact contacts[MAX_ODE_CONTACTS];
        int numContacts = dCollide(o1, o2, MAX_ODE_CONTACTS,
                                    &contacts[0].geom, sizeof(dContact));

        for (int i = 0; i < numContacts; ++i) {
            // Stiff contacts with minimal bounce for stable resting.
            // SoftERP/CFM avoided — they cause objects to sink into trimesh.
            contacts[i].surface.mode = dContactBounce | dContactApprox1;
            contacts[i].surface.mu = mu;
            contacts[i].surface.bounce = std::min(bounce, 0.2f);
            contacts[i].surface.bounce_vel = 1.0f;  // need real speed to bounce

            dJointID joint = dJointCreateContact(
                self->mODEWorld, self->mODEContacts, &contacts[i]);
            dJointAttach(joint, b1, b2);
        }
    }

    /// Handle player-vs-dynamic-object contact: apply knockback impulse
    /// and direction-aware view punch to PlayerPhysics.
    void handlePlayerDynamicContact(dGeomID otherGeom, dBodyID dynBody) {
        const dReal *vel = dBodyGetLinearVel(dynBody);
        Vector3 objVel(static_cast<float>(vel[0]),
                       static_cast<float>(vel[1]),
                       static_cast<float>(vel[2]));
        float speed = glm::length(objVel);
        if (speed < 1.0f) return;  // below threshold, no hit reaction

        // Knockback impulse scaled by object mass and speed
        dMass mass;
        dBodyGetMass(dynBody, &mass);
        float impulseScale = static_cast<float>(mass.mass) * speed * 0.01f;
        Vector3 knockDir = glm::normalize(objVel);

        // Apply velocity impulse to player
        mPlayer.applyImpulse(knockDir * impulseScale);

        // Direction-aware view punch (Source Engine style)
        // Decompose hit direction into pitch (forward) and roll (side)
        Vector3 forward = mPlayer.getForward();
        Vector3 right = mPlayer.getRight();
        float sideDot = glm::dot(knockDir, right);
        float fwdDot = glm::dot(knockDir, forward);
        float punchMag = impulseScale * 20.0f;
        mPlayer.addViewPunch(Vector3(
            fwdDot * punchMag,     // pitch
            0.0f,                   // yaw (minimal)
            sideDot * punchMag      // roll
        ));
    }

    // ── ODE static geometry creation ──

    /// Convert GLM column-major mat3 to ODE row-major dMatrix3 (3×4 array).
    static void glmMat3ToODE(const glm::mat3 &m, dMatrix3 R) {
        R[0] = m[0][0]; R[1] = m[1][0]; R[2]  = m[2][0]; R[3]  = 0;
        R[4] = m[0][1]; R[5] = m[1][1]; R[6]  = m[2][1]; R[7]  = 0;
        R[8] = m[0][2]; R[9] = m[1][2]; R[10] = m[2][2]; R[11] = 0;
    }

    /// Convert ODE position + rotation to GLM mat4 (with scale).
    static Matrix4 odeToGlmMatrix(const dReal *pos, const dReal *R,
                                   const Vector3 &scale) {
        // ODE dMatrix3 is row-major 3x4: R[row*4+col]
        // GLM mat4 is column-major: m[col][row]
        Matrix4 m(1.0f);
        m[0][0] = static_cast<float>(R[0])  * scale.x;
        m[1][0] = static_cast<float>(R[1])  * scale.y;
        m[2][0] = static_cast<float>(R[2])  * scale.z;
        m[0][1] = static_cast<float>(R[4])  * scale.x;
        m[1][1] = static_cast<float>(R[5])  * scale.y;
        m[2][1] = static_cast<float>(R[6])  * scale.z;
        m[0][2] = static_cast<float>(R[8])  * scale.x;
        m[1][2] = static_cast<float>(R[9])  * scale.y;
        m[2][2] = static_cast<float>(R[10]) * scale.z;
        m[3][0] = static_cast<float>(pos[0]);
        m[3][1] = static_cast<float>(pos[1]);
        m[3][2] = static_cast<float>(pos[2]);
        return m;
    }

    /// Sync awake dynamic ODE bodies back to collision geometry and renderer.
    void syncDynamicBodies() {
        if (!mObjectCollision || !mObjectStates) return;
        ++mODEFrameCount;
        int awakeCount = 0;
        for (auto &[objID, body] : mODEBodies) {
            if (!dBodyIsEnabled(body)) continue;  // sleeping — no update needed
            ++awakeCount;

            const dReal *pos = dBodyGetPosition(body);
            const dReal *vel = dBodyGetLinearVel(body);
            const dReal *R = dBodyGetRotation(body);

            // Log awake bodies every 30 frames (~0.5s at 60Hz)
            if (mODEFrameCount % 30 == 0) {
                dMass mass;
                dBodyGetMass(body, &mass);
                std::fprintf(stderr, "  [ODE] obj=%d mass=%.1f pos=(%.2f,%.2f,%.2f) "
                             "vel=(%.2f,%.2f,%.2f)\n",
                             objID, mass.mass,
                             pos[0], pos[1], pos[2],
                             vel[0], vel[1], vel[2]);
            }

            // Look up scale from ObjectCollisionBody (scale doesn't change at runtime)
            Vector3 scale(1.0f);
            auto scaleIt = mODEScales.find(objID);
            if (scaleIt != mODEScales.end()) scale = scaleIt->second;

            Matrix4 modelMatrix = odeToGlmMatrix(pos, R, scale);
            Vector3 position(static_cast<float>(pos[0]),
                             static_cast<float>(pos[1]),
                             static_cast<float>(pos[2]));

            // Sync collision geometry (AABB, rotation, cell registration)
            mObjectCollision->updateBodyTransform(objID, modelMatrix);

            // Sync renderer (ObjectState)
            applyModelMatrix(*mObjectStates, objID, modelMatrix, position, scale);
        }
        if (awakeCount > 0 && mODEFrameCount % 30 == 0) {
            std::fprintf(stderr, "  [ODE] %d bodies awake\n", awakeCount);
        }
    }

public:
    /// Set the ObjectStateMap for dynamic body renderer sync.
    /// Called by darknessRender after world state is constructed.
    void setObjectStates(ObjectStateMap *states) { mObjectStates = states; }

    /// Build ODE trimesh from WR cell solid polygons so dynamic objects
    /// collide with floors, walls, and ceilings instead of falling through.
    void buildWorldTrimesh() {
        if (!mODESpace) return;
        const auto &wr = mCollision.getWR();

        // Collect all solid polygon triangles from all cells.
        // Each polygon is a fan: vertex[0], vertex[1..n-1] form triangles.
        std::vector<float> verts;
        std::vector<dTriIndex> indices;

        for (uint32_t ci = 0; ci < wr.numCells; ++ci) {
            const auto &cell = wr.cells[ci];
            int numSolid = cell.numPolygons - cell.numPortals;

            for (int pi = 0; pi < numSolid; ++pi) {
                const auto &poly = cell.polygons[pi];
                if (poly.count < 3) continue;

                const auto &idx = cell.polyIndices[pi];
                // Fan triangulation: v0, v1, v2 ... vN → (v0,v1,v2), (v0,v2,v3), ...
                uint32_t baseVertIdx = static_cast<uint32_t>(verts.size() / 3);

                // Add all polygon vertices
                for (int vi = 0; vi < poly.count; ++vi) {
                    const Vector3 &v = cell.vertices[idx[vi]];
                    verts.push_back(v.x);
                    verts.push_back(v.y);
                    verts.push_back(v.z);
                }

                // Fan triangulation — reversed winding for ODE.
                // WR cell polygons face inward (into the cell). ODE trimeshes
                // are single-sided and need outward-facing normals (objects
                // collide from the normal side). Reversing the winding makes
                // objects inside cells collide with floors/walls correctly.
                for (int ti = 1; ti < poly.count - 1; ++ti) {
                    indices.push_back(static_cast<dTriIndex>(baseVertIdx));
                    indices.push_back(static_cast<dTriIndex>(baseVertIdx + ti + 1));
                    indices.push_back(static_cast<dTriIndex>(baseVertIdx + ti));
                }
            }
        }

        if (indices.empty()) return;

        // Build ODE trimesh data — ODE takes ownership of the pointers,
        // but we must keep the arrays alive for the lifetime of the trimesh.
        mWorldVerts = std::move(verts);
        mWorldIndices = std::move(indices);

        mWorldTrimeshData = dGeomTriMeshDataCreate();
        dGeomTriMeshDataBuildSingle(mWorldTrimeshData,
            mWorldVerts.data(), 3 * sizeof(float),
            static_cast<int>(mWorldVerts.size() / 3),
            mWorldIndices.data(), static_cast<int>(mWorldIndices.size()),
            3 * sizeof(dTriIndex));

        mWorldTrimesh = dCreateTriMesh(mODESpace, mWorldTrimeshData,
                                        nullptr, nullptr, nullptr);

        // Static trimesh — no body, at world origin
        dGeomSetPosition(mWorldTrimesh, 0, 0, 0);

        int numTris = static_cast<int>(mWorldIndices.size() / 3);
        int numVerts = static_cast<int>(mWorldVerts.size() / 3);
        std::fprintf(stderr, "ODE world trimesh: %d triangles, %d vertices\n",
                     numTris, numVerts);
    }

    /// Create ODE geoms for all collision bodies. Static objects get geoms
    /// only; dynamic objects (P$PhysAttr.mass > 0) get dBodyID + dGeomID.
    /// isKinematic callback excludes objects managed by other systems
    /// (DoorSystem, MovingTerrainSystem, PressurePlateSystem).
    void buildODEGeoms(PropertyService *propSvc,
                       std::function<bool(int32_t)> isKinematic = nullptr) {
        if (!mObjectCollision || !mODESpace) return;

        int staticCount = 0, dynamicCount = 0;
        for (size_t i = 0; i < mObjectCollision->bodyCount(); ++i) {
            const auto &body = mObjectCollision->getBody(i);
            if (body.isEdgeTrigger) continue;
            if (body.shapeType == CollisionShapeType::None) continue;

            // Create geom for the shape
            dGeomID geom = nullptr;
            if (body.shapeType == CollisionShapeType::OBB) {
                geom = dCreateBox(mODESpace,
                    body.edgeLengths.x, body.edgeLengths.y, body.edgeLengths.z);
            } else {
                geom = dCreateSphere(mODESpace, body.sphereRadius);
            }

            dGeomSetPosition(geom, body.worldPos.x, body.worldPos.y, body.worldPos.z);
            dMatrix3 R;
            glmMat3ToODE(body.rotation, R);
            dGeomSetRotation(geom, R);

            // All objects start as STATIC ODE geoms. Dynamic simulation is
            // activated per-object at runtime (via scripts, explosions, frob
            // pickup/throw, etc.) by calling makeDynamic(objID). In the
            // original engine, most objects with P$PhysAttr are NOT actively
            // simulated — they just have collision shapes. Only specific
            // objects get physics models registered at runtime.
            //
            // We read and cache P$PhysAttr for future dynamic activation
            // but don't create dBodyIDs at load time.
            bool isDynamic = false;
            PropPhysAttr attr{};
            if (propSvc) {
                if (getTypedProperty<PropPhysAttr>(propSvc, "PhysAttr", body.objID, attr)) {
                    // Cache material properties for future use
                }
            }

            // Allocate per-geom data (lives for duration of ODE world)
            auto *gd = new ODEGeomData{
                body.objID,
                isDynamic ? attr.friction : 0.8f,
                isDynamic ? attr.elasticity : 0.1f
            };
            dGeomSetData(geom, gd);
            mGeomDataPtrs.push_back(gd);  // track for cleanup

            if (isDynamic) {
                // Create rigid body
                dBodyID odeBody = dBodyCreate(mODEWorld);

                // Set mass from P$PhysAttr
                dMass odeMass;
                if (body.shapeType == CollisionShapeType::OBB) {
                    dMassSetBox(&odeMass, 1.0,
                        body.edgeLengths.x, body.edgeLengths.y, body.edgeLengths.z);
                } else {
                    dMassSetSphere(&odeMass, 1.0, body.sphereRadius);
                }
                dMassAdjust(&odeMass, attr.mass);
                dBodySetMass(odeBody, &odeMass);

                // Position and rotation
                dBodySetPosition(odeBody, body.worldPos.x, body.worldPos.y, body.worldPos.z);
                dBodySetRotation(odeBody, R);

                // Attach geom to body
                dGeomSetBody(geom, odeBody);

                // Start all dynamic bodies as sleeping. Without WR floor
                // geometry in ODE, awake bodies would fall through the world.
                // Bodies wake when disturbed (player push, object collision).
                dBodyDisable(odeBody);

                mODEScales[body.objID] = body.objectScale;
                mODEBodies[body.objID] = odeBody;
                ++dynamicCount;
            } else {
                ++staticCount;
            }

            mODEGeoms[body.objID] = geom;
        }

        // Create player kinematic capsule
        {
            float radius = 1.2f;
            float length = 6.0f - 2.0f * radius;
            mPlayerGeom = dCreateCapsule(mODESpace, radius, length);
            auto *pgd = new ODEGeomData{static_cast<int32_t>(PLAYER_GEOM_TAG), 0.5f, 0.0f};
            dGeomSetData(mPlayerGeom, pgd);
            mGeomDataPtrs.push_back(pgd);

            Vector3 pos = mPlayer.getPosition();
            dGeomSetPosition(mPlayerGeom, pos.x, pos.y, pos.z);
        }

        std::fprintf(stderr, "ODE geoms: %d static, %d dynamic, 1 player capsule\n",
                     staticCount, dynamicCount);
    }

    /// Activate an object as a dynamic ODE body at runtime. Called by
    /// scripts (Phase 6), explosions, or frob pickup/throw to make a
    /// previously static object respond to physics forces.
    /// Returns true if the body was created, false if already dynamic
    /// or no geom exists for this object.
    bool makeDynamic(int32_t objID, float mass, float friction = 0.5f,
                     float elasticity = 0.1f) {
        if (mODEBodies.find(objID) != mODEBodies.end()) return false;
        auto geomIt = mODEGeoms.find(objID);
        if (geomIt == mODEGeoms.end()) return false;

        dGeomID geom = geomIt->second;
        const ObjectCollisionBody *collBody =
            mObjectCollision ? mObjectCollision->findBodyByObjID(objID) : nullptr;
        if (!collBody) return false;

        dBodyID odeBody = dBodyCreate(mODEWorld);
        dMass odeMass;
        if (collBody->shapeType == CollisionShapeType::OBB) {
            dMassSetBox(&odeMass, 1.0,
                collBody->edgeLengths.x, collBody->edgeLengths.y,
                collBody->edgeLengths.z);
        } else {
            dMassSetSphere(&odeMass, 1.0, collBody->sphereRadius);
        }
        dMassAdjust(&odeMass, mass);
        dBodySetMass(odeBody, &odeMass);

        const dReal *pos = dGeomGetPosition(geom);
        const dReal *R = dGeomGetRotation(geom);
        dBodySetPosition(odeBody, pos[0], pos[1], pos[2]);
        dBodySetRotation(odeBody, R);
        dGeomSetBody(geom, odeBody);

        // Update per-geom material data
        auto *gd = static_cast<ODEGeomData *>(dGeomGetData(geom));
        if (gd) { gd->friction = friction; gd->elasticity = elasticity; }

        mODEBodies[objID] = odeBody;
        mODEScales[objID] = collBody->objectScale;
        return true;
    }

    /// Check if an object has a dynamic ODE body (for player push logic).
    bool hasDynamicBody(int32_t objID) const {
        return mODEBodies.find(objID) != mODEBodies.end();
    }

    /// Get the mass of a dynamic ODE body (0 if not found).
    float getDynamicBodyMass(int32_t objID) const {
        auto it = mODEBodies.find(objID);
        if (it == mODEBodies.end()) return 0.0f;
        dMass mass;
        dBodyGetMass(it->second, &mass);
        return static_cast<float>(mass.mass);
    }

    /// Wake and push a dynamic object (called when player collides with it).
    void pushDynamicObject(int32_t objID, const Vector3 &force) {
        auto it = mODEBodies.find(objID);
        if (it == mODEBodies.end()) return;
        dBodyEnable(it->second);
        dBodyAddForce(it->second, force.x, force.y, force.z);
    }

private:
    // ── Data members ──

    CollisionGeometry mCollision;    // world collision geometry from WR cells

    // Object collision world — built from .bin bounding boxes and P$PhysType.
    // Used for ALL player-vs-object collision (static and dynamic).
    // Dynamic bodies sync ODE rotation back via updateBodyTransform() each frame.
    std::unique_ptr<ObjectCollisionWorld> mObjectCollision;

    ObjectPushSystem *mPushSystem = nullptr;  // kinematic object pushing (Task 61)

    PlayerPhysics mPlayer;           // custom player movement simulation

    Vector3 mGravity;                // world gravity vector
    ContactCallback mContactCb;      // optional contact notification callback
    FootstepCallback mFootstepCb;    // footstep sound callback
    LandingCallback mLandingCb;      // landing impact sound callback

    // Stored body descriptors for deferred dynamic body creation (Task 61)
    std::unordered_map<EntityID, PhysicsBodyDesc> mBodyDescs;

    // ── ODE world state ──
    dWorldID       mODEWorld    = nullptr;
    dSpaceID       mODESpace    = nullptr;
    dJointGroupID  mODEContacts = nullptr;
    std::unordered_map<int32_t, dGeomID>  mODEGeoms;   // objID → geom (static or dynamic)
    std::unordered_map<int32_t, dBodyID>  mODEBodies;  // objID → dynamic body
    std::unordered_map<int32_t, Vector3>  mODEScales;  // objID → object scale for matrix
    std::vector<ODEGeomData *>            mGeomDataPtrs; // owned per-geom data for cleanup
    dGeomID        mPlayerGeom  = nullptr;             // kinematic player capsule
    float          mODEAccum    = 0.0f;               // fixed-timestep accumulator for ODE
    uint32_t       mODEFrameCount = 0;                // frame counter for throttled logging
    ObjectStateMap *mObjectStates = nullptr;           // renderer transform sync (not owned)

    // World geometry trimesh — solid WR cell polygons for object-vs-world collision
    dTriMeshDataID mWorldTrimeshData = nullptr;
    dGeomID        mWorldTrimesh     = nullptr;
    std::vector<float>     mWorldVerts;    // vertex data (owned, ODE references it)
    std::vector<dTriIndex> mWorldIndices;  // index data (owned, ODE references it)
};

} // namespace Darkness

#endif // __DARKPHYSICS_H
