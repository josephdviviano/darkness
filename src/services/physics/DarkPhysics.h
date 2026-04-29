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
        // Capture pre-collision velocity for push system — after collision
        // resolution, the velocity component into walls/objects is zeroed out.
        Vector3 preCollisionVel = mPlayer.getVelocity();

        // Step player physics (custom 5-sphere constraint model)
        mPlayer.step(dt, mContactCb);

        // Sync player capsule position for ODE collision detection
        if (mPlayerGeom) {
            Vector3 pos = mPlayer.getPosition();
            dGeomSetPosition(mPlayerGeom, pos.x, pos.y, pos.z);
        }

        // Apply push forces to dynamic ODE bodies from player contacts BEFORE
        // the ODE step so forces take effect this frame (not next frame).
        // Uses the pre-collision velocity (post-collision has wall component removed).
        if (mPushSystem) {
            const auto &contacts = mPlayer.getContacts();
            for (const auto &c : contacts) {
                if (c.objectId < 0) continue;  // terrain contact
                if (!mPushSystem->isPushable(c.objectId)) continue;

                float vDotN = glm::dot(preCollisionVel, c.normal);
                if (vDotN >= 0.0f) continue;  // moving away

                auto bodyIt = mODEBodies.find(c.objectId);
                if (bodyIt != mODEBodies.end()) {
                    // ODE dynamic body — apply impulse as force over one physics step.
                    // Dark Engine elastic transfer: 2*m1/(m1+m2) * dampening
                    float objMass = mPushSystem->getMass(c.objectId);
                    float elastic = (2.0f * PLAYER_MASS) / (PLAYER_MASS + objMass);
                    constexpr float kPushDampening = 0.06f;
                    // Contact normal points from object toward player; vDotN is negative
                    // (into object), so normal * vDotN gives push direction away from player.
                    Vector3 pushVel = c.normal * vDotN * elastic * kPushDampening;
                    // Convert velocity impulse to force: F = m * dv / dt
                    float fixedDt = mPlayer.getTimestep().fixedDt;
                    Vector3 force = pushVel * (objMass / fixedDt);
                    // Player pushes are strictly horizontal — suppress ALL vertical
                    // force (both up and down). Downward force tips objects over;
                    // upward force lifts them. Gravity handles vertical motion.
                    force.z = 0.0f;

                    dBodyEnable(bodyIt->second);
                    dBodyAddForce(bodyIt->second, force.x, force.y, force.z);

                    // Player bounce-back: slight deceleration on push impact.
                    // Very subtle — the player should feel mild resistance, not
                    // a camera jolt. Scale is 0.005 (vs 0.06 for the object),
                    // so pushing a 30kg crate while running gives ~0.04 units/s
                    // deceleration. Only noticeable as a tiny "thud" feel.
                    constexpr float kPlayerReactionScale = 0.005f;
                    float playerElastic = (2.0f * objMass) / (PLAYER_MASS + objMass);
                    Vector3 playerReaction = c.normal * vDotN * playerElastic * kPlayerReactionScale;
                    playerReaction.z = 0.0f;  // horizontal only
                    mPlayer.applyImpulse(playerReaction);

                    // Wake bodies in contact with the pushed object (stack propagation).
                    // Without this, pushing the bottom of a stack only moves the bottom
                    // while stacked objects remain sleeping. (Inspired by Godot's
                    // wakeup_neighbours pattern.)
                    wakeContactNeighbours(bodyIt->second);

                    static int pushDbg = 0;
                    if (pushDbg++ < 30)
                        std::fprintf(stderr, "[ODE-PUSH] obj %d: vDotN=%.2f elastic=%.2f "
                                     "force=(%.1f,%.1f,%.1f) mass=%.1f reaction=%.3f\n",
                                     c.objectId, vDotN, elastic,
                                     force.x, force.y, force.z, objMass,
                                     glm::length(playerReaction));
                }
            }
            // Non-ODE pushable objects still handled by kinematic system
            mPushSystem->processPlayerContacts(contacts, preCollisionVel);
        }

        // Step ODE world at a fixed 100 Hz, decoupled from the player's
        // 12.5 Hz tick. The player rate is locked by the engine convention
        // ("preserve 12.5 Hz behavior exactly"), but ODE's discrete trimesh
        // collision tunnels small fast bodies through floors at 80 ms steps:
        // a key-sized geom under gravity covers more than its own thickness
        // per step. 10 ms substeps give ~8x more collision opportunities and
        // eliminate the dropped-key-through-floor failure mode without
        // changing any player dynamics. Push forces applied above are
        // consumed during the first substep (ODE clears forces after each
        // dWorldQuickStep), which is fine — push is a per-tick impulse.
        if (mODEWorld) {
            constexpr float kODEFixedDt = 1.0f / 100.0f;
            constexpr int kMaxODEStepsPerFrame = 30;  // 300 ms wall-clock cap
            mODEAccum += dt;
            int steps = 0;
            while (mODEAccum >= kODEFixedDt && steps < kMaxODEStepsPerFrame) {
                // 1) Collision detection — creates contact joints
                dSpaceCollide(mODESpace, this, &odeNearCallback);

                // 2) Step the solver. Angular damping + auto-disable handle
                //    resting stability — the original engine accumulated
                //    rotational velocity from contacts with per-axis filtering
                //    by P$PhysAttr.rot_axes; it never locked rotation outright.
                dWorldQuickStep(mODEWorld, kODEFixedDt);
                dJointGroupEmpty(mODEContacts);
                mODEAccum -= kODEFixedDt;
                ++steps;
            }
            if (steps >= kMaxODEStepsPerFrame) mODEAccum = 0.0f;  // spiral guard
        }

        // Sync dynamic bodies back to collision geometry + renderer
        syncDynamicBodies();

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
        static int w = 0; if (w++ < 5)
            std::fprintf(stderr, "[DEFAULT] DarkPhysics::getPosition: entity %d not found, returning origin\n", id);
        return Vector3(0.0f);
    }

    Vector3 getVelocity(EntityID id) const override {
        // TODO (Task 26): dBodyGetLinearVel for ODE bodies — stub returns zero
        static int w = 0; if (w++ < 5)
            std::fprintf(stderr, "[DEFAULT] DarkPhysics::getVelocity: STUB returning zero for entity %d\n", id);
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
    /// Also wires the IsPushable callback so stair stepping is suppressed
    /// for pushable objects (player pushes them instead of climbing over).
    void setPushSystem(ObjectPushSystem *pushSys) {
        mPushSystem = pushSys;
        if (pushSys) {
            mPlayer.setIsPushableCallback([pushSys](int32_t objID) -> bool {
                return pushSys->isPushable(objID);
            });
        }
    }

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

        // Light linear damping so objects on trimesh surfaces still settle
        // (auto-disable threshold catches them once below 0.05 u/s) without
        // visibly slowing thrown/pushed objects in flight. The original
        // engine had no global linear damping at all; 0.01 is a small
        // numerical cushion against micro-bounce jitter on trimesh contacts.
        dWorldSetLinearDamping(mODEWorld, 0.01);
        // Angular damping: 0.08 allows barrels/bottles to roll realistically
        // while still settling in a reasonable time. The original engine had
        // no angular damping — objects spun until friction stopped them.
        // 0.08 is a compromise: enough to prevent infinite spinning but low
        // enough for visible rolling on pushes and falls.
        dWorldSetAngularDamping(mODEWorld, 0.08);
        dWorldSetLinearDampingThreshold(mODEWorld, 0.01);
        dWorldSetAngularDampingThreshold(mODEWorld, 0.01);

        std::fprintf(stderr, "ODE world created (gravity=%.1f, ERP=0.8, CFM=1e-4, "
                     "linDamp=0.01, angDamp=0.08)\n", -GRAVITY);
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

            // dSpaceCollide only does broadphase (AABB overlap). Confirm
            // actual contact before reacting — without this, a held object
            // re-enabled near the player capsule fires the dynamic-contact
            // handler on AABB overlap alone, view-punching the thrower
            // from their own throw.
            dContact tmp[MAX_ODE_CONTACTS];
            int numContacts = dCollide(o1, o2, MAX_ODE_CONTACTS,
                                        &tmp[0].geom, sizeof(dContact));
            if (numContacts == 0) return;

            // Closing-velocity gate: only react when the object is moving
            // toward the player along player→object. Filters out the
            // common "I just threw this" case (object moves away from
            // player → closingSpeed ≤ 0) while still catching incoming
            // projectiles (closingSpeed > 0).
            const dReal *playerPosR = dGeomGetPosition(isPlayer1 ? o1 : o2);
            const dReal *objPosR = dGeomGetPosition(otherGeom);
            Vector3 playerToObj(
                static_cast<float>(objPosR[0] - playerPosR[0]),
                static_cast<float>(objPosR[1] - playerPosR[1]),
                static_cast<float>(objPosR[2] - playerPosR[2]));
            float pToObjLen = glm::length(playerToObj);
            if (pToObjLen > 1e-3f) {
                Vector3 axis = playerToObj / pToObjLen;
                const dReal *vel = dBodyGetLinearVel(dynBody);
                Vector3 objVel(static_cast<float>(vel[0]),
                               static_cast<float>(vel[1]),
                               static_cast<float>(vel[2]));
                float closingSpeed = -glm::dot(objVel, axis);
                if (closingSpeed < 0.5f) return;  // moving away or stationary
            }

            self->handlePlayerDynamicContact(otherGeom, dynBody);
            return;
        }

        // Normal object-vs-object collision
        dBodyID b1 = dGeomGetBody(o1);
        dBodyID b2 = dGeomGetBody(o2);

        // Skip if both are static (no dynamics to solve)
        if (!b1 && !b2) return;

        // Read per-object material properties from ODEGeomData.
        // When both geoms have data, average friction and take max elasticity.
        // When only one has data (e.g. object-vs-world-trimesh), use that
        // object's friction directly. Default 0.5 matches P$PhysAttr default.
        float mu = 0.5f, bounce = 0.1f;
        auto *d1 = static_cast<ODEGeomData *>(dGeomGetData(o1));
        auto *d2 = static_cast<ODEGeomData *>(dGeomGetData(o2));
        bool valid1 = d1 && d1->objID != static_cast<int32_t>(PLAYER_GEOM_TAG);
        bool valid2 = d2 && d2->objID != static_cast<int32_t>(PLAYER_GEOM_TAG);
        if (valid1 && valid2) {
            // Both are registered objects: average friction, max elasticity
            mu = (d1->friction + d2->friction) * 0.5f;
            bounce = std::max(d1->elasticity, d2->elasticity);
        } else if (valid1) {
            // Object vs world trimesh (or untagged geom): use object's friction
            mu = d1->friction;
            bounce = d1->elasticity;
        } else if (valid2) {
            mu = d2->friction;
            bounce = d2->elasticity;
        }

        // Generate contact points
        dContact contacts[MAX_ODE_CONTACTS];
        int numContacts = dCollide(o1, o2, MAX_ODE_CONTACTS,
                                    &contacts[0].geom, sizeof(dContact));

        // Detect trimesh involvement for contact parameter tuning.
        // Trimesh contacts need stiffer parameters to prevent sinking.
        bool hasTrimesh = (dGeomGetClass(o1) == dTriMeshClass ||
                           dGeomGetClass(o2) == dTriMeshClass);

        for (int i = 0; i < numContacts; ++i) {
            contacts[i].surface.mode = dContactBounce | dContactApprox1;
            contacts[i].surface.mu = mu;
            contacts[i].surface.bounce = std::min(bounce, 0.2f);
            contacts[i].surface.bounce_vel = 1.0f;  // need real speed to bounce

            if (hasTrimesh) {
                // Trimesh contacts: slightly soft to absorb micro-bounce jitter
                // on uneven world geometry, but stiff enough to prevent sinking.
                contacts[i].surface.mode |= dContactSoftERP | dContactSoftCFM;
                contacts[i].surface.soft_erp = 0.5;   // moderate correction
                contacts[i].surface.soft_cfm = 0.001;  // slight compliance
            } else if (b1 && b2) {
                // Object-on-object: softer for stable stacking.
                // (Inspired by Godot's per-contact soft parameters for stack stability.)
                contacts[i].surface.mode |= dContactSoftERP | dContactSoftCFM;
                contacts[i].surface.soft_erp = 0.3;   // gentler correction
                contacts[i].surface.soft_cfm = 0.005;  // more compliant
            }

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
        // High threshold: only react to objects moving fast enough to hurt
        // (thrown objects, explosions). Normal push contacts are ~1-2 units/s
        // and should NOT trigger view punch or knockback.
        if (speed < 5.0f) return;

        // Knockback impulse scaled by object mass and speed.
        // Very conservative: 0.002 scale means a 30kg object at 10 units/s
        // gives 0.6 units/s player impulse (barely perceptible movement).
        dMass mass;
        dBodyGetMass(dynBody, &mass);
        float impulseScale = static_cast<float>(mass.mass) * speed * 0.002f;
        Vector3 knockDir = glm::normalize(objVel);

        // Push the impulse into the lagged knockback channel rather than
        // adding it directly to velocity. PlayerPhysics::integrateKnockback
        // bleeds it into mVelocity over ~750 ms with an exponential transfer,
        // so the player feels a smooth shove instead of a one-tick teleport.
        mPlayer.applyKnockback(knockDir * impulseScale);

        // Direction-aware view punch. The spring is now slow + slightly
        // overdamped (PUNCH_SPRING=22, PUNCH_DAMPING=11), so the camera
        // crawls back over ~1 s without overshoot. Magnitude is halved
        // (3.0 → 1.5) to keep the kick from over-rotating against the
        // softer spring — the impulse persists much longer now.
        Vector3 forward = mPlayer.getForward();
        Vector3 right = mPlayer.getRight();
        float sideDot = glm::dot(knockDir, right);
        float fwdDot = glm::dot(knockDir, forward);
        float punchMag = impulseScale * 1.5f;
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
            // Include orientation: local Z axis in world space shows tilt
            if (mODEFrameCount % 30 == 0) {
                dMass mass;
                dBodyGetMass(body, &mass);
                const dReal *angVel = dBodyGetAngularVel(body);
                std::fprintf(stderr, "  [ODE] obj=%d mass=%.1f pos=(%.2f,%.2f,%.2f) "
                             "vel=(%.2f,%.2f,%.2f) angVel=(%.2f,%.2f,%.2f) "
                             "localZ=(%.3f,%.3f,%.3f)\n",
                             objID, mass.mass,
                             pos[0], pos[1], pos[2],
                             vel[0], vel[1], vel[2],
                             angVel[0], angVel[1], angVel[2],
                             R[8], R[9], R[10]);  // local Z axis in world coords
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

            // Create geom for the shape. SphereHat is treated as OBB —
            // the original engine's sphere+plane composite was a workaround
            // for missing OBB-vs-OBB collision; ODE provides true OBB-vs-OBB
            // via dBoxBox so we build SphereHats as boxes from the model bbox.
            const bool boxLike =
                (body.shapeType == CollisionShapeType::OBB ||
                 body.shapeType == CollisionShapeType::SphereHat);
            dGeomID geom = nullptr;
            if (boxLike) {
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

                // Set mass from P$PhysAttr — match the box/sphere choice above.
                dMass odeMass;
                if (boxLike) {
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
        // SphereHat shares the OBB-style box geom (see buildODEGeoms).
        const bool boxLike =
            (collBody->shapeType == CollisionShapeType::OBB ||
             collBody->shapeType == CollisionShapeType::SphereHat);
        if (boxLike) {
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

        // Start disabled (sleeping). Objects are woken by player push, explosions,
        // or contact propagation. Avoids ~130 bodies all settling simultaneously
        // on mission load (which would cause a CPU spike).
        dBodyDisable(odeBody);

        mODEBodies[objID] = odeBody;
        mODEScales[objID] = collBody->objectScale;
        return true;
    }

    /// Wake all ODE bodies that are currently in contact with the given body.
    /// Implements Godot's "wakeup_neighbours" pattern: when pushing the bottom
    /// of a stack, all objects on top must also wake so they respond to the
    /// movement. Uses the ODE contact joint group (which is cleared each step),
    /// so this must be called BEFORE dWorldQuickStep.
    void wakeContactNeighbours(dBodyID body) {
        if (!body) return;
        // Walk all joints on this body. Contact joints connect two bodies that
        // are touching. Wake any sleeping body on the other end.
        int numJoints = dBodyGetNumJoints(body);
        for (int i = 0; i < numJoints; ++i) {
            dJointID joint = dBodyGetJoint(body, i);
            if (dJointGetType(joint) != dJointTypeContact) continue;
            dBodyID b1 = dJointGetBody(joint, 0);
            dBodyID b2 = dJointGetBody(joint, 1);
            dBodyID other = (b1 == body) ? b2 : b1;
            if (other && !dBodyIsEnabled(other)) {
                dBodyEnable(other);
            }
        }
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

    // ── Held-object primitives (used by GrabSystem) ──
    //
    // While an object is held, ODE simulation is suspended for it: the body
    // is disabled (no integration) and the geom is disabled (no collision
    // queries). The grab system then teleports the geom to a player-driven
    // target each frame via setHoldTransform(). On release, the body is
    // re-enabled and given an initial velocity (zero for drop, forward
    // impulse for throw).

    /// Suspend simulation for objID — disables both the dynamic body and
    /// the geom so the object stops integrating, doesn't collide with the
    /// world or the player capsule, and zero its velocities. Also flags
    /// the ObjectCollisionWorld body so the player's custom narrowphase
    /// (testPlayerSpheres) ignores it — without this, the held OBB tracking
    /// the camera would push the player capsule on every frame.
    /// Returns true if the object had a dynamic body to suspend.
    bool enterHold(int32_t objID) {
        auto it = mODEBodies.find(objID);
        if (it == mODEBodies.end()) return false;
        dBodyID body = it->second;
        // Stop any current motion before suspension.
        dBodySetLinearVel(body, 0, 0, 0);
        dBodySetAngularVel(body, 0, 0, 0);
        dBodyDisable(body);

        // Remove from ODE collision space — the held geom shouldn't generate
        // contacts with the world trimesh or the player capsule. Matches the
        // original engine: held items clip through walls and don't collide
        // with the carrier.
        auto gIt = mODEGeoms.find(objID);
        if (gIt != mODEGeoms.end()) dGeomDisable(gIt->second);

        // Tell the player's custom narrowphase to skip this body too. ODE's
        // player capsule and the engine's sphere-vs-OBB system are independent
        // collision paths; both must ignore the held object to prevent the
        // player's head/body spheres from being pushed by their own carry.
        if (mObjectCollision) mObjectCollision->setSkipPlayerCollision(objID, true);
        return true;
    }

    /// While an object is held, drive its world transform every frame.
    /// Updates the ODE geom (kept in sync so re-enable picks up the right
    /// pose) and the renderer state (via the same applyModelMatrix path
    /// that all sim systems use, so scale + matrix flag match what
    /// syncDynamicBodies will overwrite on release).
    /// rotMat3 is column-major (glm convention).
    /// ObjectCollisionWorld AABBs are NOT updated — the body is flagged
    /// skipPlayerCollision in enterHold, so its stored worldPos is unused
    /// while held.
    void setHoldTransform(int32_t objID, const Vector3 &pos,
                          const glm::mat3 &rotMat3) {
        auto gIt = mODEGeoms.find(objID);
        if (gIt == mODEGeoms.end()) return;
        dGeomID geom = gIt->second;
        dGeomSetPosition(geom, pos.x, pos.y, pos.z);
        dMatrix3 R;
        glmMat3ToODE(rotMat3, R);
        dGeomSetRotation(geom, R);

        // Body pose tracks geom — when re-enabled, integration starts from here.
        auto bIt = mODEBodies.find(objID);
        if (bIt != mODEBodies.end()) {
            dBodySetPosition(bIt->second, pos.x, pos.y, pos.z);
            dBodySetRotation(bIt->second, R);
        }

        // Mirror into the renderer using the same code path as the dynamics
        // sync. This applies the correct P$Scale (looked up from mODEScales)
        // so the held object renders at its actual size instead of unit
        // scale, and sets ObjectState.hasMatrix consistently with what
        // syncDynamicBodies will write after release.
        if (mObjectStates) {
            Vector3 scale(1.0f);
            auto sIt = mODEScales.find(objID);
            if (sIt != mODEScales.end()) scale = sIt->second;

            // Build the same row-major model matrix syncDynamicBodies uses.
            // odeToGlmMatrix takes ODE pos/R + scale; reuse it for parity.
            const dReal posR[3] = {pos.x, pos.y, pos.z};
            Matrix4 modelMatrix = odeToGlmMatrix(posR, R, scale);
            applyModelMatrix(*mObjectStates, objID, modelMatrix, pos, scale);
        }
    }

    /// End the hold: clear collision flags, re-enable body+geom, apply
    /// initial linear velocity. linVel is in world units/sec; angVel in
    /// rad/sec (defaults to zero). After release, syncDynamicBodies takes
    /// over and overwrites ObjectStateMap each frame from the live ODE
    /// position — no special cleanup needed on the renderer side.
    ///
    /// Returns true on success, false if the release was DENIED because the
    /// chosen release position would penetrate world geometry. On denial NO
    /// state is mutated — the body stays held and the caller (GrabSystem)
    /// keeps the grab active. This is the engine-side equivalent of the
    /// player having to back away from a wall before they can throw.
    ///
    /// Geometry: the body is normally snapped forward of the player capsule
    /// (out to capsule_radius + obj_circumradius + ε along player→object).
    /// This mirrors the original engine's PUSHOUT pattern. Before applying
    /// the snap, we raycast the path from player to the proposed release
    /// position; if a wall blocks it, the release is denied so a clipped
    /// snap doesn't drop the crate touching a wall (which would then push
    /// the player back the moment ODE catches the contact).
    bool releaseFromHold(int32_t objID, const Vector3 &linVel,
                         const Vector3 &angVel = Vector3(0.0f)) {
        auto gIt = mODEGeoms.find(objID);
        auto bIt = mODEBodies.find(objID);
        if (bIt == mODEBodies.end()) return false;
        dBodyID body = bIt->second;

        // ── Compute desired release position ──
        // Pushout target if the carry pose is too close; current geom pose
        // otherwise. We don't mutate anything yet — we have to validate
        // clearance first so denial is a true no-op.
        Vector3 releasePos;
        bool needsSnap = false;
        float circumR = 0.0f;

        if (gIt != mODEGeoms.end()) {
            dGeomID geom = gIt->second;
            circumR = computeGeomCircumradius(geom);
            const dReal *gPos = dGeomGetPosition(geom);
            const Vector3 playerPos = mPlayer.getPosition();
            Vector3 objPos(static_cast<float>(gPos[0]),
                           static_cast<float>(gPos[1]),
                           static_cast<float>(gPos[2]));
            Vector3 toObj = objPos - playerPos;
            float toObjLen = glm::length(toObj);
            constexpr float kPlayerCapsuleRadius = 1.2f;
            constexpr float kEpsilon = 0.05f;
            float minDist = kPlayerCapsuleRadius + circumR + kEpsilon;
            if (toObjLen < minDist) {
                Vector3 dir;
                float linLen = glm::length(linVel);
                if (linLen > 1e-3f) {
                    dir = linVel / linLen;
                } else if (toObjLen > 1e-3f) {
                    dir = toObj / toObjLen;
                } else {
                    // Truly degenerate: snap straight up so gravity takes over.
                    dir = Vector3(0.0f, 0.0f, 1.0f);
                }
                releasePos = playerPos + dir * minDist;
                needsSnap = true;
            } else {
                releasePos = objPos;
            }

            // ── Clearance check against the world trimesh ──
            // Cast a ray from the player to (releasePos + leading-face
            // margin). If anything blocks the path closer than the box's
            // leading face would reach, deny the release. The carrier
            // keeps the object and has to step away from the wall.
            //
            // We only test the world trimesh today; static OBB objects
            // (columns, statues) aren't covered. Phase 1 (convex hulls)
            // changes how those are represented; revisit then.
            if (mWorldTrimesh) {
                Vector3 fromPlayer = releasePos - playerPos;
                float releaseDist = glm::length(fromPlayer);
                if (releaseDist > 1e-3f) {
                    Vector3 rayDir = fromPlayer / releaseDist;
                    float maxRay = releaseDist + circumR + 0.1f;
                    dGeomID rayGeom = dCreateRay(0, maxRay);
                    dGeomRaySet(rayGeom,
                        playerPos.x, playerPos.y, playerPos.z,
                        rayDir.x, rayDir.y, rayDir.z);
                    dContactGeom hit;
                    int n = dCollide(rayGeom, mWorldTrimesh, 1,
                                     &hit, sizeof(dContactGeom));
                    dGeomDestroy(rayGeom);
                    if (n > 0 && hit.depth < releaseDist + circumR) {
                        std::fprintf(stderr,
                            "[DarkPhysics] release obj=%d DENIED — wall at "
                            "%.2fu blocks release at %.2fu (+%.2fu circumR)\n",
                            objID, static_cast<float>(hit.depth),
                            releaseDist, circumR);
                        return false;
                    }
                }
            }

            // ── Commit: apply snap, enable geom ──
            if (needsSnap) {
                dGeomSetPosition(geom, releasePos.x, releasePos.y, releasePos.z);
                dBodySetPosition(body, releasePos.x, releasePos.y, releasePos.z);
            }
            dGeomEnable(geom);
        }

        // Clear the player-narrowphase skip flag now that we've committed
        // to releasing. (Done late so a denial above leaves the flag set
        // and the grab loop can keep running cleanly.)
        if (mObjectCollision) mObjectCollision->setSkipPlayerCollision(objID, false);

        dBodyEnable(body);
        dBodySetLinearVel(body, linVel.x, linVel.y, linVel.z);
        dBodySetAngularVel(body, angVel.x, angVel.y, angVel.z);

        std::fprintf(stderr,
            "[DarkPhysics] release obj=%d linVel=(%.2f,%.2f,%.2f)\n",
            objID, linVel.x, linVel.y, linVel.z);
        return true;
    }

    /// Conservative bounding-sphere radius for a dGeom. Used by the release
    /// pushout above to keep the freshly-re-enabled body clear of the
    /// player capsule. Box class returns half-diagonal; sphere returns
    /// radius; everything else falls back to the geom AABB half-extent.
    static float computeGeomCircumradius(dGeomID geom) {
        switch (dGeomGetClass(geom)) {
            case dBoxClass: {
                dVector3 sides;
                dGeomBoxGetLengths(geom, sides);
                return 0.5f * std::sqrt(
                    static_cast<float>(sides[0] * sides[0] +
                                       sides[1] * sides[1] +
                                       sides[2] * sides[2]));
            }
            case dSphereClass:
                return static_cast<float>(dGeomSphereGetRadius(geom));
            default: {
                dReal aabb[6];
                dGeomGetAABB(geom, aabb);
                float dx = static_cast<float>(aabb[1] - aabb[0]);
                float dy = static_cast<float>(aabb[3] - aabb[2]);
                float dz = static_cast<float>(aabb[5] - aabb[4]);
                return 0.5f * std::sqrt(dx * dx + dy * dy + dz * dz);
            }
        }
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
