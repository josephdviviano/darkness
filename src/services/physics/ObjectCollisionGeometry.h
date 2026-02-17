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
 *    ObjectCollisionGeometry — collision bodies and queries for placed world
 *    objects (crates, furniture, doors, etc). Header-only, matching the
 *    pattern of CollisionGeometry.h and CellGeometry.h.
 *
 *    The object collision system:
 *    - Slab-based sphere-vs-OBB with Minkowski expansion
 *    - Portal cell broadphase (objects registered in overlapping WR cells)
 *
 *    The two-phase collision response (impulse bounce on first contact + constraint
 *    normal removal for persistent contacts) is integrated in PlayerPhysics.h,
 *    not in this file. This file handles collision body creation and detection only.
 *
 *    ODE UPGRADE PATH:
 *    This entire file implements custom collision detection that could be
 *    replaced by ODE (Open Dynamics Engine) for dynamic object simulation.
 *    Key extension points are annotated with "ODE UPGRADE" comments throughout.
 *    The upgrade strategy:
 *    1. Static objects: dGeomID only (no dBodyID) — immovable collision shapes
 *    2. Dynamic objects: dBodyID + dGeomID — ODE manages motion via dWorldStep
 *    3. Player remains custom (proven constraint-based system), but registers
 *       as kinematic dGeomID so ODE nearCallback can detect player-object pairs
 *    4. dNearCallback generates contact joints for object-vs-object dynamics
 *    See DarkPhysics.h for the integration point where dWorldID would be created.
 *
 *    Depends on: DarknessMath.h, CollisionGeometry.h (for SphereContact),
 *    CellGeometry.h (for findCameraCell), WRChunkParser.h (for cell data).
 *
 *****************************************************************************/

#ifndef __OBJECTCOLLISIONGEOMETRY_H
#define __OBJECTCOLLISIONGEOMETRY_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "DarknessMath.h"
#include "CollisionGeometry.h"  // for SphereContact
#include "CellGeometry.h"       // for findCameraCell
#include "WRChunkParser.h"      // for WRParsedData

namespace Darkness {

// Forward declarations for types used only in build().
// NOTE: Callers of ObjectCollisionWorld::build() must include TypedProperty.h
// (for getPropertyRawData) and ObjectPropParser.h (for ObjectPlacement).
struct ObjectPlacement;
struct ParsedBinMesh;
class PropertyService;

// ============================================================================
// Constants
// ============================================================================

/// Epsilon added to sphere radius during Minkowski expansion of OBB.
/// Prevents exact-touching cases from being missed.
constexpr float kSphereVsOBBEpsilon = 0.001f;

/// Distance threshold for breaking a persistent object contact.
/// Contact is destroyed when sphere-to-OBB-face distance exceeds this.
constexpr float kBreakObjectContactDist = 0.2f;

/// Distance threshold for door contacts (3x normal).
constexpr float kBreakDoorContactDist = 0.6f;

/// Relative velocity threshold for creating a persistent contact.
/// If the relative velocity along the normal is below this, a persistent
/// contact is created instead of just a bounce.
constexpr float kBreakObjectContactVel = 5.0f;

/// Damping factor for sphere-vs-OBB bounce impulse.
/// Only 2% of the elastic bounce velocity is retained — nearly inelastic.
/// ODE UPGRADE: Would be replaced by dContact.surface.bounce = 0.02.
constexpr float kOBBBounceScale = 0.02f;

/// Damping factor for sphere-vs-sphere bounce impulse.
/// 50% of elastic bounce — more bouncy than OBB contacts.
/// ODE UPGRADE: Would be replaced by dContact.surface.bounce = 0.5.
constexpr float kSphereBounceScale = 0.5f;

/// Epsilon shrink applied to OBB edge lengths (prevents exact edge contact
/// numerical issues).
constexpr float kOBBEdgeShrink = 0.999f;

/// Maximum number of constraint normals the solver can handle per frame.
constexpr int kMaxConstraints = 12;

// ── P$CollisionType bitfield flags (from COLLPROP.H) ──
// Objects without COLLISION_BOUNCE or COLLISION_NORESULT are completely passable.
constexpr uint32_t COLLISION_NONE     = 0x00;
constexpr uint32_t COLLISION_BOUNCE   = 0x01;  // object physically bounces on collision
constexpr uint32_t COLLISION_KILL     = 0x02;  // destroy on collision (projectiles)
constexpr uint32_t COLLISION_SLAY     = 0x04;  // slay target on collision
constexpr uint32_t COLLISION_NO_SOUND = 0x08;  // suppress collision sound
constexpr uint32_t COLLISION_NORESULT = 0x10;  // detect collision but no physics response

// ============================================================================
// Collision shape type — maps P$PhysType.type field
// ============================================================================

/// Collision shape type for an object's physics body.
enum class CollisionShapeType : uint8_t {
    OBB       = 0,  // Oriented bounding box (most furniture, doors, crates)
    Sphere    = 1,  // Bounding sphere
    SphereHat = 2,  // Sphere with flat top (AI heads) — treated as sphere for now
    None      = 3   // No collision (markers, scripts, particles)
};

// ============================================================================
// ObjectCollisionBody — one per collidable placed object
// ============================================================================

/// Collision body for a single placed world object.
/// Built at level load time from P$PhysType, P$PhysDims, and model bounding box.
/// All spatial data is in world space (position + orientation already baked in).
///
/// ODE UPGRADE: This struct would be replaced by a dGeomID handle.
/// For static objects: dGeomID = dCreateBox(space, lx, ly, lz) + dGeomSetPosition/Rotation.
/// For dynamic objects: also create dBodyID and attach with dGeomSetBody().
/// The ObjectCollisionBody data (edgeLengths, rotation, worldPos) would be
/// passed directly to dCreateBox/dGeomSetPosition/dGeomSetRotation.
struct ObjectCollisionBody {
    int32_t objID;                       // game object ID (for contact reporting)
    CollisionShapeType shapeType;        // OBB or Sphere

    // ── OBB data (shapeType == OBB) ──
    Vector3 worldPos;                    // OBB center in world coordinates
    Vector3 edgeLengths;                 // full edge lengths (X, Y, Z) — NOT half-extents
    glm::mat3 rotation;                  // 3x3 rotation matrix: columns = local axes in world space

    // ── Sphere data (shapeType == Sphere) ──
    float sphereRadius = 0.0f;          // bounding sphere radius (scaled)

    // ── Broadphase data ──
    // Portal cells this object's AABB overlaps. Built at load time, never changes
    // for static objects. Used for portal-cell-based broadphase queries.
    // ODE UPGRADE: Would be replaced by dHashSpace or dQuadTreeSpace membership.
    // ODE's dSpaceCollide2() handles broadphase internally.
    std::vector<int32_t> cellIDs;

    // Special flag from P$PhysType — affects PointVsNotSpecial narrowphase filter.
    // Special objects interact differently with sphere models (the sphere radius is NOT zeroed vs special).
    bool isSpecial = false;

    // ── Object stepping fields (from P$PhysAttr) ──
    // climbableSides: 6-bit bitmask of climbable OBB faces (bits 0-5 = +X,+Y,+Z,-X,-Y,-Z).
    // When != 0, stair stepping is suppressed and collision result is set to "nothing" —
    // the climbing system handles these objects instead. Default 0 = no climbable sides.
    int32_t climbableSides = 0;

    // isEdgeTrigger: if true, this OBB is a volume trigger (pressure plates, trip wires)
    // that generates entry/exit events instead of physical collision. Stair stepping
    // returns immediately when encountering an edge trigger OBB. Default false.
    bool isEdgeTrigger = false;

    // Precomputed world-space AABB for quick rejection tests
    Vector3 aabbMin;
    Vector3 aabbMax;
};

// ============================================================================
// ObjectContact — persistent contact between player submodel and object
// ============================================================================

/// Persistent contact between a player sphere and an object face.
/// Created when the relative velocity is below kBreakObjectContactVel.
/// Destroyed when distance exceeds kBreakObjectContactDist or submodel slides off face.
///
/// ODE UPGRADE: Would be replaced by ODE persistent contact joints managed
/// via dJointGroupCreate/dJointGroupEmpty per step. Contact surface parameters
/// (mu, bounce, bounce_vel) would replace our manual impulse calculation.
struct ObjectContact {
    int32_t objID;                 // which object
    int subModelIdx;               // which player sphere (0-4)
    int faceIdx;                   // which OBB face (0-5) or -1 for sphere
    Vector3 normal;                // contact normal (points away from object toward player)
    float distance;                // current separation (positive = apart, negative = penetrating)
    size_t bodyIdx;                // index into ObjectCollisionWorld::mBodies
};

// ============================================================================
// Sphere-vs-OBB collision test — slab-based with Minkowski expansion
// ============================================================================

/// Result of a sphere-vs-OBB collision test
struct OBBCollisionResult {
    bool hit;           // true if collision detected
    int faceIdx;        // which OBB face was hit (0-5), or -1 if no hit
    float penetration;  // depth of penetration (positive = overlapping)
    Vector3 normal;     // contact normal (outward from OBB face toward sphere)
    Vector3 point;      // approximate contact point on OBB surface
};

/// Slab-based sphere-vs-OBB collision test.
///
/// The OBB is Minkowski-expanded by the sphere radius: each face plane is pushed
/// outward by (radius + epsilon), then the sphere center is tested as a point
/// against the expanded OBB. If the point is inside all 6 slabs, a collision
/// is reported on the face with the shallowest penetration (largest signed distance).
///
/// This is the discrete (single-frame) version of a "swept test".
/// For Phase 2 (static objects at 60Hz), discrete testing is equivalent since the
/// player moves at most ~0.37 units/frame (max speed 22 u/s at 60Hz), well below
/// typical object dimensions (2+ units).
///
/// ODE UPGRADE: This entire function would be replaced by dCollide(geom1, geom2, N, contacts).
/// ODE handles sphere-vs-box natively with its internal SAT solver. The dContactGeom
/// output provides the same data (normal, depth, position) that this function returns.
///
/// @param sphereCenter  World-space center of the player sphere
/// @param sphereRadius  Radius of the player sphere (0 for point detectors)
/// @param body          The object's collision body (OBB)
/// @return              Collision result with face index, penetration, normal
inline OBBCollisionResult checkSphereVsOBB(
    const Vector3 &sphereCenter,
    float sphereRadius,
    const ObjectCollisionBody &body)
{
    OBBCollisionResult result;
    result.hit = false;
    result.faceIdx = -1;
    result.penetration = 0.0f;
    result.normal = Vector3(0.0f);
    result.point = Vector3(0.0f);

    // Inflated radius for Minkowski expansion
    float inflatedRadius = sphereRadius + kSphereVsOBBEpsilon;

    // Half-extents from full edge lengths
    Vector3 halfExtents = body.edgeLengths * 0.5f;

    // 6 face normals: columns of the rotation matrix (axes 0,1,2) and their negatives
    // GLM mat3 columns = local axes in world space
    Vector3 normals[6];
    normals[0] = body.rotation[0];   // local X axis (+)
    normals[1] = body.rotation[1];   // local Y axis (+)
    normals[2] = body.rotation[2];   // local Z axis (+)
    normals[3] = -body.rotation[0];  // local X axis (-)
    normals[4] = -body.rotation[1];  // local Y axis (-)
    normals[5] = -body.rotation[2];  // local Z axis (-)

    // Compute plane constants with Minkowski expansion.
    // For face i, the plane equation is: dot(normal[i], P) = d[i]
    // where d[i] = dot(normal[i], center + normal[i] * halfExtent[i%3])
    // Then inflate: d[i] += inflatedRadius
    //
    // Since normals[0..2] are unit vectors and normals[3..5] are their negatives:
    // d[i] = dot(normal[i], body.worldPos) + halfExtents[i%3] + inflatedRadius
    float d[6];
    for (int i = 0; i < 6; ++i) {
        // Plane constant: center projected onto normal + half-extent along that axis
        float centerDot = glm::dot(normals[i], body.worldPos);
        d[i] = centerDot + halfExtents[i % 3] + inflatedRadius;
    }

    // Slab test: check if sphere center is inside all 6 expanded slabs.
    // For each slab, compute signed distance from sphere center to the plane.
    // dist > 0 means the sphere center is OUTSIDE this slab.
    // dist <= 0 means the sphere center is inside (or on) this slab.
    //
    // Track the face with the largest (least negative) distance — that's the
    // shallowest penetration and the face we should push along.
    float bestDist = -1e30f;
    int bestFace = -1;

    for (int i = 0; i < 6; ++i) {
        float dist = glm::dot(normals[i], sphereCenter) - d[i];

        if (dist > 0.0f) {
            // Sphere center is outside this slab — no collision possible.
            // Early out: the sphere cannot be inside the expanded OBB.
            return result;
        }

        // Track the shallowest penetration (largest distance, closest to 0)
        if (dist > bestDist) {
            bestDist = dist;
            bestFace = i;
        }
    }

    // If we reach here, the sphere center is inside all 6 slabs.
    // Collision detected on the face with the shallowest penetration.
    if (bestFace < 0) {
        return result; // shouldn't happen, but guard
    }

    result.hit = true;
    result.faceIdx = bestFace;
    result.normal = normals[bestFace];
    // Penetration depth = how far past the face the sphere is
    // bestDist is negative (inside), so penetration = -bestDist
    result.penetration = -bestDist;
    // Approximate contact point: project sphere center onto the face plane
    result.point = sphereCenter - normals[bestFace] * bestDist;

    return result;
}

/// Sphere-vs-sphere collision test for sphere-shaped objects.
/// Standard distance-based test with normal along the center-to-center vector.
///
/// ODE UPGRADE: Would be replaced by dCollide() with dSphereClass geoms.
///
/// @param sphereCenter  World-space center of the player sphere
/// @param sphereRadius  Radius of the player sphere
/// @param body          The object's collision body (Sphere type)
/// @return              Collision result
inline OBBCollisionResult checkSphereVsSphere(
    const Vector3 &sphereCenter,
    float sphereRadius,
    const ObjectCollisionBody &body)
{
    OBBCollisionResult result;
    result.hit = false;
    result.faceIdx = -1;
    result.penetration = 0.0f;
    result.normal = Vector3(0.0f);
    result.point = Vector3(0.0f);

    Vector3 diff = sphereCenter - body.worldPos;
    float distSq = glm::dot(diff, diff);
    float combinedRadius = sphereRadius + body.sphereRadius + kSphereVsOBBEpsilon;

    if (distSq >= combinedRadius * combinedRadius) {
        return result; // no contact
    }

    float dist = std::sqrt(distSq);
    if (dist < 1e-6f) {
        // Degenerate case: centers coincide. Push along Z (up).
        result.hit = true;
        result.normal = Vector3(0.0f, 0.0f, 1.0f);
        result.penetration = combinedRadius;
        result.point = body.worldPos;
        return result;
    }

    result.hit = true;
    result.normal = diff / dist;  // points from object toward player sphere
    result.penetration = combinedRadius - dist;
    // Contact point: on the object's sphere surface toward the player
    result.point = body.worldPos + result.normal * body.sphereRadius;
    return result;
}

// ============================================================================
// Ray-vs-OBB intersection — slab method for object stair stepping
// ============================================================================

/// Result of a ray-vs-OBB intersection test.
struct RayOBBResult {
    bool hit;           // true if ray intersects the OBB
    int faceIdx;        // which OBB face was hit (0-5), -1 if no hit
    float t;            // parametric hit time along ray (0.0 = start, 1.0 = end)
    Vector3 point;      // world-space hit point on the OBB surface
};

/// Slab-based ray-vs-OBB intersection test.
///
/// Tests a ray segment (start → end) against the 6 face planes of an OBB.
/// Uses the standard slab method: for each axis, compute parametric entry/exit
/// times. If the maximum entry time is less than the minimum exit time and
/// occurs before the ray end, the ray intersects the OBB.
///
/// Used by the object stair stepping system to cast UP/FWD/DOWN rays against
/// OBBs instead of terrain. All three step-check rays use this test.
///
/// @param rayStart   World-space ray start point
/// @param rayEnd     World-space ray end point
/// @param body       The OBB collision body to test against
/// @return           Intersection result with face index, parametric t, hit point
inline RayOBBResult rayVsOBB(
    const Vector3 &rayStart,
    const Vector3 &rayEnd,
    const ObjectCollisionBody &body)
{
    RayOBBResult result;
    result.hit = false;
    result.faceIdx = -1;
    result.t = 0.0f;
    result.point = Vector3(0.0f);

    // Half-extents from full edge lengths
    Vector3 halfExtents = body.edgeLengths * 0.5f;

    // 6 face normals: rotation matrix columns (sides 0-2) and negated (sides 3-5)
    Vector3 normals[6];
    normals[0] = body.rotation[0];   // +X
    normals[1] = body.rotation[1];   // +Y
    normals[2] = body.rotation[2];   // +Z
    normals[3] = -body.rotation[0];  // -X
    normals[4] = -body.rotation[1];  // -Y
    normals[5] = -body.rotation[2];  // -Z

    // Plane constants: d[i] = dot(normal[i], center + normal[i] * halfExtent)
    // = dot(normal[i], center) + halfExtent[i%3]
    float d[6];
    for (int i = 0; i < 6; ++i) {
        d[i] = glm::dot(normals[i], body.worldPos) + halfExtents[i % 3];
    }

    // Slab intersection: track maximum entry time and minimum exit time.
    // Epsilon matching the original engine's tolerance for inside/outside test.
    constexpr float kSlabEpsilon = 0.0001f;
    float maxEntryTime = -1000000.0f;
    float minExitTime  =  1000000.0f;
    int bestSide = -1;

    for (int i = 0; i < 6; ++i) {
        float startDist = glm::dot(normals[i], rayStart) - d[i];
        float endDist   = glm::dot(normals[i], rayEnd)   - d[i];

        // Both inside this slab — no constraint
        if (startDist <= kSlabEpsilon && endDist <= kSlabEpsilon)
            continue;

        // Both outside this slab — ray cannot intersect OBB
        if (startDist >= -kSlabEpsilon && endDist >= -kSlabEpsilon) {
            bestSide = -1;
            break;
        }

        // Crossing — compute parametric intersection time
        float t = startDist / (startDist - endDist);

        if (startDist < 0.0f) {
            // Ray exiting this slab — track minimum exit time
            if (t < minExitTime)
                minExitTime = t;
        } else {
            // Ray entering this slab — track maximum entry time and face
            if (t > maxEntryTime) {
                maxEntryTime = t;
                bestSide = i;
            }
        }
    }

    if (bestSide < 0)
        return result; // missed — ray outside one slab entirely or inside all

    // Valid hit requires: entry before ray end AND entry before exit
    if (maxEntryTime < 1.0f && minExitTime > maxEntryTime) {
        result.hit = true;
        result.faceIdx = bestSide;
        result.t = maxEntryTime;
        result.point = rayStart + (rayEnd - rayStart) * maxEntryTime;
    }

    return result;
}

/// Compute the world-space outward normal for a given OBB face index (0-5).
///
/// Face index mapping matches the Dark Engine convention:
///   0 = +X axis (rotation column 0)
///   1 = +Y axis (rotation column 1)
///   2 = +Z axis (rotation column 2)
///   3 = -X axis (negated column 0)
///   4 = -Y axis (negated column 1)
///   5 = -Z axis (negated column 2)
///
/// @param body      The OBB collision body
/// @param faceIdx   Face index (0-5)
/// @return          Unit normal vector in world space
inline Vector3 getOBBFaceNormal(const ObjectCollisionBody &body, int faceIdx) {
    Vector3 normal = body.rotation[faceIdx % 3];
    if (faceIdx > 2)
        normal = -normal;
    return normal;
}

// ============================================================================
// ObjectCollisionWorld — manages all object collision bodies and queries
// ============================================================================

/// Manages collision bodies for all placed world objects and provides
/// sphere-vs-object queries for the player physics system.
///
/// Built once at level load time. For Phase 2, all objects are static
/// (no position updates during gameplay).
///
/// ODE UPGRADE: This class would be replaced by a dSpaceID containing all
/// object dGeomIDs. The build() method would call dCreateBox/dCreateSphere
/// for each object and add them to the space. testPlayerSpheres() would be
/// replaced by dSpaceCollide2(playerSpace, objectSpace, nearCallback) where
/// the nearCallback creates contact joints via dJointCreateContact().
/// For dynamic objects, dBodyID would also be created and dGeomSetBody() called.
class ObjectCollisionWorld {
public:
    ObjectCollisionWorld() = default;

    // ── Build collision bodies from level data ──

    /// Create collision bodies for all placed objects that have physics.
    /// Reads P$PhysType and P$PhysDims from the property system to determine
    /// shape type and dimensions. Falls back to model bounding box if no
    /// explicit physics properties are found.
    ///
    /// ODE UPGRADE: Each body would become a dGeomID in a dHashSpace.
    /// Static objects get dGeomID only; dynamic objects also get dBodyID.
    /// dGeomSetPosition/dGeomSetRotation replaces our stored worldPos/rotation.
    ///
    /// @param objects   All concrete placed objects from ObjectPropParser
    /// @param models    Parsed .bin models with bounding boxes
    /// @param propSvc   Property service for reading P$PhysType/P$PhysDims
    /// @param wr        Parsed WR cell data for portal-cell broadphase registration
    inline void build(
        const std::vector<ObjectPlacement> &objects,
        const std::unordered_map<std::string, ParsedBinMesh> &models,
        PropertyService *propSvc,
        const WRParsedData &wr)
    {
        mBodies.clear();
        mObjIDToBody.clear();
        mCellToObjects.clear();
        mWR = &wr;

        int skippedNoPos = 0, skippedNoModel = 0, skippedNone = 0, skippedNoPhysType = 0;
        int skippedNoCollision = 0, skippedRemoveOnSleep = 0, createdOBB = 0, createdSphere = 0;
        int specialCount = 0;

        for (size_t i = 0; i < objects.size(); ++i) {
            const auto &obj = objects[i];

            // Skip objects without world placement
            if (!obj.hasPosition) {
                ++skippedNoPos;
                continue;
            }

            // Skip objects whose model wasn't loaded
            std::string modelName(obj.modelName);
            auto mit = models.find(modelName);
            if (mit == models.end() || !mit->second.valid) {
                ++skippedNoModel;
                continue;
            }
            const auto &mesh = mit->second;

            // Determine collision shape type from P$PhysType (with inheritance).
            // Objects without P$PhysType don't participate in physics collision at all
            // Defaulting to OBB for objects without it would create
            // collision bodies for visual-only decorations (room pieces, terrain, etc.).
            CollisionShapeType shapeType = CollisionShapeType::OBB;
            bool hasPhysType = false;
            bool isSpecial = false;     // P$PhysType.special flag
            if (propSvc) {
                // Forward-declared; implementation uses getTypedProperty from TypedProperty.h
                // which is included by the caller (DarkPhysics.h)
                size_t rawSize = 0;
                const uint8_t *rawData = getPropertyRawData(propSvc, "PhysType", obj.objID, rawSize);
                if (rawData && rawSize >= 4) {
                    uint32_t ptype;
                    std::memcpy(&ptype, rawData, sizeof(uint32_t));
                    hasPhysType = true;

                    if (ptype == 3) {
                        // PhysType.type == None — skip, no collision
                        ++skippedNone;
                        continue;
                    } else if (ptype == 1 || ptype == 2) {
                        shapeType = CollisionShapeType::Sphere;
                    } else {
                        shapeType = CollisionShapeType::OBB;
                    }

                    // Check remove_on_sleep flag at offset 8 in P$PhysType.
                    // P$PhysType layout: {uint32 type, int32 num_submodels,
                    //                     bool32 remove_on_sleep, bool32 special}
                    //
                    // Static objects with remove_on_sleep=TRUE
                    // go to sleep immediately at level start (zero velocity, no forces),
                    // which triggers PhysDeregisterModel() — completely destroying the
                    // physics model. The object remains visual but has no collision.
                    //
                    // For Phase 2 (static objects only, no dynamic simulation),
                    // remove_on_sleep=TRUE effectively means "no collision."
                    if (rawSize >= 12) {
                        int32_t removeOnSleep;
                        std::memcpy(&removeOnSleep, rawData + 8, sizeof(int32_t));
                        if (removeOnSleep != 0) {
                            ++skippedRemoveOnSleep;
                            continue;
                        }
                    }

                    // Read the 'special' flag at offset 12.
                    // 'special' maps to kPMF_Special on the
                    // physics model. It interacts with PointVsNotSpecial in the
                    // narrowphase: sphere models with PointVsNotSpecial=TRUE treat
                    // non-special OBBs with zeroed radius (point-only collision).
                    if (rawSize >= 16) {
                        int32_t spec;
                        std::memcpy(&spec, rawData + 12, sizeof(int32_t));
                        isSpecial = (spec != 0);
                    }
                }
            }

            // No P$PhysType property found (not on object, not through inheritance) -- no physics model, skip.
            if (!hasPhysType) {
                ++skippedNoPhysType;
                continue;
            }

            // Check P$CollisionType — the master switch for collision detection checks:
            // Objects without the Bounce or NoResult bits set are completely passable.
            // Property name in .pldef: "Collision" (label: "CollisionType")
            // Data: single uint32 bitfield (physcollisionresult).
            if (propSvc) {
                size_t collSize = 0;
                const uint8_t *collData = getPropertyRawData(
                    propSvc, "Collision", obj.objID, collSize);
                if (collData && collSize >= 4) {
                    uint32_t collFlags;
                    std::memcpy(&collFlags, collData, sizeof(uint32_t));
                    if (!(collFlags & (COLLISION_BOUNCE | COLLISION_NORESULT))) {
                        // No bounce or noresult — object is passable
                        ++skippedNoCollision;
                        continue;
                    }
                }
                // If no P$CollisionType property exists, default is to have bounce collision.
            }

            // Build the collision body
            ObjectCollisionBody body;
            body.objID = obj.objID;
            body.shapeType = shapeType;
            body.isSpecial = isSpecial;

            // Read P$PhysAttr for object stepping fields (OBB-only).
            // P$PhysAttr layout (52 bytes): gravity(0), mass(4), density(8),
            // elasticity(12), base_friction(16), cog_offset(20), rot_axes(32),
            // rest_axes(36), climbable_sides(40), edge_trigger(44), pore_size(48).
            // climbable_sides: 6-bit bitmask of climbable OBB faces.
            // edge_trigger: BOOL (int32) — volume trigger, not physical.
            if (propSvc && shapeType == CollisionShapeType::OBB) {
                size_t attrSize = 0;
                const uint8_t *attrData = getPropertyRawData(
                    propSvc, "PhysAttr", obj.objID, attrSize);
                if (attrData && attrSize >= 48) {
                    int32_t climbSides;
                    std::memcpy(&climbSides, attrData + 40, sizeof(int32_t));
                    body.climbableSides = climbSides;

                    int32_t edgeTrig;
                    std::memcpy(&edgeTrig, attrData + 44, sizeof(int32_t));
                    body.isEdgeTrigger = (edgeTrig != 0);
                }
            }

            // Compute rotation matrix from object facing angles.
            // Original rotation order: M = Rz(heading) * Ry(pitch) * Rx(bank)
            // NON-NEGATED angles (negation is only for bgfx GPU transpose compensation).
            const float angScale = 2.0f * 3.14159265358979f / 65536.0f;
            float h = static_cast<float>(obj.heading) * angScale;
            float p = static_cast<float>(obj.pitch)   * angScale;
            float b = static_cast<float>(obj.bank)     * angScale;
            // glm::eulerAngleZYX produces a 4x4 matrix; extract upper-left 3x3
            glm::mat4 rotMat4 = glm::eulerAngleZYX(h, p, b);
            body.rotation = glm::mat3(rotMat4);

            // Model bounding box dimensions (unscaled, from .bin header)
            Vector3 bboxMin(mesh.bboxMin[0], mesh.bboxMin[1], mesh.bboxMin[2]);
            Vector3 bboxMax(mesh.bboxMax[0], mesh.bboxMax[1], mesh.bboxMax[2]);
            Vector3 bboxSize = bboxMax - bboxMin;
            Vector3 bboxCenter = (bboxMin + bboxMax) * 0.5f;

            // Apply P$Scale to dimensions
            Vector3 scale(obj.scaleX, obj.scaleY, obj.scaleZ);

            if (shapeType == CollisionShapeType::OBB) {
                // Default edge lengths = model bbox dimensions * scale * 0.999.
                body.edgeLengths = bboxSize * scale * kOBBEdgeShrink;

                // Check P$PhysDims for explicit size override.
                // each concrete object gets its own copy. InitPhysProperty writes the model
                // bbox * scale into PhysDims.size, and UpdatePhysModel then applies
                // PhysDims.size as the final edge lengths (overriding the 0.999 shrink).
                //
                // PhysDims.size values ALREADY INCLUDE the object's scale — they are
                // the FINAL edge lengths, not unscaled dimensions. Do NOT multiply by
                // scale again. Also no 0.999 shrink.
                if (propSvc) {
                    size_t dimSize = 0;
                    const uint8_t *dimData = getPropertyRawData(propSvc, "PhysDims", obj.objID, dimSize);
                    if (dimData && dimSize >= 44) {
                        // P$PhysDims layout: float radius[2] (8) + vector offset[2] (24)
                        //                    + vector size (12) + bool32 pt_vs_terrain (4)
                        //                    + bool32 pt_vs_not_special (4) = 52 bytes
                        // sizeXYZ starts at byte offset 32
                        float sx, sy, sz;
                        std::memcpy(&sx, dimData + 32, sizeof(float));
                        std::memcpy(&sy, dimData + 36, sizeof(float));
                        std::memcpy(&sz, dimData + 40, sizeof(float));
                        // Only override if explicit sizes are set (non-zero). Use values directly.
                        if (sx > 0.001f && sy > 0.001f && sz > 0.001f) {
                            body.edgeLengths = Vector3(sx, sy, sz);
                        }
                    }
                }

                // OBB center = object position + rotated model bbox center offset
                Vector3 rotatedCenter = body.rotation * (bboxCenter * scale);
                body.worldPos = Vector3(obj.x, obj.y, obj.z) + rotatedCenter;

                ++createdOBB;
            } else {
                // Sphere shape — use P$PhysDims.radius[0] or model sphere radius
                body.sphereRadius = std::max({bboxSize.x, bboxSize.y, bboxSize.z}) * 0.5f
                                    * std::max({scale.x, scale.y, scale.z});

                if (propSvc) {
                    size_t dimSize = 0;
                    const uint8_t *dimData = getPropertyRawData(propSvc, "PhysDims", obj.objID, dimSize);
                    if (dimData && dimSize >= 4) {
                        float r0;
                        std::memcpy(&r0, dimData, sizeof(float));
                        if (r0 > 0.001f) {
                            body.sphereRadius = r0 * std::max({scale.x, scale.y, scale.z});
                        }
                    }
                }

                body.worldPos = Vector3(obj.x, obj.y, obj.z);
                // Edge lengths unused for spheres but zero them out
                body.edgeLengths = Vector3(0.0f);

                ++createdSphere;
            }

            if (isSpecial) ++specialCount;

            // Compute world-space AABB for broadphase registration.
            // For OBBs: enumerate all 8 rotated corners.
            // For spheres: AABB = center +/- radius.
            computeAABB(body);

            // Register in portal cells — find all cells the AABB overlaps
            registerInCells(body, wr);

            mBodies.push_back(std::move(body));
            mObjIDToBody[mBodies.back().objID] = mBodies.size() - 1;
        }

        // Build reverse lookup: cell index → body indices
        for (size_t bi = 0; bi < mBodies.size(); ++bi) {
            for (int32_t cell : mBodies[bi].cellIDs) {
                mCellToObjects[cell].push_back(bi);
            }
        }

        std::fprintf(stderr,
            "ObjectCollision: %zu bodies (%d OBB, %d sphere, %d special), %zu cells mapped\n"
            "  skipped: %d noPos, %d noModel, %d noPhysType, %d noColl, %d none, %d sleep\n",
            mBodies.size(), createdOBB, createdSphere, specialCount,
            mCellToObjects.size(),
            skippedNoPos, skippedNoModel, skippedNoPhysType, skippedNoCollision,
            skippedNone, skippedRemoveOnSleep);
    }

    // ── Runtime collision queries ──

    /// Test all 5 player spheres against nearby object collision bodies.
    /// Uses portal-cell broadphase to find candidates, then runs narrowphase
    /// (slab test for OBBs, distance test for spheres).
    ///
    /// Results are appended to outContacts as SphereContact structs with:
    /// - cellIdx = -1  (sentinel: this is an object contact, not a WR polygon)
    /// - polyIdx = (bodyIndex << 4) | faceIdx  (encoded for persistent matching)
    /// - textureIdx = -1
    ///
    /// ODE UPGRADE: This would be replaced by dSpaceCollide2() in a dNearCallback.
    /// The nearCallback would call dCollide() for each pair and create contact
    /// joints from the resulting dContactGeom array.
    ///
    /// @param sphereCenters  World-space centers of all 5 player submodel spheres
    /// @param sphereRadii    Radii of all 5 submodel spheres (0 for point detectors)
    /// @param playerCell     Current WR cell containing the player body center
    /// @param outContacts    Contacts are appended here (caller manages clear)
    inline void testPlayerSpheres(
        const Vector3 *sphereCenters,
        const float *sphereRadii,
        int numSpheres,
        int32_t playerCell,
        std::vector<SphereContact> &outContacts) const
    {
        if (mBodies.empty() || playerCell < 0 || !mWR)
            return;

        // ── Broadphase: collect candidate objects from portal-connected cells ──
        // Start with the player's cell, then add portal-adjacent cells.
        // Deduplicate candidate body indices to avoid testing the same object
        // from multiple cells.
        mCandidateSet.clear();
        mCandidates.clear();

        // Collect from player's cell
        collectCandidatesFromCell(playerCell);

        // Collect from portal-adjacent cells (player sphere might reach into them)
        const auto &cell = mWR->cells[playerCell];
        int numSolid = cell.numPolygons - cell.numPortals;
        for (int pi = numSolid; pi < cell.numPolygons; ++pi) {
            int32_t tgtCell = static_cast<int32_t>(cell.polygons[pi].tgtCell);
            if (tgtCell >= 0 && tgtCell < static_cast<int32_t>(mWR->numCells)) {
                collectCandidatesFromCell(tgtCell);
            }
        }

        // ── Narrowphase: test each sphere against each candidate ──
        for (size_t ci = 0; ci < mCandidates.size(); ++ci) {
            size_t bi = mCandidates[ci];
            const auto &body = mBodies[bi];

            for (int s = 0; s < numSpheres; ++s) {
                OBBCollisionResult cr;

                if (body.shapeType == CollisionShapeType::OBB) {
                    cr = checkSphereVsOBB(sphereCenters[s], sphereRadii[s], body);
                } else if (body.shapeType == CollisionShapeType::Sphere ||
                           body.shapeType == CollisionShapeType::SphereHat) {
                    cr = checkSphereVsSphere(sphereCenters[s], sphereRadii[s], body);
                } else {
                    continue; // None type — shouldn't be in mBodies, but guard
                }

                if (cr.hit) {
                    SphereContact contact;
                    contact.normal = cr.normal;
                    contact.penetration = cr.penetration;
                    // Object contact sentinel: cellIdx = -1 (not a WR polygon).
                    // WR contacts have cellIdx >= 0, so no collision between namespaces.
                    contact.cellIdx = -1;
                    // Encode body index and face for persistent contact matching.
                    // bodyIndex in upper bits, faceIdx in lower 4 bits.
                    // For sphere contacts (faceIdx = -1), -1 & 0xF = 0xF, which is
                    // distinct from OBB face indices (0-5) and avoids false matches.
                    contact.polyIdx = static_cast<int32_t>((bi << 4) | (cr.faceIdx & 0xF));
                    contact.textureIdx = -1;
                    contact.age = 0;
                    // Tag with object ID and player submodel index for
                    // object stepping decisions in tryStairStep().
                    contact.objectId = body.objID;
                    contact.submodelIdx = static_cast<int8_t>(s);

                    outContacts.push_back(contact);
                }
            }
        }
    }

    // ── Diagnostics ──

    /// Number of collision bodies created.
    size_t bodyCount() const { return mBodies.size(); }

    /// Number of portal cells with registered objects.
    size_t cellCount() const { return mCellToObjects.size(); }

    /// Access a body by index (for debug display).
    const ObjectCollisionBody &getBody(size_t idx) const { return mBodies[idx]; }

    /// Access all bodies.
    const std::vector<ObjectCollisionBody> &getBodies() const { return mBodies; }

    /// Find a collision body by game object ID.
    /// Returns nullptr if no body exists for the given object ID.
    /// Used by object stair stepping to look up OBB properties
    /// (climbable sides, edge trigger, rotation, edge lengths).
    const ObjectCollisionBody *findBodyByObjID(int32_t objID) const {
        auto it = mObjIDToBody.find(objID);
        return (it != mObjIDToBody.end()) ? &mBodies[it->second] : nullptr;
    }

private:
    std::vector<ObjectCollisionBody> mBodies;

    // Object ID → index into mBodies. Built at load time for O(1) body lookup.
    // Used by object stair stepping to find OBB properties from contact objectId.
    std::unordered_map<int32_t, size_t> mObjIDToBody;

    // Portal cell → indices into mBodies. Built at load time, static.
    // ODE UPGRADE: Would be replaced by dHashSpace containing all dGeomIDs.
    std::unordered_map<int32_t, std::vector<size_t>> mCellToObjects;

    // Pointer to WR data for cell access during queries. Must outlive this object.
    const WRParsedData *mWR = nullptr;

    // Scratch buffers for broadphase (mutable to allow const query methods).
    // NOT thread-safe: testPlayerSpheres() must not be called concurrently.
    // This is fine for Phase 2 (single-threaded physics step).
    mutable std::unordered_set<size_t> mCandidateSet;
    mutable std::vector<size_t> mCandidates;

    // ── AABB computation ──

    /// Compute world-space axis-aligned bounding box for a collision body.
    /// For OBBs: enumerate all 8 rotated corners.
    /// For spheres: center +/- radius.
    static void computeAABB(ObjectCollisionBody &body) {
        if (body.shapeType == CollisionShapeType::OBB) {
            // Scale rotation columns by half-extents to get corner offsets
            Vector3 halfExt = body.edgeLengths * 0.5f;
            Vector3 ax = body.rotation[0] * halfExt.x;
            Vector3 ay = body.rotation[1] * halfExt.y;
            Vector3 az = body.rotation[2] * halfExt.z;

            // Track min/max across all 8 sign combinations
            body.aabbMin = body.worldPos;
            body.aabbMax = body.worldPos;

            for (int sx = -1; sx <= 1; sx += 2) {
                for (int sy = -1; sy <= 1; sy += 2) {
                    for (int sz = -1; sz <= 1; sz += 2) {
                        Vector3 corner = body.worldPos
                            + ax * static_cast<float>(sx)
                            + ay * static_cast<float>(sy)
                            + az * static_cast<float>(sz);
                        body.aabbMin = glm::min(body.aabbMin, corner);
                        body.aabbMax = glm::max(body.aabbMax, corner);
                    }
                }
            }
        } else {
            // Sphere AABB
            Vector3 rv(body.sphereRadius);
            body.aabbMin = body.worldPos - rv;
            body.aabbMax = body.worldPos + rv;
        }
    }

    // ── Portal cell registration ──

    /// Register an object in all portal cells its AABB overlaps.
    /// Uses findCameraCell() at the center, then BFS through portal adjacency
    /// to find additional cells the AABB touches.
    void registerInCells(ObjectCollisionBody &body, const WRParsedData &wr) {
        body.cellIDs.clear();

        // Find the cell containing the object center
        int32_t centerCell = findCameraCell(wr, body.worldPos.x, body.worldPos.y, body.worldPos.z);
        if (centerCell < 0) {
            // Object center is outside all cells — try corners of AABB
            centerCell = findCameraCell(wr, body.aabbMin.x, body.aabbMin.y, body.aabbMin.z);
            if (centerCell < 0) {
                return; // completely outside world geometry
            }
        }

        // BFS from center cell through portals to find all cells the AABB overlaps
        std::vector<int32_t> queue;
        std::unordered_set<int32_t> visited;

        queue.push_back(centerCell);
        visited.insert(centerCell);

        while (!queue.empty()) {
            int32_t cellIdx = queue.back();
            queue.pop_back();

            body.cellIDs.push_back(cellIdx);

            // Check portal-connected cells
            const auto &cell = wr.cells[cellIdx];
            int numSolid = cell.numPolygons - cell.numPortals;
            for (int pi = numSolid; pi < cell.numPolygons; ++pi) {
                int32_t tgtCell = static_cast<int32_t>(cell.polygons[pi].tgtCell);
                if (tgtCell < 0 || tgtCell >= static_cast<int32_t>(wr.numCells))
                    continue;
                if (visited.count(tgtCell))
                    continue;

                // Check if the target cell overlaps the object's AABB.
                // Use the cell's bounding sphere for a quick overlap test.
                const auto &tgt = wr.cells[tgtCell];
                if (aabbOverlapsSphere(body.aabbMin, body.aabbMax,
                                        tgt.center, tgt.radius)) {
                    visited.insert(tgtCell);
                    queue.push_back(tgtCell);
                }
            }
        }
    }

    /// Quick test: does an AABB overlap a bounding sphere?
    static bool aabbOverlapsSphere(const Vector3 &aabbMin, const Vector3 &aabbMax,
                                    const Vector3 &sphereCenter, float sphereRadius) {
        // Find the closest point on the AABB to the sphere center
        float dx = std::max(aabbMin.x - sphereCenter.x, 0.0f);
        dx = std::max(dx, sphereCenter.x - aabbMax.x);
        float dy = std::max(aabbMin.y - sphereCenter.y, 0.0f);
        dy = std::max(dy, sphereCenter.y - aabbMax.y);
        float dz = std::max(aabbMin.z - sphereCenter.z, 0.0f);
        dz = std::max(dz, sphereCenter.z - aabbMax.z);

        return (dx * dx + dy * dy + dz * dz) <= (sphereRadius * sphereRadius);
    }

    /// Collect candidate body indices from a single portal cell.
    /// Deduplicates using mCandidateSet.
    inline void collectCandidatesFromCell(int32_t cellIdx) const {
        auto it = mCellToObjects.find(cellIdx);
        if (it == mCellToObjects.end())
            return;

        for (size_t bi : it->second) {
            if (mCandidateSet.insert(bi).second) {
                mCandidates.push_back(bi);
            }
        }
    }
};

} // namespace Darkness

#endif // __OBJECTCOLLISIONGEOMETRY_H
