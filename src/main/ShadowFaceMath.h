/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2024-2026 darkness contributors
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

// Omni-light shadow face geometry — S1 of PLAN.HIGH_RES_SHADOWS.md.
//
// An omni light renders SIX 2D faces into a shadow atlas, not a cube map:
// the project's GLSL 120 fragment profile has no cube-shadow samplers, and
// 2D faces pack into one atlas with per-face culling for free. Faces store
// LINEAR DISTANCE from the light (R32F), not device depth — the shadow test
// is then a plain distance compare with a plain SAMPLER2D on every profile,
// it round-trips through CPU readback for the acceptance cross-check, and it
// dodges every projective-depth precision and convention trap (GL vs Metal
// depth ranges never enter the comparison).
//
// This header is deliberately bgfx-free (GLM only) so the face selection and
// projection math — which the S4 world shader must MIRROR exactly — is
// property-testable headlessly. ShadowMapCache.h owns the GPU side.

#pragma once

#include "DarknessMath.h"

#include <glm/gtc/matrix_transform.hpp>

#include <cmath>
#include <cstdint>

namespace Darkness {

constexpr int kShadowFaceCount = 6;

// Face order: +X, -X, +Y, -Y, +Z, -Z. Up vectors are a free internal
// convention (we control both the render and the lookup), chosen so no
// face's forward is parallel to its up.
inline Vector3 shadowFaceDir(int face) {
    switch (face) {
    case 0: return Vector3( 1,  0,  0);
    case 1: return Vector3(-1,  0,  0);
    case 2: return Vector3( 0,  1,  0);
    case 3: return Vector3( 0, -1,  0);
    case 4: return Vector3( 0,  0,  1);
    default:return Vector3( 0,  0, -1);
    }
}

inline Vector3 shadowFaceUp(int face) {
    return (face >= 4) ? Vector3(0, 1, 0) : Vector3(0, 0, 1);
}

// Which face covers direction `v` (light → point): the major axis. Every
// non-zero direction maps to exactly one face; ties resolve consistently by
// the comparison order below, and the same order must be used in GLSL.
inline int shadowFaceForDirection(const Vector3 &v) {
    const float ax = std::abs(v.x), ay = std::abs(v.y), az = std::abs(v.z);
    if (ax >= ay && ax >= az) return v.x >= 0.0f ? 0 : 1;
    if (ay >= az)             return v.y >= 0.0f ? 2 : 3;
    return v.z >= 0.0f ? 4 : 5;
}

// View matrix of a face: right-handed look-along from the light.
inline Matrix4 shadowFaceView(const Vector3 &lightPos, int face) {
    const Vector3 dir = shadowFaceDir(face);
    return glm::lookAtRH(lightPos, lightPos + dir, shadowFaceUp(face));
}

// 90° square projection covering the face exactly. Clip conventions are the
// GPU's concern at render time (ShadowMapCache applies the backend's depth
// range); the UV math below never touches clip z — faces store linear
// distance, so this projection only has to LAND texels in the right place.
inline Matrix4 shadowFaceProj(float nearZ, float farZ) {
    return glm::perspectiveRH_NO(glm::radians(90.0f), 1.0f, nearZ, farZ);
}

// The 0..1-depth-range variant, for backends whose NDC z is [0,1] (Metal,
// D3D, Vulkan — bgfx caps `homogeneousDepth == false`). The x/y rows are
// IDENTICAL to shadowFaceProj's, so every UV property proven against the
// NO variant holds for this one; only clip z differs, and clip z is used
// solely to clip and to order fragments — the stored value is linear
// distance from the fragment stage. Render with the wrong variant and the
// backend clips half the depth range away (geometry silently missing from
// faces), which is exactly the convention trap the linear-distance design
// keeps OUT of the comparison.
inline Matrix4 shadowFaceProjZO(float nearZ, float farZ) {
    return glm::perspectiveRH_ZO(glm::radians(90.0f), 1.0f, nearZ, farZ);
}

// Conservative face-culling predicate: can `face` of a light at `lightPos`
// with reach `reach` possibly see a sphere (center, radius)? Used to skip
// rendering faces whose 90° frustum misses every cell bounding sphere
// within the light's reach — a torch on a wall never renders the face
// pointing INTO the wall. Conservative by construction: plane distances
// compare against -radius and the reach test allows the sphere to straddle
// the boundary, so a false return PROVES no point of the sphere lands on
// the face. No near-plane term: anything nearer than the near plane still
// occludes beyond it, and culling it would punch a hole in the map.
inline bool shadowFaceSeesSphere(const Vector3 &lightPos, int face,
                                 const Vector3 &center, float radius,
                                 float reach) {
    const Vector3 d = center - lightPos;
    if (glm::length(d) - radius > reach) return false;
    const Vector3 fwd = shadowFaceDir(face);
    const Vector3 up = shadowFaceUp(face);
    const Vector3 right = glm::normalize(glm::cross(fwd, up));
    const Vector3 realUp = glm::cross(right, fwd);
    // 90° frustum: the four side planes pass through the light with inward
    // unit normals (fwd ± right)/√2 and (fwd ± realUp)/√2.
    constexpr float kInvSqrt2 = 0.70710678f;
    if (glm::dot(d, fwd + right) * kInvSqrt2 < -radius) return false;
    if (glm::dot(d, fwd - right) * kInvSqrt2 < -radius) return false;
    if (glm::dot(d, fwd + realUp) * kInvSqrt2 < -radius) return false;
    if (glm::dot(d, fwd - realUp) * kInvSqrt2 < -radius) return false;
    return true;
}

// Where `point` lands on `face`, in [0,1]² — the lookup the S4 shader
// mirrors. Returns false when the point projects outside the face (caller
// picked the wrong face, or the point sits behind the light plane).
inline bool shadowFaceUV(const Vector3 &lightPos, int face,
                         const Vector3 &point, float &u, float &v) {
    const Vector3 d = point - lightPos;
    const Vector3 fwd = shadowFaceDir(face);
    const float z = glm::dot(d, fwd);
    if (z <= 1e-6f) return false;
    const Vector3 up = shadowFaceUp(face);
    const Vector3 right = glm::normalize(glm::cross(fwd, up));
    const Vector3 realUp = glm::cross(right, fwd);
    // 90° FOV: half-extent at depth z is exactly z.
    const float x = glm::dot(d, right) / z;
    const float y = glm::dot(d, realUp) / z;
    u = x * 0.5f + 0.5f;
    v = y * 0.5f + 0.5f;
    return u >= 0.0f && u <= 1.0f && v >= 0.0f && v <= 1.0f;
}

// The value a face texel stores for `point`: distance from the light,
// normalised by the light's reach. The shadow test compares this against
// the sampled value plus a bias.
inline float shadowFaceDistance(const Vector3 &lightPos, const Vector3 &point,
                                float reach) {
    return reach > 0.0f ? glm::length(point - lightPos) / reach : 1.0f;
}

// Atlas tiling: face `f` of pool slot `slot` in a square atlas of
// `tilesPerRow` × tilesPerRow tiles, each `tileSize` texels. Returns the
// tile origin in texels. Slot layout is caller policy (LRU pool); this only
// fixes the geometry so CPU and shader agree.
inline void shadowAtlasTileOrigin(int slot, int face, int tilesPerRow,
                                  int tileSize, int &outX, int &outY) {
    const int tile = slot * kShadowFaceCount + face;
    outX = (tile % tilesPerRow) * tileSize;
    outY = (tile / tilesPerRow) * tileSize;
}

} // namespace Darkness
