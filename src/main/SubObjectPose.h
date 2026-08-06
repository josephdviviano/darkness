/******************************************************************************
 *
 *    This file is part of the Darkness engine
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

// SubObjectPose.h — pose LGMD sub-objects (model "joints") at draw time.
//
// An LGMD model is a tree of rigid parts. Each part stores its geometry in its
// own local space plus a frame (`rot`, `axle`) placing that space inside its
// parent's. `movement` says how the joint value moves the part:
//
//     0  static  — the model root; never moves
//     1  rotate  — turn about the part's local +X axis, joint value in RADIANS
//     2  slide   — translate along the part's local +X axis, in world units
//
// THE HINGE AXIS IS LOCAL +X. Established from the shipped models, where the
// artist always orients `rot` so local X lands on the intended axis — five
// independent semantic confirmations, each of which would be absurd under a
// local-Y or local-Z reading:
//
//   DOOR17SW  @s00bb  a valve wheel, radially symmetric about local X
//                     (y/z extents 1.12 each, ±0.56), range [0, 2*pi]
//   MECLOCK2          14 clock hands, every one planar with local X as the
//                     plane normal (x extent exactly 0.00)
//   CHESTLOC  @s01bb  the lid: local X maps to a line through the chest's
//                     top-REAR edge (axle y = 0.401 vs the body's y-max 0.400)
//   CAMERA    @s00ss  local X maps to world +Z — a security camera panning
//                     about the vertical, range [0, 3*pi/2]
//   TU_F      @s00top a slide joint whose local X maps to world +Z: a turret
//                     that rises 2.1 units out of its housing
//
// RANGES ARE NOT APPLIED. `minRange`/`maxRange` are authored travel limits, but
// they are not a runtime clamp: 5 stock parts carry an interval that EXCLUDES
// zero (MECLOCK2 @s12ll is [-1.745, -13.352]; the CAM* family is
// [-0.314, -1.571]), so clamping would displace those parts in the rest pose,
// and 4 more author [0, 0] on a slide joint, which clamping would freeze
// forever. Animation limits belong to the tweq config (CfgTweqJo low/high),
// which is per-object and does bound the value. We keep the mesh range in
// BinSubObject for diagnostics and future use.

#pragma once

#include <cmath>
#include <cstdio>
#include <vector>

#include "DarknessMath.h"
#include <glm/gtc/matrix_transform.hpp>

#include "BinMeshParser.h"

namespace Darkness {

/// The joint value moves the part about/along its own local +X axis.
static const Vector3 kJointAxis = Vector3(1.0f, 0.0f, 0.0f);

/// Movement codes from SubObjectHeader::movement.
enum SubObjMovement : uint8_t {
    kSubObjStatic = 0,
    kSubObjRotate = 1,
    kSubObjSlide  = 2,
};

/// Pick the joint value driving a sub-object out of an object's joint array.
/// Rotate joints are stored in DEGREES (P$JointPos and CfgTweqJo both use
/// degrees; the mesh ranges are radians) and converted here. Slide joints are
/// world units and pass through. Parts whose joint slot is outside the array —
/// stock models index up to 13, the property only stores 6 — read as 0, which
/// is the rest pose.
inline float subObjectJointValue(const BinSubObject &s,
                                 const float *joints, int jointCount) {
    if (s.movement == kSubObjStatic) return 0.0f;
    if (!joints || s.jointIdx < 0 || s.jointIdx >= jointCount) return 0.0f;

    const float raw = joints[s.jointIdx];
    if (s.movement == kSubObjRotate)
        return raw * (3.14159265358979f / 180.0f);
    return raw;
}

/// Build a sub-object's transform into its PARENT's space for a given joint
/// value. Zero gives the rest pose.
inline Matrix4 subObjectLocalMatrix(const BinSubObject &s, float jointValue,
                                    bool isRoot) {
    // The root's `rot` is all zeros on disk (verified across all 1782 stock
    // models), so it is a placeholder, not a basis — the root frame is the
    // model frame.
    if (isRoot) return Matrix4(1.0f);

    Matrix3 basis = glm::make_mat3(s.rot);
    if (std::fabs(glm::determinant(basis)) < 1e-6f) {
        // A degenerate basis would collapse the part to a plane or a point.
        // No stock model does this; a hand-authored one might.
        static int warnCount = 0;
        if (warnCount++ < 10)
            std::fprintf(stderr, "[FALLBACK] SubObjectPose: sub-object '%s' has a "
                         "degenerate rot basis (det~0) — using identity, part will "
                         "render unrotated\n", s.name);
        basis = Matrix3(1.0f);
    }

    Matrix4 local = glm::translate(Matrix4(1.0f),
                                   Vector3(s.axle[0], s.axle[1], s.axle[2]))
                    * Matrix4(basis);

    if (s.movement == kSubObjRotate)
        return local * glm::rotate(Matrix4(1.0f), jointValue, kJointAxis);
    if (s.movement == kSubObjSlide)
        return local * glm::translate(Matrix4(1.0f), kJointAxis * jointValue);
    return local;
}

/// Compose model-space matrices for every sub-object of a mesh, walking the
/// child/next tree so a part inherits its whole ancestor chain.
///
/// Pass `joints = nullptr` for the rest pose. `out` is resized to
/// `mesh.subObjects.size()`.
///
/// This composition is what the old baked path got wrong beyond animation: it
/// applied each part's OWN frame only, so the 24 stock models nested two or
/// more levels deep (CAMERA, TU_F, DPUMP, PUMPER, the lock family, loco,
/// testflag) drew their deeper parts detached from the model.
inline void composeSubObjectMatrices(const ParsedBinMesh &mesh,
                                     const float *joints, int jointCount,
                                     std::vector<Matrix4> &out) {
    const int n = static_cast<int>(mesh.subObjects.size());
    out.assign(static_cast<size_t>(n < 0 ? 0 : n), Matrix4(1.0f));
    if (n <= 0) return;

    std::vector<bool> visited(static_cast<size_t>(n), false);

    // Explicit stack rather than recursion: FM models can nest arbitrarily and
    // a malformed child/next pair must not blow the C++ stack.
    std::vector<std::pair<int, Matrix4>> stack;
    stack.emplace_back(0, Matrix4(1.0f));

    while (!stack.empty()) {
        auto [idx, parentM] = stack.back();
        stack.pop_back();

        // Walk this sibling chain under the same parent matrix.
        while (idx >= 0 && idx < n && !visited[static_cast<size_t>(idx)]) {
            visited[static_cast<size_t>(idx)] = true;
            const BinSubObject &s = mesh.subObjects[static_cast<size_t>(idx)];

            const float jv = subObjectJointValue(s, joints, jointCount);
            out[static_cast<size_t>(idx)] =
                parentM * subObjectLocalMatrix(s, jv, idx == 0);

            if (s.child >= 0)
                stack.emplace_back(s.child, out[static_cast<size_t>(idx)]);
            idx = s.next;
        }
    }

    // Unreachable parts keep the identity they were assigned above; they draw
    // in model space, which is at least visible rather than silently dropped.
    for (int i = 0; i < n; ++i) {
        if (visited[static_cast<size_t>(i)]) continue;
        static int warnCount = 0;
        if (warnCount++ < 10)
            std::fprintf(stderr, "[FALLBACK] SubObjectPose: sub-object %d ('%s') is "
                         "not reachable from the model root — drawing it untransformed\n",
                         i, mesh.subObjects[static_cast<size_t>(i)].name);
    }
}

/// Model-space position of the attachment point carrying the given engine
/// index, if this mesh has one. Matches on BinVHot::index, never on array
/// position — see the note on BinVHot for why those differ.
///
/// `joints` poses the owning part; pass nullptr for the rest pose. The point is
/// pushed through the owning sub-object's full ancestor chain, so a vhot on a
/// moving part lands where that part currently is rather than where its local
/// space would put it at the model origin.
inline bool findVHotModelSpace(const ParsedBinMesh &mesh, uint32_t index,
                               const float *joints, int jointCount,
                               Vector3 &out) {
    for (const auto &v : mesh.vhots) {
        if (v.index != index) continue;

        const Vector3 local(v.point[0], v.point[1], v.point[2]);
        if (v.subObj <= 0 || v.subObj >= static_cast<int>(mesh.subObjects.size())) {
            // Root-owned (or a model with no sub-object table): the part's
            // frame IS the model frame, so the stored point needs no transform.
            out = local;
            return true;
        }

        std::vector<Matrix4> mats;
        composeSubObjectMatrices(mesh, joints, jointCount, mats);
        out = Vector3(mats[static_cast<size_t>(v.subObj)] * glm::vec4(local, 1.0f));
        return true;
    }
    return false;
}

/// True when a mesh has parts that need a per-submesh transform at all. Single
/// part models — 1541 of the 1782 stock models — skip the whole path.
inline bool meshHasSubObjectTransforms(const ParsedBinMesh &mesh) {
    return mesh.subObjects.size() > 1;
}

/// True when any of a mesh's parts would actually move for these joint values.
/// Lets the renderer reuse a model's precomputed rest matrices instead of
/// recomposing per frame for the overwhelmingly common at-rest case.
inline bool meshJointsAtRest(const ParsedBinMesh &mesh,
                             const float *joints, int jointCount) {
    if (!joints) return true;
    for (const auto &s : mesh.subObjects) {
        if (s.movement == kSubObjStatic) continue;
        if (s.jointIdx < 0 || s.jointIdx >= jointCount) continue;
        if (std::fabs(joints[s.jointIdx]) > 1e-6f) return false;
    }
    return true;
}

} // namespace Darkness
