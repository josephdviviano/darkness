/******************************************************************************
 *
 *    This file is part of Darkness engine
 *    Copyright (C) 2005-2009 openDarkEngine team
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

/// @file DarknessMath.h
/// @brief Central math header — GLM-backed type aliases for the Darkness engine.
///
/// All engine code should include this header (or one of the legacy redirects)
/// for Vector2, Vector3, Quaternion, Matrix3, Matrix4, and Plane types.
/// The underlying implementation is GLM 1.0 (MIT license).
///
/// GLM reference: https://github.com/g-truc/glm

#pragma once

// GLM experimental extensions (gtx/) are stable and well-tested; the define
// silences the "may change in the future" compile-time warning.
#define GLM_ENABLE_EXPERIMENTAL

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace Darkness {

// --- Core type aliases ---
// These replace the former custom Darkness math types with GLM equivalents.
// Field access (.x, .y, .z, .w) and all arithmetic operators are unchanged.

using Vector2    = glm::vec2;
using Vector3    = glm::vec3;
using Quaternion = glm::quat;   // Constructor order: (w, x, y, z) — matches old Darkness convention
using Matrix3    = glm::mat3;
using Matrix4    = glm::mat4;

// --- Plane ---
// GLM has no Plane type. We keep our own, now using glm::vec3 for the normal.
// Plane equation: dot(normal, point) + d = 0
// Positive side = same direction as normal, negative side = opposite.

struct Plane {
    enum Side {
        NO_SIDE,
        POSITIVE_SIDE,
        NEGATIVE_SIDE,
        BOTH_SIDE
    };

    glm::vec3 normal;
    float d;

    Plane() : normal(0.0f, 0.0f, 0.0f), d(0.0f) {}
    Plane(const glm::vec3 &normal_, float d_) : normal(normal_), d(d_) {}
    Plane(float a, float b, float c, float d_) : normal(a, b, c), d(d_) {}

    float getDistance(const glm::vec3 &point) const {
        return glm::dot(normal, point) + d;
    }

    Side getSide(const glm::vec3 &point) const {
        float dist = getDistance(point);
        if (dist < 0.0f)
            return NEGATIVE_SIDE;
        if (dist > 0.0f)
            return POSITIVE_SIDE;
        return NO_SIDE;
    }
};

} // namespace Darkness
