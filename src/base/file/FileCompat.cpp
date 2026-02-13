/******************************************************************************
 *
 *    This file is part of openDarkEngine project
 *    Copyright (C) 2009 openDarkEngine team
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
 *	  $Id$
 *
 *****************************************************************************/

/** @file FileCompat.cpp
 * @brief A various utility methods for File class usage - implementation
 */

#include "config.h"

#include "FileCompat.h"
#include "DarknessMath.h"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Darkness {
// Vector2
File &operator<<(File &st, const Vector2 &val) {
    st << val.x << val.y;
    return st;
}

File &operator>>(File &st, Vector2 &val) {
    st >> val.x >> val.y;
    return st;
}

// Vector3
File &operator<<(File &st, const Vector3 &val) {
    st << val.x << val.y << val.z;
    return st;
}

File &operator>>(File &st, Vector3 &val) {
    st >> val.x >> val.y >> val.z;
    return st;
}

// Plane
File &operator<<(File &st, const Plane &val) {
    st << val.normal << val.d;
    return st;
}

File &operator>>(File &st, Plane &val) {
    st >> val.normal >> val.d;
    return st;
}

File &operator<<(File &st, const Quaternion &val) {
    int16_t xi, yi, zi;

    // Convert quaternion → rotation matrix → Euler angles (ZYX order)
    // GLM's eulerAngle functions work with mat4, so we go quat → mat3 → mat4
    Matrix3 m3 = glm::mat3_cast(val);
    Matrix4 m4(m3);

    float x_rad, y_rad, z_rad;
    glm::extractEulerAngleZYX(m4, z_rad, y_rad, x_rad);

    xi = static_cast<int16_t>(x_rad * 32768 / M_PI);
    yi = static_cast<int16_t>(y_rad * 32768 / M_PI);
    zi = static_cast<int16_t>(z_rad * 32768 / M_PI);

    st << xi << yi << zi;

    return st;
}

File &operator>>(File &st, Quaternion &val) {
    int16_t xi, yi, zi;
    float x, y, z;

    st >> xi >> yi >> zi;

    x = ((float)(xi) / 32768) * static_cast<float>(M_PI);
    y = ((float)(yi) / 32768) * static_cast<float>(M_PI);
    z = ((float)(zi) / 32768) * static_cast<float>(M_PI);

    // Convert Euler angles (ZYX order) → rotation matrix → quaternion
    // GLM's eulerAngleZYX returns mat4, cast to mat3 for quat_cast
    Matrix4 m4 = glm::eulerAngleZYX(z, y, x);
    Matrix3 m3(m4);
    val = glm::quat_cast(m3);

    return st;
}

} // namespace Darkness
