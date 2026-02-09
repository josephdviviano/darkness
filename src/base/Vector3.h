/******************************************************************************
 *
 *    This file is part of openDarkEngine project
 *    Copyright (C) 2005-2006 openDarkEngine team
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

#ifndef __VECTOR3_H
#define __VECTOR3_H

#include <cmath>

namespace Darkness {

struct Vector3 {
    float x, y, z;

    Vector3() : x(0), y(0), z(0) {}
    Vector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vector3 operator+(const Vector3 &rhs) const {
        return Vector3(x + rhs.x, y + rhs.y, z + rhs.z);
    }

    Vector3 operator-(const Vector3 &rhs) const {
        return Vector3(x - rhs.x, y - rhs.y, z - rhs.z);
    }

    Vector3 operator*(float s) const {
        return Vector3(x * s, y * s, z * s);
    }

    Vector3 &operator+=(const Vector3 &rhs) {
        x += rhs.x; y += rhs.y; z += rhs.z;
        return *this;
    }

    Vector3 &operator-=(const Vector3 &rhs) {
        x -= rhs.x; y -= rhs.y; z -= rhs.z;
        return *this;
    }

    bool operator==(const Vector3 &rhs) const {
        return x == rhs.x && y == rhs.y && z == rhs.z;
    }

    bool operator!=(const Vector3 &rhs) const {
        return !(*this == rhs);
    }

    float dotProduct(const Vector3 &rhs) const {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }

    Vector3 crossProduct(const Vector3 &rhs) const {
        return Vector3(
            y * rhs.z - z * rhs.y,
            z * rhs.x - x * rhs.z,
            x * rhs.y - y * rhs.x
        );
    }

    float length() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    float squaredLength() const {
        return x * x + y * y + z * z;
    }

    Vector3 normalisedCopy() const {
        float len = length();
        if (len > 0.0f) {
            float inv = 1.0f / len;
            return Vector3(x * inv, y * inv, z * inv);
        }
        return *this;
    }

    static const Vector3 ZERO;
    static const Vector3 UNIT_X;
    static const Vector3 UNIT_Y;
    static const Vector3 UNIT_Z;
};

inline Vector3 operator*(float s, const Vector3 &v) {
    return Vector3(s * v.x, s * v.y, s * v.z);
}

} // namespace Darkness

#endif
