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

#ifndef __QUATERNION_H
#define __QUATERNION_H

#include <cmath>

namespace Darkness {

struct Matrix3;

struct Quaternion {
    float w, x, y, z;

    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(float w_, float x_, float y_, float z_)
        : w(w_), x(x_), y(y_), z(z_) {}

    Quaternion operator+(const Quaternion &rhs) const {
        return Quaternion(w + rhs.w, x + rhs.x, y + rhs.y, z + rhs.z);
    }

    Quaternion operator*(const Quaternion &rhs) const {
        return Quaternion(
            w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
            w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
            w * rhs.y + y * rhs.w + z * rhs.x - x * rhs.z,
            w * rhs.z + z * rhs.w + x * rhs.y - y * rhs.x
        );
    }

    bool operator==(const Quaternion &rhs) const {
        return w == rhs.w && x == rhs.x && y == rhs.y && z == rhs.z;
    }

    bool operator!=(const Quaternion &rhs) const {
        return !(*this == rhs);
    }

    float length() const {
        return std::sqrt(w * w + x * x + y * y + z * z);
    }

    Quaternion normalisedCopy() const {
        float len = length();
        if (len > 0.0f) {
            float inv = 1.0f / len;
            return Quaternion(w * inv, x * inv, y * inv, z * inv);
        }
        return *this;
    }

    void ToRotationMatrix(Matrix3 &rot) const;
    void FromRotationMatrix(const Matrix3 &rot);

    static const Quaternion IDENTITY;
};

} // namespace Darkness

#endif
