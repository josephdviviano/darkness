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

#ifndef __MATRIX4_H
#define __MATRIX4_H

#include <cmath>
#include <cstring>
#include "Vector3.h"

namespace Opde {

struct Matrix4 {
    float m[4][4];

    Matrix4() { std::memset(m, 0, sizeof(m)); }

    static Matrix4 identity() {
        Matrix4 r;
        r.m[0][0] = 1.0f;
        r.m[1][1] = 1.0f;
        r.m[2][2] = 1.0f;
        r.m[3][3] = 1.0f;
        return r;
    }

    static Matrix4 translation(float x, float y, float z) {
        Matrix4 r = identity();
        r.m[0][3] = x;
        r.m[1][3] = y;
        r.m[2][3] = z;
        return r;
    }

    // Right-handed perspective projection
    // fovY in radians, aspect = width/height
    static Matrix4 perspective(float fovY, float aspect, float nearZ, float farZ) {
        float tanHalfFov = std::tan(fovY * 0.5f);
        Matrix4 r;
        r.m[0][0] = 1.0f / (aspect * tanHalfFov);
        r.m[1][1] = 1.0f / tanHalfFov;
        r.m[2][2] = farZ / (nearZ - farZ);
        r.m[2][3] = (nearZ * farZ) / (nearZ - farZ);
        r.m[3][2] = -1.0f;
        return r;
    }

    // Right-handed look-at view matrix
    static Matrix4 lookAt(const Vector3 &eye, const Vector3 &target, const Vector3 &up) {
        Vector3 f = (target - eye).normalisedCopy();
        Vector3 r = f.crossProduct(up).normalisedCopy();
        Vector3 u = r.crossProduct(f);

        Matrix4 result = identity();
        result.m[0][0] = r.x;
        result.m[0][1] = r.y;
        result.m[0][2] = r.z;
        result.m[0][3] = -r.dotProduct(eye);
        result.m[1][0] = u.x;
        result.m[1][1] = u.y;
        result.m[1][2] = u.z;
        result.m[1][3] = -u.dotProduct(eye);
        result.m[2][0] = -f.x;
        result.m[2][1] = -f.y;
        result.m[2][2] = -f.z;
        result.m[2][3] = f.dotProduct(eye);
        return result;
    }

    Matrix4 operator*(const Matrix4 &rhs) const {
        Matrix4 r;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                r.m[i][j] = m[i][0] * rhs.m[0][j] +
                             m[i][1] * rhs.m[1][j] +
                             m[i][2] * rhs.m[2][j] +
                             m[i][3] * rhs.m[3][j];
        return r;
    }

    const float *data() const { return &m[0][0]; }
};

} // namespace Opde

#endif
