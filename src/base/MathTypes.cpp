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

#include "Vector3.h"
#include "Vector2.h"
#include "Quaternion.h"
#include "Matrix3.h"

namespace Darkness {

// Vector3 constants
const Vector3 Vector3::ZERO(0.0f, 0.0f, 0.0f);
const Vector3 Vector3::UNIT_X(1.0f, 0.0f, 0.0f);
const Vector3 Vector3::UNIT_Y(0.0f, 1.0f, 0.0f);
const Vector3 Vector3::UNIT_Z(0.0f, 0.0f, 1.0f);

// Vector2 constants
const Vector2 Vector2::ZERO(0.0f, 0.0f);

// Quaternion constants
const Quaternion Quaternion::IDENTITY(1.0f, 0.0f, 0.0f, 0.0f);

void Quaternion::ToRotationMatrix(Matrix3 &rot) const {
    float tx  = 2.0f * x;
    float ty  = 2.0f * y;
    float tz  = 2.0f * z;
    float twx = tx * w;
    float twy = ty * w;
    float twz = tz * w;
    float txx = tx * x;
    float txy = ty * x;
    float txz = tz * x;
    float tyy = ty * y;
    float tyz = tz * y;
    float tzz = tz * z;

    rot.m[0][0] = 1.0f - (tyy + tzz);
    rot.m[0][1] = txy - twz;
    rot.m[0][2] = txz + twy;
    rot.m[1][0] = txy + twz;
    rot.m[1][1] = 1.0f - (txx + tzz);
    rot.m[1][2] = tyz - twx;
    rot.m[2][0] = txz - twy;
    rot.m[2][1] = tyz + twx;
    rot.m[2][2] = 1.0f - (txx + tyy);
}

void Quaternion::FromRotationMatrix(const Matrix3 &rot) {
    float trace = rot.m[0][0] + rot.m[1][1] + rot.m[2][2];

    if (trace > 0.0f) {
        float s = std::sqrt(trace + 1.0f);
        w = s * 0.5f;
        s = 0.5f / s;
        x = (rot.m[2][1] - rot.m[1][2]) * s;
        y = (rot.m[0][2] - rot.m[2][0]) * s;
        z = (rot.m[1][0] - rot.m[0][1]) * s;
    } else {
        int i = 0;
        if (rot.m[1][1] > rot.m[0][0]) i = 1;
        if (rot.m[2][2] > rot.m[i][i]) i = 2;

        static const int next[3] = {1, 2, 0};
        int j = next[i];
        int k = next[j];

        float s = std::sqrt(rot.m[i][i] - rot.m[j][j] - rot.m[k][k] + 1.0f);
        float q[4];
        q[i] = s * 0.5f;
        if (s != 0.0f) s = 0.5f / s;
        q[3] = (rot.m[k][j] - rot.m[j][k]) * s;
        q[j] = (rot.m[j][i] + rot.m[i][j]) * s;
        q[k] = (rot.m[k][i] + rot.m[i][k]) * s;

        x = q[0]; y = q[1]; z = q[2]; w = q[3];
    }
}

} // namespace Darkness
