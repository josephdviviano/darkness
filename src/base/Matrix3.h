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

#ifndef __MATRIX3_H
#define __MATRIX3_H

#include <cmath>

namespace Opde {

struct Matrix3 {
    float m[3][3];

    Matrix3() {
        m[0][0] = 1; m[0][1] = 0; m[0][2] = 0;
        m[1][0] = 0; m[1][1] = 1; m[1][2] = 0;
        m[2][0] = 0; m[2][1] = 0; m[2][2] = 1;
    }

    /// Construct from Euler angles in ZYX order (bank, pitch, heading)
    void FromEulerAnglesZYX(float z_rad, float y_rad, float x_rad) {
        float cx = std::cos(x_rad), sx = std::sin(x_rad);
        float cy = std::cos(y_rad), sy = std::sin(y_rad);
        float cz = std::cos(z_rad), sz = std::sin(z_rad);

        m[0][0] = cy * cz;
        m[0][1] = cy * sz;
        m[0][2] = -sy;
        m[1][0] = sx * sy * cz - cx * sz;
        m[1][1] = sx * sy * sz + cx * cz;
        m[1][2] = sx * cy;
        m[2][0] = cx * sy * cz + sx * sz;
        m[2][1] = cx * sy * sz - sx * cz;
        m[2][2] = cx * cy;
    }

    /// Extract Euler angles in ZYX order (bank, pitch, heading)
    void ToEulerAnglesZYX(float &z_rad, float &y_rad, float &x_rad) const {
        // y = asin(-m[0][2])
        float sy = -m[0][2];
        if (sy >= 1.0f)
            y_rad = static_cast<float>(M_PI / 2.0);
        else if (sy <= -1.0f)
            y_rad = static_cast<float>(-M_PI / 2.0);
        else
            y_rad = std::asin(sy);

        float cy = std::cos(y_rad);
        if (std::fabs(cy) > 1e-6f) {
            x_rad = std::atan2(m[1][2], m[2][2]);
            z_rad = std::atan2(m[0][1], m[0][0]);
        } else {
            // gimbal lock
            x_rad = std::atan2(-m[2][1], m[1][1]);
            z_rad = 0.0f;
        }
    }
};

} // namespace Opde

#endif
