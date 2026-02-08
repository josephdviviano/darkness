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

#ifndef __VECTOR2_H
#define __VECTOR2_H

#include <cmath>

namespace Opde {

struct Vector2 {
    float x, y;

    Vector2() : x(0), y(0) {}
    Vector2(float x_, float y_) : x(x_), y(y_) {}

    Vector2 operator+(const Vector2 &rhs) const {
        return Vector2(x + rhs.x, y + rhs.y);
    }

    Vector2 operator-(const Vector2 &rhs) const {
        return Vector2(x - rhs.x, y - rhs.y);
    }

    Vector2 operator*(float s) const {
        return Vector2(x * s, y * s);
    }

    bool operator==(const Vector2 &rhs) const {
        return x == rhs.x && y == rhs.y;
    }

    bool operator!=(const Vector2 &rhs) const {
        return !(*this == rhs);
    }

    float dotProduct(const Vector2 &rhs) const {
        return x * rhs.x + y * rhs.y;
    }

    float length() const {
        return std::sqrt(x * x + y * y);
    }

    static const Vector2 ZERO;
};

} // namespace Opde

#endif
