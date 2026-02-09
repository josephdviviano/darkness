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

#ifndef __PLANE_H
#define __PLANE_H

#include "Vector3.h"

namespace Darkness {

struct Plane {
    enum Side {
        NO_SIDE,
        POSITIVE_SIDE,
        NEGATIVE_SIDE,
        BOTH_SIDE
    };

    Vector3 normal;
    float d;

    Plane() : normal(Vector3::ZERO), d(0.0f) {}
    Plane(const Vector3 &normal_, float d_) : normal(normal_), d(d_) {}
    Plane(float a, float b, float c, float d_)
        : normal(a, b, c), d(d_) {}

    float getDistance(const Vector3 &point) const {
        return normal.dotProduct(point) + d;
    }

    Side getSide(const Vector3 &point) const {
        float dist = getDistance(point);
        if (dist < 0.0f)
            return NEGATIVE_SIDE;
        if (dist > 0.0f)
            return POSITIVE_SIDE;
        return NO_SIDE;
    }
};

} // namespace Darkness

#endif
