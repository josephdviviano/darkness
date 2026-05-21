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

// DoorAudioGeometry.h — Door geometry hand-off struct shared between
// DoorSystem (in src/services/sim) and AudioService (in src/services/audio).
//
// Lives in its own small header so AudioService.cpp can consume it without
// dragging in DoorSystem.h's transitive dependency on BinMeshParser.h, which
// lives in src/main and is not visible to DarknessServices translation units.

#pragma once

#include <cstdint>
#include <vector>

#include "DarknessMath.h"

namespace Darkness {

// Per-door audio geometry handed to AudioService::registerDoorGeometry at
// scene-build time. The vertices are in the door's MODEL-LOCAL frame (origin
// at the door slab center), scaled to final world dimensions — the world
// transform should NOT re-apply baseScale. `worldTransform` is the door's
// current pose (T * R * pivot logic, without scale) and is what the audio
// instanced mesh is created with; subsequent updates flow through
// AudioService::setDoorTransform.
struct DoorAudioGeometry {
    int32_t objID = 0;
    std::vector<float>   localVertices;   ///< 3 floats per vertex
    std::vector<int32_t> indices;         ///< 3 per triangle
    Matrix4 worldTransform = Matrix4(1.0f);
};

} // namespace Darkness
