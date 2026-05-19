/******************************************************************************
 *
 *    This file is part of the Darkness project (Dark Engine port).
 *    Copyright (C) 2026 Darkness team
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *****************************************************************************/

// CellGraph has been retired. An earlier migration attempt routed sound
// propagation through a BSP-cell graph; the cells turned out to be
// erratically placed (BSP leaves are convex but their centroids can fall
// near boundaries or above the listener), producing unnatural sound
// paths. Sound propagation is back on the ROOM_DB portal graph BFS in
// RoomService::propagateSoundPath.
//
// This stub file remains in place so external clones of the repo that
// have an older build cache don't see stale include-paths during the
// next rebuild. The header declares nothing.

#pragma once
