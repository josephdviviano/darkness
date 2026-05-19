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

// ProbeGraph has been retired. The probe-graph subsystem (~1500 LOC of
// probe placement, edge-build, connectivity-repair, A* / Yen and a
// per-mission on-disk cache) was the previous sound-propagation backend.
// Sound propagation now runs ROOM_DB portal-graph BFS per call in
// RoomService::propagateSoundPath.
//
// This stub file remains in place so external clones of the repo that
// have an older build cache don't see stale include-paths during the
// next rebuild. The header intentionally declares nothing; any
// remaining `#include "room/ProbeGraph.h"` is a no-op.

#pragma once
