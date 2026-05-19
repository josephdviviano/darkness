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

// SourceTreeCache has been retired alongside CellGraph (see CellGraph.h
// for context). Sound propagation now re-runs BFS over the ROOM_DB
// portal graph on each call; per-source caching may return in a future
// PR if profiling shows the BFS dominates a frame, but Phase 1 of the
// migration found per-call BFS over ~200 rooms cheap enough to skip
// caching entirely.

#pragma once
