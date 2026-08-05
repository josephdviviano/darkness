/******************************************************************************
 *
 *    This file is part of the Darkness engine
 *    Copyright (C) 2024-2026 Darkness contributors
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

// ObjectVisibility.h — which objects the renderer actually drew.
//
// The tweq system's update-rate gates need an on-screen test: 184 of the 456
// shipped tweq configs are gated to run only while their object is visible,
// and 9 more run only while it is NOT. Only the renderer knows, so it records
// what it drew and the sim asks.
//
// The contract the sim relies on (TweqSystem::VisibilityQuery) is "visible OR
// not renderable at all". That second half is the important one: an invisible
// marker carrying a delete countdown — a waypoint, a particle anchor — must not
// be frozen forever by a gate whose entire premise is that the player can see
// it. Objects the renderer has no instance for are therefore always "visible",
// which makes the gate a no-op for them rather than a trap.

#pragma once

#include <cstdint>
#include <unordered_map>
#include <unordered_set>

namespace Darkness {

class ObjectVisibility {
public:
    /// Declare the set of objects the renderer can draw at all. Anything
    /// outside it is treated as permanently visible, so the gate never applies
    /// to objects that could not be on screen in the first place.
    void setRenderable(std::unordered_set<int32_t> ids) {
        mRenderable = std::move(ids);
    }

    /// A runtime-spawned object becomes renderable when it gains an instance.
    void addRenderable(int32_t objID) { mRenderable.insert(objID); }

    /// Start a new frame. Call once per frame before any draw.
    void beginFrame() { ++mFrame; }

    /// Record that an object was submitted for drawing this frame.
    void markDrawn(int32_t objID) { mLastDrawn[objID] = mFrame; }

    /// Was this object on screen recently?
    ///
    /// "Recently" spans two frames, not one: the sim step runs BEFORE the draw
    /// within a frame, so at the moment a tweq asks, the freshest stamp
    /// available is the previous frame's. A one-frame window would report every
    /// visible object as off-screen on the frame it first appears and stutter
    /// them permanently.
    bool wasVisible(int32_t objID) const {
        if (mRenderable.find(objID) == mRenderable.end())
            return true;   // not renderable — the gate does not apply

        auto it = mLastDrawn.find(objID);
        if (it == mLastDrawn.end())
            return false;
        return (mFrame - it->second) <= 1;
    }

    /// Drop an object that has left the world, so its stamp cannot be read by
    /// a later object reusing its ID.
    void forget(int32_t objID) {
        mLastDrawn.erase(objID);
        mRenderable.erase(objID);
    }

    size_t renderableCount() const { return mRenderable.size(); }

private:
    std::unordered_set<int32_t> mRenderable;
    std::unordered_map<int32_t, uint32_t> mLastDrawn;
    uint32_t mFrame = 0;
};

} // namespace Darkness
