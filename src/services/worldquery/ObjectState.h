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

// ObjectState.h — Mutable per-object runtime state
//
// Objects in the Dark Engine database (P$Position etc.) are loaded once at
// mission start and are conceptually immutable. When game systems need to
// move objects at runtime (doors opening, platforms translating, tweqs
// animating), they create an ObjectState entry that overrides the static
// property data.
//
// Two consumers read ObjectState:
//   1. The renderer — checks for a transform override before using the
//      static ObjectPlacement position/rotation from the .mis file.
//   2. IWorldQuery (ObjSysWorldState) — getPosition()/getOrientation()
//      return the override when present, falling back to PropertyService.
//
// Game systems (DoorSystem, TweqSystem, MovingTerrainSystem) write to
// ObjectState via ObjectStateMap::set(). All access is main-thread only.
//
// ObjectState stores position + orientation as float angles (heading, pitch,
// bank in radians) for direct use by the renderer's buildModelMatrix(), plus
// a pre-computed Quaternion for IWorldQuery consumers. When setting angles,
// the quaternion is automatically derived. When setting a quaternion directly,
// the angles are derived.

#pragma once

#include <cstdint>
#include <cmath>
#include <unordered_map>

#include "DarknessMath.h"

namespace Darkness {

/// Flags describing mutable object state.
enum ObjectStateFlags : uint32_t {
    kObjStateActive    = 1 << 0,   // Object is actively simulated (door opening, etc.)
    kObjStateSleeping  = 1 << 1,   // Object was active but is now at rest
    kObjStateDestroyed = 1 << 2,   // Object has been destroyed (skip rendering + queries)
    kObjStateHidden    = 1 << 3,   // Temporarily invisible (blink tweq, etc.)
};

/// Per-object mutable runtime state. Overrides the static P$Position when
/// present. Only objects that move at runtime get an ObjectState entry —
/// most objects in a mission have no entry and use their static data.
struct ObjectState {
    // ── Transform (canonical representation) ──
    Vector3  position  = {0.0f, 0.0f, 0.0f};
    float    heading   = 0.0f;   // radians, Z rotation
    float    pitch     = 0.0f;   // radians, Y rotation
    float    bank      = 0.0f;   // radians, X rotation
    Vector3  scale     = {1.0f, 1.0f, 1.0f};

    // Pre-computed quaternion for IWorldQuery consumers.
    // Updated whenever heading/pitch/bank change.
    Quaternion orientation = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);

    // ── State flags ──
    uint32_t flags = kObjStateActive;

    // ── Convenience setters ──

    /// Set position + angles (in radians). Recomputes quaternion.
    void setTransform(const Vector3 &pos, float h, float p, float b) {
        position = pos;
        heading  = h;
        pitch    = p;
        bank     = b;
        recomputeQuaternion();
    }

    /// Set position only, keep current orientation.
    void setPosition(const Vector3 &pos) { position = pos; }

    /// Set angles in radians. Recomputes quaternion.
    void setAngles(float h, float p, float b) {
        heading = h;
        pitch   = p;
        bank    = b;
        recomputeQuaternion();
    }

    /// Initialize from static ObjectPlacement data (binary radians int16_t).
    /// Used when creating an ObjectState for an object that was previously static.
    void initFromBinaryRadians(float x, float y, float z,
                                int16_t bh, int16_t bp, int16_t bb,
                                float sx = 1.0f, float sy = 1.0f, float sz = 1.0f) {
        static constexpr float kAngScale = 2.0f * 3.14159265f / 65536.0f;
        position = Vector3(x, y, z);
        heading  = static_cast<float>(bh) * kAngScale;
        pitch    = static_cast<float>(bp) * kAngScale;
        bank     = static_cast<float>(bb) * kAngScale;
        scale    = Vector3(sx, sy, sz);
        recomputeQuaternion();
    }

private:
    /// Recompute quaternion from Euler angles.
    /// Dark Engine rotation order: M = Rz(heading) * Ry(pitch) * Rx(bank)
    void recomputeQuaternion() {
        // glm::eulerAngleZYX produces the same rotation order
        Matrix4 rotMat = glm::eulerAngleZYX(heading, pitch, bank);
        orientation = glm::quat_cast(rotMat);
    }
};

/// Map of object ID → mutable state. Only objects with runtime state changes
/// have entries. Main-thread only — no synchronization needed.
class ObjectStateMap {
public:
    /// Check if an object has a runtime state override.
    bool has(int32_t objID) const {
        return mStates.find(objID) != mStates.end();
    }

    /// Get mutable reference to an object's state. Creates a default entry
    /// if none exists.
    ObjectState &get(int32_t objID) { return mStates[objID]; }

    /// Get const pointer to an object's state, or nullptr if no override exists.
    const ObjectState *tryGet(int32_t objID) const {
        auto it = mStates.find(objID);
        return (it != mStates.end()) ? &it->second : nullptr;
    }

    /// Set an object's runtime state. Overwrites any existing entry.
    void set(int32_t objID, const ObjectState &state) { mStates[objID] = state; }

    /// Remove an object's runtime state override (reverts to static P$Position).
    void remove(int32_t objID) { mStates.erase(objID); }

    /// Remove all entries (e.g., on mission unload).
    void clear() { mStates.clear(); }

    /// Number of objects with runtime state overrides.
    size_t size() const { return mStates.size(); }

    /// Iterator access for systems that need to scan all active objects.
    auto begin() const { return mStates.begin(); }
    auto end()   const { return mStates.end(); }
    auto begin()       { return mStates.begin(); }
    auto end()         { return mStates.end(); }

private:
    std::unordered_map<int32_t, ObjectState> mStates;
};

} // namespace Darkness
