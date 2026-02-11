/******************************************************************************
 *
 *    This file is part of the Darkness engine
 *
 *    SpatialIndex — spatial hash grid for efficient entity proximity queries.
 *    Maps 3D world positions to grid cells, enabling O(1)-per-cell radius
 *    and AABB queries. Designed for the ~1000-entity scale typical of Dark
 *    Engine missions.
 *
 *    Cell size default (32 world units) keeps radius queries to manageable
 *    3x3x3 cell neighborhoods for common AI/acoustic ranges (50-100 units).
 *
 *****************************************************************************/

#ifndef __SPATIALINDEX_H
#define __SPATIALINDEX_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "WorldQueryTypes.h"

namespace Darkness {

class SpatialIndex {
public:
    explicit SpatialIndex(float cellSize = 32.0f)
        : mCellSize(cellSize), mInvCellSize(1.0f / cellSize) {}

    // -- Mutation --

    /// Insert an entity at a world position.
    void insert(EntityID id, const Vector3 &pos) {
        mPositions[id] = pos;
        uint64_t key = cellKeyFromPos(pos);
        mCells[key].push_back(id);
    }

    /// Remove an entity (uses its stored position to find the correct cell).
    void remove(EntityID id) {
        auto posIt = mPositions.find(id);
        if (posIt == mPositions.end())
            return;

        uint64_t key = cellKeyFromPos(posIt->second);
        auto cellIt = mCells.find(key);
        if (cellIt != mCells.end()) {
            auto &vec = cellIt->second;
            vec.erase(std::remove(vec.begin(), vec.end(), id), vec.end());
            if (vec.empty())
                mCells.erase(cellIt);
        }

        mPositions.erase(posIt);
    }

    /// Update an entity's position (remove from old cell, insert into new).
    void update(EntityID id, const Vector3 &newPos) {
        remove(id);
        insert(id, newPos);
    }

    /// Clear all entries.
    void clear() {
        mCells.clear();
        mPositions.clear();
    }

    // -- Queries --

    /// Find all entities within radius of center (exact distance check).
    /// When the cell range is large, falls back to iterating all entities
    /// for O(n) rather than O(cells^3) traversal.
    std::vector<EntityID> queryRadius(const Vector3 &center,
                                      float radius) const {
        std::vector<EntityID> result;
        if (radius < 0.0f)
            return result;

        float radiusSq = radius * radius;

        // Compute cell range covering the sphere's AABB
        int32_t minCX = toCell(center.x - radius);
        int32_t maxCX = toCell(center.x + radius);
        int32_t minCY = toCell(center.y - radius);
        int32_t maxCY = toCell(center.y + radius);
        int32_t minCZ = toCell(center.z - radius);
        int32_t maxCZ = toCell(center.z + radius);

        // If the cell range exceeds the number of occupied cells, iterate
        // all entities directly instead — avoids O(cells^3) for large radii
        int64_t cellRange = static_cast<int64_t>(maxCX - minCX + 1) *
                            (maxCY - minCY + 1) * (maxCZ - minCZ + 1);

        if (cellRange > static_cast<int64_t>(mCells.size()) * 2) {
            // Brute-force: check every tracked entity
            for (const auto &[id, pos] : mPositions) {
                float dx = pos.x - center.x;
                float dy = pos.y - center.y;
                float dz = pos.z - center.z;
                if (dx * dx + dy * dy + dz * dz <= radiusSq)
                    result.push_back(id);
            }
            return result;
        }

        for (int32_t cz = minCZ; cz <= maxCZ; ++cz) {
            for (int32_t cy = minCY; cy <= maxCY; ++cy) {
                for (int32_t cx = minCX; cx <= maxCX; ++cx) {
                    auto it = mCells.find(cellKey(cx, cy, cz));
                    if (it == mCells.end())
                        continue;

                    for (EntityID id : it->second) {
                        auto posIt = mPositions.find(id);
                        if (posIt == mPositions.end())
                            continue;

                        const Vector3 &p = posIt->second;
                        float dx = p.x - center.x;
                        float dy = p.y - center.y;
                        float dz = p.z - center.z;
                        float distSq = dx * dx + dy * dy + dz * dz;

                        // Include entities at exactly the radius (<=)
                        if (distSq <= radiusSq)
                            result.push_back(id);
                    }
                }
            }
        }

        return result;
    }

    /// Find all entities within an axis-aligned bounding box (exact check).
    /// When the cell range is large, falls back to iterating all entities.
    std::vector<EntityID> queryAABB(const BBox &box) const {
        std::vector<EntityID> result;

        int32_t minCX = toCell(box.min.x);
        int32_t maxCX = toCell(box.max.x);
        int32_t minCY = toCell(box.min.y);
        int32_t maxCY = toCell(box.max.y);
        int32_t minCZ = toCell(box.min.z);
        int32_t maxCZ = toCell(box.max.z);

        // Large-range fallback: iterate all entities directly
        int64_t cellRange = static_cast<int64_t>(maxCX - minCX + 1) *
                            (maxCY - minCY + 1) * (maxCZ - minCZ + 1);

        if (cellRange > static_cast<int64_t>(mCells.size()) * 2) {
            for (const auto &[id, pos] : mPositions) {
                if (pos.x >= box.min.x && pos.x <= box.max.x &&
                    pos.y >= box.min.y && pos.y <= box.max.y &&
                    pos.z >= box.min.z && pos.z <= box.max.z) {
                    result.push_back(id);
                }
            }
            return result;
        }

        for (int32_t cz = minCZ; cz <= maxCZ; ++cz) {
            for (int32_t cy = minCY; cy <= maxCY; ++cy) {
                for (int32_t cx = minCX; cx <= maxCX; ++cx) {
                    auto it = mCells.find(cellKey(cx, cy, cz));
                    if (it == mCells.end())
                        continue;

                    for (EntityID id : it->second) {
                        auto posIt = mPositions.find(id);
                        if (posIt == mPositions.end())
                            continue;

                        const Vector3 &p = posIt->second;
                        if (p.x >= box.min.x && p.x <= box.max.x &&
                            p.y >= box.min.y && p.y <= box.max.y &&
                            p.z >= box.min.z && p.z <= box.max.z) {
                            result.push_back(id);
                        }
                    }
                }
            }
        }

        return result;
    }

    // -- Diagnostics --

    /// Number of entities currently tracked.
    size_t size() const { return mPositions.size(); }

    /// Number of occupied grid cells.
    size_t cellCount() const { return mCells.size(); }

private:
    float mCellSize;
    float mInvCellSize; // 1.0 / cellSize, precomputed for fast floor division

    // Cell key → list of entity IDs in that cell
    std::unordered_map<uint64_t, std::vector<EntityID>> mCells;

    // Entity ID → stored position (for exact distance/containment checks)
    std::unordered_map<EntityID, Vector3> mPositions;

    /// Convert a world coordinate to a cell index.
    int32_t toCell(float v) const {
        return static_cast<int32_t>(std::floor(v * mInvCellSize));
    }

    /// Pack three cell coordinates into a 64-bit key.
    /// uint16_t wrapping handles negative indices; 48-bit key space is ample.
    static uint64_t cellKey(int32_t cx, int32_t cy, int32_t cz) {
        return (static_cast<uint64_t>(static_cast<uint16_t>(cz)) << 32) |
               (static_cast<uint64_t>(static_cast<uint16_t>(cy)) << 16) |
               static_cast<uint64_t>(static_cast<uint16_t>(cx));
    }

    /// Convenience: compute cell key directly from a world position.
    uint64_t cellKeyFromPos(const Vector3 &pos) const {
        return cellKey(toCell(pos.x), toCell(pos.y), toCell(pos.z));
    }
};

} // namespace Darkness

#endif // __SPATIALINDEX_H
