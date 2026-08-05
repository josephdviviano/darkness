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

// SpawnSystem.h — create objects at runtime from an archetype.
//
// Emitter tweqs spawn OBJECTS, not GPU particles: their config names an
// archetype ("H2OSplash", "MossSpore", "RippleRing"), and the spawned object
// then runs its own tweqs — typically a Delete countdown that cleans it up.
// That is the whole lifecycle of a Thief effect: emit, animate, expire.
//
// Spawning has to touch three things at once, which is why it lives here in the
// renderer rather than in a sim subsystem:
//   1. ObjectService — allocate a concrete object under the archetype, so it
//      inherits every property the archetype defines.
//   2. The placement map + renderer instance list — so it has a transform, gets
//      drawn, and is visible to the systems that scan placements.
//   3. TweqSystem — so its own tweqs start running.
//
// Models are NOT loaded on demand. Every archetype an emitter can name is
// resolved at load and its model pre-loaded with the rest (the same thing the
// Models tweq already does for its variants), because a GPU upload mid-frame
// would stall. An emission naming a model that was not pre-loaded is reported
// rather than silently invisible.

#pragma once

#include <cstdio>
#include <cstring>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "DarknessException.h"
#include "DarknessMath.h"
#include "ObjectPropParser.h"
#include "object/ObjectService.h"
#include "property/DarkPropertyDefs.h"
#include "property/Property.h"
#include "property/PropertyService.h"
#include "property/TypedProperty.h"
#include "sim/SimCommon.h"
#include "sim/TweqSystem.h"
#include "worldquery/ObjectState.h"

namespace Darkness {

class SpawnSystem {
public:
    void init(ObjectService *objSvc, PropertyService *propSvc,
              ObjectStateMap *objectStates,
              std::unordered_map<int32_t, ObjPlacementInfo> *placements,
              ObjectPropData *objData, std::vector<int32_t> *objCellIDs,
              TweqSystem *tweqSystem) {
        mObjSvc = objSvc;
        mPropSvc = propSvc;
        mObjectStates = objectStates;
        mPlacements = placements;
        mObjData = objData;
        mObjCellIDs = objCellIDs;
        mTweqSystem = tweqSystem;
    }

    /// Collect the model names every emitter in the mission can spawn, so the
    /// asset loader picks them up alongside the statically placed models.
    /// Mirrors TweqSystem::collectModelNames for the Models tweq.
    static std::vector<std::string> collectEmitterModelNames(
        PropertyService *propSvc, ObjectService *objSvc) {
        std::vector<std::string> names;
        if (!propSvc || !objSvc) return names;

        std::unordered_set<std::string> seen;
        for (int slot = 0; slot < 5; ++slot) {
            const char *cfgProp = TweqSystem::kEmitterCfgProps[slot];
            for (int id : getAllObjectsWithProperty(propSvc, cfgProp)) {
                PropCfgTweqEmitter cfg;
                if (!getTypedProperty<PropCfgTweqEmitter>(propSvc, cfgProp, id,
                                                          cfg))
                    continue;
                std::string arch(cfg.emitWhat, strnlen(cfg.emitWhat, 16));
                if (arch.empty() || !seen.insert(arch).second) continue;

                const int archID = objSvc->named(arch);
                if (archID == 0) continue;

                char model[16] = {};
                if (!getTypedProperty<char[16]>(propSvc, "ModelName", archID,
                                                model))
                    continue;
                std::string m(model, strnlen(model, 16));
                if (!m.empty()) names.push_back(std::move(m));
            }
        }
        std::sort(names.begin(), names.end());
        names.erase(std::unique(names.begin(), names.end()), names.end());
        return names;
    }

    /// Turn one emission into a live object. Returns the new object ID, or 0.
    int32_t spawn(const EmitRequest &req) {
        if (!mObjSvc || !mPropSvc || !mObjData) return 0;

        const int archID = mObjSvc->named(req.archetypeName);
        if (archID == 0) {
            reportOnce(req.archetypeName,
                       "no archetype of that name exists in the gamesys");
            return 0;
        }

        const int32_t objID = static_cast<int32_t>(mObjSvc->create(archID));
        if (objID <= 0) {
            reportOnce(req.archetypeName, "ObjectService refused to create it");
            return 0;
        }

        // Give it a transform. P$Position is what every placement-scanning
        // system reads, so the property has to be written, not just the runtime
        // state.
        //
        // The record must be CREATED first. A freshly created object has no
        // Position record — the archetype cannot supply one either, since
        // Position is registered with the "never" inheritor — and the storage's
        // setField begins with a find() that returns false when the record is
        // absent. `set` discards that, and only logs when the property NAME is
        // unknown, so this failed completely silently: every spawned object
        // reported position (0,0,0), which also kept it out of the spatial
        // index and therefore out of every AI and audio query.
        if (Property *posProp = mPropSvc->getProperty("Position")) {
            if (!posProp->has(objID)) posProp->createProperty(objID);
        }
        if (!mPropSvc->set(objID, "Position", "position", Variant(req.position))) {
            reportOnce(req.archetypeName,
                       "could not write P$Position, so it spawns at the origin "
                       "and stays invisible to AI and audio queries");
        }

        char model[16] = {};
        getTypedProperty<char[16]>(mPropSvc, "ModelName", objID, model);

        // Placement entry: the base transform every tweq type reads.
        ObjPlacementInfo pl{};
        pl.x = req.position.x;
        pl.y = req.position.y;
        pl.z = req.position.z;
        pl.sx = pl.sy = pl.sz = 1.0f;
        std::memcpy(pl.modelName, model, sizeof(pl.modelName));
        if (mPlacements) (*mPlacements)[objID] = pl;

        if (mObjectStates) {
            ObjectState &os = mObjectStates->get(objID);
            os.position = req.position;
            os.scale = Vector3(1.0f);
        }

        // Renderer instance. Appending is safe mid-frame-loop: the draw pass
        // reads the vector by const ref each frame, indices already handed out
        // stay valid, and GPU buffers are per-model and shared.
        ObjectPlacement inst{};
        inst.objID = objID;
        std::memcpy(inst.modelName, model, sizeof(inst.modelName));
        inst.x = req.position.x;
        inst.y = req.position.y;
        inst.z = req.position.z;
        inst.scaleX = inst.scaleY = inst.scaleZ = 1.0f;
        inst.renderAlpha = 1.0f;
        inst.hasPosition = true;
        // Reuse the slot of an earlier spawn that has since expired, so a level
        // that emits for an hour does not grow the draw list for an hour.
        // objCellIDs is parallel to objects[] and indexed by position, so the
        // two must stay in lockstep. -1 means "find my cell at draw time",
        // which is what moving objects already do.
        size_t idx;
        if (!mFreeInstanceSlots.empty()) {
            idx = mFreeInstanceSlots.back();
            mFreeInstanceSlots.pop_back();
            mObjData->objects[idx] = inst;
            if (mObjCellIDs && idx < mObjCellIDs->size()) (*mObjCellIDs)[idx] = -1;
        } else {
            idx = mObjData->objects.size();
            mObjData->objects.push_back(inst);
            if (mObjCellIDs) mObjCellIDs->push_back(-1);
        }
        mInstanceOfSpawn[objID] = idx;

        // Its own tweqs — most emitted objects carry a Delete countdown, which
        // is what stops them accumulating.
        if (mTweqSystem) mTweqSystem->registerObject(mPropSvc, objID);

        ++mSpawned;
        // Spawned objects are appended to the renderer's instance list and are
        // never removed from it — a Delete tweq only marks them destroyed. A
        // level that emits without bound would grow that list without bound,
        // so the running total is reported rather than left to be discovered.
        if (mSpawned % 250 == 0)
            std::fprintf(stderr, "[SPAWN] %u spawned, %u recycled, %zu live "
                         "(%zu renderer instances)\n",
                         mSpawned, mRecycled, mInstanceOfSpawn.size(),
                         mObjData->objects.size());
        if (mSpawned <= 10)
            std::fprintf(stderr, "[SPAWN] obj %d '%s' from '%s' at "
                         "(%.1f, %.1f, %.1f) vel (%.1f, %.1f, %.1f)\n",
                         objID, model, req.archetypeName.c_str(),
                         req.position.x, req.position.y, req.position.z,
                         req.velocity.x, req.velocity.y, req.velocity.z);

        if (model[0] == '\0')
            reportOnce(req.archetypeName,
                       "the archetype has no ModelName, so it spawns invisible");

        // The emission flags that change WHERE or HOW an object spawns are
        // carried on the request and not acted on. Named individually rather
        // than dropped in silence, because each one moves the object.
        if (req.miscFlags & (kTweqMiscVHot | kTweqMiscPushOut |
                             kTweqMiscGravity | kTweqMiscNoPhysics)) {
            static int warnCount = 0;
            if (warnCount++ < 3)
                std::fprintf(stderr, "[FALLBACK] SpawnSystem: obj %d ignores "
                             "emission flags 0x%03x%s%s%s%s\n", objID,
                             req.miscFlags,
                             (req.miscFlags & kTweqMiscVHot) ? " VHot(anchor at vhot)" : "",
                             (req.miscFlags & kTweqMiscPushOut) ? " PushOut(clear the emitter)" : "",
                             (req.miscFlags & kTweqMiscGravity) ? " Gravity" : "",
                             (req.miscFlags & kTweqMiscNoPhysics) ? " NoPhysics" : "");
        }

        // Velocity is carried on the request but nothing integrates it yet:
        // emitted objects need a dynamic physics body, which object physics
        // does not create at runtime. Announced rather than silently dropped.
        if (glm::length(req.velocity) > 1e-4f && mVelWarned < 3) {
            ++mVelWarned;
            std::fprintf(stderr, "[FALLBACK] SpawnSystem: obj %d spawned with "
                         "velocity (%.2f, %.2f, %.2f) but no runtime physics "
                         "body — it will sit still where it was emitted\n",
                         objID, req.velocity.x, req.velocity.y, req.velocity.z);
        }
        return objID;
    }

    /// Hand a spawned object's renderer slot back when it expires. Called from
    /// the same destroy funnel that takes it out of collision and world
    /// queries; a no-op for objects this system did not spawn.
    void recycle(int32_t objID) {
        auto it = mInstanceOfSpawn.find(objID);
        if (it == mInstanceOfSpawn.end()) return;   // not ours

        const size_t idx = it->second;
        mInstanceOfSpawn.erase(it);

        // Shed every other trace of it. Handing back the renderer slot alone
        // still leaked into the placement map, the tweq map, the object-state
        // map and the object system — measured at 500 spawns / 478 still live
        // after 3 minutes of MISS13, with object IDs climbing past 143,000.
        // Those maps are scanned linearly on hot paths, so the leak was also a
        // steady slowdown.
        if (mPlacements) mPlacements->erase(objID);
        if (mTweqSystem) mTweqSystem->forgetObject(objID);
        if (mObjectStates) mObjectStates->remove(objID);
        // Only objects this system created may have their ID released: we own
        // every reference to them. Placed objects keep their IDs forever
        // because doors, the renderer's static instance array and audio voices
        // all cache raw IDs, and recycling one would repoint those at a
        // different object.
        if (mObjSvc) {
            // Teardown walks every registered property and relation for this
            // object. A malformed one must not take the process down — report
            // it and leave the ID allocated rather than abort mid-frame.
            try {
                mObjSvc->destroySpawned(objID);
            } catch (const BasicException &e) {
                static int warnCount = 0;
                if (warnCount++ < 5)
                    std::fprintf(stderr, "[FALLBACK] SpawnSystem: destroying "
                                 "spawned obj %d threw (%s) — its ID stays "
                                 "allocated\n", objID, e.getDetails().c_str());
            }
        }
        ++mRecycled;

        if (!mObjData || idx >= mObjData->objects.size()) {
            std::fprintf(stderr, "[FALLBACK] SpawnSystem: obj %d had renderer "
                         "slot %zu, which is out of range — the slot is lost "
                         "rather than reused\n", objID, idx);
            return;
        }

        // Blank the slot so it draws nothing until something reuses it. The
        // destroyed flag on ObjectState already suppresses the draw; this makes
        // the slot safe even after that state is gone.
        mObjData->objects[idx] = ObjectPlacement{};
        mObjData->objects[idx].objID = 0;
        mObjData->objects[idx].hasPosition = false;
        mFreeInstanceSlots.push_back(idx);
    }

    uint32_t spawnCount() const { return mSpawned; }
    uint32_t recycledCount() const { return mRecycled; }
    size_t liveSpawnCount() const { return mInstanceOfSpawn.size(); }

private:
    void reportOnce(const std::string &archetype, const char *why) {
        if (!mReported.insert(archetype).second) return;
        std::fprintf(stderr, "[FALLBACK] SpawnSystem: cannot spawn '%s' — %s\n",
                     archetype.c_str(), why);
    }

    ObjectService *mObjSvc = nullptr;
    PropertyService *mPropSvc = nullptr;
    ObjectStateMap *mObjectStates = nullptr;
    std::unordered_map<int32_t, ObjPlacementInfo> *mPlacements = nullptr;
    ObjectPropData *mObjData = nullptr;
    std::vector<int32_t> *mObjCellIDs = nullptr;
    TweqSystem *mTweqSystem = nullptr;

    uint32_t mSpawned = 0;
    uint32_t mRecycled = 0;
    int mVelWarned = 0;
    // Renderer instance slot per live spawned object, and the slots freed by
    // expired ones.
    std::unordered_map<int32_t, size_t> mInstanceOfSpawn;
    std::vector<size_t> mFreeInstanceSlots;
    std::unordered_set<std::string> mReported;
};

} // namespace Darkness
