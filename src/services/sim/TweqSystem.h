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

// TweqSystem.h — Procedural object animation (tweens/sequences)
//
// Implements the Dark Engine tweq system: per-object rotation, scaling, model
// cycling, and visibility toggling driven by configuration properties and
// controlled via script messages (TurnOn/TurnOff).
//
// Architecture:
//   - TweqSystem is a SimListener: receives simStep(simTime, dt) from SimService
//   - Each tweq has a TweqInstance with config (from Cfg property) and runtime state
//   - Transforms are written to ObjectStateMap for the renderer
//   - TurnOn/TurnOff activate/halt tweqs; TweqComplete messages notify scripts
//
// Inspired by the openDarkEngine (OPDE) service architecture (GPLv2).
// Tweq animation algorithm reimplemented from Dark Engine behavior analysis.

#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "DarknessMath.h"
#include "SimCommon.h"
#include "object/ObjectService.h"
#include "property/DarkPropertyDefs.h"
#include "property/PropertyService.h"
#include "property/TypedProperty.h"
#include "BinMeshParser.h"
#include "SubObjectPose.h"
#include "worldquery/ObjectState.h"

namespace Darkness {

// ── Tweq activation actions (matches Dark Engine eTweqDo) ──
enum TweqAction : int32_t {
    kTweqDoDefault   = 0,  // Toggle: if on→halt, if off→activate
    kTweqDoActivate  = 1,  // Start (if not already running)
    kTweqDoHalt      = 2,  // Stop
    kTweqDoReset     = 3,  // Reset to initial state
    kTweqDoContinue  = 4,  // Continue (start if stopped)
    kTweqDoForward   = 5,  // Start forward
    kTweqDoReverse   = 6,  // Start reverse
};

// ── Completion result (matches Dark Engine halt actions) ──
static constexpr int kTweqStatusQuo   = -1;  // Still running, no action needed
static constexpr int kTweqFrameEvent  = -2;  // Frame boundary event (flicker toggle)

// ── Per-tweq runtime state ──
struct TweqInstance {
    int32_t   objID = 0;
    eTweqType type  = kTweqTypeRotate;

    // Config (copied from Cfg property at init)
    uint8_t   cfgCurve = 0;   // TweqCurveFlags
    uint8_t   cfgAnim  = 0;   // TweqAnimFlags
    uint8_t   cfgHalt  = 0;   // TweqHaltAction
    uint16_t  cfgMisc  = 0;   // TweqMiscFlags

    // Per-axis config. Rotate/Scale use the first 3 slots (X/Y/Z); Joints uses
    // all 6 (one per model joint slot) — the per-slot algorithm is identical.
    static constexpr int kMaxAxes = 6;
    PropTweqAxisConfig axes[kMaxAxes] = {};
    int       axisCount = 3;
    int32_t   primaryAxis = 0;   // 0=all, else 1-indexed axis/joint

    // Simple config (Flicker/Models/Delete)
    uint16_t  cfgRate = 0;       // ms per step

    // Lock tweq — the single model joint slot this lock drives
    int       lockJointSlot = 0;

    // Emitter tweq. An object can carry five independent emitters, so the slot
    // is part of this instance's identity (see tweqKey).
    int       emitterSlot = 0;      // 0..4
    char      emitWhat[17] = {};    // archetype symbolic name
    Vector3   emitVelocity{0.0f};
    Vector3   emitAngleRandom{0.0f};

    // Models tweq
    char      modelNames[6][16] = {};
    int       modelCount = 0;    // number of valid model names
    float     baseAnchorZ = 0.0f; // bbox bottom Z of first model (anchor reference)

    // ── Runtime state ──
    bool      active = false;
    uint32_t  axisState[kMaxAxes] = {}; // per-axis TweqStateFlags
    float     values[kMaxAxes] = {};    // angles (degrees), scale factors, or
                                        //   joint positions (degrees / units)
    float     elapsedMs = 0.0f;  // for timed tweqs
    int16_t   curFrame = 0;      // for Models tweq
    bool      flickerHidden = false;  // current flicker visibility state
    bool      hasAnimLight = false;  // object has AnimLight — flicker controls light, not visibility
    int       logFrames = 0;       // diagnostic: frame counter for per-tweq logging

    // Base transform snapshot
    SimTransform base;
    bool      hasObjectState = false;
};

// ── Callback for tweq completion events ──
using TweqEventCallback = std::function<void(int32_t objID, eTweqType type,
                                              int haltAction)>;

// ── One object emission requested by an emitter tweq ──
//
// The tweq decides WHAT to emit, WHERE and with what velocity; turning that
// into a live object is the spawner's job, because it needs the object system,
// the renderer's instance list and the asset set — none of which belong in a
// sim subsystem.
struct EmitRequest {
    int32_t     emitterObjID = 0;   // who is emitting
    int         emitterSlot = 0;    // which of the object's five emitters
    std::string archetypeName;      // symbolic name of the archetype to spawn
    Vector3     position{0.0f};     // world position to spawn at
    Vector3     velocity{0.0f};     // initial velocity, flags already applied
    uint16_t    miscFlags = 0;      // TweqMiscFlags — Gravity, NoPhysics, VHot…
};

using EmitCallback = std::function<void(const EmitRequest &)>;

// ── Query: is this object currently on screen? ──
//
// CONTRACT: returns true when the object is visible OR when it is not
// renderable at all. The second half matters — an invisible marker with a
// delete countdown (waypoints, particle anchors) must not be frozen forever
// by a gate whose whole premise is "the player can see it". The renderer owns
// that distinction, so it answers rather than the sim guessing.
using VisibilityQuery = std::function<bool(int32_t objID)>;

// ── Callback fired when a tweq destroys or slays its object ──
// Wired to whatever has to take the object out of the live world: physics
// bodies, the spatial index, world queries. Unlike TweqEventCallback this is
// NOT gated on the Scripts flag — an object dying is not an optional
// notification.
using ObjectDestroyCallback = std::function<void(int32_t objID)>;

// ============================================================================
// TweqSystem — manages all tweq animations
// ============================================================================

class TweqSystem : public SimListener {
public:
    TweqSystem() : mRng(std::random_device{}()) {}

    /// The five emitter slots' property names, in slot order.
    static constexpr const char *kEmitterCfgProps[5] = {
        "CfgTweqEm", "CfgTweq2E", "CfgTweq3E", "CfgTweq4E", "CfgTweq5E"};
    static constexpr const char *kEmitterStateProps[5] = {
        "StTweqEmi", "StTweq2Em", "StTweq3Em", "StTweq4Em", "StTweq5Em"};

    // ── Pre-init: collect model names for asset loading ──

    /// Scan all positioned objects for CfgTweqMo properties and collect the
    /// model names they reference. Called BEFORE loadObjectAssets so variant
    /// models (flame3c, etc.) are loaded and have GPU buffers.
    /// Scan all objects with CfgTweqMo (Models tweq config) and collect the
    /// model names they reference. Uses property inheritance so archetype
    /// configs are found for all concrete objects.
    static std::vector<std::string> collectModelNames(PropertyService *propSvc) {
        std::vector<std::string> names;
        if (!propSvc) return names;
        // Get all objects (including archetypes) that have CfgTweqMo
        auto ids = getAllObjectsWithProperty(propSvc, "CfgTweqMo");
        for (int id : ids) {
            PropCfgTweqModels cfg;
            if (!getTypedProperty<PropCfgTweqModels>(propSvc, "CfgTweqMo", id, cfg))
                continue;
            for (int i = 0; i < 6; ++i) {
                if (cfg.modelName[i][0] != '\0') {
                    names.emplace_back(cfg.modelName[i],
                        strnlen(cfg.modelName[i], 16));
                }
            }
        }
        // Deduplicate
        std::sort(names.begin(), names.end());
        names.erase(std::unique(names.begin(), names.end()), names.end());
        return names;
    }

    // ── Initialization ──

    void init(PropertyService *propSvc,
              ObjectStateMap *objectStates,
              const std::unordered_map<int32_t, ObjPlacementInfo> *placements,
              const std::unordered_map<std::string, ParsedBinMesh> *parsedModels = nullptr) {
        mObjectStates = objectStates;
        mPlacements = placements;
        mParsedModels = parsedModels;

        // Scan for each tweq type. Property names are chunk names from the pldef
        // (e.g., "CfgTweqRo"), NOT labels (e.g., "TweqRotateConfig") — the
        // PropertyService registers by chunk name.
        initTweqsOfType<PropCfgTweqVector, PropStTweqVector>(
            propSvc, "CfgTweqRo", "StTweqRot", kTweqTypeRotate);
        initTweqsOfType<PropCfgTweqVector, PropStTweqVector>(
            propSvc, "CfgTweqSc", "StTweqSca", kTweqTypeScale);
        initTweqsOfType<PropCfgTweqSimple, PropStTweqSimple>(
            propSvc, "CfgTweqBl", "StTweqBli", kTweqTypeFlicker);
        initTweqsOfType<PropCfgTweqModels, PropStTweqSimple>(
            propSvc, "CfgTweqMo", "StTweqMod", kTweqTypeModels);
        initTweqsOfType<PropCfgTweqJoints, PropStTweqJoints>(
            propSvc, "CfgTweqJo", "StTweqJoi", kTweqTypeJoints);
        initTweqsOfType<PropCfgTweqSimple, PropStTweqSimple>(
            propSvc, "CfgTweqDe", "StTweqDel", kTweqTypeDelete);
        initTweqsOfType<PropCfgTweqLock, PropStTweqLock>(
            propSvc, "CfgTweqLo", "StTweqLoc", kTweqTypeLock);

        // Five independent emitter slots per object.
        for (int slot = 0; slot < 5; ++slot) {
            initTweqsOfType<PropCfgTweqEmitter, PropStTweqSimple>(
                propSvc, kEmitterCfgProps[slot], kEmitterStateProps[slot],
                kTweqTypeEmitter, slot);
        }

        // Seed the joint pose of objects that carry a JointPos but no joints
        // tweq — a lever left thrown, a chest authored open. Without this they
        // would render at rest until something animated them.
        seedStaticJointPositions(propSvc);

        // Count by type for diagnostics
        int counts[8] = {};
        int autoStart = 0;
        for (const auto &[key, tw] : mTweqs) {
            if (tw.type >= 0 && tw.type < 8) counts[tw.type]++;
            if (tw.active) autoStart++;
        }
        const char *typeNames[] = {"Scale","Rotate","Joints","Models",
                                    "Delete","Emitter","Flicker","Lock"};
        std::fprintf(stderr, "TweqSystem: %zu instances (", mTweqs.size());
        bool first = true;
        for (int i = 0; i < 8; ++i) {
            if (counts[i] > 0) {
                std::fprintf(stderr, "%s%d %s", first ? "" : ", ",
                             counts[i], typeNames[i]);
                first = false;
            }
        }
        std::fprintf(stderr, "), %d running from their stored state\n", autoStart);
    }

    // ── Activation API ──

    /// Activate/halt tweqs on an object. Returns true if the object has tweqs.
    bool activate(int32_t objID, TweqAction action,
                  eTweqType typeFilter = kTweqTypeAll) {
        bool found = false;
        for (auto &[key, tw] : mTweqs) {
            if (tw.objID != objID) continue;
            if (typeFilter != kTweqTypeAll && tw.type != typeFilter) continue;
            found = true;
            applyAction(tw, action);
        }
        return found;
    }

    /// Check if an object has any tweq instances.
    bool hasTweqs(int32_t objID) const {
        for (const auto &[key, tw] : mTweqs)
            if (tw.objID == objID) return true;
        return false;
    }

    /// Live animation-state bits (kTweqStateOn / kTweqStateReverse) for an
    /// object's tweqs, OR-ed across matching instances.
    ///
    /// Scripts used to read these off the StTweq* properties, but those are
    /// stored as opaque blobs (RawDataStorage::getField always fails), so the
    /// query silently returned "nothing set" for every object. The running
    /// instance is the honest source anyway — the property is only the
    /// load-time snapshot.
    uint32_t animStateBits(int32_t objID,
                           eTweqType typeFilter = kTweqTypeAll) const {
        uint32_t bits = 0;
        for (const auto &[key, tw] : mTweqs) {
            if (tw.objID != objID) continue;
            if (typeFilter != kTweqTypeAll && tw.type != typeFilter) continue;
            if (tw.active) bits |= kTweqStateOn;
            for (int i = 0; i < tw.axisCount; ++i)
                bits |= (tw.axisState[i] & kTweqStateReverse);
        }
        return bits;
    }

    /// Decode a lock tweq's joint selector.
    ///
    /// The engine reads `CfgTweqLock::targetJoint` as a byte and decrements it
    /// when non-zero, so 0 and 1 both name slot 0. Stock data uses 0, 1 and 2.
    /// [BIN: `if (sel != 0) sel--` at FUN_00595C40, Thief2.exe NewDark 1.28]
    static int lockJointSlotFromConfig(int32_t targetJoint) {
        int slot = static_cast<int>(static_cast<int8_t>(targetJoint & 0xFF));
        if (slot != 0) --slot;
        return slot;
    }

    /// Set callback for tweq completion events (for TweqComplete messages).
    void setEventCallback(TweqEventCallback cb) { mEventCallback = std::move(cb); }

    /// Give an object created at runtime the tweqs its archetype defines.
    ///
    /// The object must already be in the placements map — the spawner puts it
    /// there — because that is where every tweq type reads its base transform
    /// from. Safe to call twice: instances are keyed, so a second call
    /// overwrites rather than duplicating.
    void registerObject(PropertyService *propSvc, int32_t objID) {
        if (!propSvc || objID <= 0) return;
        const size_t before = mTweqs.size();

        initTweqsOfType<PropCfgTweqVector, PropStTweqVector>(
            propSvc, "CfgTweqRo", "StTweqRot", kTweqTypeRotate, 0, objID,
            /*activateWhenStateless=*/true);
        initTweqsOfType<PropCfgTweqVector, PropStTweqVector>(
            propSvc, "CfgTweqSc", "StTweqSca", kTweqTypeScale, 0, objID,
            /*activateWhenStateless=*/true);
        initTweqsOfType<PropCfgTweqSimple, PropStTweqSimple>(
            propSvc, "CfgTweqBl", "StTweqBli", kTweqTypeFlicker, 0, objID,
            /*activateWhenStateless=*/true);
        initTweqsOfType<PropCfgTweqModels, PropStTweqSimple>(
            propSvc, "CfgTweqMo", "StTweqMod", kTweqTypeModels, 0, objID,
            /*activateWhenStateless=*/true);
        initTweqsOfType<PropCfgTweqJoints, PropStTweqJoints>(
            propSvc, "CfgTweqJo", "StTweqJoi", kTweqTypeJoints, 0, objID,
            /*activateWhenStateless=*/true);
        initTweqsOfType<PropCfgTweqSimple, PropStTweqSimple>(
            propSvc, "CfgTweqDe", "StTweqDel", kTweqTypeDelete, 0, objID,
            /*activateWhenStateless=*/true);
        initTweqsOfType<PropCfgTweqLock, PropStTweqLock>(
            propSvc, "CfgTweqLo", "StTweqLoc", kTweqTypeLock, 0, objID,
            /*activateWhenStateless=*/true);
        for (int slot = 0; slot < 5; ++slot) {
            initTweqsOfType<PropCfgTweqEmitter, PropStTweqSimple>(
                propSvc, kEmitterCfgProps[slot], kEmitterStateProps[slot],
                kTweqTypeEmitter, slot, objID,
                /*activateWhenStateless=*/true);
        }

        // A spawned object with no tweqs never expires, so it would sit in the
        // world forever. Worth knowing about rather than discovering as a leak.
        if (mTweqs.size() == before) {
            static int warnCount = 0;
            if (warnCount++ < 5)
                std::fprintf(stderr, "[FALLBACK] TweqSystem: runtime object %d "
                             "inherited no tweqs — nothing will ever expire it\n",
                             objID);
        }
    }

    /// Does this tweq get a tick this frame?
    ///
    /// The config word's anim bits are update-RATE gates, not activation:
    ///   OFFSCRN  run ONLY while off screen (9 shipped configs) — the inverse
    ///            of the default, not merely a relaxation of it
    ///   SIM      run always (272 shipped configs)
    ///   neither  run only while on screen (184 shipped configs)
    ///
    /// The two radius gates are deliberately NOT applied: their distances
    /// (nominally 20 and 80 units) have no derivation, and this file already
    /// shipped one inherited guess about these flags that turned out wrong.
    /// SIMRADSM appears on zero shipped configs and SIMRADLG on 11, so leaving
    /// them ungated costs 11 objects a slightly-too-eager tick.
    bool shouldTickThisFrame(const TweqInstance &tw) const {
        if (!mVisibilityGating || !mVisibilityQuery) return true;  // fail open

        const bool onScreen = mVisibilityQuery(tw.objID);
        if (tw.cfgAnim & kTweqAnimOffscreen) return !onScreen;
        if (tw.cfgAnim & kTweqAnimSim) return true;
        return onScreen;
    }

    /// Drop every tweq instance belonging to an object that has left the
    /// world. Without this a destroyed object's OTHER tweqs keep running —
    /// a Models tweq goes on rewriting modelNameOverride forever — and the
    /// map grows without bound, which matters because activate(), hasTweqs(),
    /// animStateBits() and applyComposedTransform() all scan it linearly.
    ///
    /// Returns how many instances were dropped.
    size_t forgetObject(int32_t objID) {
        size_t dropped = 0;
        for (auto it = mTweqs.begin(); it != mTweqs.end(); ) {
            if (it->second.objID == objID) {
                it = mTweqs.erase(it);
                ++dropped;
            } else {
                ++it;
            }
        }
        return dropped;
    }

    /// Number of emissions requested so far (diagnostics).
    uint32_t emissionCount() const { return mEmitFired; }

    /// Wire the on-screen test used by the update-rate gates. Until this is
    /// set the gates cannot be applied and every tweq ticks every frame.
    void setVisibilityQuery(VisibilityQuery q) { mVisibilityQuery = std::move(q); }

    /// Turn the update-rate gates on or off.
    ///
    /// ON is faithful: the config word's flags say when a tweq should tick, and
    /// 184 of the 456 shipped configs are gated to "only while on screen".
    /// OFF ticks everything every frame, which costs almost nothing (measured
    /// 0.054 ms/frame for all 539 tweq instances of the heaviest shipped level,
    /// ~0.3% of a 60 Hz budget) and is the escape hatch if faithful gating ever
    /// reads worse than it simulates — a part caught mid-motion as it comes on
    /// screen, say. This is a look-versus-fidelity dial, NOT a performance one.
    void setVisibilityGating(bool on) { mVisibilityGating = on; }

    bool visibilityGating() const { return mVisibilityGating; }

    /// Set the callback that turns an emitter tweq's emission into a live
    /// object. Without it, emissions are counted and reported but nothing
    /// spawns.
    void setEmitCallback(EmitCallback cb) { mEmitCallback = std::move(cb); }

    /// Set the callback that removes an object from the live world when a tweq
    /// destroys or slays it.
    void setDestroyCallback(ObjectDestroyCallback cb) {
        mDestroyCallback = std::move(cb);
    }

    // ── SimListener interface ──

    void simStep(float simTime, float delta) override {
        if (delta <= 0.0f) return;

        float dt_ms = delta * 1000.0f;
        ++mFrameCount;

        // Collect objects that need transform updates (may have multiple tweqs)
        mDirtyObjects.clear();

        for (auto &[key, tw] : mTweqs) {
            if (!tw.active) continue;
            if (!shouldTickThisFrame(tw)) continue;

            int result = kTweqStatusQuo;
            const char *typeName = "?";

            switch (tw.type) {
            case kTweqTypeRotate:
                typeName = "Rotate";
                result = processVectorTweq(tw, dt_ms);
                mDirtyObjects.insert(tw.objID);
                break;
            case kTweqTypeScale:
                typeName = "Scale";
                result = processVectorTweq(tw, dt_ms);
                mDirtyObjects.insert(tw.objID);
                break;

            case kTweqTypeFlicker:
                typeName = "Flicker";
                result = processFlickerTweq(tw, dt_ms);
                break;

            case kTweqTypeModels:
                typeName = "Models";
                result = processModelsTweq(tw, dt_ms);
                break;

            case kTweqTypeJoints:
                typeName = "Joints";
                result = processJointsTweq(tw, dt_ms);
                break;

            case kTweqTypeLock:
                typeName = "Lock";
                result = processLockTweq(tw, dt_ms);
                break;

            case kTweqTypeDelete:
                typeName = "Delete";
                result = processDeleteTweq(tw, dt_ms);
                break;

            case kTweqTypeEmitter:
                typeName = "Emitter";
                result = processEmitterTweq(tw, dt_ms);
                break;

            default:
                break;
            }

            // Log first 5 frames per tweq for debugging
            if (tw.logFrames < 1) {
                const ObjectState *os = mObjectStates ? mObjectStates->tryGet(tw.objID) : nullptr;
                std::fprintf(stderr, "[TWEQ] obj=%d type=%s frame=%d "
                             "val=(%.1f,%.1f,%.1f) elapsed=%.0fms curFrame=%d "
                             "hidden=%d hasAnimLight=%d result=%d",
                             tw.objID, typeName, tw.logFrames,
                             tw.values[0], tw.values[1], tw.values[2],
                             tw.elapsedMs, (int)tw.curFrame,
                             tw.flickerHidden ? 1 : 0,
                             tw.hasAnimLight ? 1 : 0, result);
                if (os) {
                    std::fprintf(stderr, " OS: pos=(%.1f,%.1f,%.1f) "
                                 "h=%.3f p=%.3f b=%.3f "
                                 "hasMtx=%d flags=0x%x model='%s'",
                                 os->position.x, os->position.y, os->position.z,
                                 os->heading, os->pitch, os->bank,
                                 os->hasMatrix ? 1 : 0, os->flags,
                                 os->modelNameOverride.c_str());
                } else {
                    std::fprintf(stderr, " OS: (none)");
                }
                // Also log the static placement for comparison
                if (mPlacements) {
                    auto pit = mPlacements->find(tw.objID);
                    if (pit != mPlacements->end()) {
                        const auto &pl = pit->second;
                        std::fprintf(stderr, " PL: pos=(%.1f,%.1f,%.1f) "
                                     "h=%d p=%d b=%d",
                                     pl.x, pl.y, pl.z,
                                     pl.heading, pl.pitch, pl.bank);
                    }
                }
                std::fprintf(stderr, "\n");
                ++tw.logFrames;
            }

            if (result != kTweqStatusQuo && result != kTweqFrameEvent) {
                handleCompletion(tw, result);
            }
        }

        // Apply composed transforms for all dirty objects
        for (int32_t objID : mDirtyObjects) {
            applyComposedTransform(objID);
        }

        // Nothing is iterating mTweqs from here on, so the deferred work that
        // mutates it can run.
        //
        // Teardown first, then spawning: a destroyed object returns its ID to
        // the free stack, so this frame's spawns can reuse it instead of
        // extending the object range.
        if (!mPendingDestroys.empty()) {
            mDestroyBatch.clear();
            mDestroyBatch.swap(mPendingDestroys);
            for (int32_t objID : mDestroyBatch)
                mDestroyCallback(objID);
        }

        // Swap before dispatching: a spawned object can itself carry an
        // emitter, and its emissions must land in the NEXT frame's batch
        // rather than extending this loop indefinitely.
        if (!mPendingEmissions.empty()) {
            mEmissionBatch.clear();
            mEmissionBatch.swap(mPendingEmissions);
            for (const EmitRequest &req : mEmissionBatch)
                mEmitCallback(req);
        }
    }

    // ── Accessors (for tests and diagnostics) ──

    /// Pack objID + tweqType (+ emitter slot) into a unique map key.
    ///
    /// Emitters are the one type an object can carry more than one of — five
    /// slots — so the slot rides in the high nibble of the type byte. Types run
    /// 0..9, so slot 0 gives exactly the old key and every other type is
    /// unaffected.
    static uint64_t tweqKey(int32_t objID, eTweqType type, int slot = 0) {
        return (static_cast<uint64_t>(static_cast<uint32_t>(objID)) << 8) |
               (static_cast<uint64_t>(slot & 0xF) << 4) |
               static_cast<uint64_t>(type);
    }

    const std::unordered_map<uint64_t, TweqInstance> &getTweqs() const { return mTweqs; }
    size_t count() const { return mTweqs.size(); }

    /// Collect all model names referenced by Models tweqs (for pre-loading).
    /// Returns names that may not be in the static uniqueModels set.
    std::vector<std::string> getReferencedModelNames() const {
        std::vector<std::string> names;
        for (const auto &[key, tw] : mTweqs) {
            if (tw.type == kTweqTypeModels) {
                for (int i = 0; i < 6; ++i) {
                    if (tw.modelNames[i][0] != '\0') {
                        names.emplace_back(tw.modelNames[i],
                            strnlen(tw.modelNames[i], 16));
                    }
                }
            }
        }
        return names;
    }

    /// Inject a tweq instance and set the object state map (for unit tests).
    void injectForTest(const TweqInstance &tw, ObjectStateMap *states) {
        mTweqs[tweqKey(tw.objID, tw.type, tw.emitterSlot)] = tw;
        mObjectStates = states;
    }

    /// Get a tweq instance by objID + type (for test assertions).
    const TweqInstance *getInstanceForTest(int32_t objID, eTweqType type) const {
        auto it = mTweqs.find(tweqKey(objID, type));
        return (it != mTweqs.end()) ? &it->second : nullptr;
    }

private:
    // ── Initialization helpers ──

    template <typename CfgT, typename StT>
    void initTweqsOfType(PropertyService *propSvc,
                          const char *cfgPropName, const char *stPropName,
                          eTweqType tweqType, int slot = 0,
                          int32_t onlyObjID = 0,
                          bool activateWhenStateless = false) {
        if (!propSvc || !mPlacements) return;

        for (const auto &[objID, placement] : *mPlacements) {
            if (objID <= 0) continue;  // skip archetypes
            // Runtime registration re-runs the same scan for a single object,
            // so a spawned object gets its tweqs on exactly the path a placed
            // one does — no second, drifting copy of this logic.
            if (onlyObjID != 0 && objID != onlyObjID) continue;

            // Check if this object has the config property (with inheritance)
            CfgT cfg;
            if (!getTypedProperty<CfgT>(propSvc, cfgPropName, objID, cfg))
                continue;

            // Verify raw data size matches struct
            {
                size_t rawSize = 0;
                getPropertyRawData(propSvc, cfgPropName, objID, rawSize);
                if (rawSize != 0 && rawSize != sizeof(CfgT)) {
                    std::fprintf(stderr, "TweqSystem: WARNING: %s size mismatch for "
                                 "obj %d: expected %zu, got %zu\n",
                                 cfgPropName, objID, sizeof(CfgT), rawSize);
                }
            }

            TweqInstance tw;
            tw.objID = objID;
            tw.type = tweqType;
            tw.emitterSlot = slot;
            initFromConfig(tw, cfg, tweqType);

            // Read state property if present
            StT st;
            if (getTypedProperty<StT>(propSvc, stPropName, objID, st)) {
                initFromState(tw, st, tweqType);
            } else if (activateWhenStateless) {
                // A runtime-created object has no authored state BY
                // CONSTRUCTION: tweq state properties are registered "never",
                // so they do not inherit, and nothing in the archetype chain
                // supplies one. "No stored state" therefore means defaults
                // here, not "stored as off" — and a spawned effect whose whole
                // config is a self-destruct countdown has to run it or it never
                // leaves the world.
                //
                // INFERRED, not established: the original's projectile launcher
                // is not in any material available to us, so what it does to a
                // freshly created object's tweqs is unverified. Scoped to the
                // runtime path so placed objects keep their authored,
                // state-driven behaviour either way.
                tw.active = true;
            }

            // Capture base transform from placement data
            initBaseTransform(tw, placement);

            // NO auto-activation from the config word. Whether a tweq is
            // running lives in the STATE word's on/off bit, which
            // initFromState has already applied above; the config word's
            // SIM bit is an update-rate gate ("tick me continually rather than
            // only while I am on screen"), sitting in the same family as the
            // small-radius, large-radius and offscreen gates.
            //
            // Treating SIM as auto-start ran tweqs the mission had stored as
            // OFF: all 7 shipped lock configs carry it, so 424 locks drove
            // themselves to the unlocked pose within ~0.3 s of mission start,
            // and MISS13's 80 emitter states are stored off yet every one of
            // them began emitting at load.

            // The two radius gates stay unapplied — see shouldTickThisFrame.
            if (tw.cfgAnim & (kTweqAnimSimSmallRadius | kTweqAnimSimLargeRadius)) {
                static int warnCount = 0;
                if (warnCount++ < 3)
                    std::fprintf(stderr, "[FALLBACK] TweqSystem: obj %d has a "
                                 "radius update gate (anim=0x%02x) that is not "
                                 "applied — it ticks whenever it is on screen\n",
                                 objID, tw.cfgAnim);
            }

            // Check if this object has an AnimLight property. Flicker tweqs on
            // objects with AnimLights should toggle the light, not hide the object.
            // (Light toggling is deferred — for now, just skip visibility toggle.)
            if (tweqType == kTweqTypeFlicker) {
                size_t animSize = 0;
                if (getPropertyRawData(propSvc, "AnimLight", objID, animSize)) {
                    tw.hasAnimLight = true;
                }
            }

            // Initialize current values for Scale tweqs (start at 1.0, not 0.0)
            if (tweqType == kTweqTypeScale) {
                tw.values[0] = tw.base.scale.x;
                tw.values[1] = tw.base.scale.y;
                tw.values[2] = tw.base.scale.z;
            }

            // A lock's joint follows the object's lock state: locked sits at
            // `low`, unlocked at `high`. Confirmed both ways — the engine's
            // "set all my lock joint positions appropriately" command writes
            // exactly that, and in shipped data every object carrying P$Locked
            // has its stored value equal to `low` (10/10 in MISS6).
            // [BIN: FUN_00595C40, Thief2.exe NewDark 1.28]
            //
            // The stored state wins when present: it is the authored pose, and
            // the engine ships that command precisely because the two can drift.
            if (tweqType == kTweqTypeLock) {
                PropStTweqLock st;
                if (!getTypedProperty<PropStTweqLock>(propSvc, stPropName,
                                                       objID, st)) {
                    PropLocked locked{};
                    const bool isLocked =
                        getTypedProperty<PropLocked>(propSvc, "Locked", objID,
                                                     locked) &&
                        locked.isLocked != 0;
                    tw.values[0] = isLocked ? tw.axes[0].low : tw.axes[0].high;
                }
                writeJointPose(tw, /*seedAll=*/true);
            }

            // Joints start wherever JointPos left them (a lever authored
            // thrown, a chest authored open), not at zero.
            if (tweqType == kTweqTypeJoints) {
                PropJointPos jp;
                if (getTypedProperty<PropJointPos>(propSvc, "JointPos", objID, jp)) {
                    for (int i = 0; i < kJointSlotCount; ++i)
                        tw.values[i] = jp.joint[i];
                }
                writeJointPose(tw, /*seedAll=*/true);
            }

            // Initialize current values for Rotate tweqs from base angles
            if (tweqType == kTweqTypeRotate) {
                // Extract current angles from base rotation matrix (in degrees)
                // The tweq operates in degrees matching the Dark Engine convention
                static constexpr float kAngScale = 2.0f * 3.14159265f / 65536.0f;
                static constexpr float kRadToDeg = 180.0f / 3.14159265f;
                tw.values[0] = static_cast<float>(placement.bank)    * kAngScale * kRadToDeg;
                tw.values[1] = static_cast<float>(placement.pitch)   * kAngScale * kRadToDeg;
                tw.values[2] = static_cast<float>(placement.heading) * kAngScale * kRadToDeg;
            }

            // Compute base anchor Z from the first model (used as fixed reference
            // for anchor compensation — avoids Z bobbing between model variants).
            if (tweqType == kTweqTypeModels && tw.modelCount > 0 &&
                (tw.cfgMisc & kTweqMiscAnchor) && mParsedModels) {
                tw.baseAnchorZ = getModelBBoxBottomZ(
                    std::string(tw.modelNames[0], strnlen(tw.modelNames[0], 16)));
            }

            // For active Models tweqs, apply the initial model immediately so
            // objects with staticModel='' don't flash as invisible on the first frame.
            if (tweqType == kTweqTypeModels && tw.active && tw.modelCount > 0) {
                int frame = std::clamp(static_cast<int>(tw.curFrame), 0, tw.modelCount - 1);
                if (tw.modelNames[frame][0] != '\0' && mObjectStates) {
                    ensureObjectState(tw);
                    ObjectState &os = mObjectStates->get(tw.objID);
                    os.modelNameOverride = std::string(tw.modelNames[frame],
                        strnlen(tw.modelNames[frame], 16));
                }
            }

            // Log Models tweq config at init for debugging
            if (tweqType == kTweqTypeModels) {
                // Dump raw property bytes to verify struct alignment
                size_t rawSz = 0;
                const uint8_t *raw = getPropertyRawData(propSvc, cfgPropName, objID, rawSz);
                if (raw && rawSz >= 8) {
                    std::fprintf(stderr, "  TweqModels obj=%d RAW[%zu]: hdr=%02x %02x %02x %02x %04x %04x names=",
                                 objID, rawSz, raw[0], raw[1], raw[2], raw[3],
                                 *reinterpret_cast<const uint16_t*>(raw+4),
                                 *reinterpret_cast<const uint16_t*>(raw+6));
                    // Print each 16-byte model name slot
                    for (int s = 0; s < 6 && (8 + s*16) < (int)rawSz; ++s) {
                        char name[17] = {};
                        std::memcpy(name, raw + 8 + s*16, 16);
                        std::fprintf(stderr, "[%d]='%s' ", s, name);
                    }
                    std::fprintf(stderr, "\n");
                }
                std::fprintf(stderr, "  TweqModels obj=%d: staticModel='%.16s' rate=%dms models=[",
                             objID, placement.modelName, tw.cfgRate);
                for (int i = 0; i < 6; ++i) {
                    if (tw.modelNames[i][0] != '\0')
                        std::fprintf(stderr, "'%s'%s", tw.modelNames[i],
                                     i < 5 && tw.modelNames[i+1][0] != '\0' ? "," : "");
                }
                std::fprintf(stderr, "] count=%d anim=0x%02x misc=0x%03x halt=%d active=%d",
                             tw.modelCount, tw.cfgAnim, tw.cfgMisc, tw.cfgHalt, tw.active ? 1 : 0);
                // Flag decode
                if (tw.cfgMisc & kTweqMiscAnchor) std::fprintf(stderr, " ANCHOR");
                if (tw.cfgMisc & kTweqMiscVHot) std::fprintf(stderr, " VHOT");
                if (tw.cfgMisc & kTweqMiscScripts) std::fprintf(stderr, " SCRIPTS");
                std::fprintf(stderr, "\n");
            }

            mTweqs[tweqKey(objID, tweqType, slot)] = std::move(tw);
        }
    }

    /// Extract config fields into TweqInstance (vector config: Rotate/Scale)
    void initFromConfig(TweqInstance &tw, const PropCfgTweqVector &cfg, eTweqType) {
        tw.cfgCurve = cfg.curve;
        tw.cfgAnim  = cfg.anim;
        tw.cfgHalt  = cfg.halt;
        tw.cfgMisc  = cfg.misc;
        tw.axes[0]  = cfg.x;
        tw.axes[1]  = cfg.y;
        tw.axes[2]  = cfg.z;
        tw.primaryAxis = cfg.primary;
    }

    /// Extract config fields (joints config)
    ///
    /// Each joint carries its own flag block on top of rate/low/high. The
    /// per-joint blocks in shipped data repeat the header's curve/anim, so the
    /// shared per-axis algorithm reads the header flags exactly as the vector
    /// tweqs do; only rate/low/high are genuinely per-joint.
    void initFromConfig(TweqInstance &tw, const PropCfgTweqJoints &cfg, eTweqType) {
        tw.cfgCurve = cfg.curve;
        tw.cfgAnim  = cfg.anim;
        tw.cfgHalt  = cfg.halt;
        tw.cfgMisc  = cfg.misc;
        tw.axisCount = kJointSlotCount;
        for (int i = 0; i < kJointSlotCount; ++i) {
            tw.axes[i].rate = cfg.joint[i].rate;
            tw.axes[i].low  = cfg.joint[i].low;
            tw.axes[i].high = cfg.joint[i].high;
        }
        tw.primaryAxis = cfg.primary;
    }

    /// Extract config fields (lock config)
    ///
    /// One axis: the joint the lock drives, between `low` (locked) and `high`
    /// (unlocked). `targetJoint` is read as a byte and decremented when
    /// non-zero, so 0 and 1 both name slot 0.
    void initFromConfig(TweqInstance &tw, const PropCfgTweqLock &cfg, eTweqType) {
        tw.cfgCurve = cfg.curve;
        tw.cfgAnim  = cfg.anim;
        tw.cfgHalt  = cfg.halt;
        tw.cfgMisc  = cfg.misc;
        tw.axisCount = 1;
        tw.axes[0].rate = cfg.lock.rate;
        tw.axes[0].low  = cfg.lock.low;
        tw.axes[0].high = cfg.lock.high;

        int slot = lockJointSlotFromConfig(cfg.targetJoint);
        if (slot < 0 || slot >= kJointSlotCount) {
            std::fprintf(stderr, "[FALLBACK] TweqSystem: obj %d lock tweq targets "
                         "joint slot %d, outside the %d the property stores — "
                         "clamping to 0, the lock will drive the wrong part\n",
                         tw.objID, slot, kJointSlotCount);
            slot = 0;
        }
        tw.lockJointSlot = slot;
    }

    /// Extract state fields (lock state)
    void initFromState(TweqInstance &tw, const PropStTweqLock &st, eTweqType) {
        if (st.anim & kTweqStateOn) tw.active = true;
        tw.axisState[0] = st.axisState;
        tw.values[0] = st.value;
        seedReverseFromBase(tw, st.anim);
    }

    /// Extract config fields (emitter config)
    void initFromConfig(TweqInstance &tw, const PropCfgTweqEmitter &cfg,
                        eTweqType) {
        tw.cfgCurve = cfg.curve;
        tw.cfgAnim  = cfg.anim;
        tw.cfgHalt  = cfg.halt;
        tw.cfgMisc  = cfg.misc;
        tw.cfgRate  = cfg.rate;
        std::memcpy(tw.emitWhat, cfg.emitWhat, 16);
        tw.emitWhat[16] = '\0';
        tw.emitVelocity = Vector3(cfg.velocity[0], cfg.velocity[1],
                                  cfg.velocity[2]);
        tw.emitAngleRandom = Vector3(cfg.angleRandom[0], cfg.angleRandom[1],
                                     cfg.angleRandom[2]);
        // curFrame counts emissions down to zero, the way the flicker tweq
        // counts its toggles.
        tw.curFrame = static_cast<int16_t>(
            std::clamp<int32_t>(cfg.maxFrames, 0, 32767));
        if (cfg.rate == 0) {
            std::fprintf(stderr, "[DEFAULT] TweqSystem: obj %d Emitter cfgRate=0 "
                         "with Sim flag — would emit every tick\n", tw.objID);
        }
    }

    /// Extract config fields (simple config: Flicker)
    void initFromConfig(TweqInstance &tw, const PropCfgTweqSimple &cfg, eTweqType) {
        tw.cfgCurve = cfg.curve;
        tw.cfgAnim  = cfg.anim;
        tw.cfgHalt  = cfg.halt;
        tw.cfgMisc  = cfg.misc;
        tw.cfgRate  = cfg.rate;
        if (cfg.rate == 0) {
            std::fprintf(stderr, "[DEFAULT] TweqSystem: obj %d Flicker cfgRate=0 with Sim flag — instant cycle, likely parse issue\n", tw.objID);
        }
    }

    /// Extract config fields (Models config)
    void initFromConfig(TweqInstance &tw, const PropCfgTweqModels &cfg, eTweqType) {
        tw.cfgCurve = cfg.curve;
        tw.cfgAnim  = cfg.anim;
        tw.cfgHalt  = cfg.halt;
        tw.cfgMisc  = cfg.misc;
        tw.cfgRate  = cfg.rate;
        if (cfg.rate == 0) {
            std::fprintf(stderr, "[DEFAULT] TweqSystem: obj %d Models cfgRate=0 with Sim flag — instant cycle, likely parse issue\n", tw.objID);
        }
        // Copy model names. modelCount = index of first empty slot (not last
        // non-empty). The Dark Engine stops cycling at the first empty name;
        // slots after the gap (e.g., mecgas0 at [5] after empty [4]) are
        // script-accessible state variants, not part of the animation cycle.
        tw.modelCount = 0;
        for (int i = 0; i < 6; ++i) {
            std::memcpy(tw.modelNames[i], cfg.modelName[i], 16);
            tw.modelNames[i][15] = '\0';
        }
        for (int i = 0; i < 6; ++i) {
            if (tw.modelNames[i][0] == '\0') break;
            tw.modelCount = i + 1;
        }
    }

    /// Seed per-axis direction from the state word's Reverse bit.
    ///
    /// The base `anim` word carries the tweq's direction; the per-axis words
    /// carry each axis's own. Reading only the per-axis words dropped the
    /// direction for every tweq that stores it in the base word alone — 565 of
    /// 1300 shipped lock states do exactly that (typically anim=0x0003 with a
    /// zero axis word), so those locks, parked at their locked position, ran
    /// FORWARD to the unlocked pose instead of staying put.
    static void seedReverseFromBase(TweqInstance &tw, uint16_t animWord) {
        if (!(animWord & kTweqStateReverse)) return;
        for (int i = 0; i < tw.axisCount && i < TweqInstance::kMaxAxes; ++i)
            tw.axisState[i] |= static_cast<uint32_t>(kTweqStateReverse);
    }

    /// Extract state fields (vector state: Rotate/Scale)
    void initFromState(TweqInstance &tw, const PropStTweqVector &st, eTweqType) {
        if (st.anim & kTweqStateOn) tw.active = true;
        tw.axisState[0] = st.x;
        tw.axisState[1] = st.y;
        tw.axisState[2] = st.z;
        seedReverseFromBase(tw, st.anim);
        // Per-axis reverse flags from state
        for (int i = 0; i < 3; ++i) {
            if (tw.axisState[i] & kTweqStateReverse) {
                // Will be handled via axisState during processing
            }
        }
    }

    /// Extract state fields (joints state)
    void initFromState(TweqInstance &tw, const PropStTweqJoints &st, eTweqType) {
        if (st.anim & kTweqStateOn) tw.active = true;
        for (int i = 0; i < kJointSlotCount; ++i)
            tw.axisState[i] = st.joint[i];
        seedReverseFromBase(tw, st.anim);
    }

    /// Extract state fields (simple state: Flicker/Models)
    void initFromState(TweqInstance &tw, const PropStTweqSimple &st, eTweqType) {
        if (st.anim & kTweqStateOn) tw.active = true;
        seedReverseFromBase(tw, st.anim);
        tw.elapsedMs = static_cast<float>(st.time);
        tw.curFrame  = static_cast<int16_t>(st.frame);
    }

    void initBaseTransform(TweqInstance &tw, const ObjPlacementInfo &pl) {
        tw.base.position = Vector3(pl.x, pl.y, pl.z);
        tw.base.scale = Vector3(pl.sx, pl.sy, pl.sz);

        // Build rotation matrix from binary radians (same as DoorSystem)
        static constexpr float kAngScale = 2.0f * 3.14159265f / 65536.0f;
        float h = static_cast<float>(pl.heading) * kAngScale;
        float p = static_cast<float>(pl.pitch)   * kAngScale;
        float b = static_cast<float>(pl.bank)     * kAngScale;
        tw.base.rotation = glm::eulerAngleZYX(h, p, b);
    }

    // ── Activation ──

    void applyAction(TweqInstance &tw, TweqAction action) {
        switch (action) {
        case kTweqDoActivate:
        case kTweqDoContinue:
            if (!tw.active) {
                tw.active = true;
                tw.elapsedMs = 0.0f;
            }
            break;

        case kTweqDoHalt:
            tw.active = false;
            break;

        case kTweqDoDefault:
            if (tw.active)
                tw.active = false;
            else {
                tw.active = true;
                tw.elapsedMs = 0.0f;
            }
            break;

        case kTweqDoReset:
            tw.active = false;
            tw.elapsedMs = 0.0f;
            tw.curFrame = 0;
            for (int i = 0; i < tw.axisCount; ++i)
                tw.axisState[i] &= ~static_cast<uint32_t>(kTweqStateReverse);
            break;

        case kTweqDoForward:
            tw.active = true;
            tw.elapsedMs = 0.0f;
            for (int i = 0; i < tw.axisCount; ++i)
                tw.axisState[i] &= ~static_cast<uint32_t>(kTweqStateReverse);
            break;

        case kTweqDoReverse:
            tw.active = true;
            tw.elapsedMs = 0.0f;
            for (int i = 0; i < tw.axisCount; ++i)
                tw.axisState[i] |= static_cast<uint32_t>(kTweqStateReverse);
            break;
        }
    }

    // ── Core axis processing (Dark Engine processTweqAxis algorithm) ──

    /// Process a single axis. Returns kTweqStatusQuo if still running,
    /// or a TweqHaltAction value if the axis reached its bounds and completed.
    int processAxis(TweqInstance &tw, int axisIdx, float dt_ms) {
        const PropTweqAxisConfig &cfg = tw.axes[axisIdx];

        // Skip axes with zero rate (inactive)
        if (std::abs(cfg.rate) < 1e-6f) return kTweqStatusQuo;

        float eff_rate = cfg.rate;
        bool isReverse = (tw.axisState[axisIdx] & kTweqStateReverse) != 0;
        if (isReverse) eff_rate *= -1.0f;

        // Dark Engine time step normalization: step = ms / 100.0
        float step = dt_ms / 100.0f;

        float new_val = tw.values[axisIdx];

        if (tw.cfgCurve & kTweqCurveMul) {
            // Multiplicative mode
            if (tw.cfgCurve & kTweqCurveJitterMask) {
                float delta = 0.05f + std::abs(1.0f - eff_rate);
                float fac = static_cast<float>(tw.cfgCurve & kTweqCurveJitterMask);
                float r = randFloat();
                delta = 1.0f + (delta * fac * r / 2.0f);
                eff_rate *= delta;
            }
            new_val *= eff_rate;
        } else {
            // Additive mode (most common)
            new_val += eff_rate * step;
            if (tw.cfgCurve & kTweqCurveJitterMask) {
                float fac = static_cast<float>(tw.cfgCurve & kTweqCurveJitterMask);
                new_val += eff_rate * randFloat() * fac * step / 2.0f;
            }
        }

        // Check bounds (unless NoLimit flag is set)
        if (!(tw.cfgAnim & kTweqAnimNoLimit)) {
            float lo = std::min(cfg.low, cfg.high);
            float hi = std::max(cfg.low, cfg.high);

            int clip = 0;
            if (new_val < lo) clip = 1;       // below low bound
            else if (new_val > hi) clip = 2;  // above high bound

            if (clip != 0) {
                // Check end condition: OneBounce completes only when returning
                // to the starting edge (reverse flag will be set on first hit)
                bool isEnd = true;
                if (tw.cfgAnim & kTweqAnimOneBounce) {
                    if (!isReverse) isEnd = false;  // first half, continue
                }

                if (tw.cfgAnim & kTweqAnimWrap) {
                    // Wrap to opposite edge
                    new_val = (clip == 1) ? hi : lo;
                    // Wrapping never triggers completion on its own
                } else {
                    // Bounce: clamp to limit and reverse direction
                    new_val = (clip == 1) ? lo : hi;
                    tw.axisState[axisIdx] ^= static_cast<uint32_t>(kTweqStateReverse);
                }

                tw.values[axisIdx] = new_val;
                if (isEnd) return tw.cfgHalt;
            }
        }

        tw.values[axisIdx] = new_val;
        return kTweqStatusQuo;
    }

    // ── Type-specific processors ──

    int processVectorTweq(TweqInstance &tw, float dt_ms) {
        int result = kTweqStatusQuo;

        // Copy global state to primary axis (Dark Engine convention)
        if (tw.primaryAxis > 0 && tw.primaryAxis <= 3) {
            // primaryAxis is 1-indexed (1=X, 2=Y, 3=Z)
            // No need to copy here — axisState is already per-axis
        }

        for (int i = 0; i < tw.axisCount; ++i) {
            int rv = processAxis(tw, i, dt_ms);
            // Primary axis (or all axes if primary=0) controls completion
            if (tw.primaryAxis == 0 || i == (tw.primaryAxis - 1)) {
                if (rv != kTweqStatusQuo) result = rv;
            }
        }

        return result;
    }

    /// Publish a joints tweq's current values to the object's render pose.
    ///
    /// Joints move parts INSIDE the model, never the object itself, so this
    /// deliberately does not touch the object transform and does not mark the
    /// object dirty for applyComposedTransform.
    /// Publish a tweq's joint values to the object's render pose.
    ///
    /// Every tweq writes ONLY the slots it drives. A Joints tweq used to write
    /// all six, which put it in a fight with the Lock tweq on the same object:
    /// 171 shipped objects carry both, and on `chestloc` the joints config
    /// defines j1 alone, so it rewrote joints[0] = 0 every tick and pinned the
    /// very lock plate the Lock tweq was driving. Whichever ran last won, and
    /// that is unordered_map iteration order — hash-dependent, and not stable
    /// across builds or platforms.
    ///
    /// `seedAll` is for the one-time init pose, where the values come from
    /// JointPos and every slot is legitimately this tweq's to place.
    void writeJointPose(TweqInstance &tw, bool seedAll = false) {
        if (!mObjectStates) return;
        static_assert(ObjectState::kJointSlots == kJointSlotCount,
                      "ObjectState joint array must match P$JointPos slot count");

        ensureObjectState(tw);
        ObjectState &os = mObjectStates->get(tw.objID);
        if (tw.type == kTweqTypeLock) {
            os.joints[tw.lockJointSlot] = tw.values[0];
        } else if (seedAll) {
            for (int i = 0; i < kJointSlotCount; ++i)
                os.joints[i] = tw.values[i];
        } else {
            // Only the slots this tweq actually drives. Read from the live
            // rate rather than a cached mask so the two cannot desync —
            // processAxis skips a zero-rate slot outright, so such a slot is
            // not this tweq's to write either.
            for (int i = 0; i < kJointSlotCount && i < tw.axisCount; ++i)
                if (std::fabs(tw.axes[i].rate) > 1e-6f)
                    os.joints[i] = tw.values[i];
        }
        os.hasJoints = true;
    }

    /// Apply P$JointPos to objects that have a stored pose but no joints tweq.
    /// Objects with a tweq are seeded from the same property in
    /// initTweqsOfType, so they are skipped here.
    void seedStaticJointPositions(PropertyService *propSvc) {
        if (!propSvc || !mObjectStates || !mPlacements) return;

        int seeded = 0;
        for (const auto &[objID, placement] : *mPlacements) {
            if (objID <= 0) continue;
            // A joints or lock tweq has already published this object's pose
            // from the same property (or, for a lock, from its lock state).
            const ObjectState *existing = mObjectStates->tryGet(objID);
            if (existing && existing->hasJoints) continue;

            PropJointPos jp;
            if (!getTypedProperty<PropJointPos>(propSvc, "JointPos", objID, jp))
                continue;

            // An all-zero pose is the rest pose the renderer already draws;
            // creating an ObjectState for it would only cost work per frame.
            bool anyNonZero = false;
            for (int i = 0; i < kJointSlotCount; ++i)
                if (std::fabs(jp.joint[i]) > 1e-6f) anyNonZero = true;
            if (!anyNonZero) continue;

            // Seed the transform only when we are the ones creating the entry.
            // Another system may already own this object's pose (a door, a
            // moving platform) and resetting it to the placement would undo
            // whatever it did.
            const bool fresh = (mObjectStates->tryGet(objID) == nullptr);
            ObjectState &os = mObjectStates->get(objID);
            if (fresh) {
                os.initFromBinaryRadians(placement.x, placement.y, placement.z,
                                         placement.heading, placement.pitch,
                                         placement.bank,
                                         placement.sx, placement.sy, placement.sz);
            }
            for (int i = 0; i < kJointSlotCount; ++i)
                os.joints[i] = jp.joint[i];
            os.hasJoints = true;
            ++seeded;
        }

        if (seeded > 0)
            std::fprintf(stderr, "TweqSystem: %d objects seeded from JointPos "
                         "(no joints tweq)\n", seeded);
    }

    /// An emitter spawns one object every `cfgRate` milliseconds until its
    /// frame budget runs out, then takes its halt action — usually deleting
    /// itself, since most emitters are one-shot effect spawners.
    int processEmitterTweq(TweqInstance &tw, float dt_ms) {
        if (tw.cfgRate == 0 && !(tw.cfgAnim & kTweqAnimNoLimit)) {
            // Guard the degenerate config the [DEFAULT] warning at init flags:
            // a zero rate with a frame budget would drain it in one tick.
            tw.active = false;
            return tw.cfgHalt;
        }

        tw.elapsedMs += dt_ms;
        if (tw.elapsedMs < static_cast<float>(tw.cfgRate))
            return kTweqStatusQuo;
        tw.elapsedMs -= static_cast<float>(tw.cfgRate);

        emitOne(tw);

        if (!(tw.cfgAnim & kTweqAnimNoLimit)) {
            --tw.curFrame;
            if (tw.curFrame <= 0) {
                tw.active = false;
                return tw.cfgHalt;
            }
        }
        return kTweqFrameEvent;
    }

    /// Build one emission and hand it to the spawner.
    void emitOne(TweqInstance &tw) {
        if (tw.emitWhat[0] == '\0') {
            static int warnCount = 0;
            if (warnCount++ < 5)
                std::fprintf(stderr, "[FALLBACK] TweqSystem: obj %d emitter %d has "
                             "no archetype name — emitting nothing\n",
                             tw.objID, tw.emitterSlot);
            return;
        }

        EmitRequest req;
        req.emitterObjID = tw.objID;
        req.emitterSlot = tw.emitterSlot;
        req.archetypeName.assign(tw.emitWhat, strnlen(tw.emitWhat, 16));
        req.miscFlags = tw.cfgMisc;
        req.position = tw.base.position;
        if (mObjectStates) {
            if (const ObjectState *os = mObjectStates->tryGet(tw.objID))
                req.position = os->position;
        }

        // ZeroVel wins outright; otherwise the authored velocity, rotated into
        // the emitter's frame when RelVel is set, then spread by AngleRandom.
        if (!(tw.cfgMisc & kTweqMiscZeroVel)) {
            Vector3 v = tw.emitVelocity;
            if (tw.cfgMisc & kTweqMiscRelVel)
                v = Vector3(tw.base.rotation * glm::vec4(v, 0.0f));

            // Angle Random is an ANGLE, per the engine's own field label, and
            // it is authored in degrees like every other rotational tweq
            // value: `sword_hilt`, `doorknob` and `junklever` all ship
            // (359, 359, 359) — the same "almost a full turn" idiom JointPos
            // uses — against a velocity of (0, 0, -0.1). Adding those as
            // linear velocity, which this used to do, launches the emitted
            // object out of the level at 359 units/s.
            //
            // Applied here as a random per-axis rotation of the emission
            // direction. That the field is an angle is established; that this
            // is the engine's exact construction is NOT — a cone about the
            // velocity is equally plausible. Revisit if emitted objects ever
            // get integrated velocity to compare against.
            const Vector3 spreadDeg(tw.emitAngleRandom.x * randFloat(),
                                    tw.emitAngleRandom.y * randFloat(),
                                    tw.emitAngleRandom.z * randFloat());
            if (glm::length(tw.emitAngleRandom) > 1e-6f &&
                glm::length(v) > 1e-6f) {
                const float kDegToRad = 3.14159265358979f / 180.0f;
                const Matrix4 spread =
                    glm::eulerAngleZYX(spreadDeg.z * kDegToRad,
                                       spreadDeg.y * kDegToRad,
                                       spreadDeg.x * kDegToRad);
                v = Vector3(spread * glm::vec4(v, 0.0f));
            }
            req.velocity = v;
        }

        ++mEmitFired;
        if (mEmitCallback) {
            // Deferred, NOT dispatched here. The spawner registers the new
            // object's tweqs, which inserts into mTweqs — and this runs from
            // inside simStep's range-for over that same map. Mutating it mid
            // iteration is undefined behaviour, and in practice rehashing
            // silently skipped live tweqs (measured: 9-16 of 16 emitters
            // serviced per frame instead of 16).
            mPendingEmissions.push_back(std::move(req));
        } else {
            static int warnCount = 0;
            if (warnCount++ < 3)
                std::fprintf(stderr, "[FALLBACK] TweqSystem: obj %d emitting '%s' "
                             "with no emit callback wired — the tweq runs but "
                             "nothing spawns\n",
                             tw.objID, req.archetypeName.c_str());
        }
    }

    int processJointsTweq(TweqInstance &tw, float dt_ms) {
        int result = processVectorTweq(tw, dt_ms);
        writeJointPose(tw);
        return result;
    }

    /// A lock tweq animates one model joint — a chest's lock plate, a safe's
    /// dial, a door's handle — between the locked and unlocked positions.
    int processLockTweq(TweqInstance &tw, float dt_ms) {
        int result = processAxis(tw, 0, dt_ms);
        writeJointPose(tw);
        return result;
    }

    /// A delete tweq is a countdown that removes its object when it expires.
    /// It carries no per-axis config at all: `cfgRate` milliseconds, then the
    /// halt action (Destroy / Slay / Stop) decides what happens.
    ///
    /// This is how one-shot effects clean themselves up — fire and moss arrow
    /// residue, particle bursts, flares, frost shards, spent waypoints.
    int processDeleteTweq(TweqInstance &tw, float dt_ms) {
        tw.elapsedMs += dt_ms;
        if (tw.elapsedMs < static_cast<float>(tw.cfgRate))
            return kTweqStatusQuo;

        tw.elapsedMs = 0.0f;
        tw.active = false;
        // An object disappearing is worth seeing: if a placed object vanishes
        // that the original kept, this is where it went.
        if (mDeleteFired < 20)
            std::fprintf(stderr, "[TWEQ_DELETE] obj %d expired after %u ms, "
                         "halt action %u\n", tw.objID, tw.cfgRate, tw.cfgHalt);
        ++mDeleteFired;
        return tw.cfgHalt;
    }

    /// Ensure an ObjectState entry exists with the correct base transform.
    /// Called before flicker/model tweqs modify flags or modelNameOverride,
    /// because mObjectStates->get() creates a default entry at position (0,0,0)
    /// which would make the renderer place the object at the origin.
    ///
    /// Uses initFromBinaryRadians() with the raw int16 placement angles so the
    /// renderer's bx::mtxRotateXYZ fallback path produces a bit-identical matrix
    /// to the static render path. Do NOT set hasMatrix — let the renderer build
    /// the bx matrix itself to avoid GLM/bx convention mismatches.
    void ensureObjectState(TweqInstance &tw) {
        if (!mObjectStates || tw.hasObjectState) return;
        tw.hasObjectState = true;

        auto it = mPlacements ? mPlacements->find(tw.objID) : decltype(mPlacements->end()){};
        if (mPlacements && it != mPlacements->end()) {
            const auto &pl = it->second;
            mObjectStates->get(tw.objID).initFromBinaryRadians(
                pl.x, pl.y, pl.z, pl.heading, pl.pitch, pl.bank,
                pl.sx, pl.sy, pl.sz);
        } else {
            // Fallback: set position from SimTransform
            std::fprintf(stderr, "[FALLBACK] TweqSystem: obj %d not in placements map, using base transform pos=(%.1f,%.1f,%.1f)\n",
                         tw.objID, tw.base.position.x, tw.base.position.y, tw.base.position.z);
            ObjectState &os = mObjectStates->get(tw.objID);
            os.position = tw.base.position;
            os.scale = tw.base.scale;
        }
    }

    int processFlickerTweq(TweqInstance &tw, float dt_ms) {
        tw.elapsedMs += dt_ms;

        if (tw.elapsedMs >= static_cast<float>(tw.cfgRate)) {
            tw.elapsedMs -= static_cast<float>(tw.cfgRate);

            // Toggle visibility — but only for objects without AnimLights.
            // Torches/candles have AnimLights; the flicker should control the
            // light intensity, not hide the model. Light toggling is deferred
            // (needs LightingSystem API). For now, AnimLight objects just get
            // the frame event for script/callback use.
            tw.flickerHidden = !tw.flickerHidden;
            if (mObjectStates && !tw.hasAnimLight) {
                ensureObjectState(tw);
                ObjectState &os = mObjectStates->get(tw.objID);
                if (tw.flickerHidden)
                    os.flags |= kObjStateHidden;
                else
                    os.flags &= ~kObjStateHidden;
            }

            // Check frame limit
            if (!(tw.cfgAnim & kTweqAnimNoLimit)) {
                tw.curFrame--;
                if (tw.curFrame <= 0) return tw.cfgHalt;
            }

            return kTweqFrameEvent;
        }

        return kTweqStatusQuo;
    }

    int processModelsTweq(TweqInstance &tw, float dt_ms) {
        if (tw.modelCount <= 0) return kTweqStatusQuo;

        tw.elapsedMs += dt_ms;
        int result = kTweqStatusQuo;

        // Duration per model frame (rate + 1 ms, matching Dark Engine)
        float duration = static_cast<float>(tw.cfgRate + 1);

        // Process elapsed time. Continue on kTweqHaltContinue (3) because
        // bounce/wrap returns it but the tweq should keep running.
        while ((result == kTweqStatusQuo || result == static_cast<int>(kTweqHaltContinue))
               && tw.elapsedMs >= duration) {
            result = kTweqStatusQuo;  // reset for next iteration
            tw.elapsedMs -= duration;

            bool isReverse = (tw.axisState[0] & kTweqStateReverse) != 0;

            if (tw.cfgMisc & kTweqMiscRandom) {
                // Random frame selection
                tw.curFrame = static_cast<int16_t>(
                    std::uniform_int_distribution<int>(0, tw.modelCount - 1)(mRng));
            } else if (isReverse) {
                if (tw.curFrame > 0) {
                    tw.curFrame--;
                } else {
                    result = hitModelEdge(tw, false);
                }
            } else {
                if (tw.curFrame < tw.modelCount - 1) {
                    tw.curFrame++;
                } else {
                    result = hitModelEdge(tw, true);
                }
            }
        }

        // Always apply the model name override for the current frame.
        {
            int frame = std::clamp(static_cast<int>(tw.curFrame), 0, tw.modelCount - 1);
            std::string newModelName(tw.modelNames[frame],
                strnlen(tw.modelNames[frame], 16));
            if (mObjectStates && !newModelName.empty()) {
                ensureObjectState(tw);
                ObjectState &os = mObjectStates->get(tw.objID);

                // Anchor compensation: when the ANCHOR flag is set, adjust the
                // object's Z position to keep the bottom of the bounding box
                // fixed. Different flame model variants have different bbox
                // heights, so without this the flame bobs up and down.
                // Matches Dark Engine get_anchor/finalize_anchor logic.
                // Anchor compensation: adjust Z so the bbox bottom stays at a
                // fixed height. Uses baseAnchorZ (first model's bbox bottom)
                // as a constant reference to avoid per-swap Z bobbing.
                if ((tw.cfgMisc & kTweqMiscAnchor) && mParsedModels) {
                    float newAnchorZ = getModelBBoxBottomZ(newModelName);
                    // Reset to placement Z, then apply offset from base model
                    os.position.z = tw.base.position.z + (tw.baseAnchorZ - newAnchorZ);
                }

                // Log model swap with GPU lookup check
                if (tw.logFrames < 1 && mParsedModels) {
                    bool parsed = mParsedModels->count(newModelName) > 0;
                    std::fprintf(stderr, "[TWEQ_MODEL] obj=%d swap '%s' -> '%s' "
                                 "parsed=%d pos=(%.2f,%.2f,%.2f)\n",
                                 tw.objID,
                                 os.modelNameOverride.c_str(),
                                 newModelName.c_str(),
                                 parsed ? 1 : 0,
                                 os.position.x, os.position.y, os.position.z);
                }
                os.modelNameOverride = std::move(newModelName);
            }
        }

        return result;
    }

    int hitModelEdge(TweqInstance &tw, bool atHigh) {
        if (tw.cfgAnim & kTweqAnimWrap) {
            // Wrap to opposite edge
            tw.curFrame = atHigh ? 0 : static_cast<int16_t>(tw.modelCount - 1);
            return kTweqStatusQuo;
        } else {
            // Bounce: reverse direction
            tw.axisState[0] ^= static_cast<uint32_t>(kTweqStateReverse);

            bool isEnd = true;
            if (tw.cfgAnim & kTweqAnimOneBounce) {
                if (!(tw.axisState[0] & kTweqStateReverse))
                    isEnd = true;  // completed full bounce
                else
                    isEnd = false; // first half
            }

            if (isEnd) return tw.cfgHalt;
            return kTweqStatusQuo;
        }
    }

    // ── Transform application ──

    /// Compose all active tweqs for an object into one transform and write to ObjectState.
    void applyComposedTransform(int32_t objID) {
        if (!mObjectStates) return;

        // Find base transform and accumulate tweq effects
        const SimTransform *base = nullptr;
        Vector3 tweqRotDeg(0.0f);       // accumulated rotation offset in degrees
        Vector3 tweqScale(1.0f);         // accumulated scale multiplier
        bool hasRotate = false;
        bool hasScale = false;

        for (auto &[key, tw] : mTweqs) {
            if (tw.objID != objID) continue;
            if (!base) base = &tw.base;

            if (tw.type == kTweqTypeRotate && tw.active) {
                tweqRotDeg = Vector3(tw.values[0], tw.values[1], tw.values[2]);
                hasRotate = true;
            } else if (tw.type == kTweqTypeScale && tw.active) {
                tweqScale = Vector3(tw.values[0], tw.values[1], tw.values[2]);
                hasScale = true;
            }
        }

        if (!base) return;

        // Build composed matrix: T(pos) * R_base * R_tweq * S(scale)
        static constexpr float kDegToRad = 3.14159265f / 180.0f;
        Matrix4 fullGlm;

        if (hasRotate) {
            // Tweq rotation values are absolute angles (not deltas from base).
            // Build rotation directly from tweq values.
            float rx = tweqRotDeg.x * kDegToRad;  // bank
            float ry = tweqRotDeg.y * kDegToRad;  // pitch
            float rz = tweqRotDeg.z * kDegToRad;  // heading
            Matrix4 tweqRot = glm::eulerAngleZYX(rz, ry, rx);

            Vector3 finalScale = hasScale ? tweqScale : base->scale;
            Matrix4 scaleMat = glm::scale(Matrix4(1.0f), finalScale);
            Matrix4 worldTrans = glm::translate(Matrix4(1.0f), base->position);
            fullGlm = worldTrans * tweqRot * scaleMat;
        } else if (hasScale) {
            Matrix4 scaleMat = glm::scale(Matrix4(1.0f), tweqScale);
            Matrix4 worldTrans = glm::translate(Matrix4(1.0f), base->position);
            fullGlm = worldTrans * base->rotation * scaleMat;
        } else {
            // No active vector tweqs — shouldn't reach here, but handle gracefully
            Matrix4 scaleMat = glm::scale(Matrix4(1.0f), base->scale);
            Matrix4 worldTrans = glm::translate(Matrix4(1.0f), base->position);
            fullGlm = worldTrans * base->rotation * scaleMat;
        }

        Vector3 finalScale = hasScale ? tweqScale : base->scale;
        applyModelMatrix(*mObjectStates, objID, fullGlm, base->position, finalScale);
    }

    // ── Completion handling ──

    void handleCompletion(TweqInstance &tw, int haltAction) {
        // Send TweqComplete event if Scripts flag is set
        if ((tw.cfgMisc & kTweqMiscScripts) && mEventCallback) {
            mEventCallback(tw.objID, tw.type, haltAction);
        }

        switch (haltAction) {
        case kTweqHaltDestroy:
        case kTweqHaltSlay:
            // Mark object as destroyed — the renderer skips it from here on.
            if (mObjectStates) {
                mObjectStates->get(tw.objID).flags |= kObjStateDestroyed;
            }
            // ... and take it out of the rest of the simulation. The flag alone
            // used to be the whole of it, so a slain object went on colliding,
            // being heard and answering area queries while invisible.
            //
            // Deferred for the same reason emissions are: the callback tears
            // down a spawned object, which drops its tweq instances — and this
            // runs from inside simStep's range-for over that very map. Calling
            // it inline aborted the process the first time a spawned effect
            // expired.
            if (mDestroyCallback) {
                mPendingDestroys.push_back(tw.objID);
            } else {
                static int warnCount = 0;
                if (warnCount++ < 3)
                    std::fprintf(stderr, "[FALLBACK] TweqSystem: obj %d destroyed "
                                 "with no destroy callback wired — it stops "
                                 "rendering but stays in physics, audio and world "
                                 "queries\n", tw.objID);
            }
            tw.active = false;
            break;

        case kTweqHaltRemoveProp:
            tw.active = false;
            break;

        case kTweqHaltStop:
            tw.active = false;
            break;

        case kTweqHaltContinue:
            // Keep running — wrapping/bouncing tweqs do this
            break;
        }
    }

    // ── Utility ──

    /// Get the bounding box bottom Z for a model (for anchor compensation).
    /// Returns the Z coordinate of the bbox minimum (model-space bottom).
    /// Returns 0 if the model isn't found.
    float getModelBBoxBottomZ(const std::string &modelName) const {
        if (!mParsedModels || modelName.empty()) {
            static int warnCount = 0;
            if (warnCount++ < 5)
                std::fprintf(stderr, "[DEFAULT] TweqSystem::getModelBBoxBottomZ: no parsedModels or empty name '%s', returning 0.0\n", modelName.c_str());
            return 0.0f;
        }
        auto it = mParsedModels->find(modelName);
        if (it == mParsedModels->end() || !it->second.valid) {
            static int warnCount = 0;
            if (warnCount++ < 5)
                std::fprintf(stderr, "[DEFAULT] TweqSystem::getModelBBoxBottomZ: model '%s' not found or invalid, returning 0.0\n", modelName.c_str());
            return 0.0f;
        }
        return it->second.bboxMin[2];  // Z = vertical in Dark Engine
    }

    /// Objects removed by an expired delete tweq this session. Reported so a
    /// mission that quietly loses placed objects is visible rather than a
    /// mystery.
    uint32_t mDeleteFired = 0;

    /// Random float in [-1, 1] (matches Dark Engine frand_hack)
    float randFloat() {
        return std::uniform_real_distribution<float>(-1.0f, 1.0f)(mRng);
    }

    // ── Data ──
    std::unordered_map<uint64_t, TweqInstance> mTweqs;
    ObjectStateMap *mObjectStates = nullptr;
    const std::unordered_map<int32_t, ObjPlacementInfo> *mPlacements = nullptr;
    TweqEventCallback mEventCallback;
    ObjectDestroyCallback mDestroyCallback;
    EmitCallback mEmitCallback;
    VisibilityQuery mVisibilityQuery;
    bool mVisibilityGating = true;
    uint32_t mEmitFired = 0;
    // Objects destroyed during a step, torn down after it. See handleCompletion().
    std::vector<int32_t> mPendingDestroys;
    std::vector<int32_t> mDestroyBatch;
    // Emissions raised during a step, dispatched after it. See emitOne().
    std::vector<EmitRequest> mPendingEmissions;
    std::vector<EmitRequest> mEmissionBatch;
    std::mt19937 mRng;
    uint32_t mFrameCount = 0;
    const std::unordered_map<std::string, ParsedBinMesh> *mParsedModels = nullptr;

    // Scratch set for per-frame dirty object tracking (avoids allocation per frame)
    std::unordered_set<int32_t> mDirtyObjects;
};

} // namespace Darkness
