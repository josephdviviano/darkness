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

    // Per-axis config (Rotate/Scale)
    PropTweqAxisConfig axes[3] = {};
    int32_t   primaryAxis = 0;   // 0=all, 1=X, 2=Y, 3=Z (1-indexed!)

    // Simple config (Flicker/Models/Delete)
    uint16_t  cfgRate = 0;       // ms per step

    // Models tweq
    char      modelNames[6][16] = {};
    int       modelCount = 0;    // number of valid model names

    // ── Runtime state ──
    bool      active = false;
    uint32_t  axisState[3] = {}; // per-axis TweqStateFlags
    float     values[3] = {};    // accumulated angles (degrees) or scale factors
    float     elapsedMs = 0.0f;  // for timed tweqs
    int16_t   curFrame = 0;      // for Models tweq
    bool      flickerHidden = false;  // current flicker visibility state

    // Base transform snapshot
    SimTransform base;
    bool      hasObjectState = false;
};

// ── Callback for tweq completion events ──
using TweqEventCallback = std::function<void(int32_t objID, eTweqType type,
                                              int haltAction)>;

// ============================================================================
// TweqSystem — manages all tweq animations
// ============================================================================

class TweqSystem : public SimListener {
public:
    TweqSystem() : mRng(std::random_device{}()) {}

    // ── Initialization ──

    void init(PropertyService *propSvc,
              ObjectStateMap *objectStates,
              const std::unordered_map<int32_t, ObjPlacementInfo> *placements) {
        mObjectStates = objectStates;
        mPlacements = placements;

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
        std::fprintf(stderr, "), %d auto-started (Sim flag)\n", autoStart);
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

    /// Set callback for tweq completion events (for TweqComplete messages).
    void setEventCallback(TweqEventCallback cb) { mEventCallback = std::move(cb); }

    // ── SimListener interface ──

    void simStep(float simTime, float delta) override {
        if (delta <= 0.0f) return;
        float dt_ms = delta * 1000.0f;

        // Collect objects that need transform updates (may have multiple tweqs)
        mDirtyObjects.clear();

        for (auto &[key, tw] : mTweqs) {
            if (!tw.active) continue;

            int result = kTweqStatusQuo;

            switch (tw.type) {
            case kTweqTypeRotate:
            case kTweqTypeScale:
                result = processVectorTweq(tw, dt_ms);
                mDirtyObjects.insert(tw.objID);
                break;

            case kTweqTypeFlicker:
                result = processFlickerTweq(tw, dt_ms);
                break;

            case kTweqTypeModels:
                result = processModelsTweq(tw, dt_ms);
                break;

            default:
                break;
            }

            if (result != kTweqStatusQuo && result != kTweqFrameEvent) {
                handleCompletion(tw, result);
            }
        }

        // Apply composed transforms for all dirty objects
        for (int32_t objID : mDirtyObjects) {
            applyComposedTransform(objID);
        }
    }

    // ── Accessors (for tests and diagnostics) ──

    /// Pack objID + tweqType into a unique map key
    static uint64_t tweqKey(int32_t objID, eTweqType type) {
        return (static_cast<uint64_t>(static_cast<uint32_t>(objID)) << 8) |
               static_cast<uint64_t>(type);
    }

    const std::unordered_map<uint64_t, TweqInstance> &getTweqs() const { return mTweqs; }
    size_t count() const { return mTweqs.size(); }

    /// Inject a tweq instance and set the object state map (for unit tests).
    void injectForTest(const TweqInstance &tw, ObjectStateMap *states) {
        mTweqs[tweqKey(tw.objID, tw.type)] = tw;
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
                          eTweqType tweqType) {
        if (!propSvc || !mPlacements) return;

        for (const auto &[objID, placement] : *mPlacements) {
            if (objID <= 0) continue;  // skip archetypes

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
            initFromConfig(tw, cfg, tweqType);

            // Read state property if present
            StT st;
            if (getTypedProperty<StT>(propSvc, stPropName, objID, st)) {
                initFromState(tw, st, tweqType);
            }

            // Capture base transform from placement data
            initBaseTransform(tw, placement);

            // Auto-activate if Sim flag is set (always-on tweqs like fans)
            if (tw.cfgAnim & kTweqAnimSim) {
                tw.active = true;
            }

            // Initialize current values for Scale tweqs (start at 1.0, not 0.0)
            if (tweqType == kTweqTypeScale) {
                tw.values[0] = tw.base.scale.x;
                tw.values[1] = tw.base.scale.y;
                tw.values[2] = tw.base.scale.z;
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

            mTweqs[tweqKey(objID, tweqType)] = std::move(tw);
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

    /// Extract config fields (simple config: Flicker)
    void initFromConfig(TweqInstance &tw, const PropCfgTweqSimple &cfg, eTweqType) {
        tw.cfgCurve = cfg.curve;
        tw.cfgAnim  = cfg.anim;
        tw.cfgHalt  = cfg.halt;
        tw.cfgMisc  = cfg.misc;
        tw.cfgRate  = cfg.rate;
    }

    /// Extract config fields (Models config)
    void initFromConfig(TweqInstance &tw, const PropCfgTweqModels &cfg, eTweqType) {
        tw.cfgCurve = cfg.curve;
        tw.cfgAnim  = cfg.anim;
        tw.cfgHalt  = cfg.halt;
        tw.cfgMisc  = cfg.misc;
        tw.cfgRate  = cfg.rate;
        tw.modelCount = 0;
        for (int i = 0; i < 6; ++i) {
            std::memcpy(tw.modelNames[i], cfg.modelName[i], 16);
            tw.modelNames[i][15] = '\0';  // ensure null termination
            if (tw.modelNames[i][0] != '\0') tw.modelCount = i + 1;
        }
    }

    /// Extract state fields (vector state: Rotate/Scale)
    void initFromState(TweqInstance &tw, const PropStTweqVector &st, eTweqType) {
        if (st.anim & kTweqStateOn) tw.active = true;
        tw.axisState[0] = st.x;
        tw.axisState[1] = st.y;
        tw.axisState[2] = st.z;
        // Per-axis reverse flags from state
        for (int i = 0; i < 3; ++i) {
            if (tw.axisState[i] & kTweqStateReverse) {
                // Will be handled via axisState during processing
            }
        }
    }

    /// Extract state fields (simple state: Flicker/Models)
    void initFromState(TweqInstance &tw, const PropStTweqSimple &st, eTweqType) {
        if (st.anim & kTweqStateOn) tw.active = true;
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
            for (int i = 0; i < 3; ++i)
                tw.axisState[i] &= ~static_cast<uint32_t>(kTweqStateReverse);
            break;

        case kTweqDoForward:
            tw.active = true;
            tw.elapsedMs = 0.0f;
            for (int i = 0; i < 3; ++i)
                tw.axisState[i] &= ~static_cast<uint32_t>(kTweqStateReverse);
            break;

        case kTweqDoReverse:
            tw.active = true;
            tw.elapsedMs = 0.0f;
            for (int i = 0; i < 3; ++i)
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

        for (int i = 0; i < 3; ++i) {
            int rv = processAxis(tw, i, dt_ms);
            // Primary axis (or all axes if primary=0) controls completion
            if (tw.primaryAxis == 0 || i == (tw.primaryAxis - 1)) {
                if (rv != kTweqStatusQuo) result = rv;
            }
        }

        return result;
    }

    /// Ensure an ObjectState entry exists with the correct base transform.
    /// Called before flicker/model tweqs modify flags or modelNameOverride,
    /// because mObjectStates->get() creates a default entry at position (0,0,0)
    /// which would make the renderer place the object at the origin.
    void ensureObjectState(TweqInstance &tw) {
        if (!mObjectStates || tw.hasObjectState) return;
        tw.hasObjectState = true;
        // Write base transform so the object stays in place
        Matrix4 baseMat = glm::translate(Matrix4(1.0f), tw.base.position)
                        * tw.base.rotation
                        * glm::scale(Matrix4(1.0f), tw.base.scale);
        applyModelMatrix(*mObjectStates, tw.objID, baseMat,
                         tw.base.position, tw.base.scale);
    }

    int processFlickerTweq(TweqInstance &tw, float dt_ms) {
        tw.elapsedMs += dt_ms;

        if (tw.elapsedMs >= static_cast<float>(tw.cfgRate)) {
            tw.elapsedMs -= static_cast<float>(tw.cfgRate);

            // Toggle visibility
            tw.flickerHidden = !tw.flickerHidden;
            if (mObjectStates) {
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

        while (result == kTweqStatusQuo && tw.elapsedMs >= duration) {
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

        // Apply model name override
        if (result == kTweqStatusQuo || result == kTweqFrameEvent) {
            int frame = std::clamp(static_cast<int>(tw.curFrame), 0, tw.modelCount - 1);
            if (mObjectStates && tw.modelNames[frame][0] != '\0') {
                ensureObjectState(tw);
                ObjectState &os = mObjectStates->get(tw.objID);
                os.modelNameOverride = std::string(tw.modelNames[frame],
                    strnlen(tw.modelNames[frame], 16));
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
            // Mark object as destroyed — renderer will skip it
            if (mObjectStates) {
                mObjectStates->get(tw.objID).flags |= kObjStateDestroyed;
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

    /// Random float in [-1, 1] (matches Dark Engine frand_hack)
    float randFloat() {
        return std::uniform_real_distribution<float>(-1.0f, 1.0f)(mRng);
    }

    // ── Data ──
    std::unordered_map<uint64_t, TweqInstance> mTweqs;
    ObjectStateMap *mObjectStates = nullptr;
    const std::unordered_map<int32_t, ObjPlacementInfo> *mPlacements = nullptr;
    TweqEventCallback mEventCallback;
    std::mt19937 mRng;

    // Scratch set for per-frame dirty object tracking (avoids allocation per frame)
    std::unordered_set<int32_t> mDirtyObjects;
};

} // namespace Darkness
