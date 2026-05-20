/******************************************************************************
 *
 *    This file is part of the darkness project
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

#ifndef __PROBE_MANAGER_H
#define __PROBE_MANAGER_H

/// @file ProbeManager.h
/// Acoustic probe baking, loading, saving, and in-memory storage.
///
/// Extracted from AudioService — owns the Steam Audio probe batch and the
/// mirrored probe-position array used by debug overlays. AudioService keeps
/// thin facades (bakeProbes/loadProbes/getProbeCount/getProbePositions) that
/// forward to ProbeManager so callers in the renderer and debug console need
/// no changes.
///
/// Threading: ProbeManager is owned and called only from the main thread.
/// `bakeProbes` is a blocking call (~10-60 s) and uses an `std::atomic<float>`
/// progress hook so a UI thread can poll. The Steam Audio probe batch handle
/// returned by `getProbeBatch()` is read by the reflection-sim worker on a
/// separate thread; the caller is responsible for invoking the supplied
/// `waitForReflectionThread` callback before any mutation (release/load) of
/// the batch.

#include "DarknessMath.h"

#include <atomic>
#include <functional>
#include <string>
#include <vector>

// Forward declarations for Steam Audio opaque handles
struct _IPLContext_t;
typedef _IPLContext_t* IPLContext;
struct _IPLScene_t;
typedef _IPLScene_t* IPLScene;
struct _IPLSimulator_t;
typedef _IPLSimulator_t* IPLSimulator;
struct _IPLProbeBatch_t;
typedef _IPLProbeBatch_t* IPLProbeBatch;

namespace Darkness {

/// Per-bake parameters supplied by AudioService (read from its tuning state).
/// All distance fields are in engine feet (ProbeManager converts to meters
/// at the IPL boundary internally).
struct ProbeBakeParams {
    /// Scene AABB used to position the probe-generation OBB. Engine feet.
    Vector3 sceneMin{0.0f, 0.0f, 0.0f};
    Vector3 sceneMax{0.0f, 0.0f, 0.0f};

    /// Reverb max distance (used as pathing visibility/path range).
    float   propagationMaxDist  = 200.0f;

    /// Reflection bake quality.
    int     reflectionNumRays    = 1024;
    int     reflectionNumBounces = 4;
    float   reflectionDuration   = 2.0f;
    int     bakeDiffuseSamples   = 128;
    int     simulatorThreads     = 0;       ///< 0 = auto
    int     ambisonicsOrder      = 0;
    std::string sceneType        = "default";  ///< "default" or "embree"

    /// Override grid spacing/height in engine feet. Use -1.0f to fall back
    /// to ProbeManager's configured spacing/height.
    float   spacingFtOverride    = -1.0f;
    float   heightFtOverride     = -1.0f;

    /// Extra elevations (engine feet, above each floor probe position) at
    /// which to add replicated probes. Empty = floor-grid only (legacy
    /// behaviour). Typical values: {10.0f} to cover wall-mounted torches
    /// and ceiling-mounted lamps that emit well above floor height.
    /// Steam Audio's pathing matches source/listener to nearest probe; if
    /// only floor probes exist, an elevated emitter routes to the floor
    /// probe at its (x,y), which misrepresents its actual geometric
    /// location and can produce odd pathing chains.
    std::vector<float> additionalElevations;

    /// Per-portal axial anchors (in engine feet) to seed extra probes at.
    /// Each entry contributes up to 2 probes — one at `center + normal *
    /// axialOffsetFt` and one at `center - normal * axialOffsetFt` — so
    /// each adjoining room gets a probe just inside its volume rather
    /// than a ring sitting ON the doorway plane.  The on-plane ring
    /// design produced two pathological hotspots:
    ///   (a) doorway-shaped IRs (rays from 0.5 m off the plane bounced
    ///       within the narrow doorway corridor → very high early energy)
    ///   (b) duplicate probes per shared doorway (each portal appears in
    ///       both adjoining rooms' portal lists; the basis symmetry made
    ///       the 4 ring positions identical between the two directions →
    ///       8 probes at 4 unique points per doorway).
    /// AudioService now visits each portal pair in canonical orientation
    /// only and supplies the inward-facing `normal`; ProbeManager handles
    /// the ±offset expansion plus a proximity-dedup pass against the
    /// floor + elevation tiers so window-style openings still seed cover
    /// but doorways already covered by the floor grid don't double-count.
    /// Empty = floor grid only (no portal-centric anchors).
    struct PortalAxis {
        Vector3 center;
        Vector3 normal;   ///< Unit, oriented arbitrarily — both ± sides emit
    };
    std::vector<PortalAxis> portalAxes;

    /// Axial probe offset on each side of `PortalAxis::center` (engine
    /// feet). 1 ft (≈0.3 m) is far enough to bake the *room's* acoustics
    /// rather than the narrow corridor's.
    float portalAxialOffsetFt = 1.0f;

    /// Skip a portal-axis candidate if any existing floor/elevation probe
    /// lies within this many feet — the floor grid already covers it.
    /// One floor spacing is a sensible default (≈ the radius at which
    /// IPL would natively interpolate between neighbours anyway).
    float portalDedupRadiusFt = 5.0f;
};

/// Construction-time wiring. ProbeManager holds a reference to the IPL
/// context for the lifetime of the audio service, plus a callback used to
/// quiesce the reflection-sim worker before any probe-batch mutation.
struct ProbeManagerDeps {
    /// Steam Audio context — supplied by AudioService at init time.
    IPLContext context = nullptr;

    /// Function that blocks until the reflection-sim background thread is
    /// idle. Called before any probe-batch register/unregister/release so
    /// Steam Audio never sees a torn batch. Required.
    std::function<void()> waitForReflectionThread;
};

/// Owns the Steam Audio probe batch, the mirrored probe-position array, and
/// the bake-time grid configuration. Persistent across mission boundaries —
/// `loadProbes` and `bakeProbes` both call `releaseBatch` internally before
/// installing a new batch.
class ProbeManager {
public:
    explicit ProbeManager(ProbeManagerDeps deps);
    ~ProbeManager();

    ProbeManager(const ProbeManager &) = delete;
    ProbeManager &operator=(const ProbeManager &) = delete;

    // ── Bake / Load / Save ────────────────────────────────────────────────

    /** Bake acoustic probes for the current scene.
     *  Generates probes on a uniform floor grid, bakes pathing visibility,
     *  bakes reflection IRs, and writes them to `outputPath` (with a CRC
     *  integrity envelope, plus sidecar CSVs for positions/energy).
     *  Blocking call (~10-60 seconds). Progress is reported via the atomic
     *  float (0.0-1.0).
     *  @param scene       Acoustic scene to bake against (raycast target)
     *  @param outputPath  File path for the .probes output
     *  @param params      Per-bake tuning, scene bounds, and grid overrides
     *  @param progress    Atomic float updated with bake progress (optional)
     *  @return true if baking succeeded */
    bool bakeProbes(IPLScene scene,
                    const std::string &outputPath,
                    const ProbeBakeParams &params,
                    std::atomic<float> *progress);

    /** Load baked probe data from disk and register with the reflection
     *  simulator.  Releases any previously-loaded batch first.
     *  @param probePath           Path to .probes file
     *  @param reflectionSimulator Simulator to register the batch with
     *  @return true if loaded successfully */
    bool loadProbes(const std::string &probePath,
                    IPLSimulator reflectionSimulator);

    /** Release the currently-loaded probe batch.
     *  If a reflection simulator is supplied (and non-null), the batch is
     *  unregistered from it first.  Safe to call when no batch is loaded.
     *  Caller must ensure the reflection-sim background thread is idle
     *  (the deps `waitForReflectionThread` is invoked internally). */
    void releaseBatch(IPLSimulator reflectionSimulator);

    /** Standard probe file path for a mission: stored in
     *  `~/darkness/{gameName}/baked_probes/{missionName}.probes`. Creates
     *  intermediate directories if they don't exist. */
    static std::string getProbeFilePath(const std::string &misPath,
                                         const std::string &gameName = "thief2");

    // ── Accessors ─────────────────────────────────────────────────────────

    /// Number of probes currently loaded (0 if no batch is installed).
    int getProbeCount() const { return mProbeCount; }

    /// True iff the loaded batch carries baked reflection IRs (some legacy
    /// .probes files only contain pathing data).
    bool hasReflections() const { return mProbesHaveReflections; }

    /// Opaque handle to the underlying Steam Audio probe batch. May be
    /// nullptr if no batch is loaded. Read-only — callers must not release
    /// the handle directly; use `releaseBatch` instead.
    IPLProbeBatch getProbeBatch() const { return mIplProbeBatch; }

    /// Snapshot of probe positions in engine feet. Populated by
    /// `bakeProbes` and `loadProbes`; empty if no probes are loaded.
    /// Used by the renderer to draw a debug overlay. The vector is
    /// rebuilt on every bake/load, so cache by index — values do not
    /// change between re-bakes.
    const std::vector<Vector3> &getProbePositions() const { return mProbePositions; }

    // ── Bake-time grid configuration ──────────────────────────────────────
    //
    // Takes effect on the next bakeProbes() call — does NOT relocate
    // existing probes. Range-clamped to keep CPU cost reasonable.

    void  setProbeSpacingFt(float ft) { mProbeSpacingFt = std::max(1.0f, std::min(ft, 20.0f)); }
    float getProbeSpacingFt() const { return mProbeSpacingFt; }
    void  setProbeHeightFt(float ft) { mProbeHeightFt = std::max(0.5f, std::min(ft, 20.0f)); }
    float getProbeHeightFt() const { return mProbeHeightFt; }

private:
    ProbeManagerDeps mDeps;

    // ── Owned state ───────────────────────────────────────────────────────

    IPLProbeBatch mIplProbeBatch = nullptr;  ///< Loaded probe data (positions + baked paths + reflections)
    int  mProbeCount = 0;                    ///< Number of probes in the batch
    bool mProbesHaveReflections = false;     ///< True if loaded probes contain baked reflection IRs

    /// Grid parameters used at bake time. Read by bakeProbes() when the
    /// caller passes a negative override. Live-tunable via console but only
    /// take effect on the next re-bake.
    float mProbeSpacingFt = 5.0f;
    float mProbeHeightFt  = 5.0f;

    /// Probe positions in engine feet. Populated by bakeProbes() and loaded
    /// from a sidecar file by loadProbes(). Used purely for debug overlay
    /// rendering — Steam Audio holds the canonical copy internally.
    std::vector<Vector3> mProbePositions;
};

} // namespace Darkness

#endif // __PROBE_MANAGER_H
