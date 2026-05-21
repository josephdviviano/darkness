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
/// Acoustic probe baking, loading, saving, and in-memory storage. Owns the
/// Steam Audio probe batch + mirrored probe-position array used by debug
/// overlays.
///
/// Threading: main-thread only. `bakeProbes` is blocking (~10-60s) and
/// reports via an atomic float so a UI thread can poll. The probe batch
/// is read by the reflection-sim worker; callers MUST invoke the supplied
/// `waitForReflectionThread` callback before any batch mutation.

#include "DarknessMath.h"

#include <atomic>
#include <cstdint>
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

/// Outcome of bake-time filter. Nudge = recoverable displacement;
/// preserves probes just too close to a wall while still dropping those
/// inside solid geometry.
enum class ProbeFilterResult {
    Accept,
    Reject,
    Nudge,
};

struct ProbeFilterDecision {
    ProbeFilterResult result = ProbeFilterResult::Accept;
    /// Unit vector pointing away from the offending surface (Nudge only).
    Vector3 nudgeDir{0.0f, 0.0f, 0.0f};
    /// Engine feet — minimum displacement to satisfy the filter next iter.
    /// Callers add a small margin to avoid boundary ping-pong from jitter.
    float nudgeDistFt = 0.0f;
};

using ProbeFilterFn = std::function<ProbeFilterDecision(const Vector3 &)>;

/// Per-bake parameters supplied by AudioService (read from its tuning state).
/// All distance fields are in engine feet (ProbeManager converts to meters
/// at the IPL boundary internally).
struct ProbeBakeParams {
    /// Scene AABB used to position the probe-generation OBB. Engine feet.
    Vector3 sceneMin{0.0f, 0.0f, 0.0f};
    Vector3 sceneMax{0.0f, 0.0f, 0.0f};

    /// Reverb max distance (used as pathing visibility/path range).
    float   propagationMaxDist  = 200.0f;

    /// Reflection bake quality — separate from realtime params so the bake
    /// can afford higher quality without affecting per-frame sim cost.
    int     bakeNumRays          = 4096;
    int     bakeNumBounces       = 8;
    float   bakeDuration         = 4.0f;
    int     bakeDiffuseSamples   = 256;
    int     simulatorThreads     = 0;       ///< 0 = auto
    int     ambisonicsOrder      = 1;       ///< Bake-side ambisonic order
    std::string sceneType        = "default";  ///< "default" or "embree"

    /// Override grid spacing/height in engine feet. Use -1.0f to fall back
    /// to ProbeManager's configured spacing/height.
    float   spacingFtOverride    = -1.0f;
    float   heightFtOverride     = -1.0f;

    /// Extra elevations (feet, above floor) for replicated probes. Empty
    /// = floor-grid only. Without elevated probes an elevated emitter
    /// routes to the floor probe at its (x,y), misrepresenting geometry.
    std::vector<float> additionalElevations;

    /// Per-portal axial anchors. Each contributes up to 2 probes (center
    /// ± normal · offset) so each adjoining room gets a probe just
    /// inside its volume rather than on the doorway plane. Caller
    /// supplies canonical orientation only; ProbeManager handles ±
    /// expansion + dedup against floor/elevation tiers.
    struct PortalAxis {
        Vector3 center;
        Vector3 normal;   ///< Unit; both ± sides emit
    };
    std::vector<PortalAxis> portalAxes;

    /// Axial probe offset on each side of PortalAxis::center (feet).
    /// 1 ft ≈ 0.3 m — far enough to bake the room's acoustics, not the
    /// narrow corridor's.
    float portalAxialOffsetFt = 1.0f;

    /// Skip a portal-axis candidate if an existing floor/elevation probe
    /// is within this many feet (the floor grid already covers it).
    float portalDedupRadiusFt = 5.0f;

    /// Elevation-tier sparsity multiplier. Floor probes are binned with
    /// binSize = floorSpacing × this; one elevation probe per bin centroid
    /// per tier. 2.0 = 1:4 ratio (default); 1.0 = 1:1 (legacy).
    float elevationSparsityMul = 2.0f;

    /// Global dedup radius (feet) applied AFTER all placement passes.
    /// Earlier passes have priority (floor always wins). 0 = disabled.
    float globalDedupRadiusFt = 2.0f;

    /// Pre-bake validity filter applied to every candidate. Returns
    /// Accept / Reject / Nudge. Pre-filtering avoids near-zero IRs from
    /// in-wall probes and comb-filtering from near-wall probes. Empty
    /// = keep everything.
    ProbeFilterFn probeFilter;

    /// Informational — what wall clearance (feet) the caller's filter
    /// enforces; logged for traceability. Does NOT derive the filter.
    float minWallClearanceFt = 0.0f;
};

/// Construction-time wiring.
struct ProbeManagerDeps {
    IPLContext context = nullptr;
    /// Blocks until the reflection-sim worker is idle. Required — called
    /// before any probe-batch register/unregister/release so Steam Audio
    /// never sees a torn batch.
    std::function<void()> waitForReflectionThread;
};

/// Owns the Steam Audio probe batch + mirrored probe-position array +
/// bake-time grid config. Persistent across missions — loadProbes/
/// bakeProbes both call releaseBatch internally before installing a new one.
class ProbeManager {
public:
    explicit ProbeManager(ProbeManagerDeps deps);
    ~ProbeManager();

    ProbeManager(const ProbeManager &) = delete;
    ProbeManager &operator=(const ProbeManager &) = delete;

    /// Bake probes for the current scene. Generates a floor grid, bakes
    /// pathing + reflection IRs, writes `.probes` with CRC envelope.
    /// Blocking (~10-60s); progress is reported via the atomic float.
    bool bakeProbes(IPLScene scene,
                    const std::string &outputPath,
                    const ProbeBakeParams &params,
                    std::atomic<float> *progress);

    /// Load probes from disk + register with the reflection simulator.
    /// Releases any previously-loaded batch first.
    bool loadProbes(const std::string &probePath,
                    IPLSimulator reflectionSimulator);

    /// Release the loaded batch. Unregisters from `reflectionSimulator`
    /// if supplied. Safe when no batch is loaded. waitForReflectionThread
    /// is invoked internally before any IPL mutation.
    void releaseBatch(IPLSimulator reflectionSimulator);

    /// Standard mission probe path:
    /// ~/darkness/{gameName}/baked_probes/{missionName}.probes
    static std::string getProbeFilePath(const std::string &misPath,
                                         const std::string &gameName = "thief2");

    int getProbeCount() const { return mProbeCount; }
    /// Legacy .probes files only contain pathing data — false then.
    bool hasReflections() const { return mProbesHaveReflections; }
    /// Read-only. Callers must use releaseBatch(), not direct release.
    IPLProbeBatch getProbeBatch() const { return mIplProbeBatch; }
    /// Probe positions (feet). Rebuilt on every bake/load.
    const std::vector<Vector3> &getProbePositions() const { return mProbePositions; }

    // Bake-time grid config — takes effect on next bake. Does NOT relocate.
    void  setProbeSpacingFt(float ft) { mProbeSpacingFt = std::max(1.0f, std::min(ft, 20.0f)); }
    float getProbeSpacingFt() const { return mProbeSpacingFt; }
    void  setProbeHeightFt(float ft) { mProbeHeightFt = std::max(0.5f, std::min(ft, 20.0f)); }
    float getProbeHeightFt() const { return mProbeHeightFt; }

private:
    ProbeManagerDeps mDeps;

    IPLProbeBatch mIplProbeBatch = nullptr;
    int  mProbeCount = 0;
    bool mProbesHaveReflections = false;

    /// Negative override → use these values at bake time.
    float mProbeSpacingFt = 5.0f;
    float mProbeHeightFt  = 5.0f;

    /// Debug-overlay mirror (Steam Audio holds the canonical copy).
    std::vector<Vector3> mProbePositions;
};

} // namespace Darkness

#endif // __PROBE_MANAGER_H
