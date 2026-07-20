// RenderConfig.h — YAML + CLI configuration for darknessRender
// Config precedence: CLI flags > YAML config file > hardcoded defaults
#pragma once

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace Darkness {

/// Water tunable bounds — SINGLE SOURCE OF TRUTH.
///
/// Both consumers must read these: the YAML clamp in loadConfig() below, and
/// the debug-console slider registration in DarknessRender.cpp
/// (registerConsoleSettings). They used to be written out twice, by hand, and
/// they disagreed — in both directions:
///
///     setting          YAML max   console max
///     wave_amplitude      10.0        5.0
///     uv_distortion        0.1        0.5
///     water_rotation       1.0        0.5
///
/// Only the YAML-wider rows were a real bug: `water: { wave_amplitude: 8.0 }`
/// boots at 8.0 and renders fine, but DebugConsole::addFloat's setter rejects
/// out-of-range input, so the first console edit strands the value — there is
/// no way back to what the user's own config file asked for. The console-wider
/// row (uv_distortion) is not that bug: every YAML-valid value was already
/// settable. It just offered values that could never be persisted back.
///
/// So: **the YAML clamp is authoritative and the console follows it.** The
/// config file is the persistent contract; the console must not offer a value
/// you cannot save. Values below are therefore the pre-existing YAML bounds,
/// unchanged — this fix moves the console, not the config, and no config that
/// worked before behaves differently now.
///
/// These are not physical limits. To retune, change it HERE; both consumers
/// follow and cannot drift apart again.
namespace WaterRange {
constexpr float kWaveAmplitudeMin = 0.0f, kWaveAmplitudeMax = 10.0f; // console was 5.0
constexpr float kUvDistortionMin  = 0.0f, kUvDistortionMax  = 0.1f;  // console was 0.5
constexpr float kWaterRotationMin = 0.0f, kWaterRotationMax = 1.0f;  // console was 0.5
constexpr float kWaterScrollMin   = 0.0f, kWaterScrollMax   = 1.0f;  // already agreed
} // namespace WaterRange

/// Clamp a YAML-supplied value, announcing when the config file asked for
/// something we would not honour. Silently rewriting a user's stated intent is
/// the same class of bug as a silent fallback: the value in the file and the
/// value in the engine disagree, and nothing says so.
inline float clampConfigValue(float v, float lo, float hi, const char *key) {
    if (v < lo || v > hi) {
        const float clamped = (v < lo) ? lo : hi;
        std::fprintf(stderr,
            "[FALLBACK] config: %s = %g is outside [%g, %g] — clamping to %g. "
            "The engine will not use the value your config file specifies.\n",
            key, static_cast<double>(v), static_cast<double>(lo),
            static_cast<double>(hi), static_cast<double>(clamped));
        return clamped;
    }
    return v;
}

// All configurable settings for the renderer.
// Defaults match the original hardcoded values.
struct RenderConfig {
    // -- graphics --
    // Filter terminology: both keys end in "_filter" and use string enums.
    //   texture_filter:  point | bilinear | trilinear | anisotropic
    //   lightmap_filter: bilinear | bicubic
    // Internally still stored as small ints to keep the runtime hot path branchless.
    int  filterMode        = 0;     // texture filter: 0=point, 1=bilinear, 2=trilinear, 3=anisotropic
    int  lightmapFiltering = 0;     // lightmap filter: 0=bilinear (default), 1=bicubic
    bool linearMips        = false; // gamma-correct mipmap generation
    bool sharpMips         = false; // unsharp mask on mip levels

    // -- paths -- (CLI flags --res / --schemas override these)
    std::string resPath;     // Thief 2 RES directory containing fam.crf / obj.crf / snd.crf
    std::string schemasPath; // schema directory (.sch / .spc / .arc files)

    // -- water -- (ranges: see WaterRange below)
    float waveAmplitude   = 0.3f;    // vertex Z displacement in world units (0 = flat)
    float uvDistortion    = 0.015f;  // UV wobble strength (0 = no ripple)
    float waterRotation   = 0.015f;  // UV rotation speed in rad/s (0 = no rotation)
    float waterScrollSpeed = 0.05f;  // UV scroll speed in world units/s (0 = no drift)

    // -- audio.performance: engine + reverb throughput knobs --
    // Scope tags:
    //   [GLOBAL]      audio engine / hardware
    //   [DIRECT]      direct path only (no reflections)
    //   [REALTIME]    realtime ray-traced reflections only
    //   [BAKE]        offline probe bake only
    //   [REFLECTIONS] both realtime + baked-probe reflection convolution
    int  audioSampleRate         = 48000; // [GLOBAL] device output sample rate (22050|32000|44100|48000|96000)
    int  audioFrameSize          = 1024;  // [GLOBAL] audio engine frame size in samples (256–4096)
    int  audioSoundCacheMB       = 64;    // [GLOBAL] decoded-audio LRU cache budget (MB)

    // -- audio.engine: device-callback / mixer-thread topology (PR D) --
    // ring_mixer: true = mix graph renders on a dedicated mixer thread into
    // a lock-free ring; the device callback only drains the ring (crackle
    // fix — the render no longer competes with the HAL deadline). false =
    // legacy in-callback rendering (pre-PR-D behavior, kept for A/B).
    bool  audioRingMixer         = true;  // [GLOBAL]
    // ring_margin_ms: ring fill target the mixer thread maintains. <= 0 =
    // auto (two engine blocks, min 21.4 ms). Larger = more scheduling
    // slack, more output latency. (0=auto–500)
    float audioRingMarginMs      = -1.0f; // [GLOBAL]
    int  reflectionRateDivisor   = 2;     // [REFLECTIONS] reflection pipeline rate: 1=full 48kHz, 2=half 24kHz, 4=quarter 12kHz
    int  maxActiveVoices         = 64;    // [GLOBAL] hard cap on simultaneous voices (Dark Engine baseline)
    // [REFLECTIONS] Cap on total reverb voices (realtime + baked combined).
    // CPU governor — every reverb voice runs a per-source convolution
    // regardless of mode. Setting to 0 disables all reverb convolution
    // entirely (fully dry).
    int  reverbVoices            = 16;
    // [REALTIME] Of the reverb voices above, how many may run with
    // realtime ray-traced IRs. 0 = baked-only (recommended; all eligible
    // voices route through baked-probe reverb). Range [0, reverbVoices].
    int  reverbVoicesRealtime    = 0;
    int  reflectionThrottle      = 4;     // [REFLECTIONS] signal the reflection-sim worker every Nth audio loop step (1–32); paces the shared baked-reverb IR refresh in baked-only mode too
    int  simMaxOcclusionSamples  = 32;    // [DIRECT+REFLECTIONS] per-source occlusion sample cap (Steam Audio sim) (4–256)
    // Explicit thread counts for reverb work. `convThreads` are the
    // per-voice convolution workers; `simThreads` are the Steam Audio
    // ray-trace simulator threads. Both default to 0 = auto: total =
    // max(2, hwconc - 2), split into ~35% conv / ~65% sim in baked-only
    // mode (~45% / ~55% with realtime voices) — the sim is the bottleneck
    // in both modes. Set BOTH > 0 to use literal values. If only one is
    // > 0, AudioService emits a warning and falls back to auto for both.
    int   convThreads            = 0;
    int   simThreads             = 0;
    std::string sceneType        = "default"; // [REFLECTIONS] IPL scene backend ("default" or "embree" — embree falls back to default if Steam Audio wasn't built with embree)

    // -- audio.reflections: convolution reverb feel --
    //
    // Bake and realtime parameters are split: the bake runs once per
    // mission and is cached as a .probes file, so it can afford much
    // higher quality settings than the realtime sim, which runs every
    // reflection_throttle audio frames.
    //
    // `ambisonicsOrder` here is the REALTIME order; the bake can record
    // a higher order in `bakeAmbisonicsOrder` since the runtime decoder
    // downmixes higher-order baked IRs to the requested realtime order.

    /// Reflection pipeline is HYBRID-only (Steam Audio's
    /// `IPL_REFLECTIONEFFECTTYPE_HYBRID`): early convolution head
    /// (length = `hybridTransitionTime`) plus a parametric tail driven by
    /// RT60 baked into the probe data. CONVOLUTION and PARAMETRIC modes
    /// were removed — the `reflections.type` YAML key is now deprecated
    /// and emits a [FALLBACK] warning if present.

    bool  realtimeReflections = true;  // master enable for HYBRID reflection pipeline
    float hybridTransitionTime = 1.0f;  // seconds — convolution head length (Steam Audio Unity/Unreal default)
    float hybridOverlapPercent = 0.25f; // fraction of transition_time for crossfade

    int   ambisonicsOrder     = 0;      // realtime ambisonic order (0–3)

    // Realtime simulation params (running every reflection_throttle frames)
    int   realtimeNumRays         = 1024;  // rays per realtime sim step (128–8192)
    int   realtimeNumBounces      = 4;     // bounces per ray (1–8)
    float realtimeDuration        = 1.1f;  // IR duration in seconds (>hybridTransitionTime + 0.1 s margin)
    int   realtimeDiffuseSamples  = 32;    // diffuse scattering samples per bounce (16–256)

    // Offline bake params (run once per mission; cached as .probes files).
    int   bakeNumRays             = 4096;  // rays per bake step (1024–65536)
    int   bakeNumBounces          = 8;     // bake bounces (1–64)
    float bakeDuration            = 1.1f;  // bake IR duration in seconds (>= realtime.duration)
    int   bakeDiffuseSamples      = 256;   // bake diffuse samples (32–4096)
    int   bakeAmbisonicsOrder     = 1;     // bake ambisonic order (0–3)

    // -- audio.probes: baked-probe grid generation --
    // Spacing/height feed bakeProbes(); a denser grid produces smoother reverb
    // interpolation at the cost of ~(spacing_old/spacing_new)^2 disk space and
    // proportionally longer bake time. The defaults match the prior hardcoded
    // values; halve spacing (e.g. 2.5) to test whether residual footstep
    // reverb A/B variance is driven by probe sparsity.
    float audioProbeSpacingFt = 5.0f;  // grid spacing in feet (1.0–20.0; requires re-bake to take effect)
    float audioProbeHeightFt  = 5.0f;  // probe height above floor in feet (0.5–20.0)
    // Extra elevation tier (in feet, above each floor probe) to densify
    // pathing coverage for wall-mounted torches and ceiling lamps. Default
    // {10.0} covers wall-height emitters. Empty = floor-only (legacy).
    std::vector<float> audioProbeElevations = { 10.0f };
    // Bake-time validity filter. Drops any probe candidate that either
    //   (a) doesn't sit inside any room (BSP void / inside solid), or
    //   (b) is within this many engine feet of the nearest room wall.
    // 0 disables the clearance check (the inside-solid check still
    // runs). Higher values prefer well-conditioned IRs over coverage
    // near walls; cranking past ~half the typical corridor width will
    // start rejecting probes in narrow passages. Requires a re-bake.
    float audioProbeMinWallClearanceFt = 5.0f;
    // Elevation-tier sparsity multiplier. Floor probes are binned on a
    // coarser (x, y) grid (binSize = spacing × this) and one elevation
    // probe is placed at each bin's centroid per tier. Default 2.0 =
    // 2×2 binning = 1:4 ratio (~75% fewer elevation probes vs legacy
    // 1:1). Higher values further reduce density; 1.0 restores legacy
    // 1:1 behaviour. Requires a re-bake.
    float audioProbeElevationSparsityMul = 2.0f;
    // Global dedup pass radius (engine feet) applied after all probe
    // placement (floor, elevation, portal, emitter). Probes within this
    // distance of an earlier-kept probe get dropped. Default 2.0 ft is
    // conservative — catches obvious overlaps without eating into the
    // 5 ft grid spacing. 0 = dedup disabled. Requires a re-bake.
    float audioProbeGlobalDedupRadiusFt = 2.0f;

    // -- audio.pathing_probes: sparse ROOM_PORTAL pathing batch --
    //
    // Splits pathing probes onto a separate Steam Audio probe batch
    // (sparse ROOM_PORTAL graph: one probe per room centroid, two per
    // portal). Reflection probes remain dense (UNIFORMFLOOR + elevation
    // + portal axes + emitter) for high-quality IR sampling.
    //
    // The cost of Steam Audio's `findAlternatePaths` is roughly quadratic
    // in probe count when dynamic geometry (e.g. door OBBs) invalidates
    // baked paths at runtime; a sparse pathing batch keeps that cost
    // microsecond-scale. Disable to revert to single-batch baking (the
    // .probes file will then contain only the reflection batch and
    // runtime pathing falls back to the synthetic-bypass branch).
    bool audioPathingProbesEnabled = true;

    // Proximity dedup radius for the PATHING batch only (engine feet).
    // Applied after all pathing-candidate emission (portal, centroid,
    // emitter). 10 ft handles the typical compound-doorway / sub-room
    // clusters in Thief 2 levels without dropping legitimately distinct
    // probes. 0 disables the pass. Separate from `global_dedup_radius_ft`,
    // which only affects the dense reflection batch.
    float audioPathingDedupRadiusFt = 10.0f;

    // EXPERIMENTAL single-edge visRange override for the pathing bake
    // (`audio.pathing_probes.vis_range_override_ft`). 0 (default) = use
    // the coverage-derived cap (governing x margin, clamped — see
    // AudioService::prepareProbeBakeParams). Nonzero = force the bake's
    // IPLPathBakeParams::visRange to EXACTLY this value, bypassing the
    // derivation and its clamps. A/B lever for the range sweep the
    // offline §37 analysis motivates (~80 ft keeps the aperture graph
    // healthy at a fraction of the edges); recorded in the .probes
    // header like the derived value, so runtime follows automatically.
    // Requires --force-pathing-bake to take effect on an existing cache
    // (deliberately no auto-rebake: experimental knob).
    float audioPathingVisRangeOverrideFt = 0.0f;

    // Pathing probe layout density tier (`audio.pathing_probes.density`).
    // Valid values:
    //   "baseline" — Tier 0: the original Dark Engine room/portal
    //                graph's nodes (1 per room centroid + 1 per non-door
    //                portal center + door flanking pairs + emitter
    //                mirrors). Selectable — fast dev bakes / low-end.
    //   "bends"    — Tier 1 (default): baseline, with every non-door
    //                portal's center probe replaced by a flanking pair —
    //                explicit solver bend points at each opening.
    //   ("high" is RESERVED for a future Tier 2 — room-span subdivision
    //    for long halls — and is rejected at parse until it exists.)
    // WHY bends is the default (user decision 2026-07-12, supersedes the
    // 2026-07-11 baseline flip): fidelity first — flanking pairs at every
    // aperture give the solver explicit bend points, and they measured
    // BETTER worst-case door spikes. The baseline flip rested on
    // misattributed numbers: re-review at ns8 found bends' worst
    // door-spike window = 316.8 ms vs baseline's 535-696 ms (baseline's
    // spikes were hidden by log rate-limiting; the "387-700 ms" figure
    // pinned on bends was old scatter). Baseline still wins median
    // pathing (p50 66 vs 85 ms) and bake time (17.7 vs 28.4 min at ns8),
    // which is why it remains selectable rather than removed.
    // Kept as the validated string; mapped to the PathingProbeDensity
    // enum at the AudioService boundary (DarknessRender.cpp). Recorded
    // in the .probes v4 header — changing it triggers a loud automatic
    // pathing-only re-bake on next run.
    std::string audioPathingDensity = "bends";

    // Force a fresh pathing bake even when the existing .probes file
    // already contains a valid pathing section. The loaded reflection IR
    // section is carried forward unchanged (pathing-only re-bake), so
    // this is fast — useful for iterating on pathing-bake parameters
    // (e.g. dedup radius) without paying the reflection bake.
    //
    // Default false. No YAML key by design — this is a per-invocation
    // override. See PLAN.AUDIO_PROFILING.md §4.3.
    bool forcePathingBake = false;

    // Bake-quality profile: true = `--bake-quality dev`. Besides the
    // reflection-bake overrides applied directly in the CLI parser, this
    // flag selects the pathing visibility sampling constant
    // (SteamAudioPathing.h kPathingVisSamplesDev vs Ship — both 4 since
    // 2026-07-11, split retained structurally) for BOTH
    // the bake and the runtime pathing simulator — one flag, both sides,
    // so they cannot diverge within a run. Cross-run cache mismatches
    // are caught against the .probes v3 header (automatic pathing-only
    // re-bake). No YAML key by design — per-invocation profile.
    bool devBakeProfile = false;

    // -- audio.occlusion: occlusion + material scaling --
    // (diffuseSamples / bakeDiffuseSamples moved to realtimeDiffuseSamples /
    //  bakeDiffuseSamples under reflections — legacy occlusion.diffuse_samples
    //  / occlusion.bake_diffuse_samples keys still parsed with deprecation warning.)
    float occlusionRadius     = 5.0f;  // volumetric occlusion sphere radius (world units = feet, 0.3–30)
    int   occlusionSamples    = 16;    // ray samples per source for occlusion gradient (4–64)
    float transmissionScale   = 1.0f;  // multiplier for material transmission (1=physical, 10=through-walls game-friendly)
    float absorptionScale     = 1.0f;  // multiplier for material absorption (1=physical, <1=more reflective)

    // -- audio.propagation: cell-graph sound routing + door blocking --
    bool  portalRouting       = true;   // portal-graph sound routing through doorways
    bool  probePathing        = true;   // baked probe diffraction (when available)
    float propagationMaxDist  = 200.0f; // max sound propagation distance through portal graph (world units)
    float doorLpfOpenHz       = 20000.0f; // LPF cutoff for fully open door (Hz)
    float doorLpfBlockedHz    = 800.0f;   // LPF cutoff for fully blocked door (Hz)
    // Floor on propagation/portal scale. Default 0.0 disables the floor
    // (was 0.001; the FP-noise reasoning was speculative — left in place
    // as a config knob in case it needs to be re-enabled). With smooth
    // occlusion ramps, attenuation naturally reaches 0 without dropouts.
    float propMinAttenuation  = 0.0f;
    // N-path BFS: how many simultaneous portal-graph paths to keep per
    // listener room. 1 = single shortest path; 2 = original Dark Engine
    // (the per-room propagation record's second predecessor slot); 3+ = modernized. Clamped to [1, 4].
    uint32_t propMaxPaths     = 2;
    // Alternates kept only if their effective distance is within this
    // many world units of the primary. Matches the original engine's
    // distance-difference cap default = 10. Clamped to [0, 50].
    float    propMaxPathDiff  = 10.0f;
    // Multiplier on the scalar gain produced by Steam Audio's baked
    // pathing eqCoeffs. 1.0 = identity. Use > 1 to make through-portal
    // sound louder than the bake implies, without re-baking. Does NOT
    // affect the LPF blocking factor. Clamped to [0.1, 10.0].
    float    pathingGainScale = 1.0f;
    // Companion knob: multiplier on the LPF blocking factor emitted
    // by eqCoeffsToDspMapping. Legacy mapping (blocking = 1 − eqHigh)
    // produces blocking ≈ 0.98 for typical cross-room ambients
    // (eqHigh ≈ 0.02), pegging the door-LPF cutoff at ~400 Hz and
    // making distant voices unrecognisably muffled. Lower this
    // (e.g. 0.3-0.5) to keep the LPF more open. Clamped to [0.0, 1.0].
    float    pathingBlockingScale = 1.0f;
    // Minimum interval (seconds) between successive Steam Audio
    // pathing-simulation updates. iplSimulatorRunPathing is CPU-heavy
    // and runs on the PathingSimulator worker thread; this interval
    // sets how often the worker is signalled. Since staging went
    // event-driven, an idle due tick stages nothing — the interval is
    // door-event quantization delay, not a pacing knob; 0.05 s halves
    // the 0.1 s Unity/Unreal-default door-event latency at measured
    // near-zero cost (2026-07-19 survey). 0.0 = run every frame
    // (legacy / A-B diagnostic). Clamped to [0.0, 1.0] seconds.
    // (Default aligned with AudioService::mPathingUpdateInterval — this
    // value overwrites the service default at init wiring, so the two
    // must agree or a config without the key silently reverts.)
    float    pathingUpdateInterval = 0.05f;

    // Router-gated search (PLAN.PATHING_DESIGN.md §49 lever 1): suppress
    // a Steam Audio pathing solve whenever the hybrid gate's route says
    // the voice is unreachable — SA's findAlternatePaths would drain the
    // entire reachable probe component (~16-36 BVH rays per edge, the
    // measured 170-745 ms [PATHING_SLOW] class) only to return the same
    // no-route verdict the volume gate already silences the voice on.
    // Default ON; the switch exists for A/B measurement only.
    bool     pathingRouterGate = true;

    // Time constant (ms) for the audio-thread smoother on pathing EQ/SH
    // parameters. Steam Audio's built-in PathEffect ramps are frame-
    // count based and collapse to ~20-60 ms at our small buffers, so a
    // fresh solve with a large level change (door between loud/quiet
    // spaces) steps audibly; this smoother is time-based and frame-size
    // independent. 0 disables (verbatim application, Valve-plugin
    // behavior). Clamped to [0, 1000].
    float    pathingSmoothingMs = 100.0f;

    // Per-band weights for collapsing Steam Audio's 3-band eqCoeffs into
    // the scalar portalAttenuation gain. Applied as
    //   gain = wL·eqLow + wM·eqMid + wH·eqHigh
    // then scaled by pathingGainScale. Default {0.25, 0.50, 0.25}
    // matches the legacy mid-heavy perceptual weighting (roughly
    // A-weighted). Weights are NOT auto-normalised — sums other than
    // 1.0 produce a flat boost/cut. Each component clamped to [0, 1].
    // Typical tunings:
    //   {0.25, 0.50, 0.25} default mid-heavy
    //   {0.50, 0.40, 0.10} bass-heavy — fuller cross-room sounds
    //   {0.33, 0.34, 0.33} flat — highs contribute equally to loudness
    //   {0.10, 0.40, 0.50} treble-heavy — muffled = quiet
    float    pathingGainWeightLow  = 0.25f;
    float    pathingGainWeightMid  = 0.50f;
    float    pathingGainWeightHigh = 0.25f;

    // -- audio.spatialization: HRTF + distance model --
    float hrtfVolume          = 1.0f;   // HRTF output gain (1.0 = raw HRTF, lower = quieter)
    std::string hrtfInterpolation = "bilinear"; // HRTF interpolation: "nearest" or "bilinear"
    float spatialBlend        = 1.0f;   // binaural blend (0=mono, 1=full HRTF)

    // -- audio.ambient: P$AmbientHack tuning --
    //
    // Voice lifecycle (Group D — 2026-05): spawn fires on `dist < amb.radius`
    // (no hysteresis multiplier; the original-engine spawn boundary), halt
    // fires on a dB-based audibility threshold rather than the Euclidean
    // stop boundary. Halt is preceded by a `halt_fade_out_ms` ramp so the
    // discrete stopVoice event is perceptually hidden; spawn is followed by
    // a `spawn_fade_in_ms` ramp so the first audio callback's silent
    // initVoiceDSP defaults don't pop. See AmbientSoundManager.cpp for the
    // audibility derivation (mapping.gain × distanceAttenuation × schema_cb).
    int   ambDefaultPriority    = 64;   // priority for ambients without explicit value
    // [AMBIENT] volume ramp 0→full on voice spawn (0–2000 ms). 150 ms hides
    // the spawn pop without making the appearance feel laggy.
    int   ambientSpawnFadeInMs  = 150;
    // [AMBIENT] volume ramp full→0 before halt-stop (0–2000 ms). 250 ms
    // hides the halt pop; the voice keeps running through the fade so a
    // late audibility recovery can cancel and ramp back up.
    int   ambientHaltFadeOutMs  = 250;
    // [AMBIENT] dB level below which voice is considered "inaudible" and
    // halt is triggered (-80 to -20). −50 dB ≈ 0.00316 linear; well below
    // the perceptual floor of a stationary listener in a typical Thief
    // mission ambient mix.
    float ambientHaltAudibilityThresholdDb = -50.0f;
    // [AMBIENT] consecutive frames below threshold required to trigger
    // halt (5–600). At 60 Hz, 30 frames ≈ 0.5 s — long enough to ignore
    // single-frame eq-flicker dips, short enough that a player walking
    // away from a source halts the voice within a beat.
    int   ambientHaltBelowThresholdFrames = 30;
    // Per-voice spatialBlend override for AMB_ENVIRONMENTAL ambients (room
    // tone, wind, church reverberance). 1.0 = full HRTF point-source pan;
    // 0.0 = mono passthrough (no directional cue). Object-attached ambients
    // (no AMB_ENVIRONMENTAL flag) keep full HRTF at 1.0 regardless.
    // Default 0.3 = mostly diffuse with a subtle directional hint, so
    // "wind from outside" still leans in the right direction but doesn't
    // feel like a laser pointer.
    float ambEnvironmentalSpatialBlend = 0.3f;  // (0.0–1.0)
    // Global linear multiplier applied to every ambient voice's per-frame
    // volume. Compensates for the loudness re-baseline
    // introduced when Steam Audio became the sole player-audio propagation
    // authority (centibel falloff curve over schema radius retired). One
    // knob, default 1.0 = no change. Tune by ear.
    float ambGlobalVolumeScale = 1.0f;  // (0.0–4.0)

    // -- audio.mixer: global gains --
    float mixerMasterGain     = 1.0f;   // global output gain multiplier
    float mixerDirectGain     = 1.0f;   // dry-bus multiplier (direct path only)
    float mixerReflectionGain = 1.0f;   // reverb wet bus gain multiplier
    float reflectionRampMs    = 10.0f;  // reflection-bus fade-in time (ms) on first activation

    // -- audio.dsp: master bus DSP chain --
    bool  dspLimiter         = true;    // soft tanh limiter (prevents digital clipping)
    float dspLimiterKnee     = 0.8f;    // knee threshold (0.5–0.95, higher = later onset)
    bool  dspCompressor      = true;    // master bus compressor (tames transients)
    float dspCompThreshold   = -15.0f;  // compressor threshold in dBFS (-30 to 0)
    float dspCompRatio       = 3.0f;    // compression ratio (1.5 to 10)
    float dspCompAttackMs    = 10.0f;   // compressor attack time (1–100 ms)
    float dspCompReleaseMs   = 250.0f;  // compressor release time (50–2000 ms)
    bool  dspEQ              = true;    // low-shelf EQ (adds bass weight)
    float dspEQFreq          = 120.0f;  // EQ center frequency in Hz (60–500)
    float dspEQGain          = 3.0f;    // EQ gain in dB (-6 to +6)
    float dspEQQ             = 0.707f;  // EQ filter Q (0.3–2.0; 0.707 = Butterworth)
    bool  dspDucking         = false;   // ambient ducking when SFX plays (disabled by default)
    float dspDuckAmount      = 0.5f;    // ambient volume during ducking (0.1–1.0)
    float dspDuckAttackMs    = 50.0f;   // ducking attack time (10–500 ms)
    float dspDuckReleaseMs   = 500.0f;  // ducking release time (50–5000 ms)
    bool  dspWetSaturation       = false;  // wet-bus tape/phonograph saturator (off = transparent)
    float dspWetSaturationDrive  = 1.0f;   // saturation drive (1.0 = brick-wall only, 2-4 = tape, 5-10 = phonograph)


    // -- physics --
    int physicsRate = 60;  // physics timestep Hz: 12 = vintage (12.5Hz), 60 = modern, 120 = ultra

    // -- developer --
    bool showObjects      = true;   // render object meshes
    bool showFallbackCubes = false; // show colored cubes for objects with missing models
    bool portalCulling    = true;   // portal/frustum culling
    bool cameraCollision  = false;  // sphere collision against world geometry
    bool debugObjects     = false;  // dump per-object filtering diagnostics to stderr
    bool stepLog          = false;  // stair step diagnostics to stderr ([STEP] prefix)
    bool togglePlatforms  = false;  // auto-activate all moving terrain at startup
    bool noProbes         = false;  // skip probe baking (no spatial audio)
    bool audioLog         = false;  // enable audio/sound/schema log output

    // -- audio perf-capture (PLAN.AUDIO_PROFILING.md §1.2, §1.4) --
    // Label used to tag the per-run JSONL artifact directory.
    //   ./perf/<mission>/<utc_iso>__<perf_label>/audio_perf.jsonl
    // Defaults to "default" so an un-labelled invocation still produces an
    // artifact. Must be filesystem-safe (alphanumeric + '_-.') — the binary
    // validates and rejects on launch.
    std::string perfLabel = "default";
    // Walltime budget (seconds) from main() start. 0 = disabled (run forever
    // until SDL quit). Set via --exit-after-seconds N to let scripted sweeps
    // (tools/perf_sweep.sh) run unattended.
    float       exitAfterSeconds = 0.0f;
    // Capture the engine's final stereo f32 output to output.wav in the
    // per-run perf directory (next to audio_perf.jsonl). Listenable per-run
    // evidence for A/B comparisons; analyzed offline by
    // tools/wav_artifacts.py (PLAN.AUDIO_PERF.md PR 0.2). Set via
    // --capture-wav or YAML developer.capture_wav.
    bool        captureWav = false;

    // -- auto-fly probe-tour (companion to --exit-after-seconds) --
    // Drives the fly-mode camera through a deterministic random tour of the
    // N nearest pathing probes. Without movement, the listener position is
    // stationary and the JSONL perf artifact reflects only one geometric
    // configuration; with auto-fly enabled every sweep iteration captures
    // the same flythrough trajectory. See src/main/AutoFlyTour.h.
    //
    // Activation forces fly mode (physics off) on first frame after init
    // because the physics integrator owns camera position when on.
    bool        autoFly             = false;     // --auto-fly
    float       autoFlySpeed        = 10.0f;     // --auto-fly-speed (ft/s)
    int         autoFlyWaypoints    = 50;        // --auto-fly-waypoints
    uint32_t    autoFlySeed         = 0xC0FFEEu; // --auto-fly-seed
    float       autoFlyPauseSec     = 0.0f;      // --auto-fly-pause-sec

    // -- audio capture point (spin-in-place acoustic probe) --
    // --audio-capture x,y,z pins the listener at a fixed world point
    // (Dark Engine feet, Z-up), forces physics off, enables audio_log, and
    // spins the camera in place for `audioCaptureSeconds` completing
    // `audioCaptureRotations` full turns before exiting cleanly. Lets us do a
    // full-azimuth Steam Audio capture at a chosen point with no manual
    // navigation. See src/main/AudioCaptureSpin.h. Mutually exclusive with
    // --auto-fly (capture wins if both are given).
    bool        audioCapture          = false;   // --audio-capture x,y,z
    float       audioCaptureX         = 0.0f;     // target X
    float       audioCaptureY         = 0.0f;     // target Y
    float       audioCaptureZ         = 0.0f;     // target Z
    float       audioCaptureSeconds   = 15.0f;    // --audio-capture-seconds
    float       audioCaptureRotations = 3.0f;     // --audio-capture-rotations

    // -- auto-run probe-tour (on-foot stress harness) --
    // Physics-mode sibling of --auto-fly: the player RUNS a deterministic
    // waypoint tour of nearby pathing probes via the real movement-intent
    // API, generating footstep voices (~2.5 spawns/s at run speed), BSP
    // collision, and portal crossings — the audio stress profile a flying
    // camera cannot produce. See src/main/AutoRunTour.h. Mutually
    // exclusive with --auto-fly (auto-run wins with a warning) and
    // --audio-capture (capture wins).
    bool        autoRun             = false;      // --auto-run
    int         autoRunWaypoints    = 50;         // --auto-run-waypoints
    uint32_t    autoRunSeed         = 0xC0FFEEu;  // --auto-run-seed
    std::string autoRunSpeedMode    = "run";      // --auto-run-speed-mode run|walk|creep

    // -- audio sample-selection RNG seed --
    // Seeds AudioService's schema-sample-selection PRNG so two A/B stress
    // runs pick the SAME wav per schema event (sample choice varies clip
    // length → voice lifetime → voice-count profile). -1 = unseeded
    // (std::random_device, the default shipping behavior).
    int64_t     audioRngSeed        = -1;         // --audio-rng-seed

    // -- door-swing stress harness (DEV-ONLY) --
    // --stress-doors N toggles the N doors nearest the camera (of the
    // doors with a usable audio OBB) open/closed every ~2 s during a run.
    // Exists solely to exercise the O2a door-dirty-gated pathing
    // re-solve path and its [DOOR_ROUTE_LATENCY] staleness metric
    // (PLAN.PATHING_DESIGN.md §6 decision 1) under a scripted swing load
    // that a hands-off tour can't produce. Bypasses the frob/script
    // layer (DoorSystem::activate directly), so no door foley schemas
    // fire — intentional: the metric under test is route updates of
    // OTHER voices, not door sounds. 0 = disabled (the default; never
    // ship-enabled).
    int         stressDoors         = 0;          // --stress-doors N

    // --stress-door-ids "a,b,..." (DEV-ONLY, diagnostic companion to
    // --stress-doors): toggle EXACTLY these door object IDs every cycle
    // instead of the N nearest the camera. Exists for the door-event
    // pathing-latency hypothesis runs: single-door runs pinned to a door
    // of a known alternate-route detour class (NONE/LONG/SHORT — see
    // analysis/door_detour_class.py) need the same door swinging all run,
    // which the distance-ranked selection can't guarantee on a moving
    // tour. Empty = disabled; when set it overrides the nearest-N pick
    // (a nonzero --stress-doors is still required to arm the harness).
    std::vector<int32_t> stressDoorIDs;           // --stress-door-ids "a,b"

    // --spawn-override "x,y,z[,yaw]" (DEV-ONLY, diagnostic companion to
    // --stress-doors / --auto-run): force the camera/player start to an
    // explicit engine-feet position instead of the mission's spawn
    // marker. Positioned runtime measurements (e.g. the door-stress-
    // while-standing-in-a-hub-room cell from PLAN.PATHING_DESIGN.md §10)
    // need the listener parked in a SPECIFIC room; nothing ships a
    // teleport. Applied loudly ([SPAWN_OVERRIDE] banner) right after
    // initRuntimeState in DarknessRender.cpp. Never ship-enabled.
    bool        spawnOverride       = false;      // --spawn-override "x,y,z[,yaw]"
    float       spawnOverrideX      = 0.0f;
    float       spawnOverrideY      = 0.0f;
    float       spawnOverrideZ      = 0.0f;
    float       spawnOverrideYaw    = 0.0f;
};

// Result of CLI parsing — values that are CLI-only (not in YAML).
struct CliResult {
    const char* misPath    = nullptr;  // positional arg: mission file
    std::string resPath;               // --res <path>
    std::string schemasPath;           // --schemas <path>
    std::string configPath;            // --config <path>
    bool        helpRequested = false; // --help / -h
};

// Load settings from a YAML config file into cfg.
// Returns true if the file was loaded successfully.
// Returns false (silently) if the file doesn't exist — this is the normal case.
// Prints to stderr and returns false on parse errors.
inline bool loadConfigFromYAML(const std::string& path, RenderConfig& cfg) {
    // Check if file exists before trying to parse
    FILE* f = std::fopen(path.c_str(), "r");
    if (!f) return false;
    std::fclose(f);

    try {
        YAML::Node root = YAML::LoadFile(path);

        // paths section — fallback values used when --res / --schemas not given
        if (YAML::Node paths = root["paths"]) {
            if (paths["res"])     cfg.resPath     = paths["res"].as<std::string>();
            if (paths["schemas"]) cfg.schemasPath = paths["schemas"].as<std::string>();
        }

        // graphics section — both filter knobs use string enums for symmetry.
        if (YAML::Node gfx = root["graphics"]) {
            if (gfx["texture_filter"]) {
                std::string val = gfx["texture_filter"].as<std::string>();
                if      (val == "point")        cfg.filterMode = 0;
                else if (val == "bilinear")     cfg.filterMode = 1;
                else if (val == "trilinear")    cfg.filterMode = 2;
                else if (val == "anisotropic")  cfg.filterMode = 3;
                else                            cfg.filterMode = 0; // unknown → default
            }
            if (gfx["lightmap_filter"]) {
                std::string val = gfx["lightmap_filter"].as<std::string>();
                if      (val == "bicubic")  cfg.lightmapFiltering = 1;
                else                        cfg.lightmapFiltering = 0; // "bilinear" or unknown → default
            }
            if (gfx["linear_mips"])  cfg.linearMips = gfx["linear_mips"].as<bool>();
            if (gfx["sharp_mips"])   cfg.sharpMips  = gfx["sharp_mips"].as<bool>();
        }

        // water section
        if (YAML::Node water = root["water"]) {
            if (water["wave_amplitude"]) {
                cfg.waveAmplitude = clampConfigValue(
                    water["wave_amplitude"].as<float>(),
                    WaterRange::kWaveAmplitudeMin,
                    WaterRange::kWaveAmplitudeMax, "water.wave_amplitude");
            }
            if (water["uv_distortion"]) {
                cfg.uvDistortion = clampConfigValue(
                    water["uv_distortion"].as<float>(),
                    WaterRange::kUvDistortionMin,
                    WaterRange::kUvDistortionMax, "water.uv_distortion");
            }
            if (water["rotation_speed"]) {
                cfg.waterRotation = clampConfigValue(
                    water["rotation_speed"].as<float>(),
                    WaterRange::kWaterRotationMin,
                    WaterRange::kWaterRotationMax, "water.rotation_speed");
            }
            if (water["scroll_speed"]) {
                cfg.waterScrollSpeed = clampConfigValue(
                    water["scroll_speed"].as<float>(),
                    WaterRange::kWaterScrollMin,
                    WaterRange::kWaterScrollMax, "water.scroll_speed");
            }
        }

        // audio section — organized into named subsections.
        // Layout: audio.{performance,reflections,occlusion,propagation,
        //               spatialization,ambient,mixer,dsp}
        if (YAML::Node audio = root["audio"]) {
            // -- audio.performance --
            if (YAML::Node perf = audio["performance"]) {
                if (perf["sample_rate"]) {
                    int v = perf["sample_rate"].as<int>();
                    // Snap to one of the supported rates
                    if      (v <= 22050) cfg.audioSampleRate = 22050;
                    else if (v <= 32000) cfg.audioSampleRate = 32000;
                    else if (v <= 44100) cfg.audioSampleRate = 44100;
                    else if (v <= 48000) cfg.audioSampleRate = 48000;
                    else                 cfg.audioSampleRate = 96000;
                }
                if (perf["frame_size"]) {
                    cfg.audioFrameSize = perf["frame_size"].as<int>();
                    if (cfg.audioFrameSize < 256)  cfg.audioFrameSize = 256;
                    if (cfg.audioFrameSize > 4096) cfg.audioFrameSize = 4096;
                }
                if (perf["sound_cache_mb"]) {
                    cfg.audioSoundCacheMB = perf["sound_cache_mb"].as<int>();
                    if (cfg.audioSoundCacheMB < 4)    cfg.audioSoundCacheMB = 4;
                    if (cfg.audioSoundCacheMB > 1024) cfg.audioSoundCacheMB = 1024;
                }
                if (perf["rate_divisor"]) {
                    int div = perf["rate_divisor"].as<int>();
                    cfg.reflectionRateDivisor = (div >= 4) ? 4 : (div >= 2) ? 2 : 1;
                }
                if (perf["max_active_voices"]) {
                    cfg.maxActiveVoices = perf["max_active_voices"].as<int>();
                    if (cfg.maxActiveVoices < 8)   cfg.maxActiveVoices = 8;
                    if (cfg.maxActiveVoices > 256) cfg.maxActiveVoices = 256;
                }
                // Reverb voice caps (renamed from max_reflection_voices /
                // max_realtime_voices in 2026-05 config cleanup).
                if (perf["reverb_voices"]) {
                    cfg.reverbVoices = perf["reverb_voices"].as<int>();
                    if (cfg.reverbVoices < 0)  cfg.reverbVoices = 0;
                    if (cfg.reverbVoices > 64) cfg.reverbVoices = 64;
                }
                if (perf["reverb_voices_realtime"]) {
                    cfg.reverbVoicesRealtime = perf["reverb_voices_realtime"].as<int>();
                    if (cfg.reverbVoicesRealtime < 0)  cfg.reverbVoicesRealtime = 0;
                    if (cfg.reverbVoicesRealtime > 64) cfg.reverbVoicesRealtime = 64;
                }
                if (perf["reflection_throttle"]) {
                    cfg.reflectionThrottle = perf["reflection_throttle"].as<int>();
                    if (cfg.reflectionThrottle < 1)  cfg.reflectionThrottle = 1;
                    if (cfg.reflectionThrottle > 32) cfg.reflectionThrottle = 32;
                }
                if (perf["sim_max_occlusion_samples"]) {
                    cfg.simMaxOcclusionSamples = perf["sim_max_occlusion_samples"].as<int>();
                    if (cfg.simMaxOcclusionSamples < 4)   cfg.simMaxOcclusionSamples = 4;
                    if (cfg.simMaxOcclusionSamples > 256) cfg.simMaxOcclusionSamples = 256;
                }
                // Explicit thread counts for reverb work. Both 0 = auto.
                // Both > 0 = use literal values. Mixed = auto + warn (see
                // AudioService init).
                if (perf["conv_threads"]) {
                    cfg.convThreads = perf["conv_threads"].as<int>();
                    if (cfg.convThreads < 0)  cfg.convThreads = 0;
                    if (cfg.convThreads > 64) cfg.convThreads = 64;
                }
                if (perf["sim_threads"]) {
                    cfg.simThreads = perf["sim_threads"].as<int>();
                    if (cfg.simThreads < 0)  cfg.simThreads = 0;
                    if (cfg.simThreads > 64) cfg.simThreads = 64;
                }
                if (perf["scene_type"]) {
                    cfg.sceneType = perf["scene_type"].as<std::string>();
                    if (cfg.sceneType != "default" && cfg.sceneType != "embree")
                        cfg.sceneType = "default";
                }
                // Deprecated keys — emit one-shot warnings so existing yamls
                // get a friendly notice. AudioService auto-derives the
                // equivalent values now (see NOTES.AUDIO_CONFIG_AUDIT.md).
                static const char* kDeprecated[] = {
                    "convolution_workers", "simulator_threads",
                    "max_reflection_voices", "max_realtime_voices",
                    "sim_max_rays", "direct_max_sources",
                    "reflection_max_sources", "sim_max_sources",
                    "reflection_demote_hysteresis_frames",
                    "reverb_threads", "reverb_threads_conv_share",
                };
                for (const char* key : kDeprecated) {
                    if (perf[key]) {
                        std::fprintf(stderr,
                            "WARN: audio.performance.%s is no longer used "
                            "(replaced by conv_threads/sim_threads/"
                            "reverb_voices/reverb_voices_realtime or "
                            "auto-derived). Safe to remove from "
                            "darknessRender.yaml.\n", key);
                    }
                }
            }

            // -- audio.engine (device-callback / mixer-thread topology, PR D) --
            if (YAML::Node eng = audio["engine"]) {
                if (eng["ring_mixer"]) {
                    cfg.audioRingMixer = eng["ring_mixer"].as<bool>();
                }
                if (eng["ring_margin_ms"]) {
                    cfg.audioRingMarginMs = eng["ring_margin_ms"].as<float>();
                    // <= 0 = auto; clamp the ceiling so a typo can't
                    // request a multi-second ring.
                    if (cfg.audioRingMarginMs > 500.0f)
                        cfg.audioRingMarginMs = 500.0f;
                }
            }

            // -- audio.reflections --
            //
            // Layout (see PLAN.AUDIO_REALTIME_ARCHITECTURE.md):
            //   reflections.enabled                      — master toggle
            //   reflections.ambisonics_order             — REALTIME ambisonic
            //                                              order (top-level
            //                                              key, for symmetry
            //                                              with the existing
            //                                              ambisonicsOrder
            //                                              consumers in
            //                                              AudioService).
            //   reflections.type / hybrid_*              — algorithm select
            //   reflections.realtime.{rays,bounces,duration,diffuse_samples}
            //   reflections.bake.{rays,bounces,duration,diffuse_samples,ambisonics_order}
            //
            // bake.ambisonics_order may be >= the realtime order — the
            // runtime IPLReflectionEffect processes only the first
            // (realtime_order+1)^2 channels of each baked IR (Steam Audio
            // truncates the higher-order channels at apply time). The
            // AudioService validator rejects bake_order < realtime_order
            // only (the runtime cannot synthesise channels that the bake
            // did not generate).
            if (YAML::Node refl = audio["reflections"]) {
                if (refl["enabled"]) cfg.realtimeReflections = refl["enabled"].as<bool>();

                if (refl["ambisonics_order"]) {
                    cfg.ambisonicsOrder = refl["ambisonics_order"].as<int>();
                    if (cfg.ambisonicsOrder < 0) cfg.ambisonicsOrder = 0;
                    if (cfg.ambisonicsOrder > 3) cfg.ambisonicsOrder = 3;
                }

                if (refl["type"]) std::fprintf(stderr, "[FALLBACK] darknessRender.yaml: 'reflections.type' is deprecated and ignored — HYBRID mode is now the only supported reflection algorithm\n");
                if (refl["bake_skip"]) {
                    std::fprintf(stderr,
                        "[FALLBACK] darknessRender.yaml: "
                        "'audio.reflections.bake_skip' is deprecated and "
                        "ignored — reflection bake is no longer skippable; "
                        "every bake runs both pathing and reflection sections. "
                        "Remove the key from your YAML.\n");
                }
                if (refl["hybrid_transition_time"]) {
                    cfg.hybridTransitionTime = refl["hybrid_transition_time"].as<float>();
                    if (cfg.hybridTransitionTime < 0.1f) cfg.hybridTransitionTime = 0.1f;
                    if (cfg.hybridTransitionTime > 8.0f) cfg.hybridTransitionTime = 8.0f;
                }
                if (refl["hybrid_overlap_percent"]) {
                    cfg.hybridOverlapPercent = refl["hybrid_overlap_percent"].as<float>();
                    if (cfg.hybridOverlapPercent < 0.0f) cfg.hybridOverlapPercent = 0.0f;
                    if (cfg.hybridOverlapPercent > 1.0f) cfg.hybridOverlapPercent = 1.0f;
                }

                if (YAML::Node rt = refl["realtime"]) {
                    if (rt["rays"]) {
                        cfg.realtimeNumRays = rt["rays"].as<int>();
                        if (cfg.realtimeNumRays < 128)  cfg.realtimeNumRays = 128;
                        if (cfg.realtimeNumRays > 8192) cfg.realtimeNumRays = 8192;
                    }
                    if (rt["bounces"]) {
                        cfg.realtimeNumBounces = rt["bounces"].as<int>();
                        if (cfg.realtimeNumBounces < 1) cfg.realtimeNumBounces = 1;
                        if (cfg.realtimeNumBounces > 8) cfg.realtimeNumBounces = 8;
                    }
                    if (rt["duration"]) {
                        cfg.realtimeDuration = rt["duration"].as<float>();
                        if (cfg.realtimeDuration < 0.5f) cfg.realtimeDuration = 0.5f;
                        if (cfg.realtimeDuration > 4.0f) cfg.realtimeDuration = 4.0f;
                    }
                    if (rt["diffuse_samples"]) {
                        cfg.realtimeDiffuseSamples = rt["diffuse_samples"].as<int>();
                        if (cfg.realtimeDiffuseSamples < 16)  cfg.realtimeDiffuseSamples = 16;
                        if (cfg.realtimeDiffuseSamples > 256) cfg.realtimeDiffuseSamples = 256;
                    }
                }

                // New bake sub-block.
                if (YAML::Node bk = refl["bake"]) {
                    if (bk["rays"]) {
                        cfg.bakeNumRays = bk["rays"].as<int>();
                        if (cfg.bakeNumRays < 1024)  cfg.bakeNumRays = 1024;
                        if (cfg.bakeNumRays > 65536) cfg.bakeNumRays = 65536;
                    }
                    if (bk["bounces"]) {
                        cfg.bakeNumBounces = bk["bounces"].as<int>();
                        if (cfg.bakeNumBounces < 1)  cfg.bakeNumBounces = 1;
                        if (cfg.bakeNumBounces > 64) cfg.bakeNumBounces = 64;
                    }
                    if (bk["duration"]) {
                        cfg.bakeDuration = bk["duration"].as<float>();
                        if (cfg.bakeDuration < 0.5f) cfg.bakeDuration = 0.5f;
                        if (cfg.bakeDuration > 8.0f) cfg.bakeDuration = 8.0f;
                    }
                    if (bk["diffuse_samples"]) {
                        cfg.bakeDiffuseSamples = bk["diffuse_samples"].as<int>();
                        if (cfg.bakeDiffuseSamples < 32)   cfg.bakeDiffuseSamples = 32;
                        if (cfg.bakeDiffuseSamples > 4096) cfg.bakeDiffuseSamples = 4096;
                    }
                    if (bk["ambisonics_order"]) {
                        cfg.bakeAmbisonicsOrder = bk["ambisonics_order"].as<int>();
                        if (cfg.bakeAmbisonicsOrder < 0) cfg.bakeAmbisonicsOrder = 0;
                        if (cfg.bakeAmbisonicsOrder > 3) cfg.bakeAmbisonicsOrder = 3;
                    }
                }

                if (refl["runtime_ir_clamp_ms"]) std::fprintf(stderr, "[FALLBACK] darknessRender.yaml: 'reflections.runtime_ir_clamp_ms' is deprecated and ignored — the HYBRID convolution head is already bounded by hybrid_transition_time\n");
            }

            // -- audio.probes --
            // Bake-time grid parameters. A halved spacing quadruples probe
            // count on a 2D floor grid and adds proportional bake time, but
            // reduces footstep-reverb amplitude variance between probes.
            if (YAML::Node prb = audio["probes"]) {
                if (prb["spacing"]) {
                    cfg.audioProbeSpacingFt = prb["spacing"].as<float>();
                    if (cfg.audioProbeSpacingFt < 1.0f)  cfg.audioProbeSpacingFt = 1.0f;
                    if (cfg.audioProbeSpacingFt > 20.0f) cfg.audioProbeSpacingFt = 20.0f;
                }
                if (prb["height"]) {
                    cfg.audioProbeHeightFt = prb["height"].as<float>();
                    if (cfg.audioProbeHeightFt < 0.5f)  cfg.audioProbeHeightFt = 0.5f;
                    if (cfg.audioProbeHeightFt > 20.0f) cfg.audioProbeHeightFt = 20.0f;
                }
                if (prb["elevations"]) {
                    cfg.audioProbeElevations.clear();
                    for (const auto &el : prb["elevations"]) {
                        float v = el.as<float>();
                        if (v > 0.0f && v < 200.0f) {
                            cfg.audioProbeElevations.push_back(v);
                        }
                    }
                }
                if (prb["min_wall_clearance_ft"]) {
                    cfg.audioProbeMinWallClearanceFt =
                        prb["min_wall_clearance_ft"].as<float>();
                    // Hard floor of 0 (disables check); upper guard
                    // matches AudioService::setProbeMinWallClearanceFt.
                    if (cfg.audioProbeMinWallClearanceFt < 0.0f)
                        cfg.audioProbeMinWallClearanceFt = 0.0f;
                    if (cfg.audioProbeMinWallClearanceFt > 50.0f)
                        cfg.audioProbeMinWallClearanceFt = 50.0f;
                }
                if (prb["elevation_sparsity_mul"]) {
                    cfg.audioProbeElevationSparsityMul =
                        prb["elevation_sparsity_mul"].as<float>();
                    if (cfg.audioProbeElevationSparsityMul < 1.0f)
                        cfg.audioProbeElevationSparsityMul = 1.0f;
                    if (cfg.audioProbeElevationSparsityMul > 8.0f)
                        cfg.audioProbeElevationSparsityMul = 8.0f;
                }
                if (prb["global_dedup_radius_ft"]) {
                    cfg.audioProbeGlobalDedupRadiusFt =
                        prb["global_dedup_radius_ft"].as<float>();
                    if (cfg.audioProbeGlobalDedupRadiusFt < 0.0f)
                        cfg.audioProbeGlobalDedupRadiusFt = 0.0f;
                    if (cfg.audioProbeGlobalDedupRadiusFt > 10.0f)
                        cfg.audioProbeGlobalDedupRadiusFt = 10.0f;
                }
            }

            // -- audio.pathing_probes --
            if (YAML::Node pp = audio["pathing_probes"]) {
                if (pp["enabled"])
                    cfg.audioPathingProbesEnabled = pp["enabled"].as<bool>();
                if (pp["dedup_radius_ft"]) {
                    cfg.audioPathingDedupRadiusFt =
                        pp["dedup_radius_ft"].as<float>();
                    if (cfg.audioPathingDedupRadiusFt < 0.0f)
                        cfg.audioPathingDedupRadiusFt = 0.0f;
                    if (cfg.audioPathingDedupRadiusFt > 30.0f)
                        cfg.audioPathingDedupRadiusFt = 30.0f;
                }
                if (pp["vis_range_override_ft"]) {
                    cfg.audioPathingVisRangeOverrideFt =
                        pp["vis_range_override_ft"].as<float>();
                    if (cfg.audioPathingVisRangeOverrideFt < 0.0f)
                        cfg.audioPathingVisRangeOverrideFt = 0.0f;
                    if (cfg.audioPathingVisRangeOverrideFt > 400.0f)
                        cfg.audioPathingVisRangeOverrideFt = 400.0f;
                }
                if (pp["density"]) {
                    // Name list mirrors pathingProbeDensityFromName
                    // (ProbeManager.h, the canonical string→enum map;
                    // not included here to keep RenderConfig free of
                    // audio-stack headers). A name accepted here but
                    // unknown there is caught loudly at the setter
                    // boundary in DarknessRender.cpp.
                    const std::string d = pp["density"].as<std::string>();
                    if (d == "baseline" || d == "bends") {
                        cfg.audioPathingDensity = d;
                    } else {
                        // Reject-at-parse, loudly. "high" is reserved for
                        // a future Tier 2 (room-span subdivision) and is
                        // deliberately NOT accepted until that tier
                        // exists — accepting it now would be a silently-
                        // ignored knob (per the single-source-of-truth
                        // rule from the PR-A config audit). Keep the
                        // default rather than aborting the whole config
                        // load, but say exactly what happened.
                        std::fprintf(stderr,
                            "[FALLBACK] audio.pathing_probes.density: "
                            "invalid value '%s' — valid values are "
                            "'baseline' (Tier 0: original room/portal "
                            "graph nodes) and 'bends' (Tier 1, "
                            "default: + flanking pairs at every "
                            "portal). 'high' is reserved for a future "
                            "Tier 2 and not yet implemented. Using "
                            "default '%s'.\n",
                            d.c_str(), cfg.audioPathingDensity.c_str());
                    }
                }
            }

            // -- audio.occlusion --
            if (YAML::Node occ = audio["occlusion"]) {
                if (occ["radius"]) {
                    cfg.occlusionRadius = occ["radius"].as<float>();
                    // Range in engine units (feet). Converted to meters at the
                    // IPL boundary, so 30 ft ≈ 9 m caps a "very large industrial
                    // source" which is about as wide as makes physical sense.
                    if (cfg.occlusionRadius < 0.3f) cfg.occlusionRadius = 0.3f;
                    if (cfg.occlusionRadius > 30.0f) cfg.occlusionRadius = 30.0f;
                }
                if (occ["samples"]) {
                    cfg.occlusionSamples = occ["samples"].as<int>();
                    if (cfg.occlusionSamples < 4)  cfg.occlusionSamples = 4;
                    if (cfg.occlusionSamples > 64) cfg.occlusionSamples = 64;
                }
                if (occ["transmission_scale"]) {
                    cfg.transmissionScale = occ["transmission_scale"].as<float>();
                    if (cfg.transmissionScale < 0.1f)   cfg.transmissionScale = 0.1f;
                    if (cfg.transmissionScale > 100.0f) cfg.transmissionScale = 100.0f;
                }
                if (occ["absorption_scale"]) {
                    cfg.absorptionScale = occ["absorption_scale"].as<float>();
                    if (cfg.absorptionScale < 0.01f) cfg.absorptionScale = 0.01f;
                    if (cfg.absorptionScale > 10.0f) cfg.absorptionScale = 10.0f;
                }
            }

            // -- audio.propagation --
            if (YAML::Node prop = audio["propagation"]) {
                if (prop["portal_routing"]) cfg.portalRouting = prop["portal_routing"].as<bool>();
                if (prop["probe_pathing"])  cfg.probePathing  = prop["probe_pathing"].as<bool>();
                if (prop["max_distance"]) {
                    cfg.propagationMaxDist = prop["max_distance"].as<float>();
                    if (cfg.propagationMaxDist < 10.0f)   cfg.propagationMaxDist = 10.0f;
                    if (cfg.propagationMaxDist > 5000.0f) cfg.propagationMaxDist = 5000.0f;
                }
                if (prop["door_lpf_open_hz"]) {
                    cfg.doorLpfOpenHz = prop["door_lpf_open_hz"].as<float>();
                    if (cfg.doorLpfOpenHz < 1000.0f)  cfg.doorLpfOpenHz = 1000.0f;
                    if (cfg.doorLpfOpenHz > 24000.0f) cfg.doorLpfOpenHz = 24000.0f;
                }
                if (prop["door_lpf_blocked_hz"]) {
                    cfg.doorLpfBlockedHz = prop["door_lpf_blocked_hz"].as<float>();
                    if (cfg.doorLpfBlockedHz < 100.0f)  cfg.doorLpfBlockedHz = 100.0f;
                    if (cfg.doorLpfBlockedHz > 10000.0f) cfg.doorLpfBlockedHz = 10000.0f;
                }
                if (prop["min_attenuation"]) {
                    cfg.propMinAttenuation = prop["min_attenuation"].as<float>();
                    if (cfg.propMinAttenuation < 0.0f)   cfg.propMinAttenuation = 0.0f;
                    if (cfg.propMinAttenuation > 0.1f)   cfg.propMinAttenuation = 0.1f;
                }
                if (prop["max_paths"]) {
                    int n = prop["max_paths"].as<int>();
                    if (n < 1) n = 1;
                    if (n > 4) n = 4;
                    cfg.propMaxPaths = static_cast<uint32_t>(n);
                }
                if (prop["max_path_diff"]) {
                    cfg.propMaxPathDiff = prop["max_path_diff"].as<float>();
                    if (cfg.propMaxPathDiff < 0.0f)  cfg.propMaxPathDiff = 0.0f;
                    if (cfg.propMaxPathDiff > 50.0f) cfg.propMaxPathDiff = 50.0f;
                }
                if (prop["pathing_gain_scale"]) {
                    cfg.pathingGainScale = prop["pathing_gain_scale"].as<float>();
                    if (cfg.pathingGainScale < 0.1f)  cfg.pathingGainScale = 0.1f;
                    if (cfg.pathingGainScale > 10.0f) cfg.pathingGainScale = 10.0f;
                }
                if (prop["pathing_blocking_scale"]) {
                    cfg.pathingBlockingScale = prop["pathing_blocking_scale"].as<float>();
                    if (cfg.pathingBlockingScale < 0.0f) cfg.pathingBlockingScale = 0.0f;
                    if (cfg.pathingBlockingScale > 1.0f) cfg.pathingBlockingScale = 1.0f;
                }
                if (prop["pathing_update_interval"]) {
                    cfg.pathingUpdateInterval = prop["pathing_update_interval"].as<float>();
                    if (cfg.pathingUpdateInterval < 0.0f) cfg.pathingUpdateInterval = 0.0f;
                    if (cfg.pathingUpdateInterval > 1.0f) cfg.pathingUpdateInterval = 1.0f;
                }
                if (prop["pathing_router_gate"]) {
                    cfg.pathingRouterGate = prop["pathing_router_gate"].as<bool>();
                }
                if (prop["pathing_smoothing_ms"]) {
                    cfg.pathingSmoothingMs = prop["pathing_smoothing_ms"].as<float>();
                    if (cfg.pathingSmoothingMs < 0.0f) cfg.pathingSmoothingMs = 0.0f;
                    if (cfg.pathingSmoothingMs > 1000.0f) cfg.pathingSmoothingMs = 1000.0f;
                }
                if (prop["pathing_gain_band_weights"]) {
                    YAML::Node w = prop["pathing_gain_band_weights"];
                    if (w.IsSequence() && w.size() == 3) {
                        auto clamp01 = [](float v) {
                            if (v < 0.0f) return 0.0f;
                            return v > 1.0f ? 1.0f : v;
                        };
                        cfg.pathingGainWeightLow  = clamp01(w[0].as<float>());
                        cfg.pathingGainWeightMid  = clamp01(w[1].as<float>());
                        cfg.pathingGainWeightHigh = clamp01(w[2].as<float>());
                    } else {
                        std::fprintf(stderr, "[CONFIG] pathing_gain_band_weights must be a "
                                     "3-element sequence [low, mid, high]; ignoring.\n");
                    }
                }
            }

            // -- audio.spatialization --
            if (YAML::Node spat = audio["spatialization"]) {
                if (spat["hrtf_volume"]) {
                    cfg.hrtfVolume = spat["hrtf_volume"].as<float>();
                    if (cfg.hrtfVolume < 0.0f) cfg.hrtfVolume = 0.0f;
                    if (cfg.hrtfVolume > 4.0f) cfg.hrtfVolume = 4.0f;
                }
                if (spat["hrtf_interpolation"]) {
                    cfg.hrtfInterpolation = spat["hrtf_interpolation"].as<std::string>();
                    if (cfg.hrtfInterpolation != "nearest" && cfg.hrtfInterpolation != "bilinear")
                        cfg.hrtfInterpolation = "bilinear";
                }
                if (spat["spatial_blend"]) {
                    cfg.spatialBlend = spat["spatial_blend"].as<float>();
                    if (cfg.spatialBlend < 0.0f) cfg.spatialBlend = 0.0f;
                    if (cfg.spatialBlend > 1.0f) cfg.spatialBlend = 1.0f;
                }
                if (spat["distance_model"]) {
                    std::fprintf(stderr,
                        "[CONFIG_DEPRECATED] audio.spatialization.distance_model "
                        "is no longer supported. The distance model is now always "
                        "INVERSEDISTANCE with per-voice minDistance derived from "
                        "the schema's P$SchAttFac. Remove the key from your YAML.\n");
                }
            }

            // -- audio.ambient --
            if (YAML::Node amb = audio["ambient"]) {
                // Group D (2026-05): the radius-multiplier gate was replaced
                // by a dB-based audibility halt. Old keys are still parsed
                // but ignored; emit a one-shot WARN so existing yamls get a
                // pointer to the replacement knobs.
                static const char* kDeprecatedAmbient[] = {
                    "hysteresis_start_mul",
                    "hysteresis_stop_mul",
                };
                for (const char* key : kDeprecatedAmbient) {
                    if (amb[key]) {
                        std::fprintf(stderr,
                            "WARN: audio.ambient.%s is no longer used "
                            "(replaced by halt_audibility_threshold_db / "
                            "halt_below_threshold_frames + spawn_fade_in_ms"
                            " / halt_fade_out_ms — see darknessRender."
                            "example.yaml). Safe to remove from "
                            "darknessRender.yaml.\n", key);
                    }
                }
                if (amb["falloff_curve"]) {
                    std::fprintf(stderr,
                        "[CONFIG_DEPRECATED] audio.ambient.falloff_curve is no "
                        "longer supported. The Dark Engine centibel falloff curve "
                        "was removed when Steam Audio became the sole player-audio "
                        "propagation authority — Steam Audio's distance model "
                        "handles all attenuation now. Tune audio.ambient."
                        "global_volume_scale to compensate for the loudness "
                        "re-baseline. Remove the key from your YAML.\n");
                }
                if (amb["spawn_fade_in_ms"]) {
                    cfg.ambientSpawnFadeInMs = amb["spawn_fade_in_ms"].as<int>();
                    if (cfg.ambientSpawnFadeInMs < 0)    cfg.ambientSpawnFadeInMs = 0;
                    if (cfg.ambientSpawnFadeInMs > 2000) cfg.ambientSpawnFadeInMs = 2000;
                }
                if (amb["halt_fade_out_ms"]) {
                    cfg.ambientHaltFadeOutMs = amb["halt_fade_out_ms"].as<int>();
                    if (cfg.ambientHaltFadeOutMs < 0)    cfg.ambientHaltFadeOutMs = 0;
                    if (cfg.ambientHaltFadeOutMs > 2000) cfg.ambientHaltFadeOutMs = 2000;
                }
                if (amb["halt_audibility_threshold_db"]) {
                    cfg.ambientHaltAudibilityThresholdDb =
                        amb["halt_audibility_threshold_db"].as<float>();
                    if (cfg.ambientHaltAudibilityThresholdDb < -80.0f)
                        cfg.ambientHaltAudibilityThresholdDb = -80.0f;
                    if (cfg.ambientHaltAudibilityThresholdDb > -20.0f)
                        cfg.ambientHaltAudibilityThresholdDb = -20.0f;
                }
                if (amb["halt_below_threshold_frames"]) {
                    cfg.ambientHaltBelowThresholdFrames =
                        amb["halt_below_threshold_frames"].as<int>();
                    if (cfg.ambientHaltBelowThresholdFrames < 5)
                        cfg.ambientHaltBelowThresholdFrames = 5;
                    if (cfg.ambientHaltBelowThresholdFrames > 600)
                        cfg.ambientHaltBelowThresholdFrames = 600;
                }
                if (amb["default_priority"]) {
                    cfg.ambDefaultPriority = amb["default_priority"].as<int>();
                    if (cfg.ambDefaultPriority < 0)   cfg.ambDefaultPriority = 0;
                    if (cfg.ambDefaultPriority > 255) cfg.ambDefaultPriority = 255;
                }
                if (amb["environmental_spatial_blend"]) {
                    cfg.ambEnvironmentalSpatialBlend = amb["environmental_spatial_blend"].as<float>();
                    if (cfg.ambEnvironmentalSpatialBlend < 0.0f) cfg.ambEnvironmentalSpatialBlend = 0.0f;
                    if (cfg.ambEnvironmentalSpatialBlend > 1.0f) cfg.ambEnvironmentalSpatialBlend = 1.0f;
                }
                if (amb["global_volume_scale"]) {
                    cfg.ambGlobalVolumeScale = amb["global_volume_scale"].as<float>();
                    if (cfg.ambGlobalVolumeScale < 0.0f) cfg.ambGlobalVolumeScale = 0.0f;
                    if (cfg.ambGlobalVolumeScale > 4.0f) cfg.ambGlobalVolumeScale = 4.0f;
                }
            }

            // -- audio.mixer --
            if (YAML::Node mix = audio["mixer"]) {
                if (mix["master_gain"]) {
                    cfg.mixerMasterGain = mix["master_gain"].as<float>();
                    if (cfg.mixerMasterGain < 0.0f) cfg.mixerMasterGain = 0.0f;
                    if (cfg.mixerMasterGain > 4.0f) cfg.mixerMasterGain = 4.0f;
                }
                if (mix["direct_gain"]) {
                    cfg.mixerDirectGain = mix["direct_gain"].as<float>();
                    if (cfg.mixerDirectGain < 0.0f) cfg.mixerDirectGain = 0.0f;
                    if (cfg.mixerDirectGain > 4.0f) cfg.mixerDirectGain = 4.0f;
                }
                if (mix["reflection_gain"]) {
                    cfg.mixerReflectionGain = mix["reflection_gain"].as<float>();
                    if (cfg.mixerReflectionGain < 0.0f) cfg.mixerReflectionGain = 0.0f;
                    if (cfg.mixerReflectionGain > 4.0f) cfg.mixerReflectionGain = 4.0f;
                }
                if (mix["reflection_ramp_ms"]) {
                    cfg.reflectionRampMs = mix["reflection_ramp_ms"].as<float>();
                    if (cfg.reflectionRampMs < 1.0f)   cfg.reflectionRampMs = 1.0f;
                    if (cfg.reflectionRampMs > 1000.0f) cfg.reflectionRampMs = 1000.0f;
                }
            }

            // -- audio.dsp --
            if (YAML::Node dsp = audio["dsp"]) {
                if (dsp["limiter_enabled"]) cfg.dspLimiter = dsp["limiter_enabled"].as<bool>();
                if (dsp["limiter_knee"]) {
                    cfg.dspLimiterKnee = dsp["limiter_knee"].as<float>();
                    if (cfg.dspLimiterKnee < 0.5f)  cfg.dspLimiterKnee = 0.5f;
                    if (cfg.dspLimiterKnee > 0.95f) cfg.dspLimiterKnee = 0.95f;
                }
                if (dsp["compressor_enabled"]) cfg.dspCompressor = dsp["compressor_enabled"].as<bool>();
                if (dsp["compressor_threshold_db"]) {
                    cfg.dspCompThreshold = dsp["compressor_threshold_db"].as<float>();
                    if (cfg.dspCompThreshold < -30.0f) cfg.dspCompThreshold = -30.0f;
                    if (cfg.dspCompThreshold > 0.0f)   cfg.dspCompThreshold = 0.0f;
                }
                if (dsp["compressor_ratio"]) {
                    cfg.dspCompRatio = dsp["compressor_ratio"].as<float>();
                    if (cfg.dspCompRatio < 1.5f) cfg.dspCompRatio = 1.5f;
                    if (cfg.dspCompRatio > 10.0f) cfg.dspCompRatio = 10.0f;
                }
                if (dsp["compressor_attack_ms"]) {
                    cfg.dspCompAttackMs = dsp["compressor_attack_ms"].as<float>();
                    if (cfg.dspCompAttackMs < 1.0f)   cfg.dspCompAttackMs = 1.0f;
                    if (cfg.dspCompAttackMs > 100.0f) cfg.dspCompAttackMs = 100.0f;
                }
                if (dsp["compressor_release_ms"]) {
                    cfg.dspCompReleaseMs = dsp["compressor_release_ms"].as<float>();
                    if (cfg.dspCompReleaseMs < 50.0f)   cfg.dspCompReleaseMs = 50.0f;
                    if (cfg.dspCompReleaseMs > 2000.0f) cfg.dspCompReleaseMs = 2000.0f;
                }
                if (dsp["eq_enabled"]) cfg.dspEQ = dsp["eq_enabled"].as<bool>();
                if (dsp["eq_freq_hz"]) {
                    cfg.dspEQFreq = dsp["eq_freq_hz"].as<float>();
                    if (cfg.dspEQFreq < 60.0f)  cfg.dspEQFreq = 60.0f;
                    if (cfg.dspEQFreq > 500.0f) cfg.dspEQFreq = 500.0f;
                }
                if (dsp["eq_gain_db"]) {
                    cfg.dspEQGain = dsp["eq_gain_db"].as<float>();
                    if (cfg.dspEQGain < -6.0f) cfg.dspEQGain = -6.0f;
                    if (cfg.dspEQGain > 6.0f)  cfg.dspEQGain = 6.0f;
                }
                if (dsp["eq_q"]) {
                    cfg.dspEQQ = dsp["eq_q"].as<float>();
                    if (cfg.dspEQQ < 0.3f) cfg.dspEQQ = 0.3f;
                    if (cfg.dspEQQ > 2.0f) cfg.dspEQQ = 2.0f;
                }
                if (dsp["ducking_enabled"]) cfg.dspDucking = dsp["ducking_enabled"].as<bool>();
                if (dsp["ducking_amount"]) {
                    cfg.dspDuckAmount = dsp["ducking_amount"].as<float>();
                    if (cfg.dspDuckAmount < 0.1f) cfg.dspDuckAmount = 0.1f;
                    if (cfg.dspDuckAmount > 1.0f) cfg.dspDuckAmount = 1.0f;
                }
                if (dsp["ducking_attack_ms"]) {
                    cfg.dspDuckAttackMs = dsp["ducking_attack_ms"].as<float>();
                    if (cfg.dspDuckAttackMs < 10.0f)  cfg.dspDuckAttackMs = 10.0f;
                    if (cfg.dspDuckAttackMs > 500.0f) cfg.dspDuckAttackMs = 500.0f;
                }
                if (dsp["ducking_release_ms"]) {
                    cfg.dspDuckReleaseMs = dsp["ducking_release_ms"].as<float>();
                    if (cfg.dspDuckReleaseMs < 50.0f)   cfg.dspDuckReleaseMs = 50.0f;
                    if (cfg.dspDuckReleaseMs > 5000.0f) cfg.dspDuckReleaseMs = 5000.0f;
                }
                if (dsp["wet_saturation_enabled"]) cfg.dspWetSaturation = dsp["wet_saturation_enabled"].as<bool>();
                if (dsp["wet_saturation_drive"]) {
                    cfg.dspWetSaturationDrive = dsp["wet_saturation_drive"].as<float>();
                    if (cfg.dspWetSaturationDrive < 1.0f)  cfg.dspWetSaturationDrive = 1.0f;
                    if (cfg.dspWetSaturationDrive > 10.0f) cfg.dspWetSaturationDrive = 10.0f;
                }
            }
        }

        // physics section
        if (YAML::Node phys = root["physics"]) {
            if (phys["rate"]) {
                // Accept string names or integer Hz values
                try {
                    std::string val = phys["rate"].as<std::string>();
                    if (val == "vintage" || val == "12.5" || val == "12")
                        cfg.physicsRate = 12;
                    else if (val == "ultra" || val == "120")
                        cfg.physicsRate = 120;
                    else
                        cfg.physicsRate = 60;  // "modern" or unknown → default
                } catch (...) {
                    int val = phys["rate"].as<int>(60);
                    if (val <= 12) cfg.physicsRate = 12;
                    else if (val >= 120) cfg.physicsRate = 120;
                    else cfg.physicsRate = 60;
                }
            }
        }

        // developer section
        if (YAML::Node dev = root["developer"]) {
            if (dev["show_objects"])        cfg.showObjects       = dev["show_objects"].as<bool>();
            if (dev["show_fallback_cubes"]) cfg.showFallbackCubes = dev["show_fallback_cubes"].as<bool>();
            if (dev["portal_culling"])      cfg.portalCulling     = dev["portal_culling"].as<bool>();
            if (dev["camera_collision"])    cfg.cameraCollision   = dev["camera_collision"].as<bool>();
            if (dev["step_log"])            cfg.stepLog           = dev["step_log"].as<bool>();
            if (dev["debug_objects"])       cfg.debugObjects      = dev["debug_objects"].as<bool>();
            if (dev["toggle_platforms"])    cfg.togglePlatforms   = dev["toggle_platforms"].as<bool>();
            if (dev["no_probes"])           cfg.noProbes          = dev["no_probes"].as<bool>();
            if (dev["audio_log"])           cfg.audioLog          = dev["audio_log"].as<bool>();
            if (dev["capture_wav"])         cfg.captureWav        = dev["capture_wav"].as<bool>();
        }

        std::fprintf(stderr, "Loaded config from %s\n", path.c_str());
        return true;
    } catch (const YAML::Exception& e) {
        // The file existed and we tried to parse it — a syntax error here
        // means everything past the bad line was silently skipped, leaving
        // the program running with a half-applied config. The library only
        // returns false (so unit tests can exercise this path); the BINARY
        // is expected to detect "file existed but parse failed" and abort.
        // See DarknessRender.cpp's call site for the abort.
        std::fprintf(stderr,
            "\nERROR: failed to parse %s\n"
            "  %s\n"
            "Fix the syntax error or remove the file to run with defaults.\n",
            path.c_str(), e.what());
        return false;
    }
}

// ── --set dispatch table (PLAN.AUDIO_PROFILING.md §1.4) ──
//
// Generic CLI override: `--set audio.dotted.path=value`. Applied AFTER YAML
// load (so it wins over file values) and BEFORE the AudioService init pass
// reads `cfg`. The supported leaves cover every audio-tunable knob enumerated
// in PLAN.AUDIO_PROFILING.md §3 (the per-tunable metric map). Each entry
// here maps yaml-key -> RenderConfig field; clamping is performed inline,
// matching the YAML loader's clamps so a --set override and a YAML value
// behave identically.
//
// Leaf types supported:
//   - float  (numeric leaves; std::stof)
//   - int    (integer leaves; std::stoi)
//   - bool   (true/false/1/0/yes/no)
//   - string (verbatim, with light validation for enum-like keys)
//
// Per feedback_no_silent_fallbacks: unknown paths emit a [FALLBACK] stderr
// line listing the path that was rejected so a typo is impossible to miss.
//
// Implementation note: a hardcoded if/else dispatch is fine — the set is
// small, additive over the project lifetime, and avoids pulling in a
// YAML/JSON-path mini-parser. When a new YAML knob lands, also add it
// here (and to the help text further down).
inline bool applySetOverride(const std::string& path, const std::string& valueStr,
                             RenderConfig& cfg) {
    auto toBool = [&](bool& out) -> bool {
        if (valueStr == "true"  || valueStr == "1" || valueStr == "yes") { out = true;  return true; }
        if (valueStr == "false" || valueStr == "0" || valueStr == "no")  { out = false; return true; }
        return false;
    };
    auto toFloat = [&](float& out) -> bool {
        try { out = std::stof(valueStr); return true; } catch (...) { return false; }
    };
    auto toInt = [&](int& out) -> bool {
        try { out = std::stoi(valueStr); return true; } catch (...) { return false; }
    };

    // Helpers wrap field assignment + clamp into one line each. Each lambda
    // returns the new clamped value for traceability; the discard is fine.
    auto clampF = [](float v, float lo, float hi) {
        return (v < lo) ? lo : (v > hi ? hi : v);
    };
    auto clampI = [](int v, int lo, int hi) {
        return (v < lo) ? lo : (v > hi ? hi : v);
    };

    // -- audio.performance --
    if (path == "audio.performance.sample_rate") {
        int v; if (!toInt(v)) return false;
        if      (v <= 22050) cfg.audioSampleRate = 22050;
        else if (v <= 32000) cfg.audioSampleRate = 32000;
        else if (v <= 44100) cfg.audioSampleRate = 44100;
        else if (v <= 48000) cfg.audioSampleRate = 48000;
        else                 cfg.audioSampleRate = 96000;
        return true;
    }
    if (path == "audio.performance.frame_size") {
        int v; if (!toInt(v)) return false;
        cfg.audioFrameSize = clampI(v, 256, 4096); return true;
    }
    if (path == "audio.performance.sound_cache_mb") {
        int v; if (!toInt(v)) return false;
        cfg.audioSoundCacheMB = clampI(v, 4, 1024); return true;
    }
    if (path == "audio.performance.rate_divisor") {
        int v; if (!toInt(v)) return false;
        cfg.reflectionRateDivisor = (v >= 4) ? 4 : (v >= 2) ? 2 : 1; return true;
    }
    if (path == "audio.performance.max_active_voices") {
        int v; if (!toInt(v)) return false;
        cfg.maxActiveVoices = clampI(v, 8, 256); return true;
    }
    if (path == "audio.performance.reverb_voices") {
        int v; if (!toInt(v)) return false;
        cfg.reverbVoices = clampI(v, 0, 64); return true;
    }
    if (path == "audio.performance.reverb_voices_realtime") {
        int v; if (!toInt(v)) return false;
        cfg.reverbVoicesRealtime = clampI(v, 0, 64); return true;
    }
    if (path == "audio.performance.reflection_throttle") {
        int v; if (!toInt(v)) return false;
        cfg.reflectionThrottle = clampI(v, 1, 32); return true;
    }
    if (path == "audio.performance.sim_max_occlusion_samples") {
        int v; if (!toInt(v)) return false;
        cfg.simMaxOcclusionSamples = clampI(v, 4, 256); return true;
    }
    if (path == "audio.performance.conv_threads") {
        int v; if (!toInt(v)) return false;
        cfg.convThreads = clampI(v, 0, 64); return true;
    }
    if (path == "audio.performance.sim_threads") {
        int v; if (!toInt(v)) return false;
        cfg.simThreads = clampI(v, 0, 64); return true;
    }
    if (path == "audio.performance.scene_type") {
        cfg.sceneType = (valueStr == "embree" ? "embree" : "default");
        return true;
    }

    // -- audio.engine (PR D ring mixer) --
    if (path == "audio.engine.ring_mixer") {
        return toBool(cfg.audioRingMixer);
    }
    if (path == "audio.engine.ring_margin_ms") {
        float v; if (!toFloat(v)) return false;
        // <= 0 = auto (two engine blocks, min 21.4 ms); cap mirrors the
        // YAML loader's anti-typo ceiling.
        cfg.audioRingMarginMs = (v > 500.0f) ? 500.0f : v;
        return true;
    }

    // -- audio.reflections --
    if (path == "audio.reflections.enabled") {
        return toBool(cfg.realtimeReflections);
    }
    if (path == "audio.reflections.ambisonics_order") {
        int v; if (!toInt(v)) return false;
        cfg.ambisonicsOrder = clampI(v, 0, 3); return true;
    }
    if (path == "audio.reflections.bake_skip") {
        std::fprintf(stderr,
            "[FALLBACK] applySetOverride: 'audio.reflections.bake_skip' "
            "is deprecated and ignored — reflection bake is no longer "
            "skippable.\n");
        return true;
    }
    if (path == "audio.reflections.hybrid_transition_time") {
        float v; if (!toFloat(v)) return false;
        cfg.hybridTransitionTime = clampF(v, 0.1f, 8.0f); return true;
    }
    if (path == "audio.reflections.hybrid_overlap_percent") {
        float v; if (!toFloat(v)) return false;
        cfg.hybridOverlapPercent = clampF(v, 0.0f, 1.0f); return true;
    }
    if (path == "audio.reflections.realtime.rays") {
        int v; if (!toInt(v)) return false;
        cfg.realtimeNumRays = clampI(v, 128, 8192); return true;
    }
    if (path == "audio.reflections.realtime.bounces") {
        int v; if (!toInt(v)) return false;
        cfg.realtimeNumBounces = clampI(v, 1, 8); return true;
    }
    if (path == "audio.reflections.realtime.duration") {
        float v; if (!toFloat(v)) return false;
        cfg.realtimeDuration = clampF(v, 0.5f, 4.0f); return true;
    }
    if (path == "audio.reflections.realtime.diffuse_samples") {
        int v; if (!toInt(v)) return false;
        cfg.realtimeDiffuseSamples = clampI(v, 16, 256); return true;
    }
    if (path == "audio.reflections.bake.rays") {
        int v; if (!toInt(v)) return false;
        cfg.bakeNumRays = clampI(v, 1024, 65536); return true;
    }
    if (path == "audio.reflections.bake.bounces") {
        int v; if (!toInt(v)) return false;
        cfg.bakeNumBounces = clampI(v, 1, 64); return true;
    }
    if (path == "audio.reflections.bake.duration") {
        float v; if (!toFloat(v)) return false;
        cfg.bakeDuration = clampF(v, 0.5f, 8.0f); return true;
    }
    if (path == "audio.reflections.bake.diffuse_samples") {
        int v; if (!toInt(v)) return false;
        cfg.bakeDiffuseSamples = clampI(v, 32, 4096); return true;
    }
    if (path == "audio.reflections.bake.ambisonics_order") {
        int v; if (!toInt(v)) return false;
        cfg.bakeAmbisonicsOrder = clampI(v, 0, 3); return true;
    }

    // -- audio.probes --
    if (path == "audio.probes.spacing") {
        float v; if (!toFloat(v)) return false;
        cfg.audioProbeSpacingFt = clampF(v, 1.0f, 20.0f); return true;
    }
    if (path == "audio.probes.height") {
        float v; if (!toFloat(v)) return false;
        cfg.audioProbeHeightFt = clampF(v, 0.5f, 20.0f); return true;
    }
    if (path == "audio.probes.min_wall_clearance_ft") {
        float v; if (!toFloat(v)) return false;
        cfg.audioProbeMinWallClearanceFt = clampF(v, 0.0f, 50.0f); return true;
    }
    if (path == "audio.probes.elevation_sparsity_mul") {
        float v; if (!toFloat(v)) return false;
        cfg.audioProbeElevationSparsityMul = clampF(v, 1.0f, 8.0f); return true;
    }
    if (path == "audio.probes.global_dedup_radius_ft") {
        float v; if (!toFloat(v)) return false;
        cfg.audioProbeGlobalDedupRadiusFt = clampF(v, 0.0f, 10.0f); return true;
    }

    // -- audio.pathing_probes --
    if (path == "audio.pathing_probes.enabled") {
        return toBool(cfg.audioPathingProbesEnabled);
    }
    if (path == "audio.pathing_probes.dedup_radius_ft") {
        float v; if (!toFloat(v)) return false;
        cfg.audioPathingDedupRadiusFt = clampF(v, 0.0f, 30.0f); return true;
    }
    if (path == "audio.pathing_probes.vis_range_override_ft") {
        float v; if (!toFloat(v)) return false;
        cfg.audioPathingVisRangeOverrideFt = clampF(v, 0.0f, 400.0f);
        return true;
    }
    if (path == "audio.pathing_probes.density") {
        // Same validation as the YAML parser: baseline | bends only
        // ("high" reserved for a future Tier 2); name list mirrors
        // pathingProbeDensityFromName (ProbeManager.h) — see the YAML
        // parser comment. Returning false routes through the caller's
        // loud invalid-value report.
        if (valueStr == "baseline" || valueStr == "bends") {
            cfg.audioPathingDensity = valueStr;
            return true;
        }
        std::fprintf(stderr,
            "[FALLBACK] audio.pathing_probes.density: invalid value "
            "'%s' — valid: 'baseline' | 'bends' ('high' reserved, not "
            "yet implemented)\n", valueStr.c_str());
        return false;
    }

    // -- audio.occlusion --
    if (path == "audio.occlusion.radius") {
        float v; if (!toFloat(v)) return false;
        cfg.occlusionRadius = clampF(v, 0.3f, 30.0f); return true;
    }
    if (path == "audio.occlusion.samples") {
        int v; if (!toInt(v)) return false;
        cfg.occlusionSamples = clampI(v, 4, 64); return true;
    }
    if (path == "audio.occlusion.transmission_scale") {
        float v; if (!toFloat(v)) return false;
        cfg.transmissionScale = clampF(v, 0.1f, 100.0f); return true;
    }
    if (path == "audio.occlusion.absorption_scale") {
        float v; if (!toFloat(v)) return false;
        cfg.absorptionScale = clampF(v, 0.01f, 10.0f); return true;
    }

    // -- audio.propagation --
    if (path == "audio.propagation.portal_routing") {
        return toBool(cfg.portalRouting);
    }
    if (path == "audio.propagation.probe_pathing") {
        return toBool(cfg.probePathing);
    }
    if (path == "audio.propagation.max_distance") {
        float v; if (!toFloat(v)) return false;
        cfg.propagationMaxDist = clampF(v, 10.0f, 5000.0f); return true;
    }
    if (path == "audio.propagation.door_lpf_open_hz") {
        float v; if (!toFloat(v)) return false;
        cfg.doorLpfOpenHz = clampF(v, 1000.0f, 24000.0f); return true;
    }
    if (path == "audio.propagation.door_lpf_blocked_hz") {
        float v; if (!toFloat(v)) return false;
        cfg.doorLpfBlockedHz = clampF(v, 100.0f, 10000.0f); return true;
    }
    if (path == "audio.propagation.min_attenuation") {
        float v; if (!toFloat(v)) return false;
        cfg.propMinAttenuation = clampF(v, 0.0f, 0.1f); return true;
    }
    if (path == "audio.propagation.max_paths") {
        int v; if (!toInt(v)) return false;
        cfg.propMaxPaths = static_cast<uint32_t>(clampI(v, 1, 4)); return true;
    }
    if (path == "audio.propagation.max_path_diff") {
        float v; if (!toFloat(v)) return false;
        cfg.propMaxPathDiff = clampF(v, 0.0f, 50.0f); return true;
    }
    if (path == "audio.propagation.pathing_gain_scale") {
        float v; if (!toFloat(v)) return false;
        cfg.pathingGainScale = clampF(v, 0.1f, 10.0f); return true;
    }
    if (path == "audio.propagation.pathing_blocking_scale") {
        float v; if (!toFloat(v)) return false;
        cfg.pathingBlockingScale = clampF(v, 0.0f, 1.0f); return true;
    }
    if (path == "audio.propagation.pathing_update_interval") {
        float v; if (!toFloat(v)) return false;
        cfg.pathingUpdateInterval = clampF(v, 0.0f, 1.0f); return true;
    }
    if (path == "audio.propagation.pathing_router_gate") {
        bool v; if (!toBool(v)) return false;
        cfg.pathingRouterGate = v; return true;
    }
    if (path == "audio.propagation.pathing_smoothing_ms") {
        float v; if (!toFloat(v)) return false;
        cfg.pathingSmoothingMs = clampF(v, 0.0f, 1000.0f); return true;
    }

    // -- audio.spatialization --
    if (path == "audio.spatialization.hrtf_volume") {
        float v; if (!toFloat(v)) return false;
        cfg.hrtfVolume = clampF(v, 0.0f, 4.0f); return true;
    }
    if (path == "audio.spatialization.hrtf_interpolation") {
        cfg.hrtfInterpolation = (valueStr == "nearest") ? "nearest" : "bilinear";
        return true;
    }
    if (path == "audio.spatialization.spatial_blend") {
        float v; if (!toFloat(v)) return false;
        cfg.spatialBlend = clampF(v, 0.0f, 1.0f); return true;
    }

    // -- audio.ambient --
    if (path == "audio.ambient.spawn_fade_in_ms") {
        int v; if (!toInt(v)) return false;
        cfg.ambientSpawnFadeInMs = clampI(v, 0, 2000); return true;
    }
    if (path == "audio.ambient.halt_fade_out_ms") {
        int v; if (!toInt(v)) return false;
        cfg.ambientHaltFadeOutMs = clampI(v, 0, 2000); return true;
    }
    if (path == "audio.ambient.halt_audibility_threshold_db") {
        float v; if (!toFloat(v)) return false;
        cfg.ambientHaltAudibilityThresholdDb = clampF(v, -80.0f, -20.0f); return true;
    }
    if (path == "audio.ambient.halt_below_threshold_frames") {
        int v; if (!toInt(v)) return false;
        cfg.ambientHaltBelowThresholdFrames = clampI(v, 5, 600); return true;
    }
    if (path == "audio.ambient.default_priority") {
        int v; if (!toInt(v)) return false;
        cfg.ambDefaultPriority = clampI(v, 0, 255); return true;
    }
    if (path == "audio.ambient.environmental_spatial_blend") {
        float v; if (!toFloat(v)) return false;
        cfg.ambEnvironmentalSpatialBlend = clampF(v, 0.0f, 1.0f); return true;
    }
    if (path == "audio.ambient.global_volume_scale") {
        float v; if (!toFloat(v)) return false;
        cfg.ambGlobalVolumeScale = clampF(v, 0.0f, 4.0f); return true;
    }

    // -- audio.mixer --
    if (path == "audio.mixer.master_gain") {
        float v; if (!toFloat(v)) return false;
        cfg.mixerMasterGain = clampF(v, 0.0f, 4.0f); return true;
    }
    if (path == "audio.mixer.direct_gain") {
        float v; if (!toFloat(v)) return false;
        cfg.mixerDirectGain = clampF(v, 0.0f, 4.0f); return true;
    }
    if (path == "audio.mixer.reflection_gain") {
        float v; if (!toFloat(v)) return false;
        cfg.mixerReflectionGain = clampF(v, 0.0f, 4.0f); return true;
    }
    if (path == "audio.mixer.reflection_ramp_ms") {
        float v; if (!toFloat(v)) return false;
        cfg.reflectionRampMs = clampF(v, 1.0f, 1000.0f); return true;
    }

    // -- audio.dsp --
    if (path == "audio.dsp.limiter_enabled")   { return toBool(cfg.dspLimiter); }
    if (path == "audio.dsp.limiter_knee") {
        float v; if (!toFloat(v)) return false;
        cfg.dspLimiterKnee = clampF(v, 0.5f, 0.95f); return true;
    }
    if (path == "audio.dsp.compressor_enabled") { return toBool(cfg.dspCompressor); }
    if (path == "audio.dsp.compressor_threshold_db") {
        float v; if (!toFloat(v)) return false;
        cfg.dspCompThreshold = clampF(v, -30.0f, 0.0f); return true;
    }
    if (path == "audio.dsp.compressor_ratio") {
        float v; if (!toFloat(v)) return false;
        cfg.dspCompRatio = clampF(v, 1.5f, 10.0f); return true;
    }
    if (path == "audio.dsp.compressor_attack_ms") {
        float v; if (!toFloat(v)) return false;
        cfg.dspCompAttackMs = clampF(v, 1.0f, 100.0f); return true;
    }
    if (path == "audio.dsp.compressor_release_ms") {
        float v; if (!toFloat(v)) return false;
        cfg.dspCompReleaseMs = clampF(v, 50.0f, 2000.0f); return true;
    }
    if (path == "audio.dsp.eq_enabled") { return toBool(cfg.dspEQ); }
    if (path == "audio.dsp.eq_freq_hz") {
        float v; if (!toFloat(v)) return false;
        cfg.dspEQFreq = clampF(v, 60.0f, 500.0f); return true;
    }
    if (path == "audio.dsp.eq_gain_db") {
        float v; if (!toFloat(v)) return false;
        cfg.dspEQGain = clampF(v, -6.0f, 6.0f); return true;
    }
    if (path == "audio.dsp.eq_q") {
        float v; if (!toFloat(v)) return false;
        cfg.dspEQQ = clampF(v, 0.3f, 2.0f); return true;
    }
    if (path == "audio.dsp.ducking_enabled") { return toBool(cfg.dspDucking); }
    if (path == "audio.dsp.ducking_amount") {
        float v; if (!toFloat(v)) return false;
        cfg.dspDuckAmount = clampF(v, 0.1f, 1.0f); return true;
    }
    if (path == "audio.dsp.ducking_attack_ms") {
        float v; if (!toFloat(v)) return false;
        cfg.dspDuckAttackMs = clampF(v, 10.0f, 500.0f); return true;
    }
    if (path == "audio.dsp.ducking_release_ms") {
        float v; if (!toFloat(v)) return false;
        cfg.dspDuckReleaseMs = clampF(v, 50.0f, 5000.0f); return true;
    }
    if (path == "audio.dsp.wet_saturation_enabled") { return toBool(cfg.dspWetSaturation); }
    if (path == "audio.dsp.wet_saturation_drive") {
        float v; if (!toFloat(v)) return false;
        cfg.dspWetSaturationDrive = clampF(v, 1.0f, 10.0f); return true;
    }

    return false; // unknown path
}

// Validate --perf-label string: only [A-Za-z0-9_.-] allowed so the resulting
// directory name is safe across macOS / Linux / Windows + shell-quote-free.
inline bool isPerfLabelValid(const std::string& s) {
    if (s.empty() || s.size() > 64) return false;
    for (char c : s) {
        bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                  (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.';
        if (!ok) return false;
    }
    return true;
}

// Parse CLI arguments. The CLI surface is intentionally minimal — every
// other tunable lives in the YAML config. The flags below are the things
// that genuinely need per-invocation override:
//   <mission.mis>      first non-flag arg, the mission to load
//   --res <path>       runtime asset directory (overrides paths.res)
//   --schemas <path>   schema directory (overrides paths.schemas)
//   --config <path>    YAML path (defaults to ./darknessRender.yaml)
//   --force-pathing-bake    drop existing pathing section + re-bake it
//   --set <p>=<v>      generic YAML-path override; repeatable
//   --perf-label <s>   tag the per-run audio_perf.jsonl directory
//   --exit-after-seconds N  exit cleanly after N seconds of wall-clock
//   --auto-fly         enable deterministic probe-tour flythrough
//   --auto-fly-speed N        ft/s (default 10)
//   --auto-fly-waypoints N    N-nearest probes to visit (default 50)
//   --auto-fly-seed N         shuffle seed (default 0xC0FFEE)
//   --auto-fly-pause-sec N    dwell per waypoint (default 0)
//   --audio-capture x,y,z     pin listener at a point, spin in place, exit
//   --audio-capture-seconds N capture window length (default 15)
//   --audio-capture-rotations N full yaw turns over the window (default 3)
//   --capture-wav      record final engine output to output.wav in the
//                      per-run perf directory (= developer.capture_wav)
//   --stress-doors N   DEV-ONLY: toggle the N nearest doors every ~2 s
//                      (O2a door-route-latency stress harness)
//   --stress-door-ids "a,b"  DEV-ONLY: with --stress-doors armed, toggle
//                      exactly these door object IDs instead of nearest-N
//   --spawn-override "x,y,z[,yaw]"  DEV-ONLY: force the camera/player start
//                      position (positioned diagnostic runs)
//   --help / -h        print usage
//
// Unknown flags are reported but otherwise ignored — when a removed flag
// shows up in old shell history, the user gets a clear message instead
// of a silent parse miss.
inline CliResult applyCliOverrides(int argc, char* argv[], RenderConfig& cfg) {
    CliResult cli;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            cli.helpRequested = true;
        } else if (std::strcmp(argv[i], "--res") == 0 && i + 1 < argc) {
            cli.resPath = argv[++i];
        } else if (std::strcmp(argv[i], "--schemas") == 0 && i + 1 < argc) {
            cli.schemasPath = argv[++i];
        } else if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            cli.configPath = argv[++i];
        } else if (std::strcmp(argv[i], "--skip-reflection-bake") == 0) {
            // Deprecated CLI flag — reflection bake is no longer
            // skippable; every probe-bake-needed path runs the full bake
            // (pathing + reflections).
            std::fprintf(stderr,
                "[FALLBACK] --skip-reflection-bake is deprecated and "
                "ignored — reflection bake is no longer skippable. "
                "Remove the flag from your invocation.\n");
        } else if (std::strcmp(argv[i], "--force-pathing-bake") == 0) {
            // Force a fresh pathing bake even when the existing .probes
            // file already has a valid pathing section. See
            // PLAN.AUDIO_PROFILING.md §4.3.
            cfg.forcePathingBake = true;
        } else if (std::strcmp(argv[i], "--set") == 0 && i + 1 < argc) {
            // --set audio.foo.bar=value  (yaml-dotted-path=leaf)
            std::string arg = argv[++i];
            auto eq = arg.find('=');
            if (eq == std::string::npos) {
                std::fprintf(stderr,
                    "[FALLBACK] --set: missing '=' in '%s' — expected "
                    "audio.dotted.path=value (ignored)\n", arg.c_str());
                continue;
            }
            std::string path  = arg.substr(0, eq);
            std::string value = arg.substr(eq + 1);
            if (!applySetOverride(path, value, cfg)) {
                std::fprintf(stderr,
                    "[FALLBACK] --set: unknown YAML path '%s' — ignored "
                    "(value was '%s')\n", path.c_str(), value.c_str());
            } else {
                std::fprintf(stderr,
                    "--set: '%s' = '%s' applied (clamped to legal range)\n",
                    path.c_str(), value.c_str());
            }
        } else if (std::strcmp(argv[i], "--perf-label") == 0 && i + 1 < argc) {
            std::string label = argv[++i];
            if (!isPerfLabelValid(label)) {
                std::fprintf(stderr,
                    "[FALLBACK] --perf-label: '%s' rejected — must be "
                    "1-64 chars of [A-Za-z0-9_.-]. Keeping default '%s'.\n",
                    label.c_str(), cfg.perfLabel.c_str());
            } else {
                cfg.perfLabel = label;
            }
        } else if (std::strcmp(argv[i], "--exit-after-seconds") == 0 && i + 1 < argc) {
            try {
                cfg.exitAfterSeconds = std::stof(argv[++i]);
                if (cfg.exitAfterSeconds < 0.0f) cfg.exitAfterSeconds = 0.0f;
            } catch (...) {
                std::fprintf(stderr,
                    "[FALLBACK] --exit-after-seconds: non-numeric value "
                    "'%s' — ignored (run remains open-ended)\n", argv[i]);
            }
        } else if (std::strcmp(argv[i], "--auto-fly") == 0) {
            cfg.autoFly = true;
        } else if (std::strcmp(argv[i], "--auto-fly-speed") == 0 && i + 1 < argc) {
            try {
                cfg.autoFlySpeed = std::stof(argv[++i]);
                if (cfg.autoFlySpeed < 0.0f) cfg.autoFlySpeed = 0.0f;
            } catch (...) {
                std::fprintf(stderr,
                    "[FALLBACK] --auto-fly-speed: non-numeric value '%s'"
                    " — keeping default %.1f\n", argv[i], cfg.autoFlySpeed);
            }
        } else if (std::strcmp(argv[i], "--auto-fly-waypoints") == 0 && i + 1 < argc) {
            try {
                cfg.autoFlyWaypoints = std::stoi(argv[++i]);
                if (cfg.autoFlyWaypoints < 1) cfg.autoFlyWaypoints = 1;
            } catch (...) {
                std::fprintf(stderr,
                    "[FALLBACK] --auto-fly-waypoints: non-integer value "
                    "'%s' — keeping default %d\n",
                    argv[i], cfg.autoFlyWaypoints);
            }
        } else if (std::strcmp(argv[i], "--auto-fly-seed") == 0 && i + 1 < argc) {
            try {
                // Accept decimal or 0xHEX; std::stoul base=0 auto-detects.
                cfg.autoFlySeed = static_cast<uint32_t>(
                    std::stoul(argv[++i], nullptr, 0));
            } catch (...) {
                std::fprintf(stderr,
                    "[FALLBACK] --auto-fly-seed: invalid '%s' — keeping "
                    "default 0x%08x\n", argv[i], cfg.autoFlySeed);
            }
        } else if (std::strcmp(argv[i], "--auto-fly-pause-sec") == 0 && i + 1 < argc) {
            try {
                cfg.autoFlyPauseSec = std::stof(argv[++i]);
                if (cfg.autoFlyPauseSec < 0.0f) cfg.autoFlyPauseSec = 0.0f;
            } catch (...) {
                std::fprintf(stderr,
                    "[FALLBACK] --auto-fly-pause-sec: non-numeric value "
                    "'%s' — keeping default %.2f\n",
                    argv[i], cfg.autoFlyPauseSec);
            }
        } else if (std::strcmp(argv[i], "--audio-capture") == 0 && i + 1 < argc) {
            // --audio-capture x,y,z — pin the listener at a world point and
            // spin in place for a hands-free acoustic capture. Parsed as a
            // single comma-separated arg so the position is one shell token.
            float x = 0.0f, y = 0.0f, z = 0.0f;
            if (std::sscanf(argv[++i], "%f,%f,%f", &x, &y, &z) == 3) {
                cfg.audioCapture  = true;
                cfg.audioCaptureX = x;
                cfg.audioCaptureY = y;
                cfg.audioCaptureZ = z;
            } else {
                std::fprintf(stderr,
                    "[FALLBACK] --audio-capture: could not parse '%s' as "
                    "x,y,z (e.g. --audio-capture 12.5,-45,-47) — capture "
                    "disabled\n", argv[i]);
            }
        } else if (std::strcmp(argv[i], "--audio-capture-seconds") == 0 && i + 1 < argc) {
            try {
                cfg.audioCaptureSeconds = std::stof(argv[++i]);
                if (cfg.audioCaptureSeconds < 0.1f) cfg.audioCaptureSeconds = 0.1f;
            } catch (...) {
                std::fprintf(stderr,
                    "[FALLBACK] --audio-capture-seconds: non-numeric value "
                    "'%s' — keeping default %.1f\n",
                    argv[i], cfg.audioCaptureSeconds);
            }
        } else if (std::strcmp(argv[i], "--audio-capture-rotations") == 0 && i + 1 < argc) {
            try {
                cfg.audioCaptureRotations = std::stof(argv[++i]);
                if (cfg.audioCaptureRotations < 0.0f) cfg.audioCaptureRotations = 0.0f;
            } catch (...) {
                std::fprintf(stderr,
                    "[FALLBACK] --audio-capture-rotations: non-numeric value "
                    "'%s' — keeping default %.1f\n",
                    argv[i], cfg.audioCaptureRotations);
            }
        } else if (std::strcmp(argv[i], "--auto-run") == 0) {
            cfg.autoRun = true;
        } else if (std::strcmp(argv[i], "--auto-run-waypoints") == 0 && i + 1 < argc) {
            try {
                cfg.autoRunWaypoints = std::stoi(argv[++i]);
                if (cfg.autoRunWaypoints < 1) cfg.autoRunWaypoints = 1;
            } catch (...) {
                std::fprintf(stderr,
                    "[FALLBACK] --auto-run-waypoints: non-integer value "
                    "'%s' — keeping default %d\n",
                    argv[i], cfg.autoRunWaypoints);
            }
        } else if (std::strcmp(argv[i], "--auto-run-seed") == 0 && i + 1 < argc) {
            try {
                // Accept decimal or 0xHEX; std::stoul base=0 auto-detects.
                cfg.autoRunSeed = static_cast<uint32_t>(
                    std::stoul(argv[++i], nullptr, 0));
            } catch (...) {
                std::fprintf(stderr,
                    "[FALLBACK] --auto-run-seed: invalid '%s' — keeping "
                    "default 0x%08x\n", argv[i], cfg.autoRunSeed);
            }
        } else if (std::strcmp(argv[i], "--auto-run-speed-mode") == 0 && i + 1 < argc) {
            const char *mode = argv[++i];
            if (std::strcmp(mode, "run") == 0 || std::strcmp(mode, "walk") == 0
                || std::strcmp(mode, "creep") == 0) {
                cfg.autoRunSpeedMode = mode;
            } else {
                std::fprintf(stderr,
                    "[FALLBACK] --auto-run-speed-mode: unknown mode '%s' "
                    "(expected run|walk|creep) — keeping default '%s'\n",
                    mode, cfg.autoRunSpeedMode.c_str());
            }
        } else if (std::strcmp(argv[i], "--audio-rng-seed") == 0 && i + 1 < argc) {
            try {
                // Accept decimal or 0xHEX. Negative = leave unseeded.
                cfg.audioRngSeed = static_cast<int64_t>(
                    std::stoll(argv[++i], nullptr, 0));
            } catch (...) {
                std::fprintf(stderr,
                    "[FALLBACK] --audio-rng-seed: invalid '%s' — keeping "
                    "unseeded (random_device)\n", argv[i]);
            }
        } else if (std::strcmp(argv[i], "--stress-doors") == 0 && i + 1 < argc) {
            // DEV-ONLY door-swing stress harness (see AppConfig comment).
            try {
                cfg.stressDoors = std::stoi(argv[++i]);
                if (cfg.stressDoors < 0) cfg.stressDoors = 0;
            } catch (...) {
                std::fprintf(stderr,
                    "[FALLBACK] --stress-doors: non-integer value '%s' — "
                    "harness stays disabled\n", argv[i]);
                cfg.stressDoors = 0;
            }
        } else if (std::strcmp(argv[i], "--stress-door-ids") == 0 && i + 1 < argc) {
            // DEV-ONLY explicit-door companion to --stress-doors (see
            // AppConfig comment). Comma-separated object IDs; a malformed
            // token discards the whole list LOUDLY rather than silently
            // stressing a partial set.
            const char* arg = argv[++i];
            std::vector<int32_t> ids;
            bool ok = true;
            const char* p = arg;
            while (*p != '\0') {
                char* end = nullptr;
                long v = std::strtol(p, &end, 10);
                if (end == p) { ok = false; break; }
                ids.push_back(static_cast<int32_t>(v));
                p = end;
                if (*p == ',') ++p;
                else if (*p != '\0') { ok = false; break; }
            }
            if (ok && !ids.empty()) {
                cfg.stressDoorIDs = ids;
            } else {
                std::fprintf(stderr,
                    "[FALLBACK] --stress-door-ids: malformed list '%s' — "
                    "explicit-door override stays disabled\n", arg);
            }
        } else if (std::strcmp(argv[i], "--spawn-override") == 0 && i + 1 < argc) {
            // DEV-ONLY positioned-start override (see AppConfig comment).
            // "x,y,z" or "x,y,z,yaw"; a malformed token discards the
            // whole override LOUDLY rather than silently starting
            // somewhere unintended.
            const char* arg = argv[++i];
            float vals[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            int   count = 0;
            bool  ok = true;
            const char* p = arg;
            while (*p != '\0' && count < 4) {
                char* end = nullptr;
                float v = std::strtof(p, &end);
                if (end == p) { ok = false; break; }
                vals[count++] = v;
                p = end;
                if (*p == ',') ++p;
                else if (*p != '\0') { ok = false; break; }
            }
            if (ok && *p == '\0' && (count == 3 || count == 4)) {
                cfg.spawnOverride    = true;
                cfg.spawnOverrideX   = vals[0];
                cfg.spawnOverrideY   = vals[1];
                cfg.spawnOverrideZ   = vals[2];
                cfg.spawnOverrideYaw = (count == 4) ? vals[3] : 0.0f;
            } else {
                std::fprintf(stderr,
                    "[FALLBACK] --spawn-override: malformed \"x,y,z[,yaw]\" "
                    "value '%s' — using the mission spawn\n", arg);
            }
        } else if (std::strcmp(argv[i], "--bake-quality") == 0 && i + 1 < argc) {
            // Bake-quality profile. "dev" forces the fast iteration bake
            // (rays=2048 bounces=8 diffuse=256, ~64× cheaper per probe
            // than the ship-quality yaml settings — MISS2's 2590-probe
            // ship bake projected ~25 HOURS). "ship" is a documented
            // no-op: respect the yaml. Applied here because CLI parsing
            // runs after the yaml load, so this deliberately overrides
            // reflections.bake.* values.
            const char *q = argv[++i];
            if (std::strcmp(q, "dev") == 0) {
                // Reflection rays halved 4096 → 2048 (2026-07: dev bakes
                // must total < 10 min; reflection phase measured ~10 min
                // at 4096 on MISS2's 626 dev probes, cost is linear in
                // rays → ~5 min). Dev-tier IR fidelity is acceptable for
                // iteration; ship bakes are untouched.
                cfg.bakeNumRays        = 2048;
                cfg.bakeNumBounces     = 8;
                cfg.bakeDiffuseSamples = 256;
                // Pathing visibility sampling selects
                // kPathingVisSamplesDev for BOTH bake and runtime — see
                // devBakeProfile. (Ship and Dev are both 4 since
                // 2026-07-11, so this no longer changes ray count; the
                // profile plumbing is kept for the .probes header record.)
                cfg.devBakeProfile     = true;
                // Density reduction (user directive 2026-07-05: dev bakes
                // must be < 10 min). FLOOR_POLY emits one candidate per
                // BSP floor polygon regardless of spacing, so the GLOBAL
                // DEDUP radius is the density control: 12 ft collapses
                // MISS2's 3,304 candidates ~4-6x harder than the yaml's
                // 5 ft. Spacing raised alongside because committed probes
                // get influence radius = spacing — the sparser set must
                // still cover the level or [PERF refl_cache] hitRate
                // drops and reverb regions go dark (watch that metric on
                // every dev-bake validation).
                cfg.audioProbeGlobalDedupRadiusFt = 18.0f;
                cfg.audioProbeSpacingFt           = 20.0f;
                std::fprintf(stderr,
                    "--bake-quality dev: bake rays=2048 bounces=8 "
                    "diffuse=256 (~64x cheaper/probe than ship yaml) + "
                    "probe density reduced (global_dedup 18 ft, spacing "
                    "20 ft); pathing numSamples 4 (same as ship; "
                    "recorded in the .probes header). Cached "
                    ".probes from this bake are DEV QUALITY/DENSITY — "
                    "re-bake without this flag for milestone/ship "
                    "fidelity (the header mismatch check will do it "
                    "automatically, pathing-only).\n");
            } else if (std::strcmp(q, "ship") != 0) {
                std::fprintf(stderr,
                    "[FALLBACK] --bake-quality: unknown profile '%s' "
                    "(expected dev|ship) — keeping yaml bake settings\n", q);
            }
        } else if (std::strcmp(argv[i], "--audio-log") == 0) {
            // CLI mirror of the YAML `developer.audio_log` key. Perf runs
            // need it: nearly all [PERF *] histogram recording is gated on
            // the audio_log verbosity flag, and the generic --set resolver
            // only covers audio.* leaves — developer.* is out of its reach.
            cfg.audioLog = true;
        } else if (std::strcmp(argv[i], "--capture-wav") == 0) {
            // CLI mirror of the YAML `developer.capture_wav` key. Records
            // the engine's final stereo output to output.wav next to the
            // per-run audio_perf.jsonl — listenable evidence for A/B runs,
            // analyzed offline by tools/wav_artifacts.py
            // (PLAN.AUDIO_PERF.md PR 0.2).
            cfg.captureWav = true;
        } else if (argv[i][0] != '-' && !cli.misPath) {
            // First non-flag argument is the mission file
            cli.misPath = argv[i];
        } else if (argv[i][0] == '-') {
            std::fprintf(stderr,
                "Warning: unknown CLI flag '%s' (ignored). All tunables live in the YAML config; run --help for the full list.\n",
                argv[i]);
        }
    }

    return cli;
}

} // namespace Darkness
