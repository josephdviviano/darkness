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

    // -- water --
    float waveAmplitude   = 0.3f;    // vertex Z displacement in world units (0 = flat)
    float uvDistortion    = 0.015f;  // UV wobble strength (0 = no ripple)
    float waterRotation   = 0.015f;  // UV rotation speed in rad/s (0 = no rotation)
    float waterScrollSpeed = 0.05f;  // UV scroll speed in world units/s (0 = no drift)

    // -- audio.performance: engine + sim throughput knobs --
    int  audioSampleRate         = 48000; // device output sample rate (Hz; clamped to 22050/32000/44100/48000/96000)
    int  audioFrameSize          = 1024;  // audio engine frame size in samples (256–4096)
    int  audioSoundCacheMB       = 64;    // decoded-audio LRU cache budget (MB)
    int  reflectionRateDivisor   = 2;     // reflection pipeline rate: 1=full 48kHz, 2=half 24kHz, 4=quarter 12kHz
    int  convolutionWorkers      = 0;     // convolution worker threads (0 = auto: hwconc-3)
    int  simulatorThreads        = 0;     // ray-tracing sim threads (0 = auto: hwconc-2)
    int  maxActiveVoices         = 64;    // hard cap on simultaneous voices (Dark Engine baseline)
    int  maxReflectionVoices     = 16;    // max voices with per-source convolution reverb
    int  reflectionThrottle      = 4;     // run reflection sim every Nth frame (1–32)
    int  simMaxOcclusionSamples  = 32;    // upper bound on per-source occlusion samples (Steam Audio sim)
    int  simMaxRays              = 4096;  // upper bound on rays per sim step (Steam Audio sim)
    // Source pools: direct sim is cheap per-source so it can be much larger
    // than the reflection pool. Legacy YAML key `sim_max_sources` (if present)
    // is mapped to `reflectionMaxSources` for back-compat.
    int  directMaxSources        = 256;   // Steam Audio direct-only simulator source pool size (4–1024)
    int  reflectionMaxSources    = 32;    // Steam Audio reflection simulator source pool size (4–256)
    // Demote-only fallback (stage 2.2): every voice starts with a
    // reflection source so it routes through baked reverb by default.
    // Normal voices that stay outside the top-N reflection candidate pool
    // for this many consecutive frames release their source and play dry
    // for the rest of their life. Default ≈10 s at 60 fps — intentionally
    // high so this fires only as a sparing fallback when convolution/sim
    // budget is under genuine pressure. PlayerEmitted + Ambient voices
    // are excluded.
    int  reflectionDemoteHysteresisFrames = 600; // (1–3600)
    std::string sceneType        = "default"; // IPL scene backend: "default" or "embree"

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

    /// Reflection effect algorithm. Hybrid splits the IR into an early
    /// convolution portion (length = `hybridTransitionTime`) and a late
    /// parametric portion; parametric tails cannot beat from
    /// per-frame IR crossfade stacking (the failure mode that prompted
    /// the migration).
    enum class ReflectionType { Convolution, Hybrid, Parametric };

    bool  realtimeReflections = true;  // master enable for convolution reverb
    ReflectionType reflectionType = ReflectionType::Convolution;
    float hybridTransitionTime = 2.0f;  // seconds — convolution portion of IR
    float hybridOverlapPercent = 0.25f; // fraction of transition_time for crossfade

    int   ambisonicsOrder     = 0;      // realtime ambisonic order (0–3)

    // Realtime simulation params (running every reflection_throttle frames)
    int   realtimeNumRays         = 1024;  // rays per realtime sim step (128–8192)
    int   realtimeNumBounces      = 4;     // bounces per ray (1–8)
    float realtimeDuration        = 2.0f;  // IR duration in seconds (0.5–4.0)
    int   realtimeDiffuseSamples  = 32;    // diffuse scattering samples per bounce (16–256)

    // Offline bake params (run once per mission; cached as .probes files).
    int   bakeNumRays             = 4096;  // rays per bake step (1024–65536)
    int   bakeNumBounces          = 8;     // bake bounces (1–64)
    float bakeDuration            = 4.0f;  // bake IR duration in seconds
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
    // Add a 4-probe ring (±0.5 m on the portal plane) around each
    // RoomService portal centroid. Densifies the pathing visibility
    // graph at doorways so cross-room chains route through the doorway
    // rather than wrapping around through grid probes on either side.
    bool audioProbePortalRings = true;

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
    float propMinAttenuation  = 0.001f;   // floor on propagation/portal scale (prevents total silence from FP noise)
    // N-path BFS: how many simultaneous portal-graph paths to keep per
    // listener room. 1 = single shortest path; 2 = original Dark Engine
    // (cBFRoomInfo::previous_room_2); 3+ = modernized. Clamped to [1, 4].
    uint32_t propMaxPaths     = 2;
    // Alternates kept only if their effective distance is within this
    // many world units of the primary. Matches the original engine's
    // kMaxDistDiff = 10. Clamped to [0, 50].
    float    propMaxPathDiff  = 10.0f;
    // Multiplier on the scalar gain produced by Steam Audio's baked
    // pathing eqCoeffs. 1.0 = identity. Use > 1 to make through-portal
    // sound louder than the bake implies, without re-baking. Does NOT
    // affect the LPF blocking factor. Clamped to [0.1, 10.0].
    float    pathingGainScale = 1.0f;

    // -- audio.spatialization: HRTF + distance model --
    float hrtfVolume          = 1.0f;   // HRTF output gain (1.0 = raw HRTF, lower = quieter)
    std::string hrtfInterpolation = "bilinear"; // HRTF interpolation: "nearest" or "bilinear"
    float spatialBlend        = 1.0f;   // binaural blend (0=mono, 1=full HRTF)
    std::string distanceModel = "default"; // distance attenuation: "default" or "inverse_distance"

    // -- audio.ambient: P$AmbientHack tuning --
    float ambHysteresisStartMul = 1.5f; // multiplier on radius for ambient activation distance
    float ambHysteresisStopMul  = 2.0f; // multiplier on radius for ambient deactivation distance
    std::string ambFalloffCurve = "quadratic"; // ambient distance falloff: "linear" or "quadratic"
    int   ambDefaultPriority    = 64;   // priority for ambients without explicit value
    // Per-voice spatialBlend override for AMB_ENVIRONMENTAL ambients (room
    // tone, wind, church reverberance). 1.0 = full HRTF point-source pan;
    // 0.0 = mono passthrough (no directional cue). Object-attached ambients
    // (no AMB_ENVIRONMENTAL flag) keep full HRTF at 1.0 regardless.
    // Default 0.3 = mostly diffuse with a subtle directional hint, so
    // "wind from outside" still leans in the right direction but doesn't
    // feel like a laser pointer.
    float ambEnvironmentalSpatialBlend = 0.3f;  // (0.0–1.0)

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
                cfg.waveAmplitude = water["wave_amplitude"].as<float>();
                if (cfg.waveAmplitude < 0.0f) cfg.waveAmplitude = 0.0f;
                if (cfg.waveAmplitude > 10.0f) cfg.waveAmplitude = 10.0f;
            }
            if (water["uv_distortion"]) {
                cfg.uvDistortion = water["uv_distortion"].as<float>();
                if (cfg.uvDistortion < 0.0f) cfg.uvDistortion = 0.0f;
                if (cfg.uvDistortion > 0.1f) cfg.uvDistortion = 0.1f;
            }
            if (water["rotation_speed"]) {
                cfg.waterRotation = water["rotation_speed"].as<float>();
                if (cfg.waterRotation < 0.0f) cfg.waterRotation = 0.0f;
                if (cfg.waterRotation > 1.0f) cfg.waterRotation = 1.0f;
            }
            if (water["scroll_speed"]) {
                cfg.waterScrollSpeed = water["scroll_speed"].as<float>();
                if (cfg.waterScrollSpeed < 0.0f) cfg.waterScrollSpeed = 0.0f;
                if (cfg.waterScrollSpeed > 1.0f) cfg.waterScrollSpeed = 1.0f;
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
                if (perf["convolution_workers"]) {
                    cfg.convolutionWorkers = perf["convolution_workers"].as<int>();
                    if (cfg.convolutionWorkers < 0)  cfg.convolutionWorkers = 0;
                    if (cfg.convolutionWorkers > 16) cfg.convolutionWorkers = 16;
                }
                if (perf["simulator_threads"]) {
                    cfg.simulatorThreads = perf["simulator_threads"].as<int>();
                    if (cfg.simulatorThreads < 0)  cfg.simulatorThreads = 0;
                    if (cfg.simulatorThreads > 64) cfg.simulatorThreads = 64;
                }
                if (perf["max_active_voices"]) {
                    cfg.maxActiveVoices = perf["max_active_voices"].as<int>();
                    if (cfg.maxActiveVoices < 8)   cfg.maxActiveVoices = 8;
                    if (cfg.maxActiveVoices > 256) cfg.maxActiveVoices = 256;
                }
                if (perf["max_reflection_voices"]) {
                    cfg.maxReflectionVoices = perf["max_reflection_voices"].as<int>();
                    if (cfg.maxReflectionVoices < 1)  cfg.maxReflectionVoices = 1;
                    if (cfg.maxReflectionVoices > 64) cfg.maxReflectionVoices = 64;
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
                if (perf["sim_max_rays"]) {
                    cfg.simMaxRays = perf["sim_max_rays"].as<int>();
                    if (cfg.simMaxRays < 128)   cfg.simMaxRays = 128;
                    if (cfg.simMaxRays > 16384) cfg.simMaxRays = 16384;
                }
                // Legacy alias: `sim_max_sources` maps to the reflection cap
                // so existing configs keep their old reflection-sim behavior.
                // New configs should prefer the split keys below.
                if (perf["sim_max_sources"]) {
                    cfg.reflectionMaxSources = perf["sim_max_sources"].as<int>();
                    if (cfg.reflectionMaxSources < 4)   cfg.reflectionMaxSources = 4;
                    if (cfg.reflectionMaxSources > 256) cfg.reflectionMaxSources = 256;
                }
                if (perf["direct_max_sources"]) {
                    cfg.directMaxSources = perf["direct_max_sources"].as<int>();
                    if (cfg.directMaxSources < 4)    cfg.directMaxSources = 4;
                    if (cfg.directMaxSources > 1024) cfg.directMaxSources = 1024;
                }
                if (perf["reflection_max_sources"]) {
                    cfg.reflectionMaxSources = perf["reflection_max_sources"].as<int>();
                    if (cfg.reflectionMaxSources < 4)   cfg.reflectionMaxSources = 4;
                    if (cfg.reflectionMaxSources > 256) cfg.reflectionMaxSources = 256;
                }
                if (perf["reflection_demote_hysteresis_frames"]) {
                    cfg.reflectionDemoteHysteresisFrames =
                        perf["reflection_demote_hysteresis_frames"].as<int>();
                    if (cfg.reflectionDemoteHysteresisFrames < 1)    cfg.reflectionDemoteHysteresisFrames = 1;
                    if (cfg.reflectionDemoteHysteresisFrames > 3600) cfg.reflectionDemoteHysteresisFrames = 3600;
                }
                if (perf["scene_type"]) {
                    cfg.sceneType = perf["scene_type"].as<std::string>();
                    if (cfg.sceneType != "default" && cfg.sceneType != "embree")
                        cfg.sceneType = "default";
                }
            }

            // -- audio.reflections --
            //
            // Layout (see PLAN.HYBRID_REVERB.md):
            //   reflections.enabled / ambisonics_order   — shared
            //   reflections.type / hybrid_*              — algorithm select
            //   reflections.realtime.{rays,bounces,duration,diffuse_samples}
            //   reflections.bake.{rays,bounces,duration,diffuse_samples,ambisonics_order}
            if (YAML::Node refl = audio["reflections"]) {
                if (refl["enabled"]) cfg.realtimeReflections = refl["enabled"].as<bool>();

                if (refl["ambisonics_order"]) {
                    cfg.ambisonicsOrder = refl["ambisonics_order"].as<int>();
                    if (cfg.ambisonicsOrder < 0) cfg.ambisonicsOrder = 0;
                    if (cfg.ambisonicsOrder > 3) cfg.ambisonicsOrder = 3;
                }

                if (refl["type"]) {
                    std::string val = refl["type"].as<std::string>();
                    if      (val == "hybrid")     cfg.reflectionType = RenderConfig::ReflectionType::Hybrid;
                    else if (val == "parametric") cfg.reflectionType = RenderConfig::ReflectionType::Parametric;
                    else if (val == "convolution") cfg.reflectionType = RenderConfig::ReflectionType::Convolution;
                    else {
                        std::fprintf(stderr,
                            "WARN: unknown reflections.type '%s' — falling back to 'convolution'\n",
                            val.c_str());
                        cfg.reflectionType = RenderConfig::ReflectionType::Convolution;
                    }
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
                if (prb["portal_rings"]) {
                    cfg.audioProbePortalRings = prb["portal_rings"].as<bool>();
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
                    cfg.distanceModel = spat["distance_model"].as<std::string>();
                    if (cfg.distanceModel != "default" && cfg.distanceModel != "inverse_distance")
                        cfg.distanceModel = "default";
                }
            }

            // -- audio.ambient --
            if (YAML::Node amb = audio["ambient"]) {
                if (amb["hysteresis_start_mul"]) {
                    cfg.ambHysteresisStartMul = amb["hysteresis_start_mul"].as<float>();
                    if (cfg.ambHysteresisStartMul < 1.0f) cfg.ambHysteresisStartMul = 1.0f;
                    if (cfg.ambHysteresisStartMul > 5.0f) cfg.ambHysteresisStartMul = 5.0f;
                }
                if (amb["hysteresis_stop_mul"]) {
                    cfg.ambHysteresisStopMul = amb["hysteresis_stop_mul"].as<float>();
                    if (cfg.ambHysteresisStopMul < 1.0f) cfg.ambHysteresisStopMul = 1.0f;
                    if (cfg.ambHysteresisStopMul > 5.0f) cfg.ambHysteresisStopMul = 5.0f;
                }
                if (amb["falloff_curve"]) {
                    cfg.ambFalloffCurve = amb["falloff_curve"].as<std::string>();
                    if (cfg.ambFalloffCurve != "linear" && cfg.ambFalloffCurve != "quadratic")
                        cfg.ambFalloffCurve = "quadratic";
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

// Parse CLI arguments. The CLI surface is intentionally minimal — every
// other tunable lives in the YAML config. The flags below are the things
// that genuinely need per-invocation override:
//   <mission.mis>      first non-flag arg, the mission to load
//   --res <path>       runtime asset directory (overrides paths.res)
//   --schemas <path>   schema directory (overrides paths.schemas)
//   --config <path>    YAML path (defaults to ./darknessRender.yaml)
//   --help / -h        print usage
//
// Unknown flags are reported but otherwise ignored — when a removed flag
// shows up in old shell history, the user gets a clear message instead
// of a silent parse miss.
inline CliResult applyCliOverrides(int argc, char* argv[], RenderConfig& cfg) {
    CliResult cli;
    (void)cfg; // currently nothing is set on cfg via CLI

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            cli.helpRequested = true;
        } else if (std::strcmp(argv[i], "--res") == 0 && i + 1 < argc) {
            cli.resPath = argv[++i];
        } else if (std::strcmp(argv[i], "--schemas") == 0 && i + 1 < argc) {
            cli.schemasPath = argv[++i];
        } else if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            cli.configPath = argv[++i];
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
