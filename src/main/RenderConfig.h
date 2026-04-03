// RenderConfig.h — YAML + CLI configuration for darknessRender
// Config precedence: CLI flags > YAML config file > hardcoded defaults
#pragma once

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <yaml-cpp/yaml.h>

namespace Darkness {

// All configurable settings for the renderer.
// Defaults match the original hardcoded values.
struct RenderConfig {
    // -- graphics --
    int  lightmapFiltering = 0; // lightmap filtering: 0=bilinear (default), 1=bicubic
    int  filterMode = 0;        // texture filtering: 0=point, 1=bilinear, 2=trilinear, 3=aniso
    bool linearMips = false;  // gamma-correct mipmap generation
    bool sharpMips  = false;  // unsharp mask on mip levels

    // -- water --
    float waveAmplitude   = 0.3f;    // vertex Z displacement in world units (0 = flat)
    float uvDistortion    = 0.015f;  // UV wobble strength (0 = no ripple)
    float waterRotation   = 0.015f;  // UV rotation speed in rad/s (0 = no rotation)
    float waterScrollSpeed = 0.05f;  // UV scroll speed in world units/s (0 = no drift)

    // -- audio --
    int  reflectionRateDivisor = 2;    // 1=full (48kHz), 2=half (24kHz), 4=quarter (12kHz)
    int  convolutionWorkers = 0;       // parallel convolution worker threads (0=auto)
    int  maxReflectionVoices = 8;      // max ACTIVE voices with reflection convolution (tail voices are free)
    int  ambisonicsOrder = 0;          // 0 = omnidirectional reverb (1ch, 4x cheaper), 1 = directional (4ch)
    int  reflectionNumRays   = 1024;   // rays per reflection sim step (128–8192)
    int  reflectionNumBounces = 4;     // bounces per ray (1–8)
    float reflectionDuration = 2.0f;   // max reverb tail in seconds (0.5–4.0)
    int  reflectionThrottle  = 4;      // run reflection sim every Nth frame (1–32)
    float transmissionScale  = 10.0f;  // multiply all material transmission coefficients (1=physical, 10=audible through walls)
    float absorptionScale    = 1.0f;   // multiply all material absorption coefficients (1=physical, 0.5=more reflective)
    int   diffuseSamples     = 64;     // diffuse scattering samples for real-time reflection sim (16-256)
    int   bakeDiffuseSamples = 128;    // diffuse scattering samples for probe baking (32-512, higher=smoother)
    float occlusionRadius    = 3.0f;   // volumetric occlusion source sphere radius (world units, 0.1-10)
    int   occlusionSamples   = 16;     // volumetric occlusion ray samples per source (4-64)

    // Propagation layer toggles (all on by default — debug use only)
    bool portalRouting       = true;   // portal-graph sound routing through doorways
    bool probePathing        = true;   // baked probe diffraction/pathing (when available)
    bool realtimeReflections = true;   // Steam Audio real-time convolution reverb

    // -- audio DSP chain --
    bool  dspLimiter    = true;    // soft tanh limiter (prevents digital clipping)
    float dspLimiterKnee = 0.8f;   // knee threshold (0.5–0.95, higher = later onset)
    bool  dspCompressor = true;    // master bus compressor (tames transients)
    float dspCompThreshold = -15.0f; // compressor threshold in dBFS (-30 to 0)
    float dspCompRatio = 3.0f;     // compression ratio (1.5 to 10)
    bool  dspEQ         = true;    // low-shelf EQ (adds bass weight)
    float dspEQFreq     = 120.0f;  // EQ center frequency in Hz (60–500)
    float dspEQGain     = 3.0f;    // EQ gain in dB (-6 to +6)
    bool  dspDucking    = false;   // ambient ducking when SFX plays (disabled by default)
    float dspDuckAmount = 0.5f;    // ambient volume during ducking (0.1–1.0)


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

        // graphics section
        if (YAML::Node gfx = root["graphics"]) {
            if (gfx["lightmap_filtering"]) {
                std::string val = gfx["lightmap_filtering"].as<std::string>();
                if (val == "bicubic") cfg.lightmapFiltering = 1;
                else cfg.lightmapFiltering = 0;  // "bilinear" or unknown → default
            }
            if (gfx["filter_mode"])  cfg.filterMode = gfx["filter_mode"].as<int>();
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

        // audio section
        if (YAML::Node audio = root["audio"]) {
            if (audio["half_rate_reflections"]) {
                // Backward-compatible: bool → divisor 2 or 1
                cfg.reflectionRateDivisor = audio["half_rate_reflections"].as<bool>() ? 2 : 1;
            }
            if (audio["reflection_rate_divisor"]) {
                int div = audio["reflection_rate_divisor"].as<int>();
                cfg.reflectionRateDivisor = (div >= 4) ? 4 : (div >= 2) ? 2 : 1;
            }
            if (audio["convolution_workers"]) {
                cfg.convolutionWorkers = audio["convolution_workers"].as<int>();
                if (cfg.convolutionWorkers < 0) cfg.convolutionWorkers = 0;
                if (cfg.convolutionWorkers > 16) cfg.convolutionWorkers = 16;
            }
            if (audio["ambisonics_order"]) {
                cfg.ambisonicsOrder = audio["ambisonics_order"].as<int>();
                if (cfg.ambisonicsOrder < 0) cfg.ambisonicsOrder = 0;
                if (cfg.ambisonicsOrder > 1) cfg.ambisonicsOrder = 1;
            }
            if (audio["max_reflection_voices"]) {
                cfg.maxReflectionVoices = audio["max_reflection_voices"].as<int>();
                if (cfg.maxReflectionVoices < 1) cfg.maxReflectionVoices = 1;
                if (cfg.maxReflectionVoices > 64) cfg.maxReflectionVoices = 64;
            }
            if (audio["reflection_rays"]) {
                cfg.reflectionNumRays = audio["reflection_rays"].as<int>();
                if (cfg.reflectionNumRays < 128) cfg.reflectionNumRays = 128;
                if (cfg.reflectionNumRays > 8192) cfg.reflectionNumRays = 8192;
            }
            if (audio["reflection_bounces"]) {
                cfg.reflectionNumBounces = audio["reflection_bounces"].as<int>();
                if (cfg.reflectionNumBounces < 1) cfg.reflectionNumBounces = 1;
                if (cfg.reflectionNumBounces > 8) cfg.reflectionNumBounces = 8;
            }
            if (audio["reflection_duration"]) {
                cfg.reflectionDuration = audio["reflection_duration"].as<float>();
                if (cfg.reflectionDuration < 0.5f) cfg.reflectionDuration = 0.5f;
                if (cfg.reflectionDuration > 4.0f) cfg.reflectionDuration = 4.0f;
            }
            if (audio["reflection_throttle"]) {
                cfg.reflectionThrottle = audio["reflection_throttle"].as<int>();
                if (cfg.reflectionThrottle < 1) cfg.reflectionThrottle = 1;
                if (cfg.reflectionThrottle > 32) cfg.reflectionThrottle = 32;
            }
            if (audio["transmission_scale"]) {
                cfg.transmissionScale = audio["transmission_scale"].as<float>();
                if (cfg.transmissionScale < 0.1f) cfg.transmissionScale = 0.1f;
                if (cfg.transmissionScale > 100.0f) cfg.transmissionScale = 100.0f;
            }
            if (audio["absorption_scale"]) {
                cfg.absorptionScale = audio["absorption_scale"].as<float>();
                if (cfg.absorptionScale < 0.01f) cfg.absorptionScale = 0.01f;
                if (cfg.absorptionScale > 10.0f) cfg.absorptionScale = 10.0f;
            }
            if (audio["diffuse_samples"]) {
                cfg.diffuseSamples = audio["diffuse_samples"].as<int>();
                if (cfg.diffuseSamples < 16) cfg.diffuseSamples = 16;
                if (cfg.diffuseSamples > 256) cfg.diffuseSamples = 256;
            }
            if (audio["bake_diffuse_samples"]) {
                cfg.bakeDiffuseSamples = audio["bake_diffuse_samples"].as<int>();
                if (cfg.bakeDiffuseSamples < 32) cfg.bakeDiffuseSamples = 32;
                if (cfg.bakeDiffuseSamples > 512) cfg.bakeDiffuseSamples = 512;
            }
            if (audio["occlusion_radius"]) {
                cfg.occlusionRadius = audio["occlusion_radius"].as<float>();
                if (cfg.occlusionRadius < 0.1f) cfg.occlusionRadius = 0.1f;
                if (cfg.occlusionRadius > 10.0f) cfg.occlusionRadius = 10.0f;
            }
            if (audio["occlusion_samples"]) {
                cfg.occlusionSamples = audio["occlusion_samples"].as<int>();
                if (cfg.occlusionSamples < 4) cfg.occlusionSamples = 4;
                if (cfg.occlusionSamples > 64) cfg.occlusionSamples = 64;
            }
            if (audio["portal_routing"])
                cfg.portalRouting = audio["portal_routing"].as<bool>();
            if (audio["probe_pathing"])
                cfg.probePathing = audio["probe_pathing"].as<bool>();
            if (audio["realtime_reflections"])
                cfg.realtimeReflections = audio["realtime_reflections"].as<bool>();

            // DSP chain settings
            if (audio["dsp_limiter"])
                cfg.dspLimiter = audio["dsp_limiter"].as<bool>();
            if (audio["dsp_limiter_knee"]) {
                cfg.dspLimiterKnee = audio["dsp_limiter_knee"].as<float>();
                if (cfg.dspLimiterKnee < 0.5f) cfg.dspLimiterKnee = 0.5f;
                if (cfg.dspLimiterKnee > 0.95f) cfg.dspLimiterKnee = 0.95f;
            }
            if (audio["dsp_compressor"])
                cfg.dspCompressor = audio["dsp_compressor"].as<bool>();
            if (audio["dsp_comp_threshold"]) {
                cfg.dspCompThreshold = audio["dsp_comp_threshold"].as<float>();
                if (cfg.dspCompThreshold < -30.0f) cfg.dspCompThreshold = -30.0f;
                if (cfg.dspCompThreshold > 0.0f) cfg.dspCompThreshold = 0.0f;
            }
            if (audio["dsp_comp_ratio"]) {
                cfg.dspCompRatio = audio["dsp_comp_ratio"].as<float>();
                if (cfg.dspCompRatio < 1.5f) cfg.dspCompRatio = 1.5f;
                if (cfg.dspCompRatio > 10.0f) cfg.dspCompRatio = 10.0f;
            }
            if (audio["dsp_eq"])
                cfg.dspEQ = audio["dsp_eq"].as<bool>();
            if (audio["dsp_eq_freq"]) {
                cfg.dspEQFreq = audio["dsp_eq_freq"].as<float>();
                if (cfg.dspEQFreq < 60.0f) cfg.dspEQFreq = 60.0f;
                if (cfg.dspEQFreq > 500.0f) cfg.dspEQFreq = 500.0f;
            }
            if (audio["dsp_eq_gain"]) {
                cfg.dspEQGain = audio["dsp_eq_gain"].as<float>();
                if (cfg.dspEQGain < -6.0f) cfg.dspEQGain = -6.0f;
                if (cfg.dspEQGain > 6.0f) cfg.dspEQGain = 6.0f;
            }
            if (audio["dsp_ducking"])
                cfg.dspDucking = audio["dsp_ducking"].as<bool>();
            if (audio["dsp_duck_amount"]) {
                cfg.dspDuckAmount = audio["dsp_duck_amount"].as<float>();
                if (cfg.dspDuckAmount < 0.1f) cfg.dspDuckAmount = 0.1f;
                if (cfg.dspDuckAmount > 1.0f) cfg.dspDuckAmount = 1.0f;
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
            if (dev["show_objects"])        cfg.showObjects      = dev["show_objects"].as<bool>();
            if (dev["show_fallback_cubes"]) cfg.showFallbackCubes = dev["show_fallback_cubes"].as<bool>();
            if (dev["portal_culling"])      cfg.portalCulling     = dev["portal_culling"].as<bool>();
            if (dev["camera_collision"])    cfg.cameraCollision   = dev["camera_collision"].as<bool>();
            if (dev["step_log"])            cfg.stepLog           = dev["step_log"].as<bool>();
        }

        std::fprintf(stderr, "Loaded config from %s\n", path.c_str());
        return true;
    } catch (const YAML::Exception& e) {
        std::fprintf(stderr, "Warning: failed to parse config %s: %s\n",
                     path.c_str(), e.what());
        return false;
    }
}

// Parse CLI arguments into the config struct and extract CLI-only values.
// Processes all flags in a single pass using else-if chain.
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
        } else if (std::strcmp(argv[i], "--lightmap-filtering") == 0 && i + 1 < argc) {
            const char *val = argv[++i];
            if (std::strcmp(val, "bicubic") == 0) cfg.lightmapFiltering = 1;
            else cfg.lightmapFiltering = 0;  // "bilinear" or unknown → default
        } else if (std::strcmp(argv[i], "--no-objects") == 0) {
            cfg.showObjects = false;
        } else if (std::strcmp(argv[i], "--show-fallback") == 0) {
            cfg.showFallbackCubes = true;
        } else if (std::strcmp(argv[i], "--no-cull") == 0) {
            cfg.portalCulling = false;
        } else if (std::strcmp(argv[i], "--filter") == 0) {
            cfg.filterMode = 1;  // bilinear
        } else if (std::strcmp(argv[i], "--linear-mips") == 0) {
            cfg.linearMips = true;
        } else if (std::strcmp(argv[i], "--sharp-mips") == 0) {
            cfg.sharpMips = true;
        } else if (std::strcmp(argv[i], "--collision") == 0) {
            cfg.cameraCollision = true;
        } else if (std::strcmp(argv[i], "--debug-objects") == 0) {
            cfg.debugObjects = true;
        } else if (std::strcmp(argv[i], "--step-log") == 0) {
            cfg.stepLog = true;
        } else if (std::strcmp(argv[i], "--toggle-platforms") == 0) {
            cfg.togglePlatforms = true;
        } else if (std::strcmp(argv[i], "--physics-rate") == 0 && i + 1 < argc) {
            int val = std::atoi(argv[++i]);
            if (val <= 12) cfg.physicsRate = 12;
            else if (val >= 120) cfg.physicsRate = 120;
            else cfg.physicsRate = 60;
        } else if (std::strcmp(argv[i], "--wave-amp") == 0 && i + 1 < argc) {
            cfg.waveAmplitude = static_cast<float>(std::atof(argv[++i]));
            if (cfg.waveAmplitude < 0.0f) cfg.waveAmplitude = 0.0f;
            if (cfg.waveAmplitude > 10.0f) cfg.waveAmplitude = 10.0f;
        } else if (std::strcmp(argv[i], "--uv-distort") == 0 && i + 1 < argc) {
            cfg.uvDistortion = static_cast<float>(std::atof(argv[++i]));
            if (cfg.uvDistortion < 0.0f) cfg.uvDistortion = 0.0f;
            if (cfg.uvDistortion > 0.1f) cfg.uvDistortion = 0.1f;
        } else if (std::strcmp(argv[i], "--water-rot") == 0 && i + 1 < argc) {
            cfg.waterRotation = static_cast<float>(std::atof(argv[++i]));
            if (cfg.waterRotation < 0.0f) cfg.waterRotation = 0.0f;
            if (cfg.waterRotation > 1.0f) cfg.waterRotation = 1.0f;
        } else if (std::strcmp(argv[i], "--water-scroll") == 0 && i + 1 < argc) {
            cfg.waterScrollSpeed = static_cast<float>(std::atof(argv[++i]));
            if (cfg.waterScrollSpeed < 0.0f) cfg.waterScrollSpeed = 0.0f;
            if (cfg.waterScrollSpeed > 1.0f) cfg.waterScrollSpeed = 1.0f;
        } else if (argv[i][0] != '-' && !cli.misPath) {
            // First non-flag argument is the mission file
            cli.misPath = argv[i];
        }
    }

    return cli;
}

} // namespace Darkness
