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

    // -- physics --
    int physicsRate = 60;  // physics timestep Hz: 12 = vintage (12.5Hz), 60 = modern, 120 = ultra

    // -- developer --
    bool showObjects      = true;   // render object meshes
    bool showFallbackCubes = false; // show colored cubes for objects with missing models
    bool portalCulling    = true;   // portal/frustum culling
    bool forceFlicker     = false;  // force all animated lights to flicker mode
    bool cameraCollision  = false;  // sphere collision against world geometry
    bool debugObjects     = false;  // dump per-object filtering diagnostics to stderr
    bool stepLog          = false;  // stair step diagnostics to stderr ([STEP] prefix)
};

// Result of CLI parsing — values that are CLI-only (not in YAML).
struct CliResult {
    const char* misPath    = nullptr;  // positional arg: mission file
    std::string resPath;               // --res <path>
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
            if (dev["force_flicker"])       cfg.forceFlicker      = dev["force_flicker"].as<bool>();
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
        } else if (std::strcmp(argv[i], "--force-flicker") == 0) {
            cfg.forceFlicker = true;
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
