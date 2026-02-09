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
    int  lmScale    = 1;      // lightmap upscale factor (1-8)
    int  filterMode = 0;      // texture filtering: 0=point, 1=bilinear, 2=trilinear, 3=aniso
    bool linearMips = false;  // gamma-correct mipmap generation
    bool sharpMips  = false;  // unsharp mask on mip levels

    // -- developer --
    bool showObjects   = true;   // render object meshes
    bool portalCulling = true;   // portal/frustum culling
    bool forceFlicker  = false;  // force all animated lights to flicker mode
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
            if (gfx["lm_scale"]) {
                cfg.lmScale = gfx["lm_scale"].as<int>();
                if (cfg.lmScale < 1) cfg.lmScale = 1;
                if (cfg.lmScale > 8) cfg.lmScale = 8;
            }
            if (gfx["filter_mode"])  cfg.filterMode = gfx["filter_mode"].as<int>();
            if (gfx["linear_mips"])  cfg.linearMips = gfx["linear_mips"].as<bool>();
            if (gfx["sharp_mips"])   cfg.sharpMips  = gfx["sharp_mips"].as<bool>();
        }

        // developer section
        if (YAML::Node dev = root["developer"]) {
            if (dev["show_objects"])   cfg.showObjects   = dev["show_objects"].as<bool>();
            if (dev["portal_culling"]) cfg.portalCulling  = dev["portal_culling"].as<bool>();
            if (dev["force_flicker"])  cfg.forceFlicker   = dev["force_flicker"].as<bool>();
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
        } else if (std::strcmp(argv[i], "--lm-scale") == 0 && i + 1 < argc) {
            cfg.lmScale = std::atoi(argv[++i]);
            if (cfg.lmScale < 1) cfg.lmScale = 1;
            if (cfg.lmScale > 8) cfg.lmScale = 8;
        } else if (std::strcmp(argv[i], "--no-objects") == 0) {
            cfg.showObjects = false;
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
        } else if (argv[i][0] != '-' && !cli.misPath) {
            // First non-flag argument is the mission file
            cli.misPath = argv[i];
        }
    }

    return cli;
}

} // namespace Darkness
