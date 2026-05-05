// Unit tests for RenderConfig (YAML + CLI configuration).
// The CLI surface is intentionally minimal; almost all tunables live in YAML.
#include <catch2/catch_test_macros.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "RenderConfig.h"

namespace fs = std::filesystem;

// Helper: write a temporary YAML file and return its path.
// The file is deleted when the returned guard goes out of scope.
struct TmpFile {
    fs::path path;
    explicit TmpFile(const std::string& content) {
        path = fs::temp_directory_path() / ("darkness_test_" + std::to_string(reinterpret_cast<uintptr_t>(this)) + ".yaml");
        std::ofstream out(path);
        out << content;
    }
    ~TmpFile() { std::error_code ec; fs::remove(path, ec); }
    TmpFile(const TmpFile&) = delete;
    TmpFile& operator=(const TmpFile&) = delete;
};

// Helper: build argc/argv from a vector of strings.
// Returns owning vector of char* pointers valid for the lifetime of `args`.
static std::vector<char*> makeArgv(std::vector<std::string>& args) {
    std::vector<char*> ptrs;
    for (auto& s : args) ptrs.push_back(s.data());
    return ptrs;
}

// ---- Test cases ----

TEST_CASE("RenderConfig defaults", "[config]") {
    Darkness::RenderConfig cfg;

    CHECK(cfg.lightmapFiltering == 0);
    CHECK(cfg.filterMode == 0);
    CHECK(cfg.linearMips == false);
    CHECK(cfg.sharpMips  == false);
    CHECK(cfg.resPath.empty());
    CHECK(cfg.schemasPath.empty());
    CHECK(cfg.waveAmplitude == 0.3f);
    CHECK(cfg.uvDistortion  == 0.015f);
    CHECK(cfg.waterRotation == 0.015f);
    CHECK(cfg.waterScrollSpeed == 0.05f);
    CHECK(cfg.showObjects      == true);
    CHECK(cfg.showFallbackCubes == false);
    CHECK(cfg.portalCulling    == true);
    CHECK(cfg.cameraCollision  == false);
    CHECK(cfg.debugObjects     == false);
    CHECK(cfg.togglePlatforms  == false);
    CHECK(cfg.noProbes         == false);
    CHECK(cfg.audioLog         == false);
}

TEST_CASE("YAML full load — graphics + developer", "[config][yaml]") {
    TmpFile tmp(R"(
graphics:
  lightmap_filter: bicubic
  texture_filter: trilinear
  linear_mips: true
  sharp_mips: true
developer:
  show_objects: false
  show_fallback_cubes: true
  portal_culling: false
  camera_collision: true
  debug_objects: true
  toggle_platforms: true
  no_probes: true
  audio_log: true
)");

    Darkness::RenderConfig cfg;
    bool ok = Darkness::loadConfigFromYAML(tmp.path.string(), cfg);

    REQUIRE(ok);
    CHECK(cfg.lightmapFiltering == 1);  // bicubic
    CHECK(cfg.filterMode        == 2);  // trilinear
    CHECK(cfg.linearMips        == true);
    CHECK(cfg.sharpMips         == true);
    CHECK(cfg.showObjects       == false);
    CHECK(cfg.showFallbackCubes == true);
    CHECK(cfg.portalCulling     == false);
    CHECK(cfg.cameraCollision   == true);
    CHECK(cfg.debugObjects      == true);
    CHECK(cfg.togglePlatforms   == true);
    CHECK(cfg.noProbes          == true);
    CHECK(cfg.audioLog          == true);
}

TEST_CASE("YAML partial load — unset fields keep defaults", "[config][yaml]") {
    TmpFile tmp(R"(
graphics:
  lightmap_filter: bicubic
)");

    Darkness::RenderConfig cfg;
    bool ok = Darkness::loadConfigFromYAML(tmp.path.string(), cfg);

    REQUIRE(ok);
    CHECK(cfg.lightmapFiltering == 1);
    // Everything else should be default
    CHECK(cfg.filterMode == 0);
    CHECK(cfg.linearMips == false);
    CHECK(cfg.sharpMips  == false);
    CHECK(cfg.showObjects   == true);
    CHECK(cfg.portalCulling == true);
}

TEST_CASE("YAML missing file returns false, config unchanged", "[config][yaml]") {
    Darkness::RenderConfig cfg;
    Darkness::RenderConfig orig = cfg;

    bool ok = Darkness::loadConfigFromYAML("/tmp/nonexistent_darkness_config_xyz.yaml", cfg);

    CHECK_FALSE(ok);
    // Config must be unchanged
    CHECK(cfg.lightmapFiltering == orig.lightmapFiltering);
    CHECK(cfg.filterMode        == orig.filterMode);
    CHECK(cfg.showObjects       == orig.showObjects);
}

TEST_CASE("YAML malformed file returns false, no crash", "[config][yaml]") {
    TmpFile tmp("{{{{not valid yaml at all : : :");

    Darkness::RenderConfig cfg;
    Darkness::RenderConfig orig = cfg;

    bool ok = Darkness::loadConfigFromYAML(tmp.path.string(), cfg);

    CHECK_FALSE(ok);
    // Config must be unchanged — malformed YAML should not partially apply
    CHECK(cfg.lightmapFiltering == orig.lightmapFiltering);
    CHECK(cfg.filterMode        == orig.filterMode);
    CHECK(cfg.showObjects       == orig.showObjects);
}

TEST_CASE("CLI surface: --res, --schemas, --config, --help, positional", "[config][cli]") {
    Darkness::RenderConfig cfg;
    std::vector<std::string> args = {
        "darknessRender", "test.mis",
        "--res", "/path/to/res",
        "--schemas", "/path/to/schemas",
        "--config", "my.yaml",
        "--help"
    };
    auto argv = makeArgv(args);
    int argc = static_cast<int>(argv.size());

    Darkness::CliResult cli = Darkness::applyCliOverrides(argc, argv.data(), cfg);

    CHECK(cli.helpRequested == true);
    CHECK(cli.resPath     == "/path/to/res");
    CHECK(cli.schemasPath == "/path/to/schemas");
    CHECK(cli.configPath  == "my.yaml");
    REQUIRE(cli.misPath != nullptr);
    CHECK(std::string(cli.misPath) == "test.mis");
}

TEST_CASE("CLI ignores removed flags with a warning", "[config][cli]") {
    // Old flags from before the YAML-only refactor. They should be silently
    // ignored (with a stderr warning) rather than mutating cfg.
    Darkness::RenderConfig cfg;
    std::vector<std::string> args = {
        "darknessRender", "mission.mis",
        "--no-objects", "--no-cull", "--filter",
        "--lightmap-filtering", "bicubic",
        "--linear-mips", "--collision",
        "--physics-rate", "120",
        "--wave-amp", "5.0",
    };
    auto argv = makeArgv(args);
    int argc = static_cast<int>(argv.size());

    Darkness::applyCliOverrides(argc, argv.data(), cfg);

    // None of the removed CLI flags should mutate cfg.
    CHECK(cfg.showObjects       == true);   // default
    CHECK(cfg.portalCulling     == true);   // default
    CHECK(cfg.filterMode        == 0);      // default
    CHECK(cfg.lightmapFiltering == 0);      // default
    CHECK(cfg.linearMips        == false);  // default
    CHECK(cfg.cameraCollision   == false);  // default
    CHECK(cfg.physicsRate       == 60);     // default
    CHECK(cfg.waveAmplitude     == 0.3f);   // default
}

TEST_CASE("texture_filter string parsing", "[config][yaml]") {
    SECTION("point → 0") {
        TmpFile tmp("graphics:\n  texture_filter: point\n");
        Darkness::RenderConfig cfg;
        Darkness::loadConfigFromYAML(tmp.path.string(), cfg);
        CHECK(cfg.filterMode == 0);
    }
    SECTION("bilinear → 1") {
        TmpFile tmp("graphics:\n  texture_filter: bilinear\n");
        Darkness::RenderConfig cfg;
        Darkness::loadConfigFromYAML(tmp.path.string(), cfg);
        CHECK(cfg.filterMode == 1);
    }
    SECTION("trilinear → 2") {
        TmpFile tmp("graphics:\n  texture_filter: trilinear\n");
        Darkness::RenderConfig cfg;
        Darkness::loadConfigFromYAML(tmp.path.string(), cfg);
        CHECK(cfg.filterMode == 2);
    }
    SECTION("anisotropic → 3") {
        TmpFile tmp("graphics:\n  texture_filter: anisotropic\n");
        Darkness::RenderConfig cfg;
        Darkness::loadConfigFromYAML(tmp.path.string(), cfg);
        CHECK(cfg.filterMode == 3);
    }
    SECTION("unknown defaults to 0 (point)") {
        TmpFile tmp("graphics:\n  texture_filter: foo\n");
        Darkness::RenderConfig cfg;
        Darkness::loadConfigFromYAML(tmp.path.string(), cfg);
        CHECK(cfg.filterMode == 0);
    }
}

TEST_CASE("lightmap_filter string parsing", "[config][yaml]") {
    SECTION("bilinear → 0") {
        TmpFile tmp("graphics:\n  lightmap_filter: bilinear\n");
        Darkness::RenderConfig cfg;
        Darkness::loadConfigFromYAML(tmp.path.string(), cfg);
        CHECK(cfg.lightmapFiltering == 0);
    }
    SECTION("bicubic → 1") {
        TmpFile tmp("graphics:\n  lightmap_filter: bicubic\n");
        Darkness::RenderConfig cfg;
        Darkness::loadConfigFromYAML(tmp.path.string(), cfg);
        CHECK(cfg.lightmapFiltering == 1);
    }
    SECTION("unknown defaults to 0 (bilinear)") {
        TmpFile tmp("graphics:\n  lightmap_filter: foobar\n");
        Darkness::RenderConfig cfg;
        Darkness::loadConfigFromYAML(tmp.path.string(), cfg);
        CHECK(cfg.lightmapFiltering == 0);
    }
}

TEST_CASE("paths section: res / schemas", "[config][yaml]") {
    SECTION("YAML populates resPath and schemasPath") {
        TmpFile tmp(R"(
paths:
  res: /Volumes/THIEF2/RES
  schemas: /Volumes/THIEF2_CD2/EDITOR/SCHEMA
)");
        Darkness::RenderConfig cfg;
        Darkness::loadConfigFromYAML(tmp.path.string(), cfg);
        CHECK(cfg.resPath     == "/Volumes/THIEF2/RES");
        CHECK(cfg.schemasPath == "/Volumes/THIEF2_CD2/EDITOR/SCHEMA");
    }
    SECTION("missing paths section leaves both empty") {
        TmpFile tmp("graphics:\n  texture_filter: point\n");
        Darkness::RenderConfig cfg;
        Darkness::loadConfigFromYAML(tmp.path.string(), cfg);
        CHECK(cfg.resPath.empty());
        CHECK(cfg.schemasPath.empty());
    }
    SECTION("partial paths section: only res set") {
        TmpFile tmp("paths:\n  res: /tmp/RES\n");
        Darkness::RenderConfig cfg;
        Darkness::loadConfigFromYAML(tmp.path.string(), cfg);
        CHECK(cfg.resPath == "/tmp/RES");
        CHECK(cfg.schemasPath.empty());
    }
}

TEST_CASE("YAML water section", "[config][yaml]") {
    TmpFile tmp(R"(
water:
  wave_amplitude: 3.5
  uv_distortion: 0.05
  rotation_speed: 0.2
  scroll_speed: 0.08
)");

    Darkness::RenderConfig cfg;
    bool ok = Darkness::loadConfigFromYAML(tmp.path.string(), cfg);

    REQUIRE(ok);
    CHECK(cfg.waveAmplitude == 3.5f);
    CHECK(cfg.uvDistortion  == 0.05f);
    CHECK(cfg.waterRotation == 0.2f);
    CHECK(cfg.waterScrollSpeed == 0.08f);
}

TEST_CASE("water YAML clamping", "[config][clamp]") {
    SECTION("YAML clamps wave_amplitude below 0 to 0") {
        TmpFile tmp("water:\n  wave_amplitude: -2.0\n");
        Darkness::RenderConfig cfg;
        Darkness::loadConfigFromYAML(tmp.path.string(), cfg);
        CHECK(cfg.waveAmplitude == 0.0f);
    }
    SECTION("YAML clamps wave_amplitude above 10 to 10") {
        TmpFile tmp("water:\n  wave_amplitude: 50.0\n");
        Darkness::RenderConfig cfg;
        Darkness::loadConfigFromYAML(tmp.path.string(), cfg);
        CHECK(cfg.waveAmplitude == 10.0f);
    }
    SECTION("YAML clamps uv_distortion below 0 to 0") {
        TmpFile tmp("water:\n  uv_distortion: -1.0\n");
        Darkness::RenderConfig cfg;
        Darkness::loadConfigFromYAML(tmp.path.string(), cfg);
        CHECK(cfg.uvDistortion == 0.0f);
    }
    SECTION("YAML clamps uv_distortion above 0.1 to 0.1") {
        TmpFile tmp("water:\n  uv_distortion: 5.0\n");
        Darkness::RenderConfig cfg;
        Darkness::loadConfigFromYAML(tmp.path.string(), cfg);
        CHECK(cfg.uvDistortion == 0.1f);
    }
    SECTION("YAML clamps rotation_speed above 1.0 to 1.0") {
        TmpFile tmp("water:\n  rotation_speed: 5.0\n");
        Darkness::RenderConfig cfg;
        Darkness::loadConfigFromYAML(tmp.path.string(), cfg);
        CHECK(cfg.waterRotation == 1.0f);
    }
    SECTION("YAML clamps scroll_speed above 1.0 to 1.0") {
        TmpFile tmp("water:\n  scroll_speed: 5.0\n");
        Darkness::RenderConfig cfg;
        Darkness::loadConfigFromYAML(tmp.path.string(), cfg);
        CHECK(cfg.waterScrollSpeed == 1.0f);
    }
}
