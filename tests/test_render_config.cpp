// Unit tests for RenderConfig (YAML + CLI configuration).
// The CLI surface is intentionally minimal; almost all tunables live in YAML.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

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

// Reverb voice caps (renamed from max_reflection_voices /
// max_realtime_voices in 2026-05 config cleanup). reverb_voices is the
// total convolution-pool budget; reverb_voices_realtime is the subset
// that runs realtime ray-traced IRs (0 = baked-only).
TEST_CASE("audio reverb voice cap YAML keys", "[config][yaml][audio]") {
    SECTION("defaults: reverb_voices=16, reverb_voices_realtime=0") {
        Darkness::RenderConfig cfg;
        CHECK(cfg.reverbVoices         == 16);
        CHECK(cfg.reverbVoicesRealtime == 0);
    }
    SECTION("explicit values override defaults") {
        TmpFile tmp(R"(
audio:
  performance:
    reverb_voices: 8
    reverb_voices_realtime: 4
)");
        Darkness::RenderConfig cfg;
        REQUIRE(Darkness::loadConfigFromYAML(tmp.path.string(), cfg));
        CHECK(cfg.reverbVoices         == 8);
        CHECK(cfg.reverbVoicesRealtime == 4);
    }
    SECTION("clamping: reverb_voices < 0 → 0, > 64 → 64") {
        TmpFile lo("audio:\n  performance:\n    reverb_voices: -5\n");
        Darkness::RenderConfig cfg1;
        Darkness::loadConfigFromYAML(lo.path.string(), cfg1);
        CHECK(cfg1.reverbVoices == 0);

        TmpFile hi("audio:\n  performance:\n    reverb_voices: 999\n");
        Darkness::RenderConfig cfg2;
        Darkness::loadConfigFromYAML(hi.path.string(), cfg2);
        CHECK(cfg2.reverbVoices == 64);
    }
    SECTION("clamping: reverb_voices_realtime same as total") {
        TmpFile hi("audio:\n  performance:\n    reverb_voices_realtime: 999\n");
        Darkness::RenderConfig cfg;
        Darkness::loadConfigFromYAML(hi.path.string(), cfg);
        CHECK(cfg.reverbVoicesRealtime == 64);
    }
}

// Pathing probe layout density (PR B, probe layout tiers). Valid values
// are "baseline" and "bends" ONLY; anything else — including the
// reserved-for-Tier-2 "high" — is rejected at parse with a loud stderr
// message and the default kept.
TEST_CASE("audio pathing_probes density YAML key", "[config][yaml][audio]") {
    SECTION("default is baseline") {
        // Baseline became the default 2026-07-11 (matrix2: beats bends
        // on both bake time and runtime path-solve cost).
        Darkness::RenderConfig cfg;
        CHECK(cfg.audioPathingDensity == "baseline");
    }
    SECTION("baseline accepted") {
        TmpFile tmp("audio:\n  pathing_probes:\n    density: baseline\n");
        Darkness::RenderConfig cfg;
        REQUIRE(Darkness::loadConfigFromYAML(tmp.path.string(), cfg));
        CHECK(cfg.audioPathingDensity == "baseline");
    }
    SECTION("bends accepted") {
        TmpFile tmp("audio:\n  pathing_probes:\n    density: bends\n");
        Darkness::RenderConfig cfg;
        REQUIRE(Darkness::loadConfigFromYAML(tmp.path.string(), cfg));
        CHECK(cfg.audioPathingDensity == "bends");
    }
    SECTION("'high' is reserved — rejected, default kept") {
        TmpFile tmp("audio:\n  pathing_probes:\n    density: high\n");
        Darkness::RenderConfig cfg;
        REQUIRE(Darkness::loadConfigFromYAML(tmp.path.string(), cfg));
        CHECK(cfg.audioPathingDensity == "baseline");
    }
    SECTION("garbage rejected, default kept") {
        TmpFile tmp("audio:\n  pathing_probes:\n    density: ultra\n");
        Darkness::RenderConfig cfg;
        REQUIRE(Darkness::loadConfigFromYAML(tmp.path.string(), cfg));
        CHECK(cfg.audioPathingDensity == "baseline");
    }
    SECTION("--set override: valid values apply, invalid rejected") {
        Darkness::RenderConfig cfg;
        CHECK(Darkness::applySetOverride("audio.pathing_probes.density",
                                         "baseline", cfg));
        CHECK(cfg.audioPathingDensity == "baseline");
        CHECK(Darkness::applySetOverride("audio.pathing_probes.density",
                                         "bends", cfg));
        CHECK(cfg.audioPathingDensity == "bends");
        CHECK_FALSE(Darkness::applySetOverride("audio.pathing_probes.density",
                                               "high", cfg));
        CHECK(cfg.audioPathingDensity == "bends");
    }
}

// Reverb thread allocation: two explicit integer counts. Both 0 =
// auto (hwconc-2 split). Both > 0 = literal values. Mixed handled in
// AudioService init with a [REVERB_THREADS][FALLBACK] warning.
TEST_CASE("audio reverb thread count YAML keys", "[config][yaml][audio]") {
    SECTION("defaults: conv_threads=0 (auto), sim_threads=0 (auto)") {
        Darkness::RenderConfig cfg;
        CHECK(cfg.convThreads == 0);
        CHECK(cfg.simThreads  == 0);
    }
    SECTION("explicit values override defaults") {
        TmpFile tmp(R"(
audio:
  performance:
    conv_threads: 3
    sim_threads:  5
)");
        Darkness::RenderConfig cfg;
        REQUIRE(Darkness::loadConfigFromYAML(tmp.path.string(), cfg));
        CHECK(cfg.convThreads == 3);
        CHECK(cfg.simThreads  == 5);
    }
    SECTION("clamping: conv_threads/sim_threads < 0 → 0, > 64 → 64") {
        TmpFile lo("audio:\n  performance:\n    conv_threads: -1\n    sim_threads: -5\n");
        Darkness::RenderConfig cfg1;
        Darkness::loadConfigFromYAML(lo.path.string(), cfg1);
        CHECK(cfg1.convThreads == 0);
        CHECK(cfg1.simThreads  == 0);

        TmpFile hi("audio:\n  performance:\n    conv_threads: 999\n    sim_threads: 200\n");
        Darkness::RenderConfig cfg2;
        Darkness::loadConfigFromYAML(hi.path.string(), cfg2);
        CHECK(cfg2.convThreads == 64);
        CHECK(cfg2.simThreads  == 64);
    }
}

// Deprecated keys (removed in 2026-05 cleanup) should be parsed without
// crashing — the loader emits a WARN to stderr and otherwise ignores
// them. Test verifies the loader returns true on a yaml containing only
// deprecated keys.
TEST_CASE("audio deprecated keys parse without error", "[config][yaml][audio]") {
    TmpFile tmp(R"(
audio:
  performance:
    convolution_workers: 4
    simulator_threads: 2
    max_reflection_voices: 8
    max_realtime_voices: 0
    sim_max_rays: 4096
    direct_max_sources: 256
    reflection_max_sources: 32
    sim_max_sources: 16
    reflection_demote_hysteresis_frames: 600
    reverb_threads: 8
    reverb_threads_conv_share: 0.5
  reflections:
    bake_skip: true
)");
    Darkness::RenderConfig cfg;
    REQUIRE(Darkness::loadConfigFromYAML(tmp.path.string(), cfg));
    // Deprecated keys ignored — fields fall back to RenderConfig defaults.
    CHECK(cfg.reverbVoices         == 16);
    CHECK(cfg.reverbVoicesRealtime == 0);
    CHECK(cfg.convThreads          == 0);
    CHECK(cfg.simThreads           == 0);
    // bake_skip is gone — yaml load is a no-op (no field to assert against).
    // The stderr [FALLBACK] warn is the user-visible signal.
}

// Q3 — per-voice spatialBlend override for AMB_ENVIRONMENTAL ambients.
// Makes room ambients (wind, church reverberance) feel less point-source-like
// while leaving object-attached ambients (no AMB_ENVIRONMENTAL flag)
// directional. 1.0 = full HRTF; 0.0 = mono passthrough.
TEST_CASE("audio environmental ambient spatial blend YAML key", "[config][yaml][audio]") {
    SECTION("default is 0.3 (mostly diffuse with subtle directional hint)") {
        Darkness::RenderConfig cfg;
        CHECK(cfg.ambEnvironmentalSpatialBlend == Catch::Approx(0.3f));
    }
    SECTION("explicit value overrides default") {
        TmpFile tmp(R"(
audio:
  ambient:
    environmental_spatial_blend: 0.6
)");
        Darkness::RenderConfig cfg;
        REQUIRE(Darkness::loadConfigFromYAML(tmp.path.string(), cfg));
        CHECK(cfg.ambEnvironmentalSpatialBlend == Catch::Approx(0.6f));
    }
    SECTION("clamping: -1.0 → 0.0, 2.0 → 1.0") {
        TmpFile lo("audio:\n  ambient:\n    environmental_spatial_blend: -1.0\n");
        Darkness::RenderConfig cfg1;
        Darkness::loadConfigFromYAML(lo.path.string(), cfg1);
        CHECK(cfg1.ambEnvironmentalSpatialBlend == Catch::Approx(0.0f));

        TmpFile hi("audio:\n  ambient:\n    environmental_spatial_blend: 2.0\n");
        Darkness::RenderConfig cfg2;
        Darkness::loadConfigFromYAML(hi.path.string(), cfg2);
        CHECK(cfg2.ambEnvironmentalSpatialBlend == Catch::Approx(1.0f));
    }
}

// Group D (2026-05) — dB-based ambient halt + spawn/halt fade ramps.
// Replaces the legacy Euclidean radius × hysteresis_*_mul gate. New keys
// live alongside the existing audio.ambient.* block.
TEST_CASE("audio ambient Group D halt + fade YAML keys", "[config][yaml][audio]") {
    SECTION("defaults match the spec") {
        Darkness::RenderConfig cfg;
        CHECK(cfg.ambientSpawnFadeInMs               == 150);
        CHECK(cfg.ambientHaltFadeOutMs               == 250);
        CHECK(cfg.ambientHaltAudibilityThresholdDb   == Catch::Approx(-50.0f));
        CHECK(cfg.ambientHaltBelowThresholdFrames    == 30);
    }
    SECTION("explicit YAML values override defaults") {
        TmpFile tmp(R"(
audio:
  ambient:
    spawn_fade_in_ms: 300
    halt_fade_out_ms: 400
    halt_audibility_threshold_db: -45
    halt_below_threshold_frames: 120
)");
        Darkness::RenderConfig cfg;
        REQUIRE(Darkness::loadConfigFromYAML(tmp.path.string(), cfg));
        CHECK(cfg.ambientSpawnFadeInMs             == 300);
        CHECK(cfg.ambientHaltFadeOutMs             == 400);
        CHECK(cfg.ambientHaltAudibilityThresholdDb == Catch::Approx(-45.0f));
        CHECK(cfg.ambientHaltBelowThresholdFrames  == 120);
    }
    SECTION("clamping enforces documented ranges") {
        TmpFile lo(R"(
audio:
  ambient:
    spawn_fade_in_ms: -100
    halt_fade_out_ms: -50
    halt_audibility_threshold_db: -200
    halt_below_threshold_frames: 1
)");
        Darkness::RenderConfig cfg1;
        Darkness::loadConfigFromYAML(lo.path.string(), cfg1);
        CHECK(cfg1.ambientSpawnFadeInMs             == 0);
        CHECK(cfg1.ambientHaltFadeOutMs             == 0);
        CHECK(cfg1.ambientHaltAudibilityThresholdDb == Catch::Approx(-80.0f));
        CHECK(cfg1.ambientHaltBelowThresholdFrames  == 5);

        TmpFile hi(R"(
audio:
  ambient:
    spawn_fade_in_ms: 9999
    halt_fade_out_ms: 9999
    halt_audibility_threshold_db: 0
    halt_below_threshold_frames: 9999
)");
        Darkness::RenderConfig cfg2;
        Darkness::loadConfigFromYAML(hi.path.string(), cfg2);
        CHECK(cfg2.ambientSpawnFadeInMs             == 2000);
        CHECK(cfg2.ambientHaltFadeOutMs             == 2000);
        CHECK(cfg2.ambientHaltAudibilityThresholdDb == Catch::Approx(-20.0f));
        CHECK(cfg2.ambientHaltBelowThresholdFrames  == 600);
    }
    SECTION("--set CLI overrides for the same keys") {
        Darkness::RenderConfig cfg;
        CHECK(Darkness::applySetOverride("audio.ambient.spawn_fade_in_ms",
                                         "500", cfg));
        CHECK(cfg.ambientSpawnFadeInMs == 500);
        CHECK(Darkness::applySetOverride("audio.ambient.halt_fade_out_ms",
                                         "600", cfg));
        CHECK(cfg.ambientHaltFadeOutMs == 600);
        CHECK(Darkness::applySetOverride(
            "audio.ambient.halt_audibility_threshold_db", "-30", cfg));
        CHECK(cfg.ambientHaltAudibilityThresholdDb == Catch::Approx(-30.0f));
        CHECK(Darkness::applySetOverride(
            "audio.ambient.halt_below_threshold_frames", "200", cfg));
        CHECK(cfg.ambientHaltBelowThresholdFrames == 200);
    }
}

// Group D — the radius × hysteresis_*_mul gate retired in 2026-05. The
// loader still accepts the old keys (emits a one-shot WARN to stderr) so
// existing yamls don't fail to parse; everything else falls through to
// defaults. Mirrors the audio.performance deprecation test above.
TEST_CASE("audio ambient deprecated hysteresis keys parse without error",
          "[config][yaml][audio]") {
    TmpFile tmp(R"(
audio:
  ambient:
    hysteresis_start_mul: 1.5
    hysteresis_stop_mul: 2.0
)");
    Darkness::RenderConfig cfg;
    REQUIRE(Darkness::loadConfigFromYAML(tmp.path.string(), cfg));
    // Deprecated keys are ignored — Group D defaults remain in place.
    CHECK(cfg.ambientSpawnFadeInMs             == 150);
    CHECK(cfg.ambientHaltFadeOutMs             == 250);
    CHECK(cfg.ambientHaltAudibilityThresholdDb == Catch::Approx(-50.0f));
    CHECK(cfg.ambientHaltBelowThresholdFrames  == 30);
}
