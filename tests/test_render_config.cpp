// Unit tests for RenderConfig (YAML + CLI configuration)
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

    CHECK(cfg.lmScale    == 1);
    CHECK(cfg.filterMode == 0);
    CHECK(cfg.linearMips == false);
    CHECK(cfg.sharpMips  == false);
    CHECK(cfg.showObjects   == true);
    CHECK(cfg.portalCulling == true);
    CHECK(cfg.forceFlicker  == false);
}

TEST_CASE("YAML full load", "[config][yaml]") {
    TmpFile tmp(R"(
graphics:
  lm_scale: 4
  filter_mode: 2
  linear_mips: true
  sharp_mips: true
developer:
  show_objects: false
  portal_culling: false
  force_flicker: true
)");

    Darkness::RenderConfig cfg;
    bool ok = Darkness::loadConfigFromYAML(tmp.path.string(), cfg);

    REQUIRE(ok);
    CHECK(cfg.lmScale    == 4);
    CHECK(cfg.filterMode == 2);
    CHECK(cfg.linearMips == true);
    CHECK(cfg.sharpMips  == true);
    CHECK(cfg.showObjects   == false);
    CHECK(cfg.portalCulling == false);
    CHECK(cfg.forceFlicker  == true);
}

TEST_CASE("YAML partial load — unset fields keep defaults", "[config][yaml]") {
    TmpFile tmp(R"(
graphics:
  lm_scale: 3
)");

    Darkness::RenderConfig cfg;
    bool ok = Darkness::loadConfigFromYAML(tmp.path.string(), cfg);

    REQUIRE(ok);
    CHECK(cfg.lmScale    == 3);
    // Everything else should be default
    CHECK(cfg.filterMode == 0);
    CHECK(cfg.linearMips == false);
    CHECK(cfg.sharpMips  == false);
    CHECK(cfg.showObjects   == true);
    CHECK(cfg.portalCulling == true);
    CHECK(cfg.forceFlicker  == false);
}

TEST_CASE("YAML missing file returns false, config unchanged", "[config][yaml]") {
    Darkness::RenderConfig cfg;
    Darkness::RenderConfig orig = cfg;

    bool ok = Darkness::loadConfigFromYAML("/tmp/nonexistent_darkness_config_xyz.yaml", cfg);

    CHECK_FALSE(ok);
    // Config must be unchanged
    CHECK(cfg.lmScale      == orig.lmScale);
    CHECK(cfg.filterMode   == orig.filterMode);
    CHECK(cfg.showObjects  == orig.showObjects);
}

TEST_CASE("YAML malformed file returns false, no crash", "[config][yaml]") {
    TmpFile tmp("{{{{not valid yaml at all : : :");

    Darkness::RenderConfig cfg;
    Darkness::RenderConfig orig = cfg;

    bool ok = Darkness::loadConfigFromYAML(tmp.path.string(), cfg);

    CHECK_FALSE(ok);
    // Config must be unchanged — malformed YAML should not partially apply
    CHECK(cfg.lmScale      == orig.lmScale);
    CHECK(cfg.filterMode   == orig.filterMode);
    CHECK(cfg.showObjects  == orig.showObjects);
}

TEST_CASE("CLI flags override defaults", "[config][cli]") {
    Darkness::RenderConfig cfg;
    std::vector<std::string> args = {
        "darknessRender", "mission.mis",
        "--no-objects", "--no-cull", "--filter",
        "--lm-scale", "4", "--force-flicker",
        "--linear-mips", "--sharp-mips"
    };
    auto argv = makeArgv(args);
    int argc = static_cast<int>(argv.size());

    Darkness::CliResult cli = Darkness::applyCliOverrides(argc, argv.data(), cfg);

    CHECK(cfg.showObjects   == false);
    CHECK(cfg.portalCulling == false);
    CHECK(cfg.filterMode    == 1);
    CHECK(cfg.lmScale       == 4);
    CHECK(cfg.forceFlicker  == true);
    CHECK(cfg.linearMips    == true);
    CHECK(cfg.sharpMips     == true);
    CHECK(cli.misPath != nullptr);
    CHECK(std::string(cli.misPath) == "mission.mis");
}

TEST_CASE("CLI-only fields: --res, --config, --help, positional", "[config][cli]") {
    Darkness::RenderConfig cfg;
    std::vector<std::string> args = {
        "darknessRender", "test.mis",
        "--res", "/path/to/res",
        "--config", "my.yaml",
        "--help"
    };
    auto argv = makeArgv(args);
    int argc = static_cast<int>(argv.size());

    Darkness::CliResult cli = Darkness::applyCliOverrides(argc, argv.data(), cfg);

    CHECK(cli.helpRequested == true);
    CHECK(cli.resPath    == "/path/to/res");
    CHECK(cli.configPath == "my.yaml");
    REQUIRE(cli.misPath != nullptr);
    CHECK(std::string(cli.misPath) == "test.mis");
}

TEST_CASE("lm_scale clamping in YAML and CLI", "[config][clamp]") {
    SECTION("YAML clamps below 1 to 1") {
        TmpFile tmp("graphics:\n  lm_scale: -5\n");
        Darkness::RenderConfig cfg;
        Darkness::loadConfigFromYAML(tmp.path.string(), cfg);
        CHECK(cfg.lmScale == 1);
    }
    SECTION("YAML clamps above 8 to 8") {
        TmpFile tmp("graphics:\n  lm_scale: 99\n");
        Darkness::RenderConfig cfg;
        Darkness::loadConfigFromYAML(tmp.path.string(), cfg);
        CHECK(cfg.lmScale == 8);
    }
    SECTION("CLI clamps below 1 to 1") {
        Darkness::RenderConfig cfg;
        std::vector<std::string> args = {"prog", "--lm-scale", "0"};
        auto argv = makeArgv(args);
        Darkness::applyCliOverrides(static_cast<int>(argv.size()), argv.data(), cfg);
        CHECK(cfg.lmScale == 1);
    }
    SECTION("CLI clamps above 8 to 8") {
        Darkness::RenderConfig cfg;
        std::vector<std::string> args = {"prog", "--lm-scale", "100"};
        auto argv = makeArgv(args);
        Darkness::applyCliOverrides(static_cast<int>(argv.size()), argv.data(), cfg);
        CHECK(cfg.lmScale == 8);
    }
}

TEST_CASE("CLI overrides YAML — last-write wins", "[config][precedence]") {
    // Simulate the real config loading order: YAML first, then CLI
    TmpFile tmp(R"(
graphics:
  lm_scale: 6
  filter_mode: 2
developer:
  show_objects: false
)");

    Darkness::RenderConfig cfg;
    bool ok = Darkness::loadConfigFromYAML(tmp.path.string(), cfg);
    REQUIRE(ok);
    CHECK(cfg.lmScale == 6);

    // CLI overrides lm_scale but not filter_mode or show_objects
    std::vector<std::string> args = {"prog", "--lm-scale", "2"};
    auto argv = makeArgv(args);
    Darkness::applyCliOverrides(static_cast<int>(argv.size()), argv.data(), cfg);

    CHECK(cfg.lmScale       == 2);  // CLI wins
    CHECK(cfg.filterMode    == 2);  // YAML preserved
    CHECK(cfg.showObjects   == false);  // YAML preserved
}
