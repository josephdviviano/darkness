// Config documentation parity — the checks that keep `--help`, the example
// yaml and the parser from drifting apart again.
//
// History that motivates this file: by 2026-08-05 the hand-written help text
// listed 21 of ~170 keys and omitted every graphics subsection the visual
// campaign had added, while README.md still called it the canonical config
// reference. Two live keys (`graphics.tweq_visibility_gating`,
// `audio.pathing_probes.vis_range_override_ft`) existed in the parser and in
// neither yaml file, so nobody could discover them. Nothing failed; the
// documentation just quietly stopped being true.
//
// Three sources have to agree, and each check below binds one pair:
//
//   parser (RenderConfig.h)  ──[1]──>  example yaml  ──[2]──>  --help text
//
// [1] every key the parser reads is documented, and
// [2] every documented key reaches the generated help.
//
// Failures name the offending key, because "the config docs are stale" is not
// an actionable message.
//
// RELATIONSHIP TO tools/config_audit.py — deliberate overlap, not an oversight.
// The auditor is the authoritative and larger check: it runs these two plus the
// example↔live key parity and a dead-documentation check, understands more of
// the ways a key can be read, and runs from the pre-commit hook. This file
// exists because the hook fires at commit time and the test suite fires every
// build: a mismatch introduced at 10am should not wait until the commit to
// surface. If the two ever disagree, the auditor is right.
#include <catch2/catch_test_macros.hpp>

#include <fstream>
#include <regex>
#include <set>
#include <sstream>
#include <string>

#include <yaml-cpp/yaml.h>

#include "ConfigHelpText.h"

namespace {

// Keys the parser reads ONLY to tell the user they are gone. The list lives in
// tools/deprecated_config_keys.txt, not here: tools/config_audit.py runs the
// same check from the pre-commit hook and needs the same list, and two copies
// of an allowlist is how an allowlist goes stale.
std::set<std::string> parseDeprecated(const std::string &text) {
    std::set<std::string> out;
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        line = line.substr(0, line.find('#'));
        const size_t b = line.find_first_not_of(" \t\r");
        if (b == std::string::npos)
            continue;
        const size_t e = line.find_last_not_of(" \t\r");
        out.insert(line.substr(b, e - b + 1));
    }
    return out;
}

// Every key name appearing anywhere in the example yaml. Names rather than
// full paths: the parser is read by text scan below, which sees `node["key"]`
// without the path that reached it.
std::set<std::string> exampleKeyNames(const YAML::Node &node) {
    std::set<std::string> out;
    if (node.IsMap()) {
        for (const auto &kv : node) {
            out.insert(kv.first.as<std::string>());
            const std::set<std::string> child = exampleKeyNames(kv.second);
            out.insert(child.begin(), child.end());
        }
    }
    return out;
}

std::string readFile(const std::string &path) {
    std::ifstream in(path);
    REQUIRE(in.good());
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

} // namespace

TEST_CASE("every config key the parser reads is documented in the example yaml",
          "[config][docs]") {
    // Scanning RenderConfig.h as text rather than introspecting a registry,
    // because there is no registry — the parser is an if-chain of
    // `node["key"]` lookups. Crude, but it is the only thing that sees a key
    // the moment it is added: on first run it flagged 8 of 173 keys, and all 8
    // were either genuine deprecations or genuinely undocumented.
    //
    // NOTE this scan knows only the `node["key"]` form. config_audit.py also
    // understands the readTint() helper (KEY_READ_PATTERNS) and so is the more
    // accurate of the two; a key read ONLY through a helper is invisible here.
    // That is a false negative, never a false alarm, which is the right way
    // round for a test that gates the build.
    const std::string src = readFile(std::string(REPO_ROOT) + "/src/main/RenderConfig.h");

    const YAML::Node example =
        YAML::LoadFile(std::string(REPO_ROOT) + "/darknessRender.example.yaml");
    const std::set<std::string> documented = exampleKeyNames(example);

    const std::set<std::string> deprecated =
        parseDeprecated(readFile(std::string(REPO_ROOT) +
                                 "/tools/deprecated_config_keys.txt"));
    REQUIRE_FALSE(deprecated.empty());   // a silently empty allowlist weakens the check

    // Custom raw-string delimiter: the pattern itself contains `)"`, which
    // would close a plain R"(...)".
    const std::regex lookup(R"RE(\[\s*"([a-z0-9_]+)"\s*\])RE");
    std::set<std::string> undocumented;
    for (std::sregex_iterator it(src.begin(), src.end(), lookup), end;
         it != end; ++it) {
        const std::string key = (*it)[1].str();
        if (documented.count(key) || deprecated.count(key))
            continue;
        undocumented.insert(key);
    }

    for (const std::string &key : undocumented) {
        // Either document it in darknessRender.example.yaml (and add it to
        // darknessRender.yaml too — the two are kept at key parity), or, if
        // the parser only reads it to report its removal, add it to
        // tools/deprecated_config_keys.txt instead.
        INFO("undocumented config key read by RenderConfig.h: " << key);
        CHECK(false);
    }
    CHECK(undocumented.empty());
}

TEST_CASE("the generated help text covers every key in the example yaml",
          "[config][docs]") {
    // Guards the one manual step left: ConfigHelpText.h is generated, and
    // nothing forces regeneration at build time (that would put Python in the
    // build). This test is what makes forgetting it loud.
    const YAML::Node example =
        YAML::LoadFile(std::string(REPO_ROOT) + "/darknessRender.example.yaml");
    const std::set<std::string> documented = exampleKeyNames(example);

    // Collect the keys the help actually LISTS, rather than substring-matching
    // the whole blob: a key name occurring inside some other key's description
    // would otherwise count as documented.
    std::set<std::string> listed;
    for (int i = 0; i < Darkness::kConfigHelpLineCount; ++i) {
        std::string line = Darkness::kConfigHelpLines[i];
        const size_t first = line.find_first_not_of(' ');
        if (first == std::string::npos)
            continue;
        line = line.substr(first);
        const size_t colon = line.find(':');
        if (colon != std::string::npos)
            listed.insert(line.substr(0, colon));
    }

    std::set<std::string> missing;
    for (const std::string &key : documented) {
        if (!listed.count(key))
            missing.insert(key);
    }

    for (const std::string &key : missing) {
        INFO("key in darknessRender.example.yaml but missing from --help: "
             << key << " — regenerate with tools/gen_config_help.py");
        CHECK(false);
    }
    CHECK(missing.empty());
}

TEST_CASE("the generated help text is not obviously truncated", "[config][docs]") {
    // A generator bug that emitted an empty or tiny table would still satisfy
    // the coverage test above if the example yaml were also empty. Assert the
    // shape independently: the reference is a couple of hundred lines, every
    // non-blank line is indented (it is a nested key listing), and no line
    // runs past the width the generator targets.
    CHECK(Darkness::kConfigHelpLineCount > 100);

    for (int i = 0; i < Darkness::kConfigHelpLineCount; ++i) {
        const std::string line = Darkness::kConfigHelpLines[i];
        if (line.empty())
            continue;
        INFO("line " << i << ": " << line);
        CHECK(line[0] == ' ');
        // Multi-byte UTF-8 (the ellipsis, en dashes) makes byte length exceed
        // column count, so allow headroom rather than asserting the exact
        // target width.
        CHECK(line.size() < 140);
    }
}
