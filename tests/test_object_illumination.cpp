// Object illumination unit tests. Exercises the math and shadow-cache logic
// directly against a synthesized WRParsedData (no real .mis fixture
// required). Each test sets up just enough world data to exercise one
// behavior in isolation: distance falloff, spotlight cone, radius cutoff,
// sun-slot patching, ambient + ExtraLight, dynamic lights, HSB→RGB.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

#include "DarknessMath.h"
#include "WRChunkParser.h"
#include "RenderParamsParser.h"
#include "ObjectIllumination.h"
#include "DynamicLightList.h"

using Catch::Matchers::WithinAbs;
using namespace Darkness;

// ── Helpers ────────────────────────────────────────────────────────────────

// Build a single-cell world that fills 3D space (one giant cell with no
// portals or solid polygons). Any raycast within this cell terminates at
// the cell boundary without hitting world geometry — i.e. all in-cell
// lights are visible. Useful for testing the lighting math without
// raycasting noise.
static WRParsedData makeOpenWorld(int numStaticLights) {
    WRParsedData wr{};
    wr.numCells = 1;
    wr.cells.resize(1);

    auto &cell = wr.cells[0];
    // Single huge bounding box — vertices at ±1000 on each axis, but no
    // polygons (so raycastWorld traverses the cell and exits without a hit).
    cell.numPolygons = 0;
    cell.numPortals  = 0;
    cell.numPlanes   = 0;
    cell.numTextured = 0;
    cell.center = Vector3(0.0f);
    cell.radius = 1000.0f;
    cell.flowGroup = 0;

    // light_indices: index 0 = count, [1..count] = indices.
    cell.lightIndices.resize(static_cast<size_t>(numStaticLights) + 1);
    cell.lightIndices[0] = static_cast<uint16_t>(numStaticLights);
    for (int i = 0; i < numStaticLights; ++i)
        cell.lightIndices[1 + i] = static_cast<uint16_t>(i);

    wr.numStaticLights = numStaticLights;
    wr.staticLights.resize(numStaticLights);
    return wr;
}

// Default render params (no ambient, sun off) so tests aren't polluted by
// background lighting unless explicitly added.
static RenderParams makeQuietRenderParams() {
    RenderParams rp{};
    rp.ambientLight = Vector3(0.0f);
    rp.useSun = false;
    rp.sunHue = 0.0f;
    rp.sunSaturation = 0.0f;
    rp.sunBrightness = 0.0f;
    rp.sunlightVector = Vector3(0.0f, 0.0f, -1.0f);
    recomputeRenderParamsDerived(rp);
    rp.sunScaledRgb = Vector3(0.0f);  // override the post-derive multiplication
    return rp;
}

// ── Distance attenuation ───────────────────────────────────────────────────

TEST_CASE("ObjectIlluminator: 1/r distance falloff for omni light",
          "[lighting]") {
    WRParsedData wr = makeOpenWorld(2);
    // Slot 0: sun (no contribution since brightness is zero in quiet rp).
    wr.staticLights[0] = WRStaticLight{
        Vector3(0.0f), Vector3(0.0f), Vector3(0.0f),
        -1.0f, 0.0f, 0.0f
    };
    // Slot 1: omni light at +X = 10, brightness (4, 8, 12).
    wr.staticLights[1] = WRStaticLight{
        Vector3(10.0f, 0.0f, 0.0f), Vector3(0.0f), Vector3(4.0f, 8.0f, 12.0f),
        -1.0f, 0.0f, 0.0f
    };

    RenderParams rp = makeQuietRenderParams();
    ObjectIlluminator ill;
    ill.setMissionData(&wr, &rp, nullptr);

    // Object at origin: distance to light = 10 → contribution = bright/10.
    // The blue channel pre-clamp would be 12/10 = 1.2 — output is clamped
    // at 1.0 per channel (matches the engine's CLUT clamp at 0.99).
    Vector3 r = ill.compute(/*objId=*/1, Vector3(0.0f), /*radius=*/0.0f, /*cell=*/0);
    CHECK_THAT(r.x, WithinAbs(0.4f, 1e-4f));
    CHECK_THAT(r.y, WithinAbs(0.8f, 1e-4f));
    CHECK_THAT(r.z, WithinAbs(1.0f, 1e-4f));  // clamped from 1.2
}

// ── Spotlight cone ─────────────────────────────────────────────────────────

TEST_CASE("ObjectIlluminator: spotlight inner/outer cone falloff",
          "[lighting]") {
    WRParsedData wr = makeOpenWorld(2);
    wr.staticLights[0] = WRStaticLight{};  // sun slot, no contribution
    wr.staticLights[0].inner = -1.0f;

    // Spotlight at origin pointing +X. Inner cos(15°), outer cos(30°).
    float ci = std::cos(15.0f * 3.14159265358979f / 180.0f);
    float co = std::cos(30.0f * 3.14159265358979f / 180.0f);
    wr.staticLights[1] = WRStaticLight{
        Vector3(0.0f),                         // loc
        Vector3(1.0f, 0.0f, 0.0f),             // dir = +X
        Vector3(1.0f, 1.0f, 1.0f),             // bright
        ci, co, 0.0f                           // inner, outer, radius
    };

    RenderParams rp = makeQuietRenderParams();
    ObjectIlluminator ill;
    ill.setMissionData(&wr, &rp, nullptr);

    // Sample at a fixed distance of 1.0 along several angles in the X-Y
    // plane. The spotlight contribution = scale × bright × (1/dist).
    // dist = 1, so result = scale × bright.
    auto sample = [&](float angleDeg) -> Vector3 {
        float a = angleDeg * 3.14159265358979f / 180.0f;
        Vector3 pos(std::cos(a), std::sin(a), 0.0f);
        return ill.compute(/*objId=*/1, pos, /*radius=*/0.0f, /*cell=*/0);
    };

    // Angle 0° = full spotlight intensity (within inner cone).
    Vector3 a0 = sample(0.0f);
    CHECK_THAT(a0.x, WithinAbs(1.0f, 1e-3f));

    // Angle 15° = on the inner-cone boundary: still full.
    Vector3 a15 = sample(15.0f);
    CHECK_THAT(a15.x, WithinAbs(1.0f, 1e-3f));

    // Angle 30° = on the outer boundary: zero.
    Vector3 a30 = sample(30.0f);
    CHECK_THAT(a30.x, WithinAbs(0.0f, 1e-3f));

    // Angle 45° = past outer cone: zero.
    Vector3 a45 = sample(45.0f);
    CHECK_THAT(a45.x, WithinAbs(0.0f, 1e-3f));

    // Angle ~22.5° = halfway between inner and outer: roughly 0.5.
    // The interpolation is linear in cos(angle), not angle, so the exact
    // midpoint value is between the cos boundaries; we accept a band.
    Vector3 amid = sample(22.5f);
    CHECK(amid.x > 0.2f);
    CHECK(amid.x < 0.8f);
}

// ── Radius cutoff ──────────────────────────────────────────────────────────

TEST_CASE("ObjectIlluminator: radius cutoff zeroes contribution",
          "[lighting]") {
    WRParsedData wr = makeOpenWorld(2);
    wr.staticLights[0] = WRStaticLight{};
    wr.staticLights[0].inner = -1.0f;
    wr.staticLights[1] = WRStaticLight{
        Vector3(0.0f), Vector3(0.0f), Vector3(1.0f, 1.0f, 1.0f),
        -1.0f, 0.0f, /*radius=*/10.0f
    };

    RenderParams rp = makeQuietRenderParams();
    ObjectIlluminator ill;
    ill.setMissionData(&wr, &rp, nullptr);

    // At distance 9 (< radius 10), contribution = 1/9 ≈ 0.111.
    Vector3 inside = ill.compute(1, Vector3(9.0f, 0, 0), 0.0f, 0);
    CHECK(inside.x > 0.1f);

    // At distance 11 (> radius 10), contribution = 0.
    // Object 2 to avoid sharing the shadow cache with object 1.
    Vector3 outside = ill.compute(2, Vector3(11.0f, 0, 0), 0.0f, 0);
    CHECK_THAT(outside.x, WithinAbs(0.0f, 1e-4f));
}

// ── Sun slot patching ─────────────────────────────────────────────────────

TEST_CASE("ObjectIlluminator: sun slot is patched per-evaluation",
          "[lighting]") {
    WRParsedData wr = makeOpenWorld(1);  // just slot 0 = sun
    // Disk values for slot 0 are scratch; we expect them to be overwritten.
    wr.staticLights[0] = WRStaticLight{
        Vector3(99.0f), Vector3(0.0f), Vector3(0.0f),
        99.0f, 99.0f, 99.0f  // garbage to detect non-overwrite
    };

    // Custom render params with a known sun direction and brightness.
    RenderParams rp{};
    rp.ambientLight = Vector3(0.0f);
    rp.useSun = true;
    // Sun comes from straight up (-Z direction → light points down).
    rp.sunlightVector = Vector3(0.0f, 0.0f, -1.0f);
    rp.sunRgb = Vector3(1.0f, 1.0f, 1.0f);
    rp.sunScaledRgb = Vector3(100.0f, 100.0f, 100.0f);
    // Manually compute sunlightNorm (would be done by recomputeRenderParamsDerived
    // but we want exact control here).
    rp.sunlightNorm = Vector3(0.0f, 0.0f, 1.0f);  // points TOWARD sun (+Z)

    ObjectIlluminator ill;
    ill.setMissionData(&wr, &rp, nullptr);

    // Object at origin, radius 0. Sun slot.loc should be patched to
    // origin + (+Z) * 125 = (0, 0, 125). Sun slot.bright should equal
    // sunScaledRgb. Output contribution = sunScaledRgb / 125.
    Vector3 r = ill.compute(1, Vector3(0.0f), 0.0f, 0);
    CHECK_THAT(r.x, WithinAbs(100.0f / 125.0f, 1e-3f));
    CHECK_THAT(r.y, WithinAbs(100.0f / 125.0f, 1e-3f));
    CHECK_THAT(r.z, WithinAbs(100.0f / 125.0f, 1e-3f));
}

// ── Ambient + (no ExtraLight property service) ────────────────────────────

TEST_CASE("ObjectIlluminator: ambient passes through with no lights",
          "[lighting]") {
    WRParsedData wr{};
    wr.numCells = 1;
    wr.cells.resize(1);
    wr.cells[0].lightIndices.resize(1);
    wr.cells[0].lightIndices[0] = 0;  // no lights in cell

    RenderParams rp = makeQuietRenderParams();
    rp.ambientLight = Vector3(0.1f, 0.2f, 0.3f);

    ObjectIlluminator ill;
    ill.setMissionData(&wr, &rp, nullptr);

    Vector3 r = ill.compute(1, Vector3(0.0f), 0.0f, 0);
    CHECK_THAT(r.x, WithinAbs(0.1f, 1e-4f));
    CHECK_THAT(r.y, WithinAbs(0.2f, 1e-4f));
    CHECK_THAT(r.z, WithinAbs(0.3f, 1e-4f));
}

// ── Animated-light multiplier ──────────────────────────────────────────────

TEST_CASE("ObjectIlluminator: setLightMultiplier scales static contribution",
          "[lighting]") {
    WRParsedData wr = makeOpenWorld(2);
    wr.staticLights[0] = WRStaticLight{};
    wr.staticLights[0].inner = -1.0f;
    // Slot 1: omni light at +X = 10, brightness (1, 1, 1).
    wr.staticLights[1] = WRStaticLight{
        Vector3(10.0f, 0.0f, 0.0f), Vector3(0.0f), Vector3(1.0f, 1.0f, 1.0f),
        -1.0f, 0.0f, 0.0f
    };

    RenderParams rp = makeQuietRenderParams();
    ObjectIlluminator ill;
    ill.setMissionData(&wr, &rp, nullptr);

    // Default multiplier 1.0 → contribution = 1/10 = 0.1.
    Vector3 full = ill.compute(1, Vector3(0.0f), 0.0f, 0);
    CHECK_THAT(full.x, WithinAbs(0.1f, 1e-4f));

    // Multiplier 0.5 → contribution = 0.05.
    ill.setLightMultiplier(1, 0.5f);
    Vector3 half = ill.compute(2, Vector3(0.0f), 0.0f, 0);
    CHECK_THAT(half.x, WithinAbs(0.05f, 1e-4f));

    // Multiplier 0 → contribution skipped entirely (light is "off").
    ill.setLightMultiplier(1, 0.0f);
    Vector3 off = ill.compute(3, Vector3(0.0f), 0.0f, 0);
    CHECK_THAT(off.x, WithinAbs(0.0f, 1e-4f));

    // resetLightMultipliers brings everyone back to 1.0.
    ill.resetLightMultipliers();
    Vector3 again = ill.compute(4, Vector3(0.0f), 0.0f, 0);
    CHECK_THAT(again.x, WithinAbs(0.1f, 1e-4f));
}

// ── Output clamping ────────────────────────────────────────────────────────

TEST_CASE("ObjectIlluminator: output clamps to 1.0 per channel",
          "[lighting]") {
    WRParsedData wr = makeOpenWorld(2);
    wr.staticLights[0] = WRStaticLight{};
    wr.staticLights[0].inner = -1.0f;
    // Slot 1: very bright light at distance 1 → contribution = 50 per ch.
    wr.staticLights[1] = WRStaticLight{
        Vector3(1.0f, 0.0f, 0.0f), Vector3(0.0f), Vector3(50.0f, 50.0f, 50.0f),
        -1.0f, 0.0f, 0.0f
    };

    RenderParams rp = makeQuietRenderParams();
    ObjectIlluminator ill;
    ill.setMissionData(&wr, &rp, nullptr);

    Vector3 r = ill.compute(1, Vector3(0.0f), 0.0f, 0);
    CHECK_THAT(r.x, WithinAbs(1.0f, 1e-4f));
    CHECK_THAT(r.y, WithinAbs(1.0f, 1e-4f));
    CHECK_THAT(r.z, WithinAbs(1.0f, 1e-4f));
}

// ── Empty cell light list ──────────────────────────────────────────────────

TEST_CASE("ObjectIlluminator: cell with no lights returns ambient only",
          "[lighting]") {
    WRParsedData wr{};
    wr.numCells = 1;
    wr.cells.resize(1);
    // Don't even populate lightIndices — empty array.
    RenderParams rp = makeQuietRenderParams();
    rp.ambientLight = Vector3(0.5f, 0.5f, 0.5f);

    ObjectIlluminator ill;
    ill.setMissionData(&wr, &rp, nullptr);

    Vector3 r = ill.compute(1, Vector3(0.0f), 0.0f, 0);
    CHECK_THAT(r.x, WithinAbs(0.5f, 1e-4f));
}

// ── Dynamic lights ─────────────────────────────────────────────────────────

TEST_CASE("ObjectIlluminator: dynamic lights add to total per frame",
          "[lighting]") {
    WRParsedData wr = makeOpenWorld(1);
    wr.staticLights[0] = WRStaticLight{};
    wr.staticLights[0].inner = -1.0f;

    RenderParams rp = makeQuietRenderParams();

    DynamicLightList dyn;
    dyn.add(Vector3(10.0f, 0.0f, 0.0f), Vector3(2.0f, 2.0f, 2.0f), 0.0f);

    ObjectIlluminator ill;
    ill.setMissionData(&wr, &rp, nullptr);
    ill.setDynamicLights(&dyn);

    Vector3 r = ill.compute(1, Vector3(0.0f), 0.0f, 0);
    // Distance 10 → contribution = 2/10 = 0.2.
    CHECK_THAT(r.x, WithinAbs(0.2f, 1e-3f));
    CHECK_THAT(r.y, WithinAbs(0.2f, 1e-3f));
    CHECK_THAT(r.z, WithinAbs(0.2f, 1e-3f));

    // After reset, dynamic light is gone — even calling compute twice on
    // the same object should return ambient (= 0) because the cache only
    // covers static lights, not dynamic.
    dyn.reset();
    Vector3 r2 = ill.compute(2, Vector3(0.0f), 0.0f, 0);
    CHECK_THAT(r2.x, WithinAbs(0.0f, 1e-4f));
}

TEST_CASE("DynamicLightList: respects MAX_DYNAMIC cap", "[lighting]") {
    DynamicLightList dyn;
    for (int i = 0; i < DynamicLightList::kMaxDynamicLights + 10; ++i)
        dyn.add(Vector3(static_cast<float>(i), 0, 0), Vector3(1.0f), 0.0f);
    CHECK(dyn.count() == DynamicLightList::kMaxDynamicLights);
}

// ── HSB→RGB ────────────────────────────────────────────────────────────────

TEST_CASE("RenderParamsParser: HSB→RGB primary colors", "[lighting]") {
    // Hue 0, full saturation = pure red.
    Vector3 red = hsbToRgb(0.0f, 1.0f);
    CHECK_THAT(red.x, WithinAbs(1.0f, 1e-4f));
    CHECK_THAT(red.y, WithinAbs(0.0f, 1e-4f));
    CHECK_THAT(red.z, WithinAbs(0.0f, 1e-4f));

    // Hue 1/3, full saturation = pure green.
    Vector3 green = hsbToRgb(1.0f / 3.0f, 1.0f);
    CHECK_THAT(green.x, WithinAbs(0.0f, 1e-4f));
    CHECK_THAT(green.y, WithinAbs(1.0f, 1e-4f));
    CHECK_THAT(green.z, WithinAbs(0.0f, 1e-4f));

    // Hue 2/3, full saturation = pure blue.
    Vector3 blue = hsbToRgb(2.0f / 3.0f, 1.0f);
    CHECK_THAT(blue.x, WithinAbs(0.0f, 1e-4f));
    CHECK_THAT(blue.y, WithinAbs(0.0f, 1e-4f));
    CHECK_THAT(blue.z, WithinAbs(1.0f, 1e-4f));
}

TEST_CASE("RenderParamsParser: HSB→RGB zero saturation = white",
          "[lighting]") {
    // Saturation = 0 should produce white regardless of hue.
    for (float h : {0.0f, 0.25f, 0.5f, 0.75f, 0.9f}) {
        Vector3 c = hsbToRgb(h, 0.0f);
        CHECK_THAT(c.x, WithinAbs(1.0f, 1e-4f));
        CHECK_THAT(c.y, WithinAbs(1.0f, 1e-4f));
        CHECK_THAT(c.z, WithinAbs(1.0f, 1e-4f));
    }
}

TEST_CASE("RenderParamsParser: recompute populates derived fields",
          "[lighting]") {
    RenderParams rp{};
    rp.sunHue = 0.0f;
    rp.sunSaturation = 1.0f;
    rp.sunBrightness = 100.0f;
    rp.sunlightVector = Vector3(0.0f, 0.0f, -1.0f);
    recomputeRenderParamsDerived(rp);

    CHECK_THAT(rp.sunRgb.x, WithinAbs(1.0f, 1e-4f));
    CHECK_THAT(rp.sunScaledRgb.x, WithinAbs(100.0f, 1e-4f));
    // sunlightNorm points TOWARD the sun = -normalize(sunlight_vector).
    // With sunlight_vector pointing "down" (-Z), the normal points up (+Z).
    CHECK_THAT(rp.sunlightNorm.z, WithinAbs(1.0f, 1e-4f));
}
