// Light-corona unit tests.
//
// Three things are worth pinning down here, all of which were established
// from data or from the binaries rather than chosen:
//
//   1. The TGA decoder, because bitmap.crf is the first place this renderer
//      meets the format and CORONA.TGA's shape lives entirely in its alpha
//      channel — a decoder that dropped alpha would produce a white square
//      instead of a glow, and would look "loaded fine" doing it.
//   2. PropCorona's on-disk layout, checked against the actual bytes of the
//      one authored corona in retail Thief 2 (MISS5 object 35).
//   3. Object occlusion, which cannot be observed from a fixed camera in a
//      real mission — the geometry has to be built deliberately.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cstring>
#include <vector>

#include "DarknessMath.h"
#include "TGADecoder.h"
#include "property/DarkPropertyDefs.h"
// ObjectCollisionGeometry.h forward-declares ObjectPlacement and
// ParsedBinMesh and leaves defining them to its includer.
#include "ObjectPropParser.h"
#include "BinMeshParser.h"
#include "physics/ObjectCollisionGeometry.h"

using Catch::Matchers::WithinAbs;
using namespace Darkness;

// ── Helpers ────────────────────────────────────────────────────────────────

/// Build an uncompressed 32-bit BGRA TGA, bottom-left origin — exactly the
/// shape CORONA.TGA has.
static std::vector<uint8_t> makeTGA32(uint16_t w, uint16_t h,
                                      const std::vector<uint8_t> &bgra,
                                      bool topDown = false) {
    std::vector<uint8_t> out(18, 0);
    out[2] = 2; // uncompressed true-colour
    out[12] = static_cast<uint8_t>(w & 0xFF);
    out[13] = static_cast<uint8_t>(w >> 8);
    out[14] = static_cast<uint8_t>(h & 0xFF);
    out[15] = static_cast<uint8_t>(h >> 8);
    out[16] = 32;
    out[17] = static_cast<uint8_t>(0x08 | (topDown ? 0x20 : 0x00));
    out.insert(out.end(), bgra.begin(), bgra.end());
    return out;
}

/// One axis-aligned box body centred at `centre`.
static ObjectCollisionBody makeBox(int32_t objID, const Vector3 &centre,
                                   const Vector3 &edges) {
    ObjectCollisionBody b{};
    b.objID = objID;
    b.shapeType = CollisionShapeType::OBB;
    b.worldPos = centre;
    b.edgeLengths = edges;
    b.rotation = glm::mat3(1.0f);
    b.aabbMin = centre - edges * 0.5f;
    b.aabbMax = centre + edges * 0.5f;
    return b;
}

// ── TGA decoding ───────────────────────────────────────────────────────────

TEST_CASE("TGA: 32-bit uncompressed decodes with channels swapped and rows flipped",
          "[corona][tga]") {
    // 2x2, bottom-left origin. On disk row 0 is the BOTTOM row.
    // Bottom row: pure blue opaque, pure green half-alpha
    // Top row:    pure red opaque, white transparent
    std::vector<uint8_t> bgra = {
        255, 0,   0,   255,   // bottom-left  B=255 -> blue
        0,   255, 0,   128,   // bottom-right G=255 -> green, alpha 128
        0,   0,   255, 255,   // top-left     R=255 -> red
        255, 255, 255, 0,     // top-right    white, alpha 0
    };
    auto file = makeTGA32(2, 2, bgra);
    REQUIRE(looksLikeTGA(file.data(), file.size()));

    DecodedImage img = decodeTGA(file.data(), file.size());
    REQUIRE(img.width == 2);
    REQUIRE(img.height == 2);
    REQUIRE(img.rgba.size() == 2 * 2 * 4);

    // Output is top-left origin, so the file's LAST row comes first.
    auto px = [&](int x, int y) { return &img.rgba[((y * 2) + x) * 4]; };
    CHECK(px(0, 0)[0] == 255); CHECK(px(0, 0)[1] == 0);   CHECK(px(0, 0)[2] == 0);
    CHECK(px(1, 0)[0] == 255); CHECK(px(1, 0)[1] == 255); CHECK(px(1, 0)[2] == 255);
    CHECK(px(0, 1)[2] == 255); // blue
    CHECK(px(1, 1)[1] == 255); // green

    // Alpha is the whole point of using TGA here — a corona's shape is its
    // alpha ramp, so it must survive verbatim.
    CHECK(px(0, 0)[3] == 255);
    CHECK(px(1, 0)[3] == 0);
    CHECK(px(0, 1)[3] == 255);
    CHECK(px(1, 1)[3] == 128);
}

TEST_CASE("TGA: descriptor bit 5 selects top-left origin", "[corona][tga]") {
    std::vector<uint8_t> bgra = {
        255, 0, 0, 255,   // row 0
        0, 0, 255, 255,   // row 1
    };
    DecodedImage bottomUp = decodeTGA(makeTGA32(1, 2, bgra, false).data(),
                                      18 + bgra.size());
    DecodedImage topDown  = decodeTGA(makeTGA32(1, 2, bgra, true).data(),
                                      18 + bgra.size());
    // Same bytes, opposite row order.
    CHECK(bottomUp.rgba[0] == topDown.rgba[4]);
    CHECK(bottomUp.rgba[4] == topDown.rgba[0]);
}

TEST_CASE("TGA: RLE and uncompressed decode identically", "[corona][tga]") {
    // 4x1 of two colours, encoded as one repeat packet then one literal.
    std::vector<uint8_t> raw;
    for (int i = 0; i < 2; ++i) { raw.insert(raw.end(), {10, 20, 30, 255}); }
    for (int i = 0; i < 2; ++i) { raw.insert(raw.end(), {40, 50, 60, 128}); }
    DecodedImage plain = decodeTGA(makeTGA32(4, 1, raw).data(), 18 + raw.size());

    std::vector<uint8_t> rle(18, 0);
    rle[2] = 10; // RLE true-colour
    rle[12] = 4; rle[14] = 1; rle[16] = 32; rle[17] = 0x08;
    rle.push_back(0x81);                                  // repeat, count 2
    rle.insert(rle.end(), {10, 20, 30, 255});
    rle.push_back(0x01);                                  // literal, count 2
    rle.insert(rle.end(), {40, 50, 60, 128});
    rle.insert(rle.end(), {40, 50, 60, 128});
    DecodedImage packed = decodeTGA(rle.data(), rle.size());

    REQUIRE(packed.width == plain.width);
    CHECK(packed.rgba == plain.rgba);
}

TEST_CASE("TGA: malformed input throws rather than reading past the buffer",
          "[corona][tga]") {
    std::vector<uint8_t> truncated(18, 0);
    truncated[2] = 2;
    truncated[12] = 64; truncated[14] = 64; truncated[16] = 32;
    // Header claims 64x64 but carries no pixel data at all.
    CHECK_THROWS(decodeTGA(truncated.data(), truncated.size()));
}

TEST_CASE("TGA: the sniff rejects PCX and GIF headers", "[corona][tga]") {
    // A PCX starts 0x0A 0x05 0x01 0x08 — imageType would read as 1
    // (colour-mapped) with colorMapType 5, which is not a valid map type.
    const uint8_t pcx[18] = { 0x0A, 0x05, 0x01, 0x08, 0, 0, 0, 0,
                              0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    CHECK_FALSE(looksLikeTGA(pcx, sizeof(pcx)));

    const uint8_t gif[18] = { 'G', 'I', 'F', '8', '7', 'a',
                              0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    CHECK_FALSE(looksLikeTGA(gif, sizeof(gif)));
}

// ── P$Corona on-disk layout ────────────────────────────────────────────────

TEST_CASE("PropCorona matches the retail MISS5 record byte for byte",
          "[corona][property]") {
    // The 52 bytes of retail Thief 2 MISS5 object 35 — the only functional
    // corona the shipped campaign authors. Field ORDER here is what the
    // editor's own label table gives; this test is what keeps a future
    // "tidy-up" of the struct from silently reinterpreting it.
    static_assert(sizeof(PropCorona) == 52, "layout drift");

    const uint8_t rec[52] = {
        0x81, 0x00, 0x00, 0x00,                          // flags = 0x81
        0x00, 0x00, 0x96, 0xC2,                          // pos.x
        0x9A, 0x59, 0x3D, 0x43,                          // pos.y
        0x00, 0x00, 0x30, 0x41,                          // pos.z
        0x00, 0x00, 0x00, 0x40,                          // radiusNear = 2.0
        0x00, 0x00, 0x82, 0x42,                          // radiusFar  = 65.0
        0x00, 0x00, 0x96, 0x43,                          // maxDist    = 300.0
        0x9A, 0x99, 0x19, 0x3E,                          // alpha      = 0.15
        'c', 'o', 'r', 'o', 'n', 'a', 0, 0,              // texture[0..7]
        0, 0, 0, 0, 0, 0, 0, 0,                          // texture[8..15]
        0x00, 0x00, 0x00, 0x00,                          // trailing
    };

    PropCorona pc{};
    std::memcpy(&pc, rec, sizeof(pc));

    CHECK(pc.flags == 0x81u);
    CHECK_THAT(pc.posX, WithinAbs(-75.0f, 0.001f));
    CHECK_THAT(pc.posY, WithinAbs(189.35f, 0.01f));
    CHECK_THAT(pc.posZ, WithinAbs(11.0f, 0.001f));
    CHECK_THAT(pc.radiusNear, WithinAbs(2.0f, 0.001f));
    CHECK_THAT(pc.radiusFar, WithinAbs(65.0f, 0.001f));
    CHECK_THAT(pc.maxDist, WithinAbs(300.0f, 0.001f));
    CHECK_THAT(pc.alpha, WithinAbs(0.15f, 0.001f));
    CHECK(std::string(pc.texture) == "corona");
    CHECK(pc.trailing == 0u);

    // The far radius is LARGER than the near one. That is not a mistake in
    // the data: the radii are world units, so a billboard has to grow with
    // distance to hold its apparent size. Any code that "fixes" this by
    // swapping them makes distant coronas vanish.
    CHECK(pc.radiusFar > pc.radiusNear);
}

// ── Object occlusion ───────────────────────────────────────────────────────

TEST_CASE("segmentVsAABB: broad phase accepts crossings and rejects misses",
          "[corona][occlusion]") {
    const Vector3 boxMin(-1, -1, -1), boxMax(1, 1, 1);

    SECTION("straight through") {
        Vector3 from(-5, 0, 0), to(5, 0, 0);
        CHECK(segmentVsAABB(from, to - from, boxMin, boxMax));
    }
    SECTION("segment stops short of the box") {
        Vector3 from(-5, 0, 0), to(-2, 0, 0);
        CHECK_FALSE(segmentVsAABB(from, to - from, boxMin, boxMax));
    }
    SECTION("passes beside the box") {
        Vector3 from(-5, 5, 0), to(5, 5, 0);
        CHECK_FALSE(segmentVsAABB(from, to - from, boxMin, boxMax));
    }
    SECTION("parallel to an axis and outside that slab") {
        // Degenerate direction on Y and Z — must not divide by zero into a
        // false accept.
        Vector3 from(-5, 9, 0), to(5, 9, 0);
        CHECK_FALSE(segmentVsAABB(from, to - from, boxMin, boxMax));
    }
    SECTION("starts inside") {
        Vector3 from(0, 0, 0), to(5, 0, 0);
        CHECK(segmentVsAABB(from, to - from, boxMin, boxMax));
    }
}

TEST_CASE("A door-sized OBB between viewer and light blocks the corona ray",
          "[corona][occlusion]") {
    // Viewer at the origin, light 20 units down +Y — the geometry of standing
    // in a corridor looking at a torch on the far wall.
    const Vector3 viewer(0, 0, 0);
    const Vector3 light(0, 20, 0);

    SECTION("closed door across the corridor occludes") {
        // A door slab: wide and tall, thin along the direction of travel.
        auto door = makeBox(500, Vector3(0, 10, 0), Vector3(8, 0.5f, 12));
        CHECK(rayVsOBB(viewer, light, door).hit);
    }

    SECTION("the same door swung out of the way does not") {
        // Open: the slab has rotated and translated clear of the doorway.
        auto door = makeBox(500, Vector3(6, 10, 0), Vector3(0.5f, 8, 12));
        CHECK_FALSE(rayVsOBB(viewer, light, door).hit);
    }

    SECTION("an object past the light does not occlude it") {
        auto crate = makeBox(501, Vector3(0, 30, 0), Vector3(4, 4, 4));
        CHECK_FALSE(rayVsOBB(viewer, light, crate).hit);
    }

    SECTION("an object off to the side does not occlude it") {
        auto crate = makeBox(502, Vector3(10, 10, 0), Vector3(4, 4, 4));
        CHECK_FALSE(rayVsOBB(viewer, light, crate).hit);
    }
}

TEST_CASE("Occlusion is partial when only part of the corona disc is covered",
          "[corona][occlusion]") {
    // This is what stops a corona popping as it slides behind a doorframe:
    // the trace samples a ring around the light, so an edge occluder blocks
    // some samples and not others, and the fraction drives the fade.
    const Vector3 viewer(0, 0, 0);
    const Vector3 light(0, 20, 0);
    const float discRadius = 4.0f;

    // A slab covering only the -X half of the corridor.
    auto halfDoor = makeBox(600, Vector3(-4, 10, 0), Vector3(8, 0.5f, 12));

    // Ring samples in the camera plane (camera looks +Y, so right = +X,
    // up = +Z), matching what updateCoronas() builds.
    int blocked = 0, clear = 0;
    for (int s = 0; s < 4; ++s) {
        const float ang = 1.5707963f * static_cast<float>(s);
        Vector3 target = light + Vector3(std::cos(ang) * discRadius, 0.0f,
                                         std::sin(ang) * discRadius);
        if (rayVsOBB(viewer, target, halfDoor).hit) ++blocked;
        else ++clear;
    }
    CHECK(blocked > 0);
    CHECK(clear > 0);
}
