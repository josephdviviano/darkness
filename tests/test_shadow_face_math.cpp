/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2024-2026 darkness contributors
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *****************************************************************************/

// ShadowFaceMath properties (S1, PLAN.HIGH_RES_SHADOWS.md). The S4 world
// shader must MIRROR this math in GLSL — these tests pin the C++ side so
// the mirror has a stable reference, and the S1 acceptance cross-check
// (GPU faces vs raycastWorld) closes the loop on the GPU copy.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "ShadowFaceMath.h"
#include "LightmapBake.h"   // bakeRand01 — the deterministic test RNG

#include <cmath>

using namespace Darkness;

TEST_CASE("every direction maps to exactly one covering face", "[shadow]") {
    uint32_t rng = 42u;
    for (int i = 0; i < 100000; ++i) {
        Vector3 d(bakeRand01(rng) * 2.0f - 1.0f,
                  bakeRand01(rng) * 2.0f - 1.0f,
                  bakeRand01(rng) * 2.0f - 1.0f);
        if (glm::length(d) < 1e-3f) continue;
        const int face = shadowFaceForDirection(d);
        REQUIRE(face >= 0);
        REQUIRE(face < kShadowFaceCount);
        // The chosen face's axis is the major component, pointing the
        // right way.
        const Vector3 axis = shadowFaceDir(face);
        const float along = glm::dot(d, axis);
        REQUIRE(along > 0.0f);
        REQUIRE(along >= std::abs(d.x) - 1e-5f * std::abs(d.x));
        REQUIRE(along + 1e-5f >= std::abs(d.y));
        REQUIRE(along + 1e-5f >= std::abs(d.z));
    }
}

TEST_CASE("the covering face projects the point inside its UV square",
          "[shadow]") {
    uint32_t rng = 7u;
    const Vector3 light(12.0f, -34.0f, 5.0f);
    int tested = 0;
    for (int i = 0; i < 100000; ++i) {
        Vector3 d(bakeRand01(rng) * 2.0f - 1.0f,
                  bakeRand01(rng) * 2.0f - 1.0f,
                  bakeRand01(rng) * 2.0f - 1.0f);
        if (glm::length(d) < 1e-2f) continue;
        const Vector3 p = light + d * (1.0f + bakeRand01(rng) * 50.0f);
        const int face = shadowFaceForDirection(p - light);
        float u = -1.0f, v = -1.0f;
        REQUIRE(shadowFaceUV(light, face, p, u, v));
        REQUIRE(u >= 0.0f);
        REQUIRE(u <= 1.0f);
        REQUIRE(v >= 0.0f);
        REQUIRE(v <= 1.0f);
        ++tested;
    }
    REQUIRE(tested > 90000);

    // The face axis itself lands dead centre.
    for (int f = 0; f < kShadowFaceCount; ++f) {
        float u = 0, v = 0;
        REQUIRE(shadowFaceUV(light, f, light + shadowFaceDir(f) * 10.0f,
                             u, v));
        REQUIRE(u == Catch::Approx(0.5f).margin(1e-5));
        REQUIRE(v == Catch::Approx(0.5f).margin(1e-5));
    }
}

TEST_CASE("UV math agrees with the face view-projection matrix", "[shadow]") {
    // The GPU rasterises through shadowFaceView×shadowFaceProj; the lookup
    // side computes UVs directly. They must be the same mapping or every
    // shadow test reads a neighbouring texel's distance.
    uint32_t rng = 99u;
    const Vector3 light(-3.0f, 8.0f, 21.0f);
    const float farZ = 100.0f;
    for (int i = 0; i < 20000; ++i) {
        Vector3 d(bakeRand01(rng) * 2.0f - 1.0f,
                  bakeRand01(rng) * 2.0f - 1.0f,
                  bakeRand01(rng) * 2.0f - 1.0f);
        if (glm::length(d) < 1e-2f) continue;
        const Vector3 p = light + d * (2.0f + bakeRand01(rng) * 40.0f);
        const int face = shadowFaceForDirection(p - light);
        float u = 0, v = 0;
        if (!shadowFaceUV(light, face, p, u, v)) continue;

        const Matrix4 vp = shadowFaceProj(0.05f, farZ)
                         * shadowFaceView(light, face);
        const glm::vec4 clip = vp * glm::vec4(p, 1.0f);
        REQUIRE(clip.w > 0.0f);
        const float cu = clip.x / clip.w * 0.5f + 0.5f;
        const float cv = clip.y / clip.w * 0.5f + 0.5f;
        REQUIRE(cu == Catch::Approx(u).margin(1e-3));
        // GL clip y and our v both grow with the face's up axis — the
        // projection has no y-flip, so they must agree directly.
        REQUIRE(cv == Catch::Approx(v).margin(1e-3));
    }
}

TEST_CASE("stored distance is monotonic along a ray and reach-normalised",
          "[shadow]") {
    const Vector3 light(0.0f, 0.0f, 0.0f);
    const float reach = 80.0f;
    float prev = -1.0f;
    for (int i = 1; i <= 50; ++i) {
        const Vector3 p = Vector3(1.0f, 0.4f, 0.2f)
                        * (static_cast<float>(i) * 1.5f);
        const float dist = shadowFaceDistance(light, p, reach);
        REQUIRE(dist > prev);
        prev = dist;
    }
    REQUIRE(shadowFaceDistance(light, Vector3(reach, 0, 0), reach)
            == Catch::Approx(1.0f));
}

TEST_CASE("ZO projection differs from NO only in the z rows", "[shadow]") {
    // The GPU renders with whichever variant matches the backend's clip
    // convention (bgfx caps homogeneousDepth); the UV lookup math is proven
    // against the NO variant. That transfer is valid only if x/y/w behave
    // identically — pin it.
    const float nearZ = 0.05f, farZ = 120.0f;
    const Matrix4 no = shadowFaceProj(nearZ, farZ);
    const Matrix4 zo = shadowFaceProjZO(nearZ, farZ);
    for (int col = 0; col < 4; ++col) {
        REQUIRE(no[col][0] == zo[col][0]);   // x row
        REQUIRE(no[col][1] == zo[col][1]);   // y row
        REQUIRE(no[col][3] == zo[col][3]);   // w row
    }
    // And the z ranges land on each convention's boundaries.
    auto ndcZ = [](const Matrix4 &m, float viewZ) {
        const glm::vec4 clip = m * glm::vec4(0.0f, 0.0f, viewZ, 1.0f);
        return clip.z / clip.w;
    };
    REQUIRE(ndcZ(no, -nearZ) == Catch::Approx(-1.0f).margin(1e-4));
    REQUIRE(ndcZ(no, -farZ)  == Catch::Approx( 1.0f).margin(1e-4));
    REQUIRE(ndcZ(zo, -nearZ) == Catch::Approx( 0.0f).margin(1e-4));
    REQUIRE(ndcZ(zo, -farZ)  == Catch::Approx( 1.0f).margin(1e-4));
}

TEST_CASE("face culling is conservative: never drops a sphere holding a "
          "projectable point", "[shadow]") {
    uint32_t rng = 1234u;
    const Vector3 light(4.0f, -2.0f, 9.0f);
    const float reach = 60.0f;
    int kept = 0;
    for (int i = 0; i < 50000; ++i) {
        Vector3 d(bakeRand01(rng) * 2.0f - 1.0f,
                  bakeRand01(rng) * 2.0f - 1.0f,
                  bakeRand01(rng) * 2.0f - 1.0f);
        if (glm::length(d) < 1e-2f) continue;
        d = glm::normalize(d);
        const float dist = 0.5f + bakeRand01(rng) * (reach - 0.5f);
        const Vector3 p = light + d * dist;
        const int face = shadowFaceForDirection(p - light);
        float u, v;
        if (!shadowFaceUV(light, face, p, u, v)) continue;
        // Any sphere CONTAINING p must survive culling for p's face.
        const float radius = 0.01f + bakeRand01(rng) * 20.0f;
        Vector3 off(bakeRand01(rng) * 2.0f - 1.0f,
                    bakeRand01(rng) * 2.0f - 1.0f,
                    bakeRand01(rng) * 2.0f - 1.0f);
        const float offLen = glm::length(off);
        const Vector3 center =
            p + (offLen > 1e-4f ? off * (radius * 0.99f *
                                         bakeRand01(rng) / offLen)
                                : Vector3(0.0f));
        REQUIRE(shadowFaceSeesSphere(light, face, center, radius, reach));
        ++kept;
    }
    REQUIRE(kept > 40000);
}

TEST_CASE("face culling rejects what it can prove away", "[shadow]") {
    const Vector3 light(0.0f, 0.0f, 0.0f);
    const float reach = 50.0f;
    for (int f = 0; f < kShadowFaceCount; ++f) {
        const Vector3 fwd = shadowFaceDir(f);
        // Sphere squarely behind this face's frustum: centred on -fwd,
        // radius small enough not to straddle the diagonal planes.
        REQUIRE_FALSE(
            shadowFaceSeesSphere(light, f, -fwd * 20.0f, 2.0f, reach));
        // On-axis sphere is seen…
        REQUIRE(shadowFaceSeesSphere(light, f, fwd * 20.0f, 2.0f, reach));
        // …until it leaves the reach sphere entirely.
        REQUIRE_FALSE(
            shadowFaceSeesSphere(light, f, fwd * (reach + 5.0f), 2.0f, reach));
        // A huge sphere behind the light still straddles the frustum planes
        // and must be kept (conservatism beats tightness).
        REQUIRE(shadowFaceSeesSphere(light, f, -fwd * 3.0f, 10.0f, reach));
    }
}

TEST_CASE("atlas tiles are disjoint and inside the atlas", "[shadow]") {
    const int tileSize = 512;
    const int tilesPerRow = 8;                   // 4096 atlas
    const int slots = (tilesPerRow * tilesPerRow) / kShadowFaceCount;
    std::vector<uint64_t> seen;
    for (int s = 0; s < slots; ++s) {
        for (int f = 0; f < kShadowFaceCount; ++f) {
            int x = -1, y = -1;
            shadowAtlasTileOrigin(s, f, tilesPerRow, tileSize, x, y);
            REQUIRE(x >= 0);
            REQUIRE(y >= 0);
            REQUIRE(x + tileSize <= tilesPerRow * tileSize);
            REQUIRE(y + tileSize <= tilesPerRow * tileSize);
            REQUIRE(x % tileSize == 0);
            REQUIRE(y % tileSize == 0);
            const uint64_t key = (static_cast<uint64_t>(x) << 32)
                               | static_cast<uint32_t>(y);
            REQUIRE(std::find(seen.begin(), seen.end(), key) == seen.end());
            seen.push_back(key);
        }
    }
}
