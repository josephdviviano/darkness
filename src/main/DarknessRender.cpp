/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2005-2009 openDarkEngine team
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

// World geometry viewer with baked lightmaps and object mesh rendering

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <vector>
#include <string>
#include <algorithm>
#include <chrono>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <queue>

#include <SDL.h>
#include <SDL_syswm.h>

#include <bgfx/bgfx.h>
#include <bgfx/platform.h>
#include <bx/math.h>

#include "../shaders/basic_shader.h"
#include "../shaders/textured_shader.h"
#include "../shaders/lightmapped_shader.h"
#include "../shaders/water_shader.h"
#include "WRChunkParser.h"
#include "TXListParser.h"
#include "CRFTextureLoader.h"
#include "CRFModelLoader.h"
#include "LightmapAtlas.h"
#include "SpawnFinder.h"
#include "LightingSystem.h"
#include "ObjectPropParser.h"
#include "BinMeshParser.h"
#include "RenderConfig.h"

static const int WINDOW_WIDTH  = 1280;
static const int WINDOW_HEIGHT = 720;

// ── Vertex layouts ──

struct PosColorVertex {
    float x, y, z;
    uint32_t abgr;

    static bgfx::VertexLayout layout;

    static void init() {
        layout.begin()
            .add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
            .add(bgfx::Attrib::Color0, 4, bgfx::AttribType::Uint8, true)
            .end();
    }
};

bgfx::VertexLayout PosColorVertex::layout;

struct PosColorUVVertex {
    float x, y, z;
    uint32_t abgr;
    float u, v;

    static bgfx::VertexLayout layout;

    static void init() {
        layout.begin()
            .add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
            .add(bgfx::Attrib::Color0, 4, bgfx::AttribType::Uint8, true)
            .add(bgfx::Attrib::TexCoord0, 2, bgfx::AttribType::Float)
            .end();
    }
};

bgfx::VertexLayout PosColorUVVertex::layout;

struct PosUV2Vertex {
    float x, y, z;
    float u0, v0; // diffuse texture UV
    float u1, v1; // lightmap atlas UV

    static bgfx::VertexLayout layout;

    static void init() {
        layout.begin()
            .add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
            .add(bgfx::Attrib::TexCoord0, 2, bgfx::AttribType::Float)
            .add(bgfx::Attrib::TexCoord1, 2, bgfx::AttribType::Float)
            .end();
    }
};

bgfx::VertexLayout PosUV2Vertex::layout;

// ── Cell+texture group for batched draw calls with portal culling ──
// Groups are sorted by (cellID, txtIndex) so per-cell index ranges are contiguous.
// When portal culling is active, only groups whose cellID is in the visible set
// are submitted. When culling is off, all groups are submitted (same as before).

struct CellTextureGroup {
    uint32_t cellID    = 0;
    uint8_t  txtIndex  = 0;
    uint8_t  flowGroup = 0;  // flow group index (water only, 0 = no flow)
    uint32_t firstIndex = 0;
    uint32_t numIndices = 0;
};

// ── World mesh (textured) ──

struct WorldMesh {
    std::vector<PosColorUVVertex> vertices;
    std::vector<uint32_t> indices;
    std::vector<CellTextureGroup> groups; // sorted by (cellID, txtIndex)
    float cx, cy, cz;
};

// ── Flat-only mesh (fallback) ──

struct FlatMesh {
    std::vector<PosColorVertex> vertices;
    std::vector<uint32_t> indices;
    std::vector<CellTextureGroup> groups; // sorted by (cellID, txtIndex)
    float cx, cy, cz;
};

struct LightmappedMesh {
    std::vector<PosUV2Vertex> vertices;
    std::vector<uint32_t> indices;
    std::vector<CellTextureGroup> groups;
    float cx, cy, cz;
};

static uint32_t packABGR(float r, float g, float b) {
    uint8_t ri = static_cast<uint8_t>(std::min(std::max(r, 0.0f), 1.0f) * 255.0f);
    uint8_t gi = static_cast<uint8_t>(std::min(std::max(g, 0.0f), 1.0f) * 255.0f);
    uint8_t bi = static_cast<uint8_t>(std::min(std::max(b, 0.0f), 1.0f) * 255.0f);
    return 0xff000000u | (bi << 16) | (gi << 8) | ri;
}

// Overload with explicit alpha for translucent geometry (e.g. water surfaces)
static uint32_t packABGR(float r, float g, float b, float a) {
    uint8_t ri = static_cast<uint8_t>(std::min(std::max(r, 0.0f), 1.0f) * 255.0f);
    uint8_t gi = static_cast<uint8_t>(std::min(std::max(g, 0.0f), 1.0f) * 255.0f);
    uint8_t bi = static_cast<uint8_t>(std::min(std::max(b, 0.0f), 1.0f) * 255.0f);
    uint8_t ai = static_cast<uint8_t>(std::min(std::max(a, 0.0f), 1.0f) * 255.0f);
    return (uint32_t(ai) << 24) | (uint32_t(bi) << 16) | (uint32_t(gi) << 8) | ri;
}

// ── Mipmap generation and texture creation with full mip chain ──

// Alpha handling mode for mipmap generation:
// ALPHA_TEST  — preserves alpha coverage so cutout geometry (grates, foliage)
//               doesn't shrink or disappear at distance
// ALPHA_BLEND — no coverage adjustment (for fully opaque textures like skybox faces)
enum class MipAlphaMode { ALPHA_TEST, ALPHA_BLEND };

// sRGB linearization helpers for gamma-correct mipmap filtering.
// Simplified pow(2.2) approximation — accurate enough for texture downsampling.
static float srgbToLinear(uint8_t v) {
    float f = v / 255.0f;
    return std::pow(f, 2.2f);
}

static uint8_t linearToSrgb(float v) {
    float f = std::pow(std::max(0.0f, std::min(1.0f, v)), 1.0f / 2.2f);
    return static_cast<uint8_t>(f * 255.0f + 0.5f);
}

// ── Kaiser-windowed sinc filter for high-quality mipmap downsampling ──
// Replaces the naive 2x2 box filter with a wider kernel that reduces aliasing
// on high-frequency textures. Uses premultiplied alpha to prevent dark halos
// at transparent edges.

// Modified Bessel function I0 — polynomial approximation (Abramowitz & Stegun 9.8.1)
static double besselI0(double x) {
    double ax = std::abs(x);
    if (ax < 3.75) {
        double t = x / 3.75;
        t *= t;
        return 1.0 + t * (3.5156229 + t * (3.0899424 + t * (1.2067492
             + t * (0.2659732 + t * (0.0360768 + t * 0.0045813)))));
    }
    double t = 3.75 / ax;
    return (std::exp(ax) / std::sqrt(ax))
         * (0.39894228 + t * (0.01328592 + t * (0.00225319
         + t * (-0.00157565 + t * (0.00916281 + t * (-0.02057706
         + t * (0.02635537 + t * (-0.01647633 + t * 0.00392377))))))));
}

// Kaiser window function: windowed sinc for anti-aliased downsampling
// radius=3.0 gives a 6-tap diameter kernel; beta=4.0 gives a sharp cutoff
// with minimal ringing — good balance for game textures
static double kaiserWindow(double x, double radius, double beta) {
    if (std::abs(x) > radius) return 0.0;
    double r = x / radius;
    return besselI0(beta * std::sqrt(std::max(0.0, 1.0 - r * r))) / besselI0(beta);
}

// Normalized sinc function: sin(pi*x) / (pi*x)
static double sincFilter(double x) {
    if (std::abs(x) < 1e-6) return 1.0;
    double px = M_PI * x;
    return std::sin(px) / px;
}

// Downsample RGBA image by 2x using Kaiser-windowed sinc filter.
// Non-separable 2D kernel — up to ~37 taps per output pixel, fast enough
// for the 64-256px textures typical in Dark Engine assets at load time.
//
// premultiply: multiply RGB by alpha before filtering, divide after
//              (prevents dark halos at transparent edges)
// linearize:   convert sRGB to linear before filtering, back to sRGB after
//              (gamma-correct downsampling for more accurate color blending)
static std::vector<uint8_t> downsampleKaiser(const uint8_t *src,
                                              uint32_t srcW, uint32_t srcH,
                                              bool premultiply, bool linearize) {
    uint32_t dstW = std::max(1u, srcW / 2);
    uint32_t dstH = std::max(1u, srcH / 2);
    std::vector<uint8_t> dst(dstW * dstH * 4);

    // Kaiser kernel parameters
    const double radius = 3.0;
    const double beta = 4.0;

    // Precompute Kaiser kernel window radius in source pixels
    // Scale factor is 2.0 (halving dimensions), filter radius in source space = radius * scale
    const int filterRadius = static_cast<int>(std::ceil(radius));

    for (uint32_t dy = 0; dy < dstH; ++dy) {
        for (uint32_t dx = 0; dx < dstW; ++dx) {
            // Center of the output pixel in source coordinates
            double cx = (dx + 0.5) * 2.0 - 0.5;
            double cy = (dy + 0.5) * 2.0 - 0.5;

            // Accumulate weighted samples
            double sumR = 0, sumG = 0, sumB = 0, sumA = 0;
            double sumWeight = 0;

            int sx0 = std::max(0, static_cast<int>(std::floor(cx)) - filterRadius);
            int sy0 = std::max(0, static_cast<int>(std::floor(cy)) - filterRadius);
            int sx1 = std::min(static_cast<int>(srcW) - 1,
                               static_cast<int>(std::ceil(cx)) + filterRadius);
            int sy1 = std::min(static_cast<int>(srcH) - 1,
                               static_cast<int>(std::ceil(cy)) + filterRadius);

            for (int sy = sy0; sy <= sy1; ++sy) {
                double fy = (sy - cy) / 2.0;  // normalized to filter space
                double wy = sincFilter(fy) * kaiserWindow(fy, radius, beta);

                for (int sx = sx0; sx <= sx1; ++sx) {
                    double fx = (sx - cx) / 2.0;
                    double wx = sincFilter(fx) * kaiserWindow(fx, radius, beta);
                    double w = wx * wy;

                    const uint8_t *p = src + (sy * srcW + sx) * 4;
                    float r, g, b, a;

                    // Decode alpha
                    a = p[3] / 255.0f;

                    // Decode RGB with optional sRGB linearization
                    if (linearize) {
                        r = srgbToLinear(p[0]);
                        g = srgbToLinear(p[1]);
                        b = srgbToLinear(p[2]);
                    } else {
                        r = p[0] / 255.0f;
                        g = p[1] / 255.0f;
                        b = p[2] / 255.0f;
                    }

                    // Premultiplied alpha filtering: weight RGB by alpha
                    // to prevent transparent pixels from bleeding dark into edges
                    if (premultiply) {
                        r *= a;
                        g *= a;
                        b *= a;
                    }

                    sumR += w * r;
                    sumG += w * g;
                    sumB += w * b;
                    sumA += w * a;
                    sumWeight += w;
                }
            }

            // Normalize by total weight
            if (sumWeight > 1e-10) {
                sumR /= sumWeight;
                sumG /= sumWeight;
                sumB /= sumWeight;
                sumA /= sumWeight;
            }

            // Un-premultiply: recover RGB from premultiplied values
            if (premultiply && sumA > 1e-6) {
                sumR /= sumA;
                sumG /= sumA;
                sumB /= sumA;
            }

            // Convert back to sRGB if we linearized
            uint8_t *out = dst.data() + (dy * dstW + dx) * 4;
            if (linearize) {
                out[0] = linearToSrgb(static_cast<float>(sumR));
                out[1] = linearToSrgb(static_cast<float>(sumG));
                out[2] = linearToSrgb(static_cast<float>(sumB));
            } else {
                out[0] = static_cast<uint8_t>(std::min(std::max(sumR, 0.0), 1.0) * 255.0 + 0.5);
                out[1] = static_cast<uint8_t>(std::min(std::max(sumG, 0.0), 1.0) * 255.0 + 0.5);
                out[2] = static_cast<uint8_t>(std::min(std::max(sumB, 0.0), 1.0) * 255.0 + 0.5);
            }
            out[3] = static_cast<uint8_t>(std::min(std::max(sumA, 0.0), 1.0) * 255.0 + 0.5);
        }
    }
    return dst;
}

// ── Alpha-coverage preservation for alpha-tested textures ──
// Prevents cutout geometry (grates, fences, foliage) from shrinking/disappearing
// at distance by adjusting mip alpha values to maintain the same coverage ratio
// as the base level.

// Compute the fraction of pixels with alpha >= threshold (matching shader discard)
static float computeAlphaCoverage(const uint8_t *rgba, uint32_t w, uint32_t h,
                                   uint8_t threshold = 128) {
    if (w == 0 || h == 0) return 0.0f;
    uint32_t count = 0;
    uint32_t total = w * h;
    for (uint32_t i = 0; i < total; ++i) {
        if (rgba[i * 4 + 3] >= threshold) ++count;
    }
    return static_cast<float>(count) / static_cast<float>(total);
}

// Rescale alpha values in the mip so that the coverage (fraction of pixels
// passing the alpha test) matches targetCoverage. Uses a histogram-based
// approach: build alpha histogram, find the alpha value that yields the
// target coverage, then remap so that value maps to the discard threshold (128).
static void preserveAlphaCoverage(uint8_t *rgba, uint32_t w, uint32_t h,
                                   float targetCoverage) {
    if (w == 0 || h == 0) return;
    if (targetCoverage <= 0.0f || targetCoverage >= 1.0f) return;

    uint32_t total = w * h;

    // Build cumulative histogram of alpha values
    uint32_t hist[256] = {};
    for (uint32_t i = 0; i < total; ++i) {
        hist[rgba[i * 4 + 3]]++;
    }

    // Find the alpha threshold that yields targetCoverage
    // Count pixels >= each threshold level
    uint32_t targetCount = static_cast<uint32_t>(targetCoverage * total + 0.5f);
    uint32_t cumAbove = 0;
    int currentThreshold = 255;
    for (int a = 255; a >= 0; --a) {
        cumAbove += hist[a];
        if (cumAbove >= targetCount) {
            currentThreshold = a;
            break;
        }
    }

    // Remap alphas so that currentThreshold maps to 128 (the shader discard threshold)
    // Scale = 128.0 / currentThreshold (if currentThreshold > 0)
    if (currentThreshold <= 0) return;

    float scale = 128.0f / static_cast<float>(currentThreshold);
    for (uint32_t i = 0; i < total; ++i) {
        float a = rgba[i * 4 + 3] * scale;
        rgba[i * 4 + 3] = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, a + 0.5f)));
    }
}

// ── Optional unsharp-mask sharpening for mip levels ──
// Counteracts the inherent softening from downsampling by applying a subtle
// sharpen: out = original + strength * (original - gaussian_blur).
// Uses a 3x3 Gaussian blur kernel with sigma ~0.85.

static void sharpenMipLevel(uint8_t *rgba, uint32_t w, uint32_t h, float strength) {
    if (w < 3 || h < 3 || strength <= 0.0f) return;

    // 3x3 Gaussian kernel (sigma ~0.85), normalized to sum=1
    // Center=4/16, edges=2/16, corners=1/16
    static const float kernel[3][3] = {
        {1.0f/16, 2.0f/16, 1.0f/16},
        {2.0f/16, 4.0f/16, 2.0f/16},
        {1.0f/16, 2.0f/16, 1.0f/16}
    };

    // Work on a copy so we read unmodified original values
    std::vector<uint8_t> blurred(w * h * 4);

    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            float sum[4] = {};
            for (int ky = -1; ky <= 1; ++ky) {
                // Clamp with signed arithmetic to avoid unsigned underflow
                int iy = static_cast<int>(y) + ky;
                uint32_t sy = static_cast<uint32_t>(std::min(std::max(iy, 0), static_cast<int>(h) - 1));
                for (int kx = -1; kx <= 1; ++kx) {
                    int ix = static_cast<int>(x) + kx;
                    uint32_t sx = static_cast<uint32_t>(std::min(std::max(ix, 0), static_cast<int>(w) - 1));
                    float kw = kernel[ky + 1][kx + 1];
                    const uint8_t *p = rgba + (sy * w + sx) * 4;
                    for (int c = 0; c < 4; ++c) sum[c] += kw * p[c];
                }
            }
            uint8_t *out = blurred.data() + (y * w + x) * 4;
            for (int c = 0; c < 4; ++c) {
                out[c] = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, sum[c] + 0.5f)));
            }
        }
    }

    // Apply unsharp mask: out = original + strength * (original - blurred)
    // Only sharpen RGB, leave alpha untouched
    uint32_t total = w * h;
    for (uint32_t i = 0; i < total; ++i) {
        uint8_t *p = rgba + i * 4;
        const uint8_t *b = blurred.data() + i * 4;
        for (int c = 0; c < 3; ++c) {
            float val = p[c] + strength * (static_cast<float>(p[c]) - b[c]);
            p[c] = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, val + 0.5f)));
        }
    }
}

// Create a bgfx texture with a full mip chain using Kaiser-windowed sinc downsampling.
// Returns the texture handle. Uses the provided sampler flags for the base level.
//
// alphaMode: ALPHA_TEST preserves coverage (cutout geometry), ALPHA_BLEND skips it
// linearize: sRGB→linear before filtering, linear→sRGB after (gamma-correct mips)
// sharpen:   apply subtle unsharp mask to each mip level to preserve detail
static bgfx::TextureHandle createMipmappedTexture(
    const uint8_t *rgba, uint32_t w, uint32_t h, uint64_t samplerFlags,
    MipAlphaMode alphaMode = MipAlphaMode::ALPHA_TEST,
    bool linearize = false,
    bool sharpen = false)
{
    // Count mip levels: floor(log2(max(w,h))) + 1
    uint32_t mipCount = 1;
    {
        uint32_t maxDim = std::max(w, h);
        while (maxDim > 1) { maxDim >>= 1; ++mipCount; }
    }

    // Measure base-level alpha coverage for preservation in smaller mips
    float baseCoverage = 0.0f;
    if (alphaMode == MipAlphaMode::ALPHA_TEST) {
        baseCoverage = computeAlphaCoverage(rgba, w, h, 128);
    }

    // Generate all mip levels on CPU
    struct MipLevel { std::vector<uint8_t> data; uint32_t w, h; };
    std::vector<MipLevel> mips(mipCount);
    mips[0].data.assign(rgba, rgba + w * h * 4);
    mips[0].w = w;
    mips[0].h = h;

    for (uint32_t i = 1; i < mipCount; ++i) {
        mips[i].w = std::max(1u, mips[i-1].w / 2);
        mips[i].h = std::max(1u, mips[i-1].h / 2);

        // Kaiser-windowed sinc downsample with premultiplied alpha (always)
        // and optional sRGB linearization
        mips[i].data = downsampleKaiser(mips[i-1].data.data(),
                                         mips[i-1].w, mips[i-1].h,
                                         true, linearize);

        // Optional: sharpen to counteract filter softening (strength 0.4)
        if (sharpen) {
            sharpenMipLevel(mips[i].data.data(), mips[i].w, mips[i].h, 0.4f);
        }

        // Preserve alpha coverage so cutout geometry doesn't shrink at distance
        // Only for ALPHA_TEST textures with partial coverage (not fully opaque/transparent)
        if (alphaMode == MipAlphaMode::ALPHA_TEST
            && baseCoverage > 0.0f && baseCoverage < 1.0f) {
            preserveAlphaCoverage(mips[i].data.data(), mips[i].w, mips[i].h,
                                   baseCoverage);
        }
    }

    // Concatenate all mip levels into one contiguous buffer for bgfx
    uint32_t totalSize = 0;
    for (const auto &m : mips) totalSize += static_cast<uint32_t>(m.data.size());

    const bgfx::Memory *mem = bgfx::alloc(totalSize);
    uint32_t offset = 0;
    for (const auto &m : mips) {
        std::memcpy(mem->data + offset, m.data.data(), m.data.size());
        offset += static_cast<uint32_t>(m.data.size());
    }

    return bgfx::createTexture2D(
        static_cast<uint16_t>(w),
        static_cast<uint16_t>(h),
        true, // hasMips — bgfx expects all levels in the memory buffer
        1, bgfx::TextureFormat::RGBA8,
        samplerFlags, mem);
}

// ── Build flat-shaded mesh (no textures) ──

static FlatMesh buildFlatMesh(const Darkness::WRParsedData &wr) {
    FlatMesh mesh;

    // Sun direction for Lambertian shading
    float sunDir[3] = { 0.3f, 0.8f, 0.4f };
    float sunLen = std::sqrt(sunDir[0]*sunDir[0] + sunDir[1]*sunDir[1] + sunDir[2]*sunDir[2]);
    sunDir[0] /= sunLen; sunDir[1] /= sunLen; sunDir[2] /= sunLen;

    float sumX = 0, sumY = 0, sumZ = 0;
    int cellCount = 0;

    // Temporary per-(cell, texture) triangle lists for portal culling support
    // Key: (cellID << 8) | txtIndex — cell in high bits for sort order
    std::map<uint64_t, std::vector<uint32_t>> cellTexTriangles;

    for (uint32_t ci = 0; ci < wr.numCells; ++ci) {
        const auto &cell = wr.cells[ci];

        sumX += cell.center.x;
        sumY += cell.center.y;
        sumZ += cell.center.z;
        ++cellCount;

        int numSolid = cell.numPolygons - cell.numPortals;

        for (int pi = 0; pi < numSolid; ++pi) {
            const auto &poly = cell.polygons[pi];
            const auto &indices = cell.polyIndices[pi];

            if (poly.count < 3) continue;

            // Skip sky polygons (texture 249 = BACKHACK_IDX) — these are holes
            // that should reveal the sky dome rendered behind world geometry
            if (pi < cell.numTextured && cell.texturing[pi].txt == 249)
                continue;

            // Get plane normal for flat shading
            Darkness::Plane plane;
            if (poly.plane < cell.planes.size())
                plane = cell.planes[poly.plane];

            Darkness::Vector3 n = plane.normal.normalisedCopy();
            float dot = n.x * sunDir[0] + n.y * sunDir[1] + n.z * sunDir[2];
            float brightness = std::max(dot, 0.0f) * 0.85f + 0.15f;

            // Color based on cell media type
            uint32_t color;
            if (cell.mediaType == 2) {
                // Water: blue tint
                color = packABGR(brightness * 0.4f, brightness * 0.5f, brightness * 0.8f);
            } else {
                // Air: warm stone beige
                color = packABGR(brightness * 0.75f, brightness * 0.65f, brightness * 0.5f);
            }

            // Emit vertices for this polygon
            uint32_t baseVertex = static_cast<uint32_t>(mesh.vertices.size());

            for (int vi = 0; vi < poly.count; ++vi) {
                uint8_t idx = indices[vi];
                if (idx >= cell.vertices.size()) continue;
                const auto &v = cell.vertices[idx];
                mesh.vertices.push_back({v.x, v.y, v.z, color});
            }

            // Fan-triangulate: (0, t+1, t) — matches original WRCell.cpp winding
            // Group by (cellID, 0) since flat mesh has no texture index
            uint64_t key = (static_cast<uint64_t>(ci) << 8);
            auto &triList = cellTexTriangles[key];
            for (int t = 1; t < poly.count - 1; ++t) {
                triList.push_back(baseVertex);
                triList.push_back(baseVertex + t + 1);
                triList.push_back(baseVertex + t);
            }
        }
    }

    // Build sorted index buffer: groups sorted by (cellID, txtIndex)
    // std::map is already sorted by key
    for (auto &kv : cellTexTriangles) {
        uint32_t cellID = static_cast<uint32_t>(kv.first >> 8);
        uint8_t txtIdx = static_cast<uint8_t>(kv.first & 0xFF);
        CellTextureGroup grp;
        grp.cellID = cellID;
        grp.txtIndex = txtIdx;
        grp.firstIndex = static_cast<uint32_t>(mesh.indices.size());
        grp.numIndices = static_cast<uint32_t>(kv.second.size());
        mesh.groups.push_back(grp);
        mesh.indices.insert(mesh.indices.end(), kv.second.begin(), kv.second.end());
    }

    if (cellCount > 0) {
        mesh.cx = sumX / cellCount;
        mesh.cy = sumY / cellCount;
        mesh.cz = sumZ / cellCount;
    } else {
        mesh.cx = mesh.cy = mesh.cz = 0;
    }

    return mesh;
}

// ── Texture dimensions for UV normalization ──

struct TexDimensions {
    uint32_t w, h;
};

// ── Water flow data ──
// Forward declarations needed by buildWaterMesh and parseFlowData.

// FLOW_TEX chunk: 256 entries mapping flow group index to water textures.
// in_texture/out_texture are runtime-allocated palette indices — NOT usable as
// TXLIST indices. Instead, the name field holds the texture base name which maps
// to fam.crf paths: "water/<name>in.PCX" (air side) / "water/<name>out.PCX".
static constexpr int MAX_FLOW_GROUPS = 256;

struct FlowTexEntry {
    int16_t inTexture;     // runtime palette index (NOT a TXLIST index)
    int16_t outTexture;    // runtime palette index (NOT a TXLIST index)
    char    name[28];      // texture base name (e.g. "gr" → water/grin.PCX)
};
static_assert(sizeof(FlowTexEntry) == 32, "FLOW_TEX entry must be 32 bytes");

// CELL_MOTION chunk contains two sequential arrays:
// 1. PortalCellMotion[256] — current state (center, angle, flags, axis)
// 2. sMedMoCellMotion[256] — velocities (center_change, angle_change)
// The state stores the current animation position; the velocities are per-second deltas.
#pragma pack(push, 1)
struct PortalCellMotion {
    float    centerX, centerY, centerZ; // rotation pivot / UV origin in world space
    uint16_t angle;                     // current rotation angle (binary radians, 65536 = 360°)
    int32_t  inMotion;                  // 0=disabled, 1=enabled
    uint8_t  majorAxis;                 // water plane: 0=X, 1=Y, 2=Z (default Z for horizontal)
    uint8_t  pad1;
    uint8_t  pad2;
};
static_assert(sizeof(PortalCellMotion) == 21, "PortalCellMotion must be 21 bytes");

struct MedMoCellMotion {
    float    centerChangeX, centerChangeY, centerChangeZ; // UV scroll velocity (world units/sec)
    uint16_t angleChange;  // rotation velocity (binary radians/sec, cast to int16_t for sign)
};
static_assert(sizeof(MedMoCellMotion) == 14, "MedMoCellMotion must be 14 bytes");
#pragma pack(pop)

// Parsed flow data for the mission — texture mapping and animation velocities.
struct FlowData {
    FlowTexEntry      textures[MAX_FLOW_GROUPS];
    PortalCellMotion  state[MAX_FLOW_GROUPS];     // current animation state
    MedMoCellMotion   velocity[MAX_FLOW_GROUPS];  // animation velocities (per second)
    bool hasFlowTex    = false;  // true if FLOW_TEX chunk was found
    bool hasCellMotion = false;  // true if CELL_MOTION chunk was found
};

// ── Build water surface mesh from portal polygons ──
// Water portals live at indices [numSolid..numPolygons) with flags != 0.
// Only emit from air cells (mediaType==1) to avoid double-rendering — the
// same portal polygon exists in both the air and water cell.
// Texture source priority:
//   1. FLOW_TEX chunk (via cell.flowGroup) — canonical water texture source
//   2. Polygon texturing data (fallback if no FLOW_TEX or flowGroup==0)
//   3. Flat blue-green vertex color (if no texture available)

static WorldMesh buildWaterMesh(const Darkness::WRParsedData &wr,
                                const std::unordered_map<uint8_t, TexDimensions> &texDims,
                                const FlowData &flowData,
                                const std::unordered_map<uint8_t, TexDimensions> &flowTexDims) {
    WorldMesh mesh;
    // Semi-transparent dark blue-green — fallback for non-textured water portals
    uint32_t waterColor = packABGR(0.2f, 0.32f, 0.4f, 0.35f);
    // Semi-transparent white — lets texture colors show through for textured water
    uint32_t texWaterColor = packABGR(1.0f, 1.0f, 1.0f, 0.35f);
    int waterPolyCount = 0;

    // Composite key for grouping triangles by texture source.
    // Flow-textured water uses 0x8000 | flowGroup (high bit set).
    // TXLIST-textured water uses (txtIndex << 8) | flowGroup.
    // This ensures each texture source gets its own draw group.
    std::unordered_map<uint16_t, std::vector<uint32_t>> texTriangles;
    // Track flow group and texture index per composite key
    std::unordered_map<uint16_t, uint8_t> keyFlowGroup;
    std::unordered_map<uint16_t, uint8_t> keyTxtIndex;

    for (uint32_t ci = 0; ci < wr.numCells; ++ci) {
        const auto &cell = wr.cells[ci];
        // Only emit from air cells to avoid double-rendering at portal boundaries
        if (cell.mediaType != 1) continue;
        int numSolid = cell.numPolygons - cell.numPortals;

        for (int pi = numSolid; pi < cell.numPolygons; ++pi) {
            const auto &poly = cell.polygons[pi];
            const auto &indices = cell.polyIndices[pi];
            if (poly.count < 3) continue;
            // Water portal: flags != 0 (non-water portals have flags == 0)
            if (poly.flags == 0) continue;

            // Resolve flow group: water portals connect an air cell to a water cell.
            // The flowGroup is on the water cell (target), not the air cell (source).
            uint8_t cellFlowGroup = 0;
            bool hasFlowTex = false;
            if (poly.tgtCell < wr.numCells) {
                cellFlowGroup = wr.cells[poly.tgtCell].flowGroup;
                if (cellFlowGroup > 0) {
                    hasFlowTex = flowTexDims.count(cellFlowGroup) > 0;
                }
            }

            // Determine texture: prefer flow texture (loaded by name from fam.crf),
            // then polygon texturing from TXLIST, then flat color.
            bool hasPolyTex = (pi < cell.numTextured);
            uint8_t polyTxtIdx = 0;
            if (hasPolyTex) {
                polyTxtIdx = cell.texturing[pi].txt;
                if (polyTxtIdx == 0 || polyTxtIdx == 249)
                    hasPolyTex = false;
            }

            // Flow texture takes priority — polygon texture is fallback
            uint8_t txtIdx = hasFlowTex ? 0 : polyTxtIdx;
            bool isTextured = hasFlowTex || (txtIdx > 0);

            float texW = 64.0f, texH = 64.0f;
            if (hasFlowTex) {
                // Dimensions from flow texture loaded by name from fam.crf
                auto fit = flowTexDims.find(cellFlowGroup);
                if (fit != flowTexDims.end()) {
                    texW = static_cast<float>(fit->second.w);
                    texH = static_cast<float>(fit->second.h);
                }
            } else if (isTextured) {
                auto it = texDims.find(txtIdx);
                if (it != texDims.end()) {
                    texW = static_cast<float>(it->second.w);
                    texH = static_cast<float>(it->second.h);
                }
            }

            uint32_t baseVertex = static_cast<uint32_t>(mesh.vertices.size());

            if (isTextured && hasPolyTex) {
                // Textured water with polygon UV data — use projected UVs
                const auto &tex = cell.texturing[pi];
                Darkness::Vector3 origin(0, 0, 0);
                if (tex.originVertex < indices.size()) {
                    uint8_t oi = indices[tex.originVertex];
                    if (oi < cell.vertices.size())
                        origin = cell.vertices[oi];
                }

                float mag2_u = tex.axisU.squaredLength();
                float mag2_v = tex.axisV.squaredLength();
                float sh_u = tex.u / 4096.0f;
                float sh_v = tex.v / 4096.0f;
                float dotp = tex.axisU.dotProduct(tex.axisV);

                for (int vi = 0; vi < poly.count; ++vi) {
                    uint8_t idx = indices[vi];
                    if (idx >= cell.vertices.size()) continue;
                    const auto &coord = cell.vertices[idx];

                    Darkness::Vector3 tmp = coord - origin;
                    float u, v;

                    if (std::abs(dotp) < 1e-6f) {
                        u = tex.axisU.dotProduct(tmp) / mag2_u + sh_u;
                        v = tex.axisV.dotProduct(tmp) / mag2_v + sh_v;
                    } else {
                        float corr = 1.0f / (mag2_u * mag2_v - dotp * dotp);
                        float cu = corr * mag2_v;
                        float cv = corr * mag2_u;
                        float cross = corr * dotp;
                        float pu = tex.axisU.dotProduct(tmp);
                        float pv = tex.axisV.dotProduct(tmp);
                        u = pu * cu - pv * cross + sh_u;
                        v = pv * cv - pu * cross + sh_v;
                    }

                    // Normalize: projection is in 64-texel units, scale to 0..1 UV
                    u /= (texW / 64.0f);
                    v /= (texH / 64.0f);

                    mesh.vertices.push_back({coord.x, coord.y, coord.z, texWaterColor, u, v});
                }
            } else if (isTextured) {
                // Textured water from FLOW_TEX but no polygon UV data —
                // compute planar UVs by projecting onto the water plane.
                // Dark Engine uses 4 world units per texture repeat (TEXTURE_UNIT_LENGTH).
                constexpr float TEX_SCALE = 4.0f;
                for (int vi = 0; vi < poly.count; ++vi) {
                    uint8_t idx = indices[vi];
                    if (idx >= cell.vertices.size()) continue;
                    const auto &coord = cell.vertices[idx];

                    // Project position onto the two axes perpendicular to the
                    // water plane. Most water is Z-up (horizontal), so use X,Y.
                    float u = coord.x / (texW * TEX_SCALE / 64.0f);
                    float v = coord.y / (texH * TEX_SCALE / 64.0f);

                    mesh.vertices.push_back({coord.x, coord.y, coord.z, texWaterColor, u, v});
                }
            } else {
                // Non-textured water portal: flat color, no UVs
                for (int vi = 0; vi < poly.count; ++vi) {
                    uint8_t idx = indices[vi];
                    if (idx >= cell.vertices.size()) continue;
                    const auto &v = cell.vertices[idx];
                    mesh.vertices.push_back({v.x, v.y, v.z, waterColor, 0.0f, 0.0f});
                }
            }

            // Fan triangulation: same winding as buildFlatMesh
            // Composite key: flow-textured uses 0x8000|flowGroup, TXLIST-textured
            // uses (txtIdx<<8)|flowGroup to avoid key collisions.
            uint16_t key;
            if (hasFlowTex) {
                key = 0x8000 | static_cast<uint16_t>(cellFlowGroup);
            } else {
                key = (static_cast<uint16_t>(txtIdx) << 8) | cellFlowGroup;
            }
            keyFlowGroup[key] = cellFlowGroup;
            keyTxtIndex[key] = txtIdx;
            auto &triList = texTriangles[key];
            for (int t = 1; t < poly.count - 1; ++t) {
                triList.push_back(baseVertex);
                triList.push_back(baseVertex + t + 1);
                triList.push_back(baseVertex + t);
            }
            ++waterPolyCount;
        }
    }

    // Build sorted index buffer with texture groups
    std::vector<uint16_t> sortedKeys;
    for (auto &kv : texTriangles)
        sortedKeys.push_back(kv.first);
    std::sort(sortedKeys.begin(), sortedKeys.end());

    for (uint16_t key : sortedKeys) {
        auto &triList = texTriangles[key];
        CellTextureGroup grp;
        grp.cellID = 0; // water mesh is not portal-culled
        grp.txtIndex = keyTxtIndex[key];
        grp.flowGroup = keyFlowGroup[key];
        grp.firstIndex = static_cast<uint32_t>(mesh.indices.size());
        grp.numIndices = static_cast<uint32_t>(triList.size());
        mesh.groups.push_back(grp);
        mesh.indices.insert(mesh.indices.end(), triList.begin(), triList.end());
    }

    mesh.cx = mesh.cy = mesh.cz = 0;
    // Count flow-textured vs TXLIST-textured vs flat water groups
    int flowTexGroups = 0, txlistTexGroups = 0, flatGroups = 0;
    for (const auto &g : mesh.groups) {
        if (g.flowGroup > 0) ++flowTexGroups;
        else if (g.txtIndex > 0) ++txlistTexGroups;
        else ++flatGroups;
    }
    std::fprintf(stderr, "Water mesh: %d polygons, %zu vertices, %zu indices, %zu groups"
                 " (%d flow-textured, %d txlist, %d flat)\n",
                 waterPolyCount, mesh.vertices.size(), mesh.indices.size(), mesh.groups.size(),
                 flowTexGroups, txlistTexGroups, flatGroups);
    return mesh;
}

// ── Build textured mesh with UV coordinates ──

static WorldMesh buildTexturedMesh(const Darkness::WRParsedData &wr,
                                   const std::unordered_map<uint8_t, TexDimensions> &texDims) {
    WorldMesh mesh;

    // Sun direction for Lambertian shading
    float sunDir[3] = { 0.3f, 0.8f, 0.4f };
    float sunLen = std::sqrt(sunDir[0]*sunDir[0] + sunDir[1]*sunDir[1] + sunDir[2]*sunDir[2]);
    sunDir[0] /= sunLen; sunDir[1] /= sunLen; sunDir[2] /= sunLen;

    float sumX = 0, sumY = 0, sumZ = 0;
    int cellCount = 0;

    // Temporary per-(cell, texture) triangle lists for portal culling support
    // Key: (cellID << 8) | txtIndex — cell in high bits for sort order
    std::map<uint64_t, std::vector<uint32_t>> cellTexTriangles;

    for (uint32_t ci = 0; ci < wr.numCells; ++ci) {
        const auto &cell = wr.cells[ci];

        sumX += cell.center.x;
        sumY += cell.center.y;
        sumZ += cell.center.z;
        ++cellCount;

        int numSolid = cell.numPolygons - cell.numPortals;

        for (int pi = 0; pi < numSolid; ++pi) {
            const auto &poly = cell.polygons[pi];
            const auto &indices = cell.polyIndices[pi];

            if (poly.count < 3) continue;

            Darkness::Plane plane;
            if (poly.plane < cell.planes.size())
                plane = cell.planes[poly.plane];

            Darkness::Vector3 n = plane.normal.normalisedCopy();
            float dot = n.x * sunDir[0] + n.y * sunDir[1] + n.z * sunDir[2];
            float brightness = std::max(dot, 0.0f) * 0.85f + 0.15f;

            // Determine if this polygon is textured
            bool isTextured = (pi < cell.numTextured);
            uint8_t txtIdx = 0;
            float texW = 64.0f, texH = 64.0f;

            if (isTextured) {
                const auto &tex = cell.texturing[pi];
                txtIdx = tex.txt;

                // Skip sky polygons entirely — they become holes for the sky dome
                if (txtIdx == 249)
                    continue;

                // Null texture falls through to flat shading
                if (txtIdx == 0)
                    isTextured = false;
            }

            if (isTextured) {
                auto it = texDims.find(txtIdx);
                if (it != texDims.end()) {
                    texW = static_cast<float>(it->second.w);
                    texH = static_cast<float>(it->second.h);
                }
            }

            // Neutral white lighting for textured, warm tint for flat
            uint32_t color;
            if (isTextured) {
                color = packABGR(brightness, brightness, brightness);
            } else if (cell.mediaType == 2) {
                color = packABGR(brightness * 0.4f, brightness * 0.5f, brightness * 0.8f);
            } else {
                color = packABGR(brightness * 0.75f, brightness * 0.65f, brightness * 0.5f);
            }

            uint32_t baseVertex = static_cast<uint32_t>(mesh.vertices.size());

            // UV computation (ref: WRCell.cpp:145-212)
            if (isTextured) {
                const auto &tex = cell.texturing[pi];
                Darkness::Vector3 origin(0, 0, 0);
                if (tex.originVertex < indices.size()) {
                    uint8_t oi = indices[tex.originVertex];
                    if (oi < cell.vertices.size())
                        origin = cell.vertices[oi];
                }

                float mag2_u = tex.axisU.squaredLength();
                float mag2_v = tex.axisV.squaredLength();
                float sh_u = tex.u / 4096.0f;
                float sh_v = tex.v / 4096.0f;
                float dotp = tex.axisU.dotProduct(tex.axisV);

                for (int vi = 0; vi < poly.count; ++vi) {
                    uint8_t idx = indices[vi];
                    if (idx >= cell.vertices.size()) continue;
                    const auto &coord = cell.vertices[idx];

                    Darkness::Vector3 tmp = coord - origin;
                    float u, v;

                    if (std::abs(dotp) < 1e-6f) {
                        // Orthogonal axes — direct projection
                        u = tex.axisU.dotProduct(tmp) / mag2_u + sh_u;
                        v = tex.axisV.dotProduct(tmp) / mag2_v + sh_v;
                    } else {
                        // Non-orthogonal correction (WRCell.cpp:196-203)
                        float corr = 1.0f / (mag2_u * mag2_v - dotp * dotp);
                        float cu = corr * mag2_v;
                        float cv = corr * mag2_u;
                        float cross = corr * dotp;
                        float pu = tex.axisU.dotProduct(tmp);
                        float pv = tex.axisV.dotProduct(tmp);
                        u = pu * cu - pv * cross + sh_u;
                        v = pv * cv - pu * cross + sh_v;
                    }

                    // Normalize: projection is in 64-texel units, scale to 0..1 UV
                    u /= (texW / 64.0f);
                    v /= (texH / 64.0f);

                    mesh.vertices.push_back({coord.x, coord.y, coord.z, color, u, v});
                }
            } else {
                for (int vi = 0; vi < poly.count; ++vi) {
                    uint8_t idx = indices[vi];
                    if (idx >= cell.vertices.size()) continue;
                    const auto &coord = cell.vertices[idx];
                    mesh.vertices.push_back({coord.x, coord.y, coord.z, color, 0.0f, 0.0f});
                }
            }

            // Fan-triangulate, grouped by (cellID, txtIndex) for portal culling
            uint8_t key = isTextured ? txtIdx : 0; // 0 = flat group
            uint64_t cellTexKey = (static_cast<uint64_t>(ci) << 8) | key;
            auto &triList = cellTexTriangles[cellTexKey];
            for (int t = 1; t < poly.count - 1; ++t) {
                triList.push_back(baseVertex);
                triList.push_back(baseVertex + t + 1);
                triList.push_back(baseVertex + t);
            }
        }
    }

    // Build sorted index buffer: groups sorted by (cellID, txtIndex)
    // std::map is already sorted by key
    for (auto &kv : cellTexTriangles) {
        uint32_t cellID = static_cast<uint32_t>(kv.first >> 8);
        uint8_t txtIdx = static_cast<uint8_t>(kv.first & 0xFF);
        CellTextureGroup grp;
        grp.cellID = cellID;
        grp.txtIndex = txtIdx;
        grp.firstIndex = static_cast<uint32_t>(mesh.indices.size());
        grp.numIndices = static_cast<uint32_t>(kv.second.size());
        mesh.groups.push_back(grp);
        mesh.indices.insert(mesh.indices.end(), kv.second.begin(), kv.second.end());
    }

    if (cellCount > 0) {
        mesh.cx = sumX / cellCount;
        mesh.cy = sumY / cellCount;
        mesh.cz = sumZ / cellCount;
    } else {
        mesh.cx = mesh.cy = mesh.cz = 0;
    }

    return mesh;
}

// ── Build lightmapped mesh with dual UV channels ──

static LightmappedMesh buildLightmappedMesh(
    const Darkness::WRParsedData &wr,
    const std::unordered_map<uint8_t, TexDimensions> &texDims,
    const Darkness::LightmapAtlasSet &lmAtlas)
{
    LightmappedMesh mesh;

    float sumX = 0, sumY = 0, sumZ = 0;
    int cellCount = 0;

    // Temporary per-(cell, texture) triangle lists for portal culling support
    std::map<uint64_t, std::vector<uint32_t>> cellTexTriangles;

    auto findWrap = [](float x) -> float {
        if (x >= 0) return -64.0f * static_cast<int>(x / 64.0f);
        else return -64.0f * (-1 + static_cast<int>(x / 64.0f));
    };

    for (uint32_t ci = 0; ci < wr.numCells; ++ci) {
        const auto &cell = wr.cells[ci];

        sumX += cell.center.x;
        sumY += cell.center.y;
        sumZ += cell.center.z;
        ++cellCount;

        int numSolid = cell.numPolygons - cell.numPortals;

        for (int pi = 0; pi < numSolid; ++pi) {
            const auto &poly = cell.polygons[pi];
            const auto &indices = cell.polyIndices[pi];

            if (poly.count < 3) continue;

            bool isTextured = (pi < cell.numTextured);
            uint8_t txtIdx = 0;
            float texW = 64.0f, texH = 64.0f;

            if (isTextured) {
                const auto &tex = cell.texturing[pi];
                txtIdx = tex.txt;

                // Skip sky polygons entirely — they become holes for the sky dome
                if (txtIdx == 249)
                    continue;

                // Null texture falls through to flat shading
                if (txtIdx == 0)
                    isTextured = false;
            }

            if (isTextured) {
                auto it = texDims.find(txtIdx);
                if (it != texDims.end()) {
                    texW = static_cast<float>(it->second.w);
                    texH = static_cast<float>(it->second.h);
                }
            }

            // Check if this polygon has lightmap data
            bool hasLightmap = isTextured && pi < static_cast<int>(cell.lightInfos.size())
                && cell.lightInfos[pi].lx > 0 && cell.lightInfos[pi].ly > 0
                && ci < lmAtlas.entries.size()
                && pi < static_cast<int>(lmAtlas.entries[ci].size());

            uint32_t baseVertex = static_cast<uint32_t>(mesh.vertices.size());

            if (isTextured) {
                const auto &tex = cell.texturing[pi];
                Darkness::Vector3 origin(0, 0, 0);
                if (tex.originVertex < indices.size()) {
                    uint8_t oi = indices[tex.originVertex];
                    if (oi < cell.vertices.size())
                        origin = cell.vertices[oi];
                }

                float mag2_u = tex.axisU.squaredLength();
                float mag2_v = tex.axisV.squaredLength();
                float sh_u = tex.u / 4096.0f;
                float sh_v = tex.v / 4096.0f;
                float dotp = tex.axisU.dotProduct(tex.axisV);

                // Lightmap shift and entry (if available)
                const Darkness::WRLightInfo *li = nullptr;
                const Darkness::LmapEntry *lmEntry = nullptr;
                float lsh_u = 0, lsh_v = 0;

                if (hasLightmap) {
                    li = &cell.lightInfos[pi];
                    lmEntry = &lmAtlas.entries[ci][pi];
                    lsh_u = (0.5f - li->u) + tex.u / 1024.0f;
                    lsh_v = (0.5f - li->v) + tex.v / 1024.0f;
                }

                // First pass: compute raw lightmap UVs to find min for wrapping
                struct VertexData {
                    float x, y, z;
                    float diffU, diffV;
                    float lmU, lmV;
                };
                std::vector<VertexData> verts(poly.count);

                for (int vi = 0; vi < poly.count; ++vi) {
                    uint8_t idx = indices[vi];
                    if (idx >= cell.vertices.size()) { verts[vi] = {}; continue; }
                    const auto &coord = cell.vertices[idx];
                    verts[vi].x = coord.x;
                    verts[vi].y = coord.y;
                    verts[vi].z = coord.z;

                    Darkness::Vector3 tmp = coord - origin;
                    float pu = tex.axisU.dotProduct(tmp);
                    float pv = tex.axisV.dotProduct(tmp);
                    float projU, projV;

                    if (std::abs(dotp) < 1e-6f) {
                        projU = pu / mag2_u;
                        projV = pv / mag2_v;
                    } else {
                        float corr = 1.0f / (mag2_u * mag2_v - dotp * dotp);
                        float cu = corr * mag2_v;
                        float cv = corr * mag2_u;
                        float cross = corr * dotp;
                        projU = pu * cu - pv * cross;
                        projV = pv * cv - pu * cross;
                    }

                    // Diffuse UV
                    verts[vi].diffU = (projU + sh_u) / (texW / 64.0f);
                    verts[vi].diffV = (projV + sh_v) / (texH / 64.0f);

                    // Lightmap UV (raw, before wrapping)
                    if (hasLightmap) {
                        verts[vi].lmU = 4.0f * projU + lsh_u;
                        verts[vi].lmV = 4.0f * projV + lsh_v;
                    }
                }

                if (hasLightmap) {
                    // Find min lightmap UV for wrap correction
                    float minLmU = verts[0].lmU, minLmV = verts[0].lmV;
                    for (int vi = 1; vi < poly.count; ++vi) {
                        if (verts[vi].lmU < minLmU) minLmU = verts[vi].lmU;
                        if (verts[vi].lmV < minLmV) minLmV = verts[vi].lmV;
                    }

                    float wrapU = findWrap(minLmU);
                    float wrapV = findWrap(minLmV);

                    float invLx = 1.0f / static_cast<float>(li->lx);
                    float invLy = 1.0f / static_cast<float>(li->ly);

                    for (int vi = 0; vi < poly.count; ++vi) {
                        float lmU = (verts[vi].lmU + wrapU) * invLx;
                        float lmV = (verts[vi].lmV + wrapV) * invLy;

                        // Remap to atlas coordinates
                        float atlasU = lmU * lmEntry->atlasSU + lmEntry->atlasU;
                        float atlasV = lmV * lmEntry->atlasSV + lmEntry->atlasV;

                        mesh.vertices.push_back({
                            verts[vi].x, verts[vi].y, verts[vi].z,
                            verts[vi].diffU, verts[vi].diffV,
                            atlasU, atlasV
                        });
                    }
                } else {
                    // No lightmap — point at white fallback pixel in atlas
                    const Darkness::LmapEntry &fallback = (ci < lmAtlas.entries.size() && !lmAtlas.entries[ci].empty())
                        ? lmAtlas.entries[ci][0] : Darkness::LmapEntry{0, 0, 0, 0, 0, 0, 0, 0, 0};

                    for (int vi = 0; vi < poly.count; ++vi) {
                        mesh.vertices.push_back({
                            verts[vi].x, verts[vi].y, verts[vi].z,
                            verts[vi].diffU, verts[vi].diffV,
                            fallback.atlasU, fallback.atlasV
                        });
                    }
                }
            } else {
                // Non-textured polygon — flat fallback
                for (int vi = 0; vi < poly.count; ++vi) {
                    uint8_t idx = indices[vi];
                    if (idx >= cell.vertices.size()) continue;
                    const auto &coord = cell.vertices[idx];
                    mesh.vertices.push_back({coord.x, coord.y, coord.z, 0.0f, 0.0f, 0.0f, 0.0f});
                }
            }

            // Fan-triangulate, grouped by (cellID, txtIndex) for portal culling
            uint8_t key = isTextured ? txtIdx : 0;
            uint64_t cellTexKey = (static_cast<uint64_t>(ci) << 8) | key;
            auto &triList = cellTexTriangles[cellTexKey];
            for (int t = 1; t < poly.count - 1; ++t) {
                triList.push_back(baseVertex);
                triList.push_back(baseVertex + t + 1);
                triList.push_back(baseVertex + t);
            }
        }
    }

    // Build sorted index buffer: groups sorted by (cellID, txtIndex)
    for (auto &kv : cellTexTriangles) {
        uint32_t cellID = static_cast<uint32_t>(kv.first >> 8);
        uint8_t txtIdx = static_cast<uint8_t>(kv.first & 0xFF);
        CellTextureGroup grp;
        grp.cellID = cellID;
        grp.txtIndex = txtIdx;
        grp.firstIndex = static_cast<uint32_t>(mesh.indices.size());
        grp.numIndices = static_cast<uint32_t>(kv.second.size());
        mesh.groups.push_back(grp);
        mesh.indices.insert(mesh.indices.end(), kv.second.begin(), kv.second.end());
    }

    if (cellCount > 0) {
        mesh.cx = sumX / cellCount;
        mesh.cy = sumY / cellCount;
        mesh.cz = sumZ / cellCount;
    } else {
        mesh.cx = mesh.cy = mesh.cz = 0;
    }

    return mesh;
}

// ── Object mesh rendering ──

// Per-material submesh draw range within an object model's index buffer
struct ObjectSubMeshGPU {
    uint32_t firstIndex;
    uint32_t indexCount;
    std::string matName;   // lowercase material name, for texture lookup
    bool textured;         // true = Darkness::MD_MAT_TMAP, false = Darkness::MD_MAT_COLOR
};

// GPU buffers for a single unique model (shared by all instances)
struct ObjectModelGPU {
    bgfx::VertexBufferHandle vbh = BGFX_INVALID_HANDLE;
    bgfx::IndexBufferHandle ibh = BGFX_INVALID_HANDLE;
    std::vector<ObjectSubMeshGPU> subMeshes;
    bool valid = false;
};

// Generate a stable colour from a model name hash (for flat-shaded objects)
static uint32_t colorFromName(const std::string &name) {
    // Simple hash -> hue mapping for distinct per-model colours
    uint32_t hash = 5381;
    for (char c : name) hash = hash * 33 + static_cast<uint8_t>(c);

    // HSV -> RGB with S=0.6, V=0.85 for pleasant pastel tones
    float hue = static_cast<float>(hash % 360);
    float s = 0.6f, v = 0.85f;
    float c = v * s;
    float x = c * (1.0f - std::fabs(std::fmod(hue / 60.0f, 2.0f) - 1.0f));
    float m = v - c;
    float r, g, b;
    if (hue < 60)       { r = c; g = x; b = 0; }
    else if (hue < 120) { r = x; g = c; b = 0; }
    else if (hue < 180) { r = 0; g = c; b = x; }
    else if (hue < 240) { r = 0; g = x; b = c; }
    else if (hue < 300) { r = x; g = 0; b = c; }
    else                { r = c; g = 0; b = x; }
    return packABGR(r + m, g + m, b + m);
}

// Build a small unit cube as PosColorVertex (fallback for missing models)
static void buildFallbackCube(std::vector<PosColorVertex> &verts,
                               std::vector<uint32_t> &indices,
                               uint32_t color) {
    // 8 vertices of a unit cube centered at origin (±0.5)
    const float h = 0.5f;
    const float p[8][3] = {
        {-h,-h,-h}, { h,-h,-h}, { h, h,-h}, {-h, h,-h},
        {-h,-h, h}, { h,-h, h}, { h, h, h}, {-h, h, h}
    };
    uint32_t base = static_cast<uint32_t>(verts.size());
    for (int i = 0; i < 8; ++i) {
        verts.push_back({p[i][0], p[i][1], p[i][2], color});
    }
    // 12 triangles (6 faces x 2)
    const uint32_t idx[36] = {
        0,2,1, 0,3,2, // front (-Z)
        4,5,6, 4,6,7, // back (+Z)
        0,1,5, 0,5,4, // bottom
        2,3,7, 2,7,6, // top
        0,4,7, 0,7,3, // left
        1,2,6, 1,6,5  // right
    };
    for (int i = 0; i < 36; ++i) {
        indices.push_back(base + idx[i]);
    }
}

// ── Camera-to-cell lookup ──
// Find which WR cell the camera is currently inside using a proper
// point-in-convex-cell test against each cell's bounding planes.
// ── Camera cell detection ──
// Find which cell the camera is in. Returns cell index, or -1 if outside
// all cells. When the camera is near a water boundary, it may be inside
// both an air and water cell — we prefer the smallest containing cell.

static int32_t findCameraCell(const Darkness::WRParsedData &wr,
                               float cx, float cy, float cz) {
    Darkness::Vector3 pt(cx, cy, cz);
    float bestRadius = 1e30f;
    int32_t bestCell = -1;

    for (uint32_t i = 0; i < wr.numCells; ++i) {
        const auto &cell = wr.cells[i];

        // Quick reject: skip cells whose bounding sphere doesn't contain the point
        float dx = cx - cell.center.x;
        float dy = cy - cell.center.y;
        float dz = cz - cell.center.z;
        float dist2 = dx*dx + dy*dy + dz*dz;
        if (dist2 > cell.radius * cell.radius)
            continue;

        // Precise test: point must be on the positive side of all cell planes.
        // Dark Engine convention: cell planes face inward, so inside = positive
        // distance (normal · point + d > 0). Small negative epsilon for boundary.
        bool inside = true;
        for (const auto &plane : cell.planes) {
            if (plane.getDistance(pt) < -0.1f) {
                inside = false;
                break;
            }
        }

        if (inside) {
            // Prefer the smallest containing cell — water cells are typically
            // smaller than the adjacent air cells they border.
            if (cell.radius < bestRadius) {
                bestRadius = cell.radius;
                bestCell = static_cast<int32_t>(i);
            }
        }
    }

    return bestCell;
}

// Convenience wrapper: returns mediaType (1=air, 2=water) for underwater detection
static uint8_t getCameraMediaType(const Darkness::WRParsedData &wr,
                                   float cx, float cy, float cz) {
    int32_t cell = findCameraCell(wr, cx, cy, cz);
    if (cell >= 0 && cell < static_cast<int32_t>(wr.numCells))
        return wr.cells[cell].mediaType;
    return 1; // default: air
}

// ── Portal culling infrastructure ──

// Portal connectivity info for BFS traversal
struct PortalInfo {
    uint32_t tgtCell;           // destination cell index
    Darkness::Plane plane;          // portal plane (for backface cull)
    // Portal polygon AABB (for frustum test)
    float minX, minY, minZ;
    float maxX, maxY, maxZ;
};

// Build portal adjacency graph from WR data.
// cellPortals[cellID] = list of portals leading to other cells.
static std::vector<std::vector<PortalInfo>>
buildPortalGraph(const Darkness::WRParsedData &wr) {
    std::vector<std::vector<PortalInfo>> cellPortals(wr.numCells);

    for (uint32_t ci = 0; ci < wr.numCells; ++ci) {
        const auto &cell = wr.cells[ci];
        int numSolid = cell.numPolygons - cell.numPortals;

        for (int pi = numSolid; pi < cell.numPolygons; ++pi) {
            const auto &poly = cell.polygons[pi];
            const auto &indices = cell.polyIndices[pi];

            if (poly.count < 3) continue;
            if (poly.tgtCell >= wr.numCells) continue;

            PortalInfo portal;
            portal.tgtCell = poly.tgtCell;

            // Get portal plane from the polygon's plane reference
            if (poly.plane < cell.planes.size()) {
                portal.plane = cell.planes[poly.plane];
            }

            // Compute AABB of portal polygon vertices
            portal.minX = portal.minY = portal.minZ =  1e30f;
            portal.maxX = portal.maxY = portal.maxZ = -1e30f;

            for (int vi = 0; vi < poly.count; ++vi) {
                uint8_t idx = indices[vi];
                if (idx >= cell.vertices.size()) continue;
                const auto &v = cell.vertices[idx];
                portal.minX = std::min(portal.minX, v.x);
                portal.minY = std::min(portal.minY, v.y);
                portal.minZ = std::min(portal.minZ, v.z);
                portal.maxX = std::max(portal.maxX, v.x);
                portal.maxY = std::max(portal.maxY, v.y);
                portal.maxZ = std::max(portal.maxZ, v.z);
            }

            cellPortals[ci].push_back(portal);
        }
    }

    return cellPortals;
}

// ── View frustum for portal culling ──
// 6 planes extracted from the view-projection matrix (VP = view * proj).
// Each plane stored as (a, b, c, d) where ax + by + cz + d >= 0 means inside.

struct ViewFrustum {
    float planes[6][4]; // left, right, bottom, top, near, far

    // Extract frustum planes from a combined view-projection matrix.
    // Standard technique: each plane is a sum/difference of VP matrix rows.
    // Matrix is in row-major order (bx convention).
    void extractFromVP(const float *vp) {
        // Left: row3 + row0
        planes[0][0] = vp[ 3] + vp[ 0];
        planes[0][1] = vp[ 7] + vp[ 4];
        planes[0][2] = vp[11] + vp[ 8];
        planes[0][3] = vp[15] + vp[12];
        // Right: row3 - row0
        planes[1][0] = vp[ 3] - vp[ 0];
        planes[1][1] = vp[ 7] - vp[ 4];
        planes[1][2] = vp[11] - vp[ 8];
        planes[1][3] = vp[15] - vp[12];
        // Bottom: row3 + row1
        planes[2][0] = vp[ 3] + vp[ 1];
        planes[2][1] = vp[ 7] + vp[ 5];
        planes[2][2] = vp[11] + vp[ 9];
        planes[2][3] = vp[15] + vp[13];
        // Top: row3 - row1
        planes[3][0] = vp[ 3] - vp[ 1];
        planes[3][1] = vp[ 7] - vp[ 5];
        planes[3][2] = vp[11] - vp[ 9];
        planes[3][3] = vp[15] - vp[13];
        // Near: row3 + row2
        planes[4][0] = vp[ 3] + vp[ 2];
        planes[4][1] = vp[ 7] + vp[ 6];
        planes[4][2] = vp[11] + vp[10];
        planes[4][3] = vp[15] + vp[14];
        // Far: row3 - row2
        planes[5][0] = vp[ 3] - vp[ 2];
        planes[5][1] = vp[ 7] - vp[ 6];
        planes[5][2] = vp[11] - vp[10];
        planes[5][3] = vp[15] - vp[14];

        // Normalize each plane
        for (int i = 0; i < 6; ++i) {
            float len = std::sqrt(planes[i][0]*planes[i][0] +
                                  planes[i][1]*planes[i][1] +
                                  planes[i][2]*planes[i][2]);
            if (len > 1e-8f) {
                float inv = 1.0f / len;
                planes[i][0] *= inv;
                planes[i][1] *= inv;
                planes[i][2] *= inv;
                planes[i][3] *= inv;
            }
        }
    }

    // Test if an AABB is completely outside any frustum plane.
    // Returns true if the AABB is at least partially inside the frustum.
    // Conservative: may return true for some AABBs that are outside
    // (the "positive vertex" test), but never false for visible AABBs.
    bool testAABB(float minX, float minY, float minZ,
                  float maxX, float maxY, float maxZ) const {
        for (int i = 0; i < 6; ++i) {
            // Find the "positive vertex" — the AABB corner most in the
            // direction of the plane normal
            float px = (planes[i][0] >= 0) ? maxX : minX;
            float py = (planes[i][1] >= 0) ? maxY : minY;
            float pz = (planes[i][2] >= 0) ? maxZ : minZ;
            float dist = planes[i][0]*px + planes[i][1]*py +
                         planes[i][2]*pz + planes[i][3];
            if (dist < 0.0f)
                return false; // entirely outside this plane
        }
        return true;
    }
};

// BFS portal traversal: starting from the camera cell, follow portals
// that are visible to the view frustum. Returns the set of visible cell IDs.
static std::unordered_set<uint32_t>
portalBFS(const Darkness::WRParsedData &wr,
          const std::vector<std::vector<PortalInfo>> &cellPortals,
          int32_t startCell,
          const ViewFrustum &frustum,
          float camX, float camY, float camZ) {
    std::unordered_set<uint32_t> visible;

    if (startCell < 0 || startCell >= static_cast<int32_t>(wr.numCells))
        return visible;

    std::queue<uint32_t> queue;
    queue.push(static_cast<uint32_t>(startCell));
    visible.insert(static_cast<uint32_t>(startCell));

    Darkness::Vector3 camPos(camX, camY, camZ);

    while (!queue.empty()) {
        uint32_t ci = queue.front();
        queue.pop();

        if (ci >= cellPortals.size()) continue;

        for (const auto &portal : cellPortals[ci]) {
            // Already visited?
            if (visible.count(portal.tgtCell))
                continue;

            // Backface test: skip portals facing away from the camera.
            // Camera must be on the positive side of the portal plane
            // (Dark Engine cell planes face inward).
            float camDist = portal.plane.getDistance(camPos);
            if (camDist < -0.1f)
                continue;

            // Frustum test: check if portal AABB intersects view frustum
            if (!frustum.testAABB(portal.minX, portal.minY, portal.minZ,
                                  portal.maxX, portal.maxY, portal.maxZ))
                continue;

            // Portal is visible — add target cell and continue traversal
            visible.insert(portal.tgtCell);
            queue.push(portal.tgtCell);
        }
    }

    return visible;
}

// ── Fog parameters ──

// Global mission fog — parsed from the FOG chunk in .mis files.
// Linear distance fog: fully transparent at camera, fully opaque at 'distance'.
struct FogParams {
    bool enabled;           // true if FOG chunk present and distance > 0
    float r, g, b;          // fog color in 0..1 range
    float distance;         // distance at which fog is fully opaque
};

// Parse the FOG chunk from a .mis file if present.
// Format: DarkDBChunkFOG = { int32 red, int32 green, int32 blue, float distance }
static FogParams parseFogChunk(const char *misPath) {
    FogParams fog = { false, 0.0f, 0.0f, 0.0f, 1.0f };

    try {
        Darkness::FilePtr fp(new Darkness::StdFile(misPath, Darkness::File::FILE_R));
        Darkness::FileGroupPtr db(new Darkness::DarkFileGroup(fp));

        if (!db->hasFile("FOG")) {
            std::fprintf(stderr, "No FOG chunk — fog disabled\n");
            return fog;
        }

        Darkness::FilePtr chunk = db->getFile("FOG");

        int32_t red, green, blue;
        float distance;
        chunk->readElem(&red, sizeof(red));
        chunk->readElem(&green, sizeof(green));
        chunk->readElem(&blue, sizeof(blue));
        chunk->readElem(&distance, sizeof(distance));

        if (distance > 0.0f) {
            fog.enabled = true;
            fog.r = static_cast<float>(red) / 255.0f;
            fog.g = static_cast<float>(green) / 255.0f;
            fog.b = static_cast<float>(blue) / 255.0f;
            fog.distance = distance;
            std::fprintf(stderr, "FOG: color=(%d,%d,%d) distance=%.1f\n",
                         red, green, blue, distance);
        } else {
            std::fprintf(stderr, "FOG: chunk present but distance=0 — fog disabled\n");
        }
    } catch (const std::exception &e) {
        std::fprintf(stderr, "Failed to read FOG: %s (fog disabled)\n", e.what());
    }

    return fog;
}

// Parse FLOW_TEX and CELL_MOTION chunks from a .mis file.
// FLOW_TEX: 256 × 32-byte entries mapping flow group to water texture names.
// CELL_MOTION: two sequential arrays — PortalCellMotion[256] (state) then
// MedMoCellMotion[256] (velocities). The velocities give the per-second scroll
// and rotation rates for each flow group, as set by the level designer.
static FlowData parseFlowData(const char *misPath) {
    FlowData data = {};

    try {
        Darkness::FilePtr fp(new Darkness::StdFile(misPath, Darkness::File::FILE_R));
        Darkness::FileGroupPtr db(new Darkness::DarkFileGroup(fp));

        // Parse FLOW_TEX: 256 × 32 bytes = 8192 bytes
        if (db->hasFile("FLOW_TEX")) {
            Darkness::FilePtr chunk = db->getFile("FLOW_TEX");
            for (int i = 0; i < MAX_FLOW_GROUPS; ++i) {
                chunk->readElem(&data.textures[i], sizeof(FlowTexEntry));
            }
            data.hasFlowTex = true;

            // Log flow groups with non-empty texture names
            int namedCount = 0;
            for (int i = 1; i < MAX_FLOW_GROUPS; ++i) {
                if (data.textures[i].name[0] != '\0') {
                    char safeName[29] = {};
                    std::memcpy(safeName, data.textures[i].name, 28);
                    std::fprintf(stderr, "  flow group %d: name='%s' (palette idx in=%d out=%d)\n",
                                 i, safeName, data.textures[i].inTexture, data.textures[i].outTexture);
                    ++namedCount;
                }
            }
            std::fprintf(stderr, "FLOW_TEX: %d named flow groups\n", namedCount);
        } else {
            std::fprintf(stderr, "No FLOW_TEX chunk — water textures from polygon data only\n");
        }

        // Parse CELL_MOTION: PortalCellMotion[256] (21 bytes each) then
        // MedMoCellMotion[256] (14 bytes each) = 256*(21+14) = 8960 bytes
        if (db->hasFile("CELL_MOTION")) {
            Darkness::FilePtr chunk = db->getFile("CELL_MOTION");
            // First array: current state
            for (int i = 0; i < MAX_FLOW_GROUPS; ++i) {
                chunk->readElem(&data.state[i], sizeof(PortalCellMotion));
            }
            // Second array: velocities (scroll + rotation speeds)
            for (int i = 0; i < MAX_FLOW_GROUPS; ++i) {
                chunk->readElem(&data.velocity[i], sizeof(MedMoCellMotion));
            }
            data.hasCellMotion = true;

            // Log flow groups with non-zero state (center or angle set by the level)
            int motionCount = 0;
            for (int i = 1; i < MAX_FLOW_GROUPS; ++i) {
                const auto &st = data.state[i];
                const auto &vel = data.velocity[i];
                bool hasState = (st.centerX != 0 || st.centerY != 0 || st.centerZ != 0 || st.angle != 0);
                bool hasVelocity = (vel.centerChangeX != 0 || vel.centerChangeY != 0 ||
                                    vel.centerChangeZ != 0 || vel.angleChange != 0);
                if (hasState || hasVelocity) {
                    float angDeg = st.angle * 360.0f / 65536.0f;
                    float angVelDeg = static_cast<float>(static_cast<int16_t>(vel.angleChange))
                                      * 360.0f / 65536.0f;
                    std::fprintf(stderr, "  motion %d: center=(%.1f,%.1f,%.1f) angle=%.1f° "
                                 "vel=(%.2f,%.2f,%.2f)/s rot=%.1f°/s axis=%d\n",
                                 i, st.centerX, st.centerY, st.centerZ, angDeg,
                                 vel.centerChangeX, vel.centerChangeY, vel.centerChangeZ,
                                 angVelDeg, st.majorAxis);
                    ++motionCount;
                }
            }
            std::fprintf(stderr, "CELL_MOTION: %d groups with non-zero state/velocity\n", motionCount);
        } else {
            std::fprintf(stderr, "No CELL_MOTION chunk — using default water animation\n");
        }
    } catch (const std::exception &e) {
        std::fprintf(stderr, "Failed to read flow data: %s\n", e.what());
    }

    return data;
}

// ── Sky dome ──

// Sky rendering parameters — either from SKYOBJVAR chunk or defaults
struct SkyParams {
    bool enabled;           // useNewSky from SKYOBJVAR (true if chunk present)
    bool fog;               // whether the sky should be affected by fog
    int numLatPoints;       // latitude rings (default 8)
    int numLonPoints;       // longitude segments (default 24)
    float dipAngle;         // how far below horizon to extend (degrees, default 10)

    // 5-stop vertex colour gradient (pole → dip below horizon)
    // Each stop is {r, g, b} in 0..1 range
    float poleColor[3];           // zenith (straight up)
    float fortyFiveColor[3];      // 45 degrees from pole
    float seventyColor[3];        // 70 degrees from pole
    float horizonColor[3];        // at the horizon (90 degrees)
    float dipColor[3];            // below horizon
};

// Default sky colours — dark blue-grey for Thief 2's nighttime atmosphere
static SkyParams defaultSkyParams() {
    SkyParams p;
    p.enabled = true;
    p.fog = true;  // default: sky affected by fog (overridden by SKYOBJVAR if present)
    p.numLatPoints = 8;
    p.numLonPoints = 24;
    p.dipAngle = 10.0f;

    // Deep blue zenith fading to grey horizon
    p.poleColor[0]       = 0.05f; p.poleColor[1]       = 0.05f; p.poleColor[2]       = 0.20f;
    p.fortyFiveColor[0]  = 0.10f; p.fortyFiveColor[1]  = 0.15f; p.fortyFiveColor[2]  = 0.35f;
    p.seventyColor[0]    = 0.30f; p.seventyColor[1]    = 0.40f; p.seventyColor[2]    = 0.60f;
    p.horizonColor[0]    = 0.50f; p.horizonColor[1]    = 0.55f; p.horizonColor[2]    = 0.60f;
    p.dipColor[0]        = 0.25f; p.dipColor[1]        = 0.25f; p.dipColor[2]        = 0.30f;

    return p;
}

// Parse SKYOBJVAR chunk from a .mis file if present.
// Uses the DarkFileGroup chunk API to read the binary struct directly.
static SkyParams parseSkyObjVar(const char *misPath) {
    SkyParams params = defaultSkyParams();

    try {
        Darkness::FilePtr fp(new Darkness::StdFile(misPath, Darkness::File::FILE_R));
        Darkness::FileGroupPtr db(new Darkness::DarkFileGroup(fp));

        if (!db->hasFile("SKYOBJVAR")) {
            std::fprintf(stderr, "No SKYOBJVAR chunk — using default sky colours\n");
            return params;
        }

        Darkness::FilePtr chunk = db->getFile("SKYOBJVAR");

        // Read the binary struct: matches DarkDBChunkSKYOBJVAR from DarkDBDefs.h
        // Fields: enable(4), fog(4), atmos_radius(4), earth_radius(4),
        //         lat_count(4), lon_count(4), dip_angle(4),
        //         5 × Color(12) = 60, glow_color(12),
        //         glow_lat(4), glow_lon(4), glow_ang(4), glow_scale(4),
        //         glow_method(4), clip_lat(4)
        // Total: 120 bytes
        uint32_t enable, fog;
        float atmosRadius, earthRadius;
        int32_t latCount, lonCount;
        float dipAngle;
        float colors[5][3]; // pole, 45, 70, horizon, dip
        float glowColor[3], glowLat, glowLon, glowAng, glowScale;
        uint32_t glowMethod;
        float clipLat;

        chunk->readElem(&enable, sizeof(enable));
        chunk->readElem(&fog, sizeof(fog));
        chunk->readElem(&atmosRadius, sizeof(atmosRadius));
        chunk->readElem(&earthRadius, sizeof(earthRadius));
        chunk->readElem(&latCount, sizeof(latCount));
        chunk->readElem(&lonCount, sizeof(lonCount));
        chunk->readElem(&dipAngle, sizeof(dipAngle));

        // Read 5 control colours (pole, 45, 70, horizon, dip)
        for (int i = 0; i < 5; ++i) {
            chunk->readElem(&colors[i][0], sizeof(float));
            chunk->readElem(&colors[i][1], sizeof(float));
            chunk->readElem(&colors[i][2], sizeof(float));
        }

        // Read glow parameters (not used yet but consume for completeness)
        chunk->readElem(&glowColor[0], sizeof(float));
        chunk->readElem(&glowColor[1], sizeof(float));
        chunk->readElem(&glowColor[2], sizeof(float));
        chunk->readElem(&glowLat, sizeof(float));
        chunk->readElem(&glowLon, sizeof(float));
        chunk->readElem(&glowAng, sizeof(float));
        chunk->readElem(&glowScale, sizeof(float));
        chunk->readElem(&glowMethod, sizeof(glowMethod));
        chunk->readElem(&clipLat, sizeof(float));

        if (enable) {
            params.enabled = true;
            params.fog = (fog != 0);
            params.numLatPoints = (latCount > 2 && latCount < 32) ? latCount : 8;
            params.numLonPoints = (lonCount > 4 && lonCount < 64) ? lonCount : 24;
            params.dipAngle = dipAngle;

            // Copy control colours
            std::memcpy(params.poleColor, colors[0], sizeof(float) * 3);
            std::memcpy(params.fortyFiveColor, colors[1], sizeof(float) * 3);
            std::memcpy(params.seventyColor, colors[2], sizeof(float) * 3);
            std::memcpy(params.horizonColor, colors[3], sizeof(float) * 3);
            std::memcpy(params.dipColor, colors[4], sizeof(float) * 3);

            std::fprintf(stderr, "SKYOBJVAR: lat=%d lon=%d dip=%.1f\n",
                         params.numLatPoints, params.numLonPoints, params.dipAngle);
        } else {
            std::fprintf(stderr, "SKYOBJVAR: new sky disabled, using defaults\n");
        }
    } catch (const std::exception &e) {
        std::fprintf(stderr, "Failed to read SKYOBJVAR: %s (using defaults)\n", e.what());
    }

    return params;
}

// Sky dome mesh data — a vertex-coloured hemisphere
struct SkyDome {
    std::vector<PosColorVertex> vertices;
    std::vector<uint16_t> indices;
};

// Build a vertex-coloured hemisphere for sky rendering.
// Latitude goes from 0 (pole/zenith) to 90+dipAngle degrees (below horizon).
// Colour is interpolated through 5 control stops matching Dark Engine's sky system.
static SkyDome buildSkyDome(const SkyParams &sky) {
    SkyDome dome;

    const float PI = 3.14159265f;
    const float DEG2RAD = PI / 180.0f;

    int numLat = sky.numLatPoints;   // rings from pole to below-horizon
    int numLon = sky.numLonPoints;   // segments around

    // Total angular range: 0 (pole) to 90 + dipAngle (below horizon)
    float maxLat = (90.0f + sky.dipAngle) * DEG2RAD;

    // Radius — large enough that it feels infinitely far (no depth test anyway)
    const float radius = 1000.0f;

    // 5-stop gradient: pole(0°), 45°, 70°, 90°(horizon), 90+dip°
    // Stored as {latDegrees, r, g, b}
    struct ColorStop { float lat; float r, g, b; };
    ColorStop stops[5] = {
        {  0.0f, sky.poleColor[0],       sky.poleColor[1],       sky.poleColor[2]       },
        { 45.0f, sky.fortyFiveColor[0],  sky.fortyFiveColor[1],  sky.fortyFiveColor[2]  },
        { 70.0f, sky.seventyColor[0],    sky.seventyColor[1],    sky.seventyColor[2]    },
        { 90.0f, sky.horizonColor[0],    sky.horizonColor[1],    sky.horizonColor[2]    },
        { 90.0f + sky.dipAngle,
                  sky.dipColor[0],        sky.dipColor[1],        sky.dipColor[2]        },
    };

    // Interpolate colour at a given latitude (degrees)
    auto lerpColor = [&](float latDeg) -> uint32_t {
        // Find the surrounding stops
        int i = 0;
        for (; i < 4; ++i) {
            if (latDeg <= stops[i + 1].lat) break;
        }
        i = std::min(i, 3);

        float range = stops[i + 1].lat - stops[i].lat;
        float t = (range > 0.001f) ? (latDeg - stops[i].lat) / range : 0.0f;
        t = std::max(0.0f, std::min(1.0f, t));

        float r = stops[i].r + t * (stops[i + 1].r - stops[i].r);
        float g = stops[i].g + t * (stops[i + 1].g - stops[i].g);
        float b = stops[i].b + t * (stops[i + 1].b - stops[i].b);
        return packABGR(r, g, b);
    };

    // Vertex 0: pole (zenith, Z-up)
    dome.vertices.push_back({ 0.0f, 0.0f, radius, lerpColor(0.0f) });

    // Latitude rings: numLat-1 rings between pole and max latitude
    for (int lat = 1; lat < numLat; ++lat) {
        float latFrac = static_cast<float>(lat) / static_cast<float>(numLat - 1);
        float latRad = latFrac * maxLat;
        float latDeg = latFrac * (90.0f + sky.dipAngle);

        float sinLat = std::sin(latRad);
        float cosLat = std::cos(latRad);

        uint32_t color = lerpColor(latDeg);

        for (int lon = 0; lon < numLon; ++lon) {
            float lonRad = static_cast<float>(lon) / static_cast<float>(numLon) * 2.0f * PI;
            float x = radius * sinLat * std::cos(lonRad);
            float y = radius * sinLat * std::sin(lonRad);
            float z = radius * cosLat;
            dome.vertices.push_back({ x, y, z, color });
        }
    }

    // Indices — pole fan (vertex 0 connects to first ring)
    for (int lon = 0; lon < numLon; ++lon) {
        int next = (lon + 1) % numLon;
        dome.indices.push_back(0);
        dome.indices.push_back(static_cast<uint16_t>(1 + lon));
        dome.indices.push_back(static_cast<uint16_t>(1 + next));
    }

    // Quad strips between consecutive latitude rings
    for (int lat = 1; lat < numLat - 1; ++lat) {
        int ringA = 1 + (lat - 1) * numLon;
        int ringB = 1 + lat * numLon;

        for (int lon = 0; lon < numLon; ++lon) {
            int next = (lon + 1) % numLon;

            // Two triangles per quad
            dome.indices.push_back(static_cast<uint16_t>(ringA + lon));
            dome.indices.push_back(static_cast<uint16_t>(ringB + lon));
            dome.indices.push_back(static_cast<uint16_t>(ringB + next));

            dome.indices.push_back(static_cast<uint16_t>(ringA + lon));
            dome.indices.push_back(static_cast<uint16_t>(ringB + next));
            dome.indices.push_back(static_cast<uint16_t>(ringA + next));
        }
    }

    std::fprintf(stderr, "Sky dome: %zu vertices, %zu indices (%zu triangles)\n",
                 dome.vertices.size(), dome.indices.size(), dome.indices.size() / 3);

    return dome;
}

// ── Textured skybox (old sky system) ──

// Per-face draw range within the skybox cube's index buffer
struct SkyboxFace {
    std::string key;       // "n", "s", "e", "w", "t"
    uint32_t firstIndex;
    uint32_t indexCount;   // always 6
};

// Skybox cube mesh — 5 textured quads (N/S/E/W/T) viewed from inside
struct SkyboxCube {
    std::vector<PosColorUVVertex> vertices;
    std::vector<uint16_t> indices;
    std::vector<SkyboxFace> faces;
};

// Build 5 textured quads for the skybox cube.
// Each face is a quad with UV-mapped texture, white vertex colour (texture
// appears unmodified), and CW winding from inside for CULL_CCW compatibility.
// Z-up coordinate system: N=-X, S=+X, E=+Y, W=-Y, T=+Z.
static SkyboxCube buildSkyboxCube() {
    SkyboxCube cube;

    const float h = 500.0f; // half-size — arbitrary, no depth test
    const uint32_t white = 0xFFFFFFFFu; // ABGR white

    // Each face: 4 vertices (TL, TR, BL, BR) + 6 indices (2 triangles)
    // Winding order: TL→BL→TR, TR→BL→BR (CW from inside)
    struct FaceDef {
        const char *key;
        float verts[4][3]; // TL, TR, BL, BR positions
    };

    // Face vertex definitions — CW winding from inside the cube.
    // Dark Engine skybox camera angles per face:
    //   'n' heading=180° → -X wall, 'e' heading=90° → +Y wall,
    //   's' heading=0°   → +X wall, 'w' heading=270° → -Y wall,
    //   't' pitch=-90°   → +Z wall (top)
    // UV orientation: u increases in camera-right direction, v increases downward.
    // Camera right at each heading from our viewer's Camera::move():
    //   heading=0°: right=-Y, heading=90°: right=+X,
    //   heading=180°: right=+Y, heading=270°: right=-X
    const int NUM_FACES = 5;
    FaceDef faceDefs[NUM_FACES];

    // North (-X wall): heading=180°, camera right=+Y, up=+Z
    // Screen: left=-Y, right=+Y, top=+Z, bottom=-Z
    faceDefs[0].key = "n";
    faceDefs[0].verts[0][0] = -h; faceDefs[0].verts[0][1] = -h; faceDefs[0].verts[0][2] =  h; // TL
    faceDefs[0].verts[1][0] = -h; faceDefs[0].verts[1][1] =  h; faceDefs[0].verts[1][2] =  h; // TR
    faceDefs[0].verts[2][0] = -h; faceDefs[0].verts[2][1] = -h; faceDefs[0].verts[2][2] = -h; // BL
    faceDefs[0].verts[3][0] = -h; faceDefs[0].verts[3][1] =  h; faceDefs[0].verts[3][2] = -h; // BR

    // East (+Y wall): heading=90°, camera right=+X, up=+Z
    // Screen: left=-X, right=+X, top=+Z, bottom=-Z
    faceDefs[1].key = "e";
    faceDefs[1].verts[0][0] = -h; faceDefs[1].verts[0][1] =  h; faceDefs[1].verts[0][2] =  h; // TL
    faceDefs[1].verts[1][0] =  h; faceDefs[1].verts[1][1] =  h; faceDefs[1].verts[1][2] =  h; // TR
    faceDefs[1].verts[2][0] = -h; faceDefs[1].verts[2][1] =  h; faceDefs[1].verts[2][2] = -h; // BL
    faceDefs[1].verts[3][0] =  h; faceDefs[1].verts[3][1] =  h; faceDefs[1].verts[3][2] = -h; // BR

    // South (+X wall): heading=0°, camera right=-Y, up=+Z
    // Screen: left=+Y, right=-Y, top=+Z, bottom=-Z
    faceDefs[2].key = "s";
    faceDefs[2].verts[0][0] =  h; faceDefs[2].verts[0][1] =  h; faceDefs[2].verts[0][2] =  h; // TL
    faceDefs[2].verts[1][0] =  h; faceDefs[2].verts[1][1] = -h; faceDefs[2].verts[1][2] =  h; // TR
    faceDefs[2].verts[2][0] =  h; faceDefs[2].verts[2][1] =  h; faceDefs[2].verts[2][2] = -h; // BL
    faceDefs[2].verts[3][0] =  h; faceDefs[2].verts[3][1] = -h; faceDefs[2].verts[3][2] = -h; // BR

    // West (-Y wall): heading=270°, camera right=-X, up=+Z
    // Screen: left=+X, right=-X, top=+Z, bottom=-Z
    faceDefs[3].key = "w";
    faceDefs[3].verts[0][0] =  h; faceDefs[3].verts[0][1] = -h; faceDefs[3].verts[0][2] =  h; // TL
    faceDefs[3].verts[1][0] = -h; faceDefs[3].verts[1][1] = -h; faceDefs[3].verts[1][2] =  h; // TR
    faceDefs[3].verts[2][0] =  h; faceDefs[3].verts[2][1] = -h; faceDefs[3].verts[2][2] = -h; // BL
    faceDefs[3].verts[3][0] = -h; faceDefs[3].verts[3][1] = -h; faceDefs[3].verts[3][2] = -h; // BR

    // Top (+Z wall): heading=90° then pitch up 90°, camera right=+X
    // Screen top = -Y (what was behind when facing +Y), bottom = +Y
    faceDefs[4].key = "t";
    faceDefs[4].verts[0][0] = -h; faceDefs[4].verts[0][1] = -h; faceDefs[4].verts[0][2] =  h; // TL
    faceDefs[4].verts[1][0] =  h; faceDefs[4].verts[1][1] = -h; faceDefs[4].verts[1][2] =  h; // TR
    faceDefs[4].verts[2][0] = -h; faceDefs[4].verts[2][1] =  h; faceDefs[4].verts[2][2] =  h; // BL
    faceDefs[4].verts[3][0] =  h; faceDefs[4].verts[3][1] =  h; faceDefs[4].verts[3][2] =  h; // BR

    for (const auto &fd : faceDefs) {
        SkyboxFace face;
        face.key = fd.key;
        face.firstIndex = static_cast<uint32_t>(cube.indices.size());
        face.indexCount = 6;

        uint16_t base = static_cast<uint16_t>(cube.vertices.size());

        // TL(0,0), TR(1,0), BL(0,1), BR(1,1)
        cube.vertices.push_back({ fd.verts[0][0], fd.verts[0][1], fd.verts[0][2], white, 0.0f, 0.0f });
        cube.vertices.push_back({ fd.verts[1][0], fd.verts[1][1], fd.verts[1][2], white, 1.0f, 0.0f });
        cube.vertices.push_back({ fd.verts[2][0], fd.verts[2][1], fd.verts[2][2], white, 0.0f, 1.0f });
        cube.vertices.push_back({ fd.verts[3][0], fd.verts[3][1], fd.verts[3][2], white, 1.0f, 1.0f });

        // Two triangles (CW from inside — matches dome convention):
        // TL→TR→BL, TR→BR→BL
        cube.indices.push_back(base + 0);
        cube.indices.push_back(base + 1);
        cube.indices.push_back(base + 2);

        cube.indices.push_back(base + 1);
        cube.indices.push_back(base + 3);
        cube.indices.push_back(base + 2);

        cube.faces.push_back(std::move(face));
    }

    std::fprintf(stderr, "Skybox cube: %zu vertices, %zu indices (%zu faces)\n",
                 cube.vertices.size(), cube.indices.size(), cube.faces.size());

    return cube;
}

// Build a 4x4 model matrix from position + binary-radian angles.
// Dark Engine uses heading/pitch/bank in binary radians (65536 = 360 degrees).
//
// Dark Engine coordinate system (Z-up):
//   heading = rotation around Z axis  (angvec.tz)
//   pitch   = rotation around Y axis  (angvec.ty)
//   bank    = rotation around X axis  (angvec.tx)
//
// Dark Engine rotation order: R = Rx(bank) * Ry(pitch) * Rz(heading)
// bx::mtxRotateXYZ(ax, ay, az) = Rx(ax) * Ry(ay) * Rz(az) — same order.
static void buildModelMatrix(float *mtx, float x, float y, float z,
                              int16_t heading, int16_t pitch, int16_t bank,
                              float sx = 1.0f, float sy = 1.0f, float sz = 1.0f) {
    const float angScale = 2.0f * 3.14159265f / 65536.0f;

    float h = static_cast<float>(heading) * angScale;
    float p = static_cast<float>(pitch)   * angScale;
    float b = static_cast<float>(bank)    * angScale;

    // Build rotation matrix matching the Dark Engine rotation convention:
    //   M_DE = Rz(heading) * Ry(pitch) * Rx(bank)  (column-vector, column-major)
    // where bank=X-rot, pitch=Y-rot, heading=Z-rot (Z-up coordinate system).
    // bx stores row-major; Metal reads as column-major (= transpose), which
    // effectively reverses the rotation direction. Negating all angles
    // compensates, producing the correct Dark Engine rotation on the GPU.
    bx::mtxRotateXYZ(mtx, -b, -p, -h);

    // Apply scale in local model space: M = T * R * S
    // bx uses row-vector convention (v * M), so to apply model-space scale
    // before rotation, we multiply: M_bx = Scale_rows * R_bx.
    // After bgfx's transposition for the GPU (column-vector: M^T * v),
    // this becomes R_gpu * S * v — scale first, then rotate. Correct.
    mtx[ 0] *= sx;  mtx[ 1] *= sx;  mtx[ 2] *= sx;  // row 0 *= sx
    mtx[ 4] *= sy;  mtx[ 5] *= sy;  mtx[ 6] *= sy;  // row 1 *= sy
    mtx[ 8] *= sz;  mtx[ 9] *= sz;  mtx[10] *= sz;  // row 2 *= sz

    mtx[12] = x;
    mtx[13] = y;
    mtx[14] = z;
}

// ── Fly Camera ──

struct Camera {
    float pos[3];
    float yaw;
    float pitch;

    void init(float x, float y, float z) {
        pos[0] = x; pos[1] = y; pos[2] = z;
        yaw = 0; pitch = 0;
    }

    void getViewMatrix(float *mtx) const {
        // Dark Engine uses Z-up: X=right, Y=forward, Z=up.
        // Forward direction from yaw (heading about Z) and pitch (tilt up/down).
        // At yaw=0: forward = +X (Dark Engine convention — identity rotation faces +X).
        float cosPitch = std::cos(pitch);
        float fwd[3] = {
            std::cos(yaw) * cosPitch,   // X
            std::sin(yaw) * cosPitch,   // Y
            std::sin(pitch)              // Z (up/down tilt)
        };

        float at[3] = { pos[0] + fwd[0], pos[1] + fwd[1], pos[2] + fwd[2] };

        bx::Vec3 eye = { pos[0], pos[1], pos[2] };
        bx::Vec3 target = { at[0], at[1], at[2] };
        bx::Vec3 upV = { 0.0f, 0.0f, 1.0f };  // Z is up in Dark Engine

        bx::mtxLookAt(mtx, eye, target, upV, bx::Handedness::Right);
    }

    // Sky view matrix: rotation only, no translation.
    // Sky dome follows camera rotation but appears infinitely far away.
    void getSkyViewMatrix(float *mtx) const {
        float cosPitch = std::cos(pitch);
        float fwd[3] = {
            std::cos(yaw) * cosPitch,
            std::sin(yaw) * cosPitch,
            std::sin(pitch)
        };

        // Look from origin — sky dome is centered at (0,0,0)
        bx::Vec3 eye = { 0.0f, 0.0f, 0.0f };
        bx::Vec3 target = { fwd[0], fwd[1], fwd[2] };
        bx::Vec3 upV = { 0.0f, 0.0f, 1.0f };

        bx::mtxLookAt(mtx, eye, target, upV, bx::Handedness::Right);
    }

    void move(float forward, float right, float up) {
        // Dark Engine Z-up: yaw rotates in XY plane, Z is vertical.
        float cosPitch = std::cos(pitch);
        float fwd[3] = {
            std::cos(yaw) * cosPitch,   // X
            std::sin(yaw) * cosPitch,   // Y
            std::sin(pitch)              // Z
        };
        // Right vector: forward × up in right-handed Z-up = -Y at yaw=0
        float rt[3] = {
            std::sin(yaw),   // X
           -std::cos(yaw),   // Y
            0.0f              // Z (horizontal movement only)
        };

        pos[0] += fwd[0] * forward + rt[0] * right;
        pos[1] += fwd[1] * forward + rt[1] * right;
        pos[2] += fwd[2] * forward + up;  // Z is vertical
    }
};

static void printHelp() {
    std::fprintf(stderr,
        "darknessRender — Dark Engine world geometry viewer\n"
        "\n"
        "Usage:\n"
        "  darknessRender <mission.mis> [--res <path>] [--config <path>] [--lm-scale <N>]\n"
        "\n"
        "Options:\n"
        "  --res <path>   Path to Thief 2 RES directory containing fam.crf.\n"
        "                 Enables lightmapped+textured rendering. Without this\n"
        "                 flag, geometry is rendered with flat Lambertian shading.\n"
        "  --lm-scale <N> Lightmap upscale factor (1-8, default 1).\n"
        "                 1 = vintage (original blocky lightmaps).\n"
        "                 2/4/8 = progressively smoother shadows via bicubic\n"
        "                 interpolation. Higher values use more atlas memory.\n"
        "  --no-objects   Disable object mesh rendering (world geometry only).\n"
        "  --no-cull      Start with portal culling disabled (see all geometry).\n"
        "  --filter       Start with bilinear texture filtering (default: point/crispy).\n"
        "  --force-flicker Force all animated lights to flicker mode (debug).\n"
        "  --linear-mips  Gamma-correct mipmap generation (sRGB linearization).\n"
        "  --sharp-mips   Sharpen mip levels to preserve detail at distance.\n"
        "  --config <path> Path to YAML config file (default: ./darknessRender.yaml).\n"
        "  --help         Show this help message.\n"
        "\n"
        "Controls:\n"
        "  WASD           Move forward/left/back/right\n"
        "  Mouse          Look around\n"
        "  Space/LShift   Move up/down\n"
        "  Q/E            Move up/down (alternate)\n"
        "  Ctrl           Sprint (3x speed)\n"
        "  Scroll wheel   Adjust movement speed (shown in title bar)\n"
        "  C              Toggle portal culling on/off\n"
        "  F              Cycle texture filtering (point/bilinear/trilinear/aniso)\n"
        "  Home           Teleport to player spawn point\n"
        "  Esc            Quit\n"
        "\n"
        "Resource setup:\n"
        "  The --res path should point to a directory containing fam.crf, which\n"
        "  holds the PCX textures used by Dark Engine levels. This can come from:\n"
        "\n"
        "  1. Mounted ISO (macOS):\n"
        "     hdiutil mount ../disk_images/thief_2_disk_1.iso\n"
        "     --res /Volumes/THIEF2_INSTALL_C/THIEF2/RES\n"
        "\n"
        "  2. GOG/Steam install directory:\n"
        "     --res /path/to/Thief2/RES\n"
        "\n"
        "  3. Any directory containing fam.crf\n"
        "\n"
        "Examples:\n"
        "  # Flat-shaded (no external resources needed):\n"
        "  darknessRender path/to/miss6.mis\n"
        "\n"
        "  # Textured, using mounted Thief 2 disc:\n"
        "  darknessRender path/to/miss6.mis --res /Volumes/THIEF2_INSTALL_C/THIEF2/RES\n"
        "\n"
        "  # Textured, using GOG install:\n"
        "  darknessRender path/to/miss6.mis --res ~/GOG/Thief2/RES\n"
    );
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        printHelp();
        return 1;
    }

    // Parse config: hardcoded defaults → YAML file → CLI overrides
    Darkness::RenderConfig cfg;

    // First CLI pass: extract --config path (and detect --help early)
    Darkness::CliResult cli = Darkness::applyCliOverrides(argc, argv, cfg);

    if (cli.helpRequested) {
        printHelp();
        return 0;
    }

    if (!cli.misPath) {
        std::fprintf(stderr, "Error: no mission file specified.\n\n");
        printHelp();
        return 1;
    }

    // Load YAML config (defaults to ./darknessRender.yaml if no --config flag)
    std::string configPath = cli.configPath.empty() ? "darknessRender.yaml" : cli.configPath;
    Darkness::loadConfigFromYAML(configPath, cfg);

    // Re-apply CLI so flags always win over YAML values
    cli = Darkness::applyCliOverrides(argc, argv, cfg);

    // Unpack into local variables — rest of file uses these unchanged
    const char *misPath    = cli.misPath;
    std::string resPath    = cli.resPath;
    int  lmScale           = cfg.lmScale;
    bool showObjects       = cfg.showObjects;
    bool forceFlicker      = cfg.forceFlicker;
    bool portalCulling     = cfg.portalCulling;
    int  filterMode        = cfg.filterMode;
    bool linearMips        = cfg.linearMips;
    bool sharpMips         = cfg.sharpMips;
    float waveAmplitude    = cfg.waveAmplitude;
    float uvDistortion     = cfg.uvDistortion;
    float waterRotation    = cfg.waterRotation;
    float waterScrollSpeed = cfg.waterScrollSpeed;

    bool texturedMode = !resPath.empty();

    // Extract mission base name (e.g. "miss6" from "path/to/miss6.mis")
    // for constructing skybox texture filenames like "skyhw/miss6n.PCX"
    std::string missionName;
    {
        std::string p(misPath);
        size_t slash = p.find_last_of("/\\");
        std::string base = (slash != std::string::npos) ? p.substr(slash + 1) : p;
        size_t dot = base.find('.');
        missionName = (dot != std::string::npos) ? base.substr(0, dot) : base;
        // Lowercase for case-insensitive CRF matching
        std::transform(missionName.begin(), missionName.end(), missionName.begin(),
                       [](unsigned char c) { return std::tolower(c); });
    }

    // Parse WR geometry
    std::fprintf(stderr, "Loading WR geometry from %s...\n", misPath);

    Darkness::WRParsedData wrData;
    try {
        wrData = Darkness::parseWRChunk(misPath);
    } catch (const std::exception &e) {
        std::fprintf(stderr, "Failed to parse WR chunk: %s\n", e.what());
        return 1;
    }

    std::fprintf(stderr, "Loaded %u cells\n", wrData.numCells);

    // Build portal adjacency graph for portal culling
    auto cellPortals = buildPortalGraph(wrData);
    {
        int totalPortals = 0;
        for (const auto &pl : cellPortals) totalPortals += static_cast<int>(pl.size());
        std::fprintf(stderr, "Portal graph: %d portals across %u cells\n",
                     totalPortals, wrData.numCells);
    }

    // Find player spawn point from L$PlayerFactory + P$Position chunks
    Darkness::SpawnInfo spawnInfo = Darkness::findSpawnPoint(misPath);

    // Parse animated light properties from mission database
    auto lightSources = Darkness::parseAnimLightProperties(misPath);

    // Build reverse index: lightnum → list of (cellIdx, polyIdx) affected
    auto animLightIndex = Darkness::buildAnimLightIndex(wrData);

    // Ensure all lightnums referenced in WR data have a LightSource entry.
    // Lights without P$AnimLight properties default to mode 4 (max brightness).
    for (const auto &kv : animLightIndex) {
        if (lightSources.find(kv.first) == lightSources.end()) {
            Darkness::LightSource ls = {};
            ls.lightNum = kv.first;
            ls.mode = Darkness::ANIM_MAX_BRIGHT;
            ls.maxBright = 1.0f;
            ls.minBright = 0.0f;
            ls.brightness = 1.0f;
            ls.prevIntensity = 1.0f;
            lightSources[kv.first] = ls;
        }
    }

    // Apply --force-flicker: override all lights to flicker mode for debugging
    if (forceFlicker) {
        for (auto &[num, ls] : lightSources) {
            ls.mode = Darkness::ANIM_FLICKER;
            ls.inactive = false;
            ls.minBright = 0.0f;
            ls.maxBright = 1.0f;
            ls.brightenTime = 0.15f;
            ls.dimTime = 0.15f;
            ls.brightness = ls.maxBright;
            ls.countdown = 0.1f;
            ls.isRising = false;
        }
        std::fprintf(stderr, "Force-flicker: all %zu lights set to flicker mode\n",
                     lightSources.size());
    }

    // Animated light diagnostics
    {
        int modeCounts[10] = {};
        int inactiveCount = 0;
        int fromMIS = 0, fromDefault = 0;
        for (const auto &[num, ls] : lightSources) {
            if (ls.mode < 10) modeCounts[ls.mode]++;
            if (ls.inactive) inactiveCount++;
            if (ls.objectId != 0) fromMIS++; else fromDefault++;
        }
        std::fprintf(stderr, "Animated lights: %zu sources (%d from MIS, %d defaulted), "
                     "%zu indexed lightnums\n",
                     lightSources.size(), fromMIS, fromDefault, animLightIndex.size());
        const char *modeNames[] = {
            "flip", "smooth", "random", "min_bright", "max_bright",
            "zero", "brighten", "dim", "semi_random", "flicker"
        };
        for (int m = 0; m < 10; ++m) {
            if (modeCounts[m] > 0)
                std::fprintf(stderr, "  mode %d (%s): %d lights\n",
                             m, modeNames[m], modeCounts[m]);
        }
        if (inactiveCount > 0)
            std::fprintf(stderr, "  inactive: %d lights\n", inactiveCount);
    }

    // Parse sky dome parameters from SKYOBJVAR chunk (if present)
    SkyParams skyParams = parseSkyObjVar(misPath);
    SkyDome skyDome = buildSkyDome(skyParams);

    // Parse global fog parameters from FOG chunk (if present)
    FogParams fogParams = parseFogChunk(misPath);

    // Parse water flow data — FLOW_TEX (texture mapping) and CELL_MOTION (animation state)
    FlowData flowData = parseFlowData(misPath);

    // Parse TXLIST if in textured mode
    Darkness::TXList txList;
    if (texturedMode) {
        try {
            txList = Darkness::parseTXList(misPath);
            std::fprintf(stderr, "TXLIST: %zu textures, %zu families\n",
                         txList.textures.size(), txList.families.size());
        } catch (const std::exception &e) {
            std::fprintf(stderr, "Failed to parse TXLIST: %s (falling back to flat)\n",
                         e.what());
            texturedMode = false;
        }
    }

    // Collect unique texture indices used by world geometry and water flow groups
    std::unordered_set<uint8_t> usedTextures;
    if (texturedMode) {
        // World geometry textures from polygon texturing data
        for (const auto &cell : wrData.cells) {
            for (int pi = 0; pi < cell.numTextured; ++pi) {
                uint8_t txt = cell.texturing[pi].txt;
                if (txt != 0 && txt != 249)
                    usedTextures.insert(txt);
            }
        }
        // Note: FLOW_TEX texture indices are runtime palette positions, NOT TXLIST
        // indices. Water textures are loaded separately by name from fam.crf below.
        std::fprintf(stderr, "Unique texture indices used: %zu\n", usedTextures.size());
    }

    // Load world textures from fam.crf (indexed by TXLIST)
    std::unordered_map<uint8_t, Darkness::DecodedImage> loadedTextures;
    std::unordered_map<uint8_t, TexDimensions> texDims;

    if (texturedMode) {
        Darkness::CRFTextureLoader loader(resPath);
        if (!loader.isOpen()) {
            std::fprintf(stderr, "CRF not available, falling back to flat shading\n");
            texturedMode = false;
        } else {
            int loaded = 0;
            for (uint8_t idx : usedTextures) {
                if (idx >= txList.textures.size()) continue;
                const auto &entry = txList.textures[idx];
                auto img = loader.loadTexture(entry.family, entry.name);
                texDims[idx] = { img.width, img.height };
                loadedTextures[idx] = std::move(img);
                ++loaded;
            }
            std::fprintf(stderr, "Loaded %d/%zu textures from CRF\n",
                         loaded, usedTextures.size());
        }
    }

    // Load water flow textures from fam.crf by name.
    // FLOW_TEX name field (e.g. "gr") maps to "water/<name>in.PCX" for the
    // air-side texture and "water/<name>out.PCX" for the underwater side.
    // Keyed by flow group index (1-255).
    std::unordered_map<uint8_t, Darkness::DecodedImage> flowLoadedTextures;
    std::unordered_map<uint8_t, TexDimensions> flowTexDims;

    if (texturedMode && flowData.hasFlowTex) {
        Darkness::CRFTextureLoader loader(resPath);
        if (loader.isOpen()) {
            // Collect unique flow groups used by cells
            std::unordered_set<uint8_t> usedFlowGroups;
            for (const auto &cell : wrData.cells) {
                if (cell.flowGroup > 0)
                    usedFlowGroups.insert(cell.flowGroup);
            }

            int loaded = 0;
            for (uint8_t fg : usedFlowGroups) {
                const auto &fe = flowData.textures[fg];
                if (fe.name[0] == '\0') continue;

                // Extract base name, trimming trailing nulls/spaces
                std::string baseName(fe.name, strnlen(fe.name, 28));
                while (!baseName.empty() && (baseName.back() == ' ' || baseName.back() == '\0'))
                    baseName.pop_back();
                if (baseName.empty()) continue;

                // Air-side texture: "water/<name>in" (e.g. "gr" → "water/grin")
                std::string inName = baseName + "in";
                auto img = loader.loadTexture("water", inName);

                // Check if we got a real texture (not the 8x8 fallback)
                if (img.width > 8 || img.height > 8) {
                    std::fprintf(stderr, "Flow group %d: loaded water/%s.PCX (%ux%u)\n",
                                 fg, inName.c_str(), img.width, img.height);
                    flowTexDims[fg] = { img.width, img.height };
                    flowLoadedTextures[fg] = std::move(img);
                    ++loaded;
                } else {
                    std::fprintf(stderr, "Flow group %d: water/%s.PCX not found, trying waterhw/\n",
                                 fg, inName.c_str());
                    // Some missions may use WATERHW family instead
                    auto img2 = loader.loadTexture("waterhw", inName);
                    if (img2.width > 8 || img2.height > 8) {
                        std::fprintf(stderr, "Flow group %d: loaded waterhw/%s.PCX (%ux%u)\n",
                                     fg, inName.c_str(), img2.width, img2.height);
                        flowTexDims[fg] = { img2.width, img2.height };
                        flowLoadedTextures[fg] = std::move(img2);
                        ++loaded;
                    } else {
                        std::fprintf(stderr, "Flow group %d: no water texture found for '%s'\n",
                                     fg, baseName.c_str());
                    }
                }
            }
            if (loaded > 0) {
                std::fprintf(stderr, "Loaded %d flow group water textures from CRF\n", loaded);
            }
        }
    }

    // ── Load skybox face textures (old sky system) ──
    // Missions without SKYOBJVAR use a textured skybox with per-mission PCX
    // textures in fam.crf under skyhw/ (e.g. skyhw/miss6n.PCX for north face).
    std::unordered_map<std::string, Darkness::DecodedImage> skyboxImages;
    bool hasSkybox = false;

    if (texturedMode) {
        Darkness::CRFTextureLoader skyLoader(resPath);
        if (skyLoader.isOpen()) {
            // 5 faces: n=north(+Y), s=south(-Y), e=east(+X), w=west(-X), t=top(+Z)
            const char *suffixes[] = { "n", "s", "e", "w", "t" };
            int loaded = 0;
            for (const char *suf : suffixes) {
                std::string texName = missionName + suf;
                auto img = skyLoader.loadTexture("skyhw", texName);
                // Real texture is larger than the 8x8 fallback checkerboard
                if (img.width > 8 || img.height > 8) {
                    skyboxImages[suf] = std::move(img);
                    ++loaded;
                }
            }
            // Skybox available if at least the 4 side faces loaded (top optional)
            hasSkybox = skyboxImages.count("n") && skyboxImages.count("s")
                     && skyboxImages.count("e") && skyboxImages.count("w");
            if (hasSkybox) {
                std::fprintf(stderr, "Skybox: loaded %d/5 faces for %s (textured skybox active)\n",
                             loaded, missionName.c_str());
                for (auto &kv : skyboxImages) {
                    std::fprintf(stderr, "  face '%s': %ux%u\n",
                                 kv.first.c_str(), kv.second.width, kv.second.height);
                }
            } else if (loaded > 0) {
                std::fprintf(stderr, "Skybox: partial load (%d faces), falling back to dome\n", loaded);
            }
        }
    }

    // ── Parse object placements from .mis ──

    Darkness::ObjectPropData objData;
    if (showObjects) {
        try {
            objData = Darkness::parseObjectProps(misPath);
        } catch (const std::exception &e) {
            std::fprintf(stderr, "Failed to parse object props: %s\n", e.what());
            showObjects = false;
        }
        if (objData.objects.empty()) {
            std::fprintf(stderr, "No objects to render\n");
            showObjects = false;
        }
    }

    // Precompute which cell each object is in for portal culling.
    // Objects don't move (yet), so this is a one-time lookup at load time.
    // -1 = outside all cells (always rendered to avoid popping).
    std::vector<int32_t> objCellIDs;
    if (showObjects) {
        objCellIDs.resize(objData.objects.size());
        for (size_t i = 0; i < objData.objects.size(); ++i) {
            const auto &obj = objData.objects[i];
            if (obj.hasPosition) {
                objCellIDs[i] = findCameraCell(wrData, obj.x, obj.y, obj.z);
            } else {
                objCellIDs[i] = -1;
            }
        }
    }

    // Load .bin models from obj.crf (if --res provided and objects enabled)
    std::unordered_map<std::string, Darkness::ParsedBinMesh> parsedModels;

    if (showObjects && !resPath.empty()) {
        Darkness::CRFModelLoader modelLoader(resPath);
        if (modelLoader.isOpen()) {
            int loaded = 0, failed = 0;
            for (const auto &name : objData.uniqueModels) {
                auto binData = modelLoader.loadModel(name);
                if (binData.empty()) {
                    ++failed;
                    continue;
                }
                try {
                    auto mesh = Darkness::parseBinModel(binData.data(), binData.size());
                    if (mesh.valid) {
                        parsedModels[name] = std::move(mesh);
                        ++loaded;
                    } else {
                        // Log first few failures for debugging
                        if (failed < 5) {
                            // Show magic header of failed file
                            char hdr[5] = {};
                            if (binData.size() >= 4)
                                std::memcpy(hdr, binData.data(), 4);
                            std::fprintf(stderr, "  model '%s': parse failed "
                                         "(size=%zu, magic='%s')\n",
                                         name.c_str(), binData.size(), hdr);
                        }
                        ++failed;
                    }
                } catch (const std::exception &e) {
                    // Some .bin files may be AI meshes (LGMM) or corrupt
                    if (failed < 5) {
                        std::fprintf(stderr, "  model '%s': exception: %s\n",
                                     name.c_str(), e.what());
                    }
                    ++failed;
                }
            }
            std::fprintf(stderr, "Loaded %d/%zu models from obj.crf (%d failed)\n",
                         loaded, objData.uniqueModels.size(), failed);
        } else {
            std::fprintf(stderr, "obj.crf not available, using fallback cubes\n");
        }
    }

    // ── Load object textures from txt16.crf (16-bit) with txt.crf fallback ──

    // Collect unique MD_MAT_TMAP material names from all parsed models
    std::unordered_set<std::string> objMatNames;
    for (const auto &kv : parsedModels) {
        for (const auto &mat : kv.second.materials) {
            if (mat.type == Darkness::MD_MAT_TMAP) {
                // Lowercase the name for case-insensitive matching
                std::string lname(mat.name);
                std::transform(lname.begin(), lname.end(), lname.begin(),
                               [](unsigned char c) { return std::tolower(c); });
                objMatNames.insert(lname);
            }
        }
    }

    // Load object textures from obj.crf (txt16/ and txt/ subdirectories inside it).
    // Dark Engine stores object textures as GIF/PCX files within obj.crf, not in
    // separate txt16.crf/txt.crf archives.
    std::unordered_map<std::string, Darkness::DecodedImage> objTexImages;

    if (!objMatNames.empty() && !resPath.empty()) {
        // Reuse obj.crf for texture lookup (same archive that holds .bin models)
        Darkness::CRFTextureLoader objTexLoader(resPath, "obj.crf");

        int loaded = 0;
        for (const auto &name : objMatNames) {
            if (!objTexLoader.isOpen()) break;
            auto img = objTexLoader.loadObjectTexture(name);
            // Check if we got a real texture (not the 8x8 fallback checkerboard)
            if (img.width > 8 || img.height > 8) {
                objTexImages[name] = std::move(img);
                ++loaded;
            }
        }
        std::fprintf(stderr, "Loaded %d/%zu object textures from obj.crf\n",
                     loaded, objMatNames.size());
    }

    // ── SDL2 + bgfx init ──

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window *window = SDL_CreateWindow(
        "darkness — lightmapped renderer",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WINDOW_WIDTH, WINDOW_HEIGHT,
        SDL_WINDOW_SHOWN | SDL_WINDOW_ALLOW_HIGHDPI
    );

    if (!window) {
        std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_SysWMinfo wmi;
    SDL_VERSION(&wmi.version);
    if (!SDL_GetWindowWMInfo(window, &wmi)) {
        std::fprintf(stderr, "SDL_GetWindowWMInfo failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    bgfx::renderFrame(); // single-threaded mode

    bgfx::Init bInit;
    bInit.type = bgfx::RendererType::Metal;
    bInit.resolution.width  = WINDOW_WIDTH;
    bInit.resolution.height = WINDOW_HEIGHT;
    bInit.resolution.reset  = BGFX_RESET_VSYNC;

#if BX_PLATFORM_OSX
    bInit.platformData.nwh = wmi.info.cocoa.window;
#elif BX_PLATFORM_LINUX
    bInit.platformData.ndt = wmi.info.x11.display;
    bInit.platformData.nwh = (void *)(uintptr_t)wmi.info.x11.window;
#elif BX_PLATFORM_WINDOWS
    bInit.platformData.nwh = wmi.info.win.window;
#endif

    if (!bgfx::init(bInit)) {
        std::fprintf(stderr, "bgfx::init failed\n");
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    // View 0: Sky pass — clears colour + depth, renders sky dome with no depth writes
    // When fog is enabled, use fog colour as clear colour so uncovered sky matches
    uint32_t skyClearColor = 0x1a1a2eFF;
    if (fogParams.enabled) {
        uint8_t fr = static_cast<uint8_t>(fogParams.r * 255.0f);
        uint8_t fg = static_cast<uint8_t>(fogParams.g * 255.0f);
        uint8_t fb = static_cast<uint8_t>(fogParams.b * 255.0f);
        skyClearColor = (uint32_t(fr) << 24) | (uint32_t(fg) << 16) | (uint32_t(fb) << 8) | 0xFF;
    }
    bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
                        skyClearColor, 1.0f, 0);
    bgfx::setViewRect(0, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);

    // View 1: World + objects pass — clears depth only, preserves sky colour
    bgfx::setViewClear(1, BGFX_CLEAR_DEPTH, 0, 1.0f, 0);
    bgfx::setViewRect(1, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);

    PosColorVertex::init();
    PosColorUVVertex::init();
    PosUV2Vertex::init();

    // ── Shaders ──

    // Flat-color program (existing basic shader)
    bgfx::ShaderHandle flatVsh = bgfx::createShader(
        bgfx::makeRef(vs_basic_metal, sizeof(vs_basic_metal))
    );
    bgfx::ShaderHandle flatFsh = bgfx::createShader(
        bgfx::makeRef(fs_basic_metal, sizeof(fs_basic_metal))
    );
    bgfx::ProgramHandle flatProgram = bgfx::createProgram(flatVsh, flatFsh, true);

    // Textured program
    bgfx::ShaderHandle texVsh = bgfx::createShader(
        bgfx::makeRef(vs_textured_metal, sizeof(vs_textured_metal))
    );
    bgfx::ShaderHandle texFsh = bgfx::createShader(
        bgfx::makeRef(fs_textured_metal, sizeof(fs_textured_metal))
    );
    bgfx::ProgramHandle texturedProgram = bgfx::createProgram(texVsh, texFsh, true);

    // Lightmapped program
    bgfx::ProgramHandle lightmappedProgram = bgfx::createProgram(
        bgfx::createShader(bgfx::makeRef(vs_lightmapped_metal, sizeof(vs_lightmapped_metal))),
        bgfx::createShader(bgfx::makeRef(fs_lightmapped_metal, sizeof(fs_lightmapped_metal))),
        true);

    // Water program: vertex displacement + textured fragment with UV distortion
    bgfx::ProgramHandle waterProgram = bgfx::createProgram(
        bgfx::createShader(bgfx::makeRef(vs_water_metal, sizeof(vs_water_metal))),
        bgfx::createShader(bgfx::makeRef(fs_water_metal, sizeof(fs_water_metal))),
        true);

    bgfx::UniformHandle s_texColor = bgfx::createUniform("s_texColor", bgfx::UniformType::Sampler);
    bgfx::UniformHandle s_texLightmap = bgfx::createUniform("s_texLightmap", bgfx::UniformType::Sampler);
    bgfx::UniformHandle u_waterParams = bgfx::createUniform("u_waterParams", bgfx::UniformType::Vec4);
    bgfx::UniformHandle u_waterFlow = bgfx::createUniform("u_waterFlow", bgfx::UniformType::Vec4);
    bgfx::UniformHandle u_fogColor = bgfx::createUniform("u_fogColor", bgfx::UniformType::Vec4);
    bgfx::UniformHandle u_fogParams = bgfx::createUniform("u_fogParams", bgfx::UniformType::Vec4);

    // ── Build lightmap atlas (if textured mode) ──

    bool lightmappedMode = false;
    Darkness::LightmapAtlasSet lmAtlasSet;
    std::vector<bgfx::TextureHandle> lightmapAtlasHandles;

    if (texturedMode) {
        lmAtlasSet = Darkness::buildLightmapAtlases(wrData, lmScale);
        if (!lmAtlasSet.atlases.empty()) {
            lightmappedMode = true;

            // Initial blend pass: apply animated overlays at initial intensities.
            // buildLightmapAtlases() only wrote static lightmaps into the atlas.
            // We must blend in overlay contributions so lights at max brightness
            // (mode 4, the default) have correct initial appearance.
            if (!animLightIndex.empty()) {
                // Compute initial intensities for all lights
                std::unordered_map<int16_t, float> initIntensities;
                for (const auto &[lightNum, light] : lightSources) {
                    initIntensities[lightNum] = (light.maxBright > 0.0f)
                        ? light.brightness / light.maxBright : 0.0f;
                }

                // Collect unique (cell, poly) pairs — a polygon may appear under
                // multiple lightnums, but blendAnimatedLightmap re-blends all
                // overlays at once, so we only need to call it once per polygon.
                std::unordered_set<uint64_t> blendedSet;
                int blendedPolys = 0;
                for (const auto &[lightNum, polys] : animLightIndex) {
                    for (const auto &[ci, pi] : polys) {
                        uint64_t key = (static_cast<uint64_t>(ci) << 32)
                                      | static_cast<uint32_t>(pi);
                        if (!blendedSet.insert(key).second) continue;

                        Darkness::blendAnimatedLightmap(
                            lmAtlasSet.atlases[0], wrData, ci, pi,
                            lmAtlasSet.entries[ci][pi],
                            initIntensities, lmScale);
                        ++blendedPolys;
                    }
                }
                std::fprintf(stderr, "Initial lightmap blend: %d polygons\n",
                             blendedPolys);
            }

            for (const auto &atlas : lmAtlasSet.atlases) {
                // Create texture WITHOUT initial data so it stays mutable —
                // bgfx treats textures with initial mem as immutable.
                // We upload via updateTexture2D immediately after creation.
                // Point filtering so --lm-scale 1 gives the original blocky/vintage
                // look. Higher lm-scale values bake bicubic smoothing into the
                // atlas texels, providing progressively smoother lighting.
                bgfx::TextureHandle th = bgfx::createTexture2D(
                    static_cast<uint16_t>(atlas.size),
                    static_cast<uint16_t>(atlas.size),
                    false, 1, bgfx::TextureFormat::RGBA8,
                    BGFX_SAMPLER_MIN_POINT | BGFX_SAMPLER_MAG_POINT
                    | BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP);
                // Upload initial atlas data
                const bgfx::Memory *mem = bgfx::copy(atlas.rgba.data(),
                    static_cast<uint32_t>(atlas.rgba.size()));
                bgfx::updateTexture2D(th, 0, 0, 0, 0,
                    static_cast<uint16_t>(atlas.size),
                    static_cast<uint16_t>(atlas.size), mem);
                lightmapAtlasHandles.push_back(th);
            }
            std::fprintf(stderr, "Created %zu lightmap atlas GPU texture(s)\n",
                         lightmapAtlasHandles.size());
        }
    }

    // ── Build geometry and create GPU buffers ──

    float camX, camY, camZ;
    bgfx::VertexBufferHandle vbh;
    bgfx::IndexBufferHandle ibh;

    LightmappedMesh lmMesh;
    WorldMesh worldMesh;
    FlatMesh flatMesh;

    // Create bgfx texture handles
    std::unordered_map<uint8_t, bgfx::TextureHandle> textureHandles;
    // Flow water textures keyed by flow group index (loaded from fam.crf by name)
    std::unordered_map<uint8_t, bgfx::TextureHandle> flowTextureHandles;

    if (lightmappedMode) {
        lmMesh = buildLightmappedMesh(wrData, texDims, lmAtlasSet);
        camX = lmMesh.cx; camY = lmMesh.cy; camZ = lmMesh.cz;

        std::fprintf(stderr, "Geometry (lightmapped): %zu vertices, %zu indices (%zu triangles), %zu texture groups\n",
                     lmMesh.vertices.size(), lmMesh.indices.size(),
                     lmMesh.indices.size() / 3, lmMesh.groups.size());

        if (lmMesh.vertices.empty()) {
            std::fprintf(stderr, "No geometry to render\n");
            bgfx::shutdown();
            SDL_DestroyWindow(window);
            SDL_Quit();
            return 1;
        }

        const bgfx::Memory *vbMem = bgfx::copy(
            lmMesh.vertices.data(),
            static_cast<uint32_t>(lmMesh.vertices.size() * sizeof(PosUV2Vertex))
        );
        vbh = bgfx::createVertexBuffer(vbMem, PosUV2Vertex::layout);

        const bgfx::Memory *ibMem = bgfx::copy(
            lmMesh.indices.data(),
            static_cast<uint32_t>(lmMesh.indices.size() * sizeof(uint32_t))
        );
        ibh = bgfx::createIndexBuffer(ibMem, BGFX_BUFFER_INDEX32);

        // Create bgfx textures with full mip chains from loaded images
        for (auto &kv : loadedTextures) {
            uint8_t idx = kv.first;
            const auto &img = kv.second;
            textureHandles[idx] = createMipmappedTexture(
                img.rgba.data(), img.width, img.height,
                BGFX_SAMPLER_MIN_POINT | BGFX_SAMPLER_MAG_POINT
                | BGFX_SAMPLER_U_MIRROR | BGFX_SAMPLER_V_MIRROR,
                MipAlphaMode::ALPHA_TEST, linearMips, sharpMips);
        }
        std::fprintf(stderr, "Created %zu GPU textures (mipmapped)\n", textureHandles.size());

        // Create GPU textures for flow group water textures (loaded by name)
        // Water textures are fully opaque (alpha blending via vertex color),
        // so use ALPHA_BLEND (no coverage preservation needed).
        for (auto &kv : flowLoadedTextures) {
            uint8_t fg = kv.first;
            const auto &img = kv.second;
            flowTextureHandles[fg] = createMipmappedTexture(
                img.rgba.data(), img.width, img.height,
                BGFX_SAMPLER_U_MIRROR | BGFX_SAMPLER_V_MIRROR,
                MipAlphaMode::ALPHA_BLEND, linearMips, sharpMips);
        }
        if (!flowTextureHandles.empty()) {
            std::fprintf(stderr, "Created %zu flow water GPU textures\n", flowTextureHandles.size());
        }
    } else if (texturedMode) {
        worldMesh = buildTexturedMesh(wrData, texDims);
        camX = worldMesh.cx; camY = worldMesh.cy; camZ = worldMesh.cz;

        std::fprintf(stderr, "Geometry (textured): %zu vertices, %zu indices (%zu triangles), %zu texture groups\n",
                     worldMesh.vertices.size(), worldMesh.indices.size(),
                     worldMesh.indices.size() / 3, worldMesh.groups.size());

        if (worldMesh.vertices.empty()) {
            std::fprintf(stderr, "No geometry to render\n");
            bgfx::shutdown();
            SDL_DestroyWindow(window);
            SDL_Quit();
            return 1;
        }

        const bgfx::Memory *vbMem = bgfx::copy(
            worldMesh.vertices.data(),
            static_cast<uint32_t>(worldMesh.vertices.size() * sizeof(PosColorUVVertex))
        );
        vbh = bgfx::createVertexBuffer(vbMem, PosColorUVVertex::layout);

        const bgfx::Memory *ibMem = bgfx::copy(
            worldMesh.indices.data(),
            static_cast<uint32_t>(worldMesh.indices.size() * sizeof(uint32_t))
        );
        ibh = bgfx::createIndexBuffer(ibMem, BGFX_BUFFER_INDEX32);

        for (auto &kv : loadedTextures) {
            uint8_t idx = kv.first;
            const auto &img = kv.second;
            textureHandles[idx] = createMipmappedTexture(
                img.rgba.data(), img.width, img.height,
                BGFX_SAMPLER_MIN_POINT | BGFX_SAMPLER_MAG_POINT
                | BGFX_SAMPLER_U_MIRROR | BGFX_SAMPLER_V_MIRROR,
                MipAlphaMode::ALPHA_TEST, linearMips, sharpMips);
        }
        std::fprintf(stderr, "Created %zu GPU textures (mipmapped)\n", textureHandles.size());
    } else {
        flatMesh = buildFlatMesh(wrData);
        camX = flatMesh.cx; camY = flatMesh.cy; camZ = flatMesh.cz;

        std::fprintf(stderr, "Geometry (flat): %zu vertices, %zu indices (%zu triangles)\n",
                     flatMesh.vertices.size(), flatMesh.indices.size(),
                     flatMesh.indices.size() / 3);

        if (flatMesh.vertices.empty()) {
            std::fprintf(stderr, "No geometry to render\n");
            bgfx::shutdown();
            SDL_DestroyWindow(window);
            SDL_Quit();
            return 1;
        }

        const bgfx::Memory *vbMem = bgfx::copy(
            flatMesh.vertices.data(),
            static_cast<uint32_t>(flatMesh.vertices.size() * sizeof(PosColorVertex))
        );
        vbh = bgfx::createVertexBuffer(vbMem, PosColorVertex::layout);

        const bgfx::Memory *ibMem = bgfx::copy(
            flatMesh.indices.data(),
            static_cast<uint32_t>(flatMesh.indices.size() * sizeof(uint32_t))
        );
        ibh = bgfx::createIndexBuffer(ibMem, BGFX_BUFFER_INDEX32);
    }

    // ── Build water surface mesh from portal polygons ──
    WorldMesh waterMesh = buildWaterMesh(wrData, texDims, flowData, flowTexDims);
    bgfx::VertexBufferHandle waterVBH = BGFX_INVALID_HANDLE;
    bgfx::IndexBufferHandle waterIBH = BGFX_INVALID_HANDLE;
    bool hasWater = !waterMesh.vertices.empty();

    if (hasWater) {
        waterVBH = bgfx::createVertexBuffer(
            bgfx::copy(waterMesh.vertices.data(),
                        static_cast<uint32_t>(waterMesh.vertices.size() * sizeof(PosColorUVVertex))),
            PosColorUVVertex::layout);
        waterIBH = bgfx::createIndexBuffer(
            bgfx::copy(waterMesh.indices.data(),
                        static_cast<uint32_t>(waterMesh.indices.size() * sizeof(uint32_t))),
            BGFX_BUFFER_INDEX32);
    }

    // ── Create GPU buffers for object meshes ──

    // Per-unique-model GPU buffers (PosColorUVVertex for textured rendering)
    std::unordered_map<std::string, ObjectModelGPU> objModelGPU;

    // Object texture GPU handles (keyed by lowercase material name)
    std::unordered_map<std::string, bgfx::TextureHandle> objTextureHandles;

    // Fallback cube: a single shared VBH/IBH for models not found in obj.crf
    bgfx::VertexBufferHandle fallbackCubeVBH = BGFX_INVALID_HANDLE;
    bgfx::IndexBufferHandle fallbackCubeIBH = BGFX_INVALID_HANDLE;
    uint32_t fallbackCubeIndexCount = 0;

    if (showObjects) {
        // Build fallback cube
        {
            std::vector<PosColorVertex> cubeVerts;
            std::vector<uint32_t> cubeIndices;
            buildFallbackCube(cubeVerts, cubeIndices, 0xff808080u);
            fallbackCubeIndexCount = static_cast<uint32_t>(cubeIndices.size());
            fallbackCubeVBH = bgfx::createVertexBuffer(
                bgfx::copy(cubeVerts.data(),
                    static_cast<uint32_t>(cubeVerts.size() * sizeof(PosColorVertex))),
                PosColorVertex::layout);
            fallbackCubeIBH = bgfx::createIndexBuffer(
                bgfx::copy(cubeIndices.data(),
                    static_cast<uint32_t>(cubeIndices.size() * sizeof(uint32_t))),
                BGFX_BUFFER_INDEX32);
        }

        // Create bgfx texture handles with mip chains from loaded object texture images
        for (auto &kv : objTexImages) {
            const auto &img = kv.second;
            objTextureHandles[kv.first] = createMipmappedTexture(
                img.rgba.data(), img.width, img.height,
                BGFX_SAMPLER_MIN_POINT | BGFX_SAMPLER_MAG_POINT
                | BGFX_SAMPLER_U_MIRROR | BGFX_SAMPLER_V_MIRROR,
                MipAlphaMode::ALPHA_TEST, linearMips, sharpMips);
        }
        if (!objTextureHandles.empty()) {
            std::fprintf(stderr, "Created %zu object texture GPU handles\n",
                         objTextureHandles.size());
        }

        // Create GPU buffers for each parsed .bin model
        for (auto &kv : parsedModels) {
            const std::string &name = kv.first;
            const Darkness::ParsedBinMesh &mesh = kv.second;

            if (mesh.vertices.empty() || mesh.indices.empty()) continue;

            // Build a vertex-to-material map so we can colour vertices appropriately.
            // Each submesh covers a range of indices; map each index to its material.
            std::vector<int> vertexMatIndex(mesh.vertices.size(), -1);
            for (const auto &sm : mesh.subMeshes) {
                int mi = sm.matIndex;
                for (uint32_t ii = sm.firstIndex; ii < sm.firstIndex + sm.indexCount; ++ii) {
                    if (ii < mesh.indices.size()) {
                        uint32_t vi = mesh.indices[ii];
                        if (vi < vertexMatIndex.size()) {
                            vertexMatIndex[vi] = mi;
                        }
                    }
                }
            }

            // Fallback colour for non-textured models or Darkness::MD_MAT_COLOR materials
            uint32_t fallbackColor = colorFromName(name);

            // Build set of material indices that have actually-loaded textures,
            // so we only assign white to vertices that will be textured at draw time.
            std::unordered_set<int> loadedMatIndices;
            for (int mi = 0; mi < static_cast<int>(mesh.materials.size()); ++mi) {
                if (mesh.materials[mi].type == Darkness::MD_MAT_TMAP) {
                    std::string lname(mesh.materials[mi].name);
                    std::transform(lname.begin(), lname.end(), lname.begin(),
                                   [](unsigned char c) { return std::tolower(c); });
                    if (objTexImages.count(lname)) {
                        loadedMatIndices.insert(mi);
                    }
                }
            }

            // Convert BinVert -> PosColorUVVertex
            std::vector<PosColorUVVertex> gpuVerts(mesh.vertices.size());
            for (size_t i = 0; i < mesh.vertices.size(); ++i) {
                const auto &bv = mesh.vertices[i];

                // Apply simple directional lighting using the vertex normal
                float dot = bv.nx * 0.3f + bv.ny * 0.8f + bv.nz * 0.4f;
                float brightness = std::max(dot, 0.0f) * 0.7f + 0.3f;

                // Determine vertex colour based on material type
                uint32_t vertColor;
                int mi = vertexMatIndex[i];
                if (mi >= 0 && loadedMatIndices.count(mi)) {
                    // Textured material with loaded texture: white * brightness
                    // (texture will modulate the final colour at draw time)
                    vertColor = packABGR(brightness, brightness, brightness);
                } else if (mi >= 0 && mi < static_cast<int>(mesh.materials.size()) &&
                           mesh.materials[mi].type == Darkness::MD_MAT_COLOR) {
                    // Solid-colour material: BGRA from material data * brightness
                    const uint8_t *c = mesh.materials[mi].colour;
                    vertColor = packABGR(c[2] / 255.0f * brightness,   // R (colour is BGRA)
                                         c[1] / 255.0f * brightness,   // G
                                         c[0] / 255.0f * brightness);  // B
                } else {
                    // No loaded texture, unknown material, or palette — use hash colour
                    uint8_t r = fallbackColor & 0xFF;
                    uint8_t g = (fallbackColor >> 8) & 0xFF;
                    uint8_t b = (fallbackColor >> 16) & 0xFF;
                    vertColor = packABGR(r / 255.0f * brightness,
                                         g / 255.0f * brightness,
                                         b / 255.0f * brightness);
                }

                gpuVerts[i] = {
                    bv.x, bv.y, bv.z,
                    vertColor,
                    bv.u, bv.v
                };
            }

            ObjectModelGPU gpu;
            gpu.vbh = bgfx::createVertexBuffer(
                bgfx::copy(gpuVerts.data(),
                    static_cast<uint32_t>(gpuVerts.size() * sizeof(PosColorUVVertex))),
                PosColorUVVertex::layout);
            gpu.ibh = bgfx::createIndexBuffer(
                bgfx::copy(mesh.indices.data(),
                    static_cast<uint32_t>(mesh.indices.size() * sizeof(uint32_t))),
                BGFX_BUFFER_INDEX32);

            // Build per-submesh GPU draw info from parsed submeshes
            for (const auto &sm : mesh.subMeshes) {
                ObjectSubMeshGPU gsm;
                gsm.firstIndex = sm.firstIndex;
                gsm.indexCount = sm.indexCount;
                gsm.textured = false;
                if (sm.matIndex >= 0 && sm.matIndex < static_cast<int>(mesh.materials.size())) {
                    const auto &mat = mesh.materials[sm.matIndex];
                    gsm.textured = (mat.type == Darkness::MD_MAT_TMAP);
                    // Lowercase material name for texture lookup
                    gsm.matName = mat.name;
                    std::transform(gsm.matName.begin(), gsm.matName.end(),
                                   gsm.matName.begin(),
                                   [](unsigned char c) { return std::tolower(c); });
                }
                gpu.subMeshes.push_back(gsm);
            }

            gpu.valid = true;
            objModelGPU[name] = std::move(gpu);
        }
        std::fprintf(stderr, "Object GPU buffers: %zu models + fallback cube\n",
                     objModelGPU.size());
    }

    // ── Create sky dome GPU buffers ──
    bgfx::VertexBufferHandle skyVBH = BGFX_INVALID_HANDLE;
    bgfx::IndexBufferHandle skyIBH = BGFX_INVALID_HANDLE;
    uint32_t skyIndexCount = 0;

    if (!skyDome.vertices.empty()) {
        skyVBH = bgfx::createVertexBuffer(
            bgfx::copy(skyDome.vertices.data(),
                static_cast<uint32_t>(skyDome.vertices.size() * sizeof(PosColorVertex))),
            PosColorVertex::layout);
        skyIBH = bgfx::createIndexBuffer(
            bgfx::copy(skyDome.indices.data(),
                static_cast<uint32_t>(skyDome.indices.size() * sizeof(uint16_t))));
        skyIndexCount = static_cast<uint32_t>(skyDome.indices.size());
    }

    // ── Create textured skybox GPU buffers (old sky system) ──
    bgfx::VertexBufferHandle skyboxVBH = BGFX_INVALID_HANDLE;
    bgfx::IndexBufferHandle skyboxIBH = BGFX_INVALID_HANDLE;
    SkyboxCube skyboxCube;
    std::unordered_map<std::string, bgfx::TextureHandle> skyboxTexHandles;

    if (hasSkybox) {
        skyboxCube = buildSkyboxCube();

        skyboxVBH = bgfx::createVertexBuffer(
            bgfx::copy(skyboxCube.vertices.data(),
                static_cast<uint32_t>(skyboxCube.vertices.size() * sizeof(PosColorUVVertex))),
            PosColorUVVertex::layout);
        skyboxIBH = bgfx::createIndexBuffer(
            bgfx::copy(skyboxCube.indices.data(),
                static_cast<uint32_t>(skyboxCube.indices.size() * sizeof(uint16_t))));

        // Create GPU textures with mip chains for each loaded skybox face
        // Skybox faces are fully opaque so use ALPHA_BLEND (no coverage preservation)
        for (auto &kv : skyboxImages) {
            const auto &img = kv.second;
            skyboxTexHandles[kv.first] = createMipmappedTexture(
                img.rgba.data(), img.width, img.height,
                BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP,
                MipAlphaMode::ALPHA_BLEND, linearMips, sharpMips);
        }
        std::fprintf(stderr, "Skybox GPU: %zu face textures created\n",
                     skyboxTexHandles.size());
    }

    const char *modeStr = lightmappedMode ? "lightmapped" :
                          texturedMode ? "textured" : "flat-shaded";
    std::fprintf(stderr, "Render window opened (%dx%d, Metal, %s)\n",
                 WINDOW_WIDTH, WINDOW_HEIGHT, modeStr);
    std::fprintf(stderr, "Portal culling: %s (toggle with C key)\n",
                 portalCulling ? "ON" : "OFF");

    // Use spawn point if found, otherwise fall back to centroid
    float spawnX = camX, spawnY = camY, spawnZ = camZ;
    float spawnYaw = 0.0f;
    if (spawnInfo.found) {
        spawnX = spawnInfo.x;
        spawnY = spawnInfo.y;
        spawnZ = spawnInfo.z;
        spawnYaw = spawnInfo.yaw;
        std::fprintf(stderr, "Camera at spawn (%.1f, %.1f, %.1f)\n",
                     spawnX, spawnY, spawnZ);
    } else {
        std::fprintf(stderr, "Camera at centroid (%.1f, %.1f, %.1f)\n",
                     camX, camY, camZ);
    }

    std::fprintf(stderr, "Controls: WASD=move, mouse=look, Space/LShift=up/down, "
                 "scroll=speed, C=toggle culling, Home=spawn, Esc=quit\n");

    Camera cam;
    cam.init(spawnX, spawnY, spawnZ);
    cam.yaw = spawnYaw;

    SDL_SetRelativeMouseMode(SDL_TRUE);

    float moveSpeed = 20.0f; // adjustable via scroll wheel
    const float MOUSE_SENS = 0.002f;
    const float PI = 3.14159265f;

    // Portal culling stats for title bar display
    uint32_t cullVisibleCells = 0;  // updated each frame when culling is on
    uint32_t cullTotalCells = wrData.numCells;

    // Helper to update window title with current speed, culling, and filter stats
    auto updateTitle = [&]() {
        char title[256];
        const char *filterNames[] = { "point", "bilinear", "trilinear", "aniso" };
        const char *filterStr = filterNames[filterMode % 4];
        if (portalCulling) {
            std::snprintf(title, sizeof(title),
                "darkness — %s [speed: %.1f] [cull: %u/%u cells] [%s]",
                modeStr, moveSpeed, cullVisibleCells, cullTotalCells, filterStr);
        } else {
            std::snprintf(title, sizeof(title),
                "darkness — %s [speed: %.1f] [cull: OFF] [%s]",
                modeStr, moveSpeed, filterStr);
        }
        SDL_SetWindowTitle(window, title);
    };
    updateTitle();

    uint64_t renderState = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
                         | BGFX_STATE_WRITE_Z | BGFX_STATE_DEPTH_TEST_LESS
                         | BGFX_STATE_CULL_CW;

    auto lastTime = std::chrono::high_resolution_clock::now();
    float waterElapsed = 0.0f;

    bool running = true;
    while (running) {
        auto now = std::chrono::high_resolution_clock::now();
        float dt = std::chrono::duration<float>(now - lastTime).count();
        lastTime = now;
        dt = std::min(dt, 0.1f);
        waterElapsed += dt;

        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT) {
                running = false;
            } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_ESCAPE) {
                running = false;
            } else if (ev.type == SDL_MOUSEMOTION) {
                cam.yaw   -= ev.motion.xrel * MOUSE_SENS;
                cam.pitch -= ev.motion.yrel * MOUSE_SENS;
                cam.pitch = std::max(-PI * 0.49f, std::min(PI * 0.49f, cam.pitch));
            } else if (ev.type == SDL_MOUSEWHEEL) {
                // Scroll wheel adjusts movement speed: 1.5x per tick
                if (ev.wheel.y > 0) {
                    for (int i = 0; i < ev.wheel.y; ++i)
                        moveSpeed *= 1.5f;
                } else if (ev.wheel.y < 0) {
                    for (int i = 0; i < -ev.wheel.y; ++i)
                        moveSpeed /= 1.5f;
                }
                moveSpeed = std::max(1.0f, std::min(500.0f, moveSpeed));
                updateTitle();
            } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_HOME) {
                // Teleport back to spawn point
                cam.pos[0] = spawnX;
                cam.pos[1] = spawnY;
                cam.pos[2] = spawnZ;
                cam.yaw = spawnYaw;
                cam.pitch = 0;
                std::fprintf(stderr, "Teleported to spawn (%.1f, %.1f, %.1f)\n",
                             spawnX, spawnY, spawnZ);
            } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_c) {
                // Toggle portal culling
                portalCulling = !portalCulling;
                std::fprintf(stderr, "Portal culling: %s\n",
                             portalCulling ? "ON" : "OFF");
            } else if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_f) {
                // Cycle texture filtering: point → bilinear → trilinear → anisotropic
                filterMode = (filterMode + 1) % 4;
                const char *filterNames[] = {
                    "point (crispy)", "bilinear", "trilinear", "anisotropic"
                };
                std::fprintf(stderr, "Texture filtering: %s\n", filterNames[filterMode]);
            }
        }

        // Keyboard movement
        const Uint8 *keys = SDL_GetKeyboardState(nullptr);
        float forward = 0, right = 0, up = 0;
        float speed = moveSpeed;
        if (keys[SDL_SCANCODE_LCTRL] || keys[SDL_SCANCODE_RCTRL])
            speed *= 3.0f;

        if (keys[SDL_SCANCODE_W]) forward += speed * dt;
        if (keys[SDL_SCANCODE_S]) forward -= speed * dt;
        if (keys[SDL_SCANCODE_D]) right   += speed * dt;
        if (keys[SDL_SCANCODE_A]) right   -= speed * dt;
        if (keys[SDL_SCANCODE_SPACE])  up += speed * dt;
        if (keys[SDL_SCANCODE_LSHIFT]) up -= speed * dt;
        if (keys[SDL_SCANCODE_Q]) up += speed * dt;
        if (keys[SDL_SCANCODE_E]) up -= speed * dt;

        cam.move(forward, right, up);

        // ── Animated lightmap update ──
        // Advance all light animation timers and re-blend changed lightmaps
        if (lightmappedMode && !lmAtlasSet.atlases.empty()) {
            bool anyLightChanged = false;
            std::unordered_map<int16_t, float> currentIntensities;

            for (auto &[lightNum, light] : lightSources) {
                bool changed = Darkness::updateLightAnimation(light, dt);
                float intensity = (light.maxBright > 0.0f)
                    ? light.brightness / light.maxBright : 0.0f;
                currentIntensities[lightNum] = intensity;
                if (changed) anyLightChanged = true;
            }

            // Re-blend changed lightmaps into atlas CPU buffer
            if (anyLightChanged) {
                for (auto &[lightNum, light] : lightSources) {
                    float intensity = currentIntensities[lightNum];
                    if (std::abs(intensity - light.prevIntensity) < 0.002f) continue;
                    light.prevIntensity = intensity;

                    auto it = animLightIndex.find(lightNum);
                    if (it == animLightIndex.end()) continue;

                    for (auto &[ci, pi] : it->second) {
                        Darkness::blendAnimatedLightmap(
                            lmAtlasSet.atlases[0], wrData, ci, pi,
                            lmAtlasSet.entries[ci][pi],
                            currentIntensities, lmScale);
                    }
                }

                // Upload full atlas to GPU
                const auto &atlas = lmAtlasSet.atlases[0];
                const bgfx::Memory *mem = bgfx::copy(
                    atlas.rgba.data(), static_cast<uint32_t>(atlas.rgba.size()));
                bgfx::updateTexture2D(lightmapAtlasHandles[0], 0, 0, 0, 0,
                    static_cast<uint16_t>(atlas.size),
                    static_cast<uint16_t>(atlas.size), mem);
            }
        }

        // Set up projection matrix (shared by both views)
        float proj[16];
        bx::mtxProj(proj, 60.0f,
                     float(WINDOW_WIDTH) / float(WINDOW_HEIGHT),
                     0.1f, 5000.0f,
                     bgfx::getCaps()->homogeneousDepth,
                     bx::Handedness::Right);

        // Texture sampler flags for the current filtering mode (shared by both views).
        // UINT32_MAX = use texture's baked POINT flags (default).
        // Other modes override with explicit sampler flags per draw call.
        // Two variants: MIRROR wrap (world/object textures), CLAMP wrap (skybox).
        uint32_t texSampler, skySampler;
        switch (filterMode) {
        case 0: // Point: use texture's baked POINT flags
            texSampler = UINT32_MAX;
            skySampler = UINT32_MAX;
            break;
        case 1: // Bilinear: linear min/mag, point mip
            texSampler = BGFX_SAMPLER_U_MIRROR | BGFX_SAMPLER_V_MIRROR
                       | BGFX_SAMPLER_MIP_POINT;
            skySampler = BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP
                       | BGFX_SAMPLER_MIP_POINT;
            break;
        case 2: // Trilinear: linear min/mag/mip
            texSampler = BGFX_SAMPLER_U_MIRROR | BGFX_SAMPLER_V_MIRROR;
            skySampler = BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP;
            break;
        case 3: // Anisotropic: aniso min/mag, linear mip
            texSampler = BGFX_SAMPLER_U_MIRROR | BGFX_SAMPLER_V_MIRROR
                       | BGFX_SAMPLER_MIN_ANISOTROPIC | BGFX_SAMPLER_MAG_ANISOTROPIC;
            skySampler = BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP
                       | BGFX_SAMPLER_MIN_ANISOTROPIC | BGFX_SAMPLER_MAG_ANISOTROPIC;
            break;
        default:
            texSampler = UINT32_MAX;
            skySampler = UINT32_MAX;
            break;
        }

        // ── Underwater detection ──
        // Check if the camera is inside a water cell (mediaType==2).
        // When submerged, override fog with short-range blue-green water fog.
        bool underwater = (getCameraMediaType(wrData, cam.pos[0], cam.pos[1], cam.pos[2]) == 2);

        // Water fog defaults: dark blue-green tint, short visibility range
        static constexpr float waterFogR = 0.10f, waterFogG = 0.18f, waterFogB = 0.25f;
        static constexpr float waterFogDist = 80.0f;

        // ── Fog uniform values (reused before every bgfx::submit) ──
        // bgfx uniforms are per-submit: cleared after each draw call.
        // We set them before every submit so all shaders receive fog data.
        // When underwater, override with water fog regardless of FOG chunk.
        float fogColorArr[4], fogOnArr[4];
        if (underwater) {
            fogColorArr[0] = waterFogR; fogColorArr[1] = waterFogG;
            fogColorArr[2] = waterFogB; fogColorArr[3] = 1.0f;
            fogOnArr[0] = 1.0f; fogOnArr[1] = waterFogDist;
            fogOnArr[2] = 0.0f; fogOnArr[3] = 0.0f;
        } else {
            fogColorArr[0] = fogParams.r; fogColorArr[1] = fogParams.g;
            fogColorArr[2] = fogParams.b; fogColorArr[3] = 1.0f;
            fogOnArr[0] = fogParams.enabled ? 1.0f : 0.0f;
            fogOnArr[1] = fogParams.distance;
            fogOnArr[2] = 0.0f; fogOnArr[3] = 0.0f;
        }
        float fogOffArr[4] = { 0.0f, 1.0f, 0.0f, 0.0f };
        // Sky fog: underwater always fogs sky; otherwise respect SKYOBJVAR.fog
        bool skyFogged = underwater || (fogParams.enabled && skyParams.fog);
        float *skyFogArr = skyFogged ? fogOnArr : fogOffArr;

        // Update clear color per-frame for underwater tinting
        if (underwater) {
            uint8_t cr = static_cast<uint8_t>(waterFogR * 255.0f);
            uint8_t cg = static_cast<uint8_t>(waterFogG * 255.0f);
            uint8_t cb = static_cast<uint8_t>(waterFogB * 255.0f);
            uint32_t waterClear = (uint32_t(cr) << 24) | (uint32_t(cg) << 16)
                                | (uint32_t(cb) << 8) | 0xFF;
            bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
                                waterClear, 1.0f, 0);
        } else {
            bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
                                skyClearColor, 1.0f, 0);
        }

        // Helper: set fog uniforms before each submit call
        // (bgfx clears all uniform state after each submit)
        auto setFogOn = [&]() {
            bgfx::setUniform(u_fogColor, fogColorArr);
            bgfx::setUniform(u_fogParams, fogOnArr);
        };
        auto setFogSky = [&]() {
            bgfx::setUniform(u_fogColor, fogColorArr);
            bgfx::setUniform(u_fogParams, skyFogArr);
        };

        // ── View 0: Sky pass ──
        // Sky rendered with rotation-only view (no translation),
        // no depth write/test — appears infinitely far behind everything.
        {
            float skyView[16];
            cam.getSkyViewMatrix(skyView);
            bgfx::setViewTransform(0, skyView, proj);

            float skyModel[16];
            bx::mtxIdentity(skyModel);

            // Cull CCW (not CW) because the camera is inside the cube/sphere —
            // we see the back faces, so cull the outward-facing front faces.
            uint64_t skyState = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
                              | BGFX_STATE_CULL_CCW;

            if (hasSkybox && bgfx::isValid(skyboxVBH)) {
                // Textured skybox (old sky system) — render each face with its texture
                for (const auto &face : skyboxCube.faces) {
                    auto texIt = skyboxTexHandles.find(face.key);
                    if (texIt == skyboxTexHandles.end()) continue;

                    setFogSky();
                    bgfx::setTransform(skyModel);
                    bgfx::setVertexBuffer(0, skyboxVBH);
                    bgfx::setIndexBuffer(skyboxIBH, face.firstIndex, face.indexCount);
                    bgfx::setState(skyState);
                    bgfx::setTexture(0, s_texColor, texIt->second, skySampler);
                    bgfx::submit(0, texturedProgram);
                }
            } else if (bgfx::isValid(skyVBH)) {
                // Procedural dome (new sky system) — vertex-coloured hemisphere
                setFogSky();
                bgfx::setTransform(skyModel);
                bgfx::setVertexBuffer(0, skyVBH);
                bgfx::setIndexBuffer(skyIBH);
                bgfx::setState(skyState);
                bgfx::submit(0, flatProgram);
            }
        }

        // ── View 1: World + objects pass ──
        float view[16];
        cam.getViewMatrix(view);
        bgfx::setViewTransform(1, view, proj);

        // ── Portal culling: determine visible cells ──
        // Build view-projection matrix and extract frustum planes for portal tests.
        // When culling is disabled, visibleCells remains empty (skip filtering).
        std::unordered_set<uint32_t> visibleCells;
        if (portalCulling) {
            float vp[16];
            bx::mtxMul(vp, view, proj);
            ViewFrustum frustum;
            frustum.extractFromVP(vp);

            int32_t camCell = findCameraCell(wrData, cam.pos[0], cam.pos[1], cam.pos[2]);
            visibleCells = portalBFS(wrData, cellPortals, camCell, frustum,
                                     cam.pos[0], cam.pos[1], cam.pos[2]);
            cullVisibleCells = static_cast<uint32_t>(visibleCells.size());
        }
        // Update title bar with culling stats every frame
        updateTitle();

        // Identity model transform for world geometry
        float model[16];
        bx::mtxIdentity(model);

        // Lambda: check if a cell should be rendered (culling filter)
        auto isCellVisible = [&](uint32_t cellID) -> bool {
            if (!portalCulling) return true; // culling off: render everything
            return visibleCells.count(cellID) > 0;
        };

        if (lightmappedMode) {
            for (const auto &grp : lmMesh.groups) {
                if (!isCellVisible(grp.cellID)) continue;

                setFogOn();
                bgfx::setTransform(model);
                bgfx::setVertexBuffer(0, vbh);
                bgfx::setIndexBuffer(ibh, grp.firstIndex, grp.numIndices);
                bgfx::setState(renderState);

                if (grp.txtIndex == 0) {
                    bgfx::submit(1, flatProgram);
                } else {
                    auto it = textureHandles.find(grp.txtIndex);
                    if (it != textureHandles.end()) {
                        bgfx::setTexture(0, s_texColor, it->second, texSampler);
                        if (!lightmapAtlasHandles.empty())
                            bgfx::setTexture(1, s_texLightmap, lightmapAtlasHandles[0]);
                        bgfx::submit(1, lightmappedProgram);
                    } else {
                        bgfx::submit(1, flatProgram);
                    }
                }
            }
        } else if (texturedMode) {
            for (const auto &grp : worldMesh.groups) {
                if (!isCellVisible(grp.cellID)) continue;

                setFogOn();
                bgfx::setTransform(model);
                bgfx::setVertexBuffer(0, vbh);
                bgfx::setIndexBuffer(ibh, grp.firstIndex, grp.numIndices);
                bgfx::setState(renderState);

                if (grp.txtIndex == 0) {
                    bgfx::submit(1, flatProgram);
                } else {
                    auto it = textureHandles.find(grp.txtIndex);
                    if (it != textureHandles.end()) {
                        bgfx::setTexture(0, s_texColor, it->second, texSampler);
                        bgfx::submit(1, texturedProgram);
                    } else {
                        bgfx::submit(1, flatProgram);
                    }
                }
            }
        } else {
            for (const auto &grp : flatMesh.groups) {
                if (!isCellVisible(grp.cellID)) continue;

                setFogOn();
                bgfx::setTransform(model);
                bgfx::setVertexBuffer(0, vbh);
                bgfx::setIndexBuffer(ibh, grp.firstIndex, grp.numIndices);
                bgfx::setState(renderState);
                bgfx::submit(1, flatProgram);
            }
        }

        // ── Draw object meshes (per-submesh for textured/solid materials) ──
        if (showObjects) {
            for (size_t oi = 0; oi < objData.objects.size(); ++oi) {
                const auto &obj = objData.objects[oi];
                if (!obj.hasPosition) continue;

                // Portal culling: skip objects in non-visible cells.
                // Objects outside all cells (cellID == -1) are always rendered.
                if (portalCulling && oi < objCellIDs.size()) {
                    int32_t objCell = objCellIDs[oi];
                    if (objCell >= 0 && visibleCells.count(static_cast<uint32_t>(objCell)) == 0)
                        continue;
                }

                // Compute per-object model matrix from position + angles
                float objMtx[16];
                buildModelMatrix(objMtx, obj.x, obj.y, obj.z,
                                 obj.heading, obj.pitch, obj.bank,
                                 obj.scaleX, obj.scaleY, obj.scaleZ);

                // Find the GPU model for this object's model name
                std::string modelName(obj.modelName);
                auto it = objModelGPU.find(modelName);

                if (it != objModelGPU.end() && it->second.valid) {
                    const auto &gpuModel = it->second;

                    // Draw each submesh with appropriate shader/texture
                    for (const auto &sm : gpuModel.subMeshes) {
                        if (sm.indexCount == 0) continue;

                        setFogOn();
                        bgfx::setTransform(objMtx);
                        bgfx::setVertexBuffer(0, gpuModel.vbh);
                        bgfx::setIndexBuffer(gpuModel.ibh, sm.firstIndex, sm.indexCount);
                        bgfx::setState(renderState);

                        if (sm.textured) {
                            // Look up object texture by material name
                            auto texIt = objTextureHandles.find(sm.matName);
                            if (texIt != objTextureHandles.end()) {
                                bgfx::setTexture(0, s_texColor, texIt->second, texSampler);
                                bgfx::submit(1, texturedProgram);
                            } else {
                                // Texture not loaded — fall back to flat shading
                                // (vertex colour is white*brightness, so it will appear grey)
                                bgfx::submit(1, flatProgram);
                            }
                        } else {
                            // Solid-colour submesh — vertex colour carries material colour
                            bgfx::submit(1, flatProgram);
                        }
                    }
                } else {
                    // Fallback: small coloured cube for missing models
                    setFogOn();
                    bgfx::setTransform(objMtx);
                    bgfx::setVertexBuffer(0, fallbackCubeVBH);
                    bgfx::setIndexBuffer(fallbackCubeIBH);
                    bgfx::setState(renderState);
                    bgfx::submit(1, flatProgram);
                }
            }
        }

        // ── Water surfaces: alpha-blended, no depth write, double-sided ──
        // Rendered last so all opaque geometry is already in the depth buffer.
        // Per-texture-group rendering: textured water uses waterProgram (no discard),
        // non-textured water falls back to flatProgram with blue-green vertex color.
        if (hasWater) {
            float identity[16];
            bx::mtxIdentity(identity);
            // Alpha blend, depth test (read) but no depth write, no face culling
            uint64_t waterState = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
                | BGFX_STATE_DEPTH_TEST_LESS
                | BGFX_STATE_BLEND_ALPHA;
            // No BGFX_STATE_WRITE_Z — water doesn't occlude geometry behind it
            // No BGFX_STATE_CULL_* — water is visible from both sides

            for (const auto &grp : waterMesh.groups) {
                setFogOn();
                bgfx::setTransform(identity);
                bgfx::setVertexBuffer(0, waterVBH);
                bgfx::setIndexBuffer(waterIBH, grp.firstIndex, grp.numIndices);
                bgfx::setState(waterState);

                if (grp.txtIndex != 0 || grp.flowGroup > 0) {
                    // Textured water: water shader with vertex displacement + UV distortion
                    // u_waterParams: x=elapsed time, y=scroll speed, z=wave amplitude, w=UV distortion
                    float waterParams[4] = { waterElapsed, waterScrollSpeed, waveAmplitude, uvDistortion };
                    bgfx::setUniform(u_waterParams, waterParams);

                    // u_waterFlow: x=rotation speed, y=use_world_uv flag, z=tex_unit_len
                    // Flow-textured water: vertex shader computes UVs from world position
                    // with rotation (matching Dark Engine's flow animation system).
                    // TXLIST-textured water: uses pre-computed UVs from mesh.
                    bool isFlowTextured = (grp.flowGroup > 0);
                    float useWorldUV = isFlowTextured ? 1.0f : 0.0f;
                    constexpr float TEX_UNIT_LEN = 4.0f; // 4 world units per texture repeat
                    float waterFlow[4] = { waterRotation, useWorldUV, TEX_UNIT_LEN, 0.0f };
                    bgfx::setUniform(u_waterFlow, waterFlow);

                    // Resolve texture: flow texture by group first, then TXLIST by index
                    bgfx::TextureHandle tex = BGFX_INVALID_HANDLE;
                    if (grp.flowGroup > 0) {
                        auto fit = flowTextureHandles.find(grp.flowGroup);
                        if (fit != flowTextureHandles.end())
                            tex = fit->second;
                    }
                    if (!bgfx::isValid(tex) && grp.txtIndex != 0) {
                        auto it = textureHandles.find(grp.txtIndex);
                        if (it != textureHandles.end())
                            tex = it->second;
                    }

                    if (bgfx::isValid(tex)) {
                        bgfx::setTexture(0, s_texColor, tex, texSampler);
                        bgfx::submit(1, waterProgram);
                    } else {
                        bgfx::submit(1, flatProgram);
                    }
                } else {
                    // Non-textured water: flat blue-green from vertex color
                    bgfx::submit(1, flatProgram);
                }
            }
        }

        bgfx::frame();
    }

    // Cleanup — water surface buffers and flow textures
    if (bgfx::isValid(waterVBH)) bgfx::destroy(waterVBH);
    if (bgfx::isValid(waterIBH)) bgfx::destroy(waterIBH);
    for (auto &kv : flowTextureHandles)
        bgfx::destroy(kv.second);

    // Cleanup — textured skybox buffers
    if (bgfx::isValid(skyboxVBH)) bgfx::destroy(skyboxVBH);
    if (bgfx::isValid(skyboxIBH)) bgfx::destroy(skyboxIBH);
    for (auto &kv : skyboxTexHandles)
        bgfx::destroy(kv.second);

    // Cleanup — sky dome buffers
    if (bgfx::isValid(skyVBH)) bgfx::destroy(skyVBH);
    if (bgfx::isValid(skyIBH)) bgfx::destroy(skyIBH);

    // Cleanup — object GPU buffers and textures
    for (auto &kv : objModelGPU) {
        if (bgfx::isValid(kv.second.vbh)) bgfx::destroy(kv.second.vbh);
        if (bgfx::isValid(kv.second.ibh)) bgfx::destroy(kv.second.ibh);
    }
    for (auto &kv : objTextureHandles) {
        bgfx::destroy(kv.second);
    }
    if (bgfx::isValid(fallbackCubeVBH)) bgfx::destroy(fallbackCubeVBH);
    if (bgfx::isValid(fallbackCubeIBH)) bgfx::destroy(fallbackCubeIBH);

    // Cleanup — world geometry and textures
    for (auto &h : lightmapAtlasHandles)
        bgfx::destroy(h);
    for (auto &kv : textureHandles)
        bgfx::destroy(kv.second);
    bgfx::destroy(s_texLightmap);
    bgfx::destroy(s_texColor);
    bgfx::destroy(u_waterParams);
    bgfx::destroy(u_waterFlow);
    bgfx::destroy(u_fogColor);
    bgfx::destroy(u_fogParams);

    bgfx::destroy(ibh);
    bgfx::destroy(vbh);
    bgfx::destroy(flatProgram);
    bgfx::destroy(texturedProgram);
    bgfx::destroy(lightmappedProgram);
    bgfx::destroy(waterProgram);

    bgfx::shutdown();
    SDL_DestroyWindow(window);
    SDL_Quit();

    std::fprintf(stderr, "Clean shutdown.\n");
    return 0;
}
