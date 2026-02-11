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

// DarknessRendererCore.h — Low-level rendering primitives and infrastructure
//
// Vertex formats, texture creation pipeline, camera, collision, portal culling,
// object GPU types, and fog data. These are reusable building blocks that any
// render pass would need.

#pragma once

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <vector>
#include <string>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <queue>

#include <bgfx/bgfx.h>
#include <bx/math.h>

#include "WRChunkParser.h"

namespace Darkness {

// ── Vertex layouts ──

struct PosColorVertex {
    float x, y, z;
    uint32_t abgr;

    inline static bgfx::VertexLayout layout;

    static void init() {
        layout.begin()
            .add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
            .add(bgfx::Attrib::Color0, 4, bgfx::AttribType::Uint8, true)
            .end();
    }
};

struct PosColorUVVertex {
    float x, y, z;
    uint32_t abgr;
    float u, v;

    inline static bgfx::VertexLayout layout;

    static void init() {
        layout.begin()
            .add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
            .add(bgfx::Attrib::Color0, 4, bgfx::AttribType::Uint8, true)
            .add(bgfx::Attrib::TexCoord0, 2, bgfx::AttribType::Float)
            .end();
    }
};

struct PosUV2Vertex {
    float x, y, z;
    float u0, v0; // diffuse texture UV
    float u1, v1; // lightmap atlas UV

    inline static bgfx::VertexLayout layout;

    static void init() {
        layout.begin()
            .add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
            .add(bgfx::Attrib::TexCoord0, 2, bgfx::AttribType::Float)
            .add(bgfx::Attrib::TexCoord1, 2, bgfx::AttribType::Float)
            .end();
    }
};

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

// ── Texture dimensions for UV normalization ──

struct TexDimensions {
    uint32_t w, h;
};

// ── Color packing ──

inline uint32_t packABGR(float r, float g, float b) {
    uint8_t ri = static_cast<uint8_t>(std::min(std::max(r, 0.0f), 1.0f) * 255.0f);
    uint8_t gi = static_cast<uint8_t>(std::min(std::max(g, 0.0f), 1.0f) * 255.0f);
    uint8_t bi = static_cast<uint8_t>(std::min(std::max(b, 0.0f), 1.0f) * 255.0f);
    return 0xff000000u | (bi << 16) | (gi << 8) | ri;
}

// Overload with explicit alpha for translucent geometry (e.g. water surfaces)
inline uint32_t packABGR(float r, float g, float b, float a) {
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
inline float srgbToLinear(uint8_t v) {
    float f = v / 255.0f;
    return std::pow(f, 2.2f);
}

inline uint8_t linearToSrgb(float v) {
    float f = std::pow(std::max(0.0f, std::min(1.0f, v)), 1.0f / 2.2f);
    return static_cast<uint8_t>(f * 255.0f + 0.5f);
}

// ── Kaiser-windowed sinc filter for high-quality mipmap downsampling ──
// Replaces the naive 2x2 box filter with a wider kernel that reduces aliasing
// on high-frequency textures. Uses premultiplied alpha to prevent dark halos
// at transparent edges.

// Modified Bessel function I0 — polynomial approximation (Abramowitz & Stegun 9.8.1)
inline double besselI0(double x) {
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
inline double kaiserWindow(double x, double radius, double beta) {
    if (std::abs(x) > radius) return 0.0;
    double r = x / radius;
    return besselI0(beta * std::sqrt(std::max(0.0, 1.0 - r * r))) / besselI0(beta);
}

// Normalized sinc function: sin(pi*x) / (pi*x)
inline double sincFilter(double x) {
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
inline std::vector<uint8_t> downsampleKaiser(const uint8_t *src,
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
inline float computeAlphaCoverage(const uint8_t *rgba, uint32_t w, uint32_t h,
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
inline void preserveAlphaCoverage(uint8_t *rgba, uint32_t w, uint32_t h,
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

inline void sharpenMipLevel(uint8_t *rgba, uint32_t w, uint32_t h, float strength) {
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
inline bgfx::TextureHandle createMipmappedTexture(
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

// ── Object mesh rendering ──

// Per-material submesh draw range within an object model's index buffer
struct ObjectSubMeshGPU {
    uint32_t firstIndex;
    uint32_t indexCount;
    std::string matName;   // lowercase material name, for texture lookup
    bool textured;         // true = Darkness::MD_MAT_TMAP, false = Darkness::MD_MAT_COLOR
    float matTrans;        // per-material translucency from .bin mat_extra (0=opaque)
};

// GPU buffers for a single unique model (shared by all instances)
struct ObjectModelGPU {
    bgfx::VertexBufferHandle vbh = BGFX_INVALID_HANDLE;
    bgfx::IndexBufferHandle ibh = BGFX_INVALID_HANDLE;
    std::vector<ObjectSubMeshGPU> subMeshes;
    bool valid = false;
};

// Generate a stable colour from a model name hash (for flat-shaded objects)
inline uint32_t colorFromName(const std::string &name) {
    // Simple hash -> hue mapping for distinct per-model colours
    uint32_t hash = 5381;
    for (char c : name) hash = hash * 33 + static_cast<uint8_t>(c);

    // HSV -> RGB with S=0.6, V=0.85 for pleasant pastel tones
    float hue = static_cast<float>(hash % 360);
    float s = 0.6f, v = 0.85f;
    float c_ = v * s;
    float x = c_ * (1.0f - std::fabs(std::fmod(hue / 60.0f, 2.0f) - 1.0f));
    float m = v - c_;
    float r, g, b;
    if (hue < 60)       { r = c_; g = x; b = 0; }
    else if (hue < 120) { r = x; g = c_; b = 0; }
    else if (hue < 180) { r = 0; g = c_; b = x; }
    else if (hue < 240) { r = 0; g = x; b = c_; }
    else if (hue < 300) { r = x; g = 0; b = c_; }
    else                { r = c_; g = 0; b = x; }
    return packABGR(r + m, g + m, b + m);
}

// ── Camera cell detection ──
// Find which cell the camera is in. Returns cell index, or -1 if outside
// all cells. When the camera is near a water boundary, it may be inside
// both an air and water cell — we prefer the smallest containing cell.

inline int32_t findCameraCell(const Darkness::WRParsedData &wr,
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
inline uint8_t getCameraMediaType(const Darkness::WRParsedData &wr,
                                   float cx, float cy, float cz) {
    int32_t cell = findCameraCell(wr, cx, cy, cz);
    if (cell >= 0 && cell < static_cast<int32_t>(wr.numCells))
        return wr.cells[cell].mediaType;
    return 1; // default: air
}

// ── Camera collision infrastructure ──

// Camera sphere radius — matches Dark Engine PLAYER_RADIUS
static constexpr float CAMERA_SPHERE_RADIUS = 1.2f;
// Number of constraint-projection iterations for corner handling
static constexpr int   COLLISION_ITERATIONS = 3;

// Test if a point (assumed to lie on the polygon's plane) is inside a convex polygon.
// Uses the winding/cross-product method: the point must be on the interior side of
// every edge when traversed in order, relative to the polygon's plane normal.
inline bool pointInConvexPolygon(const Darkness::Vector3 &p,
                                 const std::vector<Darkness::Vector3> &verts,
                                 const std::vector<uint8_t> &indices,
                                 const Darkness::Vector3 &normal) {
    int n = static_cast<int>(indices.size());
    if (n < 3) return false;

    for (int i = 0; i < n; ++i) {
        const auto &a = verts[indices[i]];
        const auto &b = verts[indices[(i + 1) % n]];
        Darkness::Vector3 edge = b - a;
        Darkness::Vector3 toP  = p - a;
        Darkness::Vector3 cross = edge.crossProduct(toP);
        if (cross.dotProduct(normal) < 0.0f)
            return false;
    }
    return true;
}

// Apply sphere collision against solid polygons in the camera's current cell.
// Only checks the cell containing the camera center — within one convex cell,
// all plane normals face inward so pushes always go the correct direction.
// Uses polygon-level checks (not plane-level) to correctly handle planes that
// are partially portal and partially solid wall.
inline void applyCameraCollision(
    const Darkness::WRParsedData &wr,
    float oldPos[3], float newPos[3])
{
    // Find the cell containing the camera center
    int32_t cellIdx = findCameraCell(wr, newPos[0], newPos[1], newPos[2]);
    if (cellIdx < 0) {
        cellIdx = findCameraCell(wr, oldPos[0], oldPos[1], oldPos[2]);
        if (cellIdx < 0)
            return;  // Completely outside world
    }

    for (int iter = 0; iter < COLLISION_ITERATIONS; ++iter) {
        const auto &cell = wr.cells[cellIdx];
        int numSolid = cell.numPolygons - cell.numPortals;
        bool corrected = false;

        for (int pi = 0; pi < numSolid; ++pi) {
            const auto &poly = cell.polygons[pi];
            if (poly.count < 3) continue;

            const auto &plane = cell.planes[poly.plane];
            Darkness::Vector3 pos(newPos[0], newPos[1], newPos[2]);
            float dist = plane.getDistance(pos);

            // Cell planes face inward: positive = inside, negative = past the wall.
            // Push whenever dist < radius (sphere penetrates the plane).
            // Negative dist is fine — single-cell normals always push inward.
            if (dist >= CAMERA_SPHERE_RADIUS)
                continue;

            // Project camera onto the polygon's plane to check if the
            // penetration is through this specific polygon's surface area.
            // Use negated normal for the winding test: WR polygon vertices
            // are CCW when viewed from outside the cell (outward-facing for
            // rendering), but cell plane normals face inward. Negate so the
            // winding test matches the vertex order.
            Darkness::Vector3 projected = pos - plane.normal * dist;
            Darkness::Vector3 outNormal(
                -plane.normal.x, -plane.normal.y, -plane.normal.z);

            if (!pointInConvexPolygon(projected, cell.vertices,
                                      cell.polyIndices[pi], outNormal))
                continue;

            // Push sphere along inward normal until it clears the wall
            float push = CAMERA_SPHERE_RADIUS - dist;
            newPos[0] += plane.normal.x * push;
            newPos[1] += plane.normal.y * push;
            newPos[2] += plane.normal.z * push;
            corrected = true;
        }

        if (!corrected)
            break;

        // Push may have moved camera through a portal into an adjacent cell
        int32_t newCell = findCameraCell(wr, newPos[0], newPos[1], newPos[2]);
        if (newCell >= 0) {
            cellIdx = newCell;
        } else {
            // Pushed outside all cells — revert
            newPos[0] = oldPos[0];
            newPos[1] = oldPos[1];
            newPos[2] = oldPos[2];
            return;
        }
    }

    // Final validation
    if (findCameraCell(wr, newPos[0], newPos[1], newPos[2]) < 0) {
        newPos[0] = oldPos[0];
        newPos[1] = oldPos[1];
        newPos[2] = oldPos[2];
    }
}

// ── Portal culling infrastructure ──

// Portal connectivity info for BFS traversal
struct CellPortalInfo {
    uint32_t tgtCell;           // destination cell index
    Darkness::Plane plane;          // portal plane (for backface cull)
    // Portal polygon AABB (for frustum test)
    float minX, minY, minZ;
    float maxX, maxY, maxZ;
};

// Build portal adjacency graph from WR data.
// cellPortals[cellID] = list of portals leading to other cells.
inline std::vector<std::vector<CellPortalInfo>>
buildPortalGraph(const Darkness::WRParsedData &wr) {
    std::vector<std::vector<CellPortalInfo>> cellPortals(wr.numCells);

    for (uint32_t ci = 0; ci < wr.numCells; ++ci) {
        const auto &cell = wr.cells[ci];
        int numSolid = cell.numPolygons - cell.numPortals;

        for (int pi = numSolid; pi < cell.numPolygons; ++pi) {
            const auto &poly = cell.polygons[pi];
            const auto &indices = cell.polyIndices[pi];

            if (poly.count < 3) continue;
            if (poly.tgtCell >= wr.numCells) continue;

            CellPortalInfo portal;
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
inline std::unordered_set<uint32_t>
portalBFS(const Darkness::WRParsedData &wr,
          const std::vector<std::vector<CellPortalInfo>> &cellPortals,
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
inline void buildModelMatrix(float *mtx, float x, float y, float z,
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

} // namespace Darkness
