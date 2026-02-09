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

// Lightmap atlas packer — collects per-polygon lightmaps into GPU-friendly
// atlas textures. Uses BSP-based rectangle packing (FreeSpaceInfo from OPDE).

#pragma once

#include "WRChunkParser.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <unordered_map>
#include <vector>

namespace Opde {

// ── BSP-based 2D rectangle allocator (from OPDE FreeSpaceInfo.h) ──

class FreeSpaceInfo {
    std::unique_ptr<FreeSpaceInfo> mChild[2];

    FreeSpaceInfo() : x(0), y(0), w(0), h(0) {}

public:
    int x, y, w, h;
    bool used = false;

    FreeSpaceInfo(int x, int y, int w, int h) : x(x), y(y), w(w), h(h) {}

    bool isLeaf() const { return !mChild[0]; }

    FreeSpaceInfo *allocate(int sw, int sh) {
        if (!isLeaf()) {
            FreeSpaceInfo *result = mChild[0]->allocate(sw, sh);
            if (result) return result;
            return mChild[1]->allocate(sw, sh);
        }

        if (used) return nullptr;
        if (sw > w || sh > h) return nullptr;
        if (w == sw && h == sh) { used = true; return this; }

        int dw = w - sw;
        int dh = h - sh;

        if (dw > dh) {
            mChild[0].reset(new FreeSpaceInfo(x,      y, sw,     h));
            mChild[1].reset(new FreeSpaceInfo(x + sw, y, w - sw, h));
        } else {
            mChild[0].reset(new FreeSpaceInfo(x, y,      w, sh));
            mChild[1].reset(new FreeSpaceInfo(x, y + sh, w, h - sh));
        }

        return mChild[0]->allocate(sw, sh);
    }
};

// ── Pixel conversion ──

inline void convertLmPixel(const uint8_t *src, int lightSize,
                           uint8_t &r, uint8_t &g, uint8_t &b) {
    if (lightSize == 1) {
        r = g = b = src[0]; // grayscale
    } else {
        uint16_t val = src[0] | (src[1] << 8); // little-endian xBGR 5:5:5
        r = (val & 0x1F) << 3;
        g = ((val >> 5) & 0x1F) << 3;
        b = ((val >> 10) & 0x1F) << 3;
    }
}

// ── Bicubic (Catmull-Rom) lightmap upscaling ──

// Catmull-Rom spline weight for sample offset i (-1, 0, 1, 2) at fractional t
inline float catmullRomWeight(float t, int i) {
    // Standard Catmull-Rom with tau = 0.5
    switch (i) {
        case -1: return 0.5f * (-t + 2.0f * t * t - t * t * t);
        case  0: return 0.5f * (2.0f - 5.0f * t * t + 3.0f * t * t * t);
        case  1: return 0.5f * (t + 4.0f * t * t - 3.0f * t * t * t);
        case  2: return 0.5f * (-t * t + t * t * t);
        default: return 0.0f;
    }
}

// Sample a float RGB image at fractional (fx, fy) with bicubic interpolation.
// Clamps at edges (no wrapping — individual lightmaps, not tiled textures).
inline void sampleBicubic(const std::vector<float> &img, int w, int h,
                          float fx, float fy, float &r, float &g, float &b) {
    int ix = static_cast<int>(std::floor(fx));
    int iy = static_cast<int>(std::floor(fy));
    float fracX = fx - static_cast<float>(ix);
    float fracY = fy - static_cast<float>(iy);

    r = g = b = 0.0f;

    for (int jj = -1; jj <= 2; ++jj) {
        float wy = catmullRomWeight(fracY, jj);
        int sy = std::max(0, std::min(h - 1, iy + jj));

        for (int ii = -1; ii <= 2; ++ii) {
            float wx = catmullRomWeight(fracX, ii);
            int sx = std::max(0, std::min(w - 1, ix + ii));

            int off = (sy * w + sx) * 3;
            float weight = wx * wy;
            r += img[off + 0] * weight;
            g += img[off + 1] * weight;
            b += img[off + 2] * weight;
        }
    }
}

// In-place separable Gaussian blur on a float RGB image.
// Radius is the half-width of the kernel; sigma = radius / 2.
// Clamps at edges.
inline void gaussianBlur(std::vector<float> &img, int w, int h, int radius) {
    if (radius < 1) return;

    float sigma = static_cast<float>(radius) / 2.0f;
    float invSigma2 = 1.0f / (2.0f * sigma * sigma);

    // Build 1D kernel
    int kernelSize = radius * 2 + 1;
    std::vector<float> kernel(kernelSize);
    float sum = 0.0f;
    for (int i = 0; i < kernelSize; ++i) {
        float d = static_cast<float>(i - radius);
        kernel[i] = std::exp(-d * d * invSigma2);
        sum += kernel[i];
    }
    for (int i = 0; i < kernelSize; ++i)
        kernel[i] /= sum;

    // Horizontal pass
    std::vector<float> tmp(w * h * 3);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            float r = 0, g = 0, b = 0;
            for (int k = -radius; k <= radius; ++k) {
                int sx = std::max(0, std::min(w - 1, x + k));
                int off = (y * w + sx) * 3;
                float wt = kernel[k + radius];
                r += img[off + 0] * wt;
                g += img[off + 1] * wt;
                b += img[off + 2] * wt;
            }
            int off = (y * w + x) * 3;
            tmp[off + 0] = r;
            tmp[off + 1] = g;
            tmp[off + 2] = b;
        }
    }

    // Vertical pass
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            float r = 0, g = 0, b = 0;
            for (int k = -radius; k <= radius; ++k) {
                int sy = std::max(0, std::min(h - 1, y + k));
                int off = (sy * w + x) * 3;
                float wt = kernel[k + radius];
                r += tmp[off + 0] * wt;
                g += tmp[off + 1] * wt;
                b += tmp[off + 2] * wt;
            }
            int off = (y * w + x) * 3;
            img[off + 0] = r;
            img[off + 1] = g;
            img[off + 2] = b;
        }
    }
}

// Upscale a raw lightmap by the given integer factor using bicubic interpolation
// followed by a Gaussian blur pass for aggressive smoothing.
// src: raw lightmap bytes (lx * ly pixels, lightSize bytes/pixel)
// Returns RGBA8 image of size (lx*scale) x (ly*scale).
inline std::vector<uint8_t> upscaleLightmap(
    const uint8_t *src, int lx, int ly, int lightSize, int scale)
{
    // Convert source to float RGB (0.0 - 1.0)
    std::vector<float> fimg(lx * ly * 3);
    for (int i = 0; i < lx * ly; ++i) {
        uint8_t r, g, b;
        convertLmPixel(&src[i * lightSize], lightSize, r, g, b);
        fimg[i * 3 + 0] = r / 255.0f;
        fimg[i * 3 + 1] = g / 255.0f;
        fimg[i * 3 + 2] = b / 255.0f;
    }

    int outW = lx * scale;
    int outH = ly * scale;

    // Bicubic upscale into float buffer
    std::vector<float> upscaled(outW * outH * 3);
    for (int oy = 0; oy < outH; ++oy) {
        for (int ox = 0; ox < outW; ++ox) {
            float srcX = (static_cast<float>(ox) + 0.5f) / static_cast<float>(scale) - 0.5f;
            float srcY = (static_cast<float>(oy) + 0.5f) / static_cast<float>(scale) - 0.5f;

            float r, g, b;
            sampleBicubic(fimg, lx, ly, srcX, srcY, r, g, b);

            int off = (oy * outW + ox) * 3;
            upscaled[off + 0] = r;
            upscaled[off + 1] = g;
            upscaled[off + 2] = b;
        }
    }

    // Gaussian blur — radius proportional to scale for aggressive smoothing.
    // This spreads lighting beyond the original sample points.
    int blurRadius = scale * 2;
    gaussianBlur(upscaled, outW, outH, blurRadius);

    // Convert to RGBA8
    std::vector<uint8_t> out(outW * outH * 4);
    for (int i = 0; i < outW * outH; ++i) {
        out[i * 4 + 0] = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, upscaled[i * 3 + 0] * 255.0f)));
        out[i * 4 + 1] = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, upscaled[i * 3 + 1] * 255.0f)));
        out[i * 4 + 2] = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, upscaled[i * 3 + 2] * 255.0f)));
        out[i * 4 + 3] = 255;
    }

    return out;
}

// ── Atlas data structures ──

struct LmapEntry {
    float atlasU, atlasV;    // offset in atlas (0-1)
    float atlasSU, atlasSV;  // size in atlas (0-1)
    int atlasIndex;          // which atlas (usually 0)
    // Pixel coordinates for runtime atlas updates (blending animated lightmaps)
    int pixelX, pixelY;      // position in atlas
    int pixelW, pixelH;      // dimensions in atlas (= lx*scale, ly*scale)
};

struct AtlasTexture {
    int size;                // edge length (power of 2)
    std::vector<uint8_t> rgba; // size * size * 4 bytes
};

struct LightmapAtlasSet {
    std::vector<AtlasTexture> atlases;
    // entries[cellIdx][polyIdx] for O(1) lookup during mesh building
    std::vector<std::vector<LmapEntry>> entries;
};

// ── Atlas builder ──

inline LightmapAtlasSet buildLightmapAtlases(const WRParsedData &wr, int lmScale = 1) {
    LightmapAtlasSet result;
    result.entries.resize(wr.numCells);

    for (uint32_t ci = 0; ci < wr.numCells; ++ci)
        result.entries[ci].resize(wr.cells[ci].numTextured);

    // Collect all lightmaps with valid dimensions
    // Scaled dimensions are used for atlas allocation; original dimensions for source data
    struct LmRef {
        uint32_t cellIdx;
        int polyIdx;
        int lx, ly;         // original source dimensions
        int slx, sly;       // scaled dimensions (lx*lmScale, ly*lmScale)
        int area;            // scaled area for sorting
    };

    std::vector<LmRef> refs;
    for (uint32_t ci = 0; ci < wr.numCells; ++ci) {
        const auto &cell = wr.cells[ci];
        for (int pi = 0; pi < cell.numTextured; ++pi) {
            const auto &li = cell.lightInfos[pi];
            if (li.lx > 0 && li.ly > 0) {
                int origLx = static_cast<int>(li.lx);
                int origLy = static_cast<int>(li.ly);
                int scaledLx = origLx * lmScale;
                int scaledLy = origLy * lmScale;
                refs.push_back({ci, pi, origLx, origLy,
                                scaledLx, scaledLy,
                                scaledLx * scaledLy});
            }
        }
    }

    // Sort largest first for better packing
    std::sort(refs.begin(), refs.end(),
              [](const LmRef &a, const LmRef &b) { return a.area > b.area; });

    // Start with a small atlas and grow as needed.
    // Allow larger atlases when upscaling lightmaps.
    int atlasSize = 64;
    const int maxAtlasSize = (lmScale > 1) ? 8192 : 2048;

    // Try packing — grow atlas if it fails
    while (atlasSize <= maxAtlasSize) {
        auto packer = std::make_unique<FreeSpaceInfo>(0, 0, atlasSize, atlasSize);

        // Reserve pixel (0,0) as pure white for non-lightmapped polygon fallback
        // Use a 2x2 block so bilinear filtering doesn't bleed
        FreeSpaceInfo *whiteBlock = packer->allocate(2, 2);
        if (!whiteBlock) {
            atlasSize *= 2;
            continue;
        }

        bool allFit = true;
        // Temporary storage for allocations before committing
        std::vector<FreeSpaceInfo *> allocations(refs.size(), nullptr);

        for (size_t i = 0; i < refs.size(); ++i) {
            allocations[i] = packer->allocate(refs[i].slx, refs[i].sly);
            if (!allocations[i]) {
                allFit = false;
                break;
            }
        }

        if (!allFit) {
            atlasSize *= 2;
            continue;
        }

        // All fit — build the atlas texture
        AtlasTexture atlas;
        atlas.size = atlasSize;
        atlas.rgba.resize(atlasSize * atlasSize * 4, 0);

        // Fill the white fallback block
        for (int dy = 0; dy < 2; ++dy) {
            for (int dx = 0; dx < 2; ++dx) {
                int px = (whiteBlock->y + dy) * atlasSize + (whiteBlock->x + dx);
                atlas.rgba[px * 4 + 0] = 255;
                atlas.rgba[px * 4 + 1] = 255;
                atlas.rgba[px * 4 + 2] = 255;
                atlas.rgba[px * 4 + 3] = 255;
            }
        }

        float invSize = 1.0f / static_cast<float>(atlasSize);

        // Set up fallback entry pointing to center of white block
        LmapEntry fallbackEntry;
        fallbackEntry.atlasU = (whiteBlock->x + 0.5f) * invSize;
        fallbackEntry.atlasV = (whiteBlock->y + 0.5f) * invSize;
        fallbackEntry.atlasSU = 0.0f;
        fallbackEntry.atlasSV = 0.0f;
        fallbackEntry.atlasIndex = 0;
        fallbackEntry.pixelX = whiteBlock->x;
        fallbackEntry.pixelY = whiteBlock->y;
        fallbackEntry.pixelW = 0;
        fallbackEntry.pixelH = 0;

        // Initialize all entries to fallback
        for (uint32_t ci = 0; ci < wr.numCells; ++ci) {
            for (int pi = 0; pi < wr.cells[ci].numTextured; ++pi) {
                result.entries[ci][pi] = fallbackEntry;
            }
        }

        // Blit lightmap pixels and record entries
        for (size_t i = 0; i < refs.size(); ++i) {
            const auto &ref = refs[i];
            const auto &alloc = allocations[i];
            const auto &cell = wr.cells[ref.cellIdx];
            const auto &lmData = cell.staticLightmaps[ref.polyIdx];

            if (lmScale == 1) {
                // Direct blit — no upscaling overhead
                for (int ly = 0; ly < ref.ly; ++ly) {
                    for (int lx = 0; lx < ref.lx; ++lx) {
                        int srcOff = (ly * ref.lx + lx) * wr.lightSize;
                        int dstPx = (alloc->y + ly) * atlasSize + (alloc->x + lx);

                        uint8_t r, g, b;
                        convertLmPixel(&lmData[srcOff], wr.lightSize, r, g, b);

                        atlas.rgba[dstPx * 4 + 0] = r;
                        atlas.rgba[dstPx * 4 + 1] = g;
                        atlas.rgba[dstPx * 4 + 2] = b;
                        atlas.rgba[dstPx * 4 + 3] = 255;
                    }
                }
            } else {
                // Bicubic upscale, then blit the result
                auto upscaled = upscaleLightmap(lmData.data(), ref.lx, ref.ly,
                                                wr.lightSize, lmScale);
                for (int oy = 0; oy < ref.sly; ++oy) {
                    for (int ox = 0; ox < ref.slx; ++ox) {
                        int srcIdx = (oy * ref.slx + ox) * 4;
                        int dstPx = (alloc->y + oy) * atlasSize + (alloc->x + ox);

                        atlas.rgba[dstPx * 4 + 0] = upscaled[srcIdx + 0];
                        atlas.rgba[dstPx * 4 + 1] = upscaled[srcIdx + 1];
                        atlas.rgba[dstPx * 4 + 2] = upscaled[srcIdx + 2];
                        atlas.rgba[dstPx * 4 + 3] = 255;
                    }
                }
            }

            // Record atlas UV entry (using scaled dimensions in atlas space)
            LmapEntry &entry = result.entries[ref.cellIdx][ref.polyIdx];
            entry.atlasU = static_cast<float>(alloc->x) * invSize;
            entry.atlasV = static_cast<float>(alloc->y) * invSize;
            entry.atlasSU = static_cast<float>(ref.slx) * invSize;
            entry.atlasSV = static_cast<float>(ref.sly) * invSize;
            entry.atlasIndex = 0;
            // Pixel coordinates for runtime atlas updates (animated lightmaps)
            entry.pixelX = alloc->x;
            entry.pixelY = alloc->y;
            entry.pixelW = ref.slx;
            entry.pixelH = ref.sly;
        }

        result.atlases.push_back(std::move(atlas));

        std::fprintf(stderr, "Lightmap atlas: %dx%d, %zu lightmaps packed (scale %dx)\n",
                     atlasSize, atlasSize, refs.size(), lmScale);
        break;
    }

    if (result.atlases.empty()) {
        std::fprintf(stderr, "Warning: lightmap atlas exceeded max size %d, no lightmaps\n",
                     maxAtlasSize);
    }

    return result;
}

// ── Animated lightmap blending ──
//
// Re-blends a single polygon's lightmap into the atlas CPU buffer:
//   result = static + sum(intensity[i] * overlay[i])
// Overlays are in bit order of animflags, mapped to lightnum via cell.animMap.
// If lmScale > 1, upscales the blended result before writing to atlas.
//
// Parameters:
//   atlas       — atlas CPU buffer to write into
//   wr          — parsed WR data (for static/animated lightmap bytes, cell info)
//   cellIdx     — cell index
//   polyIdx     — textured polygon index within cell
//   entry       — atlas entry with pixel position/size
//   intensities — lightnum → current intensity (0.0-1.0) for each animated light
//   lmScale     — lightmap upscale factor (1 = no upscale)
inline void blendAnimatedLightmap(
    AtlasTexture &atlas,
    const WRParsedData &wr,
    uint32_t cellIdx, int polyIdx,
    const LmapEntry &entry,
    const std::unordered_map<int16_t, float> &intensities,
    int lmScale)
{
    const auto &cell = wr.cells[cellIdx];
    const auto &li = cell.lightInfos[polyIdx];
    int lx = static_cast<int>(li.lx);
    int ly = static_cast<int>(li.ly);
    if (lx <= 0 || ly <= 0) return;

    int pixelCount = lx * ly;
    const auto &staticLm = cell.staticLightmaps[polyIdx];
    const auto &overlays = cell.animLightmaps[polyIdx];

    // Blend in float space: result = static + sum(intensity[i] * overlay[i])
    std::vector<float> blended(pixelCount * 3);
    for (int p = 0; p < pixelCount; ++p) {
        uint8_t r, g, b;
        convertLmPixel(&staticLm[p * wr.lightSize], wr.lightSize, r, g, b);
        blended[p * 3 + 0] = r / 255.0f;
        blended[p * 3 + 1] = g / 255.0f;
        blended[p * 3 + 2] = b / 255.0f;
    }

    // Walk set bits in animflags to find which overlays to add.
    // Each set bit corresponds to an animMap index, which gives us the lightnum.
    int overlayIdx = 0;
    uint32_t flags = li.animflags;
    while (flags) {
        int bit = __builtin_ctz(flags); // lowest set bit
        flags &= flags - 1;            // clear lowest set bit

        if (overlayIdx >= static_cast<int>(overlays.size())) break;

        // Map bit position → animMap index → lightnum
        // The bit position in animflags corresponds to the animMap slot
        float intensity = 1.0f; // default: fully on
        if (bit < static_cast<int>(cell.animMap.size())) {
            int16_t lightNum = cell.animMap[bit];
            auto it = intensities.find(lightNum);
            if (it != intensities.end())
                intensity = it->second;
        }

        // Additive blend: result += intensity * overlay
        const auto &overlay = overlays[overlayIdx];
        for (int p = 0; p < pixelCount; ++p) {
            uint8_t r, g, b;
            convertLmPixel(&overlay[p * wr.lightSize], wr.lightSize, r, g, b);
            blended[p * 3 + 0] += intensity * (r / 255.0f);
            blended[p * 3 + 1] += intensity * (g / 255.0f);
            blended[p * 3 + 2] += intensity * (b / 255.0f);
        }

        ++overlayIdx;
    }

    // Clamp to [0, 1]
    for (auto &v : blended)
        v = std::max(0.0f, std::min(1.0f, v));

    // Write blended result into atlas (with optional upscaling)
    if (lmScale > 1) {
        // Upscale using existing bicubic + blur pipeline
        // First convert blended float → raw bytes for upscaleLightmap()
        // We use lightSize=1 (grayscale) format but store as 3 separate channels
        // Actually, build RGBA8 directly from float buffer, then upscale that
        // Simpler: convert to a temp raw buffer, upscale, blit

        // Build temporary RGBA8 at original resolution
        int outW = lx * lmScale;
        int outH = ly * lmScale;

        // Bicubic upscale from float buffer
        std::vector<float> upscaled(outW * outH * 3);
        for (int oy = 0; oy < outH; ++oy) {
            for (int ox = 0; ox < outW; ++ox) {
                float srcX = (static_cast<float>(ox) + 0.5f) / static_cast<float>(lmScale) - 0.5f;
                float srcY = (static_cast<float>(oy) + 0.5f) / static_cast<float>(lmScale) - 0.5f;

                float r, g, b;
                sampleBicubic(blended, lx, ly, srcX, srcY, r, g, b);

                int off = (oy * outW + ox) * 3;
                upscaled[off + 0] = r;
                upscaled[off + 1] = g;
                upscaled[off + 2] = b;
            }
        }

        // Gaussian blur for smooth lighting
        int blurRadius = lmScale * 2;
        gaussianBlur(upscaled, outW, outH, blurRadius);

        // Blit upscaled result into atlas
        for (int oy = 0; oy < outH && (entry.pixelY + oy) < atlas.size; ++oy) {
            for (int ox = 0; ox < outW && (entry.pixelX + ox) < atlas.size; ++ox) {
                int srcOff = (oy * outW + ox) * 3;
                int dstPx = (entry.pixelY + oy) * atlas.size + (entry.pixelX + ox);

                atlas.rgba[dstPx * 4 + 0] = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, upscaled[srcOff + 0] * 255.0f)));
                atlas.rgba[dstPx * 4 + 1] = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, upscaled[srcOff + 1] * 255.0f)));
                atlas.rgba[dstPx * 4 + 2] = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, upscaled[srcOff + 2] * 255.0f)));
                atlas.rgba[dstPx * 4 + 3] = 255;
            }
        }
    } else {
        // Direct blit at 1:1 scale
        for (int py = 0; py < ly && (entry.pixelY + py) < atlas.size; ++py) {
            for (int px = 0; px < lx && (entry.pixelX + px) < atlas.size; ++px) {
                int srcOff = (py * lx + px) * 3;
                int dstPx = (entry.pixelY + py) * atlas.size + (entry.pixelX + px);

                atlas.rgba[dstPx * 4 + 0] = static_cast<uint8_t>(blended[srcOff + 0] * 255.0f);
                atlas.rgba[dstPx * 4 + 1] = static_cast<uint8_t>(blended[srcOff + 1] * 255.0f);
                atlas.rgba[dstPx * 4 + 2] = static_cast<uint8_t>(blended[srcOff + 2] * 255.0f);
                atlas.rgba[dstPx * 4 + 3] = 255;
            }
        }
    }

}

} // namespace Opde
