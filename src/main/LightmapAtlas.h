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
// atlas textures. Uses BSP-based rectangle packing (FreeSpaceInfo from original codebase).

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

namespace Darkness {

// ── BSP-based 2D rectangle allocator (from original codebase FreeSpaceInfo.h) ──

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

// ── Atlas data structures ──

struct LmapEntry {
    float atlasU, atlasV;    // offset in atlas (0-1)
    float atlasSU, atlasSV;  // size in atlas (0-1)
    int atlasIndex;          // which atlas (usually 0)
    // Pixel coordinates for runtime atlas updates (blending animated lightmaps)
    int pixelX, pixelY;      // position in atlas (data origin, inside padding border)
    int pixelW, pixelH;      // dimensions in atlas (= lx, ly)
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

// ── Edge padding helper ──
// Fills a 2px border around a lightmap region in the atlas by edge-clamping.
// This prevents bilinear/bicubic filtering from sampling neighboring lightmaps.
// dataX/dataY = top-left of the actual data; lx/ly = data dimensions.
// The padding area is dataX-2..dataX+lx+1, dataY-2..dataY+ly+1.
inline void fillEdgePadding(std::vector<uint8_t> &rgba, int atlasSize,
                            int dataX, int dataY, int lx, int ly) {
    // Helper: get a pointer to the RGBA pixel at (px, py) in the atlas
    auto pixel = [&](int px, int py) -> uint8_t * {
        return &rgba[(py * atlasSize + px) * 4];
    };

    // Fill left/right padding columns (including corners)
    for (int dy = -2; dy < ly + 2; ++dy) {
        int srcRow = std::max(0, std::min(ly - 1, dy));
        // Left 2 columns: repeat leftmost data pixel in this row
        const uint8_t *srcLeft = pixel(dataX, dataY + srcRow);
        for (int dx = -2; dx < 0; ++dx)
            std::memcpy(pixel(dataX + dx, dataY + dy), srcLeft, 4);
        // Right 2 columns: repeat rightmost data pixel in this row
        const uint8_t *srcRight = pixel(dataX + lx - 1, dataY + srcRow);
        for (int dx = lx; dx < lx + 2; ++dx)
            std::memcpy(pixel(dataX + dx, dataY + dy), srcRight, 4);
    }

    // Fill top/bottom padding rows (between the left/right columns already filled)
    for (int dx = 0; dx < lx; ++dx) {
        // Top 2 rows: repeat topmost data pixel in this column
        const uint8_t *srcTop = pixel(dataX + dx, dataY);
        for (int dy = -2; dy < 0; ++dy)
            std::memcpy(pixel(dataX + dx, dataY + dy), srcTop, 4);
        // Bottom 2 rows: repeat bottommost data pixel in this column
        const uint8_t *srcBot = pixel(dataX + dx, dataY + ly - 1);
        for (int dy = ly; dy < ly + 2; ++dy)
            std::memcpy(pixel(dataX + dx, dataY + dy), srcBot, 4);
    }
}

// ── Atlas builder ──
// Packs all per-polygon lightmaps into a single atlas texture at 1:1 resolution
// with 2px edge-clamped padding for GPU bilinear/bicubic filtering.

inline LightmapAtlasSet buildLightmapAtlases(const WRParsedData &wr) {
    LightmapAtlasSet result;
    result.entries.resize(wr.numCells);

    for (uint32_t ci = 0; ci < wr.numCells; ++ci)
        result.entries[ci].resize(wr.cells[ci].numTextured);

    // Collect all lightmaps with valid dimensions
    struct LmRef {
        uint32_t cellIdx;
        int polyIdx;
        int lx, ly;         // source dimensions
        int area;            // for sorting
    };

    std::vector<LmRef> refs;
    for (uint32_t ci = 0; ci < wr.numCells; ++ci) {
        const auto &cell = wr.cells[ci];
        for (int pi = 0; pi < cell.numTextured; ++pi) {
            const auto &li = cell.lightInfos[pi];
            if (li.lx > 0 && li.ly > 0) {
                int origLx = static_cast<int>(li.lx);
                int origLy = static_cast<int>(li.ly);
                refs.push_back({ci, pi, origLx, origLy, origLx * origLy});
            }
        }
    }

    // Sort largest first for better packing
    std::sort(refs.begin(), refs.end(),
              [](const LmRef &a, const LmRef &b) { return a.area > b.area; });

    // Start with a small atlas and grow as needed
    int atlasSize = 64;
    const int maxAtlasSize = 4096;

    // Try packing — grow atlas if it fails.
    // Each lightmap allocates (lx+4, ly+4) to include 2px padding border.
    while (atlasSize <= maxAtlasSize) {
        auto packer = std::make_unique<FreeSpaceInfo>(0, 0, atlasSize, atlasSize);

        // Reserve pixel (0,0) as pure white for non-lightmapped polygon fallback.
        // Use a 2x2 block so bilinear filtering doesn't bleed into neighbours.
        FreeSpaceInfo *whiteBlock = packer->allocate(2, 2);
        if (!whiteBlock) {
            atlasSize *= 2;
            continue;
        }

        bool allFit = true;
        // Temporary storage for allocations before committing
        std::vector<FreeSpaceInfo *> allocations(refs.size(), nullptr);

        for (size_t i = 0; i < refs.size(); ++i) {
            // Allocate data + 4px (2px padding on each side)
            allocations[i] = packer->allocate(refs[i].lx + 4, refs[i].ly + 4);
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

        // Blit lightmap pixels and record entries.
        // Data is placed at (alloc + 2, alloc + 2) — the 2px border is filled
        // by edge-clamping after the blit.
        for (size_t i = 0; i < refs.size(); ++i) {
            const auto &ref = refs[i];
            const auto &alloc = allocations[i];
            const auto &cell = wr.cells[ref.cellIdx];
            const auto &lmData = cell.staticLightmaps[ref.polyIdx];

            // Data origin inside the padded allocation
            int dataX = alloc->x + 2;
            int dataY = alloc->y + 2;

            // Direct 1:1 blit
            for (int ly = 0; ly < ref.ly; ++ly) {
                for (int lx = 0; lx < ref.lx; ++lx) {
                    int srcOff = (ly * ref.lx + lx) * wr.lightSize;
                    int dstPx = (dataY + ly) * atlasSize + (dataX + lx);

                    uint8_t r, g, b;
                    convertLmPixel(&lmData[srcOff], wr.lightSize, r, g, b);

                    atlas.rgba[dstPx * 4 + 0] = r;
                    atlas.rgba[dstPx * 4 + 1] = g;
                    atlas.rgba[dstPx * 4 + 2] = b;
                    atlas.rgba[dstPx * 4 + 3] = 255;
                }
            }

            // Fill 2px edge-clamped padding around the data
            fillEdgePadding(atlas.rgba, atlasSize, dataX, dataY, ref.lx, ref.ly);

            // Record atlas UV entry — UVs point to inner data region (inside padding)
            LmapEntry &entry = result.entries[ref.cellIdx][ref.polyIdx];
            entry.atlasU = static_cast<float>(dataX) * invSize;
            entry.atlasV = static_cast<float>(dataY) * invSize;
            entry.atlasSU = static_cast<float>(ref.lx) * invSize;
            entry.atlasSV = static_cast<float>(ref.ly) * invSize;
            entry.atlasIndex = 0;
            // Pixel coordinates for runtime atlas updates (animated lightmaps)
            entry.pixelX = dataX;
            entry.pixelY = dataY;
            entry.pixelW = ref.lx;
            entry.pixelH = ref.ly;
        }

        result.atlases.push_back(std::move(atlas));

        std::fprintf(stderr, "Lightmap atlas: %dx%d, %zu lightmaps packed (1:1 + 2px padding)\n",
                     atlasSize, atlasSize, refs.size());
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
// Re-blends a single polygon's lightmap into the atlas CPU buffer at 1:1:
//   result = static + sum(intensity[i] * overlay[i])
// Overlays are in bit order of animflags, mapped to lightnum via cell.animMap.
// After blending, re-fills the 2px edge padding for correct GPU filtering.
//
// Parameters:
//   atlas       — atlas CPU buffer to write into
//   wr          — parsed WR data (for static/animated lightmap bytes, cell info)
//   cellIdx     — cell index
//   polyIdx     — textured polygon index within cell
//   entry       — atlas entry with pixel position/size
//   intensities — lightnum → current intensity (0.0-1.0) for each animated light
inline void blendAnimatedLightmap(
    AtlasTexture &atlas,
    const WRParsedData &wr,
    uint32_t cellIdx, int polyIdx,
    const LmapEntry &entry,
    const std::unordered_map<int16_t, float> &intensities)
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

    // Direct blit at 1:1 scale into atlas
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

    // Re-fill 2px edge padding so GPU filtering stays correct after blend
    fillEdgePadding(atlas.rgba, atlas.size, entry.pixelX, entry.pixelY, lx, ly);
}

} // namespace Darkness
