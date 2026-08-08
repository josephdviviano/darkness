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

#include "HalfFloat.h"
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

// RGBA8 atlas — the DIRECTION layer only (octahedral dir, directionality
// ratio, hemisphere openness). All three are bounded quantities in [0,1] whose
// smallest meaningful step is far above 1/255, so 8 bits is genuinely enough
// there. The radiance atlas is a different problem and a different type; see
// HdrAtlasTexture.
struct AtlasTexture {
    int size;                // edge length (power of 2)
    std::vector<uint8_t> rgba; // size * size * 4 bytes
};

// RGBA16F atlas — the LIGHTMAP itself.
//
// This was RGBA8 until 2026-08-08 and the format was the dominant source of
// visible banding, for a reason that only shows up when you measure where the
// data actually sits rather than what the container permits. Measured on
// MISS6's re-bake: the median lit texel is 14/255 and 91% of lit texels are
// below 32/255 — the whole level lives in the bottom 6% of the range, where
// one 8-bit count is a 7.1% relative step. 82% of lit texels sit between
// 13/255 and 20/255, which is EIGHT distinct codes for four fifths of the
// level. At the other end, 0.985% of texels were pinned flat at 255 by a
// clamp, destroying the bright regions that should read as brightest and
// starving the bloom chain of anything above 1.0.
//
// Half-float fixes both ends at once because its precision is RELATIVE: that
// same 13..20/255 band holds 639 distinct values instead of 8, the step at the
// median texel is 0.04% instead of 7.1%, and the ceiling moves from 1.0 to
// 65504 so the clamp could be deleted outright.
//
// Half rather than float32 because the atlas is 64 MB at 4096² and this
// doubles it; RGBA rather than RGB because bgfx has no three-channel half
// format (R16F/RG16F/RGBA16F only), so the fourth channel is paid for whether
// used or not. It is currently written as 1.0 and read by nothing —
// deliberately reserved rather than speculatively defined; the leading
// candidate is a per-texel baked shadow term for the S4 demotion ladder, and
// the multi-light case it would otherwise have to solve is already handled
// exactly by the per-light overlay decomposition.
//
// Modern practice agrees on HDR lightmaps: Godot bakes to RGBAH (half) and
// reserves RGBA8 for its shadowMASK; Unity's high-quality mode is HDR (BC6H),
// with 8-bit RGBM only as a fallback. Both attributions in NOTES.SOURCE.md.
struct HdrAtlasTexture {
    int size = 0;                 // edge length (power of 2)
    std::vector<uint16_t> texels; // size * size * 4 halves
};

struct LightmapAtlasSet {
    std::vector<HdrAtlasTexture> atlases;
    // entries[cellIdx][polyIdx] for O(1) lookup during mesh building
    std::vector<std::vector<LmapEntry>> entries;
};

// ── Edge padding helper ──
// Fills a 2px border around a lightmap region in the atlas by edge-clamping.
// This prevents bilinear/bicubic filtering from sampling neighboring lightmaps.
// dataX/dataY = top-left of the actual data; lx/ly = data dimensions.
// The padding area is dataX-2..dataX+lx+1, dataY-2..dataY+ly+1.
// Templated on the texel component so the RGBA8 direction atlas and the
// RGBA16F lightmap atlas share one implementation — the padding is a pure
// copy and never looks at what the components mean.
template <typename Texel>
inline void fillEdgePadding(std::vector<Texel> &rgba, int atlasSize,
                            int dataX, int dataY, int lx, int ly, int pad = 2) {
    constexpr size_t kTexelBytes = 4 * sizeof(Texel);
    // Helper: get a pointer to the RGBA pixel at (px, py) in the atlas
    auto pixel = [&](int px, int py) -> Texel * {
        return &rgba[(static_cast<size_t>(py) * atlasSize + px) * 4];
    };

    // Fill left/right padding columns (including corners)
    for (int dy = -pad; dy < ly + pad; ++dy) {
        int srcRow = std::max(0, std::min(ly - 1, dy));
        // Left 2 columns: repeat leftmost data pixel in this row
        const Texel *srcLeft = pixel(dataX, dataY + srcRow);
        for (int dx = -pad; dx < 0; ++dx)
            std::memcpy(pixel(dataX + dx, dataY + dy), srcLeft, kTexelBytes);
        // Right 2 columns: repeat rightmost data pixel in this row
        const Texel *srcRight = pixel(dataX + lx - 1, dataY + srcRow);
        for (int dx = lx; dx < lx + pad; ++dx)
            std::memcpy(pixel(dataX + dx, dataY + dy), srcRight, kTexelBytes);
    }

    // Fill top/bottom padding rows (between the left/right columns already filled)
    for (int dx = 0; dx < lx; ++dx) {
        // Top 2 rows: repeat topmost data pixel in this column
        const Texel *srcTop = pixel(dataX + dx, dataY);
        for (int dy = -pad; dy < 0; ++dy)
            std::memcpy(pixel(dataX + dx, dataY + dy), srcTop, kTexelBytes);
        // Bottom 2 rows: repeat bottommost data pixel in this column
        const Texel *srcBot = pixel(dataX + dx, dataY + ly - 1);
        for (int dy = ly; dy < ly + pad; ++dy)
            std::memcpy(pixel(dataX + dx, dataY + dy), srcBot, kTexelBytes);
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
        HdrAtlasTexture atlas;
        atlas.size = atlasSize;
        atlas.texels.assign(static_cast<size_t>(atlasSize) * atlasSize * 4, 0);
        const uint16_t kHalfOne = floatToHalf(1.0f);

        // Fill the white fallback block
        for (int dy = 0; dy < 2; ++dy) {
            for (int dx = 0; dx < 2; ++dx) {
                size_t px = static_cast<size_t>(whiteBlock->y + dy) * atlasSize
                          + (whiteBlock->x + dx);
                atlas.texels[px * 4 + 0] = kHalfOne;
                atlas.texels[px * 4 + 1] = kHalfOne;
                atlas.texels[px * 4 + 2] = kHalfOne;
                atlas.texels[px * 4 + 3] = kHalfOne;
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

                    // The shipped bytes ARE the vintage truth. Widening n/255
                    // to half is not bit-exact (those are not dyadic
                    // rationals) but it is far below the source's own quantum:
                    // measured max error 2.4e-4, i.e. 0.062 of ONE 8-bit
                    // count, and every value round-trips to the same 8-bit
                    // code. Invariant 2 — vintage is the shipped atlas,
                    // untouched — survives; the container widened, the numbers
                    // did not move.
                    atlas.texels[dstPx * 4 + 0] = floatToHalf(r / 255.0f);
                    atlas.texels[dstPx * 4 + 1] = floatToHalf(g / 255.0f);
                    atlas.texels[dstPx * 4 + 2] = floatToHalf(b / 255.0f);
                    atlas.texels[dstPx * 4 + 3] = kHalfOne;
                }
            }

            // Fill 2px edge-clamped padding around the data
            fillEdgePadding(atlas.texels, atlasSize, dataX, dataY, ref.lx, ref.ly);

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

// ── Density scaling ────────────────────────────────────────────────────────
//
// Returns the same packing at `density` times the linear resolution: every
// pixel position, every size, the atlas edge and the padding all multiply by d.
//
// The point of scaling EVERYTHING uniformly — padding included — is that the
// normalised UVs come out bit-identical:
//
//     atlasU  = (x*d) / (size*d) = x / size
//     atlasSU = (w*d) / (size*d) = w / size
//
// so a mesh built against the 1:1 atlas addresses a denser one correctly with
// no rebuild. That is what lets a higher-resolution re-bake be an in-place
// texture swap against the shipped atlas instead of a parallel vertex buffer.
//
// Packing validity is preserved because a uniform scale cannot make two
// non-overlapping rectangles overlap.
inline LightmapAtlasSet scaleAtlasPlacement(const LightmapAtlasSet &src,
                                            int density) {
    LightmapAtlasSet out;
    if (density < 1) density = 1;
    out.entries = src.entries;
    for (auto &cellEntries : out.entries) {
        for (auto &e : cellEntries) {
            e.pixelX *= density;
            e.pixelY *= density;
            e.pixelW *= density;
            e.pixelH *= density;
            // atlasU/atlasSU deliberately untouched — they are already correct
            // for any density, which is the whole point.
        }
    }
    for (const auto &a : src.atlases) {
        HdrAtlasTexture t;
        t.size = a.size * density;
        t.texels.assign(static_cast<size_t>(t.size) * t.size * 4, 0);
        out.atlases.push_back(std::move(t));
    }
    return out;
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
    HdrAtlasTexture &atlas,
    const WRParsedData &wr,
    uint32_t cellIdx, int polyIdx,
    const LmapEntry &entry,
    const std::unordered_map<int16_t, float> &intensities,
    bool debugTint = false)
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

    // Clamp to [0, 1]. This is the VINTAGE path — the shipped 5:5:5 lumels
    // accumulate clamped at 31 and the original's blend clamps too, so the
    // ceiling here is authentic and stays (invariant 2: vintage is the shipped
    // atlas, untouched). Only the RE-BAKE path dropped its clamp when the
    // atlas went half-float.
    for (auto &v : blended)
        v = std::max(0.0f, std::min(1.0f, v));

    // Debug tint: add magenta overlay to animated lightmap polygons.
    // Blends 40% magenta (R=1, G=0, B=1) over the lightmap so the polygon
    // is clearly visible while still showing the underlying lighting.
    if (debugTint) {
        for (int p = 0; p < pixelCount; ++p) {
            float r = blended[p * 3 + 0];
            float g = blended[p * 3 + 1];
            float b = blended[p * 3 + 2];
            blended[p * 3 + 0] = std::min(1.0f, r * 0.6f + 0.4f); // boost red
            blended[p * 3 + 1] = g * 0.4f;                         // suppress green
            blended[p * 3 + 2] = std::min(1.0f, b * 0.6f + 0.4f); // boost blue
        }
    }

    // Direct blit at 1:1 scale into atlas
    for (int py = 0; py < ly && (entry.pixelY + py) < atlas.size; ++py) {
        for (int px = 0; px < lx && (entry.pixelX + px) < atlas.size; ++px) {
            int srcOff = (py * lx + px) * 3;
            int dstPx = (entry.pixelY + py) * atlas.size + (entry.pixelX + px);

            atlas.texels[dstPx * 4 + 0] = floatToHalf(blended[srcOff + 0]);
            atlas.texels[dstPx * 4 + 1] = floatToHalf(blended[srcOff + 1]);
            atlas.texels[dstPx * 4 + 2] = floatToHalf(blended[srcOff + 2]);
            atlas.texels[dstPx * 4 + 3] = floatToHalf(1.0f);
        }
    }

    // Re-fill 2px edge padding so GPU filtering stays correct after blend
    fillEdgePadding(atlas.texels, atlas.size, entry.pixelX, entry.pixelY, lx, ly);
}

} // namespace Darkness
