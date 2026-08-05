/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2025 darkness contributors
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

// TGA image decoder — Truevision TARGA to RGBA32 pixel data.
//
// The third image format in Thief 2's data, after PCX (world textures) and
// GIF (object textures). It is used for the effect sprites in `bitmap.crf`:
// coronas, rain, snow, leaves, the search-indicator. Those are exactly the
// images that need a real alpha channel, which is why they are TGA and not
// palettized: a palette can express "index 0 is a hole", but a corona is a
// continuous radial falloff from opaque centre to transparent rim, and the
// falloff *is* the effect.
//
// `CORONA.TGA` is 64x64, uncompressed, 32-bit BGRA, bottom-left origin.
// The other sprites in the archive cover the 24-bit and RLE cases, so all
// four combinations are handled rather than just the one we need today.

#pragma once

#include "PCXDecoder.h" // DecodedImage

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace Darkness {

#pragma pack(push, 1)
struct TGAHeader {
    uint8_t  idLength;
    uint8_t  colorMapType;  // 0 = none, 1 = present
    uint8_t  imageType;     // 1/2/3 uncompressed, 9/10/11 RLE
    uint16_t cmapFirst;
    uint16_t cmapLength;
    uint8_t  cmapEntrySize; // bits per palette entry
    uint16_t xOrigin, yOrigin;
    uint16_t width, height;
    uint8_t  bitsPerPixel;
    uint8_t  descriptor;    // bits 0-3 = alpha depth, bit 5 = top-left origin
};
#pragma pack(pop)
static_assert(sizeof(TGAHeader) == 18, "TGA header must be 18 bytes");

/// Cheap sniff for callers that pick a decoder by content. TGA has no magic
/// number — the format predates the convention — so this validates the
/// fields that a header must satisfy rather than matching a signature.
/// Deliberately strict: it is used to choose between decoders, and a false
/// positive here means feeding PCX bytes to this function.
inline bool looksLikeTGA(const uint8_t *data, size_t size) {
    if (size < sizeof(TGAHeader)) return false;
    TGAHeader h;
    std::memcpy(&h, data, sizeof(h));
    if (h.colorMapType > 1) return false;
    switch (h.imageType) {
    case 1: case 2: case 3: case 9: case 10: case 11: break;
    default: return false;
    }
    if (h.width == 0 || h.height == 0) return false;
    switch (h.bitsPerPixel) {
    case 8: case 15: case 16: case 24: case 32: break;
    default: return false;
    }
    // A colour-mapped image must carry a map; a true-colour one must not.
    const bool mapped = (h.imageType == 1 || h.imageType == 9);
    if (mapped != (h.colorMapType == 1)) return false;
    return true;
}

namespace detail {

/// Expand one source pixel to RGBA8. TGA channel order on disk is BGR(A).
inline void tgaPixelToRGBA(const uint8_t *src, uint8_t bpp,
                           const uint8_t *cmap, uint8_t cmapEntrySize,
                           uint16_t cmapFirst, uint16_t cmapLength,
                           uint8_t *dst) {
    switch (bpp) {
    case 8:
        if (cmap) {
            // Colour-mapped: index into the palette, which is itself stored
            // in one of the true-colour layouts. `cmapFirst` is the index the
            // stored map starts at — a sparse map omits the leading entries.
            uint32_t idx = src[0];
            idx = (idx >= cmapFirst) ? (idx - cmapFirst) : 0u;
            if (idx >= cmapLength) idx = 0;
            const uint8_t entryBytes = static_cast<uint8_t>(cmapEntrySize / 8);
            tgaPixelToRGBA(cmap + idx * entryBytes, cmapEntrySize,
                           nullptr, 0, 0, 0, dst);
        } else {
            // Greyscale.
            dst[0] = dst[1] = dst[2] = src[0];
            dst[3] = 255;
        }
        return;
    case 15:
    case 16: {
        // 5-5-5 with an optional attribute bit. Replicating the top bits into
        // the low ones spreads 0..31 across the full 0..255 range, so white
        // stays white instead of landing on 248.
        const uint16_t v = static_cast<uint16_t>(src[0] | (src[1] << 8));
        const uint8_t b5 = static_cast<uint8_t>(v & 0x1F);
        const uint8_t g5 = static_cast<uint8_t>((v >> 5) & 0x1F);
        const uint8_t r5 = static_cast<uint8_t>((v >> 10) & 0x1F);
        dst[0] = static_cast<uint8_t>((r5 << 3) | (r5 >> 2));
        dst[1] = static_cast<uint8_t>((g5 << 3) | (g5 >> 2));
        dst[2] = static_cast<uint8_t>((b5 << 3) | (b5 >> 2));
        // Bit 15 is the attribute bit. 16-bit TGAs in the wild leave it clear
        // even on fully opaque images, so treating it as alpha would make the
        // whole image vanish. Opaque is the only safe reading.
        dst[3] = 255;
        return;
    }
    case 24:
        dst[0] = src[2]; dst[1] = src[1]; dst[2] = src[0]; dst[3] = 255;
        return;
    case 32:
        dst[0] = src[2]; dst[1] = src[1]; dst[2] = src[0]; dst[3] = src[3];
        return;
    default:
        dst[0] = dst[1] = dst[2] = 0; dst[3] = 255;
        return;
    }
}

} // namespace detail

/// Decode a TGA image to top-left-origin RGBA32.
/// Throws std::runtime_error on a malformed or unsupported file.
inline DecodedImage decodeTGA(const uint8_t *data, size_t size) {
    if (size < sizeof(TGAHeader))
        throw std::runtime_error("TGA file too small");

    TGAHeader hdr;
    std::memcpy(&hdr, data, sizeof(hdr));

    const bool rle    = (hdr.imageType >= 9 && hdr.imageType <= 11);
    const bool mapped = (hdr.imageType == 1 || hdr.imageType == 9);

    switch (hdr.imageType) {
    case 1: case 2: case 3: case 9: case 10: case 11: break;
    default:
        throw std::runtime_error("TGA: unsupported image type");
    }

    const uint8_t bpp = hdr.bitsPerPixel;
    if (bpp != 8 && bpp != 15 && bpp != 16 && bpp != 24 && bpp != 32)
        throw std::runtime_error("TGA: unsupported bit depth");
    if (hdr.width == 0 || hdr.height == 0)
        throw std::runtime_error("TGA: zero-sized image");

    size_t pos = sizeof(TGAHeader) + hdr.idLength;
    if (pos > size) throw std::runtime_error("TGA: truncated ID field");

    // Colour map, if any.
    const uint8_t *cmap = nullptr;
    if (hdr.colorMapType == 1) {
        if (hdr.cmapEntrySize != 15 && hdr.cmapEntrySize != 16 &&
            hdr.cmapEntrySize != 24 && hdr.cmapEntrySize != 32)
            throw std::runtime_error("TGA: unsupported colour map entry size");
        const size_t cmapBytes =
            static_cast<size_t>(hdr.cmapLength) * (hdr.cmapEntrySize / 8);
        if (pos + cmapBytes > size)
            throw std::runtime_error("TGA: truncated colour map");
        cmap = data + pos;
        pos += cmapBytes;
    }
    if (mapped && !cmap)
        throw std::runtime_error("TGA: colour-mapped image without a map");

    const uint32_t w = hdr.width;
    const uint32_t h = hdr.height;
    const uint8_t srcBytes = static_cast<uint8_t>((bpp + 7) / 8);
    const size_t pixelCount = static_cast<size_t>(w) * h;

    DecodedImage img;
    img.width  = w;
    img.height = h;
    img.rgba.assign(pixelCount * 4, 0);

    // Decode into a linear scanline-order buffer first, then flip if needed.
    // Bit 5 of the descriptor selects the origin: clear = bottom-left (the
    // TGA default and what every sprite in bitmap.crf uses), set = top-left.
    // Everything downstream — bgfx included — wants top-left.
    std::vector<uint8_t> linear(pixelCount * 4, 0);

    if (!rle) {
        const size_t need = pixelCount * srcBytes;
        if (pos + need > size)
            throw std::runtime_error("TGA: truncated pixel data");
        for (size_t i = 0; i < pixelCount; ++i) {
            detail::tgaPixelToRGBA(data + pos + i * srcBytes, bpp, cmap,
                                   hdr.cmapEntrySize, hdr.cmapFirst,
                                   hdr.cmapLength, &linear[i * 4]);
        }
    } else {
        // RLE: a packet header byte, then either one repeated pixel (top bit
        // set) or a literal run. Runs may cross scanlines — the spec forbids
        // it but encoders do it anyway, so decode against the whole image
        // rather than row by row.
        size_t out = 0;
        while (out < pixelCount) {
            if (pos >= size)
                throw std::runtime_error("TGA: truncated RLE stream");
            const uint8_t packet = data[pos++];
            const size_t count = static_cast<size_t>(packet & 0x7F) + 1;
            if (out + count > pixelCount)
                throw std::runtime_error("TGA: RLE run overruns image");

            if (packet & 0x80) {
                if (pos + srcBytes > size)
                    throw std::runtime_error("TGA: truncated RLE run");
                uint8_t px[4];
                detail::tgaPixelToRGBA(data + pos, bpp, cmap,
                                       hdr.cmapEntrySize, hdr.cmapFirst,
                                       hdr.cmapLength, px);
                pos += srcBytes;
                for (size_t i = 0; i < count; ++i)
                    std::memcpy(&linear[(out + i) * 4], px, 4);
            } else {
                if (pos + count * srcBytes > size)
                    throw std::runtime_error("TGA: truncated RLE literal");
                for (size_t i = 0; i < count; ++i) {
                    detail::tgaPixelToRGBA(data + pos + i * srcBytes, bpp, cmap,
                                           hdr.cmapEntrySize, hdr.cmapFirst,
                                           hdr.cmapLength,
                                           &linear[(out + i) * 4]);
                }
                pos += count * srcBytes;
            }
            out += count;
        }
    }

    const bool topDown = (hdr.descriptor & 0x20) != 0;
    for (uint32_t y = 0; y < h; ++y) {
        const uint32_t srcRow = topDown ? y : (h - 1 - y);
        std::memcpy(&img.rgba[static_cast<size_t>(y) * w * 4],
                    &linear[static_cast<size_t>(srcRow) * w * 4],
                    static_cast<size_t>(w) * 4);
    }

    return img;
}

} // namespace Darkness
