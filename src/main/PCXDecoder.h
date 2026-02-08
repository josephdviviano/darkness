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

// PCX image decoder — 8-bit palettized PCX to RGBA32 pixel data

#pragma once

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace Opde {

struct DecodedImage {
    uint32_t width, height;
    std::vector<uint8_t> rgba; // width * height * 4 bytes (RGBA)
};

#pragma pack(push, 1)
struct PCXHeader {
    uint8_t manufacturer;
    uint8_t version;
    uint8_t encoding;
    uint8_t bitsPerPixel;
    uint16_t xMin, yMin, xMax, yMax;
    uint16_t hDpi, vDpi;
    uint8_t colormap[48];
    uint8_t reserved;
    uint8_t numPlanes;
    uint16_t bytesPerLine;
    uint16_t paletteInfo;
    uint16_t hScreenSize, vScreenSize;
    uint8_t filler[54];
};
#pragma pack(pop)

inline DecodedImage decodePCX(const uint8_t *data, size_t size) {
    if (size < 128 + 769)
        throw std::runtime_error("PCX file too small");

    const PCXHeader *hdr = reinterpret_cast<const PCXHeader *>(data);

    if (hdr->manufacturer != 0x0A)
        throw std::runtime_error("Not a PCX file");
    if (hdr->encoding != 1)
        throw std::runtime_error("PCX: unsupported encoding");
    if (hdr->bitsPerPixel != 8 || hdr->numPlanes != 1)
        throw std::runtime_error("PCX: only 8-bit palettized supported");

    uint32_t width = hdr->xMax - hdr->xMin + 1;
    uint32_t height = hdr->yMax - hdr->yMin + 1;

    // Read 256-color palette from end of file
    if (data[size - 769] != 0x0C)
        throw std::runtime_error("PCX: palette marker not found");

    const uint8_t *palette = data + size - 768;

    // RLE decompress pixel data
    uint32_t totalPixels = width * height;
    std::vector<uint8_t> indices(totalPixels);

    const uint8_t *src = data + 128;
    const uint8_t *srcEnd = data + size - 769;
    uint32_t dstPos = 0;
    uint32_t bytesPerLine = hdr->bytesPerLine;

    for (uint32_t row = 0; row < height && src < srcEnd; ++row) {
        uint32_t col = 0;
        while (col < bytesPerLine && src < srcEnd) {
            uint8_t byte = *src++;
            uint8_t count, value;
            if ((byte & 0xC0) == 0xC0) {
                count = byte & 0x3F;
                if (src >= srcEnd) break;
                value = *src++;
            } else {
                count = 1;
                value = byte;
            }
            for (uint8_t j = 0; j < count && col < bytesPerLine; ++j, ++col) {
                if (col < width && dstPos < totalPixels) {
                    indices[dstPos++] = value;
                }
            }
        }
    }

    // Convert palette indices to RGBA
    DecodedImage img;
    img.width = width;
    img.height = height;
    img.rgba.resize(totalPixels * 4);

    for (uint32_t i = 0; i < totalPixels; ++i) {
        uint8_t idx = indices[i];
        img.rgba[i * 4 + 0] = palette[idx * 3 + 0];
        img.rgba[i * 4 + 1] = palette[idx * 3 + 1];
        img.rgba[i * 4 + 2] = palette[idx * 3 + 2];
        img.rgba[i * 4 + 3] = (idx == 0) ? 0 : 255; // index 0 = transparent
    }

    return img;
}

} // namespace Opde
