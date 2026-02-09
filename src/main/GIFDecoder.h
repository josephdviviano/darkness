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

// GIF image decoder — GIF87a/89a paletted images to RGBA32 pixel data.
// Supports single-frame paletted GIF images as used by Dark Engine object
// textures (txt16/*.gif and txt/*.gif inside obj.crf).

#pragma once

#include "PCXDecoder.h" // for DecodedImage

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace Darkness {

// Minimal GIF LZW decompressor for single-frame paletted images
inline DecodedImage decodeGIF(const uint8_t *data, size_t size) {
    if (size < 13)
        throw std::runtime_error("GIF file too small");

    // Validate signature: "GIF87a" or "GIF89a"
    if (std::memcmp(data, "GIF87a", 6) != 0 &&
        std::memcmp(data, "GIF89a", 6) != 0)
        throw std::runtime_error("Not a GIF file");

    // Logical screen descriptor
    uint16_t screenW = data[6] | (data[7] << 8);
    uint16_t screenH = data[8] | (data[9] << 8);
    uint8_t packed = data[10];
    // uint8_t bgColor = data[11]; // background color index
    // uint8_t aspect = data[12];

    bool hasGCT = (packed >> 7) & 1;
    uint32_t gctSize = hasGCT ? (1u << ((packed & 0x07) + 1)) : 0;

    // Read Global Color Table (3 bytes per entry: RGB)
    size_t pos = 13;
    std::vector<uint8_t> globalPalette;
    if (hasGCT) {
        uint32_t palBytes = gctSize * 3;
        if (pos + palBytes > size)
            throw std::runtime_error("GIF: truncated global color table");
        globalPalette.assign(data + pos, data + pos + palBytes);
        pos += palBytes;
    }

    // Track transparency from Graphic Control Extension
    bool hasTransparency = false;
    uint8_t transparentIndex = 0;

    // Skip extension blocks and find first image descriptor
    while (pos < size) {
        uint8_t marker = data[pos++];

        if (marker == 0x21) {
            // Extension block
            if (pos >= size) break;
            uint8_t extLabel = data[pos++];

            if (extLabel == 0xF9 && pos + 4 < size) {
                // Graphic Control Extension
                // uint8_t blockSize = data[pos]; // should be 4
                uint8_t gcPacked = data[pos + 1];
                // uint16_t delay = data[pos + 2] | (data[pos + 3] << 8);
                uint8_t transIdx = data[pos + 4];
                if (gcPacked & 0x01) {
                    hasTransparency = true;
                    transparentIndex = transIdx;
                }
            }

            // Skip all sub-blocks in this extension
            while (pos < size) {
                uint8_t blockLen = data[pos++];
                if (blockLen == 0) break; // block terminator
                pos += blockLen;
            }
        } else if (marker == 0x2C) {
            // Image descriptor found
            pos--; // put marker back
            break;
        } else if (marker == 0x3B) {
            // Trailer — end of GIF, no image found
            throw std::runtime_error("GIF: no image data found");
        }
        // Unknown marker — skip
    }

    if (pos >= size || data[pos] != 0x2C)
        throw std::runtime_error("GIF: no image descriptor found");

    // Image descriptor
    pos++; // skip 0x2C marker
    if (pos + 9 > size)
        throw std::runtime_error("GIF: truncated image descriptor");

    // uint16_t imgLeft = data[pos] | (data[pos+1] << 8);
    // uint16_t imgTop = data[pos+2] | (data[pos+3] << 8);
    uint16_t imgW = data[pos + 4] | (data[pos + 5] << 8);
    uint16_t imgH = data[pos + 6] | (data[pos + 7] << 8);
    uint8_t imgPacked = data[pos + 8];
    pos += 9;

    bool hasLCT = (imgPacked >> 7) & 1;
    bool interlaced = (imgPacked >> 6) & 1;
    uint32_t lctSize = hasLCT ? (1u << ((imgPacked & 0x07) + 1)) : 0;

    // Use local color table if present, otherwise global
    const uint8_t *palette;
    std::vector<uint8_t> localPalette;
    if (hasLCT) {
        uint32_t palBytes = lctSize * 3;
        if (pos + palBytes > size)
            throw std::runtime_error("GIF: truncated local color table");
        localPalette.assign(data + pos, data + pos + palBytes);
        pos += palBytes;
        palette = localPalette.data();
    } else if (!globalPalette.empty()) {
        palette = globalPalette.data();
    } else {
        throw std::runtime_error("GIF: no color table");
    }

    // LZW minimum code size
    if (pos >= size)
        throw std::runtime_error("GIF: truncated LZW data");
    uint8_t minCodeSize = data[pos++];
    if (minCodeSize < 2 || minCodeSize > 11)
        throw std::runtime_error("GIF: invalid LZW minimum code size");

    // Collect all sub-block data into a single buffer
    std::vector<uint8_t> lzwData;
    while (pos < size) {
        uint8_t blockLen = data[pos++];
        if (blockLen == 0) break;
        if (pos + blockLen > size) break;
        lzwData.insert(lzwData.end(), data + pos, data + pos + blockLen);
        pos += blockLen;
    }

    // LZW decompression
    uint32_t clearCode = 1u << minCodeSize;
    uint32_t eoiCode = clearCode + 1;
    uint32_t totalPixels = static_cast<uint32_t>(imgW) * static_cast<uint32_t>(imgH);

    // LZW table: each entry is (prefix, suffix) where prefix = -1 for roots
    struct LZWEntry {
        int32_t prefix;
        uint8_t suffix;
        uint16_t length;
    };

    const uint32_t MAX_TABLE = 4096;
    std::vector<LZWEntry> table(MAX_TABLE);
    std::vector<uint8_t> pixels;
    pixels.reserve(totalPixels);

    // Initialize table with root entries
    auto resetTable = [&](uint32_t &tableSize, uint32_t &codeSize, uint32_t &nextCode) {
        tableSize = eoiCode + 1;
        codeSize = minCodeSize + 1;
        nextCode = eoiCode + 1;
        for (uint32_t i = 0; i < clearCode; ++i) {
            table[i] = {-1, static_cast<uint8_t>(i), 1};
        }
    };

    // Bit reader for LZW stream
    uint32_t bitPos = 0;
    uint32_t totalBits = static_cast<uint32_t>(lzwData.size()) * 8;

    auto readBits = [&](uint32_t numBits) -> uint32_t {
        if (bitPos + numBits > totalBits)
            return eoiCode; // signal end
        uint32_t result = 0;
        for (uint32_t i = 0; i < numBits; ++i) {
            uint32_t byteIdx = (bitPos + i) / 8;
            uint32_t bitIdx = (bitPos + i) % 8;
            if (lzwData[byteIdx] & (1 << bitIdx))
                result |= (1 << i);
        }
        bitPos += numBits;
        return result;
    };

    // Decode a table entry into pixel values (in reverse order)
    auto decodeEntry = [&](uint32_t code) {
        // Stack-based decoding to avoid recursion
        std::vector<uint8_t> stack;
        uint32_t c = code;
        while (c != static_cast<uint32_t>(-1) && c < MAX_TABLE) {
            stack.push_back(table[c].suffix);
            c = static_cast<uint32_t>(table[c].prefix);
            if (stack.size() > totalPixels + 1) break; // safety
        }
        // Push in reverse (stack is built backwards)
        for (auto it = stack.rbegin(); it != stack.rend(); ++it) {
            if (pixels.size() < totalPixels)
                pixels.push_back(*it);
        }
        return stack.empty() ? uint8_t(0) : stack.back(); // first byte of entry
    };

    uint32_t tableSize, codeSize, nextCode;
    resetTable(tableSize, codeSize, nextCode);

    // Read first code (should be clear code)
    uint32_t code = readBits(codeSize);
    if (code == clearCode) {
        resetTable(tableSize, codeSize, nextCode);
        code = readBits(codeSize);
    }

    if (code == eoiCode || code >= tableSize) {
        // Degenerate image
        goto done;
    }

    {
        // Output first code
        decodeEntry(code);
        uint32_t prevCode = code;

        while (pixels.size() < totalPixels) {
            code = readBits(codeSize);

            if (code == eoiCode) break;

            if (code == clearCode) {
                resetTable(tableSize, codeSize, nextCode);
                code = readBits(codeSize);
                if (code == eoiCode) break;
                if (code >= tableSize) break;
                decodeEntry(code);
                prevCode = code;
                continue;
            }

            if (code < nextCode) {
                // Code is in table
                uint8_t firstByte = decodeEntry(code);

                // Add new entry: prevCode + firstByte of current code
                if (nextCode < MAX_TABLE) {
                    table[nextCode] = {static_cast<int32_t>(prevCode), firstByte,
                                       static_cast<uint16_t>(table[prevCode].length + 1)};
                    nextCode++;
                }
            } else if (code == nextCode) {
                // Special case: code not yet in table
                // New entry = prevCode entry + first byte of prevCode entry
                uint32_t c = prevCode;
                while (table[c].prefix != -1) c = static_cast<uint32_t>(table[c].prefix);
                uint8_t firstByte = table[c].suffix;

                if (nextCode < MAX_TABLE) {
                    table[nextCode] = {static_cast<int32_t>(prevCode), firstByte,
                                       static_cast<uint16_t>(table[prevCode].length + 1)};
                    nextCode++;
                }

                decodeEntry(code);
            } else {
                // Invalid code
                break;
            }

            prevCode = code;

            // Increase code size when table grows past current bit width
            if (nextCode >= (1u << codeSize) && codeSize < 12) {
                codeSize++;
            }
        }
    }

done:
    // Pad with zeros if we got fewer pixels than expected
    while (pixels.size() < totalPixels)
        pixels.push_back(0);

    // De-interlace if needed
    std::vector<uint8_t> deinterlaced;
    if (interlaced) {
        deinterlaced.resize(totalPixels);
        // GIF interlace passes: rows 0,8,16...; 4,12,20...; 2,6,10...; 1,3,5...
        const int startRow[4] = {0, 4, 2, 1};
        const int stepRow[4] = {8, 8, 4, 2};
        uint32_t srcRow = 0;
        for (int pass = 0; pass < 4; ++pass) {
            for (int row = startRow[pass]; row < imgH; row += stepRow[pass]) {
                if (srcRow < static_cast<uint32_t>(imgH)) {
                    std::memcpy(&deinterlaced[row * imgW],
                                &pixels[srcRow * imgW], imgW);
                    srcRow++;
                }
            }
        }
        pixels = std::move(deinterlaced);
    }

    // Convert palette indices to RGBA
    DecodedImage img;
    img.width = imgW;
    img.height = imgH;
    img.rgba.resize(totalPixels * 4);

    for (uint32_t i = 0; i < totalPixels; ++i) {
        uint8_t idx = pixels[i];
        img.rgba[i * 4 + 0] = palette[idx * 3 + 0]; // R
        img.rgba[i * 4 + 1] = palette[idx * 3 + 1]; // G
        img.rgba[i * 4 + 2] = palette[idx * 3 + 2]; // B
        // Dark Engine always treats palette index 0 as transparent, regardless
        // of GIF format transparency extensions.  Also honor the GIF89a
        // Graphic Control Extension transparent index if present.
        if (idx == 0 || (hasTransparency && idx == transparentIndex))
            img.rgba[i * 4 + 3] = 0;
        else
            img.rgba[i * 4 + 3] = 255;
    }

    return img;
}

} // namespace Darkness
