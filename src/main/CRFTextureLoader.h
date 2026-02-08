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

// CRF texture loader — loads PCX/GIF textures from CRF (ZIP) archives via zziplib

#pragma once

#include "PCXDecoder.h"
#include "GIFDecoder.h"

#include <cstdio>
#include <string>
#include <vector>
#include <zzip/zzip.h>

namespace Opde {

class CRFTextureLoader {
public:
    // Open fam.crf in the res directory (for world textures)
    CRFTextureLoader(const std::string &resPath) : mDir(nullptr) {
        std::string crfPath = resPath + "/fam.crf";
        zzip_error_t err = ZZIP_NO_ERROR;
        mDir = zzip_dir_open(crfPath.c_str(), &err);
        if (!mDir) {
            std::fprintf(stderr, "CRF: Failed to open %s (error %d)\n",
                         crfPath.c_str(), err);
        } else {
            std::fprintf(stderr, "CRF: Opened %s\n", crfPath.c_str());
        }
    }

    // Open a specific CRF file (e.g. txt16.crf, txt.crf) for object textures
    CRFTextureLoader(const std::string &resPath, const std::string &crfName)
        : mDir(nullptr)
    {
        std::string crfPath = resPath + "/" + crfName;
        zzip_error_t err = ZZIP_NO_ERROR;
        mDir = zzip_dir_open(crfPath.c_str(), &err);
        if (!mDir) {
            std::fprintf(stderr, "CRF: Failed to open %s (error %d)\n",
                         crfPath.c_str(), err);
        } else {
            std::fprintf(stderr, "CRF: Opened %s\n", crfPath.c_str());
        }
    }

    ~CRFTextureLoader() {
        if (mDir) {
            zzip_dir_close(mDir);
        }
    }

    CRFTextureLoader(const CRFTextureLoader &) = delete;
    CRFTextureLoader &operator=(const CRFTextureLoader &) = delete;

    DecodedImage loadTexture(const std::string &family,
                             const std::string &name) {
        if (!mDir) return makeFallback();

        // Try "family/name.PCX" then "family/name.pcx"
        std::string paths[2];
        if (!family.empty()) {
            paths[0] = family + "/" + name + ".PCX";
            paths[1] = family + "/" + name + ".pcx";
        } else {
            paths[0] = name + ".PCX";
            paths[1] = name + ".pcx";
        }

        for (const auto &path : paths) {
            ZZIP_FILE *fp = zzip_file_open(mDir, path.c_str(), ZZIP_CASELESS);
            if (!fp) continue;

            // Read entire file into buffer
            std::vector<uint8_t> buf;
            buf.reserve(65536);
            uint8_t tmp[4096];
            zzip_ssize_t n;
            while ((n = zzip_file_read(fp, tmp, sizeof(tmp))) > 0) {
                buf.insert(buf.end(), tmp, tmp + n);
            }
            zzip_file_close(fp);

            if (buf.size() < 128 + 769) continue;

            try {
                return decodePCX(buf.data(), buf.size());
            } catch (const std::exception &e) {
                std::fprintf(stderr, "CRF: Failed to decode %s: %s\n",
                             path.c_str(), e.what());
            }
        }

        return makeFallback();
    }

    // Load an object texture by material name from within the CRF archive.
    // Material names in .bin files already include the file extension (e.g.
    // "HILT.GIF", "SWORDBL.GIF"). We search txt16/<name> first, then txt/<name>
    // (matching Dark Engine's lookup order from ManualBinFileLoader.cpp).
    DecodedImage loadObjectTexture(const std::string &matName) {
        if (!mDir) return makeFallback();

        // Search order: txt16/ preferred (16-bit), txt/ fallback (8-bit)
        // Material name already includes extension (e.g. "BARREL2.GIF")
        std::string paths[2] = {
            "txt16/" + matName,
            "txt/" + matName
        };

        for (const auto &path : paths) {
            ZZIP_FILE *fp = zzip_file_open(mDir, path.c_str(), ZZIP_CASELESS);
            if (!fp) continue;

            // Read entire file into buffer
            std::vector<uint8_t> buf;
            buf.reserve(65536);
            uint8_t tmp[4096];
            zzip_ssize_t n;
            while ((n = zzip_file_read(fp, tmp, sizeof(tmp))) > 0) {
                buf.insert(buf.end(), tmp, tmp + n);
            }
            zzip_file_close(fp);

            if (buf.size() < 13) continue; // too small for any image

            try {
                // Detect format by magic bytes
                if (buf.size() >= 6 && std::memcmp(buf.data(), "GIF", 3) == 0) {
                    return decodeGIF(buf.data(), buf.size());
                } else if (buf.size() >= 128 + 769 && buf[0] == 0x0A) {
                    return decodePCX(buf.data(), buf.size());
                }
            } catch (const std::exception &e) {
                std::fprintf(stderr, "CRF: Failed to decode %s: %s\n",
                             path.c_str(), e.what());
            }
        }

        return makeFallback();
    }

    bool isOpen() const { return mDir != nullptr; }

private:
    ZZIP_DIR *mDir;

    static DecodedImage makeFallback() {
        // 8x8 magenta/black checkerboard
        DecodedImage img;
        img.width = 8;
        img.height = 8;
        img.rgba.resize(8 * 8 * 4);
        for (uint32_t y = 0; y < 8; ++y) {
            for (uint32_t x = 0; x < 8; ++x) {
                uint32_t i = (y * 8 + x) * 4;
                bool dark = ((x + y) & 1) == 0;
                img.rgba[i + 0] = dark ? 0 : 255; // R
                img.rgba[i + 1] = 0;               // G
                img.rgba[i + 2] = dark ? 0 : 255;  // B
                img.rgba[i + 3] = 255;              // A
            }
        }
        return img;
    }
};

} // namespace Opde
