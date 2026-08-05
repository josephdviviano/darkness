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

// CRF texture loader — loads PCX/GIF/TGA textures from CRF (ZIP) archives via zziplib

#pragma once

#include "PCXDecoder.h"
#include "GIFDecoder.h"
#include "TGADecoder.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <zzip/zzip.h>

namespace Darkness {

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

        // Try each path, keep the best (largest) result. txt16/ may have tiny
        // 2x2 placeholders while txt/ has the real texture, or vice versa.
        DecodedImage bestImg = {};
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
                DecodedImage img = {};
                if (buf.size() >= 6 && std::memcmp(buf.data(), "GIF", 3) == 0) {
                    img = decodeGIF(buf.data(), buf.size());
                } else if (buf.size() >= 128 + 769 && buf[0] == 0x0A) {
                    img = decodePCX(buf.data(), buf.size());
                } else {
                    std::fprintf(stderr, "CRF: unknown format for %s (%zu bytes, magic=%02x%02x%02x)\n",
                                 path.c_str(), buf.size(),
                                 buf.size() > 0 ? buf[0] : 0,
                                 buf.size() > 1 ? buf[1] : 0,
                                 buf.size() > 2 ? buf[2] : 0);
                    continue;
                }
                // Keep the larger version (txt/ may have real texture when
                // txt16/ only has a 2x2 placeholder, or vice versa)
                if (img.width * img.height > bestImg.width * bestImg.height) {
                    bestImg = std::move(img);
                }
            } catch (const std::exception &e) {
                std::fprintf(stderr, "CRF: Failed to decode %s: %s\n",
                             path.c_str(), e.what());
            }
        }

        // Return the best decoded image, or fallback if nothing decoded
        if (bestImg.width > 0 && bestImg.height > 0) return bestImg;
        return makeFallback();
    }

    // Load an effect sprite by bare name from `bitmap.crf` — coronas, rain,
    // snow, leaves. Unlike object materials, the names that reach here come
    // from a property field (P$Corona's 16-byte `texture`) and carry NO
    // extension, so we append candidates rather than joining the name
    // directly.
    //
    // Search order mirrors the archive's own shape: the root holds the
    // single-frame sprite (`CORONA.TGA`) and `txt/` holds the animated-family
    // variants (`txt/CORONA00.TGA`). Root wins, because a corona that is not
    // animated should not pick up frame 0 of something that is.
    //
    // Returns `found == false` rather than the magenta checkerboard when the
    // name resolves to nothing: a checkerboard billboard drawn additively over
    // the scene would be a far louder failure than the missing glow, and the
    // caller needs to be able to say which happened. Mirrors the editor's own
    // "can't find corona texture" diagnostic.
    struct BitmapResult {
        DecodedImage image;
        bool found = false;
        std::string path; // archive path that resolved, for logging
    };

    BitmapResult loadBitmap(const std::string &name) {
        BitmapResult result;
        if (!mDir || name.empty()) return result;

        static const char *kExts[] = { ".tga", ".pcx", ".gif" };
        std::vector<std::string> paths;
        paths.reserve(12);
        for (const char *ext : kExts) paths.push_back(name + ext);
        for (const char *ext : kExts) paths.push_back("txt/" + name + ext);
        // Animated families store frame 0 with a "00" suffix; a name that
        // refers to the family as a whole resolves to its first frame.
        for (const char *ext : kExts) paths.push_back("txt/" + name + "00" + ext);

        for (const auto &path : paths) {
            std::vector<uint8_t> buf;
            if (!readArchiveFile(path, buf)) continue;
            try {
                result.image = decodeByContent(buf, path);
            } catch (const std::exception &e) {
                std::fprintf(stderr, "CRF: Failed to decode %s: %s\n",
                             path.c_str(), e.what());
                continue;
            }
            if (result.image.width == 0 || result.image.height == 0) continue;
            result.found = true;
            result.path  = path;
            return result;
        }
        return result;
    }

    bool isOpen() const { return mDir != nullptr; }

private:
    ZZIP_DIR *mDir;

    // Read a whole archive member into `buf`. Returns false if absent.
    bool readArchiveFile(const std::string &path, std::vector<uint8_t> &buf) {
        ZZIP_FILE *fp = zzip_file_open(mDir, path.c_str(), ZZIP_CASELESS);
        if (!fp) return false;
        buf.clear();
        buf.reserve(65536);
        uint8_t tmp[4096];
        zzip_ssize_t n;
        while ((n = zzip_file_read(fp, tmp, sizeof(tmp))) > 0) {
            buf.insert(buf.end(), tmp, tmp + n);
        }
        zzip_file_close(fp);
        return !buf.empty();
    }

    // Pick a decoder from the bytes, not the file name. The archive mixes
    // extensions freely (`convey00.GIF` sits among the .tga sprites), and a
    // wrong guess here decodes garbage rather than failing.
    static DecodedImage decodeByContent(const std::vector<uint8_t> &buf,
                                        const std::string &path) {
        if (buf.size() >= 6 && std::memcmp(buf.data(), "GIF", 3) == 0)
            return decodeGIF(buf.data(), buf.size());
        if (buf.size() >= 128 + 769 && buf[0] == 0x0A)
            return decodePCX(buf.data(), buf.size());
        if (looksLikeTGA(buf.data(), buf.size()))
            return decodeTGA(buf.data(), buf.size());
        throw std::runtime_error("unrecognised image format in " + path);
    }

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

} // namespace Darkness
