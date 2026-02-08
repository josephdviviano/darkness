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

// CRF model loader — loads .bin model files from obj.crf (ZIP) archives
// via zziplib. Same pattern as CRFTextureLoader.h.

#pragma once

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>
#include <zzip/zzip.h>

namespace Opde {

class CRFModelLoader {
public:
    CRFModelLoader(const std::string &resPath) : mDir(nullptr) {
        // Try obj.crf in the res directory
        std::string crfPath = resPath + "/obj.crf";
        zzip_error_t err = ZZIP_NO_ERROR;
        mDir = zzip_dir_open(crfPath.c_str(), &err);
        if (!mDir) {
            std::fprintf(stderr, "CRFModel: Failed to open %s (error %d)\n",
                         crfPath.c_str(), err);
        } else {
            std::fprintf(stderr, "CRFModel: Opened %s\n", crfPath.c_str());
        }
    }

    ~CRFModelLoader() {
        if (mDir) {
            zzip_dir_close(mDir);
        }
    }

    CRFModelLoader(const CRFModelLoader &) = delete;
    CRFModelLoader &operator=(const CRFModelLoader &) = delete;

    // Load a .bin model file by name (e.g. "sword" -> reads "sword.bin").
    // Returns the raw file contents, or empty vector if not found.
    std::vector<uint8_t> loadModel(const std::string &name) {
        if (!mDir) return {};

        // Try direct name + ".bin" extension
        std::string path = name + ".bin";

        ZZIP_FILE *fp = zzip_file_open(mDir, path.c_str(), ZZIP_CASELESS);
        if (!fp) {
            // Some models may be in subdirectories — try without extension
            // in case the name already includes it
            fp = zzip_file_open(mDir, name.c_str(), ZZIP_CASELESS);
            if (!fp) return {};
        }

        // Read entire file into buffer
        std::vector<uint8_t> buf;
        buf.reserve(32768); // typical .bin files are 5-50KB
        uint8_t tmp[4096];
        zzip_ssize_t n;
        while ((n = zzip_file_read(fp, tmp, sizeof(tmp))) > 0) {
            buf.insert(buf.end(), tmp, tmp + n);
        }
        zzip_file_close(fp);

        return buf;
    }

    bool isOpen() const { return mDir != nullptr; }

private:
    ZZIP_DIR *mDir;
};

} // namespace Opde
