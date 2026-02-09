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

// Standalone TXLIST chunk parser — reads texture name/family table from .mis files

#pragma once

#include "File.h"
#include "FileGroup.h"
#include "FileCompat.h"

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace Darkness {

#pragma pack(push, 1)

struct DarkDBChunkTXLIST {
    uint32_t length;
    uint32_t txt_count;
    uint32_t fam_count;
};

struct DarkDBTXLIST_fam {
    char name[16];
};

struct DarkDBTXLIST_texture {
    uint8_t one;
    uint8_t fam;
    uint16_t zero;
    char name[16];
};

#pragma pack(pop)

struct TXEntry {
    std::string family;
    std::string name;
    std::string fullPath; // "family/name" for CRF lookup
};

struct TXList {
    std::vector<std::string> families;
    std::vector<TXEntry> textures;
};

inline TXList parseTXList(const std::string &misPath) {
    FilePtr fp(new StdFile(misPath, File::FILE_R));
    FileGroupPtr db(new DarkFileGroup(fp));

    if (!db->hasFile("TXLIST")) {
        throw std::runtime_error("No TXLIST chunk found in " + misPath);
    }

    FilePtr chunk = db->getFile("TXLIST");

    DarkDBChunkTXLIST hdr;
    chunk->read(&hdr, sizeof(hdr));

    TXList result;

    // Read family names (fam_count entries)
    result.families.resize(hdr.fam_count);
    for (uint32_t i = 0; i < hdr.fam_count; ++i) {
        DarkDBTXLIST_fam fam;
        chunk->read(&fam, sizeof(fam));
        fam.name[15] = '\0';
        result.families[i] = fam.name;
    }

    // Read texture entries (txt_count entries)
    result.textures.resize(hdr.txt_count);
    for (uint32_t i = 0; i < hdr.txt_count; ++i) {
        DarkDBTXLIST_texture tex;
        chunk->read(&tex, sizeof(tex));
        tex.name[15] = '\0';

        TXEntry &entry = result.textures[i];
        entry.name = tex.name;

        if (tex.fam > 0 && tex.fam <= hdr.fam_count) {
            entry.family = result.families[tex.fam - 1]; // fam is 1-based
            entry.fullPath = entry.family + "/" + entry.name;
        } else {
            entry.family = "";
            entry.fullPath = entry.name;
        }
    }

    return result;
}

} // namespace Darkness
