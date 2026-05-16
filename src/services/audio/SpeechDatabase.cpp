/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2024-2026 darkness contributors
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

#include "SpeechDatabase.h"

#include <algorithm>
#include <cstdio>
#include <ostream>

namespace Darkness {

bool SpeechDatabase::loadFromChunk(const uint8_t *data, size_t size) {
    // Defensive: empty / null inputs are not an error condition, but they
    // do not populate the database. Callers should check isLoaded().
    if (data == nullptr || size == 0) {
        clear();
        return false;
    }

    // Capture raw bytes verbatim. The full tag-DB decoder will operate on
    // this buffer in a later pass; for now we just hang onto it and
    // expose a header peek for diagnostics.
    mRaw.assign(data, data + size);
    mLoaded = true;

    // TODO(speech): once the on-disk format is understood, decode the
    // binary tree of tag -> object-set mappings here and populate typed
    // query structures. The PLAN.SOUND_DATA_PARSING.md Unit D notes
    // point at the relevant reference material.

    return true;
}

void SpeechDatabase::clear() {
    mRaw.clear();
    mRaw.shrink_to_fit();
    mLoaded = false;
}

std::vector<uint8_t> SpeechDatabase::headerBytes(size_t n) const {
    // Clamp request to actual buffer length to avoid out-of-range reads
    // when the chunk is unexpectedly short.
    const size_t count = std::min(n, mRaw.size());
    return std::vector<uint8_t>(mRaw.begin(), mRaw.begin() + count);
}

void SpeechDatabase::dump(std::ostream &out) const {
    if (!mLoaded) {
        out << "SpeechDatabase: <not loaded>\n";
        return;
    }

    out << "SpeechDatabase: loaded, " << mRaw.size() << " bytes\n";

    // Hex-dump the first 16 bytes (or fewer) as the header peek. We
    // print two groups of 8 for readability, similar to a classic
    // hexdump tool. snprintf used for portable, locale-free hex output.
    const std::vector<uint8_t> head = headerBytes(16);
    out << "  header[0.." << head.size() << "):";
    char buf[8];
    for (size_t i = 0; i < head.size(); ++i) {
        if (i == 8) out << " ";
        std::snprintf(buf, sizeof(buf), " %02x", head[i]);
        out << buf;
    }
    out << "\n";
}

} // namespace Darkness
