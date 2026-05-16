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

#include "EnvSoundDatabase.h"

#include <algorithm>
#include <iomanip>
#include <ostream>

namespace Darkness {

bool EnvSoundDatabase::loadFromChunk(const uint8_t *data, size_t size) {
    // Reject empty / null inputs. A real chunk is on the order of ~19KB
    // for Thief 2's dark.gam; the original engine's tag database carries
    // at least a header and one tree node, so any zero-byte payload is
    // structurally invalid.
    if (data == nullptr || size == 0) {
        mBytes.clear();
        return false;
    }

    // Store the bytes verbatim — follow-up work will decode the binary
    // tree of tag → object-set mappings in place from this buffer.
    mBytes.assign(data, data + size);
    return true;
}

std::vector<uint8_t> EnvSoundDatabase::headerBytes(size_t n) const {
    const size_t take = std::min(n, mBytes.size());
    return std::vector<uint8_t>(mBytes.begin(), mBytes.begin() + take);
}

void EnvSoundDatabase::dump(std::ostream &out) const {
    out << "ENV_SOUND: ";
    if (mBytes.empty()) {
        out << "(not loaded)\n";
        return;
    }

    out << mBytes.size() << " bytes\n";
    out << "  header[0..16]:";

    // Save / restore stream state so callers don't see lingering hex
    // formatting on shared ostreams (e.g. std::cout).
    const std::ios_base::fmtflags savedFlags = out.flags();
    const char savedFill = out.fill();

    const size_t headerLen = std::min<size_t>(16, mBytes.size());
    out << std::hex << std::setfill('0');
    for (size_t i = 0; i < headerLen; ++i) {
        out << ' ' << std::setw(2) << static_cast<unsigned>(mBytes[i]);
    }
    out << '\n';

    out.flags(savedFlags);
    out.fill(savedFill);
}

} // namespace Darkness
