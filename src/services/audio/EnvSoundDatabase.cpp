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
#include <cstring>
#include <iomanip>
#include <ostream>

namespace Darkness {

namespace {

// Lightweight bounds-checked little-endian reader over a raw byte buffer.
// On overflow we mark the reader "failed"; subsequent reads return zero
// and the surrounding parser can detect the failure and stop.
struct LeReader {
    const uint8_t *base = nullptr;
    size_t         size = 0;
    size_t         pos  = 0;
    bool           ok   = true;

    bool need(size_t n) {
        if (!ok || pos + n > size) { ok = false; return false; }
        return true;
    }
    int32_t i32() {
        if (!need(4)) return 0;
        int32_t v;
        std::memcpy(&v, base + pos, 4);
        pos += 4;
        return v;
    }
    uint32_t u32() {
        if (!need(4)) return 0;
        uint32_t v;
        std::memcpy(&v, base + pos, 4);
        pos += 4;
        return v;
    }
    float f32() {
        if (!need(4)) return 0.0f;
        float v;
        std::memcpy(&v, base + pos, 4);
        pos += 4;
        return v;
    }
    void read(void *dst, size_t n) {
        if (!need(n)) return;
        std::memcpy(dst, base + pos, n);
        pos += n;
    }
};

// Sanity bounds match the original engine's debug-build assertion that
// a node holds fewer than 10000 records. Anything outside this
// range almost certainly means we lost sync with the stream.
constexpr int32_t kMaxRecordsPerNode = 10000;
constexpr size_t  kMaxTraversalDepth = 64;
constexpr size_t  kMaxEntries        = 65536; // hard cap as a runaway guard

// Walk one TagDB node. Pushes any data records onto `out` along
// with a copy of the current key path. Recurses into all branches. The
// engine recursion depth is bounded by tag-type count, which is small.
// Returns false on parse failure (reader overflow or bad counts).
bool decodeNode(LeReader &r,
                std::vector<EnvSoundDatabase::KeySegment> &path,
                std::vector<EnvSoundDatabase::EnvSoundEntry> &out,
                size_t depth)
{
    if (depth > kMaxTraversalDepth) return false;
    if (out.size() > kMaxEntries)   return false;

    const int32_t nData = r.i32();
    if (!r.ok || nData < 0 || nData > kMaxRecordsPerNode) return false;

    // Read data records; record a single entry per node when non-empty,
    // mirroring how the engine stores its "object-set" leaves.
    if (nData > 0) {
        EnvSoundDatabase::EnvSoundEntry entry;
        entry.keyPath = path;
        entry.data.reserve(static_cast<size_t>(nData));
        for (int32_t i = 0; i < nData; ++i) {
            EnvSoundDatabase::DataRecord d;
            d.schemaObjID = r.i32();
            d.weight      = r.f32();
            if (!r.ok) return false;
            entry.data.push_back(d);
        }
        out.push_back(std::move(entry));
    }

    const int32_t nBranch = r.i32();
    if (!r.ok || nBranch < 0 || nBranch > kMaxRecordsPerNode) return false;

    for (int32_t b = 0; b < nBranch; ++b) {
        EnvSoundDatabase::KeySegment seg;
        seg.keyType = r.u32();

        // 8-byte union — slurp once, then reinterpret both ways. The
        // engine writes it as a raw 8-byte blob, so we decode both
        // forms and let downstream code pick the right one based on
        // the surrounding tag-type's "flags" entry (kTagFlagInt).
        uint8_t raw[8];
        r.read(raw, 8);
        if (!r.ok) return false;
        std::memcpy(&seg.keyMin, raw + 0, 4);
        std::memcpy(&seg.keyMax, raw + 4, 4);
        std::memcpy(seg.keyEnums, raw, 8);

        path.push_back(seg);
        if (!decodeNode(r, path, out, depth + 1)) return false;
        path.pop_back();
    }

    return true;
}

} // namespace

bool EnvSoundDatabase::loadFromChunk(const uint8_t *data, size_t size) {
    mBytes.clear();
    mLocalTagRequired.clear();
    mEntries.clear();
    mTail.clear();
    mLoaded = false;

    if (data == nullptr || size == 0) return false;

    // Capture the bytes verbatim regardless of decode success so callers
    // can still inspect / fingerprint malformed chunks.
    mBytes.assign(data, data + size);

    LeReader r{data, size, 0, true};

    // Header: int32 nLocalTagRequired, then that many bitarray bytes.
    const int32_t nReq = r.i32();
    if (!r.ok || nReq < 0 || static_cast<size_t>(nReq) > size) {
        return false;
    }
    if (nReq > 0) {
        mLocalTagRequired.resize(static_cast<size_t>(nReq));
        r.read(mLocalTagRequired.data(), static_cast<size_t>(nReq));
        if (!r.ok) {
            mLocalTagRequired.clear();
            return false;
        }
    }

    // Root TagDB node. Parse what we can; on failure preserve the
    // unparsed tail so callers can investigate.
    std::vector<KeySegment> path;
    const size_t parseStart = r.pos;
    const bool ok = decodeNode(r, path, mEntries, 0);

    // Anything we couldn't reach goes into tailBytes(). Even a "successful"
    // decode may leave trailing padding in some chunks; we capture both.
    if (r.pos < size) {
        mTail.assign(data + r.pos, data + size);
    }

    // Consider the chunk "loaded" if the header parsed AND we either
    // decoded the full tree, or we decoded at least one entry (partial
    // decode is still useful for cross-checking).
    mLoaded = (parseStart > 0) && (ok || !mEntries.empty());
    return mLoaded;
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
    out << mBytes.size() << " bytes";
    if (mLoaded) {
        out << "  entries=" << mEntries.size()
            << "  required-tag-bits=" << mLocalTagRequired.size()
            << "  tail=" << mTail.size();
    } else {
        out << "  (parse failed)";
    }
    out << '\n';

    // 16-byte hex preview. Save/restore stream state to avoid polluting
    // shared ostreams (e.g. std::cout) with lingering hex formatting.
    out << "  header[0..16]:";
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
