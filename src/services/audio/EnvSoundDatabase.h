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

// EnvSoundDatabase — decodes the ENV_SOUND tag-database chunk found in dark.gam.
//
// ON-DISK LAYOUT (per Dark Engine convention, ENV_SOUND v1.1):
//
//   uint32 nLocalTagRequired
//   uint8[nLocalTagRequired]             // packed bitarray flagging "required" tags
//   TagDB root                           // recursive — see below
//
// Each TagDB node is serialized as:
//
//   int32  nData
//   {int32 schemaObjID, float weight}[nData]    // data record, 8 bytes each
//   int32  nBranch
//   TagBranch[nBranch]
//
// And each TagBranch is a branch key followed by a child TagDB:
//
//   uint32 keyType                       // a tag-type index (interpretation
//                                        //   lives in the speech-domain name
//                                        //   maps; here we just capture the raw
//                                        //   index since the env-sound chunk
//                                        //   does not embed its own name maps)
//   uint8[8]                             // a 64-bit union: either {int32 min, int32 max}
//                                        //   for integer keys, or 8 enum bytes
//                                        //   terminated by the 0xFF 'unused' sentinel.
//                                        //   We surface both interpretations.
//   TagDB                                // recursive child
//
// We decode the full tree into a flat list of leaf records, where each entry
// carries the chain of keys that lead from the root to that leaf plus all
// (schemaObjID, weight) pairs stored on the node. This is enough for callers
// to cross-check the chunk against the `.sch` schemas and to build object-set
// indices keyed by tag pattern. The original engine's "bitarray of required
// tags" header is also exposed verbatim.

#pragma once

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <vector>

namespace Darkness {

class EnvSoundDatabase {
public:
    // A single key segment in the path from root to a leaf. Stores both
    // possible interpretations of the 8-byte union so callers can pick
    // whichever the surrounding key-type dictates (we don't have the
    // speech-domain name maps in ENV_SOUND to disambiguate ourselves).
    struct KeySegment {
        uint32_t keyType = 0;       // tag-type index (engine domain-local)
        int32_t  keyMin  = 0;       // integer-range low (signed)
        int32_t  keyMax  = 0;       // integer-range high (signed)
        uint8_t  keyEnums[8] = {0}; // alternate enum-byte view; 0xFF terminates
    };

    // One (schemaObjID, weight) record stored at a leaf or interior node.
    struct DataRecord {
        int32_t schemaObjID = 0;    // engine ObjID of the schema selected by this key path
        float   weight      = 1.0f; // matching weight (always 1.0 in stock Thief 2)
    };

    // A decoded entry: the chain of keys from root to this node, plus all
    // (schemaObjID, weight) data records stored at the node. Interior nodes with no data
    // are skipped — only nodes carrying at least one data record become
    // entries (mirrors how the engine queries the tree).
    struct EnvSoundEntry {
        std::vector<KeySegment> keyPath;
        std::vector<DataRecord> data;
    };

    EnvSoundDatabase() = default;

    // Decode `data` (raw ENV_SOUND chunk body). Returns false on
    // nullptr / size==0; on a structurally bad buffer the parser stops
    // cleanly, returns true if it managed to decode at least one entry,
    // and stashes any unparsed trailing bytes in `tailBytes()`.
    bool loadFromChunk(const uint8_t *data, size_t size);

    bool   isLoaded() const { return mLoaded; }
    size_t rawSize() const { return mBytes.size(); }
    const std::vector<uint8_t> &rawBytes() const { return mBytes; }

    // First N bytes of the raw chunk — useful for fingerprinting / debug.
    std::vector<uint8_t> headerBytes(size_t n = 16) const;

    // The "local tag required" bitarray that follows the size prefix in
    // the chunk body. One bit per tag-type index.
    const std::vector<uint8_t> &localTagRequired() const {
        return mLocalTagRequired;
    }

    // Decoded leaf/interior nodes (only those carrying data).
    const std::vector<EnvSoundEntry> &entries() const { return mEntries; }

    // Trailing bytes the decoder didn't consume (zero-length on a clean
    // decode; non-empty when the parser bailed early on a malformed
    // buffer).
    const std::vector<uint8_t> &tailBytes() const { return mTail; }

    // Print parse summary + first 16 header bytes to `out`.
    void dump(std::ostream &out) const;

private:
    std::vector<uint8_t>        mBytes;
    std::vector<uint8_t>        mLocalTagRequired;
    std::vector<EnvSoundEntry>  mEntries;
    std::vector<uint8_t>        mTail;
    bool                        mLoaded = false;
};

} // namespace Darkness
