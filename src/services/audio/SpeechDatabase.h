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

// SpeechDatabase — first-pass parser for the Speech_DB chunk found in
// dark.gam (~117 KB). Speech_DB is the Dark Engine's serialized form of an
// opaque tag database (a binary tree of tag -> object-set mappings) used
// by AI speech-concept lookup at runtime.
//
// FIRST-PASS PARSER — chunk-presence detection only. Full binary-tree
// decoding is deferred to follow-up work (see Unit D in
// .claude/PLAN.SOUND_DATA_PARSING.md). For now we:
//
//   * capture and own the raw chunk bytes
//   * expose chunk size and a peek at the leading header bytes
//   * provide a `dump()` debug printer
//
// The interface is intentionally future-proof: when full decoding lands,
// callers using the methods below will not need to change. New typed
// query accessors will simply be added alongside the existing ones.
//
// Per-object speech properties (P$SpchVoice, P$VoiceIdx, P$MaxSpchPa,
// P$MinSpchPa) are NOT handled here — they live in the property service
// and are already wired up via DarkPropertyDefs.h.

#pragma once

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <vector>

namespace Darkness {

class SpeechDatabase {
public:
    SpeechDatabase() = default;
    ~SpeechDatabase() = default;

    // Non-copyable but movable — raw buffer can be large (~117 KB).
    SpeechDatabase(const SpeechDatabase &) = delete;
    SpeechDatabase &operator=(const SpeechDatabase &) = delete;
    SpeechDatabase(SpeechDatabase &&) = default;
    SpeechDatabase &operator=(SpeechDatabase &&) = default;

    // Load the raw Speech_DB chunk. Stores the bytes verbatim and parses
    // what we can decode at this stage (currently: nothing beyond size +
    // header peek). Returns false when `data == nullptr` or `size == 0`.
    //
    // Calling loadFromChunk a second time replaces any previous content.
    bool loadFromChunk(const uint8_t *data, size_t size);

    // Forget any previously loaded chunk.
    void clear();

    // True once a non-empty buffer has been loaded.
    bool isLoaded() const { return mLoaded; }

    // Number of bytes captured from the chunk.
    size_t rawSize() const { return mRaw.size(); }

    // Return the first N bytes of the chunk (or fewer, if the chunk is
    // shorter). Useful for logging/diagnostics and for future format
    // sniffing.
    std::vector<uint8_t> headerBytes(size_t n = 16) const;

    // Read-only view of the full captured buffer. Future decoders will
    // operate on this directly.
    const std::vector<uint8_t> &rawBytes() const { return mRaw; }

    // Pretty-print everything we currently know about the chunk to
    // `out`: size, loaded flag, and a hex dump of the leading header
    // bytes. Produces non-empty output even when nothing is loaded
    // (prints a "not loaded" marker).
    void dump(std::ostream &out) const;

private:
    std::vector<uint8_t> mRaw;
    bool mLoaded = false;
};

} // namespace Darkness
