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

// First-pass parser for the ENV_SOUND chunk found in dark.gam.
//
// ENV_SOUND is the original engine's compiled environmental-sound tag
// database — a binary tree of tag → object-set mappings. It is the
// compiled form of the same data the .sch parser derives from `env_tag`
// directives at runtime; once decoded, we can cross-check the two
// against each other.
//
// This implementation performs chunk-presence detection and raw byte
// storage only. Full tag-tree decoding is deferred to follow-up work.
// The captured bytes (and the 16-byte header preview) let callers
// verify chunk presence, size, and a fingerprint of its contents
// without committing to a layout interpretation yet.

#pragma once

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <vector>

namespace Darkness {

class EnvSoundDatabase {
public:
    EnvSoundDatabase() = default;

    /// Copy the chunk bytes into internal storage. Returns false on
    /// nullptr or size==0; otherwise true. Subsequent calls overwrite
    /// any previously-loaded bytes.
    bool loadFromChunk(const uint8_t *data, size_t size);

    /// True once a non-empty chunk has been loaded.
    bool isLoaded() const { return !mBytes.empty(); }

    /// Total chunk size in bytes (0 if not loaded).
    size_t rawSize() const { return mBytes.size(); }

    /// Read-only access to the captured raw bytes.
    const std::vector<uint8_t> &rawBytes() const { return mBytes; }

    /// Return up to the first `n` bytes of the chunk (clamped to size).
    /// Useful for fingerprinting / header inspection during the
    /// follow-up decode work.
    std::vector<uint8_t> headerBytes(size_t n = 16) const;

    /// Print size + first-16-byte hex dump to `out`. Intended for
    /// CLI inspection (e.g. darknessHeadless) and test debugging.
    void dump(std::ostream &out) const;

private:
    std::vector<uint8_t> mBytes;
};

} // namespace Darkness
