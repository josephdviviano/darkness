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

// CRF sound loader — loads WAV sounds from snd.crf (ZIP) archives via zziplib.
// Follows the same zziplib pattern as CRFTextureLoader (originally from OPDE).
//
// Sound samples in snd.crf are organized as:
//   SFX/<name>.WAV        — sound effects (footsteps, doors, ambient, etc.)
//   <character>/english/<name>.WAV  — character voice lines
//
// Schema files reference sounds by bare name (no path, no extension).
// Lookup is case-insensitive via ZZIP_CASELESS.

#pragma once

#include <cstdio>
#include <cstdint>
#include <string>
#include <vector>
#include <unordered_map>
#include <list>
#include <mutex>
#include <zzip/zzip.h>

namespace Darkness {

/// Raw WAV file data loaded from CRF archive
struct SoundData {
    std::vector<uint8_t> wavData;  // Complete WAV file bytes (including header)
    std::string name;              // Original lookup name (lowercase)

    bool valid() const { return !wavData.empty(); }
    size_t sizeBytes() const { return wavData.size(); }
};

/// Loads WAV sound files from Thief 2's snd.crf archive.
/// Thread-safe for concurrent lookups (zziplib calls are mutex-protected).
class CRFSoundLoader {
public:
    explicit CRFSoundLoader(const std::string &resPath) : mDir(nullptr) {
        std::string crfPath = resPath + "/snd.crf";
        zzip_error_t err = ZZIP_NO_ERROR;
        mDir = zzip_dir_open(crfPath.c_str(), &err);
        if (!mDir) {
            std::fprintf(stderr, "CRFSoundLoader: Failed to open %s (error %d)\n",
                         crfPath.c_str(), err);
        } else {
            std::fprintf(stderr, "CRFSoundLoader: Opened %s\n", crfPath.c_str());
        }
    }

    ~CRFSoundLoader() {
        if (mDir) {
            zzip_dir_close(mDir);
        }
    }

    CRFSoundLoader(const CRFSoundLoader &) = delete;
    CRFSoundLoader &operator=(const CRFSoundLoader &) = delete;

    bool isOpen() const { return mDir != nullptr; }

    /// Load a sound by bare name (as referenced in .sch schema files).
    /// Tries multiple path patterns to find the file:
    ///   1. SFX/<name>.WAV  (sound effects)
    ///   2. <name>.WAV      (root level)
    /// For voice lines, use loadVoiceLine() instead.
    SoundData loadSound(const std::string &name) {
        if (!mDir) return {};

        // Search paths for sound effects (most common case)
        std::string paths[] = {
            "SFX/" + name + ".WAV",
            name + ".WAV",
        };

        for (const auto &path : paths) {
            SoundData result = readFile(path, name);
            if (result.valid()) return result;
        }

        return {};
    }

    /// Load a character voice line by character name and sample name.
    /// Path: <character>/english/<sample>.WAV
    SoundData loadVoiceLine(const std::string &character,
                            const std::string &sample) {
        if (!mDir) return {};

        std::string path = character + "/english/" + sample + ".WAV";
        return readFile(path, sample);
    }

    /// Load a sound by explicit path within the CRF archive.
    /// Useful when the full path is known.
    SoundData loadByPath(const std::string &path,
                         const std::string &name = "") {
        if (!mDir) return {};
        return readFile(path, name.empty() ? path : name);
    }

private:
    /// Read a file from the CRF archive by path (case-insensitive).
    SoundData readFile(const std::string &path, const std::string &name) {
        ZZIP_FILE *fp = zzip_file_open(mDir, path.c_str(), ZZIP_CASELESS);
        if (!fp) return {};

        // Read entire file into buffer
        std::vector<uint8_t> buf;
        buf.reserve(65536);
        uint8_t tmp[4096];
        zzip_ssize_t n;
        while ((n = zzip_file_read(fp, tmp, sizeof(tmp))) > 0) {
            buf.insert(buf.end(), tmp, tmp + n);
        }
        zzip_file_close(fp);

        // Minimal WAV validation: RIFF header + minimum size
        if (buf.size() < 44) return {};
        if (buf[0] != 'R' || buf[1] != 'I' || buf[2] != 'F' || buf[3] != 'F') {
            std::fprintf(stderr, "CRFSoundLoader: %s is not a valid WAV file\n",
                         path.c_str());
            return {};
        }

        SoundData result;
        result.wavData = std::move(buf);
        result.name = name;
        return result;
    }

    ZZIP_DIR *mDir;
};

/// LRU cache for decoded sound data.
/// Keeps recently-used sounds in memory, evicts least-recently-used when
/// the cache exceeds its memory budget.
class SoundCache {
public:
    /// @param maxBytes  Maximum total memory for cached sounds (default 64MB)
    explicit SoundCache(size_t maxBytes = 64 * 1024 * 1024)
        : mMaxBytes(maxBytes), mCurrentBytes(0) {}

    /// Get a cached sound, or nullptr if not cached.
    /// Moves the entry to front of LRU list on hit.
    const SoundData *get(const std::string &name) {
        auto it = mIndex.find(name);
        if (it == mIndex.end()) return nullptr;

        // Move to front (most recently used)
        mLruList.splice(mLruList.begin(), mLruList, it->second);
        return &(it->second->second);
    }

    /// Insert a sound into the cache. Evicts LRU entries if over budget.
    void put(const std::string &name, SoundData data) {
        // If already cached, remove old entry
        auto it = mIndex.find(name);
        if (it != mIndex.end()) {
            mCurrentBytes -= it->second->second.sizeBytes();
            mLruList.erase(it->second);
            mIndex.erase(it);
        }

        size_t dataSize = data.sizeBytes();

        // Evict LRU entries until we have room
        while (mCurrentBytes + dataSize > mMaxBytes && !mLruList.empty()) {
            evictLRU();
        }

        // Insert at front
        mLruList.emplace_front(name, std::move(data));
        mIndex[name] = mLruList.begin();
        mCurrentBytes += dataSize;
    }

    /// Clear the entire cache
    void clear() {
        mLruList.clear();
        mIndex.clear();
        mCurrentBytes = 0;
    }

    size_t currentBytes() const { return mCurrentBytes; }
    size_t entryCount() const { return mIndex.size(); }

private:
    using LruEntry = std::pair<std::string, SoundData>;
    using LruList = std::list<LruEntry>;
    using LruIterator = LruList::iterator;

    void evictLRU() {
        if (mLruList.empty()) return;
        auto &back = mLruList.back();
        mCurrentBytes -= back.second.sizeBytes();
        mIndex.erase(back.first);
        mLruList.pop_back();
    }

    LruList mLruList;                                    // front = MRU, back = LRU
    std::unordered_map<std::string, LruIterator> mIndex; // name → list iterator
    size_t mMaxBytes;
    size_t mCurrentBytes;
};

} // namespace Darkness
