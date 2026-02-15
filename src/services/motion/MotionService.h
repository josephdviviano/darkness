/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2005-2009 openDarkEngine team
 *    Copyright (C) 2026 darkness team
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

// MotionService.h — Motion capture database service
//
// Loads .mi (motion info) + .mc (motion capture) file pairs from motions.crf
// (ZIP archive via zziplib). Provides a read-only database of motion clips
// with per-frame root translation data for first-person head bob.
//
// Service architecture:
//   MotionService (this file)  — global database, stateless after load
//   MotionPlayback.h           — per-entity playback state (owned by consumer)
//
// The motion system touches sound (footfall flags), AI (locomotion planning),
// physics (position constraints), rendering (joint transforms), scripts (PlayMotion),
// and camera (butt offset). Many of these are still TODO.
//
// Binary format references documented in .claude/NOTES.SOURCE.md (gitignored).

#pragma once

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

#include <zzip/zzip.h>

#include "DarknessMath.h"

namespace Darkness {

// ── Motion flag bits ──
// Per-frame event markers embedded in .mi files. These match the original
// engine's flag definitions for cross-system compatibility.
// Sound, physics, and scripts will consume these flags.
//
// Sound, physics, and scripts consume these flags via MotionPlayback callbacks.
// Flags are loaded from .mi files in motions.crf during MotionService::loadFromCRF().
enum MotionFlags : uint32_t {
    MF_STANDING        = 0x00000001,  // character at rest
    MF_LEFT_FOOTFALL   = 0x00000002,  // left foot touches ground
    MF_RIGHT_FOOTFALL  = 0x00000004,  // right foot touches ground
    MF_LEFT_FOOTUP     = 0x00000008,  // left foot lifts
    MF_RIGHT_FOOTUP    = 0x00000010,  // right foot lifts
    MF_FIRE_RELEASE    = 0x00000020,  // weapon release point
    MF_CAN_INTERRUPT   = 0x00000040,  // motion can be cancelled
    MF_START_MOT       = 0x00000080,  // motion starting
    MF_END_MOT         = 0x00000100,  // motion ending
    // Triggers 1–8 for game-specific callbacks
    MF_TRIGGER1        = 0x00001000,
    MF_TRIGGER2        = 0x00002000,
    MF_TRIGGER3        = 0x00004000,
    MF_TRIGGER4        = 0x00008000,
    MF_TRIGGER5        = 0x00010000,
    MF_TRIGGER6        = 0x00020000,  // weapon charge sound
    MF_TRIGGER7        = 0x00040000,  // search sound
    MF_TRIGGER8        = 0x00080000,  // weapon swing sound
};

// ── Motion clip ──
// Single parsed motion — .mi header + .mc per-frame root translation.
// v1: root translation only. Joint rotations reserved for v2 (3rd-person/NPC).
struct MotionClip {
    std::string name;               // motion name from .mi (e.g. "humwlklt")
    int   numFrames = 0;            // frame count from .mi header
    int   freq      = 30;           // playback FPS from .mi (always 30 in Thief 2)
    float duration  = 0.0f;         // numFrames / freq (seconds)

    // Root translation — pelvis displacement per frame.
    // Coordinate convention (motion-local):
    //   X = forward displacement
    //   Y = lateral sway (negative = left)
    //   Z = vertical bob (positive = up)
    std::vector<Vector3> rootTranslation;  // [numFrames] entries

    // Derived motion metrics (computed after parsing)
    float totalForwardDisp = 0.0f;  // rootTranslation.back().x
    float zMin = 0.0f;             // minimum Z across all frames
    float zMax = 0.0f;             // maximum Z across all frames

    // Motion flags — event markers at specific frames.
    // Sorted by frame number. Used by sound (footfalls), physics (foot
    // contact up/down), and scripts (trigger callbacks).
    std::vector<std::pair<int, uint32_t>> flags;  // {frame, flagBits}
};

// ── Player motion IDs ──
// Named slots for player locomotion clips. Maps semantic actions to clips.
// Future: replace with tag-based schema queries from a full motion database.
enum class PlayerMotionID {
    WalkLeft,           // humwlklt — left foot walk stride
    WalkRight,          // humwlkrt — right foot walk stride
    RunLeft,            // humlrunl — left foot run stride
    RunRight,           // humlrunr — right foot run stride
    CrouchWalkLeft,     // humdwlkl — crouching left stride
    CrouchWalkRight,    // humdwlkr — crouching right stride
    Stand,              // humstand — idle stance (breathing)
    JumpForward,        // humjumpf — forward jump
    Count
};

// ── MI/MC binary format constants ──
// Component type field in the .mi component array
static constexpr int32_t CM_ROT   = 0;  // joint rotation (quaternion per frame)
static constexpr int32_t CM_TRANS = 1;  // root translation (vector3 per frame)

// MI header size (fixed portion before component + flag arrays)
static constexpr size_t MI_HEADER_SIZE = 96;

// Per-component record in .mi file (12 bytes each)
struct MIComponent {
    int32_t type;       // CM_ROT or CM_TRANS
    int32_t joint_id;   // which skeleton joint
    int32_t handle;     // index into MC offset table
};
static_assert(sizeof(MIComponent) == 12, "MIComponent must be 12 bytes");

// Per-flag record in .mi file (8 bytes each)
struct MIFlag {
    int32_t  frame;     // frame number where flag fires
    uint32_t flags;     // flag bits (MotionFlags)
};
static_assert(sizeof(MIFlag) == 8, "MIFlag must be 8 bytes");

// ── Motion name → PlayerMotionID mapping ──
// Hardcoded for the human player creature. Future: data-driven from MotGaitDe
// property on the Player archetype object.
struct PlayerMotionEntry {
    PlayerMotionID id;
    const char *miName;   // .mi filename (without extension)
    const char *mcName;   // .mc filename (without extension — includes trailing _)
};

static constexpr PlayerMotionEntry PLAYER_MOTION_TABLE[] = {
    { PlayerMotionID::WalkLeft,        "humwlklt",  "humwlklt_" },
    { PlayerMotionID::WalkRight,       "humwlkrt",  "humwlkrt_" },
    { PlayerMotionID::RunLeft,         "humlrunl",  "humlrunl_" },
    { PlayerMotionID::RunRight,        "humlrunr",  "humlrunr_" },
    { PlayerMotionID::CrouchWalkLeft,  "humdwlkl",  "humdwlkl_" },
    { PlayerMotionID::CrouchWalkRight, "humdwlkr",  "humdwlkr_" },
    { PlayerMotionID::Stand,           "humstand",  "humstand_" },
    { PlayerMotionID::JumpForward,     "humjumpf",  "humjumpf_" },
};

// ════════════════════════════════════════════════════════════════════
// MotionService — Motion capture database
// ════════════════════════════════════════════════════════════════════
//
// Loads player locomotion motions from motions.crf at startup, then
// provides read-only access to clip data. Stateless after load —
// safe to share across systems.
//
// Usage:
//   MotionService svc;
//   svc.loadFromCRF("/path/to/RES");
//   const MotionClip *clip = svc.getClip(PlayerMotionID::WalkLeft);
//   Vector3 offset = MotionService::sampleRootTranslation(*clip, 0.3f);

class MotionService {
public:
    // ── Load ──

    /// Load player locomotion motions from motions.crf in the given res path.
    /// motions.crf is present in ALL Dark Engine games — a missing file
    /// indicates a bad --res path. Logs errors for each clip that fails.
    /// Returns true if at least the walk clips loaded successfully.
    inline bool loadFromCRF(const std::string &resPath) {
        std::string crfPath = resPath + "/motions.crf";
        zzip_error_t err = ZZIP_NO_ERROR;
        ZZIP_DIR *dir = zzip_dir_open(crfPath.c_str(), &err);
        if (!dir) {
            std::fprintf(stderr, "MotionService: ERROR — failed to open %s (error %d)\n"
                                 "  motions.crf is required. Check --res path.\n",
                         crfPath.c_str(), err);
            return false;
        }

        int loaded = 0;
        int failed = 0;

        for (const auto &entry : PLAYER_MOTION_TABLE) {
            int idx = static_cast<int>(entry.id);
            MotionClip &clip = mClips[idx];

            if (parseMotionPair(dir, entry.miName, entry.mcName, clip)) {
                mNameIndex[clip.name] = idx;
                ++loaded;
                std::fprintf(stderr, "  Motion: %-12s  %3d frames @ %dHz  (%.3fs, fwd=%.2f, Z=[%.3f,%.3f], %zu flags)\n",
                             clip.name.c_str(), clip.numFrames, clip.freq,
                             clip.duration, clip.totalForwardDisp,
                             clip.zMin, clip.zMax, clip.flags.size());
            } else {
                ++failed;
                std::fprintf(stderr, "  Motion: %-12s  FAILED to load\n", entry.miName);
            }
        }

        zzip_dir_close(dir);

        // Require at least walk left + walk right
        bool hasWalk = (mClips[(int)PlayerMotionID::WalkLeft].numFrames > 0 &&
                        mClips[(int)PlayerMotionID::WalkRight].numFrames > 0);

        if (!hasWalk) {
            std::fprintf(stderr, "MotionService: ERROR — walk motions not loaded!\n");
        }

        mLoaded = hasWalk;
        std::fprintf(stderr, "MotionService: %d clips loaded, %d failed%s\n",
                     loaded, failed, mLoaded ? "" : " (UNUSABLE)");
        return mLoaded;
    }

    // ── Query ──

    /// Get a motion clip by player motion ID. Returns nullptr if not loaded.
    inline const MotionClip* getClip(PlayerMotionID id) const {
        int idx = static_cast<int>(id);
        if (idx < 0 || idx >= static_cast<int>(PlayerMotionID::Count))
            return nullptr;
        return mClips[idx].numFrames > 0 ? &mClips[idx] : nullptr;
    }

    /// Get a motion clip by name. Returns nullptr if not found.
    inline const MotionClip* getClipByName(const std::string &name) const {
        auto it = mNameIndex.find(name);
        if (it == mNameIndex.end()) return nullptr;
        return &mClips[it->second];
    }

    /// Is the service loaded and usable?
    inline bool isLoaded() const { return mLoaded; }

    /// Number of clips successfully loaded.
    inline int clipCount() const {
        int n = 0;
        for (const auto &c : mClips)
            if (c.numFrames > 0) ++n;
        return n;
    }

    // ── Sampling utilities (static, no state) ──

    /// Sample root translation at a given time within a clip.
    /// Uses linear interpolation between adjacent frames for smooth
    /// sub-frame sampling at any physics rate (12.5/60/120 Hz).
    /// Time is clamped to [0, duration].
    static inline Vector3 sampleRootTranslation(const MotionClip &clip, float time) {
        if (clip.rootTranslation.empty()) return Vector3(0.0f);
        if (clip.numFrames <= 1) return clip.rootTranslation[0];

        // Clamp time to clip range
        time = std::max(0.0f, std::min(time, clip.duration));

        // Continuous frame index
        float frameFloat = time * static_cast<float>(clip.freq);
        int frameA = static_cast<int>(frameFloat);
        int frameB = frameA + 1;

        // Clamp frame indices
        if (frameA >= clip.numFrames - 1) {
            return clip.rootTranslation[clip.numFrames - 1];
        }
        frameA = std::max(0, frameA);
        frameB = std::min(frameB, clip.numFrames - 1);

        // Linear interpolation
        float frac = frameFloat - static_cast<float>(frameA);
        return glm::mix(clip.rootTranslation[frameA],
                        clip.rootTranslation[frameB], frac);
    }

    /// Get motion flags active at a given time.
    /// Returns the combined flags for the frame at floor(time * freq).
    /// Returns 0 if no flags at that frame.
    static inline uint32_t getFlagsAtTime(const MotionClip &clip, float time) {
        if (clip.flags.empty()) return 0;
        int frame = static_cast<int>(time * static_cast<float>(clip.freq));
        frame = std::max(0, std::min(frame, clip.numFrames - 1));

        uint32_t result = 0;
        for (const auto &[f, bits] : clip.flags) {
            if (f == frame) result |= bits;
        }
        return result;
    }

    /// Get all flags fired between two times (exclusive start, inclusive end).
    /// Used by MotionPlayback to emit flag callbacks for frames crossed
    /// during a single physics step.
    static inline uint32_t getFlagsBetween(const MotionClip &clip,
                                            float timeStart, float timeEnd) {
        if (clip.flags.empty()) return 0;
        int frameStart = static_cast<int>(timeStart * static_cast<float>(clip.freq));
        int frameEnd   = static_cast<int>(timeEnd * static_cast<float>(clip.freq));
        frameStart = std::max(-1, frameStart);  // -1 so frame 0 is included on first call
        frameEnd   = std::min(frameEnd, clip.numFrames - 1);

        uint32_t result = 0;
        for (const auto &[f, bits] : clip.flags) {
            if (f > frameStart && f <= frameEnd) result |= bits;
        }
        return result;
    }

private:
    // ── CRF file reading helper ──
    // Reads an entire file from the open CRF archive into a byte vector.
    // Returns empty vector on failure.
    static inline std::vector<uint8_t> readCRFFile(ZZIP_DIR *dir, const std::string &name) {
        std::vector<uint8_t> data;
        ZZIP_FILE *fp = zzip_file_open(dir, name.c_str(), ZZIP_CASELESS);
        if (!fp) return data;

        data.reserve(4096);
        uint8_t tmp[4096];
        zzip_ssize_t n;
        while ((n = zzip_file_read(fp, tmp, sizeof(tmp))) > 0) {
            data.insert(data.end(), tmp, tmp + n);
        }
        zzip_file_close(fp);
        return data;
    }

    // ── MI file parser ──
    // Parses the .mi binary header to extract frame count, freq, name,
    // component list, and motion flags. Returns the index of the root
    // translation component (CM_TRANS) in the component array, or -1
    // if not found.
    inline bool parseMI(const std::vector<uint8_t> &data, MotionClip &clip,
                        int &rootCompIndex) {
        rootCompIndex = -1;

        if (data.size() < MI_HEADER_SIZE) {
            std::fprintf(stderr, "MotionService: MI file too small (%zu bytes)\n",
                         data.size());
            return false;
        }

        const uint8_t *p = data.data();

        // Read fixed header fields
        int32_t  type;       std::memcpy(&type,       p + 0,  4);
        uint32_t sig;        std::memcpy(&sig,        p + 4,  4);
        float    numFramesF; std::memcpy(&numFramesF, p + 8,  4);
        int32_t  freq;       std::memcpy(&freq,       p + 12, 4);
        int32_t  motNum;     std::memcpy(&motNum,     p + 16, 4);

        // Name: 12 bytes, null-terminated
        char nameBuf[13] = {};
        std::memcpy(nameBuf, p + 20, 12);
        clip.name = nameBuf;

        clip.numFrames = static_cast<int>(numFramesF);
        clip.freq = freq > 0 ? freq : 30;
        clip.duration = static_cast<float>(clip.numFrames) / static_cast<float>(clip.freq);

        // On-disk layout of mps_motion (after the 96-byte mps_motion_info):
        //
        //   int32 num_components     (4 bytes)
        //   int32 components_ptr     (4 bytes, always NULL on disk — serialized pointer)
        //   int32 num_flags          (4 bytes)
        //   int32 flags_ptr          (4 bytes, always NULL on disk — serialized pointer)
        //   MIComponent[num_components]  (12 bytes each)
        //   MIFlag[num_flags]            (8 bytes each)
        //
        // The two pointer fields are part of the in-memory mps_motion struct
        // (32-bit pointers). On disk they're serialized as NULL and the actual
        // data arrays follow after all four fields. The original engine writes
        // the entire struct with ITagFile_Write, then the data arrays inline.

        size_t offset = MI_HEADER_SIZE;

        // Read the four mps_motion fields after mps_motion_info (16 bytes total)
        if (offset + 16 > data.size()) {
            // No motion data — might be a virtual motion
            return clip.numFrames > 0;
        }

        int32_t numComponents;
        std::memcpy(&numComponents, p + offset, 4);
        offset += 4;

        // Skip components pointer (NULL on disk, was a 32-bit in-memory pointer)
        offset += 4;

        int32_t numFlags;
        std::memcpy(&numFlags, p + offset, 4);
        offset += 4;

        // Skip flags pointer (NULL on disk, was a 32-bit in-memory pointer)
        offset += 4;

        // Validate counts
        if (numComponents < 0 || numComponents > 64) {
            std::fprintf(stderr, "MotionService: MI bad num_components=%d\n", numComponents);
            return false;
        }

        // Read component array
        size_t compDataSize = static_cast<size_t>(numComponents) * sizeof(MIComponent);
        if (offset + compDataSize > data.size()) {
            std::fprintf(stderr, "MotionService: MI truncated (components)\n");
            return false;
        }

        for (int i = 0; i < numComponents; ++i) {
            MIComponent comp;
            std::memcpy(&comp, p + offset, sizeof(MIComponent));
            offset += sizeof(MIComponent);

            if (comp.type == CM_TRANS) {
                // The handle field is the index into the MC offset table,
                // NOT the sequential position in the MI component list.
                // Using 'i' here would read rotation data instead of translation.
                rootCompIndex = comp.handle;
            }
        }

        // Read flag array — motion event markers (footfalls, triggers, etc.)
        if (numFlags > 0 && numFlags < 256 &&
            offset + static_cast<size_t>(numFlags) * sizeof(MIFlag) <= data.size()) {
            clip.flags.reserve(numFlags);
            for (int i = 0; i < numFlags; ++i) {
                MIFlag flag;
                std::memcpy(&flag, p + offset, sizeof(MIFlag));
                offset += sizeof(MIFlag);
                clip.flags.emplace_back(flag.frame, flag.flags);
            }
        }

        return clip.numFrames > 0;
    }

    // ── MC file parser ──
    // Parses the .mc binary file to extract root translation data.
    // rootCompIndex identifies which entry in the MC offset table
    // contains the translation data (from the MI component list).
    inline bool parseMC(const std::vector<uint8_t> &data, int numFrames,
                        int rootCompIndex, MotionClip &clip) {
        if (data.size() < 4) {
            std::fprintf(stderr, "MotionService: MC file too small (%zu bytes)\n",
                         data.size());
            return false;
        }

        const uint8_t *p = data.data();

        // Read offset table header
        int32_t numEntries;
        std::memcpy(&numEntries, p, 4);

        if (numEntries <= 0 || numEntries > 64) {
            std::fprintf(stderr, "MotionService: MC bad numEntries=%d\n", numEntries);
            return false;
        }

        if (rootCompIndex < 0 || rootCompIndex >= numEntries) {
            std::fprintf(stderr, "MotionService: MC rootCompIndex=%d out of range (entries=%d)\n",
                         rootCompIndex, numEntries);
            return false;
        }

        // Read offset table
        size_t tableSize = 4 + numEntries * 4;
        if (tableSize > data.size()) {
            std::fprintf(stderr, "MotionService: MC truncated (offset table)\n");
            return false;
        }

        uint32_t rootOffset;
        std::memcpy(&rootOffset, p + 4 + rootCompIndex * 4, 4);

        // Root translation: float[3] × numFrames starting at rootOffset
        size_t translationSize = static_cast<size_t>(numFrames) * 3 * sizeof(float);
        if (rootOffset + translationSize > data.size()) {
            std::fprintf(stderr, "MotionService: MC truncated at root translation "
                         "(offset=%u, need=%zu, have=%zu)\n",
                         rootOffset, rootOffset + translationSize, data.size());
            return false;
        }

        // Extract per-frame root translation
        clip.rootTranslation.resize(numFrames);
        clip.zMin = std::numeric_limits<float>::max();
        clip.zMax = std::numeric_limits<float>::lowest();

        const float *floats = reinterpret_cast<const float*>(p + rootOffset);
        for (int i = 0; i < numFrames; ++i) {
            float x = floats[i * 3 + 0];
            float y = floats[i * 3 + 1];
            float z = floats[i * 3 + 2];
            clip.rootTranslation[i] = Vector3(x, y, z);

            if (z < clip.zMin) clip.zMin = z;
            if (z > clip.zMax) clip.zMax = z;
        }

        // Compute total forward displacement
        if (numFrames > 0) {
            clip.totalForwardDisp = clip.rootTranslation[numFrames - 1].x;
        }

        return true;
    }

    // ── Combined MI+MC parser ──
    // Loads and parses a .mi + .mc file pair from the CRF archive.
    inline bool parseMotionPair(ZZIP_DIR *dir, const char *miBaseName,
                                const char *mcBaseName, MotionClip &clip) {
        // Load .mi file
        std::string miPath = std::string(miBaseName) + ".mi";
        auto miData = readCRFFile(dir, miPath);
        if (miData.empty()) {
            std::fprintf(stderr, "MotionService: could not read %s\n", miPath.c_str());
            return false;
        }

        // Parse MI header
        int rootCompIndex = -1;
        if (!parseMI(miData, clip, rootCompIndex)) {
            std::fprintf(stderr, "MotionService: failed to parse MI for %s\n", miBaseName);
            return false;
        }

        if (rootCompIndex < 0) {
            // No root translation component — might be rotation-only motion.
            // For v1 we only need root translation; treat as non-fatal.
            std::fprintf(stderr, "MotionService: %s has no root translation component\n",
                         miBaseName);
            // Still valid clip with just metadata + flags, but no root data
            return true;
        }

        // Load .mc file
        std::string mcPath = std::string(mcBaseName) + ".mc";
        auto mcData = readCRFFile(dir, mcPath);
        if (mcData.empty()) {
            std::fprintf(stderr, "MotionService: could not read %s\n", mcPath.c_str());
            return false;
        }

        // Parse MC data
        if (!parseMC(mcData, clip.numFrames, rootCompIndex, clip)) {
            std::fprintf(stderr, "MotionService: failed to parse MC for %s\n", mcBaseName);
            return false;
        }

        return true;
    }

    // ── Storage ──
    std::array<MotionClip, static_cast<int>(PlayerMotionID::Count)> mClips;
    std::unordered_map<std::string, int> mNameIndex;  // name → PlayerMotionID index
    bool mLoaded = false;
};

} // namespace Darkness
