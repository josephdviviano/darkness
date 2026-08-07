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

// Disk cache for the lightmap re-bake — the piece that turns a ~10 s load
// cost into a shippable one (PLAN.HIGH_RES_SHADOWS.md S2). Written beside the
// mission the way the audio probe bakes are (`mission.probes` precedent):
// `<mission>.lmbake`, a versioned header + one zlib blob carrying the base
// atlas and the per-polygon animated overlay records.
//
// Validity is the WHOLE key: file format version, formula version (bump
// kLmBakeFormulaVersion whenever LightmapBake.h changes what the same
// parameters produce), every bake parameter, and a content hash of the
// mission file itself. Any mismatch is a MISS reported with its reason —
// never a silent fallback — and the caller re-bakes and overwrites.

#pragma once

#include "LightmapAtlas.h"
#include "LightmapBake.h"

#include <zlib.h>

#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#include <sys/types.h>
#endif

namespace Darkness {

constexpr uint32_t kLmBakeCacheMagic    = 0x424D4C44; // "DLMB" little-endian
// v2: gather parameters (bounce samples, AO strength) joined the key.
// v3: optional dominant-direction atlas appended to the blob.
// v4: falloff-naturalisation parameters (reach hops, soft radius) in the key.
// v5: physical-falloff parameters (mode, anchor) in the key.
// v6: throw-derived intensity (throwAlpha) in the key.
// v7: S3 door shadows (doorShadows flag) in the key — door-adjacent
//     lights move from base to overlays, so the decomposition differs.
constexpr uint32_t kLmBakeCacheVersion  = 7;
// Bump when the bake FORMULA changes meaning — i.e. when identical parameters
// would now produce different lumels. Parameter changes do not need a bump;
// they are part of the key.
//   v2: Continuous storage grew the smooth toe at the zero-crossing.
//   v3: bright synthesis no longer overwrites slots of anim lights WITHOUT
//       overlays (static-baked lights keep disk bright in the base — the
//       MISS6 cathedral-lamp overbright fix). The synthesis feeds the bake
//       input, so identical parameters now produce a different base.
//   v4: door-overlay DIRECTION bake went door-transparent — REVERTED
//       (leaked specular through closed doors: light.rgb is total energy,
//       ambient fed the lobe).
//   v5: direction is door-occluded at load and re-encoded at runtime by
//       door events; v4 caches carry the leaky field and must die.
constexpr uint32_t kLmBakeFormulaVersion = 5;

struct LmBakeCacheKey {
    uint64_t missionHash = 0;
    int32_t  density = 1;
    int32_t  samples = 1;
    int32_t  supersample = 1;
    float    emitter = 0.0f;
    uint32_t fivebit = 0;
    int32_t  bounceSamples = 0;
    float    aoStrength = 0.0f;
    int32_t  reachExpand = 0;
    float    softRadius = 0.0f;
    int32_t  falloffPhysical = 0;
    float    falloffAnchor = 0.0f;
    float    throwAlpha = 0.0f;
    int32_t  doorShadows = 0;
};

// Where the cache lives: ~/darkness/{gameName}/baked_lightmaps/{mission}.lmbake
// — the same per-user layout the audio probe bakes use, and for the same
// reason: mission directories are often read-only (CD images, reference
// trees), and a cache that silently fails to write re-bakes every load.
// Creates the directory chain as a side effect.
inline std::string getLightmapBakeCachePath(const std::string &misPath,
                                            const std::string &gameName
                                                = "thief2") {
    std::string missionName;
    const auto lastSlash = misPath.find_last_of("/\\");
    std::string filename = (lastSlash != std::string::npos)
        ? misPath.substr(lastSlash + 1) : misPath;
    const auto dotPos = filename.rfind('.');
    missionName = (dotPos != std::string::npos)
        ? filename.substr(0, dotPos) : filename;
    for (auto &c : missionName) c = static_cast<char>(std::tolower(c));

    const char *home = std::getenv("HOME");
    if (!home) home = ".";
    const std::string dir =
        std::string(home) + "/darkness/" + gameName + "/baked_lightmaps";

    std::string pathSoFar;
    for (size_t i = 0; i < dir.size(); ++i) {
        if (dir[i] == '/' || i == dir.size() - 1) {
            pathSoFar = dir.substr(0, i + 1);
#ifdef _WIN32
            _mkdir(pathSoFar.c_str());
#else
            mkdir(pathSoFar.c_str(), 0755);
#endif
        }
    }
    return dir + "/" + missionName + ".lmbake";
}

// FNV-1a over the file's bytes. The mission decides the bake, so the mission's
// content is the key — mtimes lie across copies and re-installs.
inline uint64_t hashFileContents(const std::string &path) {
    std::FILE *fp = std::fopen(path.c_str(), "rb");
    if (!fp) return 0;
    uint64_t h = 1469598103934665603ull;
    uint8_t buf[65536];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof(buf), fp)) > 0)
        for (size_t i = 0; i < n; ++i)
            h = (h ^ buf[i]) * 1099511628211ull;
    std::fclose(fp);
    return h;
}

namespace detail {
// Append plain-old-data to a byte vector / read it back with bounds checking.
template <typename T>
inline void put(std::vector<uint8_t> &v, const T &x) {
    const uint8_t *p = reinterpret_cast<const uint8_t *>(&x);
    v.insert(v.end(), p, p + sizeof(T));
}
template <typename T>
inline bool get(const std::vector<uint8_t> &v, size_t &off, T &x) {
    if (off + sizeof(T) > v.size()) return false;
    std::memcpy(&x, v.data() + off, sizeof(T));
    off += sizeof(T);
    return true;
}
inline bool getBytes(const std::vector<uint8_t> &v, size_t &off,
                     std::vector<uint8_t> &out, size_t n) {
    if (off + n > v.size()) return false;
    out.assign(v.begin() + off, v.begin() + off + n);
    off += n;
    return true;
}
} // namespace detail

inline bool writeLightmapBakeCache(const std::string &cachePath,
                                   const LmBakeCacheKey &key,
                                   const AtlasTexture &atlas,
                                   const std::vector<RebakedAnimPoly> &animPolys,
                                   const AtlasTexture *dirAtlas = nullptr) {
    // Serialise the logical stream, then compress once.
    std::vector<uint8_t> blob;
    blob.reserve(atlas.rgba.size() + (1u << 20));
    blob.insert(blob.end(), atlas.rgba.begin(), atlas.rgba.end());
    detail::put(blob, static_cast<uint32_t>(animPolys.size()));
    for (const auto &rec : animPolys) {
        detail::put(blob, rec.ci);
        detail::put(blob, rec.pi);
        detail::put(blob, static_cast<int32_t>(rec.w));
        detail::put(blob, static_cast<int32_t>(rec.h));
        detail::put(blob, static_cast<uint32_t>(rec.overlays.size()));
        for (size_t k = 0; k < rec.overlays.size(); ++k)
            detail::put(blob, k < rec.overlayLightIdx.size()
                                  ? rec.overlayLightIdx[k]
                                  : static_cast<int16_t>(-1));
        blob.insert(blob.end(), rec.baseCrop.begin(), rec.baseCrop.end());
        for (const auto &ov : rec.overlays)
            blob.insert(blob.end(), ov.begin(), ov.end());
    }
    // v3: dominant-direction atlas (same dimensions as the base atlas).
    detail::put(blob, static_cast<uint8_t>(
        (dirAtlas && !dirAtlas->rgba.empty()) ? 1 : 0));
    if (dirAtlas && !dirAtlas->rgba.empty())
        blob.insert(blob.end(), dirAtlas->rgba.begin(), dirAtlas->rgba.end());

    uLongf compBound = compressBound(static_cast<uLong>(blob.size()));
    std::vector<uint8_t> comp(compBound);
    if (compress2(comp.data(), &compBound, blob.data(),
                  static_cast<uLong>(blob.size()), 6) != Z_OK)
        return false;
    comp.resize(compBound);

    std::FILE *fp = std::fopen(cachePath.c_str(), "wb");
    if (!fp) return false;
    std::vector<uint8_t> hdr;
    detail::put(hdr, kLmBakeCacheMagic);
    detail::put(hdr, kLmBakeCacheVersion);
    detail::put(hdr, kLmBakeFormulaVersion);
    detail::put(hdr, key.missionHash);
    detail::put(hdr, key.density);
    detail::put(hdr, key.samples);
    detail::put(hdr, key.supersample);
    detail::put(hdr, key.emitter);
    detail::put(hdr, key.fivebit);
    detail::put(hdr, key.bounceSamples);
    detail::put(hdr, key.aoStrength);
    detail::put(hdr, key.reachExpand);
    detail::put(hdr, key.softRadius);
    detail::put(hdr, key.falloffPhysical);
    detail::put(hdr, key.falloffAnchor);
    detail::put(hdr, key.throwAlpha);
    detail::put(hdr, key.doorShadows);
    detail::put(hdr, static_cast<uint32_t>(atlas.size));
    detail::put(hdr, static_cast<uint64_t>(blob.size()));
    detail::put(hdr, static_cast<uint64_t>(comp.size()));
    bool ok = std::fwrite(hdr.data(), 1, hdr.size(), fp) == hdr.size()
           && std::fwrite(comp.data(), 1, comp.size(), fp) == comp.size();
    std::fclose(fp);
    return ok;
}

// Returns true on a valid hit; on any miss, `whyMiss` names the reason so the
// caller can report it — a cache that silently re-bakes and a cache that
// silently serves stale data are the two failure modes this format exists to
// exclude.
inline bool readLightmapBakeCache(const std::string &cachePath,
                                  const LmBakeCacheKey &key,
                                  AtlasTexture &atlas,
                                  std::vector<RebakedAnimPoly> &animPolys,
                                  std::string &whyMiss,
                                  AtlasTexture *dirAtlas = nullptr) {
    std::FILE *fp = std::fopen(cachePath.c_str(), "rb");
    if (!fp) { whyMiss = "no cache file"; return false; }
    std::fseek(fp, 0, SEEK_END);
    const long fsize = std::ftell(fp);
    std::fseek(fp, 0, SEEK_SET);
    std::vector<uint8_t> raw(static_cast<size_t>(std::max(0L, fsize)));
    const bool readOk =
        std::fread(raw.data(), 1, raw.size(), fp) == raw.size();
    std::fclose(fp);
    if (!readOk) { whyMiss = "short read"; return false; }

    size_t off = 0;
    uint32_t magic = 0, ver = 0, fver = 0, fivebit = 0, atlasSize = 0;
    uint64_t hash = 0, rawSize = 0, compSize = 0;
    int32_t density = 0, samples = 0, ss = 0, bounceSamples = 0;
    int32_t reachExpand = 0, falloffPhysical = 0;
    float emitter = 0.0f, aoStrength = 0.0f, softRadius = 0.0f;
    float falloffAnchor = 0.0f, throwAlpha = 0.0f;
    int32_t doorShadows = 0;
    using detail::get;
    if (!get(raw, off, magic) || magic != kLmBakeCacheMagic) {
        whyMiss = "bad magic"; return false;
    }
    if (!get(raw, off, ver) || ver != kLmBakeCacheVersion) {
        whyMiss = "format version mismatch"; return false;
    }
    if (!get(raw, off, fver) || fver != kLmBakeFormulaVersion) {
        whyMiss = "formula version mismatch"; return false;
    }
    if (!get(raw, off, hash) || !get(raw, off, density) ||
        !get(raw, off, samples) || !get(raw, off, ss) ||
        !get(raw, off, emitter) || !get(raw, off, fivebit) ||
        !get(raw, off, bounceSamples) || !get(raw, off, aoStrength) ||
        !get(raw, off, reachExpand) || !get(raw, off, softRadius) ||
        !get(raw, off, falloffPhysical) || !get(raw, off, falloffAnchor) ||
        !get(raw, off, throwAlpha) || !get(raw, off, doorShadows) ||
        !get(raw, off, atlasSize) || !get(raw, off, rawSize) ||
        !get(raw, off, compSize)) {
        whyMiss = "truncated header"; return false;
    }
    if (hash != key.missionHash) { whyMiss = "mission content changed"; return false; }
    if (density != key.density || samples != key.samples ||
        ss != key.supersample || emitter != key.emitter ||
        fivebit != key.fivebit || bounceSamples != key.bounceSamples ||
        aoStrength != key.aoStrength || reachExpand != key.reachExpand ||
        softRadius != key.softRadius ||
        falloffPhysical != key.falloffPhysical ||
        falloffAnchor != key.falloffAnchor ||
        throwAlpha != key.throwAlpha ||
        doorShadows != key.doorShadows) {
        whyMiss = "bake parameters differ"; return false;
    }
    if (off + compSize > raw.size()) { whyMiss = "truncated body"; return false; }

    std::vector<uint8_t> blob(static_cast<size_t>(rawSize));
    uLongf destLen = static_cast<uLongf>(rawSize);
    if (uncompress(blob.data(), &destLen, raw.data() + off,
                   static_cast<uLong>(compSize)) != Z_OK ||
        destLen != rawSize) {
        whyMiss = "decompression failed"; return false;
    }

    const size_t atlasBytes = static_cast<size_t>(atlasSize) * atlasSize * 4;
    size_t boff = 0;
    if (blob.size() < atlasBytes) { whyMiss = "atlas payload short"; return false; }
    atlas.size = static_cast<int>(atlasSize);
    atlas.rgba.assign(blob.begin(), blob.begin() + atlasBytes);
    boff = atlasBytes;

    uint32_t numPolys = 0;
    if (!get(blob, boff, numPolys)) { whyMiss = "poly count missing"; return false; }
    animPolys.clear();
    animPolys.reserve(numPolys);
    for (uint32_t i = 0; i < numPolys; ++i) {
        RebakedAnimPoly rec;
        int32_t w = 0, h = 0;
        uint32_t nOv = 0;
        if (!get(blob, boff, rec.ci) || !get(blob, boff, rec.pi) ||
            !get(blob, boff, w) || !get(blob, boff, h) ||
            !get(blob, boff, nOv)) {
            whyMiss = "overlay record truncated"; return false;
        }
        rec.w = w; rec.h = h;
        rec.overlayLightIdx.resize(nOv);
        for (uint32_t k = 0; k < nOv; ++k)
            if (!get(blob, boff, rec.overlayLightIdx[k])) {
                whyMiss = "overlay index truncated"; return false;
            }
        const size_t crop = static_cast<size_t>(w) * h * 3;
        if (!detail::getBytes(blob, boff, rec.baseCrop, crop)) {
            whyMiss = "base crop truncated"; return false;
        }
        rec.overlays.resize(nOv);
        for (uint32_t k = 0; k < nOv; ++k)
            if (!detail::getBytes(blob, boff, rec.overlays[k], crop)) {
                whyMiss = "overlay payload truncated"; return false;
            }
        animPolys.push_back(std::move(rec));
    }
    // v3: optional dominant-direction atlas.
    uint8_t hasDir = 0;
    if (!get(blob, boff, hasDir)) { whyMiss = "dir flag missing"; return false; }
    if (hasDir) {
        if (blob.size() - boff < atlasBytes) {
            whyMiss = "dir atlas truncated"; return false;
        }
        if (dirAtlas) {
            dirAtlas->size = static_cast<int>(atlasSize);
            dirAtlas->rgba.assign(blob.begin() + boff,
                                  blob.begin() + boff + atlasBytes);
        }
        boff += atlasBytes;
    } else if (dirAtlas) {
        dirAtlas->size = 0;
        dirAtlas->rgba.clear();
    }
    whyMiss.clear();
    return true;
}

} // namespace Darkness
