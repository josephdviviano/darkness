/******************************************************************************
 *
 *    This file is part of the darkness project
 *    Copyright (C) 2025 darkness contributors
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

// Mission render-parameters parser. Reads the RENDPARAMS chunk from a .mis
// file (or returns engine-spec default values when absent), then computes the
// derived sun colour and sun normal used by the per-object lighting code.
//
// On-disk layout (84 bytes, Dark Engine convention):
//   char[16]    pal_res             (palette resource name; not used by us)
//   vec3        ambient_light       (RGB ambient base for object lighting)
//   int32       use_sun             (BOOL32)
//   int32       sunlight_quad       (BOOL32 — quadratic falloff toggle)
//   vec3        sunlight_vector     (sun direction; not pre-normalized)
//   float       sun_h, sun_s, sun_b (HSB; brightness 0..1023)
//   vec3        sun_scaled_rgb      (cached on disk; we recompute)
//   vec3        sun_rgb             (cached on disk; we recompute)
//
// Defaults when chunk missing: ambient=(0.25,0.25,0.25), use_sun=false,
// sunlight_vector=(0.25,0.1,-1), sun_h=0, sun_s=1, sun_b=100. These match the
// engine's hard-coded reset values.

#pragma once

#include "DarknessMath.h"
#include "File.h"
#include "FileGroup.h"
#include "FileCompat.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>

namespace Darkness {

// All values in 0..1 range except sunBrightness (0..1023, the editor field).
struct RenderParams {
    Vector3 ambientLight  = Vector3(0.25f, 0.25f, 0.25f);
    Vector3 sunlightVector = Vector3(0.25f, 0.1f, -1.0f);  // raw direction
    Vector3 sunlightNorm   = Vector3(0.0f, 0.0f, 0.0f);    // -normalize(vector)
    Vector3 sunRgb         = Vector3(1.0f, 1.0f, 1.0f);    // 0..1
    Vector3 sunScaledRgb   = Vector3(100.0f, 100.0f, 100.0f); // sunRgb * sunB
    float   sunHue         = 0.0f;
    float   sunSaturation  = 1.0f;
    float   sunBrightness  = 100.0f;
    bool    useSun         = false;
    bool    sunlightQuad   = false;
    bool    fromChunk      = false; // false = used defaults
};

// Convert HSB to RGB using the engine's piecewise-linear hue mapping. Output
// channels are in 0..1 range. Saturation 0 produces white; saturation 1
// produces a fully-saturated hue. The 1/3 / 2/3 hue boundaries divide red→
// green, green→blue, blue→red, matching the editor preview.
inline Vector3 hsbToRgb(float hue, float saturation) {
    // Wrap the hue into [0,1) tolerantly. Negative or >1 hues would otherwise
    // produce nonsense; the editor clamps to the slider so this is mostly
    // belt-and-braces for hand-edited missions.
    if (!std::isfinite(hue)) hue = 0.0f;
    hue = hue - std::floor(hue);
    if (saturation < 0.0f) saturation = 0.0f;
    if (saturation > 1.0f) saturation = 1.0f;

    float h3 = hue * 3.0f;
    float r, g, b;
    if (h3 < 1.0f) {
        // red → green
        r = 1.0f - h3;
        g = h3;
        b = 0.0f;
    } else if (h3 < 2.0f) {
        // green → blue
        g = 2.0f - h3;
        b = h3 - 1.0f;
        r = 0.0f;
    } else {
        // blue → red
        b = 3.0f - h3;
        r = h3 - 2.0f;
        g = 0.0f;
    }

    // Blend with white toward low saturation.
    float isat = 1.0f - saturation;
    r = r * saturation + isat;
    g = g * saturation + isat;
    b = b * saturation + isat;
    return Vector3(r, g, b);
}

// Recompute derived fields (sunRgb, sunScaledRgb, sunlightNorm) from the
// stored hue/saturation/brightness/vector. The sun normal points TOWARD the
// sun (i.e. opposite to the direction the sunlight travels), so adding
// `sunlightNorm * SUNLIGHT_DISTANCE` to a point places a virtual sun light
// "above" it for falloff calculations.
inline void recomputeRenderParamsDerived(RenderParams &p) {
    p.sunRgb = hsbToRgb(p.sunHue, p.sunSaturation);
    p.sunScaledRgb = p.sunRgb * p.sunBrightness;

    float vlen = glm::length(p.sunlightVector);
    if (vlen > 1e-6f)
        p.sunlightNorm = -(p.sunlightVector / vlen);
    else
        p.sunlightNorm = Vector3(0.0f, 0.0f, 1.0f); // straight up fallback
}

// Parse RENDPARAMS chunk from .mis file. If missing or unreadable, returns the
// engine's documented defaults (with derived fields populated). Tolerates
// short payloads (older chunk versions) by falling back to defaults for any
// trailing field that's not present.
inline RenderParams parseRenderParamsChunk(const std::string &misPath) {
    RenderParams p;

    try {
        FilePtr fp(new StdFile(misPath, File::FILE_R));
        FileGroupPtr db(new DarkFileGroup(fp));

        if (!db->hasFile("RENDPARAMS")) {
            std::fprintf(stderr,
                         "No RENDPARAMS chunk — using default ambient/sun values\n");
            recomputeRenderParamsDerived(p);
            return p;
        }

        FilePtr chunk = db->getFile("RENDPARAMS");
        size_t chunkSize = chunk->size();

        // Read pal_res but discard — palette is set up elsewhere.
        char palRes[16] = {};
        if (chunkSize >= 16) chunk->read(palRes, 16);

        if (chunkSize >= 16 + 12) {
            chunk->read(&p.ambientLight, sizeof(p.ambientLight));
        }
        if (chunkSize >= 16 + 12 + 4) {
            int32_t useSun = 0;
            chunk->read(&useSun, sizeof(useSun));
            p.useSun = (useSun != 0);
        }
        if (chunkSize >= 16 + 12 + 4 + 4) {
            int32_t quad = 0;
            chunk->read(&quad, sizeof(quad));
            p.sunlightQuad = (quad != 0);
        }
        if (chunkSize >= 16 + 12 + 4 + 4 + 12) {
            chunk->read(&p.sunlightVector, sizeof(p.sunlightVector));
        }
        if (chunkSize >= 16 + 12 + 4 + 4 + 12 + 4) {
            chunk->read(&p.sunHue, sizeof(p.sunHue));
        }
        if (chunkSize >= 16 + 12 + 4 + 4 + 12 + 4 + 4) {
            chunk->read(&p.sunSaturation, sizeof(p.sunSaturation));
        }
        if (chunkSize >= 16 + 12 + 4 + 4 + 12 + 4 + 4 + 4) {
            chunk->read(&p.sunBrightness, sizeof(p.sunBrightness));
        }
        // Trailing sun_scaled_rgb and sun_rgb on disk are ignored — we always
        // recompute from h/s/b for consistency with the editor's behavior.

        p.fromChunk = true;
        std::fprintf(stderr,
                     "RENDPARAMS: ambient=(%.2f,%.2f,%.2f) sun=%s "
                     "h=%.2f s=%.2f b=%.1f vec=(%.2f,%.2f,%.2f)\n",
                     p.ambientLight.x, p.ambientLight.y, p.ambientLight.z,
                     p.useSun ? "on" : "off",
                     p.sunHue, p.sunSaturation, p.sunBrightness,
                     p.sunlightVector.x, p.sunlightVector.y, p.sunlightVector.z);
    } catch (const std::exception &e) {
        std::fprintf(stderr,
                     "Failed to read RENDPARAMS: %s — using defaults\n", e.what());
        p = RenderParams{};
    }

    recomputeRenderParamsDerived(p);
    return p;
}

} // namespace Darkness
