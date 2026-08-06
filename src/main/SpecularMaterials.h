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

// Per-material specular presets for the dominant-direction lightmap term —
// the same texture-name → material-class pipeline the acoustic system uses
// (AcousticMaterials.h: keyword matching over TXLIST names, abbreviation
// aliases derived empirically via `txlist_audit`). One keyword table serves
// both systems; this header only attaches SPECULAR physics to each class.
//
// The values follow physically-based convention rather than classic
// fixed-function Phong tables, because the styling goal is mood, not gloss
// (user direction 2026-08-06: highlight contrast and contours WITHOUT
// brightening — the "plastic" look is exactly specular without Fresnel):
//
//  * f0 — Schlick Fresnel reflectance at normal incidence. Dielectrics sit
//    in the published 0.02–0.05 band (the PBR-standard 0.04 default; ice
//    1.31 IOR → 0.018), which is what keeps surfaces nearly matte face-on
//    and pushes the sheen to grazing angles — contours and silhouettes.
//  * f90 — grazing-angle reflectance. Physically 1.0 for smooth surfaces;
//    reduced for rough/fibrous classes where microfacet self-shadowing eats
//    the grazing response (the standard rough-surface correction).
//  * exponent — Blinn-Phong shininess, normalised by (n+8)/8 in the shader
//    so energy concentrates instead of adding as n rises.
//  * metalness — conductors tint their specular by albedo (dielectric
//    highlights are white). Thief metal is old, dark iron: F0 well below
//    polished iron's ~0.56.
//
// References: Schlick Fresnel + "everything has Fresnel" argument
// (filmicworlds.com/blog/everything-has-fresnel); dielectric F0 band
// (Adobe/Substance PBR guide, standard 0.02–0.05 with 0.04 default).

#pragma once

#include "audio/AcousticMaterials.h"

#include <array>
#include <cstdint>
#include <string>

namespace Darkness {

struct SpecularPreset {
    float f0        = 0.035f;  // normal-incidence reflectance
    float f90       = 0.50f;   // grazing reflectance (rough-surface reduced)
    float exponent  = 24.0f;   // Blinn-Phong shininess
    float metalness = 0.0f;    // 1 = tint specular by albedo
};

inline const SpecularPreset &specularPresetForClass(const std::string &cls) {
    // Roughness ordering within the dielectric band: glazed/vitreous at the
    // top (tight glints), worked masonry in the middle (broad low sheen),
    // fibrous/granular at the bottom (grazing-only whisper).
    static const struct { const char *cls; SpecularPreset p; } kTable[] = {
        {"glass",    {0.040f, 1.00f, 160.0f, 0.0f}},
        {"ice",      {0.018f, 1.00f, 140.0f, 0.0f}},
        {"ceramic",  {0.045f, 0.90f, 110.0f, 0.0f}},
        {"tile",     {0.045f, 0.85f,  90.0f, 0.0f}},
        // Old, dark, unpolished iron — far below polished iron's 0.56, but
        // still albedo-tinted and glossier than masonry.
        {"metal",    {0.220f, 0.90f,  56.0f, 0.85f}},
        {"wood",     {0.040f, 0.60f,  26.0f, 0.0f}},
        {"leaf",     {0.040f, 0.60f,  30.0f, 0.0f}},   // waxy
        {"stone",    {0.040f, 0.55f,  20.0f, 0.0f}},
        {"concrete", {0.040f, 0.50f,  16.0f, 0.0f}},
        {"brick",    {0.040f, 0.45f,  14.0f, 0.0f}},
        {"plaster",  {0.035f, 0.50f,  12.0f, 0.0f}},
        {"rock",     {0.040f, 0.45f,  12.0f, 0.0f}},
        {"bark",     {0.030f, 0.35f,  10.0f, 0.0f}},
        {"gravel",   {0.025f, 0.30f,  10.0f, 0.0f}},
        {"dirt",     {0.020f, 0.30f,   8.0f, 0.0f}},
        {"carpet",   {0.020f, 0.25f,   8.0f, 0.0f}},
        {"hay",      {0.015f, 0.20f,   6.0f, 0.0f}},
    };
    static const SpecularPreset kGeneric{};   // modest matte fallback
    for (const auto &e : kTable)
        if (cls == e.cls) return e.p;
    return kGeneric;
}

// Texture name → specular preset, through the acoustic keyword pipeline.
// `wasGeneric` reports fallbacks so the caller can audit coverage loudly —
// the same discipline txlist_audit applies to the acoustic side.
inline const SpecularPreset &specularPresetForTexture(const std::string &texName,
                                                      bool *wasGeneric = nullptr) {
    const std::string keyword = lookupAcousticMaterialKeyword(texName);
    const std::string cls = acousticMaterialClass(keyword);
    if (wasGeneric) *wasGeneric = (cls == "generic");
    return specularPresetForClass(cls);
}

} // namespace Darkness
