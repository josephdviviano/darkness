/*
 * Darkness — cross-platform port of the Dark Engine.
 * Copyright (C) 2024 Joseph Viviano
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef DARKNESS_OBJECT_LIGHTING_CONSTANTS_H
#define DARKNESS_OBJECT_LIGHTING_CONSTANTS_H

/* Maximum number of lights evaluated per object in the per-vertex
 * Lambertian shading path.
 *
 * This file is the single source of truth for that cap. It is included
 * by both:
 *   - the C++ array-build (src/main/ObjectIllumination.h, exposed as
 *     constexpr int kObjectLightCap), and
 *   - the per-vertex shaders (vs_*_pervertex.sc), which size their
 *     uniform arrays and loop bound from OBJECT_LIGHT_CAP.
 *
 * To raise or lower the cap: edit the value below and rebuild. Both
 * worlds will pick it up. The shader bytecode is regenerated whenever
 * this file changes (CMake tracks it as a shader source dependency).
 *
 * Why 32: the engine's per-cell light_indices cap is 96, but in practice
 * the per-object visible-light count is far smaller (4–12 in heavily
 * lit rooms after the shadow cache filters occluded lights). 32 is
 * generous headroom for that, plus a few dynamic lights.
 *
 * Why not 96 (the engine cap): each light costs 3 vec4 of vertex-shader
 * uniform space, and bgfx's Metal renderer ring-allocates per-draw
 * uniforms in a fixed 8 MB per-frame buffer. With cap=96 each per-
 * vertex draw consumes ~7 KB; missions with many visible objects (300+
 * draw calls per frame) exhaust the ring mid-frame and the next write
 * lands in a read-only adjacent allocation → SIGBUS in
 * RendererContextMtl::setShaderUniform. cap=32 brings per-draw cost
 * down to ~2 KB and leaves comfortable headroom on every mission.
 *
 * If a future mission genuinely has objects affected by >32 lights,
 * options are: bump the cap (verifying the per-frame uniform total
 * stays within budget), sort the cell light list by contribution
 * before packing, or move the per-vertex uniforms into a UBO that
 * doesn't share the ring.
 */
#define OBJECT_LIGHT_CAP 32

#endif /* DARKNESS_OBJECT_LIGHTING_CONSTANTS_H */
