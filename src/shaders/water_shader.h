// Water shader: vertex displacement + textured fragment with UV distortion.
// Vertex shader adds sine-wave Z displacement for visible water undulation.
// Fragment shader scrolls and wobbles UVs, no alpha discard (preserves vertex alpha).
// To regenerate:
//   shaderc -f vs_water.sc -o vs_water.metal.bin --type vertex --platform osx -p metal -i <bgfx_include> --varyingdef varying_water.def.sc
//   shaderc -f fs_water.sc -o fs_water.metal.bin --type fragment --platform osx -p metal -i <bgfx_include> --varyingdef varying_water.def.sc
//   bin2c -f vs_water.metal.bin -o vs_water_metal.h -n vs_water_metal
//   bin2c -f fs_water.metal.bin -o fs_water_metal.h -n fs_water_metal

#pragma once

#include "vs_water_metal.h"
#include "fs_water_metal.h"
