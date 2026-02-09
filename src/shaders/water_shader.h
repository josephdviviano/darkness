// Water shader: textured fragment without alpha discard.
// Vertex shader is shared with textured_shader.h (vs_textured_metal).
// Only the fragment shader is water-specific — it omits the alpha discard
// so that vertex alpha (35% for water) is preserved instead of killed.
// To regenerate:
//   shaderc -f fs_water.sc -o fs_water.metal.bin --type fragment --platform osx -p metal -i <bgfx_include> --varyingdef varying_water.def.sc
//   bin2c -f fs_water.metal.bin -o fs_water_metal.h -n fs_water_metal

#pragma once

// Note: vs_textured_metal is already included via textured_shader.h
#include "fs_water_metal.h"
