// Auto-generated shader bytecode for bgfx Metal backend.
// Compiled from shaders/vs_lightmapped.sc and shaders/fs_lightmapped.sc using shaderc.
// To regenerate:
//   shaderc -f vs_lightmapped.sc -o vs_lightmapped.metal.bin --type vertex --platform osx -p metal -i <bgfx_include> --varyingdef varying_lightmapped.def.sc
//   shaderc -f fs_lightmapped.sc -o fs_lightmapped.metal.bin --type fragment --platform osx -p metal -i <bgfx_include> --varyingdef varying_lightmapped.def.sc
//   bin2c -f vs_lightmapped.metal.bin -o vs_lightmapped_metal.h -n vs_lightmapped_metal
//   bin2c -f fs_lightmapped.metal.bin -o fs_lightmapped_metal.h -n fs_lightmapped_metal

#pragma once

#include "vs_lightmapped_metal.h"
#include "fs_lightmapped_metal.h"
