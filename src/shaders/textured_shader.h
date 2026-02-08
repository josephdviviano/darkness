// Auto-generated shader bytecode for bgfx Metal backend.
// Compiled from shaders/vs_textured.sc and shaders/fs_textured.sc using shaderc.
// To regenerate:
//   shaderc -f vs_textured.sc -o generated/vs_textured.metal.bin --type vertex --platform osx -p metal -i <bgfx_include> --varyingdef varying_textured.def.sc
//   shaderc -f fs_textured.sc -o generated/fs_textured.metal.bin --type fragment --platform osx -p metal -i <bgfx_include> --varyingdef varying_textured.def.sc
//   bin2c -f vs_textured.metal.bin -o vs_textured_metal.h -n vs_textured_metal
//   bin2c -f fs_textured.metal.bin -o fs_textured_metal.h -n fs_textured_metal

#pragma once

#include "vs_textured_metal.h"
#include "fs_textured_metal.h"
