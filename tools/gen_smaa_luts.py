#!/usr/bin/env python3
"""Generate src/main/SMAALookupTextures.h from the SMAA reference PNGs.

SMAA needs two precomputed lookup textures — AreaTex (the coverage-area table
the blending-weight pass indexes with a distance/crossing-edge pair) and
SearchTex (the packed edge-run-length table its bilinear search reads). They
are data, not code: the algorithm is meaningless without the exact reference
tables, so they are embedded rather than regenerated or approximated.

The reference distributes them as PNGs. We decode here, at generation time,
and emit raw bytes, so the engine needs no PNG decoder at runtime: bgfx's own
image library can decode PNG, but only by linking lodepng and tinyexr into
the render binary, which is a lot of dependency for two lookup tables that
never change.

Run when the tables need regenerating (essentially never — they are fixed by
the algorithm):

    python3 tools/gen_smaa_luts.py <dir-with-AreaTex.png-and-SearchTex.png> \
        src/main/SMAALookupTextures.h

Decoding is done here rather than with Pillow so the script has no
dependencies beyond the standard library.
"""

import os
import struct
import sys
import zlib

# The reference tables' dimensions. Asserted rather than inferred: a PNG of
# the wrong size would otherwise be embedded silently and produce a subtly
# wrong image that is very hard to trace back to here.
AREATEX_W, AREATEX_H = 160, 560
SEARCHTEX_W, SEARCHTEX_H = 64, 16


def _paeth(a, b, c):
    p = a + b - c
    pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
    if pa <= pb and pa <= pc:
        return a
    if pb <= pc:
        return b
    return c


def decode_png(path):
    """Minimal PNG decoder: 8-bit greyscale (type 0) and grey+alpha (type 4).

    Returns (width, height, channels, bytes). Those are the only two colour
    types the SMAA tables use, and anything else is rejected loudly rather
    than guessed at.
    """
    data = open(path, "rb").read()
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise SystemExit(f"{path}: not a PNG")

    width = height = 0
    channels = 0
    idat = bytearray()
    pos = 8
    while pos < len(data):
        (length,) = struct.unpack(">I", data[pos : pos + 4])
        ctype = data[pos + 4 : pos + 8]
        body = data[pos + 8 : pos + 8 + length]
        pos += 12 + length  # length + type + body + CRC

        if ctype == b"IHDR":
            width, height, bitdepth, colortype = struct.unpack(">IIBB", body[:10])
            interlace = body[12]
            if bitdepth != 8:
                raise SystemExit(f"{path}: bit depth {bitdepth}, expected 8")
            if interlace != 0:
                raise SystemExit(f"{path}: interlaced PNGs are not supported")
            if colortype == 0:
                channels = 1
            elif colortype == 4:
                channels = 2
            else:
                raise SystemExit(
                    f"{path}: colour type {colortype}, expected 0 (grey) or 4 (grey+alpha)"
                )
        elif ctype == b"IDAT":
            idat += body
        elif ctype == b"IEND":
            break

    if width == 0 or height == 0 or channels == 0:
        raise SystemExit(f"{path}: no IHDR chunk")

    raw = zlib.decompress(bytes(idat))
    stride = width * channels
    out = bytearray(width * height * channels)
    prev = bytearray(stride)
    src = 0
    for y in range(height):
        ftype = raw[src]
        src += 1
        line = bytearray(raw[src : src + stride])
        src += stride
        if ftype == 1:  # Sub
            for i in range(channels, stride):
                line[i] = (line[i] + line[i - channels]) & 0xFF
        elif ftype == 2:  # Up
            for i in range(stride):
                line[i] = (line[i] + prev[i]) & 0xFF
        elif ftype == 3:  # Average
            for i in range(stride):
                left = line[i - channels] if i >= channels else 0
                line[i] = (line[i] + ((left + prev[i]) >> 1)) & 0xFF
        elif ftype == 4:  # Paeth
            for i in range(stride):
                left = line[i - channels] if i >= channels else 0
                upleft = prev[i - channels] if i >= channels else 0
                line[i] = (line[i] + _paeth(left, prev[i], upleft)) & 0xFF
        elif ftype != 0:
            raise SystemExit(f"{path}: unknown filter type {ftype} on row {y}")
        out[y * stride : (y + 1) * stride] = line
        prev = line

    return width, height, channels, bytes(out)


def emit_array(fh, name, blob):
    fh.write(f"inline const uint8_t {name}[] = {{\n")
    for i in range(0, len(blob), 24):
        row = ", ".join(f"0x{b:02x}" for b in blob[i : i + 24])
        fh.write(f"    {row},\n")
    fh.write("};\n\n")


def main():
    if len(sys.argv) != 3:
        raise SystemExit(f"usage: {sys.argv[0]} <smaa-png-dir> <output-header>")
    src_dir, out_path = sys.argv[1], sys.argv[2]

    aw, ah, ac, area = decode_png(os.path.join(src_dir, "AreaTex.png"))
    sw, sh, sc, search = decode_png(os.path.join(src_dir, "SearchTex.png"))

    if (aw, ah, ac) != (AREATEX_W, AREATEX_H, 2):
        raise SystemExit(f"AreaTex.png is {aw}x{ah}x{ac}, expected "
                         f"{AREATEX_W}x{AREATEX_H}x2")
    if (sw, sh, sc) != (SEARCHTEX_W, SEARCHTEX_H, 1):
        raise SystemExit(f"SearchTex.png is {sw}x{sh}x{sc}, expected "
                         f"{SEARCHTEX_W}x{SEARCHTEX_H}x1")

    with open(out_path, "w") as fh:
        fh.write(f"""\
// GENERATED FILE — do not edit. Regenerate with tools/gen_smaa_luts.py.
//
// SMAA lookup tables, decoded from the reference AreaTex.png / SearchTex.png
// into the raw texture bytes the GPU wants. AreaTex is two-channel (RG8),
// SearchTex single-channel (R8); rows run top-down, i.e. row 0 is v = 0,
// which is how bgfx uploads texture data on every backend.
//
// The tables are fixed by the algorithm and carry no tunable parameters, so
// this file should never need regenerating.
//
// Copyright (C) 2013 Jorge Jimenez (jorge@iryoku.com)
// Copyright (C) 2013 Jose I. Echevarria (joseignacioechevarria@gmail.com)
// Copyright (C) 2013 Belen Masia (bmasia@unizar.es)
// Copyright (C) 2013 Fernando Navarro (fernandn@microsoft.com)
// Copyright (C) 2013 Diego Gutierrez (diegog@unizar.es)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
// of the Software, and to permit persons to whom the Software is furnished to
// do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software. As clarification, there
// is no requirement that the copyright notice and permission be included in
// binary distributions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <cstdint>

namespace Darkness {{

// AreaTex: RG8, {AREATEX_W}x{AREATEX_H}. Seven stacked {AREATEX_W}x80 blocks,
// one per subsample pattern; SMAA 1x reads only the first (offset 0), the rest
// exist for the multisampled variants (S2x/T2x/4x).
constexpr uint16_t kSmaaAreaTexWidth  = {AREATEX_W};
constexpr uint16_t kSmaaAreaTexHeight = {AREATEX_H};

// SearchTex: R8, {SEARCHTEX_W}x{SEARCHTEX_H}. Must be sampled with POINT
// filtering — it packs discrete run-length codes, and interpolating between
// two of them yields a distance that means nothing.
constexpr uint16_t kSmaaSearchTexWidth  = {SEARCHTEX_W};
constexpr uint16_t kSmaaSearchTexHeight = {SEARCHTEX_H};

""")
        emit_array(fh, "kSmaaAreaTex", area)
        emit_array(fh, "kSmaaSearchTex", search)
        fh.write("} // namespace Darkness\n")

    print(f"wrote {out_path}: AreaTex {len(area)} bytes, "
          f"SearchTex {len(search)} bytes")


if __name__ == "__main__":
    main()
