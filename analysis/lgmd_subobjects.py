#!/usr/bin/env python3
###############################################################################
#
#    This file is part of the darkness project
#    Copyright (C) 2024-2026 darkness contributors
#
#    This program is free software; you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation; either version 2 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program; if not, write to the Free Software
#    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#
###############################################################################
"""LGMD sub-object (model joint) survey over an obj.crf archive.

Prints, per model with more than one sub-object: the child/next tree with each
part's depth, `movement`, joint slot, authored range, `rot` basis, axle, and —
critically — the part's LOCAL vertex extents.

    python3 analysis/lgmd_subobjects.py                 # summary only
    python3 analysis/lgmd_subobjects.py addoor01 chest  # detail on matches
    python3 analysis/lgmd_subobjects.py --crf /path/to/obj.crf

ALWAYS read the extents, not the names. `@s00bb` on a door reads like a leaf
and is a knob; that mistake produced a whole wrong revision of the door
analysis. The extents, the axle placement and the authored range together are
what identify a part.

The `rot` triples printed are the images of the part's local X / Y / Z axes in
its parent's space (the matrix is stored column-major). The joint moves the
part about — or, for a slide, along — its local **X**; see NOTES.PROJECT.md,
"LGMD sub-object joints".

Layout, after the 4-byte `LGMD` magic + 4-byte version: `num_verts` +0x3E,
`num_objs` +0x45, `offset_objs` +0x46, `offset_verts` +0x56. Sub-object stride
0x5D, with `point_start`/`sub_num_points` at +0x4D.
"""
import struct
import sys
import zipfile

DEFAULT_CRF = '/Volumes/THIEF2_INSTALL_C/THIEF2/RES/obj.crf'

H_NUM_VERTS = 0x3E
H_NUM_OBJS = 0x45
H_OFF_OBJS = 0x46
H_OFF_VERTS = 0x56
STRIDE = 0x5D

MOVEMENT = {0: 'static', 1: 'rotate', 2: 'slide'}


def read_subobjects(b):
    """Return (parts, vertex_table), or (None, None) if the record is short."""
    num = b[H_NUM_OBJS]
    off = struct.unpack_from('<I', b, H_OFF_OBJS)[0]
    if num < 1 or off + num * STRIDE > len(b):
        return None, None

    nv = struct.unpack_from('<H', b, H_NUM_VERTS)[0]
    ov = struct.unpack_from('<I', b, H_OFF_VERTS)[0]
    if ov + 12 * nv > len(b):
        return None, None
    verts = [struct.unpack_from('<3f', b, ov + 12 * i) for i in range(nv)]

    parts = []
    for i in range(num):
        s = b[off + i * STRIDE: off + (i + 1) * STRIDE]
        child, nxt = struct.unpack_from('<hh', s, 0x45)
        ps, pn = struct.unpack_from('<hh', s, 0x4D)
        parts.append(dict(
            i=i,
            name=s[0:8].split(b'\0')[0].decode('latin-1'),
            movement=s[8],
            joint=struct.unpack_from('<i', s, 9)[0],
            lo=struct.unpack_from('<f', s, 0x0D)[0],
            hi=struct.unpack_from('<f', s, 0x11)[0],
            rot=struct.unpack_from('<9f', s, 0x15),
            axle=struct.unpack_from('<3f', s, 0x39),
            child=child, next=nxt, point_start=ps, point_count=pn))
    return parts, verts


def depths(parts):
    """Depth of each part, walking the child/next tree from the root."""
    out = {}

    def walk(idx, depth, guard=0):
        while 0 <= idx < len(parts) and guard < 1000:
            if idx in out:
                return
            out[idx] = depth
            if parts[idx]['child'] >= 0:
                walk(parts[idx]['child'], depth + 1, guard + 1)
            idx = parts[idx]['next']
            guard += 1

    walk(0, 0)
    return out


def extents(part, verts):
    pts = verts[part['point_start']:part['point_start'] + part['point_count']]
    if not pts:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    lo = tuple(min(p[k] for p in pts) for k in range(3))
    hi = tuple(max(p[k] for p in pts) for k in range(3))
    return tuple(hi[k] - lo[k] for k in range(3)), lo, hi


def main():
    argv = sys.argv[1:]
    crf = DEFAULT_CRF
    if '--crf' in argv:
        k = argv.index('--crf')
        crf = argv[k + 1]
        del argv[k:k + 2]
    wanted = [a.lower() for a in argv]

    try:
        zf = zipfile.ZipFile(crf)
    except OSError as exc:
        sys.stderr.write("cannot open %s: %s\n" % (crf, exc))
        sys.stderr.write("mount the Thief 2 install ISO, or pass --crf PATH\n")
        return 1

    multi = 0
    depth_hist = {}
    move_hist = {}
    deep = []
    unordered = []
    max_joint = -1

    for name in sorted(n for n in zf.namelist() if n.lower().endswith('.bin')):
        b = zf.read(name)
        if len(b) < 0x7A or b[:4] != b'LGMD':
            continue
        parts, verts = read_subobjects(b)
        if parts is None:
            sys.stderr.write("skipped (truncated sub-object table): %s\n" % name)
            continue

        for p in parts:
            move_hist[p['movement']] = move_hist.get(p['movement'], 0) + 1
            if p['movement'] != 0:
                max_joint = max(max_joint, p['joint'])
                if p['lo'] > p['hi']:
                    unordered.append((name, p['name'], p['lo'], p['hi']))

        if len(parts) <= 1:
            continue
        multi += 1

        d = depths(parts)
        md = max(d.values()) if d else 0
        depth_hist[md] = depth_hist.get(md, 0) + 1
        if md > 1:
            deep.append((name, md, len(parts)))

        base = name.split('/')[-1].lower()
        if wanted and not any(w in base for w in wanted):
            continue

        print('%s  num_objs=%d maxdepth=%d' % (name, len(parts), md))
        for p in parts:
            ext, lo, hi = extents(p, verts)
            print('  [%d] %-9s depth=%d %-6s joint=%2d range=[%8.3f,%8.3f] '
                  'child=%d next=%d'
                  % (p['i'], p['name'], d.get(p['i'], -1),
                     MOVEMENT.get(p['movement'], '?%d' % p['movement']),
                     p['joint'], p['lo'], p['hi'], p['child'], p['next']))
            print('      axle=(%7.3f,%7.3f,%7.3f) extent=(%.2f,%.2f,%.2f) '
                  'lo=(%.2f,%.2f,%.2f) hi=(%.2f,%.2f,%.2f)'
                  % (p['axle'] + ext + lo + hi))
            print('      local X->(%6.3f %6.3f %6.3f)  Y->(%6.3f %6.3f %6.3f)  '
                  'Z->(%6.3f %6.3f %6.3f)' % p['rot'])
        print()

    print('multi-sub-object models : %d' % multi)
    print('movement histogram      : %s'
          % {MOVEMENT.get(k, k): v for k, v in sorted(move_hist.items())})
    print('max-depth histogram     : %s' % dict(sorted(depth_hist.items())))
    print('highest joint slot used : %d' % max_joint)
    print('ranges with min > max   : %d  %s'
          % (len(unordered), [u[:2] for u in unordered]))
    print('models deeper than 1    : %d' % len(deep))
    for name, md, count in deep:
        print('   %-32s depth=%d parts=%d' % (name, md, count))
    return 0


if __name__ == '__main__':
    sys.exit(main())
