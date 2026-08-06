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
"""Lumel census — how coarse is the shipped lightmap bake, and what would a
higher-resolution re-bake cost?

Produced the measurements PLAN.HIGH_RES_SHADOWS.md is built on. Reads the
WR / WRRGB chunk straight out of a mission file; needs no build and no engine.

    python3 analysis/lumel_census.py density    <mission.mis> [...]
    python3 analysis/lumel_census.py validate   <mission.mis> [...]
    python3 analysis/lumel_census.py lights     <mission.mis> [...]
    python3 analysis/lumel_census.py anim       <mission.mis> [...]
    python3 analysis/lumel_census.py atlas      <mission.mis> [...]
    python3 analysis/lumel_census.py shadowload <mission.mis> [...]

  density     lumel counts and the world-space size of one lumel
  validate    cross-checks that derivation against each polygon's own extent
  lights      static light table: count, spot/omni, radius bounds, work units
  anim        how much of the light table is animated (must stay live)
  atlas       packed-atlas footprint under each density policy
  shadowload  distinct lights in range of a camera — the shadow-map demand a
              fully dynamic renderer would have to meet

Lumel size derivation: the renderer builds lightmap UVs as lmU = 4*projU + shift
where projU counts multiples of |axisU| (DarknessRendererExtended.h), so one
lumel spans |axisU| / 4 world units. `validate` is what confirms it: the stored
lx/ly must always cover the polygon's own extent in that space.
"""
import math
import os
import struct
import sys


# ── Chunk-file access ────────────────────────────────────────────────────────

def read_toc(data):
    """Chunk name -> (payload offset, length). Payload starts after the 24-byte
    per-chunk header, matching DarkFileGroup's item.offset + sizeof(header)."""
    inv = struct.unpack_from('<I', data, 0)[0]
    count = struct.unpack_from('<I', data, inv)[0]
    entries, off = {}, inv + 4
    for _ in range(count):
        name = data[off:off + 12].split(b'\0')[0].decode('ascii', 'replace')
        offset, length = struct.unpack_from('<II', data, off + 12)
        entries[name] = (offset + 24, length)
        off += 20
    return entries


class Cell:
    """One parsed WR cell — only the fields this census needs."""
    __slots__ = ('verts', 'poly_idx', 'axes', 'infos', 'num_lights', 'anim_map',
                 'center', 'light_set')


def parse_wr(path, want_geometry=False):
    """-> (cells, light_table, lightSize). light_table is None for grayscale WR.

    want_geometry=False skips vertex/index retention, which is most of the cost.
    """
    data = open(path, 'rb').read()
    toc = read_toc(data)
    if 'WREXT' in toc and 'WRRGB' not in toc and 'WR' not in toc:
        raise NotImplementedError('WREXT worldrep not supported (see WR-1)')
    name = 'WRRGB' if 'WRRGB' in toc else 'WR'
    if name not in toc:
        raise ValueError('no worldrep chunk')
    light_size = 2 if name == 'WRRGB' else 1

    p = toc[name][0] + 4                       # skip WRHeader.unk
    ncell = struct.unpack_from('<I', data, p)[0]
    p += 4
    cells = []
    for _ in range(ncell):
        nv, npoly, ntex = data[p], data[p + 1], data[p + 2]
        nplane = data[p + 4]
        p += 7 + 4 + 2                          # + nxn + polymapSize
        nanim = data[p]
        p += 2                                  # + flowGroup
        c = Cell()
        c.center = struct.unpack_from('<fff', data, p)
        p += 16                                 # center + radius

        if want_geometry:
            c.verts = [struct.unpack_from('<fff', data, p + i * 12)
                       for i in range(nv)]
        else:
            c.verts = None
        p += nv * 12

        counts = []
        for _ in range(npoly):
            counts.append(data[p + 1])
            p += 8

        c.axes = []
        for _ in range(ntex):
            au = struct.unpack_from('<fff', data, p)
            av = struct.unpack_from('<fff', data, p + 12)
            origin_vertex = data[p + 29]
            c.axes.append((au, av, origin_vertex))
            p += 48
        p += 4                                  # numIndices
        if want_geometry:
            c.poly_idx = []
            for n in counts:
                c.poly_idx.append(list(data[p:p + n]))
                p += n
        else:
            c.poly_idx = None
            p += sum(counts)
        p += nplane * 16
        c.anim_map = [struct.unpack_from('<h', data, p + i * 2)[0]
                      for i in range(nanim)]
        p += nanim * 2

        c.infos = []
        for _ in range(ntex):
            lx = struct.unpack_from('<H', data, p + 4)[0]
            ly = data[p + 6]
            animflags = struct.unpack_from('<I', data, p + 16)[0]
            c.infos.append((lx, ly, animflags))
            p += 20
        for (lx, ly, animflags) in c.infos:
            p += lx * ly * light_size * (1 + bin(animflags).count('1'))

        nli = struct.unpack_from('<I', data, p)[0]
        p += 4
        # Element 0 is the count; [1..count] index the static light table.
        c.num_lights = struct.unpack_from('<H', data, p)[0] if nli else 0
        c.light_set = frozenset(
            struct.unpack_from('<H', data, p + i * 2)[0]
            for i in range(1, min(c.num_lights + 1, nli)))
        p += nli * 2
        cells.append(c)

    # BSP (skipped), then the static light table.
    nx = struct.unpack_from('<i', data, p)[0]
    p += 4 + nx * 16
    nb = struct.unpack_from('<i', data, p)[0]
    p += 4 + nb * 20
    lights = None
    if light_size == 2:
        num_light = struct.unpack_from('<i', data, p)[0]
        p += 8                                  # + num_dyn
        lights = []
        for i in range(min(num_light, 768)):
            f = struct.unpack_from('<12f', data, p + i * 48)
            lights.append(dict(loc=f[0:3], dir=f[3:6], bright=f[6:9],
                               inner=f[9], outer=f[10], radius=f[11]))
    return cells, lights, light_size


# ── Helpers ──────────────────────────────────────────────────────────────────

def vlen(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def pctile(sorted_vals, q):
    if not sorted_vals:
        return 0
    return sorted_vals[min(len(sorted_vals) - 1, int(q * len(sorted_vals)))]


def atlas_side(area, slack=1.18):
    """Smallest power-of-two square atlas that could hold `area` texels.
    The slack matches what the shipped BSP packer actually achieves."""
    s = 64
    while s * s < area * slack and s < 131072:
        s *= 2
    return s


def lumel_sizes(cells):
    """-> list of (lx, ly, u_size, v_size) per lightmapped polygon."""
    out = []
    for c in cells:
        for i, (lx, ly, _af) in enumerate(c.infos):
            if lx <= 0 or ly <= 0:
                continue
            au, av, _ov = c.axes[i]
            lu, lv = vlen(au), vlen(av)
            if lu > 1e-6 and lv > 1e-6:
                out.append((lx, ly, lu / 4.0, lv / 4.0))
    return out


# ── Commands ─────────────────────────────────────────────────────────────────

def cmd_density(paths):
    print(f"{'mission':<12} {'cells':>6} {'lm polys':>9} {'lumels':>10} "
          f"{'anim lumels':>12} {'1x atlas':>9}   lumel world size p5/p50/p95")
    total = 0
    for path in paths:
        cells, _lights, ls = parse_wr(path)
        polys = lumel_sizes(cells)
        lumels = sum(lx * ly for lx, ly, _, _ in polys)
        anim = sum(lx * ly * bin(af).count('1')
                   for c in cells for (lx, ly, af) in c.infos)
        area = sum((lx + 4) * (ly + 4) for lx, ly, _, _ in polys)
        sizes = sorted([s for _, _, s, _ in polys] + [s for _, _, _, s in polys])
        total += lumels
        print(f"{os.path.basename(path):<12} {len(cells):>6} {len(polys):>9} "
              f"{lumels:>10,} {anim:>12,} {atlas_side(area):>6}^2   "
              f"{pctile(sizes, .05):.2f} / {pctile(sizes, .50):.2f} / "
              f"{pctile(sizes, .95):.2f}")
    print(f"\ntotal lumels: {total:,}")


def cmd_validate(paths):
    """Does the stored lx/ly really cover the polygon's extent in lumel space?

    A negative margin anywhere would mean the |axisU|/4 derivation is wrong.
    """
    print("margin = stored lx - measured extent, in lumels. Negative would "
          "falsify the derivation.")
    print(f"{'mission':<12} {'u margin p5/p50/p95':>26} "
          f"{'v margin p5/p50/p95':>26} {'negative':>9}")
    for path in paths:
        cells, _lights, _ls = parse_wr(path, want_geometry=True)
        mu, mv = [], []
        for c in cells:
            for i, (lx, ly, _af) in enumerate(c.infos):
                if lx <= 0 or ly <= 0:
                    continue
                au, av, ov = c.axes[i]
                ids = c.poly_idx[i]
                if ov >= len(ids) or ids[ov] >= len(c.verts):
                    continue
                org = c.verts[ids[ov]]
                m2u = sum(x * x for x in au)
                m2v = sum(x * x for x in av)
                dp = sum(a * b for a, b in zip(au, av))
                if m2u < 1e-9 or m2v < 1e-9:
                    continue
                us, vs = [], []
                for k in ids:
                    if k >= len(c.verts):
                        continue
                    d = tuple(c.verts[k][j] - org[j] for j in range(3))
                    pu = sum(a * b for a, b in zip(au, d))
                    pv = sum(a * b for a, b in zip(av, d))
                    if abs(dp) < 1e-6:
                        a, b = pu / m2u, pv / m2v
                    else:
                        corr = 1.0 / (m2u * m2v - dp * dp)
                        a = pu * corr * m2v - pv * corr * dp
                        b = pv * corr * m2u - pu * corr * dp
                    us.append(4.0 * a)
                    vs.append(4.0 * b)
                if not us:
                    continue
                mu.append(lx - (max(us) - min(us)))
                mv.append(ly - (max(vs) - min(vs)))
        mu.sort()
        mv.sort()
        neg = sum(1 for x in mu + mv if x < -0.01)
        print(f"{os.path.basename(path):<12} "
              f"{pctile(mu,.05):8.2f}{pctile(mu,.5):9.2f}{pctile(mu,.95):9.2f} "
              f"{pctile(mv,.05):8.2f}{pctile(mv,.5):9.2f}{pctile(mv,.95):9.2f} "
              f"{100.0*neg/max(1,len(mu)+len(mv)):8.1f}%")


def cmd_lights(paths):
    """Static light table, and the bake work unit: lumel-light pairs.

    Most lights carry radius 0 — no distance cutoff — so their reach is defined
    by the per-cell light_indices list, not by geometry. See the plan §1f.
    """
    print(f"{'mission':<12} {'lights':>7} {'spot':>5} {'unbounded':>10} "
          f"{'lights/cell p50/p95/max':>24} {'lumel-light pairs':>18}")
    grand = 0
    for path in paths:
        cells, lights, _ls = parse_wr(path)
        if lights is None:
            print(f"{os.path.basename(path):<12} grayscale WR — no light table")
            continue
        spots = sum(1 for lgt in lights if lgt['inner'] != -1.0)
        unbounded = sum(1 for lgt in lights if lgt['radius'] <= 0.0)
        per_cell = sorted(c.num_lights for c in cells)
        pairs = sum(sum(lx * ly for (lx, ly, _) in c.infos) * c.num_lights
                    for c in cells)
        grand += pairs
        print(f"{os.path.basename(path):<12} {len(lights):>7} {spots:>5} "
              f"{unbounded:>10} "
              f"{pctile(per_cell,.5):>8}{pctile(per_cell,.95):>8}"
              f"{per_cell[-1] if per_cell else 0:>8} {pairs:>18,}")
    print(f"\ntotal lumel-light pairs at 1x: {grand:,}")
    for k in (2, 4):
        print(f"  at {k}x linear density: {grand*k*k:,}")


def cmd_anim(paths):
    """Split the light table into the part a static bake can own forever and
    the part that has to stay live."""
    print(f"{'mission':<12} {'lights':>7} {'animated':>9} {'polys touched':>21} "
          f"{'lumels in them':>22}")
    for path in paths:
        cells, lights, _ls = parse_wr(path)
        # A polygon with any animflags bit set is touched by an animated light.
        touched = sum(1 for c in cells for (lx, ly, af) in c.infos
                      if lx and ly and af)
        lm_polys = sum(1 for c in cells for (lx, ly, _af) in c.infos
                       if lx and ly)
        lum_touched = sum(lx * ly for c in cells for (lx, ly, af) in c.infos
                          if lx and ly and af)
        lum_all = sum(lx * ly for c in cells for (lx, ly, _af) in c.infos
                      if lx and ly)
        # The animated set is the distinct light numbers named by any cell's
        # anim_map. Everything else in the light table never changes and can be
        # owned by a static bake forever.
        animated = set()
        for c in cells:
            animated.update(c.anim_map)
        n = len(lights) if lights else 0
        print(f"{os.path.basename(path):<12} {n:>7} "
              f"{len(animated):>5} ({100.0*len(animated)/max(1,n):4.1f}%) "
              f"{touched:>9,}/{lm_polys:<9,} ({100.0*touched/max(1,lm_polys):4.1f}%) "
              f"{lum_touched:>10,}/{lum_all:<10,} "
              f"({100.0*lum_touched/max(1,lum_all):4.1f}%)")


POLICIES = [
    ("shipped (1x)", lambda s: 1),
    ("flat 2x", lambda s: 2),
    ("flat 4x", lambda s: 4),
    ("flat 8x", lambda s: 8),
    ("target 0.50u, cap 8x", lambda s: _pot(s / 0.50, 8)),
    ("target 0.25u, cap 8x", lambda s: _pot(s / 0.25, 8)),
]


def _pot(x, kmax):
    k = 1
    while k < kmax and k < x:
        k *= 2
    return k


def cmd_atlas(paths):
    """Packed-atlas footprint under each density policy.

    Measured result worth not rediscovering: a per-polygon "target lumel size"
    policy costs MORE than a flat multiplier at the same median, because it
    pushes the already-coarse polygons hardest. Use the flat multiplier.
    """
    for path in paths:
        cells, _lights, _ls = parse_wr(path)
        polys = lumel_sizes(cells)
        print(f"\n=== {os.path.basename(path)}: {len(polys):,} lightmapped "
              f"polygons, {sum(lx*ly for lx,ly,_,_ in polys):,} lumels at 1x ===")
        print(f"{'policy':<24} {'lumels':>14} {'packed texels':>15} "
              f"{'atlas':>9} {'RGB8 MB':>9} {'median lumel u':>15}")
        for label, fn in POLICIES:
            area = lum = 0
            sizes = []
            for lx, ly, su, sv in polys:
                ku, kv = fn(su), fn(sv)
                nx, ny = lx * ku, ly * kv
                area += (nx + 4) * (ny + 4)
                lum += nx * ny
                sizes.append(su / ku)
            sizes.sort()
            s = atlas_side(area)
            print(f"{label:<24} {lum:>14,} {area:>15,} {s:>6}^2 "
                  f"{s*s*3/1048576:>8.1f} {sizes[len(sizes)//2]:>15.3f}")


def cmd_shadowload(paths, radii=(40.0, 130.0, 165.0), samples=250, seed=1234):
    """Distinct lights within R of a sampled camera cell.

    Per-cell counts understate the shadow-map demand of a fully dynamic
    renderer, because a frame draws many cells and the pool must cover the
    union. Radii are Thief world units (roughly a foot each); for scale, HPL2
    drops shadows entirely past 40 m (~130u) and Godot's distance_fade_shadow
    defaults to 50 m (~165u). Those two engines budget 11 and 88 simultaneous
    shadow-casting lights respectively — this is what they would be asked for.

    Upper bound: the union includes lights that portal culling would drop.
    """
    import random
    header = "  ".join(f"{'R=' + str(int(r)) + 'u  p50/p95/max':>23}"
                       for r in radii)
    print(f"{'mission':<12} {header}")
    for path in paths:
        cells, _lights, _ls = parse_wr(path)
        rng = random.Random(seed)
        idx = list(range(len(cells)))
        rng.shuffle(idx)
        idx = idx[:samples]
        cols = []
        for radius in radii:
            r2 = radius * radius
            counts = []
            for i in idx:
                cx, cy, cz = cells[i].center
                union = set()
                for other in cells:
                    dx = other.center[0] - cx
                    dy = other.center[1] - cy
                    dz = other.center[2] - cz
                    if dx * dx + dy * dy + dz * dz <= r2:
                        union |= other.light_set
                counts.append(len(union))
            counts.sort()
            cols.append(f"{pctile(counts,.5):>8}{pctile(counts,.95):>7}"
                        f"{counts[-1]:>8}")
        print(f"{os.path.basename(path):<12} " + "  ".join(cols))


COMMANDS = {'density': cmd_density, 'validate': cmd_validate,
            'lights': cmd_lights, 'anim': cmd_anim, 'atlas': cmd_atlas,
            'shadowload': cmd_shadowload}


if __name__ == '__main__':
    if len(sys.argv) < 3 or sys.argv[1] not in COMMANDS:
        print(__doc__)
        sys.exit(1)
    ok = []
    for a in sys.argv[2:]:
        try:
            parse_wr(a)
            ok.append(a)
        except Exception as e:
            print(f"{os.path.basename(a)}: skipped ({e})", file=sys.stderr)
    if not ok:
        sys.exit(1)
    COMMANDS[sys.argv[1]](ok)
