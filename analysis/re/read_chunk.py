#!/usr/bin/env python3
"""Read a raw chunk out of a Dark Engine .gam / .mis / .sav database.

File-var structs are chunk-backed: the file-var's inline variable name IS the
chunk name (RENDPARAMS, AIHearStat, WATERBANKS, ...), and the chunk payload is
exactly the payload struct. So the SHIPPED values for every tunable struct can
be read straight out of DARK.GAM / a mission, which beats the binary's fallback
defaults — the binary template only applies when the chunk is absent.

Usage:
    python3 analysis/re/read_chunk.py <db> --list
    python3 analysis/re/read_chunk.py <db> <CHUNK> [--floats|--ints|--hex]

Example:
    python3 analysis/re/read_chunk.py DARK.GAM AIHearStat --floats

Layout (src/base/file/darkdb.h):
    header : u32 inv_offset; u32 zero; u32 one; u8 pad[256]; u32 0xEFBEADDE
    TOC    : u32 count, then count * { char name[12]; u32 offset; u32 length }
    chunk  : { char name[12]; u32 ver_hi; u32 ver_lo; u32 zero } then payload

See .claude/PLAN.BINARY_RE_PIPELINE.md and analysis/re/FINDINGS.md.
"""
import struct
import sys

HDR_SIZE = 4 + 4 + 4 + 256 + 4
CHUNK_HDR = 12 + 4 + 4 + 4


def load_toc(path):
    blob = open(path, "rb").read()
    inv_offset = struct.unpack_from("<I", blob, 0)[0]
    magic = struct.unpack_from("<I", blob, HDR_SIZE - 4)[0]
    if magic != 0xEFBEADDE:
        sys.stderr.write(f"[RE] bad magic 0x{magic:08X} — not a Dark database?\n")
    count = struct.unpack_from("<I", blob, inv_offset)[0]
    toc = {}
    off = inv_offset + 4
    for _ in range(count):
        name = blob[off:off + 12].split(b"\0")[0].decode("latin-1", "replace")
        c_off, c_len = struct.unpack_from("<II", blob, off + 12)
        toc[name] = (c_off, c_len)
        off += 20
    return blob, toc


def main():
    if len(sys.argv) < 3:
        sys.stderr.write(__doc__ or "")
        return 2
    path = sys.argv[1]
    blob, toc = load_toc(path)

    if sys.argv[2] == "--list":
        for name in sorted(toc):
            off, ln = toc[name]
            print(f"  {name:<16} offset=0x{off:08X} length={ln}")
        print(f"\n({len(toc)} chunks)")
        return 0

    name = sys.argv[2]
    mode = sys.argv[3] if len(sys.argv) > 3 else "--floats"
    if name not in toc:
        sys.stderr.write(f"[RE] no chunk {name!r}; try --list\n")
        return 1
    off, ln = toc[name]
    hdr_name = blob[off:off + 12].split(b"\0")[0].decode("latin-1", "replace")
    v_hi, v_lo = struct.unpack_from("<II", blob, off + 12)
    # The TOC length is the PAYLOAD size; the 24-byte chunk header sits before
    # it and is not counted.
    payload = blob[off + CHUNK_HDR: off + CHUNK_HDR + ln]

    print(f"chunk {hdr_name!r}  v{v_hi}.{v_lo}  payload={len(payload)} bytes")
    print()
    for i in range(0, len(payload) - 3, 4):
        raw = payload[i:i + 4]
        f = struct.unpack("<f", raw)[0]
        s = struct.unpack("<i", raw)[0]
        txt = "".join(chr(b) if 32 <= b < 127 else "." for b in raw)
        print(f"  +{i:<4d} 0x{struct.unpack('<I', raw)[0]:08X}  "
              f"float={f:<14.6g} int={s:<12d} {txt}")
    tail = len(payload) % 4
    if tail:
        print(f"  (+{len(payload) - tail} .. {len(payload)}: {payload[-tail:]!r})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
