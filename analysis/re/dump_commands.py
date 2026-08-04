#!/usr/bin/env python3
"""Dump the Dark Engine console/command table from a binary.

The engine registers console commands in a flat .data table of 24-byte records:

    +0x00  char* name        "ai_spew_zones"
    +0x04  int   (0)
    +0x08  void* handler     function address
    +0x0C  char* help        "spew all AI path zones"
    +0x10  int   flags       (6 observed on AI commands)
    +0x14  int   (0)

Every command carries a human-written help string, so the table is the single
best map of engine subsystems and their debug hooks — especially in DromEd.exe,
which registers far more than the game binary.

Usage:
    python3 analysis/re/dump_commands.py <exe> [substring-filter]

Example:
    python3 analysis/re/dump_commands.py .../DromEd.exe ai_

See .claude/PLAN.BINARY_RE_PIPELINE.md and analysis/re/FINDINGS.md.
"""
import importlib.util
import os
import re
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    "find_refs", os.path.join(HERE, "find_refs.py"))
_fr = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_fr)

STRIDE = 0x18
NAME_RE = re.compile(r"^[a-z_][a-z0-9_]{1,40}$")


def main():
    if len(sys.argv) < 2:
        sys.stderr.write(__doc__ or "")
        return 2
    img = _fr.Image(sys.argv[1])
    filt = sys.argv[2].lower() if len(sys.argv) > 2 else None

    rows = []
    for start, end, data, sect in img.spans:
        if sect != ".data":
            continue
        for off in range(0, len(data) - STRIDE, 4):
            name_p, zero1, fn, help_p = struct.unpack_from("<IIII", data, off)
            if zero1 != 0 or img.section_of(fn) != ".text":
                continue
            name = img.cstring(name_p, 48)
            helptxt = img.cstring(help_p, 120)
            if not name or not NAME_RE.match(name):
                continue
            if helptxt is None or not helptxt.isprintable():
                continue
            flags = struct.unpack_from("<I", data, off + 0x10)[0]
            rows.append((start + off, name, fn, helptxt, flags))

    # Dedup on name+handler; the scan steps 4 bytes so a record can match twice.
    seen, out = set(), []
    for va, name, fn, helptxt, flags in rows:
        key = (name, fn)
        if key in seen:
            continue
        seen.add(key)
        out.append((va, name, fn, helptxt, flags))

    if filt:
        out = [r for r in out if filt in r[1].lower() or filt in r[3].lower()]
    out.sort(key=lambda r: r[1])

    print(f"{len(out)} command(s)"
          f"{f' matching {filt!r}' if filt else ''} in "
          f"{os.path.basename(sys.argv[1])}\n")
    print(f"  {'command':<28} {'handler':<10} help")
    print(f"  {'-'*28} {'-'*10} {'-'*50}")
    for va, name, fn, helptxt, flags in out:
        print(f"  {name:<28} 0x{fn:08X} {helptxt}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
