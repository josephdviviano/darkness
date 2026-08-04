#!/usr/bin/env python3
"""Find 32-bit pointers to a target VA by scanning the raw PE image.

Ghidra only creates a reference where its analysis decided a location holds a
pointer. Static descriptor tables (arrays of {const char* name, type, offset})
are frequently left untyped, so `query.py --xref` reports nothing even though
the table plainly exists. This scans the mapped image bytes directly, which does
not depend on any analysis having happened.

Usage:
    python3 analysis/re/find_refs.py <binary> <VA> [<VA> ...]
    python3 analysis/re/find_refs.py <binary> --dump <VA> <count>   # hexdump words

Example:
    python3 analysis/re/find_refs.py .../Thief2.exe 0x76E8F0

See .claude/PLAN.BINARY_RE_PIPELINE.md.
"""
import struct
import sys

try:
    import pefile
except ImportError:
    sys.stderr.write("[RE] pefile missing — pip install pefile\n")
    sys.exit(1)


class Image:
    """Flat VA-addressable view of a PE's mapped sections."""

    def __init__(self, path):
        self.pe = pefile.PE(path, fast_load=True)
        self.base = self.pe.OPTIONAL_HEADER.ImageBase
        self.spans = []  # (va_start, va_end, bytes)
        for s in self.pe.sections:
            data = s.get_data()
            va = self.base + s.VirtualAddress
            name = s.Name.rstrip(b"\0").decode("latin-1", "replace")
            self.spans.append((va, va + len(data), data, name))

    def read(self, va, n):
        for start, end, data, _ in self.spans:
            if start <= va and va + n <= end:
                off = va - start
                return data[off:off + n]
        return None

    def cstring(self, va, limit=200):
        for start, end, data, _ in self.spans:
            if start <= va < end:
                off = va - start
                nul = data.find(b"\0", off, min(off + limit, len(data)))
                if nul < 0:
                    return None
                return data[off:nul].decode("latin-1", "replace")
        return None

    def section_of(self, va):
        for start, end, _, name in self.spans:
            if start <= va < end:
                return name
        return None

    def find_dword(self, value):
        needle = struct.pack("<I", value)
        hits = []
        for start, end, data, name in self.spans:
            pos = data.find(needle)
            while pos >= 0:
                hits.append((start + pos, name))
                pos = data.find(needle, pos + 1)
        return hits


def main():
    if len(sys.argv) < 3:
        sys.stderr.write(__doc__ or "")
        return 2
    img = Image(sys.argv[1])

    if sys.argv[2] == "--strings":
        # Printable-ASCII runs in a VA range, straight from the image. Needed
        # because descriptor tables embed inline char arrays that Ghidra often
        # never defines as data, so they are absent from the SQLite strings
        # table.
        lo = int(sys.argv[3], 0)
        hi = int(sys.argv[4], 0)
        minlen = int(sys.argv[5], 0) if len(sys.argv) > 5 else 3
        print(f"printable runs in 0x{lo:X}..0x{hi:X} "
              f"(section {img.section_of(lo)}):")
        cur, start = [], None
        va = lo
        while va < hi:
            b = img.read(va, 1)
            if b is None:
                va += 1
                continue
            c = b[0]
            if 0x20 <= c < 0x7F:
                if start is None:
                    start = va
                cur.append(chr(c))
            else:
                if start is not None and len(cur) >= minlen:
                    print(f"  0x{start:08X}  {''.join(cur)}")
                cur, start = [], None
            va += 1
        if start is not None and len(cur) >= minlen:
            print(f"  0x{start:08X}  {''.join(cur)}")
        return 0

    if sys.argv[2] == "--dump":
        va = int(sys.argv[3], 0)
        count = int(sys.argv[4], 0) if len(sys.argv) > 4 else 32
        print(f"dump {count} dwords at 0x{va:X} "
              f"(section {img.section_of(va)}):")
        for i in range(count):
            a = va + i * 4
            raw = img.read(a, 4)
            if raw is None:
                print(f"  0x{a:08X}  <unmapped>")
                continue
            w = struct.unpack("<I", raw)[0]
            s = img.cstring(w) if 0x400000 < w < 0x1000000 else None
            note = ""
            if s is not None and s.isprintable() and len(s) > 1:
                note = f'   -> "{s}"'
            elif w < 0x10000:
                note = f"   ({w})"
            print(f"  0x{a:08X}  0x{w:08X}{note}")
        return 0

    for arg in sys.argv[2:]:
        target = int(arg, 0)
        s = img.cstring(target)
        label = f' "{s}"' if s else ""
        hits = img.find_dword(target)
        print(f"pointers to 0x{target:X}{label}: {len(hits)} hit(s)")
        for va, sect in hits:
            print(f"    0x{va:08X}  [{sect}]")
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
