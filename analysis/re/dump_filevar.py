#!/usr/bin/env python3
"""Dump a Dark Engine file-var descriptor: field names, types and struct offsets.

The engine registers externally-tunable structs through `cFileVar<Payload,
&gDescriptor>`. Each registration is a header followed by an array of field
descriptors, and BOTH carry literal strings — so the layout of the payload
struct is recoverable from the binary without decompiling anything.

Observed layout (NewDark 1.28):

    header, 0x28 bytes
        +0x00  char  var_name[12]     inline, e.g. "AIHearStat"
        +0x0C  char* label            e.g. "AIHearingStats"
        +0x10  char* payload_type     e.g. "sHearingStats"
        +0x14  int   flags[5]
    field records, stride 0x40, immediately after the header
        +0x00  char  label[0x20]      inline, e.g. "VeryLow: DistMul"
        +0x20  int   ui_range         editor slider bound (0 = unset)
        +0x24  int   type             4 = float (others not yet mapped)
        +0x28  int   struct_offset    byte offset into the payload struct
        +0x2C  int   ...              zero in every record seen so far

Usage:
    python3 analysis/re/dump_filevar.py <binary> <payload-type-name> [max_fields]

Example:
    python3 analysis/re/dump_filevar.py .../Thief2.exe sHearingStats

See .claude/PLAN.BINARY_RE_PIPELINE.md and analysis/re/FINDINGS.md.
"""
import importlib.util
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    "find_refs", os.path.join(HERE, "find_refs.py"))
_fr = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_fr)

HEADER_SIZE = 0x28
FIELD_STRIDE = 0x40
TYPE_NAMES = {4: "float"}


def u32(img, va):
    raw = img.read(va, 4)
    return None if raw is None else struct.unpack("<I", raw)[0]


def printable(s):
    return bool(s) and all(0x20 <= ord(c) < 0x7F for c in s)


def main():
    if len(sys.argv) < 3:
        sys.stderr.write(__doc__ or "")
        return 2
    binary, typename = sys.argv[1], sys.argv[2]
    max_fields = int(sys.argv[3], 0) if len(sys.argv) > 3 else 64

    img = _fr.Image(binary)

    # Locate the payload type-name string, then the .data pointer to it. That
    # pointer sits at header+0x10.
    cands = []
    for start, end, data, sect in img.spans:
        needle = typename.encode() + b"\0"
        pos = data.find(needle)
        while pos >= 0:
            cands.append(start + pos)
            pos = data.find(needle, pos + 1)
    if not cands:
        print(f"[RE] type-name string {typename!r} not found")
        return 1

    # PRIMARY ANCHOR: the trailer, keyed off the inline .data copy of the type
    # name. It carries struct_size, field_count AND a pointer straight to the
    # field array — so the header is not needed to find the fields at all.
    # This is what makes cFileVar2 work: cFileVar2 registers no .data pointer to
    # its type-name string, so a header-first search fails on it, but every
    # file-var still has a trailer.
    blocks = []
    for s_va in cands:
        if img.section_of(s_va) != ".data":
            continue
        size = u32(img, s_va + 0x20)
        count = u32(img, s_va + 0x28)
        fld = u32(img, s_va + 0x2C)
        if (size is None or count is None or fld is None
                or not (0 < count <= 256) or size > 0x10000
                or img.section_of(fld) is None):
            continue
        blocks.append((s_va, size, count, fld))

    if not blocks:
        print(f"[RE] found {typename!r} at "
              f"{', '.join(hex(c) for c in cands)} but no usable trailer "
              f"(expected struct_size/count/field-ptr at +0x20/+0x28/+0x2C)")
        return 1

    for s_va, size, count, first_field in blocks:
        # The header is optional metadata (var name + label). Locate it via a
        # .data/.rdata pointer to the type name if one exists; cFileVar2 has none.
        # The header's +0x10 points at the .rdata copy of the type name, not the
        # inline .data copy the trailer is keyed off — so search pointers to
        # EVERY copy, and accept the one whose -0x10 has a plausible inline name.
        hdr = None
        for c in cands:
            for ptr_va, psect in img.find_dword(c):
                if psect not in (".data", ".rdata"):
                    continue
                cand_hdr = ptr_va - 0x10
                nm = img.cstring(cand_hdr, 12)
                if nm and printable(nm) and len(nm) >= 3:
                    hdr = cand_hdr
                    break
            if hdr is not None:
                break
        print("=" * 74)
        print(f"payload type : {typename}  (inline .data copy @ 0x{s_va:X})")
        if hdr is not None:
            print(f"header       : 0x{hdr:08X}  var name "
                  f"{img.cstring(hdr) or '<none>'!r}, label "
                  f"{img.cstring(u32(img, hdr + 0x0C) or 0) or '<none>'!r}")
        else:
            print("header       : none (cFileVar2 — no .data pointer to the "
                  "type name; trailer used instead)")
        print(f"struct_size  : {size} bytes")
        print(f"field_count  : {count}")
        print(f"field_array  : 0x{first_field:08X}")
        print()
        print(f"  {'#':>2}  {'addr':<10} {'field label':<32} "
              f"{'ui':>5} {'type':<7} {'off':>4}")
        print(f"  {'-'*2}  {'-'*10} {'-'*32} {'-'*5} {'-'*7} {'-'*4}")
        # The trailer gives an exact field_count, so walk exactly that many
        # records rather than guessing a terminator. No heuristic stop needed.
        va = first_field
        n = 0
        for _ in range(min(count, max_fields)):
            lbl = img.cstring(va, 0x20)
            ui = u32(img, va + 0x20)
            sz = u32(img, va + 0x24)
            off = u32(img, va + 0x28)
            if lbl is None or sz is None or off is None:
                print(f"  {n:>2}  0x{va:08X} <unreadable record>")
                break
            szname = TYPE_NAMES.get(sz, f"{sz}B")
            print(f"  {n:>2}  0x{va:08X} {lbl.strip():<32} "
                  f"{ui:>5} {szname:<7} {off:>4}")
            n += 1
            va += FIELD_STRIDE
        # --- defaults ------------------------------------------------------
        # The defaults template, when present, sits at +0x30 from the inline
        # .data type name (i.e. immediately after the trailer).
        defaults = s_va + 0x30

        # Not every registration block carries an inline defaults template.
        # Where it does not, +0x30 lands on the NEXT file-var header, whose
        # first bytes are an inline variable name — printable ASCII. Detect
        # that and say so, rather than printing the name's bytes as floats.
        probe_txt = img.cstring(defaults, 0x20)
        if probe_txt and printable(probe_txt) and len(probe_txt) >= 4:
            print(f"  NO inline defaults template — 0x{defaults:08X} holds "
                  f"{probe_txt!r}, which is the next file-var header.")
            print(f"  (defaults for this struct are zero-initialised or set in "
                  f"code; find them by xref to the struct instance)\n")
            continue
        # Plausibility check. A real defaults template decodes to sane values.
        # When +0x30 lands on unrelated data that is not ASCII (so the string
        # guard above misses it), the giveaway is fields whose float reading is
        # denormal-tiny while the int reading is a small counter — the signature
        # of an index/count table sitting where the template was expected.
        nonzero = normal_floats = 0
        for i in range(n):
            off_i = u32(img, first_field + i * FIELD_STRIDE + 0x28) or 0
            raw_i = img.read(defaults + off_i, 4)
            if raw_i is None:
                continue
            fv_i = struct.unpack("<f", raw_i)[0]
            iv_i = struct.unpack("<i", raw_i)[0]
            if iv_i != 0:
                nonzero += 1
                if 1e-6 <= abs(fv_i) <= 1e9:
                    normal_floats += 1
        # A genuine template has at least one field in normal float range. A
        # region of small counters/indices has none — every nonzero value is a
        # denormal when read as float. (sHearingStats passes: its distMul fields
        # are normal floats even though its dbAdd fields are ints.)
        if nonzero and normal_floats == 0:
            print(f"  DEFAULTS UNRELIABLE — {nonzero} nonzero field(s) at "
                  f"0x{defaults:08X}, none in normal float range.")
            print(f"  That signature is a counter/index table, not a defaults "
                  f"template; this file-var stores defaults elsewhere. "
                  f"NOT reporting values.\n")
            continue

        print(f"  defaults template @ 0x{defaults:08X}")
        print(f"    {'field':<32} {'off':>4}  {'as float':>14}  {'as int':>12}")
        print(f"    {'-'*32} {'-'*4}  {'-'*14}  {'-'*12}")
        for i in range(n):
            rec = first_field + i * FIELD_STRIDE
            lbl = (img.cstring(rec, 0x20) or "").strip()
            off = u32(img, rec + 0x28) or 0
            raw = img.read(defaults + off, 4)
            if raw is None:
                continue
            fv = struct.unpack("<f", raw)[0]
            iv = struct.unpack("<i", raw)[0]
            print(f"    {lbl:<32} {off:>4}  {fv:>14.6g}  {iv:>12d}")
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
