#!/usr/bin/env python3
"""SQL front-end over an exported Ghidra index.

Usage:
    python3 analysis/re/query.py [--db NAME] "<SQL>"
    python3 analysis/re/query.py --schema
    python3 analysis/re/query.py --xref 0x9EA660      # who references this VA
    python3 analysis/re/query.py --str Sound          # strings matching, + who uses them
    python3 analysis/re/query.py --fn Light           # functions whose name matches

Addresses may be written as 0x… anywhere in the SQL; they are passed through to
SQLite, which understands hex literals. Results print addresses as hex.

Default db: ../re_projects/Thief2_ND128.db

See .claude/PLAN.BINARY_RE_PIPELINE.md.
"""
import os
import sqlite3
import sys

PROJDIR = "/Users/jdv/code/darkness/re_projects"
DEFAULT_DB = "Thief2_ND128"

ADDR_COLS = {"addr", "from_addr", "to_addr", "start", "end", "from_func"}


def connect(name):
    path = name if os.path.sep in name else os.path.join(PROJDIR, name + ".db")
    if not os.path.exists(path):
        sys.stderr.write(f"[RE] no index at {path}\n"
                         f"[RE] build it: python3 analysis/re/export_index.py "
                         f"<project> <binary>\n")
        sys.exit(1)
    return sqlite3.connect(path)


def show(cur, limit=200):
    cols = [d[0] for d in cur.description] if cur.description else []
    rows = cur.fetchmany(limit)
    if not cols:
        return
    widths = [len(c) for c in cols]
    out = []
    for r in rows:
        cells = []
        for c, v in zip(cols, r):
            if v is not None and c in ADDR_COLS and isinstance(v, int):
                v = f"0x{v:X}"
            cells.append("" if v is None else str(v))
        out.append(cells)
        widths = [max(w, len(x)) for w, x in zip(widths, cells)]
    widths = [min(w, 70) for w in widths]
    print("  ".join(c.ljust(w) for c, w in zip(cols, widths)))
    print("  ".join("-" * w for w in widths))
    for cells in out:
        print("  ".join(x[:70].ljust(w) for x, w in zip(cells, widths)))
    extra = cur.fetchone()
    print(f"\n({len(out)} row(s){', truncated' if extra else ''})")


def main():
    args = sys.argv[1:]
    dbname = DEFAULT_DB
    if args and args[0] == "--db":
        dbname = args[1]
        args = args[2:]
    if not args:
        sys.stderr.write(__doc__ or "")
        return 2

    db = connect(dbname)

    if args[0] == "--schema":
        show(db.execute(
            "SELECT name, sql FROM sqlite_master WHERE type='table'"), 50)
        print()
        show(db.execute("SELECT key, value FROM meta"), 50)
        return 0

    if args[0] == "--xref":
        target = int(args[1], 0)
        print(f"references TO 0x{target:X}:")
        show(db.execute(
            "SELECT x.from_addr, x.type, x.is_call, f.name AS in_function "
            "FROM xrefs x LEFT JOIN functions f ON f.addr = x.from_func "
            "WHERE x.to_addr = ? ORDER BY x.from_addr", (target,)))
        return 0

    if args[0] == "--str":
        pat = f"%{args[1]}%"
        print("matching strings and the functions that reference them:")
        show(db.execute(
            "SELECT s.addr, s.value, f.name AS used_by, x.from_addr "
            "FROM strings s "
            "LEFT JOIN xrefs x ON x.to_addr = s.addr "
            "LEFT JOIN functions f ON f.addr = x.from_func "
            "WHERE s.value LIKE ? ORDER BY s.addr", (pat,)))
        return 0

    if args[0] == "--fn":
        pat = f"%{args[1]}%"
        show(db.execute(
            "SELECT addr, name, size, param_count, signature FROM functions "
            "WHERE name LIKE ? ORDER BY name", (pat,)))
        return 0

    show(db.execute(args[0]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
