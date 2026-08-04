#!/usr/bin/env python3
"""Export an analyzed Ghidra program to a queryable SQLite index.

This is the load-bearing piece of the pipeline. Clicking through a GUI does not
scale to a 5 MB binary; being able to ask "which functions reference address X"
as SQL across tens of thousands of functions and references is what makes the
analysis tractable.

Usage:
    python3 analysis/re/export_index.py <project-name> <binary-path> [out.db]

Default output: ../re_projects/<project-name>.db

Then query with analysis/re/query.py, e.g.:
    python3 analysis/re/query.py "SELECT name, addr FROM functions
                                  WHERE name LIKE '%Sound%' LIMIT 20"

See .claude/PLAN.BINARY_RE_PIPELINE.md.
"""
import os
import sqlite3
import sys

PROJDIR = "/Users/jdv/code/darkness/re_projects"
JDK_CANDIDATES = [
    "/opt/homebrew/opt/openjdk@21/libexec/openjdk.jdk/Contents/Home",
    "/opt/homebrew/opt/openjdk/libexec/openjdk.jdk/Contents/Home",
]
# pyghidra needs this even when installed from Ghidra's own bundled wheel.
GHIDRA_CANDIDATES = [
    "/opt/homebrew/Cellar/ghidra/12.1.2/libexec",
    "/opt/homebrew/opt/ghidra/libexec",
]

SCHEMA = """
PRAGMA journal_mode=OFF;
PRAGMA synchronous=OFF;

CREATE TABLE meta (key TEXT PRIMARY KEY, value TEXT);

CREATE TABLE blocks (
    name TEXT, start INTEGER, end INTEGER, size INTEGER,
    r INTEGER, w INTEGER, x INTEGER, initialized INTEGER);

CREATE TABLE functions (
    addr INTEGER PRIMARY KEY, name TEXT, size INTEGER,
    is_thunk INTEGER, is_external INTEGER, calling_convention TEXT,
    param_count INTEGER, signature TEXT);

CREATE TABLE symbols (
    addr INTEGER, name TEXT, type TEXT, namespace TEXT,
    source TEXT, is_primary INTEGER);

CREATE TABLE strings (
    addr INTEGER, value TEXT, length INTEGER, datatype TEXT);

CREATE TABLE xrefs (
    from_addr INTEGER, to_addr INTEGER, type TEXT,
    is_call INTEGER, from_func INTEGER);
"""

INDICES = """
CREATE INDEX idx_fn_name    ON functions(name);
CREATE INDEX idx_sym_addr   ON symbols(addr);
CREATE INDEX idx_sym_name   ON symbols(name);
CREATE INDEX idx_str_addr   ON strings(addr);
CREATE INDEX idx_xref_to    ON xrefs(to_addr);
CREATE INDEX idx_xref_from  ON xrefs(from_addr);
CREATE INDEX idx_xref_fnfrm ON xrefs(from_func);
"""


def ensure_env():
    """Locate the keg-only JDK and the Ghidra install.

    Homebrew's openjdk@21 is keg-only (never symlinked into the system JVM
    path), and pyghidra requires GHIDRA_INSTALL_DIR explicitly even when it was
    installed from Ghidra's own bundled wheel. Resolve both here rather than
    depending on the caller's shell environment.
    """
    if not os.environ.get("JAVA_HOME"):
        for c in JDK_CANDIDATES:
            if os.path.isfile(os.path.join(c, "bin", "java")):
                os.environ["JAVA_HOME"] = c
                break
        else:
            sys.stderr.write("[RE] no JDK found — brew install openjdk@21\n")
            sys.exit(1)

    if not os.environ.get("GHIDRA_INSTALL_DIR"):
        for c in GHIDRA_CANDIDATES:
            if os.path.isdir(os.path.join(c, "Ghidra")):
                os.environ["GHIDRA_INSTALL_DIR"] = c
                break
        else:
            sys.stderr.write("[RE] Ghidra not found — brew install ghidra "
                             "(FORMULA, not --cask)\n")
            sys.exit(1)


def main():
    if len(sys.argv) < 3:
        sys.stderr.write(__doc__ or "")
        return 2
    proj_name = sys.argv[1]
    binary = sys.argv[2]
    out = sys.argv[3] if len(sys.argv) > 3 else os.path.join(
        PROJDIR, proj_name + ".db")

    ensure_env()
    import pyghidra
    pyghidra.start()

    if os.path.exists(out):
        os.remove(out)
    db = sqlite3.connect(out)
    db.executescript(SCHEMA)

    print(f"[RE] opening {proj_name} :: {os.path.basename(binary)}")
    # nested_project_location=False is REQUIRED: analyzeHeadless writes the
    # standalone (flat) project layout, while pyghidra defaults to its own
    # nested layout. With the default, pyghidra does not find the project,
    # silently creates a fresh one and re-imports the binary UNANALYZED — the
    # export then reports 0 functions and looks like an API problem.
    with pyghidra.open_program(binary, project_location=PROJDIR,
                               project_name=proj_name, analyze=False,
                               nested_project_location=False) as api:
        prog = api.getCurrentProgram()
        nfn = prog.getFunctionManager().getFunctionCount()
        if nfn == 0:
            sys.stderr.write(
                "[RE] ABORT: program has 0 functions — this is an unanalyzed "
                "import, not the analyzed project.\n"
                "[RE] Run analysis first: sh analysis/re/import_binary.sh "
                f"{proj_name} {binary}\n")
            sys.exit(1)
        _export(db, prog, proj_name, binary)

    print("[RE] building indices")
    db.executescript(INDICES)
    db.commit()

    counts = {t: db.execute(f"SELECT COUNT(*) FROM {t}").fetchone()[0]
              for t in ("blocks", "functions", "symbols", "strings", "xrefs")}
    db.close()

    print(f"[RE] wrote {out}")
    for t, n in counts.items():
        print(f"       {t:<10s} {n:>9,d}")
    return 0


def _export(db, prog, proj_name, binary):

    base = prog.getImageBase().getOffset()
    db.executemany("INSERT INTO meta VALUES (?,?)", [
        ("project", proj_name),
        ("binary", binary),
        ("program", prog.getName()),
        ("image_base", hex(base)),
        ("language", str(prog.getLanguageID())),
        ("compiler", str(prog.getCompilerSpec().getCompilerSpecID())),
        ("executable_md5", str(prog.getExecutableMD5())),
        ("ghidra_creation", str(prog.getCreationDate())),
    ])

    # --- memory blocks ---
    rows = []
    for b in prog.getMemory().getBlocks():
        rows.append((str(b.getName()), b.getStart().getOffset(),
                     b.getEnd().getOffset(), int(b.getSize()),
                     int(b.isRead()), int(b.isWrite()), int(b.isExecute()),
                     int(b.isInitialized())))
    db.executemany("INSERT INTO blocks VALUES (?,?,?,?,?,?,?,?)", rows)
    print(f"[RE]   blocks    {len(rows)}")

    # --- functions ---
    rows = []
    fm = prog.getFunctionManager()
    for f in fm.getFunctions(True):
        try:
            sig = str(f.getSignature().getPrototypeString())
        except Exception:
            sig = ""
        rows.append((f.getEntryPoint().getOffset(), str(f.getName()),
                     int(f.getBody().getNumAddresses()),
                     int(f.isThunk()), int(f.isExternal()),
                     str(f.getCallingConventionName()),
                     int(f.getParameterCount()), sig))
    db.executemany(
        "INSERT OR REPLACE INTO functions VALUES (?,?,?,?,?,?,?,?)", rows)
    print(f"[RE]   functions {len(rows)}")

    # --- symbols ---
    rows = []
    for s in prog.getSymbolTable().getAllSymbols(True):
        a = s.getAddress()
        if a is None:
            continue
        rows.append((a.getOffset(), str(s.getName()),
                     str(s.getSymbolType()), str(s.getParentNamespace().getName()),
                     str(s.getSource()), int(s.isPrimary())))
        if len(rows) >= 50000:
            db.executemany("INSERT INTO symbols VALUES (?,?,?,?,?,?)", rows)
            rows = []
    db.executemany("INSERT INTO symbols VALUES (?,?,?,?,?,?)", rows)
    print("[RE]   symbols   done")

    # --- defined strings ---
    # Data.hasStringValue() rather than DefinedDataIterator.definedStrings(),
    # which does not exist in Ghidra 12.
    rows = []
    for d in prog.getListing().getDefinedData(True):
        try:
            if not d.hasStringValue():
                continue
            val = str(d.getValue())
        except Exception:
            continue
        rows.append((d.getAddress().getOffset(), val, len(val),
                     str(d.getDataType().getName())))
        if len(rows) >= 50000:
            db.executemany("INSERT INTO strings VALUES (?,?,?,?)", rows)
            rows = []
    db.executemany("INSERT INTO strings VALUES (?,?,?,?)", rows)
    print("[RE]   strings   done")

    # --- references ---
    rm = prog.getReferenceManager()
    rows = []
    total = 0
    it = rm.getReferenceSourceIterator(prog.getMemory().getLoadedAndInitializedAddressSet(), True)
    while it.hasNext():
        src = it.next()
        fn = fm.getFunctionContaining(src)
        fn_entry = fn.getEntryPoint().getOffset() if fn is not None else None
        for r in rm.getReferencesFrom(src):
            rt = r.getReferenceType()
            rows.append((src.getOffset(), r.getToAddress().getOffset(),
                         str(rt), int(rt.isCall()), fn_entry))
        if len(rows) >= 100000:
            db.executemany("INSERT INTO xrefs VALUES (?,?,?,?,?)", rows)
            total += len(rows)
            rows = []
    db.executemany("INSERT INTO xrefs VALUES (?,?,?,?,?)", rows)
    total += len(rows)
    print(f"[RE]   xrefs     {total}")


if __name__ == "__main__":
    sys.exit(main())
