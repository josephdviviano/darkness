#!/usr/bin/env python3
"""Decompile functions from an analyzed Ghidra project.

Usage:
    python3 analysis/re/decompile.py <project> <binary> <VA> [<VA> ...]
    python3 analysis/re/decompile.py <project> <binary> --reads <VA> [--window N]

`--reads` finds every function that references the given address and decompiles
each one — the usual question when chasing "what actually consumes this field".

Example:
    python3 analysis/re/decompile.py Thief2_ND128 .../Thief2.exe 0x5B76D0

See .claude/PLAN.BINARY_RE_PIPELINE.md.
"""
import os
import sys

PROJDIR = "/Users/jdv/code/darkness/re_projects"
JDK_CANDIDATES = [
    "/opt/homebrew/opt/openjdk@21/libexec/openjdk.jdk/Contents/Home",
    "/opt/homebrew/opt/openjdk/libexec/openjdk.jdk/Contents/Home",
]
GHIDRA_CANDIDATES = [
    "/opt/homebrew/Cellar/ghidra/12.1.2/libexec",
    "/opt/homebrew/opt/ghidra/libexec",
]


def ensure_env():
    if not os.environ.get("JAVA_HOME"):
        for c in JDK_CANDIDATES:
            if os.path.isfile(os.path.join(c, "bin", "java")):
                os.environ["JAVA_HOME"] = c
                break
    if not os.environ.get("GHIDRA_INSTALL_DIR"):
        for c in GHIDRA_CANDIDATES:
            if os.path.isdir(os.path.join(c, "Ghidra")):
                os.environ["GHIDRA_INSTALL_DIR"] = c
                break
    if not os.environ.get("JAVA_HOME") or not os.environ.get("GHIDRA_INSTALL_DIR"):
        sys.stderr.write("[RE] JDK or Ghidra not found — see import_binary.sh\n")
        sys.exit(1)


def main():
    if len(sys.argv) < 4:
        sys.stderr.write(__doc__ or "")
        return 2
    proj, binary = sys.argv[1], sys.argv[2]
    rest = sys.argv[3:]

    ensure_env()
    import pyghidra
    pyghidra.start()

    from ghidra.app.decompiler import DecompInterface
    from ghidra.util.task import ConsoleTaskMonitor

    with pyghidra.open_program(binary, project_location=PROJDIR,
                               project_name=proj, analyze=False,
                               nested_project_location=False) as api:
        prog = api.getCurrentProgram()
        if prog.getFunctionManager().getFunctionCount() == 0:
            sys.stderr.write("[RE] ABORT: unanalyzed program (0 functions)\n")
            return 1
        af = prog.getAddressFactory().getDefaultAddressSpace()
        fm = prog.getFunctionManager()
        rm = prog.getReferenceManager()

        # Raw disassembly. The decompiler sometimes folds a string reference
        # away entirely (config-var registration loops are a common case), so
        # when you need to see exactly which global a value lands in, read the
        # instructions instead.
        if rest and rest[0] == "--disasm":
            start = af.getAddress(rest[1])
            count = int(rest[2], 0) if len(rest) > 2 else 40
            listing = prog.getListing()
            ins = listing.getInstructionAt(start)
            if ins is None:
                ins = listing.getInstructionAfter(start)
            n = 0
            while ins is not None and n < count:
                a = ins.getAddress()
                refs = []
                for r in ins.getReferencesFrom():
                    t = r.getToAddress()
                    d = prog.getListing().getDataAt(t)
                    sval = ""
                    if d is not None and d.hasStringValue():
                        sval = f'  "{d.getValue()}"'
                    refs.append(f"-> 0x{t.getOffset():X}{sval}")
                print(f"  0x{a.getOffset():08X}  {ins}"
                      f"{('   ' + ' '.join(refs)) if refs else ''}")
                ins = ins.getNext()
                n += 1
            return 0

        targets = []
        if rest and rest[0] == "--reads":
            addr = af.getAddress(rest[1])
            seen = set()
            for r in rm.getReferencesTo(addr):
                fn = fm.getFunctionContaining(r.getFromAddress())
                if fn is not None and fn.getEntryPoint().getOffset() not in seen:
                    seen.add(fn.getEntryPoint().getOffset())
                    targets.append(fn)
            if not targets:
                print(f"[RE] no function references {rest[1]}")
                return 1
        else:
            for a in rest:
                fn = fm.getFunctionContaining(af.getAddress(a))
                if fn is None:
                    print(f"[RE] no function at {a}")
                    continue
                targets.append(fn)

        dec = DecompInterface()
        dec.openProgram(prog)
        mon = ConsoleTaskMonitor()
        for fn in targets:
            print("=" * 74)
            print(f"{fn.getName()}  @ 0x{fn.getEntryPoint().getOffset():X}"
                  f"  ({fn.getBody().getNumAddresses()} bytes)")
            print("=" * 74)
            res = dec.decompileFunction(fn, 120, mon)
            if res.decompileCompleted():
                print(res.getDecompiledFunction().getC())
            else:
                print(f"[RE] decompile failed: {res.getErrorMessage()}")
        dec.dispose()
    return 0


if __name__ == "__main__":
    sys.exit(main())
