#!/usr/bin/env python3
"""Catalog the import table of a Dark Engine binary.

Phase-1 bootstrap extractor. Answers "what OS/middleware APIs does the engine
actually call" without any disassembly — in particular which audio stack is in
use (DirectSound / EAX / OpenAL), which the third-party RE effort never
extracted despite cataloguing it at bootstrap.

Usage:
    python3 analysis/re/bootstrap_imports.py <binary> [<binary> ...]
    python3 analysis/re/bootstrap_imports.py --audio <binary>   # audio DLLs only

Output is a stable, diffable listing: run it against two binary versions and
diff to see what changed between them.

See .claude/PLAN.BINARY_RE_PIPELINE.md.
"""
import os
import sys

try:
    import pefile
except ImportError:
    sys.stderr.write("[RE] pefile missing — pip install pefile\n")
    sys.exit(1)

# DLLs worth calling out explicitly when surveying the audio stack.
AUDIO_DLLS = {
    "dsound.dll", "dsound3d.dll", "eax.dll", "openal32.dll", "soft_oal.dll",
    "winmm.dll", "msacm32.dll", "dxguid.dll", "dinput.dll", "dinput8.dll",
    "avformat.dll", "avcodec.dll", "ffmpeg.dll", "lgvid.dll",
}


def describe(path, audio_only=False):
    pe = pefile.PE(path, fast_load=True)
    pe.parse_data_directories(directories=[
        pefile.DIRECTORY_ENTRY["IMAGE_DIRECTORY_ENTRY_IMPORT"],
        pefile.DIRECTORY_ENTRY["IMAGE_DIRECTORY_ENTRY_DELAY_IMPORT"],
    ])

    base = pe.OPTIONAL_HEADER.ImageBase
    machine = pefile.MACHINE_TYPE.get(pe.FILE_HEADER.Machine,
                                      hex(pe.FILE_HEADER.Machine))
    print("=" * 78)
    print(f"{os.path.basename(path)}")
    print("=" * 78)
    print(f"  path        : {path}")
    print(f"  size        : {os.path.getsize(path):,} bytes")
    print(f"  machine     : {machine}")
    print(f"  image base  : 0x{base:X}")
    print(f"  sections    : "
          f"{', '.join(s.Name.rstrip(chr(0).encode()).decode('latin-1', 'replace') for s in pe.sections)}")
    print()

    groups = []
    for attr, label in (("DIRECTORY_ENTRY_IMPORT", "import"),
                        ("DIRECTORY_ENTRY_DELAY_IMPORT", "delay-import")):
        for entry in getattr(pe, attr, []) or []:
            dll = (entry.dll or b"").decode("latin-1", "replace")
            names = []
            for imp in entry.imports:
                if imp.name:
                    names.append(imp.name.decode("latin-1", "replace"))
                else:
                    names.append(f"#ordinal_{imp.ordinal}")
            groups.append((dll, label, sorted(names)))

    groups.sort(key=lambda g: g[0].lower())

    audio_hits = [g for g in groups if g[0].lower() in AUDIO_DLLS]

    print(f"  {len(groups)} imported module(s), "
          f"{sum(len(g[2]) for g in groups)} symbol(s)")
    print()
    print("  --- AUDIO-RELEVANT MODULES ---")
    if audio_hits:
        for dll, label, names in audio_hits:
            print(f"    {dll}  ({label}, {len(names)} symbols)")
            for n in names:
                print(f"        {n}")
    else:
        print("    none of the known audio DLLs are imported.")
        print("    => the audio backend is loaded DYNAMICALLY (LoadLibrary) or")
        print("       statically linked; look for the DLL name as a string.")
    print()

    if audio_only:
        return

    print("  --- ALL MODULES ---")
    for dll, label, names in groups:
        mark = " *" if dll.lower() in AUDIO_DLLS else ""
        print(f"    {dll:<24s} {label:<13s} {len(names):>4d} symbols{mark}")
    print()


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    audio_only = "--audio" in sys.argv[1:]
    if not args:
        sys.stderr.write(__doc__ or "usage: bootstrap_imports.py <binary>...\n")
        return 2
    for path in args:
        if not os.path.isfile(path):
            sys.stderr.write(f"[RE] not found: {path}\n")
            continue
        describe(path, audio_only)
    return 0


if __name__ == "__main__":
    sys.exit(main())
