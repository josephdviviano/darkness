#!/usr/bin/env python3
"""Keep every config surface in sync: the parser, the two yamls, and --help.

Four files have to agree about what options exist, and nothing but discipline
used to make them. Discipline lost: by 2026-08-05 `--help` listed 21 of ~170
keys, two live keys were documented nowhere at all, and the live config was 29
keys behind the example. Each was invisible until someone went looking.

    src/main/RenderConfig.h        the parser — decides what a key DOES
        │  A: every parsed key is documented
        ▼
    darknessRender.example.yaml    the reference — every key, default, why
        │  B: every documented key is really parsed   (dead documentation)
        │  C: same key set as the live config          (or it is never tested)
        │  D: the generated help is current
        ├──────────────► darknessRender.yaml
        └──────────────► src/main/ConfigHelpText.h ──► darknessRender --help

Usage:
    python3 tools/config_audit.py              # check the working tree
    python3 tools/config_audit.py --fix        # regenerate the help, then check
    python3 tools/config_audit.py --report     # option-surface census
    python3 tools/config_audit.py --staged     # check the git index (pre-commit)
    python3 tools/config_audit.py --precommit  # --staged, but skip when no
                                               #   config file is being committed

Exit status is 1 if any check fails, so it works as a pre-commit hook. Every
failure names the key and the fix; "config docs are stale" is not actionable.
"""

import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_config_help  # noqa: E402  (path set above)

try:
    import yaml
except ImportError:
    # Same call the leak hook makes when its detector is missing: a machine
    # without PyYAML has no way to pass, so blocking every config commit there
    # helps nobody. Scream — loudly enough that it cannot be mistaken for a
    # pass — and let the commit through. Interactive runs still fail, because
    # there the user asked for an answer and "unavailable" is not one.
    sys.stderr.write(
        "\n[CONFIG_AUDIT UNAVAILABLE] PyYAML is not importable — the config "
        "surfaces are NOT being checked.\n  Install it: python3 -m pip install "
        "pyyaml\n\n")
    sys.exit(0 if "--precommit" in sys.argv else 1)

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

PARSER = "src/main/RenderConfig.h"
EXAMPLE = "darknessRender.example.yaml"
LIVE = "darknessRender.yaml"
HELP = "src/main/ConfigHelpText.h"
DEPRECATED = "tools/deprecated_config_keys.txt"

# Files whose staging should trigger the audit in --precommit mode.
CONFIG_FILES = {PARSER, EXAMPLE, HELP, "tools/gen_config_help.py",
                "tools/config_audit.py", DEPRECATED}

# How a key can be READ by the parser. Both forms are needed: most keys are
# `node["key"]`, but tints go through a helper, and a key read only through a
# form missing here looks like dead documentation to check B. If B reports a
# key you can see being parsed, the pattern list is what is out of date.
KEY_READ_PATTERNS = [
    re.compile(r'\[\s*"([a-z0-9_]+)"\s*\]'),              # node["key"]
    re.compile(r'readTint\(\s*\w+\s*,\s*"([a-z0-9_]+)"'),  # readTint(node, "key", …)
]

# Sanity floor for the scan: if a future refactor moves parsing to a registry,
# the regexes above silently match nothing and every check turns green while
# checking nothing. Failing loudly beats passing vacuously.
MIN_EXPECTED_PARSER_KEYS = 100


class Failure(Exception):
    pass


# ── reading the sources ───────────────────────────────────────────────────

def read(path, staged):
    """File content, from the git index when auditing a commit.

    The index — not the diff — is what the resulting commit contains: for an
    unmodified file `git show :path` returns the same bytes as HEAD, so this
    reads the whole prospective tree, not just what changed.
    """
    if staged:
        r = subprocess.run(["git", "show", f":{path}"], cwd=REPO,
                           capture_output=True, text=True)
        if r.returncode == 0:
            return r.stdout
        # Not in the index (new untracked file): fall back to the worktree, and
        # say so rather than treating a missing file as an empty one.
        sys.stderr.write(f"  note: {path} is not in the git index — "
                         f"reading the working tree copy\n")
    with open(os.path.join(REPO, path), encoding="utf-8") as fh:
        return fh.read()


def key_names(node):
    """Every key NAME in a parsed yaml tree.

    Names, not dotted paths: the parser is read by text scan, which sees
    `node["key"]` without the path that reached it. Sequences are leaves —
    `rgb_filter: [1, 1, 1]` is one key with a list value, not three keys.
    """
    out = set()
    if isinstance(node, dict):
        for k, v in node.items():
            out.add(str(k))
            out |= key_names(v)
    return out


def parser_keys(src):
    keys = set()
    for pattern in KEY_READ_PATTERNS:
        keys |= set(pattern.findall(src))
    return keys


def deprecated_keys(text):
    out = set()
    for line in text.splitlines():
        line = line.split("#")[0].strip()
        if line:
            out.add(line)
    return out


# ── the checks ────────────────────────────────────────────────────────────

def check_parser_documented(parsed, documented, deprecated):
    """A: every key the parser reads is documented in the example yaml."""
    missing = sorted(parsed - documented - deprecated)
    if not missing:
        return []
    out = [f"{len(missing)} config key(s) parsed but documented nowhere:"]
    for k in missing:
        out.append(f"    {k}")
    out.append("  Fix: document each in darknessRender.example.yaml (and add it")
    out.append(f"  to {LIVE} too — check C). If the parser reads it only to")
    out.append(f"  report its removal, add it to {DEPRECATED} instead.")
    return out


def check_no_dead_docs(parsed, documented):
    """B: every documented key is actually read by the parser."""
    dead = sorted(documented - parsed)
    if not dead:
        return []
    out = [f"{len(dead)} documented config key(s) the parser never reads:"]
    for k in dead:
        out.append(f"    {k}")
    out.append("  Fix: either the key is gone and the documentation should go")
    out.append("  with it, or the parser reads it through a form that")
    out.append(f"  KEY_READ_PATTERNS in {os.path.basename(__file__)} does not")
    out.append("  recognise yet — a new helper like readTint(). Check which")
    out.append("  before deleting anything.")
    if len(dead) > 3:
        # The usual cause of a LARGE list here is a split commit rather than
        # rotten docs: the yaml change is staged and the RenderConfig.h change
        # that reads those keys is not, so the commit would document options
        # that do not exist yet.
        out.append("  Many at once usually means the parser change is not staged")
        out.append("  alongside the yaml change — stage both, or --no-verify if")
        out.append("  you are deliberately splitting the commit.")
    return out


def check_live_parity(documented, live_text):
    """C: the live config carries exactly the example's key set."""
    if live_text is None:
        print(f"  - {LIVE} absent (gitignored, not on this machine) — "
              f"parity check skipped")
        return []
    live = key_names(yaml.safe_load(live_text))
    missing = sorted(documented - live)
    extra = sorted(live - documented)
    if not missing and not extra:
        return []
    out = []
    if missing:
        out.append(f"{len(missing)} key(s) in {EXAMPLE} but missing from {LIVE}:")
        for k in missing:
            out.append(f"    {k}")
        out.append(f"  Fix: add them to {LIVE}. A key that exists only in the")
        out.append("  example is a key nobody ever runs with, so the C++ default")
        out.append("  is the only value ever exercised.")
    if extra:
        out.append(f"{len(extra)} key(s) in {LIVE} but missing from {EXAMPLE}:")
        for k in extra:
            out.append(f"    {k}")
        out.append(f"  Fix: document them in {EXAMPLE}. Values may differ between")
        out.append("  the two files; the key sets may not.")
    return out


def check_help_current(example_text, help_text):
    """D: the generated help header matches the example yaml."""
    expected = gen_config_help.generate(example_text)
    if expected == help_text:
        return []
    return [
        f"{HELP} is stale relative to {EXAMPLE}.",
        "  Fix: python3 tools/config_audit.py --fix",
        f"  (or: python3 tools/gen_config_help.py {EXAMPLE} {HELP})",
    ]


# ── census ────────────────────────────────────────────────────────────────

def field_for_key(src):
    """Map each parsed key to the cfg.<field> its parse site assigns.

    Heuristic — the first `cfg.<field>` within a few lines of the lookup — and
    it is only used by --report, never by a blocking check. Container keys
    (`graphics:`, `film:`) legitimately have no field.
    """
    lines = src.split("\n")
    out = {}
    for i, line in enumerate(lines):
        for pattern in KEY_READ_PATTERNS:
            for key in pattern.findall(line):
                if key in out:
                    continue
                for j in range(i, min(i + 4, len(lines))):
                    m = re.search(r"cfg\.([A-Za-z_][A-Za-z0-9_]*)", lines[j])
                    if m:
                        out[key] = m.group(1)
                        break
    return out


def source_files():
    for root, _, files in os.walk(os.path.join(REPO, "src")):
        for f in files:
            if f.endswith((".h", ".cpp", ".inl")):
                yield os.path.join(root, f)


def leaf_paths(node, prefix=""):
    """Every settable option, as a dotted path.

    Leaves, not names: this is the count that answers "how many options are
    there", and it is the one to narrow. `key_names` deliberately collapses
    repeats (`enabled` appears in a dozen sections) because the parser scan
    sees bare names — using it here would undercount.
    """
    out = {}
    if isinstance(node, dict):
        for k, v in node.items():
            p = f"{prefix}.{k}" if prefix else str(k)
            if isinstance(v, dict):
                out.update(leaf_paths(v, p))
            else:
                out[p] = v
    return out


def report(example_text, live_text, deprecated, src):
    example = yaml.safe_load(example_text)

    print("\n══ option surface ══\n")
    total = 0
    for section, node in example.items():
        n = len(leaf_paths(node))
        total += n
        print(f"  {section + ':':<14} {n:>4} options")
    print(f"  {'TOTAL':<14} {total:>4} settable options\n")

    # Parsed but never consumed — the CFG-1 audit, automated. A key whose cfg
    # field appears nowhere outside RenderConfig.h is parsed, clamped, stored,
    # and then ignored: the user sets it and nothing happens.
    fields = field_for_key(src)
    blob = ""
    for path in source_files():
        if path.endswith("RenderConfig.h"):
            continue
        with open(path, encoding="utf-8", errors="replace") as fh:
            blob += fh.read()

    unconsumed = sorted(k for k, f in fields.items()
                        if k not in deprecated and f"{f}" not in blob)
    print(f"  parsed but never read outside the parser: {len(unconsumed)}")
    for k in unconsumed:
        print(f"    {k:<28} (cfg.{fields[k]})")
    if unconsumed:
        print("    ^ each is a knob the user can set that does nothing. Wire it")
        print("      up or delete it — CFG-1's rule.")

    # Never tuned: the live config agrees with the example's default. Not a
    # defect — most keys should sit at their default — but it is the list to
    # start from when narrowing the surface, because a key nobody has ever
    # moved is a key whose value nobody has an opinion about.
    if live_text is not None:
        live = yaml.safe_load(live_text)
        ex_leaves, live_leaves = leaf_paths(example), leaf_paths(live)
        same = [k for k, v in ex_leaves.items()
                if k in live_leaves and live_leaves[k] == v]
        print(f"\n  live value still at the example default: "
              f"{len(same)} of {len(ex_leaves)}")
        tuned = sorted(k for k in ex_leaves
                       if k in live_leaves and live_leaves[k] != ex_leaves[k])
        print(f"  tuned away from the default: {len(tuned)}")
        for k in tuned:
            print(f"    {k:<44} {ex_leaves[k]!r} -> {live_leaves[k]!r}")

    print(f"\n  deprecated keys still parsed for the warning: {len(deprecated)}")
    print(f"    {', '.join(sorted(deprecated))}\n")


# ── driver ────────────────────────────────────────────────────────────────

def main():
    args = set(sys.argv[1:])
    staged = bool(args & {"--staged", "--precommit"})

    if "--precommit" in args:
        changed = subprocess.run(
            ["git", "diff", "--cached", "--name-only"], cwd=REPO,
            capture_output=True, text=True).stdout.split()
        if not (set(changed) & CONFIG_FILES):
            return 0
        print("config audit (staged):")
    elif "--report" not in args:
        print(f"config audit ({'index' if staged else 'working tree'}):")

    src = read(PARSER, staged)
    example_text = read(EXAMPLE, staged)
    help_text = read(HELP, staged)
    try:
        # The live config is gitignored, so it is a working-tree read even when
        # auditing the index — there is no staged version to compare against.
        with open(os.path.join(REPO, LIVE), encoding="utf-8") as fh:
            live_text = fh.read()
    except FileNotFoundError:
        live_text = None

    deprecated = deprecated_keys(read(DEPRECATED, staged))
    parsed = parser_keys(src)
    documented = key_names(yaml.safe_load(example_text))

    if len(parsed) < MIN_EXPECTED_PARSER_KEYS:
        sys.stderr.write(
            f"\nCONFIG AUDIT ABORTED: only {len(parsed)} keys found in "
            f"{PARSER}, expected at least {MIN_EXPECTED_PARSER_KEYS}.\n"
            f"KEY_READ_PATTERNS no longer matches how config is parsed, so "
            f"every check below would pass without checking anything.\n\n")
        return 1

    if "--report" in args:
        report(example_text, live_text, deprecated, src)
        return 0

    if "--fix" in args:
        generated = gen_config_help.generate(example_text)
        path = os.path.join(REPO, HELP)
        if generated != help_text:
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(generated)
            print(f"  - regenerated {HELP}")
            help_text = generated
        else:
            print(f"  - {HELP} already current")

    problems = []
    problems += check_parser_documented(parsed, documented, deprecated)
    problems += check_no_dead_docs(parsed, documented)
    problems += check_live_parity(documented, live_text)
    problems += check_help_current(example_text, help_text)

    if problems:
        sys.stderr.write("\nCONFIG AUDIT FAILED\n\n")
        for line in problems:
            sys.stderr.write(("  " if line.startswith("  ") else "  ") + line + "\n")
        sys.stderr.write(
            "\n  Re-run: python3 tools/config_audit.py "
            "(add --fix to regenerate the help)\n"
            "  Bypass once: git commit --no-verify\n\n")
        return 1

    live_note = "" if live_text is not None else f", {LIVE} absent"
    print(f"  OK — {len(documented)} keys agree across the parser, both yamls "
          f"and --help{live_note}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
