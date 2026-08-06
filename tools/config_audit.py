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
        │  E: the struct default is what the example says it is
        ├──────────────► darknessRender.yaml
        └──────────────► src/main/ConfigHelpText.h ──► darknessRender --help

Usage:
    python3 tools/config_audit.py              # check the working tree
    python3 tools/config_audit.py --fix        # regenerate the help, then check
    python3 tools/config_audit.py --report     # option-surface census
    python3 tools/config_audit.py --trace KEY  # where does this value come from?
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


def key_paths(node, prefix=""):
    """Every key as a DOTTED PATH.

    Check C compares two yaml files, and both carry full path information —
    unlike the parser comparisons, which are limited to bare names because the
    parser is read by text scan. Using names there too was a real blind spot:
    a nested key could go missing from the live config and stay invisible so
    long as its leaf name appeared anywhere else in the tree. That is exactly
    how `graphics.post_process.halation.threshold` was dropped while the audit
    reported OK — `threshold` also exists under `bloom`.
    """
    out = set()
    if isinstance(node, dict):
        for k, v in node.items():
            path = f"{prefix}.{k}" if prefix else str(k)
            out.add(path)
            out |= key_paths(v, path)
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


def check_live_parity(example_text, live_text):
    """C: the live config carries exactly the example's key set."""
    if live_text is None:
        print(f"  - {LIVE} absent (gitignored, not on this machine) — "
              f"parity check skipped")
        return []
    # Paths, not bare names — see key_paths().
    live = key_paths(yaml.safe_load(live_text))
    documented = key_paths(yaml.safe_load(example_text))
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

# `YAML::Node child = parent["key"]` — the parser's block structure, which is
# what makes a key's FULL path recoverable rather than just its name.
BIND_RE = re.compile(r'YAML::Node\s+(\w+)\s*=\s*(\w+)\s*\[\s*"([a-z0-9_]+)"\s*\]')
LOOKUP_RE = re.compile(r'\b(\w+)\s*\[\s*"([a-z0-9_]+)"\s*\]')
TINT_RE = re.compile(r'readTint\(\s*(\w+)\s*,\s*"([a-z0-9_]+)"')
FIELD_RE = re.compile(r"cfg\.([A-Za-z_]\w*)")

# A struct field with an initialiser: `float ppExposure = 1.0f;`
DECL_RE = re.compile(
    r"^\s{4}(?:bool|int|float|double|std::string|uint\d+_t)\s+([A-Za-z_]\w*)"
    r"\s*=\s*([^;{]+);", re.M)

# `# [default: 1024]` in the example yaml — states the C++ fallback where the
# shipped value deliberately differs from it.
DEFAULT_MARKER_RE = re.compile(r"\[default:\s*([^\]]+)\]")


def path_to_field(src_text):
    """Map each key's FULL dotted path to the `cfg.<field>` it assigns.

    Paths, not bare names, because names collide badly: `enabled` appears in
    a dozen blocks, `rays` in both realtime and bake, `height` in both
    render_scale and probes. A name-keyed map reported 14 false mismatches
    before this existed — every one a collision, not a real divergence.
    """
    src = src_text.split("\n")
    varpath = {"root": ""}     # the parser's own root node
    out = {}
    for i, line in enumerate(src):
        for m in BIND_RE.finditer(line):
            child, parent, key = m.groups()
            base = varpath.get(parent)
            if base is not None:
                varpath[child] = f"{base}.{key}" if base else key
        for m in list(LOOKUP_RE.finditer(line)) + list(TINT_RE.finditer(line)):
            var, key = m.group(1), m.group(2)
            if var not in varpath or BIND_RE.search(line):
                continue
            base = varpath[var]
            full = f"{base}.{key}" if base else key
            if full in out:
                continue
            for j in range(i, min(i + 5, len(src))):
                f = FIELD_RE.search(src[j])
                if f:
                    out[full] = f.group(1)
                    break
    return out


def struct_defaults(src_text):
    return {m.group(1): m.group(2).strip() for m in DECL_RE.finditer(src_text)}


def documented_defaults(example_text):
    """Per-key `[default: X]` markers, read from the example yaml's comments."""
    out = {}
    stack = []
    for raw in example_text.split("\n"):
        line = raw.rstrip()
        if not line.strip() or line.strip().startswith("#"):
            continue
        m = re.match(r"^(\s*)([A-Za-z_][\w]*):(.*)$", line)
        if not m:
            continue
        indent, key, rest = len(m.group(1)), m.group(2), m.group(3)
        stack[:] = [(i, k) for i, k in stack if i < indent]
        stack.append((indent, key))
        marker = DEFAULT_MARKER_RE.search(rest)
        if marker:
            out[".".join(k for _, k in stack)] = marker.group(1).strip()
    return out


def _as_value(text):
    """Normalise a C++ initialiser or a marker into a comparable value."""
    t = text.strip().rstrip("f").strip('"').strip()
    if t in ("true", "false"):
        return t == "true"
    try:
        return float(t)
    except ValueError:
        return t


def check_defaults(src_text, example_text):
    """E: the struct default is what the example yaml says it is.

    The subtlety this exists for: the example's VALUE is the shipped setting,
    which is not always the same as the C++ fallback you get by deleting the
    key. Where they differ deliberately, the comment carries `[default: X]`
    and that is what gets compared. Where there is no marker, the value is
    taken as the claim.
    """
    fields = struct_defaults(src_text)
    pathfield = path_to_field(src_text)
    markers = documented_defaults(example_text)
    leaves = leaf_paths(yaml.safe_load(example_text))

    problems, checked, unverifiable = [], 0, []
    for path, value in leaves.items():
        field = pathfield.get(path)
        if not field or field not in fields:
            unverifiable.append((path, "no struct field found"))
            continue
        cpp = _as_value(fields[field])
        claim = _as_value(markers[path]) if path in markers else value
        if isinstance(claim, (list, dict)) or claim is None:
            unverifiable.append((path, "sequence — needs a [default: …] marker"))
            continue
        if isinstance(claim, str) or isinstance(cpp, str):
            unverifiable.append((path, f"enum/string vs cfg.{field} = {fields[field]}"))
            continue
        checked += 1
        if isinstance(claim, bool) or isinstance(cpp, bool):
            if bool(claim) == bool(cpp):
                continue
        elif abs(float(claim) - float(cpp)) < 1e-9:
            continue
        problems.append((path, field, claim, fields[field],
                         "marker" if path in markers else "value"))

    print(f"  - defaults: {checked} compared, {len(unverifiable)} not comparable "
          f"(enums, sequences, unmapped)")

    if not problems:
        return []
    out = [f"{len(problems)} key(s) whose documented default is not the "
           f"struct default:"]
    for path, field, claim, cpp, src in problems:
        out.append(f"    {path}")
        out.append(f"      example {src} says {claim!r}, cfg.{field} = {cpp}")
    out.append("  Fix: whichever is wrong. If the example's VALUE is a")
    out.append("  deliberate recommendation that differs from the C++ fallback,")
    out.append("  state the fallback in the comment as `[default: X]` — then a")
    out.append("  reader can see both what ships and what deleting the key gives.")
    return out


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

def trace(query, src_text, example_text, live_text):
    """Answer "where does this value come from?" for one key.

    The question that motivated check E: with four places a value can live,
    reading any one of them tells you less than you think.
    """
    fields = struct_defaults(src_text)
    pathfield = path_to_field(src_text)
    markers = documented_defaults(example_text)
    ex_leaves = leaf_paths(yaml.safe_load(example_text))
    live_leaves = leaf_paths(yaml.safe_load(live_text)) if live_text else {}

    hits = [p for p in ex_leaves if query in p]
    if not hits:
        hits = [p for p in pathfield if query in p]
    if not hits:
        print(f"  no config key matching '{query}'")
        return 1

    src_lines = src_text.split("\n")
    for path in sorted(hits):
        field = pathfield.get(path)
        print(f"\n══ {path}")
        print(f"  parsed into      cfg.{field}" if field else
              "  parsed into      (no cfg field found — check the parse site)")
        if field and field in fields:
            print(f"  C++ default      {fields[field]}"
                  f"   <- what you get if the key is ABSENT")
        if path in markers:
            print(f"  documented       [default: {markers[path]}] in the example comment")
        print(f"  example ships    {ex_leaves.get(path, '(absent)')!r}")
        if live_text is not None:
            lv = live_leaves.get(path)
            same = " (same as example)" if lv == ex_leaves.get(path) else " <- TUNED"
            print(f"  live config      {lv!r}{same if lv is not None else ' (absent)'}")
        # Clamp range, read from the parse site's clampConfigValue call.
        if field:
            for i, line in enumerate(src_lines):
                if f"cfg.{field}" in line and "clampConfigValue" in "".join(
                        src_lines[i:i + 3]):
                    rng = re.search(r"(\w+::k\w+),\s*(\w+::k\w+)",
                                    "".join(src_lines[i:i + 4]))
                    if rng:
                        print(f"  clamped to       {rng.group(1)} .. {rng.group(2)}")
                    break
            # Consumers, excluding the parser itself.
            hits_out = []
            for root, _, files in os.walk(os.path.join(REPO, "src")):
                for f in files:
                    if not f.endswith((".h", ".cpp", ".inl")) or f == "RenderConfig.h":
                        continue
                    p = os.path.join(root, f)
                    with open(p, encoding="utf-8", errors="replace") as fh:
                        if field in fh.read():
                            hits_out.append(os.path.relpath(p, REPO))
            print(f"  read by          {', '.join(hits_out) if hits_out else 'NOTHING OUTSIDE THE PARSER'}")
    return 0


def main():
    args = set(sys.argv[1:])
    positional = [a for a in sys.argv[1:] if not a.startswith("-")]
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

    if "--trace" in args:
        if not positional:
            raise SystemExit("usage: config_audit.py --trace <key-or-substring>")
        return trace(positional[0], src, example_text, live_text)

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
    problems += check_live_parity(example_text, live_text)
    problems += check_help_current(example_text, help_text)
    problems += check_defaults(src, example_text)

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
