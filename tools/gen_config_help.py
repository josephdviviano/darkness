#!/usr/bin/env python3
"""Generate src/main/ConfigHelpText.h from darknessRender.example.yaml.

`--help`'s config reference used to be a hand-written string literal, and it
drifted: by 2026-08-05 it listed 21 of ~170 keys and omitted every graphics
subsection the visual campaign had added, while README.md still called it the
canonical reference. Hand-syncing three places -- the parser, the example
yaml, and a help string -- is what produced that, so the help string stops
being one of the three: it is derived from the example yaml, which is the file
that already carries every key with its default and its reasoning.

Regenerate after editing darknessRender.example.yaml:

    python3 tools/gen_config_help.py darknessRender.example.yaml \
        src/main/ConfigHelpText.h

Forgetting to is caught by tests/test_config_help.cpp, which re-reads the
example yaml and fails naming any key the generated text is missing.

The example yaml is read as TEXT, not parsed as YAML: order and comments are
the whole point here, and both are lost by a parse.
"""

import re
import sys

# Layout of the generated reference. Descriptions are truncated at a word
# boundary to keep the whole thing scannable in an 100-column terminal --
# the example yaml remains the place for the full reasoning.
DESC_COLUMN = 42
MAX_WIDTH = 100

KEY_RE = re.compile(r"^(?P<indent> *)(?P<key>[A-Za-z_][A-Za-z_0-9]*):(?P<rest>.*)$")
COMMENT_RE = re.compile(r"^(?P<indent> *)#(?P<text>.*)$")

# A comment line that is only decoration carries no description.
DECORATION_RE = re.compile(r"^[\s─=—-]*$")


def split_value_and_comment(rest):
    """Split a YAML line's tail into (value, inline comment).

    Naive on purpose: a '#' inside a quoted string would be misread, but the
    example yaml has no such case, and the generator asserts nothing about
    values it cannot interpret -- it copies them through.
    """
    rest = rest.rstrip()
    if not rest:
        return "", ""
    # An inline comment needs whitespace before the '#', so a value like
    # "#ff00ff" would not be mistaken for one.
    m = re.search(r"\s+#\s?(.*)$", rest)
    if m:
        return rest[: m.start()].strip(), m.group(1).strip()
    return rest.strip(), ""


def truncate(text, width):
    if len(text) <= width:
        return text
    cut = text[: width - 1]
    if " " in cut:
        cut = cut[: cut.rindex(" ")]
    return cut + "…"


def collect(lines):
    """Walk the file, pairing each key with a one-line description.

    Description precedence: the key's own inline comment, else the first
    non-decoration line of the comment block immediately above it. The first
    line is used rather than the last because these blocks are written summary
    first, detail after.
    """
    out = []
    pending = []          # comment block currently accumulating
    prev_top_level = None

    for raw in lines:
        line = raw.rstrip("\n")

        if not line.strip():
            pending = []          # a blank line ends a comment block
            continue

        cm = COMMENT_RE.match(line)
        if cm:
            text = cm.group("text").strip()
            if not DECORATION_RE.match(text):
                # Strip the box-drawing decoration off section headings so
                # "── Antialiasing ──" reads as "Antialiasing".
                text = text.strip("─ ").strip()
                if text:
                    pending.append(text)
            continue

        km = KEY_RE.match(line)
        if not km:
            pending = []
            continue

        indent = len(km.group("indent"))
        key = km.group("key")
        value, inline = split_value_and_comment(km.group("rest"))
        desc = inline or (pending[0] if pending else "")
        pending = []

        # A section heading that just restates the key ("── paths ──" above
        # `paths:`) is noise in a two-column layout.
        if desc.replace(" ", "_").lower() == key.lower():
            desc = ""
        # Comment blocks often run "…the following. e.g." onto the next line;
        # a dangling lead-in reads worse than no description tail.
        desc = re.sub(r"[\s,:]*\be\.?g\.?$", "", desc).rstrip(" :,-")

        # Blank line between top-level sections, so the output has the same
        # shape as the file it came from.
        top = key if indent == 0 else prev_top_level
        if indent == 0:
            if prev_top_level is not None:
                out.append("")
            prev_top_level = key

        out.append(format_entry(indent, key, value, desc))
        _ = top

    return out


def format_entry(indent, key, value, desc):
    left = " " * (indent + 2) + key + ":"
    if value:
        left += " " + value
    if not desc:
        return left.rstrip()
    pad = max(DESC_COLUMN - len(left), 1)
    room = MAX_WIDTH - len(left) - pad
    if room < 20:            # long key/value: description goes on regardless
        room = 30
        pad = 1
    return left + " " * pad + truncate(desc, room)


def c_escape(s):
    return s.replace("\\", "\\\\").replace('"', '\\"')


def generate(example_text):
    """Return the full text of ConfigHelpText.h for a given example yaml.

    Separate from main() so tools/config_audit.py can regenerate in memory and
    compare against the checked-in header — the check that catches a stale
    generated file — without writing anything.
    """
    import io
    fh = io.StringIO()
    _emit(fh, collect(example_text.splitlines(keepends=True)))
    return fh.getvalue()


def main():
    if len(sys.argv) != 3:
        raise SystemExit(f"usage: {sys.argv[0]} <example-yaml> <output-header>")
    src, dst = sys.argv[1], sys.argv[2]

    text = open(src, encoding="utf-8").read()
    entries = collect(text.splitlines(keepends=True))
    with open(dst, "w", encoding="utf-8") as fh:
        _emit(fh, entries)

    keys = sum(1 for e in entries if e.strip())
    print(f"wrote {dst}: {keys} lines from {src}")


def _emit(fh, entries):
    fh.write(f"""\
// GENERATED FILE — do not edit.
// Regenerate with:
//   python3 tools/gen_config_help.py darknessRender.example.yaml \\
//       src/main/ConfigHelpText.h
//
// The YAML config reference printed by `darknessRender --help`, derived from
// darknessRender.example.yaml so the two cannot disagree. The example file is
// the authored source: every key, its default, and the reasoning behind it.
//
// Emitted as an array of lines rather than one string literal because MSVC
// caps a single string literal at 65535 bytes and this reference is close
// enough to that to be worth not thinking about again.
//
// tests/test_config_help.cpp fails if this file is stale relative to the
// example yaml, naming the missing keys.

#pragma once

namespace Darkness {{

inline const char *const kConfigHelpLines[] = {{
""")
    for line in entries:
        fh.write(f'    "{c_escape(line)}",\n')
    fh.write("};\n\n")
    fh.write("inline constexpr int kConfigHelpLineCount =\n"
             "    static_cast<int>(sizeof(kConfigHelpLines) "
             "/ sizeof(kConfigHelpLines[0]));\n\n")
    fh.write("} // namespace Darkness\n")


if __name__ == "__main__":
    main()
