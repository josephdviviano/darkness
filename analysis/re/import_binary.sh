#!/bin/sh
# Import + auto-analyze a Dark Engine binary into a Ghidra project, headless.
#
# Usage: sh analysis/re/import_binary.sh <project-name> <binary-path>
#
# Everything is scripted so a re-run reproduces the project from the binary and
# so the pipeline can be re-pointed at another exe version. Projects land in
# .claude/re/projects (gitignored); only this script is tracked.
#
# See .claude/PLAN.BINARY_RE_PIPELINE.md.
set -e

GHIDRA_HOME=/opt/homebrew/Cellar/ghidra/12.1.2/libexec
HEADLESS="$GHIDRA_HOME/support/analyzeHeadless"
# NOTE: Ghidra rejects any path element starting with '.' ("Path element
# starting with '.' is not permitted"), so the project CANNOT live under
# .claude/. It also must not live inside the repo — these projects run to
# hundreds of MB. Keep them a sibling of the repo, like the other references.
PROJDIR=/Users/jdv/code/darkness/re_projects

# Homebrew's openjdk@21 is KEG-ONLY: it is never symlinked into the system JVM
# path, so /usr/libexec/java_home cannot see it and Ghidra's launcher dies with
# "Unable to locate a Java Runtime" + "no TTY detected". Point JAVA_HOME at the
# keg explicitly rather than relying on the user's shell environment.
if [ -z "$JAVA_HOME" ]; then
    for candidate in \
        /opt/homebrew/opt/openjdk@21/libexec/openjdk.jdk/Contents/Home \
        /opt/homebrew/opt/openjdk/libexec/openjdk.jdk/Contents/Home
    do
        if [ -x "$candidate/bin/java" ]; then
            JAVA_HOME="$candidate"
            export JAVA_HOME
            break
        fi
    done
fi
if [ -z "$JAVA_HOME" ]; then
    echo "[RE] no JDK found. Install with: brew install openjdk@21" >&2
    exit 1
fi
echo "[RE] JAVA_HOME: $JAVA_HOME"

if [ $# -lt 2 ]; then
    echo "usage: $0 <project-name> <binary-path>" >&2
    exit 2
fi

PROJNAME="$1"
BINARY="$2"

if [ ! -f "$BINARY" ]; then
    echo "[RE] binary not found: $BINARY" >&2
    exit 1
fi
if [ ! -x "$HEADLESS" ]; then
    echo "[RE] analyzeHeadless not found at $HEADLESS" >&2
    echo "[RE] install with: brew install ghidra   (FORMULA, not --cask)" >&2
    exit 1
fi

mkdir -p "$PROJDIR"

echo "[RE] project : $PROJNAME"
echo "[RE] binary  : $BINARY"
echo "[RE] projdir : $PROJDIR"
echo "[RE] starting headless import + auto-analysis"

"$HEADLESS" "$PROJDIR" "$PROJNAME" \
    -import "$BINARY" \
    -analysisTimeoutPerFile 7200 \
    -loader PeLoader

echo "[RE] done: $PROJNAME"
