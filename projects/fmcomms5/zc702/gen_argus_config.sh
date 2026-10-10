#!/usr/bin/env bash
#
# gen_argus_config.sh -- emit the generated block of argus/config.py
#
# Lives in and runs from the RTL repo. Run it immediately after bitgen, then
# paste its stdout over the generated block in the Python repo's
# argus/config.py.
#
#   ./gen_argus_config.sh -b build/system_top.bit
#   ./gen_argus_config.sh -C ~/w/rtl-repo -b .../system_top.bit
#
# Why a bitstream argument: the SHA that matters is the one HEAD pointed at
# when the bitstream was built, not wherever HEAD is now. Given the .bit, this
# script compares its mtime against the last commit and refuses to pretend if
# the repo has moved on since. Without -b it can only report HEAD and trust
# that you are running it at the right moment.
#
# Exit 0 clean, 1 if any warning was emitted (the block is still printed, with
# the warning inlined so it survives the paste).

set -euo pipefail

REPO="."
BIT=""
SYSID_VERSION="0x00010161"      # ADI axi_sysid core; changes only with the core

usage() {
    sed -n '2,/^set -euo/p' "$0" | sed 's/^# \{0,1\}//; $d'
    exit "${1:-0}"
}

while getopts ":C:b:v:h" opt; do
    case "$opt" in
        C) REPO="$OPTARG" ;;
        b) BIT="$OPTARG" ;;
        v) SYSID_VERSION="$OPTARG" ;;
        h) usage 0 ;;
        \?) echo "unknown option -$OPTARG" >&2; usage 1 ;;
        :)  echo "option -$OPTARG needs an argument" >&2; usage 1 ;;
    esac
done

die() { echo "gen_argus_config: $*" >&2; exit 2; }

git -C "$REPO" rev-parse --git-dir >/dev/null 2>&1 \
    || die "'$REPO' is not a git repository"

SHA=$(git -C "$REPO" rev-parse HEAD)
BRANCH=$(git -C "$REPO" rev-parse --abbrev-ref HEAD)
COMMIT_EPOCH=$(git -C "$REPO" log -1 --format=%ct)
COMMIT_UTC=$(date -u -d "@$COMMIT_EPOCH" '+%Y-%m-%d %H:%M:%S UTC')

WARNINGS=()

# The RTL tree must be clean, or the SHA does not describe what was built.
if [ -n "$(git -C "$REPO" status --porcelain --untracked-files=no)" ]; then
    WARNINGS+=("RTL tree was DIRTY -- this SHA does not fully describe the build")
fi

BUILT_LINE="# built: unknown (no -b <bitstream> given); HEAD committed $COMMIT_UTC"

if [ -n "$BIT" ]; then
    [ -f "$BIT" ] || die "no such bitstream: $BIT"
    BIT_EPOCH=$(stat -c %Y "$BIT")
    BIT_UTC=$(date -u -d "@$BIT_EPOCH" '+%Y-%m-%d %H:%M:%S UTC')
    BUILT_LINE="# built: $BIT_UTC   $(basename "$BIT")"

    # If you committed after building, HEAD is not what went into the .bit.
    if [ "$COMMIT_EPOCH" -gt "$BIT_EPOCH" ]; then
        WARNINGS+=("HEAD was committed AFTER the bitstream was written --"
                   "the SHA below is almost certainly NOT the one in the sysid ROM."
                   "Rebuild, or check out the commit the .bit came from.")
    fi
fi

# ---------------------------------------------------------------------------

echo "# --- generated block: paste from gen_argus_config.sh -----------------------"
echo "#"
echo "# generated $(date -u '+%Y-%m-%d %H:%M:%S UTC') from $(cd "$REPO" && pwd)"
if [ "${#WARNINGS[@]}" -gt 0 ]; then
    for w in "${WARNINGS[@]}"; do echo "# WARNING: $w"; done
fi
echo
echo "# ADI axi_sysid core version. Changes only when the ADI core itself changes."
echo "SYSID_VERSION = $SYSID_VERSION          # verify with: argus_version.py --host"
echo
echo "# 40-char lowercase git SHA-1 recorded in the sysid ROM at build time."
echo "GIT_SHA = \"$SHA\""
echo
echo "$BUILT_LINE   branch $BRANCH"
echo
echo "# --- end generated block ---------------------------------------------------"

if [ "${#WARNINGS[@]}" -gt 0 ]; then
    echo >&2
    for w in "${WARNINGS[@]}"; do echo "WARNING: $w" >&2; done
    exit 1
fi
exit 0
