#!/bin/bash
# check-protocol-sync — fail if a vendored protocol header has drifted.
#
# WHY THIS EXISTS
#   Sunshine used to carry hand-copied copies of both protocol headers, and
#   both had silently drifted: its retro-capture-protocol.h was a whole
#   protocol version behind (v2 vs v3, missing DISPLAY_SET/DISPLAY_ACK), and
#   its retro-overlay-protocol.h had `uint8_t reserved[3]` where the owner had
#   split out `gate_window_ms`. Nothing broke only because version negotiation
#   covered one and the struct sizes happened to still match for the other.
#   That was luck. This makes the next drift loud instead of lucky.
#
#   retro-capture-protocol.h is owned by this repo (../include) — the client
#   includes it directly, so it cannot drift.
#
#   retro-overlay-protocol.h is owned by retro-overlay, which lives in the
#   42 MB retro-stream workspace — too heavy to submodule for one header, so
#   this is a copy and needs checking.
#
# Usage: ./check-protocol-sync.sh [path-to-retro-overlay-checkout]
#   Skips (exit 0) when the owning checkout is not present, so it is safe in
#   CI and on machines that only have this repo.
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
owner_dir="${1:-$HOME/retro-stream/retro-overlay}"
owner="$owner_dir/include/retro-overlay-protocol.h"
copy="$here/include/retro-overlay-protocol.h"

if [[ ! -f $owner ]]; then
    echo "check-protocol-sync: $owner not present; skipping"
    exit 0
fi

if diff -q "$owner" "$copy" >/dev/null; then
    echo "check-protocol-sync: retro-overlay-protocol.h in sync"
    exit 0
fi

echo "check-protocol-sync: DRIFT — retro-overlay-protocol.h differs from its owner" >&2
diff -u "$owner" "$copy" >&2 || true
echo >&2
echo "retro-overlay owns this header. Copy it forward and re-check the ABI:" >&2
echo "  cp '$owner' '$copy'" >&2
echo "Note the same struct layout is mirrored a third time inside" >&2
echo "furnace/jobs/sunshine-arm/patches/ffmpeg-rockchip-rkmpp-osd.patch —" >&2
echo "update that in the same commit or the encoder reads the wrong bytes." >&2
exit 1
