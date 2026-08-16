#!/bin/bash
# check-protocol-sync — fail if a vendored protocol header, or the third
# copy of the OSD ABI living inside the ffmpeg encoder patch, has drifted.
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
#   rovl_osd_side_data is mirrored a THIRD time inside the ffmpeg-rockchip
#   OSD patch as RKMPPOsdSideData: the encoder cannot include the overlay
#   header, so the layout is retyped there. That copy is what the VEPU
#   actually reads, and a silent mismatch means the encoder reads the wrong
#   bytes — so it is checked field for field, not just warned about.
#
# Usage: ./check-protocol-sync.sh [path-to-retro-overlay-checkout] [osd-patch]
#   Each check skips (exit 0) when its owning file is absent, so this is safe
#   in CI and on machines that only have this repo.
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
owner_dir="${1:-$HOME/retro-stream/packages/overlay}"
owner="$owner_dir/include/retro-overlay-protocol.h"
copy="$here/include/retro-overlay-protocol.h"
osd_patch="${2:-$HOME/furnace/jobs/sunshine-arm/patches/ffmpeg-rockchip-rkmpp-osd.patch}"

status=0

# ---- 1. the vendored overlay header vs its owner ----
if [[ ! -f $owner ]]; then
    echo "check-protocol-sync: $owner not present; skipping header check"
elif diff -q "$owner" "$copy" >/dev/null; then
    echo "check-protocol-sync: retro-overlay-protocol.h in sync"
else
    echo "check-protocol-sync: DRIFT — retro-overlay-protocol.h differs from its owner" >&2
    diff -u "$owner" "$copy" >&2 || true
    echo >&2
    echo "retro-overlay owns this header. Copy it forward and re-check the ABI:" >&2
    echo "  cp '$owner' '$copy'" >&2
    status=1
fi

# ---- 2. the OSD ABI retyped inside the ffmpeg encoder patch ----
# Compare the field sequence of rovl_osd_side_data (this repo's copy, which
# check 1 ties to the owner) against RKMPPOsdSideData in the patch. Names of
# the array-bound macros differ by design (ROVL_* vs RKMPP_OSD_*), so those
# are normalized to their numeric values before comparing.
extract_fields() {
    # $1 file, $2 struct-opening regex — prints "type name" per field, and
    # resolves the two array bounds to numbers.
    awk -v start="$2" '
        $0 ~ start { inside = 1 }
        inside {
            line = $0
            sub(/^\+/, "", line)                 # patch context marker
            sub(/\/\*.*\*\//, "", line)          # comments
            gsub(/[ \t]+/, " ", line)
            gsub(/^ | $/, "", line)
            if (line ~ /^\}/) { depth--; if (depth <= 0) exit }
            if (line ~ /\{/) { depth++; next }
            if (line ~ /;$/) print line
        }
    ' "$1" |
    sed -E -e 's/ROVL_MAX_REGIONS|RKMPP_OSD_MAX_REGIONS/8/g' \
           -e 's/ROVL_PALETTE_SIZE|RKMPP_OSD_PALETTE_SIZE/256/g' \
           -e 's/__attribute__\(\(packed\)\)|ROVL_PACKED//g' \
           -e 's/[ \t]+/ /g' -e 's/^ | $//g'
}

if [[ ! -f $osd_patch ]]; then
    echo "check-protocol-sync: $osd_patch not present; skipping OSD ABI check"
else
    owner_fields=$(extract_fields "$copy" 'struct .*rovl_osd_side_data')
    patch_fields=$(extract_fields "$osd_patch" 'struct.*RKMPPOsdSideData')
    if [[ -z $patch_fields ]]; then
        echo "check-protocol-sync: could not find RKMPPOsdSideData in $osd_patch" >&2
        status=1
    elif [[ "$owner_fields" == "$patch_fields" ]]; then
        echo "check-protocol-sync: OSD ABI (rovl_osd_side_data <-> RKMPPOsdSideData) in sync"
    else
        echo "check-protocol-sync: DRIFT — the OSD ABI in the ffmpeg patch does not match" >&2
        diff -u <(echo "$owner_fields") <(echo "$patch_fields") >&2 || true
        echo >&2
        echo "The encoder reads these bytes directly. Update RKMPPOsdSideData in" >&2
        echo "  $osd_patch" >&2
        echo "in the same commit as the header, then rebuild the builder image." >&2
        status=1
    fi
fi

exit $status
