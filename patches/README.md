# Local patches for third-party submodules

Patches here fix bugs in submodules whose upstreams we do not control. They
must be re-applied after any `git submodule update`, which checks out the
pinned upstream commit and discards working-tree changes.

Apply (idempotent — `--check` first, skip if already applied):

```bash
p=patches/inputtino-ds5-motion-negative-values.patch
git -C third-party/inputtino apply --check "$p" 2>/dev/null \
  && git -C third-party/inputtino apply "$p"
```

`init.sh` (retro-stream-home) and `~/bin/build-sunshine-arm` both do this
automatically before building.

Proper fix: fork the submodule (e.g. NeilGraham/inputtino), land the patch
there, and point `.gitmodules` at the fork — then delete the patch here.

| patch | submodule | what it fixes |
| --- | --- | --- |
| `inputtino-ds5-motion-negative-values.patch` | third-party/inputtino | DS5 gyro/accel negatives saturated to 0 in `to_le_signed` (half the motion range lost) |

## ffmpeg-rockchip patches (applied in the builder image, not here)

`ffmpeg-rockchip-*.patch` target `/opt/ffmpeg-rockchip` inside the furnace
Dockerfiles (both bookworm and trixie), which apply **and commit** them so
`build_ffmpeg_rkmpp.sh`'s revision stamp invalidates the cached ffmpeg build.
They are re-generated against `FFMPEG_ROCKCHIP_REF` when that pin bumps.
Order matters: `rkmpp-osd` is generated on top of `nv15-main10`.

| patch | what it adds |
| --- | --- |
| `ffmpeg-rockchip-nv15-main10.patch` | NV15 input + HEVC Main 10 (10-bit HDR) |
| `ffmpeg-rockchip-rkmpp-osd.patch` | `AV_FRAME_DATA_RKMPP_OSD` side data → VEPU580 hardware OSD (`KEY_OSD_DATA2`); struct ABI mirrors `retro-overlay-protocol.h` — keep byte-identical in one commit |
