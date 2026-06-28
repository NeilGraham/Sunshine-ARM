# Handoff: Hardware JPEG decode (mjpeg_rkmpp) on RK3566 / Rock 3C

Goal: make the capture pipeline decode the USB capture card's **MJPEG** on the
RK3566/RK3588 hardware JPEG unit (`mjpeg_rkmpp`) → NV12, instead of the CPU
software decoder. (This is JPEG **decode**. H.264/HEVC **encode** is already
hardware via `h264_rkmpp` and works.)

Status: **NOT working** — `mjpeg_rkmpp` *hangs* (blocks, does not error) on the
first decode of this card's stream. Default config is therefore pinned to a safe
format. This doc is how to pick the work back up.

---

## 0. Current state / safety

- Safe runtime config: `retro-stream.toml` → `[capture] format = "yuyv422"`.
  Web UI (`:47990`) + Moonlight (`:47984/:47989`) come up; YUYV→NV12 via swscale.
- The **deployed binary still prefers HW MJPEG** (the hanging path). It is only
  safe because the format is pinned to a non-MJPEG value. Setting
  `format = "mjpeg"` or `"auto"` (auto picks MJPG first) will hang Sunshine on
  startup again. Keep it `yuyv422` until the HW path is fixed or made opt-in.
- Repo: `~/Sunshine-ARM`, branch `feature/rockchip-bookworm-latest`.
  HEAD `9cf65532` = multi-format negotiation + HW-MJPEG-preference + force-format.
  (mbp build tree `~/Sunshine-Rockchip-Build/Sunshine-ARM` mirrors it at `c641607a`.)

### CRITICAL verification lesson
The log line `RKMPP direct V4L2: capturing /dev/video0 as MJPEG(HW-decode)->NV12`
is printed in `init()` **before any frame is decoded** — it is NOT proof the HW
decoder works. The hang happens later, in the first `decode_mjpeg_to_nv12()`.
**Always verify with `ss -ltnp | grep 47990`** (web UI listening) after a restart,
not just the capture log line. If `:47990` is absent and the log ends at
`Using RKMPP direct V4L2 encode path`, the decoder is hung.

---

## 1. Build + deploy workflow (furnace, from ~/Sunshine-ARM)

Builds are delegated to the **`mbp`** SSH host (it has Docker; the Rock 3C does
not). furnace syncs this working tree to mbp, builds in a Debian image, pulls the
`.deb` back.

```bash
cd ~/Sunshine-ARM
furnace run                          # Debian 12 (default). ~15 min first time.
# pulls -> build/cpack_artifacts/Sunshine.deb

# deploy on this Rock 3C (use dpkg -i, NOT apt — the version string often
# doesn't change so apt will skip it):
sudo dpkg -i build/cpack_artifacts/Sunshine.deb
sudo systemctl restart sunshine-retro.service
sleep 4 && ss -ltnp | grep 47990     # <-- MUST show the web UI listening
```

Debian 13: `furnace run --image furnace/debian-13-trixie.Dockerfile`.

### How the furnace setup works (already configured in `furnace/`)
- `furnace/config.toml` — target `mbp`, docker on, command
  `export HOME=/home/builder && ./scripts/linux_build.sh --skip-cuda --num-processors=4 --rkmpp=/opt/ffmpeg-rockchip`,
  pulls `build/cpack_artifacts/Sunshine.deb`.
- `furnace/debian-12-bookworm.Dockerfile` / `debian-13-trixie.Dockerfile` —
  Radxa apt repo + node + a uid-501 build user with passwordless sudo, and
  `ffmpeg-rockchip` baked at `/opt/ffmpeg-rockchip` (pinned `ARG FFMPEG_ROCKCHIP_REF`).
- `furnace/radxa-apt/` — Radxa sources + keyring.
- `furnace/README.md` — full rationale.

### furnace gotchas already solved (don't re-discover them)
1. Container runs as **non-root uid 501** (mbp's uid). `linux_build.sh` needs
   root → the image gives uid 501 passwordless sudo and we drop `--sudo-off`.
2. furnace sets `HOME=/tmp` (no `.bashrc`); `linux_build.sh` does `source ~/.bashrc`
   → command sets `HOME=/home/builder`.
3. **Submodules must be initialized in this working tree** (furnace syncs the tree,
   not mbp's prepared one): `git submodule update --init --recursive` (done once).
4. First build compiles `ffmpeg-rockchip`; furnace's persistent `build/` caches it,
   so later builds only recompile Sunshine. Use `furnace run --clean` to reset.

### Fallback build (proven, runs as root on mbp, if furnace misbehaves)
```bash
scp src/platform/linux/rkmpp.cpp mbp:Sunshine-Rockchip-Build/Sunshine-ARM/src/platform/linux/rkmpp.cpp
ssh mbp 'export PATH=/usr/local/bin:$PATH; cd ~/Sunshine-Rockchip-Build && ./scripts/docker_build_arm64_rkmpp.sh'
scp mbp:Sunshine-Rockchip-Build/build/docker-artifacts/Sunshine.deb /tmp/Sunshine.deb
sudo dpkg -i /tmp/Sunshine.deb && sudo systemctl restart sunshine-retro.service
```

---

## 2. The file to focus on

**`src/platform/linux/rkmpp.cpp`** — class `v4l2_nv12_source_t`. Relevant members:

- `negotiate_format(w,h)` — preference-ordered V4L2 `S_FMT` (NV12, MJPG, YUYV,
  YU12, BGR3); honors `SUNSHINE_RKMPP_V4L2_FORMAT` (`parse_forced_format`).
- `open_mjpeg_decoder(w,h)` — currently prefers `avcodec_find_decoder_by_name("mjpeg_rkmpp")`
  (HW) then SW `AV_CODEC_ID_MJPEG`. Sets `mjpeg_ctx->width/height` before
  `avcodec_open2` (RKMPP needs dims).
- `mjpeg_decode_frame()` — `av_packet` → `avcodec_send_packet` → `avcodec_receive_frame`.
  **This is where it hangs** on HW.
- `decode_mjpeg_to_nv12()` — wraps the above with a runtime HW→SW fallback, plus
  DRM_PRIME → NV12 download (`av_hwframe_transfer_data`) and the planar→NV12
  interleave. The fallback only catches *failures*, not *hangs*.

Config plumbing (separate repo `~/retro-stream`, uncommitted):
- `src/main.rs` `--print-capture-format` → prints `[capture].format`.
- `install.sh` sets `SUNSHINE_RKMPP_V4L2_FORMAT="$(retro-stream --print-capture-format)"`
  in the unit (already live-patched into `/etc/systemd/system/sunshine-retro.service`).

---

## 3. What's known about the hang

- The card emits **complete JPEGs WITH Huffman tables** (verified: `SOI DQT DQT
  SOF0 DHT×4 SOS EOI`). So it is **not** a missing-DHT problem.
- Hang is in `avcodec_send_packet`/`avcodec_receive_frame` of `mjpeg_rkmpp` on the
  first frame, during Sunshine's **startup encoder probe**.
- Upstream context: Rockchip MPP issues #482 and #586 — MPP's MJPEG decode on
  USB/UVC streams blocks; MJPEG decode reportedly needs the MPP **advanced API**.
- The fork authors already abandoned HW for this reason (commit `ab008ad4`,
  "prefer SW mjpeg decoder — UVC MJPEG fails hardware parser").

### Capture a test frame for offline experiments
```bash
v4l2-ctl -d /dev/video0 --set-fmt-video=width=1920,height=1080,pixelformat=MJPG \
  --stream-mmap --stream-count=1 --stream-to=/tmp/frame.mjpg
```

---

## 4. Research directions (most promising first)

1. **Isolate it.** Write a tiny C program that links the built ffmpeg-rockchip
   (`build/ffmpeg-rkmpp`) and decodes `/tmp/frame.mjpg` with `mjpeg_rkmpp`,
   printing where it blocks. Removes Sunshine from the loop. Build it inside the
   same Docker image (it has the rkmpp libs). This is the fastest signal.
2. **Is it the decode or the DRM_PRIME download?** `mjpeg_rkmpp` outputs
   `AV_PIX_FMT_DRM_PRIME`; the hang may be in `av_hwframe_transfer_data`, not the
   decode. Try forcing a non-PRIME output, or time each step. Consider setting
   `mjpeg_ctx->pix_fmt`/a `get_format` callback to request NV12 directly.
3. **Probe context.** Sunshine probes encoders at startup, possibly before the
   card streams steadily. Try deferring `v4l2_nv12_source_t` creation / first HW
   decode until real frames are flowing, so the probe never touches HW. (Look at
   where `set_frame`/`convert` run during probe vs stream.)
4. **MPP buffer group / advanced API.** Per #586, MPP MJPEG may need explicit
   external buffer setup. Check whether the ffmpeg-rockchip `mjpeg_rkmpp` wrapper
   exposes options (`-init_hw_device`, `extra_hw_frames`, buffer count) and set
   them on `mjpeg_ctx` via `av_opt_set`.
5. **mjpeg2jpeg / JFIF.** Even though DHT is present, MPP may want a JFIF APP0.
   Enable `--enable-bsf=mjpeg2jpeg` in `scripts/build_ffmpeg_rkmpp.sh` (the configure
   line currently enables only `h264_metadata`/`hevc_metadata` bsfs) and run the
   packets through the `mjpeg2jpeg` BSF before decode.
6. **Versions.** Try a newer `ffmpeg-rockchip` ref and/or newer `librockchip-mpp`
   (Radxa repo) — the MJPEG decoder has changed over time.
7. **Watchdog fallback (pragmatic).** If HW can't be made reliable, run the first
   decode on a worker thread with a timeout; on timeout, mark HW unusable and use
   SW for the session. Interrupting a blocked MPP call is fragile — treat as last
   resort and never block Sunshine's startup.

### Safety while testing
Make the HW path **explicit opt-in** first, so a hang never bricks startup:
add a `mjpeg-hw` token to `parse_forced_format()` (only that token enables
`mjpeg_rkmpp`; plain `mjpeg` stays SW), keep `auto` preferring YUYV over MJPG.
Then test with `[capture] format = "mjpeg-hw"` and **always** check
`ss -ltnp | grep 47990` after restart.

---

## 5. Definition of done
`[capture] format = "mjpeg-hw"` (or auto) → restart → `:47990` listening →
log shows `MJPEG(HW-decode)->NV12` AND a Moonlight client gets live video with
CPU usage noticeably lower than the YUYV/SW path (compare `top` during stream).
