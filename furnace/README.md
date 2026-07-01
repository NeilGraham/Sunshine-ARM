# Furnace build for the Rockchip RKMPP Sunshine fork

This delegates the ARM64 RKMPP build to the `mbp` SSH host (which has Docker),
the same way `~/retro-stream/furnace` delegates the retro-stream build. It
replaces the older manual flow (`Sunshine-Rockchip-Build/scripts/docker_build_arm64_rkmpp.sh`
run directly on `mbp`, then `scp` to the Rock 3C).

## One-shot build + deploy

```bash
cd ~/Sunshine-ARM
furnace run                       # build on mbp (Debian 12), pull back the .deb
# -> build/cpack_artifacts/Sunshine.deb

# install on this Rock 3C:
sudo apt install ./build/cpack_artifacts/Sunshine.deb
sudo systemctl restart sunshine
```

Build Debian 13 instead of 12:

```bash
furnace run --image furnace/debian-13-trixie.Dockerfile
```

## Files

- `config.toml` — target `mbp`, Docker on, builds via `scripts/linux_build.sh`,
  pulls back `build/cpack_artifacts/Sunshine.deb`.
- `debian-12-bookworm.Dockerfile` / `debian-13-trixie.Dockerfile` — build images.
  furnace auto-selects the one matching the inferred platform; override with
  `--image`. Each sets up the Radxa apt repo and bakes `ffmpeg-rockchip` (pinned
  to `ARG FFMPEG_ROCKCHIP_REF`) into `/opt/ffmpeg-rockchip`, plus the package
  dependencies needed by `scripts/linux_build.sh`.
- `radxa-apt/` — Radxa apt sources + keyring, copied into the image so
  `librockchip-mpp-dev` / `librga` resolve.

## Notes

- **Architecture must match.** furnace requires the source (Rock 3C, arm64) and
  builder (`mbp`, Apple-silicon arm64) to share an architecture — they do.
- **Caching.** The first build compiles `ffmpeg-rockchip` into
  `build/ffmpeg-rkmpp`, seeds `build/.ccache`, and fills `build/npm-cache`.
  furnace protects the gitignored `build/` on the remote, so subsequent runs
  skip unchanged RKMPP FFmpeg work and use compiler/npm caches.
- **Bumping ffmpeg-rockchip.** Change `FFMPEG_ROCKCHIP_REF` in both Dockerfiles
  and run `furnace run --clean` once to rebuild the image and the ffmpeg cache.
- The build deps (boost, cmake, ninja, openssl, …) are installed in the Docker
  image. The furnace command passes `--skip-deps` to avoid repeating `apt-get`
  work on every run.
- **Non-root build user.** furnace runs the container as the build host's
  `uid:gid` (mbp `grahamneiln` = `501:20`), but `linux_build.sh` needs root for
  apt and `/usr/local`. So the images create a matching user (`ARG BUILD_UID` /
  `BUILD_GID`) with passwordless sudo, the build runs *without* `--sudo-off`, and
  the command sets `HOME=/home/builder` (furnace's `HOME=/tmp` has no `.bashrc`,
  which `linux_build.sh` sources). If you ever build from a host with a different
  uid, override `--build-arg BUILD_UID=…` (or just rebuild on mbp where it's 501).
