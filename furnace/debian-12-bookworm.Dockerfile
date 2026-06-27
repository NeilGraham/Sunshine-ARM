# Furnace build image for the Rockchip RKMPP Sunshine fork — Debian 12 (bookworm).
#
# Mirrors docker/radxa-*-arm64-rkmpp.Dockerfile from the Sunshine-Rockchip-Build
# integration repo, but is self-contained for `furnace run --ssh mbp`:
#   - Sets up the Radxa apt repo so librockchip-mpp / librga are installable.
#   - Bakes the ffmpeg-rockchip source at a pinned commit into /opt so the build
#     does not depend on a sibling checkout. linux_build.sh --rkmpp builds it
#     into build/ffmpeg-rkmpp (cached across runs by furnace's persistent remote
#     workspace, so only the first build pays the ffmpeg cost).
#
# The remaining build dependencies (boost, cmake, ninja, openssl, ...) are
# installed by scripts/linux_build.sh itself (apt-get as root; --sudo-off only
# drops the sudo prefix), so they are intentionally not listed here.
#
# Build context is the project root (Sunshine-ARM); COPY paths are relative to it.
FROM debian:12

ENV DEBIAN_FRONTEND=noninteractive

# ffmpeg-rockchip commit baked into the image (github.com/nyanmisaka/ffmpeg-rockchip).
ARG FFMPEG_ROCKCHIP_REF=40c412dacc

RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates \
      curl \
      git \
      gnupg \
      pkg-config \
      sudo \
    && curl -fsSL https://deb.nodesource.com/setup_22.x | bash - \
    && apt-get install -y --no-install-recommends nodejs \
    && rm -rf /var/lib/apt/lists/*

COPY furnace/radxa-apt/*.list /etc/apt/sources.list.d/
COPY furnace/radxa-apt/radxa-archive-keyring.gpg /usr/share/keyrings/radxa-archive-keyring.gpg

RUN apt-get update && apt-get install -y --no-install-recommends \
      libcap-dev \
      libdrm-dev \
      libgbm-dev \
      librockchip-mpp-dev \
      librga-dev \
      librga2 \
    && rm -rf /var/lib/apt/lists/*

# Bake the ffmpeg-rockchip source at a pinned commit; linux_build.sh --rkmpp
# points here. (A shallow fetch of the exact commit keeps the image small.)
RUN git clone --filter=blob:none https://github.com/nyanmisaka/ffmpeg-rockchip.git /opt/ffmpeg-rockchip \
    && git -C /opt/ffmpeg-rockchip checkout "${FFMPEG_ROCKCHIP_REF}"

WORKDIR /work
