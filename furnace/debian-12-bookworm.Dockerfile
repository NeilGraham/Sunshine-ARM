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
# The Debian build dependencies are installed here so repeated furnace runs can
# skip linux_build.sh's apt step and go straight to configure/build/package.
#
# Build context is the project root (Sunshine-ARM); COPY paths are relative to it.
FROM debian:12

ENV DEBIAN_FRONTEND=noninteractive

# ffmpeg-rockchip commit baked into the image (github.com/nyanmisaka/ffmpeg-rockchip).
ARG FFMPEG_ROCKCHIP_REF=40c412dacc

# furnace runs the container as the build host's uid:gid (mbp grahamneiln =
# 501:20). linux_build.sh needs root (apt + writes to /usr/local), so create a
# matching user with passwordless sudo and run the build with sudo (no
# --sudo-off). Override BUILD_UID/BUILD_GID if building from a different host.
ARG BUILD_UID=501
ARG BUILD_GID=20

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
      appstream \
      appstream-util \
      bison \
      build-essential \
      ccache \
      cmake \
      desktop-file-utils \
      doxygen \
      file \
      flex \
      g++-12 \
      gcc-12 \
      glslang-tools \
      graphviz \
      libcurl4-openssl-dev \
      libdrm-dev \
      libgbm-dev \
      libevdev-dev \
      libfmt-dev \
      libminiupnpc-dev \
      libnotify-dev \
      libnuma-dev \
      libopus-dev \
      libpipewire-0.3-dev \
      libpulse-dev \
      librockchip-mpp-dev \
      librga-dev \
      librga2 \
      libssl-dev \
      libsystemd-dev \
      libudev-dev \
      libva-dev \
      libvulkan-dev \
      libwayland-dev \
      libx11-dev \
      libxcb-shm0-dev \
      libxcb-xfixes0-dev \
      libxcb1-dev \
      libxfixes-dev \
      libxrandr-dev \
      libxtst-dev \
      ninja-build \
      python3-jinja2 \
      python3-setuptools \
      qt6-base-dev \
      qt6-svg-dev \
      systemd \
      udev \
      wget \
      xvfb \
    && rm -rf /var/lib/apt/lists/*

# Bake the ffmpeg-rockchip source at a pinned commit; linux_build.sh --rkmpp
# points here. (A shallow fetch of the exact commit keeps the image small.)
RUN git clone --filter=blob:none https://github.com/nyanmisaka/ffmpeg-rockchip.git /opt/ffmpeg-rockchip \
    && git -C /opt/ffmpeg-rockchip checkout "${FFMPEG_ROCKCHIP_REF}"

# 10-bit HDR: teach the rkmpp encoder NV15 input + HEVC Main 10 (upstream has
# neither at this ref). Committed (not just applied) so build_ffmpeg_rkmpp.sh's
# revision stamp changes and the cached ffmpeg build is invalidated.
COPY patches/ffmpeg-rockchip-nv15-main10.patch /tmp/nv15-main10.patch
RUN git -C /opt/ffmpeg-rockchip apply /tmp/nv15-main10.patch \
    && git -C /opt/ffmpeg-rockchip -c user.name=furnace -c user.email=furnace@local \
       commit -am "rkmppenc: NV15 input + HEVC Main 10 (retro-stream 10-bit HDR)" \
    && rm /tmp/nv15-main10.patch

# Stream overlay: VEPU580 hardware OSD via AV_FRAME_DATA_RKMPP_OSD side data
# (retro-overlay daemon -> Sunshine client -> encoder; blend happens inside
# the encode pass). Generated against the nv15 patch above — keep this apply
# AFTER it. Committed for the same revision-stamp reason.
COPY patches/ffmpeg-rockchip-rkmpp-osd.patch /tmp/rkmpp-osd.patch
RUN git -C /opt/ffmpeg-rockchip apply /tmp/rkmpp-osd.patch \
    && git -C /opt/ffmpeg-rockchip -c user.name=furnace -c user.email=furnace@local \
       commit -am "rkmppenc: hardware OSD side data (retro-stream overlay)" \
    && rm /tmp/rkmpp-osd.patch

# Build user matching furnace's --user uid:gid, with passwordless sudo so the
# (non-root) furnace container can still apt-install and write to /usr/local.
RUN (getent group "${BUILD_GID}" || groupadd -g "${BUILD_GID}" builder) \
    && useradd -o -u "${BUILD_UID}" -g "${BUILD_GID}" -m -s /bin/bash builder \
    && echo 'builder ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/builder \
    && chmod 0440 /etc/sudoers.d/builder \
    && chown -R "${BUILD_UID}:${BUILD_GID}" /opt/ffmpeg-rockchip

WORKDIR /work
