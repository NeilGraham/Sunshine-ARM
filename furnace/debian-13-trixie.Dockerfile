# Furnace build image for the Rockchip RKMPP Sunshine fork — Debian 13 (trixie).
#
# Identical to furnace/debian-12-bookworm.Dockerfile except for the base image.
# The Radxa apt lists are the bookworm ones (Radxa has no trixie suite yet); the
# librockchip-mpp / librga packages install fine on trixie. See that file for the
# full rationale.
FROM debian:trixie

ENV DEBIAN_FRONTEND=noninteractive

ARG FFMPEG_ROCKCHIP_REF=40c412dacc

# See debian-12-bookworm.Dockerfile for the rationale; furnace runs as the build
# host's uid:gid (mbp = 501:20) and linux_build.sh needs root via sudo.
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
      g++-14 \
      gcc-14 \
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
      systemd-dev \
      udev \
      wget \
      xvfb \
    && rm -rf /var/lib/apt/lists/*

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

# Build user matching furnace's --user uid:gid, with passwordless sudo.
RUN (getent group "${BUILD_GID}" || groupadd -g "${BUILD_GID}" builder) \
    && useradd -o -u "${BUILD_UID}" -g "${BUILD_GID}" -m -s /bin/bash builder \
    && echo 'builder ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/builder \
    && chmod 0440 /etc/sudoers.d/builder \
    && chown -R "${BUILD_UID}:${BUILD_GID}" /opt/ffmpeg-rockchip

WORKDIR /work
