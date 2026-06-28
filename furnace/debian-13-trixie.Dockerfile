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
      libdrm-dev \
      libgbm-dev \
      librockchip-mpp-dev \
      librga-dev \
      librga2 \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --filter=blob:none https://github.com/nyanmisaka/ffmpeg-rockchip.git /opt/ffmpeg-rockchip \
    && git -C /opt/ffmpeg-rockchip checkout "${FFMPEG_ROCKCHIP_REF}"

# Build user matching furnace's --user uid:gid, with passwordless sudo.
RUN (getent group "${BUILD_GID}" || groupadd -g "${BUILD_GID}" builder) \
    && useradd -o -u "${BUILD_UID}" -g "${BUILD_GID}" -m -s /bin/bash builder \
    && echo 'builder ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/builder \
    && chmod 0440 /etc/sudoers.d/builder \
    && chown -R "${BUILD_UID}:${BUILD_GID}" /opt/ffmpeg-rockchip

WORKDIR /work
