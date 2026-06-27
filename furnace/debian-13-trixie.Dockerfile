# Furnace build image for the Rockchip RKMPP Sunshine fork — Debian 13 (trixie).
#
# Identical to furnace/debian-12-bookworm.Dockerfile except for the base image.
# The Radxa apt lists are the bookworm ones (Radxa has no trixie suite yet); the
# librockchip-mpp / librga packages install fine on trixie. See that file for the
# full rationale.
FROM debian:trixie

ENV DEBIAN_FRONTEND=noninteractive

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

RUN git clone --filter=blob:none https://github.com/nyanmisaka/ffmpeg-rockchip.git /opt/ffmpeg-rockchip \
    && git -C /opt/ffmpeg-rockchip checkout "${FFMPEG_ROCKCHIP_REF}"

WORKDIR /work
