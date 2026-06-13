FROM --platform=linux/arm64/v8 debian:12

ENV DEBIAN_FRONTEND=noninteractive

COPY docker/radxa-apt/*.list /etc/apt/sources.list.d/
COPY docker/radxa-apt/radxa-archive-keyring.gpg /usr/share/keyrings/radxa-archive-keyring.gpg

RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates \
      curl \
      git \
      gnupg \
      sudo \
    && curl -fsSL https://deb.nodesource.com/setup_22.x | bash - \
    && apt-get install -y --no-install-recommends nodejs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work/SUNSHINE-ROCKCHIP-INTEGRATION/Sunshine-ARM

CMD ["./scripts/linux_build.sh", "--sudo-off", "--skip-cuda", "--rkmpp=../ffmpeg-rockchip"]
