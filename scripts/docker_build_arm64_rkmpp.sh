#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
sunshine_dir="$(cd "${script_dir}/.." >/dev/null 2>&1 && pwd -P)"
integration_dir="$(cd "${sunshine_dir}/.." >/dev/null 2>&1 && pwd -P)"
apt_dir="${sunshine_dir}/docker/radxa-apt"
image_name="sunshine-radxa-bookworm-arm64-rkmpp"
docker_context="$(mktemp -d)"
artifact_dir="${sunshine_dir}/build/docker-artifacts"
build_jobs="${SUNSHINE_DOCKER_BUILD_JOBS:-2}"
trap 'rm -rf "$docker_context"' EXIT

missing=0
for file in \
  "${apt_dir}/70-radxa.list" \
  "${apt_dir}/80-radxa-rk3566.list" \
  "${apt_dir}/radxa-archive-keyring.gpg"; do
  if [ ! -f "$file" ]; then
    echo "Missing $file" >&2
    missing=1
  fi
done

if [ "$missing" -ne 0 ]; then
  cat >&2 <<EOF

Copy these from the Rock 3C first:
  /etc/apt/sources.list.d/70-radxa.list
  /etc/apt/sources.list.d/80-radxa-rk3566.list
  /usr/share/keyrings/radxa-archive-keyring.gpg

Put them in:
  ${apt_dir}

EOF
  exit 1
fi

mkdir -p "${docker_context}/docker/radxa-apt"
cp "${sunshine_dir}/docker/radxa-bookworm-arm64-rkmpp.Dockerfile" "${docker_context}/"
cp "${apt_dir}/"*.list "${docker_context}/docker/radxa-apt/"
cp "${apt_dir}/radxa-archive-keyring.gpg" "${docker_context}/docker/radxa-apt/"
mkdir -p "$artifact_dir"

docker build \
  --platform linux/arm64/v8 \
  -f "${docker_context}/radxa-bookworm-arm64-rkmpp.Dockerfile" \
  -t "$image_name" \
  "$docker_context"

docker run --rm \
  --platform linux/arm64/v8 \
  -v "${integration_dir}:/src/SUNSHINE-ROCKCHIP-INTEGRATION:ro" \
  -v "${artifact_dir}:/out" \
  -e "SUNSHINE_DOCKER_BUILD_JOBS=${build_jobs}" \
  "$image_name" \
  bash -lc '
    set -euo pipefail

    mkdir -p /work/SUNSHINE-ROCKCHIP-INTEGRATION
    cd /src/SUNSHINE-ROCKCHIP-INTEGRATION
    tar \
      --exclude=Sunshine-ARM/build \
      --exclude=Sunshine-ARM/node_modules \
      --exclude=Sunshine-ARM/package-lock.json \
      --exclude=Sunshine-ARM/npm-shrinkwrap.json \
      -cf - . | tar -C /work/SUNSHINE-ROCKCHIP-INTEGRATION -xf -

    cd /work/SUNSHINE-ROCKCHIP-INTEGRATION/Sunshine-ARM
    ./scripts/linux_build.sh --sudo-off --skip-cuda --num-processors="${SUNSHINE_DOCKER_BUILD_JOBS}" --rkmpp=../ffmpeg-rockchip

    if [ -d build/cpack_artifacts ]; then
      cp -a build/cpack_artifacts/. /out/
    fi

    find build -maxdepth 4 -type f \( -name "*.deb" -o -name "*.rpm" \) -exec cp -v {} /out/ \;
  '

echo "Artifacts available on host at: ${artifact_dir}"
