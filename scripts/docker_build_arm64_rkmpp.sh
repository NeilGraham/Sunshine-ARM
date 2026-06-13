#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
sunshine_dir="$(cd "${script_dir}/.." >/dev/null 2>&1 && pwd -P)"
integration_dir="$(cd "${sunshine_dir}/.." >/dev/null 2>&1 && pwd -P)"
apt_dir="${sunshine_dir}/docker/radxa-apt"
image_name="sunshine-radxa-bookworm-arm64-rkmpp"
docker_context="$(mktemp -d)"
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

docker build \
  --platform linux/arm64/v8 \
  -f "${docker_context}/radxa-bookworm-arm64-rkmpp.Dockerfile" \
  -t "$image_name" \
  "$docker_context"

docker run --rm \
  --platform linux/arm64/v8 \
  -v "${integration_dir}:/work/SUNSHINE-ROCKCHIP-INTEGRATION" \
  "$image_name"
