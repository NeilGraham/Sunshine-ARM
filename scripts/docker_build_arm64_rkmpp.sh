#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
sunshine_dir="$(readlink -f "${script_dir}/..")"
integration_dir="$(readlink -f "${sunshine_dir}/..")"
apt_dir="${sunshine_dir}/docker/radxa-apt"
image_name="sunshine-radxa-bookworm-arm64-rkmpp"

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

docker build \
  --platform linux/arm64/v8 \
  -f "${sunshine_dir}/docker/radxa-bookworm-arm64-rkmpp.Dockerfile" \
  -t "$image_name" \
  "$sunshine_dir"

docker run --rm \
  --platform linux/arm64/v8 \
  -v "${integration_dir}:/work/SUNSHINE-ROCKCHIP-INTEGRATION" \
  "$image_name"
