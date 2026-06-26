#!/usr/bin/env bash
set -euo pipefail

source_dir=""
prefix=""
jobs="$(nproc)"

usage() {
  cat <<EOF
Usage: $0 --source /path/to/ffmpeg-rockchip --prefix /path/to/install [--jobs N]
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --source)
      source_dir="$2"
      shift 2
      ;;
    --source=*)
      source_dir="${1#*=}"
      shift
      ;;
    --prefix)
      prefix="$2"
      shift 2
      ;;
    --prefix=*)
      prefix="${1#*=}"
      shift
      ;;
    --jobs)
      jobs="$2"
      shift 2
      ;;
    --jobs=*)
      jobs="${1#*=}"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if [ -z "$source_dir" ] || [ -z "$prefix" ]; then
  usage >&2
  exit 1
fi

source_dir="$(readlink -f "$source_dir")"
prefix="$(readlink -m "$prefix")"
build_dir="$(readlink -m "${prefix}/../ffmpeg-rkmpp-build")"

if [ ! -x "${source_dir}/configure" ]; then
  echo "FFmpeg configure script not found in: ${source_dir}" >&2
  exit 1
fi

rm -rf "$build_dir" "$prefix"
mkdir -p "$build_dir" "$prefix"

pushd "$build_dir" >/dev/null
"${source_dir}/configure" \
  --prefix="$prefix" \
  --disable-shared \
  --enable-static \
  --disable-doc \
  --disable-debug \
  --enable-version3 \
  --disable-programs \
  --disable-avdevice \
  --disable-avfilter \
  --disable-avformat \
  --disable-postproc \
  --disable-swresample \
  --disable-everything \
  --enable-libdrm \
  --enable-rkmpp \
  --enable-decoder=mjpeg_rkmpp \
  --enable-decoder=mjpeg \
  --enable-encoder=h264_rkmpp \
  --enable-encoder=hevc_rkmpp \
  --enable-bsf=h264_metadata \
  --enable-bsf=hevc_metadata

make -j"$jobs"
make install

cbs_objects=()
while IFS= read -r object; do
  cbs_objects+=("$object")
done < <(find libavcodec -type f \( \
  -name 'cbs*.o' -o \
  -name 'h2645_parse.o' -o \
  -name 'h264_parse.o' -o \
  -name 'h264_ps.o' -o \
  -name 'h264_levels.o' -o \
  -name 'hevc_ps.o' \
\) | sort)

if [ "${#cbs_objects[@]}" -gt 0 ]; then
  ar rcs "${prefix}/lib/libcbs.a" "${cbs_objects[@]}"
else
  ar rcs "${prefix}/lib/libcbs.a"
fi
popd >/dev/null

mkdir -p "${prefix}/include/libavcodec" "${prefix}/include/libavutil"
find "${source_dir}/libavcodec" -maxdepth 1 -name '*.h' -exec cp -f {} "${prefix}/include/libavcodec/" \;
find "${source_dir}/libavutil" -maxdepth 1 -name '*.h' -exec cp -f {} "${prefix}/include/libavutil/" \;
cp -f "${build_dir}/config.h" "${prefix}/include/libavcodec/config.h"
cp -f "${build_dir}/config.h" "${prefix}/include/libavutil/config.h"

echo "Prepared RKMPP FFmpeg at ${prefix}"
