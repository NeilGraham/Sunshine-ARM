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
stamp_file="${prefix}/.rkmpp-build-stamp"
source_revision="$(git -C "$source_dir" rev-parse HEAD 2>/dev/null || echo unknown)"

configure_args=(
  "--prefix=$prefix"
  "--disable-shared"
  "--enable-static"
  "--disable-doc"
  "--disable-debug"
  "--enable-version3"
  "--disable-programs"
  "--disable-avdevice"
  "--disable-avfilter"
  "--disable-avformat"
  "--disable-postproc"
  "--disable-swresample"
  "--disable-everything"
  "--enable-libdrm"
  "--enable-rkmpp"
  "--enable-decoder=mjpeg_rkmpp"
  "--enable-decoder=mjpeg"
  "--enable-encoder=h264_rkmpp"
  "--enable-encoder=hevc_rkmpp"
  "--enable-bsf=h264_metadata"
  "--enable-bsf=hevc_metadata"
)

if [ ! -x "${source_dir}/configure" ]; then
  echo "FFmpeg configure script not found in: ${source_dir}" >&2
  exit 1
fi

new_stamp="$(
  {
    printf 'source=%s\n' "$source_dir"
    printf 'revision=%s\n' "$source_revision"
    printf 'configure_args='
    printf '%q ' "${configure_args[@]}"
    printf '\n'
  }
)"

if [ -f "$stamp_file" ] && [ -f "${prefix}/lib/libavcodec.a" ] && [ -f "${prefix}/lib/libavutil.a" ]; then
  old_stamp="$(cat "$stamp_file")"
  if [ "$old_stamp" = "$new_stamp" ]; then
    echo "Reusing cached RKMPP FFmpeg at ${prefix}"
    exit 0
  fi
fi

rm -rf "$build_dir" "$prefix"
mkdir -p "$build_dir" "$prefix"

pushd "$build_dir" >/dev/null
"${source_dir}/configure" "${configure_args[@]}"

make -j"$jobs"
# Sunshine only needs the libraries and headers from the RKMPP FFmpeg build.
# The example-doc install step can fail on some mounted build volumes when it
# tries to adjust permissions on doc/examples/Makefile, so avoid the broader
# top-level install target here.
make install-libs install-headers

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
printf '%s' "$new_stamp" > "$stamp_file"

echo "Prepared RKMPP FFmpeg at ${prefix}"
