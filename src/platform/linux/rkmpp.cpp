/**
 * @file src/platform/linux/rkmpp.cpp
 * @brief Rockchip MPP (RKMPP) encode device.
 *
 * RKMPP encodes in hardware, but feeding it software frames forces Sunshine to
 * do the capture color conversion on the CPU (libswscale), which dominates CPU
 * usage while streaming.
 *
 * Neither of the obvious zero-copy paths works on this SoC:
 *   - The Mali GPU (Panfrost) refuses to use an *imported* DMA-BUF as a
 *     multi-plane FBO colour attachment, so rendering RGB->NV12 straight into
 *     the encoder buffer produced an all-green (zero) image.
 *   - The RGA 2D engine cannot map frame-sized DMA-BUFs on this kernel
 *     (rga2 "swiotlb buffer is full" / "map dma buffer error"), so it can't be
 *     used to blit into the encoder buffer either.
 *
 * So this device keeps the capture zero-copy (KMS DMA-BUF) and does the colour
 * conversion on the Mali GPU into a *native* render target (native textures are
 * renderable, unlike imported ones), then reads the NV12 result back into the
 * encoder's MPP DMA-BUF. The VPU then encodes that buffer. This removes the CPU
 * colour conversion; the only remaining cost is a single NV12 read-back.
 */
// standard includes
#include <array>
#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <cstdlib>
#include <fcntl.h>
#include <poll.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <tuple>
#include <unistd.h>
#include <utility>
#include <vector>

// lib includes
#include <linux/videodev2.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/hwcontext.h>
#include <libavutil/hwcontext_drm.h>
#include <libavutil/imgutils.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

// local includes
#include "graphics.h"
#include "misc.h"
#include "rkmpp.h"
#include "src/logging.h"
#include "src/platform/common.h"
#include "src/utility.h"
#include "src/video.h"

using namespace std::literals;

namespace rkmpp {
  int xioctl(int fd, unsigned long request, void *arg) {
    int status;
    do {
      status = ioctl(fd, request, arg);
    } while (status < 0 && errno == EINTR);
    return status;
  }

  enum class scale_filter_e {
    automatic,
    nearest,
    linear,
  };

  class v4l2_nv12_source_t {
  public:
    ~v4l2_nv12_source_t() {
      stop();
    }

    bool init(const char *device, int width, int height, int fps) {
      fd = open(device, O_RDWR | O_NONBLOCK);
      if (fd < 0) {
        char string[1024];
        BOOST_LOG(warning) << "RKMPP direct V4L2: couldn't open "sv << device << ": "sv << strerror_r(errno, string, sizeof(string));
        return false;
      }

      forced_aspect_ratio = parse_aspect_ratio(std::getenv("SUNSHINE_RKMPP_ASPECT_RATIO"));
      scale_filter = parse_scale_filter(std::getenv("SUNSHINE_RKMPP_SCALE_FILTER"));
      if (!negotiate_format(width, height)) {
        return false;
      }

      v4l2_streamparm parm {};
      parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      parm.parm.capture.timeperframe.numerator = 1;
      parm.parm.capture.timeperframe.denominator = fps;
      xioctl(fd, VIDIOC_S_PARM, &parm);

      v4l2_requestbuffers req {};
      req.count = 4;
      req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      req.memory = V4L2_MEMORY_MMAP;
      if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0 || req.count < 2) {
        char string[1024];
        BOOST_LOG(warning) << "RKMPP direct V4L2: VIDIOC_REQBUFS failed: "sv << strerror_r(errno, string, sizeof(string));
        return false;
      }

      buffers.resize(req.count);
      for (std::uint32_t x = 0; x < req.count; ++x) {
        v4l2_buffer buf {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = x;
        if (xioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
          return false;
        }

        buffers[x].length = buf.length;
        buffers[x].start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        if (buffers[x].start == MAP_FAILED) {
          buffers[x].start = nullptr;
          return false;
        }

        if (xioctl(fd, VIDIOC_QBUF, &buf) < 0) {
          return false;
        }
      }

      int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        return false;
      }

      streaming = true;
      std::string format_label;
      if (is_mjpeg) {
        format_label = mjpeg_hw ? "MJPEG(HW-decode)->NV12" : "MJPEG(SW-decode)->NV12";
      } else if (is_raw_convert) {
        format_label = std::string(av_get_pix_fmt_name(raw_av_fmt)) + "(swscale)->NV12";
      } else {
        format_label = "NV12";
      }
      BOOST_LOG(info) << "RKMPP direct V4L2: capturing "sv << device << " as "sv << format_label
                      << ' ' << this->width << 'x' << this->height << '@' << fps;
      return true;
    }

    bool copy_latest_to(AVFrame *mapped_frame, int dst_width, int dst_height) {
      v4l2_buffer latest {};
      bool have_latest = false;

      for (;;) {
        v4l2_buffer buf {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (xioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
          if (errno == EAGAIN && !have_latest) {
            pollfd pfd {fd, POLLIN, 0};
            auto timeout_ms = latest_frame.empty() ? 250 : 8;
            if (poll(&pfd, 1, timeout_ms) > 0) {
              continue;
            }
          }
          break;
        }

        if (have_latest) {
          xioctl(fd, VIDIOC_QBUF, &latest);
        }
        latest = buf;
        have_latest = true;
      }

      if (have_latest) {
        auto *src = (const std::uint8_t *) buffers[latest.index].start;
        if (is_mjpeg) {
          decode_mjpeg_to_nv12(src, latest.bytesused);
        } else if (is_raw_convert) {
          if (convert_raw_to_nv12(src, convert_buf) && !is_zeroed_nv12(convert_buf.data())) {
            latest_frame = convert_buf;
          }
        } else if (!is_zeroed_nv12(src)) {
          latest_frame.assign(src, src + width * height * 3 / 2);
        }
        xioctl(fd, VIDIOC_QBUF, &latest);
      }

      if (latest_frame.empty()) {
        return false;
      }

      missing_frame_logged = false;
      copy_nv12(latest_frame.data(), mapped_frame, dst_width, dst_height);
      return true;
    }

    bool should_log_missing_frame() {
      if (missing_frame_logged) {
        return false;
      }
      missing_frame_logged = true;
      return true;
    }

    bool is_zeroed_nv12(const std::uint8_t *src) const {
      auto size = (std::size_t) width * height * 3 / 2;
      auto step = std::max<std::size_t>(1, size / 4096);

      for (std::size_t offset = 0; offset < size; offset += step) {
        if (src[offset] != 0) {
          return false;
        }
      }

      return true;
    }

    void copy_black_to(AVFrame *mapped_frame, int dst_width, int dst_height) {
      std::fill_n(mapped_frame->data[0], (std::size_t) mapped_frame->linesize[0] * dst_height, 16);
      std::fill_n(mapped_frame->data[1], (std::size_t) mapped_frame->linesize[1] * (dst_height / 2), 128);
    }

    int width {};
    int height {};

  private:
    struct buffer_t {
      void *start {};
      std::size_t length {};
    };

    void stop() {
      if (mjpeg_ctx) {
        avcodec_free_context(&mjpeg_ctx);
        mjpeg_ctx = nullptr;
      }
      if (sws_ctx) {
        sws_freeContext(sws_ctx);
        sws_ctx = nullptr;
      }
      if (fd >= 0 && streaming) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(fd, VIDIOC_STREAMOFF, &type);
      }
      streaming = false;

      for (auto &buffer : buffers) {
        if (buffer.start) {
          munmap(buffer.start, buffer.length);
        }
      }
      buffers.clear();

      if (fd >= 0) {
        v4l2_requestbuffers req {};
        req.count = 0;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        xioctl(fd, VIDIOC_REQBUFS, &req);
      }

      if (fd >= 0) {
        close(fd);
      }
      fd = -1;
    }

    void copy_nv12(const std::uint8_t *src, AVFrame *dst, int dst_width, int dst_height) {
      const auto *src_y = src;
      const auto *src_uv = src + width * height;

      if (dst_width == width && dst_height == height) {
        for (int y = 0; y < height; ++y) {
          std::memcpy(dst->data[0] + (std::size_t) y * dst->linesize[0], src_y + (std::size_t) y * width, width);
        }

        for (int y = 0; y < height / 2; ++y) {
          std::memcpy(dst->data[1] + (std::size_t) y * dst->linesize[1], src_uv + (std::size_t) y * width, width);
        }

        return;
      }

      std::fill_n(dst->data[0], (std::size_t) dst->linesize[0] * dst_height, 16);
      std::fill_n(dst->data[1], (std::size_t) dst->linesize[1] * (dst_height / 2), 128);

      auto output_aspect = forced_aspect_ratio > 0.0f ? forced_aspect_ratio : (float) width / height;
      auto out_w = dst_width;
      auto out_h = (int) (dst_width / output_aspect);
      if (out_h > dst_height) {
        out_h = dst_height;
        out_w = (int) (dst_height * output_aspect);
      }
      out_w = std::max(2, out_w & ~1);
      out_h = std::max(2, out_h & ~1);
      auto off_x = ((dst_width - out_w) / 2) & ~1;
      auto off_y = ((dst_height - out_h) / 2) & ~1;

      const bool linear_scale = scale_filter == scale_filter_e::linear ||
                                (scale_filter == scale_filter_e::automatic && (out_w != width || out_h != height));
      if (linear_scale) {
        scale_nv12_linear(src_y, src_uv, dst, off_x, off_y, out_w, out_h);
      } else {
        scale_nv12_nearest(src_y, src_uv, dst, off_x, off_y, out_w, out_h);
      }
    }

    void scale_nv12_nearest(const std::uint8_t *src_y, const std::uint8_t *src_uv, AVFrame *dst, int off_x, int off_y, int out_w, int out_h) {
      for (int y = 0; y < out_h; ++y) {
        auto sy = std::min(height - 1, (int) ((std::int64_t) y * height / out_h));
        auto *dst_row = dst->data[0] + (std::size_t) (off_y + y) * dst->linesize[0] + off_x;
        const auto *src_row = src_y + (std::size_t) sy * width;
        for (int x = 0; x < out_w; ++x) {
          auto sx = std::min(width - 1, (int) ((std::int64_t) x * width / out_w));
          dst_row[x] = src_row[sx];
        }
      }

      for (int y = 0; y < out_h / 2; ++y) {
        auto sy = std::min(height / 2 - 1, (int) ((std::int64_t) y * (height / 2) / (out_h / 2)));
        auto *dst_row = dst->data[1] + (std::size_t) (off_y / 2 + y) * dst->linesize[1] + off_x;
        const auto *src_row = src_uv + (std::size_t) sy * width;
        for (int x = 0; x < out_w; x += 2) {
          auto sx = std::min(width - 2, ((int) ((std::int64_t) x * width / out_w)) & ~1);
          dst_row[x] = src_row[sx];
          dst_row[x + 1] = src_row[sx + 1];
        }
      }
    }

    void scale_plane_linear(const std::uint8_t *src, int src_stride, int src_w, int src_h, std::uint8_t *dst, int dst_stride, int dst_w, int dst_h, int channels) {
      for (int y = 0; y < dst_h; ++y) {
        float src_y_f = ((y + 0.5f) * src_h / dst_h) - 0.5f;
        if (src_y_f < 0.0f) {
          src_y_f = 0.0f;
        }
        int y0 = std::min(src_h - 1, (int) src_y_f);
        int y1 = std::min(src_h - 1, y0 + 1);
        float fy = src_y_f - y0;

        auto *dst_row = dst + (std::size_t) y * dst_stride;
        const auto *src_row0 = src + (std::size_t) y0 * src_stride;
        const auto *src_row1 = src + (std::size_t) y1 * src_stride;
        for (int x = 0; x < dst_w; ++x) {
          float src_x_f = ((x + 0.5f) * src_w / dst_w) - 0.5f;
          if (src_x_f < 0.0f) {
            src_x_f = 0.0f;
          }
          int x0 = std::min(src_w - 1, (int) src_x_f);
          int x1 = std::min(src_w - 1, x0 + 1);
          float fx = src_x_f - x0;

          for (int c = 0; c < channels; ++c) {
            auto p00 = src_row0[x0 * channels + c];
            auto p10 = src_row0[x1 * channels + c];
            auto p01 = src_row1[x0 * channels + c];
            auto p11 = src_row1[x1 * channels + c];
            auto top = p00 + (p10 - p00) * fx;
            auto bottom = p01 + (p11 - p01) * fx;
            auto value = top + (bottom - top) * fy;
            dst_row[x * channels + c] = (std::uint8_t) std::min(255.0f, std::max(0.0f, value + 0.5f));
          }
        }
      }
    }

    void scale_nv12_linear(const std::uint8_t *src_y, const std::uint8_t *src_uv, AVFrame *dst, int off_x, int off_y, int out_w, int out_h) {
      auto *dst_y = dst->data[0] + (std::size_t) off_y * dst->linesize[0] + off_x;
      auto *dst_uv = dst->data[1] + (std::size_t) (off_y / 2) * dst->linesize[1] + off_x;

      scale_plane_linear(src_y, width, width, height, dst_y, dst->linesize[0], out_w, out_h, 1);
      scale_plane_linear(src_uv, width, width / 2, height / 2, dst_uv, dst->linesize[1], out_w / 2, out_h / 2, 2);
    }

    struct forced_format_t {
      std::uint32_t v4l2 {};
      bool mjpeg_hw {};
      bool is_forced {};
    };

    // Negotiate the capture format, preferring non-blocking paths: NV12
    // (native, no conversion), then raw formats converted with libswscale, then
    // MJPEG decoded with software. Hardware MJPEG is only used when explicitly
    // requested with SUNSHINE_RKMPP_V4L2_FORMAT=mjpeg-hw because mjpeg_rkmpp can
    // block inside MPP on UVC streams.
    bool negotiate_format(int requested_width, int requested_height) {
      static const struct {
        std::uint32_t v4l2;
        AVPixelFormat av;  // AV_PIX_FMT_NONE: NV12 (native) or MJPEG (decoded)
      } candidates[] = {
        {V4L2_PIX_FMT_NV12, AV_PIX_FMT_NONE},    // native, zero conversion
        {V4L2_PIX_FMT_YUYV, AV_PIX_FMT_YUYV422}, // raw, libswscale convert
        {V4L2_PIX_FMT_YUV420, AV_PIX_FMT_YUV420P},
        {V4L2_PIX_FMT_BGR24, AV_PIX_FMT_BGR24},
        {V4L2_PIX_FMT_MJPEG, AV_PIX_FMT_NONE}, // JPEG decode
      };

      // SUNSHINE_RKMPP_V4L2_FORMAT (fed from [capture].format in retro-stream.toml)
      // keeps the default preference-ordered negotiation unless a format is
      // explicitly pinned.
      auto forced = parse_forced_format(std::getenv("SUNSHINE_RKMPP_V4L2_FORMAT"));
      std::array<std::size_t, sizeof(candidates) / sizeof(candidates[0])> order {
        0, 1, 2, 3, 4
      };
      const bool mjpeg_prefers_fallback = forced.is_forced && forced.v4l2 == V4L2_PIX_FMT_MJPEG;

      if (mjpeg_prefers_fallback) {
        BOOST_LOG(info) << "RKMPP direct V4L2: format preference "sv << fourcc_str(forced.v4l2)
                        << (forced.mjpeg_hw ? " with hardware MJPEG decode"sv : ""sv);
        order = std::array<std::size_t, sizeof(candidates) / sizeof(candidates[0])> {4, 0, 1, 2, 3};
      } else if (forced.is_forced) {
        BOOST_LOG(info) << "RKMPP direct V4L2: format pinned to "sv << fourcc_str(forced.v4l2)
                        << (forced.mjpeg_hw ? " with hardware MJPEG decode"sv : ""sv);
      }

      const auto sizes = capture_size_fallbacks(requested_width, requested_height);
      for (auto size : sizes) {
        for (auto idx : order) {
          auto candidate = candidates[idx];
          if (forced.is_forced && !mjpeg_prefers_fallback && candidate.v4l2 != forced.v4l2) {
            continue;
          }
          v4l2_format fmt {};
          fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
          fmt.fmt.pix.width = size.first;
          fmt.fmt.pix.height = size.second;
          fmt.fmt.pix.pixelformat = candidate.v4l2;
          fmt.fmt.pix.field = V4L2_FIELD_NONE;
          if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
            continue;
          }
          if (fmt.fmt.pix.pixelformat != candidate.v4l2 ||
              (int) fmt.fmt.pix.width != size.first || (int) fmt.fmt.pix.height != size.second) {
            continue;
          }

          if (!activate_format(candidate.v4l2, candidate.av, size.first, size.second, forced.mjpeg_hw)) {
            return false;
          }
          if (size.first != requested_width || size.second != requested_height) {
            BOOST_LOG(info) << "RKMPP direct V4L2: requested "sv << requested_width << 'x' << requested_height
                            << " unavailable; capturing "sv << size.first << 'x' << size.second
                            << " and scaling to encoder frame"sv;
          }
          if (mjpeg_prefers_fallback && candidate.v4l2 != V4L2_PIX_FMT_MJPEG) {
            BOOST_LOG(info) << "RKMPP direct V4L2: MJPG unsupported at "sv << size.first << 'x' << size.second
                            << "; falling back to "sv << fourcc_str(candidate.v4l2);
          }
          return true;
        }
      }

      if (forced.is_forced) {
        BOOST_LOG(warning) << "RKMPP direct V4L2: pinned format "sv << fourcc_str(forced.v4l2)
                           << " not supported by device at "sv << requested_width << 'x' << requested_height;
      } else {
        BOOST_LOG(warning) << "RKMPP direct V4L2: device offers no supported pixel format at "sv
                           << requested_width << 'x' << requested_height;
      }
      return false;
    }

    static std::vector<std::pair<int, int>> capture_size_fallbacks(int width, int height) {
      std::vector<std::pair<int, int>> sizes;
      auto add = [&](int w, int h) {
        if (std::find(sizes.begin(), sizes.end(), std::make_pair(w, h)) == sizes.end()) {
          sizes.emplace_back(w, h);
        }
      };

      add(width, height);
      if (height == 480) {
        add(720, 480);
        add(640, 480);
      } else if (height == 576) {
        add(720, 576);
      }

      return sizes;
    }

    bool activate_format(std::uint32_t v4l2, AVPixelFormat av, int width, int height, bool mjpeg_hw) {
      capture_fourcc = v4l2;
      this->width = width;
      this->height = height;
      is_mjpeg = false;
      is_raw_convert = false;
      raw_av_fmt = AV_PIX_FMT_NONE;

      if (v4l2 == V4L2_PIX_FMT_MJPEG) {
        if (!open_mjpeg_decoder(width, height, mjpeg_hw)) {
          return false;
        }
        is_mjpeg = true;
      } else if (av != AV_PIX_FMT_NONE) {
        raw_av_fmt = av;
        is_raw_convert = true;
      }
      return true;
    }

    // FourCC (e.g. V4L2_PIX_FMT_YUYV) as its four ASCII characters, for logs.
    static std::string fourcc_str(std::uint32_t fourcc) {
      char chars[5] = {
        (char) (fourcc & 0xff),
        (char) ((fourcc >> 8) & 0xff),
        (char) ((fourcc >> 16) & 0xff),
        (char) ((fourcc >> 24) & 0xff),
        0,
      };
      return chars;
    }

    static float parse_aspect_ratio(const char *env) {
      if (!env || !*env) {
        return 0.0f;
      }
      std::string v(env);
      std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return (char) std::tolower(c); });
      v.erase(std::remove_if(v.begin(), v.end(), [](unsigned char c) { return std::isspace(c); }), v.end());
      if (v.empty() || v == "auto" || v == "current") {
        return 0.0f;
      }

      auto colon = v.find(':');
      if (colon == std::string::npos) {
        colon = v.find('/');
      }
      if (colon != std::string::npos) {
        try {
          auto num = std::stof(v.substr(0, colon));
          auto den = std::stof(v.substr(colon + 1));
          if (num > 0.0f && den > 0.0f) {
            return num / den;
          }
        } catch (...) {
        }
      }

      try {
        auto ratio = std::stof(v);
        if (ratio > 0.0f) {
          return ratio;
        }
      } catch (...) {
      }

      BOOST_LOG(warning) << "RKMPP direct V4L2: invalid SUNSHINE_RKMPP_ASPECT_RATIO '"sv << env << "'; using auto"sv;
      return 0.0f;
    }

    static scale_filter_e parse_scale_filter(const char *env) {
      if (!env || !*env) {
        return scale_filter_e::automatic;
      }
      std::string v(env);
      std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return (char) std::tolower(c); });
      v.erase(std::remove_if(v.begin(), v.end(), [](unsigned char c) { return std::isspace(c) || c == '-' || c == '_'; }), v.end());

      if (v.empty() || v == "auto" || v == "automatic") {
        return scale_filter_e::automatic;
      }
      if (v == "nearest" || v == "point") {
        return scale_filter_e::nearest;
      }
      if (v == "linear" || v == "bilinear") {
        return scale_filter_e::linear;
      }

      BOOST_LOG(warning) << "RKMPP direct V4L2: invalid SUNSHINE_RKMPP_SCALE_FILTER '"sv << env << "'; using auto"sv;
      return scale_filter_e::automatic;
    }

    // Map a SUNSHINE_RKMPP_V4L2_FORMAT token to a V4L2 fourcc. Plain "mjpeg"
    // still uses software decode, but the capture negotiation treats it as a
    // preference rather than a hard pin so unsupported resolutions can fall
    // back to a raw format. Only "mjpeg-hw" opts into the potentially blocking
    // mjpeg_rkmpp decoder for experiments.
    static forced_format_t parse_forced_format(const char *env) {
      if (!env) {
        return {};
      }
      std::string v(env);
      std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return (char) std::tolower(c); });
      if (v.empty() || v == "auto") {
        return {};
      }
      if (v == "nv12") {
        return {V4L2_PIX_FMT_NV12, false, true};
      }
      if (v == "mjpeg" || v == "mjpg") {
        return {V4L2_PIX_FMT_MJPEG, false, true};
      }
      if (v == "mjpeg-hw" || v == "mjpg-hw" || v == "mjpeg_rkmpp" || v == "mjpg_rkmpp") {
        return {V4L2_PIX_FMT_MJPEG, true, true};
      }
      if (v == "yuyv" || v == "yuyv422" || v == "yuy2") {
        return {V4L2_PIX_FMT_YUYV, false, true};
      }
      if (v == "yu12" || v == "yuv420" || v == "i420") {
        return {V4L2_PIX_FMT_YUV420, false, true};
      }
      if (v == "bgr3" || v == "bgr24") {
        return {V4L2_PIX_FMT_BGR24, false, true};
      }
      BOOST_LOG(warning) << "RKMPP direct V4L2: unknown SUNSHINE_RKMPP_V4L2_FORMAT '"sv << env << "'; using auto"sv;
      return {};
    }

    static AVPixelFormat mjpeg_rkmpp_get_format(AVCodecContext * /* ctx */, const AVPixelFormat *pix_fmts) {
      for (const AVPixelFormat *fmt = pix_fmts; *fmt != AV_PIX_FMT_NONE; ++fmt) {
        if (*fmt == AV_PIX_FMT_NV12) {
          return *fmt;
        }
      }
      for (const AVPixelFormat *fmt = pix_fmts; *fmt != AV_PIX_FMT_NONE; ++fmt) {
        if (*fmt == AV_PIX_FMT_DRM_PRIME) {
          return *fmt;
        }
      }
      return pix_fmts[0];
    }

    bool open_mjpeg_decoder(int width, int height, bool prefer_hw) {
      // UVC MJPEG works reliably with FFmpeg's software decoder. mjpeg_rkmpp is
      // useful to test on hardware, but some streams block inside MPP instead
      // of returning an error, so never select it implicitly.
      const AVCodec *mjpeg_codec = prefer_hw ? avcodec_find_decoder_by_name("mjpeg_rkmpp") : nullptr;
      mjpeg_hw = (mjpeg_codec != nullptr);
      if (!mjpeg_codec && prefer_hw) {
        BOOST_LOG(warning) << "RKMPP direct V4L2: mjpeg_rkmpp unavailable; falling back to software MJPEG decoder"sv;
      }
      if (!mjpeg_codec) {
        mjpeg_codec = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
        mjpeg_hw = false;
      }
      if (!mjpeg_codec) {
        BOOST_LOG(warning) << "RKMPP direct V4L2: no MJPEG decoder available"sv;
        return false;
      }
      mjpeg_ctx = avcodec_alloc_context3(mjpeg_codec);
      if (mjpeg_ctx) {
        mjpeg_ctx->width  = width;
        mjpeg_ctx->height = height;
        if (mjpeg_hw) {
          mjpeg_ctx->pix_fmt = AV_PIX_FMT_YUVJ420P;
          mjpeg_ctx->get_format = mjpeg_rkmpp_get_format;
        }
      }
      if (!mjpeg_ctx || avcodec_open2(mjpeg_ctx, mjpeg_codec, nullptr) < 0) {
        BOOST_LOG(warning) << "RKMPP direct V4L2: MJPEG avcodec_open2 failed"sv;
        avcodec_free_context(&mjpeg_ctx);
        if (mjpeg_hw) {
          BOOST_LOG(warning) << "RKMPP direct V4L2: falling back to software MJPEG decoder"sv;
          mjpeg_hw = false;
          return open_mjpeg_decoder(width, height, false);
        }
        return false;
      }
      return true;
    }

    // Convert one raw capture buffer (raw_av_fmt at width x height) to NV12 with
    // libswscale, into out. UVC raw formats are tightly packed, so we describe
    // the planes with av_image_fill_arrays at byte alignment.
    bool convert_raw_to_nv12(const std::uint8_t *src, std::vector<std::uint8_t> &out) {
      out.resize((std::size_t) width * height * 3 / 2);

      std::uint8_t *planes[4] = {};
      int src_stride[4] = {};
      if (av_image_fill_arrays(planes, src_stride, src, raw_av_fmt, width, height, 1) < 0) {
        return false;
      }
      const std::uint8_t *src_data[4] = {planes[0], planes[1], planes[2], planes[3]};

      std::uint8_t *dst[4] = {out.data(), out.data() + (std::size_t) width * height, nullptr, nullptr};
      int dst_stride[4] = {width, width, 0, 0};

      sws_ctx = sws_getCachedContext(
        sws_ctx,
        width, height, raw_av_fmt,
        width, height, AV_PIX_FMT_NV12,
        SWS_BILINEAR, nullptr, nullptr, nullptr
      );
      if (!sws_ctx) {
        return false;
      }

      sws_scale(sws_ctx, src_data, src_stride, 0, height, dst, dst_stride);
      return true;
    }

    // Decode one MJPEG packet into a fresh AVFrame with the current decoder, or
    // nullptr on failure.
    AVFrame *mjpeg_decode_frame(const std::uint8_t *mjpeg_data, std::uint32_t mjpeg_size) {
      AVPacket *pkt = av_packet_alloc();
      if (!pkt) {
        return nullptr;
      }
      if (av_new_packet(pkt, (int) mjpeg_size) < 0) {
        av_packet_free(&pkt);
        return nullptr;
      }
      std::memcpy(pkt->data, mjpeg_data, mjpeg_size);

      int ret = avcodec_send_packet(mjpeg_ctx, pkt);
      av_packet_free(&pkt);
      if (ret < 0) {
        return nullptr;
      }

      AVFrame *decoded = av_frame_alloc();
      if (!decoded) {
        return nullptr;
      }
      if (avcodec_receive_frame(mjpeg_ctx, decoded) < 0) {
        av_frame_free(&decoded);
        return nullptr;
      }
      return decoded;
    }

    // Reopen the MJPEG context with the software decoder. Runtime fallback for
    // when the hardware decoder rejects a stream; the SW decoder also handles
    // UVC frames that omit Huffman tables (it inserts default tables).
    bool reopen_mjpeg_sw() {
      if (mjpeg_ctx) {
        avcodec_free_context(&mjpeg_ctx);
      }
      const AVCodec *sw = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
      if (!sw) {
        return false;
      }
      mjpeg_ctx = avcodec_alloc_context3(sw);
      if (mjpeg_ctx) {
        mjpeg_ctx->width  = width;
        mjpeg_ctx->height = height;
      }
      if (!mjpeg_ctx || avcodec_open2(mjpeg_ctx, sw, nullptr) < 0) {
        avcodec_free_context(&mjpeg_ctx);
        return false;
      }
      mjpeg_hw = false;
      return true;
    }

    bool convert_planar_yuv_to_nv12(const AVFrame *cpu, std::vector<std::uint8_t> &out) {
      const auto pix_fmt = (AVPixelFormat) cpu->format;
      const bool is_420 = pix_fmt == AV_PIX_FMT_YUV420P || pix_fmt == AV_PIX_FMT_YUVJ420P;
      const bool is_422 = pix_fmt == AV_PIX_FMT_YUV422P || pix_fmt == AV_PIX_FMT_YUVJ422P;
      if (!is_420 && !is_422) {
        return false;
      }

      auto *dst_y = out.data();
      auto *dst_uv = dst_y + (std::size_t) width * height;

      for (int y = 0; y < height; ++y) {
        std::memcpy(dst_y + (std::size_t) y * width,
                    cpu->data[0] + (std::size_t) y * cpu->linesize[0],
                    width);
      }

      for (int y = 0; y < height / 2; ++y) {
        const auto *src_u0 = cpu->data[1] + (std::size_t) y * cpu->linesize[1];
        const auto *src_v0 = cpu->data[2] + (std::size_t) y * cpu->linesize[2];
        const std::uint8_t *src_u1 = src_u0;
        const std::uint8_t *src_v1 = src_v0;

        if (is_422) {
          src_u0 = cpu->data[1] + (std::size_t) (y * 2) * cpu->linesize[1];
          src_v0 = cpu->data[2] + (std::size_t) (y * 2) * cpu->linesize[2];
          src_u1 = cpu->data[1] + (std::size_t) std::min(y * 2 + 1, height - 1) * cpu->linesize[1];
          src_v1 = cpu->data[2] + (std::size_t) std::min(y * 2 + 1, height - 1) * cpu->linesize[2];
        }

        auto *dst = dst_uv + (std::size_t) y * width;
        for (int x = 0; x < width / 2; ++x) {
          if (is_422) {
            dst[x * 2] = (std::uint8_t) (((int) src_u0[x] + src_u1[x] + 1) / 2);
            dst[x * 2 + 1] = (std::uint8_t) (((int) src_v0[x] + src_v1[x] + 1) / 2);
          } else {
            dst[x * 2] = src_u0[x];
            dst[x * 2 + 1] = src_v0[x];
          }
        }
      }

      return true;
    }

    bool decode_mjpeg_to_nv12(const std::uint8_t *mjpeg_data, std::uint32_t mjpeg_size) {
      if (mjpeg_size == 0) return false;

      AVFrame *decoded = mjpeg_decode_frame(mjpeg_data, mjpeg_size);
      if (!decoded && mjpeg_hw && reopen_mjpeg_sw()) {
        BOOST_LOG(warning) << "RKMPP direct V4L2: hardware MJPEG decode failed; falling back to software decoder"sv;
        decoded = mjpeg_decode_frame(mjpeg_data, mjpeg_size);
      }
      if (!decoded) {
        return false;
      }

      // mjpeg_rkmpp outputs DRM_PRIME (hardware buffer); download to CPU NV12.
      // SW MJPEG may output 4:2:0 or 4:2:2 planar JPEG formats, so normalize
      // known layouts directly and keep swscale as the uncommon-format fallback.
      AVFrame *cpu = decoded;
      if (decoded->format == AV_PIX_FMT_DRM_PRIME) {
        cpu = av_frame_alloc();
        if (!cpu || av_hwframe_transfer_data(cpu, decoded, 0) < 0) {
          av_frame_free(&cpu);
          av_frame_free(&decoded);
          return false;
        }
        av_frame_free(&decoded);
      }

      auto nv12_size = (std::size_t) width * height * 3 / 2;
      latest_frame.resize(nv12_size);

      bool converted = false;
      if (cpu->format == AV_PIX_FMT_NV12) {
        auto *dst_y  = latest_frame.data();
        auto *dst_uv = dst_y + (std::size_t) width * height;

        for (int y = 0; y < height; ++y) {
          std::memcpy(dst_y + (std::size_t) y * width,
                      cpu->data[0] + (std::size_t) y * cpu->linesize[0],
                      width);
        }

        for (int y = 0; y < height / 2; ++y) {
          std::memcpy(dst_uv + (std::size_t) y * width,
                      cpu->data[1] + (std::size_t) y * cpu->linesize[1],
                      width);
        }
        converted = true;
      } else {
        converted = convert_planar_yuv_to_nv12(cpu, latest_frame);
      }

      if (!converted) {
        std::uint8_t *dst[4] = {latest_frame.data(), latest_frame.data() + (std::size_t) width * height, nullptr, nullptr};
        int dst_stride[4] = {width, width, 0, 0};

        sws_ctx = sws_getCachedContext(
          sws_ctx,
          width, height, (AVPixelFormat) cpu->format,
          width, height, AV_PIX_FMT_NV12,
          SWS_BILINEAR, nullptr, nullptr, nullptr
        );
        if (!sws_ctx) {
          av_frame_free(&cpu);
          if (cpu != decoded) {
            av_frame_free(&decoded);
          }
          return false;
        }

        sws_scale(sws_ctx, cpu->data, cpu->linesize, 0, height, dst, dst_stride);
      }

      av_frame_free(&cpu);
      return true;
    }

    int fd {-1};
    bool streaming {};
    bool missing_frame_logged {};
    bool is_mjpeg {};
    bool mjpeg_hw {};
    bool is_raw_convert {};
    float forced_aspect_ratio {};
    scale_filter_e scale_filter {scale_filter_e::automatic};
    std::uint32_t capture_fourcc {V4L2_PIX_FMT_NV12};
    AVPixelFormat raw_av_fmt {AV_PIX_FMT_NONE};
    AVCodecContext *mjpeg_ctx {};
    SwsContext *sws_ctx {};
    std::vector<std::uint8_t> latest_frame;
    std::vector<std::uint8_t> convert_buf;
    std::vector<buffer_t> buffers;
  };

  class rkmpp_t: public platf::avcodec_encode_device_t {
  public:
    ~rkmpp_t() override {
    }

    int init(int in_width, int in_height, file_t &&render_device, int offset_x, int offset_y) {
      file = std::move(render_device);

      if (!gbm::create_device) {
        BOOST_LOG(warning) << "libgbm not initialized"sv;
        return -1;
      }

      // Marker so video.cpp uses this device instead of the CPU software path.
      this->data = (void *) rkmpp_init_avcodec_hardware_input_buffer;

      gbm.reset(gbm::create_device(file.el));
      if (!gbm) {
        char string[1024];
        BOOST_LOG(error) << "Couldn't create GBM device: ["sv << strerror_r(errno, string, sizeof(string)) << ']';
        return -1;
      }

      display = egl::make_display(gbm.get());
      if (!display) {
        return -1;
      }

      auto ctx_opt = egl::make_ctx(display.get());
      if (!ctx_opt) {
        return -1;
      }
      ctx = std::move(*ctx_opt);

      width = in_width;
      height = in_height;
      this->offset_x = offset_x;
      this->offset_y = offset_y;
      sequence = 0;

      if (auto v4l2_device = std::getenv("SUNSHINE_RKMPP_V4L2")) {
        direct_v4l2_device = v4l2_device;
      }

      return 0;
    }

    int set_frame(AVFrame *frame, AVBufferRef *hw_frames_ctx_buf) override {
      // The capture display has its own EGL context that may be current on this
      // thread. Ensure our context (which owns the render target/shaders) is
      // current before touching any GL object, otherwise the FBO appears
      // incomplete (GL_INVALID_FRAMEBUFFER_OPERATION) and nothing is rendered.
      make_current();

      this->hwframe.reset(frame);
      this->frame = frame;

      if (!hw_frames_ctx_buf) {
        BOOST_LOG(error) << "RKMPP encode device requires a hardware frames context"sv;
        return -1;
      }

      if (!frame->buf[0]) {
        if (av_hwframe_get_buffer(hw_frames_ctx_buf, frame, 0)) {
          BOOST_LOG(error) << "Couldn't get an RKMPP hwframe"sv;
          return -1;
        }
      }

      if (!direct_v4l2_device.empty() && !direct_v4l2) {
        auto source = std::make_unique<v4l2_nv12_source_t>();
        if (source->init(direct_v4l2_device.c_str(), frame->width, frame->height, 60)) {
          direct_v4l2 = std::move(source);
        } else {
          BOOST_LOG(warning) << "RKMPP direct V4L2 unavailable; falling back to KMS GPU-convert path"sv;
        }
      }

      if (direct_v4l2) {
        BOOST_LOG(info) << "Using RKMPP direct V4L2 encode path"sv;
      } else {
        BOOST_LOG(info) << "Using RKMPP GPU-convert + read-back encode path"sv;

        // Native NV12 render target the GPU converts into.
        auto nv12_opt = egl::create_target(frame->width, frame->height, (AVPixelFormat) AV_PIX_FMT_NV12);
        if (!nv12_opt) {
          return -1;
        }

        auto sws_opt = egl::sws_t::make(width, height, frame->width, frame->height, (AVPixelFormat) AV_PIX_FMT_NV12);
        if (!sws_opt) {
          return -1;
        }

        this->nv12 = std::move(*nv12_opt);
        this->sws = std::move(*sws_opt);
      }

      return 0;
    }

    void apply_colorspace() override {
      if (!direct_v4l2) {
        sws.apply_colorspace(colorspace);
      }
    }

    int convert(platf::img_t &img) override {
      // Our GL objects only exist in our context; make it current (the capture
      // display may have left its own context current on this thread).
      make_current();

      video::avcodec_frame_t mapped_frame {av_frame_alloc()};
      if (!mapped_frame) {
        BOOST_LOG(error) << "Couldn't allocate RKMPP mapped frame"sv;
        return -1;
      }
      mapped_frame->format = AV_PIX_FMT_NV12;

      auto status = av_hwframe_map(mapped_frame.get(), frame, AV_HWFRAME_MAP_WRITE | AV_HWFRAME_MAP_OVERWRITE);
      if (status < 0) {
        char string[AV_ERROR_MAX_STRING_SIZE];
        BOOST_LOG(error) << "Couldn't map RKMPP hwframe for writing: "sv << av_make_error_string(string, AV_ERROR_MAX_STRING_SIZE, status);
        return -1;
      }

      if (!mapped_frame->data[0] || !mapped_frame->data[1] || mapped_frame->linesize[0] <= 0 || mapped_frame->linesize[1] <= 0) {
        BOOST_LOG(error) << "RKMPP mapped frame is not writable NV12"sv;
        return -1;
      }

      if (direct_v4l2) {
        if (!direct_v4l2->copy_latest_to(mapped_frame.get(), frame->width, frame->height)) {
          if (direct_v4l2->should_log_missing_frame()) {
            BOOST_LOG(warning) << "RKMPP direct V4L2: no capture frame available; sending black frames until capture resumes"sv;
          }
          direct_v4l2->copy_black_to(mapped_frame.get(), frame->width, frame->height);
        }
        return 0;
      }

      auto &descriptor = (egl::img_descriptor_t &) img;

      if (descriptor.sequence == 0) {
        rgb = egl::create_blank(img);
      } else if (descriptor.sequence > sequence) {
        sequence = descriptor.sequence;

        rgb = egl::rgb_t {};

        auto rgb_opt = egl::import_source(display.get(), descriptor.sd);
        if (!rgb_opt) {
          return -1;
        }
        rgb = std::move(*rgb_opt);
      }

      if (offset_x || offset_y) {
        sws.load_vram(descriptor, offset_x, offset_y, rgb->tex[0]);
      } else {
        // The Rockchip/Panfrost path can sample imported KMS DMA-BUF textures,
        // but using them as an FBO source for Sunshine's resize copy can fail
        // with GL_INVALID_FRAMEBUFFER_OPERATION. Let the RGB->NV12 shader scale
        // directly from the imported texture instead.
        sws.load_vram_direct(rgb->tex[0]);
      }

      if (sws.convert(nv12->buf)) {
        return -1;
      }

      // Read the converted NV12 planes into FFmpeg's mapped RKMPP frame. The
      // map/unmap path uses RKMPP's own buffer pointer and cache-sync rules,
      // avoiding stale all-green frames from raw DMA-BUF mmap writes.

      gl::ctx.PixelStorei(GL_PACK_ALIGNMENT, 1);

      // Y plane (R8): one byte per pixel, destination stride is y_pitch bytes.
      gl::ctx.PixelStorei(GL_PACK_ROW_LENGTH, mapped_frame->linesize[0]);
      gl::ctx.GetTextureSubImage(
        nv12->tex[0], 0, 0, 0, 0,
        frame->width, frame->height, 1,
        GL_RED, GL_UNSIGNED_BYTE,
        mapped_frame->linesize[0] * frame->height, mapped_frame->data[0]
      );

      // UV plane (RG8): two bytes per pixel, half resolution.
      gl::ctx.PixelStorei(GL_PACK_ROW_LENGTH, mapped_frame->linesize[1] / 2);
      gl::ctx.GetTextureSubImage(
        nv12->tex[1], 0, 0, 0, 0,
        frame->width / 2, frame->height / 2, 1,
        GL_RG, GL_UNSIGNED_BYTE,
        mapped_frame->linesize[1] * (frame->height / 2), mapped_frame->data[1]
      );

      gl::ctx.PixelStorei(GL_PACK_ROW_LENGTH, 0);

      return 0;
    }

    file_t file;

    gbm::gbm_t gbm;
    egl::display_t display;
    egl::ctx_t ctx;

    frame_t hwframe;

    egl::sws_t sws;
    egl::nv12_t nv12;
    egl::rgb_t rgb;

    int width {}, height {};
    int offset_x {}, offset_y {};
    std::uint64_t sequence {};
    std::string direct_v4l2_device;
    std::unique_ptr<v4l2_nv12_source_t> direct_v4l2;

  private:
    void make_current() {
      eglMakeCurrent(display.get(), EGL_NO_SURFACE, EGL_NO_SURFACE, std::get<1>(ctx.el));
    }
  };

  int rkmpp_init_avcodec_hardware_input_buffer(platf::avcodec_encode_device_t * /* base */, AVBufferRef **hw_device_buf) {
    auto status = av_hwdevice_ctx_create(hw_device_buf, AV_HWDEVICE_TYPE_RKMPP, nullptr, nullptr, 0);
    if (status < 0) {
      char string[AV_ERROR_MAX_STRING_SIZE];
      BOOST_LOG(error) << "Failed to create an RKMPP device: "sv << av_make_error_string(string, AV_ERROR_MAX_STRING_SIZE, status);
      return -1;
    }

    return 0;
  }

  std::unique_ptr<platf::avcodec_encode_device_t> make_avcodec_encode_device(int width, int height, file_t &&card, int offset_x, int offset_y) {
    auto device = std::make_unique<rkmpp_t>();

    if (device->init(width, height, std::move(card), offset_x, offset_y)) {
      return nullptr;
    }

    return device;
  }
}  // namespace rkmpp
