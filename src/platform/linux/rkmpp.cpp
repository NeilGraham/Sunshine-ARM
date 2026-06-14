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
#include <algorithm>
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
#include <vector>

// lib includes
#include <linux/videodev2.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/hwcontext.h>
#include <libavutil/hwcontext_drm.h>
#include <libavutil/pixdesc.h>
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

      this->width = width;
      this->height = height;

      v4l2_format fmt {};
      fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      fmt.fmt.pix.width = width;
      fmt.fmt.pix.height = height;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
      fmt.fmt.pix.field = V4L2_FIELD_NONE;
      if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        char string[1024];
        BOOST_LOG(warning) << "RKMPP direct V4L2: VIDIOC_S_FMT failed: "sv << strerror_r(errno, string, sizeof(string));
        return false;
      }

      if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_NV12 || (int) fmt.fmt.pix.width != width || (int) fmt.fmt.pix.height != height) {
        BOOST_LOG(warning) << "RKMPP direct V4L2: device returned unsupported format "
                           << fmt.fmt.pix.width << 'x' << fmt.fmt.pix.height
                           << " fourcc=0x"sv << util::hex(fmt.fmt.pix.pixelformat).to_string_view();
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
      BOOST_LOG(info) << "RKMPP direct V4L2: capturing "sv << device << " as NV12 "sv << width << 'x' << height << '@' << fps;
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
        latest_frame.assign(src, src + width * height * 3 / 2);
        xioctl(fd, VIDIOC_QBUF, &latest);
      }

      if (latest_frame.empty()) {
        return false;
      }

      copy_nv12(latest_frame.data(), mapped_frame, dst_width, dst_height);
      return true;
    }

    int width {};
    int height {};

  private:
    struct buffer_t {
      void *start {};
      std::size_t length {};
    };

    void stop() {
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

      auto scale = std::min((float) dst_width / width, (float) dst_height / height);
      auto out_w = std::max(2, (int) (width * scale) & ~1);
      auto out_h = std::max(2, (int) (height * scale) & ~1);
      auto off_x = ((dst_width - out_w) / 2) & ~1;
      auto off_y = ((dst_height - out_h) / 2) & ~1;

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

    int fd {-1};
    bool streaming {};
    std::vector<std::uint8_t> latest_frame;
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
        BOOST_LOG(info) << "Using RKMPP direct V4L2 NV12 encode path"sv;
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
          BOOST_LOG(warning) << "RKMPP direct V4L2: no capture frame available"sv;
          return -1;
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
