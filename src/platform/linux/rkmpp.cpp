/**
 * @file src/platform/linux/rkmpp.cpp
 * @brief Zero-copy Rockchip MPP (RKMPP) encode device.
 *
 * RKMPP encodes in hardware, but feeding it software NV12 frames forces Sunshine
 * to perform the RGB->NV12 color conversion on the CPU (libswscale) plus a full
 * GPU->CPU readback every frame. That dominates CPU usage while streaming.
 *
 * This device avoids that entirely: it allocates an NV12 frame from FFmpeg's
 * RKMPP hardware frames pool (which is backed by a DMA-BUF / MppBuffer), imports
 * that DMA-BUF into EGL, and performs the RGB->NV12 conversion on the Mali GPU.
 * The same MppBuffer is then handed straight to h264_rkmpp / hevc_rkmpp via an
 * AV_PIX_FMT_DRM_PRIME frame, so the encoder imports it without any copy.
 *
 * This mirrors the structure of the VAAPI integration in vaapi.cpp.
 */
// standard includes
#include <array>
#include <cstring>

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
#include "src/config.h"
#include "src/logging.h"
#include "src/platform/common.h"
#include "src/utility.h"
#include "src/video.h"

using namespace std::literals;

namespace rkmpp {
  // DRM fourcc codes for the NV12 sub-planes as imported into EGL.
  // NV12 is exposed to GL as a single-channel R8 luma plane and a two-channel
  // GR88 chroma plane, matching how the VAAPI path imports NV12 surfaces.
  constexpr std::uint32_t fourcc(char a, char b, char c, char d) {
    return (std::uint32_t) a | ((std::uint32_t) b << 8) | ((std::uint32_t) c << 16) | ((std::uint32_t) d << 24);
  }

  constexpr std::uint32_t DRM_FORMAT_R8 = fourcc('R', '8', ' ', ' ');
  constexpr std::uint32_t DRM_FORMAT_GR88 = fourcc('G', 'R', '8', '8');

  // Matches the sentinel in graphics.cpp. RKMPP buffers are always linear, so we
  // import them via the implicit (no explicit modifier) EGL path, which is the
  // most broadly compatible across Mali/Panfrost drivers.
  constexpr std::uint64_t DRM_FORMAT_MOD_INVALID = (((std::uint64_t) 0 << 56) | (((1ULL << 56) - 1) & 0x00ffffffffffffffULL));

  class rkmpp_t: public platf::avcodec_encode_device_t {
  public:
    int init(int in_width, int in_height, file_t &&render_device, int offset_x, int offset_y) {
      file = std::move(render_device);

      if (!gbm::create_device) {
        BOOST_LOG(warning) << "libgbm not initialized"sv;
        return -1;
      }

      // Marker used by video.cpp to recognise this as a real hardware encode
      // device (rather than falling back to the CPU software encode device).
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

      return 0;
    }

    int set_frame(AVFrame *frame, AVBufferRef *hw_frames_ctx_buf) override {
      this->hwframe.reset(frame);
      this->frame = frame;

      if (!hw_frames_ctx_buf) {
        BOOST_LOG(error) << "RKMPP encode device requires a hardware frames context"sv;
        return -1;
      }

      // Allocate an NV12 frame from the RKMPP pool. The backing MppBuffer is a
      // DMA-BUF we can both render into (via EGL) and submit to the encoder.
      if (!frame->buf[0]) {
        if (av_hwframe_get_buffer(hw_frames_ctx_buf, frame, 0)) {
          BOOST_LOG(error) << "Couldn't get an RKMPP hwframe"sv;
          return -1;
        }
      }

      auto hw_frames_ctx = (AVHWFramesContext *) hw_frames_ctx_buf->data;

      auto desc = (const AVDRMFrameDescriptor *) frame->data[0];
      if (!desc || desc->nb_objects < 1 || desc->nb_layers < 1) {
        BOOST_LOG(error) << "RKMPP hwframe has an unexpected DRM descriptor"sv;
        return -1;
      }
      if (desc->objects[0].fd < 0) {
        BOOST_LOG(error) << "RKMPP hwframe has an invalid DMA-BUF fd"sv;
        return -1;
      }

      const auto &layer = desc->layers[0];
      if (layer.nb_planes < 2) {
        BOOST_LOG(error) << "RKMPP hwframe is not a biplanar (NV12) surface"sv;
        return -1;
      }

      // The NV12 surface is a single DMA-BUF object with two planes (Y, UV).
      // EGL imports each plane as its own texture, so they share the same fd.
      // The nv12 target takes ownership of the duplicated fd (unused entries
      // default to -1).
      std::array<file_t, egl::nv12_img_t::num_fds> fds;

      int dmabuf_fd = dup(desc->objects[0].fd);
      if (dmabuf_fd < 0) {
        BOOST_LOG(error) << "Couldn't duplicate RKMPP DMA-BUF fd"sv;
        return -1;
      }
      fds[0] = dmabuf_fd;

      egl::surface_descriptor_t sd_y {};
      sd_y.width = frame->width;
      sd_y.height = frame->height;
      sd_y.fourcc = DRM_FORMAT_R8;
      sd_y.modifier = DRM_FORMAT_MOD_INVALID;
      std::fill_n(sd_y.fds, 4, -1);
      sd_y.fds[0] = dmabuf_fd;
      sd_y.offsets[0] = layer.planes[0].offset;
      sd_y.pitches[0] = layer.planes[0].pitch;

      egl::surface_descriptor_t sd_uv {};
      sd_uv.width = frame->width / 2;
      sd_uv.height = frame->height / 2;
      sd_uv.fourcc = DRM_FORMAT_GR88;
      sd_uv.modifier = DRM_FORMAT_MOD_INVALID;
      std::fill_n(sd_uv.fds, 4, -1);
      sd_uv.fds[0] = dmabuf_fd;
      sd_uv.offsets[0] = layer.planes[1].offset;
      sd_uv.pitches[0] = layer.planes[1].pitch;

      auto nv12_opt = egl::import_target(display.get(), std::move(fds), sd_y, sd_uv);
      if (!nv12_opt) {
        return -1;
      }

      auto sws_opt = egl::sws_t::make(width, height, frame->width, frame->height, (AVPixelFormat) hw_frames_ctx->sw_format);
      if (!sws_opt) {
        return -1;
      }

      this->sws = std::move(*sws_opt);
      this->nv12 = std::move(*nv12_opt);

      return 0;
    }

    void apply_colorspace() override {
      sws.apply_colorspace(colorspace);
    }

    int convert(platf::img_t &img) override {
      auto &descriptor = (egl::img_descriptor_t &) img;

      if (descriptor.sequence == 0) {
        // For dummy images, use a blank RGB texture instead of importing a DMA-BUF
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

      sws.load_vram(descriptor, offset_x, offset_y, rgb->tex[0]);

      if (sws.convert(nv12->buf)) {
        return -1;
      }

      // The encoder (MPP) reads this DMA-BUF directly with no implicit fence
      // against the GPU. Ensure the RGB->NV12 render is complete before the
      // buffer is handed to the encoder to avoid reading a partial frame.
      gl::ctx.Finish();

      return 0;
    }

    file_t file;

    gbm::gbm_t gbm;
    egl::display_t display;
    egl::ctx_t ctx;

    // This must be destroyed before display_t to ensure the GPU driver is still
    // loaded when the underlying buffers are released.
    frame_t hwframe;

    egl::sws_t sws;
    egl::nv12_t nv12;
    egl::rgb_t rgb;

    int width, height;
    int offset_x, offset_y;
    std::uint64_t sequence;
  };

  int rkmpp_init_avcodec_hardware_input_buffer(platf::avcodec_encode_device_t * /* base */, AVBufferRef **hw_device_buf) {
    // The RKMPP device talks to the MPP service directly; it does not take a DRI
    // render-node path, so no device string is passed.
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
