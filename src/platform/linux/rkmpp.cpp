/**
 * @file src/platform/linux/rkmpp.cpp
 * @brief Zero-copy Rockchip MPP (RKMPP) encode device using RGA for scaling.
 *
 * RKMPP encodes in hardware, but feeding it software frames forces Sunshine to
 * do the capture color conversion and scaling on the CPU (libswscale), which
 * dominates CPU usage while streaming.
 *
 * The Mali GPU cannot render into the MPP-allocated encoder buffer (Panfrost
 * refuses to use an imported DMA-BUF as a multi-plane FBO colour attachment,
 * which produced an all-green image). So instead this device uses the RGA 2D
 * engine: it crops+scales the captured RGB framebuffer DMA-BUF straight into an
 * RGB DMA-BUF allocated from FFmpeg's RKMPP hardware frames pool, then hands
 * that buffer to h264_rkmpp / hevc_rkmpp as an AV_PIX_FMT_DRM_PRIME frame. The
 * VPU performs the RGB->YUV colour conversion internally during encode.
 *
 * Result: capture -> RGA (crop/scale, no CPU) -> VPU (CSC + encode), with no CPU
 * colour conversion and no GPU readback.
 */
// standard includes
#include <cstdint>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/hwcontext.h>
#include <libavutil/hwcontext_drm.h>
#include <libavutil/pixdesc.h>
}

#include <rga/im2d.h>
#include <rga/rga.h>

// local includes
#include "graphics.h"  // egl::img_descriptor_t / surface_descriptor_t (capture descriptor types)
#include "misc.h"
#include "rkmpp.h"
#include "src/logging.h"
#include "src/platform/common.h"
#include "src/utility.h"
#include "src/video.h"

using namespace std::literals;

namespace rkmpp {
  constexpr std::uint32_t fourcc(char a, char b, char c, char d) {
    return (std::uint32_t) a | ((std::uint32_t) b << 8) | ((std::uint32_t) c << 16) | ((std::uint32_t) d << 24);
  }

  // DRM packed 32-bit RGB fourccs (matches drm_fourcc.h).
  constexpr std::uint32_t DRM_FORMAT_XRGB8888 = fourcc('X', 'R', '2', '4');
  constexpr std::uint32_t DRM_FORMAT_ARGB8888 = fourcc('A', 'R', '2', '4');
  constexpr std::uint32_t DRM_FORMAT_XBGR8888 = fourcc('X', 'B', '2', '4');
  constexpr std::uint32_t DRM_FORMAT_ABGR8888 = fourcc('A', 'B', '2', '4');

  /**
   * @brief Map a DRM packed-RGB fourcc to the matching RGA format.
   *
   * DRM fourccs name channels MSB->LSB of a little-endian 32-bit word, so e.g.
   * XRGB8888 is stored in memory as B,G,R,X -> RK_FORMAT_BGRA_8888.
   */
  static int drm_fourcc_to_rga(std::uint32_t drm_fourcc) {
    switch (drm_fourcc) {
      case DRM_FORMAT_XRGB8888:
      case DRM_FORMAT_ARGB8888:
        return RK_FORMAT_BGRA_8888;
      case DRM_FORMAT_XBGR8888:
      case DRM_FORMAT_ABGR8888:
        return RK_FORMAT_RGBA_8888;
      default:
        // Sunshine treats the KMS framebuffer as BGRx, so default to that.
        return RK_FORMAT_BGRA_8888;
    }
  }

  class rkmpp_t: public platf::avcodec_encode_device_t {
  public:
    int init(int in_width, int in_height, int offset_x, int offset_y) {
      // Marker so video.cpp uses this device instead of the CPU software path.
      this->data = (void *) rkmpp_init_avcodec_hardware_input_buffer;

      // Source crop region within the captured framebuffer.
      this->crop_width = in_width;
      this->crop_height = in_height;
      this->offset_x = offset_x;
      this->offset_y = offset_y;

      BOOST_LOG(info) << "Using RGA zero-copy RKMPP encode path"sv;

      return 0;
    }

    int set_frame(AVFrame *frame, AVBufferRef *hw_frames_ctx_buf) override {
      this->hwframe.reset(frame);
      this->frame = frame;

      if (!hw_frames_ctx_buf) {
        BOOST_LOG(error) << "RKMPP encode device requires a hardware frames context"sv;
        return -1;
      }

      // Allocate the destination RGB buffer (a DMA-BUF backed MppBuffer) that
      // both RGA writes into and the encoder reads from.
      if (!frame->buf[0]) {
        if (av_hwframe_get_buffer(hw_frames_ctx_buf, frame, 0)) {
          BOOST_LOG(error) << "Couldn't get an RKMPP hwframe"sv;
          return -1;
        }
      }

      auto desc = (const AVDRMFrameDescriptor *) frame->data[0];
      if (!desc || desc->nb_objects < 1 || desc->nb_layers < 1) {
        BOOST_LOG(error) << "RKMPP hwframe has an unexpected DRM descriptor"sv;
        return -1;
      }
      if (desc->objects[0].fd < 0) {
        BOOST_LOG(error) << "RKMPP hwframe has an invalid DMA-BUF fd"sv;
        return -1;
      }

      dst_fd = desc->objects[0].fd;
      dst_width = frame->width;
      dst_height = frame->height;
      // Pitch is in bytes; RGA strides are in pixels (32 bpp packed RGB).
      dst_wstride = desc->layers[0].planes[0].pitch / 4;
      dst_format = drm_fourcc_to_rga(desc->layers[0].format);

      return 0;
    }

    int convert(platf::img_t &img) override {
      auto &descriptor = (egl::img_descriptor_t &) img;

      // Dummy/probe images (sequence 0) and cursor-only frames have no captured
      // framebuffer. Leave the destination buffer as-is (encodes as black).
      if (descriptor.sequence == 0 || descriptor.sd.fds[0] < 0) {
        return 0;
      }

      const auto &sd = descriptor.sd;

      if (!logged_src) {
        logged_src = true;
        BOOST_LOG(info) << "RGA source framebuffer: "sv << sd.width << 'x' << sd.height
                        << " fourcc=0x"sv << util::hex(sd.fourcc).to_string_view()
                        << " modifier=0x"sv << util::hex(sd.modifier).to_string_view()
                        << " pitch="sv << sd.pitches[0]
                        << " -> dst "sv << dst_width << 'x' << dst_height << " wstride="sv << dst_wstride;
      }

      im_handle_param_t src_param {
        (uint32_t) (sd.pitches[0] / 4),
        (uint32_t) sd.height,
        (uint32_t) drm_fourcc_to_rga(sd.fourcc)
      };
      im_handle_param_t dst_param {
        (uint32_t) dst_wstride,
        (uint32_t) dst_height,
        (uint32_t) dst_format
      };

      rga_buffer_handle_t src_handle = importbuffer_fd(sd.fds[0], &src_param);
      rga_buffer_handle_t dst_handle = importbuffer_fd(dst_fd, &dst_param);
      if (!src_handle || !dst_handle) {
        BOOST_LOG(error) << "RGA: failed to import DMA-BUF (src="sv << sd.fds[0] << ", dst="sv << dst_fd << ')';
        if (src_handle) {
          releasebuffer_handle(src_handle);
        }
        if (dst_handle) {
          releasebuffer_handle(dst_handle);
        }
        return -1;
      }

      rga_buffer_t src = wrapbuffer_handle_t(src_handle, sd.width, sd.height, sd.pitches[0] / 4, sd.height, drm_fourcc_to_rga(sd.fourcc));
      rga_buffer_t dst = wrapbuffer_handle_t(dst_handle, dst_width, dst_height, dst_wstride, dst_height, dst_format);
      rga_buffer_t pat {};

      // Crop the captured region, scale it to the encoder dimensions.
      im_rect src_rect {offset_x, offset_y, crop_width, crop_height};
      im_rect dst_rect {0, 0, dst_width, dst_height};
      im_rect pat_rect {0, 0, 0, 0};

      auto status = improcess(src, dst, pat, src_rect, dst_rect, pat_rect, 0, nullptr, nullptr, IM_SYNC);

      releasebuffer_handle(src_handle);
      releasebuffer_handle(dst_handle);

      if (status != IM_STATUS_SUCCESS) {
        BOOST_LOG(error) << "RGA conversion failed: "sv << imStrError_t(status);
        return -1;
      }

      return 0;
    }

    // Owns the encoder input frame for the session lifetime.
    frame_t hwframe;

    int dst_fd {-1};
    int dst_width {}, dst_height {}, dst_wstride {}, dst_format {};

    int crop_width {}, crop_height {};
    int offset_x {}, offset_y {};

    bool logged_src {false};
  };

  int rkmpp_init_avcodec_hardware_input_buffer(platf::avcodec_encode_device_t * /* base */, AVBufferRef **hw_device_buf) {
    // The RKMPP device talks to the MPP service directly; no DRI node path.
    auto status = av_hwdevice_ctx_create(hw_device_buf, AV_HWDEVICE_TYPE_RKMPP, nullptr, nullptr, 0);
    if (status < 0) {
      char string[AV_ERROR_MAX_STRING_SIZE];
      BOOST_LOG(error) << "Failed to create an RKMPP device: "sv << av_make_error_string(string, AV_ERROR_MAX_STRING_SIZE, status);
      return -1;
    }

    return 0;
  }

  std::unique_ptr<platf::avcodec_encode_device_t> make_avcodec_encode_device(int width, int height, int offset_x, int offset_y) {
    auto device = std::make_unique<rkmpp_t>();

    if (device->init(width, height, offset_x, offset_y)) {
      return nullptr;
    }

    return device;
  }
}  // namespace rkmpp
