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
#include <cstring>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <linux/dma-buf.h>

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
  class rkmpp_t: public platf::avcodec_encode_device_t {
  public:
    ~rkmpp_t() override {
      if (mapped && mapped != MAP_FAILED) {
        munmap(mapped, mapped_size);
      }
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

      BOOST_LOG(info) << "Using RKMPP GPU-convert + read-back encode path"sv;

      return 0;
    }

    int set_frame(AVFrame *frame, AVBufferRef *hw_frames_ctx_buf) override {
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

      auto desc = (const AVDRMFrameDescriptor *) frame->data[0];
      if (!desc || desc->nb_objects < 1 || desc->nb_layers < 1 || desc->layers[0].nb_planes < 2) {
        BOOST_LOG(error) << "RKMPP hwframe is not a biplanar NV12 DMA-BUF"sv;
        return -1;
      }

      dmabuf_fd = desc->objects[0].fd;
      mapped_size = desc->objects[0].size;
      y_offset = desc->layers[0].planes[0].offset;
      y_pitch = desc->layers[0].planes[0].pitch;
      uv_offset = desc->layers[0].planes[1].offset;
      uv_pitch = desc->layers[0].planes[1].pitch;

      // The encoder buffer is filled from the CPU via glGetTextureSubImage, so
      // map it for writing.
      mapped = mmap(nullptr, mapped_size, PROT_READ | PROT_WRITE, MAP_SHARED, dmabuf_fd, 0);
      if (mapped == MAP_FAILED) {
        char string[1024];
        BOOST_LOG(error) << "Couldn't mmap RKMPP encoder buffer: ["sv << strerror_r(errno, string, sizeof(string)) << ']';
        return -1;
      }

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

      return 0;
    }

    void apply_colorspace() override {
      sws.apply_colorspace(colorspace);
    }

    int convert(platf::img_t &img) override {
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

      sws.load_vram(descriptor, offset_x, offset_y, rgb->tex[0]);

      if (sws.convert(nv12->buf)) {
        return -1;
      }

      // Read the converted NV12 planes back into the encoder's DMA-BUF.
      sync_dma_buf(DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE);

      gl::ctx.PixelStorei(GL_PACK_ALIGNMENT, 1);

      // Y plane (R8): one byte per pixel, destination stride is y_pitch bytes.
      gl::ctx.PixelStorei(GL_PACK_ROW_LENGTH, y_pitch);
      gl::ctx.GetTextureSubImage(
        nv12->tex[0], 0, 0, 0, 0,
        frame->width, frame->height, 1,
        GL_RED, GL_UNSIGNED_BYTE,
        (int) (mapped_size - y_offset), (std::uint8_t *) mapped + y_offset
      );

      // UV plane (RG8): two bytes per pixel, half resolution.
      gl::ctx.PixelStorei(GL_PACK_ROW_LENGTH, uv_pitch / 2);
      gl::ctx.GetTextureSubImage(
        nv12->tex[1], 0, 0, 0, 0,
        frame->width / 2, frame->height / 2, 1,
        GL_RG, GL_UNSIGNED_BYTE,
        (int) (mapped_size - uv_offset), (std::uint8_t *) mapped + uv_offset
      );

      gl::ctx.PixelStorei(GL_PACK_ROW_LENGTH, 0);

      sync_dma_buf(DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE);

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

    void *mapped {nullptr};
    std::size_t mapped_size {};
    int dmabuf_fd {-1};
    std::ptrdiff_t y_offset {}, uv_offset {};
    int y_pitch {}, uv_pitch {};

    int width {}, height {};
    int offset_x {}, offset_y {};
    std::uint64_t sequence {};

  private:
    void sync_dma_buf(std::uint64_t flags) {
      struct dma_buf_sync sync {};
      sync.flags = flags;
      ioctl(dmabuf_fd, DMA_BUF_IOCTL_SYNC, &sync);
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
