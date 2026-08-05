/**
 * @file src/platform/linux/rkmpp.cpp
 * @brief Rockchip MPP (RKMPP) encode device.
 *
 * Two capture paths feed the RKMPP encoder:
 *
 *  - retro-capture socket client (the console path): a standalone daemon
 *    (~/retro-capture) owns the HDMI-RX V4L2 device, absorbs every input
 *    transition and driver wedge internally, and delivers NV12 frames at the
 *    stream geometry as dma-buf fds over a SEQPACKET Unix socket
 *    (retro-capture-protocol.h, MIT, vendored — the protocol spec lives in
 *    the retro-capture repo's PROTOCOL.md). This file wraps the received
 *    pool buffers as DRM_PRIME AVFrames; zero-copy end to end.
 *
 *    The V4L2 capture engine, driver-recovery ladder, RGA scaler, and CRT
 *    shader that used to live here (~3200 lines) were migrated INTO that
 *    daemon; see the retro-capture repo for the port map (its DESIGN.md §6
 *    references this file's history at commit 2f1e6a43 for archaeology, and
 *    for the future UVC/MJPEG backend notes).
 *
 *  - KMS GPU-convert + readback (the desktop path): capture from KMS,
 *    convert RGB->NV12 on the Mali into a native render target, read back
 *    into the encoder's mapped buffer. Used when no capture daemon socket is
 *    available.
 */
// standard includes
#include <chrono>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

// lib includes
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/hwcontext.h>
#include <libavutil/hwcontext_drm.h>
#include <libavutil/imgutils.h>
#include <libavutil/pixdesc.h>
}

// local includes
#include "src/audio_gate.h"
#include "graphics.h"
#include "misc.h"
#include "retro-capture-protocol.h"
#include "retro-overlay.h"
#include "rkmpp.h"
#include "src/logging.h"
#include "src/platform/common.h"
#include "src/utility.h"
#include "src/video.h"

using namespace std::literals;

namespace rkmpp {

  static std::string fourcc_str(std::uint32_t fourcc) {
    const char s[5] = {
      (char) (fourcc & 0xff),
      (char) ((fourcc >> 8) & 0xff),
      (char) ((fourcc >> 16) & 0xff),
      (char) ((fourcc >> 24) & 0xff),
      0,
    };
    return s;
  }

  // Thin client for the retro-capture daemon. Consumer-agnostic protocol; the
  // only Sunshine-specific piece is wrapping pool buffers as DRM_PRIME
  // AVFrames for the RKMPP encoder (the shape rga_scaler_t::enc_frame() used
  // to build around its own CMA buffer).
  class capture_client_t {
  public:
    ~capture_client_t() {
      for (auto &w : wrappers) {
        if (w) {
          av_frame_free(&w);
        }
      }
      for (auto fd : pool_fds) {
        if (fd >= 0) {
          ::close(fd);
        }
      }
      if (sock >= 0) {
        ::close(sock);
      }
      if (hw_frames_ctx) {
        av_buffer_unref(&hw_frames_ctx);
      }
    }

    static const char *socket_path() {
      const char *p = std::getenv("RETRO_CAPTURE_SOCKET");
      return (p && *p) ? p : RCAP_SOCKET_DEFAULT;
    }

    // Connect + HELLO + SETUP + STREAM_INFO + pool import. Returns nullptr on
    // any failure (caller falls back to the KMS path).
    static std::unique_ptr<capture_client_t> connect(int width, int height, AVBufferRef *hw_frames_ctx_buf) {
      auto client = std::make_unique<capture_client_t>();

      client->sock = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
      if (client->sock < 0) {
        return nullptr;
      }
      sockaddr_un addr {};
      addr.sun_family = AF_UNIX;
      std::strncpy(addr.sun_path, socket_path(), sizeof(addr.sun_path) - 1);
      if (::connect(client->sock, (sockaddr *) &addr, sizeof(addr)) < 0) {
        BOOST_LOG(info) << "retro-capture: no daemon at "sv << socket_path()
                        << " ("sv << strerror(errno) << "); using KMS capture"sv;
        return nullptr;
      }

      rcap_hello hello {};
      hello.hdr = {RCAP_MSG_HELLO, 0};
      hello.magic = RCAP_MAGIC;
      hello.ver_min = RCAP_PROTO_VERSION;
      hello.ver_max = RCAP_PROTO_VERSION;
      hello.role = RCAP_ROLE_CONSUMER;
      std::strncpy(hello.name, "sunshine", sizeof(hello.name) - 1);
      if (!client->send_msg(&hello, sizeof(hello))) {
        return nullptr;
      }

      std::uint8_t buf[RCAP_MAX_MSG_SIZE];
      if (client->recv_timeout(buf, sizeof(buf), 3000) < (ssize_t) sizeof(rcap_hello_ack) ||
          ((rcap_hdr *) buf)->type != RCAP_MSG_HELLO_ACK) {
        BOOST_LOG(warning) << "retro-capture: daemon refused HELLO"sv;
        return nullptr;
      }

      // The session's bit depth decides the pool format we request: the
      // encoder was created for the frames context's sw_format, and an
      // 8-bit session must get NV12 even while the daemon captures 10-bit
      // (the daemon downconverts). SETUP v2 length-extension carries it.
      bool want_nv15 = false;
      if (hw_frames_ctx_buf) {
        auto *fctx = (AVHWFramesContext *) hw_frames_ctx_buf->data;
        want_nv15 = fctx->sw_format == AV_PIX_FMT_NV15;
      }
      rcap_setup2 setup {};
      setup.base.hdr = {RCAP_MSG_SETUP, 0};
      setup.base.width = width;
      setup.base.height = height;
      setup.base.fps_num = 0;  // daemon default cadence; the encoder paces itself
      setup.base.fps_den = 0;
      setup.fourcc = want_nv15 ? 0x3531564e /* DRM_FORMAT_NV15 */ : 0x3231564e /* NV12 */;
      if (!client->send_msg(&setup, sizeof(setup))) {
        return nullptr;
      }

      // STREAM_INFO then pool_count POOL_ADD messages (one fd each).
      bool have_info = false;
      while (!have_info || client->pool_fds.size() < client->sinfo.pool_count) {
        iovec iov {buf, sizeof(buf)};
        char cbuf[CMSG_SPACE(sizeof(int))];
        msghdr mh {};
        mh.msg_iov = &iov;
        mh.msg_iovlen = 1;
        mh.msg_control = cbuf;
        mh.msg_controllen = sizeof(cbuf);
        pollfd pfd {client->sock, POLLIN, 0};
        if (poll(&pfd, 1, 3000) <= 0) {
          BOOST_LOG(warning) << "retro-capture: timeout waiting for stream setup"sv;
          return nullptr;
        }
        ssize_t n = recvmsg(client->sock, &mh, 0);
        if (n < (ssize_t) sizeof(rcap_hdr)) {
          return nullptr;
        }
        auto *hdr = (rcap_hdr *) buf;
        if (hdr->type == RCAP_MSG_STREAM_INFO && n >= (ssize_t) sizeof(rcap_stream_info)) {
          client->sinfo = *(rcap_stream_info *) buf;
          have_info = true;
        } else if (hdr->type == RCAP_MSG_POOL_ADD && n >= (ssize_t) sizeof(rcap_pool_add)) {
          int fd = -1;
          for (cmsghdr *cm = CMSG_FIRSTHDR(&mh); cm; cm = CMSG_NXTHDR(&mh, cm)) {
            if (cm->cmsg_level == SOL_SOCKET && cm->cmsg_type == SCM_RIGHTS) {
              std::memcpy(&fd, CMSG_DATA(cm), sizeof(int));
            }
          }
          auto *add = (rcap_pool_add *) buf;
          if (fd < 0 || add->index != client->pool_fds.size()) {
            BOOST_LOG(warning) << "retro-capture: bad POOL_ADD"sv;
            return nullptr;
          }
          client->pool_fds.push_back(fd);
        } else if (hdr->type == RCAP_MSG_ERROR) {
          BOOST_LOG(warning) << "retro-capture: daemon error "sv << ((rcap_error *) buf)->code;
          return nullptr;
        }
      }

      if ((int) client->sinfo.width != width || (int) client->sinfo.height != height) {
        BOOST_LOG(warning) << "retro-capture: daemon stream "sv << client->sinfo.width << 'x'
                           << client->sinfo.height << " != requested "sv << width << 'x' << height;
        return nullptr;
      }

      // STREAM_INFO is authoritative: the encoder was configured for the
      // requested layout, so a different pool format (old daemon, or NV15
      // requested while the capture just left 10-bit) is unusable.
      if (client->sinfo.fourcc != setup.fourcc) {
        BOOST_LOG(warning) << "retro-capture: daemon served pool format "sv
                           << fourcc_str(client->sinfo.fourcc) << " but this session needs "sv
                           << fourcc_str(setup.fourcc) << "; not using the daemon path"sv;
        return nullptr;
      }

      client->hw_frames_ctx = av_buffer_ref(hw_frames_ctx_buf);
      client->wrappers.assign(client->pool_fds.size(), nullptr);
      client->leased.assign(client->pool_fds.size(), false);

      BOOST_LOG(info) << "retro-capture: connected — "sv << client->sinfo.width << 'x'
                      << client->sinfo.height << ' ' << fourcc_str(client->sinfo.fourcc)
                      << " pool of "sv << client->sinfo.pool_count
                      << " (stride "sv << client->sinfo.stride_y << ", zero-copy DRM_PRIME encode)"sv;
      return client;
    }

    void release_lease(int index) {
      if (index < 0 || !leased[index]) {
        return;
      }
      rcap_release rel {{RCAP_MSG_RELEASE, 0}, (std::uint32_t) index, 0, 0};
      if (send_msg(&rel, sizeof(rel))) {
        leased[index] = false;
      }
    }

    // Drain the socket, keep the newest FRAME, release superseded leases, and
    // return the DRM_PRIME wrapper for the current buffer. Newest-frame-wins:
    // identical latency semantics to the old in-process update_latest_frame().
    // Returns nullptr only before the first frame or after daemon death.
    AVFrame *next_frame() {
      if (sock < 0) {
        return cur_index >= 0 ? wrapper_for(cur_index) : nullptr;
      }
      std::uint8_t buf[RCAP_MAX_MSG_SIZE];
      int newest = -1;
      std::uint32_t newest_flags = 0;
      for (;;) {
        ssize_t n = recv(sock, buf, sizeof(buf), MSG_DONTWAIT);
        if (n < 0) {
          if (errno == EAGAIN || errno == EWOULDBLOCK) {
            break;
          }
          if (errno == EINTR) {
            continue;
          }
          mark_dead("recv error");
          break;
        }
        if (n == 0) {
          mark_dead("daemon closed");
          break;
        }
        auto *hdr = (rcap_hdr *) buf;
        if (hdr->type == RCAP_MSG_FRAME && n >= (ssize_t) sizeof(rcap_frame)) {
          auto *f = (rcap_frame *) buf;
          if (f->index < pool_fds.size()) {
            if (!leased[f->index]) {
              leased[f->index] = true;  // lease created by first reference
            }
            // A frame superseded within this same drain must be released here:
            // it is neither cur_index nor the drain's winner, so the post-loop
            // release below never sees it, and the daemon cannot re-send an
            // index it still considers leased — each occurrence would
            // permanently shrink the 4-buffer pool.
            if (newest >= 0 && newest != (int) f->index && newest != cur_index) {
              release_lease(newest);
            }
            newest = (int) f->index;
            newest_flags = f->flags;
          }
        }
        // (ERROR after streaming started ends in EOF; nothing else expected.)
      }

      if (newest >= 0) {
        // In-process audio squelch across video transitions: the daemon
        // notifies retro-audio via its FIFO at the precise ladder points; the
        // protocol's flag transitions give this process the same signal for
        // its own outgoing-audio gate (replaces the triggers that lived in
        // the migrated capture code).
        const bool now_held = (newest_flags & (RCAP_FRAME_HELD | RCAP_FRAME_BLACK)) != 0;
        if (now_held != last_was_held) {
          audio_gate::trigger(audio_gate::reneg_ms.load(std::memory_order_relaxed), false);
          last_was_held = now_held;
        }
        if (cur_index != newest) {
          // Superseded: hand the buffer back so the daemon can write into it.
          release_lease(cur_index);
        }
        cur_index = newest;
      }
      return cur_index >= 0 ? wrapper_for(cur_index) : nullptr;
    }

    // Non-blocking reconnect pacing after daemon death: systemd restarts the
    // daemon within seconds; until then the encoder keeps re-sending the last
    // received buffer (the fds outlive the daemon process).
    void maybe_reconnect(int width, int height) {
      if (sock >= 0) {
        return;
      }
      const auto now = std::chrono::steady_clock::now();
      if (now - last_reconnect_at < std::chrono::seconds(1)) {
        return;
      }
      last_reconnect_at = now;
      auto fresh = connect(width, height, hw_frames_ctx);
      if (fresh) {
        BOOST_LOG(info) << "retro-capture: reconnected"sv;
        *this = std::move(*fresh);
      }
    }

    capture_client_t() = default;
    capture_client_t(capture_client_t &&) = default;
    capture_client_t &operator=(capture_client_t &&) = default;

  private:
    void mark_dead(const char *why) {
      if (sock >= 0) {
        BOOST_LOG(warning) << "retro-capture: "sv << why
                           << "; holding last frame and retrying connect"sv;
        ::close(sock);
        sock = -1;
      }
    }

    bool send_msg(const void *msg, std::size_t len) {
      ssize_t n;
      do {
        n = send(sock, msg, len, MSG_NOSIGNAL);
      } while (n < 0 && errno == EINTR);
      return n == (ssize_t) len;
    }

    ssize_t recv_timeout(void *buf, std::size_t len, int timeout_ms) {
      pollfd pfd {sock, POLLIN, 0};
      if (poll(&pfd, 1, timeout_ms) <= 0) {
        return -1;
      }
      return recv(sock, buf, len, 0);
    }

    // DRM_PRIME AVFrame around pool buffer `index` (built once, reused every
    // frame — same descriptor shape the old enc_frame() produced).
    AVFrame *wrapper_for(int index) {
      if (wrappers[index]) {
        return wrappers[index];
      }
      auto *desc = (AVDRMFrameDescriptor *) av_mallocz(sizeof(AVDRMFrameDescriptor));
      if (!desc) {
        return nullptr;
      }
      desc->nb_objects = 1;
      desc->objects[0].fd = pool_fds[index];
      desc->objects[0].size = sinfo.buffer_size;
      desc->objects[0].format_modifier = sinfo.modifier;
      desc->nb_layers = 1;
      desc->layers[0].format = sinfo.fourcc;  // DRM_FORMAT_NV12 / DRM_FORMAT_NV15
      desc->layers[0].nb_planes = 2;
      desc->layers[0].planes[0].object_index = 0;
      desc->layers[0].planes[0].offset = sinfo.offset_y;
      desc->layers[0].planes[0].pitch = sinfo.stride_y;
      desc->layers[0].planes[1].object_index = 0;
      desc->layers[0].planes[1].offset = sinfo.offset_uv;
      desc->layers[0].planes[1].pitch = sinfo.stride_uv;

      AVFrame *f = av_frame_alloc();
      if (!f) {
        av_free(desc);
        return nullptr;
      }
      f->format = AV_PIX_FMT_DRM_PRIME;
      f->width = sinfo.width;
      f->height = sinfo.height;
      f->data[0] = (std::uint8_t *) desc;
      f->buf[0] = av_buffer_create((std::uint8_t *) desc, sizeof(*desc),
                                   [](void *, std::uint8_t *d) {
                                     av_free(d);
                                   },
                                   nullptr, 0);
      if (hw_frames_ctx) {
        f->hw_frames_ctx = av_buffer_ref(hw_frames_ctx);
      }
      wrappers[index] = f;
      return f;
    }

    int sock {-1};
    rcap_stream_info sinfo {};
    std::vector<int> pool_fds;
    std::vector<AVFrame *> wrappers;
    std::vector<bool> leased;
    int cur_index {-1};
    bool last_was_held {true};  // stream primes with BLACK
    AVBufferRef *hw_frames_ctx {};
    std::chrono::steady_clock::time_point last_reconnect_at {};
  };

  class rkmpp_t: public platf::avcodec_encode_device_t {
  public:
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

      // SUNSHINE_RKMPP_V4L2 marks this host as a capture-streaming box; the
      // capture itself is owned by the retro-capture daemon (socket client
      // connected in set_frame once the stream geometry is known).
      capture_expected = std::getenv("SUNSHINE_RKMPP_V4L2") != nullptr;

      return 0;
    }

    int set_frame(AVFrame *frame, AVBufferRef *hw_frames_ctx_buf) override {
      // The capture display has its own EGL context that may be current on this
      // thread. Ensure our context is current before touching any GL object.
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

      if (capture_expected) {
        capture = capture_client_t::connect(frame->width, frame->height, hw_frames_ctx_buf);
        if (capture) {
          BOOST_LOG(info) << "Using retro-capture daemon encode path (zero-copy DRM_PRIME)"sv;
          return 0;
        }
        BOOST_LOG(warning) << "retro-capture daemon unavailable; falling back to KMS GPU-convert path"sv;
      }

      BOOST_LOG(info) << "Using RKMPP GPU-convert + read-back encode path"sv;

      // Native NV12 render target the GPU converts into.
      auto nv12_opt = egl::create_nv12_target(frame->width, frame->height, (AVPixelFormat) AV_PIX_FMT_NV12);
      if (!nv12_opt) {
        return -1;
      }

      auto sws_opt = egl::sws_t::make(width, height, frame->width, frame->height, (AVPixelFormat) AV_PIX_FMT_NV12, false);
      if (!sws_opt) {
        return -1;
      }

      this->nv12 = std::move(*nv12_opt);
      this->sws = std::move(*sws_opt);

      return 0;
    }

    void apply_colorspace() override {
      if (!capture) {
        sws.apply_colorspace(colorspace, false);
      }
    }

    int convert(platf::img_t &img) override {
      make_current();

      if (capture) {
        capture->maybe_reconnect(frame->width, frame->height);
        AVFrame *enc = capture->next_frame();
        if (!enc) {
          // Only reachable before the daemon's first (BLACK-primed) frame or
          // if the wrapper allocation failed.
          if (!missing_logged) {
            BOOST_LOG(warning) << "retro-capture: no frame available yet"sv;
            missing_logged = true;
          }
          return -1;
        }
        missing_logged = false;
        this->frame = enc;
        // Encoder OSD (stream overlay): attach/remove side data on the
        // wrapper. No pixel work; hidden overlay attaches nothing.
        auto *desc = (AVDRMFrameDescriptor *) enc->data[0];
        rovl::attach(enc, enc->width, enc->height,
                     desc->layers[0].format == 0x3531564e /* DRM_FORMAT_NV15 */);
        return 0;
      }

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
        sws.load_vram(descriptor, offset_x, offset_y, rgb->tex[0], false);
      } else {
        // The Rockchip/Panfrost path can sample imported KMS DMA-BUF textures,
        // but using them as an FBO source for Sunshine's resize copy can fail
        // with GL_INVALID_FRAMEBUFFER_OPERATION. Let the RGB->NV12 shader scale
        // directly from the imported texture instead.
        sws.load_vram_direct(rgb->tex[0]);
      }

      if (sws.convert_nv12(nv12->buf)) {
        return -1;
      }

      // Read the converted NV12 planes into FFmpeg's mapped RKMPP frame. The
      // map/unmap path uses RKMPP's own buffer pointer and cache-sync rules,
      // avoiding stale all-green frames from raw DMA-BUF mmap writes.

      gl::ctx.PixelStorei(GL_PACK_ALIGNMENT, 1);

      gl::ctx.PixelStorei(GL_PACK_ROW_LENGTH, mapped_frame->linesize[0]);
      gl::ctx.GetTextureSubImage(
        nv12->tex[0], 0, 0, 0, 0,
        frame->width, frame->height, 1,
        GL_RED, GL_UNSIGNED_BYTE,
        mapped_frame->linesize[0] * frame->height, mapped_frame->data[0]
      );

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
    bool capture_expected {};
    bool missing_logged {};
    std::unique_ptr<capture_client_t> capture;

  private:
    void make_current() {
      auto context = std::get<1>(ctx.el);
      if (eglGetCurrentContext() == context) {
        return;
      }
      eglMakeCurrent(display.get(), EGL_NO_SURFACE, EGL_NO_SURFACE, context);
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

  // ---- daemon HDR surface (10-bit HDR project) ----

  bool daemon_hdr_active() {
    if (!std::getenv("SUNSHINE_RKMPP_V4L2")) {
      return false;  // not a capture-streaming box
    }

    // Callers run at session-setup/probe cadence; cache briefly so repeated
    // is_hdr() checks within one setup don't each round-trip the socket.
    static std::mutex mu;
    static std::chrono::steady_clock::time_point checked_at {};
    static bool cached = false;
    std::lock_guard lk(mu);
    const auto now = std::chrono::steady_clock::now();
    if (checked_at.time_since_epoch().count() != 0 && now - checked_at < 1s) {
      return cached;
    }
    checked_at = now;
    cached = false;

    const char *env = std::getenv("RETRO_CAPTURE_SOCKET");
    const char *path = (env && *env) ? env : RCAP_SOCKET_DEFAULT;
    int sock = ::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
    if (sock < 0) {
      return false;
    }
    sockaddr_un addr {};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);

    std::uint8_t buf[RCAP_MAX_MSG_SIZE];
    const auto recv_reply = [&]() -> ssize_t {
      pollfd pfd {sock, POLLIN, 0};
      if (poll(&pfd, 1, 250) <= 0) {
        return -1;
      }
      return ::recv(sock, buf, sizeof(buf), 0);
    };

    do {
      if (::connect(sock, (sockaddr *) &addr, sizeof(addr)) < 0) {
        break;
      }
      rcap_hello hello {};
      hello.hdr = {RCAP_MSG_HELLO, 0};
      hello.magic = RCAP_MAGIC;
      hello.ver_min = 1;
      hello.ver_max = RCAP_PROTO_VERSION;
      hello.role = RCAP_ROLE_STATUS;
      std::strncpy(hello.name, "sunshine-hdr", sizeof(hello.name) - 1);
      if (::send(sock, &hello, sizeof(hello), MSG_NOSIGNAL) != (ssize_t) sizeof(hello)) {
        break;
      }
      ssize_t n = recv_reply();
      if (n < (ssize_t) sizeof(rcap_hdr) || ((rcap_hdr *) buf)->type != RCAP_MSG_HELLO_ACK) {
        break;
      }
      rcap_status_get get {{RCAP_MSG_STATUS_GET, 0}};
      if (::send(sock, &get, sizeof(get), MSG_NOSIGNAL) != (ssize_t) sizeof(get)) {
        break;
      }
      n = recv_reply();
      if (n <= (ssize_t) sizeof(rcap_hdr) || ((rcap_hdr *) buf)->type != RCAP_MSG_STATUS) {
        break;
      }
      buf[n - 1] = 0;  // defensive; STATUS is NUL-terminated within the datagram
      // PROTOCOL.md 3.10: input.bit_depth is 10 iff the capture is NV15,
      // which the daemon defines as BT.2020+PQ content (the MVP contract).
      cached = std::strstr((const char *) buf + sizeof(rcap_hdr), "\"bit_depth\":10") != nullptr;
    } while (false);
    ::close(sock);
    return cached;
  }

  bool daemon_hdr_metadata(SS_HDR_METADATA &metadata) {
    if (!daemon_hdr_active()) {
      return false;
    }
    // Standard HDR10 defaults (CTA-861.3 units, matching txctl): BT.2020
    // primaries, D65 white point, 1000/0.005-nit mastering display,
    // MaxCLL 1000, MaxFALL 400. Moonlight clients tone-map from these.
    metadata = {};
    metadata.displayPrimaries[0] = {35400, 14600};  // R (0.708, 0.292)
    metadata.displayPrimaries[1] = {8500, 39850};  // G (0.170, 0.797)
    metadata.displayPrimaries[2] = {6550, 2300};  // B (0.131, 0.046)
    metadata.whitePoint = {15635, 16450};  // D65 (0.3127, 0.3290)
    metadata.maxDisplayLuminance = 1000;  // nits
    metadata.minDisplayLuminance = 50;  // 0.0001-nit units -> 0.005 nits
    metadata.maxContentLightLevel = 1000;
    metadata.maxFrameAverageLightLevel = 400;
    return true;
  }
}  // namespace rkmpp
