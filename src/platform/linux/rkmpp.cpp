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
#include <limits>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <fcntl.h>
#include <poll.h>
#include <chrono>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <thread>
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

#ifdef SUNSHINE_BUILD_RGA
#include <drm_fourcc.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <rga/rga.h>
#include <rga/im2d.hpp>
#endif

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
    cubic,
    average,
  };

  enum class line_blend_e {
    automatic,
    disabled,
    enabled,
  };

  // Post-process effect applied by the GPU scaler while it scales/converts.
  // The RGA 2D engine is fixed-function (no programmable stage), so any effect
  // forces the GPU scaler path and disables the capture passthrough fast path.
  enum class shader_effect_e {
    disabled,
    crt_basic,
  };

  // CRT effect tuning (SUNSHINE_RKMPP_CRT_PARAMS). Ranges are clamped at parse
  // time; the retro-stream web UI produces the same token format.
  struct crt_params_t {
    float scan {0.35f};  // scanline strength, 0..1
    float sharp {0.4f};  // beam sharpness, 0..1 (maps to a pow() exponent)
    float mask {0.12f};  // aperture-grille strength, 0..1
    float bright {0.15f};  // brightness gain compensating scanline/mask loss, 0..0.5
    float soft {0.0f};  // horizontal softness, 0..1
    float pitch {3.0f};  // grille period in output pixels, 2..4
    bool fade {true};  // fade scanlines out near 1:1 vertical scale (moiré guard)
  };

  struct view_region_t {
    float left {};
    float top {};
    float right {1.0f};
    float bottom {1.0f};
    bool enabled {};
  };

  struct direct_frame_t {
    const std::uint8_t *data {};
    std::uint32_t fourcc {};
    int width {};
    int height {};
    int stride {};
    bool nv12 {};
    // dma-buf export of the V4L2 buffer holding this frame (-1 when the frame
    // lives in CPU memory, e.g. decoded MJPEG, or EXPBUF is unsupported).
    // Lets the RGA path read the capture without a CPU copy — V4L2 mmap
    // memory is uncached, and memcpying a 1080p frame out of it per frame
    // costs more CPU than the entire rest of the pipeline.
    int dmabuf_fd {-1};
    std::uint32_t dmabuf_size {};
  };

  struct scale_region_t {
    int src_left {};
    int src_top {};
    int src_w {};
    int src_h {};
    int off_x {};
    int off_y {};
    int out_w {};
    int out_h {};
  };

  // A linked GPU program together with its uniform locations, resolved once at
  // link time. glGetUniformLocation does a string lookup plus driver-side
  // validation, so caching avoids repeating it for every draw of every frame.
  struct direct_program_t {
    gl::program_t program;
    GLint image_loc {-1};
    GLint rect_loc {-1};
    GLint width_loc {-1};
    GLint crt_loc {-1};
    GLint crt2_loc {-1};
  };

  class v4l2_nv12_source_t {
  public:
    ~v4l2_nv12_source_t() {
      stop();
    }

    // The Rockchip HDMI-RX follows the incoming signal, but the vendor driver
    // only re-latches its DV timings reliably on a hotplug edge. Behind an
    // HDMI switch there is none: a source sleeping or an input change swaps
    // the signal in-band — often through an idle carrier without AVI
    // InfoFrames, where the driver's own format-change path gives up
    // (`avi_pkt_rcv failed`) — leaving the configured timings stale and the
    // capture black even after the source returns. Re-latch here so every
    // session starts against the live signal. No-op for devices without DV
    // timings support (USB capture cards), when no stable signal is present,
    // or when the timings already match.
    void sync_dv_timings() {
      // Right after a source appears the query can fail transiently: the PHY
      // already has lock but the AVI InfoFrame hasn't been received yet, and
      // the vendor driver then fails the whole query (`wait avi_pkt_rcv
      // failed`). The InfoFrame repeats every video frame, so a couple of
      // short retries ride out the race; without them this replay adopts
      // nothing, delivers no frames, and the fruitless-replay hold in
      // maybe_recover_source delays the next attempt.
      v4l2_dv_timings live {};
      bool measured = false;
      for (int attempt = 0; attempt < 3; ++attempt) {
        if (attempt > 0) {
          std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
        const int query_err = xioctl(fd, VIDIOC_QUERY_DV_TIMINGS, &live) == 0 ? 0 : errno;
        if (query_err == ENOTTY) {
          return;  // no DV timings support (USB capture cards)
        }
        if (query_err == 0 && live.type == V4L2_DV_BT_656_1120 &&
            live.bt.width > 0 && live.bt.height > 0) {
          measured = true;
          break;
        }
      }
      if (!measured) {
        return;  // no stable signal to latch onto
      }

      const auto clock_close = [](std::uint64_t a, std::uint64_t b) {  // within 1% (59.94 vs 60 nominal)
        const auto diff = a > b ? a - b : b - a;
        return diff * 100 <= b;
      };
      v4l2_dv_timings cur {};
      const bool have_cur = xioctl(fd, VIDIOC_G_DV_TIMINGS, &cur) == 0;
      if (have_cur &&
          cur.bt.width == live.bt.width && cur.bt.height == live.bt.height &&
          cur.bt.interlaced == live.bt.interlaced &&
          (live.bt.pixelclock == 0 || clock_close(live.bt.pixelclock, cur.bt.pixelclock))) {
        return;
      }

      // S_DV_TIMINGS validates strictly against the driver's mode table, and
      // QUERY's measurement is not table-exact: mid-lock it can omit the
      // pixelclock entirely, and 1000/1001-rate signals measure ~0.1% off the
      // table's nominal clock. Resolve the measurement to a canonical table
      // entry: match geometry (and totals/clock when measured), and prefer the
      // highest clock among the remainder — a bare 2200x1125 measurement is
      // 1080p60 far more often than 1080p30 on the sources behind the switch.
      const auto totals = [](const v4l2_bt_timings &bt) {
        return std::make_pair(
          bt.width + bt.hfrontporch + bt.hsync + bt.hbackporch,
          bt.height + bt.vfrontporch + bt.vsync + bt.vbackporch +
            (bt.interlaced ? bt.il_vfrontporch + bt.il_vsync + bt.il_vbackporch : 0));
      };
      const auto live_totals = totals(live.bt);
      const bool have_live_totals = live_totals.first > live.bt.width && live_totals.second > live.bt.height;

      v4l2_dv_timings pick {};
      bool picked = false;
      v4l2_enum_dv_timings ent {};
      for (ent.index = 0; xioctl(fd, VIDIOC_ENUM_DV_TIMINGS, &ent) == 0; ++ent.index) {
        const auto &bt = ent.timings.bt;
        if (bt.width != live.bt.width || bt.height != live.bt.height ||
            bt.interlaced != live.bt.interlaced) {
          continue;
        }
        if (have_live_totals && totals(bt) != live_totals) {
          continue;
        }
        if (live.bt.pixelclock > 0 && !clock_close(live.bt.pixelclock, bt.pixelclock)) {
          continue;  // >1% off: different mode
        }
        if (!picked || bt.pixelclock > pick.bt.pixelclock) {
          pick = ent.timings;
          picked = true;
        }
      }
      if (!picked) {
        if (live.bt.pixelclock == 0) {
          BOOST_LOG(warning) << "RKMPP direct V4L2: live signal "sv << live.bt.width << 'x' << live.bt.height
                             << " matches no driver timing entry and has no measured pixelclock; keeping current timings"sv;
          return;
        }
        pick = live;  // complete measurement, just not in the table — try it raw
      }

      if (xioctl(fd, VIDIOC_S_DV_TIMINGS, &pick) != 0) {
        char string[1024];
        BOOST_LOG(warning) << "RKMPP direct V4L2: couldn't adopt live signal timings ("sv
                           << pick.bt.width << 'x' << pick.bt.height << "): "sv
                           << strerror_r(errno, string, sizeof(string));
        return;
      }
      BOOST_LOG(info) << "RKMPP direct V4L2: adopted live signal timings "sv
                      << pick.bt.width << 'x' << pick.bt.height
                      << (pick.bt.interlaced ? 'i' : 'p')
                      << " pixelclock "sv << pick.bt.pixelclock
                      << (have_cur ? " (driver held stale timings)"sv : ""sv);
    }

    bool init(const char *device, int width, int height, int fps) {
      dev_path = device;
      req_width = width;
      req_height = height;
      req_fps = fps;
      last_frame_at = std::chrono::steady_clock::now();

      fd = open(device, O_RDWR | O_NONBLOCK);
      if (fd < 0) {
        char string[1024];
        BOOST_LOG(warning) << "RKMPP direct V4L2: couldn't open "sv << device << ": "sv << strerror_r(errno, string, sizeof(string));
        return false;
      }

      // Ask the driver to tell us about in-band signal changes (an HDMI
      // switch flipping inputs, a console changing video mode); stall
      // detection in maybe_recover_source covers drivers that don't.
      v4l2_event_subscription sub {};
      sub.type = V4L2_EVENT_SOURCE_CHANGE;
      source_events = xioctl(fd, VIDIOC_SUBSCRIBE_EVENT, &sub) == 0;

      // Platform capture blocks like the Rockchip HDMI-RX are multiplanar-only
      // devices; USB capture cards are single-planar. Pick the buffer type once
      // so every ioctl below can share it.
      v4l2_capability cap {};
      if (xioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
        auto caps = (cap.capabilities & V4L2_CAP_DEVICE_CAPS) ? cap.device_caps : cap.capabilities;
        mplane = !(caps & V4L2_CAP_VIDEO_CAPTURE) && (caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE);
        if (mplane) {
          BOOST_LOG(info) << "RKMPP direct V4L2: multiplanar capture device"sv;
        }
      }

      // Must run before negotiate_format: its G_FMT fallback reads a format
      // derived from whatever timings the driver currently holds.
      sync_dv_timings();

      forced_aspect_ratio = parse_aspect_ratio(std::getenv("SUNSHINE_RKMPP_ASPECT_RATIO"));
      scale_filter = parse_scale_filter(std::getenv("SUNSHINE_RKMPP_SCALE_FILTER"));
      line_blend = parse_line_blend(std::getenv("SUNSHINE_RKMPP_LINE_BLEND"));
      view_region = parse_view_region(std::getenv("SUNSHINE_RKMPP_VIEW_REGION"));
      if (!negotiate_format(width, height)) {
        return false;
      }

      v4l2_streamparm parm {};
      parm.type = buf_type();
      parm.parm.capture.timeperframe.numerator = 1;
      parm.parm.capture.timeperframe.denominator = fps;
      xioctl(fd, VIDIOC_S_PARM, &parm);

      v4l2_requestbuffers req {};
      req.count = 4;
      req.type = buf_type();
      req.memory = V4L2_MEMORY_MMAP;
      if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0 || req.count < 2) {
        char string[1024];
        BOOST_LOG(warning) << "RKMPP direct V4L2: VIDIOC_REQBUFS failed: "sv << strerror_r(errno, string, sizeof(string));
        return false;
      }

      buffers.resize(req.count);
      for (std::uint32_t x = 0; x < req.count; ++x) {
        v4l2_plane planes[VIDEO_MAX_PLANES] {};
        v4l2_buffer buf {};
        buf.type = buf_type();
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = x;
        if (mplane) {
          buf.m.planes = planes;
          buf.length = VIDEO_MAX_PLANES;
        }
        if (xioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
          return false;
        }

        // negotiate_format only accepts contiguous single-memory-plane layouts,
        // so plane 0 carries the whole frame in both APIs.
        auto length = mplane ? planes[0].length : buf.length;
        auto offset = mplane ? planes[0].m.mem_offset : buf.m.offset;
        buffers[x].length = length;
        buffers[x].start = mmap(nullptr, length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset);
        if (buffers[x].start == MAP_FAILED) {
          buffers[x].start = nullptr;
          return false;
        }

        // Export the buffer as a dma-buf so RGA can read the frame directly
        // (see direct_frame_t::dmabuf_fd). Optional: drivers without EXPBUF
        // (or non-contiguous buffers) simply leave the CPU-copy path in use.
        v4l2_exportbuffer exp {};
        exp.type = buf_type();
        exp.index = x;
        exp.plane = 0;
        exp.flags = O_RDWR | O_CLOEXEC;
        if (xioctl(fd, VIDIOC_EXPBUF, &exp) == 0) {
          buffers[x].dmabuf_fd = exp.fd;
        }

        if (xioctl(fd, VIDIOC_QBUF, &buf) < 0) {
          return false;
        }
      }

      int type = buf_type();
      if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        return false;
      }

      streaming = true;

      // Session marker for retro-web: while this exists (and the PID inside is
      // alive), the capture device belongs to the live stream and screenshots
      // must go through the request-file handshake, never the device.
      if (auto *marker = std::fopen(streaming_marker_path().c_str(), "wb")) {
        std::fprintf(marker, "%d\n", (int) getpid());
        std::fclose(marker);
      }

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

    // Requeue a buffer back to the driver by index (used for the held zero-copy
    // buffer, which we keep dequeued across frames).
    void requeue_buffer(std::uint32_t index) {
      v4l2_plane planes[VIDEO_MAX_PLANES] {};
      v4l2_buffer buf {};
      buf.type = buf_type();
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = index;
      if (mplane) {
        buf.m.planes = planes;
        buf.length = VIDEO_MAX_PLANES;
      }
      xioctl(fd, VIDIOC_QBUF, &buf);
    }

    // A source change mid-session (HDMI switch input flip, console video-mode
    // switch) invalidates the negotiated capture: the driver either stops
    // delivering (frozen stream) or delivers a new geometry into buffers
    // negotiated for the old one (stride garbage; unwritten NV12 renders as
    // green bars). Detect it — the driver's SOURCE_CHANGE event when
    // supported, a delivery stall otherwise — and replay the full init:
    // DV-timings re-latch, format negotiation, buffer setup. Throttled so an
    // absent source retries calmly until it appears (the held last-good frame
    // is shown meanwhile — see update_latest_frame's suppression).
    void maybe_recover_source() {
      constexpr auto STALL_AFTER = std::chrono::milliseconds(1200);
      constexpr auto RECOVER_BACKOFF = std::chrono::milliseconds(2500);

      // A trigger latches recover_pending and stays latched until a replay
      // runs or a probe proves the capture healthy. Consuming the trigger on
      // a single held tick loses it: a console can change modes while frames
      // are still flowing through the stale config (a PS3 boots at 480p and
      // switches to 1080p at the menu — garbage frames keep the stall
      // detector quiet, and the SOURCE_CHANGE event only fires once).
      if (source_events && fd >= 0) {
        v4l2_event ev {};
        while (xioctl(fd, VIDIOC_DQEVENT, &ev) == 0) {
          recover_pending = recover_pending || ev.type == V4L2_EVENT_SOURCE_CHANGE;
        }
      }

      const auto now = std::chrono::steady_clock::now();
      const bool stalled = now - last_frame_at >= STALL_AFTER;
      const bool was_pending = recover_pending;
      recover_pending = recover_pending || stalled;
      if (!recover_pending) {
        return;
      }
      if (!was_pending) {
        pending_since = now;
      }
      // Probing (a couple of register-read ioctls) is much cheaper than a
      // replay, so a fresh episode probes on a much shorter throttle: the
      // replay then starts within ~250 ms of the new signal locking instead
      // of waiting out the replay backoff — that wait was a visible second of
      // limbo on every HDMI-switch flip. Real transitions settle within a few
      // seconds; past that the hold is an absent source, so decay to the slow
      // tick — the driver v4l2_err()s every no-link/no-lock query, and fast
      // probes would flood the kernel log for as long as the source is gone.
      constexpr auto PROBE_FAST = std::chrono::milliseconds(250);
      constexpr auto PROBE_FAST_WINDOW = std::chrono::seconds(10);
      const auto probe_backoff = now - pending_since <= PROBE_FAST_WINDOW ? PROBE_FAST : RECOVER_BACKOFF;
      if (now - last_probe_at < probe_backoff) {
        return;
      }
      last_probe_at = now;

      // A replay is pointless without a signal to renegotiate against, and it
      // is not free: every replay reallocates the buffer queue from CMA, and
      // on the vendor 6.1 kernel a sustained alloc/free loop can wedge
      // cma_alloc in an unkillable D-state (lru_cache_disable never returns),
      // taking /dev/video0 down with it until reboot. While the driver
      // reports no locked signal, hold the current state (the last good frame
      // stays up) and just re-probe on the backoff tick. Devices without DV
      // timings support (ENOTTY: USB capture cards) can't be probed — replay
      // as before; same for a lost fd, which only init can reopen.
      bool probe_locked = false;
      v4l2_bt_timings probe_bt {};
      if (fd >= 0) {
        v4l2_dv_timings live {};
        const int query_err = xioctl(fd, VIDIOC_QUERY_DV_TIMINGS, &live) == 0 ? 0 : errno;
        const bool locked = query_err == 0 &&
                            live.type == V4L2_DV_BT_656_1120 &&
                            live.bt.width > 0 && live.bt.height > 0;
        probe_locked = locked;
        if (locked) {
          probe_bt = live.bt;
        }
        if (!locked && query_err != ENOTTY) {
          if (!recovery_logged) {
            BOOST_LOG(info) << "RKMPP direct V4L2: "sv
                            << (stalled ? "capture stalled"sv : "source change reported"sv)
                            << " and no signal is locked; holding the last frame until a source appears"sv;
            recovery_logged = true;
          }
          // The RX PHY can wedge while the device sits open and powered
          // through a no-signal hold: cr register writes fail in a storm and
          // lock never returns even once the source is live (observed after
          // a boot against the switch's idle carrier followed by a console
          // power-on — minutes of failures, then instant lock on the first
          // open after a close). Closing the last fd runtime-suspends the
          // hdmirx block; the replay's fresh open powers it back up with a
          // clean PHY. So when the hold outlives any real transition, stop
          // holding the device open — the fd<0 replay path below reopens on
          // the normal backoff, giving a gentle open/probe/close cycle
          // (~20 s period) until a signal locks, instead of a permanently
          // powered, permanently wedged receiver.
          constexpr auto NO_LOCK_POWER_CYCLE = std::chrono::seconds(20);
          if (fd >= 0 && now - pending_since >= NO_LOCK_POWER_CYCLE) {
            BOOST_LOG(info) << "RKMPP direct V4L2: no lock after "sv
                            << std::chrono::duration_cast<std::chrono::seconds>(now - pending_since).count()
                            << " s; releasing the capture device to reset the RX PHY"sv;
            stop();
          }
          return;
        }
        // A benign trigger (brief stall that already resolved, event for a
        // change the driver absorbed): frames are flowing and the locked
        // signal matches what we're configured for — nothing to replay.
        if (locked && !stalled) {
          const auto clock_close = [](std::uint64_t a, std::uint64_t b) {
            const auto diff = a > b ? a - b : b - a;
            return diff * 100 <= b;
          };
          v4l2_dv_timings cur {};
          if (xioctl(fd, VIDIOC_G_DV_TIMINGS, &cur) == 0 &&
              cur.bt.width == live.bt.width && cur.bt.height == live.bt.height &&
              cur.bt.interlaced == live.bt.interlaced &&
              (live.bt.pixelclock == 0 || clock_close(live.bt.pixelclock, cur.bt.pixelclock))) {
            recover_pending = false;
            return;
          }
        }
      }

      if (now - last_recover_at < RECOVER_BACKOFF) {
        return;  // replays stay throttled even though probing is frequent
      }

      // A locked signal is not necessarily a usable one: an HDMI switch with
      // every input asleep emits an idle carrier (1440x480, no AVI InfoFrame)
      // that measures fine but is rejected by S_DV_TIMINGS, so a replay
      // renegotiates against the stale config and delivers nothing. Replaying
      // in a loop against such a signal is the CMA alloc/free churn the
      // no-signal hold exists to prevent. If the last replay produced no
      // frames and the live signal hasn't changed since, hold — probing stays
      // cheap, and a console powering on changes the timings, which unblocks
      // the replay immediately. A slow retry remains as a safety net so a
      // transiently failed init against a real signal can't hold forever.
      // "No frames since the replay" must be read from ever_produced (cleared
      // by stop(), set on the first presented frame) — last_frame_at is reset
      // by init() as a stall-detection grace period and always looks fresh.
      // A real signal can also fail a replay transiently (observed: a console
      // powering on locks the PHY a beat before its InfoFrames flow, and the
      // whole init lands in that window), so the first few fruitless replays
      // retry on the normal backoff; only a signal that keeps producing
      // nothing settles into the slow hold — that persistence is what marks
      // the idle-carrier case whose replay churn is the CMA hazard.
      constexpr auto SAME_SIGNAL_RETRY = std::chrono::seconds(30);
      constexpr int QUICK_REPLAYS = 3;  // total attempts before the slow hold
      const auto clock_close = [](std::uint64_t a, std::uint64_t b) {
        const auto diff = a > b ? a - b : b - a;
        return diff * 100 <= b;
      };
      const bool same_fruitless_signal =
        probe_locked && last_replay_valid && !ever_produced &&
        probe_bt.width == last_replay_bt.width && probe_bt.height == last_replay_bt.height &&
        probe_bt.interlaced == last_replay_bt.interlaced &&
        (probe_bt.pixelclock == 0 || last_replay_bt.pixelclock == 0 ||
         clock_close(probe_bt.pixelclock, last_replay_bt.pixelclock));
      if (same_fruitless_signal && fruitless_replays >= QUICK_REPLAYS &&
          now - last_recover_at < SAME_SIGNAL_RETRY) {
        if (!recovery_logged) {
          BOOST_LOG(info) << "RKMPP direct V4L2: signal "sv << probe_bt.width << 'x' << probe_bt.height
                          << " unchanged after "sv << fruitless_replays
                          << " fruitless re-inits; holding until it changes"sv;
          recovery_logged = true;
        }
        return;
      }
      last_recover_at = now;
      last_replay_bt = probe_bt;
      last_replay_valid = probe_locked;
      fruitless_replays = same_fruitless_signal ? fruitless_replays + 1 : 1;

      recover_pending = false;
      BOOST_LOG(info) << "RKMPP direct V4L2: "sv
                      << (stalled ? "capture stalled"sv : "source change reported"sv)
                      << "; re-initializing capture"sv;
      recovery_logged = true;
      const auto device = dev_path;
      stop();
      if (!init(device.c_str(), req_width, req_height, req_fps)) {
        stop();  // failed part-way: release the fd so the next attempt starts clean
      }
    }

    bool update_latest_frame() {
      maybe_recover_source();

      v4l2_buffer latest {};
      std::uint32_t latest_used = 0;
      bool have_latest = false;

      for (;;) {
        v4l2_plane planes[VIDEO_MAX_PLANES] {};
        v4l2_buffer buf {};
        buf.type = buf_type();
        buf.memory = V4L2_MEMORY_MMAP;
        if (mplane) {
          buf.m.planes = planes;
          buf.length = VIDEO_MAX_PLANES;
        }

        if (xioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
          if (errno == EAGAIN && !have_latest) {
            pollfd pfd {fd, POLLIN, 0};
            auto timeout_ms = ever_produced ? 8 : 250;
            if (poll(&pfd, 1, timeout_ms) > 0) {
              continue;
            }
          }
          break;
        }

        if (have_latest) {
          requeue_buffer(latest.index);
        }
        latest = buf;
        latest.m.planes = nullptr;  // stack plane array; only .index/.bytesused are read below
        latest_used = mplane ? planes[0].bytesused : buf.bytesused;
        have_latest = true;
      }

      // While a recovery is owed the config is stale, so whatever the driver
      // delivers is the new signal squeezed through the old geometry — stride
      // garbage and green half-frames. Requeue instead of presenting and keep
      // showing the held last-good frame (the input-switch freeze a TV gives);
      // the replay lands within a probe tick of the new signal locking. The
      // deliveries deliberately don't touch last_frame_at: they are not
      // evidence of health, and a benign trigger is cleared by the probe
      // before this suppression is ever visible.
      if (have_latest && recover_pending) {
        requeue_buffer(latest.index);
        have_latest = false;
      }

      if (have_latest) {
        last_frame_at = std::chrono::steady_clock::now();
        recovery_logged = false;
        auto *src = (const std::uint8_t *) buffers[latest.index].start;
        if (is_mjpeg) {
          // Decoded into the CPU latest_frame; the capture buffer is free now.
          decode_mjpeg_to_nv12(src, latest_used);
          requeue_buffer(latest.index);
          if (!latest_frame.empty()) {
            latest_ptr = latest_frame.data();
            ever_produced = true;
          }
        } else if (is_raw_convert || !is_zeroed_nv12(src)) {
          // Zero-copy: hand the GPU upload the mmap'd buffer directly instead of
          // memcpying it. Keep this buffer dequeued and release the previously
          // held one; the upload (glTexSubImage2D) consumes the pointer
          // synchronously, so requeuing one frame later is safe.
          if (held_index >= 0) {
            requeue_buffer((std::uint32_t) held_index);
          }
          held_index = (int) latest.index;
          latest_ptr = src;
          ever_produced = true;
        } else {
          // Zeroed NV12 frame: discard it and keep showing the last good frame.
          requeue_buffer(latest.index);
        }
      }

      if (!ever_produced) {
        return false;
      }

      maybe_serve_screenshot();
      missing_frame_logged = false;
      return true;
    }

    static std::string runtime_dir() {
      const char *dir = std::getenv("XDG_RUNTIME_DIR");
      if (!dir || !*dir) {
        dir = "/run/retro-stream";
      }
      return dir;
    }

    static std::string streaming_marker_path() {
      return runtime_dir() + "/sunshine-streaming";
    }

    // Serve on-demand raw-frame snapshots for the retro-web view-region UI.
    // A second V4L2 consumer would renegotiate the HDMI-RX timings and kill the
    // live stream, so the web UI instead drops a request file and this capture
    // loop writes out the latest frame it already holds. No VPU involvement:
    // the web server JPEG-encodes the raw frame itself. The request check is a
    // single access() per frame; the 1 MB file write happens on a detached
    // thread so it can never stall frame pacing.
    void maybe_serve_screenshot() {
      if (!latest_ptr) {
        return;
      }
      const auto dir = runtime_dir();
      const auto request = dir + "/sunshine-screenshot.request";
      if (access(request.c_str(), F_OK) != 0) {
        return;
      }
      unlink(request.c_str());

      auto out_fourcc = capture_fourcc;
      auto out_stride = capture_stride;
      if (is_mjpeg) {
        // latest_ptr points into the decoded packed-NV12 latest_frame.
        out_fourcc = V4L2_PIX_FMT_NV12;
        out_stride = width;
      }
      const auto bytes = snapshot_frame_bytes(out_fourcc, out_stride, height);
      if (bytes == 0) {
        return;
      }

      // Copy the frame now (latest_ptr is only stable on this thread) and let a
      // detached thread do the tmpfs writes.
      std::vector<std::uint8_t> frame_copy(latest_ptr, latest_ptr + bytes);
      std::thread([dir, frame_copy = std::move(frame_copy), out_fourcc, out_stride, w = width, h = height]() {
        // raw first, then meta — the reader treats the meta file's appearance
        // as "raw is complete", so both are written to temp names and renamed.
        const auto raw_path = dir + "/sunshine-screenshot.raw";
        const auto meta_path = dir + "/sunshine-screenshot.meta";
        const auto raw_tmp = raw_path + ".tmp";
        const auto meta_tmp = meta_path + ".tmp";

        auto *raw_file = std::fopen(raw_tmp.c_str(), "wb");
        if (!raw_file) {
          return;
        }
        const auto written = std::fwrite(frame_copy.data(), 1, frame_copy.size(), raw_file);
        std::fclose(raw_file);
        if (written != frame_copy.size() || std::rename(raw_tmp.c_str(), raw_path.c_str()) != 0) {
          unlink(raw_tmp.c_str());
          return;
        }

        auto *meta_file = std::fopen(meta_tmp.c_str(), "wb");
        if (!meta_file) {
          return;
        }
        std::fprintf(meta_file, "%s %d %d %d\n", fourcc_str(out_fourcc).c_str(), w, h, out_stride);
        std::fclose(meta_file);
        if (std::rename(meta_tmp.c_str(), meta_path.c_str()) != 0) {
          unlink(meta_tmp.c_str());
          return;
        }
        BOOST_LOG(info) << "RKMPP direct V4L2: served view-region screenshot frame ("sv
                        << fourcc_str(out_fourcc) << ' ' << w << 'x' << h << ')';
      }).detach();
    }

    // Total bytes of the capture buffer for a snapshot, per format. The stride
    // is the luma line stride; chroma layouts follow V4L2's contiguous
    // single-memory-plane conventions.
    static std::size_t snapshot_frame_bytes(std::uint32_t fourcc, int stride, int height) {
      const auto s = (std::size_t) stride;
      const auto h = (std::size_t) height;
      switch (fourcc) {
        case V4L2_PIX_FMT_NV12:
          return s * h * 3 / 2;
        case V4L2_PIX_FMT_NV16:
          return s * h * 2;
        case V4L2_PIX_FMT_NV24:
          return s * h * 3;
        case V4L2_PIX_FMT_YUYV:  // stride covers 2 bytes/px
        case V4L2_PIX_FMT_BGR24: // stride covers 3 bytes/px
        case V4L2_PIX_FMT_RGB24:
          return s * h;
        default:
          return 0;
      }
    }

    direct_frame_t latest_direct_frame() const {
      direct_frame_t frame;
      frame.data = latest_ptr;
      frame.fourcc = capture_fourcc;
      frame.width = width;
      frame.height = height;
      frame.stride = capture_stride;
      frame.nv12 = !is_raw_convert || is_mjpeg;
      if (is_mjpeg) {
        frame.fourcc = V4L2_PIX_FMT_NV12;
        frame.stride = width;
      } else if (held_index >= 0 && latest_ptr == buffers[held_index].start) {
        // The zero-copy frame is the held V4L2 buffer itself; hand its dma-buf
        // to consumers that can read it without a CPU copy (the RGA path).
        frame.dmabuf_fd = buffers[held_index].dmabuf_fd;
        frame.dmabuf_size = (std::uint32_t) buffers[held_index].length;
      }
      return frame;
    }

    scale_region_t scale_region(int dst_width, int dst_height) const {
      scale_region_t region;
      region.src_w = width;
      region.src_h = height;

      if (view_region.enabled) {
        region.src_left = std::clamp((int) (view_region.left * width), 0, width - 2) & ~1;
        region.src_top = std::clamp((int) (view_region.top * height), 0, height - 2) & ~1;
        auto src_right = std::clamp((int) (view_region.right * width), region.src_left + 2, width) & ~1;
        auto src_bottom = std::clamp((int) (view_region.bottom * height), region.src_top + 2, height) & ~1;
        region.src_w = std::max(2, src_right - region.src_left);
        region.src_h = std::max(2, src_bottom - region.src_top);
      }

      if (dst_width == region.src_w && dst_height == region.src_h) {
        region.out_w = region.src_w;
        region.out_h = region.src_h;
        return region;
      }

      auto output_aspect = forced_aspect_ratio > 0.0f ? forced_aspect_ratio : (float) region.src_w / region.src_h;
      region.out_w = dst_width;
      region.out_h = (int) (dst_width / output_aspect);
      if (region.out_h > dst_height) {
        region.out_h = dst_height;
        region.out_w = (int) (dst_height * output_aspect);
      }
      region.out_w = std::max(2, region.out_w & ~1);
      region.out_h = std::max(2, region.out_h & ~1);
      region.off_x = ((dst_width - region.out_w) / 2) & ~1;
      region.off_y = ((dst_height - region.out_h) / 2) & ~1;
      return region;
    }

    bool copy_latest_to(AVFrame *mapped_frame, int dst_width, int dst_height) {
      if (!update_latest_frame()) {
        return false;
      }
      copy_nv12(latest_ptr, mapped_frame, dst_width, dst_height);
      return true;
    }

    // True when the configured scaling settings are a no-op for this stream size,
    // so the captured frame can be handed to the encoder unscaled (direct stream):
    //  - the capture is already NV12 (native NV12 or MJPEG-decoded to NV12),
    //  - no view-region crop is set,
    //  - the stream size equals the capture size (no resize), and
    //  - no forced aspect ratio, or one that matches the capture (no letterbox).
    bool passthrough_ok(int dst_width, int dst_height) const {
      const bool nv12 = !is_raw_convert || is_mjpeg;
      if (!nv12 || view_region.enabled) {
        return false;
      }
      if (width != dst_width || height != dst_height) {
        return false;
      }
      if (forced_aspect_ratio > 0.0f) {
        const float src_aspect = (float) width / height;
        if (std::fabs(forced_aspect_ratio - src_aspect) > 0.01f) {
          return false;
        }
      }
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

    // The encode device reads the parsed filter to pick a scaling backend
    // (nearest -> GPU; the rest -> RGA) and the RGA interpolation mode.
    scale_filter_e get_scale_filter() const {
      return scale_filter;
    }

    int width {};
    int height {};

  private:
    struct buffer_t {
      void *start {};
      std::size_t length {};
      int dmabuf_fd {-1};
    };

    // Single-planar for USB capture cards, multiplanar for platform blocks
    // like the Rockchip HDMI-RX (see the VIDIOC_QUERYCAP probe in init).
    v4l2_buf_type buf_type() const {
      return mplane ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE : V4L2_BUF_TYPE_VIDEO_CAPTURE;
    }

    bool mplane {};

    void stop() {
      unlink(streaming_marker_path().c_str());
      if (mjpeg_ctx) {
        avcodec_free_context(&mjpeg_ctx);
        mjpeg_ctx = nullptr;
      }
      if (sws_ctx) {
        sws_freeContext(sws_ctx);
        sws_ctx = nullptr;
      }
      if (fd >= 0 && streaming) {
        int type = buf_type();
        xioctl(fd, VIDIOC_STREAMOFF, &type);
      }
      streaming = false;

      for (auto &buffer : buffers) {
        if (buffer.start) {
          munmap(buffer.start, buffer.length);
        }
        if (buffer.dmabuf_fd >= 0) {
          close(buffer.dmabuf_fd);
        }
      }
      buffers.clear();

      if (fd >= 0) {
        v4l2_requestbuffers req {};
        req.count = 0;
        req.type = buf_type();
        req.memory = V4L2_MEMORY_MMAP;
        xioctl(fd, VIDIOC_REQBUFS, &req);
      }

      if (fd >= 0) {
        close(fd);
      }
      fd = -1;

      // The zero-copy pointers referenced the buffers just unmapped.
      latest_ptr = nullptr;
      held_index = -1;
      ever_produced = false;
    }

    void copy_nv12(const std::uint8_t *src, AVFrame *dst, int dst_width, int dst_height) {
      int src_left = 0;
      int src_top = 0;
      int src_w = width;
      int src_h = height;

      if (view_region.enabled) {
        src_left = std::clamp((int) (view_region.left * width), 0, width - 2) & ~1;
        src_top = std::clamp((int) (view_region.top * height), 0, height - 2) & ~1;
        auto src_right = std::clamp((int) (view_region.right * width), src_left + 2, width) & ~1;
        auto src_bottom = std::clamp((int) (view_region.bottom * height), src_top + 2, height) & ~1;
        src_w = std::max(2, src_right - src_left);
        src_h = std::max(2, src_bottom - src_top);
      }

      const auto *src_y = src + (std::size_t) src_top * width + src_left;
      const auto *src_uv = src + (std::size_t) width * height + (std::size_t) (src_top / 2) * width + src_left;

      if (dst_width == src_w && dst_height == src_h) {
        for (int y = 0; y < src_h; ++y) {
          std::memcpy(dst->data[0] + (std::size_t) y * dst->linesize[0], src_y + (std::size_t) y * width, src_w);
        }

        for (int y = 0; y < src_h / 2; ++y) {
          std::memcpy(dst->data[1] + (std::size_t) y * dst->linesize[1], src_uv + (std::size_t) y * width, src_w);
        }

        if (should_line_blend(src_h, dst_height, src_h)) {
          blend_nv12_lines(dst, 0, 0, src_w, src_h);
        }

        return;
      }

      std::fill_n(dst->data[0], (std::size_t) dst->linesize[0] * dst_height, 16);
      std::fill_n(dst->data[1], (std::size_t) dst->linesize[1] * (dst_height / 2), 128);

      auto output_aspect = forced_aspect_ratio > 0.0f ? forced_aspect_ratio : (float) src_w / src_h;
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
                                (scale_filter == scale_filter_e::automatic && (out_w != src_w || out_h != src_h));
      if (linear_scale) {
        scale_nv12_linear(src_y, src_uv, width, src_w, src_h, dst, off_x, off_y, out_w, out_h);
      } else {
        scale_nv12_nearest(src_y, src_uv, width, src_w, src_h, dst, off_x, off_y, out_w, out_h);
      }

      if (should_line_blend(src_h, out_h, dst_height)) {
        blend_nv12_lines(dst, off_x, off_y, out_w, out_h);
      }
    }

    void scale_nv12_nearest(const std::uint8_t *src_y, const std::uint8_t *src_uv, int src_stride, int src_w, int src_h, AVFrame *dst, int off_x, int off_y, int out_w, int out_h) {
      for (int y = 0; y < out_h; ++y) {
        auto sy = std::min(src_h - 1, (int) ((std::int64_t) y * src_h / out_h));
        auto *dst_row = dst->data[0] + (std::size_t) (off_y + y) * dst->linesize[0] + off_x;
        const auto *src_row = src_y + (std::size_t) sy * src_stride;
        for (int x = 0; x < out_w; ++x) {
          auto sx = std::min(src_w - 1, (int) ((std::int64_t) x * src_w / out_w));
          dst_row[x] = src_row[sx];
        }
      }

      for (int y = 0; y < out_h / 2; ++y) {
        auto sy = std::min(src_h / 2 - 1, (int) ((std::int64_t) y * (src_h / 2) / (out_h / 2)));
        auto *dst_row = dst->data[1] + (std::size_t) (off_y / 2 + y) * dst->linesize[1] + off_x;
        const auto *src_row = src_uv + (std::size_t) sy * src_stride;
        for (int x = 0; x < out_w; x += 2) {
          auto sx = std::min(src_w - 2, ((int) ((std::int64_t) x * src_w / out_w)) & ~1);
          dst_row[x] = src_row[sx];
          dst_row[x + 1] = src_row[sx + 1];
        }
      }
    }

    void scale_plane_linear(const std::uint8_t *src, int src_stride, int src_w, int src_h, std::uint8_t *dst, int dst_stride, int dst_w, int dst_h, int channels) {
      if (src_h == dst_h) {
        scale_plane_linear_horizontal(src, src_stride, src_w, src_h, dst, dst_stride, dst_w, channels);
        return;
      }

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

    void scale_plane_linear_horizontal(const std::uint8_t *src, int src_stride, int src_w, int src_h, std::uint8_t *dst, int dst_stride, int dst_w, int channels) {
      if (src_w == dst_w) {
        for (int y = 0; y < src_h; ++y) {
          std::memcpy(dst + (std::size_t) y * dst_stride, src + (std::size_t) y * src_stride, (std::size_t) src_w * channels);
        }
        return;
      }

      std::vector<std::pair<int, int>> xmap(dst_w);
      for (int x = 0; x < dst_w; ++x) {
        auto pos = (((std::int64_t) (x * 2 + 1) * src_w * 256) / (dst_w * 2)) - 128;
        if (pos < 0) {
          pos = 0;
        }
        auto x0 = std::min(src_w - 1, (int) (pos >> 8));
        auto x1 = std::min(src_w - 1, x0 + 1);
        auto frac = x1 == x0 ? 0 : (int) (pos & 0xff);
        xmap[x] = {x0, frac};
      }

      for (int y = 0; y < src_h; ++y) {
        auto *dst_row = dst + (std::size_t) y * dst_stride;
        const auto *src_row = src + (std::size_t) y * src_stride;
        for (int x = 0; x < dst_w; ++x) {
          auto [x0, frac] = xmap[x];
          const auto *p0 = src_row + (std::size_t) x0 * channels;
          const auto *p1 = src_row + (std::size_t) std::min(src_w - 1, x0 + 1) * channels;
          auto *out = dst_row + (std::size_t) x * channels;
          for (int c = 0; c < channels; ++c) {
            out[c] = (std::uint8_t) (((int) p0[c] * (256 - frac) + (int) p1[c] * frac + 128) >> 8);
          }
        }
      }
    }

    void scale_nv12_linear(const std::uint8_t *src_y, const std::uint8_t *src_uv, int src_stride, int src_w, int src_h, AVFrame *dst, int off_x, int off_y, int out_w, int out_h) {
      auto *dst_y = dst->data[0] + (std::size_t) off_y * dst->linesize[0] + off_x;
      auto *dst_uv = dst->data[1] + (std::size_t) (off_y / 2) * dst->linesize[1] + off_x;

      scale_plane_linear(src_y, src_stride, src_w, src_h, dst_y, dst->linesize[0], out_w, out_h, 1);
      scale_plane_linear(src_uv, src_stride, src_w / 2, src_h / 2, dst_uv, dst->linesize[1], out_w / 2, out_h / 2, 2);
    }

    bool should_line_blend(int src_h, int out_h, int dst_h) const {
      if (line_blend == line_blend_e::disabled) {
        return false;
      }
      if (line_blend == line_blend_e::enabled) {
        return out_h >= 4;
      }

      // 240p/480i-era sources often arrive through HDMI capture hardware as
      // alternating-line SD frames. When 720x480 is widened to 16:9 the height is
      // unchanged, so normal scaling preserves those horizontal lines exactly.
      return src_h <= 576 && out_h == src_h && out_h <= dst_h && out_h >= 4;
    }

    void blend_plane_lines(std::uint8_t *plane, int stride, int off_x, int off_y, int width, int height, int channels) {
      if (height < 3 || width <= 0) {
        return;
      }

      const auto row_bytes = (std::size_t) width * channels;
      std::vector<std::uint8_t> prev(row_bytes);
      std::vector<std::uint8_t> cur(row_bytes);
      std::vector<std::uint8_t> next(row_bytes);

      auto *row0 = plane + (std::size_t) off_y * stride + off_x * channels;
      auto *row1 = plane + (std::size_t) (off_y + 1) * stride + off_x * channels;
      std::memcpy(prev.data(), row0, row_bytes);
      std::memcpy(cur.data(), row1, row_bytes);

      for (int y = 1; y < height - 1; ++y) {
        auto *next_row = plane + (std::size_t) (off_y + y + 1) * stride + off_x * channels;
        std::memcpy(next.data(), next_row, row_bytes);
        auto *dst_row = plane + (std::size_t) (off_y + y) * stride + off_x * channels;
        for (std::size_t x = 0; x < row_bytes; ++x) {
          dst_row[x] = (std::uint8_t) (((int) prev[x] + cur[x] + next[x] + 1) / 3);
        }
        prev.swap(cur);
        cur.swap(next);
      }
    }

    void blend_nv12_lines(AVFrame *dst, int off_x, int off_y, int out_w, int out_h) {
      blend_plane_lines(dst->data[0], dst->linesize[0], off_x, off_y, out_w, out_h, 1);
      blend_plane_lines(dst->data[1], dst->linesize[1], off_x / 2, off_y / 2, out_w / 2, out_h / 2, 2);
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
        {V4L2_PIX_FMT_YUYV, AV_PIX_FMT_YUYV422}, // raw, GPU converts to NV12
        {V4L2_PIX_FMT_BGR24, AV_PIX_FMT_BGR24},  // raw, RGA converts to NV12 (HDMI-RX RGB sources)
        {V4L2_PIX_FMT_MJPEG, AV_PIX_FMT_NONE}, // JPEG decode
      };

      // SUNSHINE_RKMPP_V4L2_FORMAT (fed from [capture].format in retro-stream.toml)
      // keeps the default preference-ordered negotiation unless a format is
      // explicitly pinned.
      auto forced = parse_forced_format(std::getenv("SUNSHINE_RKMPP_V4L2_FORMAT"));
      std::array<std::size_t, sizeof(candidates) / sizeof(candidates[0])> order {
        0, 1, 2, 3
      };
      const bool mjpeg_prefers_fallback = forced.is_forced && forced.v4l2 == V4L2_PIX_FMT_MJPEG;

      if (mjpeg_prefers_fallback) {
        BOOST_LOG(info) << "RKMPP direct V4L2: format preference "sv << fourcc_str(forced.v4l2)
                        << (forced.mjpeg_hw ? " with hardware MJPEG decode"sv : ""sv);
        order = std::array<std::size_t, sizeof(candidates) / sizeof(candidates[0])> {3, 0, 1, 2};
      } else if (forced.is_forced) {
        BOOST_LOG(info) << "RKMPP direct V4L2: format pinned to "sv << fourcc_str(forced.v4l2)
                        << (forced.mjpeg_hw ? " with hardware MJPEG decode"sv : ""sv);
      }

      auto sizes = capture_size_fallbacks(requested_width, requested_height);

      // Scaler-less devices (e.g. the Rockchip on-board HDMI-RX) only accept
      // the incoming signal's timing, whatever the requested stream size is.
      // Append the driver's current format size as a last resort; the
      // capture->stream scaler handles the difference.
      {
        v4l2_format cur {};
        cur.type = buf_type();
        if (xioctl(fd, VIDIOC_G_FMT, &cur) == 0) {
          int native_w = mplane ? (int) cur.fmt.pix_mp.width : (int) cur.fmt.pix.width;
          int native_h = mplane ? (int) cur.fmt.pix_mp.height : (int) cur.fmt.pix.height;
          if (native_w > 0 && native_h > 0 &&
              std::find(sizes.begin(), sizes.end(), std::make_pair(native_w, native_h)) == sizes.end()) {
            sizes.emplace_back(native_w, native_h);
          }
        }
      }

      // 1 = success, 0 = no candidate matched, -1 = fatal activate failure.
      auto try_candidates = [&](bool ignore_pin) -> int {
        for (auto size : sizes) {
          for (auto idx : order) {
            auto candidate = candidates[idx];
            if (!ignore_pin && forced.is_forced && !mjpeg_prefers_fallback && candidate.v4l2 != forced.v4l2) {
              continue;
            }
            v4l2_format fmt {};
            fmt.type = buf_type();
            if (mplane) {
              fmt.fmt.pix_mp.width = size.first;
              fmt.fmt.pix_mp.height = size.second;
              fmt.fmt.pix_mp.pixelformat = candidate.v4l2;
              fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
              fmt.fmt.pix_mp.num_planes = 1;
            } else {
              fmt.fmt.pix.width = size.first;
              fmt.fmt.pix.height = size.second;
              fmt.fmt.pix.pixelformat = candidate.v4l2;
              fmt.fmt.pix.field = V4L2_FIELD_NONE;
            }
            if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
              continue;
            }
            auto got_format = mplane ? fmt.fmt.pix_mp.pixelformat : fmt.fmt.pix.pixelformat;
            auto got_width = (int) (mplane ? fmt.fmt.pix_mp.width : fmt.fmt.pix.width);
            auto got_height = (int) (mplane ? fmt.fmt.pix_mp.height : fmt.fmt.pix.height);
            if (got_format != candidate.v4l2 || got_width != size.first || got_height != size.second) {
              continue;
            }
            // The mmap/upload paths assume one contiguous memory plane per frame
            // (NV12/YUYV/BGR24/MJPEG all have contiguous fourccs); reject others.
            if (mplane && fmt.fmt.pix_mp.num_planes != 1) {
              continue;
            }
            auto stride = (int) (mplane ? fmt.fmt.pix_mp.plane_fmt[0].bytesperline : fmt.fmt.pix.bytesperline);

            if (!activate_format(candidate.v4l2, candidate.av, size.first, size.second, stride, forced.mjpeg_hw)) {
              return -1;
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
            return 1;
          }
        }
        return 0;
      };

      auto result = try_candidates(false);
      if (result == 0 && forced.is_forced && !mjpeg_prefers_fallback) {
        // A pin the device cannot honor (e.g. the Rockchip HDMI-RX only accepts
        // the pixel format matching the incoming signal's color format) should
        // degrade to automatic selection, not lose the direct path entirely.
        BOOST_LOG(warning) << "RKMPP direct V4L2: pinned format "sv << fourcc_str(forced.v4l2)
                           << " not accepted by device; retrying with automatic format selection"sv;
        result = try_candidates(true);
      }
      if (result > 0) {
        return true;
      }
      if (result == 0) {
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

    bool activate_format(std::uint32_t v4l2, AVPixelFormat av, int width, int height, int bytes_per_line, bool mjpeg_hw) {
      capture_fourcc = v4l2;
      this->width = width;
      this->height = height;
      capture_stride = std::max(width, bytes_per_line);
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
        return scale_filter_e::nearest;
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
      if (v == "cubic" || v == "bicubic") {
        return scale_filter_e::cubic;
      }
      if (v == "average" || v == "avg") {
        return scale_filter_e::average;
      }

      BOOST_LOG(warning) << "RKMPP direct V4L2: invalid SUNSHINE_RKMPP_SCALE_FILTER '"sv << env << "'; using nearest"sv;
      return scale_filter_e::nearest;
    }

    static line_blend_e parse_line_blend(const char *env) {
      if (!env || !*env) {
        return line_blend_e::disabled;
      }
      std::string v(env);
      std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return (char) std::tolower(c); });
      v.erase(std::remove_if(v.begin(), v.end(), [](unsigned char c) { return std::isspace(c) || c == '-' || c == '_'; }), v.end());

      if (v.empty() || v == "auto" || v == "automatic") {
        return line_blend_e::disabled;
      }
      if (v == "on" || v == "true" || v == "1" || v == "enabled" || v == "blend") {
        return line_blend_e::enabled;
      }
      if (v == "off" || v == "false" || v == "0" || v == "disabled" || v == "none") {
        return line_blend_e::disabled;
      }

      BOOST_LOG(warning) << "RKMPP direct V4L2: invalid SUNSHINE_RKMPP_LINE_BLEND '"sv << env << "'; using auto"sv;
      return line_blend_e::disabled;
    }

    static view_region_t parse_view_region(const char *env) {
      view_region_t region;
      if (!env || !*env) {
        return region;
      }

      std::string v(env);
      std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return (char) std::tolower(c); });
      if (v == "auto" || v == "none" || v == "clear") {
        return region;
      }

      for (auto &c : v) {
        if (c == ',' || c == ';' || c == ':' || std::isspace((unsigned char) c)) {
          c = ' ';
        }
      }

      std::vector<float> parts;
      std::size_t pos = 0;
      while (pos < v.size()) {
        auto next = v.find(' ', pos);
        auto token = v.substr(pos, next == std::string::npos ? std::string::npos : next - pos);
        if (!token.empty()) {
          try {
            parts.push_back(std::stof(token));
          } catch (...) {
            parts.clear();
            break;
          }
        }
        if (next == std::string::npos) {
          break;
        }
        pos = next + 1;
      }

      if (parts.size() == 4) {
        auto left = std::clamp(parts[0], 0.0f, 1.0f);
        auto top = std::clamp(parts[1], 0.0f, 1.0f);
        auto right = std::clamp(parts[2], 0.0f, 1.0f);
        auto bottom = std::clamp(parts[3], 0.0f, 1.0f);
        if (right - left >= 0.01f && bottom - top >= 0.01f) {
          region.left = left;
          region.top = top;
          region.right = right;
          region.bottom = bottom;
          region.enabled = !(left == 0.0f && top == 0.0f && right == 1.0f && bottom == 1.0f);
          return region;
        }
      }

      BOOST_LOG(warning) << "RKMPP direct V4L2: invalid SUNSHINE_RKMPP_VIEW_REGION '"sv << env << "'; using full frame"sv;
      return {};
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
      if (capture_stride > src_stride[0]) {
        src_stride[0] = capture_stride;
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
    scale_filter_e scale_filter {scale_filter_e::nearest};
    line_blend_e line_blend {line_blend_e::disabled};
    view_region_t view_region;
    std::uint32_t capture_fourcc {V4L2_PIX_FMT_NV12};
    int capture_stride {};
    AVPixelFormat raw_av_fmt {AV_PIX_FMT_NONE};
    AVCodecContext *mjpeg_ctx {};
    SwsContext *sws_ctx {};
    std::vector<std::uint8_t> latest_frame;
    std::vector<std::uint8_t> convert_buf;
    // Zero-copy capture state: latest_ptr points at the current frame's pixels,
    // which for NV12/raw formats is the mmap'd V4L2 buffer kept dequeued in
    // held_index (requeued only once a newer good frame replaces it). For MJPEG
    // it points into the decoded latest_frame. ever_produced gates the first
    // valid frame.
    const std::uint8_t *latest_ptr {};
    int held_index {-1};
    bool ever_produced {};
    std::vector<buffer_t> buffers;
    // Mid-session source recovery (see maybe_recover_source): the init
    // parameters to replay, whether the driver posts SOURCE_CHANGE events,
    // and the delivery/attempt timestamps that drive stall detection and
    // reinit throttling.
    std::string dev_path;
    int req_width {};
    int req_height {};
    int req_fps {};
    bool source_events {};
    std::chrono::steady_clock::time_point last_frame_at {};
    std::chrono::steady_clock::time_point pending_since {};
    std::chrono::steady_clock::time_point last_probe_at {};
    std::chrono::steady_clock::time_point last_recover_at {};
    v4l2_bt_timings last_replay_bt {};
    bool last_replay_valid {};
    int fruitless_replays {};
    bool recover_pending {};
    bool recovery_logged {};
  };

#ifdef SUNSHINE_BUILD_RGA
  // Scales/letterboxes a captured NV12 frame into the encoder's NV12 buffer
  // using the Rockchip RGA 2D engine instead of the Mali GPU, freeing the GPU.
  //
  // RGA2 can only address low (<4 GB) memory and can't reach the UVC capture
  // buffer directly, so the capture is copied into a CMA dma-buf, RGA scales +
  // crops + letterboxes into a second CMA dma-buf, and the result is copied into
  // the encoder frame. The CMA heap must be large enough (grow with cma=256M);
  // init() probes that and the caller falls back to the GPU path if it isn't.
  class rga_scaler_t {
  public:
    ~rga_scaler_t() {
      free_enc_frame();
      release(src_buf);
      release(dst_buf);
      release(enc_buf);
      if (heap_fd >= 0) {
        close(heap_fd);
      }
    }

    bool init(int dst_w, int dst_h, scale_filter_e filter) {
      const char *heap_env = std::getenv("SUNSHINE_RKMPP_RGA_HEAP");
      heap_name = (heap_env && *heap_env) ? heap_env : "/dev/dma_heap/reserved";
      // The shared scaling-method setting drives the interpolation; an explicit
      // SUNSHINE_RKMPP_RGA_INTERP still overrides it for experimentation.
      interp = interp_from_filter(filter);
      if (auto override_env = std::getenv("SUNSHINE_RKMPP_RGA_INTERP")) {
        interp = parse_interp(override_env);
      }

      heap_fd = open(heap_name.c_str(), O_RDWR | O_CLOEXEC);
      if (heap_fd < 0) {
        BOOST_LOG(warning) << "RGA: cannot open dma-heap "sv << heap_name << "; RGA scaler unavailable"sv;
        return false;
      }

      // Confirm the heap can supply, and RGA can map, an output-frame-sized
      // low-memory buffer. This catches the too-small-CMA case at startup.
      buffer_t probe {};
      if (!alloc(probe, (std::size_t) dst_w * dst_h * 3 / 2)) {
        BOOST_LOG(warning) << "RGA: dma-heap "sv << heap_name << " cannot supply a "sv
                           << dst_w << 'x' << dst_h << " NV12 buffer (grow CMA, e.g. cma=256M); RGA scaler unavailable"sv;
        return false;
      }
      auto handle = importbuffer_fd(probe.fd, (int) probe.len);
      bool importable = handle > 0;
      if (importable) {
        releasebuffer_handle(handle);
      }
      release(probe);
      if (!importable) {
        BOOST_LOG(warning) << "RGA: importbuffer_fd failed (buffer not RGA-addressable); RGA scaler unavailable"sv;
        return false;
      }
      return true;
    }

    bool scale(const direct_frame_t &src, const scale_region_t &region, AVFrame *dst, int dst_w, int dst_h) {
      // NV12 and BGR24 capture are handled on the RGA path (RGA converts BGR
      // to NV12 during the scale); other formats fall back.
      if (!rga_src_supported(src)) {
        return false;
      }
      if (!ensure(src_buf, src.width, src.height, src_frame_bytes(src)) ||
          !ensure(dst_buf, dst_w, dst_h, (std::size_t) dst_w * dst_h * 3 / 2)) {
        return false;
      }

      // Copy the (possibly strided) capture into the packed src dma-buf.
      sync(src_buf.fd, true, DMA_BUF_SYNC_WRITE);
      copy_src_in(src, src_buf);
      sync(src_buf.fd, false, DMA_BUF_SYNC_WRITE);

      // Pre-clear letterbox bars; only needed when the output geometry changes
      // since RGA overwrites just the active rectangle each frame.
      const bool letterboxed = region.out_w != dst_w || region.out_h != dst_h;
      if (letterboxed && geometry_changed(region, dst_w, dst_h)) {
        sync(dst_buf.fd, true, DMA_BUF_SYNC_WRITE);
        fill_black(dst_buf, dst_w, dst_h);
        sync(dst_buf.fd, false, DMA_BUF_SYNC_WRITE);
      }
      last_region = region;
      last_dst_w = dst_w;
      last_dst_h = dst_h;

      rga_buffer_t s = wrapbuffer_handle(src_buf.handle, src.width, src.height, rga_src_format(src));
      rga_buffer_t d = wrapbuffer_handle(dst_buf.handle, dst_w, dst_h, RK_FORMAT_YCbCr_420_SP);
      im_rect srect {region.src_left, region.src_top, region.src_w, region.src_h};
      im_rect drect {region.off_x, region.off_y, region.out_w, region.out_h};
      im_rect prect {};
      im_opt_t opt {};
      opt.interp = interp;
      rga_buffer_t pat {};
      auto status = improcess(s, d, pat, srect, drect, prect, -1, nullptr, &opt, IM_SYNC);
      if (status != IM_STATUS_SUCCESS) {
        if (!fail_logged) {
          BOOST_LOG(warning) << "RGA: improcess failed: "sv << imStrError_t(status);
          fail_logged = true;
        }
        return false;
      }
      fail_logged = false;

      // Copy the scaled NV12 result into the encoder's mapped frame.
      sync(dst_buf.fd, true, DMA_BUF_SYNC_READ);
      copy_nv12_out(dst_buf, dst, dst_w, dst_h);
      sync(dst_buf.fd, false, DMA_BUF_SYNC_READ);
      return true;
    }

    // Direct-encode path (SUNSHINE_RKMPP_RGA_DIRECT): RGA scales into a CMA
    // buffer that the encoder imports as a DRM_PRIME frame, removing the output
    // copy. Returns a DRM_PRIME AVFrame to hand to the encoder, or nullptr on a
    // hard allocation failure. The encoder buffer itself is not RGA-addressable
    // on this SoC, hence the import-our-own-buffer approach.
    AVFrame *scale_direct(const direct_frame_t &src, const scale_region_t &region, int dst_w, int dst_h, AVBufferRef *hw_frames_ctx) {
      if (!rga_src_supported(src) || !ensure_enc(dst_w, dst_h)) {
        return nullptr;
      }

      // Zero-copy source: import the capture buffer's dma-buf and let RGA read
      // it in place. The CPU-copy staging buffer is only a fallback — V4L2
      // mmap memory is uncached, so the per-frame memcpy out of it costs more
      // CPU than everything else in the pipeline combined (a 1080p60 BGR24
      // stream is ~370 MB/s of uncached reads).
      rga_buffer_handle_t imported = 0;
      if (src.dmabuf_fd >= 0 && src.dmabuf_size > 0) {
        imported = importbuffer_fd(src.dmabuf_fd, (int) src.dmabuf_size);
        if (!imported && !import_fail_logged) {
          BOOST_LOG(warning) << "RGA: couldn't import capture dma-buf; falling back to CPU copy"sv;
          import_fail_logged = true;
        }
      }
      if (!imported) {
        if (!ensure(src_buf, src.width, src.height, src_frame_bytes(src))) {
          return nullptr;
        }
        sync(src_buf.fd, true, DMA_BUF_SYNC_WRITE);
        copy_src_in(src, src_buf);
        sync(src_buf.fd, false, DMA_BUF_SYNC_WRITE);
      }

      // Clear letterbox bars when the output geometry changes (the encoder frame
      // is OVERWRITE-mapped, so bars are not preserved across a geometry change).
      const bool letterboxed = region.out_w != dst_w || region.out_h != dst_h;
      if (letterboxed && geometry_changed(region, dst_w, dst_h)) {
        sync(enc_buf.fd, true, DMA_BUF_SYNC_WRITE);
        fill_black_strided(enc_buf, enc_wstride, enc_hstride);
        sync(enc_buf.fd, false, DMA_BUF_SYNC_WRITE);
      }
      last_region = region;
      last_dst_w = dst_w;
      last_dst_h = dst_h;

      rga_buffer_t s;
      if (imported) {
        // The V4L2 buffer can be strided; wrapbuffer strides are in pixels.
        const int wstride_px = src.fourcc == V4L2_PIX_FMT_BGR24 ? src.stride / 3 : src.stride;
        s = wrapbuffer_handle(imported, src.width, src.height, rga_src_format(src), wstride_px, src.height);
      } else {
        s = wrapbuffer_handle(src_buf.handle, src.width, src.height, rga_src_format(src));
      }
      rga_buffer_t d = wrapbuffer_handle(enc_buf.handle, dst_w, dst_h, RK_FORMAT_YCbCr_420_SP, enc_wstride, enc_hstride);
      im_rect srect {region.src_left, region.src_top, region.src_w, region.src_h};
      im_rect drect {region.off_x, region.off_y, region.out_w, region.out_h};
      im_rect prect {};
      im_opt_t opt {};
      opt.interp = interp;
      rga_buffer_t pat {};
      auto status = improcess(s, d, pat, srect, drect, prect, -1, nullptr, &opt, IM_SYNC);
      if (imported) {
        releasebuffer_handle(imported);
      }
      if (status != IM_STATUS_SUCCESS) {
        if (!fail_logged) {
          BOOST_LOG(warning) << "RGA: direct improcess failed: "sv << imStrError_t(status);
          fail_logged = true;
        }
      } else {
        fail_logged = false;
      }
      return enc_frame(hw_frames_ctx, dst_w, dst_h);
    }

    // Direct stream: copy the (already-NV12, same-size) capture straight into the
    // encoder buffer and wrap it as DRM_PRIME — no RGA scaling pass. Used when the
    // stream settings exactly match the capture so no scale/crop/letterbox is
    // needed; the 2D engine round-trip is skipped entirely.
    AVFrame *passthrough_direct(const direct_frame_t &src, int dst_w, int dst_h, AVBufferRef *hw_frames_ctx) {
      if (!src.nv12 || src.width != dst_w || src.height != dst_h || !ensure_enc(dst_w, dst_h)) {
        return nullptr;
      }
      sync(enc_buf.fd, true, DMA_BUF_SYNC_WRITE);
      auto *dstp = (std::uint8_t *) enc_buf.map;
      for (int row = 0; row < src.height; ++row) {
        std::memcpy(dstp + (std::size_t) row * enc_wstride, src.data + (std::size_t) row * src.stride, src.width);
      }
      const auto *uv = src.data + (std::size_t) src.stride * src.height;
      auto *dstuv = dstp + (std::size_t) enc_wstride * enc_hstride;
      for (int row = 0; row < src.height / 2; ++row) {
        std::memcpy(dstuv + (std::size_t) row * enc_wstride, uv + (std::size_t) row * src.stride, src.width);
      }
      sync(enc_buf.fd, false, DMA_BUF_SYNC_WRITE);
      last_dst_w = -1;  // not a scaled frame; force bar re-clear if scaling resumes
      return enc_frame(hw_frames_ctx, dst_w, dst_h);
    }

    // Produce a black DRM_PRIME frame for the direct-encode path (no capture).
    AVFrame *black_direct(int dst_w, int dst_h, AVBufferRef *hw_frames_ctx) {
      if (!ensure_enc(dst_w, dst_h)) {
        return nullptr;
      }
      sync(enc_buf.fd, true, DMA_BUF_SYNC_WRITE);
      fill_black_strided(enc_buf, enc_wstride, enc_hstride);
      sync(enc_buf.fd, false, DMA_BUF_SYNC_WRITE);
      last_dst_w = -1;  // force bar re-clear on the next real frame
      return enc_frame(hw_frames_ctx, dst_w, dst_h);
    }

  private:
    struct buffer_t {
      int fd {-1};
      void *map {};
      std::size_t len {};
      int w {};
      int h {};
      rga_buffer_handle_t handle {};
    };

    // Map the shared scale-filter setting to an RGA interpolation mode. nearest
    // never reaches RGA (it is routed to the GPU path), so it falls to default.
    static int interp_from_filter(scale_filter_e filter) {
      switch (filter) {
        case scale_filter_e::linear:
          return IM_INTERP_LINEAR;
        case scale_filter_e::cubic:
          return IM_INTERP_CUBIC;
        case scale_filter_e::average:
          return IM_INTERP_AVERAGE;
        case scale_filter_e::automatic:
        case scale_filter_e::nearest:
        default:
          return IM_INTERP_DEFAULT;
      }
    }

    static int parse_interp(const char *env) {
      if (!env || !*env) {
        return IM_INTERP_DEFAULT;
      }
      std::string v(env);
      std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return (char) std::tolower(c); });
      v.erase(std::remove_if(v.begin(), v.end(), [](unsigned char c) { return std::isspace(c) || c == '-' || c == '_'; }), v.end());
      if (v == "linear" || v == "bilinear") {
        return IM_INTERP_LINEAR;
      }
      if (v == "cubic" || v == "bicubic") {
        return IM_INTERP_CUBIC;
      }
      if (v == "average" || v == "avg") {
        return IM_INTERP_AVERAGE;
      }
      return IM_INTERP_DEFAULT;
    }

    bool alloc(buffer_t &b, std::size_t len) {
      dma_heap_allocation_data data {};
      data.len = len;
      data.fd_flags = O_RDWR | O_CLOEXEC;
      if (xioctl(heap_fd, DMA_HEAP_IOCTL_ALLOC, &data) < 0) {
        return false;
      }
      b.fd = (int) data.fd;
      b.len = len;
      b.map = mmap(nullptr, len, PROT_READ | PROT_WRITE, MAP_SHARED, b.fd, 0);
      if (b.map == MAP_FAILED) {
        b.map = nullptr;
        close(b.fd);
        b.fd = -1;
        return false;
      }
      return true;
    }

    void release(buffer_t &b) {
      if (b.handle > 0) {
        releasebuffer_handle(b.handle);
        b.handle = 0;
      }
      if (b.map) {
        munmap(b.map, b.len);
        b.map = nullptr;
      }
      if (b.fd >= 0) {
        close(b.fd);
        b.fd = -1;
      }
      b.w = b.h = 0;
      b.len = 0;
    }

    bool ensure(buffer_t &b, int w, int h, std::size_t bytes) {
      if (b.fd >= 0 && b.w == w && b.h == h && b.len >= bytes) {
        return true;
      }
      release(b);
      if (!alloc(b, bytes)) {
        return false;
      }
      b.handle = importbuffer_fd(b.fd, (int) b.len);
      if (b.handle <= 0) {
        release(b);
        return false;
      }
      b.w = w;
      b.h = h;
      return true;
    }

    // Frame byte size of a direct capture frame in the packed src dma-buf
    // (NV12 or BGR24 — the two formats the RGA source path accepts).
    static std::size_t src_frame_bytes(const direct_frame_t &src) {
      const std::size_t px = (std::size_t) src.width * src.height;
      return src.fourcc == V4L2_PIX_FMT_BGR24 ? px * 3 : px * 3 / 2;
    }

    // Whether the RGA source path can consume this capture frame directly.
    static bool rga_src_supported(const direct_frame_t &src) {
      return src.nv12 || src.fourcc == V4L2_PIX_FMT_BGR24;
    }

    // librga source descriptor format for a supported capture frame. RGA does
    // the BGR888 -> NV12 color-space conversion in the same pass as the scale.
    static int rga_src_format(const direct_frame_t &src) {
      return src.fourcc == V4L2_PIX_FMT_BGR24 ? RK_FORMAT_BGR_888 : RK_FORMAT_YCbCr_420_SP;
    }

    void sync(int fd, bool start, std::uint64_t rw) {
      dma_buf_sync s {};
      s.flags = rw | (start ? DMA_BUF_SYNC_START : DMA_BUF_SYNC_END);
      xioctl(fd, DMA_BUF_IOCTL_SYNC, &s);
    }

    void copy_nv12_in(const direct_frame_t &src, buffer_t &b) {
      auto *dstp = (std::uint8_t *) b.map;
      for (int row = 0; row < src.height; ++row) {
        std::memcpy(dstp + (std::size_t) row * src.width, src.data + (std::size_t) row * src.stride, src.width);
      }
      const auto *uv = src.data + (std::size_t) src.stride * src.height;
      auto *dstuv = dstp + (std::size_t) src.width * src.height;
      for (int row = 0; row < src.height / 2; ++row) {
        std::memcpy(dstuv + (std::size_t) row * src.width, uv + (std::size_t) row * src.stride, src.width);
      }
    }

    // Copy a (possibly strided) BGR24 capture into the packed src dma-buf.
    void copy_bgr_in(const direct_frame_t &src, buffer_t &b) {
      auto *dstp = (std::uint8_t *) b.map;
      const auto row_bytes = (std::size_t) src.width * 3;
      for (int row = 0; row < src.height; ++row) {
        std::memcpy(dstp + (std::size_t) row * row_bytes, src.data + (std::size_t) row * src.stride, row_bytes);
      }
    }

    void copy_src_in(const direct_frame_t &src, buffer_t &b) {
      if (src.fourcc == V4L2_PIX_FMT_BGR24) {
        copy_bgr_in(src, b);
      } else {
        copy_nv12_in(src, b);
      }
    }

    void copy_nv12_out(buffer_t &b, AVFrame *dst, int dst_w, int dst_h) {
      const auto *srcp = (const std::uint8_t *) b.map;
      for (int row = 0; row < dst_h; ++row) {
        std::memcpy(dst->data[0] + (std::size_t) row * dst->linesize[0], srcp + (std::size_t) row * dst_w, dst_w);
      }
      const auto *srcuv = srcp + (std::size_t) dst_w * dst_h;
      for (int row = 0; row < dst_h / 2; ++row) {
        std::memcpy(dst->data[1] + (std::size_t) row * dst->linesize[1], srcuv + (std::size_t) row * dst_w, dst_w);
      }
    }

    void fill_black(buffer_t &b, int w, int h) {
      auto *p = (std::uint8_t *) b.map;
      std::memset(p, 16, (std::size_t) w * h);
      std::memset(p + (std::size_t) w * h, 128, (std::size_t) w * h / 2);
    }

    // Black-fill the whole strided encoder buffer (Y=16, UV=128).
    void fill_black_strided(buffer_t &b, int ws, int hs) {
      auto *p = (std::uint8_t *) b.map;
      std::memset(p, 16, (std::size_t) ws * hs);
      std::memset(p + (std::size_t) ws * hs, 128, (std::size_t) ws * hs / 2);
    }

    bool geometry_changed(const scale_region_t &r, int dst_w, int dst_h) const {
      return dst_w != last_dst_w || dst_h != last_dst_h ||
             r.off_x != last_region.off_x || r.off_y != last_region.off_y ||
             r.out_w != last_region.out_w || r.out_h != last_region.out_h;
    }

    // The encoder buffer uses 16-aligned strides (MPP encode requirement). Y is
    // packed at stride enc_wstride; UV starts at enc_wstride*enc_hstride.
    bool ensure_enc(int w, int h) {
      const int ws = (w + 15) & ~15;
      const int hs = (h + 15) & ~15;
      if (enc_buf.fd >= 0 && enc_wstride == ws && enc_hstride == hs) {
        return true;
      }
      free_enc_frame();
      release(enc_buf);
      if (!alloc(enc_buf, (std::size_t) ws * hs * 3 / 2)) {
        return false;
      }
      enc_buf.handle = importbuffer_fd(enc_buf.fd, (int) enc_buf.len);
      if (enc_buf.handle <= 0) {
        release(enc_buf);
        return false;
      }
      enc_buf.w = w;
      enc_buf.h = h;
      enc_wstride = ws;
      enc_hstride = hs;
      // Clear once so the stride padding (read as edge macroblocks by the
      // encoder) is black; RGA overwrites the visible region each frame.
      sync(enc_buf.fd, true, DMA_BUF_SYNC_WRITE);
      fill_black_strided(enc_buf, ws, hs);
      sync(enc_buf.fd, false, DMA_BUF_SYNC_WRITE);
      return true;
    }

    // Build (once per encoder buffer) and return a DRM_PRIME AVFrame wrapping
    // enc_buf as NV12, for the rkmpp encoder to import.
    AVFrame *enc_frame(AVBufferRef *hw_frames_ctx, int dst_w, int dst_h) {
      if (enc_frame_ptr) {
        return enc_frame_ptr;
      }
      auto *desc = (AVDRMFrameDescriptor *) av_mallocz(sizeof(AVDRMFrameDescriptor));
      if (!desc) {
        return nullptr;
      }
      desc->nb_objects = 1;
      desc->objects[0].fd = enc_buf.fd;
      desc->objects[0].size = enc_buf.len;
      desc->objects[0].format_modifier = DRM_FORMAT_MOD_LINEAR;
      desc->nb_layers = 1;
      desc->layers[0].format = DRM_FORMAT_NV12;
      desc->layers[0].nb_planes = 2;
      desc->layers[0].planes[0].object_index = 0;
      desc->layers[0].planes[0].offset = 0;
      desc->layers[0].planes[0].pitch = enc_wstride;
      desc->layers[0].planes[1].object_index = 0;
      desc->layers[0].planes[1].offset = (std::ptrdiff_t) enc_wstride * enc_hstride;
      desc->layers[0].planes[1].pitch = enc_wstride;

      AVFrame *f = av_frame_alloc();
      if (!f) {
        av_free(desc);
        return nullptr;
      }
      f->format = AV_PIX_FMT_DRM_PRIME;
      f->width = dst_w;
      f->height = dst_h;
      f->data[0] = (std::uint8_t *) desc;
      f->buf[0] = av_buffer_create((std::uint8_t *) desc, sizeof(*desc),
                                   [](void *, std::uint8_t *d) { av_free(d); }, nullptr, 0);
      if (hw_frames_ctx) {
        f->hw_frames_ctx = av_buffer_ref(hw_frames_ctx);
      }
      enc_frame_ptr = f;
      return enc_frame_ptr;
    }

    void free_enc_frame() {
      if (enc_frame_ptr) {
        av_frame_free(&enc_frame_ptr);
      }
    }

    int heap_fd {-1};
    std::string heap_name;
    int interp {IM_INTERP_DEFAULT};
    buffer_t src_buf;
    buffer_t dst_buf;
    buffer_t enc_buf;
    int enc_wstride {};
    int enc_hstride {};
    AVFrame *enc_frame_ptr {};
    scale_region_t last_region;
    int last_dst_w {-1};
    int last_dst_h {-1};
    bool fail_logged {};
    bool import_fail_logged {};
  };
#endif

  class rkmpp_t: public platf::avcodec_encode_device_t {
  public:
    ~rkmpp_t() override {
#ifdef SUNSHINE_BUILD_RGA
      if (enc_hw_frames_ctx) {
        av_buffer_unref(&enc_hw_frames_ctx);
      }
#endif
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

      parse_capture_resolution(std::getenv("SUNSHINE_RKMPP_CAPTURE_RESOLUTION"), capture_width, capture_height);
      passthrough_enabled = parse_pipeline_enabled(std::getenv("SUNSHINE_RKMPP_PASSTHROUGH"));

      shader_effect = parse_shader_effect(std::getenv("SUNSHINE_RKMPP_SHADER"));
      if (shader_effect == shader_effect_e::crt_basic) {
        crt = parse_crt_params(std::getenv("SUNSHINE_RKMPP_CRT_PARAMS"));
        if (passthrough_enabled) {
          // Passthrough hands captured frames straight to the encoder; the
          // effect only exists in the GPU scale pass, so it must always run.
          passthrough_enabled = false;
        }
        BOOST_LOG(info) << "RKMPP direct V4L2: CRT shader enabled (scan="sv << crt.scan
                        << ", sharp="sv << crt.sharp << ", mask="sv << crt.mask
                        << ", bright="sv << crt.bright << ", soft="sv << crt.soft
                        << ", pitch="sv << crt.pitch << ", fade="sv << crt.fade
                        << "); using GPU scaler"sv;
      }

#ifdef SUNSHINE_BUILD_RGA
      // The RGA 2D engine is the default capture scaler (much lower latency than
      // the Mali GPU on Rockchip). SUNSHINE_RKMPP_SCALER=gpu forces the GPU
      // path; RGA also falls back to GPU automatically when it is unavailable
      // (e.g. CMA too small) or when the nearest filter is requested.
      use_rga = true;
      if (auto scaler = std::getenv("SUNSHINE_RKMPP_SCALER")) {
        std::string v(scaler);
        std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return (char) std::tolower(c); });
        use_rga = !(v == "gpu" || v == "egl" || v == "opengl" || v == "off" || v == "none");
      }
      // RGA writes a CMA buffer the encoder imports as DRM_PRIME, removing the
      // output copy (lower latency, less CPU). Default on; SUNSHINE_RKMPP_RGA_DIRECT=0
      // falls back to the copy path if a board's encoder rejects imported frames.
      rga_direct = true;
      if (auto direct = std::getenv("SUNSHINE_RKMPP_RGA_DIRECT")) {
        std::string v(direct);
        std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return (char) std::tolower(c); });
        rga_direct = !(v == "0" || v == "off" || v == "false" || v == "no");
      }
#endif

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
        // Capture at the forced resolution when set, otherwise at the stream
        // resolution. The scaler (RGA/GPU/CPU) handles the size difference before
        // encode. A smaller capture can cut capture-card/USB and conversion work
        // on SD sources, but the encode still runs at the client-requested stream
        // resolution.
        const int cap_w = capture_width ? capture_width : frame->width;
        const int cap_h = capture_height ? capture_height : frame->height;
        if (capture_width || capture_height) {
          BOOST_LOG(info) << "RKMPP direct V4L2: forcing capture "sv << cap_w << 'x' << cap_h
                          << " -> stream "sv << frame->width << 'x' << frame->height;
        }
        auto source = std::make_unique<v4l2_nv12_source_t>();
        if (source->init(direct_v4l2_device.c_str(), cap_w, cap_h, 60)) {
          direct_v4l2 = std::move(source);
        } else {
          BOOST_LOG(warning) << "RKMPP direct V4L2 unavailable; falling back to KMS GPU-convert path"sv;
        }
      }

      if (direct_v4l2) {
        BOOST_LOG(info) << "Using RKMPP direct V4L2 encode path"sv;
        auto nv12_opt = egl::create_target(frame->width, frame->height, (AVPixelFormat) AV_PIX_FMT_NV12);
        if (!nv12_opt) {
          return -1;
        }
        this->nv12 = std::move(*nv12_opt);
#ifdef SUNSHINE_BUILD_RGA
        // RGA handles every filter except nearest-neighbour, which it lacks; the
        // nearest filter therefore routes to the GPU path (which does nearest).
        // Shader effects also require the GPU: RGA has no programmable stage.
        const auto filter = direct_v4l2->get_scale_filter();
        if (use_rga && shader_effect != shader_effect_e::disabled) {
          BOOST_LOG(info) << "RKMPP direct V4L2: shader effect selected; using GPU scaler (RGA has no programmable stage)"sv;
        } else if (use_rga && filter == scale_filter_e::nearest) {
          BOOST_LOG(info) << "RKMPP direct V4L2: nearest filter selected; using GPU scaler (RGA has no nearest)"sv;
        } else if (use_rga) {
          if (rga.init(frame->width, frame->height, filter)) {
            rga_ready = true;
            if (rga_direct) {
              enc_hw_frames_ctx = av_buffer_ref(hw_frames_ctx_buf);
            }
            BOOST_LOG(info) << "RKMPP direct V4L2: RGA 2D scaler enabled"sv
                            << (rga_direct ? " (direct dma-buf encode)"sv : ""sv);
          } else {
            BOOST_LOG(warning) << "RKMPP direct V4L2: RGA scaler unavailable; falling back to GPU scaling"sv;
          }
        }
#endif
        if (!rga_ready && init_direct_v4l2_gpu()) {
          BOOST_LOG(info) << "RKMPP direct V4L2: GPU scaling/conversion enabled"sv;
        } else if (!rga_ready) {
          BOOST_LOG(warning) << "RKMPP direct V4L2: GPU scaling unavailable; falling back to CPU scaling"sv;
        }
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

#ifdef SUNSHINE_BUILD_RGA
      // Direct-encode RGA path: scale into a CMA buffer and hand the encoder a
      // DRM_PRIME frame wrapping it (no map, no copy). Replaces 'frame' with the
      // wrapper so encode_avcodec sends it.
      if (direct_v4l2 && rga_ready && rga_direct) {
        const int w = frame->width;
        const int h = frame->height;
        const bool passthrough = passthrough_enabled && direct_v4l2->passthrough_ok(w, h);
        AVFrame *enc;
        if (!direct_v4l2->update_latest_frame()) {
          if (direct_v4l2->should_log_missing_frame()) {
            BOOST_LOG(warning) << "RKMPP direct V4L2: no capture frame available; sending black frames until capture resumes"sv;
          }
          enc = rga.black_direct(w, h, enc_hw_frames_ctx);
        } else if (passthrough) {
          // Stream settings match the capture exactly: no scaling needed. With
          // an exported capture dma-buf the identity blit still goes through
          // RGA — a 2D-engine copy beats memcpying the frame out of uncached
          // V4L2 mmap memory. The CPU passthrough copy remains the fallback.
          const auto direct = direct_v4l2->latest_direct_frame();
          if (direct.dmabuf_fd >= 0) {
            enc = rga.scale_direct(direct, direct_v4l2->scale_region(w, h), w, h, enc_hw_frames_ctx);
          } else {
            enc = rga.passthrough_direct(direct, w, h, enc_hw_frames_ctx);
          }
        } else {
          enc = rga.scale_direct(direct_v4l2->latest_direct_frame(), direct_v4l2->scale_region(w, h), w, h, enc_hw_frames_ctx);
        }
        if (!enc) {
          BOOST_LOG(error) << "RKMPP direct V4L2: RGA direct-encode buffer unavailable"sv;
          return -1;
        }
        this->frame = enc;
        return 0;
      }
#endif

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
        // Direct stream: when the stream settings match the capture exactly, copy
        // the captured NV12 straight into the encoder frame, skipping the RGA/GPU
        // scaling pass entirely.
        if (passthrough_enabled && direct_v4l2->passthrough_ok(frame->width, frame->height)) {
          if (!direct_v4l2->copy_latest_to(mapped_frame.get(), frame->width, frame->height)) {
            if (direct_v4l2->should_log_missing_frame()) {
              BOOST_LOG(warning) << "RKMPP direct V4L2: no capture frame available; sending black frames until capture resumes"sv;
            }
            direct_v4l2->copy_black_to(mapped_frame.get(), frame->width, frame->height);
          }
          return 0;
        }
#ifdef SUNSHINE_BUILD_RGA
        if (rga_ready) {
          // RGA 2D engine path: scale/convert on the dedicated 2D block instead
          // of the Mali GPU. Synchronous (RGA is fast and on its own engine).
          if (!direct_v4l2->update_latest_frame()) {
            if (direct_v4l2->should_log_missing_frame()) {
              BOOST_LOG(warning) << "RKMPP direct V4L2: no capture frame available; sending black frames until capture resumes"sv;
            }
            direct_v4l2->copy_black_to(mapped_frame.get(), frame->width, frame->height);
          } else if (!rga.scale(direct_v4l2->latest_direct_frame(), direct_v4l2->scale_region(frame->width, frame->height), mapped_frame.get(), frame->width, frame->height)) {
            if (direct_v4l2->should_log_missing_frame()) {
              BOOST_LOG(warning) << "RKMPP direct V4L2: RGA scale failed; sending black frame"sv;
            }
            direct_v4l2->copy_black_to(mapped_frame.get(), frame->width, frame->height);
          }
          return 0;
        }
#endif
        if (direct_gpu_ready && pipeline_enabled) {
          // Pipelined path: hand the encoder the frame the GPU rendered during
          // the previous encode cycle, then capture the next frame and kick its
          // render so the GPU works while the VPU encodes what we just handed
          // off. The render flushed last cycle is already complete, so this
          // readback does not stall. Costs one frame of latency.
          if (gpu_render_pending) {
            readback_direct_v4l2_gpu(mapped_frame.get());
          } else {
            direct_v4l2->copy_black_to(mapped_frame.get(), frame->width, frame->height);
          }

          gpu_render_pending = false;
          if (!direct_v4l2->update_latest_frame()) {
            if (direct_v4l2->should_log_missing_frame()) {
              BOOST_LOG(warning) << "RKMPP direct V4L2: no capture frame available; sending black frames until capture resumes"sv;
            }
          } else if (render_direct_v4l2_gpu(direct_v4l2->latest_direct_frame(), direct_v4l2->scale_region(frame->width, frame->height))) {
            gpu_render_pending = true;
          } else if (direct_v4l2->should_log_missing_frame()) {
            BOOST_LOG(warning) << "RKMPP direct V4L2: GPU render failed; sending black frame"sv;
          }
          return 0;
        }

        // Synchronous path (SUNSHINE_RKMPP_PIPELINE=off or GPU unavailable).
        if (!direct_v4l2->update_latest_frame()) {
          if (direct_v4l2->should_log_missing_frame()) {
            BOOST_LOG(warning) << "RKMPP direct V4L2: no capture frame available; sending black frames until capture resumes"sv;
          }
          direct_v4l2->copy_black_to(mapped_frame.get(), frame->width, frame->height);
        } else if (direct_gpu_ready && convert_direct_v4l2_gpu(direct_v4l2->latest_direct_frame(), direct_v4l2->scale_region(frame->width, frame->height), mapped_frame.get())) {
          return 0;
        } else {
          BOOST_LOG(warning) << "RKMPP direct V4L2: GPU conversion failed; sending black frame"sv;
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
    // Forced V4L2 capture resolution (SUNSHINE_RKMPP_CAPTURE_RESOLUTION). 0 = auto,
    // i.e. capture at the client-requested stream resolution (the default). When
    // set, the device captures at this size and the scaler upscales/downscales to
    // the stream resolution before encode. This can save capture-side work on SD
    // sources, but not encoder resolution.
    int capture_width {};
    int capture_height {};
    // Direct-stream fast path: when the stream settings already match the capture
    // (no scale/crop/letterbox), feed the captured frame to the encoder unscaled.
    // SUNSHINE_RKMPP_PASSTHROUGH=0 disables it for comparison.
    bool passthrough_enabled {true};
    std::unique_ptr<v4l2_nv12_source_t> direct_v4l2;
    gl::tex_t direct_v4l2_tex;
    // 0 = NV12 (both planes), 1 = YUYV luma, 2 = YUYV chroma,
    // 3 = NV12 luma with CRT effect, 4 = YUYV luma with CRT effect (3/4 are
    // only linked when SUNSHINE_RKMPP_SHADER selects an effect).
    direct_program_t direct_v4l2_program[5];
    // GPU post-process effect (SUNSHINE_RKMPP_SHADER). Applied to the luma
    // plane only: scaling chroma samples would shift colors, since U/V are
    // biased around 128 rather than 0.
    shader_effect_e shader_effect {shader_effect_e::disabled};
    bool crt_programs_ready {};
    crt_params_t crt;
    bool direct_gpu_ready {};
    bool pipeline_enabled {true};
    bool gpu_render_pending {};
    int direct_tex_width {};
    int direct_tex_height {};
    std::uint32_t direct_tex_fourcc {};
    bool rga_ready {};
#ifdef SUNSHINE_BUILD_RGA
    bool use_rga {};
    bool rga_direct {};
    rga_scaler_t rga;
    AVBufferRef *enc_hw_frames_ctx {};
#endif

  private:
    // SUNSHINE_RKMPP_PIPELINE toggles deferred-readback pipelining (default on).
    // Disabling restores the synchronous render+readback path for comparison.
    static bool parse_pipeline_enabled(const char *env) {
      if (!env || !*env) {
        return true;
      }
      std::string v(env);
      std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return (char) std::tolower(c); });
      v.erase(std::remove_if(v.begin(), v.end(), [](unsigned char c) { return std::isspace(c) || c == '-' || c == '_'; }), v.end());
      if (v == "off" || v == "false" || v == "0" || v == "disabled" || v == "none" || v == "sync") {
        return false;
      }
      return true;
    }

    // SUNSHINE_RKMPP_SHADER selects a GPU post-process effect applied during
    // the scale/convert pass. Effects need the programmable GPU path, so
    // enabling one routes scaling away from RGA and disables passthrough.
    static shader_effect_e parse_shader_effect(const char *env) {
      if (!env || !*env) {
        return shader_effect_e::disabled;
      }
      std::string v(env);
      std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return (char) std::tolower(c); });
      v.erase(std::remove_if(v.begin(), v.end(), [](unsigned char c) { return std::isspace(c) || c == '-' || c == '_'; }), v.end());
      if (v.empty() || v == "disabled" || v == "off" || v == "none" || v == "0") {
        return shader_effect_e::disabled;
      }
      if (v == "crtbasic" || v == "crt") {
        return shader_effect_e::crt_basic;
      }
      BOOST_LOG(warning) << "RKMPP direct V4L2: invalid SUNSHINE_RKMPP_SHADER '"sv << env << "'; shader disabled"sv;
      return shader_effect_e::disabled;
    }

    // SUNSHINE_RKMPP_CRT_PARAMS tunes the CRT effect as a comma-separated
    // "key=value" list, e.g. "scan=0.35,sharp=0.4,mask=0.12,bright=0.15,
    // soft=0,pitch=3,fade=1". Unknown keys warn; missing keys keep defaults;
    // values are clamped to their supported ranges.
    static crt_params_t parse_crt_params(const char *env) {
      crt_params_t out;
      if (!env || !*env) {
        return out;
      }
      std::string v(env);
      std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return (char) std::tolower(c); });
      v.erase(std::remove_if(v.begin(), v.end(), [](unsigned char c) { return std::isspace(c); }), v.end());

      std::size_t pos = 0;
      while (pos < v.size()) {
        auto comma = v.find(',', pos);
        auto token = v.substr(pos, comma == std::string::npos ? std::string::npos : comma - pos);
        pos = comma == std::string::npos ? v.size() : comma + 1;
        auto eq = token.find('=');
        if (token.empty()) {
          continue;
        }
        if (eq == std::string::npos) {
          BOOST_LOG(warning) << "RKMPP direct V4L2: ignoring SUNSHINE_RKMPP_CRT_PARAMS token '"sv << token << "'"sv;
          continue;
        }
        auto key = token.substr(0, eq);
        auto val = token.substr(eq + 1);
        try {
          if (key == "scan") {
            out.scan = std::clamp(std::stof(val), 0.0f, 1.0f);
          } else if (key == "sharp") {
            out.sharp = std::clamp(std::stof(val), 0.0f, 1.0f);
          } else if (key == "mask") {
            out.mask = std::clamp(std::stof(val), 0.0f, 1.0f);
          } else if (key == "bright") {
            out.bright = std::clamp(std::stof(val), 0.0f, 0.5f);
          } else if (key == "soft") {
            out.soft = std::clamp(std::stof(val), 0.0f, 1.0f);
          } else if (key == "pitch") {
            out.pitch = std::clamp(std::stof(val), 2.0f, 4.0f);
          } else if (key == "fade") {
            out.fade = !(val == "0" || val == "off" || val == "false" || val == "no");
          } else {
            BOOST_LOG(warning) << "RKMPP direct V4L2: unknown SUNSHINE_RKMPP_CRT_PARAMS key '"sv << key << "'"sv;
          }
        } catch (...) {
          BOOST_LOG(warning) << "RKMPP direct V4L2: invalid SUNSHINE_RKMPP_CRT_PARAMS value '"sv << token << "'"sv;
        }
      }
      return out;
    }

    // SUNSHINE_RKMPP_CAPTURE_RESOLUTION forces the V4L2 capture size as
    // "WIDTHxHEIGHT" (e.g. "720x480"). Empty/"auto" leaves capture at the
    // client-requested stream resolution. Parsed values are written to out_w/out_h
    // (left at 0 on auto/invalid input).
    static void parse_capture_resolution(const char *env, int &out_w, int &out_h) {
      out_w = 0;
      out_h = 0;
      if (!env || !*env) {
        return;
      }
      std::string v(env);
      std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return (char) std::tolower(c); });
      v.erase(std::remove_if(v.begin(), v.end(), [](unsigned char c) { return std::isspace(c); }), v.end());
      if (v.empty() || v == "auto" || v == "0" || v == "native" || v == "match" || v == "stream") {
        return;
      }
      auto x = v.find('x');
      if (x == std::string::npos) {
        x = v.find('*');
      }
      if (x != std::string::npos) {
        try {
          auto w = std::stoi(v.substr(0, x));
          auto h = std::stoi(v.substr(x + 1));
          if (w >= 2 && h >= 2) {
            out_w = w & ~1;
            out_h = h & ~1;
            return;
          }
        } catch (...) {
        }
      }
      BOOST_LOG(warning) << "RKMPP direct V4L2: invalid SUNSHINE_RKMPP_CAPTURE_RESOLUTION '"sv << env << "'; using stream resolution"sv;
    }

    util::Either<gl::program_t, std::string> make_direct_program(const std::string_view &fragment_source) {
      static constexpr std::string_view vertex_source = R"glsl(
#version 300 es

#ifdef GL_ES
precision mediump float;
#endif

out vec2 tex;

void main()
{
  float idHigh = float(gl_VertexID >> 1);
  float idLow = float(gl_VertexID & int(1));

  float x = idHigh * 4.0 - 1.0;
  float y = idLow * 4.0 - 1.0;

  float u = idHigh * 2.0;
  float v = idLow * 2.0;

  gl_Position = vec4(x, y, 0.0, 1.0);
  tex = vec2(u, v);
}
)glsl";

      auto vertex = gl::shader_t::compile(vertex_source, GL_VERTEX_SHADER);
      if (vertex.has_right()) {
        return vertex.right();
      }

      auto fragment = gl::shader_t::compile(fragment_source, GL_FRAGMENT_SHADER);
      if (fragment.has_right()) {
        return fragment.right();
      }

      return gl::program_t::link(vertex.left(), fragment.left());
    }

    bool init_direct_v4l2_gpu() {
      static constexpr std::string_view nv12_fragment = R"glsl(
#version 300 es

#ifdef GL_ES
precision mediump float;
#endif

uniform sampler2D image;
uniform vec4 src_rect;

in vec2 tex;
layout(location = 0) out vec4 color;

void main()
{
  color = texture(image, mix(src_rect.xy, src_rect.zw, tex));
}
)glsl";

      static constexpr std::string_view yuyv_y_fragment = R"glsl(
#version 300 es

#ifdef GL_ES
precision mediump float;
#endif

uniform sampler2D image;
uniform vec4 src_rect;
uniform float src_width;

in vec2 tex;
layout(location = 0) out float color;

void main()
{
  vec2 st = mix(src_rect.xy, src_rect.zw, tex);
  float x = max(0.0, st.x * src_width - 0.5);
  float pair_x = floor(x * 0.5);
  vec4 yuyv = texture(image, vec2((pair_x + 0.5) / (src_width * 0.5), st.y));
  color = mod(floor(x + 0.5), 2.0) < 1.0 ? yuyv.r : yuyv.b;
}
)glsl";

      static constexpr std::string_view yuyv_uv_fragment = R"glsl(
#version 300 es

#ifdef GL_ES
precision mediump float;
#endif

uniform sampler2D image;
uniform vec4 src_rect;
uniform float src_width;

in vec2 tex;
layout(location = 0) out vec2 color;

void main()
{
  vec2 st = mix(src_rect.xy, src_rect.zw, tex);
  float x = max(0.0, st.x * src_width - 0.5);
  float pair_x = floor(x * 0.5);
  vec4 yuyv = texture(image, vec2((pair_x + 0.5) / (src_width * 0.5), st.y));
  color = yuyv.ga;
}
)glsl";

      // CRT variants of the luma shaders: scanlines follow the *source* lines
      // (recovered from the sample coordinate) and an aperture grille darkens
      // one output column per pitch period. Both modulate luma above video
      // black so black bars/letterboxing stay untouched. highp is required:
      // fract() of source-line coordinates up to ~1080 is meaningless in fp16.
      //
      // crt_params:  x = scanline strength (pre-faded on the CPU against the
      //              vertical scale factor), y = beam pow() exponent,
      //              z = grille strength, w = brightness gain.
      // crt_params2: x = horizontal softness, y = grille pitch in pixels.
      static constexpr std::string_view nv12_y_crt_fragment = R"glsl(
#version 300 es

#ifdef GL_ES
precision highp float;
#endif

uniform sampler2D image;
uniform vec4 src_rect;
uniform vec4 crt_params;
uniform vec2 crt_params2;

in vec2 tex;
layout(location = 0) out float color;

void main()
{
  vec2 st = mix(src_rect.xy, src_rect.zw, tex);
  vec2 texel = 1.0 / vec2(textureSize(image, 0));
  float luma = texture(image, st).r;
  float side = 0.5 * (texture(image, st - vec2(texel.x, 0.0)).r + texture(image, st + vec2(texel.x, 0.0)).r);
  luma = mix(luma, side, crt_params2.x * 0.5);
  float src_line = st.y / texel.y;
  float beam = 0.5 + 0.5 * cos(6.28318530718 * (fract(src_line) - 0.5));
  beam = pow(beam, crt_params.y);
  float scan = 1.0 - crt_params.x * (1.0 - beam);
  float grille = 1.0 - crt_params.z * step(crt_params2.y - 1.0, mod(gl_FragCoord.x, crt_params2.y));
  const float black = 16.0 / 255.0;
  color = black + max(luma - black, 0.0) * scan * grille * (1.0 + crt_params.w);
}
)glsl";

      static constexpr std::string_view yuyv_y_crt_fragment = R"glsl(
#version 300 es

#ifdef GL_ES
precision highp float;
#endif

uniform sampler2D image;
uniform vec4 src_rect;
uniform float src_width;
uniform vec4 crt_params;
uniform vec2 crt_params2;

in vec2 tex;
layout(location = 0) out float color;

float luma_at(float x, float y)
{
  float pair_x = floor(x * 0.5);
  vec4 yuyv = texture(image, vec2((pair_x + 0.5) / (src_width * 0.5), y));
  return mod(floor(x + 0.5), 2.0) < 1.0 ? yuyv.r : yuyv.b;
}

void main()
{
  vec2 st = mix(src_rect.xy, src_rect.zw, tex);
  float x = max(0.0, st.x * src_width - 0.5);
  float luma = luma_at(x, st.y);
  float side = 0.5 * (luma_at(max(x - 1.0, 0.0), st.y) + luma_at(min(x + 1.0, src_width - 1.0), st.y));
  luma = mix(luma, side, crt_params2.x * 0.5);
  float src_line = st.y * float(textureSize(image, 0).y);
  float beam = 0.5 + 0.5 * cos(6.28318530718 * (fract(src_line) - 0.5));
  beam = pow(beam, crt_params.y);
  float scan = 1.0 - crt_params.x * (1.0 - beam);
  float grille = 1.0 - crt_params.z * step(crt_params2.y - 1.0, mod(gl_FragCoord.x, crt_params2.y));
  const float black = 16.0 / 255.0;
  color = black + max(luma - black, 0.0) * scan * grille * (1.0 + crt_params.w);
}
)glsl";

      auto nv12 = make_direct_program(nv12_fragment);
      if (nv12.has_right()) {
        BOOST_LOG(error) << "RKMPP direct V4L2: NV12 GPU shader failed: "sv << nv12.right();
        return false;
      }
      direct_v4l2_program[0].program = std::move(nv12.left());

      auto yuyv_y = make_direct_program(yuyv_y_fragment);
      if (yuyv_y.has_right()) {
        BOOST_LOG(error) << "RKMPP direct V4L2: YUYV luma GPU shader failed: "sv << yuyv_y.right();
        return false;
      }
      direct_v4l2_program[1].program = std::move(yuyv_y.left());

      auto yuyv_uv = make_direct_program(yuyv_uv_fragment);
      if (yuyv_uv.has_right()) {
        BOOST_LOG(error) << "RKMPP direct V4L2: YUYV chroma GPU shader failed: "sv << yuyv_uv.right();
        return false;
      }
      direct_v4l2_program[2].program = std::move(yuyv_uv.left());

      // The CRT programs are optional: a failure logs and drops back to the
      // plain shaders rather than losing the whole GPU path.
      if (shader_effect == shader_effect_e::crt_basic) {
        auto nv12_y_crt = make_direct_program(nv12_y_crt_fragment);
        auto yuyv_y_crt = make_direct_program(yuyv_y_crt_fragment);
        if (nv12_y_crt.has_right() || yuyv_y_crt.has_right()) {
          BOOST_LOG(error) << "RKMPP direct V4L2: CRT shader failed; effect disabled: "sv
                           << (nv12_y_crt.has_right() ? nv12_y_crt.right() : yuyv_y_crt.right());
          shader_effect = shader_effect_e::disabled;
        } else {
          direct_v4l2_program[3].program = std::move(nv12_y_crt.left());
          direct_v4l2_program[4].program = std::move(yuyv_y_crt.left());
          crt_programs_ready = true;
        }
      }

      for (auto &entry : direct_v4l2_program) {
        auto handle = entry.program.handle();
        if (handle == std::numeric_limits<GLuint>::max()) {
          continue;
        }
        entry.image_loc = gl::ctx.GetUniformLocation(handle, "image");
        entry.rect_loc = gl::ctx.GetUniformLocation(handle, "src_rect");
        entry.width_loc = gl::ctx.GetUniformLocation(handle, "src_width");
        entry.crt_loc = gl::ctx.GetUniformLocation(handle, "crt_params");
        entry.crt2_loc = gl::ctx.GetUniformLocation(handle, "crt_params2");
      }

      direct_v4l2_tex = gl::tex_t::make(2);
      direct_gpu_ready = true;
      pipeline_enabled = parse_pipeline_enabled(std::getenv("SUNSHINE_RKMPP_PIPELINE"));
      gpu_render_pending = false;
      BOOST_LOG(info) << "RKMPP direct V4L2: GPU readback pipelining "sv
                      << (pipeline_enabled ? "enabled (+1 frame latency)"sv : "disabled"sv);
      gl_drain_errors;
      return true;
    }

    void set_direct_sampler(direct_program_t &program, int texture_index, const scale_region_t &region, const direct_frame_t &src, bool uv_plane) {
      gl::ctx.UseProgram(program.program.handle());

      if (program.image_loc >= 0) {
        gl::ctx.Uniform1i(program.image_loc, texture_index);
      }

      if (program.rect_loc >= 0) {
        float src_w = (float) src.width;
        float src_h = (float) src.height;
        float left = region.src_left / src_w;
        float top = region.src_top / src_h;
        float right = (region.src_left + region.src_w) / src_w;
        float bottom = (region.src_top + region.src_h) / src_h;
        if (uv_plane && src.nv12) {
          left = (region.src_left / 2) / (src_w / 2.0f);
          top = (region.src_top / 2) / (src_h / 2.0f);
          right = ((region.src_left + region.src_w) / 2) / (src_w / 2.0f);
          bottom = ((region.src_top + region.src_h) / 2) / (src_h / 2.0f);
        }
        gl::ctx.Uniform4f(program.rect_loc, left, top, right, bottom);
      }

      if (program.width_loc >= 0) {
        gl::ctx.Uniform1f(program.width_loc, (float) src.width);
      }

      if (program.crt_loc >= 0) {
        // Fade scanlines out as the vertical scale factor approaches 1:1;
        // below ~2x the line pattern can't resolve cleanly and turns into
        // moiré. The grille is keyed to output pixels, so it never aliases.
        float fade = 1.0f;
        if (crt.fade) {
          float vscale = region.src_h > 0 ? (float) region.out_h / (float) region.src_h : 1.0f;
          fade = std::clamp(vscale - 1.0f, 0.0f, 1.0f);
        }
        // sharp 0..1 maps to a beam pow() exponent of 0.5 (soft, wide lines)
        // through 3.0 (thin, hard lines).
        float beam_exponent = 0.5f + 2.5f * crt.sharp;
        gl::ctx.Uniform4f(program.crt_loc, crt.scan * fade, beam_exponent, crt.mask, crt.bright);
      }

      if (program.crt2_loc >= 0) {
        gl::ctx.Uniform2f(program.crt2_loc, crt.soft, crt.pitch);
      }
    }

    void set_direct_texture_params(bool linear_filter) {
      auto filter = linear_filter ? GL_LINEAR : GL_NEAREST;
      for (auto texture : direct_v4l2_tex) {
        gl::ctx.BindTexture(GL_TEXTURE_2D, texture);
        gl::ctx.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
        gl::ctx.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);
        // The CRT softness taps sample one texel past the edge; the ES default
        // wrap is GL_REPEAT, which would bleed the opposite border in.
        gl::ctx.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        gl::ctx.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
      }
      gl::ctx.BindTexture(GL_TEXTURE_2D, 0);
    }

    bool direct_texture_shape_changed(const direct_frame_t &src) const {
      return direct_tex_width != src.width || direct_tex_height != src.height || direct_tex_fourcc != src.fourcc;
    }

    bool upload_direct_frame(const direct_frame_t &src, bool linear_filter) {
      if (!src.data || (!src.nv12 && src.fourcc != V4L2_PIX_FMT_YUYV)) {
        return false;
      }

      const bool allocate = direct_texture_shape_changed(src);
      if (allocate) {
        direct_tex_width = src.width;
        direct_tex_height = src.height;
        direct_tex_fourcc = src.fourcc;
      }
      set_direct_texture_params(linear_filter && src.nv12);

      gl::ctx.PixelStorei(GL_UNPACK_ALIGNMENT, 1);

      if (src.nv12) {
        gl::ctx.ActiveTexture(GL_TEXTURE0);
        gl::ctx.BindTexture(GL_TEXTURE_2D, direct_v4l2_tex[0]);
        gl::ctx.PixelStorei(GL_UNPACK_ROW_LENGTH, src.stride);
        if (allocate) {
          gl::ctx.TexImage2D(GL_TEXTURE_2D, 0, GL_R8, src.width, src.height, 0, GL_RED, GL_UNSIGNED_BYTE, src.data);
        } else {
          gl::ctx.TexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, src.width, src.height, GL_RED, GL_UNSIGNED_BYTE, src.data);
        }

        gl::ctx.ActiveTexture(GL_TEXTURE1);
        gl::ctx.BindTexture(GL_TEXTURE_2D, direct_v4l2_tex[1]);
        gl::ctx.PixelStorei(GL_UNPACK_ROW_LENGTH, src.stride / 2);
        if (allocate) {
          gl::ctx.TexImage2D(GL_TEXTURE_2D, 0, GL_RG8, src.width / 2, src.height / 2, 0, GL_RG, GL_UNSIGNED_BYTE, src.data + (std::size_t) src.stride * src.height);
        } else {
          gl::ctx.TexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, src.width / 2, src.height / 2, GL_RG, GL_UNSIGNED_BYTE, src.data + (std::size_t) src.stride * src.height);
        }
      } else {
        gl::ctx.ActiveTexture(GL_TEXTURE0);
        gl::ctx.BindTexture(GL_TEXTURE_2D, direct_v4l2_tex[0]);
        gl::ctx.PixelStorei(GL_UNPACK_ROW_LENGTH, src.stride / 4);
        if (allocate) {
          gl::ctx.TexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, src.width / 2, src.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, src.data);
        } else {
          gl::ctx.TexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, src.width / 2, src.height, GL_RGBA, GL_UNSIGNED_BYTE, src.data);
        }
      }

      gl::ctx.PixelStorei(GL_UNPACK_ROW_LENGTH, 0);
      gl::ctx.BindTexture(GL_TEXTURE_2D, 0);
      return true;
    }

    // Upload the captured frame and draw the scaled/letterboxed result into the
    // NV12 render target, then flush so the GPU starts immediately. The result
    // is left in nv12->tex for readback_direct_v4l2_gpu(). Splitting render from
    // readback lets the pipelined path overlap this GPU work with the VPU encode
    // of the previous frame (see convert()).
    bool render_direct_v4l2_gpu(const direct_frame_t &src, const scale_region_t &region) {
      const bool linear_filter = false;
      if (!upload_direct_frame(src, linear_filter)) {
        return false;
      }

      // Fuse the black-border clear with the scaled draw in a single render
      // pass per plane. On the Mali (tile-based) GPU this lets the driver use a
      // fast tile clear and resolve each framebuffer once; clearing in a
      // separate pass first would force the cleared contents to be reloaded
      // into tile memory before the draw.
      const float y_black[] = {16.0f / 255.0f, 0.0f, 0.0f, 0.0f};
      const float uv_black[] = {128.0f / 255.0f, 128.0f / 255.0f, 0.0f, 0.0f};

      // CRT (or any future effect) applies to the luma plane only; chroma
      // keeps the plain scale shader.
      const bool crt = crt_programs_ready && shader_effect == shader_effect_e::crt_basic;

      if (src.nv12) {
        for (int plane = 0; plane < 2; ++plane) {
          auto &program = direct_v4l2_program[(plane == 0 && crt) ? 3 : 0];
          gl::ctx.ActiveTexture(plane == 0 ? GL_TEXTURE0 : GL_TEXTURE1);
          gl::ctx.BindTexture(GL_TEXTURE_2D, direct_v4l2_tex[plane]);
          set_direct_sampler(program, plane, region, src, plane == 1);
          gl::ctx.BindFramebuffer(GL_FRAMEBUFFER, nv12->buf[plane]);
          gl::ctx.ClearBufferfv(GL_COLOR, 0, plane == 0 ? y_black : uv_black);
          gl::ctx.Viewport(region.off_x / (plane + 1), region.off_y / (plane + 1), region.out_w / (plane + 1), region.out_h / (plane + 1));
          gl::ctx.DrawArrays(GL_TRIANGLES, 0, 3);
        }
      } else if (src.fourcc == V4L2_PIX_FMT_YUYV) {
        gl::ctx.ActiveTexture(GL_TEXTURE0);
        gl::ctx.BindTexture(GL_TEXTURE_2D, direct_v4l2_tex[0]);

        set_direct_sampler(direct_v4l2_program[crt ? 4 : 1], 0, region, src, false);
        gl::ctx.BindFramebuffer(GL_FRAMEBUFFER, nv12->buf[0]);
        gl::ctx.ClearBufferfv(GL_COLOR, 0, y_black);
        gl::ctx.Viewport(region.off_x, region.off_y, region.out_w, region.out_h);
        gl::ctx.DrawArrays(GL_TRIANGLES, 0, 3);

        set_direct_sampler(direct_v4l2_program[2], 0, region, src, true);
        gl::ctx.BindFramebuffer(GL_FRAMEBUFFER, nv12->buf[1]);
        gl::ctx.ClearBufferfv(GL_COLOR, 0, uv_black);
        gl::ctx.Viewport(region.off_x / 2, region.off_y / 2, region.out_w / 2, region.out_h / 2);
        gl::ctx.DrawArrays(GL_TRIANGLES, 0, 3);
      } else {
        return false;
      }

      gl::ctx.BindFramebuffer(GL_FRAMEBUFFER, 0);
      gl::ctx.BindTexture(GL_TEXTURE_2D, 0);
      // Kick the GPU now so the draws execute while the CPU returns to encode
      // the previous frame; without this the commands could sit in the client
      // command buffer until the next GL call, defeating the overlap.
      gl::ctx.Flush();
      gl_drain_errors;
      return true;
    }

    // Read the rendered NV12 planes from nv12->tex into the encoder's mapped
    // DMA buffer. In the pipelined path the render was flushed a full encode
    // cycle earlier, so the GPU is already done and this does not stall.
    void readback_direct_v4l2_gpu(AVFrame *mapped_frame) {
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
      gl::ctx.BindTexture(GL_TEXTURE_2D, 0);
      gl_drain_errors;
    }

    // Synchronous render + immediate readback (no pipelining): used when
    // SUNSHINE_RKMPP_PIPELINE is disabled.
    bool convert_direct_v4l2_gpu(const direct_frame_t &src, const scale_region_t &region, AVFrame *mapped_frame) {
      if (!render_direct_v4l2_gpu(src, region)) {
        return false;
      }
      readback_direct_v4l2_gpu(mapped_frame);
      return true;
    }

    void make_current() {
      // The capture path may leave its own context current on this thread, but
      // for the direct V4L2 path it usually doesn't, so skip the redundant
      // driver call when our context is already current.
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
}  // namespace rkmpp
