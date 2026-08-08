/**
 * @file src/platform/linux/rkmpp.cpp
 * @brief Rockchip MPP (RKMPP) encode device.
 *
 * Two capture paths feed the RKMPP encoder:
 *
 *  - retro-capture socket client (the console path): a standalone daemon
 *    (~/retro-capture) owns the HDMI-RX V4L2 device, absorbs every input
 *    transition and driver wedge internally, and delivers NV12 frames at the
 *    stream geometry as dma-buf fds over a SEQPACKET Unix socket. The
 *    protocol session lives in the vendored MIT client library
 *    (third-party/retro-capture, mirrored verbatim from the retro-capture
 *    repo, which owns the protocol spec in its PROTOCOL.md). This file wraps
 *    the received pool buffers as DRM_PRIME AVFrames; zero-copy end to end.
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
#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <optional>
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
#include "src/config.h"  // min_max_avg_periodic_logger's enable test reads config::sunshine
#include "graphics.h"
#include "misc.h"
// The vendored MIT retro-capture clients (third-party/retro-capture); the raw
// protocol header is still needed by the RCAP_FRAME_* flag policy below.
#include "retro-capture-client.h"
#include "retro-overlay-client.h"
#include "third-party/retro-capture/include/retro-capture-protocol.h"
#include "rkmpp.h"
#include "src/logging.h"
#include "src/platform/common.h"
#include "src/utility.h"
#include "src/video.h"

using namespace std::literals;

namespace rkmpp {

  // One-time bridge from the MIT clients' logger callbacks into BOOST_LOG.
  static void client_log(retro::capture::log_level level, const char *message) {
    if (level == retro::capture::log_warning) {
      BOOST_LOG(warning) << message;
    } else {
      BOOST_LOG(info) << message;
    }
  }

  static void install_capture_logger() {
    static std::once_flag once;
    std::call_once(once, [] {
      retro::capture::set_logger(client_log);
      retro::overlay::set_logger(client_log);
    });
  }

  // Encoder OSD: hand the overlay client's state to the RKMPP encoder as
  // frame side data. This is the consumer-side glue the client deliberately
  // leaves out — it is the only FFmpeg-shaped part of the overlay path.
  static void attach_osd(AVFrame *frame, int width, int height, bool ten_bit) {
    retro::overlay::report_encode_geometry(width, height, ten_bit);

    // The wrapper frames are long-lived and never unref'd between encodes:
    // stale side data must go, visible or not.
    av_frame_remove_side_data(frame, AV_FRAME_DATA_RKMPP_OSD);

    rovl_osd_side_data osd {};
    if (!retro::overlay::current_osd(osd)) {
      return;
    }
    auto *sd = av_frame_new_side_data(frame, AV_FRAME_DATA_RKMPP_OSD, sizeof(osd));
    if (!sd) {
      return;
    }
    std::memcpy(sd->data, &osd, sizeof(osd));
  }

  // Sunshine's frame source on top of the vendored retro-capture client
  // (third-party/retro-capture, MIT). The protocol session — connect/SETUP,
  // pool import, the newest-frame-wins drain and its lease bookkeeping —
  // lives in the library; this adapter keeps the two pieces that are
  // deliberately NOT in it (see the library header's "DELIBERATELY NO"
  // notes): wrapping pool buffers as DRM_PRIME AVFrames for the RKMPP
  // encoder, and the audio-gate policy that decides what a HELD/BLACK run
  // means for outgoing audio.
  class frame_source_t {
  public:
    ~frame_source_t() {
      for (auto &w : wrappers) {
        if (w) {
          av_frame_free(&w);
        }
      }
      if (hw_frames_ctx) {
        av_buffer_unref(&hw_frames_ctx);
      }
      // The client closes the pool fds it owns; wrappers must go first.
    }

    // Connect + import the pool. Returns nullptr on any failure (caller
    // falls back to the KMS path).
    static std::unique_ptr<frame_source_t> connect(int width, int height, AVBufferRef *hw_frames_ctx_buf) {
      install_capture_logger();

      // The session's bit depth decides the pool format we request: the
      // encoder was created for the frames context's sw_format, and an
      // 8-bit session must get NV12 even while the daemon captures 10-bit
      // (the daemon downconverts).
      bool want_nv15 = false;
      if (hw_frames_ctx_buf) {
        auto *fctx = (AVHWFramesContext *) hw_frames_ctx_buf->data;
        want_nv15 = fctx->sw_format == AV_PIX_FMT_NV15;
      }
      const std::uint32_t fourcc = want_nv15 ? 0x3531564e /* DRM_FORMAT_NV15 */ : 0x3231564e /* NV12 */;

      auto client = retro::capture::client::connect(width, height, fourcc, "sunshine");
      if (!client) {
        return nullptr;
      }

      auto src = std::make_unique<frame_source_t>();
      src->client = std::move(client);
      src->want_fourcc = fourcc;
      src->hw_frames_ctx = av_buffer_ref(hw_frames_ctx_buf);
      src->wrappers.assign(src->client->pool_size(), nullptr);
      // Per-FRAME telemetry, for the one field this path needs: the instant
      // the picture was actually taken. `src` is never moved after this (it
      // lives in the unique_ptr the caller stores), so the pointer is stable.
      src->client->set_frame_observer(&frame_source_t::observe, src.get());
      return src;
    }

    // The instant the HDMI-RX frame we are about to encode was DQBUF'd by the
    // daemon, or nullopt before the first timestamped frame.
    //
    // THE point of measurement for this pipeline. Sunshine's own "Frame
    // processing latency" is measured from `img->frame_timestamp`, which the
    // kmsgrab capture thread stamps when its METRONOME TICKED — on the daemon
    // path that tick is unrelated to when the picture was taken, so the metric
    // it produces moves with the free-running phase between two clocks and
    // reports dead time as if it were pipeline cost. Stamping the frame's own
    // DQBUF instant instead turns the same metric into a true capture->encoded
    // age: the number a game stream is actually judged on, and the one that
    // cannot be gamed by choosing when to look.
    //
    // dqbuf_ns is CLOCK_MONOTONIC, which is precisely what steady_clock is on
    // glibc — same epoch, no conversion.
    //
    // FRESH ONLY, deliberately. A HELD re-emission carries the ORIGINAL
    // frame's dqbuf_ns (PROTOCOL.md 3.4), so through a signal-loss run its
    // "age" climbs without bound — seconds — and that is a true statement
    // about the picture but a false one about the pipeline. Reported as
    // latency it would say the encoder stalled for four seconds when the
    // encoder did nothing wrong; the stream suite would raise
    // encode_latency_spike at its 100 ms threshold, and Moonlight's overlay
    // would show the same nonsense. Held runs are already measured, as held
    // frames. So a re-emission keeps the metronome's stamp and this metric
    // stays what it claims to be: the age of NEW content.
    std::optional<std::chrono::steady_clock::time_point> capture_time() const {
      if (last_dqbuf_ns == 0 || !(last_flags & RCAP_FRAME_FRESH)) {
        return std::nullopt;
      }
      return std::chrono::steady_clock::time_point(
        std::chrono::nanoseconds(last_dqbuf_ns));
    }

    // Drain the socket via the client and return the DRM_PRIME wrapper for
    // the current buffer. Newest-frame-wins: identical latency semantics to
    // the old in-process update_latest_frame(). Returns nullptr only before
    // the first frame or after a wrapper allocation failure.
    AVFrame *next_frame() {
      const auto t_enter = std::chrono::steady_clock::now();
      auto f = client->next();

      // Nothing FRESH in this drain means one of two things, and both used to
      // end the same way: we polled ahead of the daemon and it has not sent
      // this slot yet, or the RX silently skipped a frame and the daemon's
      // cadence watchdog re-emitted the last one. Either way, encoding what
      // we already hold FABRICATES a duplicate picture, and the next drain
      // then finds two queued and drops one — the N,N,N+2 judder the stream
      // suite measures at 0.05% on 1080p60 and 0.47% on 4K60 (tests/stream
      // FINDING 15). Wait for the frame that is already on its way instead of
      // inventing one; a skip tells the truth about a lost frame, a duplicate
      // does not.
      //
      // Safe to block: the daemon re-arms its cadence watchdog on every send,
      // so in every state SOME frame lands well inside the cap. And we only
      // wait while the hold is BRIEF — a sustained hold is real signal loss,
      // where the re-emission IS the correct picture and stalling the encoder
      // would only starve the stream. That bounds the cost of a dead source
      // to `max_wait_run` frames.
      // The cap is a FRACTION OF THE FRAME PERIOD, never a fixed duration.
      // This is a game stream: 20 ms is 1.2 periods at 60 Hz and nearly five
      // at 240 Hz, so a constant that looks harmless at one rate is dead time
      // at another. Half a period is the default — enough to collect a frame
      // that is already in flight (measured cost at 4K60: 0.15 ms of the
      // 15.5 ms budget), while bounding what a genuinely late frame can add
      // to content-to-glass latency at HALF A FRAME, at any refresh rate.
      //
      // The period is measured from the stream itself rather than plumbed in,
      // so it needs no call-site change and follows a mid-session mode switch.
      // 1.5 periods, matching the window retro-capture's own cadence watchdog
      // uses before it decides the source has stalled: waiting less than the
      // daemon waits means both ends give up on the same tick and the held
      // re-emission wins. Measured at 4K60 — 1.5 gives 0 duplicates, 0.5 gives
      // 9, because the daemon's loop stretches to 20-58 ms under encoder
      // contention. Still period-relative, so it is 25 ms at 60 Hz, 12.5 ms at
      // 120 Hz and 6.25 ms at 240 Hz rather than a constant that only suits 60.
      static const double wait_fraction = [] {
        const char *env = std::getenv("SUNSHINE_RETRO_FRESH_WAIT_FRACTION");
        const double v = env && *env ? std::atof(env) : 1.5;
        return (v > 0.0 && v <= 2.0) ? v : 1.5;
      }();
      const int wait_ms = period_ms_ewma > 0.0 ?
                            (int) (period_ms_ewma * wait_fraction) : 0;
      constexpr int max_wait_run = 3;
      if (wait_ms > 0 && !(f.flags & RCAP_FRAME_FRESH) &&
          nonfresh_run < max_wait_run && client->alive()) {
        const int cfd = client->fd();
        if (cfd >= 0) {
          // Keep waiting until something FRESH lands, not merely until the
          // socket is readable: the daemon's watchdog re-emission arrives
          // first in the missing-frame case, and accepting it is exactly the
          // duplicate we came here to avoid. Each held frame seen on the way
          // is still folded into `f` so the audio gate below observes the run.
          const auto deadline = std::chrono::steady_clock::now() +
                                std::chrono::milliseconds(wait_ms);
          while (!(f.flags & RCAP_FRAME_FRESH)) {
            const auto now = std::chrono::steady_clock::now();
            if (now >= deadline) {
              break;
            }
            const auto left =
              std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
            pollfd pfd {cfd, POLLIN, 0};
            if (poll(&pfd, 1, left > 0 ? (int) left : 1) <= 0) {
              break;
            }
            const auto g = client->next();
            if (g.flags != 0) {
              f = g;  // a real message; an empty re-drain must not clear f
            }
          }
        }
      }
      nonfresh_run = (f.flags & RCAP_FRAME_FRESH) ? 0 : nonfresh_run + 1;

      // What the wait COST and what it BOUGHT, measured separately, because
      // the two answer different questions and only one of them is latency.
      //
      // `fresh-frame wait` is dead time this call spent blocked. It is NOT
      // added end-to-end latency — the metronome would otherwise have slept
      // the same interval before the next tick — but it is the honest size of
      // the phase error between Sunshine's timer and the source's cadence, and
      // driving it toward zero is what makes the wait a genuine no-op instead
      // of a trade against duplicates.
      //
      // `frame age at encode` is the real number: how stale the picture is at
      // the instant the encoder receives it. Invariant to metronome phase,
      // which is exactly why it is trustworthy where "Frame processing
      // latency" was not.
      const auto t_have = std::chrono::steady_clock::now();
      wait_logger.collect_and_log(
        std::chrono::duration<double, std::milli>(t_have - t_enter).count());
      if (const auto taken = capture_time()) {
        age_logger.collect_and_log(
          std::chrono::duration<double, std::milli>(t_have - *taken).count());
      }

      // Cadence floor for re-emissions — AFTER the measurements above, so this
      // deliberate pacing never shows up as acquisition cost.
      //
      // The capture thread oversamples its tick (kmsgrab, daemon path) so a
      // published frame is collected promptly; while frames flow, the FRESH
      // wait turns those extra ticks into source pacing and the encoder still
      // runs at source rate. But a DEAD source has no FRESH frames to pace
      // against: the wait gives up after max_wait_run, and every remaining
      // tick would then re-encode the held picture at the OVERSAMPLED rate —
      // duplicates several times faster than the stream's own frame rate,
      // flooding encoder and network at exactly the moment there is nothing to
      // send. Hold a re-emission to one source period so a dead source costs
      // what it did before oversampling: the held cadence, and nothing more.
      if (!(f.flags & RCAP_FRAME_FRESH) && period_ms_ewma > 0.0 &&
          last_returned.time_since_epoch().count() != 0) {
        const auto since =
          std::chrono::duration<double, std::milli>(t_have - last_returned).count();
        if (since < period_ms_ewma) {
          std::this_thread::sleep_for(
            std::chrono::duration<double, std::milli>(period_ms_ewma - since));
        }
      }
      last_returned = std::chrono::steady_clock::now();

      // Measure the source's frame period from FRESH arrivals. EWMA over a
      // long window so one late frame cannot widen the very cap that exists
      // to bound lateness; seeded on the first interval so the wait is armed
      // within two frames of session start. Intervals outside [1 ms, 100 ms]
      // are ignored — those are a stall or a mode change, not a period.
      if (f.flags & RCAP_FRAME_FRESH) {
        const auto now = std::chrono::steady_clock::now();
        if (last_fresh.time_since_epoch().count() != 0) {
          const double ms =
            std::chrono::duration<double, std::milli>(now - last_fresh).count();
          if (ms >= 1.0 && ms <= 100.0) {
            period_ms_ewma = period_ms_ewma > 0.0 ?
                               (period_ms_ewma * 0.99 + ms * 0.01) : ms;
          }
        }
        last_fresh = now;
      }

      // f.flags is non-zero iff at least one FRAME message arrived in this
      // drain (every daemon frame carries FRESH, HELD or BLACK), which is
      // when the inline client used to run this policy.
      if (f.flags != 0) {
        // In-process audio squelch across video transitions: the daemon
        // notifies retro-audio via its FIFO at the precise ladder points; the
        // protocol's flag transitions give this process the same signal for
        // its own outgoing-audio gate (replaces the triggers that lived in
        // the migrated capture code).
        // NOT every held frame is a renegotiation. Measured on a healthy,
        // locked 4K60 link (2026-08-06): the HDMI-RX silently fails to
        // deliver a frame 5-9 times a minute, which reaches us as exactly ONE
        // HELD re-emission — the capture daemon itself loses nothing
        // (capture.json: drain_dropped 0, driver sequence continuous,
        // DQBUF->send p99 9.5 ms). Arming a 500 ms squelch on the way into
        // AND back out of each of those was the "audio cutout every ~6 s":
        // up to a full second of muted audio per single dropped frame.
        //
        // The gate exists for the RX audio FIFO re-init click, which only
        // accompanies a real renegotiation: signal loss, the pre-first-frame
        // BLACK prime, or a held run that outlasts any single dropped frame
        // by an order of magnitude. Squelch those, ignore the one-frame
        // blips. (The daemon independently notifies retro-audio over its FIFO
        // at the precise ladder points, so this is belt-and-braces, not the
        // only protection.)
        static const auto hold_ms = [] {
          const char *env = std::getenv("SUNSHINE_AUDIO_GATE_HOLD_MS");
          const int ms = env && *env ? std::atoi(env) : 120;
          return std::chrono::milliseconds(ms > 0 ? ms : 120);
        }();
        const bool held_now = (f.flags & (RCAP_FRAME_HELD | RCAP_FRAME_BLACK)) != 0;
        const bool reneg_now = (f.flags & (RCAP_FRAME_BLACK | RCAP_FRAME_SIGNAL_LOST)) != 0;
        const auto now = std::chrono::steady_clock::now();
        if (!held_now) {
          held_since = {};
        } else if (held_since.time_since_epoch().count() == 0) {
          held_since = now;
        }
        const bool sustained = held_now &&
                               held_since.time_since_epoch().count() != 0 &&
                               now - held_since >= hold_ms;
        const bool gate_worthy = reneg_now || sustained;
        if (gate_worthy != last_was_held) {
          audio_gate::trigger(audio_gate::reneg_ms.load(std::memory_order_relaxed), false);
          last_was_held = gate_worthy;
        }
      }

      return f.index >= 0 ? wrapper_for(f.index) : nullptr;
    }

    // Non-blocking reconnect pacing after daemon death (the client rate
    // limits to one attempt a second): systemd restarts the daemon within
    // seconds; until then the encoder keeps re-sending the last received
    // buffer (the fds outlive the daemon process).
    void maybe_reconnect(int width, int height) {
      const bool was_alive = client->alive();
      client->maybe_reconnect(width, height, want_fourcc, "sunshine");
      if (!was_alive && client->alive()) {
        // A new session means a new pool and stream layout: drop the wrapper
        // cache so it is rebuilt against the fresh fds. (The inline client's
        // move-assign reconnect leaked the old wrappers and double-closed the
        // new socket; the library's impl swap plus this explicit reset is the
        // corrected shape.)
        for (auto &w : wrappers) {
          if (w) {
            av_frame_free(&w);
          }
        }
        wrappers.assign(client->pool_size(), nullptr);
      }
    }

  private:
    // DRM_PRIME AVFrame around pool buffer `index` (built once, reused every
    // frame — same descriptor shape the old enc_frame() produced).
    AVFrame *wrapper_for(int index) {
      if (wrappers[index]) {
        return wrappers[index];
      }
      const auto &sinfo = client->info();
      auto *desc = (AVDRMFrameDescriptor *) av_mallocz(sizeof(AVDRMFrameDescriptor));
      if (!desc) {
        return nullptr;
      }
      desc->nb_objects = 1;
      desc->objects[0].fd = client->pool_fd(index);
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

    // Per-FRAME telemetry hook. Records the DQBUF instant of the most recent
    // FRAME message in the drain which — arrival order, newest-frame-wins — is
    // the frame next_frame() hands the encoder. HELD re-emissions deliberately
    // carry the ORIGINAL frame's dqbuf_ns (PROTOCOL.md 3.4), so a held picture
    // reports its true age instead of masquerading as new.
    static void observe(const retro::capture::frame_telemetry &t, void *user) {
      auto *self = static_cast<frame_source_t *>(user);
      self->last_dqbuf_ns = t.dqbuf_ns;
      self->last_flags = t.flags;
    }

    std::unique_ptr<retro::capture::client> client;
    std::vector<AVFrame *> wrappers;
    std::uint64_t last_dqbuf_ns {};
    std::uint32_t last_flags {};
    // Debug-severity 20 s periodic reports, same "(min/max/avg): a/b/c" shape
    // as Sunshine's own latency loggers — which is what the stream suite's
    // collector already scrapes, so these need no collector change to appear
    // in a run report.
    logging::min_max_avg_periodic_logger<double> age_logger {debug, "Capture: frame age at encode", "ms"};
    logging::min_max_avg_periodic_logger<double> wait_logger {debug, "Capture: fresh-frame wait", "ms"};
    std::uint32_t want_fourcc {};
    bool last_was_held {true};  // stream primes with BLACK
    // Consecutive drains that produced no FRESH frame. Bounds how long
    // next_frame() will wait for real content before accepting the daemon's
    // re-emission: brief runs are a missing frame we refuse to fabricate,
    // long ones are signal loss we must keep streaming through.
    int nonfresh_run {};
    // Measured source frame period, and the last FRESH arrival it came from.
    // Drives the wait cap so it scales with refresh rate instead of assuming
    // 60 Hz; zero until the first interval is seen, which disables waiting.
    double period_ms_ewma {};
    std::chrono::steady_clock::time_point last_fresh {};
    // Last time next_frame() handed a picture back, for the re-emission
    // cadence floor above. Zero until the first frame.
    std::chrono::steady_clock::time_point last_returned {};
    // Start of the current unbroken held run, for the renegotiation test
    // above; zero when fresh content is flowing.
    std::chrono::steady_clock::time_point held_since {};
    AVBufferRef *hw_frames_ctx {};
  };

  // Set while an encode session is sourcing frames from the daemon socket;
  // read by kmsgrab from its capture thread. Namespace-scope because the
  // reader has no handle on the encode device.
  static std::atomic<bool> g_daemon_capture_active {false};

  class rkmpp_t: public platf::avcodec_encode_device_t {
  public:
    ~rkmpp_t() override {
      if (capture) {
        g_daemon_capture_active.store(false, std::memory_order_relaxed);
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
        capture = frame_source_t::connect(frame->width, frame->height, hw_frames_ctx_buf);
        if (capture) {
          BOOST_LOG(info) << "Using retro-capture daemon encode path (zero-copy DRM_PRIME)"sv;
          // Tells kmsgrab its framebuffer export is dead work for this
          // session (see daemon_capture_active).
          g_daemon_capture_active.store(true, std::memory_order_relaxed);
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
        // Re-stamp the capture time with the frame's own DQBUF instant. The
        // kmsgrab capture thread stamped this when its metronome ticked, which
        // on the daemon path is a tick number, not a picture — see
        // frame_source_t::capture_time(). videoThread reads frame_timestamp
        // AFTER convert() for exactly this handoff, so "Frame processing
        // latency" (host_lat on the client, and the RTP header extension
        // Moonlight reports) becomes true capture->encoded age.
        if (const auto taken = capture->capture_time()) {
          img.frame_timestamp = *taken;
        }
        this->frame = enc;
        // Encoder OSD (stream overlay): attach/remove side data on the
        // wrapper. No pixel work; hidden overlay attaches nothing.
        auto *desc = (AVDRMFrameDescriptor *) enc->data[0];
        attach_osd(enc, enc->width, enc->height,
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
    std::unique_ptr<frame_source_t> capture;

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

    // PROTOCOL.md 3.10: input.bit_depth is 10 iff the capture is NV15, which
    // the daemon defines as BT.2020+PQ content (the MVP contract).
    install_capture_logger();
    cached = retro::capture::query_status("sunshine-hdr").input_bit_depth == 10;
    return cached;
  }

  bool daemon_capture_active() {
    return g_daemon_capture_active.load(std::memory_order_relaxed);
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
