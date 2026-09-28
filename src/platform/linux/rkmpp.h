/**
 * @file src/platform/linux/rkmpp.h
 * @brief Declarations for Rockchip MPP (RKMPP) zero-copy encode device.
 */
#pragma once

// standard includes
#include <cstdint>
#include <memory>

// local includes
#include "misc.h"
#include "src/platform/common.h"

extern "C" {
#include <libavutil/rational.h>
}

extern "C" struct AVBufferRef;

namespace rkmpp {
  /**
   * @brief Create the RKMPP FFmpeg hardware device context.
   *
   * Stored as `avcodec_encode_device_t::data` so that video.cpp can build the
   * base hwdevice context for the encoder. Mirrors the VAAPI integration.
   *
   * @param base The encode device requesting the context (unused).
   * @param hw_device_buf Output parameter receiving the new device context.
   * @return 0 on success, negative on failure.
   */
  int rkmpp_init_avcodec_hardware_input_buffer(platf::avcodec_encode_device_t *base, AVBufferRef **hw_device_buf);

  /**
   * @brief Create an RKMPP encode device: Mali converts RGB->NV12, read back
   *        into the encoder's DMA-BUF.
   * @param width Captured region width.
   * @param height Captured region height.
   * @param card Render node fd used for GBM/EGL (ownership transferred).
   * @param offset_x Horizontal offset of the captured region.
   * @param offset_y Vertical offset of the captured region.
   * @return The encode device or nullptr on failure.
   */
  std::unique_ptr<platf::avcodec_encode_device_t> make_avcodec_encode_device(int width, int height, file_t &&card, int offset_x, int offset_y);

  /**
   * @brief Whether the retro-capture daemon is currently ingesting 10-bit
   *        (NV15 / BT.2020+PQ) content.
   *
   * On a capture box the KMS connector's HDR state is irrelevant — HDR-ness
   * is decided by what the daemon captures. kmsgrab's is_hdr() consults this
   * when SUNSHINE_RKMPP_V4L2 marks the host as capture-streaming. The result
   * is cached briefly; returns false when the daemon is unreachable.
   */
  bool daemon_hdr_active();

  /**
   * @brief HDR10 metadata for a daemon 10-bit session.
   *
   * The HDMI-RX driver does not parse the source's HDR InfoFrame (MVP
   * scope), so this serves standard HDR10 defaults (BT.2020 primaries, D65,
   * 1000-nit mastering display, MaxCLL 1000 / MaxFALL 400).
   *
   * @param metadata Filled when the daemon reports 10-bit input.
   * @return True when metadata was written.
   */
  bool daemon_hdr_metadata(SS_HDR_METADATA &metadata);

  /**
   * @brief Whether an encode session is currently sourcing frames from the
   *        retro-capture daemon socket.
   *
   * False until a session connects, and false again for a session that fell
   * back to KMS GPU-convert — so a caller that skips work on the strength of
   * this is never skipping work the fallback needs. kmsgrab uses it to avoid
   * re-exporting a KMS framebuffer that the encoder is going to discard.
   */
  bool daemon_capture_active();

  /**
   * @brief Measured interval between FRESH frames from the capture daemon, ms.
   *
   * The SOURCE's cadence, which is not the client's requested rate and must not
   * be confused with it: a client may ask for 120 fps from a 60 fps console, and
   * pacing capture off the request then samples twice as often as there is
   * anything to sample. Zero until the first interval is measured.
   */
  double daemon_source_period_ms();

  /**
   * @brief Whether an encode at this geometry and rate needs the pipelined
   *        (non-LOW_DELAY) encoder to keep up.
   *
   * With AV_CODEC_FLAG_LOW_DELAY, hevc_rkmpp blocks on every frame's packet,
   * so exactly one frame is ever inside MPP and the second VEPU580 core of
   * the RK3588 never gets overlapping work: 4K caps at ~65 fps however fast
   * frames arrive. Without it MPP keeps frames in flight and the two cores
   * overlap (133 fps at 4K measured). The cost is up to one frame period of
   * added delay, so it is used only above the one-core budget — every rate
   * up to 4K60 keeps the blocking encoder unchanged.
   *
   * SUNSHINE_RKMPP_PIPELINE: "auto" (default), "1" force on, "0" force off.
   */
  bool want_pipelined_encode(int width, int height, double fps);

  /// Set by the encoder setup for the session it just built; the capture
  /// path reads it to hold buffer leases while the encoder still reads them.
  void set_pipelined_encode(bool on);

  // ---- source-rate following (capture box) ----
  //
  // A client's requested frame rate says what it can display, not what the
  // console sends: Moonlight asks for 120 while a PS5 sends 59.94. Built for
  // the request, the encoder pipelines four 4K frames for a rate that never
  // arrives (54 ms host latency where 19 ms is possible) and budgets bits for
  // twice the frames it gets. So on the daemon path the encoder is sized for
  // the SOURCE: its rate as the capture spec defines it (CTA-861's N or
  // N*1000/1001), decimated by a whole factor when the client asked for less.
  // The plan follows the source across mode switches without reopening the
  // encoder or cutting the stream (see live_encode_rate()).

  /**
   * @brief How a session feeds the encoder.
   */
  struct encode_rate_t {
    AVRational source {0, 1};  ///< the source's spec rate; 0/1 = unknown
    AVRational rate {0, 1};  ///< exact rate frames reach the encoder; 0/1 = unknown
    int decimation {1};  ///< encode every Nth new source frame (source faster than the request)
  };

  /**
   * @brief The spec rate behind a measured refresh rate.
   *
   * The receiver measures the pixel clock, so a 60000/1001 signal reads as
   * 59.941 and a 60 Hz one as 60.002. Snaps to the nearest CTA-861 / CVT
   * rate — an integer N or its N*1000/1001 twin — when the measurement is
   * within 0.04% of it (the two families sit 0.1% apart); anything else is
   * kept at the measurement's own millihertz resolution.
   *
   * @param fps Measured refresh rate, Hz.
   * @return The exact rate, or 0/1 for a non-positive input.
   */
  AVRational snap_source_rate(double fps);

  /**
   * @brief Plan the encode rate for a requested and a source rate.
   *
   * A source no faster than the request is encoded frame for frame at its
   * own rate. A faster one is decimated by the smallest whole factor that
   * brings it to or under the request (119.88 for a 60 request -> every 2nd
   * frame, 59.94 exactly), never by a fraction, which would judder. A source
   * within 1% above the request counts as equal: the 0.1% between 60 and
   * 59.94 is a clock, not a rate to halve.
   *
   * @param requested The client's requested rate.
   * @param source The source's spec rate, 0/1 if unknown.
   * @return The plan; rate 0/1 when the source is unknown.
   */
  encode_rate_t plan_encode_rate(AVRational requested, AVRational source);

  /**
   * @brief The source's mode right now, from the daemon's STATUS.
   */
  struct source_mode_t {
    AVRational rate {0, 1};  ///< spec rate (snap_source_rate); 0/1 = unknown
    int width {};  ///< input geometry as the receiver locked it; 0 = unknown
    int height {};
  };

  /**
   * @brief The source's mode right now, from the daemon's STATUS.
   *
   * One STATUS-role round trip (sub-millisecond; no consumer slot). Rate 0/1
   * and a zero size when not a capture box, the daemon is absent, or the
   * input is unlocked.
   */
  source_mode_t daemon_source_mode();

  /**
   * @brief The source's spec rate right now (daemon_source_mode().rate).
   */
  AVRational daemon_source_rate();

  /**
   * @brief Start a session's source-rate following.
   *
   * Called by the encoder setup before the capture connects. Resets the
   * live state; the capture path re-plans with `requested` whenever the
   * source re-locks.
   *
   * @param requested The client's requested rate.
   * @param plan The plan the encoder was opened with.
   */
  void begin_encode_rate(AVRational requested, const encode_rate_t &plan);

  /**
   * @brief Live source-rate state for the encode loop, one read per frame.
   */
  struct live_rate_t {
    std::uint64_t generation {};  ///< changes whenever `plan` or the source size does
    encode_rate_t plan;  ///< what the encoder should run at now
    int source_width {};  ///< the source's geometry at the last (re-)lock; 0 = unknown
    int source_height {};
    bool signal_lost {};  ///< the frame about to be encoded is a signal-loss re-emission
    bool fresh {};  ///< the frame about to be encoded is new content
  };

  /**
   * @brief Read the live state (atomics; a mutex only when the plan moved).
   */
  live_rate_t live_encode_rate();

  /**
   * @brief Period of the encoded frame cadence, ms: the source period times
   *        the decimation. Zero until the source rate is known.
   *
   * kmsgrab's capture tick oversamples this rather than the client's rate.
   */
  double daemon_encode_period_ms();

  // ---- source-geometry following (capture box, capture protocol v4) ----
  //
  // The same idea for resolution: the daemon delivers the console's own
  // resolution inside the pool (capped by the client's request, never
  // upscaled), and the encoder follows it frame by frame. Each wrapper frame
  // carries the picture's size in width/height; the pool, its strides and
  // its fds never change. Only for a client that declared it follows a
  // mid-stream size change (video::RS_CAP_DYNAMIC_RESOLUTION): every other
  // client keeps the size it asked for.

  /**
   * @brief Whether the session about to connect should follow the source's
   *        geometry. Set by the encoder setup, read when the capture connects.
   */
  void set_follow_geometry(bool on);

  /**
   * @brief Live gate on geometry following, for a shared (fan-out) stream: a
   *        listener that cannot follow a size change turns it off for as long
   *        as it is attached. Takes effect on the next captured frame.
   */
  void set_follow_geometry_allowed(bool allowed);

  /**
   * @brief Published by the encode loop: whether the encoder currently runs
   *        at the capture pool's full size (so a listener that cannot follow a
   *        size change can join without seeing one).
   */
  void note_encode_at_pool_size(bool at_pool);
  bool encode_at_pool_size();
}  // namespace rkmpp
