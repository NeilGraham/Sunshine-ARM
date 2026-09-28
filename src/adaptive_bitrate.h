/**
 * @file src/adaptive_bitrate.h
 * @brief Network-adaptive bitrate for the capture-box stream.
 *
 * The client's requested bitrate is a CEILING, not an order. Under it:
 *
 *  - the ceiling follows the source's mode: Moonlight picked its default
 *    bitrate for the mode the user asked for, and the ratio between that and
 *    Moonlight's own default for the mode the console actually sends is the
 *    bitrate the same quality needs (a 720p60 game on a 4K120 session gets a
 *    tenth of it, not all of it);
 *
 *  - the target backs off the moment the network starts losing packets and
 *    climbs back once it stops. The first warning every current Moonlight
 *    already sends: a frame that needed FEC to recover (SS_FRAME_FEC_STATUS,
 *    control type 0x5502) is loss the player has NOT seen yet, so backing off
 *    there costs nothing visible. A frame FEC could not recover, and a
 *    keyframe or reference-invalidation request (the client's answer to a
 *    lost frame), back off harder.
 *
 * The goal is zero dropped frames first: backing off is fast (next frame,
 * multiplicative), coming back is slow (after a quiet hold, then in steps).
 * A link that simply is that good never sees a change: the target stays at
 * the ceiling as long as nothing is lost.
 *
 * controller_t is the pure policy (clock injected, no threads) so it can be
 * unit tested; the free functions are the process-wide instance the control
 * stream (signals) and the encode loop (per-frame target) share.
 */
#pragma once

// standard includes
#include <chrono>
#include <cstdint>

namespace abr {
  using clock = std::chrono::steady_clock;

  /**
   * @brief Moonlight's own default bitrate for a stream mode, in kbps.
   *
   * moonlight-qt StreamingPreferences::getDefaultBitrate(): a per-resolution
   * factor interpolated over the Shield-era table, times a frame-rate factor
   * that grows linearly to 60 fps and with the square root above it.
   */
  double moonlight_default_kbps(int width, int height, double fps);

  /**
   * @brief The requested bitrate rescaled to the mode actually being streamed.
   *
   * requested x default(mode) / default(requested mode), never above the
   * request and never below a twentieth of it.
   */
  std::int64_t mode_ceiling_bps(std::int64_t requested_bps, int req_width, int req_height, double req_fps, int width, int height, double fps);

  /**
   * @brief The automatic floor: an eighth of the requested bitrate, at least 2 Mbps.
   */
  std::int64_t auto_floor_bps(std::int64_t requested_bps);

  /**
   * @brief The adaptation policy. Not thread-safe; see the free functions.
   */
  class controller_t {
  public:
    /// What happened since the last take_window(), for the periodic log.
    struct window_t {
      int recovered {};  ///< frames FEC had to repair (loss the player did not see)
      int lost {};  ///< frames FEC could not repair
      int keyframe_requests {};  ///< IDR / reference-invalidation requests counted as loss
      int decreases {};
      std::int64_t low_bps {};  ///< lowest target in the window
    };

    void begin(std::int64_t ceiling_bps, std::int64_t floor_bps, clock::time_point now);

    /// The mode-scaled ceiling moved (the source re-locked in another mode, or
    /// the stream changed size). An unconstrained target follows it; a
    /// backed-off one keeps its distance below.
    void set_ceiling(std::int64_t ceiling_bps, clock::time_point now);

    /// Keyframe requests before `until` are not loss: the encoder itself just
    /// started a new stream head (session start, re-lock, size change).
    void grace_until(clock::time_point until);

    /// A frame that lost packets: `recovered` when FEC repaired it.
    void on_frame_loss(bool recovered, clock::time_point now);

    /// The client asked for a keyframe or invalidated references.
    void on_keyframe_request(clock::time_point now);

    /// Run the policy; returns the bitrate the encoder should use now.
    std::int64_t update(clock::time_point now);

    std::int64_t target() const {
      return target_;
    }

    std::int64_t ceiling() const {
      return ceiling_;
    }

    std::int64_t floor() const {
      return floor_;
    }

    window_t take_window();

    // Policy constants (exposed for the tests).
    static constexpr auto decrease_spacing = std::chrono::milliseconds(250);  ///< a decrease needs this long to show on the wire
    static constexpr auto hold_after_loss = std::chrono::milliseconds(2000);  ///< quiet time before climbing
    static constexpr auto hold_after_severe = std::chrono::milliseconds(3000);
    static constexpr auto step_spacing = std::chrono::milliseconds(500);
    static constexpr double severe_factor = 0.70;  ///< lost frame / keyframe request
    static constexpr double mild_factor = 0.85;  ///< FEC-recovered frame
    static constexpr double mild_limit = 0.50;  ///< FEC-only loss never pushes below this share of the ceiling
    static constexpr double fast_step = 1.08;  ///< climbing back toward the rate that broke
    static constexpr double probe_step = 0.02;  ///< share of the ceiling added per step past it

  private:
    std::int64_t ceiling_ {};
    std::int64_t floor_ {};
    std::int64_t target_ {};
    std::int64_t knee_ {};  ///< the target when loss last started
    int pending_recovered_ {};
    int pending_lost_ {};
    int pending_keyframe_requests_ {};
    clock::time_point grace_until_ {};
    clock::time_point last_decrease_ {};
    clock::time_point last_step_ {};
    clock::time_point hold_until_ {};
    window_t window_ {};
  };

  // ---- the process-wide instance (thread-safe) ----

  /**
   * @brief Start adapting for a session. `enabled` false leaves tick() at 0
   *        (the caller keeps the requested bitrate) and ignores the signals.
   */
  void session_begin(bool enabled, std::int64_t ceiling_bps, std::int64_t floor_bps);

  /// Stop adapting (the session ended).
  void session_end();

  /// See controller_t::set_ceiling.
  void set_ceiling(std::int64_t ceiling_bps);

  /// See controller_t::grace_until: keyframe requests in the next `span` are not loss.
  void grace_for(std::chrono::milliseconds span);

  /// Control stream: a client reported a frame that lost packets (FEC status).
  void report_frame_loss(bool recovered);

  /// Control stream: a client asked for a keyframe / invalidated references.
  void report_keyframe_request();

  /// Encode loop, once per frame: the bitrate to use now; 0 when not adapting.
  std::int64_t tick();
}  // namespace abr
