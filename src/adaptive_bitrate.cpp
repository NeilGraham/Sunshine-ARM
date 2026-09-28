/**
 * @file src/adaptive_bitrate.cpp
 * @brief Network-adaptive bitrate for the capture-box stream (see the header).
 */
// header include
#include "adaptive_bitrate.h"

// standard includes
#include <algorithm>
#include <cmath>
#include <iterator>
#include <mutex>
#include <unordered_map>

// ABR_POLICY_ONLY builds just the policy (the controller and the mode math)
// for a standalone test, without Sunshine's logging.
#ifndef ABR_POLICY_ONLY
  // local includes
  #include "logging.h"
#endif

using namespace std::literals;

namespace abr {
  double moonlight_default_kbps(int width, int height, double fps) {
    // moonlight-qt StreamingPreferences::getDefaultBitrate(), verbatim policy.
    const double frame_rate_factor = (fps <= 60.0 ? fps : std::sqrt(fps / 60.0) * 60.0) / 30.0;
    static constexpr struct {
      long pixels;
      double factor;
    } table[] {
      {640L * 360, 1},
      {854L * 480, 2},
      {1280L * 720, 5},
      {1920L * 1080, 10},
      {2560L * 1440, 20},
      {3840L * 2160, 40},
    };
    const long pixels = (long) width * height;
    double resolution_factor = table[std::size(table) - 1].factor;
    for (std::size_t i = 0; i < std::size(table); ++i) {
      if (pixels <= table[i].pixels) {
        if (i == 0 || pixels == table[i].pixels) {
          resolution_factor = table[i].factor;
        } else {
          const auto &lo = table[i - 1];
          const auto &hi = table[i];
          resolution_factor = lo.factor + (double) (pixels - lo.pixels) / (hi.pixels - lo.pixels) * (hi.factor - lo.factor);
        }
        break;
      }
    }
    return resolution_factor * frame_rate_factor * 1000.0;
  }

  std::int64_t mode_ceiling_bps(std::int64_t requested_bps, int req_width, int req_height, double req_fps, int width, int height, double fps) {
    if (requested_bps <= 0 || req_width <= 0 || req_height <= 0 || req_fps <= 0.0 ||
        width <= 0 || height <= 0 || fps <= 0.0) {
      return requested_bps;
    }
    const double requested_default = moonlight_default_kbps(req_width, req_height, req_fps);
    if (requested_default <= 0.0) {
      return requested_bps;
    }
    // 59.94 against a requested 60 is the source's clock, not a slower mode.
    if (std::abs(fps - req_fps) <= req_fps * 0.01) {
      fps = req_fps;
    }
    const double ratio = std::clamp(moonlight_default_kbps(width, height, fps) / requested_default, 0.05, 1.0);
    return (std::int64_t) std::llround((double) requested_bps * ratio);
  }

  std::int64_t auto_floor_bps(std::int64_t requested_bps) {
    return std::max<std::int64_t>(2'000'000, requested_bps / 8);
  }

  void controller_t::begin(std::int64_t ceiling_bps, std::int64_t floor_bps, clock::time_point now) {
    ceiling_ = std::max<std::int64_t>(ceiling_bps, 1);
    floor_ = std::clamp<std::int64_t>(floor_bps, 1, ceiling_);
    target_ = ceiling_;
    knee_ = ceiling_;
    pending_recovered_ = pending_lost_ = pending_keyframe_requests_ = 0;
    last_knee_bps_ = 0;
    last_knee_at_ = {};
    held_bps_ = 0;
    grace_until_ = now;
    last_decrease_ = {};
    last_step_ = now;
    hold_until_ = now;
    window_ = {};
    window_.low_bps = target_;
  }

  void controller_t::start_at(std::int64_t bps, clock::time_point now) {
    const auto start = std::clamp(bps, floor_, ceiling_);
    if (start >= ceiling_) {
      return;
    }
    // As if the link had just broken at this rate: probe past it slowly.
    target_ = start;
    knee_ = start;
    held_bps_ = start;
    last_knee_bps_ = start;
    last_knee_at_ = now;
    last_step_ = now;
    hold_until_ = now + hold_after_loss;
    window_.low_bps = std::min(window_.low_bps, target_);
  }

  void controller_t::set_ceiling(std::int64_t ceiling_bps, clock::time_point now) {
    ceiling_bps = std::max<std::int64_t>(ceiling_bps, 1);
    if (ceiling_bps == ceiling_) {
      return;
    }
    const bool unconstrained = target_ >= ceiling_;
    // Keep a backed-off target the same share below the new ceiling: the
    // link did not get better or worse because the console changed mode.
    const double share = (double) target_ / (double) ceiling_;
    const bool rising = ceiling_bps > ceiling_;
    ceiling_ = ceiling_bps;
    floor_ = std::min(floor_, ceiling_);
    target_ = unconstrained ? ceiling_ : std::clamp<std::int64_t>((std::int64_t) (share * ceiling_), floor_, ceiling_);
    knee_ = std::min(knee_, ceiling_);
    if (rising && unconstrained && last_knee_bps_ > 0 && now - last_knee_at_ < knee_memory && last_knee_bps_ < target_) {
      // The link broke at last_knee_bps_ moments ago; a bigger picture does
      // not make it faster. Resume there and probe upward.
      target_ = std::max(floor_, last_knee_bps_);
      knee_ = target_;
      hold_until_ = std::max(hold_until_, now + step_spacing);
    }
    last_step_ = now;
    window_.low_bps = std::min(window_.low_bps, target_);
  }

  void controller_t::grace_until(clock::time_point until) {
    grace_until_ = std::max(grace_until_, until);
  }

  void controller_t::on_frame_loss(bool recovered, clock::time_point) {
    if (recovered) {
      ++pending_recovered_;
      ++window_.recovered;
    } else {
      ++pending_lost_;
      ++window_.lost;
    }
  }

  void controller_t::on_keyframe_request(clock::time_point now) {
    if (now < grace_until_) {
      return;
    }
    ++pending_keyframe_requests_;
    ++window_.keyframe_requests;
  }

  std::int64_t controller_t::update(clock::time_point now) {
    const bool severe = pending_lost_ > 0 || pending_keyframe_requests_ > 0;
    const bool mild = pending_recovered_ > 0;
    pending_recovered_ = pending_lost_ = pending_keyframe_requests_ = 0;

    if (severe || mild) {
      // Loss is still being reported: whatever happens below, do not climb
      // until it has been quiet for a while.
      hold_until_ = std::max(hold_until_, now + (severe ? hold_after_severe : hold_after_loss));
      // Space decreases so each one can show on the wire before the next: a
      // report describes a frame sent one round trip and an encoder budget
      // window ago, so a burst of reports is one event, not many.
      if (last_decrease_ == clock::time_point {} || now - last_decrease_ >= decrease_spacing) {
        // FEC-repaired loss is invisible to the player: back off gently, and
        // never below half the ceiling on its strength alone (random Wi-Fi
        // loss that FEC always repairs is not congestion, and chasing it to
        // the floor would only cost picture quality). A frame actually lost,
        // or a keyframe request, may go all the way to the floor.
        const std::int64_t bound = severe ? floor_ : std::max(floor_, (std::int64_t) (ceiling_ * mild_limit));
        if (target_ > bound) {
          knee_ = target_;
          last_knee_bps_ = target_;
          last_knee_at_ = now;
          target_ = std::max(bound, (std::int64_t) (target_ * (severe ? severe_factor : mild_factor)));
          held_bps_ = target_;
          last_decrease_ = now;
          ++window_.decreases;
        }
      }
    } else if (now >= hold_until_ && target_ < ceiling_ && now - last_step_ >= step_spacing) {
      // Quiet: climb. Quickly while well below the rate the link broke at,
      // then in small probes past it.
      const double next = target_ < knee_ * 0.9 ? target_ * fast_step : target_ + ceiling_ * probe_step;
      target_ = std::min<std::int64_t>(ceiling_, (std::int64_t) next);
      last_step_ = now;
      if (target_ >= knee_) {
        knee_ = ceiling_;  // past it without loss: the old knee no longer applies
      }
    }
    window_.low_bps = std::min(window_.low_bps, target_);
    return target_;
  }

  controller_t::window_t controller_t::take_window() {
    auto w = window_;
    window_ = {};
    window_.low_bps = target_;
    return w;
  }

#ifndef ABR_POLICY_ONLY
  namespace {
    std::mutex mu;
    bool active = false;  // under mu
    controller_t ctl;  // under mu
    std::int64_t last_logged_bps = 0;  // under mu
    clock::time_point window_start;  // under mu
    std::uint64_t session_client = 0;  // under mu

    // Per paired client: the rate its link held when its last session ended
    // (only for sessions that had to back off). Process lifetime only.
    struct remembered_t {
      std::int64_t bps;
      clock::time_point at;
    };

    std::unordered_map<std::uint64_t, remembered_t> remembered;  // under mu
    constexpr auto remember_for = std::chrono::minutes(30);

    double mbps(std::int64_t bps) {
      return std::round(bps / 100000.0) / 10.0;
    }
  }  // namespace

  std::int64_t session_begin(bool enabled, std::int64_t ceiling_bps, std::int64_t floor_bps, std::uint64_t client_key) {
    std::lock_guard lk(mu);
    active = enabled;
    if (!enabled) {
      return 0;
    }
    const auto now = clock::now();
    ctl.begin(ceiling_bps, floor_bps, now);
    session_client = client_key;
    std::int64_t learned = 0;
    if (client_key) {
      if (auto it = remembered.find(client_key); it != remembered.end()) {
        if (now - it->second.at < remember_for) {
          learned = it->second.bps;
          ctl.start_at(learned, now);
        } else {
          remembered.erase(it);
        }
      }
    }
    // The client's own first keyframe requests are the stream starting.
    ctl.grace_until(now + 3s);
    last_logged_bps = ctl.target();
    window_start = now;
    BOOST_LOG(info) << "Dynamic bitrate: "sv << mbps(ctl.ceiling()) << " Mbps ceiling, "sv << mbps(ctl.floor())
                    << " Mbps floor; backing off on packet loss"sv;
    if (learned && ctl.target() < ctl.ceiling()) {
      BOOST_LOG(info) << "Dynamic bitrate: this client's link held "sv << mbps(learned)
                      << " Mbps last session; starting there and probing up"sv;
    }
    return ctl.target();
  }

  void session_end() {
    std::lock_guard lk(mu);
    if (active && session_client) {
      // Remember a link that had to back off; forget one that never did.
      if (const auto held = ctl.held_bps(); held > 0) {
        remembered[session_client] = {std::min(held, ctl.target()), clock::now()};
      } else {
        remembered.erase(session_client);
      }
    }
    active = false;
  }

  void set_ceiling(std::int64_t ceiling_bps) {
    std::lock_guard lk(mu);
    if (!active || ceiling_bps == ctl.ceiling()) {
      return;
    }
    const auto was = ctl.ceiling();
    ctl.set_ceiling(ceiling_bps, clock::now());
    BOOST_LOG(info) << "Dynamic bitrate: ceiling "sv << mbps(was) << " -> "sv << mbps(ctl.ceiling())
                    << " Mbps for the source's mode; target "sv << mbps(ctl.target()) << " Mbps"sv;
    last_logged_bps = ctl.target();
  }

  void grace_for(std::chrono::milliseconds span) {
    std::lock_guard lk(mu);
    if (active) {
      ctl.grace_until(clock::now() + span);
    }
  }

  void report_frame_loss(bool recovered) {
    std::lock_guard lk(mu);
    if (active) {
      ctl.on_frame_loss(recovered, clock::now());
    }
  }

  void report_keyframe_request() {
    std::lock_guard lk(mu);
    if (active) {
      ctl.on_keyframe_request(clock::now());
    }
  }

  std::int64_t tick() {
    std::lock_guard lk(mu);
    if (!active) {
      return 0;
    }
    const auto now = clock::now();
    const auto before = ctl.target();
    const auto target = ctl.update(now);
    if (target < before) {
      BOOST_LOG(info) << "Dynamic bitrate: packet loss — "sv << mbps(before) << " -> "sv << mbps(target) << " Mbps"sv;
      last_logged_bps = target;
    } else if (target > before && (target == ctl.ceiling() || target >= last_logged_bps * 5 / 4)) {
      // Climbing is logged at the ceiling and every +25%, not per step.
      BOOST_LOG(info) << "Dynamic bitrate: quiet — back up to "sv << mbps(target) << " Mbps"sv
                      << (target == ctl.ceiling() ? " (ceiling)"sv : ""sv);
      last_logged_bps = target;
    }
    // A summary every 10 s while anything is happening.
    if (now - window_start >= 10s) {
      window_start = now;
      const auto w = ctl.take_window();
      if (w.recovered || w.lost || w.keyframe_requests || target < ctl.ceiling()) {
        BOOST_LOG(info) << "Dynamic bitrate: "sv << mbps(target) << " Mbps (ceiling "sv << mbps(ctl.ceiling())
                        << ", low "sv << mbps(w.low_bps) << "); last 10 s: "sv << w.recovered
                        << " frames repaired by FEC, "sv << w.lost << " lost, "sv << w.keyframe_requests
                        << " keyframe requests, "sv << w.decreases << " back-offs"sv;
      }
    }
    return target;
  }
#endif
}  // namespace abr
