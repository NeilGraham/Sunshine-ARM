/**
 * @file src/audio_gate.h
 * @brief Cross-module squelch flag for the HDMI-RX transition click.
 *
 * The vendor rk_hdmirx audio path re-inits its capture FIFO on every source
 * re-lock (video mode switch, HDCP/audio blip, input flip). That FIFO re-init
 * dumps a burst of full-scale samples into the PCM stream — a loud click/pop
 * measured directly at the RX capture (peak 31929/32767 ~= 0.97 FS,
 * 2026-07-08 root-cause). The driver is built into the vendor 6.1 kernel, so a
 * proper fix is a kernel rebuild (parked). This flag lets the video capture
 * path (rkmpp.cpp), which is the first code to know a re-lock/re-init is
 * happening, tell the audio capture path (audio.cpp) to squelch the PCM stream
 * across the transition so the burst never reaches the Opus encoder / client.
 *
 * Header-only, lock-free (single atomic), safe to call from any thread.
 */
#pragma once

// standard includes
#include <atomic>
#include <chrono>
#include <cstdint>

namespace audio_gate {
  /**
   * @brief Steady-clock millisecond timestamp until which audio should be squelched.
   *
   * 0 means "not gated". Written by trigger(), read by active(). steady_clock
   * so it is immune to wall-clock jumps (this host has no RTC — see
   * rock5b-watchdog-autoreset memory).
   */
  inline std::atomic<std::int64_t> squelch_until_ms {0};

  /**
   * @brief Squelch duration (ms) the renegotiation sites (rkmpp.cpp) arm.
   *
   * Shared so the env tunable SUNSHINE_AUDIO_GATE_MS, read once at audio
   * capture start (audio.cpp), controls the reneg-triggered squelch length
   * even though the triggers fire from the video path. Defaults to 500 ms.
   * The self-detected squelch heuristic in audio.cpp uses its own shorter
   * duration and does not consult this.
   */
  inline std::atomic<int> reneg_ms {500};

  inline std::int64_t now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
  }

  /**
   * @brief Arm (or extend) the squelch for @p ms milliseconds from now.
   *
   * Extend-don't-shorten: a trigger that would end sooner than an already
   * pending squelch is ignored, so overlapping re-init events (rkmpp fires from
   * several sites for one transition) never cut a longer squelch short.
   */
  inline void trigger(int ms) {
    if (ms <= 0) {
      return;
    }
    const std::int64_t until = now_ms() + ms;
    std::int64_t cur = squelch_until_ms.load(std::memory_order_relaxed);
    while (until > cur &&
           !squelch_until_ms.compare_exchange_weak(cur, until, std::memory_order_relaxed)) {
      // cur reloaded with the current value; retry until we win or a later
      // deadline already covers us.
    }
  }

  /**
   * @brief True while the squelch is armed (now < squelch_until_ms).
   */
  inline bool active() {
    return now_ms() < squelch_until_ms.load(std::memory_order_relaxed);
  }
}  // namespace audio_gate
