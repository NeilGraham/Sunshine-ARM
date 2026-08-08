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
#include <cstdlib>

// The gate-notify FIFO itself belongs to retro-capture's client library
// (third-party/retro-capture, MIT): it is that project's pipe, and its daemon
// writes the same "gate" lines from the other side. What stays here is the
// squelch policy — how long, and which events deserve it. The library is only
// linked in RKMPP builds, which are the only ones with an HDMI-RX to gate.
#if defined(__linux__) && defined(SUNSHINE_BUILD_RKMPP)
  #include "retro-capture-client.h"
  #define SUNSHINE_HAVE_GATE_FIFO 1
#endif

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
   * @brief Best-effort one-line notification to the gate FIFO. Never blocks,
   * never fails the caller.
   *
   * SUNSHINE_AUDIO_GATE_FIFO points at retro-audio's event FIFO, so its
   * supervisor can mute the capture loopback and re-decide the stream format
   * the moment the video path re-locks — the RX re-handshake can flip the
   * audio between LPCM and IEC 61937, and every pactl round-trip before that
   * mute is raw bitstream static played as PCM. The pipe (fd caching,
   * absent-reader retry, SIGPIPE) is the client library's; the decision to
   * poke it is ours.
   */
  inline void notify() {
#ifdef SUNSHINE_HAVE_GATE_FIFO
    retro::capture::audio_gate_notify(std::getenv("SUNSHINE_AUDIO_GATE_FIFO"));
#endif
  }

  /**
   * @brief Arm (or extend) the squelch for @p ms milliseconds from now.
   *
   * Extend-don't-shorten: a trigger that would end sooner than an already
   * pending squelch is ignored, so overlapping re-init events (rkmpp fires from
   * several sites for one transition) never cut a longer squelch short.
   *
   * When @p notify_fifo is set (the rkmpp reneg sites), also pokes the
   * gate-notify FIFO (see notify()) so retro-audio's supervisor learns about
   * the re-lock at the same instant the in-process squelch arms. The audio.cpp
   * self-detect heuristic passes false: it fires on click-shaped PROGRAM
   * content too (the ac-click soak fixture is built from exactly that), and a
   * format flip it could herald is already caught upstream by retro-audio's
   * own scanner — so it must not churn the format tap.
   */
  inline void trigger(int ms, bool notify_fifo = true) {
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
    if (notify_fifo) {
      notify();
    }
  }

  /**
   * @brief True while the squelch is armed (now < squelch_until_ms).
   */
  inline bool active() {
    return now_ms() < squelch_until_ms.load(std::memory_order_relaxed);
  }
}  // namespace audio_gate
