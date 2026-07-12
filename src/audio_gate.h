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

#ifdef __linux__
  // gate-notify FIFO (best-effort, Linux-only like the rest of this fork)
  #include <cerrno>
  #include <csignal>
  #include <fcntl.h>
  #include <unistd.h>
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
   * @brief Lazily-resolved fd of the gate-notify FIFO (SUNSHINE_AUDIO_GATE_FIFO).
   *
   * -2 = unresolved (env not read yet, or the open should be retried — the
   * reader may simply not be up yet), -1 = disabled (env unset/empty),
   * >= 0 = open fd. Pointing the env at retro-audio's event FIFO lets its
   * supervisor mute the capture loopback and re-decide the stream format the
   * moment the video path re-locks — the RX re-handshake can flip the audio
   * between LPCM and IEC 61937, and every pactl round-trip before that mute is
   * raw bitstream static played as PCM.
   */
  inline std::atomic<int> notify_fd {-2};

  /**
   * @brief Best-effort one-line notification to the gate FIFO. Never blocks,
   * never fails the caller.
   */
  inline void notify() {
#ifdef __linux__
    int fd = notify_fd.load(std::memory_order_relaxed);
    if (fd == -1) {
      return;
    }
    if (fd == -2) {
      const char *path = std::getenv("SUNSHINE_AUDIO_GATE_FIFO");
      if (!path || !*path) {
        notify_fd.store(-1, std::memory_order_relaxed);
        return;
      }
      // A FIFO write can raise SIGPIPE if the reader vanishes between open and
      // write (retro-audio recreates its FIFO on restart). Nothing in Sunshine
      // handles SIGPIPE (asio sends use MSG_NOSIGNAL), so ignore it process-
      // wide — but only on hosts that opted into the FIFO.
      ::signal(SIGPIPE, SIG_IGN);
      // O_NONBLOCK: ENXIO when no reader has the FIFO open — stay unresolved
      // and retry on the next trigger instead of blocking the video thread.
      fd = ::open(path, O_WRONLY | O_NONBLOCK | O_CLOEXEC);
      if (fd < 0) {
        return;
      }
      int expected = -2;
      if (!notify_fd.compare_exchange_strong(expected, fd, std::memory_order_relaxed)) {
        ::close(fd);  // another thread won the race; use its fd
        fd = expected;
        if (fd < 0) {
          return;
        }
      }
    }
    if (::write(fd, "gate\n", 5) < 0 && errno == EPIPE) {
      // Stale fd (reader recreated its FIFO): drop it and re-resolve next time.
      ::close(fd);
      notify_fd.store(-2, std::memory_order_relaxed);
    }
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
