/*
 * retro-capture-client — C++ consumer client for the retro-capture daemon.
 *
 * Copyright (c) 2026 Neil Graham
 * SPDX-License-Identifier: MIT
 *
 * WHY THIS EXISTS
 *   This code used to live inside the Sunshine fork, as a 370-line class in
 *   the middle of rkmpp.cpp, alongside a hand-copied retro-capture-protocol.h
 *   that had already drifted a protocol version behind this repo. Both the
 *   protocol and its client belong to the daemon that defines them, so they
 *   live here now and Sunshine vendors this directory verbatim
 *   (third-party/retro-capture/), with a build-failing diff tripwire in
 *   ~/bin/build-sunshine-arm keeping the copy honest.
 *
 * DELIBERATELY NO FFmpeg
 *   The client hands back pool indices and dma-buf file descriptors. Wrapping
 *   those into AVFrames is the consumer's job — that is the only part that was
 *   ever Sunshine-specific, and keeping it out means retro-capture never grows
 *   an FFmpeg dependency (see this repo's CMakeLists: "No FFmpeg").
 *
 * DELIBERATELY NO POLICY
 *   next() reports the frame flags the daemon sent and nothing more. What a
 *   HELD or BLACK run *means* — how long to tolerate it, whether to squelch
 *   audio — is the consumer's policy, and Sunshine keeps it.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace retro::capture {

  /// Severity passed to a log callback. Deliberately not an enum class so a
  /// consumer can map it onto its own logger with a plain switch.
  enum log_level : int {
    log_info = 0,
    log_warning = 1,
  };

  /// Consumers route the client's diagnostics into their own logging. Sunshine
  /// forwards these to BOOST_LOG; the default drops them.
  using log_fn = void (*)(log_level level, const char *message);

  void set_logger(log_fn fn);

  /// The pool geometry the daemon served, straight from STREAM_INFO. Mirrors
  /// the fields a consumer needs to describe a buffer to its own graphics
  /// stack; see PROTOCOL.md for the authoritative definitions.
  struct stream_info {
    std::uint32_t width {};
    std::uint32_t height {};
    std::uint32_t fourcc {};  ///< DRM_FORMAT_NV12 / DRM_FORMAT_NV15
    std::uint32_t stride_y {};
    std::uint32_t stride_uv {};
    std::uint32_t offset_y {};
    std::uint32_t offset_uv {};
    std::uint64_t buffer_size {};
    std::uint64_t modifier {};
    std::uint32_t pool_count {};
  };

  /// The buffer a consumer should encode this tick.
  struct frame {
    int index {-1};          ///< pool index; -1 before the first frame
    std::uint32_t flags {};  ///< RCAP_FRAME_* bits from the daemon
    bool changed {false};    ///< index differs from the previous next() call
  };

  /// One FRAME message as it crossed the socket, before newest-frame-wins
  /// collapses a drain to its winner. Telemetry, not policy: the daemon's
  /// pipeline timestamps plus the client's own receive time, exactly the CSV
  /// row the test-consumer oracle logs. Consumers that only encode never need
  /// this; the default is no observer.
  struct frame_telemetry {
    std::uint64_t serial {};
    std::uint32_t index {};
    std::uint32_t flags {};
    std::uint64_t dqbuf_ns {};
    std::uint64_t process_done_ns {};
    std::uint64_t send_ns {};
    std::uint64_t recv_ns {};  ///< CLOCK_MONOTONIC at recv, taken by the client
  };

  /// Called from inside next()'s drain loop for every FRAME message, in
  /// arrival order, on the caller's thread. Set once before the loop starts.
  using frame_observer = void (*)(const frame_telemetry &t, void *user);

  /// Fields a consumer may need from the daemon's STATUS document without
  /// holding the (single) CONSUMER slot. Additive: a field the daemon did not
  /// report keeps the default below, so an older daemon degrades quietly.
  struct status {
    bool valid {false};      ///< a STATUS document was received and parsed
    int input_bit_depth {0}; ///< input.bit_depth: 8, or 10 for a BT.2020+PQ
                             ///< (NV15) capture — PROTOCOL.md §3.10
    bool signal_locked {false};
  };

  /// Best-effort "gate\n" line to the retro-audio event FIFO at `fifo_path`,
  /// telling its supervisor a video re-lock is happening right now — the RX
  /// re-handshake can flip the audio between LPCM and IEC 61937, and every
  /// round-trip before that notification is raw bitstream static played as
  /// PCM. Never blocks and never fails the caller: a missing FIFO, an absent
  /// reader, or a reader that vanished all resolve to "did nothing".
  ///
  /// A null/empty path disables it permanently for the process. WHAT counts
  /// as gate-worthy stays the caller's policy — this only owns the pipe.
  ///
  /// SIDE EFFECT, deliberate and inherited from the code this replaced: the
  /// first call with a usable path sets SIGPIPE to SIG_IGN process-wide,
  /// because write() to a FIFO whose reader vanished raises it and there is
  /// no MSG_NOSIGNAL for write(). Consumers that handle SIGPIPE themselves
  /// should not call this.
  void audio_gate_notify(const char *fifo_path);

  /// One-shot STATUS-role query: connect, HELLO, STATUS_GET, parse, close.
  /// Deliberately synchronous with a short timeout and no caching — callers
  /// run it at session-setup/probe cadence, never per frame, and decide their
  /// own caching. Returns a status with valid=false if the daemon is absent
  /// or does not answer in time.
  status query_status(const char *name);

  /// A connected consumer session. Not thread-safe: drive it from one thread,
  /// as the encoder loop does.
  class client {
  public:
    ~client();

    client(client &&) noexcept;
    client &operator=(client &&) noexcept;
    client(const client &) = delete;
    client &operator=(const client &) = delete;

    /// Socket path: $RETRO_CAPTURE_SOCKET, else the protocol default.
    static const char *socket_path();

    /// Connect, HELLO, SETUP, then import the buffer pool. `want_fourcc` is the
    /// pool format the consumer's encoder was configured for — the daemon may
    /// be capturing at a different bit depth and will downconvert, but a pool
    /// it cannot serve in that exact format is a failed connect rather than a
    /// silent mismatch. `name` is the consumer name announced in HELLO and
    /// surfaced in the daemon's STATUS as consumer_name. Zero width/height
    /// means "the daemon's configured output" and zero want_fourcc means "any
    /// pool format" (both sent wire-identically to a v1 consumer) — oracle
    /// conveniences; a real encoder passes all three explicitly. Returns
    /// nullptr on any failure — see PROTOCOL.md §4 for what a consumer is
    /// allowed to do next (note that a fallback capturing something OTHER
    /// than the HDMI-RX, as Sunshine's KMS path does, is not a substitute).
    static std::unique_ptr<client> connect(int width, int height, std::uint32_t want_fourcc,
                                           const char *name);

    /// Drain the socket, keep the newest FRAME, release superseded leases.
    /// Newest-frame-wins: a consumer that falls behind skips frames rather than
    /// queueing them. Returns index -1 only before the first frame.
    frame next();

    /// Re-establish the session after daemon death, at most once a second.
    /// Until it succeeds the previously received buffers stay valid — the
    /// dma-buf fds outlive the daemon process — so a consumer can keep
    /// re-sending the last frame instead of tearing down.
    void maybe_reconnect(int width, int height, std::uint32_t want_fourcc, const char *name);

    /// Per-FRAME telemetry hook (see frame_observer). Survives reconnects.
    void set_frame_observer(frame_observer fn, void *user);

    bool alive() const;
    const stream_info &info() const;
    std::size_t pool_size() const;
    /// dma-buf fd for a pool slot. Owned by the client; do not close.
    int pool_fd(std::size_t index) const;
    /// The session socket, for callers that want to poll() for readability
    /// between next() calls instead of pacing on their own timer (the
    /// test-consumer oracle does; Sunshine's encoder loop does not).
    /// -1 while dead. Do not read or close.
    int fd() const;

  private:
    client();
    struct impl;
    std::unique_ptr<impl> p;
  };

}  // namespace retro::capture
