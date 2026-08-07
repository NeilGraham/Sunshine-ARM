/*
 * retro-overlay-client — C++ SINK-role client for the retro-overlay daemon.
 *
 * Copyright (c) 2026 Neil Graham
 * SPDX-License-Identifier: MIT
 *
 * WHY THIS LIVES HERE
 *   It was 352 lines inside the Sunshine fork next to a hand-copied
 *   retro-overlay-protocol.h that had drifted from the owning repo (Sunshine's
 *   copy still had `uint8_t reserved[3]` where the owner had split out
 *   `gate_window_ms`). It is grouped with the capture client because both are
 *   consumer-side plumbing for the same encoder, and because retro-overlay is
 *   a Rust crate inside the 42 MB retro-stream workspace — far too heavy to
 *   submodule into Sunshine for one header.
 *
 *   retro-overlay remains the PROTOCOL owner. include/retro-overlay-protocol.h
 *   here is a copy of its reference header (the "GPL seam" its Cargo.toml
 *   names), kept honest by `make check-protocol-sync`.
 *
 * DELIBERATELY NO FFmpeg
 *   A background thread mirrors the daemon's {surfaces, palette, present}
 *   state. current_osd() hands back a filled rovl_osd_side_data; turning that
 *   into AV_FRAME_DATA_RKMPP_OSD side data is the consumer's ~15 lines, and
 *   the only part that was ever Sunshine-specific.
 *
 *   Daemon absent or dead => current_osd() returns false => the consumer
 *   attaches nothing => the stream is byte-identical to a build without this.
 */
#pragma once

#include "retro-capture-client.h"  // log_level / log_fn
#include "retro-overlay-protocol.h"

namespace retro::overlay {

  // The enumerators need importing alongside the type: `using` on an
  // unscoped enum's name does not carry its enumerators into this namespace.
  using retro::capture::log_fn;
  using retro::capture::log_level;
  using retro::capture::log_info;
  using retro::capture::log_warning;

  void set_logger(log_fn fn);

  /**
   * Tell the daemon what the encoder is producing. Cheap and idempotent; call
   * it once per frame. The client thread re-announces SINK_INFO only when the
   * geometry actually drifts from what the daemon was last told.
   *
   * Also starts the client thread on first use.
   */
  void report_encode_geometry(int width, int height, bool ten_bit);

  /**
   * Fill `out` with the OSD state to attach to the frame being encoded.
   *
   * @return true if the overlay is visible and its surface is valid. False
   *         means attach nothing — which is also what happens when the daemon
   *         is absent, so the no-overlay path costs a mutex and a bool.
   */
  bool current_osd(rovl_osd_side_data &out);

}  // namespace retro::overlay
