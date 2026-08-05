/**
 * @file src/platform/linux/retro-overlay.h
 * @brief retro-overlay client: encoder-OSD side data fed by the overlay daemon.
 *
 * A background thread owns a SINK-role connection to the retro-overlay daemon
 * (SOCK_SEQPACKET, see the vendored retro-overlay-protocol.h) and mirrors the
 * daemon's latest {surfaces, palette, present} state. The encode thread calls
 * rovl::attach() once per frame; it attaches AV_FRAME_DATA_RKMPP_OSD side data
 * when the overlay is visible and removes it when hidden. Daemon absent or
 * dead ⇒ no side data ⇒ the stream is byte-identical to a build without this
 * client.
 */
#pragma once

struct AVFrame;

namespace rovl {

  /**
   * Attach or remove OSD side data on the frame about to be encoded.
   * Starts the client thread on first use. Never blocks on the socket.
   *
   * @param frame     The DRM_PRIME wrapper frame handed to the encoder.
   *                  Long-lived and reused: stale side data is removed here.
   * @param width     Encoded frame width in pixels.
   * @param height    Encoded frame height in pixels.
   * @param ten_bit   True for NV15/10-bit sessions (daemon picks PQ palette).
   */
  void attach(AVFrame *frame, int width, int height, bool ten_bit);

}  // namespace rovl
