/**
 * @file src/platform/linux/rkmpp.h
 * @brief Declarations for Rockchip MPP (RKMPP) zero-copy encode device.
 */
#pragma once

// standard includes
#include <memory>

// local includes
#include "misc.h"
#include "src/platform/common.h"

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
}  // namespace rkmpp
