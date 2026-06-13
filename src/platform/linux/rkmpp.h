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
   * @brief Create a zero-copy RKMPP encode device backed by EGL color conversion.
   * @param width Captured image width.
   * @param height Captured image height.
   * @param card Render node fd used for GBM/EGL (ownership transferred).
   * @param offset_x Horizontal offset of the captured region.
   * @param offset_y Vertical offset of the captured region.
   * @return The encode device or nullptr on failure.
   */
  std::unique_ptr<platf::avcodec_encode_device_t> make_avcodec_encode_device(int width, int height, file_t &&card, int offset_x, int offset_y);
}  // namespace rkmpp
