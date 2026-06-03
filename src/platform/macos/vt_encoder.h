/**
 * @file src/platform/macos/vt_encoder.h
 * @brief Native VideoToolbox encoder session factory for macOS.
 */
#pragma once

// standard includes
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

// local includes
#include "src/video.h"

namespace video {
  struct vt_encoded_frame_t {
    std::vector<std::uint8_t> data;
    int64_t frame_index;
    bool idr;
  };

  class native_vt_encode_session_t: public encode_session_t {
  public:
    virtual std::optional<vt_encoded_frame_t> encode_frame(int64_t frame_index) = 0;
    virtual bool has_failed() const = 0;
  };

  std::unique_ptr<encode_session_t> make_vt_native_encode_session(
    platf::display_t *disp,
    const encoder_t &encoder,
    const config_t &config,
    int width,
    int height,
    std::unique_ptr<platf::avcodec_encode_device_t> encode_device
  );
}  // namespace video
