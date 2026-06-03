/**
 * @file src/platform/macos/vt_encoder.mm
 * @brief Native VideoToolbox encoder session for macOS.
 */
// standard includes
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iterator>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

// platform includes
#include <CoreFoundation/CoreFoundation.h>
#include <CoreMedia/CoreMedia.h>
#include <CoreVideo/CoreVideo.h>
#include <VideoToolbox/VideoToolbox.h>

// local includes
#include "src/config.h"
#include "src/logging.h"
#include "src/platform/macos/av_img_t.h"
#include "src/platform/macos/nv12_zero_device.h"
#include "src/platform/macos/vt_encoder.h"

using namespace std::literals;

namespace video {
  namespace {

    constexpr std::uint8_t start_code[] {0x00, 0x00, 0x00, 0x01};

    void release_sample(CMSampleBufferRef sample) {
      if (sample) {
        CFRelease(sample);
      }
    }

    void release_pixel_buffer(CVPixelBufferRef pixel_buffer) {
      if (pixel_buffer) {
        CVPixelBufferRelease(pixel_buffer);
      }
    }

    std::string osstatus_to_string(OSStatus status) {
      CFErrorRef error = CFErrorCreate(kCFAllocatorDefault, kCFErrorDomainOSStatus, status, nullptr);
      if (!error) {
        return std::to_string(status);
      }

      CFStringRef description = CFErrorCopyDescription(error);
      CFRelease(error);
      if (!description) {
        return std::to_string(status);
      }

      char buffer[512] {};
      if (!CFStringGetCString(description, buffer, sizeof(buffer), kCFStringEncodingUTF8)) {
        CFRelease(description);
        return std::to_string(status);
      }

      CFRelease(description);
      return buffer;
    }

    void log_osstatus(boost::log::sources::severity_logger<int> &level, std::string_view context, OSStatus status) {
      BOOST_LOG(level) << "VideoToolbox: "sv << context << " failed: "sv << osstatus_to_string(status);
    }

    CFNumberRef make_i32(int32_t value) {
      return CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &value);
    }

    OSStatus session_set_i32(VTCompressionSessionRef session, CFStringRef key, int32_t value) {
      CFNumberRef number = make_i32(value);
      auto status = VTSessionSetProperty(session, key, number);
      CFRelease(number);
      return status;
    }

    OSStatus session_set_bool(VTCompressionSessionRef session, CFStringRef key, bool value) {
      return VTSessionSetProperty(session, key, value ? kCFBooleanTrue : kCFBooleanFalse);
    }

    CMVideoCodecType codec_type_from_config(const config_t &config) {
      switch (config.videoFormat) {
        case 0:
          return kCMVideoCodecType_H264;
        case 1:
          return kCMVideoCodecType_HEVC;
        case 2:
          return kCMVideoCodecType_AV1;
        default:
          return 0;
      }
    }

    std::string_view codec_name(CMVideoCodecType codec_type) {
      switch (codec_type) {
        case kCMVideoCodecType_H264:
          return "h264"sv;
        case kCMVideoCodecType_HEVC:
          return "hevc"sv;
        case kCMVideoCodecType_AV1:
          return "av1"sv;
        default:
          return "unknown"sv;
      }
    }

    CFStringRef profile_from_config(CMVideoCodecType codec_type, const config_t &config) {
      switch (codec_type) {
        case kCMVideoCodecType_H264:
          return kVTProfileLevel_H264_High_AutoLevel;
        case kCMVideoCodecType_HEVC:
          return config.dynamicRange ? kVTProfileLevel_HEVC_Main10_AutoLevel : kVTProfileLevel_HEVC_Main_AutoLevel;
        default:
          return nullptr;
      }
    }

    OSType pixel_format_from_config(const config_t &config) {
      return config.dynamicRange ? kCVPixelFormatType_420YpCbCr10BiPlanarVideoRange : kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange;
    }

    CFStringRef vt_primaries(const sunshine_colorspace_t &colorspace) {
      switch (colorspace.colorspace) {
        case colorspace_e::rec601:
          return kCVImageBufferColorPrimaries_SMPTE_C;
        case colorspace_e::bt2020:
        case colorspace_e::bt2020sdr:
          return kCVImageBufferColorPrimaries_ITU_R_2020;
        case colorspace_e::rec709:
        default:
          return kCVImageBufferColorPrimaries_ITU_R_709_2;
      }
    }

    CFStringRef vt_transfer(const sunshine_colorspace_t &colorspace) {
      switch (colorspace.colorspace) {
        case colorspace_e::bt2020:
          return kCVImageBufferTransferFunction_SMPTE_ST_2084_PQ;
        case colorspace_e::bt2020sdr:
          return kCVImageBufferTransferFunction_ITU_R_2020;
        case colorspace_e::rec601:
        case colorspace_e::rec709:
        default:
          return kCVImageBufferTransferFunction_ITU_R_709_2;
      }
    }

    CFStringRef vt_matrix(const sunshine_colorspace_t &colorspace) {
      switch (colorspace.colorspace) {
        case colorspace_e::rec601:
          return kCVImageBufferYCbCrMatrix_ITU_R_601_4;
        case colorspace_e::bt2020:
        case colorspace_e::bt2020sdr:
          return kCVImageBufferYCbCrMatrix_ITU_R_2020;
        case colorspace_e::rec709:
        default:
          return kCVImageBufferYCbCrMatrix_ITU_R_709_2;
      }
    }

    CFMutableDictionaryRef create_encoder_spec() {
      auto spec = CFDictionaryCreateMutable(kCFAllocatorDefault, 3, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);

      if (config::video.vt.vt_require_sw) {
        CFDictionarySetValue(spec, kVTVideoEncoderSpecification_EnableHardwareAcceleratedVideoEncoder, kCFBooleanFalse);
      } else if (!config::video.vt.vt_allow_sw) {
        CFDictionarySetValue(spec, kVTVideoEncoderSpecification_RequireHardwareAcceleratedVideoEncoder, kCFBooleanTrue);
      } else {
        CFDictionarySetValue(spec, kVTVideoEncoderSpecification_EnableHardwareAcceleratedVideoEncoder, kCFBooleanTrue);
      }

      return spec;
    }

    CFMutableDictionaryRef create_source_attrs(const config_t &config) {
      auto attrs = CFDictionaryCreateMutable(kCFAllocatorDefault, 4, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);

      CFNumberRef pixel_format = make_i32((int32_t) pixel_format_from_config(config));
      CFNumberRef width = make_i32(config.width);
      CFNumberRef height = make_i32(config.height);
      CFDictionaryRef io_surface_props = CFDictionaryCreate(kCFAllocatorDefault, nullptr, nullptr, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);

      CFDictionarySetValue(attrs, kCVPixelBufferPixelFormatTypeKey, pixel_format);
      CFDictionarySetValue(attrs, kCVPixelBufferWidthKey, width);
      CFDictionarySetValue(attrs, kCVPixelBufferHeightKey, height);
      CFDictionarySetValue(attrs, kCVPixelBufferIOSurfacePropertiesKey, io_surface_props);

      CFRelease(pixel_format);
      CFRelease(width);
      CFRelease(height);
      CFRelease(io_surface_props);

      return attrs;
    }

    void append_start_code(std::vector<std::uint8_t> &data) {
      data.insert(std::end(data), std::begin(start_code), std::end(start_code));
    }

    bool sample_is_keyframe(CMSampleBufferRef sample) {
      CFArrayRef attachments = CMSampleBufferGetSampleAttachmentsArray(sample, false);
      if (!attachments || CFArrayGetCount(attachments) == 0) {
        return true;
      }

      auto attachment = (CFDictionaryRef) CFArrayGetValueAtIndex(attachments, 0);
      auto not_sync = (CFBooleanRef) CFDictionaryGetValue(attachment, kCMSampleAttachmentKey_NotSync);
      if (not_sync) {
        return !CFBooleanGetValue(not_sync);
      }

      auto depends_on_others = (CFBooleanRef) CFDictionaryGetValue(attachment, kCMSampleAttachmentKey_DependsOnOthers);
      return !depends_on_others || depends_on_others == kCFBooleanFalse || !CFBooleanGetValue(depends_on_others);
    }

    OSStatus get_parameter_set_info(CMVideoCodecType codec_type, CMFormatDescriptionRef format_desc, size_t &param_count, int &nal_length_bytes) {
      if (codec_type == kCMVideoCodecType_H264) {
        return CMVideoFormatDescriptionGetH264ParameterSetAtIndex(format_desc, 0, nullptr, nullptr, &param_count, &nal_length_bytes);
      }

      if (codec_type == kCMVideoCodecType_HEVC) {
        return CMVideoFormatDescriptionGetHEVCParameterSetAtIndex(format_desc, 0, nullptr, nullptr, &param_count, &nal_length_bytes);
      }

      return kCMFormatDescriptionError_ValueNotAvailable;
    }

    OSStatus get_parameter_set(CMVideoCodecType codec_type, CMFormatDescriptionRef format_desc, size_t index, const std::uint8_t *&param, size_t &param_size) {
      if (codec_type == kCMVideoCodecType_H264) {
        return CMVideoFormatDescriptionGetH264ParameterSetAtIndex(format_desc, index, &param, &param_size, nullptr, nullptr);
      }

      if (codec_type == kCMVideoCodecType_HEVC) {
        return CMVideoFormatDescriptionGetHEVCParameterSetAtIndex(format_desc, index, &param, &param_size, nullptr, nullptr);
      }

      return kCMFormatDescriptionError_ValueNotAvailable;
    }

    bool append_parameter_sets(CMVideoCodecType codec_type, CMFormatDescriptionRef format_desc, std::vector<std::uint8_t> &data, int &nal_length_bytes) {
      size_t param_count = 0;
      auto status = get_parameter_set_info(codec_type, format_desc, param_count, nal_length_bytes);
      if (status == kCMFormatDescriptionBridgeError_InvalidParameter || status == kCMFormatDescriptionError_InvalidParameter) {
        param_count = codec_type == kCMVideoCodecType_HEVC ? 3 : 2;
        nal_length_bytes = 4;
      } else if (status != noErr) {
        log_osstatus(error, "getting VideoToolbox parameter set count"sv, status);
        return false;
      }

      for (size_t i = 0; i < param_count; ++i) {
        const std::uint8_t *param = nullptr;
        size_t param_size = 0;
        status = get_parameter_set(codec_type, format_desc, i, param, param_size);
        if (status != noErr) {
          log_osstatus(error, "getting VideoToolbox parameter set"sv, status);
          return false;
        }

        append_start_code(data);
        data.insert(std::end(data), param, param + param_size);
      }

      return true;
    }

    bool append_sample_nals(CMSampleBufferRef sample, int nal_length_bytes, std::vector<std::uint8_t> &data) {
      CMBlockBufferRef block = CMSampleBufferGetDataBuffer(sample);
      if (!block) {
        BOOST_LOG(error) << "VideoToolbox: sample has no block buffer"sv;
        return false;
      }

      const auto block_size = CMBlockBufferGetDataLength(block);
      if (block_size == 0) {
        BOOST_LOG(error) << "VideoToolbox: sample block buffer is empty"sv;
        return false;
      }

      std::vector<std::uint8_t> block_data(block_size);
      auto status = CMBlockBufferCopyDataBytes(block, 0, block_size, block_data.data());
      if (status != noErr) {
        log_osstatus(error, "copying VideoToolbox sample block"sv, status);
        return false;
      }

      size_t pos = 0;
      while (pos < block_data.size()) {
        if (nal_length_bytes <= 0 || nal_length_bytes > 4 || pos + (size_t) nal_length_bytes > block_data.size()) {
          BOOST_LOG(error) << "VideoToolbox: invalid NAL length header"sv;
          return false;
        }

        uint32_t nal_size = 0;
        for (int i = 0; i < nal_length_bytes; ++i) {
          nal_size = (nal_size << 8) | block_data[pos + i];
        }
        pos += nal_length_bytes;

        if (nal_size == 0 || pos + nal_size > block_data.size()) {
          BOOST_LOG(error) << "VideoToolbox: invalid NAL block"sv;
          return false;
        }

        append_start_code(data);
        data.insert(std::end(data), std::begin(block_data) + pos, std::begin(block_data) + pos + nal_size);
        pos += nal_size;
      }

      return true;
    }

    std::optional<size_t> find_start_code(const std::vector<std::uint8_t> &data, size_t offset) {
      for (size_t i = offset; i + 3 < data.size(); ++i) {
        if (data[i] == 0 && data[i + 1] == 0 && ((data[i + 2] == 1) || (i + 3 < data.size() && data[i + 2] == 0 && data[i + 3] == 1))) {
          return i;
        }
      }

      return std::nullopt;
    }

    size_t start_code_size(const std::vector<std::uint8_t> &data, size_t offset) {
      return data[offset + 2] == 1 ? 3 : 4;
    }

    void raise_h264_slice_priority(std::vector<std::uint8_t> &data) {
      auto start = find_start_code(data, 0);
      while (start) {
        const auto nal_offset = *start + start_code_size(data, *start);
        if (nal_offset >= data.size()) {
          return;
        }

        const auto nal_type = data[nal_offset] & 0x1F;
        if (nal_type == 5) {
          data[nal_offset] = (data[nal_offset] & ~(3 << 5)) | (3 << 5);
        } else if (nal_type == 1) {
          const auto old_priority = (data[nal_offset] >> 5) & 0x3;
          if (old_priority != 0) {
            data[nal_offset] = (data[nal_offset] & ~(3 << 5)) | (2 << 5);
          }
        }

        start = find_start_code(data, nal_offset + 1);
      }
    }

    std::optional<vt_encoded_frame_t> sample_to_annex_b(CMVideoCodecType codec_type, CMSampleBufferRef sample, int64_t frame_index) {
      if (!CMSampleBufferDataIsReady(sample)) {
        BOOST_LOG(error) << "VideoToolbox: output sample is not ready"sv;
        return std::nullopt;
      }

      CMFormatDescriptionRef format_desc = CMSampleBufferGetFormatDescription(sample);
      if (!format_desc) {
        BOOST_LOG(error) << "VideoToolbox: output sample has no format description"sv;
        return std::nullopt;
      }

      size_t param_count = 0;
      int nal_length_bytes = 4;
      auto status = get_parameter_set_info(codec_type, format_desc, param_count, nal_length_bytes);
      if (status != noErr &&
          status != kCMFormatDescriptionBridgeError_InvalidParameter &&
          status != kCMFormatDescriptionError_InvalidParameter) {
        log_osstatus(error, "getting VideoToolbox NAL length size"sv, status);
        return std::nullopt;
      }

      std::vector<std::uint8_t> data;
      if (auto block = CMSampleBufferGetDataBuffer(sample)) {
        data.reserve(CMBlockBufferGetDataLength(block) + 128);
      }

      const auto keyframe = sample_is_keyframe(sample);
      if (keyframe && !append_parameter_sets(codec_type, format_desc, data, nal_length_bytes)) {
        return std::nullopt;
      }

      if (!append_sample_nals(sample, nal_length_bytes, data)) {
        return std::nullopt;
      }

      if (codec_type == kCMVideoCodecType_H264) {
        raise_h264_slice_priority(data);
      }

      return vt_encoded_frame_t {std::move(data), frame_index, keyframe};
    }

    class native_vt_encode_session_impl_t: public native_vt_encode_session_t {
    public:
      native_vt_encode_session_impl_t(
        config_t config,
        sunshine_colorspace_t colorspace,
        std::unique_ptr<platf::avcodec_encode_device_t> encode_device
      ):
          config {config},
          colorspace {colorspace},
          encode_device {std::move(encode_device)} {
        auto fps = config.framerateX100 > 0 ? framerateX100_to_rational(config.framerateX100) : AVRational {config.framerate, 1};
        fps_num = fps.num;
        fps_den = fps.den;
        codec_type = codec_type_from_config(config);
      }

      ~native_vt_encode_session_impl_t() override {
        if (session) {
          VTCompressionSessionCompleteFrames(session, kCMTimeInvalid);
          VTCompressionSessionInvalidate(session);
          CFRelease(session);
        }

        release_pixel_buffer(pixel_buffer);

        for (auto &sample : output_queue) {
          release_sample(sample.sample);
        }
      }

      bool init(platf::display_t *disp) {
        if (!validate_config(disp)) {
          return false;
        }

        if (auto *nv12_device = dynamic_cast<platf::nv12_zero_device *>(encode_device.get())) {
          nv12_device->set_resolution(config.width, config.height);
        } else {
          BOOST_LOG(error) << "VideoToolbox: native encoder requires NV12/P010 capture buffers"sv;
          return false;
        }

        CFMutableDictionaryRef encoder_spec = create_encoder_spec();
        CFMutableDictionaryRef source_attrs = create_source_attrs(config);

        auto status = VTCompressionSessionCreate(
          kCFAllocatorDefault,
          config.width,
          config.height,
          codec_type,
          encoder_spec,
          source_attrs,
          nullptr,
          compression_output_callback,
          this,
          &session
        );

        CFRelease(encoder_spec);
        CFRelease(source_attrs);

        if (status != noErr) {
          log_osstatus(error, "creating native VideoToolbox compression session"sv, status);
          return false;
        }

        if (!verify_hardware_mode()) {
          return false;
        }

        if (!configure_session()) {
          return false;
        }

        status = VTCompressionSessionPrepareToEncodeFrames(session);
        if (status != noErr) {
          log_osstatus(error, "preparing native VideoToolbox compression session"sv, status);
          return false;
        }

        BOOST_LOG(info) << "VideoToolbox: using native "sv << codec_name(codec_type) << (using_hardware ? " hardware"sv : " software"sv) << " encoder"sv;
        return true;
      }

      int convert(platf::img_t &img) override {
        auto *av_img = dynamic_cast<platf::av_img_t *>(&img);
        if (!av_img || !av_img->pixel_buffer || !av_img->pixel_buffer->buf) {
          BOOST_LOG(error) << "VideoToolbox: native encoder received an incompatible frame"sv;
          return -1;
        }

        auto new_pixel_buffer = av_img->pixel_buffer->buf;
        CVPixelBufferRetain(new_pixel_buffer);
        release_pixel_buffer(pixel_buffer);
        pixel_buffer = new_pixel_buffer;

        apply_pixel_buffer_attachments(pixel_buffer);
        return 0;
      }

      void request_idr_frame() override {
        force_idr = true;
      }

      void request_normal_frame() override {
        force_idr = false;
      }

      void invalidate_ref_frames(int64_t first_frame, int64_t last_frame) override {
        BOOST_LOG(error) << "VideoToolbox: native encoder does not support reference frame invalidation"sv;
        request_idr_frame();
      }

      std::optional<vt_encoded_frame_t> encode_frame(int64_t frame_index) override {
        if (!session || !pixel_buffer) {
          failed = true;
          return std::nullopt;
        }

        CFMutableDictionaryRef frame_props = nullptr;
        if (force_idr) {
          frame_props = CFDictionaryCreateMutable(kCFAllocatorDefault, 1, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
          CFDictionarySetValue(frame_props, kVTEncodeFrameOptionKey_ForceKeyFrame, kCFBooleanTrue);
        }

        VTEncodeInfoFlags info_flags = 0;
        const auto pts = CMTimeMake(frame_index * fps_den, fps_num);
        const auto duration = CMTimeMake(fps_den, fps_num);
        auto status = VTCompressionSessionEncodeFrame(
          session,
          pixel_buffer,
          pts,
          duration,
          frame_props,
          reinterpret_cast<void *>(static_cast<intptr_t>(frame_index)),
          &info_flags
        );

        if (frame_props) {
          CFRelease(frame_props);
        }

        if (status != noErr) {
          log_osstatus(error, "encoding native VideoToolbox frame"sv, status);
          failed = true;
          return std::nullopt;
        }

        status = VTCompressionSessionCompleteFrames(session, pts);
        if (status != noErr) {
          log_osstatus(error, "completing native VideoToolbox frame"sv, status);
          failed = true;
          return std::nullopt;
        }

        force_idr = false;

        std::unique_lock lock {output_mutex};
        if (async_status != noErr) {
          log_osstatus(error, "native VideoToolbox async encode"sv, async_status);
          async_status = noErr;
          failed = true;
          return std::nullopt;
        }

        if (output_queue.empty()) {
          return std::nullopt;
        }

        auto sample = output_queue.front();
        output_queue.pop_front();
        lock.unlock();

        auto packet = sample_to_annex_b(codec_type, sample.sample, sample.frame_index);
        if (!packet) {
          failed = true;
        }
        release_sample(sample.sample);
        return packet;
      }

      bool has_failed() const override {
        return failed;
      }

    private:
      struct encoded_sample_t {
        CMSampleBufferRef sample;
        int64_t frame_index;
      };

      static void compression_output_callback(
        void *output_callback_refcon,
        void *source_frame_refcon,
        OSStatus status,
        VTEncodeInfoFlags info_flags,
        CMSampleBufferRef sample_buffer
      ) {
        auto *session = static_cast<native_vt_encode_session_impl_t *>(output_callback_refcon);
        session->handle_output(source_frame_refcon, status, info_flags, sample_buffer);
      }

      void handle_output(void *source_frame_refcon, OSStatus status, VTEncodeInfoFlags info_flags, CMSampleBufferRef sample_buffer) {
        std::lock_guard lock {output_mutex};
        if (status != noErr) {
          async_status = status;
          return;
        }

        if ((info_flags & kVTEncodeInfo_FrameDropped) || !sample_buffer) {
          return;
        }

        CFRetain(sample_buffer);
        output_queue.push_back(encoded_sample_t {
          sample_buffer,
          static_cast<int64_t>(reinterpret_cast<intptr_t>(source_frame_refcon)),
        });
      }

      bool validate_config(platf::display_t *disp) {
        if (config.chromaSamplingType == 1) {
          BOOST_LOG(error) << "VideoToolbox: native encoder does not support YUV 4:4:4"sv;
          return false;
        }

        if (codec_type == kCMVideoCodecType_AV1) {
          BOOST_LOG(error) << "VideoToolbox: native AV1 encode is not implemented in Sunshine yet"sv;
          return false;
        }

        if (codec_type != kCMVideoCodecType_H264 && codec_type != kCMVideoCodecType_HEVC) {
          BOOST_LOG(error) << "VideoToolbox: unsupported codec type"sv;
          return false;
        }

        if (codec_type == kCMVideoCodecType_H264 && config.dynamicRange) {
          BOOST_LOG(error) << "VideoToolbox: H.264 10-bit encode is not supported"sv;
          return false;
        }

        if (codec_type == kCMVideoCodecType_H264 && config.numRefFrames > 0) {
          BOOST_LOG(error) << "VideoToolbox: H.264 reference frame restriction is disabled to avoid all-IDR output on Apple Silicon"sv;
          return false;
        }

        if (config.width <= 0 || config.height <= 0 || config.framerate <= 0) {
          BOOST_LOG(error) << "VideoToolbox: invalid stream dimensions or frame rate"sv;
          return false;
        }

        if (!disp) {
          BOOST_LOG(error) << "VideoToolbox: missing display"sv;
          return false;
        }

        return true;
      }

      bool verify_hardware_mode() {
        CFBooleanRef hardware_ref = nullptr;
        auto status = VTSessionCopyProperty(session, kVTCompressionPropertyKey_UsingHardwareAcceleratedVideoEncoder, kCFAllocatorDefault, &hardware_ref);
        using_hardware = status == noErr && hardware_ref && CFBooleanGetValue(hardware_ref);

        if (hardware_ref) {
          CFRelease(hardware_ref);
        }

        if (config::video.vt.vt_require_sw && using_hardware) {
          BOOST_LOG(error) << "VideoToolbox: software encoding was required, but a hardware session was created"sv;
          return false;
        }

        if (!config::video.vt.vt_allow_sw && !config::video.vt.vt_require_sw && !using_hardware) {
          BOOST_LOG(error) << "VideoToolbox: hardware encoding was required, but VideoToolbox created a software session"sv;
          return false;
        }

        return true;
      }

      bool configure_session() {
        auto profile = profile_from_config(codec_type, config);
        if (profile) {
          auto status = VTSessionSetProperty(session, kVTCompressionPropertyKey_ProfileLevel, profile);
          if (status != noErr) {
            log_osstatus(error, "setting VideoToolbox profile"sv, status);
            return false;
          }
        }

        if (auto status = session_set_bool(session, kVTCompressionPropertyKey_AllowFrameReordering, false); status != noErr) {
          log_osstatus(error, "disabling VideoToolbox frame reordering"sv, status);
          return false;
        }

        if (auto status = session_set_i32(session, kVTCompressionPropertyKey_ExpectedFrameRate, std::max(1, fps_num / fps_den)); status != noErr) {
          log_osstatus(warning, "setting VideoToolbox expected frame rate"sv, status);
        }

        if (auto status = session_set_i32(session, kVTCompressionPropertyKey_MaxKeyFrameInterval, std::numeric_limits<int32_t>::max()); status != noErr) {
          log_osstatus(warning, "setting VideoToolbox max keyframe interval"sv, status);
        }

        if (config.numRefFrames > 0 && codec_type == kCMVideoCodecType_HEVC) {
          if (__builtin_available(macOS 13.0, *)) {
            if (auto status = session_set_i32(session, kVTCompressionPropertyKey_ReferenceBufferCount, config.numRefFrames); status != noErr) {
              log_osstatus(error, "setting VideoToolbox reference buffer count"sv, status);
              return false;
            }
          } else {
            BOOST_LOG(error) << "VideoToolbox: reference frame restriction requires macOS 13 or newer"sv;
            return false;
          }
        }

        if (auto status = set_bitrate(); status != noErr) {
          log_osstatus(error, "setting VideoToolbox bitrate"sv, status);
          return false;
        }

        if (__builtin_available(macOS 11.0, *)) {
          if (auto status = session_set_bool(session, kVTCompressionPropertyKey_PrioritizeEncodingSpeedOverQuality, true); status != noErr) {
            log_osstatus(warning, "prioritizing VideoToolbox encoding speed"sv, status);
          }
        }

        if (auto status = session_set_bool(session, kVTCompressionPropertyKey_RealTime, config::video.vt.vt_realtime != 0); status != noErr) {
          log_osstatus(warning, "setting VideoToolbox realtime mode"sv, status);
        }

        if (codec_type == kCMVideoCodecType_H264 && config::video.vt.vt_coder == 2) {
          if (auto status = VTSessionSetProperty(session, kVTCompressionPropertyKey_H264EntropyMode, kVTH264EntropyMode_CAVLC); status != noErr) {
            log_osstatus(warning, "setting VideoToolbox H.264 entropy mode"sv, status);
          }
        } else if (codec_type == kCMVideoCodecType_H264 && config::video.vt.vt_coder == 1) {
          if (auto status = VTSessionSetProperty(session, kVTCompressionPropertyKey_H264EntropyMode, kVTH264EntropyMode_CABAC); status != noErr) {
            log_osstatus(warning, "setting VideoToolbox H.264 entropy mode"sv, status);
          }
        }

        return set_colorspace();
      }

      OSStatus set_bitrate() {
        const auto bitrate = ((config::video.max_bitrate > 0) ? std::min(config.bitrate, config::video.max_bitrate) : config.bitrate) * 1000;

        if (__builtin_available(macOS 13.0, *)) {
          if (using_hardware && !config::video.vt.vt_require_sw) {
            auto status = session_set_i32(session, kVTCompressionPropertyKey_ConstantBitRate, bitrate);
            if (status == noErr) {
              return noErr;
            }

            log_osstatus(warning, "setting VideoToolbox constant bitrate, falling back to average bitrate"sv, status);
          }
        }

        auto status = session_set_i32(session, kVTCompressionPropertyKey_AverageBitRate, bitrate);
        if (status != noErr) {
          return status;
        }

        const int32_t bytes_per_second = std::max(1, bitrate / 8);
        const double one_second = 1.0;
        CFNumberRef data_rate = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &bytes_per_second);
        CFNumberRef duration = CFNumberCreate(kCFAllocatorDefault, kCFNumberDoubleType, &one_second);
        CFTypeRef values[] {data_rate, duration};
        CFArrayRef limits = CFArrayCreate(kCFAllocatorDefault, values, 2, &kCFTypeArrayCallBacks);

        status = VTSessionSetProperty(session, kVTCompressionPropertyKey_DataRateLimits, limits);

        CFRelease(data_rate);
        CFRelease(duration);
        CFRelease(limits);

        return status == kVTPropertyNotSupportedErr ? noErr : status;
      }

      bool set_colorspace() {
        CFTypeRef keys[] {
          kVTCompressionPropertyKey_ColorPrimaries,
          kVTCompressionPropertyKey_TransferFunction,
          kVTCompressionPropertyKey_YCbCrMatrix,
        };
        CFTypeRef values[] {
          vt_primaries(colorspace),
          vt_transfer(colorspace),
          vt_matrix(colorspace),
        };

        CFDictionaryRef props = CFDictionaryCreate(kCFAllocatorDefault, keys, values, 3, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
        auto status = VTSessionSetProperties(session, props);
        CFRelease(props);

        if (status != noErr) {
          log_osstatus(error, "setting VideoToolbox color properties"sv, status);
          return false;
        }

        return true;
      }

      void apply_pixel_buffer_attachments(CVPixelBufferRef buffer) {
        CVBufferSetAttachment(buffer, kCVImageBufferColorPrimariesKey, vt_primaries(colorspace), kCVAttachmentMode_ShouldPropagate);
        CVBufferSetAttachment(buffer, kCVImageBufferTransferFunctionKey, vt_transfer(colorspace), kCVAttachmentMode_ShouldPropagate);
        CVBufferSetAttachment(buffer, kCVImageBufferYCbCrMatrixKey, vt_matrix(colorspace), kCVAttachmentMode_ShouldPropagate);
      }

      config_t config;
      sunshine_colorspace_t colorspace;
      std::unique_ptr<platf::avcodec_encode_device_t> encode_device;

      VTCompressionSessionRef session {};
      CVPixelBufferRef pixel_buffer {};
      CMVideoCodecType codec_type {};
      int fps_num {};
      int fps_den {};
      bool force_idr {};
      bool using_hardware {};
      bool failed {};

      std::mutex output_mutex;
      std::deque<encoded_sample_t> output_queue;
      OSStatus async_status {noErr};
    };

  }  // namespace

  std::unique_ptr<encode_session_t> make_vt_native_encode_session(
    platf::display_t *disp,
    const encoder_t &encoder,
    const config_t &config,
    int width,
    int height,
    std::unique_ptr<platf::avcodec_encode_device_t> encode_device
  ) {
    (void) width;
    (void) height;

    auto &video_format = encoder.codec_from_config(config);
    if (!video_format[encoder_t::PASSED]) {
      BOOST_LOG(error) << "VideoToolbox: "sv << video_format.name << " mode not supported"sv;
      return nullptr;
    }

    auto colorspace = encode_device->colorspace;
    auto session = std::make_unique<native_vt_encode_session_impl_t>(config, colorspace, std::move(encode_device));
    if (!session->init(disp)) {
      return nullptr;
    }

    return session;
  }

}  // namespace video
