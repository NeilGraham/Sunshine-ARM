/**
 * @file tests/unit/platform/macos/test_vt_encoder.mm
 * @brief Native VideoToolbox hardware encoder smoke tests.
 */

#include "../../../tests_common.h"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <CoreVideo/CoreVideo.h>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>
#include <VideoToolbox/VideoToolbox.h>

namespace {

  struct sample_result_t {
    std::mutex mutex;
    std::condition_variable cv;
    std::vector<CMSampleBufferRef> samples;
    OSStatus status = noErr;
    bool failed = false;

    ~sample_result_t() {
      for (auto sample : samples) {
        CFRelease(sample);
      }
    }
  };

  void encode_callback(void *output_callback_refcon, void *, OSStatus status, VTEncodeInfoFlags, CMSampleBufferRef sample_buffer) {
    auto &result = *static_cast<sample_result_t *>(output_callback_refcon);

    {
      std::lock_guard lock {result.mutex};
      result.status = status;
      result.failed = status != noErr;
      if (status == noErr && sample_buffer != nullptr) {
        CFRetain(sample_buffer);
        result.samples.emplace_back(sample_buffer);
      }
    }

    result.cv.notify_one();
  }

  CFNumberRef cf_number(std::int32_t value) {
    return CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &value);
  }

  class cf_dictionary_t {
  public:
    cf_dictionary_t():
        dictionary {CFDictionaryCreateMutable(kCFAllocatorDefault, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks)} {}

    ~cf_dictionary_t() {
      if (dictionary != nullptr) {
        CFRelease(dictionary);
      }
    }

    cf_dictionary_t(const cf_dictionary_t &) = delete;
    cf_dictionary_t &operator=(const cf_dictionary_t &) = delete;

    operator CFMutableDictionaryRef() const {
      return dictionary;
    }

  private:
    CFMutableDictionaryRef dictionary;
  };

  class cf_number_t {
  public:
    explicit cf_number_t(std::int32_t value):
        number {cf_number(value)} {}

    ~cf_number_t() {
      if (number != nullptr) {
        CFRelease(number);
      }
    }

    cf_number_t(const cf_number_t &) = delete;
    cf_number_t &operator=(const cf_number_t &) = delete;

    operator CFNumberRef() const {
      return number;
    }

  private:
    CFNumberRef number;
  };

  class vt_session_t {
  public:
    vt_session_t() = default;

    ~vt_session_t() {
      if (session != nullptr) {
        VTCompressionSessionInvalidate(session);
        CFRelease(session);
      }
    }

    vt_session_t(const vt_session_t &) = delete;
    vt_session_t &operator=(const vt_session_t &) = delete;

    VTCompressionSessionRef *out() {
      return &session;
    }

    operator VTCompressionSessionRef() const {
      return session;
    }

  private:
    VTCompressionSessionRef session = nullptr;
  };

  class pixel_buffer_t {
  public:
    pixel_buffer_t() = default;

    ~pixel_buffer_t() {
      if (buffer != nullptr) {
        CVPixelBufferRelease(buffer);
      }
    }

    pixel_buffer_t(const pixel_buffer_t &) = delete;
    pixel_buffer_t &operator=(const pixel_buffer_t &) = delete;

    CVPixelBufferRef *out() {
      return &buffer;
    }

    operator CVPixelBufferRef() const {
      return buffer;
    }

  private:
    CVPixelBufferRef buffer = nullptr;
  };

  struct encode_case_t {
    CMVideoCodecType codec;
    OSType pixel_format;
    CFStringRef profile;
    std::string name;
  };

  void fill_pixel_buffer(CVPixelBufferRef pixel_buffer) {
    CVPixelBufferLockBaseAddress(pixel_buffer, 0);

    const auto pixel_format = CVPixelBufferGetPixelFormatType(pixel_buffer);
    const auto height = CVPixelBufferGetHeight(pixel_buffer);
    const auto width = CVPixelBufferGetWidth(pixel_buffer);

    if (pixel_format == kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange) {
      auto *luma = static_cast<std::uint8_t *>(CVPixelBufferGetBaseAddressOfPlane(pixel_buffer, 0));
      auto luma_stride = CVPixelBufferGetBytesPerRowOfPlane(pixel_buffer, 0);
      for (std::size_t y = 0; y < height; ++y) {
        std::memset(luma + y * luma_stride, 0x10, width);
      }

      auto *chroma = static_cast<std::uint8_t *>(CVPixelBufferGetBaseAddressOfPlane(pixel_buffer, 1));
      auto chroma_stride = CVPixelBufferGetBytesPerRowOfPlane(pixel_buffer, 1);
      for (std::size_t y = 0; y < height / 2; ++y) {
        std::memset(chroma + y * chroma_stride, 0x80, width);
      }
    } else if (pixel_format == kCVPixelFormatType_420YpCbCr10BiPlanarVideoRange) {
      auto *luma = static_cast<std::uint16_t *>(CVPixelBufferGetBaseAddressOfPlane(pixel_buffer, 0));
      auto luma_stride = CVPixelBufferGetBytesPerRowOfPlane(pixel_buffer, 0) / sizeof(std::uint16_t);
      for (std::size_t y = 0; y < height; ++y) {
        std::fill(luma + y * luma_stride, luma + y * luma_stride + width, 64 << 6);
      }

      auto *chroma = static_cast<std::uint16_t *>(CVPixelBufferGetBaseAddressOfPlane(pixel_buffer, 1));
      auto chroma_stride = CVPixelBufferGetBytesPerRowOfPlane(pixel_buffer, 1) / sizeof(std::uint16_t);
      for (std::size_t y = 0; y < height / 2; ++y) {
        std::fill(chroma + y * chroma_stride, chroma + y * chroma_stride + width, 512 << 6);
      }
    }

    CVPixelBufferUnlockBaseAddress(pixel_buffer, 0);
  }

  void expect_parameter_sets(const encode_case_t &encode_case, CMFormatDescriptionRef format_description) {
    const std::uint8_t *parameter_set = nullptr;
    std::size_t parameter_set_size = 0;
    std::size_t parameter_set_count = 0;
    int nal_header_length = 0;

    OSStatus status = noErr;
    if (encode_case.codec == kCMVideoCodecType_H264) {
      status = CMVideoFormatDescriptionGetH264ParameterSetAtIndex(
        format_description,
        0,
        &parameter_set,
        &parameter_set_size,
        &parameter_set_count,
        &nal_header_length
      );
    } else {
      status = CMVideoFormatDescriptionGetHEVCParameterSetAtIndex(
        format_description,
        0,
        &parameter_set,
        &parameter_set_size,
        &parameter_set_count,
        &nal_header_length
      );
    }

    ASSERT_EQ(status, noErr) << encode_case.name << " parameter set extraction failed";
    ASSERT_NE(parameter_set, nullptr);
    EXPECT_GT(parameter_set_size, 0u);
    EXPECT_GE(parameter_set_count, 2u);
    EXPECT_EQ(nal_header_length, 4);
  }

  void require_native_hardware_encode(const encode_case_t &encode_case) {
    constexpr int width = 128;
    constexpr int height = 72;
    sample_result_t result;

    cf_dictionary_t encoder_spec;
    CFDictionarySetValue(
      encoder_spec,
      kVTVideoEncoderSpecification_RequireHardwareAcceleratedVideoEncoder,
      kCFBooleanTrue
    );

    cf_dictionary_t io_surface_properties;
    cf_dictionary_t source_attributes;
    cf_number_t pixel_format {static_cast<std::int32_t>(encode_case.pixel_format)};
    cf_number_t width_ref {width};
    cf_number_t height_ref {height};
    CFDictionarySetValue(source_attributes, kCVPixelBufferPixelFormatTypeKey, pixel_format);
    CFDictionarySetValue(source_attributes, kCVPixelBufferWidthKey, width_ref);
    CFDictionarySetValue(source_attributes, kCVPixelBufferHeightKey, height_ref);
    CFDictionarySetValue(source_attributes, kCVPixelBufferIOSurfacePropertiesKey, io_surface_properties);

    vt_session_t session;
    auto status = VTCompressionSessionCreate(
      kCFAllocatorDefault,
      width,
      height,
      encode_case.codec,
      encoder_spec,
      source_attributes,
      nullptr,
      encode_callback,
      &result,
      session.out()
    );
    if (status != noErr) {
      GTEST_SKIP() << encode_case.name << " hardware VideoToolbox session is unavailable: " << status;
    }

    ASSERT_EQ(VTSessionSetProperty(session, kVTCompressionPropertyKey_RealTime, kCFBooleanTrue), noErr);
    ASSERT_EQ(VTSessionSetProperty(session, kVTCompressionPropertyKey_AllowFrameReordering, kCFBooleanFalse), noErr);
    ASSERT_EQ(VTSessionSetProperty(session, kVTCompressionPropertyKey_ProfileLevel, encode_case.profile), noErr);

    CFTypeRef hardware_ref = nullptr;
    status = VTSessionCopyProperty(
      session,
      kVTCompressionPropertyKey_UsingHardwareAcceleratedVideoEncoder,
      nullptr,
      &hardware_ref
    );
    ASSERT_EQ(status, noErr);
    ASSERT_NE(hardware_ref, nullptr);
    ASSERT_EQ(CFGetTypeID(hardware_ref), CFBooleanGetTypeID());
    EXPECT_TRUE(CFBooleanGetValue(static_cast<CFBooleanRef>(hardware_ref))) << encode_case.name << " did not use hardware acceleration";
    CFRelease(hardware_ref);

    pixel_buffer_t pixel_buffer;
    status = CVPixelBufferCreate(
      kCFAllocatorDefault,
      width,
      height,
      encode_case.pixel_format,
      source_attributes,
      pixel_buffer.out()
    );
    ASSERT_EQ(status, kCVReturnSuccess);

    fill_pixel_buffer(pixel_buffer);

    cf_dictionary_t encode_options;
    CFDictionarySetValue(encode_options, kVTEncodeFrameOptionKey_ForceKeyFrame, kCFBooleanTrue);

    status = VTCompressionSessionEncodeFrame(
      session,
      pixel_buffer,
      CMTimeMake(0, 60),
      CMTimeMake(1, 60),
      encode_options,
      nullptr,
      nullptr
    );
    ASSERT_EQ(status, noErr);
    ASSERT_EQ(VTCompressionSessionCompleteFrames(session, CMTimeMake(0, 60)), noErr);

    std::unique_lock lock {result.mutex};
    ASSERT_TRUE(result.cv.wait_for(lock, std::chrono::seconds(5), [&result] {
      return result.failed || !result.samples.empty();
    }))
      << encode_case.name << " did not produce an encoded sample";
    ASSERT_FALSE(result.failed) << encode_case.name << " encode failed: " << result.status;
    ASSERT_FALSE(result.samples.empty());
    auto sample = result.samples.front();
    lock.unlock();

    ASSERT_TRUE(CMSampleBufferDataIsReady(sample));
    auto *format_description = CMSampleBufferGetFormatDescription(sample);
    ASSERT_NE(format_description, nullptr);
    expect_parameter_sets(encode_case, format_description);

    auto *block_buffer = CMSampleBufferGetDataBuffer(sample);
    ASSERT_NE(block_buffer, nullptr);
    EXPECT_GT(CMBlockBufferGetDataLength(block_buffer), 4u);
  }

}  // namespace

TEST(VideoToolboxHardwareEncoderTest, H264EncodesNV12Frame) {
  require_native_hardware_encode({
    kCMVideoCodecType_H264,
    kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange,
    kVTProfileLevel_H264_High_AutoLevel,
    "H.264",
  });
}

TEST(VideoToolboxHardwareEncoderTest, HEVCMain10EncodesP010Frame) {
  require_native_hardware_encode({
    kCMVideoCodecType_HEVC,
    kCVPixelFormatType_420YpCbCr10BiPlanarVideoRange,
    kVTProfileLevel_HEVC_Main10_AutoLevel,
    "HEVC Main10",
  });
}
