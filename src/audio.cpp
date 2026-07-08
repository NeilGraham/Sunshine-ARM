/**
 * @file src/audio.cpp
 * @brief Definitions for audio capture and encoding.
 */
// standard includes
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>
#include <thread>
#include <utility>

// lib includes
#include <opus/opus_multistream.h>

// local includes
#include "audio.h"
#include "audio_gate.h"
#include "config.h"
#include "globals.h"
#include "logging.h"
#include "platform/common.h"
#include "thread_safe.h"
#include "utility.h"

namespace audio {
  using namespace std::literals;
  using opus_t = util::safe_ptr<OpusMSEncoder, opus_multistream_encoder_destroy>;
  using sample_queue_t = std::shared_ptr<safe::queue_t<std::vector<float>>>;

  static int start_audio_control(audio_ctx_t &ctx);
  static void stop_audio_control(audio_ctx_t &);
  static void apply_surround_params(opus_stream_config_t &stream, const stream_params_t &params);

  int map_stream(int channels, bool quality);

  constexpr auto SAMPLE_RATE = 48000;

  // NOTE: If you adjust the bitrates listed here, make sure to update the
  // corresponding bitrate adjustment logic in rtsp_stream::cmd_announce()
  opus_stream_config_t stream_configs[MAX_STREAM_CONFIG] {
    {
      SAMPLE_RATE,
      2,
      1,
      1,
      platf::speaker::map_stereo,
      96000,
    },
    {
      SAMPLE_RATE,
      2,
      1,
      1,
      platf::speaker::map_stereo,
      512000,
    },
    {
      SAMPLE_RATE,
      6,
      4,
      2,
      platf::speaker::map_surround51,
      256000,
    },
    {
      SAMPLE_RATE,
      6,
      6,
      0,
      platf::speaker::map_surround51,
      1536000,
    },
    {
      SAMPLE_RATE,
      8,
      5,
      3,
      platf::speaker::map_surround71,
      450000,
    },
    {
      SAMPLE_RATE,
      8,
      8,
      0,
      platf::speaker::map_surround71,
      2048000,
    },
  };

  void encodeThread(sample_queue_t samples, config_t config, void *channel_data) {
    auto packets = mail::man->queue<packet_t>(mail::audio_packets);
    auto stream = stream_configs[map_stream(config.channels, config.flags[config_t::HIGH_QUALITY])];
    if (config.flags[config_t::CUSTOM_SURROUND_PARAMS]) {
      apply_surround_params(stream, config.customStreamParams);
    }

    // Encoding takes place on this thread
    platf::set_thread_name("audio::encode");
    platf::adjust_thread_priority(platf::thread_priority_e::high);

    opus_t opus {opus_multistream_encoder_create(
      stream.sampleRate,
      stream.channelCount,
      stream.streams,
      stream.coupledStreams,
      stream.mapping,
      OPUS_APPLICATION_RESTRICTED_LOWDELAY,
      nullptr
    )};

    opus_multistream_encoder_ctl(opus.get(), OPUS_SET_BITRATE(stream.bitrate));
    opus_multistream_encoder_ctl(opus.get(), OPUS_SET_VBR(0));

    BOOST_LOG(info) << "Opus initialized: "sv << stream.sampleRate / 1000 << " kHz, "sv
                    << stream.channelCount << " channels, "sv
                    << stream.bitrate / 1000 << " kbps (total), LOWDELAY"sv;

    auto frame_size = config.packetDuration * stream.sampleRate / 1000;
    while (auto sample = samples->pop()) {
      buffer_t packet {1400};

      int bytes = opus_multistream_encode_float(opus.get(), sample->data(), frame_size, std::begin(packet), (opus_int32) packet.size());
      if (bytes < 0) {
        BOOST_LOG(error) << "Couldn't encode audio: "sv << opus_strerror(bytes);
        packets->stop();

        return;
      }

      packet.fake_resize(bytes);
      packets->raise(channel_data, std::move(packet));
    }
  }

  void capture(safe::mail_t mail, config_t config, void *channel_data) {
    auto shutdown_event = mail->event<bool>(mail::shutdown);
    if (!config::audio.stream) {
      shutdown_event->view();
      return;
    }
    auto stream = stream_configs[map_stream(config.channels, config.flags[config_t::HIGH_QUALITY])];
    if (config.flags[config_t::CUSTOM_SURROUND_PARAMS]) {
      apply_surround_params(stream, config.customStreamParams);
    }

    auto ref = get_audio_ctx_ref();
    if (!ref) {
      return;
    }

    auto init_failure_fg = util::fail_guard([&shutdown_event]() {
      BOOST_LOG(error) << "Unable to initialize audio capture. The stream will not have audio."sv;

      // Wait for shutdown to be signalled if we fail init.
      // This allows streaming to continue without audio.
      shutdown_event->view();
    });

    auto &control = ref->control;
    if (!control) {
      return;
    }

    // Order of priority:
    // 1. Virtual sink
    // 2. Audio sink
    // 3. Host
    std::string *sink = &ref->sink.host;
    if (!config::audio.sink.empty()) {
      sink = &config::audio.sink;
    }

    // Prefer the virtual sink if host playback is disabled or there's no other sink
    if (ref->sink.null && (!config.flags[config_t::HOST_AUDIO] || sink->empty())) {
      auto &null = *ref->sink.null;
      switch (stream.channelCount) {
        case 2:
          sink = &null.stereo;
          break;
        case 6:
          sink = &null.surround51;
          break;
        case 8:
          sink = &null.surround71;
          break;
      }
    }

    // Only the first to start a session may change the default sink
    if (!ref->sink_flag->exchange(true, std::memory_order_acquire)) {
      // If the selected sink is different than the current one, change sinks.
      ref->restore_sink = ref->sink.host != *sink;
      if (ref->restore_sink) {
        if (control->set_sink(*sink)) {
          return;
        }
      }
    }

    auto frame_size = config.packetDuration * stream.sampleRate / 1000;
    bool host_audio = config.flags[config_t::HOST_AUDIO];
    bool continuous_audio = config.flags[config_t::CONTINUOUS_AUDIO];
    auto mic = control->microphone(stream.mapping, stream.channelCount, stream.sampleRate, frame_size, continuous_audio, host_audio);
    if (!mic) {
      return;
    }

    // Audio is initialized, so we don't want to print the failure message
    init_failure_fg.disable();

    // Capture takes place on this thread
    platf::adjust_thread_priority(platf::thread_priority_e::critical);

    auto samples = std::make_shared<sample_queue_t::element_type>(30);
    std::thread thread {encodeThread, samples, config, channel_data};

    auto fg = util::fail_guard([&]() {
      samples->stop();
      thread.join();

      shutdown_event->view();
    });

    int samples_per_frame = frame_size * stream.channelCount;

    // --- Audio gate: squelch the HDMI-RX FIFO-dump click (see audio_gate.h) ---
    // The rk_hdmirx audio FIFO re-inits on every source re-lock and dumps a
    // full-scale burst (measured peak ~0.97 FS). rkmpp.cpp arms audio_gate for
    // the transitions it can see; here we ramp/zero the PCM across an armed
    // gate and self-detect same-geometry re-inits rkmpp never sees.
    //
    // Env tunables, read once at capture start:
    //   SUNSHINE_AUDIO_GATE     off/0 disables the whole gate (default on)
    //   SUNSHINE_AUDIO_GATE_MS  reneg-trigger squelch duration (default 500)
    //   SUNSHINE_AUDIO_TAP=path append post-gate audio as s16le (debug)
    const auto env_val = [](const char *name) -> const char * {
      const char *v = std::getenv(name);
      return (v && *v) ? v : nullptr;
    };
    bool gate_enabled = true;
    if (const char *v = env_val("SUNSHINE_AUDIO_GATE")) {
      gate_enabled = !(std::strcmp(v, "off") == 0 || std::strcmp(v, "0") == 0);
    }
    if (const char *v = env_val("SUNSHINE_AUDIO_GATE_MS")) {
      int ms = std::atoi(v);
      if (ms > 0) {
        audio_gate::reneg_ms.store(ms, std::memory_order_relaxed);
      }
    }
    std::FILE *tap = nullptr;
    if (const char *path = env_val("SUNSHINE_AUDIO_TAP")) {
      tap = std::fopen(path, "ab");
      BOOST_LOG(info) << "audio gate: post-gate tap "sv << (tap ? "open -> "sv : "FAILED to open "sv) << path;
    }
    BOOST_LOG(info) << "audio gate: "sv << (gate_enabled ? "enabled"sv : "disabled"sv)
                    << " (reneg squelch "sv << audio_gate::reneg_ms.load(std::memory_order_relaxed) << " ms)"sv;

    // ~10 ms raised-cosine edge ramp, applied only at gate entry/exit; the
    // middle of an armed gate is hard silence. gate_phase in [0,1] is
    // "openness" and the applied gain is a raised cosine of it.
    const int ramp_frames = std::max(1, stream.sampleRate / 100);  // 10 ms of per-channel frames
    const double ramp_step = 1.0 / ramp_frames;
    double gate_phase = 1.0;  // fully open
    const int channels = std::max(1, stream.channelCount);

    // Rolling ~200 ms RMS across frames for the squelch heuristic.
    const int rms_window_frames = std::max(1, 200 / std::max(1, config.packetDuration));
    std::deque<std::pair<double, long long>> rms_ring;  // (sum-of-squares, sample count) per frame
    double rms_sumsq = 0.0;
    long long rms_count = 0;
    std::uint64_t squelch_count = 0;

    auto fg_tap = util::fail_guard([&tap]() {
      if (tap) {
        std::fclose(tap);
      }
    });

    while (!shutdown_event->peek()) {
      std::vector<float> sample_buffer;
      sample_buffer.resize(samples_per_frame);

      auto status = mic->sample(sample_buffer);
      switch (status) {
        case platf::capture_e::ok:
          break;
        case platf::capture_e::timeout:
          continue;
        case platf::capture_e::reinit:
          BOOST_LOG(info) << "Reinitializing audio capture"sv;
          mic.reset();
          do {
            mic = control->microphone(stream.mapping, stream.channelCount, stream.sampleRate, frame_size, continuous_audio, host_audio);
            if (!mic) {
              BOOST_LOG(warning) << "Couldn't re-initialize audio input"sv;
            }
          } while (!mic && !shutdown_event->view(5s));
          continue;
        default:
          return;
      }

      if (gate_enabled) {
        // Per-frame stats (from the raw, pre-gate samples).
        float peak = 0.0f;
        double frame_sumsq = 0.0;
        for (float s : sample_buffer) {
          const float a = std::fabs(s);
          if (a > peak) {
            peak = a;
          }
          frame_sumsq += static_cast<double>(s) * s;
        }

        bool gate_now = audio_gate::active();

        // Squelch heuristic: a near-silent recent window (< ~-60 dBFS) with a
        // sudden full-scale spike is the FIFO-dump click on a re-init rkmpp
        // never sees (e.g. a same-geometry menu->Apollo relock). Evaluate the
        // rolling RMS over PRIOR frames (before folding in this one) so the
        // spike can't mask its own quiet lead-in.
        const double rolling_rms = rms_count > 0 ? std::sqrt(rms_sumsq / static_cast<double>(rms_count)) : 0.0;
        bool heuristic_zero = false;
        if (!gate_now && rolling_rms < 1e-3 && peak > 0.97f) {
          audio_gate::trigger(300);
          gate_now = true;
          heuristic_zero = true;  // sacrifice this whole click frame
          gate_phase = 0.0;       // jump closed: a smooth fade can't outrun the click already in-frame
          ++squelch_count;
          BOOST_LOG(info) << "audio gate: squelch triggered (n="sv << squelch_count
                          << ", peak="sv << peak << ", rolling_rms="sv << rolling_rms << ")"sv;
        }

        // Advance the rolling RMS window with this frame's energy.
        rms_ring.emplace_back(frame_sumsq, static_cast<long long>(sample_buffer.size()));
        rms_sumsq += frame_sumsq;
        rms_count += static_cast<long long>(sample_buffer.size());
        while (static_cast<int>(rms_ring.size()) > rms_window_frames) {
          rms_sumsq -= rms_ring.front().first;
          rms_count -= rms_ring.front().second;
          rms_ring.pop_front();
        }

        // Apply the envelope. A heuristic hit hard-zeros the frame; otherwise
        // ramp gate_phase toward the target (0 closed / 1 open) one step per
        // per-channel frame, with a raised-cosine gain. Untouched when fully
        // open and no gate is armed.
        if (heuristic_zero) {
          std::fill(sample_buffer.begin(), sample_buffer.end(), 0.0f);
        } else if (gate_now || gate_phase < 1.0) {
          const double target = gate_now ? 0.0 : 1.0;
          for (size_t i = 0; i < sample_buffer.size(); i += channels) {
            if (gate_phase < target) {
              gate_phase = std::min(target, gate_phase + ramp_step);
            } else if (gate_phase > target) {
              gate_phase = std::max(target, gate_phase - ramp_step);
            }
            const float gain = static_cast<float>(0.5 * (1.0 - std::cos(M_PI * gate_phase)));
            for (int c = 0; c < channels && i + static_cast<size_t>(c) < sample_buffer.size(); ++c) {
              sample_buffer[i + c] *= gain;
            }
          }
        }
      }

      if (tap) {
        // The only observation point downstream of the gate. s16le, clamped;
        // best-effort, never fails capture.
        std::vector<std::int16_t> pcm;
        pcm.reserve(sample_buffer.size());
        for (float s : sample_buffer) {
          float v = s * 32767.0f;
          v = std::min(32767.0f, std::max(-32768.0f, v));
          pcm.push_back(static_cast<std::int16_t>(std::lrintf(v)));
        }
        std::fwrite(pcm.data(), sizeof(std::int16_t), pcm.size(), tap);
      }

      samples->raise(std::move(sample_buffer));
    }
  }

  audio_ctx_ref_t get_audio_ctx_ref() {
    static auto control_shared {safe::make_shared<audio_ctx_t>(start_audio_control, stop_audio_control)};
    return control_shared.ref();
  }

  bool is_audio_ctx_sink_available(const audio_ctx_t &ctx) {
    if (!ctx.control) {
      return false;
    }

    const std::string &sink = ctx.sink.host.empty() ? config::audio.sink : ctx.sink.host;
    if (sink.empty()) {
      return false;
    }

    return ctx.control->is_sink_available(sink);
  }

  int map_stream(int channels, bool quality) {
    int shift = quality ? 1 : 0;
    switch (channels) {
      case 2:
        return STEREO + shift;
      case 6:
        return SURROUND51 + shift;
      case 8:
        return SURROUND71 + shift;
    }
    return STEREO;
  }

  int start_audio_control(audio_ctx_t &ctx) {
    auto fg = util::fail_guard([]() {
      BOOST_LOG(warning) << "There will be no audio"sv;
    });

    ctx.sink_flag = std::make_unique<std::atomic_bool>(false);

    // The default sink has not been replaced yet.
    ctx.restore_sink = false;

    if (!(ctx.control = platf::audio_control())) {
      return 0;
    }

    auto sink = ctx.control->sink_info();
    if (!sink) {
      // Let the calling code know it failed
      ctx.control.reset();
      return 0;
    }

    ctx.sink = std::move(*sink);

    fg.disable();
    return 0;
  }

  void stop_audio_control(audio_ctx_t &ctx) {
    // restore audio-sink if applicable
    if (!ctx.restore_sink) {
      return;
    }

    // Change back to the host sink, unless there was none
    const std::string &sink = ctx.sink.host.empty() ? config::audio.sink : ctx.sink.host;
    if (!sink.empty()) {
      // Best effort, it's allowed to fail
      ctx.control->set_sink(sink);
    }
  }

  void apply_surround_params(opus_stream_config_t &stream, const stream_params_t &params) {
    stream.channelCount = params.channelCount;
    stream.streams = params.streams;
    stream.coupledStreams = params.coupledStreams;
    stream.mapping = params.mapping;
  }
}  // namespace audio
