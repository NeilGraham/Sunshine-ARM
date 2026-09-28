/**
 * @file tests/unit/test_adaptive_bitrate.cpp
 * @brief Test src/adaptive_bitrate.* (the policy only; no threads, injected clock).
 */
#include "../tests_common.h"

#include <cmath>

#include <src/adaptive_bitrate.h>

using namespace std::literals;

namespace {
  constexpr std::int64_t mbps = 1'000'000;

  struct AdaptiveBitrateTest: ::testing::Test {
    abr::controller_t ctl;
    abr::clock::time_point t0 {abr::clock::now()};

    // Advance the clock frame by frame at 60 fps, running the policy.
    std::int64_t run(abr::clock::time_point &now, std::chrono::milliseconds span) {
      const auto end = now + span;
      std::int64_t target = ctl.target();
      while (now < end) {
        now += 16ms;
        target = ctl.update(now);
      }
      return target;
    }
  };
}  // namespace

TEST(AdaptiveBitrateMode, MoonlightDefaultCurve) {
  // moonlight-qt's own defaults: 1080p60 = 20 Mbps, 4K60 = 80 Mbps, 720p60 = 10.
  EXPECT_NEAR(abr::moonlight_default_kbps(1920, 1080, 60), 20000, 1);
  EXPECT_NEAR(abr::moonlight_default_kbps(3840, 2160, 60), 80000, 1);
  EXPECT_NEAR(abr::moonlight_default_kbps(1280, 720, 60), 10000, 1);
  // Above 60 fps the frame-rate factor grows with the square root.
  EXPECT_NEAR(abr::moonlight_default_kbps(3840, 2160, 120), 40 * std::sqrt(2.0) * 2 * 1000, 1);
}

TEST(AdaptiveBitrateMode, CeilingFollowsTheSourceMode) {
  // The PS5 case: 80 Mbps asked for 4K120, the console sends 4K 59.94.
  const auto ps5 = abr::mode_ceiling_bps(80 * mbps, 3840, 2160, 120, 3840, 2160, 60000.0 / 1001);
  EXPECT_NEAR((double) ps5 / mbps, 80 * (2 * 59.94 / 60) / (2 * std::sqrt(2.0)), 0.1);
  // The same mode as requested keeps the full bitrate, and a bigger or faster
  // mode never raises it.
  EXPECT_EQ(abr::mode_ceiling_bps(80 * mbps, 3840, 2160, 60, 3840, 2160, 60), 80 * mbps);
  EXPECT_EQ(abr::mode_ceiling_bps(20 * mbps, 1920, 1080, 60, 3840, 2160, 120), 20 * mbps);
  // A PS3 at 720p60 on that session: an eighth-ish, not all of it.
  const auto ps3 = abr::mode_ceiling_bps(80 * mbps, 3840, 2160, 120, 1280, 720, 60000.0 / 1001);
  EXPECT_LT(ps3, 10 * mbps);
  EXPECT_GT(ps3, 5 * mbps);
  // Unknown modes change nothing.
  EXPECT_EQ(abr::mode_ceiling_bps(80 * mbps, 3840, 2160, 120, 0, 0, 0), 80 * mbps);
}

TEST(AdaptiveBitrateMode, AutoFloor) {
  EXPECT_EQ(abr::auto_floor_bps(80 * mbps), 10 * mbps);
  EXPECT_EQ(abr::auto_floor_bps(8 * mbps), 2 * mbps);
}

TEST_F(AdaptiveBitrateTest, ACleanLinkNeverMoves) {
  ctl.begin(56 * mbps, 7 * mbps, t0);
  auto now = t0;
  EXPECT_EQ(run(now, 60s), 56 * mbps);
}

TEST_F(AdaptiveBitrateTest, RepairedLossBacksOffGentlyAndOnlyToHalf) {
  ctl.begin(40 * mbps, 5 * mbps, t0);
  auto now = t0;
  ctl.on_frame_loss(true, now);
  now += 16ms;
  EXPECT_EQ(ctl.update(now), (std::int64_t) (40 * mbps * abr::controller_t::mild_factor));
  // FEC keeps repairing frames every 100 ms for 10 s: the target stops at half
  // the ceiling, because the player has seen nothing.
  for (int i = 0; i < 100; ++i) {
    ctl.on_frame_loss(true, now);
    now += 100ms;
    ctl.update(now);
  }
  EXPECT_EQ(ctl.target(), 20 * mbps);
}

TEST_F(AdaptiveBitrateTest, LostFramesGoToTheFloorQuickly) {
  ctl.begin(80 * mbps, 10 * mbps, t0);
  auto now = t0;
  // A lost frame every 50 ms: back-offs are spaced 250 ms apart, 0.7x each.
  std::int64_t last = ctl.target();
  int decreases = 0;
  for (int i = 0; i < 60; ++i) {
    ctl.on_frame_loss(false, now);
    now += 50ms;
    const auto t = ctl.update(now);
    if (t < last) {
      ++decreases;
      last = t;
    }
  }
  EXPECT_EQ(ctl.target(), 10 * mbps);
  EXPECT_LE(decreases, 13);  // 3 s / 250 ms
  EXPECT_GE(decreases, 5);  // 80 * 0.7^5 < 17: several steps, not one
}

TEST_F(AdaptiveBitrateTest, ABurstOfReportsIsOneBackOff) {
  ctl.begin(50 * mbps, 5 * mbps, t0);
  auto now = t0;
  for (int i = 0; i < 8; ++i) {
    ctl.on_frame_loss(false, now);
  }
  now += 16ms;
  EXPECT_EQ(ctl.update(now), (std::int64_t) (50 * mbps * abr::controller_t::severe_factor));
  // More reports inside the spacing window: held, not cut again.
  ctl.on_frame_loss(false, now);
  now += 16ms;
  EXPECT_EQ(ctl.update(now), (std::int64_t) (50 * mbps * abr::controller_t::severe_factor));
}

TEST_F(AdaptiveBitrateTest, ClimbsBackAfterQuiet) {
  ctl.begin(40 * mbps, 5 * mbps, t0);
  auto now = t0;
  ctl.on_frame_loss(false, now);
  now += 16ms;
  const auto low = ctl.update(now);
  EXPECT_LT(low, 40 * mbps);
  // Held through the quiet period after a severe loss...
  EXPECT_EQ(run(now, 2900ms), low);
  // ...then climbs back to the ceiling within ~15 s.
  EXPECT_EQ(run(now, 15s), 40 * mbps);
}

TEST_F(AdaptiveBitrateTest, KeyframeRequestsInGraceAreNotLoss) {
  ctl.begin(40 * mbps, 5 * mbps, t0);
  ctl.grace_until(t0 + 3s);
  auto now = t0 + 1s;
  ctl.on_keyframe_request(now);
  now += 16ms;
  EXPECT_EQ(ctl.update(now), 40 * mbps);
  now = t0 + 4s;
  ctl.on_keyframe_request(now);
  now += 16ms;
  EXPECT_LT(ctl.update(now), 40 * mbps);
}

TEST_F(AdaptiveBitrateTest, CeilingMovesCarryTheBackOff) {
  ctl.begin(56 * mbps, 7 * mbps, t0);
  auto now = t0;
  // Unconstrained: follows the new ceiling exactly.
  ctl.set_ceiling(7 * mbps, now);
  EXPECT_EQ(ctl.target(), 7 * mbps);
  ctl.set_ceiling(56 * mbps, now);
  EXPECT_EQ(ctl.target(), 56 * mbps);
  // Backed off to 70%: keeps the same share of a new ceiling.
  ctl.on_frame_loss(false, now);
  now += 16ms;
  ctl.update(now);
  ctl.set_ceiling(28 * mbps, now);
  EXPECT_NEAR((double) ctl.target() / mbps, 28 * abr::controller_t::severe_factor, 0.01);
}
