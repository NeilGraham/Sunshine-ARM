/**
 * @file tests/unit/test_input.cpp
 * @brief Tests for retained stream input and virtual gamepad lifecycle behavior.
 */

// standard includes
#include <memory>
#include <string>

// local includes
#include "../tests_common.h"
#include "src/config.h"
#include "src/input.h"
#include "src/platform/virtualhid_input.h"

namespace {
  /**
   * @brief Fixture that installs a fake global virtual input backend.
   */
  class InputGamepadSessionTest: public ::testing::Test {
  protected:
    /**
     * @brief Preserve configuration and install observable fake devices.
     */
    void SetUp() override {
      original_input_ = config::input;
      config::input.controller = true;
      config::input.gamepad = "xseries";

      auto platform_input = platf::input();
      ASSERT_TRUE(platform_input);
      auto &context = platf::virtualhid::get_input_context(platform_input);
      context = platf::virtualhid::input_context_t {lvh::BackendKind::fake};
      runtime_ = context.runtime.get();
      ASSERT_NE(runtime_, nullptr);
      input::testing::set_platform_input(std::move(platform_input));
    }

    /**
     * @brief Destroy retained test sessions and restore configuration.
     */
    void TearDown() override {
      input::terminate_gamepads();
      input::testing::set_platform_input({});
      runtime_ = nullptr;
      config::input = std::move(original_input_);
    }

    /**
     * @brief Access the fake runtime installed for the current test.
     *
     * @return Fake libvirtualhid runtime.
     */
    lvh::Runtime &runtime() const {
      return *runtime_;
    }

  private:
    lvh::Runtime *runtime_ = nullptr;  ///< Fake runtime installed in the global input backend.
    config::input_t original_input_;  ///< Input configuration restored after each test.
  };
}  // namespace

TEST_F(InputGamepadSessionTest, ReusesGamepadsAcrossPauseAndDestroysThemOnTermination) {
  const std::string session_id = "paired-client-certificate";
  auto first_mail = std::make_shared<safe::mail_raw_t>();
  auto first = input::alloc(first_mail, session_id);
  ASSERT_NE(first, nullptr);
  const auto active_devices_before_gamepad = runtime().active_device_count();

  const platf::gamepad_arrival_t metadata {LI_CTYPE_XBOX, 0, 0};
  const auto original_id = input::testing::alloc_gamepad(first, 0, metadata);
  ASSERT_GE(original_id, 0);
  EXPECT_EQ(runtime().active_device_count(), active_devices_before_gamepad + 1);

  const std::weak_ptr<input::input_t> paused = first;
  first.reset();
  EXPECT_FALSE(paused.expired());
  EXPECT_EQ(runtime().active_device_count(), active_devices_before_gamepad + 1);

  auto resumed_mail = std::make_shared<safe::mail_raw_t>();
  auto resumed = input::alloc(resumed_mail, session_id);
  ASSERT_NE(resumed, nullptr);
  EXPECT_EQ(paused.lock(), resumed);
  EXPECT_EQ(input::testing::gamepad_id(resumed, 0), original_id);
  EXPECT_EQ(runtime().active_device_count(), active_devices_before_gamepad + 1);

  input::terminate_gamepads(session_id);
  EXPECT_EQ(input::testing::gamepad_id(resumed, 0), -1);
  EXPECT_EQ(runtime().active_device_count(), active_devices_before_gamepad);

  auto replacement = input::alloc(std::make_shared<safe::mail_raw_t>(), session_id);
  EXPECT_NE(replacement, resumed);
}

TEST_F(InputGamepadSessionTest, KeepsConcurrentClientsAndEvictsDetachedOnes) {
  // Two clients streaming at once (shared-encoder fan-out): each keeps its own
  // pad. A client whose session has ENDED is evicted the next time a
  // different client connects; a client still streaming is never touched.
  // Device counts are measured as deltas: alloc() itself creates a client's
  // keyboard/mouse context, which is not what this test is about.
  const platf::gamepad_arrival_t metadata {LI_CTYPE_XBOX, 0, 0};

  auto first = input::alloc(std::make_shared<safe::mail_raw_t>(), "client-a");
  ASSERT_NE(first, nullptr);
  const auto before_a_pad = runtime().active_device_count();
  const auto a_id = input::testing::alloc_gamepad(first, 0, metadata);
  ASSERT_GE(a_id, 0);
  EXPECT_EQ(runtime().active_device_count(), before_a_pad + 1);

  const auto before_b = runtime().active_device_count();
  auto second = input::alloc(std::make_shared<safe::mail_raw_t>(), "client-b");
  ASSERT_NE(second, nullptr);
  const auto client_context_devices = runtime().active_device_count() - before_b;
  const auto before_b_pad = runtime().active_device_count();
  const auto b_id = input::testing::alloc_gamepad(second, 0, metadata);
  ASSERT_GE(b_id, 0);
  EXPECT_NE(a_id, b_id);
  EXPECT_EQ(input::testing::gamepad_id(first, 0), a_id);
  EXPECT_EQ(runtime().active_device_count(), before_b_pad + 1);

  // The first client's session ends: retained (a resume gets the same pad)...
  const auto both_live = runtime().active_device_count();
  input::detach(first);
  EXPECT_EQ(runtime().active_device_count(), both_live);
  auto resumed = input::alloc(std::make_shared<safe::mail_raw_t>(), "client-a");
  EXPECT_EQ(resumed, first);
  EXPECT_EQ(input::testing::gamepad_id(resumed, 0), a_id);
  EXPECT_EQ(runtime().active_device_count(), both_live);
  input::detach(resumed);

  // ...until a THIRD client connects, which evicts it but not the live second.
  auto third = input::alloc(std::make_shared<safe::mail_raw_t>(), "client-c");
  ASSERT_NE(third, nullptr);
  EXPECT_EQ(input::testing::gamepad_id(first, 0), -1);
  EXPECT_EQ(input::testing::gamepad_id(second, 0), b_id);
  EXPECT_EQ(runtime().active_device_count(), both_live + client_context_devices - 1);

  auto replacement = input::alloc(std::make_shared<safe::mail_raw_t>(), "client-a");
  EXPECT_NE(replacement, first);
}
