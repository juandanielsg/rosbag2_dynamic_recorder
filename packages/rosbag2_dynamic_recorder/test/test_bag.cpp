// Copyright 2026 Juan Daniel Suárez González
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// A recorded message never waits for a slow operation on the writer: it is staged, written in
/// order once the writer is free, and reported lost only if the stage overflows.

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_dynamic_recorder/bag.hpp"

namespace rosbag2_dynamic_recorder
{

/// Holds the writer the way ensure_channel() and friends do while storage is slow.
struct BagTestAccess
{
  static void hold_busy(Bag & bag, std::chrono::milliseconds how_long)
  {
    std::lock_guard<std::timed_mutex> lock(bag.mutex_);
    Bag::Busy busy(bag);
    std::this_thread::sleep_for(how_long);
  }
};

}  // namespace rosbag2_dynamic_recorder

using rosbag2_dynamic_recorder::Bag;
using rosbag2_dynamic_recorder::BagTestAccess;

namespace
{

constexpr char kTopic[] = "/staged";
constexpr char kType[] = "std_msgs/msg/UInt8MultiArray";

class BagStaging : public ::testing::Test
{
protected:
  void SetUp() override
  {
    uri_ = (std::filesystem::temp_directory_path() /
      ("test_bag_" + std::to_string(::getpid()) + "_" +
      ::testing::UnitTest::GetInstance()->current_test_info()->name())).string();
    std::filesystem::remove_all(uri_);
    clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  }

  void TearDown() override {std::filesystem::remove_all(uri_);}

  void open(Bag & bag, uint64_t cache_bytes)
  {
    rosbag2_storage::StorageOptions options;
    options.uri = uri_;
    options.storage_id = "mcap";
    options.max_cache_size = cache_bytes;
    rosbag2_cpp::ConverterOptions converter{"cdr", "cdr"};
    ASSERT_EQ(bag.open(options, converter, {}, clock_->now()), "");
    ASSERT_EQ(bag.ensure_channel({0u, kTopic, kType, "cdr", {}, ""}), "");
  }

  /// A serialized message whose first 8 bytes carry `n`, so order can be checked on read.
  static std::shared_ptr<const rclcpp::SerializedMessage> numbered(uint64_t n, size_t size = 64)
  {
    auto msg = std::make_shared<rclcpp::SerializedMessage>(size);
    auto & raw = msg->get_rcl_serialized_message();
    std::memset(raw.buffer, 0, size);
    std::memcpy(raw.buffer, &n, sizeof(n));
    raw.buffer_length = size;
    return msg;
  }

  std::vector<uint64_t> read_back()
  {
    rosbag2_cpp::Reader reader;
    reader.open(uri_);
    std::vector<uint64_t> out;
    while (reader.has_next()) {
      auto msg = reader.read_next();
      if (msg->topic_name == kTopic) {
        uint64_t n = 0;
        std::memcpy(&n, msg->serialized_data->buffer, sizeof(n));
        out.push_back(n);
      }
    }
    return out;
  }

  std::string uri_;
  rclcpp::Clock::SharedPtr clock_;
};

TEST_F(BagStaging, writes_during_a_slow_operation_return_at_once_and_land_in_order)
{
  Bag bag(rclcpp::get_logger("test_bag"), clock_);
  open(bag, 1024 * 1024);
  bag.write(numbered(0), kTopic, kType, 1, 1);

  std::thread slow([&bag] {BagTestAccess::hold_busy(bag, std::chrono::milliseconds(300));});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));  // let it take the writer
  const auto start = std::chrono::steady_clock::now();
  for (uint64_t n = 1; n <= 5; ++n) {
    bag.write(numbered(n), kTopic, kType, static_cast<int64_t>(n + 1), 0);
  }
  const auto took = std::chrono::steady_clock::now() - start;
  slow.join();
  EXPECT_LT(took, std::chrono::milliseconds(100)) <<
    "five writes against a 300 ms operation must not wait for it";

  bag.write(numbered(6), kTopic, kType, 7, 0);
  ASSERT_TRUE(bag.close());
  EXPECT_EQ(read_back(), (std::vector<uint64_t>{0, 1, 2, 3, 4, 5, 6})) <<
    "staged messages go in before the next one, in the order they came";
}

TEST_F(BagStaging, a_stage_that_overflows_reports_the_loss_instead_of_blocking)
{
  Bag bag(rclcpp::get_logger("test_bag"), clock_);
  uint64_t lost = 0;
  std::mutex lost_mutex;
  bag.set_loss_callback(
    [&](const std::string & topic, uint64_t count) {
      std::lock_guard<std::mutex> lock(lost_mutex);
      EXPECT_EQ(topic, kTopic);
      lost += count;
    });
  // A 4 KiB cache, so a 4 KiB stage: three 1.5 KiB messages fit twice over, not three times.
  open(bag, 4096);

  std::thread slow([&bag] {BagTestAccess::hold_busy(bag, std::chrono::milliseconds(200));});
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  for (uint64_t n = 1; n <= 3; ++n) {
    bag.write(numbered(n, 1536), kTopic, kType, static_cast<int64_t>(n), 0);
  }
  slow.join();
  ASSERT_TRUE(bag.close());
  EXPECT_EQ(lost, 1u);
  EXPECT_EQ(read_back(), (std::vector<uint64_t>{1, 2}));
}

TEST_F(BagStaging, what_is_staged_after_a_stop_is_not_recorded)
{
  Bag bag(rclcpp::get_logger("test_bag"), clock_);
  open(bag, 1024 * 1024);
  bag.write(numbered(1), kTopic, kType, 1, 0);
  ASSERT_TRUE(bag.close());
  bag.write(numbered(2), kTopic, kType, 2, 0);  // bag closed: dropped, and not an error
  EXPECT_EQ(read_back(), (std::vector<uint64_t>{1}));
}

TEST_F(BagStaging, a_new_bag_never_opens_over_an_existing_one)
{
  using rosbag2_dynamic_recorder::unused_bag_path;
  EXPECT_EQ(unused_bag_path(uri_), uri_) << "nothing there yet";
  std::filesystem::create_directories(uri_);
  std::filesystem::create_directories(uri_ + "(1)");
  EXPECT_EQ(unused_bag_path(uri_), uri_ + "(2)");
  std::filesystem::remove_all(uri_ + "(1)");
}

}  // namespace
