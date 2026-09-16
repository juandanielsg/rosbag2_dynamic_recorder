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

/// The parameter rules, checked without launching a recorder per case. The service tests cover
/// two of these refusals end to end; here every one is a few milliseconds and cannot flake on a
/// slow process start.

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_dynamic_recorder/recorder_config.hpp"

using rosbag2_dynamic_recorder::RecorderConfig;
using rosbag2_dynamic_recorder::declare_parameters;

class RecorderConfigTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  /// Declare the parameters on a throwaway node carrying `overrides`, as a launch file would.
  static RecorderConfig configure(const std::vector<rclcpp::Parameter> & overrides = {})
  {
    rclcpp::Node node("config_test", rclcpp::NodeOptions().parameter_overrides(overrides));
    return declare_parameters(node);
  }

  static std::string refusal(const std::vector<rclcpp::Parameter> & overrides)
  {
    try {
      configure(overrides);
    } catch (const std::invalid_argument & e) {
      return e.what();
    }
    return "";
  }
};

TEST_F(RecorderConfigTest, defaults_record_everything_and_guard_nothing)
{
  const auto c = configure();
  EXPECT_EQ(c.storage.uri, "dynamic_bag");
  EXPECT_EQ(c.storage.storage_id, "mcap");
  EXPECT_EQ(c.converter.output_serialization_format, "cdr");
  EXPECT_TRUE(c.record_subscription_events && c.record_pause_events);
  EXPECT_TRUE(c.record_low_disk_events && c.record_bag_size_limit_events);
  EXPECT_FALSE(c.low_disk_check_enabled());
  EXPECT_FALSE(c.bag_size_check_enabled());
  EXPECT_TRUE(c.cache_enabled());
  EXPECT_TRUE(c.profiles.empty());
}

TEST_F(RecorderConfigTest, negative_sizes_are_refused_by_name)
{
  for (const char * name : {"max_cache_size", "max_bagfile_size", "max_bagfile_duration",
    "min_free_space", "max_bag_size"})
  {
    const auto message = refusal({rclcpp::Parameter(name, -1)});
    EXPECT_NE(message.find(name), std::string::npos) << name << ": " << message;
  }
}

TEST_F(RecorderConfigTest, an_impossible_free_space_percentage_is_refused)
{
  EXPECT_NE(refusal({rclcpp::Parameter("min_free_space_percent", 150.0)}).find("percent"),
    std::string::npos);
  EXPECT_EQ(refusal({rclcpp::Parameter("min_free_space_percent", 100.0)}), "");
}

TEST_F(RecorderConfigTest, a_guard_needs_a_positive_check_period)
{
  EXPECT_NE(refusal({rclcpp::Parameter("max_bag_size", 1000),
        rclcpp::Parameter("storage_check_period", 0.0)}).find("storage_check_period"),
    std::string::npos);
  // Without a guard the period is irrelevant, so it is not validated.
  EXPECT_EQ(refusal({rclcpp::Parameter("storage_check_period", 0.0)}), "");
}

TEST_F(RecorderConfigTest, snapshot_mode_needs_a_cache)
{
  EXPECT_NE(refusal({rclcpp::Parameter("snapshot_mode", true),
        rclcpp::Parameter("max_cache_size", 0)}).find("snapshot_mode"), std::string::npos);
  EXPECT_TRUE(configure({rclcpp::Parameter("snapshot_mode", true)}).storage.snapshot_mode);
}

TEST_F(RecorderConfigTest, profiles_keep_declaration_order_with_sorted_unique_topics)
{
  const auto c = configure({
      rclcpp::Parameter("profile_names", std::vector<std::string>{"small", "", "large"}),
      rclcpp::Parameter("profiles.small", std::vector<std::string>{"/b", "/a", "/b"}),
      rclcpp::Parameter("profiles.large", std::vector<std::string>{"/c"}),
    });
  ASSERT_EQ(c.profiles.size(), 2u) << "the empty name is skipped, not an error";
  EXPECT_EQ(c.profiles[0].first, "small");
  EXPECT_EQ(c.profiles[0].second, (std::vector<std::string>{"/a", "/b"}));
  EXPECT_EQ(c.profiles[1].first, "large");
}
