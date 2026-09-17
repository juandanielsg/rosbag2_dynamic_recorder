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

#include "rosbag2_dynamic_recorder/recorder_config.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace rosbag2_dynamic_recorder
{

namespace
{
/// Default writer cache, as rosbag2's own recorder ships it. Also the launch files' default.
constexpr int64_t kDefaultMaxCacheSize = 100 * 1024 * 1024;

uint64_t non_negative(const char * name, int64_t value)
{
  if (value < 0) {
    throw std::invalid_argument(std::string(name) + " must be >= 0");
  }
  return static_cast<uint64_t>(value);
}

/// Profiles are declared as a list of names plus one string-array parameter each, rather than a
/// nested structure, because ROS 2 parameters have no nested arrays and this keeps a plain YAML
/// file readable:
///   profile_names: ["idle", "navigation"]
///   profiles:
///     idle: ["/tf", "/odom"]
///     navigation: ["/tf", "/odom", "/scan"]
std::vector<std::pair<std::string, std::vector<std::string>>> declare_profiles(rclcpp::Node & node)
{
  std::vector<std::pair<std::string, std::vector<std::string>>> profiles;
  const auto names = node.declare_parameter<std::vector<std::string>>(
    "profile_names", std::vector<std::string>{});
  for (const auto & name : names) {
    if (name.empty()) {
      RCLCPP_WARN(node.get_logger(), "Ignoring an empty profile name");
      continue;
    }
    auto topics = node.declare_parameter<std::vector<std::string>>(
      "profiles." + name, std::vector<std::string>{});
    if (topics.empty()) {
      RCLCPP_WARN(node.get_logger(),
        "Profile '%s' lists no topics; applying it would record nothing", name.c_str());
    }
    std::sort(topics.begin(), topics.end());
    topics.erase(std::unique(topics.begin(), topics.end()), topics.end());
    profiles.emplace_back(name, std::move(topics));
  }
  if (!profiles.empty()) {
    RCLCPP_INFO(node.get_logger(), "Loaded %zu recording profile(s)", profiles.size());
  }
  return profiles;
}
}  // namespace

RecorderConfig declare_parameters(rclcpp::Node & node)
{
  RecorderConfig c;
  c.storage.uri = node.declare_parameter<std::string>("uri", "dynamic_bag");
  c.storage.storage_id = node.declare_parameter<std::string>("storage_id", "mcap");
  c.serialization_format = node.declare_parameter<std::string>("serialization_format", "cdr");
  c.converter.input_serialization_format = c.serialization_format;
  c.converter.output_serialization_format = c.serialization_format;
  c.record_subscription_events = node.declare_parameter<bool>("record_subscription_events", true);
  c.record_pause_events = node.declare_parameter<bool>("record_pause_events", true);
  c.record_low_disk_events = node.declare_parameter<bool>("record_low_disk_events", true);
  c.record_bag_size_limit_events =
    node.declare_parameter<bool>("record_bag_size_limit_events", true);
  c.snapshot_mode = node.declare_parameter<bool>("snapshot_mode", false);
  c.storage.snapshot_mode = c.snapshot_mode;

  // The rest of rosbag2's StorageOptions, passed through untouched. These are the knobs that
  // decide whether a small computer keeps up: a time-bounded cache is something an operator can
  // reason about where "100 MB" is not, and the MCAP presets trade CPU against disk bandwidth in
  // either direction. Without them the writer runs on defaults that nothing here can change.
  c.storage.max_cache_size = non_negative(
    "max_cache_size", node.declare_parameter<int64_t>("max_cache_size", kDefaultMaxCacheSize));
  c.max_cache_duration = static_cast<int64_t>(non_negative(
      "max_cache_duration", node.declare_parameter<int64_t>("max_cache_duration", 0)));
  c.storage.max_bagfile_size = non_negative(
    "max_bagfile_size", node.declare_parameter<int64_t>("max_bagfile_size", 0));
  c.storage.max_bagfile_duration = non_negative(
    "max_bagfile_duration", node.declare_parameter<int64_t>("max_bagfile_duration", 0));
  c.storage.storage_preset_profile =
    node.declare_parameter<std::string>("storage_preset_profile", "");
  c.storage.storage_config_uri = node.declare_parameter<std::string>("storage_config_uri", "");
#if ROSBAG2_DYNAMIC_RECORDER_HAS_MAX_CACHE_DURATION
  c.storage.max_cache_duration = static_cast<uint32_t>(c.max_cache_duration);
#else
  // Refusing is better than silently writing synchronously when the operator asked for a buffer.
  if (c.max_cache_duration != 0) {
    throw std::invalid_argument(
      "max_cache_duration is not supported by this rosbag2; use max_cache_size");
  }
#endif
  // Snapshot mode is a circular buffer flushed on demand, so it is meaningless without a cache.
  // The writer accepts either bound: a cache limited only by duration is a valid buffer.
  if (c.snapshot_mode && !c.cache_enabled()) {
    throw std::invalid_argument(
      "snapshot_mode requires max_cache_size > 0 or max_cache_duration > 0");
  }

  c.min_free_space = non_negative(
    "min_free_space", node.declare_parameter<int64_t>("min_free_space", 0));
  c.min_free_space_percent = node.declare_parameter<double>("min_free_space_percent", 0.0);
  if (c.min_free_space_percent < 0.0 || c.min_free_space_percent > 100.0) {
    throw std::invalid_argument("min_free_space_percent must be between 0 and 100");
  }
  c.max_bag_size = non_negative(
    "max_bag_size", node.declare_parameter<int64_t>("max_bag_size", 0));
  c.storage_check_period_s = node.declare_parameter<double>("storage_check_period", 1.0);
  if ((c.low_disk_check_enabled() || c.bag_size_check_enabled()) &&
    c.storage_check_period_s <= 0.0)
  {
    throw std::invalid_argument(
      "storage_check_period must be > 0 when a free-space minimum or a bag size limit is set");
  }

  c.messages_lost_report_period_s =
    node.declare_parameter<double>("messages_lost_report_period", 5.0);
  c.status_publish_period_s = node.declare_parameter<double>("status_publish_period", 1.0);
  c.start_paused = node.declare_parameter<bool>("start_paused", false);
  c.use_sim_time = node.get_parameter("use_sim_time").as_bool();
  c.initial_topics = node.declare_parameter<std::vector<std::string>>(
    "topics", std::vector<std::string>{});
  c.profiles = declare_profiles(node);
  return c;
}

}  // namespace rosbag2_dynamic_recorder
