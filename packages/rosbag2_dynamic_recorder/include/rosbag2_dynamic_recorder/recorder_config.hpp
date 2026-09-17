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

#ifndef ROSBAG2_DYNAMIC_RECORDER__RECORDER_CONFIG_HPP_
#define ROSBAG2_DYNAMIC_RECORDER__RECORDER_CONFIG_HPP_

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/node.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_storage/storage_options.hpp"

namespace rosbag2_dynamic_recorder
{

/// Everything the recorder reads from its parameters, validated and in the shape it uses.
///
/// Declared once by declare_parameters(); the node keeps the struct rather than a copy of each
/// field, so a parameter added here is available everywhere without a second member.
struct RecorderConfig
{
  /// Passed to the writer untouched, apart from the uri suffix record() may add.
  rosbag2_storage::StorageOptions storage;
  rosbag2_cpp::ConverterOptions converter;
  std::string serialization_format;
  /// Declared on every distro so a params file loads on all of them; only Rolling's writer can
  /// honour it, and elsewhere a non-zero value is refused rather than silently ignored.
  int64_t max_cache_duration{0};
  bool snapshot_mode{false};
  bool start_paused{false};
  /// rclcpp's own parameter, read rather than declared. When set, messages are stamped with the
  /// node clock instead of the middleware's receive time, so the bag's timeline is the
  /// simulation's, and nothing is opened or subscribed until /clock has been heard.
  bool use_sim_time{false};
  std::vector<std::string> initial_topics;
  /// In declaration order, each topic list sorted and deduplicated.
  std::vector<std::pair<std::string, std::vector<std::string>>> profiles;

  bool record_subscription_events{true};
  bool record_pause_events{true};
  bool record_low_disk_events{true};
  bool record_bag_size_limit_events{true};

  /// The disk guard: stop when free space falls below either bound. 0 disables that bound;
  /// both 0 disables the check, which is the default.
  uint64_t min_free_space{0};
  double min_free_space_percent{0.0};
  /// The bag's own cap across every split. 0 disables, the default.
  uint64_t max_bag_size{0};
  double storage_check_period_s{1.0};

  double messages_lost_report_period_s{5.0};
  double status_publish_period_s{1.0};

  bool low_disk_check_enabled() const {return min_free_space > 0 || min_free_space_percent > 0.0;}
  bool bag_size_check_enabled() const {return max_bag_size > 0;}
  /// False means every message is written synchronously from its callback.
  bool cache_enabled() const {return storage.max_cache_size > 0 || max_cache_duration != 0;}
};

/// Declare every parameter on `node` and validate the result.
///
/// Throws std::invalid_argument naming the parameter for a value the recorder cannot run with,
/// so a bad launch fails at startup rather than after the first bag is open. Profile problems
/// that can be worked around are logged on the node and skipped.
RecorderConfig declare_parameters(rclcpp::Node & node);

}  // namespace rosbag2_dynamic_recorder

#endif  // ROSBAG2_DYNAMIC_RECORDER__RECORDER_CONFIG_HPP_
