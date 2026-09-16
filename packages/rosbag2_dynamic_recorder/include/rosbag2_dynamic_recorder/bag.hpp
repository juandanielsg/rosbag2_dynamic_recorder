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

#ifndef ROSBAG2_DYNAMIC_RECORDER__BAG_HPP_
#define ROSBAG2_DYNAMIC_RECORDER__BAG_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rcutils/time.h"
#include "rosbag2_cpp/bag_events.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

namespace rosbag2_dynamic_recorder
{

/// The bag being written: rosbag2's writer, the channels it has, and the counters describing it.
///
/// Every operation takes the one lock inside, so the recorder never holds a writer lock itself
/// and the invariant "the writer is only touched while open" lives here rather than in comments
/// across the node. Write failures are counted and logged at a rate that cannot itself become
/// the problem: a full disk used to take the whole node down, and the remaining topics and any
/// later recovery are worth more than a clean death.
class Bag
{
public:
  using EventCallbacks = rosbag2_cpp::bag_events::WriterEventCallbacks;

  struct Info
  {
    bool open{false};
    std::string uri;
    rclcpp::Time opened_at;
    uint64_t messages_written{0};
    uint64_t write_errors{0};
    uint64_t splits{0};
  };

  Bag(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);

  /// Open a new bag at `options.uri`. Returns why it could not, or empty on success. The
  /// counters restart: a new bag is a fresh episode.
  std::string open(
    const rosbag2_storage::StorageOptions & options,
    const rosbag2_cpp::ConverterOptions & converter, const EventCallbacks & callbacks,
    rclcpp::Time opened_at);

  /// Close the bag. Returns false if it was not open, so a repeated stop is a no-op.
  bool close();

  bool is_open() const;
  std::string uri() const;
  Info info() const;

  /// The type this bag records `topic` under, if it has a channel for it.
  std::optional<std::string> channel_type(const std::string & topic) const;

  /// Create the channel for `metadata.name` unless the bag already has it. Returns why it could
  /// not, or empty. Refuses a topic the bag already holds under a different type: writing the
  /// new type into that channel would silently corrupt the recording, and a publisher restarted
  /// with a changed type is the realistic way to get here.
  std::string ensure_channel(const rosbag2_storage::TopicMetadata & metadata);

  /// Write one message. Silently dropped when the bag is closed; counted and logged when the
  /// writer fails.
  void write(
    std::shared_ptr<const rclcpp::SerializedMessage> message, const std::string & topic,
    const std::string & type, rcutils_time_point_value_t recv_ns,
    rcutils_time_point_value_t send_ns);

  /// Write a recorder event, creating its channel on first use. Events explain the bag, so
  /// unlike write() this is never gated on anything but the bag being open.
  void write_event(
    const std::string & topic, const std::string & type,
    std::shared_ptr<const rclcpp::SerializedMessage> message, rcutils_time_point_value_t stamp_ns);

  /// Close the current file and open the next. False when closed or the writer refused.
  bool split();

  /// Flush the snapshot buffer to disk. False when closed, not in snapshot mode, or refused.
  bool take_snapshot();

private:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  mutable std::mutex mutex_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  bool open_{false};
  std::string uri_;
  std::string serialization_format_;
  rclcpp::Time opened_at_;
  /// Topic -> type of every channel created in this bag, so a second request for the same
  /// topic skips the expensive create_topic() and a type change is caught.
  std::unordered_map<std::string, std::string> channels_;

  std::atomic_uint64_t messages_written_{0};
  std::atomic_uint64_t write_errors_{0};
  std::atomic_uint64_t splits_{0};
};

}  // namespace rosbag2_dynamic_recorder

#endif  // ROSBAG2_DYNAMIC_RECORDER__BAG_HPP_
