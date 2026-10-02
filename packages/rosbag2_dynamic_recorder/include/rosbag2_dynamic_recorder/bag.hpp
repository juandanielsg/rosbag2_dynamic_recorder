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
#include <deque>
#include <functional>
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
#include "rosbag2_dynamic_recorder/timings.hpp"

namespace rosbag2_dynamic_recorder
{

/// Where a new bag can be opened for `uri`: `uri` itself if nothing is there, else the first free
/// `uri(N)`, as upstream's recorder picks. Nothing when every suffix up to a large bound is taken.
std::optional<std::string> unused_bag_path(const std::string & uri);

/// The bag being written: rosbag2's writer, the channels it has, and the counters describing it.
///
/// Every operation takes the one lock inside, so the writer is only ever touched while open.
/// A write failure is counted and logged, throttled, rather than thrown: the other topics, and
/// any later recovery, are worth more than ending the process over a full disk.
///
/// A recorded message never waits for a slow operation. Creating a channel, writing an event,
/// splitting, snapshotting and closing all reach storage, and rosbag2 serialises storage behind
/// the mutex its cache thread holds while writing to disk, so during a disk stall such an
/// operation holds the writer for as long as the stall lasts. A message arriving meanwhile is
/// staged instead, and written ahead of the next one once the writer is free; otherwise every
/// subscription callback would block and their queues overflow.
class Bag
{
public:
  using EventCallbacks = rosbag2_cpp::bag_events::WriterEventCallbacks;
  /// For messages the bag drops itself: staged while the writer was busy, with the stage full.
  using LossCallback = std::function<void(const std::string & topic, uint64_t count)>;
  /// For a rollover to a new file: the files, and the reason split() was given, or empty when the
  /// writer rolled over on its own at a size or duration limit. Runs with the bag's lock held,
  /// inside the writer, so it must not call back into the bag.
  using RolloverCallback = std::function<void(
        const rosbag2_cpp::bag_events::BagSplitInfo & info, const std::string & reason)>;

  struct Info
  {
    bool open{false};
    std::string uri;
    rclcpp::Time opened_at;
    /// When the last bag was closed; meaningful only while `open` is false. Zero before any close.
    rclcpp::Time closed_at;
    uint64_t messages_written{0};
    uint64_t write_errors{0};
    uint64_t splits{0};
  };

  Bag(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);

  /// Report write-lock contention to `timings`; null, the default, measures nothing. Set before
  /// the first write.
  void set_timings(Timings * timings) {timings_ = timings;}

  /// Where messages the bag had to drop are reported. Set before the first write.
  void set_loss_callback(LossCallback callback) {on_lost_ = std::move(callback);}

  /// Where rollovers to a new file are reported. Closing the bag is not one. Set before open().
  void set_rollover_callback(RolloverCallback callback) {on_rollover_ = std::move(callback);}

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
  /// `reason` reaches the rollover callback.
  bool split(const std::string & reason);

  /// Flush the snapshot buffer to disk. False when closed, not in snapshot mode, or refused.
  bool take_snapshot();

private:
  // test/test_bag.cpp holds the writer busy the way a slow operation does, which no public call
  // can do on demand.
  friend struct BagTestAccess;

  struct Staged
  {
    std::shared_ptr<const rclcpp::SerializedMessage> message;
    std::string topic;
    std::string type;
    rcutils_time_point_value_t recv_ns;
    rcutils_time_point_value_t send_ns;
  };

  /// Marks the writer busy for the lifetime of a slow operation, and on the way out writes what
  /// was staged meanwhile. Constructed with mutex_ held, and destroyed before it is released.
  class Busy
  {
public:
    explicit Busy(Bag & bag);
    ~Busy();
    Busy(const Busy &) = delete;
    Busy & operator=(const Busy &) = delete;

private:
    Bag & bag_;
  };

  /// Create a channel the bag does not have yet; mutex_ held. Returns why it could not, or empty.
  std::string create_channel_locked(const rosbag2_storage::TopicMetadata & metadata);
  /// Write one message, counting it or the failure; mutex_ held.
  void write_locked(
    std::shared_ptr<const rclcpp::SerializedMessage> message, const std::string & topic,
    const std::string & type, rcutils_time_point_value_t recv_ns,
    rcutils_time_point_value_t send_ns);
  /// Write everything staged, oldest first; mutex_ held. Dropped unwritten once closed.
  void flush_staged_locked();
  /// Stage a message the busy writer cannot take now, or drop it if the stage is full.
  void stage(Staged && staged);

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  Timings * timings_{nullptr};
  LossCallback on_lost_;
  RolloverCallback on_rollover_;
  /// What the split() in progress was asked for; empty otherwise. mutex_ held.
  std::string split_reason_;

  // Timed so that a waiting message can notice the writer turned busy after it started waiting.
  mutable std::timed_mutex mutex_;
  std::atomic_bool busy_{false};
  std::mutex staged_mutex_;
  std::deque<Staged> staged_;
  size_t staged_bytes_{0};
  /// Checked on every write without taking staged_mutex_; nonzero means there is work.
  std::atomic<size_t> staged_count_{0};
  /// The stage holds at most this much, the writer's own cache size: what it would have absorbed.
  size_t max_staged_bytes_{0};
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  bool open_{false};
  std::string uri_;
  std::string serialization_format_;
  rclcpp::Time opened_at_;
  rclcpp::Time closed_at_;
  /// Topic -> type of every channel created in this bag, so a second request for the same
  /// topic skips the expensive create_topic() and a type change is caught.
  std::unordered_map<std::string, std::string> channels_;

  std::atomic_uint64_t messages_written_{0};
  std::atomic_uint64_t write_errors_{0};
  std::atomic_uint64_t splits_{0};
};

}  // namespace rosbag2_dynamic_recorder

#endif  // ROSBAG2_DYNAMIC_RECORDER__BAG_HPP_
