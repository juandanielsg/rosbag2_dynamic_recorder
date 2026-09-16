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

#ifndef ROSBAG2_DYNAMIC_RECORDER__SCHEDULER_HPP_
#define ROSBAG2_DYNAMIC_RECORDER__SCHEDULER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/time.h"

namespace rosbag2_dynamic_recorder
{

/// Resume, split and record requests asked for at a future time rather than immediately.
///
/// Two mechanisms, because they answer different needs. A node-time schedule is a one-shot
/// timer, so it fires even on a silent robot. A publish- or receive-time schedule can only be
/// evaluated against arriving messages, so it is checked from the message path and will not
/// fire if the traffic stops. Only the newest schedule of each kind is live.
///
/// Timers run in the callback group given at construction, which the recorder shares with its
/// services; that group is MutuallyExclusive, and rclcpp holds it from TimerBase::call() to the
/// callback, so a cancelled timer's callback never runs and nothing here needs an epoch.
class Scheduler
{
public:
  enum class Kind { Resume, Split, Record };

  /// The clock a scheduled time is compared against, as the services encode it.
  enum class Mode : int32_t { NodeTime = 0, PublishTime = 1, ReceiveTime = 2 };

  /// A request's mode field as a Mode, or nothing for a value the services do not define.
  static std::optional<Mode> mode_from(int32_t value);

  struct Fired
  {
    bool resume{false};
    bool split{false};
  };

  Scheduler(rclcpp::Node & node, rclcpp::CallbackGroup::SharedPtr group);

  /// Arm a one-shot node-time timer for `kind`, retiring any pending one of that kind.
  void arm(Kind kind, std::chrono::nanoseconds delta, std::function<void()> action);

  /// Queue a resume or split against message time. `tracking_topic` empty means any topic.
  void schedule_at_message_time(
    Kind kind, rcutils_time_point_value_t at_ns, Mode mode, const std::string & tracking_topic);

  /// From the message path: which message-time schedules this message satisfies. Each fires
  /// once. Cheap when nothing is queued: one relaxed load and no lock.
  Fired fired(
    const std::string & topic, rcutils_time_point_value_t send_ns,
    rcutils_time_point_value_t recv_ns);

  /// Forget every pending resume and split. A split queued against a bag that has since been
  /// closed, or a resume queued before a stop, must not fire against the next recording. A
  /// scheduled record is left alone: it exists precisely to run while stopped.
  void clear();

private:
  struct Pending
  {
    bool active{false};
    rcutils_time_point_value_t at_ns{0};
    Mode mode{Mode::NodeTime};
    std::string tracking_topic;

    bool satisfied_by(
      const std::string & topic, rcutils_time_point_value_t send_ns,
      rcutils_time_point_value_t recv_ns) const;
  };

  rclcpp::TimerBase::SharedPtr & slot(Kind kind);

  rclcpp::Node & node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::TimerBase::SharedPtr resume_timer_;
  rclcpp::TimerBase::SharedPtr split_timer_;
  rclcpp::TimerBase::SharedPtr record_timer_;

  std::mutex mutex_;
  Pending resume_;
  Pending split_;
  /// Fast path for the message callback: false means no message-time schedule can fire, so
  /// fired() returns without taking the lock on every arriving message.
  std::atomic_bool pending_{false};
};

}  // namespace rosbag2_dynamic_recorder

#endif  // ROSBAG2_DYNAMIC_RECORDER__SCHEDULER_HPP_
