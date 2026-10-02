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

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/time.h"

namespace rosbag2_dynamic_recorder
{

/// Resume, split and record requests asked for at a future time rather than immediately.
///
/// A node-time schedule is a one-shot timer, so it fires even on a silent robot. A publish- or
/// receive-time schedule can only be checked against arriving messages, so it fires from the
/// message path and never if traffic stops. Each kind has one slot: a new schedule replaces the
/// pending one, whichever clock either was set on.
///
/// Timers run in the callback group given at construction. The recorder shares that
/// MutuallyExclusive group with its services, and rclcpp holds it from TimerBase::call() to the
/// callback, so a cancelled timer's callback never runs.
class Scheduler
{
public:
  enum class Kind { Resume, Split, Record };

  /// The clock a scheduled time is compared against, numbered as the services number it.
  enum class Mode : int32_t { NodeTime = 0, PublishTime = 1, ReceiveTime = 2 };

  /// A request's mode field as a Mode, or nothing for a value the services do not define.
  static std::optional<Mode> mode_from(int32_t value);

  /// A schedule waiting to fire.
  struct Scheduled
  {
    Kind kind;
    /// When it fires, on the clock `mode` names.
    rcutils_time_point_value_t at_ns;
    Mode mode;
    /// Message time only: the topic whose messages are checked. Empty means any.
    std::string tracking_topic;
  };

  /// The message-time schedules one message has fired.
  struct Due
  {
    bool resume{false};
    bool split{false};
  };

  Scheduler(rclcpp::Node & node, rclcpp::CallbackGroup::SharedPtr group);

  /// Run `action` once, `delay` from now on the node clock, so under use_sim_time it keeps to the
  /// simulation's time.
  void schedule_at_node_time(Kind kind, std::chrono::nanoseconds delay, std::function<void()> action);

  /// Fire a resume or split on the first message stamped at or after `at_ns`.
  void schedule_at_message_time(
    Kind kind, rcutils_time_point_value_t at_ns, Mode mode, const std::string & tracking_topic);

  /// Every schedule waiting to fire, in Kind order.
  std::vector<Scheduled> pending() const;

  /// Called for every recorded message: the message-time schedules it fires, each only once.
  /// Takes no lock while no message-time schedule is pending.
  Due take_due(
    const std::string & topic, rcutils_time_point_value_t send_ns,
    rcutils_time_point_value_t recv_ns);

  /// Drop every pending resume and split, so none fires against the next recording. A scheduled
  /// record is kept: it exists to run while stopped.
  void clear();

private:
  struct Slot
  {
    rclcpp::TimerBase::SharedPtr timer;
    std::optional<Scheduled> scheduled;  // Under mutex_.
  };

  Slot & slot(Kind kind) {return slots_[static_cast<size_t>(kind)];}
  /// Cancel `kind`'s timer and put `scheduled` in its slot.
  void replace(Kind kind, Scheduled scheduled);
  /// Recompute message_time_pending_; mutex_ held.
  void update_message_time_pending();

  rclcpp::Node & node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  mutable std::mutex mutex_;
  std::array<Slot, 3> slots_;
  /// False when no message-time schedule can fire, so take_due() returns without the lock.
  std::atomic_bool message_time_pending_{false};
};

}  // namespace rosbag2_dynamic_recorder

#endif  // ROSBAG2_DYNAMIC_RECORDER__SCHEDULER_HPP_
