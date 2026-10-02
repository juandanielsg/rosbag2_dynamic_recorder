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

#include "rosbag2_dynamic_recorder/scheduler.hpp"

#include <string>
#include <utility>
#include <vector>

namespace rosbag2_dynamic_recorder
{

namespace
{
bool fires_on_messages(const std::optional<Scheduler::Scheduled> & scheduled)
{
  return scheduled && scheduled->mode != Scheduler::Mode::NodeTime;
}

bool is_due(
  const std::optional<Scheduler::Scheduled> & scheduled, const std::string & topic,
  rcutils_time_point_value_t send_ns, rcutils_time_point_value_t recv_ns)
{
  if (!fires_on_messages(scheduled)) {
    return false;
  }
  if (!scheduled->tracking_topic.empty() && scheduled->tracking_topic != topic) {
    return false;
  }
  const auto stamp = scheduled->mode == Scheduler::Mode::PublishTime ? send_ns : recv_ns;
  // A zero stamp is one the middleware did not supply, not a time before the schedule.
  return stamp != 0 && stamp >= scheduled->at_ns;
}
}  // namespace

std::optional<Scheduler::Mode> Scheduler::mode_from(int32_t value)
{
  switch (value) {
    case 0: return Mode::NodeTime;
    case 1: return Mode::PublishTime;
    case 2: return Mode::ReceiveTime;
    default: return std::nullopt;
  }
}

Scheduler::Scheduler(rclcpp::Node & node, rclcpp::CallbackGroup::SharedPtr group)
: node_(node), group_(std::move(group))
{
}

void Scheduler::update_message_time_pending()
{
  message_time_pending_.store(
    fires_on_messages(slot(Kind::Resume).scheduled) ||
    fires_on_messages(slot(Kind::Split).scheduled),
    std::memory_order_relaxed);
}

void Scheduler::replace(Kind kind, Scheduled scheduled)
{
  if (slot(kind).timer) {
    slot(kind).timer->cancel();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  slot(kind).scheduled = std::move(scheduled);
  update_message_time_pending();
}

void Scheduler::schedule_at_node_time(
  Kind kind, std::chrono::nanoseconds delay, std::function<void()> action)
{
  replace(kind, {kind, node_.now().nanoseconds() + delay.count(), Mode::NodeTime, ""});
  auto & timer = slot(kind).timer;
  timer = node_.create_timer(
    delay,
    [this, kind, &timer, action = std::move(action)]() {
      timer->cancel();  // One shot.
      {
        // Off the list before the action runs, so a status it publishes no longer shows it. A
        // newer schedule would have cancelled this timer, so the slot is still this one's.
        std::lock_guard<std::mutex> lock(mutex_);
        slot(kind).scheduled.reset();
      }
      action();
    },
    group_);
}

void Scheduler::schedule_at_message_time(
  Kind kind, rcutils_time_point_value_t at_ns, Mode mode, const std::string & tracking_topic)
{
  replace(kind, {kind, at_ns, mode, tracking_topic});
}

std::vector<Scheduler::Scheduled> Scheduler::pending() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<Scheduled> out;
  for (const auto & s : slots_) {
    if (s.scheduled) {
      out.push_back(*s.scheduled);
    }
  }
  return out;
}

Scheduler::Due Scheduler::take_due(
  const std::string & topic, rcutils_time_point_value_t send_ns,
  rcutils_time_point_value_t recv_ns)
{
  if (!message_time_pending_.load(std::memory_order_relaxed)) {
    return {};
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const auto take = [&](Kind kind) {
      auto & scheduled = slot(kind).scheduled;
      const bool due = is_due(scheduled, topic, send_ns, recv_ns);
      if (due) {
        scheduled.reset();
      }
      return due;
    };
  Due due;
  due.resume = take(Kind::Resume);
  due.split = take(Kind::Split);
  update_message_time_pending();
  return due;
}

void Scheduler::clear()
{
  for (const auto kind : {Kind::Resume, Kind::Split}) {
    if (slot(kind).timer) {
      slot(kind).timer->cancel();
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  slot(Kind::Resume).scheduled.reset();
  slot(Kind::Split).scheduled.reset();
  update_message_time_pending();
}

}  // namespace rosbag2_dynamic_recorder
