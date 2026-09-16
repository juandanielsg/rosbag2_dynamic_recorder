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

namespace rosbag2_dynamic_recorder
{

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

rclcpp::TimerBase::SharedPtr & Scheduler::slot(Kind kind)
{
  switch (kind) {
    case Kind::Resume: return resume_timer_;
    case Kind::Split: return split_timer_;
    default: return record_timer_;
  }
}

void Scheduler::arm(Kind kind, std::chrono::nanoseconds delta, std::function<void()> action)
{
  auto & timer = slot(kind);
  if (timer) {
    timer->cancel();
  }
  timer = node_.create_wall_timer(
    delta,
    [&timer, action = std::move(action)]() {
      timer->cancel();  // One shot.
      action();
    },
    group_);
}

void Scheduler::schedule_at_message_time(
  Kind kind, rcutils_time_point_value_t at_ns, Mode mode, const std::string & tracking_topic)
{
  std::lock_guard<std::mutex> lock(mutex_);
  (kind == Kind::Resume ? resume_ : split_) = {true, at_ns, mode, tracking_topic};
  pending_.store(true, std::memory_order_relaxed);
}

bool Scheduler::Pending::satisfied_by(
  const std::string & topic, rcutils_time_point_value_t send_ns,
  rcutils_time_point_value_t recv_ns) const
{
  if (!active || mode == Mode::NodeTime) {
    return false;
  }
  if (!tracking_topic.empty() && tracking_topic != topic) {
    return false;
  }
  const auto stamp = mode == Mode::PublishTime ? send_ns : recv_ns;
  // A zero stamp means the middleware did not supply one; comparing against it would fire
  // immediately and for the wrong reason.
  return stamp != 0 && stamp >= at_ns;
}

Scheduler::Fired Scheduler::fired(
  const std::string & topic, rcutils_time_point_value_t send_ns,
  rcutils_time_point_value_t recv_ns)
{
  if (!pending_.load(std::memory_order_relaxed)) {
    return {};
  }
  std::lock_guard<std::mutex> lock(mutex_);
  Fired fired;
  fired.resume = resume_.satisfied_by(topic, send_ns, recv_ns);
  fired.split = split_.satisfied_by(topic, send_ns, recv_ns);
  resume_.active = resume_.active && !fired.resume;
  split_.active = split_.active && !fired.split;
  // Keep the fast path honest: only a schedule still active warrants taking the lock again.
  pending_.store(resume_.active || split_.active, std::memory_order_relaxed);
  return fired;
}

void Scheduler::clear()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    resume_ = {};
    split_ = {};
    pending_.store(false, std::memory_order_relaxed);
  }
  for (auto * timer : {&resume_timer_, &split_timer_}) {
    if (*timer) {
      (*timer)->cancel();
    }
  }
}

}  // namespace rosbag2_dynamic_recorder
