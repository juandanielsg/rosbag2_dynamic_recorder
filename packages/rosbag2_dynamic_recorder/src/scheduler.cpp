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

Scheduler::Pending & Scheduler::pending_for(Kind kind)
{
  switch (kind) {
    case Kind::Resume: return resume_;
    case Kind::Split: return split_;
    default: return record_;
  }
}

void Scheduler::update_fast_path_locked()
{
  const auto by_message = [](const Pending & p) {return p.active && p.mode != Mode::NodeTime;};
  pending_.store(by_message(resume_) || by_message(split_), std::memory_order_relaxed);
}

void Scheduler::arm(Kind kind, std::chrono::nanoseconds delta, std::function<void()> action)
{
  auto & timer = slot(kind);
  if (timer) {
    timer->cancel();
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Replaces a message-time schedule of this kind too: only the newest is live.
    pending_for(kind) = {true, node_.now().nanoseconds() + delta.count(), Mode::NodeTime, ""};
    update_fast_path_locked();
  }
  // The node clock rather than a wall timer, so under use_sim_time a schedule keeps to the
  // simulation's time even when it runs slow or pauses.
  timer = node_.create_timer(
    delta,
    [this, kind, &timer, action = std::move(action)]() {
      timer->cancel();  // One shot.
      {
        // Off the list before the action runs, so a status the action publishes no longer shows
        // it. A newer schedule would have cancelled this timer first, so the slot is still ours.
        std::lock_guard<std::mutex> lock(mutex_);
        pending_for(kind).active = false;
      }
      action();
    },
    group_);
}

void Scheduler::schedule_at_message_time(
  Kind kind, rcutils_time_point_value_t at_ns, Mode mode, const std::string & tracking_topic)
{
  // Retire a node-time timer of this kind: the newest schedule replaces it.
  if (auto & timer = slot(kind)) {
    timer->cancel();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  pending_for(kind) = {true, at_ns, mode, tracking_topic};
  update_fast_path_locked();
}

std::vector<Scheduler::Scheduled> Scheduler::pending() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<Scheduled> out;
  const auto add = [&out](Kind kind, const Pending & p) {
      if (p.active) {
        out.push_back({kind, p.at_ns, p.mode, p.tracking_topic});
      }
    };
  add(Kind::Resume, resume_);
  add(Kind::Split, split_);
  add(Kind::Record, record_);
  return out;
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
  update_fast_path_locked();
  return fired;
}

void Scheduler::clear()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    resume_ = {};
    split_ = {};
    update_fast_path_locked();
  }
  for (auto * timer : {&resume_timer_, &split_timer_}) {
    if (*timer) {
      (*timer)->cancel();
    }
  }
}

}  // namespace rosbag2_dynamic_recorder
