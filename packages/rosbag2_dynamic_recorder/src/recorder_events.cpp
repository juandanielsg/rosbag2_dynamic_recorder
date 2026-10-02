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

// The recorder's events, its storage guards, and its periodic reports.

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_dynamic_recorder/dynamic_recorder.hpp"

namespace rosbag2_dynamic_recorder
{

namespace
{
/// Event history kept for late joiners. The UI's own history depth is chosen against it.
constexpr size_t kEventHistoryDepth = 10;
/// debug_timings operations kept for a slow reader.
constexpr size_t kDebugTimingsDepth = 100;
}  // namespace

void DynamicRecorder::create_event_publishers()
{
  // Transient-local, so a late subscriber still sees recent events.
  const auto event_qos = rclcpp::QoS(kEventHistoryDepth).transient_local();
  pub_subscription_change_ =
    create_publisher<SubscriptionChangeEvent>("~/events/subscription_change", event_qos);
  pub_pause_ = create_publisher<PauseEvent>("~/events/pause", event_qos);
  pub_write_split_ = create_publisher<WriteSplitEvent>("~/events/write_split", event_qos);
  pub_file_split_ = create_publisher<FileSplitEvent>("~/events/file_split", event_qos);
  pub_low_disk_ = create_publisher<LowDiskEvent>("~/events/low_disk", event_qos);
  pub_bag_size_limit_ = create_publisher<BagSizeLimitEvent>("~/events/bag_size_limit", event_qos);
  // Volatile: each report carries the losses since the previous one, so replaying old reports to
  // a late joiner would count them twice.
  pub_messages_lost_ = create_publisher<MessagesLostEvent>(
    "~/events/messages_lost", rclcpp::QoS(kEventHistoryDepth));
  // Latched, so a client that joins mid-recording gets the current state at once.
  pub_status_ = create_publisher<RecorderStatus>("~/status", rclcpp::QoS(1).transient_local());
  if (config_.debug_timings) {
    // Transient-local, so a reader still discovering the recorder gets the startup operations.
    pub_debug_timings_ = create_publisher<DebugTimings>(
      "~/debug/timings", rclcpp::QoS(kDebugTimingsDepth).transient_local());
  }
}

template<typename EventT>
void DynamicRecorder::record_event(
  const typename rclcpp::Publisher<EventT>::SharedPtr & publisher, const EventT & event)
{
  // One serializer per event type, built on first use.
  static const rclcpp::Serialization<EventT> serialization;
  rclcpp::SerializedMessage serialized;
  serialization.serialize_message(&event, &serialized);
  Timings::phase_here("event_publish");
  // Not gated on paused_: a change while paused, or the pause itself, still has to be explained.
  bag_.write_event(
    publisher->get_topic_name(), rosidl_generator_traits::name<EventT>(),
    std::make_shared<const rclcpp::SerializedMessage>(std::move(serialized)),
    rclcpp::Time(event.stamp).nanoseconds());
}

template<typename EventT>
void DynamicRecorder::emit_event(
  const typename rclcpp::Publisher<EventT>::SharedPtr & publisher, bool record, EventT & event)
{
  event.stamp = now();
  event.node_name = get_fully_qualified_name();
  if (!publisher) {
    return;
  }
  publisher->publish(event);
  if (record) {
    record_event(publisher, event);
  }
}

void DynamicRecorder::emit_subscription_change(
  const std::string & topic, const std::string & type, uint8_t action,
  const std::string & reason)
{
  SubscriptionChangeEvent event;
  event.topic_name = topic;
  event.topic_type = type;
  event.action = action;
  event.reason = reason;
  // In the bag, a channel that stops mid-recording is otherwise indistinguishable from lost data.
  emit_event(pub_subscription_change_, config_.record_subscription_events, event);
}

void DynamicRecorder::emit_pause_event(uint8_t action, const std::string & reason)
{
  PauseEvent event;
  event.action = action;
  event.reason = reason;
  // A pause leaves a hole in every topic at once, as a crash or a stall does.
  emit_event(pub_pause_, config_.record_pause_events, event);
}

void DynamicRecorder::emit_initial_pause_state()
{
  if (paused_.load()) {
    emit_pause_event(PauseEvent::PAUSED, "startup");
  }
}

Bag::EventCallbacks DynamicRecorder::writer_event_callbacks()
{
  Bag::EventCallbacks callbacks;
  callbacks.write_split_callback =
    [this](rosbag2_cpp::bag_events::BagSplitInfo & info) {
      WriteSplitEvent event;
      event.closed_file = info.closed_file;
      event.opened_file = info.opened_file;
      event.node_name = get_fully_qualified_name();
      if (pub_write_split_) {
        pub_write_split_->publish(event);
      }
      RCLCPP_INFO(get_logger(), "Bag split: '%s' -> '%s'",
        info.closed_file.c_str(), info.opened_file.c_str());
    };
#if ROSBAG2_DYNAMIC_RECORDER_HAS_WRITER_MESSAGES_LOST
  // The writer's own losses (a full cache, a failed write), kept apart from the transport's
  // because the remedy is local: the cache, the storage preset, the topic set or the disk.
  callbacks.messages_lost_callback =
    [this](const std::vector<rosbag2_cpp::bag_events::MessagesLostInfo> & infos) {
      for (const auto & info : infos) {
        losses_.note_recorder_loss(info.topic_name, info.num_messages_lost);
      }
    };
#endif
  // Jazzy and Kilted writers do not report their losses: there, lost_in_recorder is unknown.
  return callbacks;
}

void DynamicRecorder::on_rollover(
  const rosbag2_cpp::bag_events::BagSplitInfo & info, const std::string & reason)
{
  FileSplitEvent event;
  event.stamp = now();
  event.node_name = get_fully_qualified_name();
  event.closed_file = info.closed_file;
  event.opened_file = info.opened_file;
  if (!reason.empty()) {
    event.reason = reason;
  } else {
    // The writer rolled over on its own, at whichever limit is set.
    const bool by_size = config_.storage.max_bagfile_size > 0;
    const bool by_duration = config_.storage.max_bagfile_duration > 0;
    event.reason = by_size && !by_duration ? "limit:max_bagfile_size" :
      (by_duration && !by_size ? "limit:max_bagfile_duration" : "limit");
  }
  if (!pub_file_split_) {
    return;
  }
  pub_file_split_->publish(event);
  if (!config_.record_split_events) {
    return;
  }
  std::lock_guard<std::mutex> lock(deferred_mutex_);
  deferred_split_events_.push_back(std::move(event));
  if (!deferred_timer_ || deferred_timer_->is_canceled()) {
    deferred_timer_ = create_wall_timer(
      std::chrono::nanoseconds(0), [this]() {record_deferred_events();}, service_callback_group_);
  }
}

void DynamicRecorder::record_deferred_events()
{
  std::vector<FileSplitEvent> events;
  {
    std::lock_guard<std::mutex> lock(deferred_mutex_);
    if (deferred_timer_) {
      deferred_timer_->cancel();
    }
    events.swap(deferred_split_events_);
  }
  // Into the file the rollover opened, stamped when it happened.
  for (const auto & event : events) {
    record_event(pub_file_split_, event);
  }
}

void DynamicRecorder::check_free_space()
{
  if (!is_recording()) {
    return;
  }
  const auto low = storage_guard_->low_disk(bag_.uri());
  if (!low) {
    return;
  }
  RCLCPP_ERROR(get_logger(),
    "Free space on the bag's filesystem is %llu bytes, below the %llu byte minimum; stopping "
    "recording to leave the disk usable for the system",
    static_cast<unsigned long long>(low->space.free_bytes),
    static_cast<unsigned long long>(low->threshold));
  LowDiskEvent event;
  event.action = LowDiskEvent::STOPPED;
  event.free_space_bytes = low->space.free_bytes;
  event.total_space_bytes = low->space.total_bytes;
  event.min_free_space = config_.min_free_space;
  event.min_free_space_percent = config_.min_free_space_percent;
  event.reason = "timer";
  // Emit, then stop, so the explanation is inside the recording it ends. Writing it spends a
  // little of the reserve, which is what the minimum is for.
  emit_event(pub_low_disk_, config_.record_low_disk_events, event);
  stopped_for_low_disk_.store(true);
  stop();
}

void DynamicRecorder::check_bag_size()
{
  if (!is_recording()) {
    return;
  }
  const auto too_big = storage_guard_->bag_too_big(bag_.uri());
  if (!too_big) {
    return;
  }
  RCLCPP_ERROR(get_logger(),
    "The bag is %llu bytes on disk, past the %llu byte limit; stopping recording",
    static_cast<unsigned long long>(too_big->bag_size),
    static_cast<unsigned long long>(config_.max_bag_size));
  BagSizeLimitEvent event;
  event.action = BagSizeLimitEvent::STOPPED;
  event.bag_size_bytes = too_big->bag_size;
  event.max_bag_size = config_.max_bag_size;
  event.reason = "timer";
  // Emit, then stop, as the disk guard does.
  emit_event(pub_bag_size_limit_, config_.record_bag_size_limit_events, event);
  stopped_for_max_bag_size_.store(true);
  stop();
}

void DynamicRecorder::publish_messages_lost()
{
  const auto losses = losses_.drain();
  if (losses.empty()) {
    return;  // Nothing lost, nothing reported, as upstream does.
  }
  MessagesLostEvent event;
  event.node_name = get_fully_qualified_name();
  for (const auto & loss : losses) {
    auto & stat = event.messages_lost_statistics.emplace_back();
    stat.topic_name = loss.topic;
    stat.messages_lost_in_transport = loss.in_transport;
    stat.messages_lost_in_recorder = loss.in_recorder;
  }
  pub_messages_lost_->publish(event);
}

void DynamicRecorder::publish_debug_timings()
{
  for (auto & record : timings_->drain_operations()) {
    DebugTimings msg;
    msg.operation = std::move(record.operation);
    msg.start_mono_ns = record.start_mono_ns;
    for (const auto & phase : record.phases) {
      msg.phase_names.emplace_back(phase.name);
      msg.phase_ns.push_back(phase.ns);
    }
    pub_debug_timings_->publish(msg);
  }
  const auto contention = timings_->drain_contention();
  DebugTimings msg;
  msg.operation = "periodic";
  msg.start_mono_ns = contention.since_mono_ns;
  msg.lock_waits = contention.waits;
  msg.lock_wait_ns = contention.wait_ns;
  msg.lock_wait_max_ns = contention.max_wait_ns;
  msg.staged = contention.staged;
  msg.stage_dropped = contention.stage_dropped;
  pub_debug_timings_->publish(msg);
}

}  // namespace rosbag2_dynamic_recorder
