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

#include "rosbag2_dynamic_recorder/bag.hpp"

#include <memory>
#include <string>
#include <utility>

namespace rosbag2_dynamic_recorder
{

Bag::Bag(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
: logger_(std::move(logger)), clock_(std::move(clock)),
  // On the node's clock type from the start: before the first open it is still subtracted from
  // now() for the status, and rclcpp refuses arithmetic across clock types.
  opened_at_(int64_t{0}, clock_->get_clock_type())
{
}

std::string Bag::open(
  const rosbag2_storage::StorageOptions & options,
  const rosbag2_cpp::ConverterOptions & converter, const EventCallbacks & callbacks,
  rclcpp::Time opened_at)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (open_) {
    return "already recording";
  }
  auto writer = std::make_unique<rosbag2_cpp::Writer>();
  try {
    writer->open(options, converter);
  } catch (const std::exception & e) {
    // Bad path, no permission. Reported rather than thrown so the caller can stay alive and
    // try again with a better path.
    return "could not open '" + options.uri + "': " + e.what();
  }
  // The split count is this bag's own; the caller's callback still runs after it.
  EventCallbacks counted = callbacks;
  counted.write_split_callback =
    [this, notify = callbacks.write_split_callback](rosbag2_cpp::bag_events::BagSplitInfo & info) {
      splits_.fetch_add(1, std::memory_order_relaxed);
      if (notify) {
        notify(info);
      }
    };
  writer->add_event_callbacks(counted);

  writer_ = std::move(writer);
  open_ = true;
  uri_ = options.uri;
  serialization_format_ = converter.output_serialization_format;
  opened_at_ = opened_at;
  channels_.clear();
  messages_written_.store(0, std::memory_order_relaxed);
  write_errors_.store(0, std::memory_order_relaxed);
  splits_.store(0, std::memory_order_relaxed);
  return "";
}

bool Bag::close()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!open_) {
    return false;
  }
  open_ = false;
  writer_->close();
  return true;
}

bool Bag::is_open() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return open_;
}

std::string Bag::uri() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return uri_;
}

Bag::Info Bag::info() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return {
    open_, uri_, opened_at_,
    messages_written_.load(std::memory_order_relaxed),
    write_errors_.load(std::memory_order_relaxed),
    splits_.load(std::memory_order_relaxed),
  };
}

std::optional<std::string> Bag::channel_type(const std::string & topic) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto known = channels_.find(topic);
  if (known == channels_.end()) {
    return std::nullopt;
  }
  return known->second;
}

std::string Bag::ensure_channel(const rosbag2_storage::TopicMetadata & metadata)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!open_) {
    return "not recording";
  }
  const auto known = channels_.find(metadata.name);
  if (known != channels_.end()) {
    if (known->second != metadata.type) {
      return "'" + metadata.name + "' is already in this bag as '" + known->second +
             "'; it now publishes '" + metadata.type +
             "'. Start a new bag to record the new type.";
    }
    return "";
  }
  // Resolves the message definition internally, which is the expensive part of adding a topic.
  try {
    writer_->create_topic(metadata);
  } catch (const std::exception & e) {
    return "could not create a channel for '" + metadata.name + "': " + e.what();
  }
  channels_.emplace(metadata.name, metadata.type);
  return "";
}

void Bag::write(
  std::shared_ptr<const rclcpp::SerializedMessage> message, const std::string & topic,
  const std::string & type, rcutils_time_point_value_t recv_ns, rcutils_time_point_value_t send_ns)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!open_) {
    return;
  }
  try {
    writer_->write(std::move(message), topic, type, recv_ns, send_ns);
    messages_written_.fetch_add(1, std::memory_order_relaxed);
  } catch (const std::exception & e) {
    write_errors_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 5000,
      "Failed to write a message on '%s': %s (%lu write errors so far)",
      topic.c_str(), e.what(),
      static_cast<unsigned long>(write_errors_.load(std::memory_order_relaxed)));
  }
}

void Bag::write_event(
  const std::string & topic, const std::string & type,
  std::shared_ptr<const rclcpp::SerializedMessage> message, rcutils_time_point_value_t stamp_ns)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!open_) {
    return;
  }
  if (channels_.count(topic) == 0) {
    try {
      writer_->create_topic({0u, topic, type, serialization_format_, {}, ""});
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Could not create the event channel '%s': %s", topic.c_str(),
        e.what());
      return;
    }
    channels_.emplace(topic, type);
  }
  try {
    writer_->write(std::move(message), topic, type, stamp_ns, stamp_ns);
    messages_written_.fetch_add(1, std::memory_order_relaxed);
  } catch (const std::exception & e) {
    write_errors_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_ERROR(logger_, "Could not record an event on '%s': %s", topic.c_str(), e.what());
  }
}

bool Bag::split()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!open_) {
    return false;
  }
  try {
    writer_->split_bagfile();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to split the bag: %s", e.what());
    return false;
  }
  return true;
}

bool Bag::take_snapshot()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!open_) {
    return false;
  }
  try {
    return writer_->take_snapshot();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Snapshot failed: %s", e.what());
    return false;
  }
}

}  // namespace rosbag2_dynamic_recorder
