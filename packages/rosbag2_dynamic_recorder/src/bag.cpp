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

#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>

namespace rosbag2_dynamic_recorder
{

namespace
{
using Lock = std::lock_guard<std::timed_mutex>;

// How often a message waiting for the writer looks again at whether it turned busy, and should be
// staged rather than wait. Bounds how long a message can wait for a slow operation it did not see
// start: the writer is only ever held briefly otherwise.
constexpr auto kBusyPoll = std::chrono::microseconds(200);

// The stage's size when the writer has no cache to take it from.
constexpr size_t kDefaultMaxStagedBytes = 64 * 1024 * 1024;

// How many "(N)" suffixes unused_bag_path() tries.
constexpr int kMaxBagPathSuffix = 10000;
}  // namespace

std::optional<std::string> unused_bag_path(const std::string & uri)
{
  namespace fs = std::filesystem;
  std::error_code ec;
  if (!fs::exists(uri, ec)) {
    return uri;
  }
  for (int i = 1; i < kMaxBagPathSuffix; ++i) {
    fs::path candidate(uri);
    candidate += "(" + std::to_string(i) + ")";
    if (!fs::exists(candidate, ec)) {
      return candidate.generic_string();
    }
  }
  return std::nullopt;
}

Bag::Busy::Busy(Bag & bag)
: bag_(bag)
{
  bag_.busy_.store(true, std::memory_order_release);
}

Bag::Busy::~Busy()
{
  // Clear first, then write what was staged: a message that sees the flag clear waits for the
  // lock, which is still held for the flush, and so lands after everything staged before it.
  bag_.busy_.store(false, std::memory_order_release);
  bag_.flush_staged_locked();
}

Bag::Bag(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
: logger_(std::move(logger)), clock_(std::move(clock)),
  // On the node's clock type from the start: before the first open it is still subtracted from
  // now() for the status, and rclcpp refuses arithmetic across clock types.
  opened_at_(int64_t{0}, clock_->get_clock_type()),
  closed_at_(int64_t{0}, clock_->get_clock_type())
{
}

std::string Bag::open(
  const rosbag2_storage::StorageOptions & options,
  const rosbag2_cpp::ConverterOptions & converter, const EventCallbacks & callbacks,
  rclcpp::Time opened_at)
{
  Lock lock(mutex_);
  Timings::phase_here("open:lock_wait");
  if (open_) {
    return "already recording";
  }
  auto writer = std::make_unique<rosbag2_cpp::Writer>();
  try {
    writer->open(options, converter);
    Timings::phase_here("writer_open");
  } catch (const std::exception & e) {
    // Bad path, no permission. Reported rather than thrown so the caller can stay alive and
    // try again with a better path.
    return "could not open '" + options.uri + "': " + e.what();
  }
  // The split count is this bag's own; the caller's callback still runs after it.
  EventCallbacks counted = callbacks;
  counted.write_split_callback =
    [this, notify = callbacks.write_split_callback](rosbag2_cpp::bag_events::BagSplitInfo & info) {
      // The writer reports closing the bag the same way, with no file opened after it. That is
      // the end of the recording, not a rollover: counting it put one file too many in the status
      // of every stopped recorder.
      const bool rollover = !info.opened_file.empty();
      if (rollover) {
        splits_.fetch_add(1, std::memory_order_relaxed);
      }
      if (notify) {
        notify(info);
      }
      if (rollover && on_rollover_) {
        on_rollover_(info, split_reason_);
      }
    };
  writer->add_event_callbacks(counted);

  writer_ = std::move(writer);
  open_ = true;
  uri_ = options.uri;
  serialization_format_ = converter.output_serialization_format;
  opened_at_ = opened_at;
  channels_.clear();
  max_staged_bytes_ = options.max_cache_size > 0 ?
    static_cast<size_t>(options.max_cache_size) : kDefaultMaxStagedBytes;
  messages_written_.store(0, std::memory_order_relaxed);
  write_errors_.store(0, std::memory_order_relaxed);
  splits_.store(0, std::memory_order_relaxed);
  return "";
}

bool Bag::close()
{
  Lock lock(mutex_);
  Timings::phase_here("close:lock_wait");
  if (!open_) {
    return false;
  }
  // The moment the recording ends, before the flush and close that can take seconds on a slow
  // disk: nothing arriving after it is written.
  closed_at_ = clock_->now();
  {
    Busy busy(*this);
    // What arrived before the stop is written; what arrives while the writer flushes and closes
    // came after it and is not.
    flush_staged_locked();
    open_ = false;
    writer_->close();
    Timings::phase_here("writer_close");
  }
  return true;
}

bool Bag::is_open() const
{
  Lock lock(mutex_);
  return open_;
}

std::string Bag::uri() const
{
  Lock lock(mutex_);
  return uri_;
}

Bag::Info Bag::info() const
{
  Lock lock(mutex_);
  return {
    open_, uri_, opened_at_, closed_at_,
    messages_written_.load(std::memory_order_relaxed),
    write_errors_.load(std::memory_order_relaxed),
    splits_.load(std::memory_order_relaxed),
  };
}

std::optional<std::string> Bag::channel_type(const std::string & topic) const
{
  Lock lock(mutex_);
  const auto known = channels_.find(topic);
  if (known == channels_.end()) {
    return std::nullopt;
  }
  return known->second;
}

std::string Bag::ensure_channel(const rosbag2_storage::TopicMetadata & metadata)
{
  Lock lock(mutex_);
  Timings::phase_here("channel:lock_wait");
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
  Busy busy(*this);
  return create_channel_locked(metadata);
}

std::string Bag::create_channel_locked(const rosbag2_storage::TopicMetadata & metadata)
{
  // Resolves the message definition and writes the channel to storage, behind rosbag2's storage
  // mutex: as slow as a disk stall. The caller holds the writer busy, so messages are staged.
  try {
    writer_->create_topic(metadata);
    Timings::phase_here("create_topic");
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
  std::unique_lock<std::timed_mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    // Only a message that finds the lock held is timed, so an uncontended write costs exactly
    // what it did before.
    const int64_t start = timings_ != nullptr ? Timings::now_ns() : 0;
    while (!lock.try_lock_for(kBusyPoll)) {
      if (busy_.load(std::memory_order_acquire)) {
        if (timings_ != nullptr) {
          timings_->note_lock_wait(Timings::now_ns() - start);
        }
        stage({std::move(message), topic, type, recv_ns, send_ns});
        return;
      }
    }
    if (timings_ != nullptr) {
      timings_->note_lock_wait(Timings::now_ns() - start);
    }
  }
  if (!open_) {
    return;
  }
  // Anything staged arrived earlier, so goes first.
  if (staged_count_.load(std::memory_order_acquire) > 0) {
    flush_staged_locked();
  }
  write_locked(std::move(message), topic, type, recv_ns, send_ns);
}

void Bag::write_locked(
  std::shared_ptr<const rclcpp::SerializedMessage> message, const std::string & topic,
  const std::string & type, rcutils_time_point_value_t recv_ns, rcutils_time_point_value_t send_ns)
{
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

void Bag::stage(Staged && staged)
{
  const size_t bytes = staged.message->size();
  {
    std::lock_guard<std::mutex> lock(staged_mutex_);
    if (staged_bytes_ + bytes <= max_staged_bytes_) {
      staged_bytes_ += bytes;
      staged_.push_back(std::move(staged));
      staged_count_.fetch_add(1, std::memory_order_release);
      if (timings_ != nullptr) {
        timings_->note_staged(false);
      }
      return;
    }
  }
  // The stage is as large as the writer's cache: a slow operation outlasting it would have
  // overflowed the cache too. Counted, like the writer's own losses, as lost in the recorder.
  if (timings_ != nullptr) {
    timings_->note_staged(true);
  }
  if (on_lost_) {
    on_lost_(staged.topic, 1);
  }
}

void Bag::flush_staged_locked()
{
  std::deque<Staged> batch;
  {
    std::lock_guard<std::mutex> lock(staged_mutex_);
    batch.swap(staged_);
    staged_bytes_ = 0;
    staged_count_.store(0, std::memory_order_release);
  }
  if (!open_) {
    return;  // stopped: what was staged after the stop is not recorded
  }
  for (auto & s : batch) {
    write_locked(std::move(s.message), s.topic, s.type, s.recv_ns, s.send_ns);
  }
}

void Bag::write_event(
  const std::string & topic, const std::string & type,
  std::shared_ptr<const rclcpp::SerializedMessage> message, rcutils_time_point_value_t stamp_ns)
{
  Lock lock(mutex_);
  Timings::phase_here("event:lock_wait");
  if (!open_) {
    return;
  }
  Busy busy(*this);
  if (channels_.count(topic) == 0) {
    const auto error = create_channel_locked({0u, topic, type, serialization_format_, {}, ""});
    if (!error.empty()) {
      RCLCPP_ERROR(logger_, "%s", error.c_str());
      return;
    }
  }
  write_locked(std::move(message), topic, type, stamp_ns, stamp_ns);
  Timings::phase_here("event_write");
}

bool Bag::split(const std::string & reason)
{
  Lock lock(mutex_);
  Timings::phase_here("split:lock_wait");
  if (!open_) {
    return false;
  }
  Busy busy(*this);
  // Read by the rollover callback, which runs inside split_bagfile() on this thread. Any other
  // rollover is the writer's own, at a limit, and finds it empty.
  split_reason_ = reason;
  bool ok = true;
  try {
    writer_->split_bagfile();
    Timings::phase_here("writer_split");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to split the bag: %s", e.what());
    ok = false;
  }
  split_reason_.clear();
  return ok;
}

bool Bag::take_snapshot()
{
  Lock lock(mutex_);
  if (!open_) {
    return false;
  }
  Busy busy(*this);
  try {
    return writer_->take_snapshot();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Snapshot failed: %s", e.what());
    return false;
  }
}

}  // namespace rosbag2_dynamic_recorder
