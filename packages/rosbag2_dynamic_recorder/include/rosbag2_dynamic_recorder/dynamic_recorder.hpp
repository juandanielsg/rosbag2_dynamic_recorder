// Copyright 2026 juandanielsg
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

#ifndef ROSBAG2_DYNAMIC_RECORDER__DYNAMIC_RECORDER_HPP_
#define ROSBAG2_DYNAMIC_RECORDER__DYNAMIC_RECORDER_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/storage_options.hpp"

#include "rosbag2_interfaces/msg/messages_lost_event.hpp"
#include "rosbag2_interfaces/msg/write_split_event.hpp"
#include "rosbag2_interfaces/srv/is_paused.hpp"
#include "rosbag2_interfaces/srv/pause.hpp"
#include "rosbag2_interfaces/srv/record.hpp"
#include "rosbag2_interfaces/srv/resume.hpp"
#include "rosbag2_interfaces/srv/snapshot.hpp"
#include "rosbag2_interfaces/srv/split_bagfile.hpp"
#include "rosbag2_interfaces/srv/stop.hpp"
#include "rosbag2_interfaces/srv/toggle_paused.hpp"
#include "rosbag2_dynamic_recorder_interfaces/msg/recorder_status.hpp"
#include "rosbag2_dynamic_recorder_interfaces/msg/subscription_change_event.hpp"
#include "rosbag2_dynamic_recorder_interfaces/srv/get_status.hpp"
#include "rosbag2_dynamic_recorder_interfaces/srv/get_subscribed_topics.hpp"
#include "rosbag2_dynamic_recorder_interfaces/srv/set_topics.hpp"
#include "rosbag2_dynamic_recorder_interfaces/srv/subscribe_topics.hpp"
#include "rosbag2_dynamic_recorder_interfaces/srv/unsubscribe_topics.hpp"

namespace rosbag2_dynamic_recorder
{

/// A recorder that owns its subscriptions, so the recorded topic set can change at runtime
/// without stopping the writer.
///
/// Unlike rosbag2_transport::Recorder, the topic set is not fixed by RecordOptions at
/// construction and there is no TopicFilter or discovery loop. Topics are added and removed
/// explicitly through services, which means a change costs a gap on the affected topic only,
/// and never a bag split.
class DynamicRecorder : public rclcpp::Node
{
public:
  explicit DynamicRecorder(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DynamicRecorder() override;

  /// Stop recording: disable all callbacks, drop subscriptions, close the writer.
  /// Idempotent. The topic selection is remembered so record() can restore it.
  void stop();

  /// Open a new bag and start recording again after stop(), restoring the topic selection that
  /// was in place when it stopped.
  ///
  /// Without this, stop() is a dead end: the writer is closed with no way to reopen it, and the
  /// only recovery is restarting the process.
  /// \param uri Optional output path. Empty reuses the configured one.
  /// \return false if already recording.
  bool record(const std::string & uri = "");

  /// Topics currently subscribed, sorted.
  std::vector<std::string> subscribed_topics() const;

  /// Pause recording. Subscriptions stay up and messages keep arriving; they are discarded
  /// rather than written, so the bag shows a gap on every topic and no channel is torn down.
  void pause();
  void resume();
  bool is_paused() const;

  /// Close the current bag file and open the next one. Recording continues throughout.
  bool split_bagfile();

  /// Flush the in-memory circular buffer to disk. Requires snapshot_mode.
  bool take_snapshot();

  /// Total messages dropped by the transport layer since startup, across all topics.
  uint64_t total_messages_lost() const;

  /// Messages detected as missing via gaps in publisher sequence numbers. Independent of whether
  /// the transport reported anything, so it catches loss that raises no event.
  uint64_t total_messages_missed() const;

private:
  using SubscribeTopics = rosbag2_dynamic_recorder_interfaces::srv::SubscribeTopics;
  using UnsubscribeTopics = rosbag2_dynamic_recorder_interfaces::srv::UnsubscribeTopics;
  using SetTopics = rosbag2_dynamic_recorder_interfaces::srv::SetTopics;
  using GetStatus = rosbag2_dynamic_recorder_interfaces::srv::GetStatus;
  using GetSubscribedTopics =
    rosbag2_dynamic_recorder_interfaces::srv::GetSubscribedTopics;
  using Pause = rosbag2_interfaces::srv::Pause;
  using Record = rosbag2_interfaces::srv::Record;
  using Resume = rosbag2_interfaces::srv::Resume;
  using TogglePaused = rosbag2_interfaces::srv::TogglePaused;
  using IsPaused = rosbag2_interfaces::srv::IsPaused;
  using SplitBagfile = rosbag2_interfaces::srv::SplitBagfile;
  using Snapshot = rosbag2_interfaces::srv::Snapshot;
  using Stop = rosbag2_interfaces::srv::Stop;
  using SubscriptionChangeEvent =
    rosbag2_dynamic_recorder_interfaces::msg::SubscriptionChangeEvent;
  using RecorderStatus = rosbag2_dynamic_recorder_interfaces::msg::RecorderStatus;
  using WriteSplitEvent = rosbag2_interfaces::msg::WriteSplitEvent;
  using MessagesLostEvent = rosbag2_interfaces::msg::MessagesLostEvent;

  /// Resolve a topic's type from the ROS graph.
  /// \return the type, or nullopt if the topic is absent or offers more than one type.
  std::optional<std::string> resolve_type(const std::string & topic_name) const;

  /// Create the writer channel and the subscription for one topic.
  /// \return true if the topic is subscribed on return (including if it already was).
  bool subscribe_topic(const std::string & topic_name, const std::string & topic_type);

  /// Disable callbacks and drop the subscription. The writer channel is left in place, so the
  /// topic keeps its existing messages and becomes a sparse channel.
  /// \return false if the topic was not subscribed.
  bool unsubscribe_topic(const std::string & topic_name);

  /// Shared by subscribe_topics and set_topics: resolve types, subscribe, and fill the
  /// subscribed/unavailable lists.
  void subscribe_batch(
    const std::vector<std::string> & topics,
    const std::vector<std::string> & topic_types,
    std::vector<std::string> & subscribed_out,
    std::vector<std::string> & unavailable_out);

  void handle_subscribe_topics(
    const std::shared_ptr<SubscribeTopics::Request> request,
    std::shared_ptr<SubscribeTopics::Response> response);
  void handle_unsubscribe_topics(
    const std::shared_ptr<UnsubscribeTopics::Request> request,
    std::shared_ptr<UnsubscribeTopics::Response> response);
  void handle_set_topics(
    const std::shared_ptr<SetTopics::Request> request,
    std::shared_ptr<SetTopics::Response> response);
  void handle_get_subscribed_topics(
    const std::shared_ptr<GetSubscribedTopics::Request> request,
    std::shared_ptr<GetSubscribedTopics::Response> response);
  void handle_pause(
    const std::shared_ptr<Pause::Request> request, std::shared_ptr<Pause::Response> response);
  void handle_resume(
    const std::shared_ptr<Resume::Request> request, std::shared_ptr<Resume::Response> response);
  void handle_toggle_paused(
    const std::shared_ptr<TogglePaused::Request> request,
    std::shared_ptr<TogglePaused::Response> response);
  void handle_is_paused(
    const std::shared_ptr<IsPaused::Request> request,
    std::shared_ptr<IsPaused::Response> response);
  void handle_split_bagfile(
    const std::shared_ptr<SplitBagfile::Request> request,
    std::shared_ptr<SplitBagfile::Response> response);
  void handle_snapshot(
    const std::shared_ptr<Snapshot::Request> request,
    std::shared_ptr<Snapshot::Response> response);
  void handle_stop(
    const std::shared_ptr<Stop::Request> request, std::shared_ptr<Stop::Response> response);
  void handle_record(
    const std::shared_ptr<Record::Request> request, std::shared_ptr<Record::Response> response);

  /// True while a bag is open. Used to reject topic changes with an accurate reason rather than
  /// reporting the topics themselves as unavailable.
  bool is_recording() const;
  void handle_get_status(
    const std::shared_ptr<GetStatus::Request> request,
    std::shared_ptr<GetStatus::Response> response);

  /// Sum of the file sizes in the bag directory. Returns 0 rather than throwing if the directory
  /// cannot be read, since a status call must not fail just because of a stat error.
  uint64_t bag_size_bytes() const;

  /// Single source of truth for both ~/status and ~/get_status, so the two cannot drift.
  RecorderStatus build_status() const;

  /// Publish current state. Called on a timer and after every mutating operation, so a UI sees
  /// a change immediately rather than up to a tick later.
  void publish_status();

  /// Publish a subscription change on ~/events/subscription_change and, unless disabled, write it
  /// into the bag so the resulting sparse channel explains itself.
  void emit_subscription_change(
    const std::string & topic_name, const std::string & topic_type, uint8_t action,
    const std::string & reason);

  /// Register write-split and messages-lost callbacks on the writer.
  void setup_writer_events();

  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  /// Guards writer_ access and recording_. rosbag2_cpp::Writer has its own internal lock, but we
  /// need recording_ and the write call to be consistent: without this, a write can land after
  /// stop() has closed the writer.
  mutable std::mutex writer_mutex_;
  bool recording_{false};

  /// Topics for which a writer channel already exists. create_topic() is idempotent, but
  /// resolving a message definition costs ~300-470ms, so skipping it matters.
  std::unordered_map<std::string, std::string> known_channels_;

  std::unordered_map<std::string, rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  mutable std::mutex subscriptions_mutex_;

  /// Services run here, separate from the default group used by subscription callbacks, so that
  /// the ~0.5s cost of adding a topic cannot block message delivery.
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;

  rclcpp::Service<SubscribeTopics>::SharedPtr srv_subscribe_topics_;
  rclcpp::Service<UnsubscribeTopics>::SharedPtr srv_unsubscribe_topics_;
  rclcpp::Service<SetTopics>::SharedPtr srv_set_topics_;
  rclcpp::Service<GetSubscribedTopics>::SharedPtr srv_get_subscribed_topics_;
  rclcpp::Service<Pause>::SharedPtr srv_pause_;
  rclcpp::Service<Resume>::SharedPtr srv_resume_;
  rclcpp::Service<TogglePaused>::SharedPtr srv_toggle_paused_;
  rclcpp::Service<IsPaused>::SharedPtr srv_is_paused_;
  rclcpp::Service<SplitBagfile>::SharedPtr srv_split_bagfile_;
  rclcpp::Service<Snapshot>::SharedPtr srv_snapshot_;
  rclcpp::Service<Stop>::SharedPtr srv_stop_;
  rclcpp::Service<Record>::SharedPtr srv_record_;
  rclcpp::Service<GetStatus>::SharedPtr srv_get_status_;

  rclcpp::Publisher<SubscriptionChangeEvent>::SharedPtr pub_subscription_change_;
  rclcpp::Publisher<WriteSplitEvent>::SharedPtr pub_write_split_;
  rclcpp::Publisher<MessagesLostEvent>::SharedPtr pub_messages_lost_;
  rclcpp::Publisher<RecorderStatus>::SharedPtr pub_status_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  rclcpp::Serialization<SubscriptionChangeEvent> subscription_change_serialization_;

  /// Checked in the write path. Atomic because it is read on every message and written from a
  /// service callback on a different thread.
  std::atomic_bool paused_{false};

  /// Last publication sequence number seen, keyed topic -> publisher GID -> sequence.
  ///
  /// Sequence numbers are per PUBLISHER, not per topic. Keying by topic alone is wrong the moment
  /// a topic has more than one publisher -- /tf routinely does -- because the independent counters
  /// interleave and every alternation looks like an enormous gap. That mistake produced a reported
  /// 201,027,600 missing messages against 12,728 written.
  ///
  /// Cleared per topic on unsubscribe: publishers keep counting while we are not listening, and
  /// on re-subscribe that jump is deliberate, not loss.
  std::unordered_map<std::string, std::unordered_map<std::string, uint64_t>> last_publication_seq_;
  std::mutex sequence_mutex_;
  std::atomic_uint64_t messages_missed_{0};
  std::atomic_bool sequence_numbers_available_{false};

  /// Per-topic transport-layer losses accumulated since the last MessagesLostEvent.
  std::unordered_map<std::string, uint64_t> messages_lost_since_last_event_;
  std::atomic_uint64_t total_messages_lost_{0};
  std::mutex messages_lost_mutex_;
  rclcpp::TimerBase::SharedPtr messages_lost_timer_;

  /// Reason stamped onto the next SubscriptionChangeEvent. Safe as shared state because every
  /// service handler runs in service_callback_group_, which is MutuallyExclusive: only one
  /// topic-changing operation is ever in flight.
  std::string current_reason_{"startup"};

  std::string serialization_format_;
  bool record_subscription_events_{true};
  bool snapshot_mode_{false};

  std::string uri_;
  std::string storage_id_;
  /// Kept so a new bag can be opened after stop() without reconstructing the node.
  rosbag2_storage::StorageOptions storage_options_;
  rosbag2_cpp::ConverterOptions converter_options_;
  /// Topic selection at the moment of stop(), restored by record().
  std::vector<std::string> topics_at_stop_;
  rclcpp::Time recording_started_;
  std::atomic_uint64_t messages_written_{0};
  std::atomic_uint64_t bag_splits_{0};
};

}  // namespace rosbag2_dynamic_recorder

#endif  // ROSBAG2_DYNAMIC_RECORDER__DYNAMIC_RECORDER_HPP_
