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

#ifndef ROSBAG2_DYNAMIC_RECORDER__DYNAMIC_RECORDER_HPP_
#define ROSBAG2_DYNAMIC_RECORDER__DYNAMIC_RECORDER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_dynamic_recorder/bag.hpp"
#include "rosbag2_dynamic_recorder/loss_accounting.hpp"
#include "rosbag2_dynamic_recorder/recorder_config.hpp"
#include "rosbag2_dynamic_recorder/scheduler.hpp"
#include "rosbag2_dynamic_recorder/storage_guard.hpp"
#include "rosbag2_storage/storage_options.hpp"

#include "rosbag2_interfaces/msg/write_split_event.hpp"
#include "rosbag2_interfaces/srv/is_paused.hpp"
#include "rosbag2_interfaces/srv/pause.hpp"
#include "rosbag2_interfaces/srv/snapshot.hpp"
#include "rosbag2_interfaces/srv/toggle_paused.hpp"
// The ROSBAG2_DYNAMIC_RECORDER_HAS_UPSTREAM_* macros are set by CMake from the installed
// rosbag2_interfaces headers. Each stock type is used where its definition is field-identical to
// the one this project was written against (rosbag2 0.34, Rolling), so a client written for
// stock rosbag2 can drive the recorder; elsewhere -- Jazzy and Kilted, whose Record, Resume and
// SplitBagfile predate scheduling, whose Stop has no return code and which have no
// MessagesLostEvent -- the field-identical copy in rosbag2_dynamic_recorder_interfaces stands in.
#if ROSBAG2_DYNAMIC_RECORDER_HAS_UPSTREAM_MESSAGES_LOST_EVENT
#include "rosbag2_interfaces/msg/messages_lost_event.hpp"
#else
#include "rosbag2_dynamic_recorder_interfaces/msg/messages_lost_event.hpp"
#endif
#if ROSBAG2_DYNAMIC_RECORDER_HAS_UPSTREAM_RECORD
#include "rosbag2_interfaces/srv/record.hpp"
#else
#include "rosbag2_dynamic_recorder_interfaces/srv/record.hpp"
#endif
#if ROSBAG2_DYNAMIC_RECORDER_HAS_UPSTREAM_RESUME
#include "rosbag2_interfaces/srv/resume.hpp"
#else
#include "rosbag2_dynamic_recorder_interfaces/srv/resume.hpp"
#endif
#if ROSBAG2_DYNAMIC_RECORDER_HAS_UPSTREAM_SPLIT_BAGFILE
#include "rosbag2_interfaces/srv/split_bagfile.hpp"
#else
#include "rosbag2_dynamic_recorder_interfaces/srv/split_bagfile.hpp"
#endif
#if ROSBAG2_DYNAMIC_RECORDER_HAS_UPSTREAM_STOP
#include "rosbag2_interfaces/srv/stop.hpp"
#else
#include "rosbag2_dynamic_recorder_interfaces/srv/stop.hpp"
#endif
#include "rosbag2_dynamic_recorder_interfaces/msg/low_disk_event.hpp"
#include "rosbag2_dynamic_recorder_interfaces/msg/bag_size_limit_event.hpp"
#include "rosbag2_dynamic_recorder_interfaces/msg/pause_event.hpp"
#include "rosbag2_dynamic_recorder_interfaces/msg/profile.hpp"
#include "rosbag2_dynamic_recorder_interfaces/msg/recorder_status.hpp"
#include "rosbag2_dynamic_recorder_interfaces/msg/subscription_change_event.hpp"
#include "rosbag2_dynamic_recorder_interfaces/srv/get_status.hpp"
#include "rosbag2_dynamic_recorder_interfaces/srv/get_profiles.hpp"
#include "rosbag2_dynamic_recorder_interfaces/srv/get_subscribed_topics.hpp"
#include "rosbag2_dynamic_recorder_interfaces/srv/set_profile.hpp"
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

  /// Apply a named profile: record exactly its topics and nothing else.
  /// \return false if no profile of that name is configured.
  bool set_profile(const std::string & name, std::vector<std::string> & subscribed_out,
    std::vector<std::string> & unsubscribed_out, std::vector<std::string> & unavailable_out);

  /// Name of the profile whose topic list exactly matches what is being recorded, or "".
  ///
  /// Derived from the live subscription set rather than remembered. A remembered name would go
  /// stale the moment someone changed one topic by hand, and would then be a lie in the status.
  std::string active_profile() const;

  /// Pause recording. Subscriptions stay up and messages keep arriving; they are discarded
  /// rather than written, so the bag shows a gap on every topic and no channel is torn down.
  ///
  /// `reason` is stamped onto the PauseEvent, which is what makes that gap legible afterwards.
  /// It is a parameter rather than shared state because a scheduled resume fires from the message
  /// path while a service call may be in flight, so there is no single "current" reason to read.
  void pause(const std::string & reason);
  void resume(const std::string & reason);
  bool is_paused() const;

  /// Close the current bag file and open the next one. Recording continues throughout.
  bool split_bagfile();

  /// Flush the in-memory circular buffer to disk. Requires snapshot_mode.
  bool take_snapshot();

  /// Messages the transport reported as dropped before delivery, summed over all topics.
  uint64_t messages_lost_in_transport() const;

  /// Messages that reached the recorder and were then dropped by the writer -- cache overflow
  /// when the disk cannot keep up, or a storage write that failed -- summed over all topics.
  ///
  /// Kept apart from the transport figure because the remedies are opposite: transport loss
  /// points at the publisher, QoS, or the network; recorder loss points at the cache size, the
  /// storage preset, the topic set, or the disk.
  uint64_t messages_lost_in_recorder() const;

  /// messages_lost_in_transport() + messages_lost_in_recorder().
  uint64_t total_messages_lost() const;

  /// Messages detected as missing via gaps in publisher sequence numbers. Independent of whether
  /// the transport reported anything, so it catches loss that raises no event.
  uint64_t total_messages_missed() const;

private:
  using SubscribeTopics = rosbag2_dynamic_recorder_interfaces::srv::SubscribeTopics;
  using UnsubscribeTopics = rosbag2_dynamic_recorder_interfaces::srv::UnsubscribeTopics;
  using SetTopics = rosbag2_dynamic_recorder_interfaces::srv::SetTopics;
  using SetProfile = rosbag2_dynamic_recorder_interfaces::srv::SetProfile;
  using GetProfiles = rosbag2_dynamic_recorder_interfaces::srv::GetProfiles;
  using Profile = rosbag2_dynamic_recorder_interfaces::msg::Profile;
  using GetStatus = rosbag2_dynamic_recorder_interfaces::srv::GetStatus;
  using GetSubscribedTopics =
    rosbag2_dynamic_recorder_interfaces::srv::GetSubscribedTopics;
  using Pause = rosbag2_interfaces::srv::Pause;
  using TogglePaused = rosbag2_interfaces::srv::TogglePaused;
  using IsPaused = rosbag2_interfaces::srv::IsPaused;
  using Snapshot = rosbag2_interfaces::srv::Snapshot;
  // Stock or copy, decided at the includes above; the code below is written once.
#if ROSBAG2_DYNAMIC_RECORDER_HAS_UPSTREAM_RECORD
  using Record = rosbag2_interfaces::srv::Record;
#else
  using Record = rosbag2_dynamic_recorder_interfaces::srv::Record;
#endif
#if ROSBAG2_DYNAMIC_RECORDER_HAS_UPSTREAM_RESUME
  using Resume = rosbag2_interfaces::srv::Resume;
#else
  using Resume = rosbag2_dynamic_recorder_interfaces::srv::Resume;
#endif
#if ROSBAG2_DYNAMIC_RECORDER_HAS_UPSTREAM_SPLIT_BAGFILE
  using SplitBagfile = rosbag2_interfaces::srv::SplitBagfile;
#else
  using SplitBagfile = rosbag2_dynamic_recorder_interfaces::srv::SplitBagfile;
#endif
#if ROSBAG2_DYNAMIC_RECORDER_HAS_UPSTREAM_STOP
  using Stop = rosbag2_interfaces::srv::Stop;
#else
  using Stop = rosbag2_dynamic_recorder_interfaces::srv::Stop;
#endif
  using SubscriptionChangeEvent =
    rosbag2_dynamic_recorder_interfaces::msg::SubscriptionChangeEvent;
  using PauseEvent = rosbag2_dynamic_recorder_interfaces::msg::PauseEvent;
  using LowDiskEvent = rosbag2_dynamic_recorder_interfaces::msg::LowDiskEvent;
  using BagSizeLimitEvent = rosbag2_dynamic_recorder_interfaces::msg::BagSizeLimitEvent;
  using RecorderStatus = rosbag2_dynamic_recorder_interfaces::msg::RecorderStatus;
  using WriteSplitEvent = rosbag2_interfaces::msg::WriteSplitEvent;
#if ROSBAG2_DYNAMIC_RECORDER_HAS_UPSTREAM_MESSAGES_LOST_EVENT
  using MessagesLostEvent = rosbag2_interfaces::msg::MessagesLostEvent;
#else
  using MessagesLostEvent = rosbag2_dynamic_recorder_interfaces::msg::MessagesLostEvent;
#endif

  /// Resolve a topic's type from an already-fetched view of the graph.
  ///
  /// Takes the graph snapshot rather than querying, so a batch of twenty topics costs one graph
  /// query instead of twenty.
  /// \return the type, or nullopt if the topic is absent or offers more than one type.
  std::optional<std::string> resolve_type(
    const std::string & topic_name,
    const std::map<std::string, std::vector<std::string>> & graph) const;

  /// Create the writer channel and the subscription for one topic.
  ///
  /// Never throws. A bad topic name or an unloadable type would otherwise escape a service
  /// callback and take the whole process down, so failures are reported through the return value
  /// and last_failure_reason_ instead.
  /// \return true if the topic is subscribed on return (including if it already was).
  bool subscribe_topic(const std::string & topic_name, const std::string & topic_type);

  /// Disable callbacks and drop the subscription. The writer channel is left in place, so the
  /// topic keeps its existing messages and becomes a sparse channel.
  /// \return false if the topic was not subscribed.
  bool unsubscribe_topic(const std::string & topic_name);

  /// active_profile() against an already-computed topic set, so a caller that has one does not pay
  /// for a second lock/copy/sort of the subscription map.
  std::string active_profile_for(const std::vector<std::string> & current) const;

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
  /// True under use_sim_time until the first /clock message: no bag is opened before it, because
  /// everything the recorder stamps would read as time 0.
  bool waiting_for_clock() const;
  /// Why a call that needs an open bag is being refused, for the response's error_string.
  std::string stopped_reason() const;
  void handle_get_status(
    const std::shared_ptr<GetStatus::Request> request,
    std::shared_ptr<GetStatus::Response> response);
  void handle_set_profile(
    const std::shared_ptr<SetProfile::Request> request,
    std::shared_ptr<SetProfile::Response> response);
  void handle_get_profiles(
    const std::shared_ptr<GetProfiles::Request> request,
    std::shared_ptr<GetProfiles::Response> response);

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

  /// Publish a pause or resume on ~/events/pause and, unless disabled, write it into the bag so
  /// the resulting gap across every topic explains itself.
  void emit_pause_event(uint8_t action, const std::string & reason);

  /// Open the bag and subscribe the initial topics: the end of construction, deferred under
  /// use_sim_time until the clock has started.
  void start();

  /// Write a PAUSED event if the bag is opening while already paused, so a recording that starts
  /// with a hole says why. Covers both start_paused and a ~/record issued while paused.
  void emit_initial_pause_state();

  /// Stop recording if free space on the bag's filesystem has fallen below the configured minimum.
  ///
  /// Runs on the storage timer, independently of the bag's own size limits, because it protects
  /// the filesystem rather than the bag: logs, core dumps or a second recorder can fill the disk
  /// with max_bagfile_size or max_bag_size set. When triggered it emits a LowDiskEvent (published
  /// and, unless disabled, written into the bag) and then stops, in that order, so the
  /// explanation survives the stop.
  void check_free_space();

  /// Stop recording if the bag directory has grown past max_bag_size.
  ///
  /// Shares the storage timer with check_free_space() but guards the other side: the bag rather
  /// than the disk. max_bagfile_size only rolls to a new file, so without this a recording left
  /// running has no upper bound. Measures fresh rather than through the status cache.
  void check_bag_size();

  /// Stamp `event` with the node clock and name, publish it on `pub`, and if `record` is set write
  /// it into the bag as well. Every event stream goes through here: the in-bag copy is the point
  /// of the mechanism, since a gap or an end that only the topic explained would be lost with the
  /// topic. Defined in the .cpp; only used there.
  template<typename EventT>
  void emit_event(
    const typename rclcpp::Publisher<EventT>::SharedPtr & pub, bool record, EventT & event);

  /// Graph topics a pattern is allowed to match.
  ///
  /// Excludes this node's own topics: the event channels are already written into the bag
  /// directly, so subscribing to them would record every event twice. Also excludes hidden
  /// topics, matching what stock `ros2 bag record` leaves out by default.
  std::vector<std::string> patternable_graph_topics() const;

  /// Combine explicitly named topics with a regex selection over `candidates`.
  ///
  /// `exclude_regex` is applied to the COMBINED set, so it filters explicitly named topics too --
  /// that is what makes "everything under /robot except the depth camera" one call.
  ///
  /// Returns false and fills `error_out` on an invalid expression, so a typo is refused with the
  /// reason rather than silently matching nothing.
  bool resolve_selection(
    const std::vector<std::string> & requested_topics,
    const std::vector<std::string> & requested_types,
    const std::string & pattern, const std::string & exclude,
    const std::vector<std::string> & candidates,
    std::vector<std::string> & topics_out, std::vector<std::string> & types_out,
    std::string & error_out) const;

  /// The write-split and messages-lost callbacks a bag is opened with.
  Bag::EventCallbacks writer_event_callbacks();

  /// Offer `~/<name>` on the service callback group, handled by `handler`.
  template<typename SrvT>
  typename rclcpp::Service<SrvT>::SharedPtr serve(
    const std::string & name,
    void (DynamicRecorder::* handler)(
      std::shared_ptr<typename SrvT::Request>, std::shared_ptr<typename SrvT::Response>))
  {
    return create_service<SrvT>(
      "~/" + name,
      std::bind(handler, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), service_callback_group_);
  }

  /// What a scheduled service answers with. The codes are the service's own constants, passed
  /// in because Resume and SplitBagfile number theirs differently.
  struct ScheduleCodes
  {
    int32_t success;
    int32_t invalid_mode;
    int32_t invalid_topic;
    /// For an immediate action that reports failure; unused by a kind that cannot fail.
    int32_t failed;
  };
  struct ScheduleOutcome
  {
    int32_t code;
    std::string error;
  };

  /// The one path behind Resume and SplitBagfile: run `action` now for a zero time, arm a
  /// node-time timer, or queue a message-time schedule. `action` gets the reason to stamp on the
  /// event and returns whether it succeeded.
  ScheduleOutcome schedule_or_run(
    Scheduler::Kind kind, const builtin_interfaces::msg::Time & at, int32_t mode,
    const std::string & tracking_topic, const ScheduleCodes & codes,
    std::function<bool(const std::string &)> action);

  /// The node clock, held so const readers can ask whether it has started (Clock::started() is
  /// not const). Declared before bag_, which is built on it.
  rclcpp::Clock::SharedPtr clock_;
  /// The open bag, its channels and its counters. Owns the only writer lock.
  Bag bag_;

  /// A subscription, the switch that silences it, and its sequence bookkeeping.
  ///
  /// Dropping the handle is not enough to stop a callback already queued in the executor. rclcpp 33
  /// (Rolling) has GenericSubscription::disable_callbacks() for exactly that; on Jazzy and Kilted
  /// the callback checks `enabled` first, which gives the same guarantee on every distro.
  ///
  /// Sequence numbers are per PUBLISHER, not per topic. Keying by topic alone is wrong the moment
  /// a topic has more than one publisher -- /tf routinely does -- because the independent counters
  /// interleave and every alternation looks like an enormous gap. That mistake produced a reported
  /// 201,027,600 missing messages against 12,728 written. Keeping the map on the subscription (as
  /// a shared_ptr the callback co-owns) means the write path needs no shared lock, and a topic
  /// that is dropped and re-added starts clean instead of reading the publisher's continued
  /// counting as loss. The map is only touched from this subscription's own callbacks, which are
  /// mutually exclusive.
  struct Subscription
  {
    rclcpp::GenericSubscription::SharedPtr handle;
    std::shared_ptr<std::atomic<bool>> enabled;
    std::shared_ptr<LossAccounting::SequenceMap> last_publication_seq;
  };
  static void silence(Subscription & subscription);

  std::unordered_map<std::string, Subscription> subscriptions_;
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
  rclcpp::Service<SetProfile>::SharedPtr srv_set_profile_;
  rclcpp::Service<GetProfiles>::SharedPtr srv_get_profiles_;

  rclcpp::Publisher<SubscriptionChangeEvent>::SharedPtr pub_subscription_change_;
  rclcpp::Publisher<PauseEvent>::SharedPtr pub_pause_;
  rclcpp::Publisher<WriteSplitEvent>::SharedPtr pub_write_split_;
  rclcpp::Publisher<MessagesLostEvent>::SharedPtr pub_messages_lost_;
  rclcpp::Publisher<LowDiskEvent>::SharedPtr pub_low_disk_;
  rclcpp::Publisher<BagSizeLimitEvent>::SharedPtr pub_bag_size_limit_;
  rclcpp::Publisher<RecorderStatus>::SharedPtr pub_status_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  /// Checked in the write path. Atomic because it is read on every message and written from a
  /// service callback on a different thread.
  std::atomic_bool paused_{false};

  /// Missed, transport-lost and writer-lost counts, and the per-topic deltas the periodic
  /// MessagesLostEvent reports.
  LossAccounting losses_;
  rclcpp::TimerBase::SharedPtr messages_lost_timer_;

  /// Resume, split and record requests for a future time, node-time or message-time.
  std::unique_ptr<Scheduler> scheduler_;

  /// Reason stamped onto the next SubscriptionChangeEvent. Safe as shared state because every
  /// service handler runs in service_callback_group_, which is MutuallyExclusive: only one
  /// topic-changing operation is ever in flight.
  std::string current_reason_{"startup"};

  /// Why the most recent subscribe attempt failed, for the service to report. Shares
  /// current_reason_'s safety argument: only one service handler runs at a time.
  std::string last_failure_reason_;

  /// Everything read from the parameters. Read-only after construction, so no lock is needed.
  RecorderConfig config_;

  /// One timer paces both guards: they fire rarely, and staggering them buys nothing.
  rclcpp::TimerBase::SharedPtr storage_check_timer_;
  /// Polls for the first /clock under use_sim_time, then runs start() once and stops.
  rclcpp::TimerBase::SharedPtr clock_wait_timer_;
  /// Why the recorder stopped itself, for the status. Each guard fires once per recording because
  /// it stops it; ~/record clears both when the next bag opens.
  std::atomic_bool stopped_for_low_disk_{false};
  std::atomic_bool stopped_for_max_bag_size_{false};

  /// The disk floor and bag ceiling, and the bag-size figure the status reports.
  std::unique_ptr<StorageGuard> storage_guard_;
  /// config_.storage plus the uri suffix record() picks when the configured path is taken.
  rosbag2_storage::StorageOptions storage_options_;
  /// Topic selection at the moment of stop(), restored by record(). Under subscriptions_mutex_.
  std::vector<std::string> topics_at_stop_;
};

}  // namespace rosbag2_dynamic_recorder

#endif  // ROSBAG2_DYNAMIC_RECORDER__DYNAMIC_RECORDER_HPP_
