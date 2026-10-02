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
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_dynamic_recorder/bag.hpp"
#include "rosbag2_dynamic_recorder/interfaces.hpp"
#include "rosbag2_dynamic_recorder/loss_accounting.hpp"
#include "rosbag2_dynamic_recorder/recorder_config.hpp"
#include "rosbag2_dynamic_recorder/scheduler.hpp"
#include "rosbag2_dynamic_recorder/storage_guard.hpp"
#include "rosbag2_dynamic_recorder/timings.hpp"
#include "rosbag2_dynamic_recorder/topic_selection.hpp"
#include "rosbag2_storage/storage_options.hpp"

namespace rosbag2_dynamic_recorder
{

/// A recorder that owns its subscriptions, so the recorded topic set can change at runtime
/// without stopping the writer.
///
/// Unlike rosbag2_transport::Recorder, the topic set is not fixed at construction and there is no
/// discovery loop: topics are added and removed through services, so a change costs a gap on the
/// affected topic only, never a bag split.
///
/// The implementation is split by concern: dynamic_recorder.cpp (lifecycle and subscriptions),
/// recorder_services.cpp (the services and the status), recorder_events.cpp (events, the storage
/// guards and the periodic reports).
class DynamicRecorder : public rclcpp::Node
{
public:
  explicit DynamicRecorder(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DynamicRecorder() override;

  /// Close the bag and drop every subscription, remembering the selection for record(). A no-op
  /// when already stopped.
  void stop();

  /// Open a new bag after stop() and restore the selection recorded then. `uri` empty reuses the
  /// last path, suffixed if a bag is already there. Returns why it could not, or empty.
  std::string record(const std::string & uri = "");

  /// Discard arriving messages without dropping any subscription, so the bag shows a gap on every
  /// topic. `reason` is stamped on the PauseEvent that explains that gap.
  void pause(const std::string & reason);
  void resume(const std::string & reason);

  /// Topics currently subscribed, sorted.
  std::vector<std::string> subscribed_topics() const;

  /// The group every service and timer runs in, apart from the subscriptions, so a slow operation
  /// cannot stall recording. Exposed so an executable can give it a thread of its own.
  rclcpp::CallbackGroup::SharedPtr service_callback_group() const {return service_callback_group_;}

private:
  // --- Lifecycle and subscriptions (dynamic_recorder.cpp) ----------------------------------

  /// Open the first bag and subscribe the configured topics: the end of construction, deferred
  /// under use_sim_time until /clock has started.
  void start();

  /// What a batch subscription did, and why the last failure in it failed.
  struct BatchResult
  {
    std::vector<std::string> subscribed;
    std::vector<std::string> unavailable;
    std::string error;
  };

  /// Subscribe every topic in `selection`, resolving missing types from one graph query.
  BatchResult subscribe_batch(const Selection & selection, const std::string & reason);

  /// Make the subscriptions exactly `desired`: drop the rest first, then add what is missing.
  /// Topics in both are never touched, so they record without a gap.
  BatchResult replace_topics(
    const Selection & desired, const std::string & reason,
    std::vector<std::string> & unsubscribed_out);

  enum class SubscribeResult { Added, AlreadySubscribed, Failed };

  /// Create the channel and the subscription for one topic. Never throws: a bad name or an
  /// unloadable type is reported in `error` instead of escaping a service callback.
  SubscribeResult subscribe_topic(
    const std::string & topic, const std::string & type, const std::string & reason,
    std::string & error);

  /// Drop the subscription. The channel stays, so the topic becomes a sparse channel.
  /// Returns false if the topic was not subscribed.
  bool unsubscribe_topic(const std::string & topic, const std::string & reason);

  /// A subscription, the switch that silences it, and its sequence bookkeeping.
  ///
  /// Dropping the handle does not stop a callback already queued in the executor. rclcpp 33
  /// (Rolling) has GenericSubscription::disable_callbacks() for that; elsewhere the callback
  /// checks `enabled` first, which gives the same guarantee.
  ///
  /// Sequence numbers are per publisher: a topic with several publishers (/tf routinely) has
  /// independent counters. The map lives with the subscription, co-owned by its callback, so the
  /// write path needs no shared lock and a re-added topic starts clean.
  struct Subscription
  {
    rclcpp::GenericSubscription::SharedPtr handle;
    std::shared_ptr<std::atomic<bool>> enabled;
    std::shared_ptr<LossAccounting::SequenceMap> last_sequence;
  };
  static void disable(Subscription & subscription);

  /// Everything one recorded message goes through: sequence tracking, schedules, the pause gate
  /// and the write.
  void on_message(
    const std::string & topic, const std::string & type, LossAccounting::SequenceMap & last_sequence,
    std::shared_ptr<const rclcpp::SerializedMessage> message, const rclcpp::MessageInfo & info);

  /// One info line for a batch of topic changes, nothing for an empty one. The per-topic record is
  /// the SubscriptionChangeEvent.
  void log_topic_change(const char * verb, const std::vector<std::string> & topics) const;

  bool is_recording() const {return bag_.is_open();}
  /// True under use_sim_time until the first /clock message: a bag opened before it would be
  /// stamped from time 0.
  bool waiting_for_clock() const;

  // --- Services and status (recorder_services.cpp) ------------------------------------------

  void create_services();

  /// Offer `~/<name>` on the service callback group. Every call that can change something is a
  /// timed operation under debug_timings; reads (get_*, is_*) are not, or polling buries them.
  template<typename SrvT>
  void serve(
    const std::string & name,
    void (DynamicRecorder::* handler)(
      std::shared_ptr<typename SrvT::Request>, std::shared_ptr<typename SrvT::Response>));

  void handle_subscribe_topics(
    std::shared_ptr<SubscribeTopics::Request> request,
    std::shared_ptr<SubscribeTopics::Response> response);
  void handle_unsubscribe_topics(
    std::shared_ptr<UnsubscribeTopics::Request> request,
    std::shared_ptr<UnsubscribeTopics::Response> response);
  void handle_set_topics(
    std::shared_ptr<SetTopics::Request> request, std::shared_ptr<SetTopics::Response> response);
  void handle_set_profile(
    std::shared_ptr<SetProfile::Request> request, std::shared_ptr<SetProfile::Response> response);
  void handle_get_profiles(
    std::shared_ptr<GetProfiles::Request> request,
    std::shared_ptr<GetProfiles::Response> response);
  void handle_get_subscribed_topics(
    std::shared_ptr<GetSubscribedTopics::Request> request,
    std::shared_ptr<GetSubscribedTopics::Response> response);
  void handle_pause(
    std::shared_ptr<Pause::Request> request, std::shared_ptr<Pause::Response> response);
  void handle_resume(
    std::shared_ptr<Resume::Request> request, std::shared_ptr<Resume::Response> response);
  void handle_toggle_paused(
    std::shared_ptr<TogglePaused::Request> request,
    std::shared_ptr<TogglePaused::Response> response);
  void handle_is_paused(
    std::shared_ptr<IsPaused::Request> request, std::shared_ptr<IsPaused::Response> response);
  void handle_split_bagfile(
    std::shared_ptr<SplitBagfile::Request> request,
    std::shared_ptr<SplitBagfile::Response> response);
  void handle_snapshot(
    std::shared_ptr<Snapshot::Request> request, std::shared_ptr<Snapshot::Response> response);
  void handle_stop(
    std::shared_ptr<Stop::Request> request, std::shared_ptr<Stop::Response> response);
  void handle_record(
    std::shared_ptr<Record::Request> request, std::shared_ptr<Record::Response> response);
  void handle_get_status(
    std::shared_ptr<GetStatus::Request> request, std::shared_ptr<GetStatus::Response> response);

  /// The selection a subscribe or set_topics request asks for, with the graph as the pattern's
  /// candidates. Returns why it cannot be honoured (including that no bag is open), or empty.
  std::string select_from_graph(
    const std::vector<std::string> & topics, const std::vector<std::string> & types,
    const std::string & regex, const std::string & exclude_regex, Selection & out) const;

  /// What a scheduled service answers with. The codes are the service's own constants, passed
  /// in because Resume and SplitBagfile number theirs differently.
  struct ScheduleCodes
  {
    int32_t success;
    int32_t invalid_mode;
    int32_t invalid_topic;
    /// For an immediate action that fails; unused by an action that cannot.
    int32_t failed;
  };
  struct ScheduleOutcome
  {
    int32_t code;
    std::string error;
  };

  /// The one path behind Resume and SplitBagfile: run `action` now for a zero or past time, arm a
  /// node-time timer, or queue a message-time schedule. `action` gets the reason to stamp on its
  /// event and returns whether it succeeded.
  ScheduleOutcome schedule_or_run(
    Scheduler::Kind kind, const builtin_interfaces::msg::Time & at, int32_t mode,
    const std::string & tracking_topic, const ScheduleCodes & codes,
    std::function<bool(const std::string &)> action);

  /// Why a call that needs an open bag is refused.
  std::string not_recording_reason() const;

  /// The profile whose topics are exactly `topics`, or empty. Derived on every call rather than
  /// remembered, so a hand-made change cannot leave a stale name in the status.
  std::string active_profile_for(const std::vector<std::string> & topics) const;

  /// The one source of both ~/status and ~/get_status.
  RecorderStatus build_status() const;

  /// Publish the status. Also called after every change, so a UI sees it at once.
  void publish_status();

  // --- Events, storage guards and periodic reports (recorder_events.cpp) ---------------------

  void create_event_publishers();

  /// Stamp `event`, publish it on `publisher`, and if `record` is set write it into the bag too:
  /// a gap or an end explained only on a topic would be lost with that topic.
  template<typename EventT>
  void emit_event(
    const typename rclcpp::Publisher<EventT>::SharedPtr & publisher, bool record, EventT & event);

  /// Write an already stamped and published event into the bag.
  template<typename EventT>
  void record_event(
    const typename rclcpp::Publisher<EventT>::SharedPtr & publisher, const EventT & event);

  void emit_subscription_change(
    const std::string & topic, const std::string & type, uint8_t action,
    const std::string & reason);
  void emit_pause_event(uint8_t action, const std::string & reason);

  /// A PAUSED event for a bag that opens while paused, so a recording that starts with a hole
  /// says why.
  void emit_initial_pause_state();

  /// The write-split and messages-lost callbacks a bag is opened with.
  Bag::EventCallbacks writer_event_callbacks();

  /// A rollover to a new file: publish a FileSplitEvent now and queue its in-bag copy. The writer
  /// calls this with the bag's lock held, where writing into the bag would deadlock, so the copy
  /// is written from the service group straight afterwards.
  void on_rollover(
    const rosbag2_cpp::bag_events::BagSplitInfo & info, const std::string & reason);

  /// Write the queued FileSplitEvent copies into the bag. stop() calls it before closing, so a
  /// rollover just before a stop is still explained.
  void record_deferred_events();

  /// Stop if free space under the bag fell below the configured minimum. Guards the filesystem,
  /// which logs or another recorder can fill whatever the bag's own limits.
  void check_free_space();

  /// Stop if the bag directory grew past max_bag_size. max_bagfile_size only rolls to a new file,
  /// so without this a recording left running has no upper bound.
  void check_bag_size();

  /// Publish the per-topic losses since the last report as a MessagesLostEvent.
  void publish_messages_lost();

  /// Publish the operations finished since the last call, and the write-lock contention.
  void publish_debug_timings();

  // --- State ----------------------------------------------------------------------------------

  /// Everything read from the parameters. Read-only after construction.
  RecorderConfig config_;
  /// config_.storage plus the uri record() moved to.
  rosbag2_storage::StorageOptions storage_options_;

  /// The node clock, held so const readers can ask whether it has started (Clock::started() is
  /// not const). Declared before bag_, which is built on it.
  rclcpp::Clock::SharedPtr clock_;
  /// The open bag, its channels and counters. Owns the only writer lock.
  Bag bag_;
  /// Always present, so the hooks never test for it; inert unless debug_timings is set.
  std::unique_ptr<Timings> timings_;

  std::unordered_map<std::string, Subscription> subscriptions_;
  /// Topic selection at the moment of stop(), restored by record().
  std::vector<std::string> topics_at_stop_;
  /// Guards subscriptions_ and topics_at_stop_.
  mutable std::mutex subscriptions_mutex_;

  /// Read on every message, written from a service: atomic.
  std::atomic_bool paused_{false};
  /// Why the recorder stopped itself, for the status. ~/record clears both.
  std::atomic_bool stopped_for_low_disk_{false};
  std::atomic_bool stopped_for_max_bag_size_{false};

  LossAccounting losses_;
  std::unique_ptr<Scheduler> scheduler_;
  /// The disk floor and the bag ceiling, and the bag size the status reports.
  std::unique_ptr<StorageGuard> storage_guard_;

  /// Services and timers run here, apart from the subscription callbacks, so creating a channel
  /// during a disk stall cannot hold up message delivery.
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;

  rclcpp::Publisher<SubscriptionChangeEvent>::SharedPtr pub_subscription_change_;
  rclcpp::Publisher<PauseEvent>::SharedPtr pub_pause_;
  rclcpp::Publisher<WriteSplitEvent>::SharedPtr pub_write_split_;
  rclcpp::Publisher<FileSplitEvent>::SharedPtr pub_file_split_;
  rclcpp::Publisher<MessagesLostEvent>::SharedPtr pub_messages_lost_;
  rclcpp::Publisher<LowDiskEvent>::SharedPtr pub_low_disk_;
  rclcpp::Publisher<BagSizeLimitEvent>::SharedPtr pub_bag_size_limit_;
  rclcpp::Publisher<RecorderStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<DebugTimings>::SharedPtr pub_debug_timings_;

  /// FileSplitEvents published but not yet written into the bag (see on_rollover()), and the
  /// one-shot timer that writes them. Both under deferred_mutex_.
  std::mutex deferred_mutex_;
  std::vector<FileSplitEvent> deferred_split_events_;
  rclcpp::TimerBase::SharedPtr deferred_timer_;

  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr messages_lost_timer_;
  /// Paces both storage guards.
  rclcpp::TimerBase::SharedPtr storage_check_timer_;
  rclcpp::TimerBase::SharedPtr debug_timings_timer_;
  /// Polls for the first /clock under use_sim_time, then runs start() once.
  rclcpp::TimerBase::SharedPtr clock_wait_timer_;
};

}  // namespace rosbag2_dynamic_recorder

#endif  // ROSBAG2_DYNAMIC_RECORDER__DYNAMIC_RECORDER_HPP_
