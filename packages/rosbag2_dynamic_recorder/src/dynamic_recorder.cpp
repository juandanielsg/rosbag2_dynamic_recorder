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

// Lifecycle and subscriptions. Services are in recorder_services.cpp, events and guards in
// recorder_events.cpp.

#include "rosbag2_dynamic_recorder/dynamic_recorder.hpp"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rclcpp/serialized_message.hpp"
#include "rmw/rmw.h"
#include "rosbag2_storage/qos.hpp"
#include "rosbag2_storage/topic_metadata.hpp"
#include "rosbag2_transport/recorder.hpp"  // type_description_hash_for_topic()

namespace rosbag2_dynamic_recorder
{

namespace
{
/// How often to look for the first /clock under use_sim_time.
constexpr auto kClockPollPeriod = std::chrono::milliseconds(50);
/// How often debug_timings publishes, and so how finely contention can be placed in time.
constexpr auto kDebugTimingsPeriod = std::chrono::milliseconds(200);
}  // namespace

DynamicRecorder::DynamicRecorder(const rclcpp::NodeOptions & options)
: rclcpp::Node("rosbag2_dynamic_recorder", options), clock_(get_clock()),
  bag_(get_logger(), clock_)
{
  config_ = rosbag2_dynamic_recorder::declare_parameters(*this);
  storage_options_ = config_.storage;
  paused_ = config_.start_paused;
  timings_ = std::make_unique<Timings>(config_.debug_timings);
  if (config_.debug_timings) {
    bag_.set_timings(timings_.get());
  }
  // Messages the bag drops itself are losses in the recorder, like the writer's own.
  bag_.set_loss_callback(
    [this](const std::string & topic, uint64_t count) {losses_.note_recorder_loss(topic, count);});
  bag_.set_rollover_callback(
    [this](const rosbag2_cpp::bag_events::BagSplitInfo & info, const std::string & reason) {
      on_rollover(info, reason);
    });
  storage_guard_ = std::make_unique<StorageGuard>(StorageGuard::Limits{
      config_.min_free_space, config_.min_free_space_percent, config_.max_bag_size});
  if (!config_.cache_enabled()) {
    RCLCPP_WARN(get_logger(),
      "max_cache_size and max_cache_duration are both 0: every message is written synchronously "
      "from its callback, which on slow storage blocks delivery and loses messages nothing reports");
  }

  service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  scheduler_ = std::make_unique<Scheduler>(*this, service_callback_group_);
  create_services();
  create_event_publishers();

  const auto every = [this](double seconds, std::function<void()> callback) {
      return create_wall_timer(
        std::chrono::duration<double>(seconds), std::move(callback), service_callback_group_);
    };
  if (config_.status_publish_period_s > 0.0) {
    status_timer_ = every(config_.status_publish_period_s, [this]() {publish_status();});
  }
  if (config_.messages_lost_report_period_s > 0.0) {
    messages_lost_timer_ =
      every(config_.messages_lost_report_period_s, [this]() {publish_messages_lost();});
  }
  if (config_.debug_timings) {
    // On the service group, so publishing runs between operations, never inside one.
    debug_timings_timer_ = create_wall_timer(
      kDebugTimingsPeriod, [this]() {publish_debug_timings();}, service_callback_group_);
    RCLCPP_INFO(get_logger(), "debug_timings: publishing on ~/debug/timings");
  }
  if (config_.low_disk_check_enabled() || config_.bag_size_check_enabled()) {
    // If both guards trip on one tick, the first stops the recording and the second finds it
    // stopped, so only one event is written.
    storage_check_timer_ = every(
      config_.storage_check_period_s, [this]() {check_free_space(); check_bag_size();});
  }
  if (config_.low_disk_check_enabled()) {
    RCLCPP_INFO(get_logger(),
      "Disk guard armed: stopping if free space falls below %llu bytes or %.1f%% of the "
      "filesystem, checked every %.1fs",
      static_cast<unsigned long long>(config_.min_free_space), config_.min_free_space_percent,
      config_.storage_check_period_s);
  }
  if (config_.bag_size_check_enabled()) {
    RCLCPP_INFO(get_logger(),
      "Bag size guard armed: stopping if the bag grows past %llu bytes, checked every %.1fs",
      static_cast<unsigned long long>(config_.max_bag_size), config_.storage_check_period_s);
  }

  if (!waiting_for_clock()) {
    start();
    return;
  }
  // The executor that would deliver /clock is not spinning yet, so wait on a timer. The recorder
  // answers meanwhile, and its status says what it is waiting for.
  RCLCPP_INFO(get_logger(), "use_sim_time is set: waiting for /clock before opening the bag");
  clock_wait_timer_ = create_wall_timer(
    kClockPollPeriod,
    [this]() {
      if (!waiting_for_clock()) {
        clock_wait_timer_->cancel();
        start();
      }
    },
    service_callback_group_);
}

DynamicRecorder::~DynamicRecorder()
{
  stop();
}

void DynamicRecorder::start()
{
  const auto error =
    bag_.open(storage_options_, config_.converter, writer_event_callbacks(), now());
  if (!error.empty()) {
    // A bad launch fails at startup, not after the first bag is open.
    throw std::runtime_error(error);
  }
  RCLCPP_INFO(get_logger(), "Recording to '%s' (storage_id=%s%s%s)%s%s%s",
    storage_options_.uri.c_str(),
    storage_options_.storage_id.c_str(),
    storage_options_.storage_preset_profile.empty() ? "" : ", preset=",
    storage_options_.storage_preset_profile.c_str(),
    config_.snapshot_mode ? " [snapshot mode]" : "",
    paused_.load() ? " [started paused]" : "",
    config_.use_sim_time ? " [sim time]" : "");

  // Before the first subscriptions, so the reason a paused bag's head is empty comes first.
  emit_initial_pause_state();
  const auto result = subscribe_batch({config_.initial_topics, {}}, "startup");
  for (const auto & topic : result.unavailable) {
    RCLCPP_WARN(get_logger(), "Initial topic '%s' unavailable at startup", topic.c_str());
  }
  publish_status();
}

void DynamicRecorder::stop()
{
  Timings::phase_here("preamble");
  record_deferred_events();
  if (!bag_.close()) {
    return;
  }
  {
    // Disable before dropping, so a callback already queued cannot fire on a subscription that
    // is going away.
    std::lock_guard<std::mutex> lock(subscriptions_mutex_);
    topics_at_stop_.clear();
    for (auto & [topic, subscription] : subscriptions_) {
      topics_at_stop_.push_back(topic);
      disable(subscription);
    }
    std::sort(topics_at_stop_.begin(), topics_at_stop_.end());
    subscriptions_.clear();
  }
  Timings::phase_here("destroy_subscriptions");
  scheduler_->clear();
  RCLCPP_INFO(get_logger(), "Recording stopped, bag closed.");
  publish_status();
}

std::string DynamicRecorder::record(const std::string & uri)
{
  if (bag_.is_open()) {
    return "already recording";
  }
  // rosbag2 refuses to open over an existing bag, which a stop and record on one path would hit.
  const std::string requested = uri.empty() ? storage_options_.uri : uri;
  const auto path = unused_bag_path(requested);
  if (!path) {
    const std::string error = "no free bag path near '" + requested + "'";
    RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    return error;
  }
  storage_options_.uri = *path;

  losses_.reset();
  const auto error =
    bag_.open(storage_options_, config_.converter, writer_event_callbacks(), now());
  if (!error.empty()) {
    // The recorder stays stopped but alive, so the caller can fix the path and try again.
    RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    return error;
  }
  // Cleared only once the new bag is open: a failed open is still stopped for the same reason.
  stopped_for_low_disk_.store(false);
  stopped_for_max_bag_size_.store(false);
  storage_guard_->reset();
  RCLCPP_INFO(get_logger(), "Recording to '%s'", storage_options_.uri.c_str());

  // Recording does not clear the paused flag, so a bag reopened while paused says so first.
  emit_initial_pause_state();
  std::vector<std::string> restore;
  {
    std::lock_guard<std::mutex> lock(subscriptions_mutex_);
    restore.swap(topics_at_stop_);
  }
  const auto result = subscribe_batch({restore, {}}, "service:record");
  for (const auto & topic : result.unavailable) {
    RCLCPP_WARN(get_logger(), "Could not restore '%s' on the new bag", topic.c_str());
  }
  publish_status();
  return "";
}

void DynamicRecorder::pause(const std::string & reason)
{
  if (!paused_.exchange(true)) {
    RCLCPP_INFO(get_logger(), "Recording paused (%s).", reason.c_str());
    // Before publish_status(), so the bag records the pause when it took effect.
    emit_pause_event(PauseEvent::PAUSED, reason);
    publish_status();
  }
}

void DynamicRecorder::resume(const std::string & reason)
{
  if (paused_.exchange(false)) {
    RCLCPP_INFO(get_logger(), "Recording resumed (%s).", reason.c_str());
    emit_pause_event(PauseEvent::RESUMED, reason);
    publish_status();
  }
}

std::vector<std::string> DynamicRecorder::subscribed_topics() const
{
  std::lock_guard<std::mutex> lock(subscriptions_mutex_);
  std::vector<std::string> topics;
  topics.reserve(subscriptions_.size());
  for (const auto & entry : subscriptions_) {
    topics.push_back(entry.first);
  }
  std::sort(topics.begin(), topics.end());
  return topics;
}

bool DynamicRecorder::waiting_for_clock() const
{
  return config_.use_sim_time && !clock_->started();
}

DynamicRecorder::BatchResult DynamicRecorder::subscribe_batch(
  const Selection & selection, const std::string & reason)
{
  BatchResult result;
  // Queried once, and only when a type has to be looked up.
  std::optional<TopicGraph> graph;
  std::vector<std::string> added;
  for (size_t i = 0; i < selection.topics.size(); ++i) {
    const auto & topic = selection.topics[i];
    std::string type = i < selection.types.size() ? selection.types[i] : "";
    if (type.empty()) {
      if (!graph) {
        graph = get_topic_names_and_types();
      }
      const auto resolved = single_type(*graph, topic);
      if (!resolved) {
        result.error = "cannot resolve a single type for '" + topic +
          "'; it is not on the graph, or publishes more than one type";
        RCLCPP_WARN(get_logger(), "%s", result.error.c_str());
        result.unavailable.push_back(topic);
        continue;
      }
      type = *resolved;
    }
    std::string error;
    switch (subscribe_topic(topic, type, reason, error)) {
      case SubscribeResult::Added:
        added.push_back(topic);
        result.subscribed.push_back(topic);
        break;
      case SubscribeResult::AlreadySubscribed:
        result.subscribed.push_back(topic);
        break;
      case SubscribeResult::Failed:
        result.error = error;
        result.unavailable.push_back(topic);
        break;
    }
  }
  log_topic_change("Subscribed", added);
  return result;
}

DynamicRecorder::BatchResult DynamicRecorder::replace_topics(
  const Selection & desired, const std::string & reason,
  std::vector<std::string> & unsubscribed_out)
{
  const std::unordered_set<std::string> keep(desired.topics.begin(), desired.topics.end());
  for (const auto & topic : subscribed_topics()) {
    if (keep.count(topic) == 0 && unsubscribe_topic(topic, reason)) {
      unsubscribed_out.push_back(topic);
    }
  }
  log_topic_change("Unsubscribed", unsubscribed_out);
  return subscribe_batch(desired, reason);
}

DynamicRecorder::SubscribeResult DynamicRecorder::subscribe_topic(
  const std::string & topic, const std::string & type, const std::string & reason,
  std::string & error)
{
  {
    std::lock_guard<std::mutex> lock(subscriptions_mutex_);
    if (subscriptions_.count(topic) > 0) {
      return SubscribeResult::AlreadySubscribed;
    }
  }
  const auto fail = [this, &error](std::string why) {
      error = std::move(why);
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      return SubscribeResult::Failed;
    };

  Timings::phase_here("preamble");
  // The graph query is the first thing to validate the name, and throws on a malformed one.
  std::vector<rclcpp::TopicEndpointInfo> endpoints;
  try {
    endpoints = get_publishers_info_by_topic(topic);
    Timings::phase_here("graph_query");
  } catch (const std::exception & e) {
    return fail("'" + topic + "' is not a usable topic name: " + e.what());
  }
  const auto qos = rosbag2_storage::Rosbag2QoS::adapt_request_to_offers(topic, endpoints);
  std::vector<rclcpp::QoS> offered_qos;
  offered_qos.reserve(endpoints.size());
  for (const auto & endpoint : endpoints) {
    offered_qos.push_back(endpoint.qos_profile());
  }
  Timings::phase_here("qos");
  const auto channel_error = bag_.ensure_channel({
      0u, topic, type, config_.serialization_format, offered_qos,
      rosbag2_transport::type_description_hash_for_topic(endpoints)});
  if (!channel_error.empty()) {
    return fail(channel_error);
  }

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.event_callbacks.message_lost_callback =
    [this, topic](const rclcpp::QOSMessageLostInfo & info) {
      losses_.note_transport_loss(topic, info.total_count_change);
    };
  auto enabled = std::make_shared<std::atomic<bool>>(true);
  auto last_sequence = std::make_shared<LossAccounting::SequenceMap>();
  auto callback =
    [this, topic, type, enabled, last_sequence](
    std::shared_ptr<const rclcpp::SerializedMessage> message, const rclcpp::MessageInfo & info) {
      if (enabled->load(std::memory_order_acquire)) {
        on_message(topic, type, *last_sequence, std::move(message), info);
      }
    };
  rclcpp::GenericSubscription::SharedPtr handle;
  try {
    // An explicit type skips the graph lookup, so an unloadable one first fails here.
    handle = create_generic_subscription(topic, type, qos, callback, subscription_options);
    Timings::phase_here("create_subscription");
  } catch (const std::exception & e) {
    return fail("could not subscribe to '" + topic + "' as '" + type + "': " + e.what());
  }

  {
    std::lock_guard<std::mutex> lock(subscriptions_mutex_);
    subscriptions_[topic] = Subscription{
      std::move(handle), std::move(enabled), std::move(last_sequence)};
  }
  RCLCPP_DEBUG(get_logger(), "Subscribed '%s' [%s]", topic.c_str(), type.c_str());
  Timings::phase_here("register");
  emit_subscription_change(topic, type, SubscriptionChangeEvent::SUBSCRIBED, reason);
  return SubscribeResult::Added;
}

bool DynamicRecorder::unsubscribe_topic(const std::string & topic, const std::string & reason)
{
  {
    std::lock_guard<std::mutex> lock(subscriptions_mutex_);
    auto it = subscriptions_.find(topic);
    if (it == subscriptions_.end()) {
      return false;
    }
    disable(it->second);
    subscriptions_.erase(it);  // Destroys the reader, and the sequence map with it.
  }
  RCLCPP_DEBUG(get_logger(), "Unsubscribed '%s'", topic.c_str());
  Timings::phase_here("destroy_subscription");
  // Outside subscriptions_mutex_: emitting takes the bag's lock, and the two never nest.
  emit_subscription_change(
    topic, bag_.channel_type(topic).value_or(""), SubscriptionChangeEvent::UNSUBSCRIBED, reason);
  return true;
}

void DynamicRecorder::disable(Subscription & subscription)
{
  subscription.enabled->store(false, std::memory_order_release);
#if ROSBAG2_DYNAMIC_RECORDER_HAS_DISABLE_CALLBACKS
  subscription.handle->disable_callbacks();
#endif
}

void DynamicRecorder::on_message(
  const std::string & topic, const std::string & type, LossAccounting::SequenceMap & last_sequence,
  std::shared_ptr<const rclcpp::SerializedMessage> message, const rclcpp::MessageInfo & info)
{
  const auto & rmw_info = info.get_rmw_message_info();
  // Under sim time the receive stamp is the node clock, so the bag's timeline is the
  // simulation's. The send stamp stays the middleware's wall time, as upstream leaves it.
  bool stamp_with_node_clock = config_.use_sim_time;
  rcutils_time_point_value_t send_ns = rmw_info.source_timestamp;
#ifdef _WIN32
  // As rosbag2_transport does: rmw_connextdds on Windows gives no usable timestamps.
  if (std::string(rmw_get_implementation_identifier()).find("rmw_connextdds") !=
    std::string::npos)
  {
    stamp_with_node_clock = true;
    send_ns = 0;
  }
#endif
  const rcutils_time_point_value_t recv_ns =
    stamp_with_node_clock ? now().nanoseconds() : rmw_info.received_timestamp;

  // Before the pause gate: a message discarded while paused was received, not missed.
  const auto sequence = rmw_info.publication_sequence_number;
  if (sequence != RMW_MESSAGE_INFO_SEQUENCE_NUMBER_UNSUPPORTED) {
    const std::string publisher(
      reinterpret_cast<const char *>(rmw_info.publisher_gid.data), RMW_GID_STORAGE_SIZE);
    losses_.note_sequence(last_sequence, publisher, sequence, topic);
  }

  // Also before the pause gate, so a scheduled resume records this very message.
  const auto due = scheduler_->take_due(topic, send_ns, recv_ns);
  if (due.resume) {
    Timings::Operation operation(*timings_, "schedule:resume");
    RCLCPP_INFO(get_logger(), "Scheduled resume reached on '%s'", topic.c_str());
    resume("schedule:resume");
  }
  if (due.split) {
    Timings::Operation operation(*timings_, "schedule:split");
    RCLCPP_INFO(get_logger(), "Scheduled split reached on '%s'", topic.c_str());
    bag_.split("schedule:split");
  }
  if (due.resume || due.split) {
    publish_status();  // The schedule has left the list.
  }

  if (!paused_.load()) {
    bag_.write(std::move(message), topic, type, recv_ns, send_ns);
  }
}

void DynamicRecorder::log_topic_change(
  const char * verb, const std::vector<std::string> & topics) const
{
  if (topics.empty()) {
    return;
  }
  std::string list;
  for (const auto & topic : topics) {
    list += (list.empty() ? "" : ", ") + topic;
  }
  RCLCPP_INFO(get_logger(), "%s %zu topic(s): %s", verb, topics.size(), list.c_str());
}

}  // namespace rosbag2_dynamic_recorder
