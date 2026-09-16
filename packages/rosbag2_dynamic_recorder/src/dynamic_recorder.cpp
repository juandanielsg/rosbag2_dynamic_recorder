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

#include "rosbag2_dynamic_recorder/dynamic_recorder.hpp"

#include "rosbag2_dynamic_recorder/bag.hpp"
#include "rosbag2_dynamic_recorder/loss_accounting.hpp"
#include "rosbag2_dynamic_recorder/recorder_config.hpp"
#include "rosbag2_dynamic_recorder/scheduler.hpp"
#include "rosbag2_dynamic_recorder/storage_guard.hpp"

#include <algorithm>
#include <filesystem>
#include <memory>
#include <regex>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rclcpp/serialized_message.hpp"

#include "rmw/rmw.h"

#include "rosbag2_cpp/bag_events.hpp"

#include "rosbag2_storage/qos.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/topic_metadata.hpp"
#include "rosbag2_transport/recorder.hpp"  // type_description_hash_for_topic()

namespace rosbag2_dynamic_recorder
{

namespace
{
constexpr int kReturnSuccess = 0;
constexpr int kReturnError = 1;

/// Events keep this many for late joiners. The UI's own history depth is chosen against it.
constexpr size_t kEventHistoryDepth = 10;

/// How many "(N)" suffixes record() tries before giving up on a bag path that already exists.
constexpr size_t kMaxBagPathSuffix = 10000;

/// Nanoseconds in a second, for converting between builtin_interfaces/Time and the raw
/// nanosecond count the graph and status use.
constexpr rcutils_time_point_value_t kNanosecondsPerSecond = 1000000000LL;

/// Verdict for every service that subscribes a batch. Each requested topic lands in exactly one
/// of the subscribed/unavailable lists, so "nothing subscribed, something unavailable" means the
/// whole request failed and the caller has to see that; an empty request stays a success, and a
/// partial one still carries the reason.
template<typename ResponseT>
void set_subscribe_result(ResponseT & response, bool subscribed_any, const std::string & reason)
{
  const bool all_failed = !subscribed_any && !response.unavailable_topics.empty();
  response.return_code = all_failed ? kReturnError : kReturnSuccess;
  response.error_string = all_failed && reason.empty()
    ? "none of the requested topics could be subscribed"
    : (response.unavailable_topics.empty() ? "" : reason);
}

rcutils_time_point_value_t to_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<rcutils_time_point_value_t>(stamp.sec) * kNanosecondsPerSecond +
         stamp.nanosec;
}

}  // namespace

DynamicRecorder::DynamicRecorder(const rclcpp::NodeOptions & options)
: rclcpp::Node("rosbag2_dynamic_recorder", options), bag_(get_logger(), get_clock())
{
  config_ = rosbag2_dynamic_recorder::declare_parameters(*this);
  storage_guard_ = std::make_unique<StorageGuard>(StorageGuard::Limits{
      config_.min_free_space, config_.min_free_space_percent, config_.max_bag_size});
  storage_options_ = config_.storage;
  paused_ = config_.start_paused;

  const auto open_error =
    bag_.open(storage_options_, config_.converter, writer_event_callbacks(), now());
  if (!open_error.empty()) {
    throw std::runtime_error(open_error);
  }
  RCLCPP_INFO(get_logger(), "Recording to '%s' (storage_id=%s%s%s)%s%s",
    storage_options_.uri.c_str(),
    storage_options_.storage_id.c_str(),
    storage_options_.storage_preset_profile.empty() ? "" : ", preset=",
    storage_options_.storage_preset_profile.c_str(),
    config_.snapshot_mode ? " [snapshot mode]" : "",
    paused_.load() ? " [started paused]" : "");
  if (!config_.cache_enabled()) {
    // Synchronous writes hold the subscription callback for the duration of each disk write.
    // That is exactly the condition under which publishers overwrite their own history unseen,
    // so say so once rather than let it show up later as an unexplained messages_missed.
    RCLCPP_WARN(get_logger(),
      "max_cache_size and max_cache_duration are both 0: every message is written synchronously "
      "from its callback, which on slow storage blocks delivery and loses messages nothing reports");
  }

  // Services get their own callback group. Adding a topic resolves its message definition inside
  // create_topic(), which is milliseconds on a local disk and far more on slow or remote storage;
  // keeping that off the group that runs subscription callbacks means an in-flight set_topics
  // cannot stall recording of untouched topics.
  service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  scheduler_ = std::make_unique<Scheduler>(*this, service_callback_group_);
  srv_subscribe_topics_ =
    serve<SubscribeTopics>("subscribe_topics", &DynamicRecorder::handle_subscribe_topics);
  srv_unsubscribe_topics_ =
    serve<UnsubscribeTopics>("unsubscribe_topics", &DynamicRecorder::handle_unsubscribe_topics);
  srv_set_topics_ = serve<SetTopics>("set_topics", &DynamicRecorder::handle_set_topics);
  srv_get_subscribed_topics_ =
    serve<GetSubscribedTopics>(
    "get_subscribed_topics", &DynamicRecorder::handle_get_subscribed_topics);
  srv_pause_ = serve<Pause>("pause", &DynamicRecorder::handle_pause);
  srv_resume_ = serve<Resume>("resume", &DynamicRecorder::handle_resume);
  srv_toggle_paused_ = serve<TogglePaused>("toggle_paused", &DynamicRecorder::handle_toggle_paused);
  srv_is_paused_ = serve<IsPaused>("is_paused", &DynamicRecorder::handle_is_paused);
  srv_split_bagfile_ = serve<SplitBagfile>("split_bagfile", &DynamicRecorder::handle_split_bagfile);
  srv_snapshot_ = serve<Snapshot>("snapshot", &DynamicRecorder::handle_snapshot);
  srv_stop_ = serve<Stop>("stop", &DynamicRecorder::handle_stop);
  srv_record_ = serve<Record>("record", &DynamicRecorder::handle_record);
  srv_get_status_ = serve<GetStatus>("get_status", &DynamicRecorder::handle_get_status);
  srv_set_profile_ = serve<SetProfile>("set_profile", &DynamicRecorder::handle_set_profile);
  srv_get_profiles_ = serve<GetProfiles>("get_profiles", &DynamicRecorder::handle_get_profiles);

  // Events use a small transient-local depth so a late subscriber still sees recent changes.
  const auto event_qos = rclcpp::QoS(kEventHistoryDepth).transient_local();
  pub_subscription_change_ =
    create_publisher<SubscriptionChangeEvent>("~/events/subscription_change", event_qos);
  pub_pause_ = create_publisher<PauseEvent>("~/events/pause", event_qos);
  pub_write_split_ = create_publisher<WriteSplitEvent>("~/events/write_split", event_qos);
  // Volatile, unlike the others: each of these carries the losses *since the last one*, so a late
  // joiner replaying old reports would double-count what the status already totals.
  pub_messages_lost_ = create_publisher<MessagesLostEvent>(
    "~/events/messages_lost", rclcpp::QoS(kEventHistoryDepth));
  pub_low_disk_ = create_publisher<LowDiskEvent>("~/events/low_disk", event_qos);
  pub_bag_size_limit_ =
    create_publisher<BagSizeLimitEvent>("~/events/bag_size_limit", event_qos);

  // Latched depth 1: a panel opened mid-recording gets current state immediately instead of
  // waiting up to a full tick for the first publication.
  pub_status_ = create_publisher<RecorderStatus>(
    "~/status", rclcpp::QoS(1).transient_local());
  if (config_.status_publish_period_s > 0.0) {
    status_timer_ = create_wall_timer(
      std::chrono::duration<double>(config_.status_publish_period_s),
      [this]() {publish_status();},
      service_callback_group_);
  }

  if (config_.messages_lost_report_period_s > 0.0) {
    messages_lost_timer_ = create_wall_timer(
      std::chrono::duration<double>(config_.messages_lost_report_period_s),
      [this]() {
        const auto losses = losses_.drain();
        if (losses.empty()) {
          return;  // Topics with no losses are not reported, matching the upstream contract.
        }
        MessagesLostEvent event;
        event.node_name = get_fully_qualified_name();
        for (const auto & loss : losses) {
          decltype(event.messages_lost_statistics)::value_type stat;
          stat.topic_name = loss.topic;
          stat.messages_lost_in_transport = loss.in_transport;
          stat.messages_lost_in_recorder = loss.in_recorder;
          event.messages_lost_statistics.push_back(stat);
        }
        pub_messages_lost_->publish(event);
      },
      service_callback_group_);
  }

  if (config_.low_disk_check_enabled() || config_.bag_size_check_enabled()) {
    // Both guards on one tick. If both trip at once the first stops the recording and the second
    // finds it already stopped, so at most one event is written.
    storage_check_timer_ = create_wall_timer(
      std::chrono::duration<double>(config_.storage_check_period_s),
      [this]() {check_free_space(); check_bag_size();},
      service_callback_group_);
  }
  if (config_.low_disk_check_enabled()) {
    RCLCPP_INFO(get_logger(),
      "Disk guard armed: stopping if free space falls below %llu bytes or %.1f%% of the "
      "filesystem, checked every %.1fs",
      static_cast<unsigned long long>(config_.min_free_space),
      config_.min_free_space_percent,
      config_.storage_check_period_s);
  }
  if (config_.bag_size_check_enabled()) {
    RCLCPP_INFO(get_logger(),
      "Bag size guard armed: stopping if the bag grows past %llu bytes, checked every %.1fs",
      static_cast<unsigned long long>(config_.max_bag_size),
      config_.storage_check_period_s);
  }

  // Before the initial subscriptions, so a bag started with start_paused opens with the reason
  // its head is empty rather than with channels that appear to fail immediately.
  emit_initial_pause_state();

  if (!config_.initial_topics.empty()) {
    std::vector<std::string> subscribed;
    std::vector<std::string> unavailable;
    subscribe_batch(config_.initial_topics, {}, subscribed, unavailable);
    for (const auto & topic : unavailable) {
      RCLCPP_WARN(get_logger(), "Initial topic '%s' unavailable at startup", topic.c_str());
    }
  }
}

DynamicRecorder::~DynamicRecorder()
{
  stop();
}

void DynamicRecorder::silence(Subscription & subscription)
{
  subscription.enabled->store(false, std::memory_order_release);
#if ROSBAG2_DYNAMIC_RECORDER_HAS_DISABLE_CALLBACKS
  subscription.handle->disable_callbacks();
#endif
}

void DynamicRecorder::stop()
{
  if (!bag_.close()) {
    return;
  }
  {
    // Disable callbacks before dropping the handles, so callbacks already queued in the executor
    // cannot fire on a subscription that is going away. Same idiom as RecorderImpl::stop().
    std::lock_guard<std::mutex> lock(subscriptions_mutex_);
    topics_at_stop_.clear();
    topics_at_stop_.reserve(subscriptions_.size());
    for (auto & [topic_name, subscription] : subscriptions_) {
      topics_at_stop_.push_back(topic_name);
      silence(subscription);
    }
    std::sort(topics_at_stop_.begin(), topics_at_stop_.end());
    subscriptions_.clear();
  }
  scheduler_->clear();
  RCLCPP_INFO(get_logger(), "Recording stopped, bag closed.");
  publish_status();
}

std::vector<std::string> DynamicRecorder::subscribed_topics() const
{
  std::lock_guard<std::mutex> lock(subscriptions_mutex_);
  std::vector<std::string> topics;
  topics.reserve(subscriptions_.size());
  for (const auto & [topic_name, _] : subscriptions_) {
    topics.push_back(topic_name);
  }
  std::sort(topics.begin(), topics.end());
  return topics;
}

std::optional<std::string> DynamicRecorder::resolve_type(
  const std::string & topic_name,
  const std::map<std::string, std::vector<std::string>> & graph) const
{
  const auto it = graph.find(topic_name);
  if (it == graph.end() || it->second.empty()) {
    return std::nullopt;
  }
  if (it->second.size() > 1) {
    // Ambiguous: the caller must say which type it means.
    return std::nullopt;
  }
  return it->second.front();
}

bool DynamicRecorder::subscribe_topic(
  const std::string & topic_name, const std::string & topic_type)
{
  {
    std::lock_guard<std::mutex> lock(subscriptions_mutex_);
    if (subscriptions_.count(topic_name) > 0) {
      return true;  // Already subscribed; not an error.
    }
  }

  // The graph query validates the topic name and throws on a malformed one, so it needs the same
  // protection as the subscription call below -- this is the first thing an invalid name hits.
  std::vector<rclcpp::TopicEndpointInfo> endpoints;
  try {
    endpoints = get_publishers_info_by_topic(topic_name);
  } catch (const std::exception & e) {
    last_failure_reason_ = "'" + topic_name + "' is not a usable topic name: " + e.what();
    RCLCPP_ERROR(get_logger(), "%s", last_failure_reason_.c_str());
    return false;
  }
  const auto qos = rosbag2_storage::Rosbag2QoS::adapt_request_to_offers(topic_name, endpoints);

  // Diagnostic: messages have been observed going missing from bags without the transport or the
  // writer reporting any loss. A subscription QoS weaker than what the publisher offers would
  // explain drops that raise no event, so record what we actually asked for versus what was
  // offered. Logged against the TB4 sim, this refuted the QoS hypothesis: we subscribe RELIABLE
  // with history matching or deeper than what is offered. The loss is upstream of us -- publishers
  // at KEEP_LAST(10) overwrite their history while the callback thread is blocked, so the samples
  // are never delivered and no reader-side loss event can fire.
  {
    const auto describe = [](const rclcpp::QoS & q) {
        const auto & p = q.get_rmw_qos_profile();
        std::string reliability = p.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE ?
          "reliable" : (p.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT ?
          "best_effort" : "unknown");
        std::string durability = p.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL ?
          "transient_local" : "volatile";
        std::string history = p.history == RMW_QOS_POLICY_HISTORY_KEEP_ALL ? "keep_all" :
          ("keep_last(" + std::to_string(p.depth) + ")");
        return reliability + "/" + durability + "/" + history;
      };
    std::string offered;
    for (const auto & endpoint : endpoints) {
      offered += (offered.empty() ? "" : ", ") + describe(endpoint.qos_profile());
    }
    RCLCPP_INFO(get_logger(), "QoS '%s': offered [%s] -> subscribing %s",
      topic_name.c_str(), offered.c_str(), describe(qos).c_str());
  }

  std::vector<rclcpp::QoS> offered_qos_profiles;
  offered_qos_profiles.reserve(endpoints.size());
  for (const auto & endpoint : endpoints) {
    offered_qos_profiles.push_back(endpoint.qos_profile());
  }
  const auto channel_error = bag_.ensure_channel({
      0u, topic_name, topic_type, config_.serialization_format, offered_qos_profiles,
      rosbag2_transport::type_description_hash_for_topic(endpoints)});
  if (!channel_error.empty()) {
    last_failure_reason_ = channel_error;
    RCLCPP_ERROR(get_logger(), "%s", last_failure_reason_.c_str());
    return false;
  }

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.event_callbacks.message_lost_callback =
    [this, topic_name](const rclcpp::QOSMessageLostInfo & info) {
      losses_.note_transport_loss(topic_name, info.total_count_change);
    };
  auto enabled = std::make_shared<std::atomic<bool>>(true);
  // Co-owned by the callback and the subscription entry; the map dies when the subscription does,
  // so a dropped-and-readded topic starts from a clean sequence position.
  auto last_publication_seq = std::make_shared<LossAccounting::SequenceMap>();
  auto callback =
    [this, topic_name, topic_type, enabled, last_publication_seq](
    std::shared_ptr<const rclcpp::SerializedMessage> message, const rclcpp::MessageInfo & info)
    {
      // See Subscription: this subscription may already have been dropped.
      if (!enabled->load(std::memory_order_acquire)) {
        return;
      }
      rcutils_time_point_value_t recv_timestamp{0};
      rcutils_time_point_value_t send_timestamp{0};
      // Ported from rosbag2_transport::RecorderImpl::create_subscription(): rmw_connextdds on
      // Windows does not provide usable received/source timestamps.
#ifdef _WIN32
      if (std::string(rmw_get_implementation_identifier()).find("rmw_connextdds") !=
        std::string::npos)
      {
        recv_timestamp = now().nanoseconds();
        send_timestamp = 0;
      } else {
        recv_timestamp = info.get_rmw_message_info().received_timestamp;
        send_timestamp = info.get_rmw_message_info().source_timestamp;
      }
#else
      recv_timestamp = info.get_rmw_message_info().received_timestamp;
      send_timestamp = info.get_rmw_message_info().source_timestamp;
#endif

      // Sequence tracking happens before the pause gate on purpose: while paused we still
      // receive messages and simply decline to write them, so they are not missing.
      const auto & rmw_info = info.get_rmw_message_info();
      const auto sequence = rmw_info.publication_sequence_number;
      if (sequence != RMW_MESSAGE_INFO_SEQUENCE_NUMBER_UNSUPPORTED) {
        losses_.note_sequence(
          *last_publication_seq,
          std::string(reinterpret_cast<const char *>(rmw_info.publisher_gid.data),
            RMW_GID_STORAGE_SIZE),
          sequence);
      }

      // Before the pause gate, so a scheduled resume takes effect for this very message rather
      // than the next one, and before the writer lock, since firing takes that lock itself.
      const auto fired = scheduler_->fired(topic_name, send_timestamp, recv_timestamp);
      if (fired.resume) {
        RCLCPP_INFO(get_logger(), "Scheduled resume reached on '%s'", topic_name.c_str());
        resume("schedule:resume");
      }
      if (fired.split) {
        RCLCPP_INFO(get_logger(), "Scheduled split reached on '%s'", topic_name.c_str());
        split_bagfile();
      }

      // Checked before taking the lock: while paused this is the whole cost of a message.
      if (paused_.load()) {
        return;
      }
      bag_.write(message, topic_name, topic_type, recv_timestamp, send_timestamp);
    };

  rclcpp::GenericSubscription::SharedPtr subscription;
  try {
    subscription = create_generic_subscription(
      topic_name, topic_type, qos, callback, subscription_options);
  } catch (const std::exception & e) {
    // Reachable from any caller that supplies topic_types explicitly, since that path skips the
    // graph lookup: an invalid topic name or an unloadable type lands here. It used to terminate
    // the process from inside a service callback.
    last_failure_reason_ =
      "could not subscribe to '" + topic_name + "' as '" + topic_type + "': " + e.what();
    RCLCPP_ERROR(get_logger(), "%s", last_failure_reason_.c_str());
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(subscriptions_mutex_);
    subscriptions_[topic_name] = Subscription{
      std::move(subscription), std::move(enabled), std::move(last_publication_seq)};
  }
  RCLCPP_INFO(get_logger(), "Subscribed '%s' [%s]", topic_name.c_str(), topic_type.c_str());
  emit_subscription_change(
    topic_name, topic_type, SubscriptionChangeEvent::SUBSCRIBED, current_reason_);
  return true;
}

bool DynamicRecorder::unsubscribe_topic(const std::string & topic_name)
{
  {
    std::lock_guard<std::mutex> lock(subscriptions_mutex_);
    auto it = subscriptions_.find(topic_name);
    if (it == subscriptions_.end()) {
      return false;
    }
    silence(it->second);
    // Erasing the entry drops the per-subscription sequence map with it, so re-subscribing starts
    // clean rather than reading the publisher's continued counting as loss.
    subscriptions_.erase(it);
  }
  RCLCPP_INFO(get_logger(), "Unsubscribed '%s'", topic_name.c_str());

  // Emitted outside subscriptions_mutex_: emitting takes the bag's lock, and keeping the two
  // uncrossed here means there is no lock-ordering cycle with subscribe_topic().
  emit_subscription_change(
    topic_name, bag_.channel_type(topic_name).value_or(""),
    SubscriptionChangeEvent::UNSUBSCRIBED, current_reason_);
  return true;
}

std::vector<std::string> DynamicRecorder::patternable_graph_topics() const
{
  const std::string own_prefix = std::string(get_fully_qualified_name()) + "/";
  std::vector<std::string> out;
  for (const auto & [name, types] : get_topic_names_and_types()) {
    (void)types;
    // Never let a pattern pull in our own topics. The event channels are written into the bag
    // directly already, so subscribing to them would record every event twice and make the
    // provenance harder to read rather than easier -- the opposite of what they are for.
    if (name.rfind(own_prefix, 0) == 0) {
      continue;
    }
    // ROS 2 marks a topic hidden with a name token starting in an underscore, and stock
    // `ros2 bag record` leaves those out by default. A pattern should not be the thing that
    // sneaks them in.
    if (name.find("/_") != std::string::npos) {
      continue;
    }
    out.push_back(name);
  }
  std::sort(out.begin(), out.end());
  return out;
}

bool DynamicRecorder::resolve_selection(
  const std::vector<std::string> & requested_topics,
  const std::vector<std::string> & requested_types,
  const std::string & pattern, const std::string & exclude,
  const std::vector<std::string> & candidates,
  std::vector<std::string> & topics_out, std::vector<std::string> & types_out,
  std::string & error_out) const
{
  topics_out = requested_topics;
  types_out = requested_types;
  // Keep the two the same length so the index-based type lookup in subscribe_batch stays valid
  // once matches are appended with no type of their own.
  types_out.resize(topics_out.size());

  if (pattern.empty() && exclude.empty()) {
    return true;
  }

  try {
    if (!pattern.empty()) {
      const std::regex include_re(pattern);
      const std::unordered_set<std::string> already(topics_out.begin(), topics_out.end());
      for (const auto & name : candidates) {
        // regex_search, not regex_match: "camera" should find "/robot/camera/image", which is
        // how stock `ros2 bag record -e` behaves and therefore what people will expect.
        if (already.count(name) == 0 && std::regex_search(name, include_re)) {
          topics_out.push_back(name);
          types_out.push_back("");
        }
      }
    }

    if (!exclude.empty()) {
      const std::regex exclude_re(exclude);
      std::vector<std::string> kept_topics;
      std::vector<std::string> kept_types;
      for (size_t i = 0; i < topics_out.size(); ++i) {
        if (!std::regex_search(topics_out[i], exclude_re)) {
          kept_topics.push_back(topics_out[i]);
          kept_types.push_back(types_out[i]);
        }
      }
      topics_out = std::move(kept_topics);
      types_out = std::move(kept_types);
    }
  } catch (const std::regex_error & e) {
    // Refused with the reason rather than quietly matching nothing, which would look identical
    // to a pattern that simply found no topics.
    error_out = std::string("invalid regular expression: ") + e.what();
    return false;
  }
  return true;
}

void DynamicRecorder::subscribe_batch(
  const std::vector<std::string> & topics,
  const std::vector<std::string> & topic_types,
  std::vector<std::string> & subscribed_out,
  std::vector<std::string> & unavailable_out)
{
  last_failure_reason_.clear();
  // The graph is only needed for topics whose type was not supplied. A caller that names every
  // type skips the query entirely; when it is needed, one snapshot covers the whole batch rather
  // than one query per topic.
  std::map<std::string, std::vector<std::string>> graph;
  for (size_t i = 0; i < topics.size(); ++i) {
    if (i >= topic_types.size() || topic_types[i].empty()) {
      graph = get_topic_names_and_types();
      break;
    }
  }
  for (size_t i = 0; i < topics.size(); ++i) {
    const auto & topic_name = topics[i];

    std::string topic_type;
    if (i < topic_types.size() && !topic_types[i].empty()) {
      topic_type = topic_types[i];
    } else {
      const auto resolved = resolve_type(topic_name, graph);
      if (!resolved.has_value()) {
        last_failure_reason_ = "cannot resolve a single type for '" + topic_name +
          "'; it is not on the graph, or publishes more than one type";
        RCLCPP_WARN(get_logger(), "%s", last_failure_reason_.c_str());
        unavailable_out.push_back(topic_name);
        continue;
      }
      topic_type = resolved.value();
    }

    if (subscribe_topic(topic_name, topic_type)) {
      subscribed_out.push_back(topic_name);
    } else {
      unavailable_out.push_back(topic_name);
    }
  }
}

void DynamicRecorder::handle_subscribe_topics(
  const std::shared_ptr<SubscribeTopics::Request> request,
  std::shared_ptr<SubscribeTopics::Response> response)
{
  if (!is_recording()) {
    // Without this the topics come back listed as "unavailable", whose documented
    // meaning is that they are absent from the graph or ambiguous. The real reason is
    // that there is no open bag to record into.
    response->return_code = kReturnError;
    response->error_string =
      "recorder is stopped; call ~/record to open a new bag first";
    return;
  }
  current_reason_ = "service:subscribe_topics";
  if (!request->topic_types.empty() && request->topic_types.size() != request->topics.size()) {
    response->return_code = kReturnError;
    response->error_string = "topic_types must be empty or the same length as topics";
    return;
  }

  std::vector<std::string> topics;
  std::vector<std::string> types;
  std::string selection_error;
  // The graph is only the candidate pool for a pattern; an explicit-list call skips the query.
  const auto candidates =
    request->regex.empty() ? std::vector<std::string>{} : patternable_graph_topics();
  if (!resolve_selection(
      request->topics, request->topic_types, request->regex, request->exclude_regex,
      candidates, topics, types, selection_error))
  {
    response->return_code = kReturnError;
    response->error_string = selection_error;
    return;
  }

  // A pattern that matches nothing is refused rather than reported as a quiet success. Subscribe
  // exists to add topics, so adding none is a request that was not met -- and the overwhelmingly
  // likely cause is a typo in the expression, which a success would hide.
  if (topics.empty() && !request->regex.empty()) {
    response->return_code = kReturnError;
    response->error_string =
      "regex '" + request->regex + "' matched no topics on the graph";
    return;
  }

  subscribe_batch(topics, types, response->subscribed_topics, response->unavailable_topics);
  set_subscribe_result(*response, !response->subscribed_topics.empty(), last_failure_reason_);
  publish_status();
}

void DynamicRecorder::handle_unsubscribe_topics(
  const std::shared_ptr<UnsubscribeTopics::Request> request,
  std::shared_ptr<UnsubscribeTopics::Response> response)
{
  current_reason_ = "service:unsubscribe_topics";

  std::vector<std::string> topics;
  std::vector<std::string> types;
  std::string selection_error;
  // Candidates are what is being recorded, not the graph: dropping a topic only means anything
  // for one already subscribed, and matching the graph would silently do nothing for a topic
  // that has since left it.
  if (!resolve_selection(
      request->topics, {}, request->regex, request->exclude_regex,
      subscribed_topics(), topics, types, selection_error))
  {
    response->return_code = kReturnError;
    response->error_string = selection_error;
    return;
  }

  for (const auto & topic_name : topics) {
    if (unsubscribe_topic(topic_name)) {
      response->unsubscribed_topics.push_back(topic_name);
    } else {
      response->not_subscribed_topics.push_back(topic_name);
    }
  }

  if (response->unsubscribed_topics.empty() && !topics.empty()) {
    response->return_code = kReturnError;
    response->error_string = "none of the requested topics were subscribed";
  } else {
    response->return_code = kReturnSuccess;
  }
  publish_status();
}

void DynamicRecorder::handle_set_topics(
  const std::shared_ptr<SetTopics::Request> request,
  std::shared_ptr<SetTopics::Response> response)
{
  if (!is_recording()) {
    // Without this the topics come back listed as "unavailable", whose documented
    // meaning is that they are absent from the graph or ambiguous. The real reason is
    // that there is no open bag to record into.
    response->return_code = kReturnError;
    response->error_string =
      "recorder is stopped; call ~/record to open a new bag first";
    return;
  }
  current_reason_ = "service:set_topics";
  if (!request->topic_types.empty() && request->topic_types.size() != request->topics.size()) {
    response->return_code = kReturnError;
    response->error_string = "topic_types must be empty or the same length as topics";
    return;
  }

  std::vector<std::string> topics;
  std::vector<std::string> types;
  std::string selection_error;
  // As in subscribe: only a pattern needs the graph as its candidate pool.
  const auto candidates =
    request->regex.empty() ? std::vector<std::string>{} : patternable_graph_topics();
  if (!resolve_selection(
      request->topics, request->topic_types, request->regex, request->exclude_regex,
      candidates, topics, types, selection_error))
  {
    response->return_code = kReturnError;
    response->error_string = selection_error;
    return;
  }
  // Unlike subscribe, an empty result is NOT an error here: set_topics with nothing is the
  // documented way to stop recording every topic without closing the bag, and a pattern that
  // matches nothing is the same request arrived at differently.

  const std::unordered_set<std::string> desired(topics.begin(), topics.end());

  // Drop first, then add. Topics in both sets are never touched, which is the whole point:
  // switching profiles must not interrupt the topics common to both.
  for (const auto & topic_name : subscribed_topics()) {
    if (desired.count(topic_name) == 0 && unsubscribe_topic(topic_name)) {
      response->unsubscribed_topics.push_back(topic_name);
    }
  }

  std::vector<std::string> subscribed;
  subscribe_batch(topics, types, subscribed, response->unavailable_topics);
  set_subscribe_result(*response, !subscribed.empty(), last_failure_reason_);
  response->subscribed_topics = subscribed_topics();
  publish_status();
}

void DynamicRecorder::handle_get_subscribed_topics(
  const std::shared_ptr<GetSubscribedTopics::Request> /*request*/,
  std::shared_ptr<GetSubscribedTopics::Response> response)
{
  response->topics = subscribed_topics();
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
  // Losses reported here are the writer's own: the cache was full because the disk could not
  // keep up, or a storage write failed. They used to be added to the same counter as the
  // transport-side losses collected by the subscription message_lost_callback, which put a slow
  // SD card on the status line as "loss reported by transport" and sent the operator to debug
  // the network. Kept apart so the number points at its own remedy.
#if ROSBAG2_DYNAMIC_RECORDER_HAS_WRITER_MESSAGES_LOST
  callbacks.messages_lost_callback =
    [this](const std::vector<rosbag2_cpp::bag_events::MessagesLostInfo> & infos) {
      for (const auto & info : infos) {
        losses_.note_recorder_loss(info.topic_name, info.num_messages_lost);
      }
    };
#else
  // Jazzy and Kilted writers do not report their losses, so messages_lost_in_recorder stays 0
  // there: unknown, not zero, and the docs say so.
#endif
  return callbacks;
}

void DynamicRecorder::emit_subscription_change(
  const std::string & topic_name, const std::string & topic_type, uint8_t action,
  const std::string & reason)
{
  SubscriptionChangeEvent event;
  event.topic_name = topic_name;
  event.topic_type = topic_type;
  event.action = action;
  event.reason = reason;
  // In the bag too: a channel that stops mid-bag is otherwise indistinguishable from lost data.
  emit_event(pub_subscription_change_, config_.record_subscription_events, event);
}

void DynamicRecorder::emit_pause_event(uint8_t action, const std::string & reason)
{
  PauseEvent event;
  event.action = action;
  event.reason = reason;
  // A pause leaves a hole in every topic at once, which is exactly what a crash, a network fault
  // or the environmental rosbag2 stall (~1.2s across every topic, roughly every 31.2s) also look
  // like. Without the in-bag copy the bag cannot tell them apart.
  emit_event(pub_pause_, config_.record_pause_events, event);
}

void DynamicRecorder::emit_initial_pause_state()
{
  if (paused_.load()) {
    emit_pause_event(PauseEvent::PAUSED, "startup");
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
  // Emit, then stop: the event is written into the bag by emit, and stop() closes it. Reversed,
  // the explanation would be outside the recording it explains. Writing it costs a little of the
  // space the guard is preserving, which is exactly why the minimum is above zero; if the write
  // still fails, the bag counts it and the stop proceeds.
  emit_event(pub_low_disk_, config_.record_low_disk_events, event);
  stopped_for_low_disk_.store(true);
  stop();
}

void DynamicRecorder::check_bag_size()
{
  if (!is_recording()) {
    return;
  }
  const auto big = storage_guard_->bag_too_big(bag_.uri());
  if (!big) {
    return;
  }
  RCLCPP_ERROR(get_logger(),
    "The bag is %llu bytes on disk, past the %llu byte limit; stopping recording",
    static_cast<unsigned long long>(big->bag_size),
    static_cast<unsigned long long>(config_.max_bag_size));
  BagSizeLimitEvent event;
  event.action = BagSizeLimitEvent::STOPPED;
  event.bag_size_bytes = big->bag_size;
  event.max_bag_size = config_.max_bag_size;
  event.reason = "timer";
  // Same order as the disk guard: the event goes into the bag, then stop() closes it. It lands a
  // few hundred bytes past a limit that is already exceeded, but the limit caps the recording,
  // not the disk; the disk guard is what stands between the bag and a full filesystem.
  emit_event(pub_bag_size_limit_, config_.record_bag_size_limit_events, event);
  stopped_for_max_bag_size_.store(true);
  stop();
}

template<typename EventT>
void DynamicRecorder::emit_event(
  const typename rclcpp::Publisher<EventT>::SharedPtr & pub, bool record, EventT & event)
{
  event.stamp = now();
  event.node_name = get_fully_qualified_name();
  if (!pub) {
    return;
  }
  pub->publish(event);
  if (!record) {
    return;
  }
  // One serializer per event type, built on first use: the typesupport lookup behind it is not
  // free, and events are rare enough that a member per type was clutter for nothing.
  static const rclcpp::Serialization<EventT> serialization;
  rclcpp::SerializedMessage serialized;
  serialization.serialize_message(&event, &serialized);
  // Deliberately not gated on paused_: a topic change while paused still has to be explicable,
  // and an event explaining a pause obviously cannot be suppressed by that same pause.
  bag_.write_event(
    pub->get_topic_name(), rosidl_generator_traits::name<EventT>(),
    std::make_shared<const rclcpp::SerializedMessage>(std::move(serialized)),
    to_nanoseconds(event.stamp));
}

void DynamicRecorder::pause(const std::string & reason)
{
  if (!paused_.exchange(true)) {
    RCLCPP_INFO(get_logger(), "Recording paused (%s).", reason.c_str());
    // Before publish_status(), so the bag records the pause at the moment it took effect rather
    // than after a status publication that could itself block. Both take the writer lock in turn;
    // neither holds it across the other, which std::mutex would deadlock on.
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

bool DynamicRecorder::is_paused() const
{
  return paused_.load();
}

bool DynamicRecorder::split_bagfile()
{
  return bag_.split();
}

bool DynamicRecorder::take_snapshot()
{
  return bag_.take_snapshot();
}

uint64_t DynamicRecorder::messages_lost_in_transport() const
{
  return losses_.totals().lost_in_transport;
}

uint64_t DynamicRecorder::messages_lost_in_recorder() const
{
  return losses_.totals().lost_in_recorder;
}

uint64_t DynamicRecorder::total_messages_lost() const
{
  return messages_lost_in_transport() + messages_lost_in_recorder();
}

uint64_t DynamicRecorder::total_messages_missed() const
{
  return losses_.totals().missed;
}

void DynamicRecorder::handle_pause(
  const std::shared_ptr<Pause::Request> /*request*/, std::shared_ptr<Pause::Response> /*response*/)
{
  pause("service:pause");
}

void DynamicRecorder::handle_resume(
  const std::shared_ptr<Resume::Request> request, std::shared_ptr<Resume::Response> response)
{
  // Resume is only meaningful against an open bag. Without this, a resume (or a queued scheduled
  // resume) would arm a timer against a recording that does not exist. Pause has no return code
  // to report through, so the same guard cannot be offered there; TogglePaused is unaffected
  // because it calls resume()/pause() directly rather than through this handler.
  if (!is_recording()) {
    response->return_code = Resume::Response::RETURN_CODE_RESUME_FAILED;
    response->error_string = "recorder is stopped; call ~/record to open a new bag first";
    return;
  }
  const auto outcome = schedule_or_run(
    Scheduler::Kind::Resume, request->resume_time, request->resume_mode,
    request->tracking_topic_name,
    {Resume::Response::RETURN_CODE_SUCCESS, Resume::Response::RETURN_CODE_INVALID_RESUME_MODE,
      Resume::Response::RETURN_CODE_INVALID_TRACKING_TOPIC, Resume::Response::RETURN_CODE_SUCCESS},
    [this](const std::string & reason) {resume(reason); return true;});
  response->return_code = outcome.code;
  response->error_string = outcome.error;
}

void DynamicRecorder::handle_toggle_paused(
  const std::shared_ptr<TogglePaused::Request> /*request*/,
  std::shared_ptr<TogglePaused::Response> /*response*/)
{
  if (paused_.load()) {
    resume("service:toggle_paused");
  } else {
    pause("service:toggle_paused");
  }
}

void DynamicRecorder::handle_is_paused(
  const std::shared_ptr<IsPaused::Request> /*request*/,
  std::shared_ptr<IsPaused::Response> response)
{
  response->paused = is_paused();
}

void DynamicRecorder::handle_split_bagfile(
  const std::shared_ptr<SplitBagfile::Request> request,
  std::shared_ptr<SplitBagfile::Response> response)
{
  if (!is_recording()) {
    response->return_code = SplitBagfile::Response::RETURN_CODE_NOT_RECORDING;
    response->error_string = "not recording";
    return;
  }
  const auto outcome = schedule_or_run(
    Scheduler::Kind::Split, request->split_time, request->split_mode,
    request->tracking_topic_name,
    {SplitBagfile::Response::RETURN_CODE_SUCCESS,
      SplitBagfile::Response::RETURN_CODE_INVALID_SPLIT_MODE,
      SplitBagfile::Response::RETURN_CODE_INVALID_TRACKING_TOPIC,
      SplitBagfile::Response::RETURN_CODE_SPLIT_FAILED},
    [this](const std::string &) {return split_bagfile();});
  response->return_code = outcome.code;
  response->error_string = outcome.error;
}

DynamicRecorder::ScheduleOutcome DynamicRecorder::schedule_or_run(
  Scheduler::Kind kind, const builtin_interfaces::msg::Time & at, int32_t mode,
  const std::string & tracking_topic, const ScheduleCodes & codes,
  std::function<bool(const std::string &)> action)
{
  const std::string verb = kind == Scheduler::Kind::Resume ? "resume" : "split";
  const auto run_now = [&]() -> ScheduleOutcome {
      if (!action("service:" + verb)) {
        return {codes.failed, "the writer could not " + verb + " the bag"};
      }
      return {codes.success, ""};
    };
  const auto at_ns = to_nanoseconds(at);
  if (at_ns == 0) {
    return run_now();
  }
  const auto parsed = Scheduler::mode_from(mode);
  if (!parsed) {
    return {codes.invalid_mode,
      verb + "_mode must be 0 (node time), 1 (publish time) or 2 (receive time)"};
  }
  if (!tracking_topic.empty()) {
    // A schedule keyed to a topic nobody is recording would wait forever, so refuse it rather
    // than accept a request that cannot come true.
    const auto current = subscribed_topics();
    if (std::find(current.begin(), current.end(), tracking_topic) == current.end()) {
      return {codes.invalid_topic,
        "tracking_topic_name '" + tracking_topic + "' is not being recorded"};
    }
  }
  if (*parsed != Scheduler::Mode::NodeTime) {
    scheduler_->schedule_at_message_time(kind, at_ns, *parsed, tracking_topic);
    return {codes.success, ""};
  }
  const auto delta = at_ns - now().nanoseconds();
  if (delta <= 0) {
    return run_now();
  }
  // A timer rather than the message path: node-time schedules must fire on a silent robot.
  scheduler_->arm(
    kind, std::chrono::nanoseconds(delta),
    [action, verb]() {action("schedule:" + verb);});
  return {codes.success, ""};
}

void DynamicRecorder::handle_snapshot(
  const std::shared_ptr<Snapshot::Request> /*request*/,
  std::shared_ptr<Snapshot::Response> response)
{
  response->success = take_snapshot();
  // take_snapshot() already logged the reason; only add the hint a caller cannot infer from it,
  // and only when snapshot mode is off, since with it on that line would be the duplicate.
  if (!response->success && !config_.snapshot_mode) {
    RCLCPP_WARN(get_logger(),
      "Snapshot failed because snapshot_mode is disabled -- set the snapshot_mode parameter");
  }
}

void DynamicRecorder::handle_stop(
  const std::shared_ptr<Stop::Request> /*request*/, std::shared_ptr<Stop::Response> response)
{
  if (!is_recording()) {
    response->return_code = kReturnError;
    response->error_string = "recording is already stopped";
    return;
  }
  current_reason_ = "service:stop";
  stop();
  response->return_code = kReturnSuccess;
}

void DynamicRecorder::handle_get_status(
  const std::shared_ptr<GetStatus::Request> /*request*/,
  std::shared_ptr<GetStatus::Response> response)
{
  response->status = build_status();
}

DynamicRecorder::RecorderStatus DynamicRecorder::build_status() const
{
  RecorderStatus status;
  status.storage_id = config_.storage.storage_id;
  status.snapshot_mode = config_.snapshot_mode;
  status.paused = paused_.load();
  const auto bag = bag_.info();
  status.recording = bag.open;
  status.uri = bag.uri;
  status.recording_started = bag.opened_at;
  status.elapsed_seconds = (now() - bag.opened_at).seconds();
  // One lock/copy/sort of the subscription map, reused for both fields.
  status.subscribed_topics = subscribed_topics();
  status.active_profile = active_profile_for(status.subscribed_topics);
  status.messages_written = bag.messages_written;
  const auto losses = losses_.totals();
  status.messages_lost_in_transport = losses.lost_in_transport;
  status.messages_lost_in_recorder = losses.lost_in_recorder;
  status.messages_lost = status.messages_lost_in_transport + status.messages_lost_in_recorder;
  status.write_errors = bag.write_errors;
  status.messages_missed = losses.missed;
  status.sequence_numbers_available = losses.sequence_numbers_available;
  status.bag_splits = bag.splits;
  status.bag_size_bytes = storage_guard_->bag_size(status.uri);
  const auto space = StorageGuard::filesystem_space(status.uri);
  status.free_space_bytes = space.free_bytes;
  status.total_space_bytes = space.total_bytes;
  status.min_free_space = config_.min_free_space;
  status.min_free_space_percent = config_.min_free_space_percent;
  status.stopped_for_low_disk = stopped_for_low_disk_.load();
  status.max_bag_size = config_.max_bag_size;
  status.stopped_for_max_bag_size = stopped_for_max_bag_size_.load();
  return status;
}

void DynamicRecorder::publish_status()
{
  if (!pub_status_) {
    return;
  }
  try {
    pub_status_->publish(build_status());
  } catch (const std::exception & e) {
    // stop() publishes, and stop() also runs from the destructor -- which can be after context
    // shutdown, where publishing is invalid. Throwing from a destructor would terminate.
    RCLCPP_DEBUG(get_logger(), "Could not publish status: %s", e.what());
  }
}

bool DynamicRecorder::is_recording() const
{
  return bag_.is_open();
}

bool DynamicRecorder::record(const std::string & uri)
{
  if (bag_.is_open()) {
    return false;
  }
  if (!uri.empty()) {
    storage_options_.uri = uri;
  }
  // rosbag2 refuses to open over an existing bag directory, and stop()/record() on the same
  // configured path would hit exactly that. Pick the next free suffix, as upstream's recorder
  // does, rather than failing or overwriting someone's data.
  namespace fs = std::filesystem;
  std::error_code ec;
  if (fs::exists(storage_options_.uri, ec)) {
    const fs::path base(storage_options_.uri);
    bool found = false;
    for (size_t i = 1; i < kMaxBagPathSuffix && !found; ++i) {
      fs::path candidate = base;
      candidate += "(" + std::to_string(i) + ")";
      if (!fs::exists(candidate, ec)) {
        storage_options_.uri = candidate.generic_string();
        found = true;
      }
    }
    if (!found) {
      last_failure_reason_ = "no free bag path near '" + base.generic_string() +
        "'; " + std::to_string(kMaxBagPathSuffix) + " suffixed directories already exist";
      RCLCPP_ERROR(get_logger(), "%s", last_failure_reason_.c_str());
      return false;
    }
  }

  // Publisher sequence positions from the previous bag would read as loss the first time each
  // topic is seen again, but stop() already dropped every subscription and with it each
  // per-subscription sequence map, so the restored ones start clean.
  losses_.reset();
  const auto open_error =
    bag_.open(storage_options_, config_.converter, writer_event_callbacks(), now());
  if (!open_error.empty()) {
    // Reported rather than thrown from inside a service callback, and the recorder stays
    // stopped but alive so the caller can fix the path and try again.
    last_failure_reason_ = open_error;
    RCLCPP_ERROR(get_logger(), "%s", last_failure_reason_.c_str());
    return false;
  }
  // Only now is this a fresh episode: clear last time's self-inflicted stop. Clearing before
  // the open would erase the reason the recorder is stopped when the open itself fails.
  stopped_for_low_disk_.store(false);
  stopped_for_max_bag_size_.store(false);
  storage_guard_->reset();
  RCLCPP_INFO(get_logger(), "Recording to '%s'", storage_options_.uri.c_str());

  // ~/record does not clear the paused flag, so a bag reopened while paused would otherwise begin
  // with an unexplained hole exactly like the one this event exists to account for.
  emit_initial_pause_state();

  std::vector<std::string> restore;
  {
    std::lock_guard<std::mutex> lock(subscriptions_mutex_);
    restore.swap(topics_at_stop_);
  }
  if (!restore.empty()) {
    current_reason_ = "service:record";
    std::vector<std::string> subscribed;
    std::vector<std::string> unavailable;
    subscribe_batch(restore, {}, subscribed, unavailable);
    for (const auto & topic : unavailable) {
      RCLCPP_WARN(get_logger(), "Could not restore '%s' on the new bag", topic.c_str());
    }
  }
  publish_status();
  return true;
}

void DynamicRecorder::handle_record(
  const std::shared_ptr<Record::Request> request, std::shared_ptr<Record::Response> response)
{
  if (is_recording()) {
    response->return_code = kReturnError;
    response->error_string = "already recording";
    return;
  }

  // Record carries no mode field upstream, only a timestamp, so it is node-time by definition.
  const auto at_ns = to_nanoseconds(request->start_time);
  const auto delta = at_ns == 0 ? 0 : at_ns - now().nanoseconds();
  if (delta > 0) {
    scheduler_->arm(
      Scheduler::Kind::Record, std::chrono::nanoseconds(delta),
      [this, uri = request->uri]() {
        if (!record(uri)) {
          RCLCPP_ERROR(get_logger(), "Scheduled recording failed to start: %s",
            last_failure_reason_.c_str());
        }
      });
    response->return_code = kReturnSuccess;
    return;
  }

  if (!record(request->uri)) {
    response->return_code = kReturnError;
    response->error_string =
      last_failure_reason_.empty() ? "could not start recording" : last_failure_reason_;
    return;
  }
  response->return_code = kReturnSuccess;
}

std::string DynamicRecorder::active_profile() const
{
  return active_profile_for(subscribed_topics());
}

std::string DynamicRecorder::active_profile_for(const std::vector<std::string> & current) const
{
  for (const auto & [name, topics] : config_.profiles) {
    if (topics == current) {
      return name;
    }
  }
  return "";
}

bool DynamicRecorder::set_profile(
  const std::string & name, std::vector<std::string> & subscribed_out,
  std::vector<std::string> & unsubscribed_out, std::vector<std::string> & unavailable_out)
{
  const auto profile = std::find_if(
    config_.profiles.begin(), config_.profiles.end(),
    [&name](const auto & entry) {return entry.first == name;});
  if (profile == config_.profiles.end()) {
    return false;
  }

  // Deliberately the same path as set_topics: drop what the profile omits, then add what it
  // wants, leaving topics common to both untouched. Switching profiles must not interrupt the
  // topics the two modes share -- that is the entire reason profiles exist here.
  const std::unordered_set<std::string> desired(profile->second.begin(), profile->second.end());
  for (const auto & topic_name : subscribed_topics()) {
    if (desired.count(topic_name) == 0 && unsubscribe_topic(topic_name)) {
      unsubscribed_out.push_back(topic_name);
    }
  }
  subscribe_batch(profile->second, {}, subscribed_out, unavailable_out);
  return true;
}

void DynamicRecorder::handle_set_profile(
  const std::shared_ptr<SetProfile::Request> request,
  std::shared_ptr<SetProfile::Response> response)
{
  if (!is_recording()) {
    response->return_code = kReturnError;
    response->error_string = "recorder is stopped; call ~/record to open a new bag first";
    return;
  }
  current_reason_ = "service:set_profile:" + request->name;
  if (!set_profile(request->name, response->subscribed_topics,
    response->unsubscribed_topics, response->unavailable_topics))
  {
    response->return_code = kReturnError;
    std::string known;
    for (const auto & [name, _] : config_.profiles) {
      known += (known.empty() ? "" : ", ") + name;
    }
    response->error_string = "no profile named '" + request->name + "'" +
      (known.empty() ? "; none are configured" : "; configured: " + known);
    return;
  }
  set_subscribe_result(*response, !response->subscribed_topics.empty(), last_failure_reason_);
  response->subscribed_topics = subscribed_topics();
  publish_status();
}

void DynamicRecorder::handle_get_profiles(
  const std::shared_ptr<GetProfiles::Request> /*request*/,
  std::shared_ptr<GetProfiles::Response> response)
{
  response->profiles.reserve(config_.profiles.size());
  for (const auto & [name, topics] : config_.profiles) {
    Profile profile;
    profile.name = name;
    profile.topics = topics;
    response->profiles.push_back(profile);
  }
  response->active_profile = active_profile();
}

}  // namespace rosbag2_dynamic_recorder
