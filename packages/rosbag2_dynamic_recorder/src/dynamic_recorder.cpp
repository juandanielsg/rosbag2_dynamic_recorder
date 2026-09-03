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

#include "rosbag2_dynamic_recorder/dynamic_recorder.hpp"

#include <algorithm>
#include <filesystem>
#include <memory>
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
}  // namespace

DynamicRecorder::DynamicRecorder(const rclcpp::NodeOptions & options)
: rclcpp::Node("rosbag2_dynamic_recorder", options)
{
  const auto uri = declare_parameter<std::string>("uri", "dynamic_bag");
  const auto storage_id = declare_parameter<std::string>("storage_id", "mcap");
  uri_ = uri;
  storage_id_ = storage_id;
  serialization_format_ = declare_parameter<std::string>("serialization_format", "cdr");
  record_subscription_events_ = declare_parameter<bool>("record_subscription_events", true);
  snapshot_mode_ = declare_parameter<bool>("snapshot_mode", false);
  const auto max_cache_size = declare_parameter<int64_t>("max_cache_size", 100 * 1024 * 1024);
  const auto messages_lost_report_period_s =
    declare_parameter<double>("messages_lost_report_period", 5.0);
  const auto status_publish_period_s = declare_parameter<double>("status_publish_period", 1.0);
  paused_ = declare_parameter<bool>("start_paused", false);
  const auto initial_topics = declare_parameter<std::vector<std::string>>(
    "topics", std::vector<std::string>{});

  // Profiles are declared as a list of names plus one string-array parameter each, rather than a
  // nested structure, because ROS 2 parameters have no nested arrays and this keeps a plain YAML
  // file readable:
  //   profile_names: ["idle", "navigation"]
  //   profiles:
  //     idle: ["/tf", "/odom"]
  //     navigation: ["/tf", "/odom", "/scan"]
  const auto profile_names = declare_parameter<std::vector<std::string>>(
    "profile_names", std::vector<std::string>{});
  for (const auto & name : profile_names) {
    if (name.empty()) {
      RCLCPP_WARN(get_logger(), "Ignoring an empty profile name");
      continue;
    }
    auto topics = declare_parameter<std::vector<std::string>>(
      "profiles." + name, std::vector<std::string>{});
    if (topics.empty()) {
      RCLCPP_WARN(get_logger(),
        "Profile '%s' lists no topics; applying it would record nothing", name.c_str());
    }
    std::sort(topics.begin(), topics.end());
    topics.erase(std::unique(topics.begin(), topics.end()), topics.end());
    profiles_.emplace_back(name, std::move(topics));
  }
  if (!profiles_.empty()) {
    RCLCPP_INFO(get_logger(), "Loaded %zu recording profile(s)", profiles_.size());
  }

  storage_options_.uri = uri;
  storage_options_.storage_id = storage_id;
  storage_options_.snapshot_mode = snapshot_mode_;
  // Snapshot mode is a circular buffer flushed on demand, so it is meaningless without a cache.
  storage_options_.max_cache_size = static_cast<uint64_t>(max_cache_size);
  if (snapshot_mode_ && storage_options_.max_cache_size == 0) {
    throw std::invalid_argument("snapshot_mode requires max_cache_size > 0");
  }

  converter_options_.input_serialization_format = serialization_format_;
  converter_options_.output_serialization_format = serialization_format_;

  writer_ = std::make_unique<rosbag2_cpp::Writer>();
  writer_->open(storage_options_, converter_options_);
  recording_ = true;
  recording_started_ = now();
  setup_writer_events();
  RCLCPP_INFO(get_logger(), "Recording to '%s' (storage_id=%s)%s%s", uri.c_str(),
    storage_id.c_str(), snapshot_mode_ ? " [snapshot mode]" : "",
    paused_.load() ? " [started paused]" : "");

  // Services get their own callback group. Adding a topic costs ~0.5s, almost all of it message
  // definition resolution inside create_topic(); keeping that off the group that runs subscription
  // callbacks means an in-flight set_topics cannot stall recording of untouched topics.
  service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  const auto qos = rclcpp::ServicesQoS();

  srv_subscribe_topics_ = create_service<SubscribeTopics>(
    "~/subscribe_topics",
    std::bind(&DynamicRecorder::handle_subscribe_topics, this,
      std::placeholders::_1, std::placeholders::_2),
    qos, service_callback_group_);

  srv_unsubscribe_topics_ = create_service<UnsubscribeTopics>(
    "~/unsubscribe_topics",
    std::bind(&DynamicRecorder::handle_unsubscribe_topics, this,
      std::placeholders::_1, std::placeholders::_2),
    qos, service_callback_group_);

  srv_set_topics_ = create_service<SetTopics>(
    "~/set_topics",
    std::bind(&DynamicRecorder::handle_set_topics, this,
      std::placeholders::_1, std::placeholders::_2),
    qos, service_callback_group_);

  srv_get_subscribed_topics_ = create_service<GetSubscribedTopics>(
    "~/get_subscribed_topics",
    std::bind(&DynamicRecorder::handle_get_subscribed_topics, this,
      std::placeholders::_1, std::placeholders::_2),
    qos, service_callback_group_);

  srv_pause_ = create_service<Pause>(
    "~/pause",
    std::bind(&DynamicRecorder::handle_pause, this,
      std::placeholders::_1, std::placeholders::_2),
    qos, service_callback_group_);

  srv_resume_ = create_service<Resume>(
    "~/resume",
    std::bind(&DynamicRecorder::handle_resume, this,
      std::placeholders::_1, std::placeholders::_2),
    qos, service_callback_group_);

  srv_toggle_paused_ = create_service<TogglePaused>(
    "~/toggle_paused",
    std::bind(&DynamicRecorder::handle_toggle_paused, this,
      std::placeholders::_1, std::placeholders::_2),
    qos, service_callback_group_);

  srv_is_paused_ = create_service<IsPaused>(
    "~/is_paused",
    std::bind(&DynamicRecorder::handle_is_paused, this,
      std::placeholders::_1, std::placeholders::_2),
    qos, service_callback_group_);

  srv_split_bagfile_ = create_service<SplitBagfile>(
    "~/split_bagfile",
    std::bind(&DynamicRecorder::handle_split_bagfile, this,
      std::placeholders::_1, std::placeholders::_2),
    qos, service_callback_group_);

  srv_snapshot_ = create_service<Snapshot>(
    "~/snapshot",
    std::bind(&DynamicRecorder::handle_snapshot, this,
      std::placeholders::_1, std::placeholders::_2),
    qos, service_callback_group_);

  srv_stop_ = create_service<Stop>(
    "~/stop",
    std::bind(&DynamicRecorder::handle_stop, this,
      std::placeholders::_1, std::placeholders::_2),
    qos, service_callback_group_);

  srv_record_ = create_service<Record>(
    "~/record",
    std::bind(&DynamicRecorder::handle_record, this,
      std::placeholders::_1, std::placeholders::_2),
    qos, service_callback_group_);

  srv_get_status_ = create_service<GetStatus>(
    "~/get_status",
    std::bind(&DynamicRecorder::handle_get_status, this,
      std::placeholders::_1, std::placeholders::_2),
    qos, service_callback_group_);

  srv_set_profile_ = create_service<SetProfile>(
    "~/set_profile",
    std::bind(&DynamicRecorder::handle_set_profile, this,
      std::placeholders::_1, std::placeholders::_2),
    qos, service_callback_group_);

  srv_get_profiles_ = create_service<GetProfiles>(
    "~/get_profiles",
    std::bind(&DynamicRecorder::handle_get_profiles, this,
      std::placeholders::_1, std::placeholders::_2),
    qos, service_callback_group_);

  // Events use a small transient-local depth so a late subscriber still sees recent changes.
  const auto event_qos = rclcpp::QoS(10).transient_local();
  pub_subscription_change_ =
    create_publisher<SubscriptionChangeEvent>("~/events/subscription_change", event_qos);
  pub_write_split_ = create_publisher<WriteSplitEvent>("~/events/write_split", event_qos);
  pub_messages_lost_ = create_publisher<MessagesLostEvent>("~/events/messages_lost", rclcpp::QoS(10));

  // Latched depth 1: a panel opened mid-recording gets current state immediately instead of
  // waiting up to a full tick for the first publication.
  pub_status_ = create_publisher<RecorderStatus>(
    "~/status", rclcpp::QoS(1).transient_local());
  if (status_publish_period_s > 0.0) {
    status_timer_ = create_wall_timer(
      std::chrono::duration<double>(status_publish_period_s),
      [this]() {publish_status();},
      service_callback_group_);
  }

  if (messages_lost_report_period_s > 0.0) {
    messages_lost_timer_ = create_wall_timer(
      std::chrono::duration<double>(messages_lost_report_period_s),
      [this]() {
        MessagesLostEvent event;
        event.node_name = get_fully_qualified_name();
        {
          std::lock_guard<std::mutex> lock(messages_lost_mutex_);
          if (messages_lost_since_last_event_.empty()) {
            return;  // Topics with no losses are not reported, matching the upstream contract.
          }
          for (const auto & [topic_name, count] : messages_lost_since_last_event_) {
            rosbag2_interfaces::msg::MessagesLostEventTopicStat stat;
            stat.topic_name = topic_name;
            stat.messages_lost_in_transport = count;
            stat.messages_lost_in_recorder = 0;
            event.messages_lost_statistics.push_back(stat);
          }
          messages_lost_since_last_event_.clear();
        }
        pub_messages_lost_->publish(event);
      },
      service_callback_group_);
  }

  if (!initial_topics.empty()) {
    std::vector<std::string> subscribed;
    std::vector<std::string> unavailable;
    subscribe_batch(initial_topics, {}, subscribed, unavailable);
    for (const auto & topic : unavailable) {
      RCLCPP_WARN(get_logger(), "Initial topic '%s' unavailable at startup", topic.c_str());
    }
  }
}

DynamicRecorder::~DynamicRecorder()
{
  stop();
}

void DynamicRecorder::stop()
{
  {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    if (!recording_) {
      return;
    }
    recording_ = false;
  }
  {
    // Disable callbacks before dropping the handles, so callbacks already queued in the executor
    // cannot fire on a subscription that is going away. Same idiom as RecorderImpl::stop().
    std::lock_guard<std::mutex> lock(subscriptions_mutex_);
    topics_at_stop_.clear();
    topics_at_stop_.reserve(subscriptions_.size());
    for (auto & [topic_name, subscription] : subscriptions_) {
      topics_at_stop_.push_back(topic_name);
      subscription->disable_callbacks();
    }
    std::sort(topics_at_stop_.begin(), topics_at_stop_.end());
    subscriptions_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    writer_->close();
  }
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

std::optional<std::string> DynamicRecorder::resolve_type(const std::string & topic_name) const
{
  const auto names_and_types = get_topic_names_and_types();
  const auto it = names_and_types.find(topic_name);
  if (it == names_and_types.end() || it->second.empty()) {
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
  // offered. See notes/recording-stall.md.
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

  {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    if (!recording_) {
      return false;
    }
    const auto known = known_channels_.find(topic_name);
    if (known != known_channels_.end() && known->second != topic_type) {
      // The bag already has a channel for this topic under a different type. Writing the new
      // type into it would silently corrupt the recording -- a publisher restarted with a
      // changed type is the realistic way to get here. Refuse instead.
      last_failure_reason_ = "'" + topic_name + "' is already in this bag as '" + known->second +
        "'; it now publishes '" + topic_type + "'. Start a new bag to record the new type.";
      RCLCPP_ERROR(get_logger(), "%s", last_failure_reason_.c_str());
      return false;
    }
    if (known == known_channels_.end()) {
      std::vector<rclcpp::QoS> offered_qos_profiles;
      offered_qos_profiles.reserve(endpoints.size());
      for (const auto & endpoint : endpoints) {
        offered_qos_profiles.push_back(endpoint.qos_profile());
      }
      const rosbag2_storage::TopicMetadata topic_metadata{
        0u,
        topic_name,
        topic_type,
        serialization_format_,
        offered_qos_profiles,
        rosbag2_transport::type_description_hash_for_topic(endpoints),
      };
      // Resolves the message definition internally; this is the expensive call (~300-470ms for a
      // type with nested members), which is why known_channels_ guards it.
      try {
        writer_->create_topic(topic_metadata);
      } catch (const std::exception & e) {
        last_failure_reason_ = "could not create a channel for '" + topic_name + "': " + e.what();
        RCLCPP_ERROR(get_logger(), "%s", last_failure_reason_.c_str());
        return false;
      }
      known_channels_.emplace(topic_name, topic_type);
    }
  }

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.event_callbacks.message_lost_callback =
    [this, topic_name](const rclcpp::QOSMessageLostInfo & info) {
      total_messages_lost_.fetch_add(info.total_count_change);
      std::lock_guard<std::mutex> lock(messages_lost_mutex_);
      messages_lost_since_last_event_[topic_name] += info.total_count_change;
    };
  auto callback =
    [this, topic_name, topic_type](
    std::shared_ptr<const rclcpp::SerializedMessage> message, const rclcpp::MessageInfo & info)
    {
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
        sequence_numbers_available_.store(true, std::memory_order_relaxed);
        // Per publisher, not per topic: two publishers on one topic have unrelated counters.
        const std::string publisher_key(
          reinterpret_cast<const char *>(rmw_info.publisher_gid.data), RMW_GID_STORAGE_SIZE);
        std::lock_guard<std::mutex> sequence_lock(sequence_mutex_);
        auto & per_publisher = last_publication_seq_[topic_name];
        const auto previous = per_publisher.find(publisher_key);
        if (previous != per_publisher.end() && sequence > previous->second + 1) {
          messages_missed_.fetch_add(
            sequence - previous->second - 1, std::memory_order_relaxed);
        }
        per_publisher[publisher_key] = sequence;
      }

      // Checked before taking the lock: while paused this is the whole cost of a message.
      if (paused_.load()) {
        return;
      }
      std::lock_guard<std::mutex> lock(writer_mutex_);
      if (!recording_) {
        return;
      }
      try {
        writer_->write(message, topic_name, topic_type, recv_timestamp, send_timestamp);
        messages_written_.fetch_add(1, std::memory_order_relaxed);
      } catch (const std::exception & e) {
        // A full disk used to take the whole node down and with it the rest of the recording.
        // Count it, say so at a rate that cannot itself become the problem, and keep going: the
        // remaining topics and any later recovery are worth more than a clean death.
        write_errors_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
          "Failed to write a message on '%s': %s (%lu write errors so far)",
          topic_name.c_str(), e.what(),
          static_cast<unsigned long>(write_errors_.load(std::memory_order_relaxed)));
      }
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
    subscriptions_[topic_name] = std::move(subscription);
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
    it->second->disable_callbacks();
    subscriptions_.erase(it);
  }
  {
    // Forget the sequence position: the publisher keeps counting while we are not listening, and
    // on re-subscribe that jump is deliberate, not loss.
    std::lock_guard<std::mutex> sequence_lock(sequence_mutex_);
    last_publication_seq_.erase(topic_name);
  }
  RCLCPP_INFO(get_logger(), "Unsubscribed '%s'", topic_name.c_str());

  // Emitted outside subscriptions_mutex_: emitting takes writer_mutex_, and keeping the two
  // uncrossed here means there is no lock-ordering cycle with subscribe_topic().
  std::string topic_type;
  {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    const auto known = known_channels_.find(topic_name);
    if (known != known_channels_.end()) {
      topic_type = known->second;
    }
  }
  emit_subscription_change(
    topic_name, topic_type, SubscriptionChangeEvent::UNSUBSCRIBED, current_reason_);
  return true;
}

void DynamicRecorder::subscribe_batch(
  const std::vector<std::string> & topics,
  const std::vector<std::string> & topic_types,
  std::vector<std::string> & subscribed_out,
  std::vector<std::string> & unavailable_out)
{
  last_failure_reason_.clear();
  for (size_t i = 0; i < topics.size(); ++i) {
    const auto & topic_name = topics[i];

    std::string topic_type;
    if (i < topic_types.size() && !topic_types[i].empty()) {
      topic_type = topic_types[i];
    } else {
      const auto resolved = resolve_type(topic_name);
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

  subscribe_batch(
    request->topics, request->topic_types,
    response->subscribed_topics, response->unavailable_topics);

  // Nothing subscribed is reported as a failure so the caller notices, but an empty request is
  // not an error.
  if (response->subscribed_topics.empty() && !request->topics.empty()) {
    response->return_code = kReturnError;
    response->error_string = last_failure_reason_.empty()
      ? "none of the requested topics could be subscribed"
      : last_failure_reason_;
  } else {
    response->return_code = kReturnSuccess;
    // Partial success still has to say what went wrong, or a caller sees an empty error and a
    // short subscribed list with no explanation.
    response->error_string =
      response->unavailable_topics.empty() ? "" : last_failure_reason_;
  }
  publish_status();
}

void DynamicRecorder::handle_unsubscribe_topics(
  const std::shared_ptr<UnsubscribeTopics::Request> request,
  std::shared_ptr<UnsubscribeTopics::Response> response)
{
  current_reason_ = "service:unsubscribe_topics";
  for (const auto & topic_name : request->topics) {
    if (unsubscribe_topic(topic_name)) {
      response->unsubscribed_topics.push_back(topic_name);
    } else {
      response->not_subscribed_topics.push_back(topic_name);
    }
  }

  if (response->unsubscribed_topics.empty() && !request->topics.empty()) {
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

  const std::unordered_set<std::string> desired(request->topics.begin(), request->topics.end());

  // Drop first, then add. Topics in both sets are never touched, which is the whole point:
  // switching profiles must not interrupt the topics common to both.
  for (const auto & topic_name : subscribed_topics()) {
    if (desired.count(topic_name) == 0 && unsubscribe_topic(topic_name)) {
      response->unsubscribed_topics.push_back(topic_name);
    }
  }

  std::vector<std::string> subscribed;
  subscribe_batch(
    request->topics, request->topic_types, subscribed, response->unavailable_topics);

  response->subscribed_topics = subscribed_topics();
  response->return_code = kReturnSuccess;
  publish_status();
}

void DynamicRecorder::handle_get_subscribed_topics(
  const std::shared_ptr<GetSubscribedTopics::Request> /*request*/,
  std::shared_ptr<GetSubscribedTopics::Response> response)
{
  response->topics = subscribed_topics();
}

void DynamicRecorder::setup_writer_events()
{
  rosbag2_cpp::bag_events::WriterEventCallbacks callbacks;
  callbacks.write_split_callback =
    [this](rosbag2_cpp::bag_events::BagSplitInfo & info) {
      WriteSplitEvent event;
      event.closed_file = info.closed_file;
      event.opened_file = info.opened_file;
      event.node_name = get_fully_qualified_name();
      bag_splits_.fetch_add(1, std::memory_order_relaxed);
      if (pub_write_split_) {
        pub_write_split_->publish(event);
      }
      RCLCPP_INFO(get_logger(), "Bag split: '%s' -> '%s'",
        info.closed_file.c_str(), info.opened_file.c_str());
    };
  // Losses reported here are storage-side or cache-overflow, distinct from the transport-side
  // losses collected by the subscription message_lost_callback.
  callbacks.messages_lost_callback =
    [this](const std::vector<rosbag2_cpp::bag_events::MessagesLostInfo> & infos) {
      std::lock_guard<std::mutex> lock(messages_lost_mutex_);
      for (const auto & info : infos) {
        messages_lost_since_last_event_[info.topic_name] += info.num_messages_lost;
        total_messages_lost_.fetch_add(info.num_messages_lost);
      }
    };
  writer_->add_event_callbacks(callbacks);
}

void DynamicRecorder::emit_subscription_change(
  const std::string & topic_name, const std::string & topic_type, uint8_t action,
  const std::string & reason)
{
  SubscriptionChangeEvent event;
  event.stamp = now();
  event.topic_name = topic_name;
  event.topic_type = topic_type;
  event.action = action;
  event.reason = reason;
  event.node_name = get_fully_qualified_name();

  if (pub_subscription_change_) {
    pub_subscription_change_->publish(event);
  }
  if (!record_subscription_events_ || !pub_subscription_change_) {
    return;
  }

  // Write the event into the bag too. This is the point of the whole mechanism: a channel that
  // stops mid-bag is otherwise indistinguishable from lost data.
  const std::string event_topic = pub_subscription_change_->get_topic_name();
  const std::string event_type = "rosbag2_dynamic_recorder_interfaces/msg/SubscriptionChangeEvent";

  rclcpp::SerializedMessage serialized;
  subscription_change_serialization_.serialize_message(&event, &serialized);
  auto serialized_ptr = std::make_shared<const rclcpp::SerializedMessage>(std::move(serialized));

  std::lock_guard<std::mutex> lock(writer_mutex_);
  if (!recording_) {
    return;
  }
  try {
    if (known_channels_.count(event_topic) == 0) {
      const rosbag2_storage::TopicMetadata metadata{
        0u, event_topic, event_type, serialization_format_, {}, ""};
      writer_->create_topic(metadata);
      known_channels_.emplace(event_topic, event_type);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Could not create the event channel: %s", e.what());
    return;
  }
  const auto stamp =
    static_cast<rcutils_time_point_value_t>(event.stamp.sec) * 1000000000LL + event.stamp.nanosec;
  // Deliberately not gated on paused_: a topic change while paused still has to be explicable.
  try {
    writer_->write(serialized_ptr, event_topic, event_type, stamp, stamp);
    messages_written_.fetch_add(1, std::memory_order_relaxed);
  } catch (const std::exception & e) {
    write_errors_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_ERROR(get_logger(), "Could not record a subscription change: %s", e.what());
  }
}

void DynamicRecorder::pause()
{
  if (!paused_.exchange(true)) {
    RCLCPP_INFO(get_logger(), "Recording paused.");
    publish_status();
  }
}

void DynamicRecorder::resume()
{
  if (paused_.exchange(false)) {
    RCLCPP_INFO(get_logger(), "Recording resumed.");
    publish_status();
  }
}

bool DynamicRecorder::is_paused() const
{
  return paused_.load();
}

bool DynamicRecorder::split_bagfile()
{
  std::lock_guard<std::mutex> lock(writer_mutex_);
  if (!recording_) {
    return false;
  }
  try {
    writer_->split_bagfile();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to split the bag: %s", e.what());
    return false;
  }
  return true;
}

bool DynamicRecorder::take_snapshot()
{
  std::lock_guard<std::mutex> lock(writer_mutex_);
  if (!recording_) {
    return false;
  }
  try {
    return writer_->take_snapshot();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Snapshot failed: %s", e.what());
    return false;
  }
}

uint64_t DynamicRecorder::total_messages_lost() const
{
  return total_messages_lost_.load();
}

uint64_t DynamicRecorder::total_messages_missed() const
{
  return messages_missed_.load(std::memory_order_relaxed);
}

void DynamicRecorder::handle_pause(
  const std::shared_ptr<Pause::Request> /*request*/, std::shared_ptr<Pause::Response> /*response*/)
{
  pause();
}

void DynamicRecorder::handle_resume(
  const std::shared_ptr<Resume::Request> request, std::shared_ptr<Resume::Response> response)
{
  const bool scheduled = request->resume_time.sec != 0 || request->resume_time.nanosec != 0;
  if (scheduled) {
    // Timestamp-scheduled resume is not implemented yet; say so rather than resuming immediately
    // and silently doing something other than what was asked.
    response->return_code = Resume::Response::RETURN_CODE_RESUME_FAILED;
    response->error_string =
      "scheduled resume_time is not supported yet; send an empty resume_time to resume now";
    return;
  }
  resume();
  response->return_code = Resume::Response::RETURN_CODE_SUCCESS;
}

void DynamicRecorder::handle_toggle_paused(
  const std::shared_ptr<TogglePaused::Request> /*request*/,
  std::shared_ptr<TogglePaused::Response> /*response*/)
{
  if (paused_.load()) {
    resume();
  } else {
    pause();
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
  const bool scheduled = request->split_time.sec != 0 || request->split_time.nanosec != 0;
  if (scheduled) {
    response->return_code = SplitBagfile::Response::RETURN_CODE_INVALID_SPLIT_MODE;
    response->error_string =
      "scheduled split_time is not supported yet; send an empty split_time to split now";
    return;
  }
  if (!split_bagfile()) {
    response->return_code = SplitBagfile::Response::RETURN_CODE_NOT_RECORDING;
    response->error_string = "not recording";
    return;
  }
  response->return_code = SplitBagfile::Response::RETURN_CODE_SUCCESS;
}

void DynamicRecorder::handle_snapshot(
  const std::shared_ptr<Snapshot::Request> /*request*/,
  std::shared_ptr<Snapshot::Response> response)
{
  response->success = take_snapshot();
  if (!response->success) {
    RCLCPP_WARN(get_logger(), "Snapshot failed. snapshot_mode is %s.",
      snapshot_mode_ ? "enabled" : "disabled -- set the snapshot_mode parameter");
  }
}

void DynamicRecorder::handle_stop(
  const std::shared_ptr<Stop::Request> /*request*/, std::shared_ptr<Stop::Response> response)
{
  bool was_recording;
  {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    was_recording = recording_;
  }
  if (!was_recording) {
    response->return_code = kReturnError;
    response->error_string = "recording is already stopped";
    return;
  }
  current_reason_ = "service:stop";
  stop();
  response->return_code = kReturnSuccess;
}

uint64_t DynamicRecorder::bag_size_bytes() const
{
  namespace fs = std::filesystem;
  std::error_code ec;
  const fs::path dir(uri_);
  if (!fs::is_directory(dir, ec)) {
    return 0;
  }
  uint64_t total = 0;
  for (fs::recursive_directory_iterator it(dir, ec), end; it != end; it.increment(ec)) {
    if (ec) {
      break;  // Report what we counted rather than failing the whole status call.
    }
    if (it->is_regular_file(ec)) {
      const auto size = it->file_size(ec);
      if (!ec) {
        total += size;
      }
    }
  }
  return total;
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
  status.uri = uri_;
  status.storage_id = storage_id_;
  status.snapshot_mode = snapshot_mode_;
  status.paused = paused_.load();
  {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    status.recording = recording_;
  }
  status.recording_started = recording_started_;
  status.elapsed_seconds = (now() - recording_started_).seconds();
  status.subscribed_topics = subscribed_topics();
  status.active_profile = active_profile();
  status.messages_written = messages_written_.load(std::memory_order_relaxed);
  status.messages_lost = total_messages_lost_.load();
  status.write_errors = write_errors_.load(std::memory_order_relaxed);
  status.messages_missed = messages_missed_.load(std::memory_order_relaxed);
  status.sequence_numbers_available =
    sequence_numbers_available_.load(std::memory_order_relaxed);
  status.bag_splits = bag_splits_.load(std::memory_order_relaxed);
  status.bag_size_bytes = bag_size_bytes();
  return status;
}

void DynamicRecorder::publish_status()
{
  if (pub_status_) {
    pub_status_->publish(build_status());
  }
}

bool DynamicRecorder::is_recording() const
{
  std::lock_guard<std::mutex> lock(writer_mutex_);
  return recording_;
}

bool DynamicRecorder::record(const std::string & uri)
{
  std::vector<std::string> restore;
  {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    if (recording_) {
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
      for (size_t i = 1; i < 10000; ++i) {
        fs::path candidate = base;
        candidate += "(" + std::to_string(i) + ")";
        if (!fs::exists(candidate, ec)) {
          storage_options_.uri = candidate.generic_string();
          break;
        }
      }
    }
    uri_ = storage_options_.uri;

    // A new bag has no channels, and publisher sequence positions from the previous bag would
    // read as loss the first time each topic is seen again.
    known_channels_.clear();
    {
      std::lock_guard<std::mutex> sequence_lock(sequence_mutex_);
      last_publication_seq_.clear();
    }
    messages_written_.store(0, std::memory_order_relaxed);
    messages_missed_.store(0, std::memory_order_relaxed);
    total_messages_lost_.store(0);
    bag_splits_.store(0, std::memory_order_relaxed);

    auto writer = std::make_unique<rosbag2_cpp::Writer>();
    try {
      writer->open(storage_options_, converter_options_);
    } catch (const std::exception & e) {
      // Bad path, no permission, or the suffix search below exhausted its range. Report it
      // rather than terminating from inside a service callback, and leave the recorder stopped
      // but alive so the caller can fix the path and try again.
      last_failure_reason_ = std::string("could not open '") + storage_options_.uri + "': " +
        e.what();
      RCLCPP_ERROR(get_logger(), "%s", last_failure_reason_.c_str());
      return false;
    }
    writer_ = std::move(writer);
    write_errors_.store(0, std::memory_order_relaxed);
    recording_ = true;
    recording_started_ = now();
    restore = topics_at_stop_;
    topics_at_stop_.clear();
  }

  setup_writer_events();
  RCLCPP_INFO(get_logger(), "Recording to '%s'", uri_.c_str());

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
  const bool scheduled = request->start_time.sec != 0 || request->start_time.nanosec != 0;
  if (scheduled) {
    // Same policy as resume and split_bagfile: refuse clearly rather than accept the field and
    // quietly do something else.
    response->return_code = kReturnError;
    response->error_string =
      "scheduled start_time is not supported yet; send an empty start_time to start now";
    return;
  }
  const bool was_recording = is_recording();
  if (!record(request->uri)) {
    response->return_code = kReturnError;
    response->error_string = was_recording
      ? "already recording"
      : (last_failure_reason_.empty() ? "could not start recording" : last_failure_reason_);
    return;
  }
  response->return_code = kReturnSuccess;
}

std::string DynamicRecorder::active_profile() const
{
  const auto current = subscribed_topics();  // already sorted
  for (const auto & [name, topics] : profiles_) {
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
    profiles_.begin(), profiles_.end(),
    [&name](const auto & entry) {return entry.first == name;});
  if (profile == profiles_.end()) {
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
  std::vector<std::string> subscribed;
  subscribe_batch(profile->second, {}, subscribed, unavailable_out);
  subscribed_out = subscribed_topics();
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
    for (const auto & [name, _] : profiles_) {
      known += (known.empty() ? "" : ", ") + name;
    }
    response->error_string = "no profile named '" + request->name + "'" +
      (known.empty() ? "; none are configured" : "; configured: " + known);
    return;
  }
  response->return_code = kReturnSuccess;
  publish_status();
}

void DynamicRecorder::handle_get_profiles(
  const std::shared_ptr<GetProfiles::Request> /*request*/,
  std::shared_ptr<GetProfiles::Response> response)
{
  response->profiles.reserve(profiles_.size());
  for (const auto & [name, topics] : profiles_) {
    Profile profile;
    profile.name = name;
    profile.topics = topics;
    response->profiles.push_back(profile);
  }
  response->active_profile = active_profile();
}

}  // namespace rosbag2_dynamic_recorder
