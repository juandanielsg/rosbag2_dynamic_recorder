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
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rmw/rmw.h"

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
  serialization_format_ = declare_parameter<std::string>("serialization_format", "cdr");
  const auto initial_topics = declare_parameter<std::vector<std::string>>(
    "topics", std::vector<std::string>{});

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = uri;
  storage_options.storage_id = storage_id;

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = serialization_format_;
  converter_options.output_serialization_format = serialization_format_;

  writer_ = std::make_unique<rosbag2_cpp::Writer>();
  writer_->open(storage_options, converter_options);
  recording_ = true;
  RCLCPP_INFO(get_logger(), "Recording to '%s' (storage_id=%s)", uri.c_str(), storage_id.c_str());

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
    for (auto & [_, subscription] : subscriptions_) {
      subscription->disable_callbacks();
    }
    subscriptions_.clear();
  }
  std::lock_guard<std::mutex> lock(writer_mutex_);
  writer_->close();
  RCLCPP_INFO(get_logger(), "Recording stopped, bag closed.");
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

  const auto endpoints = get_publishers_info_by_topic(topic_name);
  const auto qos = rosbag2_storage::Rosbag2QoS::adapt_request_to_offers(topic_name, endpoints);

  {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    if (!recording_) {
      return false;
    }
    if (known_channels_.count(topic_name) == 0) {
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
      writer_->create_topic(topic_metadata);
      known_channels_.emplace(topic_name, topic_type);
    }
  }

  rclcpp::SubscriptionOptions subscription_options;
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

      std::lock_guard<std::mutex> lock(writer_mutex_);
      if (!recording_) {
        return;
      }
      writer_->write(message, topic_name, topic_type, recv_timestamp, send_timestamp);
    };

  auto subscription = create_generic_subscription(
    topic_name, topic_type, qos, callback, subscription_options);

  std::lock_guard<std::mutex> lock(subscriptions_mutex_);
  subscriptions_[topic_name] = std::move(subscription);
  RCLCPP_INFO(get_logger(), "Subscribed '%s' [%s]", topic_name.c_str(), topic_type.c_str());
  return true;
}

bool DynamicRecorder::unsubscribe_topic(const std::string & topic_name)
{
  std::lock_guard<std::mutex> lock(subscriptions_mutex_);
  auto it = subscriptions_.find(topic_name);
  if (it == subscriptions_.end()) {
    return false;
  }
  it->second->disable_callbacks();
  subscriptions_.erase(it);
  RCLCPP_INFO(get_logger(), "Unsubscribed '%s'", topic_name.c_str());
  return true;
}

void DynamicRecorder::subscribe_batch(
  const std::vector<std::string> & topics,
  const std::vector<std::string> & topic_types,
  std::vector<std::string> & subscribed_out,
  std::vector<std::string> & unavailable_out)
{
  for (size_t i = 0; i < topics.size(); ++i) {
    const auto & topic_name = topics[i];

    std::string topic_type;
    if (i < topic_types.size() && !topic_types[i].empty()) {
      topic_type = topic_types[i];
    } else {
      const auto resolved = resolve_type(topic_name);
      if (!resolved.has_value()) {
        RCLCPP_WARN(get_logger(),
          "Cannot resolve a single type for '%s'; not on the graph, or ambiguous",
          topic_name.c_str());
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
    response->error_string = "none of the requested topics could be subscribed";
  } else {
    response->return_code = kReturnSuccess;
  }
}

void DynamicRecorder::handle_unsubscribe_topics(
  const std::shared_ptr<UnsubscribeTopics::Request> request,
  std::shared_ptr<UnsubscribeTopics::Response> response)
{
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
}

void DynamicRecorder::handle_set_topics(
  const std::shared_ptr<SetTopics::Request> request,
  std::shared_ptr<SetTopics::Response> response)
{
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
}

void DynamicRecorder::handle_get_subscribed_topics(
  const std::shared_ptr<GetSubscribedTopics::Request> /*request*/,
  std::shared_ptr<GetSubscribedTopics::Response> response)
{
  response->topics = subscribed_topics();
}

}  // namespace rosbag2_dynamic_recorder
