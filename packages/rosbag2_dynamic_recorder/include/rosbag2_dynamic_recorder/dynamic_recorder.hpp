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

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"

#include "rosbag2_interfaces/srv/get_subscribed_topics.hpp"
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
  /// Idempotent.
  void stop();

  /// Topics currently subscribed, sorted.
  std::vector<std::string> subscribed_topics() const;

private:
  using SubscribeTopics = rosbag2_dynamic_recorder_interfaces::srv::SubscribeTopics;
  using UnsubscribeTopics = rosbag2_dynamic_recorder_interfaces::srv::UnsubscribeTopics;
  using SetTopics = rosbag2_dynamic_recorder_interfaces::srv::SetTopics;
  using GetSubscribedTopics = rosbag2_interfaces::srv::GetSubscribedTopics;

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

  std::string serialization_format_;
};

}  // namespace rosbag2_dynamic_recorder

#endif  // ROSBAG2_DYNAMIC_RECORDER__DYNAMIC_RECORDER_HPP_
