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

#ifndef ROSBAG2_DYNAMIC_RECORDER__TOPIC_SELECTION_HPP_
#define ROSBAG2_DYNAMIC_RECORDER__TOPIC_SELECTION_HPP_

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace rosbag2_dynamic_recorder
{

/// Topic name -> the types published on it, as rclcpp::Node::get_topic_names_and_types() returns.
using TopicGraph = std::map<std::string, std::vector<std::string>>;

/// Topics to record and their types, index-aligned. An empty type is resolved from the graph.
struct Selection
{
  std::vector<std::string> topics;
  std::vector<std::string> types;
};

/// Combine explicitly named topics with every candidate `include_regex` finds, then drop every
/// topic `exclude_regex` finds. The exclusion applies to the combined set, so it filters named
/// topics too. Both expressions search rather than fully match, as `ros2 bag record -e` does.
///
/// `types` must be empty or as long as `topics`. Returns why the request is invalid (that, or an
/// expression that does not compile), or empty on success.
std::string select_topics(
  const std::vector<std::string> & topics, const std::vector<std::string> & types,
  const std::string & include_regex, const std::string & exclude_regex,
  const std::vector<std::string> & candidates, Selection & out);

/// The graph topics a pattern may match, sorted. Leaves out `node_name`'s own topics, whose events
/// are already written into the bag, and hidden topics, as `ros2 bag record` does by default.
std::vector<std::string> pattern_candidates(const TopicGraph & graph, const std::string & node_name);

/// The one type `topic` is published as, or nothing when it is not on the graph or is published
/// as several types and the caller has to say which.
std::optional<std::string> single_type(const TopicGraph & graph, const std::string & topic);

}  // namespace rosbag2_dynamic_recorder

#endif  // ROSBAG2_DYNAMIC_RECORDER__TOPIC_SELECTION_HPP_
