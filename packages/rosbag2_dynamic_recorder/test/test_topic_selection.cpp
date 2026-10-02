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

/// Turning names, patterns and the graph into the topics a call records, without a graph.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "rosbag2_dynamic_recorder/topic_selection.hpp"

using rosbag2_dynamic_recorder::Selection;
using rosbag2_dynamic_recorder::TopicGraph;
using rosbag2_dynamic_recorder::pattern_candidates;
using rosbag2_dynamic_recorder::select_topics;
using rosbag2_dynamic_recorder::single_type;
using Topics = std::vector<std::string>;

TEST(TopicSelection, named_topics_keep_their_types_and_matches_get_none)
{
  Selection out;
  EXPECT_EQ(
    select_topics(
      {"/scan"}, {"sensor_msgs/msg/LaserScan"}, "camera", "",
      {"/scan", "/robot/camera/image", "/odom"}, out), "");
  EXPECT_EQ(out.topics, (Topics{"/scan", "/robot/camera/image"})) << "a search, not a full match";
  EXPECT_EQ(out.types, (Topics{"sensor_msgs/msg/LaserScan", ""}));
}

TEST(TopicSelection, the_exclusion_filters_named_topics_too)
{
  Selection out;
  EXPECT_EQ(
    select_topics({"/robot/depth"}, {}, "^/robot/", "depth", {"/robot/depth", "/robot/rgb"}, out),
    "");
  EXPECT_EQ(out.topics, (Topics{"/robot/rgb"}));
  EXPECT_EQ(out.types, (Topics{""})) << "types stay aligned with topics";
}

TEST(TopicSelection, invalid_requests_are_refused_with_the_reason)
{
  Selection out{{"untouched"}, {""}};
  EXPECT_NE(select_topics({"/a", "/b"}, {"t"}, "", "", {}, out), "") << "one type for two topics";
  EXPECT_NE(select_topics({}, {}, "(", "", {"/a"}, out).find("invalid regular expression"),
    std::string::npos);
  EXPECT_EQ(out.topics, (Topics{"untouched"})) << "a refused selection leaves the output alone";
}

TEST(TopicSelection, patterns_never_match_the_recorders_own_or_hidden_topics)
{
  const TopicGraph graph{
    {"/rec/events/pause", {"t"}}, {"/recorder_b/status", {"t"}}, {"/_hidden", {"t"}},
    {"/ns/_private/x", {"t"}}, {"/scan", {"t"}}};
  EXPECT_EQ(pattern_candidates(graph, "/rec"), (Topics{"/recorder_b/status", "/scan"}));
}

TEST(TopicSelection, a_type_resolves_only_when_the_graph_has_exactly_one)
{
  const TopicGraph graph{{"/one", {"a/msg/A"}}, {"/two", {"a/msg/A", "b/msg/B"}}, {"/none", {}}};
  EXPECT_EQ(single_type(graph, "/one"), "a/msg/A");
  EXPECT_FALSE(single_type(graph, "/two")) << "ambiguous";
  EXPECT_FALSE(single_type(graph, "/none"));
  EXPECT_FALSE(single_type(graph, "/absent"));
}
