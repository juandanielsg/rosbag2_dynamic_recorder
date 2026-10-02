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

#include "rosbag2_dynamic_recorder/topic_selection.hpp"

#include <regex>
#include <string>
#include <unordered_set>
#include <vector>

namespace rosbag2_dynamic_recorder
{

std::string select_topics(
  const std::vector<std::string> & topics, const std::vector<std::string> & types,
  const std::string & include_regex, const std::string & exclude_regex,
  const std::vector<std::string> & candidates, Selection & out)
{
  if (!types.empty() && types.size() != topics.size()) {
    return "topic_types must be empty or the same length as topics";
  }
  Selection selected{topics, types};
  selected.types.resize(selected.topics.size());
  try {
    if (!include_regex.empty()) {
      const std::regex include(include_regex);
      const std::unordered_set<std::string> named(topics.begin(), topics.end());
      for (const auto & name : candidates) {
        if (named.count(name) == 0 && std::regex_search(name, include)) {
          selected.topics.push_back(name);
          selected.types.emplace_back();
        }
      }
    }
    if (!exclude_regex.empty()) {
      const std::regex exclude(exclude_regex);
      Selection kept;
      for (size_t i = 0; i < selected.topics.size(); ++i) {
        if (!std::regex_search(selected.topics[i], exclude)) {
          kept.topics.push_back(selected.topics[i]);
          kept.types.push_back(selected.types[i]);
        }
      }
      selected = std::move(kept);
    }
  } catch (const std::regex_error & e) {
    // Refused rather than matching nothing, which would look like a pattern that found no topics.
    return std::string("invalid regular expression: ") + e.what();
  }
  out = std::move(selected);
  return "";
}

std::vector<std::string> pattern_candidates(const TopicGraph & graph, const std::string & node_name)
{
  const std::string own_prefix = node_name + "/";
  std::vector<std::string> candidates;
  for (const auto & entry : graph) {
    const auto & name = entry.first;
    const bool own = name.rfind(own_prefix, 0) == 0;
    // A name token starting with an underscore marks a hidden topic.
    const bool hidden = name.find("/_") != std::string::npos;
    if (!own && !hidden) {
      candidates.push_back(name);
    }
  }
  return candidates;  // Sorted already: the graph is a std::map.
}

std::optional<std::string> single_type(const TopicGraph & graph, const std::string & topic)
{
  const auto it = graph.find(topic);
  if (it == graph.end() || it->second.size() != 1) {
    return std::nullopt;
  }
  return it->second.front();
}

}  // namespace rosbag2_dynamic_recorder
