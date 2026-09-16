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

#include "rosbag2_dynamic_recorder/loss_accounting.hpp"

#include <string>
#include <utility>
#include <vector>

namespace rosbag2_dynamic_recorder
{

void LossAccounting::note_sequence(
  SequenceMap & seen, const std::string & publisher, uint64_t sequence)
{
  sequence_numbers_available_.store(true, std::memory_order_relaxed);
  const auto previous = seen.find(publisher);
  if (previous != seen.end() && sequence > previous->second + 1) {
    missed_.fetch_add(sequence - previous->second - 1, std::memory_order_relaxed);
  }
  seen[publisher] = sequence;
}

void LossAccounting::note_transport_loss(const std::string & topic, uint64_t count)
{
  lost_in_transport_.fetch_add(count);
  std::lock_guard<std::mutex> lock(pending_mutex_);
  pending_[topic].in_transport += count;
}

void LossAccounting::note_recorder_loss(const std::string & topic, uint64_t count)
{
  lost_in_recorder_.fetch_add(count);
  std::lock_guard<std::mutex> lock(pending_mutex_);
  pending_[topic].in_recorder += count;
}

std::vector<LossAccounting::TopicLoss> LossAccounting::drain()
{
  std::vector<TopicLoss> losses;
  std::lock_guard<std::mutex> lock(pending_mutex_);
  losses.reserve(pending_.size());
  for (auto & [topic, counts] : pending_) {
    losses.push_back({topic, counts.in_transport, counts.in_recorder});
  }
  pending_.clear();
  return losses;
}

LossAccounting::Totals LossAccounting::totals() const
{
  return {
    missed_.load(std::memory_order_relaxed),
    lost_in_transport_.load(),
    lost_in_recorder_.load(),
    sequence_numbers_available_.load(std::memory_order_relaxed),
  };
}

void LossAccounting::reset()
{
  missed_.store(0, std::memory_order_relaxed);
  lost_in_transport_.store(0);
  lost_in_recorder_.store(0);
  std::lock_guard<std::mutex> lock(pending_mutex_);
  pending_.clear();
}

}  // namespace rosbag2_dynamic_recorder
