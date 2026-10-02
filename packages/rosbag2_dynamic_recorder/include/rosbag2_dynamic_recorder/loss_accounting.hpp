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

#ifndef ROSBAG2_DYNAMIC_RECORDER__LOSS_ACCOUNTING_HPP_
#define ROSBAG2_DYNAMIC_RECORDER__LOSS_ACCOUNTING_HPP_

#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace rosbag2_dynamic_recorder
{

/// Every way a message can go missing, counted where it is noticed and reported in one place.
///
/// Three figures, kept apart because their remedies are opposite. `missed` is detected from gaps
/// in publisher sequence numbers and catches loss that raises no event. `lost_in_transport` is
/// what the transport chose to report before delivery, a floor rather than a total; it points at
/// the publisher, its QoS or the network. `lost_in_recorder` is what the writer dropped after
/// delivery -- a full cache or a failed write -- and points at the cache, the storage preset,
/// the topic set or the disk. Per-topic deltas are kept for the periodic MessagesLostEvent, and
/// per-topic totals over the bag for the status.
///
/// Thread safety: the totals are atomics, the per-topic map has its own lock, and a sequence
/// map belongs to one subscription and is only ever touched from that subscription's callback.
class LossAccounting
{
public:
  /// Last publication sequence seen per publisher on one subscription. Per publisher, not per
  /// topic: two publishers on one topic have unrelated counters. Owned by the subscription, so a
  /// dropped-and-readded topic starts clean rather than reading the publisher's continued
  /// counting as loss.
  using SequenceMap = std::unordered_map<std::string, uint64_t>;

  struct Totals
  {
    uint64_t missed{0};
    uint64_t lost_in_transport{0};
    uint64_t lost_in_recorder{0};
    /// False until any message carries a sequence number; `missed` means nothing before then.
    bool sequence_numbers_available{false};
  };

  struct TopicLoss
  {
    std::string topic;
    uint64_t in_transport{0};
    uint64_t in_recorder{0};
    /// Only in topic_totals(); a drained report carries no missed figure.
    uint64_t missed{0};
  };

  /// Record `sequence` from `publisher` on a subscription's map, counting any gap since the
  /// last one as missed against `topic`. The first sequence from a publisher establishes the
  /// position only. Takes no lock unless there is a gap.
  void note_sequence(
    SequenceMap & seen, const std::string & publisher, uint64_t sequence,
    const std::string & topic);

  void note_transport_loss(const std::string & topic, uint64_t count);
  void note_recorder_loss(const std::string & topic, uint64_t count);

  /// Per-topic losses since the previous drain, then forget them. Empty when nothing was lost,
  /// so a report can be skipped rather than published empty.
  std::vector<TopicLoss> drain();

  Totals totals() const;

  /// Every figure per topic over this bag, for each topic that has lost anything, sorted by name.
  /// Unlike drain() it forgets nothing: it is what the status reports.
  std::vector<TopicLoss> topic_totals() const;

  /// A new bag starts every figure at zero, including the undrained per-topic deltas: those
  /// were owed to the previous bag and must not open the next one's first report.
  void reset();

private:
  std::atomic_uint64_t missed_{0};
  std::atomic_bool sequence_numbers_available_{false};
  std::atomic_uint64_t lost_in_transport_{0};
  std::atomic_uint64_t lost_in_recorder_{0};

  struct Pending
  {
    uint64_t in_transport{0};
    uint64_t in_recorder{0};
  };
  /// Guards both per-topic maps.
  mutable std::mutex pending_mutex_;
  std::unordered_map<std::string, Pending> pending_;
  /// Ordered, so topic_totals() comes out sorted without a sort.
  std::map<std::string, TopicLoss> per_topic_;
};

}  // namespace rosbag2_dynamic_recorder

#endif  // ROSBAG2_DYNAMIC_RECORDER__LOSS_ACCOUNTING_HPP_
