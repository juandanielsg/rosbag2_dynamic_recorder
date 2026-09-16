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

/// The three loss figures and the per-topic report, without a transport to lose anything.

#include <gtest/gtest.h>

#include <string>

#include "rosbag2_dynamic_recorder/loss_accounting.hpp"

using rosbag2_dynamic_recorder::LossAccounting;

TEST(LossAccounting, missed_is_the_gap_in_each_publishers_sequence)
{
  LossAccounting losses;
  EXPECT_FALSE(losses.totals().sequence_numbers_available) << "nothing seen yet";

  LossAccounting::SequenceMap seen;
  losses.note_sequence(seen, "pub-a", 10);
  EXPECT_TRUE(losses.totals().sequence_numbers_available);
  EXPECT_EQ(losses.totals().missed, 0u) << "the first number only sets the position";
  losses.note_sequence(seen, "pub-a", 11);
  losses.note_sequence(seen, "pub-a", 14);
  EXPECT_EQ(losses.totals().missed, 2u) << "12 and 13 never arrived";

  // A second publisher on the same topic has its own counter: 1 after 14 is not a gap.
  losses.note_sequence(seen, "pub-b", 1);
  losses.note_sequence(seen, "pub-b", 2);
  EXPECT_EQ(losses.totals().missed, 2u);

  // A fresh map, as a re-subscribed topic gets, starts clean however far the publisher got.
  LossAccounting::SequenceMap fresh;
  losses.note_sequence(fresh, "pub-a", 500);
  EXPECT_EQ(losses.totals().missed, 2u);
}

TEST(LossAccounting, transport_and_recorder_losses_stay_apart_and_drain_per_topic)
{
  LossAccounting losses;
  EXPECT_TRUE(losses.drain().empty()) << "no losses, no report";

  losses.note_transport_loss("/scan", 3);
  losses.note_recorder_loss("/scan", 5);
  losses.note_recorder_loss("/odom", 1);
  const auto totals = losses.totals();
  EXPECT_EQ(totals.lost_in_transport, 3u);
  EXPECT_EQ(totals.lost_in_recorder, 6u);

  auto report = losses.drain();
  ASSERT_EQ(report.size(), 2u);
  const auto & scan = report[0].topic == "/scan" ? report[0] : report[1];
  EXPECT_EQ(scan.in_transport, 3u);
  EXPECT_EQ(scan.in_recorder, 5u);
  EXPECT_TRUE(losses.drain().empty()) << "a drain reports each loss once";
  EXPECT_EQ(losses.totals().lost_in_recorder, 6u) << "draining the report leaves the totals";
}

TEST(LossAccounting, reset_forgets_undrained_deltas_along_with_the_totals)
{
  LossAccounting losses;
  LossAccounting::SequenceMap seen;
  losses.note_sequence(seen, "p", 1);
  losses.note_sequence(seen, "p", 5);
  losses.note_transport_loss("/scan", 2);
  losses.reset();
  const auto totals = losses.totals();
  EXPECT_EQ(totals.missed, 0u);
  EXPECT_EQ(totals.lost_in_transport, 0u);
  EXPECT_TRUE(losses.drain().empty()) << "last bag's losses must not open the next bag's report";
}
