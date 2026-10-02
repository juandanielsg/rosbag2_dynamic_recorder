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

/// debug_timings: what an operation records, that nesting and threads keep phases apart, and
/// that switched off it records nothing at all.

#include <gtest/gtest.h>

#include <string>
#include <thread>
#include <vector>

#include "rosbag2_dynamic_recorder/timings.hpp"

using rosbag2_dynamic_recorder::Timings;

namespace
{
std::vector<std::string> names(const Timings::Record & record)
{
  std::vector<std::string> out;
  for (const auto & phase : record.phases) {
    out.emplace_back(phase.name);
  }
  return out;
}
}  // namespace

TEST(Timings, off_records_nothing_and_phase_marks_are_harmless)
{
  Timings timings(false);
  {
    Timings::Operation operation(timings, "subscribe_topics");
    Timings::phase_here("graph_query");
    operation.phase("qos");
  }
  Timings::phase_here("outside any operation");
  timings.note_lock_wait(1000);  // never called when off, but must not matter if it were
  EXPECT_TRUE(timings.drain_operations().empty());
}

TEST(Timings, an_operation_records_its_phases_in_order_and_names_the_remainder)
{
  Timings timings(true);
  {
    Timings::Operation operation(timings, "subscribe_topics");
    Timings::phase_here("graph_query");
    Timings::phase_here("create_topic");
  }
  const auto records = timings.drain_operations();
  ASSERT_EQ(records.size(), 1u);
  EXPECT_EQ(records[0].operation, "subscribe_topics");
  EXPECT_EQ(names(records[0]), (std::vector<std::string>{"graph_query", "create_topic", "rest"}));
  for (const auto & phase : records[0].phases) {
    EXPECT_GE(phase.ns, 0);
  }
  EXPECT_TRUE(timings.drain_operations().empty()) << "drained once";
}

TEST(Timings, a_nested_operation_collects_its_own_phases_and_hands_back)
{
  Timings timings(true);
  {
    Timings::Operation outer(timings, "record");
    Timings::phase_here("writer_open");
    {
      Timings::Operation inner(timings, "schedule:resume");
      Timings::phase_here("event_write");
    }
    Timings::phase_here("resubscribe");
  }
  const auto records = timings.drain_operations();
  ASSERT_EQ(records.size(), 2u);
  EXPECT_EQ(records[0].operation, "schedule:resume") << "finished first";
  EXPECT_EQ(names(records[0]), (std::vector<std::string>{"event_write", "rest"}));
  EXPECT_EQ(records[1].operation, "record");
  EXPECT_EQ(names(records[1]), (std::vector<std::string>{"writer_open", "resubscribe", "rest"}));
}

TEST(Timings, phases_marked_on_another_thread_do_not_land_in_this_operation)
{
  Timings timings(true);
  {
    Timings::Operation operation(timings, "stop");
    std::thread([] {Timings::phase_here("from a subscription callback");}).join();
  }
  const auto records = timings.drain_operations();
  ASSERT_EQ(records.size(), 1u);
  EXPECT_EQ(names(records[0]), (std::vector<std::string>{"rest"}));
}

TEST(Timings, lock_contention_accumulates_and_resets_per_drain)
{
  Timings timings(true);
  timings.note_lock_wait(100);
  timings.note_lock_wait(300);
  timings.note_lock_wait(200);
  const auto first = timings.drain_contention();
  EXPECT_EQ(first.waits, 3u);
  EXPECT_EQ(first.wait_ns, 600);
  EXPECT_EQ(first.max_wait_ns, 300);
  const auto second = timings.drain_contention();
  EXPECT_EQ(second.waits, 0u);
  EXPECT_EQ(second.max_wait_ns, 0);
  EXPECT_GE(second.since_mono_ns, first.since_mono_ns) << "periods follow on";
}

TEST(Timings, staged_and_dropped_messages_are_counted_apart)
{
  Timings timings(true);
  timings.note_staged(false);
  timings.note_staged(false);
  timings.note_staged(true);
  const auto period = timings.drain_contention();
  EXPECT_EQ(period.staged, 2u);
  EXPECT_EQ(period.stage_dropped, 1u);
  EXPECT_EQ(timings.drain_contention().staged, 0u) << "reset per period";
}
