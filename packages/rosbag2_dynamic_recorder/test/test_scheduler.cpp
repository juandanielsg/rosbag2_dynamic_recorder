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

/// Message-time schedules against synthetic stamps, and node-time timers against a spinning
/// executor: the rules that decide when a queued resume or split fires.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_dynamic_recorder/scheduler.hpp"

using rosbag2_dynamic_recorder::Scheduler;
using Kind = Scheduler::Kind;
using Mode = Scheduler::Mode;

class SchedulerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("scheduler_test");
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    scheduler_ = std::make_unique<Scheduler>(*node_, group_);
  }

  /// Spin until `done` or `limit` passes; returns whether it was done.
  bool spin_until(const bool & done, std::chrono::milliseconds limit)
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_);
    const auto deadline = std::chrono::steady_clock::now() + limit;
    while (!done && std::chrono::steady_clock::now() < deadline) {
      executor.spin_once(std::chrono::milliseconds(10));
    }
    return done;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  std::unique_ptr<Scheduler> scheduler_;
};

TEST_F(SchedulerTest, modes_are_the_services_constants_and_nothing_else)
{
  EXPECT_EQ(Scheduler::mode_from(0), Mode::NodeTime);
  EXPECT_EQ(Scheduler::mode_from(1), Mode::PublishTime);
  EXPECT_EQ(Scheduler::mode_from(2), Mode::ReceiveTime);
  EXPECT_FALSE(Scheduler::mode_from(3));
  EXPECT_FALSE(Scheduler::mode_from(-1));
}

TEST_F(SchedulerTest, a_message_time_schedule_fires_once_on_the_clock_it_named)
{
  scheduler_->schedule_at_message_time(Kind::Resume, 1000, Mode::PublishTime, "");
  EXPECT_FALSE(scheduler_->fired("/a", 999, 5000).resume) << "publish time not reached";
  EXPECT_FALSE(scheduler_->fired("/a", 0, 5000).resume) << "a missing stamp never fires";
  EXPECT_TRUE(scheduler_->fired("/a", 1000, 0).resume);
  EXPECT_FALSE(scheduler_->fired("/a", 2000, 0).resume) << "each schedule fires once";

  scheduler_->schedule_at_message_time(Kind::Split, 1000, Mode::ReceiveTime, "");
  EXPECT_FALSE(scheduler_->fired("/a", 5000, 999).split) << "receive time not reached";
  EXPECT_TRUE(scheduler_->fired("/a", 0, 1000).split);
}

TEST_F(SchedulerTest, a_tracking_topic_restricts_which_message_can_fire_it)
{
  scheduler_->schedule_at_message_time(Kind::Resume, 100, Mode::ReceiveTime, "/scan");
  EXPECT_FALSE(scheduler_->fired("/odom", 0, 500).resume);
  EXPECT_TRUE(scheduler_->fired("/scan", 0, 500).resume);
}

TEST_F(SchedulerTest, clear_forgets_queued_schedules)
{
  scheduler_->schedule_at_message_time(Kind::Resume, 100, Mode::ReceiveTime, "");
  scheduler_->schedule_at_message_time(Kind::Split, 100, Mode::ReceiveTime, "");
  scheduler_->clear();
  const auto fired = scheduler_->fired("/a", 0, 500);
  EXPECT_FALSE(fired.resume || fired.split);
}

TEST_F(SchedulerTest, a_node_time_timer_fires_once_and_the_newest_arming_wins)
{
  bool first = false;
  bool second = false;
  scheduler_->arm(Kind::Resume, std::chrono::milliseconds(20), [&]() {first = true;});
  scheduler_->arm(Kind::Resume, std::chrono::milliseconds(40), [&]() {second = true;});
  EXPECT_TRUE(spin_until(second, std::chrono::seconds(2)));
  EXPECT_FALSE(first) << "the replaced timer must not fire";

  bool cleared = false;
  scheduler_->arm(Kind::Split, std::chrono::milliseconds(20), [&]() {cleared = true;});
  scheduler_->clear();
  EXPECT_FALSE(spin_until(cleared, std::chrono::milliseconds(100)));
}
