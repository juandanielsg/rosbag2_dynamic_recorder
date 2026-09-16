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

/// The storage guard against a real directory: what it counts, what it caches, when it fires.

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#include "rosbag2_dynamic_recorder/storage_guard.hpp"

using rosbag2_dynamic_recorder::StorageGuard;
namespace fs = std::filesystem;

class StorageGuardTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dir_ = fs::temp_directory_path() / "storage_guard_test";
    fs::remove_all(dir_);
    fs::create_directories(dir_ / "nested");
    write(dir_ / "a.mcap", 100);
    write(dir_ / "b.mcap", 50);
    write(dir_ / "nested" / "ignored.mcap", 1000);
  }
  void TearDown() override {fs::remove_all(dir_);}

  static void write(const fs::path & path, size_t bytes)
  {
    std::ofstream(path) << std::string(bytes, 'x');
  }

  fs::path dir_;
};

TEST_F(StorageGuardTest, bag_size_counts_regular_files_in_the_directory_only)
{
  StorageGuard guard({});
  EXPECT_EQ(guard.measure_bag_size(dir_.string()), 150u) << "nested files are not a bag's";
  EXPECT_EQ(guard.measure_bag_size((dir_ / "missing").string()), 0u);
}

TEST_F(StorageGuardTest, the_cached_size_outlives_a_change_until_reset_or_expiry)
{
  StorageGuard guard({}, std::chrono::hours(1));
  EXPECT_EQ(guard.bag_size(dir_.string()), 150u);
  write(dir_ / "c.mcap", 25);
  EXPECT_EQ(guard.bag_size(dir_.string()), 150u) << "within the TTL the old figure is reused";
  guard.reset();
  EXPECT_EQ(guard.bag_size(dir_.string()), 175u);

  StorageGuard uncached({}, std::chrono::seconds(0));
  EXPECT_EQ(uncached.bag_size(dir_.string()), 175u);
  write(dir_ / "d.mcap", 1);
  EXPECT_EQ(uncached.bag_size(dir_.string()), 176u);
}

TEST_F(StorageGuardTest, the_size_cap_measures_fresh_and_is_off_at_zero)
{
  StorageGuard capped({0, 0.0, 120}, std::chrono::hours(1));
  EXPECT_EQ(capped.bag_size(dir_.string()), 150u);  // warms the cache
  fs::remove(dir_ / "a.mcap");
  EXPECT_FALSE(capped.bag_too_big(dir_.string())) << "50 bytes on disk now, whatever the cache says";
  write(dir_ / "a.mcap", 100);
  const auto verdict = capped.bag_too_big(dir_.string());
  ASSERT_TRUE(verdict);
  EXPECT_EQ(verdict->bag_size, 150u);

  StorageGuard uncapped({0, 0.0, 0});
  EXPECT_FALSE(uncapped.bag_too_big(dir_.string()));
}

TEST_F(StorageGuardTest, the_free_space_floor_is_the_stricter_bound_and_never_fires_blind)
{
  const auto space = StorageGuard::filesystem_space(dir_.string());
  ASSERT_TRUE(space.valid);
  ASSERT_GT(space.total_bytes, 0u);

  // A byte floor above everything the disk has must fire, and report itself as the threshold.
  StorageGuard bytes({space.total_bytes + 1, 0.0, 0});
  auto low = bytes.low_disk(dir_.string());
  ASSERT_TRUE(low);
  EXPECT_EQ(low->threshold, space.total_bytes + 1);

  // A percentage floor above what is free must fire, and win over a smaller byte floor.
  StorageGuard percent({1, 100.0, 0});
  low = percent.low_disk(dir_.string());
  ASSERT_TRUE(low) << "no filesystem has 100% of its capacity free";
  EXPECT_EQ(low->threshold, space.total_bytes);

  EXPECT_FALSE(StorageGuard({0, 0.0, 0}).low_disk(dir_.string())) << "no floor, no verdict";
  EXPECT_FALSE(StorageGuard({space.total_bytes + 1, 100.0, 0}).low_disk("/definitely/not/here"))
    << "an unreadable filesystem is unknown, not empty";
}
