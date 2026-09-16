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

#include "rosbag2_dynamic_recorder/storage_guard.hpp"

#include <algorithm>
#include <filesystem>
#include <string>

namespace rosbag2_dynamic_recorder
{

StorageGuard::StorageGuard(Limits limits, std::chrono::steady_clock::duration cache_ttl)
: limits_(limits), cache_ttl_(cache_ttl)
{
}

void StorageGuard::reset()
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  cached_bag_size_ = 0;
  cached_at_ = {};
}

StorageGuard::Space StorageGuard::filesystem_space(const std::string & path)
{
  std::error_code ec;
  const auto space = std::filesystem::space(path, ec);
  if (ec) {
    return {};
  }
  return {static_cast<uint64_t>(space.available), static_cast<uint64_t>(space.capacity), true};
}

uint64_t StorageGuard::bag_size(const std::string & uri) const
{
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (cached_at_.time_since_epoch().count() != 0 && now - cached_at_ < cache_ttl_) {
      return cached_bag_size_;
    }
  }
  return measure_bag_size(uri);
}

uint64_t StorageGuard::measure_bag_size(const std::string & uri) const
{
  namespace fs = std::filesystem;
  std::error_code ec;
  uint64_t total = 0;
  if (fs::is_directory(uri, ec)) {
    for (fs::directory_iterator it(uri, ec), end; it != end; it.increment(ec)) {
      if (ec) {
        break;  // Report what was counted rather than failing the whole status call.
      }
      if (it->is_regular_file(ec)) {
        const auto size = it->file_size(ec);
        if (!ec) {
          total += size;
        }
      }
    }
  }
  std::lock_guard<std::mutex> lock(cache_mutex_);
  cached_bag_size_ = total;
  cached_at_ = std::chrono::steady_clock::now();
  return total;
}

std::optional<StorageGuard::LowDisk> StorageGuard::low_disk(const std::string & uri) const
{
  const Space space = filesystem_space(uri);
  if (!space.valid) {
    return std::nullopt;
  }
  const auto percent_bytes = static_cast<uint64_t>(
    static_cast<double>(space.total_bytes) * limits_.min_free_space_percent / 100.0);
  const uint64_t threshold = std::max(limits_.min_free_space, percent_bytes);
  if (space.free_bytes >= threshold) {
    return std::nullopt;
  }
  return LowDisk{space, threshold};
}

std::optional<StorageGuard::BagTooBig> StorageGuard::bag_too_big(const std::string & uri) const
{
  if (limits_.max_bag_size == 0) {
    return std::nullopt;
  }
  const uint64_t size = measure_bag_size(uri);
  if (size <= limits_.max_bag_size) {
    return std::nullopt;
  }
  return BagTooBig{size};
}

}  // namespace rosbag2_dynamic_recorder
