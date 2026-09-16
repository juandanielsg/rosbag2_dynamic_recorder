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

#ifndef ROSBAG2_DYNAMIC_RECORDER__STORAGE_GUARD_HPP_
#define ROSBAG2_DYNAMIC_RECORDER__STORAGE_GUARD_HPP_

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

namespace rosbag2_dynamic_recorder
{

/// What the recorder knows about the disk under the bag, and whether it should stop for it.
///
/// Two independent limits: a floor on free space, which protects the filesystem from the bag,
/// and a ceiling on the bag directory, which caps the recording. Both are measured on demand,
/// and the bag size is memoised briefly because it is the one status field that costs a walk.
/// No ROS in here, so it is tested against a directory rather than a node.
class StorageGuard
{
public:
  struct Limits
  {
    /// Stop when free space falls below either bound; 0 disables that bound. When both are set
    /// the stricter applies.
    uint64_t min_free_space{0};
    double min_free_space_percent{0.0};
    /// Stop when the bag directory, across every split, grows past this. 0 disables.
    uint64_t max_bag_size{0};
  };

  struct Space
  {
    uint64_t free_bytes{0};
    uint64_t total_bytes{0};
    /// False when the filesystem could not be read; the figures are then 0, not "empty".
    bool valid{false};
  };

  struct LowDisk
  {
    Space space;
    /// The bound that was breached, in bytes, for the log line.
    uint64_t threshold{0};
  };

  struct BagTooBig
  {
    uint64_t bag_size{0};
  };

  /// How long a bag-size measurement stays good enough to reuse. The status is published every
  /// second; bag size is already documented as bytes flushed rather than captured, so a few
  /// seconds of staleness on top changes nothing a caller could act on.
  static constexpr std::chrono::seconds kDefaultCacheTtl{3};

  explicit StorageGuard(
    Limits limits, std::chrono::steady_clock::duration cache_ttl = kDefaultCacheTtl);

  const Limits & limits() const {return limits_;}

  /// Forget the cached size: a new bag is empty, and last bag's figure would be actively wrong.
  void reset();

  /// Free and total bytes on the filesystem holding `path`. `available` rather than `free`:
  /// reserved blocks are not writable by an ordinary process, and counting them would let the
  /// check fire too late on exactly the embedded filesystems it exists to protect.
  static Space filesystem_space(const std::string & path);

  /// Bytes on disk under `uri`, reusing a measurement younger than the cache TTL.
  uint64_t bag_size(const std::string & uri) const;

  /// Bytes on disk under `uri`, measured now. Refreshes the cache. Not recursive: a rosbag2 bag
  /// directory is flat, so descending would only add a walk that grows with the split count.
  uint64_t measure_bag_size(const std::string & uri) const;

  /// The free-space breach to act on, if any. Never fires on a filesystem that could not be
  /// read: an unknown figure is not a low one.
  std::optional<LowDisk> low_disk(const std::string & uri) const;

  /// The size-cap breach to act on, if any. Measures fresh: the guard must not stop late because
  /// the status cache was warm.
  std::optional<BagTooBig> bag_too_big(const std::string & uri) const;

private:
  Limits limits_;
  std::chrono::steady_clock::duration cache_ttl_;
  mutable std::mutex cache_mutex_;
  mutable uint64_t cached_bag_size_{0};
  mutable std::chrono::steady_clock::time_point cached_at_{};
};

}  // namespace rosbag2_dynamic_recorder

#endif  // ROSBAG2_DYNAMIC_RECORDER__STORAGE_GUARD_HPP_
