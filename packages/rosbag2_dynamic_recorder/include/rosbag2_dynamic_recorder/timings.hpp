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

#ifndef ROSBAG2_DYNAMIC_RECORDER__TIMINGS_HPP_
#define ROSBAG2_DYNAMIC_RECORDER__TIMINGS_HPP_

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace rosbag2_dynamic_recorder
{

/// Opt-in timing of the recorder's own work (parameter debug_timings), published on
/// ~/debug/timings.
///
/// Built so that it cannot distort what it measures. Off, every hook is one predictable branch
/// and nothing is read, stored or published. On, the cost stays off the per-message path: an
/// operation is timed phase by phase (a clock read per phase, against phases of milliseconds),
/// and a recorded message is only timed when it finds the write lock already held, where two
/// clock reads are small next to the wait itself.
class Timings
{
public:
  struct Phase
  {
    const char * name;
    int64_t ns;
  };

  struct Record
  {
    std::string operation;
    int64_t start_mono_ns;
    std::vector<Phase> phases;
  };

  struct Contention
  {
    int64_t since_mono_ns;
    uint64_t waits;
    int64_t wait_ns;
    int64_t max_wait_ns;
    uint64_t staged;
    uint64_t stage_dropped;
  };

  /// One operation, timed on the thread that runs it. Inert when timings are off. While it is
  /// alive, phase_here() on the same thread marks its phases, so code that does not know whether
  /// it runs inside an operation (the bag) can still say where the time went.
  class Operation
  {
public:
    Operation(Timings & timings, std::string name);
    ~Operation();
    Operation(const Operation &) = delete;
    Operation & operator=(const Operation &) = delete;

    /// Close the running phase under `name` and start the next. `name` must outlive the
    /// operation: a string literal.
    void phase(const char * name);

private:
    Timings * timings_;  // null when off
    Operation * outer_;
    Record record_;
    int64_t phase_start_;
  };

  explicit Timings(bool enabled);

  bool enabled() const {return enabled_;}

  /// Mark a phase of the operation running on this thread, if there is one.
  static void phase_here(const char * name);

  /// A recorded message waited `ns` for the write lock.
  void note_lock_wait(int64_t ns);

  /// A recorded message was staged while a slow operation held the writer, or dropped because
  /// the stage was full.
  void note_staged(bool dropped);

  /// Finished operations since the last call, oldest first.
  std::vector<Record> drain_operations();

  /// Lock contention since the last call.
  Contention drain_contention();

  static int64_t now_ns();

private:
  void finish(Record && record);

  const bool enabled_;
  std::mutex mutex_;
  std::vector<Record> finished_;

  std::atomic<int64_t> contention_since_;
  std::atomic<uint64_t> waits_{0};
  std::atomic<int64_t> wait_ns_{0};
  std::atomic<int64_t> max_wait_ns_{0};
  std::atomic<uint64_t> staged_{0};
  std::atomic<uint64_t> stage_dropped_{0};
};

}  // namespace rosbag2_dynamic_recorder

#endif  // ROSBAG2_DYNAMIC_RECORDER__TIMINGS_HPP_
