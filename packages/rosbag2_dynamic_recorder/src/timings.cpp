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

#include "rosbag2_dynamic_recorder/timings.hpp"

#include <chrono>
#include <string>
#include <utility>
#include <vector>

namespace rosbag2_dynamic_recorder
{

namespace
{
// The operation running on this thread, if any. Operations nest (record() subscribes topics),
// and the innermost one collects the phases.
thread_local Timings::Operation * t_current = nullptr;
}  // namespace

int64_t Timings::now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

Timings::Timings(bool enabled)
: enabled_(enabled), contention_since_(enabled ? now_ns() : 0)
{
}

Timings::Operation::Operation(Timings & timings, std::string name)
: timings_(timings.enabled() ? &timings : nullptr), outer_(nullptr), phase_start_(0)
{
  if (timings_ == nullptr) {
    return;
  }
  record_.operation = std::move(name);
  record_.start_mono_ns = now_ns();
  phase_start_ = record_.start_mono_ns;
  outer_ = t_current;
  t_current = this;
}

Timings::Operation::~Operation()
{
  if (timings_ == nullptr) {
    return;
  }
  // Whatever ran after the last marked phase is still the operation's time; name it so the
  // phases always add up to the whole.
  phase("rest");
  t_current = outer_;
  timings_->finish(std::move(record_));
}

void Timings::Operation::phase(const char * name)
{
  if (timings_ == nullptr) {
    return;
  }
  const int64_t now = now_ns();
  record_.phases.push_back({name, now - phase_start_});
  phase_start_ = now;
}

void Timings::phase_here(const char * name)
{
  if (t_current != nullptr) {
    t_current->phase(name);
  }
}

void Timings::note_lock_wait(int64_t ns)
{
  waits_.fetch_add(1, std::memory_order_relaxed);
  wait_ns_.fetch_add(ns, std::memory_order_relaxed);
  int64_t seen = max_wait_ns_.load(std::memory_order_relaxed);
  while (ns > seen &&
    !max_wait_ns_.compare_exchange_weak(seen, ns, std::memory_order_relaxed))
  {
  }
}

void Timings::note_staged(bool dropped)
{
  (dropped ? stage_dropped_ : staged_).fetch_add(1, std::memory_order_relaxed);
}

void Timings::finish(Record && record)
{
  std::lock_guard<std::mutex> lock(mutex_);
  finished_.push_back(std::move(record));
}

std::vector<Timings::Record> Timings::drain_operations()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<Record> out;
  out.swap(finished_);
  return out;
}

Timings::Contention Timings::drain_contention()
{
  const int64_t now = now_ns();
  // Not one atomic snapshot: a wait landing between these reads is counted in one period or the
  // next, never lost and never twice, which is all a per-period rate needs.
  return {
    contention_since_.exchange(now, std::memory_order_relaxed),
    waits_.exchange(0, std::memory_order_relaxed),
    wait_ns_.exchange(0, std::memory_order_relaxed),
    max_wait_ns_.exchange(0, std::memory_order_relaxed),
    staged_.exchange(0, std::memory_order_relaxed),
    stage_dropped_.exchange(0, std::memory_order_relaxed),
  };
}

}  // namespace rosbag2_dynamic_recorder
