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

// The recorder's services and its status.

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rosbag2_dynamic_recorder/dynamic_recorder.hpp"

namespace rosbag2_dynamic_recorder
{

namespace
{
constexpr int kReturnSuccess = 0;
constexpr int kReturnError = 1;
constexpr int64_t kNanosecondsPerSecond = 1000000000LL;

/// Not rclcpp::Time: a request may carry a negative time, which rclcpp::Time throws on.
int64_t to_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * kNanosecondsPerSecond + stamp.nanosec;
}

builtin_interfaces::msg::Time to_time_msg(int64_t nanoseconds)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(nanoseconds / kNanosecondsPerSecond);
  stamp.nanosec = static_cast<uint32_t>(nanoseconds % kNanosecondsPerSecond);
  return stamp;
}

template<typename ResponseT>
void refuse(ResponseT & response, const std::string & error)
{
  response.return_code = kReturnError;
  response.error_string = error;
}

/// The verdict of a call that subscribes a batch. Every requested topic lands in subscribed or
/// unavailable, so nothing subscribed and something unavailable means the whole call failed; a
/// partial success still carries the reason.
template<typename ResponseT>
void set_subscribe_result(ResponseT & response, bool subscribed_any, const std::string & error)
{
  const bool all_failed = !subscribed_any && !response.unavailable_topics.empty();
  response.return_code = all_failed ? kReturnError : kReturnSuccess;
  response.error_string = all_failed && error.empty() ?
    "none of the requested topics could be subscribed" :
    (response.unavailable_topics.empty() ? "" : error);
}
}  // namespace

template<typename SrvT>
void DynamicRecorder::serve(
  const std::string & name,
  void (DynamicRecorder::* handler)(
    std::shared_ptr<typename SrvT::Request>, std::shared_ptr<typename SrvT::Response>))
{
  const bool timed = name.rfind("get_", 0) != 0 && name.rfind("is_", 0) != 0;
  services_.push_back(
    create_service<SrvT>(
      "~/" + name,
      [this, handler, name, timed](
        std::shared_ptr<typename SrvT::Request> request,
        std::shared_ptr<typename SrvT::Response> response)
      {
        std::optional<Timings::Operation> operation;
        if (timed) {
          operation.emplace(*timings_, name);
        }
        (this->*handler)(std::move(request), std::move(response));
      },
      rclcpp::ServicesQoS(), service_callback_group_));
}

void DynamicRecorder::create_services()
{
  serve<SubscribeTopics>("subscribe_topics", &DynamicRecorder::handle_subscribe_topics);
  serve<UnsubscribeTopics>("unsubscribe_topics", &DynamicRecorder::handle_unsubscribe_topics);
  serve<SetTopics>("set_topics", &DynamicRecorder::handle_set_topics);
  serve<SetProfile>("set_profile", &DynamicRecorder::handle_set_profile);
  serve<GetProfiles>("get_profiles", &DynamicRecorder::handle_get_profiles);
  serve<GetSubscribedTopics>(
    "get_subscribed_topics", &DynamicRecorder::handle_get_subscribed_topics);
  serve<Pause>("pause", &DynamicRecorder::handle_pause);
  serve<Resume>("resume", &DynamicRecorder::handle_resume);
  serve<TogglePaused>("toggle_paused", &DynamicRecorder::handle_toggle_paused);
  serve<IsPaused>("is_paused", &DynamicRecorder::handle_is_paused);
  serve<SplitBagfile>("split_bagfile", &DynamicRecorder::handle_split_bagfile);
  serve<Snapshot>("snapshot", &DynamicRecorder::handle_snapshot);
  serve<Stop>("stop", &DynamicRecorder::handle_stop);
  serve<Record>("record", &DynamicRecorder::handle_record);
  serve<GetStatus>("get_status", &DynamicRecorder::handle_get_status);
}

std::string DynamicRecorder::select_from_graph(
  const std::vector<std::string> & topics, const std::vector<std::string> & types,
  const std::string & regex, const std::string & exclude_regex, Selection & out) const
{
  // Otherwise the topics would come back "unavailable", which means absent from the graph.
  if (!is_recording()) {
    return not_recording_reason();
  }
  // Only a pattern needs the graph.
  const auto candidates = regex.empty() ? std::vector<std::string>{} :
    pattern_candidates(get_topic_names_and_types(), get_fully_qualified_name());
  return select_topics(topics, types, regex, exclude_regex, candidates, out);
}

void DynamicRecorder::handle_subscribe_topics(
  std::shared_ptr<SubscribeTopics::Request> request,
  std::shared_ptr<SubscribeTopics::Response> response)
{
  Selection selection;
  auto error = select_from_graph(
    request->topics, request->topic_types, request->regex, request->exclude_regex, selection);
  // Subscribe exists to add topics: a pattern that adds none is most likely a typo.
  if (error.empty() && selection.topics.empty() && !request->regex.empty()) {
    error = "regex '" + request->regex + "' matched no topics on the graph";
  }
  if (!error.empty()) {
    refuse(*response, error);
    return;
  }
  const auto result = subscribe_batch(selection, "service:subscribe_topics");
  response->subscribed_topics = result.subscribed;
  response->unavailable_topics = result.unavailable;
  set_subscribe_result(*response, !result.subscribed.empty(), result.error);
  publish_status();
}

void DynamicRecorder::handle_unsubscribe_topics(
  std::shared_ptr<UnsubscribeTopics::Request> request,
  std::shared_ptr<UnsubscribeTopics::Response> response)
{
  // A pattern matches what is being recorded, not the graph: only a recorded topic can be dropped.
  Selection selection;
  const auto error = select_topics(
    request->topics, {}, request->regex, request->exclude_regex, subscribed_topics(), selection);
  if (!error.empty()) {
    refuse(*response, error);
    return;
  }
  for (const auto & topic : selection.topics) {
    auto & list = unsubscribe_topic(topic, "service:unsubscribe_topics") ?
      response->unsubscribed_topics : response->not_subscribed_topics;
    list.push_back(topic);
  }
  log_topic_change("Unsubscribed", response->unsubscribed_topics);
  if (response->unsubscribed_topics.empty() && !selection.topics.empty()) {
    refuse(*response, "none of the requested topics were subscribed");
  } else {
    response->return_code = kReturnSuccess;
  }
  publish_status();
}

void DynamicRecorder::handle_set_topics(
  std::shared_ptr<SetTopics::Request> request, std::shared_ptr<SetTopics::Response> response)
{
  // Unlike subscribe, selecting nothing is valid: it stops every topic without closing the bag.
  Selection selection;
  const auto error = select_from_graph(
    request->topics, request->topic_types, request->regex, request->exclude_regex, selection);
  if (!error.empty()) {
    refuse(*response, error);
    return;
  }
  const auto result =
    replace_topics(selection, "service:set_topics", response->unsubscribed_topics);
  response->unavailable_topics = result.unavailable;
  set_subscribe_result(*response, !result.subscribed.empty(), result.error);
  response->subscribed_topics = subscribed_topics();
  publish_status();
}

void DynamicRecorder::handle_set_profile(
  std::shared_ptr<SetProfile::Request> request, std::shared_ptr<SetProfile::Response> response)
{
  if (!is_recording()) {
    refuse(*response, not_recording_reason());
    return;
  }
  const auto & profiles = config_.profiles;
  const auto profile = std::find_if(
    profiles.begin(), profiles.end(),
    [&request](const auto & entry) {return entry.first == request->name;});
  if (profile == profiles.end()) {
    std::string known;
    for (const auto & entry : profiles) {
      known += (known.empty() ? "" : ", ") + entry.first;
    }
    refuse(*response, "no profile named '" + request->name + "'" +
      (known.empty() ? "; none are configured" : "; configured: " + known));
    return;
  }
  // The same path as set_topics, so topics the two selections share record without a gap.
  const auto result = replace_topics(
    {profile->second, {}}, "service:set_profile:" + request->name,
    response->unsubscribed_topics);
  response->unavailable_topics = result.unavailable;
  set_subscribe_result(*response, !result.subscribed.empty(), result.error);
  response->subscribed_topics = subscribed_topics();
  publish_status();
}

void DynamicRecorder::handle_get_profiles(
  std::shared_ptr<GetProfiles::Request>, std::shared_ptr<GetProfiles::Response> response)
{
  for (const auto & [name, topics] : config_.profiles) {
    Profile profile;
    profile.name = name;
    profile.topics = topics;
    response->profiles.push_back(std::move(profile));
  }
  response->active_profile = active_profile_for(subscribed_topics());
}

void DynamicRecorder::handle_get_subscribed_topics(
  std::shared_ptr<GetSubscribedTopics::Request>,
  std::shared_ptr<GetSubscribedTopics::Response> response)
{
  response->topics = subscribed_topics();
}

void DynamicRecorder::handle_pause(
  std::shared_ptr<Pause::Request>, std::shared_ptr<Pause::Response>)
{
  pause("service:pause");
}

void DynamicRecorder::handle_resume(
  std::shared_ptr<Resume::Request> request, std::shared_ptr<Resume::Response> response)
{
  // Pause has no return code to refuse with, so only resume can say there is no bag.
  if (!is_recording()) {
    response->return_code = Resume::Response::RETURN_CODE_RESUME_FAILED;
    response->error_string = not_recording_reason();
    return;
  }
  const auto outcome = schedule_or_run(
    Scheduler::Kind::Resume, request->resume_time, request->resume_mode,
    request->tracking_topic_name,
    {Resume::Response::RETURN_CODE_SUCCESS, Resume::Response::RETURN_CODE_INVALID_RESUME_MODE,
      Resume::Response::RETURN_CODE_INVALID_TRACKING_TOPIC, Resume::Response::RETURN_CODE_SUCCESS},
    [this](const std::string & reason) {resume(reason); return true;});
  response->return_code = outcome.code;
  response->error_string = outcome.error;
}

void DynamicRecorder::handle_toggle_paused(
  std::shared_ptr<TogglePaused::Request>, std::shared_ptr<TogglePaused::Response>)
{
  if (paused_.load()) {
    resume("service:toggle_paused");
  } else {
    pause("service:toggle_paused");
  }
}

void DynamicRecorder::handle_is_paused(
  std::shared_ptr<IsPaused::Request>, std::shared_ptr<IsPaused::Response> response)
{
  response->paused = paused_.load();
}

void DynamicRecorder::handle_split_bagfile(
  std::shared_ptr<SplitBagfile::Request> request,
  std::shared_ptr<SplitBagfile::Response> response)
{
  if (!is_recording()) {
    response->return_code = SplitBagfile::Response::RETURN_CODE_NOT_RECORDING;
    response->error_string = "not recording";
    return;
  }
  const auto outcome = schedule_or_run(
    Scheduler::Kind::Split, request->split_time, request->split_mode,
    request->tracking_topic_name,
    {SplitBagfile::Response::RETURN_CODE_SUCCESS,
      SplitBagfile::Response::RETURN_CODE_INVALID_SPLIT_MODE,
      SplitBagfile::Response::RETURN_CODE_INVALID_TRACKING_TOPIC,
      SplitBagfile::Response::RETURN_CODE_SPLIT_FAILED},
    [this](const std::string & reason) {return bag_.split(reason);});
  response->return_code = outcome.code;
  response->error_string = outcome.error;
}

DynamicRecorder::ScheduleOutcome DynamicRecorder::schedule_or_run(
  Scheduler::Kind kind, const builtin_interfaces::msg::Time & at, int32_t mode,
  const std::string & tracking_topic, const ScheduleCodes & codes,
  std::function<bool(const std::string &)> action)
{
  const std::string verb = kind == Scheduler::Kind::Resume ? "resume" : "split";
  const std::string service = kind == Scheduler::Kind::Resume ? "resume" : "split_bagfile";
  const auto run_now = [&]() -> ScheduleOutcome {
      if (!action("service:" + service)) {
        return {codes.failed, "the writer could not " + verb + " the bag"};
      }
      return {codes.success, ""};
    };
  const auto at_ns = to_nanoseconds(at);
  if (at_ns == 0) {
    return run_now();
  }
  const auto parsed_mode = Scheduler::mode_from(mode);
  if (!parsed_mode) {
    return {codes.invalid_mode,
      verb + "_mode must be 0 (node time), 1 (publish time) or 2 (receive time)"};
  }
  if (!tracking_topic.empty()) {
    // A schedule on a topic nobody records would never fire.
    const auto current = subscribed_topics();
    if (std::find(current.begin(), current.end(), tracking_topic) == current.end()) {
      return {codes.invalid_topic,
        "tracking_topic_name '" + tracking_topic + "' is not being recorded"};
    }
  }
  if (*parsed_mode != Scheduler::Mode::NodeTime) {
    scheduler_->schedule_at_message_time(kind, at_ns, *parsed_mode, tracking_topic);
    publish_status();
    return {codes.success, ""};
  }
  const auto delay = at_ns - now().nanoseconds();
  if (delay <= 0) {
    return run_now();
  }
  scheduler_->schedule_at_node_time(
    kind, std::chrono::nanoseconds(delay),
    [this, action, verb]() {
      Timings::Operation operation(*timings_, "schedule:" + verb);
      action("schedule:" + verb);
      publish_status();
    });
  publish_status();
  return {codes.success, ""};
}

void DynamicRecorder::handle_snapshot(
  std::shared_ptr<Snapshot::Request>, std::shared_ptr<Snapshot::Response> response)
{
  response->success = bag_.take_snapshot();
  if (!response->success && !config_.snapshot_mode) {
    RCLCPP_WARN(get_logger(),
      "Snapshot failed because snapshot_mode is disabled -- set the snapshot_mode parameter");
  }
}

void DynamicRecorder::handle_stop(
  std::shared_ptr<Stop::Request>, std::shared_ptr<Stop::Response> response)
{
  if (!is_recording()) {
    refuse(*response, "recording is already stopped");
    return;
  }
  stop();
  response->return_code = kReturnSuccess;
}

void DynamicRecorder::handle_record(
  std::shared_ptr<Record::Request> request, std::shared_ptr<Record::Response> response)
{
  if (is_recording()) {
    refuse(*response, "already recording");
    return;
  }
  if (waiting_for_clock()) {
    refuse(*response, not_recording_reason());
    return;
  }
  // Record has no mode field, so a scheduled start is node time.
  const auto at_ns = to_nanoseconds(request->start_time);
  const auto delay = at_ns == 0 ? 0 : at_ns - now().nanoseconds();
  if (delay > 0) {
    scheduler_->schedule_at_node_time(
      Scheduler::Kind::Record, std::chrono::nanoseconds(delay),
      [this, uri = request->uri]() {
        Timings::Operation operation(*timings_, "schedule:record");
        const auto error = record(uri);
        if (!error.empty()) {
          RCLCPP_ERROR(get_logger(), "Scheduled recording failed to start: %s", error.c_str());
          publish_status();  // record() publishes on success only; the schedule has still gone.
        }
      });
    publish_status();
    response->return_code = kReturnSuccess;
    return;
  }
  const auto error = record(request->uri);
  if (!error.empty()) {
    refuse(*response, error);
    return;
  }
  response->return_code = kReturnSuccess;
}

void DynamicRecorder::handle_get_status(
  std::shared_ptr<GetStatus::Request>, std::shared_ptr<GetStatus::Response> response)
{
  response->status = build_status();
}

std::string DynamicRecorder::not_recording_reason() const
{
  return waiting_for_clock() ?
         "waiting for /clock: use_sim_time is set and nothing has been published on it yet" :
         "recorder is stopped; call ~/record to open a new bag first";
}

std::string DynamicRecorder::active_profile_for(const std::vector<std::string> & topics) const
{
  for (const auto & [name, profile_topics] : config_.profiles) {
    if (profile_topics == topics) {
      return name;
    }
  }
  return "";
}

RecorderStatus DynamicRecorder::build_status() const
{
  RecorderStatus status;
  const auto bag = bag_.info();
  const auto stamp = now();
  status.stamp = stamp;
  status.uri = bag.uri;
  status.storage_id = config_.storage.storage_id;
  status.recording = bag.open;
  status.paused = paused_.load();
  status.snapshot_mode = config_.snapshot_mode;
  status.recording_started = bag.opened_at;
  // A stopped recorder reports how long its last bag ran. Zero before any bag.
  status.elapsed_seconds = ((bag.open ? stamp : bag.closed_at) - bag.opened_at).seconds();
  status.subscribed_topics = subscribed_topics();
  status.active_profile = active_profile_for(status.subscribed_topics);

  status.messages_written = bag.messages_written;
  status.write_errors = bag.write_errors;
  status.bag_splits = bag.splits;
  const auto losses = losses_.totals();
  status.messages_missed = losses.missed;
  status.sequence_numbers_available = losses.sequence_numbers_available;
  status.messages_lost_in_transport = losses.lost_in_transport;
  status.messages_lost_in_recorder = losses.lost_in_recorder;
  status.messages_lost = losses.lost_in_transport + losses.lost_in_recorder;
  for (const auto & loss : losses_.topic_totals()) {
    auto & entry = status.topic_losses.emplace_back();
    entry.topic_name = loss.topic;
    entry.messages_missed = loss.missed;
    entry.messages_lost_in_transport = loss.in_transport;
    entry.messages_lost_in_recorder = loss.in_recorder;
  }

  status.bag_size_bytes = storage_guard_->bag_size(status.uri);
  const auto space = StorageGuard::filesystem_space(status.uri);
  status.free_space_bytes = space.free_bytes;
  status.total_space_bytes = space.total_bytes;
  status.min_free_space = config_.min_free_space;
  status.min_free_space_percent = config_.min_free_space_percent;
  status.stopped_for_low_disk = stopped_for_low_disk_.load();
  status.max_bag_size = config_.max_bag_size;
  status.stopped_for_max_bag_size = stopped_for_max_bag_size_.load();
  status.use_sim_time = config_.use_sim_time;
  status.waiting_for_clock = waiting_for_clock();

  for (const auto & scheduled : scheduler_->pending()) {
    using Action = decltype(status.schedules)::value_type;
    auto & entry = status.schedules.emplace_back();
    entry.action = scheduled.kind == Scheduler::Kind::Resume ? Action::RESUME :
      (scheduled.kind == Scheduler::Kind::Split ? Action::SPLIT : Action::RECORD);
    entry.time = to_time_msg(scheduled.at_ns);
    // Scheduler::Mode is numbered as the message's mode constants are.
    entry.mode = static_cast<uint8_t>(scheduled.mode);
    entry.tracking_topic = scheduled.tracking_topic;
  }
  return status;
}

void DynamicRecorder::publish_status()
{
  if (!pub_status_) {
    return;
  }
  try {
    pub_status_->publish(build_status());
  } catch (const std::exception & e) {
    // stop() publishes, and also runs from the destructor, possibly after context shutdown.
    RCLCPP_DEBUG(get_logger(), "Could not publish status: %s", e.what());
  }
}

}  // namespace rosbag2_dynamic_recorder
