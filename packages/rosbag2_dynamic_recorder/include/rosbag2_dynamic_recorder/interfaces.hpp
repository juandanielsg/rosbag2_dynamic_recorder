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

#ifndef ROSBAG2_DYNAMIC_RECORDER__INTERFACES_HPP_
#define ROSBAG2_DYNAMIC_RECORDER__INTERFACES_HPP_

/// Every message and service type the recorder offers, under one name each.
///
/// Where the installed rosbag2_interfaces type is field-identical to the rosbag2 0.34 (Rolling)
/// one, the stock type is used, so a client written for stock rosbag2 can drive the recorder.
/// Elsewhere (Jazzy and Kilted: Record, Resume and SplitBagfile without scheduling, Stop without
/// a return code, no MessagesLostEvent) the copy in rosbag2_dynamic_recorder_interfaces stands in.
/// The ROSBAG2_DYNAMIC_RECORDER_HAS_UPSTREAM_* macros come from CMake, which reads the installed
/// headers; dynrec.services makes the same choice on the client side.

#include "rosbag2_dynamic_recorder_interfaces/msg/bag_size_limit_event.hpp"
#include "rosbag2_dynamic_recorder_interfaces/msg/debug_timings.hpp"
#include "rosbag2_dynamic_recorder_interfaces/msg/file_split_event.hpp"
#include "rosbag2_dynamic_recorder_interfaces/msg/low_disk_event.hpp"
#include "rosbag2_dynamic_recorder_interfaces/msg/pause_event.hpp"
#include "rosbag2_dynamic_recorder_interfaces/msg/profile.hpp"
#include "rosbag2_dynamic_recorder_interfaces/msg/recorder_status.hpp"
#include "rosbag2_dynamic_recorder_interfaces/msg/subscription_change_event.hpp"
#include "rosbag2_dynamic_recorder_interfaces/srv/get_profiles.hpp"
#include "rosbag2_dynamic_recorder_interfaces/srv/get_status.hpp"
#include "rosbag2_dynamic_recorder_interfaces/srv/get_subscribed_topics.hpp"
#include "rosbag2_dynamic_recorder_interfaces/srv/set_profile.hpp"
#include "rosbag2_dynamic_recorder_interfaces/srv/set_topics.hpp"
#include "rosbag2_dynamic_recorder_interfaces/srv/subscribe_topics.hpp"
#include "rosbag2_dynamic_recorder_interfaces/srv/unsubscribe_topics.hpp"
#include "rosbag2_interfaces/msg/write_split_event.hpp"
#include "rosbag2_interfaces/srv/is_paused.hpp"
#include "rosbag2_interfaces/srv/pause.hpp"
#include "rosbag2_interfaces/srv/snapshot.hpp"
#include "rosbag2_interfaces/srv/toggle_paused.hpp"

#if ROSBAG2_DYNAMIC_RECORDER_HAS_UPSTREAM_MESSAGES_LOST_EVENT
#include "rosbag2_interfaces/msg/messages_lost_event.hpp"
#else
#include "rosbag2_dynamic_recorder_interfaces/msg/messages_lost_event.hpp"
#endif
#if ROSBAG2_DYNAMIC_RECORDER_HAS_UPSTREAM_RECORD
#include "rosbag2_interfaces/srv/record.hpp"
#else
#include "rosbag2_dynamic_recorder_interfaces/srv/record.hpp"
#endif
#if ROSBAG2_DYNAMIC_RECORDER_HAS_UPSTREAM_RESUME
#include "rosbag2_interfaces/srv/resume.hpp"
#else
#include "rosbag2_dynamic_recorder_interfaces/srv/resume.hpp"
#endif
#if ROSBAG2_DYNAMIC_RECORDER_HAS_UPSTREAM_SPLIT_BAGFILE
#include "rosbag2_interfaces/srv/split_bagfile.hpp"
#else
#include "rosbag2_dynamic_recorder_interfaces/srv/split_bagfile.hpp"
#endif
#if ROSBAG2_DYNAMIC_RECORDER_HAS_UPSTREAM_STOP
#include "rosbag2_interfaces/srv/stop.hpp"
#else
#include "rosbag2_dynamic_recorder_interfaces/srv/stop.hpp"
#endif

namespace rosbag2_dynamic_recorder
{

namespace own_msg = rosbag2_dynamic_recorder_interfaces::msg;
namespace own_srv = rosbag2_dynamic_recorder_interfaces::srv;

using BagSizeLimitEvent = own_msg::BagSizeLimitEvent;
using DebugTimings = own_msg::DebugTimings;
using FileSplitEvent = own_msg::FileSplitEvent;
using LowDiskEvent = own_msg::LowDiskEvent;
using PauseEvent = own_msg::PauseEvent;
using Profile = own_msg::Profile;
using RecorderStatus = own_msg::RecorderStatus;
using SubscriptionChangeEvent = own_msg::SubscriptionChangeEvent;
using WriteSplitEvent = rosbag2_interfaces::msg::WriteSplitEvent;

using GetProfiles = own_srv::GetProfiles;
using GetStatus = own_srv::GetStatus;
using GetSubscribedTopics = own_srv::GetSubscribedTopics;
using SetProfile = own_srv::SetProfile;
using SetTopics = own_srv::SetTopics;
using SubscribeTopics = own_srv::SubscribeTopics;
using UnsubscribeTopics = own_srv::UnsubscribeTopics;
using IsPaused = rosbag2_interfaces::srv::IsPaused;
using Pause = rosbag2_interfaces::srv::Pause;
using Snapshot = rosbag2_interfaces::srv::Snapshot;
using TogglePaused = rosbag2_interfaces::srv::TogglePaused;

#if ROSBAG2_DYNAMIC_RECORDER_HAS_UPSTREAM_MESSAGES_LOST_EVENT
using MessagesLostEvent = rosbag2_interfaces::msg::MessagesLostEvent;
#else
using MessagesLostEvent = own_msg::MessagesLostEvent;
#endif
#if ROSBAG2_DYNAMIC_RECORDER_HAS_UPSTREAM_RECORD
using Record = rosbag2_interfaces::srv::Record;
#else
using Record = own_srv::Record;
#endif
#if ROSBAG2_DYNAMIC_RECORDER_HAS_UPSTREAM_RESUME
using Resume = rosbag2_interfaces::srv::Resume;
#else
using Resume = own_srv::Resume;
#endif
#if ROSBAG2_DYNAMIC_RECORDER_HAS_UPSTREAM_SPLIT_BAGFILE
using SplitBagfile = rosbag2_interfaces::srv::SplitBagfile;
#else
using SplitBagfile = own_srv::SplitBagfile;
#endif
#if ROSBAG2_DYNAMIC_RECORDER_HAS_UPSTREAM_STOP
using Stop = rosbag2_interfaces::srv::Stop;
#else
using Stop = own_srv::Stop;
#endif

}  // namespace rosbag2_dynamic_recorder

#endif  // ROSBAG2_DYNAMIC_RECORDER__INTERFACES_HPP_
