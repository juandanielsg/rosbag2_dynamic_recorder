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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_dynamic_recorder/dynamic_recorder.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rosbag2_dynamic_recorder::DynamicRecorder>();

  // MultiThreadedExecutor is required, not merely preferred: services run in their own callback
  // group so that adding a topic (~0.5s, dominated by message definition resolution) cannot block
  // the subscription callbacks that are writing messages.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  // Explicit stop before rclcpp teardown, so the bag is closed and its metadata written even
  // though the destructor would also do it.
  node->stop();
  rclcpp::shutdown();
  return 0;
}
