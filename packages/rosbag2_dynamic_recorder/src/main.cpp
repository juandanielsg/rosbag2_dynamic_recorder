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
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_dynamic_recorder/dynamic_recorder.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rosbag2_dynamic_recorder::DynamicRecorder>();

  // One single-threaded executor per callback group, each on its own thread: a slow service (a
  // channel created during a disk stall) cannot block the callbacks writing messages. Both groups
  // are mutually exclusive, so neither could use a second thread; a MultiThreadedExecutor only
  // adds idle threads contending for its wait set, which measurably costs CPU.
  rclcpp::executors::SingleThreadedExecutor services;
  services.add_callback_group(node->service_callback_group(), node->get_node_base_interface());
  // After the service group is claimed, so add_node() takes every group but that one.
  rclcpp::executors::SingleThreadedExecutor messages;
  messages.add_node(node);

  std::thread service_thread([&services]() {services.spin();});
  messages.spin();
  services.cancel();
  service_thread.join();

  // Before rclcpp teardown, so the bag is closed and its metadata written while publishing works.
  node->stop();
  rclcpp::shutdown();
  return 0;
}
