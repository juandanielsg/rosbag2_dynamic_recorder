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

  // One single-threaded executor per callback group, each on its own thread. The services need a
  // thread apart from the subscriptions, so that a slow operation (a channel created during a disk
  // stall) cannot block the callbacks that are writing messages. Neither group ever needs more
  // than one: both are mutually exclusive. A MultiThreadedExecutor gave them one thread per core,
  // all but two of them idle contenders for its wait set. On the benchmark's steady workload
  // (B1, 6 reps each) this cut the recorder from 28-32% of a core to 21-22%, on Fast DDS and
  // Cyclone alike; a MultiThreadedExecutor limited to 2 threads was worse than either, at 36%.
  rclcpp::executors::SingleThreadedExecutor services;
  services.add_callback_group(node->service_callback_group(), node->get_node_base_interface());
  // After the service group is claimed, so add_node() takes every group but that one.
  rclcpp::executors::SingleThreadedExecutor messages;
  messages.add_node(node);

  std::thread service_thread([&services]() {services.spin();});
  messages.spin();
  services.cancel();
  service_thread.join();

  // Explicit stop before rclcpp teardown, so the bag is closed and its metadata written even
  // though the destructor would also do it.
  node->stop();
  rclcpp::shutdown();
  return 0;
}
