cmake_minimum_required(VERSION 3.16)
project(rosbag2_dynamic_recorder_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SubscribeTopics.srv"
  "srv/UnsubscribeTopics.srv"
  "srv/SetTopics.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
