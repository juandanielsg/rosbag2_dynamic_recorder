cmake_minimum_required(VERSION 3.16)
project(rosbag2_dynamic_recorder_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SubscriptionChangeEvent.msg"
  "msg/PauseEvent.msg"
  "msg/RecorderStatus.msg"
  "msg/Profile.msg"
  "srv/SubscribeTopics.srv"
  "srv/UnsubscribeTopics.srv"
  "srv/SetTopics.srv"
  "srv/GetStatus.srv"
  "srv/GetSubscribedTopics.srv"
  "srv/SetProfile.srv"
  "srv/GetProfiles.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
