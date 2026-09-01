cmake_minimum_required(VERSION 3.16)
project(rosbag2_dynamic_recorder)

# NOTE: current Rolling has removed ament_target_dependencies(); use the exported namespaced
# targets directly, as rosbag2_examples_cpp does.

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_interfaces REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(rosbag2_dynamic_recorder_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/dynamic_recorder.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  rosbag2_transport::rosbag2_transport
  ${rosbag2_interfaces_TARGETS}
  ${rosbag2_dynamic_recorder_interfaces_TARGETS}
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  ament_cmake_ros_core::ament_ros_defaults
  rmw::rmw
)

add_executable(dynamic_recorder src/main.cpp)
target_link_libraries(dynamic_recorder PRIVATE
  ament_cmake_ros_core::ament_ros_defaults
  ${PROJECT_NAME}
  rclcpp::rclcpp
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS dynamic_recorder
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  rosbag2_interfaces
  rosbag2_transport
  rosbag2_dynamic_recorder_interfaces
)

ament_package()
