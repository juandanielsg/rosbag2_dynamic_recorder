cmake_minimum_required(VERSION 3.16)
project(rosbag2_dynamic_recorder)

# Builds against Jazzy, Kilted and Rolling from one source tree. The rosbag2 API this package
# uses is the same on all three except for the two points probed below, and the probes look at
# the installed headers rather than at package versions, because a version number says when a
# package was cut, not what it contains.
#
# NOTE: Rolling has removed ament_target_dependencies(); use the exported namespaced targets
# directly, as rosbag2_examples_cpp does. Every distro this builds on exports them.

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_interfaces REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(rosbag2_dynamic_recorder_interfaces REQUIRED)

# What ament_cmake_ros_core::ament_ros_defaults sets on Rolling, written out. Rolling's rosbag2
# links that target, but Jazzy does not ship the package and Kilted ships it without the target,
# so depending on it was the one line that stopped both from configuring.
add_library(ros_defaults INTERFACE)
target_compile_features(ros_defaults INTERFACE cxx_std_17)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(ros_defaults INTERFACE -Wall -Wextra -Wpedantic)
endif()

# Set <out> to TRUE when <header>, found through <target>'s include directories, contains
# <token>. With an empty token it is a plain existence check.
function(dynrec_header_has out target header token)
  get_target_property(dirs ${target} INTERFACE_INCLUDE_DIRECTORIES)
  set(${out} FALSE PARENT_SCOPE)
  foreach(dir IN LISTS dirs)
    if(EXISTS "${dir}/${header}")
      if(token STREQUAL "")
        set(${out} TRUE PARENT_SCOPE)
      else()
        file(READ "${dir}/${header}" contents)
        string(FIND "${contents}" "${token}" pos)
        if(NOT pos EQUAL -1)
          set(${out} TRUE PARENT_SCOPE)
        endif()
      endif()
      return()
    endif()
  endforeach()
endfunction()

# Whether the writer reports its own losses (a full cache, a failed storage write). Rolling only;
# on Jazzy and Kilted those losses go uncounted and messages_lost_in_recorder stays 0.
dynrec_header_has(HAS_WRITER_MESSAGES_LOST rosbag2_cpp::rosbag2_cpp
  rosbag2_cpp/bag_events.hpp messages_lost_callback)
# Whether StorageOptions has a time bound on the writer cache. Rolling only.
dynrec_header_has(HAS_MAX_CACHE_DURATION rosbag2_storage::rosbag2_storage
  rosbag2_storage/storage_options.hpp max_cache_duration)
# Whether a subscription can be told to drop callbacks already queued for it. rclcpp 33 (Rolling)
# only; older rclcpp gets the same guarantee from a flag the callback checks itself.
dynrec_header_has(HAS_DISABLE_CALLBACKS rclcpp::rclcpp
  rclcpp/generic_subscription.hpp disable_callbacks)

# Which of the stock rosbag2_interfaces types this recorder can offer its services and events
# under. Each is used where the installed definition is field-identical to the one this project
# was written against (rosbag2 0.34), so a client written for stock rosbag2 can drive it, and is
# replaced by the copy in rosbag2_dynamic_recorder_interfaces where it is not: Jazzy and Kilted
# have the pre-scheduling shapes of Record, Resume and SplitBagfile, a Stop with no return code,
# and no MessagesLostEvent. dynrec.services makes the same choice from the same evidence.
set(_ifaces rosbag2_interfaces::rosbag2_interfaces__rosidl_generator_cpp)
dynrec_header_has(HAS_UPSTREAM_MESSAGES_LOST_EVENT ${_ifaces}
  rosbag2_interfaces/msg/messages_lost_event.hpp "")
dynrec_header_has(HAS_UPSTREAM_RECORD ${_ifaces}
  rosbag2_interfaces/srv/detail/record__struct.hpp start_time)
dynrec_header_has(HAS_UPSTREAM_RESUME ${_ifaces}
  rosbag2_interfaces/srv/detail/resume__struct.hpp resume_mode)
dynrec_header_has(HAS_UPSTREAM_SPLIT_BAGFILE ${_ifaces}
  rosbag2_interfaces/srv/detail/split_bagfile__struct.hpp split_mode)
dynrec_header_has(HAS_UPSTREAM_STOP ${_ifaces}
  rosbag2_interfaces/srv/detail/stop__struct.hpp return_code)

set(_features
  HAS_WRITER_MESSAGES_LOST HAS_MAX_CACHE_DURATION HAS_DISABLE_CALLBACKS
  HAS_UPSTREAM_MESSAGES_LOST_EVENT HAS_UPSTREAM_RECORD HAS_UPSTREAM_RESUME
  HAS_UPSTREAM_SPLIT_BAGFILE HAS_UPSTREAM_STOP)
set(_feature_defs "")
set(_feature_report "")
foreach(feature IN LISTS _features)
  if(${feature})
    list(APPEND _feature_defs "ROSBAG2_DYNAMIC_RECORDER_${feature}=1")
  else()
    list(APPEND _feature_defs "ROSBAG2_DYNAMIC_RECORDER_${feature}=0")
  endif()
  string(APPEND _feature_report " ${feature}=${${feature}}")
endforeach()
message(STATUS "rosbag2 features:${_feature_report}")

add_library(${PROJECT_NAME} SHARED
  src/dynamic_recorder.cpp
  src/recorder_config.cpp
  src/storage_guard.cpp
  src/loss_accounting.cpp
  src/scheduler.cpp
  src/bag.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# PUBLIC: the header includes one or the other definition of each service, so anything that
# includes it has to agree.
target_compile_definitions(${PROJECT_NAME} PUBLIC ${_feature_defs})
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  rosbag2_transport::rosbag2_transport
  ${rosbag2_interfaces_TARGETS}
  ${rosbag2_dynamic_recorder_interfaces_TARGETS}
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  ros_defaults
  rmw::rmw
)

add_executable(dynamic_recorder src/main.cpp)
target_link_libraries(dynamic_recorder PRIVATE
  ros_defaults
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
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Unit tests for the parts that stand on their own; the node itself is covered end to end below.
  find_package(ament_cmake_gtest REQUIRED)
  foreach(unit recorder_config storage_guard loss_accounting scheduler)
    ament_add_gtest(test_${unit} test/test_${unit}.cpp)
    target_link_libraries(test_${unit} ${PROJECT_NAME})
  endforeach()

  # Integration tests: they start a real recorder process and drive its services, because the
  # contract worth protecting is the service behaviour, not the internals.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_recorder_services test/test_recorder_services.py
    TIMEOUT 600
    ENV RCUTILS_LOGGING_BUFFERED_STREAM=1
  )
endif()

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
