cmake_minimum_required(VERSION 3.5)
project(openzen_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

option(OPENZEN_ROS_BINARY_LIBRARIES "Download binary dependencies" OFF)

SET(ZEN_BLUETOOTH OFF CACHE BOOL "Don't build bluetooth")
SET(ZEN_TESTS OFF CACHE BOOL "Don't build tests")
SET(ZEN_EXAMPLES OFF CACHE BOOL "Don't build examples")
SET(ZEN_USE_BINARY_LIBRARIES ${OPENZEN_ROS_BINARY_LIBRARIES} CACHE BOOL "Don't download binary dependencies (windows only)")


if($ENV{ROS_DISTRO} STREQUAL "foxy")
  add_subdirectory(openzen)
endif()


## Build 
add_executable(openzen_node
  src/OpenZenNode.cpp
)
target_link_libraries(openzen_node
  OpenZen
)

ament_target_dependencies(openzen_node
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs)

install(TARGETS openzen_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

if (BUILD_TESTING)
  # launch_test is documented here:
  # https://github.com/ros2/launch/tree/master/launch_testing
  find_package(launch_testing_ament_cmake)
  add_launch_test(test/openzen_test.launch.py)
endif()

install(
  DIRECTORY script
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/script/ld_library_path_hook.sh.in")

ament_package()

