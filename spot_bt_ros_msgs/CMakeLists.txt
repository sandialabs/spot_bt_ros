cmake_minimum_required(VERSION 3.8)
project(spot_bt_ros_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(bosdyn_api_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PlanSpotArmTask.action"
  "action/PlanSpotBodyTask.action"
  "action/SpotArmTask.action"
  "action/SpotBodyTask.action"
  "action/SpotPerceptionTask.action"
  "msg/SpotState.msg"
  DEPENDENCIES std_msgs geometry_msgs bosdyn_api_msgs
)

ament_package()
