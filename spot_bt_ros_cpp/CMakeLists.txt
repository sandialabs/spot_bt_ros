cmake_minimum_required(VERSION 3.8)
project(spot_bt_ros_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(bosdyn_api_msgs REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(spot_msgs REQUIRED)
find_package(spot_bt_ros_msgs REQUIRED)

set(BASE_PLUGIN_DEPS
  behaviortree_ros2
  btcpp_ros2_interfaces
)

set(TRIGGER_PLUGIN_DEPS
  behaviortree_ros2
  btcpp_ros2_interfaces
  std_srvs
)

set(SPOT_BT_ROS_PLUGIN_DEPS
  behaviortree_ros2
  btcpp_ros2_interfaces
  spot_bt_ros_msgs
)

include_directories(include)

#################################################
# Build actions/general plugins

# Claim Lease
add_library(claim_lease_plugin SHARED src/actions/general/claim_lease.cpp)
target_compile_definitions(claim_lease_plugin PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(claim_lease_plugin ${TRIGGER_PLUGIN_DEPS})

# Dock
add_library(dock_plugin SHARED src/actions/general/dock.cpp)
target_compile_definitions(dock_plugin PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(dock_plugin ${BASE_PLUGIN_DEPS} spot_msgs)

# EStop Gentle
add_library(estop_gentle_plugin SHARED src/actions/general/estop_gentle.cpp)
target_compile_definitions(estop_gentle_plugin PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(estop_gentle_plugin ${TRIGGER_PLUGIN_DEPS})

# EStop Hard
add_library(estop_hard_plugin SHARED src/actions/general/estop_hard.cpp)
target_compile_definitions(estop_hard_plugin PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(estop_hard_plugin ${TRIGGER_PLUGIN_DEPS})

# EStop Release
add_library(estop_release_plugin SHARED src/actions/general/estop_release.cpp)
target_compile_definitions(estop_release_plugin PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(estop_release_plugin ${TRIGGER_PLUGIN_DEPS})

# Power Off
add_library(power_off_plugin SHARED src/actions/general/power_off.cpp)
target_compile_definitions(power_off_plugin PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(power_off_plugin ${TRIGGER_PLUGIN_DEPS})

# Power On
add_library(power_on_plugin SHARED src/actions/general/power_on.cpp)
target_compile_definitions(power_on_plugin PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(power_on_plugin ${TRIGGER_PLUGIN_DEPS})

# Release Lease
add_library(release_lease_plugin SHARED src/actions/general/release_lease.cpp)
target_compile_definitions(release_lease_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(release_lease_plugin ${TRIGGER_PLUGIN_DEPS})

# Self Right
add_library(self_right_plugin SHARED src/actions/general/self_right.cpp)
target_compile_definitions(self_right_plugin PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(self_right_plugin ${TRIGGER_PLUGIN_DEPS})

# Stop
add_library(stop_plugin SHARED src/actions/general/stop.cpp)
target_compile_definitions(stop_plugin PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(stop_plugin ${TRIGGER_PLUGIN_DEPS})

# Undock
add_library(undock_plugin SHARED src/actions/general/undock.cpp)
target_compile_definitions(undock_plugin PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(undock_plugin ${TRIGGER_PLUGIN_DEPS})

#################################################
# Build actions/movement plugins

# Crouch
add_library(crouch_plugin SHARED src/actions/movement/crouch.cpp)
target_compile_definitions(crouch_plugin PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(crouch_plugin ${TRIGGER_PLUGIN_DEPS})

# MoveToTarget
add_library(move_to_target_plugin SHARED src/actions/movement/move_to_target.cpp)
target_compile_definitions(move_to_target_plugin PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(move_to_target_plugin ${SPOT_BT_ROS_PLUGIN_DEPS})

# Rollover (single-file)
add_library(rollover_plugin SHARED src/actions/movement/rollover.cpp)
target_compile_definitions(rollover_plugin PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(rollover_plugin ${TRIGGER_PLUGIN_DEPS})

# Sit
add_library(sit_plugin SHARED src/actions/movement/sit.cpp)
target_compile_definitions(sit_plugin PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(sit_plugin ${TRIGGER_PLUGIN_DEPS})

# Stand
add_library(stand_plugin SHARED src/actions/movement/stand.cpp)
target_compile_definitions(stand_plugin PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(stand_plugin ${TRIGGER_PLUGIN_DEPS})

#################################################
# Build actions/perception plugins

# DetectFiducialMarkers
add_library(detect_fiducial_markers_plugin SHARED src/actions/perception/detect_fiducial_markers.cpp)
target_compile_definitions(detect_fiducial_markers_plugin PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(detect_fiducial_markers_plugin ${BASE_PLUGIN_DEPS} spot_msgs bosdyn_api_msgs)

# DetectWorldObjects
add_library(detect_world_objects_plugin SHARED src/actions/perception/detect_world_objects.cpp)
target_compile_definitions(detect_world_objects_plugin PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(detect_world_objects_plugin ${BASE_PLUGIN_DEPS} spot_msgs bosdyn_api_msgs)

#################################################
# Build actions/planner plugins

# ComputeNewWaypoint
add_library(compute_new_waypoint_plugin SHARED src/actions/planner/compute_new_waypoint.cpp)
target_compile_definitions(compute_new_waypoint_plugin PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(compute_new_waypoint_plugin ${SPOT_BT_ROS_PLUGIN_DEPS} geometry_msgs)

# ComputePathToFiducial
add_library(compute_path_to_fiducial_plugin SHARED src/actions/planner/compute_path_to_fiducial.cpp)
target_compile_definitions(compute_path_to_fiducial_plugin PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(compute_path_to_fiducial_plugin ${SPOT_BT_ROS_PLUGIN_DEPS} geometry_msgs bosdyn_api_msgs)

# ComputePathToPose
add_library(compute_path_to_pose_plugin SHARED src/actions/planner/compute_path_to_pose.cpp)
target_compile_definitions(compute_path_to_pose_plugin PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(compute_path_to_pose_plugin ${SPOT_BT_ROS_PLUGIN_DEPS} geometry_msgs)

#################################################
# Build actions/arm plugins

# ArmCarry
add_library(arm_carry_plugin SHARED src/actions/arm/arm_carry.cpp)
target_compile_definitions(arm_carry_plugin PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(arm_carry_plugin ${TRIGGER_PLUGIN_DEPS})

# ArmFreeze
add_library(arm_freeze_plugin SHARED src/actions/arm/arm_freeze.cpp)
target_compile_definitions(arm_freeze_plugin PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(arm_freeze_plugin ${SPOT_BT_ROS_PLUGIN_DEPS})

# ArmStow
add_library(arm_stow_plugin SHARED src/actions/arm/arm_stow.cpp)
target_compile_definitions(arm_stow_plugin PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(arm_stow_plugin ${TRIGGER_PLUGIN_DEPS})

# ArmUnstow
add_library(arm_unstow_plugin SHARED src/actions/arm/arm_unstow.cpp)
target_compile_definitions(arm_unstow_plugin PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(arm_unstow_plugin ${TRIGGER_PLUGIN_DEPS})

# GripperClose
add_library(gripper_close_plugin SHARED src/actions/arm/gripper_close.cpp)
target_compile_definitions(gripper_close_plugin PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(gripper_close_plugin ${TRIGGER_PLUGIN_DEPS})

# GripperOpen
add_library(gripper_open_plugin SHARED src/actions/arm/gripper_open.cpp)
target_compile_definitions(gripper_open_plugin PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(gripper_open_plugin ${TRIGGER_PLUGIN_DEPS})

#################################################
# Install

install(TARGETS
  claim_lease_plugin
  dock_plugin
  estop_gentle_plugin
  estop_hard_plugin
  estop_release_plugin
  power_off_plugin
  power_on_plugin
  release_lease_plugin
  self_right_plugin
  stop_plugin
  undock_plugin
  crouch_plugin
  move_to_target_plugin
  rollover_plugin
  sit_plugin
  stand_plugin
  detect_fiducial_markers_plugin
  detect_world_objects_plugin
  compute_new_waypoint_plugin
  compute_path_to_fiducial_plugin
  compute_path_to_pose_plugin
  arm_carry_plugin
  arm_freeze_plugin
  arm_stow_plugin
  arm_unstow_plugin
  gripper_close_plugin
  gripper_open_plugin
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
