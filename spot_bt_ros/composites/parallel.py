"""spot_bt_ros Parallel composites."""
from __future__ import annotations

import py_trees

import py_trees_ros

import rclpy

from sensor_msgs.msg import Image

from spot_msgs.msg import (
    BatteryStateArray,
    Metrics,
    WiFiState,
)

from spot_bt_ros.behaviors.actions.perception import (
    create_rgb_camera_children,
    create_depth_camera_children,
)
from spot_bt_ros.cameras import (
    ROS_DEPTH_CAMERA_TOPICS,
    ROS_DEPTH_CAMERA_TOPICS_WITH_ARM,
    ROS_IMAGE_CAMERA_TOPICS,
    ROS_IMAGE_CAMERA_TOPICS_WITH_ARM
)


def create_spot_status_parallel(
    name: str = "Get Spot Status", memory: bool = False
) -> py_trees.composites.Parallel:
    """Create a parallel for getting Spot status information."""
    parallel = py_trees.composites.Parallel(name, memory=memory)
    parallel.add_children([
        py_trees_ros.subscribers.ToBlackboard(
            name="get_state_metrics",
            topic_name="status/metrics",
            topic_type=Metrics,
            qos_profile=rclpy.qos.ReliabilityPolicy,
            blackboard_variables={"status/metrics": None},
        ),
        py_trees_ros.subscribers.ToBlackboard(
            name="get_battery_metrics",
            topic_name="status/battery_states",
            topic_type=BatteryStateArray,
            qos_profile=rclpy.qos.ReliabilityPolicy,
            blackboard_variables={"status/battery": None},
        ),
        py_trees_ros.subscribers.ToBlackboard(
            name="get_wifi_metrics",
            topic_name="status/wifi",
            topic_type=WiFiState,
            qos_profile=rclpy.qos.ReliabilityPolicy,
            blackboard_variables={"status/wifi": None},
        ),
    ])

    return parallel


def create_rgb_camera_parallel(
    name: str = "Get RGB Camera Images", memory: bool = False, has_arm: bool = True
) -> py_trees.composites.Parallel:
    """Create a parallel for getting all images around Spot to the Blackboard."""
    parallel = py_trees.composites.Parallel(name, memory=memory)
    parallel.add_children(create_rgb_camera_children(has_arm))

    return parallel


def create_depth_camera_parallel(
    name: str = "Get Depth Camera Images", memory: bool = False, has_arm: bool = True
) -> py_trees.composites.Parallel:
    """Create a parallel for getting all images around Spot to the Blackboard."""
    parallel = py_trees.composites.Parallel(name, memory=memory)
    parallel.add_children(create_depth_camera_children(has_arm))

    return parallel
