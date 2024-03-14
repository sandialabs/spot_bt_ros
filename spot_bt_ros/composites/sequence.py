"""spot_bt_ros Sequence composites."""
from __future__ import annotations

import py_trees

import py_trees_ros

import rclpy

from spot_msgs.msg import (
    BatteryStateArray,
    Metrics,
    WiFiState,
)

from spot_bt_ros.behaviors.actions.general import (
    ClaimLease,
    RobotDock,
    RobotPowerOff,
    RobotPowerOn,
    RobotUndock,
)
from spot_bt_ros.behaviors.actions.perception import (
    create_rgb_camera_children,
    create_depth_camera_children,
    DetectFiducialMarkers,
)
from spot_bt_ros.behaviors.conditions.perception import IsAnyFiducialMarkerDetected


def create_dock_sequence(name: str = "Dock and Turn Off", memory: bool = True) -> py_trees.composites.Sequence:
    """Create a generic Dock Sequence for Spot."""

    sequence = py_trees.composites.Sequence(name, memory=memory)
    sequence.add_children(
        [
            RobotDock(name="Dock Spot"),
            RobotPowerOff(name="Power OFF Spot"),
        ]
    )
    return sequence


def create_exploration_sequence(name: str = "Explore", memory: bool = True) -> py_trees.composites.Sequence:
    """Create an exploration sequence for Spot."""

    sequence = py_trees.composites.Sequence(name, memory=memory)
    sequence.add_children(
        [
            DetectFiducialMarkers(name="Detect Fiducial Markers"),
            IsAnyFiducialMarkerDetected(name="Is Fiducial Detected?"),
        ]
    )
    return sequence


def create_find_dock_sequence(name: str = "Find Dock", memory: bool = True) -> py_trees.composites.Sequence:
    """Create a dock finding sequence for Spot."""

    sequence = py_trees.composites.Sequence(name, memory=memory)
    sequence.add_children([
            DetectFiducialMarkers(name="Detect")
        ]
    )
    return sequence


def create_move_to_target_sequence(name: str = "Explore", memory: bool = True) -> py_trees.composites.Sequence:
    """Create an exploration sequence for Spot."""

    sequence = py_trees.composites.Sequence(name, memory=memory)
    sequence.add_children(
        [
            DetectFiducialMarkers(name="Detect Fiducial Markers"),
            IsAnyFiducialMarkerDetected(name="Is Fiducial Detected?"),
        ]
    )
    return sequence


def create_undock_sequence(name: str = "Turn On and Undock", memory: bool = True) -> py_trees.composites.Sequence:

    """Create a generic Undock Sequence for Spot."""
    sequence = py_trees.composites.Sequence(name, memory=memory)
    sequence.add_children(
        [
            ClaimLease(name="Claim Lease"),
            RobotPowerOn(name="Power ON Spot"),
            RobotUndock(name="Undock Spot"),
        ]
    )

    return sequence


def create_spot_status_sequence(
    name: str = "Get Spot Status", memory: bool = False
) -> py_trees.composites.Sequence:
    """Create a sequence for getting Spot status information."""
    sequence = py_trees.composites.Sequence(name, memory=memory)
    sequence.add_children([
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

    return sequence


def create_rgb_camera_sequence(
    name: str = "Get RGB Camera Images", memory: bool = False, has_arm: bool = True
) -> py_trees.composites.Sequence:
    """Create a sequence for getting all images around Spot to the Blackboard."""
    sequence = py_trees.composites.Sequence(name, memory=memory)
    sequence.add_children(create_rgb_camera_children(has_arm))

    return sequence


def create_depth_camera_sequence(
    name: str = "Get Depth Camera Images", memory: bool = False, has_arm: bool = True
) -> py_trees.composites.Sequence:
    """Create a sequence for getting all images around Spot to the Blackboard."""
    sequence = py_trees.composites.Sequence(name, memory=memory)
    sequence.add_children(create_depth_camera_children(has_arm))

    return sequence
