"""spot_bt_ros_node movement controller launch file."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for spot_bt_ros_node movement controller."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dock_id",
                description="Fiducial marker ID for the dock.",
                default_value="549",
            ),
            DeclareLaunchArgument(
                "has_arm",
                description="Whether Spot has an arm.",
                default_value="True",
            ),
            DeclareLaunchArgument(
                "docked",
                description="Dock status for Spot.",
                default_value="True",
            ),
            DeclareLaunchArgument(
                "standing",
                description="Whether Spot is standing or not.",
                default_value="False",
            ),
            DeclareLaunchArgument(
                "gripper_open",
                description="State of gripper open if Spot has an arm.",
                default_value="False",
            ),
            Node(
                package="spot_bt_ros_node",
                executable="spot_controller_node",
                output="screen",
                parameters=[
                    {
                        "dock_id": LaunchConfiguration("dock_id"),
                        "has_arm": LaunchConfiguration("has_arm"),
                        "docked": LaunchConfiguration("docked"),
                        "standing": LaunchConfiguration("standing"),
                        "gripper_open": LaunchConfiguration("gripper_open"),
                    }
                ]
            ),
        ]
    )
