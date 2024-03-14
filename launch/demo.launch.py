"""Example ROS 2 Launch file for spot_bt_ros."""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def get_demo_type_node(context: LaunchContext) -> list[Node]:
    """Get demo type from launch configuration"""
    demo_type = LaunchConfiguration("demo_type").perform(context)
    if demo_type.lower() not in ["arm", "fiducial", "pose"]:
        raise ValueError("The only demo_types supported are: arm, fiducial, pose")

    return [
        Node(
            package="spot_bt_ros",
            executable=f"spot_{demo_type}_demo",
            output="screen",
        )
    ]


def generate_launch_description():
    config_file = LaunchConfiguration("config_file", default="")
    has_arm = LaunchConfiguration("has_arm", default="True")

    # Spot Driver launch file.
    spot_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("spot_driver"), "launch"),
            "/spot_driver.launch.py"
        ]),
        launch_arguments={"config_file": config_file, "has_arm": has_arm}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                description="Path to configuration file for the driver.",
                default_value="",
            ),
            DeclareLaunchArgument(
                "has_arm",
                description="Whether spot has an arm.",
                default_value="True",
            ),
            DeclareLaunchArgument(
                "demo_type",
                description="Available spot_bt_ros demos to try.",
                choices=["arm", "fiducial", "pose"],
                default_value="arm",
            ),
            spot_driver_launch,
            OpaqueFunction(function=get_demo_type_node),
        ]
    )