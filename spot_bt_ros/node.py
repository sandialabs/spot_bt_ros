from __future__ import annotations

from functools import partial
import logging
from typing import Any

from bdai_ros2_wrappers.action_client import ActionClientWrapper
from bdai_ros2_wrappers.utilities import fqn

from rclpy.client import Client
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import TwistWithCovarianceStamped

from nav_msgs.msg import Odometry

from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState

from spot_msgs.action import RobotCommand
from spot_msgs.msg import BatteryStateArray
from spot_msgs.msg import Metrics
from spot_msgs.msg import WiFiState
from spot_msgs.srv import Dock
from spot_msgs.srv import ListWorldObjects

from std_srvs.srv import Trigger

from spot_bt_ros.data import SpotImagesDict
from spot_bt_ros.data import SpotStatus


TRIGGER_SERVICES = [
    "claim",
    "release",
    "stop",
    "self_right",
    "sit",
    "stand",
    "power_on",
    "power_off",
    "estop/hard",
    "estop/gentle",
    "estop/release",
    "undock",
]


class SpotBT(Node):
    def __init__(self, namespace: str | None = None, enable_depth: bool = False):
        super().__init__("spot_bt", namespace=namespace)
        if namespace is not None:
            self._name = f"{namespace}/"
            self._ns = namespace
        else:
            self._name = ""
            self._ns = ""

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self.images = SpotImagesDict()
        self.state = SpotStatus()
        self._sub = {
            "metrics": self.create_subscription(
                Metrics, "status/metrics", self.metrics_callback, 1
            ),
            "wifi": self.create_subscription(
                WiFiState, "status/wifi", self.wifi_callback, 1
            ),
            "battery": self.create_subscription(
                BatteryStateArray, "status/battery_states", self.battery_callback, 1
            ),
            "joint_state": self.create_subscription(
                JointState, "joint_states", self.joint_state_callback, 1
            ),
            "odom": self.create_subscription(
                Odometry, "odometry", self.odom_callback, 1
            ),
            "odom_twist": self.create_subscription(
                TwistWithCovarianceStamped, "odometry/twist", self.odom_twist_callback, 1
            ),
            "camera": {
                "back": self.create_subscription(
                    Image,
                    "camera/back/image",
                    partial(self.rgb_camera_callback, camera="back"),
                    10
                ),
                "frontleft": self.create_subscription(
                    Image,
                    "camera/frontleft/image",
                    partial(self.rgb_camera_callback, camera="frontleft"),
                    10
                ),
                "frontright": self.create_subscription(
                    Image,
                    "camera/frontright/image",
                    partial(self.rgb_camera_callback, camera="frontright"),
                    10
                ),
                "left": self.create_subscription(
                    Image,
                    "camera/left/image",
                    partial(self.rgb_camera_callback, camera="left"),
                    10
                ),
                "right": self.create_subscription(
                    Image,
                    "camera/right/image",
                    partial(self.rgb_camera_callback, camera="right"),
                    10
                ),
            }
        }

        if enable_depth:
            self._sub["depth"] = {
                "back": self.create_subscription(
                    Image,
                    "depth/back/image",
                    partial(self.depth_camera_callback, camera="back"),
                    10
                ),
                "frontleft": self.create_subscription(
                    Image,
                    "depth/frontleft/image",
                    partial(self.depth_camera_callback, camera="frontleft"),
                    10
                ),
                "frontright": self.create_subscription(
                    Image,
                    "depth/frontright/image",
                    partial(self.depth_camera_callback, camera="frontright"),
                    10
                ),
                "left": self.create_subscription(
                    Image,
                    "depth/left/image",
                    partial(self.depth_camera_callback, camera="left"),
                    10
                ),
                "right": self.create_subscription(
                    Image,
                    "depth/right/image",
                    partial(self.depth_camera_callback, camera="right"),
                    10
                ),
            }

        self._command_map: dict[str, Client] = {}
        for service in TRIGGER_SERVICES:
            self._command_map[service] = self.create_client(Trigger, service)
            self.get_logger().info(f"Waiting for service {service}")
            self._command_map[service].wait_for_service()
            self.get_logger().info(f"Found service {service}")

        # add dock command
        self._command_map["dock"] = self.create_client(Dock, "dock")

        # add world objects command
        self._command_map["list"] = self.create_client(
            ListWorldObjects, "list_world_objects"
        )

        self.command_client = ActionClientWrapper(
            RobotCommand, "robot_command", # "spot_bt"
        )

    def command(self, command: str, **kwargs) -> Any:
        """Issue a command for Spot to do."""
        try:
            if command in TRIGGER_SERVICES:
                future = self._command_map[command].call_async(Trigger.Request())
            elif command == "dock":
                request = Dock.Request()
                request.dock_id = kwargs["dock_id"]
                future = self._command_map[command].call_async(request)
            elif command == "list":
                if kwargs.get("request") is not None:
                    request = kwargs["request"]
                else:
                    request = ListWorldObjects.Request()
                future = self._command_map[command].call_async(request)

            else:
                future = self._command_map[command].call_async(Trigger.Request())
        except KeyError:
            err = f"No command {command}"
            self.get_logger().error(err)
            return Trigger.Response(success=False, message=err)
        self._executor.spin_until_future_complete(future)
        return future.result()

    def rgb_camera_callback(self, msg: Image, camera: str):
        """Save RGB camera images from Spot."""
        self.images.rgb[camera] = msg

    def depth_camera_callback(self, msg: Image, camera: str):
        """Save depth camera images from Spot."""
        self.images.depth[camera] = msg

    def metrics_callback(self, msg: Metrics):
        """Set status for metric state."""
        self.state.metrics = msg

    def wifi_callback(self, msg: WiFiState):
        """Set status for wifi state."""
        self.state.wifi = msg

    def battery_callback(self, msg: BatteryStateArray):
        """Set status for battery state."""
        self.state.battery = msg

    def joint_state_callback(self, msg: JointState):
        """Set status for joint state callback."""
        self.state.joint_state = msg

    def odom_callback(self, msg: Odometry):
        """Set status for odometry state callback."""
        self.state.odom = msg

    def odom_twist_callback(self, msg: TwistWithCovarianceStamped):
        """Set status for odometry twist state callback."""
        self.state.odom_twist = msg

    def shutdown(self):
        """Close external connections."""


class SpotBTNodeless:
    def __init__(self, node: Node, namespace: str | None = None, enable_depth: bool = False):
        self._logger = logging.getLogger(fqn(self.__class__))
        if namespace is not None:
            self._name = f"{namespace}/"
            self._ns = namespace
        else:
            self._name = ""
            self._ns = ""

        self.images = SpotImagesDict()
        self.state = SpotStatus()
        self._sub = {
            "metrics": node.create_subscription(
                Metrics, "status/metrics", self.metrics_callback, 1
            ),
            "wifi": node.create_subscription(
                WiFiState, "status/wifi", self.wifi_callback, 1
            ),
            "battery": node.create_subscription(
                BatteryStateArray, "status/battery_states", self.battery_callback, 1
            ),
            "joint_state": node.create_subscription(
                JointState, "joint_states", self.joint_state_callback, 1
            ),
            "odom": node.create_subscription(
                Odometry, "odometry", self.odom_callback, 1
            ),
            "odom_twist": node.create_subscription(
                TwistWithCovarianceStamped, "odometry/twist", self.odom_twist_callback, 1
            ),
            "camera": {
                "back": node.create_subscription(
                    Image,
                    "camera/back/image",
                    partial(self.rgb_camera_callback, camera="back"),
                    10
                ),
                "frontleft": node.create_subscription(
                    Image,
                    "camera/frontleft/image",
                    partial(self.rgb_camera_callback, camera="frontleft"),
                    10
                ),
                "frontright": node.create_subscription(
                    Image,
                    "camera/frontright/image",
                    partial(self.rgb_camera_callback, camera="frontright"),
                    10
                ),
                "left": node.create_subscription(
                    Image,
                    "camera/left/image",
                    partial(self.rgb_camera_callback, camera="left"),
                    10
                ),
                "right": node.create_subscription(
                    Image,
                    "camera/right/image",
                    partial(self.rgb_camera_callback, camera="right"),
                    10
                ),
            }
        }

        if enable_depth:
            self._sub["depth"] = {
                "back": node.create_subscription(
                    Image,
                    "depth/back/image",
                    partial(self.depth_camera_callback, camera="back"),
                    10
                ),
                "frontleft": node.create_subscription(
                    Image,
                    "depth/frontleft/image",
                    partial(self.depth_camera_callback, camera="frontleft"),
                    10
                ),
                "frontright": node.create_subscription(
                    Image,
                    "depth/frontright/image",
                    partial(self.depth_camera_callback, camera="frontright"),
                    10
                ),
                "left": node.create_subscription(
                    Image,
                    "depth/left/image",
                    partial(self.depth_camera_callback, camera="left"),
                    10
                ),
                "right": node.create_subscription(
                    Image,
                    "depth/right/image",
                    partial(self.depth_camera_callback, camera="right"),
                    10
                ),
            }

        self._command_map: dict[str, Client] = {}
        for service in TRIGGER_SERVICES:
            self._command_map[service] = node.create_client(Trigger, service)
            self._logger.info("Waiting for service %s", service)
            self._command_map[service].wait_for_service()
            self._logger.info("Found service %s", service)

        # add dock command
        self._command_map["dock"] = node.create_client(Dock, "dock")

        # add world objects command
        self._command_map["list"] = node.create_client(
            ListWorldObjects, "list_world_objects"
        )

        self.command_client = ActionClientWrapper(
            RobotCommand, "robot_command", node,
        )

    def command(self, command: str, **kwargs) -> Any:
        """Issue a command for Spot to do."""
        try:
            if command in TRIGGER_SERVICES:
                return self._command_map[command].call(Trigger.Request())

            if command == "dock":
                request = Dock.Request()
                request.dock_id = kwargs["dock_id"]
                return self._command_map[command].call(request)

            if command == "list":
                if kwargs.get("request") is not None:
                    request = kwargs["request"]
                else:
                    request = ListWorldObjects.Request()
                return self._command_map[command].call(request)

        except KeyError:
            err = f"No command {command}"
            self._logger.error(err)
            return Trigger.Response(success=False, message=err)

    def rgb_camera_callback(self, msg: Image, camera: str):
        """Save RGB camera images from Spot."""
        self.images.rgb[camera] = msg

    def depth_camera_callback(self, msg: Image, camera: str):
        """Save depth camera images from Spot."""
        self.images.depth[camera] = msg

    def metrics_callback(self, msg: Metrics):
        """Set status for metric state."""
        self.state.metrics = msg

    def wifi_callback(self, msg: WiFiState):
        """Set status for wifi state."""
        self.state.wifi = msg

    def battery_callback(self, msg: BatteryStateArray):
        """Set status for battery state."""
        self.state.battery = msg

    def joint_state_callback(self, msg: JointState):
        """Set status for joint state callback."""
        self.state.joint_state = msg

    def odom_callback(self, msg: Odometry):
        """Set status for odometry state callback."""
        self.state.odom = msg

    def odom_twist_callback(self, msg: TwistWithCovarianceStamped):
        """Set status for odometry twist state callback."""
        self.state.odom_twist = msg

    def shutdown(self):
        """Close external connections."""
