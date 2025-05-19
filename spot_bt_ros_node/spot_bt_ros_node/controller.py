"""Stand-alone controller for Spot Body and Arm Motion."""

from __future__ import annotations

from threading import Event

import rclpy
from rclpy.action import ActionServer
from rclpy.action import ActionClient
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from spot_msgs.action import RobotCommand

from spot_bt_ros_msgs.action import SpotArmTask
from spot_bt_ros_msgs.action import SpotBodyTask

from spot_bt_ros_node.arm import SpotArmMixin
from spot_bt_ros_node.body import SpotBodyMixin
from spot_bt_ros_node.transform import SpotTFBase


class SpotMotionController(Node, SpotBodyMixin, SpotArmMixin):
    """Spot body and arm motion controller node."""

    def __init__(self, namespace: str | None = None):
        super().__init__("spot_motion_node", namespace=namespace)
        if namespace is not None:
            self._name = f"{namespace}/"
            self._ns = namespace
        else:
            self._name = ""
            self._ns = ""

        self.declare_parameters(
            "",
            [
                ("dock_id", 549),
                ("has_arm", True),
                ("docked", True),
                ("standing", False),
                ("gripper_open", False),
            ],
        )

        # Useful action variables
        self._group = ReentrantCallbackGroup()
        self._tf = SpotTFBase(node=self, namespace=namespace)
        self._dock_id = self.get_parameter("dock_id").get_parameter_value().integer_value
        self._saved_body_feedback_msg = None
        self._called_action_goal_handle: ServerGoalHandle = None
        self._action_done_event = Event()
        self._body_action_result = None

        # Create action client to Spot driver
        self._action_client = ActionClient(
            self, RobotCommand, "robot_command", callback_group=self._group
        )

        # Create action server interface to behavior tree.
        self._action_server: dict[str, ActionServer] = {
            "body": ActionServer(
                self,
                SpotBodyTask,
                "spot/motion/controller/body",
                self._execute_body_action,
                callback_group=self._group,
            ),
        }

        # Create a dict of predefined actions
        self._predefined_commands: dict[str, callable] = {
            "spin": self.build_spin_command,
            "back_up": self.build_back_up_command,
        }

        # Add arm related functionality, if applicable
        self._has_arm = self.get_parameter("has_arm").get_parameter_value().bool_value
        if self._has_arm:
            self._predefined_commands["freeze"] = self.build_arm_freeze_command
            self._predefined_commands["deploy"]= self.build_arm_deploy_command

            # Create action server interface to Spot arm behavior tree actions
            self._saved_arm_feedback_msg = None
            self._arm_action_result = None
            self._action_server["arm"] = ActionServer(
                self,
                SpotArmTask,
                "spot/motion/controller/arm",
                self._execute_arm_action,
                callback_group=self._group,
            )

    def shutdown(self):
        """Close external connections."""
        self._tf.shutdown()

    def get_time(self) -> int:
        """Get current clock time in nanoseconds."""
        return self.get_clock().now().nanoseconds

    @property
    def name(self) -> str:
        """Get node name."""
        return self.get_name()


def main(args=None):
    """Start SpotMotionController node."""
    rclpy.init(args=args)

    node = SpotMotionController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().warning(f"Killing node: {node.name}")
        executor.shutdown()
        node.shutdown()
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
