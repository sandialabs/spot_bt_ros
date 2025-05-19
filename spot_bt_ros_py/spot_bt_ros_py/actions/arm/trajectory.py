"""Spot arm trajectory state related actions."""

from __future__ import annotations


from py_trees.behaviour import Behaviour
from py_trees.blackboard import Client
from py_trees.common import Access
from py_trees.common import Status

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future

from spot_bt_ros_msgs.action import SpotArmTask


class ArmTrajectory(Behaviour):
    """
    Behavior action for Spot to move manipulator arm to a target.

    Attributes:
      blackboard (py_trees.blackboard.Client): Maintains BT variables associated with a
        specific blackboard client.
      mission_blackboard (py_trees.blackboard.Client): Maintains BT variables associated
        with a mission specific blackboard client.
      client (rclpy.action.ActionClient): A ROS action client to connect to the Spot
        controller.
      future (rclpy.client.Future): Stores future information for the async ROS
        client action call.
      node (rclpy.node.Node): Stores the main ROS node for the BT process. This allows
        our action to use some of the nodes functionality.
    """

    def __init__(self, name: str):
        """
        Initialize an MoveToTarget action object.

        Args:
          name (str): The action name. This will be applied to logging messages and image
            outputs of the action.
        """
        super().__init__(name)
        self.blackboard: Client = None
        self.mission_blackboard: Client = None
        self.client: ActionClient = None
        self.future: Future = None
        self.node: Node = None


    def setup(self, **kwargs):
        """Setup ROS 2 subs/pubs/clients for connection to spot_driver."""
        self.logger.debug(f"{self.qualified_name}.setup()")
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_msg = f"didn't find 'node' in setup's kwargs [{self.qualified_name}]"
            raise KeyError(error_msg) from e

    def initialise(self):
        """Initialize variables and perform action behavior for first tick."""
        self.logger.debug(f"  {self.name} [ArmTrajectory::initialise()]")
        self.blackboard = self.attach_blackboard_client("clients")
        self.blackboard.register_key(key="arm_controller", access=Access.WRITE)
        self.client = self.blackboard.arm_controller

        self.mission_blackboard = self.attach_blackboard_client("mission")
        self.mission_blackboard.register_key(key="target", access=Access.WRITE)

        # Send action client request
        goal_msg = SpotArmTask.Goal()
        goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.action = "move_arm_to_target"
        goal_msg.goal = self.mission_blackboard.target
        self.future = self.client.send_goal_async(goal_msg, feedback_callback=self._feedback)

    def update(self) -> Status:
        """Run the ArmTrajectory behavior when ticked."""
        self.logger.debug(f"  {self.name} [ArmTrajectory::update()]")

        if self.future.done():
            result = self.future.result().get_result().result
            if result.success:
                return Status.SUCCESS

            self.logger.error(f"{result.message}")
            return Status.FAILURE

        self.logger.debug(f"  {self.name} [ArmTrajectory::update()][RUNNING]")
        return Status.RUNNING

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.logger.debug(
            f"  {self.name} [ArmTrajectory::terminate()]"
            f"[{self.status}->{new_status}]"
        )

    def _feedback(self, feedback_msg: SpotArmTask.Feedback):
        """Log action feedback for ArmTrajectory."""
        self.node.get_logger().info(
            feedback_msg.feedback.feedback, throttle_duration_sec=0.1
        )
