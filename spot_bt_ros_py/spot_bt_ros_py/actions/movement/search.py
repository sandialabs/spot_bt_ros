"""Spot search-related actions."""

from __future__ import annotations

from py_trees.behaviour import Behaviour
from py_trees.blackboard import Client
from py_trees.common import Access
from py_trees.common import Status

from rclpy.action import ActionClient
from rclpy.client import Future
from rclpy.node import Node

from spot_bt_ros_msgs.action import SpotBodyTask


class ExecuteSearch(Behaviour):
    """
    Execute Search behavior for Spot.

    Attributes:
      blackboard (py_trees.blackboard.Client): Maintains BT variables associated with a
        specific blackboard client.
      client (rclpy.action.ActionClient): A ROS action client to connect to the Spot
        controller.
      future (rclpy.task.Future): Stores future information for the async ROS
        client action call.
      node (rclpy.node.Node): Stores the main ROS node for the BT process. This allows
        our action to use some of the nodes functionality.
    """

    def __init__(self, name: str):
        """
        Initialize an ExecuteSearch action object.

        Args:
          name (str): The action name. This will be applied to logging messages and image
            outputs of the action.
        """
        super().__init__(name)
        self.blackboard: Client = None
        self.client: ActionClient = None
        self.future: Future = None
        self.node: Node = None

    def setup(self, **kwargs):
        """Setup ExecuteSearch behavior before initialization."""
        self.logger.debug(f"{self.name} [ExecuteSearch::setup()]")
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_msg = f"didn't find 'node' in setup's kwargs [{self.qualified_name}]"
            raise KeyError(error_msg) from e

    def initialise(self):
        """Initialize variables and perform action behavior for first tick."""
        self.logger.debug(f"  {self.name} [ExecuteSearch::initialise()]")
        self.blackboard = self.attach_blackboard_client("clients")
        self.blackboard.register_key(key="controller", access=Access.WRITE)
        self.client = self.blackboard.controller

        # Send action client request
        goal_msg = SpotBodyTask.Goal()
        goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.action = "search"
        self.future = self.client.send_goal_async(goal_msg, feedback_callback=self._feedback)

    def update(self) -> Status:
        """Run the ExecuteSearch behavior when ticked."""
        self.logger.debug(f"  {self.name} [ExecuteSearch::update()]")

        if self.future.done():
            result = self.future.result().get_result().result
            if result.success:
                return Status.SUCCESS

            self.logger.error(f"{result.message}")
            return Status.FAILURE

        self.logger.debug(f"  {self.name} [ExecuteSearch::update()][RUNNING]")
        return Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f"  {self.name} [ExecuteSearch::terminate()]"
            f"[{self.status}->{new_status}]"
        )

    def _feedback(self, feedback_msg: SpotBodyTask.Feedback):
        """Log action feedback for ExecuteSearch."""
        self.node.get_logger().info(
            feedback_msg.feedback.feedback, throttle_duration_sec=0.1
        )
