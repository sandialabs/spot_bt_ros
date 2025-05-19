"""Spot motion actions."""

from __future__ import annotations

from py_trees.behaviour import Behaviour
from py_trees.blackboard import Client
from py_trees.common import Access
from py_trees.common import Status

from rclpy.action import ActionClient
from rclpy.client import Client as ServiceClient
from rclpy.node import Node
from rclpy.task import Future

from std_srvs.srv import Trigger

from spot_bt_ros_msgs.action import SpotBodyTask


class MoveToTarget(Behaviour):
    """
    Move Spot to a target.

    Attributes:
      blackboard (py_trees.blackboard.Client): Maintains BT variables associated with a
        specific blackboard client.
      mission_blackboard (py_trees.blackboard.Client): Maintains BT variables associated
        with a mission specific blackboard client.
      client (rclpy.action.ActionClient): A ROS action client to connect to the Spot
        controller.
      future (rclpy.task.Future): Stores future information for the async ROS
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
        self.logger.debug(f"  {self.name} [MoveToTarget::initialise()]")
        self.blackboard = self.attach_blackboard_client("clients")
        self.blackboard.register_key(key="controller", access=Access.WRITE)
        self.client = self.blackboard.controller

        self.mission_blackboard = self.attach_blackboard_client("mission")
        self.mission_blackboard.register_key(key="target", access=Access.WRITE)

        # Send action client request
        goal_msg = SpotBodyTask.Goal()
        goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.action = "move_to_target"
        goal_msg.goal = self.mission_blackboard.target
        self.future = self.client.send_goal_async(goal_msg, feedback_callback=self._feedback)

    def update(self) -> Status:
        """Run the MoveToTarget behavior when ticked."""
        self.logger.debug(f"  {self.name} [MoveToTarget::update()]")

        if self.future.done():
            result = self.future.result().get_result().result
            if result.success:
                return Status.SUCCESS

            self.logger.error(f"{result.message}")
            return Status.FAILURE

        self.logger.debug(f"  {self.name} [MoveToTarget::update()][RUNNING]")
        return Status.RUNNING

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f"  {self.name} [MoveToTarget::terminate()]"
            f"[{self.status}->{new_status}]"
        )

    def _feedback(self, feedback_msg: SpotBodyTask.Feedback):
        """Log action feedback for MoveToTarget."""
        self.node.get_logger().info(
            feedback_msg.feedback.feedback, throttle_duration_sec=0.1
        )


class FollowPath(Behaviour):
    """
    Action for Spot to follow a given path.

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
        Initialize an FollowPath action object.

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
        """Setup ROS 2 subs/pubs/clients for connection to spot_driver."""
        self.logger.debug(f"{self.qualified_name}.setup()")
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_msg = f"didn't find 'node' in setup's kwargs [{self.qualified_name}]"
            raise KeyError(error_msg) from e

    def initialise(self):
        """Initialize variables and perform action behavior for first tick."""
        self.logger.debug(f"  {self.name} [FollowPath::initialise()]")
        self.blackboard = self.attach_blackboard_client("clients")
        self.blackboard.register_key(key="controller", access=Access.WRITE)
        self.client = self.blackboard.controller

        # Send action client request
        goal_msg = SpotBodyTask.Goal()
        goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.action = "follow"
        # TODO ADD PATH TO SEND
        self.future = self.client.send_goal_async(goal_msg, feedback_callback=self._feedback)

    def update(self) -> Status:
        """Run the FollowPath behavior when ticked."""
        self.logger.debug(f"  {self.name} [FollowPath::update()]")

        if self.future.done():
            result = self.future.result().get_result().result
            if result.success:
                return Status.SUCCESS

            self.logger.error(f"{result.message}")
            return Status.FAILURE

        self.logger.debug(f"  {self.name} [FollowPath::update()][RUNNING]")
        return Status.RUNNING

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f"  {self.name} [FollowPath::terminate()]"
            f"[{self.status}->{new_status}]"
        )

    def _feedback(self, feedback_msg: SpotBodyTask.Feedback):
        """Log action feedback for FollowPath."""
        self.node.get_logger().info(
            feedback_msg.feedback.feedback, throttle_duration_sec=0.1
        )


class Crouch(Behaviour):
    """
    Spot action behavior to crouch.

    Attributes:
      blackboard (py_trees.blackboard.Client): Maintains BT variables associated with a
        specific blackboard client.
      client (rclpy.client.Client): A ROS service client to connect to the `spot_driver`.
      future (rclpy.task.Future): Stores future information for the async ROS
        client service call.
      node (rclpy.node.Node): Stores the main ROS node for the BT process. This allows
        our action to use some of the nodes functionality.
    """

    def __init__(self, name: str):
        """
        Initialize an Crouch action object.

        Args:
          name (str): The action name. This will be applied to logging messages and image
            outputs of the action.
        """
        super().__init__(name)
        self.blackboard: Client = None
        self.client: ServiceClient = None
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

        # Create client
        self.client = self.node.create_client(Trigger, "crouch")
        self.client.wait_for_service()

    def initialise(self):
        """Initialize variables and perform action behavior for first tick."""
        self.logger.debug(f"  {self.name} [Crouch::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)

        # Send client request
        self.future = self.client.call_async(Trigger.Request())

    def update(self) -> Status:
        """Run the Crouch behavior when ticked."""
        self.logger.debug(f"  {self.name} [Crouch::update()]")

        if self.future.done():
            if self.future.result().success:
                self.blackboard.state.crouching = True
                return Status.SUCCESS

            self.logger.error(f"{self.future.result().message}")
            return Status.FAILURE

        self.logger.debug(f"  {self.name} [Crouch::update()][RUNNING]")
        return Status.RUNNING

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f"  {self.name} [Crouch::terminate()]"
            f"[{self.status}->{new_status}]"
        )


class Rollover(Behaviour):
    """
    Spot action behavior to rollover.

    Attributes:
      blackboard (py_trees.blackboard.Client): Maintains BT variables associated with a
        specific blackboard client.
      client (rclpy.client.Client): A ROS service client to connect to the `spot_driver`.
      future (rclpy.task.Future): Stores future information for the async ROS
        client service call.
      node (rclpy.node.Node): Stores the main ROS node for the BT process. This allows
        our action to use some of the nodes functionality.
    """

    def __init__(self, name: str):
        """
        Initialize an Rollover action object.

        Args:
          name (str): The action name. This will be applied to logging messages and image
            outputs of the action.
        """
        super().__init__(name)
        self.client: ServiceClient = None
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

        # Create client
        self.client = self.node.create_client(Trigger, "rollover")
        self.client.wait_for_service()

    def initialise(self):
        """Initialize variables and perform action behavior for first tick."""
        self.logger.debug(f"  {self.name} [Rollover::initialise()]")

        # Send client request
        self.future = self.client.call_async(Trigger.Request())

    def update(self) -> Status:
        """Run the Rollover behavior when ticked."""
        self.logger.debug(f"  {self.name} [Rollover::update()]")

        if self.future.done():
            if self.future.result().success:
                return Status.SUCCESS

            self.logger.error(f"{self.future.result().message}")
            return Status.FAILURE

        self.logger.debug(f"  {self.name} [Rollover::update()][RUNNING]")
        return Status.RUNNING

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f"  {self.name} [Rollover::terminate()]"
            f"[{self.status}->{new_status}]"
        )
