"""Robot Pose related actions"""

from __future__ import annotations

from py_trees.behaviour import Behaviour
from py_trees.blackboard import Client
from py_trees.common import Access
from py_trees.common import Status

from rclpy.client import Client as ServiceClient
from rclpy.node import Node
from rclpy.task import Future

from spot_bt_ros_msgs.action import SpotBodyTask

from std_srvs.srv import Trigger


class Sit(Behaviour):
    """
    Behavior class for getting Spot to sit.

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
        Initialize an Sit action object.

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
        self.client = self.node.create_client(Trigger, "sit")
        self.client.wait_for_service()

    def initialise(self):
        """Initialize variables and perform action behavior for first tick."""
        self.logger.debug(f"  {self.name} [Sit::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)

        # Send client request
        self.future = self.client.call_async(Trigger.Request())

    def update(self) -> Status:
        """Run the Sit behavior when ticked."""
        self.logger.debug(f"  {self.name} [Sit::update()]")

        if self.future.done():
            if self.future.result().success:
                self.blackboard.state.standing = False
                return Status.SUCCESS

            self.logger.error(f"{self.future.result().message}")
            return Status.FAILURE

        self.logger.debug(f"  {self.name} [Sit::update()][RUNNING]")
        return Status.RUNNING

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f"  {self.name} [Sit::terminate()]"
            f"[{self.status}->{new_status}]"
        )


class Stand(Behaviour):
    """
    Behavior class for getting Spot to stand.

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
        Initialize an Stand action object.

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
        self.client = self.node.create_client(Trigger, "stand")
        self.client.wait_for_service()

    def initialise(self):
        """Initialize variables and perform action behavior for first tick."""
        self.logger.debug(f"  {self.name} [Stand::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)

        # Send client request
        self.future = self.client.call_async(Trigger.Request())

    def update(self) -> Status:
        """Run the Stand behavior when ticked."""
        self.logger.debug(f"  {self.name} [Stand::update()]")

        if self.future.done():
            if self.future.result().success:
                self.blackboard.state.standing = True
                return Status.SUCCESS

            self.logger.error(f"{self.future.result().message}")
            return Status.FAILURE

        self.logger.debug(f"  {self.name} [Stand::update()][RUNNING]")
        return Status.RUNNING

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f"  {self.name} [Stand::terminate()]"
            f"[{self.status}->{new_status}]"
        )


class Wait(Behaviour):
    """
    Behavior class for Spot to wait a designated amount of time.

    Attributes:
      blackboard (py_trees.blackboard.Client): Maintains BT variables associated with a
        specific blackboard client.
      mission_blackboard (py_trees.blackboard.Client): Maintains BT variables associated
        with a mission specific blackboard client.
      client (rclpy.client.Client): A ROS service client to connect to the `spot_driver`.
      future (rclpy.task.Future): Stores future information for the async ROS
        client service call.
      node (rclpy.node.Node): Stores the main ROS node for the BT process. This allows
        our action to use some of the nodes functionality.
    """

    def __init__(self, name: str):
        """
        Initialize an Wait action object.

        Args:
          name (str): The action name. This will be applied to logging messages and image
            outputs of the action.
        """
        super().__init__(name)
        self.blackboard: Client = None
        self.mission_blackboard: Client = None
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

    def initialise(self):
        """Initialize variables and perform action behavior for first tick."""
        self.logger.debug(f"  {self.name} [Wait::initialise()]")
        self.blackboard = self.attach_blackboard_client("clients")
        self.blackboard.register_key(key="controller", access=Access.WRITE)
        self.client = self.blackboard.controller
        self.mission_blackboard = self.attach_blackboard_client("mission")
        self.mission_blackboard.register_key(key="wait_time", access=Access.WRITE)

        # Send action client request
        goal_msg = SpotBodyTask.Goal()
        goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.action = "wait"
        goal_msg.options = [self.mission_blackboard.wait_time]
        self.future = self.client.send_goal_async(goal_msg, feedback_callback=self._feedback)

    def update(self) -> Status:
        """Run the Wait behavior when ticked."""
        self.logger.debug(f"  {self.name} [Wait::update()]")

        if self.future.done():
            if self.future.result().success:
                return Status.SUCCESS

            self.logger.error(f"{self.future.result().message}")
            return Status.FAILURE

        self.logger.debug(f"  {self.name} [Wait::update()][RUNNING]")
        return Status.RUNNING

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f"  {self.name} [Wait::terminate()]"
            f"[{self.status}->{new_status}]"
        )

    def _feedback(self, feedback_msg: SpotBodyTask.Feedback):
        """Display the feedback of the Wait action."""
        self.logger.info(f"{feedback_msg.feedback}")
