"""Spot power function related actions"""

from __future__ import annotations

from py_trees.behaviour import Behaviour
from py_trees.blackboard import Client
from py_trees.common import Access
from py_trees.common import Status

from rclpy.client import Client as ServiceClient
from rclpy.node import Node
from rclpy.task import Future

from std_srvs.srv import Trigger


class PowerOn(Behaviour):
    """
    Behavior action for powering on Spot.

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
        Initialize an PowerOn action object.

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
        self.client = self.node.create_client(Trigger, "power_on")
        self.client.wait_for_service()

    def initialise(self):
        """Initialize variables and perform action behavior for first tick."""
        self.logger.debug(f"  {self.name} [PowerOn::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)

        # Send client request
        self.future = self.client.call_async(Trigger.Request())

    def update(self) -> Status:
        """Run the PowerOn behavior when ticked."""
        self.logger.debug(f"  {self.name} [PowerOn::update()]")

        if self.future.done():
            if self.future.result().success:
                self.blackboard.state.powered_on = True
                return Status.SUCCESS

            self.logger.error(f"{self.future.result().message}")
            return Status.FAILURE

        self.logger.debug(f"  {self.name} [PowerOn::update()][RUNNING]")
        return Status.RUNNING

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f"  {self.name} [PowerOn::terminate()]"
            f"[{self.status}->{new_status}]"
        )


class PowerOff(Behaviour):
    """
    Behavior action for powering off Spot.

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
        Initialize an PowerOff action object.

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
        self.client = self.node.create_client(Trigger, "power_off")
        self.client.wait_for_service()

    def initialise(self):
        """Initialize variables and perform action behavior for first tick."""
        self.logger.debug(f"  {self.name} [PowerOff::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)

        # Send client request
        self.future = self.client.call_async(Trigger.Request())

    def update(self) -> Status:
        """Run the PowerOff behavior when ticked."""
        self.logger.debug(f"  {self.name} [PowerOff::update()]")

        if self.future.done():
            if self.future.result().success:
                self.blackboard.state.powered_on = False
                return Status.SUCCESS

            self.logger.error(f"{self.future.result().message}")
            return Status.FAILURE

        self.logger.debug(f"  {self.name} [PowerOff::update()][RUNNING]")
        return Status.RUNNING

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f"  {self.name} [PowerOff::terminate()]"
            f"[{self.status}->{new_status}]"
        )
