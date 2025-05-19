"""Safety-oriented Spot actions."""

from __future__ import annotations

from py_trees.behaviour import Behaviour
from py_trees.blackboard import Client
from py_trees.common import Access
from py_trees.common import Status

from rclpy.client import Client as ServiceClient
from rclpy.client import Future
from rclpy.node import Node

from std_srvs.srv import Trigger


class Stop(Behaviour):
    """
    Behavior action to stop Spot in place.

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
        Initialize an Stop action object.

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
        self.client = self.node.create_client(Trigger, "stop")
        self.client.wait_for_service()

    def initialise(self):
        """Initialize variables and perform action behavior for first tick."""
        self.logger.debug(f"  {self.name} [Stop::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)

        # Send client request
        self.future = self.client.call_async(Trigger.Request())

    def update(self) -> Status:
        """Run the Stop behavior when ticked."""
        self.logger.debug(f"  {self.name} [Stop::update()]")

        if self.future.done():
            if self.future.result().success:
                self.blackboard.state.stopped = True
                return Status.SUCCESS

            self.logger.error(f"{self.future.result().message}")
            return Status.FAILURE

        self.logger.debug(f"  {self.name} [Stop::update()][RUNNING]")
        return Status.RUNNING

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f"  {self.name} [RobotStop::terminate()]"
            f"[{self.status}->{new_status}]"
        )


class EStopHard(Behaviour):
    """
    Behavior action for a hard emergency stop.

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
        Initialize an EStopHard action object.

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
        self.client = self.node.create_client(Trigger, "estop/hard")
        self.client.wait_for_service()

    def initialise(self):
        """Initialize variables and perform action behavior for first tick."""
        self.logger.debug(f"  {self.name} [EStopHard::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)

        # Send client request
        self.future = self.client.call_async(Trigger.Request())

    def update(self) -> Status:
        """Run the EStopHard behavior when ticked."""
        self.logger.debug(f"  {self.name} [EStopHard::update()]")

        if self.future.done():
            if self.future.result().success:
                return Status.SUCCESS

            self.logger.error(f"{self.future.result().message}")
            return Status.FAILURE

        self.logger.debug(f"  {self.name} [EStopHard::update()][RUNNING]")
        return Status.RUNNING

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f"  {self.name} [EStopHard::terminate()]"
            f"[{self.status}->{new_status}]"
        )


class EStopGentle(Behaviour):
    """
    Behavior action for a gentle emergency stop.

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
        Initialize an EStopGentle action object.

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
        self.client = self.node.create_client(Trigger, "estop/gentle")
        self.client.wait_for_service()

    def initialise(self):
        """Initialize variables and perform action behavior for first tick."""
        self.logger.debug(f"  {self.name} [EStopGentle::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)

        # Send client request
        self.future = self.client.call_async(Trigger.Request())

    def update(self) -> Status:
        """Run the EStopGentle behavior when ticked."""
        self.logger.debug(f"  {self.name} [EStopGentle::update()]")

        if self.future.done():
            if self.future.result().success:
                return Status.SUCCESS

            self.logger.error(f"{self.future.result().message}")
            return Status.FAILURE

        self.logger.debug(f"  {self.name} [EStopGentle::update()][RUNNING]")
        return Status.RUNNING

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f"  {self.name} [EStopGentle::terminate()]"
            f"[{self.status}->{new_status}]"
        )


class EStopRelease(Behaviour):
    """
    Behavior action for releasing the emergency stop.

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
        Initialize an EStopRelease action object.

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
        self.client = self.node.create_client(Trigger, "estop/release")
        self.client.wait_for_service()

    def initialise(self):
        """Initialize variables and perform action behavior for first tick."""
        self.logger.debug(f"  {self.name} [EStopRelease::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)

        # Send client request
        self.future = self.client.call_async(Trigger.Request())

    def update(self) -> Status:
        """Run the EStopRelease behavior when ticked."""
        self.logger.debug(f"  {self.name} [EStopRelease::update()]")

        if self.future.done():
            if self.future.result().success:
                return Status.SUCCESS

            self.logger.error(f"{self.future.result().message}")
            return Status.FAILURE

        self.logger.debug(f"  {self.name} [EStopRelease::update()][RUNNING]")
        return Status.RUNNING

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f"  {self.name} [EStopRelease::terminate()]"
            f"[{self.status}->{new_status}]"
        )


class SelfRight(Behaviour):
    """
    Behavior action for Spot to self-right.

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
        Initialize an SelfRight action object.

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
        self.client = self.node.create_client(Trigger, "self_right")
        self.client.wait_for_service()

    def initialise(self):
        """Initialize variables and perform action behavior for first tick."""
        self.logger.debug(f"  {self.name} [SelfRight::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)

        # Send client request
        self.future = self.client.call_async(Trigger.Request())

    def update(self) -> Status:
        """Run the SelfRight behavior when ticked."""
        self.logger.debug(f"  {self.name} [SelfRight::update()]")

        if self.future.done():
            if self.future.result().success:
                return Status.SUCCESS

            self.logger.error(f"{self.future.result().message}")
            return Status.FAILURE

        self.logger.debug(f"  {self.name} [SelfRight::update()][RUNNING]")
        return Status.RUNNING

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f"  {self.name} [SelfRight::terminate()]"
            f"[{self.status}->{new_status}]"
        )
