"""Spot arm state-related actions"""

from __future__ import annotations

from py_trees.behaviour import Behaviour
from py_trees.blackboard import Client
from py_trees.common import Access
from py_trees.common import Status

from rclpy.action import ActionClient
from rclpy.client import Client as ServiceClient
from rclpy.client import Future
from rclpy.node import Node

from spot_bt_ros_msgs.action import SpotArmTask

from std_srvs.srv import Trigger


class ArmCarry(Behaviour):
    """
    Behavior action for getting Spot's arm to carry an object.

    Attributes:
      blackboard (py_trees.blackboard.Client): Maintains BT variables associated with a
        specific blackboard client.
      client (rclpy.client.Client): A ROS service client to connect to the `spot_driver`.
      future (rclpy.client.Future): Stores future information for the async ROS
        client service call.
      node (rclpy.node.Node): Stores the main ROS node for the BT process. This allows
        our action to use some of the nodes functionality.
    """

    def __init__(self, name: str):
        """
        Initialize an ArmCarry action object.

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
        """Setup ROS 2 subs/pubs/clients for connection to driver nodes."""
        self.logger.debug(f"{self.qualified_name}.setup()")
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_msg = f"didn't find 'node' in setup's kwargs [{self.qualified_name}]"
            raise KeyError(error_msg) from e

        # Create client
        self.client = self.node.create_client(Trigger, "arm_carry")
        self.client.wait_for_service()

    def initialise(self):
        """Initialize variables and perform action behavior for first tick."""
        self.logger.debug(f"  {self.name} [ArmCarry::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)

        # Send client request
        self.future = self.client.call_async(Trigger.Request())

    def update(self) -> Status:
        """Run the ArmCarry behavior when ticked."""
        self.logger.debug(f"  {self.name} [ArmCarry::update()]")

        if self.future.done():
            if self.future.result().success:
                self.blackboard.state.arm_carrying = True
                return Status.SUCCESS

            self.logger.error(f"{self.future.result().message}")
            return Status.FAILURE

        self.logger.debug(f"  {self.name} [ArmCarry::update()][RUNNING]")
        return Status.RUNNING

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.logger.debug(
            f"  {self.name} [ArmCarry::terminate()]"
            f"[{self.status}->{new_status}]"
        )


class ArmStow(Behaviour):
    """
    Behavior action for getting Spot to stow manipulator arm.

    Attributes:
      blackboard (py_trees.blackboard.Client): Maintains BT variables associated with a
        specific blackboard client.
      client (rclpy.client.Client): A ROS service client to connect to the `spot_driver`.
      future (rclpy.client.Future): Stores future information for the async ROS
        client service call.
      node (rclpy.node.Node): Stores the main ROS node for the BT process. This allows
        our action to use some of the nodes functionality.
    """

    def __init__(self, name: str):
        """
        Initialize an ArmStow action object.

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
        """Setup ROS 2 subs/pubs/clients for connection to driver nodes."""
        self.logger.debug(f"{self.qualified_name}.setup()")
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_msg = f"didn't find 'node' in setup's kwargs [{self.qualified_name}]"
            raise KeyError(error_msg) from e

        # Create client
        self.client = self.node.create_client(Trigger, "arm_stow")
        self.client.wait_for_service()

    def initialise(self):
        """Initialize variables and perform action behavior for first tick."""
        self.logger.debug(f"  {self.name} [ArmStow::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)

        # Send client request
        self.future = self.client.call_async(Trigger.Request())

    def update(self) -> Status:
        """Run the ArmStow behavior when ticked."""
        self.logger.debug(f"  {self.name} [ArmStow::update()]")

        if self.future.done():
            if self.future.result().success:
                self.blackboard.state.arm_stowed = True
                return Status.SUCCESS

            self.logger.error(f"{self.future.result().message}")
            return Status.FAILURE

        self.logger.debug(f"  {self.name} [ArmStow::update()][RUNNING]")
        return Status.RUNNING

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.logger.debug(
            f"  {self.name} [ArmStow::terminate()]"
            f"[{self.status}->{new_status}]"
        )


class ArmUnstow(Behaviour):
    """
    Behavior action for getting Spot to unstow manipulator arm.

    Attributes:
      blackboard (py_trees.blackboard.Client): Maintains BT variables associated with a
        specific blackboard client.
      client (rclpy.client.Client): A ROS service client to connect to the `spot_driver`.
      future (rclpy.client.Future): Stores future information for the async ROS
        client service call.
      node (rclpy.node.Node): Stores the main ROS node for the BT process. This allows
        our action to use some of the nodes functionality.
    """

    def __init__(self, name: str):
        """
        Initialize an ArmUnstow action object.

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
        """Setup ROS 2 subs/pubs/clients for connection to driver nodes."""
        self.logger.debug(f"{self.qualified_name}.setup()")
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_msg = f"didn't find 'node' in setup's kwargs [{self.qualified_name}]"
            raise KeyError(error_msg) from e

        # Create client
        self.client = self.node.create_client(Trigger, "arm_unstow")
        self.client.wait_for_service()

    def initialise(self):
        """Initialize variables and perform action behavior for first tick."""
        self.logger.debug(f"  {self.name} [ArmUnstow::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)

        # Send client request
        self.future = self.client.call_async(Trigger.Request())

    def update(self) -> Status:
        """Run the ArmUnstow behavior when ticked."""
        self.logger.debug(f"  {self.name} [ArmUnstow::update()]")

        if self.future.done():
            if self.future.result().success:
                self.blackboard.state.arm_stowed = False
                return Status.SUCCESS

            self.logger.error(f"{self.future.result().message}")
            return Status.FAILURE

        self.logger.debug(f"  {self.name} [ArmUnstow::update()][RUNNING]")
        return Status.RUNNING

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.logger.debug(
            f"  {self.name} [ArmUnstow::terminate()]"
            f"[{self.status}->{new_status}]"
        )


class ArmDeploy(Behaviour):
    """
    Behavior action for deploying manipulator arm.

    Attributes:
      blackboard (py_trees.blackboard.Client): Maintains BT variables associated with a
        specific blackboard client.
      client (rclpy.action.ActionClient): A ROS action client to connect to the Spot
        controller.
      future (rclpy.client.Future): Stores future information for the async ROS
        client service call.
      node (rclpy.node.Node): Stores the main ROS node for the BT process. This allows
        our action to use some of the nodes functionality.
    """

    def __init__(self, name: str):
        """
        Initialize an ArmDeploy action object.

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
        """Setup ROS 2 subs/pubs/clients for connection to driver nodes."""
        self.logger.debug(f"{self.qualified_name}.setup()")
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_msg = f"didn't find 'node' in setup's kwargs [{self.qualified_name}]"
            raise KeyError(error_msg) from e

    def initialise(self):
        """Initialize variables and perform action behavior for first tick."""
        self.logger.debug(f"  {self.name} [ArmDeploy::initialise()]")
        self.blackboard = self.attach_blackboard_client("clients")
        self.blackboard.register_key(key="arm_controller", access=Access.WRITE)
        self.client = self.blackboard.arm_controller

        # Send action client request
        goal_msg = SpotArmTask.Goal()
        goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.action = "deploy"
        self.future = self.client.send_goal_async(goal_msg, feedback_callback=self._feedback)

    def update(self) -> Status:
        """Run the ArmDeploy behavior when ticked."""
        self.logger.debug(f"  {self.name} [ArmDeploy::update()]")

        if self.future.done():
            result = self.future.result().get_result().result
            if result.success:
                return Status.SUCCESS

            self.logger.error(f"{result.message}")
            return Status.FAILURE

        self.logger.debug(f"  {self.name} [ArmDeploy::update()][RUNNING]")
        return Status.RUNNING

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.logger.debug(
            f"  {self.name} [ArmDeploy::terminate()]"
            f"[{self.status}->{new_status}]"
        )

    def _feedback(self, feedback_msg: SpotArmTask.Feedback):
        """Display the feedback of the ArmDeploy action."""
        self.node.get_logger().info(
            feedback_msg.feedback.feedback, throttle_duration_sec=1.0
        )


class ArmFreeze(Behaviour):
    """
    Behavior action for freezing manipulator arm.

    Attributes:
      blackboard (py_trees.blackboard.Client): Maintains BT variables associated with a
        specific blackboard client.
      client (rclpy.action.ActionClient): A ROS action client to connect to the Spot
        controller.
      future (rclpy.client.Future): Stores future information for the async ROS
        client service call.
      node (rclpy.node.Node): Stores the main ROS node for the BT process. This allows
        our action to use some of the nodes functionality.
    """

    def __init__(self, name: str):
        """
        Initialize an ArmFreeze action object.

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
        """Setup ROS 2 subs/pubs/clients for connection to driver nodes."""
        self.logger.debug(f"{self.qualified_name}.setup()")
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_msg = f"didn't find 'node' in setup's kwargs [{self.qualified_name}]"
            raise KeyError(error_msg) from e

    def initialise(self):
        """Initialize variables and perform action behavior for first tick."""
        self.logger.debug(f"  {self.name} [ArmFreeze::initialise()]")
        self.blackboard = self.attach_blackboard_client("clients")
        self.blackboard.register_key(key="arm_controller", access=Access.WRITE)
        self.client = self.blackboard.arm_controller

        # Send action client request
        goal_msg = SpotArmTask.Goal()
        goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.action = "freeze"
        self.future = self.client.send_goal_async(goal_msg, feedback_callback=self._feedback)

    def update(self) -> Status:
        """Run the ArmDeploy behavior when ticked."""
        self.logger.debug(f"  {self.name} [ArmFreeze::update()]")

        if self.future.done():
            result = self.future.result().get_result().result
            if result.success:
                return Status.SUCCESS

            self.logger.error(f"{result.message}")
            return Status.FAILURE

        self.logger.debug(f"  {self.name} [ArmFreeze::update()][RUNNING]")
        return Status.RUNNING

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.logger.debug(
            f"  {self.name} [ArmFreeze::terminate()]"
            f"[{self.status}->{new_status}]"
        )

    def _feedback(self, feedback_msg: SpotArmTask.Feedback):
        """Display the feedback of the ArmFreeze action."""
        self.node.get_logger().info(
                feedback_msg.feedback.feedback, throttle_duration_sec=1.0
            )
