"""Spot state related actions."""

from __future__ import annotations

from py_trees.behaviour import Behaviour
from py_trees.blackboard import Client
from py_trees.common import Access
from py_trees.common import Status

from py_trees_ros.subscribers import ToBlackboard
from py_trees_ros.utilities import qos_profile_unlatched

import rclpy
from rclpy.client import Client as ServiceClient
from rclpy.node import Node
from rclpy.task import Future

from spot_msgs.msg import BatteryStateArray
from spot_msgs.msg import BehaviorFaultState
from spot_msgs.msg import EStopStateArray
from spot_msgs.msg import LeaseArray
from spot_msgs.msg import Metrics
from spot_msgs.msg import SystemFaultState
from spot_msgs.msg import WiFiState

from std_srvs.srv import Trigger


class ClaimLease(Behaviour):
    """
    Behavior action to claim the Spot lease.
    
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
        Initialize an ClaimLease action object.

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
        self.client = self.node.create_client(Trigger, "claim")
        self.client.wait_for_service()

    def initialise(self):
        """Initialize variables and perform action behavior for first tick."""
        self.logger.debug(f"  {self.name} [ClaimLease::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)

        # Send client request
        self.future = self.client.call_async(Trigger.Request())

    def update(self) -> Status:
        """Run the ClaimLease behavior when ticked."""
        self.logger.debug(f"  {self.name} [ClaimLease::update()]")

        if self.future.done():
            if self.future.result().success:
                self.blackboard.state.lease_claimed = True
                return Status.SUCCESS

            self.logger.error(f"{self.future.result().message}")
            return Status.FAILURE

        self.logger.debug(f"  {self.name} [ClaimLease::update()][RUNNING]")
        return Status.RUNNING

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.logger.debug(
            f"  {self.name} [ClaimLease::terminate()]"
            f"[{self.status}->{new_status}]"
        )


class ReleaseLease(Behaviour):
    """
    Behavior action to release the Spot lease.

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
        Initialize an ReleaseLease action object.

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
        self.client = self.node.create_client(Trigger, "release")
        self.client.wait_for_service()

    def initialise(self):
        """Initialize variables and perform action behavior for first tick."""
        self.logger.debug(f"  {self.name} [ReleaseLease::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.WRITE)

        # Send client request
        self.future = self.client.call_async(Trigger.Request())

    def update(self) -> Status:
        """Run the ReleaseLease behavior when ticked."""
        self.logger.debug(f"  {self.name} [ReleaseLease::update()]")

        if self.future.done():
            if self.future.result().success:
                self.blackboard.state.lease_claimed = False
                return Status.SUCCESS

            self.logger.error(f"{self.future.result().message}")
            return Status.FAILURE

        self.logger.debug(f"  {self.name} [ReleaseLease::update()][RUNNING]")
        return Status.RUNNING

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.logger.debug(
            f"  {self.name} [ReleaseLease::terminate()]"
            f"[{self.status}->{new_status}]"
        )


def create_check_spot_battery_action(
    topic: str = "status/battery_states", qos_profile: int = qos_profile_unlatched
) -> ToBlackboard:
    """Create a check Spot battery status action for use in the BT."""
    return ToBlackboard(
        name="status",
        topic_name=topic,
        topic_type=BatteryStateArray,
        # qos_profile=qos_profile(),
        qos_profile=10,
        blackboard_variables={"battery": None},
    )


def create_check_behavior_faults_action(
    topic: str = "status/behavior_faults", qos_profile: int = qos_profile_unlatched
) -> ToBlackboard:
    """Create a check Spot behavior faults status action for use in the BT."""
    return ToBlackboard(
        name="status",
        topic_name=topic,
        topic_type=BehaviorFaultState,
        # qos_profile=qos_profile(),
        qos_profile=10,
        blackboard_variables={"behavior_faults": None},
    )


def create_check_estop_action(
    topic: str = "status/estop", qos_profile: int = qos_profile_unlatched
) -> ToBlackboard:
    """Create a check Spot EStop status action for use in the BT."""
    return ToBlackboard(
        name="status",
        topic_name=topic,
        topic_type=EStopStateArray,
        # qos_profile=qos_profile(),
        qos_profile=10,
        blackboard_variables={"estop": None},
    )


def create_check_leases_action(
    topic: str = "status/leases", qos_profile: int = rclpy.qos.ReliabilityPolicy
) -> ToBlackboard:
    """Create a check Spot leases status action for use in the BT."""
    return ToBlackboard(
        name="status",
        topic_name=topic,
        topic_type=LeaseArray,
        # qos_profile=qos_profile,
        qos_profile=10,
        blackboard_variables={"leases": None},
    )


def create_check_spot_metrics_action(
    topic: str = "status/metrics", qos_profile: int = rclpy.qos.ReliabilityPolicy
) -> ToBlackboard:
    """Create a check Spot metrics status action for use in the BT."""
    return ToBlackboard(
        name="status",
        topic_name=topic,
        topic_type=Metrics,
        # qos_profile=qos_profile,
        qos_profile=10,
        blackboard_variables={"metrics": None},
    )


def create_check_system_faults_action(
    topic: str = "status/system_faults", qos_profile: int = rclpy.qos.ReliabilityPolicy
) -> ToBlackboard:
    """Create a check Spot system faults status action for use in the BT."""
    return ToBlackboard(
        name="status",
        topic_name=topic,
        topic_type=SystemFaultState,
        # qos_profile=qos_profile,
        qos_profile=10,
        blackboard_variables={"system_faults": None},
    )



def create_check_spot_wifi_action(
    topic: str = "status/wifi", qos_profile: int = rclpy.qos.ReliabilityPolicy
) -> ToBlackboard:
    """Create a check Spot wifi status action for use in the BT."""
    return ToBlackboard(
        name="status",
        topic_name=topic,
        topic_type=WiFiState,
        # qos_profile=qos_profile,
        qos_profile=10,
        blackboard_variables={"wifi": None},
    )
