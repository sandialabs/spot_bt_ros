"""Useful Spot behavior tree utilities."""

from __future__ import annotations

from typing import Any

from py_trees.blackboard import Client
from py_trees.common import Access

from rclpy.node import Node
from rclpy.action import ActionClient

from spot_msgs.msg import BatteryStateArray
from spot_msgs.msg import BehaviorFaultState
from spot_msgs.msg import EStopStateArray
from spot_msgs.msg import LeaseArray
from spot_msgs.msg import Metrics
from spot_msgs.msg import SystemFaultState
from spot_msgs.msg import WiFiState

from spot_bt_ros_msgs.action import PlanSpotArmTask
from spot_bt_ros_msgs.action import PlanSpotBodyTask
from spot_bt_ros_msgs.action import SpotArmTask
from spot_bt_ros_msgs.action import SpotBodyTask

from spot_bt_ros_py.data import SpotState


def create_client_blackboard(
    node: Node,
    controller_topic: str = "spot/body/actions/motion",
    controller_type: Any = SpotBodyTask,
    planner_topic: str = "spot/planner/search",
    planner_type: Any = PlanSpotBodyTask,
    *,
    has_arm: bool = False,
    share_topics: bool = False,
    enable_arm_controller: bool = True,
    arm_controller_topic: str = "spot/arm/actions/motion",
    arm_controller_type: Any = SpotArmTask,
    enable_arm_planner: bool = True,
    arm_planner_topic: str = "spot/planner/arm",
    arm_planner_type: Any = PlanSpotArmTask,
) -> Client:
    """
    Create a default Spot 'client' blackboard.
    
    Args:
      node (rclpy.node.Node):
      controller_topic (str):
      controller_type (Any):
      planner_topic (str):
      planner_type (Any):
      has_arm (bool):
      share_topics (bool):
      enable_arm_controller (bool):
      arm_controller_topic (str):
      arm_controller_type (Any):
      enable_arm_planner (bool):
      arm_planner_topic (str):
      arm_planner_type (Any):
    """
    blackboard = Client(name="clients")
    blackboard.register_key(key="controller", access=Access.WRITE)
    blackboard.controller = ActionClient(node, controller_type, controller_topic)
    blackboard.register_key(key="planner", access=Access.WRITE)
    blackboard.planner = ActionClient(node, planner_type, planner_topic)

    if has_arm:
        if share_topics:
            arm_controller_topic = controller_topic
            arm_controller_type = controller_type
            arm_planner_topic = planner_topic
            arm_planner_type = planner_type

        if enable_arm_controller:
            blackboard.register_key(key="arm_controller", access=Access.WRITE)
            blackboard.arm_controller = ActionClient(
                node, arm_controller_type, arm_controller_topic
            )

        if enable_arm_planner:
            blackboard.register_key(key="arm_planner", access=Access.WRITE)
            blackboard.planner = ActionClient(
                node, arm_planner_type, arm_planner_topic
            )

    return blackboard


def create_status_blackboard(
    *,
    docked: bool = False,
    add_behavior_faults_status: bool = False,
    add_estop_status: bool = False,
    add_leases_status: bool = False,
    add_system_faults_status: bool = False,
) -> Client:
    """Create a default Spot 'status' blackboard."""
    blackboard = Client(name="status")
    blackboard.register_key(key="battery", access=Access.WRITE)
    blackboard.battery = BatteryStateArray()
    blackboard.register_key(key="metrics", access=Access.WRITE)
    blackboard.metrics = Metrics()
    blackboard.register_key(key="state", access=Access.WRITE)
    blackboard.state = SpotState(docked=docked)
    blackboard.register_key(key="wifi", access=Access.WRITE)
    blackboard.wifi = WiFiState()

    # Add optional blackboard variables
    if add_behavior_faults_status:
        blackboard.register_key(key="behavior_faults", access=Access.WRITE)
        blackboard.behavior_faults = BehaviorFaultState()

    if add_estop_status:
        blackboard.register_key(key="estop", access=Access.WRITE)
        blackboard.estop = EStopStateArray()

    if add_leases_status:
        blackboard.register_key(key="leases", access=Access.WRITE)
        blackboard.leases = LeaseArray()

    if add_system_faults_status:
        blackboard.register_key(key="system_faults", access=Access.WRITE)
        blackboard.system_faults = SystemFaultState()

    return blackboard


def create_mission_blackboard(
    *,
    dock_id: int | None = None,
    wait_time: float = 5.0,
    spin_amount: float = 45.0,
    backup_amount: float = 1.0,
) -> Client:
    """Create a default Spot 'mission' blackboard."""
    blackboard = Client(name="mission")

    # Default blackboard variables
    blackboard.register_key(key="wait_time", access=Access.WRITE)
    blackboard.wait_time = wait_time
    blackboard.register_key(key="spin_amount", access=Access.WRITE)
    blackboard.spin_amount = spin_amount
    blackboard.register_key(key="backup_amount", access=Access.WRITE)
    blackboard.backup_amount = backup_amount

    # TODO add fiducial and target objects

    # TODO add target and current pose variables

    # Add optional blackboard variables
    if dock_id is not None:
        blackboard.register_key(key="dock_id", access=Access.WRITE)
        blackboard.dock_id = dock_id

    return blackboard
