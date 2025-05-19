"""spot_bt_ros_py Fiducial Arm Demonstration."""

from __future__ import annotations

import sys

from py_trees import logging
from py_trees.blackboard import Blackboard
from py_trees.blackboard import Client
from py_trees.common import Access
from py_trees.composites import Parallel
from py_trees.composites import Sequence
from py_trees.display import render_dot_tree

from py_trees_ros.exceptions import TimedOutError
from py_trees_ros.trees import BehaviourTree

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException

from spot_bt_ros_msgs.action import PlanSpotArmTask
from spot_bt_ros_msgs.action import PlanSpotBodyTask
from spot_bt_ros_msgs.action import SpotArmTask
from spot_bt_ros_msgs.action import SpotBodyTask

import synchros2.process as ros_process

from spot_bt_ros_py.actions.arm import ArmDeploy
from spot_bt_ros_py.actions.arm import ArmStow
from spot_bt_ros_py.actions.arm import ArmTrajectory
from spot_bt_ros_py.actions.arm import ComputeArmPathToFiducial
from spot_bt_ros_py.behaviors.general import power_on_and_stand
from spot_bt_ros_py.behaviors.general import sit_and_power_off
from spot_bt_ros_py.composites.selector import create_generic_fiducial_selector
from spot_bt_ros_py.composites.sequence import create_spot_status_sequence
from spot_bt_ros_py.tick import generic_pre_tick_handler
from spot_bt_ros_py.utils import create_mission_blackboard
from spot_bt_ros_py.utils import create_status_blackboard


def create_arm_motion_behavior() -> Sequence:
    """Create a simple arm motion behavior."""
    behavior = Sequence("Deploy arm and touch fiducial", memory=True)
    behavior.add_children(
        [
            ArmDeploy(name="Deploy arm"),
            create_generic_fiducial_selector(),
            ComputeArmPathToFiducial(name="Compute arm path to fiducial"),
            ArmTrajectory(name="Move arm to fiducial"),
            ArmStow(name="Stow arm"),
        ]
    )

    return behavior


def create_task(memory: bool = False) -> Sequence:
    """Create the demo task for the system."""
    sequence = Sequence("Fiducial arm routine", memory=memory)
    sequence.add_children(
        [
            power_on_and_stand(),
            create_arm_motion_behavior(),
            sit_and_power_off(),
        ]
    )

    return sequence


def create_root() -> Parallel:
    """Create the root for the Autonomy capability."""
    root = Sequence("Demo fiducial arm", memory=True)
    root.add_children(
        [
            create_spot_status_sequence(),
            create_task(),
        ]
    )

    render_dot_tree(root)

    return root


@ros_process.main()
def main():
    """Create demo behavior tree and run."""

    # Set BT debug level and activity stream
    logging.level = logging.Level.DEBUG
    Blackboard.enable_activity_stream(maximum_size=100)

    # Create action clients blackboard
    clients_blackboard = Client(name="clients")
    clients_blackboard.register_key(key="controller", access=Access.WRITE)
    clients_blackboard.controller = ActionClient(
        main.node, SpotBodyTask, "spot/motion/controller/body"
    )
    clients_blackboard.register_key(key="arm_controller", access=Access.WRITE)
    clients_blackboard.arm_controller = ActionClient(
        main.node, SpotArmTask, "spot/motion/controller/arm"
    )
    clients_blackboard.register_key(key="planner", access=Access.WRITE)
    clients_blackboard.planner = ActionClient(
        main.node, PlanSpotBodyTask, "spot/nav/planner/body"
    )
    clients_blackboard.register_key(key="arm_planner", access=Access.WRITE)
    clients_blackboard.planner = ActionClient(
        main.node, PlanSpotArmTask, "spot/nav/planner/arm"
    )

    # Create status blackboard
    status_blackboard = create_status_blackboard(docked=True)

    # Create mission blackboard
    mission_blackboard = create_mission_blackboard(dock_id=549)
    mission_blackboard.register_key(key="fiducials", access=Access.WRITE)
    mission_blackboard.fiducials = None
    mission_blackboard.register_key(key="target", access=Access.WRITE)
    mission_blackboard.target = None

    # Enable tree stewardship
    root = create_root()
    tree = BehaviourTree(root)
    tree.add_pre_tick_handler(generic_pre_tick_handler)

    # Setup the behavior tree
    try:
        tree.setup(timeout=15)
        # tree.setup(node=main.node, timeout=15)
    except TimedOutError as e:
        main.node.get_logger().error(f"Failed to setup the tree, aborting [{e}]")
        main.node.destroy_node()
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        main.node.get_logger().error("Tree setup interrupted!")
        main.node.destroy_node()
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=1000.0)

    # Execute the behavior tree
    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        main.node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
