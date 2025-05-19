"""spot_bt_ros_py Search Demonstration"""

from __future__ import annotations

import sys

from py_trees import logging
from py_trees.behaviour import Behaviour
from py_trees.blackboard import Blackboard
from py_trees.blackboard import Client
from py_trees.common import Access
from py_trees.common import Status
from py_trees.composites import Parallel
from py_trees.composites import Selector
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

from spot_bt_ros_py.actions.movement import MoveToTarget
from spot_bt_ros_py.actions.movement import ComputeNewWaypoint
from spot_bt_ros_py.actions.perception import DetectFiducialMarkers
from spot_bt_ros_py.behaviors.general import dock
from spot_bt_ros_py.behaviors.general import undock
from spot_bt_ros_py.composites.sequence import create_spot_status_sequence
from spot_bt_ros_py.conditions.perception import IsAnyFiducialMarkerDetected
from spot_bt_ros_py.tick import generic_pre_tick_handler
from spot_bt_ros_py.utils import create_mission_blackboard
from spot_bt_ros_py.utils import create_status_blackboard



class IsFiducialMoveComplete(Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard: Client = None

    def initialise(self):
        """Initialize variables for condition behavior on first tick."""
        self.logger.debug(f"  {self.name} [IsFiducialMoveComplete::initialise()]")
        self.blackboard = self.attach_blackboard_client("mission")
        self.blackboard.register_key(key="fiducial_move_complete", access=Access.READ)

    def update(self):
        """Run the IsFiducialMoveComplete condition when ticked."""
        self.logger.debug(f"  {self.name} [IsFiducialMoveComplete::update()]")
        if self.blackboard.fiducial_move_complete:
            return Status.SUCCESS

        return Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate condition and save information."""
        self.logger.debug(
            f"  {self.name} [IsFiducialMoveComplete::terminate()]"
            f"[{self.status}->{new_status}]"
        )


class MarkFiducialMoveComplete(Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard: Client = None

    def initialise(self):
        """Initialize variables and perform action behavior for first tick."""
        self.logger.debug(f"  {self.name} [MarkFiducialMoveComplete::initialise()]")
        self.blackboard = self.attach_blackboard_client("mission")
        self.blackboard.register_key(key="fiducial_move_complete", access=Access.WRITE)

    def update(self):
        """Run the IsFiducialMoveComplete condition when ticked."""
        self.logger.debug(f"  {self.name} [MarkFiducialMoveComplete::update()]")
        self.blackboard.fiducial_move_complete = True
        return Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate condition and save information."""
        self.logger.debug(
            f"  {self.name} [MarkFiducialMoveComplete::terminate()]"
            f"[{self.status}->{new_status}]"
        )


def create_search_and_detect_selector(
    name="Search for Marker, if not Detected", memory: bool = False
) -> Selector:
    """Create a selector to check if a fiducial is found and move to it."""
    sequence = Sequence("Search and Detect", memory=True)
    sequence.add_children(
        [
            # TODO replace with planner call
            ComputeNewWaypoint("Get random search trajectory"),
            MoveToTarget("Execute search"),
            DetectFiducialMarkers("Detect fiducial marker"),
        ]
    )
    selector = Selector(name, memory)
    selector.add_children(
        [
            IsAnyFiducialMarkerDetected("Is any fiducial marker detected?"),
            sequence,
        ]
    )

    return selector


def create_search_detect_and_move_behavior(
    name="Search, Detect, and Move to Fiducial", memory: bool = False
) -> Sequence:
    """Create a Search, Detect and Move to Fiducial Sub-Behavior"""
    sequence = Sequence(name, memory)
    sequence.add_children(
        [
            create_search_and_detect_selector(),
            IsAnyFiducialMarkerDetected("Is non-fiducial marker detected?", include_dock=False),
            MoveToTarget("Move to fiducial target"),
            MarkFiducialMoveComplete("Mark completed"),
        ]
    )
    return sequence


def create_search_and_move_to_fiducial_ppa(
    name: str = "Move to Fiducial PPA", memory: bool = False
) -> Selector:
    """Create Move to Fiducial Precondition-Postcondition-Action."""
    ppa = Selector(name, memory)
    ppa.add_children(
        [
            IsFiducialMoveComplete("Is move to fiducial complete?"),
            create_search_detect_and_move_behavior(),
        ]
    )

    return ppa


def create_task(memory: bool = False) -> Sequence:
    """Create the demo task for the system."""
    sequence = Sequence("Search routine", memory=memory)
    sequence.add_children(
        [
            undock(),
            create_search_and_move_to_fiducial_ppa(),
            dock(),
        ]
    )

    return sequence


def create_root() -> Parallel:
    """Create the root for the Autonomy capability."""
    root = Sequence("Demo search", memory=True)
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
    mission_blackboard.register_key(key="fiducial_move_complete", access=Access.WRITE)
    mission_blackboard.fiducial_move_complete = False

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
