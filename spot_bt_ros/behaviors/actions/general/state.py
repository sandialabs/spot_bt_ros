"""Robot state related behaviors."""
from __future__ import annotations

import py_trees

from spot_bt_ros.data import Blackboards


class ClaimLease(py_trees.behaviour.Behaviour):
    """Behavior to claim robot lease."""

    def __init__(self, name: str):
        super().__init__(name)
        self.robot = None
        self.blackboard = Blackboards()

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [ClaimLease::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot

    def update(self) -> py_trees.common.Status:
        """Run the ClaimLease behavior when ticked."""
        self.logger.debug(f"  {self.name} [ClaimLease::update()]")
        self.robot.get_logger().info("Claiming Spot...")
        result = self.robot.command("claim")
        if result.success:
            self.robot.get_logger().info("Spot claimed!")
            return py_trees.common.Status.SUCCESS

        self.robot.get_logger().error("Failed to claim Spot.")
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.logger.debug(
            f" {self.name} [ClaimLease::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class ReleaseLease(py_trees.behaviour.Behaviour):
    """Behavior to release robot lease."""

    def __init__(self, name: str):
        super().__init__(name)
        self.robot = None
        self.blackboard = Blackboards()

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [ReleaseLease::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot

    def update(self) -> py_trees.common.Status:
        """Run the ReleaseLease behavior when ticked."""
        self.logger.debug(f"  {self.name} [ReleaseLease::update()]")
        self.robot.get_logger().info("Releasing Spot...")
        result = self.robot.command("release")
        if result.success:
            self.robot.get_logger().info("Spot released!")
            return py_trees.common.Status.SUCCESS

        self.robot.get_logger().error("Failed to release Spot.")
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.logger.debug(
            f" {self.name} [ReleaseLease::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )
