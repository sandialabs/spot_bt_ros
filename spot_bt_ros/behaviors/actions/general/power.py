"""Robot power function related behaviors"""
from __future__ import annotations

import py_trees

from spot_bt_ros.data import Blackboards


class RobotPowerOn(py_trees.behaviour.Behaviour):
    """Behavior class for powering-on robot."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [RobotPowerOn::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot

    def update(self) -> py_trees.common.Status:
        """Run the PowerOn behavior when ticked."""
        self.logger.debug(f"  {self.name} [RobotPowerOn::update()]")
        self.robot.get_logger().info("Powering ON Spot.")
        try:
            result = self.robot.command("power_on")
            if not result.success:
                self.robot.get_logger().error(f"{result.message}")
                return py_trees.common.Status.FAILURE
        except:  # pylint: disable=bare-except
            self.robot.shutdown()
            return py_trees.common.Status.FAILURE

        self.robot.get_logger().info("Spot powered on!")
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f" {self.name} [RobotPowerOn::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class RobotPowerOff(py_trees.behaviour.Behaviour):
    """Behavior class for powering-off robot."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [RobotPowerOff::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot

    def update(self) -> py_trees.common.Status:
        """Run the PowerOff behavior when ticked."""
        self.logger.debug(f"  {self.name} [RobotPowerOff::update()]")
        try:
            result = self.robot.command("power_off")
            if not result.success:
                self.robot.get_logger().error(f"{result.message}")
                return py_trees.common.Status.FAILURE
        except:  # pylint: disable=bare-except
            self.robot.shutdown()
            return py_trees.common.Status.FAILURE

        self.robot.get_logger().info("Spot powered off!")
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f" {self.name} [RobotPowerOff::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )
