from __future__ import annotations

import py_trees

from spot_bt_ros.data import Blackboards


class RobotDock(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.dock_id = None

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [RobotDock::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.state.register_key(
            key="dock_id", access=py_trees.common.Access.READ
        )
        self.robot = self.blackboard.state.robot
        self.dock_id = int(self.blackboard.state.dock_id)

    def update(self) -> py_trees.common.Status:
        """Run the Docking behavior when ticked."""
        self.logger.debug(f"  {self.name} [RobotDock::update()]")
        self.robot.get_logger().info("Docking Spot.")
        try:
            result = self.robot.command("dock", dock_id=self.dock_id)
            if result.success:
                self.robot.get_logger().info(f"{result.message}")
                return py_trees.common.Status.SUCCESS

            self.robot.get_logger().error(f"{result.message}")
            return py_trees.common.Status.FAILURE

        except:  # pylint: disable=bare-except
            self.robot.shutdown()
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f" {self.name} [RobotDock::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class RobotUndock(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [RobotUndock::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("Spot State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot

    def update(self) -> py_trees.common.Status:
        """Run the Docking behavior when ticked."""
        self.logger.debug(f"  {self.name} [RobotUndock::update()]")
        self.robot.get_logger().info("Undocking Spot.")
        try:
            if self.robot.state.is_docked:
                result = self.robot.command("undock")
                if result.success:
                    self.robot.get_logger().info(f"{result.message}")
                    self.robot.state.is_docked = False
                    return py_trees.common.Status.SUCCESS

                self.robot.get_logger().error(f"{result.message}")
                return py_trees.common.Status.FAILURE
            
            return py_trees.common.Status.SUCCESS

        except:  # pylint: disable=bare-except
            self.robot.shutdown()
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f" {self.name} [RobotUndock::terminate().terminate()]"
            f"[{self.status}->{new_status}]")
