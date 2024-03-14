"""Robot Pose related behaviors"""
from __future__ import annotations

import py_trees

from spot_bt_ros.data import Blackboards


class RobotPose(py_trees.behaviour.Behaviour):
    """Behavior class for getting robot to pose."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.pose = None

    def setup(self, **kwargs):
        """Setup RobotPose behavior before initialization."""
        self.logger.debug(f"  {self.name} [RobotPose::setup()]")

    def initialise(self):
        """Initialize robot object and client behavior for first tick."""
        self.logger.debug(f"  {self.name} [RobotPose::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.state.register_key(
            key="pose", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot
        self.pose = self.blackboard.state.pose

    def update(self) -> py_trees.common.Status:
        """Run the RobotPose behavior when ticked."""
        self.logger.debug(f"  {self.name} [RobotPose::update()]")
        try:
            # assert self.robot.is_powered_on(), "Robot power on failed."

            action_goal = self.pose.create_command()
            self.robot.command_client.send_goal_and_wait("strike_pose", action_goal)
            self.pose.mark()

        except:  # pylint: disable=bare-except
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.blackboard.state.pose = self.pose
        self.logger.debug(
            f" {self.name} [RobotPose::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class RobotSit(py_trees.behaviour.Behaviour):
    """Behavior class for getting robot to sit."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None

    def setup(self, **kwargs):
        """Setup RobotPose behavior before initialization."""
        self.logger.debug(f"  {self.name} [RobotSit::setup()]")

    def initialise(self):
        """Initialize robot object and client behavior for first tick."""
        self.logger.debug(f"  {self.name} [RobotSit::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot

    def update(self) -> py_trees.common.Status:
        """Run the RobotSit behavior when ticked."""
        self.logger.debug(f"  {self.name} [RobotSit::update()]")
        try:
            result = self.robot.command("sit")
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
            f" {self.name} [RobotSit::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class RobotStand(py_trees.behaviour.Behaviour):
    """Behavior class for getting robot to stand."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None

    def setup(self, **kwargs):
        """Setup RobotPose behavior before initialization."""
        self.logger.debug(f"  {self.name} [RobotStand::setup()]")

    def initialise(self):
        """Initialize robot object and client behavior for first tick."""
        self.logger.debug(f"  {self.name} [RobotStand::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot

    def update(self) -> py_trees.common.Status:
        """Run the RobotStand behavior when ticked."""
        self.logger.debug(f"  {self.name} [RobotStand::update()]")
        try:
            result = self.robot.command("stand")
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
            f" {self.name} [RobotStand::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )
