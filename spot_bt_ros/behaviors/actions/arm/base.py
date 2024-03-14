from __future__ import annotations

from bosdyn.client.robot_command import RobotCommandBuilder

import py_trees

import spot_driver.conversions as conv

from spot_msgs.action import RobotCommand  # type: ignore

from spot_bt_ros.data import Blackboards


class ArmStow(py_trees.behaviour.Behaviour):
    """Behavior class for getting robot to stow manipulator arm."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None

    def setup(self, **kwargs):
        """Setup ArmStow behavior before initialization."""
        self.logger.debug(f"  {self.name} [ArmStow::setup()]")

    def initialise(self):
        """Initialize robot object and client behavior for first tick."""
        self.logger.debug(f"  {self.name} [ArmStow::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot

    def update(self) -> py_trees.common.Status:
        """Run the ArmStow behavior when ticked."""
        self.logger.debug(f"  {self.name} [ArmStow::update()]")

        # Create command, conver to a ROS message, and send
        command = RobotCommandBuilder.arm_stow_command()
        action_goal = RobotCommand.Goal()
        conv.convert_proto_to_bosdyn_msgs_robot_command(
            command, action_goal.command
        )
        self.robot.get_logger().info("Stow command issued.")
        self.robot.command_client.send_goal_and_wait(
            "arm_stow", action_goal
        )

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.logger.debug(
            f" {self.name} [ArmStow::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class ArmUnstow(py_trees.behaviour.Behaviour):
    """Behavior class for getting robot to unstow manipulator arm."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None

    def setup(self, **kwargs):
        """Setup ArmUnstow behavior before initialization."""
        self.logger.debug(f"  {self.name} [ArmUnstow::setup()]")

    def initialise(self):
        """Initialize robot object and client behavior for first tick."""
        self.logger.debug(f"  {self.name} [ArmUnstow::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(key="robot", access=py_trees.common.Access.WRITE)
        self.robot = self.blackboard.state.robot

    def update(self) -> py_trees.common.Status:
        """Run the ArmUnstow behavior when ticked."""
        self.logger.debug(f"  {self.name} [ArmUnstow::update()]")

        # Create command, conver to a ROS message, and send
        command = RobotCommandBuilder.arm_ready_command()
        action_goal = RobotCommand.Goal()
        conv.convert_proto_to_bosdyn_msgs_robot_command(
            command, action_goal.command
        )
        self.robot.get_logger().info("Unstow command issued.")
        self.robot.command_client.send_goal_and_wait(
            "arm_unstow", action_goal
        )

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.logger.debug(
            f" {self.name} [ArmUnstow::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )
