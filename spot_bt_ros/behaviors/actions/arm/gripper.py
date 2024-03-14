from __future__ import annotations

from bosdyn.client.robot_command import RobotCommandBuilder

import py_trees

import spot_driver.conversions as conv

from spot_msgs.action import RobotCommand  # type: ignore

from spot_bt_ros.data import Blackboards


class CloseGripper(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.command = None

    def setup(self, **kwargs):
        """Setup CloseGripper behavior before initialization."""
        self.logger.debug(f"  {self.name} [CloseGripper::setup()]")

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [CloseGripper::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.arm = self.attach_blackboard_client("Arm")
        self.blackboard.arm.register_key(
            key="command", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot
        self.command = self.blackboard.arm.command

    def update(self) -> py_trees.common.Status:
        """Run the CloseGripper behavior when ticked."""
        self.logger.debug(f"  {self.name} [CloseGripper::update()]")

        # Create command, convert to a ROS message, and send
        if self.command is not None:
            self.logger.debug(f"  {self.name} [CloseGripper::NOT_BUILDING]")
            command = RobotCommandBuilder.claw_gripper_close_command()
        else:
            self.logger.debug(f"  {self.name} [CloseGripper::BUILDING]")
            command = RobotCommandBuilder.claw_gripper_close_command(
                build_on_command=self.command
            )
        action_goal = RobotCommand.Goal()
        conv.convert_proto_to_bosdyn_msgs_robot_command(
            command, action_goal.command
        )
        self.robot.command_client.send_goal_and_wait(
            "close_gripper", action_goal
        )
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.logger.debug(
            f" {self.name} [CloseGripper::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class OpenGripper(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.command = None

    def setup(self, **kwargs):
        """Setup OpenGripper behavior before initialization."""
        self.logger.debug(f"  {self.name} [OpenGripper::setup()]")

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [OpenGripper::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.arm = self.attach_blackboard_client("Arm")
        self.blackboard.arm.register_key(
            key="command", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot
        self.command = self.blackboard.arm.command

    def update(self) -> py_trees.common.Status:
        """Run the OpenGripper behavior when ticked."""
        self.logger.debug(f"  {self.name} [OpenGripper::update()]")

        # Create command, convert to a ROS message, and send
        if self.command is not None:
            command = RobotCommandBuilder.claw_gripper_open_command()
        else:
            
            command = RobotCommandBuilder.claw_gripper_open_command(
                build_on_command=self.command
            )
        action_goal = RobotCommand.Goal()
        conv.convert_proto_to_bosdyn_msgs_robot_command(
            command, action_goal.command
        )
        self.robot.command_client.send_goal_and_wait(
            "open_gripper", action_goal
        )
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.logger.debug(
            f" {self.name} [OpenGripper::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )
