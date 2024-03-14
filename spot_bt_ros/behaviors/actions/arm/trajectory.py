from __future__ import annotations

from bosdyn.api import geometry_pb2

from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandBuilder

import py_trees

from spot_driver import conversions

from spot_msgs.action import RobotCommand

from spot_bt_ros.data import ArmPose, ArmPoses, Blackboards
from spot_bt_ros.transform import SpotTF


class ArmTrajectory(py_trees.behaviour.Behaviour):
    """Behavior class for getting robot to move manipulator arm to a single point."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.state = None
        self.target = None
        self.command = None
        self.tf: SpotTF = None

    def setup(self, **kwargs):
        """Setup ArmTrajectory behavior before initialization."""
        self.logger.debug(f"  {self.name} [ArmTrajectory::setup()]")

    def initialise(self):
        """Initialize robot object and client behavior for first tick."""
        self.logger.debug(f"  {self.name} [ArmTrajectory::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.state.register_key(
            key="tf_odom", access=py_trees.common.Access.WRITE
        )
        self.blackboard.arm = self.attach_blackboard_client("Arm")
        self.blackboard.arm.register_key(
            key="target", access=py_trees.common.Access.WRITE
        )
        self.blackboard.arm.register_key(
            key="command", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot
        self.tf = self.blackboard.state.tf_odom
        self.target = self.blackboard.arm.target
        self.command = self.blackboard.arm.command

    def update(self) -> py_trees.common.Status:
        """Run the ArmTrajectory behavior when ticked."""
        self.logger.debug(f"  {self.name} [ArmTrajectory::update()]")
        # try:
        # Make the arm pose RobotCommand
        # build a position to move the arm to (in meters, relative to and
        # expressed in the gravity aligned body frame).
        # hand_ewrt_flat_body = self.target.position
        if isinstance(self.target, ArmPoses):
            target = self.target.poses[0]
            self.robot.get_logger().warn("Multiple poses given to ArmTrajector.")
        else:
            target = self.target

        hand_ewrt_flat_body = target.hand_pose()

        # Rotation as a quaternion
        # flat_body_Q_hand = self.target.orientation
        flat_body_Q_hand = target.hand_orientation()

        flat_body_T_hand = geometry_pb2.SE3Pose(
            position=hand_ewrt_flat_body, rotation=flat_body_Q_hand
        )

        odom_T_flat_body = self.tf.lookup_se3_odom_tform_body(
            GRAV_ALIGNED_BODY_FRAME_NAME
        )

        odom_T_hand = odom_T_flat_body * math_helpers.SE3Pose.from_obj(
            flat_body_T_hand
        )

        # Duration in seconds
        seconds = 2  # TODO Consider making this a parameter/config

        arm_command = RobotCommandBuilder.arm_pose_command(
            odom_T_hand.x,
            odom_T_hand.y,
            odom_T_hand.z,
            odom_T_hand.rot.w,
            odom_T_hand.rot.x,
            odom_T_hand.rot.y,
            odom_T_hand.rot.z,
            ODOM_FRAME_NAME,
            seconds,
        )

        # Make the open gripper RobotCommand
        # TODO Fix this. This should be what the current state of the gripper is.
        gripper_command = (
            RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)
        )

        # Combine the arm and gripper commands into one RobotCommand
        command = RobotCommandBuilder.build_synchro_command(
            gripper_command, arm_command
        )

        # Save command for possible gripper use
        self.command = command
        # Convert to ROS 2 message and send
        action_goal = RobotCommand.Goal()
        conversions.convert_proto_to_bosdyn_msgs_robot_command(
            command, action_goal.command
        )
        self.robot.command_client.send_goal_and_wait(
            "arm_trajectory", action_goal
        )

        # Wait until the arm arrives at the goal.
        self.robot.get_logger().info("Arm motion complete.")
        return py_trees.common.Status.SUCCESS

        # except:  # pylint: disable=bare-except
        #     return py_trees.common.Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.blackboard.arm.command = self.command
        self.logger.debug(
            f" {self.name} [ArmTrajectory::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class ArmTrajectories(py_trees.behaviour.Behaviour):
    """Behavior class for getting robot to move manipulator arm to multiple points."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.state = None
        self.target = None
        self.command = None
        self.tf: SpotTF = None

    def setup(self, **kwargs):
        """Setup ArmTrajectories behavior before initialization."""
        self.logger.debug(f"  {self.name} [ArmTrajectories::setup()]")

    def initialise(self):
        """Initialize robot object and client behavior for first tick."""
        self.logger.debug(f"  {self.name} [ArmTrajectories::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.state.register_key(
            key="tf_odom", access=py_trees.common.Access.WRITE
        )
        self.blackboard.arm = self.attach_blackboard_client("Arm")
        self.blackboard.arm.register_key(
            key="target", access=py_trees.common.Access.WRITE
        )
        self.blackboard.arm.register_key(
            key="command", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot
        self.tf = self.blackboard.state.tf_odom
        self.target = self.blackboard.arm.target
        self.command = self.blackboard.arm.command

    def update(self) -> py_trees.common.Status:
        """Run the ArmTrajectories behavior when ticked."""
        self.logger.debug(f"  {self.name} [ArmTrajectories::update()]")
        # try:
        # Make the arm pose RobotCommand
        # build a position to move the arm to (in meters, relative to and
        # expressed in the gravity aligned body frame).
        # hand_ewrt_flat_body = self.target.position
        if isinstance(self.target, ArmPose):
            targets = [self.target.pose]
            self.robot.get_logger().warn("Only one pose given to ArmTrajectories")
        else:
            targets = self.target.poses

        for i, target in enumerate(targets):
            hand_ewrt_flat_body = target.hand_pose()

            # Rotation as a quaternion
            # flat_body_Q_hand = self.target.orientation
            flat_body_Q_hand = target.hand_orientation()

            flat_body_T_hand = geometry_pb2.SE3Pose(
                position=hand_ewrt_flat_body, rotation=flat_body_Q_hand
            )

            odom_T_flat_body = self.tf.lookup_se3_odom_tform_body(
                GRAV_ALIGNED_BODY_FRAME_NAME
            )

            odom_T_hand = odom_T_flat_body * math_helpers.SE3Pose.from_obj(
                flat_body_T_hand
            )

            # Duration in seconds
            seconds = 2  # TODO Consider making this a parameter/config

            arm_command = RobotCommandBuilder.arm_pose_command(
                odom_T_hand.x,
                odom_T_hand.y,
                odom_T_hand.z,
                odom_T_hand.rot.w,
                odom_T_hand.rot.x,
                odom_T_hand.rot.y,
                odom_T_hand.rot.z,
                ODOM_FRAME_NAME,
                seconds,
            )

            # Make the open gripper RobotCommand
            # TODO Fix this. This should be what the current state of the gripper is.
            gripper_command = (
                RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)
            )

            # Combine the arm and gripper commands into one RobotCommand
            command = RobotCommandBuilder.build_synchro_command(
                gripper_command, arm_command
            )

            # Save command for possible gripper use
            self.command = command
            # Convert to ROS 2 message and send
            action_goal = RobotCommand.Goal()
            conversions.convert_proto_to_bosdyn_msgs_robot_command(
                command, action_goal.command
            )
            self.robot.command_client.send_goal_and_wait(
                "arm_trajectory", action_goal
            )

            # Wait until the arm arrives at the goal.
            self.robot.get_logger().info(f"Arm pose {i} trajectory complete.")

        self.robot.get_logger().info("All Arm pose trajectories complete!")
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.blackboard.arm.command = self.command
        self.logger.debug(
            f" {self.name} [ArmTrajectories::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )
