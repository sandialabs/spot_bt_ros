"""Spot arm command building and execution functions."""

from __future__ import annotations

from bosdyn.api import geometry_pb2
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from bosdyn.client.frame_helpers import VISION_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandBuilder

from bosdyn_msgs.conversions import convert

from geometry_msgs.msg import PoseStamped

from rclpy.action.server import ServerGoalHandle
from rclpy.task import Future

from spot_msgs.action import RobotCommand  # type: ignore

from spot_bt_ros_msgs.action import SpotArmTask

from spot_bt_ros_node.data import ArmPoseBasic


class SpotArmMixin:
    """Special Spot Arm functionality."""

    ARM_TRIGGER_SERVICES = [
        "freeze",  # Freeze arm position
        "deploy",  # Deploy arm to specific configuration
    ]

    def build_arm_freeze_command(self) -> RobotCommand.Goal:
        """Build Spot Arm freeze command and convert to ROS action msg."""
        command = RobotCommandBuilder.arm_joint_freeze_command()
        goal_msg = RobotCommand.Goal()
        convert(command, goal_msg.command)
        return goal_msg

    def build_arm_deploy_command(
        self, x: float = 0.60, y: float = 0.0, z: float = 0.35
    ) -> RobotCommand.Goal:
        """Build Spot Arm deploy command and convert to ROS action msg."""
        # pylint: disable=invalid-name
        target = ArmPoseBasic(x=x, y=y, z=z, w=1.0, qx=0.0, qy=0.0, qz=0.0)
        hand_ewrt_flat_body = target.hand_pose()

        # Rotation as a quaternion
        flat_body_Q_hand = target.hand_orientation()

        flat_body_T_hand = geometry_pb2.SE3Pose(
            position=hand_ewrt_flat_body, rotation=flat_body_Q_hand
        )

        odom_T_flat_body = self._tf.lookup_se3_odom_tform_grav_body()
        odom_T_hand = odom_T_flat_body * math_helpers.SE3Pose.from_obj(flat_body_T_hand)

        # Duration in seconds
        seconds = 5  # TODO Consider making this a parameter/config

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
        # command = RobotCommandBuilder.arm_ready_command(),

        # Make the open gripper RobotCommand
        # TODO Fix this. This should be what the current state of the gripper is.
        gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(0.0)

        # Combine the arm and gripper commands into one RobotCommand
        command = RobotCommandBuilder.build_synchro_command(
            gripper_command, arm_command
        )

        # Convert to ROS action message
        goal_msg = RobotCommand.Goal()
        convert(command, goal_msg.command)

        return goal_msg

    def build_arm_trajectory_command(
        self, target: PoseStamped, frame: str = ODOM_FRAME_NAME
    ) -> RobotCommand.Goal:
        """Create an arm trajectory given an arm pose."""
        # pylint: disable=invalid-name
        hand_ewrt_flat_body = geometry_pb2.Vec3(
            x=target.pose.position.x,
            y=target.pose.position.y,
            z=target.pose.position.z,
        )

        # Rotation as a quaternion
        flat_body_Q_hand = geometry_pb2.Quaternion(
            w=target.pose.orientation.w,
            x=target.pose.orientation.x,
            y=target.pose.orientation.y,
            z=target.pose.orientation.z,
        )

        flat_body_T_hand = geometry_pb2.SE3Pose(
            position=hand_ewrt_flat_body, rotation=flat_body_Q_hand
        )

        # Duration in seconds
        seconds = 2  # TODO Consider making this a parameter/config

        if frame == ODOM_FRAME_NAME:
            odom_T_flat_body = self._tf.lookup_se3_odom_tform_grav_body()
            odom_T_hand = odom_T_flat_body * math_helpers.SE3Pose.from_obj(flat_body_T_hand)
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
        else:
            vision_T_flat_body = self._tf.lookup_se3_vision_tform_grav_body()
            vision_T_hand = vision_T_flat_body * math_helpers.SE3Pose.from_obj(flat_body_T_hand)
            arm_command = RobotCommandBuilder.arm_pose_command(
                vision_T_hand.x,
                vision_T_hand.y,
                vision_T_hand.z,
                vision_T_hand.rot.w,
                vision_T_hand.rot.x,
                vision_T_hand.rot.y,
                vision_T_hand.rot.z,
                VISION_FRAME_NAME,
                seconds,
            )

        # Make the open gripper RobotCommand
        # TODO Fix this. This should be what the current state of the gripper is.
        gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)

        # Combine the arm and gripper commands into one RobotCommand
        command = RobotCommandBuilder.build_synchro_command(
            gripper_command, arm_command
        )

        # Save command for possible gripper use
        self.command = command

        # Convert to ROS action message
        goal_msg = RobotCommand.Goal()
        convert(command, goal_msg.command)

        return goal_msg

    def _execute_arm_action(
        self, goal_handle: ServerGoalHandle
    ) -> SpotArmTask.Result:
        """Execute predefined Spot arm action."""
        self._called_action_goal_handle = goal_handle
        action: str = goal_handle.request.action
        goal_msg: RobotCommand.Goal = None
        future: Future = None

        # Create correct action request
        if action in self.ARM_TRIGGER_SERVICES:
            goal_msg = self._predefined_arm_commands[action]()
        else:
            goal_msg = self.build_arm_trajectory_command(goal_handle.request.goal)

        # Send robot_command action
        future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._save_spot_arm_action_feedback
        )
        future.add_done_callback(self._get_arm_response)

        # Wait for action to complete
        self._action_done_event.wait()

        # Logic for called action completion
        result = SpotArmTask.Result()
        result.header.stamp = self.get_clock().now().to_msg()
        if self._arm_action_result.success:
            goal_handle.succeed()
            result.success = True
            result.message = f"Result: Action '{action}' completed successfully!"
            self.get_logger().debug(result.message)
        else:
            goal_handle.abort()
            result.success = False
            result.message = self._arm_action_result.message
            self.get_logger().error(result.message)

        # Reset variables
        self._saved_arm_feedback_msg = None

        return result

    def _get_arm_response(self, future: Future):
        """Get arm response from the action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Arm goal from Spot rejected!")
            return

        self.get_logger().info("Arm goal accepted!")
        self._pre_action_result = goal_handle.get_result_async()
        self._pre_action_result.add_done_callback(self._get_arm_result)

    def _get_arm_result(self, future: Future):
        """Get arm result from the action server."""
        self._arm_action_result = future.result().result
        self._action_done_event.set()  # Release hold on called action callback
        self.get_logger().debug("Arm action finished...")

    def _save_spot_arm_action_feedback(self, feedback_msg: RobotCommand.Feedback):
        "Save Spot wrapper action feedback message and send back to called action client."
        self._saved_arm_feedback_msg = feedback_msg
        feedback = (
            self._saved_arm_feedback_msg.feedback.feedback.command.synchronized_feedback
            .arm_command_feedback.feedback.arm_cartesian_feedback
        )
        called_action_feedback_msg = SpotArmTask.Feedback()
        called_action_feedback_msg.feedback = (
            "Arm cartesian feedback: \n"
            f"\tDistance to goal (pos): {feedback.measured_pos_distance_to_goal} meters\n"
            f"\tTracking error (pos): {feedback.measured_pos_tracking_error} meters\n"
            f"\tDistance to goal (rot): {feedback.measured_rot_distance_to_goal} radians\n"
            f"\tTracking error (rot): {feedback.measured_rot_tracking_error} radians"
        )
        self._called_action_goal_handle.publish_feedback(called_action_feedback_msg)
