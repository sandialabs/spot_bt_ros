"""Spot body command building and execution functions."""

from __future__ import annotations

from bosdyn.api.spot import robot_command_pb2
from bosdyn.api.world_object_pb2 import WorldObject
from bosdyn.client.frame_helpers import get_a_tform_b
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME
from bosdyn.client.frame_helpers import VISION_FRAME_NAME
from bosdyn.client.math_helpers import Quat
from bosdyn.client.math_helpers import SE2Pose
from bosdyn.client.math_helpers import SE3Pose
from bosdyn.client.robot_command import RobotCommandBuilder

from bosdyn_msgs.conversions import convert

from bosdyn_api_msgs.msg import WorldObject as WorldObjectMsg

from geometry_msgs.msg import PoseStamped

import numpy as np

from rclpy.action.server import ServerGoalHandle
from rclpy.task import Future
from rclpy.duration import Duration

from spot_msgs.action import RobotCommand

from spot_bt_ros_msgs.action import SpotBodyTask

from spot_bt_ros_node.data import SE2_TRAJ_BODY_MOVEMENT_STATUS
from spot_bt_ros_node.data import SE2_TRAJ_FINAL_GOAL_STATUS
from spot_bt_ros_node.data import SE2_TRAJ_SPOT_STATUS
from spot_bt_ros_node.utils import get_desired_angle
from spot_bt_ros_node.utils import get_default_mobility_parameters


class SpotBodyMixin:
    """Special Spot Body functionality."""

    BODY_PREDEFINED_ACTIONS = [
        "spin",  # Rotate the robot in place
        "back_up",  # Back up a certain distance
        "wait", # Wait for a designated about of time
    ]

    def build_move_to_fiducial_command(self, fiducial: WorldObjectMsg) -> RobotCommand.Goal:
        """Build a command for Spot to move to a fiducial marker."""
        # pylint: disable=no-member
        # Get target fiducial to move Spot towards
        target_fiducial = WorldObject()
        convert(fiducial, target_fiducial)
        # Get vision transform with respect to fiducial.
        vision_tform_fiducial = get_a_tform_b(
            target_fiducial.transforms_snapshot,
            VISION_FRAME_NAME,
            target_fiducial.apriltag_properties.frame_name_fiducial,
        ).to_proto()
        fiducial_rt_world = vision_tform_fiducial.position

        # Compute the go-to point with offsets
        robot_rt_world = self.tf.lookup_se3_vision_tform_body()
        robot_to_fiducial_ewrt_world = np.array(
            [
                fiducial_rt_world.x - robot_rt_world.x,
                fiducial_rt_world.y - robot_rt_world.y,
                0,
            ]
        )
        robot_to_fiducial_ewrt_world_norm = (
            robot_to_fiducial_ewrt_world / np.linalg.norm(robot_to_fiducial_ewrt_world)
        )
        heading = get_desired_angle(robot_to_fiducial_ewrt_world_norm)
        goto_rt_world = np.array(
            [
                fiducial_rt_world.x - robot_to_fiducial_ewrt_world_norm[0] * 1.0,
                fiducial_rt_world.y - robot_to_fiducial_ewrt_world_norm[1] * 1.0,
            ]
        )

        # Set mobility parameters
        # TODO: Make this into a static function possibly.
        mobility_parameters = get_default_mobility_parameters()

        # Build command, convert, and send to Spot
        proto_goal = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=goto_rt_world[0],
            goal_y=goto_rt_world[1],
            goal_heading=heading,
            frame_name=VISION_FRAME_NAME,
            # params=mobility_parameters,
            body_height=0.0,
            locomotion_hint=robot_command_pb2.HINT_AUTO,
            # build_on_command=self.blackboard.command,
        )
        goal_msg = RobotCommand.Goal()
        convert(proto_goal, goal_msg.command)

        return goal_msg

    def build_back_up_command(self, distance: float = 1.0) -> RobotCommand.Goal:
        """Build a command for Spot to back up a certain distance."""
        world_t_robot = self._tf.lookup_a_tform_b(
            VISION_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME
        )
        world_t_robot_se2 = SE3Pose(
            world_t_robot.transform.translation.x,
            world_t_robot.transform.translation.y,
            world_t_robot.transform.translation.z,
            Quat(
                world_t_robot.transform.rotation.w,
                world_t_robot.transform.rotation.x,
                world_t_robot.transform.rotation.y,
                world_t_robot.transform.rotation.z,
            ),
        ).get_closest_se2_transform()
        goal = SE3Pose(
            world_t_robot.transform.translation.x,
            world_t_robot.transform.translation.y - distance,
            world_t_robot.transform.translation.z,
            Quat(
                world_t_robot.transform.rotation.w,
                world_t_robot.transform.rotation.x,
                world_t_robot.transform.rotation.y,
                world_t_robot.transform.rotation.z,
            ),
        ).get_closest_se2_transform()
        world_t_goal = world_t_robot_se2 * goal
        proto_goal = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=world_t_goal.x,
            goal_y=world_t_goal.y,
            goal_heading=world_t_goal.angle,
            frame_name=VISION_FRAME_NAME,  # use Boston Dynamics' frame conventions
            params=RobotCommandBuilder.mobility_params(stairs_mode=False),
        )
        goal_msg = RobotCommand.Goal()
        convert(proto_goal, goal_msg.command)

        return goal_msg

    def build_spin_command(self, angle: float) -> RobotCommand.Goal:
        """Build a command for Spot to turn in place."""
        body_tform_goal = SE2Pose(x=0.0, y=0.0, angle=angle)
        vision_tform_body = self._tf.lookup_se2_vision_tform_body(
            frame_b=GRAV_ALIGNED_BODY_FRAME_NAME
        )
        vision_tform_goal = vision_tform_body * body_tform_goal
        proto_goal = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=vision_tform_goal.x,
            goal_y=vision_tform_goal.y,
            goal_heading=vision_tform_goal.angle,
            frame_name=VISION_FRAME_NAME,
            params=RobotCommandBuilder.mobility_params(stairs_mode=False),
            # build_on_command=self.blackboard.command,
        )
        goal_msg = RobotCommand.Goal()
        convert(proto_goal, goal_msg.command)

        return goal_msg

    def build_se2_trajectory_point_command(
        self, pose: PoseStamped, frame_name: str = VISION_FRAME_NAME
    ) -> RobotCommand.Goal:
        """Build an SE(2) trajectory point command."""
        proto_goal = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=pose.pose.position.x,
            goal_y=pose.pose.position.y,
            goal_heading=pose.pose.orientation.z,
            frame_name=frame_name,
            params=RobotCommandBuilder.mobility_params(stairs_mode=False),
            # build_on_command=self.blackboard.command,
        )
        goal_msg = RobotCommand.Goal()
        convert(proto_goal, goal_msg.command)

        return goal_msg

    def _execute_wait_action(self, goal_handle: ServerGoalHandle) -> SpotBodyTask.Result:
        """Execute a `wait` predefined action."""
        wait_duration = int(goal_handle.request.options[0])
        feedback_msg = SpotBodyTask.Feedback()
        count = 0
        duration = Duration(seconds=1)
        while count < wait_duration:
            self.get_clock().sleep_for(duration)
            count += 1
            feedback_msg.header.stamp = self.get_clock().now().to_msg()
            feedback_msg.feedback = (
                "Feedback: Action 'wait' still ongoing... "
                f"(Duration: {count} / {wait_duration} seconds)"
            )
            self.get_logger().debug(feedback_msg.feedback)
            goal_handle.publish_feedback(feedback_msg)

        # Set action completion
        goal_handle.succeed()
        result = SpotBodyTask.Result()
        result.header.stamp = self.get_clock().now().to_msg()
        result.success = True
        result.message = "Result: Action 'wait' completed successfully!"
        self.get_logger().debug(result.message)

        return result

    def _execute_body_action(
        self, goal_handle: ServerGoalHandle
    ) -> SpotBodyTask.Result:
        """Execute an asynchronous Spot body action."""
        self._called_action_goal_handle = goal_handle
        action: str = goal_handle.request.action
        goal_msg: RobotCommand.Goal = None
        future: Future = None

        # Create correct action request
        if action in self.BODY_PREDEFINED_ACTIONS:
            if action == "spin":
                goal_msg = self.build_spin_command(
                    goal_handle.request.options[0]  # spin_amount
                )
            elif action == "back_up":
                goal_msg = self.build_back_up_command(
                    goal_handle.request.options[0]  # backup_amount
                )
            elif action == "wait":
                return self._execute_wait_action(goal_handle)
        else:
            goal_msg = self.build_se2_trajectory_point_command(goal_handle.request.goal)

        # Send robot command action
        future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._save_spot_body_action_feedback
        )
        future.add_done_callback(self._get_body_response)

        # Wait for action to complete
        self._action_done_event.wait()

        # Logic for called action completion
        result = SpotBodyTask.Result()
        result.header.stamp = self.get_clock().now().to_msg()
        if self._body_action_result.success:
            goal_handle.succeed()
            result.success = True
            result.message = f"Result: Action '{action}' completed successfully!"
            self.get_logger().debug(result.message)
        else:
            goal_handle.abort()
            result.success = False
            result.message = self._body_action_result.message
            self.get_logger().error(result.message)

        # Reset variables
        self._saved_body_feedback_msg = None

        return result

    def _get_body_response(self, future: Future):
        """Get body response from the action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Body goal from Spot rejected!")
            return

        self.get_logger().info("Body goal accepted!")
        self._pre_action_result = goal_handle.get_result_async()
        self._pre_action_result.add_done_callback(self._get_body_result)

    def _get_body_result(self, future: Future):
        """Get body result from the action server."""
        self._body_action_result = future.result().result
        self._action_done_event.set()  # Release hold on called action callback
        self.get_logger().debug("Body action finished...")

    def _save_spot_body_action_feedback(self, feedback_msg: RobotCommand.Feedback):
        "Save Spot wrapper action feedback message and send back to called action client."
        self._saved_body_feedback_msg = feedback_msg
        status_value = feedback_msg.feedback.feedback.command.synchronized_feedback.mobility_command_feedback.feedback.se2_trajectory_feedback.status.value
        body_motion_status_value = feedback_msg.feedback.feedback.command.synchronized_feedback.mobility_command_feedback.feedback.se2_trajectory_feedback.body_movement_status.value
        final_goal_status_value = feedback_msg.feedback.feedback.command.synchronized_feedback.mobility_command_feedback.feedback.se2_trajectory_feedback.final_goal_status.value
        called_action_feedback_msg = SpotBodyTask.Feedback()
        called_action_feedback_msg.feedback = (
            "Action status:\n"
            f"\tstatus value: {SE2_TRAJ_SPOT_STATUS[status_value]}\n"
            f"\tbody_motion_status: {SE2_TRAJ_BODY_MOVEMENT_STATUS[body_motion_status_value]}\n"
            f"\tfinal_goal_status: {SE2_TRAJ_FINAL_GOAL_STATUS[final_goal_status_value]}"
        )
        self._called_action_goal_handle.publish_feedback(called_action_feedback_msg)
