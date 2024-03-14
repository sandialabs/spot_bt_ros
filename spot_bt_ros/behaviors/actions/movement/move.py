"""Robot Motion behaviors."""
from __future__ import annotations

from bosdyn.api.spot import robot_command_pb2
from bosdyn.api.world_object_pb2 import WorldObject
from bosdyn.client.frame_helpers import get_a_tform_b
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME
from bosdyn.client.frame_helpers import VISION_FRAME_NAME
from bosdyn.client.math_helpers import SE2Pose
from bosdyn.client.robot_command import RobotCommandBuilder

import numpy as np

import py_trees

from spot_driver import conversions

from spot_msgs.action import RobotCommand

from spot_bt_ros.data import Blackboards
from spot_bt_ros.data import Pose
from spot_bt_ros.utils import get_desired_angle
from spot_bt_ros.utils import get_default_mobility_parameters


class MoveToFiducial(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.tf = None
        self.fiducials = None
        self.pose = None
        self.dock_id = None

    def initialise(self):
        """Initialize robot and client objects for behavior on first tick."""
        self.logger.debug(f"  {self.name} [MoveToFiducial::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.state.register_key(
            key="tf_vision", access=py_trees.common.Access.WRITE
        )
        self.blackboard.state.register_key(
            key="dock_id", access=py_trees.common.Access.WRITE
        )
        self.blackboard.perception = self.attach_blackboard_client("Perception")
        self.blackboard.perception.register_key(
            key="fiducials", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot
        self.tf = self.blackboard.state.tf_vision
        self.dock_id = self.blackboard.state.dock_id
        self.fiducials = self.blackboard.perception.fiducials

    def update(self) -> py_trees.common.Status:
        """Run the MoveToFiducial behavior when ticked."""
        self.logger.debug(f"  {self.name} [MoveToFiducial::update()]")
        if len(self.fiducials) > 1:
            # Check which fiducial is not a dock
            for fiducial in self.fiducials:
                if int(self.dock_id) != fiducial.apriltag_properties.tag_id:
                    target_fiducial = WorldObject()
                    conversions.convert_bosdyn_msgs_world_object_to_proto(
                        fiducial, target_fiducial
                    )
        else:
            target_fiducial = WorldObject()
            conversions.convert_bosdyn_msgs_world_object_to_proto(
                self.fiducials[0], target_fiducial
            )

        # Get vision transform with respect to fiducial.
        vision_tform_fiducial = get_a_tform_b(
            target_fiducial.transforms_snapshot, VISION_FRAME_NAME,
            target_fiducial.apriltag_properties.frame_name_fiducial).to_proto()
        fiducial_rt_world = vision_tform_fiducial.position

        # Compute the go-to point with offsets
        robot_rt_world = self.tf.lookup_se3_vision_tform_body()
        robot_to_fiducial_ewrt_world = np.array([
            fiducial_rt_world.x - robot_rt_world.x,
            fiducial_rt_world.y - robot_rt_world.y,
            0,
        ])
        robot_to_fiducial_ewrt_world_norm = robot_to_fiducial_ewrt_world / np.linalg.norm(
            robot_to_fiducial_ewrt_world
        )
        heading = get_desired_angle(robot_to_fiducial_ewrt_world_norm)
        goto_rt_world = np.array([
            fiducial_rt_world.x - robot_to_fiducial_ewrt_world_norm[0] * 1.0,
            fiducial_rt_world.y - robot_to_fiducial_ewrt_world_norm[1] * 1.0,
        ])

        # Set mobility parameters
        # TODO: Make this into a static function possibly.
        mobility_parameters = get_default_mobility_parameters()

        # Build command, convert, and send to Spot
        proto_goal = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=goto_rt_world[0],
            goal_y=goto_rt_world[1],
            goal_heading=heading,
            frame_name=VISION_FRAME_NAME,
            params=mobility_parameters,
            body_height=0.0,
            locomotion_hint=robot_command_pb2.HINT_AUTO,
        )
        action_goal = RobotCommand.Goal()
        conversions.convert_proto_to_bosdyn_msgs_robot_command(
            proto_goal, action_goal.command
        )
        self.robot.command_client.send_goal_and_wait("move_to_fiducial", action_goal)

        self.robot.get_logger().info("Successfully moved to Fiducial marker.")
        return py_trees.common.Status.SUCCESS


    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f" {self.name} [MoveToFiducial::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class TurnInPlace(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.tf = None
        self.pose: Pose = None
        self.cmd_id = None

    def initialise(self):
        """Initialize robot and client objects for behavior on first tick."""
        self.logger.debug(f"  {self.name} [TurnInPlace::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.state.register_key(
            key="tf_vision", access=py_trees.common.Access.WRITE
        )
        self.blackboard.state.register_key(
            key="pose", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot
        self.tf = self.blackboard.state.tf_vision
        self.pose = self.blackboard.state.pose

    def update(self) -> py_trees.common.Status:
        """Run the TurnInPlace behavior when ticked."""
        self.logger.debug(f"  {self.name} [TurnInPlace::update()]")
        try:
            body_tform_goal = SE2Pose(x=0.0, y=0.0, angle=self.pose.angle)
            vision_tform_body = self.tf.wrapper.lookup_se2_vision_tform_body(
                frame_b=GRAV_ALIGNED_BODY_FRAME_NAME
            )
            vision_tform_goal = vision_tform_body * body_tform_goal
            proto_goal = RobotCommandBuilder.synchro_se2_trajectory_point_command(
                goal_x=vision_tform_goal.x,
                goal_y=vision_tform_goal.y,
                goal_heading=vision_tform_goal.angle,
                frame_name=VISION_FRAME_NAME,
                params=RobotCommandBuilder.mobility_params(stair_hint=False),
            )
            action_goal = RobotCommand.Goal()
            conversions.convert_proto_to_bosdyn_msgs_robot_command(
                proto_goal, action_goal.command
            )
            self.robot.command_client.send_goal_and_wait("turn_in_place", action_goal)

        except:  # pylint: disable=bare-except
            self.robot.get_logger().error("Failed to rotate.")
            return py_trees.common.Status.FAILURE

        self.robot.get_logger().info("Successfully rotated!")
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f" {self.name} [TurnInPlace::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )
