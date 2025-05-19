"""Random simple search for fiducial planner."""

from __future__ import annotations

import random

from bosdyn.api.world_object_pb2 import WorldObject
from bosdyn.client.frame_helpers import get_a_tform_b
from bosdyn.client.frame_helpers import VISION_FRAME_NAME
from bosdyn.client.math_helpers import SE2Pose

from bosdyn_msgs.conversions import convert

from bosdyn_api_msgs.msg import WorldObject as WorldObjectMsg

from geometry_msgs.msg import PoseStamped

import numpy as np

import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from spot_msgs.action import RobotCommand

from spot_bt_ros_msgs.action import PlanSpotArmTask
from spot_bt_ros_msgs.action import PlanSpotBodyTask

from spot_bt_ros_node.transform import SpotTFBase
from spot_bt_ros_node.utils import get_desired_angle


class RandomSimpleSearchPlanner(Node):
    """A random simple search for fiducial planner."""

    def __init__(self, namespace: str | None = None):
        super().__init__("random_simple_search_planner", namespace=namespace)

        if namespace is not None:
            self._name = f"{namespace}/"
            self._ns = namespace
        else:
            self._name = ""
            self._ns = ""

        self.declare_parameters(
            "",
            [
                ("dock_id", 549),
                ("has_arm", True),
                ("docked", True),
                ("standing", False),
            ],
        )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._tf = SpotTFBase(node=self, namespace=namespace)
        self._dock_id = self.get_parameter("dock_id").get_parameter_value().integer_value
        self._saved_feedback_msg = None

        self._is_rotating = True
        self._is_moving = False
        self._done = False
        self._angle = 0.0
        self._target = None

        # Create action server interface to behavior tree.
        self._action_server: dict[str, ActionServer] = {
            "body": ActionServer(
                self, PlanSpotBodyTask, "spot/nav/planner/body", self._compute_new_pose,
            ),
        }

        # Add arm related planning
        self._has_arm = self.get_parameter("has_arm").get_parameter_value().bool_value
        if self._has_arm:
            self._action_server: dict[str, ActionServer] = {
            "arm": ActionServer(
                self, PlanSpotArmTask, "spot/nav/planner/arm", self._compute_new_arm_pose,
            ),
        }

    def _compute_motion_to_fiducial_pose(self, world_object: WorldObjectMsg) -> tuple[PoseStamped, str]:
        """Compute a robot command to move body to target fiducial."""
        # pylint: disable=no-member,assigning-non-slot
        self.get_logger().info("Compute pose to target fiducial.")
        target_fiducial = WorldObject()
        convert(world_object, target_fiducial)

        # Get vision transform with respect to fiducial.
        vision_tform_fiducial = get_a_tform_b(
            target_fiducial.transforms_snapshot,
            VISION_FRAME_NAME,
            target_fiducial.apriltag_properties.frame_name_fiducial,
        ).to_proto()
        fiducial_rt_world = vision_tform_fiducial.position

        # Compute the go-to point with offsets
        robot_rt_world = self._tf.lookup_se3_vision_tform_body()
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

        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
        goal_pose_msg.pose.position.x = goto_rt_world[0]
        goal_pose_msg.pose.position.y = goto_rt_world[1]
        goal_pose_msg.pose.orientation.z = heading

        return goal_pose_msg, VISION_FRAME_NAME
    
    def _compute_arm_motion_to_fiducial_pose(self, world_object: WorldObjectMsg) -> tuple[PoseStamped, str]:
        """Compute a robot command to move arm to target fiducial."""
        # pylint: disable=no-member,assigning-non-slot
        self.get_logger().info("Compute arm pose to target fiducial.")
        target_fiducial = WorldObject()
        convert(world_object, target_fiducial)

        vision_T_hand = get_a_tform_b(
            target_fiducial.transforms_snapshot,
            VISION_FRAME_NAME,
            target_fiducial.apriltag_properties.frame_name_fiducial,
        )

        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
        goal_pose_msg.pose.position.x = vision_T_hand.x
        goal_pose_msg.pose.position.y = vision_T_hand.y
        goal_pose_msg.pose.position.z = vision_T_hand.z
        goal_pose_msg.pose.orientation.w = vision_T_hand.rot.w
        goal_pose_msg.pose.orientation.x = vision_T_hand.rot.x
        goal_pose_msg.pose.orientation.y = vision_T_hand.rot.y
        goal_pose_msg.pose.orientation.z = vision_T_hand.rot.z

        return goal_pose_msg, VISION_FRAME_NAME

    def _compute_random_search_pose(self) -> tuple[PoseStamped, str]:
        """Create the robot command and convert to a ROS message."""
        self.get_logger().info("Generating next motion point...")
        x = 0.0
        y = 0.0
        if self._is_rotating:
            self._angle = 30.0 * random.uniform(-1.0, 1.0)

        if self._is_moving:
            x = np.cos(self._angle * np.pi / 180.0)
            y = np.sin(self._angle * np.pi / 180.0)
            self._angle = 0.0

        body_tform_goal = SE2Pose(x=x, y=y, angle=self._angle)
        frame_tform_body = self._tf.lookup_se2_vision_tform_grav_body()
        frame_tform_goal = frame_tform_body * body_tform_goal
        self._angle = frame_tform_goal.angle

        # Set to ROS msg
        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
        goal_pose_msg.pose.position.x = frame_tform_goal.x
        goal_pose_msg.pose.position.y = frame_tform_goal.y
        goal_pose_msg.pose.orientation.z = frame_tform_goal.angle

        return goal_pose_msg, VISION_FRAME_NAME

    def _compute_new_pose(self, goal_handle: ServerGoalHandle) -> PlanSpotBodyTask.Result:
        """Compute new pose for random search action."""
        action: str = goal_handle.request.action

        # Compute new pose and send feedback, if needed.
        # TODO make the functions compute functions async
        feedback_msg = PlanSpotBodyTask.Feedback()
        start = self.get_time()
        while True:
            feedback_msg.header.stamp = self.get_clock().now().to_msg()
            goal_handle.publish_feedback(feedback_msg)
            if action == "fiducial":
                goal_pose, frame = self._compute_motion_to_fiducial_pose(
                    goal_handle.request.world_object
                )
            else:
                goal_pose, frame = self._compute_random_search_pose()

            feedback_msg.feedback = (
                    f"Feedback: Computing '{action} pose..."
                    f"(Duration: {self.get_time() - start})"
                )
            self.get_logger().info(feedback_msg.feedback)
            goal_handle.publish_feedback(feedback_msg)
            break

        # Logic for action completion
        goal_handle.succeed()
        result = PlanSpotBodyTask.Result()
        result.header.stamp = self.get_clock().now().to_msg()
        result.success = True
        result.goal_pose = goal_pose
        result.frame = frame
        result.message = f"Result: Computing new '{action}' pose successfully!"
        self.get_logger().info(result.message)

        # Reset variables
        self._saved_feedback_msg = None

        return result

    def _compute_new_arm_pose(self, goal_handle: ServerGoalHandle) -> PlanSpotArmTask.Result:
        """Compute new arm pose for random search action."""
        action: str = goal_handle.request.action

        # Compute new pose and send feedback, if needed.
        # TODO make the functions compute functions async
        feedback_msg = PlanSpotArmTask.Feedback()
        start = self.get_time()
        while True:
            feedback_msg.header.stamp = self.get_clock().now().to_msg()
            goal_handle.publish_feedback(feedback_msg)
            if action == "fiducial":
                goal_pose, frame = self._compute_arm_motion_to_fiducial_pose(
                    goal_handle.request.world_object
                )
            else:
                # TODO Fill this with other tasking or a fallback procedure.
                goal_pose = PoseStamped()
                frame = VISION_FRAME_NAME
                goal_pose.header.stamp = self.get_clock().now().to_msg()


            feedback_msg.feedback = (
                    f"Feedback: Computing '{action} arm pose..."
                    f"(Duration: {self.get_time() - start})"
                )
            self.get_logger().info(feedback_msg.feedback)
            goal_handle.publish_feedback(feedback_msg)
            break

        # Logic for action completion
        goal_handle.succeed()
        result = PlanSpotArmTask.Result()
        result.header.stamp = self.get_clock().now().to_msg()
        result.success = True
        result.goal_pose = goal_pose
        result.frame = frame
        result.message = f"Result: Computing new '{action}' arm pose successfully!"
        self.get_logger().info(result.message)

        # Reset variables
        self._saved_feedback_msg = None

        return result

    def shutdown(self):
        """Close external connections."""
        self._tf.shutdown()

    def get_time(self) -> int:
        """Get current clock time in nanoseconds."""
        return self.get_clock().now().nanoseconds

    def _save_spot_action_feedback(self, feedback_msg: RobotCommand.Feedback):
        "Save Spot Wrapper action feedback message to send back to caller."
        self._saved_feedback_msg = feedback_msg

    @property
    def name(self) -> str:
        """Get node name."""
        return self.get_name()


def main(args=None):
    """Start RandomSimpleSearchPlanner node."""
    rclpy.init(args=args)

    node = RandomSimpleSearchPlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().warning(f"Killing node: {node.name}")
        node.shutdown()
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
