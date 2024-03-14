from __future__ import annotations

from cv_bridge import CvBridge

import py_trees

import py_trees_ros

import rclpy

from sensor_msgs.msg import Image

from spot_bt_ros.cameras import (
    ROS_DEPTH_CAMERA_TOPICS,
    ROS_DEPTH_CAMERA_TOPICS_WITH_ARM,
)
from spot_bt_ros.data import Blackboards


class SaveDepthImage(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name: str,
        image_source: str | None = None,
    ):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.bridge = None
        self.depth_image = None
        # TODO: Add option if image_source is None.
        self.image_src = image_source
        self.robot = None

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [TakeDepthImage::initialise()]")
        self.bridge = CvBridge()
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.perception = self.attach_blackboard_client("Perception")
        self.blackboard.perception.register_key(
            key="depth_images", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot

    def update(self) -> py_trees.common.Status:
        """Run the TakeDepthImage behavior when ticked."""
        self.logger.debug(f"  {self.name} [TakeDepthImage::update()]")
        try:
            self.robot.get_logger().info(f"Saving {self.image_src} depth image.")
            image_msg = self.robot.images.depth[self.image_src]
            image_cv = self.bridge.imgmsg_to_cv2(
                image_msg, desired_encoding="passthrough"
            )
            self.depth_image = image_cv

        except:  # pylint: disable=bare-except
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.blackboard.perception.depth_images[self.image_src] = self.depth_image
        self.logger.debug(
            f" {self.name} [TakeDepthImage::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class TakeDepthImageAllCameras(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name: str,
    ):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.bridge = None
        self.depth_images = {}

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [TakeDepthImageAllCameras::initialise()]")
        self.bridge = CvBridge()
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.perception = self.attach_blackboard_client("Perception")
        self.blackboard.perception.register_key(
            key="depth_images", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot

    def update(self) -> py_trees.common.Status:
        """Run the TakeImageAllCameras behavior when ticked."""
        self.logger.debug(f"  {self.name} [TakeDepthImageAllCameras::update()]")
        try:
            for msg, src in self.robot.images.depth.items():
                self.robot.get_logger().info(f"Saving {src} depth image.")
                image_cv = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding="passthrough"
                )
                self.depth_images[src] = image_cv

        except:  # pylint: disable=bare-except
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.blackboard.perception.depth_images = self.depth_images
        self.logger.debug(
            f" {self.name} [TakeDepthImageAllCameras::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


def create_depth_camera_children(has_arm: False) -> list[py_trees_ros.subscriber.ToBlackboard]:
    """Create a list of depth camera ToBlackboard objects."""
    if has_arm:
        return [
            py_trees_ros.subscribers.ToBlackboard(
                name=camera,
                topic_name=topic,
                topic_type=Image,
                qos_profile=rclpy.qos.ReliabilityPolicy,
                blackboard_variables={f"camera/depth/{camera}": None}
            )
            for camera, topic in ROS_DEPTH_CAMERA_TOPICS_WITH_ARM.items()
        ]
    
    return [
        py_trees_ros.subscribers.ToBlackboard(
            name=camera,
            topic_name=topic,
            topic_type=Image,
            qos_profile=rclpy.qos.ReliabilityPolicy,
            blackboard_variables={f"camera/depth/{camera}": None}
        )
        for camera, topic in ROS_DEPTH_CAMERA_TOPICS.items()
    ]
