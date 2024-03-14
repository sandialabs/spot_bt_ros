from __future__ import annotations

from datetime import datetime

import cv2

from cv_bridge import CvBridge

import py_trees

import py_trees_ros

import rclpy

from sensor_msgs.msg import Image

from spot_bt_ros.data import Blackboards
from spot_bt_ros.cameras import (
    IMAGE_CAMERA_OPTIONS,
    IMAGE_CAMERA_OPTIONS_WITH_ARM,
    ROS_IMAGE_CAMERA_TOPICS,
    ROS_IMAGE_CAMERA_TOPICS_WITH_ARM,
)


class SaveImage(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name: str,
        image_source: str | None = None,
    ):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.bridge = None
        self.rgb_image = None
        # TODO: Add option if image_source is None.
        self.image_src = image_source
        self.robot = None

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [SaveImage::initialise()]")
        self.bridge = CvBridge()
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.perception = self.attach_blackboard_client("Perception")
        self.blackboard.perception.register_key(
            key="rgb_images", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot

    def update(self) -> py_trees.common.Status:
        """Run the TakeImage behavior when ticked."""
        self.logger.debug(f"  {self.name} [SaveImage::update()]")
        try:
            self.robot.get_logger().info(f"Saving {self.image_src} RGB image.")
            image_msg = self.robot.images.rgb[self.image_src]
            image_cv = self.bridge.imgmsg_to_cv2(
                image_msg, desired_encoding="passthrough"
            )
            self.rgb_image = image_cv

        except:  # pylint: disable=bare-except
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.blackboard.perception.rgb_images[self.image_src] = self.rgb_image
        self.logger.debug(
            f" {self.name} [SaveImage::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class SaveImageToDrive(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name: str,
        image_source: str | None = None,
    ):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.bridge = None
        self.rgb_image = None
        # TODO: Add option if image_source is None.
        self.image_src = image_source
        self.robot = None

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [SaveImageToDrive::initialise()]")
        self.bridge = CvBridge()
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.perception = self.attach_blackboard_client("Perception")
        self.blackboard.perception.register_key(
            key="rgb_images", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot

    def update(self) -> py_trees.common.Status:
        """Run the TakeImage behavior when ticked."""
        self.logger.debug(f"  {self.name} [SaveImageToDrive::update()]")
        try:
            self.robot.get_logger().info(f"Saving {self.image_src} RGB image.")
            image_msg = self.robot.images.rgb[self.image_src]
            image_cv = self.bridge.imgmsg_to_cv2(
                image_msg, desired_encoding="passthrough"
            )
            self.rgb_image = image_cv
            timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
            cv2.imwrite(
                f"/home/zmkakis/{timestamp}_{self.image_src}.jpg", image_cv
            )
        except:  # pylint: disable=bare-except
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.blackboard.perception.rgb_images[self.image_src] = self.rgb_image
        self.logger.debug(
            f" {self.name} [SaveImageToDrive::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class SaveImageAllCameras(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name: str,
        pixel_format: str = "PIXEL_FORMAT_RGB_U8",
        use_arm: bool = False,
    ):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.bridge = None
        self.rgb_images = {}
        if use_arm:
            self.image_sources = IMAGE_CAMERA_OPTIONS_WITH_ARM
        else:
            self.image_sources = IMAGE_CAMERA_OPTIONS

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [SaveImageAllCameras::initialise()]")
        self.bridge = CvBridge()
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.perception = self.attach_blackboard_client("Perception")
        self.blackboard.perception.register_key(
            key="rgb_images", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot

    def update(self) -> py_trees.common.Status:
        """Run the TakeImageAllCameras behavior when ticked."""
        self.logger.debug(f"  {self.name} [SaveImageAllCameras::update()]")
        try:
            for msg, src in self.robot.images.rgb.items():
                self.robot.get_logger().info(f"Saving {src} RGB image.")
                image_cv = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding="passthrough"
                )
                self.rgb_images[src] = image_cv

        except:  # pylint: disable=bare-except
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.blackboard.perception.rgb_images = self.rgb_images
        self.logger.debug(
            f" {self.name} [SaveImageAllCameras::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class SaveImageAllCamerasToDrive(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name: str,
        pixel_format: str = "PIXEL_FORMAT_RGB_U8",
        use_arm: bool = False,
    ):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.bridge = None
        self.rgb_images = {}

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [SaveImageAllCameras::initialise()]")
        self.bridge = CvBridge()
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.perception = self.attach_blackboard_client("Perception")
        self.blackboard.perception.register_key(
            key="rgb_images", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot

    def update(self) -> py_trees.common.Status:
        """Run the TakeImageAllCameras behavior when ticked."""
        self.logger.debug(f"  {self.name} [SaveImageAllCameras::update()]")
        try:
            timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
            for msg, src in self.robot.images.rgb.items():
                self.robot.get_logger().info(f"Saving {src} RGB image.")
                image_cv = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding="passthrough"
                )
                self.rgb_images[src] = image_cv
                cv2.imwrite(f"/home/zmkakis/{timestamp}_{src}.jpg", image_cv)

        except:  # pylint: disable=bare-except
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.blackboard.perception.rgb_images = self.rgb_images
        self.logger.debug(
            f" {self.name} [SaveImageAllCameras::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


def create_rbg_camera_children(has_arm: False) -> list[py_trees_ros.subscriber.ToBlackboard]:
    """Create a list of rgb camera ToBlackboard objects."""
    if has_arm:
        return [
            py_trees_ros.subscribers.ToBlackboard(
                name=camera,
                topic_name=topic,
                topic_type=Image,
                qos_profile=rclpy.qos.ReliabilityPolicy,
                blackboard_variables={f"camera/rgb/{camera}": None}
            )
            for camera, topic in ROS_IMAGE_CAMERA_TOPICS_WITH_ARM.items()
        ]
    
    return [
        py_trees_ros.subscribers.ToBlackboard(
            name=camera,
            topic_name=topic,
            topic_type=Image,
            qos_profile=rclpy.qos.ReliabilityPolicy,
            blackboard_variables={f"camera/rgb/{camera}": None}
        )
        for camera, topic in ROS_IMAGE_CAMERA_TOPICS.items()
    ]
