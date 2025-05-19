"""ROS 2 Transform wrappers for Spot."""

from __future__ import annotations

from bosdyn.client.frame_helpers import BODY_FRAME_NAME
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from bosdyn.client.frame_helpers import VISION_FRAME_NAME
from bosdyn.client.math_helpers import SE2Pose
from bosdyn.client.math_helpers import SE3Pose

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

import synchros2.tf_listener_wrapper as tfl
from synchros2.utilities import namespace_with

from geometry_msgs.msg import TransformStamped

from spot_bt_ros_node.data import ros_transform_to_se2_pose
from spot_bt_ros_node.data import ros_transform_to_se3_pose


class TFListenerWrapper(tfl.TFListenerWrapper):
    """Wrapper for 2D (SE(2)) and 3D (SE(3)) homogeneous transformation matrix transforms."""
    def lookup_se3_a_tform_b(
        self,
        frame_a: str,
        frame_b: str,
        transform_time: float | None = None,
        timeout: float | None = None,
        wait_for_frames: bool = False,
    ) -> SE3Pose:
        """
        Parameters:
            frame_a: Base frame for transform.  The transform returned will be
                frame_a_t_frame_b
            frame_b: Tip frame for transform.  The transform returned will be
                frame_a_t_frame_b
            transform_time: The time at which to look up the transform.  If left
                at None, will be the most recent transform available
            timeout: The time to wait for the transform to become available if
                the transform_time is beyond the most recent transform in the
                buffer.
            wait_for_frames: If true, waits timeout amount of time for a path to
                exist from frame_a to frame_b in the buffer. If false, this will
                return immediately if a path does not exist even if timeout is not
                None.  Note that wait_for_transform can be used to wait indefinitely
                for a transform to become available.
        Returns:
            The transform frame_a_t_frame_b at the time specified.
        Raises:
            All the possible TransformExceptions.
        """
        return ros_transform_to_se3_pose(
            tfl.TFListenerWrapper.lookup_a_tform_b(
                self, frame_a, frame_b, transform_time, timeout, wait_for_frames
            ).transform
        )

    def lookup_se2_a_tform_b(
        self,
        frame_a: str,
        frame_b: str,
        transform_time: float | None = None,
        timeout: float | None = None,
        wait_for_frames: bool = False,
    ) -> SE2Pose:
        """
        Parameters:
            frame_a: Base frame for transform.  The transform returned will be frame_a_t_frame_b
            frame_b: Tip frame for transform.  The transform returned will be frame_a_t_frame_b
            transform_time: The time at which to look up the transform.  If left at None, will be the most recent
                transform available
            timeout: The time to wait for the transform to become available if the transform_time is beyond the most
                recent transform in the buffer.
            wait_for_frames: If true, waits timeout amount of time for a path to exist from frame_a to frame_b in the
                the buffer.  If false, this will return immediately if a path does not exist even if timeout is not
                None.  Note that wait_for_transform can be used to wait indefinitely for a transform to become
                available.
        Returns:
            The transform frame_a_t_frame_b at the time specified.
        Raises:
            All the possible TransformExceptions.
        """
        return ros_transform_to_se2_pose(
            tfl.TFListenerWrapper.lookup_a_tform_b(
                self, frame_a, frame_b, transform_time, timeout, wait_for_frames
            ).transform
        )

class SpotTFMixin:
    """Convenience functions for Transforms."""

    def lookup_odom_tform_body(self) -> TransformStamped:
        """Lookup transform from Odom to Body frame"""
        transform = TransformStamped()
        if self.wrapper.wait_for_a_tform_b(
            self._odom_frame, self._body_frame, timeout_sec=0.0
        ):
            transform = self.wrapper.lookup_a_tform_b(
                self._odom_frame, self._body_frame
            )

        return transform

    def lookup_odom_tform_grav_body(self) -> TransformStamped:
        """Lookup transform from Odom to Grav-Aligned Body frame"""
        transform = TransformStamped()
        if self.wrapper.wait_for_a_tform_b(
            self._odom_frame, self._grav_body_frame, timeout_sec=0.0
        ):
            transform = self.wrapper.lookup_a_tform_b(
                self._odom_frame, self._grav_body_frame
            )

        return transform

    def lookup_se2_odom_tform_body(self) -> SE2Pose | None:
        """Lookup SE(2) transform from Odom to Body frame."""
        transform: SE2Pose = None
        if self.wrapper.wait_for_a_tform_b(
            self._odom_frame, self._body_frame, timeout_sec=0.0
        ):
            transform = self.wrapper.lookup_se2_a_tform_b(
                self._odom_frame, self._body_frame
            )

        return transform

    def lookup_se2_odom_tform_grav_body(self) -> SE2Pose | None:
        """Lookup SE(2) transform from Odom to Body frame."""
        transform: SE2Pose = None
        if self.wrapper.wait_for_a_tform_b(
            self._odom_frame, self._grav_body_frame, timeout_sec=0.0
        ):
            transform = self.wrapper.lookup_se2_a_tform_b(
                self._odom_frame, self._grav_body_frame
            )

        return transform

    def lookup_se3_odom_tform_body(self) -> SE3Pose | None:
        """Lookup SE(3) transform from Odom to Body frame."""
        transform: SE3Pose = None
        if self.wrapper.wait_for_a_tform_b(
            self._odom_frame, self._body_frame, timeout_sec=0.0
        ):
            transform = self.wrapper.lookup_se3_a_tform_b(
                self._odom_frame,
                self._body_frame,
            )

        return transform

    def lookup_se3_odom_tform_grav_body(self) -> SE3Pose:
        """Lookup SE(3) transform from Odom to Grav-Aligned Body frame."""
        transform: SE3Pose = None
        if self.wrapper.wait_for_a_tform_b(
            self._odom_frame, self._grav_body_frame, timeout_sec=0.0
        ):
            transform = self.wrapper.lookup_se3_a_tform_b(
                self._odom_frame,
                self._grav_body_frame,
            )

        return transform

    def lookup_vision_tform_body(self) -> TransformStamped:
        """Lookup transform from vision to Body frame"""
        transform = TransformStamped()
        if self.wrapper.wait_for_a_tform_b(
            self._vision_frame, self._body_frame, timeout_sec=0.0
        ):
            transform = self.wrapper.lookup_a_tform_b(
                self._vision_frame, self._body_frame
            )

        return transform

    def lookup_vision_tform_grav_body(self) -> TransformStamped:
        """Lookup transform from vision to Grav-Aligned Body frame"""
        transform = TransformStamped()
        if self.wrapper.wait_for_a_tform_b(
            self._vision_frame, self._grav_body_frame, timeout_sec=0.0
        ):
            transform = self.wrapper.lookup_a_tform_b(
                self._vision_frame, self._grav_body_frame
            )

        return transform

    def lookup_se2_vision_tform_body(self) -> SE2Pose | None:
        """Lookup SE(2) transform from Vision to Body frame."""
        transform: SE2Pose = None
        if self.wrapper.wait_for_a_tform_b(
            self._vision_frame, self._body_frame, timeout_sec=0.0
        ):
            transform = self.wrapper.lookup_se2_a_tform_b(
                self._vision_frame, self._body_frame
            )

        return transform

    def lookup_se2_vision_tform_grav_body(self) -> SE2Pose | None:
        """Lookup SE(2) transform from Vision to Grav-Aligned Body frame."""
        transform: SE2Pose = None
        if self.wrapper.wait_for_a_tform_b(
            self._vision_frame, self._grav_body_frame, timeout_sec=0.0
        ):
            transform = self.wrapper.lookup_se2_a_tform_b(
                self._vision_frame, self._grav_body_frame
            )

        return transform

    def lookup_se3_vision_tform_body(self) -> SE3Pose | None:
        """Lookup SE(3) transform from Vision to Body frame."""
        transform: SE3Pose = None
        if self.wrapper.wait_for_a_tform_b(
            self._vision_frame, self._body_frame, timeout_sec=0.0
        ):
            transform = self.wrapper.lookup_se3_a_tform_b(
                self._vision_frame,
                self._body_frame,
            )

        return transform

    def lookup_se3_vision_tform_grav_body(self) -> SE3Pose | None:
        """Lookup SE(3) transform from Vision to Body frame."""
        transform: SE3Pose = None
        if self.wrapper.wait_for_a_tform_b(
            self._vision_frame, self._grav_body_frame, timeout_sec=0.0
        ):
            transform = self.wrapper.lookup_se3_a_tform_b(
                self._vision_frame,
                self._grav_body_frame,
            )

        return transform


class SpotTFBase(SpotTFMixin):
    """Base Spot Transform Wrapper."""

    def __init__(
        self,
        node: Node,
        namespace: str | None = None
    ):
        self._name = f"{namespace}/" if namespace is not None else ""
        self._body_frame = namespace_with(self._name, BODY_FRAME_NAME)
        self._grav_body_frame = namespace_with(self._name, GRAV_ALIGNED_BODY_FRAME_NAME)
        self._odom_frame = namespace_with(self._name, ODOM_FRAME_NAME)
        self._vision_frame = namespace_with(self._name, VISION_FRAME_NAME)
        self.wrapper = TFListenerWrapper(node)

    def shutdown(self):
        """Shutdown TFListener wrapper."""
        self.wrapper.shutdown()


class SpotTF(Node, SpotTFMixin):
    """Spot Transform Wrapper Node."""

    def __init__(
        self,
        namespace: str | None = None,
    ):
        super().__init__("spot_tf_node")
        self._name = f"{namespace}/" if namespace is not None else ""
        self._body_frame = namespace_with(self._name, BODY_FRAME_NAME)
        self._grav_body_frame = namespace_with(self._name, GRAV_ALIGNED_BODY_FRAME_NAME)
        self._odom_frame = namespace_with(self._name, ODOM_FRAME_NAME)
        self._vision_frame = namespace_with(self._name, VISION_FRAME_NAME)
        self.wrapper = TFListenerWrapper(self)

    @property
    def name(self) -> str:
        """Get node name."""
        return self.get_name()

    def shutdown(self):
        """Shutdown TFListener wrapper."""
        self.wrapper.shutdown()


def main(args=None):
    """Start SpotTF node."""
    rclpy.init(args=args)

    node = SpotTF()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().warning(f"Killing node: {node.name}")

    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
