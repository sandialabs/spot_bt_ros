from __future__ import annotations

import bdai_ros2_wrappers.tf_listener_wrapper as tfl
from bdai_ros2_wrappers.utilities import namespace_with

from bosdyn.client.frame_helpers import BODY_FRAME_NAME
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from bosdyn.client.frame_helpers import VISION_FRAME_NAME
from bosdyn.client.math_helpers import SE2Pose
from bosdyn.client.math_helpers import SE3Pose

from rclpy.node import Node

from geometry_msgs.msg import TransformStamped

from spot_bt_ros.data import ros_transform_to_se2_pose
from spot_bt_ros.data import ros_transform_to_se3_pose


class TFListenerWrapper(tfl.TFListenerWrapper):
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


class SpotTF:
    def __init__(
        self,
        node: Node,
        namespace: str | None = None,
        frame_a: str = BODY_FRAME_NAME,
        frame_b: str = VISION_FRAME_NAME
    ):
        self._name = f"{namespace}/" if namespace is not None else ""
        self._frame_a_name = namespace_with(self._name, frame_a)
        self._frame_b_name = namespace_with(self._name, frame_b)
        self.wrapper = TFListenerWrapper(node)
        self.wrapper.wait_for_a_tform_b(self._frame_a_name, self._frame_b_name)

    def lookup_odom_tform_body(self, frame_b: str = BODY_FRAME_NAME) -> TransformStamped:
        """Lookup transform from Odom to Body frame"""
        if frame_b == GRAV_ALIGNED_BODY_FRAME_NAME:
            return self.wrapper.lookup_a_tform_b(
                namespace_with(self._name, ODOM_FRAME_NAME),
                namespace_with(self._name, GRAV_ALIGNED_BODY_FRAME_NAME),
            )

        return self.wrapper.lookup_a_tform_b(
            namespace_with(self._name, ODOM_FRAME_NAME),
            namespace_with(self._name, BODY_FRAME_NAME),
        )

    def lookup_se2_odom_tform_body(self, frame_b: str = BODY_FRAME_NAME) -> SE2Pose:
        """Lookup SE2 transform from Odom to Body frame."""
        if frame_b == GRAV_ALIGNED_BODY_FRAME_NAME:
            return self.wrapper.lookup_se2_a_tform_b(
                namespace_with(self._name, ODOM_FRAME_NAME),
                namespace_with(self._name, GRAV_ALIGNED_BODY_FRAME_NAME),
            )

        return self.wrapper.lookup_se2_a_tform_b(
            namespace_with(self._name, ODOM_FRAME_NAME),
            namespace_with(self._name, BODY_FRAME_NAME),
        )

    def lookup_se3_odom_tform_body(self, frame_b: str = BODY_FRAME_NAME) -> SE3Pose:
        """Lookup SE3 transform from Odom to Body frame."""
        if frame_b == GRAV_ALIGNED_BODY_FRAME_NAME:
            return self.wrapper.lookup_se3_a_tform_b(
                namespace_with(self._name, ODOM_FRAME_NAME),
                namespace_with(self._name, GRAV_ALIGNED_BODY_FRAME_NAME),
            )

        return self.wrapper.lookup_se3_a_tform_b(
            namespace_with(self._name, ODOM_FRAME_NAME),
            namespace_with(self._name, BODY_FRAME_NAME),
        )

    def lookup_vision_tform_body(self, frame_b: str = BODY_FRAME_NAME) -> TransformStamped:
        """Lookup transform from vision to Body frame"""
        if frame_b == GRAV_ALIGNED_BODY_FRAME_NAME:
            return self.wrapper.lookup_a_tform_b(
                namespace_with(self._name, VISION_FRAME_NAME),
                namespace_with(self._name, GRAV_ALIGNED_BODY_FRAME_NAME),
            )

        return self.wrapper.lookup_a_tform_b(
            namespace_with(self._name, VISION_FRAME_NAME),
            namespace_with(self._name, BODY_FRAME_NAME),
        )

    def lookup_se2_vision_tform_body(self, frame_b: str = BODY_FRAME_NAME) -> SE2Pose:
        """Lookup SE2 transform from Vision to Body frame."""
        if frame_b == GRAV_ALIGNED_BODY_FRAME_NAME:
            return self.wrapper.lookup_se2_a_tform_b(
                namespace_with(self._name, VISION_FRAME_NAME),
                namespace_with(self._name, GRAV_ALIGNED_BODY_FRAME_NAME),
            )

        return self.wrapper.lookup_se2_a_tform_b(
            namespace_with(self._name, VISION_FRAME_NAME),
            namespace_with(self._name, BODY_FRAME_NAME),
        )

    def lookup_se3_vision_tform_body(self, frame_b: str = BODY_FRAME_NAME) -> SE3Pose:
        """Lookup SE3 transform from Vision to Body frame."""
        if frame_b == GRAV_ALIGNED_BODY_FRAME_NAME:
            return self.wrapper.lookup_se3_a_tform_b(
                namespace_with(self._name, VISION_FRAME_NAME),
                namespace_with(self._name, GRAV_ALIGNED_BODY_FRAME_NAME),
            )

        return self.wrapper.lookup_se3_a_tform_b(
            namespace_with(self._name, VISION_FRAME_NAME),
            namespace_with(self._name, BODY_FRAME_NAME),
        )