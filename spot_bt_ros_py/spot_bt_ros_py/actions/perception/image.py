"""RGB camera related subscribers"""

from __future__ import annotations

from py_trees_ros import subscribers

from rclpy.qos import ReliabilityPolicy

from sensor_msgs.msg import Image

from spot_bt_ros_node.cameras import ROS_IMAGE_CAMERA_TOPICS
from spot_bt_ros_node.cameras import ROS_IMAGE_CAMERA_TOPICS_WITH_ARM


def create_rgb_camera_children(
    has_arm: False,
) -> list[subscribers.ToBlackboard]:
    """
    Create a list of RGB camera ToBlackboard objects.

    Args:
      has_arm (bool): Whether the current Spot model used with this code base has an
        arm. This will add one additional RGB camera topic.

    Returns:
      out (list[py_trees_ros.subscribers.ToBlackboard]): A list of actions that capture
        RGB images for all of Spot's cameras.
    """
    if has_arm:
        return [
            subscribers.ToBlackboard(
                name="images",
                topic_name=topic,
                topic_type=Image,
                qos_profile=ReliabilityPolicy.BEST_EFFORT,
                blackboard_variables={f"camera_rgb_{camera}": None},
            )
            for camera, topic in ROS_IMAGE_CAMERA_TOPICS_WITH_ARM.items()
        ]

    return [
        subscribers.ToBlackboard(
            name="images",
            topic_name=topic,
            topic_type=Image,
            qos_profile=ReliabilityPolicy.BEST_EFFORT,
            blackboard_variables={f"camera_rgb_{camera}": None},
        )
        for camera, topic in ROS_IMAGE_CAMERA_TOPICS.items()
    ]
