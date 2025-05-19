"""spot_bt_ros_py Parallel composites."""

from __future__ import annotations

from py_trees.common import ParallelPolicy
from py_trees.composites import Parallel

from spot_bt_ros_py.actions.general import create_check_spot_battery_action
from spot_bt_ros_py.actions.general import create_check_spot_metrics_action
from spot_bt_ros_py.actions.general import create_check_spot_wifi_action
from spot_bt_ros_py.actions.perception import create_rgb_camera_children
from spot_bt_ros_py.actions.perception import create_depth_camera_children


def create_spot_status_parallel(
    name: str = "Get Spot Status",
) -> Parallel:
    """
    Create a parallel for getting Spot status information.

    Args:
      name (str): Name given to the parallel control node for Spot status.

    Returns:
      out (py_trees.composites.Parallel): A parallel control node for Spot status.
    """
    parallel = Parallel(name, policy=ParallelPolicy.SuccessOnOne)
    parallel.add_children(
        [
            create_check_spot_metrics_action(),
            create_check_spot_battery_action(),
            create_check_spot_wifi_action(),
        ]
    )

    return parallel


def create_rgb_camera_parallel(
    name: str = "Get RGB Camera Images", has_arm: bool = True
) -> Parallel:
    """
    Create a parallel for getting all images around Spot to the Blackboard.

    Args:
      name (str): Name given to the parallel control node for Spot RGB image storage.
      has_arm (bool): Whether the Spot robot has an arm. This will add an extra child for
        arm camera data.

    Returns:
      out (py_trees.composites.Parallel): A parallel control node for Spot RGB
        image storage.
    """
    parallel = Parallel(name, policy=ParallelPolicy.SuccessOnAll)
    parallel.add_children(create_rgb_camera_children(has_arm))

    return parallel


def create_depth_camera_parallel(
    name: str = "Get Depth Camera Images", has_arm: bool = True
) -> Parallel:
    """
    Create a parallel for getting all images around Spot to the Blackboard.

    Args:
      name (str): Name given to the parallel control node for Spot depth image storage.
      has_arm (bool): Whether the Spot robot has an arm. This will add an extra child for
        arm camera data.

    Returns:
      out (py_trees.composites.Parallel): A parallel control node for Spot depth
        image storage.
    """
    parallel = Parallel(name, policy=ParallelPolicy.SuccessOnAll)
    parallel.add_children(create_depth_camera_children(has_arm))

    return parallel
