"""spot_bt_ros_py Sequence composites."""

from __future__ import annotations

from py_trees.composites import Sequence

from spot_bt_ros_py.actions.general import create_check_spot_battery_action
from spot_bt_ros_py.actions.general import create_check_spot_metrics_action
from spot_bt_ros_py.actions.general import create_check_spot_wifi_action
from spot_bt_ros_py.actions.general import Dock
from spot_bt_ros_py.actions.general import PowerOff
from spot_bt_ros_py.actions.general import PowerOn
from spot_bt_ros_py.actions.general import ReleaseLease
from spot_bt_ros_py.actions.general import Undock

from spot_bt_ros_py.actions.perception import create_rgb_camera_children
from spot_bt_ros_py.actions.perception import create_depth_camera_children


def create_dock_sequence(
    name: str = "Dock and Turn Off", memory: bool = True, release_lease: bool = True
) -> Sequence:
    """
    Create a generic Dock Sequence for Spot.

    Args:
      name (str): Name given to the sequence directing docking logic.
      memory (bool): Resume with the status RUNNING child rather than a tick going through
        all actions/conditions in the sequence.
      release_lease (bool): Add an action to release the Spot lease once shut down.

    Returns:
      out (py_trees.composites.Sequence): A sequence of docking actions.
    """

    sequence = Sequence(name, memory=memory)
    sequence.add_children(
        [
            Dock(name="Dock Spot"),
            PowerOff(name="Power OFF Spot"),
        ]
    )
    if release_lease:
        sequence.add_child(ReleaseLease("Release Spot Lease"))

    return sequence


def create_undock_sequence(
    name: str = "Turn On and Undock", memory: bool = True
) -> Sequence:
    """
    Create a generic Undock Sequence for Spot.

    Args:
      name (str): Name given to the sequence directing docking logic.
      memory (bool): Resume with the status RUNNING child rather than a tick going through
        all actions/conditions in the sequence.

    Returns:
      out (py_trees.composites.Sequence): A sequence of undocking actions.
    """
    sequence = Sequence(name, memory=memory)
    sequence.add_children(
        [
            PowerOn(name="Power ON Spot"),
            Undock(name="Undock Spot"),
        ]
    )

    return sequence


def create_spot_status_sequence(
    name: str = "Get Spot Status", memory: bool = False
) -> Sequence:
    """
    Create a sequence for getting Spot status information.

    Args:
      name (str): Name given to the sequence directing docking logic.
      memory (bool): Resume with the status RUNNING child rather than a tick going through
        all actions/conditions in the sequence.

    Returns:
      out (py_trees.composites.Sequence): A sequence of Spot status actions.
    """
    sequence = Sequence(name, memory=memory)
    sequence.add_children(
        [
            create_check_spot_metrics_action(),
            create_check_spot_battery_action(),
            create_check_spot_wifi_action(),
        ]
    )

    return sequence


def create_rgb_camera_sequence(
    name: str = "Get RGB Camera Images", memory: bool = True, has_arm: bool = True
) -> Sequence:
    """
    Create a sequence for getting all images around Spot to the Blackboard.

    Args:
      name (str): Name given to the sequence directing docking logic.
      memory (bool): Resume with the status RUNNING child rather than a tick going through
        all actions/conditions in the sequence.
      has_arm (bool): Whether the Spot robot has an arm. This will add an extra child for
        arm camera data.

    Returns:
      out (py_trees.composites.Sequence): A sequence of Spot RGB camera storage actions.
    """
    sequence = Sequence(name, memory=memory)
    sequence.add_children(create_rgb_camera_children(has_arm))

    return sequence


def create_depth_camera_sequence(
    name: str = "Get Depth Camera Images", memory: bool = True, has_arm: bool = True
) -> Sequence:
    """
    Create a sequence for getting all images around Spot to the Blackboard.

    Args:
      name (str): Name given to the sequence directing docking logic.
      memory (bool): Resume with the status RUNNING child rather than a tick going through
        all actions/conditions in the sequence.
      has_arm (bool): Whether the Spot robot has an arm. This will add an extra child for
        arm camera data.

    Returns:
      out (py_trees.composites.Sequence): A sequence of Spot depth camera storage actions.
    """
    sequence = Sequence(name, memory=memory)
    sequence.add_children(create_depth_camera_children(has_arm))

    return sequence
