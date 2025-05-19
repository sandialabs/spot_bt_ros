"""Useful general sub-behaviors for Spot."""

from __future__ import annotations

from py_trees.composites import Sequence

from spot_bt_ros_py.composites.selector import create_dock_selector
from spot_bt_ros_py.composites.selector import create_lease_claim_selector
from spot_bt_ros_py.composites.selector import create_lease_release_selector
from spot_bt_ros_py.composites.selector import create_power_off_selector
from spot_bt_ros_py.composites.selector import create_power_on_selector
from spot_bt_ros_py.composites.selector import create_sitting_selector
from spot_bt_ros_py.composites.selector import create_standing_selector
from spot_bt_ros_py.composites.selector import create_undock_selector


def undock() -> Sequence:
    """Undocking sub-behavior for Spot."""
    behavior = Sequence("Undock Routine", memory=False)
    behavior.add_children(
        [
            create_lease_claim_selector(),
            create_power_on_selector(),
            create_undock_selector(),
        ]
    )

    return behavior


def dock() -> Sequence:
    """Docking sub-behavior for Spot."""
    behavior = Sequence("Dock Routine", memory=False)
    behavior.add_children(
        [
            create_dock_selector(),
            create_power_off_selector(),
            create_lease_release_selector(),
        ]
    )

    return behavior


def sit_and_power_off() -> Sequence:
    """Make Spot Sit and Shutdown properly sub-behavior."""
    behavior = Sequence("Sit and Power Off", memory=False)
    behavior.add_children(
        [
            create_sitting_selector(),
            create_power_off_selector(),
            create_lease_release_selector(),
        ]
    )

    return behavior


def power_on_and_stand() -> Sequence:
    """Power on Spot and have him stand sub-beahvior."""
    behavior = Sequence("Power On and Stand", memory=False)
    behavior.add_children(
        [
            create_lease_claim_selector(),
            create_power_on_selector(),
            create_standing_selector(),
        ]
    )

    return behavior
