"""spot_bt_ros_py Selector composites."""

from __future__ import annotations

from py_trees.composites import Selector

from spot_bt_ros_py.actions.general import ClaimLease
from spot_bt_ros_py.actions.general import Dock
from spot_bt_ros_py.actions.general import ReleaseLease
from spot_bt_ros_py.actions.general import PowerOff
from spot_bt_ros_py.actions.general import PowerOn
from spot_bt_ros_py.actions.general import Undock
from spot_bt_ros_py.actions.movement import Sit
from spot_bt_ros_py.actions.movement import Stand
from spot_bt_ros_py.actions.perception import DetectFiducialMarkers
from spot_bt_ros_py.conditions.general import IsDocked
from spot_bt_ros_py.conditions.general import IsLeaseClaimed
from spot_bt_ros_py.conditions.general import IsLeaseReleased
from spot_bt_ros_py.conditions.general import IsPoweredOff
from spot_bt_ros_py.conditions.general import IsPoweredOn
from spot_bt_ros_py.conditions.general import IsUndocked
from spot_bt_ros_py.conditions.movement import IsSitting
from spot_bt_ros_py.conditions.movement import IsStanding
from spot_bt_ros_py.conditions.perception import IsAnyFiducialMarkerDetected


def create_generic_fiducial_selector(
    name: str = "Fiducial Detection", memory: bool = False, no_dock: bool = False
) -> Selector:
    """
    Create a generic fiducial selector for Spot.

    Args:
      name (str): Name given to the sequence directing docking logic.
      memory (bool): Resume with the status RUNNING child rather than a tick going through
        all actions/conditions in the sequence.
      no_dock (bool): Whether you want the fiducial detection to take into account
        Spot's dock.

    Returns:
      out (py_trees.composites.Selector): A selector for fiducial detection.
    """
    selector = Selector(name, memory=memory)
    selector.add_children(
        [
            IsAnyFiducialMarkerDetected(name="Is Fiducial Detected?"),
            DetectFiducialMarkers(name="Detect Fiducial Markers", no_dock=no_dock),
        ]
    )
    return selector


def create_lease_claim_selector(
    name: str = "Check and Claim Lease", memory: bool = False
) -> Selector:
    """
    Create an lease claim selector check.

    Args:
      name (str): Name given to the sequence directing docking logic.
      memory (bool): Resume with the status RUNNING child rather than a tick going through
        all actions/conditions in the sequence.

    Returns:
      out (py_trees.composites.Selector): A selector for Spot lease claiming.
    """
    selector = Selector(name, memory)
    selector.add_children(
        [
            IsLeaseClaimed("Is Lease Claimed?"),
            ClaimLease("Claim Lease"),
        ]
    )

    return selector


def create_lease_release_selector(
    name: str = "Check and Release Lease", memory: bool = False
) -> Selector:
    """
    Create an lease release selector check.

    Args:
      name (str): Name given to the sequence directing docking logic.
      memory (bool): Resume with the status RUNNING child rather than a tick going through
        all actions/conditions in the sequence.

    Returns:
      out (py_trees.composites.Selector): A selector for Spot lease releasing.
    """
    selector = Selector(name, memory)
    selector.add_children(
        [
            IsLeaseReleased("Is Lease Released?"),
            ReleaseLease("Release Lease"),
        ]
    )

    return selector


def create_dock_selector(name: str = "Check and dock", memory: bool = False) -> Selector:
    """
    Create an dock selector with a dock state check.

    Args:
      name (str): Name given to the sequence directing docking logic.
      memory (bool): Resume with the status RUNNING child rather than a tick going through
        all actions/conditions in the sequence.

    Returns:
      out (py_trees.composites.Selector): A selector for status check and initiating docking
       procedure.
    """
    selector = Selector(name, memory)
    selector.add_children(
        [
            IsDocked("Is Robot Docked?"),
            Dock("Dock Robot"),
        ]
    )

    return selector


def create_undock_selector(name: str = "Check and Undock", memory: bool = False) -> Selector:
    """
    Create an undock selector with a dock state check.

    Args:
      name (str): Name given to the sequence directing docking logic.
      memory (bool): Resume with the status RUNNING child rather than a tick going through
        all actions/conditions in the sequence.

    Returns:
      out (py_trees.composites.Selector): A selector for status check and initiating undocking
        procedure.
    """
    selector = Selector(name, memory)
    selector.add_children(
        [
            IsUndocked("Is Robot Undocked?"),
            Undock("Undock Robot"),
        ]
    )

    return selector


def create_power_off_selector(name: str = "Check and Power Off", memory: bool = False) -> Selector:
    """
    Create an power off selector with a power state check.

    Args:
      name (str): Name given to the sequence directing docking logic.
      memory (bool): Resume with the status RUNNING child rather than a tick going through
        all actions/conditions in the sequence.

    Returns:
      out (py_trees.composites.Selector): A selector for status check and initiating power off
        procedure.
    """
    selector = Selector(name, memory)
    selector.add_children(
        [
            IsPoweredOff("Is Robot Powered Off?"),
            PowerOff("Power Off Spot"),
        ]
    )

    return selector


def create_power_on_selector(name: str = "Check and Power On", memory: bool = False) -> Selector:
    """
    Create an power on selector with a power state check.

    Args:
      name (str): Name given to the sequence directing docking logic.
      memory (bool): Resume with the status RUNNING child rather than a tick going through
        all actions/conditions in the sequence.

    Returns:
      out (py_trees.composites.Selector): A selector for status check and initiating power on
        procedure.
    """
    selector = Selector(name, memory)
    selector.add_children(
        [
            IsPoweredOn("Is Robot Powered On?"),
            PowerOn("Power On Spot"),
        ]
    )

    return selector


def create_sitting_selector(name: str = "Check and Sit", memory: bool = False) -> Selector:
    """
    Create a selector for sitting Spot.

    Args:
      name (str): Name given to the sequence directing docking logic.
      memory (bool): Resume with the status RUNNING child rather than a tick going through
        all actions/conditions in the sequence.

    Returns:
      out (py_trees.composites.Selector): A selector for status check and initiating sitting
        procedure.
    """
    selector = Selector(name, memory)
    selector.add_children(
        [
            IsSitting("Is Spot Sitting?"),
            Sit("Sit"),
        ]
    )

    return selector


def create_standing_selector(name: str = "Check and Stand", memory: bool = True) -> Selector:
    """
    Create a selector for standing Spot.

    Args:
      name (str): Name given to the sequence directing docking logic.
      memory (bool): Resume with the status RUNNING child rather than a tick going through
        all actions/conditions in the sequence.

    Returns:
      out (py_trees.composites.Selector): A selector for status check and initiating standing
        procedure.
    """
    selector = Selector(name, memory)
    selector.add_children(
        [
            IsStanding("Is Spot Standing?"),
            Stand("Stand"),
        ]
    )

    return selector
