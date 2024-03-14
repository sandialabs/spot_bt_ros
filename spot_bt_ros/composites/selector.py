"""spot_bt_ros Selector composites."""
from __future__ import annotations

import py_trees

from spot_bt_ros.behaviors.actions.perception import DetectFiducialMarkers
from spot_bt_ros.behaviors.conditions.perception import IsAnyFiducialMarkerDetected
from spot_bt_ros.behaviors.conditions.perception import IsDockFiducialMarkerDetected
from spot_bt_ros.composites.sequence import create_exploration_sequence


def create_fiducial_search_selector(name: str = "Fiducial Search", memory: bool = True) -> py_trees.composites.Selector:
    """Create a generic fiducial search selector for Spot."""
    selector = py_trees.composites.Selector(name, memory=memory)
    selector.add_children(
        [
            IsAnyFiducialMarkerDetected(name="Is Fiducial Detected?"),
            create_exploration_sequence(),
        ]
    )
    return selector


def create_dock_search_selector(name: str = "Dock Search", memory: bool = True) -> py_trees.composites.Selector:
    """Create a generic dock search selector for Spot."""
    selector = py_trees.composites.Selector(name, memory=memory)
    selector.add_child(
        IsDockFiducialMarkerDetected(name="Is Dock Fiducial Detected?")
    )
    selector.add_child(create_exploration_sequence())
    return selector


def create_generic_fiducial_selector(name: str = "Fiducial Detection", memory: bool = True) -> py_trees.composites.Selector:
    """Create a generic fiducial selector for Spot."""
    selector = py_trees.composites.Selector(name, memory=memory)
    selector.add_children(
        [
            IsAnyFiducialMarkerDetected(name="Is Fiducial Detected?"),
            DetectFiducialMarkers(name="Detect Fiducial Markers"),
        ]
    )
    return selector
