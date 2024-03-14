"""Fiducial marker related conditions."""
from __future__ import annotations

import py_trees

from spot_bt_ros.data import Blackboards


class IsAnyFiducialMarkerDetected(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, include_dock: bool = False):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.dock_id = None
        self.fiducials = (
            None  # google.protobuf.pyext._message.RepeatedCompositeContainer
        )
        self.include_dock = include_dock

    def initialise(self):
        """Initialize robot and client objects for condition on first tick."""
        self.logger.debug(f"  {self.name} [IsAnyFiducialMarkerDetected::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="dock_id", access=py_trees.common.Access.READ
        )
        self.blackboard.perception = self.attach_blackboard_client("Perception")
        self.blackboard.perception.register_key(
            key="fiducials", access=py_trees.common.Access.READ
        )
        self.dock_id = self.blackboard.state.dock_id
        self.fiducials = self.blackboard.perception.fiducials

    def update(self) -> py_trees.common.Status:
        """Run the IsAnyFiducialMarkerDetected condition when ticked."""
        self.logger.debug(f"  {self.name} [IsAnyFiducialMarkerDetected::update()]")
        if self.fiducials is None:
            return py_trees.common.Status.FAILURE

        if len(self.fiducials) == 0:
            return py_trees.common.Status.FAILURE

        if not self.include_dock:
            if len(self.fiducials) == 1 and int(self.dock_id) == self.fiducials.apriltag_properties.tag_id:
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate condition and save information."""
        self.logger.debug(
            f" {self.name} [IsAnyFiducialMarkerDetected::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class IsDockFiducialMarkerDetected(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.dock_id = None
        self.fiducials = (
            None  # google.protobuf.pyext._message.RepeatedCompositeContainer
        )

    def initialise(self):
        """Initialize robot and client objects for condition on first tick."""
        self.logger.debug(f"  {self.name} [IsDockFiducialMarkerDetected::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="dock_id", access=py_trees.common.Access.READ
        )
        self.blackboard.perception = self.attach_blackboard_client("Perception")
        self.blackboard.perception.register_key(
            key="fiducials", access=py_trees.common.Access.READ
        )
        self.dock_id = self.blackboard.state.dock_id
        self.fiducials = self.blackboard.perception.fiducials

    def update(self) -> py_trees.common.Status:
        """Run the IsDockFiducialMarkerDetected condition when ticked."""
        self.logger.debug(f"  {self.name} [IsDockFiducialMarkerDetected::update()]")
        if self.fiducials is None:
            return py_trees.common.Status.FAILURE

        for fiducial in self.fiducials:
            if int(self.dock_id) == fiducial.apriltag_properties.tag_id:
                return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate condition and save information."""
        self.logger.debug(
            f" {self.name} [IsDockFiducialMarkerDetected::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class IsSpecificFiducialMarkerDetected(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.target = None
        self.fiducials = (
            None  # google.protobuf.pyext._message.RepeatedCompositeContainer
        )

    def initialise(self):
        """Initialize robot and client objects for condition on first tick."""
        self.logger.debug(f"  {self.name} [IsSpecificFiducialMarkerDetected::initialise()]")
        self.blackboard.perception = self.attach_blackboard_client("Perception")
        self.blackboard.perception.register_key(
            key="fiducials", access=py_trees.common.Access.READ
        )
        self.blackboard.perception.register_key(
            key="target_fiducial", access=py_trees.common.Access.READ
        )
        self.fiducials = self.blackboard.perception.fiducials
        self.target = self.blackboard.perception.target_fiducial

    def update(self) -> py_trees.common.Status:
        """Run the IsSpecificFiducialMarkerDetected condition when ticked."""
        self.logger.debug(f"  {self.name} [IsSpecificFiducialMarkerDetected::update()]")
        if self.fiducials is None:
            return py_trees.common.Status.FAILURE

        if len(self.fiducials) == 0:
            return py_trees.common.Status.FAILURE

        for fiducial in self.fiducials:
            if int(self.target) == fiducial.apriltag_properties.tag_id:
                return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.FAILURE


    def terminate(self, new_status: str):
        """Terminate condition and save information."""
        self.logger.debug(
            f" {self.name} [IsSpecificFiducialMarkerDetected::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )
