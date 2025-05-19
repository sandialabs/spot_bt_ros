"""Fiducial marker related conditions."""

from __future__ import annotations

from py_trees.behaviour import Behaviour
from py_trees.blackboard import Client
from py_trees.common import Access
from py_trees.common import Status


class IsAnyFiducialMarkerDetected(Behaviour):
    """Behavior condition to check if ANY fiducial is detected."""

    def __init__(self, name: str, include_dock: bool = False):
        super().__init__(name)
        self.blackboard: Client = None
        self.dock_id = None
        self.fiducials = (
            None  # google.protobuf.pyext._message.RepeatedCompositeContainer
        )
        self.include_dock = include_dock

    def initialise(self):
        """Initialize variables for condition behavior on first tick."""
        self.logger.debug(f"  {self.name} [IsAnyFiducialMarkerDetected::initialise()]")
        self.blackboard = self.attach_blackboard_client("mission")
        self.blackboard.register_key(key="dock_id", access=Access.READ)
        self.dock_id = self.blackboard.dock_id
        self.blackboard.register_key(key="fiducials", access=Access.READ)
        self.fiducials = self.blackboard.fiducials

    def update(self) -> Status:
        """Run the IsAnyFiducialMarkerDetected condition when ticked."""
        self.logger.debug(f"  {self.name} [IsAnyFiducialMarkerDetected::update()]")
        if self.fiducials is None:
            return Status.FAILURE

        if len(self.fiducials) == 0:
            return Status.FAILURE

        if not self.include_dock:
            if (
                len(self.fiducials) == 1
                and int(self.dock_id) == self.fiducials[0].apriltag_properties.tag_id
            ):
                return Status.FAILURE

        return Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate condition and save information."""
        self.logger.debug(
            f"  {self.name} [IsAnyFiducialMarkerDetected::terminate()]"
            f"[{self.status}->{new_status}]"
        )


class IsDockFiducialMarkerDetected(Behaviour):
    """Behavior condition to check if dock fiducial is detected."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard: Client = None
        self.dock_id = None
        self.fiducials = (
            None  # google.protobuf.pyext._message.RepeatedCompositeContainer
        )

    def initialise(self):
        """Initialize variables for condition behavior on first tick."""
        self.logger.debug(f"  {self.name} [IsDockFiducialMarkerDetected::initialise()]")
        self.blackboard = self.attach_blackboard_client("mission")
        self.blackboard.register_key(key="dock_id", access=Access.READ)
        self.dock_id = self.blackboard.dock_id
        self.blackboard.register_key(key="fiducials", access=Access.READ)
        self.fiducials = self.blackboard.fiducials

    def update(self) -> Status:
        """Run the IsDockFiducialMarkerDetected condition when ticked."""
        self.logger.debug(f"  {self.name} [IsDockFiducialMarkerDetected::update()]")
        if self.fiducials is None:
            return Status.FAILURE

        for fiducial in self.fiducials:
            if int(self.dock_id) == fiducial.apriltag_properties.tag_id:
                return Status.SUCCESS

        return Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate condition and save information."""
        self.logger.debug(
            f"  {self.name} [IsDockFiducialMarkerDetected::terminate()]"
            f"[{self.status}->{new_status}]"
        )


class IsSpecificFiducialMarkerDetected(Behaviour):
    """Behavior condition to check if a specific fiducial is detected."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard: Client = None
        self.target = None
        self.fiducials = (
            None  # google.protobuf.pyext._message.RepeatedCompositeContainer
        )

    def initialise(self):
        """Initialize variables for condition behavior on first tick."""
        self.logger.debug(
            f"  {self.name} [IsSpecificFiducialMarkerDetected::initialise()]"
        )
        self.blackboard = self.attach_blackboard_client("mission")
        self.blackboard.register_key(key="fiducials", access=Access.READ)
        self.fiducials = self.blackboard.fiducials
        self.blackboard.register_key(key="target_fiducial", access=Access.READ)
        self.target = self.blackboard.target_fiducial

    def update(self) -> Status:
        """Run the IsSpecificFiducialMarkerDetected condition when ticked."""
        self.logger.debug(f"  {self.name} [IsSpecificFiducialMarkerDetected::update()]")
        if self.fiducials is None:
            return Status.FAILURE

        if len(self.fiducials) == 0:
            return Status.FAILURE

        for fiducial in self.fiducials:
            if int(self.target) == fiducial.apriltag_properties.tag_id:
                return Status.SUCCESS

        return Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate condition and save information."""
        self.logger.debug(
            f"  {self.name} [IsSpecificFiducialMarkerDetected::terminate()]"
            f"[{self.status}->{new_status}]"
        )
