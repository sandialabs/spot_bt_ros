"""Spot docking related conditions."""

from __future__ import annotations

from py_trees.behaviour import Behaviour
from py_trees.blackboard import Client
from py_trees.common import Access
from py_trees.common import Status


class IsDocked(Behaviour):
    """Behavior condition to check if Spot is docked."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard: Client = None

    def initialise(self):
        """Initialize variables for condition behavior on first tick."""
        self.logger.debug(f"  {self.name} [IsDocked::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.READ)

    def update(self) -> Status:
        """Run the IsDocked condition when ticked."""
        self.logger.debug(f"  {self.name} [IsDocked::update()]")
        if self.blackboard.state.docked:
            return Status.SUCCESS

        return Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate condition and save information."""
        self.logger.debug(
            f"  {self.name} [IsDocked::terminate()]"
            f"[{self.status}->{new_status}]"
        )


class IsUndocked(Behaviour):
    """Behavior condition to check if Spot is undocked."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard: Client = None

    def initialise(self):
        """Initialize variables for condition behavior on first tick."""
        self.logger.debug(f"  {self.name} [IsUndocked::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.READ)

    def update(self) -> Status:
        """Run the IsUndocked condition when ticked."""
        self.logger.debug(f"  {self.name} [IsUndocked::update()]")
        if self.blackboard.state.docked:
            return Status.FAILURE

        return Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate condition and save information."""
        self.logger.debug(
            f"  {self.name} [IsUndocked::terminate()]"
            f"[{self.status}->{new_status}]"
        )
