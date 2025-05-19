"""Spot pose related conditions."""

from __future__ import annotations

from py_trees.behaviour import Behaviour
from py_trees.blackboard import Client
from py_trees.common import Access
from py_trees.common import Status


class IsSitting(Behaviour):
    """Behavior condition to check if Spot is sitting."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard: Client = None

    def initialise(self):
        """Initialize variables for condition behavior on first tick."""
        self.logger.debug(f"  {self.name} [IsSitting::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.READ)

    def update(self) -> Status:
        """Run the IsSitting condition when ticked."""
        self.logger.debug(f"  {self.name} [IsSitting::update()]")
        if self.blackboard.state.standing:
            return Status.FAILURE

        return Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate condition and save information."""
        self.logger.debug(
            f"  {self.name} [IsSitting::terminate()]"
            f"[{self.status}->{new_status}]"
        )


class IsStanding(Behaviour):
    """Behavior condition to check if Spot is standing."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard: Client = None

    def initialise(self):
        """Initialize variables for condition behavior on first tick."""
        self.logger.debug(f"  {self.name} [IsStanding::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.READ)

    def update(self) -> Status:
        """Run the IsStanding condition when ticked."""
        self.logger.debug(f"  {self.name} [IsStanding::update()]")
        if self.blackboard.state.standing:
            return Status.SUCCESS

        return Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate condition and save information."""
        self.logger.debug(
            f"  {self.name} [IsStanding::terminate()]"
            f"[{self.status}->{new_status}]"
        )
