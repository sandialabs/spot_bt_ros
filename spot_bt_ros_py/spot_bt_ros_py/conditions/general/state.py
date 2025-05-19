"""Spot state related conditions."""

from __future__ import annotations

from py_trees.behaviour import Behaviour
from py_trees.blackboard import Client
from py_trees.common import Access
from py_trees.common import Status


class IsLeaseClaimed(Behaviour):
    """Behavior condition to check if Spot lease is claimed."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard: Client = None

    def initialise(self):
        """Initialize variables for condition behavior on first tick."""
        self.logger.debug(f"  {self.name} [IsLeaseClaimed::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.READ)

    def update(self) -> Status:
        """Run the IsLeaseClaimed behavior when ticked."""
        self.logger.debug(f"  {self.name} [IsLeaseClaimed::update()]")
        if self.blackboard.state.lease_claimed:
            return Status.SUCCESS

        return Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.logger.debug(
            f"  {self.name} [IsLeaseClaimed::terminate()]"
            f"[{self.status}->{new_status}]"
        )


class IsLeaseReleased(Behaviour):
    """Behavior condition to check if Spot lease is released."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard: Client = None

    def initialise(self):
        """Initialize variables for condition behavior on first tick."""
        self.logger.debug(f"  {self.name} [IsLeaseReleased::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.READ)

    def update(self) -> Status:
        """Run the IsLeaseReleased behavior when ticked."""
        self.logger.debug(f"  {self.name} [IsLeaseReleased::update()]")
        if self.blackboard.state.lease_claimed:
            return Status.FAILURE

        return Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate beheavior and save information."""
        self.logger.debug(
            f"  {self.name} [IsLeaseReleased::terminate()]"
            f"[{self.status}->{new_status}]"
        )
