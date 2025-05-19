"""Spot power related conditions."""

from __future__ import annotations

from py_trees.behaviour import Behaviour
from py_trees.blackboard import Client
from py_trees.common import Access
from py_trees.common import Status


class IsPoweredOn(Behaviour):
    """Behavior condition to check if Spot is powered."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard: Client = None

    def initialise(self):
        """Initialize variables for condition behavior on first tick."""
        self.logger.debug(f"  {self.name} [IsPoweredOn::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.READ)

    def update(self) -> Status:
        """Run the IsPoweredOn condition when ticked."""
        self.logger.debug(f"  {self.name} [IsPoweredOn::update()]")
        if self.blackboard.state.powered_on:
            return Status.SUCCESS

        return Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate condition and save information."""
        self.logger.debug(
            f"  {self.name} [IsPoweredOn::terminate()]"
            f"[{self.status}->{new_status}]"
        )


class IsPoweredOff(Behaviour):
    """Behavior condition to check if Spot is powered."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard: Client = None

    def initialise(self):
        """Initialize variables for condition behavior on first tick."""
        self.logger.debug(f"  {self.name} [IsPoweredOff::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.READ)

    def update(self) -> Status:
        """Run the IsPoweredOff condition when ticked."""
        self.logger.debug(f"  {self.name} [IsPoweredOff::update()]")
        if self.blackboard.state.powered_on:
            return Status.FAILURE

        return Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate condition and save information."""
        self.logger.debug(
            f"  {self.name} [IsPoweredOff::terminate()]"
            f"[{self.status}->{new_status}]"
        )
