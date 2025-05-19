"""Spot general arm-related conditions."""

from __future__ import annotations

from py_trees.behaviour import Behaviour
from py_trees.blackboard import Client
from py_trees.common import Access
from py_trees.common import Status


class IsArmStowed(Behaviour):
    """Behavior condition to check if Spot's arm is stowed."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard: Client = None

    def initialise(self):
        """Initialize variables for condition behavior on first tick."""
        self.logger.debug(f"  {self.name} [IsArmStowed::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.READ)

    def update(self) -> Status:
        """Run the IsArmStowed condition when ticked."""
        self.logger.debug(f"  {self.name} [IsArmStowed::update()]")
        if self.blackboard.state.arm_stowed:
            return Status.SUCCESS

        return Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate condition and save information."""
        self.logger.debug(
            f"  {self.name} [IsArmStowed::terminate()]"
            f"[{self.status}->{new_status}]"
        )


class IsArmUnstowed(Behaviour):
    """Behavior condition to check if Spot's arm is unstowed."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard: Client = None

    def initialise(self):
        """Initialize variables for condition behavior on first tick."""
        self.logger.debug(f"  {self.name} [IsArmUnstowed::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.READ)

    def update(self) -> Status:
        """Run the IsArmUnstowed condition when ticked."""
        self.logger.debug(f"  {self.name} [IsArmUnstowed::update()]")
        if self.blackboard.state.arm_stowed:
            return Status.FAILURE

        return Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate condition and save information."""
        self.logger.debug(
            f"  {self.name} [IsArmUnstowed::terminate()]"
            f"[{self.status}->{new_status}]"
        )


class IsArmDeployed(Behaviour):
    """Behavior condition to check if Spot's arm is deployed."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard: Client = None

    def initialise(self):
        """Initialize variables for condition behavior on first tick."""
        self.logger.debug(f"  {self.name} [IsArmDeployed::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.READ)

    def update(self) -> Status:
        """Run the IsArmDeployed condition when ticked."""
        self.logger.debug(f"  {self.name} [IsArmDeployed::update()]")
        if not self.blackboard.state.arm_stowed:
            return Status.SUCCESS

        return Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate condition and save information."""
        self.logger.debug(
            f"  {self.name} [IsArmDeployed::terminate()]"
            f"[{self.status}->{new_status}]"
        )
