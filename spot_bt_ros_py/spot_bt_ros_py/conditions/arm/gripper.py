"""Spot general gripper-related conditions."""

from __future__ import annotations

from py_trees.behaviour import Behaviour
from py_trees.blackboard import Client
from py_trees.common import Access
from py_trees.common import Status


class IsGripperOpen(Behaviour):
    """Behavior condition to check if Spot's gripper is open."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard: Client = None

    def initialise(self):
        """Initialize variables for condition behavior on first tick."""
        self.logger.debug(f"  {self.name} [IsGripperOpen::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.READ)

    def update(self) -> Status:
        """Run the IsGripperOpen condition when ticked."""
        self.logger.debug(f"  {self.name} [IsGripperOpen::update()]")
        if self.blackboard.state.gripper_open:
            return Status.SUCCESS

        return Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate condition and save information."""
        self.logger.debug(
            f"  {self.name} [IsGripperOpen::terminate()]"
            f"[{self.status}->{new_status}]"
        )


class IsGripperClosed(Behaviour):
    """Behavior condition to check if Spot's gripper is closed."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard: Client = None

    def initialise(self):
        """Initialize variables for condition behavior on first tick."""
        self.logger.debug(f"  {self.name} [IsGripperClosed::initialise()]")
        self.blackboard = self.attach_blackboard_client("status")
        self.blackboard.register_key(key="state", access=Access.READ)

    def update(self) -> Status:
        """Run the IsGripperClosed condition when ticked."""
        self.logger.debug(f"  {self.name} [IsGripperClosed::update()]")
        if self.blackboard.state.gripper_open:
            return Status.FAILURE

        return Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate condition and save information."""
        self.logger.debug(
            f"  {self.name} [IsGripperClosed::terminate()]"
            f"[{self.status}->{new_status}]"
        )
