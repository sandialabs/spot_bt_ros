"""Spot goal related conditions."""

from __future__ import annotations

from py_trees.behaviour import Behaviour
from py_trees.blackboard import Client
from py_trees.common import Access
from py_trees.common import Status


class GoalReached(Behaviour):
    """Behavior condition to check if goal was reached."""

    def __init__(self, name: str):
        super().__init__(name)
        self.mission_blackboard: Client = None
        self.status_blackboard: Client = None

    def initialise(self):
        """Initialize variables for condition behavior on first tick."""
        self.logger.debug(f"  {self.name} [GoalReached::initialise()]")
        self.mission_blackboard = self.attach_blackboard_client("mission")
        self.mission_blackboard.register_key(key="goal", access=Access.WRITE)
        self.status_blackboard = self.attach_blackboard_client("status")
        self.status_blackboard.register_key(key="current_pose", access=Access.WRITE)

    def update(self) -> Status:
        """Run the GoalReached condition when ticked."""
        self.logger.debug(f"  {self.name} [GoalReached::update()]")

        # TODO create logic for measuring distance to goal
        if self.mission_blackboard.mission.goal:
            return Status.SUCCESS

        return Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate condition and save information."""
        self.logger.debug(
            f"  {self.name} [GoalReached::terminate()]"
            f"[{self.status}->{new_status}]"
        )


class GoalUpdated(Behaviour):
    """Behavior condition to check if goal was updated."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard: Client = None

    def initialise(self):
        """Initialize variables for condition behavior on first tick."""
        self.logger.debug(f"  {self.name} [GoalUpdated::initialise()]")
        self.blackboard = self.attach_blackboard_client("mission")
        self.blackboard.register_key(key="goal_updated", access=Access.WRITE)

    def update(self) -> Status:
        """Run the GoalUpdated condition when ticked."""
        self.logger.debug(f"  {self.name} [GoalUpdated::update()]")
        if self.blackboard.mission.goal_updated:
            # Reset variable
            self.blackboard.mission.goal_updated = False
            return Status.SUCCESS

        return Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate condition and save information."""
        self.logger.debug(
            f"  {self.name} [GoalUpdated::terminate()]"
            f"[{self.status}->{new_status}]"
        )
