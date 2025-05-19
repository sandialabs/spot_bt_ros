"""Useful data types for BT actions and conditions."""

from __future__ import annotations

from dataclasses import dataclass
from dataclasses import field


@dataclass
class SpotState:
    """Dataclass containing useful Spot behavior tree state information."""

    # Boolean Statuses (Body)
    docked: bool = field(default=False)
    lease_claimed: bool = field(default=False)
    powered_on: bool = field(default=False)
    standing: bool = field(default=False)
    crouching: bool = field(default=False)
    stopped: bool = field(default=False)

    # Boolean Statuses (Arm)
    arm_stowed: bool = field(default=True)
    gripper_open: bool = field(default=False)
    arm_carrying: bool = field(default=False)

    # Boolean Statuses (Safety)
