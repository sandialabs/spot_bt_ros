"""Graph related conditions."""

from __future__ import annotations

from bosdyn.client.recording import GraphNavRecordingServiceClient

from py_trees.behaviour import Behaviour
from py_trees.common import Access
from py_trees.common import Status

from spot_bt_ros_py.data import Blackboards


class IsGraphRecording(Behaviour):
    """Behavior class for Spot to start recording a GraphNav."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.recording_client = None

    def initialise(self):
        """Initialize variables for condition behavior on first tick."""
        self.logger.debug(f"  {self.name} [IsGraphRecording::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(key="robot", access=Access.WRITE)
        self.robot = self.blackboard.state.robot
        self.blackboard.graph = self.attach_blackboard_client("Graph")
        self.blackboard.graph.register_key(key="recording_client", access=Access.WRITE)
        self.recording_client = self.robot.ensure_client(GraphNavRecordingServiceClient)

    def update(self) -> Status:
        """Run the IsGraphRecording condition when ticked."""
        self.logger.debug(f"  {self.name} [IsGraphRecording::update()]")
        status = self.recording_client.get_record_status()
        if status.is_recording:
            return Status.SUCCESS

        return Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate condition and save information."""
        self.logger.debug(
            f"  {self.name} [IsGraphRecording::terminate()]"
            f"[{self.status}->{new_status}]"
        )
