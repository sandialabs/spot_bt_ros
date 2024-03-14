from __future__ import annotations

from bosdyn.client.recording import GraphNavRecordingServiceClient

import py_trees

from spot_bt_ros.data import Blackboards


class IsGraphRecording(py_trees.behaviour.Behaviour):
    """Behavior class for Spot to start recording a GraphNav."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.recording_client = None

    def setup(self):
        """Setup the Graph Recording."""
        self.logger.debug(f"\t{self.name} [IsGraphRecording::setup()]")

    def initialise(self):
        """Initialize robot and client objects for condition on first tick."""
        self.logger.debug(f"\t{self.name} [IsGraphRecording::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot
        self.blackboard.graph = self.attach_blackboard_client("Graph")
        self.blackboard.graph.register_key(
            key="recording_client", access=py_trees.common.Access.WRITE
        )
        self.recording_client = self.robot.ensure_client(GraphNavRecordingServiceClient)

    def update(self) -> py_trees.common.Status:
        """Run the IsGraphRecording condition when ticked."""
        self.logger.debug(f"\t\t{self.name} [IsGraphRecording::update()]")
        status = self.recording_client.get_record_status()
        if status.is_recording:
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate condition and save information."""
        self.logger.debug(
            f"\t{self.name} [IsGraphRecording::terminate().terminate()]" +
            f"[{self.status}->{new_status}]"
        )
