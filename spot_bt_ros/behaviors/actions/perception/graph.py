from __future__ import annotations

import os

from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.map_processing import MapProcessingServiceClient
from bosdyn.client.recording import GraphNavRecordingServiceClient
from bosdyn.client.recording import NotReadyYetError

import py_trees

from spot_bt_ros.data import Blackboards


class StartRecordingGraph(py_trees.behaviour.Behaviour):
    """Behavior class for Spot to start recording a GraphNav."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.graph_nav_client = None
        self.map_processing_client = None
        self.recording_client = None
        self.recording_environment = None

    def setup(self, **kwargs):
        """Setup the Graph Recording."""
        self.logger.debug(f"  {self.name} [StartRecordingGraph::setup()]")

    def initialise(self):
        """Initialize robot object and client behavior for first tick."""
        self.logger.debug(f"  {self.name} [StartRecordingGraph::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot
        self.blackboard.graph = self.attach_blackboard_client("Graph")
        self.blackboard.graph.register_key(
            key="recording_client", access=py_trees.common.Access.WRITE
        )
        self.blackboard.graph.register_key(
            key="recording_environment", access=py_trees.common.Access.WRITE
        )
        self.blackboard.graph.register_key(
            key="graph_nav_client", access=py_trees.common.Access.WRITE
        )
        self.blackboard.graph.register_key(
            key="map_processing_client", access=py_trees.common.Access.WRITE
        )
        client_metadata = GraphNavRecordingServiceClient.make_client_metadata(
            session_name="TEST_SESSION",
            client_username="zmkakis",
            client_id="RecordingClient",
            client_type="Python SDK",
        )
        self.recording_client = self.robot.ensure_client(GraphNavRecordingServiceClient)
        self.recording_environment = (
            GraphNavRecordingServiceClient.make_recording_environment(
                waypoint_env=GraphNavRecordingServiceClient.make_waypoint_environment(
                    client_metadata=client_metadata
                )
            )
        )
        self.graph_nav_client = self.robot.ensure_client(
            GraphNavClient.default_service_name
        )
        self.map_processing_client = self.robot.ensure_client(
            MapProcessingServiceClient.default_service_name
        )

    def update(self) -> py_trees.common.Status:
        """Run the StartRecordingGraph behavior when ticked."""
        self.logger.debug(f"  {self.name} [StartRecordingGraph::update()]")
        try:
            graph = self.graph_nav_client.download_graph()
            if graph is not None:
                if len(graph.waypoints) > 0:
                    localization_state = self.graph_nav_client.get_localization_state()
                    if not localization_state.localization.waypoint_id:
                        self.logger.warning(
                            f"  {self.name} [StartRecordingGraph::update()]"
                            + "\n\t Current map exists but not localized! Clearing map then beginning recording.",
                        )
                        self.graph_nav_client.clear_graph()
                    return py_trees.common.Status.RUNNING

            status = self.recording_client.start_recording(
                recording_environment=self.recording_environment
            )
            self.logger.debug(f"  {status}")
            return py_trees.common.Status.SUCCESS

        except:  # pylint: disable=bare-except
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.blackboard.graph.recording_client = self.recording_client
        self.blackboard.graph.recording_environment = self.recording_environment
        self.blackboard.graph.graph_nav_client = self.graph_nav_client
        self.blackboard.graph.map_processing_client = self.map_processing_client
        self.logger.debug(
            f" {self.name} [StartRecordingGraph::terminate().terminate()]"
            + f"[{self.status}->{new_status}]"
        )


class StopRecordingGraph(py_trees.behaviour.Behaviour):
    """Behavior class for Spot to start recording a GraphNav."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.recording_client = None

    def setup(self, **kwargs):
        """Setup the Graph Recording."""
        self.logger.debug(f"  {self.name} [StopRecordingGraph::setup()]")

    def initialise(self):
        """Initialize robot object and client behavior for first tick."""
        self.logger.debug(f"  {self.name} [StopRecordingGraph::initialise()]")
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
        """Run the StartRecordingGraph behavior when ticked."""
        self.logger.debug(f"  {self.name} [StopRecordingGraph::update()]")
        try:
            status = self.recording_client.stop_recording()
            self.logger.debug(f"  {status}")
            return py_trees.common.Status.SUCCESS

        except NotReadyYetError:
            return py_trees.common.Status.RUNNING

        except:  # pylint: disable=bare-except
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.blackboard.graph.recording_client = self.recording_client
        self.logger.debug(
            f" {self.name} [StartRecordingGraph::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class LoadGraph(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        os.getcwd()
        os.chdir("~/")


class DownloadGraph(py_trees.behaviour.Behaviour):
    """Behavior class for Spot to download a GraphNav recording."""

    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.graph = None
        self.edges = None
        self.waypoints = None
        self.graph_nav_client = None

    def setup(self, **kwargs):
        """Setup the Graph Downloading."""
        self.logger.debug(f"  {self.name} [DownloadGraph::setup()]")

    def initialise(self):
        """Initialize robot object and client behavior for first tick."""
        self.logger.debug(f"  {self.name} [Downloadraph::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot
        self.blackboard.graph = self.attach_blackboard_client("Graph")
        self.blackboard.graph.register_key(
            key="graph_nav_client", access=py_trees.common.Access.WRITE
        )
        self.blackboard.graph.register_key(
            key="graph", access=py_trees.common.Access.WRITE
        )
        self.blackboard.graph.register_key(
            key="edges", access=py_trees.common.Access.WRITE
        )
        self.blackboard.graph.register_key(
            key="waypoints", access=py_trees.common.Access.WRITE
        )
        self.graph_nav_client = self.blackboard.graph.graph_nav_client

    def update(self) -> py_trees.common.Status:
        """Run the DownloadGraph behavior when ticked."""
        self.logger.debug(f"  {self.name} [DownloadGraph::update()]")
        self.graph = self.graph_nav_client.download_graph()
        if self.graph is None:
            self.logger.error("Failed to download graph!")
            return py_trees.common.Status.FAILURE

        self.waypoints = self.graph.way
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.blackboard.graph.graph = self.graph
        self.blackboard.graph.graph_nav_client = self.graph_nav_client
        self.logger.debug(
            f" {self.name} [DownloadGraph::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )


class OptimizeAnchoring(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.graph_nav_client = None
        self.map_processing_client = None

    def setup(self, **kwargs):
        """Setup the Graph Downloading."""
        self.logger.debug(f"  {self.name} [OptimizeAnchoring::setup()]")

    def initialise(self):
        """Initialize robot object and client behavior for first tick."""
        self.logger.debug(f"  {self.name} [OptimizeAnchoring::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot
        self.blackboard.graph = self.attach_blackboard_client("Graph")
        self.blackboard.graph.register_key(
            key="graph_nav_client", access=py_trees.common.Access.WRITE
        )
        self.blackboard.graph.register_key(
            key="map_processing_client", access=py_trees.common.Access.WRITE
        )
        self.graph_nav_client = self.robot.ensure_client(
            GraphNavClient.default_service_name
        )
        self.map_processing_client = self.robot.ensure_client(
            MapProcessingServiceClient.default_service_name
        )

    def update(self) -> py_trees.common.Status:
        """Run the OptimizeAnchoring behavior when ticked."""
        pass

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f" {self.name} [OptimizeAnchoring::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )