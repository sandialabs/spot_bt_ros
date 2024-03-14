from __future__ import annotations

from bosdyn.api import world_object_pb2

import py_trees

from bosdyn_msgs.msg import WorldObjectType

from spot_msgs.srv import ListWorldObjects

from spot_bt_ros.data import Blackboards


class DetectFiducialMarkers(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.robot = None
        self.fiducials = (
            None  # google.protobuf.pyext._message.RepeatedCompositeContainer
        )

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"  {self.name} [DetectFiducialMarkers::initialise()]")
        self.blackboard.state = self.attach_blackboard_client("State")
        self.blackboard.state.register_key(
            key="robot", access=py_trees.common.Access.WRITE
        )
        self.blackboard.perception = self.attach_blackboard_client("Perception")
        self.blackboard.perception.register_key(
            key="fiducials", access=py_trees.common.Access.WRITE
        )
        self.robot = self.blackboard.state.robot

    def update(self) -> py_trees.common.Status:
        """Run the DetectFiducialMarkers behavior when ticked."""
        self.logger.debug(f"  {self.name} [DetectFiducialMarkers::update()]")
        request = ListWorldObjects.Request()
        # request.request.object_type = world_object_pb2.WORLD_OBJECT_APRILTAG
        temp = WorldObjectType()
        temp.value = world_object_pb2.WORLD_OBJECT_APRILTAG
        request.request.object_type = [temp]
        self.fiducials = self.robot.command("list", request=request)
        print(f"THESE ARE THE FIDUCIALS I FOUND: {self.fiducials}")
        # try:
        #     request = ListWorldObjects.Request()
        #     request.request.object_type = [world_object_pb2.WORLD_OBJECT_APRILTAG]
        #     self.fiducials = self.robot.command("list", request=request)
        #     print(f"THESE ARE THE FIDUCIALS I FOUND: {self.fiducials}")

        # except:  # pylint: disable=bare-except
        #     return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.blackboard.perception.fiducials = self.fiducials.response.world_objects
        self.logger.debug(
            f" {self.name} [DetectFiducialMarkers::terminate().terminate()]" +
            f"[{self.status}->{new_status}]"
        )
