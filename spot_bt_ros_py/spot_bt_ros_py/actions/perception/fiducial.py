"""Fiducial detection related actions."""

from __future__ import annotations

from py_trees.behaviour import Behaviour
from py_trees.blackboard import Client
from py_trees.common import Access
from py_trees.common import Status

from rclpy.client import Client as rclClient
from rclpy.node import Node
from rclpy.task import Future

from bosdyn_msgs.msg import WorldObjectType

from spot_msgs.srv import ListWorldObjects


class DetectFiducialMarkers(Behaviour):
    """
    Detect fiducial markers using all of Spot's on-board cameras.

    Attributes:
      blackboard (py_trees.blackboard.Client): Maintains BT variables associated with a
        specific blackboard client.
      client (rclpy.client.Client): A ROS service client to connect to the `spot_driver`.
      future (rclpy.task.Future): Stores future information for the async ROS
        client service call.
      node (rclpy.node.Node): Stores the main ROS node for the BT process. This allows
        our action to use some of the nodes functionality.
      no_dock (bool): Whether the fiducial detection service call will/will not attempt to
        detect Spot's dock.
      dock_id (int): The dock fiducial id to skip if `no_dock` is `True`.
    """

    def __init__(self, name: str, no_dock: bool = False):
        """
        Initialize a DetectFiducialMarkers action object.

        Args:
          name (str): The action name. This will be applied to logging messages and image
            outputs of the action.
          no_dock (bool): Whether you want the fiducial detection to take into account
            Spot's dock.
        """
        super().__init__(name)
        self.blackboard: Client = None
        self.client: rclClient = None
        self.future: Future = None
        self.node: Node = None
        self.no_dock: bool = no_dock
        self.dock_id: int = None

    def setup(self, **kwargs):
        """Setup ROS 2 subs/pubs/clients for connection to spot_driver."""
        self.logger.debug(f"{self.qualified_name}.setup()")
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_msg = f"didn't find 'node' in setup's kwargs [{self.qualified_name}]"
            raise KeyError(error_msg) from e

        self.client = self.node.create_client(ListWorldObjects, "list_world_objects")
        self.client.wait_for_service()

    def initialise(self):
        """Initialize variables and perform action behavior for first tick."""
        self.logger.debug(f"  {self.name} [DetectFiducialMarkers::initialise()]")
        self.blackboard = self.attach_blackboard_client("mission")
        self.blackboard.register_key(key="fiducials", access=Access.WRITE)

        if self.no_dock:
            self.blackboard.register_key(key="dock_id", access=Access.READ)
            self.dock_id = self.blackboard.dock_id

        # Send client request
        request = ListWorldObjects.Request()
        msg = WorldObjectType()
        msg.value = 2  # world_object_pb2.WORLD_OBJECT_APRILTAG = Literal[2]
        request.request.object_type = [msg]
        self.future = self.client.call_async(request)

    def update(self) -> Status:
        """Run the DetectFiducialMarkers behavior when ticked."""
        self.logger.debug(f"  {self.name} [DetectFiducialMarkers::update()]")

        if self.future.done():
            fiducials = self.future.result().response.world_objects
            if fiducials is None or len(fiducials) == 0:
                return Status.FAILURE

            self.logger.info(f"No. of World objects found: {len(fiducials)}")

            # Remove detected dock fiducial
            if self.no_dock:
                self.logger.debug("Removing detected dock fiducial, if found.")
                for i, fiducial in enumerate(fiducials):
                    if fiducial.apriltag_properties.tag_id == self.dock_id:
                        fiducials.pop(i)
                        self.logger.debug(f"Removed dock id {self.dock_id} from detection!")

            if len(fiducials) == 0:
                return Status.FAILURE

            # Set fiducials to blackboard
            self.blackboard.fiducials = fiducials

            return Status.SUCCESS

        self.logger.debug(f"  {self.name} [DetectFiducialMarkers::update()][RUNNING]")
        return Status.RUNNING

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f"  {self.name} [DetectFiducialMarkers::terminate()]"
            f"[{self.status}->{new_status}]"
        )
