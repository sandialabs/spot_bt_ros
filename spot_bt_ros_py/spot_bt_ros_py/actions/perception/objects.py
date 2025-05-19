"""World object related actions."""

from __future__ import annotations

from py_trees.behaviour import Behaviour
from py_trees.blackboard import Client
from py_trees.common import Access
from py_trees.common import Status

from rclpy.client import Client as rclClient
from rclpy.node import Node
from rclpy.task import Future

from spot_msgs.srv import ListWorldObjects


class DetectWorldObjects(Behaviour):
    """
    Detect world objects using all of Spot's on-board cameras.

    Attributes:
      blackboard (py_trees.blackboard.Client): Maintains BT variables associated with a
        specific blackboard client.
      client (rclpy.client.Client): A ROS service client to connect to the `spot_driver`.
      future (rclpy.task.Future): Stores future information for the async ROS
        client service call.
      node (rclpy.node.Node): Stores the main ROS node for the BT process. This allows
        our action to use some of the nodes functionality.
    """

    def __init__(self, name: str):
        """
        Initialize a DetectWorldObjects action object.

        Args:
          name (str): The action name. This will be applied to logging messages and image
            outputs of the action.
        """
        super().__init__(name)
        self.blackboard: Client = None
        self.client: rclClient = None
        self.future: Future = None
        self.node: Node = None

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
        self.logger.debug(f"  {self.name} [DetectWorldObjects::initialise()]")
        self.blackboard = self.attach_blackboard_client("mission")
        self.blackboard.register_key(key="world_objects", access=Access.WRITE)

        # Send client request
        request = ListWorldObjects.Request()
        self.future = self.client.call_async(request)

    def update(self) -> Status:
        """Run the DetectWorldObjects behavior when ticked."""
        self.logger.debug(f"  {self.name} [DetectWorldObjects::update()]")
        if self.future.done():
            # Examine the transform snapshot for the world object!
            for obj in self.future:
                self.node.get_logger().info(f"ID: {obj.id}")
                full_snapshot = obj.transforms_snapshot
                for edge in full_snapshot.child_to_parent_edge_map:
                    self.node.get_logger().info(
                        f"Child frame name: {edge}. Parent frame name: "
                        f"{full_snapshot.child_to_parent_edge_map[edge].parent_frame_name}"
                    )

            self.blackboard.world_objects = self.future.response.world_objects
            return Status.SUCCESS

            # TODO: ADD FAILURE MODE IF NOT OBJECTS FOUND!

        return Status.RUNNING

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f"  {self.name} [DetectWorldObjects::terminate()]"
            f"[{self.status}->{new_status}]"
        )
