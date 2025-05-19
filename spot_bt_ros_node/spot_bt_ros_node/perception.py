"""Stand-alone node version of Spot Perception Commander. (DEPRECATED)"""

from __future__ import annotations

from bosdyn.api import world_object_pb2

from bosdyn_api_msgs.msg import WorldObject

from bosdyn_msgs.msg import WorldObjectType

from rclpy.client import Client
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.client import Future
from rclpy.executors import ExternalShutdownException

from spot_msgs.srv import ListWorldObjects

from spot_bt_ros_msgs.action import SpotPerceptionTask


PERCEPTION_SERVICES = [
    "list_fiducials",
    "list_world_objects",
]


class SpotPerception(Node):
    """DEPRECATED - A perception controller for interfacing with Spot."""
    # TODO Remove from setup.py too

    def __init__(self, namespace: str | None = None):
        super().__init__("spot_perception_node", namespace=namespace)
        if namespace is not None:
            self._name = f"{namespace}/"
            self._ns = namespace
        else:
            self._name = ""
            self._ns = ""

        self.declare_parameters(
            "",
            [
                ("dock_id", 549),
                ("has_arm", True),
                ("docked", True),
                ("standing", False),
            ],
        )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._dock_id = self.get_parameter("dock_id").get_parameter_value().integer_value
        # self._saved_feedback_msg = None
        self._found_world_objects: list[WorldObject] = []
        self._found_fiducials: list[int] = []

        # TODO: Add publishers that publish what is currently being seen (possibly)

        self._command_map: dict[str, Client] = {
            "list": self.create_client(
                ListWorldObjects, "list_world_objects"
            ),
        }

        # Create action server interface to Spot perception beahvior tree actions
        self._action_server = ActionServer(
            self,
            SpotPerceptionTask,
            "spot/perception/actions/basic",
            self._execute_basic_perception_action,
        )

    def _parse_list_fiducials(self, response: ListWorldObjects.Response):
        """Parse an incoming list of world objects (fiducials)."""
        self._found_world_objects = []
        self._found_fiducials = []
        if response.response.world_objects is not None or len(response.response.world_objects) > 0:
            self._found_world_objects = response.response.world_objects
            for world_object in self._found_world_objects:
                self._found_fiducials.append(world_object.apriltag_properties.tag_id)

    def _execute_basic_perception_action(
        self, goal_handle: ServerGoalHandle
    ) -> SpotPerceptionTask.Result:
        """Execute a basic Spot perception task."""
        action: str = goal_handle.request.action
        future = Future()
        result = SpotPerceptionTask.Result()

        # Send basic perception action request
        if action == "list_fiducials":
            request = ListWorldObjects.Request()
            msg = WorldObjectType()
            msg.value = world_object_pb2.WORLD_OBJECT_APRILTAG
            request.request.object_type = [msg]
            future = self._command_map["list"].call_async(request)
        else:
            goal_handle.abort()
            result.header = self.get_clock().now().to_msg()
            result.status = "FAILURE"
            result.message = f"No command {action}"
            self.get_logger().error(result.message)
            return result

         # Wait for action to complete
        feedback_msg = SpotPerceptionTask.Feedback()
        start = self.get_time()
        while not future.done():
            feedback_msg.header = self.get_clock().now().to_msg()
            feedback_msg.status = "RUNNING"
            feedback_msg.message = (
                f"Feedback: Action '{action}' still running... "
                f"(Duration: {self.get_time() - start})"
            )
            self.get_logger().info(feedback_msg.message)
            goal_handle.publish_feedback(feedback_msg)

        # Logic for action completion
        result.header = self.get_clock().now().to_msg()
        if future.result().success:
            goal_handle.succeed()
            # TODO: Add area to parse results
            result.success = True
            result.message = f"Result: Action '{action}' completed successfully!"
            self.get_logger().info(result.message)
        else:
            goal_handle.abort()
            result.success = False
            result.message = future.result().message
            self.get_logger().error(result.message)

        return result

    def get_time(self) -> int:
        """Get current clock time in nanoseconds."""
        return self.get_clock().now().nanoseconds

    @property
    def name(self) -> str:
        """Get node name."""
        return self.get_name()



def main(args=None):
    """Start Spot Perception node."""
    rclpy.init(args=args)

    node = SpotPerception()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().warning(f"Killing node: {node.name}")
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
