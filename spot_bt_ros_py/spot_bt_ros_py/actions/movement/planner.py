"""Spot planner-related actions."""

from __future__ import annotations

from py_trees.behaviour import Behaviour
from py_trees.blackboard import Client
from py_trees.common import Access
from py_trees.common import Status

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future

from spot_bt_ros_msgs.action import PlanSpotBodyTask


class ComputePathToPose(Behaviour):
    """
    Send planner request to compute path to target.

    Attributes:
      blackboard (py_trees.blackboard.Client): Maintains BT variables associated with a
        specific blackboard client.
      mission_blackboard (py_trees.blackboard.Client): Maintains BT variables associated
        with a mission specific blackboard client.
      client (rclpy.action.ActionClient): A ROS action client to connect to the Spot
        controller.
      future (rclpy.task.Future): Stores future information for the async ROS
        client action call.
      node (rclpy.node.Node): Stores the main ROS node for the BT process. This allows
        our action to use some of the nodes functionality.
    """

    def __init__(self, name: str):
        """
        Initialize an ComputePathToPose action object.

        Args:
          name (str): The action name. This will be applied to logging messages and image
            outputs of the action.
        """
        super().__init__(name)
        self.blackboard: Client = None
        self.mission_blackboard: Client = None
        self.client: ActionClient = None
        self.future: Future = None
        self.node: Node = None

    def setup(self, **kwargs):
        """Setup ComputePathToPose behavior before initialization."""
        self.logger.debug(f"{self.name} [ComputePathToPose::setup()]")
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_msg = f"didn't find 'node' in setup's kwargs [{self.qualified_name}]"
            raise KeyError(error_msg) from e

    def initialise(self):
        """Initialize variables and perform action behavior for first tick."""
        self.logger.debug(f"  {self.name} [ComputePathToPose::initialise()]")
        self.blackboard = self.attach_blackboard_client("clients")
        self.blackboard.register_key(key="planner", access=Access.WRITE)
        self.client = self.blackboard.planner
        self.mission_blackboard = self.attach_blackboard_client("mission")
        self.mission_blackboard.register_key(key="target", access=Access.READ)

        # Send action client request
        goal_msg = PlanSpotBodyTask.Goal()
        goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.action = "target"
        goal_msg.target = self.mission_blackboard.target
        self.future = self.client.send_goal_async(goal_msg, feedback_callback=self._feedback)

    def update(self) -> Status:
        """Run the ComputePathToPose behavior when ticked."""
        self.logger.debug(f"  {self.name} [ComputePathToPose::update()]")

        if self.future.done():
            result = self.future.result().get_result().result
            if result.success:
                return Status.SUCCESS

            self.logger.error(f"{result.message}")
            return Status.FAILURE

        self.logger.debug(f"  {self.name} [ComputePathToPose::update()][RUNNING]")
        return Status.RUNNING

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f"  {self.name} [ComputePathToPose::terminate()]"
            f"[{self.status}->{new_status}]"
        )

    def _feedback(self, feedback_msg: PlanSpotBodyTask.Feedback):
        """Log action feedback for ComputePathToPose."""
        self.node.get_logger().info(
            feedback_msg.feedback.feedback, throttle_duration_sec=0.1
        )


class ComputePathToFiducial(Behaviour):
    """
    Send planner request to compute path to fiducial target.

    Attributes:
      blackboard (py_trees.blackboard.Client): Maintains BT variables associated with a
        specific blackboard client.
      mission_blackboard (py_trees.blackboard.Client): Maintains BT variables associated
        with a mission specific blackboard client.
      client (rclpy.action.ActionClient): A ROS action client to connect to the Spot
        controller.
      future (rclpy.task.Future): Stores future information for the async ROS
        client action call.
      node (rclpy.node.Node): Stores the main ROS node for the BT process. This allows
        our action to use some of the nodes functionality.
    """

    def __init__(self, name: str):
        """
        Initialize an ComputePathToFiducial action object.

        Args:
          name (str): The action name. This will be applied to logging messages and image
            outputs of the action.
        """
        super().__init__(name)
        self.blackboard: Client = None
        self.mission_blackboard: Client = None
        self.client: ActionClient = None
        self.future: Future = None
        self.node: Node = None

    def setup(self, **kwargs):
        """Setup ComputePathToFiducial behavior before initialization."""
        self.logger.debug(f"{self.name} [ComputePathToFiducial::setup()]")
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_msg = f"didn't find 'node' in setup's kwargs [{self.qualified_name}]"
            raise KeyError(error_msg) from e

    def initialise(self):
        """Initialize variables and perform action behavior for first tick."""
        self.logger.debug(f"  {self.name} [ComputePathToFiducial::initialise()]")
        self.blackboard = self.attach_blackboard_client("clients")
        self.blackboard.register_key(key="planner", access=Access.WRITE)
        self.client = self.blackboard.planner
        self.mission_blackboard = self.attach_blackboard_client("mission")
        self.mission_blackboard.register_key(key="dock_id", access=Access.READ)
        self.mission_blackboard.register_key(key="fiducials", access=Access.WRITE)
        self.mission_blackboard.register_key(key="target", access=Access.WRITE)

        # Make sure fiducial is not a dock fiducial
        # TODO need a better way of choosing fiducial marker
        dock_id = self.mission_blackboard.dock_id
        fiducials = self.mission_blackboard.fiducials
        select_fiducial = None
        self.logger.info(f"Length of saved fiducials: {len(fiducials)}")
        if len(fiducials) >= 1:
            for fiducial in fiducials:
                if fiducial.apriltag_properties.tag_id != dock_id:
                    select_fiducial = fiducial

        self.logger.info(f"select_fiducial of type {type(select_fiducial)}")
        # Send action client request
        goal_msg = PlanSpotBodyTask.Goal()
        goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.action = "fiducial"
        goal_msg.world_object = select_fiducial
        self.future = self.client.send_goal_async(goal_msg, feedback_callback=self._feedback)

    def update(self) -> Status:
        """Run the ComputePathToFiducial behavior when ticked."""
        self.logger.debug(f"  {self.name} [ComputePathToFiducial::update()]")

        if self.future.done():
            result = self.future.result().get_result().result
            if result.success:
                self.mission_blackboard.target = result.goal_pose
                return Status.SUCCESS

            self.logger.error(f"{result.message}")
            return Status.FAILURE

        self.logger.debug(f"  {self.name} [ComputePathToFiducial::update()][RUNNING]")
        return Status.RUNNING

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f"  {self.name} [ComputePathToFiducial::terminate()]"
            f"[{self.status}->{new_status}]"
        )

    def _feedback(self, feedback_msg: PlanSpotBodyTask.Feedback):
        """Log action feedback for ComputePathToFiducial."""
        self.node.get_logger().info(
            feedback_msg.feedback.feedback, throttle_duration_sec=0.1
        )


class ComputeNewWaypoint(Behaviour):
    """
    Send planer request to compute a new waypoint.

    Attributes:
      blackboard (py_trees.blackboard.Client): Maintains BT variables associated with a
        specific blackboard client.
    mission_blackboard (py_trees.blackboard.Client): Maintains BT variables associated
        with a mission specific blackboard client.
      client (rclpy.action.ActionClient): A ROS action client to connect to the Spot
        controller.
      future (rclpy.task.Future): Stores future information for the async ROS
        client action call.
      node (rclpy.node.Node): Stores the main ROS node for the BT process. This allows
        our action to use some of the nodes functionality.
    """

    def __init__(self, name: str):
        """
        Initialize an ComputeNewDirection action object.

        Args:
          name (str): The action name. This will be applied to logging messages and image
            outputs of the action.
        """
        super().__init__(name)
        self.blackboard: Client = None
        self.mission_blackboard: Client = None
        self.client: ActionClient = None
        self.future: Future = None
        self.node: Node = None

    def setup(self, **kwargs):
        """Setup ComputeNewDirection behavior before initialization."""
        self.logger.debug(f"{self.name} [ComputeNewDirection::setup()]")
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_msg = f"didn't find 'node' in setup's kwargs [{self.qualified_name}]"
            raise KeyError(error_msg) from e

    def initialise(self):
        """Initialize variables and perform action behavior for first tick."""
        self.logger.debug(f"  {self.name} [ComputeNewDirection::initialise()]")
        self.blackboard = self.attach_blackboard_client("clients")
        self.blackboard.register_key(key="planner", access=Access.WRITE)
        self.client = self.blackboard.planner
        self.mission_blackboard = self.attach_blackboard_client("mission")
        self.mission_blackboard.register_key(key="target", access=Access.WRITE)

        # Send action client request
        goal_msg = PlanSpotBodyTask.Goal()
        goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.action = "search"
        self.future = self.client.send_goal_async(goal_msg, feedback_callback=self._feedback)

    def update(self) -> Status:
        """Run the ComputeNewDirection behavior when ticked."""
        self.logger.debug(f"  {self.name} [ComputeNewDirection::update()]")

        if self.future.done():
            result = self.future.result().get_result().result
            if result.success:
                self.mission_blackboard.target = result.goal_pose
                return Status.SUCCESS

            self.logger.error(f"{result.message}")
            return Status.FAILURE

        self.logger.debug(f"  {self.name} [ComputeNewDirection::update()][RUNNING]")
        return Status.RUNNING

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.logger.debug(
            f"  {self.name} [ComputeNewDirection::terminate()]"
            f"[{self.status}->{new_status}]"
        )

    def _feedback(self, feedback_msg: PlanSpotBodyTask.Feedback):
        """Log action feedback for ComputeNewDirection."""
        self.node.get_logger().info(
            feedback_msg.feedback.feedback, throttle_duration_sec=0.1
        )
