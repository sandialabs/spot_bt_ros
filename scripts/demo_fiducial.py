"""spot_bt_ros Fiducial Movement Demonstration."""
from __future__ import annotations

import time

import bdai_ros2_wrappers.process as ros_process

import py_trees

from spot_bt_ros.data import Blackboards
from spot_bt_ros.data import Pose
from spot_bt_ros.behaviors.actions.movement import MoveToFiducial
from spot_bt_ros.behaviors.actions.movement import RobotPose
from spot_bt_ros.composites.selector import create_generic_fiducial_selector
from spot_bt_ros.composites.sequence import create_dock_sequence
from spot_bt_ros.composites.sequence import create_undock_sequence
from spot_bt_ros.node import SpotBT
from spot_bt_ros.tick import generic_pre_tick_handler
from spot_bt_ros.transform import SpotTF



def create_root() -> py_trees.composites.Sequence:
    """Create the root for the Autonomy capability."""
    root = py_trees.composites.Sequence("DemoFiducial", memory=True)
    set_robot_pose = RobotPose(name="Spot Pose")

    root.add_child(create_undock_sequence())
    root.add_children(
        [
            set_robot_pose,
        ]
    )
    root.add_child(create_generic_fiducial_selector())
    root.add_children(
        [
            MoveToFiducial(name="Move to Fiducial"),
        ]
    )
    root.add_child(create_dock_sequence())

    py_trees.display.render_dot_tree(root)

    return root


@ros_process.main()
def main():
    robot = SpotBT()
    tf_odom = SpotTF(main.node)
    tf_vision = SpotTF(main.node)

    # Create behavior tree and blackboard
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    py_trees.blackboard.Blackboard.enable_activity_stream(maximum_size=100)
    blackboard = Blackboards()
    blackboard.state = py_trees.blackboard.Client(name="State")
    blackboard.state.register_key(key="robot", access=py_trees.common.Access.WRITE)
    blackboard.state.register_key(key="tf_odom", access=py_trees.common.Access.WRITE)
    blackboard.state.register_key(key="tf_vision", access=py_trees.common.Access.WRITE)
    blackboard.state.register_key(key="dock_id", access=py_trees.common.Access.WRITE)
    blackboard.state.register_key(key="pose", access=py_trees.common.Access.WRITE)
    blackboard.state.robot = robot
    blackboard.state.tf_odom = tf_odom
    blackboard.state.tf_vision = tf_vision
    blackboard.state.dock_id = 549
    blackboard.state.pose = Pose()
    blackboard.state.pose.set_pose(yaw=0.4, roll=0.0, pitch=0.0)
    blackboard.perception = py_trees.blackboard.Client(name="Perception")
    blackboard.perception.register_key(
        key="fiducials", access=py_trees.common.Access.WRITE
    )
    blackboard.perception.register_key(
        key="world_objects", access=py_trees.common.Access.WRITE
    )
    blackboard.perception.fiducials = None
    blackboard.perception.world_objects = None

    # create visitors
    debug_visitor = py_trees.visitors.DebugVisitor()

    # Enable tree stewardship
    root = create_root()
    behavior_tree = py_trees.trees.BehaviourTree(root)
    behavior_tree.add_pre_tick_handler(generic_pre_tick_handler)
    behavior_tree.visitors.append(debug_visitor)
    # behavior_tree.add_post_tick_handler(generic_post_tick_handler)
    behavior_tree.setup(timeout=15)
    root.setup_with_descendants()

    # Begin autonomy loop
    while True:
        try:
            behavior_tree.tick()
            time.sleep(0.1)
        except KeyboardInterrupt:
            robot.shutdown()
            break

    robot.shutdown()


if __name__ == "__main__":
    main()
