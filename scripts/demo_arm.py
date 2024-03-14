"""spot_bt_ros Arm Demonstration."""
from __future__ import annotations

import time

import bdai_ros2_wrappers.process as ros_process

from bosdyn.client.math_helpers import SE3Pose

import py_trees

from spot_bt_ros.data import Blackboards
from spot_bt_ros.data import ArmPose, ArmPoses
from spot_bt_ros.behaviors.actions.general import ClaimLease, RobotPowerOff, RobotPowerOn
from spot_bt_ros.behaviors.actions.arm import ArmStow, ArmUnstow
from spot_bt_ros.behaviors.actions.arm import CloseGripper, OpenGripper
from spot_bt_ros.behaviors.actions.arm import ArmTrajectory, ArmTrajectories
from spot_bt_ros.composites.sequence import create_dock_sequence
from spot_bt_ros.composites.sequence import create_undock_sequence
from spot_bt_ros.node import SpotBT
from spot_bt_ros.tick import generic_pre_tick_handler
from spot_bt_ros.transform import SpotTF



def create_root() -> py_trees.composites.Sequence:
    """Create the root for the Autonomy capability."""
    root = py_trees.composites.Sequence("DemoArm", memory=True)

    root.add_child(create_undock_sequence())
    root.add_children(
        [
            ClaimLease(name="claim_lease"),
            RobotPowerOn(name="power_on"),
            ArmUnstow(name="unstow_arm"),
            # ArmTrajectory(name="move_arm_to_postion"),
            ArmTrajectories(name="move_arm_to_postions"),
            OpenGripper(name="open_gripper"),
            CloseGripper(name="close_gripper"),
            ArmStow(name="stow_arm"),
            RobotPowerOff(name="power_off"),
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
    blackboard.arm = py_trees.blackboard.Client(name="Arm")
    blackboard.arm.register_key(key="target", access=py_trees.common.Access.WRITE)
    blackboard.arm.register_key(key="command", access=py_trees.common.Access.WRITE)
    # blackboard.arm.target = ArmPose(
    #     SE3Pose(x=1.0, y=0.0, z=0.5, rot=[1.0, 0.0, 0.0, 0.0]),
    #     False,
    # )
    blackboard.arm.target = ArmPoses([
        ArmPose(
            SE3Pose(x=1.0, y=0.0, z=0.0, rot=[1.0, 0.0, 0.0, 0.0]),
            False,
        ),
        ArmPose(
            SE3Pose(x=1.0, y=1.0, z=0.0, rot=[1.0, 0.0, 0.0, 0.0]),
            False,
        ),
        ArmPose(
            SE3Pose(x=1.0, y=0.0, z=1.0, rot=[1.0, 0.0, 0.0, 0.0]),
            False,
        ),
    ])
    blackboard.arm.command = None

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
    try:
        for i in range(1):
            print(f"\n------- Tick {i} -------")
            behavior_tree.tick()
            time.sleep(0.1)

    except KeyboardInterrupt:
        robot.shutdown()

    robot.shutdown()


if __name__ == "__main__":
    main()
