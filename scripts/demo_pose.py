"""spot_bt_ros Pose Demonstration."""
from __future__ import annotations

import bdai_ros2_wrappers.process as ros_process

import py_trees

from spot_bt_ros.data import Pose
from spot_bt_ros.behaviors.actions.general import ClaimLease
from spot_bt_ros.behaviors.actions.general import RobotDock
from spot_bt_ros.behaviors.actions.general import RobotPowerOff
from spot_bt_ros.behaviors.actions.general import RobotPowerOn
from spot_bt_ros.behaviors.actions.general import RobotUndock
from spot_bt_ros.behaviors.actions.movement import RobotPose
from spot_bt_ros.node import SpotBT
from spot_bt_ros.transform import SpotTF



def create_root() -> py_trees.composites.Sequence:
    root = py_trees.composites.Sequence("DemoPose", memory=True)
    claim_robot = ClaimLease(name="Claim Lease")
    power_on_robot = RobotPowerOn(name="Power ON Spot")
    undock_robot = RobotUndock(name="Undock Spot")
    set_robot_pose = RobotPose(name="Spot Pose")
    dock_robot = RobotDock(name="Dock Spot")
    power_off_robot = RobotPowerOff(name="Power OFF Spot")
    root.add_children(
        [
            claim_robot,
            power_on_robot,
            undock_robot,
            set_robot_pose,
            dock_robot,
            power_off_robot,
        ]
    )
    return root


@ros_process.main()
def main():
    robot = SpotBT()
    tf_vision = SpotTF(main.node)

    # Create behavior tree and blackboard
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    py_trees.blackboard.Blackboard.enable_activity_stream(maximum_size=100)
    blackboard = py_trees.blackboard.Client(name="State")
    blackboard.register_key(key="robot", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="tf_vision", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="pose", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="dock_id", access=py_trees.common.Access.WRITE)
    blackboard.robot = robot
    blackboard.tf_vision = tf_vision
    blackboard.pose = Pose()
    blackboard.pose.set_pose(yaw=0.4, roll=0.0, pitch=0.0)
    blackboard.dock_id = 549

    root = create_root()
    root.setup_with_descendants()
    for i in range(1):
        print(f"\n------- Tick {i} -------")
        root.tick_once()
        print(py_trees.display.unicode_tree(root=root, show_status=True))

    robot.shutdown()


if __name__ == "__main__":
    main()
