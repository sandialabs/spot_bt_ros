# spot_bt_ros

`spot_bt_ros` is a ROS 2 Python Behavior Trees package for creating autonomous behavior for the [Boston Dynamics' Spot](https://bostondynamics.com/products/spot/) robot.  Behavior Trees allow users to structure numerous actions and conditions to assist a robotic agent in switching between different tasks.

To learn more about Behavior Trees, we suggest the following resources:

- [*Behavior Trees in Robotics and AI*](https://arxiv.org/abs/1709.00084) - (Chapters 1-3)
- [*Introduction to Behavior Trees*](https://roboticseabass.com/2021/05/08/introduction-to-behavior-trees/)
- [`py_trees` *Documentation*](https://py-trees.readthedocs.io/en/devel/introduction.html)
- [`py_trees_ros` *Documentation*](https://py-trees-ros.readthedocs.io/en/devel/)

This version of `spot_bt_ros` works with [ROS 2 Humble](https://docs.ros.org/en/humble/index.html) and Ubuntu 22.04. Other versions have not been tested!

## Installation

Since this is a ROS 2 package, it can be added to an existing ROS workspace for compilation. If you are starting from scratch, first install the following the dependencies:

```bash
sudo apt install ros-humble-py-trees ros-humble-py-trees-ros ros-humble-behaviortree-cpp
```

Next, create the workspace along with a `src` directory:

```bash
mkdir -p colcon_ws/src
```

`cd` into the `src` folder of the workspace and clone the `spot_ros2` package (along with submodules) from the Boston Dynamics AI Institute's [GitHub repo](https://github.com/bdaiinstitute/spot_ros2). Follow the instructions within that repo to begin installation of `spot_ros2` dependencies.

```bash
cd colcon_ws/src
git clone https://github.com/bdaiinstitute/spot_ros2.git
cd spot_ros2
git submodule init
git submodule update
./install_spot_ros2.sh
```

In addition, download the `BehaviorTree.ROS2` [GitHub repo]() package to use the C++ behaviors.

```bash
cd colcon_ws/src
git clone https://github.com/BehaviorTree/BehaviorTree.ROS2.git
```

Once complete, clone `spot_bt_ros` into the workspace's `src` directory.

```bash
git clone https://github.com/sandialabs/spot_bt_ros.git
```

Finally, build the workspace using `colcon build` outside of the `src` directory and then source the install.

```bash
cd colcon_ws
colcon build --symlink-install
. install/setup.bash
```

## Examples

The package provides 3 distinct examples for users to try and template their own versions. The examples are options within the `demo.launch.py` launch file.

- `arm` (*DEFAULT*)
  - Arm motion using different end points.
- `fiducial`
  - A fiducial move behavior, where Spot will undock, pose, move to a fiducial and then return to the dock.
- `fiducial_arm`
  - Same as the `fiducial` demo, but Spot will deploy his arm to touch the fiducial.
- `search`
  - Spot will undergo a random search motion to find a fiducial marker and then return to dock.

To the default option, execute the following:

```bash
ros2 launch spot_bt_ros_bringup demo.launch.py
```

Use the `demo_type` option with the name of the other example to run.

```bash
ros2 launch spot_bt_ros_bringup demo.launch.py 'demo_type:=fiducial'
```

## The Blackboard

Maintaining state between behavior tree actions and conditions requires the use of a Blackboard. When creating a script, you will need to create a blackboard, register the variable whose state you wish to maintain, and initialize the variable. We can do this with the following code:

```python
import py_trees

blackboard = py_trees.blackboard.Client(name="mission")
blackboard.register_key(key="dock_id", access=py_trees.common.Access.WRITE)
blackboard.dock_id = 549
```

When attempting to access them in a custom action or condition, you will need to attach to the same blackboard created and register the desired variable within the `intialise()` method call.

```python
import py_trees

class MyCustomAction(py_trees.behaviour.Behaviour):
    def __init__(self, name:str):
        super().__init__(name)
        self.blackboard = None
        self.dock_id = None

    def initialise(self):
        self.blackboard = self.attach_blackboard_client("mission")
        self.blackboard.register_key(
            key="dock_id", access=py_trees.common.Access.READ
        )
        self.dock_id = self.blackboard.dock_id

    ...
```

Now the variable `dock_id` is accessable by the remainder of the `MyCustomAction` class. We assigned the desired blackboard variable to a custom class attribute `self.dock_id` for clarity; however, users may use the `self.blackboard.dock_id` variable instead. Note that the `access` option in the `register_key` method will determine how the variable may be used within the class with `READ` and `WRITE` meaning *read-only* and *writtable*, respectively. If you make the variable writtable, you will need to save the variable *within* the blackboard attribute you registered it from. For example, if we want to change the `dock_id`, we would assign it like this:

```python
self.blackboard.dock_id = 100
```

### **New** Multiple Blackboards Info

We segment variables into multiple blackboards to better manage their use during operation. The `spot_bt_ros_py` package uses 4 different blackboards with unique variables associated with a specific Spot functionality defined below. It should be noted that the `spot_bt_ros_cpp` package does not differentiate between blackboard clients and all values are available within the same blackboard.

#### `clients` Blackboard

- `controller`: `rclpy.ActionClient`
  - Action client that connects to the `spot_bt_ros_node` motion controller server (`controller.py`).  This use the following action topic: `/spot/controller/body`.
- `arm_controller`: `rclpy.ActionClient`
  - Action client that connects to the `spot_bt_ros_node` motion controller server (`controller.py`).  This use the following action topic: `/spot/controller/body`.
- `planner`: `rclpy.ActionClient`
  - By default, it is an action client that connects to the `spot_bt_ros_node` simple planner server (`planners/simple.py`). This can be changed to any planner the user wishes to use; however, we use the following topics to plan body motion: `/spot/planner/body`.
- `arm_planner`: `rclpy.ActionClient`
  - By default, it is an action client that connects to the `spot_bt_ros_node` simple planner server (`planners/simple.py`). This can be changed to any planner the user wishes to use; however, we use the following topics to plan arm motion: `/spot/planner/arm`.

#### `status` Blackboard

- `state`: `spot_bt_ros_py/SpotState`
  - A collection state booleans related to Spot such as `standing`, `gripper_open`, `powered_on`, etc.
- `battery`: `spot_msgs/msg/BatteryStateArray`
  - Battery state of Spot.
- `behavior_faults`: `spot_msgs/msg/BehaviorFaultState`
  - Any behavior faults that are occuring with Spot.
- `estop`: `spot_msgs/msg/EStopStateArray`
  - Status of Spot's EStop state (`hard`, `gentle`, `released`).
- `leases`: `spot_msgs/msg/LeaseArray`
  - Status of Spot's lease.
- `metrics`: `spot_msgs/msg/Metrics`
  - Spot metrics.
- `system_faults`: `spot_msgs/msg/SystemFaultState`
  - Any system faults occuring with Spot.
- `wifi`: `spot_msgs/msg/WiFiState`
  - If connected to Spot using WiFi, this will show the current state of that connection. 

#### `mission` Blackboard

- `dock_id` : `int`
  - The fiducial ID associated with Spot's dock.
- `target` : `geometry_msgs.msg.PoseStamped`
  - The goal or target for Spot's navigation.
- `fiducials` : `list[bosdyn_api_msgs.msg.WorldObject]`
  - A list of fiducial markers detected by Spot. This is a subset of all world objects Spot can detect. In C++, this is an `std::vector`.
- `world_objects` : `list[bosdyn_api_msgs.msg.WorldObject]`
  - A list of *all* world objects detect by Spot. In C++, this is an `std::vector`.
- `backup_amount` : `float`
  - The amount Spot will back up (in meters) during a fallback procedure.
- `spin_amount` : `float`
  - The amount Spot will spin (in radiains) during a fallback procedure.
- `wait_duration` : `float`
  - The amount Spot will wait before continuing with the remainder of the tree.

## Available Actions

  NOTE: Actions containing `clients` blackboard variables are denoted with the '`*`' symbol.

### Python

| Action         | Module             | Blackboard | Parameters       | Type(s)  |
| -------------- | ------------------ | ---------- | ---------------- | -------- |
| `Dock`         | `general.docking`  | `mission`,`status` | `dock_id`,`state.docked` | `int`,`bool` |
| `Undock`       | `general.docking`  | `status`   | `state.docked` | `bool` | 
| `Backup`       | `general.fallback` | `*`,`mission` | `backup_amount` | `float` |
| `Spin`         | `general.fallback` | `*`,`mission` | `spin_amount` | `float` |
| `Wait`         | `general.fallback` | `*`,`mission` | `wait_duration` | `float` |
| `PowerOn`      | `general.power`    | `status`   | `state.powered_on` | `bool` |
| `PowerOff`     | `general.power`    | `status`   | `state.powered_on` | `bool` |
| `Stop`         | `general.safety`   | `status`   | `state.stopped`  | `bool` |
| `EStopHard`    | `general.safety`   |            |                  |          |
| `EStopGentle`  | `general.safety`   |            |                  |          |
| `EstopRelease` | `general.safety`   |            |                  |          |
| `SelfRight`    | `general.safety`   |            |                  |          |
| `ClaimLease`   | `general.state`    | `status`   | `state.lease_claimed` | `bool` |
| `ReleaseLease` | `general.state`    | `status`   | `state.lease_claimed` | `bool` |
| `MoveToTarget` | `movement.move`    | `*`,`mission` | `target`  | `geometry_msgs.msg.PoseStamepd` |
| `FollowPath`   | `movement.move`    | `*`        |                  |          |
| `Crouch`       | `movement.move`    | `status`   | `state.crouching` | `bool` |
| `Rollover`     | `movement.move`    |            |                  |          |
| `ComputePathToPose` | `movement.planner` | `*`,`mission` | `target` | `geometry_msgs.msg.PoseStamped` |
| `ComputePathToFiducial` | `movement.planner` | `*`,`mission` | `dock_id`,`fiducials`,`target` | `int`,`list[bosdyn_api_msgs.msg.WorldObject]`,`geometry_msgs.msg.PoseStamped` |
| `ComputeNewWaypoint` | `movement.planner` | `*`,`mission` | `target` | `geometry_msgs.msg.PoseStamped` |
| `Sit`          | `movement.pose` | `status`   | `state.standing` | `bool` |
| `Stand`        | `movement.pose` | `status`   | `state.standing` | `bool` |
| `Wait`         | `movement.pose` | `mission`  | `wait_time`      | `float` |
| `ExecuteSearch` | `movement.search` | `*`        |                  |          |
| `DetectFiducialMarkers` | `perception.fiducial` | `mission` | `dock_id`,`fiducials` | `int`,`list[bosdyn_api_msgs.msg.WorldObject]` |
| `DetectWorldObjects` | `perception.objects` | `mission` | `world_objects` | `list[bosdyn_api_msgs.msg.WorldObject]` |

### C++ (`BehaviorTree.CPP` plugins)

| Action                  | Module       | Inputs ports     | Outputs ports    | Type(s)  |
| ----------------------- | ------------ | ---------------- | ---------------- | -------- |
| `Dock`                  | `general`    | `dock_id`,`docked` | `docked` | `int`, `bool` |
| `Undock`                | `general`    | `docked`         | `docked`         | `bool`   |
| `Backup`                | `general`    | `backup_amount`  |                  | `double` |
| `Spin`                  | `general`    | `spin_amount`    |                  | `double` |
| `Wait`                  | `general`    | `wait_duration`  |                  | `double` |
| `PowerOn`               | `general`    | `powered_on`     | `powered_on`     | `bool`   |
| `PowerOff`              | `general`    | `powered_on`     | `powered_on`     | `bool`   |
| `Stop`                  | `general`    |                  |                  |          |
| `EStopHard`             | `general`    |                  |                  |          |
| `EStopGentle`           | `general`    |                  |                  |          |
| `EstopRelease`          | `general`    |                  |                  |          |
| `SelfRight`             | `general`    |                  |                  |          |
| `ClaimLease`            | `general`    | `lease_claimed`  | `lease_claimed`  | `bool`   |
| `ReleaseLease`          | `general`    | `lease_claimed`  | `lease_claimed`  | `bool`   |
| `Crouch`                | `movement`   |                  |                  |          |
| `MoveToTarget`          | `movement`   |                  |                  |          |
| `Sit`                   | `movement`   | `standing`       | `standing`       | `bool`   |
| `Stand`                 | `movement`   | `standing`       | `standing`       | `bool`   |
| `DetectFiducialMarker`  | `perception` | | `fiducials` | `std::vector<bosdyn_api_msgs::msg::WorldObject>` |
| `DetectWorldObjects`    | `perception` | | `world_objects` | `std::vector<bosdyn_api_msgs::msg::WorldObject>` |
| `ComputeNewWaypoint`    | `planner`    | | `target` | `geometry_msgs::msg::PoseStamped` |
| `ComputePathToFiducial` | `planner`    | `dock_id`,`fiducials` | `target` | `int`,`std::vector<bosdyn_api_msgs::msg::WorldObject>`,`geometry_msgs::msg::PoseStamped` |
| `ComputePathToPose`     | `planner`    | `target` | `target` | `geometry_msgs::msg::PoseStamped` |
| `ArmCarry`              | `arm`        |                  |                  |          |
| `ArmFreeze`             | `arm`        |                  |                  |          |
| `ArmStow`               | `arm`        | `arm_stowed`     | `arm_stowed`     | `bool`   |
| `ArmUnstow`             | `arm`        | `arm_stowed`     | `arm_stowed`     | `bool`   |
| `GripperClose`          | `arm`        | `gripper_open`   | `gripper_open`   | `bool`   |
| `GripperOpen`           | `arm`        | `gripper_open`   | `gripper_open`   | `bool`   |

## Available Conditions

### Python

| Condition         | Module              | Blackboard | Parameters           | Type(s) |
| ----------------- | ------------------- | ---------- | -------------------- | ------- |
| `IsArmStowed`     | `arm.general`       | `status`   | `state.arm_stowed`   | `bool`  |
| `IsArmUnstowed`   | `arm.general`       | `status`   | `state.arm_stowed`   | `bool`  |
| `IsArmDeployed`   | `arm.general`       | `status`   | `state.arm_stowed`   | `bool`  |
| `IsGripperOpen`   | `arm.gripper`       | `status`   | `state.gripper_open` | `bool`  |
| `IsGripperClosed` | `arm.gripper`       | `status`   | `state.gripper_open` | `bool`  |
| `IsDocked`        | `general.docking`   | `status`   | `state.docked`       | `bool`  |
| `IsUndocked`      | `general.docking`   | `status`   | `state.docked`       | `bool`  |
| `IsPoweredOn`     | `general.power`     | `status`   | `state.powered_on`   | `bool`  |
| `IsPoweredOff`    | `general.power`     | `status`   | `state.powered_on`   | `bool`  |
| `IsLeaseClaimed`  | `general.state`     | `status`   | `state.lease_claimed` | `bool`  |
| `IsLeaseReleased` | `general.state`     | `status`   | `state.lease_claimed` | `bool`  |
| `IsSitting`       | `movement.pose`     | `status`   | `state.standing`     | `bool`  |
| `IsStanding`      | `movement.pose`     | `status`   | `state.standing`     | `bool`  |
| `IsAnyFiducialMarkerDetected` | `perception.fiducial` | `mission` | `dock_id`,`fiducials` | `int`,`list[bosdyn_api_msgs.msg.WorldObject]`  |
| `IsDockFiducialDetected` | `perception.fiducial` | `mission` | `dock_id`,`fiducials` | `int`,`list[bosdyn_api_msgs.msg.WorldObject]`  |
| `IsSpecificFiducialMarkerDetected` | `perception.fiducial` | `mission` | `fiducials`,`target_fiducials` | `list[bosdyn_api_msgs.msg.WorldObject]`,`list[bosdyn_api_msgs.msg.WorldObject]`  |

### C++ (`BehaviorTree.CPP` plugins)

***More conditions coming soon.***

## Available Composites (Control Nodes)

### Python

| Function          | Module    | Control Type  | Arguments          | Type(s) |
| ----------------- | --------- | ------------- | ------------------ | ------- |
| `create_spot_status_parallel` | `parallel` | Parallel | `name` | `str` |
| `create_rgb_camera_parallel` | `parallel` | Parallel | `name`,`has_arm` | `str`,`bool` |
| `create_depth_camera_parallel` | `parallel` | Parallel | `name`,`has_arm` | `str`,`bool` |
| `create_generic_fiducial_selector` | `selector` | Fallback | `name`,`memory`,`no_dock` | `str`,`bool`,`bool` |
| `create_lease_claim_selector` | `selector` | Fallback | `name`,`memory` | `str`,`bool` |
| `create_lease_release_selector` | `selector` | Fallback | `name`,`memory` | `str`,`bool` |
| `create_dock_selector` | `selector` | Fallback | `name`,`memory` | `str`,`bool` |
| `create_undock_selector` | `selector` | Fallback | `name`,`memory` | `str`,`bool` |
| `create_power_off_selector` | `selector` | Fallback | `name`,`memory` | `str`,`bool` |
| `create_power_on_selector` | `selector` | Fallback | `name`,`memory` | `str`,`bool` |
| `create_sitting_selector` | `selector` | Fallback | `name`,`memory` | `str`,`bool` |
| `create_standing_selector` | `selector` |  Fallback | `name`,`memory` | `str`,`bool` |
| `create_dock_sequence` | `sequence` | Sequence | `name`,`memory`,`release_lease` | `str`,`bool`,`bool` |
| `create_undock_sequence` | `sequence` | Sequence | `name`,`memory` | `str`,`bool` |
| `create_spot_status_sequence` | `sequence` | Sequence | `name`,`memory` | `str`,`bool` |
| `create_rgb_camera_sequence` | `sequence` | Sequence | `name`,`memory`,`has_arm` | `str`,`bool`,`bool` |
| `create_depth_camera_sequence` | `sequence` | Sequence | `name`,`memory`,`has_arm` | `str`,`bool`,`bool` |

### C++ (`BehaviorTree.CPP` plugins)

***More composites (control nodes) coming soon.***

## Available Behaviors

These are available control sequences and conditions that constitute specific a behavior for Spot. Examples of these used are in demos located in this repository's `scripts` directory.

### Python

| Behavior             | Module    | Arguments          | Type(s) |
| -------------------- | --------- | ------------------ | ------- |
| `undock`             | `general` |                    |         |
| `dock`               | `general` |                    |         |
| `sit_and_power_off`  | `general` |                    |         |
| `power_on_and_stand` | `general` |                    |         |

### C++ (`BehaviorTree.CPP` plugins)

***More behaviors coming soon.***

## Citation

If you use `spot_bt_ros` for your work, please cite the following paper:

```
@article{SHOMAN2024110398,
    title = {Machine learning at the edge to improve in-field safeguards inspections},
    journal = {Annals of Nuclear Energy},
    volume = {200},
    pages = {110398},
    year = {2024},
    issn = {0306-4549},
    doi = {https://doi.org/10.1016/j.anucene.2024.110398},
    url = {https://www.sciencedirect.com/science/article/pii/S0306454924000604},
    author = {Nathan Shoman and Kyle Williams and Burzin Balsara and Adithya Ramakrishnan and Zahi Kakish and Jamie Coram and Philip Honnold and Tania Rivas and Heidi Smartt},
    keywords = {Nonproliferation, International Nuclear Safeguards, Machine learning, }
}
```