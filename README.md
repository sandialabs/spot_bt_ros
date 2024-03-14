# spot_bt_ros

`spot_bt_ros` is a ROS 2 Python Behavior Trees package for creating autonomous behavior for the [Boston Dynamics' Spot](https://bostondynamics.com/products/spot/) robot.  Behavior Trees allow users to structure numerous actions and conditions to assist a robotic agent in switching between different tasks.

To learn more about Behavior Trees, we suggest the following resources:

- [*Behavior Trees in Robotics and AI*](https://arxiv.org/abs/1709.00084) - (Chapters 1-3)
- [*Introduction to Behavior Trees*](https://roboticseabass.com/2021/05/08/introduction-to-behavior-trees/)
- [`py_trees` *Documentation*](https://py-trees.readthedocs.io/en/devel/introduction.html)

This version of `spot_bt_ros` works with [ROS 2 Humble](https://docs.ros.org/en/humble/index.html) and Ubuntu 22.04. Other versions have not been tested!

## Installation

Since this is a ROS 2 package, it can be added to an existing ROS workspace for compilation. If you are starting from scratch, first install the following the dependencies:

```bash
sudo apt install ros-humble-py-trees ros-humble-py-trees-ros
```

Next, create the workspace along with a `src` directory:

```bash
mkdir -p colcon_ws/src
```

`cd` into the `src` folder of the workspace and clone the `spot_ros2` package (along with submodules) from the Boston Dynamics AI Institute's [GitHub repo](https://github.com/bdaiinstitute/spot_ros2). Follow the instructions within that repo to begin installation of `spot_ros2` dependencies. Once complete, clone `spot_bt_ros` into the workspace's `src` directory.

```bash
cd colcon_ws/src
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
- `fiducial`
- `pose`

To the default option, execute the following:

```bash
ros2 launch spot_bt_ros demo.launch.py
```

Use the `demo_type` option with the name of the other example to run.

```bash
ros2 launch spot_bt_ros demo.launch.py 'demo_type:=fiducial'
```

## The Blackboard

Maintaining state between behavior tree actions and conditions requires the use of a Blackboard. When creating a script, you will need to create a blackboard, register the variable whose state you wish to maintain, and initialize the variable. We can do this with the following code:

```python
import py_trees

blackboard = py_trees.blackboard.Client(name="State")
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
        self.blackboard = self.attach_blackboard_client("State")
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

### Using Multiple Blackboards

Depending on the context, it may be necessary to have multiple blackboards to better segment variables based on their use case. The `spot_bt` package uses 4 different blackboards with unique variables associated with a specific Spot functionality defined below.

#### `State` Blackboard

- `robot`
  - This will contain `Robot` object used to create clients for the Spot SDK's client-server architecture. This is the most used blackboard variable!
- `dock_id`
  - The integer value associated with the fiducial marker used to dock Spot.
- `state`
  - The current state of Spot within the world. This will capture many of the transformations used to calculate distances between two points.
- `pose`
  - A `spot_bt.data.Pose` object used to store poses and create protos to send to Spot.

#### `Arm` Blackboard

- `target`
  - A `spot_bt.data.ArmPose` or `spot_bt.data.ArmPoses` object that stores a single or multiple `bosdyn.client.math_helpers.SE3Pose` objects, respectively, for Spot's arm to follow.

#### `Graph` Blackboard

- `recording_client`
- `recording_environment`
- `graph_nav_client`
- `map_processing_client`


#### `Perception` Blackboard

- `fiducials`
  - A `list` of fiducials found in the environment from Spot's current camera.
- `model`
  - A work in progress, but this should store a PyTorch model object for use by Spot.
- `camera`
  - The camera used to capture images.
- `images`
  - A `list` containing images taken by Spot's cameras. This does not store historical pictures, just the most recent images taken. 
- `depth_camera`
  - The depth camera used to capture images.
- `depth_images`
  - A `list` containing depth images taken by Spot's depth cameras. This does not store historical pictures, just the most recent images taken.
- `world_objects`
  - A `list` of world objects found in the environment from Spot's current camera.

A helper dataclass named `spot_bt.data.Blackboards` is available and used through out this repository to store the multiple blackboards mentioned above. Users may add their own unique variables to each of the blackboards to suite their requirements.


## Citation

If you use `spot_bt` for your work, please cite the following paper:

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