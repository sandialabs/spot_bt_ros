"""Setup procedure for spot_bt_ros_node package."""

# from glob import glob
# import os
from setuptools import setup

PACKAGE_NAME = "spot_bt_ros_node"


setup(
    name=PACKAGE_NAME,
    version="0.2.0",
    packages=[PACKAGE_NAME],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        # (os.path.join("share", PACKAGE_NAME, "launch"), glob("launch/*.launch.py")),
        # (os.path.join("share", PACKAGE_NAME, "scripts"), glob("scripts/*.py")),
    ],
    install_requires=["setuptools", "py_trees", "py_trees_ros"],
    zip_safe=True,
    maintainer="zmkakis",
    maintainer_email="zmkakis@sandia.gov",
    description="Behavior Tree ROS 2 node package for the Boston Dynamics' Spot robot.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "spot_controller_node = spot_bt_ros_node.controller:main",
            "spot_perception_node = spot_bt_ros_node.perception:main",
            "spot_tf_node = spot_bt_ros_node.transform:main",
            "simple_planner = spot_bt_ros_node.planner.simple:main",
        ],
    },
)
