"""Setup procedure for spot_bt_ros package."""
import os
from setuptools import setup

from glob import glob

PACKAGE_NAME = "spot_bt_ros"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=[PACKAGE_NAME],
    data_files=[
       ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (os.path.join("share", PACKAGE_NAME, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", PACKAGE_NAME, "scripts"), glob("scripts/*.py")),
    ],
    install_requires=["setuptools", "py_trees", "py_trees_ros"],
    zip_safe=True,
    maintainer="zmkakis",
    maintainer_email="zmkakis@sandia.gov",
    description="Behavior Tree ROS 2 package for Boston Dynamics' Spot robot.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "spot_arm_demo = scripts.demo_arm:main",
            "spot_fiducial_demo = scripts.demo_fiducial:main",
            "spot_pose_demo = scripts.demo_pose:main",
        ],
    },
)
