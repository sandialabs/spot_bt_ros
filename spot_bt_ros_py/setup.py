"""Setup procedure for spot_bt_ros_py package."""

from glob import glob
import os
from setuptools import setup

PACKAGE_NAME = "spot_bt_ros_py"


setup(
    name=PACKAGE_NAME,
    version="0.2.0",
    packages=[PACKAGE_NAME],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (os.path.join("share", PACKAGE_NAME, "scripts"), glob("scripts/*.py")),
    ],
    install_requires=["setuptools", "py_trees", "py_trees_ros"],
    zip_safe=True,
    maintainer="zmkakis",
    maintainer_email="zmkakis@sandia.gov",
    description="Behavior Tree ROS 2 Python package for the Boston Dynamics' Spot robot.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "spot_arm_demo = scripts.demo_arm:main",
            "spot_fiducial_arm_demo = scripts.demo_fiducial_arm:main",
            "spot_fiducial_demo = scripts.demo_fiducial:main",
            "spot_search_demo = scripts.demo_search:main",
        ],
    },
)
