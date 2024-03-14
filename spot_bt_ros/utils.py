from __future__ import annotations

from bosdyn import geometry
from bosdyn.api import geometry_pb2
from bosdyn.api import trajectory_pb2
from bosdyn.api.spot import robot_command_pb2
from bosdyn.client.math_helpers import Quat

import numpy as np


def get_desired_angle(xhat: np.ndarray) -> float:
    """Compute heading based on the vector from robot to object."""
    zhat = [0.0, 0.0, 1.0]
    yhat = np.cross(zhat, xhat)
    mat = np.array([xhat, yhat, zhat]).transpose()
    return Quat.from_matrix(mat).to_yaw()


def get_default_mobility_parameters(
    x_velocity_limit: float = 0.5,
    y_velocity_limit: float = 0.5,
    angular_velocity_limit: float = 1.0,
) -> robot_command_pb2.MobilityParams:
    """Generate default mobility parameters for Spot."""
    obstacles = robot_command_pb2.ObstacleParams(
        disable_vision_body_obstacle_avoidance=True,
        disable_vision_foot_obstacle_avoidance=True,
        disable_vision_foot_constraint_avoidance=True,
        obstacle_avoidance_padding=0.001,
    )
    body_control = set_default_body_control()
    speed_limit = geometry_pb2.SE2VelocityLimit(
        max_vel=geometry_pb2.SE2Velocity(
            linear=geometry_pb2.Vec2(x=x_velocity_limit, y=y_velocity_limit),
            angular=angular_velocity_limit,
        )
    )
    return robot_command_pb2.MobilityParams(
        obstacle_params=obstacles,
        vel_limit=speed_limit,
        body_control=body_control,
    )


def set_default_body_control():
    """Set default body control parameters to current body position."""
    footprint_R_body = geometry.EulerZXY()
    position = geometry_pb2.Vec3(x=0.0, y=0.0, z=0.0)
    rotation = footprint_R_body.to_quaternion()
    pose = geometry_pb2.SE3Pose(position=position, rotation=rotation)
    point = trajectory_pb2.SE3TrajectoryPoint(pose=pose)
    trajectory = trajectory_pb2.SE3Trajectory(points=[point])
    return robot_command_pb2.BodyControlParams(base_offset_rt_footprint=trajectory)
