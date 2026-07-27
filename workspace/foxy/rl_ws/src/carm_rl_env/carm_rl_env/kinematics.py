import math

import numpy as np


ARM_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")

JOINT_LOWER = np.array([-2.9671, 0.0, -3.1416, -2.6704, -1.5708, -2.8275], dtype=np.float32)
JOINT_UPPER = np.array([2.9671, 3.1416, 0.0, 2.6704, 1.5708, 2.8275], dtype=np.float32)

JOINT_ORIGINS = (
    ((0.0, 0.0, 0.157), (0.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (1.5708, 0.0, 3.1416)),
    ((0.35, 0.0, 0.0), (0.0, 0.0, 1.5708)),
    ((0.079, 0.242, 0.0), (-1.5708, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (1.5708, 0.0, 0.0)),
    ((0.0, 0.108, 0.0), (-1.5708, 0.0, 0.0)),
)

TCP_OFFSET = np.array([0.064891, 0.005939, -0.184486], dtype=np.float32)


def _rot_x(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float32)


def _rot_y(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float32)


def _rot_z(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float32)


def _rpy_matrix(rpy):
    roll, pitch, yaw = rpy
    return _rot_z(yaw) @ _rot_y(pitch) @ _rot_x(roll)


def _transform(xyz, rpy):
    out = np.eye(4, dtype=np.float32)
    out[:3, :3] = _rpy_matrix(rpy)
    out[:3, 3] = np.asarray(xyz, dtype=np.float32)
    return out


def _joint_rotation(angle):
    out = np.eye(4, dtype=np.float32)
    out[:3, :3] = _rot_z(float(angle))
    return out


def forward_tcp_position(joint_positions):
    transform = np.eye(4, dtype=np.float32)
    for joint_value, (xyz, rpy) in zip(joint_positions, JOINT_ORIGINS):
        transform = transform @ _transform(xyz, rpy) @ _joint_rotation(joint_value)
    tcp = transform @ np.array([TCP_OFFSET[0], TCP_OFFSET[1], TCP_OFFSET[2], 1.0], dtype=np.float32)
    return tcp[:3].astype(np.float32)
