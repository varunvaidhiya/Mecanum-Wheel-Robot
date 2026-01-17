"""Data specifications and constants"""

from dataclasses import dataclass
from typing import List, Tuple
import numpy as np

@dataclass
class StateSpec:
    """Robot state specification"""
    names: List[str]
    ranges: List[Tuple[float, float]]  # (min, max)
    units: List[str]
    dim: int

# State: [x, y, theta, vel_x, vel_y, omega, motor_fl, motor_fr, motor_bl, motor_br]
STATE_SPEC = StateSpec(
    names=['x', 'y', 'theta', 'vel_x', 'vel_y', 'omega', 
           'motor_fl', 'motor_fr', 'motor_bl', 'motor_br'],
    ranges=[
        (-10.0, 10.0),   # x (m)
        (-10.0, 10.0),   # y (m)
        (-np.pi, np.pi), # theta (rad)
        (-2.0, 2.0),     # vel_x (m/s)
        (-2.0, 2.0),     # vel_y (m/s)
        (-3.0, 3.0),     # omega (rad/s)
        (-255, 255),     # motor FL PWM
        (-255, 255),     # motor FR PWM
        (-255, 255),     # motor BL PWM
        (-255, 255),     # motor BR PWM
    ],
    units=['m', 'm', 'rad', 'm/s', 'm/s', 'rad/s', 
           'pwm', 'pwm', 'pwm', 'pwm'],
    dim=10
)

@dataclass
class ActionSpec:
    """Robot action specification"""
    names: List[str]
    ranges: List[Tuple[float, float]]
    units: List[str]
    dim: int

# Action: [linear_x, linear_y, angular_z]
ACTION_SPEC = ActionSpec(
    names=['linear_x', 'linear_y', 'angular_z'],
    ranges=[
        (-2.0, 2.0),   # linear_x (m/s)
        (-2.0, 2.0),   # linear_y (m/s)
        (-3.0, 3.0),   # angular_z (rad/s)
    ],
    units=['m/s', 'm/s', 'rad/s'],
    dim=3
)

@dataclass
class CameraConfig:
    """Camera configuration"""
    name: str
    topic: str
    resolution: Tuple[int, int]  # (height, width)
    fps: int
    encoding: str

DEFAULT_CAMERAS = [
    CameraConfig('front', '/camera/front/image_raw/compressed', (480, 640), 30, 'rgb8'),
    CameraConfig('wrist', '/camera/wrist/image_raw/compressed', (480, 640), 30, 'rgb8'),
]
