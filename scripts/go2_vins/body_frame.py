"""Rigid conversion from the patched VINS IMU body to the Go2 base.

T_BI maps IMU coordinates into base: p_B = R_BI p_I + t_BI.
VINS linear velocity is already expressed in I (not world). Units: m, s, rad.
This module has no ROS, SDK, device or motion dependency.
"""

import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"{name}: expected {size} finite numbers")
    return result


def rotation_xyzw(value):
    q = vector(value, 4, "quaternion_xyzw")
    if abs(np.linalg.norm(q) - 1.0) > 1e-3:
        raise ValueError("quaternion must be normalized (xyzw)")
    return Rotation.from_quat(q).as_matrix()


def load_mount(path):
    """Require measured mount, never silently assume a zero offset."""
    config = json.loads(Path(path).read_text())
    if config["mount_measured"] is not True:
        raise ValueError("安装变换尚未测量：请先填写 T_base_imu，不能使用占位值")
    t_bi = vector(config["translation_base_imu_m"], 3, "translation_base_imu_m")
    r_bi = rotation_xyzw(config["quaternion_base_imu_xyzw"])
    for key in ("world_frame", "imu_frame", "base_frame", "measurement_note"):
        if not isinstance(config[key], str) or not config[key].strip():
            raise ValueError(f"{key} must be a nonempty string")
    if len({config["world_frame"], config["imu_frame"], config["base_frame"]}) != 3:
        raise ValueError("world, IMU and base frame names must differ")
    return config, r_bi, t_bi


def convert(p_wi, q_wi_xyzw, v_i, omega_i, r_bi, t_bi):
    """Return base world position/quaternion and base-frame linear/angular speed.

    T_WB = T_WI inv(T_BI)
    omega_B = R_BI omega_I
    v_B = R_BI v_I - omega_B x t_BI

    omega_I is the same-time camera gyro. Raw gyro bias is NOT removed here.
    """
    p_wi = vector(p_wi, 3, "position_world_imu")
    v_i = vector(v_i, 3, "linear_velocity_imu")
    omega_i = vector(omega_i, 3, "angular_velocity_imu")
    r_wi = rotation_xyzw(q_wi_xyzw)
    r_wb = r_wi @ r_bi.T
    p_wb = p_wi - r_wb @ t_bi
    omega_b = r_bi @ omega_i
    v_b = r_bi @ v_i - np.cross(omega_b, t_bi)
    return p_wb, Rotation.from_matrix(r_wb).as_quat(), v_b, omega_b


class SamplePairing:
    """Pair propagated odometry with its own input IMU sample, either arrival order.

    VINS converts integer ROS stamps through double seconds and truncates back.
    1 us covers that numerical rounding at current epoch; NOT sensor sync tolerance.
    No interpolation, extrapolation or latest-sample substitution is performed.
    """

    def __init__(self):
        self.imus = []
        self.pending = []
        self.dropped = 0

    def add_imu(self, stamp_ns, value):
        self.imus.append((stamp_ns, value))
        self.imus = self.imus[-400:]
        return self._match()

    def add_odometry(self, stamp_ns, value):
        if len(self.pending) == 40:
            self.pending.pop(0)
            self.dropped += 1
        self.pending.append((stamp_ns, value))
        return self._match()

    def _match(self):
        pairs, remaining = [], []
        for stamp, odometry in self.pending:
            if not self.imus:
                remaining.append((stamp, odometry))
                continue
            imu_stamp, imu = min(self.imus, key=lambda item: abs(item[0] - stamp))
            if abs(imu_stamp - stamp) <= 1000:
                pairs.append((odometry, imu))
            else:
                remaining.append((stamp, odometry))
        self.pending = remaining
        return pairs
