"""Offline checks: geometry, lever-arm sign, timestamps; never initialize ROS."""

import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from body_frame import SamplePairing, convert, load_mount


class BodyFrameTest(unittest.TestCase):
    def test_identity(self):
        p, q, v, w = convert([1, 2, 3], [0, 0, 0, 1], [4, 5, 6],
                             [0, 0, 0.2], np.eye(3), np.zeros(3))
        np.testing.assert_allclose(p, [1, 2, 3])
        np.testing.assert_allclose(q, [0, 0, 0, 1])
        np.testing.assert_allclose(v, [4, 5, 6])
        np.testing.assert_allclose(w, [0, 0, 0.2])

    def test_world_yaw_and_offset(self):
        q_wi = Rotation.from_euler("z", 90, degrees=True).as_quat()
        p, _, v, _ = convert([1, 2.3, 0.4], q_wi, [0.5, 0, 0],
                             [0, 0, 0], np.eye(3), [0.3, 0, 0.4])
        np.testing.assert_allclose(p, [1, 2, 0], atol=1e-12)
        # Linear input is already body-frame: world yaw must NOT rotate it again.
        np.testing.assert_allclose(v, [0.5, 0, 0], atol=1e-12)

    def test_optical_axes(self):
        # Camera right/down/forward -> Go2 forward/left/up; a synthetic alignment.
        r_bi = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        q_wi = Rotation.from_matrix(r_bi).as_quat()
        p, q, v, w = convert([0.3, 0, 0.1], q_wi, [0, 0, 0.5],
                             [0, -0.2, 0], r_bi, [0.3, 0, 0.1])
        np.testing.assert_allclose(p, [0, 0, 0], atol=1e-12)
        np.testing.assert_allclose(Rotation.from_quat(q).as_matrix(), np.eye(3), atol=1e-12)
        np.testing.assert_allclose(w, [0, 0, 0.2], atol=1e-12)
        np.testing.assert_allclose(v, [0.5, -0.06, 0], atol=1e-12)

    def test_in_place_rotation_stationary_base(self):
        # IMU 0.30m ahead: left turn 1rad/s creates +0.30m/s at IMU, base stays put.
        _, _, v, w = convert([0.3, 0, 0], [0, 0, 0, 1], [0, 0.3, 0],
                             [0, 0, 1], np.eye(3), [0.3, 0, 0])
        np.testing.assert_allclose(v, [0, 0, 0], atol=1e-12)
        np.testing.assert_allclose(w, [0, 0, 1])

    def test_nonplanar_mount(self):
        # A pitched/rolled camera must not force the robot base attitude to zero.
        r_bi = Rotation.from_euler("xyz", [15, -20, 30], degrees=True).as_matrix()
        r_wb = Rotation.from_euler("xyz", [3, 5, 70], degrees=True).as_matrix()
        t_bi = np.array([0.3, 0.02, 0.1])
        p_wb = np.array([2, -1, 0.3])
        omega_b = np.array([0.1, -0.2, 0.4])
        # Independently construct a camera on a base moving forward at 0.4m/s.
        q_wi = Rotation.from_matrix(r_wb @ r_bi).as_quat()
        p_wi = p_wb + r_wb @ t_bi
        v_i = r_bi.T @ (np.array([0.4, 0, 0]) + np.cross(omega_b, t_bi))
        p, q, v, w = convert(p_wi, q_wi, v_i, r_bi.T @ omega_b, r_bi, t_bi)
        np.testing.assert_allclose(p, p_wb, atol=1e-12)
        np.testing.assert_allclose(Rotation.from_quat(q).as_matrix(), r_wb, atol=1e-12)
        np.testing.assert_allclose(v, [0.4, 0, 0], atol=1e-12)
        np.testing.assert_allclose(w, omega_b, atol=1e-12)

    def test_pairing_arrival_and_rounding(self):
        stamp = 1794129468_123456789
        pairing = SamplePairing()
        self.assertEqual(pairing.add_odometry(stamp - 200, "odom1"), [])
        self.assertEqual(pairing.add_imu(stamp, "imu1"), [("odom1", "imu1")])
        self.assertEqual(pairing.add_imu(stamp + 5_000_000, "imu2"), [])
        self.assertEqual(pairing.add_odometry(stamp + 5_000_100, "odom2"),
                         [("odom2", "imu2")])
        self.assertEqual(pairing.add_odometry(stamp + 2_000_000, "unmatched"), [])
        self.assertEqual(len(pairing.pending), 1)

    def test_missing_mount_and_invalid_input(self):
        path = Path(__file__).parent / "config/go2_mount.unmeasured.json"
        with self.assertRaisesRegex(ValueError, "尚未测量"):
            load_mount(path)
        config = json.loads(path.read_text())
        config.update(mount_measured=True, translation_base_imu_m=[0.3, 0, 0.1],
                      quaternion_base_imu_xyzw=[0, 0, 0, 1])
        with tempfile.TemporaryDirectory() as directory:
            measured = Path(directory) / "mount.json"
            measured.write_text(json.dumps(config))
            _, rotation, offset = load_mount(measured)
            np.testing.assert_allclose(rotation, np.eye(3))
            np.testing.assert_allclose(offset, [0.3, 0, 0.1])
        with self.assertRaises(ValueError):
            convert([0, 0, 0], [0, 0, 0, 0], [0, 0, 0],
                    [0, 0, 0], np.eye(3), [0, 0, 0])
        with self.assertRaises(ValueError):
            convert([0, 0, 0], [0, 0, 0, 1], [float("nan"), 0, 0],
                    [0, 0, 0], np.eye(3), [0, 0, 0])


if __name__ == "__main__":
    unittest.main()
