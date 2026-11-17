"""Independent comparisons of the hand-derived DH chain with expanded URDF."""
import pathlib
import subprocess
import unittest

import numpy as np
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation

from gp7_kinematics.kinematics import GP7Kinematics
from gp7_kinematics.urdf_model import URDFModel


class DHKinematicsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = pathlib.Path(get_package_share_directory('gp7_description')) / 'urdf/gp7.urdf.xacro'
        xml = subprocess.check_output(['xacro', str(path)], text=True)
        cls.urdf = URDFModel(xml)
        cls.tool = URDFModel(xml, tip='tool0')
        cls.model = GP7Kinematics()

    def test_fk_matches_urdf_at_random_poses_and_joint_limits(self):
        rng = np.random.default_rng(7)
        poses = np.vstack([np.zeros(6), self.urdf.lower, self.urdf.upper,
                           rng.uniform(self.urdf.lower, self.urdf.upper, (200, 6))])
        for q in poses:
            np.testing.assert_allclose(self.model.fk(q), self.urdf.fk(q), atol=2e-14)
            tool = self.model.transforms(q)[-1] @ self.model.DH6_TO_TOOL0
            np.testing.assert_allclose(tool, self.tool.fk(q), atol=2e-14)

    def test_joint_limits_and_speed_match_urdf(self):
        np.testing.assert_allclose(self.model.LOWER, self.urdf.lower)
        np.testing.assert_allclose(self.model.UPPER, self.urdf.upper)
        np.testing.assert_allclose(self.model.SPEED, self.urdf.speed)
        # Independent standard-GP7 specification, manufacturer Table 5-2.
        # Catch the prior imported U-axis -116/+255 range even if both the
        # D-H constants and URDF were accidentally changed together.
        np.testing.assert_allclose(self.urdf.lower,
            np.deg2rad([-170, -65, -70, -190, -135, -360]))
        np.testing.assert_allclose(self.urdf.upper,
            np.deg2rad([170, 145, 190, 190, 135, 360]))

    def test_full_tcp_jacobian_matches_urdf_finite_differences(self):
        rng = np.random.default_rng(8)
        for q in rng.uniform(self.urdf.lower, self.urdf.upper, (20, 6)):
            jacobian = self.model.jacobian(q)
            for i in range(6):
                dq = np.zeros(6)
                dq[i] = 1e-6
                plus, minus = self.urdf.fk(q + dq), self.urdf.fk(q - dq)
                numerical = np.r_[(plus[:3, 3] - minus[:3, 3]) / 2e-6,
                    Rotation.from_matrix(plus[:3, :3] @ minus[:3, :3].T).as_rotvec() / 2e-6]
                np.testing.assert_allclose(jacobian[:, i], numerical, atol=2e-8)

    def test_ik_reaches_urdf_generated_targets(self):
        rng = np.random.default_rng(9)
        for q in rng.uniform(self.urdf.lower + .1, self.urdf.upper - .1, (30, 6)):
            target = self.urdf.fk(q)
            solution, ok, info = self.model.ik(target, seed=q + rng.normal(0, .08, 6))
            self.assertTrue(ok, info)
            self.assertTrue(np.all(solution >= self.urdf.lower) and np.all(solution <= self.urdf.upper))
            actual = self.urdf.fk(solution)
            self.assertLess(np.linalg.norm(actual[:3, 3] - target[:3, 3]), 1e-4)
            angle = Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).magnitude()
            self.assertLess(angle, 1e-3)

    def test_pick_ik_from_home_and_descent(self):
        target = np.eye(4)
        target[:3, :3] = np.diag([1., -1., -1.])
        target[:3, 3] = [.35, -.25, .225]
        seed = np.zeros(6)
        for z in np.linspace(.225, .025, 21):
            target[2, 3] = z
            seed, ok, info = self.model.ik(target, seed=seed)
            self.assertTrue(ok, info)
            actual = self.urdf.fk(seed)
            self.assertLess(np.linalg.norm(actual[:3, 3] - target[:3, 3]), 1e-4)
            self.assertLess(Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).magnitude(), 1e-3)

    def test_pi_rotation_is_not_false_success(self):
        for axis in np.eye(3):
            rotation = Rotation.from_rotvec(np.pi * axis).as_matrix()
            self.assertAlmostEqual(np.linalg.norm(self.model.rotation_error(rotation, np.eye(3))), np.pi)
        target = self.urdf.fk(np.zeros(6))
        target[:3, :3] = Rotation.from_rotvec([np.pi, 0, 0]).as_matrix() @ target[:3, :3]
        solution, ok, info = self.model.ik(target)
        if ok:
            actual = self.urdf.fk(solution)
            self.assertLess(Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).magnitude(), 1e-3)
        self.assertGreater(info['iterations'], 0)

    def test_unreachable_pose_is_rejected(self):
        target = np.eye(4)
        target[:3, 3] = [10, 10, 10]
        _, ok, _ = self.model.ik(target, max_iterations=30)
        self.assertFalse(ok)
