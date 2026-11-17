"""Standard-DH GP7 kinematics, verified against the project URDF.

All public poses are base_link -> grasp_tcp, in metres and radians.
Frame assignment and derivation: KINEMATICS_DERIVATION.md at project root.
"""
from __future__ import annotations

import math
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares


class GP7Kinematics:
    JOINT_NAMES = ('joint_s', 'joint_l', 'joint_u', 'joint_r', 'joint_b', 'joint_t')
    # Standard DH: a_i [m], alpha_i [rad], d_i [m], theta offset [rad].
    # theta_i = q_i + offset; z_(i-1) follows each URDF positive joint axis.
    DH = np.array([
        [0.040, -math.pi / 2, 0.000, 0.000],
        [0.445, math.pi,      0.000, -math.pi / 2],
        [0.040, -math.pi / 2, 0.000, 0.000],
        [0.000, math.pi / 2, -0.440, 0.000],
        [0.000, -math.pi / 2, 0.000, 0.000],
        [0.000, 0.000,       -0.080, 0.000],
    ])
    BASE_TO_DH0 = np.eye(4)
    BASE_TO_DH0[2, 3] = 0.330
    # DH frame 6 -> tool0 is Rx(pi); TCP is 107.5 mm along tool0 +Z.
    DH6_TO_TOOL0 = np.diag([1., -1., -1., 1.])
    DH6_TO_TCP = DH6_TO_TOOL0.copy()
    DH6_TO_TCP[2, 3] = -0.1075
    LOWER = np.deg2rad([-170, -65, -70, -190, -135, -360])
    UPPER = np.deg2rad([170, 145, 190, 190, 135, 360])
    SPEED = np.deg2rad([375, 315, 410, 550, 550, 1000])

    @staticmethod
    def dh_transform(a, alpha, d, theta):
        c, s, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
        return np.array([[c, -s * ca, s * sa, a * c], [s, c * ca, -c * sa, a * s], [0.0, sa, ca, d], [0.0, 0.0, 0.0, 1.0]])

    def transforms(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise ValueError('Expected six joint angles in radians.')
        current = self.BASE_TO_DH0.copy()
        frames = [current.copy()]
        for angle, (a, alpha, d, offset) in zip(q, self.DH):
            current = current @ self.dh_transform(a, alpha, d, angle + offset)
            frames.append(current)
        return frames

    def fk(self, q):
        return self.transforms(q)[-1] @ self.DH6_TO_TCP

    def jacobian(self, q):
        frames = self.transforms(q)
        tip = (frames[-1] @ self.DH6_TO_TCP)[:3, 3]
        jacobian = np.zeros((6, 6))
        for index in range(6):
            z_axis, origin = frames[index][:3, 2], frames[index][:3, 3]
            jacobian[:3, index] = np.cross(z_axis, tip - origin)
            jacobian[3:, index] = z_axis
        return jacobian

    @staticmethod
    def rotation_error(target, current):
        # SO(3) logarithm remains valid at pi, where division by sin(angle) fails.
        return Rotation.from_matrix(target @ current.T).as_rotvec()

    def ik(self, target, seed=None, max_iterations=300, damping=0.03, position_tolerance=1e-4, orientation_tolerance=1e-3):
        target = np.asarray(target, dtype=float)
        if target.shape != (4, 4) or not np.all(np.isfinite(target)):
            raise ValueError('Target must be a finite 4x4 homogeneous transform.')
        if (not np.allclose(target[3], [0, 0, 0, 1], atol=1e-8)
                or not np.allclose(target[:3, :3].T @ target[:3, :3], np.eye(3), atol=1e-8)
                or not np.isclose(np.linalg.det(target[:3, :3]), 1., atol=1e-8)):
            raise ValueError('Target must contain a proper rotation and homogeneous last row.')
        if max_iterations < 1 or damping <= 0 or min(position_tolerance, orientation_tolerance) <= 0:
            raise ValueError('Iteration count, damping and tolerances must be positive.')
        if seed is not None:
            seed = np.asarray(seed, dtype=float)
            if seed.shape != (6,) or not np.all(np.isfinite(seed)):
                raise ValueError('Seed must contain six finite joint angles.')
        q = np.zeros(6) if seed is None else np.clip(np.asarray(seed, dtype=float), self.LOWER, self.UPPER)
        weights = np.diag([1.0, 1.0, 1.0, 0.25, 0.25, 0.25])
        for iteration in range(max_iterations):
            current = self.fk(q)
            position_error = target[:3, 3] - current[:3, 3]
            orientation_error = self.rotation_error(target[:3, :3], current[:3, :3])
            if np.linalg.norm(position_error) < position_tolerance and np.linalg.norm(orientation_error) < orientation_tolerance:
                return q, True, {'iterations': iteration, 'position_error_m': float(np.linalg.norm(position_error)), 'orientation_error_rad': float(np.linalg.norm(orientation_error))}
            jacobian = weights @ self.jacobian(q)
            error = weights @ np.concatenate((position_error, orientation_error))
            delta = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping ** 2 * np.eye(6), error)
            q = np.clip(q + np.clip(delta, -0.15, 0.15), self.LOWER, self.UPPER)
        # Projected DLS can stall at a joint limit. A bounded trust-region
        # solve can recover without stepping outside the URDF limits.
        def residual(candidate):
            pose = self.fk(candidate)
            return weights @ np.r_[target[:3, 3] - pose[:3, 3],
                                    self.rotation_error(target[:3, :3], pose[:3, :3])]

        result = least_squares(residual, q, bounds=(self.LOWER, self.UPPER),
                               max_nfev=max_iterations, ftol=1e-12, xtol=1e-12, gtol=1e-12)
        q = result.x
        final = self.fk(q)
        position_error = float(np.linalg.norm(target[:3, 3] - final[:3, 3]))
        orientation_error = float(np.linalg.norm(self.rotation_error(target[:3, :3], final[:3, :3])))
        converged = position_error < position_tolerance and orientation_error < orientation_tolerance
        return q, converged, {'iterations': max_iterations,
                              'fallback_evaluations': result.nfev,
                              'position_error_m': position_error,
                              'orientation_error_rad': orientation_error}



if __name__ == '__main__':
    model = GP7Kinematics()
    samples = [np.deg2rad([20, -20, 45, 10, 30, -15]), np.deg2rad([-40, 35, 20, -20, -30, 70])]
    for expected in samples:
        solution, converged, residual = model.ik(model.fk(expected))
        print(f'converged={converged} residual={residual} q_deg={np.rad2deg(solution).round(2)}')
        if not converged:
            raise SystemExit(1)
