"""Forward and damped-least-squares inverse kinematics for a provisional GP7 model.

Replace DH and transforms only after measuring the SolidWorks joint axes.
"""
from __future__ import annotations

import math
import numpy as np


class GP7Kinematics:
    JOINT_NAMES = ('joint_s', 'joint_l', 'joint_u', 'joint_r', 'joint_b', 'joint_t')
    # Standard DH: a [m], alpha [rad], d [m], theta offset [rad]. PROVISIONAL.
    DH = np.array([[0.00, math.pi / 2, 0.38, 0.0], [0.36, 0.00, 0.00, 0.0], [0.34, 0.00, 0.00, 0.0], [0.00, math.pi / 2, 0.00, 0.0], [0.00, -math.pi / 2, 0.12, 0.0], [0.00, 0.00, 0.12, 0.0]])
    LOWER = np.deg2rad([-170, -65, -70, -190, -135, -360])
    UPPER = np.deg2rad([170, 145, 190, 190, 135, 360])

    @staticmethod
    def dh_transform(a, alpha, d, theta):
        c, s, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
        return np.array([[c, -s * ca, s * sa, a * c], [s, c * ca, -c * sa, a * s], [0.0, sa, ca, d], [0.0, 0.0, 0.0, 1.0]])

    def transforms(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (6,):
            raise ValueError('Expected six joint angles in radians.')
        frames, current = [np.eye(4)], np.eye(4)
        for angle, (a, alpha, d, offset) in zip(q, self.DH):
            current = current @ self.dh_transform(a, alpha, d, angle + offset)
            frames.append(current)
        return frames

    def fk(self, q):
        return self.transforms(q)[-1]

    def jacobian(self, q):
        frames = self.transforms(q)
        tip, jacobian = frames[-1][:3, 3], np.zeros((6, 6))
        for index in range(6):
            z_axis, origin = frames[index][:3, 2], frames[index][:3, 3]
            jacobian[:3, index] = np.cross(z_axis, tip - origin)
            jacobian[3:, index] = z_axis
        return jacobian

    @staticmethod
    def rotation_error(target, current):
        rotation = target @ current.T
        angle = math.acos(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
        if angle < 1e-9:
            return np.zeros(3)
        axis = np.array([rotation[2, 1] - rotation[1, 2], rotation[0, 2] - rotation[2, 0], rotation[1, 0] - rotation[0, 1]])
        return angle * axis / (2.0 * math.sin(angle))

    def ik(self, target, seed=None, max_iterations=300, damping=0.03, position_tolerance=1e-4, orientation_tolerance=1e-3):
        target = np.asarray(target, dtype=float)
        if target.shape != (4, 4):
            raise ValueError('Target must be a 4x4 homogeneous transform.')
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
        final = self.fk(q)
        return q, False, {'iterations': max_iterations, 'position_error_m': float(np.linalg.norm(target[:3, 3] - final[:3, 3])), 'orientation_error_rad': float(np.linalg.norm(self.rotation_error(target[:3, :3], final[:3, :3])))}


if __name__ == '__main__':
    model = GP7Kinematics()
    samples = [np.deg2rad([20, -20, 45, 10, 30, -15]), np.deg2rad([-40, 35, 20, -20, -30, 70])]
    for expected in samples:
        solution, converged, residual = model.ik(model.fk(expected))
        print(f'converged={converged} residual={residual} q_deg={np.rad2deg(solution).round(2)}')
        if not converged:
            raise SystemExit(1)
