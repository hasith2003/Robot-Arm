"""Smooth fixed-orientation Cartesian paths and waypoint IK conversion."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np


class TrajectoryPlanningError(RuntimeError):
    """Raised when an interpolated Cartesian waypoint has no valid IK solution."""


@dataclass(frozen=True)
class CartesianSample:
    time_s: float
    position_m: np.ndarray
    velocity_mps: np.ndarray
    acceleration_mps2: np.ndarray


class CartesianSegment:
    """Quintic position interpolation with a constant object/tool orientation.

    The quintic law has zero velocity and acceleration at both ends. During a
    carried-object segment, set `rotation` to the initial object orientation so
    the path preserves the required upright attitude.
    """

    def __init__(self, start_m, end_m, rotation, duration_s):
        self.start_m = np.asarray(start_m, dtype=float)
        self.end_m = np.asarray(end_m, dtype=float)
        self.rotation = np.asarray(rotation, dtype=float)
        self.duration_s = float(duration_s)
        if self.start_m.shape != (3,) or self.end_m.shape != (3,):
            raise ValueError('Start and end positions must be 3-vectors.')
        if self.rotation.shape != (3, 3):
            raise ValueError('Rotation must be a 3x3 matrix.')
        if self.duration_s <= 0:
            raise ValueError('Duration must be positive.')

    def sample(self, time_s):
        time_s = min(max(float(time_s), 0.0), self.duration_s)
        tau = time_s / self.duration_s
        blend = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
        blend_dot = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / self.duration_s
        blend_ddot = (60 * tau - 180 * tau**2 + 120 * tau**3) / self.duration_s**2
        displacement = self.end_m - self.start_m
        return CartesianSample(time_s, self.start_m + blend * displacement, blend_dot * displacement, blend_ddot * displacement)

    def transform(self, time_s):
        transform = np.eye(4)
        transform[:3, :3] = self.rotation
        transform[:3, 3] = self.sample(time_s).position_m
        return transform

    def ik_waypoints(self, model, samples=50, seed=None):
        """Return continuous joint solutions, using the previous solution as seed."""
        if samples < 2:
            raise ValueError('At least two samples are required.')
        q_seed = np.zeros(6) if seed is None else np.asarray(seed, dtype=float)
        points = []
        for time_s in np.linspace(0.0, self.duration_s, samples):
            q, converged, residual = model.ik(self.transform(time_s), seed=q_seed)
            if not converged:
                raise TrajectoryPlanningError(f'IK failed at t={time_s:.3f}s: {residual}')
            points.append((float(time_s), q, residual))
            q_seed = q
        return points
