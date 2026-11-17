"""C2-continuous joint commands, checked between waypoints as well as at them."""
import numpy as np
from scipy.interpolate import CubicSpline


def smooth_joint_trajectory(points, lower, upper, speed):
    times = np.asarray([t for t, _ in points], dtype=float)
    positions = np.asarray([q for _, q in points], dtype=float)
    if (len(times) < 2 or not np.all(np.isfinite(times))
            or not np.all(np.isfinite(positions)) or times[0] != 0
            or np.any(np.diff(times) <= 0)):
        raise ValueError('Trajectory needs finite waypoints and increasing times starting at zero')
    lower, upper, speed = map(np.asarray, (lower, upper, speed))
    if positions.shape != (len(times), len(lower)):
        raise ValueError('Trajectory joint dimensions do not match limits')
    spline = CubicSpline(times, positions, bc_type='clamped')
    # A cubic reaches a position extremum where its first derivative is zero;
    # its speed extrema occur at second-derivative roots or interval endpoints.
    for joint in range(len(lower)):
        curve = CubicSpline(times, positions[:, joint], bc_type='clamped')
        for derivative, bound in ((0, None), (1, speed[joint])):
            roots = curve.derivative(derivative + 1).roots(extrapolate=False)
            roots = roots[np.isfinite(roots)]
            values = curve(np.r_[times, roots], nu=derivative)
            if derivative == 0:
                if np.min(values) < lower[joint] - 1e-10 or np.max(values) > upper[joint] + 1e-10:
                    raise RuntimeError(f'Interpolated joint {joint} exceeds its position limits')
            elif np.max(np.abs(values)) > bound + 1e-10:
                raise RuntimeError(f'Interpolated joint {joint} exceeds its speed limit')
    # Supplying q, qdot AND qddot makes the controller's quintic interpolation
    # reproduce each cubic interval exactly (and preserve continuous qddot).
    return times, positions, spline(times, 1), spline(times, 2)
