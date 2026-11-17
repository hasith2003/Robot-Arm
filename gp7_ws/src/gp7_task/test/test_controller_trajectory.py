import numpy as np
import pytest
from gp7_task.controller_trajectory import smooth_joint_trajectory


def test_interpolated_speed_violation_is_rejected():
    # Average speed is 1, but a rest-to-rest cubic peaks at 1.5.
    with pytest.raises(RuntimeError, match='speed limit'):
        smooth_joint_trajectory([(0, [0]), (1, [1])], [-2], [2], [1.2])


def test_interior_position_overshoot_is_rejected():
    with pytest.raises(RuntimeError, match='position limits'):
        smooth_joint_trajectory([(0, [0]), (1, [1]), (2, [1]), (3, [0])],
                                [-1], [1], [10])


def test_commands_have_matching_position_velocity_and_acceleration():
    from scipy.interpolate import BPoly, CubicSpline
    points = [(0, [0]), (1, [.2]), (2, [.7]), (3, [1])]
    t, q, v, a = smooth_joint_trajectory(points, [-2], [2], [2])
    np.testing.assert_allclose(v[[0, -1]], 0, atol=1e-14)
    controller = BPoly.from_derivatives(t, [[q[i, 0], v[i, 0], a[i, 0]] for i in range(len(t))])
    reference = CubicSpline(t, q[:, 0], bc_type='clamped')
    for derivative in (0, 1, 2):
        samples = np.linspace(0, 3, 301)
        np.testing.assert_allclose(controller(samples, nu=derivative),
                                   reference(samples, nu=derivative), atol=1e-12)
