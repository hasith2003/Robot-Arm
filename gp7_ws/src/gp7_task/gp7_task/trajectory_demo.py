"""A deterministic trajectory smoke test using FK-generated endpoint poses."""
import numpy as np
from gp7_kinematics.kinematics import GP7Kinematics
from gp7_task.trajectory import CartesianSegment


def main():
    model = GP7Kinematics()
    initial_q = np.deg2rad([20, 10, 20, 0, 30, 0])
    initial = model.fk(initial_q)
    # Keeps initial orientation; the 3 cm lift lies within this provisional
    # model's tested orientation-constrained workspace.
    path = CartesianSegment(initial[:3, 3], initial[:3, 3] + np.array([0.0, 0.0, 0.03]), initial[:3, :3], 2.0)
    waypoints = path.ik_waypoints(model, samples=21, seed=initial_q)
    print(f'Generated {len(waypoints)} fixed-orientation IK waypoints.')
    print('Maximum joint displacement (deg):', np.rad2deg(np.max(np.abs(np.diff([q for _, q, _ in waypoints], axis=0)), axis=0)).round(2))


if __name__ == '__main__':
    main()
