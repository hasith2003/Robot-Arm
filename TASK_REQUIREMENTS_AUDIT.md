# GP7 task-sheet audit — 17 November 2026

The accepted reachable scene completed fresh headless and GUI runs, including
pickup, transport, release, settlement and retreat. The scene positions were
not changed. This validates the tested simulation with the tolerances below;
it is not a claim of exact physical accuracy or success for every possible pose.

## Requirement checks

| Task-sheet requirement | Finding and evidence |
|---|---|
| GP7 imported into Gazebo; at least 4 DoF | PASS: imported six-revolute-joint GP7, with a two-jaw simplified gripper. All six packages built successfully. |
| Robot at world origin | PASS: spawn coordinates and world-to-base transform are zero; executor independently checks measured model position within 1 mm. |
| Box A dimensions 0.30 × 0.30 × 0.20 m | PASS: SDF visual and collision dimensions agree. Centre Z = 0.10 m puts its bottom on the ground. |
| Box A at task-sheet XY (1, 1) m | USER-ACCEPTED DEVIATION: preserve XY (0.50, 0.30) m, as explicitly requested. The robot cannot reach the specified original destination from its fixed base. Do not claim literal compliance with this coordinate in the submission. |
| Box B dimensions 0.05 × 0.05 × 0.05 m; reachable ground location | PASS for the selected initial point (0.35, -0.25, 0.025) m. It is a fixed chosen reachable location, not a new random sample on every launch. |
| Pick up B and place at centre of A's top | PASS: measured final centre (0.500000000004, 0.299999999976, 0.224999133691) m. Target centre (0.50, 0.30, 0.225) m accounts for both boxes' half-heights. |
| Box B orientation unchanged throughout motion | PASS within the 0.035 rad acceptance threshold. Monitoring now begins at the initial measured box pose, before approach, descent and jaw closing. Recorded maximum angular change was about 1.16e-8 degrees. |
| Smooth velocity profile | PASS: quintic Cartesian position timing; continuous cubic joint interpolation with position, velocity and acceleration supplied to the trajectory controller. Every segment starts and ends at zero commanded joint velocity. |
| Joint ranges and maximum speeds | PASS: command extrema checked between waypoints against URDF limits; measured positions and speeds independently checked during execution. Full Cartesian interpolation checked for TCP position and orientation between IK knots. |
| Visualize and label coordinate frames | Exact FK-generated figure supplied in `validation/latest/dh_coordinate_frames.png` and PDF. Frame definitions and directions are in `KINEMATICS_DERIVATION.md`. The earlier generated photo annotations are schematic and should not be used to measure frame locations. |
| D-H table and forward matrices | Implemented and documented in `KINEMATICS_DERIVATION.md`; tests compare the D-H model against the independently expanded URDF chain. Include fixed base and TCP transforms. |
| Inverse kinematics equations | Bounded numerical IK implemented; residual definitions and DLS derivation documented. Runtime task uses bounded least squares on the live URDF. IK alone does not prove collision avoidance. |
| IK, trajectory and velocity-control snippets in report | Source locations below supply these; final submission must include the snippets and explain the actual control architecture. |
| Explanation of implementation and results | This audit, the derivation and recorded numerical evidence provide the content. |
| Member index numbers and concise final report | STILL REQUIRED: no final submission report with member indices was located. Source documentation alone is not the finished report. |
| Screen-recorded simulation video | Actual GUI render recording supplied: `validation/latest/gp7_verified_demo_720p.mp4`. It shows the arm approach, pickup, transfer, placement and retreat. Screen layout/camera changes are visible; no object animation or synthetic motion was substituted. |
| Optional GitHub repository | Existing `Robot-Arm` Git checkout found. Publishing/submitting it and adding a report link remain submission steps. |

## Confirmed corrections

1. Object-orientation monitoring originally began only at attachment. It now
   covers the initial approach, descent and gripper closing as well.
2. Position-only intermediate trajectory points originally produced linear
   interpolation with changes in commanded velocity at knots. Commands now
   include consistent position, velocity and acceleration values, so the
   controller reproduces a continuous cubic spline within each segment.
3. Checking only average waypoint speed could miss interpolation overshoot.
   Position and speed extrema are now checked analytically between knots.
4. Runtime evidence is saved when `GP7_EVIDENCE_DIR` is set, including failed
   runs: measured joint states, measured object poses, commanded trajectories
   and a JSON outcome. Success still requires actual controller feedback and
   measured stable placement after retreat.

These changes were mirrored into `Robot-Arm/gp7_ws`. No box pose, arm length,
joint limit, collision geometry, tolerance or grasp constraint was relaxed to
obtain success.

## Final recorded-run measurements

- Every state reached: PREGRASP, DESCEND, CLOSE, VERIFY_GRASP, LIFT, TRANSFER,
  LOWER, RELEASE, VERIFY_PLACE, RETREAT, DONE.
- 5,071 measured joint-state samples and 3,062 object-pose samples.
- Final position error: 8.6631e-7 m, approximately **0.00087 mm**.
- Post-retreat position drift over the checked 0.5-second interval: 0 m.
- Maximum measured joint speed: about 0.336 rad/s (19.2 degrees/s).
- All measured speeds below their respective URDF maxima; smallest measured
  joint-position margin about 1.030 rad (59.0 degrees).
- Numerical measurements are specific to this idealized simulator; their small
  errors do not imply comparable accuracy on a real robot.

The controller uses position command interfaces with Gazebo's built-in
position-error-to-velocity tracking, configured with gain 0.5 at a 100 Hz ROS
controller update rate. The project supplies timed joint position/velocity/
acceleration trajectories. It does **not** implement a custom velocity PID.
The physics step is 1 ms; the 10 ms controller period is intentional.

## Grasp and dynamics scope

The jaws close around the cube with 0.5 mm clearance per face. After measured
TCP alignment and gripper feedback pass, an explicit Gazebo detachable fixed
joint attaches the cube. The joint is removed before release and settlement
is verified under gravity/contact. This is an idealized grasp, not a validated
friction/contact grasp. Link masses, inertias and motor efforts are simulation
approximations. Do not claim hardware-level grasp or dynamics validation.

## Reproduce and inspect

```bash
source /opt/ros/jazzy/setup.bash
source gp7_ws/install/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider gp7_ws/src/gp7_task/test
GP7_EVIDENCE_DIR="$PWD/analysis/new_run" bash gp7_ws/run_pick_place.sh
```

Stop the previous simulation before starting another. The shared launcher lock
rejects concurrent instances in the same ROS domain. GUI closure alone may
leave ROS launch alive; stop the launcher terminal with Ctrl+C as well.

Validation: **16 tests pass in both workspace copies**. They cover the
independent URDF/D-H FK and Jacobian, numerical IK, 180-degree rotation error,
failure of an unreachable target, all carried-object segments including
interpolation between knots, spline speed and position overshoot, controller
interpolation equivalence, and state-machine transition guards.

Evidence directories `../analysis/audit_headless`, `../analysis/audit_gui` and
`../analysis/audit_video` contain `result.json` plus CSV measurements. Final plots:

- `validation/latest/measured_results.png` / `.pdf`
- `validation/latest/commanded_velocities.png` / `.pdf`
- `validation/latest/final.png`
- `validation/latest/simulation.log`

Relevant source for the final report:

- IK: `gp7_ws/src/gp7_kinematics/gp7_kinematics/urdf_model.py`, `ik`.
- Cartesian quintic planning: same file, `cartesian`.
- Continuous joint commands and limits: `gp7_ws/src/gp7_task/gp7_task/controller_trajectory.py`.
- Controller messages, state execution and measured checks: `gp7_ws/src/gp7_task/gp7_task/pick_place.py`.
- Actual control configuration: `gp7_ws/src/gp7_simulation/config/gp7_controllers.yaml`
  and `gp7_ws/src/gp7_description/urdf/gp7.urdf.xacro`.

The literal destination deviation and final report/member details remain
submission matters. The tested reachable simulation itself completes the task.
