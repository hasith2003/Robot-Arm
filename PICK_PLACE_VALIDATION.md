# Full task validation

## Current audit — 17 November 2026

See `TASK_REQUIREMENTS_AUDIT.md` for the current requirement-by-requirement
result and `analysis/audit_video/result.json` for final recorded-run metrics.
All 16 tests pass in both workspace copies. Fresh headless and GUI runs
complete through DONE with the accepted box positions unchanged. Commands now
include continuous spline velocities and accelerations, interpolation extrema
are checked against joint limits, and object orientation is monitored from
the initial pose before approach. Measured CSVs, plots and an unobstructed
Gazebo render recording are saved under `analysis/audit_video`.

The D-H implementation has been corrected and independently checked against
the URDF; later references below to a provisional D-H model are historical.
The final report/member indices remain submission work, and the fixed-joint
grasp and approximate dynamics retain their stated scope.

## Earlier validation records

The headless ROS Jazzy / Gazebo Harmonic run completed every state:
PREGRASP → DESCEND → CLOSE → VERIFY_GRASP → LIFT → TRANSFER → LOWER →
RELEASE → VERIFY_PLACE → RETREAT → DONE.

Measured final Box B centre: **(0.500, 0.300, 0.225) m**.
Maximum measured object orientation change during grasp, transport, placement
and retreat rounded to **0.000 degrees**. The stable placement check also
compared two successive measured poses, 0.5 seconds apart.

The task uses the imported URDF's actual joint origins, axes and limits,
not the old provisional D-H table. FK's zero TCP position is
(0.6675, 0, 0.815) m; the analytical Jacobian matches central finite
differences for all six joints. Downward descent IK meets endpoint and
orientation tolerances within the URDF joint limits. All five unit tests pass.

All Cartesian transport segments are solved before starting the approach.
Dense quintic position waypoints provide slow starts/stops. The numerical
planner checks joint speed limits. Controllers enforce tracking/goal
tolerances, and the executor independently checks measured joint targets,
jaw positions, TCP alignment, attachment acknowledgement, object lift,
upright orientation and stable placement.

The simulator plugin starts detached and creates/removes a physical fixed
joint only on explicit task commands. No object-pose teleport or scripted
box animation is used. Grasp force/friction is abstracted by that constraint;
the closing jaws leave 0.5 mm clearance on each cube face to avoid fighting
the simplified contact solver.

Scope: this validates the current reachable scene. Box A remains at the
provisional XY position (0.50, 0.30); the assignment's requested (1, 1)
coordinate and reach discussion remain deferred. Link masses, inertia and
torque budgets are still simulation approximations.

Run: `bash ~/Downloads/Robotics/gp7_ws/run_pick_place.sh`.


## Revalidation — 2026-11-17

Rebuilt all six packages and reproduced a successful headless cycle, then
completed a GUI cycle with the corrected executor. The corrected run reached
DONE with Box B at (0.500, 0.300, 0.225) m and maximum measured orientation
change rounding to 0.000 degrees. The final position is now measured after
retreat, including two poses 0.5 seconds apart to check placement stability.

Corrections applied to both workspace copies:
- Reject stale object-pose feedback while monitoring transport and placement.
- Recheck placement position and stability after the arm retreats.
- Stop the launch when the pick-and-place process fails.
- Launch ROS/Gazebo in an owned process group and clean up its children on
  exit. This fixes the orphaned Gazebo process/controller conflict reproduced
  when restarting the original launcher.
- Use a shared per-domain lock to reject concurrent launcher instances.

Validation: five existing unit tests pass; launch argument loading and shell
syntax checks pass. An isolated launcher check confirmed duplicate rejection
and termination of the child process group. The GUI run confirms the new
post-retreat check passes. The reachable scene, fixed-joint grasp abstraction,
provisional D-H model, and approximate dynamics retain their stated scope.
