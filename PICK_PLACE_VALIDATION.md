# Full task validation

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
