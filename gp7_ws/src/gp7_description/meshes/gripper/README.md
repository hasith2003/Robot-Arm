# Custom project parallel gripper

Original simulation geometry created for this project, in metres. The task
sheet permits a simplified gripper and states that the supplied GP7 CAD has
no end effector. These meshes are not a commercial gripper's CAD model.

- `flange_mount.stl`: round mounting plate, radius 30 mm, thickness 8 mm.
- `body.stl`: chamfered housing, 70 x 100 x 57 mm, axial extent Z=8..65 mm.
- `jaw.stl`: one chamfered finger, 36 x 15 x 65 mm; used twice as separate links.
- `generate_meshes.py`: reproducible mesh source, using Python's standard library.

The flange mount starts at tool0 Z=0. The tool link attaches directly to the
robot flange with zero translation. The jaws slide along opposite Y axes,
starting at Y=+/-47.5 mm, Z=65 mm. Their clear gap is
`0.080 - q_left - q_right` metres. Commanding 0.015 m on each joint produces
a 50 mm gap. The jaw stroke is 25 mm per side. `grasp_tcp` is centred between
the jaws at tool0 Z=107.5 mm. The closing command tests gripper width and does
not establish a grasp without approach, contact and object-state verification.

Visual meshes are closed triangular surfaces. Collision geometry uses simple
boxes and a cylinder for predictable simulation. Masses/inertias remain
engineering approximations. No manufacturer torque or gripping-force claim
is implied.

Arm-only demos disable attachment. Full pick/place runs enable the custom
`gp7_grasp` plugin, which starts detached and responds to explicit attach/detach
commands. The task validates the TCP and cube alignment before attachment,
then monitors actual box orientation and confirms placement. The grasp uses
a fixed-joint abstraction with 0.5 mm clearance on each jaw face; it does not
simulate friction-based gripping force.
