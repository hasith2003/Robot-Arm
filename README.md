# GP7 Robot Arm — Gazebo Pick and Place

ROS 2 Jazzy / Gazebo Harmonic simulation of a Yaskawa GP7 with a custom STL
parallel gripper. The robot picks up a 5 cm cube (Box B), places it on the
larger blue box (Box A), releases it, and retreats. The executor checks joint
feedback, grasp alignment, object orientation and stable placement.

## Requirements

- Ubuntu 24.04 with ROS 2 Jazzy installed at `/opt/ros/jazzy`.
- Gazebo Harmonic (Gazebo Sim 8), supplied through the ROS Gazebo packages.
- A desktop session for the Gazebo window, or `gui:=false` for headless use.

After installing ROS 2 Jazzy, install the project dependencies:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions python3-numpy python3-scipy python3-pytest \
  ros-jazzy-ament-cmake ros-jazzy-ament-cmake-python \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-robot-state-publisher ros-jazzy-xacro \
  ros-jazzy-gz-sim-vendor ros-jazzy-gz-plugin-vendor \
  ros-jazzy-gz-transport-vendor ros-jazzy-gz-msgs-vendor
```

## Run on the existing project computer

Stop any previous project launch with **Ctrl+C**. Open a **new terminal** and
run this exact command:

```bash
bash ~/Downloads/Robotics/gp7_ws/run_pick_place.sh
```

The script sources ROS, builds the workspace, opens Gazebo, activates all
three controllers and automatically runs one complete pick-and-place cycle.
You do not need separate terminals to start the controllers or task.
Keep the terminal open while the simulation runs; stop it with **Ctrl+C**.

Successful completion prints:

```text
PICK AND PLACE VERIFIED: Box B=[0.5   0.3   0.225], maximum orientation error=0.000 degrees.
```

The measured orientation value can vary slightly between runs. The robot
should release the red cube onto the centre of the blue box and retreat.
To repeat the full cycle, stop the launch and run the script again.

## Adjust transport speed

```bash
GP7_SPEED_FACTOR=2 bash ~/Downloads/Robotics/gp7_ws/run_pick_place.sh
```

Default factor 1 preserves the original timing. Factor 2 halves the durations
of lift, transfer, lowering and retreat. Approach, pickup descent, finger
motion and settling/verification waits keep their original timing. Factor 1.5
is a smaller increase. Waypoint counts and existing position/speed and tracking
checks remain unchanged. Total task time is not halved. The final transport-only
speed setting has not been verified by a completed full simulation trial.

The environment variable also works with `run_pick_place_at.sh`. A new
position/speed combination needs separate validation. Stop the previous
simulation before starting another.

## Try a different pickup position

Keep the destination unchanged and place the small cube at another ground-level
position (X and Y in metres):

```bash
bash ~/Downloads/Robotics/gp7_ws/run_pick_place_at.sh 0.30 -0.30
```

Stop the previous GUI simulation with Ctrl+C first. The helper creates a separate
temporary world with cube centre Z=0.025 m; the default world is unchanged. For a
fresh clone, substitute `~/Downloads/Robot-Arm` for `~/Downloads/Robotics`.
The existing `gui:=false` argument is also supported. Finite coordinates and
initial overlap with Box A are checked, but this is not a workspace or collision
validator. The executor still checks IK, interpolated joint limits, grasp
alignment and measured placement; not every position is feasible. A successful
trial is evidence for that tested scene only, not universal reachability or
collision-free motion.

## Run from a fresh GitHub clone

```bash
mkdir -p ~/Downloads
cd ~/Downloads
git clone https://github.com/hasith2003/Robot-Arm.git
bash ~/Downloads/Robot-Arm/gp7_ws/run_pick_place.sh
```

The clone folder is `Robot-Arm`, so its path differs from the existing local
`Robotics` folder above. The launcher works from any checkout location.

For a headless run:

```bash
bash ~/Downloads/Robot-Arm/gp7_ws/run_pick_place.sh gui:=false
```

## Controllers and diagnostics

| Controller | Purpose |
| --- | --- |
| `arm_controller` | Executes trajectories for the six GP7 arm joints |
| `gripper_controller` | Moves the two sliding fingers |
| `joint_state_broadcaster` | Publishes measured joint positions and velocities |

The full-task launcher uses ROS domain **91** and Gazebo partition
`gp7_pick_place_91`. Run only one copy of this launcher at a time. For a
diagnostic terminal connected to the fresh clone:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Downloads/Robot-Arm/gp7_ws
source install/setup.bash
export ROS_DOMAIN_ID=91
export GZ_PARTITION=gp7_pick_place_91
ros2 control list_controllers
```

All three controllers should be `active`. If the robot is missing or the
task fails, read the launch terminal's first error. If dependency installation
returns HTTP 404, refresh with `sudo apt update` and retry installation.

## Project contents

| Package | Contents |
| --- | --- |
| `yaskawa_gp7_support` | Imported GP7 arm geometry and visual/collision meshes |
| `gp7_description` | Simulation URDF/Xacro, control interfaces and STL gripper |
| `gp7_simulation` | World, controller configuration and ROS 2 launch |
| `gp7_kinematics` | URDF-derived FK, Jacobian, bounded IK and Cartesian waypoints |
| `gp7_task` | Pick-and-place executor, measured motion demos and state machine |
| `gp7_grasp` | Initially detached Gazebo fixed-joint grasp plugin |

Gripper mesh dimensions and regeneration instructions are in
[the gripper README](gp7_ws/src/gp7_description/meshes/gripper/README.md).

## Validation and current limits

The complete headless cycle has passed, with measured Box B placement at
`(0.50, 0.30, 0.225)` m. See [validation evidence](PICK_PLACE_VALIDATION.md),
[the control audit](CONTROL_AUDIT.md), and [the project workflow](PROJECT_WORKFLOW.md).
The workflow refers to assignment resources supplied separately.

- The working scene uses reachable Box A centre `(0.50, 0.30, 0.10)` m. The
  assignment's requested `(1, 1, 0)` layout/reach question remains unresolved.
- Grasping uses an explicitly commanded fixed joint after measured alignment;
  it is a simulation abstraction, not a friction-based grasp model.
- Arm masses, inertias and torque budgets are simulation approximations.
- The task uses `urdf_model.py`. The standard D-H implementation in
  `kinematics.py` now matches the imported URDF, including the grasp TCP.
  See [the frame assignment and derivation](KINEMATICS_DERIVATION.md).

To build and run the five mathematical/state tests in the fresh clone:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Downloads/Robot-Arm/gp7_ws
colcon build --symlink-install
source install/setup.bash
python3 -m pytest src/gp7_task/test -q
```

Build/install/log outputs and private local folders are excluded from Git.
See [third-party attribution and licensing](THIRD_PARTY.md).
