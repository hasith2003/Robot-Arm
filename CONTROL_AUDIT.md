# GP7 control audit — 2026-10-11

## Confirmed root cause

All six imported arm joints had `effort="1"`: a 1 N·m torque limit. These upstream placeholders are insufficient to support the approximate link masses and custom gripper under gravity. Increasing the proportional position gain and delaying physics did not remove this limit.

The original demo also lacked position-error tolerances. The trajectory controller could report success after the scheduled duration while actual positions were far from the target.

## Runtime reproduction and verification

Tests ran with Gazebo Sim 8.15.0, ROS Jazzy and an isolated ROS domain/Gazebo partition, without a GUI. Each run launched its own simulator and terminated its own process group afterward.

Before the correction, with gain 0.5 and the paused/timed startup:

- Initial and final `joint_l`: approximately 2.386993 rad.
- Initial and final `joint_u`: approximately -0.886313 rad.
- Maximum base angle during the run: approximately 0.000000015 rad rather than 0.25 rad.
- Demo nevertheless printed “Joint motion demo completed.”

After the correction, with gravity enabled and the same real meshes:

| Joint | Measured offset target (rad) | Measured final home (rad) |
| --- | ---: | ---: |
| joint_s | 0.250000 | 0.000000000002 |
| joint_l | -0.150000 | -0.000000000002 |
| joint_u | 0.200000 | 0.000002570037 |
| joint_b | 0.150000 | 0.000000990838 |

The demo passed its measured target check, four-second hold, return-home check, action status and controller tolerances. This verifies simulated joint motion; it does not establish CAD mass accuracy or complete the pick-and-place task.

## Corrections

- Parameterized imported torque placeholders. The simulation wrapper sets S/L/U to 300 N·m, R/B to 100 N·m and T to 50 N·m. These are simulation budgets for approximate masses, not GP7 manufacturer ratings.
- Kept the documented Gazebo position gain at 0.5 and joint holding enabled. Removed the unsupported claim that a missing gain caused the collapse.
- Replaced controller/unpause timers with robot-spawn completion followed by grouped controller activation. Physics runs while Gazebo holds unclaimed joints. Launch shuts down if spawning or activation fails.
- Added arm path/goal tolerances and independent measured-state verification to the demo. It now holds the offset pose visibly and bounds action waits.
- Preserved `tool0` through URDF-to-SDF fixed-joint conversion. Previously the detachable-joint plugin failed because that link was merged away.
- Retained explicit string typing for robot-description XML and mesh resource resolution. Preserved existing resource search paths.
- Added a `gui:=false` launch option for headless runtime checks.

## Remaining findings from the codebase review

- The provisional D-H chain and U-axis limits do not match the imported URDF. The completed task now uses the separate live-URDF model; the report's D-H derivation still needs review.
- A full executor now connects measured motion, grasp, placement and retreat through `task_state.py`. See [the validation record](PICK_PLACE_VALIDATION.md).
- The upstream detachable joint's automatic initial attachment has been replaced for task runs by `gp7_grasp`, an initially-detached project plugin. The executor manages and confirms attachment only at a validated grasp pose.
- Gripper geometry now uses custom STL meshes, mounted flush with the flange. Collision/contact behavior remains simplified; jaw movement does not prove a physical grasp.
- The world uses a provisional reachable Box A position (0.50, 0.30). The assignment coordinate and reach question remain deferred as requested.
- Link masses and diagonal inertias are approximate. Their success in this motor-control test is not evidence of accurate dynamics.
- Multiple concurrently running simulators can expose duplicate controller managers. The demo rejects multiple joint-state publishers; stop the prior launch before starting a new one.

## Relevant upstream behavior

- [Jazzy gz_ros2_control documentation](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html): position interfaces use velocity commands proportional to position error; holding is enabled by default.
- [Gazebo 8 detachable-joint documentation](https://gazebosim.org/api/sim/8/src_2systems_2detachable__joint_2DetachableJoint_8hh.html): the plugin initially attaches two models.
