# EN3563 Robotics mini-project — complete execution workflow

## 1. Recommended approach and scope

Build the **six-axis Yaskawa GP7 simulation in Gazebo**, starting with simple geometry, then adding the supplied CAD meshes. Implement and explain your own forward kinematics, inverse kinematics, Cartesian trajectory generation, and velocity control. Use ROS 2 to connect the components. MoveIt is optional for collision checking or comparison; it should not replace the mathematics required in your report.

This is a project plan based on the supplied files, not a completed robot implementation. Simulation is the recommended assumption because the folder contains GP7 CAD and specifications. The physical alternative is outlined at the end. No deadline was provided; the schedule below is an estimate.

## 2. What the supplied files tell us

| File | What it supplies | How to use it |
|---|---|---|
| `Task_sheet.pdf`, pp. 1–2 | Assessment requirements, environment, deliverables | Treat as the acceptance checklist |
| `Resources-20261011/G7_manual.pdf` | GP7/GP8 mechanical manual, joint limits, dimensions, wrist load information | Use GP7-specific tables and figures; especially Table 5-2, §5.4 Fig. 5-3(b), §6 |
| `Resources-20261011/GP7_GP8_datasheet.pdf`, pp. 1–2 | Summary specifications for both robots | Use the GP7 column; **the dimension drawing is labelled GP8** |
| `Resources-20261011/GP7-(Solidworks-2020).zip` | SolidWorks assemblies and parts | Measure joint geometry and export meshes |

The ZIP inventory contains 38 entries: two directories and 36 CAD files. It includes `GP7 Robot.SLDASM` both at the top level and in the components directory, plus `GP7_S_AXIS`, `GP7_L_AXIS`, `GP7_U_AXIS`, `GP7_R_AXIS`, `GP7_B_AXIS`, `GP7_T_AXIS`, base assemblies, and individual parts. It also includes `GP7_envelope.SLDPRT`, which is a workspace reference and should not become a physical collision link.

Archive contents were inspected, but the native SolidWorks assemblies were not opened or their mates validated. Do not assume CAD assembly names alone establish the correct robot frames.

The supplied manual lists approximately **37 kg** for the GP7, while the older datasheet lists **34 kg**. Record this document discrepancy. Neither total mass is a substitute for individual link masses and inertias.

## 3. Resolve the assignment's geometry issue first

The task specifies a robot at `(0,0,0)` and Box A at `(1,1,0)` metres. Its horizontal distance from the base is:

`r = sqrt(1² + 1²) = 1.414 m`.

The GP7's specified maximum horizontal reach is **0.927 m**. The destination is therefore about **0.487 m beyond the nominal P-point reach**. A conventional short downward gripper does not resolve this horizontal shortfall. Exact TCP reach depends on the tool transform, but an unusually long offset tool would be a different design requiring explicit validation. Even the closest corner of a centred 0.30 m square is approximately 1.202 m from the origin.

**Action:** obtain instructor clarification before claiming compliance with this layout. Suggested message, for your team to send:

> The simulation task places Box A at (1 m, 1 m, 0), while the supplied GP7 has a maximum horizontal reach of 927 mm. May we use a reachable Box A location, or is another coordinate interpretation intended? We will keep the robot dimensions and joint limits unchanged. Also, should box coordinates refer to their base centres, and is a simulated grasp attachment acceptable?

While awaiting clarification, keep the required scene in a configuration file and develop using a separately labelled provisional scene. A candidate Box A base centre such as `(0.50,0.30,0)` has radius 0.583 m, but still requires full orientation-constrained IK and collision checks. It is **not an approved replacement**. Do not scale the robot, move its base, or report an alternative scene as the original specification.

Interpret floor-standing box coordinates as base centres provisionally. With centre-origin box geometry:

| Item | Size | Geometry-centre height |
|---|---|---|
| Box A | 0.30 × 0.30 × 0.20 m | 0.100 m |
| Box B on ground | 0.05 × 0.05 × 0.05 m | 0.025 m |
| Box B resting on A | Same | 0.225 m |

Putting the centre of Box A at z=0 would bury half of it in the floor. Keep the distinction between assignment coordinates, object centre, flange, and TCP explicit.

## 4. Requirements and proof

| Requirement | Implementation | Evidence to capture |
|---|---|---|
| Group of at most three | Record members and index numbers | Report title page |
| GP7 in Gazebo | Six revolute joints and fixed base at origin | Scene and joint-motion demonstration |
| Correct boxes and placement | Parameterised world file | Dimension/pose configuration and screenshot |
| Random reachable Box B | Seeded sampling followed by full feasibility checks | Seed and accepted starting pose |
| Box B orientation unchanged | Fixed object rotation throughout transport | Measured orientation-error plot |
| Coordinate frames | Label base, links, flange and TCP | Frame diagram/screenshots |
| D–H parameters | Measured geometry and explicit convention | Six-row table and offsets |
| Forward kinematics | Product of homogeneous transforms | Equations and numerical validation |
| Inverse kinematics | Derived analytical or numerical method | Derivation, code and residuals |
| Smooth velocity | Time-parameterised trajectory | Desired/actual joint velocity plots |
| Velocity control | Feedback controller tracking trajectory | Controller explanation and tracking results |
| Submission | Concise report and recorded demonstration | Final files; optional repository link |

“Upright (orientation unchanged)” is best implemented as **constant roll, pitch, and yaw**, not merely a vertical box axis. Add a coloured face or marker to the cube so yaw changes are visible.

## 5. Software and project organisation

Recommended baseline: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, `ros_gz`, `ros2_control`, `gz_ros2_control`, and RViz. Gazebo documents Harmonic as the pairing for Jazzy [1]. Use documentation for these versions consistently.

Proposed repository structure:

```text
robotics_project/
  README.md
  src/
    gp7_description/       # Xacro/URDF, meshes, joint and tool parameters
    gp7_simulation/        # Gazebo worlds, spawn/launch, ROS bridges
    gp7_control/           # Controller configuration and launch
    gp7_kinematics/        # D–H table, FK, Jacobian, IK
    gp7_task/              # Sampling, trajectory, grasp state machine
  config/                 # Required/provisional scene and test seeds
  tests/                  # Numerical and integration checks
  results/                # Logs, plots and run metadata
  docs/                   # Frame drawings, report and demo script
```

Make one launch command start the complete simulation. Use simulation time throughout. Record package versions, dependencies, seed, tool transform and scene configuration for each final run.

## 6. Phase A — measure and define the model

1. Open the top-level SolidWorks assembly with its original directory structure. Resolve missing references and inspect the duplicate assembly before choosing which one to export.
2. Identify the six axes in order: **S → L → U → R → B → T**.
3. Set and document a reference pose. Measure joint-axis locations and directions, common-normal distances, flange transform, and any offsets.
4. Compare measurements with the actual GP7 drawing: manual **printed page 5-6, PDF page 46**. It shows dimensions including 330, 445, 40, 440 and 80 mm; these are drawing dimensions, not automatically a D–H table.
5. Choose standard D–H and state it explicitly. Define joint i about `z_(i-1)`, and `x_i` along the common normal. Document choices for intersecting axes.
6. Produce a six-row table with `a_i, alpha_i, d_i, theta_i`, including `theta_i = sign_i*q_i + offset_i`. Fill it from measured axis geometry, not a generic robot table.
7. Separately define the URDF link/joint frames and their mapping to the D–H frames. They need not be identical, but must describe the same motion.
8. Define `T_flange_tcp` and the gripper geometry before calculating pick/place poses.

For standard D–H:

```text
A_i = RotZ(theta_i) TransZ(d_i) TransX(a_i) RotX(alpha_i)

      [ cθ  -sθ cα   sθ sα   a cθ ]
A_i = [ sθ   cθ cα  -cθ sα   a sθ ]
      [  0      sα      cα      d ]
      [  0       0       0      1 ]

T_world_tcp = T_world_dh0 A_1 A_2 A_3 A_4 A_5 A_6 T_dh6_tcp
```

**Gate A:** frame drawing, measured table, sign/offset mapping and tool transform reviewed by another team member. A final numerical D–H table cannot be responsibly inferred from the ZIP filenames or the marketing drawing alone.

## 7. Phase B — simplified robot and forward kinematics

Build a primitive-geometry URDF/Xacro with the measured axes. Fix the base to the world. Include inertial properties, collision geometry, joint limits, and the tool. Check each joint individually with a small positive command and compare motion with the frame diagram.

GP7 floor-mounted limits from manual Table 5-2 and datasheet GP7 column:

| Joint | Minimum | Maximum | Manufacturer maximum speed |
|---|---:|---:|---:|
| S | −170° | +170° | 375°/s |
| L | −65° | +145° | 315°/s |
| U | −70° | +190° | 410°/s |
| R | −190° | +190° | 550°/s |
| B | −135° | +135° | 550°/s |
| T | −360° | +360° | 1000°/s |

Convert degrees to radians in software. Transform limits if your coordinate signs or offsets differ. Treat T as bounded; do not declare it continuous merely because it turns through a full revolution in each direction. Use much lower configurable speeds for development. The supplied speed limits are not acceleration or actuator torque limits; label any assumed simulation values.

Implement FK and compare against the URDF transform at home and at least 20 varied valid configurations. Compare both position and rotation; an incorrect tool offset can hide behind an apparently plausible endpoint.

**Gate B:** FK and URDF agree within numerical tolerance, joints move in the intended directions, and the robot is stable in Gazebo.

## 8. Phase C — inverse kinematics

Start with numerical damped least-squares IK derived from your FK. It is practical and explainable. Confirm with the instructor if a closed-form analytical solution is expected by “derive the equations.”

For a revolute chain, use a geometric Jacobian expressed in the base frame:

```text
Jv_i = z_(i-1) × (p_tcp − o_(i-1))
Jw_i = z_(i-1)
e_p  = p_desired − p_current
e_R  = Log(R_desired R_currentᵀ)^vee
```

Use position/orientation scaling to handle metres and radians:

```text
e_w = W [e_p; e_R]
J_w = W J
Δq  = J_wᵀ (J_w J_wᵀ + λ² I)^−1 e_w
q_next = q + η Δq
```

Use a linear solve rather than explicitly computing a matrix inverse. Bound each update, enforce joint limits, use a line search or step reduction, and stop on convergence or a finite iteration limit. Seed each waypoint from the previous solution to preserve continuity. Retry several seeds for initial poses. Reject any solution with excessive pose residual, collisions, or a discontinuous branch change.

Test reachable FK-generated poses, known unreachable poses, joint-limit cases, and near-singular configurations. An IK failure should produce a clear failure state, never a substituted zero-angle command.

**Gate C:** both the grasp and place poses and the entire connecting path pass IK, joint limits and collision checks. Being inside a 927 mm circle is only a preliminary screen.

## 9. Phase D — scene, tool and grasp

Use a simple parallel-jaw gripper whose opening exceeds 50 mm with clearance. Ensure the fingers can reach around a ground-level cube without hitting the floor. Include the tool mass and payload in the model.

Sample Box B with a fixed random seed inside a candidate region. Reject samples that intersect the base or Box A, cannot be approached vertically, fail the fixed-orientation IK, or lack a collision-free transfer path. Use a maximum attempt count and report sampling failure.

Two grasp implementations:

- **Contact grasp:** fingers close with friction and force sufficient to carry the cube. This requires contact tuning and explicit slip checks.
- **Simplified attachment:** attach the cube to the tool only after the tool is within a defined grasp tolerance and the gripper is closed. Release at the placement surface. Disclose this modelling assumption and confirm it is acceptable; it is not evidence of a realistic friction grasp.

Do not teleport the cube along the trajectory. Preserve the relative transform at attachment. With `T_tcp_box` fixed during grasp:

`T_world_tcp_desired = T_world_box_desired inverse(T_tcp_box)`.

This is why the desired box centre cannot be used directly as the desired TCP. For a ground-resting cube, its centre is z=0.025 m and its top is z=0.05 m; a top-surface TCP would refer to the latter.

**Gate D:** lift and hold the cube without unintended rotation or slip, then release it and let it settle under gravity.

## 10. Phase E — path and smooth trajectory

Use a state machine:

```text
HOME → PREGRASP → DESCEND → CLOSE → VERIFY_GRASP
     → LIFT → TRANSFER → LOWER → RELEASE → VERIFY_PLACE → RETREAT → DONE
```

Every action needs a success check and timeout. On invalid planning or tracking, stop progressing through the state machine. Do not release a carried object simply because a timer expired.

Plan object poses first while carrying:

1. Lift vertically clear of the ground and nearby obstacles.
2. Transfer above Box A with clearance for the **whole robot, gripper and payload**.
3. Lower vertically until the cube rests at centre height 0.225 m.
4. Release near contact, allow settling, and retreat without touching the cube.

Maintain `R_world_box(t) = R_initial` for the complete carry motion. Derive TCP rotation through the grasp transform. Endpoint-only orientation checks are insufficient, and straight interpolation between joint configurations generally does not preserve object orientation.

For each Cartesian segment, use a smooth quintic time law:

```text
τ = t/T, 0 ≤ τ ≤ 1
s(τ)   = 10τ³ − 15τ⁴ + 6τ⁵
s_dot  = (30τ² − 60τ³ + 30τ⁴)/T
s_ddot = (60τ − 180τ² + 120τ³)/T²
p(t)   = p_start + s(τ)(p_end − p_start)
R(t)   = R_fixed
```

This has zero velocity and acceleration at segment endpoints, making a stop at lift/transfer/lower transitions straightforward. It is a smooth polynomial profile, not a strict jerk-limited seven-phase S-curve; explain that distinction if asked.

Sample the Cartesian path, solve continuous IK, and derive joint velocities and accelerations consistently. Check the interpolated controller trajectory between samples as well. Refine sampling where curvature or orientation error is high.

Choose the duration conservatively, then evaluate every joint. If limits are exceeded, increase duration by at least:

`k = max(1, max_i,t |qdot_i|/vlimit_i, sqrt(max_i,t |qddot_i|/alimit_i))`

and regenerate timing and derivatives. Add margin. Acceleration limits must be documented engineering choices where not specified. Time scaling cannot repair a singularity, branch jump or collision; replan those paths.

**Gate E:** continuous-path checks show fixed object orientation, clearance, no joint-limit violation and smooth bounded velocities.

## 11. Phase F — execution and velocity feedback

Suggested data flow:

```text
Task state machine
  → desired object poses → TCP poses
  → Cartesian trajectory → IK → timed joint trajectory
  → joint_trajectory_controller → gz_ros2_control → Gazebo
                                  ↑ joint feedback
Gazebo object state → independent success/orientation evaluator
```

Use `joint_state_broadcaster` and a `joint_trajectory_controller`. The Gazebo control plugin connects ROS control interfaces to simulated joints [2]. For explicit velocity feedback, configure a velocity command interface and the required position/velocity state interfaces; the trajectory controller supports PID mapping of tracking errors to velocity commands [3]. Tune at low speed.

A simple controller explanation is:

`qdot_cmd = qdot_ref + Kp*(q_ref − q_measured)`

with command saturation and tracking-error monitoring. A configured PID adds integral/derivative behaviour; explain the controller actually used rather than claiming this simplified expression is its exact implementation. Avoid competing controllers commanding the same joint.

Send time-stamped trajectory points with consistent position and velocity data, inspect the action result, and verify measured convergence before changing task state. Merely assigning object or joint poses does not demonstrate velocity control.

Log time, desired/actual joint position and velocity, TCP pose, cube pose, controller result and task state. Keep trajectory planning and feedback measurements in the same clock domain.

**Gate F:** bounded tracking error, no persistent saturation, and all states finish using measured checks.

## 12. Phase G — replace visuals with CAD

Once the simplified robot works:

1. Export meshes for each rigid link group in a documented reference frame, using a format supported by the selected simulator.
2. Convert millimetres to metres exactly once; verify a known dimension.
3. Retain the validated joint origins, axes and limits.
4. Use detailed meshes for visuals and simple primitives/convex approximations for collision.
5. Obtain credible link mass properties from CAD with assigned materials, or explicitly document estimates. Check centre-of-mass frames and positive-definite inertia tensors. Do not invent “manufacturer” inertias.
6. Exclude envelope/reference parts from physical geometry.
7. Repeat FK comparison, collision checks and the successful transport test.

**Gate G:** CAD visuals match the validated kinematic chain and do not degrade simulation stability or introduce hidden collisions.

## 13. Validation and measurable success

Suggested project targets below are **team-defined**, not numerical thresholds given in the task sheet. Adjust and document them before final evaluation.

| Check | Suggested acceptance |
|---|---|
| Mathematical FK vs URDF | Position error < 0.1 mm; rotation error < 0.01° |
| IK residual before execution | Position error < 1 mm; rotation error < 0.5° |
| Cube orientation during carry | Maximum full rotation error < 1° |
| Final cube centre | Within 5 mm horizontally of Box A centre |
| Resting height | Near 0.225 m with stated contact tolerance |
| After release | Remains on Box A, stable for at least 2 s |
| Limits | No joint position or configured speed/acceleration violation |
| Collisions | None except intended finger/object and support contact |
| Repeatability | Ten recorded, seeded reachable starts; report every failure |

Use the full rotation error:

`angle_error = acos(clamp((trace(R_initialᵀ R_measured) − 1)/2, −1, 1))`.

Evaluate **measured cube orientation**, not only desired TCP orientation. Plot joint velocities and position tracking errors; also plot cube orientation error and position over time. Measure derivatives with care because sampled simulation data can be noisy.

Include an unreachable-destination test for `(1,1)` to document why clarification is necessary. A graceful “infeasible” result is useful evidence, but it does not replace a successful approved pick-and-place demonstration.

## 14. Team split and suggested ten-session schedule

| Session | Work | Completion evidence |
|---|---|---|
| 1 | Clarify target, freeze stack, open CAD | Decisions, initial assembly inspection |
| 2 | Frames, axis measurements and D–H | Reviewed geometry and frame diagram |
| 3 | Simplified URDF and FK | Correct individual joint motion and FK checks |
| 4 | IK and feasible workspace | Residuals and feasible path examples |
| 5 | Boxes, tool and grasp | Lift/hold/release demonstration |
| 6 | Cartesian path and time law | Fixed orientation and bounded planned velocities |
| 7 | Feedback control and task integration | Full provisional-scene cycle with logs |
| 8 | CAD visuals and robustness | Stable full-model cycle |
| 9 | Multiple starts, failure cases, plots | Validation table and saved results |
| 10 | Report, recording and clean-start test | Submission package |

For three members: **A** owns CAD/URDF/world, **B** owns frames/FK/IK, and **C** owns trajectory/control/logging. All three agree on axis signs, frames, units and tool transform before implementation. Review one another's outputs and integrate daily. For fewer members, keep the same work sequence and combine roles.

The critical dependency chain is: geometry → FK → IK → pose path → timing → control → demonstrated result. Gripper and world modelling can develop alongside kinematics after frame conventions are agreed.

## 15. Report and video workflow

Write the report as you work:

1. Members and index numbers; task and chosen implementation.
2. Source files, assumptions and any instructor-approved layout clarification.
3. Labelled link frames, measured geometry and D–H table.
4. FK transforms and a numerical example.
5. IK derivation, solver choices and validation.
6. Object/TCP transforms and constant-orientation path.
7. Time law and velocity controller, with the requested code snippets.
8. Simulation, gripper modelling and implementation details.
9. Results with measured plots, tolerances, failures and limitations.
10. References, credits and optional repository link.

Suggested demonstration: show the complete scene and coordinate axes; display starting/destination coordinates and seed; run a continuous pick → lift → transfer → lower → release → retreat; show the cube stable after release; then show the measured orientation and joint-velocity plots. A marked cube face makes orientation preservation visible. State explicitly if the grasp is an attachment abstraction.

Before submission, start from the documented launch procedure, confirm all mesh paths are portable, verify the video opens, check report member details, and include the repository link if used. The task sheet requires one group submission containing the report and video; it does not supply a deadline or prescribed report length.

## 16. If you choose the physical route instead

The brief permits a serial manipulator with at least four DoF and acknowledged open-source designs. First select a mechanism that supports the required fixed object orientation over the chosen path; four DoF alone does not guarantee arbitrary 3D position plus constant full orientation. Define a reachable workspace, gripper and payload; size actuators from gravity loads and acceleration demands; provide suitable power, homing and joint limits; calibrate offsets and link dimensions; then perform the same FK/IK, trajectory and feedback validation. Add hardware assembly and bench-test time before the integration sessions. The GP7-specific box layout appears under the simulation branch of the brief.

## References and analysis artifacts

Local sources: `Task_sheet.pdf`; GP7 manual HW1483944, Table 5-2 and Fig. 5-3(b); GP7/GP8 datasheet DS-866 (2021); supplied SolidWorks archive.

Online primary documentation checked for the proposed software stack:

1. [Gazebo Harmonic documentation](https://gazebosim.org/docs/harmonic/).
2. [gz_ros2_control — Jazzy](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html).
3. [joint_trajectory_controller — Jazzy](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html).

Extracted PDF text is in `analysis/`. The actual GP7 dimension drawing is saved as `analysis/gp7_dimensions.png` for convenient inspection. Original resources have been preserved.
