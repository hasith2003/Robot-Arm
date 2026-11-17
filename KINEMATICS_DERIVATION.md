# GP7 standard D-H derivation and implementation

This model is derived from the project's imported URDF geometry, not a fresh
measurement of the SolidWorks CAD. Units are metres and radians. The public
FK/IK pose is `base_link -> grasp_tcp`. The world's fixed transform to
`base_link` is identity in this project. D-H frames are mathematical frames;
they need not coincide with the mesh/link frames named in the URDF.

## 1. Expand and inspect the robot description

From `gp7_ws`, run:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
xacro src/gp7_description/urdf/gp7.urdf.xacro > gp7_expanded.urdf
```

The six arm joint definitions are in
`src/yaskawa_gp7_support/urdf/gp7_macro.xacro`. The wrapper
`src/gp7_description/urdf/gp7.urdf.xacro` adds the gripper and TCP.
Trace the chain from `base_link` through S, L, U, R, B, T, flange,
`tool0`, and `grasp_tcp`. Do not include the finger branches or the
`vendor_base_link` branch in the serial arm chain.

## 2. Record joint axes at the zero configuration

At q=0 all six moving-joint origin rotations are identity. Their axes and
axis points in `base_link` coordinates are:

| Joint | Point on axis (m) | Positive rotation axis |
|---|---|---|
| S | (0, 0, 0) | +Z |
| L | (0.040, 0, 0.330) | +Y |
| U | (0.040, 0, 0.775) | -Y |
| R | (0.040, 0, 0.815) | -X |
| B | (0.480, 0, 0.815) | -Y |
| T | (0.560, 0, 0.815) | -X |

Positions accumulate the parent-frame URDF translations. An axis is a line,
so its D-H origin can be moved along that line without changing the joint.

## 3. Assign the standard D-H frames

Use joint i's positive URDF axis as z_(i-1). Choose x_i along a common
normal to z_(i-1) and z_i. Complete each frame with y_i = z_i cross x_i.
For intersecting axes the common-normal length is zero; choose the listed
perpendicular direction. For the antiparallel L/U axes choose +Z along their
separation. The following is one valid assignment, expressed at q=0:

| Frame | Origin in base_link (m) | x direction | z direction |
|---|---|---|---|
| 0 | (0, 0, 0.330) | +X | +Z (S) |
| 1 | (0.040, 0, 0.330) | +X | +Y (L) |
| 2 | (0.040, 0, 0.775) | +Z | -Y (U) |
| 3 | (0.040, 0, 0.815) | +Z | -X (R) |
| 4 | (0.480, 0, 0.815) | +Z | -Y (B) |
| 5 | (0.480, 0, 0.815) | +Z | -X (T) |
| 6 | (0.560, 0, 0.815) | +Z | -X (terminal) |

Frame 5 is at the B/T axis intersection, 80 mm behind the URDF T origin.
Frame 6 moves those 80 mm along the same T axis. Frame 0 is on the S axis,
330 mm above the URDF base origin. These choices make the D-H transforms
simple while preserving the robot exactly.

## 4. Read the four parameters between each pair of frames

This implementation uses standard D-H with

```text
A_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i)
theta_i = q_i + offset_i
```

Measure a_i along x_i, alpha_i about x_i, d_i along z_(i-1), and theta_i
about z_(i-1). Positive rotation follows the right-hand rule. Here every q_i
has the same positive direction as its URDF joint; no extra joint sign is
required.

| i / joint | a_i (m) | alpha_i | d_i (m) | theta_i |
|---|---:|---:|---:|---|
| 1 / S | 0.040 | -pi/2 | 0 | q_S |
| 2 / L | 0.445 | pi | 0 | q_L - pi/2 |
| 3 / U | 0.040 | -pi/2 | 0 | q_U |
| 4 / R | 0 | pi/2 | -0.440 | q_R |
| 5 / B | 0 | -pi/2 | 0 | q_B |
| 6 / T | 0 | 0 | -0.080 | q_T |

Negative d_4 and d_6 occur because the corresponding z axes point toward -X.
Different valid frame choices can produce different tables. A table is only
meaningful together with its frame definitions, base transform and tool transform.

The standard matrix is:

```text
A_i = [ cos(theta) -sin(theta)cos(alpha)  sin(theta)sin(alpha) a cos(theta) ]
      [ sin(theta)  cos(theta)cos(alpha) -cos(theta)sin(alpha) a sin(theta) ]
      [     0             sin(alpha)            cos(alpha)          d      ]
      [     0                  0                     0              1      ]
```

## 5. Include the base and TCP transforms

```text
T_base_DH0 = translation(0, 0, 0.330)
T_DH6_tool0 = Rx(pi)
T_tool0_TCP = translation(0, 0, 0.1075)

T_base_TCP(q) = T_base_DH0 A_1(q_S) ... A_6(q_T)
               T_DH6_tool0 T_tool0_TCP
```

The URDF flange rotation is Rz(pi) Ry(-pi/2). Combined with the orientation
of D-H frame 6, this gives `T_DH6_tool0 = Rx(pi)`. The combined final matrix is:

```text
T_DH6_TCP = [ 1  0  0   0      ]
           [ 0 -1  0   0      ]
           [ 0  0 -1  -0.1075 ]
           [ 0  0  0   1      ]
```

At zero angles, FK must give:

```text
T_base_TCP(0) = [ 0  0  1  0.6675 ]
               [ 0 -1  0  0      ]
               [ 1  0  0  0.815  ]
               [ 0  0  0  1      ]
```

`transforms(q)` returns the seven D-H frame poses in `base_link` coordinates.
`fk(q)` adds the final TCP transform. To calculate the flange/tool0 pose,
use `transforms(q)[-1] @ DH6_TO_TOOL0`.

## 6. Compute the TCP Jacobian and solve IK

For each revolute joint, use its pre-joint D-H frame origin o_(i-1) and
z axis in base coordinates, with the actual TCP position p:

```text
J_linear[:, i] = z_(i-1) cross (p - o_(i-1))
J_angular[:, i] = z_(i-1)
```

The position error is p_target - p_current. The angular error is the rotation
vector of R_target R_current^T, expressed in base coordinates. SciPy's rotation
logarithm handles 180-degree rotations, fixing the former division-by-sine bug.

The numerical IK update uses weighted damped least squares:

```text
Jw = W J, ew = W error, W = diag(1, 1, 1, 0.25, 0.25, 0.25)
delta_q = Jw^T (Jw Jw^T + damping^2 I)^(-1) ew
```

Steps are limited to 0.15 rad per joint and projected into URDF joint limits.
If this stalls, a bounded least-squares solve attempts recovery from the
last iterate. Each stage has a `max_iterations` iteration/evaluation budget.
Success requires BOTH position error < 0.1 mm and angular error < 0.001 rad
by default. A failed local solve does not prove a pose is unreachable; use
a suitable seed or another branch. Multiple valid joint solutions can produce
the same TCP pose. IK does not check collisions or trajectory speed.

URDF limits in degrees are S [-170,170], L [-65,145], U [-116,255],
R [-190,190], B [-135,135], T [-360,360].

## 7. Validate against an independent model

The regression tests expand the URDF and compare the hand-derived chain
against `URDFModel`, rather than generating all expectations with the D-H FK.
They check random configurations and limits for both TCP and flange FK,
all six Jacobian rows against finite differences of the URDF, limits/speeds,
IK against URDF-generated poses, the downward pick descent, the 180-degree
orientation regression, and rejection of a distant unreachable target.

```bash
cd gp7_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest -q src/gp7_task/test
python3 src/gp7_kinematics/gp7_kinematics/kinematics.py
```

The successful simulation continues using its live-URDF planner. This D-H
implementation provides a matching, independently checked mathematical model
for the report and callers of `GP7Kinematics`.

Validation result: all 12 tests pass in both workspace copies. An additional
1,000 random URDF/D-H comparisons produced maximum position error
6.87e-16 m and maximum angular error 7.83e-16 rad (floating-point precision).
