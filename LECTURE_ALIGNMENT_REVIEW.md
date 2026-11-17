# GP7 review against the six supplied lecture-note PDFs

Reviewed 17 November 2026. Page references below are **PDF page numbers**, not
necessarily the slide numbers printed in the corner. The PDFs were treated as
technical reference material. Handwritten annotations and pasted chat/AI
screenshots were distinguished from the original typeset lecture material.

## Findings

The current standard D-H table, frame definitions in `KINEMATICS_DERIVATION.md`,
the matrix-generated figure in the report and the implemented FK are
consistent with the lecture rules and the project's URDF. No robot dimensions,
joint signs, scene positions or working simulation algorithms were changed
as part of this review.

**Correction to earlier illustrations:** the generated photo annotation
`analysis/gp7_frame_origins_annotated.png` is not an accurate rendering of the
implemented positive axes. In the pictured downward-tool pose, its z3 arrow
points distally, whereas the code's z3 points toward the elbow along the
forearm; its z5 arrow points downward, whereas the code's z5 points upward,
opposite the TCP approach axis. Its O0 placement is also only schematic: the
actual O0 is at base height 0.330 m, level with the shoulder origin. Do not use
that photo as the report's frame definition. The earlier generated infographic
also contained incorrect inset arrows, even where its parameter table was
correct. Use the exact matrix-generated figures instead.

Correct figures:

- `validation/latest/dh_coordinate_frames.pdf`: full right-handed D-H triads at q=0;
  this is the figure already included in the compiled report.
- `validation/lecture_alignment/verified_bent_pose_axes.pdf`: exact projected
  origins and joint z-axis directions at the recorded retreat pose. This
  intentionally shows joint z axes only; it does not claim to show full triads.

## 1. Why O3 does not coincide with O4 and O5 here

Lecture 6, p.15 gives two constraints:

1. The x_i axis is perpendicular to z_(i-1).
2. The x_i axis intersects z_(i-1).

Lecture 6, pp.24–30 explains that assignments are not unique, but they must
obey those constraints. The z_i axis is associated with joint i+1, not joint i.

At q=0, the relevant axes of this GP7 are:

```text
z2: elbow joint axis: (0.040, 0, 0.775) + lambda (0, -1, 0)
z3: wrist-roll axis: (0.040, 0, 0.815) + mu     (-1, 0, 0)
```

They are skew, separated by a 0.040 m common normal. Choosing x3=+Z places
O3 at (0.040, 0, 0.815). The x3 line intersects the elbow axis at
(0.040, 0, 0.775), satisfying the second D-H constraint.

The wrist-axis intersection is instead

```text
W = O4 = O5 = (0.480, 0, 0.815) m.
```

If O3 were moved to W while keeping x3=+Z, its x3 line would have x=0.480,
whereas the elbow axis has x=0.040. The two lines would miss each other by
**0.440 m**, violating the second D-H constraint. Choosing a diagonal x3 from
the elbow to W would not solve this: that line would not be perpendicular to
z3, and a coordinate frame's x3 must also be perpendicular to its own z3.

Thus, for these physical joint axes and this standard-D-H assignment,
**O4=O5, but O3 is 440 mm back along the joint-4 axis**. Merely moving O3 to W
is not a permitted cosmetic change to the existing frame table.

Lecture 7, pp.35–39 treats an **isolated three-joint wrist**, numbered locally
from 0. Its local O0 can be placed at the wrist centre because no preceding
elbow axis constrains that local base. Those local frame labels cannot simply
be substituted into the full six-joint chain. The handwritten annotation on
Lecture 8, p.15 explicitly says O_c=O4=O5 and that O3 is there *often*, not
always. The generic example does not require all three full-arm origins to
coincide in this GP7.

## 2. Frame assignment and D-H table

The current zero-angle assignment is:

| Frame | Origin in robot base, m | x | y | z |
|---|---|---|---|---|
| 0 | (0, 0, 0.330) | +X | +Y | +Z |
| 1 | (0.040, 0, 0.330) | +X | -Z | +Y |
| 2 | (0.040, 0, 0.775) | +Z | -X | -Y |
| 3 | (0.040, 0, 0.815) | +Z | +Y | -X |
| 4 | (0.480, 0, 0.815) | +Z | -X | -Y |
| 5 | (0.480, 0, 0.815) | +Z | +Y | -X |
| 6 | (0.560, 0, 0.815) | +Z | +Y | -X |

All triads obey x cross y = z. The z_(i-1) directions and their axis lines
match the positive joint axes in the URDF. At this zero pose joint 4 and
joint 6 are collinear; a spherical wrist does **not** require all three axes
to be pairwise perpendicular at every pose. Some pasted AI material in the
annotated notes implies otherwise; the original lecture definition only
requires intersection of the three revolute axes. Adjacent wrist axes remain
perpendicular; the first and third axes can coincide at a wrist singularity.

Lecture 6, p.20 and Lecture 7, p.3 use exactly the implemented order:

```text
A_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i)
```

| i | a_i, m | alpha_i | d_i, m | theta_i |
|---|---:|---:|---:|---|
| 1 | 0.040 | -90 deg | 0 | q1 |
| 2 | 0.445 | 180 deg | 0 | q2 - 90 deg |
| 3 | 0.040 | -90 deg | 0 | q3 |
| 4 | 0 | +90 deg | -0.440 | q4 |
| 5 | 0 | -90 deg | 0 | q5 |
| 6 | 0 | 0 deg | -0.080 | q6 |

The 180-degree twist in row 2 is valid: the chosen L/U positive axes are
**antiparallel**. Lecture 6, p.30 illustrates same-direction parallel axes
with zero twist; it does not require antiparallel directed axes to have zero
twist. Negative d values follow the signed axes and do not imply negative
physical distances.

Lecture 6, p.26 allows O0 anywhere on the first joint axis, so the 0.330 m
fixed base transform is valid. The complete FK must include it.

### Final-frame naming

Lecture 6, p.31 normally places the terminal frame at the tool between the
fingers, with its z axis as the approach direction. Our implementation uses an
auxiliary D-H frame 6 at the flange and explicitly adds a separate grasp TCP:

```text
T_base,TCP = Tz(0.330) A1 A2 A3 A4 A5 A6 [Rx(pi) Tz(0.1075)]
```

This complete chain is valid. Clarify in the report/diagram that DH6 is the
flange frame and TCP is the approach frame. In the downward-tool pose, their
positive z axes point in opposite directions.

An optional equivalent lecture-style convention puts frame 6 directly at TCP:
keep rows 1–5 and replace row 6 by

```text
a6=0, alpha6=180 deg, d6=-0.1875 m, theta6=q6.
```

Then omit the separate final TCP transform. Algebraically,

```text
Rz(q6) Tz(-0.080) Rx(pi) Tz(0.1075)
  = Rz(q6) Tz(-0.1875) Rx(pi).
```

Both conventions were checked numerically. They must not be mixed or the
tool offset would be counted twice. This is an optional reparameterization,
not a required repair of the current model.

## 3. Inverse kinematics: what to add to match the lectures

Lecture 8, pp.10–11 distinguishes numerical from closed-form IK and prefers
closed form for efficient branch selection. It does not prove numerical IK
incorrect. The task sheet asks for IK equations, without requiring closed
form. The current bounded numerical solver is valid and its report description
matches the code.

For a stronger lecture-aligned derivation, add wrist decoupling rather than
presenting the optimizer alone. The following formulas match **our joint
signs and TCP**, not an arbitrary textbook arm.

### Wrist centre and inverse position

For a desired TCP pose (R,p),

```text
W = p - 0.1875 R [0,0,1]^T.
```

The positive 0.1875 m is flange-to-wrist plus flange-to-TCP along the tool's
approach direction. Do not use the old D-H d6=-0.080 with R_TCP directly in
the lecture's wrist-centre expression: it omits the gripper and reverses the
relevant axis convention.

Let signed radial distance r be either +hypot(Wx,Wy) or -hypot(Wx,Wy).
Choose q1=atan2(Wy,Wx) for the first branch or that angle plus pi for the
second. Then set

```text
u = r - 0.040;                 v = Wz - 0.330
L2 = 0.445
L3 = sqrt(0.440^2+0.040^2) = 0.44181444068749043 m
beta = atan2(0.440,0.040) = 84.8055710922652 deg
D = (u^2+v^2-L2^2-L3^2)/(2 L2 L3)
phi = atan2( +/-sqrt(1-D^2), D )
q3 = beta - phi
q2 = atan2(u,v) - atan2(L3 sin(phi), L2+L3 cos(phi)).
```

Test both shoulder and elbow branches, allow equivalent 2-pi representations
within limits, and reject |D|>1 beyond numerical tolerance. A wrist centre on
the base axis needs special handling because q1 is then not fixed by atan2.
These are the law-of-cosines and two-link geometry used in Lecture 8, p.25,
with the actual GP7 offset included. Treating the forearm as a straight
0.440 m link without its 0.040 m offset would be wrong for this URDF.

### Inverse orientation and the middle-angle sign

Let R03 be the rotation after A1 A2 A3 and D_tool=diag(1,-1,-1). Then

```text
M = R03^T R_TCP D_tool
  = Rz(q4) Ry(-q5) Rz(q6).
```

Lecture 7, pp.37–39 and Lecture 8, p.33 show a Z-Y-Z wrist, but their twist
signs differ from ours. Therefore the middle Euler angle here is **-q5**, not
q5. For a nonsingular wrist:

```text
s5 = +/-sqrt(M13^2+M23^2)
q5 = atan2(s5,M33)
q4 = atan2(-M23/s5,-M13/s5)
q6 = atan2(-M32/s5, M31/s5).
```

At q5=0, only q4+q6 is determined by the wrist rotation; preserve a suitable
seed/continuity choice and respect limits. A derivation or implementation must
handle singularities and branches rather than divide by zero.

The diagnostic geometric derivation found a limit-valid FK-verified solution
for all 1,003 tested URDF targets, including zero and the two simultaneous
joint-bound configurations. This is not a claim that a complete globally
robust production IK solver has been implemented: the diagnostic is not the
solver used by the demonstrated task and specially constructed singular
cases still require careful treatment.

## 4. Velocity kinematics and control

Lecture 9, pp.14,20,22 and Lecture 11, p.3 imply, in a common base frame,

```text
Jv_i = z_(i-1) cross (p_TCP-o_(i-1))
Jw_i = z_(i-1)
[v_TCP; omega_TCP] = J(q) qdot
Rdot = skew(omega_TCP) R.
```

The implemented geometric Jacobian follows these expressions and uses the
actual TCP position, not the flange position. The additional fixed tool
rotation does not change the spatial angular-velocity vector, but the TCP
translation changes linear velocity and must be included.

Velocity kinematics is not the same thing as a low-level velocity controller.
The task sends timed position/velocity/acceleration waypoints through a
position-command joint trajectory controller. Gazebo supplies position-error
tracking. The report already describes this accurately; do not relabel it as
a custom Jacobian-based resolved-rate controller or a custom velocity PID.
The supplied lectures do not by themselves impose either controller design
as an assignment requirement.

## 5. Trajectory planning: correct parts and a measured improvement

The quintic Cartesian blend beta(s)=10s^3-15s^4+6s^5 matches Lecture 11,
pp.20–23. It gives zero Cartesian velocity and acceleration at both ends.
Keeping a constant TCP rotation during carried segments preserves object
orientation through the fixed grasp transform. Lecture 12's LSPB profile is
an alternative, not a requirement to replace a valid polynomial trajectory.

The interpolated joint trajectory currently uses a **clamped cubic spline**.
It is C2 between internal knots and has zero endpoint joint velocities.
Clamping the first derivatives does not force endpoint second derivatives
to zero. The six nominal arm segments showed maximum endpoint accelerations
between 6.93e-5 and 1.3853e-4 rad/s^2. At a transition to a stationary hold,
this is a small acceleration discontinuity, the concern discussed in Lecture
11, p.19. It is not evidence that the frame assignment is wrong.

Recommended improvement if zero-acceleration joins are desired: fit a quintic
spline with both first and second derivatives constrained to zero at segment
boundaries. A SciPy boundary specification is

```python
zeros = np.zeros(number_of_joints)
curve = make_interp_spline(times, positions, k=5,
    bc_type=([(1, zeros), (2, zeros)], [(1, zeros), (2, zeros)]))
```

This was tested **offline for endpoint conditions only**, giving endpoint
velocity and acceleration at approximately 1e-14 and 5e-13 in their respective
units. Before replacing the working controller trajectory, check polynomial
position/speed extrema, continuous Cartesian position/orientation accuracy
between knots, and rerun the simulation. The current cubic-extremum logic
cannot simply be reused unchanged for a quintic implementation. The change
would remove acceleration steps at holds, but does not automatically imply
continuous jerk or a bounded-jerk seven-segment S-curve.

The report already avoids claiming global acceleration continuity. It should
retain that qualification unless the trajectory implementation is upgraded
and revalidated.

Minor documentation correction recommended in the `CartesianSegment` helper,
which is not used by the running pick-and-place executor: its docstring says
to pass the initial *object*
rotation as the target rotation. Its transforms actually represent the tool
pose. In this task R_object=I while R_tool=diag(1,-1,-1), so the general rule
must include the fixed object-to-tool transform. The working executor already
uses the correct tool rotation; this is not a runtime failure.

## 6. Evidence and recommended actions

Fresh checks in `validation/lecture_alignment/verification_results.json`:

| Check | Maximum error / finding |
|---|---:|
| Configurations checked | 1,003 |
| D-H/URDF TCP position | 6.78e-16 m |
| D-H/URDF TCP orientation | 8.14e-16 rad |
| D-H perpendicular-axis constraint | 3.34e-16 |
| D-H intersection/translation residual | 2.52e-16 m |
| Right-handed triad residual | 5.44e-16 |
| Positive joint-axis line match | 6.49e-16 m |
| O4/O5 origin difference | 0 m |
| Wrist-axis intersection residual | 6.49e-16 m |
| Moving O3 to wrist: missed intersection | **0.440 m** |
| Velocity identities, 100 configurations | <6.38e-10 in respective SI units |
| Existing automated tests | **16 passed** |

1. Keep the current D-H table, base/tool transforms and URDF lengths.
2. Replace the earlier generated photo/infographic annotations with the exact
   figures; distinguish DH6 from TCP and retain O3 near the elbow.
3. Add the geometric decoupling derivation to the report if close alignment
   with Lecture 8 is desired. Include the 40 mm elbow offset, 187.5 mm total
   wrist-to-TCP offset, wrist sign convention, branches and singular cases.
4. Describe the existing velocity-control architecture honestly; its
   mathematical Jacobian already agrees with the velocity notes.
5. Consider zero-acceleration quintic joint interpolation as a separate
   improvement, with new extrema checks and simulation validation.
6. Correct that helper's tool/object-rotation docstring when editing it.

This review provides model consistency checks and explicit mathematical
reasoning. It does not establish independently measured physical dimensions,
global IK completeness, collision avoidance for arbitrary paths, or physical
grasp/dynamics accuracy. The six lecture PDFs do not supply those missing
measurements or guarantees.
