# Standard GP7 U-axis limit correction — 17 November 2026

The supplied manufacturer's GP7 instructions, Table 5-2, and general GP7
datasheet list U-axis limits **-70 to +190 degrees**. The imported fizyr
description used **-116 to +255 degrees**. The latter values must not be
presented as verified limits for the supplied general-GP7 specification.

The correction was made after checking the coordinate mapping, not by
assuming that labels in a working-envelope drawing were joint limits.

Primary comparison model:
https://github.com/ros-industrial/motoman/blob/noetic-devel/motoman_gp7_support/urdf/gp7_macro.xacro

Its six positive axes and zero-angle rotations agree with the project's
description. Some origins differ along the same rotation axis: the first
origin is moved along Z; the roll origin along its X-axis line; and the
terminal offset is represented as a fixed transform. Those translations
commute with their associated revolute rotations. Therefore the joint-angle
mapping is identity, without an extra U offset or sign reversal.

`analysis/verify_gp7_limit_mapping.py` independently assembles that reference
chain and compared 1,001 configurations. The maximum TCP matrix difference
was 4.45e-16, positive-axis difference was zero, and axis-line mismatch was
below 2.88e-16 m. This establishes equivalence of the model coordinates,
not physical controller calibration.

Changed in both workspace copies:

- The imported URDF's joint_u limits are now -70/+190 degrees.
- `GP7Kinematics.LOWER/UPPER` use the same corrected values.
- The existing bounds regression test independently checks the manufacturer
  values, so changing both models to the same wrong range will not pass.
- The report uses the corrected bounds and re-evaluated margins.

The D-H parameter table, frame locations, link lengths and accepted box
positions were not changed. All recorded U angles, approximately -43 to
0 degrees, are inside the corrected interval. Rechecking 5,071 measured
states gives a minimum margin of **0.4712429846 rad**, replacing the old
1.030 rad margin calculated with the earlier wider imported bounds.

All 16 tests pass in both workspace copies. A fresh isolated Gazebo run with
the corrected bounds reached DONE, placed the cube at the same target and
passed orientation, speed and post-retreat stability checks. Evidence is in
`analysis/gp7_corrected_limits_run` and `report/validation_evidence`.

Earlier audit files and original recorded-run JSON are retained as history.
Their old margin field describes the earlier software bounds; it must not
be interpreted as the margin under the corrected standard-GP7 limits. The
submission report explicitly distinguishes recorded GUI evidence from the
new corrected-limit run and the bounds recheck.
