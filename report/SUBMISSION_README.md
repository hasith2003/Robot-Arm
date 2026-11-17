# EN3563 Robotics group report

The report is personalized for:

- N.A.H.G. Nettikumara — 230436X
- K.PCP. Keerawella — 230332B
- J.U.P. Lelwala — 230374E

Repository: https://github.com/hasith2003/Robot-Arm

`main.pdf` is the compiled report. `main.tex` and `figures/` are the complete
Overleaf source. In Overleaf, use **New Project → Upload Project**, select the
provided Overleaf ZIP, set `main.tex` as the main document and compile with
pdfLaTeX. No index-number or repository placeholders remain. The formal
references section and citation markers have been removed as requested;
brief model-source acknowledgement remains in the introduction.

For local compilation, run `pdflatex main.tex` twice, or `tectonic main.tex`.
LaTeX Workshop in VS Code requires an installed TeX compiler.

The robot figures use the actual project URDF visual meshes, with frame
origins calculated by FK. Full x/y/z triads on enlarged actual-mesh crops, the terminal z6 axis, both 40 mm
offsets, viewing direction and recorded panel-(b) angles are included.

## Validation evidence

`validation_evidence/` includes:

- `interpolation_samples.csv`: all 321 samples for each of five nominal
  carried-object segments, 1,605 samples in total.
- `interpolation_results.json`: per-segment errors and acceptance thresholds.
- `pytest.txt` and `pytest_repository_copy.txt`: 16 passing tests in each copy.
- `test_urdf_model.py`: the interpolation test source.
- `frame_annotation_checks.json`: 1,004 configurations checking D–H axis
  constraints, right-handed frames and agreement with URDF axes and TCP.
- `figure_pose.json`: full-precision angles and view conventions for Figure 1.
- `joint_coordinate_mapping.json`: comparison to the ROS-Industrial GP7
  coordinate chain; no joint-angle offset or sign conversion is required.
- `recorded_measured_joints.csv` and `recorded_joint_limit_check.json`:
  all recorded joint states and their recheck against the corrected bounds.
- `corrected_limits_run.json` and `corrected_limits_simulation.log`: a fresh
  Gazebo cycle reaching DONE with U-axis bounds -70 to +190 degrees enforced.

The original recorded GUI run used the earlier imported U bounds. Its
unmodified result is retained in `recorded_run_original_result.json` for
provenance. Its original margin field uses those earlier bounds. The report's
corrected 0.471 rad margin is computed from the recorded CSV using the new
standard-GP7 bounds and independently confirmed by the new Gazebo run. The
joint path remained inside the corrected range throughout. `results.json`
groups the original run, the recheck and the fresh run without conflating them.

The accompanying video is an actual recording of the Gazebo renderer. The
report retains the selected reachable destination and explicitly describes
the idealized fixed-joint grasp and approximate simulation dynamics.
