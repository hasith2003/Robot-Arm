# Source and licensing

The repository's existing MIT license is preserved in `LICENSE`. The ROS
packages declare Apache-2.0 in their package manifests; that license text is
included in `LICENSE-APACHE`. The MIT license does not replace third-party
licensing terms.

## Yaskawa GP7 description

`gp7_ws/src/yaskawa_gp7_support` vendors the URDF/SRDF description and STL
meshes from [fizyr/yaskawa_gp7_support](https://github.com/fizyr/yaskawa_gp7_support),
upstream commit `47ebe48f9177c79ab0417625ed121c3edb46234a`.
Upstream names Maarten de Vries as author and declares Apache V2.0 in
`package.xml`. Its original README, author metadata and package license are
retained, and the Apache license text is included in the package.

Project changes port its build to ROS 2 ament, resolve mesh paths, remove a
duplicate Xacro constant, add approximate simulation inertias and parameterize
effort limits. The robot geometry is imported; these dynamics are not measured
manufacturer data. Legacy ROS 1 launch files are retained for provenance;
use the project's ROS 2 launcher instead.

The gripper STL meshes are original simplified project geometry, with a
reproducible Python generator. No proprietary assignment PDFs or SolidWorks
archives are distributed in this repository.
