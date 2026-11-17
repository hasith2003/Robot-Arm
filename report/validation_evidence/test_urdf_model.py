import pathlib
import subprocess
import unittest
import numpy as np
from ament_index_python.packages import get_package_share_directory
from gp7_kinematics.urdf_model import URDFModel


class URDFKinematicsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path=pathlib.Path(get_package_share_directory('gp7_description'))/'urdf/gp7.urdf.xacro'
        cls.model=URDFModel(subprocess.check_output(['xacro',str(path)],text=True))

    def test_zero_pose_matches_imported_link_dimensions(self):
        # 40+440+80+107.5 mm along X; 330+445+40 mm along Z.
        np.testing.assert_allclose(self.model.fk(np.zeros(6))[:3,3],[.6675,0,.815],atol=1e-10)

    def test_jacobian_matches_finite_difference(self):
        q=np.array([.2,.3,-.4,.1,-.5,-.1]); j=self.model.jacobian(q)
        for i in range(6):
            dq=np.zeros(6);dq[i]=1e-6
            numerical=(self.model.fk(q+dq)[:3,3]-self.model.fk(q-dq)[:3,3])/2e-6
            np.testing.assert_allclose(j[:3,i],numerical,atol=1e-7)

    def test_downward_descent_preserves_orientation_and_limits(self):
        t=np.eye(4);t[:3,:3]=np.diag([1.,-1.,-1.]);t[:3,3]=[.35,-.25,.225]
        q=self.model.ik(t,np.zeros(6),retries=True)
        points=self.model.cartesian(t[:3,3],np.array([.35,-.25,.025]),t[:3,:3],q,5,samples=21)
        for _,q in points:
            self.assertTrue(np.all(q>=self.model.lower) and np.all(q<=self.model.upper))
            np.testing.assert_allclose(self.model.fk(q)[:3,:3],t[:3,:3],atol=.001)
        np.testing.assert_allclose(self.model.fk(points[-1][1])[:3,3],[.35,-.25,.025],atol=.0001)

    def test_full_task_interpolation_preserves_tool_orientation(self):
        from scipy.interpolate import CubicSpline
        from scipy.spatial.transform import Rotation
        from gp7_task.controller_trajectory import smooth_joint_trajectory
        down = np.diag([1., -1., -1.])
        pre = np.array([.35, -.25, .225]); pick = np.array([.35, -.25, .025])
        lift = np.array([.35, -.25, .42]); above = np.array([.5, .3, .42])
        release = np.array([.5, .3, .227])
        target = np.eye(4); target[:3, :3] = down; target[:3, 3] = pre
        seed = self.model.ik(target, np.zeros(6), retries=True)
        for start, end, duration in ((pre, pick, 5), (pick, lift, 6),
                (lift, above, 8), (above, release, 5), (release, above, 5)):
            path = self.model.cartesian(start, end, down, seed, duration)
            times, q, _, _ = smooth_joint_trajectory(path, self.model.lower, self.model.upper, self.model.speed)
            curve = CubicSpline(times, q, bc_type='clamped')
            # Check the continuous controller interpolation, not just IK knots.
            for time_s in np.linspace(0, duration, 321):
                tcp = self.model.fk(curve(time_s))
                error = Rotation.from_matrix(tcp[:3, :3] @ down.T).magnitude()
                self.assertLess(error, .001)
                s = time_s / duration
                expected = start + (10*s**3-15*s**4+6*s**5)*(end-start)
                self.assertLess(np.linalg.norm(tcp[:3, 3]-expected), .0001)
            seed = q[-1]
