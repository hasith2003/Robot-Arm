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
