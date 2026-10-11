"""FK, Jacobian and bounded numerical IK derived from the expanded live URDF."""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares


class URDFModel:
    def __init__(self, xml, tip='grasp_tcp'):
        root = ET.fromstring(xml)
        by_child = {j.find('child').attrib['link']: j for j in root.findall('joint')}
        chain = []
        while tip in by_child:
            j = by_child[tip]
            chain.append(j)
            tip = j.find('parent').attrib['link']
        self.chain = []
        self.names, self.lower, self.upper, self.speed = [], [], [], []
        for j in reversed(chain):
            origin = j.find('origin')
            xyz = np.fromstring(origin.attrib.get('xyz', '0 0 0') if origin is not None else '0 0 0', sep=' ')
            rpy = np.fromstring(origin.attrib.get('rpy', '0 0 0') if origin is not None else '0 0 0', sep=' ')
            t = np.eye(4); t[:3, 3] = xyz; t[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
            axis = j.find('axis')
            a = np.fromstring(axis.attrib.get('xyz', '1 0 0') if axis is not None else '1 0 0', sep=' ')
            moving = j.attrib['type'] != 'fixed'
            if moving:
                assert j.attrib['type'] == 'revolute'
                lim = j.find('limit').attrib
                self.names.append(j.attrib['name']); self.lower.append(float(lim['lower']))
                self.upper.append(float(lim['upper'])); self.speed.append(float(lim['velocity']))
            self.chain.append((t, a, moving))
        self.lower, self.upper, self.speed = map(np.array, (self.lower, self.upper, self.speed))

    def frames(self, q):
        t = np.eye(4); axes, origins = [], []; i = 0
        for origin, axis, moving in self.chain:
            t = t @ origin
            if moving:
                axes.append(t[:3, :3] @ axis); origins.append(t[:3, 3].copy())
                r = np.eye(4); r[:3, :3] = Rotation.from_rotvec(axis * q[i]).as_matrix()
                t = t @ r; i += 1
        return t, axes, origins

    def fk(self, q):
        return self.frames(q)[0]

    def jacobian(self, q):
        t, axes, origins = self.frames(q)
        return np.array([np.r_[np.cross(a, t[:3, 3]-o), a] for a,o in zip(axes,origins)]).T

    def ik(self, target, seed=None, retries=False):
        def residual(q):
            t = self.fk(q)
            return np.r_[t[:3,3]-target[:3,3], .3 * Rotation.from_matrix(target[:3,:3] @ t[:3,:3].T).as_rotvec()]
        seeds = [np.zeros(6) if seed is None else np.array(seed)]
        if retries:
            rng = np.random.default_rng(7)
            seeds += [rng.uniform(self.lower, self.upper) for _ in range(20)]
        for start in seeds:
            result = least_squares(residual, np.clip(start, self.lower+1e-8, self.upper-1e-8), bounds=(self.lower,self.upper), max_nfev=250, ftol=1e-10, xtol=1e-10, gtol=1e-10)
            err = residual(result.x)
            if np.linalg.norm(err[:3]) < 0.0001 and np.linalg.norm(err[3:]) < .0003:
                return result.x
        raise RuntimeError(f'URDF IK failed at {target[:3,3]}: residual {err}')

    def cartesian(self, start, finish, rotation, seed, seconds, samples=81):
        points = []
        for s in np.linspace(0,1,samples):
            blend = 10*s**3 - 15*s**4 + 6*s**5
            t = np.eye(4); t[:3,:3] = rotation; t[:3,3] = start + blend*(finish-start)
            seed = self.ik(t, seed)
            points.append((s*seconds, seed.copy()))
        rate = np.abs(np.diff([q for _,q in points],axis=0)) / (seconds/(samples-1))
        if np.any(rate > self.speed):
            raise RuntimeError('Planned Cartesian trajectory exceeds URDF joint speed limits')
        return points
