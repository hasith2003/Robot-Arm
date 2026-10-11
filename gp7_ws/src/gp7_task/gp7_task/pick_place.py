"""Measured pick/place with URDF IK and an explicitly declared fixed-joint grasp."""
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from action_msgs.msg import GoalStatus
from scipy.spatial.transform import Rotation
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.empty_pb2 import Empty
from gp7_kinematics.urdf_model import URDFModel
from gp7_task.task_state import PickPlaceStateMachine


class PickPlace(Node):
    def __init__(self):
        super().__init__('gp7_pick_place')
        self.arm = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.joints = {}; self.joint_time = 0
        self.sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.lock = threading.Lock(); self.poses = {}; self.grasp = ''; self.grasp_time = 0
        self.gz = GzNode()
        self.gz.subscribe(Pose_V, '/world/gp7_pick_place/pose/info', self.pose_callback)
        self.gz.subscribe(StringMsg, '/box_b/grasp_state', self.grasp_callback)
        self.attach = self.gz.advertise('/box_b/attach', Empty)
        self.detach = self.gz.advertise('/box_b/detach', Empty)
        self.machine = PickPlaceStateMachine()
        self.monitor = False; self.orientation_error_max = 0.; self.failure = None

    def joint_callback(self, msg):
        self.joints = dict(zip(msg.name,msg.position)); self.joint_time = time.monotonic()

    def pose_callback(self, msg):
        with self.lock:
            for p in msg.pose:
                if p.name in ('box_a','box_b','gp7'):
                    self.poses[p.name] = (np.array([p.position.x,p.position.y,p.position.z]),
                        Rotation.from_quat([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]).as_matrix(), time.monotonic())
            if self.monitor and 'box_b' in self.poses:
                angle = np.linalg.norm(Rotation.from_matrix(self.poses['box_b'][1] @ self.box_rotation.T).as_rotvec())
                self.orientation_error_max = max(self.orientation_error_max, angle)
                if angle > .035:
                    self.failure = f'Box orientation changed by {np.rad2deg(angle):.2f} degrees'

    def grasp_callback(self, msg):
        with self.lock:
            self.grasp = msg.data; self.grasp_time = time.monotonic()

    def spin(self, seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            if self.failure: raise RuntimeError(self.failure)

    def wait(self, future, timeout):
        deadline = time.monotonic()+timeout
        while not future.done() and time.monotonic()<deadline:
            self.spin(.05)
        if not future.done(): raise RuntimeError('Controller/service timed out')
        return future.result()

    def object_pose(self, name):
        with self.lock:
            pose = self.poses.get(name)
        if pose is None or time.monotonic()-pose[2]>2:
            raise RuntimeError(f'No fresh Gazebo pose for {name}')
        return pose[0].copy(),pose[1].copy()

    def q(self):
        if time.monotonic()-self.joint_time>1: raise RuntimeError('Joint state feedback is stale')
        return np.array([self.joints[n] for n in self.model.names])

    def switch_grasp(self, attached):
        pub = self.attach if attached else self.detach
        expected = 'attached' if attached else 'detached'
        sent = time.monotonic(); deadline = sent+5
        while time.monotonic()<deadline:
            pub.publish(Empty()); self.spin(.15)
            with self.lock:
                if self.grasp == expected and self.grasp_time > sent: return
        raise RuntimeError(f'Grasp plugin did not confirm {expected}')

    def send(self, client, names, points, timeout=40):
        goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = list(names)
        for stamp,q in points:
            p = JointTrajectoryPoint(); p.positions = list(map(float,q))
            ns = int(round(stamp*1e9)); p.time_from_start.sec=ns//10**9; p.time_from_start.nanosec=ns%10**9
            goal.trajectory.points.append(p)
        # Position-only dense waypoints are linearly interpolated; explicitly
        # stop at the final waypoint for the controller's velocity tolerance.
        goal.trajectory.points[-1].velocities = [0.]*len(names)
        handle = self.wait(client.send_goal_async(goal), 10)
        if not handle.accepted: raise RuntimeError('Trajectory rejected')
        try:
            response = self.wait(handle.get_result_async(), timeout)
        except Exception:
            handle.cancel_goal_async(); raise
        if response.status != GoalStatus.STATUS_SUCCEEDED or response.result.error_code != 0:
            raise RuntimeError(f'Trajectory failed: {response.result.error_string}')

    def fingers(self, q):
        names=['left_finger_joint','right_finger_joint']
        initial=np.array([self.joints[n] for n in names])
        points=[(s*3,initial+(10*s**3-15*s**4+6*s**5)*(np.array([q,q])-initial)) for s in np.linspace(0,1,51)]
        try:
            self.send(self.gripper, names, points)
        except RuntimeError as e:
            raise RuntimeError(f'{e}; measured fingers={[self.joints[n] for n in names]}, TCP={self.model.fk(self.q())}, box={self.object_pose("box_b")}') from e
        self.spin(.2)
        if any(abs(self.joints[n]-q)>.001 for n in ('left_finger_joint','right_finger_joint')):
            raise RuntimeError('Measured gripper width did not match target')

    def advance(self, condition):
        self.machine.advance(condition)
        self.get_logger().info(f'State: {self.machine.state.name}')

    def move(self, points):
        self.send(self.arm,self.model.names,points)
        self.spin(.15)
        if np.max(np.abs(self.q()-points[-1][1]))>.03: raise RuntimeError('Measured arm goal missed')

    def run(self):
        if not self.arm.wait_for_server(timeout_sec=15) or not self.gripper.wait_for_server(timeout_sec=10):
            raise RuntimeError('Arm/gripper controllers unavailable')
        parameters = AsyncParameterClient(self, '/robot_state_publisher')
        if not parameters.wait_for_services(timeout_sec=10): raise RuntimeError('Robot description unavailable')
        xml = self.wait(parameters.get_parameters(['robot_description']),5).values[0].string_value
        self.model = URDFModel(xml)
        self.spin(1)
        if self.count_publishers('/joint_states')!=1: raise RuntimeError('Multiple simulations share this ROS domain')
        self.switch_grasp(False)
        self.spin(.3)
        self.pick,self.box_rotation = self.object_pose('box_b')
        a,_ = self.object_pose('box_a')
        base,_ = self.object_pose('gp7')
        if np.linalg.norm(base)>.001: raise RuntimeError('Robot model must be spawned at world origin')
        if abs(self.pick[2]-.025)>.003: raise RuntimeError('Box B must initially rest on the ground')
        self.place = a + np.array([0,0,.125])  # A centre + half-height + half B.
        down = np.diag([1.,-1.,-1.])
        pre = self.pick + [0,0,.20]
        lift = self.pick.copy(); lift[2]=.42
        above = self.place.copy(); above[2]=.42
        release = self.place + [0,0,.002]
        t=np.eye(4); t[:3,:3]=down; t[:3,3]=pre
        home=self.q()
        if np.max(abs(home))>.03: raise RuntimeError('Arm must start at home for the validated approach')
        # Use the exact home seed to select a reproducible wrist branch rather
        # than amplifying simulation roundoff near its initial singularity.
        pre_q=self.model.ik(t,np.zeros(6),retries=True)
        self.get_logger().info(f'Planning pick={self.pick}, place={self.place}, pregrasp joints={pre_q}')
        # Plan every Cartesian segment before touching the object.
        segments=[]; seed=pre_q
        for start,end,seconds in [(pre,self.pick,5),(self.pick,lift,6),(lift,above,8),(above,release,5),(release,above,5)]:
            path=self.model.cartesian(start,end,down,seed,seconds)
            segments.append(path); seed=path[-1][1]
        self.get_logger().info('URDF IK preflight passed: all Cartesian waypoints keep the tool pointing down.')
        self.advance('motion_complete')
        self.fingers(0.)
        # Quintic joint approach is used only while the box is on the ground.
        approach=[(s*7,home+(10*s**3-15*s**4+6*s**5)*(pre_q-home)) for s in np.linspace(0,1,81)]
        self.move(approach); self.advance('motion_complete')
        self.move(segments[0]); self.advance('motion_complete')
        # 0.5 mm clearance on each face prevents the simplified contact solver
        # fighting the grasp constraint. The fixed-joint grasp is explicit.
        self.fingers(.0145); self.advance('gripper_closed')
        box,_=self.object_pose('box_b'); tcp=self.model.fk(self.q())
        if np.linalg.norm(tcp[:3,3]-box)>.002: raise RuntimeError('Grasp rejected: cube is not centred between the jaws')
        if np.linalg.norm(Rotation.from_matrix(tcp[:3,:3] @ down.T).as_rotvec())>.02:
            raise RuntimeError('Grasp rejected: tool orientation is incorrect')
        self.monitor=True
        self.switch_grasp(True); self.advance('object_attached')
        self.move(segments[1]); self.advance('motion_complete')
        lifted,_=self.object_pose('box_b')
        if np.linalg.norm(lifted-lift)>.005: raise RuntimeError('Lift verification failed: object did not follow tool')
        self.move(segments[2]); self.advance('motion_complete')
        self.move(segments[3]); self.advance('motion_complete')
        self.switch_grasp(False); self.fingers(0.); self.advance('gripper_open')
        self.spin(1)
        placed,_=self.object_pose('box_b')
        self.spin(.5); settled,_=self.object_pose('box_b')
        if np.linalg.norm(settled-self.place)>.004 or np.linalg.norm(settled-placed)>.001:
            raise RuntimeError(f'Placement verification failed: actual={settled}, target={self.place}')
        self.advance('object_stable_on_target')
        self.move(segments[4]); self.advance('motion_complete')
        self.get_logger().info(f'PICK AND PLACE VERIFIED: Box B={settled.round(5)}, maximum orientation error={np.rad2deg(self.orientation_error_max):.3f} degrees.')


def main():
    rclpy.init(); node=PickPlace()
    try:
        node.run()
    finally:
        node.destroy_node(); rclpy.shutdown()
