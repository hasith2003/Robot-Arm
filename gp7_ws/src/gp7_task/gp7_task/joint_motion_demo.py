"""Send a conservative controller smoke-test trajectory to Gazebo.

This sends joint targets directly, not the provisional D-H IK model. It proves
the controller path before CAD-measured kinematics are used for pick-and-place.
"""
import rclpy
import math
import time
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatus


JOINTS = ['joint_s', 'joint_l', 'joint_u', 'joint_r', 'joint_b', 'joint_t']
TARGET = [0.25, -0.15, 0.20, 0.0, 0.15, 0.0]


def point(positions, seconds):
    result = JointTrajectoryPoint()
    result.positions = positions
    result.velocities = [0.0] * len(positions)
    result.time_from_start.sec = seconds
    return result


class JointMotionDemo(Node):
    def __init__(self):
        super().__init__('gp7_joint_motion_demo')
        self.client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.actual = None
        self.received_at = None
        self.target_seen = False
        self.observe_target = False
        self.joint_states = self.create_subscription(JointState, '/joint_states', self.record_state, 10)

    def record_state(self, msg):
        states = dict(zip(msg.name, msg.position))
        if not all(joint in states for joint in JOINTS):
            return
        actual = [states[joint] for joint in JOINTS]
        if not all(math.isfinite(value) for value in actual):
            return
        self.actual = actual
        self.received_at = time.monotonic()
        if self.observe_target and max(abs(a - b) for a, b in zip(actual, TARGET)) < 0.04:
            self.target_seen = True

    def wait(self, future, seconds):
        rclpy.spin_until_future_complete(self, future, timeout_sec=seconds)
        if not future.done():
            raise RuntimeError('Timed out waiting for arm_controller; check physics and /clock.')
        return future.result()

    def run(self):
        if not self.client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('arm_controller is unavailable. Launch the simulation and confirm it is active.')
        deadline = time.monotonic() + 5.0
        while self.actual is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.actual is None:
            raise RuntimeError('No measured GP7 joint states received.')
        if self.count_publishers('/joint_states') != 1:
            raise RuntimeError('Expected one joint-state publisher. Close other simulation instances.')
        if max(abs(angle) for angle in self.actual) > 0.15:
            raise RuntimeError(f'Arm is not held at its startup pose: {self.actual}. Restart the simulation.')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        # A deliberately slow, small test movement within every GP7 joint limit.
        # Hold the offset pose for four seconds so motion is easy to observe.
        goal.trajectory.points = [point([0.0] * 6, 2), point(TARGET, 6), point(TARGET, 10), point([0.0] * 6, 14)]
        self.observe_target = True
        future = self.client.send_goal_async(goal)
        handle = self.wait(future, 10.0)
        if not handle or not handle.accepted:
            raise RuntimeError('The trajectory controller rejected the demo goal.')
        self.get_logger().info('Goal accepted; waiting for execution.')
        result_future = handle.get_result_async()
        try:
            response = self.wait(result_future, 30.0)
        except RuntimeError:
            handle.cancel_goal_async()
            raise
        result = response.result
        if response.status != GoalStatus.STATUS_SUCCEEDED or result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f'Trajectory failed: {result.error_string}')
        if not self.target_seen:
            raise RuntimeError('Controller returned success, but measured joints never reached the offset pose.')
        if time.monotonic() - self.received_at > 1.0 or max(abs(angle) for angle in self.actual) > 0.03:
            raise RuntimeError(f'Measured joints did not return to home: {self.actual}')
        self.get_logger().info('Joint motion verified: measured joints reached the offset pose and returned home.')


def main():
    rclpy.init()
    node = JointMotionDemo()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
