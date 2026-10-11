"""Open the custom mesh gripper, then set a 50 mm clear jaw width."""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from action_msgs.msg import GoalStatus


class GripperDemo(Node):
    def __init__(self):
        super().__init__('gp7_gripper_demo')
        self.client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')

    def wait(self, future, timeout):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError('Gripper controller timed out; check physics and /clock.')
        return future.result()

    def move(self, opening, seconds):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['left_finger_joint', 'right_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = [opening, opening]
        point.velocities = [0.0, 0.0]
        point.time_from_start.sec = seconds
        goal.trajectory.points = [point]
        future = self.client.send_goal_async(goal)
        handle = self.wait(future, 10.0)
        if not handle or not handle.accepted:
            raise RuntimeError('Gripper goal was rejected.')
        result_future = handle.get_result_async()
        try:
            response = self.wait(result_future, seconds + 10.0)
        except RuntimeError:
            handle.cancel_goal_async()
            raise
        if response.status != GoalStatus.STATUS_SUCCEEDED or response.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError('Gripper motion failed.')

    def run(self):
        if not self.client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('gripper_controller is unavailable.')
        self.move(0.0, 1)
        self.move(0.015, 2)
        self.get_logger().info('Gripper motion completed: 80 mm open to 50 mm clear jaw width. This tests the jaws; no object is grasped.')


def main():
    rclpy.init()
    node = GripperDemo()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
