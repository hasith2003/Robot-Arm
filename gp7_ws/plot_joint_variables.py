#!/usr/bin/env python3
"""Live measured GP7 joint angles; run alongside the existing simulation."""
import threading
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

NAMES = ('joint_s', 'joint_l', 'joint_u', 'joint_r', 'joint_b', 'joint_t')


class JointPlot(Node):
    def __init__(self):
        super().__init__('gp7_live_joint_plot')
        self.lock = threading.Lock()
        self.samples = deque(maxlen=18000)
        self.first_stamp = None
        self.last_stamp = None
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.receive, 10)

    def receive(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in NAMES):
            return
        angles = np.rad2deg([positions[name] for name in NAMES])
        if not np.all(np.isfinite(angles)):
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            # A restarted simulation has a new time origin.
            if self.last_stamp is not None and stamp < self.last_stamp:
                self.samples.clear()
                self.first_stamp = None
            if self.first_stamp is None:
                self.first_stamp = stamp
            self.last_stamp = stamp
            self.samples.append((stamp - self.first_stamp, *angles))


def main():
    rclpy.init()
    node = JointPlot()
    worker = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    worker.start()
    fig, axes = plt.subplots(3, 2, figsize=(11, 8), sharex=True)
    fig.canvas.manager.set_window_title('GP7 — live measured joint variables')
    fig.suptitle('GP7 measured joint angles — waiting for /joint_states')
    lines = []
    for i, ax in enumerate(axes.flat):
        line, = ax.plot([], [], lw=1.8, color=f'C{i}')
        lines.append(line)
        ax.set_title(f'q{i+1}: {NAMES[i]} ({NAMES[i][-1].upper()} axis)')
        ax.set_ylabel('Angle (degrees)')
        ax.grid(True, alpha=.3)
    for ax in axes[-1]:
        ax.set_xlabel('Simulation time since first received sample (s)')
    fig.tight_layout(rect=(0, 0, 1, .94))

    def update(_):
        with node.lock:
            data = np.array(node.samples)
        if not len(data):
            return lines
        fig.suptitle('GP7 measured joint angles — /joint_states')
        for i, (ax, line) in enumerate(zip(axes.flat, lines)):
            line.set_data(data[:, 0], data[:, i+1])
            ax.relim()
            ax.autoscale_view()
        return lines

    animation = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    try:
        plt.show()
    finally:
        rclpy.shutdown()
        worker.join(timeout=2)
        node.destroy_node()


if __name__ == '__main__':
    main()
