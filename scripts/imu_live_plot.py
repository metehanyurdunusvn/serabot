#!/usr/bin/env python3
"""
SeraBot IMU live plot.

Subscribes to /imu (sensor_msgs/msg/Imu) and draws the last few seconds of
angular velocity, linear acceleration and orientation (roll/pitch/yaw) live.

Usage (simulation must be running, in a sourced terminal):
    python3 scripts/imu_live_plot.py
    python3 scripts/imu_live_plot.py --window 30
    python3 scripts/imu_live_plot.py --snapshot imu.png --duration 20   # save a PNG, no window
"""

import argparse
from collections import deque
import math
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

AXES = ('x', 'y', 'z')
COLORS = ('#1f77b4', '#ff7f0e', '#2ca02c')


def quaternion_to_rpy(q):
    """Return roll, pitch, yaw in degrees from a geometry_msgs Quaternion."""
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    sinp = max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return tuple(math.degrees(a) for a in (roll, pitch, yaw))


class ImuBuffer(Node):
    """Keep a rolling window of IMU samples, timestamped with the message header."""

    def __init__(self, topic: str, window_s: float) -> None:
        super().__init__('imu_live_plot')
        self.window_s = window_s
        self.lock = threading.Lock()
        self.t = deque()
        self.gyro = [deque() for _ in AXES]
        self.accel = [deque() for _ in AXES]
        self.rpy = [deque() for _ in AXES]
        self.count = 0
        self.create_subscription(Imu, topic, self.on_imu, qos_profile_sensor_data)
        self.get_logger().info(f'Listening to {topic} (window {window_s:.0f} s)')

    def on_imu(self, msg: Imu) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        gyro = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        accel = (msg.linear_acceleration.x, msg.linear_acceleration.y,
                 msg.linear_acceleration.z)
        rpy = quaternion_to_rpy(msg.orientation)
        with self.lock:
            if self.t and stamp < self.t[-1]:      # simulation was reset: start over
                self.clear()
            self.t.append(stamp)
            for i in range(3):
                self.gyro[i].append(gyro[i])
                self.accel[i].append(accel[i])
                self.rpy[i].append(rpy[i])
            while self.t and stamp - self.t[0] > self.window_s:
                self.t.popleft()
                for series in (*self.gyro, *self.accel, *self.rpy):
                    series.popleft()
            self.count += 1

    def clear(self) -> None:
        for series in (self.t, *self.gyro, *self.accel, *self.rpy):
            series.clear()

    def snapshot(self):
        with self.lock:
            return (list(self.t), [list(s) for s in self.gyro],
                    [list(s) for s in self.accel], [list(s) for s in self.rpy], self.count)


def build_figure(plt, window_s):
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(11, 8))
    fig.canvas.manager.set_window_title('SeraBot – IMU live plot')
    titles = ('Angular velocity (rad/s)', 'Linear acceleration (m/s²)',
              'Orientation (deg)')
    labels = (('ω x', 'ω y', 'ω z'), ('a x', 'a y', 'a z'), ('roll', 'pitch', 'yaw'))
    lines = []
    for ax, title, names in zip(axes, titles, labels):
        ax.set_ylabel(title)
        ax.grid(True, alpha=0.3)
        lines.append([ax.plot([], [], color=c, lw=1.4, label=n)[0]
                      for c, n in zip(COLORS, names)])
        ax.legend(loc='upper left', ncol=3, fontsize=9)
    axes[-1].set_xlabel(f'Simulation time (s) – last {window_s:.0f} s')
    status = fig.suptitle('Waiting for /imu ...', fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    return fig, axes, lines, status


def update_figure(node, axes, lines, status, window_s):
    t, gyro, accel, rpy, count = node.snapshot()
    if not t:
        return
    for row, data in enumerate((gyro, accel, rpy)):
        for line, values in zip(lines[row], data):
            line.set_data(t, values)
        lo = min(min(v) for v in data)
        hi = max(max(v) for v in data)
        pad = max(0.1, 0.15 * (hi - lo))
        axes[row].set_ylim(lo - pad, hi + pad)
    axes[-1].set_xlim(max(t[0], t[-1] - window_s), t[-1] + 0.05)
    status.set_text(
        f'/imu  ·  {count} msgs  ·  ω z = {gyro[2][-1]:+.3f} rad/s  ·  '
        f'a x = {accel[0][-1]:+.3f}  a z = {accel[2][-1]:+.2f} m/s²  ·  '
        f'yaw = {rpy[2][-1]:+.1f}°')


def main() -> None:
    parser = argparse.ArgumentParser(description='Live plot of SeraBot IMU data')
    parser.add_argument('--topic', default='imu', help='IMU topic (default: imu)')
    parser.add_argument('--window', type=float, default=20.0,
                        help='seconds of data shown (default: 20)')
    parser.add_argument('--snapshot', metavar='FILE',
                        help='save a PNG after --duration seconds instead of opening a window')
    parser.add_argument('--duration', type=float, default=15.0,
                        help='seconds to record before saving --snapshot (default: 15)')
    args, ros_args = parser.parse_known_args()

    if args.snapshot:
        import matplotlib
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    rclpy.init(args=ros_args)
    node = ImuBuffer(args.topic, args.window)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    fig, axes, lines, status = build_figure(plt, args.window)
    try:
        if args.snapshot:
            import time
            time.sleep(args.duration)
            update_figure(node, axes, lines, status, args.window)
            fig.savefig(args.snapshot, dpi=120)
            print(f'Saved {args.snapshot} ({node.count} IMU messages)')
        else:
            animation = FuncAnimation(  # noqa: F841 (must stay referenced)
                fig, lambda _: update_figure(node, axes, lines, status, args.window),
                interval=50, cache_frame_data=False)
            plt.show()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
