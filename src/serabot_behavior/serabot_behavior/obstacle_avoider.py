"""
SeraBot obstacle avoider node.

Subscribes to /scan (sensor_msgs/LaserScan) and publishes /cmd_vel
(geometry_msgs/TwistStamped, matching the ros_gz_bridge config). The decision
logic lives in serabot_behavior.avoidance so it can be tested without ROS.
"""

import math
from typing import Optional

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from serabot_behavior.avoidance import (AvoidanceParams, decide, Decision, FORWARD,
                                        is_stale, STOP, STOP_DECISION)


class ObstacleAvoider(Node):
    """Drive forward while the front is clear, otherwise turn toward free space."""

    def __init__(self) -> None:
        """Declare parameters, create the subscriber, publisher and control timer."""
        super().__init__('obstacle_avoider')

        threshold = self.declare_parameter('obstacle_threshold', 0.6).value
        front_deg = self.declare_parameter('front_half_angle_deg', 30.0).value
        side_deg = self.declare_parameter('side_max_angle_deg', 90.0).value
        forward_speed = self.declare_parameter('forward_speed', 0.2).value
        turn_speed = self.declare_parameter('turn_speed', 0.6).value
        self.scan_timeout = float(self.declare_parameter('scan_timeout', 0.5).value)
        control_rate = float(self.declare_parameter('control_rate', 10.0).value)
        self.frame_id = str(self.declare_parameter('frame_id', 'base_footprint').value)

        if self.scan_timeout <= 0.0 or control_rate <= 0.0:
            raise ValueError('scan_timeout and control_rate must be > 0')
        self.params = AvoidanceParams(
            obstacle_threshold=float(threshold),
            forward_speed=float(forward_speed),
            turn_speed=float(turn_speed),
            front_half_angle=math.radians(float(front_deg)),
            side_max_angle=math.radians(float(side_deg)),
        )

        self.last_scan: Optional[LaserScan] = None
        self.last_scan_ns: Optional[int] = None
        self.state = STOP
        self.previous_motion = FORWARD

        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / control_rate, self.on_timer)

        self.get_logger().info(
            f'obstacle_avoider started: threshold={self.params.obstacle_threshold:.2f} m, '
            f'front=+/-{front_deg:.0f} deg, v={self.params.forward_speed:.2f} m/s, '
            f'w={self.params.turn_speed:.2f} rad/s, timeout={self.scan_timeout:.2f} s')

    def on_scan(self, msg: LaserScan) -> None:
        """Store the latest scan and its arrival time (node clock)."""
        self.last_scan = msg
        self.last_scan_ns = self.get_clock().now().nanoseconds

    def on_timer(self) -> None:
        """Run one control step: stop on missing/stale data, otherwise decide."""
        now_ns = self.get_clock().now().nanoseconds
        scan = self.last_scan
        if scan is None or is_stale(self.last_scan_ns, now_ns, self.scan_timeout):
            if self.state != STOP:
                self.get_logger().warning('No fresh /scan data, stopping')
            decision = STOP_DECISION
        else:
            decision = decide(scan.ranges, scan.angle_min, scan.angle_increment,
                              scan.range_min, scan.range_max, self.params,
                              self.previous_motion)
            if decision.state == STOP and self.state != STOP:
                self.get_logger().warning('No valid beam in front sector, stopping')
        self.apply(decision)

    def apply(self, decision: Decision) -> None:
        """Publish the decision and log state changes."""
        if decision.state != self.state:
            self.get_logger().info(f'state: {self.state} -> {decision.state}')
            self.state = decision.state
        if decision.state != STOP:
            self.previous_motion = decision.state
        self.publish(decision.linear, decision.angular)

    def publish(self, linear: float, angular: float) -> None:
        """Publish a TwistStamped velocity command."""
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.frame_id
        cmd.twist.linear.x = linear
        cmd.twist.angular.z = angular
        self.cmd_pub.publish(cmd)


def main(args: Optional[list] = None) -> None:
    """Run the node until shutdown, then try to leave the robot stopped."""
    rclpy.init(args=args)
    node: Optional[ObstacleAvoider] = None
    try:
        node = ObstacleAvoider()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.publish(0.0, 0.0)
            except Exception:  # context may already be shut down after Ctrl+C
                pass
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
