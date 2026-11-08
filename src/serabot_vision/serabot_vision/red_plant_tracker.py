"""ROS 2 camera+LiDAR navigator. Never run alongside obstacle_avoider."""
import math
from typing import Optional

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

from serabot_behavior.avoidance import STOP, is_stale
from .red_detection import RedTarget, find_red_target
from .tracking import Pose, VisionNavigator, TrackingParams


def yaw_from_quaternion(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class RedPlantTracker(Node):
    def __init__(self) -> None:
        super().__init__('red_plant_tracker')
        self.bridge = CvBridge()
        # The Gazebo greenhouse provides a clear central aisle from
        # its default spawn; keep the robot there before visual approach.
        # Pass -p aisle_cruise_distance:=0 to use generic reactive mode.
        self.declare_parameter('aisle_cruise_distance', 3.55)
        aisle = float(self.get_parameter('aisle_cruise_distance').value)
        self.navigator = VisionNavigator(
            tracking=TrackingParams(aisle_cruise_distance=max(0.0, aisle)))
        self.scan_timeout = 0.5
        self.camera_timeout = 0.8
        self.odom_timeout = 0.5
        self.last_scan: Optional[LaserScan] = None
        self.last_scan_ns: Optional[int] = None
        self.last_image_ns: Optional[int] = None
        self.last_odom_ns: Optional[int] = None
        self.target: Optional[RedTarget] = None
        self.pose: Optional[Pose] = None
        self.state = STOP
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.create_subscription(Image, 'camera/image_raw', self.on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self.on_odom,
                                 qos_profile_sensor_data)
        self.create_timer(0.1, self.on_timer)
        self.get_logger().info(
            'Vision navigator active; launch simulation with behavior:=false')

    def on_image(self, msg: Image) -> None:
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.target = find_red_target(bgr)
            self.last_image_ns = self.get_clock().now().nanoseconds
        except (CvBridgeError, ValueError) as error:
            self.target = None
            self.last_image_ns = None
            self.get_logger().warning(f'Cannot decode camera image: {error}')

    def on_scan(self, msg: LaserScan) -> None:
        self.last_scan = msg
        self.last_scan_ns = self.get_clock().now().nanoseconds

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.pose = (float(p.x), float(p.y),
                     yaw_from_quaternion(msg.pose.pose.orientation))
        self.last_odom_ns = self.get_clock().now().nanoseconds

    def publish(self, linear: float, angular: float) -> None:
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_footprint'
        cmd.twist.linear.x = linear
        cmd.twist.angular.z = angular
        self.pub.publish(cmd)

    def on_timer(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if (self.last_scan is None or self.pose is None or
                is_stale(self.last_scan_ns, now_ns, self.scan_timeout) or
                is_stale(self.last_image_ns, now_ns, self.camera_timeout) or
                is_stale(self.last_odom_ns, now_ns, self.odom_timeout)):
            self.publish(0.0, 0.0)
            if self.state != STOP:
                self.get_logger().warning('Stale sensor data; stopping')
            self.state = STOP
            return
        scan = self.last_scan
        decision = self.navigator.step(
            scan.ranges, scan.angle_min, scan.angle_increment,
            scan.range_min, scan.range_max, self.target, self.pose)
        if decision.state != self.state:
            self.get_logger().info(f'state: {self.state} -> {decision.state}')
            self.state = decision.state
        self.publish(decision.linear, decision.angular)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RedPlantTracker()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.publish(0.0, 0.0)
            except Exception:
                pass
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
