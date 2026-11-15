"""Shared odometry gating, status, reference path and clean stopping."""
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from .control_math import positive


def latched_qos():
    return QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ControllerBase(Node):
    def __init__(self, name):
        super().__init__(name)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/control_status', latched_qos())
        self.reference_pub = self.create_publisher(Path, '/reference_path', latched_qos())
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        self.x = self.y = self.theta = 0.0
        self.odom_time = None
        self.odom_frame = 'odom'
        self.odom_timeout = self.number('odom_timeout', 1.0)
        self.max_v = self.number('max_v', 0.2)
        self.max_w = self.number('max_w', 1.0)
        self.done = False
        self.started = None
        self.reference_sent = False
        self.reference_points = []
        self.last_state = None

    def number(self, name, default):
        return positive(name, self.declare_parameter(name, default).value)

    def odom_callback(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
            return
        self.x, self.y = p.x, p.y
        self.theta = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom_frame = msg.header.frame_id or 'odom'
        self.odom_time = self.get_clock().now().nanoseconds

    def ready(self):
        now = self.get_clock().now().nanoseconds
        if self.odom_time is None or not 0 <= (now - self.odom_time) / 1e9 < self.odom_timeout:
            self.command()
            self.status('waiting_odom')
            return False
        if self.started is None:
            self.started = now
        if not self.reference_sent:
            self.set_reference(self.reference_points)
        return True

    def set_reference(self, points):
        self.reference_points = points
        path = Path()
        path.header.frame_id = self.odom_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.reference_pub.publish(path)
        self.reference_sent = True

    def command(self, v=0.0, w=0.0):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.cmd_pub.publish(msg)

    def status(self, state, **extra):
        elapsed = 0.0 if self.started is None else (
            self.get_clock().now().nanoseconds - self.started) / 1e9
        data = dict(node=self.get_name(), state=state, x=self.x, y=self.y,
                    yaw=self.theta, elapsed=elapsed, **extra)
        self.status_pub.publish(String(data=json.dumps(data)))
        if state != self.last_state:
            self.get_logger().info(json.dumps(data, ensure_ascii=False))
            self.last_state = state

    def finish(self, **extra):
        self.done = True
        self.command()
        self.status('completed', **extra)


def run_node(factory, args=None):
    # Keep the context alive during KeyboardInterrupt to send zero commands.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = factory()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                for _ in range(5):
                    node.command()
                    time.sleep(0.03)
            node.destroy_node()
        rclpy.try_shutdown()
