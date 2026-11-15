"""Experiment recording and stopping helpers; control laws stay in handout files."""
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from .controller_base import latched_qos


def number(node, name, default):
    value = float(node.declare_parameter(name, default).value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be positive and finite')
    return value


def waypoints_parameter(node, defaults):
    raw = node.declare_parameter('waypoints_json', '').value
    if not raw:
        return defaults
    values = json.loads(raw)
    if not isinstance(values, list) or not values or len(values) % 2:
        raise ValueError('waypoints_json must be [x1,y1,x2,y2,...]')
    if any(isinstance(v, bool) or not isinstance(v, (int, float))
           or not math.isfinite(v) for v in values):
        raise ValueError('waypoints must contain finite numbers')
    return list(zip(values[::2], values[1::2]))


class ExperimentSupport:
    """Publish evidence/reference paths without replacing the controller algorithm."""

    def __init__(self, node, goals):
        self.node, self.goals = node, goals
        self.timeout = number(node, 'odom_timeout', 1.0)
        self.last_odom = None
        self.frame = 'odom'
        self.start = None
        self.reference_sent = False
        self.last_state = None
        self.done = False
        self.status_pub = node.create_publisher(String, '/control_status', latched_qos())
        self.path_pub = node.create_publisher(Path, '/reference_path', latched_qos())

    def note_odom(self, msg):
        self.last_odom = self.node.get_clock().now().nanoseconds
        self.frame = msg.header.frame_id or 'odom'

    def ready(self):
        now = self.node.get_clock().now().nanoseconds
        if self.last_odom is None or not 0 <= (now - self.last_odom) / 1e9 < self.timeout:
            self.node.cmd_pub.publish(Twist())
            self.status('waiting_odom')
            return False
        if self.start is None:
            self.start = now
        if not self.reference_sent:
            path = Path()
            path.header.frame_id = self.frame
            path.header.stamp = self.node.get_clock().now().to_msg()
            for x, y in [(self.node.x, self.node.y)] + self.goals:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x, pose.pose.position.y = float(x), float(y)
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            self.path_pub.publish(path)
            self.reference_sent = True
        return True

    def status(self, state, **extra):
        node = self.node
        elapsed = 0.0 if self.start is None else (
            node.get_clock().now().nanoseconds - self.start) / 1e9
        data = dict(node=node.get_name(), state=state, x=node.x, y=node.y,
                    yaw=node.theta, elapsed=elapsed, **extra)
        self.status_pub.publish(String(data=json.dumps(data)))
        if state != self.last_state:
            node.get_logger().info(json.dumps(data, ensure_ascii=False))
            self.last_state = state

    def complete(self, **extra):
        self.done = True
        self.node.cmd_pub.publish(Twist())
        self.status('completed', **extra)


def run_controller(factory, args=None):
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
                    node.cmd_pub.publish(Twist())
                    time.sleep(0.03)
            node.destroy_node()
        rclpy.try_shutdown()
