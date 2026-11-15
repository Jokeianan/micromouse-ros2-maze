"""Leave a persistent RViz trail and save odometry, reference and plots."""
import csv
import json
import math
import os
from pathlib import Path as FilePath

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Empty

from .controller_base import latched_qos
from .control_math import positive, wrap


class OdomPath(Node):
    def __init__(self):
        super().__init__('odom_path')
        topic = self.declare_parameter('topic', '/trajectory').value
        self.step = positive('step', self.declare_parameter('step', 0.003).value)
        self.max_pts = int(self.declare_parameter('max_points', 20000).value)
        if self.max_pts < 2:
            raise ValueError('max_points must be >= 2')
        output = self.declare_parameter('output_dir', '~/ch6_results/manual').value
        self.output = FilePath(os.path.expanduser(output))
        self.output.mkdir(parents=True, exist_ok=True)
        self.file = (self.output / 'trajectory.csv').open('w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['stamp_sec', 'x', 'y', 'yaw', 'linear_x', 'angular_z', 'segment'])
        self.path = Path()
        self.last = None
        self.last_stamp = None
        self.segment = 0
        self.reference = []
        self.completed = False
        self.pub = self.create_publisher(Path, topic, latched_qos())
        self.create_subscription(Odometry, '/odom', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(Path, '/reference_path', self.reference_cb, latched_qos())
        self.create_subscription(String, '/control_status', self.status_cb, latched_qos())
        self.create_service(Empty, '/clear_trajectory', self.clear_cb)
        self.create_timer(0.2, self.publish_path)
        self.get_logger().info(f'Trail: {topic}; output: {self.output}')

    def clear_cb(self, request, response):
        self.path = Path()
        self.last = None
        self.segment += 1
        self.pub.publish(self.path)
        return response

    def odom_cb(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        frame = msg.header.frame_id or 'odom'
        if (self.path.header.frame_id and self.path.header.frame_id != frame) or (
                self.last_stamp is not None and stamp < self.last_stamp):
            self.clear_cb(None, None)
        self.path.header.frame_id = frame
        self.last_stamp = stamp
        if self.last is not None and math.dist((x, y), self.last[:2]) < self.step and (
                abs(wrap(yaw - self.last[2])) < 0.03):
            return
        self.last = (x, y, yaw)
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = frame
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        if len(self.path.poses) > self.max_pts:
            del self.path.poses[:len(self.path.poses) - self.max_pts]
        self.writer.writerow([stamp, x, y, yaw, msg.twist.twist.linear.x,
                              msg.twist.twist.angular.z, self.segment])

    def reference_cb(self, msg):
        self.reference = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        (self.output / 'reference.json').write_text(json.dumps(self.reference))

    def status_cb(self, msg):
        data = json.loads(msg.data)
        if data['state'] in ('completed', 'aborted') and not self.completed:
            self.completed = True
            (self.output / 'controller_result.json').write_text(
                json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
            self.save_plot()

    def publish_path(self):
        self.file.flush()
        if self.path.header.frame_id:
            self.path.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(self.path)

    def save_plot(self):
        self.file.flush()
        if not self.path.poses:
            return
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 7))
        if self.reference:
            rx, ry = zip(*self.reference)
            ax.plot(rx, ry, 'o--', color='#e69025', label='Reference / waypoints')
        xs = [p.pose.position.x for p in self.path.poses]
        ys = [p.pose.position.y for p in self.path.poses]
        ax.plot(xs, ys, color='#008fbd', linewidth=2, label='Measured /odom trajectory')
        ax.scatter(xs[0], ys[0], c='green', marker='s', label='Start', zorder=5)
        ax.scatter(xs[-1], ys[-1], c='red', marker='x', label='End', zorder=5)
        ax.set(xlabel='x / m', ylabel='y / m', title='TurtleBot3 measured trajectory')
        ax.axis('equal')
        ax.grid(True, alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(self.output / 'trajectory.png', dpi=180)
        plt.close(fig)

    def destroy_node(self):
        self.save_plot()
        self.file.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdomPath()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
