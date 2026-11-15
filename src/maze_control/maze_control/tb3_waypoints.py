#!/usr/bin/env python3
"""基于 Ch6 指导书原始代码修改；[新增]/[调整] 标出实验增强。"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, pi, sin, cos, isfinite
# [新增] 辅助模块只负责参数、运行记录和停车；控制公式保留在本文件。
from rclpy.qos import qos_profile_sensor_data
from .handout_support import ExperimentSupport, number, waypoints_parameter, run_controller

class WaypointController(Node):
    def __init__(self, node_name='tb3_waypoints'):
        super().__init__(node_name)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.Kp_dist = number(self, 'kp_dist', 0.8)  # [调整] 加密小目标的距离响应
        self.Kp_ang = number(self, 'kp_ang', 2.5)  # [调整] 转角连续前进、小半径转弯
        self.max_v = number(self, 'max_v', 0.2)
        self.max_w = number(self, 'max_w', 1.0)
        self.threshold = number(self, 'threshold', 0.015)  # 小于 4 cm 航点间距

        # 矩形航点列表，可自行修改
        # [用户要求] 共 100 个航点，每边 25 个，边长 1 m、间距 0.04 m。
        # 四个角点各出现一次，最后一个目标为原点。
        self.waypoints = (
            [(i / 25.0, 0.0) for i in range(1, 26)]
            + [(1.0, i / 25.0) for i in range(1, 26)]
            + [(1.0 - i / 25.0, 1.0) for i in range(1, 26)]
            + [(0.0, 1.0 - i / 25.0) for i in range(1, 26)]
        )
        self.waypoints = waypoints_parameter(self, self.waypoints)
        self.wp_index = 0
        self.support = ExperimentSupport(self, self.waypoints)

        self.timer = self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg):
        # [新增] 无效位姿不参与控制。
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if not all(isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
            return
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.theta = atan2(siny_cosp, cosy_cosp)
        self.support.note_odom(msg)  # [新增] 记录有效里程计接收时间

    def control_loop(self):
        # [新增] 收到里程计才开始；正常完成后持续停车。
        if not self.support.ready():
            return
        if self.support.done:
            self.cmd_pub.publish(Twist())
            return
        # 到点就切换并在同一拍计算下个目标的速度，不插入停车拍。
        while self.wp_index < len(self.waypoints):
            gx, gy = self.waypoints[self.wp_index]
            dist_err = sqrt((gx - self.x)**2 + (gy - self.y)**2)
            if dist_err > self.threshold:
                break
            self.get_logger().info(
                f"到达航点 {self.wp_index + 1}/{len(self.waypoints)}: "
                f"({gx:.2f},{gy:.2f})，误差={dist_err:.4f} m，连续切换")
            self.wp_index += 1
        if self.wp_index >= len(self.waypoints):
            self.support.complete(waypoints_reached=self.wp_index, error=dist_err,
                                  goal_tolerance=self.threshold)
            return
        gx, gy = self.waypoints[self.wp_index]
        dx = gx - self.x
        dy = gy - self.y
        dist_err = sqrt(dx**2 + dy**2)
        theta_des = atan2(dy, dx)
        theta_err = theta_des - self.theta
        theta_err = atan2(sin(theta_err), cos(theta_err))

        twist = Twist()
        twist.linear.x = self.Kp_dist * dist_err
        twist.angular.z = self.Kp_ang * theta_err
        twist.linear.x = max(-self.max_v, min(self.max_v, twist.linear.x))
        twist.angular.z = max(-self.max_w, min(self.max_w, twist.angular.z))
        self.support.status('running', waypoint=self.wp_index + 1,
                            total_waypoints=len(self.waypoints), error=dist_err)
        self.cmd_pub.publish(twist)

def main(args=None):
    run_controller(WaypointController, args)


if __name__ == '__main__':
    main()
