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

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('tb3_pure_pursuit')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.waypoints = [(1.0,0.0), (1.0,1.0), (0.0,1.0), (0.0,0.0)]
        self.waypoints = waypoints_parameter(self, self.waypoints)
        self.wp_index = 0
        self.Ld = number(self, 'ld', 0.35)          # 前瞻距离 m
        self.max_v = number(self, 'max_v', 0.2)        # 恒速前进
        self.max_w = number(self, 'max_w', 1.0)
        self.arrive_threshold = number(self, 'arrive_threshold', 0.1)  # 参考代码 10 cm
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
        # 参考代码的连续前进方式；切点也不额外停车。
        while self.wp_index < len(self.waypoints):
            gx, gy = self.waypoints[self.wp_index]
            dist = sqrt((gx - self.x)**2 + (gy - self.y)**2)
            if dist >= self.arrive_threshold:
                break
            self.get_logger().info(f"到达航点 {self.wp_index + 1}，连续切换")
            self.wp_index += 1
        if self.wp_index >= len(self.waypoints):
            self.support.complete(corners_reached=self.wp_index, error=dist,
                                  goal_tolerance=self.arrive_threshold)
            return
        gx, gy = self.waypoints[self.wp_index]
        dx = gx - self.x
        dy = gy - self.y
        dist = sqrt(dx**2 + dy**2)
        # 前瞻角：当前航点相对机器人航向的夹角
        alpha = atan2(dy, dx) - self.theta
        alpha = atan2(sin(alpha), cos(alpha))
        twist = Twist()
        twist.linear.x = self.max_v
        # 纯追踪曲率公式 omega = 2*v*sin(alpha)/Ld
        omega = 2.0 * twist.linear.x * sin(alpha) / self.Ld
        twist.angular.z = max(-self.max_w, min(self.max_w, omega))
        self.support.status('running', waypoint=self.wp_index + 1, error=dist)
        self.cmd_pub.publish(twist)

def main(args=None):
    run_controller(PurePursuitController, args)


if __name__ == '__main__':
    main()
