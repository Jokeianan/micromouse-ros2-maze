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

class Point2PointController(Node):
    def __init__(self):
        super().__init__('tb3_point2point')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)

        # 机器人当前位姿
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # 控制参数
        self.Kp_dist = number(self, 'kp_dist', 0.6)  # [调整] 原值 0.3，本次调参提高响应      # 距离比例增益
        self.Kp_ang = number(self, 'kp_ang', 0.8)       # 角度比例增益
        self.max_v = number(self, 'max_v', 0.2)        # 最大线速度 m/s
        self.max_w = number(self, 'max_w', 1.0)        # 最大角速度 rad/s
        self.goal_threshold = number(self, 'goal_threshold', 0.05)  # 保留指导书 5 cm 到达阈值  # 到达判定阈值 m

        # [调整] 将指导书的单个目标扩展为四个连续的点到点任务。
        self.goals = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]
        self.goals = waypoints_parameter(self, self.goals)
        self.goal_index = 0
        self.goal_x, self.goal_y = self.goals[self.goal_index]
        self.support = ExperimentSupport(self, self.goals)

        # 控制循环定时器 20Hz
        self.timer = self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg):
        # [新增] 无效位姿不参与控制。
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if not all(isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
            return
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # 四元数转航向角 yaw
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
        # 连续切换四个点到点任务；中间角点不停车、不原地对齐。
        while self.goal_index < len(self.goals):
            self.goal_x, self.goal_y = self.goals[self.goal_index]
            dist_err = sqrt((self.goal_x - self.x)**2 + (self.goal_y - self.y)**2)
            if dist_err > self.goal_threshold:
                break
            self.get_logger().info(
                f"P 控制目标 {self.goal_index + 1}/{len(self.goals)} 到达，"
                f"误差={dist_err:.4f} m，连续切换")
            self.goal_index += 1
        if self.goal_index >= len(self.goals):
            self.support.complete(goals_reached=self.goal_index, error=dist_err,
                                  goal_tolerance=self.goal_threshold)
            return
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist_err = sqrt(dx**2 + dy**2)
        theta_des = atan2(dy, dx)
        theta_err = theta_des - self.theta
        # 角度归一化到 [-pi, pi]
        theta_err = atan2(sin(theta_err), cos(theta_err))

        twist = Twist()
        twist.linear.x = self.Kp_dist * dist_err
        twist.angular.z = self.Kp_ang * theta_err
        # 速度限幅，防止剧烈震荡；保留参考代码同时前进和转向。
        twist.linear.x = max(-self.max_v, min(self.max_v, twist.linear.x))
        twist.angular.z = max(-self.max_w, min(self.max_w, twist.angular.z))
        self.support.status('running', goal_index=self.goal_index + 1, error=dist_err)
        self.cmd_pub.publish(twist)

def main(args=None):
    run_controller(Point2PointController, args)


if __name__ == '__main__':
    main()
