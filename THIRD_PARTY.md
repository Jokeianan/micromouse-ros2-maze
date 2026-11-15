# 第三方资料与模型

## TurtleBot3 / ROBOTIS

机器人描述与Gazebo模型以ROBOTIS的TurtleBot3 Burger为基础。随源码保留：

- `src/micromouse_practice/urdf/TURTLEBOT3_DESCRIPTION_COPYRIGHT`
- `src/micromouse_practice/urdf/TURTLEBOT3_GAZEBO_COPYRIGHT`

上游项目：https://github.com/ROBOTIS-GIT/turtlebot3 及 https://github.com/ROBOTIS-GIT/turtlebot3_simulations 。模型所需网格资源通过安装 `turtlebot3_description` 依赖提供；这些资源未重新打包到本仓库。

## SLAM Toolbox

在线建图使用外部安装的SLAM Toolbox。`online_slam.yaml`参考其Humble配置并结合本项目设置参数。上游：https://github.com/SteveMacenski/slam_toolbox 。SLAM Toolbox本体不包含在本仓库源码中，适用其自身许可证。

## ROS 2 / Gazebo / RViz

ROS 2 Humble、Gazebo Classic、gazebo_ros、RViz、robot_state_publisher、xacro及Python数值库作为环境依赖安装，适用各自项目的许可证。

项目地图与控制、规划代码用于课程实验。根目录许可证遵循现有功能包声明，第三方文件的声明与许可保留原有约定。
