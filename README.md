# 地老鼠走迷宫：ROS 2 感知与控制

基于 TurtleBot3 Burger 的 Gazebo 迷宫实验。基础任务使用激光左墙跟随；拓展任务从空地图启动 SLAM Toolbox，再使用洪水填充距离场选择探索目标、规划已观测安全区域内的路径，并执行局部避障跟踪。

## 环境

- Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11。
- TurtleBot3 Burger 差速模型，ROS 2 Python 功能包，使用仿真时间。
- `maze_control`：运动控制、里程计轨迹记录与基础运动练习。
- `micromouse_practice`：地图、机器人模型、雷达处理、左墙跟随、在线建图与洪水填充。

这是源码仓库；实验报告、讲解图片、演示视频和历史运行数据暂不包含在本次上传中。

## 目录

```text
src/
  maze_control/             # 运动控制与轨迹记录
  micromouse_practice/
    config/                # 地图元数据、控制参数与 SLAM 参数
    launch/                # 保留已有 launch 入口
    micromouse_practice/    # 控制、建图与规划 Python 模块
    rviz/                  # RViz 配置
    urdf/                  # URDF、SDF 与第三方版权声明
    worlds/                # 正式迷宫与准备地图
docs/manual_start.md       # 多终端逐步启动说明
tools/                     # 规划与地图导出回归检查、可选 SLAM 依赖工具
source_worlds/             # 原始地图，供几何核验使用
```

## 安装与编译

先配置好 ROS 2 Humble 的软件源和基础环境。在 Ubuntu 终端安装依赖：

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs \
  ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-description \
  ros-humble-robot-state-publisher ros-humble-xacro ros-humble-rviz2 \
  ros-humble-slam-toolbox python3-colcon-common-extensions \
  python3-numpy python3-scipy python3-matplotlib python3-yaml
```

将仓库直接作为工作空间：

```bash
git clone https://github.com/Jokeianan/micromouse-ros2-maze.git ~/ros2maze_ws
cd ~/ros2maze_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select maze_control micromouse_practice
source install/setup.bash
```

仓库为私有仓库，克隆时需要 GitHub 账号拥有访问权限。

## 终端逐步运行

按 [多终端启动说明](docs/manual_start.md) 依次启动 Gazebo、机器人状态发布器、生成小车、SLAM、记录器和 RViz，最后选择一个控制器。切换算法时结束上一套实验，并重新从起点和空地图启动。

左墙控制器：

```bash
ros2 run micromouse_practice wall_follow --ros-args \
  --params-file "$MAZE_SHARE/config/control_tuning.yaml" \
  -p use_sim_time:=true -p output_dir:="$MAZE_RUN_DIR" \
  -p map:=migong_smooth -p algorithm:=flow -p mode:=wall_follow \
  -p require_map:=true -p timeout:=1200.0
```

洪水填充控制器：

```bash
ros2 run micromouse_practice flood_navigation --ros-args \
  --params-file "$MAZE_SHARE/config/control_tuning.yaml" \
  -p use_sim_time:=true -p output_dir:="$MAZE_RUN_DIR" \
  -p goal_dx:=10.97 -p goal_dy:=6.96 -p max_v:=0.5 -p timeout:=1800.0
```

以上命令依赖启动说明中的环境变量与前置节点，不能单独替代完整仿真环境。运行期间 `/cmd_vel` 应只有一个发布者。

## 算法与边界

### 基础任务：左墙跟随

正常通道控制按前方危险、左前方过近、左墙丢失、正常沿墙的优先级选择动作，每周期重新判断。前方避障触发阈值为 0.32 m，解除阈值为 0.38 m；期望墙距为 0.40 m。45°与90°激光窄束用于估计墙面方向和预瞄墙距。

里程计用于入口与出口阶段。启用 `require_map` 时，订阅地图仅检查 SLAM 就绪，不读取栅格决定转弯。已知出口坐标用于出口清出与完成判定，不提供迷宫内部路线。

### 拓展任务：在线建图与洪水填充

SLAM 地图分辨率为 0.04 m，规划保守聚合到 0.08 m，并考虑机器人半径和安全余量。四邻域洪水传播生成等权栅格距离场。未知区域可以参与乐观目标距离估计，但实际执行路径必须通过已观测、经过安全处理的区域；随着地图更新重新选择目标与路径。

局部控制结合扫描时间戳变换、路径跟踪和预测避障，包含对齐、制动、重新观察及到达停车。直道速度上限为 0.5 m/s，实际速度受转弯和净空约束。

本方法不保证未知迷宫中的实际总行程全局最短，也不意味着左墙跟随能在任意带墙岛或环路的迷宫中找到目标。

源码中保留已知地图 A*、基础运动练习和准备地图作为历史扩展。A* 模式读取已知 world 几何；洪水填充模式不使用该已知地图路径。

## 通信与坐标

| 数据 | ROS 2 接口 | 用途 |
| --- | --- | --- |
| 激光 | `/scan`，`sensor_msgs/msg/LaserScan` | 雷达分区、建图与避障 |
| 里程计 | `/odom`，`nav_msgs/msg/Odometry` | 连续运动反馈 |
| 速度指令 | `/cmd_vel`，`geometry_msgs/msg/Twist` | 差速运动控制 |
| 在线地图 | `/map`，`nav_msgs/msg/OccupancyGrid` | SLAM 输出与在线规划 |
| 路径 | `/reference_path`、`/local_trajectory`、`/trajectory` | 规划、预测与已行驶轨迹 |
| 状态 | `/control_status` | 控制状态与记录 |

模型主链为 `map → odom → base_footprint → base_link → base_scan`。SLAM 提供地图修正，差速插件提供连续里程计，机器人状态发布器提供模型内部变换。仿真节点使用 `use_sim_time:=true`。

## 回归检查

在已安装 ROS 2 依赖并编译工作空间后：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/test_flood_planner.py
python3 tools/test_tuned_controls.py
python3 tools/test_map_export.py
python3 tools/test_grid_planner.py
```

这些检查覆盖洪水拓扑、不可达目标、薄墙聚合、未知空间边界、控制保护、TF时刻组合、地图导出方向及历史A*几何约束；不替代 Gazebo 完整通关实验。

## 资料与版权

项目功能包声明 Apache-2.0 许可，第三方模型与依赖见 [THIRD_PARTY.md](THIRD_PARTY.md)。本项目使用 AI 工具辅助代码整理、控制与规划实现和实验材料制作；提交或展示前应自行复跑并核对实现。
