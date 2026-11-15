# 多终端逐步启动

先编译两个功能包，再在各终端逐步启动仿真、模型、建图与控制。每个新终端均需加载环境。



源码修改后在编译终端执行一次；已编译的环境可直接进入下一步。

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2maze_ws
colcon build --symlink-install --packages-select maze_control micromouse_practice
source install/setup.bash
```

**每打开一个新终端或新标签页，都先输入以下环境行**。这些变量不会自动传到其他终端。

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2maze_ws
source install/setup.bash
MAZE_SHARE="$(ros2 pkg prefix --share micromouse_practice)"
MAZE_RUN_DIR="$HOME/maze_manual_demo/wall"
export GAZEBO_MODEL_DATABASE_URI="" LIBGL_ALWAYS_SOFTWARE=1 SVGA_VGPU10=0
```

运行左墙组时，各终端统一使用 `wall` 结果目录；另一次运行洪水组时，各终端统一改为 `flood`，RViz配置也改为 `flood.rviz`。重复实验时换一个新结果目录，避免覆盖上一次记录。

## 1 终端1 启动仿真服务

```bash
gzserver --verbose "$MAZE_SHARE/worlds/migong_smooth.world" \
  -s libgazebo_ros_init.so -s libgazebo_ros_factory.so
```

保持此终端运行。加载的是平整正式迷宫；机器人此时尚未生成。另开检查终端输入 `ros2 topic echo /clock --once`，看到仿真时间后继续。

## 2 终端2 打开Gazebo画面

```bash
gzclient
```

保持运行。确认显示当前迷宫，检查墙面和入口全貌。画面出现不等于模型与传感器已经齐备。

## 3 终端3 启动机器人结构与TF发布

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p use_sim_time:=true \
  -p robot_description:="$(xacro "$MAZE_SHARE/urdf/turtlebot3_burger.urdf")"
```

保持运行。这一步发布机器人描述以及内部连杆变换；`odom` 到底盘的变换在小车生成后由差速插件提供。

## 4 终端4 在入口生成小车并检查传感器

```bash
ros2 run gazebo_ros spawn_entity.py -entity tb3_burger \
  -file "$MAZE_SHARE/urdf/model.sdf" -x -5.52 -y -3.48 -z 0.02
```

生成成功后该命令退出。不要重复生成同名机器人。继续输入：

```bash
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic list
```

预期看到扫描数组和位姿消息。雷达安装坐标系为 `base_scan`，实际以消息头为准。此时没有启动速度控制器，小车应保持静止。

## 5 终端5 启动空地图SLAM

左墙对比组也启动SLAM，但仅旁路记录，不读取栅格决定转向；洪水组需要使用SLAM地图。

先在此终端检查：

```bash
ros2 pkg prefix slam_toolbox
```

**当前项目原先部署了用户目录SLAM运行时**。若上述命令提示找不到包，在这个终端补充以下环境变量，然后再次检查。若系统安装的SLAM已经能正常找到，则跳过这组用户目录配置。

```bash
SLAM_ROOT="$HOME/.local/share/micromouse_slam"
SLAM_PREFIX="$SLAM_ROOT/opt/ros/humble"
export AMENT_PREFIX_PATH="$SLAM_PREFIX${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export LD_LIBRARY_PATH="$SLAM_PREFIX/lib:$SLAM_ROOT/usr/lib:$SLAM_ROOT/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONPATH="$SLAM_PREFIX/local/lib/python3.10/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
ros2 pkg prefix slam_toolbox
```

包被找到后再启动节点：

```bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  --params-file "$MAZE_SHARE/config/online_slam.yaml" \
  -p use_sim_time:=true
```

保持运行，不加载保存地图或历史位姿图。在检查终端输入 `ros2 topic echo /map --once`，确认出现OccupancyGrid。小车尚未行驶，地图先覆盖附近观测范围。

## 6 终端6 保存在线地图

```bash
ros2 run micromouse_practice map_recorder --ros-args \
  -p use_sim_time:=true -p output_dir:="$MAZE_RUN_DIR"
```

保持运行。地图会写入该结果目录的 `mapping` 子目录，包含阶段数据。

## 7 终端7 启动轨迹记录

```bash
ros2 run maze_control tb3_odom_path --ros-args \
  -p use_sim_time:=true -p output_dir:="$MAZE_RUN_DIR"
```

保持运行。此节点记录里程计，发布 `/trajectory`，不向小车发送速度。

## 8 终端8 启动RViz

左墙对比组：

```bash
rviz2 -d "$MAZE_SHARE/rviz/mapping.rviz" --ros-args -p use_sim_time:=true
```

另一次洪水组改用：

```bash
rviz2 -d "$MAZE_SHARE/rviz/flood.rviz" --ros-args -p use_sim_time:=true
```

两种配置只选一个。固定坐标系为 `map`。此时可以看见模型、激光和局部地图；参考路径及局部预测轨迹要等洪水控制器启动后出现。

## 9 终端9 最后启动所选控制器

**左墙组运行以下命令**。这里明确选择 `flow` 控制器，加载平整地图的入口和出口元数据；地图仅用于就绪检查。

```bash
ros2 run micromouse_practice wall_follow --ros-args \
  --params-file "$MAZE_SHARE/config/control_tuning.yaml" \
  -p use_sim_time:=true -p output_dir:="$MAZE_RUN_DIR" \
  -p map:=migong_smooth -p algorithm:=flow -p mode:=wall_follow \
  -p require_map:=true -p timeout:=1200.0
```

**洪水组在另一次干净实验中运行以下命令**。目标偏移为终点减起点，分别是10.97m、6.96m。不能与左墙节点同时运行。

```bash
ros2 run micromouse_practice flood_navigation --ros-args \
  --params-file "$MAZE_SHARE/config/control_tuning.yaml" \
  -p use_sim_time:=true -p output_dir:="$MAZE_RUN_DIR" \
  -p goal_dx:=10.97 -p goal_dy:=6.96 -p max_v:=0.5 -p timeout:=1800.0
```

仅看到“节点启动”还不够，应看到小车从入场到运行的实际状态变化。到达后记录 `completed`、位置以及 `v=0、w=0`。

## 10 检查终端展示通信与状态

以下命令依次输入；持续输出的命令按Ctrl+C退出后再输下一条：

```bash
ros2 topic info /cmd_vel --verbose
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic echo /control_status --once
ros2 param get /tb3_wall_follow use_sim_time
```

洪水组最后一条的节点名换成 `/tb3_flood_navigation`。预期 `/cmd_vel` 只有一个发布者；先前的控制器或遥控节点若仍在发布，应先结束它，再继续演示。

## 11 收尾与两算法切换

先在控制器终端按Ctrl+C并确认停车，再结束记录器、SLAM、RViz、状态发布器和Gazebo。逐节点启动时需要逐终端收尾，不能依赖一个launch进程自动关闭全部节点。

运行另一组时，从新的结果目录重新进行步骤1至9，重新生成起点小车并从空地图开始。只替换控制器而沿用旧车位和旧地图，不构成同条件的两算法演示。

