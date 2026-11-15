"""Launch one experiment with Gazebo, a persistent trajectory and RViz."""
from datetime import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    mode, world = value('mode'), value('world')
    choices = {'observe', 'open_loop', 'point2point', 'waypoints', 'pure_pursuit'}
    if mode not in choices or world not in {'empty', 'standard'}:
        raise ValueError(f'Invalid mode/world: {mode}/{world}')
    shared = get_package_share_directory('maze_control')
    params = {'use_sim_time': value('use_sim_time').lower() == 'true'}
    nodes = []
    if value('sim').lower() == 'true':
        gazebo = get_package_share_directory('turtlebot3_gazebo')
        filename = 'empty_world.launch.py' if world == 'empty' else 'turtlebot3_world.launch.py'
        nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(gazebo, 'launch', filename))))
    nodes.append(Node(package='maze_control', executable='tb3_odom_path',
                      parameters=[params, {'output_dir': value('output_dir')}], output='screen'))
    if mode != 'observe':
        control = dict(params)
        if mode in {'point2point', 'waypoints', 'pure_pursuit'} and value('waypoints_json').strip():
            control['waypoints_json'] = value('waypoints_json')
        if mode == 'pure_pursuit':
            control['ld'] = float(value('ld'))
            control['arrive_threshold'] = float(value('arrive_threshold'))
        nodes.append(Node(package='maze_control', executable='tb3_' + mode,
                          parameters=[control], output='screen'))
    if value('rviz').lower() == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2',
                          arguments=['-d', os.path.join(shared, 'rviz', 'ch6.rviz')],
                          parameters=[params], output='screen'))
    return nodes


def generate_launch_description():
    os.environ.setdefault('TURTLEBOT3_MODEL', 'burger')
    defaults = dict(mode='observe', world='empty', sim='true', rviz='true',
                    use_sim_time='true', ld='0.35',
                    arrive_threshold='0.1',
                    waypoints_json='',
                    output_dir=os.path.expanduser('~/ch6_results/') +
                    datetime.now().strftime('%Y%m%d_%H%M%S'))
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v)
                              for k, v in defaults.items()] + [OpaqueFunction(function=launch_nodes)])
