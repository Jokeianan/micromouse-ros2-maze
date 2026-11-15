"""One command: unknown-map SLAM + existing laser left-wall motion."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    root = Path(get_package_share_directory('micromouse_practice'))
    defaults = dict(map='migong_smooth', gui='true', rviz='true', algorithm='sprint', tuning='tuned',
                    output_dir=str(Path.home()/'maze_online_mapping'), timeout='1200')
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
    return LaunchDescription(args+[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(root/'launch/practice.launch.py')),
        launch_arguments=dict(mode='mapping', **{k:LaunchConfiguration(k) for k in defaults}).items())])
