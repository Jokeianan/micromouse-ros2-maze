"""Start empty-map SLAM and flood-fill navigation without a wall follower."""
from pathlib import Path
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    root=Path(get_package_share_directory('micromouse_practice'))
    defaults=dict(map='migong_smooth',gui='true',rviz='true',timeout='1800',tuning='tuned',
                  output_dir=str(Path.home()/'maze_flood_results'/datetime.now().strftime('%Y%m%d_%H%M%S')))
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root/'launch/practice.launch.py')),
            launch_arguments=dict(mode='flood',**{k:LaunchConfiguration(k) for k in defaults}).items())])
