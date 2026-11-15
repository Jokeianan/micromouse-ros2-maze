"""Find system SLAM Toolbox or the optional user-owned Humble runtime."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_prefix, PackageNotFoundError


def prepare_slam_runtime():
    try:
        get_package_prefix('slam_toolbox')
        return {}
    except PackageNotFoundError:
        root = Path.home()/'.local/share/micromouse_slam'
        prefix = root/'opt/ros/humble'
        if not (prefix/'lib/slam_toolbox/async_slam_toolbox_node').is_file():
            raise RuntimeError('SLAM Toolbox is missing. Install ros-humble-slam-toolbox or run tools/install_slam_user.py on Ubuntu.')
        def prepend(name, paths):
            values = [str(p) for p in paths]
            if os.environ.get(name): values.append(os.environ[name])
            return ':'.join(values)
        # Launch resolves the Node executable through ament in this process.
        os.environ['AMENT_PREFIX_PATH'] = prepend('AMENT_PREFIX_PATH', [prefix])
        return {
            'AMENT_PREFIX_PATH': os.environ['AMENT_PREFIX_PATH'],
            'LD_LIBRARY_PATH': prepend('LD_LIBRARY_PATH', [prefix/'lib', root/'usr/lib', root/'usr/lib/x86_64-linux-gnu']),
            'PYTHONPATH': prepend('PYTHONPATH', [prefix/'local/lib/python3.10/dist-packages']),
        }
